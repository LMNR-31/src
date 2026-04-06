#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;

enum class TakeoffFSM { WAIT_FCU, PUBLISH_TAKEOFF, MONITOR };

class TakeoffNode : public rclcpp::Node
{
public:
  TakeoffNode() : Node("takeoff"), fsm_(TakeoffFSM::WAIT_FCU),
    attempt_start_(rclcpp::Time(0, 0, RCL_ROS_TIME))
  {
    // Parameters
    this->declare_parameter<std::string>("uav_name", "uav1");
    this->declare_parameter<double>("takeoff_altitude", 1.75);
    this->declare_parameter<double>("altitude_threshold", -1.0);  // <0 → use takeoff_altitude - 0.3
    this->declare_parameter<bool>("use_current_xy", true);
    this->declare_parameter<double>("x", 0.0);
    this->declare_parameter<double>("y", 0.0);
    this->declare_parameter<std::string>("frame_id", "map");
    this->declare_parameter<double>("check_after_sec", 10.0);
    this->declare_parameter<int>("max_attempts", 3);
    this->declare_parameter<double>("rate_hz", 10.0);

    uav_name_        = this->get_parameter("uav_name").as_string();
    takeoff_alt_     = this->get_parameter("takeoff_altitude").as_double();
    altitude_thresh_ = this->get_parameter("altitude_threshold").as_double();
    use_current_xy_  = this->get_parameter("use_current_xy").as_bool();
    fallback_x_      = this->get_parameter("x").as_double();
    fallback_y_      = this->get_parameter("y").as_double();
    frame_id_        = this->get_parameter("frame_id").as_string();
    check_after_sec_ = this->get_parameter("check_after_sec").as_double();
    max_attempts_    = this->get_parameter("max_attempts").as_int();
    double rate_hz   = this->get_parameter("rate_hz").as_double();

    if (altitude_thresh_ < 0.0) {
      altitude_thresh_ = std::max(takeoff_alt_ - 0.3, 0.2);
    }

    // Publishers / subscribers
    waypoints_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/waypoints", 10);

    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
      "/" + uav_name_ + "/mavros/state", 10,
      std::bind(&TakeoffNode::stateCallback, this, _1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/" + uav_name_ + "/mavros/local_position/odom", 10,
      std::bind(&TakeoffNode::odomCallback, this, _1));

    auto period_ms = std::chrono::milliseconds(
      static_cast<int>(1000.0 / rate_hz));
    timer_ = this->create_wall_timer(
      period_ms, std::bind(&TakeoffNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(),
      "takeoff node started. uav=%s takeoff_alt=%.2f threshold=%.2f "
      "check_after=%.1fs max_attempts=%d rate=%.1fHz",
      uav_name_.c_str(), takeoff_alt_, altitude_thresh_,
      check_after_sec_, max_attempts_, rate_hz);
  }

private:
  // ── callbacks ────────────────────────────────────────────────────────────

  void stateCallback(const mavros_msgs::msg::State::SharedPtr msg)
  {
    fcu_connected_ = msg->connected;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_z_  = msg->pose.pose.position.z;
    odom_x_     = msg->pose.pose.position.x;
    odom_y_     = msg->pose.pose.position.y;
    odom_received_ = true;
  }

  // ── FSM timer ────────────────────────────────────────────────────────────

  void timerCallback()
  {
    switch (fsm_) {
      case TakeoffFSM::WAIT_FCU:
        if (!fcu_connected_) {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
            "Waiting for FCU connection…");
          return;
        }
        RCLCPP_INFO(this->get_logger(), "FCU connected. Proceeding to publish takeoff waypoint.");
        fsm_ = TakeoffFSM::PUBLISH_TAKEOFF;
        break;  // will transition to PUBLISH_TAKEOFF on next timer tick

      case TakeoffFSM::PUBLISH_TAKEOFF:
        if (use_current_xy_ && !odom_received_) {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Waiting for odometry before publishing takeoff waypoint…");
          return;
        }
        publishTakeoffWaypoint();
        attempt_start_ = this->now();
        fsm_ = TakeoffFSM::MONITOR;
        break;

      case TakeoffFSM::MONITOR:
        monitorAltitude();
        break;
    }
  }

  // ── helpers ──────────────────────────────────────────────────────────────

  void publishTakeoffWaypoint()
  {
    geometry_msgs::msg::PoseArray waypoints;
    waypoints.header.frame_id = frame_id_;
    waypoints.header.stamp    = this->now();

    geometry_msgs::msg::Pose pose;
    if (use_current_xy_) {
      pose.position.x = odom_x_;
      pose.position.y = odom_y_;
    } else {
      pose.position.x = fallback_x_;
      pose.position.y = fallback_y_;
    }
    pose.position.z    = takeoff_alt_;
    pose.orientation.w = 1.0;

    waypoints.poses.push_back(pose);
    waypoints_pub_->publish(waypoints);

    RCLCPP_INFO(this->get_logger(),
      "Takeoff waypoint published to /waypoints (attempt %d/%d): "
      "x=%.2f y=%.2f z=%.2f",
      attempt_count_ + 1, max_attempts_,
      pose.position.x, pose.position.y, pose.position.z);
  }

  void monitorAltitude()
  {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Current altitude z=%.2f m (target threshold=%.2f m, attempt %d/%d)",
      current_z_, altitude_thresh_, attempt_count_ + 1, max_attempts_);

    // Success: altitude threshold reached
    if (current_z_ >= altitude_thresh_) {
      RCLCPP_INFO(this->get_logger(),
        "Takeoff successful: z=%.2f m >= %.2f m. Shutting down.",
        current_z_, altitude_thresh_);
      rclcpp::shutdown();
      return;
    }

    // Check 10-second timeout
    double elapsed = (this->now() - attempt_start_).seconds();
    if (elapsed >= check_after_sec_) {
      attempt_count_++;
      if (attempt_count_ >= max_attempts_) {
        RCLCPP_ERROR(this->get_logger(),
          "Takeoff failed after %d attempt(s): z=%.2f m < %.2f m. Shutting down.",
          max_attempts_, current_z_, altitude_thresh_);
        rclcpp::shutdown();
        return;
      }
      RCLCPP_WARN(this->get_logger(),
        "Altitude not reached after %.1fs (z=%.2f m). Retrying (attempt %d/%d)…",
        check_after_sec_, current_z_, attempt_count_ + 1, max_attempts_);
      fsm_ = TakeoffFSM::PUBLISH_TAKEOFF;
    }
  }

  // ── state ────────────────────────────────────────────────────────────────

  TakeoffFSM fsm_;

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr waypoints_pub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr    state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr    odom_sub_;
  rclcpp::TimerBase::SharedPtr                                timer_;

  bool   fcu_connected_  {false};
  bool   odom_received_  {false};
  double current_z_      {0.0};
  double odom_x_         {0.0};
  double odom_y_         {0.0};

  rclcpp::Time attempt_start_;
  int          attempt_count_ {0};

  // Parameters
  std::string uav_name_;
  double      takeoff_alt_;
  double      altitude_thresh_;
  bool        use_current_xy_;
  double      fallback_x_;
  double      fallback_y_;
  std::string frame_id_;
  double      check_after_sec_;
  int         max_attempts_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TakeoffNode>());
  rclcpp::shutdown();
  return 0;
}
