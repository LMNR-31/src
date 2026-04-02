// pouso.cpp — ROS 2 node for publishing an arbitrary landing point on the map.
//
// Publishes a two-waypoint descent sequence to /waypoints so that
// my_drone_controller navigates the drone to the desired XY position and
// then descends to ground level.
//
// Parameters
//   uav_name        (string, default "uav1")    — UAV namespace prefix
//   x               (double, default  0.0)      — target landing X (ENU, m)
//   y               (double, default  0.0)      — target landing Y (ENU, m)
//   use_current_xy  (bool,   default  true)     — ignore x/y and use live odom XY
//   landing_z       (double, default  0.05)     — final ground altitude (m)
//   frame_id        (string, default "map")     — coordinate frame
//   rate_hz         (double, default  10.0)     — timer rate (Hz)
//   check_after_sec (double, default  15.0)     — seconds before giving up
//
// Published topics
//   /waypoints  [geometry_msgs/PoseArray]   — consumed by my_drone_controller

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;

enum class PousoFSM { WAIT_FCU, WAIT_ODOM, PUBLISH, MONITOR };

class PousoNode : public rclcpp::Node
{
public:
  PousoNode()
  : Node("pouso"),
    fsm_(PousoFSM::WAIT_FCU),
    attempt_start_(rclcpp::Time(0, 0, RCL_ROS_TIME))
  {
    // ── parameters ──────────────────────────────────────────────────────────
    this->declare_parameter<std::string>("uav_name",        "uav1");
    this->declare_parameter<double>     ("x",               0.0);
    this->declare_parameter<double>     ("y",               0.0);
    this->declare_parameter<bool>       ("use_current_xy",  true);
    this->declare_parameter<double>     ("landing_z",       0.05);
    this->declare_parameter<std::string>("frame_id",        "map");
    this->declare_parameter<double>     ("rate_hz",         10.0);
    this->declare_parameter<double>     ("check_after_sec", 15.0);

    uav_name_        = this->get_parameter("uav_name").as_string();
    target_x_        = this->get_parameter("x").as_double();
    target_y_        = this->get_parameter("y").as_double();
    use_current_xy_  = this->get_parameter("use_current_xy").as_bool();
    landing_z_       = this->get_parameter("landing_z").as_double();
    frame_id_        = this->get_parameter("frame_id").as_string();
    check_after_sec_ = this->get_parameter("check_after_sec").as_double();
    double rate_hz   = this->get_parameter("rate_hz").as_double();

    // ── publisher / subscribers ──────────────────────────────────────────────
    waypoints_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/waypoints", 10);

    set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>(
      "/" + uav_name_ + "/mavros/set_mode");

    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
      "/" + uav_name_ + "/mavros/state", 10,
      std::bind(&PousoNode::stateCallback, this, _1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/" + uav_name_ + "/mavros/local_position/odom", 10,
      std::bind(&PousoNode::odomCallback, this, _1));

    auto period_ms = std::chrono::milliseconds(
      static_cast<int>(1000.0 / rate_hz));
    timer_ = this->create_wall_timer(
      period_ms, std::bind(&PousoNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(),
      "pouso node started. uav=%s landing_z=%.2f use_current_xy=%s "
      "target=(%.2f, %.2f) check_after=%.1fs",
      uav_name_.c_str(), landing_z_,
      use_current_xy_ ? "true" : "false",
      target_x_, target_y_, check_after_sec_);
  }

private:
  // ── callbacks ──────────────────────────────────────────────────────────────

  void stateCallback(const mavros_msgs::msg::State::SharedPtr msg)
  {
    fcu_connected_ = msg->connected;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
    current_z_ = msg->pose.pose.position.z;
    odom_received_ = true;
  }

  // ── FSM timer ──────────────────────────────────────────────────────────────

  void timerCallback()
  {
    switch (fsm_) {

      case PousoFSM::WAIT_FCU:
        if (!fcu_connected_) {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
            "Aguardando conexão com FCU…");
          return;
        }
        RCLCPP_INFO(this->get_logger(), "FCU conectado. Aguardando odometria…");
        fsm_ = PousoFSM::WAIT_ODOM;
        break;

      case PousoFSM::WAIT_ODOM:
        if (!odom_received_) {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Aguardando odometria antes de publicar ponto de pouso…");
          return;
        }
        RCLCPP_INFO(this->get_logger(),
          "Odometria recebida. Posição atual: X=%.2f Y=%.2f Z=%.2f",
          current_x_, current_y_, current_z_);
        fsm_ = PousoFSM::PUBLISH;
        break;

      case PousoFSM::PUBLISH:
        callAutoLand();
        publishLandingWaypoints();
        attempt_start_ = this->now();
        fsm_ = PousoFSM::MONITOR;
        break;

      case PousoFSM::MONITOR:
        monitorLanding();
        break;
    }
  }

  // ── helpers ────────────────────────────────────────────────────────────────

  void callAutoLand()
  {
    if (!set_mode_client_->wait_for_service(std::chrono::seconds(3))) {
      RCLCPP_WARN(this->get_logger(),
        "Serviço set_mode não disponível. Pulando AUTO.LAND.");
      return;
    }

    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->base_mode   = 0;
    request->custom_mode = "AUTO.LAND";

    set_mode_client_->async_send_request(request,
      [this](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
        auto result = future.get();
        if (result->mode_sent) {
          RCLCPP_INFO(this->get_logger(),
            "✅ Modo AUTO.LAND enviado com sucesso ao FCU.");
        } else {
          RCLCPP_WARN(this->get_logger(),
            "⚠️  Falha ao enviar modo AUTO.LAND ao FCU.");
        }
      });
  }

  void publishLandingWaypoints()
  {
    double land_x = use_current_xy_ ? current_x_ : target_x_;
    double land_y = use_current_xy_ ? current_y_ : target_y_;

    geometry_msgs::msg::PoseArray waypoints;
    waypoints.header.frame_id = frame_id_;
    waypoints.header.stamp    = this->now();

    // WP 1 — hover at current altitude above landing spot (smooth approach)
    geometry_msgs::msg::Pose wp_approach;
    wp_approach.position.x    = land_x;
    wp_approach.position.y    = land_y;
    wp_approach.position.z    = current_z_;
    wp_approach.orientation.w = 1.0;
    waypoints.poses.push_back(wp_approach);

    // WP 2 — descend to ground level
    geometry_msgs::msg::Pose wp_ground;
    wp_ground.position.x    = land_x;
    wp_ground.position.y    = land_y;
    wp_ground.position.z    = landing_z_;
    wp_ground.orientation.w = 1.0;
    waypoints.poses.push_back(wp_ground);

    waypoints_pub_->publish(waypoints);

    RCLCPP_INFO(this->get_logger(),
      "🛬 Ponto de pouso publicado em /waypoints:");
    RCLCPP_INFO(this->get_logger(),
      "   Posição XY: (%.2f, %.2f) [fonte: %s]",
      land_x, land_y,
      use_current_xy_ ? "odometria atual" : "parâmetros x/y");
    RCLCPP_INFO(this->get_logger(),
      "   WP[0]: X=%.2f Y=%.2f Z=%.2f (hover de aproximação)",
      wp_approach.position.x, wp_approach.position.y, wp_approach.position.z);
    RCLCPP_INFO(this->get_logger(),
      "   WP[1]: X=%.2f Y=%.2f Z=%.2f (solo)",
      wp_ground.position.x, wp_ground.position.y, wp_ground.position.z);
  }

  void monitorLanding()
  {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "⬇️  Descendo… altitude atual Z=%.2f m (alvo=%.2f m)",
      current_z_, landing_z_);

    if (current_z_ <= landing_z_ + 0.15) {
      RCLCPP_INFO(this->get_logger(),
        "✅ Pouso concluído: Z=%.2f m ≤ %.2f m. Encerrando.",
        current_z_, landing_z_ + 0.15);
      rclcpp::shutdown();
      return;
    }

    double elapsed = (this->now() - attempt_start_).seconds();
    if (elapsed >= check_after_sec_) {
      RCLCPP_WARN(this->get_logger(),
        "⚠️  Timeout aguardando pouso (%.1fs): Z=%.2f m. Encerrando.",
        check_after_sec_, current_z_);
      rclcpp::shutdown();
    }
  }

  // ── state ──────────────────────────────────────────────────────────────────

  PousoFSM fsm_;

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr waypoints_pub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr    state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr    odom_sub_;
  rclcpp::TimerBase::SharedPtr                                timer_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr        set_mode_client_;

  bool   fcu_connected_ {false};
  bool   odom_received_ {false};
  double current_x_     {0.0};
  double current_y_     {0.0};
  double current_z_     {0.0};

  rclcpp::Time attempt_start_;

  // Parameters
  std::string uav_name_;
  double      target_x_;
  double      target_y_;
  bool        use_current_xy_;
  double      landing_z_;
  std::string frame_id_;
  double      check_after_sec_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PousoNode>());
  rclcpp::shutdown();
  return 0;
}
