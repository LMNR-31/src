// pouso.cpp — ROS 2 node for publishing an arbitrary landing point on the map.
//
// Publishes a two-waypoint descent sequence to /waypoints so that
// my_drone_controller navigates the drone to the desired XY position and
// then descends to ground level.
//
// Parameters
//   uav_name         (string, default "uav1")    — UAV namespace prefix
//   x                (double, default  0.0)      — target landing X (ENU, m)
//   y                (double, default  0.0)      — target landing Y (ENU, m)
//   use_current_xy   (bool,   default  true)     — ignore x/y and use live odom XY
//   landing_z        (double, default  0.05)     — final ground altitude (m)
//   frame_id         (string, default "map")     — coordinate frame
//   rate_hz          (double, default  10.0)     — timer rate (Hz)
//   check_after_sec  (double, default  15.0)     — seconds before giving up
//   use_yolo_h       (bool,   default  false)    — use YOLO H detection for landing XY
//   h_topic          (string, default "/landing_pad/h_relative_position") — YOLO H topic
//   h_collect_time_s (double, default  1.0)      — seconds to hover and collect H detections (COLLECT_H state) before choosing best
//   h_timeout_s      (double, default  0.75)     — max age (s) of a valid H detection (rolling window)
//   max_h_range_m    (double, default  6.0)      — max planar range (m) to accept a detection
//   prefer_closest_h (bool,   default  true)     — pick H closest to drone; false = latest
//
// Published topics
//   /waypoints  [geometry_msgs/PoseArray]   — consumed by my_drone_controller

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <chrono>
#include <cmath>
#include <vector>
#include <algorithm>

using namespace std::chrono_literals;
using std::placeholders::_1;

enum class PousoFSM { WAIT_FCU, WAIT_ODOM, COLLECT_H, PUBLISH, MONITOR };

struct HDetection
{
  rclcpp::Time stamp;
  double right {0.0};
  double front {0.0};
  double range {0.0};
};

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
    this->declare_parameter<bool>       ("use_yolo_h",      false);
    this->declare_parameter<std::string>("h_topic",         "/landing_pad/h_relative_position");
    this->declare_parameter<double>     ("h_collect_time_s", 1.0);
    this->declare_parameter<double>     ("h_timeout_s",     0.75);
    this->declare_parameter<double>     ("max_h_range_m",   6.0);
    this->declare_parameter<bool>       ("prefer_closest_h", true);

    uav_name_        = this->get_parameter("uav_name").as_string();
    target_x_        = this->get_parameter("x").as_double();
    target_y_        = this->get_parameter("y").as_double();
    use_current_xy_  = this->get_parameter("use_current_xy").as_bool();
    landing_z_       = this->get_parameter("landing_z").as_double();
    frame_id_        = this->get_parameter("frame_id").as_string();
    check_after_sec_ = this->get_parameter("check_after_sec").as_double();
    double rate_hz   = this->get_parameter("rate_hz").as_double();
    use_yolo_h_      = this->get_parameter("use_yolo_h").as_bool();
    h_topic_         = this->get_parameter("h_topic").as_string();
    h_collect_time_s_ = this->get_parameter("h_collect_time_s").as_double();
    h_timeout_s_     = this->get_parameter("h_timeout_s").as_double();
    max_h_range_m_   = this->get_parameter("max_h_range_m").as_double();
    prefer_closest_h_ = this->get_parameter("prefer_closest_h").as_bool();

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

    if (use_yolo_h_) {
      h_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        h_topic_, 10,
        std::bind(&PousoNode::hCallback, this, _1));
    }

    auto period_ms = std::chrono::milliseconds(
      static_cast<int>(1000.0 / rate_hz));
    timer_ = this->create_wall_timer(
      period_ms, std::bind(&PousoNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(),
      "pouso node started. uav=%s landing_z=%.2f use_current_xy=%s "
      "target=(%.2f, %.2f) check_after=%.1fs use_yolo_h=%s",
      uav_name_.c_str(), landing_z_,
      use_current_xy_ ? "true" : "false",
      target_x_, target_y_, check_after_sec_,
      use_yolo_h_ ? "true" : "false");
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
    // Extract yaw from quaternion (ENU convention)
    const auto & q = msg->pose.pose.orientation;
    current_yaw_ = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    odom_received_ = true;
  }

  void hCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    double right = msg->point.x;
    double front = msg->point.y;
    double range = std::hypot(right, front);

    if (range > max_h_range_m_) {
      RCLCPP_DEBUG(this->get_logger(),
        "[yolo_h] Ignorado: range=%.2fm > max=%.2fm right=%.2f front=%.2f",
        range, max_h_range_m_, right, front);
      return;
    }

    // During collection window, track the best (closest) candidate
    if (fsm_ == PousoFSM::COLLECT_H) {
      h_collect_count_++;
      if (!has_best_h_ || range < best_collected_h_.range) {
        best_collected_h_ = {this->now(), right, front, range};
        has_best_h_ = true;
      }
    }

    h_detections_.push_back({this->now(), right, front, range});

    // Prune detections older than h_timeout_s_
    rclcpp::Time now = this->now();
    h_detections_.erase(
      std::remove_if(
        h_detections_.begin(), h_detections_.end(),
        [&](const HDetection & d) {
          return (now - d.stamp).seconds() > h_timeout_s_;
        }),
      h_detections_.end());
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
        if (use_yolo_h_) {
          RCLCPP_INFO(this->get_logger(),
            "[yolo_h] Iniciando coleta de detecções H por %.1fs…",
            h_collect_time_s_);
          collect_start_    = this->now();
          h_collect_count_  = 0;
          has_best_h_       = false;
          fsm_ = PousoFSM::COLLECT_H;
        } else {
          fsm_ = PousoFSM::PUBLISH;
        }
        break;

      case PousoFSM::COLLECT_H:
        {
          double elapsed = (this->now() - collect_start_).seconds();
          if (elapsed < h_collect_time_s_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
              "[yolo_h] Coletando H… %.1f/%.1fs (%d detecções aceitas)",
              elapsed, h_collect_time_s_, h_collect_count_);
            return;
          }
          // Collection window elapsed
          if (has_best_h_) {
            RCLCPP_INFO(this->get_logger(),
              "[yolo_h] Coleta concluída (%.1fs): %d detecções aceitas, "
              "melhor range=%.2fm (right=%.2f front=%.2f)",
              h_collect_time_s_, h_collect_count_,
              best_collected_h_.range, best_collected_h_.right, best_collected_h_.front);
          } else {
            RCLCPP_WARN(this->get_logger(),
              "[yolo_h] Coleta concluída (%.1fs): NENHUMA detecção H válida. "
              "Usando fallback (odometria/parâmetros).",
              h_collect_time_s_);
          }
          fsm_ = PousoFSM::PUBLISH;
          break;
        }

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
    double land_x;
    double land_y;
    bool used_yolo_h = false;

    if (use_yolo_h_) {
      if (has_best_h_) {
        double yaw = current_yaw_;
        double dx  = std::cos(yaw) * best_collected_h_.front + std::sin(yaw) * best_collected_h_.right;
        double dy  = std::sin(yaw) * best_collected_h_.front - std::cos(yaw) * best_collected_h_.right;
        land_x = current_x_ + dx;
        land_y = current_y_ + dy;
        used_yolo_h = true;
        RCLCPP_INFO(this->get_logger(),
          "[yolo_h] Alvo YOLO-H: XY=(%.2f, %.2f) | right=%.2f front=%.2f "
          "range=%.2f yaw=%.3frad",
          land_x, land_y, best_collected_h_.right, best_collected_h_.front,
          best_collected_h_.range, yaw);
      } else {
        RCLCPP_WARN(this->get_logger(),
          "[yolo_h] Nenhuma detecção H válida coletada. "
          "Usando fallback (odometria/parâmetros).");
        land_x = use_current_xy_ ? current_x_ : target_x_;
        land_y = use_current_xy_ ? current_y_ : target_y_;
      }
    } else {
      land_x = use_current_xy_ ? current_x_ : target_x_;
      land_y = use_current_xy_ ? current_y_ : target_y_;
    }

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

    const char * fonte =
      used_yolo_h ? "yolo-H" : (use_current_xy_ ? "odometria atual" : "parâmetros x/y");
    RCLCPP_INFO(this->get_logger(),
      "🛬 Ponto de pouso publicado em /waypoints:");
    RCLCPP_INFO(this->get_logger(),
      "   Posição XY: (%.2f, %.2f) [fonte: %s]",
      land_x, land_y, fonte);
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

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr                waypoints_pub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr                   state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr                   odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr          h_sub_;
  rclcpp::TimerBase::SharedPtr                                               timer_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr                       set_mode_client_;

  bool   fcu_connected_ {false};
  bool   odom_received_ {false};
  double current_x_     {0.0};
  double current_y_     {0.0};
  double current_z_     {0.0};
  double current_yaw_   {0.0};

  std::vector<HDetection> h_detections_;

  rclcpp::Time attempt_start_;
  rclcpp::Time collect_start_;
  int          h_collect_count_ {0};
  bool         has_best_h_      {false};
  HDetection   best_collected_h_;

  // Parameters
  std::string uav_name_;
  double      target_x_;
  double      target_y_;
  bool        use_current_xy_;
  double      landing_z_;
  std::string frame_id_;
  double      check_after_sec_;
  bool        use_yolo_h_;
  std::string h_topic_;
  double      h_collect_time_s_;
  double      h_timeout_s_;
  double      max_h_range_m_;
  bool        prefer_closest_h_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PousoNode>());
  rclcpp::shutdown();
  return 0;
}
