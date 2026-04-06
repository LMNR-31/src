// pouso.cpp — ROS 2 node for publishing an arbitrary landing point on the map.
//
// Landing uses an explicit two-phase FSM to avoid XY-drift oscillation:
//
//   CENTER  — hover at target XY at approach altitude until the drone is
//             centred (dxy ≤ xy_hold_tol) continuously for xy_hold_stable_s
//             seconds.  Waypoints are published on phase entry; the drone
//             controller handles the positioning.
//   DESCEND — command a single descent waypoint to landing_z.  Waypoints are
//             NOT reissued on minor XY drift.  If drift exceeds xy_abort_tol
//             the phase reverts to CENTER (drone climbs back to approach_z
//             and recentres before retrying the descent).
//
// Parameters
//   uav_name          (string, default "uav1")   — UAV namespace prefix
//   x                 (double, default  0.0)     — target landing X (ENU, m)
//   y                 (double, default  0.0)     — target landing Y (ENU, m)
//   use_current_xy    (bool,   default  true)    — ignore x/y and use live odom XY
//   landing_z         (double, default  0.05)    — final ground altitude (m)
//   frame_id          (string, default "map")    — coordinate frame
//   rate_hz           (double, default  10.0)    — timer rate (Hz)
//   check_after_sec   (double, default  15.0)    — seconds before giving up
//   xy_hold_tol       (double, default  0.10)    — max planar error (m) considered centred
//   xy_hold_stable_s  (double, default  1.0)     — seconds to remain within xy_hold_tol before descending
//   xy_abort_tol      (double, default  0.5)     — abort descent and recenter if dxy exceeds this (m)
//   approach_z        (double, default -1.0)     — hover altitude for CENTER phase (m); -1 = current odom Z
//   use_yolo_h        (bool,   default  false)   — use YOLO H detection for landing XY
//   h_topic           (string, default "/landing_pad/h_relative_position") — YOLO H topic
//   h_collect_time_s  (double, default  1.0)     — seconds to hover and collect H detections before choosing best
//   h_timeout_s       (double, default  0.75)    — max age (s) of a valid H detection (rolling window)
//   max_h_range_m     (double, default  6.0)     — max planar range (m) to accept a detection
//   prefer_closest_h  (bool,   default  true)    — pick H closest to drone; false = latest
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

enum class PousoFSM { WAIT_FCU, WAIT_ODOM, COLLECT_H, CENTER, DESCEND };

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
    attempt_start_(rclcpp::Time(0, 0, RCL_ROS_TIME)),
    center_stable_since_(rclcpp::Time(0, 0, RCL_ROS_TIME))
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
    this->declare_parameter<double>     ("xy_hold_tol",      0.10);
    this->declare_parameter<double>     ("xy_hold_stable_s", 1.0);
    this->declare_parameter<double>     ("xy_abort_tol",     0.5);
    this->declare_parameter<double>     ("approach_z",      -1.0);

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
    xy_hold_tol_      = this->get_parameter("xy_hold_tol").as_double();
    xy_hold_stable_s_ = this->get_parameter("xy_hold_stable_s").as_double();
    xy_abort_tol_     = this->get_parameter("xy_abort_tol").as_double();
    approach_z_param_ = this->get_parameter("approach_z").as_double();

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

    std::string approach_z_str = (approach_z_param_ >= 0.0)
      ? (std::to_string(approach_z_param_) + "m")
      : std::string("auto(odom)");
    RCLCPP_INFO(this->get_logger(),
      "pouso node started. uav=%s landing_z=%.2f use_current_xy=%s "
      "target=(%.2f, %.2f) check_after=%.1fs "
      "xy_hold_tol=%.3fm xy_hold_stable_s=%.1fs xy_abort_tol=%.3fm "
      "approach_z=%s use_yolo_h=%s",
      uav_name_.c_str(), landing_z_,
      use_current_xy_ ? "true" : "false",
      target_x_, target_y_, check_after_sec_,
      xy_hold_tol_, xy_hold_stable_s_, xy_abort_tol_,
      approach_z_str.c_str(),
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
          startLanding();
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
          startLanding();
          break;
        }

      case PousoFSM::CENTER:
        runCenter();
        break;

      case PousoFSM::DESCEND:
        runDescend();
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

  // Compute landing target from YOLO-H / odom / params and store in
  // active_land_x_ / active_land_y_.
  void computeLandingTarget()
  {
    if (use_yolo_h_) {
      if (has_best_h_) {
        double yaw = current_yaw_;
        double dx  = std::cos(yaw) * best_collected_h_.front + std::sin(yaw) * best_collected_h_.right;
        double dy  = std::sin(yaw) * best_collected_h_.front - std::cos(yaw) * best_collected_h_.right;
        active_land_x_ = current_x_ + dx;
        active_land_y_ = current_y_ + dy;
        RCLCPP_INFO(this->get_logger(),
          "[yolo_h] Alvo YOLO-H: XY=(%.2f, %.2f) | right=%.2f front=%.2f "
          "range=%.2f yaw=%.3frad",
          active_land_x_, active_land_y_,
          best_collected_h_.right, best_collected_h_.front,
          best_collected_h_.range, yaw);
      } else {
        RCLCPP_WARN(this->get_logger(),
          "[yolo_h] Nenhuma detecção H válida coletada. "
          "Usando fallback (odometria/parâmetros).");
        active_land_x_ = use_current_xy_ ? current_x_ : target_x_;
        active_land_y_ = use_current_xy_ ? current_y_ : target_y_;
      }
    } else {
      active_land_x_ = use_current_xy_ ? current_x_ : target_x_;
      active_land_y_ = use_current_xy_ ? current_y_ : target_y_;
    }
  }

  // Called once after odom/COLLECT_H is ready: compute target, set approach
  // altitude, call AUTO.LAND, then enter CENTER phase.
  void startLanding()
  {
    computeLandingTarget();

    // Determine approach altitude: use param if set, else use current odom Z.
    approach_z_ = (approach_z_param_ >= 0.0) ? approach_z_param_ : current_z_;

    const char * fonte = use_yolo_h_ && has_best_h_
      ? "yolo-H"
      : (use_current_xy_ ? "odometria atual" : "parâmetros x/y");
    RCLCPP_INFO(this->get_logger(),
      "🛬 Ponto de pouso: XY=(%.2f, %.2f) [fonte: %s] | approach_z=%.2f m | landing_z=%.2f m",
      active_land_x_, active_land_y_, fonte, approach_z_, landing_z_);

    callAutoLand();
    attempt_start_ = this->now();
    enterCenter();
  }

  // Transition to CENTER phase: publish hover waypoint and reset stability
  // tracking.  Safe to call from DESCEND abort as well.
  void enterCenter()
  {
    double dxy = std::hypot(current_x_ - active_land_x_, current_y_ - active_land_y_);
    RCLCPP_INFO(this->get_logger(),
      "🎯 Entrando em CENTER: dxy=%.3f m, alvo=(%.2f, %.2f), approach_z=%.2f m",
      dxy, active_land_x_, active_land_y_, approach_z_);

    in_center_stable_    = false;
    center_stable_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    publishCenterWaypoint();
    fsm_ = PousoFSM::CENTER;
  }

  // Publish a single hover waypoint at approach altitude above the landing
  // target.  Only called when entering CENTER.
  void publishCenterWaypoint()
  {
    geometry_msgs::msg::PoseArray waypoints;
    waypoints.header.frame_id = frame_id_;
    waypoints.header.stamp    = this->now();

    geometry_msgs::msg::Pose wp;
    wp.position.x    = active_land_x_;
    wp.position.y    = active_land_y_;
    wp.position.z    = approach_z_;
    wp.orientation.w = 1.0;
    waypoints.poses.push_back(wp);

    waypoints_pub_->publish(waypoints);
    RCLCPP_INFO(this->get_logger(),
      "   WP[0]: X=%.2f Y=%.2f Z=%.2f (hover de aproximação)",
      wp.position.x, wp.position.y, wp.position.z);
  }

  // Publish a single descent waypoint to landing_z.  Called once when
  // transitioning from CENTER to DESCEND.
  void publishDescentWaypoint()
  {
    geometry_msgs::msg::PoseArray waypoints;
    waypoints.header.frame_id = frame_id_;
    waypoints.header.stamp    = this->now();

    geometry_msgs::msg::Pose wp;
    wp.position.x    = active_land_x_;
    wp.position.y    = active_land_y_;
    wp.position.z    = landing_z_;
    wp.orientation.w = 1.0;
    waypoints.poses.push_back(wp);

    waypoints_pub_->publish(waypoints);
    RCLCPP_INFO(this->get_logger(),
      "   WP[0]: X=%.2f Y=%.2f Z=%.2f (solo — descida)",
      wp.position.x, wp.position.y, wp.position.z);
  }

  // CENTER phase: wait until drone is stably centred before descending.
  void runCenter()
  {
    double dxy = std::hypot(current_x_ - active_land_x_, current_y_ - active_land_y_);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "🎯 CENTER: dxy=%.3f m (tol=%.3f m) | approach_z=%.2f m",
      dxy, xy_hold_tol_, approach_z_);

    if (dxy <= xy_hold_tol_) {
      if (!in_center_stable_) {
        in_center_stable_    = true;
        center_stable_since_ = this->now();
      }
      double stable_dur = (this->now() - center_stable_since_).seconds();
      if (stable_dur >= xy_hold_stable_s_) {
        RCLCPP_INFO(this->get_logger(),
          "✅ Centrado: dxy=%.3f m ≤ %.3f m por %.1fs. Iniciando descida para Z=%.2f m.",
          dxy, xy_hold_tol_, stable_dur, landing_z_);
        publishDescentWaypoint();
        fsm_ = PousoFSM::DESCEND;
        return;
      }
    } else {
      in_center_stable_ = false;
    }

    double elapsed = (this->now() - attempt_start_).seconds();
    if (elapsed >= check_after_sec_) {
      RCLCPP_WARN(this->get_logger(),
        "⚠️  Timeout em CENTER (%.1fs): dxy=%.3f m. Encerrando.",
        check_after_sec_, dxy);
      rclcpp::shutdown();
    }
  }

  // DESCEND phase: monitor descent; no waypoint republishing on minor drift.
  // Abort back to CENTER only when drift exceeds xy_abort_tol_.
  void runDescend()
  {
    double dxy  = std::hypot(current_x_ - active_land_x_, current_y_ - active_land_y_);
    bool   z_ok = (current_z_ <= landing_z_ + 0.15);
    bool   xy_ok = (dxy <= xy_hold_tol_);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "⬇️  DESCEND: Z=%.2f m (alvo=%.2f m) | dxy=%.3f m (tol=%.3f m) XY=%s",
      current_z_, landing_z_, dxy, xy_hold_tol_, xy_ok ? "OK" : "FORA");

    if (z_ok && xy_ok) {
      RCLCPP_INFO(this->get_logger(),
        "✅ Pouso concluído: Z=%.2f m ≤ %.2f m, dxy=%.3f m ≤ %.3f m. Encerrando.",
        current_z_, landing_z_ + 0.15, dxy, xy_hold_tol_);
      rclcpp::shutdown();
      return;
    }

    // Abort on excessive drift — return to CENTER (drone climbs back to
    // approach_z_ and recentres before retrying descent).
    if (dxy > xy_abort_tol_) {
      RCLCPP_WARN(this->get_logger(),
        "⚠️  Deriva excessiva durante descida: dxy=%.3f m > xy_abort_tol=%.3f m. "
        "Abortando descida — voltando para CENTER.",
        dxy, xy_abort_tol_);
      enterCenter();
      return;
    }

    double elapsed = (this->now() - attempt_start_).seconds();
    if (elapsed >= check_after_sec_) {
      RCLCPP_WARN(this->get_logger(),
        "⚠️  Timeout aguardando pouso (%.1fs): Z=%.2f m, dxy=%.3f m. Encerrando.",
        check_after_sec_, current_z_, dxy);
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
  double active_land_x_ {0.0};  // frozen target; valid from CENTER state onward
  double active_land_y_ {0.0};  // frozen target; valid from CENTER state onward
  double approach_z_    {0.0};  // hover altitude for CENTER phase (set in startLanding())

  std::vector<HDetection> h_detections_;

  rclcpp::Time attempt_start_;
  rclcpp::Time collect_start_;
  rclcpp::Time center_stable_since_;
  bool         in_center_stable_  {false};
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
  double      xy_hold_tol_;
  double      xy_hold_stable_s_;
  double      xy_abort_tol_;
  double      approach_z_param_;  // raw param; <0 means "use current odom Z"
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
