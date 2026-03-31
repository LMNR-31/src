// missao_P_T.cpp — Mission sequence: Takeoff → wait 5 s → Landing (pouso).
//
// Executes a simple two-phase mission:
//   1. Publish a takeoff waypoint to /waypoints (same logic as `takeoff`).
//   2. Wait until the drone reaches the takeoff altitude (or timeout/retry).
//   3. Wait `wait_sec` seconds (ROS wall-clock, timer-based, default 5 s).
//   4. Publish landing waypoints to /waypoints (same logic as `pouso`).
//   5. Wait until the drone lands (or timeout).
//
// Usage
//   ros2 run drone_control missao_P_T
//   ros2 run drone_control missao_P_T --ros-args \
//       -p uav_name:=uav1 \
//       -p takeoff_altitude:=1.5 \
//       -p wait_sec:=5.0
//
// Parameters (all optional — defaults match `takeoff` and `pouso`)
//   uav_name              (string, default "uav1")  — UAV namespace prefix
//   takeoff_altitude      (double, default  1.5)    — takeoff target altitude [m]
//   altitude_threshold    (double, default -1.0)    — success threshold (<0 → auto: takeoff_altitude-0.3)
//   landing_z             (double, default  0.05)   — final landing altitude [m]
//   use_current_xy        (bool,   default  true)   — use live odometry XY for both phases
//   x                     (double, default  0.0)    — static X (used when use_current_xy=false)
//   y                     (double, default  0.0)    — static Y (used when use_current_xy=false)
//   frame_id              (string, default "map")   — coordinate frame
//   rate_hz               (double, default 10.0)    — timer tick rate [Hz]
//   wait_sec              (double, default  5.0)    — wait between takeoff and landing [s]
//   takeoff_timeout_sec   (double, default 10.0)    — max time per takeoff attempt [s]
//   max_takeoff_attempts  (int,    default  3)      — retry count for takeoff
//   landing_timeout_sec   (double, default 15.0)    — max time to wait for landing [s]

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;

enum class MissionFSM {
  WAIT_FCU,
  WAIT_ODOM,
  TAKEOFF,
  MONITOR_TAKEOFF,
  WAIT_5S,
  POUSO,
  MONITOR_LANDING,
  DONE
};

class MissaoPTNode : public rclcpp::Node
{
public:
  MissaoPTNode()
  : Node("missao_P_T"),
    fsm_(MissionFSM::WAIT_FCU),
    phase_start_(rclcpp::Time(0, 0, RCL_ROS_TIME))
  {
    // ── parameters ──────────────────────────────────────────────────────────
    this->declare_parameter<std::string>("uav_name",             "uav1");
    this->declare_parameter<double>     ("takeoff_altitude",      1.5);
    this->declare_parameter<double>     ("altitude_threshold",   -1.0);
    this->declare_parameter<double>     ("landing_z",             0.05);
    this->declare_parameter<bool>       ("use_current_xy",        true);
    this->declare_parameter<double>     ("x",                     0.0);
    this->declare_parameter<double>     ("y",                     0.0);
    this->declare_parameter<std::string>("frame_id",             "map");
    this->declare_parameter<double>     ("rate_hz",              10.0);
    this->declare_parameter<double>     ("wait_sec",              5.0);
    this->declare_parameter<double>     ("takeoff_timeout_sec",  10.0);
    this->declare_parameter<int>        ("max_takeoff_attempts",  3);
    this->declare_parameter<double>     ("landing_timeout_sec",  15.0);

    uav_name_        = this->get_parameter("uav_name").as_string();
    takeoff_alt_     = this->get_parameter("takeoff_altitude").as_double();
    altitude_thresh_ = this->get_parameter("altitude_threshold").as_double();
    landing_z_       = this->get_parameter("landing_z").as_double();
    use_current_xy_  = this->get_parameter("use_current_xy").as_bool();
    fallback_x_      = this->get_parameter("x").as_double();
    fallback_y_      = this->get_parameter("y").as_double();
    frame_id_        = this->get_parameter("frame_id").as_string();
    wait_sec_        = this->get_parameter("wait_sec").as_double();
    takeoff_timeout_ = this->get_parameter("takeoff_timeout_sec").as_double();
    max_attempts_    = this->get_parameter("max_takeoff_attempts").as_int();
    landing_timeout_ = this->get_parameter("landing_timeout_sec").as_double();
    double rate_hz   = this->get_parameter("rate_hz").as_double();

    if (altitude_thresh_ < 0.0) {
      altitude_thresh_ = std::max(takeoff_alt_ - 0.3, 0.2);
    }

    // ── publisher / subscribers ──────────────────────────────────────────────
    waypoints_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/waypoints", 10);

    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
      "/" + uav_name_ + "/mavros/state", 10,
      std::bind(&MissaoPTNode::stateCallback, this, _1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/" + uav_name_ + "/mavros/local_position/odom", 10,
      std::bind(&MissaoPTNode::odomCallback, this, _1));

    auto period_ms = std::chrono::milliseconds(
      static_cast<int>(1000.0 / rate_hz));
    timer_ = this->create_wall_timer(
      period_ms, std::bind(&MissaoPTNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(),
      "[missao_P_T] node started. uav=%s takeoff_alt=%.2f landing_z=%.2f wait=%.1fs",
      uav_name_.c_str(), takeoff_alt_, landing_z_, wait_sec_);
  }

private:
  // ── callbacks ──────────────────────────────────────────────────────────────

  void stateCallback(const mavros_msgs::msg::State::SharedPtr msg)
  {
    fcu_connected_ = msg->connected;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_x_     = msg->pose.pose.position.x;
    current_y_     = msg->pose.pose.position.y;
    current_z_     = msg->pose.pose.position.z;
    odom_received_ = true;
  }

  // ── FSM timer ──────────────────────────────────────────────────────────────

  void timerCallback()
  {
    switch (fsm_) {

      case MissionFSM::WAIT_FCU:
        if (!fcu_connected_) {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
            "[missao_P_T] Aguardando conexão com FCU…");
          return;
        }
        RCLCPP_INFO(this->get_logger(),
          "[missao_P_T] FCU conectado. Aguardando odometria…");
        fsm_ = MissionFSM::WAIT_ODOM;
        break;

      case MissionFSM::WAIT_ODOM:
        if (!odom_received_) {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "[missao_P_T] Aguardando odometria…");
          return;
        }
        RCLCPP_INFO(this->get_logger(),
          "[missao_P_T] Odometria recebida. "
          "Posição atual: X=%.2f Y=%.2f Z=%.2f. "
          "Iniciando FASE 1: Takeoff.",
          current_x_, current_y_, current_z_);
        fsm_ = MissionFSM::TAKEOFF;
        break;

      case MissionFSM::TAKEOFF:
        publishTakeoffWaypoint();
        phase_start_ = this->now();
        fsm_         = MissionFSM::MONITOR_TAKEOFF;
        break;

      case MissionFSM::MONITOR_TAKEOFF:
        monitorTakeoff();
        break;

      case MissionFSM::WAIT_5S:
        waitBetweenPhases();
        break;

      case MissionFSM::POUSO:
        publishLandingWaypoints();
        phase_start_ = this->now();
        fsm_         = MissionFSM::MONITOR_LANDING;
        break;

      case MissionFSM::MONITOR_LANDING:
        monitorLanding();
        break;

      case MissionFSM::DONE:
        break;
    }
  }

  // ── helpers ────────────────────────────────────────────────────────────────

  void publishTakeoffWaypoint()
  {
    geometry_msgs::msg::PoseArray waypoints;
    waypoints.header.frame_id = frame_id_;
    waypoints.header.stamp    = this->now();

    geometry_msgs::msg::Pose pose;
    pose.position.x    = use_current_xy_ ? current_x_ : fallback_x_;
    pose.position.y    = use_current_xy_ ? current_y_ : fallback_y_;
    pose.position.z    = takeoff_alt_;
    pose.orientation.w = 1.0;

    waypoints.poses.push_back(pose);
    waypoints_pub_->publish(waypoints);

    RCLCPP_INFO(this->get_logger(),
      "[missao_P_T] 🚀 Takeoff publicado em /waypoints "
      "(tentativa %d/%d): x=%.2f y=%.2f z=%.2f",
      attempt_count_ + 1, max_attempts_,
      pose.position.x, pose.position.y, pose.position.z);
  }

  void monitorTakeoff()
  {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "[missao_P_T] Subindo… Z=%.2f m (alvo≥%.2f m, tentativa %d/%d)",
      current_z_, altitude_thresh_, attempt_count_ + 1, max_attempts_);

    if (current_z_ >= altitude_thresh_) {
      RCLCPP_INFO(this->get_logger(),
        "[missao_P_T] ✅ Takeoff concluído: Z=%.2f m. "
        "Aguardando %.1f s antes do pouso…",
        current_z_, wait_sec_);
      phase_start_ = this->now();
      fsm_         = MissionFSM::WAIT_5S;
      return;
    }

    double elapsed = (this->now() - phase_start_).seconds();
    if (elapsed >= takeoff_timeout_) {
      attempt_count_++;
      if (attempt_count_ >= max_attempts_) {
        RCLCPP_ERROR(this->get_logger(),
          "[missao_P_T] ❌ Takeoff falhou após %d tentativa(s): Z=%.2f m. Abortando.",
          max_attempts_, current_z_);
        rclcpp::shutdown();
        return;
      }
      RCLCPP_WARN(this->get_logger(),
        "[missao_P_T] Altitude não atingida após %.1fs (Z=%.2f m). "
        "Republicando (tentativa %d/%d)…",
        takeoff_timeout_, current_z_, attempt_count_ + 1, max_attempts_);
      fsm_ = MissionFSM::TAKEOFF;
    }
  }

  void waitBetweenPhases()
  {
    double elapsed = (this->now() - phase_start_).seconds();
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "[missao_P_T] ⏳ Aguardando %.1f s antes do pouso… (%.1f s / %.1f s)",
      wait_sec_, elapsed, wait_sec_);
    if (elapsed >= wait_sec_) {
      RCLCPP_INFO(this->get_logger(),
        "[missao_P_T] Espera de %.1f s concluída. Iniciando FASE 2: Pouso.",
        wait_sec_);
      fsm_ = MissionFSM::POUSO;
    }
  }

  void publishLandingWaypoints()
  {
    double land_x = use_current_xy_ ? current_x_ : fallback_x_;
    double land_y = use_current_xy_ ? current_y_ : fallback_y_;

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
      "[missao_P_T] 🛬 Ponto de pouso publicado em /waypoints:");
    RCLCPP_INFO(this->get_logger(),
      "   XY: (%.2f, %.2f) [fonte: %s]",
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
      "[missao_P_T] ⬇️  Descendo… Z=%.2f m (alvo≤%.2f m)",
      current_z_, landing_z_ + 0.15);

    if (current_z_ <= landing_z_ + 0.15) {
      RCLCPP_INFO(this->get_logger(),
        "[missao_P_T] ✅ Missão concluída: pouso em Z=%.2f m. Encerrando.",
        current_z_);
      fsm_ = MissionFSM::DONE;
      rclcpp::shutdown();
      return;
    }

    double elapsed = (this->now() - phase_start_).seconds();
    if (elapsed >= landing_timeout_) {
      RCLCPP_WARN(this->get_logger(),
        "[missao_P_T] ⚠️  Timeout aguardando pouso (%.1fs): Z=%.2f m. Encerrando.",
        landing_timeout_, current_z_);
      fsm_ = MissionFSM::DONE;
      rclcpp::shutdown();
    }
  }

  // ── state ──────────────────────────────────────────────────────────────────

  MissionFSM fsm_;
  rclcpp::Time phase_start_;
  int attempt_count_ {0};

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr waypoints_pub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr    state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr    odom_sub_;
  rclcpp::TimerBase::SharedPtr                                timer_;

  bool   fcu_connected_  {false};
  bool   odom_received_  {false};
  double current_x_      {0.0};
  double current_y_      {0.0};
  double current_z_      {0.0};

  // Parameters
  std::string uav_name_;
  double      takeoff_alt_;
  double      altitude_thresh_;
  double      landing_z_;
  bool        use_current_xy_;
  double      fallback_x_;
  double      fallback_y_;
  std::string frame_id_;
  double      wait_sec_;
  double      takeoff_timeout_;
  int         max_attempts_;
  double      landing_timeout_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissaoPTNode>());
  rclcpp::shutdown();
  return 0;
}
