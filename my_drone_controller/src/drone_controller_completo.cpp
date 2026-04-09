#include "my_drone_controller/drone_controller_completo.hpp"

#include <cmath>
#include <functional>
#include <string>

using namespace std::chrono_literals;

namespace drone_control {

// ============================================================
// CONSTRUCTOR
// ============================================================

DroneControllerCompleto::DroneControllerCompleto()
: Node("drone_controller_completo")
{
  RCLCPP_INFO(this->get_logger(), "\n");
  RCLCPP_INFO(this->get_logger(), "╔════════════════════════════════════════════════════════════╗");
  RCLCPP_INFO(this->get_logger(), "║      🚁 CONTROLADOR INTELIGENTE DE DRONE - MODULAR       ║");
  RCLCPP_INFO(this->get_logger(), "║     COM ATIVAÇÃO OFFBOARD + ARM + DETECÇÃO DE POUSO     ║");
  RCLCPP_INFO(this->get_logger(), "║      FSM 5 ESTADOS — FLUXO DE POUSO ÚNICO (DISARM)      ║");
  RCLCPP_INFO(this->get_logger(), "╚════════════════════════════════════════════════════════════╝\n");

  load_parameters();
  setup_publishers();
  setup_subscribers();
  setup_services();
  init_variables();

  RCLCPP_INFO(this->get_logger(), "\n📊 STATUS INICIAL:");
  RCLCPP_INFO(this->get_logger(), "   Estado: %d (aguardando waypoint)", state_voo_);
  RCLCPP_INFO(this->get_logger(), "   Controlador: %s", controlador_ativo_ ? "ATIVO" : "INATIVO");
  RCLCPP_INFO(this->get_logger(), "   Pouso: %s\n", pouso_em_andamento_ ? "SIM" : "NÃO");
}

// ============================================================
// SETUP
// ============================================================

void DroneControllerCompleto::load_parameters()
{
  this->declare_parameter("hover_altitude",        config_.hover_altitude);
  this->declare_parameter("hover_altitude_margin", config_.hover_altitude_margin);
  this->declare_parameter("max_altitude",          config_.max_altitude);
  this->declare_parameter("min_altitude",          config_.min_altitude);
  this->declare_parameter("waypoint_duration",     config_.waypoint_duration);
  this->declare_parameter("max_waypoint_distance", config_.max_waypoint_distance);
  this->declare_parameter("land_z_threshold",      config_.land_z_threshold);
  this->declare_parameter("activation_timeout",      config_.activation_timeout);
  this->declare_parameter("command_timeout",         config_.command_timeout);
  this->declare_parameter("landing_timeout",         config_.landing_timeout);
  this->declare_parameter("offboard_confirm_timeout", config_.offboard_confirm_timeout);
  this->declare_parameter("takeoff_z_boost",         config_.takeoff_z_boost);
  this->declare_parameter("takeoff_xy_origin_threshold_m", config_.takeoff_xy_origin_threshold_m);
  this->declare_parameter("latch_pose_max_age_s",         config_.latch_pose_max_age_s);

  config_.hover_altitude          = this->get_parameter("hover_altitude").as_double();
  config_.hover_altitude_margin   = this->get_parameter("hover_altitude_margin").as_double();
  config_.max_altitude            = this->get_parameter("max_altitude").as_double();
  config_.min_altitude            = this->get_parameter("min_altitude").as_double();
  config_.waypoint_duration       = this->get_parameter("waypoint_duration").as_double();
  config_.max_waypoint_distance   = this->get_parameter("max_waypoint_distance").as_double();
  config_.land_z_threshold        = this->get_parameter("land_z_threshold").as_double();
  config_.activation_timeout      = this->get_parameter("activation_timeout").as_double();
  config_.command_timeout         = this->get_parameter("command_timeout").as_double();
  config_.landing_timeout         = this->get_parameter("landing_timeout").as_double();
  config_.offboard_confirm_timeout = this->get_parameter("offboard_confirm_timeout").as_double();
  config_.takeoff_z_boost         = this->get_parameter("takeoff_z_boost").as_double();
  config_.takeoff_xy_origin_threshold_m =
    this->get_parameter("takeoff_xy_origin_threshold_m").as_double();
  config_.latch_pose_max_age_s    = this->get_parameter("latch_pose_max_age_s").as_double();

  this->declare_parameter<bool>("enabled", true);
  enabled_ = this->get_parameter("enabled").as_bool();
  RCLCPP_INFO(this->get_logger(), "⚙️  enabled=%s", enabled_ ? "true" : "false");

  this->declare_parameter<bool>("override_active", false);
  override_active_ = this->get_parameter("override_active").as_bool();
  RCLCPP_INFO(this->get_logger(), "⚙️  override_active=%s", override_active_ ? "true" : "false");

  this->declare_parameter<double>("monitor_waypoint_goal_rate_hz", 5.0);
  this->declare_parameter<double>("monitor_waypoints_rate_hz",     1.0);
  this->declare_parameter<bool>  ("monitor_publish_only_when_active", true);
  monitor_waypoint_goal_rate_hz_    = this->get_parameter("monitor_waypoint_goal_rate_hz").as_double();
  monitor_waypoints_rate_hz_        = this->get_parameter("monitor_waypoints_rate_hz").as_double();
  monitor_publish_only_when_active_ = this->get_parameter("monitor_publish_only_when_active").as_bool();
  RCLCPP_INFO(this->get_logger(),
    "⚙️  monitor_waypoint_goal_rate_hz=%.1f  monitor_waypoints_rate_hz=%.1f"
    "  monitor_publish_only_when_active=%s",
    monitor_waypoint_goal_rate_hz_, monitor_waypoints_rate_hz_,
    monitor_publish_only_when_active_ ? "true" : "false");

  this->declare_parameter<bool>  ("publish_state_voo", true);
  this->declare_parameter<double>("state_voo_pub_rate_hz", 1.0);
  publish_state_voo_      = this->get_parameter("publish_state_voo").as_bool();
  state_voo_pub_rate_hz_  = this->get_parameter("state_voo_pub_rate_hz").as_double();
  RCLCPP_INFO(this->get_logger(),
    "⚙️  publish_state_voo=%s  state_voo_pub_rate_hz=%.1f",
    publish_state_voo_ ? "true" : "false", state_voo_pub_rate_hz_);

  // ── Topic-name parameters (separates command-input from status-output) ───
  this->declare_parameter<std::string>("waypoints_cmd_topic",        "/waypoints");
  this->declare_parameter<std::string>("waypoints_status_topic",     "/waypoints");
  this->declare_parameter<std::string>("waypoint_goal_cmd_topic",    "/waypoint_goal");
  this->declare_parameter<std::string>("waypoint_goal_status_topic", "/waypoint_goal");
  waypoints_cmd_topic_        = this->get_parameter("waypoints_cmd_topic").as_string();
  waypoints_status_topic_     = this->get_parameter("waypoints_status_topic").as_string();
  waypoint_goal_cmd_topic_    = this->get_parameter("waypoint_goal_cmd_topic").as_string();
  waypoint_goal_status_topic_ = this->get_parameter("waypoint_goal_status_topic").as_string();
  RCLCPP_INFO(this->get_logger(),
    "⚙️  waypoints_cmd_topic=%s  waypoints_status_topic=%s",
    waypoints_cmd_topic_.c_str(), waypoints_status_topic_.c_str());
  RCLCPP_INFO(this->get_logger(),
    "⚙️  waypoint_goal_cmd_topic=%s  waypoint_goal_status_topic=%s",
    waypoint_goal_cmd_topic_.c_str(), waypoint_goal_status_topic_.c_str());
  if (waypoints_cmd_topic_ == waypoints_status_topic_) {
    RCLCPP_WARN(this->get_logger(),
      "⚠️  waypoints_cmd_topic == waypoints_status_topic ('%s'): "
      "using the same topic for command input and status output may cause "
      "publisher conflicts when other nodes (e.g. pad_waypoint_supervisor) "
      "also publish waypoints. Consider setting waypoints_status_topic to "
      "'/waypoints_status'.",
      waypoints_cmd_topic_.c_str());
  }
  if (waypoint_goal_cmd_topic_ == waypoint_goal_status_topic_) {
    RCLCPP_WARN(this->get_logger(),
      "⚠️  waypoint_goal_cmd_topic == waypoint_goal_status_topic ('%s'): "
      "using the same topic for command input and status output may cause "
      "publisher conflicts. Consider setting waypoint_goal_status_topic to "
      "'/waypoint_goal_status'.",
      waypoint_goal_cmd_topic_.c_str());
  }

  param_cb_handle_ = this->add_on_set_parameters_callback(
    std::bind(&DroneControllerCompleto::onSetParameters, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "⚙️  Configuração carregada:");
  RCLCPP_INFO(this->get_logger(), "   hover_altitude=%.2f m  margin=%.3f m",
    config_.hover_altitude, config_.hover_altitude_margin);
  RCLCPP_INFO(this->get_logger(), "   waypoint_duration=%.1f s  command_timeout=%.1f s",
    config_.waypoint_duration, config_.command_timeout);
  RCLCPP_INFO(this->get_logger(),
    "⚙️  Configuração de Altitude: Mínima=%.2f m | Pouso detectado: Z < %.2f m | Máxima=%.2f m",
    config_.min_altitude, config_.land_z_threshold, config_.max_altitude);
  RCLCPP_INFO(this->get_logger(),
    "⚙️  Boost de decolagem: takeoff_z_boost=%.2f m "
    "(Z_alvo ≥ Z_real + takeoff_z_boost após ARM, evita auto-disarm do PX4)",
    config_.takeoff_z_boost);
}

void DroneControllerCompleto::setup_publishers()
{
  raw_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>(
    "/uav1/mavros/setpoint_raw/local", 10);
  trajectory_finished_pub_ = this->create_publisher<std_msgs::msg::Bool>("/trajectory_finished", 10);
  progress_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/trajectory_progress", 10);
  waypoint_reached_pub_ = this->create_publisher<std_msgs::msg::Int32>("/waypoint_reached", 10);
  mission_latch_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/mission_latch_pose", 10);
  waypoint_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    waypoint_goal_status_topic_, 10);
  waypoints_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
    waypoints_status_topic_, 10);
  state_voo_pub_ = this->create_publisher<std_msgs::msg::Int32>(
    "/drone_controller/state_voo", rclcpp::QoS(1).transient_local());
  RCLCPP_INFO(this->get_logger(), "✓ Publisher criado: /uav1/mavros/setpoint_raw/local");
  RCLCPP_INFO(this->get_logger(), "✓ Publisher criado: /trajectory_finished");
  RCLCPP_INFO(this->get_logger(), "✓ Publisher criado: /trajectory_progress");
  RCLCPP_INFO(this->get_logger(), "✓ Publisher criado: /waypoint_reached");
  RCLCPP_INFO(this->get_logger(), "✓ Publisher criado: /mission_latch_pose");
  RCLCPP_INFO(this->get_logger(), "✓ Publisher criado (status): %s", waypoint_goal_status_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "✓ Publisher criado (status): %s", waypoints_status_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "✓ Publisher criado: /drone_controller/state_voo");
}

void DroneControllerCompleto::setup_subscribers()
{
  state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
    "/uav1/mavros/state", 10,
    [this](const mavros_msgs::msg::State::SharedPtr msg) { current_state_ = *msg; });

  extended_state_sub_ = this->create_subscription<mavros_msgs::msg::ExtendedState>(
    "/uav1/mavros/extended_state", 10,
    [this](const mavros_msgs::msg::ExtendedState::SharedPtr msg) {
      last_extended_state_ = *msg;
      last_extended_state_time_ = this->now();
      extended_state_received_ = true;
    });

  waypoints_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
    waypoints_cmd_topic_, 1,
    std::bind(&DroneControllerCompleto::waypoints_callback, this, std::placeholders::_1));

  mission_waypoints_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
    "/mission_waypoints", 1,
    std::bind(&DroneControllerCompleto::mission_waypoints_callback, this, std::placeholders::_1));

  waypoint_goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    waypoint_goal_cmd_topic_, 1,
    std::bind(&DroneControllerCompleto::waypoint_goal_callback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/uav1/mavros/local_position/odom", 10,
    std::bind(&DroneControllerCompleto::odometry_callback, this, std::placeholders::_1));

  yaw_override_sub_ = this->create_subscription<drone_control::msg::YawOverride>(
    "/uav1/yaw_override/cmd", 10,
    std::bind(&DroneControllerCompleto::yaw_override_callback, this, std::placeholders::_1));

  waypoint_goal_4d_sub_ = this->create_subscription<drone_control::msg::Waypoint4D>(
    "/waypoint_goal_4d", 1,
    std::bind(&DroneControllerCompleto::waypoint_goal_4d_callback, this, std::placeholders::_1));

  waypoints_4d_sub_ = this->create_subscription<drone_control::msg::Waypoint4DArray>(
    "/waypoints_4d", 1,
    std::bind(&DroneControllerCompleto::waypoints_4d_callback, this, std::placeholders::_1));

  mission_interrupt_done_sub_ = this->create_subscription<std_msgs::msg::Int32>(
    "/mission_interrupt_done", 10,
    std::bind(&DroneControllerCompleto::mission_interrupt_done_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(),
    "✓ Subscribers criados: /uav1/mavros/state, /uav1/mavros/extended_state, "
    "%s (cmd), /mission_waypoints, %s (cmd), odometria, /uav1/yaw_override/cmd, "
    "/waypoint_goal_4d, /waypoints_4d e /mission_interrupt_done",
    waypoints_cmd_topic_.c_str(), waypoint_goal_cmd_topic_.c_str());
}

void DroneControllerCompleto::setup_services()
{
  mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/uav1/mavros/set_mode");
  arm_client_  = this->create_client<mavros_msgs::srv::CommandBool>("/uav1/mavros/cmd/arming");
  RCLCPP_INFO(this->get_logger(), "✓ Service Clients criados: set_mode e arming");

  RCLCPP_INFO(this->get_logger(), "⏳ Aguardando serviços MAVROS...");
  while (!mode_client_->wait_for_service(1s)) {
    RCLCPP_WARN(this->get_logger(), "⏳ Aguardando /uav1/mavros/set_mode...");
  }
  RCLCPP_INFO(this->get_logger(), "✓ Serviço set_mode disponível");

  while (!arm_client_->wait_for_service(1s)) {
    RCLCPP_WARN(this->get_logger(), "⏳ Aguardando /uav1/mavros/cmd/arming...");
  }
  RCLCPP_INFO(this->get_logger(), "✓ Serviço arming disponível\n");

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(10),
    std::bind(&DroneControllerCompleto::control_loop, this));
  RCLCPP_INFO(this->get_logger(), "✓ Timer criado: 100 Hz (10ms)");

  // ── Monitoring heartbeat timers ─────────────────────────────────────────
  if (monitor_waypoint_goal_rate_hz_ > 0.0) {
    auto period_ms = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double, std::milli>(1000.0 / monitor_waypoint_goal_rate_hz_));
    monitor_waypoint_goal_timer_ = this->create_wall_timer(
      period_ms,
      std::bind(&DroneControllerCompleto::monitor_waypoint_goal_heartbeat, this));
    RCLCPP_INFO(this->get_logger(),
      "✓ Heartbeat (status→%s): %.1f Hz", waypoint_goal_status_topic_.c_str(), monitor_waypoint_goal_rate_hz_);
  }
  if (monitor_waypoints_rate_hz_ > 0.0) {
    auto period_ms = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double, std::milli>(1000.0 / monitor_waypoints_rate_hz_));
    monitor_waypoints_timer_ = this->create_wall_timer(
      period_ms,
      std::bind(&DroneControllerCompleto::monitor_waypoints_heartbeat, this));
    RCLCPP_INFO(this->get_logger(),
      "✓ Heartbeat (status→%s): %.1f Hz", waypoints_status_topic_.c_str(), monitor_waypoints_rate_hz_);
  }
  // ── State-voo periodic republish timer ────────────────────────────────────
  if (publish_state_voo_ && state_voo_pub_rate_hz_ > 0.0) {
    auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / state_voo_pub_rate_hz_));
    state_voo_timer_ = this->create_wall_timer(
      period_ns,
      std::bind(&DroneControllerCompleto::monitor_state_voo_heartbeat, this));
    RCLCPP_INFO(this->get_logger(),
      "✓ Heartbeat /drone_controller/state_voo: %.1f Hz", state_voo_pub_rate_hz_);
  }
}

void DroneControllerCompleto::init_variables()
{
  state_voo_ = 0;
  controlador_ativo_ = false;
  pouso_em_andamento_ = false;
  cycle_count_ = 0;
  offboard_activated_ = false;
  offboard_mode_confirmed_ = false;
  arm_requested_ = false;
  activation_confirmed_ = false;
  takeoff_counter_ = 0;
  takeoff_target_z_ = -1.0;  // -1.0 = sentinel: computed at takeoff-command-receive time
  waypoint_goal_received_ = false;
  last_z_ = 0.0;
  pouso_start_time_set_ = false;
  pouso_start_time_ = this->now();
  disarm_requested_ = false;
  last_waypoint_goal_.pose.position.x = 0.0;
  last_waypoint_goal_.pose.position.y = 0.0;
  last_waypoint_goal_.pose.position.z = config_.hover_altitude;
  trajectory_setpoint_[0] = 0.0;
  trajectory_setpoint_[1] = 0.0;
  trajectory_setpoint_[2] = config_.hover_altitude;
  trajectory_waypoints_.clear();
  trajectory_started_ = false;
  current_waypoint_idx_ = 0;
  waypoint_duration_ = config_.waypoint_duration;
  last_waypoint_reached_idx_ = -1;
  mission_cycle_phase_ = MissionCyclePhase::NONE;
  mission_wp_follow_idx_ = 0;
  last_published_mission_wp_idx_ = -1;
  mission_waypoints_.clear();
  current_z_real_ = 0.0;
  current_x_ned_ = 0.0;
  current_y_ned_ = 0.0;
  current_z_ned_ = 0.0;
  current_vx_ned_ = 0.0;
  current_vy_ned_ = 0.0;
  current_vz_ned_ = 0.0;
  has_latch_pose_ = false;
  yaw_override_enabled_ = false;
  yaw_rate_cmd_ = 0.0;
  yaw_override_timeout_s_ = 0.3;
  hold_valid_ = false;
  hold_x_ned_ = 0.0;
  hold_y_ned_ = 0.0;
  hold_z_ned_ = 0.0;
  last_yaw_cmd_time_ = this->now();
  current_yaw_rad_ = 0.0;
  goal_yaw_rad_ = 0.0;
  using_4d_goal_ = false;
  trajectory_4d_mode_ = false;
  initial_stream_count_ = 0;
  initial_stream_done_ = false;
  post_offboard_stream_count_ = 0;
  post_offboard_stream_done_ = false;
  setpoint_pub_time_initialized_ = false;
  mission_enabled_ = false;
  planner_initialized_ = false;
}

// ============================================================
// LANDING / ACTIVATION HELPERS
// ============================================================

void DroneControllerCompleto::trigger_landing(double z)
{
  // Note: controlador_ativo_ is intentionally NOT set to false here.
  // Initiating a landing must not also initiate a pause of the controller.
  // The trajectory controller is implicitly deactivated by the state_voo_ = 4
  // transition; the takeoff handler resets controlador_ativo_ before the FSM
  // can re-enter state 2 (hover) on the next flight cycle.
  pouso_em_andamento_ = true;
  state_voo_ = 4;
  land_cmd_id_ = cmd_queue_.enqueue(CommandType::LAND, {{"z", std::to_string(z)}});
}

void DroneControllerCompleto::activate_offboard_arm_if_needed()
{
  if (current_state_.armed && current_state_.mode == "OFFBOARD") {
    RCLCPP_INFO(this->get_logger(),
      "🔋 Já OFFBOARD+ARM — reutilizando ativação existente para levantamento.");
    // Skip the full streaming/OFFBOARD/ARM sequence: the FCU is already in
    // OFFBOARD mode and armed, so all activation flags are set to reflect
    // the confirmed state and the FSM jumps straight to takeoff climb.
    initial_stream_done_ = true;
    offboard_activated_ = true;
    offboard_mode_confirmed_ = true;
    post_offboard_stream_done_ = true;
    arm_requested_ = true;
    return;
  }
  // Reset activation flags so handle_state1_takeoff() re-runs the full
  // streaming → OFFBOARD → wait-for-OFFBOARD → ARM sequence for this new takeoff attempt.
  RCLCPP_INFO(this->get_logger(),
    "🔋 Preparando streaming inicial de setpoints para OFFBOARD+ARM...");
  offboard_activated_ = false;
  offboard_mode_confirmed_ = false;
  arm_requested_ = false;
  activation_confirmed_ = false;
  initial_stream_count_ = 0;
  initial_stream_done_ = false;
  post_offboard_stream_count_ = 0;
  post_offboard_stream_done_ = false;
  // ARM and OFFBOARD are NOT requested here.  They will be triggered by
  // handle_state1_takeoff() only after stream_initial_setpoints() has
  // published INITIAL_STREAM_THRESHOLD setpoints to satisfy the PX4 FCU.
}

// ============================================================
// PARAMETER HANDLERS
// ============================================================

bool DroneControllerCompleto::apply_enabled_param(
  const rclcpp::Parameter & p, rcl_interfaces::msg::SetParametersResult & result)
{
  if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
    result.successful = false;
    result.reason = "enabled deve ser bool (true ou false)";
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  bool old_val = enabled_;
  enabled_ = p.as_bool();
  if (old_val != enabled_) {
    if (enabled_) {
      RCLCPP_INFO(this->get_logger(),
        "✅ enabled atualizado: false → true (publicação de setpoints RETOMADA)");
    } else {
      RCLCPP_INFO(this->get_logger(),
        "🚫 enabled atualizado: true → false (publicação de setpoints PAUSADA)");
    }
  }
  return true;
}

bool DroneControllerCompleto::apply_override_active_param(
  const rclcpp::Parameter & p, rcl_interfaces::msg::SetParametersResult & result)
{
  if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
    result.successful = false;
    result.reason = "override_active deve ser bool (true ou false)";
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  bool old_val = override_active_;
  override_active_ = p.as_bool();
  if (old_val != override_active_) {
    if (override_active_) {
      hold_x_ned_ = current_x_ned_;
      hold_y_ned_ = current_y_ned_;
      hold_z_ned_ = current_z_ned_;
      hold_valid_ = true;
      RCLCPP_INFO(this->get_logger(),
        "🔒 override_active: false → true (override externo ATIVO: FSM congelada, publicando hold setpoint X=%.2f Y=%.2f Z=%.2f)",
        hold_x_ned_, hold_y_ned_, hold_z_ned_);
    } else {
      RCLCPP_INFO(this->get_logger(),
        "🔓 override_active: true → false (override externo DESATIVADO: operação normal RETOMADA)");
    }
  }
  return true;
}

rcl_interfaces::msg::SetParametersResult DroneControllerCompleto::onSetParameters(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & p : params) {
    if (p.get_name() == "enabled") {
      if (!apply_enabled_param(p, result)) { return result; }
    } else if (p.get_name() == "override_active") {
      if (!apply_override_active_param(p, result)) { return result; }
    }
  }
  return result;
}

// ============================================================
// FCU SERVICE REQUESTS
// ============================================================

void DroneControllerCompleto::request_offboard()
{
  if (!mode_client_->service_is_ready()) { return; }

  auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  request->custom_mode = "OFFBOARD";

  uint64_t cmd_id = cmd_queue_.enqueue(
    CommandType::SET_MODE_OFFBOARD, {{"mode", "OFFBOARD"}});
  offboard_cmd_id_ = cmd_id;

  std::weak_ptr<DroneControllerCompleto> weak_self(
    std::static_pointer_cast<DroneControllerCompleto>(shared_from_this()));

  auto callback = [weak_self, cmd_id](
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
    auto self = weak_self.lock();
    if (!self) { return; }
    auto result = future.get();
    bool mode_set_success = result && result->mode_sent;
    self->cmd_queue_.confirm(cmd_id, mode_set_success);
    if (mode_set_success) {
      RCLCPP_INFO(self->get_logger(),
        "✅ [ID=%lu] SET_MODE OFFBOARD aceito pelo FCU", cmd_id);
    } else {
      RCLCPP_WARN(self->get_logger(),
        "⚠️ [ID=%lu] SET_MODE OFFBOARD rejeitado pelo FCU", cmd_id);
    }
  };

  mode_client_->async_send_request(request, callback);
  RCLCPP_INFO(this->get_logger(), "📡 [ID=%lu] Solicitando OFFBOARD MODE...", cmd_id);
}

void DroneControllerCompleto::request_arm()
{
  if (!arm_client_->service_is_ready()) { return; }

  auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  request->value = true;

  uint64_t cmd_id = cmd_queue_.enqueue(CommandType::ARM);
  arm_cmd_id_ = cmd_id;

  std::weak_ptr<DroneControllerCompleto> weak_self(
    std::static_pointer_cast<DroneControllerCompleto>(shared_from_this()));

  auto callback = [weak_self, cmd_id](
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
    auto self = weak_self.lock();
    if (!self) { return; }
    auto result = future.get();
    bool arm_confirmed = result && result->success;
    self->cmd_queue_.confirm(cmd_id, arm_confirmed);
    if (arm_confirmed) {
      RCLCPP_INFO(self->get_logger(), "✅ [ID=%lu] ARM confirmado pelo FCU", cmd_id);
    } else {
      RCLCPP_WARN(self->get_logger(), "⚠️ [ID=%lu] ARM rejeitado pelo FCU", cmd_id);
    }
  };

  arm_client_->async_send_request(request, callback);
  RCLCPP_INFO(this->get_logger(), "🔋 [ID=%lu] Solicitando ARM...", cmd_id);
}

void DroneControllerCompleto::request_disarm()
{
  if (!arm_client_->service_is_ready()) { return; }

  auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  request->value = false;

  uint64_t cmd_id = cmd_queue_.enqueue(CommandType::DISARM);
  disarm_cmd_id_ = cmd_id;

  std::weak_ptr<DroneControllerCompleto> weak_self(
    std::static_pointer_cast<DroneControllerCompleto>(shared_from_this()));

  auto callback = [weak_self, cmd_id](
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
    auto self = weak_self.lock();
    if (!self) { return; }
    auto result = future.get();
    bool disarm_confirmed = result && result->success;
    self->cmd_queue_.confirm(cmd_id, disarm_confirmed);
    if (disarm_confirmed) {
      RCLCPP_INFO(self->get_logger(), "✅ [ID=%lu] DISARM confirmado pelo FCU", cmd_id);
    } else {
      RCLCPP_WARN(self->get_logger(), "⚠️ [ID=%lu] DISARM rejeitado pelo FCU", cmd_id);
    }
  };

  arm_client_->async_send_request(request, callback);
  RCLCPP_INFO(this->get_logger(), "🔴 [ID=%lu] Solicitando DISARM...", cmd_id);
}

// ============================================================
// SENSOR CALLBACKS
// ============================================================

void DroneControllerCompleto::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  current_x_ned_ = msg->pose.pose.position.x;
  current_y_ned_ = msg->pose.pose.position.y;
  current_z_ned_ = msg->pose.pose.position.z;
  current_z_real_ = std::abs(current_z_ned_);
  current_vx_ned_ = msg->twist.twist.linear.x;
  current_vy_ned_ = msg->twist.twist.linear.y;
  current_vz_ned_ = msg->twist.twist.linear.z;
  const auto & q = msg->pose.pose.orientation;
  current_yaw_rad_ = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

void DroneControllerCompleto::yaw_override_callback(
  const drone_control::msg::YawOverride::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (msg->enable) {
    if (!yaw_override_enabled_) {
      hold_x_ned_ = current_x_ned_;
      hold_y_ned_ = current_y_ned_;
      hold_z_ned_ = current_z_ned_;
      hold_valid_ = true;
      RCLCPP_INFO(this->get_logger(),
        "🔒 Yaw override ATIVADO: hold X=%.2f Y=%.2f Z=%.2f (NED), yaw_rate=%.3f rad/s",
        hold_x_ned_, hold_y_ned_, hold_z_ned_, static_cast<double>(msg->yaw_rate));
    }
    yaw_override_enabled_ = true;
    yaw_rate_cmd_ = static_cast<double>(msg->yaw_rate);
    if (msg->timeout > 0.0f) {
      yaw_override_timeout_s_ = static_cast<double>(msg->timeout);
    }
  } else {
    if (yaw_override_enabled_) {
      RCLCPP_INFO(this->get_logger(),
        "🔓 Yaw override DESATIVADO: retomando operação normal da FSM.");
    }
    yaw_override_enabled_ = false;
    yaw_rate_cmd_ = 0.0;
  }
  last_yaw_cmd_time_ = this->now();
}

// ============================================================
// WAYPOINT CALLBACK HELPERS (3D)
// ============================================================

void DroneControllerCompleto::handle_landing_waypoint_command(double z)
{
  RCLCPP_WARN(this->get_logger(), "\n🛬🛬🛬 POUSO DETECTADO! Z_final = %.2f m", z);
  trigger_landing(z);
  RCLCPP_WARN(this->get_logger(), "📋 [ID=%lu] Comando LAND enfileirado", *land_cmd_id_);
  RCLCPP_WARN(this->get_logger(),
    "🛬 POUSANDO — delegando descida ao drone_soft_land\n");
}

geometry_msgs::msg::Pose DroneControllerCompleto::sanitize_takeoff_xy(
  const geometry_msgs::msg::Pose & pose)
{
  double xy_dist = std::hypot(pose.position.x, pose.position.y);
  // last_latch_pose_ is sourced from trajectory_waypoints_[...] which has already
  // passed validate_pose() in the waypoint callbacks, so its coordinates are valid.
  if (xy_dist < config_.takeoff_xy_origin_threshold_m && has_latch_pose_) {
    double age = (this->now() - last_latch_pose_time_).seconds();
    if (age < config_.latch_pose_max_age_s) {
      geometry_msgs::msg::Pose sanitized = pose;
      sanitized.position.x = last_latch_pose_.position.x;
      sanitized.position.y = last_latch_pose_.position.y;
      RCLCPP_WARN(this->get_logger(),
        "🛡️ [TAKEOFF XY SANITIZE] XY solicitado (%.3f, %.3f) está a %.3f m da origem "
        "(limiar=%.2f m) – substituído pelo latch pose (%.3f, %.3f) [age=%.1fs < %.1fs]",
        pose.position.x, pose.position.y, xy_dist,
        config_.takeoff_xy_origin_threshold_m,
        sanitized.position.x, sanitized.position.y,
        age, config_.latch_pose_max_age_s);
      return sanitized;
    }
    RCLCPP_WARN(this->get_logger(),
      "🛡️ [TAKEOFF XY SANITIZE] XY solicitado (%.3f, %.3f) próximo à origem, "
      "mas latch pose está desatualizada (age=%.1fs >= %.1fs) – usando XY solicitado",
      pose.position.x, pose.position.y, age, config_.latch_pose_max_age_s);
  } else {
    RCLCPP_DEBUG(this->get_logger(),
      "🛡️ [TAKEOFF XY] Usando XY solicitado: (%.3f, %.3f) [dist_origem=%.3f m]",
      pose.position.x, pose.position.y, xy_dist);
  }
  return pose;
}

double DroneControllerCompleto::compute_lookat_yaw(double x_target, double y_target) const
{
  constexpr double kDeadbandM = 0.05;
  const double dx = x_target - current_x_ned_;
  const double dy = y_target - current_y_ned_;
  if (std::hypot(dx, dy) < kDeadbandM) {
    return current_yaw_rad_;
  }
  return std::atan2(dy, dx);
}

void DroneControllerCompleto::handle_single_takeoff_waypoint_command(
  const geometry_msgs::msg::Pose & pose)
{
  RCLCPP_INFO(this->get_logger(), "\n⬆️ WAYPOINT DE LEVANTAMENTO recebido:");
  RCLCPP_INFO(this->get_logger(), "   Posição: X=%.2f, Y=%.2f, Z=%.2f",
    pose.position.x, pose.position.y, pose.position.z);

  log_takeoff_debug_flags("ANTES");

  // Guard against a near-origin XY that may have been generated by a publisher
  // whose latch pose was not yet available.  sanitize_takeoff_xy() overrides
  // the XY with the last known latch pose when the requested XY is within
  // takeoff_xy_origin_threshold_m of the origin AND the latch pose is fresh.
  const geometry_msgs::msg::Pose safe_pose = sanitize_takeoff_xy(pose);

  // Compute the takeoff target altitude from the Z commanded by the waypoint,
  // clamped to [min_altitude, max_altitude].  This value is fixed here, at
  // command-receive time, so that the target does not chase the rising drone
  // every 10 ms control cycle (which would cause infinite ascent).
  {
    const double z_cmd = safe_pose.position.z;
    takeoff_target_z_ = std::min(
      config_.max_altitude,
      std::max(config_.min_altitude, z_cmd));
    RCLCPP_INFO(this->get_logger(),
      "⬆️ [TAKEOFF TARGET] Z_cmd=%.2fm | clamp[%.2f..%.2f] → Z_alvo=%.2fm (fixo durante subida)",
      z_cmd, config_.min_altitude, config_.max_altitude, takeoff_target_z_);
  }

  // Compute a "look-at" yaw so the drone faces the single target waypoint
  // during takeoff climb and hover.  using_4d_goal_=true enables yaw control
  // in publish_takeoff_climb_setpoint() and handle_state2_hover().
  goal_yaw_rad_ = compute_lookat_yaw(safe_pose.position.x, safe_pose.position.y);
  using_4d_goal_ = true;
  RCLCPP_INFO(this->get_logger(),
    "🧭 [SINGLE WP YAW] dx=%.3f m  dy=%.3f m → yaw_alvo=%.3f rad",
    safe_pose.position.x - current_x_ned_,
    safe_pose.position.y - current_y_ned_,
    goal_yaw_rad_);

  last_waypoint_goal_.pose = safe_pose;
  pouso_em_andamento_ = false;
  controlador_ativo_ = false;
  trajectory_started_ = false;
  trajectory_waypoints_.clear();
  trajectory_yaws_.clear();  // cleared defensively to avoid stale 4D yaws if mode changes
  current_waypoint_idx_ = 0;

  activate_offboard_arm_if_needed();

  takeoff_cmd_id_ = cmd_queue_.enqueue(
    CommandType::TAKEOFF,
    {{"x", std::to_string(safe_pose.position.x)},
     {"y", std::to_string(safe_pose.position.y)},
     {"z", std::to_string(config_.hover_altitude)}});
  RCLCPP_INFO(this->get_logger(),
    "📋 [ID=%lu] Comando TAKEOFF enfileirado", *takeoff_cmd_id_);

  // Publish the sanitized goal so /waypoint_goal monitoring is consistent.
  publish_waypoint_goal_status(
    safe_pose.position.x, safe_pose.position.y, safe_pose.position.z);

  state_voo_ = 1;
  takeoff_counter_ = 0;

  log_takeoff_debug_flags("DEPOIS");
}

void DroneControllerCompleto::log_trajectory_waypoints_3d(
  const std::vector<geometry_msgs::msg::Pose> & poses)
{
  RCLCPP_INFO(this->get_logger(),
    "✈️ WAYPOINTS DE TRAJETÓRIA armazenados: %zu pontos", poses.size());
  for (size_t i = 0; i < poses.size(); i++) {
    RCLCPP_INFO(this->get_logger(),
      "   WP[%zu]: X=%.2f, Y=%.2f, Z=%.2f",
      i, poses[i].position.x, poses[i].position.y, poses[i].position.z);
  }
  RCLCPP_INFO(this->get_logger(), " ");
}

void DroneControllerCompleto::activate_trajectory_in_hover(size_t waypoint_count)
{
  RCLCPP_INFO(this->get_logger(), "\n✅ TRAJETÓRIA ACEITA E ATIVADA! Drone em HOVER pronto!\n");
  if (hover_cmd_id_) {
    cmd_queue_.confirm(*hover_cmd_id_, true);
    RCLCPP_INFO(this->get_logger(),
      "✅ [ID=%lu] HOVER confirmado - iniciando trajetória", *hover_cmd_id_);
    hover_cmd_id_.reset();
  }
  trajectory_cmd_id_ = cmd_queue_.enqueue(
    CommandType::TRAJECTORY,
    {{"waypoints", std::to_string(waypoint_count)}});
  RCLCPP_INFO(this->get_logger(),
    "📋 [ID=%lu] Comando TRAJECTORY enfileirado (%zu WPs)",
    *trajectory_cmd_id_, waypoint_count);
  controlador_ativo_ = true;
  pouso_em_andamento_ = false;
  state_voo_ = 3;
  RCLCPP_INFO(this->get_logger(), "✅ Trajetória ativada - Entrando em ESTADO 3\n");
}

// ============================================================
// WAYPOINT CALLBACKS
// ============================================================

void DroneControllerCompleto::waypoints_callback(
  const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // Guard against receiving our own published monitoring echo.
  if (skip_self_waypoints_count_ > 0) { skip_self_waypoints_count_--; return; }

  trajectory_4d_mode_ = false;

  if (msg->poses.size() < 1) {
    RCLCPP_WARN(this->get_logger(), "❌ Waypoints insuficientes: %zu", msg->poses.size());
    return;
  }

  for (size_t i = 0; i < msg->poses.size(); ++i) {
    if (!validate_pose(msg->poses[i], config_)) {
      RCLCPP_WARN(this->get_logger(),
        "❌ Waypoint[%zu] inválido (NaN/Inf ou fora dos limites físicos) - mensagem ignorada", i);
      return;
    }
  }

  double last_z = msg->poses.back().position.z;

  if (msg->poses.size() == 1 && last_z < config_.land_z_threshold) {
    handle_landing_waypoint_command(last_z);
    return;
  }

  if (msg->poses.size() == 1 && last_z >= config_.land_z_threshold && !is_in_flight()) {
    // Guard: do not start a new takeoff while the drone is still ARMED in state 4
    // (DISARM is pending confirmation from the FCU). If the drone is already
    // disarmed, handle_state4_disarm_reset() performs the reset and returns false
    // so we fall through and accept the new takeoff command immediately.
    if (handle_state4_disarm_reset()) {
      RCLCPP_WARN(this->get_logger(),
        "⚠️ [TAKEOFF] Ignorado: drone ainda armado em estado 4 (DISARM pendente). "
        "Aguardando confirmação de DISARM pelo FCU antes de aceitar novo takeoff.");
      return;
    }
    handle_single_takeoff_waypoint_command(msg->poses[0]);
    return;
  }

  if (msg->poses.size() >= 2 || (msg->poses.size() == 1 && is_in_flight())) {
    if (state_voo_ == 4) {
      RCLCPP_WARN(this->get_logger(),
        "⚠️ Ignorando waypoints de trajetória durante pouso (estado %d)", state_voo_);
      return;
    }

    // Detect landing-descent trajectories published by missao_P_T's pouso
    // sub-process: exactly 2 waypoints where the last one is below the landing
    // threshold.  Their detailed log is suppressed to DEBUG level so the
    // terminal is not flooded during the automated mission cycle.
    const bool is_landing_descent =
      (msg->poses.size() == 2 &&
       msg->poses.back().position.z < config_.land_z_threshold);

    if (is_landing_descent) {
      RCLCPP_DEBUG(this->get_logger(),
        "🔍 Trajetória de descida recebida (z_final=%.2fm) — log detalhado suprimido.",
        msg->poses.back().position.z);
    } else {
      RCLCPP_INFO(this->get_logger(), "🔍 Trajetória (2+ waypoints) recebida");
      RCLCPP_INFO(this->get_logger(), "   state_voo_=%d (esperado 2 para ativar)", state_voo_);
    }

    trajectory_waypoints_ = msg->poses;
    trajectory_yaws_.clear();
    current_waypoint_idx_ = 0;
    trajectory_started_ = false;
    last_waypoint_reached_idx_ = -1;
    last_waypoint_goal_.pose = msg->poses[0];
    // Cancel any ongoing mission interrupt: new external trajectory replaces it.
    mission_cycle_phase_ = MissionCyclePhase::NONE;
    mission_waypoints_.clear();
    mission_wp_follow_idx_ = 0;
    last_published_mission_wp_idx_ = -1;
    // New trajectory: disable mission gate until this trajectory finishes.
    if (mission_enabled_) {
      mission_enabled_ = false;
      if (!is_landing_descent) {
        RCLCPP_INFO(this->get_logger(),
          "🔒 [MISSION] Nova trajetória recebida — mission_enabled_=false. "
          "/mission_waypoints será ignorado durante a execução.");
      }
    }

    if (!is_landing_descent) {
      log_trajectory_waypoints_3d(trajectory_waypoints_);
    }
    publish_waypoints_status();

    if (state_voo_ != 2) {
      if (!is_landing_descent) {
        RCLCPP_INFO(this->get_logger(),
          "⏸️ Trajetória armazenada - Será ativada quando drone chegar em HOVER (ESTADO 2)");
      }
      controlador_ativo_ = false;
      pouso_em_andamento_ = false;
      return;
    }

    activate_trajectory_in_hover(msg->poses.size());
  }
}

// ============================================================
// MISSION WAYPOINTS CALLBACK (/mission_waypoints)
// ============================================================

void DroneControllerCompleto::mission_waypoints_callback(
  const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (msg->poses.empty()) { return; }

  // ── Gate: ignore /mission_waypoints until trajectory is fully complete ───
  // mission_enabled_ is set to true only in finalize_trajectory_complete(),
  // preventing any stale or spurious /mission_waypoints messages from
  // interrupting an ongoing trajectory.
  if (!mission_enabled_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
      "🔒 [MISSION] /mission_waypoints recebido com %zu pose(s) — trajetória ainda em "
      "andamento (mission_enabled_=false). Ignorando até o final da trajetória.",
      msg->poses.size());
    return;
  }

  // ── Return-home: single pose received after trajectory completion ─────────
  // When the supervisor publishes the home waypoint after all mission cycles
  // are done, the drone may be in NONE (hovering after the last FOLLOW_TAKEOFF)
  // or in WAIT_LAND_WP (holding at the last trajectory waypoint).  In both
  // cases a single-pose message that arrives while the trajectory is truly
  // finished should be treated as a navigation target, not as a per-waypoint
  // takeoff command.
  {
    const bool trajectory_complete =
      trajectory_started_ &&
      !trajectory_waypoints_.empty() &&
      current_waypoint_idx_ >= static_cast<int>(trajectory_waypoints_.size());

    if (msg->poses.size() == 1 &&
        trajectory_complete &&
        (mission_cycle_phase_ == MissionCyclePhase::NONE ||
         mission_cycle_phase_ == MissionCyclePhase::WAIT_LAND_WP))
    {
      const char * prev_phase_name   = mission_cycle_phase_name(mission_cycle_phase_);
      return_home_target_            = msg->poses[0];
      mission_cycle_phase_           = MissionCyclePhase::RETURN_HOME;
      mission_waypoints_.clear();
      mission_wp_follow_idx_         = 0;
      last_published_mission_wp_idx_ = -1;
      controlador_ativo_             = true;  // activates handle_state3_trajectory() if drone is hovering in state 2
      RCLCPP_INFO(this->get_logger(),
        "🏠 [RETURN_HOME] Waypoint de retorno à origem recebido "
        "(x=%.2f, y=%.2f, z=%.2f) após trajetória concluída (fase anterior: %s). "
        "Iniciando navegação de retorno.",
        return_home_target_.position.x,
        return_home_target_.position.y,
        return_home_target_.position.z,
        prev_phase_name);
      return;
    }
  }

  // Takeoff waypoint: single pose with z >= land_z_threshold.
  // Re-arms the drone without clearing the stored main trajectory so that
  // handle_state2_hover() can resume STATE 3 at the saved current_waypoint_idx_.
  if (msg->poses.size() == 1 &&
      msg->poses[0].position.z >= config_.land_z_threshold)
  {
    // Accept takeoff only when the mission cycle is in the WAIT_TAKEOFF_WP phase.
    // This phase is set when DISARM is confirmed after the landing, so the
    // takeoff waypoint is accepted even if the landing portion is already done.
    if (mission_cycle_phase_ != MissionCyclePhase::WAIT_TAKEOFF_WP) {
      RCLCPP_WARN(this->get_logger(),
        "⚠️ /mission_waypoints (takeoff) recebido em fase '%s' – ignorado. "
        "Esperado: WAIT_TAKEOFF_WP.",
        mission_cycle_phase_name(mission_cycle_phase_));
      return;
    }

    // Only trigger re-arm when waiting for a new command (state 0).
    // If DISARM is still pending (state 4, armed=true) wait for the FCU.
    if (handle_state4_disarm_reset()) { return; }
    if (state_voo_ != 0) { return; }  // already arming/flying, ignore duplicate

    // Guard against near-origin XY; override with latch pose when fresh.
    const geometry_msgs::msg::Pose safe_pose = sanitize_takeoff_xy(msg->poses[0]);

    {
      const double z_cmd = safe_pose.position.z;
      takeoff_target_z_ = std::min(
        config_.max_altitude,
        std::max(config_.min_altitude, z_cmd));
      RCLCPP_INFO(this->get_logger(),
        "⬆️ [MISSION TAKEOFF TARGET] Z_cmd=%.2fm | clamp[%.2f..%.2f] → Z_alvo=%.2fm (fixo durante subida)",
        z_cmd, config_.min_altitude, config_.max_altitude, takeoff_target_z_);
    }

    last_waypoint_goal_.pose = safe_pose;
    pouso_em_andamento_ = false;
    controlador_ativo_ = false;
    // Intentionally do NOT clear trajectory_waypoints_ / trajectory_started_ /
    // current_waypoint_idx_ — the main trajectory must be preserved so that
    // handle_state2_hover() can resume it after the drone reaches altitude.

    // Advance phase: drone is now climbing; trajectory resumes at z >= MISSION_RESUME_ALTITUDE_M.
    mission_cycle_phase_ = MissionCyclePhase::FOLLOW_TAKEOFF;

    activate_offboard_arm_if_needed();

    takeoff_cmd_id_ = cmd_queue_.enqueue(
      CommandType::TAKEOFF,
      {{"x", std::to_string(safe_pose.position.x)},
       {"y", std::to_string(safe_pose.position.y)},
       {"z", std::to_string(config_.hover_altitude)}});

    // Publish the sanitized goal so /waypoint_goal monitoring is consistent.
    publish_waypoint_goal_status(
      safe_pose.position.x, safe_pose.position.y, safe_pose.position.z);

    state_voo_ = 1;
    takeoff_counter_ = 0;
    RCLCPP_INFO(this->get_logger(),
      "🚀 [MISSION FOLLOW_TAKEOFF] Re-arm/decolagem iniciada (z_alvo=%.2f m). "
      "Retomará trajetória em WP[%d] após z >= %.1f m.",
      takeoff_target_z_, current_waypoint_idx_, MISSION_RESUME_ALTITUDE_M);
    return;
  }

  // Landing waypoints: store for use by handle_mission_interrupt_in_state3().
  // Accept when waiting for landing waypoints (WAIT_LAND_WP) or when the drone
  // is already navigating back to the origin (RETURN_HOME) and the landing
  // publisher fires once the drone arrives at the home position.
  if (mission_cycle_phase_ != MissionCyclePhase::WAIT_LAND_WP &&
      mission_cycle_phase_ != MissionCyclePhase::RETURN_HOME) {
    RCLCPP_WARN(this->get_logger(),
      "⚠️ /mission_waypoints (pouso) recebido em fase '%s' – ignorado. "
      "Esperado: WAIT_LAND_WP ou RETURN_HOME.",
      mission_cycle_phase_name(mission_cycle_phase_));
    return;
  }

  mission_waypoints_ = msg->poses;
  mission_wp_follow_idx_ = 0;
  mission_cycle_phase_ = MissionCyclePhase::FOLLOW_LAND;
  RCLCPP_INFO(this->get_logger(),
    "📍 [MISSION FOLLOW_LAND] %zu waypoints de pouso recebidos em /mission_waypoints.",
    msg->poses.size());
}

// ============================================================
// SHARED WAYPOINT-GOAL HELPERS
// ============================================================

bool DroneControllerCompleto::autopilot_indicates_landing() const
{
  // Only trust if we have ever received the message
  if (!extended_state_received_) { return false; }

  // Only trust if the message is fresh (within 1 second)
  const double age_s = (this->now() - last_extended_state_time_).seconds();
  if (age_s > 1.0) { return false; }

  // Only trigger when the drone is armed (avoids false positives on the ground)
  if (!current_state_.armed) { return false; }

  // Trigger when autopilot reports LANDING or LANDED
  // LANDED_STATE_LANDING (4) = autopilot is actively descending to land
  // LANDED_STATE_ON_GROUND (1) = autopilot confirms vehicle is on the ground
  const uint8_t ls = last_extended_state_.landed_state;
  return (ls == mavros_msgs::msg::ExtendedState::LANDED_STATE_LANDING ||
          ls == mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND);
}

bool DroneControllerCompleto::check_landing_in_flight(double z)
{
  if ((state_voo_ == 2 || state_voo_ == 3) && autopilot_indicates_landing()) {
    trigger_landing(z);
    const uint8_t ls = last_extended_state_.landed_state;
    const bool is_landing = (ls == mavros_msgs::msg::ExtendedState::LANDED_STATE_LANDING);
    RCLCPP_WARN(this->get_logger(),
      "🛬 [ID=%lu] POUSO DETECTADO (autopiloto)! landed_state=%d (%s), Z = %.2f m - Comando LAND enfileirado",
      *land_cmd_id_,
      static_cast<int>(ls),
      is_landing ? "LANDING" : "ON_GROUND",
      z);
    return true;
  }
  return false;
}


void DroneControllerCompleto::log_takeoff_debug_flags(const char * tag)
{
  RCLCPP_INFO(this->get_logger(),
    "🔍 DEBUG FLAGS %s: state_voo_=%d offboard_activated_=%d "
    "offboard_mode_confirmed_=%d arm_requested_=%d activation_confirmed_=%d "
    "disarm_requested_=%d takeoff_cmd_id_=%s takeoff_counter_=%d current_waypoint_idx_=%d",
    tag, state_voo_,
    static_cast<int>(offboard_activated_),
    static_cast<int>(offboard_mode_confirmed_),
    static_cast<int>(arm_requested_),
    static_cast<int>(activation_confirmed_),
    static_cast<int>(disarm_requested_),
    takeoff_cmd_id_ ? "set" : "empty",
    takeoff_counter_, current_waypoint_idx_);
}

// ============================================================
// WAYPOINT-GOAL CALLBACKS
// ============================================================

void DroneControllerCompleto::waypoint_goal_callback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // Guard against receiving our own published monitoring echo.
  if (skip_self_waypoint_goal_count_ > 0) { skip_self_waypoint_goal_count_--; return; }

  using_4d_goal_ = false;

  if (!validate_waypoint(*msg, config_)) {
    RCLCPP_WARN(this->get_logger(),
      "❌ waypoint_goal inválido (NaN/Inf ou fora dos limites físicos) - ignorado");
    return;
  }

  double x = msg->pose.position.x;
  double y = msg->pose.position.y;
  double z = msg->pose.position.z;
  last_z_ = z;

  if (check_landing_in_flight(z)) { return; }
  if (handle_state4_disarm_reset()) { return; }

  if (state_voo_ == 3) {
    trajectory_setpoint_[0] = x;
    trajectory_setpoint_[1] = y;
    trajectory_setpoint_[2] = z;
    controlador_ativo_ = true;
    pouso_em_andamento_ = false;
    publish_waypoint_goal_status(x, y, z);
    return;
  }

  RCLCPP_INFO(this->get_logger(), "\n📍 NOVO WAYPOINT RECEBIDO: X=%.2f, Y=%.2f, Z=%.2f", x, y, z);

  last_waypoint_goal_ = *msg;
  waypoint_goal_received_ = true;
  controlador_ativo_ = true;
  pouso_em_andamento_ = false;
  publish_waypoint_goal_status(x, y, z);
}

void DroneControllerCompleto::waypoint_goal_4d_callback(
  const drone_control::msg::Waypoint4D::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  geometry_msgs::msg::PoseStamped ps;
  ps.pose = msg->pose;
  if (!validate_waypoint(ps, config_)) {
    RCLCPP_WARN(this->get_logger(),
      "❌ waypoint_goal_4d inválido (NaN/Inf ou fora dos limites físicos) - ignorado");
    return;
  }

  if (std::isnan(static_cast<double>(msg->yaw))) {
    goal_yaw_rad_ = current_yaw_rad_;
    RCLCPP_INFO(this->get_logger(),
      "📍 4D Waypoint: yaw=NaN → usando yaw atual (%.3f rad)", goal_yaw_rad_);
  } else {
    double raw_yaw = static_cast<double>(msg->yaw);
    goal_yaw_rad_ = std::atan2(std::sin(raw_yaw), std::cos(raw_yaw));
    RCLCPP_INFO(this->get_logger(),
      "📍 4D Waypoint: yaw=%.3f rad (normalizado para %.3f rad)", raw_yaw, goal_yaw_rad_);
  }

  using_4d_goal_ = true;
  trajectory_4d_mode_ = false;

  auto pose_stamped = std::make_shared<geometry_msgs::msg::PoseStamped>();
  pose_stamped->pose = msg->pose;
  pose_stamped->header.stamp = this->now();
  pose_stamped->header.frame_id = "map";

  double x = pose_stamped->pose.position.x;
  double y = pose_stamped->pose.position.y;
  double z = pose_stamped->pose.position.z;
  last_z_ = z;

  if (check_landing_in_flight(z)) { return; }

  if (handle_state4_disarm_reset()) { return; }

  if (state_voo_ == 3) {
    trajectory_setpoint_[0] = x;
    trajectory_setpoint_[1] = y;
    trajectory_setpoint_[2] = z;
    controlador_ativo_ = true;
    pouso_em_andamento_ = false;
    publish_waypoint_goal_status(x, y, z);
    return;
  }

  RCLCPP_INFO(this->get_logger(),
    "\n📍 4D WAYPOINT RECEBIDO: X=%.2f, Y=%.2f, Z=%.2f, yaw=%.3f rad",
    x, y, z, goal_yaw_rad_);

  last_waypoint_goal_ = *pose_stamped;
  waypoint_goal_received_ = true;
  controlador_ativo_ = true;
  pouso_em_andamento_ = false;
  publish_waypoint_goal_status(x, y, z);
}

void DroneControllerCompleto::waypoints_4d_callback(
  const drone_control::msg::Waypoint4DArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (msg->waypoints.size() < 1) {
    RCLCPP_WARN(this->get_logger(),
      "❌ Waypoints4D insuficientes: %zu", msg->waypoints.size());
    return;
  }

  for (size_t i = 0; i < msg->waypoints.size(); ++i) {
    geometry_msgs::msg::PoseStamped ps;
    ps.pose = msg->waypoints[i].pose;
    if (!validate_waypoint(ps, config_)) {
      RCLCPP_WARN(this->get_logger(),
        "❌ Waypoint4D[%zu] inválido (NaN/Inf ou fora dos limites físicos) - mensagem ignorada", i);
      return;
    }
  }

  std::vector<geometry_msgs::msg::Pose> poses;
  std::vector<double> yaws;
  poses.reserve(msg->waypoints.size());
  yaws.reserve(msg->waypoints.size());
  for (const auto & wp : msg->waypoints) {
    poses.push_back(wp.pose);
    if (std::isnan(static_cast<double>(wp.yaw))) {
      yaws.push_back(current_yaw_rad_);
    } else {
      double raw_yaw = static_cast<double>(wp.yaw);
      yaws.push_back(std::atan2(std::sin(raw_yaw), std::cos(raw_yaw)));
    }
  }

  double last_z = poses.back().position.z;

  if (msg->waypoints.size() == 1 && last_z < config_.land_z_threshold) {
    RCLCPP_WARN(this->get_logger(), "\n🛬🛬🛬 4D POUSO DETECTADO! Z_final = %.2f m", last_z);
    trigger_landing(last_z);
    trajectory_4d_mode_ = false;
    return;
  }

  if (msg->waypoints.size() == 1) {
    // Guard: do not start a new 4D takeoff while the drone is still ARMED in
    // state 4 (DISARM pending). If the drone is already disarmed,
    // handle_state4_disarm_reset() performs the reset and returns false so we
    // fall through and accept the new takeoff command immediately.
    if (handle_state4_disarm_reset()) {
      RCLCPP_WARN(this->get_logger(),
        "⚠️ [4D TAKEOFF] Ignorado: drone ainda armado em estado 4 (DISARM pendente). "
        "Aguardando confirmação de DISARM pelo FCU antes de aceitar novo takeoff.");
      return;
    }
    // Guard: warn if a new takeoff arrives while the drone is already in flight.
    // Hipóteses: operador enviou takeoff durante voo (estado 1/2/3) por engano,
    // ou o sistema de missão reiniciou sem esperar o pouso anterior completar.
    if (is_in_flight()) {
      RCLCPP_WARN(this->get_logger(),
        "⚠️ [4D TAKEOFF] Novo takeoff recebido enquanto drone em ESTADO %d (em voo). "
        "Estado inconsistente — reiniciando ciclo de decolagem. "
        "(Hipótese: missão reenviada antes do pouso ou crash de estado da FSM.)",
        state_voo_);
    }

    RCLCPP_INFO(this->get_logger(),
      "\n⬆️ 4D WAYPOINT DE LEVANTAMENTO recebido: X=%.2f Y=%.2f Z=%.2f yaw=%.3f rad",
      poses[0].position.x, poses[0].position.y, poses[0].position.z, yaws[0]);

    // Compute and fix the takeoff target altitude at command-receive time.
    // The value is kept fixed throughout the climb (see
    // handle_single_takeoff_waypoint_command for the full rationale).
    // sanitize_takeoff_xy() does not alter Z, so poses[0].position.z is the
    // commanded altitude — clamp it to [min_altitude, max_altitude].
    {
      const double z_cmd = poses[0].position.z;
      takeoff_target_z_ = std::min(
        config_.max_altitude,
        std::max(config_.min_altitude, z_cmd));
      RCLCPP_INFO(this->get_logger(),
        "⬆️ [4D TAKEOFF TARGET] Z_cmd=%.2fm | clamp[%.2f..%.2f] → Z_alvo=%.2fm (fixo durante subida)",
        z_cmd, config_.min_altitude, config_.max_altitude, takeoff_target_z_);
    }

    goal_yaw_rad_ = yaws[0];
    using_4d_goal_ = true;
    trajectory_4d_mode_ = false;

    // Guard against near-origin XY; override with latch pose when fresh.
    geometry_msgs::msg::Pose safe_pose_4d = sanitize_takeoff_xy(poses[0]);

    geometry_msgs::msg::PoseStamped ps;
    ps.pose = safe_pose_4d;
    last_waypoint_goal_ = ps;

    pouso_em_andamento_ = false;
    controlador_ativo_ = false;
    trajectory_started_ = false;
    trajectory_waypoints_.clear();
    trajectory_yaws_.clear();
    current_waypoint_idx_ = 0;

    activate_offboard_arm_if_needed();
    takeoff_cmd_id_ = cmd_queue_.enqueue(
      CommandType::TAKEOFF,
      {{"x", std::to_string(safe_pose_4d.position.x)},
       {"y", std::to_string(safe_pose_4d.position.y)},
       {"z", std::to_string(config_.hover_altitude)}});
    state_voo_ = 1;
    takeoff_counter_ = 0;
    return;
  }

  if (state_voo_ == 4) {
    RCLCPP_WARN(this->get_logger(),
      "⚠️ Ignorando trajetória 4D durante pouso (estado %d)", state_voo_);
    return;
  }

  trajectory_waypoints_ = poses;
  trajectory_yaws_ = yaws;
  trajectory_4d_mode_ = true;
  using_4d_goal_ = false;
  current_waypoint_idx_ = 0;
  trajectory_started_ = false;
  last_waypoint_reached_idx_ = -1;

  geometry_msgs::msg::PoseStamped ps;
  ps.pose = poses[0];
  last_waypoint_goal_ = ps;

  RCLCPP_INFO(this->get_logger(),
    "✈️ 4D WAYPOINTS DE TRAJETÓRIA armazenados: %zu pontos", msg->waypoints.size());
  for (size_t i = 0; i < msg->waypoints.size(); i++) {
    RCLCPP_INFO(this->get_logger(),
      "   WP4D[%zu]: X=%.2f Y=%.2f Z=%.2f yaw=%.3f rad",
      i, poses[i].position.x, poses[i].position.y, poses[i].position.z, yaws[i]);
  }

  publish_waypoints_status();

  if (state_voo_ != 2) {
    RCLCPP_INFO(this->get_logger(),
      "⏸️ Trajetória 4D armazenada - Será ativada quando drone chegar em HOVER (ESTADO 2)");
    controlador_ativo_ = false;
    pouso_em_andamento_ = false;
    return;
  }

  activate_trajectory_in_hover(msg->waypoints.size());
}

// ============================================================
// MONITORING PUBLISHERS (/waypoint_goal, /waypoints)
// ============================================================

void DroneControllerCompleto::publish_waypoint_goal_status(double x, double y, double z)
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "map";
  msg.pose.position.x = x;
  msg.pose.position.y = y;
  msg.pose.position.z = z;
  // Guard against receiving our own echo only when cmd and status share the same topic.
  if (waypoint_goal_cmd_topic_ == waypoint_goal_status_topic_) { skip_self_waypoint_goal_count_++; }
  waypoint_goal_pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(),
    "📢 [%s] Publicado: X=%.2f, Y=%.2f, Z=%.2f", waypoint_goal_status_topic_.c_str(), x, y, z);
}

void DroneControllerCompleto::publish_waypoints_status()
{
  if (trajectory_waypoints_.empty()) { return; }
  geometry_msgs::msg::PoseArray msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "map";
  msg.poses = trajectory_waypoints_;
  // Guard against receiving our own echo only when cmd and status share the same topic.
  if (waypoints_cmd_topic_ == waypoints_status_topic_) { skip_self_waypoints_count_++; }
  waypoints_pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(),
    "📢 [%s] Publicando %zu waypoints de trajetória",
    waypoints_status_topic_.c_str(), trajectory_waypoints_.size());
}

void DroneControllerCompleto::monitor_waypoint_goal_heartbeat()
{
  std::lock_guard<std::mutex> lock(mutex_);

  // If configured to publish only when active, suppress when there is no active goal/flight.
  if (monitor_publish_only_when_active_ && !waypoint_goal_received_) { return; }

  double x = 0.0, y = 0.0, z = 0.0;

  // Source priority:
  // 1) FOLLOW_LAND phase with mission waypoints → current mission landing waypoint
  if (mission_cycle_phase_ == MissionCyclePhase::FOLLOW_LAND && !mission_waypoints_.empty()) {
    int idx = mission_wp_follow_idx_;
    if (idx < 0) { idx = 0; }
    if (static_cast<size_t>(idx) >= mission_waypoints_.size()) {
      idx = static_cast<int>(mission_waypoints_.size()) - 1;
    }
    const auto & p = mission_waypoints_[idx].position;
    x = p.x; y = p.y; z = p.z;
  }
  // 2) Trajectory active with waypoints → current trajectory waypoint (clamped)
  else if (state_voo_ == 3 && !trajectory_waypoints_.empty()) {
    int idx = current_waypoint_idx_;
    if (idx < 0) { idx = 0; }
    if (static_cast<size_t>(idx) >= trajectory_waypoints_.size()) {
      idx = static_cast<int>(trajectory_waypoints_.size()) - 1;
    }
    const auto & p = trajectory_waypoints_[idx].position;
    x = p.x; y = p.y; z = p.z;
  }
  // 3) Fall back to the last received waypoint goal
  else {
    const auto & p = last_waypoint_goal_.pose.position;
    x = p.x; y = p.y; z = p.z;
  }

  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "map";
  msg.pose.position.x = x;
  msg.pose.position.y = y;
  msg.pose.position.z = z;
  // Guard against receiving our own echo only when cmd and status share the same topic.
  if (waypoint_goal_cmd_topic_ == waypoint_goal_status_topic_) { skip_self_waypoint_goal_count_++; }
  waypoint_goal_pub_->publish(msg);
  RCLCPP_DEBUG(this->get_logger(),
    "[heartbeat %s] X=%.2f Y=%.2f Z=%.2f", waypoint_goal_status_topic_.c_str(), x, y, z);
}

void DroneControllerCompleto::monitor_waypoints_heartbeat()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (trajectory_waypoints_.empty()) { return; }
  if (monitor_publish_only_when_active_ && !is_in_flight()) { return; }

  geometry_msgs::msg::PoseArray msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "map";
  msg.poses = trajectory_waypoints_;
  // Guard against receiving our own echo only when cmd and status share the same topic.
  if (waypoints_cmd_topic_ == waypoints_status_topic_) { skip_self_waypoints_count_++; }
  waypoints_pub_->publish(msg);
  RCLCPP_DEBUG(this->get_logger(),
    "[heartbeat %s] %zu waypoints", waypoints_status_topic_.c_str(), trajectory_waypoints_.size());
}

void DroneControllerCompleto::monitor_state_voo_heartbeat()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!publish_state_voo_) { return; }
  std_msgs::msg::Int32 msg;
  msg.data = state_voo_;
  state_voo_pub_->publish(msg);
  RCLCPP_DEBUG(this->get_logger(),
    "[heartbeat /drone_controller/state_voo] state=%d", state_voo_);
}

void DroneControllerCompleto::publish_state_voo_on_change()
{
  // NOTE: This function is always called from control_loop(), which already
  // holds mutex_. Do NOT acquire the mutex here to avoid a deadlock.
  if (!publish_state_voo_) { return; }
  if (state_voo_ == last_published_state_voo_) { return; }
  std_msgs::msg::Int32 msg;
  msg.data = state_voo_;
  state_voo_pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(),
    "📡 [/drone_controller/state_voo] state_voo_ mudou: %d → %d"
    " (0=aguardando, 1=decolagem, 2=hover, 3=trajetória, 4=pouso)",
    last_published_state_voo_, state_voo_);
  last_published_state_voo_ = state_voo_;
}

// ============================================================
// SETPOINT PUBLISHING
// ============================================================

void DroneControllerCompleto::publishPositionTarget(
  double x, double y, double z, double yaw_rate, uint16_t type_mask)
{
  mavros_msgs::msg::PositionTarget pt;
  pt.header.stamp = this->now();
  pt.header.frame_id = "map";
  pt.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
  pt.type_mask = type_mask;
  pt.position.x = static_cast<float>(x);
  pt.position.y = static_cast<float>(y);
  pt.position.z = static_cast<float>(z);
  pt.yaw_rate = static_cast<float>(yaw_rate);
  raw_pub_->publish(pt);
  // Watchdog: record time of last real publish
  last_setpoint_pub_time_ = pt.header.stamp;
  setpoint_pub_time_initialized_ = true;
}

void DroneControllerCompleto::publishPositionTargetWithYaw(
  double x, double y, double z, double yaw_rad)
{
  mavros_msgs::msg::PositionTarget pt;
  pt.header.stamp = this->now();
  pt.header.frame_id = "map";
  pt.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
  pt.type_mask = MASK_POS_YAW;
  pt.position.x = static_cast<float>(x);
  pt.position.y = static_cast<float>(y);
  pt.position.z = static_cast<float>(z);
  pt.yaw = static_cast<float>(yaw_rad);
  raw_pub_->publish(pt);
  // Watchdog: record time of last real publish
  last_setpoint_pub_time_ = pt.header.stamp;
  setpoint_pub_time_initialized_ = true;
}

void DroneControllerCompleto::publishPositionTargetWithVelocityAndYaw(
  double x, double y, double z,
  double vx, double vy, double vz,
  double yaw_rad)
{
  mavros_msgs::msg::PositionTarget pt;
  pt.header.stamp = this->now();
  pt.header.frame_id = "map";
  pt.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
  pt.type_mask = MASK_POS_VEL_YAW;
  pt.position.x = static_cast<float>(x);
  pt.position.y = static_cast<float>(y);
  pt.position.z = static_cast<float>(z);
  pt.velocity.x = static_cast<float>(vx);
  pt.velocity.y = static_cast<float>(vy);
  pt.velocity.z = static_cast<float>(vz);
  pt.yaw = static_cast<float>(yaw_rad);
  raw_pub_->publish(pt);
  // Watchdog: record time of last real publish
  last_setpoint_pub_time_ = pt.header.stamp;
  setpoint_pub_time_initialized_ = true;
}

// ============================================================
// WATCHDOG HOLD SETPOINT
// ============================================================

void DroneControllerCompleto::publish_hold_setpoint()
{
  // Use captured hold position if valid; otherwise fall back to current odometry.
  if (hold_valid_) {
    publishPositionTarget(hold_x_ned_, hold_y_ned_, hold_z_ned_, 0.0, MASK_POS_ONLY);
  } else {
    publishPositionTarget(current_x_ned_, current_y_ned_, current_z_ned_, 0.0, MASK_POS_ONLY);
  }
}

// ============================================================
// MAIN CONTROL LOOP
// ============================================================

void DroneControllerCompleto::control_loop()
{
  std::lock_guard<std::mutex> lock(mutex_);

  cycle_count_++;

  const auto now = this->now();

  // Initialize watchdog timestamp on first iteration so it doesn't fire immediately.
  if (!setpoint_pub_time_initialized_) {
    last_setpoint_pub_time_ = now;
    setpoint_pub_time_initialized_ = true;
  }

  if (!enabled_) {
    // FSM is paused, but we must keep the MAVROS setpoint stream alive (≥20 Hz)
    // to prevent PX4 from dropping OFFBOARD mode.
    if ((now - last_setpoint_pub_time_).seconds() >= MAX_SETPOINT_SILENCE_S) {
      publish_hold_setpoint();
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "⏱️ Watchdog: enabled=false, publicando hold setpoint (>=20 Hz)");
    }
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "🚫 Controller desabilitado (enabled=false): FSM pausada, mantendo stream mínimo (>=20Hz)");
    return;
  }

  // Watchdog: yaw override timeout
  if (yaw_override_enabled_) {
    double since_last = (this->now() - last_yaw_cmd_time_).seconds();
    if (since_last > yaw_override_timeout_s_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "⚠️ Yaw override timeout (%.2fs sem cmd): desativando override e zerando yaw_rate.",
        since_last);
      yaw_override_enabled_ = false;
      yaw_rate_cmd_ = 0.0;
    }
  }

  // Yaw override ativo: FSM congelada, publica hold + yaw_rate
  if (yaw_override_enabled_ && hold_valid_) {
    publishPositionTarget(hold_x_ned_, hold_y_ned_, hold_z_ned_, yaw_rate_cmd_, MASK_POS_YAWRATE);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "🔒 Yaw override ativo: hold X=%.2f Y=%.2f Z=%.2f  yaw_rate=%.3f rad/s",
      hold_x_ned_, hold_y_ned_, hold_z_ned_, yaw_rate_cmd_);
    return;
  }

  // Override externo ativo: FSM congelada, mas continua publicando hold setpoint
  if (override_active_) {
    if (hold_valid_) {
      publishPositionTarget(hold_x_ned_, hold_y_ned_, hold_z_ned_, 0.0, MASK_POS_ONLY);
    } else {
      publishPositionTarget(current_x_ned_, current_y_ned_, current_z_ned_, 0.0, MASK_POS_ONLY);
    }
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "🔒 Override externo ativo (override_active=true): FSM congelada, publicando hold setpoint");
    return;
  }

  // Verificação periódica de timeouts e salvamento de log (~10 segundos)
  if (cycle_count_ % 1000 == 0) {
    auto timed_out = cmd_queue_.check_timeouts(config_.command_timeout);
    for (auto id : timed_out) {
      RCLCPP_WARN(this->get_logger(),
        "⏰ [ID=%lu] Comando TIMEOUT! (>%.0f s sem confirmação)", id, config_.command_timeout);
    }
    cmd_queue_.save_log("/tmp/drone_commands.log");
  }

  // Despacha para o método do estado atual
  switch (state_voo_) {
    case 0: handle_state0_wait_waypoint(); break;
    case 1: handle_state1_takeoff();       break;
    case 2: handle_state2_hover();         break;
    case 3: handle_state3_trajectory();    break;
    case 4: handle_state4_landing();       break;
    default:
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "❌ Estado FSM inválido: %d", state_voo_);
      break;
  }

  // Publish state_voo_ immediately whenever it changes.
  publish_state_voo_on_change();
}


}  // namespace drone_control
