#include "my_drone_controller/drone_controller_completo.hpp"

#include <cmath>

namespace drone_control {

// ============================================================
// FSM STATE 3 — TRAJETÓRIA (sub-routines)
// ============================================================

bool DroneControllerCompleto::detect_and_handle_landing_in_trajectory()
{
  if (!autopilot_indicates_landing()) { return false; }

  const uint8_t ls = last_extended_state_.landed_state;
  const bool is_landing = (ls == mavros_msgs::msg::ExtendedState::LANDED_STATE_LANDING);
  RCLCPP_WARN(this->get_logger(),
    "\n🛬🛬🛬 POUSO DETECTADO DURANTE TRAJETÓRIA (autopiloto)! landed_state=%d (%s), Z = %.2f m",
    static_cast<int>(ls),
    is_landing ? "LANDING" : "ON_GROUND",
    current_z_real_);

  if (trajectory_cmd_id_) {
    cmd_queue_.confirm(*trajectory_cmd_id_, false);
    RCLCPP_WARN(this->get_logger(),
      "⚠️ [ID=%lu] TRAJECTORY interrompida por pouso", *trajectory_cmd_id_);
    trajectory_cmd_id_.reset();
  }

  if (!land_cmd_id_) {
    land_cmd_id_ = cmd_queue_.enqueue(
      CommandType::LAND, {{"z", std::to_string(current_z_real_)}});
    RCLCPP_WARN(this->get_logger(),
      "📋 [ID=%lu] Comando LAND enfileirado (pouso durante trajetória)", *land_cmd_id_);
  }

  pouso_em_andamento_ = true;
  // Note: controlador_ativo_ is intentionally NOT set to false here.
  // Landing must not also pause the controller; the trajectory controller is
  // implicitly stopped by the state_voo_ = 4 transition below.
  state_voo_ = 4;
  return true;
}

bool DroneControllerCompleto::initialize_trajectory()
{
  if (trajectory_started_) { return true; }

  if (trajectory_waypoints_.empty()) {
    RCLCPP_WARN(this->get_logger(), "⚠️ Nenhum waypoint armazenado, voltando para ESTADO 2");
    state_voo_ = 2;
    return false;
  }

  trajectory_start_time_ = this->now();
  trajectory_started_ = true;
  current_waypoint_idx_ = 0;
  planner_initialized_ = false;

  RCLCPP_INFO(this->get_logger(),
    "✈️ Trajetória iniciada! %zu waypoints | %.1f segundos cada",
    trajectory_waypoints_.size(), waypoint_duration_);

  // ── Initialize TrajectoryPlanner_codegen and Drone_codegen ───────────────
  int n = static_cast<int>(trajectory_waypoints_.size()) - 1;
  if (n >= 1) {
    // Build flat waypoints vector: [X0..Xn, Y0..Yn, Z0..Zn]
    planner_.waypoints.resize(3 * (n + 1));
    for (int i = 0; i <= n; ++i) {
      planner_.waypoints[i]               = trajectory_waypoints_[i].position.x;
      planner_.waypoints[(n + 1) + i]     = trajectory_waypoints_[i].position.y;
      planner_.waypoints[2 * (n + 1) + i] = trajectory_waypoints_[i].position.z;
    }
    planner_.segmentTimes.assign(static_cast<size_t>(n), waypoint_duration_);
    planner_.numSegments = n;
    planner_.init();

    drone_ctrl_.init();
    planner_initialized_ = true;

    RCLCPP_INFO(this->get_logger(),
      "🚀 [PLANNER] TrajectoryPlanner_codegen inicializado: %d segmentos, %.1f s/segmento",
      n, waypoint_duration_);
    RCLCPP_INFO(this->get_logger(),
      "🎮 [CTRL] Drone_codegen inicializado. Controle de posição contínuo ativo.");
    for (int i = 0; i <= n; ++i) {
      RCLCPP_INFO(this->get_logger(),
        "   WP[%d]: X=%.2f  Y=%.2f  Z=%.2f",
        i, trajectory_waypoints_[i].position.x,
        trajectory_waypoints_[i].position.y,
        trajectory_waypoints_[i].position.z);
    }
  } else {
    RCLCPP_WARN(this->get_logger(),
      "⚠️ [PLANNER] Trajetória tem apenas 1 waypoint — usando setpoint de posição simples.");
  }

  // Publish the first waypoint as the active goal for monitoring.
  const auto & first_wp = trajectory_waypoints_[0];
  publish_waypoint_goal_status(first_wp.position.x, first_wp.position.y, first_wp.position.z);

  return true;
}

double DroneControllerCompleto::compute_yaw_for_trajectory_waypoint(int idx, bool at_last_wp)
{
  const auto & wp = trajectory_waypoints_[idx];
  double yaw_follow = 0.0;

  if (at_last_wp) {
    if (!at_last_waypoint_yaw_fixed_) {
      double dx = wp.position.x - current_x_ned_;
      double dy = wp.position.y - current_y_ned_;
      yaw_follow = std::atan2(dy, dx);
      final_waypoint_yaw_ = yaw_follow;
      at_last_waypoint_yaw_fixed_ = true;
    } else {
      yaw_follow = final_waypoint_yaw_;
    }
  } else {
    double dx = wp.position.x - current_x_ned_;
    double dy = wp.position.y - current_y_ned_;
    yaw_follow = std::atan2(dy, dx);
    final_waypoint_yaw_ = yaw_follow;
    at_last_waypoint_yaw_fixed_ = false;
  }

  return yaw_follow;
}

void DroneControllerCompleto::publish_trajectory_waypoint_setpoint(int idx)
{
  const auto & wp = trajectory_waypoints_[idx];
  size_t last_idx = trajectory_waypoints_.size() - 1;
  bool at_last_wp = (static_cast<size_t>(idx) == last_idx);

  if (trajectory_4d_mode_ && at_last_wp &&
      static_cast<size_t>(idx) < trajectory_yaws_.size()) {
    publishPositionTargetWithYaw(
      wp.position.x, wp.position.y, wp.position.z, trajectory_yaws_[idx]);
  } else {
    double yaw_follow = compute_yaw_for_trajectory_waypoint(idx, at_last_wp);
    publishPositionTargetWithYaw(wp.position.x, wp.position.y, wp.position.z, yaw_follow);
  }
}

void DroneControllerCompleto::log_trajectory_progress(int idx)
{
  const auto & wp = trajectory_waypoints_[idx];
  double progress_pct = (trajectory_waypoints_.size() > 1)
    ? (static_cast<double>(idx) / static_cast<double>(trajectory_waypoints_.size() - 1)) * 100.0
    : 100.0;

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
    "📡 Trajetória em execução | WP[%d/%zu]: X=%.2fm, Y=%.2fm, Z=%.2fm (Z_real=%.2f) | %.1f%% concluído",
    idx, trajectory_waypoints_.size() - 1,
    wp.position.x, wp.position.y, wp.position.z,
    current_z_real_, progress_pct);
}

void DroneControllerCompleto::finalize_trajectory_complete()
{
  if (trajectory_cmd_id_) {
    cmd_queue_.confirm(*trajectory_cmd_id_, true);
    RCLCPP_INFO(this->get_logger(),
      "✅ [ID=%lu] TRAJECTORY confirmada - todos os waypoints visitados", *trajectory_cmd_id_);
    trajectory_cmd_id_.reset();
  } else {
    RCLCPP_WARN(this->get_logger(),
      "⚠️ finalize_trajectory_complete chamado sem trajectory_cmd_id_ — "
      "publicando /trajectory_finished e /trajectory_progress mesmo assim.");
  }

  std_msgs::msg::Bool done_msg;
  done_msg.data = true;
  trajectory_finished_pub_->publish(done_msg);

  std_msgs::msg::Float32 progress_msg;
  progress_msg.data = 100.0;
  progress_publisher_->publish(progress_msg);

  // Update hover position to the last trajectory waypoint so the drone holds
  // at the final position when returning to HOVER (state 2).
  if (!trajectory_waypoints_.empty()) {
    last_waypoint_goal_.header.stamp = this->now();
    last_waypoint_goal_.header.frame_id = "map";
    last_waypoint_goal_.pose = trajectory_waypoints_.back();
  }

  RCLCPP_INFO(this->get_logger(),
    "📢 Trajetória terminada! Publicado /trajectory_finished = true");
  RCLCPP_INFO(this->get_logger(),
    "🛸 [HOVER] Trajetória concluída — drone em hover. Aguardando novos /waypoints.");
}

void DroneControllerCompleto::handle_mission_interrupt_in_state3()
{
  // Hold at the waypoint that was just reached while waiting for mission waypoints.
  if (mission_waypoints_.empty()) {
    if (mission_cycle_phase_ == MissionCyclePhase::RETURN_HOME) {
      // Navigate toward the home position published by the supervisor.
      // Yaw is held at 0.0 (north-facing) during the return; the supervisor
      // home pose does not encode a meaningful orientation so north is used
      // as a stable default.
      publishPositionTargetWithYaw(
        return_home_target_.position.x,
        return_home_target_.position.y,
        return_home_target_.position.z,
        0.0);
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "🏠 [RETURN_HOME] Navegando para origem "
        "(x=%.2f, y=%.2f, z=%.2f) — aguardando waypoints de pouso...",
        return_home_target_.position.x,
        return_home_target_.position.y,
        return_home_target_.position.z);
      return;
    }
    int hold_idx = current_waypoint_idx_ - 1;
    if (hold_idx >= 0 && static_cast<size_t>(hold_idx) < trajectory_waypoints_.size()) {
      publish_trajectory_waypoint_setpoint(hold_idx);
    }
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "⏳ [MISSION] Aguardando /mission_waypoints (waypoints de pouso)...");
    return;
  }

  // Clamp index.
  if (mission_wp_follow_idx_ < 0 ||
      static_cast<size_t>(mission_wp_follow_idx_) >= mission_waypoints_.size())
  {
    mission_wp_follow_idx_ = static_cast<int>(mission_waypoints_.size()) - 1;
  }

  const auto & mwp = mission_waypoints_[mission_wp_follow_idx_];
  publishPositionTargetWithYaw(mwp.position.x, mwp.position.y, mwp.position.z, 0.0);

  // Publish current mission waypoint to /waypoint_goal when the active index changes.
  if (last_published_mission_wp_idx_ != mission_wp_follow_idx_) {
    last_published_mission_wp_idx_ = mission_wp_follow_idx_;
    publish_waypoint_goal_status(mwp.position.x, mwp.position.y, mwp.position.z);
  }

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
    "🔄 [MISSION] Seguindo waypoint de missão [%d/%zu]: X=%.2f, Y=%.2f, Z=%.2f",
    mission_wp_follow_idx_,
    mission_waypoints_.size() - 1,
    mwp.position.x, mwp.position.y, mwp.position.z);

  // Advance to the next mission waypoint when the current one is reached.
  // Slightly larger tolerances than the main trajectory (XY=0.15, Z=0.25 vs
  // XY=0.10, Z=0.15) because mission waypoints (landing approach steps) are
  // intermediate altitude targets — the existing landing-detection logic will
  // trigger STATE 4 once the drone actually reaches ground level.
  if (mission_wp_follow_idx_ < static_cast<int>(mission_waypoints_.size()) - 1) {
    constexpr double XY_TOL = 0.15;
    constexpr double Z_TOL  = 0.25;
    double dx = mwp.position.x - current_x_ned_;
    double dy = mwp.position.y - current_y_ned_;
    double dz = std::abs(mwp.position.z - current_z_real_);
    double dist_xy = std::sqrt(dx * dx + dy * dy);
    if (dist_xy <= XY_TOL && dz <= Z_TOL) {
      mission_wp_follow_idx_++;
    }
  }
  // When at the last mission waypoint the existing landing-detection logic in
  // detect_and_handle_landing_in_trajectory() will fire when the autopilot
  // reports LANDING (4) or ON_GROUND (1).
}

void DroneControllerCompleto::handle_state3_trajectory()
{
  if (detect_and_handle_landing_in_trajectory()) { return; }

  if (pouso_em_andamento_ && !controlador_ativo_) {
    RCLCPP_WARN(this->get_logger(), "🛬 POUSO DETECTADO EM TRAJETÓRIA - PARANDO IMEDIATAMENTE!");
    state_voo_ = 4;
    return;
  }

  if (!initialize_trajectory()) { return; }

  // Mission interrupt: follow mission_waypoints instead of main trajectory.
  // WAIT_LAND_WP / FOLLOW_LAND: per-waypoint landing cycle.
  // RETURN_HOME: post-trajectory navigation to the home position; accepted
  //   multi-pose landing waypoints are followed in FOLLOW_LAND from here.
  if (mission_cycle_phase_ == MissionCyclePhase::WAIT_LAND_WP ||
      mission_cycle_phase_ == MissionCyclePhase::FOLLOW_LAND   ||
      mission_cycle_phase_ == MissionCyclePhase::RETURN_HOME)
  {
    handle_mission_interrupt_in_state3();
    return;
  }

  // Index bounds check (trajectory_waypoints_ is non-empty — guaranteed by
  // initialize_trajectory() which returns false when the list is empty).
  if (current_waypoint_idx_ < 0 ||
      static_cast<size_t>(current_waypoint_idx_) >= trajectory_waypoints_.size())
  {
    RCLCPP_ERROR(this->get_logger(),
      "❌ Índice de waypoint inválido: %d (tamanho=%zu)",
      current_waypoint_idx_, trajectory_waypoints_.size());
    state_voo_ = 2;
    return;
  }

  // ── Trajectory setpoint publishing ───────────────────────────────────────
  // When the planner is initialized (multi-segment trajectory), use
  // TrajectoryPlanner_codegen for continuous (Xd, Vd, Ad) and Drone_codegen
  // for the Z-velocity command.  Otherwise fall back to position-only setpoints.
  if (planner_initialized_) {
    double elapsed = (this->now() - trajectory_start_time_).seconds();

    double Xd[3], Vd[3], Ad[3];
    planner_.getNextSetpoint(elapsed, Xd, Vd, Ad);

    // Feed current odometry into the position controller.
    drone_ctrl_.r[0]  = current_x_ned_;
    drone_ctrl_.r[1]  = current_y_ned_;
    drone_ctrl_.r[2]  = current_z_ned_;
    drone_ctrl_.dr[0] = current_vx_ned_;
    drone_ctrl_.dr[1] = current_vy_ned_;
    drone_ctrl_.dr[2] = current_vz_ned_;
    drone_ctrl_.PositionCtrl(Xd, Vd, Ad);

    // Compute yaw toward the current target waypoint using existing logic.
    size_t last_idx = trajectory_waypoints_.size() - 1;
    bool at_last_wp = (static_cast<size_t>(current_waypoint_idx_) == last_idx);
    double yaw_follow = compute_yaw_for_trajectory_waypoint(
      std::min(current_waypoint_idx_, static_cast<int>(last_idx)), at_last_wp);

    // Publish position + velocity + yaw setpoint.
    // Design: XY velocity uses planner feedforward (Vd[0], Vd[1]) so PX4 can
    // anticipate and smoothly track the continuous trajectory.  Z velocity uses
    // drone_ctrl_.zdot_des (from the PID position controller) so altitude error
    // is actively corrected independently of the planner's Z feedforward.
    publishPositionTargetWithVelocityAndYaw(
      Xd[0], Xd[1], Xd[2],
      Vd[0], Vd[1], drone_ctrl_.zdot_des,
      yaw_follow);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "🛸 [PLANNER] t=%.2fs seg=%d/%d | "
      "Xd=(%.2f,%.2f,%.2f) Vd=(%.2f,%.2f) zdot_des=%.2f",
      elapsed, current_waypoint_idx_, planner_.numSegments,
      Xd[0], Xd[1], Xd[2], Vd[0], Vd[1], drone_ctrl_.zdot_des);
  } else {
    // Single-waypoint fallback: publish position setpoint as before.
    publish_trajectory_waypoint_setpoint(current_waypoint_idx_);
  }
  log_trajectory_progress(current_waypoint_idx_);

  // Detect waypoint reach by position and publish /waypoint_reached (once per index).
  {
    constexpr double XY_TOL = 0.10;  // radial XY tolerance [m]
    constexpr double Z_TOL  = 0.15;  // absolute Z tolerance [m]
    const auto & wp = trajectory_waypoints_[current_waypoint_idx_];
    double dx = wp.position.x - current_x_ned_;
    double dy = wp.position.y - current_y_ned_;
    double dz = std::abs(wp.position.z - current_z_real_);
    double dist_xy = std::sqrt(dx * dx + dy * dy);
    if (dist_xy <= XY_TOL && dz <= Z_TOL &&
        current_waypoint_idx_ != last_waypoint_reached_idx_)
    {
      last_waypoint_reached_idx_ = current_waypoint_idx_;

      std_msgs::msg::Int32 reached_msg;
      reached_msg.data = current_waypoint_idx_;
      waypoint_reached_pub_->publish(reached_msg);
      RCLCPP_INFO(this->get_logger(),
        "📍 Waypoint %d atingido (dist_xy=%.3fm, dz=%.3fm) → /waypoint_reached",
        current_waypoint_idx_, dist_xy, dz);

      // Publish the exact waypoint position as latch pose so that
      // drone_publish_landing_waypoints can use it as the XY reference for
      // landing (avoids any XY=(0,0) fallback from delayed MAVROS pose).
      {
        geometry_msgs::msg::PoseStamped latch_msg;
        latch_msg.header.stamp = this->now();
        latch_msg.header.frame_id = "map";
        latch_msg.pose = trajectory_waypoints_[last_waypoint_reached_idx_];
        mission_latch_pose_pub_->publish(latch_msg);
        // Mirror locally so sanitize_takeoff_xy() can use it without a self-subscriber.
        last_latch_pose_      = latch_msg.pose;
        last_latch_pose_time_ = latch_msg.header.stamp;
        has_latch_pose_       = true;
        RCLCPP_INFO(this->get_logger(),
          "📌 [MISSION] Latch pose publicada para WP[%d]: X=%.2f, Y=%.2f, Z=%.2f",
          last_waypoint_reached_idx_,
          latch_msg.pose.position.x, latch_msg.pose.position.y, latch_msg.pose.position.z);
      }

      // Advance to the next waypoint index.
      current_waypoint_idx_++;

      // Publish the next active trajectory waypoint to /waypoint_goal for monitoring.
      if (current_waypoint_idx_ < static_cast<int>(trajectory_waypoints_.size())) {
        const auto & next_wp = trajectory_waypoints_[current_waypoint_idx_];
        publish_waypoint_goal_status(
          next_wp.position.x, next_wp.position.y, next_wp.position.z);
      }

      // Check whether the trajectory is now complete.
      if (current_waypoint_idx_ >= static_cast<int>(trajectory_waypoints_.size())) {
        finalize_trajectory_complete();
        // Return to HOVER: drone holds at the last waypoint and waits for
        // new /waypoints. No mission cycle is started (Option B).
        RCLCPP_INFO(this->get_logger(),
          "🏁 [HOVER] Trajetória concluída (WP%d) — retornando a ESTADO 2. "
          "Aguardando novos /waypoints.",
          last_waypoint_reached_idx_);
        controlador_ativo_ = false;
        state_voo_ = 2;
        return;
      }

      // Intermediate waypoint reached: continue to the next without pausing
      // for a mission cycle.  Mission is activated only at the end of the
      // full trajectory (see finalize_trajectory_complete above).
      RCLCPP_INFO(this->get_logger(),
        "➡️ Waypoint %d atingido — avançando para próximo waypoint da trajetória.",
        last_waypoint_reached_idx_);
    }
  }
}

// ============================================================
// MISSION INTERRUPT DONE CALLBACK (/mission_interrupt_done)
// ============================================================

void DroneControllerCompleto::mission_interrupt_done_callback(
  const std_msgs::msg::Int32::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  RCLCPP_INFO(this->get_logger(),
    "📩 [MISSION] /mission_interrupt_done recebido (WP[%d]). "
    "mission_cycle_phase_=%s, state_voo_=%d",
    msg->data,
    mission_cycle_phase_name(mission_cycle_phase_),
    state_voo_);

  if (mission_cycle_phase_ == MissionCyclePhase::NONE) {
    // Already resumed via the altitude-based check in handle_state2_hover(); no-op.
    RCLCPP_INFO(this->get_logger(),
      "ℹ️ [MISSION] Trajetória já retomada (phase=NONE). Sinal ignorado.");
    return;
  }

  // Fallback resume: altitude-based check in handle_state2_hover() may not have
  // fired yet (e.g. drone still climbing).  Set controlador_ativo_ and clear
  // mission state so the FSM resumes trajectory as soon as state_voo_ reaches 2.
  RCLCPP_INFO(this->get_logger(),
    "✅ [MISSION INTERRUPT DONE] Missão final concluída (fase anterior: %s). "
    "Retornando ao modo /waypoints normal.",
    mission_cycle_phase_name(mission_cycle_phase_));

  mission_cycle_phase_ = MissionCyclePhase::NONE;
  mission_waypoints_.clear();
  mission_wp_follow_idx_ = 0;
  last_published_mission_wp_idx_ = -1;

  // Reset mission gate: system returns to normal /waypoints-based navigation.
  // mission_enabled_ will be re-set if a new trajectory finishes in the future.
  mission_enabled_ = false;
  RCLCPP_INFO(this->get_logger(),
    "🧹 [MISSION] mission_enabled_=false — /mission_waypoints limpo/ignorado. "
    "Sistema volta ao modo /waypoints normal.");

  if (!trajectory_waypoints_.empty() &&
      current_waypoint_idx_ < static_cast<int>(trajectory_waypoints_.size()))
  {
    controlador_ativo_ = true;
    RCLCPP_INFO(this->get_logger(),
      "▶️ [MISSION] controlador_ativo_=true — trajetória retomará em WP[%d] ao atingir HOVER.",
      current_waypoint_idx_);
    // Republish the stored trajectory waypoints so observers can see the active set.
    publish_waypoints_status();
  } else {
    RCLCPP_INFO(this->get_logger(),
      "ℹ️ [MISSION] Nenhum waypoint de trajetória restante (idx=%d, tamanho=%zu). "
      "Aguardando novos /waypoints.",
      current_waypoint_idx_, trajectory_waypoints_.size());
  }
}

}  // namespace drone_control
