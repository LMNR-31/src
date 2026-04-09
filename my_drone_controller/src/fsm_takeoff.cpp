#include "my_drone_controller/drone_controller_completo.hpp"

#include <cmath>

namespace drone_control {

// ============================================================
// FSM STATE 1 — DECOLAGEM (3-layer sequence)
// ============================================================

void DroneControllerCompleto::request_arm_and_offboard_activation()
{
  // Per PX4/MAVROS best practice, OFFBOARD and ARM must be sent SEPARATELY.
  // Sending ARM immediately after (or simultaneously with) the OFFBOARD request
  // causes the FCU to reject ARM even when OFFBOARD is accepted, because PX4
  // enforces that the mode transition must be complete before it honours ARM.
  //
  // This function therefore requests ONLY the OFFBOARD mode change.  The ARM
  // command is sent later by handle_state1_takeoff(), after wait_for_offboard_mode()
  // confirms that the FCU has actually switched to OFFBOARD.
  RCLCPP_INFO(this->get_logger(),
    "📡 Solicitando modo OFFBOARD (ARM aguardará confirmação do FCU)...");
  request_offboard();
  offboard_activated_ = true;
  activation_time_ = this->now();
}

// ============================================================
// PRE-ARM SETPOINT STREAMING
// ============================================================

void DroneControllerCompleto::stream_initial_setpoints()
{
  // PX4/MAVROS requires a continuous stream of position setpoints to be
  // published on /uav1/mavros/setpoint_raw/local BEFORE the FCU will accept
  // an ARM command in OFFBOARD mode.  Without this pre-stream the FCU replies
  // "ARM rejected" immediately even though it successfully enters OFFBOARD.
  //
  // We publish INITIAL_STREAM_THRESHOLD setpoints (~200 ms at 100 Hz) at the
  // target takeoff position before triggering the OFFBOARD+ARM sequence.
  // After this point the control-loop timer keeps setpoints flowing, so ARM
  // will not be rejected on subsequent retry attempts either.
  //
  // Invariant: last_waypoint_goal_ is always set before state_voo_ is moved
  // to 1 (see handle_single_takeoff_waypoint_command / waypoints_4d_callback),
  // and init_variables() initialises it to (0, 0, hover_altitude) as a safe
  // fallback, so the published position is always well-defined.
  publishPositionTarget(
    last_waypoint_goal_.pose.position.x,
    last_waypoint_goal_.pose.position.y,
    last_waypoint_goal_.pose.position.z,
    0.0, MASK_POS_ONLY);

  initial_stream_count_++;

  if (initial_stream_count_ == 1) {
    RCLCPP_INFO(this->get_logger(),
      "📡 [STREAM] Iniciando streaming inicial de setpoints antes de ARM+OFFBOARD "
      "(meta: %d mensagens)...", INITIAL_STREAM_THRESHOLD);
  }

  if (initial_stream_count_ >= INITIAL_STREAM_THRESHOLD) {
    initial_stream_done_ = true;
    RCLCPP_INFO(this->get_logger(),
      "✅ [STREAM] Streaming inicial concluído (%d setpoints publicados). "
      "Prosseguindo com solicitação de OFFBOARD+ARM.", initial_stream_count_);
  }
}

void DroneControllerCompleto::stream_post_offboard_setpoints()
{
  // PX4/MAVROS rejects ARM even after accepting OFFBOARD mode if the setpoint
  // stream has not been active long enough.  The FCU requires at least 1.5 seconds
  // of continuous setpoints in OFFBOARD mode before it will honour an ARM request.
  //
  // This function is called every control-loop iteration (100 Hz / 10 ms) AFTER
  // the FCU confirms OFFBOARD and BEFORE ARM is requested.  It accumulates
  // POST_OFFBOARD_STREAM_THRESHOLD = 150 setpoints at 10 ms per call — 1.5 s
  // total at 100 Hz, providing the same duration as the PX4/MAVROS recommendation
  // of 30 setpoints at 50 ms each — then sets post_offboard_stream_done_ to allow
  // handle_state1_takeoff() to proceed to the ARM step.

  publishPositionTarget(
    last_waypoint_goal_.pose.position.x,
    last_waypoint_goal_.pose.position.y,
    last_waypoint_goal_.pose.position.z,
    0.0, MASK_POS_ONLY);

  post_offboard_stream_count_++;

  if (post_offboard_stream_count_ == 1) {
    RCLCPP_INFO(this->get_logger(),
      "📡 [STREAM] OFFBOARD confirmado — iniciando streaming prolongado de setpoints "
      "antes do ARM (%d setpoints × 10 ms = %.1f s)...",
      POST_OFFBOARD_STREAM_THRESHOLD,
      POST_OFFBOARD_STREAM_THRESHOLD * 0.01);
  }

  if (post_offboard_stream_count_ >= POST_OFFBOARD_STREAM_THRESHOLD) {
    post_offboard_stream_done_ = true;
    RCLCPP_INFO(this->get_logger(),
      "✅ [STREAM] Streaming pós-OFFBOARD concluído (%d setpoints / %.1f s). "
      "Enviando ARM agora...",
      post_offboard_stream_count_,
      post_offboard_stream_count_ * 0.01);
  }
}

void DroneControllerCompleto::wait_for_offboard_mode()
{
  // PX4/MAVROS requires that ARM be sent AFTER the FCU has confirmed the mode
  // change to OFFBOARD.  This function polls current_state_.mode and, once the
  // FCU reports "OFFBOARD", sets offboard_mode_confirmed_ so that
  // handle_state1_takeoff() can proceed to call request_arm() in the next cycle.
  // If the FCU does not confirm within offboard_confirm_timeout seconds the
  // OFFBOARD request is retried from scratch.

  if (current_state_.mode == "OFFBOARD") {
    RCLCPP_INFO(this->get_logger(),
      "✅ FCU confirmou modo OFFBOARD — aguardando ARM na próxima etapa...");
    offboard_mode_confirmed_ = true;
    // Reset activation_time_ so the ARM-wait timeout (activation_timeout) starts
    // counting only from the moment OFFBOARD was actually confirmed.
    activation_time_ = this->now();
    return;
  }

  if ((this->now() - activation_time_).seconds() > config_.offboard_confirm_timeout) {
    RCLCPP_WARN(this->get_logger(),
      "⚠️ Timeout esperando confirmação de modo OFFBOARD (%.0f s)! Tentando novamente...",
      config_.offboard_confirm_timeout);
    RCLCPP_WARN(this->get_logger(), "   Estado atual: mode=%s",
      current_state_.mode.c_str());
    // Reset so the next cycle re-requests OFFBOARD and starts fresh.
    offboard_activated_ = false;
    offboard_mode_confirmed_ = false;
    arm_requested_ = false;
    post_offboard_stream_count_ = 0;
    post_offboard_stream_done_ = false;
    return;
  }

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
    "⏳ Aguardando FCU confirmar modo OFFBOARD... | mode=%s",
    current_state_.mode.c_str());
}

bool DroneControllerCompleto::wait_for_offboard_arm_confirmation()
{
  if (current_state_.armed && current_state_.mode == "OFFBOARD") {
    RCLCPP_INFO(this->get_logger(), "✅ OFFBOARD+ARM CONFIRMADOS! Iniciando decolagem...");
    activation_confirmed_ = true;
    takeoff_counter_ = 0;

    if (!takeoff_cmd_id_) {
      takeoff_cmd_id_ = cmd_queue_.enqueue(
        CommandType::TAKEOFF,
        {{"x", std::to_string(last_waypoint_goal_.pose.position.x)},
         {"y", std::to_string(last_waypoint_goal_.pose.position.y)},
         {"z", std::to_string(config_.hover_altitude)}});
      RCLCPP_INFO(this->get_logger(),
        "📋 [ID=%lu] Comando TAKEOFF enfileirado após ARM+OFFBOARD confirmado!", *takeoff_cmd_id_);
    }
    return true;
  }

  if ((this->now() - activation_time_).seconds() > config_.activation_timeout) {
    RCLCPP_WARN(this->get_logger(),
      "⚠️ Timeout ativação OFFBOARD+ARM (%.0f s)! Tentando novamente...",
      config_.activation_timeout);
    RCLCPP_WARN(this->get_logger(), "   Estado: armed=%d | mode=%s",
      current_state_.armed, current_state_.mode.c_str());
    // Reset all activation flags so the full sequence restarts from OFFBOARD request.
    offboard_activated_ = false;
    offboard_mode_confirmed_ = false;
    arm_requested_ = false;
    activation_confirmed_ = false;
    post_offboard_stream_count_ = 0;
    post_offboard_stream_done_ = false;
    takeoff_counter_ = 0;
    return false;
  }

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
    "⏳ Aguardando ARM... | armed=%d | mode=%s",
    current_state_.armed, current_state_.mode.c_str());
  takeoff_counter_++;
  return false;
}

void DroneControllerCompleto::publish_takeoff_climb_setpoint(double target_alt)
{
  if (using_4d_goal_) {
    publishPositionTargetWithYaw(
      last_waypoint_goal_.pose.position.x,
      last_waypoint_goal_.pose.position.y,
      target_alt, goal_yaw_rad_);
  } else {
    publishPositionTarget(
      last_waypoint_goal_.pose.position.x,
      last_waypoint_goal_.pose.position.y,
      target_alt, 0.0, MASK_POS_ONLY);
  }

  takeoff_counter_++;

  if (takeoff_counter_ == 1) {
    RCLCPP_INFO(this->get_logger(), "⬆️ Decolando para %.1f metros...", target_alt);
    RCLCPP_INFO(this->get_logger(), "   Posição: X=%.2f, Y=%.2f, Z=%.1f",
      last_waypoint_goal_.pose.position.x,
      last_waypoint_goal_.pose.position.y, target_alt);
  }

  if (takeoff_counter_ % 100 == 0) {
    RCLCPP_INFO(this->get_logger(),
      "📈 Decolando... Z_alvo=%.1fm | Z_real=%.2fm | Tempo=%.1fs",
      target_alt, current_z_real_, (double)takeoff_counter_ / 100.0);
  }
}

void DroneControllerCompleto::finalize_takeoff_on_altitude_reached(double target_alt)
{
  double arrival_threshold = target_alt - config_.hover_altitude_margin;
  if (current_z_real_ < arrival_threshold) { return; }

  RCLCPP_INFO(this->get_logger(), "✅ Decolagem concluída! Altitude = %.2fm\n", current_z_real_);

  if (takeoff_cmd_id_) {
    cmd_queue_.confirm(*takeoff_cmd_id_, true);
    RCLCPP_INFO(this->get_logger(),
      "✅ [ID=%lu] TAKEOFF confirmado! Altitude=%.2fm", *takeoff_cmd_id_, current_z_real_);
    takeoff_cmd_id_.reset();
  }

  hover_cmd_id_ = cmd_queue_.enqueue(
    CommandType::HOVER,
    {{"x", std::to_string(last_waypoint_goal_.pose.position.x)},
     {"y", std::to_string(last_waypoint_goal_.pose.position.y)},
     {"z", std::to_string(target_alt)}});
  RCLCPP_INFO(this->get_logger(), "📋 [ID=%lu] Comando HOVER enfileirado", *hover_cmd_id_);

  state_voo_ = 2;
  takeoff_counter_ = 0;
}

void DroneControllerCompleto::handle_state1_takeoff()
{
  // Step 1 — Pre-ARM streaming: publish setpoints before requesting OFFBOARD.
  // PX4 rejects ARM in OFFBOARD mode when no setpoints have been streamed yet.
  // stream_initial_setpoints() accumulates INITIAL_STREAM_THRESHOLD messages
  // (~200 ms at 100 Hz) and sets initial_stream_done_ when the threshold is met.
  if (!initial_stream_done_) {
    stream_initial_setpoints();
    return;
  }

  // Step 2 — Request OFFBOARD mode only (ARM is withheld until OFFBOARD is confirmed).
  // Sending ARM simultaneously with OFFBOARD is rejected by PX4 because the mode
  // transition has not completed yet.  See PX4 MAVROS offboard guide and the
  // wait_for_offboard_mode() implementation above for full rationale.
  if (!offboard_activated_) {
    request_arm_and_offboard_activation();
    return;
  }

  // Step 3 — Wait for the FCU to report mode == "OFFBOARD" before arming.
  // offboard_mode_confirmed_ is set by wait_for_offboard_mode() once the FCU
  // confirms the transition.  Only then does the sequence proceed to ARM.
  if (!offboard_mode_confirmed_) {
    wait_for_offboard_mode();
    return;
  }

  // Step 3.5 — Post-OFFBOARD extended streaming: publish setpoints for 1.5 s
  // after OFFBOARD is confirmed, BEFORE sending ARM.
  //
  // Rationale (PX4/MAVROS offboard guide): even after the FCU accepts the
  // OFFBOARD mode change, it may still reject ARM if the setpoint stream has
  // not been running long enough inside OFFBOARD mode.  Publishing at least
  // 30 setpoints at 50 ms intervals (= 1.5 s) gives the FCU enough time to
  // stabilise the mode and accept the subsequent ARM command.
  //
  // stream_post_offboard_setpoints() counts calls and sets
  // post_offboard_stream_done_ once POST_OFFBOARD_STREAM_THRESHOLD is reached.
  if (!post_offboard_stream_done_) {
    stream_post_offboard_setpoints();
    return;
  }

  // Step 4 — ARM the vehicle, now that OFFBOARD mode is confirmed by the FCU
  // AND the 1.5-second post-OFFBOARD streaming window has elapsed.
  // Sending ARM only after OFFBOARD is confirmed AND the stream has been
  // running long enough avoids the FCU rejecting ARM.
  if (!arm_requested_) {
    RCLCPP_INFO(this->get_logger(),
      "🔋 Modo OFFBOARD confirmado pelo FCU — solicitando ARM agora...");
    request_arm();
    arm_requested_ = true;
    return;
  }

  // Step 5 — Wait for ARM confirmation from the FCU.
  if (!activation_confirmed_) {
    if (!wait_for_offboard_arm_confirmation()) { return; }
  }

  // The takeoff target altitude was computed and fixed at command-receive time
  // (in handle_single_takeoff_waypoint_command(), mission_waypoints_callback(),
  // or the 4D equivalent) as:
  //   takeoff_target_z_ = clamp(waypoint.z, min_altitude, max_altitude)
  //
  // The drone climbs to the Z commanded by the waypoint (respecting physical
  // limits), not to hover_altitude or current_z_real_ + takeoff_z_boost.
  // Publishing a fixed target (not recomputed on every 10 ms cycle) prevents
  // the infinite-ascent bug: if the target tracked current_z_real_ every cycle
  // it would rise together with the drone and never be reached.
  //
  // Safety guard: takeoff_target_z_ should never still be -1.0 here in normal
  // operation.  If it is, fall back to a safe computation from current_z_real_.
  if (takeoff_target_z_ < 0.0) {
    RCLCPP_ERROR(this->get_logger(),
      "⚠️ [TAKEOFF] takeoff_target_z_ não foi inicializado antes de handle_state1_takeoff() "
      "(sentinel -1.0 inesperado). Calculando fallback a partir de current_z_real_=%.2fm.",
      current_z_real_);
    takeoff_target_z_ = std::max(
      config_.hover_altitude,
      current_z_real_ + config_.takeoff_z_boost);
  }

  // Use the fixed value for every control cycle until the drone reaches the target.
  const double target_altitude = takeoff_target_z_;

  publish_takeoff_climb_setpoint(target_altitude);
  finalize_takeoff_on_altitude_reached(target_altitude);
}

}  // namespace drone_control
