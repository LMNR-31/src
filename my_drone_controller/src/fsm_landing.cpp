#include "my_drone_controller/drone_controller_completo.hpp"

namespace drone_control {

// ============================================================
// FSM STATE 4 — POUSO (helpers + handler)
// ============================================================

bool DroneControllerCompleto::handle_state4_disarm_reset()
{
  if (state_voo_ != 4) { return false; }

  if (!current_state_.armed) {
    // Drone is already disarmed while the FSM is still in state 4 (this can
    // happen when a new command arrives between the FCU confirming DISARM and
    // the control-loop's handle_state4_landing() transitioning the state).
    // Transition to state 0 here and return FALSE so that the caller continues
    // to process the incoming takeoff command — without this the command would
    // always be silently dropped, requiring the operator to re-send it.
    RCLCPP_INFO(this->get_logger(),
      "✅ DRONE DESARMADO em estado 4 — transicionando para aguardar novo comando.");
    disarm_requested_ = false;
    state_voo_ = 0;
    return false;  // let the caller proceed with the new takeoff command
  }

  if (disarm_requested_) {
    // Drone is still armed and we are waiting for the FCU to confirm DISARM.
    // The new command must be dropped — accepting a takeoff while DISARM is
    // pending would conflict with the ongoing landing sequence.
    RCLCPP_INFO(this->get_logger(),
      "[DISARM] Aguardando confirmação de DISARM pelo FCU (armed=1); ignorando waypoint recebido.");
  }
  // State 4 and drone still armed: signal to caller that this was a landing
  // state and the command should not be processed yet.
  return true;
}

void DroneControllerCompleto::complete_landing()
{
  if (land_cmd_id_) {
    cmd_queue_.confirm(*land_cmd_id_, true);
    RCLCPP_WARN(this->get_logger(),
      "✅ [ID=%lu] LAND confirmado — iniciando DISARM", *land_cmd_id_);
  }

  cmd_queue_.save_log("/tmp/drone_commands.log");
  RCLCPP_INFO(this->get_logger(),
    "💾 Histórico de comandos salvo em /tmp/drone_commands.log");

  RCLCPP_WARN(this->get_logger(),
    "\n✅ POUSO CONCLUÍDO! Solicitando DISARM e aguardando confirmação do FCU...\n");

  // Mark that we are waiting for DISARM confirmation before transitioning state.
  // The state transition to 0 happens only after current_state_.armed becomes false.
  disarm_requested_ = true;
  // Clear these early so handle_state4_landing() does NOT re-enter the
  // landing-timeout block and re-call complete_landing() while we are still
  // waiting for the FCU to confirm DISARM.
  pouso_em_andamento_ = false;
  pouso_start_time_set_ = false;
}

void DroneControllerCompleto::handle_state4_landing()
{
  // Wait for FCU to confirm DISARM before transitioning state.
  if (disarm_requested_) {
    if (!current_state_.armed) {
      RCLCPP_INFO(this->get_logger(),
        "✅ DISARM confirmado pelo FCU — transicionando para aguardar novo comando.");
      disarm_requested_ = false;

      // If this DISARM is part of a mission interrupt cycle, advance to
      // WAIT_TAKEOFF_WP so the takeoff waypoint from mission_manager is accepted
      // even if the WAIT_LAND_WP / FOLLOW_LAND phase has already been processed.
      // This is the key state transition that prevents the "takeoff recebido sem
      // missão ativa" bug: the phase is now WAIT_TAKEOFF_WP, not NONE.
      if (mission_cycle_phase_ == MissionCyclePhase::FOLLOW_LAND ||
          mission_cycle_phase_ == MissionCyclePhase::WAIT_LAND_WP)
      {
        mission_cycle_phase_ = MissionCyclePhase::WAIT_TAKEOFF_WP;
        RCLCPP_INFO(this->get_logger(),
          "🔄 [MISSION WAIT_TAKEOFF_WP] DISARM confirmado durante ciclo de missão. "
          "Aguardando waypoint de decolagem em /mission_waypoints...");
      }

      state_voo_ = 0;
      RCLCPP_WARN(this->get_logger(),
        "⏳ Aguardando novo comando de waypoint para decolar novamente... "
        "(fase: %s)",
        mission_cycle_phase_name(mission_cycle_phase_));
    } else {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
        "⏳ [DISARM] Aguardando confirmação de DISARM pelo FCU...");
    }
    return;
  }

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
    "⬇️ POUSANDO — aguardando confirmação de aterrissagem...");

  if (pouso_em_andamento_) {
    if (!pouso_start_time_set_) {
      pouso_start_time_ = this->now();
      pouso_start_time_set_ = true;
      RCLCPP_INFO(this->get_logger(),
        "⏱️ Iniciando contagem de pouso (%.0f s para confirmar)...", config_.landing_timeout);
    }

    if ((this->now() - pouso_start_time_).seconds() > config_.landing_timeout) {
      complete_landing();
      return;
    }
  }
}

}  // namespace drone_control
