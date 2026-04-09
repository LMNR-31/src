#include "my_drone_controller/drone_controller_completo.hpp"

namespace drone_control {

// ============================================================
// FSM STATE 2 — HOVER
// ============================================================

void DroneControllerCompleto::handle_state2_hover()
{
  if (using_4d_goal_) {
    publishPositionTargetWithYaw(
      last_waypoint_goal_.pose.position.x,
      last_waypoint_goal_.pose.position.y,
      last_waypoint_goal_.pose.position.z,
      goal_yaw_rad_);
  } else {
    publishPositionTarget(
      last_waypoint_goal_.pose.position.x,
      last_waypoint_goal_.pose.position.y,
      last_waypoint_goal_.pose.position.z,
      0.0, MASK_POS_ONLY);
  }

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
    "🛸 Em HOVER (%.1fm) | Posição: X=%.2f, Y=%.2f | Aguardando waypoints... Controlador: %s",
    last_waypoint_goal_.pose.position.z,
    last_waypoint_goal_.pose.position.x,
    last_waypoint_goal_.pose.position.y,
    controlador_ativo_ ? "ATIVO" : "INATIVO");

  // After a mission interrupt re-arm, resume the main trajectory when the
  // drone has reached a safe altitude (z >= MISSION_RESUME_ALTITUDE_M).
  if (mission_cycle_phase_ == MissionCyclePhase::FOLLOW_TAKEOFF &&
      current_z_real_ >= MISSION_RESUME_ALTITUDE_M)
  {
    RCLCPP_INFO(this->get_logger(),
      "✅ [MISSION FOLLOW_TAKEOFF] Altitude %.2f m >= %.1f m após re-arm. "
      "Retomando trajetória principal em WP[%d]. Fase: NONE.",
      current_z_real_, MISSION_RESUME_ALTITUDE_M, current_waypoint_idx_);
    mission_cycle_phase_ = MissionCyclePhase::NONE;
    mission_waypoints_.clear();
    mission_wp_follow_idx_ = 0;
    last_published_mission_wp_idx_ = -1;
    // Only resume trajectory if there are remaining waypoints.
    if (!trajectory_waypoints_.empty() &&
        current_waypoint_idx_ < static_cast<int>(trajectory_waypoints_.size()))
    {
      controlador_ativo_ = true;
    }
    return;
  }

  // Detect landing from autopilot signal during HOVER (e.g. after trajectory
  // completes at a landing waypoint above the initial takeoff altitude).
  if (autopilot_indicates_landing()) {
    const uint8_t ls = last_extended_state_.landed_state;
    const bool is_landing = (ls == mavros_msgs::msg::ExtendedState::LANDED_STATE_LANDING);
    RCLCPP_WARN(this->get_logger(),
      "🛬 [HOVER] POUSO DETECTADO (autopiloto)! landed_state=%d (%s), Z = %.2f m - Transitando para STATE 4",
      static_cast<int>(ls),
      is_landing ? "LANDING" : "ON_GROUND",
      current_z_real_);
    trigger_landing(current_z_real_);
    return;
  }

  if (controlador_ativo_) {
    state_voo_ = 3;
    RCLCPP_INFO(this->get_logger(), "✈️ Iniciando execução de trajetória...\n");
  }

  if (pouso_em_andamento_) {
    RCLCPP_WARN(this->get_logger(), "🛬 POUSO DETECTADO NO HOVER - DESLIGANDO!");
    state_voo_ = 4;
  }
}

}  // namespace drone_control
