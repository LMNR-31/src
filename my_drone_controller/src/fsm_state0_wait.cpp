#include "my_drone_controller/drone_controller_completo.hpp"

namespace drone_control {

// ============================================================
// FSM STATE 0 — AGUARDANDO WAYPOINT
// ============================================================

void DroneControllerCompleto::handle_state0_wait_waypoint()
{
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
    "⏳ Aguardando novo comando de waypoint para decolar...");
}

}  // namespace drone_control
