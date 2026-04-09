#include "my_drone_controller/drone_controller_completo.hpp"

namespace drone_control {

// ============================================================
// MISSION CYCLE PHASE HELPERS
// ============================================================

/// Returns a stable human-readable name for each MissionCyclePhase value.
const char * DroneControllerCompleto::mission_cycle_phase_name(MissionCyclePhase p)
{
  switch (p) {
    case MissionCyclePhase::NONE:            return "NONE";
    case MissionCyclePhase::WAIT_LAND_WP:    return "WAIT_LAND_WP";
    case MissionCyclePhase::FOLLOW_LAND:     return "FOLLOW_LAND";
    case MissionCyclePhase::WAIT_TAKEOFF_WP: return "WAIT_TAKEOFF_WP";
    case MissionCyclePhase::FOLLOW_TAKEOFF:  return "FOLLOW_TAKEOFF";
    case MissionCyclePhase::RETURN_HOME:     return "RETURN_HOME";
    default:                                  return "UNKNOWN";
  }
}

}  // namespace drone_control
