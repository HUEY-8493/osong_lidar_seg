#pragma once

#include <cmath>
#include <string>

namespace cx128s2_bev_perception {

// Per-track state for the high-level vehicle status layer.  It intentionally
// does not participate in association, classification, or Kalman filtering.
struct VehicleStatusState {
  std::string value = "stopping";
  std::string last_directional_value;
  double stationary_since_sec = -1.0;
  double last_observation_sec = -1.0;
  bool motion_initialized = false;
  bool observation_gap = false;
};

struct VehicleStatusUpdate {
  double stationary_duration_sec = 0.0;
  bool timestamp_regressed = false;
};

inline bool isDirectionalVehicleStatus(const std::string & status) {
  return status == "entering" || status == "exiting";
}

// Update once for a track after its final motion_state is known for a cloud.
// A predict-only/lost track preserves its status, but cannot supply stationary
// evidence. The first zero-velocity KF sample is not proof of a stopped vehicle.
// range_speed is d(hypot(world_x, world_y))/dt: negative means exiting
// toward the world origin; positive means entering away from it.
inline VehicleStatusUpdate updateVehicleStatus(
    VehicleStatusState & state, const std::string & motion_state, double range_speed,
    double stamp_sec, double stopped_duration_sec, double direction_min_speed,
    bool has_measurement) {
  VehicleStatusUpdate result;
  if ((state.stationary_since_sec >= 0.0 && stamp_sec < state.stationary_since_sec) ||
      (state.last_observation_sec >= 0.0 && stamp_sec < state.last_observation_sec)) {
    state.stationary_since_sec = -1.0;
    state.last_observation_sec = -1.0;
    state.motion_initialized = false;
    result.timestamp_regressed = true;
  }

  if (!has_measurement) {
    state.observation_gap = true;
    return result;
  }
  if (state.last_observation_sec >= 0.0 && stamp_sec == state.last_observation_sec)
    return result;
  const bool continuous = !state.observation_gap && state.last_observation_sec >= 0.0 &&
      stamp_sec > state.last_observation_sec;
  state.motion_initialized = state.motion_initialized || motion_state != "STATIONARY" || continuous;
  // Pause the existing stationary clock across a miss. A gap neither adds
  // stationary evidence nor erases the stationary observations already made.
  if (state.observation_gap && state.last_observation_sec >= 0.0 &&
      state.stationary_since_sec >= 0.0)
    state.stationary_since_sec += stamp_sec - state.last_observation_sec;
  state.last_observation_sec = stamp_sec;
  state.observation_gap = false;

  if (motion_state == "STATIONARY") {
    if (!state.motion_initialized) {
      state.value = "stopping";
      return result;
    }
    if (state.stationary_since_sec < 0.0) state.stationary_since_sec = stamp_sec;
    result.stationary_duration_sec = stamp_sec - state.stationary_since_sec;
    state.value = result.stationary_duration_sec >= stopped_duration_sec ? "stopped" : "stopping";
    return result;
  }

  // Any observed moving state ends a stationary interval, including lateral
  // and diagonal movement; entry/exit depends on range change, not axis names.
  state.stationary_since_sec = -1.0;
  if (std::abs(range_speed) >= direction_min_speed && range_speed != 0.0) {
    state.value = range_speed > 0.0 ? "entering" : "exiting";
    state.last_directional_value = state.value;
  } else if (isDirectionalVehicleStatus(state.last_directional_value)) {
    // With too little radial evidence, preserve the most recent entry/exit
    // direction, even after a stationary interval.
    state.value = state.last_directional_value;
  } else if (!isDirectionalVehicleStatus(state.value) && range_speed != 0.0) {
    // With no prior direction, use the available signed range evidence.
    state.value = range_speed > 0.0 ? "entering" : "exiting";
    state.last_directional_value = state.value;
  }
  return result;
}

}  // namespace cx128s2_bev_perception
