#undef NDEBUG  // Keep regression assertions active in Release builds.
#include <cassert>

#include <cx128s2_bev_perception/vehicle_status.hpp>
#include <cx128s2_bev_perception/vehicle_surface_tracking.hpp>

using cx128s2_bev_perception::VehicleStatusState;
using cx128s2_bev_perception::updateVehicleStatus;

int main() {
  VehicleStatusState initial;
  updateVehicleStatus(initial, "STATIONARY", 0.0, 1.0, 10.0, 0.3, true);
  assert(!initial.motion_initialized && initial.stationary_since_sec < 0.0);
  assert(initial.value == "stopping");
  updateVehicleStatus(initial, "MOVING_FORWARD", 0.8, 1.1, 10.0, 0.3, true);
  assert(initial.motion_initialized && initial.value == "entering");
  // A gap cannot become ten seconds of stationary evidence at reconnection.
  updateVehicleStatus(initial, "STATIONARY", 0.0, 2.0, 10.0, 0.3, true);
  updateVehicleStatus(initial, "STATIONARY", 0.0, 20.0, 10.0, 0.3, false);
  updateVehicleStatus(initial, "STATIONARY", 0.0, 21.0, 10.0, 0.3, true);
  assert(initial.value == "stopping" && initial.motion_initialized);
  updateVehicleStatus(initial, "STATIONARY", 0.0, 21.1, 10.0, 0.3, true);
  updateVehicleStatus(initial, "STATIONARY", 0.0, 31.1, 10.0, 0.3, true);
  assert(initial.value == "stopped" && initial.motion_initialized);
  updateVehicleStatus(initial, "STATIONARY", 0.0, 31.1, 10.0, 0.3, true);
  assert(initial.value == "stopped" && initial.motion_initialized);
  updateVehicleStatus(initial, "STATIONARY", 0.0, 31.2, 10.0, 0.3, false);
  updateVehicleStatus(initial, "STATIONARY", 0.0, 31.3, 10.0, 0.3, true);
  assert(initial.motion_initialized && initial.value == "stopped");
  updateVehicleStatus(initial, "STATIONARY", 0.0, 31.4, 10.0, 0.3, true);
  assert(initial.motion_initialized && initial.value == "stopped");
  // Physical sequences in world: position delta is current minus previous.
  // Include negative X, Y-only, diagonal, and conflicting axis/radial signs.
  using namespace cx128s2_bev_perception::vehicle_surface;
  for (const auto & sample : std::vector<std::pair<Point2, Point2>>{
      {{10.0, 0.0}, {9.0, 0.0}}, {{9.0, 0.0}, {10.0, 0.0}},
      {{-10.0, 0.0}, {-9.0, 0.0}}, {{-9.0, 0.0}, {-10.0, 0.0}},
      {{0.0, 10.0}, {0.0, 9.0}}, {{0.0, 9.0}, {0.0, 10.0}},
      {{6.0, 8.0}, {5.4, 7.2}}, {{5.4, 7.2}, {6.0, 8.0}},
      {{10.0, 30.0}, {11.0, 27.0}}, {{11.0, 27.0}, {10.0, 30.0}}}) {
    const Point2 position = sample.second;
    const Point2 velocity{position.x - sample.first.x, position.y - sample.first.y};
    const double distance = std::hypot(position.x, position.y);
    const double range_speed = (position.x * velocity.x + position.y * velocity.y) / distance;
    const bool exiting = distance < std::hypot(sample.first.x, sample.first.y);
    VehicleStatusState physical;
    const std::string motion = std::abs(velocity.x) > std::abs(velocity.y) ?
        (velocity.x > 0.0 ? "MOVING_FORWARD" : "MOVING_BACKWARD") : "MOVING_LEFT";
    updateVehicleStatus(physical, motion, range_speed, 1.0, 10.0, 0.3, true);
    assert(physical.value == (exiting ? "exiting" : "entering"));
    assert(signedCorridorDirection(position, velocity, 0.3) ==
        (exiting ? CorridorDirection::EXIT : CorridorDirection::ENTRY));
  }

  VehicleStatusState tangent;
  updateVehicleStatus(tangent, "MOVING_FORWARD", 0.0, 1.0, 10.0, 0.3, true);
  assert(tangent.value == "stopping" && tangent.last_directional_value.empty());
  updateVehicleStatus(tangent, "MOVING_BACKWARD", -0.8, 2.0, 10.0, 0.3, true);
  updateVehicleStatus(tangent, "MOVING_FORWARD", 0.0, 3.0, 10.0, 0.3, true);
  assert(tangent.value == "exiting");  // No direction evidence at origin/tangent.
  updateVehicleStatus(tangent, "MOVING_FORWARD", 0.8, 4.0, 10.0, 0.3, false);
  assert(tangent.value == "exiting");  // Predict-only reversal cannot change status.

  VehicleStatusState state;

  updateVehicleStatus(state, "MOVING_FORWARD", 0.8, 1.0, 10.0, 0.3, true);
  assert(state.value == "entering" && state.stationary_since_sec < 0.0);

  updateVehicleStatus(state, "MOVING_BACKWARD", -0.8, 2.0, 10.0, 0.3, true);
  assert(state.value == "exiting" && state.stationary_since_sec < 0.0);

  updateVehicleStatus(state, "STATIONARY", 0.0, 100.0, 10.0, 0.3, true);
  assert(state.value == "stopping" && state.stationary_since_sec == 100.0);
  updateVehicleStatus(state, "STATIONARY", 0.0, 109.9, 10.0, 0.3, true);
  assert(state.value == "stopping");
  updateVehicleStatus(state, "STATIONARY", 0.0, 110.0, 10.0, 0.3, true);
  assert(state.value == "stopped");

  updateVehicleStatus(state, "MOVING_FORWARD", 0.7, 111.0, 10.0, 0.3, true);
  assert(state.value == "entering" && state.stationary_since_sec < 0.0);
  updateVehicleStatus(state, "STATIONARY", 0.0, 120.0, 10.0, 0.3, true);
  updateVehicleStatus(state, "STATIONARY", 0.0, 130.0, 10.0, 0.3, true);
  updateVehicleStatus(state, "MOVING_LEFT", 0.1, 131.0, 10.0, 0.3, true);
  assert(state.value == "entering" && state.stationary_since_sec < 0.0);
  updateVehicleStatus(state, "MOVING_BACKWARD", -0.7, 112.0, 10.0, 0.3, true);
  assert(state.value == "exiting" && state.stationary_since_sec < 0.0);

  updateVehicleStatus(state, "STATIONARY", 0.0, 200.0, 10.0, 0.3, true);
  const auto reverse = updateVehicleStatus(state, "STATIONARY", 0.0, 199.0, 10.0, 0.3, true);
  assert(reverse.timestamp_regressed && state.value == "stopping" && !state.motion_initialized);
  updateVehicleStatus(state, "STATIONARY", 0.0, 199.1, 10.0, 0.3, true);

  updateVehicleStatus(state, "STATIONARY", 0.0, 205.0, 10.0, 0.3, false);
  assert(state.stationary_since_sec == 199.1);  // predict-only loss does not advance the timer
}
