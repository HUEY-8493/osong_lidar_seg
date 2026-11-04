#include <cx128s2_bev_perception/vehicle_surface_tracking.hpp>
#include <cx128s2_bev_perception/linear_kalman_filter.hpp>
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <vector>
using namespace cx128s2_bev_perception::vehicle_surface;
int main() {
  // Asymmetric/partial clouds never rotate or swap entrance length/width axes.
  for (double axis : {0.0, .4}) {
    std::vector<Point2> cloud;
    for (const auto & q : std::vector<Point2>{{-3,-1},{3,-1},{3,1},{-3,1},{2,.7}})
      cloud.push_back(toWorld(q, {30, 4}, axis));
    const auto fixed=fixedYawObb(cloud,axis);
    assert(fixed.valid && fixed.yaw==axis);
    assert(std::abs(fixed.length-6)<1e-9 && std::abs(fixed.width-2)<1e-9);
    assert(std::hypot(fixed.center.x-30,fixed.center.y-4)<1e-9);
    cloud.pop_back(); cloud[2]=toWorld({2,.8},{30,4},axis);
    assert(fixedYawObb(cloud,axis).yaw==axis);
  }

  Obb reliable{{10.0, 3.0}, 0.3, 5.0, 2.0, true};
  const Obb partial{{11.0, 3.0}, 0.3, 2.0, 1.2, true};
  assert(!applyReliableGeometry(reliable, partial, 10, 15.0, 4.0));
  assert(std::abs(reliable.length - 5.0) < 1e-12);
  const Obb merge{{10.0, 3.0}, 0.3, 6.0, 2.5, true};
  assert(!applyReliableGeometry(reliable, merge, 1, 15.0, 4.0));
  assert(std::abs(reliable.length - 5.0) < 1e-12);
  assert(applyReliableGeometry(reliable, merge, 3, 15.0, 4.0));
  assert(std::abs(reliable.length - 6.0) < 1e-12);
  assert(std::abs(reliable.width - 2.0) < 1e-12);
  Obb end_face_geometry{{0,0},0,4.0,2.4,true};
  const Obb exposed_length{{-3,0},0,10.0,2.4,true};
  assert(!applyReliableGeometry(end_face_geometry,exposed_length,3,20,4));
  assert(!applyReliableGeometry(end_face_geometry,exposed_length,2,20,4,true));
  assert(!applyReliableGeometry(end_face_geometry,exposed_length,3,8,4,true));
  assert(applyReliableGeometry(end_face_geometry,exposed_length,3,20,4,true));
  assert(end_face_geometry.length==10.0);
  const Obb width_only{{10.0, 3.0}, 0.3, 6.0, 2.8, true};
  assert(!applyReliableGeometry(reliable, width_only, 10, 15.0, 4.0));
  assert(std::abs(reliable.width - 2.0) < 1e-12);
  const Point2 source{11.2, 4.1};
  const auto local = toLocal(source, reliable.center, reliable.yaw);
  const auto restored = toWorld(local, reliable.center, reliable.yaw);
  assert(std::hypot(restored.x - source.x, restored.y - source.y) < 1e-10);
  const auto a = faceSamples(5.0, 2.0);
  assert(a.size() == 32);
  for (const auto & p : a) {
    const auto q = toWorld(p, {0.0, 0.0}, 3.14159265358979323846);
    bool found = false;
    for (const auto & other : a)
      if (std::hypot(q.x - other.x, q.y - other.y) < 1e-10) found = true;
    assert(found);
  }
  const double yaw = alignedAxisYaw(0.08 + 3.14159265358979323846, 0.05);
  assert(std::abs(angleDifference(yaw, 0.08)) < 1e-10);
  const Obb predicted{{0, 0}, 0, 5.0, 2.0, true};
  assert(componentOwnershipScore(predicted, {0, 0}, {2.0, 0.4}, 3.0) > 0.0);
  assert(componentOwnershipScore(predicted, {0, 0}, {-2.0, -0.4}, 3.0) > 0.0);
  assert(componentOwnershipScore(predicted, {0, 0}, {20.0, 0}, 3.0) < 0.0);
  const auto ownership = assignComponentOwners({{0.90}, {0.82}}, {12, 7}, 1);
  assert(ownership.owner.size() == 2 && ownership.owner[0] == 0 &&
         ownership.owner[1] == 0 && ownership.primary[0] == 0);
  const auto ambiguous = assignComponentOwners({{0.71, 0.69}}, {8}, 2);
  assert(ambiguous.ambiguous[0] && ambiguous.owner[0] == -1);
  // A mid-range centroid gate alone accepts this unrelated residual, while
  // confirmed-vehicle surface ownership correctly rejects it. The node must
  // never send such a rejected component through generic association again.
  const Obb exiting_vehicle{{33.0866, 30.0}, 0.0, 6.0, 2.4, true};
  const Point2 unrelated_blob{38.5866, 30.0};
  const double mid_gate = std::sqrt(45.0);
  assert(std::hypot(unrelated_blob.x - exiting_vehicle.center.x,
                    unrelated_blob.y - exiting_vehicle.center.y) < mid_gate);
  assert(componentOwnershipScore(exiting_vehicle, exiting_vehicle.center,
                                 unrelated_blob, mid_gate) < 0.0);
  assert(componentOwnershipScore(exiting_vehicle, exiting_vehicle.center,
                                 {35.7866, 30.1}, mid_gate) >= 0.0);
  assert(vehicleLifecycleAfterMiss(VehicleLifecycle::ACTIVE, 8, 8) == VehicleLifecycle::ACTIVE);
  assert(vehicleLifecycleAfterMiss(VehicleLifecycle::ACTIVE, 9, 8) == VehicleLifecycle::LOST);
  assert(predictiveStepEnd(VehicleLifecycle::LOST, 10.0, 15.0, 10.0, 2.0) == 12.0);
  assert(predictiveStepEnd(VehicleLifecycle::DORMANT, 12.0, 36012.0, 10.0, 2.0) == 12.0);
  assert(!enoughSparseSupport(1, 5));
  assert(!enoughSparseSupport(3, 1));
  assert(enoughSparseSupport(3, 2));
  // A/C: a large near angular expectation cannot be bypassed at 30%
  // occlusion, even if the visibility model reports complete occlusion.
  double effective_expected = 0.0;
  for (int bins = 3; bins <= 6; ++bins) {
    for (double occlusion : {0.0, 0.2999, 0.3, 0.42, 0.9, 1.0}) {
      for (bool known : {false, true}) {
        assert(!enoughSparseSupport(bins, 2, 8400.0, occlusion, known,
                                    &effective_expected));
        assert(effective_expected >= 4200.0);
      }
    }
  }
  assert(!enoughSparseSupport(5, 2, 8400.0, 0.42, true, &effective_expected));
  assert(std::abs(effective_expected - 4872.0) < 1e-9);
  // B: known partial occlusion reduces the requirement without disabling it.
  assert(!enoughSparseSupport(120, 2, 2000.0, 0.0, true));
  assert(enoughSparseSupport(120, 2, 2000.0, 0.5, true, &effective_expected));
  assert(effective_expected == 1000.0);
  assert(!enoughSparseSupport(120, 1, 2000.0, 0.5, true));
  // An ownerless nearer return cannot grant occlusion relief. Even mixed
  // visibility with a known occluder cannot discount more than half the rays.
  assert(!enoughSparseSupport(120, 2, 2000.0, 1.0, false, &effective_expected));
  assert(effective_expected == 2000.0);
  assert(!enoughSparseSupport(6, 2, 2000.0, 1.0, true, &effective_expected));
  assert(effective_expected == 1000.0);
  // D: far observations with intrinsically few expected rays still work.
  for (int bins = 3; bins <= 6; ++bins)
    assert(enoughSparseSupport(bins, 2, 22.0, 0.0, false));
  assert(enoughSparseSupport(4, 2, 40.0, 0.0, false));
  assert(!enoughSparseSupport(4, 2, 40.01, 0.0, false));
  assert(enoughSparseSupport(3, 2, 60.0, 0.5, true));
  assert(!enoughSparseSupport(3, 2, 60.0, 0.5, false));
  assert(!enoughSparseSupport(4, 2, -1.0));
  assert(!enoughSparseSupport(4, 2, std::numeric_limits<double>::quiet_NaN()));
  // E: rejected raw support leaves a short dropout to the existing ACTIVE
  // prediction budget rather than immediately making the identity LOST.
  for (int missed = 1; missed <= 2; ++missed) {
    assert(!enoughSparseSupport(4, 2, 8400.0, 0.9, true));
    assert(vehicleLifecycleAfterMiss(VehicleLifecycle::ACTIVE, missed, 15) ==
           VehicleLifecycle::ACTIVE);
  }
  assert(lateralSurfaceGain(70.0, 20.0, 60.0, false) == 0.05);
  assert(lateralMeasurementStd(0.4, 70.0, 20.0, 60.0, 1) > 2.0);
  assert(std::abs(lateralInnovationGate(2.0) - 0.6) < 1e-12);
  assert(lateralMeasurementAccepted(2.2, 2.0, 2.0, true, 2.0));
  assert(!lateralMeasurementAccepted(2.8, 2.0, 2.0, true, 2.0));

  // Far OBB yaw has zero gain; repeated near FULL yaw can move gradually.
  double reliable_yaw = 0.0;
  for (double degrees : {2.0, 4.0, 7.0, 10.0}) {
    const double measured = degrees * 3.14159265358979323846 / 180.0;
    reliable_yaw += yawUpdateGain(70.0, 20.0, 60.0) *
        angleDifference(alignedAxisYaw(measured, reliable_yaw), reliable_yaw);
  }
  assert(std::abs(reliable_yaw) < 1e-12);
  const double near_measured = 6.0 * 3.14159265358979323846 / 180.0;
  assert(stabilizedYaw(0.0, near_measured, false, 10.0, 20.0, 60.0) == 0.0);
  assert(stabilizedYaw(0.0, near_measured, false, 70.0, 20.0, 60.0) == 0.0);
  assert(stabilizedYaw(0.0, near_measured, true, 70.0, 20.0, 60.0) == 0.0);
  for (int i = 0; i < 10; ++i) {
    const double aligned = alignedAxisYaw(near_measured, reliable_yaw);
    assert(yawCandidateAllowed(aligned, reliable_yaw));
    reliable_yaw += yawUpdateGain(10.0, 20.0, 60.0) *
        angleDifference(aligned, reliable_yaw);
  }
  assert(reliable_yaw > 0.04 && reliable_yaw < near_measured);
  double mid_yaw = 0.0;
  for (int i = 0; i < 100; ++i)
    mid_yaw = stabilizedYaw(mid_yaw, 7.0 * 3.14159265358979323846 / 180.0,
                            true, 40.0, 20.0, 60.0);
  assert(mid_yaw < 2.0 * 3.14159265358979323846 / 180.0);

  LateralAnchor anchor;
  anchor.valid = true;
  anchor.y = 2.0;
  assert(!advanceLateralAnchor(anchor, 2.2, 2.15, 10.0, 20.0, 60.0, 0.3));
  assert(!advanceLateralAnchor(anchor, 2.24, 2.18, 10.0, 20.0, 60.0, 0.3));
  assert(advanceLateralAnchor(anchor, 2.28, 2.21, 10.0, 20.0, 60.0, 0.3));
  assert(anchor.y > 2.0 && anchor.y < 2.03);
  const double held = anchor.y;
  assert(!advanceLateralAnchor(anchor, 2.4, 2.2, 70.0, 20.0, 60.0, 0.5));
  assert(anchor.y == held);  // PARTIAL/SPARSE do not call this helper.
  // ACTIVE -> LOST -> same-ID reacquisition keeps the reliable geometry.
  const int original_id = 10;
  auto state = VehicleLifecycle::ACTIVE;
  state = vehicleLifecycleAfterMiss(state, 15, 15);
  assert(state == VehicleLifecycle::ACTIVE);
  state = vehicleLifecycleAfterMiss(state, 16, 15);
  assert(state == VehicleLifecycle::LOST);
  assert(!vehicleLostExpired(state, 0.8, 4.0));
  ReacquireEvidence evidence;
  evidence.predicted = {10.0, 3.0};
  evidence.observed = {11.5, 3.4};
  evidence.reliable = {{10.0, 3.0}, 0.0, 5.0, 2.3, true};
  evidence.observed_length = 1.6;
  evidence.observed_width = 1.2;
  evidence.lateral_anchor_y = 3.0;
  evidence.lateral_anchor_valid = true;
  evidence.variance_x = 0.04;
  evidence.variance_y = 0.04;
  evidence.velocity_x = 2.0;
  evidence.lost_age_sec = 0.8;
  evidence.memory_sec = 4.0;
  evidence.remembered_sample_distance = 0.3;
  const auto partial_reacquire = scoreReacquisition(evidence);
  assert(decideReacquisition(partial_reacquire.value, -1.0) ==
         ReacquireDecision::ACCEPTED);
  ReacquirePending pending;
  assert(!advanceReacquirePending(pending, {11.5, 3.4}, 1.0, {2.0, 0.0}));
  assert(advanceReacquirePending(pending, {11.7, 3.4}, 1.1, {2.0, 0.0}));
  state = VehicleLifecycle::ACTIVE;
  assert(original_id == 10 && state == VehicleLifecycle::ACTIVE);
  assert(evidence.reliable.length == 5.0 && evidence.reliable.width == 2.3);
  assert(evidence.reliable.yaw == 0.0 && evidence.lateral_anchor_y == 3.0);

  // Same corridor cannot overcome physically impossible longitudinal motion.
  evidence.observed = {-2.0, 3.0};
  assert(scoreReacquisition(evidence).value < 0.0);
  evidence.observed = {30.0, 20.0};
  assert(scoreReacquisition(evidence).value < 0.0);
  evidence.observed = {11.5, 3.4};
  evidence.observed_length = 9.0;
  assert(scoreReacquisition(evidence).value < 0.0);
  evidence.observed_length = 1.6;
  assert(decideReacquisition(0.75, 0.70) == ReacquireDecision::AMBIGUOUS);
  assert(decideReacquisition(0.75, 0.40) == ReacquireDecision::ACCEPTED);
  assert(decideReacquisition(0.50, -1.0) == ReacquireDecision::REJECTED);
  pending.coherent_frames = 0;
  assert(!advanceReacquirePending(pending, {11.5, 3.4}, 2.0, {2.0, 0.0}));
  assert(!advanceReacquirePending(pending, {15.0, 3.4}, 2.1, {2.0, 0.0}));
  assert(vehicleLostExpired(VehicleLifecycle::LOST, 4.01, 4.0));
  evidence.lost_age_sec = 4.01;
  assert(scoreReacquisition(evidence).value < 0.0);

  // The observed anchor remains a sensor-derived value while the KF coasts.
  cx128s2_bev_perception::LinearKalmanFilter coast;
  coast.initialize(19.0, 2.0, 0.1, 1.0);
  coast.predict(1.0, 1.5);
  coast.update(20.0, 2.0, 0.1);
  const Point2 last_observed{20.0, 2.0};
  double prediction_time = 1.0;
  for (double frame_time : {2.0, 3.0, 5.0, 65.0, 3605.0}) {
    const double step_end = predictiveStepEnd(state == VehicleLifecycle::ACTIVE ?
        VehicleLifecycle::LOST : state, prediction_time, frame_time, 1.0, 4.0);
    double remaining = step_end - prediction_time;
    while (remaining > 1e-9) {
      const double step = std::min(remaining, 0.2);
      coast.predict(step, 1.5);
      remaining -= step;
    }
    prediction_time = step_end;
    if (frame_time >= 5.0) state = VehicleLifecycle::DORMANT;
    assert(last_observed.x == 20.0 && last_observed.y == 2.0);
  }
  const Point2 frozen{coast.x(), coast.y()};
  assert(frozen.x > last_observed.x);
  assert(prediction_time == 5.0);
  assert(predictiveStepEnd(state, prediction_time, 36005.0, 1.0, 4.0) == prediction_time);
  const auto dormant_pose = externalVehiclePose(state, last_observed, frozen, true);
  assert(dormant_pose.x == frozen.x && dormant_pose.y == frozen.y);
  assert(externalPoseSource(VehicleLifecycle::ACTIVE) ==
         ExternalPoseSource::CURRENT_OBSERVATION);
  assert(externalPoseSource(VehicleLifecycle::LOST) ==
         ExternalPoseSource::KF_PREDICTION);
  assert(externalPoseSource(state) == ExternalPoseSource::FINAL_PREDICTED_FROZEN);
  assert(externalVehiclePose(VehicleLifecycle::DORMANT, last_observed, frozen, true).x !=
         last_observed.x);
  assert(!vehicleDormantExpired(state, 35999.9, 36000.0));
  assert(vehicleDormantExpired(state, 36000.1, 36000.0));

  // Long-term identity uses the observed corridor anchor and OBB axis, not
  // the old velocity or a prediction stretched across the dormant interval.
  DormantEvidence returning;
  returning.last_observed = {-32.0, 2.0};
  returning.candidate = {-10.0, 2.0};
  returning.reliable = {{-32.0, 2.0}, 0.0, 5.0, 2.3, true};
  returning.candidate_length = 4.8;
  returning.candidate_width = 2.2;
  returning.candidate_yaw = 3.14159265358979323846;
  returning.yaw_reliable = true;
  returning.inside_watch_roi = true;
  returning.lateral_anchor_y = 2.0;
  returning.lateral_anchor_valid = true;
  returning.dormant_age_sec = 1000.0;
  returning.identity_memory_sec = 36000.0;
  const auto score = scoreDormantReacquisition(returning);
  assert(decideReacquisition(score.value, -1.0) == ReacquireDecision::ACCEPTED);
  assert(score.yaw_error < 1e-12);
  ReacquirePending dormant_pending;
  assert(!advanceReacquirePending(dormant_pending, returning.candidate, 10.0, {0.0, 0.0}));
  assert(advanceReacquirePending(dormant_pending, {-9.9, 2.0}, 10.1, {0.0, 0.0}));
  assert(original_id == 10 && returning.reliable.length == 5.0 &&
         returning.reliable.width == 2.3 && returning.reliable.yaw == 0.0);
  returning.candidate = {-40.0, 2.0};
  assert(scoreDormantReacquisition(returning).value < 0.0);
  returning.candidate = {-10.0, 7.0};
  assert(scoreDormantReacquisition(returning).value < 0.0);
  returning.candidate = {-10.0, 2.0};
  returning.candidate_length = 1.0;
  assert(scoreDormantReacquisition(returning).value < 0.0);
  returning.candidate_length = 4.8;
  returning.inside_watch_roi = false;
  assert(scoreDormantReacquisition(returning).value < 0.0);
  returning.inside_watch_roi = true;
  returning.dormant_age_sec = 36000.1;
  assert(scoreDormantReacquisition(returning).value < 0.0);
  returning.dormant_age_sec=90;returning.candidate={-70,2};
  returning.candidate_length=.3;returning.candidate_width=1.7;returning.yaw_reliable=false;
  assert(!validateFifoExit(returning).valid);
  assert(validateFifoExit(returning,50).valid);
  assert(scoreDormantReacquisition(returning,50).value>=.58);
  returning.candidate_width=.1;
  assert(scoreDormantReacquisition(returning,50).value<0);
  SingleLaneVehicleQueue fifo;fifo.append(10);fifo.append(20);
  assert(chooseFifoExit(fifo,CorridorDirection::EXIT,true,true).id==10);
  assert(chooseFifoExit(fifo,CorridorDirection::EXIT,true,false).decision==FifoExitDecision::REJECT);
  assert(chooseFifoExit(fifo,CorridorDirection::ENTRY,true,true).decision==FifoExitDecision::NOT_EXIT);
  assert(chooseFifoExit(fifo,CorridorDirection::ENTRY,true,true,true).id==10);
  assert(decideReacquisition(0.72, 0.69) == ReacquireDecision::AMBIGUOUS);
  return 0;
}
