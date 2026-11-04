#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>
#include <vector>
namespace cx128s2_bev_perception::vehicle_surface {
struct Point2 { double x = 0.0, y = 0.0; };
struct Obb { Point2 center; double yaw = 0.0, length = 0.0, width = 0.0; bool valid = false; };
inline double angleDifference(double a, double b) { return std::remainder(a - b, 2.0 * 3.14159265358979323846); }
inline double alignedAxisYaw(double candidate, double reference) {
  const double opposite = candidate + 3.14159265358979323846;
  return reference + (std::abs(angleDifference(candidate, reference)) <= std::abs(angleDifference(opposite, reference)) ?
      angleDifference(candidate, reference) : angleDifference(opposite, reference));
}
inline Point2 toLocal(Point2 point, Point2 center, double yaw) {
  const double c = std::cos(yaw), s = std::sin(yaw), dx = point.x - center.x, dy = point.y - center.y;
  return {c * dx + s * dy, -s * dx + c * dy};
}
inline Point2 toWorld(Point2 point, Point2 center, double yaw) {
  const double c = std::cos(yaw), s = std::sin(yaw);
  return {center.x + c * point.x - s * point.y, center.y + s * point.x + c * point.y};
}
inline Obb estimateObb(const std::vector<Point2> & points, double reference_yaw = std::numeric_limits<double>::quiet_NaN()) {
  Obb out;
  if (points.size() < 4) return out;
  Point2 mean;
  for (const auto & p : points) { mean.x += p.x; mean.y += p.y; }
  mean.x /= points.size(); mean.y /= points.size();
  double xx = 0.0, xy = 0.0, yy = 0.0;
  for (const auto & p : points) { const double dx = p.x - mean.x, dy = p.y - mean.y; xx += dx * dx; xy += dx * dy; yy += dy * dy; }
  double yaw = 0.5 * std::atan2(2.0 * xy, xx - yy);
  auto bounds = [&](double axis) {
    std::array<double, 4> b{1e30, -1e30, 1e30, -1e30};
    for (const auto & p : points) { const auto q = toLocal(p, mean, axis);
      b[0] = std::min(b[0], q.x); b[1] = std::max(b[1], q.x);
      b[2] = std::min(b[2], q.y); b[3] = std::max(b[3], q.y); }
    return b;
  };
  auto b = bounds(yaw);
  if (b[1] - b[0] < b[3] - b[2]) { yaw += 3.14159265358979323846 / 2.0; b = bounds(yaw); }
  if (std::isfinite(reference_yaw)) yaw = alignedAxisYaw(yaw, reference_yaw);
  out.center = toWorld({(b[0] + b[1]) * 0.5, (b[2] + b[3]) * 0.5}, mean, yaw);
  out.yaw = yaw; out.length = b[1] - b[0]; out.width = b[3] - b[2];
  out.valid = std::isfinite(out.length) && out.length > 0.1 && out.width > 0.1;
  return out;
}
// Project directly onto the site entrance axes; never estimate a PCA yaw.
inline Obb fixedYawObb(const std::vector<Point2> & points, double yaw) {
  Obb out; out.yaw = yaw;
  if (points.size() < 4 || !std::isfinite(yaw)) return out;
  std::array<double, 4> bounds{1e30, -1e30, 1e30, -1e30};
  for (const auto & p : points) {
    const auto q = toLocal(p, {0.0, 0.0}, yaw);
    bounds[0] = std::min(bounds[0], q.x); bounds[1] = std::max(bounds[1], q.x);
    bounds[2] = std::min(bounds[2], q.y); bounds[3] = std::max(bounds[3], q.y);
  }
  out.center = toWorld({(bounds[0] + bounds[1]) * .5,
                       (bounds[2] + bounds[3]) * .5}, {0.0, 0.0}, yaw);
  out.length = bounds[1] - bounds[0]; out.width = bounds[3] - bounds[2];
  out.valid = std::isfinite(out.length) && std::isfinite(out.width) &&
              out.length > .1 && out.width > .1;
  return out;
}
inline std::vector<Point2> faceSamples(double length, double width) {
  std::vector<Point2> out; out.reserve(32);
  for (int i = 0; i < 8; ++i) { const double t = (i + 0.5) / 8.0 - 0.5;
    out.push_back({t * length, -width * 0.5}); out.push_back({t * length, width * 0.5});
    out.push_back({-length * 0.5, t * width}); out.push_back({length * 0.5, t * width}); }
  return out;
}
inline double surfaceDistance(Point2 point, Point2 center, double yaw, double length, double width) {
  const auto p = toLocal(point, center, yaw);
  return std::hypot(std::max(std::abs(p.x) - length * 0.5, 0.0), std::max(std::abs(p.y) - width * 0.5, 0.0));
}
inline bool extensionAllowed(double old_length, double old_width, double new_length, double new_width,
                             int stable_frames, double max_length, double max_width) {
  return stable_frames >= 3 && new_length <= max_length && new_width <= max_width &&
      new_length <= old_length * 1.25 && new_width <= old_width * 1.25;
}
inline bool applyReliableGeometry(Obb & reliable, const Obb & observed,
                                  int stable_frames, double max_length, double max_width,
                                  bool matched_end_face = false) {
  // A supported, repeatedly matched end face constrains identity independently
  // of how quickly additional longitudinal extent enters the field of view.
  const bool anchored_extension = matched_end_face && stable_frames >= 3 &&
      observed.length <= max_length && observed.width <= max_width &&
      observed.width <= reliable.width * 1.25;
  if (!reliable.valid || !observed.valid ||
      observed.length <= reliable.length ||
      (!anchored_extension && !extensionAllowed(reliable.length, reliable.width, observed.length, observed.width,
                        stable_frames, max_length, max_width))) return false;
  reliable.length = observed.length;
  return true;
}
inline double lateralSurfaceGain(double range, double near_max, double mid_max,
                                 bool sparse) {
  if (sparse || range >= mid_max) return 0.05;
  return range < near_max ? 0.5 : 0.15;
}
inline double lateralMeasurementStd(double base_std, double range,
                                    double near_max, double mid_max,
                                    int quality_rank) {
  const int band = range < near_max ? 0 : (range < mid_max ? 1 : 2);
  const double multiplier[3][3] = {{2.0, 6.0, 10.0},
                                   {3.0, 5.0, 8.0},
                                   {4.0, 6.0, 8.0}};
  const int rank = std::clamp(quality_rank, 0, 2);
  return std::min(4.0, base_std * multiplier[rank][band]);
}
inline double lateralInnovationGate(double reliable_width) {
  return std::clamp(0.3 * reliable_width, 0.35, 0.8);
}
inline double yawUpdateGain(double range, double near_max, double mid_max) {
  return range < near_max ? 0.08 : (range < mid_max ? 0.002 : 0.0);
}
inline bool yawCandidateAllowed(double aligned_candidate, double reliable_yaw) {
  return std::abs(angleDifference(aligned_candidate, reliable_yaw)) <= 0.15;
}
inline double stabilizedYaw(double reliable_yaw, double candidate_yaw,
                            bool reliable_full, double range,
                            double near_max, double mid_max) {
  if (!reliable_full) return reliable_yaw;
  const double aligned = alignedAxisYaw(candidate_yaw, reliable_yaw);
  if (!yawCandidateAllowed(aligned, reliable_yaw)) return reliable_yaw;
  return reliable_yaw + yawUpdateGain(range, near_max, mid_max) *
      angleDifference(aligned, reliable_yaw);
}
inline bool lateralMeasurementAccepted(double measured_y, double predicted_y,
                                       double anchor_y, bool anchor_valid,
                                       double reliable_width) {
  return std::abs(measured_y - predicted_y) <=
      lateralInnovationGate(reliable_width) &&
      (!anchor_valid || std::abs(measured_y - anchor_y) <=
       std::max(0.75, reliable_width * 0.5));
}
struct LateralAnchor {
  double y = 0.0;
  double last_candidate_y = std::numeric_limits<double>::quiet_NaN();
  int consistent_frames = 0;
  bool valid = false;
};
inline bool advanceLateralAnchor(LateralAnchor & anchor, double candidate_y,
                                 double predicted_y, double range,
                                 double near_max, double mid_max,
                                 double support_ratio) {
  if (!anchor.valid) return false;
  const bool near = range < near_max;
  if (range >= mid_max || support_ratio < (near ? 0.2 : 0.4) ||
      std::abs(candidate_y - predicted_y) > 0.45 ||
      std::abs(candidate_y - anchor.y) < 0.05) {
    anchor.consistent_frames = 0;
    anchor.last_candidate_y = std::numeric_limits<double>::quiet_NaN();
    return false;
  }
  if (std::isfinite(anchor.last_candidate_y) &&
      std::abs(candidate_y - anchor.last_candidate_y) <= 0.15 &&
      (candidate_y - anchor.y) * (anchor.last_candidate_y - anchor.y) > 0.0)
    ++anchor.consistent_frames;
  else anchor.consistent_frames = 1;
  anchor.last_candidate_y = candidate_y;
  if (anchor.consistent_frames < 3) return false;
  const double step = std::clamp((near ? 0.05 : 0.002) *
                                 (candidate_y - anchor.y), -0.03, 0.03);
  anchor.y += step;
  return true;
}
inline double componentOwnershipScore(const Obb & geometry, Point2 predicted_center,
                                      Point2 component_center, double gate) {
  if (!geometry.valid || gate <= 0.0) return -1.0;
  const double distance = std::hypot(component_center.x - predicted_center.x,
                                     component_center.y - predicted_center.y);
  if (distance > gate + geometry.length * 0.5) return -1.0;
  const double surface = surfaceDistance(component_center, predicted_center,
                                         geometry.yaw, geometry.length, geometry.width);
  if (surface > 0.8) return -1.0;
  return 1.0 - surface / 0.8 -
      distance / std::max(1.0, gate + geometry.length * 0.5) * 0.3;
}
struct OwnershipDecision {
  std::vector<int> owner;
  std::vector<int> primary;
  std::vector<bool> ambiguous;
};
inline OwnershipDecision assignComponentOwners(
    const std::vector<std::vector<double>> & scores,
    const std::vector<int> & point_counts, size_t track_count, double margin = 0.12) {
  OwnershipDecision out;
  out.owner.assign(scores.size(), -1);
  out.primary.assign(track_count, -1);
  out.ambiguous.assign(scores.size(), false);
  for (size_t fi = 0; fi < scores.size(); ++fi) {
    int best = -1;
    double first = -1.0, second = -1.0;
    for (size_t ti = 0; ti < std::min(track_count, scores[fi].size()); ++ti) {
      const double score = scores[fi][ti];
      if (score < 0.0) continue;
      if (score > first) { second = first; first = score; best = static_cast<int>(ti); }
      else second = std::max(second, score);
    }
    if (best < 0) continue;
    if (first - second < margin) { out.ambiguous[fi] = true; continue; }
    out.owner[fi] = best;
    if (out.primary[best] < 0 ||
        point_counts[fi] > point_counts[out.primary[best]]) out.primary[best] = fi;
  }
  return out;
}
enum class VehicleLifecycle { ACTIVE, LOST, DORMANT };
inline VehicleLifecycle vehicleLifecycleAfterMiss(VehicleLifecycle state,
                                                  int lost_frames, int active_limit) {
  return state == VehicleLifecycle::ACTIVE && lost_frames > active_limit ?
      VehicleLifecycle::LOST : state;
}
inline bool vehicleLostExpired(VehicleLifecycle state, double lost_age_sec,
                               double memory_sec) {
  return state == VehicleLifecycle::LOST && lost_age_sec > memory_sec;
}
inline double predictiveStepEnd(VehicleLifecycle state, double previous_time,
                                double frame_time, double lost_since,
                                double prediction_window_sec) {
  if (state == VehicleLifecycle::DORMANT) return previous_time;
  if (state == VehicleLifecycle::LOST)
    return std::max(previous_time,
        std::min(frame_time, lost_since + prediction_window_sec));
  return frame_time;
}
inline bool vehicleDormantExpired(VehicleLifecycle state, double dormant_age_sec,
                                  double identity_memory_sec) {
  return state == VehicleLifecycle::DORMANT && dormant_age_sec > identity_memory_sec;
}
enum class ExternalPoseSource { CURRENT_OBSERVATION, KF_PREDICTION, FINAL_PREDICTED_FROZEN };
inline ExternalPoseSource externalPoseSource(VehicleLifecycle state) {
  if (state == VehicleLifecycle::DORMANT) return ExternalPoseSource::FINAL_PREDICTED_FROZEN;
  if (state == VehicleLifecycle::LOST) return ExternalPoseSource::KF_PREDICTION;
  return ExternalPoseSource::CURRENT_OBSERVATION;
}
inline Point2 externalVehiclePose(VehicleLifecycle state, Point2 current,
                                  Point2 final_predicted, bool final_valid) {
  return state == VehicleLifecycle::DORMANT && final_valid ? final_predicted : current;
}
struct DormantEvidence {
  Point2 last_observed, candidate;
  Obb reliable;
  double candidate_length = 0.0, candidate_width = 0.0;
  double candidate_yaw = 0.0;
  bool yaw_reliable = false, inside_watch_roi = true;
  double lateral_anchor_y = 0.0;
  bool lateral_anchor_valid = false;
  double dormant_age_sec = 0.0, identity_memory_sec = 0.0;
};
struct ReacquireEvidence {
  Point2 predicted, observed;
  Obb reliable;
  double observed_length = 0.0, observed_width = 0.0;
  double observed_yaw = 0.0;
  bool yaw_reliable = false;
  double lateral_anchor_y = 0.0;
  bool lateral_anchor_valid = false;
  double variance_x = 0.0, variance_y = 0.0;
  double velocity_x = 0.0;
  double lost_age_sec = 0.0, memory_sec = 0.0;
  double remembered_sample_distance = -1.0;
};
struct ReacquireScore {
  double value = -1.0;
  double position_error = 0.0, lateral_error = 0.0;
  double yaw_error = 0.0, surface_error = 0.0;
  const char * reason = "REJECTED";
};
inline ReacquireScore scoreReacquisition(const ReacquireEvidence & e) {
  ReacquireScore out;
  const double dx = e.observed.x - e.predicted.x;
  const double dy = e.observed.y - e.predicted.y;
  out.position_error = std::hypot(dx, dy);
  out.lateral_error = e.lateral_anchor_valid ?
      std::abs(e.observed.y - e.lateral_anchor_y) : std::abs(dy);
  if (!e.reliable.valid || e.lost_age_sec < 0.0 ||
      e.lost_age_sec > e.memory_sec) { out.reason = "TIME_OR_GEOMETRY"; return out; }
  const double sigma_x = std::sqrt(std::max(0.0, e.variance_x));
  const double sigma_y = std::sqrt(std::max(0.0, e.variance_y));
  const double x_gate = std::min(6.0, std::max(1.5, 3.0 * sigma_x + e.reliable.length * 0.5));
  const double y_gate = std::max(1.1, 0.75 * e.reliable.width + 0.4);
  if (std::abs(dx) > x_gate || std::abs(dy) > y_gate ||
      out.lateral_error > y_gate + std::min(0.4, 2.0 * sigma_y)) {
    out.reason = "POSITION_OR_LATERAL"; return out;
  }
  if (e.observed_length > 1.5 * e.reliable.length ||
      e.observed_width > 1.6 * e.reliable.width) {
    out.reason = "OVERSIZED_MERGE"; return out;
  }
  out.surface_error = surfaceDistance(e.observed, e.predicted,
      e.reliable.yaw, e.reliable.length, e.reliable.width);
  if (out.surface_error > 0.9) { out.reason = "SURFACE"; return out; }
  if (e.yaw_reliable) {
    out.yaw_error = std::abs(angleDifference(
        alignedAxisYaw(e.observed_yaw, e.reliable.yaw), e.reliable.yaw));
    if (out.yaw_error > 0.5) { out.reason = "YAW"; return out; }
  }
  const double position_score = std::max(0.0, 1.0 - std::abs(dx) / x_gate);
  const double lateral_score = std::max(0.0, 1.0 - out.lateral_error /
      (y_gate + std::min(0.4, 2.0 * sigma_y)));
  const double box_surface_score = std::max(0.0, 1.0 - out.surface_error / 0.9);
  const double sample_score = e.remembered_sample_distance >= 0.0 ?
      std::max(0.0, 1.0 - e.remembered_sample_distance / 1.5) : box_surface_score;
  const double surface_score = 0.5 * box_surface_score + 0.5 * sample_score;
  const double yaw_score = e.yaw_reliable ?
      std::max(0.0, 1.0 - out.yaw_error / 0.5) : 0.7;
  const double geometry_score = std::clamp(1.0 -
      std::max(e.observed_length / e.reliable.length - 1.0,
               e.observed_width / e.reliable.width - 1.0), 0.0, 1.0);
  // Reversal remains possible; longitudinal motion is a soft cue only.
  const double motion_scale = std::max(0.5, std::abs(e.velocity_x) *
      e.lost_age_sec + 3.0 * sigma_x);
  const double motion_score = std::max(0.0, 1.0 - std::abs(dx) / motion_scale);
  const double time_score = std::max(0.0, 1.0 - e.lost_age_sec / e.memory_sec);
  out.value = 0.20 * position_score + 0.22 * lateral_score +
      0.22 * surface_score + 0.08 * yaw_score +
      0.12 * geometry_score + 0.10 * motion_score + 0.06 * time_score;
  out.reason = "SCORED";
  return out;
}
inline ReacquireScore scoreDormantReacquisition(const DormantEvidence & e,
                                                double reachable_progress_m = 3.0) {
  ReacquireScore out;
  if (!e.reliable.valid || !e.inside_watch_roi ||
      e.dormant_age_sec < 0.0 || e.dormant_age_sec > e.identity_memory_sec) {
    out.reason = "TIME_OR_CORRIDOR"; return out;
  }
  const double origin_distance = std::hypot(e.last_observed.x, e.last_observed.y);
  // Near the origin the exit direction is numerically unstable.  Require
  // proximity to the observed anchor instead of extrapolating a direction.
  const Point2 toward_origin = origin_distance >= 1.0 ?
      Point2{-e.last_observed.x / origin_distance,
             -e.last_observed.y / origin_distance} : Point2{0.0, 0.0};
  const double dx = e.candidate.x - e.last_observed.x;
  const double dy = e.candidate.y - e.last_observed.y;
  const double progress = dx * toward_origin.x + dy * toward_origin.y;
  const double cross_track = origin_distance >= 1.0 ?
      std::abs(dx * toward_origin.y - dy * toward_origin.x) : std::hypot(dx, dy);
  out.position_error = std::hypot(dx, dy);
  out.lateral_error = e.lateral_anchor_valid ?
      std::abs(e.candidate.y - e.lateral_anchor_y) : cross_track;
  const double corridor_gate = std::max(2.2, 1.2 * e.reliable.width);
  if (progress < -reachable_progress_m || cross_track > corridor_gate ||
      out.lateral_error > corridor_gate) {
    out.reason = "RETURN_CORRIDOR"; return out;
  }
  // Long-term matching needs a substantial box; a small unrelated person or
  // pallet cannot claim a vehicle identity based on lane and time alone.
  if ((e.yaw_reliable && (e.candidate_length < 0.55 * e.reliable.length ||
                         e.candidate_width < 0.60 * e.reliable.width)) ||
      (!e.yaw_reliable && e.candidate_width < std::max(0.4, 0.25 * e.reliable.width) &&
       e.candidate_length < std::max(0.8, 0.2 * e.reliable.length))) {
    out.reason = "INSUFFICIENT_GEOMETRY"; return out;
  }
  if (e.candidate_length > 1.5 * e.reliable.length ||
      e.candidate_width > 1.6 * e.reliable.width) {
    out.reason = "OVERSIZED_MERGE"; return out;
  }
  if (e.yaw_reliable) {
    out.yaw_error = std::abs(angleDifference(
        alignedAxisYaw(e.candidate_yaw, e.reliable.yaw), e.reliable.yaw));
    if (out.yaw_error > 0.5) { out.reason = "YAW_AXIS"; return out; }
  }
  const double lateral_score = std::max(0.0, 1.0 - out.lateral_error / corridor_gate);
  const double axis_score = e.yaw_reliable ?
      std::max(0.0, 1.0 - out.yaw_error / 0.5) : 0.65;
  const double geometry_score = std::clamp(1.0 -
      std::max(e.candidate_length / e.reliable.length - 1.0,
               e.candidate_width / e.reliable.width - 1.0), 0.0, 1.0);
  const double return_score = progress >= 0.0 ? 1.0 :
      std::max(0.0, 1.0 + progress / std::max(3.0, reachable_progress_m));
  const double surface_shape_score = std::clamp(
      std::min(e.candidate_width / e.reliable.width,
               e.candidate_length / e.reliable.length), 0.0, 1.0);
  const double time_score = std::max(0.0, 1.0 -
      e.dormant_age_sec / e.identity_memory_sec);
  out.value = 0.30 * lateral_score + 0.15 * axis_score +
      0.20 * geometry_score + 0.15 * return_score +
      0.15 * surface_shape_score + 0.05 * time_score;
  out.reason = "SCORED";
  return out;
}
struct ReacquirePending {
  Point2 center;
  double stamp = 0.0;
  int coherent_frames = 0;
};
inline bool advanceReacquirePending(ReacquirePending & pending, Point2 center,
                                    double stamp, Point2 velocity) {
  const double dt = stamp - pending.stamp;
  if (pending.coherent_frames > 0 && dt > 0.0 && dt < 0.6 &&
      std::hypot(center.x - pending.center.x - velocity.x * dt,
                 center.y - pending.center.y - velocity.y * dt) < 1.0)
    ++pending.coherent_frames;
  else pending.coherent_frames = 1;
  pending.center = center;
  pending.stamp = stamp;
  return pending.coherent_frames >= 2;
}
enum class ReacquireDecision { REJECTED, AMBIGUOUS, ACCEPTED };
inline ReacquireDecision decideReacquisition(double best, double second,
                                              double minimum = 0.58,
                                              double margin = 0.12) {
  if (best < minimum) return ReacquireDecision::REJECTED;
  if (second >= 0.0 && best - second < margin) return ReacquireDecision::AMBIGUOUS;
  return ReacquireDecision::ACCEPTED;
}
inline bool enoughSparseSupport(int independent_bins, int coherent_frames,
                                double expected_rays = 0.0,
                                double occluded_fraction = 0.0,
                                bool has_known_occluder = false,
                                double * effective_expected_support = nullptr) {
  // Zero expected rays preserves the existing cluster-based temporal gate.
  // Raw observations supply a positive angular expectation. Unknown nearer
  // returns alone cannot lower it; mixed/known occlusion gets at most 50%
  // relief because the visibility aggregate cannot separate their counts.
  const double relief = has_known_occluder && std::isfinite(occluded_fraction) ?
      std::clamp(occluded_fraction, 0.0, 0.5) : 0.0;
  const double effective = expected_rays * (1.0 - relief);
  if (effective_expected_support) *effective_expected_support = effective;
  return independent_bins >= 3 && coherent_frames >= 2 &&
      std::isfinite(expected_rays) && expected_rays >= 0.0 &&
      static_cast<double>(independent_bins) * 10.0 >= effective;
}
// This queue represents the site's single lane: confirmed entry order is the
// only permissible exit identity order.  Generic Track IDs are never enqueued.
enum class CorridorDirection { UNKNOWN, ENTRY, EXIT };
inline const char * directionName(CorridorDirection direction) {
  switch (direction) {
    case CorridorDirection::ENTRY: return "ENTRY";
    case CorridorDirection::EXIT: return "EXIT";
    default: return "UNKNOWN";
  }
}
inline CorridorDirection signedCorridorDirection(Point2 position, Point2 velocity,
                                                   double minimum_speed) {
  const double distance = std::hypot(position.x, position.y);
  if (distance < 1.0 || std::hypot(velocity.x, velocity.y) < minimum_speed)
    return CorridorDirection::UNKNOWN;
  const double toward_exit = -(position.x * velocity.x + position.y * velocity.y) / distance;
  if (toward_exit >= minimum_speed) return CorridorDirection::EXIT;
  if (toward_exit <= -minimum_speed) return CorridorDirection::ENTRY;
  return CorridorDirection::UNKNOWN;
}
struct DirectionMemory {
  CorridorDirection stable = CorridorDirection::UNKNOWN;
  CorridorDirection candidate = CorridorDirection::UNKNOWN;
  int consecutive = 0;
  double signed_speed = 0.0;
};
inline CorridorDirection advanceCorridorDirection(DirectionMemory & memory,
                                                 Point2 position, Point2 velocity,
                                                 double minimum_speed,
                                                 int required_frames = 2) {
  const double distance = std::hypot(position.x, position.y);
  memory.signed_speed = distance >= 1.0 ?
      -(position.x * velocity.x + position.y * velocity.y) / distance : 0.0;
  const auto observed = signedCorridorDirection(position, velocity, minimum_speed);
  if (observed == CorridorDirection::UNKNOWN) {
    memory.candidate = CorridorDirection::UNKNOWN;
    memory.consecutive = 0;
    return memory.stable;
  }
  if (observed == memory.candidate) ++memory.consecutive;
  else { memory.candidate = observed; memory.consecutive = 1; }
  if (memory.consecutive >= required_frames) memory.stable = observed;
  return memory.stable;
}
enum class FifoPhase { NOT_REGISTERED, INSIDE, EXITING };
inline const char * fifoPhaseName(FifoPhase phase) {
  switch (phase) {
    case FifoPhase::INSIDE: return "INSIDE";
    case FifoPhase::EXITING: return "EXITING";
    default: return "NOT_REGISTERED";
  }
}
class SingleLaneVehicleQueue {
public:
  bool append(int id) {
    if (id <= 0 || contains(id)) return false;
    ids_.push_back(id);
    return true;
  }
  bool contains(int id) const {
    return std::find(ids_.begin(), ids_.end(), id) != ids_.end();
  }
  int front() const { return ids_.empty() ? -1 : ids_.front(); }
  bool remove(int id) {
    const auto it = std::find(ids_.begin(), ids_.end(), id);
    if (it == ids_.end()) return false;
    ids_.erase(it);
    return true;
  }
  void clear() { ids_.clear(); }
  size_t size() const { return ids_.size(); }
  const std::deque<int> & ids() const { return ids_; }
  bool unique() const {
    for (size_t i = 0; i < ids_.size(); ++i)
      for (size_t j = i + 1; j < ids_.size(); ++j)
        if (ids_[i] == ids_[j]) return false;
    return true;
  }
private:
  std::deque<int> ids_;
};
struct FifoSanity {
  bool valid = false;
  const char * reason = "UNKNOWN";
};
inline FifoSanity validateFifoExit(const DormantEvidence & e,
                                   double reachable_progress_m = 3.0) {
  if (!e.reliable.valid || !e.inside_watch_roi ||
      e.dormant_age_sec < 0.0 || e.dormant_age_sec > e.identity_memory_sec)
    return {false, "TIME_OR_WATCH_ROI"};
  const double d = std::hypot(e.last_observed.x, e.last_observed.y);
  const double dx = e.candidate.x - e.last_observed.x;
  const double dy = e.candidate.y - e.last_observed.y;
  const double progress = d >= 1.0 ?
      -(dx * e.last_observed.x + dy * e.last_observed.y) / d : 0.0;
  const double lateral = d >= 1.0 ?
      std::abs(dx * e.last_observed.y - dy * e.last_observed.x) / d : std::hypot(dx, dy);
  const double gate = std::max(2.8, 1.5 * e.reliable.width);
  if (progress < -reachable_progress_m || lateral > gate ||
      (e.lateral_anchor_valid && std::abs(e.candidate.y - e.lateral_anchor_y) > gate))
    return {false, "RETURN_CORRIDOR"};
  // Small views are allowed.  Only impossible oversize/merge is rejected.
  if (e.candidate_length > 1.5 * e.reliable.length ||
      e.candidate_width > 1.6 * e.reliable.width)
    return {false, "OVERSIZED_MERGE"};
  if (e.yaw_reliable && std::abs(angleDifference(
      alignedAxisYaw(e.candidate_yaw, e.reliable.yaw), e.reliable.yaw)) > 0.5)
    return {false, "YAW_AXIS"};
  return {true, "PASS"};
}
enum class FifoExitDecision { NOT_EXIT, NO_FRONT, FRONT_BUSY, REJECT, RESERVE };
struct FifoExitChoice { int id = -1; FifoExitDecision decision = FifoExitDecision::NOT_EXIT; };
inline FifoExitChoice chooseFifoExit(const SingleLaneVehicleQueue & queue,
                                    CorridorDirection direction,
                                    bool front_dormant, bool sanity_valid,
                                    bool allow_continued_entry = false) {
  if (direction != CorridorDirection::EXIT &&
      !(allow_continued_entry && direction == CorridorDirection::ENTRY))
    return {-1, FifoExitDecision::NOT_EXIT};
  const int id = queue.front();
  if (id < 0) return {-1, FifoExitDecision::NO_FRONT};
  if (!front_dormant) return {id, FifoExitDecision::FRONT_BUSY};
  if (!sanity_valid) return {id, FifoExitDecision::REJECT};
  return {id, FifoExitDecision::RESERVE};
}
// Both arguments must be successive real observed centers, never KF predictions.
inline bool fifoExitComplete(Point2 previous_observed, Point2 current_observed,
                             double reliable_length) {
  const double gate = std::max(5.0, reliable_length);
  return std::hypot(previous_observed.x, previous_observed.y) > gate &&
         std::hypot(current_observed.x, current_observed.y) <= gate;
}
}  // namespace cx128s2_bev_perception::vehicle_surface
