#include <cx128s2_bev_perception/visibility_model.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace cx128s2_bev_perception {
namespace {
constexpr double kPi = 3.14159265358979323846;
double rad(double degrees) { return degrees * kPi / 180.0; }
double deg(double radians) { return radians * 180.0 / kPi; }
}  // namespace

const char * visibilityReasonName(VisibilityReason reason) {
  switch (reason) {
    case VisibilityReason::OBSERVABLE: return "OBSERVABLE";
    case VisibilityReason::OUT_OF_HORIZONTAL_FOV: return "OUT_OF_HORIZONTAL_FOV";
    case VisibilityReason::OUT_OF_VERTICAL_FOV: return "OUT_OF_VERTICAL_FOV";
    case VisibilityReason::TOO_CLOSE: return "TOO_CLOSE";
    case VisibilityReason::TOO_FAR: return "TOO_FAR";
    case VisibilityReason::NEAR_BLIND: return "NEAR_BLIND";
    case VisibilityReason::OCCLUDED_BY_OTHER: return "OCCLUDED_BY_OTHER";
    case VisibilityReason::NEARER_RETURN_UNKNOWN: return "NEARER_RETURN_UNKNOWN";
    case VisibilityReason::SELF_OCCLUSION_POSSIBLE: return "SELF_OCCLUSION_POSSIBLE";
    case VisibilityReason::VISIBLE_EMPTY: return "VISIBLE_EMPTY";
    case VisibilityReason::FULLY_OCCLUDED_LIKELY: return "FULLY_OCCLUDED_LIKELY";
    default: return "UNKNOWN";
  }
}

void VisibilityModel::configure(const VisibilityConfig & config) {
  config_ = config;
  config_.azimuth_bin_deg = std::max(0.02, config_.azimuth_bin_deg);
  config_.elevation_bin_deg = std::max(0.02, config_.elevation_bin_deg);
  config_.ray_elevation_spacing_deg = config_.ray_elevation_spacing_deg > 0.0 ?
      std::max(0.02, config_.ray_elevation_spacing_deg) : config_.elevation_bin_deg;
  config_.static_xy_resolution_m = std::max(0.05, config_.static_xy_resolution_m);
  pose_valid_ = false;
  clearStaticCache();
  az_bin_min_ = static_cast<int>(std::floor(config_.azimuth_min_deg / config_.azimuth_bin_deg)) - 2;
  el_bin_min_ = static_cast<int>(std::floor(config_.elevation_min_deg / config_.elevation_bin_deg)) - 2;
  az_bin_count_ = static_cast<int>(std::ceil((config_.azimuth_max_deg - config_.azimuth_min_deg) /
                                             config_.azimuth_bin_deg)) + 5;
  el_bin_count_ = static_cast<int>(std::ceil((config_.elevation_max_deg - config_.elevation_min_deg) /
                                             config_.elevation_bin_deg)) + 5;
  if (az_bin_count_ <= 0 || el_bin_count_ <= 0 ||
      static_cast<int64_t>(az_bin_count_) * el_bin_count_ > 2000000) {
    throw std::invalid_argument("visibility angular depth grid is too large");
  }
  depth_.assign(static_cast<size_t>(az_bin_count_) * el_bin_count_, DepthCell{});
  occupied_depth_indices_.clear();
  occupied_depth_cells_ = 0;
}

bool VisibilityModel::setPose(const tf2::Transform & world_from_sensor) {
  const auto delta = world_from_sensor.getOrigin() - world_from_sensor_.getOrigin();
  const double rotation_dot = std::abs(world_from_sensor.getRotation().dot(world_from_sensor_.getRotation()));
  if (!pose_valid_ || delta.length2() > 1e-8 || rotation_dot < 1.0 - 1e-9) {
    world_from_sensor_ = world_from_sensor;
    sensor_from_world_ = world_from_sensor.inverse();
    pose_valid_ = true;
    steepest_downward_slope_ = 0.0;
    for (int i = 0; i <= 16; ++i) {
      const double az = rad(config_.azimuth_min_deg +
          (config_.azimuth_max_deg - config_.azimuth_min_deg) * i / 16.0);
      for (double el : {config_.elevation_min_deg, config_.elevation_max_deg}) {
        const double er = rad(el);
        const tf2::Vector3 ray = tf2::quatRotate(world_from_sensor_.getRotation(),
            tf2::Vector3(std::cos(er) * std::cos(az), std::cos(er) * std::sin(az), std::sin(er)));
        if (ray.z() < -1e-6) {
          steepest_downward_slope_ = std::max(steepest_downward_slope_,
              -ray.z() / std::hypot(ray.x(), ray.y()));
        }
      }
    }
    clearStaticCache();
    return true;
  }
  return false;
}

void VisibilityModel::clearStaticCache() {
  static_cache_.clear();
  static_cache_hits_ = static_cache_misses_ = 0;
}

void VisibilityModel::setRayElevationSpacing(double spacing_deg) {
  if (std::isfinite(spacing_deg) && spacing_deg >= 0.02 && spacing_deg <= 2.0)
    config_.ray_elevation_spacing_deg = spacing_deg;
}

VisibilityInfo VisibilityModel::cacheStaticPoint(double x, double y, double z) {
  const auto result = evaluateStaticPoint(x, y, z);
  if (pose_valid_) {
    const auto a = angles(x, y, z);
    // Store only explicitly selected static grid samples, never arbitrary
    // cluster centers or predicted vehicle surfaces.
    static_cache_.emplace(staticKey(x, y, z), staticAt(x, y, z, a));
  }
  return result;
}

VisibilityModel::Angles VisibilityModel::angles(double x, double y, double z) const {
  const tf2::Vector3 p = sensor_from_world_ * tf2::Vector3(x, y, z);
  Angles a;
  a.range = p.length();
  a.az = deg(std::atan2(p.y(), p.x()));
  if (a.az < 0.0) a.az += 360.0;
  a.el = deg(std::atan2(p.z(), std::hypot(p.x(), p.y())));
  return a;
}

VisibilityModel::CellKey VisibilityModel::staticKey(double x, double y, double z) const {
  // Quantize XYZ, including height: a ground cell must not stand for a roof cell.
  const auto q = [&](double v) { return static_cast<int>(std::floor(v / config_.static_xy_resolution_m)); };
  return {q(x), q(y), q(z)};
}

int VisibilityModel::depthIndex(int az_bin, int el_bin) const {
  const int a = az_bin - az_bin_min_;
  const int e = el_bin - el_bin_min_;
  if (a < 0 || a >= az_bin_count_ || e < 0 || e >= el_bin_count_) return -1;
  return a * el_bin_count_ + e;
}

VisibilityModel::StaticCell VisibilityModel::staticAt(
    double x, double y, double z, const Angles & a) const {
  const auto key = staticKey(x, y, z);
  const auto it = static_cache_.find(key);
  if (it != static_cache_.end()) { ++static_cache_hits_; return it->second; }
  ++static_cache_misses_;
  StaticCell c;
  const double edge_az = config_.azimuth_bin_deg;
  const double edge_el = config_.elevation_bin_deg;
  c.h = a.az >= config_.azimuth_min_deg + edge_az &&
        a.az <= config_.azimuth_max_deg - edge_az;
  c.v = a.el >= config_.elevation_min_deg + edge_el &&
        a.el <= config_.elevation_max_deg - edge_el;
  c.r = a.range >= config_.min_range_m && a.range <= config_.max_range_m;
  if (!c.h) c.reason =
      (a.az >= config_.azimuth_min_deg - edge_az &&
       a.az <= config_.azimuth_max_deg + edge_az) ?
      VisibilityReason::UNKNOWN : VisibilityReason::OUT_OF_HORIZONTAL_FOV;
  else if (a.range < config_.min_range_m) c.reason = VisibilityReason::TOO_CLOSE;
  else if (a.range > config_.max_range_m) c.reason = VisibilityReason::TOO_FAR;
  else if (!c.v) {
    c.reason =
        (a.el >= config_.elevation_min_deg - edge_el &&
         a.el <= config_.elevation_max_deg + edge_el) ?
        VisibilityReason::UNKNOWN : VisibilityReason::OUT_OF_VERTICAL_FOV;
    // Project the steepest allowed world ray to the target height. This is a
    // conservative under-sensor test; it never uses a ground-only XY mask.
    if (z < world_from_sensor_.getOrigin().z()) {
      if (steepest_downward_slope_ > 0.0) {
        const double min_horizontal = (world_from_sensor_.getOrigin().z() - z) /
            steepest_downward_slope_;
        const double horizontal = std::hypot(x - world_from_sensor_.getOrigin().x(),
                                              y - world_from_sensor_.getOrigin().y());
        if (c.reason == VisibilityReason::OUT_OF_VERTICAL_FOV && horizontal < min_horizontal) {
          c.near = true;
          c.reason = VisibilityReason::NEAR_BLIND;
        }
      }
    }
  } else c.reason = VisibilityReason::OBSERVABLE;
  return c;
}

void VisibilityModel::buildDepth(const std::vector<PointXYZIRT> & points,
                                 const std::vector<int> & owner_by_point) {
  std::fill(depth_.begin(), depth_.end(), DepthCell{});
  occupied_depth_cells_ = 0;
  occupied_depth_indices_.clear();
  if (!pose_valid_) return;
  for (size_t i = 0; i < points.size(); ++i) {
    const auto a = angles(points[i].x, points[i].y, points[i].z);
    if (!std::isfinite(a.range) || a.range < config_.min_range_m ||
        a.range > config_.max_range_m ||
        a.az < config_.azimuth_min_deg || a.az > config_.azimuth_max_deg ||
        a.el < config_.elevation_min_deg || a.el > config_.elevation_max_deg) continue;
    const int index = depthIndex(static_cast<int>(std::floor(a.az / config_.azimuth_bin_deg)),
                                 static_cast<int>(std::floor(a.el / config_.elevation_bin_deg)));
    if (index < 0) continue;
    auto & c = depth_[index];
    if (c.count == 0) {
      ++occupied_depth_cells_;
      occupied_depth_indices_.push_back(index);
    }
    ++c.count;
    c.farthest = std::max(c.farthest, a.range);
    if (a.range < c.nearest) {
      c.nearest = a.range;
      c.nearest_owner = i < owner_by_point.size() ? owner_by_point[i] : -1;
      c.nearest_x = points[i].x;
      c.nearest_y = points[i].y;
      c.nearest_z = points[i].z;
    }
  }
}


void VisibilityModel::buildGeneralDepth(const std::vector<PointXYZIRT> & points,
                                        const std::vector<int> & owner_by_point) {
  std::fill(depth_.begin(), depth_.end(), DepthCell{});
  occupied_depth_cells_ = 0;
  occupied_depth_indices_.clear();
  if (!pose_valid_) return;
  struct DepthReturn { double range; size_t point; int index; };
  // Scratch capacity is thread-local; clear every invocation before appending
  // only this frame's projections. No point pointers or tracking state persist.
  static thread_local std::vector<DepthReturn> projected;
  projected.clear();
  projected.reserve(points.size());
  // Compute the unchanged projection once per input point, retaining input order.
  for (size_t i = 0; i < points.size(); ++i) {
    const auto a = angles(points[i].x, points[i].y, points[i].z);
    if (!std::isfinite(a.range) || a.range < config_.min_range_m ||
        a.range > config_.max_range_m ||
        a.az < config_.azimuth_min_deg || a.az > config_.azimuth_max_deg ||
        a.el < config_.elevation_min_deg || a.el > config_.elevation_max_deg) continue;
    const int index = depthIndex(static_cast<int>(std::floor(a.az / config_.azimuth_bin_deg)),
                                 static_cast<int>(std::floor(a.el / config_.elevation_bin_deg)));
    if (index < 0) continue;
    projected.push_back({a.range, i, index});
  }
  for (size_t j = 0; j < projected.size(); ++j) {
    if (j + 16 < projected.size()) __builtin_prefetch(&depth_[projected[j + 16].index], 1, 3);
    const auto & value = projected[j];
    const size_t i = value.point;
    const int index = value.index;
    auto & c = depth_[index];
    if (c.count == 0) {
      ++occupied_depth_cells_;
      occupied_depth_indices_.push_back(index);
    }
    ++c.count;
    c.farthest = std::max(c.farthest, value.range);
    if (value.range < c.nearest) {
      c.nearest = value.range;
      c.nearest_owner = i < owner_by_point.size() ? owner_by_point[i] : -1;
      c.nearest_x = points[i].x;
      c.nearest_y = points[i].y;
      c.nearest_z = points[i].z;
    }
  }
}

void VisibilityModel::remapDepthOwners(
    const std::unordered_map<int, int> & cluster_to_track_owner) {
  if (cluster_to_track_owner.empty()) return;
  for (int index : occupied_depth_indices_) {
    auto & cell = depth_[index];
    const auto it = cluster_to_track_owner.find(cell.nearest_owner);
    if (it != cluster_to_track_owner.end()) cell.nearest_owner = it->second;
  }
}

VisibilityInfo VisibilityModel::evaluateStaticPoint(double x, double y, double z) const {
  VisibilityInfo out;
  if (!pose_valid_) return out;
  const auto a = angles(x, y, z);
  out.sensor_distance = a.range;
  out.azimuth_deg = a.az;
  out.elevation_deg = a.el;
  const auto s = staticAt(x, y, z, a);
  out.in_horizontal_fov = s.h;
  out.in_vertical_fov = s.v;
  out.in_valid_range = s.r;
  out.near_blind_zone = s.near;
  out.reason = s.reason;
  out.confidence = s.reason == VisibilityReason::OBSERVABLE ?
      (config_.calibration_verified ? 0.8 : 0.4) : 0.8;
  return out;
}

VisibilityInfo VisibilityModel::evaluatePoint(double x, double y, double z, int owner) const {
  VisibilityInfo out = evaluateStaticPoint(x, y, z);
  if (!pose_valid_) return out;
  const auto a = angles(x, y, z);
  const auto s = staticAt(x, y, z, a);
  if (s.reason != VisibilityReason::OBSERVABLE) return out;

  const int az_bin = static_cast<int>(std::floor(a.az / config_.azimuth_bin_deg));
  const int el_bin = static_cast<int>(std::floor(a.el / config_.elevation_bin_deg));
  // Angular footprint plus range noise; nearby bins are only used to find a
  // nearer return. Free-space evidence requires a measured farther return in
  // the central bin and is never inferred from a no-return ray.
  const double half_angle = rad(std::hypot(config_.azimuth_bin_deg,
                                           config_.elevation_bin_deg) * 0.5);
  const double margin = config_.range_noise_m + a.range * std::tan(half_angle);
  double nearest = std::numeric_limits<double>::infinity();
  int nearest_owner = -1;
  const DepthCell * nearest_cell = nullptr;
  for (int da = -1; da <= 1; ++da) {
    for (int de = -1; de <= 1; ++de) {
      const int index = depthIndex(az_bin + da, el_bin + de);
      if (index >= 0 && depth_[index].count > 0 && depth_[index].nearest < nearest) {
        nearest = depth_[index].nearest;
        nearest_owner = depth_[index].nearest_owner;
        nearest_cell = &depth_[index];
      }
    }
  }
  if (nearest + margin < a.range) {
    out.has_occluder = nearest_cell != nullptr;
    if (nearest_cell) {
      out.occluder_x = nearest_cell->nearest_x;
      out.occluder_y = nearest_cell->nearest_y;
      out.occluder_z = nearest_cell->nearest_z;
    }
    out.reason = (owner >= 0 && nearest_owner == owner) ?
        VisibilityReason::SELF_OCCLUSION_POSSIBLE :
        (nearest_owner >= 0 ? VisibilityReason::OCCLUDED_BY_OTHER :
                              VisibilityReason::NEARER_RETURN_UNKNOWN);
    out.confidence = nearest_owner >= 0 ? 0.7 : 0.5;
    return out;
  }
  const int center_index = depthIndex(az_bin, el_bin);
  if (center_index < 0 || depth_[center_index].count == 0) {
    out.reason = VisibilityReason::UNKNOWN;
    out.confidence = 0.0;
  } else if (owner >= 0 && depth_[center_index].nearest_owner == owner &&
             std::abs(depth_[center_index].nearest - a.range) <= margin) {
    out.reason = VisibilityReason::OBSERVABLE;
    out.confidence = 0.7;
  } else if (depth_[center_index].nearest > a.range + margin) {
    out.reason = VisibilityReason::VISIBLE_EMPTY;
    out.confidence = 0.6;
  } else {
    out.reason = VisibilityReason::UNKNOWN;
    out.confidence = 0.2;
  }
  return out;
}

VisibilityInfo VisibilityModel::evaluateBox(
    const VisibilityBox & box, int owner, bool confirmed_geometry) const {
  VisibilityInfo out;
  const double cx = (box.min_x + box.max_x) * 0.5;
  const double cy = (box.min_y + box.max_y) * 0.5;
  const double cz = (box.min_z + box.max_z) * 0.5;
  const auto center = evaluatePoint(cx, cy, cz, owner);
  out.sensor_distance = center.sensor_distance;
  out.azimuth_deg = center.azimuth_deg;
  out.elevation_deg = center.elevation_deg;
  out.in_horizontal_fov = center.in_horizontal_fov;
  out.in_vertical_fov = center.in_vertical_fov;
  out.in_valid_range = center.in_valid_range;
  out.near_blind_zone = center.near_blind_zone;
  if (!pose_valid_) return out;

  int other = 0, own = 0, nearer_unknown = 0;
  int horizontal_out = 0, vertical_out = 0, near = 0, far = 0;
  double confidence_sum = 0.0;
  // Six faces, 3x3 samples per face. These are visibility probes, not an
  // inferred full-vehicle OBB or a claim that a vehicle exists.
  for (int face = 0; face < 6; ++face) {
    for (int u = 0; u < 3; ++u) for (int v = 0; v < 3; ++v) {
      const double fx = box.min_x + (box.max_x - box.min_x) * (u + 0.5) / 3.0;
      const double fy = box.min_y + (box.max_y - box.min_y) * (u + 0.5) / 3.0;
      const double fz = box.min_z + (box.max_z - box.min_z) * (v + 0.5) / 3.0;
      const double gy = box.min_y + (box.max_y - box.min_y) * (v + 0.5) / 3.0;
      const double x = face == 0 ? box.min_x : face == 1 ? box.max_x : fx;
      const double y = face == 2 ? box.min_y : face == 3 ? box.max_y :
                       face < 2 ? fy : gy;
      const double z = face == 4 ? box.min_z : face == 5 ? box.max_z : fz;
      const auto p = evaluatePoint(x, y, z, owner);
      ++out.expected_samples;
      confidence_sum += p.confidence;
      if (p.reason == VisibilityReason::OUT_OF_HORIZONTAL_FOV) { ++horizontal_out; continue; }
      if (p.reason == VisibilityReason::OUT_OF_VERTICAL_FOV) { ++vertical_out; continue; }
      if (p.reason == VisibilityReason::NEAR_BLIND || p.reason == VisibilityReason::TOO_CLOSE) {
        ++near; continue;
      }
      if (p.reason == VisibilityReason::TOO_FAR) { ++far; continue; }
      ++out.observable_samples;
      if (p.reason == VisibilityReason::OCCLUDED_BY_OTHER) { ++other; ++out.occluded_samples; }
      else if (p.reason == VisibilityReason::NEARER_RETURN_UNKNOWN) {
        ++nearer_unknown; ++out.occluded_samples;
      } else if (p.reason == VisibilityReason::SELF_OCCLUSION_POSSIBLE) ++own;
      else if (p.reason == VisibilityReason::VISIBLE_EMPTY) ++out.free_samples;
      else if (p.reason == VisibilityReason::OBSERVABLE) ++out.supported_samples;
      else ++out.unknown_samples;
      if (p.has_occluder && !out.has_occluder &&
          p.reason == VisibilityReason::OCCLUDED_BY_OTHER) {
        out.has_occluder = true;
        out.occluder_x = p.occluder_x;
        out.occluder_y = p.occluder_y;
        out.occluder_z = p.occluder_z;
      }
    }
  }
  out.observable_fraction = static_cast<double>(out.observable_samples) / out.expected_samples;
  if (out.observable_samples) {
    out.occluded_fraction = static_cast<double>(out.occluded_samples) / out.observable_samples;
    out.supported_fraction = static_cast<double>(out.supported_samples) / out.observable_samples;
  }
  out.confidence = confidence_sum / out.expected_samples;
  if (!out.observable_samples) {
    if (near) out.reason = VisibilityReason::NEAR_BLIND;
    else if (horizontal_out) out.reason = VisibilityReason::OUT_OF_HORIZONTAL_FOV;
    else if (vertical_out) out.reason = VisibilityReason::OUT_OF_VERTICAL_FOV;
    else if (far) out.reason = VisibilityReason::TOO_FAR;
    else out.reason = VisibilityReason::UNKNOWN;
  } else if (confirmed_geometry && other >= 1 &&
             static_cast<double>(other) / out.observable_samples >= 0.9) {
    out.reason = VisibilityReason::FULLY_OCCLUDED_LIKELY;
  } else if (other > 0) out.reason = VisibilityReason::OCCLUDED_BY_OTHER;
  else if (own > 0) out.reason = VisibilityReason::SELF_OCCLUSION_POSSIBLE;
  else if (nearer_unknown > 0) out.reason = VisibilityReason::NEARER_RETURN_UNKNOWN;
  else if (out.supported_samples > 0) out.reason = VisibilityReason::OBSERVABLE;
  else if (out.free_samples == out.expected_samples && out.expected_samples > 0)
    out.reason = VisibilityReason::VISIBLE_EMPTY;
  else out.reason = VisibilityReason::UNKNOWN;
  return out;
}

void VisibilityModel::setMeasuredRingProfile(
    const std::vector<std::pair<double,double>> & bands) {
  // The existing CX128 profile enumerates all 128 channels. Partial profiles
  // cannot make absent channels disappear from the expectation.
  if (bands.size()!=128 || !std::all_of(bands.begin(),bands.end(),[](const auto & b){
      return std::isfinite(b.first)&&std::isfinite(b.second)&&b.first<=b.second;
    })) { measured_ring_elevation_bands_.clear(); return; }
  measured_ring_elevation_bands_=bands;
}
VisibilityInfo VisibilityModel::evaluateAdaptiveBox(
    const VisibilityBox & box, int owner, bool confirmed_geometry,
    double position_sigma_m) const {
  return evaluateAdaptiveBox(box,owner,confirmed_geometry,position_sigma_m,false);
}
VisibilityInfo VisibilityModel::evaluateAdaptiveBox(
    const VisibilityBox & box, int owner, bool confirmed_geometry,
    double position_sigma_m, bool measured_ring_profile) const {
  VisibilityInfo out = evaluateBox(box, owner, confirmed_geometry);
  if (!pose_valid_) return out;
  double az_min = 1e30, az_max = -1e30, el_min = 1e30, el_max = -1e30;
  double range_min = 1e30, range_max = 0.0;
  for (double x : {box.min_x, box.max_x})
    for (double y : {box.min_y, box.max_y})
      for (double z : {box.min_z, box.max_z}) {
        const auto a = angles(x, y, z);
        az_min = std::min(az_min, a.az); az_max = std::max(az_max, a.az);
        el_min = std::min(el_min, a.el); el_max = std::max(el_max, a.el);
        range_min = std::min(range_min, a.range);
        range_max = std::max(range_max, a.range);
      }
  // The configured sector is not wrapped. A box crossing 0 degrees uses the
  // clipped sector; it must not imply an almost full-circle vehicle footprint.
  if (az_max - az_min > 180.0) {
    az_min = config_.azimuth_min_deg;
    az_max = config_.azimuth_max_deg;
  }
  const double core_az_min = std::max(config_.azimuth_min_deg, az_min);
  const double core_az_max = std::min(config_.azimuth_max_deg, az_max);
  const double core_el_min = std::max(config_.elevation_min_deg, el_min);
  const double core_el_max = std::min(config_.elevation_max_deg, el_max);
  if (core_az_min < core_az_max && core_el_min < core_el_max) {
    const int core_na = std::max(1, static_cast<int>(std::ceil(
        (core_az_max - core_az_min) / config_.azimuth_bin_deg)));
    int core_ne = std::max(1, static_cast<int>(std::ceil(
        (core_el_max - core_el_min) / config_.ray_elevation_spacing_deg)));
    if (measured_ring_profile && measured_ring_elevation_bands_.size()==128) {
      // Count a channel whenever its measured range overlaps the footprint.
      // Min/max bands are conservative with respect to its median angle.
      core_ne=static_cast<int>(std::count_if(measured_ring_elevation_bands_.begin(),
          measured_ring_elevation_bands_.end(),[&](const auto & band){
            return band.second>=core_el_min && band.first<=core_el_max;
          }));
    }
    out.adaptive_expected_rays = core_na * core_ne;
  }
  const double distance = std::max(range_min, config_.min_range_m);
  const double sigma = std::max(0.0, position_sigma_m);
  const double margin_deg = std::min(10.0, deg(std::atan2(2.0 * sigma, distance))) +
                            config_.azimuth_bin_deg;
  az_min = std::max(config_.azimuth_min_deg, az_min - margin_deg);
  az_max = std::min(config_.azimuth_max_deg, az_max + margin_deg);
  el_min = std::max(config_.elevation_min_deg, el_min - margin_deg - config_.elevation_bin_deg);
  el_max = std::min(config_.elevation_max_deg, el_max + margin_deg + config_.elevation_bin_deg);
  out.adaptive_azimuth_width_deg = std::max(0.0, az_max - az_min);
  out.adaptive_elevation_width_deg = std::max(0.0, el_max - el_min);
  out.horizontal_point_spacing_m = 2.0 * out.sensor_distance *
      std::tan(rad(config_.azimuth_bin_deg) * 0.5);
  out.vertical_point_spacing_m = 2.0 * out.sensor_distance *
      std::tan(rad(config_.ray_elevation_spacing_deg) * 0.5);
  if (az_min >= az_max || el_min >= el_max) {
    out.memory_weight = 1.0;
    return out;
  }
  const int a0 = static_cast<int>(std::floor(az_min / config_.azimuth_bin_deg));
  const int a1 = static_cast<int>(std::ceil(az_max / config_.azimuth_bin_deg));
  const int e0 = static_cast<int>(std::floor(el_min / config_.elevation_bin_deg));
  const int e1 = static_cast<int>(std::ceil(el_max / config_.elevation_bin_deg));
  const int na = std::max(1, a1 - a0), ne = std::max(1, e1 - e0);
  // Query a bounded local window. The bin count remains the expected ray
  // support; sampled depth occupancy estimates the observed fraction.
  const int stride = std::max(1, static_cast<int>(std::ceil(
      std::sqrt(static_cast<double>(na * ne) / 1024.0))));
  int checked = 0, supported = 0;
  const double range_margin = config_.range_noise_m +
      distance * std::tan(rad(std::hypot(config_.azimuth_bin_deg,
                                           config_.elevation_bin_deg)) * 0.5);
  for (int a = a0; a < a1; a += stride) {
    for (int e = e0; e < e1; e += stride) {
      const int index = depthIndex(a, e);
      if (index < 0) continue;
      ++checked;
      const auto & cell = depth_[index];
      if (cell.count && cell.nearest >= range_min - range_margin &&
          cell.nearest <= range_max + range_margin &&
          (owner < 0 || cell.nearest_owner == owner || cell.nearest_owner < 0)) ++supported;
    }
  }
  out.adaptive_supported_rays = checked > 0 ? static_cast<int>(std::lround(
      static_cast<double>(out.adaptive_expected_rays) * supported / checked)) : 0;
  // More expected rays make the present geometry more informative. Occlusion
  // and blind geometry reduce that trust even at short distance.
  const double rays_per_azimuth = std::max(1.0,
      (config_.elevation_max_deg - config_.elevation_min_deg) /
      config_.ray_elevation_spacing_deg);
  const double density_weight = static_cast<double>(out.adaptive_expected_rays) /
      (out.adaptive_expected_rays + rays_per_azimuth);
  const double support_fraction = out.adaptive_expected_rays > 0 ?
      static_cast<double>(out.adaptive_supported_rays) / out.adaptive_expected_rays : 0.0;
  out.current_observation_weight = density_weight * (0.25 + 0.75 * support_fraction) *
      (1.0 - out.occluded_fraction);
  if (out.near_blind_zone || out.reason == VisibilityReason::NEAR_BLIND ||
      out.reason == VisibilityReason::OUT_OF_VERTICAL_FOV ||
      out.reason == VisibilityReason::OUT_OF_HORIZONTAL_FOV) {
    out.current_observation_weight = 0.0;
  }
  out.memory_weight = 1.0 - out.current_observation_weight;
  return out;
}

}  // namespace cx128s2_bev_perception
