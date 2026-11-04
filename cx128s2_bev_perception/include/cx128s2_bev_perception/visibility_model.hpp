#pragma once

struct TrackingRegressionAccess;

#include <cx128s2_bev_perception/point_types.hpp>
#include <tf2/LinearMath/Transform.h>

#include <cstdint>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>
#include <utility>

namespace cx128s2_bev_perception {

enum class VisibilityReason {
  UNKNOWN, OBSERVABLE, OUT_OF_HORIZONTAL_FOV, OUT_OF_VERTICAL_FOV,
  TOO_CLOSE, TOO_FAR, NEAR_BLIND, OCCLUDED_BY_OTHER,
  NEARER_RETURN_UNKNOWN, SELF_OCCLUSION_POSSIBLE, VISIBLE_EMPTY,
  FULLY_OCCLUDED_LIKELY
};

const char * visibilityReasonName(VisibilityReason reason);

struct VisibilityInfo {
  VisibilityReason reason = VisibilityReason::UNKNOWN;
  bool in_horizontal_fov = false;
  bool in_vertical_fov = false;
  bool in_valid_range = false;
  bool near_blind_zone = false;
  int expected_samples = 0;
  int observable_samples = 0;
  int occluded_samples = 0;
  int supported_samples = 0;
  int free_samples = 0;
  int unknown_samples = 0;
  double observable_fraction = 0.0;
  double occluded_fraction = 0.0;
  double supported_fraction = 0.0;
  double sensor_distance = 0.0;
  double azimuth_deg = 0.0;
  double elevation_deg = 0.0;
  double confidence = 0.0;  // Confidence in the visibility explanation, not vehicle class.
  int adaptive_expected_rays = 0;
  int adaptive_supported_rays = 0;
  double adaptive_azimuth_width_deg = 0.0;
  double adaptive_elevation_width_deg = 0.0;
  double horizontal_point_spacing_m = 0.0;
  double vertical_point_spacing_m = 0.0;
  double current_observation_weight = 0.0;
  double memory_weight = 0.0;
  bool has_occluder = false;
  double occluder_x = 0.0, occluder_y = 0.0, occluder_z = 0.0;
};

struct VisibilityBox {
  double min_x = 0.0, max_x = 0.0;
  double min_y = 0.0, max_y = 0.0;
  double min_z = 0.0, max_z = 0.0;
};

struct VisibilityConfig {
  std::string sensor_frame = "laser_link";
  double azimuth_min_deg = 30.0, azimuth_max_deg = 150.0;
  double elevation_min_deg = -12.5, elevation_max_deg = 12.5;
  double min_range_m = 0.15, max_range_m = 200.0;
  double azimuth_bin_deg = 0.1, elevation_bin_deg = 0.125;
  double ray_elevation_spacing_deg = 0.0;  // Zero uses the depth-grid bin.
  double range_noise_m = 0.03;
  double static_xy_resolution_m = 0.1;
  double ground_z = 0.0;
  std::vector<double> sample_heights_m{0.0, 0.6, 2.25, 4.5};
  bool calibration_verified = false;
};

class VisibilityModel {
public:
  void configure(const VisibilityConfig & config);
  bool setPose(const tf2::Transform & world_from_sensor);
  void clearStaticCache();
  void setRayElevationSpacing(double spacing_deg);
  // Explicitly populate only fixed installation/ROI samples. Moving queries
  // never insert into this cache.
  VisibilityInfo cacheStaticPoint(double x, double y, double z);
  // points are the full, transformed cloud before crop/ROI/background filtering.
  void buildDepth(const std::vector<PointXYZIRT> & points,
                  const std::vector<int> & owner_by_point);
  // General-only ordered projection path; existing callers retain buildDepth.
  void buildGeneralDepth(const std::vector<PointXYZIRT> & points,
                         const std::vector<int> & owner_by_point);
  // Remap already-built nearest cluster owners after track association; no depth rebuild.
  void remapDepthOwners(const std::unordered_map<int, int> & cluster_to_track_owner);
  VisibilityInfo evaluateStaticPoint(double x, double y, double z) const;
  VisibilityInfo evaluatePoint(double x, double y, double z, int owner = -1) const;
  VisibilityInfo evaluateBox(const VisibilityBox & box, int owner = -1,
                             bool confirmed_geometry = false) const;
  VisibilityInfo evaluateAdaptiveBox(const VisibilityBox & box, int owner,
                                     bool confirmed_geometry,
                                     double position_sigma_m) const;
  // Complete measured channel bands; no calibration offsets are invented.
  void setMeasuredRingProfile(const std::vector<std::pair<double,double>> & bands);
  VisibilityInfo evaluateAdaptiveBox(const VisibilityBox & box, int owner,
      bool confirmed_geometry, double position_sigma_m, bool measured_ring_profile) const;
  size_t depthCellCount() const { return occupied_depth_cells_; }
  size_t staticCellCount() const { return static_cache_.size(); }
  size_t staticCacheHits() const { return static_cache_hits_; }
  size_t staticCacheMisses() const { return static_cache_misses_; }
  double azimuthBinDeg() const { return config_.azimuth_bin_deg; }
  double elevationBinDeg() const { return config_.elevation_bin_deg; }
  double rayElevationSpacingDeg() const { return config_.ray_elevation_spacing_deg; }
  const VisibilityConfig & config() const { return config_; }
  const tf2::Transform & pose() const { return world_from_sensor_; }

private:
  friend struct ::TrackingRegressionAccess;
  struct Angles { double az = 0.0, el = 0.0, range = 0.0; };
  struct DepthCell {
    double nearest = 1e30, farthest = 0.0;
    int nearest_owner = -1;
    double nearest_x = 0.0, nearest_y = 0.0, nearest_z = 0.0;
    int count = 0;
  };
  struct StaticCell { VisibilityReason reason = VisibilityReason::UNKNOWN; bool h = false, v = false, r = false, near = false; };
  struct CellKey {
    int x = 0, y = 0, z = 0;
    bool operator==(const CellKey & other) const {
      return x == other.x && y == other.y && z == other.z;
    }
  };
  struct CellKeyHash {
    size_t operator()(const CellKey & key) const {
      size_t h = std::hash<int>{}(key.x);
      h ^= std::hash<int>{}(key.y) + 0x9e3779b9 + (h << 6) + (h >> 2);
      h ^= std::hash<int>{}(key.z) + 0x9e3779b9 + (h << 6) + (h >> 2);
      return h;
    }
  };
  Angles angles(double x, double y, double z) const;
  StaticCell staticAt(double x, double y, double z, const Angles & a) const;
  CellKey staticKey(double x, double y, double z) const;
  int depthIndex(int az_bin, int el_bin) const;
  VisibilityConfig config_;
  tf2::Transform world_from_sensor_{tf2::Transform::getIdentity()};
  tf2::Transform sensor_from_world_{tf2::Transform::getIdentity()};
  bool pose_valid_ = false;
  double steepest_downward_slope_ = 0.0;
  mutable std::unordered_map<CellKey, StaticCell, CellKeyHash> static_cache_;
  mutable size_t static_cache_hits_ = 0, static_cache_misses_ = 0;
  std::vector<DepthCell> depth_;
  std::vector<int> occupied_depth_indices_;
  int az_bin_min_ = 0, el_bin_min_ = 0, az_bin_count_ = 0, el_bin_count_ = 0;
  size_t occupied_depth_cells_ = 0;
  std::vector<std::pair<double,double>> measured_ring_elevation_bands_;
};

}  // namespace cx128s2_bev_perception
