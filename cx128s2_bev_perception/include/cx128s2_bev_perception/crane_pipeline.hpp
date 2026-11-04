#pragma once

#include <cstddef>
#include <cmath>
#include <algorithm>
#include <functional>
#include <cstdint>
#include <bitset>
#include <limits>
#include <array>
#include <string>
#include <deque>
#include <vector>
#include <unordered_map>
#include <cx128s2_bev_perception/visibility_model.hpp>
#include <tf2/LinearMath/Transform.h>
#include <cx128s2_bev_perception/crane_beam_detector.hpp>

namespace cx128s2_bev_perception {

// Height patches and centered least-squares z=ax+by+c. Independent from Slope.
struct CraneHeightConfig {
  bool enabled=false;
  double patch_resolution=0.10, max_neighbor_dz=0.05, max_patch_z_spread=0.08;
  double max_xy_gap=0.15;
  int min_patch_points=3, min_surface_points=20;
  double max_plane_rms=0.02, max_plane_tilt_deg=20.0;
  double max_merge_angle_deg=10.0, max_merge_gap_z=0.05;
  double min_surface_length=0.20, min_surface_width=0.02;
};
struct CraneHeightSurface {
  int id=-1;
  size_t index_begin=0,point_count=0,patch_count=0;
  double center_x=0,center_y=0,center_z=0,min_z=0,max_z=0;
  double plane_a=0,plane_b=0,plane_c=0,plane_rms=0,max_residual_bound=0;
  std::array<double,3> plane_normal{{0,0,1}};
  double length=0,width=0;
};
struct CraneHeightResult {
  std::vector<int> surface_ids,patch_ids; // original full-cloud index -> ID
  std::vector<int> candidates,surface_points,unassigned,excluded;
  std::vector<CraneHeightSurface> surfaces;
  size_t patch_count=0,valid_patch_count=0,spatial_cells=0;
  size_t neighbor_tests=0,merge_attempts=0,merged_edges=0,occlusion_flagged_points=0;
  double visibility_ms=0,patch_ms=0,fitting_ms=0,merging_ms=0,finalization_ms=0,total_ms=0;
};

enum class CraneSlopeLabel : uint8_t { UNKNOWN, GROUND, NON_GROUND };

enum class CraneBoundaryType : uint8_t {
  NONE = 0,
  HEIGHT_STEP = 1 << 0,
  SLOPE_DISCONTINUITY = 1 << 1,
  RANGE_JUMP = 1 << 2,
  GROUND_NEIGHBOR = 1 << 3
};

inline CraneBoundaryType operator|(CraneBoundaryType a, CraneBoundaryType b) {
  return static_cast<CraneBoundaryType>(static_cast<uint8_t>(a) | static_cast<uint8_t>(b));
}
inline CraneBoundaryType operator&(CraneBoundaryType a, CraneBoundaryType b) {
  return static_cast<CraneBoundaryType>(static_cast<uint8_t>(a) & static_cast<uint8_t>(b));
}
inline bool hasBoundaryFlag(CraneBoundaryType val, CraneBoundaryType flag) {
  return (static_cast<uint8_t>(val) & static_cast<uint8_t>(flag)) != 0;
}

struct CraneBoundaryClusterConfig {
  bool enabled = false;
  double slope_threshold_deg = 35.0;
  double height_step_threshold = 0.12;
  double range_jump_threshold = 0.35;
  double max_neighbor_distance = 0.40;
  double min_height_above_ground = 0.05;
  int min_cluster_points = 10;
  int max_cluster_points = 10000;
  bool include_unknown = true;
  bool enable_morphological_closing = false;
  double morph_closing_radius_m = 0.15;
};

struct CraneCluster {
  int id = 0;
  std::vector<int> point_indices;
  double min_x = 0, max_x = 0;
  double min_y = 0, max_y = 0;
  double min_z = 0, max_z = 0;
  double centroid_x = 0, centroid_y = 0, centroid_z = 0;
  double length = 0, width = 0, height = 0;
};

struct CraneBoundaryResult {
  std::vector<CraneBoundaryType> boundary_types;
  std::vector<int> boundary_indices;
  std::vector<int> height_step_indices;
  std::vector<int> slope_indices;
  std::vector<int> range_jump_indices;
  std::vector<CraneCluster> clusters;
  std::vector<int> cluster_labels;
  double boundary_detection_ms = 0.0;
  double region_growing_ms = 0.0;
  double total_ms = 0.0;
  size_t severed_edges = 0;
  size_t total_tested_edges = 0;
};

struct CraneZGaussianConfig {
  bool enabled = false;
  double sigma_z = 0.025;
  double range_sigma_factor = 0.001;
  double confidence_sigma_multiplier = 3.0;
  double step_sigma_multiplier = 2.5;
  double max_layer_z_spread = 0.25;
  double min_height_above_ground = 0.05;
  int min_layer_points = 5;
};

struct CraneZGaussianLayer {
  int id = 0;
  std::vector<int> point_indices;
  double mean_z = 0.0;
  double std_z = 0.0;
  double min_z = 0.0;
  double max_z = 0.0;
};

struct CraneZGaussianResult {
  std::vector<CraneZGaussianLayer> layers;
  std::vector<int> point_layer_ids;
  double grouping_ms = 0.0;
  size_t candidate_points = 0;
  size_t grouped_points = 0;
};

struct CraneClosedLoopConfig {
  bool enabled = false;
  double grid_resolution_m = 0.08;
  double closing_radius_m = 0.16;
  double min_containment_ratio = 0.50;
  int min_enclosed_cells = 4;
  bool use_boundary_points_for_loop = false;
  double subcluster_match_xy_gate = 0.60;
  double subcluster_match_z_gate = 0.40;
};

struct CraneSubcluster {
  int subcluster_id = -1;
  int group_id = -1;
  int layer_id = -1;
  std::vector<int> point_indices;
  std::vector<int> boundary_point_indices;
  double centroid_x = 0.0, centroid_y = 0.0, centroid_z = 0.0;
  double min_x = 0.0, max_x = 0.0;
  double min_y = 0.0, max_y = 0.0;
  double min_z = 0.0, max_z = 0.0;
  // Frame-to-frame Z motion tracking (dz_relative_i = dz_i - dz_background)
  double prev_z = 0.0;
  double dz = 0.0;
  double dz_relative = 0.0;
  double cumulative_dz_relative = 0.0;
  int tracked_frames = 0;
  bool motion_valid = false;
  bool current_raw_indices=false; // Explicit provenance; never interpret accumulated IDs as current Raw.
};

struct CraneObjectGroup {
  int group_id = -1;
  bool is_closed_loop = false;
  int enclosed_cavity_cells = 0;
  std::vector<int> subcluster_ids;
  std::vector<CraneSubcluster> subclusters;
  std::vector<int> merged_point_indices;
  double centroid_x = 0.0, centroid_y = 0.0, centroid_z = 0.0;
  double min_x = 0.0, max_x = 0.0;
  double min_y = 0.0, max_y = 0.0;
  double min_z = 0.0, max_z = 0.0;
};

struct CraneClosedLoopResult {
  std::vector<CraneObjectGroup> groups;
  std::vector<CraneSubcluster> subclusters;
  std::vector<int> point_group_ids;
  std::vector<int> point_subcluster_ids;
  double background_z = 0.0;
  double dz_background = 0.0;
  size_t closed_loop_regions = 0;
  size_t detected_closed_loops = 0;
  double grouping_ms = 0.0;
};

struct CraneRoiFilterConfig {
  bool enabled = false;
  bool only_crane_roi = true; // when true, compute ONLY inside Crane ROI regardless of FSM state
  CraneBeamState current_state = CraneBeamState::IDLE;
  bool idle_exclude_beam_roi = false; // default false: Background ROI encompasses entire [-8, 8] region in IDLE
  // Crane ROI (Beam ROI) in target_frame / world
  double beam_roi_x_min = -2.0;
  double beam_roi_x_max = 2.0;
  double beam_roi_y_min = -2.0;
  double beam_roi_y_max = 2.0;
  double beam_roi_z_min = -0.5;
  double beam_roi_z_max = 7.5;
  // Background ROI in target_frame / world
  double bg_roi_x_min = -8.0;
  double bg_roi_x_max = 8.0;
  double bg_roi_y_min = -8.0;
  double bg_roi_y_max = 8.0;
  double bg_roi_z_min = -0.5;
  double bg_roi_z_max = 7.5;
  bool active_include_background = false;
  bool expand_beam_roi = false; // when enabled, allow spatial expansion up to bg_roi bounds while keeping beam_roi for seed search

  bool insideBeamRoi(double x, double y, double z) const {
    return x >= beam_roi_x_min && x <= beam_roi_x_max &&
           y >= beam_roi_y_min && y <= beam_roi_y_max &&
           z >= beam_roi_z_min && z <= beam_roi_z_max;
  }
  bool insideBgRoi(double x, double y, double z) const {
    return x >= bg_roi_x_min && x <= bg_roi_x_max &&
           y >= bg_roi_y_min && y <= bg_roi_y_max &&
           z >= bg_roi_z_min && z <= bg_roi_z_max;
  }
  bool inActiveRoi(double x, double y, double z) const {
    if (!enabled) return true;
    if (only_crane_roi) {
      if (expand_beam_roi) {
        return insideBgRoi(x, y, z);
      }
      return insideBeamRoi(x, y, z);
    }
    if (current_state == CraneBeamState::IDLE) {
      if (idle_exclude_beam_roi) {
        return insideBgRoi(x, y, z) && !insideBeamRoi(x, y, z);
      }
      return insideBgRoi(x, y, z);
    }
    if (active_include_background || expand_beam_roi) {
      return insideBgRoi(x, y, z);
    }
    return insideBeamRoi(x, y, z);
  }
};

enum class CraneHookConnectionState : uint8_t {
  NONE = 0,
  CANDIDATE = 1,
  PERSISTENT = 2,
  MOTION_CORRELATED = 3
};

inline const char * to_string(CraneHookConnectionState s) {
  switch (s) {
    case CraneHookConnectionState::CANDIDATE: return "CANDIDATE";
    case CraneHookConnectionState::PERSISTENT: return "PERSISTENT";
    case CraneHookConnectionState::MOTION_CORRELATED: return "MOTION_CORRELATED";
    default: return "NONE";
  }
}

enum class CraneExceptionGateStatus : uint8_t {
  NONE = 0,
  PASS = 1,
  EXCLUDED = 2,
  UNKNOWN = 3
};

inline const char * to_string(CraneExceptionGateStatus s) {
  switch (s) {
    case CraneExceptionGateStatus::PASS: return "PASS";
    case CraneExceptionGateStatus::EXCLUDED: return "EXCLUDED";
    case CraneExceptionGateStatus::UNKNOWN: return "UNKNOWN";
    default: return "NONE";
  }
}

enum class CraneExceptionGateReason : uint8_t {
  NONE = 0,
  CRITERIA_MET = 1,
  LENGTH_MET = 2,
  AREA_MET = 3,
  LENGTH_AND_AREA = 4,
  BELOW_THRESHOLD = 5,
  OCCLUDED = 6,
  FOV_TRUNCATED = 7,
  TEMPORAL_HOLD = 8,
  NO_CHAIN = 9
};

inline const char * to_string(CraneExceptionGateReason r) {
  switch (r) {
    case CraneExceptionGateReason::CRITERIA_MET: return "CRITERIA_MET";
    case CraneExceptionGateReason::LENGTH_MET: return "LENGTH_MET";
    case CraneExceptionGateReason::AREA_MET: return "AREA_MET";
    case CraneExceptionGateReason::LENGTH_AND_AREA: return "LENGTH_AND_AREA";
    case CraneExceptionGateReason::BELOW_THRESHOLD: return "BELOW_THRESHOLD";
    case CraneExceptionGateReason::OCCLUDED: return "OCCLUDED";
    case CraneExceptionGateReason::FOV_TRUNCATED: return "FOV_TRUNCATED";
    case CraneExceptionGateReason::TEMPORAL_HOLD: return "TEMPORAL_HOLD";
    case CraneExceptionGateReason::NO_CHAIN: return "NO_CHAIN";
    default: return "NONE";
  }
}

enum class CraneVisibilityGateStatus : uint8_t {
  UNKNOWN = 0,
  OBSERVED = 1,
  PARTIAL = 2,
  OCCLUDED = 3,
  OUT_OF_FOV = 4,
  HELD = 5
};

inline const char * to_string(CraneVisibilityGateStatus v) {
  switch (v) {
    case CraneVisibilityGateStatus::OBSERVED: return "OBSERVED";
    case CraneVisibilityGateStatus::PARTIAL: return "PARTIAL";
    case CraneVisibilityGateStatus::OCCLUDED: return "OCCLUDED";
    case CraneVisibilityGateStatus::OUT_OF_FOV: return "OUT_OF_FOV";
    case CraneVisibilityGateStatus::HELD: return "HELD";
    default: return "UNKNOWN";
  }
}

struct CraneExceptionGateConfig {
  bool enabled = false;
  double min_length_m = 0.80;
  double min_area_m2 = 0.15;
  bool use_visibility = true;
  double max_occlusion_fraction = 0.40;
  bool scan_all_chain_nodes = true;
};

struct CraneExceptionGateResult {
  CraneExceptionGateStatus status = CraneExceptionGateStatus::NONE;
  CraneExceptionGateReason reason = CraneExceptionGateReason::NONE;
  CraneVisibilityGateStatus visibility_status = CraneVisibilityGateStatus::UNKNOWN;
  double max_connected_length = 0.0;
  double max_connected_area = 0.0;
  double largest_cluster_length = 0.0;
  double largest_cluster_area = 0.0;
  int evaluating_node_count = 0;
  double occlusion_fraction = 0.0;
  bool in_fov = true;
  bool is_held = false;
  double processing_ms = 0.0;
};

struct CraneTopDownHookConfig {
  bool enabled = true;
  double root_min_z = 4.8;             // minimum Z to qualify as Top Root (boom/trolley)
  double r0 = 0.25;                    // base XY horizontal radius [m]
  double max_sway_angle_deg = 15.0;    // max sway angle [deg]
  double max_gap_z = 2.5;              // max allowed vertical gap for tentative link [m]
  double direct_link_max_gap_z = 0.8;  // max gap for direct confident link [m]
  size_t min_subcluster_points = 10;   // min points for a valid subcluster node
  int temporal_window_n = 10;          // temporal window size N
  int persistent_confirm_m = 5;        // M frames needed for PERSISTENT
  int max_lost_frames = 5;             // track lost timeout
  double prepare_distance = 4.0;       // ground-relative readiness threshold [m]
  double roi_left_x_min = -1.40, roi_left_x_max = -0.60;
  double roi_right_x_min = 0.60, roi_right_x_max = 1.40;
  double roi_y_min = -0.50, roi_y_max = 0.50;
  double roi_z_min = -0.5;             // minimum height above ground for hook search (-0.5 enables full descent)
  size_t min_root_points = 15;         // min points for Root qualification to reject noise
  double root_measurement_std = 0.015; // Gaussian noise measurement std for Root Kalman filter [m]
  double root_move_start_x = 0.05;     // horizontal X root motion threshold [m]
  double root_move_start_y = 0.05;     // horizontal Y root motion threshold [m]
  double root_move_velocity_thresh = 0.03; // horizontal root velocity threshold [m/s]
  int root_move_stop_frames = 5;       // consecutive stable frames to stop crane_moving
  double max_lateral_drift_from_root = 0.50; // max lateral drift from plumb-line [m]
  double max_slope_angle_deg = 35.0;         // max allowed absolute slope angle from vertical [deg]
  double max_turn_angle_deg = 40.0;          // max allowed turn angle change from previous edge [deg]
};

enum class CraneRootObservationStatus : uint8_t {
  NOT_OBSERVED = 0,
  OBSERVED = 1
};

enum class CraneRootMotionStatus : uint8_t {
  UNKNOWN = 0,
  STATIONARY = 1,
  MOVING = 2
};

inline const char * to_string(CraneRootObservationStatus s) {
  switch (s) {
    case CraneRootObservationStatus::OBSERVED: return "OBSERVED";
    case CraneRootObservationStatus::NOT_OBSERVED: return "NOT_OBSERVED";
    default: return "NOT_OBSERVED";
  }
}

inline const char * to_string(CraneRootMotionStatus s) {
  switch (s) {
    case CraneRootMotionStatus::STATIONARY: return "STATIONARY";
    case CraneRootMotionStatus::MOVING: return "MOVING";
    case CraneRootMotionStatus::UNKNOWN: return "UNKNOWN";
    default: return "UNKNOWN";
  }
}

struct CraneFixedRootRoi {
  double x_min = 0.0;
  double x_max = 0.0;
  double y_min = 0.0;
  double y_max = 0.0;
  double z_min = 0.0;
  double z_max = 0.0;

  double centerX() const { return 0.5 * (x_min + x_max); }
  double centerY() const { return 0.5 * (y_min + y_max); }
  double centerZ() const { return 0.5 * (z_min + z_max); }
  double sizeX() const { return std::max(0.0, x_max - x_min); }
  double sizeY() const { return std::max(0.0, y_max - y_min); }
  double sizeZ() const { return std::max(0.0, z_max - z_min); }

  bool contains(double x, double y, double z) const {
    return (x >= x_min && x <= x_max &&
            y >= y_min && y <= y_max &&
            z >= z_min && z <= z_max);
  }
};

struct CraneFixedRootConfig {
  bool enabled = false;
  CraneFixedRootRoi left_roi{-1.25, -0.55, -0.25, 0.35, 4.80, 5.70};
  CraneFixedRootRoi right_roi{0.72, 1.42, -0.25, 0.35, 4.80, 5.80};
  size_t min_points = 15;                 // Minimum points inside ROI to declare OBSERVED
  int hold_frames = 3;                    // Frames to hold state before declaring NOT_OBSERVED
  double motion_speed_threshold = 0.05;   // [m/s] Speed threshold to declare MOVING
  double motion_disp_threshold = 0.06;    // [m] Cumulative/window displacement threshold to declare MOVING
  int motion_confirm_frames = 2;          // Consecutive frames to confirm motion transition
};

struct CraneFixedRootState {
  bool is_left = true;
  double fixed_center_x = 0.0;
  double fixed_center_y = 0.0;
  double fixed_center_z = 0.0;

  // Observation state
  CraneRootObservationStatus obs_status = CraneRootObservationStatus::NOT_OBSERVED;
  size_t point_count = 0;
  int consecutive_observed = 0;
  int consecutive_missed = 0;
  bool is_held = false;

  // Subcluster inside ROI
  bool cluster_present = false;
  int subcluster_id = -1;
  int layer_id = -1;
  double observed_centroid_x = 0.0;
  double observed_centroid_y = 0.0;
  double observed_centroid_z = 0.0;

  // Motion tracking of the pointcloud distribution inside ROI
  CraneRootMotionStatus motion_status = CraneRootMotionStatus::UNKNOWN;
  double prev_centroid_x = 0.0;
  double prev_centroid_y = 0.0;
  double prev_centroid_z = 0.0;
  bool prev_centroid_valid = false;
  double delta_x = 0.0;
  double delta_y = 0.0;
  double delta_z = 0.0;
  double delta_xy = 0.0;
  double delta_3d = 0.0;
  double velocity_xy = 0.0;
  double velocity_z = 0.0;
  double smoothed_speed = 0.0;
  int moving_confirm_count = 0;
  int stop_confirm_count = 0;
};

// Final Raw-Boundary boolean confirmation only. No geometry/association changes.
// Timestamp discontinuities never count as observed persistence.
struct CraneBoundaryMotionConfirmation {
  bool moving=false, initialized=false;
  double last_stamp=0, last_support=-1, start_since=-1, stop_since=-1;
  double start_elapsed=0, stop_elapsed=0, transition_stamp=0;
  void reset() { *this=CraneBoundaryMotionConfirmation{}; }
  bool update(double stamp, bool supported, bool evidence,
      double start_sec, double stop_sec, double loss_sec) {
    if(!std::isfinite(stamp))return moving;
    if(initialized && stamp==last_stamp)return moving;
    if(initialized && stamp<last_stamp)reset();
    if(initialized && stamp-last_stamp>loss_sec)reset();
    initialized=true;last_stamp=stamp;
    if(!supported){
      start_since=stop_since=-1;start_elapsed=stop_elapsed=0;
      if(last_support<0 || stamp-last_support>=loss_sec){
        if(moving)transition_stamp=stamp;
        moving=false;
      }
      return moving;
    }
    last_support=stamp;
    if(evidence){
      stop_since=-1;stop_elapsed=0;
      if(start_since<0)start_since=stamp;
      start_elapsed=stamp-start_since;
      if(!moving && start_elapsed+1e-9>=start_sec){moving=true;transition_stamp=stamp;}
    }else{
      start_since=-1;start_elapsed=0;
      if(stop_since<0)stop_since=stamp;
      stop_elapsed=stamp-stop_since;
      if(moving && stop_elapsed+1e-9>=stop_sec){moving=false;transition_stamp=stamp;}
    }
    return moving;
  }
};

struct CraneWorkStatusConfig {
  bool fsm_enabled = true; // Enabled by default; legacy Wire/Housing state is never modified.
  bool boundary_motion_enabled = false;
  bool boundary_temporal_enabled = false; // Opt-in; deployed defaults stay unchanged.
  double boundary_start_confirm_sec = .5, boundary_stop_confirm_sec = 1.5;
  double boundary_support_timeout_sec = 2.5;
  double cargo_roi_width_x_m = 1.20, cargo_roi_width_y_m = 1.20; // 2 x existing .60m connection margin
  double cargo_start_offset_m = .15; // Three existing 5cm occupancy cells below a real hook.
  double virtual_hook_offset_z_m = .5; // User-selected virtual offset, not a physical Hook calibration.
  double cargo_confirm_sec = 2.0;
  double cargo_loss_sec = 1.3;
  double lifting_event_hold_sec = .5; // Brief public display for a new clearance event.
  double lifting_clearance_delta_m = 0; // 0: reuse max(Z deadband, relocation threshold).
  double lifting_min_clearance_m = 0; // 0: reuse max(Z deadband, placement tolerance).
  double cargo_min_surface_width_m = .15;
  double cargo_min_fill_ratio = .35;
  double boundary_search_radius_m = .60;
  int boundary_min_coherent_points = 4;

  bool enabled = true;
  double grid_resolution_m = 0.05;     // BEV occupancy cell size r [m]
  double size_ratio_k = 1.5;           // k_size: required ratio A_C / A_H
  double connect_xy_margin = 0.60;     // maximum horizontal distance from hook to candidate [m]
  double connect_max_gap_z = 2.50;     // maximum vertical gap from hook bottom to candidate [m]
  size_t min_candidate_points = 20;    // minimum points for a valid work object candidate
  int temporal_window_n = 10;          // N frames sliding window
  int confirm_m_on = 5;                // M_on frames to transition work_status = 1
  int confirm_m_off = 2;               // M_off frames to transition work_status = 0
};

struct CraneTemporalStabilizationConfig {
  bool enabled = false;
  std::string mode = "sliding_window"; // "sliding_window" or "batch"
  int window_size = 5;                 // 5 or 10 frames
  double voxel_size_xy = 0.04;         // 0.04m XY
  double voxel_size_z = 0.03;          // 0.03m Z
  int min_hit_count = 2;               // minimum distinct frame hits
  bool temporal_stabilization_enabled = true;
  double stabilization_alpha = 0.20;   // 0.20 for smooth tracking
  double jump_tolerance_length = 0.50; // sudden jump threshold [m]
  double jump_tolerance_width = 0.25;  // [m]
  int jump_confirm_frames = 3;         // consecutive frames to confirm sudden jump
  int max_hold_missed_frames = 3;      // hold previous stabilized beam for up to 3 frames during dropout
  int node_confirm_frames = 2;         // frames to confirm new chain node
  int node_max_hold_frames = 2;        // frames to hold missing chain node
  int largest_swap_confirm_frames = 3; // consecutive frames a new largest candidate must be larger to swap
  double largest_match_dist = 0.50;    // maximum centroid distance [m] to match same largest cluster
  double yaw_jump_tolerance_deg = 30.0;// threshold to detect yaw jump [deg]
  double yaw_alpha = 0.15;             // EMA factor for smooth yaw tracking
};

struct CraneConnectionStabilizationConfig {
  bool enabled = true;
  int history_frames = 10;
  int min_direct_frames = 4;
  int min_transition_count = 0;
  int missing_hold_frames = 3;
  bool spatial_continuity_enabled = true;
  int spatial_min_direct_frames = 1;
  double spatial_max_gap_z = 0.80;
  double spatial_max_dxy = 0.35;
};

enum class CraneFixedRoiStatus : uint8_t {
  NONE = 0,
  CANDIDATE = 1,   // 2초 관측 중
  CONFIRMED = 2,   // 확정된 고정 ROI
  HELD = 3,        // 일시 관측 소실
  EXPIRED = 4      // 소멸
};

inline const char * to_string(CraneFixedRoiStatus s) {
  switch (s) {
    case CraneFixedRoiStatus::CANDIDATE: return "CANDIDATE";
    case CraneFixedRoiStatus::CONFIRMED: return "CONFIRMED";
    case CraneFixedRoiStatus::HELD: return "HELD";
    case CraneFixedRoiStatus::EXPIRED: return "EXPIRED";
    default: return "NONE";
  }
}

struct CraneDualRoiAnchorConfig {
  bool enabled = false;
  bool publish_markers = true;
  double fixed_roi_size_xy = 0.36;        // [m] Fixed Monitoring ROI XY span (36cm)
  double fixed_roi_size_z = 0.40;         // [m] Fixed Monitoring ROI Z height (40cm)
  double moving_roi_size_xy = 0.60;       // [m] Moving Monitoring ROI XY span (60cm)
  double moving_roi_size_z = 0.56;        // [m] Moving Monitoring ROI Z height (56cm)
  int confirm_frames = 3;                 // Frames required to initially lock Fixed Anchor
  int min_direct_evidence_frames = 2;     // Min direct observation frames before locking
  double max_lock_slope_deg = 28.0;       // Max slope angle [deg] for initial straightness check
  double max_lock_turn_deg = 30.0;        // Max turn angle [deg] for initial straightness check
  double noise_deadband_xy = 0.06;        // [m] Stationary XY jitter deadband
  double noise_deadband_z = 0.08;         // [m] Stationary Z jitter deadband
  double relocate_thresh_xy = 0.10;       // [m] Genuine XY movement threshold for relocation
  double relocate_thresh_z = 0.12;        // [m] Genuine Z movement threshold for relocation
  int relocate_confirm_frames = 3;        // Consecutive consistent frames to relocate Fixed Anchor
  int anchor_memory_frames = 30;          // Frames to remember confirmed anchors for return-to-origin recovery
  bool beam_intrusion_guard_enabled = true;
  double beam_min_length_m = 0.80;        // [m] Min horizontal length to classify intruding cluster as Cargo Beam
  double beam_min_aspect_ratio = 2.2;     // Min horizontal elongation ratio for beam guard
  int intrusion_hold_frames = 50;         // Max frames to preserve wire anchor during beam occlusion/intrusion
};

struct CraneZTemporalMedianConfig {
  bool enabled = true;
  int window_frames = 3;                  // Temporal median filter window size (odd number, e.g. 3)
};

enum class CraneWireBundleMode : uint8_t {
  NONE = 0,
  BOTH_STRANDS = 1,                 // Two distinct strands resolved, midpoint computed
  ESTIMATED_FROM_SINGLE_A = 2,      // Only Strand A observed; Center estimated via remembered offset
  ESTIMATED_FROM_SINGLE_B = 3,      // Only Strand B observed; Center estimated via remembered offset
  OVERLAPPED = 4,                   // Strands overlapped into a single clump
  PROVISIONAL_SINGLE = 5            // Single strand observed without prior 2-strand memory
};

inline const char * to_string(CraneWireBundleMode m) {
  switch (m) {
    case CraneWireBundleMode::BOTH_STRANDS: return "BOTH_STRANDS";
    case CraneWireBundleMode::ESTIMATED_FROM_SINGLE_A: return "ESTIMATED_FROM_SINGLE_A";
    case CraneWireBundleMode::ESTIMATED_FROM_SINGLE_B: return "ESTIMATED_FROM_SINGLE_B";
    case CraneWireBundleMode::OVERLAPPED: return "OVERLAPPED";
    case CraneWireBundleMode::PROVISIONAL_SINGLE: return "PROVISIONAL_SINGLE";
    default: return "NONE";
  }
}

struct WireBundleGeometry {
  CraneWireBundleMode mode = CraneWireBundleMode::NONE;
  double center_x = 0.0, center_y = 0.0, center_z = 0.0;
  double strand_a_x = 0.0, strand_a_y = 0.0;
  double strand_b_x = 0.0, strand_b_y = 0.0;
  bool strand_a_valid = false;
  bool strand_b_valid = false;
  double spacing_dx = 0.0, spacing_dy = 0.0; // Vector from Strand A to Strand B
  double spacing_dist = 0.0;
  int confidence_frames = 0;
};

struct CraneVirtualRopeConfig {
  bool enabled = false;                  // Default false for isolated unit tests; enabled via YAML
  double wire_min_spacing_m = 0.10;      // [m] Min separation to qualify as 2 distinct strands
  double wire_max_spacing_m = 0.45;      // [m] Max separation to belong to the same wire bundle
  double wire_merge_span_m = 0.16;       // [m] Span threshold below which strands are considered overlapped
  int memory_frames = 30;                // Frames to remember 2-strand relative spacing and orientation
  bool publish_markers = true;           // Visualize Virtual Rope Centerline in RViz
};

enum class CraneSheaveDetectionStatus : uint8_t {
  NONE = 0,
  DIRECT_OBSERVED = 1,              // Current point-supported observation; axis has its own status
  ESTIMATED = 2,                    // Partial observation or fallback to nominal geometry
  HELD = 3                          // Temporarily held from previous frame
};

inline const char * to_string(CraneSheaveDetectionStatus s) {
  switch (s) {
    case CraneSheaveDetectionStatus::DIRECT_OBSERVED: return "DIRECT_OBSERVED";
    case CraneSheaveDetectionStatus::ESTIMATED: return "ESTIMATED";
    case CraneSheaveDetectionStatus::HELD: return "HELD";
    default: return "NONE";
  }
}

// Immutable, manually reviewed observations; never updated by an online track.
struct HousingShapeReference {
  std::string id, source, label;
  int family_id = -1, source_frame = -1;
  bool manually_checked = false;
  double source_stamp = 0.0, sensor_range = 0.0, point_count = 0.0;
  double density = 0.0, reliability = 0.0;
  double min_z_offset = 0.0, max_z_offset = 0.0, peak_z_offset = 0.0;
  std::array<double, 3> dimensions{}, spectrum{}, z_distribution{}, horizontal_distribution{};
  std::array<double, 3> source_center{}, view_direction{}; // Provenance, not a runtime position prior.
  // Optional offline Raw-surface descriptor; no online registration.
  bool has_local_shape = false;
  std::array<double,3> local_extent{};
  std::array<double,27> local_occupancy{};
  bool trusted() const { return manually_checked && (label == "USER_SEEDED" || label == "VERIFIED"); }
};

struct HousingGeometryMemory {
  bool ever_confirmed = false;
  // Bounded recent full-reference observation, never refreshed by recovery.
  std::array<double,3> recent_dimensions{}, recent_spectrum{}, recent_horizontal{}, recent_center{};
  double recent_point_count=0, recent_range=0, recent_reference_stamp=0, recent_reference_timeout_sec=0;
  int recent_reference_family=-1;
  std::string recent_reference_id;

  int reference_family_id = -1;
  int track_id = -1;
  bool dormant = false; // Bounded reacquisition template; no held output or anchor
  double span_x = 0.0, span_y = 0.0; // World-axis spans, distinct from sorted length/width
  double wire_dir_x = 1.0, wire_dir_y = 0.0, wire_spacing = 0.0;
  double wire_mid_s = 0.0, wire_mid_t = 0.0;
  bool initialized = false;
  double length = 0.0;        // [m] Learned housing horizontal length (major span)
  double width = 0.0;         // [m] Learned housing horizontal width (minor span)
  double height = 0.0;        // [m] Learned housing height (max_z - min_z)
  double aspect_ratio = 1.0;  // length / width
  size_t point_count = 0;     // Learned point count
  double housing_center_x = 0.0;
  double housing_center_y = 0.0;
  double housing_center_z = 0.0;
  double axis_center_x = 0.0;
  double axis_center_y = 0.0;
  double axis_center_z = 0.0;
  std::array<double, 3> shape_eigenvalues{}; // Normalized PCA spectrum, descending
  std::array<double, 3> horizontal_distribution{}; // Distribution across observed strand separation
  std::array<double, 3> z_distribution{};   // Normalized vertical thirds
  double wire_offset_xy = 0.0;
  double last_direct_stamp = 0.0;
  double last_frame_stamp = 0.0;
  double frame_period = 0.1; // Updated from consecutive input timestamps
  int confirm_count = 0;      // Consecutive frames observed
  int missed_frames = 0;
  bool confirmed = false;     // True when confirm_count >= confirm_frames
};

struct CraneSheaveGeometry {
  bool track_memory_matched=false, negative_reference_rejected=false, temporal_pending=false, reference_from_learned=false;
  std::string recovery_reference_id;

  std::string reference_id;
  int reference_family_id = -1;
  bool reference_matched = false, reference_trusted = false, reference_assisted = false;
  double reference_score = -1.0, sensor_range = -1.0, point_density = 0.0, shape_reliability = 0.0;
  std::array<double, 3> view_direction{};
  std::array<double,3> local_extent{};
  std::array<double,27> local_occupancy{};
  double reference_local_score = -1.0;
  CraneSheaveDetectionStatus status = CraneSheaveDetectionStatus::NONE;
  // Outer point cloud centroid (observed distribution / Housing Center)
  double outer_center_x = 0.0, outer_center_y = 0.0, outer_center_z = 0.0;
  // Internal rotation axis; coordinates are valid only when axis_status is not NONE.
  double axis_center_x = 0.0, axis_center_y = 0.0, axis_center_z = 0.0;
  // Unit vector along horizontal rotation axis (perpendicular to wire plane)
  double axis_dir_x = 0.0, axis_dir_y = 0.0, axis_dir_z = 0.0;
  // Unit vector along wire strand separation plane (wheel disc plane)
  double wire_dir_x = 0.0, wire_dir_y = 0.0;
  double radius = 0.0;                  // [m] Fitted or nominal wheel radius
  double arc_angle_deg = 0.0;           // [deg] Observed angular arc span
  double circle_fit_rms = 0.0;          // [m] Circle fitting residual RMS
  size_t point_count = 0;
  std::vector<int> point_indices;
  CraneSheaveDetectionStatus axis_status = CraneSheaveDetectionStatus::NONE;
  bool wire_supported = false;
  int wire_observation_mode = 0; // 2=both direct, 1=single matched, 3=overlap matched
  double wire_spacing = 0.0, wire_mid_s = 0.0, wire_mid_t = 0.0;
  std::vector<int> candidate_ids;
  int candidate_count = 0, geometry_rejections = 0, wire_rejections = 0;
  // Diagnostic counts only: proposal, spatial, plane, raw, shape, dimensions,
  // wire, reference, association position, memory shape, ROI, no-chain.
  std::array<int,12> rejection_counts{};
  std::array<double,3> closest_rejected_center{};
  double closest_rejected_distance=-1;
  std::string closest_rejection_reason;
  double match_score = 0.0;
  double detection_ms = 0.0, matching_ms = 0.0, tracking_ms = 0.0;
  bool points_from_current_frame = false;
  bool geometry_matched = false;
  std::array<double, 3> shape_eigenvalues{};
  std::array<double, 3> horizontal_distribution{}; // Distribution across observed strand separation
  std::array<double, 3> z_distribution{};
  double wire_offset_xy = 0.0;
  double min_x = 0.0, max_x = 0.0, min_y = 0.0, max_y = 0.0, min_z = 0.0, max_z = 0.0;
  bool axis_uncertain = true;          // True when partial observation leaves axis ambiguous
  int subcluster_id = -1;
  int layer_id = -1;

  // Sheave Housing geometry properties
  double housing_length = 0.0;
  double housing_width = 0.0;
  double housing_height = 0.0;
};

struct CraneSheaveConfig {
  bool track_continuity_enabled = false; // Opt-in Housing-only policy; no library/threshold changes.

  bool reference_matching_enabled = false;
  bool reference_wire_mode_search = false; // Internal: only a reviewed-reference proposal may enable this.
  std::vector<HousingShapeReference> learned_references; // <=4 offline exemplars; tracking-only, never cold seeds.
  std::vector<HousingShapeReference> references; // At most eight reviewed, distinct observations.
  std::vector<double> debug_seed_world; // Marker only; never consulted by detection/association.
  bool enabled = false;                 // Default false for isolated unit tests; enabled via YAML
  double min_radius_m = 0.08;           // Legacy circle parameter; retained for YAML compatibility
  double max_radius_m = 0.28;           // Legacy circle parameter; unused for housing identity
  double default_radius_m = 0.155;      // Legacy nominal radius; never used to invent an axis
  size_t min_arc_points = 12;           // Compatibility name: minimum current surface support
  double max_circle_rms_m = 0.055;      // Legacy circle parameter; no wheel fit is trusted
  double max_sheave_width_m = 0.55;     // [m] Max horizontal span to reject cargo beam
  int confirm_frames = 2;               // Consecutive direct observations to confirm housing identity
  double surface_gap_m = 0.08;          // [m] Connectivity across 4x4x3 cm voxel/layer boundaries
  bool terminate_chain_at_sheave = true;// Output copy only; requires independent, reliable axis evidence
  bool publish_markers = true;          // Publish housing evidence/ROI; suppress unknown axes
};

// Independent, opt-in Crane foundation. No legacy detector is run here.
struct CraneSlopeConfig {
  int azimuth_columns = 666; // supplied code's width
  int lidar_channels = 128;  // CX128S2 PointCloud2 ring range; not an angle LUT
  // CX128S2 uses the installed driver's ring + prism model. Uniform is for
  // the original C32 start/step table (row = channels - ring - 1).
  std::string lidar_model = "cx128s2";
  double lidar_degree_start = -15.0;
  double lidar_degree_step = 1.0;
  double lidar_height = 0.0; // 0: use TF origin.z - ground_z; positive: explicit H
  double cx128s2_prism_offset_deg = 0.0;
  std::vector<double> cx128s2_prism_angles_deg = {0.0, -0.17, -0.34, -0.51};
  double model_angle_tolerance_deg = 0.25;
  double max_slope_deg = 10.0;
  double max_vertical_height = 0.08;
  double ground_z = 0.0;
  // Optional final World-Z override; negative infinity disables it.
  double force_ground_below_z = -std::numeric_limits<double>::infinity();
  bool include_ground_in_clustering = false;
  bool horizontal_enabled = false; // false retains the original vertical-only result
  double horizontal_max_azimuth_gap_deg = 0.5;
  bool height_mode = false; // select mode independently from the enabled switch
  CraneHeightConfig height;
  CraneBoundaryClusterConfig boundary_clustering;
  CraneZGaussianConfig z_gaussian;
  CraneClosedLoopConfig closed_loop;
  CraneRoiFilterConfig roi_filter;
  CraneHookConfig hook;
  CraneTopDownHookConfig top_down_hook;
  CraneFixedRootConfig fixed_root;
  CraneWorkStatusConfig work_status;
  CraneTemporalStabilizationConfig temporal_stabilization;
  CraneExceptionGateConfig exception_gate;
  CraneConnectionStabilizationConfig connection_stabilization;
  CraneDualRoiAnchorConfig dual_roi_anchor;
  CraneZTemporalMedianConfig z_temporal_median;
  CraneVirtualRopeConfig virtual_rope;
  CraneSheaveConfig sheave;
};

struct CraneHookChainNode {
  int subcluster_id = -1;
  int layer_id = -1;
  double x = 0.0, y = 0.0, z = 0.0;
  double min_x = 0.0, max_x = 0.0;
  double min_y = 0.0, max_y = 0.0;
  double min_z = 0.0, max_z = 0.0;
  size_t point_count = 0;
  std::vector<int> point_indices;
  bool is_root = false;
  int parent_subcluster_id = -1;
  double gap_z_to_parent = 0.0;
  double dxy_to_parent = 0.0;
  double slope_angle_deg = 0.0;
  double turn_angle_deg = 0.0;
  bool is_estimated_gap = false;
  bool is_held = false;
  int held_frames = 0;
  bool is_connection_stable = false;
  bool is_spatial_promoted = false;

  // Dual-ROI Anchoring & Motion Monitoring fields
  bool anchor_locked = false;
  CraneFixedRoiStatus fixed_status = CraneFixedRoiStatus::NONE;
  int anchor_id = -1;
  double initial_anchor_x = 0.0, initial_anchor_y = 0.0, initial_anchor_z = 0.0;
  double anchor_x = 0.0, anchor_y = 0.0, anchor_z = 0.0;
  double moving_x = 0.0, moving_y = 0.0, moving_z = 0.0;
  double raw_obs_x = 0.0, raw_obs_y = 0.0, raw_obs_z = 0.0;
  double disp_from_anchor_xy = 0.0, disp_from_anchor_z = 0.0;
  int relocate_count = 0;
  bool is_relocated_this_frame = false;
  bool is_beam_intruded = false;
  CraneFixedRootRoi fixed_roi;
  CraneFixedRootRoi moving_roi;

  // Virtual Rope Centerline & 2-Strand Bundle fields
  bool has_virtual_rope = false;
  WireBundleGeometry wire_bundle;

  // Sheave Detection fields
  bool is_sheave = false;
  CraneSheaveGeometry sheave;
};

struct CraneHookNearestCluster {
  bool valid = false;
  int subcluster_id = -1;
  int layer_id = -1;
  int group_id = -1;
  size_t point_count = 0;
  double centroid_x = 0.0, centroid_y = 0.0, centroid_z = 0.0;
  double ref_x = 0.0, ref_y = 0.0, ref_z = 0.0;
  double min_x = 0.0, max_x = 0.0;
  double min_y = 0.0, max_y = 0.0;
  double min_z = 0.0, max_z = 0.0;
  double length = 0.0, width = 0.0, height = 0.0;
  double yaw = 0.0;
  double area_xy = 0.0;
  double dxy = 0.0;
  double dz = 0.0;
  double dist_3d = 0.0;
  bool is_held = false;
  int held_frames = 0;
};

struct CraneHookChain {
  int chain_id = -1;
  std::string side = "UNKNOWN"; // "LEFT", "RIGHT", or "OTHER"
  CraneHookConnectionState state = CraneHookConnectionState::NONE;
  int persistent_frames = 0;
  int total_observed_frames = 0;
  int missed_frames = 0;
  bool is_held = false;
  int held_frames = 0;

  // Root node (top-most)
  int root_subcluster_id = -1;
  double root_x = 0.0, root_y = 0.0, root_z = 0.0;
  bool is_observed = false;
  CraneRootObservationStatus root_obs_status = CraneRootObservationStatus::NOT_OBSERVED;
  CraneRootMotionStatus root_motion_status = CraneRootMotionStatus::UNKNOWN;
  CraneFixedRootRoi root_roi;
  double root_observed_cx = 0.0, root_observed_cy = 0.0, root_observed_cz = 0.0;
  double root_delta_x = 0.0, root_delta_y = 0.0, root_delta_z = 0.0;
  double root_speed = 0.0;

  // Lowest node (lowest reachable subcluster in downward chain -> Hook block)
  int lowest_subcluster_id = -1;
  double lowest_x = 0.0, lowest_y = 0.0, lowest_z = 0.0;
  double lowest_min_z = 0.0, lowest_max_z = 0.0;
  size_t lowest_point_count = 0;
  std::vector<int> lowest_point_indices;

  // Path of nodes from root to lowest
  std::vector<CraneHookChainNode> nodes;
  double total_depth = 0.0; // root_z - lowest_z
  bool has_gap = false;
  double max_gap_z = 0.0;
  double max_slope_angle_deg = 0.0;
  double max_turn_angle_deg = 0.0;
  int stable_edge_count = 0;
  int locked_anchor_count = 0;
  int candidate_anchor_count = 0;
  int relocated_anchor_count = 0;
  int beam_intruded_count = 0;
  int expired_anchor_count = 0;

  // Filtered lowest position (Hook location)
  double filtered_x = 0.0, filtered_y = 0.0, filtered_z = 0.0;
  double vx = 0.0, vy = 0.0, vz = 0.0;

  // Ground reference & distance
  bool ground_valid = false;
  double ground_z = 0.0;
  double ground_distance = 0.0;

  // Nearest cluster below this Hook inside Hook ROI
  CraneHookNearestCluster nearest_cluster;
  // Spatially largest connected cluster in this Top-Down structure below the Root
  CraneHookNearestCluster largest_cluster;
  // 1st Exception Gate result
  CraneExceptionGateResult exception_gate;

  // Sheave Detection result
  bool has_sheave = false;
  CraneSheaveGeometry sheave;
};

struct CraneTopDownHookResult {
  std::vector<CraneHookChain> chains;
  // Read-only wire output snapshot, before any validated display terminal.
  std::vector<CraneHookChain> original_chains;
  CraneHookChainNode left_housing, right_housing;
  HousingGeometryMemory left_housing_memory, right_housing_memory;
  int left_chain_idx = -1;
  int right_chain_idx = -1;
  bool left_detected = false;
  bool right_detected = false;
  bool prepare_ready = false;
  double mean_ground_distance = 0.0;
  bool crane_moving = false;
  std::string crane_move_reason = "STOPPED";
  double root_mean_x = 0.0, root_mean_y = 0.0, root_mean_z = 0.0;
  double root_dx = 0.0, root_dy = 0.0;
  double root_vx = 0.0, root_vy = 0.0;
  CraneHookNearestCluster left_nearest_cluster;
  CraneHookNearestCluster right_nearest_cluster;
  CraneHookNearestCluster left_largest_cluster;
  CraneHookNearestCluster right_largest_cluster;
  CraneExceptionGateResult left_exception_gate;
  CraneExceptionGateResult right_exception_gate;
  CraneFixedRootState left_root_state;
  CraneFixedRootState right_root_state;
  double processing_ms = 0.0;
};

struct CraneBeamGeometry {
  bool detected = false;
  bool is_held = false;
  int subcluster_id = -1;
  int group_id = -1;
  double centroid_x = 0.0;
  double centroid_y = 0.0;
  double centroid_z = 0.0;
  double length = 0.0;
  double width = 0.0;
  double yaw = 0.0;
  double elongation = 0.0;
  size_t point_count = 0;
  double min_x = 0.0;
  double max_x = 0.0;
  double min_y = 0.0;
  double max_y = 0.0;
  double min_z = 0.0;
  double max_z = 0.0;
  double dz = 0.0;
  double dz_relative = 0.0;
  std::vector<int> point_indices;

  // Hook-connected Beam Seed & Spatial Expansion fields
  int seed_subcluster_id = -1;
  int seed_layer_id = -1;
  double seed_z = 0.0;
  double internal_length = 0.0;
  double internal_width = 0.0;
  size_t internal_point_count = 0;
  size_t outside_point_count = 0;
  double area_xy = 0.0;
  double robust_length = 0.0;
  size_t connected_subcluster_count = 0;
  std::vector<int> connected_subcluster_ids;
  std::vector<int> excluded_candidate_ids;
};

struct CraneHookWorkCandidate {
  int subcluster_id = -1;
  int group_id = -1;
  double centroid_x = 0.0;
  double centroid_y = 0.0;
  double centroid_z = 0.0;
  double min_z = 0.0;
  double max_z = 0.0;
  double area_xy = 0.0;
  size_t point_count = 0;
  std::vector<int> point_indices;
};

struct CraneHookWorkStatus {
  std::string side = "UNKNOWN"; // "LEFT", "RIGHT"
  bool hook_detected = false;
  double hook_x = 0.0, hook_y = 0.0, hook_z = 0.0;
  double hook_area = 0.0;
  double object_area = 0.0;
  double size_ratio = 0.0;
  std::string hook_source = "VIRTUAL_SHEAVE", hook_observation = "INVALID";
  std::string display_state = "WAITING"; // Presentation of existing READY/confirmation states.
  std::string start_evidence = "NO_COHERENT_CURRENT_Z_RISE";
  double hook_rise = 0.0, cargo_rise = 0.0;
  bool attached_current_frame = false;
  bool work_status = false;
  int positive_frames_in_window = 0;
  std::vector<int> candidate_subcluster_ids;
  std::vector<int> object_point_indices;
  double min_x = 0.0, max_x = 0.0;
  double min_y = 0.0, max_y = 0.0;
  double min_z = 0.0, max_z = 0.0;
  double centroid_x = 0.0, centroid_y = 0.0, centroid_z = 0.0;
};

struct CraneBoundaryObservation {
  double search_x=0,search_y=0,x=0,y=0,z=0,dx=0,dy=0;
  size_t point_count=0;
  std::string observation="NONE";
};

struct CraneWorkStatusResult {
  bool fsm_enabled=false, boundary_motion_enabled=false, boundary_moving=false;
  bool boundary_raw_evidence=false, boundary_raw_supported=false;
  double boundary_start_elapsed=0, boundary_stop_elapsed=0, boundary_transition_stamp=0;
  double boundary_temporal_us=0;
  std::array<CraneBoundaryObservation,8> boundary_points{};
  std::string boundary_reason="INSUFFICIENT_RAW_SUPPORT";
  CraneBeamState fsm_state=CraneBeamState::SEARCH_WIRE;
  std::array<std::string,2> sheave_visibility{{"UNKNOWN","UNKNOWN"}};
  std::array<CraneFixedRootRoi,2> cargo_detection_rois{};
  CraneFixedRootRoi cargo_tracking_roi{},completion_boundary{};
  std::string cargo_side="UNKNOWN",cargo_observation="NONE";
  int cargo_track_id=-1,cargo_cluster_id=-1;
  bool geometry_locked=false;
  double cargo_x=0,cargo_y=0,cargo_z=0,length=0,width=0,height=0,yaw=0;
  double confirmed_observation_sec=0;

  bool work_status = false;
  int working_status = 0; // Existing Cargo FSM: 0 idle, 1 working, 2 drop.
  int public_working_status = 0; // Projection only: additionally 3 lifting; work_status stays active.
  bool beam_ground_clearance_valid=false,initial_clearance_valid=false;
  double beam_ground_clearance_m=0,beam_bottom_z=0,ground_surface_z=0;
  double initial_clearance_m=0,clearance_delta_m=0;
  std::vector<double> beam_ground_clearance_samples_m,beam_bottom_samples_z,ground_surface_samples_z;
  std::vector<bool> beam_ground_clearance_sample_valid;
  std::string beam_ground_clearance_reason="NO_CURRENT_BEAM";
  // Seed dimensions above remain the existing KF/Size Lock contract. Extension is diagnostic.
  bool extension_valid=false;
  std::string extension_observation="NONE",extension_reason="NO_CONFIRMED_SEED";
  double extended_x=0,extended_y=0,extended_z=0,extended_length=0,extended_width=0,extended_height=0,extended_yaw=0;
  double observed_length_sum=0,extension_ms=0,extension_last_full_stamp=0;
  double gap_roi_z_min=0,gap_roi_z_max=0;
  std::array<double,6> observed_bounds{}; // xmin,xmax,ymin,ymax,zmin,zmax of current Raw returns
  std::vector<int> extended_raw_indices,extension_cluster_ids;
  std::vector<std::pair<int,std::string>> extension_rejections;
  std::vector<std::array<double,2>> observed_intervals,inferred_gaps; // longitudinal coordinates; gaps contain NO invented points
  std::string gap_diagnostic="UNKNOWN",gap_reason="NO_CURRENT_GEOMETRY";
  std::array<std::string,3> gap_sides{{"UNKNOWN","UNKNOWN","UNKNOWN"}}; // side+,side-,under
  std::array<double,3> lower_layer_z{},vertical_gaps{};
  std::array<size_t,3> gap_roi_points{};
  double gap_confirmed_sec=0,drop_started_stamp=0;
  CraneHookWorkStatus left_hook;
  CraneHookWorkStatus right_hook;
  std::string status_reason = "NONE";
  double processing_ms = 0.0;
};

struct CraneSlopeResult {
  CraneBeamState state = CraneBeamState::IDLE;
  // Indices always address the caller's transformed point vector. Nothing is deleted.
  std::vector<int> ground, non_ground, unknown, clustering;
  std::vector<CraneSlopeLabel> labels;
  // Before the final height override. Empty when horizontal checking is disabled.
  std::vector<CraneSlopeLabel> vertical_labels, horizontal_labels;
  size_t horizontal_neighbor_pairs = 0, horizontal_broken_pairs = 0;
  size_t horizontal_gap_breaks = 0, horizontal_unsupported_pairs = 0, horizontal_runs = 0;
  size_t observed_rings = 0, neighbor_pairs = 0, broken_pairs = 0;
  size_t overwritten_points = 0, unsupported_points = 0, model_mismatch_points = 0;
  size_t workspace_bytes = 0, cluster_count = 0;
  double segmentation_ms = 0.0;
  bool pose_valid = false;
  CraneHeightResult height;
  CraneBoundaryResult boundary_clustering;
  CraneZGaussianResult z_gaussian;
  CraneClosedLoopResult closed_loop;
  CraneTopDownHookResult top_down_hook;
  CraneHookResult hook;
  CraneBeamGeometry beam;
  bool crane_moving = false;
  CraneWorkStatusResult work_status;
  int working_status = 0;
};

class CranePipeline {
public:
  bool beamWorkProcessingAllowed() const {
    return ordered_state_==CraneBeamState::SUSPENDED || ordered_state_==CraneBeamState::LOWERING;
  }

  struct StabilizedBeamReference {
    bool valid = false;
    int confirmed_frames = 0;
    int missed_frames = 0;
    int consecutive_jump_frames = 0;
    int seed_subcluster_id = -1;
    int main_layer_id = -1;
    double seed_z = 0.0;
    double centroid_x = 0.0, centroid_y = 0.0, centroid_z = 0.0;
    double length = 0.0, width = 0.0, yaw = 0.0;
    double min_x = 0.0, max_x = 0.0;
    double min_y = 0.0, max_y = 0.0;
    double min_z = 0.0, max_z = 0.0;
    double area_xy = 0.0;
    int point_count = 0;
  };

  explicit CranePipeline(const CraneSlopeConfig & config = {});
  void updateConfig(const CraneSlopeConfig & config);
  const CraneSlopeConfig & config() const noexcept { return config_; }
  const CraneSlopeResult & process(const std::vector<PointXYZIRT> & world_points,
      const std::vector<int> & valid_indices, const tf2::Transform & sensor_to_world,
      bool ring_available, VisibilityModel * visibility = nullptr);
  const CraneClosedLoopResult & processClosedLoopGrouping(
      const std::vector<PointXYZIRT> & world_points,
      const std::vector<CraneSubcluster> & raw_subclusters,
      double external_bg_z = std::numeric_limits<double>::quiet_NaN(),
      double external_dz_bg = std::numeric_limits<double>::quiet_NaN());
  const CraneSlopeResult & processHookAndBeamDetection(
      const std::vector<PointXYZIRT> & world_points,
      const std::vector<CraneSubcluster> & subclusters,
      double stamp_sec = 0.0,
      VisibilityModel * visibility = nullptr,
      const std::vector<PointXYZIRT> * current_observation_points = nullptr);
  // Seed-conditioned use of the existing Z Gaussian grouping. No upstream layer/track mutation.
  const CraneZGaussianResult & prepareCargoExtension(const std::vector<PointXYZIRT> & raw,
      const tf2::Transform & sensor_to_world, VisibilityModel * current_visibility=nullptr);
  void updateCargoExtension(const std::vector<PointXYZIRT> & raw,
      const std::vector<CraneSubcluster> & full_layer_clusters,double stamp,
      const std::function<void()> & build_current_visibility = {});
  void recoverCargoFromRawFragments(const std::vector<PointXYZIRT>& raw,
      const std::vector<CraneSubcluster>& clusters,double stamp);
  void reset();
  static uint32_t heightColor(int id) {
    return (static_cast<uint32_t>(id)*0x9e3779u+0x5bd1e9u)&0xffffffu;
  }
  static double computeOccupiedArea(
      const std::vector<int> & point_indices,
      const std::vector<PointXYZIRT> & points,
      double grid_resolution_m);
  void observeClusters(size_t count) { result_.cluster_count = count; }
  const CraneSlopeResult & result() const { return result_; }
  bool confirmedCraneMoving() const;
  void setSheaveMotionFrame(const std::string & frame);
  void updateBeamGroundClearance(const std::vector<PointXYZIRT> & raw,double stamp,
      const CraneBeamConfig & beam_config);

  // Pre-Z-Gaussian Temporal Accumulation & Stabilization
  void setExternalCraneMoving(bool moving) { external_crane_moving_ = moving; }
  bool hasAccumulatedPoints() const { return has_accumulated_points_; }
  const std::vector<PointXYZIRT> & accumulatedPoints() const { return accumulated_points_; }
  const std::vector<int> & accumulatedIndices() const { return accumulated_indices_; }
  const StabilizedBeamReference & stabilizedBeam() const { return stabilized_beam_; }
  void clearStationaryAccumulation();
  void updateAccumulatedPoints(const std::vector<PointXYZIRT> & points,
                               const std::vector<int> & indices,
                               double stamp_sec);
  void stabilizeBeamResult(CraneBeamGeometry & b, double stamp_sec);
private:
  struct TrackedZLayer {
    int stable_id = 1;
    double mean_z = 0.0;
    double std_z = 0.03;
    int missed_frames = 0;
  };
  struct TrackedClusterState {
    int stable_id = 1;
    double cx = 0.0, cy = 0.0, cz = 0.0;
    int missed_frames = 0;
  };
  struct TrackedSubclusterState {
    int stable_id = 1;
    int layer_id = -1;
    double cx = 0.0, cy = 0.0, cz = 0.0;
    double cumulative_dz_relative = 0.0;
    int tracked_frames = 1;
    int missed_frames = 0;
  };
  struct Sample {
    int index = 0, column = 0;
    uint16_t ring = 0;
    double theta = 0.0, coordinate = 0.0, table = 0.0, denominator = 0.0;
    bool supported = false;
  };
  // Stable centered covariance: xx,xy,xz,yy,yz,zz; no 3D eigensolver.
  struct PlaneMoments {
    size_t count=0;
    std::array<double,3> mean{};
    std::array<double,6> scatter{};
    void add(const PointXYZIRT &);
    void combine(const PlaneMoments &);
  };
  struct FastPlane {
    double a=0,b=0,c=0,rms=0;
    std::array<double,3> center{},normal{{0,0,1}};
    double predict(double x,double y) const {return center[2]+a*(x-center[0])+b*(y-center[1]);}
  };
  struct PatchCell {
    int64_t x=0,y=0;
    bool operator==(const PatchCell & a) const {return x==a.x && y==a.y;}
  };
  struct PatchCellHash {
    size_t operator()(const PatchCell & a) const {
      const size_t h=std::hash<int64_t>{}(a.x);
      return h^(std::hash<int64_t>{}(a.y)+0x9e3779b9+(h<<6)+(h>>2));
    }
  };
  struct HeightPatch {
    PatchCell cell;
    int next_cell=-1,head=-1,parent=-1,next_member=-1,tail=-1,surface_id=-1;
    PlaneMoments moments;
    FastPlane model;
    double min_x=0,max_x=0,min_y=0,max_y=0,min_z=0,max_z=0,residual_bound=0;
    size_t patch_count=1;
    bool valid=false;
  };
  struct PatchEdge {int a=-1,b=-1;double x=0,y=0;};
  static bool fitPlane(const PlaneMoments &,FastPlane &);
  int patchRoot(int);
  const CraneSlopeResult & processHeight(const std::vector<PointXYZIRT> &,
      const std::vector<int> &,const tf2::Transform &,VisibilityModel *);
  void clearHeightResult();
  void processBoundaryClustering(const std::vector<PointXYZIRT> & points, const tf2::Transform & pose);
  void processZGaussianGrouping(const std::vector<PointXYZIRT> & points, const tf2::Transform & pose,
      const std::vector<int> * selected_raw=nullptr,CraneZGaussianResult * diagnostic=nullptr);
  std::unordered_map<PatchCell,int,PatchCellHash> patch_cells_;
  std::vector<HeightPatch> height_patches_;
  std::vector<int> point_next_,region_points_;
  std::vector<PatchEdge> patch_edges_;
  CraneSlopeConfig config_;
  std::array<double,128> base_sin_{}, base_cos_{}, prism_sin_{}, prism_cos_{};
  CraneSlopeResult result_;
  std::vector<Sample> samples_;
  std::vector<tf2::Vector3> horizontal_rays_;
  std::vector<int> depth_, rows_;
  std::vector<double> radius_prefix_;
  std::vector<tf2::Vector3> axes_;
  std::vector<int8_t> axis_states_;
  std::bitset<65536> rings_;
  std::vector<TrackedZLayer> tracked_z_layers_;
  int next_stable_layer_id_ = 1;
  std::vector<TrackedClusterState> tracked_boundary_clusters_;
  int next_boundary_cluster_id_ = 1;
  std::vector<TrackedSubclusterState> tracked_subclusters_;
  int next_subcluster_id_ = 1;
  std::vector<TrackedClusterState> tracked_object_groups_;
  int next_object_group_id_ = 1;
  double prev_background_z_ = 0.0;
  bool prev_background_z_valid_ = false;

  // Top-Down Hook Chain & Beam tracking state
  struct TrackedHookChain {
    int chain_id = -1;
    std::string side = "UNKNOWN";
    CraneHookConnectionState state = CraneHookConnectionState::CANDIDATE;
    int hit_count = 1;
    int lost_frames = 0;
    std::deque<bool> history;
    LinearKalmanFilter kf_xy;
    LinearKalmanFilter kf_z;
    double last_x = 0.0, last_y = 0.0, last_z = 0.0;
    CraneHookChain latest_chain;
  };
  std::vector<TrackedHookChain> tracked_hook_chains_;
  int next_hook_chain_id_ = 1;
  double last_hook_stamp_sec_ = 0.0;
  bool has_last_hook_stamp_ = false;
  LinearKalmanFilter beam_xy_filter_;
  LinearKalmanFilter beam_z_filter_;
  bool beam_initialized_ = false;

  // Root motion tracking state
  bool crane_moving_ = false;
  std::string crane_move_reason_ = "STOPPED";
  bool root_anchor_valid_ = false;
  double root_anchor_x_ = 0.0, root_anchor_y_ = 0.0;
  int root_stable_frames_ = 0;
  std::deque<std::pair<double, double>> root_position_history_;

  // Top Root Kalman smoothing state (for high-Z noise rejection)
  struct TrackedRootNode {
    bool initialized = false;
    double smoothed_x = 0.0, smoothed_y = 0.0, smoothed_z = 0.0;
    LinearKalmanFilter kf_xy;
    LinearKalmanFilter kf_z;
    int lost_frames = 0;
    bool anchor_valid = false;
    double anchor_x = 0.0, anchor_y = 0.0;
  };
  TrackedRootNode left_root_node_;
  TrackedRootNode right_root_node_;

  // Fixed Root ROI and Geometric Center Reference State
  CraneFixedRootState left_fixed_root_state_;
  CraneFixedRootState right_fixed_root_state_;

  // Work Status tracking state
  struct TrackedHookWorkState {
    std::deque<bool> history;
    std::deque<std::array<double,8>> object_observations; // stamp, local PCA L/W/H/yaw, measured XYZ
    // Current direct samples only: stamp, Hook/Cargo/Housing Z, Hook XY, Cargo XY.
    // Evidence window, not a second object tracker or a new FSM state.
    std::deque<std::array<double,8>> start_observations;
    // One previous frame of geometry evidence; Cargo KF/identity remains the only tracker.
    std::vector<std::array<double,8>> extension_observations; // stamp,world XYZ,L,W,yaw,hits
    bool current_status = false;
    int consecutive_misses = 0;
  };
  TrackedHookWorkState left_work_state_;
  TrackedHookWorkState right_work_state_;
  // The existing Work Status owns the new cycle; no separate detector or node.
  CraneWorkStatusResult work_cycle_;
  LinearKalmanFilter cargo_xy_filter_,cargo_z_filter_;
  std::vector<std::array<float,3>> cargo_owned_points_;
  std::vector<int> cargo_recovery_indices_;
  double cargo_owned_stamp_=-1,cargo_recovery_stamp_=-1;
  int cargo_owned_id_=-1;
  void rememberCargoRaw(const std::vector<PointXYZIRT>& raw,const std::vector<int>& ids,double stamp);
  std::vector<int> currentOwnedCargoRaw(const std::vector<PointXYZIRT>& raw,double stamp) const;

  std::array<LinearKalmanFilter,8> boundary_filters_{};
  std::array<CraneBoundaryObservation,8> boundary_observations_{};
  std::array<double,8> boundary_last_seen_{};
  std::array<int,8> boundary_hits_{};
  double work_last_stamp_=0,cargo_last_seen_=0,cargo_observed_sec_=0;
  double cargo_last_x_=0,cargo_last_y_=0,cargo_last_z_=0;
  double confirmation_length_=0,confirmation_width_=0;
  int cargo_hits_=0,cargo_outside_hits_=0,next_cargo_id_=1,boundary_stable_frames_=0;
  CraneBoundaryMotionConfirmation boundary_confirmation_;
  bool boundary_moving_=false;
  CraneZGaussianResult cargo_extension_layers_;
  double drop_started_stamp_=0,cargo_peak_z_=0,gap_last_stamp_=0,gap_observed_sec_=0;
  int cargo_descent_steps_=0,gap_hits_=0;
  bool cargo_descent_seen_=false,cargo_stationary_current_=false;
  double work_elapsed_current_=0,completion_last_stamp_=0;
  // Sensor-time intervals: start, end, evidence (-1 UNKNOWN, 0 FALSE, 1 TRUE).
  std::deque<std::array<double,3>> drop_observations_;
  double drop_window_first_stamp_=-1,drop_window_last_stamp_=-1;
  bool drop_candidate_=false;
  int drop_last_evidence_=-1;
  double drop_window_valid_sec_=0;
  int boundary_move_count_=0,root_move_count_=0;
  std::vector<std::vector<size_t>> boundary_moving_points_;
  std::array<int,2> root_move_ids_{{-1,-1}};
  // Independent downstream Z evidence only: UNKNOWN=0, STATIONARY=1, RISING=2, FALLING=3.
  // Each sample stores sensor timestamp, directly observed Housing Z, evidence.
  std::array<std::deque<std::array<double,3>>,2> sheave_motion_samples_;
  std::array<int,2> sheave_motion_state_{{0,0}},sheave_motion_id_{{-1,-1}};
  std::array<double,2> sheave_stationary_anchor_z_{{NAN,NAN}};
  std::array<double,2> sheave_motion_first_stamp_{{-1,-1}},sheave_motion_last_stamp_{{-1,-1}};
  std::array<double,2> sheave_motion_last_direct_{{-1,-1}};
  std::string sheave_motion_frame_;
  void resetSheaveMotion(int side);
  void updateSheaveMotion(const CraneTopDownHookResult & result,double stamp);
  int lifting_cargo_id_=-1;
  double lifting_last_stamp_=-1,lifting_event_until_=0,clearance_reference_=0;
  std::deque<std::array<double,2>> clearance_observations_;
  void updatePublicWorkStatus(double stamp,const CraneBeamConfig & beam_config);
  bool updateDropCandidate(double stamp, int evidence);
  void resetDropCandidate();
  void updateCargoCompletion(double stamp);
  // One ordered work FSM; legacy candidate phases remain measurement confirmation only.
  CraneBeamState ordered_state_=CraneBeamState::IDLE;
  int ordered_side_=-1,ordered_sheave_id_=-1;
  std::array<int,2> ordered_roi_ids_{{-1,-1}};
  double ordered_stamp_=-1,ordered_last_roi_=-1,ordered_completed_=-1;
  double ordered_start_z_=0,ordered_peak_z_=0,ordered_previous_z_=0;
  int ordered_stop_count_=0,ordered_lower_count_=0;
  double ordered_lower_accum_=0,ordered_last_seed_stamp_=-1,ordered_last_seed_z_=0;
  std::array<std::deque<std::array<double,4>>,2> ordered_roi_samples_; // stamp,q10,q50,q90 (one ROI, not 3 votes)
  void observeOrderedHookRoi(const std::vector<PointXYZIRT> & raw,double stamp);
  void projectOrderedState();
  void resetOrderedWork(const char * reason);

  void emitFieldDrop(double stamp,const char * reason);
  // Independent sensor-time confirmation for measured contact / detachment.
  std::array<double,2> field_drop_elapsed_{{0,0}},field_drop_last_{{-1,-1}};
  int field_policy_cargo_id_=-1;
  std::deque<std::array<double,2>> field_cargo_rise_; // direct measured seed Z only

  std::string last_gap_candidate_="UNKNOWN";
  void updateWorkStatus(const std::vector<PointXYZIRT> & current_points,
      const std::vector<CraneSubcluster> & subclusters,double stamp_sec);


  // Pre-Z-Gaussian Temporal Accumulation & Stabilization private state
  struct StationaryFrame {
    std::vector<PointXYZIRT> roi_points;
    double stamp_sec = 0.0;
  };
  std::deque<StationaryFrame> stationary_frames_;
  std::deque<StationaryFrame> batch_frames_;
  std::vector<PointXYZIRT> accumulated_points_;
  std::vector<int> accumulated_indices_;
  bool has_accumulated_points_ = false;
  StabilizedBeamReference stabilized_beam_;
  bool external_crane_moving_ = false;
  bool prev_hook_valid_ = false;
  double prev_hook_x_ = 0.0;
  double prev_hook_y_ = 0.0;
  double prev_hook_z_ = 0.0;

public:
  // Top-Down Hook & Largest Connected Cluster Temporal Stabilization
  struct TrackedChainNodeState {
    int layer_id = -1;
    double x = 0.0, y = 0.0, z = 0.0;
    double min_x = 0.0, max_x = 0.0;
    double min_y = 0.0, max_y = 0.0;
    double min_z = 0.0, max_z = 0.0;
    size_t point_count = 0;
    std::vector<int> point_indices;
    int subcluster_id = -1;
    int parent_subcluster_id = -1;
    double gap_z_to_parent = 0.0;
    double dxy_to_parent = 0.0;
    double slope_angle_deg = 0.0;
    double turn_angle_deg = 0.0;
    bool is_estimated_gap = false;
    int confirmed_hits = 0;
    int missed_frames = 0;
    bool is_held = false;
    bool is_connection_stable = false;
    bool is_spatial_promoted = false;
    int spatial_hold_count = 0;
    std::deque<uint8_t> edge_type_history; // 1 = DIRECT, 2 = ESTIMATED, 0 = MISSING
    int consecutive_missing = 0;

    // Layer-wise Dual-ROI Anchoring & Motion Monitoring state
    int anchor_id = -1;
    bool anchor_locked = false;
    bool initial_anchor_valid = false;
    double initial_anchor_x = 0.0, initial_anchor_y = 0.0, initial_anchor_z = 0.0;
    double anchor_x = 0.0, anchor_y = 0.0, anchor_z = 0.0;
    double anchor_length = 0.0, anchor_width = 0.0, anchor_height = 0.0;
    double moving_x = 0.0, moving_y = 0.0, moving_z = 0.0;
    double raw_obs_x = 0.0, raw_obs_y = 0.0, raw_obs_z = 0.0;
    double disp_from_anchor_xy = 0.0, disp_from_anchor_z = 0.0;
    double prev_move_dx = 0.0, prev_move_dy = 0.0, prev_move_dz = 0.0;
    int relocate_confirm_count = 0;
    int relocate_count = 0;
    bool is_relocated_this_frame = false;
    bool is_beam_intruded = false;
    int intrusion_missed_frames = 0;
    int memory_age_frames = 0;
    CraneFixedRootRoi fixed_roi;
    CraneFixedRootRoi moving_roi;
    std::deque<double> z_history;
    WireBundleGeometry wire_bundle;
    WireBundleGeometry remembered_bundle;
    int wire_bundle_memory_age = 0;
    bool is_sheave = false;
    CraneSheaveGeometry sheave;
  };

  static WireBundleGeometry resolveWireBundleGeometry(
      const CraneHookChainNode & rn,
      const std::vector<CraneSubcluster> & subclusters,
      const std::vector<PointXYZIRT> & points,
      const TrackedChainNodeState * prior_sn,
      const CraneVirtualRopeConfig & vr_cfg,
      const CraneDualRoiAnchorConfig & dual_cfg);

  static bool appendSheaveTerminal(CraneHookChain & output, const CraneHookChainNode & terminal);

  static bool matchHousingReference(CraneSheaveGeometry &, const CraneSheaveConfig &, const HousingGeometryMemory * = nullptr);

  static CraneSheaveGeometry detectSheaveCandidate(
      const CraneHookChainNode & lowest_wire_node,
      const std::vector<CraneSubcluster> & subclusters,
      const std::vector<PointXYZIRT> & points,
      const WireBundleGeometry & wire_bundle,
      const CraneSheaveConfig & sheave_cfg,
      const CraneDualRoiAnchorConfig & dual_cfg,
      const CraneSheaveGeometry * prior_sheave = nullptr,
      const HousingGeometryMemory * housing_mem = nullptr,
      const std::vector<PointXYZIRT> * current_observation_points = nullptr,
      const tf2::Vector3 * sensor_origin = nullptr);

private:
  friend struct CranePipelineRegressionAccess;
  void updateSheaveTracking(CraneTopDownHookResult &,
      const std::vector<CraneSubcluster> &, const std::vector<PointXYZIRT> &, double stamp_sec,
      const std::vector<PointXYZIRT> * current_observation_points);

  struct TrackedTopDownStructure {
    bool initialized = false;
    std::string side = "UNKNOWN";
    std::vector<TrackedChainNodeState> stable_nodes;
    std::vector<TrackedChainNodeState> pending_nodes;
    std::vector<TrackedChainNodeState> remembered_anchors;
    int next_anchor_id = 1;
    int pending_hits = 0;

    int next_housing_track_id = 1;
    bool sheave_valid = false;
    TrackedChainNodeState sheave_node;
    CraneSheaveGeometry stable_sheave;
    HousingGeometryMemory housing_memory;
    int sheave_missed_frames = 0;

    bool largest_valid = false;
    CraneHookNearestCluster stable_largest;
    int largest_confirmed_frames = 0;
    int largest_missed_frames = 0;
    int consecutive_jump_frames = 0;
    int consecutive_yaw_jump_frames = 0;
    int largest_swap_candidate_hits = 0;
    int largest_swap_candidate_sc_id = -1;

    double prev_root_x = 0.0;
    double prev_root_y = 0.0;
    double prev_root_z = 0.0;
    bool prev_root_valid = false;

    // 1st Exception Gate temporal tracking
    CraneExceptionGateStatus prev_gate_status = CraneExceptionGateStatus::NONE;
    int consecutive_drop_frames = 0;
  };

  TrackedTopDownStructure left_td_track_;
  TrackedTopDownStructure right_td_track_;
  VisibilityModel * visibility_model_ptr_ = nullptr;
  VisibilityModel * cargo_visibility_model_ptr_ = nullptr;

  void stabilizeTopDownStructure(
      CraneTopDownHookResult & td_res,
      const std::vector<CraneSubcluster> & subclusters,
      const std::vector<PointXYZIRT> & points,
      bool crane_moving,
      double root_dx,
      double root_dy);

  void evaluateExceptionGate(
      CraneTopDownHookResult & td_res,
      const std::vector<PointXYZIRT> & points,
      VisibilityModel * visibility);
};
}  // namespace cx128s2_bev_perception

