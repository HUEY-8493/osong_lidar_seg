#pragma once

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>
#include <algorithm>
#include <memory>

#include <cx128s2_bev_perception/point_types.hpp>
#include <cx128s2_bev_perception/crane_hook_detector.hpp>
#include <cx128s2_bev_perception/vehicle_surface_tracking.hpp>

namespace cx128s2_bev_perception {

// Internal FSM remains numeric; every public Crane work-status output uses this contract.
inline const char * workingStatusName(int value) {
  return value == 3 ? "lifting" : (value == 2 ? "drop" : (value == 1 ? "working" : "idle"));
}


enum class CraneBeamState {
  IDLE = 0,
  LIFTING = 1,
  SUSPENDED = 2,
  LOWERING = 3,
  COMPLETE_CONFIRM = 4,
  SEARCH_WIRE, TRACK_SHEAVE, WAIT_CARGO, READY, CONFIRM_CARGO, LOCKED_TRACKING, COMPLETED, LOWERING_CONFIRMED
};

inline const char * to_string(CraneBeamState state) {
  switch (state) {
    case CraneBeamState::IDLE: return "IDLE";
    case CraneBeamState::LOWERING_CONFIRMED: return "LOWERING_CONFIRMED";
    case CraneBeamState::SEARCH_WIRE: return "SEARCH_WIRE";
    case CraneBeamState::TRACK_SHEAVE: return "TRACK_SHEAVE";
    case CraneBeamState::WAIT_CARGO: return "WAIT_CARGO";
    case CraneBeamState::READY: return "READY";
    case CraneBeamState::CONFIRM_CARGO: return "CONFIRM_CARGO";
    case CraneBeamState::LOCKED_TRACKING: return "LOCKED_TRACKING";
    case CraneBeamState::COMPLETED: return "COMPLETED";

    case CraneBeamState::LIFTING: return "LIFTING";
    case CraneBeamState::SUSPENDED: return "SUSPENDED";
    case CraneBeamState::LOWERING: return "LOWERING";
    case CraneBeamState::COMPLETE_CONFIRM: return "COMPLETE_CONFIRM";
    default: return "UNKNOWN";
  }
}

enum class BeamMeasurementSource {
  INITIAL_ROI = 0,
  TRACKED_CLUSTER = 1
};

enum class AcquisitionResetReason : uint8_t {
  NONE = 0,
  NO_SEED_CONNECTION = 1,
};

inline const char * to_string(BeamMeasurementSource src) {
  switch (src) {
    case BeamMeasurementSource::INITIAL_ROI: return "INITIAL_ROI";
    case BeamMeasurementSource::TRACKED_CLUSTER: return "TRACKED_CLUSTER";
    default: return "UNKNOWN";
  }
}

struct VerticalMotionEvidence {
  double positive_accum = 0.0;
  double negative_accum = 0.0;
  int positive_frames = 0;
  int negative_frames = 0;

  void reset() {
    positive_accum = 0.0;
    negative_accum = 0.0;
    positive_frames = 0;
    negative_frames = 0;
  }

  void update(double dz, double eps = 0.003) {
    if (dz > eps) {
      positive_accum += dz;
      negative_accum = std::max(0.0, negative_accum - dz * 2.0);
      positive_frames++;
      negative_frames = 0;
    } else if (dz < -eps) {
      negative_accum += (-dz);
      positive_accum = std::max(0.0, positive_accum + dz * 2.0);
      negative_frames++;
      positive_frames = 0;
    } else {
      positive_accum *= 0.95;
      negative_accum *= 0.95;
    }
  }
};

struct CraneBeamConfig {
  bool enabled = true;
  std::string target_frame = "world";

  // Motion reference mode: "auto", "absolute", "relative_background"
  std::string motion_reference_mode = "auto";

  // Beam ROI (Initial Seed ROI) in target_frame - used for initial acquisition only
  double beam_roi_x_min = -2.0;
  double beam_roi_x_max = 2.0;
  double beam_roi_y_min = -2.0;
  double beam_roi_y_max = 2.0;
  double beam_roi_z_min = -0.5;
  double beam_roi_z_max = 7.5;

  // Background ROI in target_frame (Beam ROI points are excluded)
  double bg_roi_x_min = -8.0;
  double bg_roi_x_max = 8.0;
  double bg_roi_y_min = -8.0;
  double bg_roi_y_max = 8.0;
  double bg_roi_z_min = -0.5;
  double bg_roi_z_max = 7.5;

  // Point count requirements
  size_t min_beam_roi_points = 10;
  size_t min_bg_roi_points = 20;
  size_t min_beam_cluster_points = 15;

  // Z-band around z_beam for candidate extraction
  double beam_z_band = 0.30;

  // 1D Z Histogram parameters for Initial Acquisition
  double beam_z_histogram_bin_size = 0.05; // [m] bin resolution
  double beam_ground_clearance = 0.12;     // [m] min clearance above ground to search for beam in IDLE

  // Tracking gates (multi-attribute continuity)
  double tracking_gate_xy = 0.80;     // [m] max centroid XY shift per frame
  double tracking_gate_z = 0.50;      // [m] max vertical Z shift per frame
  double tracking_gate_yaw = 0.35;    // [rad] max yaw rotation (~20 deg)
  double tracking_gate_length = 1.50; // [m] max length change

  // Local Support Surface estimation
  double support_search_margin_xy = 0.60; // [m] footprint expansion for support search
  size_t min_support_points = 15;         // min points required for valid support
  double placement_tolerance = 0.08;      // [m] max gap between beam bottom and support surface
  double beam_default_height = 0.50;      // [m] nominal beam vertical thickness

  // Maximum Beam-ROI-to-Background-ROI relative height
  // accepted as near-placement.
  // Beam Z represents the cluster center, so a positive offset
  // remains even when the beam is physically resting on the surface.
  double landing_height_gap_threshold = 0.45;

  // FSM thresholds
  double lift_dz_threshold = 0.05;
  double lower_dz_threshold = 0.05;
  double stop_dz_threshold = 0.02;
  double hook_beam_dz_match_tolerance = 0.03; // [m/frame], same reference as motion_dz
  double ground_tolerance = 0.05;
  double crane_move_start_x = 0.10, crane_move_start_y = 0.10;
  double crane_move_stop_x = 0.02, crane_move_stop_y = 0.05;
  int crane_move_stop_frames = 5;

  // Confirmation frame counts
  int lift_confirm_frames = 3;
  int suspend_confirm_frames = 3;
  int lower_confirm_frames = 3;
  int placed_confirm_frames = 3;
  int complete_confirm_frames = 5;

  // Measurement dropout allowance
  int beam_measurement_max_lost_frames = 3;

  // Length estimation & lock
  double length_stable_threshold = 0.10;
  int length_lock_frames = 5;
  double cluster_bev_res = 0.05;
  double continuity_gate_xy = 2.5;  // Deprecated fallback radius

  // Exponential Moving Average (EMA) for Z
  bool use_z_ema = true;
  double z_ema_alpha = 0.3;

  // Cumulative vertical motion evidence (Slow lift / Slow lower dead-zone mitigation)
  bool cumulative_motion_enabled = true;
  double cumulative_lift_distance = 0.12;     // [m] cumulative upward distance required
  double cumulative_lower_distance = 0.12;    // [m] cumulative downward distance required
  int cumulative_motion_min_frames = 4;       // min consecutive directional frames
  double cumulative_direction_epsilon = 0.003;// [m/frame] directional motion noise threshold (0.03 m/s)

  // Local Support Surface planar & extent filtering
  double support_min_xy_extent = 0.60;        // [m] min horizontal span max(dx, dy) for planar support patch
  double support_max_z_spread = 0.08;         // [m] max vertical MAD within support patch
  bool support_temporal_continuity = true;    // prefer continuous support level if available

  // Tilted Touchdown multi-segment evaluation
  bool tilt_segmentation_enabled = true;
  int tilt_num_segments = 3;                  // longitudinal segments (Tip A, Center, Tip B)
  double min_supported_segment_ratio = 0.60;  // required fraction of segments supported for placed_candidate

  // Initial Cluster Shape Gating (Initial Acquisition rejection of humans/poles)
  double min_initial_cluster_length = 2.0;    // [m] min PCA length for initial beam cluster
  double min_initial_cluster_elongation = 1.8;// min length / width ratio for initial beam cluster

  // Initial-acquisition high-Z anomaly gate.  This is only a false-positive
  // guard; it is not a support/placement measurement.
  bool beam_high_z_gate_enabled = true;
  double beam_high_z_reject_threshold = 1.0; // [m] p90(Beam ROI Z) - background median
  bool beam_high_z_require_cluster_valid = true;

  // Sensor transform freeze (disables dynamic Gazebo TF compensation)
  // When enabled, the sensor-to-target transform is locked at startup so crane motion is observed
  // directly in the point cloud via relative background dz rather than cheated by Gazebo moving TF.
  bool freeze_sensor_transform = true;

  // Debug options
  bool publish_debug = true;
  bool publish_background_roi_marker = false; // Yellow Background ROI box (false by default to keep view clean)
  double diagnostics_period_sec = 1.0;

  // Crane Hook Detector Configuration
  CraneHookConfig hook;
};

struct CraneBeamDetectionResult {
  CraneBeamState state = CraneBeamState::IDLE;
  int working_status = 0;  // 0 = not working, 1 = working
  std::string motion_reference_mode = "absolute";

  // Validity flags
  bool beam_z_valid = false;
  bool bg_z_valid = false;
  bool beam_cluster_valid = false;
  bool beam_high_z_rejected = false;
  bool seed_roi_connected = false;
  AcquisitionResetReason acquisition_reset_reason = AcquisitionResetReason::NONE;

  // Z measurements
  double z_beam_raw = 0.0;
  double z_beam_filtered = 0.0;
  double z_bg_raw = 0.0;
  double z_bg_filtered = 0.0;
  double height_gap_raw = 0.0;
  double height_gap_filtered = 0.0;
  double beam_high_z_indicator = 0.0;

  // Velocity / dz measurements
  double dz_beam_raw = 0.0;
  double dz_beam_filtered = 0.0;
  double dz_bg_raw = 0.0;
  double dz_bg_filtered = 0.0;
  double relative_dz_raw = 0.0;
  double relative_dz_filtered = 0.0;
  double motion_dz = 0.0;
  double beam_vertical_velocity = 0.0;
  double dt_sec = 0.1;
  double v_beam_raw = 0.0;
  double v_beam_filtered = 0.0;
  double v_bg_raw = 0.0;
  double v_bg_filtered = 0.0;

  // Statistical dispersion (MAD)
  double beam_mad = 0.0;
  double bg_mad = 0.0;

  // Quartiles (min, Q1, median, Q3, max)
  double z_beam_min = 0.0;
  double z_beam_q1 = 0.0;
  double z_beam_q3 = 0.0;
  double z_beam_max = 0.0;
  double z_bg_min = 0.0;
  double z_bg_q1 = 0.0;
  double z_bg_q3 = 0.0;
  double z_bg_max = 0.0;

  // Point counts
  size_t beam_roi_point_count = 0;
  size_t bg_roi_point_count = 0;
  size_t beam_cluster_point_count = 0;

  // Cluster geometry & length
  double cluster_centroid_x = 0.0;
  double cluster_centroid_y = 0.0;
  double beam_length_raw = 0.0;
  double initial_cluster_length = 0.0;
  double initial_cluster_elongation = 0.0;
  double beam_length_locked = 0.0;
  bool length_locked = false;
  double beam_major_axis_yaw = 0.0;

  // FSM and status flags
  bool crane_moving = false;
  bool crane_move_valid = false;
  int crane_move_source_mask = 0, crane_move_measured_frames = 0;
  double crane_move_anchor_x = 0.0, crane_move_anchor_y = 0.0;
  double crane_move_hook_x = 0.0, crane_move_hook_y = 0.0;
  double crane_move_net_dx = 0.0, crane_move_net_dy = 0.0;
  double crane_move_x_span = 0.0, crane_move_y_span = 0.0;
  std::string crane_move_reason = "WAIT_CONFIRMED_HOOK";
  bool active_left = false;
  bool active_right = false;
  std::string beam_tracking_state = "UNINITIALIZED";
  bool placed_candidate = false;
  bool work_complete = false;

  // Measurement source: INITIAL_ROI or TRACKED_CLUSTER
  BeamMeasurementSource measurement_source = BeamMeasurementSource::INITIAL_ROI;

  // Local Support Surface & Placement
  bool support_valid = false;
  double support_z = 0.0;
  size_t support_point_count = 0;
  double placement_gap_z = 0.0;

  // Contact & Tilted Touchdown
  bool contact_candidate = false;
  float supported_segment_ratio = 0.0f;

  // Cumulative motion evidence
  float motion_cumulative_dz = 0.0f;

  // Tracking candidate metrics
  size_t tracking_candidate_count = 0;
  double selected_tracking_score = 0.0;

  // Diagnostic counters
  int lift_counter = 0;
  int stop_counter = 0;
  int lower_counter = 0;
  int placed_counter = 0;
  int complete_counter = 0;
  int length_lock_counter = 0;
  int lost_frames_counter = 0;

  // Crane Hook measurements and auxiliary prepare_ready flag (not an FSM state).
  CraneHookResult hook;

  // State transition notification
  bool state_changed = false;
  std::string prev_state_str;
  std::string curr_state_str;
};

class CraneBeamDetector {
public:
  explicit CraneBeamDetector(const CraneBeamConfig & config = CraneBeamConfig{});

  void updateConfig(const CraneBeamConfig & config);
  const CraneBeamConfig & getConfig() const { return config_; }

  // Process single frame
  const CraneBeamDetectionResult & process(
      const std::vector<PointXYZIRT> & points,
      const std::vector<int> & valid_indices,
      double stamp_sec);

  // Debug indices getters for PointCloud2 publishers
  const std::vector<int> & getBeamRoiIndices() const { return beam_roi_indices_; }
  const std::vector<int> & getBgRoiIndices() const { return bg_roi_indices_; }
  const std::vector<int> & getZCandidateIndices() const { return z_candidate_indices_; }
  const std::vector<int> & getBeamClusterIndices() const { return beam_cluster_indices_; }
  const std::vector<int> & getSupportIndices() const { return support_indices_; }
  const std::vector<int> & getHookLeftBodyIndices() const { return hook_detector_.getLeftBodyIndices(); }
  const std::vector<int> & getHookRightBodyIndices() const { return hook_detector_.getRightBodyIndices(); }
  const CraneHookDetector & getHookDetector() const { return hook_detector_; }
  CraneHookDetector & getHookDetector() { return hook_detector_; }

  // Reset detector state
  void reset();

  // Helper for median and MAD calculation
  static bool computeMedianAndMAD(std::vector<float> & vals, float & median, float & mad);

  // Helper for full quantiles (min, Q1, median, Q3, max, MAD)
  static bool computeQuantiles(
      std::vector<float> & vals,
      float & min_v, float & q1, float & median, float & q3, float & max_v, float & mad);

private:
  CraneBeamConfig config_;
  CraneBeamDetectionResult result_;
  CraneHookDetector hook_detector_;

  // Preallocated buffer indices
  std::vector<int> beam_roi_indices_;
  std::vector<int> bg_roi_indices_;
  std::vector<int> z_candidate_indices_;
  std::vector<int> beam_cluster_indices_;
  std::vector<int> support_indices_;
  std::vector<float> z_beam_vals_;
  std::vector<float> z_bg_vals_;

  // Continuity tracking state
  bool has_previous_cluster_ = false;
  LinearKalmanFilter beam_xy_filter_, beam_z_filter_;
  std::vector<PointXYZIRT> beam_local_geometry_;
  double locked_half_width_ = 0.0;
  double beam_lock_z_ = 0.0, beam_lock_bg_z_ = 0.0;
  int crane_move_direction_ = 0;
  std::array<bool, 2> crane_move_anchor_valid_{};
  std::array<std::array<double, 2>, 2> crane_move_anchors_{};
  std::array<std::vector<std::array<double, 2>>, 2> crane_move_hook_history_;
  std::vector<std::array<double, 2>> crane_move_history_;
  int crane_move_previous_mask_ = 0;
  std::array<int, 2> crane_move_position_sources_{}; // 1: body KF, 2: current ROI maximum
  bool cycle_prepare_ready_ = false;
  std::array<bool, 2> cycle_prepare_sides_{};
  std::array<double, 2> previous_hook_z_{};
  std::array<bool, 2> previous_hook_measured_{};
  std::array<double, 2> prepared_hook_z_{}, prepared_hook_bg_z_{};
  std::array<bool, 2> prepared_hook_valid_{};
  std::array<int, 2> hook_lift_counters_{};
  std::array<VerticalMotionEvidence, 2> hook_motion_evidence_;
  double prev_cluster_centroid_x_ = 0.0;
  double prev_cluster_centroid_y_ = 0.0;
  double prev_cluster_yaw_ = 0.0;
  double prev_cluster_length_ = 0.0;
  double prev_cluster_pitch_slope_ = 0.0;

  // Previous frame tracking for dz / EMA
  bool has_prev_stamp_ = false;
  double prev_stamp_sec_ = 0.0;
  bool has_prev_beam_z_ = false;
  double prev_beam_z_raw_ = 0.0;
  double prev_beam_z_filtered_ = 0.0;
  bool has_prev_bg_z_ = false;
  double prev_bg_z_raw_ = 0.0;
  double prev_bg_z_filtered_ = 0.0;

  double prev_beam_length_ = 0.0;

  // Latched work complete, locked length, and placement support
  bool latched_work_complete_ = false;
  double latched_locked_length_ = 0.0;
  bool latched_length_locked_ = false;
  bool latched_support_valid_ = false;
  double latched_support_z_ = 0.0;
  double latched_placement_gap_z_ = 0.0;
  float latched_supported_segment_ratio_ = 0.0f;
  bool latched_contact_candidate_ = false;

  // Vertical motion evidence accumulator (Slow motion dead-zone mitigation)
  VerticalMotionEvidence vertical_motion_evidence_;

  // Support surface temporal continuity tracking
  bool has_prev_support_z_ = false;
  double prev_support_z_ = 0.0;

  // Helper methods
  bool insideBeamRoi(double x, double y, double z) const;
  bool insideBgRoi(double x, double y, double z) const;

  // 1D Z-histogram mode extraction for initial acquisition
  bool extractAcquisitionBeamZ(
      const std::vector<PointXYZIRT> & points,
      const std::vector<int> & roi_indices,
      double ground_z,
      double & beam_z,
      float & mad);

  void extractBeamCluster(
      const std::vector<PointXYZIRT> & points,
      double z_beam,
      bool use_continuity_seed);

  // Multi-attribute continuity tracking for moving beam
  bool extractTrackedBeamCluster(
      const std::vector<PointXYZIRT> & points,
      const std::vector<int> & valid_indices);

  // Local support surface estimation beneath/around beam footprint
  void estimateLocalSupportSurface(
      const std::vector<PointXYZIRT> & points,
      const std::vector<int> & valid_indices);

  friend class CranePipeline; // Reuse the existing PCA without changing Beam detection.
  void computeClusterPCA(
      const std::vector<PointXYZIRT> & points,
      const std::vector<int> & cluster_idx,
      double & length,
      double & yaw,
      double & centroid_x,
      double & centroid_y) const;
};

}  // namespace cx128s2_bev_perception
