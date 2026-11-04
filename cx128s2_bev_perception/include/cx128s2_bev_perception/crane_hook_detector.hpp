#pragma once

#include <cmath>
#include <cstddef>
#include <vector>
#include <algorithm>
#include <limits>
#include <array>
#include <string>

#include <cx128s2_bev_perception/point_types.hpp>
#include <cx128s2_bev_perception/linear_kalman_filter.hpp>

namespace cx128s2_bev_perception {

struct HookDetection {
  bool detected = false;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  size_t body_points = 0;
  double body_density = 0.0;
  double score = 0.0;
  double window_z_min = 0.0;
  double window_z_max = 0.0;

  // Current measured body band versus immutable geometry of this confirmed track.
  std::array<double, 3> body_min{}, body_max{};
  double body_width = 0.0, body_length = 0.0;
  double body_surface_ratio = 0.0; // minor/major covariance in the XY-major / Z plane
  size_t occupied_cells = 0;
  bool reference_valid = false;
  std::array<double, 3> reference_min{}, reference_max{}; // offsets from measured center
  double reference_width = 0.0, reference_length = 0.0;
  double reference_surface_ratio = 0.0;
  bool partial_observation = false;

  // Last accepted body identity survives prediction-only frames.
  double last_measurement_z = 0.0;
  double last_window_z_min = 0.0, last_window_z_max = 0.0;
  size_t valid_window_count = 0, candidate_count = 0;
  double prediction_error_z = 0.0, previous_error_z = 0.0;
  double selection_prediction_z = 0.0, selection_previous_z = 0.0;
  double selection_gate_z = 0.0;
  double previous_window_overlap = 0.0;
  bool candidate_switched = false;
  std::string selection_reason = "NO_VALID_WINDOW";
  std::string candidate_details;

  // detected and x/y/z describe this frame's actual body measurement only.
  bool initialized = false;
  std::string tracking_state = "UNINITIALIZED";
  int confirm_hits = 0;
  int lost_frames = 0;
  double predicted_x = 0.0, predicted_y = 0.0, predicted_z = 0.0;
  double filtered_x = 0.0, filtered_y = 0.0, filtered_z = 0.0;
  double vx = 0.0, vy = 0.0, vz = 0.0;
  bool predicted_ground_distance_valid = false;
  double predicted_ground_distance = 0.0;

  // Continuous position output, independent of actual body observations and FSM evidence.
  std::string position_tracking_state = "LOST";
  bool position_valid = false; // last position exists, including an empty-ROI hold
  bool position_updated = false;
  double tracking_x = 0.0, tracking_y = 0.0, tracking_z = 0.0;
  bool fallback_point_valid = false; // current frame has a finite ROI maximum
  double fallback_x = 0.0, fallback_y = 0.0, fallback_z = 0.0;

  // Ground reference beneath this Hook
  bool ground_valid = false;
  double ground_z = 0.0;
  size_t ground_points = 0;

  // Current measured body bottom minus valid local ground Z [m].
  double ground_distance = 0.0;

  // Comparison only: current measured Hook Z minus the existing filtered Beam Z.
  // Filled by CraneBeamDetector after Beam validation; never used by tracking/readiness.
  bool beam_distance_valid = false;
  double beam_reference_z = 0.0;
  double beam_dz = 0.0;

  std::vector<int> body_indices;
  std::vector<int> roi_indices;
};

struct CraneHookConfig {
  bool enabled = true;

  // ROI bounds in target_frame (typically "world", calibrated ground at Z=0)
  double roi_left_x_min = -1.5;
  double roi_left_x_max = -0.3;
  double roi_right_x_min = 0.3;
  double roi_right_x_max = 1.5;
  double roi_y_min = -0.8;
  double roi_y_max = 0.8;
  double roi_z_min = 3.0; // minimum ground-relative body height for acquisition only
  double roi_z_max = 6.2;

  // Hook Body vertical sliding window parameters
  double window_size_z = 0.30; // [m] 30cm body span
  double window_step_z = 0.02; // [m] 2cm sliding step
  size_t min_body_points = 300; // minimum required body points

  // Local gates never move or enlarge the fixed physical ROI.
  bool tracking_enabled = true;
  int tracking_confirm_frames = 3;
  double tracking_gate_xy = 0.35; // per-axis sway/residual allowance [m]
  double tracking_gate_z = 0.45; // vertical residual allowance [m]
  double tracking_gate_z_max = 0.80; // cap for velocity/dt allowance [m]
  int tracking_max_lost_frames = 5;
  double tracking_prediction_timeout_sec = 0.75;
  double tracking_accel_std_xy = 0.5;
  double tracking_accel_std_z = 1.5;
  double tracking_measurement_std = 0.03;

  // Ground reference estimation parameters
  double ground_search_z_min = -0.5;
  double ground_search_z_max = 0.8;
  double ground_bin_size = 0.05; // 5cm bin
  size_t min_ground_points = 50;

  // Compact multi-frame reference; collection is enabled only by an explicit command.
  int reference_min_frames = 30;

  // Per-side readiness thresholds consumed by IDLE preparation; never an FSM state.
  double prepare_distance = 4.0; // [m] Confirmed descending body bottom above local ground
  double prepare_release_margin = 0.20; // [m] Ground-relative release hysteresis
  int prepare_confirm_frames = 1; // [frames] Confirmation after measured descent
};

struct CraneHookResult {
  HookDetection left;
  HookDetection right;

  // Distance metrics
  double mean_ground_distance = 0.0;
  double max_ground_distance = 0.0;
  double min_ground_distance = 0.0;

  // Auxiliary readiness flag & chattering prevention; never a Crane FSM state.
  bool prepare_ready = false;
  std::array<bool, 2> prepare_sides{};
  std::array<int, 2> prepare_side_counters{}, release_side_counters{};
  int prepare_counter = 0;
  int release_counter = 0;
};

class CraneHookDetector {
public:
  explicit CraneHookDetector(const CraneHookConfig & config = CraneHookConfig{});

  void updateConfig(const CraneHookConfig & config);
  const CraneHookConfig & getConfig() const { return config_; }

  const CraneHookResult & process(
      const std::vector<PointXYZIRT> & points,
      const std::vector<int> & valid_indices, double stamp,
      double beam_z = std::numeric_limits<double>::quiet_NaN(),
      double beam_z_separation = 0.0);

  void reset();
  void beginReferenceLearning();
  void cancelReferenceLearning();
  bool loadReference(const std::string & path, const std::string & frame, std::string & report);
  bool saveReference(const std::string & path, const std::string & frame, std::string & report);
  std::string referenceSummary() const;
  const HookDetection & getSavedReference(size_t side) const { return saved_reference_.at(side); }
  bool isReferenceLearning() const { return learning_reference_; }

  const std::vector<int> & getLeftBodyIndices() const { return result_.left.body_indices; }
  const std::vector<int> & getRightBodyIndices() const { return result_.right.body_indices; }
  const CraneHookResult & getResult() const { return result_; }

private:
  CraneHookConfig config_;
  CraneHookResult result_;
  std::array<LinearKalmanFilter, 2> xy_filters_, z_filters_;
  std::array<double, 2> last_measurement_stamps_{};
  // Only real confirmed bodies populate this memory; fallback never updates it.
  std::array<HookDetection, 2> confirmed_bodies_{};
  double last_stamp_ = std::numeric_limits<double>::quiet_NaN();
  std::array<HookDetection, 2> saved_reference_{};
  // width, length, Z span, relative bounds (6), support, XY cells, surface ratio.
  std::array<std::vector<std::array<double, 12>>, 2> reference_samples_;
  std::array<std::array<size_t, 5>, 2> reference_rejections_{};
  bool learning_reference_ = false;
  std::array<std::vector<double>, 2> ground_history_;

  HookDetection detectSingleHook(
      const std::vector<PointXYZIRT> & points,
      const std::vector<int> & roi_indices,
      const HookDetection * prediction = nullptr, double gate_z = 0.0,
      double innovation_gate_z = 0.0, const HookDetection * saved_reference = nullptr,
      double beam_z = std::numeric_limits<double>::quiet_NaN(),
      double beam_z_separation = 0.0, bool reacquiring = false);
};

}  // namespace cx128s2_bev_perception
