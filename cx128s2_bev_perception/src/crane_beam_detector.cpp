#include <cx128s2_bev_perception/crane_beam_detector.hpp>

#include <cmath>
#include <iostream>
#include <limits>
#include <numeric>
#include <unordered_map>
#include <queue>

namespace cx128s2_bev_perception {

CraneBeamDetector::CraneBeamDetector(const CraneBeamConfig & config)
: config_(config), hook_detector_(config.hook) {
  reset();
  beam_roi_indices_.reserve(20000);
  bg_roi_indices_.reserve(50000);
  z_candidate_indices_.reserve(20000);
  beam_cluster_indices_.reserve(20000);
  support_indices_.reserve(20000);
  z_beam_vals_.reserve(20000);
  z_bg_vals_.reserve(50000);
}

void CraneBeamDetector::updateConfig(const CraneBeamConfig & config) {
  config_ = config;
  hook_detector_.updateConfig(config_.hook);
}

void CraneBeamDetector::reset() {
  result_ = CraneBeamDetectionResult{};
  result_.state = CraneBeamState::IDLE;
  result_.working_status = 0;
  result_.measurement_source = BeamMeasurementSource::INITIAL_ROI;
  result_.support_valid = false;
  result_.support_z = 0.0;
  result_.support_point_count = 0;
  result_.placement_gap_z = 0.0;
  result_.contact_candidate = false;
  result_.supported_segment_ratio = 0.0f;
  result_.motion_cumulative_dz = 0.0f;
  result_.tracking_candidate_count = 0;
  result_.selected_tracking_score = 0.0;

  vertical_motion_evidence_.reset();
  has_prev_support_z_ = false;
  prev_support_z_ = 0.0;

  has_previous_cluster_ = false;
  prev_cluster_centroid_x_ = 0.0;
  prev_cluster_centroid_y_ = 0.0;
  prev_cluster_yaw_ = 0.0;
  prev_cluster_length_ = 0.0;
  prev_cluster_pitch_slope_ = 0.0;

  has_prev_stamp_ = false;
  prev_stamp_sec_ = 0.0;
  has_prev_beam_z_ = false;
  prev_beam_z_raw_ = 0.0;
  prev_beam_z_filtered_ = 0.0;
  has_prev_bg_z_ = false;
  prev_bg_z_raw_ = 0.0;
  prev_bg_z_filtered_ = 0.0;

  prev_beam_length_ = 0.0;

  latched_work_complete_ = false;
  latched_locked_length_ = 0.0;
  latched_length_locked_ = false;
  latched_support_valid_ = false;
  latched_support_z_ = 0.0;
  latched_placement_gap_z_ = 0.0;
  latched_supported_segment_ratio_ = 0.0f;
  latched_contact_candidate_ = false;

  beam_roi_indices_.clear();
  bg_roi_indices_.clear();
  z_candidate_indices_.clear();
  beam_cluster_indices_.clear();
  support_indices_.clear();
  z_beam_vals_.clear();
  z_bg_vals_.clear();

  hook_detector_.reset();
  beam_xy_filter_ = LinearKalmanFilter{};
  beam_z_filter_ = LinearKalmanFilter{};
  beam_local_geometry_.clear();
  locked_half_width_ = 0.0;
  beam_lock_z_ = beam_lock_bg_z_ = 0.0;
  crane_move_direction_ = 0;
  crane_move_anchor_valid_ = {};
  crane_move_anchors_ = {};
  for (auto & history : crane_move_hook_history_) history.clear();
  crane_move_history_.clear();
  crane_move_previous_mask_ = 0;
  crane_move_position_sources_ = {};
  cycle_prepare_ready_ = false;
  cycle_prepare_sides_ = {};
  previous_hook_z_ = {};
  previous_hook_measured_ = {};
  prepared_hook_z_ = {};
  prepared_hook_bg_z_ = {};
  prepared_hook_valid_ = {};
  hook_lift_counters_ = {};
  for (auto & evidence : hook_motion_evidence_) evidence.reset();
}

bool CraneBeamDetector::insideBeamRoi(double x, double y, double z) const {
  return x >= config_.beam_roi_x_min && x <= config_.beam_roi_x_max &&
         y >= config_.beam_roi_y_min && y <= config_.beam_roi_y_max &&
         z >= config_.beam_roi_z_min && z <= config_.beam_roi_z_max;
}

bool CraneBeamDetector::insideBgRoi(double x, double y, double z) const {
  return x >= config_.bg_roi_x_min && x <= config_.bg_roi_x_max &&
         y >= config_.bg_roi_y_min && y <= config_.bg_roi_y_max &&
         z >= config_.bg_roi_z_min && z <= config_.bg_roi_z_max;
}

bool CraneBeamDetector::computeMedianAndMAD(std::vector<float> & vals, float & median, float & mad) {
  if (vals.empty()) {
    median = 0.0f;
    mad = 0.0f;
    return false;
  }
  const size_t n = vals.size();
  const size_t mid = n / 2;
  std::nth_element(vals.begin(), vals.begin() + mid, vals.end());
  median = vals[mid];

  std::vector<float> diffs(n);
  for (size_t i = 0; i < n; ++i) {
    diffs[i] = std::abs(vals[i] - median);
  }
  std::nth_element(diffs.begin(), diffs.begin() + mid, diffs.end());
  mad = diffs[mid];
  return true;
}

bool CraneBeamDetector::computeQuantiles(
    std::vector<float> & vals,
    float & min_v, float & q1, float & median, float & q3, float & max_v, float & mad) {
  if (vals.empty()) {
    min_v = q1 = median = q3 = max_v = mad = 0.0f;
    return false;
  }
  const size_t n = vals.size();
  std::sort(vals.begin(), vals.end());
  min_v = vals.front();
  q1 = vals[n / 4];
  median = vals[n / 2];
  q3 = vals[(3 * n) / 4];
  max_v = vals.back();

  std::vector<float> diffs(n);
  for (size_t i = 0; i < n; ++i) {
    diffs[i] = std::abs(vals[i] - median);
  }
  const size_t mid = n / 2;
  std::nth_element(diffs.begin(), diffs.begin() + mid, diffs.end());
  mad = diffs[mid];
  return true;
}

void CraneBeamDetector::computeClusterPCA(
    const std::vector<PointXYZIRT> & points,
    const std::vector<int> & cluster_idx,
    double & length,
    double & yaw,
    double & centroid_x,
    double & centroid_y) const {
  if (cluster_idx.empty()) {
    length = 0.0;
    yaw = 0.0;
    centroid_x = 0.0;
    centroid_y = 0.0;
    return;
  }

  double sum_x = 0.0;
  double sum_y = 0.0;
  for (int idx : cluster_idx) {
    sum_x += points[idx].x;
    sum_y += points[idx].y;
  }
  const double n = static_cast<double>(cluster_idx.size());
  centroid_x = sum_x / n;
  centroid_y = sum_y / n;

  double cxx = 0.0, cyy = 0.0, cxy = 0.0;
  for (int idx : cluster_idx) {
    const double dx = points[idx].x - centroid_x;
    const double dy = points[idx].y - centroid_y;
    cxx += dx * dx;
    cyy += dy * dy;
    cxy += dx * dy;
  }
  cxx /= n;
  cyy /= n;
  cxy /= n;

  // Principal axis angle in 2D
  yaw = 0.5 * std::atan2(2.0 * cxy, cxx - cyy);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  double min_proj = std::numeric_limits<double>::max();
  double max_proj = std::numeric_limits<double>::lowest();
  for (int idx : cluster_idx) {
    const double dx = points[idx].x - centroid_x;
    const double dy = points[idx].y - centroid_y;
    const double proj = dx * cos_yaw + dy * sin_yaw;
    min_proj = std::min(min_proj, proj);
    max_proj = std::max(max_proj, proj);
  }
  length = std::max(0.0, max_proj - min_proj);
}

void CraneBeamDetector::extractBeamCluster(
    const std::vector<PointXYZIRT> & points,
    double z_beam,
    bool use_continuity_seed) {
  z_candidate_indices_.clear();
  beam_cluster_indices_.clear();
  result_.beam_cluster_valid = false;
  result_.seed_roi_connected = false;

  const double z_band = config_.beam_z_band;
  for (size_t i = 0; i < points.size(); ++i) {
    if (std::abs(points[i].z - z_beam) <= z_band) {
      z_candidate_indices_.push_back(static_cast<int>(i));
    }
  }

  if (z_candidate_indices_.size() < config_.min_beam_cluster_points) {
    return;
  }

  const double res = std::max(0.02, config_.cluster_bev_res);
  auto hash2d = [](int x, int y) -> int64_t {
    return (static_cast<int64_t>(static_cast<uint32_t>(x)) << 32) |
           static_cast<uint32_t>(y);
  };

  std::unordered_map<int64_t, std::vector<int>> grid;
  grid.reserve(z_candidate_indices_.size());
  for (int idx : z_candidate_indices_) {
    const auto & p = points[idx];
    const int ix = static_cast<int>(std::floor(p.x / res));
    const int iy = static_cast<int>(std::floor(p.y / res));
    grid[hash2d(ix, iy)].push_back(idx);
  }

  std::vector<int64_t> seed_keys;
  // Priority 1: previous cluster continuity seed
  if (use_continuity_seed && has_previous_cluster_) {
    const double gate_sq = config_.continuity_gate_xy * config_.continuity_gate_xy;
    for (const auto & entry : grid) {
      const int ix = static_cast<int32_t>(static_cast<uint32_t>(entry.first >> 32));
      const int iy = static_cast<int32_t>(static_cast<uint32_t>(entry.first));
      const double cx = (ix + 0.5) * res;
      const double cy = (iy + 0.5) * res;
      const double d2 = (cx - prev_cluster_centroid_x_) * (cx - prev_cluster_centroid_x_) +
                        (cy - prev_cluster_centroid_y_) * (cy - prev_cluster_centroid_y_);
      if (d2 <= gate_sq) {
        seed_keys.push_back(entry.first);
      }
    }
  }

  // Priority 2: Beam ROI seed (if continuity seed found nothing or not using continuity)
  if (seed_keys.empty()) {
    for (const auto & entry : grid) {
      const int ix = static_cast<int32_t>(static_cast<uint32_t>(entry.first >> 32));
      const int iy = static_cast<int32_t>(static_cast<uint32_t>(entry.first));
      const double cx = (ix + 0.5) * res;
      const double cy = (iy + 0.5) * res;
      if (cx >= config_.beam_roi_x_min && cx <= config_.beam_roi_x_max &&
          cy >= config_.beam_roi_y_min && cy <= config_.beam_roi_y_max) {
        seed_keys.push_back(entry.first);
      }
    }
  }

  if (seed_keys.empty()) {
    return;
  }

  // Initial acquisition is valid only when this connected component is rooted
  // at a Beam Seed ROI cell.  The seed cells and BFS already exist here, so no
  // second point-cloud pass or connected-component computation is needed.
  if (!use_continuity_seed && result_.state == CraneBeamState::IDLE) {
    result_.seed_roi_connected = true;
  }

  // Connected component BFS (8-connected)
  std::unordered_map<int64_t, bool> visited;
  std::queue<int64_t> q;
  for (int64_t sk : seed_keys) {
    if (!visited[sk]) {
      visited[sk] = true;
      q.push(sk);
    }
  }

  while (!q.empty()) {
    const int64_t curr = q.front();
    q.pop();

    const auto it = grid.find(curr);
    if (it != grid.end()) {
      beam_cluster_indices_.insert(beam_cluster_indices_.end(), it->second.begin(), it->second.end());
    }

    const int ix = static_cast<int32_t>(static_cast<uint32_t>(curr >> 32));
    const int iy = static_cast<int32_t>(static_cast<uint32_t>(curr));

    const int reach = std::max(1, static_cast<int>(std::ceil(0.18 / res)));
    for (int dx = -reach; dx <= reach; ++dx) {
      for (int dy = -reach; dy <= reach; ++dy) {
        if (dx == 0 && dy == 0) continue;
        const int64_t nk = hash2d(ix + dx, iy + dy);
        if (grid.find(nk) != grid.end() && !visited[nk]) {
          visited[nk] = true;
          q.push(nk);
        }
      }
    }
  }

  if (beam_cluster_indices_.size() >= config_.min_beam_cluster_points) {
    result_.beam_cluster_valid = true;
    result_.beam_cluster_point_count = beam_cluster_indices_.size();
    computeClusterPCA(
        points, beam_cluster_indices_,
        result_.beam_length_raw, result_.beam_major_axis_yaw,
        result_.cluster_centroid_x, result_.cluster_centroid_y);

    // Initial acquisition shape gating (reject humans, poles, boxes in IDLE state)
    if (!use_continuity_seed && result_.state == CraneBeamState::IDLE) {
      const double sin_yaw = std::sin(result_.beam_major_axis_yaw);
      const double cos_yaw = std::cos(result_.beam_major_axis_yaw);
      double min_w = std::numeric_limits<double>::max();
      double max_w = std::numeric_limits<double>::lowest();
      for (int idx : beam_cluster_indices_) {
        const double dx = points[idx].x - result_.cluster_centroid_x;
        const double dy = points[idx].y - result_.cluster_centroid_y;
        const double perp = -dx * sin_yaw + dy * cos_yaw;
        min_w = std::min(min_w, perp);
        max_w = std::max(max_w, perp);
      }
      const double beam_w = std::max(0.05, max_w - min_w);
      const double elongation = result_.beam_length_raw / beam_w;
      result_.initial_cluster_length = result_.beam_length_raw;
      result_.initial_cluster_elongation = elongation;

      if (result_.beam_length_raw < config_.min_initial_cluster_length ||
          elongation < config_.min_initial_cluster_elongation) {
        result_.beam_cluster_valid = false;
        beam_cluster_indices_.clear();
        result_.beam_cluster_point_count = 0;
        return;
      }
    }

    has_previous_cluster_ = true;
    prev_cluster_centroid_x_ = result_.cluster_centroid_x;
    prev_cluster_centroid_y_ = result_.cluster_centroid_y;
    prev_cluster_yaw_ = result_.beam_major_axis_yaw;
    prev_cluster_length_ = result_.beam_length_raw;

    // Estimate longitudinal pitch slope along major axis
    const double cos_yaw = std::cos(result_.beam_major_axis_yaw);
    const double sin_yaw = std::sin(result_.beam_major_axis_yaw);
    double sum_u_sq = 0.0;
    double sum_u_dz = 0.0;
    for (int idx : beam_cluster_indices_) {
      const auto & p = points[idx];
      const double dx = p.x - result_.cluster_centroid_x;
      const double dy = p.y - result_.cluster_centroid_y;
      const double u = dx * cos_yaw + dy * sin_yaw;
      const double dz = p.z - result_.z_beam_raw;
      sum_u_sq += u * u;
      sum_u_dz += u * dz;
    }
    if (sum_u_sq > 1e-3) {
      prev_cluster_pitch_slope_ = std::clamp(sum_u_dz / sum_u_sq, -0.40, 0.40);
    } else {
      prev_cluster_pitch_slope_ = 0.0;
    }
  }
}

bool CraneBeamDetector::extractAcquisitionBeamZ(
    const std::vector<PointXYZIRT> & points,
    const std::vector<int> & roi_indices,
    double ground_z,
    double & beam_z,
    float & mad) {
  if (roi_indices.size() < config_.min_beam_roi_points) {
    beam_z = 0.0;
    mad = 0.0f;
    return false;
  }

  const double bin_size = std::max(0.01, config_.beam_z_histogram_bin_size);
  const double z_min = config_.beam_roi_z_min;
  const double z_max = config_.beam_roi_z_max;
  const int num_bins = static_cast<int>(std::ceil((z_max - z_min) / bin_size));
  if (num_bins <= 0) return false;

  std::vector<std::vector<float>> bins(num_bins);
  for (int idx : roi_indices) {
    const float pz = points[idx].z;
    if (pz >= z_min && pz <= z_max) {
      int b = static_cast<int>(std::floor((pz - z_min) / bin_size));
      if (b >= 0 && b < num_bins) {
        bins[b].push_back(pz);
      }
    }
  }

  // Find ground cutoff: anything below ground_z + beam_ground_clearance is considered ground/surface
  const double ground_cutoff = ground_z + config_.beam_ground_clearance;

  int best_bin = -1;
  size_t best_count = 0;

  for (int b = 0; b < num_bins; ++b) {
    const double bin_center_z = z_min + (b + 0.5) * bin_size;
    if (bin_center_z < ground_cutoff) continue;
    if (bins[b].size() < config_.min_beam_cluster_points) continue;

    if (has_prev_beam_z_) {
      double dist = std::abs(bin_center_z - prev_beam_z_filtered_);
      if (dist > 1.5) continue;
    }

    if (bins[b].size() > best_count) {
      best_count = bins[b].size();
      best_bin = b;
    }
  }

  if (best_bin < 0) {
    beam_z = 0.0;
    mad = 0.0f;
    return false;
  }

  // Collect points in the selected bin and adjacent bins (+- 1 bin) for robust median
  std::vector<float> mode_points;
  for (int db = -1; db <= 1; ++db) {
    int nb = best_bin + db;
    if (nb >= 0 && nb < num_bins) {
      mode_points.insert(mode_points.end(), bins[nb].begin(), bins[nb].end());
    }
  }

  float med = 0.0f;
  if (!computeMedianAndMAD(mode_points, med, mad)) {
    return false;
  }
  beam_z = static_cast<double>(med);
  return true;
}

bool CraneBeamDetector::extractTrackedBeamCluster(
    const std::vector<PointXYZIRT> & points,
    const std::vector<int> & valid_indices) {
  z_candidate_indices_.clear();
  beam_cluster_indices_.clear();
  result_.tracking_candidate_count = 0;
  result_.selected_tracking_score = 0.0;
  // A long loss retires association for this cycle. Never fall back to another
  // component or the acquisition ROI; reset() is the only identity release.
  if (!has_previous_cluster_ || result_.lost_frames_counter > config_.beam_measurement_max_lost_frames) {
    result_.beam_tracking_state = "LOST";
    return false;
  }
  const double dt = result_.dt_sec;
  if (dt > config_.hook.tracking_prediction_timeout_sec) {
    result_.lost_frames_counter = config_.beam_measurement_max_lost_frames + 1;
    result_.beam_tracking_state = "LOST";
    return false;
  }
  double remaining = dt;
  while (remaining > 1e-9) {
    const double step = std::min(0.1, remaining);
    beam_xy_filter_.predict(step, config_.hook.tracking_accel_std_xy);
    beam_z_filter_.predict(step, config_.hook.tracking_accel_std_z);
    remaining -= step;
  }
  const vehicle_surface::Point2 predicted{beam_xy_filter_.x(), beam_xy_filter_.y()};
  const double predicted_z = beam_z_filter_.x();
  const double std_z = config_.hook.tracking_measurement_std;
  const double innovation_gate = std::min(config_.tracking_gate_z,
      std::max(config_.beam_z_band * 0.5,
        3.0 * std::sqrt(beam_z_filter_.positionVarianceX() + std_z * std_z)));
  const double xy_gate = std::min(config_.tracking_gate_xy,
      std::max(config_.cluster_bev_res, 3.0 * std::sqrt(
          std::max(beam_xy_filter_.positionVarianceX(), beam_xy_filter_.positionVarianceY()) + std_z * std_z)));
  double locked_min_u = 1e30, locked_max_u = -1e30, locked_min_v = 1e30, locked_max_v = -1e30;
  for (const auto & p : beam_local_geometry_) {
    locked_min_u = std::min(locked_min_u, static_cast<double>(p.x));
    locked_max_u = std::max(locked_max_u, static_cast<double>(p.x));
    locked_min_v = std::min(locked_min_v, static_cast<double>(p.y));
    locked_max_v = std::max(locked_max_v, static_cast<double>(p.y));
  }
  std::vector<float> local_z;
  double min_u = 1e30, max_u = -1e30, min_v = 1e30, max_v = -1e30;
  for (int idx : valid_indices) {
    const auto & p = points[idx];
    const auto q = vehicle_surface::toLocal({p.x, p.y}, predicted, prev_cluster_yaw_);
    if (q.x < locked_min_u - xy_gate || q.x > locked_max_u + xy_gate ||
        q.y < locked_min_v - xy_gate || q.y > locked_max_v + xy_gate) continue;
    const double residual_z = p.z - predicted_z - prev_cluster_pitch_slope_ * q.x;
    if (std::abs(residual_z) > config_.beam_z_band) continue;
    z_candidate_indices_.push_back(idx);
    local_z.push_back(p.z - prev_cluster_pitch_slope_ * q.x);
    min_u = std::min(min_u, q.x); max_u = std::max(max_u, q.x);
    min_v = std::min(min_v, q.y); max_v = std::max(max_v, q.y);
  }
  float z = 0.0f, mad = 0.0f;
  const bool support = local_z.size() >= config_.min_beam_cluster_points &&
      max_u - min_u >= std::min(config_.min_initial_cluster_length, prev_cluster_length_ * 0.5) &&
      max_v - min_v <= locked_max_v - locked_min_v + 2.0 * config_.cluster_bev_res &&
      computeMedianAndMAD(local_z, z, mad) && std::abs(z - predicted_z) <= innovation_gate;
  if (!support) {
    result_.beam_tracking_state = "PREDICTED";
    result_.cluster_centroid_x = predicted.x;
    result_.cluster_centroid_y = predicted.y;
    result_.z_beam_filtered = predicted_z;
    return false;
  }
  // Use locked local bounds as the anchor, rather than the centroid of a
  // clipped gate. Partial longitudinal support updates Z but not XY identity.
  const bool full_span = max_u - min_u >= prev_cluster_length_ - config_.tracking_gate_length;
  const double u = full_span ? 0.5 * (min_u + max_u - locked_min_u - locked_max_u) : 0.0;
  const double v = full_span ? 0.5 * (min_v + max_v - locked_min_v - locked_max_v) : 0.0;
  const auto measured = vehicle_surface::toWorld({u, v}, predicted, prev_cluster_yaw_);
  if (std::hypot(measured.x - predicted.x, measured.y - predicted.y) > config_.tracking_gate_xy) {
    result_.beam_tracking_state = "PREDICTED";
    return false;
  }
  if (full_span) beam_xy_filter_.update(measured.x, measured.y, std_z);
  beam_z_filter_.update(z, 0.0, std_z);
  beam_cluster_indices_ = z_candidate_indices_;
  std::vector<float> zs;
  for (int idx : beam_cluster_indices_) zs.push_back(points[idx].z);
  float lo = 0, q1 = 0, med = 0, q3 = 0, hi = 0;
  computeQuantiles(zs, lo, q1, med, q3, hi, mad);
  result_.z_beam_min = lo; result_.z_beam_q1 = q1;
  result_.z_beam_raw = z; result_.z_beam_q3 = q3; result_.z_beam_max = hi;
  result_.beam_mad = mad;
  result_.cluster_centroid_x = beam_xy_filter_.x();
  result_.cluster_centroid_y = beam_xy_filter_.y();
  result_.beam_major_axis_yaw = prev_cluster_yaw_;
  result_.beam_length_raw = result_.beam_length_locked;
  result_.beam_z_valid = result_.beam_cluster_valid = true;
  result_.beam_cluster_point_count = beam_cluster_indices_.size();
  result_.tracking_candidate_count = 1;
  result_.selected_tracking_score = 1.0;
  result_.beam_tracking_state = "MEASURED";
  prev_cluster_centroid_x_ = result_.cluster_centroid_x;
  prev_cluster_centroid_y_ = result_.cluster_centroid_y;
  return true;
}

void CraneBeamDetector::estimateLocalSupportSurface(
    const std::vector<PointXYZIRT> & points,
    const std::vector<int> & valid_indices) {
  support_indices_.clear();
  result_.support_valid = false;
  result_.support_z = 0.0;
  result_.support_point_count = 0;
  result_.placement_gap_z = 0.0;
  result_.contact_candidate = false;
  result_.supported_segment_ratio = 0.0f;

  if (!result_.beam_cluster_valid || beam_cluster_indices_.empty()) {
    return;
  }

  const double cx = result_.cluster_centroid_x;
  const double cy = result_.cluster_centroid_y;
  const double yaw = result_.beam_major_axis_yaw;
  const double cos_y = std::cos(yaw);
  const double sin_y = std::sin(yaw);

  const double inner_half_len = std::max(0.1, (result_.beam_length_raw > 0.0 ? result_.beam_length_raw / 2.0 : 4.0) - 0.05);
  const double inner_half_width = 0.20;
  const double half_len = (result_.beam_length_raw > 0.0 ? result_.beam_length_raw / 2.0 : 5.0) + config_.support_search_margin_xy;
  const double half_width = 0.15 + config_.support_search_margin_xy;

  for (int idx : valid_indices) {
    const auto & p = points[idx];
    if (p.z < config_.bg_roi_z_min) continue;

    const double dx = p.x - cx;
    const double dy = p.y - cy;
    const double u = dx * cos_y + dy * sin_y;
    const double v = -dx * sin_y + dy * cos_y;

    if (std::abs(u) > half_len || std::abs(v) > half_width) continue;

    // Inside inner beam footprint, require points to be strictly below the beam bottom.
    // In the outer margin ring, points represent the surrounding support surface (deck or ground)
    // and can be level with the beam bottom.
    const bool inside_inner = (std::abs(u) <= inner_half_len && std::abs(v) <= inner_half_width);
    const double z_max_allowed = inside_inner ? (result_.z_beam_min - 0.02) : (result_.z_beam_min + 0.05);

    if (p.z <= z_max_allowed) {
      support_indices_.push_back(idx);
    }
  }

  result_.support_point_count = support_indices_.size();
  if (support_indices_.size() < config_.min_support_points) {
    return;
  }

  // Histogram support points in 0.05m bins to find highest stable support surface
  const double z_min_support = config_.bg_roi_z_min;
  const double z_max_support = result_.z_beam_min + 0.10;
  const double bin_size = 0.05;
  const int num_bins = static_cast<int>(std::ceil((z_max_support - z_min_support) / bin_size));
  if (num_bins <= 0) return;

  std::vector<std::vector<int>> bins(num_bins);
  for (int idx : support_indices_) {
    const float pz = points[idx].z;
    int b = static_cast<int>(std::floor((pz - z_min_support) / bin_size));
    if (b >= 0 && b < num_bins) {
      bins[b].push_back(idx);
    }
  }

  // Find the highest bin with significant point support and wide planar extent
  const size_t min_bin_support = std::max(size_t(8), config_.min_support_points / 3);
  int highest_valid_bin = -1;

  for (int b = num_bins - 1; b >= 0; --b) {
    if (bins[b].size() < min_bin_support) {
      continue;
    }

    // Check spatial extent across layer (b +- 1)
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::lowest();

    for (int db = -1; db <= 1; ++db) {
      int nb = b + db;
      if (nb >= 0 && nb < num_bins) {
        for (int idx : bins[nb]) {
          min_x = std::min(min_x, static_cast<double>(points[idx].x));
          max_x = std::max(max_x, static_cast<double>(points[idx].x));
          min_y = std::min(min_y, static_cast<double>(points[idx].y));
          max_y = std::max(max_y, static_cast<double>(points[idx].y));
        }
      }
    }

    const double span_x = max_x - min_x;
    const double span_y = max_y - min_y;
    const double max_span = std::max(span_x, span_y);

    // Reject narrow vertical objects (worker: ~0.4m, pole: ~0.1m)
    if (max_span < config_.support_min_xy_extent) {
      continue;
    }

    highest_valid_bin = b;
    break;
  }

  if (highest_valid_bin < 0) {
    return;
  }

  // Compute robust median and MAD of points in the highest support layer
  std::vector<float> layer_points;
  for (int db = -1; db <= 1; ++db) {
    int nb = highest_valid_bin + db;
    if (nb >= 0 && nb < num_bins) {
      for (int idx : bins[nb]) {
        layer_points.push_back(points[idx].z);
      }
    }
  }

  float sup_med = 0.0f, sup_mad = 0.0f;
  if (!computeMedianAndMAD(layer_points, sup_med, sup_mad)) {
    return;
  }

  if (sup_mad > config_.support_max_z_spread) {
    // Non-planar support layer
    return;
  }

  // Temporal continuity preference:
  // If we already had a confirmed support level and the new one jumps upward drastically,
  // verify if the previous level is still valid and present.
  if (config_.support_temporal_continuity && has_prev_support_z_) {
    const double z_diff = sup_med - prev_support_z_;
    if (z_diff > 0.25) {
      int prev_b = static_cast<int>(std::floor((prev_support_z_ - z_min_support) / bin_size));
      if (prev_b >= 0 && prev_b < num_bins && bins[prev_b].size() >= min_bin_support) {
        sup_med = static_cast<float>(prev_support_z_);
      }
    }
  }

  result_.support_z = static_cast<double>(sup_med);
  result_.support_valid = true;
  has_prev_support_z_ = true;
  prev_support_z_ = result_.support_z;

  // Determine representative beam bottom Z
  double beam_bottom_z = result_.z_beam_min;
  if (result_.z_beam_max - result_.z_beam_min < 0.20 && config_.beam_default_height > 0.0) {
    beam_bottom_z = result_.z_beam_filtered - config_.beam_default_height;
  }

  result_.placement_gap_z = beam_bottom_z - result_.support_z;

  // Longitudinal multi-segment evaluation for tilted touchdown
  if (config_.tilt_segmentation_enabled && result_.beam_cluster_valid && result_.beam_length_raw >= 1.0) {
    const int K = std::max(2, config_.tilt_num_segments);
    const double L = result_.beam_length_raw;
    const double seg_len = L / K;
    const double start_u = -L / 2.0;

    std::vector<std::vector<float>> seg_z(K);
    for (int idx : beam_cluster_indices_) {
      const auto & p = points[idx];
      const double dx = p.x - cx;
      const double dy = p.y - cy;
      const double u = dx * cos_y + dy * sin_y;
      int k = static_cast<int>(std::floor((u - start_u) / seg_len));
      k = std::clamp(k, 0, K - 1);
      seg_z[k].push_back(p.z);
    }

    const double tol = (config_.placement_tolerance > 0.0) ? config_.placement_tolerance : config_.ground_tolerance;
    int supported_count = 0;
    int valid_segs = 0;
    bool any_contact = false;

    for (int k = 0; k < K; ++k) {
      if (seg_z[k].size() < 5) continue;
      valid_segs++;

      float seg_min = *std::min_element(seg_z[k].begin(), seg_z[k].end());
      float seg_max = *std::max_element(seg_z[k].begin(), seg_z[k].end());

      double seg_bottom = static_cast<double>(seg_min);
      if ((seg_max - seg_min) < 0.20 && config_.beam_default_height > 0.0) {
        std::nth_element(seg_z[k].begin(), seg_z[k].begin() + seg_z[k].size() / 2, seg_z[k].end());
        float seg_med = seg_z[k][seg_z[k].size() / 2];
        seg_bottom = static_cast<double>(seg_med) - config_.beam_default_height;
      }

      double seg_gap = seg_bottom - result_.support_z;
      if (seg_gap <= tol) {
        any_contact = true;
        supported_count++;
      }
    }

    result_.contact_candidate = any_contact;
    if (valid_segs > 0) {
      result_.supported_segment_ratio = static_cast<float>(supported_count) / static_cast<float>(valid_segs);
    } else {
      result_.supported_segment_ratio = 0.0f;
    }
  } else {
    const double tol = (config_.placement_tolerance > 0.0) ? config_.placement_tolerance : config_.ground_tolerance;
    result_.contact_candidate = (result_.placement_gap_z <= tol);
    result_.supported_segment_ratio = result_.contact_candidate ? 1.0f : 0.0f;
  }
}

const CraneBeamDetectionResult & CraneBeamDetector::process(
    const std::vector<PointXYZIRT> & points,
    const std::vector<int> & valid_indices,
    double stamp_sec) {
  if (!std::isfinite(stamp_sec)) { reset(); return result_; }
  if (has_prev_stamp_ && stamp_sec == prev_stamp_sec_) { result_.state_changed = false; return result_; }
  if (has_prev_stamp_ && stamp_sec < prev_stamp_sec_) reset();
  result_.state_changed = false;
  beam_roi_indices_.clear();
  bg_roi_indices_.clear();
  support_indices_.clear();
  z_beam_vals_.clear();
  z_bg_vals_.clear();
  result_.beam_high_z_rejected = false;
  result_.seed_roi_connected = false;
  result_.acquisition_reset_reason = AcquisitionResetReason::NONE;
  result_.height_gap_raw = 0.0;
  result_.height_gap_filtered = 0.0;
  result_.beam_high_z_indicator = 0.0;
  result_.initial_cluster_length = 0.0;
  result_.initial_cluster_elongation = 0.0;
  if (!result_.length_locked) result_.beam_length_raw = 0.0;

  // Determine effective motion reference mode
  std::string effective_mode = config_.motion_reference_mode;
  if (effective_mode == "auto") {
    if ((config_.target_frame == "world" || config_.target_frame == "map" || config_.target_frame == "odom") &&
        !config_.freeze_sensor_transform) {
      effective_mode = "absolute";
    } else {
      effective_mode = "relative_background";
    }
  }
  result_.motion_reference_mode = effective_mode;

  // Background ROI point partitioning
  for (int idx : valid_indices) {
    const auto & p = points[idx];
    if (insideBgRoi(p.x, p.y, p.z) && !insideBeamRoi(p.x, p.y, p.z)) {
      bg_roi_indices_.push_back(idx);
      z_bg_vals_.push_back(p.z);
    }
  }

  result_.bg_roi_point_count = bg_roi_indices_.size();
  float z_bg_f = 0.0f, bg_mad_f = 0.0f;
  float z_bg_min_f = 0.0f, z_bg_q1_f = 0.0f, z_bg_q3_f = 0.0f, z_bg_max_f = 0.0f;
  result_.bg_z_valid = computeQuantiles(
                           z_bg_vals_, z_bg_min_f, z_bg_q1_f, z_bg_f, z_bg_q3_f, z_bg_max_f, bg_mad_f) &&
                       (z_bg_vals_.size() >= config_.min_bg_roi_points) &&
                       std::isfinite(z_bg_f);
  if (result_.bg_z_valid) {
    result_.z_bg_raw = static_cast<double>(z_bg_f);
    result_.bg_mad = static_cast<double>(bg_mad_f);
    result_.z_bg_min = static_cast<double>(z_bg_min_f);
    result_.z_bg_q1 = static_cast<double>(z_bg_q1_f);
    result_.z_bg_q3 = static_cast<double>(z_bg_q3_f);
    result_.z_bg_max = static_cast<double>(z_bg_max_f);
  }

  // Calculate dt and velocities
  double dt = 0.1;
  if (has_prev_stamp_ && stamp_sec > prev_stamp_sec_) {
    dt = std::clamp(stamp_sec - prev_stamp_sec_, 0.001, 1.0);
  }
  result_.dt_sec = dt;

  // BEAM MEASUREMENT: Separation of Initial Acquisition vs. Tracked Mode
  const bool is_initial_acquisition = !result_.length_locked;

  if (is_initial_acquisition) {
    result_.measurement_source = BeamMeasurementSource::INITIAL_ROI;
    for (int idx : valid_indices) {
      const auto & p = points[idx];
      if (insideBeamRoi(p.x, p.y, p.z)) {
        beam_roi_indices_.push_back(idx);
        z_beam_vals_.push_back(p.z);
      }
    }
    result_.beam_roi_point_count = beam_roi_indices_.size();

    double beam_z = 0.0;
    float beam_mad_f = 0.0f;
    result_.beam_z_valid = extractAcquisitionBeamZ(points, beam_roi_indices_, result_.z_bg_raw, beam_z, beam_mad_f);
    // Reuse the current supported ROI-band median before Hook acquisition.
    // Beam geometry acquisition below still consumes this frame's Hook READY.
    result_.hook = hook_detector_.process(points, valid_indices, stamp_sec,
        result_.beam_z_valid ? beam_z : std::numeric_limits<double>::quiet_NaN(),
        config_.beam_z_band);
    if (result_.beam_z_valid) {
      result_.z_beam_raw = beam_z;
      result_.beam_mad = static_cast<double>(beam_mad_f);
      result_.z_beam_min = beam_z;
      result_.z_beam_q1 = beam_z;
      result_.z_beam_q3 = beam_z;
      result_.z_beam_max = beam_z;
      // Histogram Z is diagnostic until preparation supplies a real ground reference.
      bool surface_candidate = false;
      const HookDetection * hooks[] = {&result_.hook.left, &result_.hook.right};
      for (size_t side = 0; side < 2; ++side) {
        const auto & hook = *hooks[side];
        if (result_.hook.prepare_sides[side] && hook.detected && hook.ground_valid &&
            beam_z >= hook.ground_z + config_.beam_ground_clearance) surface_candidate = true;
      }
      if (result_.hook.prepare_ready && surface_candidate) {
        extractBeamCluster(points, result_.z_beam_raw, false);
        if (result_.beam_cluster_valid) {
          // First geometry is the identity for the whole cycle, including IDLE.
          std::vector<float> cluster_z;
          for (int idx : beam_cluster_indices_) cluster_z.push_back(points[idx].z);
          float center_z = 0.0f, mad = 0.0f;
          computeMedianAndMAD(cluster_z, center_z, mad);
          result_.z_beam_raw = center_z;
          result_.length_locked = latched_length_locked_ = true;
          result_.beam_length_locked = latched_locked_length_ = result_.beam_length_raw;
          result_.length_lock_counter = 1;
          cycle_prepare_ready_ = true;
          cycle_prepare_sides_ = result_.hook.prepare_sides;
          beam_lock_z_ = center_z;
          beam_lock_bg_z_ = result_.z_bg_raw;
          beam_xy_filter_.initialize(result_.cluster_centroid_x, result_.cluster_centroid_y,
              config_.hook.tracking_measurement_std, 0.5);
          beam_z_filter_.initialize(center_z, 0.0, config_.hook.tracking_measurement_std, 1.5);
          locked_half_width_ = 0.0;
          beam_local_geometry_.clear();
          double sum_uz = 0.0, sum_uu = 0.0;
          for (int idx : beam_cluster_indices_) {
            const auto q = vehicle_surface::toLocal({points[idx].x, points[idx].y},
                {result_.cluster_centroid_x, result_.cluster_centroid_y}, result_.beam_major_axis_yaw);
            auto local = points[idx]; local.x = q.x; local.y = q.y; local.z -= center_z;
            beam_local_geometry_.push_back(local);
            locked_half_width_ = std::max(locked_half_width_, std::abs(q.y));
            sum_uz += q.x * local.z; sum_uu += q.x * q.x;
          }
          prev_cluster_pitch_slope_ = sum_uu > 1e-3 ? std::clamp(sum_uz / sum_uu, -0.4, 0.4) : 0.0;
          has_prev_beam_z_ = false;
          result_.beam_tracking_state = "MEASURED";
        }
      } else {
        result_.beam_cluster_valid = false;
        result_.beam_cluster_point_count = 0;
        beam_cluster_indices_.clear(); z_candidate_indices_.clear();
      }

      // p90 of the initial ROI Z distribution is robust to isolated max-Z
      // returns while still exposing a substantial high object in the seed ROI.
      // It is intentionally raw: an EMA would hide the very short anomaly this
      // gate is intended to diagnose.  This signal is never used for support or
      // placement decisions.
      if (result_.bg_z_valid) {
        std::vector<float> roi_z;
        roi_z.reserve(beam_roi_indices_.size());
        for (int idx : beam_roi_indices_) {
          roi_z.push_back(points[idx].z);
        }
        if (!roi_z.empty()) {
          const size_t p90_index = (9 * (roi_z.size() - 1)) / 10;
          std::nth_element(roi_z.begin(), roi_z.begin() + p90_index, roi_z.end());
          result_.beam_high_z_indicator = static_cast<double>(roi_z[p90_index]) - result_.z_bg_raw;
          const bool high_z = config_.beam_high_z_gate_enabled &&
                              (result_.beam_high_z_indicator > config_.beam_high_z_reject_threshold);
          result_.beam_high_z_rejected = high_z && config_.beam_high_z_require_cluster_valid &&
                                         !result_.beam_cluster_valid;
        }
      }
    } else {
      z_candidate_indices_.clear();
      beam_cluster_indices_.clear();
      result_.beam_cluster_valid = false;
      result_.beam_cluster_point_count = 0;
    }
  } else {
    // TRACKED CLUSTER MODE: Never query the fixed initial ROI for beam height
    result_.measurement_source = BeamMeasurementSource::TRACKED_CLUSTER;
    beam_roi_indices_.clear();
    result_.beam_roi_point_count = 0;

    bool tracked = extractTrackedBeamCluster(points, valid_indices);
    if (!tracked) {
      result_.beam_z_valid = false;
      result_.beam_cluster_valid = false;
      result_.beam_cluster_point_count = 0;
    }
    result_.hook = hook_detector_.process(points, valid_indices, stamp_sec,
        result_.beam_z_valid ? result_.z_beam_raw : std::numeric_limits<double>::quiet_NaN(),
        config_.beam_z_band);
  }

  // IDLE is the only state that acquires from the fixed Seed ROI.  If no
  // Seed-ROI cell rooted the candidate component, discard only temporary
  // acquisition evidence.  Do not touch background, support, TF, or work
  // completion state; tracking states do not enter this branch.
  if (is_initial_acquisition && !result_.seed_roi_connected && !result_.beam_z_valid) {
    result_.beam_z_valid = false;
    result_.beam_cluster_valid = false;
    result_.beam_high_z_rejected = false;
    result_.z_beam_raw = 0.0;
    result_.z_beam_filtered = 0.0;
    result_.dz_beam_raw = 0.0;
    result_.dz_beam_filtered = 0.0;
    result_.beam_vertical_velocity = 0.0;
    result_.v_beam_raw = 0.0;
    result_.v_beam_filtered = 0.0;
    result_.beam_mad = 0.0;
    result_.beam_roi_point_count = 0;
    result_.beam_cluster_point_count = 0;
    result_.initial_cluster_length = 0.0;
    result_.initial_cluster_elongation = 0.0;
    result_.beam_high_z_indicator = 0.0;
    result_.lift_counter = 0;
    vertical_motion_evidence_.reset();
    has_previous_cluster_ = false;
    has_prev_beam_z_ = false;
    prev_beam_z_raw_ = 0.0;
    prev_beam_z_filtered_ = 0.0;
    z_candidate_indices_.clear();
    beam_cluster_indices_.clear();
    result_.acquisition_reset_reason = AcquisitionResetReason::NO_SEED_CONNECTION;
  }

  // Beam Z and dz filtering
  if (result_.beam_z_valid) {
    if (config_.use_z_ema && has_prev_beam_z_) {
      result_.z_beam_filtered = config_.z_ema_alpha * result_.z_beam_raw +
                                (1.0 - config_.z_ema_alpha) * prev_beam_z_filtered_;
    } else {
      result_.z_beam_filtered = result_.z_beam_raw;
    }

    if (has_prev_beam_z_) {
      result_.dz_beam_raw = result_.z_beam_raw - prev_beam_z_raw_;
      result_.dz_beam_filtered = result_.z_beam_filtered - prev_beam_z_filtered_;
      result_.beam_vertical_velocity = result_.dz_beam_filtered / dt;
      result_.v_beam_raw = result_.dz_beam_raw / dt;
      result_.v_beam_filtered = result_.dz_beam_filtered / dt;
    } else {
      result_.dz_beam_raw = 0.0;
      result_.dz_beam_filtered = 0.0;
      result_.beam_vertical_velocity = 0.0;
      result_.v_beam_raw = 0.0;
      result_.v_beam_filtered = 0.0;
    }
    prev_beam_z_raw_ = result_.z_beam_raw;
    prev_beam_z_filtered_ = result_.z_beam_filtered;
    has_prev_beam_z_ = true;
  }

  // Reuse this frame's existing Beam representative Z for independent Hook comparisons.
  // Invalid Beam/Hook measurements never reuse a stale reference or a predicted Hook.
  for (auto * hook : {&result_.hook.left, &result_.hook.right}) {
    hook->beam_distance_valid = result_.beam_z_valid && hook->detected &&
        std::isfinite(result_.z_beam_filtered) && std::isfinite(hook->z);
    if (hook->beam_distance_valid) {
      hook->beam_reference_z = result_.z_beam_filtered;
      hook->beam_dz = hook->z - hook->beam_reference_z;
    }
  }

  // Background Z and dz filtering
  if (result_.bg_z_valid) {
    if (config_.use_z_ema && has_prev_bg_z_) {
      result_.z_bg_filtered = config_.z_ema_alpha * result_.z_bg_raw +
                              (1.0 - config_.z_ema_alpha) * prev_bg_z_filtered_;
    } else {
      result_.z_bg_filtered = result_.z_bg_raw;
    }

    if (has_prev_bg_z_) {
      result_.dz_bg_raw = result_.z_bg_raw - prev_bg_z_raw_;
      result_.dz_bg_filtered = result_.z_bg_filtered - prev_bg_z_filtered_;
      result_.v_bg_raw = result_.dz_bg_raw / dt;
      result_.v_bg_filtered = result_.dz_bg_filtered / dt;
    } else {
      result_.dz_bg_raw = 0.0;
      result_.dz_bg_filtered = 0.0;
      result_.v_bg_raw = 0.0;
      result_.v_bg_filtered = 0.0;
    }
    prev_bg_z_raw_ = result_.z_bg_raw;
    prev_bg_z_filtered_ = result_.z_bg_filtered;
    has_prev_bg_z_ = true;
  }

  // Movement uses real body KF XY or the current highest-Z ROI point XY.
  // Neither prediction nor a held empty-ROI position supplies START/STOP evidence.
  // Background Z remains available for Beam/FSM motion compensation.
  const size_t move_frames = static_cast<size_t>(std::max(1, config_.crane_move_stop_frames));
  const auto xy_median = [](const std::vector<std::array<double, 2>> & samples) {
    std::array<double, 2> center{};
    for (size_t axis = 0; axis < 2; ++axis) {
      std::vector<double> values;
      for (const auto & sample : samples) values.push_back(sample[axis]);
      std::sort(values.begin(), values.end());
      center[axis] = .5 * (values[values.size() / 2] + values[(values.size() - 1) / 2]);
    }
    return center;
  };
  const HookDetection * move_hooks[] = {&result_.hook.left, &result_.hook.right};
  result_.crane_move_valid = false;
  result_.crane_move_source_mask = result_.crane_move_measured_frames = 0;
  result_.crane_move_hook_x = result_.crane_move_hook_y = 0.0;
  result_.crane_move_anchor_x = result_.crane_move_anchor_y = 0.0;
  result_.crane_move_net_dx = result_.crane_move_net_dy = 0.0;
  result_.crane_move_x_span = result_.crane_move_y_span = 0.0;
  result_.crane_move_reason = "WAIT_CONFIRMED_HOOK";
  int move_sources = 0;
  for (size_t side = 0; side < 2; ++side) {
    const auto & hook = *move_hooks[side];
    auto & history = crane_move_hook_history_[side];
    const bool trusted = !hook_detector_.isReferenceLearning() &&
        hook_detector_.getSavedReference(side).reference_valid;
    const bool measured_body = trusted && hook.initialized && hook.reference_valid && hook.detected;
    const bool fallback = hook.position_tracking_state == "FALLBACK_TRACKING" &&
        hook.position_updated && hook.fallback_point_valid;
    const int source = measured_body ? 1 : (fallback ? 2 : 0);
    const double move_x = measured_body ? hook.filtered_x : hook.fallback_x;
    const double move_y = measured_body ? hook.filtered_y : hook.fallback_y;
    if (source != 0 && source != crane_move_position_sources_[side]) {
      // Body center and ROI maximum may be different physical points. Establish
      // a fresh anchor so their offset is not interpreted as crane movement.
      history.clear();
      crane_move_anchor_valid_[side] = false;
      crane_move_history_.clear();
      crane_move_position_sources_[side] = source;
    }
    if ((!trusted || !hook.initialized || !hook.reference_valid) &&
        crane_move_position_sources_[side] != 2)
      crane_move_anchor_valid_[side] = false;
    if (source == 0 || !std::isfinite(move_x) || !std::isfinite(move_y)) {
      history.clear();
      continue;
    }
    history.push_back({move_x, move_y});
    if (history.size() > move_frames) history.erase(history.begin(), history.end() - move_frames);
    if (!crane_move_anchor_valid_[side]) {
      result_.crane_move_measured_frames = std::max(result_.crane_move_measured_frames,
          static_cast<int>(history.size()));
      if (history.size() < move_frames) continue;
      crane_move_anchors_[side] = xy_median(history);
      crane_move_anchor_valid_[side] = true;
    }
    ++move_sources;
    result_.crane_move_source_mask |= 1 << side;
    result_.crane_move_hook_x += move_x;
    result_.crane_move_hook_y += move_y;
    result_.crane_move_anchor_x += crane_move_anchors_[side][0];
    result_.crane_move_anchor_y += crane_move_anchors_[side][1];
  }
  if (move_sources > 0) {
    result_.crane_move_valid = true;
    result_.crane_move_hook_x /= move_sources; result_.crane_move_hook_y /= move_sources;
    result_.crane_move_anchor_x /= move_sources; result_.crane_move_anchor_y /= move_sources;
    result_.crane_move_net_dx = result_.crane_move_hook_x - result_.crane_move_anchor_x;
    result_.crane_move_net_dy = result_.crane_move_hook_y - result_.crane_move_anchor_y;
    // Per-side fixed anchors avoid the metre-scale Left/Right baseline jump.
    // A change of measured sources or any gap restarts the consecutive window.
    if (result_.crane_move_source_mask != crane_move_previous_mask_) crane_move_history_.clear();
    crane_move_previous_mask_ = result_.crane_move_source_mask;
    crane_move_history_.push_back({result_.crane_move_net_dx, result_.crane_move_net_dy});
    if (crane_move_history_.size() > move_frames)
      crane_move_history_.erase(crane_move_history_.begin(), crane_move_history_.end() - move_frames);
    for (size_t axis = 0; axis < 2; ++axis) {
      double low = crane_move_history_.front()[axis], high = low;
      for (const auto & xy : crane_move_history_) { low = std::min(low, xy[axis]); high = std::max(high, xy[axis]); }
      (axis == 0 ? result_.crane_move_x_span : result_.crane_move_y_span) = high - low;
    }
    result_.crane_move_measured_frames = static_cast<int>(crane_move_history_.size());
    const bool start_x = std::abs(result_.crane_move_net_dx) >= config_.crane_move_start_x;
    const bool start_y = std::abs(result_.crane_move_net_dy) >= config_.crane_move_start_y;
    if (!result_.crane_moving && (start_x || start_y)) {
      result_.crane_moving = true;
      result_.crane_move_reason = start_x ? (start_y ? "START_XY" : "START_X") : "START_Y";
      crane_move_history_ = {{result_.crane_move_net_dx, result_.crane_move_net_dy}};
      result_.crane_move_measured_frames = 1;
      result_.crane_move_x_span = result_.crane_move_y_span = 0.0;
    } else if (result_.crane_moving && crane_move_history_.size() == move_frames &&
        result_.crane_move_x_span <= config_.crane_move_stop_x &&
        result_.crane_move_y_span <= config_.crane_move_stop_y) {
      result_.crane_moving = false;
      result_.crane_move_reason = "STOP_STABLE_ANCHOR_UPDATED";
      result_.crane_move_anchor_x = result_.crane_move_anchor_y = 0.0;
      for (size_t side = 0; side < 2; ++side) {
        if (result_.crane_move_source_mask & (1 << side)) {
          crane_move_anchors_[side] = xy_median(crane_move_hook_history_[side]);
          result_.crane_move_anchor_x += crane_move_anchors_[side][0] / move_sources;
          result_.crane_move_anchor_y += crane_move_anchors_[side][1] / move_sources;
        } else crane_move_anchor_valid_[side] = false;
      }
      result_.crane_move_net_dx = result_.crane_move_hook_x - result_.crane_move_anchor_x;
      result_.crane_move_net_dy = result_.crane_move_hook_y - result_.crane_move_anchor_y;
      crane_move_history_.clear();
    } else result_.crane_move_reason = result_.crane_moving ? "HOLD_MOVING" : "HOLD_STOPPED";
  } else {
    crane_move_history_.clear();
    crane_move_previous_mask_ = 0;
    if (result_.crane_moving) result_.crane_move_reason = "HOLD_MOVING_NO_MEASURED_HOOK";
  }

  // Relative dz and unified motion_dz
  if (result_.beam_z_valid && result_.bg_z_valid) {
    result_.relative_dz_raw = result_.dz_beam_raw - result_.dz_bg_raw;
    result_.relative_dz_filtered = result_.dz_beam_filtered - result_.dz_bg_filtered;
    result_.height_gap_raw = result_.z_beam_raw - result_.z_bg_raw;
    result_.height_gap_filtered = result_.z_beam_filtered - result_.z_bg_filtered;
  }

  if (effective_mode == "absolute") {
    result_.motion_dz = result_.dz_beam_filtered;
  } else {
    result_.motion_dz = (result_.beam_z_valid && result_.bg_z_valid) ?
                        result_.relative_dz_filtered : result_.dz_beam_filtered;
  }

  // Update vertical motion evidence
  if (result_.beam_z_valid && result_.beam_cluster_valid && result_.length_locked) {
    vertical_motion_evidence_.update(result_.motion_dz, config_.cumulative_direction_epsilon);
  }
  if (vertical_motion_evidence_.positive_accum > vertical_motion_evidence_.negative_accum) {
    result_.motion_cumulative_dz = static_cast<float>(vertical_motion_evidence_.positive_accum);
  } else {
    result_.motion_cumulative_dz = static_cast<float>(-vertical_motion_evidence_.negative_accum);
  }

  // Geometry is retained even on long loss. Only bounded prediction is exposed.
  if (result_.length_locked && !result_.beam_z_valid) {
    ++result_.lost_frames_counter;
    if (result_.lost_frames_counter > std::max(0, config_.beam_measurement_max_lost_frames)) {
      result_.beam_tracking_state = "LOST";
    }
  } else if (result_.beam_z_valid) {
    result_.lost_frames_counter = 0;
  }
  if (cycle_prepare_ready_) {
    result_.hook.prepare_ready = true;
    result_.hook.prepare_sides = cycle_prepare_sides_;
  }

  // Local Support Surface Estimation (during LOWERING or COMPLETE_CONFIRM)
  if (result_.beam_cluster_valid &&
      (result_.state == CraneBeamState::LOWERING || result_.state == CraneBeamState::COMPLETE_CONFIRM)) {
    estimateLocalSupportSurface(points, valid_indices);
  } else if (result_.state == CraneBeamState::IDLE && latched_work_complete_) {
    result_.support_valid = latched_support_valid_;
    result_.support_z = latched_support_z_;
    result_.placement_gap_z = latched_placement_gap_z_;
    result_.supported_segment_ratio = latched_supported_segment_ratio_;
    result_.contact_candidate = latched_contact_candidate_;
  } else {
    support_indices_.clear();
    result_.support_valid = false;
    result_.support_z = 0.0;
    result_.support_point_count = 0;
    result_.placement_gap_z = 0.0;
  }

  // Hook motion uses the same absolute/background reference as Beam motion.
  // A returning measurement starts a fresh confirmation run after prediction.
  const HookDetection * hooks[] = {&result_.hook.left, &result_.hook.right};
  std::array<bool, 2> measured{}, up{}, down{}, stopped{}, near_ground{};
  const bool beam_measured = result_.length_locked && result_.beam_cluster_valid && result_.beam_z_valid;
  const bool reference_valid = effective_mode == "absolute" || result_.bg_z_valid;
  for (size_t side = 0; side < 2; ++side) {
    const auto & h = *hooks[side];
    // Signed net rise is relative to this prepared identity, never |dz| sum.
    if (!h.initialized || !result_.hook.prepare_sides[side]) prepared_hook_valid_[side] = false;
    const double prepared_rise = h.filtered_z - prepared_hook_z_[side] -
        (effective_mode == "absolute" ? 0.0 : result_.z_bg_filtered - prepared_hook_bg_z_[side]);
    if (result_.state == CraneBeamState::IDLE && result_.hook.prepare_sides[side] &&
        h.detected && h.initialized && reference_valid &&
        (!prepared_hook_valid_[side] || prepared_rise < 0.0)) {
      prepared_hook_z_[side] = h.filtered_z;
      prepared_hook_bg_z_[side] = result_.z_bg_filtered;
      prepared_hook_valid_[side] = true;
    }
    measured[side] = h.detected && h.initialized && h.tracking_state == "MEASURED" &&
        previous_hook_measured_[side] && reference_valid;
    const double dz = h.filtered_z - previous_hook_z_[side] -
        (effective_mode == "absolute" ? 0.0 : result_.dz_bg_filtered);
    const bool matched = beam_measured && measured[side] &&
        std::abs(dz - result_.motion_dz) <= config_.hook_beam_dz_match_tolerance;
    if (matched && dz * result_.motion_dz > 0.0 &&
        std::abs(dz) > config_.cumulative_direction_epsilon) {
      hook_motion_evidence_[side].update(dz, config_.cumulative_direction_epsilon);
    } else {
      hook_motion_evidence_[side].reset();
    }
    const auto & evidence = hook_motion_evidence_[side];
    up[side] = matched && dz > config_.cumulative_direction_epsilon &&
        (dz > config_.lift_dz_threshold || (config_.cumulative_motion_enabled &&
         evidence.positive_accum >= config_.cumulative_lift_distance &&
         evidence.positive_frames >= config_.cumulative_motion_min_frames));
    down[side] = matched && dz < -config_.cumulative_direction_epsilon &&
        (dz < -config_.lower_dz_threshold || (config_.cumulative_motion_enabled &&
         evidence.negative_accum >= config_.cumulative_lower_distance &&
         evidence.negative_frames >= config_.cumulative_motion_min_frames));
    stopped[side] = measured[side] && std::abs(dz) < config_.stop_dz_threshold &&
        (!config_.cumulative_motion_enabled || std::abs(dz) < config_.cumulative_direction_epsilon) &&
        std::abs(h.vz * dt - (effective_mode == "absolute" ? 0.0 : result_.dz_bg_filtered)) < config_.stop_dz_threshold;
    near_ground[side] = measured[side] && h.ground_valid &&
        h.ground_distance <= config_.hook.prepare_distance;
    previous_hook_z_[side] = h.filtered_z;
    previous_hook_measured_[side] = h.detected && h.initialized && h.tracking_state == "MEASURED";
  }
  const bool beam_up = beam_measured && reference_valid &&
      result_.motion_dz > config_.cumulative_direction_epsilon &&
      (result_.motion_dz > config_.lift_dz_threshold || (config_.cumulative_motion_enabled &&
       vertical_motion_evidence_.positive_accum >= config_.cumulative_lift_distance &&
       vertical_motion_evidence_.positive_frames >= config_.cumulative_motion_min_frames));
  const bool beam_down = beam_measured && reference_valid &&
      result_.motion_dz < -config_.cumulative_direction_epsilon &&
      (result_.motion_dz < -config_.lower_dz_threshold || (config_.cumulative_motion_enabled &&
       vertical_motion_evidence_.negative_accum >= config_.cumulative_lower_distance &&
       vertical_motion_evidence_.negative_frames >= config_.cumulative_motion_min_frames));
  const bool active_measured = (result_.active_left || result_.active_right) &&
      (!result_.active_left || measured[0]) && (!result_.active_right || measured[1]);
  const bool active_stopped = active_measured &&
      (!result_.active_left || stopped[0]) && (!result_.active_right || stopped[1]);
  const bool beam_stopped = beam_measured && reference_valid &&
      std::abs(result_.motion_dz) < config_.stop_dz_threshold &&
      (!config_.cumulative_motion_enabled || std::abs(result_.motion_dz) < config_.cumulative_direction_epsilon) &&
      std::abs(beam_z_filter_.vx() * dt - (effective_mode == "absolute" ? 0.0 : result_.dz_bg_filtered)) < config_.stop_dz_threshold;
  const CraneBeamState prev_state = result_.state;
  switch (result_.state) {
    case CraneBeamState::IDLE: {
      result_.work_complete = false;
      for (size_t side = 0; side < 2; ++side) {
        const auto & h = *hooks[side];
        const double net_rise = h.filtered_z - prepared_hook_z_[side] -
            (effective_mode == "absolute" ? 0.0 : result_.z_bg_filtered - prepared_hook_bg_z_[side]);
        const bool direction_consistent = hook_motion_evidence_[side].positive_frames >= 2;
        hook_lift_counters_[side] = result_.hook.prepare_ready && beam_up && up[side] &&
            prepared_hook_valid_[side] && net_rise >= config_.cumulative_lift_distance && direction_consistent ?
            hook_lift_counters_[side] + 1 : 0;
      }
      result_.lift_counter = std::max(hook_lift_counters_[0], hook_lift_counters_[1]);
      if (result_.lift_counter >= std::max(1, config_.lift_confirm_frames)) {
        result_.active_left = hook_lift_counters_[0] >= std::max(1, config_.lift_confirm_frames);
        result_.active_right = hook_lift_counters_[1] >= std::max(1, config_.lift_confirm_frames);
        result_.state = CraneBeamState::LIFTING;
        result_.lift_counter = result_.stop_counter = 0;
        vertical_motion_evidence_.reset();
        for (auto & e : hook_motion_evidence_) e.reset();
      }
      break;
    }
    case CraneBeamState::LIFTING: {
      // No direct reversal/skip. Stop must follow sufficient measured lifting.
      const bool lifted = result_.z_beam_filtered - beam_lock_z_ -
          (effective_mode == "absolute" ? 0.0 : result_.z_bg_filtered - beam_lock_bg_z_) >= config_.cumulative_lift_distance;
      result_.stop_counter = lifted && beam_stopped && active_stopped ? result_.stop_counter + 1 : 0;
      if (result_.stop_counter >= std::max(1, config_.suspend_confirm_frames)) {
        result_.state = CraneBeamState::SUSPENDED;
        result_.stop_counter = 0;
        vertical_motion_evidence_.reset();
        for (auto & e : hook_motion_evidence_) e.reset();
      }
      break;
    }
    case CraneBeamState::SUSPENDED: {
      const bool consistent = active_measured && (!result_.active_left || down[0]) && (!result_.active_right || down[1]);
      result_.lower_counter = beam_down && consistent ? result_.lower_counter + 1 : 0;
      if (result_.lower_counter >= std::max(1, config_.lower_confirm_frames)) {
        result_.state = CraneBeamState::LOWERING;
        result_.lower_counter = result_.placed_counter = 0;
        vertical_motion_evidence_.reset();
        has_prev_support_z_ = false;
      }
      break;
    }
    case CraneBeamState::LOWERING: {
      const bool near = active_measured && (!result_.active_left || near_ground[0]) && (!result_.active_right || near_ground[1]);
      result_.placed_counter = near && beam_stopped && active_stopped ? result_.placed_counter + 1 : 0;
      result_.placed_candidate = result_.placed_counter >= std::max(1, config_.placed_confirm_frames);
      if (result_.placed_candidate) {
        result_.state = CraneBeamState::COMPLETE_CONFIRM;
        result_.work_complete = latched_work_complete_ = true;
        result_.complete_counter = 0;
      }
      break;
    }
    case CraneBeamState::COMPLETE_CONFIRM: {
      result_.work_complete = true;
      // Background is primary. A one-frame spike, including its EMA recovery,
      // cannot reset a cycle: the confirmed displacement must keep its sign.
      const int direction = result_.dz_bg_filtered > 0.0 ? 1 : -1;
      const bool common_motion_measured = result_.crane_moving &&
          result_.dz_bg_raw * result_.dz_bg_filtered > 0.0 &&
          std::abs(result_.dz_bg_raw) > config_.cumulative_direction_epsilon;
      result_.complete_counter = common_motion_measured ?
          (direction == crane_move_direction_ ? result_.complete_counter + 1 : 1) : 0;
      crane_move_direction_ = common_motion_measured ? direction : 0;
      if (result_.complete_counter >= std::max(1, config_.complete_confirm_frames)) {
        reset();
      }
      break;
    }
  }
  result_.working_status = (result_.state == CraneBeamState::LIFTING ||
      result_.state == CraneBeamState::SUSPENDED || result_.state == CraneBeamState::LOWERING) ? 1 : 0;

  if (result_.state != prev_state) {
    result_.state_changed = true;
    result_.prev_state_str = to_string(prev_state);
    result_.curr_state_str = to_string(result_.state);
  }

  prev_stamp_sec_ = stamp_sec;
  has_prev_stamp_ = true;
  return result_;
}

}  // namespace cx128s2_bev_perception
