#include <unordered_map>
#include <cx128s2_bev_perception/crane_hook_z_density.hpp>
// Height Patch + centered fast plane fitting is dispatched independently from Slope.
// Rebased on the user's supplied CascadedGroundSeg::initRadiusTable /
// segmentGroundCpu / cbCloud (Guardian AI slope_segmentation, Apache-2.0).
// Retain cell representatives, prefix radius gaps, strict '<', and run Z span.
// Sensor angles come from the installed CX128S2 driver model, not fitted points.
// Apply mounting TF to that model; keep the source prefix-gap/run-height rules.
#include <cx128s2_bev_perception/crane_pipeline.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <queue>
#include <stdexcept>
#include <unordered_set>
#include <numeric>
#include <Eigen/Eigenvalues>
#include <map>
#include <set>
#include <iostream>

namespace cx128s2_bev_perception {
namespace {
constexpr double pi = 3.14159265358979323846;
// lslidar_ch_driver.hpp: big_angle_ch128s1, used by CX128S2 as well.
constexpr double cx_base_angles[32] = {
    -12,-11,-10,-9,-8,-7,-6,-5,-4.125,-4,-3.125,-3,-2.125,-2,-1.125,-1,
    -0.125,0,0.875,1,1.875,2,3,4,5,6,7,8,9,10,11,12};
constexpr double eps = 1e-6; // supplied code's denominator guard

static inline bool isBeamLikeNode(const CraneHookChainNode & n, const CraneDualRoiAnchorConfig & cfg) {
  const double dx = std::max(0.0, n.max_x - n.min_x);
  const double dy = std::max(0.0, n.max_y - n.min_y);
  const double len = std::max(dx, dy);
  const double wid = std::max(0.01, std::min(dx, dy));
  return (len >= cfg.beam_min_length_m && (len / wid) >= cfg.beam_min_aspect_ratio);
}

static inline bool isBeamLikeSubcluster(const CraneSubcluster & sc, const CraneDualRoiAnchorConfig & cfg) {
  const double dx = std::max(0.0, sc.max_x - sc.min_x);
  const double dy = std::max(0.0, sc.max_y - sc.min_y);
  const double len = std::max(dx, dy);
  const double wid = std::max(0.01, std::min(dx, dy));
  return (len >= cfg.beam_min_length_m && (len / wid) >= cfg.beam_min_aspect_ratio);
}
}

CranePipeline::CranePipeline(const CraneSlopeConfig & config) {
  updateConfig(config);
}

void CranePipeline::updateConfig(const CraneSlopeConfig & config) {
  const auto & w=config.work_status;
  if(!std::isfinite(w.virtual_hook_offset_z_m) || w.virtual_hook_offset_z_m<0)
    throw std::invalid_argument("Invalid virtual Hook offset");
  if(w.fsm_enabled && (!std::isfinite(config.exception_gate.min_length_m) || config.exception_gate.min_length_m<=0 ||
      !std::isfinite(config.exception_gate.min_area_m2) || config.exception_gate.min_area_m2<=0))
    throw std::invalid_argument("Invalid existing object-size limits for Cargo acquisition");
  for(double value:{w.boundary_start_confirm_sec,w.boundary_stop_confirm_sec,w.boundary_support_timeout_sec})
    if(!std::isfinite(value) || value<=0)throw std::invalid_argument("Invalid boundary temporal duration");
  if(w.boundary_support_timeout_sec<w.boundary_stop_confirm_sec)
    throw std::invalid_argument("Boundary support timeout must cover stop confirmation");
  if(config_.work_status.boundary_temporal_enabled!=w.boundary_temporal_enabled ||
      config_.work_status.boundary_start_confirm_sec!=w.boundary_start_confirm_sec ||
      config_.work_status.boundary_stop_confirm_sec!=w.boundary_stop_confirm_sec ||
      config_.work_status.boundary_support_timeout_sec!=w.boundary_support_timeout_sec)boundary_confirmation_.reset();
  if(!std::isfinite(w.lifting_event_hold_sec) || w.lifting_event_hold_sec<=0 ||
      !std::isfinite(w.lifting_clearance_delta_m) || w.lifting_clearance_delta_m<0 ||
      !std::isfinite(w.lifting_min_clearance_m) || w.lifting_min_clearance_m<0)
    throw std::invalid_argument("Invalid LIFTING projection parameter");
  if(w.fsm_enabled || w.boundary_motion_enabled){
  for(double value:{w.cargo_roi_width_x_m,w.cargo_roi_width_y_m,w.cargo_start_offset_m,w.cargo_confirm_sec,w.cargo_loss_sec,
      w.cargo_min_surface_width_m,w.cargo_min_fill_ratio,w.boundary_search_radius_m,
      w.grid_resolution_m,w.connect_xy_margin,w.connect_max_gap_z})
    if(!std::isfinite(value) || value<=0)throw std::invalid_argument("Invalid Work Status parameter");
  if(w.cargo_confirm_sec<2 || w.cargo_min_fill_ratio>1 || w.boundary_min_coherent_points<4 ||
      w.boundary_min_coherent_points>8 || w.connect_max_gap_z<=w.cargo_start_offset_m ||
      w.confirm_m_on<1 || w.confirm_m_off<1 || w.min_candidate_points<3)
    throw std::invalid_argument("Invalid Work Status limits");
  }
  if(config_.work_status.fsm_enabled!=w.fsm_enabled ||
      config_.work_status.boundary_motion_enabled!=w.boundary_motion_enabled ||
      config_.work_status.virtual_hook_offset_z_m!=w.virtual_hook_offset_z_m){
    left_work_state_=TrackedHookWorkState{};right_work_state_=TrackedHookWorkState{};
    work_cycle_=CraneWorkStatusResult{};
    drop_started_stamp_=cargo_peak_z_=gap_last_stamp_=gap_observed_sec_=0;resetDropCandidate();
    cargo_descent_steps_=gap_hits_=0;cargo_descent_seen_=cargo_stationary_current_=false;work_elapsed_current_=completion_last_stamp_=0;last_gap_candidate_="UNKNOWN";cargo_xy_filter_=LinearKalmanFilter{};cargo_z_filter_=LinearKalmanFilter{};cargo_owned_points_.clear();cargo_owned_stamp_=cargo_recovery_stamp_=-1;cargo_owned_id_=-1;cargo_recovery_indices_.clear();
    cargo_hits_=cargo_outside_hits_=0;cargo_last_seen_=cargo_observed_sec_=work_last_stamp_=0;
    boundary_filters_={};boundary_observations_={};boundary_hits_={};boundary_last_seen_={};boundary_moving_=false;boundary_move_count_=0;boundary_moving_points_.clear();boundary_confirmation_.reset();
  }

  if(config.height_mode) {
    const auto & h=config.height;
    for(double v:{h.patch_resolution,h.max_neighbor_dz,h.max_patch_z_spread,h.max_xy_gap,
        h.max_plane_rms,h.max_plane_tilt_deg,h.max_merge_angle_deg,h.max_merge_gap_z,
        h.min_surface_length,h.min_surface_width})
      if(!std::isfinite(v) || v<=0)throw std::invalid_argument("Invalid height-patch parameter");
    if(h.patch_resolution<.005 || h.max_xy_gap/h.patch_resolution>10 || h.min_patch_points<3 ||
        h.min_surface_points<h.min_patch_points || h.max_plane_tilt_deg>=60 || h.max_merge_angle_deg>=45 ||
        h.min_surface_width>h.min_surface_length)
      throw std::invalid_argument("Invalid height-patch limits");
    config_ = config;
    return;
  }
  if ((config.lidar_model != "cx128s2" && config.lidar_model != "uniform") ||
      (config.lidar_model == "cx128s2" && config.lidar_channels != 128) ||
      !std::isfinite(config.lidar_degree_start) ||
      !std::isfinite(config.lidar_degree_step) || config.lidar_degree_step <= 0 ||
      !std::isfinite(config.lidar_height) || config.lidar_height < 0 ||
      !std::isfinite(config.cx128s2_prism_offset_deg) ||
      config.cx128s2_prism_angles_deg.size() != 4 ||
      !std::all_of(config.cx128s2_prism_angles_deg.begin(), config.cx128s2_prism_angles_deg.end(),
          [](double a){ return std::isfinite(a) && std::abs(a) < 45; }) ||
      !std::isfinite(config.model_angle_tolerance_deg) || config.model_angle_tolerance_deg <= 0 ||
      config.model_angle_tolerance_deg > 5 ||
      !std::isfinite(config.horizontal_max_azimuth_gap_deg) ||
      config.horizontal_max_azimuth_gap_deg <= 0 || config.horizontal_max_azimuth_gap_deg > 5 ||
      config.azimuth_columns < 1 || config.azimuth_columns > 36000 ||
      config.lidar_channels < 1 || config.lidar_channels > 1024 ||
      static_cast<size_t>(config.azimuth_columns)*config.lidar_channels > 16000000 ||
      !std::isfinite(config.max_slope_deg) || config.max_slope_deg < 0 || config.max_slope_deg >= 45 ||
      !std::isfinite(config.max_vertical_height) || config.max_vertical_height <= 0 ||
      !std::isfinite(config.ground_z) ||
      std::isnan(config.force_ground_below_z) ||
      config.force_ground_below_z == std::numeric_limits<double>::infinity()) throw std::invalid_argument("Invalid Crane slope parameters");
  const bool accum_cfg_changed =
      (config_.temporal_stabilization.enabled != config.temporal_stabilization.enabled) ||
      (config_.temporal_stabilization.mode != config.temporal_stabilization.mode) ||
      (config_.temporal_stabilization.window_size != config.temporal_stabilization.window_size);
  config_ = config;
  if (accum_cfg_changed) {
    clearStationaryAccumulation();
  }
  for (int ring=0; ring<128; ++ring) {
    // Matches driver DIFOP handling, including even-group prism_offset.
    const double base = (cx_base_angles[ring/4] +
        (ring/4%2 == 0 ? config_.cx128s2_prism_offset_deg : 0.0))*pi/180;
    const double prism = config_.cx128s2_prism_angles_deg[ring%4]*pi/180;
    base_sin_[ring]=std::sin(base); base_cos_[ring]=std::cos(base);
    prism_sin_[ring]=std::sin(prism); prism_cos_[ring]=std::cos(prism);
  }
}

void CranePipeline::reset() {
  ordered_state_=CraneBeamState::IDLE;ordered_side_=ordered_sheave_id_=-1;ordered_stamp_=ordered_last_roi_=-1;
  ordered_completed_=-1;ordered_stop_count_=ordered_lower_count_=0;ordered_lower_accum_=0;ordered_last_seed_stamp_=-1;
  for(auto & samples:ordered_roi_samples_)samples.clear();ordered_roi_ids_={-1,-1};
  field_drop_elapsed_={0,0};field_drop_last_={-1,-1};field_policy_cargo_id_=-1;field_cargo_rise_.clear();
  clearHeightResult();patch_cells_.clear();height_patches_.clear();
  point_next_.clear();region_points_.clear();patch_edges_.clear();
  result_.state = CraneBeamState::IDLE;
  result_.ground.clear(); result_.non_ground.clear(); result_.unknown.clear();
  result_.clustering.clear(); result_.labels.clear();
  result_.observed_rings = result_.neighbor_pairs = result_.broken_pairs = 0;
  result_.overwritten_points = result_.unsupported_points = result_.model_mismatch_points = result_.cluster_count = 0;
  result_.workspace_bytes = 0; result_.segmentation_ms = 0; result_.pose_valid = false;
  horizontal_rays_.clear();
  result_.vertical_labels.clear(); result_.horizontal_labels.clear();
  result_.horizontal_neighbor_pairs = result_.horizontal_broken_pairs = 0;
  result_.horizontal_gap_breaks = result_.horizontal_unsupported_pairs = result_.horizontal_runs = 0;
  result_.z_gaussian.layers.clear(); result_.z_gaussian.point_layer_ids.clear();
  result_.z_gaussian.grouping_ms = 0.0; result_.z_gaussian.candidate_points = result_.z_gaussian.grouped_points = 0;
  result_.closed_loop.groups.clear(); result_.closed_loop.subclusters.clear();
  result_.closed_loop.point_group_ids.clear(); result_.closed_loop.point_subcluster_ids.clear();
  result_.closed_loop.background_z = 0.0; result_.closed_loop.dz_background = 0.0;
  result_.closed_loop.closed_loop_regions = 0; result_.closed_loop.grouping_ms = 0.0;
  samples_.clear(); depth_.clear(); rows_.clear(); radius_prefix_.clear();
  axes_.clear(); axis_states_.clear(); rings_.reset();
  tracked_z_layers_.clear(); next_stable_layer_id_ = 1;
  tracked_boundary_clusters_.clear(); next_boundary_cluster_id_ = 1;
  tracked_subclusters_.clear(); next_subcluster_id_ = 1;
  tracked_object_groups_.clear(); next_object_group_id_ = 1;
  result_.top_down_hook = CraneTopDownHookResult{};
  result_.hook = CraneHookResult{};
  result_.beam = CraneBeamGeometry{};
  result_.crane_moving = false;
  tracked_hook_chains_.clear();
  next_hook_chain_id_ = 1;
  has_last_hook_stamp_ = false;
  last_hook_stamp_sec_ = 0.0;
  beam_initialized_ = false;
  crane_moving_ = false;root_move_count_=0;
  for(int side=0;side<2;++side)resetSheaveMotion(side);
  sheave_motion_frame_.clear();
  lifting_cargo_id_=-1;lifting_last_stamp_=-1;lifting_event_until_=0;clearance_observations_.clear();
  crane_move_reason_ = "RESET";
  root_anchor_valid_ = false;
  root_anchor_x_ = 0.0;
  root_anchor_y_ = 0.0;
  root_stable_frames_ = 0;
  root_position_history_.clear();
  result_.work_status = CraneWorkStatusResult{};
  result_.working_status = 0;
  left_work_state_ = TrackedHookWorkState{};
  right_work_state_ = TrackedHookWorkState{};
  work_cycle_=CraneWorkStatusResult{};
    drop_started_stamp_=cargo_peak_z_=gap_last_stamp_=gap_observed_sec_=0;resetDropCandidate();
    cargo_descent_steps_=gap_hits_=0;cargo_descent_seen_=cargo_stationary_current_=false;work_elapsed_current_=completion_last_stamp_=0;last_gap_candidate_="UNKNOWN";cargo_xy_filter_=LinearKalmanFilter{};cargo_z_filter_=LinearKalmanFilter{};cargo_owned_points_.clear();cargo_owned_stamp_=cargo_recovery_stamp_=-1;cargo_owned_id_=-1;cargo_recovery_indices_.clear();
  boundary_filters_={};boundary_observations_={};boundary_last_seen_={};boundary_hits_={};
  work_last_stamp_=cargo_last_seen_=cargo_observed_sec_=0;cargo_hits_=cargo_outside_hits_=0;
  boundary_moving_=false;boundary_move_count_=0;boundary_moving_points_.clear();boundary_confirmation_.reset();boundary_stable_frames_=0;
  left_root_node_ = TrackedRootNode{};
  right_root_node_ = TrackedRootNode{};
  prev_hook_valid_ = false;
  prev_hook_x_ = 0.0;
  prev_hook_y_ = 0.0;
  prev_hook_z_ = 0.0;
  stabilized_beam_ = StabilizedBeamReference{};
  left_td_track_ = TrackedTopDownStructure{};
  right_td_track_ = TrackedTopDownStructure{};
  clearStationaryAccumulation();
}

const CraneSlopeResult & CranePipeline::process(const std::vector<PointXYZIRT> & points,
    const std::vector<int> & indices, const tf2::Transform & pose, bool ring_available, VisibilityModel * visibility) {
  visibility_model_ptr_ = visibility;
  if (config_.height_mode) return processHeight(points,indices,pose,visibility);
  const auto begin = std::chrono::steady_clock::now();
  result_.ground.clear(); result_.non_ground.clear(); result_.unknown.clear(); result_.clustering.clear();
  result_.closed_loop.groups.clear(); result_.closed_loop.subclusters.clear();
  result_.closed_loop.point_group_ids.clear(); result_.closed_loop.point_subcluster_ids.clear();
  result_.closed_loop.closed_loop_regions = 0; result_.closed_loop.grouping_ms = 0.0;
  result_.observed_rings = result_.neighbor_pairs = result_.broken_pairs = result_.cluster_count = 0;
  result_.overwritten_points = result_.unsupported_points = result_.model_mismatch_points = 0;
  samples_.clear(); horizontal_rays_.clear(); rings_.reset();
  result_.vertical_labels.clear(); result_.horizontal_labels.clear();
  result_.horizontal_neighbor_pairs = result_.horizontal_broken_pairs = 0;
  result_.horizontal_gap_breaks = result_.horizontal_unsupported_pairs = result_.horizontal_runs = 0;
  result_.z_gaussian.layers.clear(); result_.z_gaussian.point_layer_ids.clear();
  result_.z_gaussian.grouping_ms = 0.0; result_.z_gaussian.candidate_points = result_.z_gaussian.grouped_points = 0;
  result_.boundary_clustering = CraneBoundaryResult{};
  result_.labels.assign(points.size(), CraneSlopeLabel::UNKNOWN);
  auto & vertical_labels = config_.horizontal_enabled ? result_.vertical_labels : result_.labels;
  if (config_.horizontal_enabled) {
    vertical_labels.assign(points.size(), CraneSlopeLabel::UNKNOWN);
    result_.horizontal_labels.assign(points.size(), CraneSlopeLabel::UNKNOWN);
    horizontal_rays_.reserve(indices.size());
  }
  const double height = config_.lidar_height > 0 ? config_.lidar_height : pose.getOrigin().z() - config_.ground_z;
  result_.pose_valid = std::isfinite(height) && height > eps;
  for (int axis=0; axis<3; ++axis) {
    result_.pose_valid = result_.pose_valid && std::isfinite(pose.getOrigin()[axis]);
    for (int j=0; j<3; ++j)
      result_.pose_valid = result_.pose_valid && std::isfinite(pose.getBasis()[axis][j]);
  }
  const int width = config_.azimuth_columns, channels = config_.lidar_channels;
  depth_.assign(static_cast<size_t>(width)*channels, -1);
  axes_.resize(width); axis_states_.assign(width, 0);
  rows_.reserve(config_.horizontal_enabled ? std::max(width,channels) : channels);
  radius_prefix_.reserve(channels);
  samples_.reserve(indices.size());
  const auto inverse = pose.inverse();
  const tf2::Vector3 up(0,0,1), sensor_vertical = pose.getBasis()*up;
  const double beta = std::tan(config_.max_slope_deg*pi/180);

  for (int idx : indices) {
    if (idx < 0 || static_cast<size_t>(idx) >= points.size()) continue;
    const auto & p = points[idx];
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
    if (ring_available) rings_.set(p.ring);
    if (!result_.pose_valid || !ring_available || p.ring >= channels) {
      ++result_.unsupported_points; continue;
    }
    const auto raw = inverse*tf2::Vector3(p.x,p.y,p.z);
    const double range = raw.length();
    if (!(range > eps)) { ++result_.unsupported_points; continue; }
    double az = std::atan2(raw.y(), raw.x());
    if (az < 0) az += 2*pi;
    // Same column orientation and last-write-wins cell rule as supplied cbCloud.
    const int col = width-static_cast<int>(width*az/(2*pi))-1;
    if (col < 0 || col >= width) { ++result_.unsupported_points; continue; }
    const double center = (width-col-.5)*2*pi/width;
    if (axis_states_[col] == 0) {
      const auto horizontal = pose.getBasis()*tf2::Vector3(std::cos(center),std::sin(center),0);
      auto axis = horizontal.cross(sensor_vertical).cross(up);
      axis_states_[col] = -1;
      if (axis.length2() > eps*eps) {
        axis.normalize();
        const double orientation = axis.dot(sensor_vertical);
        if (orientation < -eps || (std::abs(orientation) <= eps && axis.dot(horizontal) < 0)) axis = -axis;
        axes_[col] = axis; axis_states_[col] = 1;
      }
    }
    // Build the source angle table from ring identity and the driver's optical
    // model. XYZ supplies measured distance/azimuth only, never channel angles.
    double elevation = (config_.lidar_degree_start +
        (channels-static_cast<int>(p.ring)-1)*config_.lidar_degree_step)*pi/180;
    if (config_.lidar_model == "cx128s2") {
      // Output XYZ azimuth is ALREADY corrected by convertCoordinate_cx128s2.
      // Its cos_list[int(angle_h*.5)] uses a half-angle rounded down to .01deg.
      const double half = std::floor(center*180/pi*50 + 1e-8)*.01*pi/180;
      const int r = p.ring;
      const double reflection = prism_cos_[r]*base_cos_[r]*std::cos(half)-prism_sin_[r]*base_sin_[r];
      const double sin_elevation = base_sin_[r]+2*reflection*prism_sin_[r];
      if (std::abs(sin_elevation) > 1) { ++result_.unsupported_points; continue; }
      elevation = std::asin(sin_elevation);
    }
    // An unknown DIFOP calibration must not silently turn into a wrong table.
    const double measured_elevation = std::atan2(raw.z(),std::hypot(raw.x(),raw.y()));
    if (std::abs(measured_elevation-elevation) > config_.model_angle_tolerance_deg*pi/180) {
      ++result_.model_mismatch_points; ++result_.unsupported_points; continue;
    }
    Sample sample; sample.index = idx; sample.ring = p.ring; sample.column = col;
    const auto ray = pose.getBasis()*tf2::Vector3(std::cos(elevation)*std::cos(center),
        std::cos(elevation)*std::sin(center),std::sin(elevation));
    if (axis_states_[col] > 0) {
      const double along = ray.dot(axes_[col]);
      sample.theta = std::atan2(ray.z(), along);
      // Measured position is compared against a MODEL-derived threshold.
      sample.coordinate = (tf2::Vector3(p.x,p.y,p.z)-pose.getOrigin()).dot(axes_[col]);
      sample.denominator = ray.z()+beta*along;
      if (ray.z() < -eps && std::abs(sample.denominator) >= eps) {
        sample.table = height*along/sample.denominator;
        sample.supported = std::isfinite(sample.table);
      }
    }
    if (!sample.supported) ++result_.unsupported_points;
    const int row = channels-static_cast<int>(p.ring)-1;
    auto & cell = depth_[static_cast<size_t>(col)*channels+row];
    if (cell >= 0) ++result_.overwritten_points;
    cell = static_cast<int>(samples_.size());
    samples_.push_back(sample);
    if (config_.horizontal_enabled) horizontal_rays_.push_back(ray);
    // Displaced returns remain UNKNOWN instead of disappearing from both outputs.
  }
  result_.observed_rings = rings_.count();

  for (int col=0; col<width; ++col) {
    rows_.clear();
    for (int row=0; row<channels; ++row) {
      const int sample = depth_[static_cast<size_t>(col)*channels+row];
      if (sample >= 0) rows_.push_back(sample);
    }
    // Supplied CPU implementation skips columns containing fewer than two cells.
    if (rows_.size() < 2) continue;
    // C32 row order encodes its angle LUT. Here ring + optical model + TF
    // determines order, independent of the measured surface heights.
    std::sort(rows_.begin(),rows_.end(),[&](int a,int b) {
      if (samples_[a].theta != samples_[b].theta) return samples_[a].theta < samples_[b].theta;
      return samples_[a].ring > samples_[b].ring;
    });

    radius_prefix_.assign(rows_.size(),0);
    for (size_t k=1; k<rows_.size(); ++k) {
      const auto & a = samples_[rows_[k-1]], & b = samples_[rows_[k]];
      const bool continuous = a.supported && b.supported && a.denominator*b.denominator > 0;
      // Actual nonuniform/missing-ring intervals replace C32's fixed alpha.
      // Telescope the same r0/prefix equation; no cone maximum or noise addition.
      radius_prefix_[k] = radius_prefix_[k-1] + (continuous ? a.table-b.table : 0);
    }
    const auto classify = [&](size_t first,size_t end) {
      float lo = points[samples_[rows_[first]].index].z, hi = lo;
      for (size_t k=first+1; k<end; ++k) {
        const float z = points[samples_[rows_[k]].index].z;
        lo = std::min(lo,z); hi = std::max(hi,z);
      }
      // Verbatim decision from the supplied CPU loop, in gravity-aligned Z.
      const auto label = hi-lo > config_.max_vertical_height ?
          CraneSlopeLabel::NON_GROUND : CraneSlopeLabel::GROUND;
      for (size_t k=first; k<end; ++k) vertical_labels[samples_[rows_[k]].index] = label;
    };
    size_t first = 0;
    for (size_t k=0; k<rows_.size(); ++k) {
      const auto & current = samples_[rows_[k]];
      if (!current.supported) {
        if (first < k) classify(first,k);
        first=k+1; continue;
      }
      if (first == k) continue;
      const auto & previous = samples_[rows_[k-1]];
      ++result_.neighbor_pairs;
      const float threshold = static_cast<float>(radius_prefix_[k]-radius_prefix_[k-1]);
      const bool join = previous.denominator*current.denominator > 0 &&
          std::abs(current.coordinate-previous.coordinate) < threshold;

      if (!join) { ++result_.broken_pairs; classify(first,k); first=k; }
    }
    if (first < rows_.size()) classify(first,rows_.size());
  }
  if (config_.horizontal_enabled) {
    // Same Ring, increasing sensor azimuth. CX128S2 has a partial FOV: never
    // connect the last column back to the first or bridge a large missing sector.
    const double max_gap = config_.horizontal_max_azimuth_gap_deg*pi/180;
    for (int row=0; row<channels; ++row) {
      rows_.clear();
      for (int col=width-1; col>=0; --col) {
        const int sample = depth_[static_cast<size_t>(col)*channels+row];
        if (sample >= 0) rows_.push_back(sample);
      }
      if (rows_.size() < 2) continue;
      const auto classify_horizontal = [&](size_t first,size_t end) {
        // One point contains no horizontal continuity evidence.
        if (end-first < 2) return;
        float lo=points[samples_[rows_[first]].index].z, hi=lo;
        for (size_t k=first+1; k<end; ++k) {
          const float z=points[samples_[rows_[k]].index].z;
          lo=std::min(lo,z); hi=std::max(hi,z);
        }
        const auto label=hi-lo > config_.max_vertical_height ?
            CraneSlopeLabel::NON_GROUND : CraneSlopeLabel::GROUND;
        for (size_t k=first; k<end; ++k)
          result_.horizontal_labels[samples_[rows_[k]].index]=label;
        ++result_.horizontal_runs;
      };
      size_t first=0;
      for (size_t k=1; k<rows_.size(); ++k) {
        const int ai=rows_[k-1], bi=rows_[k];
        const auto & a=samples_[ai]; const auto & b=samples_[bi];
        const double gap=(a.column-b.column)*2*pi/width;

        bool join=false;
        if (gap > max_gap+1e-12) {
          ++result_.horizontal_gap_breaks;
        } else {
          const auto & da=horizontal_rays_[ai]; const auto & db=horizontal_rays_[bi];
          // A same-ring scan is generally a cone, not a gravity-aligned fan.
          // Use the plane of THIS pair of model rays for the source radius LUT.
          // Both measured coordinates use that same signed horizontal axis.
          auto axis=da.cross(db).cross(up);
          if (da.z() >= -eps || db.z() >= -eps || axis.length2() <= eps*eps) {
            ++result_.horizontal_unsupported_pairs;
          } else {
            axis.normalize();
            const double middle=(width-(a.column+b.column)*.5-.5)*2*pi/width;
            const auto tangent=pose.getBasis()*tf2::Vector3(-std::sin(middle),std::cos(middle),0);
            // Source LUT assumes an outward horizontal radius. Orient the
            // pair axis toward the pair's mean horizontal viewing direction;
            // retain signed coordinates so a nadir crossing never folds.
            const double outward=axis.dot(da+db);
            if (outward < -eps || (std::abs(outward) <= eps && axis.dot(tangent) < 0)) axis=-axis;
            const double aa=da.dot(axis), ab=db.dot(axis);
            const double den_a=da.z()+beta*aa, den_b=db.z()+beta*ab;
            if (std::abs(den_a) < eps || std::abs(den_b) < eps || den_a*den_b <= 0) {
              ++result_.horizontal_unsupported_pairs;
            } else {
              const double fa=height*aa/den_a, fb=height*ab/den_b;
              const float threshold=static_cast<float>(std::atan2(da.z(),aa) < std::atan2(db.z(),ab) ? fa-fb : fb-fa);
              const auto & pa=points[a.index]; const auto & pb=points[b.index];
              const double delta=tf2::Vector3(pb.x-pa.x,pb.y-pa.y,pb.z-pa.z).dot(axis);
              ++result_.horizontal_neighbor_pairs;
              join=std::abs(delta) < threshold; // same strict source inequality
              if (!join) ++result_.horizontal_broken_pairs;
            }
          }
        }

        if (!join) { classify_horizontal(first,k); first=k; }
      }
      classify_horizontal(first,rows_.size());
    }
    // Two-direction evidence: either NON_GROUND wins; GROUND requires both.
    // Unobserved/overwritten points remain UNKNOWN. No plane fit is introduced.
    for (int idx : indices) {
      if (idx < 0 || static_cast<size_t>(idx) >= points.size()) continue;
      const auto v=vertical_labels[idx], h=result_.horizontal_labels[idx];
      result_.labels[idx]=(v==CraneSlopeLabel::NON_GROUND || h==CraneSlopeLabel::NON_GROUND) ?
          CraneSlopeLabel::NON_GROUND : (v==CraneSlopeLabel::GROUND && h==CraneSlopeLabel::GROUND) ?
          CraneSlopeLabel::GROUND : CraneSlopeLabel::UNKNOWN;
    }
  }
  // Post-classification only: leave the slope table, adjacency and runs intact.
  // Reuse the partition pass so labels, RGB and index outputs stay consistent.
  const bool force_ground = std::isfinite(config_.force_ground_below_z);
  for (int idx : indices) {
    if (idx < 0 || static_cast<size_t>(idx) >= points.size()) continue;
    const auto & p = points[idx];
    if (force_ground && std::isfinite(p.x) && std::isfinite(p.y) &&
        std::isfinite(p.z) && p.z <= config_.force_ground_below_z)
      result_.labels[idx] = CraneSlopeLabel::GROUND;
    const auto label = result_.labels[idx];
    if (label == CraneSlopeLabel::GROUND) {
      result_.ground.push_back(idx);
      if (config_.include_ground_in_clustering) result_.clustering.push_back(idx);
    } else {
      (label == CraneSlopeLabel::NON_GROUND ? result_.non_ground : result_.unknown).push_back(idx);
      result_.clustering.push_back(idx);
    }
  }
  if (config_.temporal_stabilization.enabled) {
    updateAccumulatedPoints(points, indices, 0.0);
  }
  if (config_.boundary_clustering.enabled) {
    processBoundaryClustering(points, pose);
  }
  if (config_.z_gaussian.enabled) {
    processZGaussianGrouping(points, pose);
  }
  result_.workspace_bytes = samples_.capacity()*sizeof(Sample)+rings_.size()/8+
      horizontal_rays_.capacity()*sizeof(tf2::Vector3)+
      (result_.vertical_labels.capacity()+result_.horizontal_labels.capacity())*sizeof(CraneSlopeLabel)+
      (depth_.capacity()+rows_.capacity())*sizeof(int)+radius_prefix_.capacity()*sizeof(double)+
      axes_.capacity()*sizeof(tf2::Vector3)+axis_states_.capacity()*sizeof(int8_t)+
      result_.labels.capacity()*sizeof(CraneSlopeLabel)+(result_.ground.capacity()+
      result_.non_ground.capacity()+result_.unknown.capacity()+result_.clustering.capacity())*sizeof(int);
  result_.segmentation_ms = std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();

  return result_;
}

void CranePipeline::processBoundaryClustering(const std::vector<PointXYZIRT> & points, const tf2::Transform & /*pose*/) {
  const auto t_begin = std::chrono::steady_clock::now();
  auto & out = result_.boundary_clustering;
  const auto & bcfg = config_.boundary_clustering;

  out.boundary_types.assign(points.size(), CraneBoundaryType::NONE);
  out.boundary_indices.clear();
  out.height_step_indices.clear();
  out.slope_indices.clear();
  out.range_jump_indices.clear();
  out.clusters.clear();
  out.cluster_labels.assign(points.size(), -1);
  out.severed_edges = 0;
  out.total_tested_edges = 0;

  if (samples_.empty()) return;

  const int width = config_.azimuth_columns;
  const int channels = config_.lidar_channels;
  const double slope_thresh_rad = bcfg.slope_threshold_deg * pi / 180.0;
  const double max_neighbor_d2 = bcfg.max_neighbor_distance * bcfg.max_neighbor_distance;
  const double max_gap = config_.horizontal_max_azimuth_gap_deg * pi / 180.0;
  const bool has_force_ground = std::isfinite(config_.force_ground_below_z);

  std::vector<std::vector<int>> adjacency(samples_.size());
  const auto eval_begin = std::chrono::steady_clock::now();

  const auto is_confirmed_ground = [&](int pt_idx) {
    const auto & p = points[pt_idx];
    if (has_force_ground && p.z <= config_.force_ground_below_z) return true;
    if (result_.labels[pt_idx] == CraneSlopeLabel::GROUND &&
        p.z <= config_.ground_z + bcfg.min_height_above_ground) {
      return true;
    }
    return false;
  };

  const auto evaluate_edge = [&](int sa, int sb) {
    const int ia = samples_[sa].index;
    const int ib = samples_[sb].index;
    const bool ga = is_confirmed_ground(ia);
    const bool gb = is_confirmed_ground(ib);

    if (ga && gb) return;

    ++out.total_tested_edges;
    const auto & pa = points[ia];
    const auto & pb = points[ib];

    const double dx = pb.x - pa.x;
    const double dy = pb.y - pa.y;
    const double dz = pb.z - pa.z;
    const double dxy2 = dx * dx + dy * dy;
    const double d3d2 = dxy2 + dz * dz;

    const double dxy = std::sqrt(dxy2);
    const double height_step = std::abs(dz);
    const double slope = dxy > 1e-4 ? std::atan2(height_step, dxy) : (pi * 0.5);

    const double ra = std::hypot(pa.x, pa.y, pa.z);
    const double rb = std::hypot(pb.x, pb.y, pb.z);
    const double range_jump = std::abs(rb - ra);

    CraneBoundaryType btype = CraneBoundaryType::NONE;

    if (d3d2 > max_neighbor_d2) {
      btype = btype | CraneBoundaryType::RANGE_JUMP;
    }
    if (height_step > bcfg.height_step_threshold) {
      btype = btype | CraneBoundaryType::HEIGHT_STEP;
    }
    if (slope > slope_thresh_rad && height_step > bcfg.height_step_threshold * 0.5) {
      btype = btype | CraneBoundaryType::SLOPE_DISCONTINUITY;
    }
    if (range_jump > bcfg.range_jump_threshold) {
      btype = btype | CraneBoundaryType::RANGE_JUMP;
    }
    if ((ga && !gb) || (gb && !ga)) {
      btype = btype | CraneBoundaryType::GROUND_NEIGHBOR;
    }

    if (btype != CraneBoundaryType::NONE) {
      ++out.severed_edges;
      if (!ga) {
        out.boundary_types[ia] = out.boundary_types[ia] | btype;
      }
      if (!gb) {
        out.boundary_types[ib] = out.boundary_types[ib] | btype;
      }
    } else {
      adjacency[sa].push_back(sb);
      adjacency[sb].push_back(sa);
    }
  };

  for (int col = 0; col < width; ++col) {
    for (int row = 0; row < channels; ++row) {
      const int sa = depth_[static_cast<size_t>(col) * channels + row];
      if (sa < 0) continue;

      if (row + 1 < channels) {
        const int sb = depth_[static_cast<size_t>(col) * channels + (row + 1)];
        if (sb >= 0) evaluate_edge(sa, sb);
      }

      const int next_col = (col + 1) % width;
      const double gap = (next_col > col ? (next_col - col) : (next_col + width - col)) * 2 * pi / width;
      if (gap <= max_gap + 1e-6) {
        const int sb = depth_[static_cast<size_t>(next_col) * channels + row];
        if (sb >= 0) evaluate_edge(sa, sb);
      }
    }
  }

  for (size_t i = 0; i < points.size(); ++i) {
    const auto bt = out.boundary_types[i];
    if (bt == CraneBoundaryType::NONE) continue;
    out.boundary_indices.push_back(static_cast<int>(i));
    if (hasBoundaryFlag(bt, CraneBoundaryType::HEIGHT_STEP)) out.height_step_indices.push_back(static_cast<int>(i));
    if (hasBoundaryFlag(bt, CraneBoundaryType::SLOPE_DISCONTINUITY)) out.slope_indices.push_back(static_cast<int>(i));
    if (hasBoundaryFlag(bt, CraneBoundaryType::RANGE_JUMP)) out.range_jump_indices.push_back(static_cast<int>(i));
  }

  out.boundary_detection_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - eval_begin).count();

  if (bcfg.enable_morphological_closing && bcfg.morph_closing_radius_m > 0.01) {
    std::unordered_set<int64_t> boundary_cells;
    const double res = 0.05;
    const auto to_key = [res](double x, double y) {
      int64_t ix = static_cast<int64_t>(std::floor(x / res));
      int64_t iy = static_cast<int64_t>(std::floor(y / res));
      return (ix << 32) ^ (iy & 0xffffffff);
    };
    for (int idx : out.boundary_indices) {
      boundary_cells.insert(to_key(points[idx].x, points[idx].y));
    }
    const int r_cells = std::max(1, static_cast<int>(std::round(bcfg.morph_closing_radius_m / res)));
    std::unordered_set<int64_t> dilated;
    for (int64_t key : boundary_cells) {
      int32_t ix = static_cast<int32_t>(key >> 32);
      int32_t iy = static_cast<int32_t>(key & 0xffffffff);
      for (int dx = -r_cells; dx <= r_cells; ++dx) {
        for (int dy = -r_cells; dy <= r_cells; ++dy) {
          if (dx * dx + dy * dy <= r_cells * r_cells) {
            dilated.insert((static_cast<int64_t>(ix + dx) << 32) ^ (static_cast<int64_t>(iy + dy) & 0xffffffff));
          }
        }
      }
    }
    std::unordered_set<int64_t> closed;
    for (int64_t key : dilated) {
      int32_t ix = static_cast<int32_t>(key >> 32);
      int32_t iy = static_cast<int32_t>(key & 0xffffffff);
      bool all_present = true;
      for (int dx = -r_cells; dx <= r_cells && all_present; ++dx) {
        for (int dy = -r_cells; dy <= r_cells; ++dy) {
          if (dx * dx + dy * dy <= r_cells * r_cells) {
            if (!dilated.count((static_cast<int64_t>(ix + dx) << 32) ^ (static_cast<int64_t>(iy + dy) & 0xffffffff))) {
              all_present = false; break;
            }
          }
        }
      }
      if (all_present) closed.insert(key);
    }
    for (size_t sa = 0; sa < adjacency.size(); ++sa) {
      auto & nbrs = adjacency[sa];
      const auto & pa = points[samples_[sa].index];
      nbrs.erase(std::remove_if(nbrs.begin(), nbrs.end(), [&](int sb) {
        const auto & pb = points[samples_[sb].index];
        const int64_t mid_key = to_key(0.5 * (pa.x + pb.x), 0.5 * (pa.y + pb.y));
        if (closed.count(mid_key) && !boundary_cells.count(to_key(pa.x, pa.y)) && !boundary_cells.count(to_key(pb.x, pb.y))) {
          ++out.severed_edges;
          return true;
        }
        return false;
      }), nbrs.end());
    }
  }

  const auto cluster_begin = std::chrono::steady_clock::now();
  std::vector<bool> visited(samples_.size(), false);
  std::vector<int> current_cluster_pts;
  int current_cluster_id = 0;

  for (size_t s_idx = 0; s_idx < samples_.size(); ++s_idx) {
    if (visited[s_idx]) continue;
    const int pt_idx = samples_[s_idx].index;
    if (is_confirmed_ground(pt_idx)) continue;
    if (!bcfg.include_unknown && result_.labels[pt_idx] == CraneSlopeLabel::UNKNOWN) continue;

    std::queue<int> q;
    q.push(static_cast<int>(s_idx));
    visited[s_idx] = true;
    current_cluster_pts.clear();

    while (!q.empty()) {
      const int curr = q.front();
      q.pop();
      current_cluster_pts.push_back(samples_[curr].index);

      for (int nxt : adjacency[curr]) {
        if (visited[nxt]) continue;
        const int nxt_pt_idx = samples_[nxt].index;
        if (is_confirmed_ground(nxt_pt_idx)) continue;
        if (!bcfg.include_unknown && result_.labels[nxt_pt_idx] == CraneSlopeLabel::UNKNOWN) continue;

        visited[nxt] = true;
        q.push(nxt);
      }
    }

    if (current_cluster_pts.size() >= static_cast<size_t>(bcfg.min_cluster_points) &&
        current_cluster_pts.size() <= static_cast<size_t>(bcfg.max_cluster_points)) {
      CraneCluster cl;
      cl.id = current_cluster_id++;
      cl.point_indices = current_cluster_pts;
      cl.min_x = cl.min_y = cl.min_z = std::numeric_limits<double>::infinity();
      cl.max_x = cl.max_y = cl.max_z = -std::numeric_limits<double>::infinity();
      double sum_x = 0, sum_y = 0, sum_z = 0;
      for (int idx : current_cluster_pts) {
        const auto & cp = points[idx];
        cl.min_x = std::min(cl.min_x, static_cast<double>(cp.x));
        cl.max_x = std::max(cl.max_x, static_cast<double>(cp.x));
        cl.min_y = std::min(cl.min_y, static_cast<double>(cp.y));
        cl.max_y = std::max(cl.max_y, static_cast<double>(cp.y));
        cl.min_z = std::min(cl.min_z, static_cast<double>(cp.z));
        cl.max_z = std::max(cl.max_z, static_cast<double>(cp.z));
        sum_x += cp.x; sum_y += cp.y; sum_z += cp.z;
      }
      const double inv_n = 1.0 / current_cluster_pts.size();
      cl.centroid_x = sum_x * inv_n;
      cl.centroid_y = sum_y * inv_n;
      cl.centroid_z = sum_z * inv_n;
      cl.length = std::max(0.01, cl.max_x - cl.min_x);
      cl.width = std::max(0.01, cl.max_y - cl.min_y);
      cl.height = std::max(0.01, cl.max_z - cl.min_z);
      out.clusters.push_back(std::move(cl));
    }
  }

  // Stabilize boundary cluster IDs across frames so shaking points do not flicker colors
  {
    const size_t prev_track_count = tracked_boundary_clusters_.size();
    std::vector<uint8_t> track_used(prev_track_count, 0);
    for (auto & cl : out.clusters) {
      int best_t = -1;
      double best_dist = std::numeric_limits<double>::infinity();
      for (size_t t = 0; t < prev_track_count; ++t) {
        if (track_used[t]) continue;
        const auto & tr = tracked_boundary_clusters_[t];
        const double dxy = std::hypot(cl.centroid_x - tr.cx, cl.centroid_y - tr.cy);
        const double dz = std::abs(cl.centroid_z - tr.cz);
        if (dxy <= 0.85 && dz <= 0.45) {
          const double d3 = std::hypot(dxy, dz);
          if (d3 < best_dist) {
            best_dist = d3;
            best_t = static_cast<int>(t);
          }
        }
      }
      if (best_t >= 0) {
        track_used[best_t] = 1;
        auto & tr = tracked_boundary_clusters_[best_t];
        cl.id = tr.stable_id;
        tr.cx = 0.7 * tr.cx + 0.3 * cl.centroid_x;
        tr.cy = 0.7 * tr.cy + 0.3 * cl.centroid_y;
        tr.cz = 0.7 * tr.cz + 0.3 * cl.centroid_z;
        tr.missed_frames = 0;
      } else {
        cl.id = next_boundary_cluster_id_++;
        tracked_boundary_clusters_.push_back(
            TrackedClusterState{cl.id, cl.centroid_x, cl.centroid_y, cl.centroid_z, 0});
      }
      for (int idx : cl.point_indices) {
        out.cluster_labels[idx] = cl.id;
      }
    }
    for (size_t t = prev_track_count; t > 0; --t) {
      const size_t idx = t - 1;
      if (!track_used[idx]) {
        if (++tracked_boundary_clusters_[idx].missed_frames > 25) {
          tracked_boundary_clusters_.erase(tracked_boundary_clusters_.begin() + idx);
        }
      }
    }
  }

  out.region_growing_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - cluster_begin).count();
  out.total_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t_begin).count();
}

void CranePipeline::processZGaussianGrouping(const std::vector<PointXYZIRT> & points, const tf2::Transform & pose,
    const std::vector<int> * selected_raw,CraneZGaussianResult * diagnostic) {
  const auto t_begin = std::chrono::steady_clock::now();
  auto & out = diagnostic ? *diagnostic : result_.z_gaussian;
  const auto & zcfg = config_.z_gaussian;

  const auto & pts = (!diagnostic && has_accumulated_points_ && !accumulated_points_.empty()) ? accumulated_points_ : points;

  out.layers.clear();
  if(diagnostic)out.point_layer_ids.clear();else out.point_layer_ids.assign(pts.size(), -1);
  out.candidate_points = 0;
  out.grouped_points = 0;
  out.grouping_ms = 0.0;

  if (pts.empty()) return;

  const bool has_force_ground = std::isfinite(config_.force_ground_below_z);
  const double min_z_thresh = config_.ground_z + zcfg.min_height_above_ground;
  std::vector<int> candidates;
  // Diagnostic callers supply the complete candidate list and never use this bitmap.
  std::vector<uint8_t> selected(selected_raw && diagnostic ? 0 : pts.size(), 0);

  if (selected_raw) {
    candidates=*selected_raw; // already filtered current Raw indices; never accumulated indices
  } else if (has_accumulated_points_ && !accumulated_points_.empty()) {
    candidates.reserve(accumulated_points_.size());
    for (size_t idx = 0; idx < accumulated_points_.size(); ++idx) {
      const auto & p = accumulated_points_[idx];
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
      if (config_.roi_filter.enabled && !config_.roi_filter.inActiveRoi(p.x, p.y, p.z)) continue;
      if (has_force_ground && p.z <= config_.force_ground_below_z) continue;
      if (zcfg.min_height_above_ground > 0.0 && p.z <= min_z_thresh) continue;
      candidates.push_back(static_cast<int>(idx));
      selected[idx] = 1;
    }
  } else {
    candidates.reserve(result_.clustering.size());
    for (int idx : result_.clustering) {
      if (idx < 0 || static_cast<size_t>(idx) >= points.size()) continue;
      const auto & p = points[idx];
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
      if (config_.roi_filter.enabled && !config_.roi_filter.inActiveRoi(p.x, p.y, p.z)) continue;
      if (has_force_ground && p.z <= config_.force_ground_below_z) continue;
      if (!config_.include_ground_in_clustering && zcfg.min_height_above_ground > 0.0 &&
          p.z <= min_z_thresh && result_.labels[idx] != CraneSlopeLabel::NON_GROUND) {
        continue;
      }
      candidates.push_back(idx);
      selected[idx] = 1;
    }
  }

  // Also include elevated flat surfaces that slope segmentation may have labelled GROUND due to zero intra-run Z spread
  if (!diagnostic && !has_accumulated_points_ && !config_.include_ground_in_clustering && zcfg.min_height_above_ground > 0.0) {
    for (const auto & s : samples_) {
      const int idx = s.index;
      if (selected[idx]) continue;
      const auto & p = points[idx];
      if (config_.roi_filter.enabled && !config_.roi_filter.inActiveRoi(p.x, p.y, p.z)) continue;
      if (has_force_ground && p.z <= config_.force_ground_below_z) continue;
      if (p.z > min_z_thresh) {
        candidates.push_back(idx);
        selected[idx] = 1;
      }
    }
  }

  out.candidate_points = candidates.size();
  if (candidates.empty()) return;

  // Scratch only: cache the same float Z sort key to avoid random point-cloud
  // loads on every comparison. No layer threshold, arithmetic or index order changes.
  // One buffer per existing processing thread; this does not create a worker.
  static thread_local std::vector<std::pair<float,int>> z_sort_keys;
  z_sort_keys.clear();z_sort_keys.reserve(candidates.size());
  for(int idx:candidates)z_sort_keys.emplace_back(pts[idx].z,idx);
  std::sort(z_sort_keys.begin(),z_sort_keys.end(),[](const auto& a,const auto& b){return a.first<b.first;});
  for(size_t i=0;i<candidates.size();++i)candidates[i]=z_sort_keys[i].second;

  // 1D Gaussian Kernel Density profile over [z_min, z_max]
  const double sigma_base = std::max(0.002, zcfg.sigma_z);
  const double bin_w = std::max(0.005, 0.5 * sigma_base);
  const double z_min_all = pts[candidates.front()].z;
  const double z_max_all = pts[candidates.back()].z;
  const int num_bins = std::clamp(
      static_cast<int>(std::floor((z_max_all - z_min_all) / bin_w)) + 1, 1, 20000);
  std::vector<double> raw_hist(num_bins, 0.0);
  for (int idx : candidates) {
    const int b = std::clamp(
        static_cast<int>(std::floor((pts[idx].z - z_min_all) / bin_w)), 0, num_bins - 1);
    raw_hist[b] += 1.0;
  }

  const int k_radius = std::clamp(static_cast<int>(std::ceil(3.0 * sigma_base / bin_w)), 1, 64);
  std::vector<double> kernel(2 * k_radius + 1, 0.0);
  for (int d = -k_radius; d <= k_radius; ++d) {
    const double dz = d * bin_w;
    kernel[d + k_radius] = std::exp(-0.5 * (dz * dz) / (sigma_base * sigma_base));
  }
  std::vector<double> smooth_hist(num_bins, 0.0);
  for (int b = 0; b < num_bins; ++b) {
    double acc = 0.0;
    for (int d = -k_radius; d <= k_radius; ++d) {
      const int nb = b + d;
      if (nb >= 0 && nb < num_bins) {
        acc += raw_hist[nb] * kernel[d + k_radius];
      }
    }
    smooth_hist[b] = acc;
  }
  std::vector<uint8_t> is_valley(num_bins, 0);
  for (int b = 1; b + 1 < num_bins; ++b) {
    if (smooth_hist[b] < smooth_hist[b - 1] && smooth_hist[b] <= smooth_hist[b + 1]) {
      is_valley[b] = 1;
    }
  }

  const double sx = pose.getOrigin().x();
  const double sy = pose.getOrigin().y();
  const double k_conf = std::max(0.5, zcfg.confidence_sigma_multiplier);
  const double k_step = std::max(0.5, zcfg.step_sigma_multiplier);
  const double max_spread = std::max(0.02, zcfg.max_layer_z_spread);
  const int min_pts = std::max(1, zcfg.min_layer_points);

  // Helper to check if both cur_mean and z belong to the same tracked layer from the previous frame (temporal hysteresis)
  const auto in_same_tracked_layer = [&](double z1, double z2, double sigma_eff) {
    const double hysteresis_half_band = std::max(k_conf * sigma_eff * 1.35, 0.5 * max_spread);
    if (diagnostic && diagnostic!=&cargo_extension_layers_) return false;
    for (const auto & tr : tracked_z_layers_) {
      if (std::abs(z1 - tr.mean_z) <= hysteresis_half_band &&
          std::abs(z2 - tr.mean_z) <= hysteresis_half_band) {
        return true;
      }
    }
    return false;
  };

  std::vector<int> cur_indices;
  double cur_mean = 0.0;
  double cur_m2 = 0.0;
  double cur_min_z = 0.0;
  double cur_max_z = 0.0;
  double cur_peak_density = 0.0;
  double cur_min_valley_after_peak = std::numeric_limits<double>::infinity();
  int prev_bin = 0;

  const auto finalize_layer = [&]() {
    if (static_cast<int>(cur_indices.size()) >= min_pts) {
      CraneZGaussianLayer layer;
      layer.id = static_cast<int>(out.layers.size());
      layer.point_indices = cur_indices;
      layer.mean_z = cur_mean;
      layer.std_z = cur_indices.size() > 1 ? std::sqrt(cur_m2 / cur_indices.size()) : 0.0;
      layer.min_z = cur_min_z;
      layer.max_z = cur_max_z;
      out.grouped_points += layer.point_indices.size();
      out.layers.push_back(std::move(layer));
    }
    cur_indices.clear();
    cur_mean = 0.0;
    cur_m2 = 0.0;
    cur_peak_density = 0.0;
    cur_min_valley_after_peak = std::numeric_limits<double>::infinity();
  };

  for (size_t i = 0; i < candidates.size(); ++i) {
    const int idx = candidates[i];
    const auto & p = pts[idx];
    const double z = p.z;
    const int cur_bin = std::clamp(
        static_cast<int>(std::floor((z - z_min_all) / bin_w)), 0, num_bins - 1);
    const double cur_dens = smooth_hist[cur_bin];

    if (cur_indices.empty()) {
      cur_indices.push_back(idx);
      cur_mean = z;
      cur_m2 = 0.0;
      cur_min_z = z;
      cur_max_z = z;
      cur_peak_density = cur_dens;
      cur_min_valley_after_peak = std::numeric_limits<double>::infinity();
      prev_bin = cur_bin;
      continue;
    }

    for (int b = prev_bin; b <= cur_bin; ++b) {
      if (smooth_hist[b] > cur_peak_density) {
        cur_peak_density = smooth_hist[b];
        cur_min_valley_after_peak = std::numeric_limits<double>::infinity();
      } else if (is_valley[b]) {
        cur_min_valley_after_peak = std::min(cur_min_valley_after_peak, smooth_hist[b]);
      }
    }

    const double r_xy = std::hypot(p.x - sx, p.y - sy);
    const double sigma_local = sigma_base + std::max(0.0, zcfg.range_sigma_factor) * r_xy;
    const double sample_std = cur_indices.size() > 1 ? std::sqrt(cur_m2 / cur_indices.size()) : 0.0;
    const double sigma_eff = std::hypot(sigma_local, std::min(sample_std, sigma_local));

    const double z_prev = pts[cur_indices.back()].z;
    const bool tracked_hold = in_same_tracked_layer(cur_mean, z, sigma_eff);
    const double hyst_scale = tracked_hold ? 1.35 : 1.0;

    const bool step_ok = (z - z_prev) <= (k_step * sigma_local * hyst_scale);
    const bool band_ok = std::abs(z - cur_mean) <= (k_conf * sigma_eff * hyst_scale);
    const bool spread_ok = (z - cur_min_z) <= (max_spread * (tracked_hold ? 1.15 : 1.0));
    const bool valley_crossed = !tracked_hold &&
                                std::isfinite(cur_min_valley_after_peak) &&
                                cur_min_valley_after_peak < 0.40 * cur_peak_density &&
                                cur_dens > cur_min_valley_after_peak &&
                                (z - cur_mean) > 1.8 * sigma_local;

    if (!step_ok || !band_ok || !spread_ok || valley_crossed) {
      finalize_layer();
      cur_indices.push_back(idx);
      cur_mean = z;
      cur_m2 = 0.0;
      cur_min_z = z;
      cur_max_z = z;
      cur_peak_density = cur_dens;
      cur_min_valley_after_peak = std::numeric_limits<double>::infinity();
      prev_bin = cur_bin;
    } else {
      cur_indices.push_back(idx);
      const double count = static_cast<double>(cur_indices.size());
      const double delta = z - cur_mean;
      cur_mean += delta / count;
      cur_m2 += delta * (z - cur_mean);
      cur_max_z = std::max(cur_max_z, z);
      prev_bin = cur_bin;
    }
  }
  finalize_layer();

  // Only the upstream ROI grouping owns Housing/Wire layer memory.
  if (diagnostic) {
    // Diagnostic layers retain explicit Raw indices; no full-cloud ID array or tracked layer mutation.
  } else {
    const size_t prev_track_count = tracked_z_layers_.size();
    std::vector<uint8_t> track_used(prev_track_count, 0);
    const double match_gate_z = std::max(0.25, max_spread);
    for (auto & layer : out.layers) {
      int best_t = -1;
      double best_dz = std::numeric_limits<double>::infinity();
      for (size_t t = 0; t < prev_track_count; ++t) {
        if (track_used[t]) continue;
        const double dz = std::abs(layer.mean_z - tracked_z_layers_[t].mean_z);
        if (dz <= match_gate_z && dz < best_dz) {
          best_dz = dz;
          best_t = static_cast<int>(t);
        }
      }
      if (best_t >= 0) {
        track_used[best_t] = 1;
        auto & tr = tracked_z_layers_[best_t];
        layer.id = tr.stable_id;
        tr.mean_z = 0.75 * tr.mean_z + 0.25 * layer.mean_z;
        tr.std_z = 0.75 * tr.std_z + 0.25 * layer.std_z;
        tr.missed_frames = 0;
      } else {
        layer.id = next_stable_layer_id_++;
        tracked_z_layers_.push_back(TrackedZLayer{layer.id, layer.mean_z, layer.std_z, 0});
      }
      for (int idx : layer.point_indices) {
        out.point_layer_ids[idx] = layer.id;
      }
    }
    for (size_t t = prev_track_count; t > 0; --t) {
      const size_t idx = t - 1;
      if (!track_used[idx]) {
        if (++tracked_z_layers_[idx].missed_frames > 25) {
          tracked_z_layers_.erase(tracked_z_layers_.begin() + idx);
        }
      }
    }
  }

  out.grouping_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t_begin).count();
}

const CraneClosedLoopResult & CranePipeline::processClosedLoopGrouping(
    const std::vector<PointXYZIRT> & points,
    const std::vector<CraneSubcluster> & raw_subclusters,
    double external_bg_z,
    double external_dz_bg) {
  const auto t_begin = std::chrono::steady_clock::now();
  auto & out = result_.closed_loop;
  const auto & cfg = config_.closed_loop;

  out.groups.clear();
  out.subclusters.clear();
  out.point_group_ids.assign(points.size(), -1);
  out.point_subcluster_ids.assign(points.size(), -1);
  out.closed_loop_regions = 0;
  out.grouping_ms = 0.0;

  // 1. Estimate background Z and frame-to-frame dz_background
  double bg_z = config_.ground_z;
  if (std::isfinite(external_bg_z)) {
    bg_z = external_bg_z;
    out.background_z = bg_z;
    if (std::isfinite(external_dz_bg)) {
      out.dz_background = external_dz_bg;
    } else if (prev_background_z_valid_) {
      out.dz_background = bg_z - prev_background_z_;
    } else {
      out.dz_background = 0.0;
    }
    prev_background_z_valid_ = true;
    prev_background_z_ = bg_z;
  } else {
    if (!result_.ground.empty()) {
      double sum_z = 0.0;
      size_t cnt = 0;
      for (int idx : result_.ground) {
        if (idx >= 0 && static_cast<size_t>(idx) < points.size()) {
          const double z = points[idx].z;
          if (std::isfinite(z) && std::abs(z - config_.ground_z) <= 1.0) {
            sum_z += z;
            ++cnt;
          }
        }
      }
      if (cnt > 0) {
        bg_z = sum_z / static_cast<double>(cnt);
      }
    }
    out.background_z = bg_z;
    if (prev_background_z_valid_) {
      out.dz_background = bg_z - prev_background_z_;
    } else {
      out.dz_background = 0.0;
      prev_background_z_valid_ = true;
    }
    prev_background_z_ = bg_z;
  }

  // 2. Sanitize subclusters and compute exact bounding boxes / centroids
  out.subclusters.reserve(raw_subclusters.size());
  for (const auto & raw : raw_subclusters) {
    if (raw.point_indices.empty()) continue;
    CraneSubcluster sub = raw;
    double sx = 0.0, sy = 0.0, sz = 0.0;
    double min_x = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    double min_z = std::numeric_limits<double>::infinity();
    double max_z = -std::numeric_limits<double>::infinity();
    size_t valid_pts = 0;
    for (int idx : sub.point_indices) {
      if (idx < 0 || static_cast<size_t>(idx) >= points.size()) continue;
      const auto & p = points[idx];
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
      sx += p.x; sy += p.y; sz += p.z;
      min_x = std::min<double>(min_x, p.x); max_x = std::max<double>(max_x, p.x);
      min_y = std::min<double>(min_y, p.y); max_y = std::max<double>(max_y, p.y);
      min_z = std::min<double>(min_z, p.z); max_z = std::max<double>(max_z, p.z);
      ++valid_pts;
    }
    if (valid_pts == 0) continue;
    const double inv_n = 1.0 / static_cast<double>(valid_pts);
    sub.centroid_x = sx * inv_n;
    sub.centroid_y = sy * inv_n;
    sub.centroid_z = sz * inv_n;
    sub.min_x = min_x; sub.max_x = max_x;
    sub.min_y = min_y; sub.max_y = max_y;
    sub.min_z = min_z; sub.max_z = max_z;
    out.subclusters.push_back(std::move(sub));
  }

  // 3. Track subclusters across frames and compute relative vertical motion:
  //    dz_relative,i = dz_i - dz_background
  {
    const size_t prev_track_count = tracked_subclusters_.size();
    std::vector<uint8_t> track_used(prev_track_count, 0);
    const double xy_gate = std::max(0.05, cfg.subcluster_match_xy_gate);
    const double z_gate = std::max(0.05, cfg.subcluster_match_z_gate);

    for (auto & sub : out.subclusters) {
      int best_t = -1;
      double best_score = std::numeric_limits<double>::infinity();
      for (size_t t = 0; t < prev_track_count; ++t) {
        if (track_used[t]) continue;
        const auto & tr = tracked_subclusters_[t];
        const double dxy = std::hypot(sub.centroid_x - tr.cx, sub.centroid_y - tr.cy);
        const double dz_abs = std::abs(sub.centroid_z - tr.cz);
        if (dxy <= xy_gate && dz_abs <= z_gate) {
          const double layer_penalty = (sub.layer_id >= 0 && tr.layer_id == sub.layer_id) ? 0.0 : 0.15;
          const double score = dxy + 0.5 * dz_abs + layer_penalty;
          if (score < best_score) {
            best_score = score;
            best_t = static_cast<int>(t);
          }
        }
      }
      if (best_t >= 0) {
        track_used[best_t] = 1;
        auto & tr = tracked_subclusters_[best_t];
        sub.subcluster_id = tr.stable_id;
        sub.prev_z = tr.cz;
        sub.dz = sub.centroid_z - tr.cz;
        sub.dz_relative = sub.dz - out.dz_background;
        sub.cumulative_dz_relative = tr.cumulative_dz_relative + sub.dz_relative;
        sub.tracked_frames = tr.tracked_frames + 1;
        sub.motion_valid = true;

        tr.layer_id = sub.layer_id;
        tr.cx = 0.65 * tr.cx + 0.35 * sub.centroid_x;
        tr.cy = 0.65 * tr.cy + 0.35 * sub.centroid_y;
        tr.cz = sub.centroid_z;
        tr.cumulative_dz_relative = sub.cumulative_dz_relative;
        tr.tracked_frames = sub.tracked_frames;
        tr.missed_frames = 0;
      } else {
        sub.subcluster_id = next_subcluster_id_++;
        sub.prev_z = sub.centroid_z;
        sub.dz = 0.0;
        sub.dz_relative = 0.0;
        sub.cumulative_dz_relative = 0.0;
        sub.tracked_frames = 1;
        sub.motion_valid = false;
        tracked_subclusters_.push_back(TrackedSubclusterState{
            sub.subcluster_id, sub.layer_id,
            sub.centroid_x, sub.centroid_y, sub.centroid_z,
            0.0, 1, 0});
      }
      for (int idx : sub.point_indices) {
        if (idx >= 0 && static_cast<size_t>(idx) < out.point_subcluster_ids.size()) {
          out.point_subcluster_ids[idx] = sub.subcluster_id;
        }
      }
    }

    for (size_t t = prev_track_count; t > 0; --t) {
      const size_t idx = t - 1;
      if (!track_used[idx]) {
        if (++tracked_subclusters_[idx].missed_frames > 25) {
          tracked_subclusters_.erase(tracked_subclusters_.begin() + idx);
        }
      }
    }
  }

  if (out.subclusters.empty()) {
    out.grouping_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t_begin).count();
    return out;
  }

  // 4. 2D XY Closed Loop Detection & Containment Grouping:
  //    Group(C, K_i) = Inside(K_i, Omega_C)
  //    No 3D continuity or Z gap restriction is used during initial grouping.
  double global_min_x = std::numeric_limits<double>::infinity();
  double global_max_x = -std::numeric_limits<double>::infinity();
  double global_min_y = std::numeric_limits<double>::infinity();
  double global_max_y = -std::numeric_limits<double>::infinity();
  for (const auto & sub : out.subclusters) {
    global_min_x = std::min(global_min_x, sub.min_x);
    global_max_x = std::max(global_max_x, sub.max_x);
    global_min_y = std::min(global_min_y, sub.min_y);
    global_max_y = std::max(global_max_y, sub.max_y);
  }

  double res = std::max(0.02, cfg.grid_resolution_m);
  const double span_x = std::max(0.1, global_max_x - global_min_x);
  const double span_y = std::max(0.1, global_max_y - global_min_y);
  // Keep 2D grid within CPU-friendly bounds (<= 1200 x 1200)
  if (span_x / res > 1200.0 || span_y / res > 1200.0) {
    res = std::max(res, std::max(span_x, span_y) / 1200.0);
  }

  const int closing_r = std::clamp(
      static_cast<int>(std::round(std::max(0.0, cfg.closing_radius_m) / res)), 0, 10);
  const int pad = closing_r + 2;
  const int grid_w = static_cast<int>(std::ceil(span_x / res)) + 2 * pad + 1;
  const int grid_h = static_cast<int>(std::ceil(span_y / res)) + 2 * pad + 1;
  const size_t total_cells = static_cast<size_t>(grid_w) * static_cast<size_t>(grid_h);

  const auto to_col = [&](double x) -> int {
    return std::clamp(static_cast<int>(std::floor((x - global_min_x) / res)) + pad, 0, grid_w - 1);
  };
  const auto to_row = [&](double y) -> int {
    return std::clamp(static_cast<int>(std::floor((y - global_min_y) / res)) + pad, 0, grid_h - 1);
  };

  std::vector< std::pair<int, int> > disk_offsets;
  for (int dr = -closing_r; dr <= closing_r; ++dr) {
    for (int dc = -closing_r; dc <= closing_r; ++dc) {
      if (dr * dr + dc * dc <= closing_r * closing_r) {
        disk_offsets.push_back({dr, dc});
      }
    }
  }
  if (disk_offsets.empty()) {
    disk_offsets.push_back({0, 0});
  }

  // Helper: given an occupancy grid, run 2D morphological closing + exterior BFS flood-fill,
  // and mark any closed loop region (wall + enclosed cavity) into closed_loop_mask.
  std::vector<uint8_t> dilated(total_cells, 0);
  std::vector<uint8_t> closed_occ(total_cells, 0);
  std::vector<uint8_t> exterior(total_cells, 0);
  std::vector<int> comp_ids(total_cells, -1);
  std::vector<int> bfs_queue;
  bfs_queue.reserve(total_cells);

  std::vector<uint8_t> closed_loop_mask(total_cells, 0);
  std::vector<uint8_t> cavity_cell_mask(total_cells, 0);
  const int min_enclosed = std::max(1, cfg.min_enclosed_cells);

  const auto detect_closed_loops_on_grid = [&](const std::vector<uint8_t> & occ_grid,
                                               int r0, int r1, int c0, int c1) {
    r0 = std::max(0, r0 - pad);
    r1 = std::min(grid_h - 1, r1 + pad);
    c0 = std::max(0, c0 - pad);
    c1 = std::min(grid_w - 1, c1 + pad);
    if (r1 - r0 < 2 || c1 - c0 < 2) return;

    for (int r = r0; r <= r1; ++r) {
      const size_t row_off = static_cast<size_t>(r) * grid_w;
      std::fill(dilated.begin() + row_off + c0, dilated.begin() + row_off + c1 + 1, 0);
      std::fill(closed_occ.begin() + row_off + c0, closed_occ.begin() + row_off + c1 + 1, 0);
      std::fill(exterior.begin() + row_off + c0, exterior.begin() + row_off + c1 + 1, 0);
      std::fill(comp_ids.begin() + row_off + c0, comp_ids.begin() + row_off + c1 + 1, -1);
    }

    for (int r = r0; r <= r1; ++r) {
      const size_t row_off = static_cast<size_t>(r) * grid_w;
      for (int c = c0; c <= c1; ++c) {
        if (!occ_grid[row_off + c]) continue;
        for (const auto & off : disk_offsets) {
          const int nr = r + off.first;
          const int nc = c + off.second;
          if (nr >= r0 && nr <= r1 && nc >= c0 && nc <= c1) {
            dilated[static_cast<size_t>(nr) * grid_w + nc] = 1;
          }
        }
      }
    }

    for (int r = r0; r <= r1; ++r) {
      const size_t row_off = static_cast<size_t>(r) * grid_w;
      for (int c = c0; c <= c1; ++c) {
        if (occ_grid[row_off + c]) {
          closed_occ[row_off + c] = 1;
          continue;
        }
        if (!dilated[row_off + c]) continue;
        bool all_in = true;
        for (const auto & off : disk_offsets) {
          const int nr = r + off.first;
          const int nc = c + off.second;
          if (nr < r0 || nr > r1 || nc < c0 || nc > c1 ||
              !dilated[static_cast<size_t>(nr) * grid_w + nc]) {
            all_in = false;
            break;
          }
        }
        if (all_in) {
          closed_occ[row_off + c] = 1;
        }
      }
    }

    // 4-connected BFS from sub-window border to mark EXTERIOR cells
    bfs_queue.clear();
    const auto try_seed = [&](int r, int c) {
      const size_t idx = static_cast<size_t>(r) * grid_w + c;
      if (!closed_occ[idx] && !exterior[idx]) {
        exterior[idx] = 1;
        bfs_queue.push_back(static_cast<int>(idx));
      }
    };
    for (int c = c0; c <= c1; ++c) {
      try_seed(r0, c);
      try_seed(r1, c);
    }
    for (int r = r0 + 1; r < r1; ++r) {
      try_seed(r, c0);
      try_seed(r, c1);
    }
    static constexpr int dr4[4] = {-1, 1, 0, 0};
    static constexpr int dc4[4] = {0, 0, -1, 1};
    for (size_t qi = 0; qi < bfs_queue.size(); ++qi) {
      const int cur = bfs_queue[qi];
      const int r = cur / grid_w;
      const int c = cur % grid_w;
      for (int d = 0; d < 4; ++d) {
        const int nr = r + dr4[d];
        const int nc = c + dc4[d];
        if (nr < r0 || nr > r1 || nc < c0 || nc > c1) continue;
        const size_t nidx = static_cast<size_t>(nr) * grid_w + nc;
        if (!closed_occ[nidx] && !exterior[nidx]) {
          exterior[nidx] = 1;
          bfs_queue.push_back(static_cast<int>(nidx));
        }
      }
    }

    // Find 8-connected components of (!exterior) within [r0..r1, c0..c1].
    for (int r = r0; r <= r1; ++r) {
      const size_t row_off = static_cast<size_t>(r) * grid_w;
      for (int c = c0; c <= c1; ++c) {
        const size_t idx = row_off + c;
        if (exterior[idx] || comp_ids[idx] >= 0) continue;
        bfs_queue.clear();
        bfs_queue.push_back(static_cast<int>(idx));
        comp_ids[idx] = 1;
        int cavity_count = 0;
        for (size_t qi = 0; qi < bfs_queue.size(); ++qi) {
          const int cur = bfs_queue[qi];
          if (!closed_occ[cur]) {
            ++cavity_count;
          }
          const int cr = cur / grid_w;
          const int cc = cur % grid_w;
          for (int dr = -1; dr <= 1; ++dr) {
            for (int dc = -1; dc <= 1; ++dc) {
              if (dr == 0 && dc == 0) continue;
              const int nr = cr + dr;
              const int nc = cc + dc;
              if (nr < r0 || nr > r1 || nc < c0 || nc > c1) continue;
              const size_t nidx = static_cast<size_t>(nr) * grid_w + nc;
              if (!exterior[nidx] && comp_ids[nidx] < 0) {
                comp_ids[nidx] = 1;
                bfs_queue.push_back(static_cast<int>(nidx));
              }
            }
          }
        }
        if (cavity_count >= min_enclosed) {
          for (int cell_idx : bfs_queue) {
            closed_loop_mask[cell_idx] = 1;
            if (!closed_occ[cell_idx]) {
              cavity_cell_mask[cell_idx] = 1;
            }
          }
        }
      }
    }
  };

  // Rasterize each subcluster (and per-Z-layer occupancy) so that even if an upper layer covers
  // a lower layer's hole in 2D XY, the lower layer's closed loop Omega_C is still detected!
  std::vector<uint8_t> combined_occ(total_cells, 0);
  std::vector<uint8_t> single_occ(total_cells, 0);
  std::vector<int> unique_layers;
  for (const auto & sub : out.subclusters) {
    if (std::find(unique_layers.begin(), unique_layers.end(), sub.layer_id) == unique_layers.end()) {
      unique_layers.push_back(sub.layer_id);
    }
    for (int idx : sub.point_indices) {
      const auto & p = points[idx];
      combined_occ[static_cast<size_t>(to_row(p.y)) * grid_w + to_col(p.x)] = 1;
    }
  }
  if (cfg.use_boundary_points_for_loop && !result_.boundary_clustering.boundary_indices.empty()) {
    for (int idx : result_.boundary_clustering.boundary_indices) {
      if (idx < 0 || static_cast<size_t>(idx) >= points.size()) continue;
      const auto & p = points[idx];
      if (p.x >= global_min_x && p.x <= global_max_x &&
          p.y >= global_min_y && p.y <= global_max_y) {
        combined_occ[static_cast<size_t>(to_row(p.y)) * grid_w + to_col(p.x)] = 1;
      }
    }
  }

  // Run closed-loop detection on combined XY occupancy
  detect_closed_loops_on_grid(combined_occ, 0, grid_h - 1, 0, grid_w - 1);

  // Also run per-Z-layer closed-loop detection within each layer's local XY bounding box
  if (unique_layers.size() > 1) {
    for (int lid : unique_layers) {
      int r0 = grid_h, r1 = -1, c0 = grid_w, c1 = -1;
      for (const auto & sub : out.subclusters) {
        if (sub.layer_id != lid) continue;
        r0 = std::min(r0, to_row(sub.min_y));
        r1 = std::max(r1, to_row(sub.max_y));
        c0 = std::min(c0, to_col(sub.min_x));
        c1 = std::max(c1, to_col(sub.max_x));
      }
      if (r1 - r0 < 2 || c1 - c0 < 2) continue;
      for (const auto & sub : out.subclusters) {
        if (sub.layer_id != lid) continue;
        for (int idx : sub.point_indices) {
          const auto & p = points[idx];
          single_occ[static_cast<size_t>(to_row(p.y)) * grid_w + to_col(p.x)] = 1;
        }
      }
      detect_closed_loops_on_grid(single_occ, r0, r1, c0, c1);
      for (const auto & sub : out.subclusters) {
        if (sub.layer_id != lid) continue;
        for (int idx : sub.point_indices) {
          const auto & p = points[idx];
          single_occ[static_cast<size_t>(to_row(p.y)) * grid_w + to_col(p.x)] = 0;
        }
      }
    }
  }
  // Also check each individual subcluster's local 2D footprint for closed loops
  if (out.subclusters.size() > 1) {
    for (const auto & sub : out.subclusters) {
      if (static_cast<int>(sub.point_indices.size()) < min_enclosed * 2) continue;
      const int r0 = to_row(sub.min_y);
      const int r1 = to_row(sub.max_y);
      const int c0 = to_col(sub.min_x);
      const int c1 = to_col(sub.max_x);
      if (r1 - r0 < 2 || c1 - c0 < 2) continue;
      for (int idx : sub.point_indices) {
        const auto & p = points[idx];
        single_occ[static_cast<size_t>(to_row(p.y)) * grid_w + to_col(p.x)] = 1;
      }
      detect_closed_loops_on_grid(single_occ, r0, r1, c0, c1);
      for (int idx : sub.point_indices) {
        const auto & p = points[idx];
        single_occ[static_cast<size_t>(to_row(p.y)) * grid_w + to_col(p.x)] = 0;
      }
    }
  }

  // Extract connected components of closed_loop_mask: each component is a Closed Loop domain Omega_c
  std::vector<int> loop_domain_id(total_cells, -1);
  std::vector<int> domain_cavity_counts;
  int num_domains = 0;
  for (size_t idx = 0; idx < total_cells; ++idx) {
    if (!closed_loop_mask[idx] || loop_domain_id[idx] >= 0) continue;
    const int dom_id = num_domains++;
    int cav_cnt = 0;
    bfs_queue.clear();
    bfs_queue.push_back(static_cast<int>(idx));
    loop_domain_id[idx] = dom_id;
    for (size_t qi = 0; qi < bfs_queue.size(); ++qi) {
      const int cur = bfs_queue[qi];
      if (cavity_cell_mask[cur]) ++cav_cnt;
      const int r = cur / grid_w;
      const int c = cur % grid_w;
      for (int dr = -1; dr <= 1; ++dr) {
        for (int dc = -1; dc <= 1; ++dc) {
          if (dr == 0 && dc == 0) continue;
          const int nr = r + dr;
          const int nc = c + dc;
          if (nr < 0 || nr >= grid_h || nc < 0 || nc >= grid_w) continue;
          const size_t nidx = static_cast<size_t>(nr) * grid_w + nc;
          if (closed_loop_mask[nidx] && loop_domain_id[nidx] < 0) {
            loop_domain_id[nidx] = dom_id;
            bfs_queue.push_back(static_cast<int>(nidx));
          }
        }
      }
    }
    domain_cavity_counts.push_back(cav_cnt);
  }
  out.closed_loop_regions = static_cast<size_t>(num_domains);
  out.detected_closed_loops = out.closed_loop_regions;

  // 5. Evaluate Closed Loop Containment for each subcluster K_i:
  //    Group(C, K_i) = Inside(K_i, Omega_C)
  const double min_ratio = std::clamp(cfg.min_containment_ratio, 0.05, 1.0);
  std::vector<int> subcluster_domain(out.subclusters.size(), -1);
  std::vector< std::vector<size_t> > domain_members(num_domains);

  if (num_domains > 0) {
    std::vector<int> dom_hits(num_domains, 0);
    for (size_t si = 0; si < out.subclusters.size(); ++si) {
      const auto & sub = out.subclusters[si];
      std::fill(dom_hits.begin(), dom_hits.end(), 0);
      for (int idx : sub.point_indices) {
        const auto & p = points[idx];
        const int dom = loop_domain_id[static_cast<size_t>(to_row(p.y)) * grid_w + to_col(p.x)];
        if (dom >= 0) {
          ++dom_hits[dom];
        }
      }
      int best_dom = -1;
      int best_hits = 0;
      for (int d = 0; d < num_domains; ++d) {
        if (dom_hits[d] > best_hits) {
          best_hits = dom_hits[d];
          best_dom = d;
        }
      }
      const double ratio = static_cast<double>(best_hits) /
                           static_cast<double>(std::max<size_t>(1, sub.point_indices.size()));
      const int centroid_dom =
          loop_domain_id[static_cast<size_t>(to_row(sub.centroid_y)) * grid_w + to_col(sub.centroid_x)];
      if (best_dom >= 0 && (ratio >= min_ratio || centroid_dom == best_dom)) {
        subcluster_domain[si] = best_dom;
        domain_members[best_dom].push_back(si);
      } else if (centroid_dom >= 0) {
        subcluster_domain[si] = centroid_dom;
        domain_members[centroid_dom].push_back(si);
      }
    }
  }

  // Build CraneObjectGroups:
  // - One unified group per non-empty Closed Loop domain Omega_c
  // - One standalone group per subcluster not inside any Closed Loop domain
  const auto build_group_from_members = [&](const std::vector<size_t> & member_indices,
                                            bool is_loop, int cavity_cells) {
    CraneObjectGroup grp;
    grp.is_closed_loop = is_loop;
    grp.enclosed_cavity_cells = cavity_cells;
    double sx = 0.0, sy = 0.0, sz = 0.0;
    grp.min_x = std::numeric_limits<double>::infinity();
    grp.max_x = -std::numeric_limits<double>::infinity();
    grp.min_y = std::numeric_limits<double>::infinity();
    grp.max_y = -std::numeric_limits<double>::infinity();
    grp.min_z = std::numeric_limits<double>::infinity();
    grp.max_z = -std::numeric_limits<double>::infinity();

    for (size_t si : member_indices) {
      const auto & sub = out.subclusters[si];
      grp.subcluster_ids.push_back(sub.subcluster_id);
      grp.subclusters.push_back(sub);
      grp.min_x = std::min(grp.min_x, sub.min_x);
      grp.max_x = std::max(grp.max_x, sub.max_x);
      grp.min_y = std::min(grp.min_y, sub.min_y);
      grp.max_y = std::max(grp.max_y, sub.max_y);
      grp.min_z = std::min(grp.min_z, sub.min_z);
      grp.max_z = std::max(grp.max_z, sub.max_z);
      for (int idx : sub.point_indices) {
        grp.merged_point_indices.push_back(idx);
        const auto & p = points[idx];
        sx += p.x; sy += p.y; sz += p.z;
      }
    }
    if (!grp.merged_point_indices.empty()) {
      const double inv_n = 1.0 / static_cast<double>(grp.merged_point_indices.size());
      grp.centroid_x = sx * inv_n;
      grp.centroid_y = sy * inv_n;
      grp.centroid_z = sz * inv_n;
      out.groups.push_back(std::move(grp));
    }
  };

  for (int d = 0; d < num_domains; ++d) {
    if (!domain_members[d].empty()) {
      build_group_from_members(domain_members[d], true, domain_cavity_counts[d]);
    }
  }
  for (size_t si = 0; si < out.subclusters.size(); ++si) {
    if (subcluster_domain[si] < 0) {
      build_group_from_members({si}, false, 0);
    }
  }

  // 6. Stabilize group_id across frames using tracked_object_groups_
  {
    const size_t prev_group_tracks = tracked_object_groups_.size();
    std::vector<uint8_t> track_used(prev_group_tracks, 0);
    for (auto & grp : out.groups) {
      int best_t = -1;
      double best_dist = std::numeric_limits<double>::infinity();
      for (size_t t = 0; t < prev_group_tracks; ++t) {
        if (track_used[t]) continue;
        const auto & tr = tracked_object_groups_[t];
        const double dxy = std::hypot(grp.centroid_x - tr.cx, grp.centroid_y - tr.cy);
        const double dz = std::abs(grp.centroid_z - tr.cz);
        if (dxy <= 0.85 && dz <= 0.85 && (dxy + 0.5 * dz) < best_dist) {
          best_dist = dxy + 0.5 * dz;
          best_t = static_cast<int>(t);
        }
      }
      if (best_t >= 0) {
        track_used[best_t] = 1;
        auto & tr = tracked_object_groups_[best_t];
        grp.group_id = tr.stable_id;
        tr.cx = 0.70 * tr.cx + 0.30 * grp.centroid_x;
        tr.cy = 0.70 * tr.cy + 0.30 * grp.centroid_y;
        tr.cz = 0.70 * tr.cz + 0.30 * grp.centroid_z;
        tr.missed_frames = 0;
      } else {
        grp.group_id = next_object_group_id_++;
        tracked_object_groups_.push_back(TrackedClusterState{
            grp.group_id, grp.centroid_x, grp.centroid_y, grp.centroid_z, 0});
      }

      for (auto & sub : grp.subclusters) {
        sub.group_id = grp.group_id;
        for (auto & global_sub : out.subclusters) {
          if (global_sub.subcluster_id == sub.subcluster_id) {
            global_sub.group_id = grp.group_id;
            break;
          }
        }
      }
      for (int idx : grp.merged_point_indices) {
        if (idx >= 0 && static_cast<size_t>(idx) < out.point_group_ids.size()) {
          out.point_group_ids[idx] = grp.group_id;
        }
      }
    }

    for (size_t t = prev_group_tracks; t > 0; --t) {
      const size_t idx = t - 1;
      if (!track_used[idx]) {
        if (++tracked_object_groups_[idx].missed_frames > 25) {
          tracked_object_groups_.erase(tracked_object_groups_.begin() + idx);
        }
      }
    }
  }

  out.grouping_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t_begin).count();
  return out;
}

void CranePipeline::PlaneMoments::add(const PointXYZIRT & p) {
  const std::array<double,3> value{{p.x,p.y,p.z}};
  std::array<double,3> delta{};
  ++count;
  for(int i=0;i<3;++i) {delta[i]=value[i]-mean[i];mean[i]+=delta[i]/count;}
  scatter[0]+=delta[0]*(value[0]-mean[0]);
  scatter[1]+=delta[0]*(value[1]-mean[1]);
  scatter[2]+=delta[0]*(value[2]-mean[2]);
  scatter[3]+=delta[1]*(value[1]-mean[1]);
  scatter[4]+=delta[1]*(value[2]-mean[2]);
  scatter[5]+=delta[2]*(value[2]-mean[2]);
}
void CranePipeline::PlaneMoments::combine(const PlaneMoments & other) {
  if(!other.count)return;
  if(!count) {*this=other;return;}
  const double n=count+other.count,w=static_cast<double>(count)*other.count/n;
  std::array<double,3> d{};
  for(int i=0;i<3;++i)d[i]=other.mean[i]-mean[i];
  const std::array<double,6> correction{{d[0]*d[0],d[0]*d[1],d[0]*d[2],d[1]*d[1],d[1]*d[2],d[2]*d[2]}};
  for(int i=0;i<6;++i)scatter[i]+=other.scatter[i]+w*correction[i];
  for(int i=0;i<3;++i)mean[i]+=d[i]*other.count/n;
  count+=other.count;
}
bool CranePipeline::fitPlane(const PlaneMoments & m,FastPlane & plane) {
  if(m.count<3)return false;
  const auto & s=m.scatter;
  const double trace=s[0]+s[3],det=s[0]*s[3]-s[1]*s[1];
  // A 2D XY footprint is necessary. Reject lines/duplicates without inventing a normal.
  if(!(trace>1e-12) || !(det>1e-10*trace*trace))return false;
  plane.a=(s[2]*s[3]-s[4]*s[1])/det;
  plane.b=(s[4]*s[0]-s[2]*s[1])/det;
  plane.center=m.mean;plane.c=m.mean[2]-plane.a*m.mean[0]-plane.b*m.mean[1];
  const double norm=std::sqrt(1+plane.a*plane.a+plane.b*plane.b);
  plane.normal={{-plane.a/norm,-plane.b/norm,1/norm}};
  const double residual=s[5]-plane.a*s[2]-plane.b*s[4];
  plane.rms=std::sqrt(std::max(0.0,residual)/(m.count*norm*norm));
  return std::isfinite(plane.a) && std::isfinite(plane.b) && std::isfinite(plane.c) && std::isfinite(plane.rms);
}
int CranePipeline::patchRoot(int i) {
  int root=i;
  while(height_patches_[root].parent!=root)root=height_patches_[root].parent;
  while(i!=root) {const int next=height_patches_[i].parent;height_patches_[i].parent=root;i=next;}
  return root;
}
void CranePipeline::clearHeightResult() {
  auto & h=result_.height;
  h.surface_ids.clear();h.patch_ids.clear();h.candidates.clear();h.surface_points.clear();
  h.unassigned.clear();h.excluded.clear();h.surfaces.clear();
  h.patch_count=h.valid_patch_count=h.spatial_cells=h.neighbor_tests=h.merge_attempts=h.merged_edges=h.occlusion_flagged_points=0;
  h.visibility_ms=h.patch_ms=h.fitting_ms=h.merging_ms=h.finalization_ms=h.total_ms=0;
}
const CraneSlopeResult & CranePipeline::processHeight(const std::vector<PointXYZIRT> & points,
    const std::vector<int> & indices,const tf2::Transform & pose,VisibilityModel * visibility) {
  const auto begin=std::chrono::steady_clock::now();clearHeightResult();
  auto & out=result_.height;const auto & cfg=config_.height;
  const auto elapsed=[](const auto & start) {
    return std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
  };
  result_.state=CraneBeamState::IDLE;result_.cluster_count=0;result_.workspace_bytes=0;
  result_.segmentation_ms=0;result_.pose_valid=true;
  for(int a=0;a<3;++a) {
    result_.pose_valid=result_.pose_valid && std::isfinite(pose.getOrigin()[a]);
    for(int b=0;b<3;++b)result_.pose_valid=result_.pose_valid && std::isfinite(pose.getBasis()[a][b]);
  }
  if(!cfg.enabled || !result_.pose_valid) {out.excluded=indices;out.total_ms=elapsed(begin);return result_;}
  if(!visibility)throw std::invalid_argument("Height patches require the configured VisibilityModel");
  visibility->setPose(pose);visibility->buildDepth(points,{});out.visibility_ms=elapsed(begin);
  const auto patch_begin=std::chrono::steady_clock::now();
  patch_cells_.clear();height_patches_.clear();patch_edges_.clear();region_points_.clear();
  patch_cells_.reserve(indices.size()/8+1);
  point_next_.assign(points.size(),-1);out.surface_ids.assign(points.size(),-1);out.patch_ids.assign(points.size(),-1);
  const auto & fov=visibility->config();const double gap2=cfg.max_xy_gap*cfg.max_xy_gap;
  for(int idx:indices) {
    if(idx<0 || static_cast<size_t>(idx)>=points.size())continue;
    const auto & p=points[idx];
    if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)) {out.excluded.push_back(idx);continue;}
    const auto info=visibility->evaluatePoint(p.x,p.y,p.z);
    if(!std::isfinite(info.sensor_distance) || info.sensor_distance<fov.min_range_m ||
        info.sensor_distance>fov.max_range_m || info.azimuth_deg<fov.azimuth_min_deg ||
        info.azimuth_deg>fov.azimuth_max_deg || info.elevation_deg<fov.elevation_min_deg ||
        info.elevation_deg>fov.elevation_max_deg) {out.excluded.push_back(idx);continue;}
    const double qx=std::floor(p.x/cfg.patch_resolution),qy=std::floor(p.y/cfg.patch_resolution);
    if(std::abs(qx)>static_cast<double>(INT64_MAX/4) || std::abs(qy)>static_cast<double>(INT64_MAX/4)) {
      out.excluded.push_back(idx);continue;
    }
    out.occlusion_flagged_points+=info.has_occluder; // retain actual far/multiple returns
    const PatchCell key{static_cast<int64_t>(qx),static_cast<int64_t>(qy)};
    auto [cell,inserted]=patch_cells_.try_emplace(key,-1);(void)inserted;
    int best=-1;double best_dz=std::numeric_limits<double>::infinity();
    for(int j=cell->second;j>=0;j=height_patches_[j].next_cell) {
      const auto & patch=height_patches_[j];
      const double dz=std::abs(p.z-patch.moments.mean[2]);
      if(dz>cfg.max_patch_z_spread*.5+1e-6 || dz>=best_dz ||
          std::max(patch.max_z,static_cast<double>(p.z))-std::min(patch.min_z,static_cast<double>(p.z))>cfg.max_patch_z_spread+1e-6)continue;
      bool connected=false;
      for(int k=patch.head;k>=0;k=point_next_[k]) {
        const auto & q=points[k];++out.neighbor_tests;
        const double x=static_cast<double>(p.x)-q.x,y=static_cast<double>(p.y)-q.y;
        if(x*x+y*y<=gap2 && std::abs(static_cast<double>(p.z)-q.z)<=cfg.max_neighbor_dz+1e-6) {connected=true;break;}
      }
      if(connected) {best=j;best_dz=dz;}
    }
    if(best<0) {
      best=height_patches_.size();HeightPatch patch;patch.cell=key;patch.next_cell=cell->second;
      patch.parent=patch.tail=best;patch.min_x=patch.max_x=p.x;patch.min_y=patch.max_y=p.y;patch.min_z=patch.max_z=p.z;
      height_patches_.push_back(patch);cell->second=best;
    }
    auto & patch=height_patches_[best];point_next_[idx]=patch.head;patch.head=idx;patch.moments.add(p);
    patch.min_x=std::min(patch.min_x,static_cast<double>(p.x));patch.max_x=std::max(patch.max_x,static_cast<double>(p.x));
    patch.min_y=std::min(patch.min_y,static_cast<double>(p.y));patch.max_y=std::max(patch.max_y,static_cast<double>(p.y));
    patch.min_z=std::min(patch.min_z,static_cast<double>(p.z));patch.max_z=std::max(patch.max_z,static_cast<double>(p.z));
    out.patch_ids[idx]=best;out.candidates.push_back(idx);
  }
  out.patch_count=height_patches_.size();out.spatial_cells=patch_cells_.size();out.patch_ms=elapsed(patch_begin);
  const auto fitting_begin=std::chrono::steady_clock::now();
  const double min_normal_z=std::cos(cfg.max_plane_tilt_deg*pi/180);
  for(auto & patch:height_patches_) {
    if(patch.moments.count<static_cast<size_t>(cfg.min_patch_points) || !fitPlane(patch.moments,patch.model) ||
        patch.model.normal[2]<min_normal_z || patch.model.rms>cfg.max_plane_rms)continue;
    for(int i=patch.head;i>=0;i=point_next_[i]) {
      const auto & p=points[i];patch.residual_bound=std::max(patch.residual_bound,std::abs(p.z-patch.model.predict(p.x,p.y)));
    }
    if(patch.residual_bound>cfg.max_merge_gap_z)continue;
    patch.valid=true;++out.valid_patch_count;
  }
  out.fitting_ms=elapsed(fitting_begin);
  const auto merge_begin=std::chrono::steady_clock::now();
  // There is no General Visibility point-adjacency graph to duplicate. Build
  // edges only among nearby XY cells and require a real point-pair witness.
  const int radius=static_cast<int>(std::ceil(cfg.max_xy_gap/cfg.patch_resolution));
  for(size_t i=0;i<height_patches_.size();++i) {
    const auto & a=height_patches_[i];if(!a.valid)continue;
    for(int dx=-radius;dx<=radius;++dx)for(int dy=-radius;dy<=radius;++dy) {
      const auto cell=patch_cells_.find({a.cell.x+dx,a.cell.y+dy});if(cell==patch_cells_.end())continue;
      for(int j=cell->second;j>=0;j=height_patches_[j].next_cell) {
        const auto & b=height_patches_[j];if(j<=static_cast<int>(i) || !b.valid)continue;
        if(std::max(a.min_z-b.max_z,b.min_z-a.max_z)>cfg.max_neighbor_dz+1e-6)continue;
        const double bx=std::max({a.min_x-b.max_x,b.min_x-a.max_x,0.0});
        const double by=std::max({a.min_y-b.max_y,b.min_y-a.max_y,0.0});
        if(bx*bx+by*by>gap2)continue;
        bool found=false;
        for(int ai=a.head;ai>=0 && !found;ai=point_next_[ai])for(int bi=b.head;bi>=0;bi=point_next_[bi]) {
          const auto & p=points[ai];const auto & q=points[bi];++out.neighbor_tests;
          const double x=static_cast<double>(p.x)-q.x,y=static_cast<double>(p.y)-q.y;
          if(x*x+y*y<=gap2 && std::abs(static_cast<double>(p.z)-q.z)<=cfg.max_neighbor_dz+1e-6) {
            patch_edges_.push_back({static_cast<int>(i),j,.5*(p.x+q.x),.5*(p.y+q.y)});found=true;break;
          }
        }
      }
    }
  }
  const double min_dot=std::cos(cfg.max_merge_angle_deg*pi/180);
  const auto aligned=[&](const FastPlane & a,const FastPlane & b) {
    return a.normal[0]*b.normal[0]+a.normal[1]*b.normal[1]+a.normal[2]*b.normal[2]>=min_dot;
  };
  const auto prediction_shift=[](const HeightPatch & root,const FastPlane & model) {
    double shift=0;
    for(double x:{root.min_x,root.max_x})for(double y:{root.min_y,root.max_y})
      shift=std::max(shift,std::abs(root.model.predict(x,y)-model.predict(x,y)));
    return shift;
  };
  for(const auto & edge:patch_edges_) {
    int ai=patchRoot(edge.a),bi=patchRoot(edge.b);if(ai==bi)continue;
    ++out.merge_attempts;
    if(height_patches_[ai].moments.count<height_patches_[bi].moments.count)std::swap(ai,bi);
    auto & a=height_patches_[ai];auto & b=height_patches_[bi];
    if(std::max(a.max_z,b.max_z)-std::min(a.min_z,b.min_z)>cfg.max_patch_z_spread+1e-6)continue;
    if(!aligned(a.model,b.model) || std::abs(a.model.predict(edge.x,edge.y)-b.model.predict(edge.x,edge.y))>cfg.max_merge_gap_z)continue;
    PlaneMoments moments=a.moments;moments.combine(b.moments);FastPlane model;
    if(!fitPlane(moments,model) || model.rms>cfg.max_plane_rms || model.normal[2]<min_normal_z ||
        !aligned(model,a.model) || !aligned(model,b.model))continue;
    double bound=std::max(a.residual_bound+prediction_shift(a,model),b.residual_bound+prediction_shift(b,model));
    if(bound>cfg.max_merge_gap_z+1e-9) {
      double actual_max=0;bool ok=true;
      for(int patch=ai;patch>=0 && ok;patch=height_patches_[patch].next_member)
        for(int k=height_patches_[patch].head;k>=0;k=point_next_[k]) {
          const auto & p=points[k];const double res=std::abs(p.z-model.predict(p.x,p.y));
          if(res>cfg.max_merge_gap_z) {ok=false;break;}
          actual_max=std::max(actual_max,res);
        }
      for(int patch=bi;patch>=0 && ok;patch=height_patches_[patch].next_member)
        for(int k=height_patches_[patch].head;k>=0;k=point_next_[k]) {
          const auto & p=points[k];const double res=std::abs(p.z-model.predict(p.x,p.y));
          if(res>cfg.max_merge_gap_z) {ok=false;break;}
          actual_max=std::max(actual_max,res);
        }
      if(!ok)continue;
      bound=actual_max;
    }
    a.moments=moments;a.model=model;a.residual_bound=bound;
    a.min_x=std::min(a.min_x,b.min_x);a.max_x=std::max(a.max_x,b.max_x);
    a.min_y=std::min(a.min_y,b.min_y);a.max_y=std::max(a.max_y,b.max_y);
    a.min_z=std::min(a.min_z,b.min_z);a.max_z=std::max(a.max_z,b.max_z);
    height_patches_[a.tail].next_member=bi;a.tail=b.tail;a.patch_count+=b.patch_count;b.parent=ai;++out.merged_edges;
  }
  out.merging_ms=elapsed(merge_begin);
  const auto final_begin=std::chrono::steady_clock::now();
  for(size_t i=0;i<height_patches_.size();++i) {
    auto & root=height_patches_[i];
    if(!root.valid || root.parent!=static_cast<int>(i) || root.moments.count<static_cast<size_t>(cfg.min_surface_points))continue;
    const auto & s=root.moments.scatter;const auto & center=root.moments.mean;
    // One analytic 2D principal axis per final region, then actual observed extents.
    const double yaw=.5*std::atan2(2*s[1],s[0]-s[3]),cs=std::cos(yaw),sn=std::sin(yaw);
    double umin=std::numeric_limits<double>::infinity(),umax=-umin,vmin=umin,vmax=-umin;
    region_points_.clear();
    for(int patch=i;patch>=0;patch=height_patches_[patch].next_member)for(int k=height_patches_[patch].head;k>=0;k=point_next_[k]) {
      const auto & p=points[k];const double x=p.x-center[0],y=p.y-center[1],u=cs*x+sn*y,v=-sn*x+cs*y;
      umin=std::min(umin,u);umax=std::max(umax,u);vmin=std::min(vmin,v);vmax=std::max(vmax,v);region_points_.push_back(k);
    }
    const double length=std::max(umax-umin,vmax-vmin),width=std::min(umax-umin,vmax-vmin);
    if(length+1e-6<cfg.min_surface_length || width+1e-6<cfg.min_surface_width)continue;
    CraneHeightSurface surface;surface.id=out.surfaces.size();surface.index_begin=out.surface_points.size();
    surface.point_count=region_points_.size();surface.patch_count=root.patch_count;
    surface.center_x=center[0];surface.center_y=center[1];surface.center_z=center[2];
    surface.min_z=root.min_z;surface.max_z=root.max_z;surface.plane_a=root.model.a;surface.plane_b=root.model.b;
    surface.plane_c=root.model.c;surface.plane_normal=root.model.normal;surface.plane_rms=root.model.rms;
    surface.max_residual_bound=root.residual_bound;surface.length=length;surface.width=width;
    for(int k:region_points_) {out.surface_ids[k]=surface.id;out.surface_points.push_back(k);}
    root.surface_id=surface.id;out.surfaces.push_back(surface);
  }
  for(int idx:out.candidates)if(out.surface_ids[idx]<0)out.unassigned.push_back(idx);
  out.finalization_ms=elapsed(final_begin);out.total_ms=elapsed(begin);result_.cluster_count=out.surfaces.size();
  result_.workspace_bytes=height_patches_.capacity()*sizeof(HeightPatch)+patch_edges_.capacity()*sizeof(PatchEdge)+
      (point_next_.capacity()+region_points_.capacity()+out.patch_ids.capacity()+out.surface_ids.capacity()+
       out.candidates.capacity()+out.surface_points.capacity()+out.unassigned.capacity()+out.excluded.capacity())*sizeof(int)+
      out.surfaces.capacity()*sizeof(CraneHeightSurface);
  return result_;
}

double CranePipeline::computeOccupiedArea(
    const std::vector<int> & point_indices,
    const std::vector<PointXYZIRT> & points,
    double grid_resolution_m) {
  if (point_indices.empty() || grid_resolution_m <= 0.0) return 0.0;
  const double inv_r = 1.0 / grid_resolution_m;
  std::unordered_set<uint64_t> occupied_cells;
  occupied_cells.reserve(point_indices.size());
  for (int idx : point_indices) {
    if (idx < 0 || static_cast<size_t>(idx) >= points.size()) continue;
    const auto & p = points[idx];
    const int64_t gx = static_cast<int64_t>(std::floor(p.x * inv_r));
    const int64_t gy = static_cast<int64_t>(std::floor(p.y * inv_r));
    const uint64_t key = (static_cast<uint64_t>(gx) << 32) ^ (static_cast<uint64_t>(gy) & 0xffffffffULL);
    occupied_cells.insert(key);
  }
  return static_cast<double>(occupied_cells.size()) * (grid_resolution_m * grid_resolution_m);
}

const CraneSlopeResult & CranePipeline::processHookAndBeamDetection(
    const std::vector<PointXYZIRT> & points,
    const std::vector<CraneSubcluster> & subclusters,
    double stamp_sec,
    VisibilityModel * visibility,
    const std::vector<PointXYZIRT> * current_observation_points) {
  if (visibility != nullptr) {
    visibility_model_ptr_ = visibility;
  }
  // 1. Reset frame hook and beam detection results
  result_.hook = CraneHookResult{};
  result_.beam = CraneBeamGeometry{};

  double dt = 0.1;
  if (has_last_hook_stamp_ && stamp_sec > 0.0 && (stamp_sec + 0.5 < last_hook_stamp_sec_)) {
    // Rosbag loop playback / time jump backward detected: reset temporal states cleanly
    const int next_left_housing_id=left_td_track_.next_housing_track_id;
    const int next_right_housing_id=right_td_track_.next_housing_track_id;
    left_td_track_ = TrackedTopDownStructure{};
    right_td_track_ = TrackedTopDownStructure{};
    left_td_track_.next_housing_track_id=next_left_housing_id;
    right_td_track_.next_housing_track_id=next_right_housing_id;
    tracked_hook_chains_.clear();
    left_fixed_root_state_ = CraneFixedRootState{};
    right_fixed_root_state_ = CraneFixedRootState{};
    clearStationaryAccumulation();
    stabilized_beam_ = StabilizedBeamReference{};
  } else if (has_last_hook_stamp_ && stamp_sec > last_hook_stamp_sec_) {
    dt = std::clamp(stamp_sec - last_hook_stamp_sec_, 0.001, 1.0);
  }
  last_hook_stamp_sec_ = stamp_sec;
  has_last_hook_stamp_ = (stamp_sec > 0.0);

  std::array<int,2> current_root_indices{{-1,-1}};
  // 2. Ground reference for Hook distance
  double ground_z = config_.ground_z;
  if (!result_.ground.empty()) {
    double sum_gz = 0.0;
    size_t gcnt = 0;
    for (int idx : result_.ground) {
      if (idx >= 0 && static_cast<size_t>(idx) < points.size()) {
        const double gz = points[idx].z;
        if (std::isfinite(gz) && std::abs(gz - config_.ground_z) <= 0.8) {
          sum_gz += gz;
          ++gcnt;
        }
      }
    }
    if (gcnt > 0) ground_z = sum_gz / static_cast<double>(gcnt);
  } else if (std::isfinite(result_.closed_loop.background_z)) {
    ground_z = result_.closed_loop.background_z;
  }

  // 4. Top-Down Directed Graph & Gap-Tolerant Association
  const auto t_hook_begin = std::chrono::steady_clock::now();
  auto & td_res = result_.top_down_hook;
  td_res = CraneTopDownHookResult{};
  const auto & td_cfg = config_.top_down_hook;

  if (td_cfg.enabled) {
    const double tan_theta = std::tan(td_cfg.max_sway_angle_deg * M_PI / 180.0);

    // 4.1 Filter candidate subclusters within Crane ROI and above beam/ground
    std::vector<int> candidate_sc_indices;
    candidate_sc_indices.reserve(subclusters.size());
    for (size_t i = 0; i < subclusters.size(); ++i) {
      const auto & sc = subclusters[i];
      if (sc.point_indices.size() < td_cfg.min_subcluster_points) continue;
      if (sc.centroid_z < td_cfg.roi_z_min) continue;
      if (sc.max_z <= ground_z + 0.02 && sc.centroid_z <= ground_z + 0.05) continue; // exclude points beneath ground
      if (std::abs(sc.centroid_x) > 3.0 || std::abs(sc.centroid_y) > 2.5) continue;
      candidate_sc_indices.push_back(static_cast<int>(i));
    }

    // 4.2 Identify Root and Build Top-Down Chains
    std::vector<CraneHookChain> frame_chains;



    if (config_.fixed_root.enabled) {
      const auto & fx_cfg = config_.fixed_root;

      for (int side = 0; side < 2; ++side) {
        const bool is_left = (side == 0);
        const auto & roi = is_left ? fx_cfg.left_roi : fx_cfg.right_roi;
        const double fixed_center_x = roi.centerX();
        const double fixed_center_y = roi.centerY();
        const double fixed_center_z = roi.centerZ();

        // 1. Point distribution inspection inside ROI
        size_t roi_pts_count = 0;
        double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
        for (const auto & p : points) {
          if (roi.contains(p.x, p.y, p.z)) {
            ++roi_pts_count;
            sum_x += p.x;
            sum_y += p.y;
            sum_z += p.z;
          }
        }
        const double cur_cx = (roi_pts_count > 0) ? (sum_x / roi_pts_count) : fixed_center_x;
        const double cur_cy = (roi_pts_count > 0) ? (sum_y / roi_pts_count) : fixed_center_y;
        const double cur_cz = (roi_pts_count > 0) ? (sum_z / roi_pts_count) : fixed_center_z;

        // 2. Identify Root Subcluster in this ROI
        int root_sc_idx = -1;
        double max_sc_score = -std::numeric_limits<double>::infinity();
        for (int sc_idx : candidate_sc_indices) {
          const auto & sc = subclusters[sc_idx];
          if (sc.point_indices.size() < td_cfg.min_subcluster_points) continue;
          bool in_roi = roi.contains(sc.centroid_x, sc.centroid_y, sc.centroid_z);
          if (!in_roi) {
            bool overlap = !(sc.max_x < roi.x_min || sc.min_x > roi.x_max ||
                             sc.max_y < roi.y_min || sc.min_y > roi.y_max ||
                             sc.max_z < roi.z_min || sc.min_z > roi.z_max);
            if (overlap && sc.centroid_z >= roi.z_min - 0.20) in_roi = true;
          }
          if (in_roi) {
            double score = sc.centroid_z + 0.35 * std::log(1.0 + sc.point_indices.size());
            if (score > max_sc_score) {
              max_sc_score = score;
              root_sc_idx = sc_idx;
            }
          }
        }

        // 3. State update
        auto & fx_state = is_left ? left_fixed_root_state_ : right_fixed_root_state_;
        fx_state.is_left = is_left;
        fx_state.fixed_center_x = fixed_center_x;
        fx_state.fixed_center_y = fixed_center_y;
        fx_state.fixed_center_z = fixed_center_z;
        fx_state.point_count = roi_pts_count;
        fx_state.cluster_present = (root_sc_idx >= 0);
        if (root_sc_idx >= 0) {
          fx_state.subcluster_id = subclusters[root_sc_idx].subcluster_id;
          fx_state.layer_id = subclusters[root_sc_idx].layer_id;
          fx_state.observed_centroid_x = subclusters[root_sc_idx].centroid_x;
          fx_state.observed_centroid_y = subclusters[root_sc_idx].centroid_y;
          fx_state.observed_centroid_z = subclusters[root_sc_idx].centroid_z;
        } else {
          fx_state.subcluster_id = -1;
          fx_state.layer_id = -1;
          fx_state.observed_centroid_x = cur_cx;
          fx_state.observed_centroid_y = cur_cy;
          fx_state.observed_centroid_z = cur_cz;
        }

        const bool is_now_observed = (roi_pts_count >= fx_cfg.min_points);
        if (is_now_observed) {
          fx_state.obs_status = CraneRootObservationStatus::OBSERVED;
          fx_state.consecutive_observed++;
          fx_state.consecutive_missed = 0;
          fx_state.is_held = false;
        } else {
          fx_state.consecutive_missed++;
          if (fx_state.consecutive_observed > 0 && fx_state.consecutive_missed <= fx_cfg.hold_frames) {
            fx_state.obs_status = CraneRootObservationStatus::OBSERVED;
            fx_state.is_held = true;
          } else {
            fx_state.obs_status = CraneRootObservationStatus::NOT_OBSERVED;
            fx_state.is_held = false;
          }
        }

        // 4. Motion tracking of pointcloud distribution inside ROI
        if (is_now_observed) {
          if (fx_state.prev_centroid_valid && dt > 0.001) {
            const double dx = cur_cx - fx_state.prev_centroid_x;
            const double dy = cur_cy - fx_state.prev_centroid_y;
            const double dz = cur_cz - fx_state.prev_centroid_z;
            const double dxy = std::hypot(dx, dy);
            const double d3d = std::hypot(dxy, dz);
            const double inst_speed = dxy / dt;

            fx_state.smoothed_speed = 0.25 * inst_speed + 0.75 * fx_state.smoothed_speed;
            fx_state.delta_x = dx;
            fx_state.delta_y = dy;
            fx_state.delta_z = dz;
            fx_state.delta_xy = dxy;
            fx_state.delta_3d = d3d;
            fx_state.velocity_xy = inst_speed;
            fx_state.velocity_z = std::abs(dz) / dt;

            if (fx_state.smoothed_speed >= fx_cfg.motion_speed_threshold ||
                dxy >= fx_cfg.motion_disp_threshold) {
              fx_state.moving_confirm_count++;
              fx_state.stop_confirm_count = 0;
            } else {
              fx_state.stop_confirm_count++;
              fx_state.moving_confirm_count = 0;
            }

            if (fx_state.moving_confirm_count >= fx_cfg.motion_confirm_frames) {
              fx_state.motion_status = CraneRootMotionStatus::MOVING;
            } else if (fx_state.stop_confirm_count >= 2) {
              fx_state.motion_status = CraneRootMotionStatus::STATIONARY;
            }
          } else {
            fx_state.motion_status = CraneRootMotionStatus::STATIONARY;
            fx_state.smoothed_speed = 0.0;
          }
          fx_state.prev_centroid_x = cur_cx;
          fx_state.prev_centroid_y = cur_cy;
          fx_state.prev_centroid_z = cur_cz;
          fx_state.prev_centroid_valid = true;
        } else {
          fx_state.motion_status = CraneRootMotionStatus::UNKNOWN;
          fx_state.prev_centroid_valid = false;
        }

        // 5. Build chain anchored at the Fixed Geometric Center
        CraneHookChain chain;
        chain.side = is_left ? "LEFT" : "RIGHT";
        chain.root_roi = roi;
        chain.root_obs_status = fx_state.obs_status;
        chain.root_motion_status = fx_state.motion_status;
        chain.is_observed = (fx_state.obs_status == CraneRootObservationStatus::OBSERVED);
        chain.is_held = fx_state.is_held;
        chain.root_subcluster_id = (root_sc_idx >= 0) ? subclusters[root_sc_idx].subcluster_id : -1;
        chain.root_observed_cx = cur_cx;
        chain.root_observed_cy = cur_cy;
        chain.root_observed_cz = cur_cz;
        chain.root_delta_x = fx_state.delta_x;
        chain.root_delta_y = fx_state.delta_y;
        chain.root_delta_z = fx_state.delta_z;
        chain.root_speed = fx_state.smoothed_speed;

        // FIXED ROOT COORDINATES - ZERO JITTER
        chain.root_x = fixed_center_x;
        chain.root_y = fixed_center_y;
        chain.root_z = fixed_center_z;

        CraneHookChainNode root_node;
        root_node.subcluster_id = chain.root_subcluster_id;
        root_node.layer_id = (root_sc_idx >= 0) ? subclusters[root_sc_idx].layer_id : -1;
        root_node.x = fixed_center_x;
        root_node.y = fixed_center_y;
        root_node.z = fixed_center_z;
        root_node.is_root = true;
        root_node.is_held = fx_state.is_held;
        if (root_sc_idx >= 0) {
          const auto & rsc = subclusters[root_sc_idx];
          root_node.min_x = rsc.min_x; root_node.max_x = rsc.max_x;
          root_node.min_y = rsc.min_y; root_node.max_y = rsc.max_y;
          root_node.min_z = rsc.min_z; root_node.max_z = rsc.max_z;
          root_node.point_count = rsc.point_indices.size();
          root_node.point_indices = rsc.point_indices;
        } else {
          root_node.min_x = roi.x_min; root_node.max_x = roi.x_max;
          root_node.min_y = roi.y_min; root_node.max_y = roi.y_max;
          root_node.min_z = roi.z_min; root_node.max_z = roi.z_max;
          root_node.point_count = roi_pts_count;
        }
        chain.nodes.push_back(root_node);

        // 6. Downward tree exploration
        std::vector<bool> in_chain(subclusters.size(), false);
        if (root_sc_idx >= 0) in_chain[root_sc_idx] = true;
        int curr_sc_idx = root_sc_idx;

        while (true) {
          const auto & parent_node = chain.nodes.back();
          const double curr_sc_z = (curr_sc_idx >= 0) ? subclusters[curr_sc_idx].centroid_z : parent_node.z;
          const double curr_sc_min_z = (curr_sc_idx >= 0) ? subclusters[curr_sc_idx].min_z : parent_node.z;
          const double curr_sc_max_x = (curr_sc_idx >= 0) ? subclusters[curr_sc_idx].max_x : (parent_node.x + 0.1);
          const double curr_sc_min_x = (curr_sc_idx >= 0) ? subclusters[curr_sc_idx].min_x : (parent_node.x - 0.1);
          const double curr_sc_max_y = (curr_sc_idx >= 0) ? subclusters[curr_sc_idx].max_y : (parent_node.y + 0.1);
          const double curr_sc_min_y = (curr_sc_idx >= 0) ? subclusters[curr_sc_idx].min_y : (parent_node.y - 0.1);
          const int curr_sc_gid = (curr_sc_idx >= 0) ? subclusters[curr_sc_idx].group_id : -1;

          int best_child_idx = -1;
          double best_child_cost = std::numeric_limits<double>::infinity();
          double best_child_dz = 0.0;
          double best_child_dxy = 0.0;
          double best_child_eff_x = 0.0;
          double best_child_eff_y = 0.0;
          double best_child_slope = 0.0;
          double best_child_turn = 0.0;

          for (int sc_idx : candidate_sc_indices) {
            if (in_chain[sc_idx]) continue;
            const auto & cand = subclusters[sc_idx];

            // Condition 1: Strict downward direction z_A > z_B
            const double dz = curr_sc_z - cand.centroid_z;
            if (dz < 0.03) continue;
            const double dz_gap = std::max(0.0, curr_sc_min_z - cand.max_z);
            if (dz > td_cfg.max_gap_z && dz_gap > td_cfg.max_gap_z) continue;

            // Plumb-line constraint: cand must be within max_lateral_drift_from_root from root's plumb-line
            const double bbox_root_dx = std::max({0.0, cand.min_x - chain.root_x, chain.root_x - cand.max_x});
            const double bbox_root_dy = std::max({0.0, cand.min_y - chain.root_y, chain.root_y - cand.max_y});
            const double bbox_root_dxy = std::hypot(bbox_root_dx, bbox_root_dy);
            if (bbox_root_dxy > td_cfg.max_lateral_drift_from_root) continue;

            // Spatial partition: Left vs Right
            const double span_x = cand.max_x - cand.min_x;
            if (span_x < 0.8) {
              if (side == 0 && cand.centroid_x >= 0.10) continue;
              if (side == 1 && cand.centroid_x <= -0.10) continue;
            } else {
              if (side == 0 && cand.min_x > chain.root_x + 0.35) continue;
              if (side == 1 && cand.max_x < chain.root_x - 0.35) continue;
            }

            // Effective coordinate of candidate relative to parent
            const double cand_eff_x = std::clamp(parent_node.x, cand.min_x, cand.max_x);
            const double cand_eff_y = std::clamp(parent_node.y, cand.min_y, cand.max_y);
            const double cand_eff_z = cand.centroid_z;

            // Gap-Tolerant Association gate:
            const double b_dx = std::max({0.0, cand.min_x - curr_sc_max_x, curr_sc_min_x - cand.max_x});
            const double b_dy = std::max({0.0, cand.min_y - curr_sc_max_y, curr_sc_min_y - cand.max_y});
            const double b_dxy = std::hypot(b_dx, b_dy);
            const double c_dxy = std::hypot(cand.centroid_x - parent_node.x, cand.centroid_y - parent_node.y);
            const double dxy = std::min(c_dxy, b_dxy);
            const double allowed_dxy = td_cfg.r0 + dz * tan_theta;
            if (dxy > allowed_dxy) continue;

            // Slope & Turn angle
            const double edge_dx = cand_eff_x - parent_node.x;
            const double edge_dy = cand_eff_y - parent_node.y;
            const double edge_dz = parent_node.z - cand_eff_z;
            const double edge_dxy = std::hypot(edge_dx, edge_dy);
            const double slope_angle_deg = std::atan2(edge_dxy, std::max(1e-4, edge_dz)) * 180.0 / M_PI;
            if (td_cfg.max_slope_angle_deg > 0.0 && slope_angle_deg > td_cfg.max_slope_angle_deg) {
              continue;
            }

            double turn_angle_deg = 0.0;
            if (chain.nodes.size() >= 2) {
              const auto & gparent_node = chain.nodes[chain.nodes.size() - 2];
              const double v1_x = parent_node.x - gparent_node.x;
              const double v1_y = parent_node.y - gparent_node.y;
              const double v1_z = parent_node.z - gparent_node.z;
              const double len1 = std::sqrt(v1_x * v1_x + v1_y * v1_y + v1_z * v1_z);

              const double v2_x = cand_eff_x - parent_node.x;
              const double v2_y = cand_eff_y - parent_node.y;
              const double v2_z = cand_eff_z - parent_node.z;
              const double len2 = std::sqrt(v2_x * v2_x + v2_y * v2_y + v2_z * v2_z);

              if (len1 > 1e-4 && len2 > 1e-4) {
                const double dot = (v1_x * v2_x + v1_y * v2_y + v1_z * v2_z) / (len1 * len2);
                const double clamped_dot = std::clamp(dot, -1.0, 1.0);
                turn_angle_deg = std::acos(clamped_dot) * 180.0 / M_PI;

                if (td_cfg.max_turn_angle_deg > 0.0 && turn_angle_deg > td_cfg.max_turn_angle_deg) {
                  continue;
                }
              }
            }

            // Cost
            double cost = dxy + 0.18 * dz - 0.02 * std::log(1.0 + cand.point_indices.size());
            if (curr_sc_gid >= 0 && curr_sc_gid == cand.group_id) {
              cost -= 0.25;
            }
            if (td_cfg.max_slope_angle_deg > 0.0) {
              cost += 0.25 * (slope_angle_deg / td_cfg.max_slope_angle_deg);
            }
            if (chain.nodes.size() >= 2 && td_cfg.max_turn_angle_deg > 0.0) {
              cost += 0.25 * (turn_angle_deg / td_cfg.max_turn_angle_deg);
            }

            if (cost < best_child_cost) {
              best_child_cost = cost;
              best_child_idx = sc_idx;
              best_child_dz = dz;
              best_child_dxy = dxy;
              best_child_eff_x = cand_eff_x;
              best_child_eff_y = cand_eff_y;
              best_child_slope = slope_angle_deg;
              best_child_turn = turn_angle_deg;
            }
          }

          if (best_child_idx >= 0) {
            in_chain[best_child_idx] = true;
            const auto & child_sc = subclusters[best_child_idx];
            CraneHookChainNode child_node;
            child_node.subcluster_id = child_sc.subcluster_id;
            child_node.layer_id = child_sc.layer_id;
            child_node.x = best_child_eff_x;
            child_node.y = best_child_eff_y;
            child_node.z = child_sc.centroid_z;
            child_node.min_x = child_sc.min_x; child_node.max_x = child_sc.max_x;
            child_node.min_y = child_sc.min_y; child_node.max_y = child_sc.max_y;
            child_node.min_z = child_sc.min_z; child_node.max_z = child_sc.max_z;
            child_node.point_count = child_sc.point_indices.size();
            child_node.point_indices = child_sc.point_indices;
            child_node.parent_subcluster_id = (curr_sc_idx >= 0) ? subclusters[curr_sc_idx].subcluster_id : -1;
            child_node.gap_z_to_parent = best_child_dz;
            child_node.dxy_to_parent = best_child_dxy;
            child_node.slope_angle_deg = best_child_slope;
            child_node.turn_angle_deg = best_child_turn;
            const double dz_gap = std::max(0.0, curr_sc_min_z - child_sc.max_z);
            child_node.is_estimated_gap = (best_child_dz > td_cfg.direct_link_max_gap_z || dz_gap > 0.35);


            if (child_node.is_estimated_gap) {
              chain.has_gap = true;
              chain.max_gap_z = std::max(chain.max_gap_z, best_child_dz);
            }
            chain.max_slope_angle_deg = std::max(chain.max_slope_angle_deg, best_child_slope);
            chain.max_turn_angle_deg = std::max(chain.max_turn_angle_deg, best_child_turn);
            chain.nodes.push_back(child_node);
            curr_sc_idx = best_child_idx;
          } else {
            break; // Chain reached its lowest terminus
          }
        }


        // Assign lowest node properties
        const auto & lowest = chain.nodes.back();
        chain.lowest_subcluster_id = lowest.subcluster_id;
        chain.lowest_x = lowest.x;
        chain.lowest_y = lowest.y;
        chain.lowest_z = lowest.z;
        chain.lowest_min_z = lowest.min_z;
        chain.lowest_max_z = lowest.max_z;
        chain.lowest_point_count = lowest.point_count;
        chain.lowest_point_indices = lowest.point_indices;
        chain.total_depth = chain.root_z - chain.lowest_z;
        chain.ground_valid = true;
        chain.ground_z = ground_z;
        chain.ground_distance = std::max(0.0, lowest.min_z - ground_z);

        // Select Spatially Largest Connected Cluster in the Top-Down chain below Root
        CraneHookNearestCluster largest_conn;
        double max_metric = -1.0;
        for (size_t ni = 1; ni < chain.nodes.size(); ++ni) {
          const auto & n = chain.nodes[ni];
          const double area = computeOccupiedArea(n.point_indices, points, 0.05);
          const double len = std::max(n.max_x - n.min_x, n.max_y - n.min_y);
          const double wid = std::min(n.max_x - n.min_x, n.max_y - n.min_y);
          const double hei = n.max_z - n.min_z;
          const double metric = (area > 0.0) ? area : (len * wid);
          if (metric > max_metric) {
            max_metric = metric;
            largest_conn.valid = true;
            largest_conn.subcluster_id = n.subcluster_id;
            largest_conn.layer_id = n.layer_id;
            largest_conn.point_count = n.point_count;
            largest_conn.centroid_x = n.x;
            largest_conn.centroid_y = n.y;
            largest_conn.centroid_z = n.z;
            largest_conn.ref_x = n.x;
            largest_conn.ref_y = n.y;
            largest_conn.ref_z = n.z;
            largest_conn.min_x = n.min_x;
            largest_conn.max_x = n.max_x;
            largest_conn.min_y = n.min_y;
            largest_conn.max_y = n.max_y;
            largest_conn.min_z = n.min_z;
            largest_conn.max_z = n.max_z;
            largest_conn.length = len;
            largest_conn.width = wid;
            largest_conn.height = hei;
            largest_conn.area_xy = area;
            largest_conn.dxy = std::hypot(n.x - chain.root_x, n.y - chain.root_y);
            largest_conn.dz = chain.root_z - n.z;
            largest_conn.dist_3d = std::hypot(largest_conn.dxy, largest_conn.dz);
          }
        }
        chain.largest_cluster = largest_conn;

        frame_chains.push_back(std::move(chain));
      }
    } else {
      // 4.2 Fallback: Dynamic Root Search
      int left_root_idx = -1;
      double max_left_z = -std::numeric_limits<double>::infinity();
      int right_root_idx = -1;
      double max_right_z = -std::numeric_limits<double>::infinity();

      for (int sc_idx : candidate_sc_indices) {
        const auto & sc = subclusters[sc_idx];
        if (sc.point_indices.size() < td_cfg.min_root_points) continue;
        if (sc.max_z < td_cfg.root_min_z && sc.centroid_z < td_cfg.root_min_z - 0.2) continue;

        const double base_score = sc.centroid_z + 0.35 * std::log(1.0 + sc.point_indices.size());
        if (sc.centroid_x >= td_cfg.roi_left_x_min && sc.centroid_x <= td_cfg.roi_left_x_max) {
          double root_score = base_score;
          if (left_root_node_.initialized) {
            const double pred_x = left_root_node_.smoothed_x + left_root_node_.kf_xy.vx() * dt;
            const double pred_y = left_root_node_.smoothed_y + left_root_node_.kf_xy.vy() * dt;
            const double pred_z = left_root_node_.smoothed_z + left_root_node_.kf_z.vx() * dt;
            const double d_xy = std::hypot(sc.centroid_x - pred_x, sc.centroid_y - pred_y);
            const double d_z = std::abs(sc.centroid_z - pred_z);
            const double d_3d = std::hypot(d_xy, d_z);
            if (d_3d <= 0.15) {
              root_score += 0.80;
            } else {
              root_score -= 2.5 * d_xy;
            }
          }
          if (root_score > max_left_z) {
            max_left_z = root_score;
            left_root_idx = sc_idx;
          }
        } else if (sc.centroid_x >= td_cfg.roi_right_x_min && sc.centroid_x <= td_cfg.roi_right_x_max) {
          double root_score = base_score;
          if (right_root_node_.initialized) {
            const double pred_x = right_root_node_.smoothed_x + right_root_node_.kf_xy.vx() * dt;
            const double pred_y = right_root_node_.smoothed_y + right_root_node_.kf_xy.vy() * dt;
            const double pred_z = right_root_node_.smoothed_z + right_root_node_.kf_z.vx() * dt;
            const double d_xy = std::hypot(sc.centroid_x - pred_x, sc.centroid_y - pred_y);
            const double d_z = std::abs(sc.centroid_z - pred_z);
            const double d_3d = std::hypot(d_xy, d_z);
            if (d_3d <= 0.15) {
              root_score += 0.80;
            } else {
              root_score -= 2.5 * d_xy;
            }
          }
          if (root_score > max_right_z) {
            max_right_z = root_score;
            right_root_idx = sc_idx;
          }
        }
      }

      current_root_indices={left_root_idx,right_root_idx};
      for (int side = 0; side < 2; ++side) {
        const int root_sc_idx = (side == 0) ? left_root_idx : right_root_idx;
        if (root_sc_idx < 0) continue;

        const auto & root_sc = subclusters[root_sc_idx];
        auto & tr_root = (side == 0) ? left_root_node_ : right_root_node_;
        if (!tr_root.initialized) {
          tr_root.kf_xy.initialize(root_sc.centroid_x, root_sc.centroid_y, td_cfg.root_measurement_std, 0.05);
          tr_root.kf_z.initialize(root_sc.centroid_z, 0.0, td_cfg.root_measurement_std, 0.05);
          tr_root.smoothed_x = root_sc.centroid_x;
          tr_root.smoothed_y = root_sc.centroid_y;
          tr_root.smoothed_z = root_sc.centroid_z;
          tr_root.initialized = true;
          tr_root.lost_frames = 0;
        } else {
          const double cur_vx = tr_root.kf_xy.vx();
          const double cur_vy = tr_root.kf_xy.vy();
          const double cur_speed = std::hypot(cur_vx, cur_vy);
          const double disp_x = root_sc.centroid_x - tr_root.smoothed_x;
          const double disp_y = root_sc.centroid_y - tr_root.smoothed_y;
          const double disp_xy = std::hypot(disp_x, disp_y);
          const bool is_motion = (cur_speed > 0.06 || disp_xy > 0.05 || crane_moving_);
          const double accel_q = is_motion ? 2.50 : 0.20;

          tr_root.kf_xy.predict(dt, accel_q);
          tr_root.kf_z.predict(dt, accel_q);

          double meas_x = root_sc.centroid_x;
          double meas_y = root_sc.centroid_y;
          double meas_z = root_sc.centroid_z;
          if (!is_motion) {
            const double max_inno = 0.035;
            meas_x = tr_root.smoothed_x + std::clamp(disp_x, -max_inno, max_inno);
            meas_y = tr_root.smoothed_y + std::clamp(disp_y, -max_inno, max_inno);
            meas_z = tr_root.smoothed_z + std::clamp(root_sc.centroid_z - tr_root.smoothed_z, -max_inno, max_inno);
          }
          tr_root.kf_xy.update(meas_x, meas_y, td_cfg.root_measurement_std);
          tr_root.kf_z.update(meas_z, 0.0, td_cfg.root_measurement_std);
          tr_root.smoothed_x = tr_root.kf_xy.x();
          tr_root.smoothed_y = tr_root.kf_xy.y();
          tr_root.smoothed_z = tr_root.kf_z.x();
          tr_root.lost_frames = 0;
        }

        CraneHookChain chain;
        chain.side = (side == 0) ? "LEFT" : "RIGHT";
        chain.root_subcluster_id = root_sc.subcluster_id;
        chain.root_x = tr_root.smoothed_x;
        chain.root_y = tr_root.smoothed_y;
        chain.root_z = tr_root.smoothed_z;

        CraneHookChainNode root_node;
        root_node.subcluster_id = root_sc.subcluster_id;
        root_node.layer_id = root_sc.layer_id;
        root_node.x = tr_root.smoothed_x; root_node.y = tr_root.smoothed_y; root_node.z = tr_root.smoothed_z;
        root_node.min_x = root_sc.min_x; root_node.max_x = root_sc.max_x;
        root_node.min_y = root_sc.min_y; root_node.max_y = root_sc.max_y;
        root_node.min_z = root_sc.min_z; root_node.max_z = root_sc.max_z;
        root_node.point_count = root_sc.point_indices.size();
        root_node.point_indices = root_sc.point_indices;
        root_node.is_root = true;
        chain.nodes.push_back(root_node);

        std::vector<bool> in_chain(subclusters.size(), false);
        in_chain[root_sc_idx] = true;
        int curr_sc_idx = root_sc_idx;

        while (true) {
          const auto & curr_sc = subclusters[curr_sc_idx];
          const auto & parent_node = chain.nodes.back();
          int best_child_idx = -1;
          double best_child_cost = std::numeric_limits<double>::infinity();
          double best_child_dz = 0.0;
          double best_child_dxy = 0.0;
          double best_child_eff_x = 0.0;
          double best_child_eff_y = 0.0;
          double best_child_slope = 0.0;
          double best_child_turn = 0.0;

          for (int sc_idx : candidate_sc_indices) {
            if (in_chain[sc_idx]) continue;
            const auto & cand = subclusters[sc_idx];

            const double dz = curr_sc.centroid_z - cand.centroid_z;
            if (dz < 0.03) continue;
            const double dz_gap = std::max(0.0, curr_sc.min_z - cand.max_z);
            if (dz > td_cfg.max_gap_z && dz_gap > td_cfg.max_gap_z) continue;

            const double bbox_root_dx = std::max({0.0, cand.min_x - chain.root_x, chain.root_x - cand.max_x});
            const double bbox_root_dy = std::max({0.0, cand.min_y - chain.root_y, chain.root_y - cand.max_y});
            const double bbox_root_dxy = std::hypot(bbox_root_dx, bbox_root_dy);
            if (bbox_root_dxy > td_cfg.max_lateral_drift_from_root) continue;

            const double span_x = cand.max_x - cand.min_x;
            if (span_x < 0.8) {
              if (side == 0 && cand.centroid_x >= 0.10) continue;
              if (side == 1 && cand.centroid_x <= -0.10) continue;
            } else {
              if (side == 0 && cand.min_x > chain.root_x + 0.35) continue;
              if (side == 1 && cand.max_x < chain.root_x - 0.35) continue;
            }

            const double cand_eff_x = std::clamp(parent_node.x, cand.min_x, cand.max_x);
            const double cand_eff_y = std::clamp(parent_node.y, cand.min_y, cand.max_y);
            const double cand_eff_z = cand.centroid_z;

            const double b_dx = std::max({0.0, cand.min_x - curr_sc.max_x, curr_sc.min_x - cand.max_x});
            const double b_dy = std::max({0.0, cand.min_y - curr_sc.max_y, curr_sc.min_y - cand.max_y});
            const double b_dxy = std::hypot(b_dx, b_dy);
            const double c_dxy = std::hypot(cand.centroid_x - curr_sc.centroid_x, cand.centroid_y - curr_sc.centroid_y);
            const double dxy = std::min(c_dxy, b_dxy);
            const double allowed_dxy = td_cfg.r0 + dz * tan_theta;
            if (dxy > allowed_dxy) continue;

            const double edge_dx = cand_eff_x - parent_node.x;
            const double edge_dy = cand_eff_y - parent_node.y;
            const double edge_dz = parent_node.z - cand_eff_z;
            const double edge_dxy = std::hypot(edge_dx, edge_dy);
            const double slope_angle_deg = std::atan2(edge_dxy, std::max(1e-4, edge_dz)) * 180.0 / M_PI;
            if (td_cfg.max_slope_angle_deg > 0.0 && slope_angle_deg > td_cfg.max_slope_angle_deg) {
              continue;
            }

            double turn_angle_deg = 0.0;
            if (chain.nodes.size() >= 2) {
              const auto & gparent_node = chain.nodes[chain.nodes.size() - 2];
              const double v1_x = parent_node.x - gparent_node.x;
              const double v1_y = parent_node.y - gparent_node.y;
              const double v1_z = parent_node.z - gparent_node.z;
              const double len1 = std::sqrt(v1_x * v1_x + v1_y * v1_y + v1_z * v1_z);

              const double v2_x = cand_eff_x - parent_node.x;
              const double v2_y = cand_eff_y - parent_node.y;
              const double v2_z = cand_eff_z - parent_node.z;
              const double len2 = std::sqrt(v2_x * v2_x + v2_y * v2_y + v2_z * v2_z);

              if (len1 > 1e-4 && len2 > 1e-4) {
                const double dot = (v1_x * v2_x + v1_y * v2_y + v1_z * v2_z) / (len1 * len2);
                const double clamped_dot = std::clamp(dot, -1.0, 1.0);
                turn_angle_deg = std::acos(clamped_dot) * 180.0 / M_PI;

                if (td_cfg.max_turn_angle_deg > 0.0 && turn_angle_deg > td_cfg.max_turn_angle_deg) {
                  continue;
                }
              }
            }

            double cost = dxy + 0.18 * dz - 0.02 * std::log(1.0 + cand.point_indices.size());
            if (curr_sc.group_id >= 0 && curr_sc.group_id == cand.group_id) {
              cost -= 0.25;
            }
            if (td_cfg.max_slope_angle_deg > 0.0) {
              cost += 0.25 * (slope_angle_deg / td_cfg.max_slope_angle_deg);
            }
            if (chain.nodes.size() >= 2 && td_cfg.max_turn_angle_deg > 0.0) {
              cost += 0.25 * (turn_angle_deg / td_cfg.max_turn_angle_deg);
            }

            if (cost < best_child_cost) {
              best_child_cost = cost;
              best_child_idx = sc_idx;
              best_child_dz = dz;
              best_child_dxy = dxy;
              best_child_eff_x = cand_eff_x;
              best_child_eff_y = cand_eff_y;
              best_child_slope = slope_angle_deg;
              best_child_turn = turn_angle_deg;
            }
          }

          if (best_child_idx >= 0) {
            in_chain[best_child_idx] = true;
            const auto & child_sc = subclusters[best_child_idx];
            CraneHookChainNode child_node;
            child_node.subcluster_id = child_sc.subcluster_id;
            child_node.layer_id = child_sc.layer_id;
            child_node.x = best_child_eff_x;
            child_node.y = best_child_eff_y;
            child_node.z = child_sc.centroid_z;
            child_node.min_x = child_sc.min_x; child_node.max_x = child_sc.max_x;
            child_node.min_y = child_sc.min_y; child_node.max_y = child_sc.max_y;
            child_node.min_z = child_sc.min_z; child_node.max_z = child_sc.max_z;
            child_node.point_count = child_sc.point_indices.size();
            child_node.point_indices = child_sc.point_indices;
            child_node.parent_subcluster_id = curr_sc.subcluster_id;
            child_node.gap_z_to_parent = best_child_dz;
            child_node.dxy_to_parent = best_child_dxy;
            child_node.slope_angle_deg = best_child_slope;
            child_node.turn_angle_deg = best_child_turn;
            const double dz_gap = std::max(0.0, curr_sc.min_z - child_sc.max_z);
            child_node.is_estimated_gap = (best_child_dz > td_cfg.direct_link_max_gap_z || dz_gap > 0.35);


            if (child_node.is_estimated_gap) {
              chain.has_gap = true;
              chain.max_gap_z = std::max(chain.max_gap_z, best_child_dz);
            }
            chain.max_slope_angle_deg = std::max(chain.max_slope_angle_deg, best_child_slope);
            chain.max_turn_angle_deg = std::max(chain.max_turn_angle_deg, best_child_turn);

            chain.nodes.push_back(child_node);
            curr_sc_idx = best_child_idx;
          } else {
            break;
          }
        }


        const auto & lowest = chain.nodes.back();
        chain.lowest_subcluster_id = lowest.subcluster_id;
        chain.lowest_x = lowest.x;
        chain.lowest_y = lowest.y;
        chain.lowest_z = lowest.z;
        chain.lowest_min_z = lowest.min_z;
        chain.lowest_max_z = lowest.max_z;
        chain.lowest_point_count = lowest.point_count;
        chain.lowest_point_indices = lowest.point_indices;
        chain.total_depth = chain.root_z - chain.lowest_z;
        chain.ground_valid = true;
        chain.ground_z = ground_z;
        chain.ground_distance = std::max(0.0, lowest.min_z - ground_z);

        CraneHookNearestCluster largest_conn;
        double max_metric = -1.0;
        for (size_t ni = 1; ni < chain.nodes.size(); ++ni) {
          const auto & n = chain.nodes[ni];
          const double area = computeOccupiedArea(n.point_indices, points, 0.05);
          const double len = std::max(n.max_x - n.min_x, n.max_y - n.min_y);
          const double wid = std::min(n.max_x - n.min_x, n.max_y - n.min_y);
          const double hei = n.max_z - n.min_z;
          const double metric = (area > 0.0) ? area : (len * wid);
          if (metric > max_metric) {
            max_metric = metric;
            largest_conn.valid = true;
            largest_conn.subcluster_id = n.subcluster_id;
            largest_conn.layer_id = n.layer_id;
            largest_conn.point_count = n.point_count;
            largest_conn.centroid_x = n.x;
            largest_conn.centroid_y = n.y;
            largest_conn.centroid_z = n.z;
            largest_conn.ref_x = n.x;
            largest_conn.ref_y = n.y;
            largest_conn.ref_z = n.z;
            largest_conn.min_x = n.min_x;
            largest_conn.max_x = n.max_x;
            largest_conn.min_y = n.min_y;
            largest_conn.max_y = n.max_y;
            largest_conn.min_z = n.min_z;
            largest_conn.max_z = n.max_z;
            largest_conn.length = len;
            largest_conn.width = wid;
            largest_conn.height = hei;
            largest_conn.area_xy = area;
            largest_conn.dxy = std::hypot(n.x - chain.root_x, n.y - chain.root_y);
            largest_conn.dz = chain.root_z - n.z;
            largest_conn.dist_3d = std::hypot(largest_conn.dxy, largest_conn.dz);
          }
        }
        chain.largest_cluster = largest_conn;

        frame_chains.push_back(std::move(chain));
      }
    }

    // 4.4 Temporal Consistency Tracking (M of N Window & ID Persistence)
    const size_t prev_track_count = tracked_hook_chains_.size();
    std::vector<bool> track_updated(prev_track_count, false);
    const int win_n = std::max(1, td_cfg.temporal_window_n);
    const int req_m = std::max(1, td_cfg.persistent_confirm_m);

    for (auto & chain : frame_chains) {
      int best_tr_idx = -1;
      double best_dist = std::numeric_limits<double>::infinity();

      for (size_t t = 0; t < prev_track_count; ++t) {
        if (track_updated[t]) continue;
        const auto & tr = tracked_hook_chains_[t];
        if (tr.side != chain.side) continue;

        const double d3 = std::hypot(chain.lowest_x - tr.last_x,
                                     chain.lowest_y - tr.last_y,
                                     chain.lowest_z - tr.last_z);
        if (d3 <= 0.85 && d3 < best_dist) {
          best_dist = d3;
          best_tr_idx = static_cast<int>(t);
        }
      }

      if (best_tr_idx >= 0) {
        track_updated[best_tr_idx] = true;
        auto & tr = tracked_hook_chains_[best_tr_idx];
        tr.history.push_back(true);
        if (static_cast<int>(tr.history.size()) > win_n) tr.history.pop_front();

        int hits = 0;
        for (bool h : tr.history) if (h) ++hits;

        tr.state = (hits >= req_m) ? CraneHookConnectionState::PERSISTENT :
                                     CraneHookConnectionState::CANDIDATE;
        tr.hit_count++;
        tr.lost_frames = 0;

        // Kalman Filter Update
        tr.kf_xy.predict(dt, 0.5);
        tr.kf_z.predict(dt, 1.5);
        tr.kf_xy.update(chain.lowest_x, chain.lowest_y, 0.03);
        tr.kf_z.update(chain.lowest_z, 0.0, 0.03);

        chain.chain_id = tr.chain_id;
        chain.state = tr.state;
        chain.persistent_frames = hits;
        chain.filtered_x = tr.kf_xy.x();
        chain.filtered_y = tr.kf_xy.y();
        chain.filtered_z = tr.kf_z.x();
        chain.vx = tr.kf_xy.vx();
        chain.vy = tr.kf_xy.vy();
        chain.vz = tr.kf_z.vx();

        tr.last_x = chain.filtered_x;
        tr.last_y = chain.filtered_y;
        tr.last_z = chain.filtered_z;
        tr.latest_chain = chain;
      } else {
        // New Hook Connection Track
        TrackedHookChain new_tr;
        new_tr.chain_id = next_hook_chain_id_++;
        new_tr.side = chain.side;
        new_tr.state = CraneHookConnectionState::CANDIDATE;
        new_tr.history.push_back(true);
        new_tr.hit_count = 1;
        new_tr.lost_frames = 0;
        new_tr.kf_xy.initialize(chain.lowest_x, chain.lowest_y, 0.03, 1.0);
        new_tr.kf_z.initialize(chain.lowest_z, 0.0, 0.03, 1.0);

        chain.chain_id = new_tr.chain_id;
        chain.state = new_tr.state;
        chain.persistent_frames = 1;
        chain.filtered_x = chain.lowest_x;
        chain.filtered_y = chain.lowest_y;
        chain.filtered_z = chain.lowest_z;

        new_tr.last_x = chain.filtered_x;
        new_tr.last_y = chain.filtered_y;
        new_tr.last_z = chain.filtered_z;
        new_tr.latest_chain = chain;
        tracked_hook_chains_.push_back(std::move(new_tr));
      }
    }

    // Handle coasting for unmatched tracks
    for (size_t t = prev_track_count; t > 0; --t) {
      const size_t idx = t - 1;
      if (!track_updated[idx]) {
        auto & tr = tracked_hook_chains_[idx];
        tr.history.push_back(false);
        if (static_cast<int>(tr.history.size()) > win_n) tr.history.pop_front();

        if (++tr.lost_frames <= td_cfg.max_lost_frames) {
          tr.kf_xy.predict(dt, 0.5);
          tr.kf_z.predict(dt, 1.5);
          tr.latest_chain.filtered_x = tr.kf_xy.x();
          tr.latest_chain.filtered_y = tr.kf_xy.y();
          tr.latest_chain.filtered_z = tr.kf_z.x();
          tr.latest_chain.vx = tr.kf_xy.vx();
          tr.latest_chain.vy = tr.kf_xy.vy();
          tr.latest_chain.vz = tr.kf_z.vx();
          frame_chains.push_back(tr.latest_chain);
        } else {
          tracked_hook_chains_.erase(tracked_hook_chains_.begin() + idx);
        }
      }
    }

    td_res.chains = std::move(frame_chains);

    // 4.5 Link Primary Left & Right Hook Results
    for (size_t ci = 0; ci < td_res.chains.size(); ++ci) {
      const auto & c = td_res.chains[ci];
      if (c.side == "LEFT" && (td_res.left_chain_idx < 0 || c.state == CraneHookConnectionState::PERSISTENT)) {
        td_res.left_chain_idx = static_cast<int>(ci);
        td_res.left_detected = true;
      } else if (c.side == "RIGHT" && (td_res.right_chain_idx < 0 || c.state == CraneHookConnectionState::PERSISTENT)) {
        td_res.right_chain_idx = static_cast<int>(ci);
        td_res.right_detected = true;
      }
    }

    // 4.6 Populate result_.hook for downstream interfaces
    if (td_res.left_chain_idx >= 0) {
      const auto & lc = td_res.chains[td_res.left_chain_idx];
      auto & hl = result_.hook.left;
      hl.detected = true;
      hl.x = lc.lowest_x; hl.y = lc.lowest_y; hl.z = lc.lowest_z;
      hl.filtered_x = lc.filtered_x; hl.filtered_y = lc.filtered_y; hl.filtered_z = lc.filtered_z;
      hl.vx = lc.vx; hl.vy = lc.vy; hl.vz = lc.vz;
      hl.body_points = lc.lowest_point_count;
      hl.body_indices = lc.lowest_point_indices;
      hl.body_min = {lc.lowest_x - 0.2, lc.lowest_y - 0.2, lc.lowest_min_z};
      hl.body_max = {lc.lowest_x + 0.2, lc.lowest_y + 0.2, lc.lowest_max_z};
      hl.window_z_min = lc.lowest_min_z; hl.window_z_max = lc.lowest_max_z;
      hl.initialized = true;
      hl.tracking_state = to_string(lc.state);
      hl.position_valid = true;
      hl.position_tracking_state = to_string(lc.state);
      hl.tracking_x = lc.filtered_x; hl.tracking_y = lc.filtered_y; hl.tracking_z = lc.filtered_z;
      hl.ground_valid = true;
      hl.ground_z = ground_z;
      hl.ground_distance = lc.ground_distance;
      if (result_.beam.detected) {
        hl.beam_distance_valid = true;
        hl.beam_reference_z = result_.beam.centroid_z;
        hl.beam_dz = lc.lowest_z - result_.beam.centroid_z;
      }
    }

    if (td_res.right_chain_idx >= 0) {
      const auto & rc = td_res.chains[td_res.right_chain_idx];
      auto & hr = result_.hook.right;
      hr.detected = true;
      hr.x = rc.lowest_x; hr.y = rc.lowest_y; hr.z = rc.lowest_z;
      hr.filtered_x = rc.filtered_x; hr.filtered_y = rc.filtered_y; hr.filtered_z = rc.filtered_z;
      hr.vx = rc.vx; hr.vy = rc.vy; hr.vz = rc.vz;
      hr.body_points = rc.lowest_point_count;
      hr.body_indices = rc.lowest_point_indices;
      hr.body_min = {rc.lowest_x - 0.2, rc.lowest_y - 0.2, rc.lowest_min_z};
      hr.body_max = {rc.lowest_x + 0.2, rc.lowest_y + 0.2, rc.lowest_max_z};
      hr.window_z_min = rc.lowest_min_z; hr.window_z_max = rc.lowest_max_z;
      hr.initialized = true;
      hr.tracking_state = to_string(rc.state);
      hr.position_valid = true;
      hr.position_tracking_state = to_string(rc.state);
      hr.tracking_x = rc.filtered_x; hr.tracking_y = rc.filtered_y; hr.tracking_z = rc.filtered_z;
      hr.ground_valid = true;
      hr.ground_z = ground_z;
      hr.ground_distance = rc.ground_distance;
      if (result_.beam.detected) {
        hr.beam_distance_valid = true;
        hr.beam_reference_z = result_.beam.centroid_z;
        hr.beam_dz = rc.lowest_z - result_.beam.centroid_z;
      }
    }

    // 4.7 Combined Readiness
    const auto & hl = result_.hook.left;
    const auto & hr = result_.hook.right;
    if (hl.detected && hr.detected) {
      td_res.mean_ground_distance = 0.5 * (hl.ground_distance + hr.ground_distance);
      td_res.prepare_ready = (td_res.mean_ground_distance <= td_cfg.prepare_distance &&
                              hl.tracking_state == "PERSISTENT" && hr.tracking_state == "PERSISTENT");
    } else if (hl.detected) {
      td_res.mean_ground_distance = hl.ground_distance;
    } else if (hr.detected) {
      td_res.mean_ground_distance = hr.ground_distance;
    }
    result_.hook.mean_ground_distance = td_res.mean_ground_distance;
    result_.hook.prepare_ready = td_res.prepare_ready;

    // 4.8 Track Root Motion in X and Y (crane_moving)
    root_move_count_=0;root_move_ids_={-1,-1};
    td_res.left_root_state = left_fixed_root_state_;
    td_res.right_root_state = right_fixed_root_state_;

    if (config_.fixed_root.enabled) {
      if (left_fixed_root_state_.motion_status == CraneRootMotionStatus::MOVING ||
          right_fixed_root_state_.motion_status == CraneRootMotionStatus::MOVING) {
        crane_moving_ = true;
        crane_move_reason_ = (left_fixed_root_state_.motion_status == CraneRootMotionStatus::MOVING &&
                              right_fixed_root_state_.motion_status == CraneRootMotionStatus::MOVING)
                                 ? "MOVING_DUAL_ROOT"
                                 : (left_fixed_root_state_.motion_status == CraneRootMotionStatus::MOVING ? "MOVING_LEFT_ROOT" : "MOVING_RIGHT_ROOT");
      } else if (left_fixed_root_state_.motion_status == CraneRootMotionStatus::STATIONARY ||
                 right_fixed_root_state_.motion_status == CraneRootMotionStatus::STATIONARY) {
        crane_moving_ = false;
        crane_move_reason_ = "STOPPED_ROOT_STATIONARY";
      } else {
        crane_moving_ = false;
        crane_move_reason_ = "STOPPED_ROOT_UNKNOWN";
      }
      td_res.root_mean_x = 0.5 * (left_fixed_root_state_.fixed_center_x + right_fixed_root_state_.fixed_center_x);
      td_res.root_mean_y = 0.5 * (left_fixed_root_state_.fixed_center_y + right_fixed_root_state_.fixed_center_y);
      td_res.root_mean_z = 0.5 * (left_fixed_root_state_.fixed_center_z + right_fixed_root_state_.fixed_center_z);
      td_res.root_dx = 0.5 * (left_fixed_root_state_.delta_x + right_fixed_root_state_.delta_x);
      td_res.root_dy = 0.5 * (left_fixed_root_state_.delta_y + right_fixed_root_state_.delta_y);
      td_res.root_vx = 0.5 * (left_fixed_root_state_.velocity_xy + right_fixed_root_state_.velocity_xy);
      td_res.root_vy = 0.0;
    } else {
      double left_dx = 0.0, left_dy = 0.0, left_speed = 0.0;
      bool left_active = false;
      if (left_root_node_.initialized) {
        if (!left_root_node_.anchor_valid) {
          left_root_node_.anchor_x = left_root_node_.smoothed_x;
          left_root_node_.anchor_y = left_root_node_.smoothed_y;
          left_root_node_.anchor_valid = true;
        }
        left_dx = left_root_node_.smoothed_x - left_root_node_.anchor_x;
        left_dy = left_root_node_.smoothed_y - left_root_node_.anchor_y;
        left_speed = std::hypot(left_root_node_.kf_xy.vx(), left_root_node_.kf_xy.vy());
        left_active = true;
      }

      double right_dx = 0.0, right_dy = 0.0, right_speed = 0.0;
      bool right_active = false;
      if (right_root_node_.initialized) {
        if (!right_root_node_.anchor_valid) {
          right_root_node_.anchor_x = right_root_node_.smoothed_x;
          right_root_node_.anchor_y = right_root_node_.smoothed_y;
          right_root_node_.anchor_valid = true;
        }
        right_dx = right_root_node_.smoothed_x - right_root_node_.anchor_x;
        right_dy = right_root_node_.smoothed_y - right_root_node_.anchor_y;
        right_speed = std::hypot(right_root_node_.kf_xy.vx(), right_root_node_.kf_xy.vy());
        right_active = true;
      }

      if(current_root_indices[0]>=0 && left_active && left_root_node_.lost_frames==0 &&
          (std::abs(left_dx)>=td_cfg.root_move_start_x || std::abs(left_dy)>=td_cfg.root_move_start_y) &&
          left_speed>=td_cfg.root_move_velocity_thresh){++root_move_count_;root_move_ids_[0]=current_root_indices[0];}
      if(current_root_indices[1]>=0 && current_root_indices[1]!=current_root_indices[0] && right_active && right_root_node_.lost_frames==0 &&
          (std::abs(right_dx)>=td_cfg.root_move_start_x || std::abs(right_dy)>=td_cfg.root_move_start_y) &&
          right_speed>=td_cfg.root_move_velocity_thresh){++root_move_count_;root_move_ids_[1]=current_root_indices[1];}
      if (left_active || right_active) {
        double net_dx = 0.0, net_dy = 0.0, net_vx = 0.0, net_vy = 0.0;
        double root_speed = 0.0;
        bool move_x = false, move_y = false;

        double mean_root_x = 0.0, mean_root_y = 0.0, mean_root_z = 0.0;
        if (left_active && right_active) {
          mean_root_x = 0.5 * (left_root_node_.smoothed_x + right_root_node_.smoothed_x);
          mean_root_y = 0.5 * (left_root_node_.smoothed_y + right_root_node_.smoothed_y);
          mean_root_z = 0.5 * (left_root_node_.smoothed_z + right_root_node_.smoothed_z);
          net_dx = 0.5 * (left_dx + right_dx);
          net_dy = 0.5 * (left_dy + right_dy);
          net_vx = 0.5 * (left_root_node_.kf_xy.vx() + right_root_node_.kf_xy.vx());
          net_vy = 0.5 * (left_root_node_.kf_xy.vy() + right_root_node_.kf_xy.vy());
          root_speed = std::hypot(net_vx, net_vy);

          const bool agree_dir_x = (left_dx * right_dx > 0.0) &&
              std::abs(left_dx) >= 0.5 * td_cfg.root_move_start_x &&
              std::abs(right_dx) >= 0.5 * td_cfg.root_move_start_x;
          const bool agree_dir_y = (left_dy * right_dy > 0.0) &&
              std::abs(left_dy) >= 0.5 * td_cfg.root_move_start_y &&
              std::abs(right_dy) >= 0.5 * td_cfg.root_move_start_y;
          const bool mag_x = (std::abs(left_dx) >= td_cfg.root_move_start_x || std::abs(right_dx) >= td_cfg.root_move_start_x);
          const bool mag_y = (std::abs(left_dy) >= td_cfg.root_move_start_y || std::abs(right_dy) >= td_cfg.root_move_start_y);

          move_x = (std::abs(net_dx) >= td_cfg.root_move_start_x) && mag_x && agree_dir_x;
          move_y = (std::abs(net_dy) >= td_cfg.root_move_start_y) && mag_y && agree_dir_y;
        } else if (left_active) {
          mean_root_x = left_root_node_.smoothed_x;
          mean_root_y = left_root_node_.smoothed_y;
          mean_root_z = left_root_node_.smoothed_z;
          net_dx = left_dx;
          net_dy = left_dy;
          net_vx = left_root_node_.kf_xy.vx();
          net_vy = left_root_node_.kf_xy.vy();
          root_speed = left_speed;
          move_x = std::abs(net_dx) >= td_cfg.root_move_start_x;
          move_y = std::abs(net_dy) >= td_cfg.root_move_start_y;
        } else {
          mean_root_x = right_root_node_.smoothed_x;
          mean_root_y = right_root_node_.smoothed_y;
          mean_root_z = right_root_node_.smoothed_z;
          net_dx = right_dx;
          net_dy = right_dy;
          net_vx = right_root_node_.kf_xy.vx();
          net_vy = right_root_node_.kf_xy.vy();
          root_speed = right_speed;
          move_x = std::abs(net_dx) >= td_cfg.root_move_start_x;
          move_y = std::abs(net_dy) >= td_cfg.root_move_start_y;
        }

        td_res.root_mean_x = mean_root_x;
        td_res.root_mean_y = mean_root_y;
        td_res.root_mean_z = mean_root_z;
        td_res.root_dx = net_dx;
        td_res.root_dy = net_dy;
        td_res.root_vx = net_vx;
        td_res.root_vy = net_vy;

        root_position_history_.push_back({net_dx, net_dy});
        if (root_position_history_.size() > static_cast<size_t>(td_cfg.root_move_stop_frames)) {
          root_position_history_.pop_front();
        }

        double min_span_x = net_dx, max_span_x = net_dx;
        double min_span_y = net_dy, max_span_y = net_dy;
        for (const auto & pos : root_position_history_) {
          min_span_x = std::min(min_span_x, pos.first);
          max_span_x = std::max(max_span_x, pos.first);
          min_span_y = std::min(min_span_y, pos.second);
          max_span_y = std::max(max_span_y, pos.second);
        }
        const double span_x = max_span_x - min_span_x;
        const double span_y = max_span_y - min_span_y;

        if (!crane_moving_ && (move_x || move_y) && root_speed >= td_cfg.root_move_velocity_thresh) {
          crane_moving_ = true;
          crane_move_reason_ = move_x ? (move_y ? "START_ROOT_XY" : "START_ROOT_X") : "START_ROOT_Y";
          root_stable_frames_ = 0;
        } else if (crane_moving_) {
          if (root_position_history_.size() >= static_cast<size_t>(td_cfg.root_move_stop_frames) &&
              span_x <= 0.05 && span_y <= 0.05) {
            crane_moving_ = false;
            crane_move_reason_ = "STOP_ROOT_STABLE";
            if (left_active) {
              left_root_node_.anchor_x = left_root_node_.smoothed_x;
              left_root_node_.anchor_y = left_root_node_.smoothed_y;
            }
            if (right_active) {
              right_root_node_.anchor_x = right_root_node_.smoothed_x;
              right_root_node_.anchor_y = right_root_node_.smoothed_y;
            }
          }
        }
      }
    }
    td_res.crane_moving = crane_moving_;
    td_res.crane_move_reason = crane_move_reason_;
    result_.crane_moving = crane_moving_;

    td_res.processing_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t_hook_begin).count();
  }

  result_.work_status = CraneWorkStatusResult{};
  result_.working_status = 0;

  // 4.9 Nearest Cluster Search Below Each Hook (Left & Right)
  // Note: This finds the spatially nearest cluster below each Hook inside the Hook ROI
  // without judging cargo attachment or changing work_status.
  {
    std::unordered_set<int> excluded_chain_subclusters;
    for (const auto & chain : td_res.chains) {
      if (chain.root_subcluster_id >= 0) {
        excluded_chain_subclusters.insert(chain.root_subcluster_id);
      }
      for (const auto & n : chain.nodes) {
        if (n.subcluster_id >= 0) {
          excluded_chain_subclusters.insert(n.subcluster_id);
        }
      }
      // Exclude split hook/wire subclusters inside the vertical chain column above lowest_min_z
      for (const auto & sc : subclusters) {
        if (sc.centroid_z < chain.lowest_min_z - 0.05 || sc.centroid_z > chain.root_z + 0.15) continue;
        if ((sc.max_x - sc.min_x) > 0.55 || (sc.max_y - sc.min_y) > 0.55) continue;
        const double dxy = std::hypot(sc.centroid_x - chain.lowest_x, sc.centroid_y - chain.lowest_y);
        if (dxy <= 0.35) {
          excluded_chain_subclusters.insert(sc.subcluster_id);
        }
      }
    }

    if(!config_.work_status.fsm_enabled || beamWorkProcessingAllowed()) for (int side = 0; side < 2; ++side) {
      const int chain_idx = (side == 0) ? td_res.left_chain_idx : td_res.right_chain_idx;
      if (chain_idx < 0 || chain_idx >= static_cast<int>(td_res.chains.size())) continue;

      auto & chain = td_res.chains[chain_idx];
      const double hook_x = chain.lowest_x;
      const double hook_y = chain.lowest_y;
      const double hook_z = chain.lowest_z;
      const double hook_min_z = chain.lowest_min_z;

      const double base_roi_x_min = (side == 0) ? td_cfg.roi_left_x_min : td_cfg.roi_right_x_min;
      const double base_roi_x_max = (side == 0) ? td_cfg.roi_left_x_max : td_cfg.roi_right_x_max;
      const double eff_x_min = std::min(base_roi_x_min, hook_x - 0.45);
      const double eff_x_max = std::max(base_roi_x_max, hook_x + 0.45);
      const double eff_y_min = std::min(td_cfg.roi_y_min, hook_y - 0.45);
      const double eff_y_max = std::max(td_cfg.roi_y_max, hook_y + 0.45);

      const CraneSubcluster * best_sc = nullptr;
      double best_dist_3d = std::numeric_limits<double>::infinity();
      double best_dxy = 0.0;
      double best_dz = 0.0;
      double best_ref_x = 0.0, best_ref_y = 0.0, best_ref_z = 0.0;

      for (const auto & sc : subclusters) {
        if (sc.point_indices.size() < td_cfg.min_subcluster_points) continue;
        if (excluded_chain_subclusters.count(sc.subcluster_id)) continue;

        // Exclude flat ground residue
        if (sc.min_z <= ground_z + 0.15 || sc.centroid_z <= ground_z + 0.20) continue;

        // Candidate must be below the Hook
        if (sc.centroid_z >= hook_z - 0.05 || sc.max_z > hook_min_z + 0.08) continue;

        // Reject fragments physically inside the Hook body
        const double center_dxy = std::hypot(sc.centroid_x - hook_x, sc.centroid_y - hook_y);
        if (center_dxy <= 0.25 && sc.centroid_z >= hook_min_z - 0.05 && sc.centroid_z <= hook_z + 0.15) {
          continue;
        }

        const double dz_gap = std::max(0.0, hook_min_z - sc.max_z);
        // Reject floor-hugging clutter beneath a high parked empty hook
        if (dz_gap > 2.0 && sc.min_z < ground_z + 0.35) {
          continue;
        }

        // Horizontal distance from Hook to cluster Bounding Box
        const double bbox_dx = std::max({0.0, sc.min_x - hook_x, hook_x - sc.max_x});
        const double bbox_dy = std::max({0.0, sc.min_y - hook_y, hook_y - sc.max_y});
        const double bbox_dxy = std::hypot(bbox_dx, bbox_dy);

        const bool overlaps_hook_roi =
            (sc.max_x >= eff_x_min && sc.min_x <= eff_x_max &&
             sc.max_y >= eff_y_min && sc.min_y <= eff_y_max);
        if (!overlaps_hook_roi && bbox_dxy > 0.40) continue;
        if (bbox_dxy > 0.50) continue;

        // Compute local reference point of the cluster beneath (hook_x, hook_y)
        double ref_x = sc.centroid_x;
        double ref_y = sc.centroid_y;
        double ref_z = sc.centroid_z;
        const double span_x = sc.max_x - sc.min_x;
        const double span_y = sc.max_y - sc.min_y;
        if ((span_x > 0.60 || span_y > 0.60) && !sc.point_indices.empty()) {
          double sx = 0.0, sy = 0.0, sz = 0.0;
          size_t local_cnt = 0;
          double min_pt_dxy = std::numeric_limits<double>::infinity();
          double close_x = sc.centroid_x, close_y = sc.centroid_y, close_z = sc.centroid_z;
          for (int idx : sc.point_indices) {
            if (idx < 0 || static_cast<size_t>(idx) >= points.size()) continue;
            const auto & p = points[idx];
            const double pdxy = std::hypot(static_cast<double>(p.x) - hook_x,
                                           static_cast<double>(p.y) - hook_y);
            if (pdxy <= 0.45) {
              sx += p.x; sy += p.y; sz += p.z;
              ++local_cnt;
            }
            if (pdxy < min_pt_dxy) {
              min_pt_dxy = pdxy;
              close_x = p.x; close_y = p.y; close_z = p.z;
            }
          }
          if (local_cnt > 0) {
            ref_x = sx / static_cast<double>(local_cnt);
            ref_y = sy / static_cast<double>(local_cnt);
            ref_z = sz / static_cast<double>(local_cnt);
          } else {
            ref_x = close_x;
            ref_y = close_y;
            ref_z = close_z;
          }
        }

        const double dxy = std::hypot(ref_x - hook_x, ref_y - hook_y);
        const double dz = (hook_min_z >= ref_z) ? (hook_min_z - ref_z) : std::max(0.0, hook_z - ref_z);

        // 3D combined distance (prevents selecting distant floor clusters that happen to have small dXY)
        const double eff_xy = 1.2 * bbox_dxy + 0.5 * dxy;
        const double eff_z = 0.6 * dz_gap + 0.4 * dz;
        const double dist_3d = std::hypot(eff_xy, eff_z);

        if (dist_3d < best_dist_3d) {
          best_dist_3d = dist_3d;
          best_sc = &sc;
          best_dxy = dxy;
          best_dz = dz;
          best_ref_x = ref_x;
          best_ref_y = ref_y;
          best_ref_z = ref_z;
        }
      }

      if (best_sc) {
        CraneHookNearestCluster nc;
        nc.valid = true;
        nc.subcluster_id = best_sc->subcluster_id;
        nc.layer_id = best_sc->layer_id;
        nc.group_id = best_sc->group_id;
        nc.point_count = best_sc->point_indices.size();
        nc.centroid_x = best_sc->centroid_x;
        nc.centroid_y = best_sc->centroid_y;
        nc.centroid_z = best_sc->centroid_z;
        nc.ref_x = best_ref_x;
        nc.ref_y = best_ref_y;
        nc.ref_z = best_ref_z;
        nc.min_x = best_sc->min_x;
        nc.max_x = best_sc->max_x;
        nc.min_y = best_sc->min_y;
        nc.max_y = best_sc->max_y;
        nc.min_z = best_sc->min_z;
        nc.max_z = best_sc->max_z;
        nc.length = std::max(best_sc->max_x - best_sc->min_x, best_sc->max_y - best_sc->min_y);
        nc.width = std::min(best_sc->max_x - best_sc->min_x, best_sc->max_y - best_sc->min_y);
        nc.height = best_sc->max_z - best_sc->min_z;
        nc.area_xy = computeOccupiedArea(best_sc->point_indices, points, 0.05);
        nc.dxy = best_dxy;
        nc.dz = best_dz;
        nc.dist_3d = best_dist_3d;

        chain.nearest_cluster = nc;
        if (!chain.largest_cluster.valid) {
          chain.largest_cluster = nc;
        }
        if (side == 0) {
          td_res.left_nearest_cluster = nc;
          result_.hook.left.beam_distance_valid = true;
          result_.hook.left.beam_reference_z = nc.centroid_z;
          result_.hook.left.beam_dz = nc.dz;
        } else {
          td_res.right_nearest_cluster = nc;
          result_.hook.right.beam_distance_valid = true;
          result_.hook.right.beam_reference_z = nc.centroid_z;
          result_.hook.right.beam_dz = nc.dz;
        }
      }
    }

    if (td_res.left_chain_idx >= 0 && td_res.left_chain_idx < static_cast<int>(td_res.chains.size())) {
      td_res.left_largest_cluster = td_res.chains[td_res.left_chain_idx].largest_cluster;
    }
    if (td_res.right_chain_idx >= 0 && td_res.right_chain_idx < static_cast<int>(td_res.chains.size())) {
      td_res.right_largest_cluster = td_res.chains[td_res.right_chain_idx].largest_cluster;
    }

    // Top-Down Chain & Largest Connected Cluster Temporal Stabilization
    stabilizeTopDownStructure(td_res, subclusters, points, crane_moving_, td_res.root_dx, td_res.root_dy);

    // Size-based 1st Exception Gate with FOV/Visibility Integration
    evaluateExceptionGate(td_res, points, visibility_model_ptr_);

    // Wire state, cargo gate and downstream hook policy are complete before Housing runs.
    updateSheaveTracking(td_res, subclusters, points, stamp_sec, current_observation_points);
    updateSheaveMotion(td_res,stamp_sec);
    observeOrderedHookRoi(current_observation_points?*current_observation_points:points,stamp_sec);
    if(config_.work_status.fsm_enabled || config_.work_status.boundary_motion_enabled || config_.sheave.enabled)
      updateWorkStatus(current_observation_points?*current_observation_points:points,subclusters,stamp_sec);
    result_.crane_moving=confirmedCraneMoving();
    td_res.crane_moving=result_.crane_moving;
  }

  return result_;
}

static bool hasCargoSurfacePatch(const std::vector<PointXYZIRT> & raw,const std::vector<int> & ids,
    double x,double y,double yaw,double resolution,double min_width,
    double min_vertical_span=0,size_t min_vertical_columns=0,double min_occupied_area=0) {
  // Membership alone matters here: preserve the same cells and patch tests,
  // but store them contiguously instead of allocating a tree node per cell.
  const auto key=[](int u,int v){return (uint64_t(uint32_t(u))<<32)|uint32_t(v);};
  std::vector<uint64_t> occupied;occupied.reserve(ids.size());
  std::map<uint64_t,std::array<double,2>> vertical;
  const double cs=std::cos(yaw),sn=std::sin(yaw);
  for(int id:ids){const auto & p=raw[id];
    const double u=(p.x-x)*cs+(p.y-y)*sn;
    const double v=-(p.x-x)*sn+(p.y-y)*cs;
    auto cell=key(int(std::floor(u/resolution)),int(std::floor(v/resolution)));occupied.push_back(cell);
    if(min_vertical_columns){auto pz=vertical.emplace(cell,std::array<double,2>{p.z,p.z});
      pz.first->second[0]=std::min(pz.first->second[0],double(p.z));pz.first->second[1]=std::max(pz.first->second[1],double(p.z));}
  }
  if(min_vertical_columns){size_t count=0;for(const auto & cell:vertical)if(cell.second[1]-cell.second[0]>=min_vertical_span)++count;
    if(count<min_vertical_columns)return false;}
  std::sort(occupied.begin(),occupied.end());
  occupied.erase(std::unique(occupied.begin(),occupied.end()),occupied.end());
  if(occupied.size()*resolution*resolution<min_occupied_area)return false;
  const auto contains=[&](int u,int v){return std::binary_search(occupied.begin(),occupied.end(),key(u,v));};
  const int span=std::max(2,int(std::ceil(min_width/resolution)));
  int patches=0;for(uint64_t c:occupied){
    const int u=int32_t(c>>32),v=int32_t(uint32_t(c));
    if(contains(u+1,v) && contains(u,v+1) && contains(u+1,v+1))++patches;
  }
  if(patches<(span-1)*(span-1))return false;
  for(uint64_t c:occupied){bool filled=true;
    const int cu=int32_t(c>>32),cv=int32_t(uint32_t(c));
    for(int u=0;u<span && filled;++u)for(int v=0;v<span;++v)
      if(!contains(cu+u,cv+v)){filled=false;break;}
    if(filled)return true;
  }
  return false;
}


void CranePipeline::resetOrderedWork(const char * reason){
  std::clog<<"[Crane ordered RESET] "<<reason<<"\n";
  ordered_state_=CraneBeamState::IDLE;ordered_side_=ordered_sheave_id_=-1;
  ordered_last_roi_=-1;ordered_completed_=-1;ordered_stop_count_=ordered_lower_count_=0;
  ordered_lower_accum_=0;ordered_last_seed_stamp_=-1;
  for(auto & v:ordered_roi_samples_)v.clear();ordered_roi_ids_={-1,-1};
  work_cycle_=CraneWorkStatusResult{};cargo_hits_=0;cargo_last_seen_=0;cargo_observed_sec_=0;
  cargo_outside_hits_=0;drop_started_stamp_=cargo_peak_z_=gap_last_stamp_=gap_observed_sec_=0;
  cargo_descent_steps_=gap_hits_=0;cargo_descent_seen_=cargo_stationary_current_=false;
  work_elapsed_current_=0;last_gap_candidate_="UNKNOWN";resetDropCandidate();

  cargo_xy_filter_=LinearKalmanFilter{};cargo_z_filter_=LinearKalmanFilter{};cargo_owned_points_.clear();cargo_owned_stamp_=cargo_recovery_stamp_=-1;cargo_owned_id_=-1;cargo_recovery_indices_.clear();
  left_work_state_=TrackedHookWorkState{};right_work_state_=TrackedHookWorkState{};
  field_drop_elapsed_={0,0};field_drop_last_={-1,-1};completion_last_stamp_=0;
}
void CranePipeline::projectOrderedState(){
  auto & w=work_cycle_;w.fsm_enabled=config_.work_status.fsm_enabled;w.fsm_state=ordered_state_;
  const int code=ordered_state_==CraneBeamState::LIFTING?3:
    (ordered_state_==CraneBeamState::LOWERING_CONFIRMED?2:
      ((ordered_state_==CraneBeamState::SUSPENDED || ordered_state_==CraneBeamState::LOWERING)?1:0));
  w.working_status=w.public_working_status=code;w.work_status=code==1 || code==3;
}
void CranePipeline::observeOrderedHookRoi(const std::vector<PointXYZIRT> & raw,double stamp){
  if(!config_.work_status.fsm_enabled || !std::isfinite(stamp) || stamp<=0)return;
  if(ordered_stamp_>=0 && stamp<=ordered_stamp_){if(stamp<ordered_stamp_)resetOrderedWork("SENSOR_TIME_REWIND");return;}
  ordered_stamp_=stamp;
  if(ordered_state_==CraneBeamState::LOWERING_CONFIRMED){
    if(stamp-ordered_completed_>=3.)resetOrderedWork("COMPLETION_HOLD_FINISHED");
    return;
  }
  if(beamWorkProcessingAllowed()){
    if(ordered_side_>=0){
      const auto & h=ordered_side_==0?result_.top_down_hook.left_housing:result_.top_down_hook.right_housing;
      const auto & m=ordered_side_==0?result_.top_down_hook.left_housing_memory:result_.top_down_hook.right_housing_memory;
      if(h.sheave.status==CraneSheaveDetectionStatus::DIRECT_OBSERVED && m.confirmed && m.track_id!=ordered_sheave_id_){
        resetOrderedWork("SUPPORTING_SHEAVE_ID_CHANGED");return;
      }
    }
    if(cargo_hits_==0 && ordered_last_roi_>0 && stamp-ordered_last_roi_>config_.work_status.cargo_loss_sec)
      resetOrderedWork("SUSPENDED_BEAM_ACQUISITION_FAILED");
    return;
  } // Beam tracking owns current observations after admission.
  const auto & cfg=config_.work_status;const double noise=config_.dual_roi_anchor.noise_deadband_z;
  bool active_observed=false;
  for(int side=0;side<2;++side){
    if(ordered_side_>=0 && ordered_side_!=side)continue;
    const auto & node=side==0?result_.top_down_hook.left_housing:result_.top_down_hook.right_housing;
    const auto & mem=side==0?result_.top_down_hook.left_housing_memory:result_.top_down_hook.right_housing_memory;
    auto & samples=ordered_roi_samples_[side];
    if(ordered_roi_ids_[side]!=mem.track_id){samples.clear();ordered_roi_ids_[side]=mem.track_id;}
    if(node.sheave.status!=CraneSheaveDetectionStatus::DIRECT_OBSERVED || !node.sheave.points_from_current_frame ||
       !mem.confirmed || node.is_held){samples.clear();continue;}
    const double hx=node.x,hy=node.y,hz=node.z-cfg.virtual_hook_offset_z_m;
    std::vector<int> ids;std::vector<float> zs;
    for(size_t i=0;i<raw.size();++i){const auto & p=raw[i];
      if(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z)&&
         std::abs(p.x-hx)<=cfg.cargo_roi_width_x_m*.5 && std::abs(p.y-hy)<=cfg.cargo_roi_width_y_m*.5 &&
         p.z<hz-cfg.cargo_start_offset_m && p.z>=hz-cfg.connect_max_gap_z){ids.push_back(int(i));zs.push_back(p.z);}
    }
    // Local spatial support only: no clustering, PCA, full Beam bounds or geometry here.
    if(ids.size()<cfg.min_candidate_points || !hasCargoSurfacePatch(raw,ids,hx,hy,0,cfg.grid_resolution_m,cfg.cargo_min_surface_width_m,
        noise,size_t(std::max(1,config_.z_gaussian.min_layer_points)),config_.exception_gate.min_area_m2)){samples.clear();continue;}
    std::sort(zs.begin(),zs.end());const double q10=zs[zs.size()/10],q50=zs[zs.size()/2],q90=zs[zs.size()*9/10];
    if(!samples.empty() && (stamp-samples.back()[0]>.25 || stamp-samples.back()[0]<=0))samples.clear();
    samples.push_back({stamp,q10,q50,q90});while(!samples.empty() && stamp-samples.front()[0]>cfg.cargo_confirm_sec)samples.pop_front();
    if(ordered_side_==side){active_observed=true;ordered_last_roi_=stamp;}
    if(ordered_state_==CraneBeamState::IDLE && samples.size()>=size_t(std::max(4,cfg.confirm_m_on))){
      const auto & first=samples.front();const double d0=q10-first[1],d1=q50-first[2],d2=q90-first[3];
      const double residual=std::max({d0,d1,d2})-std::min({d0,d1,d2});
      if(sheave_motion_state_[side]==2 && sheave_motion_id_[side]==mem.track_id &&
         d0>=config_.dual_roi_anchor.relocate_thresh_z && d1>=config_.dual_roi_anchor.relocate_thresh_z &&
         d2>=config_.dual_roi_anchor.relocate_thresh_z && residual<=noise){
        ordered_state_=CraneBeamState::LIFTING;ordered_side_=side;ordered_sheave_id_=mem.track_id;
        ordered_start_z_=first[2];ordered_peak_z_=q50;ordered_previous_z_=q50;
        ordered_last_roi_=stamp;ordered_stop_count_=0;active_observed=true;
        std::clog<<"[Crane ordered] IDLE -> LIFTING current supported Raw ROI and Sheave rise\n";
      }
    }else if(ordered_state_==CraneBeamState::LIFTING && ordered_side_==side){
      if(mem.track_id!=ordered_sheave_id_){resetOrderedWork("SUPPORTING_SHEAVE_ID_CHANGED");return;}
      ordered_peak_z_=std::max(ordered_peak_z_,q50);
      // Pre-cluster SUSPENDED admission is a conservative observed-body candidate:
      // directly seen local lower band and measured ground are mandatory.
      std::vector<float> ground;
      for(int i:result_.ground)if(i>=0 && size_t(i)<raw.size() && std::abs(raw[i].x-hx)<=cfg.cargo_roi_width_x_m*.5 &&
          std::abs(raw[i].y-hy)<=cfg.cargo_roi_width_y_m*.5 && raw[i].z<q10)ground.push_back(raw[i].z);
      float gz=0,mad=0;const bool airborne=ground.size()>=15 && CraneBeamDetector::computeMedianAndMAD(ground,gz,mad) &&
        mad<=noise && q10-gz>.38;
      const bool stopped=sheave_motion_state_[side]==1 && std::abs(q50-ordered_previous_z_)<.02;
      ordered_stop_count_=stopped && airborne && ordered_peak_z_-ordered_start_z_>=config_.dual_roi_anchor.relocate_thresh_z?
        ordered_stop_count_+1:0;
      ordered_previous_z_=q50;
      if(ordered_stop_count_>=3){ordered_state_=CraneBeamState::SUSPENDED;ordered_last_seed_stamp_=-1;
        std::clog<<"[Crane ordered] LIFTING -> SUSPENDED observed raised local surface + actual ground; Beam still unconfirmed\n";}
    }
  }
  if(ordered_state_==CraneBeamState::LIFTING && !active_observed && ordered_last_roi_>0 && stamp-ordered_last_roi_>=1.3)
    resetOrderedWork("HOOK_ROI_LOST_RESET_NOT_COMPLETION");
}

// Work Status is a downstream consumer. Never feed its motion or ROI back into Wire,
// accumulation, Housing memory, or the existing Beam state machine.
void CranePipeline::updateWorkStatus(const std::vector<PointXYZIRT> & raw,
    const std::vector<CraneSubcluster> & subclusters,double stamp) {
  const auto begin=std::chrono::steady_clock::now();
  const auto & cfg=config_.work_status;
  if(!cfg.fsm_enabled && !cfg.boundary_motion_enabled && !config_.sheave.enabled)return;
  if(!std::isfinite(stamp) || stamp<=0)return;
  auto reset_cargo=[&](){
    const auto motion=work_cycle_;
    work_cycle_=CraneWorkStatusResult{};
    drop_started_stamp_=cargo_peak_z_=gap_last_stamp_=gap_observed_sec_=0;resetDropCandidate();
    cargo_descent_steps_=gap_hits_=0;cargo_descent_seen_=cargo_stationary_current_=false;work_elapsed_current_=completion_last_stamp_=0;last_gap_candidate_="UNKNOWN";
    work_cycle_.boundary_motion_enabled=motion.boundary_motion_enabled;
    work_cycle_.boundary_points=motion.boundary_points;work_cycle_.boundary_moving=motion.boundary_moving;
    work_cycle_.boundary_reason=motion.boundary_reason;
    work_cycle_.boundary_raw_evidence=motion.boundary_raw_evidence;
    work_cycle_.boundary_raw_supported=motion.boundary_raw_supported;
    work_cycle_.boundary_start_elapsed=motion.boundary_start_elapsed;
    work_cycle_.boundary_stop_elapsed=motion.boundary_stop_elapsed;
    work_cycle_.boundary_transition_stamp=motion.boundary_transition_stamp;
    work_cycle_.boundary_temporal_us=motion.boundary_temporal_us;
    // Cargo reset must not erase this frame's independent Housing consumer.
    work_cycle_.left_hook=motion.left_hook;work_cycle_.right_hook=motion.right_hook;
    work_cycle_.cargo_detection_rois=motion.cargo_detection_rois;
    work_cycle_.sheave_visibility=motion.sheave_visibility;
    for(auto * hook:{&work_cycle_.left_hook,&work_cycle_.right_hook}){
      hook->work_status=false;hook->display_state="WAITING";
      hook->start_evidence="NO_COHERENT_CURRENT_Z_RISE";hook->hook_rise=hook->cargo_rise=0;
    }
    cargo_xy_filter_=LinearKalmanFilter{};
    cargo_z_filter_=LinearKalmanFilter{};cargo_owned_points_.clear();cargo_owned_stamp_=cargo_recovery_stamp_=-1;cargo_owned_id_=-1;cargo_recovery_indices_.clear();cargo_hits_=cargo_outside_hits_=0;
    cargo_observed_sec_=0;cargo_last_seen_=0;
    left_work_state_=TrackedHookWorkState{};right_work_state_=TrackedHookWorkState{};
  };
  const double elapsed=work_last_stamp_>0?stamp-work_last_stamp_:0;
  if(work_last_stamp_>0 && elapsed<=0){
    // Duplicate/reversed stamps never add confirmation or movement evidence.
    if(elapsed<0){reset_cargo();boundary_filters_={};boundary_hits_={};boundary_last_seen_={};boundary_moving_=false;boundary_move_count_=0;boundary_moving_points_.clear();boundary_confirmation_.reset();work_last_stamp_=stamp;}
    result_.work_status=work_cycle_;
    if(elapsed<0){work_cycle_.boundary_moving=false;result_.work_status.boundary_moving=false;}
    result_.work_status.fsm_enabled=cfg.fsm_enabled;
    result_.work_status.boundary_motion_enabled=cfg.boundary_motion_enabled;
    result_.work_status.cargo_observation=work_cycle_.geometry_locked?"HELD":"NONE";
    for(int side=0;side<2;++side){
      auto & hook=side==0?result_.work_status.left_hook:result_.work_status.right_hook;
      const auto & h=side==0?result_.top_down_hook.left_housing:result_.top_down_hook.right_housing;
      hook.display_state=hook.work_status?"PREPARING":"WAITING";
      hook.start_evidence="NO_NEW_SENSOR_STAMP";
      const bool direct=h.sheave.status==CraneSheaveDetectionStatus::DIRECT_OBSERVED;
      hook.hook_detected=(direct || (h.sheave.status==CraneSheaveDetectionStatus::HELD && h.is_held)) &&
        std::isfinite(h.x) && std::isfinite(h.y) && std::isfinite(h.z);
      hook.hook_observation=hook.hook_detected?(direct?"VIRTUAL_DIRECT":"VIRTUAL_HELD"):"INVALID";
      auto & roi=result_.work_status.cargo_detection_rois[side];roi=CraneFixedRootRoi{};
      if(hook.hook_detected){
        hook.hook_x=h.x;hook.hook_y=h.y;hook.hook_z=h.z-cfg.virtual_hook_offset_z_m;
        roi.x_min=h.x-cfg.cargo_roi_width_x_m*.5;roi.x_max=h.x+cfg.cargo_roi_width_x_m*.5;
        roi.y_min=h.y-cfg.cargo_roi_width_y_m*.5;roi.y_max=h.y+cfg.cargo_roi_width_y_m*.5;
        roi.z_max=hook.hook_z-cfg.cargo_start_offset_m;roi.z_min=hook.hook_z-cfg.connect_max_gap_z;
      }
    }
    left_work_state_.start_observations.clear();right_work_state_.start_observations.clear();

    for(auto & patch:result_.work_status.boundary_points){if(patch.observation!="NONE")patch.observation="HELD";patch.point_count=0;patch.dx=patch.dy=0;}
    work_cycle_.cargo_observation=result_.work_status.cargo_observation;
    work_cycle_.left_hook.object_point_indices.clear();work_cycle_.right_hook.object_point_indices.clear();
    result_.working_status=result_.work_status.working_status;return;
  }
  work_last_stamp_=stamp;work_elapsed_current_=elapsed;cargo_stationary_current_=false;
  if(ordered_state_!=CraneBeamState::LOWERING_CONFIRMED && work_cycle_.fsm_state!=CraneBeamState::COMPLETED && cargo_hits_>0 && stamp-cargo_last_seen_+1e-9>=1.3){
    reset_cargo();resetOrderedWork("CARGO_LOST_RESET_NOT_COMPLETION");
  }
  boundary_move_count_=0;boundary_moving_points_.clear();
  const double dt=elapsed>0?std::min(elapsed,1.0):.1;
  auto & out=work_cycle_;
  out.fsm_enabled=cfg.fsm_enabled;out.boundary_motion_enabled=cfg.boundary_motion_enabled;
  const auto & roi=config_.roi_filter;
  const auto & td=config_.top_down_hook;
  const double noise=config_.dual_roi_anchor.noise_deadband_xy;
  if(cfg.boundary_motion_enabled){
    // Eight fixed search slots on the existing world-frame Crane ROI perimeter.
    // Only measured surface patches become observations; empty slots stay NONE.
    const double xmin=roi.beam_roi_x_min,xmax=roi.beam_roi_x_max;
    const double ymin=roi.beam_roi_y_min,ymax=roi.beam_roi_y_max;
    const std::array<std::array<double,2>,8> slots{{
      {{xmin,ymin}},{{(xmin+xmax)*.5,ymin}},{{xmax,ymin}},{{xmax,(ymin+ymax)*.5}},
      {{xmax,ymax}},{{(xmin+xmax)*.5,ymax}},{{xmin,ymax}},{{xmin,(ymin+ymax)*.5}}}};
    // Retain raw order and seed tie-breaking while testing shared eligibility once.
    std::vector<size_t> eligible_indices;
    eligible_indices.reserve(raw.size());
    for(size_t j=0;j<raw.size();++j){
      const auto & p=raw[j];
      if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z) ||
          p.z<=config_.ground_z+config_.z_gaussian.min_height_above_ground ||
          p.z>roi.beam_roi_z_max)continue;
      eligible_indices.push_back(j);
    }
    std::vector<std::array<double,2>> displacements;
    std::vector<std::vector<size_t>> counted_patch_points;
    for(size_t i=0;i<8;++i){
      auto & obs=boundary_observations_[i];auto & filter=boundary_filters_[i];
      obs.search_x=slots[i][0];obs.search_y=slots[i][1];obs.dx=obs.dy=0;obs.point_count=0;
      const bool tracking=filter.initialized() && stamp-boundary_last_seen_[i]<=cfg.cargo_loss_sec;
      if(!tracking){filter=LinearKalmanFilter{};boundary_hits_[i]=0;}
      const double sx=tracking?filter.x():obs.search_x,sy=tracking?filter.y():obs.search_y;
      const double radius=tracking?config_.dual_roi_anchor.fixed_roi_size_xy*.5:cfg.boundary_search_radius_m;
      const PointXYZIRT * seed=nullptr;double best=radius;
      for(const size_t j:eligible_indices){
        const auto & p=raw[j];
        if(tracking && std::abs(p.z-obs.z)>config_.dual_roi_anchor.fixed_roi_size_z*.5)continue;
        // A component already outside the current radius cannot improve hypot.
        // Keep the original distance, traversal order and strict tie-breaking.
        const double dx=p.x-sx,dy=p.y-sy;
        if(std::abs(dx)>=best || std::abs(dy)>=best)continue;
        const double d=std::hypot(dx,dy);if(d<best){best=d;seed=&p;}
      }
      double x=0,y=0,z=0;size_t n=0;std::vector<size_t> patch_points;
      if(seed)for(const auto & p:raw)if(std::isfinite(p.z) &&
          !(std::abs(p.x-seed->x)>cfg.grid_resolution_m ||
            std::abs(p.y-seed->y)>cfg.grid_resolution_m) &&
          std::hypot(p.x-seed->x,p.y-seed->y)<=cfg.grid_resolution_m &&
          std::abs(p.z-seed->z)<=cfg.grid_resolution_m){x+=p.x;y+=p.y;z+=p.z;++n;patch_points.push_back(size_t(&p-raw.data()));}
      if(n<size_t(std::max(3,config_.height.min_patch_points))){
        obs.observation=tracking?"HELD":"NONE";continue;
      }
      x/=n;y/=n;z/=n;obs.point_count=n;
      if(!tracking){filter.initialize(x,y,td.root_measurement_std,.05);boundary_hits_[i]=1;obs.observation="REACQUIRED";}
      else{
        const double dx=x-obs.x,dy=y-obs.y;
        filter.predict(dt,.2);filter.update(x,y,td.root_measurement_std);
        obs.observation="DIRECT";obs.dx=dx;obs.dy=dy;
        if(++boundary_hits_[i]>=config_.dual_roi_anchor.confirm_frames && elapsed>0 && elapsed<=cfg.cargo_loss_sec){
          displacements.push_back({dx,dy});
          // Count current, established measured patches, not the old aggregate bool.
          // Reuse its existing XY speed threshold. Shared Raw support is one target.
          const double observation_dt=stamp-boundary_last_seen_[i];
          if(observation_dt>0 && observation_dt<=std::min(cfg.cargo_loss_sec,.25) &&
              std::hypot(dx,dy)/observation_dt>=td.root_move_velocity_thresh)
            counted_patch_points.push_back(std::move(patch_points));
        }
      }
      // Keep measured centroids for coherent displacement; KF is the tracked location.
      obs.x=x;obs.y=y;obs.z=z;boundary_last_seen_[i]=stamp;
    }
    boundary_moving_points_=std::move(counted_patch_points);
    std::vector<bool> grouped(boundary_moving_points_.size(),false);
    for(size_t i=0;i<grouped.size();++i)if(!grouped[i]){
      ++boundary_move_count_;grouped[i]=true;std::vector<size_t> queue{i};
      for(size_t q=0;q<queue.size();++q)for(size_t j=0;j<grouped.size();++j)if(!grouped[j]){
        const auto & a=boundary_moving_points_[queue[q]];const auto & b=boundary_moving_points_[j];
        size_t x=0,y=0;bool shared=false;
        while(x<a.size() && y<b.size()){
          if(a[x]==b[y]){shared=true;break;}
          if(a[x]<b[y])++x;else ++y;
        }
        if(shared){grouped[j]=true;queue.push_back(j);}
      }
    }
    bool coherent=false;
    const bool supported=displacements.size()>=size_t(cfg.boundary_min_coherent_points);
    if(displacements.size()>=size_t(cfg.boundary_min_coherent_points)){
      std::vector<double> xs,ys;for(const auto & d:displacements){xs.push_back(d[0]);ys.push_back(d[1]);}
      std::sort(xs.begin(),xs.end());std::sort(ys.begin(),ys.end());
      const double dx=xs[xs.size()/2],dy=ys[ys.size()/2];int agreeing=0;
      for(const auto & d:displacements)if(std::hypot(d[0]-dx,d[1]-dy)<=noise)++agreeing;
      coherent=agreeing>=cfg.boundary_min_coherent_points && std::hypot(dx,dy)/dt>=td.root_move_velocity_thresh;
      if(coherent){boundary_moving_=true;boundary_stable_frames_=0;}
      else if(++boundary_stable_frames_>=td.root_move_stop_frames)boundary_moving_=false;
      out.boundary_reason=coherent?"COHERENT_RAW_XY (STRUCTURE_ID_UNVERIFIED)":"STABLE_RAW_XY";
    }else{boundary_moving_=false;boundary_stable_frames_=0;out.boundary_reason="INSUFFICIENT_RAW_SUPPORT";}
    out.boundary_raw_evidence=coherent;out.boundary_raw_supported=supported;
    if(cfg.boundary_temporal_enabled){
      const auto temporal_begin=std::chrono::steady_clock::now();
      boundary_moving_=boundary_confirmation_.update(stamp,supported,coherent,
          cfg.boundary_start_confirm_sec,cfg.boundary_stop_confirm_sec,cfg.boundary_support_timeout_sec);
      out.boundary_start_elapsed=boundary_confirmation_.start_elapsed;
      out.boundary_stop_elapsed=boundary_confirmation_.stop_elapsed;
      out.boundary_transition_stamp=boundary_confirmation_.transition_stamp;
      out.boundary_temporal_us=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-temporal_begin).count();
    }
    out.boundary_points=boundary_observations_;out.boundary_moving=confirmedCraneMoving();
  }
  // COMPLETED is the existing Drop state; its three-second sensor-time hold is handled below.
  out.fsm_enabled=cfg.fsm_enabled;out.left_hook=CraneHookWorkStatus{};out.right_hook=CraneHookWorkStatus{};
  out.cargo_detection_rois={};out.sheave_visibility={"UNKNOWN","UNKNOWN"};
  const auto & result=result_.top_down_hook;
  std::array<const CraneSheaveGeometry *,2> housings{{&result.left_housing.sheave,&result.right_housing.sheave}};
  bool any_wire=!result.chains.empty(),any_housing=false,any_hook=false;
  // Conservatively prefilter once, only if a legacy bounding-box Raw query is
  // needed. Every query remains exact and retains original Raw index order.
  // Current-owned candidates already carry explicit indices and bypass this.
  std::vector<int> query_raw;bool query_raw_ready=false;
  auto prepare_query_raw=[&](){
    if(query_raw_ready)return;
    query_raw_ready=true;
    double xmin=1e9,ymin=1e9,zmin=1e9,xmax=-1e9,ymax=-1e9,zmax=-1e9;
    bool bounded=true,any=false;
    for(const auto&box:subclusters)if(!box.current_raw_indices){
      if(!std::isfinite(box.min_x)||!std::isfinite(box.max_x)||!std::isfinite(box.min_y)||
         !std::isfinite(box.max_y)||!std::isfinite(box.min_z)||!std::isfinite(box.max_z)){bounded=false;break;}
      any=true;xmin=std::min(xmin,box.min_x-.005);xmax=std::max(xmax,box.max_x+.005);
      ymin=std::min(ymin,box.min_y-.005);ymax=std::max(ymax,box.max_y+.005);
      zmin=std::min(zmin,box.min_z-.005);zmax=std::max(zmax,box.max_z+.005);
    }
    query_raw.reserve(raw.size());
    for(size_t i=0;i<raw.size();++i){const auto&p=raw[i];
      if(!bounded || (any && std::isfinite(p.z)&&p.x>=xmin&&p.x<=xmax&&
         p.y>=ymin&&p.y<=ymax&&p.z>=zmin&&p.z<=zmax))query_raw.push_back(int(i));
    }
  };
  auto indices_in=[&](const CraneSubcluster & sc){std::vector<int> ids;
    if(sc.current_raw_indices){for(int id:sc.point_indices)if(id>=0 && size_t(id)<raw.size() && std::isfinite(raw[id].x) && std::isfinite(raw[id].y) && std::isfinite(raw[id].z))ids.push_back(id);return ids;}

    prepare_query_raw();
    for(int i:query_raw){const auto & p=raw[i];
      if(std::isfinite(p.z)&&p.x>=sc.min_x-.005 && p.x<=sc.max_x+.005 &&
          p.y>=sc.min_y-.005 && p.y<=sc.max_y+.005 && p.z>=sc.min_z-.005 && p.z<=sc.max_z+.005)ids.push_back(int(i));}
    return ids;
  };
  for(int side=0;side<2;++side){
    const auto & h=*housings[side];auto & hook=side==0?out.left_hook:out.right_hook;
    hook.side=side==0?"LEFT":"RIGHT";
    const auto & memory=side==0?result.left_housing_memory:result.right_housing_memory;
    if(h.status==CraneSheaveDetectionStatus::DIRECT_OBSERVED){out.sheave_visibility[side]="VISIBLE";any_housing=true;}
    else if(memory.initialized && stamp-memory.last_direct_stamp>=0 && stamp-memory.last_direct_stamp<=cfg.cargo_loss_sec && visibility_model_ptr_){
      const auto v=visibility_model_ptr_->evaluatePoint(memory.housing_center_x,memory.housing_center_y,memory.housing_center_z);
      if(v.reason==VisibilityReason::OUT_OF_VERTICAL_FOV && v.elevation_deg>visibility_model_ptr_->config().elevation_max_deg)
        out.sheave_visibility[side]="ABOVE_FOV";
      else if(v.has_occluder || v.reason==VisibilityReason::OCCLUDED_BY_OTHER || v.reason==VisibilityReason::FULLY_OCCLUDED_LIKELY)
        out.sheave_visibility[side]="OCCLUDED";
    }
    const auto & tracked=side==0?result.left_housing:result.right_housing;
    const bool direct=h.status==CraneSheaveDetectionStatus::DIRECT_OBSERVED;
    const bool held=h.status==CraneSheaveDetectionStatus::HELD && tracked.is_held && memory.confirmed;
    if((!direct && !held) || !std::isfinite(tracked.x) || !std::isfinite(tracked.y) || !std::isfinite(tracked.z))continue;
    hook.hook_detected=true;hook.hook_x=tracked.x;hook.hook_y=tracked.y;
    hook.hook_z=tracked.z-cfg.virtual_hook_offset_z_m;
    hook.hook_observation=held?"VIRTUAL_HELD":"VIRTUAL_DIRECT";
    if(hook.hook_detected){
      any_hook=true;auto & r=out.cargo_detection_rois[side];
      r.x_min=hook.hook_x-cfg.cargo_roi_width_x_m*.5;r.x_max=hook.hook_x+cfg.cargo_roi_width_x_m*.5;
      r.y_min=hook.hook_y-cfg.cargo_roi_width_y_m*.5;r.y_max=hook.hook_y+cfg.cargo_roi_width_y_m*.5;
      r.z_max=hook.hook_z-cfg.cargo_start_offset_m;r.z_min=hook.hook_z-cfg.connect_max_gap_z;
    }
  }
  if(!cfg.fsm_enabled){result_.work_status=out;return;}
  // Keep direct Cargo measurements independent of the downstream public FSM.
  // SUSPENDED admission may consume confirmed Cargo without a prior LIFTING.
  // Candidate READY/CONFIRM are local measurement-confirmation phases, not public work states.
  if(out.geometry_locked)out.fsm_state=CraneBeamState::LOCKED_TRACKING;
  out.work_status=out.geometry_locked || cargo_hits_>=std::max(1,cfg.confirm_m_on);

  if(out.fsm_state==CraneBeamState::COMPLETED){
    if(stamp-drop_started_stamp_<3.0){
      out.work_status=false;out.working_status=2;out.drop_started_stamp=drop_started_stamp_;
      out.cargo_observation="HELD";out.extension_observation="HELD";
      out.extended_raw_indices.clear();out.extension_cluster_ids.clear();
      out.gap_diagnostic="UNKNOWN";out.gap_reason="DROP_HOLD_NO_NEW_SUPPORT_EVIDENCE";
      result_.work_status=out;result_.working_status=2;return;
    }
    reset_cargo();out.fsm_enabled=true;out.status_reason="DROP_THREE_SECONDS_ELAPSED";
    result_.work_status=out;result_.working_status=0;return; // no same-frame restart
  }

  struct Candidate {int id=-1;unsigned sides=0;double x=0,y=0,z=0,l=0,w=0,h=0,yaw=0,area=0;};
  bool current_connected_partial=false;int connection_misses=0;
  out.status_reason="NONE";
  std::vector<int> best_raw_ids;
  Candidate best;double best_score=std::numeric_limits<double>::infinity();
  std::array<bool,2> cargo_ground_valid{};std::array<double,2> cargo_ground_z{};
  std::array<nlohmann::json,2> cargo_density;
  static thread_local HookZDensity density;
  for(int side=0;side<2;++side){
    const auto & hook=side==0?out.left_hook:out.right_hook;
    if(!hook.hook_detected || hook.hook_observation!="VIRTUAL_DIRECT")continue;
    cargo_density[side]=density.detect(raw,hook.hook_x,hook.hook_y,hook.hook_z,cfg.cargo_roi_width_x_m,cfg.cargo_roi_width_y_m,result_.ground,roi.beam_roi_x_max-roi.beam_roi_x_min,roi.beam_roi_y_max-roi.beam_roi_y_min);
    cargo_ground_valid[side]=cargo_density[side].value("ground_valid",false);cargo_ground_z[side]=cargo_density[side].value("ground_z",0.0);
  }
  const auto current_owned=out.geometry_locked?currentOwnedCargoRaw(raw,stamp):std::vector<int>{};
  auto current_candidates=subclusters;
  if(out.geometry_locked && current_owned.size()>=cfg.min_candidate_points){
    // Z occupancy comes from the established Cargo's current returns. A narrow upward tail
    // cannot replace its broad body; retain all lower measured surfaces for clearance.
    std::map<int,std::set<std::pair<int,int>>> bands;
    for(int id:current_owned){const auto&p=raw[id];bands[int(std::floor(p.z/cfg.grid_resolution_m))].emplace(int(std::floor(p.x/cfg.grid_resolution_m)),int(std::floor(p.y/cfg.grid_resolution_m)));}
    int peak=0;size_t support=0;for(const auto&band:bands)if(band.second.size()>support){peak=band.first;support=band.second.size();}
    const double upper=(peak+1)*cfg.grid_resolution_m+config_.dual_roi_anchor.noise_deadband_z;
    CraneSubcluster sc;sc.current_raw_indices=true;sc.subcluster_id=out.cargo_cluster_id;
    sc.min_x=sc.min_y=sc.min_z=1e9;sc.max_x=sc.max_y=sc.max_z=-1e9;
    for(int id:current_owned){const auto&p=raw[id];if(p.z>upper)continue;sc.point_indices.push_back(id);
      sc.min_x=std::min(sc.min_x,double(p.x));sc.max_x=std::max(sc.max_x,double(p.x));sc.min_y=std::min(sc.min_y,double(p.y));sc.max_y=std::max(sc.max_y,double(p.y));sc.min_z=std::min(sc.min_z,double(p.z));sc.max_z=std::max(sc.max_z,double(p.z));sc.centroid_x+=p.x;sc.centroid_y+=p.y;sc.centroid_z+=p.z;}
    if(sc.point_indices.size()>=cfg.min_candidate_points){sc.centroid_x/=sc.point_indices.size();sc.centroid_y/=sc.point_indices.size();sc.centroid_z/=sc.point_indices.size();current_candidates.insert(current_candidates.begin(),std::move(sc));}
  }
  for(const auto & source_sc:current_candidates){
    auto sc=source_sc;std::vector<int> refined_ids;bool ground_refined=false,ground_attempted=false;
    unsigned sides=0;
    for(int side=0;side<2;++side){const auto & r=out.cargo_detection_rois[side];
      const auto & hook=side==0?out.left_hook:out.right_hook;
      const bool valid=hook.hook_detected && hook.hook_observation=="VIRTUAL_DIRECT";
      if(valid && sc.max_x>=r.x_min && sc.min_x<=r.x_max && sc.max_y>=r.y_min && sc.min_y<=r.y_max &&
          sc.max_z>=r.z_min && sc.min_z<=r.z_max)sides|=1u<<side;
    }
    // Refine only a mixed near-ground acquisition box. Existing upstream
    // clusters and tracking stay intact; current Raw owns all retained points.
    if(sides){for(int side=0;side<2;++side)if((sides&(1u<<side)) && cargo_ground_valid[side] &&
        sc.min_z<=cargo_ground_z[side]+cfg.cargo_start_offset_m &&
        (!out.geometry_locked || out.cargo_z-out.height*.5<=cargo_ground_z[side]+cfg.cargo_start_offset_m)){
      ground_attempted=true;
      const auto & d=cargo_density[side];
      if(!d.value("valid",false)){sides&=~(1u<<side);continue;}
      const auto & hook=side==0?out.left_hook:out.right_hook;
      const double step=cfg.grid_resolution_m;
      const auto all=indices_in(sc);
      double peaklo=0,peakhi=0;size_t peakcount=0;
      for(const auto & b:d["histogram"])if(b[1].get<double>()>=sc.min_z-.005 &&
          b[0].get<double>()<=sc.max_z+.005 && b[2].get<size_t>()>peakcount){peakcount=b[2].get<size_t>();peaklo=b[0];peakhi=b[1];}
      if(peakcount<size_t(cfg.min_candidate_points)){sides&=~(1u<<side);continue;}
      using Cell=std::pair<int,int>;std::map<Cell,std::vector<int>> surface;
      Cell seed{};double nearest=1e9;
      for(int id:all){const auto & p=raw[id];if(p.z<peaklo || p.z>peakhi ||
          p.z<=cargo_ground_z[side]+density.cfg.value("ground_clearance_m",.15))continue;
        Cell cell{int(std::floor(p.x/step)),int(std::floor(p.y/step))};surface[cell].push_back(id);
        const double dist=std::hypot(p.x-hook.hook_x,p.y-hook.hook_y);
        if(dist<nearest){nearest=dist;seed=cell;}}
      if(surface.empty() || nearest>cfg.connect_xy_margin){sides&=~(1u<<side);continue;}
      std::set<Cell> component{seed};std::vector<Cell> queue{seed};
      for(size_t qi=0;qi<queue.size();++qi)for(const Cell delta:{Cell{-1,0},Cell{1,0},Cell{0,-1},Cell{0,1}}){
        Cell next{queue[qi].first+delta.first,queue[qi].second+delta.second};
        if(surface.count(next) && component.insert(next).second)queue.push_back(next);}
      for(int id:all){const auto & p=raw[id];Cell cell{int(std::floor(p.x/step)),int(std::floor(p.y/step))};
        if(component.count(cell) && p.z>cargo_ground_z[side]+step && p.z<=peakhi+noise)refined_ids.push_back(id);}
      if(refined_ids.size()<size_t(cfg.min_candidate_points)){refined_ids.clear();sides&=~(1u<<side);continue;}
      sc.min_x=sc.min_y=sc.min_z=1e9;sc.max_x=sc.max_y=sc.max_z=-1e9;sc.centroid_x=sc.centroid_y=sc.centroid_z=0;
      for(int id:refined_ids){const auto & p=raw[id];sc.min_x=std::min(sc.min_x,double(p.x));sc.max_x=std::max(sc.max_x,double(p.x));
        sc.min_y=std::min(sc.min_y,double(p.y));sc.max_y=std::max(sc.max_y,double(p.y));sc.min_z=std::min(sc.min_z,double(p.z));sc.max_z=std::max(sc.max_z,double(p.z));
        sc.centroid_x+=p.x;sc.centroid_y+=p.y;sc.centroid_z+=p.z;}
      sc.centroid_x/=refined_ids.size();sc.centroid_y/=refined_ids.size();sc.centroid_z/=refined_ids.size();ground_refined=true;break;
    }}
    if(ground_attempted && !ground_refined && !sides){continue;}
    const bool locked=out.geometry_locked;
    const bool supporting_housing_held=cargo_hits_>0 &&
      ((out.cargo_side!="RIGHT" && out.left_hook.hook_observation=="VIRTUAL_HELD") ||
       (out.cargo_side!="LEFT" && out.right_hook.hook_observation=="VIRTUAL_HELD"));
    // Pause pre-Lock confirmation without changing BOTH to a different side
    // when one of the same Cargo's supporting housings is briefly held.
    if(!locked && (supporting_housing_held || !sides))continue;
    // Consume the existing BEV temporal ID as well as position and local shape.
    // A nearby independent cluster cannot silently inherit a locked Cargo ID.
    // Cluster IDs may split/relabel; the existing Cargo KF + local geometry gates retain Cargo identity.
    if(cargo_hits_>0 && std::hypot(sc.centroid_x-cargo_xy_filter_.x(),sc.centroid_y-cargo_xy_filter_.y())>
      config_.temporal_stabilization.largest_match_dist)continue;
    if(cargo_hits_>0 && std::abs(sc.centroid_z-cargo_z_filter_.x())>cfg.connect_xy_margin)continue;
    if(sc.point_indices.size()<cfg.min_candidate_points)continue;
    auto ids=ground_refined?std::move(refined_ids):indices_in(sc);if(ids.size()<cfg.min_candidate_points)continue;
    size_t current_owned_count=0;for(int id:ids)if(std::binary_search(current_owned.begin(),current_owned.end(),id))++current_owned_count;
    const bool verified_owned=out.geometry_locked && sc.current_raw_indices && current_owned_count>=cfg.min_candidate_points && current_owned_count>=.75*ids.size();
    if(out.geometry_locked && sc.current_raw_indices && !verified_owned){continue;}
    Candidate c;c.id=sc.subcluster_id;c.sides=sides;c.h=sc.max_z-sc.min_z;
    CraneBeamDetector geometry;geometry.computeClusterPCA(raw,ids,c.l,c.yaw,c.x,c.y);
    double lo=1e9,hi=-1e9;for(int id:ids){const auto & p=raw[id];c.z+=p.z;
      const double v=-(p.x-c.x)*std::sin(c.yaw)+(p.y-c.y)*std::cos(c.yaw);lo=std::min(lo,v);hi=std::max(hi,v);}
    c.w=hi-lo;c.z/=ids.size();
    if(verified_owned){c.yaw=out.yaw;double u0=1e9,u1=-1e9,v0=1e9,v1=-1e9;
      for(int id:ids){const auto&p=raw[id];const double u=(p.x-c.x)*std::cos(c.yaw)+(p.y-c.y)*std::sin(c.yaw),v=-(p.x-c.x)*std::sin(c.yaw)+(p.y-c.y)*std::cos(c.yaw);u0=std::min(u0,u);u1=std::max(u1,u);v0=std::min(v0,v);v1=std::max(v1,v);}
      c.l=u1-u0;c.w=v1-v0;}
    if(out.geometry_locked && ((out.length-c.l>config_.temporal_stabilization.jump_tolerance_length &&
        out.width-c.w>config_.temporal_stabilization.jump_tolerance_width) ||
        (out.length>=out.width*cfg.size_ratio_k && c.l<c.w*cfg.size_ratio_k))){continue;}


    unsigned measured_overlap=0;
    for(int side=0;side<2;++side)if(c.sides&(1u<<side)){
      const auto & r=out.cargo_detection_rois[side];size_t n=0;
      for(int id:ids){const auto & p=raw[id];if(p.x>=r.x_min && p.x<=r.x_max && p.y>=r.y_min && p.y<=r.y_max && p.z>=r.z_min && p.z<=r.z_max)++n;}
      if(n>=cfg.min_candidate_points)measured_overlap|=1u<<side;
    }
    c.sides=measured_overlap;
    if(!locked && !c.sides)continue;
    if(cargo_hits_>0 && !verified_owned && sc.subcluster_id!=out.cargo_cluster_id &&
        (std::abs(c.l-out.length)>config_.temporal_stabilization.jump_tolerance_length ||
         std::abs(c.w-out.width)>config_.temporal_stabilization.jump_tolerance_width ||
         std::abs(std::remainder(c.yaw-out.yaw,pi))>config_.temporal_stabilization.yaw_jump_tolerance_deg*pi/180))continue;

    // Acquisition still needs current connecting returns. Once minimal work
    // confirmation is achieved, reuse the existing locked-track geometry/KF
    // association policy without making Size Lock a tracking prerequisite.
    // A briefly hidden Sling must not turn the same observed Cargo into a new Seed.
    const unsigned known_support=out.cargo_side=="BOTH"?3u:(out.cargo_side=="LEFT"?1u:2u);
    const bool continuing_support=out.work_status && (c.sides&known_support);
    if(!locked && !continuing_support){
      unsigned connected_sides=0;
      for(int side=0;side<2;++side)if(c.sides&(1u<<side)){
        const auto & hook=side==0?out.left_hook:out.right_hook;
        const double low=sc.max_z+cfg.grid_resolution_m,high=hook.hook_z-cfg.grid_resolution_m;
        std::array<int,3> support{};

        if(high<=low){
          // The three wire bins do not exist when Hook and measured body touch.
          // Require current broad top-surface returns at the virtual Hook and valid separate Ground.
          const auto & d=cargo_density[side];size_t contact_points=0;
          std::set<std::pair<int,int>> contact_cells;
          if(d.value("ground_valid",false) && d.value("valid",false) &&
             hook.hook_z>d.value("ground_z",0.)+cfg.cargo_start_offset_m){
            for(int id:ids){const auto & p=raw[id];
              if(p.z>=hook.hook_z-2*cfg.grid_resolution_m && p.z<=hook.hook_z+cfg.grid_resolution_m &&
                 std::hypot(p.x-hook.hook_x,p.y-hook.hook_y)<=2*cfg.grid_resolution_m){
                ++contact_points;contact_cells.emplace(int(std::floor(p.x/cfg.grid_resolution_m)),int(std::floor(p.y/cfg.grid_resolution_m)));}
            }
          }

          if(contact_points>=size_t(cfg.min_candidate_points) && contact_cells.size()>=3)
            connected_sides|=1u<<side;
          else ++connection_misses;
          continue;
        }

        // Use an actual nearest measured surface return, not an empty AABB corner.
        // The existing 3-bin / 3-point / 2-cell radius criteria now follow the
        // Hook-to-surface segment; an unrelated vertical column beside Hook is not attachment evidence.
        double x=0,y=0,nearest=std::numeric_limits<double>::infinity();
        for(int id:ids){const auto & point=raw[id];const double d=std::hypot(point.x-hook.hook_x,point.y-hook.hook_y);
          if(d<nearest){nearest=d;x=point.x;y=point.y;}}
        bool surface_anchor=false;
        for(const auto & point:raw)if(point.z>low && point.z<high){
          const double t=(point.z-low)/(high-low);
          const double sx=x+t*(hook.hook_x-x),sy=y+t*(hook.hook_y-y);
          if(std::hypot(point.x-sx,point.y-sy)<=2*cfg.grid_resolution_m){
            ++support[std::clamp(int(3*t),0,2)];
            if(point.z-low<=cfg.grid_resolution_m)surface_anchor=true;
          }
        }
        // At least one current connector return must reach the existing first
        // occupancy cell above the candidate. A higher unrelated surface alone is insufficient.
        bool supported=surface_anchor;for(int n:support)if(n<3)supported=false;
        if(!supported)++connection_misses;
        if(supported)connected_sides|=1u<<side;
      }
      c.sides=connected_sides;if(!c.sides)continue;
    }else if(!locked){
      // An overlapping second Hook ROI does not establish a new supporting side.
      c.sides&=known_support;
    }
    c.area=computeOccupiedArea(ids,raw,cfg.grid_resolution_m);
    // Bounded pre-Lock pause only for the same CONNECTED surface becoming
    // marginally narrow. Do not extend an unrelated/absent candidate's work latch.
    if(cargo_hits_>0 && sc.subcluster_id==out.cargo_cluster_id && c.sides &&
        c.l>=cfg.cargo_min_surface_width_m && c.w<cfg.cargo_min_surface_width_m &&
        c.w>=cfg.cargo_min_surface_width_m-noise &&
        c.area>=std::max(cfg.grid_resolution_m*cfg.grid_resolution_m*6,cfg.cargo_min_fill_ratio*c.l*c.w) &&
        hasCargoSurfacePatch(raw,ids,c.x,c.y,c.yaw,cfg.grid_resolution_m,cfg.cargo_min_surface_width_m))current_connected_partial=true;

    // A locked Cargo can expose a narrower measured subset of the SAME owned
    // cluster. Acquisition, occupied area, KF association and lock stay intact.
    bool tracked_raw_partial=out.geometry_locked && sc.subcluster_id==out.cargo_cluster_id &&
      c.w<cfg.cargo_min_surface_width_m &&
      c.h>=config_.dual_roi_anchor.noise_deadband_z &&
      c.w>=cfg.cargo_min_surface_width_m-noise &&
      std::all_of(ids.begin(),ids.end(),[&](int id){const auto & p=raw[id];
        const double dx=p.x-cargo_xy_filter_.x(),dy=p.y-cargo_xy_filter_.y();
        const double u=dx*std::cos(out.yaw)+dy*std::sin(out.yaw);
        const double v=-dx*std::sin(out.yaw)+dy*std::cos(out.yaw);
        return std::abs(u)<=out.length*.5+noise && std::abs(v)<=out.width*.5+noise &&
          std::abs(p.z-cargo_z_filter_.x())<=out.height+config_.dual_roi_anchor.noise_deadband_z;
      }) && hasCargoSurfacePatch(raw,ids,c.x,c.y,c.yaw,cfg.grid_resolution_m,
        cfg.cargo_min_surface_width_m,config_.dual_roi_anchor.noise_deadband_z,
        size_t(std::max(1,config_.z_gaussian.min_layer_points)));

    // Same owned, already locked Cargo only: measure the currently supported
    // subset inside its existing envelope instead of enlarging that envelope.
    // Revalidate geometry, occupied area and independent vertical surface columns.
    if(!tracked_raw_partial && out.geometry_locked && sc.subcluster_id==out.cargo_cluster_id &&
        c.w<cfg.cargo_min_surface_width_m && c.w>=cfg.cargo_min_surface_width_m-noise &&
        c.h>=config_.dual_roi_anchor.noise_deadband_z &&
        c.l<=out.length+config_.temporal_stabilization.jump_tolerance_length){
      std::vector<int> bounded;bounded.reserve(ids.size());
      double original_bottom=1e9,original_top=-1e9;
      for(int id:ids){const auto & p=raw[id];
        original_bottom=std::min(original_bottom,double(p.z));original_top=std::max(original_top,double(p.z));
        const double dx=p.x-cargo_xy_filter_.x(),dy=p.y-cargo_xy_filter_.y();
        const double u=dx*std::cos(out.yaw)+dy*std::sin(out.yaw);
        const double v=-dx*std::sin(out.yaw)+dy*std::cos(out.yaw);
        if(std::abs(u)<=out.length*.5+noise && std::abs(v)<=out.width*.5+noise &&
           std::abs(p.z-cargo_z_filter_.x())<=out.height+config_.dual_roi_anchor.noise_deadband_z)
          bounded.push_back(id);
      }
      if(bounded.size()>=cfg.min_candidate_points && bounded.size()<ids.size()){
        Candidate observed=c;geometry.computeClusterPCA(raw,bounded,observed.l,observed.yaw,observed.x,observed.y);
        double lo=1e9,hi=-1e9,zlo=1e9,zhi=-1e9;observed.z=0;
        for(int id:bounded){const auto & p=raw[id];
          const double v=-(p.x-observed.x)*std::sin(observed.yaw)+(p.y-observed.y)*std::cos(observed.yaw);
          lo=std::min(lo,v);hi=std::max(hi,v);zlo=std::min(zlo,double(p.z));zhi=std::max(zhi,double(p.z));observed.z+=p.z;
        }
        observed.w=hi-lo;observed.h=zhi-zlo;observed.z/=bounded.size();
        observed.area=computeOccupiedArea(bounded,raw,cfg.grid_resolution_m);
        if(zlo==original_bottom && zhi==original_top &&
           observed.l>=cfg.cargo_min_surface_width_m && observed.w>=cfg.cargo_min_surface_width_m-noise &&
           observed.h>=config_.dual_roi_anchor.noise_deadband_z &&
           observed.area>=std::max(cfg.grid_resolution_m*cfg.grid_resolution_m*6,cfg.cargo_min_fill_ratio*observed.l*observed.w) &&
           hasCargoSurfacePatch(raw,bounded,observed.x,observed.y,observed.yaw,cfg.grid_resolution_m,
             cfg.cargo_min_surface_width_m,config_.dual_roi_anchor.noise_deadband_z,
             size_t(std::max(1,config_.z_gaussian.min_layer_points)))){
          ids=std::move(bounded);c=observed;tracked_raw_partial=true;
        }
      }
    }

    if(c.l<cfg.cargo_min_surface_width_m || (c.w<cfg.cargo_min_surface_width_m && !tracked_raw_partial) ||
        c.area<std::max(cfg.grid_resolution_m*cfg.grid_resolution_m*6,cfg.cargo_min_fill_ratio*c.l*c.w))continue;
    if(!tracked_raw_partial && !hasCargoSurfacePatch(raw,ids,c.x,c.y,c.yaw,cfg.grid_resolution_m,cfg.cargo_min_surface_width_m))continue;

    // Reuse the existing object Exception Gate's LENGTH OR OCCUPIED-AREA
    // sufficiency criterion for new work only. A small broad patch is not by
    // itself a work object; confirmed tracks keep their existing partial-view policy.
    if(!out.work_status && c.l<config_.exception_gate.min_length_m && c.area<config_.exception_gate.min_area_m2)continue;

    // Compare object-local PCA geometry, not world-accumulated extents. Once
    // locked, partial views may update position but never change locked geometry.
    if(cargo_hits_>0 && !locked && (std::abs(c.l-confirmation_length_)>config_.temporal_stabilization.jump_tolerance_length ||
        std::abs(c.w-confirmation_width_)>config_.temporal_stabilization.jump_tolerance_width))continue;
    if(locked && (c.l>out.length+config_.temporal_stabilization.jump_tolerance_length ||
        c.w>out.width+config_.temporal_stabilization.jump_tolerance_width))continue;


    // Prefer the largest currently reobserved owned surface over a nearer small Hook/sling fragment.
    const double score=out.geometry_locked && sc.current_raw_indices?-double(current_owned_count):
      (cargo_hits_>0?std::hypot(c.x-cargo_xy_filter_.x(),c.y-cargo_xy_filter_.y())+std::abs(c.z-cargo_z_filter_.x()):-c.area);
    if(score<best_score){best=c;best_score=score;best_raw_ids=std::move(ids);}
    // This complete owned candidate already outranks every legacy accumulated candidate.
    // Keep the generic Raw-candidate path exhaustive if a caller supplies additional Raw candidates.
    if(verified_owned && sc.subcluster_id==out.cargo_cluster_id &&
       std::none_of(subclusters.begin(),subclusters.end(),[](const auto&other){return other.current_raw_indices;}))break;
  }
  out.cargo_observation="NONE";
  if(best.id>=0){
    const std::string measured_side=best.sides==3?"BOTH":(best.sides==1?"LEFT":"RIGHT");
    if(!out.geometry_locked && cargo_hits_>0 && measured_side!=out.cargo_side)reset_cargo();
    const bool first=cargo_hits_==0;
    if(first){
      out.cargo_track_id=next_cargo_id_++;cargo_xy_filter_.initialize(best.x,best.y,.03,1);
      cargo_z_filter_.initialize(best.z,0,.03,1);confirmation_length_=best.l;confirmation_width_=best.w;
      cargo_observed_sec_=0;out.cargo_side=measured_side;
    }else{
      cargo_xy_filter_.predict(dt,.2);cargo_z_filter_.predict(dt,.2);
      cargo_xy_filter_.update(best.x,best.y,.03);cargo_z_filter_.update(best.z,0,.03);
      // Only consecutive actual observations add elapsed sensor time. Gaps pause
      // and reset confirmation; duplicate stamps were rejected above.
      if(stamp-cargo_last_seen_<=cfg.cargo_loss_sec && elapsed<=std::min(cfg.cargo_loss_sec,.25)) cargo_observed_sec_+=elapsed;
      else if(!out.geometry_locked){cargo_observed_sec_=0;cargo_hits_=0;}
    }
    const double nz=config_.dual_roi_anchor.noise_deadband_z;
    const bool stationary=!first && std::hypot(best.x-cargo_last_x_,best.y-cargo_last_y_)<=noise && std::abs(best.z-cargo_last_z_)<=nz;
    cargo_stationary_current_=stationary;
    if(first)cargo_peak_z_=best.z;
    cargo_peak_z_=std::max(cargo_peak_z_,best.z);
    if(!first && best.z<cargo_last_z_-nz*.1)++cargo_descent_steps_;
    if(cargo_descent_steps_>=std::max(2,cfg.confirm_m_off) && cargo_peak_z_-best.z>=std::max(3*nz,cfg.cargo_start_offset_m))cargo_descent_seen_=true;
    ++cargo_hits_;cargo_last_seen_=stamp;cargo_last_x_=best.x;cargo_last_y_=best.y;cargo_last_z_=best.z;
    out.cargo_x=cargo_xy_filter_.x();out.cargo_y=cargo_xy_filter_.y();out.cargo_z=cargo_z_filter_.x();
    // Retain the confirmed local geometry yaw; the position KF still updates.
    out.cargo_cluster_id=best.id;out.cargo_observation="DIRECT";out.confirmed_observation_sec=cargo_observed_sec_;
    if(!out.geometry_locked){
      out.length=best.l;out.width=best.w;out.height=std::max(cfg.grid_resolution_m,best.h);out.yaw=best.yaw;
      auto & observations=out.cargo_side=="RIGHT"?right_work_state_.object_observations:left_work_state_.object_observations;
      observations.push_back({stamp,best.l,best.w,std::max(cfg.grid_resolution_m,best.h),best.yaw,best.x,best.y,best.z});
      while(observations.size()>256 || (!observations.empty() && stamp-observations.front()[0]>cfg.cargo_confirm_sec+.25))observations.pop_front();
      double lmin=1e9,lmax=0,wmin=1e9,wmax=0;bool stable=true;
      for(const auto & observation:observations){
        lmin=std::min(lmin,observation[1]);lmax=std::max(lmax,observation[1]);
        wmin=std::min(wmin,observation[2]);wmax=std::max(wmax,observation[2]);
        if(std::abs(std::remainder(observation[4]-observations.front()[4],pi))>
            config_.temporal_stabilization.yaw_jump_tolerance_deg*pi/180)stable=false;
      }
      stable=stable && lmax-lmin<=config_.temporal_stabilization.jump_tolerance_length &&
        wmax-wmin<=config_.temporal_stabilization.jump_tolerance_width;
      if(cargo_hits_<std::max(1,cfg.confirm_m_on))out.fsm_state=CraneBeamState::WAIT_CARGO;
      else if(cargo_hits_==std::max(1,cfg.confirm_m_on))out.fsm_state=CraneBeamState::READY;
      else out.fsm_state=CraneBeamState::CONFIRM_CARGO;
      if(stable && cargo_observed_sec_+1e-6>=cfg.cargo_confirm_sec && cargo_hits_>=std::max(2,cfg.confirm_m_on)){
        auto median=[&](int column){std::vector<double> values;for(const auto & observation:observations)values.push_back(observation[column]);
          std::sort(values.begin(),values.end());return values[values.size()/2];};
        out.length=median(1);out.width=median(2);out.height=median(3);
        double sy=0,cy=0;for(const auto & observation:observations){sy+=std::sin(2*observation[4]);cy+=std::cos(2*observation[4]);}
        out.yaw=.5*std::atan2(sy,cy);
        out.geometry_locked=true;out.fsm_state=CraneBeamState::LOCKED_TRACKING;
        // Freeze the initial Hook search ROI union, clipped to the existing
        // world Crane ROI. It does not follow either Hook or Cargo afterwards.
        CraneFixedRootRoi boundary;bool have=false;
        for(int side=0;side<2;++side)if(best.sides&(1u<<side)){
          const auto & r=out.cargo_detection_rois[side];
          if(!have){boundary=r;have=true;}else{boundary.x_min=std::min(boundary.x_min,r.x_min);boundary.x_max=std::max(boundary.x_max,r.x_max);
            boundary.y_min=std::min(boundary.y_min,r.y_min);boundary.y_max=std::max(boundary.y_max,r.y_max);boundary.z_min=std::min(boundary.z_min,r.z_min);boundary.z_max=std::max(boundary.z_max,r.z_max);}
        }
        boundary.x_min=std::max(boundary.x_min,roi.beam_roi_x_min);boundary.x_max=std::min(boundary.x_max,roi.beam_roi_x_max);
        boundary.y_min=std::max(boundary.y_min,roi.beam_roi_y_min);boundary.y_max=std::min(boundary.y_max,roi.beam_roi_y_max);
        out.completion_boundary=boundary;
      }
    }
    // Drop is evaluated after CURRENT Raw extension/support diagnostics, never from a stale prior frame.

  }else if(cargo_hits_>0){
    cargo_outside_hits_=0;
    if(stamp-cargo_last_seen_<=cfg.cargo_loss_sec){
      out.cargo_observation="HELD";
      const bool housing_held=(out.cargo_side!="RIGHT" && out.left_hook.hook_observation=="VIRTUAL_HELD") ||
        (out.cargo_side!="LEFT" && out.right_hook.hook_observation=="VIRTUAL_HELD");
      if(!out.geometry_locked && !out.work_status && !housing_held && !current_connected_partial){
        reset_cargo();out.fsm_enabled=true;out.fsm_state=CraneBeamState::WAIT_CARGO;
      }
    }else{
      reset_cargo();out.fsm_enabled=true;out.status_reason="LOST_WITHOUT_COMPLETION";
    }
  }
  if(cargo_hits_==0 && out.fsm_state!=CraneBeamState::COMPLETED)
    out.fsm_state=!any_wire?CraneBeamState::SEARCH_WIRE:(!any_housing?CraneBeamState::TRACK_SHEAVE:CraneBeamState::WAIT_CARGO);
  out.work_status=out.fsm_state==CraneBeamState::READY || out.fsm_state==CraneBeamState::CONFIRM_CARGO || out.fsm_state==CraneBeamState::LOCKED_TRACKING;
  out.working_status=out.fsm_state==CraneBeamState::COMPLETED?2:(out.work_status?1:0);
  for(int side=0;side<2;++side){
    auto & hook=side==0?out.left_hook:out.right_hook;
    auto & samples=(side==0?left_work_state_:right_work_state_).start_observations;
    const bool own_cargo=out.cargo_side=="BOTH" || out.cargo_side==(side==0?"LEFT":"RIGHT");
    if(own_cargo && out.cargo_observation=="DIRECT")hook.object_point_indices=best_raw_ids;
    hook.work_status=out.work_status && own_cargo;
    hook.display_state=hook.work_status?"PREPARING":"WAITING";
    const auto & housing=*housings[side];
    const bool direct=hook.work_status && hook.hook_detected && out.cargo_observation=="DIRECT" &&
      best.id==out.cargo_cluster_id && (best.sides&(1u<<side)) &&
      housing.status==CraneSheaveDetectionStatus::DIRECT_OBSERVED;
    if(!direct){samples.clear();continue;}
    if(!samples.empty() && (elapsed<=0 || elapsed>.25 ||
        std::hypot(hook.hook_x-samples.back()[4],hook.hook_y-samples.back()[5])>
          config_.temporal_stabilization.largest_match_dist ||
        std::hypot(best.x-samples.back()[6],best.y-samples.back()[7])>
          config_.temporal_stabilization.largest_match_dist))samples.clear();
    samples.push_back({stamp,hook.hook_z,best.z,housing.outer_center_z,hook.hook_x,hook.hook_y,best.x,best.y});
    while(samples.size()>64 || (!samples.empty() && stamp-samples.front()[0]>cfg.cargo_confirm_sec))samples.pop_front();
    if(samples.size()<4 || stamp-samples.front()[0]<.3)continue;
    const double nz=config_.dual_roi_anchor.noise_deadband_z;
    double gap_min=1e9,gap_max=-1e9,mount_min=1e9,mount_max=-1e9;bool coherent=true;int rise_steps=0;
    for(size_t i=0;i<samples.size();++i){const auto & m=samples[i];
      gap_min=std::min(gap_min,m[1]-m[2]);gap_max=std::max(gap_max,m[1]-m[2]);
      mount_min=std::min(mount_min,m[3]-m[1]);mount_max=std::max(mount_max,m[3]-m[1]);
      if(i){
        const double dh=m[1]-samples[i-1][1],dc=m[2]-samples[i-1][2],ds=m[3]-samples[i-1][3];
        if(dh < -nz || dc < -nz || ds < -nz)coherent=false;
        if(dh>nz*.1 && dc>nz*.1 && ds>nz*.1)++rise_steps;
      }
    }
    hook.hook_rise=hook.hook_z-samples.front()[1];hook.cargo_rise=best.z-samples.front()[2];
    const double rise=std::max(3*nz,cfg.cargo_start_offset_m);
    if(coherent && rise_steps>=3 && hook.hook_rise>rise && hook.cargo_rise>rise &&
        housing.outer_center_z-samples.front()[3]>rise && gap_max-gap_min<=nz && mount_max-mount_min<=nz){
      hook.display_state="STARTING";hook.start_evidence="DIRECT_HOUSING_HOOK_CARGO_Z_RISE_WITH_STABLE_GAPS";
    }
  }

  if(cargo_hits_>0){auto & r=out.cargo_tracking_roi;const double pad=config_.temporal_stabilization.largest_match_dist;
    r.x_min=out.cargo_x-out.length*.5-pad;r.x_max=out.cargo_x+out.length*.5+pad;
    r.y_min=out.cargo_y-out.width*.5-pad;r.y_max=out.cargo_y+out.width*.5+pad;
    r.z_min=out.cargo_z-out.height*.5-pad;r.z_max=out.cargo_z+out.height*.5+pad;
  }
  if(best.id<0 && connection_misses>0)out.status_reason="INSUFFICIENT_CURRENT_RETURNS_ALONG_HOOK_TO_SURFACE";
  if(out.status_reason.empty() || out.status_reason=="NONE")out.status_reason=any_hook?"CURRENT_HOOK_GEOMETRY_CANDIDATE":"HOOK_EVIDENCE_INSUFFICIENT";
  if(best.id>=0 && out.geometry_locked && out.cargo_observation=="DIRECT")rememberCargoRaw(raw,best_raw_ids,stamp);
  out.processing_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
  out.fsm_enabled=cfg.fsm_enabled; // A Cargo-side reset must not disable the configured consumer.
  result_.work_status=out;result_.working_status=out.working_status;
}

void CranePipeline::rememberCargoRaw(const std::vector<PointXYZIRT>&raw,const std::vector<int>&ids,double stamp){
  const auto&w=work_cycle_;if(!w.geometry_locked || w.cargo_track_id<0 || w.cargo_observation!="DIRECT")return;
  cargo_owned_points_.clear();cargo_owned_points_.reserve(ids.size());
  for(int i:ids)if(i>=0&&size_t(i)<raw.size()){const auto&p=raw[i];if(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z))cargo_owned_points_.push_back({p.x,p.y,p.z});}
  if(cargo_owned_points_.size()<config_.work_status.min_candidate_points){cargo_owned_stamp_=-1;return;}
  cargo_owned_id_=w.cargo_track_id;cargo_owned_stamp_=stamp;
}

std::vector<int> CranePipeline::currentOwnedCargoRaw(const std::vector<PointXYZIRT>&raw,double stamp) const{
  std::vector<int> ids;const auto&c=config_.work_status;const auto&w=work_cycle_;
  if(!w.geometry_locked || w.cargo_track_id!=cargo_owned_id_ || cargo_owned_stamp_<0 ||
     stamp<=cargo_owned_stamp_ || stamp-cargo_owned_stamp_>c.cargo_loss_sec || cargo_owned_points_.empty())return ids;
  // Cargo ownership follows measured Cargo returns, never assumed Sheave translation.
  const std::array<double,3>shift{};
  const double cell=c.grid_resolution_m,xy=config_.dual_roi_anchor.noise_deadband_xy,znoise=config_.dual_roi_anchor.noise_deadband_z;
  const int rx=std::max(1,int(std::ceil(xy/cell))),rz=std::max(1,int(std::ceil(znoise/cell)));
  const auto key=[](int x,int y,int z){return ((uint64_t(x)&0x1fffffULL)<<42)|((uint64_t(y)&0x1fffffULL)<<21)|(uint64_t(z)&0x1fffffULL);};
  std::unordered_map<uint64_t,std::vector<std::array<double,3>>>grid;grid.reserve(cargo_owned_points_.size());
  std::array<double,3>lo{{1e9,1e9,1e9}},hi{{-1e9,-1e9,-1e9}};
  for(const auto&p:cargo_owned_points_){std::array<double,3>q{{p[0]+shift[0],p[1]+shift[1],p[2]+shift[2]}};
    grid[key(int(std::floor(q[0]/cell)),int(std::floor(q[1]/cell)),int(std::floor(q[2]/cell)))].push_back(q);
    for(int a=0;a<3;++a){lo[a]=std::min(lo[a],q[a]);hi[a]=std::max(hi[a],q[a]);}}
  for(size_t i=0;i<raw.size();++i){const auto&p=raw[i];
    if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)||p.x<lo[0]-xy||p.x>hi[0]+xy||p.y<lo[1]-xy||p.y>hi[1]+xy||p.z<lo[2]-znoise||p.z>hi[2]+znoise)continue;
    const double u=(p.x-w.cargo_x)*std::cos(w.yaw)+(p.y-w.cargo_y)*std::sin(w.yaw);
    const double v=-(p.x-w.cargo_x)*std::sin(w.yaw)+(p.y-w.cargo_y)*std::cos(w.yaw);
    if(std::abs(u)>.5*(w.length+config_.temporal_stabilization.jump_tolerance_length) ||
       std::abs(v)>.5*(w.width+config_.temporal_stabilization.jump_tolerance_width))continue;
    const int x=int(std::floor(p.x/cell)),y=int(std::floor(p.y/cell)),z=int(std::floor(p.z/cell));bool found=false;
    for(int dz=-rz;dz<=rz&&!found;++dz)for(int dx=-rx;dx<=rx&&!found;++dx)for(int dy=-rx;dy<=rx&&!found;++dy){
      const auto at=grid.find(key(x+dx,y+dy,z+dz));if(at==grid.end())continue;
      for(const auto&q:at->second)if(std::hypot(p.x-q[0],p.y-q[1])<=xy && std::abs(p.z-q[2])<=znoise){found=true;break;}}
    if(found)ids.push_back(int(i));
  }
  if(ids.size()>=c.min_candidate_points){
    static thread_local HookZDensity body_density;
    const auto&roi=config_.roi_filter;
    const double ground_width_x=roi.beam_roi_x_max-roi.beam_roi_x_min,ground_width_y=roi.beam_roi_y_max-roi.beam_roi_y_min;
    const auto ground=body_density.detectGround(raw,w.cargo_x,w.cargo_y,w.cargo_z,ground_width_x,ground_width_y,result_.ground);
    const bool ground_known=ground.value("ground_valid",false);
    const double ground_limit=ground.value("ground_z",0.)+body_density.cfg.value("ground_clearance_m",.15);
    std::map<int,std::set<std::pair<int,int>>>bands;
    for(int id:ids){const auto&p=raw[id];bands[int(std::floor(p.z/cell))].emplace(int(std::floor(p.x/cell)),int(std::floor(p.y/cell)));}
    int peak=0;size_t count=0;for(const auto&b:bands){
      if(ground_known && (b.first+1)*cell<=ground_limit)continue;
      if(b.second.size()>count){peak=b.first;count=b.second.size();}}
    if(count==0)return {};
    double center=(peak+.5)*cell,sd=body_density.stdmax;size_t body_count=0;
    for(int iteration=0;iteration<3;++iteration){
      const double radius=iteration?std::max(cell*.5,body_density.k*sd):body_density.stdmax*body_density.k;
      double sum=0,square=0;body_count=0;
      for(int id:ids)if(std::abs(raw[id].z-center)<=radius){sum+=raw[id].z;++body_count;}
      if(body_count<c.min_candidate_points)return {};
      const double mean=sum/body_count;
      for(int id:ids)if(std::abs(raw[id].z-center)<=radius)square+=(raw[id].z-mean)*(raw[id].z-mean);
      center=mean;sd=std::sqrt(square/body_count);
    }
    if(sd>body_density.stdmax)return {};
    // A supported body mode bounds the upper surface; lower measured returns are retained.
    // Reuse the existing adjustable Z-density sigma settings to reject narrow upward tails.
    const double bottom=center-w.height-znoise,top=center+std::max(cell*.5,body_density.k*sd);
    ids.erase(std::remove_if(ids.begin(),ids.end(),[&](int id){return raw[id].z<bottom || raw[id].z>top;}),ids.end());
  }
  return ids;
}

void CranePipeline::recoverCargoFromRawFragments(const std::vector<PointXYZIRT>&raw,
    const std::vector<CraneSubcluster>&clusters,double stamp){
  auto&w=work_cycle_;const auto&c=config_.work_status;
  if(!w.geometry_locked || w.cargo_observation!="HELD" || w.cargo_track_id!=cargo_owned_id_)return;
  cargo_recovery_indices_=currentOwnedCargoRaw(raw,stamp);cargo_recovery_stamp_=stamp;
  if(cargo_recovery_indices_.size()<c.min_candidate_points)return;
  std::vector<int>best;size_t best_count=0;double best_x=0,best_y=0,best_z=0;
  CraneBeamDetector geometry;
  // Raw ownership is independent of this frame's ephemeral Z/BEV fragment boundaries.
  (void)clusters;
  for(int once=0;once<1;++once){std::vector<int>owned=cargo_recovery_indices_;
    if(owned.size()<c.min_candidate_points)continue;
    double l=0,yaw=0,x=0,y=0;geometry.computeClusterPCA(raw,owned,l,yaw,x,y);double z=0,vlo=1e9,vhi=-1e9;
    for(int id:owned){const auto&p=raw[id];z+=p.z;const double v=-(p.x-x)*std::sin(yaw)+(p.y-y)*std::cos(yaw);vlo=std::min(vlo,v);vhi=std::max(vhi,v);}
    z/=owned.size();const double width=vhi-vlo;
    if(std::hypot(x-cargo_xy_filter_.x(),y-cargo_xy_filter_.y())>config_.temporal_stabilization.largest_match_dist ||
       std::abs(z-cargo_z_filter_.x())>c.connect_xy_margin ||
       l>w.length+config_.temporal_stabilization.jump_tolerance_length || width>w.width+config_.temporal_stabilization.jump_tolerance_width ||
       l<c.cargo_min_surface_width_m || width<c.cargo_min_surface_width_m)continue;
    if((w.length-l>config_.temporal_stabilization.jump_tolerance_length && w.width-width>config_.temporal_stabilization.jump_tolerance_width) ||
       (w.length>=w.width*c.size_ratio_k && l<width*c.size_ratio_k))continue;
    if(computeOccupiedArea(owned,raw,c.grid_resolution_m)<c.cargo_min_fill_ratio*l*width ||
       !hasCargoSurfacePatch(raw,owned,x,y,yaw,c.grid_resolution_m,c.cargo_min_surface_width_m))continue;
    if(owned.size()>best_count){best_count=owned.size();best=std::move(owned);best_x=x;best_y=y;best_z=z;}
  }
  if(best.empty())return;
  const double dt=stamp-cargo_last_seen_;
  if(dt<=0 || dt>c.cargo_loss_sec)return;
  cargo_xy_filter_.predict(dt,.2);cargo_z_filter_.predict(dt,.2);cargo_xy_filter_.update(best_x,best_y,.03);cargo_z_filter_.update(best_z,0,.03);
  if(dt<=std::min(.25,c.cargo_loss_sec))cargo_observed_sec_+=dt;
  cargo_last_seen_=stamp;cargo_last_x_=best_x;cargo_last_y_=best_y;cargo_last_z_=best_z;++cargo_hits_;
  w.cargo_x=cargo_xy_filter_.x();w.cargo_y=cargo_xy_filter_.y();w.cargo_z=cargo_z_filter_.x();w.confirmed_observation_sec=cargo_observed_sec_;
  w.cargo_observation="DIRECT";w.fsm_state=CraneBeamState::LOCKED_TRACKING;w.work_status=true;w.working_status=1;
  w.status_reason="CURRENT_RAW_OWNED_SURFACE_RECOVERY";
  for(int side=0;side<2;++side){auto&h=side?w.right_hook:w.left_hook;const bool own=w.cargo_side=="BOTH" || w.cargo_side==(side?"RIGHT":"LEFT");
    h.object_point_indices=own?best:std::vector<int>{};h.work_status=own;}
  rememberCargoRaw(raw,best,stamp);result_.work_status=w;
}

const CraneZGaussianResult & CranePipeline::prepareCargoExtension(
    const std::vector<PointXYZIRT> & raw,const tf2::Transform & pose,VisibilityModel * current_visibility) {
  cargo_visibility_model_ptr_=current_visibility;
  auto & out=work_cycle_;cargo_extension_layers_.layers.clear();out.extension_ms=0;
  out.extended_raw_indices.clear();out.extension_cluster_ids.clear();out.extension_rejections.clear();
  // These are current Raw observations, never cached geometry or filled gaps.
  out.observed_intervals.clear();out.inferred_gaps.clear();out.observed_bounds={};out.observed_length_sum=0;
  out.extension_observation=out.extension_valid?"HELD":"NONE";
  out.gap_diagnostic="UNKNOWN";out.gap_reason="NO_CURRENT_GEOMETRY";
  out.gap_sides={"UNKNOWN","UNKNOWN","UNKNOWN"};out.gap_roi_points={};out.lower_layer_z={};out.vertical_gaps={};out.gap_confirmed_sec=0;
  if(out.extension_valid && work_last_stamp_>out.extension_last_full_stamp+config_.work_status.cargo_loss_sec)out.extension_valid=false;
  out.extension_observation=out.extension_valid?"HELD":"NONE";
  cargo_recovery_indices_.clear();cargo_recovery_stamp_=-1;
  const bool recovering=out.geometry_locked && out.cargo_observation=="HELD";
  if(recovering){cargo_recovery_indices_=currentOwnedCargoRaw(raw,work_last_stamp_);cargo_recovery_stamp_=work_last_stamp_;}
  if(!out.fsm_enabled || out.cargo_track_id<0 || (out.cargo_observation!="DIRECT" &&
      (!recovering || cargo_recovery_indices_.size()<config_.work_status.min_candidate_points))){
    gap_hits_=0;gap_observed_sec_=0;last_gap_candidate_="UNKNOWN";
    out.extension_reason="NO_CONFIRMED_CURRENT_SEED";
    if(out.fsm_enabled && out.cargo_track_id>=0)updateCargoCompletion(work_last_stamp_);
    result_.work_status=out;return cargo_extension_layers_;
  }
  if(!out.geometry_locked && out.length<std::max(out.width*config_.work_status.size_ratio_k,config_.work_status.cargo_min_surface_width_m)){
    out.extension_reason="SEED_AXIS_UNCERTAIN";updateCargoCompletion(work_last_stamp_);result_.work_status=out;return cargo_extension_layers_;
  }
  const auto start=std::chrono::steady_clock::now();
  const auto & zc=config_.z_gaussian;
  const double range=std::hypot(out.cargo_x-pose.getOrigin().x(),out.cargo_y-pose.getOrigin().y());
  const double ztol=std::max(config_.dual_roi_anchor.noise_deadband_z,
    zc.confidence_sigma_multiplier*(zc.sigma_z+zc.range_sigma_factor*range));
  std::vector<int> extension_seed=recovering?cargo_recovery_indices_:(out.cargo_side=="RIGHT"?out.right_hook.object_point_indices:out.left_hook.object_point_indices);
  double seed_lo=1e9,seed_hi=-1e9;for(int id:extension_seed)if(id>=0&&size_t(id)<raw.size()){seed_lo=std::min(seed_lo,double(raw[id].z));seed_hi=std::max(seed_hi,double(raw[id].z));}
  const double search_center=extension_seed.empty()?out.cargo_z:(seed_lo+seed_hi)*.5;
  if(!extension_seed.empty() && seed_hi-seed_lo>out.height+ztol){out.extension_reason="SEED_CURRENT_RAW_THICKNESS_UNCERTAIN";result_.work_status=out;return cargo_extension_layers_;}
  std::vector<int> selected;selected.reserve(raw.size()/16);
  for(size_t i=0;i<raw.size();++i){const auto & p=raw[i];
    if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))continue;
    // Use the same TOTAL thickness budget later enforced on a fragment.
    // Adding ztol on both sides previously admitted a band wider than that
    // budget, so an otherwise connected Seed component rejected itself.
    if(std::abs(p.z-search_center)>.5*(out.height+ztol))continue;
    selected.push_back(int(i));
  }
  // Same KDE/layer splitter as the Crane ROI, with explicit current Raw provenance.
  processZGaussianGrouping(raw,pose,&selected,&cargo_extension_layers_);
  const auto & seed_ids=extension_seed;
  size_t most_seed_points=0;int chosen=-1;
  for(size_t i=0;i<cargo_extension_layers_.layers.size();++i){size_t n=0;
    for(int id:cargo_extension_layers_.layers[i].point_indices)
      if(std::binary_search(seed_ids.begin(),seed_ids.end(),id))++n;
    if(n>most_seed_points){most_seed_points=n;chosen=int(i);}
  }
  if(chosen>=0 && most_seed_points>=config_.work_status.min_candidate_points){
    auto layer=std::move(cargo_extension_layers_.layers[chosen]);cargo_extension_layers_.layers.clear();cargo_extension_layers_.layers.push_back(std::move(layer));
  }else{cargo_extension_layers_.layers.clear();out.extension_reason="NO_CURRENT_SEED_LAYER_PROVENANCE";}

  out.extension_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
  result_.work_status=out;
  return cargo_extension_layers_;
}

void CranePipeline::updateCargoExtension(const std::vector<PointXYZIRT> & raw,
    const std::vector<CraneSubcluster> & clusters,double stamp,
    const std::function<void()> & build_current_visibility) {
  // The node supplies current Raw depth on demand. Never query a previous frame.
  bool visibility_built=false;
  const auto ensure_current_visibility=[&](){
    if(!visibility_built){
      if(build_current_visibility)build_current_visibility();
      visibility_built=true;
    }
  };
  auto & out=work_cycle_;
  if(!out.fsm_enabled || out.cargo_track_id<0 || out.cargo_observation!="DIRECT" || !std::isfinite(stamp))return;
  if(gap_last_stamp_>0 && stamp<=gap_last_stamp_){out.extension_observation=out.extension_valid?"HELD":"NONE";result_.work_status=out;return;}
  if(out.extension_valid && stamp>out.extension_last_full_stamp+config_.work_status.cargo_loss_sec)out.extension_valid=false;
  out.extension_reason="NO_MATCHING_CURRENT_SEED_FRAGMENT";
  const auto begin=std::chrono::steady_clock::now();
  const std::array<double,7> prior_geometry{{out.extended_x,out.extended_y,out.extended_z,out.extended_length,out.extended_width,out.extended_height,out.extended_yaw}};
  const bool had_geometry=out.extension_valid;
  const auto & cfg=config_.work_status;const auto & tc=config_.temporal_stabilization;
  const auto & zc=config_.z_gaussian;
  const double cs=std::cos(out.yaw),sn=std::sin(out.yaw),cell=cfg.grid_resolution_m;
  const auto pose=cargo_visibility_model_ptr_?cargo_visibility_model_ptr_->pose():tf2::Transform::getIdentity();
  const double range=std::hypot(out.cargo_x-pose.getOrigin().x(),out.cargo_y-pose.getOrigin().y());
  const double ztol=std::max(config_.dual_roi_anchor.noise_deadband_z,
    zc.confidence_sigma_multiplier*(zc.sigma_z+zc.range_sigma_factor*range));
  auto & evidence=out.cargo_side=="RIGHT"?right_work_state_.extension_observations:left_work_state_.extension_observations;
  std::vector<std::array<double,8>> next_evidence;
  std::vector<uint8_t> evidence_used(evidence.size(),0);
  // Local association records only; no new object tracker/identity and no upstream mutation.
  struct Fragment {const CraneSubcluster * cluster;double lo,hi;int hits;bool seed;double width,center_v;size_t seed_points;};
  std::vector<Fragment> fragments;
  CraneBeamDetector geometry;
  for(const auto & sc:clusters){
    if(sc.point_indices.size()<cfg.min_candidate_points)continue;
    double l=0,yaw=0,x=0,y=0;geometry.computeClusterPCA(raw,sc.point_indices,l,yaw,x,y);
    double vmin=1e9,vmax=-1e9,umin=1e9,umax=-1e9,z=0;
    for(int id:sc.point_indices){const auto & p=raw[id];
      const double u=(p.x-out.cargo_x)*cs+(p.y-out.cargo_y)*sn;
      const double v=-(p.x-out.cargo_x)*sn+(p.y-out.cargo_y)*cs;
      umin=std::min(umin,u);umax=std::max(umax,u);vmin=std::min(vmin,v);vmax=std::max(vmax,v);z+=p.z;
    }
    z/=sc.point_indices.size();const double width=vmax-vmin;
    const auto & seed_ids=out.cargo_side=="RIGHT"?out.right_hook.object_point_indices:out.left_hook.object_point_indices;
    size_t actual_seed_points=0;for(int id:sc.point_indices)if(std::binary_search(seed_ids.begin(),seed_ids.end(),id))++actual_seed_points;
    const bool seed_component=actual_seed_points>=cfg.min_candidate_points;
    auto reject_seed=[&](const char * reason){out.extension_rejections.emplace_back(sc.subcluster_id,reason);if(seed_component)out.extension_reason=reason;};
    if(l<cfg.cargo_min_surface_width_m || width<cfg.cargo_min_surface_width_m){reject_seed("SEED_COMPONENT_SURFACE_DIMENSIONS");continue;}
    if(!seed_component && l<width*cfg.size_ratio_k){reject_seed("SEED_COMPONENT_AXIS_UNCERTAIN");continue;}
    if(cargo_extension_layers_.layers.empty() || std::abs(z-cargo_extension_layers_.layers.front().mean_z)>ztol){reject_seed("SEED_COMPONENT_HEIGHT_INCONSISTENT");continue;}
    if(sc.max_z-sc.min_z>out.height+ztol){reject_seed("SEED_COMPONENT_THICKNESS_INCONSISTENT");continue;}
    const bool seed_axis_reliable=out.length>=out.width*cfg.size_ratio_k;
    if((!seed_component || seed_axis_reliable) && std::abs(std::remainder(yaw-out.yaw,pi))>tc.yaw_jump_tolerance_deg*pi/180){reject_seed("SEED_COMPONENT_AXIS_INCONSISTENT");continue;}
    // A Raw-connected, provenance-bearing component may be wider than the
    // ROI-clipped seed. Its complete geometry must first be temporally stable.
    if(seed_component && std::abs((vmax+vmin)*.5)>std::max(cell,tc.jump_tolerance_width)){reject_seed("SEED_COMPONENT_LATERAL_INCONSISTENT");continue;}
    // A collinear sparse wire/outline is not a broad beam fragment.
    const double area=computeOccupiedArea(sc.point_indices,raw,cell);
    if(area<cfg.cargo_min_fill_ratio*l*width || !hasCargoSurfacePatch(raw,sc.point_indices,x,y,yaw,cell,cfg.cargo_min_surface_width_m)){reject_seed("SEED_COMPONENT_NO_FILLED_SURFACE");continue;}
    int hits=1;double best=tc.largest_match_dist;int matched=-1;
    for(size_t i=0;i<evidence.size();++i){const auto & p=evidence[i];
      const double dt=stamp-p[0],d=std::hypot(x-p[1],y-p[2]);
      if(evidence_used[i] || dt<=0 || dt>std::min(cfg.cargo_loss_sec,.25) || d>best ||
          std::abs(z-p[3])>ztol || std::abs(l-p[4])>tc.jump_tolerance_length ||
          std::abs(width-p[5])>tc.jump_tolerance_width ||
          (( !seed_component || seed_axis_reliable) && std::abs(std::remainder(yaw-p[6],pi))>tc.yaw_jump_tolerance_deg*pi/180))continue;
      best=d;matched=int(i);
    }
    if(matched>=0){evidence_used[matched]=1;hits=int(evidence[matched][7])+1;}
    next_evidence.push_back({stamp,x,y,z,l,width,yaw,double(hits)});
    // Raw provenance, not an AABB intersection, identifies the Seed component.
    const size_t seed_points=actual_seed_points;
    fragments.push_back({&sc,umin,umax,hits,seed_points>=cfg.min_candidate_points,width,(vmax+vmin)*.5,seed_points});
  }
  evidence=std::move(next_evidence);
  // Provisional Cargo tracks only collect geometry evidence. Public association
  // and dimensions remain unavailable until the existing working confirmation.
  if(!out.work_status){out.extension_valid=false;out.extension_observation="NONE";out.extension_reason="SEED_TEMPORAL_PENDING";updateCargoCompletion(stamp);result_.work_status=out;return;}
  size_t most_overlap=0;int seed_fragment=-1;bool seed_temporal_pending=false;
  for(size_t i=0;i<fragments.size();++i)if(fragments[i].seed){
    if(fragments[i].hits<std::max(2,cfg.confirm_m_on)){seed_temporal_pending=true;continue;}
    if(fragments[i].seed_points>most_overlap){most_overlap=fragments[i].seed_points;seed_fragment=int(i);}
  }
  for(size_t i=0;i<fragments.size();++i)fragments[i].seed=int(i)==seed_fragment;

  std::vector<uint8_t> accepted(fragments.size(),0);
  double lo=1e9,hi=-1e9;
  for(size_t i=0;i<fragments.size();++i)if(fragments[i].seed){
    accepted[i]=1;lo=std::min(lo,fragments[i].lo);hi=std::max(hi,fragments[i].hi);
  }
  if(lo>hi){
    if(seed_temporal_pending)out.extension_reason="FULL_COMPONENT_TEMPORAL_PENDING";
    if(out.extension_reason.rfind("SEED_COMPONENT_",0)!=0 && out.extension_reason!="FULL_COMPONENT_TEMPORAL_PENDING")out.extension_reason="NO_MATCHING_CURRENT_SEED_FRAGMENT";
    out.extension_observation=out.extension_valid?"HELD":"NONE";
    updateCargoCompletion(stamp);result_.work_status=out;return;
  }
  bool changed=true;
  while(changed){changed=false;
    if(out.length<out.width*cfg.size_ratio_k)break; // A broad seed has no trustworthy extension axis.

    for(size_t i=0;i<fragments.size();++i){const auto & f=fragments[i];
      if(accepted[i] || f.hits<std::max(2,cfg.confirm_m_on))continue;
      if(seed_fragment<0 || std::abs(f.width-fragments[seed_fragment].width)>tc.jump_tolerance_width ||
          std::abs(f.center_v-fragments[seed_fragment].center_v)>std::max(cell,tc.jump_tolerance_width*.5))continue;

      const double gap=f.lo>hi?f.lo-hi:(f.hi<lo?lo-f.hi:0);
      // Existing association distance bounds unsupported jumps; never enlarge to fit a Bag.
      if(gap>tc.largest_match_dist)continue;
      if(gap>cell && cargo_visibility_model_ptr_){
        ensure_current_visibility();
        const double mid=f.lo>hi?(f.lo+hi)*.5:(lo+f.hi)*.5;
        const auto v=cargo_visibility_model_ptr_->evaluatePoint(out.cargo_x+mid*cs,out.cargo_y+mid*sn,out.cargo_z);
        // Positive free-space evidence contradicts an occluded continuous beam.
        if(v.reason==VisibilityReason::VISIBLE_EMPTY || v.free_samples>0)continue;
      }
      accepted[i]=1;lo=std::min(lo,f.lo);hi=std::max(hi,f.hi);changed=true;
    }
  }
  for(size_t i=0;i<fragments.size();++i)if(!accepted[i]) {
    const auto & f=fragments[i];const double gap=f.lo>hi?f.lo-hi:(f.hi<lo?lo-f.hi:0);
    const char * reason=f.hits<std::max(2,cfg.confirm_m_on)?"TEMPORAL_PENDING":
      (gap>tc.largest_match_dist?"GAP_EXCEEDS_EXISTING_ASSOCIATION_GATE":"LATERAL_WIDTH_OR_VISIBILITY_MISMATCH");
    out.extension_rejections.emplace_back(f.cluster->subcluster_id,reason);
  }
  out.extended_raw_indices.clear();out.extension_cluster_ids.clear();out.observed_intervals.clear();out.inferred_gaps.clear();
  std::vector<std::array<double,2>> intervals;
  for(size_t i=0;i<fragments.size();++i)if(accepted[i]){
    const auto & f=fragments[i];out.extension_cluster_ids.push_back(f.cluster->subcluster_id);
    out.extended_raw_indices.insert(out.extended_raw_indices.end(),f.cluster->point_indices.begin(),f.cluster->point_indices.end());
    intervals.push_back({f.lo,f.hi});
  }
  // Keep every current, already-owned seed return: extension must not crop away the measured lower surface.
  const auto & current_seed=out.cargo_side=="RIGHT"?out.right_hook.object_point_indices:out.left_hook.object_point_indices;
  out.extended_raw_indices.insert(out.extended_raw_indices.end(),current_seed.begin(),current_seed.end());
  std::sort(out.extended_raw_indices.begin(),out.extended_raw_indices.end());
  out.extended_raw_indices.erase(std::unique(out.extended_raw_indices.begin(),out.extended_raw_indices.end()),out.extended_raw_indices.end());
  std::map<int,std::array<double,2>> raw_bins;
  for(int id:out.extended_raw_indices){const auto & p=raw[id];
    const double u=(p.x-out.cargo_x)*cs+(p.y-out.cargo_y)*sn;const int bin=int(std::floor(u/cell));
    auto [it,inserted]=raw_bins.emplace(bin,std::array<double,2>{u,u});
    if(!inserted){it->second[0]=std::min(it->second[0],u);it->second[1]=std::max(it->second[1],u);}
  }
  int previous=std::numeric_limits<int>::min();
  for(const auto & b:raw_bins){
    if(out.observed_intervals.empty() || b.first>previous+1)out.observed_intervals.push_back(b.second);
    else out.observed_intervals.back()[1]=b.second[1];
    previous=b.first;
  }
  out.observed_length_sum=0;
  for(size_t i=0;i<out.observed_intervals.size();++i){const auto & v=out.observed_intervals[i];
    out.observed_length_sum+=v[1]-v[0];if(i)out.inferred_gaps.push_back({out.observed_intervals[i-1][1],v[0]});
  }
  double vmin=1e9,vmax=-1e9,zmin=1e9,zmax=-1e9;
  out.observed_bounds={1e9,-1e9,1e9,-1e9,1e9,-1e9};
  for(int id:out.extended_raw_indices){const auto & p=raw[id];
    const double v=-(p.x-out.cargo_x)*sn+(p.y-out.cargo_y)*cs;
    vmin=std::min(vmin,v);vmax=std::max(vmax,v);zmin=std::min(zmin,double(p.z));zmax=std::max(zmax,double(p.z));
    for(int axis=0;axis<3;++axis){double q=axis==0?p.x:(axis==1?p.y:p.z);out.observed_bounds[2*axis]=std::min(out.observed_bounds[2*axis],q);out.observed_bounds[2*axis+1]=std::max(out.observed_bounds[2*axis+1],q);}
  }
  out.extended_length=hi-lo;out.extended_width=vmax-vmin;out.extended_height=zmax-zmin;out.extended_yaw=out.yaw;
  geometry.computeClusterPCA(raw,out.extended_raw_indices,out.extended_length,out.extended_yaw,out.extended_x,out.extended_y);
  const double ec=std::cos(out.extended_yaw),es=std::sin(out.extended_yaw);
  double ewmin=1e9,ewmax=-1e9,eumin=1e9,eumax=-1e9;
  for(int id:out.extended_raw_indices){const auto & p=raw[id];const double v=-(p.x-out.extended_x)*es+(p.y-out.extended_y)*ec;
    const double u=(p.x-out.extended_x)*ec+(p.y-out.extended_y)*es;eumin=std::min(eumin,u);eumax=std::max(eumax,u);
    ewmin=std::min(ewmin,v);ewmax=std::max(ewmax,v);}
  out.extended_width=ewmax-ewmin;out.extended_length=eumax-eumin;
  out.extended_x+=(eumin+eumax)*.5*ec-(ewmin+ewmax)*.5*es;
  out.extended_y+=(eumin+eumax)*.5*es+(ewmin+ewmax)*.5*ec;


  out.extended_z=(zmin+zmax)*.5;out.extension_valid=true;out.extension_observation="DIRECT";
  out.extension_reason=out.inferred_gaps.empty()?"CURRENT_RAW_CONNECTED_EXTENT":"TEMPORALLY_ASSOCIATED_FRAGMENTS_IDENTITY_UNVERIFIED";

  const double xy_noise=config_.dual_roi_anchor.noise_deadband_xy;
  const bool partial=had_geometry && (out.extended_length+xy_noise<prior_geometry[3] || out.extended_width+xy_noise<prior_geometry[4]);
  if(partial && stamp>=out.extension_last_full_stamp && stamp-out.extension_last_full_stamp<=cfg.cargo_loss_sec){
    out.extended_x=prior_geometry[0];out.extended_y=prior_geometry[1];out.extended_z=prior_geometry[2];out.extended_length=prior_geometry[3];
    out.extended_width=prior_geometry[4];out.extended_height=prior_geometry[5];out.extended_yaw=prior_geometry[6];
    out.extension_observation="PARTIAL_HELD";out.extension_reason="CURRENT_RAW_SUBSET_PREVIOUS_GEOMETRY_WITHIN_EXISTING_HOLD";
    gap_hits_=0;gap_observed_sec_=0;last_gap_candidate_="UNKNOWN";
    out.extension_ms+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
    updateCargoCompletion(stamp);result_.work_status=out;return;
  }
  out.extension_last_full_stamp=stamp;

  // Oriented side+ / side- / under footprint ROIs, using existing cell/surface scales.
  const double half_u=std::max(cfg.cargo_min_surface_width_m,out.extended_length*.25);
  const double half_v=out.extended_width*.5;
  out.gap_roi_z_min=zmin-cfg.connect_max_gap_z;out.gap_roi_z_max=zmin+ztol;
  std::array<std::vector<int>,3> side_indices;
  for(size_t i=0;i<raw.size();++i){const auto & p=raw[i];
    if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z) || p.z>zmin+ztol || p.z<zmin-cfg.connect_max_gap_z)continue;
    const double u=(p.x-out.extended_x)*ec+(p.y-out.extended_y)*es;
    const double v=-(p.x-out.extended_x)*es+(p.y-out.extended_y)*ec;
    if(std::abs(u)>half_u)continue;
    if(v>=half_v+cell && v<=half_v+cell+cfg.cargo_min_surface_width_m)side_indices[0].push_back(int(i));
    else if(v<=-half_v-cell && v>=-half_v-cell-cfg.cargo_min_surface_width_m)side_indices[1].push_back(int(i));
    else if(std::abs(v)<=half_v)side_indices[2].push_back(int(i));
  }
  for(int side=0;side<3;++side){
    out.gap_roi_points[side]=side_indices[side].size();
    CraneZGaussianResult layers;processZGaussianGrouping(raw,pose,&side_indices[side],&layers);
    const CraneZGaussianLayer * lower=nullptr;
    for(const auto & layer:layers.layers){
      // Exclude the observed beam band itself; no return is never free space.
      if(layer.mean_z>=zmin-ztol || layer.point_indices.size()<size_t(zc.min_layer_points))continue;
      if(!lower || layer.max_z>lower->max_z)lower=&layer;
    }
    if(!lower)continue;
    const double gap=zmin-lower->max_z;out.lower_layer_z[side]=lower->mean_z;out.vertical_gaps[side]=gap;
    if(gap<=2*ztol){out.gap_sides[side]="SUPPORTED";continue;}
    size_t connecting=0;for(int id:side_indices[side])if(raw[id].z>lower->max_z+ztol && raw[id].z<zmin-ztol)++connecting;
    const double v=side==0?half_v+cell+cfg.cargo_min_surface_width_m*.5:(side==1?-half_v-cell-cfg.cargo_min_surface_width_m*.5:0);
    int free_rays=0;
    if(cargo_visibility_model_ptr_)for(int k=-1;k<=1;++k){
      ensure_current_visibility();
      const double u=k*half_u*.5;
      const auto info=cargo_visibility_model_ptr_->evaluatePoint(out.extended_x+u*ec-v*es,out.extended_y+u*es+v*ec,(zmin+lower->max_z)*.5);
      if(!info.has_occluder && info.in_horizontal_fov && info.in_vertical_fov && info.in_valid_range &&
          (info.reason==VisibilityReason::VISIBLE_EMPTY || info.free_samples>0))++free_rays;
    }
    if(connecting<size_t(zc.min_layer_points) && free_rays==3)out.gap_sides[side]="GAP_OBSERVED";
  }
  std::string candidate="UNKNOWN";
  if(out.gap_sides[0]=="SUPPORTED" && out.gap_sides[1]=="SUPPORTED" && out.gap_sides[2]=="SUPPORTED")candidate="SUPPORTED";
  else if(out.gap_sides[0]=="GAP_OBSERVED" && out.gap_sides[1]=="GAP_OBSERVED" && out.gap_sides[2]=="GAP_OBSERVED")candidate="GAP_OBSERVED";
  const double gap_dt=stamp-gap_last_stamp_;
  if(candidate=="UNKNOWN" || candidate!=last_gap_candidate_ || gap_dt<=0 || gap_dt>std::min(cfg.cargo_loss_sec,.25)){
    gap_hits_=candidate=="UNKNOWN"?0:1;gap_observed_sec_=0;
  }else{++gap_hits_;gap_observed_sec_+=gap_dt;}
  last_gap_candidate_=candidate;gap_last_stamp_=stamp;out.gap_confirmed_sec=gap_observed_sec_;
  if(gap_hits_>=std::max(2,cfg.confirm_m_on) && gap_observed_sec_+1e-6>=cfg.cargo_confirm_sec)out.gap_diagnostic=candidate;
  out.gap_reason=out.gap_diagnostic=="GAP_OBSERVED"?"BOTH_SIDES_AND_UNDER_LOWER_LAYERS_WITH_POSITIVE_FREE_RAYS":
    (out.gap_diagnostic=="SUPPORTED"?"BOTH_SIDES_AND_UNDER_CURRENT_NEAR_SUPPORT_LAYERS":"INSUFFICIENT_SUPPORT_VISIBILITY_OR_TEMPORAL_AGREEMENT");
  out.extension_ms+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
  updateCargoCompletion(stamp);
  result_.work_status=out;
}

void CranePipeline::resetSheaveMotion(int side){
  sheave_motion_samples_[side].clear();sheave_motion_id_[side]=-1;sheave_motion_state_[side]=0;
  sheave_stationary_anchor_z_[side]=std::numeric_limits<double>::quiet_NaN();
  sheave_motion_first_stamp_[side]=sheave_motion_last_stamp_[side]=sheave_motion_last_direct_[side]=-1;
}
void CranePipeline::setSheaveMotionFrame(const std::string & frame){
  if(frame!=sheave_motion_frame_){
    if(!sheave_motion_frame_.empty())resetOrderedWork("SENSOR_FRAME_CHANGED");
    for(int side=0;side<2;++side)resetSheaveMotion(side);
    sheave_motion_frame_=frame;
    lifting_cargo_id_=-1;lifting_last_stamp_=-1;lifting_event_until_=0;clearance_observations_.clear();
  }
}
void CranePipeline::updateSheaveMotion(const CraneTopDownHookResult & result,double stamp){
  if(!std::isfinite(stamp) || stamp<=0)return;
  constexpr double window=.8;
  const double noise=config_.dual_roi_anchor.noise_deadband_z;
  const double movement=std::max(noise,config_.dual_roi_anchor.relocate_thresh_z);
  // Same maximum observation interval as the current Work/Cargo evidence path.
  const double max_gap=std::min(config_.work_status.cargo_loss_sec,.25);
  for(int side=0;side<2;++side){
    const auto & h=side==0?result.left_housing:result.right_housing;
    const auto & memory=side==0?result.left_housing_memory:result.right_housing_memory;
    if(!config_.sheave.enabled || !memory.initialized || !memory.ever_confirmed){resetSheaveMotion(side);continue;}
    if(sheave_motion_id_[side]!=memory.track_id || stamp<sheave_motion_last_stamp_[side]){
      resetSheaveMotion(side);sheave_motion_id_[side]=memory.track_id;
    }
    if(stamp==sheave_motion_last_stamp_[side])continue;
    auto & samples=sheave_motion_samples_[side];
    if(sheave_motion_last_stamp_[side]>0 && stamp-sheave_motion_last_stamp_[side]>max_gap){
      // Preserve the stationary anchor through a bounded tracking gap, but never
      // bridge it with direction votes or stale moving evidence.
      samples.clear();sheave_motion_state_[side]=0;sheave_motion_first_stamp_[side]=stamp;
    }
    if(sheave_motion_first_stamp_[side]<0)sheave_motion_first_stamp_[side]=stamp;
    sheave_motion_last_stamp_[side]=stamp;
    const bool direct=memory.confirmed && !memory.dormant && !h.is_held &&
      h.sheave.status==CraneSheaveDetectionStatus::DIRECT_OBSERVED &&
      h.sheave.points_from_current_frame && std::isfinite(h.sheave.outer_center_z);
    const double z=direct?h.sheave.outer_center_z:std::numeric_limits<double>::quiet_NaN();
    samples.push_back({stamp,z,0});
    const double cutoff=stamp-window;
    // Retain one boundary sample to integrate its clipped time interval exactly.
    while(samples.size()>1 && samples[1][0]<=cutoff)samples.pop_front();
    int evidence=0;
    if(direct){
      sheave_motion_last_direct_[side]=stamp;
      double first_z=z,previous_z=z,min_z=z,max_z=z;size_t hits=0,up_steps=0,down_steps=0;
      for(const auto & sample:samples)if(sample[0]>=cutoff && std::isfinite(sample[1])){
        if(hits++==0)first_z=sample[1];
        else if(sample[1]-previous_z>eps)++up_steps;
        else if(previous_z-sample[1]>eps)++down_steps;
        previous_z=sample[1];min_z=std::min(min_z,sample[1]);max_z=std::max(max_z,sample[1]);
      }
      const bool enough=hits>=size_t(std::max({2,config_.sheave.confirm_frames,config_.dual_roi_anchor.min_direct_evidence_frames}));
      auto & anchor=sheave_stationary_anchor_z_[side];
      const bool quiet=enough && max_z-min_z<=noise;
      if(!std::isfinite(anchor) && quiet)anchor=z; // First stable DIRECT position only.
      if(std::isfinite(anchor) && enough){
        const double net=z-first_z,delta=z-anchor;
        const size_t minimum_steps=size_t(std::max(1,config_.dual_roi_anchor.relocate_confirm_frames));
        const double directional_steps=double(up_steps+down_steps);
        // The existing Z movement/deadband gates apply to displacement from the
        // stopped anchor, not a new implicit minimum speed over 0.8 seconds.
        const bool upward=up_steps>=minimum_steps && up_steps>=.75*directional_steps && net>eps;
        const bool downward=down_steps>=minimum_steps && down_steps>=.75*directional_steps && net<-eps;
        if(delta>=movement && upward)evidence=2;
        else if(delta<=-movement && downward)evidence=3;
        else if(quiet && !upward && !downward)evidence=1;
      }
    }
    samples.back()[2]=evidence;
    const double freshness=(std::max(1,config_.temporal_stabilization.node_max_hold_frames)+1)*memory.frame_period;
    if(!direct && (sheave_motion_last_direct_[side]<0 || stamp-sheave_motion_last_direct_[side]>freshness)){
      sheave_motion_state_[side]=0;continue; // UNKNOWN is neither stopped nor motion.
    }
    if(stamp-sheave_motion_first_stamp_[side]+1e-9<window)continue;
    std::array<double,4> duration{};
    for(size_t i=1;i<samples.size();++i){
      const auto & previous=samples[i-1];const auto & current=samples[i];
      if(current[0]-previous[0]>max_gap)continue;
      duration[size_t(previous[2])]+=current[0]-std::max(previous[0],cutoff);
    }
    const double valid=duration[1]+duration[2]+duration[3];
    if(sheave_motion_state_[side]==2 || sheave_motion_state_[side]==3){
      double last_motion=-1;
      for(const auto & sample:samples)if(int(sample[2])==sheave_motion_state_[side])last_motion=sample[0];
      if(last_motion<0 || stamp-last_motion>freshness)sheave_motion_state_[side]=0;
    }
    if(valid+1e-9<.75*window)continue;
    const int previous_state=sheave_motion_state_[side];
    int next=previous_state;
    if(next>0 && duration[size_t(next)]<=.25*window+1e-9)next=0;
    for(int candidate=1;candidate<=3;++candidate)
      if(duration[size_t(candidate)]+1e-9>=.75*window)next=candidate;
    if(next==2 || next==3){
      double last_motion=-1;
      for(const auto & sample:samples)if(int(sample[2])==next)last_motion=sample[0];
      if(last_motion<0 || stamp-last_motion>freshness)next=0;
    }
    sheave_motion_state_[side]=next; // One exclusive state, no simultaneous rise/fall.
    // Rebase once on a confirmed STOP transition; do not chase slow Z drift
    // while the old STATIONARY state is merely being held by hysteresis.
    if(direct && next==1 && previous_state!=1 && evidence==1 && duration[1]+1e-9>=.75*window)
      sheave_stationary_anchor_z_[side]=z;
  }
}

// Downstream measurement/projection only. It never changes Cargo/Beam tracking or DROP.
void CranePipeline::updateBeamGroundClearance(const std::vector<PointXYZIRT> & raw,double stamp,
    const CraneBeamConfig & bc){
  auto & w=work_cycle_;const auto & cfg=config_.work_status;
  if(!std::isfinite(stamp) || stamp<=0 || stamp==lifting_last_stamp_)return;
  w.beam_ground_clearance_valid=false;w.beam_ground_clearance_m=w.beam_bottom_z=w.ground_surface_z=0;
  w.beam_ground_clearance_reason="NO_CURRENT_BEAM";
  const int count=std::max(2,bc.tilt_num_segments);
  w.beam_ground_clearance_samples_m.assign(count,0);w.beam_bottom_samples_z.assign(count,0);
  w.ground_surface_samples_z.assign(count,0);w.beam_ground_clearance_sample_valid.assign(count,false);
  std::vector<int> beam;
  if(w.cargo_observation=="DIRECT"){
    if(w.extension_observation=="DIRECT")beam=w.extended_raw_indices;
    else{
      beam=w.left_hook.object_point_indices;
      beam.insert(beam.end(),w.right_hook.object_point_indices.begin(),w.right_hook.object_point_indices.end());
    }
  }
  std::sort(beam.begin(),beam.end());beam.erase(std::unique(beam.begin(),beam.end()),beam.end());
  beam.erase(std::remove_if(beam.begin(),beam.end(),[&](int i){return i<0 || size_t(i)>=raw.size() ||
    !std::isfinite(raw[i].x) || !std::isfinite(raw[i].y) || !std::isfinite(raw[i].z);}),beam.end());
  if(beam.size()<cfg.min_candidate_points){updatePublicWorkStatus(stamp,bc);result_.work_status=w;return;}
  const double cs=std::cos(w.yaw),sn=std::sin(w.yaw);
  auto u=[&](int i){return (raw[i].x-w.cargo_x)*cs+(raw[i].y-w.cargo_y)*sn;};
  auto v=[&](int i){return -(raw[i].x-w.cargo_x)*sn+(raw[i].y-w.cargo_y)*cs;};
  double lo=1e9,hi=-1e9,vlo=1e9,vhi=-1e9;
  for(int i:beam){lo=std::min(lo,u(i));hi=std::max(hi,u(i));vlo=std::min(vlo,v(i));vhi=std::max(vhi,v(i));}
  const double expected_length=w.extension_valid?std::max(w.length,w.extended_length):w.length;
  if(hi-lo<cfg.cargo_min_surface_width_m || hi-lo+config_.temporal_stabilization.jump_tolerance_length<expected_length){
    w.beam_ground_clearance_reason="BEAM_LENGTH_UNOBSERVED";updatePublicWorkStatus(stamp,bc);result_.work_status=w;return;
  }
  // Independent current Raw support: never feeds Cargo selection/tracking.
  // A calibrated datum only qualifies a measured plane; it is never returned as
  // Ground by itself. Exclude every current Cargo return before fitting.
  std::vector<int> measured_ground;bool measured_ground_prepared=false;
  auto prepare_measured_ground=[&](){
  if(measured_ground_prepared)return;
  measured_ground_prepared=true;
  const double ground_noise=config_.max_vertical_height;
  std::map<std::pair<int,int>,PlaneMoments> floor_cells;
  for(size_t i=0;i<raw.size();++i){const auto&p=raw[i];
    if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)||
       std::binary_search(beam.begin(),beam.end(),int(i)))continue;
    if(u(i)<lo-bc.support_search_margin_xy || u(i)>hi+bc.support_search_margin_xy ||
       v(i)<vlo-bc.support_search_margin_xy || v(i)>vhi+bc.support_search_margin_xy ||
       std::abs(p.z-config_.ground_z)>ground_noise)continue;
    floor_cells[{int(std::floor(p.x/cfg.grid_resolution_m)),int(std::floor(p.y/cfg.grid_resolution_m))}].add(p);
  }
  PlaneMoments floor_moments;double floor_x0=1e9,floor_x1=-1e9,floor_y0=1e9,floor_y1=-1e9;
  for(const auto&cell:floor_cells){PointXYZIRT p;p.x=cell.second.mean[0];p.y=cell.second.mean[1];p.z=cell.second.mean[2];floor_moments.add(p);
    floor_x0=std::min(floor_x0,double(p.x));floor_x1=std::max(floor_x1,double(p.x));floor_y0=std::min(floor_y0,double(p.y));floor_y1=std::max(floor_y1,double(p.y));}
  FastPlane floor_plane;
  const bool measured_floor=floor_cells.size()>=bc.min_support_points &&
    floor_x1-floor_x0>=bc.support_min_xy_extent && floor_y1-floor_y0>=bc.support_min_xy_extent &&
    fitPlane(floor_moments,floor_plane) && floor_plane.rms<=ground_noise*.5 &&
    floor_plane.normal[2]>=std::cos(config_.max_slope_deg*3.14159265358979323846/180) &&
    std::abs(floor_plane.predict(w.cargo_x,w.cargo_y)-config_.ground_z)<=ground_noise;
  if(measured_floor)for(size_t i=0;i<raw.size();++i){const auto&p=raw[i];
    if(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z)&&
       !std::binary_search(beam.begin(),beam.end(),int(i)) &&
       u(i)>=lo-bc.support_search_margin_xy && u(i)<=hi+bc.support_search_margin_xy &&
       v(i)>=vlo-bc.support_search_margin_xy && v(i)<=vhi+bc.support_search_margin_xy &&
       std::abs(p.z-floor_plane.predict(p.x,p.y))<=ground_noise)measured_ground.push_back(i);
  }
  };
  const double width=(hi-lo)/count;
  const double noise=config_.dual_roi_anchor.noise_deadband_z;
  const double bin=std::max(cfg.grid_resolution_m,bc.beam_z_histogram_bin_size);
  const size_t min_band=std::max(size_t(8),bc.min_support_points/3);
  // Reuse the existing support estimator's occupied Z-band + median/MAD policy.
  // No nominal beam thickness, fixed ground height, or held surface is substituted.
  auto band_points=[&](const std::vector<int> & ids,bool highest){
    std::map<int,std::vector<int>> bins;
    for(int i:ids)bins[int(std::floor(raw[i].z/bin))].push_back(i);
    int chosen=0;bool found=false;
    for(const auto & b:bins)if(b.second.size()>=min_band){chosen=b.first;found=true;if(!highest)break;}
    std::vector<int> selected;
    if(found)for(const auto & b:bins)if(std::abs(b.first-chosen)<=1)selected.insert(selected.end(),b.second.begin(),b.second.end());
    return selected;
  };
  auto stats=[&](const std::vector<int> & ids,double & median){
    std::vector<float> zs;for(int i:ids)zs.push_back(raw[i].z);float m=0,mad=0;
    if(ids.size()<min_band || !CraneBeamDetector::computeMedianAndMAD(zs,m,mad) || mad>bc.support_max_z_spread)return false;
    median=m;return true;
  };
  std::vector<std::vector<int>> segments(count);
  for(int i:beam)segments[std::clamp(int((u(i)-lo)/width),0,count-1)].push_back(i);
  w.beam_ground_clearance_reason="BOTTOM_OR_GROUND_UNOBSERVED";
  for(int k=0;k<count;++k){
    // A tilted top face alone must not masquerade as an observed lower body.
    // Reuse the existing occupancy resolution and layer support count to require
    // several directly observed vertical columns, independently in each segment.
    std::map<std::pair<int,int>,std::array<double,2>> columns;
    for(int i:segments[k]){
      auto key=std::make_pair(int(std::floor(u(i)/cfg.grid_resolution_m)),int(std::floor(v(i)/cfg.grid_resolution_m)));
      auto inserted=columns.emplace(key,std::array<double,2>{raw[i].z,raw[i].z});
      inserted.first->second[0]=std::min(inserted.first->second[0],double(raw[i].z));
      inserted.first->second[1]=std::max(inserted.first->second[1],double(raw[i].z));
    }
    size_t vertical_columns=0;for(const auto & column:columns)if(column.second[1]-column.second[0]>=noise)++vertical_columns;
    if(vertical_columns<size_t(std::max(1,config_.z_gaussian.min_layer_points)))continue;
    double bottom=0,top=0;
    const auto measured_bottom=band_points(segments[k],false);
    if(!stats(measured_bottom,bottom) || !stats(band_points(segments[k],true),top) || top-bottom<noise)continue;
    // Sparse legs/returns below a broad top do not establish a whole Beam bottom.
    // Reuse the existing measured-surface fill ratio over this segment footprint.
    if(computeOccupiedArea(measured_bottom,raw,cfg.grid_resolution_m)<
       cfg.cargo_min_fill_ratio*width*(vhi-vlo)){
      w.beam_ground_clearance_reason="BOTTOM_FOOTPRINT_UNOBSERVED";continue;
    }
    prepare_measured_ground();
    std::vector<int> ground,under,left,right;
    double umin=1e9,umax=-1e9,vmin=1e9,vmax=-1e9;
    for(int i:measured_ground){
      // Locally flat Cargo may also carry a slope GROUND label. The same Raw
      // return cannot independently prove both Beam bottom and support Ground.
      if(std::binary_search(beam.begin(),beam.end(),i))continue;
      if(i<0 || size_t(i)>=raw.size() || !std::isfinite(raw[i].z) || !std::isfinite(raw[i].x) ||
          !std::isfinite(raw[i].y) || raw[i].z>bottom+noise)continue;
      const double pu=u(i),pv=v(i);
      if(pu<lo+k*width || pu>lo+(k+1)*width || pv<vlo-bc.support_search_margin_xy || pv>vhi+bc.support_search_margin_xy)continue;
      ground.push_back(i);umin=std::min(umin,pu);umax=std::max(umax,pu);vmin=std::min(vmin,pv);vmax=std::max(vmax,pv);
      if(pv>=vlo && pv<=vhi)under.push_back(i);else if(pv<vlo)left.push_back(i);else right.push_back(i);
    }
    if(ground.size()<bc.min_support_points || std::max(umax-umin,vmax-vmin)<bc.support_min_xy_extent)continue;
    if(under.size()<bc.min_support_points){
      double a=0,b=0;
      if(!stats(band_points(left,true),a) || !stats(band_points(right,true),b) || std::abs(a-b)>noise)continue;
    }
    double surface=0;
    if(!stats(band_points(under.size()>=bc.min_support_points?under:ground,true),surface) || surface>bottom+noise)continue;
    w.beam_bottom_samples_z[k]=bottom;w.ground_surface_samples_z[k]=surface;
    w.beam_ground_clearance_samples_m[k]=bottom-surface;w.beam_ground_clearance_sample_valid[k]=true;
  }
  if(std::all_of(w.beam_ground_clearance_sample_valid.begin(),w.beam_ground_clearance_sample_valid.end(),[](bool valid){return valid;})){
    std::vector<int> order(count);std::iota(order.begin(),order.end(),0);
    std::sort(order.begin(),order.end(),[&](int a,int b){return w.beam_ground_clearance_samples_m[a]<w.beam_ground_clearance_samples_m[b];});
    const int representative=order[count/2];
    w.beam_ground_clearance_m=w.beam_ground_clearance_samples_m[representative];
    w.beam_bottom_z=w.beam_bottom_samples_z[representative];w.ground_surface_z=w.ground_surface_samples_z[representative];
    w.beam_ground_clearance_valid=true;w.beam_ground_clearance_reason="CURRENT_DIRECT_BOTTOM_AND_GROUND";
  }
  updatePublicWorkStatus(stamp,bc);result_.work_status=w;
}

void CranePipeline::updatePublicWorkStatus(double stamp,const CraneBeamConfig & bc){
  lifting_last_stamp_=stamp;
  auto & w=work_cycle_;
  if(ordered_state_==CraneBeamState::SUSPENDED && w.geometry_locked && w.cargo_observation=="DIRECT"){
    const double dt=ordered_last_seed_stamp_>0?stamp-ordered_last_seed_stamp_:0;
    const double dz=cargo_last_z_-ordered_last_seed_z_;
    bool supporting_down=ordered_side_>=0 && sheave_motion_state_[ordered_side_]==3;
    const auto & housing=ordered_side_==0?result_.top_down_hook.left_housing:result_.top_down_hook.right_housing;
    supporting_down=supporting_down && housing.sheave.status==CraneSheaveDetectionStatus::DIRECT_OBSERVED && housing.sheave.points_from_current_frame;
    // Historical direct downward step/cumulative distance + consecutive confirmation.
    const bool direct=dt>0 && dt<=.25;
    if(direct && dz < -bc.cumulative_direction_epsilon){
      ordered_lower_accum_+=-dz;
      const bool beam_down=-dz>bc.lower_dz_threshold ||
        (bc.cumulative_motion_enabled && ordered_lower_accum_>=bc.cumulative_lower_distance);
      ordered_lower_count_=beam_down && supporting_down?ordered_lower_count_+1:0;
    }else{ordered_lower_count_=0;ordered_lower_accum_=0;}
    if(ordered_lower_count_>=std::max(1,bc.lower_confirm_frames)){
      ordered_state_=CraneBeamState::LOWERING;field_drop_elapsed_={0,0};field_drop_last_={-1,-1};
      std::clog<<"[Crane ordered] LOWERING direct Beam and supporting Sheave descent\n";
    }
    ordered_last_seed_stamp_=stamp;ordered_last_seed_z_=cargo_last_z_;
  }else if(w.cargo_observation!="DIRECT"){
    ordered_last_seed_stamp_=-1;ordered_lower_count_=0;ordered_lower_accum_=0;
  }
  updateCargoCompletion(stamp);
  projectOrderedState();result_.work_status=w;
}

// Final output policy only: never changes the trackers' existing motion inputs.
bool CranePipeline::confirmedCraneMoving() const {
  int crane_count=0,sheave_count=0;
  if(config_.work_status.boundary_motion_enabled)crane_count=boundary_move_count_;
  else if(config_.fixed_root.enabled){
    const auto direct=[](const CraneFixedRootState & r){return r.obs_status==CraneRootObservationStatus::OBSERVED &&
      !r.is_held && r.consecutive_missed==0 && r.motion_status==CraneRootMotionStatus::MOVING;};
    crane_count=direct(left_fixed_root_state_)?1:0;
    const auto & left=config_.fixed_root.left_roi;const auto & right=config_.fixed_root.right_roi;
    const bool overlapping_rois=left.x_min<=right.x_max && right.x_min<=left.x_max &&
      left.y_min<=right.y_max && right.y_min<=left.y_max && left.z_min<=right.z_max && right.z_min<=left.z_max;
    const bool shared=overlapping_rois || (left_fixed_root_state_.subcluster_id>=0 &&
      left_fixed_root_state_.subcluster_id==right_fixed_root_state_.subcluster_id);
    if(!shared && direct(right_fixed_root_state_))++crane_count;
  }
  else crane_count=root_move_count_;
  // Public Crane motion is XY only. Keep the original internal motion context
  // for KF/accumulation/structure stabilization untouched.
  return crane_count>=2;
}

void CranePipeline::resetDropCandidate(){
  drop_observations_.clear();drop_window_first_stamp_=drop_window_last_stamp_=-1;drop_candidate_=false;drop_last_evidence_=-1;drop_window_valid_sec_=0;
}
bool CranePipeline::updateDropCandidate(double stamp,int evidence){
  if(!std::isfinite(stamp) || stamp<=0)return drop_candidate_;
  if(drop_window_last_stamp_>=0 && stamp<drop_window_last_stamp_)resetDropCandidate();
  if(drop_window_last_stamp_==stamp)return drop_candidate_;
  if(drop_window_last_stamp_<0){drop_window_first_stamp_=drop_window_last_stamp_=stamp;drop_last_evidence_=evidence;return false;}
  const double dt=stamp-drop_window_last_stamp_;
  // Hold each observation only to the next valid sensor interval (at most 0.25s).
  // Longer gaps are UNKNOWN in their entirety, never back-filled with a new hit.
  drop_observations_.push_back({drop_window_last_stamp_,stamp,
    dt<=std::min(config_.work_status.cargo_loss_sec,.25)?double(drop_last_evidence_):-1.});
  drop_last_evidence_=evidence;
  drop_window_last_stamp_=stamp;
  constexpr double window=2.;
  const double cutoff=stamp-window;
  while(!drop_observations_.empty() && drop_observations_.front()[1]<=cutoff)drop_observations_.pop_front();
  if(stamp-drop_window_first_stamp_+1e-9<window)return drop_candidate_;
  double yes=0,no=0;
  for(const auto & o:drop_observations_){
    const double span=o[1]-std::max(o[0],cutoff);
    if(o[2]>0)yes+=span;else if(o[2]==0)no+=span;
  }
  drop_window_valid_sec_=yes+no;
  // UNKNOWN remains in the fixed 2s denominator. It cannot establish either edge.
  if(yes+1e-9>=.75*window)drop_candidate_=true;
  else if(yes<=.25*window+1e-9 && drop_window_valid_sec_+1e-9>=.75*window)drop_candidate_=false;
  return drop_candidate_;
}

// Field DROP is an event, not proof of a physical downward placement.
void CranePipeline::emitFieldDrop(double stamp,const char * reason){
  auto & out=work_cycle_;
  if(ordered_state_!=CraneBeamState::LOWERING)return;
  ordered_state_=CraneBeamState::LOWERING_CONFIRMED;ordered_completed_=stamp;
  out.fsm_state=CraneBeamState::LOWERING_CONFIRMED;out.work_status=false;out.working_status=out.public_working_status=2;
  drop_started_stamp_=stamp;out.drop_started_stamp=stamp;out.status_reason=reason;
  for(auto * h:{&out.left_hook,&out.right_hook}){h->work_status=false;h->display_state="DROP";}
  std::clog<<"[Crane DROP] reason="<<reason<<" cargo_id="<<out.cargo_track_id<<" sensor_stamp="<<stamp<<"\n";
  field_cargo_rise_.clear();lifting_event_until_=0;result_.working_status=2;result_.work_status=out;
}
void CranePipeline::updateCargoCompletion(double stamp){
  auto & out=work_cycle_;const auto & cfg=config_.work_status;
  if(!std::isfinite(stamp) || stamp<=0 || stamp<=completion_last_stamp_ || ordered_state_!=CraneBeamState::LOWERING)return;
  if(stamp!=lifting_last_stamp_)return;
  completion_last_stamp_=stamp;
  if(out.cargo_track_id!=field_policy_cargo_id_){
    field_policy_cargo_id_=out.cargo_track_id;field_drop_elapsed_={0,0};field_drop_last_={-1,-1};field_cargo_rise_.clear();
  }
  if(!out.work_status || !out.geometry_locked)return; // no provisional-candidate DROP
  if(cargo_last_seen_>0 && stamp-cargo_last_seen_+1e-9>=1.3){resetOrderedWork("CARGO_LOST_RESET_NOT_COMPLETION");return;}
  const bool measured=out.cargo_observation=="DIRECT";
  const bool contact=measured && stamp==lifting_last_stamp_ && out.beam_ground_clearance_valid &&
    std::isfinite(out.beam_ground_clearance_m) && out.beam_ground_clearance_m>=-.05 && out.beam_ground_clearance_m<=.38;
  bool separated=true,have=false;
  for(int side=0;side<2;++side){
    if(out.cargo_side!="BOTH" && out.cargo_side!=(side==0?"LEFT":"RIGHT"))continue;
    have=true;const auto & h=side==0?out.left_hook:out.right_hook;
    if(h.hook_observation!="VIRTUAL_DIRECT"){separated=false;continue;}
    const double u=(h.hook_x-out.cargo_x)*std::cos(out.yaw)+(h.hook_y-out.cargo_y)*std::sin(out.yaw);
    const double v=-(h.hook_x-out.cargo_x)*std::sin(out.yaw)+(h.hook_y-out.cargo_y)*std::cos(out.yaw);
    separated=separated && (std::abs(u)>out.length*.5+cfg.connect_xy_margin ||
      std::abs(v)>out.width*.5+cfg.connect_xy_margin || h.hook_z-cargo_last_z_>cfg.connect_max_gap_z+config_.dual_roi_anchor.noise_deadband_z);
  }
  const std::array<bool,2> evidence{{contact,measured && have && separated}};
  for(int branch=0;branch<2;++branch){
    double dt=field_drop_last_[branch]>0?stamp-field_drop_last_[branch]:0;
    if(!evidence[branch]){field_drop_elapsed_[branch]=0;field_drop_last_[branch]=-1;continue;}
    if(dt>0 && dt<=.25)field_drop_elapsed_[branch]+=dt;else field_drop_elapsed_[branch]=0;
    field_drop_last_[branch]=stamp;
    // Independent 0.3s current-measurement confirmation, no cross-branch votes.
    if(field_drop_elapsed_[branch]+1e-9>=.3){emitFieldDrop(stamp,branch==0?"GROUND_CONTACT":"HOOK_DETACHED");return;}
  }
  result_.work_status=out;
}

void CranePipeline::clearStationaryAccumulation() {
  stationary_frames_.clear();
  batch_frames_.clear();
  accumulated_points_.clear();
  accumulated_indices_.clear();
  has_accumulated_points_ = false;
}

void CranePipeline::updateAccumulatedPoints(
    const std::vector<PointXYZIRT> & points,
    const std::vector<int> & indices,
    double stamp_sec) {
  const auto & cfg = config_.temporal_stabilization;
  if (!cfg.enabled) {
    clearStationaryAccumulation();
    return;
  }

  const bool is_moving = crane_moving_ || external_crane_moving_;
  if (is_moving) {
    clearStationaryAccumulation();
    return;
  }

  const double min_z_thresh = config_.ground_z + config_.z_gaussian.min_height_above_ground;
  const bool filter_ground_z = (config_.z_gaussian.min_height_above_ground > 0.0);

  auto in_crane_accum_roi = [&](float x, float y, float z) -> bool {
    if (!config_.roi_filter.enabled) return true;
    if (config_.roi_filter.expand_beam_roi) {
      return config_.roi_filter.insideBeamRoi(x, y, z);
    }
    return config_.roi_filter.inActiveRoi(x, y, z);
  };

  StationaryFrame cur_frame;
  cur_frame.stamp_sec = stamp_sec;
  cur_frame.roi_points.reserve(2048);
  for (int idx : indices) {
    if (idx < 0 || static_cast<size_t>(idx) >= points.size()) continue;
    const auto & p = points[idx];
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
    if (filter_ground_z && p.z <= min_z_thresh) continue;
    if (in_crane_accum_roi(p.x, p.y, p.z)) {
      cur_frame.roi_points.push_back(p);
    }
  }

  if (cfg.mode == "batch") {
    batch_frames_.push_back(std::move(cur_frame));
    if (static_cast<int>(batch_frames_.size()) >= cfg.window_size) {
      stationary_frames_ = batch_frames_;
      batch_frames_.clear();
    } else if (stationary_frames_.empty()) {
      stationary_frames_ = batch_frames_;
    }
  } else {
    // Sliding Window
    stationary_frames_.push_back(std::move(cur_frame));
    while (static_cast<int>(stationary_frames_.size()) > cfg.window_size) {
      stationary_frames_.pop_front();
    }
  }

  if (stationary_frames_.empty()) {
    has_accumulated_points_ = false;
    return;
  }

  struct VoxelAccumulator {
    uint32_t frame_mask = 0;
    double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
    int count = 0;
    uint16_t ring = 0;
    float intensity = 0.0f;
  };

  const double v_xy = std::max(0.01, cfg.voxel_size_xy);
  const double v_z = std::max(0.01, cfg.voxel_size_z);
  std::unordered_map<uint64_t, VoxelAccumulator> voxel_map;
  voxel_map.reserve(8192);

  auto make_voxel_key = [](int64_t gx, int64_t gy, int64_t gz) -> uint64_t {
    return (static_cast<uint64_t>(gx & 0x1FFFFF) << 42) |
           (static_cast<uint64_t>(gy & 0x1FFFFF) << 21) |
           (static_cast<uint64_t>(gz & 0x1FFFFF));
  };

  for (size_t f = 0; f < stationary_frames_.size(); ++f) {
    const uint32_t f_bit = (1u << std::min(f, size_t(29)));
    for (const auto & p : stationary_frames_[f].roi_points) {
      const int64_t gx = static_cast<int64_t>(std::floor((p.x + 100.0) / v_xy));
      const int64_t gy = static_cast<int64_t>(std::floor((p.y + 100.0) / v_xy));
      const int64_t gz = static_cast<int64_t>(std::floor((p.z + 100.0) / v_z));
      const uint64_t key = make_voxel_key(gx, gy, gz);
      auto & v = voxel_map[key];
      v.frame_mask |= f_bit;
      v.sum_x += p.x;
      v.sum_y += p.y;
      v.sum_z += p.z;
      v.count++;
      v.ring = p.ring;
      v.intensity = p.intensity;
    }
  }

  accumulated_points_.clear();
  accumulated_points_.reserve(voxel_map.size() + 2048);
  // Include current frame's non-crane ROI elevated points at the same voxel resolution
  const uint32_t bg_bit = (1u << 30);
  for (int idx : indices) {
    if (idx < 0 || static_cast<size_t>(idx) >= points.size()) continue;
    const auto & p = points[idx];
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
    if (filter_ground_z && p.z <= min_z_thresh) continue;
    if (!in_crane_accum_roi(p.x, p.y, p.z)) {
      if (config_.roi_filter.expand_beam_roi && config_.roi_filter.inActiveRoi(p.x, p.y, p.z)) {
        const int64_t gx = static_cast<int64_t>(std::floor((p.x + 100.0) / v_xy));
        const int64_t gy = static_cast<int64_t>(std::floor((p.y + 100.0) / v_xy));
        const int64_t gz = static_cast<int64_t>(std::floor((p.z + 100.0) / v_z));
        const uint64_t key = make_voxel_key(gx, gy, gz);
        auto & v = voxel_map[key];
        v.frame_mask |= bg_bit;
        v.sum_x += p.x;
        v.sum_y += p.y;
        v.sum_z += p.z;
        v.count++;
        v.ring = p.ring;
        v.intensity = p.intensity;
      } else {
        accumulated_points_.push_back(p);
      }
    }
  }

  const int K = static_cast<int>(stationary_frames_.size());
  const int min_hits = (K <= 1) ? 1 : std::clamp(cfg.min_hit_count, 2, K);

  for (const auto & [key, cell] : voxel_map) {
    const bool is_bg_voxel = (cell.frame_mask & bg_bit) != 0;
    const int hits = std::bitset<32>(cell.frame_mask & ~bg_bit).count();
    if ((is_bg_voxel || hits >= min_hits) && cell.count > 0) {
      PointXYZIRT pt;
      pt.x = static_cast<float>(cell.sum_x / cell.count);
      pt.y = static_cast<float>(cell.sum_y / cell.count);
      pt.z = static_cast<float>(cell.sum_z / cell.count);
      pt.ring = cell.ring;
      pt.intensity = cell.intensity;
      accumulated_points_.push_back(pt);
    }
  }

  accumulated_indices_.resize(accumulated_points_.size());
  std::iota(accumulated_indices_.begin(), accumulated_indices_.end(), 0);
  has_accumulated_points_ = true;
}

void CranePipeline::stabilizeBeamResult(CraneBeamGeometry & b, double /*stamp_sec*/) {
  const auto & cfg = config_.temporal_stabilization;
  if (!b.detected) return;

  if (!stabilized_beam_.valid) {
    stabilized_beam_.valid = true;
    stabilized_beam_.confirmed_frames = 1;
    stabilized_beam_.missed_frames = 0;
    stabilized_beam_.consecutive_jump_frames = 0;
    stabilized_beam_.seed_subcluster_id = b.seed_subcluster_id;
    stabilized_beam_.main_layer_id = b.seed_layer_id;
    stabilized_beam_.seed_z = b.seed_z;
    stabilized_beam_.centroid_x = b.centroid_x;
    stabilized_beam_.centroid_y = b.centroid_y;
    stabilized_beam_.centroid_z = b.centroid_z;
    stabilized_beam_.length = b.length;
    stabilized_beam_.width = b.width;
    stabilized_beam_.yaw = b.yaw;
    stabilized_beam_.min_x = b.min_x;
    stabilized_beam_.max_x = b.max_x;
    stabilized_beam_.min_y = b.min_y;
    stabilized_beam_.max_y = b.max_y;
    stabilized_beam_.min_z = b.min_z;
    stabilized_beam_.max_z = b.max_z;
    stabilized_beam_.area_xy = b.area_xy;
    stabilized_beam_.point_count = b.point_count;
    return;
  }

  // 0. Guard against background-wall / floor contact leakage in the wide [-8, 8]m layer
  if (stabilized_beam_.length >= 1.5 && stabilized_beam_.length <= 7.5 &&
      b.point_count > 4500 && (b.length > 8.0 || b.width > 1.15)) {
    b.centroid_x = stabilized_beam_.centroid_x;
    b.centroid_y = stabilized_beam_.centroid_y;
    b.centroid_z = stabilized_beam_.centroid_z;
    b.length = stabilized_beam_.length;
    b.width = stabilized_beam_.width;
    b.yaw = stabilized_beam_.yaw;
    b.min_x = stabilized_beam_.min_x;
    b.max_x = stabilized_beam_.max_x;
    b.min_y = stabilized_beam_.min_y;
    b.max_y = stabilized_beam_.max_y;
    b.min_z = stabilized_beam_.min_z;
    b.max_z = stabilized_beam_.max_z;
    return;
  }

  // 1. Spatial footprint check against previous stabilized beam
  const double half_len = std::max(0.40, stabilized_beam_.length * 0.5);
  const double half_wid = std::max(0.25, stabilized_beam_.width * 0.5);
  const double ref_min_x = std::min(stabilized_beam_.min_x, stabilized_beam_.centroid_x - half_len);
  const double ref_max_x = std::max(stabilized_beam_.max_x, stabilized_beam_.centroid_x + half_len);
  const double ref_min_y = std::min(stabilized_beam_.min_y, stabilized_beam_.centroid_y - half_wid);
  const double ref_max_y = std::max(stabilized_beam_.max_y, stabilized_beam_.centroid_y + half_wid);

  const double d_box_x = std::max(0.0, std::max(ref_min_x - b.max_x, b.min_x - ref_max_x));
  const double d_box_y = std::max(0.0, std::max(ref_min_y - b.max_y, b.min_y - ref_max_y));
  const double dist_footprint = std::hypot(d_box_x, d_box_y);
  const double dist_z = std::abs(b.centroid_z - stabilized_beam_.centroid_z);

  // If spatial distance is far outside the footprint (> 1.20m beyond footprint) or unexpected vertical jump
  const bool is_abrupt = (dist_footprint > 1.20) || (dist_footprint > 0.50 && dist_z > 0.80);
  if (is_abrupt) {
    if (++stabilized_beam_.consecutive_jump_frames >= cfg.jump_confirm_frames ||
        stabilized_beam_.missed_frames > 0) {
      // Re-anchor to new position after persistent confirmation or if previous was missed
      stabilized_beam_.centroid_x = b.centroid_x;
      stabilized_beam_.centroid_y = b.centroid_y;
      stabilized_beam_.centroid_z = b.centroid_z;
      stabilized_beam_.length = b.length;
      stabilized_beam_.width = b.width;
      stabilized_beam_.yaw = b.yaw;
      stabilized_beam_.min_x = b.min_x;
      stabilized_beam_.max_x = b.max_x;
      stabilized_beam_.min_y = b.min_y;
      stabilized_beam_.max_y = b.max_y;
      stabilized_beam_.min_z = b.min_z;
      stabilized_beam_.max_z = b.max_z;
      stabilized_beam_.area_xy = b.area_xy;
      stabilized_beam_.point_count = b.point_count;
      stabilized_beam_.seed_subcluster_id = b.seed_subcluster_id;
      stabilized_beam_.main_layer_id = b.seed_layer_id;
      stabilized_beam_.seed_z = b.seed_z;
      stabilized_beam_.consecutive_jump_frames = 0;
      stabilized_beam_.confirmed_frames = 1;
      stabilized_beam_.missed_frames = 0;
    } else {
      // Hold previous stabilized position and dimensions
      b.centroid_x = stabilized_beam_.centroid_x;
      b.centroid_y = stabilized_beam_.centroid_y;
      b.centroid_z = stabilized_beam_.centroid_z;
      b.length = stabilized_beam_.length;
      b.width = stabilized_beam_.width;
      b.yaw = stabilized_beam_.yaw;
      b.min_x = stabilized_beam_.min_x;
      b.max_x = stabilized_beam_.max_x;
      b.min_y = stabilized_beam_.min_y;
      b.max_y = stabilized_beam_.max_y;
      b.min_z = stabilized_beam_.min_z;
      b.max_z = stabilized_beam_.max_z;
    }
    return;
  }

  // 2. Check for sudden dimension jump (spurious merge or transient BEV split)
  const double d_len = std::abs(b.length - stabilized_beam_.length);
  const double d_wid = std::abs(b.width - stabilized_beam_.width);
  if (d_len > cfg.jump_tolerance_length || d_wid > cfg.jump_tolerance_width) {
    if (++stabilized_beam_.consecutive_jump_frames >= cfg.jump_confirm_frames ||
        stabilized_beam_.missed_frames > 0) {
      // Confirmed persistent geometry expansion/change:
      // Fast-adopt during initial acquisition/expansion from a partial seed, otherwise smooth-converge
      const bool fast_initial_expand =
          (stabilized_beam_.confirmed_frames <= 6 && b.length > stabilized_beam_.length) ||
          (stabilized_beam_.length < 1.5 && b.length >= 1.5);
      const double alpha = fast_initial_expand ? 1.0 : cfg.stabilization_alpha;
      stabilized_beam_.length = (1.0 - alpha) * stabilized_beam_.length + alpha * b.length;
      stabilized_beam_.width = (1.0 - alpha) * stabilized_beam_.width + alpha * b.width;
      stabilized_beam_.centroid_x = (1.0 - alpha) * stabilized_beam_.centroid_x + alpha * b.centroid_x;
      stabilized_beam_.centroid_y = (1.0 - alpha) * stabilized_beam_.centroid_y + alpha * b.centroid_y;
      stabilized_beam_.centroid_z = (1.0 - alpha) * stabilized_beam_.centroid_z + alpha * b.centroid_z;
      stabilized_beam_.yaw = (1.0 - alpha) * stabilized_beam_.yaw + alpha * b.yaw;
      stabilized_beam_.min_x = b.min_x;
      stabilized_beam_.max_x = b.max_x;
      stabilized_beam_.min_y = b.min_y;
      stabilized_beam_.max_y = b.max_y;
      stabilized_beam_.min_z = (1.0 - alpha) * stabilized_beam_.min_z + alpha * b.min_z;
      stabilized_beam_.max_z = (1.0 - alpha) * stabilized_beam_.max_z + alpha * b.max_z;
      stabilized_beam_.area_xy = (1.0 - alpha) * stabilized_beam_.area_xy + alpha * b.area_xy;
      stabilized_beam_.point_count = b.point_count;
      stabilized_beam_.seed_subcluster_id = b.seed_subcluster_id;
      stabilized_beam_.main_layer_id = b.seed_layer_id;
      stabilized_beam_.seed_z = b.seed_z;
      if (fast_initial_expand) {
        stabilized_beam_.consecutive_jump_frames = 0;
      }
      stabilized_beam_.missed_frames = 0;
      stabilized_beam_.confirmed_frames++;
      b.length = stabilized_beam_.length;
      b.width = stabilized_beam_.width;
      b.centroid_x = stabilized_beam_.centroid_x;
      b.centroid_y = stabilized_beam_.centroid_y;
      b.centroid_z = stabilized_beam_.centroid_z;
      b.yaw = stabilized_beam_.yaw;
      b.min_z = stabilized_beam_.min_z;
      b.max_z = stabilized_beam_.max_z;
    } else {
      // Reject transient 1~2 frame spike/split! Hold stabilized dimensions
      b.length = stabilized_beam_.length;
      b.width = stabilized_beam_.width;
      b.min_z = stabilized_beam_.min_z;
      b.max_z = stabilized_beam_.max_z;
    }
  } else {
    // Normal match: smooth update
    stabilized_beam_.consecutive_jump_frames = 0;
    stabilized_beam_.missed_frames = 0;
    stabilized_beam_.confirmed_frames++;
    stabilized_beam_.seed_subcluster_id = b.seed_subcluster_id;
    stabilized_beam_.main_layer_id = b.seed_layer_id;
    stabilized_beam_.seed_z = b.seed_z;
    const double alpha = cfg.stabilization_alpha;
    stabilized_beam_.length = (1.0 - alpha) * stabilized_beam_.length + alpha * b.length;
    stabilized_beam_.width = (1.0 - alpha) * stabilized_beam_.width + alpha * b.width;
    stabilized_beam_.centroid_x = (1.0 - alpha) * stabilized_beam_.centroid_x + alpha * b.centroid_x;
    stabilized_beam_.centroid_y = (1.0 - alpha) * stabilized_beam_.centroid_y + alpha * b.centroid_y;
    stabilized_beam_.centroid_z = (1.0 - alpha) * stabilized_beam_.centroid_z + alpha * b.centroid_z;
    stabilized_beam_.yaw = (1.0 - alpha) * stabilized_beam_.yaw + alpha * b.yaw;
    stabilized_beam_.min_x = b.min_x;
    stabilized_beam_.max_x = b.max_x;
    stabilized_beam_.min_y = b.min_y;
    stabilized_beam_.max_y = b.max_y;
    stabilized_beam_.min_z = (1.0 - alpha) * stabilized_beam_.min_z + alpha * b.min_z;
    stabilized_beam_.max_z = (1.0 - alpha) * stabilized_beam_.max_z + alpha * b.max_z;
    stabilized_beam_.area_xy = (1.0 - alpha) * stabilized_beam_.area_xy + alpha * b.area_xy;
    stabilized_beam_.point_count = b.point_count;
    b.length = stabilized_beam_.length;
    b.width = stabilized_beam_.width;
    b.centroid_x = stabilized_beam_.centroid_x;
    b.centroid_y = stabilized_beam_.centroid_y;
    b.centroid_z = stabilized_beam_.centroid_z;
    b.yaw = stabilized_beam_.yaw;
    b.min_z = stabilized_beam_.min_z;
    b.max_z = stabilized_beam_.max_z;
  }
}

namespace {
static inline CraneFixedRootRoi makeBoxRoi(double cx, double cy, double cz, double s_xy, double s_z) {
  CraneFixedRootRoi roi;
  roi.x_min = cx - 0.5 * s_xy;
  roi.x_max = cx + 0.5 * s_xy;
  roi.y_min = cy - 0.5 * s_xy;
  roi.y_max = cy + 0.5 * s_xy;
  roi.z_min = cz - 0.5 * s_z;
  roi.z_max = cz + 0.5 * s_z;
  return roi;
}
} // namespace

WireBundleGeometry CranePipeline::resolveWireBundleGeometry(
    const CraneHookChainNode & rn,
    const std::vector<CraneSubcluster> & subclusters,
    const std::vector<PointXYZIRT> & points,
    const TrackedChainNodeState * prior_sn,
    const CraneVirtualRopeConfig & vr_cfg,
    const CraneDualRoiAnchorConfig & dual_cfg) {
  WireBundleGeometry geom;
  geom.center_x = rn.x;
  geom.center_y = rn.y;
  geom.center_z = rn.z;
  geom.mode = CraneWireBundleMode::PROVISIONAL_SINGLE;

  if (!vr_cfg.enabled) {
    return geom;
  }

  // 1. Collect candidate wire subclusters in this layer within horizontal proximity
  std::vector<const CraneSubcluster *> candidates;
  for (const auto & sc : subclusters) {
    if (sc.point_indices.size() < 3) continue;
    if (rn.layer_id >= 0 && sc.layer_id >= 0 && sc.layer_id != rn.layer_id) continue;
    if (isBeamLikeSubcluster(sc, dual_cfg)) continue;
    const double dz = std::abs(sc.centroid_z - rn.z);
    if (dz > 0.30) continue;
    const double dxy = std::hypot(sc.centroid_x - rn.x, sc.centroid_y - rn.y);
    if (dxy > 0.45) continue;
    candidates.push_back(&sc);
  }

  // Case A: 2 or more subclusters in the layer
  if (candidates.size() >= 2) {
    std::sort(candidates.begin(), candidates.end(),
              [](const CraneSubcluster * a, const CraneSubcluster * b) {
                return a->point_indices.size() > b->point_indices.size();
              });
    const auto * s1 = candidates[0];
    const auto * s2 = candidates[1];
    const double d = std::hypot(s1->centroid_x - s2->centroid_x, s1->centroid_y - s2->centroid_y);
    if (d >= vr_cfg.wire_min_spacing_m && d <= vr_cfg.wire_max_spacing_m) {
      const bool s1_is_a = (s1->centroid_x < s2->centroid_x) ||
                           (s1->centroid_x == s2->centroid_x && s1->centroid_y < s2->centroid_y);
      const auto * sa = s1_is_a ? s1 : s2;
      const auto * sb = s1_is_a ? s2 : s1;

      geom.mode = CraneWireBundleMode::BOTH_STRANDS;
      geom.center_x = 0.5 * (sa->centroid_x + sb->centroid_x);
      geom.center_y = 0.5 * (sa->centroid_y + sb->centroid_y);
      geom.center_z = rn.z;
      geom.strand_a_x = sa->centroid_x; geom.strand_a_y = sa->centroid_y; geom.strand_a_valid = true;
      geom.strand_b_x = sb->centroid_x; geom.strand_b_y = sb->centroid_y; geom.strand_b_valid = true;
      geom.spacing_dx = sb->centroid_x - sa->centroid_x;
      geom.spacing_dy = sb->centroid_y - sa->centroid_y;
      geom.spacing_dist = d;
      geom.confidence_frames = prior_sn ? (prior_sn->wire_bundle.confidence_frames + 1) : 1;
      return geom;
    }
  }

  // Case B: Merged subcluster (check internal point distribution for 2 modes along major span)
  const double span_x = std::max(0.0, rn.max_x - rn.min_x);
  const double span_y = std::max(0.0, rn.max_y - rn.min_y);
  const double span = std::max(span_x, span_y);

  if (span >= vr_cfg.wire_merge_span_m && rn.point_indices.size() >= 8) {
    const bool major_x = (span_x >= span_y);
    double min_val = 1e9, max_val = -1e9;
    for (int idx : rn.point_indices) {
      if (idx < 0 || static_cast<size_t>(idx) >= points.size()) continue;
      const auto & pt = points[idx];
      const double val = major_x ? pt.x : pt.y;
      min_val = std::min(min_val, val);
      max_val = std::max(max_val, val);
    }

    if (max_val - min_val >= vr_cfg.wire_min_spacing_m) {
      double c1 = min_val + 0.25 * (max_val - min_val);
      double c2 = max_val - 0.25 * (max_val - min_val);
      double mean_x1 = 0, mean_y1 = 0, mean_x2 = 0, mean_y2 = 0;
      int cnt1 = 0, cnt2 = 0;

      for (int iter = 0; iter < 5; ++iter) {
        mean_x1 = mean_y1 = mean_x2 = mean_y2 = 0;
        cnt1 = cnt2 = 0;
        for (int idx : rn.point_indices) {
          if (idx < 0 || static_cast<size_t>(idx) >= points.size()) continue;
          const auto & pt = points[idx];
          const double val = major_x ? pt.x : pt.y;
          if (std::abs(val - c1) < std::abs(val - c2)) {
            mean_x1 += pt.x; mean_y1 += pt.y; cnt1++;
          } else {
            mean_x2 += pt.x; mean_y2 += pt.y; cnt2++;
          }
        }
        if (cnt1 > 0) { mean_x1 /= cnt1; mean_y1 /= cnt1; c1 = major_x ? mean_x1 : mean_y1; }
        if (cnt2 > 0) { mean_x2 /= cnt2; mean_y2 /= cnt2; c2 = major_x ? mean_x2 : mean_y2; }
      }

      const double d = std::hypot(mean_x1 - mean_x2, mean_y1 - mean_y2);
      if (d >= vr_cfg.wire_min_spacing_m && d <= vr_cfg.wire_max_spacing_m && cnt1 >= 3 && cnt2 >= 3) {
        const bool c1_is_a = (mean_x1 < mean_x2) || (mean_x1 == mean_x2 && mean_y1 < mean_y2);
        const double xa = c1_is_a ? mean_x1 : mean_x2;
        const double ya = c1_is_a ? mean_y1 : mean_y2;
        const double xb = c1_is_a ? mean_x2 : mean_x1;
        const double yb = c1_is_a ? mean_y2 : mean_y1;

        geom.mode = CraneWireBundleMode::BOTH_STRANDS;
        geom.center_x = 0.5 * (xa + xb);
        geom.center_y = 0.5 * (ya + yb);
        geom.center_z = rn.z;
        geom.strand_a_x = xa; geom.strand_a_y = ya; geom.strand_a_valid = true;
        geom.strand_b_x = xb; geom.strand_b_y = yb; geom.strand_b_valid = true;
        geom.spacing_dx = xb - xa;
        geom.spacing_dy = yb - ya;
        geom.spacing_dist = d;
        geom.confidence_frames = prior_sn ? (prior_sn->wire_bundle.confidence_frames + 1) : 1;
        return geom;
      }
    }
  }

  // Case C: Single observed strand or clump -> use remembered offset if available
  const bool has_memory = prior_sn &&
                          (prior_sn->wire_bundle_memory_age <= vr_cfg.memory_frames) &&
                          (prior_sn->remembered_bundle.mode == CraneWireBundleMode::BOTH_STRANDS);

  if (has_memory) {
    const auto & mem = prior_sn->remembered_bundle;
    const double w = mem.spacing_dist;
    const double ux = (w > 1e-4) ? (mem.spacing_dx / w) : 1.0;
    const double uy = (w > 1e-4) ? (mem.spacing_dy / w) : 0.0;
    const double dx = rn.x - mem.center_x;
    const double dy = rn.y - mem.center_y;
    const double proj = dx * ux + dy * uy;

    if (proj < -0.20 * w) {
      // Strand A observed
      geom.mode = CraneWireBundleMode::ESTIMATED_FROM_SINGLE_A;
      geom.center_x = rn.x + 0.5 * mem.spacing_dx;
      geom.center_y = rn.y + 0.5 * mem.spacing_dy;
      geom.center_z = rn.z;
      geom.strand_a_x = rn.x; geom.strand_a_y = rn.y; geom.strand_a_valid = true;
      geom.strand_b_x = rn.x + mem.spacing_dx; geom.strand_b_y = rn.y + mem.spacing_dy; geom.strand_b_valid = false;
      geom.spacing_dx = mem.spacing_dx; geom.spacing_dy = mem.spacing_dy; geom.spacing_dist = w;
      geom.confidence_frames = prior_sn->wire_bundle.confidence_frames;
      return geom;
    } else if (proj > 0.20 * w) {
      // Strand B observed
      geom.mode = CraneWireBundleMode::ESTIMATED_FROM_SINGLE_B;
      geom.center_x = rn.x - 0.5 * mem.spacing_dx;
      geom.center_y = rn.y - 0.5 * mem.spacing_dy;
      geom.center_z = rn.z;
      geom.strand_a_x = rn.x - mem.spacing_dx; geom.strand_a_y = rn.y - mem.spacing_dy; geom.strand_a_valid = false;
      geom.strand_b_x = rn.x; geom.strand_b_y = rn.y; geom.strand_b_valid = true;
      geom.spacing_dx = mem.spacing_dx; geom.spacing_dy = mem.spacing_dy; geom.spacing_dist = w;
      geom.confidence_frames = prior_sn->wire_bundle.confidence_frames;
      return geom;
    } else {
      // Overlapped at center
      geom.mode = CraneWireBundleMode::OVERLAPPED;
      geom.center_x = rn.x;
      geom.center_y = rn.y;
      geom.center_z = rn.z;
      geom.spacing_dx = mem.spacing_dx; geom.spacing_dy = mem.spacing_dy; geom.spacing_dist = w;
      geom.confidence_frames = prior_sn->wire_bundle.confidence_frames;
      return geom;
    }
  }

  // Case D: Provisional single strand
  geom.mode = CraneWireBundleMode::PROVISIONAL_SINGLE;
  geom.center_x = rn.x;
  geom.center_y = rn.y;
  geom.center_z = rn.z;
  geom.strand_a_x = rn.x; geom.strand_a_y = rn.y; geom.strand_a_valid = true;
  geom.spacing_dist = 0.0;
  geom.confidence_frames = 1;
  return geom;
}

bool CranePipeline::matchHousingReference(CraneSheaveGeometry & g, const CraneSheaveConfig & cfg, const HousingGeometryMemory * memory) {
  g.negative_reference_rejected=false;g.reference_from_learned=false;
  g.reference_id.clear();g.reference_family_id=-1;g.reference_matched=false;g.reference_trusted=false;
  g.reference_score=-1;g.reference_local_score=-1;g.shape_reliability=0;
  if(!cfg.reference_matching_enabled || g.point_count<cfg.min_arc_points)return false;
  const double sizes[3]={g.housing_length,g.housing_width,g.housing_height};
  for(double size:sizes)if(!std::isfinite(size) || size<=0)return false;
  const bool reverse=g.wire_dir_x< -1e-6 || (std::abs(g.wire_dir_x)<=1e-6 && g.wire_dir_y<0);
  double best=std::numeric_limits<double>::infinity();
  double positive_shape=best,negative_shape=best;
  const size_t base_count=std::min(size_t{8},cfg.references.size());
  const bool learned_context=memory && memory->initialized && memory->ever_confirmed &&
      memory->last_frame_stamp>=memory->last_direct_stamp &&
      memory->last_frame_stamp-memory->last_direct_stamp<=3.0 &&
      g.wire_supported && g.wire_observation_mode==2;
  const size_t learned_count=learned_context?std::min(size_t{4},cfg.learned_references.size()):0;
  for(size_t i=0;i<base_count+learned_count;++i){
    const auto & r=i<base_count?cfg.references[i]:cfg.learned_references[i-base_count];
    if(i>=base_count && memory->reference_family_id>=0 && r.family_id!=memory->reference_family_id)continue;
    if(i>=base_count){
      // Few-shot observations must not extrapolate into the unlabelled upper
      // Root/merged-body view. This restricts only the additional learned bank.
      if(g.sensor_range<=0 || r.sensor_range<=0 || g.sensor_range<.75*r.sensor_range ||
          g.sensor_range>4.0/3*r.sensor_range)continue;
      double view_dot=0;for(int k=0;k<3;++k)view_dot+=g.view_direction[k]*r.view_direction[k];
      if(view_dot<.98)continue;
    }
    if(r.label=="AMBIGUOUS" || r.id.empty() || !r.manually_checked)continue;
    bool compatible=true;double score=0,z_difference=0,horizontal_forward=0,horizontal_reverse=0;
    for(int k=0;k<3;++k){
      if(!std::isfinite(r.dimensions[k]) || r.dimensions[k]<=0 || sizes[k]<.5*r.dimensions[k] || sizes[k]>2*r.dimensions[k])compatible=false;
      score+=std::abs(g.shape_eigenvalues[k]-r.spectrum[k]);
      z_difference+=std::abs(g.z_distribution[k]-r.z_distribution[k]);
      score+=.2*std::abs(g.z_distribution[k]-r.z_distribution[k]);
      horizontal_forward+=std::abs(g.horizontal_distribution[reverse?2-k:k]-r.horizontal_distribution[k]);
      horizontal_reverse+=std::abs(g.horizontal_distribution[reverse?k:2-k]-r.horizontal_distribution[k]);
    }
    // The strand/PCA axis is an unoriented line: rotating the same observed
    // surface through 180 degrees reverses its local profile. Compare both
    // signs, retaining the same dimensions, vertical-profile and score gates.
    score+=.5*std::min(horizontal_forward,horizontal_reverse);
    // A similar PCA must not compensate for an incompatible visible surface profile.
    if(!compatible || z_difference>.5 || !std::isfinite(score) || score>.5)continue;
    // Current single-frame angular support must cover at least the quarter
    // projected area implied by the existing half-size dimension bound.
    // LiDAR return count scales approximately with projected area / range^2;
    // this rejects a sparse partial patch without imposing a fixed point count
    // for all distances. No source position or side is used.
    if(r.has_local_shape && r.sensor_range>0 && g.sensor_range>0 && r.point_count>0 &&
        double(g.point_count)*std::pow(g.sensor_range/r.sensor_range,2)<.25*r.point_count)continue;
    double local_difference=-1;
    if(r.has_local_shape){
      local_difference=std::numeric_limits<double>::infinity();
      // PCA axes are unoriented and may exchange order near a repeated
      // eigenvalue. Enumerate the bounded 6 permutations x 8 signs.
      std::array<int,3> perm{0,1,2};
      do {
        bool local_sizes=true;
        for(int k=0;k<3;++k)if(g.local_extent[perm[k]]<.5*r.local_extent[k] ||
            g.local_extent[perm[k]]>2*r.local_extent[k])local_sizes=false;
        if(!local_sizes)continue;
        for(int signs=0;signs<8;++signs){double distance=0;
          for(int a=0;a<3;++a)for(int b=0;b<3;++b)for(int c=0;c<3;++c){
            const int target[3]={a,b,c};int source[3];
            for(int k=0;k<3;++k)source[perm[k]]=(signs&(1<<k))?2-target[k]:target[k];
            distance+=std::abs(g.local_occupancy[source[0]*9+source[1]*3+source[2]]-r.local_occupancy[a*9+b*3+c]);
          }
          local_difference=std::min(local_difference,distance);
        }
      }while(std::next_permutation(perm.begin(),perm.end()));
      // A broad surface with matching moments is insufficient when its
      // actual local spatial distribution does not match a learned surface.
      if(!std::isfinite(local_difference) || local_difference>.5)continue;
    }
    const double shape_score=score+(local_difference>=0?.1*local_difference:0);
    // Explicit, user-verified non-Housing surfaces are hard negatives. Their
    // class comparison contains no source XYZ, side, range or bag identity.
    if(r.label=="NEGATIVE"){negative_shape=std::min(negative_shape,shape_score);continue;}
    positive_shape=std::min(positive_shape,shape_score);
    // Range/view select an observed appearance; they never relax the evidence gates.
    double context=local_difference>=0?.1*local_difference:0;
    if(g.sensor_range>0 && r.sensor_range>0){
      context+=.02*std::abs(std::log(g.sensor_range/r.sensor_range));
      double dot=0;for(int k=0;k<3;++k)dot+=g.view_direction[k]*r.view_direction[k];
      context+=.02*(1-std::clamp(dot,-1.0,1.0));
    }
    if(score+context<best){
      best=score+context;g.reference_from_learned=i>=base_count;g.reference_id=r.id;g.reference_family_id=r.family_id;
      g.reference_matched=true;g.reference_trusted=r.trusted();g.reference_score=best;g.reference_local_score=local_difference;
      // Evidence indicator, not a probability or a physical surface-coverage estimate.
      g.shape_reliability=std::clamp(r.reliability,0.0,1.0)*std::min(1.0,double(g.point_count)/std::max(1.0,4.0*cfg.min_arc_points))*(1-score/.5);
    }
  }
  g.negative_reference_rejected=std::isfinite(negative_shape) &&
      (!g.reference_matched || negative_shape<=positive_shape);
  if(g.reference_matched && negative_shape<=positive_shape){
    g.reference_from_learned=false;g.reference_id.clear();g.reference_family_id=-1;g.reference_matched=false;g.reference_trusted=false;
    g.reference_score=-1;g.reference_local_score=-1;g.shape_reliability=0;
  }
  return g.reference_matched;
}

CraneSheaveGeometry CranePipeline::detectSheaveCandidate(
    const CraneHookChainNode & wire_node,
    const std::vector<CraneSubcluster> & subclusters,
    const std::vector<PointXYZIRT> & points,
    const WireBundleGeometry & wire_bundle,
    const CraneSheaveConfig & cfg,
    const CraneDualRoiAnchorConfig & dual,
    const CraneSheaveGeometry * /*prior_sheave*/,
    const HousingGeometryMemory * memory,
    const std::vector<PointXYZIRT> * current_observation_points,
    const tf2::Vector3 * sensor_origin) {
  const auto detection_begin=std::chrono::steady_clock::now();
  std::vector<int> candidates;int rejected=0,wire_rejected=0;double matching_ms=0.0;
  CraneSheaveGeometry best;std::array<int,12> reasons{};
  std::array<double,3> rejected_center{};double rejected_distance=-1;std::string rejected_reason;
  auto reject=[&](int gate,const char * reason,const CraneSheaveGeometry & g){
    ++reasons[gate];const double x=memory&&memory->initialized?memory->housing_center_x:wire_node.x;
    const double y=memory&&memory->initialized?memory->housing_center_y:wire_node.y;
    const double z=memory&&memory->initialized?memory->housing_center_z:wire_node.z;
    const double d=std::hypot(std::hypot(g.outer_center_x-x,g.outer_center_y-y),g.outer_center_z-z);
    if(rejected_distance<0 || d<rejected_distance){rejected_distance=d;rejected_center={g.outer_center_x,g.outer_center_y,g.outer_center_z};rejected_reason=reason;}
  };
  if (!cfg.enabled) return best;
  const double ref_x = wire_bundle.mode != CraneWireBundleMode::NONE ? wire_bundle.center_x : wire_node.x;
  const double ref_y = wire_bundle.mode != CraneWireBundleMode::NONE ? wire_bundle.center_y : wire_node.y;
  const auto & observed_points=current_observation_points?*current_observation_points:points;
  const double gap = std::max(0.0, cfg.surface_gap_m);

  // Infer the strand separation direction from current returns near this wire
  // reference. In the supplied Raw, housing spans across that direction while
  // the right hook is narrow in it. This gate also works after a horizontal rotation.
  Eigen::Vector2d plane_axis(1,0);bool plane_axis_ready=false;
  auto ensure_plane_axis=[&](){
    if(plane_axis_ready)return;
    plane_axis_ready=true;
    Eigen::Vector2d plane_mean=Eigen::Vector2d::Zero();
    Eigen::Matrix2d plane_cov=Eigen::Matrix2d::Zero();int plane_count=0;
    for(const auto &p:observed_points){if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))continue;
      if(std::abs(p.z-wire_node.z)>dual.moving_roi_size_z*0.5 || std::hypot(p.x-ref_x,p.y-ref_y)>0.45)continue;
      Eigen::Vector2d d(p.x-ref_x,p.y-ref_y);plane_mean+=d;plane_cov+=d*d.transpose();++plane_count;
    }
    if(plane_count>=6){plane_cov-=plane_mean*plane_mean.transpose()/plane_count;
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> axis(plane_cov);
      if(axis.info()==Eigen::Success && axis.eigenvalues()[1]>1e-12)plane_axis=axis.eigenvectors().col(1);
    }
  };
  // Broad surfaces only. Thin strands cannot bridge separate bodies in this graph.
  std::vector<const CraneSubcluster *> surfaces;
  for (const auto & sc : subclusters) {
    const double length = std::max(sc.max_x-sc.min_x, sc.max_y-sc.min_y);
    const double width = std::min(sc.max_x-sc.min_x, sc.max_y-sc.min_y);
    if (sc.point_indices.size() < std::max(size_t{3}, cfg.min_arc_points) || isBeamLikeSubcluster(sc, dual) ||
        length < 0.12 || width < 0.06 || length > cfg.max_sheave_width_m ||
        sc.max_z-sc.min_z < 0.08) {++reasons[0];continue;}
    if (wire_node.z-sc.centroid_z < 0.02 || wire_node.z-sc.centroid_z > 1.30 ||
        std::hypot(sc.centroid_x-ref_x, sc.centroid_y-ref_y) > 0.45) {++reasons[1];continue;}
    ensure_plane_axis();
    double plane_min=std::numeric_limits<double>::infinity(),plane_max=-plane_min;
    for(int i:sc.point_indices)if(i>=0 && static_cast<size_t>(i)<points.size()){
      const double along=points[i].x*plane_axis.x()+points[i].y*plane_axis.y();
      plane_min=std::min(plane_min,along);plane_max=std::max(plane_max,along);
    }
    if(plane_max-plane_min<0.12){++reasons[2];continue;}
    surfaces.push_back(&sc);
  }
  auto valid = [&](int i) {
    return i >= 0 && static_cast<size_t>(i) < points.size() &&
      std::isfinite(points[i].x) && std::isfinite(points[i].y) && std::isfinite(points[i].z);
  };
  std::vector<bool> used(surfaces.size(), false);
  double best_score = std::numeric_limits<double>::infinity();
  for (size_t seed=0; seed<surfaces.size(); ++seed) {
    if (used[seed]) continue;
    CraneSubcluster body = *surfaces[seed]; used[seed] = true;
    for (bool changed=true; changed;) {
      changed=false;
      for (size_t j=0;j<surfaces.size();++j) {
        if (used[j]) continue;
        const auto & other=*surfaces[j];
        const double dx=std::max(body.max_x,other.max_x)-std::min(body.min_x,other.min_x);
        const double dy=std::max(body.max_y,other.max_y)-std::min(body.min_y,other.min_y);
        const double dz=std::max(body.max_z,other.max_z)-std::min(body.min_z,other.min_z);
        if (std::max(dx,dy)>cfg.max_sheave_width_m || dz>dual.moving_roi_size_z+gap) continue;
        bool connected=false;
        for (int a:body.point_indices) {
          if (!valid(a)) continue;
          for(int b:other.point_indices) {
            if (!valid(b)) continue;
            const auto & p=points[a];const auto & q=points[b];
            if (std::hypot(std::hypot(p.x-q.x,p.y-q.y),p.z-q.z)<=gap) {connected=true;break;}
          }
          if(connected)break;
        }
        if(!connected)continue;
        body.point_indices.insert(body.point_indices.end(),other.point_indices.begin(),other.point_indices.end());
        body.min_x=std::min(body.min_x,other.min_x);body.max_x=std::max(body.max_x,other.max_x);
        body.min_y=std::min(body.min_y,other.min_y);body.max_y=std::max(body.max_y,other.max_y);
        body.min_z=std::min(body.min_z,other.min_z);body.max_z=std::max(body.max_z,other.max_z);
        used[j]=true;changed=true;
      }
    }
    CraneSheaveGeometry g;
    g.subcluster_id=body.subcluster_id;g.layer_id=body.layer_id;
    std::sort(body.point_indices.begin(),body.point_indices.end());
    body.point_indices.erase(std::unique(body.point_indices.begin(),body.point_indices.end()),body.point_indices.end());
    std::vector<int> observed_indices;
    if(current_observation_points){
      // Accumulation may propose a surface, but only current raw returns can
      // supply direct evidence, descriptors and confirmation hits.
      for(size_t i=0;i<observed_points.size();++i){const auto &p=observed_points[i];
        if(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z) &&
            p.x>=body.min_x && p.x<=body.max_x && p.y>=body.min_y && p.y<=body.max_y && p.z>=body.min_z && p.z<=body.max_z)observed_indices.push_back(i);
      }
    }else for(int i:body.point_indices)if(valid(i))observed_indices.push_back(i);
    g.points_from_current_frame=current_observation_points!=nullptr;
    Eigen::Vector3d mean=Eigen::Vector3d::Zero();
    g.point_indices=std::move(observed_indices);
    for(int i:g.point_indices)mean+=Eigen::Vector3d(observed_points[i].x,observed_points[i].y,observed_points[i].z);
    g.point_count=g.point_indices.size();if(g.point_count<std::max(size_t{3}, cfg.min_arc_points)){++reasons[3];continue;}
    mean/=g.point_count;Eigen::Matrix3d cov=Eigen::Matrix3d::Zero();
    g.min_x=g.max_x=observed_points[g.point_indices.front()].x;
    g.min_y=g.max_y=observed_points[g.point_indices.front()].y;
    g.min_z=g.max_z=observed_points[g.point_indices.front()].z;
    for(int i:g.point_indices){const auto&p=observed_points[i];Eigen::Vector3d d=Eigen::Vector3d(p.x,p.y,p.z)-mean;cov+=d*d.transpose();
      g.min_x=std::min(g.min_x,double(p.x));g.max_x=std::max(g.max_x,double(p.x));
      g.min_y=std::min(g.min_y,double(p.y));g.max_y=std::max(g.max_y,double(p.y));
      g.min_z=std::min(g.min_z,double(p.z));g.max_z=std::max(g.max_z,double(p.z));}
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(cov);
    if(eig.info()!=Eigen::Success || !eig.eigenvalues().allFinite() || eig.eigenvalues()[2]<=1e-12){++reasons[4];continue;}
    // A line (including a dense line) and two vertical strands lack a broad surface.
    if(eig.eigenvalues()[1]/eig.eigenvalues()[2]<0.08){++reasons[4];continue;}
    for(int k=0;k<3;++k)g.shape_eigenvalues[k]=std::max(0.0,eig.eigenvalues()[2-k])/eig.eigenvalues().sum();
    // Reuse the candidate covariance basis; visit this small current surface,
    // never the full Raw again. Absolute XYZ is not part of the descriptor.
    Eigen::Matrix3d basis;for(int k=0;k<3;++k)basis.col(k)=eig.eigenvectors().col(2-k);
    Eigen::Vector3d local_min=Eigen::Vector3d::Constant(1e9),local_max=Eigen::Vector3d::Constant(-1e9);
    for(int i:g.point_indices){const auto & p=observed_points[i];
      const Eigen::Vector3d v=basis.transpose()*(Eigen::Vector3d(p.x,p.y,p.z)-mean);
      local_min=local_min.cwiseMin(v);local_max=local_max.cwiseMax(v);
    }
    for(int k=0;k<3;++k)g.local_extent[k]=local_max[k]-local_min[k];
    for(int i:g.point_indices){const auto & p=observed_points[i];
      const Eigen::Vector3d v=basis.transpose()*(Eigen::Vector3d(p.x,p.y,p.z)-mean);int bin[3];
      for(int k=0;k<3;++k)bin[k]=std::clamp(int(3*(v[k]-local_min[k])/std::max(1e-9,g.local_extent[k])),0,2);
      g.local_occupancy[bin[0]*9+bin[1]*3+bin[2]]+=1.0/g.point_count;
    }

    g.housing_length=std::max(g.max_x-g.min_x,g.max_y-g.min_y);
    g.housing_width=std::min(g.max_x-g.min_x,g.max_y-g.min_y);g.housing_height=g.max_z-g.min_z;
    if(g.housing_length<0.12 || g.housing_width<0.06 || g.housing_height<0.08){++reasons[5];continue;}
    for(int i:g.point_indices){int k=std::min(2,int(3*(observed_points[i].z-g.min_z)/g.housing_height));g.z_distribution[k]+=1.0/g.point_count;}
    g.outer_center_x=mean.x();g.outer_center_y=mean.y();g.outer_center_z=mean.z();

    candidates.push_back(body.subcluster_id);
    // Current returns above the broad body must contain two compact strand groups.
    // Do not turn a remembered/synthetic strand into new observation evidence.
    double ux=1,uy=0;
    if(wire_bundle.spacing_dist>0.05){ux=wire_bundle.spacing_dx/wire_bundle.spacing_dist;uy=wire_bundle.spacing_dy/wire_bundle.spacing_dist;}
    Eigen::Matrix2d strand_cov=Eigen::Matrix2d::Zero();Eigen::Vector2d strand_mean=Eigen::Vector2d::Zero();int strand_count=0;
    for(const auto&p:observed_points){if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))continue;
      if(p.z<g.max_z+gap || p.z>g.max_z+dual.moving_roi_size_z)continue;
      if(p.x<g.min_x-gap || p.x>g.max_x+gap || p.y<g.min_y-gap || p.y>g.max_y+gap)continue;
      Eigen::Vector2d d(p.x-mean.x(),p.y-mean.y());strand_mean+=d;strand_cov+=d*d.transpose();++strand_count;
    }
    if(strand_count>=6){strand_cov-=strand_mean*strand_mean.transpose()/strand_count;
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> axis(strand_cov);
      if(axis.info()==Eigen::Success && axis.eigenvalues()[1]>1e-12){ux=axis.eigenvectors()(0,1);uy=axis.eigenvectors()(1,1);}
    }
    std::vector<double> strand_s;std::vector<std::array<double,3>> strand_samples;
    double sum_t=0;
    for(const auto&p:observed_points){if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))continue;
      if(p.z<g.max_z+gap || p.z>g.max_z+dual.moving_roi_size_z)continue;
      double dx=p.x-mean.x(),dy=p.y-mean.y();double t=-dx*uy+dy*ux,s=dx*ux+dy*uy;
      if(std::abs(t)>0.12 || std::abs(s)>0.45)continue;
      strand_s.push_back(s);strand_samples.push_back({s,p.z,t});sum_t+=t;
    }
    std::sort(strand_s.begin(),strand_s.end());
    double max_gap=0;size_t split=0;
    for(size_t k=3;k+3<=strand_s.size();++k){double d=strand_s[k]-strand_s[k-1];if(d>max_gap){max_gap=d;split=k;}}
    auto compact_span = [&](size_t begin, size_t end) {
      const size_t trim=(end-begin)/10; // Suppress sparse off-strand returns, not body points.
      return strand_s[end-1-trim]-strand_s[begin+trim];
    };
    if(split>0 && max_gap>=0.10 && max_gap<=0.45 &&
        compact_span(0,split)<=0.16 && compact_span(split,strand_s.size())<=0.16){
      double a=std::accumulate(strand_s.begin(),strand_s.begin()+split,0.0)/split;
      double b=std::accumulate(strand_s.begin()+split,strand_s.end(),0.0)/(strand_s.size()-split);
      g.wire_offset_xy=std::hypot((a+b)*0.5,sum_t/strand_s.size());
      g.wire_supported=g.wire_offset_xy<=0.5*g.housing_length;
      g.wire_observation_mode=2;g.wire_spacing=b-a;g.wire_mid_s=(a+b)*0.5;g.wire_mid_t=sum_t/strand_s.size();
    }
    // Dense current strand modes can be separated even when sparse returns
    // bridge the empty gap. Use only for a reviewed-reference proposal, and
    // require overlapping vertical support plus a majority of current returns.
    if(!g.wire_supported && cfg.reference_wire_mode_search && strand_samples.size()>=6){
      const double radius=std::max(.01,std::min(.04,gap*.5)),bin=radius*.5;
      std::map<int,int> histogram;
      for(const auto & s:strand_samples)++histogram[int(std::floor(s[0]/bin))];
      struct Mode {double s=0,t=0,zmin=1e9,zmax=-1e9;int count=0,bin_id=0;};
      std::vector<Mode> modes;
      auto window_count=[&](int id){int n=0;const double c=(id+.5)*bin;for(const auto & v:strand_samples)if(std::abs(v[0]-c)<=radius)++n;return n;};
      for(const auto & entry:histogram){
        const int n=window_count(entry.first);
        // A tail window is not a second strand: retain actual smoothed local peaks only.
        if(n<window_count(entry.first-1) || n<=window_count(entry.first+1))continue;
        const double center=(entry.first+.5)*bin;Mode m;m.bin_id=entry.first;
        for(const auto & s:strand_samples)if(std::abs(s[0]-center)<=radius){m.s+=s[0];m.t+=s[2];m.zmin=std::min(m.zmin,s[1]);m.zmax=std::max(m.zmax,s[1]);++m.count;}
        if(m.count>=3){m.s/=m.count;m.t/=m.count;modes.push_back(m);}
      }
      int support=0;Mode first,second;
      for(const auto & a:modes)for(const auto & b:modes){
        const double spacing=b.s-a.s;
        if(spacing<.10 || spacing>.45 || std::abs(a.t-b.t)>.12 ||
            std::min(a.zmax,b.zmax)-std::max(a.zmin,b.zmin)<gap)continue;
        int valley=std::min(a.count,b.count);
        for(int id=a.bin_id+1;id<b.bin_id;++id)valley=std::min(valley,window_count(id));
        if(valley*2>std::min(a.count,b.count))continue;
        if(a.count+b.count>support){support=a.count+b.count;first=a;second=b;}
      }
      if(support*2>=int(strand_samples.size()) && support>0){
        g.wire_mid_s=(first.s+second.s)*.5;g.wire_mid_t=(first.t+second.t)*.5;
        g.wire_spacing=second.s-first.s;g.wire_offset_xy=std::hypot(g.wire_mid_s,g.wire_mid_t);
        g.wire_supported=g.wire_offset_xy<=g.housing_length*.5;
        if(g.wire_supported){g.wire_observation_mode=2;g.reference_assisted=true;}
      }
    }
    if(!g.wire_supported && memory && memory->confirmed && !memory->dormant && memory->wire_spacing>0.10){
      // Only a previously paired, shape-matched track may use a current single
      // strand/overlap. This is never evidence for initializing a new identity.
      ux=memory->wire_dir_x;uy=memory->wire_dir_y;std::vector<double> single;double transverse=0;
      for(const auto&p:observed_points){
        if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z) || p.z<g.max_z+gap || p.z>g.max_z+dual.moving_roi_size_z)continue;
        const double dx=p.x-mean.x(),dy=p.y-mean.y(),t=-dx*uy+dy*ux,s=dx*ux+dy*uy;
        if(std::abs(t-memory->wire_mid_t)>0.12 || std::abs(s-memory->wire_mid_s)>0.45)continue;
        single.push_back(s);transverse+=t;
      }
      if(single.size()>=3){std::sort(single.begin(),single.end());size_t trim=single.size()/10;
        double center=std::accumulate(single.begin()+trim,single.end()-trim,0.0)/(single.size()-2*trim);
        double ds=std::abs(center-memory->wire_mid_s),dt=transverse/single.size()-memory->wire_mid_t;
        if(single[single.size()-1-trim]-single[trim]<=0.16 && std::abs(dt)<=dual.noise_deadband_xy){
          if(std::abs(ds-memory->wire_spacing*0.5)<=dual.noise_deadband_xy)g.wire_observation_mode=1;
          else if(ds<=dual.noise_deadband_xy)g.wire_observation_mode=3;
          g.wire_supported=g.wire_observation_mode!=0;g.wire_offset_xy=memory->wire_offset_xy;
        }
      }
    }
    // Eigenvectors have arbitrary signs. Align the current strand frame with
    // the frozen template instead of accepting a mirrored asymmetric surface.
    if(memory && memory->initialized && ux*memory->wire_dir_x+uy*memory->wire_dir_y<0){
      ux=-ux;uy=-uy;g.wire_mid_s=-g.wire_mid_s;g.wire_mid_t=-g.wire_mid_t;
    }
    double min_s=1e9,max_s=-1e9;
    for(int i:g.point_indices){const auto&p=observed_points[i];double along=(p.x-mean.x())*ux+(p.y-mean.y())*uy;min_s=std::min(min_s,along);max_s=std::max(max_s,along);}
    if(max_s-min_s>1e-6)for(int i:g.point_indices){const auto&p=observed_points[i];double along=(p.x-mean.x())*ux+(p.y-mean.y())*uy;
      int bin=std::clamp(int(3*(along-min_s)/(max_s-min_s)),0,2);g.horizontal_distribution[bin]+=1.0/g.point_count;}
    g.wire_dir_x=ux;g.wire_dir_y=uy;g.axis_dir_x=-uy;g.axis_dir_y=ux;
    const double area=2*(g.housing_length*g.housing_width+g.housing_length*g.housing_height+g.housing_width*g.housing_height);
    g.point_density=g.point_count/std::max(1e-6,area);
    if(sensor_origin){
      tf2::Vector3 v(mean.x()-sensor_origin->x(),mean.y()-sensor_origin->y(),mean.z()-sensor_origin->z());
      g.sensor_range=v.length();if(g.sensor_range>1e-6)for(int k=0;k<3;++k)g.view_direction[k]=v[k]/g.sensor_range;
    }
    const auto match_begin=std::chrono::steady_clock::now();
    matchHousingReference(g,cfg,memory);
    matching_ms+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-match_begin).count();
    // A current wire connection is supporting evidence, not a Housing identity.
    // Apply the configured library to ordinary proposals as well as recovery
    // proposals; an unmatched cold candidate must never seed a track or Hook.
    // A recent, already confirmed identity can bridge a short sampling/profile
    // dropout. Current broad Raw + a current paired strand are mandatory.
    // This never seeds identity, bypasses a hard negative, changes a family, or
    // refreshes the reference timestamp from its own recovered observations.
    if(cfg.track_continuity_enabled && cfg.reference_matching_enabled && !g.reference_matched &&
        !g.negative_reference_rejected && memory && memory->confirmed && !memory->dormant &&
        memory->ever_confirmed && g.wire_supported && g.wire_observation_mode==2 &&
        memory->recent_reference_stamp>0 && memory->recent_reference_family>=0 &&
        (memory->reference_family_id<0 || memory->reference_family_id==memory->recent_reference_family) &&
        memory->last_frame_stamp-memory->recent_reference_stamp<=
          memory->recent_reference_timeout_sec+1e-6){
      const double dx=g.outer_center_x-memory->housing_center_x,dy=g.outer_center_y-memory->housing_center_y;
      const double dz=g.outer_center_z-memory->housing_center_z;
      const double rx=g.outer_center_x-memory->recent_center[0],ry=g.outer_center_y-memory->recent_center[1];
      bool sizes=true;double spectrum=0,horizontal=0,reverse_horizontal=0;
      const double size[3]={g.housing_length,g.housing_width,g.housing_height};
      for(int k=0;k<3;++k){
        sizes=sizes && size[k]>=.75*memory->recent_dimensions[k] && size[k]<=4.0/3*memory->recent_dimensions[k];
        spectrum+=std::abs(g.shape_eigenvalues[k]-memory->recent_spectrum[k]);
        horizontal+=std::abs(g.horizontal_distribution[k]-memory->recent_horizontal[k]);
        reverse_horizontal+=std::abs(g.horizontal_distribution[2-k]-memory->recent_horizontal[k]);
      }
      const double range_ratio=memory->recent_range>0 && g.sensor_range>0?g.sensor_range/memory->recent_range:1.0;
      if(sizes && spectrum<=.12 && std::min(horizontal,reverse_horizontal)<=.20 &&
          g.point_count*range_ratio*range_ratio>=.5*memory->recent_point_count &&
          std::hypot(dx,dy)<=dual.noise_deadband_xy && std::abs(dz)<=dual.noise_deadband_z &&
          std::hypot(rx,ry)<=dual.moving_roi_size_xy*.25 &&
          std::abs(g.outer_center_z-memory->recent_center[2])<=dual.moving_roi_size_z*.5){
        g.track_memory_matched=true;g.geometry_matched=true;
        g.recovery_reference_id=memory->recent_reference_id;
        // Preserve the failed bank result/score; track evidence is separate.
      }
    }
    if(cfg.reference_matching_enabled && !g.reference_matched && !g.track_memory_matched){
      reject(7,"NO_HOUSING_REFERENCE",g);++rejected;
      continue;
    }
    // A reviewed surface can establish identity when the two reeving strands
    // overlap in this view. This path is available only to reviewed-reference
    // proposals AFTER matching the current broad Raw surface, never to a Chain
    // node or a wire-only candidate. Require a compact, vertically extended
    // current strand physically adjacent to the observed top of that surface.
    if(!g.wire_supported && cfg.reference_wire_mode_search &&
        g.reference_matched && g.reference_trusted && g.reference_score<=.25 && strand_samples.size()>=6){
      // Use the dominant current strand mode, rather than sparse off-strand
      // returns. The existing paired-mode search uses the same local window.
      const double radius=std::max(.01,std::min(.04,gap*.5)),bin=radius*.5;
      std::map<int,int> modes;for(const auto & v:strand_samples)++modes[int(std::floor(v[0]/bin))];
      double mode_center=0;size_t mode_count=0;
      for(const auto & entry:modes){const double center=(entry.first+.5)*bin;size_t count=0;
        for(const auto & v:strand_samples)if(std::abs(v[0]-center)<=radius)++count;
        if(count>mode_count){mode_count=count;mode_center=center;}}
      double smin=1e9,smax=-1e9,tmin=1e9,tmax=-1e9,zmin=1e9,zmax=-1e9,sm=0,tm=0;
      for(const auto & v:strand_samples)if(std::abs(v[0]-mode_center)<=radius){smin=std::min(smin,v[0]);smax=std::max(smax,v[0]);
        zmin=std::min(zmin,v[1]);zmax=std::max(zmax,v[1]);
        tmin=std::min(tmin,v[2]);tmax=std::max(tmax,v[2]);sm+=v[0];tm+=v[2];}
      const double offset=std::hypot(sm/std::max(size_t{1},mode_count),tm/std::max(size_t{1},mode_count));
      if(mode_count>=6 && mode_count*2>=strand_samples.size() && smax-smin<=.08 && tmax-tmin<=.08 && zmax-zmin>=.12 &&
          zmin-g.max_z<=gap+.03 && offset<=g.housing_length*.5){
        g.wire_supported=true;g.wire_observation_mode=3;g.wire_offset_xy=offset;
        g.wire_mid_s=sm/mode_count;g.wire_mid_t=tm/mode_count;
        g.reference_assisted=true;
      }
    }
    if(!g.wire_supported){reject(6,"CURRENT_WIRE_SUPPORT",g);++wire_rejected;continue;}
    if(cfg.reference_matching_enabled && !g.track_memory_matched && memory && memory->initialized && memory->reference_family_id>=0 &&
        (!g.reference_matched || !g.reference_trusted || g.reference_family_id!=memory->reference_family_id)){
      std::string reason=std::string("REFERENCE_FAMILY selected=")+g.reference_id+" family="+std::to_string(g.reference_family_id)+" trusted="+std::to_string(g.reference_trusted)+" score="+std::to_string(g.reference_score);
      for(const auto & r:cfg.references)if(r.trusted() && r.family_id==memory->reference_family_id){
        bool sizes_ok=true;double zd=0,score=0;const double sizes[3]={g.housing_length,g.housing_width,g.housing_height};
        const bool reverse=g.wire_dir_x< -1e-6 || (std::abs(g.wire_dir_x)<=1e-6 && g.wire_dir_y<0);
        for(int k=0;k<3;++k){sizes_ok=sizes_ok && sizes[k]>=.5*r.dimensions[k] && sizes[k]<=2*r.dimensions[k];
          zd+=std::abs(g.z_distribution[k]-r.z_distribution[k]);score+=std::abs(g.shape_eigenvalues[k]-r.spectrum[k]);
          score+=.2*std::abs(g.z_distribution[k]-r.z_distribution[k]);
          score+=.5*std::abs(g.horizontal_distribution[reverse?2-k:k]-r.horizontal_distribution[k]);}
        reason+=" template="+r.id+" sizes_ok="+std::to_string(sizes_ok)+" z_delta="+std::to_string(zd)+" shape_score="+std::to_string(score);
      }
      reject(7,reason.c_str(),g);++rejected;continue;
    }

    if(g.reference_assisted && !g.track_memory_matched && (!g.reference_matched || !g.reference_trusted)){reject(7,"UNTRUSTED_ASSISTED_REFERENCE",g);++rejected;continue;}
    double score=std::hypot(mean.x()-ref_x,mean.y()-ref_y)+0.2*(wire_node.z-mean.z());
    if(memory && memory->initialized){
      const double dxy=std::hypot(mean.x()-memory->housing_center_x,mean.y()-memory->housing_center_y);
      const double dz=std::abs(mean.z()-memory->housing_center_z);
      if(dxy>dual.moving_roi_size_xy || dz>dual.moving_roi_size_z){reject(8,"MEMORY_POSITION",g);++rejected;continue;}
      const double sizes[3]={g.housing_length,g.housing_width,g.housing_height};
      const double old[3]={memory->length,memory->width,memory->height};
      bool compatible=true;double shape_diff=0;
      for(int k=0;k<3;++k){if(sizes[k]<0.5*old[k] || sizes[k]>2*old[k])compatible=false;shape_diff+=std::abs(g.shape_eigenvalues[k]-memory->shape_eigenvalues[k]);}
      for(int k=0;k<3;++k)shape_diff+=0.2*std::abs(g.z_distribution[k]-memory->z_distribution[k]);
      double horizontal=0;
      for(int k=0;k<3;++k)horizontal+=std::abs(g.horizontal_distribution[k]-memory->horizontal_distribution[k]);
      shape_diff+=0.5*horizontal;
      const bool reviewed_alternate=g.reference_matched && g.reference_trusted &&
          memory->reference_family_id>=0 && g.reference_family_id==memory->reference_family_id;
      if(!compatible || (shape_diff>0.5 && !reviewed_alternate) || std::abs(g.wire_offset_xy-memory->wire_offset_xy)>dual.moving_roi_size_xy*0.5){reject(9,!compatible?"MEMORY_DIMENSIONS":((shape_diff>0.5 && !reviewed_alternate)?"MEMORY_SHAPE":"MEMORY_WIRE_OFFSET"),g);++rejected;continue;}
      score+=dxy+dz+shape_diff;g.geometry_matched=true;g.match_score=dxy+dz+shape_diff;
    }

    // Observed housing surface != observed internal wheel. No nominal axis or offset.
    g.status=CraneSheaveDetectionStatus::DIRECT_OBSERVED;
    g.axis_status=CraneSheaveDetectionStatus::NONE;g.axis_uncertain=true;
    if(score<best_score){best=std::move(g);best_score=score;}
  }
  std::sort(candidates.begin(),candidates.end());candidates.erase(std::unique(candidates.begin(),candidates.end()),candidates.end());
  best.rejection_counts=reasons;best.closest_rejected_center=rejected_center;best.closest_rejected_distance=rejected_distance;best.closest_rejection_reason=rejected_reason;
  best.candidate_ids=candidates;best.candidate_count=candidates.size();best.geometry_rejections=rejected;best.wire_rejections=wire_rejected;
  best.matching_ms=matching_ms;best.detection_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-detection_begin).count()-matching_ms;
  return best;
}

bool CranePipeline::appendSheaveTerminal(CraneHookChain & output, const CraneHookChainNode & terminal) {
  const auto & sh=terminal.sheave;
  if(terminal.is_held || sh.status!=CraneSheaveDetectionStatus::DIRECT_OBSERVED ||
      sh.axis_status!=CraneSheaveDetectionStatus::DIRECT_OBSERVED || sh.axis_uncertain ||
      !sh.wire_supported || !std::isfinite(sh.axis_center_x) ||
      !std::isfinite(sh.axis_center_y) || !std::isfinite(sh.axis_center_z))return false;
  // Validate a direct wire link before editing the output copy. No Z-only pruning.
  size_t parent=0;
  for(size_t i=1;i<output.nodes.size();++i){const auto & n=output.nodes[i];
    const double dz=n.z-sh.axis_center_z;
    if(!n.is_held && !n.is_sheave && dz>0.02 && dz<=1.30 &&
        std::hypot(n.x-sh.axis_center_x,n.y-sh.axis_center_y)<=0.45)parent=i;
  }
  if(parent==0)return false;
  for(size_t i=0;i<=parent;++i)if(output.nodes[i].z<=sh.axis_center_z)return false;
  output.nodes.resize(parent+1);
  auto node=terminal;node.is_sheave=true;node.x=sh.axis_center_x;node.y=sh.axis_center_y;node.z=sh.axis_center_z;
  node.parent_subcluster_id=output.nodes.back().subcluster_id;
  node.gap_z_to_parent=output.nodes.back().z-node.z;
  node.dxy_to_parent=std::hypot(output.nodes.back().x-node.x,output.nodes.back().y-node.y);
  output.nodes.push_back(node);output.has_sheave=true;output.sheave=sh;
  output.lowest_x=node.x;output.lowest_y=node.y;output.lowest_z=node.z;
  output.lowest_min_z=node.min_z;output.lowest_max_z=node.max_z;
  output.lowest_point_count=node.point_count;output.lowest_point_indices=node.point_indices;
  output.lowest_subcluster_id=node.subcluster_id;output.total_depth=output.root_z-node.z;
  return true;
}

void CranePipeline::updateSheaveTracking(CraneTopDownHookResult & result,
    const std::vector<CraneSubcluster> & subclusters,
    const std::vector<PointXYZIRT> & points, double stamp_sec,
    const std::vector<PointXYZIRT> * current_observation_points) {
  if(!config_.sheave.enabled){
    for(auto * t:{&left_td_track_,&right_td_track_}){
      t->sheave_valid=false;t->sheave_node=TrackedChainNodeState{};
      t->housing_memory=HousingGeometryMemory{};t->sheave_missed_frames=0;t->stable_sheave=CraneSheaveGeometry{};
    }
    return;
  }
  result.original_chains=result.chains;
  const auto & dual=config_.dual_roi_anchor;
  const int hold=std::max(1,config_.temporal_stabilization.node_max_hold_frames);
  for(int side=0;side<2;++side){
    auto & track=side==0?left_td_track_:right_td_track_;
    const auto side_begin=std::chrono::steady_clock::now();
    std::vector<int> proposal_ids;int geometry_rejections=0,wire_rejections=0;double detection_ms=0,matching_ms=0;
    auto & node=track.sheave_node;auto & mem=track.housing_memory;
    const int ci=side==0?result.left_chain_idx:result.right_chain_idx;
    const CraneHookChain * chain=ci>=0 && ci<int(result.original_chains.size())?&result.original_chains[ci]:nullptr;
    const double dt=stamp_sec-mem.last_frame_stamp;
    const bool duplicate=mem.last_frame_stamp>0 && dt==0;
    const int lifetime=std::max(hold+1,dual.anchor_memory_frames);
    if(mem.initialized && dt<0){mem=HousingGeometryMemory{};node=TrackedChainNodeState{};track.sheave_valid=false;}
    else if(mem.initialized && dt>(hold+1)*mem.frame_period+1e-6){
      if((mem.confirmed || mem.dormant) && dt<=lifetime*mem.frame_period){
        mem.missed_frames+=std::max(0,int(std::ceil(dt/mem.frame_period))-1);
        mem.dormant=true;mem.confirmed=false;mem.confirm_count=0;
        node=TrackedChainNodeState{};track.sheave_valid=false;
      }else {mem=HousingGeometryMemory{};node=TrackedChainNodeState{};track.sheave_valid=false;}
    }
    if(dt>0 && dt<0.5 && !mem.dormant)mem.frame_period=dt;
    mem.last_frame_stamp=stamp_sec;
    const tf2::Vector3 * origin=visibility_model_ptr_?&visibility_model_ptr_->pose().getOrigin():nullptr;
    CraneSheaveGeometry diagnostics;
    if(!chain)++diagnostics.rejection_counts[11];
    CraneSheaveGeometry observed;double best=std::numeric_limits<double>::infinity();
    if(chain && !duplicate){
      // Scan the full Raw once per side. All later covariance/context queries
      // use this local view; indices are mapped back to the original Raw frame.
      const auto & raw=current_observation_points?*current_observation_points:points;
      std::vector<PointXYZIRT> local;std::vector<int> global_indices;
      double min_x=1e9,max_x=-1e9,min_y=1e9,max_y=-1e9,min_z=1e9,max_z=-1e9;
      for(const auto & wire:chain->nodes){
        min_x=std::min(min_x,wire.x-.9);max_x=std::max(max_x,wire.x+.9);
        min_y=std::min(min_y,wire.y-.9);max_y=std::max(max_y,wire.y+.9);
        min_z=std::min(min_z,wire.z-1.30-dual.moving_roi_size_z);
        max_z=std::max(max_z,wire.z+2*dual.moving_roi_size_z);
      }
      for(size_t i=0;i<raw.size();++i){const auto&p=raw[i];
        if(p.x>=min_x && p.x<=max_x && p.y>=min_y && p.y<=max_y && p.z>=min_z && p.z<=max_z){local.push_back(p);global_indices.push_back(i);}
      }
      auto consider=[&](CraneSheaveGeometry g,const CraneHookChainNode&wire){
        for(size_t i=0;i<diagnostics.rejection_counts.size();++i)diagnostics.rejection_counts[i]+=g.rejection_counts[i];
        if(g.closest_rejected_distance>=0 && (diagnostics.closest_rejected_distance<0 || g.closest_rejected_distance<diagnostics.closest_rejected_distance)){
          diagnostics.closest_rejected_center=g.closest_rejected_center;diagnostics.closest_rejected_distance=g.closest_rejected_distance;diagnostics.closest_rejection_reason=g.closest_rejection_reason;
        }
        proposal_ids.insert(proposal_ids.end(),g.candidate_ids.begin(),g.candidate_ids.end());
        geometry_rejections+=g.geometry_rejections;wire_rejections+=g.wire_rejections;
        detection_ms+=g.detection_ms;matching_ms+=g.matching_ms;
        if(g.status!=CraneSheaveDetectionStatus::DIRECT_OBSERVED)return;
        if(dual.enabled && node.anchor_locked &&
            !node.fixed_roi.contains(g.outer_center_x,g.outer_center_y,g.outer_center_z) &&
            !node.moving_roi.contains(g.outer_center_x,g.outer_center_y,g.outer_center_z)){
          ++diagnostics.rejection_counts[10];
          const double distance=std::hypot(std::hypot(g.outer_center_x-mem.housing_center_x,g.outer_center_y-mem.housing_center_y),g.outer_center_z-mem.housing_center_z);
          if(diagnostics.closest_rejected_distance<0 || distance<diagnostics.closest_rejected_distance){
            diagnostics.closest_rejection_reason="DUAL_ROI";
            diagnostics.closest_rejected_center={g.outer_center_x,g.outer_center_y,g.outer_center_z};diagnostics.closest_rejected_distance=distance;
          }return;}
        const double cost=mem.initialized?g.match_score:wire.z-g.outer_center_z;
        if(cost<best){
          for(int&i:g.point_indices)i=global_indices.at(i);
          g.points_from_current_frame=current_observation_points!=nullptr;
          observed=std::move(g);best=cost;
        }
      };
      // Reuse production broad-surface proposals with an independent memory
      // search reference. A rectangular Raw crop is only a search window: it
      // must not become a monolithic body containing the adjacent thin wires.
      if(mem.initialized && (mem.confirmed || mem.dormant)){
        CraneHookChainNode search_reference;
        search_reference.x=mem.housing_center_x;search_reference.y=mem.housing_center_y;
        search_reference.z=mem.housing_center_z+mem.height*.5+dual.moving_roi_size_z*.5;
        WireBundleGeometry bundle;
        consider(detectSheaveCandidate(search_reference,subclusters,points,bundle,config_.sheave,dual,nullptr,&mem,&local,origin),search_reference);
      }
      if(observed.status!=CraneSheaveDetectionStatus::DIRECT_OBSERVED){
        for(const auto & wire:chain->nodes){
          if(wire.is_held || isBeamLikeNode(wire,dual))continue;
          auto bundle=resolveWireBundleGeometry(wire,subclusters,points,nullptr,config_.virtual_rope,dual);
          consider(detectSheaveCandidate(wire,subclusters,points,bundle,config_.sheave,dual,nullptr,&mem,&local,origin),wire);
        }
      }
      if(observed.status!=CraneSheaveDetectionStatus::DIRECT_OBSERVED && config_.sheave.reference_matching_enabled){
        // Current production subclusters propose XY; no stored absolute position
        // or user seed enters this search. Reference Z profiles separate the
        // broad surface from adjacent wires/Hook when Gaussian layers merge.
        std::vector<const CraneSubcluster*> seeds;
        for(const auto & sc:subclusters){
          if(sc.point_indices.size()<config_.sheave.min_arc_points || isBeamLikeSubcluster(sc,dual) ||
              std::max(sc.max_x-sc.min_x,sc.max_y-sc.min_y)<.12 || std::min(sc.max_x-sc.min_x,sc.max_y-sc.min_y)<.06 ||
              std::hypot(sc.centroid_x-chain->root_x,sc.centroid_y-chain->root_y)>.45 || sc.centroid_z>=chain->root_z)continue;
          seeds.push_back(&sc);
        }
        std::sort(seeds.begin(),seeds.end(),[](auto a,auto b){return a->point_indices.size()>b->point_indices.size();});
        if(seeds.size()>8)seeds.resize(8);
        // Current-frame XY membership depends on the seed, not the Reference.
        // Reuse indices only; preserve Raw order and every per-Reference Z gate.
        std::array<std::vector<size_t>,8> seed_xy_indices;
        std::array<bool,8> seed_xy_ready{};
        const double bin=.05; // Immutable reference schema: 5cm Z profile, independent of Gaussian preprocessing.
        for(size_t ri=0;ri<std::min(size_t{8},config_.sheave.references.size());++ri){
          const auto & ref=config_.sheave.references[ri];if(!ref.trusted())continue;
          // A reviewed foreign appearance must not silently relabel an already
          // tracked PROBABLE/unverified object. Cold start and same-family
          // recovery are allowed; cross-family takeover is not evidence.
          if(mem.initialized && mem.reference_family_id!=ref.family_id)continue;
          auto assisted_cfg=config_.sheave;assisted_cfg.references={ref};assisted_cfg.reference_wire_mode_search=true;
          for(const auto & negative:config_.sheave.references)
            if(negative.label=="NEGATIVE")assisted_cfg.references.push_back(negative);
          for(size_t si=0;si<seeds.size();++si){
            const auto * seed=seeds[si];
            std::map<int,int> profile;
            const double pad=config_.sheave.surface_gap_m;
            auto inside_xy=[&](const PointXYZIRT & p){return p.x>=seed->min_x-pad && p.x<=seed->max_x+pad && p.y>=seed->min_y-pad && p.y<=seed->max_y+pad;};
            auto & xy_indices=seed_xy_indices[si];
            if(!seed_xy_ready[si]){
              for(size_t i=0;i<local.size();++i)if(inside_xy(local[i]))xy_indices.push_back(i);
              seed_xy_ready[si]=true;
            }
            for(size_t i:xy_indices){const auto & p=local[i];
              if(p.z>=seed->min_z-ref.dimensions[2] && p.z<=seed->max_z+ref.dimensions[2])++profile[int(std::floor(p.z/bin))];
            }
            if(profile.empty())continue;
            auto peak=std::max_element(profile.begin(),profile.end(),[](auto a,auto b){return a.second<b.second;});
            if(peak->second<int(config_.sheave.min_arc_points))continue;
            const double z=(peak->first+.5)*bin-ref.peak_z_offset;
            CraneSubcluster body;body.subcluster_id=-100-int(ri*8+si);body.layer_id=-2;
            body.min_x=body.min_y=body.min_z=1e9;body.max_x=body.max_y=body.max_z=-1e9;
            for(size_t i:xy_indices){const auto & p=local[i];
              if(p.z<z+ref.min_z_offset-bin*.5 || p.z>z+ref.max_z_offset+bin*.5)continue;
              body.point_indices.push_back(i);body.centroid_x+=p.x;body.centroid_y+=p.y;body.centroid_z+=p.z;
              body.min_x=std::min(body.min_x,double(p.x));body.max_x=std::max(body.max_x,double(p.x));
              body.min_y=std::min(body.min_y,double(p.y));body.max_y=std::max(body.max_y,double(p.y));
              body.min_z=std::min(body.min_z,double(p.z));body.max_z=std::max(body.max_z,double(p.z));
            }
            if(body.point_indices.size()<config_.sheave.min_arc_points)continue;
            body.centroid_x/=body.point_indices.size();body.centroid_y/=body.point_indices.size();body.centroid_z/=body.point_indices.size();
            // A reference window is a surface proposal, not a fabricated wire
            // node. Attach it to an actual current Chain upper extent. A raised
            // Housing may overlap the Root layer; Root membership alone neither
            // accepts nor rejects it. Reviewed shape and Raw wire gates below
            // still apply before any observation can initialize a track.
            const CraneHookChainNode * connected_wire=nullptr;
            for(const auto & wire:chain->nodes){
              if(wire.is_held || (!wire.is_root && isBeamLikeNode(wire,dual)) ||
                  wire.max_z<body.max_z || wire.max_z-body.centroid_z>1.3 ||
                  std::hypot(wire.x-body.centroid_x,wire.y-body.centroid_y)>.45)continue;
              if(!connected_wire || wire.max_z<connected_wire->max_z)connected_wire=&wire;
            }
            if(!connected_wire)continue;
            CraneHookChainNode search=*connected_wire;
            // Node centers can lie below the shell top after Gaussian merging
            // or smoothing. Use its actual current Raw upper extent, not an
            // invented point above the candidate or a relaxed distance gate.
            search.z=connected_wire->max_z;
            WireBundleGeometry bundle;
            auto g=detectSheaveCandidate(search,{body},local,bundle,assisted_cfg,dual,nullptr,&mem,&local,origin);
            if(g.track_memory_matched || (g.reference_matched && g.reference_trusted && g.reference_family_id==ref.family_id)){if(!g.track_memory_matched)g.reference_assisted=true;}
            else {g.status=CraneSheaveDetectionStatus::NONE;++g.geometry_rejections;}
            consider(std::move(g),search);
          }
        }
      }
      // A confirmed template may propose a current-RAW surface inside its local
      // search window even when the global Gaussian Z layer splits differently.
      // It still passes the same shape, position and current-wire hard gates.
      if(observed.status!=CraneSheaveDetectionStatus::DIRECT_OBSERVED && mem.initialized && (mem.confirmed || mem.dormant)){
        CraneSubcluster guided;guided.subcluster_id=-2;guided.layer_id=-2;
        guided.min_x=guided.min_y=guided.min_z=1e9;guided.max_x=guided.max_y=guided.max_z=-1e9;
        for(size_t i=0;i<local.size();++i){const auto&p=local[i];
          if(std::abs(p.x-mem.housing_center_x)>mem.span_x*.5+config_.sheave.surface_gap_m ||
              std::abs(p.y-mem.housing_center_y)>mem.span_y*.5+config_.sheave.surface_gap_m ||
              std::abs(p.z-mem.housing_center_z)>mem.height*.5+config_.sheave.surface_gap_m)continue;
          guided.point_indices.push_back(i);guided.centroid_x+=p.x;guided.centroid_y+=p.y;guided.centroid_z+=p.z;
          guided.min_x=std::min(guided.min_x,double(p.x));guided.max_x=std::max(guided.max_x,double(p.x));
          guided.min_y=std::min(guided.min_y,double(p.y));guided.max_y=std::max(guided.max_y,double(p.y));
          guided.min_z=std::min(guided.min_z,double(p.z));guided.max_z=std::max(guided.max_z,double(p.z));
        }
        if(!guided.point_indices.empty()){
          guided.centroid_x/=guided.point_indices.size();guided.centroid_y/=guided.point_indices.size();guided.centroid_z/=guided.point_indices.size();
          // This is a search reference, never an observed wire/axis. Current
          // raw strand evidence remains mandatory. A held intermediate Wire
          // layer must not disable the independent Housing detector.
          CraneHookChainNode search_reference;
          search_reference.x=mem.housing_center_x;search_reference.y=mem.housing_center_y;
          search_reference.z=mem.housing_center_z+mem.height*.5+dual.moving_roi_size_z*.5;
          WireBundleGeometry bundle;
          consider(detectSheaveCandidate(search_reference,{guided},local,bundle,config_.sheave,dual,nullptr,&mem,&local,origin),search_reference);
        }
      }
    }
    const bool direct=observed.status==CraneSheaveDetectionStatus::DIRECT_OBSERVED && observed.wire_supported;
    if(direct){
      // Freeze the confirmed shape template: repeated matching cannot drift into
      // another object. Position/last-direct-time alone follow matching observations.
      if(!mem.initialized){
        mem.track_id=(side==0?0:1000000)+track.next_housing_track_id++;
        mem.reference_family_id=observed.reference_trusted?observed.reference_family_id:-1;
        mem.initialized=true;mem.length=observed.housing_length;mem.width=observed.housing_width;mem.height=observed.housing_height;
        mem.span_x=observed.max_x-observed.min_x;mem.span_y=observed.max_y-observed.min_y;
        mem.aspect_ratio=mem.length/std::max(mem.width,1e-6);mem.point_count=observed.point_count;
        mem.shape_eigenvalues=observed.shape_eigenvalues;mem.horizontal_distribution=observed.horizontal_distribution;mem.z_distribution=observed.z_distribution;mem.wire_offset_xy=observed.wire_offset_xy;
        mem.wire_dir_x=observed.wire_dir_x;mem.wire_dir_y=observed.wire_dir_y;mem.wire_spacing=observed.wire_spacing;mem.wire_mid_s=observed.wire_mid_s;mem.wire_mid_t=observed.wire_mid_t;
      }
      ++mem.confirm_count;mem.missed_frames=0;
      const bool returning_confirmed=config_.sheave.track_continuity_enabled && mem.ever_confirmed &&
          (observed.reference_trusted || observed.track_memory_matched);
      mem.confirmed=mem.confirmed || returning_confirmed || mem.confirm_count>=std::max(2,config_.sheave.confirm_frames);
      mem.ever_confirmed=mem.ever_confirmed || mem.confirmed;
      if(config_.sheave.track_continuity_enabled && mem.confirmed && observed.reference_matched && observed.reference_trusted && !observed.track_memory_matched){
        mem.recent_reference_timeout_sec=(hold+1)*mem.frame_period;
        mem.recent_reference_stamp=stamp_sec;mem.recent_reference_id=observed.reference_id;
        mem.recent_reference_family=observed.reference_family_id;
        mem.recent_dimensions={observed.housing_length,observed.housing_width,observed.housing_height};
        mem.recent_spectrum=observed.shape_eigenvalues;mem.recent_horizontal=observed.horizontal_distribution;
        mem.recent_center={observed.outer_center_x,observed.outer_center_y,observed.outer_center_z};
        mem.recent_point_count=observed.point_count;mem.recent_range=observed.sensor_range;
      }
      if(mem.confirmed)mem.dormant=false;
      mem.housing_center_x=observed.outer_center_x;mem.housing_center_y=observed.outer_center_y;mem.housing_center_z=observed.outer_center_z;
      mem.last_direct_stamp=stamp_sec;
      const double x=observed.outer_center_x,y=observed.outer_center_y,z=observed.outer_center_z;
      const double alpha=std::clamp(config_.temporal_stabilization.stabilization_alpha,0.05,0.90);
      if(node.confirmed_hits==0){node.moving_x=x;node.moving_y=y;node.moving_z=z;}
      else{node.moving_x+=(x-node.moving_x)*alpha;node.moving_y+=(y-node.moving_y)*alpha;node.moving_z+=(z-node.moving_z)*alpha;}
      node.raw_obs_x=x;node.raw_obs_y=y;node.raw_obs_z=z;
      ++node.confirmed_hits;node.missed_frames=0;node.is_held=false;node.is_sheave=true;node.is_relocated_this_frame=false;
      if(dual.enabled && mem.confirmed && node.confirmed_hits>=std::max(dual.confirm_frames,dual.min_direct_evidence_frames)){
        if(!node.anchor_locked){
          node.anchor_locked=true;node.anchor_id=mem.track_id; // Independent of wire anchor numbering.
          node.anchor_x=node.initial_anchor_x=node.moving_x;node.anchor_y=node.initial_anchor_y=node.moving_y;node.anchor_z=node.initial_anchor_z=node.moving_z;node.initial_anchor_valid=true;
        }else{
          double dx=node.moving_x-node.anchor_x,dy=node.moving_y-node.anchor_y,dz=node.moving_z-node.anchor_z;
          bool moved=std::hypot(dx,dy)>dual.relocate_thresh_xy || std::abs(dz)>dual.relocate_thresh_z;
          // Include Z in direction consistency so real lowering/raising can relocate.
          if(moved && dx*node.prev_move_dx+dy*node.prev_move_dy+dz*node.prev_move_dz>=0){
            if(++node.relocate_confirm_count>=std::max(1,dual.relocate_confirm_frames)){
              node.anchor_x=node.moving_x;node.anchor_y=node.moving_y;node.anchor_z=node.moving_z;
              ++node.relocate_count;node.is_relocated_this_frame=true;node.relocate_confirm_count=0;
            }
          }else node.relocate_confirm_count=0;
          node.prev_move_dx=dx;node.prev_move_dy=dy;node.prev_move_dz=dz;
        }
      }
      node.disp_from_anchor_xy=std::hypot(node.moving_x-node.anchor_x,node.moving_y-node.anchor_y);
      node.disp_from_anchor_z=std::abs(node.moving_z-node.anchor_z);
      const bool deadband=node.anchor_locked && node.disp_from_anchor_xy<=dual.noise_deadband_xy && node.disp_from_anchor_z<=dual.noise_deadband_z;
      node.x=deadband?node.anchor_x:node.moving_x;node.y=deadband?node.anchor_y:node.moving_y;node.z=deadband?node.anchor_z:node.moving_z;
      node.fixed_roi=makeBoxRoi(node.anchor_locked?node.anchor_x:node.x,node.anchor_locked?node.anchor_y:node.y,node.anchor_locked?node.anchor_z:node.z,dual.fixed_roi_size_xy,dual.fixed_roi_size_z);
      node.moving_roi=makeBoxRoi(node.moving_x,node.moving_y,node.moving_z,dual.moving_roi_size_xy,dual.moving_roi_size_z);
      node.sheave=observed;
      if(config_.sheave.track_continuity_enabled && !mem.confirmed){
        node.sheave.status=CraneSheaveDetectionStatus::NONE;node.sheave.temporal_pending=true;
      }
      node.point_count=observed.point_count;node.point_indices=observed.point_indices;
      node.min_x=observed.min_x;node.max_x=observed.max_x;node.min_y=observed.min_y;node.max_y=observed.max_y;node.min_z=observed.min_z;node.max_z=observed.max_z;
      node.subcluster_id=observed.subcluster_id;node.layer_id=observed.layer_id;
      track.sheave_valid=mem.confirmed;track.sheave_missed_frames=0;
    }else{
      ++mem.missed_frames;mem.confirm_count=0;node.is_relocated_this_frame=false;node.relocate_confirm_count=0;
      if(mem.confirmed && mem.missed_frames<=hold){
        node.is_held=true;node.missed_frames=mem.missed_frames;node.sheave.status=CraneSheaveDetectionStatus::HELD;
        node.point_count=0;node.point_indices.clear();node.sheave.point_indices.clear();node.sheave.point_count=0;
        node.sheave.geometry_matched=false;node.sheave.wire_supported=false;node.sheave.axis_status=CraneSheaveDetectionStatus::NONE;
        node.sheave.reference_matched=false;node.sheave.reference_assisted=false;node.sheave.reference_score=-1;
        node.sheave.sensor_range=-1;node.sheave.point_density=0;node.sheave.shape_reliability=0;
      }else if((mem.confirmed || mem.dormant) && mem.missed_frames<=lifetime){
        // Track/anchor/Held output expire at the existing Hold limit. A bounded,
        // frozen template remains for rejecting a different replacement object.
        mem.dormant=true;mem.confirmed=false;node=TrackedChainNodeState{};track.sheave_valid=false;
      }else{
        mem=HousingGeometryMemory{};mem.last_frame_stamp=stamp_sec;
        node=TrackedChainNodeState{};track.sheave_valid=false;
      }
      track.sheave_missed_frames=mem.missed_frames;
    }
    std::sort(proposal_ids.begin(),proposal_ids.end());proposal_ids.erase(std::unique(proposal_ids.begin(),proposal_ids.end()),proposal_ids.end());
    node.sheave.rejection_counts=diagnostics.rejection_counts;
    node.sheave.closest_rejected_center=diagnostics.closest_rejected_center;
    node.sheave.closest_rejected_distance=diagnostics.closest_rejected_distance;
    node.sheave.closest_rejection_reason=diagnostics.closest_rejection_reason;
    node.sheave.candidate_ids=proposal_ids;node.sheave.candidate_count=proposal_ids.size();
    node.sheave.geometry_rejections=geometry_rejections;node.sheave.wire_rejections=wire_rejections;
    node.sheave.detection_ms=detection_ms;node.sheave.matching_ms=matching_ms;
    node.sheave.tracking_ms=std::max(0.0,std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-side_begin).count()-detection_ms-matching_ms);
    track.stable_sheave=node.sheave;
    CraneHookChainNode debug;
    debug.sheave=node.sheave;debug.x=node.x;debug.y=node.y;debug.z=node.z;debug.is_held=node.is_held;
    debug.point_count=node.point_count;debug.point_indices=node.point_indices;debug.subcluster_id=node.subcluster_id;debug.layer_id=node.layer_id;
    debug.min_x=node.min_x;debug.max_x=node.max_x;debug.min_y=node.min_y;debug.max_y=node.max_y;debug.min_z=node.min_z;debug.max_z=node.max_z;
    debug.raw_obs_x=node.raw_obs_x;debug.raw_obs_y=node.raw_obs_y;debug.raw_obs_z=node.raw_obs_z;
    debug.anchor_locked=node.anchor_locked;debug.anchor_id=node.anchor_id;
    debug.anchor_x=node.anchor_x;debug.anchor_y=node.anchor_y;debug.anchor_z=node.anchor_z;
    debug.moving_x=node.moving_x;debug.moving_y=node.moving_y;debug.moving_z=node.moving_z;
    debug.fixed_roi=node.fixed_roi;debug.moving_roi=node.moving_roi;debug.relocate_count=node.relocate_count;debug.is_relocated_this_frame=node.is_relocated_this_frame;
    if(side==0){result.left_housing=debug;result.left_housing_memory=mem;}else{result.right_housing=debug;result.right_housing_memory=mem;}
    if(chain && direct && mem.confirmed && config_.sheave.terminate_chain_at_sheave)appendSheaveTerminal(result.chains[ci],debug);
  }
}

void CranePipeline::stabilizeTopDownStructure(
    CraneTopDownHookResult & td_res,
    const std::vector<CraneSubcluster> & subclusters,
    const std::vector<PointXYZIRT> & points,
    bool crane_moving,
    double /*root_dx*/,
    double /*root_dy*/) {
  const auto & cfg = config_.temporal_stabilization;
  if (!cfg.temporal_stabilization_enabled) {
    return;
  }

  const auto & dual_cfg = config_.dual_roi_anchor;
  const auto & vr_cfg = config_.virtual_rope;

  const double alpha = std::clamp(cfg.stabilization_alpha, 0.05, 0.90);
  const double alpha_yaw = std::clamp(cfg.yaw_alpha, 0.05, 0.90);
  const double jump_len = cfg.jump_tolerance_length;
  const double jump_wid = cfg.jump_tolerance_width;
  const int jump_confirm = std::max(1, cfg.jump_confirm_frames);
  const int node_confirm = std::max(1, cfg.node_confirm_frames);
  const int node_max_hold = std::max(1, cfg.node_max_hold_frames);
  const int swap_confirm = std::max(1, cfg.largest_swap_confirm_frames);
  const double match_dist = cfg.largest_match_dist;

  for (int side = 0; side < 2; ++side) {
    TrackedTopDownStructure & track = (side == 0) ? left_td_track_ : right_td_track_;
    track.side = (side == 0) ? "LEFT" : "RIGHT";
    const int chain_idx = (side == 0) ? td_res.left_chain_idx : td_res.right_chain_idx;
    CraneHookChain * raw_chain = (chain_idx >= 0 && chain_idx < static_cast<int>(td_res.chains.size())) ?
        &td_res.chains[chain_idx] : nullptr;

    const double cur_root_x = raw_chain ? raw_chain->root_x : ((side == 0) ? left_root_node_.smoothed_x : right_root_node_.smoothed_x);
    const double cur_root_y = raw_chain ? raw_chain->root_y : ((side == 0) ? left_root_node_.smoothed_y : right_root_node_.smoothed_y);
    const double d_rx = track.prev_root_valid ? (cur_root_x - track.prev_root_x) : 0.0;
    const double d_ry = track.prev_root_valid ? (cur_root_y - track.prev_root_y) : 0.0;
    track.prev_root_x = cur_root_x;
    track.prev_root_y = cur_root_y;
    track.prev_root_valid = (raw_chain != nullptr);

    // Crane motion displacement compensation for held nodes
    if (crane_moving && track.prev_root_valid) {
      for (auto & sn : track.stable_nodes) {
        sn.x += d_rx; sn.y += d_ry;
        sn.min_x += d_rx; sn.max_x += d_rx;
        sn.min_y += d_ry; sn.max_y += d_ry;
        if (dual_cfg.enabled && sn.anchor_locked) {
          sn.anchor_x += d_rx; sn.anchor_y += d_ry;
          sn.moving_x += d_rx; sn.moving_y += d_ry;
        }
      }
      if (track.largest_valid) {
        track.stable_largest.centroid_x += d_rx; track.stable_largest.centroid_y += d_ry;
        track.stable_largest.ref_x += d_rx; track.stable_largest.ref_y += d_ry;
        track.stable_largest.min_x += d_rx; track.stable_largest.max_x += d_rx;
        track.stable_largest.min_y += d_ry; track.stable_largest.max_y += d_ry;
      }
    }

    if (!raw_chain || raw_chain->nodes.empty()) {
      if (track.initialized && !track.stable_nodes.empty()) {
        bool any_held = false;
        for (auto & sn : track.stable_nodes) {
          const int effective_max_hold = (dual_cfg.enabled && sn.is_beam_intruded) ?
              dual_cfg.intrusion_hold_frames : node_max_hold;
          if (++sn.missed_frames <= effective_max_hold) {
            sn.is_held = true;
            sn.is_estimated_gap = true;
            any_held = true;
          }
        }
        if (any_held) {
          CraneHookChain held_chain;
          held_chain.chain_id = (side == 0) ? 100 : 200;
          held_chain.side = track.side;
          held_chain.state = CraneHookConnectionState::PERSISTENT;
          held_chain.root_x = cur_root_x; held_chain.root_y = cur_root_y;
          held_chain.root_z = (side == 0) ? left_root_node_.smoothed_z : right_root_node_.smoothed_z;
          held_chain.is_held = true;
          held_chain.held_frames = track.stable_nodes.front().missed_frames;

          CraneHookChainNode rnode;
          rnode.is_root = true;
          rnode.x = held_chain.root_x; rnode.y = held_chain.root_y; rnode.z = held_chain.root_z;
          held_chain.nodes.push_back(rnode);
          for (const auto & sn : track.stable_nodes) {
            const int effective_max_hold = (dual_cfg.enabled && sn.is_beam_intruded) ?
                dual_cfg.intrusion_hold_frames : node_max_hold;
            if (sn.missed_frames <= effective_max_hold) {
              CraneHookChainNode cn;
              cn.layer_id = sn.layer_id;
              cn.x = sn.x; cn.y = sn.y; cn.z = sn.z;
              cn.min_x = sn.min_x; cn.max_x = sn.max_x;
              cn.min_y = sn.min_y; cn.max_y = sn.max_y;
              cn.min_z = sn.min_z; cn.max_z = sn.max_z;
              cn.point_count = sn.point_count;
              cn.subcluster_id = sn.subcluster_id;
              cn.parent_subcluster_id = sn.parent_subcluster_id;
              cn.gap_z_to_parent = sn.gap_z_to_parent;
              cn.dxy_to_parent = sn.dxy_to_parent;
              cn.is_estimated_gap = true;
              cn.is_held = true;
              cn.held_frames = sn.missed_frames;
              cn.is_connection_stable = sn.is_connection_stable && (sn.missed_frames <= config_.connection_stabilization.missing_hold_frames);
              cn.is_spatial_promoted = sn.is_spatial_promoted;

              if (dual_cfg.enabled && sn.anchor_locked) {
                cn.anchor_locked = sn.anchor_locked;
                cn.anchor_id = sn.anchor_id;
                cn.initial_anchor_x = sn.initial_anchor_x;
                cn.initial_anchor_y = sn.initial_anchor_y;
                cn.initial_anchor_z = sn.initial_anchor_z;
                cn.anchor_x = sn.anchor_x;
                cn.anchor_y = sn.anchor_y;
                cn.anchor_z = sn.anchor_z;
                cn.moving_x = sn.moving_x;
                cn.moving_y = sn.moving_y;
                cn.moving_z = sn.moving_z;
                cn.raw_obs_x = sn.raw_obs_x;
                cn.raw_obs_y = sn.raw_obs_y;
                cn.raw_obs_z = sn.raw_obs_z;
                cn.disp_from_anchor_xy = sn.disp_from_anchor_xy;
                cn.disp_from_anchor_z = sn.disp_from_anchor_z;
                cn.relocate_count = sn.relocate_count;
                cn.is_relocated_this_frame = false;
                cn.is_beam_intruded = sn.is_beam_intruded;
                cn.fixed_roi = sn.fixed_roi;
                cn.moving_roi = sn.moving_roi;
                ++held_chain.locked_anchor_count;
                if (cn.is_beam_intruded) ++held_chain.beam_intruded_count;
              }

              if (vr_cfg.enabled) {
                cn.has_virtual_rope = (sn.wire_bundle.mode != CraneWireBundleMode::NONE);
                cn.wire_bundle = sn.wire_bundle;
              }

              if (cn.is_connection_stable) ++held_chain.stable_edge_count;
              held_chain.nodes.push_back(cn);
            }
          }
          if (held_chain.nodes.size() > 1) {
            const auto & term = held_chain.nodes.back();
            held_chain.lowest_x = term.x; held_chain.lowest_y = term.y; held_chain.lowest_z = term.z;
            held_chain.lowest_min_z = term.min_z; held_chain.lowest_max_z = term.max_z;
            held_chain.lowest_point_count = term.point_count;
            held_chain.total_depth = held_chain.root_z - held_chain.lowest_z;

            if (track.largest_valid && ++track.largest_missed_frames <= cfg.max_hold_missed_frames) {
              track.stable_largest.is_held = true;
              track.stable_largest.held_frames = track.largest_missed_frames;
              held_chain.largest_cluster = track.stable_largest;
              if (side == 0) td_res.left_largest_cluster = track.stable_largest;
              else td_res.right_largest_cluster = track.stable_largest;
            }

            const int new_cidx = static_cast<int>(td_res.chains.size());
            td_res.chains.push_back(held_chain);
            if (side == 0) { td_res.left_chain_idx = new_cidx; td_res.left_detected = true; }
            else { td_res.right_chain_idx = new_cidx; td_res.right_detected = true; }
          }
        } else {
          track.stable_nodes.clear();
          track.largest_valid = false;
        }
      }
      continue;
    }

    // Step A: Stabilize Chain Nodes with Dual-ROI Anchoring & Motion Monitoring
    const auto & raw_nodes = raw_chain->nodes;
    std::vector<bool> raw_matched(raw_nodes.size(), false);

    // Pre-check Cargo Beam Intrusion Guard
    if (dual_cfg.enabled && dual_cfg.beam_intrusion_guard_enabled) {
      for (auto & sn : track.stable_nodes) {
        if (!sn.anchor_locked) continue;
        bool beam_intruded = false;
        for (size_t ri = 1; ri < raw_nodes.size(); ++ri) {
          const auto & rn = raw_nodes[ri];
          if (!isBeamLikeNode(rn, dual_cfg)) continue;
          const double dxy = std::hypot(rn.x - sn.anchor_x, rn.y - sn.anchor_y);
          const double dz = std::abs(rn.z - sn.anchor_z);
          if (dxy <= 0.40 && dz <= 0.35) {
            beam_intruded = true;
            break;
          }
        }
        if (beam_intruded) {
          sn.is_beam_intruded = true;
          sn.intrusion_missed_frames = 0;
        } else if (sn.is_beam_intruded) {
          if (++sn.intrusion_missed_frames > 2) {
            sn.is_beam_intruded = false;
            sn.intrusion_missed_frames = 0;
          }
        }
      }
    }

    for (size_t si = 0; si < track.stable_nodes.size(); ++si) {
      auto & sn = track.stable_nodes[si];
      int best_ri = -1;
      double best_cost = std::numeric_limits<double>::infinity();

      for (size_t ri = 1; ri < raw_nodes.size(); ++ri) {
        if (raw_matched[ri]) continue;
        const auto & rn = raw_nodes[ri];
        if (rn.is_sheave) continue; // Sheave is tracked separately via sheave_node

        // Intrusion Guard: Beam-like clusters cannot match wire anchors
        if (dual_cfg.enabled && dual_cfg.beam_intrusion_guard_enabled && isBeamLikeNode(rn, dual_cfg)) {
          continue;
        }

        double ref_x = sn.x, ref_y = sn.y, ref_z = sn.z;
        if (dual_cfg.enabled && sn.anchor_locked) {
          ref_x = sn.anchor_x;
          ref_y = sn.anchor_y;
          ref_z = sn.anchor_z;
        }

        const double dz = std::abs(rn.z - ref_z);
        if (dz > 0.35) continue;
        const double dxy = std::hypot(rn.x - ref_x, rn.y - ref_y);
        if (dxy > 0.45) continue;
        const double cost = dxy + 0.5 * dz;
        if (cost < best_cost) {
          best_cost = cost;
          best_ri = static_cast<int>(ri);
        }
      }

      if (best_ri >= 0) {
        raw_matched[best_ri] = true;
        const auto & rn = raw_nodes[best_ri];

        double effective_obs_x = rn.x;
        double effective_obs_y = rn.y;
        double effective_obs_z = rn.z;

        if (vr_cfg.enabled) {
          WireBundleGeometry vr_geom = resolveWireBundleGeometry(rn, subclusters, points, &sn, vr_cfg, dual_cfg);
          sn.wire_bundle = vr_geom;
          if (vr_geom.mode == CraneWireBundleMode::BOTH_STRANDS) {
            sn.remembered_bundle = vr_geom;
            sn.wire_bundle_memory_age = 0;
          } else {
            sn.wire_bundle_memory_age++;
          }
          effective_obs_x = vr_geom.center_x;
          effective_obs_y = vr_geom.center_y;
          effective_obs_z = vr_geom.center_z;
        }

        sn.raw_obs_x = effective_obs_x;
        sn.raw_obs_y = effective_obs_y;
        sn.raw_obs_z = effective_obs_z;

        // Apply Z-axis Temporal Median Filter (Window = 3) to tracked Z observation
        const auto & z_med_cfg = config_.z_temporal_median;
        sn.z_history.push_back(effective_obs_z);
        while (static_cast<int>(sn.z_history.size()) > std::max(1, z_med_cfg.window_frames)) {
          sn.z_history.pop_front();
        }
        double effective_rn_z = effective_obs_z;
        if (z_med_cfg.enabled && !sn.z_history.empty()) {
          std::vector<double> sorted_z(sn.z_history.begin(), sn.z_history.end());
          std::sort(sorted_z.begin(), sorted_z.end());
          effective_rn_z = sorted_z[sorted_z.size() / 2];
        }

        // Z-layer split absorption: absorb any nearby split fragment inside this anchor's Fixed ROI
        if (dual_cfg.enabled && sn.anchor_locked) {
          for (size_t ri2 = 1; ri2 < raw_nodes.size(); ++ri2) {
            if (raw_matched[ri2]) continue;
            const auto & rn2 = raw_nodes[ri2];
            if (rn2.is_sheave) continue;
            if (isBeamLikeNode(rn2, dual_cfg)) continue;
            const double dxy2 = std::hypot(rn2.x - sn.anchor_x, rn2.y - sn.anchor_y);
            const double dz2 = std::abs(rn2.z - sn.anchor_z);
            if (dxy2 <= 0.30 && dz2 <= 0.25) {
              raw_matched[ri2] = true;
              sn.point_count += rn2.point_count;
              sn.point_indices.insert(sn.point_indices.end(), rn2.point_indices.begin(), rn2.point_indices.end());
            }
          }
        }

        if (dual_cfg.enabled && sn.anchor_locked) {
          sn.moving_x = (1.0 - alpha) * sn.moving_x + alpha * effective_obs_x;
          sn.moving_y = (1.0 - alpha) * sn.moving_y + alpha * effective_obs_y;
          sn.moving_z = (1.0 - alpha) * sn.moving_z + alpha * effective_rn_z;
          sn.disp_from_anchor_xy = std::hypot(sn.moving_x - sn.anchor_x, sn.moving_y - sn.anchor_y);
          sn.disp_from_anchor_z = std::abs(sn.moving_z - sn.anchor_z);

          const bool within_deadband_xy = (sn.disp_from_anchor_xy <= dual_cfg.noise_deadband_xy);
          const bool within_deadband_z = (sn.disp_from_anchor_z <= dual_cfg.noise_deadband_z);

          if (within_deadband_xy && within_deadband_z) {
            // Stationary: lock coordinates firmly to anchor coordinates to eliminate sensor jitter
            sn.x = sn.anchor_x;
            sn.y = sn.anchor_y;
            sn.z = sn.anchor_z;
            sn.relocate_confirm_count = 0;
            sn.is_relocated_this_frame = false;
          } else {
            // Outside deadband: follow moving center
            sn.x = sn.moving_x;
            sn.y = sn.moving_y;
            sn.z = sn.moving_z;
          }
        } else {
          sn.x = (1.0 - alpha) * sn.x + alpha * effective_obs_x;
          sn.y = (1.0 - alpha) * sn.y + alpha * effective_obs_y;
          sn.z = (1.0 - alpha) * sn.z + alpha * effective_rn_z;
          sn.moving_x = sn.x;
          sn.moving_y = sn.y;
          sn.moving_z = sn.z;
        }

        sn.min_x = rn.min_x; sn.max_x = rn.max_x;
        sn.min_y = rn.min_y; sn.max_y = rn.max_y;
        sn.min_z = rn.min_z; sn.max_z = rn.max_z;
        sn.layer_id = rn.layer_id;
        sn.subcluster_id = rn.subcluster_id;
        sn.point_count = std::max(sn.point_count, rn.point_count);
        sn.point_indices = rn.point_indices;
        sn.parent_subcluster_id = rn.parent_subcluster_id;
        sn.gap_z_to_parent = rn.gap_z_to_parent;
        sn.dxy_to_parent = rn.dxy_to_parent;
        sn.slope_angle_deg = rn.slope_angle_deg;
        sn.turn_angle_deg = rn.turn_angle_deg;
        sn.is_estimated_gap = rn.is_estimated_gap;
        sn.confirmed_hits++;
        sn.missed_frames = 0;
        sn.is_held = false;

        // Connection stabilization update
        const auto & cs_cfg = config_.connection_stabilization;
        if (cs_cfg.enabled) {
          sn.consecutive_missing = 0;
          const uint8_t edge_type = rn.is_estimated_gap ? 2 : 1;
          sn.edge_type_history.push_back(edge_type);
          while (static_cast<int>(sn.edge_type_history.size()) > cs_cfg.history_frames) {
            sn.edge_type_history.pop_front();
          }

          int d_cnt = 0, trans = 0;
          int last_t = 0;
          for (uint8_t t : sn.edge_type_history) {
            if (t == 1 || t == 2) {
              if (t == 1) d_cnt++;
              if (last_t != 0 && t != last_t) trans++;
              last_t = t;
            }
          }

          if (!sn.is_connection_stable) {
            if (d_cnt >= cs_cfg.min_direct_frames && trans >= cs_cfg.min_transition_count) {
              sn.is_connection_stable = true;
              sn.is_spatial_promoted = false;
            }
          } else {
            if (d_cnt == 0 && static_cast<int>(sn.edge_type_history.size()) >= cs_cfg.history_frames) {
              sn.is_connection_stable = false;
              sn.is_spatial_promoted = false;
            }
          }
        }

        // Dual-ROI Lock & Relocation Logic
        if (dual_cfg.enabled) {
          if (!sn.anchor_locked) {
            int d_cnt = 0;
            for (uint8_t t : sn.edge_type_history) if (t == 1) d_cnt++;
            const bool straight_ok = (sn.slope_angle_deg <= dual_cfg.max_lock_slope_deg) &&
                                     (sn.turn_angle_deg <= dual_cfg.max_lock_turn_deg);
            const bool hits_ok = (sn.confirmed_hits >= dual_cfg.confirm_frames);
            const bool direct_ok = (d_cnt >= dual_cfg.min_direct_evidence_frames);

            if (straight_ok && hits_ok && direct_ok) {
              sn.anchor_locked = true;
              sn.anchor_id = track.next_anchor_id++;
              sn.anchor_x = sn.x;
              sn.anchor_y = sn.y;
              sn.anchor_z = sn.z;
              sn.anchor_length = std::max(0.10, sn.max_x - sn.min_x);
              sn.anchor_width = std::max(0.10, sn.max_y - sn.min_y);
              sn.anchor_height = std::max(0.10, sn.max_z - sn.min_z);
              sn.initial_anchor_valid = true;
              sn.initial_anchor_x = sn.anchor_x;
              sn.initial_anchor_y = sn.anchor_y;
              sn.initial_anchor_z = sn.anchor_z;
              sn.moving_x = sn.x;
              sn.moving_y = sn.y;
              sn.moving_z = sn.z;
              sn.disp_from_anchor_xy = 0.0;
              sn.disp_from_anchor_z = 0.0;
              sn.relocate_confirm_count = 0;
              sn.relocate_count = 0;
              sn.is_relocated_this_frame = false;
            }
          } else {
            sn.is_relocated_this_frame = false;
            const double dx = sn.moving_x - sn.anchor_x;
            const double dy = sn.moving_y - sn.anchor_y;
            const double dz = sn.moving_z - sn.anchor_z;
            const double dxy = std::hypot(dx, dy);
            const double abs_dz = std::abs(dz);

            const bool exceed_xy = (dxy >= dual_cfg.relocate_thresh_xy);
            const bool exceed_z = (abs_dz >= dual_cfg.relocate_thresh_z);

            if (exceed_xy || exceed_z) {
              double dot_xy = (dx * sn.prev_move_dx + dy * sn.prev_move_dy);
              bool dir_consistent = (dot_xy >= 0.0) || (dxy >= 2.0 * dual_cfg.relocate_thresh_xy);

              int co_moving_layers = 0;
              for (const auto & other_sn : track.stable_nodes) {
                if (other_sn.anchor_id == sn.anchor_id || !other_sn.anchor_locked) continue;
                if (other_sn.disp_from_anchor_xy >= dual_cfg.noise_deadband_xy ||
                    other_sn.disp_from_anchor_z >= dual_cfg.noise_deadband_z) {
                  co_moving_layers++;
                }
              }
              if (crane_moving) co_moving_layers += 2;

              int required_confirm = dual_cfg.relocate_confirm_frames;
              if (co_moving_layers >= 2) {
                required_confirm = std::max(1, required_confirm - 1);
              }

              // Physical constraint on hanging wire: suppress isolated intermediate Z jump without hook motion
              bool is_isolated_z_jump = false;
              if (exceed_z && !exceed_xy && !crane_moving && track.stable_nodes.size() >= 2) {
                const auto & lowest = track.stable_nodes.back();
                if (lowest.anchor_id != sn.anchor_id && lowest.disp_from_anchor_z < dual_cfg.relocate_thresh_z) {
                  is_isolated_z_jump = true;
                }
              }

              if (dir_consistent && !is_isolated_z_jump) {
                sn.relocate_confirm_count++;
                if (sn.relocate_confirm_count >= required_confirm) {
                  sn.anchor_x = sn.moving_x;
                  sn.anchor_y = sn.moving_y;
                  sn.anchor_z = sn.moving_z;
                  sn.relocate_count++;
                  sn.is_relocated_this_frame = true;
                  sn.relocate_confirm_count = 0;

                  bool found_mem = false;
                  for (auto & mem : track.remembered_anchors) {
                    if (mem.anchor_id == sn.anchor_id) {
                      mem = sn;
                      mem.memory_age_frames = 0;
                      found_mem = true;
                      break;
                    }
                  }
                  if (!found_mem) {
                    track.remembered_anchors.push_back(sn);
                  }
                }
              } else {
                sn.relocate_confirm_count = 0;
              }
              sn.prev_move_dx = dx;
              sn.prev_move_dy = dy;
              sn.prev_move_dz = dz;
            } else {
              sn.relocate_confirm_count = 0;
            }
          }
        }
      } else {
        sn.wire_bundle_memory_age++;
        sn.is_relocated_this_frame = false;
        bool has_local_points = false;
        if (dual_cfg.enabled && sn.anchor_locked) {
          // Z-layer merge absorption check
          int local_pts = 0;
          for (const auto & p : points) {
            if (sn.fixed_roi.contains(p.x, p.y, p.z)) {
              if (++local_pts >= 5) {
                has_local_points = true;
                break;
              }
            }
          }
        }

        if (sn.is_beam_intruded) {
          sn.is_held = true;
          sn.is_estimated_gap = true;
          sn.is_connection_stable = false;
          sn.missed_frames++;
        sn.z_history.clear();
        } else if (has_local_points) {
          sn.is_held = true;
          sn.is_estimated_gap = true;
          sn.consecutive_missing = 0;
          if (sn.missed_frames < node_max_hold) {
            sn.missed_frames++;
          }
        } else {
          sn.missed_frames++;
          if (sn.missed_frames <= node_max_hold) {
            sn.is_held = true;
            sn.is_estimated_gap = true;
          }
          const auto & cs_cfg = config_.connection_stabilization;
          if (cs_cfg.enabled) {
            sn.consecutive_missing++;
            const uint8_t edge_type = (sn.missed_frames <= node_max_hold) ? 2 : 0;
            sn.edge_type_history.push_back(edge_type);
            while (static_cast<int>(sn.edge_type_history.size()) > cs_cfg.history_frames) {
              sn.edge_type_history.pop_front();
            }
            if (sn.consecutive_missing > cs_cfg.missing_hold_frames) {
              sn.is_connection_stable = false;
              sn.is_spatial_promoted = false;
              sn.edge_type_history.clear();
            }
          }
        }
      }
    }

    // New raw nodes: Return-to-Anchor Recovery and pending node confirmation
    for (size_t ri = 1; ri < raw_nodes.size(); ++ri) {
      if (raw_matched[ri]) continue;
      const auto & rn = raw_nodes[ri];
      if (rn.is_sheave) continue; // Sheave is tracked separately via sheave_node

      if (dual_cfg.enabled && dual_cfg.beam_intrusion_guard_enabled && isBeamLikeNode(rn, dual_cfg)) {
        continue;
      }

      // Check Return-to-Anchor Recovery from remembered_anchors
      bool restored_from_mem = false;
      if (dual_cfg.enabled && !track.remembered_anchors.empty()) {
        for (auto it = track.remembered_anchors.begin(); it != track.remembered_anchors.end(); ++it) {
          const double dxy = std::hypot(rn.x - it->anchor_x, rn.y - it->anchor_y);
          const double dz = std::abs(rn.z - it->anchor_z);
          if (dxy <= 0.35 && dz <= 0.35) {
            TrackedChainNodeState restored = *it;
            double eff_x = rn.x, eff_y = rn.y, eff_z = rn.z;
            if (vr_cfg.enabled) {
              WireBundleGeometry vr_geom = resolveWireBundleGeometry(rn, subclusters, points, &(*it), vr_cfg, dual_cfg);
              restored.wire_bundle = vr_geom;
              if (vr_geom.mode == CraneWireBundleMode::BOTH_STRANDS) {
                restored.remembered_bundle = vr_geom;
                restored.wire_bundle_memory_age = 0;
              } else {
                restored.wire_bundle_memory_age++;
              }
              eff_x = vr_geom.center_x;
              eff_y = vr_geom.center_y;
              eff_z = vr_geom.center_z;
            }
            restored.x = it->anchor_x;
            restored.y = it->anchor_y;
            restored.z = it->anchor_z;
            restored.moving_x = eff_x;
            restored.moving_y = eff_y;
            restored.moving_z = eff_z;
            restored.raw_obs_x = eff_x;
            restored.raw_obs_y = eff_y;
            restored.raw_obs_z = eff_z;
            restored.min_x = rn.min_x; restored.max_x = rn.max_x;
            restored.min_y = rn.min_y; restored.max_y = rn.max_y;
            restored.min_z = rn.min_z; restored.max_z = rn.max_z;
            restored.point_count = rn.point_count;
            restored.point_indices = rn.point_indices;
            restored.subcluster_id = rn.subcluster_id;
            restored.layer_id = rn.layer_id;
            restored.confirmed_hits = dual_cfg.confirm_frames;
            restored.missed_frames = 0;
            restored.is_held = false;
            restored.is_beam_intruded = false;
            restored.edge_type_history.push_back(rn.is_estimated_gap ? 2 : 1);
            track.stable_nodes.push_back(restored);
            raw_matched[ri] = true;
            restored_from_mem = true;
            track.remembered_anchors.erase(it);
            break;
          }
        }
      }
      if (restored_from_mem) continue;

      if (!track.initialized) {
        TrackedChainNodeState init_sn;
        init_sn.layer_id = rn.layer_id;
        double eff_x = rn.x, eff_y = rn.y, eff_z = rn.z;
        if (vr_cfg.enabled) {
          WireBundleGeometry vr_geom = resolveWireBundleGeometry(rn, subclusters, points, nullptr, vr_cfg, dual_cfg);
          init_sn.wire_bundle = vr_geom;
          if (vr_geom.mode == CraneWireBundleMode::BOTH_STRANDS) {
            init_sn.remembered_bundle = vr_geom;
            init_sn.wire_bundle_memory_age = 0;
          }
          eff_x = vr_geom.center_x;
          eff_y = vr_geom.center_y;
          eff_z = vr_geom.center_z;
        }
        init_sn.x = eff_x; init_sn.y = eff_y; init_sn.z = eff_z;
        init_sn.min_x = rn.min_x; init_sn.max_x = rn.max_x;
        init_sn.min_y = rn.min_y; init_sn.max_y = rn.max_y;
        init_sn.min_z = rn.min_z; init_sn.max_z = rn.max_z;
        init_sn.point_count = rn.point_count;
        init_sn.point_indices = rn.point_indices;
        init_sn.subcluster_id = rn.subcluster_id;
        init_sn.parent_subcluster_id = rn.parent_subcluster_id;
        init_sn.gap_z_to_parent = rn.gap_z_to_parent;
        init_sn.dxy_to_parent = rn.dxy_to_parent;
        init_sn.slope_angle_deg = rn.slope_angle_deg;
        init_sn.turn_angle_deg = rn.turn_angle_deg;
        init_sn.is_estimated_gap = rn.is_estimated_gap;
        init_sn.confirmed_hits = 1;
        init_sn.missed_frames = 0;
        init_sn.is_held = false;
        init_sn.moving_x = eff_x; init_sn.moving_y = eff_y; init_sn.moving_z = eff_z;
        init_sn.raw_obs_x = eff_x; init_sn.raw_obs_y = eff_y; init_sn.raw_obs_z = eff_z;
        init_sn.edge_type_history.push_back(rn.is_estimated_gap ? 2 : 1);
        track.stable_nodes.push_back(init_sn);
      } else {
        bool matched_pending = false;
        for (auto & pn : track.pending_nodes) {
          if (std::abs(pn.z - rn.z) < 0.35 && std::hypot(pn.x - rn.x, pn.y - rn.y) < 0.45) {
            matched_pending = true;
            pn.confirmed_hits++;
            double eff_x = rn.x, eff_y = rn.y, eff_z = rn.z;
            if (vr_cfg.enabled) {
              WireBundleGeometry vr_geom = resolveWireBundleGeometry(rn, subclusters, points, &pn, vr_cfg, dual_cfg);
              pn.wire_bundle = vr_geom;
              if (vr_geom.mode == CraneWireBundleMode::BOTH_STRANDS) {
                pn.remembered_bundle = vr_geom;
                pn.wire_bundle_memory_age = 0;
              } else {
                pn.wire_bundle_memory_age++;
              }
              eff_x = vr_geom.center_x;
              eff_y = vr_geom.center_y;
              eff_z = vr_geom.center_z;
            }
            pn.x = eff_x; pn.y = eff_y; pn.z = eff_z;
            pn.min_x = rn.min_x; pn.max_x = rn.max_x;
            pn.min_y = rn.min_y; pn.max_y = rn.max_y;
            pn.min_z = rn.min_z; pn.max_z = rn.max_z;
            pn.point_count = rn.point_count;
            pn.point_indices = rn.point_indices;
            pn.subcluster_id = rn.subcluster_id;
            pn.is_estimated_gap = rn.is_estimated_gap;
            pn.edge_type_history.push_back(rn.is_estimated_gap ? 2 : 1);
            pn.slope_angle_deg = rn.slope_angle_deg;
            pn.turn_angle_deg = rn.turn_angle_deg;
            pn.moving_x = eff_x; pn.moving_y = eff_y; pn.moving_z = eff_z;
            pn.raw_obs_x = eff_x; pn.raw_obs_y = eff_y; pn.raw_obs_z = eff_z;
            break;
          }
        }
        if (!matched_pending) {
          TrackedChainNodeState new_pn;
          new_pn.layer_id = rn.layer_id;
          double eff_x = rn.x, eff_y = rn.y, eff_z = rn.z;
          if (vr_cfg.enabled) {
            WireBundleGeometry vr_geom = resolveWireBundleGeometry(rn, subclusters, points, nullptr, vr_cfg, dual_cfg);
            new_pn.wire_bundle = vr_geom;
            if (vr_geom.mode == CraneWireBundleMode::BOTH_STRANDS) {
              new_pn.remembered_bundle = vr_geom;
              new_pn.wire_bundle_memory_age = 0;
            }
            eff_x = vr_geom.center_x;
            eff_y = vr_geom.center_y;
            eff_z = vr_geom.center_z;
          }
          new_pn.x = eff_x; new_pn.y = eff_y; new_pn.z = eff_z;
          new_pn.min_x = rn.min_x; new_pn.max_x = rn.max_x;
          new_pn.min_y = rn.min_y; new_pn.max_y = rn.max_y;
          new_pn.min_z = rn.min_z; new_pn.max_z = rn.max_z;
          new_pn.point_count = rn.point_count;
          new_pn.point_indices = rn.point_indices;
          new_pn.subcluster_id = rn.subcluster_id;
          new_pn.parent_subcluster_id = rn.parent_subcluster_id;
          new_pn.gap_z_to_parent = rn.gap_z_to_parent;
          new_pn.dxy_to_parent = rn.dxy_to_parent;
          new_pn.slope_angle_deg = rn.slope_angle_deg;
          new_pn.turn_angle_deg = rn.turn_angle_deg;
          new_pn.is_estimated_gap = rn.is_estimated_gap;
          new_pn.confirmed_hits = 1;
          new_pn.moving_x = eff_x; new_pn.moving_y = eff_y; new_pn.moving_z = eff_z;
          new_pn.raw_obs_x = eff_x; new_pn.raw_obs_y = eff_y; new_pn.raw_obs_z = eff_z;
          new_pn.edge_type_history.push_back(rn.is_estimated_gap ? 2 : 1);
          track.pending_nodes.push_back(new_pn);
        }
      }
    }

    // Promote confirmed pending nodes
    for (auto it = track.pending_nodes.begin(); it != track.pending_nodes.end(); ) {
      if (it->confirmed_hits >= node_confirm) {
        it->is_held = false;
        it->missed_frames = 0;
        track.stable_nodes.push_back(*it);
        it = track.pending_nodes.erase(it);
      } else {
        ++it;
      }
    }

    // Drop stale stable nodes exceeding hold window (and preserve locked anchors in remembered_anchors)
    for (auto it = track.stable_nodes.begin(); it != track.stable_nodes.end(); ) {
      const int effective_max_hold = (dual_cfg.enabled && it->is_beam_intruded) ?
          dual_cfg.intrusion_hold_frames : node_max_hold;
      if (it->missed_frames > effective_max_hold) {
        if (dual_cfg.enabled && it->anchor_locked) {
          TrackedChainNodeState mem = *it;
          mem.memory_age_frames = 0;
          track.remembered_anchors.push_back(mem);
        }
        it = track.stable_nodes.erase(it);
      } else {
        ++it;
      }
    }

    // Age remembered anchors
    if (dual_cfg.enabled) {
      for (auto it = track.remembered_anchors.begin(); it != track.remembered_anchors.end(); ) {
        if (++it->memory_age_frames > dual_cfg.anchor_memory_frames) {
          it = track.remembered_anchors.erase(it);
        } else {
          ++it;
        }
      }
    }

    // Update ROI boxes for all stable nodes
    for (auto & sn : track.stable_nodes) {
      if (dual_cfg.enabled && sn.anchor_locked) {
        sn.fixed_roi = makeBoxRoi(sn.anchor_x, sn.anchor_y, sn.anchor_z, dual_cfg.fixed_roi_size_xy, dual_cfg.fixed_roi_size_z);
        sn.moving_roi = makeBoxRoi(sn.moving_x, sn.moving_y, sn.moving_z, dual_cfg.moving_roi_size_xy, dual_cfg.moving_roi_size_z);
      }
    }

    std::sort(track.stable_nodes.begin(), track.stable_nodes.end(),
              [](const TrackedChainNodeState & a, const TrackedChainNodeState & b) {
                return a.z > b.z;
              });

    // Spatial Continuity Extension for Intermediate Connection Stabilization
    const auto & cs_cfg = config_.connection_stabilization;
    if (cs_cfg.enabled && cs_cfg.spatial_continuity_enabled && track.stable_nodes.size() >= 3) {
      const size_t num_sn = track.stable_nodes.size();

      // Step 1: Snapshot temporal stability anchors (strictly require temporal qualification to prevent cascading)
      std::vector<bool> anchor_stable(num_sn, false);
      for (size_t i = 0; i < num_sn; ++i) {
        const auto & n = track.stable_nodes[i];
        int d_cnt = 0;
        for (uint8_t t : n.edge_type_history) {
          if (t == 1) ++d_cnt;
        }
        const bool is_anchor_qualified = (n.is_connection_stable && !n.is_spatial_promoted) ||
                                         (!n.is_estimated_gap && !n.is_held);
        if (is_anchor_qualified && d_cnt >= cs_cfg.spatial_min_direct_frames && n.consecutive_missing <= 1) {
          anchor_stable[i] = true;
        }
      }

      // Step 2: Evaluate intermediate nodes (sandwiched between above and below anchors)
      for (size_t si = 1; si + 1 < num_sn; ++si) {
        auto & sn_cur = track.stable_nodes[si];

        // If already temporally stable, keep it as temporal
        if (sn_cur.is_connection_stable && !sn_cur.is_spatial_promoted) {
          continue;
        }

        const bool above_ok = anchor_stable[si - 1];
        const bool below_ok = anchor_stable[si + 1];

        int cur_d_cnt = 0;
        for (uint8_t t : sn_cur.edge_type_history) {
          if (t == 1) ++cur_d_cnt;
        }
        const bool evidence_ok = (cur_d_cnt >= cs_cfg.spatial_min_direct_frames) &&
                                 (sn_cur.consecutive_missing <= cs_cfg.missing_hold_frames);

        bool spatial_geom_ok = false;
        if (above_ok && below_ok && evidence_ok) {
          const auto & sn_above = track.stable_nodes[si - 1];
          const auto & sn_below = track.stable_nodes[si + 1];

          const double dz_above = sn_above.z - sn_cur.z;
          const double dz_below = sn_cur.z - sn_below.z;

          if (dz_above > 0.0 && dz_below > 0.0) {
            const double box_gap_z = std::max(0.0, sn_above.min_z - sn_cur.max_z);
            const bool gap_z_ok = (box_gap_z <= cs_cfg.spatial_max_gap_z) &&
                                  (dz_above <= cs_cfg.spatial_max_gap_z);

            const double dxy = std::hypot(sn_above.x - sn_cur.x, sn_above.y - sn_cur.y);
            const bool dxy_ok = (dxy <= cs_cfg.spatial_max_dxy);

            const bool angles_ok = (sn_cur.slope_angle_deg <= config_.top_down_hook.max_slope_angle_deg) &&
                                   (sn_cur.turn_angle_deg <= config_.top_down_hook.max_turn_angle_deg);

            const double total_dz = sn_above.z - sn_below.z;
            bool colin_ok = true;
            if (total_dz > 0.05) {
              const double t = (sn_cur.z - sn_below.z) / total_dz;
              const double interp_x = sn_below.x + t * (sn_above.x - sn_below.x);
              const double interp_y = sn_below.y + t * (sn_above.y - sn_below.y);
              const double colin_dist = std::hypot(sn_cur.x - interp_x, sn_cur.y - interp_y);
              colin_ok = (colin_dist <= cs_cfg.spatial_max_dxy);
            }

            if (gap_z_ok && dxy_ok && angles_ok && colin_ok) {
              spatial_geom_ok = true;
            }
          }
        }

        if (spatial_geom_ok) {
          sn_cur.is_connection_stable = true;
          sn_cur.is_spatial_promoted = true;
          sn_cur.spatial_hold_count = 0;
        } else if (sn_cur.is_spatial_promoted) {
          if (++sn_cur.spatial_hold_count <= cs_cfg.missing_hold_frames &&
              sn_cur.consecutive_missing <= cs_cfg.missing_hold_frames &&
              cur_d_cnt >= cs_cfg.spatial_min_direct_frames) {
            sn_cur.is_connection_stable = true;
          } else {
            sn_cur.is_connection_stable = false;
            sn_cur.is_spatial_promoted = false;
            sn_cur.spatial_hold_count = 0;
          }
        }
      }
    }

    track.initialized = true;

    const bool wire_out_built = !track.stable_nodes.empty();
    bool any_chain_held = false;
    if (wire_out_built) {
      std::vector<CraneHookChainNode> out_nodes;
      out_nodes.push_back(raw_nodes[0]); // Root
      int stable_conn_cnt = 0;
      int locked_cnt = 0;
      int relocated_cnt = 0;
      int intruded_cnt = 0;

      for (size_t si = 0; si < track.stable_nodes.size(); ++si) {
        const auto & sn = track.stable_nodes[si];
        CraneHookChainNode cn;
        cn.layer_id = sn.layer_id;
        cn.x = sn.x; cn.y = sn.y; cn.z = sn.z;
        cn.min_x = sn.min_x; cn.max_x = sn.max_x;
        cn.min_y = sn.min_y; cn.max_y = sn.max_y;
        cn.min_z = sn.min_z; cn.max_z = sn.max_z;
        cn.point_count = sn.point_count;
        cn.point_indices = sn.is_held ? std::vector<int>{} : sn.point_indices;
        cn.subcluster_id = sn.subcluster_id;
        cn.parent_subcluster_id = sn.parent_subcluster_id;
        cn.gap_z_to_parent = sn.gap_z_to_parent;
        cn.dxy_to_parent = sn.dxy_to_parent;
        cn.slope_angle_deg = sn.slope_angle_deg;
        cn.turn_angle_deg = sn.turn_angle_deg;
        cn.is_estimated_gap = sn.is_estimated_gap || sn.is_held;
        cn.is_held = sn.is_held;
        cn.held_frames = sn.missed_frames;
        cn.is_connection_stable = sn.is_connection_stable;
        cn.is_spatial_promoted = sn.is_spatial_promoted;

        if (dual_cfg.enabled && sn.anchor_locked) {
          cn.anchor_locked = sn.anchor_locked;
          cn.anchor_id = sn.anchor_id;
          cn.initial_anchor_x = sn.initial_anchor_x;
          cn.initial_anchor_y = sn.initial_anchor_y;
          cn.initial_anchor_z = sn.initial_anchor_z;
          cn.anchor_x = sn.anchor_x;
          cn.anchor_y = sn.anchor_y;
          cn.anchor_z = sn.anchor_z;
          cn.moving_x = sn.moving_x;
          cn.moving_y = sn.moving_y;
          cn.moving_z = sn.moving_z;
          cn.raw_obs_x = sn.raw_obs_x;
          cn.raw_obs_y = sn.raw_obs_y;
          cn.raw_obs_z = sn.raw_obs_z;
          cn.disp_from_anchor_xy = sn.disp_from_anchor_xy;
          cn.disp_from_anchor_z = sn.disp_from_anchor_z;
          cn.relocate_count = sn.relocate_count;
          cn.is_relocated_this_frame = sn.is_relocated_this_frame;
          cn.is_beam_intruded = sn.is_beam_intruded;
          cn.fixed_roi = sn.fixed_roi;
          cn.moving_roi = sn.moving_roi;
          ++locked_cnt;
          if (cn.is_relocated_this_frame) ++relocated_cnt;
          if (cn.is_beam_intruded) ++intruded_cnt;
        }

        if (vr_cfg.enabled) {
          cn.has_virtual_rope = (sn.wire_bundle.mode != CraneWireBundleMode::NONE);
          cn.wire_bundle = sn.wire_bundle;
        }

        if (sn.is_sheave) {
          cn.is_sheave = true;
          cn.sheave = sn.sheave;
        }

        if (cn.is_connection_stable) ++stable_conn_cnt;
        if (sn.is_held) any_chain_held = true;
        out_nodes.push_back(cn);
      }

      raw_chain->nodes = std::move(out_nodes);
      raw_chain->stable_edge_count = stable_conn_cnt;
      raw_chain->locked_anchor_count = locked_cnt;
      raw_chain->relocated_anchor_count = relocated_cnt;
      raw_chain->beam_intruded_count = intruded_cnt;
      double max_slope = 0.0;
      double max_turn = 0.0;
      for (const auto & cn : raw_chain->nodes) {
        max_slope = std::max(max_slope, cn.slope_angle_deg);
        max_turn = std::max(max_turn, cn.turn_angle_deg);
      }
      raw_chain->max_slope_angle_deg = max_slope;
      raw_chain->max_turn_angle_deg = max_turn;
    }

    {
      if (!raw_chain->nodes.empty()) {
        const auto & term = raw_chain->nodes.back();
        raw_chain->lowest_x = term.x; raw_chain->lowest_y = term.y; raw_chain->lowest_z = term.z;
        raw_chain->lowest_min_z = term.min_z; raw_chain->lowest_max_z = term.max_z;
        raw_chain->lowest_point_count = term.point_count;
        raw_chain->lowest_subcluster_id = term.subcluster_id;
        raw_chain->total_depth = raw_chain->root_z - raw_chain->lowest_z;
        raw_chain->is_held = any_chain_held || term.is_held;
        raw_chain->held_frames = term.held_frames;
      }
    }

    // Step B: Stabilize Largest Connected Cluster
    CraneHookNearestCluster raw_cand;
    double max_cand_metric = -1.0;
    for (size_t ni = 1; ni < raw_chain->nodes.size(); ++ni) {
      const auto & n = raw_chain->nodes[ni];
      if (n.point_indices.empty()) continue;
      const double area = computeOccupiedArea(n.point_indices, points, 0.05);
      const double len = std::max(n.max_x - n.min_x, n.max_y - n.min_y);
      const double wid = std::min(n.max_x - n.min_x, n.max_y - n.min_y);
      const double hei = n.max_z - n.min_z;
      const double metric = (area > 0.0) ? area : (len * wid);
      if (metric > max_cand_metric) {
        max_cand_metric = metric;
        raw_cand.valid = true;
        raw_cand.subcluster_id = n.subcluster_id;
        raw_cand.layer_id = n.layer_id;
        raw_cand.point_count = n.point_count;
        raw_cand.centroid_x = n.x; raw_cand.centroid_y = n.y; raw_cand.centroid_z = n.z;
        raw_cand.ref_x = n.x; raw_cand.ref_y = n.y; raw_cand.ref_z = n.z;
        raw_cand.min_x = n.min_x; raw_cand.max_x = n.max_x;
        raw_cand.min_y = n.min_y; raw_cand.max_y = n.max_y;
        raw_cand.min_z = n.min_z; raw_cand.max_z = n.max_z;
        raw_cand.length = len; raw_cand.width = wid; raw_cand.height = hei;
        raw_cand.area_xy = area;
        raw_cand.dxy = std::hypot(n.x - raw_chain->root_x, n.y - raw_chain->root_y);
        raw_cand.dz = raw_chain->root_z - n.z;
        raw_cand.dist_3d = std::hypot(raw_cand.dxy, raw_cand.dz);
      }
    }

    CraneHookNearestCluster * chosen_cluster = nullptr;
    CraneHookNearestCluster matched_same;
    bool found_same = false;

    if (track.largest_valid) {
      for (size_t ni = 1; ni < raw_chain->nodes.size(); ++ni) {
        const auto & n = raw_chain->nodes[ni];
        const double dxy = std::hypot(n.x - track.stable_largest.centroid_x,
                                      n.y - track.stable_largest.centroid_y);
        const double dz = std::abs(n.z - track.stable_largest.centroid_z);
        const bool overlap_x = (n.max_x >= track.stable_largest.min_x - 0.20 &&
                                n.min_x <= track.stable_largest.max_x + 0.20);
        const bool overlap_y = (n.max_y >= track.stable_largest.min_y - 0.20 &&
                                n.min_y <= track.stable_largest.max_y + 0.20);
        if ((dxy <= match_dist || (overlap_x && overlap_y)) && dz <= 0.40) {
          found_same = true;
          matched_same.valid = true;
          matched_same.subcluster_id = n.subcluster_id;
          matched_same.layer_id = n.layer_id;
          matched_same.point_count = n.point_count;
          matched_same.centroid_x = n.x; matched_same.centroid_y = n.y; matched_same.centroid_z = n.z;
          matched_same.ref_x = n.x; matched_same.ref_y = n.y; matched_same.ref_z = n.z;
          matched_same.min_x = n.min_x; matched_same.max_x = n.max_x;
          matched_same.min_y = n.min_y; matched_same.max_y = n.max_y;
          matched_same.min_z = n.min_z; matched_same.max_z = n.max_z;
          matched_same.length = std::max(n.max_x - n.min_x, n.max_y - n.min_y);
          matched_same.width = std::min(n.max_x - n.min_x, n.max_y - n.min_y);
          matched_same.height = n.max_z - n.min_z;
          matched_same.area_xy = computeOccupiedArea(n.point_indices, points, 0.05);
          matched_same.dxy = std::hypot(n.x - raw_chain->root_x, n.y - raw_chain->root_y);
          matched_same.dz = raw_chain->root_z - n.z;
          matched_same.dist_3d = std::hypot(matched_same.dxy, matched_same.dz);
          matched_same.is_held = n.is_held;
          break;
        }
      }
    }

    if (found_same) {
      track.largest_missed_frames = 0;
      const double m_same = (matched_same.area_xy > 0.0) ? matched_same.area_xy : (matched_same.length * matched_same.width);
      const double m_cand = (raw_cand.area_xy > 0.0) ? raw_cand.area_xy : (raw_cand.length * raw_cand.width);

      if (raw_cand.valid && raw_cand.subcluster_id != matched_same.subcluster_id && m_cand > m_same) {
        if (m_cand > 1.40 * m_same) {
          chosen_cluster = &raw_cand;
          track.largest_swap_candidate_hits = 0;
        } else {
          if (++track.largest_swap_candidate_hits >= swap_confirm) {
            chosen_cluster = &raw_cand;
            track.largest_swap_candidate_hits = 0;
          } else {
            chosen_cluster = &matched_same;
          }
        }
      } else {
        chosen_cluster = &matched_same;
        track.largest_swap_candidate_hits = 0;
      }
    } else if (track.largest_valid && ++track.largest_missed_frames <= cfg.max_hold_missed_frames) {
      track.stable_largest.is_held = true;
      track.stable_largest.held_frames = track.largest_missed_frames;
      chosen_cluster = &track.stable_largest;
    } else if (raw_cand.valid) {
      chosen_cluster = &raw_cand;
      track.largest_missed_frames = 0;
      track.largest_swap_candidate_hits = 0;
    }

    // Step C: PCA Yaw Disambiguation & Dimension Smoothing
    if (chosen_cluster && chosen_cluster->valid) {
      std::vector<int> cluster_pts;
      for (const auto & n : raw_chain->nodes) {
        if (!n.is_held && n.subcluster_id == chosen_cluster->subcluster_id && !n.point_indices.empty()) {
          for (int idx : n.point_indices) {
            if (idx >= 0 && static_cast<size_t>(idx) < points.size() &&
                std::isfinite(points[idx].x) && std::isfinite(points[idx].y)) cluster_pts.push_back(idx);
          }
          break;
        }
      }

      double raw_yaw = chosen_cluster->yaw;
      double pca_len = chosen_cluster->length;
      double pca_wid = chosen_cluster->width;

      if (!cluster_pts.empty()) {
        double sx = 0.0, sy = 0.0;
        for (int idx : cluster_pts) {
          sx += points[idx].x; sy += points[idx].y;
        }
        const double n_pts = static_cast<double>(cluster_pts.size());
        const double cx = sx / n_pts;
        const double cy = sy / n_pts;

        double cxx = 0.0, cyy = 0.0, cxy = 0.0;
        for (int idx : cluster_pts) {
          const double dx = points[idx].x - cx;
          const double dy = points[idx].y - cy;
          cxx += dx * dx; cyy += dy * dy; cxy += dx * dy;
        }
        cxx /= n_pts; cyy /= n_pts; cxy /= n_pts;

        raw_yaw = 0.5 * std::atan2(2.0 * cxy, cxx - cyy);
        const double cos_y = std::cos(raw_yaw);
        const double sin_y = std::sin(raw_yaw);

        double min_u = std::numeric_limits<double>::max(), max_u = std::numeric_limits<double>::lowest();
        double min_v = std::numeric_limits<double>::max(), max_v = std::numeric_limits<double>::lowest();
        for (int idx : cluster_pts) {
          const double dx = points[idx].x - cx;
          const double dy = points[idx].y - cy;
          const double u = dx * cos_y + dy * sin_y;
          const double v = -dx * sin_y + dy * cos_y;
          min_u = std::min(min_u, u); max_u = std::max(max_u, u);
          min_v = std::min(min_v, v); max_v = std::max(max_v, v);
        }
        pca_len = std::max(0.08, max_u - min_u);
        pca_wid = std::max(0.08, max_v - min_v);
        if (pca_len < pca_wid) {
          std::swap(pca_len, pca_wid);
          raw_yaw += M_PI * 0.5;
        }
      }

      double prev_yaw = track.largest_valid ? track.stable_largest.yaw : raw_yaw;
      auto norm_ang = [](double a) {
        while (a > M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
      };

      double d0 = norm_ang(raw_yaw - prev_yaw);
      double d_pi = norm_ang(raw_yaw + M_PI - prev_yaw);
      double d_neg_pi = norm_ang(raw_yaw - M_PI - prev_yaw);
      double best_d = d0;
      double aligned_yaw = raw_yaw;
      if (std::abs(d_pi) < std::abs(best_d)) {
        best_d = d_pi;
        aligned_yaw = raw_yaw + M_PI;
      }
      if (std::abs(d_neg_pi) < std::abs(best_d)) {
        best_d = d_neg_pi;
        aligned_yaw = raw_yaw - M_PI;
      }

      const double aspect = (pca_wid > 0.05) ? (pca_len / pca_wid) : 1.0;
      const bool is_near_90 = (std::abs(std::abs(best_d) - M_PI * 0.5) < 0.26);

      double final_yaw = aligned_yaw;
      if (track.largest_valid) {
        if (aspect >= 1.3 && is_near_90) {
          if (++track.consecutive_yaw_jump_frames < jump_confirm) {
            final_yaw = prev_yaw;
          } else {
            track.consecutive_yaw_jump_frames = 0;
            final_yaw = aligned_yaw;
          }
        } else if (std::abs(best_d) > (cfg.yaw_jump_tolerance_deg * M_PI / 180.0)) {
          if (++track.consecutive_yaw_jump_frames < jump_confirm) {
            final_yaw = prev_yaw;
          } else {
            track.consecutive_yaw_jump_frames = 0;
            final_yaw = aligned_yaw;
          }
        } else {
          track.consecutive_yaw_jump_frames = 0;
          final_yaw = norm_ang(prev_yaw + alpha_yaw * best_d);
        }
      }

      double final_len = pca_len;
      double final_wid = pca_wid;
      double final_cx = chosen_cluster->centroid_x;
      double final_cy = chosen_cluster->centroid_y;
      double final_cz = chosen_cluster->centroid_z;
      double final_area = chosen_cluster->area_xy;

      if (track.largest_valid && !chosen_cluster->is_held) {
        const double d_len = std::abs(pca_len - track.stable_largest.length);
        const double d_wid = std::abs(pca_wid - track.stable_largest.width);

        if (d_len <= jump_len && d_wid <= jump_wid) {
          track.consecutive_jump_frames = 0;
          final_len = (1.0 - alpha) * track.stable_largest.length + alpha * pca_len;
          final_wid = (1.0 - alpha) * track.stable_largest.width + alpha * pca_wid;
          final_cx = (1.0 - alpha) * track.stable_largest.centroid_x + alpha * chosen_cluster->centroid_x;
          final_cy = (1.0 - alpha) * track.stable_largest.centroid_y + alpha * chosen_cluster->centroid_y;
          final_cz = (1.0 - alpha) * track.stable_largest.centroid_z + alpha * chosen_cluster->centroid_z;
          final_area = (1.0 - alpha) * track.stable_largest.area_xy + alpha * chosen_cluster->area_xy;
        } else {
          if (++track.consecutive_jump_frames >= jump_confirm) {
            track.consecutive_jump_frames = 0;
            final_len = pca_len;
            final_wid = pca_wid;
          } else {
            final_len = track.stable_largest.length;
            final_wid = track.stable_largest.width;
          }
        }
      }

      track.stable_largest = *chosen_cluster;
      track.stable_largest.valid = true;
      track.stable_largest.yaw = final_yaw;
      track.stable_largest.length = final_len;
      track.stable_largest.width = final_wid;
      track.stable_largest.centroid_x = final_cx;
      track.stable_largest.centroid_y = final_cy;
      track.stable_largest.centroid_z = final_cz;
      track.stable_largest.area_xy = final_area;
      track.stable_largest.is_held = chosen_cluster->is_held;
      track.stable_largest.held_frames = chosen_cluster->held_frames;
      track.largest_valid = true;
      track.largest_confirmed_frames++;

      raw_chain->largest_cluster = track.stable_largest;
      if (side == 0) td_res.left_largest_cluster = track.stable_largest;
      else td_res.right_largest_cluster = track.stable_largest;
    }
  }
}

void CranePipeline::evaluateExceptionGate(
    CraneTopDownHookResult & td_res,
    const std::vector<PointXYZIRT> & points,
    VisibilityModel * visibility) {
  const auto & cfg = config_.exception_gate;
  if (!cfg.enabled) {
    return;
  }

  const auto t_start = std::chrono::steady_clock::now();

  for (int side = 0; side < 2; ++side) {
    const bool is_left = (side == 0);
    TrackedTopDownStructure & track = is_left ? left_td_track_ : right_td_track_;
    const int chain_idx = is_left ? td_res.left_chain_idx : td_res.right_chain_idx;
    CraneHookChain * chain = (chain_idx >= 0 && chain_idx < static_cast<int>(td_res.chains.size())) ?
        &td_res.chains[chain_idx] : nullptr;

    CraneExceptionGateResult gate;

    if (!chain || chain->nodes.empty()) {
      gate.status = CraneExceptionGateStatus::UNKNOWN;
      gate.reason = CraneExceptionGateReason::NO_CHAIN;
      gate.visibility_status = CraneVisibilityGateStatus::UNKNOWN;
      track.consecutive_drop_frames = 0;
      track.prev_gate_status = gate.status;
      if (is_left) td_res.left_exception_gate = gate;
      else td_res.right_exception_gate = gate;
      continue;
    }

    // 1. Check if the chain itself is currently held by temporal stabilization
    if (chain->is_held) {
      gate.status = CraneExceptionGateStatus::UNKNOWN;
      gate.reason = CraneExceptionGateReason::TEMPORAL_HOLD;
      gate.visibility_status = CraneVisibilityGateStatus::HELD;
      gate.is_held = true;
      gate.evaluating_node_count = static_cast<int>(chain->nodes.size());
      if (chain->largest_cluster.valid) {
        gate.largest_cluster_length = chain->largest_cluster.length;
        gate.largest_cluster_area = chain->largest_cluster.area_xy;
        gate.max_connected_length = chain->largest_cluster.length;
        gate.max_connected_area = chain->largest_cluster.area_xy;
      }
      gate.processing_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - t_start).count();
      chain->exception_gate = gate;
      if (is_left) td_res.left_exception_gate = gate;
      else td_res.right_exception_gate = gate;
      continue;
    }

    // 2. Scan connected nodes below Root (ni >= 1) to decouple max_length and max_area
    double max_len = 0.0;
    double max_area = 0.0;
    int eval_node_count = 0;

    if (cfg.scan_all_chain_nodes) {
      for (size_t ni = 1; ni < chain->nodes.size(); ++ni) {
        const auto & n = chain->nodes[ni];
        const double len = std::max(n.max_x - n.min_x, n.max_y - n.min_y);
        double area = 0.0;
        if (!n.point_indices.empty()) {
          area = computeOccupiedArea(n.point_indices, points, 0.05);
        }
        if (len > max_len) max_len = len;
        if (area > max_area) max_area = area;
        eval_node_count++;
      }
    }

    if (chain->largest_cluster.valid) {
      if (chain->largest_cluster.length > max_len) max_len = chain->largest_cluster.length;
      if (chain->largest_cluster.area_xy > max_area) max_area = chain->largest_cluster.area_xy;
      gate.largest_cluster_length = chain->largest_cluster.length;
      gate.largest_cluster_area = chain->largest_cluster.area_xy;
    }

    gate.max_connected_length = max_len;
    gate.max_connected_area = max_area;
    gate.evaluating_node_count = eval_node_count;

    const bool len_pass = (max_len >= cfg.min_length_m);
    const bool area_pass = (max_area >= cfg.min_area_m2);
    const bool criteria_met = len_pass || area_pass;

    // 3. Query FOV and Occlusion from VisibilityModel if enabled
    bool in_fov = true;
    bool is_occluded = false;
    double occ_fraction = 0.0;
    VisibilityReason vis_reason = VisibilityReason::OBSERVABLE;

    if (cfg.use_visibility && visibility != nullptr) {
      VisibilityBox probe_box;
      if (chain->largest_cluster.valid) {
        probe_box.min_x = chain->largest_cluster.min_x;
        probe_box.max_x = chain->largest_cluster.max_x;
        probe_box.min_y = chain->largest_cluster.min_y;
        probe_box.max_y = chain->largest_cluster.max_y;
        probe_box.min_z = chain->largest_cluster.min_z;
        probe_box.max_z = chain->largest_cluster.max_z;
      } else if (chain->nodes.size() > 1) {
        const auto & lowest = chain->nodes.back();
        probe_box.min_x = lowest.min_x;
        probe_box.max_x = lowest.max_x;
        probe_box.min_y = lowest.min_y;
        probe_box.max_y = lowest.max_y;
        probe_box.min_z = lowest.min_z;
        probe_box.max_z = lowest.max_z;
      } else {
        probe_box.min_x = chain->root_x - 0.2;
        probe_box.max_x = chain->root_x + 0.2;
        probe_box.min_y = chain->root_y - 0.2;
        probe_box.max_y = chain->root_y + 0.2;
        probe_box.min_z = chain->root_z - 0.5;
        probe_box.max_z = chain->root_z;
      }

      const auto v_info = visibility->evaluateBox(probe_box, -1, false);
      in_fov = v_info.in_horizontal_fov && v_info.in_vertical_fov;
      occ_fraction = v_info.observable_samples > 0 ? v_info.occluded_fraction : 0.0;
      is_occluded = (occ_fraction >= cfg.max_occlusion_fraction) ||
                    (v_info.reason == VisibilityReason::OCCLUDED_BY_OTHER) ||
                    (v_info.reason == VisibilityReason::FULLY_OCCLUDED_LIKELY);
      vis_reason = v_info.reason;
      gate.occlusion_fraction = occ_fraction;
      gate.in_fov = in_fov;
    }

    // 4. Tri-State Decision with Temporal Stabilization integration
    if (criteria_met) {
      gate.status = CraneExceptionGateStatus::PASS;
      if (len_pass && area_pass) gate.reason = CraneExceptionGateReason::LENGTH_AND_AREA;
      else if (len_pass) gate.reason = CraneExceptionGateReason::LENGTH_MET;
      else gate.reason = CraneExceptionGateReason::AREA_MET;

      if (!in_fov) gate.visibility_status = CraneVisibilityGateStatus::OUT_OF_FOV;
      else if (is_occluded) gate.visibility_status = CraneVisibilityGateStatus::OCCLUDED;
      else if (occ_fraction > 0.05) gate.visibility_status = CraneVisibilityGateStatus::PARTIAL;
      else gate.visibility_status = CraneVisibilityGateStatus::OBSERVED;

      track.consecutive_drop_frames = 0;
      track.prev_gate_status = CraneExceptionGateStatus::PASS;
    } else {
      // Criteria NOT met: Check why (Occlusion, FOV, or genuine small size)
      if (!in_fov || vis_reason == VisibilityReason::OUT_OF_HORIZONTAL_FOV || vis_reason == VisibilityReason::OUT_OF_VERTICAL_FOV) {
        gate.status = CraneExceptionGateStatus::UNKNOWN;
        gate.reason = CraneExceptionGateReason::FOV_TRUNCATED;
        gate.visibility_status = CraneVisibilityGateStatus::OUT_OF_FOV;
      } else if (is_occluded) {
        gate.status = CraneExceptionGateStatus::UNKNOWN;
        gate.reason = CraneExceptionGateReason::OCCLUDED;
        gate.visibility_status = CraneVisibilityGateStatus::OCCLUDED;
      } else {
        // Sufficiently observed, but below criteria.
        // Check temporal stability: If previous state was PASS, do not drop immediately to EXCLUDED on a transient!
        const int confirm_drop_frames = std::max(1, config_.temporal_stabilization.jump_confirm_frames);
        if (track.prev_gate_status == CraneExceptionGateStatus::PASS && ++track.consecutive_drop_frames < confirm_drop_frames) {
          gate.status = CraneExceptionGateStatus::UNKNOWN;
          gate.reason = CraneExceptionGateReason::TEMPORAL_HOLD;
          gate.visibility_status = CraneVisibilityGateStatus::HELD;
        } else {
          track.consecutive_drop_frames = 0;
          gate.status = CraneExceptionGateStatus::EXCLUDED;
          gate.reason = CraneExceptionGateReason::BELOW_THRESHOLD;
          gate.visibility_status = (occ_fraction > 0.05) ? CraneVisibilityGateStatus::PARTIAL : CraneVisibilityGateStatus::OBSERVED;
          track.prev_gate_status = CraneExceptionGateStatus::EXCLUDED;
        }
      }
    }

    gate.processing_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t_start).count();

    chain->exception_gate = gate;
    if (is_left) td_res.left_exception_gate = gate;
    else td_res.right_exception_gate = gate;
  }
}

} // namespace cx128s2_bev_perception

