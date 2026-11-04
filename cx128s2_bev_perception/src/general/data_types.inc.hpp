struct BackgroundCell {
  int ix = 0;
  int iy = 0;
  double center_x = 0.0;
  double center_y = 0.0;
  double occupancy_probability = 0.0;
  int seen_count = 0;
  double ground_z = 0.0;
  double mean_z = 0.0;
  double std_z = 0.0;
  double max_z = 0.0;
  double min_z = 0.0;
  double max_density = 0.0;
  double static_score = 0.0;
  double last_seen_time = 0.0;
  bool valid = false;
};

struct AccumCell {
  int seen_frames = 0;
  int point_count_sum = 0;
  int voxel_count_sum = 0;  // voxelized point count (논문 num_points에 해당)
  double z_sum = 0.0;
  double z2_sum = 0.0;
  double min_z = std::numeric_limits<double>::max();
  double max_z = std::numeric_limits<double>::lowest();
};

struct FrameCell {
  std::vector<int> indices;
  double z_sum = 0.0;
  double min_z = std::numeric_limits<double>::max();
  double max_z = std::numeric_limits<double>::lowest();
};

struct GroundCell {
  double z = 0.0;
  bool valid = false;
};

struct RoiZone {
  std::string name;
  std::string type = "watch";  // watch, ignore, hard_ignore
  std::string shape = "polygon";
  double x_min = 0.0;
  double x_max = 0.0;
  double y_min = 0.0;
  double y_max = 0.0;
  std::vector<double> polygon_x;
  std::vector<double> polygon_y;
  bool enabled = true;
};

enum class RoiDecision {
  KEEP,
  IGNORE,
  HARD_IGNORE,
  OUTSIDE_WATCH,
};

struct ShapeNode {
  int id = 0;
  std::vector<int> point_indices;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double min_x = 0.0;
  double max_x = 0.0;
  double min_y = 0.0;
  double max_y = 0.0;
  double min_z = 0.0;
  double max_z = 0.0;
  int point_count = 0;
};

struct ShapeEdge {
  int a = 0;
  int b = 0;
  double top_score = 0.0;
  double side_score = 0.0;
  double front_score = 0.0;
  double total_score = 0.0;
  std::string reason;
};

struct ClusterResult {
  int id = 0;
  std::vector<int> indices;
  std::vector<int64_t> cell_keys;
  double centroid_x = 0.0;
  double centroid_y = 0.0;
  double min_x = 0.0;
  double max_x = 0.0;
  double min_y = 0.0;
  double max_y = 0.0;
  double length = 0.0;
  double width = 0.0;
  int point_count = 0;
  int cell_count = 0;
  std::string method;
  std::string roi_name;
  std::string roi_type;
  int shape_node_count = 0;
  int shape_edge_count = 0;
  double shape_graph_score = 0.0;
  double shape_top_linearity = 0.0;
  double shape_side_linearity = 0.0;
  double shape_front_linearity = 0.0;
  double shape_area_top = 0.0;
  std::string component_type = "unknown_like";
  double component_type_confidence = 0.0;
  std::string component_type_reason;
  std::string fem_memory_id;
  std::string fem_cluster_role;
  double fem_ownership_score = 0.0;
  std::vector<ShapeNode> shape_nodes;
  std::vector<ShapeEdge> shape_edges;
};

struct ObjectFeature {
  int id = 0;
  VisibilityInfo visibility;
  cx128s2_bev_perception::vehicle_surface::Obb obb;
  std::vector<int> indices;
  double centroid_x = 0.0;
  double centroid_y = 0.0;
  double centroid_z = 0.0;
  double min_x = 0.0;
  double max_x = 0.0;
  double min_y = 0.0;
  double max_y = 0.0;
  double min_z = 0.0;
  double max_z = 0.0;
  double length = 0.0;
  double width = 0.0;
  double height = 0.0;
  int point_count = 0;
  int cell_count = 0;
  int raw_box_point_count = 0, raw_box_foreground_count = 0;
  bool raw_box_used = false;
  std::string raw_box_reason = "DISABLED";
  double density = 0.0;
  int ring_coverage = 0;
  bool vehicle_height_supported = true;
  double mean_intensity = 0.0;
  double range = 0.0;
  double radial_velocity = 0.0;
  double range_slope = 0.0;
  double speed = 0.0;
  int age = 0;
  int hits = 0;
  int lost_frames = 0;
  double persistence = 0.0;
  double background_similarity = 0.0;
  double dynamic_score = 0.0;
  int shape_node_count = 0;
  int shape_edge_count = 0;
  double shape_graph_score = 0.0;
  double shape_top_linearity = 0.0;
  double shape_side_linearity = 0.0;
  double shape_front_linearity = 0.0;
  double shape_area_top = 0.0;
  std::string component_type = "unknown_like";
  double component_type_confidence = 0.0;
  std::string component_type_reason;
  double shape_memory_match_score = 0.0;
  double shape_memory_bbox_score = 0.0;
  double shape_memory_node_score = 0.0;
  double shape_memory_endpoint_relation_score = 0.0;
  double shape_memory_edge_score = 0.0;
  double shape_memory_height_score = 0.0;
  double shape_memory_final_score = 0.0;
  double final_association_score = 0.0;
  std::string object_memory_id;
  std::string object_memory_class_label;
  bool object_memory_applicable_match = false;
  bool object_memory_skipped_by_component_type = false;
  std::string object_memory_skip_reason;
  double object_memory_bank_match_score = 0.0;
  double fem_ownership_score = 0.0;
  int fem_owned_point_count = 0;
  int fem_outside_point_count = 0;
  int fem_cargo_candidate_point_count = 0;
  int fem_attached_unknown_point_count = 0;
  double cargo_score = 0.0;
  bool cargo_loaded_candidate = false;
  std::string track_state = "NEW";
  bool inside_roi = true;
  std::string decision_reason = "BACKGROUND_DIFF";
  std::string rejection_reason;
  std::vector<ShapeNode> shape_nodes;
  std::vector<ShapeEdge> shape_edges;
};

struct DetectionResult {
  std::string class_name = "UNKNOWN";
  double score = 0.0;
  bool is_valid = false;
  std::string lifecycle_state = "CANDIDATE";
  std::string motion_state = "UNKNOWN_MOTION";
  std::string decision_reason;
  std::string rejection_reason;
};

struct ShapeMemoryNode {
  int id = 0;
  double local_x = 0.0;
  double local_y = 0.0;
  double local_z = 0.0;
  int point_count = 0;
  double confidence = 0.0;
  int observed_count = 0;
  int missing_count = 0;
  bool is_endpoint = false;
  bool is_boundary = false;
  bool is_corner_candidate = false;
  bool is_virtual = false;
  bool is_mirrored = false;
  int mirrored_from_node_id = -1;
  double visibility_confidence = 1.0;
  std::string semantic_hint = "unknown";
};

struct ShapeMemoryEdge {
  int id = 0;
  int node_a = -1;
  int node_b = -1;
  double local_dx = 0.0;
  double local_dy = 0.0;
  double local_dz = 0.0;
  double length = 0.0;
  double direction_yaw = 0.0;
  double height_delta = 0.0;
  double top_score = 0.0;
  double side_score = 0.0;
  double front_score = 0.0;
  double confidence = 0.0;
  int observed_count = 0;
  int missing_count = 0;
  bool is_structural_edge = false;
  bool is_virtual = false;
  bool is_mirrored = false;
  int mirrored_from_edge_id = -1;
  double visibility_confidence = 1.0;
};

struct EndpointRelation {
  int endpoint_a = -1;
  int endpoint_b = -1;
  double expected_distance = 0.0;
  double expected_direction_yaw = 0.0;
  double expected_height_delta = 0.0;
  double confidence = 0.0;
  std::string relation_type = "unknown";  // length, width, height, structural
  double relation_score = 0.0;
  bool uses_virtual_node = false;
  bool uses_mirrored_node = false;
  int observed_count = 0;
  int missing_count = 0;
};

struct ShapeMemoryMatchBreakdown {
  double bbox_score = 0.0;
  double node_score = 0.0;
  double endpoint_relation_score = 0.0;
  double edge_score = 0.0;
  double height_score = 0.0;
  double final_score = 0.0;
};

struct MirrorModeInfo {
  bool enabled = false;
  std::string plane = "xz";
  std::string flipped_axis = "local_y";
  double center_x = 0.0;
  double center_y = 0.0;
  double center_z = 0.0;
  double flipped_axis_center = 0.0;
  std::string mirror_axis = "local_y";
  double mirror_centerline = 0.0;
  double mirror_centerline_x = 0.0;
  double mirror_centerline_y = 0.0;
  double mirror_centerline_z = 0.0;
  double symmetry_confidence = 0.0;
  int observed_node_count = 0;
  int mirrored_node_count = 0;
  int observed_edge_count = 0;
  int mirrored_edge_count = 0;
  int near_identical_warning_count = 0;
  int invalid_edge_warning_count = 0;
};

struct ShapeMemory {
  bool valid = false;
  bool committed = false;
  bool frozen = false;
  int track_id = -1;
  double center_x = 0.0;
  double center_y = 0.0;
  double center_z = 0.0;
  double yaw = 0.0;
  double length = 0.0;
  double width = 0.0;
  double height = 0.0;
  double min_local_x = 0.0;
  double max_local_x = 0.0;
  double min_local_y = 0.0;
  double max_local_y = 0.0;
  double min_local_z = 0.0;
  double max_local_z = 0.0;
  std::vector<ShapeMemoryNode> nodes;
  std::vector<ShapeMemoryEdge> edges;
  std::vector<EndpointRelation> endpoint_relations;
  MirrorModeInfo mirror_info;
  double top_area = 0.0;
  double confidence = 0.0;
  std::string class_label = "UNKNOWN";
  std::string class_prior = "UNKNOWN";
  std::vector<std::string> applicable_component_types;
  std::string source_component_type = "unknown_like";
  double source_component_type_confidence = 0.0;
  std::string source_component_type_reason;
  double class_confidence = 0.0;
  double vehicle_like_score = 0.0;
  double truck_like_score = 0.0;
  double memory_confidence = 0.0;
  double learned_avg_speed = 0.0;
  double learned_motion_yaw = 0.0;
  double learned_radial_velocity = 0.0;
  int learned_motion_samples = 0;
  bool reprepare_needed = false;
  int low_match_frames = 0;
  int commit_frame = 0;
  int update_count = 0;
};

struct HeightProfile {
  double front_height = 0.0;
  double middle_height = 0.0;
  double rear_height = 0.0;
  double front_rear_height_delta = 0.0;
  double max_height = 0.0;
  double height_variance = 0.0;
  double cargo_score = 0.0;
};

struct ObjectMemoryEntry {
  std::string memory_id;
  std::string class_label = "UNKNOWN";
  std::string class_prior = "UNKNOWN";
  std::vector<std::string> applicable_component_types;
  std::string source_component_type = "unknown_like";
  double source_component_type_confidence = 0.0;
  std::string source_component_type_reason;
  std::string distance_profile = "mixed";
  ShapeMemory shape_memory;
  HeightProfile height_profile;
  MirrorModeInfo mirror_info;
  double memory_confidence = 0.0;
  double class_confidence = 0.0;
  int observed_count = 0;
  int matched_count = 0;
  int false_match_count = 0;
  double created_time = 0.0;
  double last_matched_time = 0.0;
};

struct ObjectMemoryBank {
  std::vector<ObjectMemoryEntry> entries;
};

struct ObjectMemoryBankMatch {
  bool matched = false;
  int entry_index = -1;
  std::string memory_id;
  std::string class_label = "UNKNOWN";
  std::string class_prior = "UNKNOWN";
  double final_score = 0.0;
  double bbox_score = 0.0;
  double node_score = 0.0;
  double edge_score = 0.0;
  double relation_score = 0.0;
  double height_profile_score = 0.0;
  double distance_profile_score = 0.0;
  bool skipped_by_component_type = false;
  int skipped_by_component_type_count = 0;
  std::string skip_reason;
};

struct FemOwnershipResult {
  std::vector<int> owned_indices;
  std::vector<int> outside_indices;
  std::vector<int> cargo_candidate_indices;
  std::vector<int> attached_unknown_indices;
  double ownership_score = 0.0;
  double bbox_score = 0.0;
  double node_support_score = 0.0;
  double edge_support_score = 0.0;
  double height_profile_score = 0.0;
  int observed_supported_nodes = 0;
  int virtual_supported_nodes = 0;
};

struct ObservedShapeGraphCache {
  bool valid = false;
  int track_id = -1;
  int start_frame = 0;
  int last_update_frame = 0;
  int captured_frames = 0;
  int target_capture_frames = 0;

  std::vector<PointXYZIRT> accumulated_local_points;
  std::vector<ShapeNode> nodes;
  std::vector<ShapeEdge> edges;

  double confidence = 0.0;
  std::string state = "empty";  // "empty", "capturing", "ready"
};

enum class GeometryEvidence { UNKNOWN, LOWER_BOUND, MEASURED };

const char * geometryEvidenceName(GeometryEvidence value) {
  switch (value) {
    case GeometryEvidence::MEASURED: return "MEASURED";
    case GeometryEvidence::LOWER_BOUND: return "LOWER_BOUND";
    default: return "UNKNOWN";
  }
}

enum class VehicleObservationQuality { FULL, PARTIAL, SPARSE, NONE };
struct SparseSupportFrame {
  double x = 0.0, y = 0.0, stamp = 0.0;
  int independent_bins = 0;
};
struct TrackObservationMemory {
  bool vehicle_confirmed = false;
  bool body_pose_anchor_valid = false;
  cx128s2_bev_perception::vehicle_surface::Point2 body_pose_anchor_local;
  double body_pose_anchor_length = 0.0, body_pose_anchor_width = 0.0;
  cx128s2_bev_perception::vehicle_surface::VehicleLifecycle vehicle_state =
      cx128s2_bev_perception::vehicle_surface::VehicleLifecycle::ACTIVE;
  double lost_since = 0.0, dormant_since = 0.0;
  double last_seen_stamp = 0.0, last_known_distance = 0.0;
  double last_observed_x = 0.0, last_observed_y = 0.0;
  bool last_observed_valid = false;
  double final_predicted_x = 0.0, final_predicted_y = 0.0;
  bool final_predicted_valid = false;
  bool fifo_exit_observed = false; // Actual exit crossing; prediction cannot set this.
  cx128s2_bev_perception::vehicle_surface::DirectionMemory corridor_direction;
  cx128s2_bev_perception::vehicle_surface::FifoPhase fifo_phase =
      cx128s2_bev_perception::vehicle_surface::FifoPhase::NOT_REGISTERED;
  uint64_t entry_sequence = 0;
  double entry_timestamp = 0.0;
  std::string fifo_last_reject_reason;
  VehicleObservationQuality last_observed_quality = VehicleObservationQuality::NONE;
  int reacquire_geometry_hold_frames = 0;
  cx128s2_bev_perception::vehicle_surface::ReacquirePending reacquire_pending;
  double reacquire_best_score = -1.0, reacquire_second_score = -1.0;
  double reacquire_position_error = 0.0, reacquire_lateral_error = 0.0;
  double reacquire_yaw_error = 0.0;
  cx128s2_bev_perception::vehicle_surface::Point2 reacquire_candidate_center;
  bool reacquire_candidate_valid = false;
  std::string reacquire_result = "NONE";
  cx128s2_bev_perception::vehicle_surface::Obb vehicle_obb;
  double vehicle_z_center = 0.0, vehicle_height = 0.0;
  std::vector<cx128s2_bev_perception::vehicle_surface::Point2> local_surface;
  std::vector<cx128s2_bev_perception::vehicle_surface::Point2> predicted_visible_world;
  std::vector<cx128s2_bev_perception::vehicle_surface::Point2> owned_component_centers;
  std::vector<cx128s2_bev_perception::vehicle_surface::Point2> actual_support_world;
  std::deque<SparseSupportFrame> sparse_history;
  cx128s2_bev_perception::vehicle_surface::LateralAnchor lateral_anchor;
  VehicleObservationQuality vehicle_quality = VehicleObservationQuality::NONE;
  double frame_stamp = 0.0;
  double predicted_x = 0.0, predicted_y = 0.0, predicted_yaw = 0.0;
  double raw_centroid_x = 0.0, raw_centroid_y = 0.0;
  double measured_x = 0.0, measured_y = 0.0;
  double measured_yaw = std::numeric_limits<double>::quiet_NaN();
  double lateral_innovation = 0.0, yaw_innovation = 0.0;
  double measurement_std_x = 0.0, measurement_std_y = 0.0;
  bool measured_center_valid = false, lateral_measurement_accepted = false;
  bool yaw_update_accepted = false, anchor_updated = false;
  std::string lateral_update_reason = "PREDICTION_ONLY";
  std::string yaw_update_reason = "PREDICTION_ONLY";
  int extension_frames = 0, owned_component_count = 0, sparse_support_count = 0;
  int expected_visible_samples = 0, visible_supported_samples = 0;
  double measurement_std_used = 0.0;
  std::string geometry_update = "NONE", association_source = "PREDICTION_ONLY";
  bool reliable = false;
  VisibilityBox last_reliable_box;
  double reliable_center_x = 0.0, reliable_center_y = 0.0, reliable_center_z = 0.0;
  double reliable_width = 0.0, reliable_height = 0.0, reliable_length = 0.0;
  double geometry_confidence = 0.0;
  int reliable_expected_rays = 0;
  double last_reliable_stamp = 0.0;
  VisibilityInfo last_visibility;
  double observation_age_sec = 0.0, missed_time_sec = 0.0;
  int stable_observation_frames = 0;
  GeometryEvidence width_evidence = GeometryEvidence::UNKNOWN;
  GeometryEvidence height_evidence = GeometryEvidence::UNKNOWN;
  GeometryEvidence length_evidence = GeometryEvidence::UNKNOWN;
  VisibilityReason miss_reason = VisibilityReason::UNKNOWN;
};

struct Track {
  int id = 0;
  VisibilityInfo visibility;
  TrackObservationMemory observation_memory;
  ObjectFeature feature;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double vx = 0.0;
  double vy = 0.0;
  double yaw = std::numeric_limits<double>::quiet_NaN();
  double last_range = 0.0;
  double range_slope = 0.0;
  double last_time = 0.0;
  double last_prediction_time = 0.0;
  int age = 0;
  int hits = 0;
  int lost_frames = 0;
  std::deque<bool> recent_hits;
  // Current measured front evidence; unrelated generic hits cannot confirm a front.
  std::deque<bool> recent_front_hits;
  bool front_class_confirmed = false;
  DetectionResult detection;
  cx128s2_bev_perception::VehicleStatusState vehicle_status;
  bool vehicle_status_initialized = false;
  bool status_velocity_reinitialized = false;
  ShapeMemory shape_memory;
  ObservedShapeGraphCache observed_shape_cache;
  // KF state is intentionally separate from x/y/vx/vy: those remain the legacy
  // association state in Phase 1, so enabling KF cannot change association.
  cx128s2_bev_perception::LinearKalmanFilter kf;
  // The measurement and prediction must retain the same physical reference.
  // This flag does not confirm identity, geometry, existence or lifecycle.
  bool position_from_body_box = false;
  bool multi_component_body = false;
  // Measured part-center offsets from the initial whole-body reference.
  std::vector<std::array<double,4>> component_references;
  cx128s2_bev_perception::vehicle_surface::Point2 observed_exit_direction{};
  bool observed_exit_direction_valid = false;
};

double clamp(double v, double lo, double hi) {
  return std::max(lo, std::min(v, hi));
}

double normalPdf(double x, double mean, double stddev) {
  const double sigma = std::max(1e-3, stddev);
  const double z = (x - mean) / sigma;
  return std::exp(-0.5 * z * z) / (sigma * std::sqrt(2.0 * M_PI));
}

int64_t hash2d(int x, int y) {
  return (static_cast<int64_t>(static_cast<uint32_t>(x)) << 32) |
         static_cast<uint32_t>(y);
}

// 3D voxel hash (FNV-1a inspired, for voxelization)
int64_t hash3d(int x, int y, int z) {
  uint64_t h = 14695981039346656037ULL;
  h ^= static_cast<uint32_t>(x); h *= 1099511628211ULL;
  h ^= static_cast<uint32_t>(y); h *= 1099511628211ULL;
  h ^= static_cast<uint32_t>(z); h *= 1099511628211ULL;
  return static_cast<int64_t>(h);
}

int unpackX(int64_t key) {
  return static_cast<int32_t>(static_cast<uint32_t>(key >> 32));
}

int unpackY(int64_t key) {
  return static_cast<int32_t>(static_cast<uint32_t>(key));
}

std::string csvSafe(const std::string & s) {
  std::string out = s;
  std::replace(out.begin(), out.end(), ',', ';');
  return out;
}

std::string joinStrings(const std::vector<std::string> & values, const std::string & sep = "|") {
  std::ostringstream ss;
  for (size_t i = 0; i < values.size(); ++i) {
    if (i > 0) ss << sep;
    ss << values[i];
  }
  return ss.str();
}

struct EdgeDiagnosticSample {
  uint64_t frame_index = 0;
  double timestamp = 0.0;
  int edge_type = 0; // 0: cardinal, 1: diagonal, 2: bridge
  int ix_a = 0, iy_a = 0;
  int ix_b = 0, iy_b = 0;
  int points_a = 0, points_b = 0;
  float min_z_a = 0.0f, max_z_a = 0.0f;
  float min_z_b = 0.0f, max_z_b = 0.0f;
  float interval_z_gap = 0.0f;
  int popcount_a = 0, popcount_b = 0;
  float exact_r_min = 0.0f;
  float tolerant_r_min = 0.0f;
  float exact_jaccard = 0.0f;
  float tolerant_jaccard = 0.0f;
  int directional_support = 0;
  bool baseline_decision = false;
};

struct ClusterDiagnostics {
  double bev_build_ms = 0.0;
  double cc_bfs_ms = 0.0;
  double point_gather_ms = 0.0;
  double filter_ms = 0.0;
  int occupied_cell_count = 0;
  int connected_component_count = 0;
  int valid_cluster_count = 0;
  int tiny_component_count = 0;
  int single_cell_component_count = 0;
  int two_cell_component_count = 0;
  int largest_component_cells = 0;
  int largest_component_points = 0;
  int median_component_cells = 0;
  int median_component_points = 0;
  // Bins: 1, 2, 3-5, 6-10, 11-30, 31+.
  std::array<int, 6> component_cell_histogram{};
  std::array<int, 6> component_point_histogram{};
  int z_rejected_direct_edges = 0;
  int bridge_candidates = 0;
  int bridge_accepted = 0;
  int z_rejected_bridge_edges = 0;
  int layer_cell_count = 0;
  int strong_layer_cell_count = 0;
  int weak_layer_cell_count = 0;
  int max_layers_per_xy_cell = 0;
  double average_layers_per_xy_cell = 0.0;
  int weak_support_candidates = 0;
  int weak_support_accepted = 0;
  int weak_support_density_rejected = 0;
  double z_layer_build_ms = 0.0;
  double density_build_ms = 0.0;

  // Z-Profile Diagnostics
  double z_profile_build_ms = 0.0;
  double z_profile_edge_diag_ms = 0.0;

  int z_profile_cardinal_edges = 0;
  int z_profile_diagonal_edges = 0;
  int z_profile_bridge_edges = 0;

  int z_profile_cardinal_accepted = 0;
  int z_profile_diagonal_accepted = 0;
  int z_profile_bridge_accepted = 0;
  int z_profile_cardinal_rejected = 0;
  int z_profile_diagonal_rejected = 0;
  int z_profile_bridge_rejected = 0;

  // Overlap histograms (10 bins: 0.0-0.1, ..., 0.9-1.0)
  std::array<int, 10> z_profile_cardinal_exact_r_min_hist{};
  std::array<int, 10> z_profile_cardinal_tolerant_r_min_hist{};
  std::array<int, 10> z_profile_cardinal_exact_jaccard_hist{};
  std::array<int, 10> z_profile_cardinal_tolerant_jaccard_hist{};

  std::array<int, 10> z_profile_diagonal_exact_r_min_hist{};
  std::array<int, 10> z_profile_diagonal_tolerant_r_min_hist{};
  std::array<int, 10> z_profile_diagonal_exact_jaccard_hist{};
  std::array<int, 10> z_profile_diagonal_tolerant_jaccard_hist{};

  std::array<int, 10> z_profile_bridge_exact_r_min_hist{};
  std::array<int, 10> z_profile_bridge_tolerant_r_min_hist{};
  std::array<int, 10> z_profile_bridge_exact_jaccard_hist{};
  std::array<int, 10> z_profile_bridge_tolerant_jaccard_hist{};

  // Overlap histograms separated by baseline accepted vs rejected
  std::array<int, 10> z_profile_cardinal_accepted_tol_r_min_hist{};
  std::array<int, 10> z_profile_cardinal_rejected_tol_r_min_hist{};
  std::array<int, 10> z_profile_diagonal_accepted_tol_r_min_hist{};
  std::array<int, 10> z_profile_diagonal_rejected_tol_r_min_hist{};
  std::array<int, 10> z_profile_bridge_accepted_tol_r_min_hist{};
  std::array<int, 10> z_profile_bridge_rejected_tol_r_min_hist{};

  // Z-gap histograms (7 bins: 0, 0-0.1, 0.1-0.2, 0.2-0.4, 0.4-0.8, 0.8-1.2, 1.2+)
  std::array<int, 7> z_profile_cardinal_z_gap_hist{};
  std::array<int, 7> z_profile_diagonal_z_gap_hist{};
  std::array<int, 7> z_profile_bridge_z_gap_hist{};

  // Support histograms
  std::array<int, 3> z_profile_diagonal_support_hist{}; // 0, 1, 2
  std::array<int, 3> z_profile_bridge_axis_support_hist{}; // 0, 1, 2

  // Sums for mean calculation
  double z_profile_cardinal_exact_r_min_sum = 0.0;
  double z_profile_cardinal_tolerant_r_min_sum = 0.0;
  double z_profile_cardinal_exact_jaccard_sum = 0.0;
  double z_profile_cardinal_tolerant_jaccard_sum = 0.0;

  double z_profile_diagonal_exact_r_min_sum = 0.0;
  double z_profile_diagonal_tolerant_r_min_sum = 0.0;
  double z_profile_diagonal_exact_jaccard_sum = 0.0;
  double z_profile_diagonal_tolerant_jaccard_sum = 0.0;

  double z_profile_bridge_exact_r_min_sum = 0.0;
  double z_profile_bridge_tolerant_r_min_sum = 0.0;
  double z_profile_bridge_exact_jaccard_sum = 0.0;
  double z_profile_bridge_tolerant_jaccard_sum = 0.0;

  double z_profile_cardinal_z_gap_sum = 0.0;
  double z_profile_diagonal_z_gap_sum = 0.0;
  double z_profile_bridge_z_gap_sum = 0.0;

  std::vector<EdgeDiagnosticSample> edge_samples;
};

