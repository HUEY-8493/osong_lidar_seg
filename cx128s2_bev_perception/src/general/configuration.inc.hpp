  void declareParams() {
    declare_parameter("general_ground_noise.enabled", true);
    declare_parameter("general_ground_noise.cell_m", 0.5);
    declare_parameter("general_ground_noise.ground_reference_z", 0.0);
    declare_parameter("general_ground_noise.floor_half_band_m", 0.15);
    declare_parameter("general_ground_noise.floor_min_points", 20);
    declare_parameter("general_ground_noise.floor_min_rings", 4);
    declare_parameter("general_ground_noise.floor_max_spread_m", 0.12);
    declare_parameter("general_ground_noise.raised_min_height_m", 0.18);
    declare_parameter("general_ground_noise.raised_max_height_m", 0.8);
    declare_parameter("general_ground_noise.raised_min_points", 3);
    declare_parameter("general_ground_noise.raised_max_rings", 4);
    declare_parameter("general_ground_noise.raised_max_floor_ratio", 0.35);
    declare_parameter("general_ground_noise.raised_top_two_fraction", 0.75);
    declare_parameter("general_ground_noise.body_min_height_m", 0.9);
    declare_parameter("general_ground_noise.body_max_height_m", 5.0);
    declare_parameter("general_ground_noise.body_min_points", 6);
    declare_parameter("general_ground_noise.body_min_rings", 3);
    declare_parameter("general_ground_noise.body_protect_cells", 2);

    declare_parameter("visibility.enabled", true);
    declare_parameter("visibility.sensor_frame", "laser_link");
    // Empty means the installed lslidar_driver/config/lslidar_ch.yaml.
    declare_parameter("visibility.driver_config_file", "");
    declare_parameter("visibility.elevation_min_deg", -12.5);
    declare_parameter("visibility.elevation_max_deg", 12.5);
    declare_parameter("visibility.elevation_bin_deg", 0.125);
    declare_parameter("visibility.physical_max_range_m", 200.0);
    declare_parameter("visibility.range_noise_m", 0.03);
    declare_parameter("visibility.ground_z", 0.0);
    declare_parameter("visibility.publish_markers", true);
    declare_parameter("visibility.adaptive_loss_enabled", false);
    declare_parameter("input_topic", "/sensing/lidar/pointcloud2");
    declare_parameter("use_tf_transform", true);
    declare_parameter("target_frame", "world");
    declare_parameter("tf_timeout_sec", 0.05);
    declare_parameter("fallback_to_raw_frame", false);
    declare_parameter("publish_transformed_cloud", true);
    declare_parameter("transformed_cloud_topic", "/debug/road_frame_points");
    declare_parameter("output_frame_policy", "target_frame");
    declare_parameter("publish_debug_topics", false);
    declare_parameter("publish_recording_frame", false);
    declare_parameter("recording_frame_topic", "/recording/perception_frame");
    declare_parameter("recording_include_full_cloud", true);
    declare_parameter("recording_include_foreground_cloud", false);
    declare_parameter("valid_x_min", -20.0);
    declare_parameter("valid_x_max", 220.0);
    declare_parameter("valid_y_min", -120.0);
    declare_parameter("valid_y_max", 120.0);
    declare_parameter("valid_z_min", -5.0);
    declare_parameter("valid_z_max", 8.0);
    declare_parameter("valid_range_min", 0.2);
    declare_parameter("valid_range_max", 200.0);
    declare_parameter("roi_enabled", false);
    declare_parameter("roi_shape", "polygon");
    declare_parameter("roi_apply_stage", "alert");
    declare_parameter("roi_x_min", 0.0);
    declare_parameter("roi_x_max", 200.0);
    declare_parameter("roi_y_min", -8.0);
    declare_parameter("roi_y_max", 8.0);
    declare_parameter("roi_polygon_x", std::vector<double>{0.0, 200.0, 200.0, 0.0});
    declare_parameter("roi_polygon_y", std::vector<double>{-4.0, -8.0, 8.0, 4.0});
    declare_parameter("roi_file", "/cx128s2_roi.yaml");
    declare_parameter("use_roi_policy", true);
    declare_parameter("roi_policy_file", "/cx128s2_roi_policy.yaml");
    declare_parameter("require_watch_roi", false);
    declare_parameter("discard_points_outside_watch_roi", false);
    declare_parameter("apply_ignore_before_learning", true);
    declare_parameter("publish_roi_policy_markers", true);
    // static: learned static-background subtraction (default).
    // ground_only: no static map; detect points above the per-frame ground estimate.
    declare_parameter("background_mode", "static");
    declare_parameter("require_manual_background_learning", true);
    declare_parameter("background_learning_input_mode", "valid");
    declare_parameter("background_learning_duration_sec", 20.0);
    declare_parameter("background_learning_frames", 200);
    declare_parameter("disable_alerts_during_background_learning", true);
    declare_parameter("background_update_mode", "manual_only");
    declare_parameter("freeze_background_after_learning", true);
    declare_parameter("auto_save_background_after_learning", true);
    declare_parameter("auto_load_background_on_start", false);
    declare_parameter("background_map_file", "/cx128s2_static_background.yaml");
    declare_parameter("background_voxel_size", 0.1);
    declare_parameter("background_grid_x_resolution", 0.5);
    declare_parameter("background_grid_y_resolution", 0.5);
    declare_parameter("background_occupancy_threshold", 0.4);
    declare_parameter("background_clear_threshold", 0.2);
    declare_parameter("background_height_diff_threshold", 0.6);
    declare_parameter("background_std_multiplier", 3.0);
    declare_parameter("gdg_density_threshold", 0.05); // Deprecated / Unused (Legacy parameter)
    declare_parameter("gdg_min_std_z", 0.05);
    declare_parameter("enable_gdg_density_test", false); // Deprecated / Unused (Legacy parameter)
    declare_parameter("min_points_per_background_cell", 1);
    declare_parameter("foreground_min_height_change", 0.35);
    declare_parameter("foreground_count_diff_threshold", 8);
    declare_parameter("foreground_classifier_mode", "statistical_score");
    declare_parameter("foreground_pdf_density_threshold", 0.3);
    declare_parameter("foreground_voxel_diff_threshold", 3);
    declare_parameter("foreground_score_threshold", 1.0);
    declare_parameter("foreground_count_weight", 0.45);
    declare_parameter("foreground_mean_z_weight", 0.25);
    declare_parameter("foreground_max_z_weight", 0.20);
    declare_parameter("foreground_min_z_weight", 0.10);
    declare_parameter("foreground_unknown_cell_score", 1.2);
    declare_parameter("foreground_min_bg_std_z", 0.05);
    declare_parameter("foreground_count_score_clip", 3.0);
    declare_parameter("foreground_z_score_clip", 3.0);
    declare_parameter("foreground_std_multiplier", 3.0);
    declare_parameter("foreground_min_persistence_frames", 3);
    declare_parameter("x_bin_size", 2.0);
    declare_parameter("y_bin_size", 0.5);
    declare_parameter("ground_percentile", 0.15);
    declare_parameter("min_points_per_ground_cell", 3);
    declare_parameter("use_temporal_ground_map", true);
    declare_parameter("ground_map_alpha", 0.05);
    declare_parameter("near_range_max", 20.0);
    declare_parameter("mid_range_max", 45.0);
    declare_parameter("long_range_max", 100.0);
    declare_parameter("ground_margin_near", 0.20);
    declare_parameter("ground_margin_mid", 0.30);
    declare_parameter("ground_margin_long", 0.50);
    declare_parameter("use_fixed_ground_plane", false);
    declare_parameter("fixed_ground_z", 0.0);
    declare_parameter("obstacle_min_height_above_ground", 0.35);
    declare_parameter("obstacle_max_height_above_ground", 5.0);
    declare_parameter("bev_x_resolution", 0.5);
    declare_parameter("bev_y_resolution", 0.25);
    declare_parameter("min_points_per_occupied_cell", 2);
    declare_parameter("enable_radius_outlier_removal", false);
    declare_parameter("skip_ror_when_dense", true);
    declare_parameter("ror_max_input_points", 20000);
    declare_parameter("ror_radius", 0.5);
    declare_parameter("ror_min_neighbors", 2);
    declare_parameter("clustering_method", "bev_connected_component");
    declare_parameter("shape_graph_voxel_size", 0.25);
    declare_parameter("shape_graph_max_edge_dist_near", 1.5);
    declare_parameter("shape_graph_max_edge_dist_mid", 3.0);
    declare_parameter("shape_graph_max_edge_dist_long", 5.0);
    declare_parameter("shape_graph_top_weight", 0.5);
    declare_parameter("shape_graph_side_weight", 0.25);
    declare_parameter("shape_graph_front_weight", 0.25);
    declare_parameter("shape_graph_min_edge_score", 0.6);
    declare_parameter("shape_graph_min_component_nodes_near", 3);
    declare_parameter("shape_graph_min_component_nodes_mid", 2);
    declare_parameter("shape_graph_min_component_nodes_long", 1);
    declare_parameter("shape_graph_use_top_view", true);
    declare_parameter("shape_graph_use_side_view", true);
    declare_parameter("shape_graph_use_front_view", true);
    declare_parameter("cluster_connectivity", 8);
    declare_parameter("cluster_z_guard_enabled", false);
    declare_parameter("cluster_gap_bridging_enabled", false);
    declare_parameter("cluster_neighbor_max_z_gap", 0.8);
    declare_parameter("cluster_bridge_max_z_gap", 0.4);
    declare_parameter("cluster_gap_cells_near", 0);
    declare_parameter("cluster_gap_cells_mid", 1);
    declare_parameter("cluster_gap_cells_long", 1);
    declare_parameter("cluster_edge_diagnostics_enabled", true);
    declare_parameter("cluster_z_layer_enabled", false);
    declare_parameter("cluster_z_layer_resolution", 0.3);
    declare_parameter("cluster_weak_support_enabled", false);
    declare_parameter("cluster_density_enabled", false);
    declare_parameter("cluster_density_window_cells", 1);
    declare_parameter("cluster_weak_support_min_density", 4);
    declare_parameter("cluster_z_profile_diagnostics_enabled", false);
    declare_parameter("cluster_z_profile_bin_size", 0.30);
    declare_parameter("cluster_input_mode", "foreground");
    declare_parameter("min_cluster_cells_near", 3);
    declare_parameter("min_cluster_cells_mid", 2);
    declare_parameter("min_cluster_cells_long", 1);
    declare_parameter("min_cluster_points_near", 8);
    declare_parameter("min_cluster_points_mid", 4);
    declare_parameter("min_cluster_points_long", 2);
    declare_parameter("vehicle_detector_enabled", true);
    declare_parameter("vehicle_require_entry_direction", false);
    declare_parameter("vehicle_fixed_yaw_rad", 0.0);
    declare_parameter("raw_vehicle_box_refinement_enabled", false);
    declare_parameter("raw_vehicle_box_margin_m", 0.10);
    declare_parameter("raw_vehicle_box_max_objects", 4);
    declare_parameter("raw_vehicle_box_min_foreground_ratio", 0.25);
    declare_parameter("raw_vehicle_box_max_centroid_shift_m", 0.35);
    declare_parameter("vehicle_min_height", 0.6);
    declare_parameter("vehicle_max_height", 4.5);
    declare_parameter("vehicle_min_width", 1.0);
    declare_parameter("vehicle_max_width", 4.0);
    declare_parameter("vehicle_max_length", 15.0);
    declare_parameter("vehicle_min_score", 0.60);
    declare_parameter("human_like_min_height", 0.8);
    declare_parameter("human_like_max_height", 2.2);
    declare_parameter("human_like_max_width", 1.2);
    declare_parameter("human_like_max_length", 1.2);
    declare_parameter("crane_like_min_height", 3.0);
    declare_parameter("crane_like_min_vertical_extent", 2.5);
    declare_parameter("crane_like_max_speed", 0.3);
    declare_parameter("unknown_obstacle_detector_enabled", true);
    declare_parameter("unknown_min_persistence_frames", 3);
    declare_parameter("unknown_min_dynamic_score", 0.4);
    declare_parameter("enable_tracking", true);
    declare_parameter("tracking_filter_mode", "legacy");
    declare_parameter("kf_process_accel_std_mps2", 1.5);
    declare_parameter("kf_measurement_position_std_m", 0.10);
    declare_parameter("kf_initial_position_std_m", 0.10);
    declare_parameter("kf_initial_velocity_std_mps", 1.0);
    declare_parameter("kf_min_dt_sec", 0.01);
    declare_parameter("kf_max_dt_sec", 0.5);
    declare_parameter("kf_debug", false);
    declare_parameter("visibility_box_prior_weight", 0.75);
    declare_parameter("enable_shape_memory_tracking", true);
    declare_parameter("shape_memory_match_weight", 0.5);
    declare_parameter("shape_memory_distance_weight", 0.5);
    declare_parameter("shape_memory_min_match_score", 0.35);
    declare_parameter("shape_memory_update_alpha", 0.2);
    declare_parameter("shape_memory_keep_while_lost", true);
    declare_parameter("shape_memory_min_update_points", 20);
    declare_parameter("shape_memory_min_update_nodes", 3);
    declare_parameter("shape_memory_dimension_shrink_limit", 0.15);
    declare_parameter("shape_memory_max_lost_frames_near", 20);
    declare_parameter("shape_memory_max_lost_frames_mid", 35);
    declare_parameter("shape_memory_max_lost_frames_long", 60);
    declare_parameter("shape_memory_learning_mode", "manual_commit");
    declare_parameter("shape_memory_freeze_after_commit", true);
    declare_parameter("shape_memory_allow_auto_update_after_commit", false);
    declare_parameter("shape_memory_allow_dimension_shrink", false);
    declare_parameter("shape_memory_commit_min_nodes", 5);
    declare_parameter("shape_memory_commit_min_edges", 4);
    declare_parameter("shape_memory_commit_min_confidence", 0.5);
    declare_parameter("shape_memory_edge_missing_decay", 0.02);
    declare_parameter("shape_memory_node_missing_decay", 0.02);
    declare_parameter("shape_memory_min_confidence_to_use", 0.25);
    declare_parameter("shape_memory_max_endpoint_relations", 12);
    declare_parameter("shape_graph_compute_policy", "manual_prepare");
    declare_parameter("shape_memory_clear_observed_cache_after_commit", true);
    declare_parameter("shape_memory_compare_voxel_size_near", 0.50);
    declare_parameter("shape_memory_compare_voxel_size_mid", 0.80);
    declare_parameter("shape_memory_compare_voxel_size_long", 1.20);
    declare_parameter("shape_memory_compare_max_points", 300);
    declare_parameter("shape_memory_max_match_nodes", 80);
    declare_parameter("shape_memory_max_match_edges", 120);
    declare_parameter("shape_memory_max_match_relations", 12);
    declare_parameter("shape_memory_reprepare_enabled", true);
    declare_parameter("shape_memory_reprepare_match_threshold", 0.25);
    declare_parameter("shape_memory_reprepare_bad_frames", 5);
    declare_parameter("object_memory_bank_enabled", true);
    declare_parameter("object_memory_bank_file", "/cx128s2_object_memory_bank.yaml");
    declare_parameter("auto_load_object_memory_bank_on_start", true);
    declare_parameter("auto_save_object_memory_bank_after_commit", true);
    declare_parameter("object_memory_bank_attach_threshold", 0.45);
    declare_parameter("shape_memory_mirror_mode_enabled", true);
    declare_parameter("shape_memory_mirror_plane", "xz");
    declare_parameter("shape_memory_mirror_center_policy", "bbox_center");
    declare_parameter("shape_memory_mirror_axis", "local_y");
    declare_parameter("shape_memory_mirror_min_observed_width_ratio", 0.35);
    declare_parameter("shape_memory_mirror_confidence_scale", 0.55);
    declare_parameter("shape_memory_mirror_virtual_node_confidence", 0.45);
    declare_parameter("shape_memory_mirror_virtual_edge_confidence", 0.40);
    declare_parameter("shape_memory_mirror_min_observed_nodes", 5);
    declare_parameter("shape_memory_mirror_only_after_prepare_ready", true);
    declare_parameter("shape_memory_mirror_warn_if_near_identical", true);
    declare_parameter("shape_memory_mirror_debug_samples", 5);
    declare_parameter(
        "shape_memory_mirror_only_for_class_labels",
        std::vector<std::string>{"TRUCK", "VEHICLE", "TRUCK_LIKE", "VEHICLE_LIKE"});
    declare_parameter("fem_assisted_clustering_enabled", false);
    declare_parameter("fem_cluster_min_match_score", 0.45);
    declare_parameter("fem_cluster_body_margin", 0.6);
    declare_parameter("fem_cluster_edge_support_dist", 0.5);
    declare_parameter("fem_cluster_node_support_dist", 0.6);
    declare_parameter("fem_cluster_absorb_orphan_points", false);
    declare_parameter("fem_cluster_split_attached_unknown", true);
    declare_parameter("fem_cluster_split_cargo", true);
    declare_parameter("cargo_detection_enabled", false);
    declare_parameter("cargo_height_delta_threshold", 0.5);
    declare_parameter("cargo_score_threshold", 0.5);
    declare_parameter("max_tracking_candidates", 80);
    declare_parameter("max_active_tracks", 100);
    declare_parameter("tentative_confirm_frames", 5);
    declare_parameter("tentative_confirm_hits", 3);
    declare_parameter("max_lost_frames_near", 15);
    declare_parameter("max_lost_frames_mid", 25);
    declare_parameter("max_lost_frames_long", 40);
    declare_parameter("vehicle_reacquire_enabled", true);
    declare_parameter("vehicle_lost_memory_sec", 4.0);
    declare_parameter("vehicle_identity_memory_sec", 36000.0);
    declare_parameter("gate_threshold_near", 9.0);
    declare_parameter("gate_threshold_mid", 12.0);
    declare_parameter("gate_threshold_long", 16.0);
    declare_parameter("motion_speed_threshold", 0.3);
    declare_parameter("direction_dominance_ratio", 1.5);
    declare_parameter("vehicle_stopped_duration_sec", 10.0);
    declare_parameter("keep_box_while_lost", true);
    declare_parameter("predicted_track_marker_alpha", 0.18);
    declare_parameter("enable_risk_assessment", false);
    declare_parameter("enable_static_structure_filter", true);
    declare_parameter("static_velocity_threshold", 0.25);
    declare_parameter("static_persistence_frames", 5);
    declare_parameter("wall_length_min", 4.0);
    declare_parameter("wall_width_max", 0.8);
    declare_parameter("wall_elongation_ratio_min", 4.0);
    declare_parameter("roi_filtered_points_topic", "/debug/roi_filtered_points");
    declare_parameter("publish_debug_roi_filtered_points", true);
    declare_parameter("publish_debug_transformed_cloud", false);
    declare_parameter("publish_debug_valid_points", true);
    declare_parameter("publish_debug_foreground_points", true);
    declare_parameter("publish_debug_static_background_points", false);
    declare_parameter("publish_debug_clusters", true);
    declare_parameter("publish_shape_graph_markers", true);
    declare_parameter("publish_shape_memory_markers", true);
    declare_parameter("shape_graph_marker_lifetime", 0.5);
    declare_parameter("debug_cloud_publish_period_sec", 0.2);
    declare_parameter("debug_cloud_max_points", 50000);
    declare_parameter("publish_diagnostics_table", true);
    declare_parameter("diagnostics_table_period_sec", 1.0);
    declare_parameter("diagnostics_table_topic", "/debug/perception_diagnostics_table");
    declare_parameter("print_diagnostics_table_to_console", true);
    declare_parameter("background_table_file", "/tmp/cx128s2_background_table.csv");
    declare_parameter("track_table_file", "/tmp/cx128s2_track_table.csv");
    declare_parameter("candidate_table_file", "/tmp/cx128s2_candidate_table.csv");
    declare_parameter("save_csv_on_shutdown", true);
    declare_parameter("marker_lifetime", 0.5);
    declare_parameter("enable_parallel_processing", false);
    declare_parameter("parallel_threads", 2);

    // Crane Beam Work Detection Mode Parameters
    declare_parameter("perception_mode", "default");
    declare_parameter("crane_pipeline", "legacy");
    declare_parameter("crane_slope.azimuth_columns", 666);
    declare_parameter("crane_slope.lidar_channels", 128);
    declare_parameter("crane_slope.max_slope_deg", 10.0);
    // Original source names are available; old max_slope_deg remains an alias.
    declare_parameter("crane_slope.max_slope_degree", get_parameter("crane_slope.max_slope_deg").as_double());
    declare_parameter("crane_slope.lidar_model", "cx128s2");
    declare_parameter("crane_slope.lidar_degree_start", -15.0);
    declare_parameter("crane_slope.lidar_degree_step", 1.0);
    declare_parameter("crane_slope.lidar_height", 0.0);
    declare_parameter("crane_slope.cx128s2_prism_offset_deg", 0.0);
    declare_parameter("crane_slope.cx128s2_prism_angles_deg", std::vector<double>{0.0,-0.17,-0.34,-0.51});
    declare_parameter("crane_slope.model_angle_tolerance_deg", 0.25);
    declare_parameter("crane_slope.max_vertical_height", 0.08);
    declare_parameter("crane_slope.ground_z", 0.0);
    declare_parameter("crane_slope.force_ground_below_z", -std::numeric_limits<double>::infinity());
    declare_parameter("crane_slope.include_ground_in_clustering", false);
    declare_parameter("crane_slope.publish_debug_clouds", false);
    declare_parameter("crane_slope.publish_colored_cloud", false);
    declare_parameter("crane_slope.horizontal_enabled", false);
    declare_parameter("crane_slope.horizontal_max_azimuth_gap_deg", 0.5);
    declare_parameter("crane_slope.publish_direction_clouds", false);
    declare_parameter("crane_slope.boundary_clustering.enabled", false);
    declare_parameter("crane_slope.boundary_clustering.slope_threshold_deg", 35.0);
    declare_parameter("crane_slope.boundary_clustering.height_step_threshold", 0.12);
    declare_parameter("crane_slope.boundary_clustering.range_jump_threshold", 0.35);
    declare_parameter("crane_slope.boundary_clustering.max_neighbor_distance", 0.40);
    declare_parameter("crane_slope.boundary_clustering.min_height_above_ground", 0.05);
    declare_parameter("crane_slope.boundary_clustering.min_cluster_points", 10);
    declare_parameter("crane_slope.boundary_clustering.max_cluster_points", 10000);
    declare_parameter("crane_slope.boundary_clustering.include_unknown", true);
    declare_parameter("crane_slope.boundary_clustering.enable_morphological_closing", false);
    declare_parameter("crane_slope.boundary_clustering.morph_closing_radius_m", 0.15);
    declare_parameter("crane_slope.z_gaussian_clustering.enabled", false);
    declare_parameter("crane_slope.z_gaussian_clustering.sigma_z", 0.025);
    declare_parameter("crane_slope.z_gaussian_clustering.range_sigma_factor", 0.001);
    declare_parameter("crane_slope.z_gaussian_clustering.confidence_sigma_multiplier", 3.0);
    declare_parameter("crane_slope.z_gaussian_clustering.step_sigma_multiplier", 2.5);
    declare_parameter("crane_slope.z_gaussian_clustering.max_layer_z_spread", 0.25);
    declare_parameter("crane_slope.z_gaussian_clustering.min_height_above_ground", 0.05);
    declare_parameter("crane_slope.z_gaussian_clustering.min_layer_points", 5);
    declare_parameter("crane_slope.closed_loop_grouping.enabled", false);
    declare_parameter("crane_slope.closed_loop_grouping.grid_resolution_m", 0.08);
    declare_parameter("crane_slope.closed_loop_grouping.closing_radius_m", 0.16);
    declare_parameter("crane_slope.closed_loop_grouping.min_containment_ratio", 0.50);
    declare_parameter("crane_slope.closed_loop_grouping.min_enclosed_cells", 2);
    declare_parameter("crane_slope.closed_loop_grouping.use_boundary_points_for_loop", true);
    declare_parameter("crane_slope.closed_loop_grouping.subcluster_match_xy_gate", 0.60);
    declare_parameter("crane_slope.closed_loop_grouping.subcluster_match_z_gate", 0.40);
    declare_parameter("crane_slope.roi_filter.enabled", true);
    declare_parameter("crane_slope.roi_filter.only_crane_roi", true);
    declare_parameter("crane_slope.roi_filter.active_include_background", false);
    declare_parameter("crane_slope.roi_filter.idle_exclude_beam_roi", false);
    declare_parameter("crane_slope.roi_filter.expand_beam_roi", true);
    declare_parameter("crane_slope.work_status.fsm_enabled", true);
    declare_parameter("crane_slope.work_status.boundary_motion_enabled", false);
    declare_parameter("crane_slope.work_status.boundary_temporal_enabled", false);
    declare_parameter("crane_slope.work_status.boundary_start_confirm_sec", 0.5);
    declare_parameter("crane_slope.work_status.boundary_stop_confirm_sec", 1.5);
    declare_parameter("crane_slope.work_status.boundary_support_timeout_sec", 2.5);
    declare_parameter("crane_slope.work_status.cargo_roi_width_x_m", 1.20);
    declare_parameter("crane_slope.work_status.cargo_roi_width_y_m", 1.20);
    declare_parameter("crane_slope.work_status.cargo_start_offset_m", 0.15);
    declare_parameter("crane_slope.work_status.virtual_hook_offset_z_m", 0.5);
    declare_parameter("crane_slope.work_status.cargo_confirm_sec", 2.0);
    declare_parameter("crane_slope.work_status.cargo_loss_sec", 1.3);
    declare_parameter("crane_slope.work_status.lifting_event_hold_sec", 0.5);
    declare_parameter("crane_slope.work_status.lifting_clearance_delta_m", 0.0);
    declare_parameter("crane_slope.work_status.lifting_min_clearance_m", 0.0);
    declare_parameter("crane_slope.work_status.cargo_min_surface_width_m", 0.15);
    declare_parameter("crane_slope.work_status.cargo_min_fill_ratio", 0.35);
    declare_parameter("crane_slope.work_status.boundary_search_radius_m", 0.60);
    declare_parameter("crane_slope.work_status.boundary_min_coherent_points", 4);
    declare_parameter("crane_slope.work_status.enabled", true);
    declare_parameter("crane_slope.work_status.grid_resolution_m", 0.05);
    declare_parameter("crane_slope.work_status.size_ratio_k", 1.5);
    declare_parameter("crane_slope.work_status.connect_xy_margin", 0.60);
    declare_parameter("crane_slope.work_status.connect_max_gap_z", 2.50);
    declare_parameter("crane_slope.work_status.min_candidate_points", 20);
    declare_parameter("crane_slope.work_status.temporal_window_n", 10);
    declare_parameter("crane_slope.work_status.confirm_m_on", 5);
    declare_parameter("crane_slope.work_status.confirm_m_off", 2);
    declare_parameter("crane_slope.temporal_stabilization.enabled", false);
    declare_parameter("crane_slope.temporal_stabilization.mode", "sliding_window");
    declare_parameter("crane_slope.temporal_stabilization.window_size", 5);
    declare_parameter("crane_slope.temporal_stabilization.voxel_size_xy", 0.04);
    declare_parameter("crane_slope.temporal_stabilization.voxel_size_z", 0.03);
    declare_parameter("crane_slope.temporal_stabilization.min_hit_count", 2);
    declare_parameter("crane_slope.temporal_stabilization.temporal_stabilization_enabled", true);
    declare_parameter("crane_slope.temporal_stabilization.stabilization_alpha", 0.20);
    declare_parameter("crane_slope.temporal_stabilization.jump_tolerance_length", 0.50);
    declare_parameter("crane_slope.temporal_stabilization.jump_tolerance_width", 0.25);
    declare_parameter("crane_slope.temporal_stabilization.jump_confirm_frames", 3);
    declare_parameter("crane_slope.temporal_stabilization.max_hold_missed_frames", 3);
    declare_parameter("crane_slope.temporal_stabilization.node_confirm_frames", 2);
    declare_parameter("crane_slope.temporal_stabilization.node_max_hold_frames", 2);
    declare_parameter("crane_slope.temporal_stabilization.largest_swap_confirm_frames", 3);
    declare_parameter("crane_slope.temporal_stabilization.largest_match_dist", 0.50);
    declare_parameter("crane_slope.temporal_stabilization.yaw_jump_tolerance_deg", 30.0);
    declare_parameter("crane_slope.temporal_stabilization.yaw_alpha", 0.15);
    declare_parameter("crane_slope.top_down_hook.max_slope_angle_deg", 35.0);
    declare_parameter("crane_slope.top_down_hook.max_turn_angle_deg", 40.0);
    declare_parameter("crane_slope.fixed_root.enabled", true);
    declare_parameter("crane_slope.fixed_root.left_roi.x_min", -1.25);
    declare_parameter("crane_slope.fixed_root.left_roi.x_max", -0.55);
    declare_parameter("crane_slope.fixed_root.left_roi.y_min", -0.25);
    declare_parameter("crane_slope.fixed_root.left_roi.y_max", 0.35);
    declare_parameter("crane_slope.fixed_root.left_roi.z_min", 4.80);
    declare_parameter("crane_slope.fixed_root.left_roi.z_max", 5.70);
    declare_parameter("crane_slope.fixed_root.right_roi.x_min", 0.72);
    declare_parameter("crane_slope.fixed_root.right_roi.x_max", 1.42);
    declare_parameter("crane_slope.fixed_root.right_roi.y_min", -0.25);
    declare_parameter("crane_slope.fixed_root.right_roi.y_max", 0.35);
    declare_parameter("crane_slope.fixed_root.right_roi.z_min", 4.80);
    declare_parameter("crane_slope.fixed_root.right_roi.z_max", 5.80);
    declare_parameter("crane_slope.fixed_root.min_points", 15);
    declare_parameter("crane_slope.fixed_root.hold_frames", 3);
    declare_parameter("crane_slope.fixed_root.motion_speed_threshold", 0.05);
    declare_parameter("crane_slope.fixed_root.motion_disp_threshold", 0.06);
    declare_parameter("crane_slope.fixed_root.motion_confirm_frames", 2);
    declare_parameter("crane_slope.connection_stabilization.enabled", true);
    declare_parameter("crane_slope.connection_stabilization.history_frames", 10);
    declare_parameter("crane_slope.connection_stabilization.min_direct_frames", 4);
    declare_parameter("crane_slope.connection_stabilization.min_transition_count", 0);
    declare_parameter("crane_slope.connection_stabilization.missing_hold_frames", 3);
    declare_parameter("crane_slope.connection_stabilization.spatial_continuity_enabled", true);
    declare_parameter("crane_slope.connection_stabilization.spatial_min_direct_frames", 1);
    declare_parameter("crane_slope.connection_stabilization.spatial_max_gap_z", 0.80);
    declare_parameter("crane_slope.connection_stabilization.spatial_max_dxy", 0.35);
    declare_parameter("crane_slope.dual_roi_anchor.enabled", true);
    declare_parameter("crane_slope.dual_roi_anchor.publish_markers", true);
    declare_parameter("crane_slope.dual_roi_anchor.fixed_roi_size_xy", 0.36);
    declare_parameter("crane_slope.dual_roi_anchor.fixed_roi_size_z", 0.40);
    declare_parameter("crane_slope.dual_roi_anchor.moving_roi_size_xy", 0.60);
    declare_parameter("crane_slope.dual_roi_anchor.moving_roi_size_z", 0.56);
    declare_parameter("crane_slope.dual_roi_anchor.confirm_frames", 3);
    declare_parameter("crane_slope.dual_roi_anchor.min_direct_evidence_frames", 2);
    declare_parameter("crane_slope.dual_roi_anchor.max_lock_slope_deg", 28.0);
    declare_parameter("crane_slope.dual_roi_anchor.max_lock_turn_deg", 30.0);
    declare_parameter("crane_slope.dual_roi_anchor.noise_deadband_xy", 0.06);
    declare_parameter("crane_slope.dual_roi_anchor.noise_deadband_z", 0.08);
    declare_parameter("crane_slope.dual_roi_anchor.relocate_thresh_xy", 0.10);
    declare_parameter("crane_slope.dual_roi_anchor.relocate_thresh_z", 0.12);
    declare_parameter("crane_slope.dual_roi_anchor.relocate_confirm_frames", 3);
    declare_parameter("crane_slope.dual_roi_anchor.anchor_memory_frames", 30);
    declare_parameter("crane_slope.dual_roi_anchor.beam_intrusion_guard_enabled", true);
    declare_parameter("crane_slope.dual_roi_anchor.beam_min_length_m", 0.80);
    declare_parameter("crane_slope.dual_roi_anchor.beam_min_aspect_ratio", 2.2);
    declare_parameter("crane_slope.dual_roi_anchor.intrusion_hold_frames", 50);

    // Virtual Rope Centerline & 2-Strand Wire Bundle
    declare_parameter("crane_slope.virtual_rope.enabled", true);
    declare_parameter("crane_slope.virtual_rope.wire_min_spacing_m", 0.10);
    declare_parameter("crane_slope.virtual_rope.wire_max_spacing_m", 0.45);
    declare_parameter("crane_slope.virtual_rope.wire_merge_span_m", 0.16);
    declare_parameter("crane_slope.virtual_rope.memory_frames", 30);
    declare_parameter("crane_slope.virtual_rope.publish_markers", true);

    // Sheave Detection
    declare_parameter("crane_slope.sheave.track_continuity_enabled", false);
    declare_parameter("crane_slope.sheave.learned_reference_file", std::string(""));
    declare_parameter("crane_slope.sheave.reference_matching_enabled", false);
    declare_parameter<std::vector<std::string>>("crane_slope.sheave.reference_ids", std::vector<std::string>{});
    declare_parameter<std::vector<std::string>>("crane_slope.sheave.reference_sources", std::vector<std::string>{});
    declare_parameter<std::vector<std::string>>("crane_slope.sheave.reference_labels", std::vector<std::string>{});
    declare_parameter<std::vector<int64_t>>("crane_slope.sheave.reference_families", std::vector<int64_t>{});
    declare_parameter<std::vector<double>>("crane_slope.sheave.reference_features", std::vector<double>{});
    declare_parameter<std::vector<double>>("crane_slope.sheave.reference_local_features", std::vector<double>{});
    declare_parameter<std::vector<double>>("crane_slope.sheave.debug_seed_world", std::vector<double>{});
    declare_parameter("crane_slope.sheave.enabled", true);
    declare_parameter("crane_slope.sheave.min_radius_m", 0.08);
    declare_parameter("crane_slope.sheave.max_radius_m", 0.28);
    declare_parameter("crane_slope.sheave.default_radius_m", 0.155);
    declare_parameter("crane_slope.sheave.min_arc_points", 12);
    declare_parameter("crane_slope.sheave.max_circle_rms_m", 0.055);
    declare_parameter("crane_slope.sheave.max_sheave_width_m", 0.55);
    declare_parameter("crane_slope.sheave.confirm_frames", 2);
    declare_parameter("crane_slope.sheave.surface_gap_m", 0.08);
    declare_parameter("crane_slope.sheave.terminate_chain_at_sheave", true);
    declare_parameter("crane_slope.sheave.publish_markers", true);

    // Z-axis Temporal Median Filter
    declare_parameter("crane_slope.z_temporal_median.enabled", true);
    declare_parameter("crane_slope.z_temporal_median.window_frames", 3);
    declare_parameter("crane_slope.exception_gate.enabled", false);
    declare_parameter("crane_slope.exception_gate.min_length_m", 0.80);
    declare_parameter("crane_slope.exception_gate.min_area_m2", 0.15);
    declare_parameter("crane_slope.exception_gate.use_visibility", true);
    declare_parameter("crane_slope.exception_gate.max_occlusion_fraction", 0.40);
    declare_parameter("crane_slope.exception_gate.scan_all_chain_nodes", true);
    declare_parameter("crane_plane.enabled", false);
    declare_parameter("crane_plane.max_neighbor_distance", 0.25);
    declare_parameter("crane_plane.max_depth_jump", 0.25);
    declare_parameter("crane_plane.depth_jump_factor", 4.0);
    declare_parameter("crane_plane.max_normal_angle_deg", 15.0);
    declare_parameter("crane_plane.max_growth_residual", 0.04);
    declare_parameter("crane_plane.min_candidate_points", 8);
    declare_parameter("crane_plane.min_line_length", 0.15);
    declare_parameter("crane_plane.min_plane_length", 0.20);
    declare_parameter("crane_plane.min_plane_width", 0.04);
    declare_parameter("crane_plane.max_rms", 0.025);
    declare_parameter("crane_plane.residual_quantile", 0.95);
    declare_parameter("crane_plane.max_quantile_residual", 0.05);
    declare_parameter("crane_plane.max_thickness_ratio", 0.15);
    declare_parameter("crane_plane.horizontal_angle_deg", 10.0);
    declare_parameter("crane_plane.vertical_angle_deg", 10.0);
    declare_parameter("crane_plane.allow_single_direction_extension", true);
    declare_parameter("crane_plane.local_radius", 2);
    declare_parameter("crane_plane.refit_interval", 16);
    declare_parameter("crane_plane.publish_debug_clouds", false);
    declare_parameter("crane_plane.publish_colored_cloud", false);
    declare_parameter("height_surface.enabled", false);
    declare_parameter("height_surface.z_bin_size", 0.05);
    declare_parameter("height_surface.max_neighbor_dz", 0.05);
    declare_parameter("height_surface.max_surface_z_spread", 0.08);
    declare_parameter("height_surface.max_xy_neighbor_distance", 0.15);
    declare_parameter("height_surface.min_surface_points", 20);
    declare_parameter("height_surface.min_surface_length", 0.20);
    declare_parameter("height_surface.min_surface_width", 0.02);
    declare_parameter("height_surface.publish_debug", false);
    declare_parameter("crane_beam_enabled", true);
    declare_parameter("crane_beam.motion_reference_mode", "auto");
    declare_parameter("crane_beam.beam_roi_x_min", -2.0);
    declare_parameter("crane_beam.beam_roi_x_max", 2.0);
    declare_parameter("crane_beam.beam_roi_y_min", -2.0);
    declare_parameter("crane_beam.beam_roi_y_max", 2.0);
    declare_parameter("crane_beam.beam_roi_z_min", -0.5);
    declare_parameter("crane_beam.beam_roi_z_max", 7.5);
    declare_parameter("crane_beam.background_roi_x_min", -8.0);
    declare_parameter("crane_beam.background_roi_x_max", 8.0);
    declare_parameter("crane_beam.background_roi_y_min", -8.0);
    declare_parameter("crane_beam.background_roi_y_max", 8.0);
    declare_parameter("crane_beam.background_roi_z_min", -0.5);
    declare_parameter("crane_beam.background_roi_z_max", 7.5);
    declare_parameter("crane_beam.min_beam_roi_points", 10);
    declare_parameter("crane_beam.min_background_roi_points", 20);
    declare_parameter("crane_beam.min_beam_cluster_points", 15);
    declare_parameter("crane_beam.beam_z_band", 0.30);
    declare_parameter("crane_beam.lift_dz_threshold", 0.05);
    declare_parameter("crane_beam.hook_beam_dz_match_tolerance", 0.03);
    declare_parameter("crane_beam.lower_dz_threshold", 0.05);
    declare_parameter("crane_beam.stop_dz_threshold", 0.02);
    declare_parameter("crane_beam.ground_tolerance", 0.05);
    declare_parameter("crane_beam.crane_move_start_x", 0.10);
    declare_parameter("crane_beam.crane_move_start_y", 0.10);
    declare_parameter("crane_beam.crane_move_stop_x", 0.02);
    declare_parameter("crane_beam.crane_move_stop_y", 0.05);
    declare_parameter("crane_beam.crane_move_stop_frames", 5);
    declare_parameter("crane_beam.lift_confirm_frames", 3);
    declare_parameter("crane_beam.suspend_confirm_frames", 3);
    declare_parameter("crane_beam.lower_confirm_frames", 3);
    declare_parameter("crane_beam.placed_confirm_frames", 3);
    declare_parameter("crane_beam.complete_confirm_frames", 5);
    declare_parameter("crane_beam.beam_measurement_max_lost_frames", 3);
    declare_parameter("crane_beam.length_stable_threshold", 0.10);
    declare_parameter("crane_beam.length_lock_frames", 5);
    declare_parameter("crane_beam.cluster_bev_res", 0.05);
    declare_parameter("crane_beam.continuity_gate_xy", 2.5);
    declare_parameter("crane_beam.beam_z_histogram_bin_size", 0.05);
    declare_parameter("crane_beam.beam_ground_clearance", 0.12);
    declare_parameter("crane_beam.tracking_gate_xy", 1.2);
    declare_parameter("crane_beam.tracking_gate_z", 0.50);
    declare_parameter("crane_beam.tracking_gate_yaw", 0.35);
    declare_parameter("crane_beam.tracking_gate_length", 1.0);
    declare_parameter("crane_beam.support_search_margin_xy", 0.60);
    declare_parameter("crane_beam.min_support_points", 15);
    declare_parameter("crane_beam.placement_tolerance", 0.10);
    declare_parameter("crane_beam.beam_default_height", 0.40);
    declare_parameter("crane_beam.landing_height_gap_threshold", 0.45);
    declare_parameter("crane_beam.use_z_ema", true);
    declare_parameter("crane_beam.z_ema_alpha", 0.3);
    declare_parameter("crane_beam.publish_debug", true);
    declare_parameter("crane_beam.publish_background_roi_marker", false);
    declare_parameter("crane_beam.diagnostics_period_sec", 1.0);

    // Hardening: Cumulative vertical motion
    declare_parameter("crane_beam.cumulative_motion_enabled", true);
    declare_parameter("crane_beam.cumulative_lift_distance", 0.12);
    declare_parameter("crane_beam.cumulative_lower_distance", 0.12);
    declare_parameter("crane_beam.cumulative_motion_min_frames", 4);
    declare_parameter("crane_beam.cumulative_direction_epsilon", 0.003);

    // Hardening: Support surface planar extent filtering
    declare_parameter("crane_beam.support_min_xy_extent", 0.60);
    declare_parameter("crane_beam.support_max_z_spread", 0.08);
    declare_parameter("crane_beam.support_temporal_continuity", true);

    // Hardening: Tilted touchdown multi-segment evaluation
    declare_parameter("crane_beam.tilt_segmentation_enabled", true);
    declare_parameter("crane_beam.tilt_num_segments", 3);
    declare_parameter("crane_beam.min_supported_segment_ratio", 0.60);

    // Hardening: Initial cluster shape gating
    declare_parameter("crane_beam.min_initial_cluster_length", 2.0);
    declare_parameter("crane_beam.min_initial_cluster_elongation", 1.8);
    declare_parameter("crane_beam.beam_high_z_gate_enabled", true);
    declare_parameter("crane_beam.beam_high_z_reject_threshold", 1.0);
    declare_parameter("crane_beam.beam_high_z_require_cluster_valid", true);

    // Sensor transform freeze (disables dynamic Gazebo moving TF compensation)
    declare_parameter("crane_beam.freeze_sensor_transform", true);

    // Crane Hook measurements & auxiliary prepare_ready parameters
    declare_parameter("crane_beam.hook.enabled", true);
    declare_parameter("crane_beam.hook.reference_file", "");
    declare_parameter("crane_beam.hook.reference_min_frames", 30);
    declare_parameter("crane_beam.hook.roi_left_x_min", -1.5);
    declare_parameter("crane_beam.hook.roi_left_x_max", -0.3);
    declare_parameter("crane_beam.hook.roi_right_x_min", 0.3);
    declare_parameter("crane_beam.hook.roi_right_x_max", 1.5);
    declare_parameter("crane_beam.hook.roi_y_min", -0.8);
    declare_parameter("crane_beam.hook.roi_y_max", 0.8);
    declare_parameter("crane_beam.hook.roi_z_min", 3.0);
    declare_parameter("crane_beam.hook.roi_z_max", 6.2);
    declare_parameter("crane_beam.hook.window_size_z", 0.30);
    declare_parameter("crane_beam.hook.window_step_z", 0.02);
    declare_parameter("crane_beam.hook.min_body_points", 300);
    declare_parameter("crane_beam.hook.tracking_enabled", true);
    declare_parameter("crane_beam.hook.tracking_confirm_frames", 3);
    declare_parameter("crane_beam.hook.tracking_gate_xy", 0.35);
    declare_parameter("crane_beam.hook.tracking_gate_z", 0.45);
    declare_parameter("crane_beam.hook.tracking_gate_z_max", 0.80);
    declare_parameter("crane_beam.hook.tracking_max_lost_frames", 5);
    declare_parameter("crane_beam.hook.tracking_prediction_timeout_sec", 0.75);
    declare_parameter("crane_beam.hook.tracking_accel_std_xy", 0.5);
    declare_parameter("crane_beam.hook.tracking_accel_std_z", 1.5);
    declare_parameter("crane_beam.hook.tracking_measurement_std", 0.03);
    declare_parameter("crane_beam.hook.ground_search_z_min", -0.5);
    declare_parameter("crane_beam.hook.ground_search_z_max", 0.8);
    declare_parameter("crane_beam.hook.ground_bin_size", 0.05);
    declare_parameter("crane_beam.hook.min_ground_points", 50);
    declare_parameter("crane_beam.hook.prepare_distance", 4.0);
    declare_parameter("crane_beam.hook.prepare_release_margin", 0.20);
    declare_parameter("crane_beam.hook.prepare_confirm_frames", 1);
  }

  void loadParams() {
    general_ground_noise_config_.enabled = get_parameter("general_ground_noise.enabled").as_bool();
    general_ground_noise_config_.cell_m = get_parameter("general_ground_noise.cell_m").as_double();
    general_ground_noise_config_.ground_reference_z = get_parameter("general_ground_noise.ground_reference_z").as_double();
    general_ground_noise_config_.floor_half_band_m = get_parameter("general_ground_noise.floor_half_band_m").as_double();
    general_ground_noise_config_.floor_min_points = get_parameter("general_ground_noise.floor_min_points").as_int();
    general_ground_noise_config_.floor_min_rings = get_parameter("general_ground_noise.floor_min_rings").as_int();
    general_ground_noise_config_.floor_max_spread_m = get_parameter("general_ground_noise.floor_max_spread_m").as_double();
    general_ground_noise_config_.raised_min_height_m = get_parameter("general_ground_noise.raised_min_height_m").as_double();
    general_ground_noise_config_.raised_max_height_m = get_parameter("general_ground_noise.raised_max_height_m").as_double();
    general_ground_noise_config_.raised_min_points = get_parameter("general_ground_noise.raised_min_points").as_int();
    general_ground_noise_config_.raised_max_rings = get_parameter("general_ground_noise.raised_max_rings").as_int();
    general_ground_noise_config_.raised_max_floor_ratio = get_parameter("general_ground_noise.raised_max_floor_ratio").as_double();
    general_ground_noise_config_.raised_top_two_fraction = get_parameter("general_ground_noise.raised_top_two_fraction").as_double();
    general_ground_noise_config_.body_min_height_m = get_parameter("general_ground_noise.body_min_height_m").as_double();
    general_ground_noise_config_.body_max_height_m = get_parameter("general_ground_noise.body_max_height_m").as_double();
    general_ground_noise_config_.body_min_points = get_parameter("general_ground_noise.body_min_points").as_int();
    general_ground_noise_config_.body_min_rings = get_parameter("general_ground_noise.body_min_rings").as_int();
    general_ground_noise_config_.body_protect_cells = get_parameter("general_ground_noise.body_protect_cells").as_int();
    general_ground_noise_config_.validate();
    input_topic_ = get_parameter("input_topic").as_string();
    use_tf_transform_ = get_parameter("use_tf_transform").as_bool();
    target_frame_ = get_parameter("target_frame").as_string();
    tf_timeout_sec_ = get_parameter("tf_timeout_sec").as_double();
    fallback_to_raw_frame_ = get_parameter("fallback_to_raw_frame").as_bool();
    publish_transformed_cloud_ = get_parameter("publish_transformed_cloud").as_bool();
    transformed_cloud_topic_ = get_parameter("transformed_cloud_topic").as_string();
    output_frame_policy_ = get_parameter("output_frame_policy").as_string();
    publish_debug_topics_ = get_parameter("publish_debug_topics").as_bool();
    publish_recording_frame_ = get_parameter("publish_recording_frame").as_bool();
    recording_frame_topic_ = get_parameter("recording_frame_topic").as_string();
    recording_include_full_cloud_ = get_parameter("recording_include_full_cloud").as_bool();
    recording_include_foreground_cloud_ = get_parameter("recording_include_foreground_cloud").as_bool();
    valid_x_min_ = get_parameter("valid_x_min").as_double();
    valid_x_max_ = get_parameter("valid_x_max").as_double();
    valid_y_min_ = get_parameter("valid_y_min").as_double();
    valid_y_max_ = get_parameter("valid_y_max").as_double();
    valid_z_min_ = get_parameter("valid_z_min").as_double();
    valid_z_max_ = get_parameter("valid_z_max").as_double();
    valid_range_min_ = get_parameter("valid_range_min").as_double();
    valid_range_max_ = get_parameter("valid_range_max").as_double();
    roi_enabled_ = get_parameter("roi_enabled").as_bool();
    roi_shape_ = get_parameter("roi_shape").as_string();
    roi_apply_stage_ = get_parameter("roi_apply_stage").as_string();
    roi_x_min_ = get_parameter("roi_x_min").as_double();
    roi_x_max_ = get_parameter("roi_x_max").as_double();
    roi_y_min_ = get_parameter("roi_y_min").as_double();
    roi_y_max_ = get_parameter("roi_y_max").as_double();
    roi_polygon_x_ = get_parameter("roi_polygon_x").as_double_array();
    roi_polygon_y_ = get_parameter("roi_polygon_y").as_double_array();
    try {
      roi_file_ = resolvePath(get_parameter("roi_file").as_string());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Failed to resolve roi_file: %s", e.what());
      throw;
    }
    use_roi_policy_ = get_parameter("use_roi_policy").as_bool();
    try {
      roi_policy_file_ = resolvePath(get_parameter("roi_policy_file").as_string());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Failed to resolve roi_policy_file: %s", e.what());
      throw;
    }
    require_watch_roi_ = get_parameter("require_watch_roi").as_bool();
    discard_points_outside_watch_roi_ = get_parameter("discard_points_outside_watch_roi").as_bool();
    apply_ignore_before_learning_ = get_parameter("apply_ignore_before_learning").as_bool();
    publish_roi_policy_markers_ = get_parameter("publish_roi_policy_markers").as_bool();
    background_mode_ = get_parameter("background_mode").as_string();
    if (background_mode_ != "static" && background_mode_ != "ground_only") {
      RCLCPP_WARN(get_logger(), "Unknown background_mode='%s'; using 'static'.",
                  background_mode_.c_str());
      background_mode_ = "static";
    }
    require_manual_background_learning_ = get_parameter("require_manual_background_learning").as_bool();
    background_learning_input_mode_ = get_parameter("background_learning_input_mode").as_string();
    background_learning_duration_sec_ = get_parameter("background_learning_duration_sec").as_double();
    background_learning_frames_ = get_parameter("background_learning_frames").as_int();
    disable_alerts_during_background_learning_ =
        get_parameter("disable_alerts_during_background_learning").as_bool();
    background_update_mode_ = get_parameter("background_update_mode").as_string();
    freeze_background_after_learning_ = get_parameter("freeze_background_after_learning").as_bool();
    auto_save_background_after_learning_ = get_parameter("auto_save_background_after_learning").as_bool();
    auto_load_background_on_start_ = get_parameter("auto_load_background_on_start").as_bool();
    try {
      background_map_file_ = resolvePath(get_parameter("background_map_file").as_string());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Failed to resolve background_map_file: %s", e.what());
      throw;
    }
    voxel_size_ = get_parameter("background_voxel_size").as_double();
    bg_x_res_ = get_parameter("background_grid_x_resolution").as_double();
    bg_y_res_ = get_parameter("background_grid_y_resolution").as_double();
    background_occupancy_threshold_ = get_parameter("background_occupancy_threshold").as_double();
    background_height_diff_threshold_ = get_parameter("background_height_diff_threshold").as_double();
    background_std_multiplier_ = get_parameter("background_std_multiplier").as_double();
    gdg_density_threshold_ = get_parameter("gdg_density_threshold").as_double(); // Deprecated / Unused (Legacy parameter)
    gdg_min_std_z_ = get_parameter("gdg_min_std_z").as_double();
    enable_gdg_density_test_ = get_parameter("enable_gdg_density_test").as_bool(); // Deprecated / Unused (Legacy parameter)
    min_points_per_background_cell_ = get_parameter("min_points_per_background_cell").as_int();
    foreground_min_height_change_ = get_parameter("foreground_min_height_change").as_double();
    foreground_count_diff_threshold_ = get_parameter("foreground_count_diff_threshold").as_int();
    foreground_classifier_mode_ = get_parameter("foreground_classifier_mode").as_string();
    foreground_pdf_density_threshold_ = get_parameter("foreground_pdf_density_threshold").as_double();
    foreground_voxel_diff_threshold_ = get_parameter("foreground_voxel_diff_threshold").as_int();
    foreground_score_threshold_ = get_parameter("foreground_score_threshold").as_double();
    foreground_count_weight_ = get_parameter("foreground_count_weight").as_double();
    foreground_mean_z_weight_ = get_parameter("foreground_mean_z_weight").as_double();
    foreground_max_z_weight_ = get_parameter("foreground_max_z_weight").as_double();
    foreground_min_z_weight_ = get_parameter("foreground_min_z_weight").as_double();
    foreground_unknown_cell_score_ = get_parameter("foreground_unknown_cell_score").as_double();
    foreground_min_bg_std_z_ = get_parameter("foreground_min_bg_std_z").as_double();
    foreground_count_score_clip_ = get_parameter("foreground_count_score_clip").as_double();
    foreground_z_score_clip_ = get_parameter("foreground_z_score_clip").as_double();
    foreground_std_multiplier_ = get_parameter("foreground_std_multiplier").as_double();
    foreground_min_persistence_frames_ = get_parameter("foreground_min_persistence_frames").as_int();
    x_bin_size_ = get_parameter("x_bin_size").as_double();
    y_bin_size_ = get_parameter("y_bin_size").as_double();
    ground_percentile_ = get_parameter("ground_percentile").as_double();
    min_points_per_ground_cell_ = get_parameter("min_points_per_ground_cell").as_int();
    use_temporal_ground_map_ = get_parameter("use_temporal_ground_map").as_bool();
    ground_map_alpha_ = get_parameter("ground_map_alpha").as_double();
    near_range_max_ = get_parameter("near_range_max").as_double();
    mid_range_max_ = get_parameter("mid_range_max").as_double();
    long_range_max_ = get_parameter("long_range_max").as_double();
    ground_margin_near_ = get_parameter("ground_margin_near").as_double();
    ground_margin_mid_ = get_parameter("ground_margin_mid").as_double();
    ground_margin_long_ = get_parameter("ground_margin_long").as_double();
    use_fixed_ground_plane_ = get_parameter("use_fixed_ground_plane").as_bool();
    fixed_ground_z_ = get_parameter("fixed_ground_z").as_double();
    obstacle_min_height_above_ground_ = get_parameter("obstacle_min_height_above_ground").as_double();
    obstacle_max_height_above_ground_ = get_parameter("obstacle_max_height_above_ground").as_double();
    bev_x_res_ = get_parameter("bev_x_resolution").as_double();
    bev_y_res_ = get_parameter("bev_y_resolution").as_double();
    min_points_per_occupied_cell_ = get_parameter("min_points_per_occupied_cell").as_int();
    enable_radius_outlier_removal_ = get_parameter("enable_radius_outlier_removal").as_bool();
    skip_ror_when_dense_ = get_parameter("skip_ror_when_dense").as_bool();
    ror_max_input_points_ = get_parameter("ror_max_input_points").as_int();
    ror_radius_ = get_parameter("ror_radius").as_double();
    ror_min_neighbors_ = get_parameter("ror_min_neighbors").as_int();
    clustering_method_ = get_parameter("clustering_method").as_string();
    shape_graph_voxel_size_ = get_parameter("shape_graph_voxel_size").as_double();
    shape_graph_max_edge_dist_near_ = get_parameter("shape_graph_max_edge_dist_near").as_double();
    shape_graph_max_edge_dist_mid_ = get_parameter("shape_graph_max_edge_dist_mid").as_double();
    shape_graph_max_edge_dist_long_ = get_parameter("shape_graph_max_edge_dist_long").as_double();
    shape_graph_top_weight_ = get_parameter("shape_graph_top_weight").as_double();
    shape_graph_side_weight_ = get_parameter("shape_graph_side_weight").as_double();
    shape_graph_front_weight_ = get_parameter("shape_graph_front_weight").as_double();
    shape_graph_min_edge_score_ = get_parameter("shape_graph_min_edge_score").as_double();
    shape_graph_min_component_nodes_near_ = get_parameter("shape_graph_min_component_nodes_near").as_int();
    shape_graph_min_component_nodes_mid_ = get_parameter("shape_graph_min_component_nodes_mid").as_int();
    shape_graph_min_component_nodes_long_ = get_parameter("shape_graph_min_component_nodes_long").as_int();
    shape_graph_use_top_view_ = get_parameter("shape_graph_use_top_view").as_bool();
    shape_graph_use_side_view_ = get_parameter("shape_graph_use_side_view").as_bool();
    shape_graph_use_front_view_ = get_parameter("shape_graph_use_front_view").as_bool();
    cluster_connectivity_ = get_parameter("cluster_connectivity").as_int();
    cluster_z_guard_enabled_ = get_parameter("cluster_z_guard_enabled").as_bool();
    cluster_gap_bridging_enabled_ = get_parameter("cluster_gap_bridging_enabled").as_bool();
    cluster_neighbor_max_z_gap_ = get_parameter("cluster_neighbor_max_z_gap").as_double();
    cluster_bridge_max_z_gap_ = get_parameter("cluster_bridge_max_z_gap").as_double();
    cluster_gap_cells_near_ = get_parameter("cluster_gap_cells_near").as_int();
    cluster_gap_cells_mid_ = get_parameter("cluster_gap_cells_mid").as_int();
    cluster_gap_cells_long_ = get_parameter("cluster_gap_cells_long").as_int();
    cluster_edge_diagnostics_enabled_ = get_parameter("cluster_edge_diagnostics_enabled").as_bool();
    cluster_z_layer_enabled_ = get_parameter("cluster_z_layer_enabled").as_bool();
    cluster_z_layer_resolution_ = get_parameter("cluster_z_layer_resolution").as_double();
    cluster_weak_support_enabled_ = get_parameter("cluster_weak_support_enabled").as_bool();
    cluster_density_enabled_ = get_parameter("cluster_density_enabled").as_bool();
    cluster_density_window_cells_ = get_parameter("cluster_density_window_cells").as_int();
    cluster_weak_support_min_density_ = get_parameter("cluster_weak_support_min_density").as_int();
    cluster_z_profile_diagnostics_enabled_ = get_parameter("cluster_z_profile_diagnostics_enabled").as_bool();
    cluster_z_profile_bin_size_ = get_parameter("cluster_z_profile_bin_size").as_double();
    cluster_input_mode_ = get_parameter("cluster_input_mode").as_string();
    min_cluster_cells_near_ = get_parameter("min_cluster_cells_near").as_int();
    min_cluster_cells_mid_ = get_parameter("min_cluster_cells_mid").as_int();
    min_cluster_cells_long_ = get_parameter("min_cluster_cells_long").as_int();
    min_cluster_points_near_ = get_parameter("min_cluster_points_near").as_int();
    min_cluster_points_mid_ = get_parameter("min_cluster_points_mid").as_int();
    min_cluster_points_long_ = get_parameter("min_cluster_points_long").as_int();
    vehicle_detector_enabled_ = get_parameter("vehicle_detector_enabled").as_bool();
    vehicle_require_entry_direction_ = get_parameter("vehicle_require_entry_direction").as_bool();
    raw_vehicle_box_refinement_enabled_ = get_parameter("raw_vehicle_box_refinement_enabled").as_bool();
    raw_vehicle_box_margin_m_ = get_parameter("raw_vehicle_box_margin_m").as_double();
    raw_vehicle_box_max_objects_ = get_parameter("raw_vehicle_box_max_objects").as_int();
    raw_vehicle_box_min_foreground_ratio_ = get_parameter("raw_vehicle_box_min_foreground_ratio").as_double();
    raw_vehicle_box_max_centroid_shift_m_ = get_parameter("raw_vehicle_box_max_centroid_shift_m").as_double();
    if (!std::isfinite(raw_vehicle_box_margin_m_) || raw_vehicle_box_margin_m_ < 0.0 || raw_vehicle_box_margin_m_ > 0.5 ||
        raw_vehicle_box_max_objects_ < 1 || raw_vehicle_box_max_objects_ > 32 ||
        !std::isfinite(raw_vehicle_box_min_foreground_ratio_) || raw_vehicle_box_min_foreground_ratio_ <= 0.0 ||
        raw_vehicle_box_min_foreground_ratio_ > 1.0 || !std::isfinite(raw_vehicle_box_max_centroid_shift_m_) ||
        raw_vehicle_box_max_centroid_shift_m_ <= 0.0 || raw_vehicle_box_max_centroid_shift_m_ > 1.0)
      throw std::invalid_argument("invalid raw_vehicle_box refinement parameters");
    vehicle_fixed_yaw_rad_ = get_parameter("vehicle_fixed_yaw_rad").as_double();
    if (!std::isfinite(vehicle_fixed_yaw_rad_)) {
      throw std::runtime_error("vehicle_fixed_yaw_rad must be finite");
    }
    vehicle_min_height_ = get_parameter("vehicle_min_height").as_double();
    vehicle_max_height_ = get_parameter("vehicle_max_height").as_double();
    vehicle_min_width_ = get_parameter("vehicle_min_width").as_double();
    vehicle_max_width_ = get_parameter("vehicle_max_width").as_double();
    vehicle_max_length_ = get_parameter("vehicle_max_length").as_double();
    vehicle_min_score_ = get_parameter("vehicle_min_score").as_double();
    human_like_min_height_ = get_parameter("human_like_min_height").as_double();
    human_like_max_height_ = get_parameter("human_like_max_height").as_double();
    human_like_max_width_ = get_parameter("human_like_max_width").as_double();
    human_like_max_length_ = get_parameter("human_like_max_length").as_double();
    crane_like_min_height_ = get_parameter("crane_like_min_height").as_double();
    crane_like_min_vertical_extent_ = get_parameter("crane_like_min_vertical_extent").as_double();
    crane_like_max_speed_ = get_parameter("crane_like_max_speed").as_double();
    unknown_obstacle_detector_enabled_ = get_parameter("unknown_obstacle_detector_enabled").as_bool();
    unknown_min_persistence_frames_ = get_parameter("unknown_min_persistence_frames").as_int();
    unknown_min_dynamic_score_ = get_parameter("unknown_min_dynamic_score").as_double();
    enable_tracking_ = get_parameter("enable_tracking").as_bool();
    tracking_filter_mode_ = get_parameter("tracking_filter_mode").as_string();
    if (tracking_filter_mode_ != "legacy" && tracking_filter_mode_ != "kalman") {
      RCLCPP_WARN(get_logger(), "Unknown tracking_filter_mode='%s'; using legacy.", tracking_filter_mode_.c_str());
      tracking_filter_mode_ = "legacy";
    }
    kf_process_accel_std_mps2_ = get_parameter("kf_process_accel_std_mps2").as_double();
    kf_measurement_position_std_m_ = get_parameter("kf_measurement_position_std_m").as_double();
    kf_initial_position_std_m_ = get_parameter("kf_initial_position_std_m").as_double();
    kf_initial_velocity_std_mps_ = get_parameter("kf_initial_velocity_std_mps").as_double();
    kf_min_dt_sec_ = get_parameter("kf_min_dt_sec").as_double();
    kf_max_dt_sec_ = get_parameter("kf_max_dt_sec").as_double();
    kf_debug_ = get_parameter("kf_debug").as_bool();
    visibility_box_prior_weight_ = get_parameter("visibility_box_prior_weight").as_double();
    if (!std::isfinite(visibility_box_prior_weight_) || visibility_box_prior_weight_ < 0.0 ||
        visibility_box_prior_weight_ > 1.0) {
      throw std::runtime_error("visibility_box_prior_weight must be in [0, 1]");
    }
    if (kf_min_dt_sec_ <= 0.0 || kf_max_dt_sec_ < kf_min_dt_sec_ ||
        kf_process_accel_std_mps2_ <= 0.0 || kf_measurement_position_std_m_ <= 0.0 ||
        kf_initial_position_std_m_ <= 0.0 || kf_initial_velocity_std_mps_ <= 0.0) {
      throw std::runtime_error("Invalid Kalman filter parameter range.");
    }
    enable_shape_memory_tracking_ = get_parameter("enable_shape_memory_tracking").as_bool();
    shape_memory_match_weight_ = get_parameter("shape_memory_match_weight").as_double();
    shape_memory_distance_weight_ = get_parameter("shape_memory_distance_weight").as_double();
    shape_memory_min_match_score_ = get_parameter("shape_memory_min_match_score").as_double();
    shape_memory_update_alpha_ = get_parameter("shape_memory_update_alpha").as_double();
    shape_memory_keep_while_lost_ = get_parameter("shape_memory_keep_while_lost").as_bool();
    shape_memory_min_update_points_ = get_parameter("shape_memory_min_update_points").as_int();
    shape_memory_min_update_nodes_ = get_parameter("shape_memory_min_update_nodes").as_int();
    shape_memory_dimension_shrink_limit_ = get_parameter("shape_memory_dimension_shrink_limit").as_double();
    shape_memory_max_lost_frames_near_ = get_parameter("shape_memory_max_lost_frames_near").as_int();
    shape_memory_max_lost_frames_mid_ = get_parameter("shape_memory_max_lost_frames_mid").as_int();
    shape_memory_max_lost_frames_long_ = get_parameter("shape_memory_max_lost_frames_long").as_int();
    try {
      shape_memory_learning_mode_ = get_parameter("shape_memory_learning_mode").as_string();
      shape_memory_freeze_after_commit_ = get_parameter("shape_memory_freeze_after_commit").as_bool();
      shape_memory_allow_auto_update_after_commit_ = get_parameter("shape_memory_allow_auto_update_after_commit").as_bool();
      shape_memory_allow_dimension_shrink_ = get_parameter("shape_memory_allow_dimension_shrink").as_bool();
      shape_memory_commit_min_nodes_ = get_parameter("shape_memory_commit_min_nodes").as_int();
      shape_memory_commit_min_edges_ = get_parameter("shape_memory_commit_min_edges").as_int();
      shape_memory_commit_min_confidence_ = get_parameter("shape_memory_commit_min_confidence").as_double();
      shape_memory_edge_missing_decay_ = get_parameter("shape_memory_edge_missing_decay").as_double();
      shape_memory_node_missing_decay_ = get_parameter("shape_memory_node_missing_decay").as_double();
      shape_memory_min_confidence_to_use_ = get_parameter("shape_memory_min_confidence_to_use").as_double();
      shape_memory_max_endpoint_relations_ = get_parameter("shape_memory_max_endpoint_relations").as_int();
      shape_graph_compute_policy_ = get_parameter("shape_graph_compute_policy").as_string();
      shape_memory_clear_observed_cache_after_commit_ =
          get_parameter("shape_memory_clear_observed_cache_after_commit").as_bool();
      shape_memory_compare_voxel_size_near_ =
          get_parameter("shape_memory_compare_voxel_size_near").as_double();
      shape_memory_compare_voxel_size_mid_ =
          get_parameter("shape_memory_compare_voxel_size_mid").as_double();
      shape_memory_compare_voxel_size_long_ =
          get_parameter("shape_memory_compare_voxel_size_long").as_double();
      shape_memory_compare_max_points_ = get_parameter("shape_memory_compare_max_points").as_int();
      shape_memory_max_match_nodes_ = get_parameter("shape_memory_max_match_nodes").as_int();
      shape_memory_max_match_edges_ = get_parameter("shape_memory_max_match_edges").as_int();
      shape_memory_max_match_relations_ = get_parameter("shape_memory_max_match_relations").as_int();
      shape_memory_reprepare_enabled_ = get_parameter("shape_memory_reprepare_enabled").as_bool();
      shape_memory_reprepare_match_threshold_ =
          get_parameter("shape_memory_reprepare_match_threshold").as_double();
      shape_memory_reprepare_bad_frames_ = get_parameter("shape_memory_reprepare_bad_frames").as_int();
      object_memory_bank_enabled_ = get_parameter("object_memory_bank_enabled").as_bool();
      object_memory_bank_file_ = resolvePath(get_parameter("object_memory_bank_file").as_string());
      auto_load_object_memory_bank_on_start_ =
          get_parameter("auto_load_object_memory_bank_on_start").as_bool();
      auto_save_object_memory_bank_after_commit_ =
          get_parameter("auto_save_object_memory_bank_after_commit").as_bool();
      object_memory_bank_attach_threshold_ =
          get_parameter("object_memory_bank_attach_threshold").as_double();
      shape_memory_mirror_mode_enabled_ =
          get_parameter("shape_memory_mirror_mode_enabled").as_bool();
      shape_memory_mirror_plane_ = get_parameter("shape_memory_mirror_plane").as_string();
      shape_memory_mirror_center_policy_ =
          get_parameter("shape_memory_mirror_center_policy").as_string();
      shape_memory_mirror_axis_ = get_parameter("shape_memory_mirror_axis").as_string();
      if (shape_memory_mirror_plane_.empty()) {
        shape_memory_mirror_plane_ = mirrorPlaneFromDeprecatedAxis(shape_memory_mirror_axis_);
        RCLCPP_WARN(
            get_logger(),
            "[DEPRECATED] shape_memory_mirror_axis is deprecated. Use shape_memory_mirror_plane instead. "
            "%s axis mirror is converted to %s plane mirror.",
            shape_memory_mirror_axis_.c_str(), shape_memory_mirror_plane_.c_str());
      }
      shape_memory_mirror_min_observed_width_ratio_ =
          get_parameter("shape_memory_mirror_min_observed_width_ratio").as_double();
      shape_memory_mirror_confidence_scale_ =
          get_parameter("shape_memory_mirror_confidence_scale").as_double();
      shape_memory_mirror_virtual_node_confidence_ =
          get_parameter("shape_memory_mirror_virtual_node_confidence").as_double();
      shape_memory_mirror_virtual_edge_confidence_ =
          get_parameter("shape_memory_mirror_virtual_edge_confidence").as_double();
      shape_memory_mirror_min_observed_nodes_ =
          get_parameter("shape_memory_mirror_min_observed_nodes").as_int();
      shape_memory_mirror_only_after_prepare_ready_ =
          get_parameter("shape_memory_mirror_only_after_prepare_ready").as_bool();
      shape_memory_mirror_warn_if_near_identical_ =
          get_parameter("shape_memory_mirror_warn_if_near_identical").as_bool();
      shape_memory_mirror_debug_samples_ =
          get_parameter("shape_memory_mirror_debug_samples").as_int();
      shape_memory_mirror_only_for_class_labels_ =
          get_parameter("shape_memory_mirror_only_for_class_labels").as_string_array();
      fem_assisted_clustering_enabled_ =
          get_parameter("fem_assisted_clustering_enabled").as_bool();
      fem_cluster_min_match_score_ = get_parameter("fem_cluster_min_match_score").as_double();
      fem_cluster_body_margin_ = get_parameter("fem_cluster_body_margin").as_double();
      fem_cluster_edge_support_dist_ = get_parameter("fem_cluster_edge_support_dist").as_double();
      fem_cluster_node_support_dist_ = get_parameter("fem_cluster_node_support_dist").as_double();
      fem_cluster_absorb_orphan_points_ = get_parameter("fem_cluster_absorb_orphan_points").as_bool();
      fem_cluster_split_attached_unknown_ =
          get_parameter("fem_cluster_split_attached_unknown").as_bool();
      fem_cluster_split_cargo_ = get_parameter("fem_cluster_split_cargo").as_bool();
      cargo_detection_enabled_ = get_parameter("cargo_detection_enabled").as_bool();
      cargo_height_delta_threshold_ = get_parameter("cargo_height_delta_threshold").as_double();
      cargo_score_threshold_ = get_parameter("cargo_score_threshold").as_double();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Failed to resolve shape memory tracking parameter: %s", e.what());
      throw;
    }
    max_tracking_candidates_ = get_parameter("max_tracking_candidates").as_int();
    max_active_tracks_ = get_parameter("max_active_tracks").as_int();
    tentative_confirm_frames_ = get_parameter("tentative_confirm_frames").as_int();
    tentative_confirm_hits_ = get_parameter("tentative_confirm_hits").as_int();
    max_lost_frames_near_ = get_parameter("max_lost_frames_near").as_int();
    max_lost_frames_mid_ = get_parameter("max_lost_frames_mid").as_int();
    max_lost_frames_long_ = get_parameter("max_lost_frames_long").as_int();
    vehicle_reacquire_enabled_ = get_parameter("vehicle_reacquire_enabled").as_bool();
    vehicle_lost_memory_sec_ = std::max(0.1,
        get_parameter("vehicle_lost_memory_sec").as_double());
    vehicle_identity_memory_sec_ = std::max(1.0,
        get_parameter("vehicle_identity_memory_sec").as_double());
    gate_threshold_near_ = get_parameter("gate_threshold_near").as_double();
    gate_threshold_mid_ = get_parameter("gate_threshold_mid").as_double();
    gate_threshold_long_ = get_parameter("gate_threshold_long").as_double();
    motion_speed_threshold_ = get_parameter("motion_speed_threshold").as_double();
    direction_dominance_ratio_ = get_parameter("direction_dominance_ratio").as_double();
    vehicle_stopped_duration_sec_ = get_parameter("vehicle_stopped_duration_sec").as_double();
    keep_box_while_lost_ = get_parameter("keep_box_while_lost").as_bool();
    predicted_track_marker_alpha_ = get_parameter("predicted_track_marker_alpha").as_double();
    enable_risk_assessment_ = get_parameter("enable_risk_assessment").as_bool();
    enable_static_structure_filter_ = get_parameter("enable_static_structure_filter").as_bool();
    static_velocity_threshold_ = get_parameter("static_velocity_threshold").as_double();
    static_persistence_frames_ = get_parameter("static_persistence_frames").as_int();
    wall_length_min_ = get_parameter("wall_length_min").as_double();
    wall_width_max_ = get_parameter("wall_width_max").as_double();
    wall_elongation_ratio_min_ = get_parameter("wall_elongation_ratio_min").as_double();
    roi_filtered_points_topic_ = get_parameter("roi_filtered_points_topic").as_string();
    publish_debug_roi_filtered_points_ = get_parameter("publish_debug_roi_filtered_points").as_bool();
    publish_debug_transformed_cloud_ = get_parameter("publish_debug_transformed_cloud").as_bool();
    publish_debug_valid_points_ = get_parameter("publish_debug_valid_points").as_bool();
    publish_debug_foreground_points_ = get_parameter("publish_debug_foreground_points").as_bool();
    publish_debug_static_background_points_ = get_parameter("publish_debug_static_background_points").as_bool();
    publish_debug_clusters_ = get_parameter("publish_debug_clusters").as_bool();
    publish_shape_graph_markers_ = get_parameter("publish_shape_graph_markers").as_bool();
    publish_shape_memory_markers_ = get_parameter("publish_shape_memory_markers").as_bool();
    shape_graph_marker_lifetime_ = get_parameter("shape_graph_marker_lifetime").as_double();
    debug_cloud_publish_period_sec_ = get_parameter("debug_cloud_publish_period_sec").as_double();
    debug_cloud_max_points_ = get_parameter("debug_cloud_max_points").as_int();
    publish_diagnostics_table_ = get_parameter("publish_diagnostics_table").as_bool();
    diagnostics_table_period_sec_ = get_parameter("diagnostics_table_period_sec").as_double();
    diagnostics_table_topic_ = get_parameter("diagnostics_table_topic").as_string();
    print_diagnostics_table_to_console_ = get_parameter("print_diagnostics_table_to_console").as_bool();
    try {
      background_table_file_ = resolvePath(get_parameter("background_table_file").as_string());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Failed to resolve background_table_file: %s", e.what());
      throw;
    }
    try {
      track_table_file_ = resolvePath(get_parameter("track_table_file").as_string());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Failed to resolve track_table_file: %s", e.what());
      throw;
    }
    try {
      candidate_table_file_ = resolvePath(get_parameter("candidate_table_file").as_string());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Failed to resolve candidate_table_file: %s", e.what());
      throw;
    }
    save_csv_on_shutdown_ = get_parameter("save_csv_on_shutdown").as_bool();
    marker_lifetime_ = get_parameter("marker_lifetime").as_double();
    enable_parallel_processing_ = get_parameter("enable_parallel_processing").as_bool();
    parallel_threads_ = get_parameter("parallel_threads").as_int();

    // Load Crane Beam parameters
    perception_mode_ = get_parameter("perception_mode").as_string();
    crane_pipeline_name_ = get_parameter("crane_pipeline").as_string();
    crane_slope_publish_debug_clouds_ = get_parameter("crane_slope.publish_debug_clouds").as_bool();
    crane_slope_publish_colored_cloud_ = get_parameter("crane_slope.publish_colored_cloud").as_bool();
    crane_slope_publish_direction_clouds_ = get_parameter("crane_slope.publish_direction_clouds").as_bool();
    crane_plane_publish_debug_clouds_ = get_parameter("crane_plane.publish_debug_clouds").as_bool();
    crane_plane_publish_colored_cloud_ = get_parameter("crane_plane.publish_colored_cloud").as_bool();
    crane_height_publish_debug_=get_parameter("height_surface.publish_debug").as_bool();
    if (crane_pipeline_name_ != "legacy" && crane_pipeline_name_ != "slope" && crane_pipeline_name_ != "height_surface")
      throw std::invalid_argument("crane_pipeline must be legacy, slope or height_surface");
    if (crane_pipeline_name_ == "slope" || crane_pipeline_name_ == "height_surface") {
      if (perception_mode_ != "crane_beam_work" || target_frame_ != "world" || !use_tf_transform_)
        throw std::invalid_argument("Crane experimental pipelines require crane_beam_work, world frame and TF");
      cx128s2_bev_perception::CraneSlopeConfig c;
      if (crane_pipeline_name_=="height_surface") {
        c.height_mode=true;
        c.height.enabled=get_parameter("height_surface.enabled").as_bool();
        c.height.patch_resolution=get_parameter("height_surface.z_bin_size").as_double();
        c.height.max_neighbor_dz=get_parameter("height_surface.max_neighbor_dz").as_double();
        c.height.max_patch_z_spread=get_parameter("height_surface.max_surface_z_spread").as_double();
        c.height.max_xy_gap=get_parameter("height_surface.max_xy_neighbor_distance").as_double();
        c.height.min_surface_points=get_parameter("height_surface.min_surface_points").as_int();
        c.height.min_surface_length=get_parameter("height_surface.min_surface_length").as_double();
        c.height.min_surface_width=get_parameter("height_surface.min_surface_width").as_double();
        crane_height_enabled_=c.height.enabled;
      } else {
      c.azimuth_columns = get_parameter("crane_slope.azimuth_columns").as_int();
      c.lidar_channels = get_parameter("crane_slope.lidar_channels").as_int();
      c.max_slope_deg = get_parameter("crane_slope.max_slope_degree").as_double();
      c.lidar_model = get_parameter("crane_slope.lidar_model").as_string();
      c.lidar_degree_start = get_parameter("crane_slope.lidar_degree_start").as_double();
      c.lidar_degree_step = get_parameter("crane_slope.lidar_degree_step").as_double();
      c.lidar_height = get_parameter("crane_slope.lidar_height").as_double();
      c.cx128s2_prism_offset_deg = get_parameter("crane_slope.cx128s2_prism_offset_deg").as_double();
      c.cx128s2_prism_angles_deg = get_parameter("crane_slope.cx128s2_prism_angles_deg").as_double_array();
      c.model_angle_tolerance_deg = get_parameter("crane_slope.model_angle_tolerance_deg").as_double();
      c.max_vertical_height = get_parameter("crane_slope.max_vertical_height").as_double();
      c.ground_z = get_parameter("crane_slope.ground_z").as_double();
      c.force_ground_below_z = get_parameter("crane_slope.force_ground_below_z").as_double();
      c.include_ground_in_clustering = get_parameter("crane_slope.include_ground_in_clustering").as_bool();
      c.horizontal_enabled = get_parameter("crane_slope.horizontal_enabled").as_bool();
      c.horizontal_max_azimuth_gap_deg = get_parameter("crane_slope.horizontal_max_azimuth_gap_deg").as_double();
      c.boundary_clustering.enabled = get_parameter("crane_slope.boundary_clustering.enabled").as_bool();
      c.boundary_clustering.slope_threshold_deg = get_parameter("crane_slope.boundary_clustering.slope_threshold_deg").as_double();
      c.boundary_clustering.height_step_threshold = get_parameter("crane_slope.boundary_clustering.height_step_threshold").as_double();
      c.boundary_clustering.range_jump_threshold = get_parameter("crane_slope.boundary_clustering.range_jump_threshold").as_double();
      c.boundary_clustering.max_neighbor_distance = get_parameter("crane_slope.boundary_clustering.max_neighbor_distance").as_double();
      c.boundary_clustering.min_height_above_ground = get_parameter("crane_slope.boundary_clustering.min_height_above_ground").as_double();
      c.boundary_clustering.min_cluster_points = get_parameter("crane_slope.boundary_clustering.min_cluster_points").as_int();
      c.boundary_clustering.max_cluster_points = get_parameter("crane_slope.boundary_clustering.max_cluster_points").as_int();
      c.boundary_clustering.include_unknown = get_parameter("crane_slope.boundary_clustering.include_unknown").as_bool();
      c.boundary_clustering.enable_morphological_closing = get_parameter("crane_slope.boundary_clustering.enable_morphological_closing").as_bool();
      c.boundary_clustering.morph_closing_radius_m = get_parameter("crane_slope.boundary_clustering.morph_closing_radius_m").as_double();
      crane_boundary_clustering_enabled_ = c.boundary_clustering.enabled;
      c.z_gaussian.enabled = get_parameter("crane_slope.z_gaussian_clustering.enabled").as_bool();
      c.z_gaussian.sigma_z = get_parameter("crane_slope.z_gaussian_clustering.sigma_z").as_double();
      c.z_gaussian.range_sigma_factor = get_parameter("crane_slope.z_gaussian_clustering.range_sigma_factor").as_double();
      c.z_gaussian.confidence_sigma_multiplier = get_parameter("crane_slope.z_gaussian_clustering.confidence_sigma_multiplier").as_double();
      c.z_gaussian.step_sigma_multiplier = get_parameter("crane_slope.z_gaussian_clustering.step_sigma_multiplier").as_double();
      c.z_gaussian.max_layer_z_spread = get_parameter("crane_slope.z_gaussian_clustering.max_layer_z_spread").as_double();
      c.z_gaussian.min_height_above_ground = get_parameter("crane_slope.z_gaussian_clustering.min_height_above_ground").as_double();
      c.z_gaussian.min_layer_points = get_parameter("crane_slope.z_gaussian_clustering.min_layer_points").as_int();
      crane_z_gaussian_clustering_enabled_ = c.z_gaussian.enabled;
      c.closed_loop.enabled = get_parameter("crane_slope.closed_loop_grouping.enabled").as_bool();
      c.closed_loop.grid_resolution_m = get_parameter("crane_slope.closed_loop_grouping.grid_resolution_m").as_double();
      c.closed_loop.closing_radius_m = get_parameter("crane_slope.closed_loop_grouping.closing_radius_m").as_double();
      c.closed_loop.min_containment_ratio = get_parameter("crane_slope.closed_loop_grouping.min_containment_ratio").as_double();
      c.closed_loop.min_enclosed_cells = get_parameter("crane_slope.closed_loop_grouping.min_enclosed_cells").as_int();
      c.closed_loop.use_boundary_points_for_loop = get_parameter("crane_slope.closed_loop_grouping.use_boundary_points_for_loop").as_bool();
      c.closed_loop.subcluster_match_xy_gate = get_parameter("crane_slope.closed_loop_grouping.subcluster_match_xy_gate").as_double();
      c.closed_loop.subcluster_match_z_gate = get_parameter("crane_slope.closed_loop_grouping.subcluster_match_z_gate").as_double();
      crane_closed_loop_grouping_enabled_ = c.closed_loop.enabled;
      c.roi_filter.enabled = get_parameter("crane_slope.roi_filter.enabled").as_bool();
      c.roi_filter.only_crane_roi = get_parameter("crane_slope.roi_filter.only_crane_roi").as_bool();
      c.roi_filter.active_include_background = get_parameter("crane_slope.roi_filter.active_include_background").as_bool();
      c.roi_filter.idle_exclude_beam_roi = get_parameter("crane_slope.roi_filter.idle_exclude_beam_roi").as_bool();
      c.roi_filter.expand_beam_roi = get_parameter("crane_slope.roi_filter.expand_beam_roi").as_bool();
      crane_slope_roi_filter_enabled_ = c.roi_filter.enabled;
      crane_slope_roi_only_crane_roi_ = c.roi_filter.only_crane_roi;
      crane_slope_roi_active_include_background_ = c.roi_filter.active_include_background;
      crane_slope_roi_idle_exclude_beam_roi_ = c.roi_filter.idle_exclude_beam_roi;
      crane_slope_roi_expand_beam_roi_ = c.roi_filter.expand_beam_roi;
      c.work_status.fsm_enabled = get_parameter("crane_slope.work_status.fsm_enabled").as_bool();
      c.work_status.boundary_motion_enabled = get_parameter("crane_slope.work_status.boundary_motion_enabled").as_bool();
      c.work_status.boundary_temporal_enabled = get_parameter("crane_slope.work_status.boundary_temporal_enabled").as_bool();
      c.work_status.boundary_start_confirm_sec = get_parameter("crane_slope.work_status.boundary_start_confirm_sec").as_double();
      c.work_status.boundary_stop_confirm_sec = get_parameter("crane_slope.work_status.boundary_stop_confirm_sec").as_double();
      c.work_status.boundary_support_timeout_sec = get_parameter("crane_slope.work_status.boundary_support_timeout_sec").as_double();
      c.work_status.cargo_roi_width_x_m=get_parameter("crane_slope.work_status.cargo_roi_width_x_m").as_double();
      c.work_status.cargo_roi_width_y_m=get_parameter("crane_slope.work_status.cargo_roi_width_y_m").as_double();
      c.work_status.cargo_start_offset_m = get_parameter("crane_slope.work_status.cargo_start_offset_m").as_double();
      c.work_status.virtual_hook_offset_z_m = get_parameter("crane_slope.work_status.virtual_hook_offset_z_m").as_double();
      c.work_status.cargo_confirm_sec = get_parameter("crane_slope.work_status.cargo_confirm_sec").as_double();
      c.work_status.cargo_loss_sec = get_parameter("crane_slope.work_status.cargo_loss_sec").as_double();
      c.work_status.lifting_event_hold_sec=get_parameter("crane_slope.work_status.lifting_event_hold_sec").as_double();
      c.work_status.lifting_clearance_delta_m=get_parameter("crane_slope.work_status.lifting_clearance_delta_m").as_double();
      c.work_status.lifting_min_clearance_m=get_parameter("crane_slope.work_status.lifting_min_clearance_m").as_double();
      c.work_status.cargo_min_surface_width_m = get_parameter("crane_slope.work_status.cargo_min_surface_width_m").as_double();
      c.work_status.cargo_min_fill_ratio = get_parameter("crane_slope.work_status.cargo_min_fill_ratio").as_double();
      c.work_status.boundary_search_radius_m = get_parameter("crane_slope.work_status.boundary_search_radius_m").as_double();
      c.work_status.boundary_min_coherent_points = get_parameter("crane_slope.work_status.boundary_min_coherent_points").as_int();
      c.work_status.enabled = get_parameter("crane_slope.work_status.enabled").as_bool();
      c.work_status.grid_resolution_m = get_parameter("crane_slope.work_status.grid_resolution_m").as_double();
      c.work_status.size_ratio_k = get_parameter("crane_slope.work_status.size_ratio_k").as_double();
      c.work_status.connect_xy_margin = get_parameter("crane_slope.work_status.connect_xy_margin").as_double();
      c.work_status.connect_max_gap_z = get_parameter("crane_slope.work_status.connect_max_gap_z").as_double();
      c.work_status.min_candidate_points = static_cast<size_t>(get_parameter("crane_slope.work_status.min_candidate_points").as_int());
      c.work_status.temporal_window_n = get_parameter("crane_slope.work_status.temporal_window_n").as_int();
      c.work_status.confirm_m_on = get_parameter("crane_slope.work_status.confirm_m_on").as_int();
      c.work_status.confirm_m_off = get_parameter("crane_slope.work_status.confirm_m_off").as_int();
      c.temporal_stabilization.enabled = get_parameter("crane_slope.temporal_stabilization.enabled").as_bool();
      c.temporal_stabilization.mode = get_parameter("crane_slope.temporal_stabilization.mode").as_string();
      c.temporal_stabilization.window_size = get_parameter("crane_slope.temporal_stabilization.window_size").as_int();
      c.temporal_stabilization.voxel_size_xy = get_parameter("crane_slope.temporal_stabilization.voxel_size_xy").as_double();
      c.temporal_stabilization.voxel_size_z = get_parameter("crane_slope.temporal_stabilization.voxel_size_z").as_double();
      c.temporal_stabilization.min_hit_count = get_parameter("crane_slope.temporal_stabilization.min_hit_count").as_int();
      c.temporal_stabilization.temporal_stabilization_enabled = get_parameter("crane_slope.temporal_stabilization.temporal_stabilization_enabled").as_bool();
      c.temporal_stabilization.stabilization_alpha = get_parameter("crane_slope.temporal_stabilization.stabilization_alpha").as_double();
      c.temporal_stabilization.jump_tolerance_length = get_parameter("crane_slope.temporal_stabilization.jump_tolerance_length").as_double();
      c.temporal_stabilization.jump_tolerance_width = get_parameter("crane_slope.temporal_stabilization.jump_tolerance_width").as_double();
      c.temporal_stabilization.jump_confirm_frames = get_parameter("crane_slope.temporal_stabilization.jump_confirm_frames").as_int();
      c.temporal_stabilization.max_hold_missed_frames = get_parameter("crane_slope.temporal_stabilization.max_hold_missed_frames").as_int();
      c.temporal_stabilization.node_confirm_frames = get_parameter("crane_slope.temporal_stabilization.node_confirm_frames").as_int();
      c.temporal_stabilization.node_max_hold_frames = get_parameter("crane_slope.temporal_stabilization.node_max_hold_frames").as_int();
      c.temporal_stabilization.largest_swap_confirm_frames = get_parameter("crane_slope.temporal_stabilization.largest_swap_confirm_frames").as_int();
      c.temporal_stabilization.largest_match_dist = get_parameter("crane_slope.temporal_stabilization.largest_match_dist").as_double();
      c.temporal_stabilization.yaw_jump_tolerance_deg = get_parameter("crane_slope.temporal_stabilization.yaw_jump_tolerance_deg").as_double();
      c.temporal_stabilization.yaw_alpha = get_parameter("crane_slope.temporal_stabilization.yaw_alpha").as_double();
      c.top_down_hook.max_slope_angle_deg = get_parameter("crane_slope.top_down_hook.max_slope_angle_deg").as_double();
      c.top_down_hook.max_turn_angle_deg = get_parameter("crane_slope.top_down_hook.max_turn_angle_deg").as_double();
      c.fixed_root.enabled = get_parameter("crane_slope.fixed_root.enabled").as_bool();
      c.fixed_root.left_roi.x_min = get_parameter("crane_slope.fixed_root.left_roi.x_min").as_double();
      c.fixed_root.left_roi.x_max = get_parameter("crane_slope.fixed_root.left_roi.x_max").as_double();
      c.fixed_root.left_roi.y_min = get_parameter("crane_slope.fixed_root.left_roi.y_min").as_double();
      c.fixed_root.left_roi.y_max = get_parameter("crane_slope.fixed_root.left_roi.y_max").as_double();
      c.fixed_root.left_roi.z_min = get_parameter("crane_slope.fixed_root.left_roi.z_min").as_double();
      c.fixed_root.left_roi.z_max = get_parameter("crane_slope.fixed_root.left_roi.z_max").as_double();
      c.fixed_root.right_roi.x_min = get_parameter("crane_slope.fixed_root.right_roi.x_min").as_double();
      c.fixed_root.right_roi.x_max = get_parameter("crane_slope.fixed_root.right_roi.x_max").as_double();
      c.fixed_root.right_roi.y_min = get_parameter("crane_slope.fixed_root.right_roi.y_min").as_double();
      c.fixed_root.right_roi.y_max = get_parameter("crane_slope.fixed_root.right_roi.y_max").as_double();
      c.fixed_root.right_roi.z_min = get_parameter("crane_slope.fixed_root.right_roi.z_min").as_double();
      c.fixed_root.right_roi.z_max = get_parameter("crane_slope.fixed_root.right_roi.z_max").as_double();
      c.fixed_root.min_points = static_cast<size_t>(get_parameter("crane_slope.fixed_root.min_points").as_int());
      c.fixed_root.hold_frames = get_parameter("crane_slope.fixed_root.hold_frames").as_int();
      c.fixed_root.motion_speed_threshold = get_parameter("crane_slope.fixed_root.motion_speed_threshold").as_double();
      c.fixed_root.motion_disp_threshold = get_parameter("crane_slope.fixed_root.motion_disp_threshold").as_double();
      c.fixed_root.motion_confirm_frames = get_parameter("crane_slope.fixed_root.motion_confirm_frames").as_int();
      c.connection_stabilization.enabled = get_parameter("crane_slope.connection_stabilization.enabled").as_bool();
      c.connection_stabilization.history_frames = get_parameter("crane_slope.connection_stabilization.history_frames").as_int();
      c.connection_stabilization.min_direct_frames = get_parameter("crane_slope.connection_stabilization.min_direct_frames").as_int();
      c.connection_stabilization.min_transition_count = get_parameter("crane_slope.connection_stabilization.min_transition_count").as_int();
      c.connection_stabilization.missing_hold_frames = get_parameter("crane_slope.connection_stabilization.missing_hold_frames").as_int();
      c.connection_stabilization.spatial_continuity_enabled = get_parameter("crane_slope.connection_stabilization.spatial_continuity_enabled").as_bool();
      c.connection_stabilization.spatial_min_direct_frames = get_parameter("crane_slope.connection_stabilization.spatial_min_direct_frames").as_int();
      c.connection_stabilization.spatial_max_gap_z = get_parameter("crane_slope.connection_stabilization.spatial_max_gap_z").as_double();
      c.connection_stabilization.spatial_max_dxy = get_parameter("crane_slope.connection_stabilization.spatial_max_dxy").as_double();
      c.dual_roi_anchor.enabled = get_parameter("crane_slope.dual_roi_anchor.enabled").as_bool();
      c.dual_roi_anchor.publish_markers = get_parameter("crane_slope.dual_roi_anchor.publish_markers").as_bool();
      c.dual_roi_anchor.fixed_roi_size_xy = get_parameter("crane_slope.dual_roi_anchor.fixed_roi_size_xy").as_double();
      c.dual_roi_anchor.fixed_roi_size_z = get_parameter("crane_slope.dual_roi_anchor.fixed_roi_size_z").as_double();
      c.dual_roi_anchor.moving_roi_size_xy = get_parameter("crane_slope.dual_roi_anchor.moving_roi_size_xy").as_double();
      c.dual_roi_anchor.moving_roi_size_z = get_parameter("crane_slope.dual_roi_anchor.moving_roi_size_z").as_double();
      c.dual_roi_anchor.confirm_frames = get_parameter("crane_slope.dual_roi_anchor.confirm_frames").as_int();
      c.dual_roi_anchor.min_direct_evidence_frames = get_parameter("crane_slope.dual_roi_anchor.min_direct_evidence_frames").as_int();
      c.dual_roi_anchor.max_lock_slope_deg = get_parameter("crane_slope.dual_roi_anchor.max_lock_slope_deg").as_double();
      c.dual_roi_anchor.max_lock_turn_deg = get_parameter("crane_slope.dual_roi_anchor.max_lock_turn_deg").as_double();
      c.dual_roi_anchor.noise_deadband_xy = get_parameter("crane_slope.dual_roi_anchor.noise_deadband_xy").as_double();
      c.dual_roi_anchor.noise_deadband_z = get_parameter("crane_slope.dual_roi_anchor.noise_deadband_z").as_double();
      c.dual_roi_anchor.relocate_thresh_xy = get_parameter("crane_slope.dual_roi_anchor.relocate_thresh_xy").as_double();
      c.dual_roi_anchor.relocate_thresh_z = get_parameter("crane_slope.dual_roi_anchor.relocate_thresh_z").as_double();
      c.dual_roi_anchor.relocate_confirm_frames = get_parameter("crane_slope.dual_roi_anchor.relocate_confirm_frames").as_int();
      c.dual_roi_anchor.anchor_memory_frames = get_parameter("crane_slope.dual_roi_anchor.anchor_memory_frames").as_int();
      c.dual_roi_anchor.beam_intrusion_guard_enabled = get_parameter("crane_slope.dual_roi_anchor.beam_intrusion_guard_enabled").as_bool();
      c.dual_roi_anchor.beam_min_length_m = get_parameter("crane_slope.dual_roi_anchor.beam_min_length_m").as_double();
      c.dual_roi_anchor.beam_min_aspect_ratio = get_parameter("crane_slope.dual_roi_anchor.beam_min_aspect_ratio").as_double();
      c.dual_roi_anchor.intrusion_hold_frames = get_parameter("crane_slope.dual_roi_anchor.intrusion_hold_frames").as_int();

      c.virtual_rope.enabled = get_parameter("crane_slope.virtual_rope.enabled").as_bool();
      c.virtual_rope.wire_min_spacing_m = get_parameter("crane_slope.virtual_rope.wire_min_spacing_m").as_double();
      c.virtual_rope.wire_max_spacing_m = get_parameter("crane_slope.virtual_rope.wire_max_spacing_m").as_double();
      c.virtual_rope.wire_merge_span_m = get_parameter("crane_slope.virtual_rope.wire_merge_span_m").as_double();
      c.virtual_rope.memory_frames = get_parameter("crane_slope.virtual_rope.memory_frames").as_int();
      c.virtual_rope.publish_markers = get_parameter("crane_slope.virtual_rope.publish_markers").as_bool();

      c.sheave.track_continuity_enabled=get_parameter("crane_slope.sheave.track_continuity_enabled").as_bool();
      c.sheave.reference_matching_enabled=get_parameter("crane_slope.sheave.reference_matching_enabled").as_bool();
      c.sheave.debug_seed_world=get_parameter("crane_slope.sheave.debug_seed_world").as_double_array();
      auto parse_reference_rows=[&](const std::vector<std::string>&ref_ids,const std::vector<std::string>&ref_sources,
          const std::vector<std::string>&ref_labels,const std::vector<int64_t>&ref_families,
          const std::vector<double>&ref_features,const std::vector<double>&ref_local,
          std::vector<cx128s2_bev_perception::HousingShapeReference>&destination,size_t max_rows){
      if(!ref_local.empty() && ref_local.size()!=30*ref_ids.size())
        throw std::invalid_argument("Housing local references require 3 PCA extents + 27 occupancy bins per row");
      constexpr size_t stride=28;
      if(ref_ids.size()>max_rows || ref_sources.size()!=ref_ids.size() || ref_labels.size()!=ref_ids.size() ||
          ref_families.size()!=ref_ids.size() || ref_features.size()!=stride*ref_ids.size())
        throw std::invalid_argument("Housing reference arrays must have matching counts, 28 features per row and bounded rows");
      if(!c.sheave.debug_seed_world.empty() && (c.sheave.debug_seed_world.size()!=3 ||
          !std::all_of(c.sheave.debug_seed_world.begin(),c.sheave.debug_seed_world.end(),[](double v){return std::isfinite(v);})))
        throw std::invalid_argument("Housing debug seed must be empty or three finite world coordinates");
      std::set<std::string> unique_ref_ids;
      for(size_t i=0;i<ref_ids.size();++i){
        const auto * v=ref_features.data()+i*stride;
        if(!std::all_of(v,v+stride,[](double x){return std::isfinite(x);}) || ref_ids[i].empty() || !unique_ref_ids.insert(ref_ids[i]).second)
          throw std::invalid_argument("Housing reference IDs must be unique and features finite");
        cx128s2_bev_perception::HousingShapeReference r;r.id=ref_ids[i];r.source=ref_sources[i];r.label=ref_labels[i];r.family_id=ref_families[i];
        for(int k=0;k<3;++k){r.dimensions[k]=v[k];r.spectrum[k]=v[3+k];r.z_distribution[k]=v[6+k];r.horizontal_distribution[k]=v[9+k];r.source_center[k]=v[18+k];r.view_direction[k]=v[22+k];}
        r.sensor_range=v[12];r.point_count=v[13];r.density=v[14];r.min_z_offset=v[15];r.max_z_offset=v[16];r.peak_z_offset=v[17];r.reliability=v[21];r.source_frame=int(v[25]);r.source_stamp=v[26];r.manually_checked=v[27]==1;
        auto distribution=[](const std::array<double,3> & a){return std::all_of(a.begin(),a.end(),[](double x){return x>=0 && x<=1;}) && std::abs(a[0]+a[1]+a[2]-1)<1e-3;};
        if(!std::all_of(r.dimensions.begin(),r.dimensions.end(),[](double x){return x>0;}) || !distribution(r.spectrum) || !distribution(r.z_distribution) || !distribution(r.horizontal_distribution) ||
            r.sensor_range<=0 || r.point_count<3 || r.reliability<0 || r.reliability>1 || r.min_z_offset>=r.max_z_offset || r.peak_z_offset<r.min_z_offset || r.peak_z_offset>r.max_z_offset ||
            (r.label!="USER_SEEDED" && r.label!="VERIFIED" && r.label!="PROBABLE" && r.label!="AMBIGUOUS" && r.label!="NEGATIVE"))
          throw std::invalid_argument("Invalid Housing reference geometry or provenance label");
        if(!ref_local.empty()){
          const double * local=ref_local.data()+30*i;
          if(!std::all_of(local,local+30,[](double x){return std::isfinite(x) && x>=0;}) ||
              local[0]<=0 || local[1]<=0 || local[2]<=0 ||
              std::abs(std::accumulate(local+3,local+30,0.0)-1)>1e-3)
            throw std::invalid_argument("Invalid PCA-local Housing descriptor");
          r.has_local_shape=true;std::copy(local,local+3,r.local_extent.begin());
          std::copy(local+3,local+30,r.local_occupancy.begin());
        }
        destination.push_back(std::move(r));
      }
      };
      parse_reference_rows(get_parameter("crane_slope.sheave.reference_ids").as_string_array(),
          get_parameter("crane_slope.sheave.reference_sources").as_string_array(),
          get_parameter("crane_slope.sheave.reference_labels").as_string_array(),
          get_parameter("crane_slope.sheave.reference_families").as_integer_array(),
          get_parameter("crane_slope.sheave.reference_features").as_double_array(),
          get_parameter("crane_slope.sheave.reference_local_features").as_double_array(),c.sheave.references,8);
      const auto learned_file=get_parameter("crane_slope.sheave.learned_reference_file").as_string();
      if(!learned_file.empty()){
        const auto model=YAML::LoadFile(learned_file);
        if(model["model_version"].as<int>()!=1 || !model["references"].IsSequence())throw std::invalid_argument("Invalid Housing learned model schema");
        std::vector<std::string> ids,sources,labels;std::vector<int64_t> families;std::vector<double> features,local;
        for(const auto&r:model["references"]){
          ids.push_back(r["id"].as<std::string>());sources.push_back(r["source"].as<std::string>());labels.push_back(r["label"].as<std::string>());families.push_back(r["family"].as<int64_t>());
          const auto f=r["features"].as<std::vector<double>>(),l=r["local_features"].as<std::vector<double>>();
          features.insert(features.end(),f.begin(),f.end());local.insert(local.end(),l.begin(),l.end());
        }
        parse_reference_rows(ids,sources,labels,families,features,local,c.sheave.learned_references,4);
        for(const auto&r:c.sheave.learned_references){
          if(!r.trusted() || r.label=="NEGATIVE")throw std::invalid_argument("Learned Housing rows require reviewed positive provenance");
          for(const auto&base:c.sheave.references)if(base.id==r.id)throw std::invalid_argument("Learned Housing ID shadows immutable base bank");
        }
      }
      c.sheave.surface_gap_m = get_parameter("crane_slope.sheave.surface_gap_m").as_double();
      c.sheave.enabled = get_parameter("crane_slope.sheave.enabled").as_bool();
      c.sheave.min_radius_m = get_parameter("crane_slope.sheave.min_radius_m").as_double();
      c.sheave.max_radius_m = get_parameter("crane_slope.sheave.max_radius_m").as_double();
      c.sheave.default_radius_m = get_parameter("crane_slope.sheave.default_radius_m").as_double();
      c.sheave.min_arc_points = get_parameter("crane_slope.sheave.min_arc_points").as_int();
      c.sheave.max_circle_rms_m = get_parameter("crane_slope.sheave.max_circle_rms_m").as_double();
      c.sheave.max_sheave_width_m = get_parameter("crane_slope.sheave.max_sheave_width_m").as_double();
      c.sheave.confirm_frames = get_parameter("crane_slope.sheave.confirm_frames").as_int();
      c.sheave.terminate_chain_at_sheave = get_parameter("crane_slope.sheave.terminate_chain_at_sheave").as_bool();
      c.sheave.publish_markers = get_parameter("crane_slope.sheave.publish_markers").as_bool();

      c.z_temporal_median.enabled = get_parameter("crane_slope.z_temporal_median.enabled").as_bool();
      c.z_temporal_median.window_frames = get_parameter("crane_slope.z_temporal_median.window_frames").as_int();
      c.exception_gate.enabled = get_parameter("crane_slope.exception_gate.enabled").as_bool();
      c.exception_gate.min_length_m = get_parameter("crane_slope.exception_gate.min_length_m").as_double();
      c.exception_gate.min_area_m2 = get_parameter("crane_slope.exception_gate.min_area_m2").as_double();
      c.exception_gate.use_visibility = get_parameter("crane_slope.exception_gate.use_visibility").as_bool();
      c.exception_gate.max_occlusion_fraction = get_parameter("crane_slope.exception_gate.max_occlusion_fraction").as_double();
      c.exception_gate.scan_all_chain_nodes = get_parameter("crane_slope.exception_gate.scan_all_chain_nodes").as_bool();
      crane_plane_enabled_ = false;
      }
      crane_pipeline_ = std::make_unique<cx128s2_bev_perception::CranePipeline>(c);
    }
    crane_beam_enabled_ = get_parameter("crane_beam_enabled").as_bool();
    crane_beam_config_.enabled = crane_beam_enabled_;
    crane_beam_config_.target_frame = target_frame_;
    crane_beam_config_.motion_reference_mode = get_parameter("crane_beam.motion_reference_mode").as_string();
    crane_beam_config_.beam_roi_x_min = get_parameter("crane_beam.beam_roi_x_min").as_double();
    crane_beam_config_.beam_roi_x_max = get_parameter("crane_beam.beam_roi_x_max").as_double();
    crane_beam_config_.beam_roi_y_min = get_parameter("crane_beam.beam_roi_y_min").as_double();
    crane_beam_config_.beam_roi_y_max = get_parameter("crane_beam.beam_roi_y_max").as_double();
    crane_beam_config_.beam_roi_z_min = get_parameter("crane_beam.beam_roi_z_min").as_double();
    crane_beam_config_.beam_roi_z_max = get_parameter("crane_beam.beam_roi_z_max").as_double();
    crane_beam_config_.bg_roi_x_min = get_parameter("crane_beam.background_roi_x_min").as_double();
    crane_beam_config_.bg_roi_x_max = get_parameter("crane_beam.background_roi_x_max").as_double();
    crane_beam_config_.bg_roi_y_min = get_parameter("crane_beam.background_roi_y_min").as_double();
    crane_beam_config_.bg_roi_y_max = get_parameter("crane_beam.background_roi_y_max").as_double();
    crane_beam_config_.bg_roi_z_min = get_parameter("crane_beam.background_roi_z_min").as_double();
    crane_beam_config_.bg_roi_z_max = get_parameter("crane_beam.background_roi_z_max").as_double();
    crane_beam_config_.min_beam_roi_points = static_cast<size_t>(get_parameter("crane_beam.min_beam_roi_points").as_int());
    crane_beam_config_.min_bg_roi_points = static_cast<size_t>(get_parameter("crane_beam.min_background_roi_points").as_int());
    crane_beam_config_.min_beam_cluster_points = static_cast<size_t>(get_parameter("crane_beam.min_beam_cluster_points").as_int());
    crane_beam_config_.beam_z_band = get_parameter("crane_beam.beam_z_band").as_double();
    crane_beam_config_.lift_dz_threshold = get_parameter("crane_beam.lift_dz_threshold").as_double();
    crane_beam_config_.hook_beam_dz_match_tolerance = get_parameter("crane_beam.hook_beam_dz_match_tolerance").as_double();
    crane_beam_config_.lower_dz_threshold = get_parameter("crane_beam.lower_dz_threshold").as_double();
    crane_beam_config_.stop_dz_threshold = get_parameter("crane_beam.stop_dz_threshold").as_double();
    crane_beam_config_.ground_tolerance = get_parameter("crane_beam.ground_tolerance").as_double();
    crane_beam_config_.crane_move_start_x = get_parameter("crane_beam.crane_move_start_x").as_double();
    crane_beam_config_.crane_move_start_y = get_parameter("crane_beam.crane_move_start_y").as_double();
    crane_beam_config_.crane_move_stop_x = get_parameter("crane_beam.crane_move_stop_x").as_double();
    crane_beam_config_.crane_move_stop_y = get_parameter("crane_beam.crane_move_stop_y").as_double();
    crane_beam_config_.crane_move_stop_frames = get_parameter("crane_beam.crane_move_stop_frames").as_int();
    crane_beam_config_.lift_confirm_frames = get_parameter("crane_beam.lift_confirm_frames").as_int();
    crane_beam_config_.suspend_confirm_frames = get_parameter("crane_beam.suspend_confirm_frames").as_int();
    crane_beam_config_.lower_confirm_frames = get_parameter("crane_beam.lower_confirm_frames").as_int();
    crane_beam_config_.placed_confirm_frames = get_parameter("crane_beam.placed_confirm_frames").as_int();
    crane_beam_config_.complete_confirm_frames = get_parameter("crane_beam.complete_confirm_frames").as_int();
    crane_beam_config_.beam_measurement_max_lost_frames = get_parameter("crane_beam.beam_measurement_max_lost_frames").as_int();
    crane_beam_config_.length_stable_threshold = get_parameter("crane_beam.length_stable_threshold").as_double();
    crane_beam_config_.length_lock_frames = get_parameter("crane_beam.length_lock_frames").as_int();
    crane_beam_config_.cluster_bev_res = get_parameter("crane_beam.cluster_bev_res").as_double();
    crane_beam_config_.continuity_gate_xy = get_parameter("crane_beam.continuity_gate_xy").as_double();
    crane_beam_config_.beam_z_histogram_bin_size = get_parameter("crane_beam.beam_z_histogram_bin_size").as_double();
    crane_beam_config_.beam_ground_clearance = get_parameter("crane_beam.beam_ground_clearance").as_double();
    crane_beam_config_.tracking_gate_xy = get_parameter("crane_beam.tracking_gate_xy").as_double();
    crane_beam_config_.tracking_gate_z = get_parameter("crane_beam.tracking_gate_z").as_double();
    crane_beam_config_.tracking_gate_yaw = get_parameter("crane_beam.tracking_gate_yaw").as_double();
    crane_beam_config_.tracking_gate_length = get_parameter("crane_beam.tracking_gate_length").as_double();
    crane_beam_config_.support_search_margin_xy = get_parameter("crane_beam.support_search_margin_xy").as_double();
    crane_beam_config_.min_support_points = static_cast<size_t>(get_parameter("crane_beam.min_support_points").as_int());
    crane_beam_config_.placement_tolerance = get_parameter("crane_beam.placement_tolerance").as_double();
    crane_beam_config_.beam_default_height = get_parameter("crane_beam.beam_default_height").as_double();
    crane_beam_config_.landing_height_gap_threshold = get_parameter("crane_beam.landing_height_gap_threshold").as_double();
    crane_beam_config_.use_z_ema = get_parameter("crane_beam.use_z_ema").as_bool();
    crane_beam_config_.z_ema_alpha = get_parameter("crane_beam.z_ema_alpha").as_double();
    crane_beam_config_.publish_debug = get_parameter("crane_beam.publish_debug").as_bool();
    crane_beam_config_.publish_background_roi_marker = get_parameter("crane_beam.publish_background_roi_marker").as_bool();
    crane_beam_config_.diagnostics_period_sec = get_parameter("crane_beam.diagnostics_period_sec").as_double();

    // Hardening: Cumulative vertical motion
    crane_beam_config_.cumulative_motion_enabled = get_parameter("crane_beam.cumulative_motion_enabled").as_bool();
    crane_beam_config_.cumulative_lift_distance = get_parameter("crane_beam.cumulative_lift_distance").as_double();
    crane_beam_config_.cumulative_lower_distance = get_parameter("crane_beam.cumulative_lower_distance").as_double();
    crane_beam_config_.cumulative_motion_min_frames = get_parameter("crane_beam.cumulative_motion_min_frames").as_int();
    crane_beam_config_.cumulative_direction_epsilon = get_parameter("crane_beam.cumulative_direction_epsilon").as_double();

    // Hardening: Support surface planar extent filtering
    crane_beam_config_.support_min_xy_extent = get_parameter("crane_beam.support_min_xy_extent").as_double();
    crane_beam_config_.support_max_z_spread = get_parameter("crane_beam.support_max_z_spread").as_double();
    crane_beam_config_.support_temporal_continuity = get_parameter("crane_beam.support_temporal_continuity").as_bool();

    // Hardening: Tilted touchdown multi-segment evaluation
    crane_beam_config_.tilt_segmentation_enabled = get_parameter("crane_beam.tilt_segmentation_enabled").as_bool();
    crane_beam_config_.tilt_num_segments = get_parameter("crane_beam.tilt_num_segments").as_int();
    crane_beam_config_.min_supported_segment_ratio = get_parameter("crane_beam.min_supported_segment_ratio").as_double();

    // Hardening: Initial cluster shape gating
    crane_beam_config_.min_initial_cluster_length = get_parameter("crane_beam.min_initial_cluster_length").as_double();
    crane_beam_config_.min_initial_cluster_elongation = get_parameter("crane_beam.min_initial_cluster_elongation").as_double();
    crane_beam_config_.beam_high_z_gate_enabled = get_parameter("crane_beam.beam_high_z_gate_enabled").as_bool();
    crane_beam_config_.beam_high_z_reject_threshold = get_parameter("crane_beam.beam_high_z_reject_threshold").as_double();
    crane_beam_config_.beam_high_z_require_cluster_valid = get_parameter("crane_beam.beam_high_z_require_cluster_valid").as_bool();

    // Sensor transform freeze
    crane_beam_config_.freeze_sensor_transform = get_parameter("crane_beam.freeze_sensor_transform").as_bool();

    // Crane Hook measurements & auxiliary prepare_ready parameters
    crane_beam_config_.hook.enabled = get_parameter("crane_beam.hook.enabled").as_bool();
    crane_hook_reference_file_ = get_parameter("crane_beam.hook.reference_file").as_string();
    if (crane_hook_reference_file_.empty()) crane_hook_reference_file_ =
        ament_index_cpp::get_package_share_directory("cx128s2_bev_perception") + "/config/crane_hook_reference.yaml";
    crane_beam_config_.hook.reference_min_frames =
        std::max(30, static_cast<int>(get_parameter("crane_beam.hook.reference_min_frames").as_int()));
    crane_beam_config_.hook.roi_left_x_min = get_parameter("crane_beam.hook.roi_left_x_min").as_double();
    crane_beam_config_.hook.roi_left_x_max = get_parameter("crane_beam.hook.roi_left_x_max").as_double();
    crane_beam_config_.hook.roi_right_x_min = get_parameter("crane_beam.hook.roi_right_x_min").as_double();
    crane_beam_config_.hook.roi_right_x_max = get_parameter("crane_beam.hook.roi_right_x_max").as_double();
    crane_beam_config_.hook.roi_y_min = get_parameter("crane_beam.hook.roi_y_min").as_double();
    crane_beam_config_.hook.roi_y_max = get_parameter("crane_beam.hook.roi_y_max").as_double();
    crane_beam_config_.hook.roi_z_min = get_parameter("crane_beam.hook.roi_z_min").as_double();
    crane_beam_config_.hook.roi_z_max = get_parameter("crane_beam.hook.roi_z_max").as_double();
    crane_beam_config_.hook.window_size_z = get_parameter("crane_beam.hook.window_size_z").as_double();
    crane_beam_config_.hook.window_step_z = get_parameter("crane_beam.hook.window_step_z").as_double();
    crane_beam_config_.hook.min_body_points = static_cast<size_t>(get_parameter("crane_beam.hook.min_body_points").as_int());
    crane_beam_config_.hook.tracking_enabled = get_parameter("crane_beam.hook.tracking_enabled").as_bool();
    crane_beam_config_.hook.tracking_confirm_frames = get_parameter("crane_beam.hook.tracking_confirm_frames").as_int();
    crane_beam_config_.hook.tracking_gate_xy = get_parameter("crane_beam.hook.tracking_gate_xy").as_double();
    crane_beam_config_.hook.tracking_gate_z = get_parameter("crane_beam.hook.tracking_gate_z").as_double();
    crane_beam_config_.hook.tracking_gate_z_max = get_parameter("crane_beam.hook.tracking_gate_z_max").as_double();
    crane_beam_config_.hook.tracking_max_lost_frames = get_parameter("crane_beam.hook.tracking_max_lost_frames").as_int();
    crane_beam_config_.hook.tracking_prediction_timeout_sec = get_parameter("crane_beam.hook.tracking_prediction_timeout_sec").as_double();
    crane_beam_config_.hook.tracking_accel_std_xy = get_parameter("crane_beam.hook.tracking_accel_std_xy").as_double();
    crane_beam_config_.hook.tracking_accel_std_z = get_parameter("crane_beam.hook.tracking_accel_std_z").as_double();
    crane_beam_config_.hook.tracking_measurement_std = get_parameter("crane_beam.hook.tracking_measurement_std").as_double();
    crane_beam_config_.hook.ground_search_z_min = get_parameter("crane_beam.hook.ground_search_z_min").as_double();
    crane_beam_config_.hook.ground_search_z_max = get_parameter("crane_beam.hook.ground_search_z_max").as_double();
    crane_beam_config_.hook.ground_bin_size = get_parameter("crane_beam.hook.ground_bin_size").as_double();
    crane_beam_config_.hook.min_ground_points = static_cast<size_t>(get_parameter("crane_beam.hook.min_ground_points").as_int());
    crane_beam_config_.hook.prepare_distance = get_parameter("crane_beam.hook.prepare_distance").as_double();
    crane_beam_config_.hook.prepare_release_margin = get_parameter("crane_beam.hook.prepare_release_margin").as_double();
    crane_beam_config_.hook.prepare_confirm_frames = get_parameter("crane_beam.hook.prepare_confirm_frames").as_int();


    if (crane_beam_detector_) {
      crane_beam_detector_->updateConfig(crane_beam_config_);
    }
  }

