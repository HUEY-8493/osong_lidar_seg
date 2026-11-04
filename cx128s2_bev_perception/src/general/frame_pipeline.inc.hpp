  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    received_frame_count_++;

    QueuedCloud qc;
    qc.msg = msg;
    qc.enqueue_time = std::chrono::steady_clock::now();
    qc.sequence = sequence_counter_++;

    {
      std::lock_guard<std::mutex> lock(latest_msg_mutex_);
      if (latest_msg_.has_value()) {
        dropped_frame_count_++;
      }
      latest_msg_ = qc;
    }
    latest_msg_cv_.notify_one();
  }

  void perceptionWorkerLoop() {
    RCLCPP_INFO(get_logger(), "Perception worker thread started.");
    while (worker_running_ && rclcpp::ok()) {
      QueuedCloud local_cloud;
      {
        std::unique_lock<std::mutex> lock(latest_msg_mutex_);
        latest_msg_cv_.wait(lock, [this]() {
          return !worker_running_ || latest_msg_.has_value();
        });
        if (!worker_running_ || !rclcpp::ok()) {
          break;
        }
        local_cloud = std::move(latest_msg_.value());
        latest_msg_.reset();
      }
      processCloudFrame(local_cloud);
    }
    RCLCPP_INFO(get_logger(), "Perception worker thread stopping.");
  }

#ifdef GENERAL_REPLAY_OFFLINE
  struct OfflinePreparedFrame {
    tf2::Transform tf;
    std::string output_frame;
    std::vector<PointXYZIRT> points;
    std::vector<uint64_t> compact_to_original;
    std::vector<int> valid, roi, ignored, hard_ignored, foreground_before_ror, foreground, owners;
    std::vector<double> similarity;
    std::vector<ClusterResult> clusters;
    std::vector<ObjectFeature> features;
    bool enabled=false, publish_confirmed=false, background_ready=false, learning_active=false;
    bool has_ring=false, has_time=false, tf_fallback=false, ror_skipped=false;
    int foreground_count=0, static_count=0, ror_input=0;
  };
  const OfflinePreparedFrame * offline_loaded_ = nullptr;
  std::function<void(const OfflinePreparedFrame &)> offline_capture_;
  std::function<void()> offline_before_tracking_;
  std::function<bool()> offline_restore_depth_;
  std::function<void()> offline_after_frame_;
  std::function<void(const std::vector<ObjectFeature> &)> offline_initial_visibility_;
  std::function<void(const cx128s2_bev_perception::msg::TrackedObjectArray &,
      const cx128s2_bev_perception::msg::TrackedObjectArray &,
      const cx128s2_bev_perception::msg::TrackedObjectArray &,
      const cx128s2_bev_perception::msg::TrackedObjectArray &,
      const cx128s2_bev_perception::msg::TrackedObjectArray &)> offline_public_;
  bool offline_prefix_only_ = false, offline_tf_fallback_ = false;
  std::vector<uint64_t> offline_compact_to_original_;
  std::vector<int> offline_foreground_before_ror_;
#endif

  void processCloudFrame(const QueuedCloud & queued_cloud) {
    if (enable_parallel_processing_) {
      omp_set_num_threads(parallel_threads_);
      omp_set_nested(0);
    } else {
      omp_set_num_threads(1);
      omp_set_nested(0);
    }
    auto start_time = std::chrono::steady_clock::now();
    input_queue_delay_ms_ = std::chrono::duration<double, std::milli>(start_time - queued_cloud.enqueue_time).count();

    const auto msg = queued_cloud.msg;
    frame_id_++;
    raw_vehicle_box_refine_ms_ = 0.0;
    raw_vehicle_box_stats_ = {};
    raw_vehicle_box_queries_ = raw_vehicle_box_used_ = 0;
    processed_frame_count_++;
    shape_graph_computed_this_frame_ = 0;
    shape_graph_skipped_this_frame_ = 0;
    last_input_frame_id_ = msg->header.frame_id;

    tf2::Transform tf;
    tf.setIdentity();
    std::string output_frame;

    double t_read_cloud = 0.0;
    // Frames are processed by the single perception worker. Reuse this backing storage instead
    // of allocating a new ~136k-point vector for every PointCloud2 message.
    auto & points = frame_points_;
    points.clear();
    {
      ScopedTimer timer(t_read_cloud);
#ifdef GENERAL_REPLAY_OFFLINE
      if (offline_loaded_) { tf = offline_loaded_->tf; output_frame = offline_loaded_->output_frame; }
      else
#endif
      if (!lookupTransform(msg, tf, output_frame)) {
        visibility_frame_valid_ = false;
        if (worker_running_ && rclcpp::ok()) {
          publishStatus();
        }
        total_pipeline_ms_ = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start_time).count();
        return;
      }
      if (!last_target_frame_.empty() && last_target_frame_ != output_frame) {
        // A different world frame invalidates all long-lived coordinates.
        resetTrackingState();
      }
      last_target_frame_ = output_frame;
      if (crane_pipeline_ && (output_frame != "world" || !validateCraneSlopeCloud(*msg))) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
            "Crane slope requires world TF and packed little-endian XYZ float32, optional ring uint16/time float32");
        return;
      }
#ifdef GENERAL_REPLAY_OFFLINE
      if (offline_loaded_) {
        points = offline_loaded_->points;
        last_input_has_ring_ = offline_loaded_->has_ring;
        last_input_has_time_ = offline_loaded_->has_time;
      } else
#endif
      if (!readCloud(msg, tf, output_frame == target_frame_, points)) {
        visibility_frame_valid_ = false;
        total_pipeline_ms_ = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start_time).count();
        return;
      }
    }
    // TF is input-sensor -> processing frame, independently of visibility diagnostics.
    sensor_origin_x_ = tf.getOrigin().x();
    sensor_origin_y_ = tf.getOrigin().y();
    if (clusterer_) clusterer_->setSensorOrigin(sensor_origin_x_, sensor_origin_y_);
    read_cloud_ms_ = t_read_cloud;
    visibility_frame_valid_ = visibility_enabled_ && output_frame == target_frame_ &&
        msg->header.frame_id == visibility_model_.config().sensor_frame;
    vehicle_surface_predict_ms_ = vehicle_visibility_query_ms_ =
        vehicle_ownership_ms_ = vehicle_sparse_support_ms_ = 0.0;
    vehicle_lost_predict_ms_ = vehicle_reacquire_ms_ = 0.0;
    vehicle_dormant_management_ms_ = vehicle_dormant_reacquire_ms_ = 0.0;
    vehicle_fifo_identity_ms_ = 0.0;
    visibility_static_lookup_ms_ = visibility_depth_build_ms_ =
        visibility_cluster_ms_ = visibility_track_ms_ = 0.0;
    adaptive_track_visibility_ms_ = track_memory_update_ms_ = 0.0;
    if (visibility_frame_valid_ && crane_pipeline_name_!="height_surface") {
      const auto static_start = std::chrono::steady_clock::now();
      estimateVisibilityAngularSpacing(points, tf);
      const bool pose_changed = visibility_model_.setPose(tf);
      if (pose_changed || visibility_prewarm_pending_) {
        prewarmVisibilityRoi();
        visibility_prewarm_pending_ = false;
      }
      publishVisibilityRingProfile(points, tf);
      visibility_static_lookup_ms_ += std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - static_start).count();
    }
    last_input_count_ = static_cast<int>(msg->width * msg->height);
    last_transformed_count_ = static_cast<int>(points.size());

    std_msgs::msg::Header out_header = msg->header;
    out_header.frame_id = (output_frame_policy_ == "target_frame") ? output_frame : msg->header.frame_id;

    const double now_sec = now().seconds();
    const bool publish_debug_cloud_this_frame =
        (publish_debug_topics_ || pub_foreground_) && (debug_cloud_publish_period_sec_ <= 0.0 ||
        now_sec - last_debug_cloud_publish_time_ >= debug_cloud_publish_period_sec_);

    if (publish_debug_cloud_this_frame) {
      last_debug_cloud_publish_time_ = now_sec;
    }

    double t_debug_cloud = 0.0;
    {
      ScopedTimer timer(t_debug_cloud);
      if (publish_debug_cloud_this_frame && publish_transformed_cloud_ && publish_debug_transformed_cloud_ && (pub_road_frame_ ? pub_road_frame_->get_subscription_count() : 0u) > 0) {
        std::vector<int> all_idx(points.size());
        std::iota(all_idx.begin(), all_idx.end(), 0);
        if (worker_running_ && rclcpp::ok()) {
          publishDebugCloud(out_header, points, all_idx, pub_road_frame_,
                            true, last_debug_valid_published_points_); // Reusing variable as dummy
        }
      }
    }

#ifdef GENERAL_REPLAY_OFFLINE
    if (offline_loaded_) {
      const auto & f = *offline_loaded_;
      last_point_bg_similarity_ = f.similarity;
      frame_cluster_owner_ = f.owners;
      background_ready_ = f.background_ready; learning_active_ = f.learning_active;
      last_valid_count_ = static_cast<int>(f.valid.size());
      last_roi_filtered_count_ = static_cast<int>(f.roi.size());
      last_roi_ignored_count_ = static_cast<int>(f.ignored.size());
      last_roi_hard_ignored_count_ = static_cast<int>(f.hard_ignored.size());
      last_foreground_count_ = f.foreground_count; last_static_count_ = f.static_count;
      last_ror_input_count_ = f.ror_input; last_ror_skipped_ = f.ror_skipped;
      last_cluster_count_ = static_cast<int>(f.clusters.size());
      if (f.enabled && visibility_frame_valid_) {
        ScopedTimer timer(visibility_depth_build_ms_);
        if (!offline_restore_depth_ || !offline_restore_depth_())
          visibility_model_.buildGeneralDepth(points, frame_cluster_owner_);
      }
      auto features = f.features;
      finishGeneralFrame(out_header, points, f.foreground, f.clusters, features, f.enabled,
          f.publish_confirmed, publish_debug_cloud_this_frame, t_debug_cloud, start_time);
      return;
    }
#endif
    std::vector<int> valid_idx;
    {
      ScopedTimer timer(valid_crop_ms_);
      valid_idx.reserve(points.size());
      resetFrameStats();
      last_point_bg_similarity_.resize(points.size());
      const size_t num_pts = points.size();

      if (enable_parallel_processing_) {
        std::vector<uint8_t> valid_mask(num_pts, 0);
        #pragma omp parallel for
        for (size_t i = 0; i < num_pts; ++i) {
          if (passValidCrop(points[i])) {
            valid_mask[i] = 1;
          }
        }
        for (size_t i = 0; i < num_pts; ++i) {
          if (valid_mask[i]) {
            valid_idx.push_back(static_cast<int>(i));
            updateRangeRingStats(points[i]);
          }
        }
      } else {
        for (size_t i = 0; i < num_pts; ++i) {
          if (!passValidCrop(points[i])) continue;
          valid_idx.push_back(static_cast<int>(i));
          updateRangeRingStats(points[i]);
        }
      }
    }
    last_valid_count_ = static_cast<int>(valid_idx.size());

    {
      ScopedTimer timer(t_debug_cloud);
      if (publish_debug_cloud_this_frame) {
        if (worker_running_ && rclcpp::ok()) {
          publishDebugCloud(out_header, points, valid_idx, pub_valid_,
                            publish_debug_valid_points_, last_debug_valid_published_points_);
        }
      }
    }

    // Branch to Crane Beam Work Detection mode if selected
    if (perception_mode_ == "crane_beam_work") {
      if (crane_pipeline_) {
        if (crane_pipeline_name_=="height_surface")
          processCraneHeight(msg,out_header,points,valid_idx,tf,publish_debug_cloud_this_frame);
        else
        processCraneSlope(msg, out_header, points, valid_idx, tf, publish_debug_cloud_this_frame);
        return;
      }
      processCraneBeamPerception(msg, out_header, points, valid_idx, now_sec, start_time,
                                 publish_debug_cloud_this_frame);
      return;
    }

    std::vector<int> roi_filtered_idx = valid_idx;
    std::vector<int> roi_ignored_idx;
    std::vector<int> roi_hard_ignored_idx;
    {
      ScopedTimer timer(roi_policy_ms_);
      if (use_roi_policy_) {
        applyRoiPolicyToIndices(points, valid_idx, roi_filtered_idx, roi_ignored_idx, roi_hard_ignored_idx);
      }
    }
    last_roi_filtered_count_ = static_cast<int>(roi_filtered_idx.size());
    last_roi_ignored_count_ = static_cast<int>(roi_ignored_idx.size());
    last_roi_hard_ignored_count_ = static_cast<int>(roi_hard_ignored_idx.size());

    {
      ScopedTimer timer(t_debug_cloud);
      if (publish_debug_cloud_this_frame) {
        if (worker_running_ && rclcpp::ok()) {
          publishDebugCloud(out_header, points, roi_filtered_idx, pub_roi_filtered_,
                            publish_debug_roi_filtered_points_, last_debug_roi_filtered_published_points_);
        }
      }
    }

    const double stamp = stampSeconds(out_header);
    {
      ScopedTimer timer(background_learning_ms_);
      if (usesStaticBackground() && learning_active_) {
        if (background_learning_input_mode_ != "valid") {
          RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 5000,
              "background_learning_input_mode='%s' is legacy. Static background learning now uses "
              "valid_idx for simple background subtraction.",
              background_learning_input_mode_.c_str());
        }
        std::vector<int> learning_idx = roi_filtered_idx;
        if (use_roi_policy_ && !apply_ignore_before_learning_) {
          for (int idx : roi_ignored_idx) {
            const auto & p = points[idx];
            if (pointPolicy(p.x, p.y) == RoiDecision::IGNORE) learning_idx.push_back(idx);
          }
        }
        accumulateBackground(points, learning_idx, stamp);
        if (learningComplete(stamp)) finalizeBackground(out_header.frame_id);
      }
    }

    std::vector<int> foreground_idx, background_idx;
    const bool keep_bg = publish_debug_cloud_this_frame && publish_debug_static_background_points_ && (pub_static_background_ ? pub_static_background_->get_subscription_count() : 0u) > 0;

    {
      ScopedTimer timer(foreground_classification_ms_);
      if (usesStaticBackground()) {
        classifyForeground(points, roi_filtered_idx, foreground_idx, background_idx, keep_bg);
      } else {
        std::vector<int> ignored_by_ground;
        if (use_fixed_ground_plane_) {
          splitFixedGroundObstacle(points, roi_filtered_idx, background_idx, foreground_idx,
                              ignored_by_ground);
        } else {
          const auto ground_map = buildGroundMap(points, roi_filtered_idx);
          splitGroundObstacle(points, roi_filtered_idx, ground_map, background_idx, foreground_idx,
                              ignored_by_ground);
        }
        foreground_cell_persistence_.clear();
        for (int idx : foreground_idx) last_point_bg_similarity_[idx] = 0.0;
        for (int idx : background_idx) last_point_bg_similarity_[idx] = 1.0;
        for (int idx : ignored_by_ground) last_point_bg_similarity_[idx] = 1.0;
        last_foreground_count_ = static_cast<int>(foreground_idx.size());
        last_static_count_ = static_cast<int>(background_idx.size());
      }
    }

    // Keep ground fitting unchanged. Only remove corroborated low-return artifacts
    // from downstream foreground; unavailable ring/floor evidence passes through.
    const auto noise_start = std::chrono::steady_clock::now();
    last_general_ground_noise_stats_ = general_ground_noise_filter_.filter(
        points, foreground_idx, general_ground_noise_removed_, general_ground_noise_config_,
        last_input_has_ring_, [&](const PointXYZIRT & p) {
          const double range = sensorRange(p.x, p.y);
          return p.x >= valid_x_min_ && p.x <= valid_x_max_ &&
              p.y >= valid_y_min_ && p.y <= valid_y_max_ &&
              p.z > general_ground_noise_config_.ground_reference_z - 0.3 &&
              p.z <= valid_z_max_ && range >= valid_range_min_ && range <= valid_range_max_;
        });
    last_general_ground_noise_stats_.elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - noise_start).count();
    last_foreground_count_ = static_cast<int>(foreground_idx.size());

#ifdef GENERAL_REPLAY_OFFLINE
    if (offline_capture_) offline_foreground_before_ror_ = foreground_idx;
#endif
    last_ror_input_count_ = static_cast<int>(foreground_idx.size());
    last_ror_skipped_ = false;

    {
      ScopedTimer timer(radius_outlier_removal_ms_);
      if (enable_radius_outlier_removal_) {
        if (skip_ror_when_dense_ &&
            static_cast<int>(foreground_idx.size()) > ror_max_input_points_) {
          last_ror_skipped_ = true;
        } else {
          std::vector<int> filtered_foreground;
          std::vector<int> ror_rejected;
          radiusOutlierRemoval(points, foreground_idx, filtered_foreground, ror_rejected);
          foreground_idx.swap(filtered_foreground);
          last_foreground_count_ = static_cast<int>(foreground_idx.size());
        }
      }
    }

    {
      ScopedTimer timer(t_debug_cloud);
      if (publish_debug_cloud_this_frame) {
        if (worker_running_ && rclcpp::ok()) {
          publishDebugCloud(out_header, points, foreground_idx, pub_foreground_,
                            publish_debug_foreground_points_, last_debug_foreground_published_points_);
          if (keep_bg) {
            publishDebugCloud(out_header, points, background_idx, pub_static_background_,
                              true, last_debug_valid_published_points_); // unused stat
          }
        }
      }
    }

    const bool publish_confirmed = !usesStaticBackground() || background_ready_ ||
                                   !require_manual_background_learning_;
    std::vector<ObjectFeature> features;
    std::vector<ClusterResult> cluster_results;
    if (publish_confirmed && !(disable_alerts_during_background_learning_ && learning_active_)) {
      {
        ScopedTimer timer(clustering_ms_);
        const double msg_stamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        cluster_results = clusterer_ ? clusterer_->cluster(points, foreground_idx, processed_frame_count_, msg_stamp)
                                     : std::vector<ClusterResult>{};
        separateVehicleUpperAttachments(points,cluster_results);
      }
      last_cluster_count_ = static_cast<int>(cluster_results.size());
      const auto cluster_diag = clusterer_ ? clusterer_->diagnostics() : ClusterDiagnostics{};
      bev_build_ms_ = cluster_diag.bev_build_ms;
      connected_component_ms_ = cluster_diag.cc_bfs_ms;
      cluster_point_gather_ms_ = cluster_diag.point_gather_ms;
      cluster_filter_ms_ = cluster_diag.filter_ms;
      z_profile_build_ms_ = cluster_diag.z_profile_build_ms;
      z_profile_edge_diag_ms_ = cluster_diag.z_profile_edge_diag_ms;
      last_occupied_cell_count_ = cluster_diag.occupied_cell_count;
      last_connected_component_count_ = cluster_diag.connected_component_count;
      last_valid_cluster_count_ = cluster_diag.valid_cluster_count;
      last_tiny_component_count_ = cluster_diag.tiny_component_count;
      last_single_cell_component_count_ = cluster_diag.single_cell_component_count;
      last_two_cell_component_count_ = cluster_diag.two_cell_component_count;
      last_largest_component_cells_ = cluster_diag.largest_component_cells;
      last_largest_component_points_ = cluster_diag.largest_component_points;
      last_median_component_cells_ = cluster_diag.median_component_cells;
      last_median_component_points_ = cluster_diag.median_component_points;
      last_component_cell_histogram_ = cluster_diag.component_cell_histogram;
      last_component_point_histogram_ = cluster_diag.component_point_histogram;
      last_z_rejected_direct_edges_ = cluster_diag.z_rejected_direct_edges;
      last_bridge_candidates_ = cluster_diag.bridge_candidates;
      last_bridge_accepted_ = cluster_diag.bridge_accepted;
      last_z_rejected_bridge_edges_ = cluster_diag.z_rejected_bridge_edges;
      last_layer_cell_count_ = cluster_diag.layer_cell_count;
      last_strong_layer_cell_count_ = cluster_diag.strong_layer_cell_count;
      last_weak_layer_cell_count_ = cluster_diag.weak_layer_cell_count;
      last_max_layers_per_xy_cell_ = cluster_diag.max_layers_per_xy_cell;
      last_average_layers_per_xy_cell_ = cluster_diag.average_layers_per_xy_cell;
      last_weak_support_candidates_ = cluster_diag.weak_support_candidates;
      last_weak_support_accepted_ = cluster_diag.weak_support_accepted;
      last_weak_support_density_rejected_ = cluster_diag.weak_support_density_rejected;
      z_layer_build_ms_ = cluster_diag.z_layer_build_ms;
      density_build_ms_ = cluster_diag.density_build_ms;
      updateShapeGraphDebugStats(cluster_results);

      // Reuse the same original-cluster ownership for visibility and raw-box support.
      if (visibility_frame_valid_ || raw_vehicle_box_refinement_enabled_) {
        frame_cluster_owner_.assign(points.size(), -1);
        for (const auto & c : cluster_results) {
          for (int idx : c.indices) {
            if (idx >= 0 && static_cast<size_t>(idx) < frame_cluster_owner_.size())
              frame_cluster_owner_[idx] = c.id;
          }
        }
      }
      if (visibility_frame_valid_) {
        const auto depth_start = std::chrono::steady_clock::now();
        visibility_model_.buildGeneralDepth(points, frame_cluster_owner_);
        visibility_depth_build_ms_ = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - depth_start).count();
      }

      {
        ScopedTimer timer(feature_build_ms_);
        features = buildFeaturesFromClusterResults(points, cluster_results, foreground_idx);
      }
      // Ground-only Raw refinement must not reintroduce removed ground points.
      std::vector<int> noise_safe_roi;
      const std::vector<int> * refinement_indices = &foreground_idx;
      if (usesStaticBackground()) {
        noise_safe_roi.reserve(roi_filtered_idx.size());
        const auto & mask = general_ground_noise_filter_.candidates();
        for (int i : roi_filtered_idx)
          if (i >= 0 && (size_t(i) >= mask.size() || !mask[size_t(i)])) noise_safe_roi.push_back(i);
        refinement_indices = &noise_safe_roi;
      }
      refineVehicleFeaturesWithRawBox(points, frame_cluster_owner_, features, refinement_indices);
    }
#ifdef GENERAL_REPLAY_OFFLINE
    if (offline_capture_) {
      OfflinePreparedFrame f;
      f.tf = tf; f.output_frame = output_frame; f.points = points;
      f.compact_to_original = offline_compact_to_original_;
      f.valid = valid_idx; f.roi = roi_filtered_idx;
      f.ignored = roi_ignored_idx; f.hard_ignored = roi_hard_ignored_idx;
      f.foreground_before_ror = offline_foreground_before_ror_;
      f.foreground = foreground_idx; f.similarity = last_point_bg_similarity_;
      f.clusters = cluster_results; f.features = features;
      f.owners = frame_cluster_owner_;
      f.enabled = publish_confirmed && !(disable_alerts_during_background_learning_ && learning_active_);
      f.publish_confirmed = publish_confirmed;
      f.background_ready = background_ready_; f.learning_active = learning_active_;
      f.has_ring = last_input_has_ring_; f.has_time = last_input_has_time_;
      f.tf_fallback = offline_tf_fallback_;
      f.foreground_count = last_foreground_count_; f.static_count = last_static_count_;
      f.ror_input = last_ror_input_count_; f.ror_skipped = last_ror_skipped_;
      offline_capture_(f);
      if (offline_prefix_only_) return;
    }
#endif
    finishGeneralFrame(out_header, points, foreground_idx, cluster_results, features,
        publish_confirmed && !(disable_alerts_during_background_learning_ && learning_active_),
        publish_confirmed, publish_debug_cloud_this_frame, t_debug_cloud, start_time);
  }

  // Shared production suffix. Offline replay supplies only pre-tracking frame data.
  void finishGeneralFrame(const std_msgs::msg::Header & out_header,
      const std::vector<PointXYZIRT> & points, const std::vector<int> & foreground_idx,
      const std::vector<ClusterResult> & cluster_results, std::vector<ObjectFeature> & features,
      bool enabled, bool publish_confirmed, bool publish_debug_cloud_this_frame,
      double t_debug_cloud, std::chrono::steady_clock::time_point start_time) {
    std::vector<int> cluster_points;
    if (enabled) {
#ifdef GENERAL_REPLAY_OFFLINE
      if (offline_before_tracking_) offline_before_tracking_();
#endif
      if (visibility_frame_valid_) {
        const auto cluster_start = std::chrono::steady_clock::now();
        for (auto & f : features) {
          VisibilityBox box{f.min_x, f.max_x, f.min_y, f.max_y, f.min_z, f.max_z};
          f.visibility = visibility_model_.evaluateBox(box, f.id, false);
        }
        visibility_cluster_ms_ = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - cluster_start).count();
      }

      if (publish_debug_cloud_this_frame && publish_debug_clusters_ && (pub_clusters_ ? pub_clusters_->get_subscription_count() : 0u) > 0) {
        for (const auto & c : cluster_results) {
          cluster_points.insert(cluster_points.end(), c.indices.begin(), c.indices.end());
        }
      }

#ifdef GENERAL_REPLAY_OFFLINE
      if (offline_initial_visibility_) offline_initial_visibility_(features);
#endif
      {
        ScopedTimer timer(tracking_ms_);
        updateTracks(features, stampSeconds(out_header), points, foreground_idx);
      }
    } else {
      track_memory_removed_ += tracks_.size();
      tracks_.clear();
      last_lost_vehicle_count_ = last_active_track_count_ = 0;
      features.clear();
      updateShapeGraphDebugStats({});
    }

    last_candidates_ = std::move(features);
#ifdef GENERAL_REPLAY_OFFLINE
    // Feature pointers are frame-local: project public messages while they are alive.
    if (offline_public_) publishObjects(out_header);
    if (offline_after_frame_) offline_after_frame_();
#endif

    {
      ScopedTimer timer(t_debug_cloud);
      if (publish_debug_cloud_this_frame) {
        if (worker_running_ && rclcpp::ok()) {
          publishDebugCloud(out_header, points, cluster_points, pub_clusters_,
                            publish_debug_clusters_, last_debug_clusters_published_points_);
        }
      }
    }
    debug_cloud_build_ms_ = t_debug_cloud;

    if ((clusterer_ && clusterer_->name() == "shape_graph") || shape_graph_compute_policy_ == "always") {
      shape_graph_computed_this_frame_ = 1;
      shape_graph_skipped_this_frame_ = 0;
    } else { // manual_prepare
      if (shape_graph_computed_this_frame_ > 0) {
        shape_graph_computed_this_frame_ = 1;
        shape_graph_skipped_this_frame_ = 0;
      } else {
        shape_graph_computed_this_frame_ = 0;
        shape_graph_skipped_this_frame_ = 1;
      }
    }

    double t_pub = 0.0;
    {
      ScopedTimer timer(t_pub);
      if (worker_running_ && rclcpp::ok()) {
        publishObjects(out_header);
        publishRecordingFrame(out_header, points, foreground_idx);
      }
    }

    double t_markers = 0.0;
    {
      ScopedTimer timer(t_markers);
      if (worker_running_ && rclcpp::ok()) {
        publishMarkers(out_header, publish_confirmed && hasDisplayableTrack());
        publishShapeGraphMarkers(out_header);
        publishShapeMemoryMarkers(out_header);
        publishRoiMarker(out_header);
        publishVisibilityMarkers(out_header);
      }
    }
    marker_build_ms_ = t_markers;

    {
      ScopedTimer timer(t_pub);
      if (worker_running_ && rclcpp::ok()) {
        publishTables(out_header);
        publishStatus();
      }
    }
    publish_ms_ = t_pub;

    fem_refine_ms_ = 0.0;
    total_pipeline_ms_ = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start_time).count();
  }

