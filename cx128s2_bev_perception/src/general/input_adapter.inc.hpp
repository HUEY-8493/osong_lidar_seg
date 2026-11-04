  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr createCloudPublisher(
      const std::string & topic) {
    if (!publish_debug_topics_) return nullptr;
    return create_publisher<sensor_msgs::msg::PointCloud2>(topic, 10);
  }

  int64_t bgKey(const PointXYZIRT & p) const {
    return hash2d(static_cast<int>(std::floor(p.x / std::max(0.05, bg_x_res_))),
                  static_cast<int>(std::floor(p.y / std::max(0.05, bg_y_res_))));
  }

  int64_t groundKey(const PointXYZIRT & p) const {
    return hash2d(static_cast<int>(std::floor(p.x / std::max(0.05, x_bin_size_))),
                  static_cast<int>(std::floor(p.y / std::max(0.05, y_bin_size_))));
  }

  int64_t bevKey(const PointXYZIRT & p) const {
    return hash2d(static_cast<int>(std::floor(p.x / std::max(0.05, bev_x_res_))),
                  static_cast<int>(std::floor(p.y / std::max(0.05, bev_y_res_))));
  }

  double stampSeconds(const std_msgs::msg::Header & h) const {
    const double t = h.stamp.sec + 1e-9 * h.stamp.nanosec;
    return t > 1e-6 ? t : now().seconds();
  }

  bool passValidCrop(const PointXYZIRT & p) const {
    const double r = sensorRange(p.x, p.y);
    return p.x >= valid_x_min_ && p.x <= valid_x_max_ && p.y >= valid_y_min_ &&
           p.y <= valid_y_max_ && p.z >= valid_z_min_ && p.z <= valid_z_max_ &&
           r >= valid_range_min_ && r <= valid_range_max_;
  }

  bool insideRoi(double x, double y) const {
    if (!roi_enabled_) return true;
    if (roi_shape_ == "box") {
      return x >= roi_x_min_ && x <= roi_x_max_ && y >= roi_y_min_ && y <= roi_y_max_;
    }
    if (roi_polygon_x_.size() < 3 || roi_polygon_x_.size() != roi_polygon_y_.size()) return true;
    bool inside = false;
    for (size_t i = 0, j = roi_polygon_x_.size() - 1; i < roi_polygon_x_.size(); j = i++) {
      const bool intersect = ((roi_polygon_y_[i] > y) != (roi_polygon_y_[j] > y)) &&
          (x < (roi_polygon_x_[j] - roi_polygon_x_[i]) * (y - roi_polygon_y_[i]) /
                       (roi_polygon_y_[j] - roi_polygon_y_[i] + 1e-9) +
                   roi_polygon_x_[i]);
      if (intersect) inside = !inside;
    }
    return inside;
  }

  bool pointInPolygon(double x, double y, const std::vector<double> & xs,
                      const std::vector<double> & ys) const {
    if (xs.size() < 3 || xs.size() != ys.size()) return false;
    bool inside = false;
    for (size_t i = 0, j = xs.size() - 1; i < xs.size(); j = i++) {
      const bool intersect = ((ys[i] > y) != (ys[j] > y)) &&
          (x < (xs[j] - xs[i]) * (y - ys[i]) / (ys[j] - ys[i] + 1e-9) + xs[i]);
      if (intersect) inside = !inside;
    }
    return inside;
  }

  bool pointInZone(double x, double y, const RoiZone & zone) const {
    if (!zone.enabled) return false;
    if (zone.shape == "box") {
      return x >= zone.x_min && x <= zone.x_max && y >= zone.y_min && y <= zone.y_max;
    }
    return pointInPolygon(x, y, zone.polygon_x, zone.polygon_y);
  }

  RoiDecision pointPolicy(double x, double y) const {
    if (!use_roi_policy_ || roi_zones_.empty()) return RoiDecision::KEEP;
    bool has_watch = false;
    bool inside_watch = false;
    for (const auto & zone : roi_zones_) {
      if (!zone.enabled) continue;
      if (zone.type == "hard_ignore" && pointInZone(x, y, zone)) return RoiDecision::HARD_IGNORE;
    }
    for (const auto & zone : roi_zones_) {
      if (!zone.enabled) continue;
      if (zone.type == "ignore" && pointInZone(x, y, zone)) return RoiDecision::IGNORE;
    }
    for (const auto & zone : roi_zones_) {
      if (!zone.enabled || zone.type != "watch") continue;
      has_watch = true;
      if (pointInZone(x, y, zone)) inside_watch = true;
    }
    if (has_watch && inside_watch) return RoiDecision::KEEP;
    if ((require_watch_roi_ || discard_points_outside_watch_roi_) && has_watch) {
      return RoiDecision::OUTSIDE_WATCH;
    }
    return RoiDecision::KEEP;
  }

  void applyRoiPolicyToIndices(const std::vector<PointXYZIRT> & points,
                               const std::vector<int> & input_idx,
                               std::vector<int> & output_idx,
                               std::vector<int> & ignored_idx,
                               std::vector<int> & hard_ignored_idx) const {
    output_idx.clear();
    ignored_idx.clear();
    hard_ignored_idx.clear();
    output_idx.reserve(input_idx.size());

    const size_t num_input = input_idx.size();
    if (enable_parallel_processing_) {
      std::vector<uint8_t> decisions(num_input, 0);
      #pragma omp parallel for
      for (size_t i = 0; i < num_input; ++i) {
        const auto & p = points[input_idx[i]];
        switch (pointPolicy(p.x, p.y)) {
          case RoiDecision::KEEP:
            decisions[i] = 0;
            break;
          case RoiDecision::HARD_IGNORE:
            decisions[i] = 1;
            break;
          case RoiDecision::IGNORE:
          case RoiDecision::OUTSIDE_WATCH:
            decisions[i] = 2;
            break;
        }
      }
      for (size_t i = 0; i < num_input; ++i) {
        int idx = input_idx[i];
        if (decisions[i] == 0) {
          output_idx.push_back(idx);
        } else if (decisions[i] == 1) {
          hard_ignored_idx.push_back(idx);
        } else {
          ignored_idx.push_back(idx);
        }
      }
    } else {
      for (int idx : input_idx) {
        const auto & p = points[idx];
        switch (pointPolicy(p.x, p.y)) {
          case RoiDecision::KEEP:
            output_idx.push_back(idx);
            break;
          case RoiDecision::HARD_IGNORE:
            hard_ignored_idx.push_back(idx);
            break;
          case RoiDecision::IGNORE:
          case RoiDecision::OUTSIDE_WATCH:
            ignored_idx.push_back(idx);
            break;
        }
      }
    }
  }

  int countRoiZones(const std::string & type) const {
    return static_cast<int>(std::count_if(roi_zones_.begin(), roi_zones_.end(),
        [&](const RoiZone & z) { return z.enabled && z.type == type; }));
  }

  std::unique_ptr<IClusterer> createClusterer() const {
    // TODO: Add DBSCAN 2D clusterer.
    // TODO: Add Euclidean XY clusterer.
    // TODO: Add range-adaptive BEV clusterer.
    // TODO: Add ROI-aware clusterer.
    // TODO: Add track-assisted clusterer.
    if (clustering_method_ == "none") {
      return std::make_unique<NoOpClusterer>();
    }
    if (clustering_method_ == "shape_graph" || clustering_method_ == "multi_view_shape_graph") {
      return std::make_unique<ShapeGraphClusterer>(
          shape_graph_voxel_size_, bev_x_res_, bev_y_res_,
          shape_graph_max_edge_dist_near_, shape_graph_max_edge_dist_mid_,
          shape_graph_max_edge_dist_long_, shape_graph_top_weight_, shape_graph_side_weight_,
          shape_graph_front_weight_, shape_graph_min_edge_score_,
          shape_graph_min_component_nodes_near_, shape_graph_min_component_nodes_mid_,
          shape_graph_min_component_nodes_long_, shape_graph_use_top_view_,
          shape_graph_use_side_view_, shape_graph_use_front_view_, min_cluster_cells_near_,
          min_cluster_cells_mid_, min_cluster_cells_long_, min_cluster_points_near_,
          min_cluster_points_mid_, min_cluster_points_long_, near_range_max_, mid_range_max_);
    }
    if (clustering_method_ != "bev_connected_component") {
      RCLCPP_WARN(get_logger(), "Unknown clustering_method='%s'. Falling back to bev_connected_component.",
                  clustering_method_.c_str());
    }
    return std::make_unique<BevConnectedComponentClusterer>(
        bev_x_res_, bev_y_res_, cluster_connectivity_, min_points_per_occupied_cell_, min_cluster_cells_near_,
        min_cluster_cells_mid_, min_cluster_cells_long_, min_cluster_points_near_,
        min_cluster_points_mid_, min_cluster_points_long_, near_range_max_, mid_range_max_,
        cluster_z_guard_enabled_, cluster_gap_bridging_enabled_, cluster_neighbor_max_z_gap_,
        cluster_bridge_max_z_gap_, cluster_gap_cells_near_, cluster_gap_cells_mid_, cluster_gap_cells_long_,
        cluster_z_layer_enabled_, cluster_z_layer_resolution_, cluster_weak_support_enabled_,
        cluster_density_enabled_, cluster_density_window_cells_, cluster_weak_support_min_density_,
        cluster_z_profile_diagnostics_enabled_, cluster_z_profile_bin_size_, valid_z_min_);
  }

  bool lookupTransform(const sensor_msgs::msg::PointCloud2::SharedPtr & msg, tf2::Transform & tf,
                       std::string & output_frame) {
    output_frame = msg->header.frame_id;
#ifdef GENERAL_REPLAY_OFFLINE
    offline_tf_fallback_ = false;
#endif
    if (!use_tf_transform_ || target_frame_.empty() || msg->header.frame_id == target_frame_) {
      return true;
    }

    // Crane Beam Work Mode: Startup Frozen Reference Transform (Candidate B)
    // Prevents dynamic Gazebo moving TF from cheating sensor-motion compensation in crane perception.
    // The initial transform T(0) is locked at startup and reused for all subsequent frames.
    if (perception_mode_ == "crane_beam_work" && crane_beam_config_.freeze_sensor_transform) {
      if (has_frozen_crane_transform_) {
        tf = frozen_crane_transform_;
        output_frame = target_frame_;
        return true;
      }
      try {
        const auto stamped = tf_buffer_->lookupTransform(
            target_frame_, msg->header.frame_id, msg->header.stamp,
            rclcpp::Duration::from_seconds(tf_timeout_sec_));
        tf2::fromMsg(stamped.transform, tf);
        frozen_crane_transform_ = tf;
        has_frozen_crane_transform_ = true;
        frozen_crane_input_frame_ = msg->header.frame_id;
        frozen_crane_target_frame_ = target_frame_;
        output_frame = target_frame_;

        const auto trans = tf.getOrigin();
        const auto rot = tf.getRotation();
        RCLCPP_INFO(get_logger(),
                    "[CraneBeam] Startup Sensor Transform FROZEN: input='%s' -> reference='%s', "
                    "translation=(%.3f, %.3f, %.3f), quat=(%.4f, %.4f, %.4f, %.4f). "
                    "Dynamic Gazebo sensor-motion compensation DISABLED for crane perception.",
                    frozen_crane_input_frame_.c_str(), frozen_crane_target_frame_.c_str(),
                    trans.x(), trans.y(), trans.z(), rot.x(), rot.y(), rot.z(), rot.w());
        return true;
      } catch (const tf2::TransformException & ex) {
        try {
          const auto stamped = tf_buffer_->lookupTransform(
              target_frame_, msg->header.frame_id, rclcpp::Time(0),
              rclcpp::Duration::from_seconds(tf_timeout_sec_));
          tf2::fromMsg(stamped.transform, tf);
          frozen_crane_transform_ = tf;
          has_frozen_crane_transform_ = true;
          frozen_crane_input_frame_ = msg->header.frame_id;
          frozen_crane_target_frame_ = target_frame_;
          output_frame = target_frame_;

          const auto trans = tf.getOrigin();
          const auto rot = tf.getRotation();
          RCLCPP_INFO(get_logger(),
                      "[CraneBeam] Startup Sensor Transform FROZEN (fallback Time(0)): input='%s' -> reference='%s', "
                      "translation=(%.3f, %.3f, %.3f), quat=(%.4f, %.4f, %.4f, %.4f).",
                      frozen_crane_input_frame_.c_str(), frozen_crane_target_frame_.c_str(),
                      trans.x(), trans.y(), trans.z(), rot.x(), rot.y(), rot.z(), rot.w());
          return true;
        } catch (const tf2::TransformException & ex2) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                               "[CraneBeam] TF lookup failed for initial freeze %s -> %s: %s (fallback failed: %s)",
                               msg->header.frame_id.c_str(), target_frame_.c_str(), ex.what(), ex2.what());
          return fallback_to_raw_frame_;
        }
      }
    }

    // Standard dynamic TF lookup for default mode (or when freeze_sensor_transform is disabled)
    try {
      const auto stamped = tf_buffer_->lookupTransform(
          target_frame_, msg->header.frame_id, msg->header.stamp,
          rclcpp::Duration::from_seconds(tf_timeout_sec_));
      tf2::fromMsg(stamped.transform, tf);
      output_frame = target_frame_;
      RCLCPP_DEBUG(get_logger(), "TF success %s -> %s", msg->header.frame_id.c_str(),
                   target_frame_.c_str());
      return true;
    } catch (const tf2::TransformException & ex) {
      try {
#ifdef GENERAL_REPLAY_OFFLINE
        offline_tf_fallback_ = true;
#endif
        const auto stamped = tf_buffer_->lookupTransform(
            target_frame_, msg->header.frame_id, rclcpp::Time(0),
            rclcpp::Duration::from_seconds(tf_timeout_sec_));
        tf2::fromMsg(stamped.transform, tf);
        output_frame = target_frame_;
        return true;
      } catch (const tf2::TransformException & ex2) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "TF failed %s -> %s: %s (fallback: %s)",
                             msg->header.frame_id.c_str(), target_frame_.c_str(), ex.what(), ex2.what());
        return fallback_to_raw_frame_;
      }
    }
  }

  bool readCloud(const sensor_msgs::msg::PointCloud2::SharedPtr & msg, const tf2::Transform & tf,
                 bool do_transform, std::vector<PointXYZIRT> & points) {
    bool has_x = false, has_y = false, has_z = false, has_i = false, has_ring = false, has_time = false;
    for (const auto & f : msg->fields) {
      has_x |= f.name == "x";
      has_y |= f.name == "y";
      has_z |= f.name == "z";
      has_i |= f.name == "intensity";
      has_ring |= f.name == "ring";
      has_time |= f.name == "time";
    }
    if (!has_x || !has_y || !has_z) {
      RCLCPP_ERROR(get_logger(), "Input PointCloud2 missing required x/y/z fields.");
      return false;
    }
    if (!has_ring && !warned_missing_ring_) {
      RCLCPP_WARN(get_logger(), "Input PointCloud2 has no ring field. Output ring will be 0.");
      warned_missing_ring_ = true;
    }
    last_input_has_time_ = has_time;
    last_input_has_ring_ = has_ring;

    sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");
    std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> it_i;
    std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<uint16_t>> it_r;
    std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> it_t;
    if (has_i) it_i = std::make_unique<sensor_msgs::PointCloud2ConstIterator<float>>(*msg, "intensity");
    if (has_ring) it_r = std::make_unique<sensor_msgs::PointCloud2ConstIterator<uint16_t>>(*msg, "ring");
    if (has_time) it_t = std::make_unique<sensor_msgs::PointCloud2ConstIterator<float>>(*msg, "time");

    const size_t n = static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);
    points.reserve(n);
#ifdef GENERAL_REPLAY_OFFLINE
    offline_compact_to_original_.clear();
#endif
    for (size_t k = 0; k < n; ++k, ++it_x, ++it_y, ++it_z) {
      PointXYZIRT p;
      p.x = *it_x;
      p.y = *it_y;
      p.z = *it_z;
      if (it_i) {
        p.intensity = **it_i;
        ++(*it_i);
      }
      if (it_r) {
        p.ring = **it_r;
        ++(*it_r);
      }
      if (it_t) {
        p.time = **it_t;
        ++(*it_t);
      }
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
      const double raw_range = std::hypot(p.x, p.y);
      if (!std::isfinite(raw_range) || raw_range < 1e-6) continue;
#ifdef GENERAL_REPLAY_OFFLINE
      offline_compact_to_original_.push_back(k);
#endif
      if (do_transform) {
        tf2::Vector3 v(p.x, p.y, p.z);
        v = tf * v;
        p.x = static_cast<float>(v.x());
        p.y = static_cast<float>(v.y());
        p.z = static_cast<float>(v.z());
      }
      points.push_back(p);
    }
    return true;
  }

