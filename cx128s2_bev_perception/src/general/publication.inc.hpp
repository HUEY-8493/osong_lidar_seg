  void publishObjects(const std_msgs::msg::Header & header) {
    const auto build_start = std::chrono::steady_clock::now();
    cx128s2_bev_perception::msg::TrackedObjectArray detected, tracked, motion, vehicles, identity;
    detected.header = tracked.header = motion.header = vehicles.header = identity.header = header;
    int moving = 0, stationary = 0, vehicle_count = 0, unknown_count = 0, rejected = 0;
    for (const auto & tr : tracks_) {
      if (persistentVehicle(tr)) {
        auto remembered = toMsg(tr);
        const auto & mem = tr.observation_memory;
        if (mem.vehicle_state == cx128s2_bev_perception::vehicle_surface::VehicleLifecycle::ACTIVE)
          remembered.lifecycle_state = tr.lost_frames == 0 ? "ACTIVE" : "TRACKED_PREDICTED";
        const auto pose = cx128s2_bev_perception::vehicle_surface::externalVehiclePose(
            mem.vehicle_state, {outputX(tr), outputY(tr)},
            {mem.final_predicted_x, mem.final_predicted_y}, mem.final_predicted_valid);
        remembered.position.x = pose.x;
        remembered.position.y = pose.y;
        if (dormantVehicle(tr)) {
          // The old velocity is retained internally but is not a meaningful
          // external motion estimate after the predictive horizon.
          remembered.velocity.x = remembered.velocity.y = 0.0;
          remembered.speed = 0.0;
          remembered.motion_state = "DORMANT";
        }
        identity.objects.push_back(std::move(remembered));
      }
      if (!physicalOutputAllowed(tr)) continue;
      const auto obj = toMsg(tr);
      if (obj.lifecycle_state == "REJECTED") {
        rejected++;
        continue;
      }
      // Candidate markers/recordings remain available, but the four public
      // arrays consumed by the bridge describe confirmed detections only.
      if (!tr.detection.is_valid || obj.lifecycle_state == "CANDIDATE") continue;
      motion.objects.push_back(obj);
      detected.objects.push_back(obj);
      if (obj.lifecycle_state == "TRACKED" || obj.lifecycle_state == "TRACKED_PREDICTED" ||
          obj.lifecycle_state == "LOST" || obj.lifecycle_state == "LOST_PREDICTIVE" ||
          obj.lifecycle_state == "DORMANT") tracked.objects.push_back(obj);
      if (obj.class_name == "VEHICLE") {
        vehicles.objects.push_back(obj);
        vehicle_count++;
      }
      if (obj.class_name == "UNKNOWN") unknown_count++;
      if (obj.motion_state == "STATIONARY") stationary++;
      else moving++;
    }
    last_vehicle_count_ = vehicle_count;
    last_unknown_count_ = unknown_count;
    last_rejected_candidate_count_ = rejected;
    last_moving_count_ = moving;
    last_stationary_count_ = stationary;
    object_arrays_build_ms_ = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - build_start).count();
    object_arrays_bytes_ = detected.objects.size() * sizeof(cx128s2_bev_perception::msg::TrackedObject) +
        tracked.objects.size() * sizeof(cx128s2_bev_perception::msg::TrackedObject) +
        motion.objects.size() * sizeof(cx128s2_bev_perception::msg::TrackedObject) +
        vehicles.objects.size() * sizeof(cx128s2_bev_perception::msg::TrackedObject);
#ifdef GENERAL_REPLAY_OFFLINE
    if (offline_public_) { offline_public_(detected, tracked, motion, vehicles, identity); return; }
#endif
    const auto publish_start = std::chrono::steady_clock::now();
    pub_detected_objects_->publish(detected);
    pub_tracked_objects_->publish(tracked);
    pub_motion_states_->publish(motion);
    pub_vehicle_tracks_->publish(vehicles);
    if (perception_mode_ != "crane_beam_work") pub_vehicle_identity_memory_->publish(identity);
    object_arrays_publish_ms_ = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - publish_start).count();
  }

  void publishRecordingFrame(const std_msgs::msg::Header & header,
                             const std::vector<PointXYZIRT> & points,
                             const std::vector<int> & foreground_idx) {
    if (!publish_recording_frame_ || !pub_recording_frame_ ||
        pub_recording_frame_->get_subscription_count() == 0) {
      return;
    }
    cx128s2_bev_perception::msg::PerceptionRecordingFrame frame;
    frame.header = header;
    frame.source_point_count = static_cast<uint32_t>(points.size());
    frame.foreground_point_count = static_cast<uint32_t>(foreground_idx.size());
    if (recording_include_full_cloud_) {
      std::vector<int> all_idx(points.size());
      std::iota(all_idx.begin(), all_idx.end(), 0);
      frame.transformed_cloud = makeCloud(header, points, all_idx);
    }
    if (recording_include_foreground_cloud_) {
      frame.foreground_cloud = makeCloud(header, points, foreground_idx);
    }
    for (const auto & tr : tracks_) {
      if (!physicalOutputAllowed(tr)) continue;
      cx128s2_bev_perception::msg::RecordingObjectCluster cluster;
      cluster.track_id = tr.id;
      cluster.cluster_id = -1;
      cluster.yaw = static_cast<float>(getTrackYaw(tr));
      cluster.observed = false;
      cluster.object = toMsg(tr);
      const auto found = recording_track_features_.find(tr.id);
      if (tr.lost_frames == 0 && found != recording_track_features_.end() && found->second != nullptr) {
        const auto & feature = *found->second;
        cluster.cluster_id = feature.id;
        cluster.observed = true;
        cluster.points = makeCloud(header, points, feature.indices);
        ++frame.cluster_count;
      }
      frame.objects.push_back(std::move(cluster));
    }
    pub_recording_frame_->publish(frame);
  }

  int publishDebugCloud(const std_msgs::msg::Header & header, const std::vector<PointXYZIRT> & points,
                        const std::vector<int> & indices,
                        const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub,
                        bool enabled, int & published_count) const {
    published_count = 0;
    if (!enabled || !pub || pub->get_subscription_count() == 0) return 0;
    
    if (debug_cloud_max_points_ <= 0 || static_cast<int>(indices.size()) <= debug_cloud_max_points_) {
      publishCloud(header, points, indices, pub);
      published_count = static_cast<int>(indices.size());
      return published_count;
    }

    std::vector<int> downsampled;
    downsampled.reserve(debug_cloud_max_points_);
    const double stride = static_cast<double>(indices.size()) / debug_cloud_max_points_;
    for (int i = 0; i < debug_cloud_max_points_; ++i) {
      int idx = static_cast<int>(i * stride);
      if (idx >= static_cast<int>(indices.size())) idx = static_cast<int>(indices.size()) - 1;
      downsampled.push_back(indices[idx]);
    }
    publishCloud(header, points, downsampled, pub);
    published_count = static_cast<int>(downsampled.size());
    return published_count;
  }

  bool usesStaticBackground() const { return background_mode_ == "static"; }

  bool hasDisplayableTrack() const {
    if (learning_active_ || (usesStaticBackground() && !background_ready_)) return false;
    for (const auto & tr : tracks_) {
      if (!physicalOutputAllowed(tr)) continue;
      if (tr.detection.lifecycle_state == "REJECTED") continue;
      return true;
    }
    return false;
  }

  void publishVisibilityMarkers(const std_msgs::msg::Header & header) {
    if (!pub_visibility_markers_ || !visibility_publish_markers_ ||
        pub_visibility_markers_->get_subscription_count() == 0) return;
    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker clear;
    clear.header = header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear);
    if (!visibility_frame_valid_) {
      pub_visibility_markers_->publish(arr);
      return;
    }
    const auto & pose = visibility_model_.pose();
    const auto origin = pose.getOrigin();
    const auto & cfg = visibility_model_.config();
    auto point = [](const tf2::Vector3 & v) {
      geometry_msgs::msg::Point p; p.x = v.x(); p.y = v.y(); p.z = v.z(); return p;
    };
    auto marker = [&](const std::string & ns, int id, int type, float r, float g, float b) {
      visualization_msgs::msg::Marker m;
      m.header = header; m.ns = ns; m.id = id; m.type = type;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.orientation.w = 1.0;
      m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 0.85f;
      m.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_);
      return m;
    };
    auto origin_marker = marker("visibility_sensor_origin", 0,
        visualization_msgs::msg::Marker::SPHERE, 1.0f, 1.0f, 1.0f);
    origin_marker.pose.position = point(origin);
    origin_marker.scale.x = origin_marker.scale.y = origin_marker.scale.z = 0.25;
    arr.markers.push_back(origin_marker);

    auto fov = marker("visibility_fov_boundary", 0,
        visualization_msgs::msg::Marker::LINE_LIST, 0.2f, 0.7f, 1.0f);
    fov.scale.x = 0.045;
    const double pi = 3.14159265358979323846;
    auto worldRay = [&](double az_deg, double el_deg) {
      const double az = az_deg * pi / 180.0, el = el_deg * pi / 180.0;
      return tf2::quatRotate(pose.getRotation(),
          tf2::Vector3(std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), std::sin(el)));
    };
    for (double az : {cfg.azimuth_min_deg, cfg.azimuth_max_deg}) {
      for (double el : {cfg.elevation_min_deg, cfg.elevation_max_deg}) {
        fov.points.push_back(point(origin));
        fov.points.push_back(point(origin + worldRay(az, el) * std::min(30.0, cfg.max_range_m)));
      }
    }
    arr.markers.push_back(fov);

    auto blind = marker("visibility_ground_intersection", 0,
        visualization_msgs::msg::Marker::LINE_STRIP, 1.0f, 0.4f, 0.1f);
    blind.scale.x = 0.08;
    for (int i = 0; i <= 24; ++i) {
      const double az = cfg.azimuth_min_deg +
          (cfg.azimuth_max_deg - cfg.azimuth_min_deg) * i / 24.0;
      const auto ray = worldRay(az, cfg.elevation_min_deg);
      if (ray.z() >= -1e-6 || origin.z() <= cfg.ground_z) continue;
      const double t = (cfg.ground_z - origin.z()) / ray.z();
      if (t >= cfg.min_range_m && t <= std::min(cfg.max_range_m, 100.0))
        blind.points.push_back(point(origin + ray * t));
    }
    if (blind.points.size() >= 2) arr.markers.push_back(blind);

    auto color = [](cx128s2_bev_perception::VisibilityReason r) {
      switch (r) {
        case cx128s2_bev_perception::VisibilityReason::OBSERVABLE: return std::array<float,3>{0.1f, 0.9f, 0.2f};
        case cx128s2_bev_perception::VisibilityReason::OUT_OF_HORIZONTAL_FOV:
        case cx128s2_bev_perception::VisibilityReason::OUT_OF_VERTICAL_FOV: return std::array<float,3>{0.5f, 0.5f, 0.5f};
        case cx128s2_bev_perception::VisibilityReason::NEAR_BLIND:
        case cx128s2_bev_perception::VisibilityReason::TOO_CLOSE: return std::array<float,3>{1.0f, 0.4f, 0.1f};
        case cx128s2_bev_perception::VisibilityReason::OCCLUDED_BY_OTHER:
        case cx128s2_bev_perception::VisibilityReason::FULLY_OCCLUDED_LIKELY: return std::array<float,3>{0.9f, 0.1f, 0.1f};
        case cx128s2_bev_perception::VisibilityReason::SELF_OCCLUSION_POSSIBLE: return std::array<float,3>{0.7f, 0.1f, 0.9f};
        case cx128s2_bev_perception::VisibilityReason::VISIBLE_EMPTY: return std::array<float,3>{0.1f, 0.9f, 0.9f};
        default: return std::array<float,3>{0.9f, 0.8f, 0.1f};
      }
    };
    auto static_cells = marker("visibility_static_sampled_cells", 0,
        visualization_msgs::msg::Marker::POINTS, 1.0f, 1.0f, 1.0f);
    static_cells.scale.x = static_cells.scale.y = 0.18;
    for (const auto & cell : visibility_static_debug_cells_) {
      static_cells.points.push_back(point(cell.first));
      const auto rgb = color(cell.second);
      std_msgs::msg::ColorRGBA c;
      c.r = rgb[0]; c.g = rgb[1]; c.b = rgb[2]; c.a = 0.75f;
      static_cells.colors.push_back(c);
    }
    if (!static_cells.points.empty()) arr.markers.push_back(static_cells);
    int id = 0;
    for (const auto & tr : tracks_) {
      if (!physicalOutputAllowed(tr)) continue;
      const auto c = color(tr.visibility.reason);
      auto box = marker("visibility_predicted_box", id,
          visualization_msgs::msg::Marker::CUBE, c[0], c[1], c[2]);
      const auto b = predictedVisibilityBox(tr);
      box.pose.position.x = (b.min_x + b.max_x) * 0.5;
      box.pose.position.y = (b.min_y + b.max_y) * 0.5;
      box.pose.position.z = (b.min_z + b.max_z) * 0.5;
      box.scale.x = std::max(0.05, b.max_x - b.min_x);
      box.scale.y = std::max(0.05, b.max_y - b.min_y);
      box.scale.z = std::max(0.05, b.max_z - b.min_z);
      box.color.a = 0.15f;
      arr.markers.push_back(box);
      auto ray = marker("visibility_track_ray", id,
          visualization_msgs::msg::Marker::LINE_LIST, c[0], c[1], c[2]);
      ray.scale.x = 0.06;
      ray.points.push_back(point(origin));
      ray.points.push_back(box.pose.position);
      arr.markers.push_back(ray);
      if (tr.visibility.has_occluder) {
        auto blocker = marker("visibility_dynamic_occluder", id,
            visualization_msgs::msg::Marker::SPHERE, 1.0f, 0.0f, 0.0f);
        blocker.pose.position.x = tr.visibility.occluder_x;
        blocker.pose.position.y = tr.visibility.occluder_y;
        blocker.pose.position.z = tr.visibility.occluder_z;
        blocker.scale.x = blocker.scale.y = blocker.scale.z = 0.3;
        arr.markers.push_back(blocker);
        auto blocked_ray = marker("visibility_occlusion_ray", id,
            visualization_msgs::msg::Marker::LINE_LIST, 1.0f, 0.0f, 0.0f);
        blocked_ray.scale.x = 0.08;
        blocked_ray.points.push_back(point(origin));
        blocked_ray.points.push_back(blocker.pose.position);
        arr.markers.push_back(blocked_ray);
      }
      ++id;
      if (id >= 80) break;
    }
    id = 0;
    for (const auto & f : last_candidates_) {
      const auto c = color(f.visibility.reason);
      auto m = marker("visibility_cluster", id++,
          visualization_msgs::msg::Marker::SPHERE, c[0], c[1], c[2]);
      m.pose.position.x = f.centroid_x; m.pose.position.y = f.centroid_y;
      m.pose.position.z = f.centroid_z;
      m.scale.x = m.scale.y = m.scale.z = 0.25;
      arr.markers.push_back(m);
      if (id >= 80) break;
    }
    pub_visibility_markers_->publish(arr);
  }

  void publishMarkers(const std_msgs::msg::Header & header, bool has_displayable_track) {
    if (!pub_markers_ || pub_markers_->get_subscription_count() == 0) return;
    visualization_msgs::msg::MarkerArray arr;
    if (!has_displayable_track || learning_active_ || (usesStaticBackground() && !background_ready_)) {
      visualization_msgs::msg::Marker clear;
      clear.header = header;
      clear.action = visualization_msgs::msg::Marker::DELETEALL;
      arr.markers.push_back(clear);
      pub_markers_->publish(arr);
      return;
    }

    visualization_msgs::msg::Marker clear_previous;
    clear_previous.header = header;
    clear_previous.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear_previous);

    for (const auto & tr : tracks_) {
      if (!physicalOutputAllowed(tr)) continue;
      if (tr.detection.lifecycle_state == "REJECTED") continue;
      auto obj = toMsg(tr);

      visualization_msgs::msg::Marker box;
      box.header = header;
      box.ns = dormantVehicle(tr) ? "dormant_vehicle_frozen_box" :
          (lostVehicle(tr) ? "lost_vehicle_predicted_box" : "new_objects");
      box.id = tr.id * 2;
      box.type = visualization_msgs::msg::Marker::CUBE;
      box.action = visualization_msgs::msg::Marker::ADD;
      box.pose.position = obj.position;
      box.pose.orientation.w = 1.0;
      if (persistentVehicle(tr)) {
        box.pose.orientation.z = std::sin(getTrackYaw(tr) * 0.5);
        box.pose.orientation.w = std::cos(getTrackYaw(tr) * 0.5);
      }
      box.scale.x = std::max(0.2, obj.dimensions.x);
      box.scale.y = std::max(0.2, obj.dimensions.y);
      box.scale.z = std::max(0.2, obj.dimensions.z);
      box.color.r = obj.class_name == "VEHICLE" ? 0.1f : 1.0f;
      box.color.g = obj.class_name == "VEHICLE" ? 0.85f : 0.65f;
      box.color.b = 0.2f;
      box.color.a = lostVehicle(tr) ? 0.15f :
          (tr.lost_frames > 0 ? static_cast<float>(predicted_track_marker_alpha_) : 0.35f);
      if (lostVehicle(tr)) {
        box.color.r = dormantVehicle(tr) ? 1.0f : 0.85f;
        box.color.g = dormantVehicle(tr) ? 0.55f : 0.25f;
        box.color.b = dormantVehicle(tr) ? 0.1f : 1.0f;
      }
      box.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_);
      arr.markers.push_back(box);

      visualization_msgs::msg::Marker text = box;
      text.ns = lostVehicle(tr) ? "lost_vehicle_text" : "new_object_text";
      text.id = tr.id * 2 + 1;
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.pose.position.z += std::max(1.0, obj.dimensions.z + 0.8);
      text.scale.z = 0.8;
      text.color.a = 1.0;
      std::ostringstream ss;
      ss << "ID " << obj.id << " " << obj.class_name << "\nState: " << obj.lifecycle_state
         << "\nMotion: " << obj.motion_state << "\nVehicle status: " << obj.vehicle_status << "\nDist: " << std::fixed
         << std::setprecision(1) << obj.distance << "m\nReason: " << obj.decision_reason;
      if (persistentVehicle(tr)) {
        const auto & fifo = tr.observation_memory;
        ss << "\nFIFO: " << cx128s2_bev_perception::vehicle_surface::fifoPhaseName(fifo.fifo_phase)
           << " " << cx128s2_bev_perception::vehicle_surface::directionName(
               fifo.corridor_direction.stable);
        if (inside_vehicle_queue_.front() == tr.id) ss << " FRONT";
      }
      text.text = ss.str();
      arr.markers.push_back(text);
      if (persistentVehicle(tr)) {
        const auto & mem = tr.observation_memory;
        visualization_msgs::msg::Marker surface = box;
        surface.ns = "vehicle_predicted_visible_surface";
        surface.id = tr.id;
        surface.type = visualization_msgs::msg::Marker::POINTS;
        surface.pose = geometry_msgs::msg::Pose{};
        surface.pose.orientation.w = 1.0;
        surface.scale.x = surface.scale.y = 0.12;
        surface.color.r = 0.15f; surface.color.g = 0.65f;
        surface.color.b = 1.0f; surface.color.a = 0.8f;
        surface.points.clear();
        for (const auto & p : mem.predicted_visible_world) {
          geometry_msgs::msg::Point q;
          q.x = p.x; q.y = p.y; q.z = tr.z;
          surface.points.push_back(q);
        }
        arr.markers.push_back(surface);
        if (lostVehicle(tr)) {
          visualization_msgs::msg::Marker observed_anchor = surface;
          observed_anchor.ns = "vehicle_last_observed_position";
          observed_anchor.type = visualization_msgs::msg::Marker::SPHERE;
          observed_anchor.pose.position.x = mem.last_observed_x;
          observed_anchor.pose.position.y = mem.last_observed_y;
          observed_anchor.pose.position.z = mem.vehicle_z_center;
          observed_anchor.scale.x = observed_anchor.scale.y = observed_anchor.scale.z = 0.25;
          observed_anchor.color.r = 0.1f; observed_anchor.color.g = 1.0f;
          observed_anchor.color.b = 0.2f; observed_anchor.color.a = 0.9f;
          if (mem.last_observed_valid) arr.markers.push_back(observed_anchor);
          if (dormantVehicle(tr) && mem.final_predicted_valid) {
            auto frozen = observed_anchor;
            frozen.ns = "vehicle_final_predicted_frozen";
            frozen.pose.position.x = mem.final_predicted_x;
            frozen.pose.position.y = mem.final_predicted_y;
            frozen.color.r = 1.0f; frozen.color.g = 0.55f; frozen.color.b = 0.1f;
            arr.markers.push_back(frozen);
          }
          visualization_msgs::msg::Marker lost_surface = surface;
          lost_surface.ns = "lost_vehicle_predicted_surface";
          lost_surface.color.r = 0.8f; lost_surface.color.g = 0.25f;
          lost_surface.color.b = 1.0f; lost_surface.color.a = 0.5f;
          lost_surface.points.clear();
          for (const auto & local : mem.local_surface) {
            const auto world = cx128s2_bev_perception::vehicle_surface::toWorld(
                local, {outputX(tr), outputY(tr)}, mem.vehicle_obb.yaw);
            geometry_msgs::msg::Point point;
            point.x = world.x; point.y = world.y; point.z = mem.vehicle_z_center;
            lost_surface.points.push_back(point);
          }
          arr.markers.push_back(lost_surface);
        }
        visualization_msgs::msg::Marker owned = surface;
        owned.ns = "vehicle_owned_components";
        owned.color.r = 1.0f; owned.color.g = 0.7f; owned.color.b = 0.1f;
        owned.scale.x = owned.scale.y = 0.22;
        owned.points.clear();
        for (const auto & p : mem.owned_component_centers) {
          geometry_msgs::msg::Point q;
          q.x = p.x; q.y = p.y; q.z = tr.z;
          owned.points.push_back(q);
        }
        arr.markers.push_back(owned);
        visualization_msgs::msg::Marker support = surface;
        support.ns = "vehicle_actual_support";
        support.color.r = 0.2f; support.color.g = 1.0f; support.color.b = 0.2f;
        support.scale.x = support.scale.y = 0.10;
        support.points.clear();
        for (const auto & p : mem.actual_support_world) {
          geometry_msgs::msg::Point q;
          q.x = p.x; q.y = p.y; q.z = mem.vehicle_z_center;
          support.points.push_back(q);
        }
        arr.markers.push_back(support);
        visualization_msgs::msg::Marker marker_base = surface;
        marker_base.points.clear();
        if (mem.reacquire_candidate_valid &&
            (lostVehicle(tr) || mem.reacquire_result == "REACQUIRED")) {
          visualization_msgs::msg::Marker line = marker_base;
          line.ns = lostVehicle(tr) ? "lost_vehicle_reacquire_candidate" :
              "vehicle_reacquire_success";
          line.type = visualization_msgs::msg::Marker::LINE_LIST;
          line.scale.x = 0.08;
          line.color.r = lostVehicle(tr) ? 1.0f : 0.1f;
          line.color.g = lostVehicle(tr) ? 0.6f : 1.0f;
          line.color.b = 0.2f; line.color.a = 0.85f;
          geometry_msgs::msg::Point point;
          point.x = outputX(tr); point.y = outputY(tr);
          point.z = mem.vehicle_z_center + 0.5;
          line.points.push_back(point);
          point.x = mem.reacquire_candidate_center.x;
          point.y = mem.reacquire_candidate_center.y;
          line.points.push_back(point);
          arr.markers.push_back(line);
        }

        auto centerMarker = [&](const char * ns, int id, double x, double y,
                                float r, float g, float b) {
          visualization_msgs::msg::Marker marker = marker_base;
          marker.ns = ns;
          marker.id = id;
          marker.type = visualization_msgs::msg::Marker::SPHERE;
          marker.pose.position.x = x;
          marker.pose.position.y = y;
          marker.pose.position.z = mem.vehicle_z_center + mem.vehicle_height * 0.5 + 0.2;
          marker.scale.x = marker.scale.y = marker.scale.z = 0.24;
          marker.color.r = r; marker.color.g = g; marker.color.b = b;
          marker.color.a = 0.95f;
          marker.points.clear();
          arr.markers.push_back(marker);
        };
        centerMarker(lostVehicle(tr) ? "lost_vehicle_predicted_center" :
                     "vehicle_predicted_center", tr.id, mem.predicted_x,
                     mem.predicted_y, lostVehicle(tr) ? 0.85f : 0.1f,
                     lostVehicle(tr) ? 0.25f : 0.5f, 1.0f);
        if (mem.lateral_anchor.valid)
          centerMarker("vehicle_lateral_anchor", tr.id, mem.predicted_x,
                       mem.lateral_anchor.y, 0.9f, 0.2f, 0.9f);
        if (mem.measured_center_valid) {
          centerMarker("vehicle_raw_centroid", tr.id, mem.raw_centroid_x,
                       mem.raw_centroid_y, 1.0f, 0.45f, 0.1f);
          centerMarker("vehicle_corrected_center", tr.id, mem.measured_x,
                       mem.measured_y, 1.0f, 0.9f, 0.1f);
          visualization_msgs::msg::Marker drift = marker_base;
          drift.ns = "vehicle_lateral_innovation";
          drift.id = tr.id;
          drift.type = visualization_msgs::msg::Marker::LINE_LIST;
          drift.scale.x = 0.06;
          drift.color.r = 1.0f; drift.color.g = 0.3f; drift.color.b = 0.15f;
          drift.points.clear();
          geometry_msgs::msg::Point p;
          p.x = mem.predicted_x; p.y = mem.predicted_y;
          p.z = mem.vehicle_z_center + mem.vehicle_height * 0.5 + 0.2;
          drift.points.push_back(p);
          p.x = mem.measured_x; p.y = mem.measured_y;
          drift.points.push_back(p);
          arr.markers.push_back(drift);
        }
        auto yawMarker = [&](const char * ns, double x, double y,
                             double yaw, float r, float g, float b) {
          visualization_msgs::msg::Marker marker = marker_base;
          marker.ns = ns;
          marker.id = tr.id;
          marker.type = visualization_msgs::msg::Marker::LINE_LIST;
          marker.scale.x = 0.09;
          marker.color.r = r; marker.color.g = g; marker.color.b = b;
          marker.points.clear();
          geometry_msgs::msg::Point p;
          p.x = x; p.y = y;
          p.z = mem.vehicle_z_center + mem.vehicle_height * 0.5 + 0.4;
          marker.points.push_back(p);
          p.x += 1.5 * std::cos(yaw);
          p.y += 1.5 * std::sin(yaw);
          marker.points.push_back(p);
          arr.markers.push_back(marker);
        };
        yawMarker("vehicle_reliable_yaw", obj.position.x, obj.position.y,
                  mem.vehicle_obb.yaw, 0.1f, 1.0f, 0.45f);
        if (mem.measured_center_valid) {
          if (tr.feature.obb.valid &&
              mem.vehicle_quality != VehicleObservationQuality::SPARSE) {
            visualization_msgs::msg::Marker measured_box = box;
            measured_box.ns = "vehicle_measured_obb";
            measured_box.id = tr.id;
            measured_box.pose.position.x = tr.feature.obb.center.x;
            measured_box.pose.position.y = tr.feature.obb.center.y;
            measured_box.pose.orientation.z = std::sin(tr.feature.obb.yaw * 0.5);
            measured_box.pose.orientation.w = std::cos(tr.feature.obb.yaw * 0.5);
            measured_box.scale.x = tr.feature.obb.length;
            measured_box.scale.y = tr.feature.obb.width;
            measured_box.color.r = 1.0f; measured_box.color.g = 0.55f;
            measured_box.color.b = 0.1f; measured_box.color.a = 0.12f;
            arr.markers.push_back(measured_box);
          }
        }
      }
    }
    pub_markers_->publish(arr);
  }

