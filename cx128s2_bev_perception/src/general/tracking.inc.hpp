  int maxLostForTrack(const Track & tr) const {
    const double r = trackSensorRange(tr);
    int base;
    if (!enable_shape_memory_tracking_ || !shape_memory_keep_while_lost_ || !tr.shape_memory.valid) {
      base = maxLostForRange(r);
    } else if (r < near_range_max_) {
      base = std::max(max_lost_frames_near_, shape_memory_max_lost_frames_near_);
    } else if (r < mid_range_max_) {
      base = std::max(max_lost_frames_mid_, shape_memory_max_lost_frames_mid_);
    } else {
      base = std::max(max_lost_frames_long_, shape_memory_max_lost_frames_long_);
    }
    // Persistent vehicles use the same conservative visibility explanation
    // even when the optional legacy adaptive-loss switch is disabled.
    if ((!visibility_adaptive_loss_enabled_ && !persistentVehicle(tr)) ||
        !visibility_frame_valid_ || tr.lost_frames == 0)
      return base;
    const auto reason = tr.observation_memory.miss_reason;
    if (reason == VisibilityReason::VISIBLE_EMPTY) return std::max(1, base / 2);
    if (reason == VisibilityReason::NEAR_BLIND ||
        reason == VisibilityReason::OUT_OF_VERTICAL_FOV ||
        reason == VisibilityReason::OUT_OF_HORIZONTAL_FOV ||
        reason == VisibilityReason::OCCLUDED_BY_OTHER ||
        reason == VisibilityReason::FULLY_OCCLUDED_LIKELY) return 2 * base;
    const auto & mem = tr.observation_memory;
    if (mem.reliable && mem.reliable_expected_rays > 0 &&
        tr.visibility.adaptive_expected_rays > 0) {
      const double scarcity = std::sqrt(static_cast<double>(mem.reliable_expected_rays) /
                                        tr.visibility.adaptive_expected_rays);
      return static_cast<int>(std::lround(base * clamp(scarcity, 1.0, 2.0)));
    }
    return base;
  }

  double sensorRange(double x, double y) const {
    return std::hypot(x - sensor_origin_x_, y - sensor_origin_y_);
  }
  double trackSensorRange(const Track & tr) const { return sensorRange(outputX(tr), outputY(tr)); }
  // This range policy limits new-candidate similarity/bank matching, not
  // confirmed surface ownership or validated identity recovery. Distance is planar.
  bool memoryMatchingBlocked(double range) const {
    return perception_mode_ != "crane_beam_work" && range >= mid_range_max_;
  }
  bool memoryMatchingBlockedPair(const Track & tr, const ObjectFeature & f) const {
    return memoryMatchingBlocked(trackSensorRange(tr)) || memoryMatchingBlocked(f.range);
  }
  bool physicalOutputAllowed(const Track & tr) const {
    if (dormantVehicle(tr)) return false;
    // Long-range confirmed FIFO identity remains a detection output even
    // without new points. This does not promote prediction to measured evidence.
    if (persistentVehicle(tr) && tr.observation_memory.last_observed_valid &&
        inside_vehicle_queue_.contains(tr.id) && trackSensorRange(tr) >= mid_range_max_)
      return true;
    if (lostVehicle(tr)) return false;
    if (tr.lost_frames == 0) return true;
    return keep_box_while_lost_ && tr.lost_frames <= maxLostForTrack(tr) &&
        (!persistentVehicle(tr) || tr.observation_memory.missed_time_sec <= vehicle_lost_memory_sec_);
  }

  double confirmedVehicleTop(const Track & tr) const {
    const auto & m=tr.observation_memory;
    return m.reliable && m.last_reliable_box.max_z>m.last_reliable_box.min_z ?
        m.last_reliable_box.max_z : m.vehicle_z_center+m.vehicle_height*.5;
  }

  // Reuse the existing range support limits and angular-support validator.
  // A PARTIAL cluster needs spatial extent as well as count; count alone is not a vehicle.
  const char * vehicleObservationFailure(const Track & tr, const ObjectFeature & f,
                                         const std::vector<PointXYZIRT> & points,
                                         bool full_required = false) const {
    const int min_points = f.range < near_range_max_ ? min_cluster_points_near_ :
        (f.range < mid_range_max_ ? min_cluster_points_mid_ : min_cluster_points_long_);
    const int min_cells = f.range < near_range_max_ ? min_cluster_cells_near_ :
        (f.range < mid_range_max_ ? min_cluster_cells_mid_ : min_cluster_cells_long_);
    if (f.point_count < min_points || f.cell_count < min_cells) return "INSUFFICIENT_SUPPORT";
    const double length = f.obb.valid ? f.obb.length : f.length;
    const double width = f.obb.valid ? f.obb.width : f.width;
    const bool full = length >= 2.0 && length <= vehicle_max_length_ &&
        width >= vehicle_min_width_ && width <= vehicle_max_width_ &&
        f.height >= vehicle_min_height_ && f.height <= vehicle_max_height_;
    // Confirmed body height is fixed. A merged overhead structure must not
    // move XY through its centroid, even if it fits the global vehicle limit.
    // The existing RAW_SPARSE path can recover current points inside the body.
    if (persistentVehicle(tr) && !lostVehicle(tr) &&
        f.max_z > confirmedVehicleTop(tr) + raw_vehicle_box_margin_m_)
      return "ABOVE_CONFIRMED_VEHICLE_HEIGHT";
    if(persistentVehicle(tr) && !lostVehicle(tr) && tr.multi_component_body){
      const double tolerance=raw_vehicle_box_margin_m_+std::min(bev_x_res_,raw_vehicle_box_margin_m_);
      for(const auto&r:tr.component_references){
        if(r[2]>vehicle_max_width_ || length<.8*r[2] || length>1.25*r[2] ||
           width<.8*r[3] || width>1.25*r[3])continue;
        if(std::abs(length-r[2])>tolerance || std::abs(width-r[3])>tolerance){
          bool referenced=false;
          correctedVehicleCenter(tr,f.centroid_x,f.centroid_y,1,&f,&referenced,&points);
          if(!referenced)return "PART_GEOMETRY_UNCERTAIN";
        }
      }
      if(length>1.03*tr.observation_memory.vehicle_obb.length &&
         !measuredVehicleExtensionSupported(tr,f,points))return "UNSUPPORTED_BODY_EXTENT";
    }
    if (full_required && !full) return "REACQUIRE_REQUIRES_VEHICLE_SIZE";
    if (!std::isfinite(length) || !std::isfinite(width) || !std::isfinite(f.height) ||
        (!persistentVehicle(tr) && (length < 0.8 || width < 0.2)) ||
        f.height < (persistentVehicle(tr) ? std::max(0.3, 0.2 * tr.observation_memory.vehicle_height) :
                   0.5 * vehicle_min_height_) ||
        length > vehicle_max_length_ || width > vehicle_max_width_ ||
        f.height > vehicle_max_height_) return "INSUFFICIENT_SPATIAL_EXTENT";
    if (persistentVehicle(tr) && !full) {
      const auto & obb = tr.observation_memory.vehicle_obb;
      if (!visibility_frame_valid_ ||
          (width < std::max(0.4, 0.25 * obb.width) &&
           length < std::max(0.8, 0.2 * obb.length))) return "NO_REMEMBERED_FACE_EXTENT";
    }
    if (!full && visibility_frame_valid_ &&
        (persistentVehicle(tr) || tr.visibility.adaptive_expected_rays > 0)) {
      std::set<std::pair<int, int>> bins;
      // Match the sensor-frame azimuth used by Expected Ray and front support.
      const auto sensor_from_world = visibility_model_.pose().inverse();
      for (int idx : f.indices) {
        if (idx < 0 || static_cast<size_t>(idx) >= points.size()) continue;
        const auto & p = points[idx];
        const auto local = sensor_from_world * tf2::Vector3(p.x, p.y, p.z);
        double az = std::atan2(local.y(), local.x()) * 180.0 / M_PI;
        if (az < 0.0) az += 360.0;
        bins.emplace(static_cast<int>(std::floor(az / std::max(0.02, visibility_model_.azimuthBinDeg()))), p.ring);
      }
      const auto support = persistentVehicle(tr) ? visibility_model_.evaluateAdaptiveBox(
          {f.min_x, f.max_x, f.min_y, f.max_y, f.min_z, f.max_z}, f.id, false, 0.0) : tr.visibility;
      if (persistentVehicle(tr) && support.adaptive_expected_rays <= 0) return "NO_VISIBLE_FACE_SUPPORT";
      if (!cx128s2_bev_perception::vehicle_surface::enoughSparseSupport(
          static_cast<int>(bins.size()), 2, support.adaptive_expected_rays,
          support.occluded_fraction, support.has_occluder)) return "PARTIAL_SUPPORT_TOO_LOW";
    }
    return nullptr;
  }

  bool kalmanEnabled() const { return tracking_filter_mode_ == "kalman"; }
  double outputX(const Track & tr) const { return kalmanEnabled() && tr.kf.initialized() ? tr.kf.x() : tr.x; }
  double outputY(const Track & tr) const { return kalmanEnabled() && tr.kf.initialized() ? tr.kf.y() : tr.y; }
  double outputVx(const Track & tr) const { return kalmanEnabled() && tr.kf.initialized() ? tr.kf.vx() : tr.vx; }
  double outputVy(const Track & tr) const { return kalmanEnabled() && tr.kf.initialized() ? tr.kf.vy() : tr.vy; }

  double trackPositionSigma(const Track & tr) const {
    if (kalmanEnabled() && tr.kf.initialized()) {
      return std::sqrt(std::max(0.0, std::max(tr.kf.positionVarianceX(),
                                              tr.kf.positionVarianceY())));
    }
    // Legacy position state has no covariance. Use its configured measurement
    // uncertainty rather than a fixed angular margin.
    return kf_measurement_position_std_m_;
  }

  VisibilityBox predictedVisibilityBox(const Track & tr) const {
    const auto & mem = tr.observation_memory;
    if (persistentVehicle(tr)) {
      const auto & obb = mem.vehicle_obb;
      const double c = std::abs(std::cos(obb.yaw)), s = std::abs(std::sin(obb.yaw));
      const double hx = (c * obb.length + s * obb.width) * 0.5;
      const double hy = (s * obb.length + c * obb.width) * 0.5;
      const double x = outputX(tr), y = outputY(tr);
      const double hz = std::max(0.3, mem.vehicle_height * 0.5);
      return {x - hx, x + hx, y - hy, y + hy,
              mem.vehicle_z_center - hz, mem.vehicle_z_center + hz};
    }
    if (mem.reliable) {
      const double dx = outputX(tr) - mem.reliable_center_x;
      const double dy = outputY(tr) - mem.reliable_center_y;
      const double dz = tr.z - mem.reliable_center_z;
      return {mem.last_reliable_box.min_x + dx, mem.last_reliable_box.max_x + dx,
              mem.last_reliable_box.min_y + dy, mem.last_reliable_box.max_y + dy,
              mem.last_reliable_box.min_z + dz, mem.last_reliable_box.max_z + dz};
    }
    const auto & f = tr.feature;
    const double dx = outputX(tr) - f.centroid_x;
    const double dy = outputY(tr) - f.centroid_y;
    const double dz = tr.z - f.centroid_z;
    return {f.min_x + dx, f.max_x + dx, f.min_y + dy, f.max_y + dy,
            f.min_z + dz, f.max_z + dz};
  }

  bool hasReliableVisibilityBox(const Track & tr) const {
    if (persistentVehicle(tr)) return true;
    // This certifies only the last observed support box, not unseen vehicle
    // length. A confirmed class and established cluster support are required
    // before the stronger FULLY_OCCLUDED_LIKELY diagnostic is allowed.
    return tr.detection.is_valid && tr.detection.class_name == "VEHICLE" &&
           tr.hits >= tentative_confirm_hits_ &&
           tr.feature.point_count >= min_cluster_points_near_ &&
           tr.feature.cell_count >= min_cluster_cells_near_ &&
           tr.feature.width >= vehicle_min_width_ &&
           tr.feature.height >= vehicle_min_height_;
  }

  void updateTrackObservationMemory(Track & tr, const ObjectFeature & f, double stamp) {
    const auto t0 = std::chrono::steady_clock::now();
    auto & mem = tr.observation_memory;
    mem.last_visibility = tr.visibility;
    mem.miss_reason = VisibilityReason::UNKNOWN;
    mem.missed_time_sec = 0.0;
    mem.observation_age_sec = mem.reliable ?
        std::max(0.0, stamp - mem.last_reliable_stamp) : 0.0;
    const int expected = tr.visibility.adaptive_expected_rays;
    const double support_ratio = expected > 0 ?
        static_cast<double>(tr.visibility.adaptive_supported_rays) / expected : 0.0;
    const double unknown_ratio = tr.visibility.observable_samples > 0 ?
        static_cast<double>(tr.visibility.unknown_samples) /
            tr.visibility.observable_samples : 1.0;
    // Geometry evidence does not commit a FIFO identity. A repeatedly observed
    // truck waiting for ENTRY direction can still remember its measured box.
    const bool general_kalman = perception_mode_ != "crane_beam_work" && kalmanEnabled();
    const bool pending_vehicle = general_kalman &&
        tr.detection.decision_reason == "AWAITING_ENTRY_DIRECTION" &&
        recentHits(tr) >= tentative_confirm_hits_ && tr.hits >= tentative_confirm_hits_;
    const bool complete = (!general_kalman || persistentVehicle(tr) || f.length >= 2.0) &&
        visibility_frame_valid_ && expected > 0 &&
        support_ratio >= 0.08 && tr.visibility.occluded_fraction < 0.3 &&
        unknown_ratio < 0.7 && !tr.visibility.near_blind_zone &&
        tr.visibility.reason != VisibilityReason::OUT_OF_VERTICAL_FOV &&
        tr.visibility.reason != VisibilityReason::OUT_OF_HORIZONTAL_FOV &&
        (tr.detection.is_valid || pending_vehicle) && tr.detection.class_name == "VEHICLE" &&
        tr.hits >= tentative_confirm_hits_ && f.point_count >= min_cluster_points_near_;
    mem.stable_observation_frames = complete ?
        std::min(1000, mem.stable_observation_frames + 1) : 0;
    const bool sparse_shrink = mem.reliable && expected < mem.reliable_expected_rays &&
        (f.length < 0.9 * mem.reliable_length ||
         f.width < 0.9 * mem.reliable_width ||
         f.height < 0.9 * mem.reliable_height);
    const bool pending_geometry_compatible = !general_kalman || mem.vehicle_confirmed || !mem.reliable ||
        (f.length >= 0.9 * mem.reliable_length &&
         f.width >= 0.9 * mem.reliable_width &&
         f.height >= 0.9 * mem.reliable_height &&
         cx128s2_bev_perception::vehicle_surface::extensionAllowed(
             mem.reliable_length, mem.reliable_width, f.length, f.width,
             mem.stable_observation_frames, vehicle_max_length_, vehicle_max_width_));
    const bool measured = complete && mem.stable_observation_frames >= 3 &&
        !sparse_shrink && pending_geometry_compatible && mem.reacquire_geometry_hold_frames == 0;
    const bool partial = visibility_frame_valid_ && !measured && expected > 0 &&
        (tr.visibility.occluded_fraction > 0.0 || support_ratio < 0.08 ||
         tr.visibility.observable_fraction < 0.8 || sparse_shrink);
    mem.width_evidence = measured ? GeometryEvidence::MEASURED :
        partial ? GeometryEvidence::LOWER_BOUND : GeometryEvidence::UNKNOWN;
    mem.height_evidence = mem.width_evidence;
    mem.length_evidence = mem.width_evidence;
    if (measured) {
      VisibilityBox box{f.min_x, f.max_x, f.min_y, f.max_y, f.min_z, f.max_z};
      if (mem.vehicle_confirmed && mem.reliable) {
        // Keep the first trusted vertical extent; a tolerated roof return must
        // not ratchet the height gate upward on successive frames.
        box.min_z=mem.last_reliable_box.min_z;
        box.max_z=mem.last_reliable_box.max_z;
      }
      if (general_kalman && !mem.vehicle_confirmed && f.obb.valid) {
        if (!mem.vehicle_obb.valid) mem.vehicle_obb = f.obb;
        else {
          cx128s2_bev_perception::vehicle_surface::applyReliableGeometry(
              mem.vehicle_obb, f.obb, mem.stable_observation_frames,
              vehicle_max_length_, vehicle_max_width_);
          mem.vehicle_obb.yaw = vehicle_fixed_yaw_rad_;
        }
      }
      // A complete, stable observation may refresh geometry. Partial frames
      // cannot reduce a previously reliable box.
      mem.last_reliable_box = box;
      mem.reliable_center_x = mem.vehicle_confirmed ? outputX(tr) : f.centroid_x;
      mem.reliable_center_y = mem.vehicle_confirmed ? outputY(tr) : f.centroid_y;
      mem.reliable_center_z = f.centroid_z;
      mem.reliable_length = mem.vehicle_confirmed ? mem.vehicle_obb.length :
          (general_kalman ? std::max(mem.reliable_length, f.length) : f.length);
      mem.reliable_width = mem.vehicle_confirmed ? mem.vehicle_obb.width :
          (general_kalman ? std::max(mem.reliable_width, f.width) : f.width);
      mem.reliable_height = mem.vehicle_confirmed ? mem.vehicle_height :
          (general_kalman ? std::max(mem.reliable_height, f.height) : f.height);
      mem.geometry_confidence = clamp(0.5 + 0.5 * support_ratio, 0.0, 1.0);
      mem.last_reliable_stamp = stamp;
      mem.reliable_expected_rays = expected;
      mem.observation_age_sec = 0.0;
      mem.reliable = true;
    }
    track_memory_update_ms_ += std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();
  }

  bool persistentVehicle(const Track & tr) const {
    return perception_mode_ != "crane_beam_work" && tr.observation_memory.vehicle_confirmed;
  }

  bool lostVehicle(const Track & tr) const {
    return persistentVehicle(tr) && tr.observation_memory.vehicle_state !=
        cx128s2_bev_perception::vehicle_surface::VehicleLifecycle::ACTIVE;
  }

  bool dormantVehicle(const Track & tr) const {
    return persistentVehicle(tr) && tr.observation_memory.vehicle_state ==
        cx128s2_bev_perception::vehicle_surface::VehicleLifecycle::DORMANT;
  }

  bool predictiveLostVehicle(const Track & tr) const {
    return persistentVehicle(tr) && tr.observation_memory.vehicle_state ==
        cx128s2_bev_perception::vehicle_surface::VehicleLifecycle::LOST;
  }

  std::string fifoQueueString() const {
    std::ostringstream out;
    out << "[";
    bool first = true;
    for (int id : inside_vehicle_queue_.ids()) {
      if (!first) out << ",";
      out << id;
      first = false;
    }
    out << "]";
    return out.str();
  }

  void registerVehicleEntry(Track & tr, double stamp) {
    using namespace cx128s2_bev_perception::vehicle_surface;
    auto & mem = tr.observation_memory;
    if (!persistentVehicle(tr) || tr.lost_frames > 0 || !mem.last_observed_valid ||
        mem.fifo_phase != FifoPhase::NOT_REGISTERED ||
        (vehicle_require_entry_direction_ &&
         mem.corridor_direction.stable != CorridorDirection::ENTRY)) return;
    if (!inside_vehicle_queue_.append(tr.id)) {
      ++vehicle_fifo_anomaly_count_;
      return;
    }
    mem.fifo_phase = FifoPhase::INSIDE;
    mem.fifo_exit_observed = false;
    mem.entry_sequence = ++next_vehicle_entry_sequence_;
    mem.entry_timestamp = stamp;
    ++vehicle_fifo_entry_count_;
    RCLCPP_INFO(get_logger(), "[VEH_FIFO_ENTRY] id=%d sequence=%llu direction=%s queue=%s",
                tr.id, static_cast<unsigned long long>(mem.entry_sequence),
                directionName(mem.corridor_direction.stable),
                fifoQueueString().c_str());
  }

  void updateVehicleSurface(Track & tr, const ObjectFeature & f,
                            const std::vector<PointXYZIRT> & points) {
    if (perception_mode_ == "crane_beam_work" || !f.obb.valid) return;
    auto & mem = tr.observation_memory;
    if (!mem.vehicle_confirmed) {
      if (!mem.reliable || !tr.detection.is_valid ||
          tr.detection.class_name != "VEHICLE" ||
          (kalmanEnabled() && mem.length_evidence != GeometryEvidence::MEASURED) ||
          f.point_count < min_cluster_points_near_ ||
          f.obb.width < vehicle_min_width_ || f.obb.width > vehicle_max_width_ ||
          f.obb.length > vehicle_max_length_ || f.obb.length < 2.0) return;
      mem.vehicle_confirmed = true;
      mem.last_seen_stamp = tr.last_time;
      mem.last_observed_x = f.obb.center.x;
      mem.last_observed_y = f.obb.center.y;
      mem.last_observed_valid = true;
      mem.last_known_distance = f.range;
      mem.last_observed_quality = VehicleObservationQuality::FULL;
      mem.vehicle_obb = f.obb;
      mem.vehicle_obb.yaw = vehicle_fixed_yaw_rad_;
      mem.vehicle_z_center = f.centroid_z;
      mem.vehicle_height = f.height;
      mem.lateral_anchor.y = f.obb.center.y;
      mem.lateral_anchor.valid = true;
      mem.reliable_center_x = f.obb.center.x;
      mem.reliable_center_y = f.obb.center.y;
      mem.reliable_length = f.obb.length;
      mem.reliable_width = f.obb.width;
      mem.local_surface = cx128s2_bev_perception::vehicle_surface::faceSamples(
          f.obb.length, f.obb.width);
      // At most 32 observed samples complement the 32 fixed face samples.
      for (size_t k = 0; k < f.indices.size() && mem.local_surface.size() < 64; k +=
           std::max<size_t>(1, f.indices.size() / 32)) {
        const auto & p = points[f.indices[k]];
        mem.local_surface.push_back(cx128s2_bev_perception::vehicle_surface::toLocal(
            {p.x, p.y}, f.obb.center, f.obb.yaw));
      }
      mem.geometry_update = "INITIAL";
      mem.predicted_x = mem.measured_x = f.obb.center.x;
      mem.predicted_y = mem.measured_y = f.obb.center.y;
      mem.predicted_yaw = f.obb.yaw;
      mem.raw_centroid_x = f.centroid_x;
      mem.raw_centroid_y = f.centroid_y;
      mem.measured_center_valid = true;
      mem.vehicle_quality = VehicleObservationQuality::FULL;
      mem.measured_yaw = f.obb.yaw;
      mem.yaw_update_accepted = true;
      mem.yaw_update_reason = "INITIAL";
      // OBB center is canonical; a partial cluster centroid is not.
      tr.x = f.obb.center.x;
      tr.y = f.obb.center.y;
      if (kalmanEnabled()) {
        if (tr.kf.initialized()) tr.kf.reanchorPosition(tr.x, tr.y);
        else tr.kf.initialize(tr.x, tr.y, kf_initial_position_std_m_,
                              kf_initial_velocity_std_mps_);
        tr.vx = tr.kf.vx();
        tr.vy = tr.kf.vy();
      }
      return;
    }
    if (mem.reacquire_geometry_hold_frames > 0) {
      mem.extension_frames = 0;
      mem.yaw_update_reason = "REACQUIRE_HOLD";
      mem.geometry_update = "NONE";
      return;
    }
    if (mem.vehicle_quality != VehicleObservationQuality::FULL) {
      mem.extension_frames = 0;
      mem.yaw_update_reason = "NOT_FULL";
      return;
    }
    const auto old = mem.vehicle_obb;
    const bool larger = f.obb.length > old.length * 1.03;
    if(tr.multi_component_body && larger && !measuredVehicleExtensionSupported(tr,f,points)){
      mem.extension_frames=0;mem.geometry_update="NONE";return;
    }
    const bool aligned_extent = mem.association_source == "EXTENT_ALIGNED";
    const bool close = std::hypot((aligned_extent ? mem.measured_x : f.obb.center.x) - outputX(tr),
                                  (aligned_extent ? mem.measured_y : f.obb.center.y) - outputY(tr)) <
        std::min(1.5, gateForRange(f.range));
    const int expected = tr.visibility.adaptive_expected_rays;
    const double support_ratio = expected > 0 ?
        static_cast<double>(tr.visibility.adaptive_supported_rays) / expected : 0.0;
    if (larger && close && f.point_count >= min_cluster_points_near_ &&
        tr.visibility.occluded_fraction < 0.3) ++mem.extension_frames;
    else mem.extension_frames = 0;
    bool expanded = false;
    if (larger && close && cx128s2_bev_perception::vehicle_surface::applyReliableGeometry(
        mem.vehicle_obb, f.obb, mem.extension_frames,
        vehicle_max_length_, vehicle_max_width_, aligned_extent)) {
      mem.reliable_length = mem.vehicle_obb.length;
      if (aligned_extent) {
        const auto sensor = cx128s2_bev_perception::vehicle_surface::toLocal(
            {sensor_origin_x_, sensor_origin_y_}, {outputX(tr), outputY(tr)}, old.yaw);
        const auto center = cx128s2_bev_perception::vehicle_surface::toWorld(
            {-std::copysign((mem.vehicle_obb.length - old.length) * 0.5, sensor.x), 0.0},
            {outputX(tr), outputY(tr)}, old.yaw);
        const double dx = center.x - outputX(tr), dy = center.y - outputY(tr);
        tr.x = center.x; tr.y = center.y;
        if (kalmanEnabled()) tr.kf.reanchorPosition(tr.x, tr.y);
        if(mem.body_pose_anchor_valid){mem.body_pose_anchor_local.x-=dx;mem.body_pose_anchor_local.y-=dy;}
        for(auto &r:tr.component_references){r[0]-=dx;r[1]-=dy;}
        mem.last_observed_x += dx; mem.last_observed_y += dy;
        mem.measured_x += dx; mem.measured_y += dy;
      }
      mem.local_surface = cx128s2_bev_perception::vehicle_surface::faceSamples(
          mem.vehicle_obb.length, mem.vehicle_obb.width);
      mem.geometry_update = "EXPANDED";
      mem.extension_frames = 0;
      expanded = true;
    }
    // Keep the orientation established at confirmation; KF has XY state only.
    mem.measured_yaw = std::numeric_limits<double>::quiet_NaN();
    mem.yaw_innovation = 0.0;
    mem.yaw_update_accepted = false;
    mem.yaw_update_reason = "LOCKED_INITIAL";
    // Only repeated, well-supported FULL observations can move the lane
    // anchor. Partial/sparse observations never change it.
    if (close && (!larger || expanded)) {
      mem.anchor_updated = cx128s2_bev_perception::vehicle_surface::advanceLateralAnchor(
          mem.lateral_anchor, f.obb.center.y, mem.predicted_y, f.range,
          near_range_max_, mid_range_max_, support_ratio);
    }
    mem.vehicle_obb.center = {outputX(tr), outputY(tr)};
  }

  static const char * vehicleQualityName(VehicleObservationQuality q) {
    switch (q) {
      case VehicleObservationQuality::FULL: return "FULL";
      case VehicleObservationQuality::PARTIAL: return "PARTIAL";
      case VehicleObservationQuality::SPARSE: return "SPARSE";
      default: return "NONE";
    }
  }

  void predictVehicleSurface(Track & tr) {
    auto & mem = tr.observation_memory;
    mem.expected_visible_samples = mem.visible_supported_samples = 0;
    mem.predicted_visible_world.clear();
    if (!persistentVehicle(tr) || !visibility_frame_valid_) return;
    const double x = outputX(tr), y = outputY(tr);
    for (const auto & local : mem.local_surface) {
      const auto transform_start = std::chrono::steady_clock::now();
      const auto p = cx128s2_bev_perception::vehicle_surface::toWorld(
          local, {x, y}, mem.vehicle_obb.yaw);
      vehicle_surface_predict_ms_ += std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - transform_start).count();
      const auto query_start = std::chrono::steady_clock::now();
      const auto v = visibility_model_.evaluatePoint(
          p.x, p.y, mem.vehicle_z_center, vehicleDepthOwner(tr.id));
      vehicle_visibility_query_ms_ += std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - query_start).count();
      if (v.reason == VisibilityReason::OUT_OF_HORIZONTAL_FOV ||
          v.reason == VisibilityReason::OUT_OF_VERTICAL_FOV ||
          v.reason == VisibilityReason::NEAR_BLIND ||
          v.reason == VisibilityReason::TOO_FAR ||
          v.reason == VisibilityReason::TOO_CLOSE ||
          v.reason == VisibilityReason::OCCLUDED_BY_OTHER ||
          v.reason == VisibilityReason::NEARER_RETURN_UNKNOWN ||
          v.reason == VisibilityReason::SELF_OCCLUSION_POSSIBLE) continue;
      if (v.in_horizontal_fov && v.in_vertical_fov && v.in_valid_range) {
        mem.predicted_visible_world.push_back(p);
        ++mem.expected_visible_samples;
        if (v.reason == VisibilityReason::OBSERVABLE) ++mem.visible_supported_samples;
      }
    }

  }

  void predictKalmanForTrack(Track & tr, double raw_dt) const {
    if (!kalmanEnabled() || !tr.kf.initialized() || raw_dt <= 0.0) return;
    // Clamp only the KF integration interval. General-mode Kalman association
    // uses this prediction; legacy mode retains its centroid/velocity prediction.
    const double dt = clamp(raw_dt, kf_min_dt_sec_, kf_max_dt_sec_);
    tr.kf.predict(dt, kf_process_accel_std_mps2_);
    if (persistentVehicle(tr)) tr.kf.dampLateralVelocity(std::exp(-dt));
  }

  static constexpr int vehicleDepthOwner(int id) { return 1000000 + id; }

  bool measuredVehicleEnd(const ObjectFeature & feature,
                          const std::vector<PointXYZIRT> & points,
                          bool left, double & end) const {
    const bool right=!left;
    const double raw_end=left?feature.min_x:feature.max_x;
    // A hook outside the body can move a raw minimum. Locate the first
    // measured X strip with broad lateral support instead of using that
    // extreme as the vehicle end. Current points only; no box freezing.
    std::map<int,std::vector<int>> strips;
    const double resolution=std::max(.05,std::min(bev_x_res_,raw_vehicle_box_margin_m_));
    for(int i:feature.indices){if(i<0 || size_t(i)>=points.size())continue;
      const auto&q=points[i];
      if(std::abs(q.x-raw_end)>2*cluster_z_layer_resolution_ ||
         q.z<feature.min_z+.5*vehicle_min_height_)continue;
      strips[int(std::floor(q.x/resolution))].push_back(i);}
    std::vector<int>order;for(const auto&kv:strips)order.push_back(kv.first);
    if(right)std::reverse(order.begin(),order.end());

    for(int x:order){const auto&indices=strips.at(x);if(indices.size()<size_t(min_cluster_points_near_))continue;
      std::set<int>ys;double y0=std::numeric_limits<double>::max(),y1=-y0,sum_x=0;
      for(int i:indices){const auto&q=points[i];sum_x+=q.x;
        y0=std::min(y0,double(q.y));y1=std::max(y1,double(q.y));
        ys.insert(int(std::floor(q.y/std::max(.05,std::min(bev_y_res_,raw_vehicle_box_margin_m_)))));}
      if(y1-y0<vehicle_min_width_ || ys.size()<size_t(std::ceil(vehicle_min_width_/std::max(.05,std::min(bev_y_res_,raw_vehicle_box_margin_m_)))))continue;
      end=sum_x/indices.size();return true;
    }
    return false;
  }

  bool measuredVehicleExtensionSupported(const Track & tr, const ObjectFeature & f,
                                         const std::vector<PointXYZIRT> & points) const {
    const auto & old=tr.observation_memory.vehicle_obb;
    const double uncertainty=raw_vehicle_box_margin_m_+std::min(bev_x_res_,raw_vehicle_box_margin_m_);
    const auto supported=[&](bool left){double end=0;
      return measuredVehicleEnd(f,points,left,end) &&
          std::abs(end-(left?f.min_x:f.max_x))<=uncertainty;};
    return !(f.min_x<outputX(tr)-.5*old.length-raw_vehicle_box_margin_m_ && !supported(true)) &&
           !(f.max_x>outputX(tr)+.5*old.length+raw_vehicle_box_margin_m_ && !supported(false));
  }

  cx128s2_bev_perception::vehicle_surface::Point2 correctedVehicleCenter(
      const Track & tr, double observed_x, double observed_y,
      double lateral_gain, const ObjectFeature * observation = nullptr,
      bool * extent_aligned = nullptr,
      const std::vector<PointXYZIRT> * measured_points = nullptr) const {
    using namespace cx128s2_bev_perception::vehicle_surface;
    const auto & obb = tr.observation_memory.vehicle_obb;
    const Point2 predicted{outputX(tr), outputY(tr)};
    if (extent_aligned) *extent_aligned = false;
    if(observation && observation->obb.valid && tr.multi_component_body && !lostVehicle(tr)) {
      Point2 measured{};int matches=0;
      for(const auto&r:tr.component_references){
        if(r[2]>vehicle_max_width_)continue;
        const double tolerance=raw_vehicle_box_margin_m_+std::min(bev_x_res_,raw_vehicle_box_margin_m_);
        if(std::abs(observation->obb.length-r[2])>tolerance || std::abs(observation->obb.width-r[3])>tolerance)continue;
        if(observation->obb.length<.8*r[2] || observation->obb.length>1.25*r[2] ||
           observation->obb.width<.8*r[3] || observation->obb.width>1.25*r[3])continue;
        const Point2 p{observation->obb.center.x-r[0],observation->obb.center.y-r[1]};
        if(std::hypot(p.x-predicted.x,p.y-predicted.y)>std::min(1.5,gateForRange(observation->range)))continue;
        measured=p;++matches;
      }
      // A unique real part observation supplies translation of the same
      // vehicle reference. Prediction only validates correspondence, never
      // supplies the longitudinal measurement or a second owned component.
      if(matches==1){if(extent_aligned)*extent_aligned=true;return measured;}
      // A joined, FOV-clipped observation can retain the already measured
      // vehicle end. Require current broad deck support at that end, and
      // temporal reachability. Neither prediction nor a remembered point is
      // inserted as a current measurement.
      if(matches==0 && measured_points && observation->obb.length<.9*obb.length)for(const auto&r:tr.component_references){
        if(r[2]>vehicle_max_width_)continue;
        const bool left=std::abs(r[0]-.5*r[2]+.5*obb.length)<=raw_vehicle_box_margin_m_;
        const bool right=std::abs(r[0]+.5*r[2]-.5*obb.length)<=raw_vehicle_box_margin_m_;
        if(left==right)continue;
        double end=0;
        if(!measuredVehicleEnd(*observation,*measured_points,left,end))continue;
        const double offset=r[0]+(left?-.5:.5)*r[2];
        const Point2 p{end-offset,observation->obb.center.y};
        if(std::hypot(p.x-predicted.x,p.y-predicted.y)>std::min(1.5,gateForRange(observation->range)))continue;
        if(extent_aligned)*extent_aligned=true;
        return p;
      }
    }
    // A growing observed box can expose more of the same vehicle behind its
    // visible end face. Its center shift is not a velocity measurement.
    // Reuse the remembered face correspondence, with the existing geometry
    // and innovation bounds, while keeping the stored dimensions unchanged.
    if (observation && observation->obb.valid && !lostVehicle(tr) &&
        observation->obb.length > obb.length &&
        observation->obb.width >= 0.8 * obb.width &&
        observation->obb.width <= 1.25 * obb.width) {
      const auto sensor = toLocal({sensor_origin_x_, sensor_origin_y_}, predicted, obb.yaw);
      bool visible_end_face = !visibility_frame_valid_;
      if (visibility_frame_valid_) {
        const auto end = toWorld({std::copysign(obb.length * 0.5, sensor.x), 0.0},
                                 predicted, obb.yaw);
        for (double z : {tr.observation_memory.vehicle_z_center,
                         tr.observation_memory.vehicle_z_center - tr.observation_memory.vehicle_height * 0.5,
                         tr.observation_memory.vehicle_z_center + tr.observation_memory.vehicle_height * 0.5}) {
          if (visibility_model_.evaluateStaticPoint(end.x, end.y, z).reason ==
              VisibilityReason::OBSERVABLE) { visible_end_face = true; break; }
        }
      }
      if (visible_end_face &&
          std::abs(sensor.x) / obb.length >= std::abs(sensor.y) / obb.width) {
        const auto center = toWorld(
            {std::copysign((observation->obb.length - obb.length) * 0.5, sensor.x), 0.0},
            observation->obb.center, obb.yaw);
        if (std::hypot(center.x - predicted.x, center.y - predicted.y) <=
            std::min(1.5, gateForRange(observation->range))) {
          if (extent_aligned) *extent_aligned = true;
          return center;
        }
      }
    }
    if (dormantVehicle(tr)) {
      const auto sensor = toLocal({sensor_origin_x_, sensor_origin_y_},
                                  {observed_x, observed_y}, obb.yaw);
      const double hx = obb.length * 0.5, hy = obb.width * 0.5;
      const Point2 face = std::abs(sensor.x) / hx >= std::abs(sensor.y) / hy ?
          Point2{std::copysign(hx, sensor.x), 0.0} :
          Point2{0.0, std::copysign(hy, sensor.y)};
      auto center = toWorld({-face.x, -face.y}, {observed_x, observed_y}, obb.yaw);
      if (face.x != 0.0 && tr.observation_memory.lateral_anchor.valid)
        center.y = tr.observation_memory.lateral_anchor.y + lateral_gain *
            (center.y - tr.observation_memory.lateral_anchor.y);
      return center;
    }
    const auto local = toLocal({observed_x, observed_y}, predicted, obb.yaw);
    // A partial centroid is matched to the closest remembered rectangle face.
    const double hx = obb.length * 0.5, hy = obb.width * 0.5;
    Point2 nearest{std::clamp(local.x, -hx, hx), std::clamp(local.y, -hy, hy)};
    if (std::abs(local.x) <= hx && std::abs(local.y) <= hy) {
      const double dx = hx - std::abs(local.x), dy = hy - std::abs(local.y);
      if (dx < dy) nearest.x = std::copysign(hx, local.x == 0.0 ? 1.0 : local.x);
      else nearest.y = std::copysign(hy, local.y == 0.0 ? 1.0 : local.y);
    }
    // Keep longitudinal correspondence, but reduce uncertain lateral
    // residual from a distant single-face match.
    const auto residual = toWorld(
        {local.x - nearest.x, lateral_gain * (local.y - nearest.y)},
        {0, 0}, obb.yaw);
    return {predicted.x + residual.x, predicted.y + residual.y};
  }

  bool measuredVehicleBodyAnchor(Track & tr, const ObjectFeature & observation,
                                 const std::vector<PointXYZIRT> & points,
                                 double & measured_x) {
    auto & mem=tr.observation_memory;
    if(!kalmanEnabled() || !tr.position_from_body_box || !persistentVehicle(tr) || lostVehicle(tr) || observation.range>=near_range_max_ ||
       mem.reacquire_geometry_hold_frames>0 || !(mem.vehicle_height>0.0))return false;
    // Establish this extra surface reference only after observed upper-attachment
    // separation or the existing confirmed-height recovery, never for a clean
    // partial body merely entering the near-range band.
    if(!mem.body_pose_anchor_valid &&
       observation.decision_reason.find("CLUSTER:vehicle_body_separated_upper_attachment")==std::string::npos &&
       observation.decision_reason.find("CLUSTER:confirmed_height_support")==std::string::npos)return false;
    auto * bev=dynamic_cast<BevConnectedComponentClusterer *>(clusterer_.get());
    if(!bev)return false;
    // Measurement sampling only: the full body and attachments remain available
    // to detection, ownership, geometry and debug output. Use a body-relative
    // upper surface, not a global world-height exclusion.
    const double bottom=mem.vehicle_z_center-mem.vehicle_height*.5;
    const double low=bottom+mem.vehicle_height*.75;
    const double high=confirmedVehicleTop(tr)+raw_vehicle_box_margin_m_;
    std::vector<int> upper;
    for(int i:observation.indices)if(i>=0 && size_t(i)<points.size() &&
        points[i].z>=low && points[i].z<=high)upper.push_back(i);
    auto parts=bev->observationLayerComponents(points,upper);
    const ClusterResult * best=nullptr;double best_distance=std::numeric_limits<double>::infinity();
    for(const auto&part:parts){
      if(part.point_count<min_cluster_points_near_)continue;
      const double lx=part.max_x-part.min_x,ly=part.max_y-part.min_y;
      double zmin=std::numeric_limits<double>::max(),zmax=std::numeric_limits<double>::lowest();
      for(int i:part.indices){zmin=std::min(zmin,double(points[i].z));zmax=std::max(zmax,double(points[i].z));}
      if(std::max(lx,ly)<vehicle_min_width_ || zmax-zmin>std::max(lx,ly))continue;
      if(mem.body_pose_anchor_valid){
        // Same extent coherence already used for observed geometry alignment.
        if(lx<.8*mem.body_pose_anchor_length || lx>1.25*mem.body_pose_anchor_length ||
           ly<.8*mem.body_pose_anchor_width || ly>1.25*mem.body_pose_anchor_width)continue;
        const double dx=(part.min_x+part.max_x)*.5-outputX(tr)-mem.body_pose_anchor_local.x;
        const double dy=(part.min_y+part.max_y)*.5-outputY(tr)-mem.body_pose_anchor_local.y;
        const double distance=std::hypot(dx,dy);
        if(distance>std::min(1.5,gateForRange(observation.range)) || distance>=best_distance)continue;
        best_distance=distance;best=&part;
      }else if(!best || part.point_count>best->point_count)best=&part;
    }
    if(!best)return false;
    const double x=(best->min_x+best->max_x)*.5,y=(best->min_y+best->max_y)*.5;
    if(!mem.body_pose_anchor_valid){
      mem.body_pose_anchor_valid=true;
      // Calibrate against this frame's genuine canonical measurement, not
      // the KF prediction. Initialization does not invent a second update.
      mem.body_pose_anchor_local={x-measured_x,y-outputY(tr)};
      mem.body_pose_anchor_length=best->max_x-best->min_x;
      mem.body_pose_anchor_width=best->max_y-best->min_y;
      return false;
    }
    measured_x=x-mem.body_pose_anchor_local.x;
    return true;
  }

  void updateConfirmedVehicleKalman(Track & tr, double measured_x,
                                    double measured_y, double range,
                                    VehicleObservationQuality quality) {
    auto & mem = tr.observation_memory;
    mem.measured_x = measured_x;
    mem.measured_y = measured_y;
    mem.measured_center_valid = true;
    mem.lateral_innovation = measured_y - outputY(tr);
    if (!kalmanEnabled()) return;
    const int quality_rank = quality == VehicleObservationQuality::FULL ? 0 :
        (quality == VehicleObservationQuality::PARTIAL ? 1 : 2);
    const int rank = mem.reacquire_geometry_hold_frames > 0 ?
        std::max(1, quality_rank) : quality_rank;
    // Visibility modulates observation uncertainty, never its measured position.
    // The existing parameter remains compatible, with a bounded finite influence.
    const double support_ratio = tr.visibility.adaptive_expected_rays > 0 ? clamp(
        static_cast<double>(tr.visibility.adaptive_supported_rays) /
        tr.visibility.adaptive_expected_rays, 0.0, 1.0) : 0.0;
    const double visibility_noise = rank > 0 && visibility_frame_valid_ ?
        1.0 + 0.5 * visibility_box_prior_weight_ * (1.0 - support_ratio) : 1.0;
    const double std_x = (rank == 0 ? 1.0 : (rank == 1 ? 4.0 : 8.0)) *
        kf_measurement_position_std_m_ * visibility_noise;
    double std_y = cx128s2_bev_perception::vehicle_surface::lateralMeasurementStd(
        std_x, range, near_range_max_, mid_range_max_, rank);
    const bool accepted = cx128s2_bev_perception::vehicle_surface::lateralMeasurementAccepted(
        measured_y, outputY(tr), mem.lateral_anchor.y,
        mem.lateral_anchor.valid, mem.vehicle_obb.width);
    // Rejected Y remains prediction dominated; X still uses its measurement.
    const double used_y = accepted ? measured_y : outputY(tr);
    if (!accepted) std_y = std::max(std_y, 2.0);
    tr.kf.update(measured_x, used_y, std_x, std_y);
    if (mem.lateral_anchor.valid) {
      const double width = mem.vehicle_obb.width;
      const double anchor_fraction = range < near_range_max_ ? 0.4 :
          (range < mid_range_max_ ? 0.2 : 0.2);
      const double anchor_std = std::clamp(width * anchor_fraction, 0.25, 0.8);
      tr.kf.updateLateralAnchor(mem.lateral_anchor.y, anchor_std);
    }
    mem.lateral_measurement_accepted = accepted;
    mem.lateral_update_reason = accepted ? "ACCEPTED" : "INNOVATION_GATE";
    mem.measurement_std_x = std_x;
    mem.measurement_std_y = std_y;
    mem.measurement_std_used = std_x;
  }

  // Current measured deck continuity, not proximity alone. Clusters remain
  // separate in the scene; only the tracking observation can own both pieces.
  bool measuredDeckConnection(const ObjectFeature & body, const ObjectFeature & part,
                              const std::vector<PointXYZIRT> & points) const {
    const double gap=cluster_z_layer_resolution_+raw_vehicle_box_margin_m_;
    if(!body.obb.valid || !part.obb.valid || body.length<=vehicle_max_width_ ||
       part.length>vehicle_max_width_ || part.length<2.0 ||
       body.width<vehicle_min_width_ || part.width<.8*body.width ||
       part.width>1.25*body.width || part.height>body.height ||
       std::max(body.max_x,part.max_x)-std::min(body.min_x,part.min_x)>vehicle_max_length_ ||
       std::max(body.max_y,part.max_y)-std::min(body.min_y,part.min_y)>vehicle_max_width_ ||
       std::abs((body.min_y+body.max_y-part.min_y-part.max_y)*.5)>gap ||
       std::min(std::abs(body.min_x-part.max_x),std::abs(part.min_x-body.max_x))>gap)
      return false;
    const double bottom=std::max(body.min_z,part.min_z);
    const bool before=part.max_x<body.max_x;
    const double seam=before?body.min_x:body.max_x;
    const double grid=std::max(.05,cluster_z_layer_resolution_);
    std::unordered_map<int64_t,std::vector<int>> cells;
    for(int i:body.indices){const auto & q=points[i];
      if(std::abs(q.x-seam)>2*gap || q.z<bottom+.5*vehicle_min_height_)continue;
      cells[hash3d(int(std::floor(q.x/grid)),int(std::floor(q.y/grid)),int(std::floor(q.z/grid)))].push_back(i);
    }
    double y0=std::numeric_limits<double>::max(),y1=-y0,z0=y0,z1=-y0;
    int support=0;const int radius=int(std::ceil(gap/grid));
    for(int i:part.indices){const auto&q=points[i];
      if(std::abs(q.x-seam)>2*gap || q.z<bottom+.5*vehicle_min_height_)continue;
      const int x=int(std::floor(q.x/grid)),y=int(std::floor(q.y/grid)),z=int(std::floor(q.z/grid));bool connected=false;
      for(int dx=-radius;dx<=radius && !connected;++dx)
       for(int dy=-radius;dy<=radius && !connected;++dy)
        for(int dz=-radius;dz<=radius && !connected;++dz){
         const auto at=cells.find(hash3d(x+dx,y+dy,z+dz));if(at==cells.end())continue;
         for(int j:at->second){const auto&r=points[j];
          if(std::hypot(std::hypot(q.x-r.x,q.y-r.y),q.z-r.z)<=gap){connected=true;break;}}
        }
      if(connected){++support;y0=std::min(y0,double(q.y));y1=std::max(y1,double(q.y));
        z0=std::min(z0,double(q.z));z1=std::max(z1,double(q.z));}
    }
    return support>=min_cluster_points_near_ && y1-y0>=vehicle_min_width_ &&
        z1-z0<=cluster_z_layer_resolution_;
  }

  void updateTracks(const std::vector<ObjectFeature> & features, double stamp, const std::vector<PointXYZIRT> & points, const std::vector<int> & foreground_indices) {
    recording_track_features_.clear();
    recording_sparse_features_.clear();
    if (!enable_tracking_) return;
    if (last_tracking_stamp_ > 0.0 && stamp + 0.5 < last_tracking_stamp_) {
      // A replay/session time rewind makes stored world identity unsafe.
      resetTrackingState();
    }
    last_tracking_stamp_ = stamp;
    last_tracking_feature_input_count_ = static_cast<int>(features.size());

    std::vector<const ObjectFeature*> candidates;
    candidates.reserve(features.size());
    for (const auto & f : features) candidates.push_back(&f);

    if (static_cast<int>(candidates.size()) > max_tracking_candidates_) {
      std::partial_sort(candidates.begin(), candidates.begin() + max_tracking_candidates_, candidates.end(),
        [](const ObjectFeature* a, const ObjectFeature* b) {
          if (a->point_count != b->point_count) return a->point_count > b->point_count;
          if (a->cell_count != b->cell_count) return a->cell_count > b->cell_count;
          return a->range < b->range;
        });
      candidates.resize(max_tracking_candidates_);
    }
    std::vector<ObjectFeature> body_observations;
    body_observations.reserve(candidates.size());
    if(perception_mode_!="crane_beam_work" && kalmanEnabled()) {
      // Reuse observed generic Track history before either piece is public.
      // An already independent public identity is never silently coalesced.
      std::vector<int> aliases;
      std::unordered_map<int,std::vector<std::array<double,4>>> new_references;
      auto prior=[&](const ObjectFeature&f)->const Track*{
        const Track*best=nullptr;double distance=gateForRange(f.range);
        for(const auto&t:tracks_){if(t.lost_frames || t.hits<3)continue;
          const double d=std::hypot(t.feature.centroid_x-f.centroid_x,t.feature.centroid_y-f.centroid_y);
          if(d<distance){best=&t;distance=d;}}
        return best;
      };
      std::vector<bool> claimed(candidates.size(),false);
      for(size_t bi=0;bi<candidates.size();++bi){
        if(claimed[bi])continue;
        const auto&body=*candidates[bi];
        if(body.component_type!="vehicle_like" || body.length<=vehicle_max_width_)continue;
        const auto*bt=prior(body);if(!bt || (!bt->multi_component_body && (bt->vehicle_status_initialized || persistentVehicle(*bt))))continue;
        int match=-1;const Track*pt=nullptr;
        for(size_t pi=0;pi<candidates.size();++pi){
          if(pi==bi || claimed[pi])continue;
          const auto&part=*candidates[pi];
          if(part.component_type!="vehicle_like" || part.length>vehicle_max_width_)continue;
          const auto*t=prior(part);if((!t || t==bt) && !bt->multi_component_body)continue;
          if(t && t!=bt && (t->vehicle_status_initialized || persistentVehicle(*t)))continue;
          // A second tall, broad body is a separate vehicle, not a deck fragment.
          if(measuredBodyReferenceSupported(part,points) || !measuredDeckConnection(body,part,points))continue;
          const double dt=stamp-bt->last_time,ptdt=t && t!=bt?stamp-t->last_time:dt;
          if(dt<=0 || ptdt<=0 || dt>.6 || ptdt>.6)continue;
          const double bd=(body.min_x+body.max_x-bt->feature.min_x-bt->feature.max_x)*.5;
          const double pd=t && t!=bt?(part.min_x+part.max_x-t->feature.min_x-t->feature.max_x)*.5:bd;
          // Both observed displacements must agree within current spatial resolution.
          if(std::abs(bd-pd)>std::max(bev_x_res_,raw_vehicle_box_margin_m_))continue;
          if(match>=0){match=-2;break;}match=int(pi);pt=t;
        }
        if(match<0)continue;
        const auto&part=*candidates[match];
        bool competing_body=false;
        for(size_t oi=0;oi<candidates.size();++oi){
          if(oi==bi || oi==size_t(match) || claimed[oi])continue;
          const auto&other=*candidates[oi];
          if(other.component_type=="vehicle_like" && other.length>vehicle_max_width_ &&
             measuredDeckConnection(other,part,points)){competing_body=true;break;}
        }
        if(competing_body)continue;
        std::vector<int>indices=body.indices;indices.insert(indices.end(),part.indices.begin(),part.indices.end());
        auto cluster=makeFemCluster(points,indices,body.id,"observed_body_components","","",0.0);
        const int saved=last_candidate_count_;auto built=buildFeaturesFromClusterResults(points,{cluster},foreground_indices);last_candidate_count_=saved;
        if(built.size()!=1 || built[0].component_type!="vehicle_like" || !built[0].rejection_reason.empty())continue;
        if(!bt->multi_component_body){const auto center=built[0].obb.center;
          for(const auto*f:{&body,&part})new_references[bt->id].push_back({
            f->obb.center.x-center.x,f->obb.center.y-center.y,f->obb.length,f->obb.width});}
        body_observations.push_back(std::move(built[0]));candidates[bi]=&body_observations.back();
        if(visibility_frame_valid_)visibility_model_.remapDepthOwners({{part.id,body.id}});
        claimed[bi]=claimed[match]=true;if(pt && pt!=bt)aliases.push_back(pt->id);
        candidates[match]=nullptr;
      }
      candidates.erase(std::remove(candidates.begin(),candidates.end(),nullptr),candidates.end());
      tracks_.erase(std::remove_if(tracks_.begin(),tracks_.end(),[&](const Track&t){
        return std::find(aliases.begin(),aliases.end(),t.id)!=aliases.end();}),tracks_.end());
      for(auto &t:tracks_){const auto at=new_references.find(t.id);
        if(at!=new_references.end())t.component_references=at->second;}
    }
    last_tracking_candidate_count_ = static_cast<int>(candidates.size());
    last_active_track_count_before_ = static_cast<int>(std::count_if(
        tracks_.begin(), tracks_.end(), [&](const Track & tr) { return !lostVehicle(tr); }));

    for (auto & tr : tracks_) {
      tr.status_velocity_reinitialized = false;
      const auto lost_predict_start = std::chrono::steady_clock::now();
      const bool was_lost = lostVehicle(tr);
      // Kalman prediction advances once per input frame, also for a generic
      // track without confirmed geometry. Reusing last_time after a miss
      // would integrate overlapping intervals (0.1, 0.2, 0.3, ... seconds).
      const bool frame_prediction = persistentVehicle(tr) ||
          (perception_mode_ != "crane_beam_work" && kalmanEnabled());
      const double previous_time = frame_prediction && tr.last_prediction_time > 0.0 ?
          tr.last_prediction_time : tr.last_time;
      auto & vm = tr.observation_memory;
      // Prediction has a strict horizon.  DORMANT keeps the final estimate
      // frozen, regardless of how many hours the identity remains in memory.
      const double prediction_end = predictiveLostVehicle(tr) ?
          vm.lost_since + vehicle_lost_memory_sec_ : stamp;
      const double effective_stamp = cx128s2_bev_perception::vehicle_surface::predictiveStepEnd(
          vm.vehicle_state, previous_time, stamp, vm.lost_since, vehicle_lost_memory_sec_);
      const double dt = std::max(0.0, effective_stamp - previous_time);
      if (!dormantVehicle(tr)) {
        if (predictiveLostVehicle(tr)) {
          // A dropped cloud can span several seconds.  Cover the entire
          // remaining predictive window despite the KF per-step dt cap.
          double remaining = dt;
          while (remaining > 1e-9) {
            const double step = std::min(remaining, kf_max_dt_sec_);
            predictKalmanForTrack(tr, step);
            remaining -= step;
          }
        } else predictKalmanForTrack(tr, dt);
        tr.last_prediction_time = effective_stamp;
        tr.x += tr.vx * dt;
        tr.y += tr.vy * dt;
        if (persistentVehicle(tr) && kalmanEnabled()) {
          tr.x = tr.kf.x();
          tr.y = tr.kf.y();
        }
      }
      if (predictiveLostVehicle(tr) && stamp >= prediction_end) {
        vm.final_predicted_x = outputX(tr);
        vm.final_predicted_y = outputY(tr);
        vm.final_predicted_valid = true;
        vm.vehicle_state = cx128s2_bev_perception::vehicle_surface::VehicleLifecycle::DORMANT;
        vm.dormant_since = prediction_end;
        ++vehicle_lost_prediction_expired_count_;
        ++vehicle_dormant_enter_count_;
      }
      vm.vehicle_quality = VehicleObservationQuality::NONE;
      vm.owned_component_count = vm.sparse_support_count = 0;
      vm.owned_component_centers.clear();
      vm.actual_support_world.clear();
      vm.geometry_update = "NONE";
      vm.association_source = "PREDICTION_ONLY";
      vm.measurement_std_used = vm.measurement_std_x = vm.measurement_std_y = 0.0;
      vm.frame_stamp = stamp;
      vm.predicted_x = outputX(tr);
      vm.predicted_y = outputY(tr);
      vm.predicted_yaw = vm.vehicle_obb.yaw;
      vm.measured_center_valid = false;
      vm.lateral_measurement_accepted = false;
      vm.yaw_update_accepted = vm.anchor_updated = false;
      vm.lateral_update_reason = vm.yaw_update_reason = "PREDICTION_ONLY";
      vm.lateral_innovation = vm.yaw_innovation = 0.0;
      vm.measured_yaw = std::numeric_limits<double>::quiet_NaN();
      vm.reacquire_best_score = vm.reacquire_second_score = -1.0;
      vm.reacquire_candidate_valid = false;
      vm.reacquire_result = was_lost ? "PREDICTION_ONLY" : "NONE";
      if (visibility_frame_valid_ && !dormantVehicle(tr)) {
        const auto visibility_track_start = std::chrono::steady_clock::now();
        tr.visibility = visibility_model_.evaluateAdaptiveBox(
            predictedVisibilityBox(tr), -2, hasReliableVisibilityBox(tr),
            trackPositionSigma(tr));
        const double elapsed_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - visibility_track_start).count();
        visibility_track_ms_ += elapsed_ms;
        adaptive_track_visibility_ms_ += elapsed_ms;
      } else {
        tr.visibility = VisibilityInfo{};
      }
      tr.observation_memory.missed_time_sec =
          std::max(0.0, stamp - tr.last_time);
      tr.observation_memory.miss_reason = tr.visibility.reason;
      if (tr.observation_memory.reliable) {
        tr.observation_memory.observation_age_sec =
            std::max(0.0, stamp - tr.observation_memory.last_reliable_stamp);
      }
      const double speed = std::hypot(tr.vx, tr.vy);
      if (!kalmanEnabled() && !persistentVehicle(tr) && speed > motion_speed_threshold_ &&
          !(tr.shape_memory.valid && tr.shape_memory.committed)) {
        tr.yaw = std::atan2(tr.vy, tr.vx);
      }
      tr.age++;
      tr.lost_frames++;
      tr.recent_hits.push_back(false);
      tr.recent_front_hits.push_back(false);
      trimHistory(tr);
      tr.detection = evaluateDetection(tr);
      if (was_lost) {
        const double elapsed = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - lost_predict_start).count();
        if (dormantVehicle(tr)) vehicle_dormant_management_ms_ += elapsed;
        else vehicle_lost_predict_ms_ += elapsed;
      }
    }
    // Reassemble only current foreground fragments of an already observed
    // far front. Keep the ordinary observation when it already passes; do not
    // grow a body, extend memory matching, or change the global BEV clusters.
    std::vector<ObjectFeature> front_observations;
    front_observations.reserve(tracks_.size()+candidates.size());
    if (perception_mode_ != "crane_beam_work" && visibility_frame_valid_) {
      for (const auto & tr : tracks_) {
        const double age = stamp - tr.last_time;
        if (!tr.front_class_confirmed || persistentVehicle(tr) || lostVehicle(tr) ||
            tr.feature.range < mid_range_max_ || age <= 0.0 || age >= 0.6) continue;
        for (int front_attempt = 0; front_attempt < 2; ++front_attempt) {
          // Preserve the existing successful path; only failed reconstructions
          // retry the measured box propagated by existing KF velocity. The last
          // filtering residual is not physical movement of that measured box.
          const double sx = front_attempt == 0 ?
              outputX(tr) - tr.feature.centroid_x : outputVx(tr) * age;
          const double sy = front_attempt == 0 ?
              outputY(tr) - tr.feature.centroid_y : outputVy(tr) * age;
          const double margin = raw_vehicle_box_margin_m_;
          std::vector<const ObjectFeature *> pieces;
          bool ordinary_front = false, contested = false;
          for (const auto * f : candidates) {
            // Individual pieces need not span the whole observed front height.
            // The rebuilt current-point observation must still pass the unchanged
            // vehicleFrontSupported height, width and angular-support checks.
            if (f->range < mid_range_max_ || !f->inside_roi || !f->rejection_reason.empty() ||
                f->min_x < tr.feature.min_x + sx - margin ||
                f->max_x > tr.feature.max_x + sx + margin ||
                f->min_y < tr.feature.min_y + sy - margin ||
                f->max_y > tr.feature.max_y + sy + margin ||
                std::max(f->min_z, tr.feature.min_z) >
                    std::min(f->max_z, tr.feature.max_z)) continue;
            Track probe;
            probe.front_class_confirmed = tr.front_class_confirmed;
            probe.visibility = visibility_model_.evaluateAdaptiveBox(
                {f->min_x,f->max_x,f->min_y,f->max_y,f->min_z,f->max_z},
                f->id, false, trackPositionSigma(tr), tr.front_class_confirmed);
            if (vehicleFrontSupported(probe, *f, points)) ordinary_front = true;
            for (const auto & other : tracks_) {
              if (other.id == tr.id) continue;
              // Keep existing body ownership ahead of front aggregation, also
              // during LOST/reacquisition. Ambiguous support is left untouched.
              if (persistentVehicle(other) &&
                  cx128s2_bev_perception::vehicle_surface::componentOwnershipScore(
                      other.observation_memory.vehicle_obb,
                      {outputX(other), outputY(other)}, {f->centroid_x, f->centroid_y},
                      gateForRange(f->range)) >= 0.0) contested = true;
              if (other.lost_frames > 1) continue;
              // A piece closer to another live front/body is not ours to absorb.
              if ((other.front_class_confirmed || persistentVehicle(other)) &&
                  std::hypot(f->centroid_x-outputX(other), f->centroid_y-outputY(other)) <=
                  std::hypot(f->centroid_x-outputX(tr), f->centroid_y-outputY(tr))) contested = true;
            }
            pieces.push_back(f);
          }
          if (ordinary_front || contested) break;
          if (pieces.size() < 2) continue;
          std::vector<int> indices;
          for (const auto * f : pieces) indices.insert(indices.end(), f->indices.begin(), f->indices.end());
          auto cluster = makeFemCluster(points, indices, pieces.front()->id,
              "observed_front_support", "", "", 0.0);
          const int previous_count = last_candidate_count_;
          auto built = buildFeaturesFromClusterResults(points, {cluster}, foreground_indices);
          last_candidate_count_ = previous_count;
          if (built.size() != 1) continue;
          auto & f = built.front();
          Track probe;
          probe.front_class_confirmed = tr.front_class_confirmed;
          probe.visibility = visibility_model_.evaluateAdaptiveBox(
              {f.min_x,f.max_x,f.min_y,f.max_y,f.min_z,f.max_z},
              f.id, false, trackPositionSigma(tr), tr.front_class_confirmed);
          // Other fragments must not masquerade as an occluder and relax the
          // evidence requirement. All counted bins are current returns.
          probe.visibility.has_occluder = false;
          probe.visibility.occluded_fraction = 0.0;
          if (!vehicleFrontSupported(probe, f, points) ||
              std::hypot(f.centroid_x-outputX(tr), f.centroid_y-outputY(tr)) >= 0.8) continue;
          f.decision_reason += "+CURRENT_FRONT_COMPONENT_SUPPORT";
          std::unordered_map<int, int> front_owners;
          for (const auto * piece : pieces) front_owners[piece->id] = f.id;
          visibility_model_.remapDepthOwners(front_owners);
          front_observations.push_back(std::move(f));
          candidates.erase(std::remove_if(candidates.begin(), candidates.end(),
              [&](const ObjectFeature * candidate) {
                return std::find(pieces.begin(),pieces.end(),candidate) != pieces.end();
              }),candidates.end());
          candidates.push_back(&front_observations.back());
          break;
        }
      }
    }
    // Reuse current-point feature building for a vehicle/overhead merged
    // component. Keep its original ID and all existing ownership/support gates;
    // only an unambiguous confirmed body may supply the locked height limit.
    if (perception_mode_ != "crane_beam_work" && !points.empty()) {
      for (auto & candidate : candidates) {
        std::optional<ObjectFeature> body; bool ambiguous_height=false;
        for (const auto & tr : tracks_) {
          if (!persistentVehicle(tr) || lostVehicle(tr)) continue;
          const double top=confirmedVehicleTop(tr)+raw_vehicle_box_margin_m_;
          if (candidate->max_z<=top || candidate->min_z>top) continue;
          const double gate=std::min(6.0,gateForRange(candidate->range)+2.0*trackPositionSigma(tr));
          if (cx128s2_bev_perception::vehicle_surface::componentOwnershipScore(
              tr.observation_memory.vehicle_obb,{outputX(tr),outputY(tr)},
              {candidate->centroid_x,candidate->centroid_y},gate)<0) continue;
          std::vector<int> lower;
          for(int idx:candidate->indices)
            if(idx>=0 && static_cast<size_t>(idx)<points.size() && points[idx].z<=top)
              lower.push_back(idx);
          if(lower.empty())continue;
          auto cluster=makeFemCluster(points,lower,candidate->id,"confirmed_height_support","","",0.0);
          const int saved_count=last_candidate_count_;
          auto built=buildFeaturesFromClusterResults(points,{cluster},foreground_indices);
          last_candidate_count_=saved_count;
          if(built.size()!=1 || !built[0].rejection_reason.empty() ||
              vehicleObservationFailure(tr,built[0],points))continue;
          if(cx128s2_bev_perception::vehicle_surface::componentOwnershipScore(
              tr.observation_memory.vehicle_obb,{outputX(tr),outputY(tr)},
              {built[0].centroid_x,built[0].centroid_y},gate)<0)continue;
          if(body){ambiguous_height=true;break;}
          body=std::move(built[0]);
        }
        if(body && !ambiguous_height) {
          front_observations.push_back(std::move(*body));
          candidate=&front_observations.back();
        }
      }
    }
    // Confirmed vehicles claim all compatible components before ordinary
    // one-feature-per-track association or new-track creation.
    std::vector<std::vector<double>> vehicle_scores(
        candidates.size(), std::vector<double>(tracks_.size(), -1.0));
    std::vector<int> candidate_counts;
    std::vector<bool> uncertain_body_component(candidates.size(),false);
    candidate_counts.reserve(candidates.size());
    const auto ownership_start = std::chrono::steady_clock::now();
    for (size_t fi = 0; fi < candidates.size(); ++fi) {
      const auto & f = *candidates[fi];
      candidate_counts.push_back(f.point_count);
      for (size_t ti = 0; ti < tracks_.size(); ++ti) {
        const auto & tr = tracks_[ti];
        if (!persistentVehicle(tr) || lostVehicle(tr)) continue;
        const char * observation_failure=vehicleObservationFailure(tr,f,points);
        if(observation_failure){
          if(tr.multi_component_body &&
             (std::string(observation_failure)=="UNSUPPORTED_BODY_EXTENT" ||
              std::string(observation_failure)=="PART_GEOMETRY_UNCERTAIN")){
            const double gate=std::min(6.0,gateForRange(f.range)+2.0*trackPositionSigma(tr));
            uncertain_body_component[fi]=uncertain_body_component[fi] ||
                cx128s2_bev_perception::vehicle_surface::componentOwnershipScore(
                  tr.observation_memory.vehicle_obb,{outputX(tr),outputY(tr)},
                  {f.centroid_x,f.centroid_y},gate)>=0;
          }
          continue;
        }
        const double gate = std::min(6.0, gateForRange(f.range) +
            2.0 * trackPositionSigma(tr));
        bool extent_aligned = false;
        const auto center = correctedVehicleCenter(
            tr, f.centroid_x, f.centroid_y, 1.0, &f, &extent_aligned, &points);
        // If a large extent change has no consistent remembered end face,
        // do not feed its shifted box center back as a FULL measurement.
        if (f.obb.valid && f.obb.length >
            1.25 * tr.observation_memory.vehicle_obb.length && !extent_aligned) continue;
        vehicle_scores[fi][ti] =
            cx128s2_bev_perception::vehicle_surface::componentOwnershipScore(
                tr.observation_memory.vehicle_obb, {outputX(tr), outputY(tr)},
                extent_aligned ? center :
                    cx128s2_bev_perception::vehicle_surface::Point2{f.centroid_x, f.centroid_y}, gate);
      }
    }
    auto ownership = cx128s2_bev_perception::vehicle_surface::assignComponentOwners(
        vehicle_scores, candidate_counts, tracks_.size());
    // Largest support is not necessarily the best position observation.
    // Prefer the uniquely remembered short body part when its current shape
    // and innovation are valid; all other components keep their same owner.
    for(size_t ti=0;ti<tracks_.size();++ti){
      const auto&tr=tracks_[ti];if(!tr.multi_component_body)continue;
      int measured_part=-1;
      for(size_t fi=0;fi<candidates.size();++fi){
        const auto&f=*candidates[fi];if(ownership.owner[fi]!=int(ti) ||
            f.length>=.9*tr.observation_memory.vehicle_obb.length)continue;
        bool aligned=false;correctedVehicleCenter(tr,f.centroid_x,f.centroid_y,1,&f,&aligned,&points);
        if(!aligned)continue;
        if(measured_part>=0){measured_part=-2;break;}measured_part=int(fi);
      }
      if(measured_part>=0)ownership.primary[ti]=measured_part;
    }
    const auto & vehicle_owner = ownership.owner;
    auto ambiguous = ownership.ambiguous;
    // An uncertain merged body is not a new vehicle. Leave its current points
    // available to the existing RAW_SPARSE path instead of spawning a duplicate.
    for(size_t fi=0;fi<candidates.size();++fi)
      if(vehicle_owner[fi]<0 && uncertain_body_component[fi])ambiguous[fi]=true;
    const auto & primary = ownership.primary;
    vehicle_ownership_ms_ += std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - ownership_start).count();
    // A tentative generic Track supplies two observed displacements before
    // direction becomes stable.  OBB yaw is intentionally never used for sign.
    const auto fifo_start = std::chrono::steady_clock::now();
    using cx128s2_bev_perception::vehicle_surface::CorridorDirection;
    std::vector<CorridorDirection> candidate_direction(
        candidates.size(), CorridorDirection::UNKNOWN);
    std::vector<int> direction_source(candidates.size(), -1);
    std::vector<bool> direction_source_used(tracks_.size(), false);
    if (perception_mode_ != "crane_beam_work") {
      for (size_t fi = 0; fi < candidates.size(); ++fi) {
        if (ambiguous[fi]) continue;
        const auto & f = *candidates[fi];
        int source = -1;
        if (vehicle_owner[fi] >= 0 &&
            primary[vehicle_owner[fi]] == static_cast<int>(fi)) {
          source = vehicle_owner[fi];
        } else if (vehicle_owner[fi] < 0) {
          double nearest = std::numeric_limits<double>::infinity();
          for (size_t ti = 0; ti < tracks_.size(); ++ti) {
            const auto & tr = tracks_[ti];
            if (lostVehicle(tr) || persistentVehicle(tr) || direction_source_used[ti]) continue;
            const double dt = stamp - tr.last_time;
            if (dt <= 0.0 || dt > 1.5) continue;
            const double distance = std::hypot(
                f.centroid_x - tr.feature.centroid_x,
                f.centroid_y - tr.feature.centroid_y);
            if (distance < std::min(3.0, gateForRange(f.range)) && distance < nearest) {
              nearest = distance;
              source = static_cast<int>(ti);
            }
          }
        }
        if (source < 0 || direction_source_used[source]) continue;
        auto & tr = tracks_[source];
        const double dt = stamp - tr.last_time;
        if (dt <= 0.0 || dt > 1.5) continue;
        direction_source_used[source] = true;
        direction_source[fi] = source;
        const cx128s2_bev_perception::vehicle_surface::Point2 velocity{
            (f.centroid_x - tr.feature.centroid_x) / dt,
            (f.centroid_y - tr.feature.centroid_y) / dt};
        candidate_direction[fi] =
            cx128s2_bev_perception::vehicle_surface::advanceCorridorDirection(
                tr.observation_memory.corridor_direction,
                {f.centroid_x, f.centroid_y}, velocity, motion_speed_threshold_);
      }
    }
    vehicle_fifo_identity_ms_ += std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - fifo_start).count();
    // Reacquisition runs after ACTIVE ownership and before ordinary association
    // or Track creation. A component has at most one owner across both states.
    const auto reacquire_start = std::chrono::steady_clock::now();
    std::vector<int> lost_owner(candidates.size(), -1);
    std::vector<int> lost_primary(tracks_.size(), -1);
    std::vector<double> lost_primary_score(tracks_.size(), -1.0);
    if (vehicle_reacquire_enabled_ && perception_mode_ != "crane_beam_work") {
      for (size_t fi = 0; fi < candidates.size(); ++fi) {
        if (vehicle_owner[fi] >= 0 || lost_owner[fi] >= 0 || ambiguous[fi]) continue;
        const auto & f = *candidates[fi];
        double best = -1.0, second = -1.0;
        int best_ti = -1;
        bool tried = false;
        for (size_t ti = 0; ti < tracks_.size(); ++ti) {
          auto & tr = tracks_[ti];
          if (!lostVehicle(tr) || vehicleObservationFailure(tr, f, points)) continue;
          auto & mem = tr.observation_memory;
          if (!predictiveLostVehicle(tr)) continue;
          const double broad_x = 6.0 + 0.5 * mem.vehicle_obb.length;
          const double broad_y = std::max(1.1, 0.75 * mem.vehicle_obb.width + 0.4) + 0.4;
          if (std::abs(f.centroid_x - outputX(tr)) > broad_x ||
              std::abs(f.centroid_y - outputY(tr)) > broad_y) continue;
          tried = true;
          double remembered_distance = std::numeric_limits<double>::infinity();
          for (const auto & local : mem.local_surface) {
            const auto world = cx128s2_bev_perception::vehicle_surface::toWorld(
                local, {outputX(tr), outputY(tr)}, mem.vehicle_obb.yaw);
            remembered_distance = std::min(remembered_distance,
                std::hypot(f.centroid_x - world.x, f.centroid_y - world.y));
          }
          if (!std::isfinite(remembered_distance)) remembered_distance = -1.0;
          const auto result = cx128s2_bev_perception::vehicle_surface::scoreReacquisition({
              {outputX(tr), outputY(tr)}, {f.centroid_x, f.centroid_y},
              mem.vehicle_obb, f.obb.valid ? f.obb.length : f.length,
              f.obb.valid ? f.obb.width : f.width,
              f.obb.valid ? f.obb.yaw : 0.0,
              f.obb.valid && f.obb.length >= 0.8 * mem.vehicle_obb.length &&
                  f.obb.width >= 0.7 * mem.vehicle_obb.width &&
                  f.point_count >= min_cluster_points_near_,
              mem.lateral_anchor.y, mem.lateral_anchor.valid,
              kalmanEnabled() && tr.kf.initialized() ? tr.kf.positionVarianceX() :
                  kf_measurement_position_std_m_ * kf_measurement_position_std_m_,
              kalmanEnabled() && tr.kf.initialized() ? tr.kf.positionVarianceY() :
                  kf_measurement_position_std_m_ * kf_measurement_position_std_m_,
              outputVx(tr), stamp - mem.lost_since, vehicle_lost_memory_sec_,
              remembered_distance});
          if (mem.reacquire_result == "PREDICTION_ONLY" ||
              result.value > mem.reacquire_best_score) {
            mem.reacquire_best_score = result.value;
            mem.reacquire_position_error = result.position_error;
            mem.reacquire_lateral_error = result.lateral_error;
            mem.reacquire_yaw_error = result.yaw_error;
            mem.reacquire_candidate_center = {f.centroid_x, f.centroid_y};
            mem.reacquire_candidate_valid = true;
            mem.reacquire_result = result.reason;
          }
          if (result.value > best) {
            second = best; best = result.value; best_ti = static_cast<int>(ti);
          } else second = std::max(second, result.value);
        }
        if (!tried) continue;
        ++vehicle_reacquire_attempt_count_;
        const auto decision = cx128s2_bev_perception::vehicle_surface::decideReacquisition(
            best, second);
        if (decision == cx128s2_bev_perception::vehicle_surface::ReacquireDecision::AMBIGUOUS) {
          ++vehicle_reacquire_ambiguous_count_;
          ambiguous[fi] = true;
          if (best_ti >= 0) {
            auto & mem = tracks_[best_ti].observation_memory;
            mem.reacquire_second_score = second;
            mem.reacquire_result = "AMBIGUOUS";
          }
          continue;
        }
        if (decision != cx128s2_bev_perception::vehicle_surface::ReacquireDecision::ACCEPTED ||
            best_ti < 0) {
          ++vehicle_reacquire_reject_count_;
          continue;
        }
        lost_owner[fi] = best_ti;
        auto & mem = tracks_[best_ti].observation_memory;
        mem.reacquire_second_score = second;
        mem.reacquire_result = "CANDIDATE";
        if (best > lost_primary_score[best_ti]) {
          lost_primary_score[best_ti] = best;
          lost_primary[best_ti] = static_cast<int>(fi);
        }
      }
      // Long-term EXIT identity is the queue front, never the highest scoring
      // DORMANT Track.  A failed front validation cannot select a later ID.
      const auto fifo_match_start = std::chrono::steady_clock::now();
      for (size_t fi = 0; fi < candidates.size(); ++fi) {
        if (vehicle_owner[fi] >= 0 || lost_owner[fi] >= 0 || ambiguous[fi]) continue;
        if (candidate_direction[fi] == CorridorDirection::UNKNOWN) {
          if (inside_vehicle_queue_.size() > 0) ++vehicle_fifo_direction_unknown_count_;
          continue;
        }
        if (candidate_direction[fi] != CorridorDirection::EXIT &&
            !(candidates[fi]->range >= mid_range_max_ &&
              candidate_direction[fi] == CorridorDirection::ENTRY)) continue;
        ++vehicle_fifo_exit_candidate_count_;
        const int front_id = inside_vehicle_queue_.front();
        if (front_id < 0) continue;
        int front_ti = -1;
        for (size_t ti = 0; ti < tracks_.size(); ++ti)
          if (tracks_[ti].id == front_id) { front_ti = static_cast<int>(ti); break; }
        if (front_ti < 0) {
          ++vehicle_fifo_anomaly_count_;
          continue;
        }
        auto & front = tracks_[front_ti];
        auto & mem = front.observation_memory;
        // Continued ENTRY recovery cannot hold a new entering vehicle behind an ACTIVE front.
        if (candidate_direction[fi] == CorridorDirection::ENTRY &&
            (!dormantVehicle(front) || mem.corridor_direction.stable != CorridorDirection::ENTRY)) continue;
        // Existing FIFO identity recovery is independent of new-candidate memory policy.
        if (!dormantVehicle(front)) {
          ++vehicle_fifo_anomaly_count_;
          ambiguous[fi] = true;
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
              "[VEH_FIFO_ANOMALY] front_id=%d state=ACTIVE_OR_SHORT_LOST separate EXIT candidate held",
              front_id);
          continue;
        }
        const auto & f = *candidates[fi];
        const bool in_watch = std::any_of(roi_zones_.begin(), roi_zones_.end(),
            [&](const RoiZone & z) {
              return z.type == "watch" && pointInZone(f.centroid_x, f.centroid_y, z);
            });
        const bool enough_points = f.point_count >= std::max(3, min_cluster_points_near_ / 2);
        const cx128s2_bev_perception::vehicle_surface::DormantEvidence evidence{
            {mem.last_observed_x, mem.last_observed_y},
            {f.centroid_x, f.centroid_y}, mem.vehicle_obb,
            f.obb.valid ? f.obb.length : f.length,
            f.obb.valid ? f.obb.width : f.width,
            f.obb.valid ? f.obb.yaw : 0.0,
            f.obb.valid && f.obb.length >= 0.8 * mem.vehicle_obb.length &&
                f.obb.width >= 0.7 * mem.vehicle_obb.width,
            in_watch, mem.lateral_anchor.y, mem.lateral_anchor.valid,
            stamp - mem.dormant_since, vehicle_identity_memory_sec_};
        // Old coordinates anchor a reachable lane region, not today's position.
        const double unseen = std::max(0.0, stamp - mem.last_seen_stamp);
        const double reachable = std::min(long_range_max_, std::max(3.0,
            std::hypot(outputVx(front), outputVy(front)) * unseen +
            0.5 * kf_process_accel_std_mps2_ * unseen * unseen));
        const bool continued_entry = candidate_direction[fi] == CorridorDirection::ENTRY;
        const bool direction_valid = !continued_entry ||
            (mem.corridor_direction.stable == CorridorDirection::ENTRY &&
             std::hypot(f.centroid_x, f.centroid_y) + mem.vehicle_obb.length * 0.5 >=
             std::hypot(mem.last_observed_x, mem.last_observed_y));
        const auto validation = cx128s2_bev_perception::vehicle_surface::validateFifoExit(evidence, reachable);
        const auto diagnostic = cx128s2_bev_perception::vehicle_surface::scoreDormantReacquisition(evidence, reachable);
        const char * observation_failure = vehicleObservationFailure(front, f, points);
        bool visible_memory_surface = false;
        if (validation.valid && !observation_failure && visibility_frame_valid_ &&
            f.max_z >= mem.vehicle_z_center - mem.vehicle_height * 0.5 &&
            f.min_z <= mem.vehicle_z_center + mem.vehicle_height * 0.5) {
          const auto center = evidence.yaw_reliable ? f.obb.center : correctedVehicleCenter(
              front, f.centroid_x, f.centroid_y,
              cx128s2_bev_perception::vehicle_surface::lateralSurfaceGain(
                  f.range, near_range_max_, mid_range_max_, false));
          const double z = std::clamp(f.centroid_z,
              mem.vehicle_z_center - mem.vehicle_height * 0.5,
              mem.vehicle_z_center + mem.vehicle_height * 0.5);
          for (const auto & local : mem.local_surface) {
            const auto world = cx128s2_bev_perception::vehicle_surface::toWorld(local, center, mem.vehicle_obb.yaw);
            if (world.x < f.min_x - 0.25 || world.x > f.max_x + 0.25 ||
                world.y < f.min_y - 0.25 || world.y > f.max_y + 0.25) continue;
            if (visibility_model_.evaluateStaticPoint(world.x, world.y, z).reason == VisibilityReason::OBSERVABLE) {
              visible_memory_surface = true; break;
            }
          }
        }
        const bool valid = mem.last_observed_valid && enough_points && direction_valid &&
            validation.valid && !observation_failure && visible_memory_surface &&
            cx128s2_bev_perception::vehicle_surface::decideReacquisition(diagnostic.value, -1.0) ==
                cx128s2_bev_perception::vehicle_surface::ReacquireDecision::ACCEPTED;
        const auto choice = cx128s2_bev_perception::vehicle_surface::chooseFifoExit(
            inside_vehicle_queue_, candidate_direction[fi], true, valid, continued_entry);
        ++vehicle_dormant_reacquire_attempt_count_;
        mem.reacquire_best_score = diagnostic.value;
        mem.reacquire_position_error = diagnostic.position_error;
        mem.reacquire_lateral_error = diagnostic.lateral_error;
        mem.reacquire_yaw_error = diagnostic.yaw_error;
        mem.reacquire_candidate_center = {f.centroid_x, f.centroid_y};
        mem.reacquire_candidate_valid = true;
        if (choice.decision != cx128s2_bev_perception::vehicle_surface::FifoExitDecision::RESERVE) {
          ++vehicle_fifo_validation_reject_count_;
          const std::string reason = !mem.last_observed_valid ? "NO_OBSERVED_ANCHOR" :
              (!enough_points ? "INSUFFICIENT_SUPPORT" :
              (!direction_valid ? "DIRECTION_MISMATCH" :
              (!validation.valid ? validation.reason :
              (observation_failure ? observation_failure :
              (!visible_memory_surface ? "NO_VISIBLE_MEMORY_SURFACE" :
              (diagnostic.value < 0.0 ? diagnostic.reason : "LOW_IDENTITY_SCORE"))))));
          if (mem.fifo_last_reject_reason != reason) {
            static rclcpp::Clock reject_log_clock(RCL_STEADY_TIME);
            RCLCPP_WARN_THROTTLE(get_logger(), reject_log_clock, 2000,
                "[VEH_FIFO_REJECT] front_id=%d reason=%s queue=%s",
                front_id, reason.c_str(), fifoQueueString().c_str());
            mem.fifo_last_reject_reason = reason;
          }
          mem.reacquire_result = reason;
          continue;
        }
        mem.fifo_last_reject_reason.clear();
        lost_owner[fi] = front_ti;
        vehicle_scores[fi][front_ti] = diagnostic.value;
        if (diagnostic.value > lost_primary_score[front_ti]) {
          lost_primary_score[front_ti] = diagnostic.value;
          lost_primary[front_ti] = static_cast<int>(fi);
        }
        mem.reacquire_result = "FIFO_CANDIDATE";
        if (mem.reacquire_pending.coherent_frames == 0) {
          RCLCPP_INFO(get_logger(),
              "[VEH_FIFO_EXIT_CAND] front_id=%d obs=(%.2f,%.2f) direction=%s validation=PASS decision=PENDING",
              front_id, f.centroid_x, f.centroid_y,
              cx128s2_bev_perception::vehicle_surface::directionName(candidate_direction[fi]));
        }
        pending_exit_track_id_ = front_id;
      }
      vehicle_fifo_identity_ms_ += std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - fifo_match_start).count();
      // Distinct plausible vehicles cannot both claim the FIFO front. Nearby
      // pieces of one remembered box remain eligible as extra owned components.
      for (size_t ti = 0; ti < tracks_.size(); ++ti) {
        if (!dormantVehicle(tracks_[ti]) || lost_primary[ti] < 0) continue;
        const auto & best = *candidates[lost_primary[ti]];
        auto & mem = tracks_[ti].observation_memory;
        double second = -1.0;
        for (size_t fi = 0; fi < candidates.size(); ++fi) {
          if (lost_owner[fi] != static_cast<int>(ti) || static_cast<int>(fi) == lost_primary[ti]) continue;
          const auto delta = cx128s2_bev_perception::vehicle_surface::toLocal(
              {candidates[fi]->centroid_x, candidates[fi]->centroid_y},
              {best.centroid_x, best.centroid_y}, mem.vehicle_obb.yaw);
          if (std::abs(delta.x) > mem.vehicle_obb.length * 0.5 ||
              std::abs(delta.y) > mem.vehicle_obb.width * 0.5)
            second = std::max(second, vehicle_scores[fi][ti]);
        }
        mem.reacquire_best_score = lost_primary_score[ti];
        mem.reacquire_second_score = second;
        if (cx128s2_bev_perception::vehicle_surface::decideReacquisition(
            mem.reacquire_best_score, second) !=
            cx128s2_bev_perception::vehicle_surface::ReacquireDecision::ACCEPTED) {
          for (size_t fi = 0; fi < candidates.size(); ++fi)
            if (lost_owner[fi] == static_cast<int>(ti)) { lost_owner[fi] = -1; ambiguous[fi] = true; }
          lost_primary[ti] = -1;
          mem.reacquire_result = "AMBIGUOUS";
        }
      }
      // A first matching frame reserves its component without changing the
      // Track lifecycle. Only coherent support in a second frame can reactivate.
      for (size_t ti = 0; ti < tracks_.size(); ++ti) {
        if (!lostVehicle(tracks_[ti])) continue;
        auto & mem = tracks_[ti].observation_memory;
        const int fi = lost_primary[ti];
        if (fi < 0) {
          mem.reacquire_pending.coherent_frames = 0;
          if (pending_exit_track_id_ == tracks_[ti].id) pending_exit_track_id_ = -1;
          continue;
        }
        const auto & f = *candidates[fi];
        const auto previous_candidate = mem.reacquire_pending.center;
        const double candidate_dt = stamp - mem.reacquire_pending.stamp;
        bool confirmed = cx128s2_bev_perception::vehicle_surface::advanceReacquirePending(
            mem.reacquire_pending, {f.centroid_x, f.centroid_y}, stamp,
            dormantVehicle(tracks_[ti]) ?
                cx128s2_bev_perception::vehicle_surface::Point2{0.0, 0.0} :
                cx128s2_bev_perception::vehicle_surface::Point2{
                    outputVx(tracks_[ti]), outputVy(tracks_[ti])});
        if (dormantVehicle(tracks_[ti]) &&
            (f.obb.length < 0.8 * mem.vehicle_obb.length || f.obb.width < 0.7 * mem.vehicle_obb.width)) {
          if (candidate_dt > 0.0 && candidate_dt < 0.6 && mem.reacquire_pending.coherent_frames > 1) {
            const auto motion = cx128s2_bev_perception::vehicle_surface::signedCorridorDirection(
                {f.centroid_x, f.centroid_y},
                {(f.centroid_x - previous_candidate.x) / candidate_dt,
                 (f.centroid_y - previous_candidate.y) / candidate_dt}, motion_speed_threshold_);
            if (motion != candidate_direction[fi]) mem.reacquire_pending.coherent_frames = 1;
          }
          confirmed = confirmed && mem.reacquire_pending.coherent_frames >= 3;
        }
        if (!confirmed) {
          lost_primary[ti] = -1;
          mem.reacquire_result = "PENDING";
        }
      }
    }
    vehicle_reacquire_ms_ += std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - reacquire_start).count();
    std::vector<int> assigned_track(candidates.size(), -1);
    std::vector<bool> used(tracks_.size(), false);
    std::vector<bool> reserved_extra(candidates.size(), false);
    for (size_t fi = 0; fi < candidates.size(); ++fi) {
      const int ti = vehicle_owner[fi] >= 0 ? vehicle_owner[fi] : lost_owner[fi];
      if (ti < 0) continue;
      auto & owner_mem = tracks_[ti].observation_memory;
      ++owner_mem.owned_component_count;
      if (owner_mem.owned_component_centers.size() < 32)
        owner_mem.owned_component_centers.push_back(
            {candidates[fi]->centroid_x, candidates[fi]->centroid_y});
      const auto & indices = candidates[fi]->indices;
      const size_t step = std::max<size_t>(1, indices.size() / 16);
      for (size_t k = 0; k < indices.size() &&
           owner_mem.actual_support_world.size() < 64; k += step) {
        const auto & p = points[indices[k]];
        owner_mem.actual_support_world.push_back({p.x, p.y});
      }
      if (primary[ti] == static_cast<int>(fi) ||
          lost_primary[ti] == static_cast<int>(fi)) {
        assigned_track[fi] = ti;
        used[ti] = true;
      } else reserved_extra[fi] = true;
    }
    if (visibility_frame_valid_) {
      std::unordered_map<int, int> owner_map;
      for (size_t fi = 0; fi < candidates.size(); ++fi)
        if (vehicle_owner[fi] >= 0 || lost_owner[fi] >= 0)
          owner_map[candidates[fi]->id] = vehicleDepthOwner(
              tracks_[vehicle_owner[fi] >= 0 ? vehicle_owner[fi] : lost_owner[fi]].id);
      visibility_model_.remapDepthOwners(owner_map);
      for (auto & tr : tracks_) {
        if (!persistentVehicle(tr) || dormantVehicle(tr)) continue;
        tr.visibility = visibility_model_.evaluateAdaptiveBox(
            predictedVisibilityBox(tr), vehicleDepthOwner(tr.id), true,
            trackPositionSigma(tr));
        tr.observation_memory.miss_reason = tr.visibility.reason;
        predictVehicleSurface(tr);
      }
    }
    std::vector<double> assigned_shape_scores(candidates.size(), 0.0);
    std::vector<double> assigned_final_scores(candidates.size(), 0.0);
    std::vector<ShapeMemoryMatchBreakdown> assigned_breakdowns(candidates.size());
    int association_pairs_checked = 0;
    last_shape_memory_association_used_count_ = 0;
    last_shape_memory_match_score_max_ = 0.0;
    last_shape_memory_match_score_mean_ = 0.0;
    last_object_memory_match_attempt_count_ = 0;
    last_object_memory_applicable_match_count_ = 0;
    last_object_memory_attach_count_ = 0;
    last_object_memory_skipped_by_component_type_count_ = 0;
    shape_memory_reprepare_needed_ = false;
    double shape_score_sum = 0.0;
    int shape_score_count = 0;

    double match_duration_ms = 0.0;
    std::vector<int> retired_generic_track_ids;
    for (size_t fi = 0; fi < candidates.size(); ++fi) {
      if (vehicle_owner[fi] >= 0 || lost_owner[fi] >= 0 || ambiguous[fi]) continue;
      const auto & f = *candidates[fi];
      int best = -1;
      double best_d2 = std::numeric_limits<double>::max();
      double best_score = -1.0;
      double best_shape_score = 0.0;
      bool best_observed_front = false;
      ShapeMemoryMatchBreakdown best_breakdown;
      const double gate = gateForRange(f.range);
      for (size_t ti = 0; ti < tracks_.size(); ++ti) {
        // Confirmed vehicles are matched only by surface ownership or the
        // validated reacquisition paths above. A generic centroid gate must
        // not reaccept a component rejected by those geometry checks.
        if (used[ti] || lostVehicle(tracks_[ti]) || persistentVehicle(tracks_[ti])) continue;
        // A measured front candidate must not accumulate persistence by jumping
        // into a nearby floor/structure fragment through the broad generic gate.
        if (perception_mode_ != "crane_beam_work" &&
            (tracks_[ti].front_class_confirmed || std::count(tracks_[ti].recent_front_hits.begin(),
                       tracks_[ti].recent_front_hits.end(), true) > 0) &&
            f.component_type != "vehicle_like") continue;
        association_pairs_checked++;
        // A missed generic track can have different legacy-EMA and KF
        // predictions. Match the same predicted position that will receive
        // the Kalman correction, rather than admitting a distant component
        // through the legacy prediction and jumping the filtered output.
        const double predicted_x = perception_mode_ != "crane_beam_work" ?
            outputX(tracks_[ti]) : tracks_[ti].x;
        const double predicted_y = perception_mode_ != "crane_beam_work" ?
            outputY(tracks_[ti]) : tracks_[ti].y;
        const bool box_reference = tracks_[ti].position_from_body_box && f.obb.valid;
        const double dx = (box_reference ? f.obb.center.x : f.centroid_x) - predicted_x;
        const double dy = (box_reference ? f.obb.center.y : f.centroid_y) - predicted_y;
        const double d2 = dx * dx + dy * dy;
        const bool inside_gate = std::abs(dx) <= gate && std::abs(dy) <= gate && d2 < gate * gate;
        if (!enable_shape_memory_tracking_ || memoryMatchingBlockedPair(tracks_[ti], f) ||
            (perception_mode_ != "crane_beam_work" &&
             (!tracks_[ti].shape_memory.valid ||
              tracks_[ti].shape_memory.confidence < shape_memory_min_confidence_to_use_ ||
              (shape_memory_learning_mode_ == "manual_commit" && !tracks_[ti].shape_memory.committed)))) {
          const bool observed_front = perception_mode_ != "crane_beam_work" &&
              f.component_type == "vehicle_like" && !tracks_[ti].recent_front_hits.empty() &&
              (tracks_[ti].front_class_confirmed || std::count(tracks_[ti].recent_front_hits.begin(),
                         tracks_[ti].recent_front_hits.end(), true) > 0) &&
              d2 <= f.width * f.width;
          if (inside_gate && ((observed_front && !best_observed_front) ||
              (observed_front == best_observed_front && d2 < best_d2))) {
            best_d2 = d2;
            best = static_cast<int>(ti);
            best_observed_front = observed_front;
          }
          continue;
        }
        ShapeMemoryMatchBreakdown breakdown;
        double shape_score = 0.0;
        {
          auto match_t0 = std::chrono::steady_clock::now();
          shape_score = computeShapeMemoryMatchScore(tracks_[ti], f, points, &breakdown);
          auto match_t1 = std::chrono::steady_clock::now();
          match_duration_ms += std::chrono::duration<double, std::milli>(match_t1 - match_t0).count();
        }
        const double distance_score = clamp(1.0 - std::sqrt(d2) / std::max(0.1, gate), 0.0, 1.0);
        const double denom = std::max(1e-3, shape_memory_distance_weight_ + shape_memory_match_weight_);
        const double score =
            (shape_memory_distance_weight_ * distance_score + shape_memory_match_weight_ * shape_score) / denom;
        if ((perception_mode_ == "crane_beam_work" ?
             (inside_gate || shape_score >= shape_memory_min_match_score_) :
             (inside_gate && !vehicleObservationFailure(tracks_[ti], f, points))) && score > best_score) {
          best_d2 = d2;
          best = static_cast<int>(ti);
          best_score = score;
          best_shape_score = shape_score;
          best_breakdown = breakdown;
          if (perception_mode_ != "crane_beam_work")
            best_observed_front = tracks_[ti].front_class_confirmed &&
                f.component_type == "vehicle_like" && d2 <= f.width * f.width;
        }
      }
      if (best >= 0) {
        assigned_track[fi] = best;
        assigned_shape_scores[fi] = best_shape_score;
        assigned_final_scores[fi] = best_score;
        assigned_breakdowns[fi] = best_breakdown;
        if (best_shape_score >= shape_memory_min_match_score_) {
          last_shape_memory_association_used_count_++;
        }
        last_shape_memory_match_score_max_ =
            std::max(last_shape_memory_match_score_max_, best_shape_score);
        shape_score_sum += best_shape_score;
        shape_score_count++;
        used[best] = true;
      }
    }
    last_tracking_association_pairs_checked_ = association_pairs_checked;
    last_shape_memory_match_score_mean_ =
        shape_score_count > 0 ? shape_score_sum / shape_score_count : 0.0;

    for (size_t fi = 0; fi < candidates.size(); ++fi) {
      if (reserved_extra[fi] || ambiguous[fi]) continue;
      const auto & f = *candidates[fi];
      if (assigned_track[fi] < 0) {
        Track tr;
        tr.id = next_track_id_++;
        ObjectFeature tf = makeTrackFeature(f);
        tf.shape_memory_match_score = 0.0;
        tr.feature = tf;
      if(f.decision_reason.find("CLUSTER:observed_body_components")!=std::string::npos)tr.multi_component_body=true;
        // A short front/partial face is not a body-center observation.
        // Require longitudinal extent beyond the permitted transverse width;
        // retain the established centroid reference for those partial tracks.
        tr.position_from_body_box = perception_mode_ != "crane_beam_work" && kalmanEnabled() &&
            f.component_type == "vehicle_like" && f.obb.valid &&
            f.obb.length > vehicle_max_width_ && (measuredBodyReferenceSupported(f,points) || f.decision_reason.find("CLUSTER:observed_body_components")!=std::string::npos);
        tr.x = tr.position_from_body_box ? f.obb.center.x : f.centroid_x;
        tr.y = tr.position_from_body_box ? f.obb.center.y : f.centroid_y;
        tr.z = f.centroid_z;
        tr.vx = 0.0;
        tr.vy = 0.0;
        tr.yaw = std::numeric_limits<double>::quiet_NaN();
        tr.last_range = f.range;
        tr.last_time = stamp;
        tr.last_prediction_time = stamp;
        if (kalmanEnabled()) {
          tr.kf.initialize(tr.x, tr.y, kf_initial_position_std_m_,
                            kf_initial_velocity_std_mps_);
        }
        if (visibility_frame_valid_) {
          const auto adaptive_start = std::chrono::steady_clock::now();
          tr.visibility = visibility_model_.evaluateAdaptiveBox(
              {f.min_x, f.max_x, f.min_y, f.max_y, f.min_z, f.max_z},
              f.id, false, trackPositionSigma(tr));
          const double elapsed_ms = std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - adaptive_start).count();
          adaptive_track_visibility_ms_ += elapsed_ms;
          visibility_track_ms_ += elapsed_ms;
        } else {
          tr.visibility = f.visibility;
        }
        tr.age = 1;
        tr.hits = 1;
        tr.recent_hits.push_back(true);
        tr.recent_front_hits.push_back(vehicleFrontSupported(tr, f, points));
        initializeShapeMemory(tr, f);
        attachObjectMemoryBankIfMatched(tr, tf, points, stamp);
        tr.feature = tf;
      if(f.decision_reason.find("CLUSTER:observed_body_components")!=std::string::npos)tr.multi_component_body=true;
        tr.detection = evaluateDetection(tr);
        tr.observation_memory.frame_stamp = stamp;
        updateTrackObservationMemory(tr, f, stamp);
        updateVehicleSurface(tr, f, points);
        tracks_.push_back(tr);
        ++track_memory_created_;
        recording_track_features_[tr.id] = &f;
        continue;
      }
      Track & tr = tracks_[assigned_track[fi]];
      const bool reacquired = lostVehicle(tr);
      const bool dormant_reacquired = dormantVehicle(tr);
      const double dt = std::max(1e-3, stamp - tr.last_time);
      const bool vehicle_owned = persistentVehicle(tr);
      if (kalmanEnabled() && !vehicle_owned && !tr.position_from_body_box &&
          !tr.vehicle_status_initialized && tr.detection.class_name != "VEHICLE" &&
          f.range < near_range_max_ &&
          perception_mode_ != "crane_beam_work" && f.component_type == "vehicle_like" &&
          f.obb.valid && f.obb.length > vehicle_max_width_ &&
          (measuredBodyReferenceSupported(f,points) || f.decision_reason.find("CLUSTER:observed_body_components")!=std::string::npos)) {
        // A near-field candidate may first be an unclassified/merged component.
        // Never change the reference of an already public vehicle from a partial view. Translate
        // its reference when a measured body becomes available; retain ID,
        // velocity and covariance rather than interpreting the offset as motion.
        const double dx=f.obb.center.x-f.centroid_x,dy=f.obb.center.y-f.centroid_y;
        tr.x+=dx;tr.y+=dy;
        if (tr.kf.initialized()) tr.kf.reanchorPosition(tr.kf.x()+dx,tr.kf.y()+dy);
        tr.position_from_body_box=true;
        tr.observation_memory.predicted_x=outputX(tr);
        tr.observation_memory.predicted_y=outputY(tr);
      }
      const auto & obb = tr.observation_memory.vehicle_obb;
      // A height-filtered component is a partial observation of a formerly
      // merged object; it cannot expand the trusted body or get FULL gain.
      const bool full = vehicle_owned && f.obb.valid &&
          f.decision_reason.find("CLUSTER:confirmed_height_support")==std::string::npos &&
          f.obb.length >= 0.85 * obb.length && f.obb.width >= 0.8 * obb.width &&
          f.height >= 0.8 * tr.observation_memory.vehicle_height &&
          f.point_count >= min_cluster_points_near_ &&
          (dormant_reacquired || (tr.visibility.adaptive_expected_rays > 0 &&
           tr.visibility.adaptive_supported_rays >=
               std::max(1, tr.visibility.adaptive_expected_rays / 12) &&
           tr.visibility.occluded_fraction < 0.3));
      bool extent_aligned = false;
      const auto face_corrected = vehicle_owned ? correctedVehicleCenter(
          tr, f.centroid_x, f.centroid_y,
          cx128s2_bev_perception::vehicle_surface::lateralSurfaceGain(
              f.range, near_range_max_, mid_range_max_,
              f.point_count < min_cluster_points_near_), &f, &extent_aligned, &points) :
          cx128s2_bev_perception::vehicle_surface::Point2{f.centroid_x, f.centroid_y};
      auto corrected =
          extent_aligned ? face_corrected :
          (vehicle_owned && !full ? correctedVehicleCenter(
              tr, f.centroid_x, f.centroid_y,
              cx128s2_bev_perception::vehicle_surface::lateralSurfaceGain(
                  f.range, near_range_max_, mid_range_max_,
                  f.point_count < min_cluster_points_near_)) :
          cx128s2_bev_perception::vehicle_surface::Point2{
              (vehicle_owned || tr.position_from_body_box) && f.obb.valid ? f.obb.center.x : f.centroid_x,
              (vehicle_owned || tr.position_from_body_box) && f.obb.valid ? f.obb.center.y : f.centroid_y});
      // A side/partial return can still measure the full longitudinal extent.
      // Do not feed predicted X back as a measurement: that preserves stale
      // velocity and makes a stationary RViz box creep along the vehicle.
      // Reuse the existing FULL length criterion; shorter fragments retain the
      // established surface policy. Growing extents keep their face anchor.
      if (vehicle_owned && !full && !extent_aligned && f.obb.valid &&
          f.obb.length >= 0.85 * obb.length && f.obb.length <= obb.length &&
          corrected.x == outputX(tr) &&
          std::abs(f.obb.center.x - outputX(tr)) <=
              std::min(1.5, gateForRange(f.range)))
        corrected.x = f.obb.center.x;  // General vehicle World yaw is locked to 0.
      double mx = corrected.x;
      const bool body_pose_measurement=vehicle_owned &&
          measuredVehicleBodyAnchor(tr,f,points,mx);
      const double my = corrected.y;
      if (vehicle_owned) {
        auto & vm = tr.observation_memory;
        vm.raw_centroid_x = f.centroid_x;
        vm.raw_centroid_y = f.centroid_y;
        // Box orientation is frozen at confirmation; KF estimates translation only.
        vm.measured_yaw = std::numeric_limits<double>::quiet_NaN();
        vm.yaw_innovation = 0.0;
        vm.yaw_update_accepted = false;
        vm.yaw_update_reason = "LOCKED_INITIAL";
        vm.vehicle_quality = full ? VehicleObservationQuality::FULL :
            (f.point_count >= min_cluster_points_near_ ?
             VehicleObservationQuality::PARTIAL : VehicleObservationQuality::SPARSE);
        vm.association_source = extent_aligned ? "EXTENT_ALIGNED" :
            (full ? "NORMAL_CLUSTER" : "PARTIAL_SUPPORT");
        if(body_pose_measurement && !extent_aligned)vm.association_source="MEASURED_BODY_ANCHOR";
        if (vm.vehicle_quality == VehicleObservationQuality::SPARSE) {
          if (!visibility_frame_valid_) {
            vm.vehicle_quality = VehicleObservationQuality::NONE;
            vm.association_source = "PREDICTION_ONLY";
            continue;
          }
          std::set<std::pair<int, int>> bins;
          const auto sensor = visibility_model_.pose().getOrigin();
          const double az_bin = std::max(0.02, visibility_model_.azimuthBinDeg());
          for (int idx : f.indices) {
            const auto & p = points[idx];
            const double az = std::atan2(p.y - sensor.y(), p.x - sensor.x()) * 180.0 / M_PI;
            bins.emplace(static_cast<int>(std::floor(az / az_bin)), static_cast<int>(p.ring));
          }
          vm.sparse_support_count = static_cast<int>(bins.size());
          int coherent = 1;
          if (!vm.sparse_history.empty()) {
            const auto & prev = vm.sparse_history.back();
            const double sparse_dt = stamp - prev.stamp;
            if (sparse_dt > 0.0 && sparse_dt < 0.6 &&
                std::hypot(f.centroid_x - prev.x - outputVx(tr) * sparse_dt,
                           f.centroid_y - prev.y - outputVy(tr) * sparse_dt) < 0.8)
              coherent = 2;
          }
          if (bins.size() >= 3)
            vm.sparse_history.push_back(
                {f.centroid_x, f.centroid_y, stamp, static_cast<int>(bins.size())});
          else vm.sparse_history.clear();
          while (vm.sparse_history.size() > 5) vm.sparse_history.pop_front();
          if (!cx128s2_bev_perception::vehicle_surface::enoughSparseSupport(
                  static_cast<int>(bins.size()), coherent)) {
            vm.vehicle_quality = VehicleObservationQuality::NONE;
            vm.association_source = "PREDICTION_ONLY";
            continue;
          }
          vm.association_source = "SPARSE_SUPPORT";
        }
      }
      // Sparse validation may still reject this frame. Change lifecycle only
      // after the observation has passed its quality gate.
      if (reacquired) {
        auto & mem = tr.observation_memory;
        if (dormant_reacquired) {
          // The 4-second KF pose is only a last estimate, not a 10-hour
          // position measurement.  Restart velocity at the observed location.
          tr.x = mx;
          tr.y = my;
          tr.vx = tr.vy = 0.0;
          if (kalmanEnabled()) tr.kf.initialize(tr.x, tr.y,
              kf_initial_position_std_m_, kf_initial_velocity_std_mps_);
          // Reacquisition restarts the KF velocity. Its first zero is not a
          // new stationary observation, even when the remembered ID was stopped.
          tr.status_velocity_reinitialized = true;
          tr.last_prediction_time = stamp;
          ++vehicle_dormant_reacquire_success_count_;
          ++vehicle_fifo_reacquire_success_count_;
          mem.fifo_phase = candidate_direction[fi] == CorridorDirection::ENTRY ?
              cx128s2_bev_perception::vehicle_surface::FifoPhase::INSIDE :
              cx128s2_bev_perception::vehicle_surface::FifoPhase::EXITING;
          mem.corridor_direction.stable = candidate_direction[fi];
          if (direction_source[fi] >= 0 &&
              direction_source[fi] != assigned_track[fi] &&
              !persistentVehicle(tracks_[direction_source[fi]]))
            retired_generic_track_ids.push_back(tracks_[direction_source[fi]].id);
          pending_exit_track_id_ = -1;
          RCLCPP_INFO(get_logger(), "[VEH_FIFO_REACQUIRE] id=%d queue_front=%d state=DORMANT->%s queue=%s",
              tr.id, inside_vehicle_queue_.front(),
              cx128s2_bev_perception::vehicle_surface::fifoPhaseName(mem.fifo_phase), fifoQueueString().c_str());
        } else ++vehicle_reacquire_success_count_;
        mem.body_pose_anchor_valid=false;
        mem.vehicle_state = cx128s2_bev_perception::vehicle_surface::VehicleLifecycle::ACTIVE;
        mem.reacquire_geometry_hold_frames = 3;
        mem.reacquire_result = "REACQUIRED";
        mem.lost_since = mem.dormant_since = 0.0;
        mem.final_predicted_valid = false;
      }
      const double meas_vx = (mx - tr.x) / dt;
      const double meas_vy = (my - tr.y) / dt;
      tr.vx = 0.45 * tr.vx + 0.55 * meas_vx;
      tr.vy = 0.45 * tr.vy + 0.55 * meas_vy;
      if (kalmanEnabled()) {
        if (vehicle_owned) {
          updateConfirmedVehicleKalman(
              tr, mx, my, f.range, tr.observation_memory.vehicle_quality);
          tr.vx = tr.kf.vx();
          tr.vy = tr.kf.vy();
        } else {
          tr.kf.update(mx, my, kf_measurement_position_std_m_);
          tr.observation_memory.measurement_std_used = kf_measurement_position_std_m_;
          if (tr.position_from_body_box) {
            auto & mem=tr.observation_memory;
            mem.measured_x=mx;mem.measured_y=my;mem.measured_center_valid=true;
            mem.raw_centroid_x=f.centroid_x;mem.raw_centroid_y=f.centroid_y;
            mem.measurement_std_x=mem.measurement_std_y=kf_measurement_position_std_m_;
            mem.association_source="INITIAL_BODY_BOX_CENTER";
          }
        }
        if (kf_debug_) {
          const auto innovation = tr.kf.innovation();
          RCLCPP_INFO(get_logger(),
              "KF track=%d dt=%.4f meas=(%.3f,%.3f) filtered=(%.3f,%.3f) v=(%.3f,%.3f) "
              "innovation=(%.3f,%.3f) norm=%.3f Ppos=(%.5f,%.5f)",
              tr.id, dt, mx, my, tr.kf.x(), tr.kf.y(), tr.kf.vx(), tr.kf.vy(),
              innovation[0], innovation[1], std::hypot(innovation[0], innovation[1]),
              tr.kf.positionVarianceX(), tr.kf.positionVarianceY());
        }
      } else if (vehicle_owned) {
        auto & vm = tr.observation_memory;
        vm.measured_x = mx;
        vm.measured_y = my;
        vm.measured_center_valid = true;
        vm.lateral_innovation = my - vm.predicted_y;
      }
      const double speed = std::hypot(tr.vx, tr.vy);
      if (!kalmanEnabled() && !vehicle_owned && speed > motion_speed_threshold_ &&
          !(tr.shape_memory.valid && tr.shape_memory.committed)) {
        tr.yaw = std::atan2(tr.vy, tr.vx);
      }
      tr.x = vehicle_owned && kalmanEnabled() ? tr.kf.x() : mx;
      tr.y = vehicle_owned && kalmanEnabled() ? tr.kf.y() : my;
      tr.z = vehicle_owned ? tr.observation_memory.vehicle_z_center : f.centroid_z;
      tr.range_slope = (f.range - tr.last_range) / dt;
      tr.last_range = f.range;
      tr.last_time = stamp;
      if (vehicle_owned) {
        auto & mem = tr.observation_memory;
        // Preserve an already observed FIFO exit through the world origin.
        // Learn its direction once from genuine qualified measurements that
        // agree with the existing KF velocity, not from prediction or OBB yaw.
        if(tr.multi_component_body && !tr.observed_exit_direction_valid &&
           (extent_aligned || full) && mem.last_observed_valid &&
           stamp-mem.last_seen_stamp<=.6 &&
           mem.fifo_phase==cx128s2_bev_perception::vehicle_surface::FifoPhase::EXITING &&
           mem.corridor_direction.stable==cx128s2_bev_perception::vehicle_surface::CorridorDirection::EXIT){
          const double dx=mx-mem.last_observed_x,dy=my-mem.last_observed_y;
          const double displacement=std::hypot(dx,dy);
          if(displacement/dt>=motion_speed_threshold_ &&
             std::hypot(outputVx(tr),outputVy(tr))>=motion_speed_threshold_ &&
             dx*outputVx(tr)+dy*outputVy(tr)>0){
            tr.observed_exit_direction={dx/displacement,dy/displacement};
            tr.observed_exit_direction_valid=true;
          }
        }
        if (mem.last_observed_valid && inside_vehicle_queue_.front() == tr.id &&
            mem.corridor_direction.stable == CorridorDirection::EXIT &&
            cx128s2_bev_perception::vehicle_surface::fifoExitComplete(
                {mem.last_observed_x, mem.last_observed_y}, {mx, my}, mem.vehicle_obb.length))
          mem.fifo_exit_observed = true;
        mem.last_observed_x = mx;
        mem.last_observed_y = my;
        mem.last_observed_valid = true;
      }
      ObjectFeature tf = makeTrackFeature(f);
      tf.shape_memory_match_score = assigned_shape_scores[fi];
      tf.shape_memory_bbox_score = assigned_breakdowns[fi].bbox_score;
      tf.shape_memory_node_score = assigned_breakdowns[fi].node_score;
      tf.shape_memory_endpoint_relation_score = assigned_breakdowns[fi].endpoint_relation_score;
      tf.shape_memory_edge_score = assigned_breakdowns[fi].edge_score;
      tf.shape_memory_height_score = assigned_breakdowns[fi].height_score;
      tf.shape_memory_final_score = assigned_breakdowns[fi].final_score;
      tf.final_association_score = assigned_final_scores[fi];
      if (shouldTryObjectMemoryBankForTrack(tr)) {
        attachObjectMemoryBankIfMatched(tr, tf, points, stamp);
      }
      tr.feature = tf;
      if(f.decision_reason.find("CLUSTER:observed_body_components")!=std::string::npos)tr.multi_component_body=true;
      if (visibility_frame_valid_) {
        const auto adaptive_start = std::chrono::steady_clock::now();
        tr.visibility = visibility_model_.evaluateAdaptiveBox(
            {f.min_x, f.max_x, f.min_y, f.max_y, f.min_z, f.max_z},
            vehicle_owned ? vehicleDepthOwner(tr.id) : f.id, false, trackPositionSigma(tr),
            tr.front_class_confirmed && !vehicle_owned);
        const double elapsed_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - adaptive_start).count();
        adaptive_track_visibility_ms_ += elapsed_ms;
        visibility_track_ms_ += elapsed_ms;
      } else {
        tr.visibility = f.visibility;
      }
      updateShapeMemory(tr, f, points);
      if (tr.shape_memory.valid && tr.shape_memory.committed) {
        const double speed_sample = std::hypot(tr.vx, tr.vy);
        const double yaw_sample = getTrackMotionYaw(tr);
        const double range = std::hypot(tr.x, tr.y);
        const double radial_sample = range > 1e-3 ? (tr.x * tr.vx + tr.y * tr.vy) / range : 0.0;
        const int n = std::max(0, tr.shape_memory.learned_motion_samples);
        tr.shape_memory.learned_avg_speed =
            (tr.shape_memory.learned_avg_speed * n + speed_sample) / std::max(1, n + 1);
        tr.shape_memory.learned_motion_yaw =
            (tr.shape_memory.learned_motion_yaw * n + yaw_sample) / std::max(1, n + 1);
        tr.shape_memory.learned_radial_velocity =
            (tr.shape_memory.learned_radial_velocity * n + radial_sample) / std::max(1, n + 1);
        tr.shape_memory.learned_motion_samples = n + 1;
      }
      if (!memoryMatchingBlockedPair(tr, f) && tr.shape_memory.valid &&
          tr.shape_memory.committed && shape_memory_reprepare_enabled_) {
        if (tf.shape_memory_final_score < shape_memory_reprepare_match_threshold_) {
          tr.shape_memory.low_match_frames++;
        } else {
          tr.shape_memory.low_match_frames = 0;
          tr.shape_memory.reprepare_needed = false;
        }
        if (tr.shape_memory.low_match_frames >= shape_memory_reprepare_bad_frames_) {
          tr.shape_memory.reprepare_needed = true;
          shape_memory_reprepare_needed_ = true;
          clearObservedShapeCache(tr.observed_shape_cache);
        }
      }
      tr.hits++;
      tr.lost_frames = 0;
      if (vehicle_owned) {
        auto & mem = tr.observation_memory;
        mem.last_seen_stamp = stamp;
        mem.last_known_distance = f.range;
        mem.last_observed_quality = mem.vehicle_quality;
      }
      tr.recent_hits.back() = true;
      tr.recent_front_hits.back() = vehicleFrontSupported(tr, f, points);
      if (tr.recent_front_hits.back() && static_cast<int>(std::count(
          tr.recent_front_hits.begin(), tr.recent_front_hits.end(), true)) >= tentative_confirm_hits_)
        tr.front_class_confirmed = true;
      tr.detection = evaluateDetection(tr);
      updateTrackObservationMemory(tr, f, stamp);
      updateVehicleSurface(tr, f, points);
      if (tr.observation_memory.reacquire_geometry_hold_frames > 0)
        --tr.observation_memory.reacquire_geometry_hold_frames;
      recording_track_features_[tr.id] = &f;
    }

    // Clustering is not ownership: a rejected merged component can still
    // contain valid returns from a remembered vehicle. Exclude returns already
    // owned by confirmed tracks, then apply the existing sparse evidence gates.
    // The 1 m spatial hash avoids every-point x every-track scans.
    if (perception_mode_ != "crane_beam_work" && visibility_frame_valid_ &&
        std::any_of(tracks_.begin(), tracks_.end(), [&](const Track & tr) {
          return persistentVehicle(tr) && !lostVehicle(tr) && tr.lost_frames > 0;
        })) {
      const auto sparse_start = std::chrono::steady_clock::now();
      std::vector<uint8_t> clustered(points.size(), 0);
      for (size_t fi = 0; fi < candidates.size(); ++fi) {
        if (vehicle_owner[fi] < 0 && lost_owner[fi] < 0) continue;
        for (int idx : candidates[fi]->indices)
          if (idx >= 0 && static_cast<size_t>(idx) < clustered.size()) clustered[idx] = 1;
      }
      std::unordered_map<int64_t, std::vector<int>> sparse_grid;
      for (int idx : foreground_indices) {
        if (idx < 0 || static_cast<size_t>(idx) >= points.size() || clustered[idx]) continue;
        const auto & p = points[idx];
        sparse_grid[hash2d(static_cast<int>(std::floor(p.x)),
                           static_cast<int>(std::floor(p.y)))].push_back(idx);
      }
      const auto sensor = visibility_model_.pose().getOrigin();
      const double az_bin = std::max(0.02, visibility_model_.azimuthBinDeg());
      for (auto & tr : tracks_) {
        if (!persistentVehicle(tr) || lostVehicle(tr) || tr.lost_frames == 0) continue;
        auto & mem = tr.observation_memory;
        const int expected = tr.visibility.adaptive_expected_rays;
        if (expected <= 0) {
          mem.sparse_history.clear();
          RCLCPP_DEBUG(get_logger(),
              "[vehicle_surface][RAW_SPARSE] track=%d sensor_range=%.3f raw_sparse_points=0 "
              "independent_bins=0 adaptive_expected_rays=%d observable_samples=%d "
              "occluded_samples=%d occluded_fraction=%.3f known_occluder=%d "
              "effective_expected_support=0 support_ratio=0 result=REJECT reason=NO_EXPECTED_SUPPORT",
              tr.id, tr.visibility.sensor_distance, expected, tr.visibility.observable_samples,
              tr.visibility.occluded_samples, tr.visibility.occluded_fraction,
              tr.visibility.has_occluder);
          continue;
        }
        const double x = outputX(tr), y = outputY(tr);
        const double radius = std::min(10.0, mem.vehicle_obb.length * 0.6 + 1.0);
        std::set<std::pair<int, int>> bins;
        double sx = 0.0, sy = 0.0;
        int count = 0;
        std::vector<int> support_indices;
        support_indices.reserve(256);
        for (int ix = static_cast<int>(std::floor(x - radius)); ix <= static_cast<int>(std::ceil(x + radius)); ++ix)
          for (int iy = static_cast<int>(std::floor(y - radius)); iy <= static_cast<int>(std::ceil(y + radius)); ++iy) {
            const auto it = sparse_grid.find(hash2d(ix, iy));
            if (it == sparse_grid.end()) continue;
            for (int idx : it->second) {
              const auto & p = points[idx];
              if (p.z < mem.vehicle_z_center - mem.vehicle_height * 0.5 ||
                  p.z > mem.vehicle_z_center + mem.vehicle_height * 0.5) continue;
              if (cx128s2_bev_perception::vehicle_surface::surfaceDistance(
                  {p.x, p.y}, {x, y}, mem.vehicle_obb.yaw,
                  mem.vehicle_obb.length, mem.vehicle_obb.width) > 0.55) continue;
              if (visibility_model_.evaluateStaticPoint(p.x, p.y, p.z).reason !=
                  VisibilityReason::OBSERVABLE) continue;
              const double own_distance =
                  cx128s2_bev_perception::vehicle_surface::surfaceDistance(
                      {p.x, p.y}, {x, y}, mem.vehicle_obb.yaw,
                      mem.vehicle_obb.length, mem.vehicle_obb.width);
              bool contested = false;
              for (const auto & other : tracks_) {
                if (other.id == tr.id || !persistentVehicle(other)) continue;
                const double other_distance =
                    cx128s2_bev_perception::vehicle_surface::surfaceDistance(
                        {p.x, p.y}, {outputX(other), outputY(other)},
                        other.observation_memory.vehicle_obb.yaw,
                        other.observation_memory.vehicle_obb.length,
                        other.observation_memory.vehicle_obb.width);
                if (other_distance < own_distance + 0.15) {
                  contested = true;
                  break;
                }
              }
              if (contested) continue;
              const double az = std::atan2(p.y - sensor.y(), p.x - sensor.x()) * 180.0 / M_PI;
              bins.emplace(static_cast<int>(std::floor(az / az_bin)), static_cast<int>(p.ring));
              sx += p.x; sy += p.y; ++count;
              support_indices.push_back(idx);
              // Count real angular support over the queried cells. Truncating
              // at 256 points made the unchanged expected-ray ratio impossible
              // to satisfy for a well-observed nearby vehicle.
            }
          }
        // Apply the existing partial-cluster validator to the measured raw
        // support volume; the remembered full height/length is not all visible.
        ObjectFeature raw_support;
        raw_support.id = -1;
        raw_support.indices = std::move(support_indices);
        raw_support.point_count = count;
        raw_support.centroid_x = count > 0 ? sx / count : x;
        raw_support.centroid_y = count > 0 ? sy / count : y;
        raw_support.range = sensorRange(raw_support.centroid_x, raw_support.centroid_y);
        raw_support.min_x = raw_support.min_y = raw_support.min_z = 1e30;
        raw_support.max_x = raw_support.max_y = raw_support.max_z = -1e30;
        std::unordered_set<int64_t> raw_cells;
        std::vector<cx128s2_bev_perception::vehicle_surface::Point2> raw_xy;
        raw_xy.reserve(raw_support.indices.size());
        for (int idx : raw_support.indices) {
          const auto & p = points[idx];
          raw_support.min_x = std::min(raw_support.min_x, static_cast<double>(p.x));
          raw_support.max_x = std::max(raw_support.max_x, static_cast<double>(p.x));
          raw_support.min_y = std::min(raw_support.min_y, static_cast<double>(p.y));
          raw_support.max_y = std::max(raw_support.max_y, static_cast<double>(p.y));
          raw_support.min_z = std::min(raw_support.min_z, static_cast<double>(p.z));
          raw_support.max_z = std::max(raw_support.max_z, static_cast<double>(p.z));
          raw_cells.insert(bevKey(p));
          raw_xy.push_back({p.x,p.y});
        }
        raw_support.obb = cx128s2_bev_perception::vehicle_surface::fixedYawObb(raw_xy, mem.vehicle_obb.yaw);
        raw_support.length = raw_support.obb.length;
        raw_support.width = raw_support.obb.width;
        raw_support.height = raw_support.max_z - raw_support.min_z;
        raw_support.cell_count = static_cast<int>(raw_cells.size());
        const char * raw_failure = count > 0 ? vehicleObservationFailure(tr,raw_support,points) : "TOO_FEW_POINTS";
        const auto raw_visibility = count > 0 ? visibility_model_.evaluateAdaptiveBox(
            {raw_support.min_x,raw_support.max_x,raw_support.min_y,raw_support.max_y,
             raw_support.min_z,raw_support.max_z}, -1, false, 0.0) : VisibilityInfo{};
        const int raw_expected = raw_visibility.adaptive_expected_rays;
        mem.sparse_support_count = static_cast<int>(bins.size());
        double effective_expected = 0.0;
        const bool angular_support =
            cx128s2_bev_perception::vehicle_surface::enoughSparseSupport(
                static_cast<int>(bins.size()), 2, raw_expected,
                raw_visibility.occluded_fraction, raw_visibility.has_occluder,
                &effective_expected);
        const bool support_sufficient = !raw_failure && raw_expected > 0 && angular_support && count >= 3;
        const double support_ratio = effective_expected > 0.0 ?
            static_cast<double>(bins.size()) / effective_expected : 0.0;
        const double cx = count > 0 ? sx / count : x;
        const double cy = count > 0 ? sy / count : y;
        int coherent = 1;
        if (support_sufficient) {
          // Preserve compatible raw history: a newer partial-cluster centroid
          // can be on another face of the same vehicle. A validated cluster is
          // additional real evidence, never a replacement for valid raw support.
          const auto coherent_with = [&](const SparseSupportFrame & prev) {
            const double dt = stamp - prev.stamp;
            return dt > 0.0 && dt < 0.6 &&
                std::hypot(cx - prev.x - outputVx(tr) * dt,
                           cy - prev.y - outputVy(tr) * dt) < 0.8;
          };
          if (!mem.sparse_history.empty() && coherent_with(mem.sparse_history.back())) coherent = 2;
          if (coherent < 2 && mem.last_observed_valid && coherent_with(
              {mem.raw_centroid_x, mem.raw_centroid_y, mem.last_seen_stamp, 0})) coherent = 2;
          mem.sparse_history.push_back({cx, cy, stamp, static_cast<int>(bins.size())});
          while (mem.sparse_history.size() > 5) mem.sparse_history.pop_front();
        } else {
          mem.sparse_history.clear();
        }
        const bool accepted = support_sufficient &&
            cx128s2_bev_perception::vehicle_surface::enoughSparseSupport(
                static_cast<int>(bins.size()), coherent);
        const char * reason = raw_failure ? raw_failure :
            (raw_expected <= 0 ? "NO_EXPECTED_PARTIAL_SUPPORT" :
            (count < 3 ? "TOO_FEW_POINTS" :
            (bins.size() < 3 ? "TOO_FEW_INDEPENDENT_BINS" :
            (!support_sufficient ? "SPARSE_SUPPORT_TOO_LOW" :
            (!accepted ? "TEMPORAL_INCONSISTENCY" : "SPARSE_SUPPORT")))));
        RCLCPP_DEBUG(get_logger(),
            "[vehicle_surface][RAW_SPARSE] track=%d sensor_range=%.3f raw_sparse_points=%d "
            "independent_bins=%zu adaptive_expected_rays=%d partial_expected_rays=%d observable_samples=%d "
            "occluded_samples=%d occluded_fraction=%.3f known_occluder=%d "
            "effective_expected_support=%.3f support_ratio=%.6f coherent_frames=%d "
            "result=%s reason=%s",
            tr.id, tr.visibility.sensor_distance, count, bins.size(), expected, raw_expected,
            tr.visibility.observable_samples, tr.visibility.occluded_samples,
            tr.visibility.occluded_fraction, tr.visibility.has_occluder,
            effective_expected, support_ratio, coherent,
            accepted ? "PASS" : "REJECT", reason);
        if (!accepted) continue;
        const double range = sensorRange(cx, cy);
        const auto corrected = correctedVehicleCenter(
            tr, cx, cy,
            cx128s2_bev_perception::vehicle_surface::lateralSurfaceGain(
                range, near_range_max_, mid_range_max_, true));
        mem.raw_centroid_x = cx;
        mem.raw_centroid_y = cy;
        mem.measured_yaw = std::numeric_limits<double>::quiet_NaN();
        mem.yaw_update_reason = "NOT_FULL";
        updateConfirmedVehicleKalman(
            tr, corrected.x, corrected.y, range, VehicleObservationQuality::SPARSE);
        tr.x = kalmanEnabled() ? tr.kf.x() : corrected.x;
        tr.y = kalmanEnabled() ? tr.kf.y() : corrected.y;
        if (kalmanEnabled()) {
          tr.vx = tr.kf.vx();
          tr.vy = tr.kf.vy();
        }
        tr.last_time = stamp;
        tr.lost_frames = 0;
        tr.hits++;
        tr.recent_hits.back() = true;
        mem.last_seen_stamp = stamp;
        if (mem.last_observed_valid && inside_vehicle_queue_.front() == tr.id &&
            mem.corridor_direction.stable == CorridorDirection::EXIT &&
            cx128s2_bev_perception::vehicle_surface::fifoExitComplete(
                {mem.last_observed_x, mem.last_observed_y}, corrected, mem.vehicle_obb.length))
          mem.fifo_exit_observed = true;
        mem.last_observed_x = corrected.x;
        mem.last_observed_y = corrected.y;
        mem.last_observed_valid = true;
        mem.last_known_distance = range;
        mem.last_observed_quality = VehicleObservationQuality::SPARSE;
        mem.vehicle_quality = VehicleObservationQuality::SPARSE;
        mem.association_source = "SPARSE_SUPPORT";
        mem.owned_component_centers.push_back({cx, cy});
        mem.actual_support_world.push_back({cx, cy});
        // Preserve real observation provenance for object counts and the
        // recorder. The source component may have been rejected or merged.
        auto & support = recording_sparse_features_[tr.id];
        support.id = -1;
        support.centroid_x = cx; support.centroid_y = cy;
        support.range = range;
        support.indices = std::move(raw_support.indices);
        support.point_count = count;
        std::set<uint16_t> rings;
        for (int idx : support.indices) {
          rings.insert(points[idx].ring);
        }
        support.cell_count = raw_support.cell_count;
        support.ring_coverage = static_cast<int>(rings.size());
        recording_track_features_[tr.id] = &support;
        tr.detection = evaluateDetection(tr);
      }
      vehicle_sparse_support_ms_ += std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - sparse_start).count();
    }

    // Detection is evaluated during prediction and association above.  Update
    // the status layer only once here, after each track's final KF output and
    // motion_state for this cloud have been established.
    for (auto & tr : tracks_) {
      if (persistentVehicle(tr) && !lostVehicle(tr) &&
          cx128s2_bev_perception::vehicle_surface::vehicleLifecycleAfterMiss(
              tr.observation_memory.vehicle_state, tr.lost_frames,
              maxLostForTrack(tr)) ==
          cx128s2_bev_perception::vehicle_surface::VehicleLifecycle::LOST) {
        auto & mem = tr.observation_memory;
        mem.vehicle_state = cx128s2_bev_perception::vehicle_surface::VehicleLifecycle::LOST;
        mem.lost_since = stamp;
        mem.final_predicted_valid = false;
        if (!mem.last_observed_valid) {
          mem.last_observed_x = outputX(tr);
          mem.last_observed_y = outputY(tr);
          mem.last_observed_valid = true;
        }
        mem.reacquire_result = "LOST_ENTER";
        tr.detection = evaluateDetection(tr);
        ++vehicle_lost_enter_count_;
      }
      if (perception_mode_ != "crane_beam_work") {
        const bool initial_vehicle_status = !tr.vehicle_status_initialized &&
            tr.lost_frames == 0 && recording_track_features_.count(tr.id) > 0 &&
            tr.detection.is_valid && tr.detection.class_name == "VEHICLE" &&
            tr.detection.lifecycle_state != "CANDIDATE" &&
            tr.detection.lifecycle_state != "REJECTED" && physicalOutputAllowed(tr);
        // Preserve existing evidence clocks and all subsequent FSM decisions.
        evaluateVehicleStatus(tr, stamp);
        if (initial_vehicle_status) {
          tr.vehicle_status.value = cx128s2_bev_perception::VehicleStatusState{}.value;
          tr.vehicle_status_initialized = true;
        }
      }
      RCLCPP_DEBUG(get_logger(),
          "[TRACK_POLICY] id=%d sensor_range=%.3f mode=%s state=%s measurement_valid=%d "
          "lost_frames=%d cluster=%d points=%d predicted=(%.3f,%.3f) measured=(%.3f,%.3f) final=(%.3f,%.3f) "
          "physical_output=%d reason=%s",
          tr.id, trackSensorRange(tr), persistentVehicle(tr) ? "CONFIRMED_SURFACE" :
              memoryMatchingBlocked(trackSensorRange(tr)) ? "FAR_MEMORY_NO_MATCH" :
              (kalmanEnabled() ? "MID_NEAR_MEMORY" : "LEGACY"),
          dormantVehicle(tr) ? "DORMANT" : (predictiveLostVehicle(tr) ? "LOST" : "ACTIVE"),
          tr.lost_frames == 0, tr.lost_frames, tr.lost_frames == 0 ? tr.feature.id : -1,
          tr.lost_frames == 0 ? tr.feature.point_count : 0, tr.observation_memory.predicted_x,
          tr.observation_memory.predicted_y, tr.feature.centroid_x, tr.feature.centroid_y,
          outputX(tr), outputY(tr), physicalOutputAllowed(tr),
          tr.lost_frames == 0 ? "VALID_OBSERVATION" :
              (physicalOutputAllowed(tr) ? "BOUNDED_PREDICTION" : "NO_CURRENT_OBSERVATION"));
      if (persistentVehicle(tr)) {
        const auto & mem = tr.observation_memory;
        const auto observed = recording_track_features_.find(tr.id);
        const auto * feature = tr.lost_frames == 0 && observed != recording_track_features_.end() ?
            observed->second : nullptr;
        const double association_distance = feature ? std::hypot(
            feature->centroid_x - mem.predicted_x, feature->centroid_y - mem.predicted_y) :
            std::numeric_limits<double>::quiet_NaN();
        RCLCPP_DEBUG(
            get_logger(),
            "[VEH_TRACK] stamp=%.3f id=%d confirmed=1 state=%s lost_frames=%d active_limit=%d "
            "measurement_valid=%d cluster=%d points=%d sparse_bins=%d "
            "predicted=(%.3f,%.3f) measured=(%.3f,%.3f) final=(%.3f,%.3f) "
            "velocity=(%.3f,%.3f) association_distance=%.3f configured_gate_m=%.3f "
            "source=%s reacquired=%d dormant=%d physical_output=%d marker=%d",
            stamp, tr.id, dormantVehicle(tr) ? "DORMANT" :
                (predictiveLostVehicle(tr) ? "LOST" : "ACTIVE"),
            tr.lost_frames, maxLostForTrack(tr), mem.measured_center_valid,
            feature ? feature->id : -1, feature ? feature->point_count : 0,
            mem.sparse_support_count, mem.predicted_x, mem.predicted_y,
            mem.measured_center_valid ? mem.measured_x : std::numeric_limits<double>::quiet_NaN(),
            mem.measured_center_valid ? mem.measured_y : std::numeric_limits<double>::quiet_NaN(),
            outputX(tr), outputY(tr), outputVx(tr), outputVy(tr), association_distance,
            gateForRange(feature ? feature->range : trackSensorRange(tr)),
            mem.association_source.c_str(), mem.reacquire_result == "REACQUIRED",
            dormantVehicle(tr), physicalOutputAllowed(tr), physicalOutputAllowed(tr));
        RCLCPP_DEBUG_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "[vehicle_surface] id=%d quality=%s pose=(%.2f,%.2f,%.2f) geometry=(%.2f,%.2f) "
            "observed=(%.2f,%.2f) visible=%d/%d rays=%d/%d components=%d sparse_bins=%d "
            "R_std=(%.2f,%.2f) anchor_y=%.2f dY=%.2f yaw_delta=%.3f yaw_reason=%s "
            "geometry_update=%s source=%s",
            tr.id, vehicleQualityName(mem.vehicle_quality), outputX(tr), outputY(tr),
            mem.vehicle_obb.yaw, mem.vehicle_obb.length, mem.vehicle_obb.width,
            tr.feature.length, tr.feature.width, mem.visible_supported_samples,
            mem.expected_visible_samples, tr.visibility.adaptive_supported_rays,
            tr.visibility.adaptive_expected_rays, mem.owned_component_count,
            mem.sparse_support_count, mem.measurement_std_x, mem.measurement_std_y,
            mem.lateral_anchor.y, outputY(tr) - mem.predicted_y,
            cx128s2_bev_perception::vehicle_surface::angleDifference(
                mem.vehicle_obb.yaw, mem.predicted_yaw),
            mem.yaw_update_reason.c_str(), mem.geometry_update.c_str(),
            mem.association_source.c_str());
      }
    }

    // Registration is tied to confirmed vehicle identity and stable ENTRY,
    // never to the first temporary BEV component or generic Track ID.
    for (auto & tr : tracks_) {
      if (perception_mode_ == "crane_beam_work" || !persistentVehicle(tr) ||
          lostVehicle(tr)) continue;
      registerVehicleEntry(tr, stamp);
      auto & mem = tr.observation_memory;
      if (mem.fifo_phase != cx128s2_bev_perception::vehicle_surface::FifoPhase::NOT_REGISTERED) {
        if (mem.corridor_direction.stable ==
            cx128s2_bev_perception::vehicle_surface::CorridorDirection::EXIT)
          mem.fifo_phase = cx128s2_bev_perception::vehicle_surface::FifoPhase::EXITING;
        else if (tr.lost_frames == 0 && mem.corridor_direction.stable ==
            cx128s2_bev_perception::vehicle_surface::CorridorDirection::ENTRY)
          { mem.fifo_phase = cx128s2_bev_perception::vehicle_surface::FifoPhase::INSIDE;
            mem.fifo_exit_observed = false; }
      }
    }

    const size_t tracks_before_removal = tracks_.size();
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(), [&](const Track & tr) {
      if (std::find(retired_generic_track_ids.begin(), retired_generic_track_ids.end(), tr.id) !=
          retired_generic_track_ids.end()) return true;
      if (dormantVehicle(tr)) {
        const auto & mem = tr.observation_memory;
        if (mem.fifo_phase == cx128s2_bev_perception::vehicle_surface::FifoPhase::EXITING &&
            mem.fifo_exit_observed && inside_vehicle_queue_.front() == tr.id) {
          const std::string before = fifoQueueString();
          inside_vehicle_queue_.remove(tr.id);
          ++vehicle_fifo_exit_complete_count_;
          RCLCPP_INFO(get_logger(), "[VEH_FIFO_EXIT_COMPLETE] id=%d queue_before=%s queue_after=%s",
              tr.id, before.c_str(), fifoQueueString().c_str());
          return true;
        }
        const bool expired = cx128s2_bev_perception::vehicle_surface::vehicleDormantExpired(
            mem.vehicle_state, stamp - mem.dormant_since, vehicle_identity_memory_sec_);
        if (expired) {
          RCLCPP_DEBUG(get_logger(), "[TRACK_DELETE] id=%d reason=IDENTITY_EXPIRED", tr.id);
          ++vehicle_dormant_expired_count_;
          const std::string before = fifoQueueString();
          if (inside_vehicle_queue_.remove(tr.id)) {
            ++vehicle_fifo_expired_count_;
            RCLCPP_WARN(get_logger(),
                "[FIFO_ID_EXPIRED] track_id=%d entry_sequence=%llu queue_before=%s queue_after=%s",
                tr.id, static_cast<unsigned long long>(mem.entry_sequence),
                before.c_str(), fifoQueueString().c_str());
          }
        }
        return expired;
      }
      // The prediction loop above transitions LOST -> DORMANT at the configured horizon.
      // Keep the ID through that finite window; DORMANT expiration is handled above.
      if (predictiveLostVehicle(tr)) return false;
      const bool expired = tr.lost_frames > maxLostForTrack(tr);
      if (expired) RCLCPP_DEBUG(get_logger(), "[TRACK_DELETE] id=%d reason=LOST_FRAME_LIMIT lost_frames=%d",
          tr.id, tr.lost_frames);
      return expired;
    }), tracks_.end());
    track_memory_removed_ += tracks_before_removal - tracks_.size();

    if (static_cast<int>(tracks_.size()) > max_active_tracks_) {
      std::sort(tracks_.begin(), tracks_.end(), [&](const Track & a, const Track & b) {
        const int pa = persistentVehicle(a) ? (lostVehicle(a) ? 1 : 0) : 2;
        const int pb = persistentVehicle(b) ? (lostVehicle(b) ? 1 : 0) : 2;
        if (pa != pb) return pa < pb;
        if (a.lost_frames != b.lost_frames) return a.lost_frames < b.lost_frames;
        if (a.hits != b.hits) return a.hits > b.hits;
        return a.age > b.age;
      });
      track_memory_removed_ += tracks_.size() - max_active_tracks_;
      tracks_.resize(max_active_tracks_);
    }
    if (pending_exit_track_id_ >= 0 && std::none_of(tracks_.begin(), tracks_.end(),
        [&](const Track & tr) { return tr.id == pending_exit_track_id_; })) pending_exit_track_id_ = -1;
    for (auto it = recording_track_features_.begin(); it != recording_track_features_.end();) {
      if (std::none_of(tracks_.begin(), tracks_.end(),
          [&](const Track & tr) { return tr.id == it->first; })) it = recording_track_features_.erase(it);
      else ++it;
    }
    // The cap or an unexpected deletion must not leave dangling queue IDs.
    const std::vector<int> queued_ids(
        inside_vehicle_queue_.ids().begin(), inside_vehicle_queue_.ids().end());
    for (int id : queued_ids) {
      const bool exists = std::any_of(tracks_.begin(), tracks_.end(),
          [&](const Track & tr) { return tr.id == id; });
      if (!exists) {
        inside_vehicle_queue_.remove(id);
        ++vehicle_fifo_anomaly_count_;
        RCLCPP_ERROR(get_logger(), "[VEH_FIFO_ANOMALY] missing Track id=%d removed from queue", id);
      }
    }
    for (const auto & tr : tracks_) {
      if (tr.observation_memory.fifo_phase !=
          cx128s2_bev_perception::vehicle_surface::FifoPhase::NOT_REGISTERED &&
          !inside_vehicle_queue_.contains(tr.id)) {
        ++vehicle_fifo_anomaly_count_;
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
            "[VEH_FIFO_ANOMALY] registered Track id=%d absent from queue", tr.id);
      }
    }
    if (!inside_vehicle_queue_.unique()) {
      ++vehicle_fifo_anomaly_count_;
      RCLCPP_ERROR(get_logger(), "[VEH_FIFO_ANOMALY] duplicate queue ID queue=%s",
          fifoQueueString().c_str());
    }

    last_lost_vehicle_count_ = static_cast<int>(std::count_if(
        tracks_.begin(), tracks_.end(), [&](const Track & tr) { return lostVehicle(tr); }));
    last_lost_predictive_count_ = static_cast<int>(std::count_if(
        tracks_.begin(), tracks_.end(), [&](const Track & tr) { return predictiveLostVehicle(tr); }));
    last_dormant_count_ = static_cast<int>(std::count_if(
        tracks_.begin(), tracks_.end(), [&](const Track & tr) { return dormantVehicle(tr); }));
    last_active_track_count_ = static_cast<int>(tracks_.size()) - last_lost_vehicle_count_;
    last_shape_memory_track_count_ = static_cast<int>(std::count_if(
        tracks_.begin(), tracks_.end(), [](const Track & tr) { return tr.shape_memory.valid; }));
    // Keep the actual current support alive through recording/publication.
    // Generic track features intentionally discard heavy point indices.
    if (!front_observations.empty()) for (auto & observed : recording_track_features_) {
      if (std::any_of(front_observations.begin(), front_observations.end(),
          [&](const ObjectFeature & front) { return &front == observed.second; })) {
        recording_sparse_features_[observed.first] = *observed.second;
        observed.second = &recording_sparse_features_.at(observed.first);
      }
    }
    for(auto & observed:recording_track_features_) {
      if(std::any_of(body_observations.begin(),body_observations.end(),[&](const ObjectFeature&f){return &f==observed.second;})){
        recording_sparse_features_[observed.first]=*observed.second;
        observed.second=&recording_sparse_features_.at(observed.first);
      }
    }
    shape_memory_match_ms_ = match_duration_ms;
  }

  double gateForRange(double r) const {
    if (r < near_range_max_) return std::sqrt(gate_threshold_near_);
    if (r < mid_range_max_) return std::sqrt(gate_threshold_mid_);
    return std::sqrt(gate_threshold_long_);
  }

  int maxLostForRange(double r) const {
    if (r < near_range_max_) return max_lost_frames_near_;
    if (r < mid_range_max_) return max_lost_frames_mid_;
    return max_lost_frames_long_;
  }

  void trimHistory(Track & tr) const {
    while (static_cast<int>(tr.recent_front_hits.size()) > tentative_confirm_frames_) {
      tr.recent_front_hits.pop_front();
    }
    while (static_cast<int>(tr.recent_hits.size()) > tentative_confirm_frames_) {
      tr.recent_hits.pop_front();
    }
  }

  int recentHits(const Track & tr) const {
    return static_cast<int>(std::count(tr.recent_hits.begin(), tr.recent_hits.end(), true));
  }

  bool vehicleFrontSupported(const Track & tr, const ObjectFeature & f,
                                const std::vector<PointXYZIRT> & points) const {
    // Reuse the observed component, installation TF, adaptive angular footprint
    // and established sparse-support ratio. A previously confirmed front may
    // continue inside mid-range with the stricter local support minima. New
    // short candidates there still require the existing full-geometry path.
    // No hidden vehicle length is invented; no old points count as support.
    if (perception_mode_ == "crane_beam_work" || persistentVehicle(tr) ||
        !visibility_frame_valid_ ||
        (f.range < mid_range_max_ && !tr.front_class_confirmed) || !f.inside_roi ||
        !f.rejection_reason.empty() || f.component_type != "vehicle_like" ||
        !f.obb.valid || f.length >= 2.0 || f.length <= 0.0 ||
        f.width < vehicle_min_width_ || f.width > vehicle_max_width_ ||
        f.height < vehicle_min_height_ || f.height > vehicle_max_height_ ||
        f.point_count < (f.range < near_range_max_ ? min_cluster_points_near_ :
            (f.range < mid_range_max_ ? min_cluster_points_mid_ : min_cluster_points_long_)) ||
        f.cell_count < (f.range < near_range_max_ ? min_cluster_cells_near_ :
            (f.range < mid_range_max_ ? min_cluster_cells_mid_ : min_cluster_cells_long_)) ||
        f.ring_coverage < min_cluster_cells_near_) return false;
    const auto & v = tr.visibility;
    if (v.adaptive_expected_rays <= 0 || v.near_blind_zone ||
        !v.in_horizontal_fov || !v.in_vertical_fov || !v.in_valid_range ||
        v.observable_samples <= 0) return false;
    // The sensor must face the observed end face: viewing along the fixed
    // length axis, rather than treating a thin side or arbitrary wall as a front.
    const double dx = sensor_origin_x_ - f.obb.center.x;
    const double dy = sensor_origin_y_ - f.obb.center.y;
    const double c = std::cos(f.obb.yaw), sn = std::sin(f.obb.yaw);
    const double longitudinal = std::abs(c * dx + sn * dy);
    const double lateral = std::abs(-sn * dx + c * dy);
    if (longitudinal <= lateral || f.length >= f.width) return false;
    std::set<std::pair<int, int>> bins;
    const auto sensor_from_world = visibility_model_.pose().inverse();
    for (int idx : f.indices) {
      if (idx < 0 || static_cast<size_t>(idx) >= points.size()) continue;
      const auto & p = points[idx];
      const auto q = sensor_from_world * tf2::Vector3(p.x, p.y, p.z);
      double az = std::atan2(q.y(), q.x()) * 180.0 / M_PI;
      if (az < 0.0) az += 360.0;
      bins.emplace(static_cast<int>(std::floor(az / visibility_model_.azimuthBinDeg())), p.ring);
    }
    return cx128s2_bev_perception::vehicle_surface::enoughSparseSupport(
        static_cast<int>(bins.size()), 2, v.adaptive_expected_rays,
        v.occluded_fraction, v.has_occluder);
  }

  DetectionResult evaluateDetection(const Track & tr) const {
    DetectionResult best;
    const auto & f = tr.feature;
    const bool locked_vehicle = persistentVehicle(tr);
    const bool pending_geometry = perception_mode_ != "crane_beam_work" &&
        kalmanEnabled() && !locked_vehicle && tr.observation_memory.reliable &&
        f.obb.valid && f.point_count >= min_cluster_points_near_ &&
        f.width >= vehicle_min_width_ && f.width <= vehicle_max_width_ &&
        f.height >= vehicle_min_height_ && f.height <= vehicle_max_height_ &&
        f.length >= 2.0 && f.length <= vehicle_max_length_;
    const double vehicle_length = pending_geometry ? tr.observation_memory.reliable_length :
        (perception_mode_ != "crane_beam_work" && f.obb.valid ? f.obb.length : f.length);
    const double vehicle_width = pending_geometry ? tr.observation_memory.reliable_width :
        (perception_mode_ != "crane_beam_work" && f.obb.valid ? f.obb.width : f.width);
    const double vehicle_height = pending_geometry ? tr.observation_memory.reliable_height : f.height;
    const bool front = perception_mode_ != "crane_beam_work" && tr.lost_frames == 0 &&
        !tr.recent_front_hits.empty() && tr.recent_front_hits.back();
    const bool front_confirmed = front && (tr.front_class_confirmed || static_cast<int>(std::count(
        tr.recent_front_hits.begin(), tr.recent_front_hits.end(), true)) >= tentative_confirm_hits_);
    const bool confirmed = (front && tr.front_class_confirmed) ||
        (recentHits(tr) >= tentative_confirm_hits_ &&
         tr.hits >= tentative_confirm_hits_ && (!front || front_confirmed));
    const double speed = std::hypot(outputVx(tr), outputVy(tr));
    const double size_score =
        0.35 * clamp((vehicle_height - vehicle_min_height_) / std::max(0.1, vehicle_max_height_ - vehicle_min_height_), 0.0, 1.0) +
        0.25 * (vehicle_width >= vehicle_min_width_ && vehicle_width <= vehicle_max_width_ ? 1.0 : 0.0) +
        0.20 * ((vehicle_length >= 2.0 && vehicle_length <= vehicle_max_length_) || front ? 1.0 : 0.0) +
        0.20 * clamp(f.dynamic_score, 0.0, 1.0);

    if (vehicle_detector_enabled_ && (f.vehicle_height_supported || tr.front_class_confirmed) &&
        vehicle_height >= vehicle_min_height_ &&
        vehicle_height <= vehicle_max_height_ && vehicle_width >= vehicle_min_width_ &&
        vehicle_width <= vehicle_max_width_ && (vehicle_length >= 2.0 || front) &&
        vehicle_length <= vehicle_max_length_ &&
        size_score >= vehicle_min_score_) {
      best.class_name = "VEHICLE";
      best.score = size_score;
      best.is_valid = confirmed;
      best.lifecycle_state =
          confirmed ? (tr.lost_frames > 0 ? "TRACKED_PREDICTED" : "TRACKED") : "CANDIDATE";
      best.decision_reason = front ?
          (confirmed ? "VISIBLE_VEHICLE_FRONT+CONFIRMED_BY_PERSISTENCE" : "VISIBLE_VEHICLE_FRONT") :
          (confirmed ? "VEHICLE_SIZE_MATCH+CONFIRMED_BY_PERSISTENCE" : "VEHICLE_SIZE_MATCH");
    } else if (unknown_obstacle_detector_enabled_ &&
               tr.hits >= unknown_min_persistence_frames_) {
      best.class_name = "UNKNOWN";
      best.score = clamp(0.4 + 0.6 * f.dynamic_score, 0.0, 1.0);
      best.is_valid = confirmed;
      best.lifecycle_state =
          confirmed ? (tr.lost_frames > 0 ? "TRACKED_PREDICTED" : "TRACKED") : "CANDIDATE";
      best.decision_reason = "BACKGROUND_DIFF+UNKNOWN_PERSISTENCE";
    } else {
      best.class_name = "UNKNOWN";
      best.score = size_score;
      best.is_valid = false;
      best.lifecycle_state = "REJECTED";
      best.rejection_reason = !f.vehicle_height_supported ? "DISCONNECTED_VEHICLE_HEIGHT" :
          (f.rejection_reason.empty() ? "LOW_VEHICLE_SCORE" : f.rejection_reason);
    }
    if (!memoryMatchingBlockedPair(tr, f) && tr.shape_memory.committed &&
        tr.shape_memory.memory_confidence > 0.5 &&
        f.shape_memory_match_score > shape_memory_min_match_score_) {
      if (tr.shape_memory.class_label != "UNKNOWN" && !tr.shape_memory.class_label.empty()) {
        best.class_name = tr.shape_memory.class_label;
      } else if (tr.shape_memory.class_prior != "UNKNOWN") {
        best.class_name = tr.shape_memory.class_prior;
      }
      best.score = std::max(best.score,
          f.shape_memory_match_score * std::max(tr.shape_memory.class_confidence,
                                                tr.shape_memory.memory_confidence));
      best.is_valid = confirmed || tr.shape_memory.frozen;
      if (best.lifecycle_state == "REJECTED") {
        best.lifecycle_state = best.is_valid ? "TRACKED" : "CANDIDATE";
        best.rejection_reason.clear();
      }
      best.decision_reason += "+SHAPE_MEMORY_PRIOR";
    }
    // Optional site policy: require ENTRY before confirming a new vehicle.
    // When disabled, parked vehicles can confirm from measured geometry and
    // persistence. Actual ENTRY evidence still controls FIFO registration.
    if (vehicle_require_entry_direction_ && !locked_vehicle && perception_mode_ != "crane_beam_work" &&
        best.class_name == "VEHICLE" &&
        tr.observation_memory.corridor_direction.stable !=
            cx128s2_bev_perception::vehicle_surface::CorridorDirection::ENTRY) {
      best.is_valid = false;
      best.lifecycle_state = "CANDIDATE";
      best.decision_reason = "AWAITING_ENTRY_DIRECTION";
    }
    if (locked_vehicle) {
      best.class_name = "VEHICLE";
      best.is_valid = true;
      best.score = std::max(best.score, tr.observation_memory.geometry_confidence);
      best.lifecycle_state = dormantVehicle(tr) ? "DORMANT" :
          (predictiveLostVehicle(tr) ? "LOST_PREDICTIVE" :
          (tr.lost_frames > 0 ? "TRACKED_PREDICTED" : "TRACKED"));
      best.decision_reason = "PERSISTENT_VEHICLE_TRACK";
      best.rejection_reason.clear();
    }
    const double vx = outputVx(tr), vy = outputVy(tr);
    if (speed < motion_speed_threshold_) best.motion_state = "STATIONARY";
    else if (std::abs(vx) > direction_dominance_ratio_ * std::abs(vy)) {
      best.motion_state = vx >= 0.0 ? "MOVING_FORWARD" : "MOVING_BACKWARD";
    } else if (std::abs(vy) > direction_dominance_ratio_ * std::abs(vx)) {
      best.motion_state = vy >= 0.0 ? "MOVING_LEFT" : "MOVING_RIGHT";
    } else {
      best.motion_state = "MOVING_DIAGONAL";
    }
    return best;
  }

  void evaluateVehicleStatus(Track & tr, double stamp) {
    const std::string previous_status = tr.vehicle_status.value;
    const double x = outputX(tr), y = outputY(tr), vx = outputVx(tr), vy = outputVy(tr);
    const double distance = std::hypot(x, y);
    // Normal-mode exit is toward TF world (0,0), matching the FIFO corridor.
    // At the origin the radial direction is undefined; retain prior direction.
    const double range_speed = distance > 1e-3 ? (x * vx + y * vy) / distance : 0.0;
    const auto & memory = tr.observation_memory;
    const bool observed_exit = persistentVehicle(tr) && memory.fifo_exit_observed &&
        memory.fifo_phase == cx128s2_bev_perception::vehicle_surface::FifoPhase::EXITING &&
        memory.corridor_direction.stable ==
            cx128s2_bev_perception::vehicle_surface::CorridorDirection::EXIT;
    // Crossing world (0,0) reverses radial range without reversing the vehicle.
    // Reuse the already observed FIFO exit until real ENTRY evidence replaces it.
    double status_range_speed = observed_exit ? -std::abs(range_speed) : range_speed;
    // A front-only centroid can shift as different rings/surfaces return.
    // Reuse the observed two-frame corridor direction before treating its KF
    // velocity as motion. Uncorroborated motion pauses the existing status
    // evidence clock; it is not evidence that the vehicle is stationary.
    bool status_observed = tr.lost_frames == 0 && !lostVehicle(tr) &&
        !tr.status_velocity_reinitialized;
    if(tr.multi_component_body && tr.observed_exit_direction_valid &&
       memory.fifo_phase==cx128s2_bev_perception::vehicle_surface::FifoPhase::EXITING){
      // A current body measurement can reverse the physical direction; merely
      // crossing (0,0) cannot. Ambiguous position observations pause the existing
      // state evidence clock. FIFO completion/expiry remains entirely separate.
      const bool measured_direction=memory.vehicle_quality==VehicleObservationQuality::FULL ||
          memory.association_source=="EXTENT_ALIGNED";
      status_observed=status_observed && measured_direction;
      if(status_observed)status_range_speed=-(vx*tr.observed_exit_direction.x+
                                             vy*tr.observed_exit_direction.y);
    }
    if (tr.front_class_confirmed && !persistentVehicle(tr) &&
        tr.detection.motion_state != "STATIONARY") {
      const auto kf_direction = cx128s2_bev_perception::vehicle_surface::signedCorridorDirection(
          {x, y}, {vx, vy}, motion_speed_threshold_);
      status_observed = status_observed && memory.corridor_direction.consecutive >= 2 &&
          memory.corridor_direction.candidate == kf_direction &&
          kf_direction != cx128s2_bev_perception::vehicle_surface::CorridorDirection::UNKNOWN;
    }
    const auto update = cx128s2_bev_perception::updateVehicleStatus(
        tr.vehicle_status, tr.detection.motion_state, status_range_speed, stamp,
        vehicle_stopped_duration_sec_, motion_speed_threshold_, status_observed);
    if (tr.vehicle_status.value == previous_status) return;

    const double speed = std::hypot(outputVx(tr), outputVy(tr));
    RCLCPP_DEBUG(
        get_logger(),
        "[VEHICLE_STATUS] track=%d %s -> %s stationary_duration=%.2f speed=%.2f world_xy=(%.2f,%.2f) velocity=(%.2f,%.2f) range_speed=%.2f direction_source=%s%s",
        tr.id, previous_status.c_str(), tr.vehicle_status.value.c_str(),
        update.stationary_duration_sec, speed, x, y, vx, vy, range_speed,
        observed_exit ? "OBSERVED_FIFO_EXIT" : "WORLD_RANGE",
        update.timestamp_regressed ? " timestamp_reset" : "");
  }

  std::string radialMotion(const Track & tr) const {
    const double x = outputX(tr), y = outputY(tr), vx = outputVx(tr), vy = outputVy(tr);
    const double d = std::hypot(x, y);
    const double rv = d > 1e-3 ? (x * vx + y * vy) / d : 0.0;
    if (std::abs(rv) < motion_speed_threshold_) return "RADIAL_STATIONARY";
    return rv < 0.0 ? "MOVING_TOWARD_SENSOR" : "MOVING_AWAY_FROM_SENSOR";
  }

  cx128s2_bev_perception::msg::TrackedObject toMsg(const Track & tr) const {
    cx128s2_bev_perception::msg::TrackedObject obj;
    const double x = outputX(tr), y = outputY(tr), vx = outputVx(tr), vy = outputVy(tr);
    const double d = std::hypot(x, y);
    const double speed = std::hypot(vx, vy);
    const double rv = d > 1e-3 ? (x * vx + y * vy) / d : 0.0;
    obj.id = tr.id;
    obj.class_name = tr.detection.class_name;
    obj.lifecycle_state = tr.detection.lifecycle_state;
    obj.motion_state = tr.detection.motion_state;
    obj.vehicle_status = tr.vehicle_status.value;
    obj.radial_motion_state = radialMotion(tr);
    obj.decision_reason = tr.detection.decision_reason.empty() ? tr.feature.decision_reason
                                                               : tr.detection.decision_reason;
    obj.rejection_reason = tr.detection.rejection_reason;
    obj.position.x = x;
    obj.position.y = y;
    obj.position.z = tr.z;
    obj.velocity.x = vx;
    obj.velocity.y = vy;
    obj.velocity.z = 0.0;
    obj.dimensions.x = tr.feature.length;
    obj.dimensions.y = tr.feature.width;
    obj.dimensions.z = tr.feature.height;
    const auto & mem = tr.observation_memory;
    if (perception_mode_ != "crane_beam_work" && kalmanEnabled() &&
        !tr.shape_memory.committed && tr.detection.class_name == "VEHICLE" && tr.feature.obb.valid) {
      obj.dimensions.x = tr.feature.obb.length;
      obj.dimensions.y = tr.feature.obb.width;
    }
    if (perception_mode_ != "crane_beam_work" && kalmanEnabled() &&
        !tr.shape_memory.committed && mem.reliable && !persistentVehicle(tr)) {
      obj.dimensions.x = mem.vehicle_obb.valid ? mem.vehicle_obb.length : mem.reliable_length;
      obj.dimensions.y = mem.vehicle_obb.valid ? mem.vehicle_obb.width : mem.reliable_width;
      obj.dimensions.z = mem.reliable_height;
    }
    if (persistentVehicle(tr)) {
      obj.dimensions.x = mem.vehicle_obb.length;
      obj.dimensions.y = mem.vehicle_obb.width;
      obj.dimensions.z = mem.vehicle_height;
      obj.position.z = mem.vehicle_z_center;
    }
    if (mem.reliable && (tr.lost_frames > 0 ||
        mem.length_evidence == GeometryEvidence::LOWER_BOUND))
      obj.dimensions.x = std::max(obj.dimensions.x, mem.reliable_length);
    if (mem.reliable && (tr.lost_frames > 0 ||
        mem.width_evidence == GeometryEvidence::LOWER_BOUND))
      obj.dimensions.y = std::max(obj.dimensions.y, mem.reliable_width);
    if (mem.reliable && (tr.lost_frames > 0 ||
        mem.height_evidence == GeometryEvidence::LOWER_BOUND))
      obj.dimensions.z = std::max(obj.dimensions.z, mem.reliable_height);
    obj.distance = static_cast<float>(trackSensorRange(tr));
    obj.speed = static_cast<float>(speed);
    obj.radial_velocity = static_cast<float>(rv);
    obj.range_slope = static_cast<float>(tr.range_slope);
    obj.confidence = static_cast<float>(tr.detection.score);
    obj.age = tr.age;
    obj.hits = tr.hits;
    obj.lost_frames = tr.lost_frames;
    // Support counts describe this frame, not the last measured cluster.
    // Geometry and identity may remain in memory without current support.
    const auto observed = recording_track_features_.find(tr.id);
    const auto * feature = tr.lost_frames == 0 && observed != recording_track_features_.end() ?
        observed->second : nullptr;
    obj.point_count = feature ? feature->point_count : 0;
    obj.cell_count = feature ? feature->cell_count : 0;
    obj.ring_coverage = feature ? feature->ring_coverage : 0;
    return obj;
  }

