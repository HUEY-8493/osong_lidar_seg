  bool measuredBodyReferenceSupported(const ObjectFeature & f,
                                      const std::vector<PointXYZIRT> & points) {
    auto * bev=dynamic_cast<BevConnectedComponentClusterer *>(clusterer_.get());
    if (!bev || !f.vehicle_height_supported) return false;
    // A long low platform plus a person/pole can pass a transient size gate.
    // It does not establish a measured body-center reference. Require the
    // same broad connected height evidence used for upper-attachment validation.
    std::vector<int> high_support;
    for (int i:f.indices) if (points[i].z>=f.min_z+vehicle_min_height_) high_support.push_back(i);
    const auto parts=bev->observationLayerComponents(points,high_support);
    return std::any_of(parts.begin(),parts.end(),[&](const auto & part) {
      return part.point_count>=min_cluster_points_near_ &&
          std::max(part.max_x-part.min_x,part.max_y-part.min_y)>=vehicle_min_width_;
    });
  }

  void separateVehicleUpperAttachments(const std::vector<PointXYZIRT> & points,
                                      std::vector<ClusterResult> & clusters) {
    if (perception_mode_ == "crane_beam_work") return;
    auto * bev=dynamic_cast<BevConnectedComponentClusterer *>(clusterer_.get());
    if (!bev) return;
    const double step=std::max(0.05,cluster_z_layer_resolution_);
    int next_id=0;for(const auto&c:clusters)next_id=std::max(next_id,c.id+1);
    std::vector<ClusterResult> attachments;
    for(auto&c:clusters){
      if(c.indices.size()<static_cast<size_t>(min_cluster_points_near_) ||
         c.length<2.0 || c.length>vehicle_max_length_ ||
         c.width<vehicle_min_width_ || c.width>vehicle_max_width_)continue;
      double low=std::numeric_limits<double>::max(),high=std::numeric_limits<double>::lowest();
      for(int i:c.indices){low=std::min(low,double(points[i].z));high=std::max(high,double(points[i].z));}
      if(high-low<=vehicle_min_height_)continue;
      if(high-low<=vehicle_max_height_) {
        // Below the old height ceiling, only a measured narrow vertical top
        // warrants a connectivity query. A broad roof/cargo remains intact;
        // full overlap without separable support is deliberately unresolved.
        const double zcut=low+.75*(high-low);
        double x0=std::numeric_limits<double>::max(),y0=x0,z0=x0;
        double x1=-x0,y1=-x0;
        for(int i:c.indices){const auto&p=points[i];if(p.z<zcut)continue;
          x0=std::min(x0,double(p.x));x1=std::max(x1,double(p.x));
          y0=std::min(y0,double(p.y));y1=std::max(y1,double(p.y));z0=std::min(z0,double(p.z));}
        const double extent=std::max(x1-x0,y1-y0);
        if(!std::isfinite(extent) || extent>=vehicle_min_width_ || high-z0<=extent)continue;
      }
      // Do not search lower cuts through a primarily upright structure until
      // its residual happens to satisfy vehicle dimensions. Initial separation
      // requires a horizontal vehicle footprint in this measured component.
      if(c.length<high-low)continue;
      // Search only within the existing valid body-height interval. Height
      // alone never excludes a point: every separated component must also be
      // narrow in XY, vertically elongated and connected in the measured grid.
      const int first=static_cast<int>(std::ceil((low+vehicle_min_height_)/step));
      const int last=static_cast<int>(std::floor((low+vehicle_max_height_)/step));
      // Rank measured horizontal-support transitions before the expensive
      // connectivity query. This proposes a cut; the same component/body
      // validation below still decides whether any separation is permitted.
      std::unordered_map<int64_t,uint64_t> columns;
      columns.reserve(c.cell_keys.size());
      bool complete=true;
      for(int i:c.indices){const int z=static_cast<int>(std::floor(points[i].z/step));
        if(z<0 || z>=64){complete=false;break;}
        columns[bevKey(points[i])] |= uint64_t{1}<<z;
      }
      if(!complete || first<=0 || last>=64)continue;
      std::array<int,64> support{};
      for(const auto&kv:columns){uint64_t mask=kv.second;while(mask){const int z=__builtin_ctzll(mask);++support[z];mask&=mask-1;}}
      std::vector<int> cuts;for(int bin=first;bin<=last;++bin)cuts.push_back(bin);
      std::stable_sort(cuts.begin(),cuts.end(),[&](int a,int b){
        return double(support[a-1])/std::max(1,support[a]) >
               double(support[b-1])/std::max(1,support[b]);
      });
      for(int bin:cuts){
        const double cut=bin*step;std::vector<int> upper;
        for(int i:c.indices)if(points[i].z>cut)upper.push_back(i);
        auto parts=bev->observationLayerComponents(points,upper);
        std::unordered_set<int> excluded;std::vector<ClusterResult> separated;
        for(auto&part:parts){
          double zmin=std::numeric_limits<double>::max(),zmax=std::numeric_limits<double>::lowest();
          for(int i:part.indices){zmin=std::min(zmin,double(points[i].z));zmax=std::max(zmax,double(points[i].z));}
          const double horizontal=std::max(part.max_x-part.min_x,part.max_y-part.min_y);
          if(horizontal>=vehicle_min_width_ ||
             zmax-zmin<=std::max(step,horizontal))continue;
          excluded.insert(part.indices.begin(),part.indices.end());separated.push_back(std::move(part));
        }
        if(excluded.empty())continue;
        double ax0=std::numeric_limits<double>::max(),ay0=ax0;
        double ax1=std::numeric_limits<double>::lowest(),ay1=ax1;
        for(int i:excluded){ax0=std::min(ax0,double(points[i].x));ax1=std::max(ax1,double(points[i].x));
          ay0=std::min(ay0,double(points[i].y));ay1=std::max(ay1,double(points[i].y));}
        // Many narrow sampled columns can together be one broad high load.
        // Do not relabel such aggregate body-width support as appendages.
        if(std::min(ax1-ax0,ay1-ay0)>=.8*c.width)continue;
        std::vector<int> body;double body_high=std::numeric_limits<double>::lowest();
        for(int i:c.indices)if(!excluded.count(i)){body.push_back(i);body_high=std::max(body_high,double(points[i].z));}
        if(body.size()<static_cast<size_t>(min_cluster_points_near_) ||
           body_high-low<vehicle_min_height_ || body_high-low>vehicle_max_height_)continue;
        // A clipped wire stub cannot provide the minimum vehicle height.
        // Verify height on current body points outside the attachment columns,
        // using existing spatial uncertainty, height and support limits. This
        // is validation only: all source indices remain in the two partitions.
        const double mx=std::max(bev_x_res_,raw_vehicle_box_margin_m_);
        const double my=std::max(bev_y_res_,raw_vehicle_box_margin_m_);
        std::vector<int> independent,high_support;uint64_t independent_layers=0;
        double independent_high=std::numeric_limits<double>::lowest();
        for(int i:body){const auto&p=points[i];
          const bool in_column=std::any_of(separated.begin(),separated.end(),[&](const auto&part){
            return p.x>=part.min_x-mx && p.x<=part.max_x+mx &&
                   p.y>=part.min_y-my && p.y<=part.max_y+my;
          });
          if(in_column)continue;
          independent.push_back(i);independent_high=std::max(independent_high,double(p.z));
          independent_layers|=uint64_t{1}<<static_cast<int>(std::floor(p.z/step));
          if(p.z>=low+vehicle_min_height_)high_support.push_back(i);
        }
        if(independent_high-low<vehicle_min_height_ ||
           !bev->connectedHeightSupported(points,independent,independent_layers,vehicle_min_height_))continue;
        const auto upper_body=bev->observationLayerComponents(points,high_support);
        const bool broad_height_support=std::any_of(upper_body.begin(),upper_body.end(),[&](const auto&part){
          return part.point_count>=min_cluster_points_near_ &&
              std::max(part.max_x-part.min_x,part.max_y-part.min_y)>=vehicle_min_width_;
        });
        if(!broad_height_support)continue;
        auto measured=makeFemCluster(points,body,c.id,"vehicle_body_separated_upper_attachment","","",0.0);
        // Removing a thin top cannot turn a still vertically elongated base
        // (e.g. an upright support) into a road-vehicle body. Reuse the same
        // horizontal-versus-vertical geometry used for attachment separation.
        if(measured.length<body_high-low || measured.length<2.0 || measured.length>vehicle_max_length_ ||
           measured.width<vehicle_min_width_ || measured.width>vehicle_max_width_)continue;
        c=std::move(measured);
        for(auto&part:separated){part.id=next_id++;part.method="observed_vertical_attachment";attachments.push_back(std::move(part));}
        break;
      }
    }
    // Both partitions retain original measured indices. The attachment stays
    // visible to diagnostics/classification and is never a vehicle measurement.
    clusters.insert(clusters.end(),std::make_move_iterator(attachments.begin()),std::make_move_iterator(attachments.end()));
  }

  std::vector<ObjectFeature> buildFeaturesFromClusterResults(
      const std::vector<PointXYZIRT> & points,
      const std::vector<ClusterResult> & clusters,
      const std::vector<int> & foreground_idx) {
    // Every ClusterResult is built from foreground_idx. Retain the argument for the public
    // feature-builder contract, but avoid rebuilding a frame-wide hash set just to obtain 1.0.
    (void)foreground_idx;
    std::vector<ObjectFeature> out;
    out.reserve(clusters.size());
    for (const auto & cluster : clusters) {
      const auto & c = cluster.indices;
      ObjectFeature f;
      f.id = cluster.id;
      f.indices = c;
      f.point_count = static_cast<int>(c.size());
      f.shape_node_count = cluster.shape_node_count;
      f.shape_edge_count = cluster.shape_edge_count;
      f.shape_graph_score = cluster.shape_graph_score;
      f.shape_top_linearity = cluster.shape_top_linearity;
      f.shape_side_linearity = cluster.shape_side_linearity;
      f.shape_front_linearity = cluster.shape_front_linearity;
      f.shape_area_top = cluster.shape_area_top;
      f.object_memory_id = cluster.fem_memory_id;
      f.object_memory_bank_match_score = cluster.fem_ownership_score;
      f.fem_ownership_score = cluster.fem_ownership_score;
      if (cluster.fem_cluster_role == "TRUCK_BODY") {
        f.fem_owned_point_count = f.point_count;
      } else if (cluster.fem_cluster_role == "TRUCK_CARGO_CANDIDATE") {
        f.fem_cargo_candidate_point_count = f.point_count;
        f.cargo_score = cluster.fem_ownership_score;
      } else if (cluster.fem_cluster_role == "ATTACHED_UNKNOWN_OBJECT") {
        f.fem_attached_unknown_point_count = f.point_count;
      } else if (cluster.fem_cluster_role == "OUTSIDE_FEM_MEMORY") {
        f.fem_outside_point_count = f.point_count;
      }
      f.shape_nodes = cluster.shape_nodes;
      f.shape_edges = cluster.shape_edges;
      f.min_x = f.min_y = f.min_z = std::numeric_limits<double>::max();
      f.max_x = f.max_y = f.max_z = std::numeric_limits<double>::lowest();
      // CX128S2 publishes rings [0, 127]. Keep a rare-value fallback so the coverage
      // remains exact if a different input supplies a larger uint16 ring value.
      std::bitset<128> rings;
      uint64_t height_layers = 0;
      bool height_layers_complete = true;
      const bool check_height_layers = perception_mode_ != "crane_beam_work" &&
          sensorRange(cluster.centroid_x, cluster.centroid_y) < near_range_max_;
      std::set<uint16_t> out_of_range_rings;
      std::unordered_set<int64_t> cells;
      const bool need_cell_fallback = cluster.cell_count <= 0;
      if (need_cell_fallback) cells.reserve(c.size());
      double bg_sim_sum = 0.0;
      for (int idx : c) {
        const auto & p = points[idx];
        f.centroid_x += p.x;
        f.centroid_y += p.y;
        f.centroid_z += p.z;
        f.mean_intensity += p.intensity;
        f.min_x = std::min(f.min_x, static_cast<double>(p.x));
        f.max_x = std::max(f.max_x, static_cast<double>(p.x));
        f.min_y = std::min(f.min_y, static_cast<double>(p.y));
        f.max_y = std::max(f.max_y, static_cast<double>(p.y));
        f.min_z = std::min(f.min_z, static_cast<double>(p.z));
        f.max_z = std::max(f.max_z, static_cast<double>(p.z));
        if (p.ring < rings.size()) rings.set(p.ring);
        else out_of_range_rings.insert(p.ring);
        if (check_height_layers) {
          const int height_bin = static_cast<int>(std::floor(
              p.z / std::max(0.05, cluster_z_layer_resolution_)));
          if (height_bin >= 0 && height_bin < 64)
            height_layers |= uint64_t{1} << height_bin;
          else height_layers_complete = false;
        }
        if (need_cell_fallback) cells.insert(bevKey(p));
        bg_sim_sum += last_point_bg_similarity_[idx];
      }
      const double n = std::max(1, f.point_count);
      f.centroid_x /= n;
      f.centroid_y /= n;
      f.centroid_z /= n;
      f.mean_intensity /= n;
      std::vector<cx128s2_bev_perception::vehicle_surface::Point2> xy;
      xy.reserve(c.size());
      for (int idx : c) xy.push_back({points[idx].x, points[idx].y});
      f.obb = perception_mode_ == "crane_beam_work" ?
          cx128s2_bev_perception::vehicle_surface::estimateObb(xy) :
          cx128s2_bev_perception::vehicle_surface::fixedYawObb(xy, vehicle_fixed_yaw_rad_);
      f.length = std::max(0.05, f.max_x - f.min_x);
      f.width = std::max(0.05, f.max_y - f.min_y);
      if (perception_mode_ != "crane_beam_work" && f.obb.valid) {
        f.length = f.obb.length; f.width = f.obb.width;
      } else if (f.width > f.length) std::swap(f.width, f.length);
      f.height = std::max(0.05, f.max_z - f.min_z);
      f.cell_count = cluster.cell_count > 0 ? cluster.cell_count : static_cast<int>(cells.size());
      f.density = f.point_count / std::max(0.1, f.length * f.width);
      f.ring_coverage = static_cast<int>(rings.count() + out_of_range_rings.size());
      f.range = sensorRange(f.centroid_x, f.centroid_y);
      // Compatibility field: now represents foreground ratio, not motion dynamics.
      f.dynamic_score = 1.0;
      f.background_similarity = bg_sim_sum / n;
      f.inside_roi = use_roi_policy_ ? (pointPolicy(f.centroid_x, f.centroid_y) == RoiDecision::KEEP)
                                     : insideRoi(f.centroid_x, f.centroid_y);
      f.component_type = classifyComponentType(
          f, f.component_type_confidence, f.component_type_reason);
      if (check_height_layers && f.range < near_range_max_ &&
          f.component_type == "vehicle_like" && height_layers_complete) {
        auto * bev = dynamic_cast<BevConnectedComponentClusterer *>(clusterer_.get());
        if (bev) f.vehicle_height_supported = bev->connectedHeightSupported(
            points, f.indices, height_layers, vehicle_min_height_);
      }
      f.decision_reason = cluster.method.empty() ? ("BACKGROUND_DIFF:" + foreground_classifier_mode_) :
          ("BACKGROUND_DIFF:" + foreground_classifier_mode_ + "+CLUSTER:" + cluster.method);
      f.decision_reason += "+component_type_" + f.component_type;
      if (!cluster.fem_memory_id.empty()) {
        f.decision_reason += "+object_memory_bank_prior+fem_ownership_split";
        if (cluster.fem_cluster_role == "TRUCK_CARGO_CANDIDATE") {
          f.cargo_loaded_candidate = true;
          f.rejection_reason.clear();
          f.decision_reason += "+cargo_loaded_candidate";
        } else if (cluster.fem_cluster_role == "ATTACHED_UNKNOWN_OBJECT") {
          f.decision_reason += "+attached_unknown_split";
        }
      }
      if (enable_static_structure_filter_ && isStaticStructure(f)) {
        f.rejection_reason = "STATIC_STRUCTURE";
      }
      if (roi_enabled_ && roi_apply_stage_ == "candidate" && !f.inside_roi) {
        f.rejection_reason = "OUTSIDE_ROI";
      }
      if (f.rejection_reason.empty()) out.push_back(f);
    }
    last_candidate_count_ = static_cast<int>(out.size());
    return out;
  }

  bool isStaticStructure(const ObjectFeature & f) const {
    const double elongation = f.length / std::max(0.05, f.width);
    return (f.length >= wall_length_min_ && f.width <= wall_width_max_ &&
            elongation >= wall_elongation_ratio_min_);
  }

  void refineVehicleFeaturesWithRawBox(const std::vector<PointXYZIRT> & points,
                                       const std::vector<int> & original_owner,
                                       std::vector<ObjectFeature> & features,
                                       const std::vector<int> * roi_source_indices = nullptr) {
    raw_vehicle_box_refine_ms_ = 0.0;
    raw_vehicle_box_stats_ = {};
    raw_vehicle_box_queries_ = raw_vehicle_box_used_ = 0;
    if (!raw_vehicle_box_refinement_enabled_ || perception_mode_ == "crane_beam_work" || features.empty()) return;
    const auto begin = std::chrono::steady_clock::now();
    std::vector<size_t> selected;
    selected.reserve(features.size());
    for (size_t i = 0; i < features.size(); ++i) {
      auto & f = features[i];
      f.raw_box_reason = "INELIGIBLE_SEED";
      const int min_points = f.range < near_range_max_ ? min_cluster_points_near_ :
          (f.range < mid_range_max_ ? min_cluster_points_mid_ : min_cluster_points_long_);
      const int min_cells = f.range < near_range_max_ ? min_cluster_cells_near_ :
          (f.range < mid_range_max_ ? min_cluster_cells_mid_ : min_cluster_cells_long_);
      // Raw points cannot create observations or rescue a tiny rejected seed.
      if (!f.obb.valid || f.point_count < min_points || f.cell_count < min_cells ||
          f.obb.length < 2.0 || f.obb.width < 0.5 * vehicle_min_width_ ||
          f.height < 0.5 * vehicle_min_height_ || f.obb.length > vehicle_max_length_ ||
          f.obb.width > vehicle_max_width_ || f.height > vehicle_max_height_ ||
          !std::isfinite(f.obb.yaw) || !std::isfinite(f.centroid_x) || !std::isfinite(f.centroid_y) ||
          !std::isfinite(f.min_z) || !std::isfinite(f.max_z)) continue;
      selected.push_back(i);
      f.raw_box_reason = "OBJECT_LIMIT";
    }
    if (selected.size() > static_cast<size_t>(raw_vehicle_box_max_objects_)) {
      std::partial_sort(selected.begin(), selected.begin() + raw_vehicle_box_max_objects_, selected.end(),
          [&](size_t a, size_t b) { return features[a].point_count > features[b].point_count; });
      selected.resize(raw_vehicle_box_max_objects_);
    }
    std::vector<cx128s2_bev_perception::raw_box::Query> queries;
    queries.reserve(selected.size());
    for (size_t i : selected) {
      const auto & f = features[i];
      queries.push_back({f.id, f.obb.center.x, f.obb.center.y,
          std::cos(f.obb.yaw), std::sin(f.obb.yaw), f.obb.length * 0.5 + raw_vehicle_box_margin_m_,
          f.obb.width * 0.5 + raw_vehicle_box_margin_m_, f.min_z - raw_vehicle_box_margin_m_,
          f.max_z + raw_vehicle_box_margin_m_});
    }
    raw_vehicle_box_queries_ = static_cast<int>(queries.size());
    raw_vehicle_box_stats_ = cx128s2_bev_perception::raw_box::collect(points, original_owner, queries,
        [&](const PointXYZIRT & p) {
          // Production reuses the raw ROI indices computed before background subtraction.
          // A standalone call without those indices still applies the normal crop/ROI policy.
          return roi_source_indices || (passValidCrop(p) && (!use_roi_policy_ || pointPolicy(p.x, p.y) == RoiDecision::KEEP) &&
              (use_roi_policy_ || !roi_enabled_ || insideRoi(p.x, p.y)));
        }, raw_vehicle_box_workspace_, roi_source_indices);
    for (size_t qi = 0; qi < queries.size(); ++qi) {
      auto & f = features[selected[qi]];
      const auto & o = raw_vehicle_box_workspace_.observations[qi];
      f.raw_box_point_count = o.points;
      f.raw_box_foreground_count = o.foreground_points;
      if (o.foreground_points < f.point_count || o.points <= o.foreground_points) {
        f.raw_box_reason = o.points <= o.foreground_points ? "NO_ADDITIONAL_RAW_POINTS" : "SEED_SUPPORT_LOST";
        continue;
      }
      const double fraction = static_cast<double>(o.foreground_points) / o.points;
      if (fraction < raw_vehicle_box_min_foreground_ratio_) { f.raw_box_reason = "BACKGROUND_DOMINATED"; continue; }
      const double x = o.sum_x / o.points, y = o.sum_y / o.points, z = o.sum_z / o.points;
      if (std::hypot(x - f.centroid_x, y - f.centroid_y) > raw_vehicle_box_max_centroid_shift_m_) {
        f.raw_box_reason = "CENTROID_SHIFT"; continue;
      }
      const double length = o.max_local_x - o.min_local_x, width = o.max_local_y - o.min_local_y;
      const double height = o.max_z - o.min_z;
      if (length < 2.0 || width < 0.5 * vehicle_min_width_ || height < 0.5 * vehicle_min_height_ ||
          length > vehicle_max_length_ || width > vehicle_max_width_ || height > vehicle_max_height_) {
        f.raw_box_reason = "INVALID_RAW_GEOMETRY"; continue;
      }
      const auto center = cx128s2_bev_perception::vehicle_surface::toWorld(
          {(o.min_local_x + o.max_local_x) * 0.5, (o.min_local_y + o.max_local_y) * 0.5},
          f.obb.center, f.obb.yaw);
      f.centroid_x = x; f.centroid_y = y; f.centroid_z = z;
      f.min_x = o.min_x; f.max_x = o.max_x; f.min_y = o.min_y; f.max_y = o.max_y;
      f.min_z = o.min_z; f.max_z = o.max_z;
      f.obb.center = center; f.obb.length = length; f.obb.width = width;
      f.length = std::max(o.max_x - o.min_x, o.max_y - o.min_y);
      f.width = std::min(o.max_x - o.min_x, o.max_y - o.min_y);
      f.height = height; f.range = sensorRange(x, y);
      f.density = f.point_count / std::max(0.1, f.length * f.width);
      f.component_type = classifyComponentType(f, f.component_type_confidence, f.component_type_reason);
      f.raw_box_used = true; f.raw_box_reason = "ACCEPTED";
      ++raw_vehicle_box_used_;
      // Original foreground indices/counts, persistence, quality and ID rules remain independent.
      RCLCPP_DEBUG(get_logger(), "[RAW_VEHICLE_BOX] cluster=%d foreground=%d raw=%d ratio=%.3f measured=(%.3f,%.3f,%.3f)",
          f.id, f.point_count, o.points, fraction, x, y, z);
    }
    raw_vehicle_box_refine_ms_ = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - begin).count();
  }

