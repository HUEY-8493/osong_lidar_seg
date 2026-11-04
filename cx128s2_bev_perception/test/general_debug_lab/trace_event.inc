   event["trace_schema"]=2;event["detail_evaluated"]=general_lab::active;
   if(general_lab::active){
    event["gates"]=general_lab::gates;
    for(auto &item:event["tracks"]){
     const int id=item.at("id");const auto found=std::find_if(n.tracks_.begin(),n.tracks_.end(),[&](const auto&t){return t.id==id;});if(found==n.tracks_.end())continue;
     const auto&tr=*found;const auto&f=tr.feature;const auto&mem=tr.observation_memory;const auto&v=tr.visibility;
     item["component_id"]=f.id;item["vehicle"]=tr.detection.class_name=="VEHICLE";item["public"]=n.physicalOutputAllowed(tr)&&tr.detection.is_valid&&tr.detection.lifecycle_state!="CANDIDATE"&&tr.detection.lifecycle_state!="REJECTED";
     item["score"]=tr.detection.score;item["decision_reason"]=tr.detection.decision_reason;
     item["observation"]={{"actual",tr.lost_frames==0},{"points",tr.lost_frames==0?f.point_count:0},{"last_seen_stamp",mem.last_seen_stamp},{"measurement_valid",mem.measured_center_valid},{"measurement",{mem.measured_x,mem.measured_y}},{"predicted",{mem.predicted_x,mem.predicted_y}},{"raw_centroid",{mem.raw_centroid_x,mem.raw_centroid_y}}};
     item["geometry"]={{"length",f.length},{"width",f.width},{"height",f.height},{"aabb",{f.min_x,f.max_x,f.min_y,f.max_y,f.min_z,f.max_z}},{"obb",{{"valid",f.obb.valid},{"x",f.obb.center.x},{"y",f.obb.center.y},{"yaw",f.obb.yaw},{"length",f.obb.length},{"width",f.obb.width}}},{"reliable",mem.reliable},{"reliable_aabb",{mem.last_reliable_box.min_x,mem.last_reliable_box.max_x,mem.last_reliable_box.min_y,mem.last_reliable_box.max_y,mem.last_reliable_box.min_z,mem.last_reliable_box.max_z}},{"reliable_size",{mem.reliable_length,mem.reliable_width,mem.reliable_height}},{"update",mem.geometry_update},{"confidence",mem.geometry_confidence}};
     item["kf"]={{"initialized",tr.kf.initialized_},{"state",tr.kf.x_},{"covariance",tr.kf.p_},{"innovation",tr.kf.innovation_},{"spatial_gate_range_m",n.gateForRange(f.range)},{"measurement_std",{mem.measurement_std_x,mem.measurement_std_y}}};
     item["visibility"]={{"reason",cx128s2_bev_perception::visibilityReasonName(v.reason)},{"range",v.sensor_distance},{"azimuth_deg",v.azimuth_deg},{"elevation_deg",v.elevation_deg},{"expected_rays",v.adaptive_expected_rays},{"supported_rays",v.adaptive_supported_rays},{"observable_samples",v.observable_samples},{"occluded_fraction",v.occluded_fraction},{"supported_fraction",v.supported_fraction},{"unknown_samples",v.unknown_samples},{"has_occluder",v.has_occluder},{"expected_surface_samples",mem.expected_visible_samples},{"visible_supported_samples",mem.visible_supported_samples}};
     item["memory"]={{"last_reliable_stamp",mem.last_reliable_stamp},{"last_observed_quality",int(mem.last_observed_quality)},{"observation_age_sec",mem.observation_age_sec},{"missed_time_sec",mem.missed_time_sec},{"fifo_phase",int(mem.fifo_phase)},{"fifo_reject_reason",mem.fifo_last_reject_reason},{"reacquire_result",mem.reacquire_result}};
    }
    if(!n.lab_detection_snapshots_.empty()){
     Writer snap;snap(std::string("GeneralLabDetectionSnapshot1"));snap(shaFile(exe));snap(count);snap(n.sensor_origin_x_);snap(n.sensor_origin_y_);
     Json params=Json::object();for(auto&key:n.list_parameters({},100).names)params[key]=n.get_parameter(key).value_to_string();snap(params.dump());snap(n.vehicle_min_score_);snap(n.lab_detection_snapshots_);
     std::ofstream file(prefix.string()+"_frame_"+std::to_string(count)+".snapshot",std::ios::binary);writeRecord(file,snap.bytes,true);
     event["snapshot_scope"]="evaluateDetection call-state only; not a complete association checkpoint";
    }
   }
   // Compact recorded history is available even outside Detailed Replay.
   // Feature values survive the callback; frame-local feature pointers do not.
   for (auto & item : event["tracks"]) {
     const int id=item.at("id");
     const auto found=std::find_if(n.tracks_.begin(),n.tracks_.end(),
         [&](const auto & tr){return tr.id==id;});
     if(found==n.tracks_.end())continue;
     const auto & tr=*found;const auto & f=tr.feature;const auto & mem=tr.observation_memory;
     const bool actual=tr.lost_frames==0;
     item["component_id"]=f.id;
     item["public"]=n.physicalOutputAllowed(tr)&&tr.detection.is_valid&&
         tr.detection.lifecycle_state!="CANDIDATE"&&tr.detection.lifecycle_state!="REJECTED";
     item["vehicle"]=tr.detection.class_name=="VEHICLE";
     item["score"]=tr.detection.score;
     item["vehicle_score_threshold"]=n.vehicle_min_score_;
     item["canonical_vehicle_lifecycle"]=n.dormantVehicle(tr)?"DORMANT":
         n.lostVehicle(tr)?"LOST":n.persistentVehicle(tr)?"ACTIVE":"UNCONFIRMED_BODY";
     item["observation"]["actual"]=actual;
     item["observation"]["points"]=actual?f.point_count:0;
     item["observation"]["current_measurement_valid"]=actual;
     if(actual) item["observation"]["current_measurement"]={
         mem.measured_center_valid?mem.measured_x:f.centroid_x,
         mem.measured_center_valid?mem.measured_y:f.centroid_y};
     else item["observation"]["current_measurement"]=nullptr;
     item["observation"]["measurement_source"]=actual?
         (mem.measured_center_valid?"applied_surface_measurement":"applied_feature_centroid"):"NONE";
     item["observation"]["prediction_valid"]=tr.age>1&&tr.kf.initialized_;
     item["observation"]["predicted"]=tr.age>1&&tr.kf.initialized_?
         Json::array({mem.predicted_x,mem.predicted_y}):Json::array();
     if(!item.contains("kf")) item["kf"]={{"initialized",tr.kf.initialized_},
         {"state",tr.kf.x_},{"covariance",tr.kf.p_},{"innovation",tr.kf.innovation_}};
     item["kf"]["innovation_current_valid"]=actual&&tr.age>1;
     if(!item.contains("geometry")) item["geometry"]={
         {"length",f.length},{"width",f.width},{"height",f.height},
         {"aabb",{f.min_x,f.max_x,f.min_y,f.max_y,f.min_z,f.max_z}},
         {"reliable",mem.reliable},
         {"reliable_size",{mem.reliable_length,mem.reliable_width,mem.reliable_height}}};
     item["current_component_type"]=actual?Json(f.component_type):Json(nullptr);
     item["current_component_reason"]=actual?Json(f.component_type_reason):Json(nullptr);
     if(general_lab::active) {
       if(lab_current_indices.count(id))item["current_indices"]=lab_current_indices.at(id);
       if(lab_current_components.count(id))item["current_component_support"]=lab_current_components.at(id);
       std::set<int> bounded; bool bounds_evaluated=false;
       for(const auto & g:general_lab::gates) if(g.value("track",-1)==id) {
         if(g.value("function",std::string{})=="front_component_bounds") {
           bounds_evaluated=true;if(g.value("pass",false))bounded.insert(g.value("cluster",-1));
         }
         if(g.value("function",std::string{})=="evaluateDetectionMetrics")
           item["vehicle_size_score"]=g.at("vehicle_size_score");
       }
       if(!actual&&bounds_evaluated)item["unmatched_bounded_components"]=bounded.size();
     }
   }
