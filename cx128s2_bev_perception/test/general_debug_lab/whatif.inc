static void whatIf(Node&n,const fs::path&snapshot,const fs::path&out){
 stop(n);std::ifstream file(snapshot,std::ios::binary);if(!file)throw std::runtime_error("Missing detection snapshot; run Detailed Replay for this frame");auto bytes=readRecord(file,true);Reader r{bytes};std::string magic,binary,params;size_t frame;double sx,sy,score;std::map<int,Track> tracks;
 r(magic);r(binary);r(frame);r(sx);r(sy);r(params);r(score);r(tracks);
 if(r.pos!=bytes.size()||magic!="GeneralLabDetectionSnapshot1"||binary!=shaFile(fs::canonical("/proc/self/exe")))throw std::runtime_error("Snapshot schema/binary mismatch; rerun Detailed Replay");
 auto original=Json::parse(params);for(auto it=original.begin();it!=original.end();++it)if(it.key()!="vehicle_min_score"&&n.get_parameter(it.key()).value_to_string()!=it.value())throw std::runtime_error("Frozen snapshot parameter mismatch: "+it.key());
 n.sensor_origin_x_=sx;n.sensor_origin_y_=sy;const double candidate=n.vehicle_min_score_;Json result={{"frame",frame},{"scope","Frozen evaluateDetection call-state only; no association or KF advancement"},{"actual_tracking_success",nullptr},{"baseline_threshold",score},{"candidate_threshold",candidate},{"tracks",Json::array()}};
 for(const auto&kv:tracks){const auto frozen=kv.second;general_lab::active=true;general_lab::gates=Json::array();n.vehicle_min_score_=score;const auto baseline=n.evaluateDetection(frozen);auto bg=general_lab::gates;general_lab::gates=Json::array();n.vehicle_min_score_=candidate;const auto changed=n.evaluateDetection(frozen);
  auto decision=[](const auto&d){return Json{{"class",d.class_name},{"valid",d.is_valid},{"score",d.score},{"lifecycle",d.lifecycle_state},{"reason",d.decision_reason},{"reject",d.rejection_reason}};};
  result["tracks"].push_back({{"id",frozen.id},{"component",frozen.feature.id},{"baseline",decision(baseline)},{"candidate",decision(changed)},{"baseline_gates",bg},{"candidate_gates",general_lab::gates}});
 }
 n.vehicle_min_score_=candidate;saveJson(out.string()+"_whatif.json",result);
}
