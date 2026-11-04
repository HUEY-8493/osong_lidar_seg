// Exercise the production node's association, filter, lifecycle and publication paths.
#define main perception_node_main
#include "../src/cx128s2_bev_perception_node.cpp"
#undef main
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <iostream>

struct TrackingRegressionAccess {
  static void phase5DeckCheck(Cx128s2BevPerception & n) {
    // A measured low deck continues across a sampled seam over vehicle width.
    // Side-by-side bodies, longitudinally separate bodies and vertical-only
    // contacts are negative controls, not just copies of implementation gates.
    std::vector<PointXYZIRT> points;
    auto make=[&](double x0,double x1,double y0,double y1,double z)->ObjectFeature{
      ObjectFeature f;f.min_x=x0;f.max_x=x1;f.min_y=y0;f.max_y=y1;f.min_z=.1;f.max_z=3;
      f.length=x1-x0;f.width=y1-y0;f.height=2.9;f.obb.valid=true;
      for(double x=x0;x<=x1+.001;x+=.05)for(double y=y0;y<=y1+.001;y+=.05){
        PointXYZIRT p{};p.x=x;p.y=y;p.z=z;f.indices.push_back(points.size());points.push_back(p);}
      return f;
    };
    auto body=make(-12,-2,-1.4,1.4,1.5),part=make(-15.1,-12.3,-1.4,1.4,1.5);
    assert(n.measuredDeckConnection(body,part,points));
    for(int i:part.indices)points[i].x-=1;
    part.min_x-=1;part.max_x-=1;
    assert(!n.measuredDeckConnection(body,part,points));
    for(int i:part.indices){points[i].x+=1;points[i].z+=1;}part.min_x+=1;part.max_x+=1;
    assert(!n.measuredDeckConnection(body,part,points));
    for(int i:part.indices){points[i].z-=1;points[i].y+=4;}part.min_y+=4;part.max_y+=4;
    assert(!n.measuredDeckConnection(body,part,points));
    std::cout<<"Phase5 measured deck continuity: connected surface positive; separated/vertical/side-by-side negatives PASS\n";
  }

  static void phase5LowerAttachmentCheck(Cx128s2BevPerception & n) {
    auto old_clusterer=std::move(n.clusterer_);n.clusterer_=n.createClusterer();
    // Controlled measured points, not bag evidence. A real body keeps its
    // support while a thin attached column descends below the old 4.5m ceiling.
    for(double hook_top:{5.5,4.2,3.4}){
      std::vector<PointXYZIRT> points;
      for(int ix=0;ix<=80;++ix)for(int iy=0;iy<=40;++iy)for(int iz=0;iz<=20;++iz){
        PointXYZIRT p{};p.x=-12+ix*.1;p.y=-1+iy*.05;p.z=.1+iz*.1;points.push_back(p);}
      const size_t body_count=points.size();
      for(double z=2.;z<=hook_top;z+=.05)for(int x=0;x<3;++x){
        PointXYZIRT p{};p.x=-10+x*.05;p.y=0;p.z=z;points.push_back(p);}
      std::vector<int>ids(points.size());std::iota(ids.begin(),ids.end(),0);
      std::vector<ClusterResult> clusters{n.makeFemCluster(points,ids,71,"test","","",0)};
      n.separateVehicleUpperAttachments(points,clusters);
      assert(clusters.size()>1);
      std::vector<int>seen(points.size());for(const auto&c:clusters)for(int i:c.indices)++seen.at(i);
      assert(std::all_of(seen.begin(),seen.end(),[](int x){return x==1;}));
      std::vector<bool>retained(points.size());for(int i:clusters[0].indices)retained[i]=true;
      for(size_t i=0;i<body_count;++i)assert(retained[i]);
    }
    n.clusterer_=std::move(old_clusterer);
    std::cout<<"Phase5 descending thin attachment 5.5/4.2/3.4m: body and exact point partition retained PASS (synthetic)\n";
  }

  static void phase5PartReferenceCheck(Cx128s2BevPerception & n) {
    Track tr;tr.multi_component_body=true;tr.position_from_body_box=true;
    tr.observation_memory.vehicle_confirmed=true;
    tr.observation_memory.vehicle_state=Lifecycle::ACTIVE;
    tr.observation_memory.vehicle_obb.valid=true;tr.observation_memory.vehicle_obb.length=15;
    tr.observation_memory.vehicle_obb.width=3;tr.x=-9;tr.y=0;tr.kf.initialize(-9,0,.16,.5);
    tr.component_references={{{-5.8,0,3.4,2.8}},{{1.6,0,11.8,3}}};
    for(const auto&r:tr.component_references)if(r[2]<=n.vehicle_max_width_)for(double movement:{0.,.1,.2}){
      ObjectFeature f;f.range=12;f.obb.valid=true;f.obb.length=r[2];f.obb.width=r[3];
      f.obb.center={-9+r[0]+movement,r[1]};bool aligned=false;
      const auto m=n.correctedVehicleCenter(tr,999,999,1,&f,&aligned);
      assert(aligned && std::abs(m.x-(-9+movement))<1e-8 && std::abs(m.y)<1e-8);
    }
    // The measured joined rear face remains visible when the front is clipped.
    {
      std::vector<PointXYZIRT>points;ObjectFeature end;end.range=12;end.obb.valid=true;
      end.obb.length=12.3;end.obb.width=3;end.obb.center={-10.15,0};
      end.min_x=-16.3;end.max_x=-4;end.min_z=.1;
      for(int i=0;i<=60;++i){PointXYZIRT p{};p.x=-16.3;p.y=-1.5+i*.05;p.z=1.5;
        end.indices.push_back(points.size());points.push_back(p);}
      bool valid=false;const auto m=n.correctedVehicleCenter(tr,0,0,1,&end,&valid,&points);
      assert(valid && std::abs(m.x+8.8)<1e-5);
      // A laterally thin hook shifts the raw bound by 20cm; the measured
      // broad body end must remain the same real plane.
      for(int i=0;i<30;++i){PointXYZIRT p{};p.x=-16.5;p.y=0;p.z=1.2+i*.03;
        end.indices.push_back(points.size());points.push_back(p);}
      end.min_x=-16.5;valid=false;
      const auto hooked=n.correctedVehicleCenter(tr,0,0,1,&end,&valid,&points);
      assert(valid && std::abs(hooked.x-m.x)<1e-8);
      for(auto&p:points)p.y=0;
      valid=true;n.correctedVehicleCenter(tr,0,0,1,&end,&valid,&points);assert(!valid);
    }
    // Identical part signatures at nearby positions remain ambiguous.
    tr.component_references.push_back(tr.component_references[0]);
    ObjectFeature f;f.range=12;f.obb.valid=true;f.obb.length=3.4;f.obb.width=2.8;f.obb.center={-14.8,0};bool aligned=true;
    n.correctedVehicleCenter(tr,f.obb.center.x,0,1,&f,&aligned);assert(!aligned);
    std::cout<<"Phase5 unique short-part reference / joined measured end, stationary/moving, narrow hook outlier, narrow-point and ambiguity rejection PASS\n";
  }
  static void phase5ExitDirectionCheck(Cx128s2BevPerception & n) {
    const auto old_filter=n.tracking_filter_mode_;n.tracking_filter_mode_="legacy";
    Track tr;tr.id=818;tr.multi_component_body=true;tr.observed_exit_direction_valid=true;
    tr.observed_exit_direction={1,0};tr.observation_memory.vehicle_confirmed=true;
    tr.observation_memory.vehicle_state=Lifecycle::ACTIVE;
    tr.observation_memory.fifo_phase=cx128s2_bev_perception::vehicle_surface::FifoPhase::EXITING;
    tr.observation_memory.vehicle_quality=VehicleObservationQuality::PARTIAL;
    tr.observation_memory.association_source="EXTENT_ALIGNED";
    tr.detection.class_name="VEHICLE";tr.detection.is_valid=true;tr.detection.motion_state="MOVING";
    tr.x=-2;tr.y=0;tr.vx=1;tr.vy=0;n.evaluateVehicleStatus(tr,1);assert(tr.vehicle_status.value=="exiting");
    tr.x=2;tr.observation_memory.corridor_direction.stable=cx128s2_bev_perception::vehicle_surface::CorridorDirection::ENTRY;
    n.evaluateVehicleStatus(tr,2);assert(tr.vehicle_status.value=="exiting");
    tr.vx=-1;n.evaluateVehicleStatus(tr,3);assert(tr.vehicle_status.value=="entering");
    tr.vx=1;tr.lost_frames=1;n.evaluateVehicleStatus(tr,4);assert(tr.vehicle_status.value=="entering");
    tr.lost_frames=0;tr.vx=0;tr.detection.motion_state="STATIONARY";
    n.evaluateVehicleStatus(tr,5);assert(tr.vehicle_status.value=="stopping");
    n.evaluateVehicleStatus(tr,5+n.vehicle_stopped_duration_sec_+.01);assert(tr.vehicle_status.value=="stopped");
    tr.vx=1;tr.detection.motion_state="MOVING";
    n.evaluateVehicleStatus(tr,6+n.vehicle_stopped_duration_sec_);assert(tr.vehicle_status.value=="exiting");
    n.tracking_filter_mode_=old_filter;
    std::cout<<"Phase5 same physical EXIT across origin, real reverse ENTRY, missing observation hold, stop/restart with same ID PASS\n";
  }
  static void phase5GeometryInterferenceCheck(Cx128s2BevPerception & n) {
    const double old_length=n.vehicle_max_length_,old_height=n.vehicle_min_height_,old_width=n.vehicle_min_width_;
    n.vehicle_max_length_=20;n.vehicle_min_height_=1.8;n.vehicle_min_width_=1.5; // Production YAML bounds.
    const auto old_filter=n.tracking_filter_mode_;n.tracking_filter_mode_="kalman";
    const double old_sx=n.sensor_origin_x_,old_sy=n.sensor_origin_y_;n.sensor_origin_x_=0;n.sensor_origin_y_=3;
    for(bool broad_extension:{false,true}){
      Track tr;tr.multi_component_body=true;tr.position_from_body_box=true;
      tr.component_references={{{-5.8,0,3.4,2.8}}};tr.x=-9;tr.y=0;tr.kf.initialize(-9,0,.16,.5);
      auto&m=tr.observation_memory;m.vehicle_confirmed=true;m.vehicle_state=Lifecycle::ACTIVE;
      m.vehicle_obb.valid=true;m.vehicle_obb.center={-9,0};m.vehicle_obb.length=15;m.vehicle_obb.width=2.8;m.vehicle_obb.yaw=0;
      m.vehicle_height=2;m.vehicle_z_center=1.1;m.reliable=true;m.vehicle_quality=VehicleObservationQuality::FULL;
      m.last_reliable_box={-16.5,-1.5,-1.4,1.4,.1,2.1};m.association_source="EXTENT_ALIGNED";
      m.measured_x=-9;m.measured_y=0;tr.visibility.adaptive_expected_rays=100;tr.visibility.adaptive_supported_rays=100;
      std::vector<PointXYZIRT>points;const double left=broad_extension?-18.5:-16.5;
      for(double x=left;x<=-1.49;x+=.1)for(double y=-1.4;y<=1.401;y+=.05)for(double z=.1;z<=2.101;z+=.1){
        PointXYZIRT p{};p.x=x;p.y=y;p.z=z;points.push_back(p);}
      if(!broad_extension)for(int i=0;i<40;++i){PointXYZIRT p{};p.x=-18.5;p.y=0;p.z=1+i*.025;points.push_back(p);}
      ObjectFeature f;f.obb.valid=true;f.obb.center={-10,0};f.obb.length=f.length=17;
      f.obb.width=f.width=2.8;f.height=2;f.min_x=-18.5;f.max_x=-1.5;f.min_y=-1.4;f.max_y=1.4;f.min_z=.1;f.max_z=2.1;
      f.range=10;f.point_count=points.size();f.cell_count=100;f.vehicle_height_supported=true;
      f.component_type="vehicle_like";f.inside_roi=true;f.centroid_x=-9;f.centroid_y=0;f.centroid_z=1.1;
      f.indices.resize(points.size());std::iota(f.indices.begin(),f.indices.end(),0);
      const char*failure=n.vehicleObservationFailure(tr,f,points);
      if(broad_extension)assert(!failure);
      else assert(failure && std::string(failure)=="UNSUPPORTED_BODY_EXTENT");
      for(int frame=0;frame<4;++frame)n.updateVehicleSurface(tr,f,points);
      if(broad_extension){assert(std::abs(m.vehicle_obb.length-17)<1e-6);
        assert(std::abs(tr.kf.x()+tr.component_references[0][0]+14.8)<1e-6);}
      else {assert(std::abs(m.vehicle_obb.length-15)<1e-6 && std::abs(tr.kf.x()+9)<1e-6 && m.extension_frames==0);}
      assert(std::abs(tr.kf.vx())<1e-12);
      if(!broad_extension){
        const bool old_visibility=n.visibility_frame_valid_,old_tracking=n.enable_tracking_;
        n.resetTrackingState();n.visibility_frame_valid_=false;n.enable_tracking_=true;
        tr.id=818;tr.hits=50;tr.age=50;tr.last_time=9.9;tr.last_range=10;
        tr.feature=f;tr.vehicle_status_initialized=true;tr.detection.class_name="VEHICLE";tr.detection.is_valid=true;
        tr.observation_memory.last_seen_stamp=9.9;tr.observation_memory.last_observed_valid=true;
        tr.observation_memory.last_observed_x=-9;tr.observation_memory.last_observed_y=0;
        n.tracks_.push_back(tr);n.updateTracks({f},10,points,f.indices);
        assert(n.tracks_.size()==1 && n.tracks_[0].id==818 && n.tracks_[0].observation_memory.vehicle_confirmed);
        assert(n.recording_track_features_.count(818)==0); // FOV unavailable: no fabricated measurement.
        n.resetTrackingState();n.visibility_frame_valid_=old_visibility;n.enable_tracking_=old_tracking;
      }
    }
    n.tracking_filter_mode_=old_filter;n.sensor_origin_x_=old_sx;n.sensor_origin_y_=old_sy;
    n.vehicle_max_length_=old_length;n.vehicle_min_height_=old_height;n.vehicle_min_width_=old_width;
    std::cout<<"Phase5 in-body-height thin hook 2m beyond raw end cannot enter KF/grow/reanchor/spawn duplicate; genuine broad extension still grows with unchanged velocity/reference PASS\n";
  }

  static void bodyMeasurementStabilityCheck(Cx128s2BevPerception & n) {
    const double old_min=n.vehicle_min_width_,old_max=n.vehicle_max_height_;
    const auto old_filter=n.tracking_filter_mode_;n.tracking_filter_mode_="kalman";
    const double old_xres=n.bev_x_res_,old_yres=n.bev_y_res_;auto old_clusterer=std::move(n.clusterer_);
    n.bev_x_res_=n.bev_y_res_=.1;n.clusterer_=n.createClusterer();
    n.vehicle_min_width_=1.5;n.vehicle_max_height_=4.5;
    for(bool moving:{false,true}){
      Track tr;tr.id=701;tr.position_from_body_box=true;auto&m=tr.observation_memory;m.vehicle_confirmed=true;
      m.vehicle_state=Lifecycle::ACTIVE;m.vehicle_obb={};m.vehicle_obb.valid=true;
      m.vehicle_obb.center={-12,0};m.vehicle_obb.length=8;m.vehicle_obb.width=3;
      m.vehicle_z_center=2.1;m.vehicle_height=4;m.reliable=true;
      m.last_reliable_box={-16,-8,-1.5,1.5,.1,4.1};tr.x=-12;tr.y=0;tr.kf.initialize(-12,0,.16,.5);
      double last=-12;
      for(int frame=0;frame<20;++frame){
        const double truth=-12+(moving?.1*frame:0.0);
        std::vector<PointXYZIRT> points;
        for(int ix=0;ix<=40;++ix)for(int iy=0;iy<=32;++iy)for(int iz=0;iz<=12;++iz){
          PointXYZIRT p{};p.x=truth+1+ix*.05;p.y=-.8+iy*.05;p.z=3.2+iz*.05;p.ring=iz;points.push_back(p);
        }
        // Low attached clutter changes the apparent box center, while the
        // genuine body surface has known stationary/translated coordinates.
        for(int i=0;i<30;++i){PointXYZIRT p{};p.x=truth-4-frame*.1;p.y=-1;p.z=.2;points.push_back(p);}
        // Moving overhead returns must not hijack a body-height patch.
        for(int i=0;i<40;++i){PointXYZIRT p{};p.x=truth+frame*.05;p.y=0;p.z=4.8+i*.03;points.push_back(p);}
        ObjectFeature f;f.indices.resize(points.size());std::iota(f.indices.begin(),f.indices.end(),0);
        f.range=std::abs(truth);f.decision_reason="CLUSTER:confirmed_height_support";double measurement=frame?truth-.4:truth;
        tr.kf.reanchorPosition(truth+.6,0); // Deliberately biased prediction.
        tr.position_from_body_box=false;
        const double retained_measurement=measurement;
        assert(!n.measuredVehicleBodyAnchor(tr,f,points,measurement) && measurement==retained_measurement);
        tr.position_from_body_box=true;
        if(frame==0){
          f.decision_reason.clear();
          assert(!n.measuredVehicleBodyAnchor(tr,f,points,measurement) && !m.body_pose_anchor_valid);
          f.decision_reason="CLUSTER:confirmed_height_support";
        }
        const bool used=n.measuredVehicleBodyAnchor(tr,f,points,measurement);
        if(frame==0){assert(!used && measurement==truth);}
        else {if(!used || std::abs(measurement-truth)>=1e-4)std::cerr<<"BODY TEST frame="<<frame<<" moving="<<moving<<" used="<<used<<" measured="<<measurement<<" truth="<<truth<<" anchor="<<m.body_pose_anchor_valid<<" local="<<m.body_pose_anchor_local.x<<" dims="<<m.body_pose_anchor_length<<","<<m.body_pose_anchor_width<<" mode="<<n.perception_mode_<<" near="<<n.near_range_max_<<"\n";assert(used && std::abs(measurement-truth)<1e-4);if(moving)assert(measurement-last>.09);}
        last=measurement;
        tr.kf.update(measurement,0,.16);
      }
      ObjectFeature empty;double unchanged=-9;assert(!n.measuredVehicleBodyAnchor(tr,empty,{},unchanged)&&unchanged==-9);
    }
    // Sample body Y more densely than a BEV cell, avoiding artificial gaps
    // from float values landing on opposite sides of voxel boundaries.
    // Real high, broad cargo is not classified as a thin vertical attachment.
    for(bool wire:{false,true}){
      std::vector<PointXYZIRT> points;
      const double top=wire?3.9:5.5;
      for(int ix=0;ix<=40;++ix)for(int iy=0;iy<=40;++iy)for(int iz=0;iz<=int(top/.2);++iz){
        PointXYZIRT p{};p.x=-12+ix*.2;p.y=-1+iy*.05;p.z=.1+iz*.2;p.ring=iz;points.push_back(p);
      }
      const size_t body_count=points.size();
      if(wire)for(int z=0;z<=30;++z)for(int x=0;x<3;++x){PointXYZIRT p{};p.x=-10+x*.05;p.y=0;p.z=3.8+z*.1;p.ring=z;points.push_back(p);}
      std::vector<int> ids(points.size());std::iota(ids.begin(),ids.end(),0);
      ObjectFeature reference;reference.indices=ids;reference.min_z=.1;
      reference.vehicle_height_supported=false;
      assert(!n.measuredBodyReferenceSupported(reference,points));
      reference.vehicle_height_supported=true;
      assert(n.measuredBodyReferenceSupported(reference,points));
      auto c=n.makeFemCluster(points,ids,7,"test","","",0);std::vector<ClusterResult> clusters{c};n.separateVehicleUpperAttachments(points,clusters);
      std::vector<int> seen(points.size());for(const auto&part:clusters)for(int i:part.indices)++seen.at(i);assert(std::all_of(seen.begin(),seen.end(),[](int n){return n==1;}));
      if(wire){assert(clusters.size()>1);std::vector<bool> retained(points.size());for(int i:clusters[0].indices)retained[i]=true;for(size_t i=0;i<body_count;++i)assert(retained[i]);}
      else assert(clusters.size()==1 && clusters[0].indices==ids);
    }
    // A low platform plus tall wires (and optionally a person) must not gain
    // artificial vehicle height from the clipped wire stubs.
    for(bool person:{false,true}){
      std::vector<PointXYZIRT> points;
      for(int ix=0;ix<=40;++ix)for(int iy=0;iy<=20;++iy)for(int iz=0;iz<=4;++iz){
        PointXYZIRT p{};p.x=-12+ix*.2;p.y=-1+iy*.1;p.z=.1+iz*.2;points.push_back(p);
      }
      for(float x:{-10.f,-7.f})for(int iz=0;iz<=60;++iz)for(int ix=0;ix<3;++ix){
        PointXYZIRT p{};p.x=x+ix*.05;p.y=0;p.z=.8+iz*.1;points.push_back(p);
      }
      if(person)for(int ix=0;ix<4;++ix)for(int iy=0;iy<4;++iy)for(int iz=0;iz<=21;++iz){
        PointXYZIRT p{};p.x=-5+ix*.1;p.y=-.7+iy*.1;p.z=.1+iz*.1;points.push_back(p);
      }
      std::vector<int> ids(points.size());std::iota(ids.begin(),ids.end(),0);
      ObjectFeature reference;reference.indices=ids;reference.min_z=.1;
      reference.vehicle_height_supported=true;
      assert(!n.measuredBodyReferenceSupported(reference,points));
      std::vector<ClusterResult> clusters{n.makeFemCluster(points,ids,8,"low_platform","","",0)};
      n.separateVehicleUpperAttachments(points,clusters);assert(clusters.size()==1 && clusters[0].indices==ids);
    }
    // A mainly upright support must not be shaved down into a vehicle-shaped
    // residue by trying lower and lower cuts.
    {
      std::vector<PointXYZIRT> points;
      for(int ix=0;ix<=24;++ix)for(int iy=0;iy<=18;++iy)for(int iz=0;iz<=38;++iz){
        PointXYZIRT p{};p.x=-24+ix*.1;p.y=-2+iy*.1;p.z=.1+iz*.1;points.push_back(p);
      }
      for(int iz=0;iz<=30;++iz)for(int ix=0;ix<3;++ix){PointXYZIRT p{};p.x=-23+ix*.05;p.y=-1.5;p.z=3.8+iz*.1;points.push_back(p);}
      std::vector<int> ids(points.size());std::iota(ids.begin(),ids.end(),0);
      std::vector<ClusterResult> clusters{n.makeFemCluster(points,ids,9,"upright_test","","",0)};
      n.separateVehicleUpperAttachments(points,clusters);assert(clusters.size()==1 && clusters[0].indices==ids);
    }
    n.bev_x_res_=old_xres;n.bev_y_res_=old_yres;n.clusterer_=std::move(old_clusterer);
    n.vehicle_min_width_=old_min;n.vehicle_max_height_=old_max;n.tracking_filter_mode_=old_filter;reset(n);
    std::cout<<"Measured body anchor: stationary/noisy box, real motion, overhead motion, biased prediction, missing observation, exhaustive split, independent body height, low platform/person, upright support, broad high cargo PASS\n";
  }

  static void partialBoxReferenceCheck(Cx128s2BevPerception & n) {
    const double saved_width=n.vehicle_max_width_;
    reset(n);n.vehicle_max_width_=4.0;
    auto & initial=seed(n,10,false);initial.observation_memory={};
    initial.detection.class_name="UNKNOWN";
    auto partial=feature(n,110.02,20);
    partial.component_type="vehicle_like";partial.length=partial.obb.length=2.12;
    partial.width=partial.obb.width=2.07;partial.height=1.82;
    partial.obb.center={partial.centroid_x-.55,partial.centroid_y};
    auto reference=get(n).kf;reference.predict(.1,n.kf_process_accel_std_mps2_);
    reference.update(partial.centroid_x,partial.centroid_y,n.kf_measurement_position_std_m_);
    n.updateTracks({partial},10.1,{},{});
    assert(get(n).lost_frames==0 && !get(n).position_from_body_box);
    assert(std::abs(get(n).kf.x()-reference.x())<1e-9 &&
           std::abs(get(n).kf.y()-reference.y())<1e-9);
    // A temporary rejection cannot reinitialize a previously public identity's reference.
    get(n).vehicle_status_initialized=true;get(n).detection.class_name="UNKNOWN";
    auto complete=feature(n,110.04,20);complete.component_type="vehicle_like";
    complete.obb.center={complete.centroid_x-.55,complete.centroid_y};
    reference=get(n).kf;reference.predict(.1,n.kf_process_accel_std_mps2_);
    reference.update(complete.centroid_x,complete.centroid_y,n.kf_measurement_position_std_m_);
    n.updateTracks({complete},10.2,{},{});
    assert(get(n).lost_frames==0 && !get(n).position_from_body_box);
    assert(std::abs(get(n).kf.x()-reference.x())<1e-9 &&
           std::abs(get(n).kf.y()-reference.y())<1e-9);
    n.vehicle_max_width_=saved_width;reset(n);
    std::cout<<"Partial-front and previously-public references retain legacy KF/ID through rejection\n";
  }

  using Lifecycle = cx128s2_bev_perception::vehicle_surface::VehicleLifecycle;
  static void disconnectedHeightCheck(Cx128s2BevPerception & n) {
    const double old_height=n.vehicle_min_height_;
    n.vehicle_min_height_=1.8; // Installed General YAML; constructor default is 0.6m.
    std::vector<PointXYZIRT> points;
    for (int ix=0;ix<=40;++ix) for (int iy=0;iy<=20;++iy)
      for (int iz=0;iz<5;++iz) {
        PointXYZIRT p{};p.x=-10+ix*.1;p.y=-1+iy*.1;p.z=.1+iz*.2;
        p.ring=iz;points.push_back(p);
      }
    for (int i=0;i<2;++i) {
      PointXYZIRT p{};p.x=-8+i*.01;p.y=0;p.z=3+i*.05;p.ring=31;points.push_back(p);
    }
    auto feature = [&]() {
      ClusterResult c;c.indices.resize(points.size());
      std::iota(c.indices.begin(),c.indices.end(),0);fillClusterStats(points,c);
      n.last_point_bg_similarity_.assign(points.size(),0);
      auto features=n.buildFeaturesFromClusterResults(points,{c},c.indices);
      assert(features.size()==1);return features.front();
    };
    auto f=feature();assert(f.component_type=="vehicle_like" && !f.vehicle_height_supported);
    Track tr;tr.feature=f;tr.hits=31;tr.recent_hits.assign(31,true);
    assert(n.evaluateDetection(tr).class_name!="VEHICLE");
    // Existing confirmed identities and partial-observation policy remain authoritative.
    tr.observation_memory.vehicle_confirmed=true;
    assert(n.evaluateDetection(tr).class_name=="VEHICLE");
    // A connected vertical observation supports the same unchanged size gate.
    for (int iz=0;iz<=30;++iz) {
      PointXYZIRT p{};p.x=-8;p.y=0;p.z=.1+iz*.1;p.ring=iz;points.push_back(p);
    }
    assert(feature().vehicle_height_supported);
    points.resize(points.size()-31);
    for(auto & p:points)p.x-=45;
    assert(feature().vehicle_height_supported); // No new far-range restriction.
    for(auto & p:points)p.x+=45;
    const auto mode=n.perception_mode_;n.perception_mode_="crane_beam_work";
    assert(feature().vehicle_height_supported);n.perception_mode_=mode;
    n.vehicle_min_height_=old_height;
    std::cout<<"Near disconnected-height false vehicle rejected; connected, far and Crane evidence retained\n";
  }
  static void frontClassificationCheck(Cx128s2BevPerception & n) {
    reset(n); auto & t=seed(n,52,false);
    t.observation_memory.reliable=false;
    n.visibility_frame_valid_=true;
    t.feature.length=t.feature.obb.length=0.4;
    t.feature.component_type="vehicle_like";t.feature.ring_coverage=8;
    t.feature.inside_roi=true;t.visibility.adaptive_expected_rays=100;
    t.visibility.in_horizontal_fov=t.visibility.in_vertical_fov=t.visibility.in_valid_range=true;
    t.visibility.observable_samples=10;
    std::vector<PointXYZIRT> points;
    for(int ring=0;ring<8;++ring)for(int k=0;k<3;++k){
      PointXYZIRT p;p.x=t.feature.centroid_x;p.y=19.f+k;p.z=1+ring*.3f;p.ring=ring;
      t.feature.indices.push_back(points.size());points.push_back(p);
    }
    assert(n.vehicleFrontSupported(t,t.feature,points));
    n.tentative_confirm_frames_=31;n.tentative_confirm_hits_=28;
    t.hits=31;t.recent_hits=std::deque<bool>(31,true);
    t.recent_front_hits=std::deque<bool>(27,true);
    auto result=n.evaluateDetection(t);
    assert(result.class_name=="VEHICLE" && !result.is_valid);
    t.recent_front_hits.push_back(true);result=n.evaluateDetection(t);
    assert(result.is_valid && result.decision_reason=="VISIBLE_VEHICLE_FRONT+CONFIRMED_BY_PERSISTENCE");
    n.updateTrackObservationMemory(t,t.feature,11);
    assert(!t.observation_memory.reliable && !t.observation_memory.vehicle_confirmed);
    t.lost_frames=1;assert(n.evaluateDetection(t).class_name!="VEHICLE");t.lost_frames=0;
    t.feature.range=44.999;assert(!n.vehicleFrontSupported(t,t.feature,points));
    t.front_class_confirmed=true;assert(n.vehicleFrontSupported(t,t.feature,points));
    t.feature.range=52;
    t.recent_hits=std::deque<bool>(31,false);t.recent_hits.back()=true;
    assert(n.evaluateDetection(t).is_valid); // Confirmed class, new measured front support.
    t.recent_front_hits.back()=false;
    assert(n.evaluateDetection(t).class_name!="VEHICLE"); // No current support: no memory-only output.
    t.recent_front_hits.back()=true;
    t.feature.inside_roi=false;assert(!n.vehicleFrontSupported(t,t.feature,points));t.feature.inside_roi=true;
    assert(!n.vehicleFrontSupported(t,t.feature,{}));
    t.feature.width=t.feature.obb.width=0.2;assert(!n.vehicleFrontSupported(t,t.feature,points));
    n.perception_mode_="crane_beam_work";assert(!n.vehicleFrontSupported(t,t.feature,points));
    reset(n);
  }
  static void frontFragmentsCheck(Cx128s2BevPerception & n) {
    const double width_saved=n.vehicle_min_width_,height_saved=n.vehicle_min_height_;
    n.vehicle_min_width_=1.5;n.vehicle_min_height_=1.8;
    const auto config_saved=n.visibility_model_.config();
    const auto pose_saved=n.visibility_model_.pose();
    const bool roi_saved=n.roi_enabled_,policy_saved=n.use_roi_policy_;
    for(int scenario=0;scenario<10;++scenario) {
      reset(n);n.sensor_origin_x_=0;n.sensor_origin_y_=3;
      n.roi_enabled_=n.use_roi_policy_=false;
      VisibilityConfig cfg;cfg.azimuth_min_deg=30;cfg.azimuth_max_deg=150;
      cfg.elevation_min_deg=-30;cfg.elevation_max_deg=30;
      cfg.azimuth_bin_deg=.1;cfg.elevation_bin_deg=.1;
      n.visibility_model_.configure(cfg);
      tf2::Transform pose;pose.setIdentity();pose.setOrigin(tf2::Vector3(0,3,7));
      pose.setRotation(tf2::Quaternion(tf2::Vector3(0,0,1),-M_PI/2));
      n.visibility_model_.setPose(pose);n.visibility_frame_valid_=true;
      std::vector<PointXYZIRT> points;std::vector<int> all;
      std::vector<ClusterResult> clusters;
      for(int piece=0;piece<2;++piece) {
        std::vector<int> indices;
        for(int ring=0;ring<32;++ring)for(int j=0;j<12;++j) {
          PointXYZIRT q;q.x=52+(j%2?.1f:-.1f);
          q.y=1.8+piece*1.4+j/11.0;
          q.z=scenario==7 ? 1+ring*.5/31 :
              scenario==8 ? 1+piece*1.6+ring*1.5/31 : 1+ring*3.2/31;
          q.ring=ring;
          indices.push_back(points.size());all.push_back(points.size());points.push_back(q);
        }
        clusters.push_back(n.makeFemCluster(points,indices,11+piece,"fixture","","",0));
      }
      n.last_point_bg_similarity_.assign(points.size(),0);
      auto previous=n.buildFeaturesFromClusterResults(points,
          {n.makeFemCluster(points,all,10,"previous_observation","","",0)},all);
      auto observations=n.buildFeaturesFromClusterResults(points,clusters,all);
      assert(previous.size()==1 && observations.size()==2);
      assert(observations[0].component_type!="vehicle_like" &&
             observations[1].component_type!="vehicle_like");
      if(scenario==8) assert(observations[0].height<n.vehicle_min_height_ &&
                             observations[1].height<n.vehicle_min_height_);
      auto & tr=seed(n,52,false);tr.feature=previous[0];
      tr.front_class_confirmed=scenario!=4;tr.observation_memory.reliable=false;
      tr.recent_front_hits=std::deque<bool>(31,true);
      tr.hits=tr.age=31;
      if(scenario==9) tr.kf.reanchorPosition(52.6,3.0); // Filter residual, not measured-box motion.
      if(scenario==1) observations.clear();
      if(scenario==2) {observations[1].min_y+=3;observations[1].max_y+=3;observations[1].centroid_y+=3;}
      if(scenario==3) {
        for(auto & f:observations) {f.indices={f.indices.front(),f.indices.back()};f.point_count=2;}
      }
      if(scenario==6) {
        auto other=tr;other.id=78;other.kf.initialize(52,3.5,.12,1);
        n.tracks_.push_back(other);
      }
      std::vector<int> owners(points.size(),-1);
      for(const auto & f:observations)for(int index:f.indices)owners[index]=f.id;
      n.visibility_model_.buildDepth(points,owners);
      n.updateTracks(observations,scenario==5?10.7:10.1,points,all);
      if(scenario==0 || scenario==8 || scenario==9) {
        assert(get(n).lost_frames==0 && get(n).recent_front_hits.back());
        assert(get(n).feature.point_count==static_cast<int>(points.size()));
        assert(get(n).feature.decision_reason.find("CURRENT_FRONT_COMPONENT_SUPPORT")!=std::string::npos);
        assert(n.recording_track_features_.at(77)==&n.recording_sparse_features_.at(77));
        assert(n.recording_track_features_.at(77)->indices.size()==points.size());
        assert(!n.persistentVehicle(get(n)));
      } else assert(get(n).lost_frames>0);
    }
    n.vehicle_min_width_=width_saved;n.vehicle_min_height_=height_saved;
    n.roi_enabled_=roi_saved;n.use_roi_policy_=policy_saved;
    n.visibility_model_.configure(config_saved);n.visibility_model_.setPose(pose_saved);
    reset(n);
  }
  static void firstVehicleStatusCheck(Cx128s2BevPerception & n) {
    reset(n);auto & t=seed(n,35);
    t.kf.x_[2]=2.0;
    t.vehicle_status.value="exiting"; // A generic candidate's previous value.
    t.vehicle_status_initialized=false;
    const double start_x=t.x,start_y=t.y;
    for(int frame=1;frame<=2;++frame) {
      auto f=feature(n,start_x+.2*frame,start_y);
      std::vector<PointXYZIRT> points;
      for(int j=0;j<100;++j) {
        PointXYZIRT p{};p.x=f.centroid_x+(j%10)*.02f;
        p.y=f.centroid_y+(j/10)*.02f;p.z=1+(j%8)*.3f;p.ring=j%8;
        f.indices.push_back(points.size());points.push_back(p);
      }
      n.updateTracks({f},10+.1*frame,points,f.indices);
      assert(get(n).vehicle_status_initialized && get(n).lost_frames==0);
      assert(get(n).detection.is_valid && get(n).detection.class_name=="VEHICLE");
      assert(n.tracks_.size()==1 && get(n).id==77);
      assert(get(n).vehicle_status.value==(frame==1?"stopping":"entering"));
    }
    reset(n);n.updateTracks({},11,{},{});
    assert(n.tracks_.empty()); // Default status cannot create a vehicle.
  }
  static void frontMotionEvidenceCheck(Cx128s2BevPerception & n) {
    reset(n);auto & t=seed(n,52,false);n.tracking_filter_mode_="ema";
    t.front_class_confirmed=true;t.x=-52;t.y=0;t.vx=-0.4;t.vy=0;
    t.detection.motion_state="MOVING_BACKWARD";
    t.vehicle_status.value="stopped";t.vehicle_status.motion_initialized=true;
    t.vehicle_status.last_observation_sec=10;t.vehicle_status.stationary_since_sec=0;
    t.observation_memory.corridor_direction.candidate=
        cx128s2_bev_perception::vehicle_surface::CorridorDirection::UNKNOWN;
    t.observation_memory.corridor_direction.consecutive=0;
    n.evaluateVehicleStatus(t,10.1);
    assert(t.vehicle_status.value=="stopped" && t.vehicle_status.observation_gap);
    assert(t.vehicle_status.stationary_since_sec==0); // No invented stationary evidence.
    t.observation_memory.corridor_direction.candidate=
        cx128s2_bev_perception::vehicle_surface::CorridorDirection::ENTRY;
    t.observation_memory.corridor_direction.consecutive=2;
    n.evaluateVehicleStatus(t,10.2);assert(t.vehicle_status.value=="entering");
    t.vx=0.5;t.detection.motion_state="MOVING_FORWARD";
    n.evaluateVehicleStatus(t,10.3);assert(t.vehicle_status.value=="entering");
    t.observation_memory.corridor_direction.candidate=
        cx128s2_bev_perception::vehicle_surface::CorridorDirection::EXIT;
    n.evaluateVehicleStatus(t,10.4);assert(t.vehicle_status.value=="exiting");
    reset(n);
  }
  static void ringProfileWithoutPublisherCheck(Cx128s2BevPerception & n) {
    reset(n);const auto saved=n.visibility_model_.config();const auto saved_pub=n.pub_visibility_ring_profile_;
    const auto old_ring=n.last_input_has_ring_;const auto old_done=n.visibility_ring_profile_published_;
    n.pub_visibility_ring_profile_.reset();n.visibility_frame_valid_=true;
    n.last_input_has_ring_=true;n.visibility_ring_profile_published_=false;
    std::vector<PointXYZIRT> points;
    for(int ring=0;ring<32;++ring){double e=(-3.2+0.2*ring)*M_PI/180;
      PointXYZIRT p;p.x=0;p.y=10*std::cos(e);p.z=10*std::sin(e);p.ring=ring;points.push_back(p);}
    tf2::Transform identity;identity.setIdentity();
    n.publishVisibilityRingProfile(points,identity);
    assert(n.visibility_ring_profile_published_);
    assert(std::abs(n.visibility_model_.rayElevationSpacingDeg()-0.2)<1e-4);
    auto cfg=saved;cfg.ray_elevation_spacing_deg=0.125;n.visibility_model_.configure(cfg);
    n.perception_mode_="crane_beam_work";n.visibility_ring_profile_published_=false;
    n.publishVisibilityRingProfile(points,identity);
    assert(!n.visibility_ring_profile_published_);
    assert(std::abs(n.visibility_model_.rayElevationSpacingDeg()-0.125)<1e-9);
    n.visibility_model_.configure(saved);n.pub_visibility_ring_profile_=saved_pub;
    n.last_input_has_ring_=old_ring;n.visibility_ring_profile_published_=old_done;reset(n);
  }
  static void statusDirectionCheck(Cx128s2BevPerception & n) {
    reset(n); auto & tr=seed(n,35);
    n.tracking_filter_mode_="ema";
    tr.x=2;tr.y=0;tr.vx=1;tr.vy=0;
    tr.detection.motion_state="MOVING_FORWARD";
    auto & m=tr.observation_memory;
    m.fifo_phase=cx128s2_bev_perception::vehicle_surface::FifoPhase::EXITING;
    m.corridor_direction.stable=cx128s2_bev_perception::vehicle_surface::CorridorDirection::EXIT;
    m.fifo_exit_observed=true;
    n.evaluateVehicleStatus(tr,10.1);
    assert(tr.vehicle_status.value=="exiting");
    m.fifo_exit_observed=false;
    n.evaluateVehicleStatus(tr,10.2);
    assert(tr.vehicle_status.value=="entering");
  }
  static ObjectFeature feature(Cx128s2BevPerception & n, double x, double y) {
    ObjectFeature f;
    f.id = 11; f.centroid_x = x; f.centroid_y = y; f.centroid_z = 2.0;
    f.length = 6.0; f.width = 2.4; f.height = 3.2;
    f.obb = {{x + 0.25, y + 0.1}, 0.0, f.length, f.width, true};
    f.min_x = x - 3; f.max_x = x + 3; f.min_y = y - 1.2; f.max_y = y + 1.2;
    f.min_z = 0.4; f.max_z = 3.6;
    f.point_count = 100; f.cell_count = 40; f.dynamic_score = 1;
    f.range = n.sensorRange(x, y);
    return f;
  }
  static void reset(Cx128s2BevPerception & n) {
    n.resetTrackingState();
    // Hand-seeded fixtures have no current recording observation.
    n.recording_track_features_.clear();
    n.recording_sparse_features_.clear();
    n.tracking_filter_mode_ = "kalman";
    n.perception_mode_ = "default";
    n.enable_tracking_ = true;
    n.near_range_max_ = 20; n.mid_range_max_ = 45; n.long_range_max_ = 100;
    n.sensor_origin_x_ = 100; n.sensor_origin_y_ = 20;
    n.visibility_frame_valid_ = false;
    n.vehicle_reacquire_enabled_ = true;
    n.keep_box_while_lost_ = false;
    n.max_lost_frames_near_ = 3; n.max_lost_frames_mid_ = 3; n.max_lost_frames_long_ = 3;
    n.vehicle_lost_memory_sec_ = 0.5;
    n.vehicle_identity_memory_sec_ = 2;
    n.enable_shape_memory_tracking_ = false;
    n.shape_graph_compute_policy_ = "manual_prepare";
    n.vehicle_require_entry_direction_ = false;
    n.vehicle_reacquire_attempt_count_ = n.vehicle_reacquire_success_count_ = 0;
    n.vehicle_dormant_reacquire_attempt_count_ = 0;
    n.memory_compare_using_cache_count_ = n.memory_compare_using_current_shape_graph_count_ = 0;
    n.object_memory_bank_enabled_ = false;
    n.background_mode_ = "ground_only";
    n.learning_active_ = false;
  }
  static Track & seed(Cx128s2BevPerception & n, double range, bool confirmed = true) {
    Track tr; tr.id = 77; tr.x = n.sensor_origin_x_ + range; tr.y = n.sensor_origin_y_;
    tr.feature = feature(n, tr.x, tr.y);
    tr.z = 2; tr.last_time = tr.last_prediction_time = 10; tr.last_range = range;
    tr.hits = tr.age = 10; tr.recent_hits = {true,true,true,true,true};
    tr.kf.initialize(tr.x, tr.y, n.kf_initial_position_std_m_, n.kf_initial_velocity_std_mps_);
    auto & m = tr.observation_memory;
    m.vehicle_confirmed = confirmed; m.vehicle_state = Lifecycle::ACTIVE;
    m.vehicle_obb = {{tr.x, tr.y}, 0, 6, 2.4, true};
    m.reliable = true; m.reliable_length = 6; m.reliable_width = 2.4; m.reliable_height = 3.2;
    m.vehicle_height = 3.2; m.vehicle_z_center = 2;
    m.last_observed_x = tr.x; m.last_observed_y = tr.y; m.last_observed_valid = true;
    m.last_seen_stamp = 10; m.lateral_anchor = {tr.y - 1, 0, 0, true};
    m.corridor_direction.stable = cx128s2_bev_perception::vehicle_surface::CorridorDirection::ENTRY;
    m.local_surface = cx128s2_bev_perception::vehicle_surface::faceSamples(6,2.4);
    n.tracks_.push_back(tr);
    n.next_track_id_ = 100;
    return n.tracks_.back();
  }
  static Track & get(Cx128s2BevPerception & n) {
    auto it = std::find_if(n.tracks_.begin(),n.tracks_.end(),[](const Track & t){return t.id==77;});
    assert(it != n.tracks_.end()); return *it;
  }
  static void nearMotion(Cx128s2BevPerception & n, double range, bool confirmed) {
    reset(n); seed(n,range,confirmed);
    // Shape memory remains stored, while similarity association is disabled nearby.
    n.enable_shape_memory_tracking_ = true;
    auto & initial = get(n);
    initial.shape_memory.valid = initial.shape_memory.committed = true;
    initial.shape_memory.memory_confidence = 1;
    initial.shape_memory.class_label = "TRUCK";
    for(int i=1;i<=10;++i) {
      auto f = feature(n,100+range+0.04*i,20+0.03*i);
      std::vector<ObjectFeature> observations{f};
      auto reference = get(n).kf;
      reference.predict(0.1,n.kf_process_accel_std_mps2_);
      if (confirmed) {
        reference.dampLateralVelocity(std::exp(-0.1));
        Track predicted=get(n);predicted.kf=reference;
        auto measured=n.correctedVehicleCenter(predicted,f.centroid_x,f.centroid_y,
            cx128s2_bev_perception::vehicle_surface::lateralSurfaceGain(
                f.range,n.near_range_max_,n.mid_range_max_,false));
        // This fixture observes the whole 6m length, despite partial support.
        measured.x=f.obb.center.x;
        const double std_x=4*n.kf_measurement_position_std_m_;
        double std_y=cx128s2_bev_perception::vehicle_surface::lateralMeasurementStd(
            std_x,f.range,n.near_range_max_,n.mid_range_max_,1);
        const bool accepted=cx128s2_bev_perception::vehicle_surface::lateralMeasurementAccepted(
            measured.y,reference.y(),predicted.observation_memory.lateral_anchor.y,true,2.4);
        if (!accepted) std_y=std::max(std_y,2.0);
        reference.update(measured.x,accepted?measured.y:reference.y(),std_x,std_y);
        reference.updateLateralAnchor(predicted.observation_memory.lateral_anchor.y,
            std::clamp(2.4*(range<20?0.4:0.2),0.25,0.8));
      } else reference.update(f.centroid_x,f.centroid_y,n.kf_measurement_position_std_m_);
      n.updateTracks(observations,10+0.1*i,{},{});
      auto & t=get(n);
      assert(t.lost_frames==0 && !n.memoryMatchingBlocked(n.trackSensorRange(t)));
      assert(std::abs(t.kf.x()-reference.x())<1e-9);
      assert(std::abs(t.kf.y()-reference.y())<1e-9);
      assert(std::abs(t.kf.vy()-reference.vy())<1e-9);
      assert(n.toMsg(t).class_name != "TRUCK");
      if(confirmed) {
        assert(t.observation_memory.lateral_update_reason!="KF_ONLY");
        assert(t.observation_memory.lateral_anchor.valid);
        assert(n.toMsg(t).dimensions.x==6 && n.toMsg(t).dimensions.y==2.4 && n.toMsg(t).dimensions.z==3.2);
      }
    }
    // Confirmation changes the position reference without erasing motion.
    auto & t=get(n);
    t.observation_memory.vehicle_confirmed=false;
    t.observation_memory.length_evidence=GeometryEvidence::MEASURED;
    t.detection.is_valid=true; t.detection.class_name="VEHICLE";
    for (int i=1;i<=20;++i) {
      t.kf.predict(0.1, n.kf_process_accel_std_mps2_);
      t.kf.update(t.feature.centroid_x + 0.25*i, t.feature.centroid_y,
                  n.kf_measurement_position_std_m_);
    }
    const double confirmation_vx=t.kf.vx(), confirmation_vy=t.kf.vy();
    const double confirmation_variance=t.kf.positionVarianceX();
    assert(confirmation_vx>2.0);
    n.updateVehicleSurface(t,t.feature,{});
    assert(t.observation_memory.vehicle_confirmed);
    assert(t.kf.x()==t.feature.obb.center.x && t.kf.y()==t.feature.obb.center.y);
    assert(t.kf.vx()==confirmation_vx && t.kf.vy()==confirmation_vy);
    assert(t.vx==confirmation_vx && t.vy==confirmation_vy);
    assert(t.kf.positionVarianceX()==confirmation_variance);
  }
  static void disappearance(Cx128s2BevPerception & n,double range,bool residual) {
    reset(n); seed(n,range);
    auto noise=feature(n,100+range,20);
    noise.length=noise.width=noise.height=0.2;
    noise.obb.length=noise.obb.width=0.2;
    // More than the old 10-point PARTIAL threshold, but no vehicle-shaped extent.
    noise.point_count=20; noise.cell_count=4;
    for(int i=1;i<=4;++i) {
      std::vector<ObjectFeature> observations;
      if(residual) observations.push_back(noise);
      n.updateTracks(observations,10+0.1*i,{},{});
      auto & t=get(n);
      assert(t.lost_frames==i);
      assert(!n.physicalOutputAllowed(t));
    }
    assert(n.predictiveLostVehicle(get(n)));
    n.updateTracks({},11.0,{},{});
    assert(n.dormantVehicle(get(n)));
    const double frozen=get(n).kf.x();
    n.updateTracks({},11.5,{},{});
    assert(get(n).kf.x()==frozen);
    n.updateTracks({},13.0,{},{});
    assert(std::none_of(n.tracks_.begin(),n.tracks_.end(),[](const Track&t){return t.id==77;}));
    assert(!n.inside_vehicle_queue_.contains(77));
    assert(n.pending_exit_track_id_==-1);
  }
  static void nearReacquire(Cx128s2BevPerception & n) {
    for (double range : {30.0, 34.999, 35.0, 35.001, 44.999, 45.0, 45.001}) {
      reset(n);n.mid_range_max_=35;auto & tr=seed(n,range);
      tr.observation_memory.vehicle_state=Lifecycle::LOST;
      tr.observation_memory.lost_since=10;tr.lost_frames=4;
      const auto f=feature(n,100+range,20);
      n.updateTracks({f},10.1,{},{});
      assert(n.predictiveLostVehicle(get(n)));
      assert(n.tracks_.size()==1); // Reserved evidence must not create another ID.
      n.updateTracks({f},10.2,{},{});
      assert(!n.lostVehicle(get(n)) && get(n).lost_frames==0);
      assert(n.vehicle_reacquire_success_count_==1 && n.tracks_.size()==1);
      assert(n.memoryMatchingBlocked(range)==(range>=35));
    }
    std::cout<<"Confirmed two-frame LOST recovery crosses 35m/45m without duplicate IDs; new-candidate memory restriction retained\n";
  }
  static void nearExit(Cx128s2BevPerception & n) {
    reset(n); seed(n,30);
    for(int i=1;i<=4;++i) {
      std::vector<ObjectFeature> obs{feature(n,130-0.1*i,20)};
      n.updateTracks(obs,10+0.1*i,{},{});
      assert(get(n).lost_frames==0);
    }
    assert(get(n).observation_memory.corridor_direction.stable==
        cx128s2_bev_perception::vehicle_surface::CorridorDirection::EXIT);
    assert(get(n).observation_memory.fifo_phase==
        cx128s2_bev_perception::vehicle_surface::FifoPhase::EXITING);
  }
  static void longRange(Cx128s2BevPerception & n) {
    reset(n); seed(n,46);
    auto f=feature(n,146.1,20);
    std::vector<ObjectFeature> observations{f};
    n.updateTracks(observations,10.1,{},{});
    auto & t=get(n);
    assert(n.memoryMatchingBlocked(n.trackSensorRange(t)) && t.lost_frames==0);
    assert(t.observation_memory.lateral_update_reason!="KF_ONLY");
    assert(t.observation_memory.measurement_std_y>n.kf_measurement_position_std_m_);
    assert(std::hypot(t.observation_memory.measured_x-f.centroid_x,
                      t.observation_memory.measured_y-f.centroid_y)>0.01);
    t.observation_memory.vehicle_state=Lifecycle::LOST;
    t.observation_memory.lost_since=10.1; t.lost_frames=4;
    n.updateTracks(observations,10.2,{},{});
    assert(n.predictiveLostVehicle(get(n)));
    n.updateTracks(observations,10.3,{},{});
    assert(!n.lostVehicle(get(n)) && get(n).lost_frames==0);
    assert(get(n).observation_memory.reacquire_result=="REACQUIRED");
  }
  static void partialLongitudinalDriftCheck(Cx128s2BevPerception & n) {
    for (double speed : {0.0, 0.8}) {
      reset(n);auto & t=seed(n,30);const double start=t.x, y=t.y;
      // The prior velocity disagrees with the next stationary measurements.
      t.kf.x_[2]=0.4;
      double initial_error=0.0;
      for(int i=1;i<=80;++i) {
        auto f=feature(n,start+speed*.1*i,y);
        f.obb.center={start+speed*.1*i,y};
        f.width=f.obb.width=1.0; // Side/partial: deliberately not FULL width.
        f.min_y=y-.5;f.max_y=y+.5;
        n.updateTracks({f},10+.1*i,{},{});
        auto & current=get(n);
        assert(current.id==77 && current.lost_frames==0);
        assert(current.observation_memory.association_source=="PARTIAL_SUPPORT");
        assert(std::abs(current.observation_memory.measured_x-f.obb.center.x)<1e-9);
        if(i==1)initial_error=std::abs(n.outputX(current)-(start+speed*.1*i));
      }
      const double error=std::abs(n.outputX(get(n))-(start+speed*8));
      std::cout<<"Partial longitudinal drift: speed="<<speed<<" final_error="<<error
               <<" velocity="<<n.outputVx(get(n))<<" initial_error="<<initial_error<<"\n";
      assert(error<0.1);
      assert(std::abs(n.outputVx(get(n))-speed)<0.1);
    }
    reset(n);auto & t=seed(n,30);const double x=t.x,y=t.y;
    auto implausible=feature(n,x+1.1,y);
    implausible.obb.center={x+3.8,y};
    implausible.min_x=x+.8;implausible.max_x=x+6.8;
    implausible.width=implausible.obb.width=1;
    n.updateTracks({implausible},10.1,{},{});
    assert(get(n).lost_frames==0);
    assert(std::abs(get(n).observation_memory.measured_x-x)<1e-9);
    std::cout<<"Partial center beyond existing innovation bound rejected\n";
    reset(n);
  }
  static void extentReferenceCheck(Cx128s2BevPerception & n) {
    // More of a stationary or moving object's length becomes visible behind
    // its sensor-facing end. Box-center motion must not become vehicle speed.
    for (double speed : {0.0, 2.5}) {
      reset(n); auto & t=seed(n,30);
      const double initial_x=t.x, initial_y=t.y;
      for (int i=1;i<=5;++i) {
        const double center_x=initial_x+speed*0.1*i;
        auto f=feature(n,center_x+3.0,initial_y);
        f.length=12.0; f.obb={{center_x+3.0,initial_y},0,12.0,2.4,true};
        f.min_x=center_x-3.0; f.max_x=center_x+9.0;
        bool aligned=false;
        const auto measured=n.correctedVehicleCenter(t,f.centroid_x,f.centroid_y,1.0,&f,&aligned);
        assert(aligned && std::abs(measured.x-center_x)<1e-9);
        n.updateTracks({f},10+0.1*i,{},{});
        assert(n.tracks_.size()==1 && get(n).id==77 && get(n).lost_frames==0);
        assert(std::abs(n.outputX(get(n))-center_x)<1.0);
      }
      auto wide=feature(n,initial_x,initial_y);
      wide.obb.length=12;wide.obb.width=4;
      bool aligned=true;
      n.correctedVehicleCenter(get(n),wide.centroid_x,wide.centroid_y,1.0,&wide,&aligned);
      assert(!aligned);
      auto unrelated=feature(n,initial_x+20,initial_y);
      unrelated.obb.length=12;
      n.correctedVehicleCenter(get(n),unrelated.centroid_x,unrelated.centroid_y,1.0,&unrelated,&aligned);
      assert(!aligned);
    }
    reset(n); auto & blind=seed(n,30);
    const auto saved_config=n.visibility_model_.config();
    const auto saved_pose=n.visibility_model_.pose();
    auto config=saved_config; config.azimuth_min_deg=90;config.azimuth_max_deg=100;
    n.visibility_model_.configure(config);
    tf2::Transform identity;identity.setIdentity();n.visibility_model_.setPose(identity);
    n.visibility_frame_valid_=true;
    auto growing=feature(n,blind.x+3,blind.y);
    growing.obb={{blind.x+3,blind.y},0,12,2.4,true};
    bool aligned=true;
    n.correctedVehicleCenter(blind,growing.centroid_x,growing.centroid_y,1.0,&growing,&aligned);
    assert(!aligned);  // An unobservable end face cannot anchor length growth.
    n.visibility_model_.configure(saved_config);n.visibility_model_.setPose(saved_pose);
    n.visibility_frame_valid_=false;
  }
  static void checks(Cx128s2BevPerception & n) {
    for(double range:{30.0,44.0}) for(bool confirmed:{false,true}) nearMotion(n,range,confirmed);
    std::cout<<"A/B: 30m and 44m updates use pre-Oct13 KF equations, surface corrections and lateral anchor\n";
    assert(n.memoryMatchingBlocked(45) && n.memoryMatchingBlocked(45.001));
    assert(!n.memoryMatchingBlocked(44.999));
    longRange(n);
    std::cout<<"C/F: 46m memory correction and two-frame LOST reacquisition retained\n";
    for(double range:{30.0,44.0,46.0}) for(bool residual:{false,true}) disappearance(n,range,residual);
    std::cout<<"D/E: absent/residual observations increment loss, freeze identity and expire safely\n";
    nearReacquire(n);
    nearExit(n);
    reset(n); auto & t=seed(n,30);
    n.keep_box_while_lost_=true; t.lost_frames=1; t.observation_memory.missed_time_sec=0.2;
    assert(n.physicalOutputAllowed(t));
    t.observation_memory.missed_time_sec=0.6; assert(!n.physicalOutputAllowed(t));
    t.observation_memory.missed_time_sec=0.2; t.lost_frames=4; assert(!n.physicalOutputAllowed(t));
    // Partial support can be valid without using remembered position as a measurement.
    auto f=feature(n,130,20); f.obb.width=f.width=0.4; f.height=1.4;
    t.lost_frames=1; assert(n.vehicleObservationFailure(t,f,{})!=nullptr);
    n.visibility_frame_valid_=true; t.visibility.adaptive_expected_rays=1000;
    assert(n.vehicleObservationFailure(t,f,{})!=nullptr);
    n.visibility_frame_valid_=false;
    n.tracking_filter_mode_="legacy"; assert(!n.memoryMatchingBlocked(n.trackSensorRange(t)));
    std::cout<<"Boundary, sensor offset, partial validation, prediction timeout and legacy selection pass\n";
  }
  static void nearMemory(Cx128s2BevPerception & n) {
    reset(n);seed(n,30);
    n.shape_memory_reprepare_enabled_=false;
    n.enable_shape_memory_tracking_=true;
    auto & t=get(n);
    t.shape_memory.valid=t.shape_memory.committed=t.shape_memory.frozen=true;
    t.shape_memory.confidence=1;t.shape_memory.length=6;t.shape_memory.width=2.4;t.shape_memory.height=3.2;
    t.observed_shape_cache.state="capturing";t.observed_shape_cache.target_capture_frames=3;
    std::vector<PointXYZIRT> points;
    auto f=feature(n,130,20);
    for(float x:{127.f,130.f,133.f})for(float y:{18.8f,20.f,21.2f})for(float z:{.4f,2.f,3.6f}) {
      PointXYZIRT p;p.x=x;p.y=y;p.z=z;p.ring=points.size();
      f.indices.push_back(points.size());points.push_back(p);
    }
    cx128s2_bev_perception::VisibilityConfig cfg;
    cfg.azimuth_min_deg=-30;cfg.azimuth_max_deg=30;
    cfg.elevation_min_deg=-30;cfg.elevation_max_deg=30;
    cfg.azimuth_bin_deg=1;cfg.elevation_bin_deg=1;cfg.ray_elevation_spacing_deg=1;
    n.visibility_model_.configure(cfg);
    tf2::Transform pose;pose.setIdentity();pose.setOrigin(tf2::Vector3(100,20,7));
    n.visibility_model_.setPose(pose);n.visibility_frame_valid_=true;
    for(int frame=1;frame<=3;++frame) {
      n.visibility_model_.buildDepth(points,std::vector<int>(points.size(),f.id));
      n.updateTracks({f},10+0.1*frame,points,f.indices);
      assert(get(n).lost_frames==0);
      assert(get(n).visibility.adaptive_expected_rays>0);
      assert(get(n).observation_memory.expected_visible_samples>0);
      assert(get(n).observed_shape_cache.captured_frames==frame);
      assert(get(n).shape_memory.low_match_frames==0 && !get(n).shape_memory.reprepare_needed);
    }
    assert(get(n).observed_shape_cache.state=="ready" && get(n).observed_shape_cache.valid);
    assert(get(n).observed_shape_cache.accumulated_local_points.size()==3*points.size());
    assert(!get(n).observed_shape_cache.nodes.empty());
    assert(n.memory_compare_using_cache_count_==0 && n.memory_compare_using_current_shape_graph_count_==0);
    // Temporal history cannot bypass insufficient raw spatial/angular support.
    n.updateTracks({},10.4,points,f.indices);
    assert(get(n).lost_frames==1 && !n.physicalOutputAllowed(get(n)));
    assert(get(n).observation_memory.sparse_support_count>0);
    // Candidate memory is blocked at/beyond the configured mid-range boundary.
    reset(n);auto & remembered=seed(n,30,false);n.enable_shape_memory_tracking_=true;
    auto & mem=remembered.shape_memory;
    mem.valid=mem.committed=true;mem.confidence=1;mem.length=6;mem.width=2.4;mem.height=3.2;
    n.object_memory_bank_enabled_=true;
    ObjectMemoryEntry entry;entry.memory_id="remembered_vehicle";entry.shape_memory=mem;
    entry.class_label="VEHICLE";entry.memory_confidence=1;
    n.object_memory_bank_.entries={entry};
    for(double range:{45.0,45.001}) {
      auto near=feature(n,100+range,20);
      assert(n.computeShapeMemoryMatchScore(get(n),near,{})==0);
      assert(n.matchObjectMemoryBank(near,{}).memory_id.empty());
    }
    auto far=feature(n,130,20);
    assert(n.computeShapeMemoryMatchScore(get(n),far,{})>n.shape_memory_min_match_score_);
    const ShapeMemory saved=mem;
    // An identical nearby shape outside the spatial gate must get a separate ID.
    reset(n);auto & near_track=seed(n,30,false);n.enable_shape_memory_tracking_=true;
    near_track.shape_memory=saved;
    n.updateTracks({feature(n,115,20)},10.1,{},{});
    assert(get(n).lost_frames==1);
    assert(n.tracks_.size()==2);
    std::cout<<"Near visibility/surface evaluation and prepared point capture persist; similarity, auto-reprepare and sparse recovery remain disabled\n";
  }
  static void rawBoxRefinementCheck(Cx128s2BevPerception & n) {
    reset(n);seed(n,30,false);
    std::vector<PointXYZIRT> points;
    ClusterResult cluster;cluster.id=11;cluster.cell_count=9;
    for(float x:{127.f,130.f,133.f})for(float y:{18.8f,20.f,21.2f})for(float z:{.5f,2.f,3.5f}) {
      PointXYZIRT p;p.x=x;p.y=y;p.z=z;cluster.indices.push_back(points.size());points.push_back(p);
    }
    const auto append=[&](float x,float y,float z){PointXYZIRT p;p.x=x;p.y=y;p.z=z;points.push_back(p);};
    append(133.05f,20,2);append(130,21.25f,3.55f);
    append(130,20,0);append(130,20,5);append(130,20,2);
    std::vector<int> owners(points.size(),-1);
    for(int i:cluster.indices)owners[i]=11;
    owners.back()=99;  // Another foreground object cannot be absorbed into this vehicle.
    n.last_point_bg_similarity_.assign(points.size(),0);
    auto seed_features=n.buildFeaturesFromClusterResults(points,{cluster},cluster.indices);
    assert(seed_features.size()==1);
    const auto original=seed_features[0];
    auto observations=seed_features;
    n.raw_vehicle_box_refinement_enabled_=false;
    n.refineVehicleFeaturesWithRawBox(points,owners,observations);
    assert(!observations[0].raw_box_used && n.raw_vehicle_box_stats_.scanned_points==0);
    n.raw_vehicle_box_refinement_enabled_=true;
    n.refineVehicleFeaturesWithRawBox(points,owners,observations);
    const auto & refined=observations[0];
    assert(refined.raw_box_used && refined.raw_box_point_count==29 && refined.raw_box_foreground_count==27);
    assert(refined.indices==original.indices && refined.point_count==original.point_count && refined.cell_count==original.cell_count);
    assert(refined.height>original.height && refined.obb.length>original.obb.length);
    assert(refined.centroid_x>original.centroid_x && refined.centroid_z>original.centroid_z);
    auto reference=get(n).kf;
    reference.predict(.1,n.kf_process_accel_std_mps2_);
    reference.update(refined.centroid_x,refined.centroid_y,n.kf_measurement_position_std_m_);
    n.updateTracks(observations,10.1,points,cluster.indices);
    assert(get(n).lost_frames==0 && get(n).feature.raw_box_used);
    assert(std::abs(get(n).kf.x()-reference.x())<1e-9 && std::abs(get(n).kf.y()-reference.y())<1e-9);
    // A background-dominated crop must fall back to the original observed geometry.
    for(int i=0;i<200;++i){append(130,20,2);owners.push_back(-1);}
    observations=seed_features;
    n.refineVehicleFeaturesWithRawBox(points,owners,observations);
    assert(!observations[0].raw_box_used && observations[0].raw_box_reason=="BACKGROUND_DOMINATED");
    assert(observations[0].centroid_x==original.centroid_x && observations[0].indices==original.indices);
    std::vector<ObjectFeature> absent;
    n.refineVehicleFeaturesWithRawBox(points,owners,absent);
    assert(absent.empty() && n.raw_vehicle_box_stats_.scanned_points==0);
    observations=seed_features;observations[0].point_count=1;
    n.refineVehicleFeaturesWithRawBox(points,owners,observations);
    assert(!observations[0].raw_box_used && n.raw_vehicle_box_stats_.scanned_points==0);
    n.raw_vehicle_box_refinement_enabled_=false;
    std::cout<<"Raw 3D points refine geometry/KF without inflating foreground support; ground/other objects/background-only and absent/tiny seeds excluded\n";
  }
  static void entryDirectionOptionCheck(Cx128s2BevPerception & n) {
    using Direction=cx128s2_bev_perception::vehicle_surface::CorridorDirection;
    reset(n); auto & t=seed(n,30,false);
    t.recent_hits={true,true,true,true,true,true,true,true,true,true};
    t.observation_memory.corridor_direction.stable=Direction::UNKNOWN;
    n.vehicle_require_entry_direction_=true;
    auto detection=n.evaluateDetection(t);
    assert(detection.class_name=="VEHICLE" && !detection.is_valid &&
        detection.lifecycle_state=="CANDIDATE" && detection.decision_reason=="AWAITING_ENTRY_DIRECTION");
    t.observation_memory.corridor_direction.stable=Direction::ENTRY;
    assert(n.evaluateDetection(t).is_valid);
    t.observation_memory.corridor_direction.stable=Direction::UNKNOWN;
    n.vehicle_require_entry_direction_=false;
    detection=n.evaluateDetection(t);
    assert(detection.is_valid && detection.class_name=="VEHICLE" && detection.lifecycle_state=="TRACKED" &&
        detection.motion_state=="STATIONARY");
    t.detection=detection;
    // Disabling ENTRY does not bypass the existing measured-geometry identity gate.
    n.updateVehicleSurface(t,t.feature,{});
    assert(!t.observation_memory.vehicle_confirmed);
    t.observation_memory.length_evidence=GeometryEvidence::MEASURED;
    n.updateVehicleSurface(t,t.feature,{});
    assert(t.observation_memory.vehicle_confirmed);
    n.registerVehicleEntry(t,10);
    assert(n.inside_vehicle_queue_.contains(t.id));
    // Existing identities stay confirmed even if the option is enabled later on restart.
    n.vehicle_require_entry_direction_=true;
    assert(n.evaluateDetection(t).is_valid);
    reset(n); auto & weak=seed(n,30,false);
    weak.hits=1;weak.recent_hits={true};weak.observation_memory.corridor_direction.stable=Direction::UNKNOWN;
    n.vehicle_require_entry_direction_=false;
    assert(!n.evaluateDetection(weak).is_valid);
    weak.hits=10;weak.recent_hits={true,true,true,true,true,true,true,true,true,true};
    weak.feature.height=0.2;
    // The optional UNKNOWN-obstacle fallback may still confirm an obstacle,
    // but invalid vehicle dimensions must never confirm it as a VEHICLE.
    assert(n.evaluateDetection(weak).class_name!="VEHICLE");
    auto result=n.onSetParameters({rclcpp::Parameter("vehicle_require_entry_direction",true)});
    assert(!result.successful && result.reason.find("restart")!=std::string::npos);
    std::cout<<"Optional ENTRY gate: parked vehicle confirms with option off; on requires ENTRY; size/persistence/geometry and FIFO rules retained\n";
  }
  static void candidatePublicationCheck(Cx128s2BevPerception & n) {
    reset(n); seed(n,30,false);
    auto & t=get(n);
    n.vehicle_require_entry_direction_=true;
    t.observation_memory.corridor_direction.stable=cx128s2_bev_perception::vehicle_surface::CorridorDirection::UNKNOWN;
    t.hits=1; t.recent_hits={true};
    t.detection=n.evaluateDetection(t);
    assert(t.detection.class_name=="VEHICLE" && !t.detection.is_valid);
    assert(t.detection.lifecycle_state=="CANDIDATE");
    auto observer=std::make_shared<rclcpp::Node>("candidate_publication_check");
    std::array<int,4> counts{{-1,-1,-1,-1}};
    int marker_adds=-1; bool candidate_label=false;
    std::vector<rclcpp::Subscription<cx128s2_bev_perception::msg::TrackedObjectArray>::SharedPtr> subs;
    const std::array<const char *,4> topics{{"/detected_objects","/tracked_objects","/object_motion_states","/road_vehicle_tracks"}};
    for(size_t i=0;i<topics.size();++i)
      subs.push_back(observer->create_subscription<cx128s2_bev_perception::msg::TrackedObjectArray>(topics[i],10,
          [&,i](cx128s2_bev_perception::msg::TrackedObjectArray::ConstSharedPtr msg){
            counts[i]=static_cast<int>(msg->objects.size());
            for(const auto & obj:msg->objects) assert(obj.lifecycle_state!="CANDIDATE");
          }));
    auto markers=observer->create_subscription<visualization_msgs::msg::MarkerArray>("/debug/object_markers",10,
        [&](visualization_msgs::msg::MarkerArray::ConstSharedPtr msg){
          marker_adds=0;
          for(const auto & marker:msg->markers) {
            if(marker.action==visualization_msgs::msg::Marker::ADD) ++marker_adds;
            if(marker.text.find("CANDIDATE")!=std::string::npos) candidate_label=true;
          }
        });
    rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(observer);
    const auto connected=[&] {
      return n.pub_detected_objects_->get_subscription_count()>0 && n.pub_tracked_objects_->get_subscription_count()>0 &&
          n.pub_motion_states_->get_subscription_count()>0 && n.pub_vehicle_tracks_->get_subscription_count()>0 &&
          n.pub_markers_->get_subscription_count()>0;
    };
    for(int i=0;i<100 && !connected();++i) {executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(20));}
    assert(connected());
    std_msgs::msg::Header header;header.frame_id="world";
    const auto publish_and_receive=[&] {
      counts.fill(-1);marker_adds=-1;candidate_label=false;
      n.publishObjects(header);n.publishMarkers(header,n.hasDisplayableTrack());
      for(int i=0;i<100 && (marker_adds<0 || std::any_of(counts.begin(),counts.end(),[](int c){return c<0;}));++i) {
        executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
    };
    publish_and_receive();
    for(int count:counts) assert(count==0);
    assert(marker_adds>=2 && candidate_label && n.last_vehicle_count_==0);
    // Even an inconsistent validity flag cannot leak the explicit CANDIDATE state.
    t.detection.is_valid=true;
    publish_and_receive();for(int count:counts) assert(count==0);
    assert(candidate_label);
    // A stationary candidate stays withheld with ENTRY on; disabling it permits confirmation.
    t.hits=10;t.recent_hits={true,true,true,true,true,true,true,true,true,true};
    t.detection=n.evaluateDetection(t);
    assert(!t.detection.is_valid && t.detection.decision_reason=="AWAITING_ENTRY_DIRECTION");
    publish_and_receive();for(int count:counts) assert(count==0);
    n.vehicle_require_entry_direction_=false;
    t.detection=n.evaluateDetection(t);
    assert(t.detection.is_valid && t.detection.lifecycle_state=="TRACKED");
    publish_and_receive();for(int count:counts) assert(count==1);
    assert(marker_adds>=2 && !candidate_label && n.last_vehicle_count_==1 && t.id==77);
    // A TRACKED label alone must not bypass the internal validity gate.
    t.detection.is_valid=false;
    publish_and_receive();for(int count:counts) assert(count==0);
    std::cout<<"Candidate remains drawn; all four bridge-input arrays stay empty until confirmation of the same ID\n";
  }
  static void outputCheck(Cx128s2BevPerception & n) {
    reset(n); seed(n,30);
    auto observer=std::make_shared<rclcpp::Node>("tracking_policy_output_check");
    int callbacks=0,empty=0,identities=0,markers=-1,recorded=-1;
    std::vector<rclcpp::Subscription<cx128s2_bev_perception::msg::TrackedObjectArray>::SharedPtr> subs;
    for(const char * topic:{"/detected_objects","/tracked_objects","/object_motion_states","/road_vehicle_tracks"})
      subs.push_back(observer->create_subscription<cx128s2_bev_perception::msg::TrackedObjectArray>(topic,10,
          [&](cx128s2_bev_perception::msg::TrackedObjectArray::ConstSharedPtr msg){++callbacks;if(msg->objects.empty())++empty;}));
    auto identity=observer->create_subscription<cx128s2_bev_perception::msg::TrackedObjectArray>("/vehicle_identity_memory",10,
        [&](cx128s2_bev_perception::msg::TrackedObjectArray::ConstSharedPtr msg){identities=msg->objects.size();if(identities)assert(msg->objects[0].lifecycle_state!="ACTIVE");});
    auto marker=observer->create_subscription<visualization_msgs::msg::MarkerArray>("/debug/object_markers",10,
        [&](visualization_msgs::msg::MarkerArray::ConstSharedPtr msg){markers=0;for(auto & m:msg->markers)if(m.action==visualization_msgs::msg::Marker::ADD)++markers;});
    auto recorder=observer->create_subscription<cx128s2_bev_perception::msg::PerceptionRecordingFrame>(
        "/recording/perception_frame",rclcpp::SensorDataQoS(),
        [&](cx128s2_bev_perception::msg::PerceptionRecordingFrame::ConstSharedPtr msg){recorded=msg->objects.size();});
    n.publish_recording_frame_=true;
    n.recording_include_full_cloud_=false;
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(observer);
    const auto connected=[&] {
      return n.pub_vehicle_tracks_->get_subscription_count()>0 && n.pub_motion_states_->get_subscription_count()>0 &&
          n.pub_detected_objects_->get_subscription_count()>0 && n.pub_tracked_objects_->get_subscription_count()>0 &&
          n.pub_vehicle_identity_memory_->get_subscription_count()>0 && n.pub_markers_->get_subscription_count()>0 &&
          n.pub_recording_frame_->get_subscription_count()>0;
    };
    for(int i=0;i<100 && !connected();++i) {
      executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    assert(connected());
    get(n).lost_frames=1;get(n).observation_memory.missed_time_sec=0.1;
    std_msgs::msg::Header header;header.frame_id="world";
    n.publishObjects(header);n.publishMarkers(header,n.hasDisplayableTrack());
    n.publishRecordingFrame(header,{},{});
    for(int i=0;i<100 && (callbacks<4 || identities==0 || markers<0 || recorded<0);++i) {
      executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    assert(callbacks==4 && empty==4 && identities==1 && markers==0 && recorded==0);
    std::cout<<"Actual ROS publications: four physical arrays and recorder empty, identity preserved, no box ADD\n";
    // Cross the configured mid boundary with the same FIFO identity. No points
    // must not remove long-range predictions from detection/tracking outputs.
    n.mid_range_max_=35;
    n.registerVehicleEntry(get(n),10); // Missed tracks cannot register.
    assert(!n.inside_vehicle_queue_.contains(77));
    get(n).lost_frames=0;n.registerVehicleEntry(get(n),10);
    assert(n.inside_vehicle_queue_.contains(77));
    for (double range : {34.999,35.0,35.001,45.0}) {
      for (Lifecycle state : {Lifecycle::ACTIVE,Lifecycle::LOST,Lifecycle::DORMANT}) {
        auto & t=get(n);auto & m=t.observation_memory;
        t.kf.initialize(n.sensor_origin_x_+range,n.sensor_origin_y_,1,1);
        t.lost_frames=100;m.missed_time_sec=100;m.vehicle_state=state;
        m.final_predicted_valid=state==Lifecycle::DORMANT;
        m.final_predicted_x=n.outputX(t);m.final_predicted_y=n.outputY(t);
        t.detection=n.evaluateDetection(t);
        const double actual_x=m.last_observed_x, actual_stamp=m.last_seen_stamp;
        const int actual_hits=t.hits;
        const bool visible=range>=35 && state!=Lifecycle::DORMANT;
        assert(n.physicalOutputAllowed(t)==visible);
        const auto obj=n.toMsg(t);
        assert(obj.point_count==0 && obj.id==77 && obj.dimensions.x==6 && obj.dimensions.y==2.4);
        if(state==Lifecycle::DORMANT) assert(obj.lifecycle_state=="DORMANT");
        callbacks=empty=identities=0;markers=recorded=-1;
        n.publishObjects(header);n.publishMarkers(header,n.hasDisplayableTrack());
        n.publishRecordingFrame(header,{},{});
        for(int i=0;i<100 && (callbacks<4 || identities==0 || markers<0 || recorded<0);++i) {
          executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        assert(callbacks==4 && identities==1);
        assert(empty==(visible?0:4));
        assert((markers>0)==visible && recorded==(visible?1:0));
        assert(n.inside_vehicle_queue_.contains(77));
        assert(m.last_observed_x==actual_x && m.last_seen_stamp==actual_stamp && t.hits==actual_hits);
      }
    }
    auto & t=get(n);n.inside_vehicle_queue_.remove(77);
    assert(!n.physicalOutputAllowed(t));
    n.inside_vehicle_queue_.append(77);t.observation_memory.vehicle_confirmed=false;
    assert(!n.physicalOutputAllowed(t));
    std::cout<<"35m FIFO detection promotion: predicted/LOST publish without points; DORMANT outputs hidden with identity/FIFO preserved; real observation memory unchanged\n";
  }
  static void visibilityPreferenceCheck(Cx128s2BevPerception & n) {
    reset(n); auto & t=seed(n,30);
    t.observation_memory.vehicle_obb.yaw=.3;
    const auto original=n.toMsg(t).dimensions;
    n.visibility_box_prior_weight_=1.0;
    n.visibility_frame_valid_=true;
    t.visibility.adaptive_expected_rays=120;
    t.visibility.adaptive_supported_rays=30;
    const double x=n.outputX(t), y=n.outputY(t);
    n.updateConfirmedVehicleKalman(t,x+.25,y+.1,30,VehicleObservationQuality::PARTIAL);
    assert(std::abs(t.kf.innovation()[0]-.25)<1e-9);
    assert(std::abs(t.kf.innovation()[1]-.1)<1e-9);
    assert(t.kf.x()>x && t.kf.x()<x+.25);
    assert(t.observation_memory.measurement_std_x>4*n.kf_measurement_position_std_m_);
    assert(n.getTrackYaw(t)==n.vehicle_fixed_yaw_rad_);
    auto dims=n.toMsg(t).dimensions;
    assert(dims.x==original.x && dims.y==original.y && dims.z==original.z);
    reset(n);auto & candidate=seed(n,30,false);
    candidate.observation_memory.reliable=false;
    candidate.feature.obb.yaw=.7;candidate.vx=0;candidate.vy=3;candidate.yaw=1.57;
    assert(n.getTrackYaw(candidate)==n.vehicle_fixed_yaw_rad_);
    n.vehicle_fixed_yaw_rad_=.4;
    assert(n.getTrackYaw(candidate)==.4);
    n.vehicle_fixed_yaw_rad_=0;
    std::cout<<"Visibility adjusts uncertainty without prediction feedback; KF never sets box yaw\n";
  }
  static void mergedSparseCheck(Cx128s2BevPerception & n) {
    // A component rejected for an attached high structure must not hide the
    // independently supported vehicle returns inside its remembered volume.
    std::vector<std::pair<double,double>> reference_positions;
    for (double ceiling : {6.4,4.4}) for (bool above_vehicle : {false,true}) {
      reset(n);seed(n,20);n.visibility_frame_valid_=true;n.roi_enabled_=false;
      // The seed represents an accepted cluster at t=10. Its raw centroid,
      // not a prediction or remembered point, anchors the first handoff.
      get(n).observation_memory.raw_centroid_x=120;
      get(n).observation_memory.raw_centroid_y=20;
      get(n).observation_memory.last_seen_stamp=10;
      cx128s2_bev_perception::VisibilityConfig cfg;
      cfg.azimuth_min_deg=60;cfg.azimuth_max_deg=120;
      cfg.elevation_min_deg=-30;cfg.elevation_max_deg=30;
      cfg.azimuth_bin_deg=.1;cfg.elevation_bin_deg=.05;cfg.ray_elevation_spacing_deg=.05;
      n.visibility_model_.configure(cfg);
      tf2::Transform pose;pose.setIdentity();pose.setOrigin(tf2::Vector3(100,20,7));
      tf2::Quaternion sensor_rotation;sensor_rotation.setRPY(0,0,-M_PI/2);
      pose.setRotation(sensor_rotation);
      n.visibility_model_.setPose(pose);
      for (int frame=1;frame<=3;++frame) {
        if (!above_vehicle && frame==2) {
          // A newer observed front can have another centroid. Keep the still
          // coherent previous raw observation instead of replacing it.
          get(n).observation_memory.raw_centroid_x=123;
          get(n).observation_memory.last_seen_stamp=10.15;
        }
        auto f=feature(n,120,20);f.height=ceiling-.4;f.max_z=ceiling;
        std::vector<PointXYZIRT> points;
        for (int i=0;i<2048;++i) {
          PointXYZIRT p;p.x=120;p.y=18.8+2.4*(i/128)/15.0;
          p.z=above_vehicle ? 6.1 : .4+3.2*(i%128)/127.0;p.ring=i%128;
          f.indices.push_back(points.size());points.push_back(p);
        }
        if (ceiling==4.4 && !above_vehicle) {
          // Moving overhead returns fit the GLOBAL height limit but lie above
          // the remembered 3.2m body. They must not pull the vehicle sideways.
          for(int i=0;i<1024;++i) {
            PointXYZIRT p;p.x=120+.5*frame;p.y=20+.2*frame;
            p.z=4.1;p.ring=i%128;
            f.indices.push_back(points.size());points.push_back(p);
          }
          f.centroid_x=120+.5*frame/3;f.centroid_y=20+.2*frame/3;
          f.obb.center={120+.25+.5*frame,20+.1+.2*frame};
        }
        f.point_count=points.size();
        n.visibility_model_.buildDepth(points,std::vector<int>(points.size(),f.id));
        n.updateTracks({f},10+.1*frame,points,f.indices);
        assert(get(n).observation_memory.vehicle_height==3.2);
        assert(get(n).observation_memory.vehicle_z_center==2);
        if(!above_vehicle) {
          if(ceiling==6.4)reference_positions.push_back({n.outputX(get(n)),n.outputY(get(n))});
          else assert(std::hypot(n.outputX(get(n))-reference_positions[frame-1].first,
                                 n.outputY(get(n))-reference_positions[frame-1].second)<1e-6);
        }

        if (!above_vehicle && (frame==1 || frame==2)) {
          assert(get(n).lost_frames==0);
          assert(get(n).observation_memory.association_source=="SPARSE_SUPPORT" ||
              get(n).feature.decision_reason.find("confirmed_height_support")!=std::string::npos);
          assert(n.toMsg(get(n)).point_count==2048);
        }
      }
      assert(get(n).id==77 && n.persistentVehicle(get(n)));
      if (above_vehicle) assert(get(n).lost_frames>0);
      else {
        assert(get(n).lost_frames==0);
        assert(get(n).observation_memory.association_source=="SPARSE_SUPPORT" ||
              get(n).feature.decision_reason.find("confirmed_height_support")!=std::string::npos);
        assert(n.toMsg(get(n)).point_count>256);
        assert(n.toMsg(get(n)).point_count==2048);
        assert(n.toMsg(get(n)).ring_coverage==128);
        assert(n.recording_track_features_.at(77)->indices.size()==2048);
      }
      assert(std::count_if(n.tracks_.begin(),n.tracks_.end(),[&](const Track & t) {
        return n.persistentVehicle(t);
      })==1);
    }
    // Filtered features must also stay alive with FOV disabled and two bodies.
    reset(n);seed(n,30);seed(n,50).id=78;n.roi_enabled_=false;
    n.visibility_frame_valid_=false;
    std::vector<PointXYZIRT> points;std::vector<ObjectFeature> observations;
    for(double x:{130.,150.}) {
      auto f=feature(n,x,20);f.id=x==130?11:12;f.height=4;f.max_z=4.4;
      for(int ix=0;ix<=6;++ix)for(int iy=0;iy<=4;++iy)for(int iz=0;iz<=4;++iz) {
        PointXYZIRT p;p.x=x-3+ix;p.y=18.8+.6*iy;p.z=.4+.8*iz;p.ring=iz;
        f.indices.push_back(points.size());points.push_back(p);
      }
      for(int i=0;i<64;++i) {
        PointXYZIRT p;p.x=x+1;p.y=20;p.z=4.1;p.ring=i;
        f.indices.push_back(points.size());points.push_back(p);
      }
      observations.push_back(f);
    }
    n.last_point_bg_similarity_.assign(points.size(),0);
    std::vector<int> indices(points.size());std::iota(indices.begin(),indices.end(),0);
    n.updateTracks(observations,10.1,points,indices);
    for(int id:{77,78}) {
      const auto & t=*std::find_if(n.tracks_.begin(),n.tracks_.end(),[&](const Track & t){return t.id==id;});
      assert(t.lost_frames==0 && t.observation_memory.vehicle_height==3.2);
      assert(n.recording_track_features_.at(id)->indices.size()==175);
      assert(n.recording_track_features_.at(id)->max_z<3.7);
    }
    std::cout<<"Moving overhead crane excluded; body XY/height unchanged, 2048 real body points retained; overhead-only cannot become observation\n";
  }
  static void partialFaceCheck(Cx128s2BevPerception & n) {
    reset(n);seed(n,35);n.visibility_frame_valid_=true;
    cx128s2_bev_perception::VisibilityConfig cfg;
    cfg.azimuth_min_deg=-30;cfg.azimuth_max_deg=30;
    cfg.elevation_min_deg=-30;cfg.elevation_max_deg=30;
    cfg.azimuth_bin_deg=1;cfg.elevation_bin_deg=1;cfg.ray_elevation_spacing_deg=1;
    n.visibility_model_.configure(cfg);
    tf2::Transform pose;pose.setIdentity();pose.setOrigin(tf2::Vector3(100,20,7));
    n.visibility_model_.setPose(pose);
    for(int frame=1;frame<=3;++frame) {
      auto f=feature(n,138+.1*frame,20);
      f.length=f.obb.length=.3;f.height=2;
      f.obb.center={f.centroid_x,f.centroid_y};
      f.min_x=f.centroid_x-.15;f.max_x=f.centroid_x+.15;
      f.min_z=1;f.max_z=3;
      std::vector<PointXYZIRT> points;
      for(int i=0;i<100;++i) {
        PointXYZIRT p;p.x=f.centroid_x+(i%2? .15:-.15);
        p.y=18.8+(i%10)*2.4/9;p.z=1+(i/10)*2.0/9;p.ring=i;
        f.indices.push_back(points.size());points.push_back(p);
      }
      n.visibility_model_.buildDepth(points,std::vector<int>(points.size(),f.id));
      get(n).visibility.adaptive_expected_rays=1000;
      assert(!n.vehicleObservationFailure(get(n),f,points));
      n.updateTracks({f},10+.1*frame,points,f.indices);
      assert(get(n).lost_frames==0 && n.tracks_.size()==1);
      auto dims=n.toMsg(get(n)).dimensions;
      assert(dims.x==6 && dims.y==2.4 && dims.z==3.2);
      assert(n.getTrackYaw(get(n))==n.vehicle_fixed_yaw_rad_);
      Track initial=get(n);initial.observation_memory.vehicle_confirmed=false;
      assert(n.vehicleObservationFailure(initial,f,points)!=nullptr);
    }
    reset(n);auto & first=seed(n,35);first.observation_memory.vehicle_obb.center={135,20};
    auto second=first;second.id=78;second.kf.initialize(135,20,.12,1);n.tracks_.push_back(second);
    n.updateTracks({feature(n,135,20)},10.1,{},{});
    assert(get(n).lost_frames==1 && n.tracks_.size()==2);
    reset(n);auto & lost=seed(n,35);
    lost.observation_memory.vehicle_state=Lifecycle::LOST;
    lost.observation_memory.lost_since=10;lost.lost_frames=4;
    auto competitor=lost;competitor.id=78;n.tracks_.push_back(competitor);
    n.updateTracks({feature(n,135,20)},10.1,{},{});
    assert(n.tracks_.size()==2 && n.vehicle_reacquire_ambiguous_count_==1);
    assert(n.lostVehicle(get(n)));
    reset(n);auto & original=seed(n,35);
    original.observation_memory.vehicle_state=Lifecycle::LOST;
    original.observation_memory.lost_since=10;original.lost_frames=4;
    n.updateTracks({feature(n,155,20)},10.1,{},{});
    assert(n.lostVehicle(get(n)) && n.vehicle_reacquire_success_count_==0);
    std::cout<<"Thin front face keeps confirmed ID/width/yaw; initial size checks and ambiguous-owner rejection remain\n";
  }
  static void fifoDormantCheck(Cx128s2BevPerception & n) {
    using namespace cx128s2_bev_perception::vehicle_surface;
    reset(n);auto & boot=seed(n,40);
    boot.observation_memory.corridor_direction.stable=CorridorDirection::UNKNOWN;
    n.vehicle_require_entry_direction_=true;n.registerVehicleEntry(boot,10);
    assert(n.inside_vehicle_queue_.size()==0);
    n.vehicle_require_entry_direction_=false;n.registerVehicleEntry(boot,10);
    n.registerVehicleEntry(boot,10.1);
    assert(n.inside_vehicle_queue_.size()==1 && n.inside_vehicle_queue_.front()==77);
    assert(boot.observation_memory.fifo_phase==FifoPhase::INSIDE);

    // Prediction closer to the exit must not erase an unobserved FIFO identity.
    reset(n);n.sensor_origin_x_=0;n.sensor_origin_y_=3;
    auto & exit=seed(n,4);exit.observation_memory.vehicle_state=Lifecycle::DORMANT;
    exit.observation_memory.fifo_phase=FifoPhase::EXITING;
    exit.observation_memory.dormant_since=10;
    exit.observation_memory.final_predicted_valid=true;
    exit.observation_memory.final_predicted_x=0;exit.observation_memory.final_predicted_y=0;
    n.inside_vehicle_queue_.append(77);n.updateTracks({},10.1,{},{});
    assert(n.tracks_.size()==1 && n.inside_vehicle_queue_.contains(77));
    // Actual observed crossing, independently of a final prediction, completes exit.
    assert(fifoExitComplete({8,0},{4,0},6));
    assert(!fifoExitComplete({8,0},{9,0},6));
    get(n).observation_memory.fifo_exit_observed=true;
    n.updateTracks({},10.2,{},{});
    assert(n.tracks_.empty() && !n.inside_vehicle_queue_.contains(77));

    for (int scenario : {0, 1, 2, 3, 4, 5}) {
      const bool returning = scenario != 1;
      reset(n);n.mid_range_max_=35;n.sensor_origin_x_=0;n.sensor_origin_y_=3;
      n.vehicle_identity_memory_sec_=1000;n.visibility_frame_valid_=true;
      auto & old=seed(n,50);auto & m=old.observation_memory;
      old.vehicle_status_initialized=true;
      old.vehicle_status.value=old.vehicle_status.last_directional_value="exiting";
      old.vehicle_status.motion_initialized=true;
      m.vehicle_state=Lifecycle::DORMANT;m.dormant_since=11;m.last_seen_stamp=10;
      m.fifo_phase=FifoPhase::INSIDE;m.last_observed_x=50;m.last_observed_y=3;
      m.final_predicted_x=85;m.final_predicted_y=3;m.final_predicted_valid=true;
      m.lateral_anchor.y=3;old.kf.initialize(85,3,.12,1);old.last_prediction_time=11;
      n.inside_vehicle_queue_.append(77);
      if (scenario==3) {
        auto next=old;next.id=79;next.observation_memory.vehicle_obb.width=3.2;
        n.tracks_.push_back(next);n.inside_vehicle_queue_.append(79);
      }
      RoiZone zone;zone.name="lane";zone.type="watch";zone.shape="box";
      zone.x_min=-100;zone.x_max=200;zone.y_min=-20;zone.y_max=20;n.roi_zones_={zone};
      VisibilityConfig cfg;cfg.azimuth_min_deg=-30;cfg.azimuth_max_deg=30;
      cfg.elevation_min_deg=-30;cfg.elevation_max_deg=30;
      cfg.azimuth_bin_deg=1;cfg.elevation_bin_deg=1;cfg.ray_elevation_spacing_deg=1;
      n.visibility_model_.configure(cfg);tf2::Transform pose;pose.setIdentity();
      pose.setOrigin(tf2::Vector3(0,3,7));n.visibility_model_.setPose(pose);
      for(int i=0;i<5;++i) {
        const double x=returning?45.3-.1*i:55+.1*i;
        auto f=feature(n,x,3);f.length=f.obb.length=.3;f.width=f.obb.width=2.4;f.height=2;
        f.obb.center={x,3};f.min_x=x-.15;f.max_x=x+.15;f.min_y=1.8;f.max_y=4.2;
        f.min_z=1;f.max_z=3;f.range=n.sensorRange(x,3);
        if(scenario==4) {
          f.centroid_y=f.obb.center.y=3.6;f.min_y+=.6;f.max_y+=.6;
          f.range=n.sensorRange(x,3.6);
        }
        if(scenario==5) {f.centroid_z+=3;f.min_z+=3;f.max_z+=3;}
        std::vector<PointXYZIRT> points;
        for(int j=0;j<100;++j) {
          PointXYZIRT p;p.x=x+(j%2?.15:-.15);p.y=1.8+(j%10)*2.4/9;
          p.z=1+(j/10)*2.0/9;p.ring=j;if(scenario==4)p.y+=.6;if(scenario==5)p.z+=3;
          f.indices.push_back(points.size());points.push_back(p);
        }
        std::vector<ObjectFeature> observations{f};
        std::vector<int> owners(points.size(),f.id);
        if(scenario==2) {
          auto other=f;other.id=12;other.centroid_x+=4;other.obb.center.x+=4;
          other.min_x+=4;other.max_x+=4;other.range=n.sensorRange(other.centroid_x,3);
          other.indices.clear();
          for(int j=0;j<100;++j) {
            auto p=points[j];p.x+=4;other.indices.push_back(points.size());points.push_back(p);owners.push_back(other.id);
          }
          observations.push_back(other);
        }
        if(scenario==3) {
          observations[0].length=observations[0].obb.length=6;
          observations[0].width=observations[0].obb.width=4;
        }
        n.visibility_model_.buildDepth(points,owners);
        std::vector<int> all(points.size());std::iota(all.begin(),all.end(),0);
        n.updateTracks(observations,100+.1*i,points,all);
        if(i<4) {
          assert(n.dormantVehicle(get(n)));
          assert(get(n).observation_memory.last_observed_x==50);
          assert(get(n).observation_memory.last_seen_stamp==10);
          assert(get(n).observation_memory.final_predicted_x==85);
        }
      }
      if(scenario==2 || scenario==3 || scenario==5) {
        assert(n.dormantVehicle(get(n)) && get(n).observation_memory.last_observed_x==50);
        assert(n.inside_vehicle_queue_.front()==77);
        if(scenario==2) assert(get(n).observation_memory.reacquire_result=="AMBIGUOUS");
        if(scenario==3) assert(n.inside_vehicle_queue_.contains(79));
        if(scenario==5) assert(get(n).observation_memory.reacquire_result=="NO_VISIBLE_MEMORY_SURFACE");
        continue;
      }
      auto & restored=get(n);assert(!n.lostVehicle(restored));
      assert(restored.vehicle_status_initialized && restored.status_velocity_reinitialized);
      assert(restored.vehicle_status.value=="exiting"); // Same ID, KF zero is not a new stop.
      assert(n.vehicle_dormant_reacquire_success_count_>0 && n.tracks_.size()==1);
      const double expected=returning?47.9:58.4;
      assert(std::abs(n.outputX(restored)-expected)<.02);
      if(scenario==4) assert(n.outputY(restored)>=3 && n.outputY(restored)<3.05);
      assert(n.inside_vehicle_queue_.front()==77);
      assert(restored.observation_memory.fifo_phase==(returning?FifoPhase::EXITING:FifoPhase::INSIDE));
      assert(restored.observation_memory.last_seen_stamp==100.4);
      assert(!restored.observation_memory.final_predicted_valid);
      auto dims=n.toMsg(restored).dimensions;assert(dims.x==6 && dims.y==2.4 && dims.z==3.2);
    }
    reset(n);auto & active_front=seed(n,30);
    active_front.observation_memory.fifo_phase=FifoPhase::INSIDE;
    n.inside_vehicle_queue_.append(77);
    auto & entering=seed(n,60,false);entering.id=88;
    const auto anomalies=n.vehicle_fifo_anomaly_count_;
    n.updateTracks({feature(n,160.1,20)},10.1,{},{});
    auto match=std::find_if(n.tracks_.begin(),n.tracks_.end(),[](const Track & t){return t.id==88;});
    assert(match!=n.tracks_.end() && match->lost_frames==0 && n.tracks_.size()==2);
    assert(n.vehicle_fifo_anomaly_count_==anomalies);
    std::cout<<"FIFO bootstrap honors ENTRY option; prediction cannot complete exit; 90-second thin-front EXIT/continued-ENTRY returns restore old ID and real center\n";
  }
  static void craneSlopeCheck() {
    rclcpp::NodeOptions options;
    options.use_global_arguments(false).arguments({"--ros-args","--params-file",
        (std::filesystem::path(__FILE__).parent_path().parent_path()/
         "config/cx128s2_bev_perception_crane.yaml").string()}).parameter_overrides({
        rclcpp::Parameter("perception_mode","crane_beam_work"),
        rclcpp::Parameter("crane_pipeline","slope"),
        rclcpp::Parameter("target_frame","world"),
        rclcpp::Parameter("input_topic","/test/crane_slope_input"),
        rclcpp::Parameter("use_tf_transform",true),
        rclcpp::Parameter("crane_beam.freeze_sensor_transform",false),
        rclcpp::Parameter("background_mode","ground_only"),
        rclcpp::Parameter("auto_load_background_on_start",false),
        rclcpp::Parameter("auto_load_object_memory_bank_on_start",false),
        rclcpp::Parameter("use_roi_policy",false),
        rclcpp::Parameter("visibility.enabled",false),
        rclcpp::Parameter("enable_parallel_processing",false),
        rclcpp::Parameter("save_csv_on_shutdown",false)});
    auto node=std::make_shared<Cx128s2BevPerception>(options);
    assert(node->crane_pipeline_ && !node->crane_beam_detector_);
    assert(node->get_parameter("bev_x_resolution").as_double()==.05);
    assert(node->get_parameter("cluster_z_layer_enabled").as_bool());
    std::vector<PointXYZIRT> points;
    std::vector<int> indices;
    tf2::Quaternion rotation;rotation.setRPY(-1.5671972765207525,-0.027489352235824036,0);
    tf2::Transform pose(rotation,{0,0,6.638276670931532});
    // Dense elevated beam plus ground, encoded in the actual input sensor frame.
    for(int surface=0;surface<2;++surface) for(int x=-60;x<=60;++x) for(int y=-8;y<=8;++y) {
      const auto p=pose.inverse()*tf2::Vector3(x*.025,y*.025,surface ? 2.0 : 0.0);
      PointXYZIRT v;v.x=p.x();v.y=p.y();v.z=p.z();v.ring=(x+60)%128;
      indices.push_back(points.size());points.push_back(v);
    }
    std_msgs::msg::Header header;header.frame_id="test_slope_lidar";header.stamp.sec=10;
    auto cloud=std::make_shared<sensor_msgs::msg::PointCloud2>(node->makeCloud(header,points,indices));
    assert(node->validateCraneSlopeCloud(*cloud));
    auto malformed=*cloud;malformed.data.pop_back();assert(!node->validateCraneSlopeCloud(malformed));
    malformed=*cloud;malformed.is_bigendian=true;assert(!node->validateCraneSlopeCloud(malformed));
    malformed=*cloud;malformed.row_step+=4;assert(!node->validateCraneSlopeCloud(malformed));
    malformed=*cloud;malformed.fields[0].datatype=sensor_msgs::msg::PointField::FLOAT64;
    assert(!node->validateCraneSlopeCloud(malformed));
    auto transform=tf2::toMsg(pose);
    geometry_msgs::msg::TransformStamped stamped;stamped.header.frame_id="world";
    stamped.child_frame_id=header.frame_id;stamped.transform=transform;
    assert(node->tf_buffer_->setTransform(stamped,"offline-test",true));
    node->processCloudFrame({cloud,std::chrono::steady_clock::now(),1});
    const auto & result=node->crane_pipeline_->result();
    assert(result.pose_valid && result.labels.size()==points.size());
    assert(result.clustering.size()>=points.size()/2 && result.cluster_count>0);
    assert(node->crane_auto_status_ && node->crane_auto_status_->state=="IDLE");
    assert(node->crane_auto_status_->header.frame_id=="world");
    assert(node->crane_auto_status_->working_status=="idle" && !node->crane_auto_status_->beam_detected);
    auto command=std::make_shared<std_msgs::msg::String>();command->data="SET_FSM LIFTING";
    node->handleCraneCommand(command);
    assert(node->crane_manual_mode_ && node->crane_pipeline_->result().state==cx128s2_bev_perception::CraneBeamState::IDLE);
    command->data="AUTO";node->handleCraneCommand(command);assert(!node->crane_manual_mode_);
    command->data="HOOK_REFERENCE_BEGIN";node->handleCraneCommand(command); // rejected without legacy access
    command->data="RESET";node->handleCraneCommand(command);
    node->processCloudFrame({cloud,std::chrono::steady_clock::now(),2});assert(!node->crane_reset_pending_);
    // Finite ground heights are supported by the current slope pipeline.
    // Exercise genuine invalid input rather than expecting valid 1m to fail.
    const double ground_before=node->crane_pipeline_->config().ground_z;
    const auto update=node->onSetParameters({rclcpp::Parameter("crane_slope.ground_z",
        std::numeric_limits<double>::quiet_NaN())});
    assert(!update.successful);
    assert(node->crane_pipeline_->config().ground_z==ground_before);
    if (const char * fixtures=std::getenv("CRANE_SLOPE_FIXTURES")) {
      std::vector<std::filesystem::path> files;
      for(const auto & entry:std::filesystem::directory_iterator(fixtures))
        if(entry.path().extension()==".bin")files.push_back(entry.path());
      std::sort(files.begin(),files.end());
      std::vector<double> totals;
      for(size_t frame=0;frame<files.size();++frame) {
        std::ifstream in(files[frame],std::ios::binary);
        cloud->data.assign(std::istreambuf_iterator<char>(in),{});
        cloud->point_step=32;cloud->width=cloud->data.size()/32;cloud->height=1;
        cloud->row_step=cloud->width*32;cloud->fields.clear();
        for(const auto & item:std::vector<std::pair<std::string,uint32_t>>{
            {"x",0},{"y",4},{"z",8},{"intensity",20},{"ring",24},{"time",28}}) {
          sensor_msgs::msg::PointField field;field.name=item.first;field.offset=item.second;field.count=1;
          field.datatype=item.first=="ring"?sensor_msgs::msg::PointField::UINT16:sensor_msgs::msg::PointField::FLOAT32;
          cloud->fields.push_back(field);
        }
        cloud->header.stamp.sec=20+frame;
        assert(node->validateCraneSlopeCloud(*cloud));
        const auto begin=std::chrono::steady_clock::now();
        node->processCloudFrame({cloud,begin,frame+3});
        totals.push_back(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count());
        const auto & r=node->crane_pipeline_->result();
        assert(r.ground.size()+r.non_ground.size()+r.unknown.size()==static_cast<size_t>(node->last_valid_count_));
        std::cout<<"slope_node_frame="<<frame<<" points="<<node->last_valid_count_
          <<" clusters="<<r.cluster_count<<" segment_ms="<<r.segmentation_ms<<" total_ms="<<totals.back()<<'\n';
      }
      assert(!totals.empty());std::sort(totals.begin(),totals.end());
      std::cout<<"slope_node_summary frames="<<totals.size()<<" median_ms="<<totals[totals.size()/2]
        <<" p95_ms="<<totals[static_cast<size_t>(.95*(totals.size()-1))]<<" max_ms="<<totals.back()<<'\n';
    }
    std::cout<<"Crane slope: actual PointCloud2 parser/TF/crop, existing clusterer, status/manual/reset, malformed schema and startup-only selection passed\n";
  }
  static void minimalTopicsCheck() {
    rclcpp::NodeOptions options;
    options.use_global_arguments(false).arguments({"--ros-args","-r","__node:=minimal_topics_check"});
    options.parameter_overrides({
        rclcpp::Parameter("use_tf_transform",false),
        rclcpp::Parameter("target_frame","laser_link"),
        rclcpp::Parameter("background_mode","ground_only"),
        rclcpp::Parameter("auto_load_background_on_start",false),
        rclcpp::Parameter("auto_load_object_memory_bank_on_start",false),
        rclcpp::Parameter("use_roi_policy",false),
        rclcpp::Parameter("publish_debug_topics",false),
        rclcpp::Parameter("publish_recording_frame",false),
        rclcpp::Parameter("save_csv_on_shutdown",false)});
    auto minimal=std::make_shared<Cx128s2BevPerception>(options);
    const std::set<std::string> kept{"/debug/object_markers","/detected_objects","/tracked_objects",
        "/object_motion_states","/road_vehicle_tracks","/vehicle_identity_memory",
        "/debug/foreground_points","/debug/visibility_markers",
        "/rosout","/parameter_events"};
    const auto advertised=minimal->get_node_graph_interface()->get_publisher_names_and_types_by_node(
        minimal->get_name(),"/");
    for(const auto & item:advertised) assert(kept.count(item.first)==1);
    for(const auto & topic:kept) {
      if(topic=="/rosout" || topic=="/parameter_events") continue;
      assert(advertised.count(topic)==1);
    }
    assert(!minimal->pub_recording_frame_ && minimal->pub_foreground_ && !minimal->pub_track_table_);
    assert(minimal->pub_visibility_markers_ && !minimal->pub_shape_memory_markers_);
    assert(!minimal->pub_crane_beam_status_ && !minimal->sub_crane_command_);
    assert(minimal->visibility_enabled_ && minimal->enable_shape_memory_tracking_);
    std::vector<PointXYZIRT> points;
    std::vector<int> indices;
    for(int i=0;i<32;++i) {
      PointXYZIRT p;p.x=30+(i%4)*.2;p.y=1+(i%8)*.1;p.z=.5+(i%3)*.5;p.ring=i;
      indices.push_back(i);points.push_back(p);
    }
    std_msgs::msg::Header header;header.frame_id="laser_link";header.stamp.sec=10;
    auto cloud=std::make_shared<sensor_msgs::msg::PointCloud2>(minimal->makeCloud(header,points,indices));
    auto observer=std::make_shared<rclcpp::Node>("foreground_output_check");
    int foreground_frames=0;
    auto sub=observer->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/debug/foreground_points",rclcpp::SensorDataQoS(),
        [&](sensor_msgs::msg::PointCloud2::ConstSharedPtr frame) {
          ++foreground_frames;assert(frame->header.frame_id=="laser_link");
          assert(frame->header.stamp.sec==10 && frame->width<=points.size());
        });
    rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(observer);
    for(int i=0;i<100 && minimal->pub_foreground_->get_subscription_count()==0;++i) {
      executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    assert(minimal->pub_foreground_->get_subscription_count()>0);
    minimal->processCloudFrame({cloud,std::chrono::steady_clock::now(),1});
    for(int i=0;i<100 && foreground_frames==0;++i) {
      executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    assert(foreground_frames==1);
    assert(minimal->last_transformed_count_==32 && minimal->visibility_frame_valid_);
    minimal->publishTables(header);minimal->publishStatus();
    minimal->publishRecordingFrame(header,points,indices);
    auto result=minimal->onSetParameters({rclcpp::Parameter("publish_debug_topics",true)});
    assert(!result.successful && result.reason.find("restart")!=std::string::npos);
    std::cout<<"Reduced topic mode retains foreground/reference boxes and six result publishers; actual foreground frames arrive without duplicate cloud/table/recorder endpoints\n";
  }
};
int main(int argc,char ** argv) {
  rclcpp::init(argc,argv);
  auto node=std::make_shared<Cx128s2BevPerception>(rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("publish_recording_frame",true)}));
  TrackingRegressionAccess::disconnectedHeightCheck(*node);
  TrackingRegressionAccess::checks(*node);
  TrackingRegressionAccess::partialLongitudinalDriftCheck(*node);
  TrackingRegressionAccess::extentReferenceCheck(*node);
  TrackingRegressionAccess::nearMemory(*node);
  TrackingRegressionAccess::outputCheck(*node);
  TrackingRegressionAccess::rawBoxRefinementCheck(*node);
  TrackingRegressionAccess::entryDirectionOptionCheck(*node);
  TrackingRegressionAccess::candidatePublicationCheck(*node);
  TrackingRegressionAccess::visibilityPreferenceCheck(*node);
  TrackingRegressionAccess::partialFaceCheck(*node);
  TrackingRegressionAccess::statusDirectionCheck(*node);
  TrackingRegressionAccess::frontClassificationCheck(*node);
  TrackingRegressionAccess::frontFragmentsCheck(*node);
  TrackingRegressionAccess::frontMotionEvidenceCheck(*node);
  TrackingRegressionAccess::firstVehicleStatusCheck(*node);
  TrackingRegressionAccess::ringProfileWithoutPublisherCheck(*node);
  TrackingRegressionAccess::mergedSparseCheck(*node);
  TrackingRegressionAccess::fifoDormantCheck(*node);
  TrackingRegressionAccess::phase5DeckCheck(*node);
  TrackingRegressionAccess::phase5PartReferenceCheck(*node);
  TrackingRegressionAccess::phase5LowerAttachmentCheck(*node);
  TrackingRegressionAccess::phase5ExitDirectionCheck(*node);
  TrackingRegressionAccess::phase5GeometryInterferenceCheck(*node);

  TrackingRegressionAccess::bodyMeasurementStabilityCheck(*node);
  TrackingRegressionAccess::partialBoxReferenceCheck(*node);
  node.reset();
  TrackingRegressionAccess::minimalTopicsCheck();
  TrackingRegressionAccess::craneSlopeCheck();
  rclcpp::shutdown();
}
