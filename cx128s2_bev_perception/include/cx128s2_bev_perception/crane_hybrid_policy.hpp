#pragma once
#include <cx128s2_bev_perception/crane_hook_z_density.hpp>
#include <deque>
#include <array>
#include <set>
#include <map>
#include <iostream>
class LatestSuspendedPolicy {
public:
 nlohmann::json cfg,evidence;HookZDensity ground;std::string state="IDLE",working="idle";int id=-1,side=-1,suspend_count=0,lift_count=0;double last=-1;
 // Confirmation memory never supplies measurements or rise history.
 bool suspended_confirmed=false;int confirmed_id=-1,confirmed_side=-1;
 double confirmed_direct_stamp=-1;std::array<double,4> confirmed_geometry{};
 struct Sample{double t,hook_z,body_z;int id,side;};std::deque<Sample> history;
 LatestSuspendedPolicy(){cfg=nlohmann::json::parse(R"json({"min_ground_clearance_m":0.10,"vertical_xy_tolerance_m":0.15,"min_vertical_support_points":3,"min_cargo_points":20,"confirmation_frames":3,"lift_min_rise_m":0.08,"lower_min_translation_m":0.08,"surface_translation_epsilon_m":0.003,"lift_match_tolerance_m":0.035}
)json");}
 template<class Points,class Result,class Config>void process(const Points&raw,const Result&r,const Config&c,double stamp){
 state="IDLE";working="idle";evidence={{"state",state},{"reason","NO_CONFIRMED_DIRECT_CARGO"},{"sides",nlohmann::json::array()}};const auto&w=r.work_status;
 if(!std::isfinite(stamp)||stamp<=0||stamp<=last){history.clear();suspend_count=lift_count=0;suspended_confirmed=false;evidence["reason"]="INVALID_OR_REWOUND_TIME";last=stamp;return;}if(last>0&&stamp-last>.25){history.clear();suspend_count=lift_count=0;}last=stamp;
 if(w.cargo_track_id!=id){id=w.cargo_track_id;side=-1;history.clear();suspend_count=lift_count=0;suspended_confirmed=false;}
 // Reuse latest Cargo confirmation; no histogram cluster is promoted into a Cargo.
 if(w.cargo_observation!="DIRECT"||id<0||!w.work_status||w.confirmed_observation_sec+1e-6<c.cargo_confirm_sec){
  history.clear();suspend_count=lift_count=0;
  if(w.cargo_observation!="HELD" || !w.geometry_locked || !w.work_status || id!=confirmed_id ||
      stamp-confirmed_direct_stamp<0 || stamp-confirmed_direct_stamp>c.cargo_loss_sec)
    suspended_confirmed=false;
  evidence["reconfirmation_memory_valid"]=suspended_confirmed;return;
 }
 working="working";state="WORKING";bool airborne=false,up=false;int chosen=-1;double current_hook=0,current_body=0;
 std::vector<int> ids;if(w.extension_observation=="DIRECT")ids=w.extended_raw_indices;else{ids=w.left_hook.object_point_indices;ids.insert(ids.end(),w.right_hook.object_point_indices.begin(),w.right_hook.object_point_indices.end());}std::sort(ids.begin(),ids.end());ids.erase(std::unique(ids.begin(),ids.end()),ids.end());
 for(int s=0;s<2;++s){const auto&h=s==0?w.left_hook:w.right_hook;const auto&node=s==0?r.top_down_hook.left_housing:r.top_down_hook.right_housing;const auto&mem=s==0?r.top_down_hook.left_housing_memory:r.top_down_hook.right_housing_memory;
  nlohmann::json j={{"side",s},{"eligible",false},{"reason","NO_CURRENT_SUPPORTING_HOOK"}};
  if(!h.hook_detected||!mem.confirmed||int(node.sheave.status)!=1||!node.sheave.points_from_current_frame||node.is_held||(w.cargo_side!="BOTH"&&w.cargo_side!=(s==0?"LEFT":"RIGHT"))){evidence["sides"].push_back(j);continue;}
  auto g=ground.detectGround(raw,h.hook_x,h.hook_y,h.hook_z,c.cargo_roi_width_x_m,c.cargo_roi_width_y_m,r.ground,cfg.value("ground_roi_width_x_m",0.),cfg.value("ground_roi_width_y_m",0.));j["ground_valid"]=g["ground_valid"];if(!g.value("ground_valid",false)){j["reason"]="GROUND_UNKNOWN";evidence["sides"].push_back(j);continue;}
  double bottom=1e9,top=-1e9,sum=0;int n=0,under=0;const double tol=cfg["vertical_xy_tolerance_m"];
  for(int k:ids)if(k>=0&&size_t(k)<raw.size()){const auto&p=raw[k];if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))continue;bottom=std::min(bottom,double(p.z));top=std::max(top,double(p.z));sum+=p.z;++n;if(std::abs(p.x-h.hook_x)<=tol&&std::abs(p.y-h.hook_y)<=tol&&p.z<h.hook_z)++under;}
  double gz=g["ground_z"],clearance=bottom-gz,gap=h.hook_z-top;bool ok=n>=cfg["min_cargo_points"].get<int>()&&under>=cfg["min_vertical_support_points"].get<int>()&&clearance>=cfg["min_ground_clearance_m"].get<double>()&&gap>=0&&gap<=c.connect_max_gap_z;
  j["hook_xyz"]={h.hook_x,h.hook_y,h.hook_z};j["ground_z"]=gz;j["bottom_z"]=n?bottom:0;j["clearance_m"]=n?clearance:0;j["hook_gap_m"]=n?gap:0;j["cargo_points"]=n;j["vertical_support_points"]=under;j["eligible"]=ok;j["reason"]=ok?"DIRECT_CARGO_AIRBORNE_UNDER_HOOK":"CLEARANCE_OR_VERTICAL_ALIGNMENT_OR_NEARNESS_FAILED";
  if(ok&&(side<0||side==s)){airborne=true;chosen=s;current_hook=h.hook_z;current_body=sum/n;
   for(const auto&old:history)if(stamp-old.t>=.3&&stamp-old.t<=.65&&old.id==id&&old.side==s){double dh=h.hook_z-old.hook_z,db=current_body-old.body_z;j["hook_rise_m"]=dh;j["cargo_rise_m"]=db;up=dh>=cfg["lift_min_rise_m"].get<double>()&&db>=cfg["lift_min_rise_m"].get<double>()&&std::abs(dh-db)<=cfg["lift_match_tolerance_m"].get<double>();break;}}
  evidence["sides"].push_back(j);
 }
 const std::array<double,4> current_geometry{{w.length,w.width,w.height,w.yaw}};
 const int confirm=cfg["confirmation_frames"];
 const bool reuse=airborne && suspended_confirmed && w.geometry_locked && id==confirmed_id &&
   chosen==confirmed_side && current_geometry==confirmed_geometry &&
   stamp-confirmed_direct_stamp>=0 && stamp-confirmed_direct_stamp<=c.cargo_loss_sec && suspend_count<confirm;
 suspend_count=airborne?(reuse?confirm:suspend_count+1):0;
 lift_count=airborne&&up?lift_count+1:0;
 if(!airborne)suspended_confirmed=false;
 if(airborne && w.geometry_locked && suspend_count>=confirm){
  suspended_confirmed=true;confirmed_id=id;confirmed_side=chosen;
  confirmed_geometry=current_geometry;confirmed_direct_stamp=stamp;
 }
 evidence["reconfirmation_reused"]=reuse;
 evidence["reconfirmation_memory_valid"]=suspended_confirmed;
 evidence["confirmed_direct_stamp"]=confirmed_direct_stamp;
 if(airborne){side=chosen;history.push_back({stamp,current_hook,current_body,id,side});while(!history.empty()&&stamp-history.front().t>.7)history.pop_front();}else{history.clear();side=-1;}
 evidence["legacy_lift_ready"]=lift_count>=confirm;
 if(suspend_count>=confirm){state="SUSPENDED";working="working";}
 // No latched old Beam state: loss of current eligibility cannot sustain LIFTING.
 evidence["state"]=state;evidence["working_status"]=working;evidence["cargo_id"]=id;evidence["suspend_count"]=suspend_count;evidence["lift_count"]=lift_count;evidence["reason"]=airborne?(state=="SUSPENDED"?"AIRBORNE_DIRECT_ADMISSION_NO_LIFT_HISTORY":"AIRBORNE_DIRECT_CURRENT_EVIDENCE"):"NO_CURRENT_ELIGIBLE_AIRBORNE_CARGO";
 }
 // Downstream additions: never write detector/KF/Geometry state.
 int work_cargo=-1,completed_cargo=-1,support_side=-1,support_track=-1;
 double motion_stamp=-1,last_work_direct=-1,last_connected=-1,input_run_start=-1,event_started=-1;
 double contact_elapsed=0,detach_elapsed=0,lower_distance=0;
 double contact_last=-1,detach_last=-1;
 int lower_count=0,lower_steps=0;
 int rise_count=0,rise_steps=0,surface_cargo=-1;double rise_distance=0;
 std::string completion_reason;
 struct SurfaceColumn { double low,high; };
 std::map<std::pair<int,int>,SurfaceColumn> previous_surface;
 double previous_surface_stamp=-1,previous_hook_z=0;
 bool previous_hook_direct=false;
 struct SurfaceSample {double stamp,hook_z; bool hook_direct; int hook_track; std::map<std::pair<int,int>,SurfaceColumn> columns;double body_median_z;};
 std::deque<SurfaceSample> surface_history;

 template<class Points,class Result,class Config,class BeamConfig>
 void processMotionAndCompletion(const Points& raw,const Result& r,const Config& c,
     const BeamConfig& bc,double stamp,int min_columns=5,double vertical_span_m=.08) {
   const auto& w=r.work_status;
   auto reset_motion=[&](){surface_history.clear();previous_surface.clear();previous_surface_stamp=-1;
     previous_hook_direct=false;lower_count=lower_steps=rise_count=rise_steps=0;lower_distance=rise_distance=0;surface_cargo=-1;};
   auto finish=[&](const std::string& reason){
     completion_reason=reason;event_started=stamp;completed_cargo=work_cargo;
     work_cargo=-1;contact_elapsed=detach_elapsed=0;contact_last=detach_last=-1;reset_motion();
     std::clog<<"[Crane Hybrid completion] reason="<<reason<<" cargo_id="<<completed_cargo
       <<" sensor_stamp="<<stamp<<'\n';
   };
   const double dt=motion_stamp>0?stamp-motion_stamp:0;
   if(!std::isfinite(stamp)||stamp<=0||dt<=0){
     // A rewind/reset has no event or motion history. Duplicate input cannot vote.
     if(!std::isfinite(stamp)||stamp<motion_stamp){work_cargo=completed_cargo=-1;
       event_started=-1;last_work_direct=last_connected=-1;completion_reason.clear();
       contact_elapsed=detach_elapsed=0;contact_last=detach_last=-1;reset_motion();
       motion_stamp=std::isfinite(stamp)&&stamp>0?stamp:-1;input_run_start=motion_stamp;}
     if(motion_stamp<0 && std::isfinite(stamp) && stamp>0){}
     else {
       // process() clears its per-frame projection first. A repeated sensor frame must
       // neither vote nor erase an already active completion event.
       if(std::isfinite(stamp) && stamp==motion_stamp && event_started>=0 && stamp-event_started<3.){
         state="LOWERING_CONFIRMATION";working="drop";evidence["state"]=state;
         evidence["working_status"]=working;evidence["completion_reason"]=completion_reason;
       }
       evidence["motion_reason"]="INVALID_OR_DUPLICATE_TIME";return;
     }
   }
   motion_stamp=stamp;
   const bool continuous=dt>0 && dt<=std::min(.25,c.cargo_loss_sec);
   if(!continuous){input_run_start=stamp;contact_elapsed=detach_elapsed=0;contact_last=detach_last=-1;reset_motion();}
   if(event_started>=0){
     if(stamp-event_started<3.){state="LOWERING_CONFIRMATION";working="drop";
       evidence["state"]=state;evidence["working_status"]=working;
       evidence["completion_reason"]=completion_reason;return;}
     event_started=-1;
     // No same-frame rearm, and no replay of the completed Cargo's event.
     state="IDLE";working="idle";evidence["state"]=state;evidence["working_status"]=working;return;
   }
   std::vector<int> ids;
   if(w.extension_observation=="DIRECT")ids=w.extended_raw_indices;
   else{ids=w.left_hook.object_point_indices;ids.insert(ids.end(),w.right_hook.object_point_indices.begin(),w.right_hook.object_point_indices.end());}
   std::sort(ids.begin(),ids.end());ids.erase(std::unique(ids.begin(),ids.end()),ids.end());
   const size_t measured_points=std::count_if(ids.begin(),ids.end(),[&](int k){return k>=0 && size_t(k)<raw.size() && std::isfinite(raw[k].x) && std::isfinite(raw[k].y) && std::isfinite(raw[k].z);});
   const bool confirmed_direct=measured_points>=cfg["min_cargo_points"].template get<size_t>() &&
     w.cargo_observation=="DIRECT" && w.cargo_track_id>=0 &&
     w.confirmed_observation_sec+1e-6>=c.cargo_confirm_sec;
   // Public SUSPENDED admission can precede stable size lock. Once that work is
   // publicly confirmed, its completion must be tracked independently of motion's lock gate.
   const bool measured=confirmed_direct && w.geometry_locked;
   const bool completed_same=w.cargo_track_id>=0 && w.cargo_track_id==completed_cargo;
   if(completed_same){state="IDLE";working="idle";evidence["state"]=state;evidence["working_status"]=working;return;}
   int current_side=-1,current_track=-1;double hook_z=0;
   bool current_hook=false,connected=false;
   for(int s=0;s<2;++s){
     if(w.cargo_side!="BOTH" && w.cargo_side!=(s==0?"LEFT":"RIGHT"))continue;
     if(work_cargo>=0 && s!=support_side)continue;
     const auto& h=s==0?w.left_hook:w.right_hook;
     const auto& node=s==0?r.top_down_hook.left_housing:r.top_down_hook.right_housing;
     const auto& mem=s==0?r.top_down_hook.left_housing_memory:r.top_down_hook.right_housing_memory;
     if(!h.hook_detected || h.hook_observation!="VIRTUAL_DIRECT" || !mem.confirmed ||
        int(node.sheave.status)!=1 || !node.sheave.points_from_current_frame || node.is_held)continue;

     current_hook=true;current_side=s;current_track=mem.track_id;hook_z=node.sheave.outer_center_z;
     const double cs=std::cos(w.yaw),sn=std::sin(w.yaw);
     const double u=(h.hook_x-w.cargo_x)*cs+(h.hook_y-w.cargo_y)*sn;
     const double v=-(h.hook_x-w.cargo_x)*sn+(h.hook_y-w.cargo_y)*cs;
     connected=std::abs(u)<=w.length*.5+c.connect_xy_margin &&
       std::abs(v)<=w.width*.5+c.connect_xy_margin &&
       h.hook_z>=w.cargo_z && h.hook_z-w.cargo_z<=c.connect_max_gap_z;
     if(work_cargo>=0 && mem.track_id!=support_track){
       if(!measured || w.cargo_track_id!=work_cargo){current_hook=false;continue;}
       if(connected){support_track=mem.track_id;reset_motion();evidence["support_rebound_from_current_observations"]=true;}
       // A replacement tracker ID alone cannot prove the original Hook detached.
       // Keep the original support identity until a connected observation rebinds it.
       else {current_hook=false;continue;}
     }
     break;
   }
   const bool public_working=state=="SUSPENDED" || state=="LIFTING" || state=="LOWERING";
   if(work_cargo<0 && confirmed_direct && current_hook && connected && public_working){
     work_cargo=w.cargo_track_id;support_side=current_side;support_track=current_track;
     contact_elapsed=detach_elapsed=0;contact_last=detach_last=-1;reset_motion();
   }
   const bool same=confirmed_direct && w.cargo_track_id==work_cargo;
   if(same){last_work_direct=stamp;if(current_hook && connected)last_connected=stamp;}
   // An input outage cannot be counted as continuously observed Cargo absence.
   const bool loss=work_cargo>=0 && last_work_direct>0 &&
     stamp-last_work_direct+1e-9>=c.cargo_loss_sec &&
     stamp-input_run_start+1e-9>=c.cargo_loss_sec;
   const bool contact=same && w.beam_ground_clearance_valid &&
     std::isfinite(w.beam_ground_clearance_m) && w.beam_ground_clearance_m>=-.05 &&
     w.beam_ground_clearance_m<=.38;
   const bool detached=same && current_hook && !connected;
   evidence["completion_armed_cargo"]=work_cargo;
   evidence["completion_same_direct_cargo"]=same;
   evidence["completion_current_hook"]=current_hook;
   evidence["completion_connected"]=connected;
   evidence["completion_loss_age_sec"]=last_work_direct>0?stamp-last_work_direct:0;
   evidence["completion_loss_candidate"]=loss;
   evidence["completion_ground_valid"]=w.beam_ground_clearance_valid;
   evidence["completion_clearance_m"]=w.beam_ground_clearance_valid?nlohmann::json(w.beam_ground_clearance_m):nlohmann::json(nullptr);

   contact_elapsed=contact?(continuous && contact_last>0?contact_elapsed+dt:0):0;
   detach_elapsed=detached?(continuous && detach_last>0?detach_elapsed+dt:0):0;
   contact_last=contact?stamp:-1;detach_last=detached?stamp:-1;
   evidence["completion_contact_candidate"]=contact;
   evidence["completion_contact_sec"]=contact_elapsed;
   evidence["completion_detached_candidate"]=detached;
   evidence["completion_detached_sec"]=detach_elapsed;
   evidence["last_confirmed_direct_stamp"]=last_work_direct;
   evidence["input_continuous_sec"]=stamp-input_run_start;
   if(loss)finish("LOSS_TIMEOUT");
   else if(contact_elapsed+1e-9>=.3)finish("GROUND_CONTACT");
   else if(detach_elapsed+1e-9>=.3)finish("HOOK_DETACHED");
   if(event_started>=0){state="LOWERING_CONFIRMATION";working="drop";
     evidence["state"]=state;evidence["working_status"]=working;
     evidence["completion_reason"]=completion_reason;return;}
   const bool motion_eligible=measured && (work_cargo<0?(current_hook && connected):same) &&
     !(!connected && (current_hook || stamp-last_connected>c.cargo_loss_sec));
   if(!motion_eligible){
     reset_motion();evidence["motion_reason"]="NO_CURRENT_SAME_CONNECTED_CARGO";return;
   }
   if(surface_cargo!=w.cargo_track_id){reset_motion();surface_cargo=w.cargo_track_id;}
   std::map<std::pair<int,int>,SurfaceColumn> columns;
   std::map<std::pair<int,int>,std::vector<double>> column_values;
   std::vector<double> body_z_values;body_z_values.reserve(ids.size());
   for(int k:ids)if(k>=0 && size_t(k)<raw.size()){
     const auto& p=raw[k];if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))continue;
     auto key=std::make_pair(int(std::floor(p.x/c.grid_resolution_m)),int(std::floor(p.y/c.grid_resolution_m)));
     column_values[key].push_back(p.z);body_z_values.push_back(p.z);
     auto item=columns.try_emplace(key,SurfaceColumn{p.z,p.z});
     item.first->second.low=std::min(item.first->second.low,double(p.z));
     item.first->second.high=std::max(item.first->second.high,double(p.z));
   }
   std::nth_element(body_z_values.begin(),body_z_values.begin()+body_z_values.size()/2,body_z_values.end());
   const double body_median_z=body_z_values[body_z_values.size()/2];
   // Measured quartiles suppress isolated sling/noise returns within a body column.
   // Both boundaries must still translate together; face loss changes the span and cannot vote.
   for(auto&item:column_values){auto&z=item.second;std::sort(z.begin(),z.end());
     columns[item.first]={z[(z.size()-1)/4],z[3*(z.size()-1)/4]};}
   std::vector<double> dz,rise_dz;size_t matched=0,all_matched=0;
   const double eps=cfg["surface_translation_epsilon_m"].template get<double>();
   while(!surface_history.empty() && stamp-surface_history.front().stamp>.8)surface_history.pop_front();
   const SurfaceSample* reference=nullptr;
   if(surface_history.size()>=3 && stamp-surface_history.front().stamp>=.3)reference=&surface_history.front();
   double surface_window=reference?stamp-reference->stamp:0;
   bool corroborating_hook=reference && current_hook && reference->hook_direct && reference->hook_track==current_track;
   const double hook_window=corroborating_hook?hook_z-reference->hook_z:0;
   const double body_window=reference?body_median_z-reference->body_median_z:0;
   if(reference){
     for(const auto& cell:columns){auto old=reference->columns.find(cell.first);if(old==reference->columns.end())continue;
       ++all_matched;double low=cell.second.low-old->second.low,high=cell.second.high-old->second.high;
       const bool thick=cell.second.high-cell.second.low>=vertical_span_m && old->second.high-old->second.low>=vertical_span_m;
       // Thin surfaces require the independently observed same Sheave's motion.
       // Thick body low/high translation remains usable with a missing Sheave.
       if(!thick && !corroborating_hook)continue;
       const bool shape=std::abs(low-high)<=bc.hook_beam_dz_match_tolerance;
       if(!shape)continue; // Different visible faces are not comparable surface columns.
       ++matched;
       const double body=(low+high)*.5;
       // A sloped body sliding in XY can change world-cell Z without rising.
       // Current body-Z translation corroborates thick cells; thin faces also need same-Sheave motion.
       const bool body_corroborates=body_window*body>0 && std::abs(body_window)>eps &&
         std::abs(body_window-body)<=bc.hook_beam_dz_match_tolerance;
       const bool hook_corroborates=corroborating_hook && hook_window*body>0 &&
         std::abs(hook_window)>eps && std::abs(hook_window-body)<=bc.hook_beam_dz_match_tolerance;
       const bool attached_motion=body_corroborates && (thick || hook_corroborates);
       if(shape && attached_motion && low>eps && high>eps && (thick || hook_window>eps))rise_dz.push_back(body);
       if(shape && attached_motion && low<-eps && high<-eps && (thick || hook_window<-eps))dz.push_back(body);
     }
   }
   evidence["surface_motion_window_sec"]=surface_window;
   evidence["surface_hook_window_m"]=hook_window;
   evidence["surface_body_median_window_m"]=body_window;
   bool current_changed=columns.size()!=previous_surface.size();
   if(!current_changed)for(const auto& item:columns){auto old=previous_surface.find(item.first);if(old==previous_surface.end() || old->second.low!=item.second.low || old->second.high!=item.second.high){current_changed=true;break;}}
   const bool up=current_changed && matched>=size_t(std::max(1,min_columns)) && rise_dz.size()>=.75*matched;
   bool down=current_changed && matched>=size_t(std::max(1,min_columns)) && dz.size()>=.75*matched;
   double delta=0;
   if(down){std::sort(dz.begin(),dz.end());delta=dz[dz.size()/2]*dt/surface_window;
     // Body descent is independently measured. Sheave motion corroborates it,
     // but a stationary/hidden Sheave cannot veto a verified Cargo surface move.
     if(current_hook && previous_hook_direct){
       evidence["lowering_hook_delta_m"]=hook_z-previous_hook_z;
       evidence["lowering_hook_corroborates"]=(hook_z-previous_hook_z<-eps &&
         std::abs((hook_z-previous_hook_z)-delta)<=bc.hook_beam_dz_match_tolerance);
     }
   }
   bool substantial_down=false,substantial_up=false;
   if(down){lower_distance+=-delta;++lower_steps;
     bool substantial=-delta>bc.lower_dz_threshold ||
       (bc.cumulative_motion_enabled && lower_distance>=cfg["lower_min_translation_m"].template get<double>() &&
        lower_steps>=bc.cumulative_motion_min_frames);
     substantial_down=substantial;++lower_count;
   }else{lower_count=0;if(up || !current_changed){lower_steps=0;lower_distance=0;}else lower_distance*=.95;}
   double rise_delta=0;
   if(up){std::sort(rise_dz.begin(),rise_dz.end());rise_delta=rise_dz[rise_dz.size()/2]*dt/surface_window;
     rise_distance+=rise_delta;++rise_steps;
     const bool substantial=rise_delta>bc.lift_dz_threshold ||
       (bc.cumulative_motion_enabled && rise_distance>=cfg["lift_min_rise_m"].template get<double>() &&
        rise_steps>=bc.cumulative_motion_min_frames);
     substantial_up=substantial;++rise_count;
   }else{rise_count=0;if(down || !current_changed){rise_steps=0;rise_distance=0;}else rise_distance*=.95;}
   evidence["surface_all_matched_columns"]=all_matched;
   evidence["lifting_up_columns"]=rise_dz.size();evidence["lifting_current_delta_m"]=rise_delta;
   evidence["lifting_accum_m"]=rise_distance;evidence["lifting_confirm_count"]=rise_count;
   evidence["lowering_matched_columns"]=matched;evidence["lowering_down_columns"]=dz.size();
   evidence["lowering_current_delta_m"]=delta;evidence["lowering_accum_m"]=lower_distance;
   evidence["lowering_confirm_count"]=lower_count;
   evidence["lowering_without_current_sheave"]=down && !current_hook;
   if(down && substantial_down && lower_count>=std::max(1,bc.lower_confirm_frames)){
     state="LOWERING";working="working";
     evidence["motion_reason"]="CURRENT_SAME_CARGO_MATCHED_SURFACE_DESCENT";
   }else if(up && substantial_up && rise_count>=cfg["confirmation_frames"].template get<int>()){
     state="LIFTING";working="lifting";
     evidence["motion_reason"]="CURRENT_SAME_CARGO_MATCHED_SURFACE_ASCENT";
   }else evidence["motion_reason"]=down?"DESCENT_CONFIRMING":up?"ASCENT_CONFIRMING":
     (matched<size_t(std::max(1,min_columns))?"INSUFFICIENT_SAME_SURFACE_COLUMNS":"NO_CONSISTENT_SURFACE_TRANSLATION");
   if(work_cargo<0 && current_hook && connected && (state=="LIFTING" || state=="LOWERING")){
     work_cargo=w.cargo_track_id;support_side=current_side;support_track=current_track;
     last_work_direct=last_connected=stamp;
   }
   surface_history.push_back({stamp,hook_z,current_hook,current_track,columns,body_median_z});
   previous_surface=std::move(columns);previous_surface_stamp=stamp;
   previous_hook_direct=current_hook;previous_hook_z=hook_z;
   evidence["state"]=state;evidence["working_status"]=working;
 }

};
