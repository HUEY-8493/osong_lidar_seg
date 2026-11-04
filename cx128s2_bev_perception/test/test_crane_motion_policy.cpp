#include <cx128s2_bev_perception/crane_pipeline.hpp>
#include <cx128s2_bev_perception/crane_hybrid_policy.hpp>
#include <cassert>
#include <iostream>
using namespace cx128s2_bev_perception;
struct Fixture {
 LatestSuspendedPolicy policy;CraneSlopeResult r;CraneWorkStatusConfig c;CraneBeamConfig bc;
 std::vector<PointXYZIRT>raw;double stamp=10;bool ordered_projection=false;
 Fixture(){c.cargo_loss_sec=1.3;c.cargo_roi_width_x_m=c.cargo_roi_width_y_m=1.2;c.cargo_confirm_sec=2;
  auto&w=r.work_status;w.cargo_track_id=1;w.cargo_cluster_id=70;w.cargo_observation="DIRECT";w.cargo_side="RIGHT";
  w.geometry_locked=w.work_status=true;w.confirmed_observation_sec=3;w.length=.8;w.width=.4;w.height=.2;
  w.extension_observation="DIRECT";w.right_hook.hook_detected=true;w.right_hook.hook_observation="VIRTUAL_DIRECT";
  auto&h=r.top_down_hook.right_housing;h.sheave.status=CraneSheaveDetectionStatus::DIRECT_OBSERVED;h.sheave.points_from_current_frame=true;
  r.top_down_hook.right_housing_memory.confirmed=true;r.top_down_hook.right_housing_memory.track_id=5;
  scene(1,2.1);
 }
 void scene(double bottom,double hook){raw.clear();r.ground.clear();auto&w=r.work_status;w.extended_raw_indices.clear();
  for(int i=0;i<20;++i)for(int j=0;j<20;++j){PointXYZIRT p;p.x=-.5+i*.05;p.y=-.5+j*.05;p.z=0;r.ground.push_back(raw.size());raw.push_back(p);}
  for(int i=0;i<21;++i)for(int j=0;j<11;++j)for(int k=0;k<5;++k){PointXYZIRT p;p.x=-.4+i*.04;p.y=-.2+j*.04;p.z=bottom+k*.05;w.extended_raw_indices.push_back(raw.size());raw.push_back(p);}
  w.cargo_z=bottom+.1;w.right_hook.hook_z=hook;r.top_down_hook.right_housing.sheave.outer_center_z=hook+.9;
 }
 std::string step(double dt=.1){stamp+=dt;if(ordered_projection)r.work_status.work_status=true;policy.process(raw,r,c,stamp);if(ordered_projection)r.work_status.work_status=false;policy.processMotionAndCompletion(raw,r,c,bc,stamp);return policy.state;}
 void warm(){for(int i=0;i<5;++i)step();assert(policy.state=="SUSPENDED");}
};
int main(){
 for(double gap:{0.,.09,.10,.11,.30,.40}){Fixture f;f.scene(gap,gap+1.1);for(int i=0;i<5;++i)f.step();assert((f.policy.state=="SUSPENDED")== (gap>=.10));}
 {Fixture f;f.warm();bool lower=false;for(int i=1;i<=12;++i){f.scene(1-i*.025,2.1-i*.025);lower|=f.step()=="LOWERING";}assert(lower);f.scene(.7,1.8);for(int i=0;i<4;++i)f.step();assert(f.policy.state=="SUSPENDED");assert(f.policy.event_started<0);}
 {Fixture f;f.warm();bool lower=false;for(int i=1;i<=12;++i){f.scene(1-i*.025,2.1);f.r.work_status.right_hook.hook_detected=false;f.r.top_down_hook.right_housing.sheave.points_from_current_frame=false;lower|=f.step()=="LOWERING";}assert(lower);}
 {Fixture f;f.warm();for(int i=1;i<12;++i){f.scene(1,2.1-i*.025);assert(f.step()!="LOWERING");}}
 {Fixture f;f.warm();for(int i=1;i<12;++i){f.scene(1-i*.02,2.1-i*.02);f.r.work_status.cargo_observation="HELD";assert(f.step()!="LOWERING");}}
 {Fixture f;f.warm();f.r.work_status.cargo_observation="HELD";for(int i=0;i<12;++i){f.step();assert(f.policy.state!="LOWERING_CONFIRMATION");}f.step();assert(f.policy.state=="LOWERING_CONFIRMATION");assert(f.policy.completion_reason=="LOSS_TIMEOUT");for(int i=0;i<29;++i)assert(f.step()=="LOWERING_CONFIRMATION");for(int i=0;i<4;++i)f.step();assert(f.policy.state=="IDLE");}
 {Fixture f;f.warm();f.r.work_status.cargo_observation="NONE";f.r.work_status.work_status=false;f.r.work_status.geometry_locked=false;f.r.work_status.cargo_track_id=-1;f.step(4);assert(f.policy.state!="LOWERING_CONFIRMATION");for(int i=0;i<12;++i){f.step();assert(f.policy.state!="LOWERING_CONFIRMATION");}f.step();assert(f.policy.completion_reason=="LOSS_TIMEOUT");}
 {Fixture f;f.r.work_status.cargo_observation="NONE";f.r.work_status.geometry_locked=false;for(int i=0;i<40;++i)assert(f.step()!="LOWERING_CONFIRMATION");}
 {Fixture f;f.warm();f.r.work_status.beam_ground_clearance_valid=true;f.r.work_status.beam_ground_clearance_m=.38;for(int i=0;i<2;++i)assert(f.step()!="LOWERING_CONFIRMATION");for(int i=0;i<3;++i)f.step();assert(f.policy.state=="LOWERING_CONFIRMATION");assert(f.policy.completion_reason=="GROUND_CONTACT");for(int i=0;i<40;++i)f.step();assert(f.policy.state=="IDLE");}
 {Fixture f;f.warm();f.r.work_status.beam_ground_clearance_valid=false;f.r.work_status.beam_ground_clearance_m=0;for(int i=0;i<10;++i)assert(f.step()!="LOWERING_CONFIRMATION");}
 {Fixture f;f.warm();f.r.work_status.right_hook.hook_x=3;for(int i=0;i<5;++i)f.step();assert(f.policy.state=="LOWERING_CONFIRMATION");assert(f.policy.completion_reason=="HOOK_DETACHED");}
 {Fixture f;f.warm();f.r.work_status.right_hook.hook_detected=false;for(int i=0;i<10;++i)assert(f.step()!="LOWERING_CONFIRMATION");}
 {Fixture f;f.warm();for(int i=1;i<10;++i){f.scene(1,2.1);f.r.work_status.cargo_track_id=i+1;assert(f.step()!="LOWERING");}}
 {Fixture f;f.warm();for(int i=1;i<12;++i){f.scene(1+(i%2)*.001,2.1+(i%2)*.001);assert(f.step()!="LOWERING");}}
 {Fixture f;f.warm();for(int i=0;i<10;++i){f.scene(1,2.1);auto&ids=f.r.work_status.extended_raw_indices;ids.erase(std::remove_if(ids.begin(),ids.end(),[&](int k){return f.raw[k].z>1.12;}),ids.end());assert(f.step()!="LOWERING");}}
 {Fixture f;f.warm();f.step(-5);assert(f.policy.state!="LOWERING"&&f.policy.state!="LOWERING_CONFIRMATION");}
 {Fixture f;f.warm();bool lower=false;for(int i=1;i<=12;++i){f.scene(1-i*.025,2.1-i*.025);lower|=f.step()=="LOWERING";}assert(lower);f.r.work_status.beam_ground_clearance_valid=true;f.r.work_status.beam_ground_clearance_m=.38;for(int i=0;i<5;++i)f.step();assert(f.policy.state=="LOWERING_CONFIRMATION" && f.policy.completion_reason=="GROUND_CONTACT");}
 {Fixture f;f.warm();f.step(-5);for(int i=0;i<5;++i)f.step();assert(f.policy.state=="SUSPENDED");assert(f.policy.completion_reason.empty());}
 {Fixture f;f.ordered_projection=true;f.warm();bool down=false;for(int i=1;i<=12;++i){f.scene(1-i*.025,2.1-i*.025);down|=f.step()=="LOWERING";}assert(down);f.r.work_status.cargo_observation="HELD";for(int i=0;i<14;++i)f.step();assert(f.policy.state=="LOWERING_CONFIRMATION");}
 {Fixture f;f.warm();bool lower=false;for(int i=1;i<=12;++i){f.scene(1-i*.02,2.1-i*.02);f.r.ground.clear();f.raw.erase(f.raw.begin(),f.raw.begin()+400);for(int&k:f.r.work_status.extended_raw_indices)k-=400;lower|=f.step()=="LOWERING";}assert(lower);assert(f.policy.completion_reason.empty());}
 {Fixture f;f.warm();bool down=false;for(int i=1;i<=12;++i){f.scene(1-i*.025,2.1);down|=f.step()=="LOWERING";}assert(down);}

 {Fixture f;f.warm();bool rising=false;for(int i=1;i<=40;++i){f.scene(1+i*.005,2.1+i*.005);rising|=f.step()=="LIFTING";}assert(rising);
  f.scene(1.2,2.3);f.step();assert(f.policy.state=="SUSPENDED");}
 {Fixture f;f.warm();bool rising=false;f.r.work_status.right_hook.hook_detected=false;
  for(int i=1;i<=12;++i){f.scene(1+i*.025,2.1);rising|=f.step()=="LIFTING";}assert(rising);}
 {Fixture f;f.warm();for(int i=1;i<=15;++i){f.scene(1,2.1+i*.025);assert(f.step()!="LIFTING");}}
 // Changing from a full section to its top face is an observation change, not ascent.
 {Fixture f;f.warm();for(int i=1;i<=12;++i){f.scene(1,2.1);auto & ids=f.r.work_status.extended_raw_indices;
  ids.erase(std::remove_if(ids.begin(),ids.end(),[&](int k){return f.raw[k].z<1.12;}),ids.end());assert(f.step()!="LIFTING");}}
 // No previous public work is needed to measure an actual confirmed body's ascent.
 {Fixture f;f.scene(.05,1.15);for(int i=0;i<5;++i)f.step();bool rising=false;
  for(int i=1;i<=15;++i){f.scene(.05+i*.025,1.15+i*.025);rising|=f.step()=="LIFTING";}assert(rising);}
 // Add many measured but vertically thin top cells: they cannot veto independently matched thick cells.
 {Fixture f;f.warm();bool lowering=false;for(int i=1;i<=15;++i){f.scene(1-i*.025,2.1-i*.025);
  for(int a=0;a<40;++a)for(int b=0;b<20;++b){PointXYZIRT p;p.x=-1+a*.05;p.y=-.8+b*.05;p.z=1.2-i*.025;
   f.r.work_status.extended_raw_indices.push_back(f.raw.size());f.raw.push_back(p);}lowering|=f.step()=="LOWERING";}assert(lowering);}
 {Fixture f;f.scene(.05,1.15);for(int i=0;i<10;++i)f.step();f.r.work_status.cargo_observation="HELD";
  for(int i=0;i<20;++i)assert(f.step()!="LOWERING_CONFIRMATION");}

 // A thin, measured body needs independent same-Sheave motion, never KF motion alone.
 {Fixture f;auto thin=[&](double z,double hook){f.scene(z,hook);auto&ids=f.r.work_status.extended_raw_indices;ids.erase(std::remove_if(ids.begin(),ids.end(),[&](int k){return f.raw[k].z>z+.001;}),ids.end());};
  thin(1,2.1);for(int i=0;i<5;++i)f.step();bool up=false;for(int i=1;i<=40;++i){thin(1+i*.005,2.1+i*.005);up|=f.step()=="LIFTING";}assert(up);}
 {Fixture f;f.warm();for(int i=1;i<=30;++i){f.scene(1,2.1+i*.02);assert(f.step()!="LIFTING");}}
 {Fixture f;f.warm();f.r.top_down_hook.right_housing_memory.track_id=9;for(int i=0;i<8;++i){f.scene(1,2.1);assert(f.step()=="SUSPENDED");}assert(f.policy.support_track==9);}

 {Fixture f;f.warm();bool lower=false;for(int i=1;i<=20;++i){f.scene(1-i*.005,2.1-i*.005);lower|=f.step()=="LOWERING";}assert(lower);}
 {Fixture f;f.warm();for(int i=1;i<=25;++i){f.scene(1-i*.002,2.1-i*.002);assert(f.step()!="LOWERING");}}

 {Fixture f;f.warm();f.r.work_status.height=1.0;
  for(int i=0;i<50;++i){f.scene(1,2.1);double shift=-i*.01;for(int k:f.r.work_status.extended_raw_indices){f.raw[k].z+=f.raw[k].x;f.raw[k].x+=shift;}f.r.work_status.cargo_x=shift;
   const auto state=f.step();assert(state!="LIFTING" && state!="LOWERING");}}

 {Fixture f;f.warm();f.r.work_status.height=1.0;
  for(int i=0;i<50;++i){f.scene(1,2.1+i*.01);double shift=-i*.01;for(int k:f.r.work_status.extended_raw_indices){f.raw[k].z+=f.raw[k].x;f.raw[k].x+=shift;}f.r.work_status.cargo_x=shift;
   const auto state=f.step();assert(state!="LIFTING" && state!="LOWERING");}}

 // Active completion projection survives repeated sensor stamps without new votes.
 {Fixture f;f.warm();f.r.work_status.beam_ground_clearance_valid=true;f.r.work_status.beam_ground_clearance_m=.38;
  for(int i=0;i<5;++i)f.step();assert(f.policy.state=="LOWERING_CONFIRMATION");auto started=f.policy.event_started;
  for(int i=0;i<4;++i){assert(f.step(0)=="LOWERING_CONFIRMATION");assert(f.policy.event_started==started);}
  f.step(-5);assert(f.policy.event_started<0 && f.policy.completion_reason.empty());}
 // All public working states arm completion, including without recorded lowering history.
 for(int kind=0;kind<3;++kind){Fixture f;f.warm();
  if(kind==1)for(int i=1;i<=30;++i){f.scene(1+i*.005,2.1+i*.005);f.step();}
  if(kind==2)for(int i=1;i<=30;++i){f.scene(1-i*.005,2.1-i*.005);f.step();}
  assert(f.policy.state==(kind==0?"SUSPENDED":kind==1?"LIFTING":"LOWERING"));
  f.r.work_status.beam_ground_clearance_valid=true;f.r.work_status.beam_ground_clearance_m=.38;
  for(int i=0;i<5;++i)f.step();assert(f.policy.state=="LOWERING_CONFIRMATION");
  for(int i=0;i<35;++i)f.step();assert(f.policy.state=="IDLE");
  f.r.work_status.cargo_track_id=2;f.r.work_status.beam_ground_clearance_valid=false;f.scene(1,2.1);
  for(int i=0;i<6;++i)f.step();assert(f.policy.state=="SUSPENDED");assert(f.policy.work_cargo==2);}

 for(int reason=0;reason<3;++reason){Fixture f;f.warm();
  if(reason==0)f.r.work_status.cargo_observation="HELD";
  if(reason==1){f.r.work_status.beam_ground_clearance_valid=true;f.r.work_status.beam_ground_clearance_m=.38;}
  if(reason==2)f.r.work_status.right_hook.hook_x=3;
  for(int i=0;i<15 && f.policy.state!="LOWERING_CONFIRMATION";++i)f.step();
  assert(f.policy.state=="LOWERING_CONFIRMATION");auto event=f.policy.event_started;
  assert(f.step(0)=="LOWERING_CONFIRMATION");assert(f.policy.event_started==event);
  for(int i=0;i<35;++i)f.step();assert(f.policy.state=="IDLE");}
 // Rewinding an active contact timer must not let its old elapsed time vote after reboot.
 {Fixture f;f.warm();f.r.work_status.beam_ground_clearance_valid=true;f.r.work_status.beam_ground_clearance_m=.38;
  f.step();f.step();f.step(-5);f.r.work_status.beam_ground_clearance_valid=false;
  for(int i=0;i<10;++i)assert(f.step()!="LOWERING_CONFIRMATION");}

 // Publicly confirmed SUSPENDED is a real work cycle even if size lock is still settling.
 for(int branch=0;branch<3;++branch){Fixture f;f.r.work_status.geometry_locked=false;f.warm();
  assert(f.policy.work_cargo==1);
  if(branch==0){f.r.work_status.cargo_observation="NONE";f.r.work_status.cargo_track_id=-1;f.r.work_status.work_status=false;f.raw.clear();}
  if(branch==1){f.r.work_status.beam_ground_clearance_valid=true;f.r.work_status.beam_ground_clearance_m=.38;}
  if(branch==2)f.r.work_status.right_hook.hook_x=3;
  for(int i=0;i<15 && f.policy.state!="LOWERING_CONFIRMATION";++i)f.step();
  assert(f.policy.state=="LOWERING_CONFIRMATION");}
 // Geometry-unlocked confirmation alone does not arm work when public admission fails.
 {Fixture f;f.r.work_status.geometry_locked=false;f.scene(.05,1.15);
  for(int i=0;i<8;++i)f.step();assert(f.policy.work_cargo<0);
  f.r.work_status.cargo_observation="HELD";for(int i=0;i<20;++i)assert(f.step()!="LOWERING_CONFIRMATION");}
 std::cout<<"PASS: 10cm boundaries, direct descent with/without sheave, hook-only/noise/HELD/ID/visibility rejection; stop->SUSPENDED; independent LOSS/GROUND/DETACH; input outage; 3s hold/duplicate suppression; restart/rewind\n";
}
