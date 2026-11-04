// Reuse the existing physical surface fixtures and baseline regressions.
#define main baseline_regression_main
#include "test_crane_v2.cpp"
#undef main
#include <limits>
#include <unordered_set>
std::vector<Point> fragment(double length=1.){
 auto p=scene();p.erase(std::remove_if(p.begin(),p.end(),[&](const Point&a){return a.z>=.99&&a.z<1.3&&std::abs(a.x+.9)>length*.5;}),p.end());return p;
}
void checkHeld(const Output&o,uint64_t id,double length){
 REQUIRE(o.cargo_shrink_held);REQUIRE(o.cargo.held);REQUIRE(!o.cargo.direct);
 REQUIRE(o.cargo.id==id);REQUIRE(std::abs(o.cargo.length-length)<1e-6);
 REQUIRE(o.cargo.raw_indices.empty());REQUIRE(!o.clearance_valid);
 REQUIRE(!o.cargo.bottom_valid);REQUIRE(o.cargo.bottom_observation=="UNKNOWN");
 REQUIRE(!o.work_complete);REQUIRE(o.state=="SUSPENDED");
 REQUIRE(o.reason=="CARGO_LENGTH_SHRINK_HOLD");REQUIRE(o.motion_dz==0);
}
void shrinkWindow(){
 Engine e;Output o;for(int i=0;i<30;++i)o=run(e,scene(),i);
 const auto id=o.cargo.id;const double length=o.cargo.length;auto small=fragment();std::unordered_set<uint32_t> raw_ids;for(const auto&p:small)raw_ids.insert(p.raw);
 for(int i=30;i<60;++i){o=run(e,small,i);checkHeld(o,id,length);REQUIRE(o.cargo_measurement_direct);REQUIRE(o.measured_cargo_length<1.1);REQUIRE(o.measured_cargo_points==o.measured_cargo_raw_indices.size());for(auto raw:o.measured_cargo_raw_indices)REQUIRE(raw_ids.count(raw));}
 o=run(e,small,60);REQUIRE(!o.cargo_shrink_held);REQUIRE(o.cargo.direct);REQUIRE(o.cargo.id==id);REQUIRE(o.cargo.length<1.1);
 for(int i=61;i<100;++i){o=run(e,small,i);REQUIRE(!o.cargo_shrink_held);REQUIRE(!o.work_complete);}
 std::cout<<"PASS shrink grace exactly3s, live fragment separate, no endless timer restart\n";
}
void recoverAndReset(){
 Engine e;Output o;for(int i=0;i<30;++i)o=run(e,scene(),i);auto id=o.cargo.id;double l=o.cargo.length;
 o=run(e,fragment(),30);checkHeld(o,id,l);
 auto bad=e.process(fragment(),0);REQUIRE(bad.reason=="INVALID_SENSOR_TIMESTAMP");
 bad=run(e,fragment(),30);REQUIRE(bad.reason=="DUPLICATE_TIMESTAMP");bad=run(e,fragment(),29);REQUIRE(bad.reason=="REVERSED_TIMESTAMP");
 o=run(e,fragment(),31);checkHeld(o,id,l);REQUIRE(std::abs(o.cargo_shrink_hold_elapsed_sec-.1)<1e-6);
 o=run(e,scene(),32);REQUIRE(!o.cargo_shrink_held);REQUIRE(o.cargo.direct);REQUIRE(o.cargo.id==id);REQUIRE(o.state=="SUSPENDED");
 o=run(e,fragment(),33);checkHeld(o,id,l);
 e.reset();o=run(e,fragment(),34);REQUIRE(!o.cargo_shrink_held);REQUIRE(!o.cargo.confirmed);REQUIRE(!o.work_complete);
 Engine cold;for(int i=0;i<30;++i){o=run(cold,fragment(),i);REQUIRE(!o.cargo_shrink_held);}
 std::cout<<"PASS recovery, timestamp rejection, restart no phantom history\n";
}
void noTriggerOnOrdinaryChange(){
 Engine e;Output o;for(int i=0;i<30;++i)o=run(e,scene(),i);
 o=run(e,fragment(2.2),30);REQUIRE(!o.cargo_shrink_held);REQUIRE(o.cargo.direct);
 o=run(e,fragment(1.6),31);REQUIRE(!o.cargo_shrink_held);REQUIRE(o.cargo.direct);
 o=run(e,scene(false),32);REQUIRE(!o.cargo_shrink_held);
 Config c;c.cargo_shrink_hold_sec=0;Engine off(c);for(int i=0;i<30;++i)run(off,scene(),i);o=run(off,fragment(),30);REQUIRE(!o.cargo_shrink_held);REQUIRE(o.cargo.direct);REQUIRE(o.cargo.length<1.1);
 std::cout<<"PASS gradual reduction/pure absence do not trigger; zero disables policy\n";
}
void missingAfterShrink(){
 Engine e;Output o;for(int i=0;i<30;++i)o=run(e,scene(),i);auto id=o.cargo.id;double l=o.cargo.length;
 o=run(e,fragment(),30);checkHeld(o,id,l);
 for(int i=31;i<60;++i){o=run(e,scene(false),i);checkHeld(o,id,l);REQUIRE(!o.cargo_measurement_direct);REQUIRE(o.measured_cargo_length==0);REQUIRE(o.measured_cargo_raw_indices.empty());}
 o=run(e,scene(false),60);REQUIRE(!o.cargo_shrink_held);REQUIRE(o.work_complete);REQUIRE(o.reason=="LOSS_TIMEOUT");
 Engine gap;for(int i=0;i<30;++i)run(gap,scene(),i);o=run(gap,fragment(),30);REQUIRE(o.cargo_shrink_held);o=run(gap,fragment(),40);REQUIRE(!o.cargo_shrink_held);REQUIRE(!o.work_complete);
 std::cout<<"PASS prior shrink grace covers later occlusion only within deadline; stream gap cancels\n";
}
void newIdAndValidation(){
 Engine e;Output o;for(int i=0;i<30;++i)o=run(e,scene(),i);auto id=o.cargo.id;
 o=run(e,fragment(),30);REQUIRE(o.cargo_shrink_held);auto different=scene();for(auto&p:different)if(p.z>=.99)p.z+=1.;for(int i=31;i<60;++i){o=run(e,different,i);REQUIRE(o.cargo_shrink_held);REQUIRE(!o.cargo_measurement_direct);}
 o=run(e,different,60);REQUIRE(!o.cargo_shrink_held);REQUIRE(o.work_complete);
 o=run(e,different,61);REQUIRE(!o.cargo_shrink_held);REQUIRE(o.cargo.id!=id);REQUIRE(!o.cargo.confirmed);
 for(double v:{-1.,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}){Config c;c.cargo_shrink_hold_sec=v;bool rejected=false;try{Engine x(c);}catch(const std::invalid_argument&){rejected=true;}REQUIRE(rejected);}
 std::cout<<"PASS foreign candidate cannot restart grace; later accepted new ID has no stale box; invalid configuration rejected\n";
}
int main(){try{shrinkWindow();recoverAndReset();noTriggerOnOrdinaryChange();missingAfterShrink();newIdAndValidation();return 0;}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
