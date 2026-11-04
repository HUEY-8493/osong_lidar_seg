#define main baseline_regression_main
#include "test_crane_v2.cpp"
#undef main
#include <limits>
#include "cx128s2_bev_perception/v2/length_lock.hpp"
std::vector<Point> fragment() {
  auto p=scene();p.erase(std::remove_if(p.begin(),p.end(),[](const Point&a){
    return a.z>=.99&&a.z<1.3&&std::abs(a.x+.9)>.5;
  }),p.end());return p;
}
int main() { try {
  LengthLock l;
  for(int i=0;i<20;++i) REQUIRE(l.update(1,true,true,3.,i*.1,i>0,false,2.)==0);
  REQUIRE(l.update(1,true,true,3.,2.,true,false,2.)==3.);
  REQUIRE(l.update(1,true,true,19.,2.1,true,false,2.)==3.);
  for(int i=22;i<44;++i) REQUIRE(l.update(1,true,true,1.,i*.1,true,false,2.)==3.);
  for(int i=44;i<64;++i) REQUIRE(l.update(1,true,true,4.,i*.1,true,false,2.)==3.);
  REQUIRE(l.update(1,true,true,4.,6.4,true,false,2.,true,true)==4.);
  REQUIRE(l.update(1,false,true,0,6.5,true,false,2.)==4.);
  REQUIRE(l.update(1,true,true,10.,8.,false,false,2.)==4.);
  REQUIRE(l.update(2,true,true,1.,8.1,true,false,2.)==0);
  REQUIRE(l.update(2,true,true,1.,10.2,false,false,2.)==0);
  REQUIRE(l.update(2,true,true,1.,10.3,true,true,2.)==0);
  LengthLock cycle;
  for(int i=0;i<40;++i) REQUIRE(cycle.update(1,true,true,19.,i*.1,i>0,false,2.,false)==0);
  for(int i=40;i<60;++i) REQUIRE(cycle.update(1,true,true,3.,i*.1,true,false,2.,true)==0);
  REQUIRE(cycle.update(1,true,true,3.,6.,true,false,2.,true)==3.);
  REQUIRE(cycle.update(1,true,true,1.,6.1,true,false,2.,false)==3.);
  REQUIRE(cycle.update(1,true,true,1.,6.2,true,false,2.,true)==3.);
  Engine e; Output o;
  for(int i=0;i<20;++i) {o=run(e,scene(),i);REQUIRE(!o.cargo.length_locked);}
  double first_work=-1;
  e.reset();
  for(int i=0;i<50;++i) {
    o=run(e,scene(),i);
    if(o.work_active&&first_work<0)first_work=i*.1;
    if(!o.cargo.confirmed)REQUIRE(!o.cargo.length_locked);
    if(o.work_active)REQUIRE(o.cargo.length_locked);
  }
  REQUIRE(o.cargo.length_locked);double locked=o.cargo.model_length;
  for(int i=50;i<120;++i) {
    o=run(e,fragment(),i); REQUIRE(o.cargo.length_locked);
    REQUIRE(o.cargo.model_length==locked);
    REQUIRE(o.measured_cargo_length<1.1);REQUIRE(o.cargo_measurement_direct);
  }
  e.reset();o=run(e,scene(),120);REQUIRE(!o.cargo.length_locked);
  Engine idle;
  bool measured_idle=false;
  for(int i=0;i<50;++i) {
    o=run(idle,scene(true,true,false),i);REQUIRE(o.state=="IDLE");
    REQUIRE(!o.cargo.length_locked);measured_idle|=o.cargo_measurement_direct;
  }
  REQUIRE(measured_idle);
  auto idle_short=scene(true,true,false);
  idle_short.erase(std::remove_if(idle_short.begin(),idle_short.end(),[](const Point&a){
    return a.z>=.99&&a.z<1.3&&std::abs(a.x+.9)>.5;
  }),idle_short.end());
  o=run(idle,idle_short,50);REQUIRE(o.state=="IDLE");
  REQUIRE(o.cargo.direct);REQUIRE(!o.cargo.length_locked);REQUIRE(o.cargo.length<1.1);
  Engine gapped;int first_confirmed=-1;
  for(int i=0;i<40;++i) {
    const bool visible=i!=2&&i!=5&&i!=10;
    o=run(gapped,scene(visible),i);
    if(o.cargo.confirmed&&first_confirmed<0)first_confirmed=i;
  }
  REQUIRE(first_confirmed>=26);REQUIRE(first_confirmed<40);
  REQUIRE(o.state=="SUSPENDED");REQUIRE(o.cargo.length_locked);
  for(double v:{0.,-1.,std::numeric_limits<double>::quiet_NaN()}) {
    Config c;c.cargo_length_confirm_sec=v;bool rejected=false;
    try{Engine bad(c);}catch(const std::invalid_argument&){rejected=true;}REQUIRE(rejected);
  }
  std::cout<<"PASS 2s lock, spike rejection, non-shrinking, sustained growth, missing/gap, new ID/completion/reset, actual fragment separation\n";
  return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
