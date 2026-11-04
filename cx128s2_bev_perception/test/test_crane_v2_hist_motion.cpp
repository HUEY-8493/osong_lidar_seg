#include "../src/v2/crane_hist_motion.hpp"
#include <iostream>
#include <stdexcept>
#define CHECK(x) do {if(!(x))throw std::runtime_error(#x);}while(0)
using namespace crane_v2;
std::vector<Point> scene(double background_shift,double cargo_z=.7) {
  std::vector<Point> p;
  for(int i=0;i<1600;++i)p.push_back({float(7+i%40*.03),float(4+i/40*.03),float(.2+background_shift),uint32_t(p.size())});
  for(int i=0;i<500;++i)p.push_back({float((i%20-10)*.025),float((i/20-12)*.015),float(cargo_z),uint32_t(p.size())});
  return p;
}
int main(){try {
  Config c;CraneHistogramMotion motion;Output o;
  for(int i=0;i<10;++i){o={};motion.update(scene(0),c,i*.1,i>0,o);CHECK(!o.crane_moving);}
  bool detected=false;
  for(int i=10;i<20;++i){o={};motion.update(scene(.03*(i-9)),c,i*.1,true,o);detected|=o.crane_moving;}
  CHECK(detected);
  o={};motion.update({},c,2.,true,o);CHECK(o.crane_moving&&!o.crane_motion_valid);
  for(int i=21;i<42;++i){o={};motion.update(scene(.3),c,i*.1,true,o);}
  CHECK(!o.crane_moving);
  CraneHistogramMotion hoist;
  for(int i=0;i<40;++i) {
    const double z=.7+i*.03;o={};o.state=i<20?"LIFTING":"LOWERING";
    o.cargo.id=1;o.cargo.center={0,0,z};o.cargo.length=2.;o.cargo.width=.6;o.cargo.height=.3;
    hoist.update(scene(0,z),c,i*.1,i>0,o);CHECK(!o.crane_moving);
    if(o.crane_motion_valid)CHECK(o.crane_hist_score<1e-8);
  }
  CraneHistogramMotion density;
  for(int i=0;i<15;++i){auto p=scene(0);if(i%2){auto copy=p;p.insert(p.end(),copy.begin(),copy.end());}o={};density.update(p,c,i*.1,i>0,o);CHECK(!o.crane_moving);}
  CraneHistogramMotion holder;
  for(int i=0;i<30;++i) {
    const double z=2.+i*.03;auto p=scene(0);o={};auto& h=o.hooks[0];
    h.direct=true;h.housing={0,0,z};h.body_min={-.12,-.12,z-.12};h.body_max={.12,.12,z+.12};
    for(int j=0;j<500;++j)p.push_back({float((j%20-10)*.005),float((j/20-12)*.005),float(z),uint32_t(p.size())});
    holder.update(p,c,i*.1,i>0,o);CHECK(!o.crane_moving);
  }
  for(int axis=0;axis<2;++axis)for(int sign:{-1,1}) {
    CraneHistogramMotion lateral;bool seen=false;
    for(int i=0;i<22;++i) {
      auto p=scene(0);const double shift=i<10?0:sign*.03*(i-9);
      for(auto& q:p)if(q.x>5) {if(axis==0)q.x+=shift;else q.y+=shift;}
      o={};lateral.update(p,c,i*.1,i>0,o);seen|=o.crane_moving;
    }
    CHECK(seen);
  }
  std::cout<<"PASS background activity/start/settling/absence hold/Cargo-only Z isolation/count normalization; no speed or direction input\n";
  return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
