#include "cx128s2_bev_perception/general_v2/core.hpp"
#include <algorithm>
#include <iostream>
#include <stdexcept>
using namespace general_v2;
void check(bool v,const char*s){if(!v)throw std::runtime_error(s);}
int main(){
 Config cfg;std::vector<Point> p;std::vector<uint32_t> valid,fg,excluded;Ground ground;ground.valid=true;
 auto add=[&](double x,double y,double z,uint16_t ring){p.push_back({x,y,z,uint32_t(p.size()),ring});valid.push_back(p.size()-1);};
 for(int i=0;i<100;++i)add(.05+.003*(i%10),.05+.003*(i/10),0,i%10);
 for(int i=0;i<10;++i){add(.1,.1,.4,34+i%2);fg.push_back(p.size()-1);}
 auto original=fg;
 detect(p,fg,cfg,ground,valid,excluded);check(fg.empty()&&excluded.size()==10,"artifact admitted into candidate");
 cfg.candidate_ground_noise_rejection=false;fg=original;detect(p,fg,cfg,ground,valid,excluded);check(fg==original&&excluded.empty(),"disabled admission changed inputs");cfg.candidate_ground_noise_rejection=true;
 for(auto&point:p)point.ring=UINT16_MAX;
 fg=original;detect(p,fg,cfg,ground,valid,excluded);check(fg==original&&excluded.empty(),"missing beam evidence must pass");
 for(size_t i=0;i<p.size();++i)p[i].ring=i<100?i%10:34+i%2;
 for(int i=0;i<9;++i)add(.6,.1,2,i%3);
 fg=original;detect(p,fg,cfg,ground,valid,excluded);check(fg==original&&excluded.empty(),"observed upper body must protect wheels");
 p.resize(110);valid.resize(110);ground.valid=false;
 fg=original;detect(p,fg,cfg,ground,valid,excluded);check(fg==original,"unknown ground must pass");ground.valid=true;
 for(size_t i=100;i<110;++i)p[i].ring=i-100;
 fg=original;detect(p,fg,cfg,ground,valid,excluded);check(fg==original,"multi-beam low object was rejected");
 // Translation and a supported slope do not depend on a hardcoded world Z=0.
 for(size_t i=0;i<p.size();++i){auto&v=p[i];v.ring=i<100?i%10:34+i%2;v.z+=1+.1*v.x;}
 ground.a=.1;ground.c=1;fg=original;detect(p,fg,cfg,ground,valid,excluded);check(fg.empty(),"candidate admission must use observed plane coordinates");
 for(size_t i=0;i<100;++i)p[i].z-=2;
 fg=original;detect(p,fg,cfg,ground,valid,excluded);check(fg==original,"no local floor support must pass");
 std::cout<<"V2 native candidate admission tests passed\n";
}
