#include "cx128s2_bev_perception/general_ground_noise_filter.hpp"
#include <iostream>
#include <stdexcept>
using namespace cx128s2_bev_perception;
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
int main(){
 GeneralGroundNoiseConfig config;GeneralGroundNoiseFilter filter;
 std::vector<PointXYZIRT> points;
 auto add=[&](float z,int ring,float x=.1f){PointXYZIRT p;p.x=x;p.y=.1f;p.z=z;p.ring=ring;points.push_back(p);};
 for(int i=0;i<100;++i)add(0,i%10);
 for(int i=0;i<10;++i)add(.4f,34+i%2);
 auto run=[&](bool rings=true){std::vector<int> foreground;for(size_t i=100;i<points.size();++i)foreground.push_back(i);std::vector<int> removed;auto stats=filter.filter(points,foreground,removed,config,rings,[](auto&){return true;});require(stats.removed_foreground==removed.size(),"count mismatch");return removed.size();};
 require(run()==10,"supported low-ring artifacts must be removed");
 require(run(false)==0,"missing ring must pass");
 config.enabled=false;require(run()==0,"disabled must pass");config.enabled=true;
 for(int i=0;i<9;++i)add(2,i%3,.6f);
 require(run()==0,"nearby observed body must protect low layer");points.resize(110);
 points[0].ring=300;require(run()==0,"unsupported beam must pass");points[0].ring=0;
 for(size_t i=0;i<100;++i)points[i].z=-1;
 require(run()==0,"unobserved floor must pass");
 for(size_t i=0;i<100;++i)points[i].z=0;
 for(size_t i=100;i<110;++i)points[i].ring=i-100;
 require(run()==0,"broad beam surface must be preserved");
 config.cell_m=0;bool rejected=false;try{config.validate();}catch(const std::invalid_argument&){rejected=true;}require(rejected,"invalid configuration must fail");
 std::cout<<"ground noise evidence and passthrough tests passed\n";
}
