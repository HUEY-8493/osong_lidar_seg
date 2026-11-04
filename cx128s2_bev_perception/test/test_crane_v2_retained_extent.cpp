#include "../src/v2/retained_extent.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
#define CHECK(x) do {if(!(x))throw std::runtime_error(#x);}while(0)
using namespace crane_v2;
int main(){try{
 RetainedExtent policy;Cargo c;c.id=1;c.direct=c.confirmed=c.connected=true;
 c.supporting_side=1;c.center={1.,0.,.7};c.length=12.;c.width=.4;c.height=.3;c.yaw=.02;
 std::array<Hook,2> h{};h[1].direct=true;h[1].virtual_hook={1.,0.,1.7};
 Output o;policy.update(c,h,true,12.,false,.08,o);CHECK(o.retained_box_valid);
 c.center={3.,.3,.8};c.length=2.;c.width=.8;c.height=.7;c.yaw=.4;
 o={};policy.update(c,h,true,12.,false,.08,o);CHECK(o.retained_box.center.x==1.);
 CHECK(o.retained_box.width==.4&&o.retained_box.height==.3&&o.retained_box.yaw==.02);
 CHECK(o.retained_box.center.z==.8);CHECK(c.center.x==3.&&c.length==2.);
 h[1].virtual_hook.x+=.2;o={};policy.update(c,h,true,12.,false,.08,o);
 CHECK(std::abs(o.retained_box.center.x-1.2)<1e-8);
 c.direct=c.connected=false;h[1].direct=false;o={};policy.update(c,h,true,12.,false,.08,o);
 CHECK(std::abs(o.retained_box.center.x-1.2)<1e-8);
 c.direct=c.connected=true;h[1].direct=true;c.length=13.;c.center.x=1.5;
 o={};policy.update(c,h,true,13.,false,.08,o);CHECK(o.retained_box.center.x==1.5);
 c.center.x=3.;c.length=13.01;o={};policy.update(c,h,true,13.01,false,.08,o);
 CHECK(o.retained_box.center.x==1.5);CHECK(o.retained_box.length==13.01);
 o={};policy.update(c,h,false,13.01,false,.08,o);CHECK(o.retained_box_valid);
 o={};policy.update(c,h,true,0.,true,.08,o);CHECK(!o.retained_box_valid);
 c.id=2;o={};policy.update(c,h,true,0.,false,.08,o);CHECK(!o.retained_box_valid);
 std::cout<<"PASS fragment center/width/yaw rejection, measured Z, Hook XY translation, loss hold, larger extent, IDLE, release and new ID\n";
 return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
