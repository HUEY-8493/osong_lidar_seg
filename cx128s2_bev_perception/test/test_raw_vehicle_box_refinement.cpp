#include <cx128s2_bev_perception/raw_vehicle_box_refinement.hpp>
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <string>
using namespace cx128s2_bev_perception;
using namespace cx128s2_bev_perception::raw_box;
static PointXYZIRT point(float x,float y,float z) { PointXYZIRT p;p.x=x;p.y=y;p.z=z;return p; }
int main(int argc,char ** argv) {
  Workspace workspace;
  Query rotated{7,10,20,0,1,3,1,1,4};
  std::vector<PointXYZIRT> points{point(10,22,2),point(10.5,18,3),point(12,20,2),
      point(10,20,.5),point(10,20,5),point(10,20,2)};
  std::vector<int> owners{7,-1,-1,-1,-1,8};
  auto stats=collect(points,owners,{rotated},[](const auto &){return true;},workspace);
  assert(stats.scanned_points==6 && stats.boxed_points==2);
  assert(workspace.observations[0].points==2 && workspace.observations[0].foreground_points==1);
  assert(workspace.observations[0].min_z==2 && workspace.observations[0].max_z==3);
  collect(points,owners,{rotated},[](const auto & p){return p.x<10.25;},workspace);
  assert(workspace.observations[0].points==1);
  std::vector<int> roi_indices{0,3,5};
  stats=collect(points,owners,{rotated},[](const auto &){return true;},workspace,&roi_indices);
  assert(stats.scanned_points==3 && stats.boxed_points==1);
  roi_indices.clear();
  stats=collect(points,owners,{rotated},[](const auto &){return true;},workspace,&roi_indices);
  assert(stats.scanned_points==0 && stats.boxed_points==0);
  std::vector<Query> overlap{{1,0,0,1,0,3,1,1,4},{2,1,0,1,0,3,1,1,4}};
  points={point(.1,0,2),point(.9,0,2),point(.4,0,2),point(.6,0,2)};
  owners={2,1,-1,-1};
  stats=collect(points,owners,overlap,[](const auto &){return true;},workspace);
  assert(stats.boxed_points==4);
  assert(workspace.observations[0].points==2 && workspace.observations[1].points==2);
  assert(workspace.observations[0].foreground_points==1 && workspace.observations[1].foreground_points==1);
  stats=collect(points,{}, {},[](const auto &){return true;},workspace);
  assert(stats.scanned_points==0 && workspace.observations.empty());
  bool rejected=false;
  try { collect(points,{},overlap,[](const auto &){return true;},workspace); }
  catch(const std::invalid_argument &) { rejected=true; }
  assert(rejected);
  points.push_back(point(std::numeric_limits<float>::quiet_NaN(),0,2));owners.push_back(-1);
  stats=collect(points,owners,overlap,[](const auto &){return true;},workspace);
  assert(stats.boxed_points==4);
  std::cout<<"Rotated 3D crop, vertical exclusion, foreground ownership, overlap, ROI predicate and empty-work fast path pass\n";
  if(argc>1 && std::string(argv[1])=="--benchmark") {
    points.resize(136000);owners.assign(points.size(),-1);
    for(size_t i=0;i<points.size();++i)
      points[i]=point(-100.f+float(i%2000)*.1f,-12.f+float((i/2000)%240)*.1f,.05f+float(i%64)*.1f);
    for(int box_count:{1,4,32}) {
      std::vector<Query> queries;
      for(int i=0;i<box_count;++i)queries.push_back({i,-80.+5.*i,-8.,1,0,3,1.2,.5,3.5});
      std::vector<double> timings;
      size_t boxed=0;
      for(int trial=0;trial<120;++trial) {
        auto begin=std::chrono::steady_clock::now();
        auto result=collect(points,owners,queries,[](const auto &){return true;},workspace);
        const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
        boxed=result.boxed_points;
        if(trial>=20)timings.push_back(ms);
      }
      std::sort(timings.begin(),timings.end());
      std::cout<<std::fixed<<std::setprecision(3)<<"raw_scan points="<<points.size()<<" boxes="<<box_count
          <<" median_ms="<<timings[timings.size()/2]<<" p95_ms="<<timings[timings.size()*95/100]
          <<" boxed_points="<<boxed<<" (synthetic helper-only benchmark)\n";
    }
  }
}
