#include "cx128s2_bev_perception/v2/core.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <chrono>
#include <sys/resource.h>
using J=nlohmann::json;
J vec(crane_v2::Vec3 p){return J::array({p.x,p.y,p.z});}
J describe(const crane_v2::Output&o){J hooks=J::array();for(auto&h:o.hooks)hooks.push_back({{"direct",h.direct},{"held",h.held},{"housing",vec(h.housing)},{"virtual_hook",vec(h.virtual_hook)},{"points",h.points},{"confidence",h.confidence}});
 return {{"stamp_ns",o.stamp_ns},{"state",o.state},{"connection_observation",o.connection_observation},{"reason",o.reason},{"work_complete",o.work_complete},{"cargo_id",o.cargo.id},{"direct",o.cargo.direct},{"held",o.cargo.held},{"confirmed",o.cargo.confirmed},{"connected",o.cargo.connected},{"center",vec(o.cargo.center)},{"dimensions",{o.cargo.length,o.cargo.width,o.cargo.height,o.cargo.yaw}},{"model_length",o.cargo.model_length},{"body_points",o.cargo.raw_indices.size()},{"bottom_valid",o.cargo.bottom_valid},{"bottom_observation",o.cargo.bottom_observation},{"measured_section_depth",o.cargo.measured_section_depth},{"bottom_z",o.cargo.bottom_z},{"ground_valid",o.ground.valid},{"ground_observation",o.ground.observation},{"ground_z",o.ground.z},{"ground_points",o.ground.raw_indices.size()},{"clearance_valid",o.clearance_valid},{"clearance",o.clearance},{"motion_dz",o.motion_dz},{"matched_fraction",o.matched_fraction},{"hooks",hooks},{"input_points",o.input_points},{"cells",o.voxel_cells},{"components",o.components},{"candidates",o.candidates},{"stage_ms",o.stage_ms},{"candidate_metrics",o.candidate_metrics},{"ground_supported",o.ground_supported},{"ground_support_held",o.ground_support_held},{"contact_ground_direct_valid",o.contact_ground_direct_valid}};
}
#ifndef V2_NO_MAIN
int main(int argc,char**argv){if(argc<3)return 2;std::ifstream in(argv[1],std::ios::binary);std::ofstream out(argv[2]);if(!in||!out)return 3;crane_v2::Engine engine;int64_t stamp;uint32_t n;size_t frame=0;std::vector<crane_v2::Point>points;auto begin=std::chrono::steady_clock::now();
 while(in.read((char*)&stamp,8)){if(!in.read((char*)&n,4)||n>10000000)return 4;points.resize(n);if(!in.read((char*)points.data(),n*sizeof(points[0])))return 4;
 auto t=std::chrono::steady_clock::now();auto o=engine.process(points,stamp);double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-t).count();J j=describe(o);j["frame"]=++frame;j["core_ms"]=ms;out<<j.dump()<<'\n';
 if(frame%500==0)std::cerr<<"processed="<<frame<<std::endl;
 if(frame==1||frame==76||frame==117||frame==207||frame==300||frame==40||frame==479||frame==482||frame==588||frame==592||frame==4528||frame==4800||frame==4918||frame==4948){std::ofstream raw(std::string(argv[2])+".F"+std::to_string(frame)+".json");raw<<J({{"support_path_raw",o.support_path_raw},{"cargo_raw",o.cargo.raw_indices},{"ground_raw",o.ground.raw_indices}}).dump();}
 }
 rusage ru{};getrusage(RUSAGE_SELF,&ru);std::ofstream(std::string(argv[2])+".summary.json")<<J({{"frames",frame},{"cpu_sec",ru.ru_utime.tv_sec+ru.ru_utime.tv_usec*1e-6+ru.ru_stime.tv_sec+ru.ru_stime.tv_usec*1e-6},{"peak_rss_kb",ru.ru_maxrss},{"wall_sec",std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count()}}).dump(2);
}
#endif
