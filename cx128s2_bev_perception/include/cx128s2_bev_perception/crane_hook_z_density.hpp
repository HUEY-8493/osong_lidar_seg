#pragma once
#include <nlohmann/json.hpp>
#include <fstream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <stdexcept>
class HookZDensity {
public:
nlohmann::json cfg;double bin,stdmax,k,ratio,scale,depth,exclude;int minpoints;
HookZDensity(){cfg=nlohmann::json::parse(R"json({
  "bin_size_m": 0.05,
  "max_std_m": 0.1,
  "sigma_multiplier": 2.0,
  "min_points": 20,
  "min_peak_ratio": 0.2,
  "proximity_scale_m": 0.75,
  "max_depth_m": 2.5,
  "exclude_below_hook_m": 0.15,
  "exclude_ground": true,
  "ground_search_z_min_m": -0.5,
  "ground_search_z_max_m": 0.8,
  "ground_min_points": 50,
  "ground_max_std_m": 0.05,
  "ground_clearance_m": 0.15
}
)json");const char*env=std::getenv("CRANE_Z_DENSITY_CONFIG");if(env){std::ifstream f(env);if(!f)throw std::runtime_error("missing Z density config");f>>cfg;}bin=cfg.at("bin_size_m");stdmax=cfg.at("max_std_m");k=cfg.at("sigma_multiplier");minpoints=cfg.at("min_points");ratio=cfg.at("min_peak_ratio");scale=cfg.at("proximity_scale_m");depth=cfg.at("max_depth_m");exclude=cfg.at("exclude_below_hook_m");if(!(bin>0&&stdmax>0&&k>0&&minpoints>0&&ratio>0&&ratio<=1&&scale>0&&depth>exclude&&exclude>=0))throw std::runtime_error("invalid Z density config");}
template<class Points> nlohmann::json detectGround(const Points&raw,double x,double y,double z,double wx,double wy,const std::vector<int>&ground={},double ground_wx=0,double ground_wy=0){
 (void)z;
 const double gwx=ground_wx>0?ground_wx:wx,gwy=ground_wy>0?ground_wy:wy;
 bool ground_valid=!ground.empty();double ground_z=0;std::vector<double> ground_values;
 if(cfg.value("exclude_ground",false)){
  for(int id:ground)if(id>=0&&size_t(id)<raw.size()&&std::abs(raw[id].x-x)<=gwx*.5&&std::abs(raw[id].y-y)<=gwy*.5)ground_values.push_back(raw[id].z);
  ground_valid=false;
  if(ground_values.size()<size_t(cfg.value("ground_min_points",50))){
   ground_values.clear();double low=cfg.value("ground_search_z_min_m",-.5),high=cfg.value("ground_search_z_max_m",.8);std::vector<std::vector<double>> gb(size_t(std::ceil((high-low)/bin))+1);
   for(const auto&p:raw)if(std::isfinite(p.z)&&std::abs(p.x-x)<=gwx*.5&&std::abs(p.y-y)<=gwy*.5&&p.z>=low&&p.z<=high)gb[std::min(gb.size()-1,size_t((p.z-low)/bin))].push_back(p.z);
   size_t peak=0;for(size_t i=1;i<gb.size();++i)if(gb[i].size()>gb[peak].size())peak=i;
   const size_t significant=std::max(size_t(cfg.value("ground_min_points",50)),size_t(std::ceil(gb[peak].size()*ratio)));
   // Select the lowest supported local mode; a denser raised Cargo top is not Ground.
   for(size_t i=0;i<gb.size();++i)if(gb[i].size()>=significant &&
       (i==0 || gb[i].size()>=gb[i-1].size()) && (i+1==gb.size() || gb[i].size()>=gb[i+1].size())){peak=i;break;}
   for(int i=std::max(0,int(peak)-1);i<=std::min(int(gb.size())-1,int(peak)+1);++i)ground_values.insert(ground_values.end(),gb[i].begin(),gb[i].end());
  }
  if(ground_values.size()>=size_t(cfg.value("ground_min_points",50))){std::sort(ground_values.begin(),ground_values.end());ground_z=ground_values[ground_values.size()/2];double sq=0;for(double v:ground_values)sq+=(v-ground_z)*(v-ground_z);ground_valid=std::sqrt(sq/ground_values.size())<=cfg.value("ground_max_std_m",.05);}

 }
 return {{"ground_valid",ground_valid},{"ground_z",ground_z},{"ground_source",ground_valid?"CURRENT_LOCAL_Z_SURFACE":"UNKNOWN"}};
}
template<class Points> nlohmann::json detect(const Points&raw,double x,double y,double z,double wx,double wy,const std::vector<int>&ground={},double ground_wx=0,double ground_wy=0){
 std::vector<unsigned char> excluded(raw.size(),0);if(cfg.value("exclude_ground",false))for(int id:ground)if(id>=0&&size_t(id)<raw.size())excluded[id]=1;

 const auto observed_ground=detectGround(raw,x,y,z,wx,wy,ground,ground_wx,ground_wy);
 const bool ground_valid=observed_ground["ground_valid"];const double ground_z=observed_ground["ground_z"];
 if(cfg.value("exclude_ground",false) && ground_valid)for(size_t i=0;i<raw.size();++i)
   if(raw[i].z<=ground_z+cfg.value("ground_clearance_m",.15))excluded[i]=1;
 int excluded_count=0;int n=int(std::ceil(depth/bin))+1;std::vector<std::vector<int>> bins(n);int total=0;
 for(size_t i=0;i<raw.size();++i){const auto&p=raw[i];double d=z-p.z;if(excluded[i]){++excluded_count;continue;}if(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(d)&&std::abs(p.x-x)<=wx*.5&&std::abs(p.y-y)<=wy*.5&&d>=exclude&&d<=depth){bins[std::min(n-1,int(d/bin))].push_back(i);++total;}}
 nlohmann::json out={{"valid",false},{"ground_valid",ground_valid},{"ground_z",ground_z},{"ground_source",ground_valid?"CURRENT_LOCAL_Z_SURFACE":"UNKNOWN"},{"roi_points",total},{"ground_excluded_points",excluded_count},{"candidates",nlohmann::json::array()},{"histogram",nlohmann::json::array()},{"config",cfg}};if(cfg.value("exclude_ground",false)&&!ground_valid){out["reason"]="GROUND_UNKNOWN";return out;}int peak=0;for(auto&b:bins)peak=std::max(peak,int(b.size()));for(int i=0;i<n;++i)out["histogram"].push_back({z-(i+1)*bin,z-i*bin,bins[i].size()});double best=-1;
 for(int i=0;i<n;++i){int count=bins[i].size();if(count<minpoints||count<peak*ratio||(i&&bins[i-1].size()>bins[i].size())||(i+1<n&&bins[i+1].size()>=bins[i].size()))continue;
 double center=z-(i+.5)*bin;std::vector<int> ids;double mean=0,sd=0;
 for(int iter=0;iter<3;++iter){ids.clear();double radius=iter?std::max(bin*.5,k*sd):stdmax*k;for(int j=std::max(0,i-int(std::ceil(stdmax*k/bin))-1);j<std::min(n,i+int(std::ceil(stdmax*k/bin))+2);++j)for(int id:bins[j])if(std::abs(raw[id].z-center)<=radius)ids.push_back(id);if(ids.empty())break;mean=0;for(int id:ids)mean+=raw[id].z;mean/=ids.size();sd=0;for(int id:ids)sd+=(raw[id].z-mean)*(raw[id].z-mean);sd=std::sqrt(sd/ids.size());center=mean;}
 if(ids.size()<size_t(minpoints)||sd>stdmax)continue;double gap=z-mean;double score=ids.size()/std::max(bin,2*sd)/(1+gap/scale);nlohmann::json c={{"points",ids.size()},{"mean_z",mean},{"std_m",sd},{"hook_gap_m",gap},{"score",score},{"peak_points",count},{"z_min",1e9},{"z_max",-1e9}};
 for(int id:ids){c["z_min"]=std::min(c["z_min"].get<double>(),double(raw[id].z));c["z_max"]=std::max(c["z_max"].get<double>(),double(raw[id].z));}out["candidates"].push_back(c);
 if(score>best){best=score;out["selected"]=c;out["valid"]=true;out["selected_xyz"]=nlohmann::json::array();size_t stride=std::max(size_t(1),ids.size()/2500);for(size_t j=0;j<ids.size();j+=stride){const auto&p=raw[ids[j]];out["selected_xyz"].push_back({p.x,p.y,p.z});}}
 }return out;
}
};
