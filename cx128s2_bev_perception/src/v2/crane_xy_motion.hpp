#pragma once
#include "cx128s2_bev_perception/v2/core.hpp"
#include <algorithm>
#include <cmath>
#include <deque>
#include <unordered_set>
namespace crane_v2 {
// Independent XY evidence from the two calibrated upper crane-body roots.
// Cargo, Hook motion and vertical FSM states are not movement inputs.
class CraneXYMotion {
  struct Sample { double stamp; std::array<Vec3,2> root; };
  std::deque<Sample> history_;
  std::array<Vec3,2> anchors_{};
  bool anchored_=false, moving_=false;
  double start_since_=-1, last_stamp_=-1;
  static double median(std::vector<double>& v) {
    auto m=v.begin()+v.size()/2;std::nth_element(v.begin(),m,v.end());return *m;
  }
public:
  void update(const std::vector<Point>& points, const Config& c, double stamp,
              bool continuous, Output& o) {
    if(!c.crane_xy_enabled) { *this=CraneXYMotion{};o.crane_motion_reason="DISABLED";return; }
    std::unordered_set<uint32_t> excluded(o.cargo.raw_indices.begin(),o.cargo.raw_indices.end());
    excluded.insert(o.measured_cargo_raw_indices.begin(),o.measured_cargo_raw_indices.end());
    std::array<std::array<std::vector<double>,3>,2> raw;
    for(const auto& p:points) {
      if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)||excluded.count(p.raw))continue;
      bool hook=false;
      for(const auto& h:o.hooks) if(h.direct||h.held) {
        if(std::hypot(p.x-h.housing.x,p.y-h.housing.y)<.35&&std::abs(p.z-h.housing.z)<.5)hook=true;
      }
      if(hook)continue;
      for(int i=0;i<2;++i) {
        const auto& r=c.crane_root_rois[i];
        if(p.x<r[0]||p.x>r[1]||p.y<r[2]||p.y>r[3]||p.z<r[4]||p.z>r[5])continue;
        raw[i][0].push_back(p.x);raw[i][1].push_back(p.y);raw[i][2].push_back(p.z);
      }
    }
    bool supported=true;
    for(int i=0;i<2;++i) {
      o.crane_root_points[i]=raw[i][0].size();
      if(raw[i][0].size()<size_t(c.crane_xy_min_points)) {supported=false;continue;}
      o.crane_root_observed[i]=true;
      o.crane_roots[i]={median(raw[i][0]),median(raw[i][1]),median(raw[i][2])};
    }
    if(!continuous||!supported||last_stamp_<0||stamp-last_stamp_>.5) {
      history_.clear();anchored_=false;start_since_=-1;
    }
    last_stamp_=stamp;
    o.crane_moving=moving_;
    if(!supported) {o.crane_motion_reason="BODY_ROOTS_UNOBSERVED_HOLD";return;}
    o.crane_motion_valid=true;
    if(!anchored_) {anchors_=o.crane_roots;anchored_=true;}
    history_.push_back({stamp,o.crane_roots});
    const double span=c.crane_xy_stop_confirm_sec;
    while(history_.size()>1&&history_[1].stamp<=stamp-span+1e-8)history_.pop_front();
    auto speed=history_.end()-1;
    for(auto it=history_.begin();it!=history_.end();++it)
      if(stamp-it->stamp>=.2) {speed=it;break;}
    const double dt=stamp-speed->stamp;
    std::array<double,2> dx{},dy{},vx{},vy{};
    bool evidence=dt>=.2-1e-8;
    for(int i=0;i<2;++i) {
      const auto& p=o.crane_roots[i];const auto& a=anchors_[i];const auto& prev=speed->root[i];
      dx[i]=p.x-a.x;dy[i]=p.y-a.y;
      if(dt>0) {vx[i]=(p.x-prev.x)/dt;vy[i]=(p.y-prev.y)/dt;}
      evidence=evidence&&std::hypot(dx[i],dy[i])>=c.crane_xy_start_m&&
        std::hypot(vx[i],vy[i])>=c.crane_xy_min_speed&&std::abs(p.z-prev.z)<.03;
    }
    evidence=evidence&&dx[0]*dx[1]+dy[0]*dy[1]>0&&vx[0]*vx[1]+vy[0]*vy[1]>0&&
      std::hypot(dx[0]-dx[1],dy[0]-dy[1])<=std::max(.025,c.crane_xy_start_m);
    if(!moving_) {
      if(evidence) {
        if(start_since_<0)start_since_=stamp;
        if(stamp-start_since_+1e-8>=c.crane_xy_start_confirm_sec)moving_=true;
      }else start_since_=-1;
    }else if(stamp-history_.front().stamp+1e-8>=span) {
      bool stable=true;
      for(int i=0;i<2;++i) {
        double xmin=1e9,xmax=-1e9,ymin=1e9,ymax=-1e9;
        for(const auto& h:history_) {
          xmin=std::min(xmin,h.root[i].x);xmax=std::max(xmax,h.root[i].x);
          ymin=std::min(ymin,h.root[i].y);ymax=std::max(ymax,h.root[i].y);
        }
        stable=stable&&xmax-xmin<=c.crane_xy_stop_span_m&&ymax-ymin<=c.crane_xy_stop_span_m;
      }
      if(stable) {moving_=false;anchors_=o.crane_roots;start_since_=-1;history_.clear();}
    }
    o.crane_moving=moving_;
    o.crane_motion_reason=moving_?"COHERENT_BODY_XY":"BODY_XY_STATIONARY";
  }
};
}
