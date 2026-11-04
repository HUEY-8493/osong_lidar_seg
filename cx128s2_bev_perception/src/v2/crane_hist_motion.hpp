#pragma once
#include "cx128s2_bev_perception/v2/core.hpp"
#include <algorithm>
#include <cmath>
#include <deque>
namespace crane_v2 {
// Coarse scene activity only: no speed or direction is estimated.
class CraneHistogramMotion {
  struct Mask {
    Vec3 center;double x=0,y=0,z=0,co=1,si=0;
    bool contains(const Point& p)const {
      const double dx=p.x-center.x,dy=p.y-center.y;
      return std::abs(dx*co+dy*si)<=x&&std::abs(-dx*si+dy*co)<=y&&std::abs(p.z-center.z)<=z;
    }
  };
  struct Frame {double stamp=0;std::vector<Point> points;std::vector<Mask> masks;};
  Frame anchor_;
  std::deque<Frame> recent_;
  bool initialized_=false,moving_=false;
  double start_since_=-1,stop_since_=-1;
  static std::vector<Mask> masks(const Output& o) {
    std::vector<Mask> out;
    if(o.cargo.id&&o.cargo.length>0) {
      auto center=o.cargo.center;double l=std::max(o.cargo.length,o.cargo.model_length),w=o.cargo.width,h=o.cargo.height,yaw=o.cargo.yaw;
      if(o.retained_box_valid) {const auto& b=o.retained_box;center=b.center;l=b.length;w=b.width;h=b.height;yaw=b.yaw;}
      out.push_back({center,l*.5+.35,std::max(.2,w*.5)+.25,std::max(.2,h*.5)+.25,std::cos(yaw),std::sin(yaw)});
    }
    for(const auto& h:o.hooks)if(h.direct||h.held) {
      const auto& lo=h.body_min;const auto& hi=h.body_max;
      if(hi.x>lo.x&&hi.y>lo.y&&hi.z>lo.z)
        out.push_back({{(lo.x+hi.x)*.5,(lo.y+hi.y)*.5,(lo.z+hi.z)*.5},(hi.x-lo.x)*.5+.15,(hi.y-lo.y)*.5+.15,(hi.z-lo.z)*.5+.2,1,0});
      else out.push_back({h.housing,.4,.4,.5,1,0});
    }
    return out;
  }
  static bool inWatch(const Point& p,const Config& c) {
    return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z)&&
      p.x>=c.x_min&&p.x<=c.x_max&&p.y>=c.y_min&&p.y<=c.y_max&&p.z>=c.z_min&&p.z<c.z_max;
  }
  static std::array<size_t,3> bins(const Config& c) {
    const bool xy=c.crane_motion_mode=="scene_histogram";
    return {{xy?size_t(std::ceil((c.x_max-c.x_min)/c.crane_hist_xy_bin_m))+1:0,
             xy?size_t(std::ceil((c.y_max-c.y_min)/c.crane_hist_xy_bin_m))+1:0,
             size_t(std::ceil((c.z_max-c.z_min)/c.crane_hist_bin_m))+1}};
  }
  static std::vector<double> histogram(const std::vector<Point>& p,const Config& c,
                                      const std::vector<Mask>& a,const std::vector<Mask>& b,uint32_t& n) {
    const auto size=bins(c);
    std::vector<double> h(size[0]+size[1]+size[2],0.);n=0;
    for(const auto& q:p) {
      if(!inWatch(q,c))continue;
      bool excluded=false;for(const auto& m:a)if(m.contains(q)){excluded=true;break;}
      if(!excluded)for(const auto& m:b)if(m.contains(q)){excluded=true;break;}
      if(excluded)continue;
      const double coords[3]={(q.x-c.x_min)/c.crane_hist_xy_bin_m,
                              (q.y-c.y_min)/c.crane_hist_xy_bin_m,(q.z-c.z_min)/c.crane_hist_bin_m};
      size_t offset=0;
      for(int axis=0;axis<3;++axis) {
        if(size[axis]) {
          const size_t bin=std::min(size[axis]-1,size_t(coords[axis]));
          const double fraction=coords[axis]-bin;
          h[offset+bin]+=1.-fraction;
          if(bin+1<size[axis])h[offset+bin+1]+=fraction;
        }
        offset+=size[axis];
      }
      ++n;
    }
    if(n)for(auto& v:h)v/=n;
    return h;
  }
  static double difference(const std::vector<Point>& current,const std::vector<Mask>& mask,
                           const Frame& ref,const Config& c,uint32_t& n,
                           std::array<double,3>* axis_scores=nullptr) {
    uint32_t other=0;auto a=histogram(current,c,mask,ref.masks,n);
    auto b=histogram(ref.points,c,mask,ref.masks,other);
    if(n<uint32_t(c.crane_hist_min_points)||other<uint32_t(c.crane_hist_min_points)||
       double(std::min(n,other))/std::max(n,other)<.5)return -1;
    double score=0;size_t offset=0;const auto size=bins(c);
    for(int axis=0;axis<3;++axis) {
      double s=0;for(size_t i=0;i<size[axis];++i)s+=std::abs(a[offset+i]-b[offset+i]);
      s*=.5;if(axis_scores)(*axis_scores)[axis]=s;
      score=std::max(score,s);offset+=size[axis];
    }
    return score;
  }
  static Frame capture(const std::vector<Point>& p,const std::vector<Mask>& m,double stamp,const Config& c) {
    Frame f;f.stamp=stamp;f.masks=m;for(const auto& q:p)if(inWatch(q,c))f.points.push_back(q);return f;
  }
public:
  void update(const std::vector<Point>& points,const Config& c,double stamp,bool continuous,Output& o) {
    o.crane_moving=moving_;auto mask=masks(o);
    if(!continuous) {initialized_=false;recent_.clear();start_since_=stop_since_=-1;}
    if(!initialized_) {
      anchor_=capture(points,mask,stamp,c);recent_.push_back(anchor_);initialized_=true;
      o.crane_motion_reason="SCENE_HIST_WARMUP";return;
    }
    uint32_t count=0,lag_count=0;
    const double score=difference(points,mask,anchor_,c,count,&o.crane_hist_axis_scores);
    while(recent_.size()>1&&recent_[1].stamp<=stamp-c.crane_hist_lag_sec+1e-8)recent_.pop_front();
    const bool lag_ready=!recent_.empty()&&stamp-recent_.front().stamp+1e-8>=c.crane_hist_lag_sec;
    const double recent=lag_ready?difference(points,mask,recent_.front(),c,lag_count):-1;
    o.crane_hist_score=std::max(0.,score);o.crane_hist_recent_score=std::max(0.,recent);o.crane_hist_points=count;
    if(score<0||!lag_ready||recent<0) {
      start_since_=stop_since_=-1;o.crane_motion_reason=score<0?"SCENE_HIST_UNOBSERVED_HOLD":"SCENE_HIST_WARMUP";
      if(score<0) {initialized_=false;recent_.clear();}
    } else {
      o.crane_motion_valid=true;
      if(!moving_) {
        if(score>=c.crane_hist_start_score) {
          if(start_since_<0)start_since_=stamp;
          if(stamp-start_since_+1e-8>=c.crane_hist_start_confirm_sec) {moving_=true;stop_since_=-1;}
        }else start_since_=-1;
      }else {
        if(recent<=c.crane_hist_stop_score) {
          if(stop_since_<0)stop_since_=stamp;
          if(stamp-stop_since_+1e-8>=c.crane_hist_stop_confirm_sec) {
            moving_=false;anchor_=capture(points,mask,stamp,c);start_since_=stop_since_=-1;
          }
        }else stop_since_=-1;
      }
      o.crane_motion_reason=moving_?"SCENE_HIST_MOVING":"SCENE_HIST_STABLE";
    }
    if(initialized_&&(recent_.empty()||stamp-recent_.back().stamp>=.15))recent_.push_back(capture(points,mask,stamp,c));
    o.crane_moving=moving_;
  }
};
}
