#pragma once
#include "cx128s2_bev_perception/v2/core.hpp"
namespace crane_v2 {
// Display-only geometry. Current Raw, track centroid and FSM stay measured.
class RetainedExtent {
  uint64_t id_=0;
  bool sampled_=false;
  int side_=0;
  Output::ExtentBox sample_;
  Vec3 last_center_;
  std::array<Vec3,2> hook_offsets_{};
  std::array<bool,2> hook_anchors_{};
public:
  void reset() { *this=RetainedExtent{}; }
  void update(const Cargo& measured, const std::array<Hook,2>& hooks,
              bool working, double locked, bool released, double extension_step,
              Output& out) {
    if(released||!measured.id) {reset();return;}
    if(id_!=measured.id) {reset();id_=measured.id;}
    (void)working; // Idle observations may be retained privately, never published early.
    if(measured.direct&&measured.connected&&
       (!sampled_||measured.length>sample_.length+extension_step)) {
      sample_={measured.center,measured.length,measured.width,measured.height,measured.yaw};
      last_center_=sample_.center;sampled_=true;
      side_=measured.supporting_side==1?1:0;
      for(int i=0;i<2;++i) {
        hook_anchors_[i]=hooks[i].direct;
        hook_offsets_[i]={sample_.center.x-hooks[i].virtual_hook.x,
                          sample_.center.y-hooks[i].virtual_hook.y,
                          sample_.center.z-hooks[i].virtual_hook.z};
      }
    }
    if(!measured.confirmed||!sampled_||locked<=0)return;
    auto center=last_center_;
    const int selected=hooks[side_].direct&&hook_anchors_[side_]?side_:1-side_;
    if(hooks[selected].direct&&hook_anchors_[selected]) {
      center.x=hooks[selected].virtual_hook.x+hook_offsets_[selected].x;
      center.y=hooks[selected].virtual_hook.y+hook_offsets_[selected].y;
      center.z=hooks[selected].virtual_hook.z+hook_offsets_[selected].z;
    }
    if(measured.direct&&measured.connected)center.z=measured.center.z;
    last_center_=center;
    out.retained_box_valid=true;out.retained_box=sample_;
    out.retained_box.center=center;out.retained_box.length=locked;
  }
};
}
