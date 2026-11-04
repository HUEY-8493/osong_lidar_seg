#pragma once
#include "cx128s2_bev_perception/v2/core.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cmath>
#include <iomanip>
#include <sstream>
namespace crane_v2 {
inline visualization_msgs::msg::MarkerArray markers(
  const Output& o, const std_msgs::msg::Header& header, const std::vector<Point>& points)
{
  using M=visualization_msgs::msg::Marker;
  visualization_msgs::msg::MarkerArray out;
  M clear;clear.header=header;clear.header.frame_id="world";
  clear.action=M::DELETEALL;out.markers.push_back(clear);
  auto make=[&](int type) {
    M m;m.header=header;m.header.frame_id="world";m.ns="crane_v2";
    m.id=out.markers.size();m.type=type;m.action=M::ADD;m.pose.orientation.w=1.;
    m.color.a=1.;m.lifetime.sec=1;return m;
  };
  // Actual Holder, its virtual Hook, and the offset link are different geometry.
  std::vector<M> holder_labels;
  for(int side=0;side<2;++side) {
    const auto& h=o.hooks[side];
    if(!h.direct&&!h.held)continue;
    auto body=make(M::LINE_LIST);body.color.g=1.;body.color.r=.1;
    body.color.a=h.direct?1.:.35;body.scale.x=.018;
    std::array<geometry_msgs::msg::Point,8> corners;
    for(int i=0;i<8;++i) {
      corners[i].x=(i&1)?h.body_max.x:h.body_min.x;
      corners[i].y=(i&2)?h.body_max.y:h.body_min.y;
      corners[i].z=(i&4)?h.body_max.z:h.body_min.z;
    }
    for(int i=0;i<8;++i)for(int axis=0;axis<3;++axis)
      if(!(i&(1<<axis))) {body.points.push_back(corners[i]);body.points.push_back(corners[i|(1<<axis)]);}
    out.markers.push_back(body);
    auto label=make(M::TEXT_VIEW_FACING);label.pose.position.x=h.housing.x;
    label.pose.position.y=h.housing.y;label.pose.position.z=h.body_max.z+.15;
    label.scale.z=.13;label.color.g=1.;label.color.r=.1;
    label.text=std::string(side==0?"LEFT HOLDER ":"RIGHT HOLDER ")+(h.direct?"DIRECT":"HELD");
    label.ns="crane_v2_holder_labels";label.id=side;
    holder_labels.push_back(label);
    auto link=make(M::LINE_LIST);link.scale.x=.01;link.color.r=1.;link.color.g=.45;
    link.color.a=h.direct?.7:.25;
    geometry_msgs::msg::Point p;p.x=h.housing.x;p.y=h.housing.y;p.z=h.housing.z;link.points.push_back(p);
    p.x=h.virtual_hook.x;p.y=h.virtual_hook.y;p.z=h.virtual_hook.z;link.points.push_back(p);
    out.markers.push_back(link);
    auto m=make(M::SPHERE);m.pose.position.x=h.virtual_hook.x;
    m.pose.position.y=h.virtual_hook.y;m.pose.position.z=h.virtual_hook.z;
    m.scale.x=m.scale.y=m.scale.z=.14;m.color.r=1.;m.color.g=.45;
    if(h.held)m.color.a=.35;
    out.markers.push_back(m);
  }
  for(int side=0;side<2;++side) if(o.crane_root_observed[side]) {
    auto m=make(M::SPHERE);const auto& p=o.crane_roots[side];
    m.pose.position.x=p.x;m.pose.position.y=p.y;m.pose.position.z=p.z;
    m.scale.x=m.scale.y=m.scale.z=.12;m.color.g=m.color.b=1.;out.markers.push_back(m);
  }
  if(o.retained_box_valid||o.cargo.direct||o.cargo.held) {
    const auto center=o.retained_box_valid?o.retained_box.center:o.cargo.center;
    const double yaw=o.retained_box_valid?o.retained_box.yaw:o.cargo.yaw;
    auto m=make(M::CUBE);m.pose.position.x=center.x;
    m.pose.position.y=center.y;m.pose.position.z=center.z;
    m.pose.orientation.z=std::sin(yaw*.5);m.pose.orientation.w=std::cos(yaw*.5);
    m.scale.x=o.cargo.length_locked?o.cargo.model_length:o.cargo.length;
    m.scale.y=o.retained_box_valid?o.retained_box.width:o.cargo.width;
    m.scale.z=o.retained_box_valid?o.retained_box.height:o.cargo.height;
    m.color.r=1.;m.color.g=.75;m.color.a=.18;out.markers.push_back(m);
  }
  // Only current Raw is green, including the live fragment during shrink hold.
  auto raw=make(M::POINTS);raw.scale.x=raw.scale.y=.035;raw.color.g=1.;
  const auto& ids=o.cargo_shrink_held?o.measured_cargo_raw_indices:o.cargo.raw_indices;
  // Raw indices belong to the input cloud, not necessarily compact vector offsets.
  std::vector<uint8_t> selected;
  for(auto id:ids) if(id>=selected.size())selected.resize(size_t(id)+1);
  for(auto id:ids) selected[id]=1;
  for(const auto& p:points) if(p.raw<selected.size()&&selected[p.raw]) {
    geometry_msgs::msg::Point q;q.x=p.x;q.y=p.y;q.z=p.z;raw.points.push_back(q);
  }
  if(!raw.points.empty())out.markers.push_back(raw);
  const auto label_center=o.retained_box_valid?o.retained_box.center:o.cargo.center;
  auto text=make(M::TEXT_VIEW_FACING);text.pose.position.x=label_center.x;
  text.pose.position.y=label_center.y;text.pose.position.z=label_center.z+1.;
  text.scale.z=.22;text.color.r=text.color.g=text.color.b=1.;
  std::ostringstream label;label<<o.state<<" ID="<<o.cargo.id<<"\n"<<std::fixed<<std::setprecision(2)
    <<(o.cargo.length_locked?"LOCKED ESTIMATED ":"OBSERVED ")
    <<(o.cargo.length_locked?o.cargo.model_length:o.cargo.length)<<"m"
    <<" / FRAGMENT "<<o.measured_cargo_length<<"m "
    <<(o.cargo.direct?"DIRECT":o.cargo.held?"HELD":"NONE")
    <<"\nCRANE MOVE="<<(o.crane_moving?"MOVING":"STILL")
    <<(o.crane_motion_valid?"":" (HELD/UNOBSERVED)");
  text.text=label.str();out.markers.push_back(text);
  for(auto& h:holder_labels)out.markers.push_back(std::move(h));
  return out;
}
inline std::array<sensor_msgs::msg::PointCloud2,2> holderClouds(
  const Output& o, const std_msgs::msg::Header& header, const std::vector<Point>& points)
{
  std::vector<uint8_t> owner;
  for(int side=0;side<2;++side)if(o.hooks[side].direct)for(auto id:o.hooks[side].raw_indices) {
    if(id>=owner.size())owner.resize(size_t(id)+1);
    owner[id]=side+1;
  }
  std::array<std::vector<std::array<float,3>>,2> xyz;
  for(const auto& p:points)if(p.raw<owner.size()&&owner[p.raw])
    xyz[owner[p.raw]-1].push_back({p.x,p.y,p.z});
  std::array<sensor_msgs::msg::PointCloud2,2> out;
  for(int side=0;side<2;++side) {
    auto& cloud=out[side];cloud.header=header;cloud.header.frame_id="world";cloud.height=1;cloud.is_dense=true;
    sensor_msgs::PointCloud2Modifier modifier(cloud);modifier.setPointCloud2FieldsByString(1,"xyz");modifier.resize(xyz[side].size());
    if(xyz[side].empty())continue;
    sensor_msgs::PointCloud2Iterator<float> x(cloud,"x"),y(cloud,"y"),z(cloud,"z");
    for(const auto& p:xyz[side]) {*x=p[0];*y=p[1];*z=p[2];++x;++y;++z;}
  }
  return out;
}
}
