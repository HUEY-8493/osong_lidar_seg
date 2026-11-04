#include "../src/v2/ros_input.hpp"
#include "../src/v2/ros_status.hpp"
#include "../src/v2/visualization.hpp"
#include <iostream>
#include <stdexcept>
#define REQUIRE(x) do { if (!(x)) throw std::runtime_error(#x); } while (0)
int main()
{
  try {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.width=2;cloud.height=1;cloud.point_step=16;cloud.row_step=40;
    for(int i=0;i<3;++i){sensor_msgs::msg::PointField f;f.name=std::string(1,"xyz"[i]);f.offset=i*4;f.count=1;f.datatype=sensor_msgs::msg::PointField::FLOAT32;cloud.fields.push_back(f);}
    cloud.data.resize(40);float values[8]={1,2,3,0,4,5,6,0};std::memcpy(cloud.data.data(),values,32);
    tf2::Transform t=tf2::Transform::getIdentity();t.setOrigin({10,20,30});std::vector<crane_v2::Point>points;
    crane_v2::decode(cloud,t,points);REQUIRE(points.size()==2);REQUIRE(points[0].x==11&&points[0].y==22&&points[0].z==33);REQUIRE(points[1].raw==1);
    cloud.data.resize(31);bool rejected=false;try{crane_v2::decode(cloud,t,points);}catch(const std::runtime_error&){rejected=true;}REQUIRE(rejected);
    cloud.data.resize(40);cloud.is_bigendian=true;rejected=false;try{crane_v2::decode(cloud,t,points);}catch(const std::runtime_error&){rejected=true;}REQUIRE(rejected);
    std_msgs::msg::Header h;h.stamp.sec=1793510000;h.stamp.nanosec=987654321;h.frame_id="laser_link";
    for(const char*state:{"IDLE","LIFTING","SUSPENDED","LOWERING","LOWERING_CONFIRMATION"}){
      crane_v2::Output o;o.state=state;o.work_complete=o.state=="LOWERING_CONFIRMATION";auto m=crane_v2::status(o,h);
      REQUIRE(m.header.stamp==h.stamp);REQUIRE(m.header.frame_id=="world");REQUIRE(m.state==state&&m.working_status==state);REQUIRE(m.work_complete==o.work_complete);REQUIRE(!m.beam_cluster_valid&&!m.beam_ground_clearance_valid);
      REQUIRE(!m.crane_moving);
    }
    crane_v2::Output pending;pending.cargo.confirmed=true;pending.cargo.length=1.;pending.cargo.model_length=19.;
    REQUIRE(!crane_v2::status(pending,h).length_locked);
    pending.cargo.length_locked=true;pending.cargo.model_length=3.;
    auto locked=crane_v2::status(pending,h);REQUIRE(locked.length_locked);REQUIRE(locked.beam_length==3.&&locked.beam_length_locked==3.);
    crane_v2::Output estimated;estimated.cargo.direct=true;estimated.cargo.raw_z_min=.2;estimated.cargo.raw_z_max=.7;estimated.cargo.bottom_z=.15;estimated.cargo.bottom_observation="ESTIMATED";estimated.clearance_valid=true;
    auto em=crane_v2::status(estimated,h);REQUIRE(em.beam_ground_clearance_reason=="V2_ESTIMATED_BODY_BOTTOM_AND_LOCAL_PLANE");REQUIRE(std::abs(em.beam_z_min-.2)<1e-6&&std::abs(em.beam_z_max-.7)<1e-6);REQUIRE(std::abs(em.beam_bottom_z-.15)<1e-6);
    crane_v2::Output held;held.cargo.held=true;held.hooks[0].held=true;auto m=crane_v2::status(held,h);REQUIRE(!m.left_hook_detected&&m.left_hook_tracked);REQUIRE(!m.left_hook_position_updated&&m.left_hook_position_valid);REQUIRE(m.beam_tracking_state=="PREDICTED");REQUIRE(!m.beam_z_valid);
    crane_v2::Output holder;holder.hooks[0].direct=true;holder.hooks[0].raw_indices={0,2};
    holder.hooks[0].body_min={1,2,3};holder.hooks[0].body_max={4,5,6};
    holder.hooks[0].housing={2.5,3.5,4.5};holder.hooks[0].virtual_hook={2.5,3.5,3.6};
    std::vector<crane_v2::Point> raw={{1,2,3,0},{4,5,6,2},{9,9,9,4}};
    auto body=crane_v2::holderClouds(holder,h,raw);
    REQUIRE(body[0].width==2&&body[1].width==0);REQUIRE(body[0].header.stamp==h.stamp&&body[0].header.frame_id=="world");
    sensor_msgs::PointCloud2ConstIterator<float> bx(body[0],"x");REQUIRE(*bx==1);++bx;REQUIRE(*bx==4);
    auto markers=crane_v2::markers(holder,h,raw);bool labelled=false;
    for(const auto& item:markers.markers)labelled|=item.text=="LEFT HOLDER DIRECT";
    REQUIRE(labelled);
    holder.hooks[0].direct=false;holder.hooks[0].held=true;
    body=crane_v2::holderClouds(holder,h,raw);REQUIRE(body[0].width==0&&body[0].data.empty());
    std::cout<<"PASS V2 ROS layout/schema/sensor timestamps/held validity/actual Holder Raw and labels\n";
  }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}
}
