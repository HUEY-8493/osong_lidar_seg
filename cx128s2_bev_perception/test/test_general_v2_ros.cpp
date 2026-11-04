#include "../src/general_v2/ros_adapter.hpp"
#include <iostream>
using namespace general_v2;
void require(bool ok, const char*why)
{
  if (!ok)
  {
    throw std::runtime_error(why);
  }
}
int main(int argc, char**argv)
{
  rclcpp::init(argc, argv);
  try {
    auto n = std::make_shared<Adapter>(true);
    for(int i=1;i<argc;++i)if(std::string(argv[i])=="--validate-config"){
      std::cout<<"GeneralV2 production profile valid; zones="<<n->core->config().zones.size()<<" min_height="<<n->core->config().obstacle_min<<"\n";
      n.reset();rclcpp::shutdown();return 0;
    }

    std::vector<Point> source;
    for (int i = -30; i <= 80; ++i)
    {
      for (int j = -30; j <= 30; ++j)
      {
        source.push_back({i*.4, j * .4, 0, uint32_t(source.size())});
      }
    }
    for (int k = 0; k < 15; ++k)
    {
      for (int i = 0; i <= 40; ++i)
      {
        for (int j = 0; j <= 13; ++j)
        {
          if (k != 14 && i != 0 && i != 40 && j != 0 && j != 13)
          {
            continue;
          }
          source.push_back({4 + i * .2, -1.3 + j * .2, .25 + k * .2, uint32_t(source.size())});
        }
      }
    }
    std_msgs::msg::Header header;
    header.frame_id = "world";
    header.stamp.sec = 1;
    auto m = cloud(header, source);
    std::vector<Point> decoded;
    auto original = decode(m, tf2::Transform::getIdentity());
    require(original.size() == source.size(), "PointCloud2 round trip");
    require(original[0].ring == UINT16_MAX, "Missing ring must remain explicitly unknown");
    auto beam_cloud = m;
    sensor_msgs::msg::PointField beam_field;
    beam_field.name="ring"; beam_field.offset=m.point_step;
    beam_field.count=1; beam_field.datatype=sensor_msgs::msg::PointField::UINT16;
    beam_cloud.fields.push_back(beam_field);
    beam_cloud.point_step=m.point_step+2;beam_cloud.row_step=beam_cloud.width*beam_cloud.point_step;
    beam_cloud.data.assign(beam_cloud.row_step,0);
    for(size_t i=0;i<source.size();++i){
      std::memcpy(beam_cloud.data.data()+i*beam_cloud.point_step,m.data.data()+i*m.point_step,m.point_step);
      const uint16_t beam=uint16_t(i%128);
      std::memcpy(beam_cloud.data.data()+i*beam_cloud.point_step+beam_field.offset,&beam,2);
    }
    require(decode(beam_cloud,tf2::Transform::getIdentity())[35].ring==35,"UINT16 beam must survive decoding");
    auto big_beam=beam_cloud;big_beam.is_bigendian=!beam_cloud.is_bigendian;
    for(size_t i=0;i<source.size();++i){
      auto*base=big_beam.data.data()+i*big_beam.point_step;
      for(const auto&field:big_beam.fields){
        const size_t bytes=field.name=="ring"?2:4;
        std::reverse(base+field.offset,base+field.offset+bytes);
      }
    }
    const auto big_decoded=decode(big_beam,tf2::Transform::getIdentity());
    require(big_decoded[35].ring==35&&big_decoded[35].x==original[35].x,"Big-endian beam and XYZ must survive decoding");
    beam_cloud.fields.back().datatype=sensor_msgs::msg::PointField::UINT8;
    require(decode(beam_cloud,tf2::Transform::getIdentity())[35].ring==35,"UINT8 beam must survive decoding");
    beam_cloud.fields.back().offset=beam_cloud.point_step;
    require(decode(beam_cloud,tf2::Transform::getIdentity())[35].ring==UINT16_MAX,"Malformed optional beam must pass through as unknown");
    // Organized rows with padding must be decoded by row_step, not packed indexing.
    auto organized = m;
    organized.width = 1;
    organized.height = m.width;
    organized.row_step = m.point_step + 8;
    organized.data.assign(size_t(organized.row_step) * organized.height, 0);
    for (size_t i = 0; i < source.size(); ++i)
    {
      std::memcpy(organized.data.data() + i * organized.row_step, m.data.data() + i * m.point_step,
                  m.point_step);
    }
    require(decode(organized, tf2::Transform::getIdentity()).size() == source.size(),
            "Organized padded cloud decoder");
    auto truncated = m;
    truncated.data.resize(1);
    bool caught = false;
    try{
      decode(truncated, tf2::Transform::getIdentity());
    }
    catch (const std::exception&){
      caught = true;
    }
    require(caught, "Truncated cloud must fail closed");
    std::optional<Result> result;
    for (int f = 0; f < 35; ++f)
    {
      m.header.stamp.sec = 1 + f / 10;
      m.header.stamp.nanosec = (f % 10) * 100000000;
      result = n->process(m, decoded, false);
      require(bool(result), "Valid sensor frame output");
    }
    require(result->objects.size() == 1 && result->objects[0].confirmed && result->objects[0].vehicle,
            "ROS adapter must run the new core");
    auto msg = message(result->objects[0]);
    require(msg.class_name == "VEHICLE" && msg.dimensions.x > 7.9 && msg.dimensions.x < 8.1,
            "Length-axis ROS message contract");
    require(msg.distance > 7.9 && msg.distance < 8.1, "Distance is planar sensor distance");
    require(msg.vehicle_status == "stopping" || msg.vehicle_status == "stopped",
            "Public status string contract");
    require(result->stamp_ns == stamp_ns(m.header.stamp), "Original nanosecond timestamp preserved");
    require(!n->process(m, decoded, false), "Duplicate frame must not advance evidence");
    auto invalid = m;
    invalid.header.stamp.sec = 0;
    invalid.header.stamp.nanosec = 0;
    require(!n->process(invalid, decoded, false), "Zero timestamp cannot use PC clock fallback");
    auto tf_missing = m;
    tf_missing.header.stamp.sec = 5;
    tf_missing.header.frame_id = "unknown_sensor";
    auto held = n->process(tf_missing, decoded, false);
    require(held && held->quality == "INPUT_UNAVAILABLE" && !held->objects.empty() &&
            held->objects[0].observation == "HELD" && held->objects[0].indices.empty(),
            "Missing sensor TF cannot become DIRECT");
    auto fresh = std::make_shared<Adapter>(true);
    fresh->tf->setTransform([]{
      geometry_msgs::msg::TransformStamped t;
      t.header.frame_id = "world";
      t.child_frame_id = "sensor";
      t.transform.rotation.w = 1;
        t.transform.translation.x = 5;
      return t;
    }
                              (), "test-static", true);
    auto input = m;
    input.header.frame_id = "sensor";
    input.header.stamp.sec = 8;
    auto transformed = fresh->process(input, decoded, false);
    require(transformed && transformed->quality == "OK", "Recorded static TF path");
    require(!transformed->objects.empty()&&transformed->sensor_origin.x()==5,"Translated sensor origin must be retained");
    const auto shifted=message(transformed->objects[0],transformed->sensor_origin);
    require(shifted.position.x>12.9&&shifted.position.x<13.1&&shifted.distance>7.9&&shifted.distance<8.1,"World position and sensor-relative distance must not be mixed");
    auto direction=result->objects[0];direction.motion="MOVING";direction.velocity=Eigen::Vector2d(.6,0);direction.range_speed=.6;
    require(message(direction).motion_state=="MOVING_FORWARD"&&message(direction).radial_motion_state=="MOVING_AWAY_FROM_SENSOR","Legacy public motion strings preserved");
    std::cout << Json({{"engine", "GeneralV2"}, {"stamp_ns", result->stamp_ns},
                        {"vehicle",
                         {{"id", msg.id}, {"length", msg.dimensions.x}, {"width", msg.dimensions.y},
                           {"height", msg.dimensions.z}, {"distance", msg.distance},
                           {"vehicle_status", msg.vehicle_status}, {"age", msg.age}}}}).dump() << '\n';
    fresh.reset();
    n.reset();
    rclcpp::shutdown();
    return 0;
  }
  catch (const std::exception&e){
    std::cerr << "GeneralV2 ROS test failure: " << e.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
}
