#pragma once
#include "cx128s2_bev_perception/general_v2/core.hpp"
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <cx128s2_bev_perception/msg/tracked_object_array.hpp>
#include <cx128s2_bev_perception/msg/perception_recording_frame.hpp>
#include <yaml-cpp/yaml.h>
#include <nlohmann/json.hpp>
#include <filesystem>
#include <cstring>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <optional>
#include <mutex>
#include <fstream>

namespace general_v2
{
using Array = cx128s2_bev_perception::msg::TrackedObjectArray;
using Json = nlohmann::json;
inline int64_t stamp_ns(const builtin_interfaces::msg::Time&t)
{
  if (t.sec < 0 || t.nanosec >= 1000000000)
  {
    return 0;
  }
  return int64_t(t.sec) * 1000000000 + t.nanosec;
}
inline std::vector<Point> decode(const sensor_msgs::msg::PointCloud2&m, const tf2::Transform&transform)
{
  struct Field
  {
    int offset = -1, bytes = 0;
  };
  Field fields[3];
  Field ring;
  for (const auto&f:m.fields)
  {
    if (f.name == "ring" && f.count == 1) {
      const int bytes = f.datatype == sensor_msgs::msg::PointField::UINT8 ? 1 :
          (f.datatype == sensor_msgs::msg::PointField::UINT16 ? 2 : 0);
      if (bytes && uint64_t(f.offset) + bytes <= m.point_step) ring = {int(f.offset), bytes};
    }
    for (int i = 0; i < 3; ++i)
    {
      if (f.name == std::string(1, "xyz"[i]))
      {
        const int bytes = f.datatype ==
                          sensor_msgs::msg::PointField::FLOAT32 ? 4 : (f.datatype ==
                                                                       sensor_msgs::msg::PointField::FLOAT64 ?
                                                                       8 : 0);
        if (f.count != 1 || !bytes || uint64_t(f.offset) + bytes > m.point_step)
        {
          throw std::runtime_error("Invalid XYZ field layout");
        }
        fields[i] = {int(f.offset), bytes};
      }
    }
  }
  for (const auto&f:fields)
  {
    if (f.offset < 0)
    {
      throw std::runtime_error("Missing XYZ field");
    }
  }
  if (uint64_t(m.width) * m.point_step > m.row_step || uint64_t(m.height) * m.row_step > m.data.size() ||
      uint64_t(m.width) * m.height > UINT32_MAX)
  {
    throw std::runtime_error("Truncated or oversized point cloud");
  }
  const uint16_t endian = 1;
  const bool host_big = *reinterpret_cast<const uint8_t*>(&endian) == 0;
  std::vector<Point> points;
  points.reserve(size_t(m.width) * m.height);
  for (uint32_t row = 0; row < m.height; ++row)
  {
    for (uint32_t col = 0; col < m.width; ++col)
    {
      double xyz[3];
      const auto*base = m.data.data() + size_t(row) * m.row_step + size_t(col) * m.point_step;
      for (int i = 0; i < 3; ++i)
      {
        uint8_t bytes[8];
        std::memcpy(bytes, base + fields[i].offset, fields[i].bytes);
        if (host_big != m.is_bigendian)
        {
          std::reverse(bytes, bytes + fields[i].bytes);
        }
        if (fields[i].bytes == 4)
        {
          float f;
          std::memcpy(&f, bytes, 4);
          xyz[i] = f;
        }
        else{
          std::memcpy(&xyz[i], bytes, 8);
        }
      }
      if (!std::isfinite(xyz[0]) || !std::isfinite(xyz[1]) || !std::isfinite(xyz[2]))
      {
        continue;
      }
      const auto p = transform * tf2::Vector3(xyz[0], xyz[1], xyz[2]);
      uint16_t beam = UINT16_MAX;
      if (ring.offset >= 0) {
        if (ring.bytes == 1) beam = base[ring.offset];
        else {
          uint8_t bytes[2] = {base[ring.offset], base[ring.offset + 1]};
          if (host_big != m.is_bigendian) std::swap(bytes[0], bytes[1]);
          std::memcpy(&beam, bytes, 2);
        }
      }
      points.push_back({p.x(), p.y(), p.z(), row * m.width + col, beam});
    }
  }
  return points;
}
inline sensor_msgs::msg::PointCloud2 cloud(const std_msgs::msg::Header&h, const std::vector<Point>&points,
                                           const std::vector<uint32_t>*indices = nullptr)
{
  sensor_msgs::msg::PointCloud2 out;
  out.header = h;
  sensor_msgs::PointCloud2Modifier mod(out);
  mod.setPointCloud2FieldsByString(1, "xyz");
  mod.resize(indices ? indices->size() : points.size());
  sensor_msgs::PointCloud2Iterator<float>x(out, "x"), y(out, "y"), z(out, "z");
  const size_t n = indices ? indices->size() : points.size();
  for (size_t i = 0; i < n; ++i, ++x, ++y, ++z)
  {
    const auto&p = points[indices ? (*indices)[i] : i];
    *x = float(p.x);
    *y = float(p.y);
    *z = float(p.z);
  }
  return out;
}
inline cx128s2_bev_perception::msg::TrackedObject message(const Object&o, const Eigen::Vector2d &sensor_origin=Eigen::Vector2d::Zero())
{
  cx128s2_bev_perception::msg::TrackedObject m;
  m.id = o.id;
  m.class_name = o.vehicle ? "VEHICLE" : "UNKNOWN";
  m.lifecycle_state = !o.confirmed ? "CANDIDATE" : (o.observation ==
                                                    "DIRECT" ? "TRACKED" : "TRACKED_PREDICTED");
  m.motion_state = "STATIONARY";
  if(o.motion!="STATIONARY"){
    const double vx=o.velocity.x(),vy=o.velocity.y();
    if(std::abs(vx)>1.5*std::abs(vy))m.motion_state=vx>=0?"MOVING_FORWARD":"MOVING_BACKWARD";
    else if(std::abs(vy)>1.5*std::abs(vx))m.motion_state=vy>=0?"MOVING_LEFT":"MOVING_RIGHT";
    else m.motion_state="MOVING_DIAGONAL";
  }
  m.vehicle_status = o.status;
  m.position.x = o.center.x();
  m.position.y = o.center.y();
  m.position.z = o.center.z();
  m.velocity.x = o.velocity.x();
  m.velocity.y = o.velocity.y();
  m.dimensions.x = o.size.x();
  m.dimensions.y = o.size.y();
  m.dimensions.z = o.size.z();
  m.distance = float((o.center.head<2>()-sensor_origin).norm());
  m.speed = float(o.velocity.norm());
  m.radial_velocity = float(o.range_speed);
  m.range_slope = float(o.range_speed);
  m.radial_motion_state = o.range_speed > .3 ? "MOVING_AWAY_FROM_SENSOR" : (o.range_speed < -.3 ? "MOVING_TOWARD_SENSOR" : "RADIAL_STATIONARY");
  m.age = o.age;
  m.hits = o.hits;
  m.lost_frames = o.lost;
  m.point_count = int(o.indices.size());
  m.cell_count = o.cells;
  m.confidence = o.confirmed ? .9f : .4f;
  m.decision_reason = "GeneralV2/" + o.observation + (o.partial ? "/PARTIAL_OBSERVATION" : "") +
                      (o.extent_locked ? "/RETAINED_EXTENT" : "/OBSERVED_EXTENT");
  return m;
}
// Preserve the existing package://, /filename and bare-filename conventions.
inline std::string resolve_path(const std::string &path)
{
  if (path.empty())
    return path;
  if (path.rfind("package://", 0) == 0)
  {
    const auto rest = path.substr(10);
    const auto slash = rest.find('/');
    if (slash == std::string::npos || slash == 0 || slash + 1 >= rest.size())
      throw std::runtime_error("Invalid package path");
    return ament_index_cpp::get_package_share_directory(rest.substr(0, slash)) + "/" + rest.substr(slash + 1);
  }
  const auto name = path.front() == '/' ? path.substr(1) : path;
  if (name.find('/') == std::string::npos)
    return ament_index_cpp::get_package_share_directory("cx128s2_bev_perception") + "/config/" + name;
  return path;
}
inline std::vector<Zone> load_zones(const std::string&file, const std::string &expected_frame = "")
{
  const auto root = YAML::LoadFile(file);
  const auto policy = root["roi_policy"] ? root["roi_policy"] : root;
  if (policy["frame_id"] && !expected_frame.empty() && policy["frame_id"].as<std::string>() != expected_frame)
    throw std::runtime_error("ROI frame does not match processing frame");
  const auto nodes = policy["zones"];
  if (!nodes || !nodes.IsSequence())
  {
    throw std::runtime_error("ROI zones sequence missing");
  }
  std::vector<Zone> zones;
  for (const auto&n:nodes)
  {
    Zone z;
    z.name = n["name"] ? n["name"].as<std::string>() : "roi";
    z.type = n["type"] ? n["type"].as<std::string>() : "watch";
    z.enabled = n["enabled"] ? n["enabled"].as<bool>() : true;
    const auto shape = n["shape"] ? n["shape"].as<std::string>() : "polygon";
    if (shape == "box")
    {
      const double a = n["x_min"].as<double>(), b = n["x_max"].as<double>(), c = n["y_min"].as<double>(),
                   d = n["y_max"].as<double>();
      if (a > b || c > d)
      {
        throw std::runtime_error("Invalid ROI box");
      }
      if (a == b || c == d)
      {
        if (z.type == "watch")
          z.enabled = false;
        else throw std::runtime_error("Zero-area exclusion ROI");
      }
      z.x = {a, b, b, a};
      z.y = {c, c, d, d};
    }
    else if (shape == "polygon")
    {
      z.x = n["polygon_x"].as<std::vector<double> >();
      z.y = n["polygon_y"].as<std::vector<double> >();
    }
    else{
      throw std::runtime_error("Unsupported ROI shape");
    }
    zones.push_back(std::move(z));
  }
  return zones;
}
inline Config parameters(rclcpp::Node&n)
{
  Config c;
  auto d = [&](const char*key, double&v){
             v = n.declare_parameter<double>(key, v);
           };
  auto i = [&](const char*key, int&v){
             const auto value = n.declare_parameter<int64_t>(key, v);
             if (value < 0 || value > INT32_MAX)
             {
               throw std::runtime_error("Integer config out of range");
             }
             v = int(value);
           };
  d("valid_x_min", c.xmin);
  d("valid_x_max", c.xmax);
  d("valid_y_min", c.ymin);
  d("valid_y_max", c.ymax);
  d("valid_z_max", c.zmax);
  d("valid_range_min", c.range_min);
  d("valid_range_max", c.range_max);
  d("general_v2.ground_crop_z_min", c.zmin);
  d("general_v2.ground_tile_m", c.ground_tile);
  d("general_v2.ground_seed_min_z", c.ground_seed_min_z);
  d("general_v2.ground_seed_max_z", c.ground_seed_max_z);
  d("general_v2.ground_inlier_m", c.ground_inlier);
  d("general_v2.ground_max_slope", c.ground_max_slope);
  d("general_v2.ground_min_span_m", c.ground_min_span);
  i("general_v2.ground_min_tiles", c.ground_min_tiles);
  i("general_v2.ground_min_points", c.ground_min_points);
  d("general_v2.ground_local_cell_m",c.ground_local_cell);
  d("general_v2.ground_local_band_m",c.ground_local_band);
  d("general_v2.ground_local_max_rise_m",c.ground_local_max_rise);
  d("general_v2.ground_local_max_slope",c.ground_local_slope);
  d("general_v2.ground_local_residual_m",c.ground_local_residual);
  i("general_v2.ground_local_min_points",c.ground_local_min_points);
  d("obstacle_min_height_above_ground", c.obstacle_min);
  d("obstacle_max_height_above_ground", c.obstacle_max);
  c.candidate_ground_noise_rejection = n.declare_parameter<bool>("general_v2.candidate_ground_noise_rejection", true);
  d("general_v2.cell_m", c.cell);
  d("general_v2.connection_gap_m", c.connection_gap);
  d("general_v2.vertical_gap_m", c.vertical_gap);
  i("general_v2.min_points", c.min_points);
  i("general_v2.min_cells", c.min_cells);
  i("general_v2.max_objects", c.max_objects);
  i("general_v2.max_cells", c.max_cells);
  d("general_v2.vehicle_min_length_m", c.vehicle_min_length);
  d("vehicle_min_width", c.vehicle_min_width);
  d("vehicle_max_width", c.vehicle_max_width);
  d("vehicle_min_height", c.vehicle_min_height);
  d("vehicle_max_height", c.vehicle_max_height);
  d("vehicle_max_length", c.vehicle_max_length);
  d("general_v2.confirmation_sec", c.confirmation_sec);
  i("general_v2.confirmation_hits", c.confirmation_hits);
  d("general_v2.hold_sec", c.hold_sec);
  d("general_v2.candidate_hold_sec", c.candidate_hold_sec);
  d("general_v2.reid_sec", c.reid_sec);
  d("general_v2.association_gate_m", c.association_gate);
  d("general_v2.extent_confirmation_sec", c.extent_confirmation_sec);
  d("general_v2.extent_growth_sec", c.extent_growth_sec);
  d("general_v2.partial_ratio", c.partial_ratio);
  d("general_v2.motion_window_sec", c.motion_window);
  d("general_v2.moving_speed_mps", c.moving_speed);
  d("general_v2.stationary_speed_mps", c.stationary_speed);
  d("general_v2.motion_confirmation_sec", c.motion_confirm_sec);
  d("vehicle_stopped_duration_sec", c.stopped_sec);
  const bool require_watch=n.declare_parameter<bool>("require_watch_roi",false);
  const bool discard_outside=n.declare_parameter<bool>("discard_points_outside_watch_roi",false);
  c.require_watch=require_watch||discard_outside;
  const bool policy = n.declare_parameter<bool>("use_roi_policy", true);
  const auto requested_file=n.declare_parameter<std::string>("roi_policy_file","/cx128s2_roi_policy.yaml");
  if (policy)
  {
    const auto file=resolve_path(requested_file);
    if (std::filesystem::is_regular_file(file))
    {
      c.zones = load_zones(file, n.get_parameter("target_frame").as_string());
    }
    else if (c.require_watch)
    {
      throw std::runtime_error("Required ROI file is missing: " + file);
    }
    else{
      RCLCPP_WARN(n.get_logger(), "No ROI file at %s; no ROI restriction", file.c_str());
    }
  }
  const bool legacy = n.declare_parameter<bool>("roi_enabled", false);
  const auto shape = n.declare_parameter<std::string>("roi_shape", "polygon");
  auto px = n.declare_parameter<std::vector<double> >("roi_polygon_x", std::vector<double>{}),
       py = n.declare_parameter<std::vector<double> >("roi_polygon_y", std::vector<double>{});
  const double xmin = n.declare_parameter<double>("roi_x_min", 0),
               xmax = n.declare_parameter<double>("roi_x_max", 200),
               ymin = n.declare_parameter<double>("roi_y_min", -8),
               ymax = n.declare_parameter<double>("roi_y_max", 8);
  if (!policy && legacy)
  {
    Zone z;
    z.name = "legacy_roi";
    if (shape == "box")
    {
      z.x = {xmin, xmax, xmax, xmin};
      z.y = {ymin, ymin, ymax, ymax};
    }
    else{
      z.x = std::move(px);
      z.y = std::move(py);
    }
    c.zones.push_back(std::move(z));
    c.require_watch = true;
  }
  c.validate();
  return c;
}
class Adapter : public rclcpp::Node
{
public:
  std::unique_ptr<Core> core;
  std::unique_ptr<tf2_ros::Buffer> tf;
  std::unique_ptr<tf2_ros::TransformListener> listener;
  std::string input, target, last_frame;
  Eigen::Vector2d last_sensor_origin=Eigen::Vector2d::Zero();
  bool use_tf = true, debug = false, record = false;
  explicit Adapter(bool offline = false) : Node("cx128s2_bev_perception")
  {
    input = declare_parameter<std::string>("input_topic", "/ch/lslidar_point_cloud");
    target = declare_parameter<std::string>("target_frame", "world");
    use_tf = declare_parameter<bool>("use_tf_transform", true);
    core = std::make_unique<Core>(parameters(*this));
    debug = declare_parameter<bool>("publish_debug_topics", false);
    record = declare_parameter<bool>("publish_recording_frame", false);
    const auto mode = declare_parameter<std::string>("perception_mode", "default");
    if (mode == "crane_beam_work")
    {
      throw std::runtime_error("GeneralV2 cannot run the Crane profile");
    }
    const auto background = declare_parameter<std::string>("background_mode", "ground_only");
    if (background != "ground_only")
    {
      throw std::runtime_error("GeneralV2 has no static background mode; set background_mode=ground_only");
    }
    tf = std::make_unique<tf2_ros::Buffer>(get_clock());
    if (!offline)
    {
      listener = std::make_unique<tf2_ros::TransformListener>(*tf);
    }
    if (!offline)
    {
      for (const auto&topic:{"/detected_objects", "/tracked_objects", "/object_motion_states",
                             "/road_vehicle_tracks", "/vehicle_identity_memory"})
      {
        pubs.push_back(create_publisher<Array>(topic, 10));
      }
      diagnostics = create_publisher<std_msgs::msg::String>("/general_v2/diagnostics", 10);
      markers = create_publisher<visualization_msgs::msg::MarkerArray>("/debug/object_markers", 10);
      roi_markers = create_publisher<visualization_msgs::msg::MarkerArray>("/debug/roi_marker", 10);
      background_status = create_publisher<diagnostic_msgs::msg::KeyValue>("/background_status", 10);
      object_status = create_publisher<diagnostic_msgs::msg::KeyValue>("/object_status", 10);
      if (debug)
      {
        road = create_publisher<sensor_msgs::msg::PointCloud2>("/debug/road_frame_points",
                                                               rclcpp::SensorDataQoS());
        foreground = create_publisher<sensor_msgs::msg::PointCloud2>("/debug/foreground_points",
                                                                     rclcpp::SensorDataQoS());
      }
      if (record)
      {
        recording =
          create_publisher<cx128s2_bev_perception::msg::PerceptionRecordingFrame>(
            declare_parameter<std::string>("recording_frame_topic", "/recording/perception_frame"), 10);
      }
      services.push_back(create_service<std_srvs::srv::Trigger>("/load_roi",
                                                                [this](const std_srvs::srv::Trigger::Request::
                                                                       SharedPtr,
                                                                       std_srvs::srv::Trigger::Response::
                                                                       SharedPtr response){
          std::lock_guard<std::mutex> lock(state_mutex);
          try{core->set_zones(load_zones(resolve_path(get_parameter("roi_policy_file").as_string()), target));
              response->success = true;
              response->message = "GeneralV2 ROI reloaded; tracks reset";
          }
          catch (const std::exception&e){response->success = false;
                                         response->message = e.what();
          }
        }));
      services.push_back(create_service<std_srvs::srv::Trigger>("/clear_roi",
                                                                [this](const std_srvs::srv::Trigger::Request::
                                                                       SharedPtr,
                                                                       std_srvs::srv::Trigger::Response::
                                                                       SharedPtr response){
          std::lock_guard<std::mutex> lock(state_mutex);
          core->set_zones({});
          response->success = true;
          response->message = "GeneralV2 ROI cleared; tracks reset";
        }));
      services.push_back(create_service<std_srvs::srv::Trigger>("/save_roi",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr response) {
          std::lock_guard<std::mutex> lock(state_mutex);
          try {
            YAML::Node root;auto policy=root["roi_policy"];policy["version"]=1;policy["frame_id"]=target;
            policy["zones"]=YAML::Node(YAML::NodeType::Sequence);
            for(const auto&zone:core->config().zones){YAML::Node z;z["name"]=zone.name;z["type"]=zone.type;z["enabled"]=zone.enabled;z["shape"]="polygon";z["polygon_x"]=zone.x;z["polygon_y"]=zone.y;policy["zones"].push_back(z);}
            const auto destination=resolve_path(get_parameter("roi_policy_file").as_string());
            const auto temporary=destination+".general_v2.tmp";
            {std::ofstream out(temporary);if(!out)throw std::runtime_error("Cannot write ROI policy");out<<root;out.flush();if(!out)throw std::runtime_error("ROI write failed");}
            std::filesystem::rename(temporary,destination);response->success=true;response->message="GeneralV2 ROI policy saved";
          }catch(const std::exception&e){response->success=false;response->message=e.what();}
        }));
      for(const std::string name:{"/learn_background","/save_background","/load_background","/clear_background","/dump_background"})
        services.push_back(create_service<std_srvs::srv::Trigger>(name,
          [](const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr response){response->success=false;response->message="GeneralV2 uses current-frame ground; static background is disabled";}));
    }
    parameter_guard=add_on_set_parameters_callback([](const std::vector<rclcpp::Parameter>&updates){
      rcl_interfaces::msg::SetParametersResult result;result.successful=true;
      for(const auto&update:updates)if(update.get_name()!="use_sim_time"){
        result.successful=false;result.reason="GeneralV2 parameters are startup-only: edit YAML and restart; use /load_roi for policy edits";break;
      }
      return result;
    });
  }
  std::mutex state_mutex;
  std::optional<Result> process(const sensor_msgs::msg::PointCloud2&m, std::vector<Point>&points,
                                bool publish = true)
  {
    std::lock_guard<std::mutex> lock(state_mutex);
    const auto started = std::chrono::steady_clock::now();
    const auto ns = stamp_ns(m.header.stamp);
    if (ns <= 0)
    {
      RCLCPP_WARN(get_logger(), "Invalid source timestamp rejected");
      return std::nullopt;
    }
    tf2::Transform transform;
    transform.setIdentity();
    std::string frame = m.header.frame_id;
    if (frame.empty())
    {
      RCLCPP_WARN(get_logger(), "Empty sensor frame rejected");
      return std::nullopt;
    }
    bool input_ok = true;
    if(!use_tf && frame!=target)throw std::runtime_error("Raw frame differs from target; enable TF or explicitly set target_frame to the sensor frame");
    if (use_tf && frame != target)
    {
      try{
        const auto t = tf->lookupTransform(target, frame, rclcpp::Time(m.header.stamp),
                                           rclcpp::Duration::from_seconds(0));
        const auto&q = t.transform.rotation;
        const auto&p = t.transform.translation;
        transform = tf2::Transform(tf2::Quaternion(q.x, q.y, q.z, q.w), tf2::Vector3(p.x, p.y, p.z));
        frame = target;
      }
      catch (const tf2::TransformException&e){
        input_ok = false;
        frame = target;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "GeneralV2 exact sensor TF unavailable: %s",
                             e.what());
      }
    }
    if (!last_frame.empty() && last_frame != frame)
    {
      core->reset();
    }
    last_frame = frame;
    points.clear();
    if (input_ok)
    {
      points = decode(m, transform);
    }
    if(input_ok)last_sensor_origin=Eigen::Vector2d(transform.getOrigin().x(),transform.getOrigin().y());
    auto result = core->process(ns, points, input_ok, last_sensor_origin);
    result.raw_points = size_t(m.width) * m.height;
    if (result.quality == "DUPLICATE_SENSOR_TIMESTAMP")
    {
      return std::nullopt;
    }
    result.total_ms = std::chrono::duration<double,
                                            std::milli>(std::chrono::steady_clock::now() - started).count();
    auto header = m.header;
    header.frame_id = frame;
    if (publish)
    {
      publish_result(result, header, points);
    }
    return result;
  }
  void publish_result(const Result&r, const std_msgs::msg::Header&header, const std::vector<Point>&points)
  {
    if (pubs.empty())
    {
      return;
    }
    std::vector<Array> arrays(5);
    for (auto&a:arrays)
    {
      a.header = header;
    }
    for (const auto&o:r.objects)
    {
      auto m = message(o,r.sensor_origin);
      if (o.observation == "DIRECT")
      {
        arrays[0].objects.push_back(m);
      }
      if (o.confirmed)
      {
        arrays[1].objects.push_back(m);
        arrays[2].objects.push_back(m);
        if (o.vehicle)
        {
          arrays[3].objects.push_back(m);
          m.lifecycle_state=o.observation=="DIRECT"?"ACTIVE":"TRACKED_PREDICTED";
          arrays[4].objects.push_back(m);
        }
      }
    }
    for (size_t i = 0; i < pubs.size(); ++i)
    {
      pubs[i]->publish(arrays[i]);
    }
    std_msgs::msg::String info;
    info.data = Json({{"engine", "GeneralV2"}, {"stamp_ns", r.stamp_ns}, {"quality", r.quality},
                       {"ground_valid", r.ground.valid}, {"ground_reason", r.ground.reason},
                       {"raw_points", r.raw_points}, {"foreground_points", r.foreground.size()}, {"candidate_excluded_points", r.candidate_excluded.size()},
                       {"pipeline_ms", r.total_ms}}).dump();
    diagnostics->publish(info);
    diagnostic_msgs::msg::KeyValue bg;
    bg.key = "background";
    bg.value = "BACKGROUND_DISABLED|engine=GeneralV2|ground=" + r.ground.reason;
    background_status->publish(bg);
    bg.key = "objects";
    bg.value = "engine=GeneralV2|vehicles=" + std::to_string(arrays[3].objects.size());
    object_status->publish(bg);
    visualization_msgs::msg::MarkerArray visual;
    visualization_msgs::msg::Marker clear;
    clear.header = header;
    clear.action = clear.DELETEALL;
    visual.markers.push_back(clear);
    for (const auto&o:r.objects)
    {
      auto box = clear;
      box.action = box.ADD;
      box.ns = "general_v2_boxes";
      box.id = o.id;
      box.type = box.CUBE;
      box.pose.position.x = o.center.x();
      box.pose.position.y = o.center.y();
      box.pose.position.z = o.center.z();
      box.pose.orientation.z = std::sin(o.yaw * .5);
      box.pose.orientation.w = std::cos(o.yaw * .5);
      box.scale.x = o.size.x();
      box.scale.y = o.size.y();
      box.scale.z = o.size.z();
      box.color.g = o.confirmed ? 1.f : .5f;
      box.color.b = o.observation == "HELD" ? 1.f : 0.f;
      box.color.a = .3f;
      visual.markers.push_back(box);
      auto text = box;
      text.ns = "general_v2_labels";
      text.type = text.TEXT_VIEW_FACING;
      text.pose.position.z += o.size.z() * .5 + .4;
      text.pose.orientation.z = 0;
      text.pose.orientation.w = 1;
      text.scale.z = .35;
      text.color.r = text.color.g = text.color.b = text.color.a = 1;
      text.text = "ID " + std::to_string(o.id) + " " + o.observation + " " + o.status +
                  (o.partial ? " PARTIAL" : "");
      visual.markers.push_back(text);
    }
    markers->publish(visual);
    visualization_msgs::msg::MarkerArray rois;
    rois.markers.push_back(clear);
    int roi_id = 0;
    for (const auto&zone:core->config().zones)
    {
      if (!zone.enabled)
      {
        continue;
      }
      auto line = clear;
      line.action = line.ADD;
      line.ns = "roi_policy";
      line.id = roi_id++;
      line.type = line.LINE_STRIP;
      line.pose.orientation.w = 1;
      line.scale.x = .08;
      line.color.r = zone.type == "watch" ? 0.f : 1.f;
      line.color.g = zone.type == "watch" ? 1.f : 0.f;
      line.color.a = 1;
      for (size_t i = 0; i <= zone.x.size(); ++i)
      {
        geometry_msgs::msg::Point p;
        p.x = zone.x[i % zone.x.size()];
        p.y = zone.y[i % zone.y.size()];
        line.points.push_back(p);
      }
      rois.markers.push_back(line);
    }
    roi_markers->publish(rois);
    if (road && road->get_subscription_count())
    {
      road->publish(cloud(header, points));
    }
    if (foreground && foreground->get_subscription_count())
    {
      foreground->publish(cloud(header, points, &r.foreground));
    }
    if (recording && recording->get_subscription_count())
    {
      cx128s2_bev_perception::msg::PerceptionRecordingFrame f;
      f.header = header;
      f.transformed_cloud = cloud(header, points);
      f.foreground_cloud = cloud(header, points, &r.foreground);
      f.source_point_count = uint32_t(r.raw_points);
      f.foreground_point_count = uint32_t(r.foreground.size());
      f.cluster_count = uint32_t(r.observations.size());
      for (const auto&o:r.objects)
      {
        cx128s2_bev_perception::msg::RecordingObjectCluster c;
        c.track_id = o.id;
        c.cluster_id = -1;
        c.yaw = float(o.yaw);
        c.observed = o.observation == "DIRECT";
        c.object = message(o,r.sensor_origin);
        c.points = cloud(header, points, &o.indices);
        f.objects.push_back(std::move(c));
      }
      recording->publish(f);
    }
  }
private:
  std::vector<rclcpp::Publisher<Array>::SharedPtr>pubs;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers, roi_markers;
  rclcpp::Publisher<diagnostic_msgs::msg::KeyValue>::SharedPtr background_status, object_status;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr road, foreground;
  rclcpp::Publisher<cx128s2_bev_perception::msg::PerceptionRecordingFrame>::SharedPtr recording;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_guard;
  std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr>services;
};
} // namespace general_v2
