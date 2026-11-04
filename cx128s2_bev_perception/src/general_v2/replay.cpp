#include "ros_adapter.hpp"
#include <rosbag2_cpp/reader.hpp>
#include <rclcpp/serialization.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <fstream>
#include <chrono>
#include <iostream>
#ifndef GENERAL_V2_SOURCE_SHA
#define GENERAL_V2_SOURCE_SHA "unknown"
#endif
using namespace general_v2;
namespace fs = std::filesystem;
int main(int argc, char**argv)
{
  if (argc == 2 && std::string(argv[1]) == "--build-info")
  {
    std::cout << Json({{"engine", "GeneralV2"}, {"source_sha256", GENERAL_V2_SOURCE_SHA},{"core_source_sha256",core_source_sha()}}).dump() << '\n';
    return 0;
  }
  if (argc < 5)
  {
    std::cerr << "usage: general_v2_replay BAG OUTPUT_PREFIX PREVIEW_DIR --ros-args --params-file YAML\n";
    return 2;
  }
  try {
    if(core_source_sha()!=GENERAL_V2_SOURCE_SHA)throw std::runtime_error("Replay/core library build mismatch");
    fs::path bag = argv[1], prefix = argv[2], previews = argv[3];
    rclcpp::init(argc - 3, argv + 3);
    auto node = std::make_shared<Adapter>(true);
    fs::create_directories(prefix.parent_path());
    fs::create_directories(previews);
    std::ofstream times(prefix.string() + "_times.csv"), vehicles(prefix.string() + "_vehicles.jsonl"),
    tracks(prefix.string() + "_tracks.csv"), trace(prefix.string() + "_trace.jsonl");
    if (!times || !vehicles || !tracks || !trace)
    {
      throw std::runtime_error("Cannot open output files");
    }
    times << "frame,input_sequence,stamp,pipeline_ms,ground_ms,cluster_ms,tracking_ms\n";
    tracks <<
      "frame,stamp,id,lost,public,vehicle,status,observation,points,x,y,vx,vy,observed_length,retained_length\n";
    // Static TF is timestamp-independent, and recorder snapshots can appear after first Raw.
    rosbag2_cpp::Reader statics;
    statics.open(bag.string());
    rosbag2_storage::StorageFilter sf;
    sf.topics = {"/tf_static"};
    statics.set_filter(sf);
    while (statics.has_next())
    {
      const auto m = statics.read_next();
      rclcpp::SerializedMessage s(*m->serialized_data);
      tf2_msgs::msg::TFMessage t;
      rclcpp::Serialization<tf2_msgs::msg::TFMessage>().deserialize_message(&s, &t);
      for (auto&v:t.transforms)
      {
        node->tf->setTransform(v, "bag-static", true);
      }
    }
    rosbag2_cpp::Reader reader;
    reader.open(bag.string());
    rosbag2_storage::StorageFilter filter;
    filter.topics = {node->input, "/tf"};
    reader.set_filter(filter);
    uint64_t count = 0, output_count = 0, vehicle_frames = 0;
    int64_t first = 0, last = 0;
    std::map<std::string, int> qualities;
    std::vector<double> durations;
    std::vector<Point> points;
    double last_preview = -1e9;
    size_t preview_count = 0, event_previews=0;
    bool pending_scene=false;
    int last_periodic_slot=-1;
    const double preview_interval=std::max(1.,double(reader.get_metadata().duration.count())*1e-9/11.);
    std::string previous_scene;
    std::vector<Json> preview_index;
    auto began = std::chrono::steady_clock::now();
    while (reader.has_next() && rclcpp::ok())
    {
      const auto raw = reader.read_next();
      rclcpp::SerializedMessage s(*raw->serialized_data);
      if (raw->topic_name == "/tf")
      {
        tf2_msgs::msg::TFMessage tf;
        rclcpp::Serialization<tf2_msgs::msg::TFMessage>().deserialize_message(&s, &tf);
        for (auto&v:tf.transforms)
        {
          node->tf->setTransform(v, "bag", false);
        }
        continue;
      }
      sensor_msgs::msg::PointCloud2 m;
      rclcpp::Serialization<sensor_msgs::msg::PointCloud2>().deserialize_message(&s, &m);
      ++count;
      const int64_t ns = stamp_ns(m.header.stamp);
      if (ns <= 0 || (last && ns <= last))
      {
        throw std::runtime_error("Invalid/nonmonotonic source timestamp at frame " + std::to_string(count));
      }
      if (!first)
      {
        first = ns;
      }
      last = ns;
      auto maybe = node->process(m, points, false);
      if (!maybe)
      {
        throw std::runtime_error("Input frame did not reach V2 output");
      }
      const auto&r = *maybe;
      ++output_count;
      ++qualities[r.quality];
      const double relative = double(ns - first) * 1e-9;
      times << count << ',' << count << ',' << std::setprecision(17) << relative << ',' << r.total_ms <<
        ',' << r.ground_ms << ',' << r.cluster_ms << ',' << r.tracking_ms << '\n';
      durations.push_back(r.total_ms);
      Json row = {{"frame", count}, {"stamp_ns", ns}, {"relative_sec", relative},
        {"stamp_sec", m.header.stamp.sec}, {"stamp_nanosec", m.header.stamp.nanosec},
        {"vehicles", Json::array()}};
      Json all = Json::array();
      std::string scene=r.quality+":"+r.ground.reason;
      for (const auto&o:r.objects)
      {
        Json obj = {{"id", o.id}, {"length", o.size.x()}, {"width", o.size.y()}, {"height", o.size.z()},
          {"observed_length", o.observed_size.x()}, {"observed_width", o.observed_size.y()},
          {"observed_height", o.observed_size.z()}, {"x", o.center.x()}, {"y", o.center.y()},
          {"z", o.center.z()}, {"yaw", o.yaw}, {"distance", (o.center.head<2>()-r.sensor_origin).norm()}, {"status", o.status},
          {"age", o.age}, {"lost", o.lost}, {"points", o.indices.size()}, {"confirmed", o.confirmed},
          {"vehicle", o.vehicle}, {"observation", o.observation}, {"extent_locked", o.extent_locked},
          {"partial", o.partial}};
        all.push_back(obj);
        if (o.confirmed && o.vehicle)
        {
          row["vehicles"].push_back(obj);
        }
        tracks << count << ',' << relative << ',' << o.id << ',' << o.lost << ',' << o.confirmed << ',' <<
          o.vehicle << ',' << o.status << ',' << o.observation << ',' << o.indices.size() << ',' <<
          o.center.x() << ',' << o.center.y() << ',' << o.velocity.x() << ',' << o.velocity.y() << ',' <<
          o.observed_size.x() << ',' << o.size.x() << '\n';
        scene += std::to_string(o.id) + ":" + o.status + ":" + o.observation + ":" +
                 (o.confirmed ? "C" : "?") +
                 ";";
      }
      vehicle_frames += !row["vehicles"].empty();
      vehicles << row.dump() << '\n';
      Json event = {{"frame", count}, {"stamp_ns", ns}, {"relative_sec", relative}, {"quality", r.quality},
        {"ground",
         {{"valid", r.ground.valid}, {"reason", r.ground.reason}, {"a", r.ground.a}, {"b", r.ground.b},
           {"c", r.ground.c}, {"support", r.ground.support}, {"residual", r.ground.residual}}},
        {"raw_points", r.raw_points}, {"foreground_points", r.foreground.size()}, {"candidate_excluded_points", r.candidate_excluded.size()}, {"objects", all}};
      trace << event.dump() << '\n';
      pending_scene=pending_scene||scene!=previous_scene;
      const int periodic_slot=int(relative/preview_interval);
      const bool periodic_due=periodic_slot>last_periodic_slot;
      if (preview_count < 32 &&
          (!preview_count || periodic_due || (pending_scene && relative-last_preview>=1 && event_previews<20)))
      {
        const auto file = "frame_" + std::to_string(count) + ".bin";
        std::ofstream binary(previews / file, std::ios::binary);
        std::vector<int32_t>owner(points.size(), -1);
        for (auto i:r.ground_indices)
        {
          owner[i] = -2;
        }
        for (auto i:r.candidate_excluded) owner[i] = -4;
        for (const auto&o:r.objects)
        {
          for (auto i:o.indices)
          {
            owner[i] = o.id;
          }
        }
        for (size_t i = 0; i < points.size(); ++i)
        {
          float xyz[3] = {float(points[i].x), float(points[i].y), float(points[i].z)};
          binary.write(reinterpret_cast<const char*>(xyz), sizeof(xyz));
          binary.write(reinterpret_cast<const char*>(&owner[i]), sizeof(int32_t));
        }
        event["raw_file"] = file;
        preview_index.push_back(event);
        last_preview = relative;
        ++preview_count;
        if(periodic_due)last_periodic_slot=periodic_slot;else ++event_previews;
        pending_scene=false;
      }
      previous_scene = scene;
      if (count % 200 == 0)
      {
        std::cerr << "GeneralV2 frame=" << count << '\n';
      }
    }
    if (!rclcpp::ok())
    {
      throw std::runtime_error("Replay interrupted");
    }
    const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count();
    std::sort(durations.begin(), durations.end());
    double mean = 0;
    for (auto v:durations)
    {
      mean += v;
    }
    if (!durations.empty())
    {
      mean /= durations.size();
    }
    Json stats = {{"engine", "GeneralV2"}, {"source_sha256", GENERAL_V2_SOURCE_SHA}, {"frames", count},
      {"output_frames", output_count}, {"vehicle_frames", vehicle_frames}, {"first_stamp_ns", first},
      {"last_stamp_ns", last}, {"quality_counts", qualities},
      {"pipeline_ms",
       {{"mean", mean},
         {"p99",
          durations.empty() ? 0 : durations[std::min(durations.size() - 1, size_t(durations.size() * .99))]},
         {"max", durations.empty() ? 0 : durations.back()}}}, {"wall_sec", wall}, {"mode", "full"}};
    std::ofstream(prefix.string() + "_stats.json") << stats.dump(2);
    std::ofstream(previews / "index.json") << Json(preview_index).dump(2);
    std::cerr << stats.dump() << '\n';
    node.reset();
    rclcpp::shutdown();
    return 0;
  }
  catch (const std::exception&e){
    std::cerr << "GeneralV2 replay error: " << e.what() << '\n';
    if (rclcpp::ok())
    {
      rclcpp::shutdown();
    }
    return 2;
  }
}
