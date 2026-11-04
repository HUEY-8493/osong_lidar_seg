// V2 runtime selected by the Crane launch profile; executable name retained.
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_msgs/msg/string.hpp>
#include "ros_input.hpp"
#include "ros_status.hpp"
#include "visualization.hpp"
#include <fstream>
#include <iomanip>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
class CraneV2Experimental final : public rclcpp::Node {
crane_v2::Engine engine_;
std::vector<crane_v2::Point>points_;
tf2_ros::Buffer tf_;
tf2_ros::TransformListener listener_;
tf2::Transform frozen_;
bool frozen_valid_ = false;
rclcpp::Publisher<cx128s2_bev_perception::msg::CraneBeamStatus>::SharedPtr publisher_;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_;
std::array<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr,2> holder_clouds_;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostic_, working_;
rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr input_;
std::ofstream audit_, shrink_audit_;
uint64_t received_ = 0, processed_ = 0;
public: CraneV2Experimental() : Node("crane_v2_experimental"), tf_(get_clock()), listener_(tf_, this, false)
{
  crane_v2::Config config;
  rcl_interfaces::msg::ParameterDescriptor startup_only;
  startup_only.read_only = true;
  config.crane_motion_mode=declare_parameter<std::string>("crane_motion_mode",config.crane_motion_mode,startup_only);
  config.crane_hist_xy_bin_m=declare_parameter<double>("crane_hist_xy_bin_m",config.crane_hist_xy_bin_m,startup_only);
  config.crane_hist_bin_m=declare_parameter<double>("crane_hist_bin_m",config.crane_hist_bin_m,startup_only);
  config.crane_hist_start_score=declare_parameter<double>("crane_hist_start_score",config.crane_hist_start_score,startup_only);
  config.crane_hist_stop_score=declare_parameter<double>("crane_hist_stop_score",config.crane_hist_stop_score,startup_only);
  config.crane_hist_lag_sec=declare_parameter<double>("crane_hist_lag_sec",config.crane_hist_lag_sec,startup_only);
  config.crane_hist_start_confirm_sec=declare_parameter<double>("crane_hist_start_confirm_sec",config.crane_hist_start_confirm_sec,startup_only);
  config.crane_hist_stop_confirm_sec=declare_parameter<double>("crane_hist_stop_confirm_sec",config.crane_hist_stop_confirm_sec,startup_only);
  config.crane_hist_min_points=declare_parameter<int>("crane_hist_min_points",config.crane_hist_min_points,startup_only);
  startup_only.description = "Independent crane-body XY motion; startup-only settings in world coordinates.";
  config.crane_xy_enabled = declare_parameter<bool>("crane_xy_enabled", config.crane_xy_enabled, startup_only);
  config.crane_xy_start_m = declare_parameter<double>("crane_xy_start_m", config.crane_xy_start_m, startup_only);
  config.crane_xy_min_speed = declare_parameter<double>("crane_xy_min_speed", config.crane_xy_min_speed, startup_only);
  config.crane_xy_start_confirm_sec = declare_parameter<double>("crane_xy_start_confirm_sec", config.crane_xy_start_confirm_sec, startup_only);
  config.crane_xy_stop_span_m = declare_parameter<double>("crane_xy_stop_span_m", config.crane_xy_stop_span_m, startup_only);
  config.crane_xy_stop_confirm_sec = declare_parameter<double>("crane_xy_stop_confirm_sec", config.crane_xy_stop_confirm_sec, startup_only);
  config.crane_xy_min_points = declare_parameter<int>("crane_xy_min_points", config.crane_xy_min_points, startup_only);
  for(int side=0;side<2;++side) {
    const auto& r=config.crane_root_rois[side];
    auto roi=declare_parameter<std::vector<double>>(side==0?"crane_xy_left_root_roi":"crane_xy_right_root_roi",
      std::vector<double>(r.begin(),r.end()),startup_only);
    if(roi.size()!=6)throw std::invalid_argument("Crane root ROI needs xmin,xmax,ymin,ymax,zmin,zmax");
    for(int j=0;j<6;++j)config.crane_root_rois[side][j]=roi[j];
  }
  startup_only.description = "Startup-only Cargo length confirmation and shrink hold; current measurements remain separate.";
  config.cargo_length_confirm_sec = declare_parameter<double>("cargo_length_confirm_sec", config.cargo_length_confirm_sec, startup_only);
  config.cargo_shrink_hold_sec = declare_parameter<double>("cargo_shrink_hold_sec", config.cargo_shrink_hold_sec, startup_only);
  config.cargo_shrink_ratio = declare_parameter<double>("cargo_shrink_ratio", config.cargo_shrink_ratio, startup_only);
  config.cargo_shrink_recovery_ratio = declare_parameter<double>("cargo_shrink_recovery_ratio", config.cargo_shrink_recovery_ratio, startup_only);
  engine_ = crane_v2::Engine(config);
  if (const char*path = std::getenv("CRANE_V2_SHRINK_AUDIT"))
  {
    shrink_audit_.open(path);
    shrink_audit_ << std::setprecision(17);
    if (shrink_audit_)
      shrink_audit_ << "sensor_stamp_ns,cargo_id,measured_direct,measured_length_m,measured_width_m,measured_height_m,measured_raw_points,display_direct,display_held,display_length_m,display_width_m,display_height_m,shrink_held,hold_elapsed_sec,state\n";
  }
  auto topic = declare_parameter<std::string>("input_topic", "/ch/lslidar_point_cloud");
  auto target = declare_parameter<std::string>("target_frame", "world");
  if (target != "world")
    throw std::invalid_argument("Crane V2 requires target_frame=world for the validated installation geometry");
  RCLCPP_INFO(get_logger(), "Crane V2 R15+C4: input=%s frame=world length_confirm=%.2fs shrink_hold=%.2fs; legacy F2 detector parameters are not used",
              topic.c_str(), config.cargo_length_confirm_sec, config.cargo_shrink_hold_sec);
  publisher_ = create_publisher<cx128s2_bev_perception::msg::CraneBeamStatus>("/crane_beam/status", 10);
  markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("/crane_hook/markers", 1);
  holder_clouds_[0]=create_publisher<sensor_msgs::msg::PointCloud2>("/crane_hook/left_body",1);
  holder_clouds_[1]=create_publisher<sensor_msgs::msg::PointCloud2>("/crane_hook/right_body",1);
  working_ = create_publisher<std_msgs::msg::String>("/crane_beam/working_status", 10);
  diagnostic_ = create_publisher<std_msgs::msg::String>("/crane_beam/diagnostics", 10);
  if (const char*path = std::getenv("CRANE_V2_AUDIT"))
    audit_.open(path);
  input_ = create_subscription<sensor_msgs::msg::PointCloud2>(topic, rclcpp::QoS(10), [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m){
      ++received_;
      auto start = std::chrono::steady_clock::now();
      try{
        if (!frozen_valid_)
        {
          auto t = tf_.lookupTransform("world", m->header.frame_id, tf2::TimePointZero);
          tf2::fromMsg(t.transform, frozen_);
          frozen_valid_ = true;
        }
        crane_v2::decode(*m, frozen_, points_);
        int64_t stamp = int64_t(m->header.stamp.sec) * 1000000000LL + m->header.stamp.nanosec;
        auto o = engine_.process(points_, stamp);
        // Rejected sensor timestamps must not publish a synthetic IDLE frame.
        // The Core leaves its history intact for all three rejection reasons.
        if (o.reason == "INVALID_SENSOR_TIMESTAMP"||o.reason == "DUPLICATE_TIMESTAMP"||o.reason == "REVERSED_TIMESTAMP")
        {
          RCLCPP_WARN(get_logger(), "V2 sensor frame rejected: %s", o.reason.c_str());
          return;
        }
        double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
        ++processed_;
        publisher_->publish(crane_v2::status(o, m->header));
        markers_->publish(crane_v2::markers(o, m->header, points_));
        if(holder_clouds_[0]->get_subscription_count()||holder_clouds_[1]->get_subscription_count()) {
          auto clouds=crane_v2::holderClouds(o,m->header,points_);
          for(int side=0;side<2;++side)holder_clouds_[side]->publish(clouds[side]);
        }
        std_msgs::msg::String working;working.data=o.state;working_->publish(working);
        std_msgs::msg::String diag;
        diag.data = "pure_cpp_ms=" + std::to_string(ms) + ";bottom_observation=" + o.cargo.bottom_observation;
        diag.data += ";crane_motion=" + o.crane_motion_reason + ";hist_score=" +
          std::to_string(o.crane_hist_score) + ";z_hist_score=" + std::to_string(o.crane_hist_axis_scores[2]) +
          ";x_hist_score=" + std::to_string(o.crane_hist_axis_scores[0]) +
          ";y_hist_score=" + std::to_string(o.crane_hist_axis_scores[1]) +
          ";hist_recent=" + std::to_string(o.crane_hist_recent_score) +
          ";background_points=" + std::to_string(o.crane_hist_points) + ";root_points=" +
          std::to_string(o.crane_root_points[0]) + "," + std::to_string(o.crane_root_points[1]);
        diagnostic_->publish(diag);
        if (shrink_audit_)
          shrink_audit_ << stamp << ',' << o.cargo.id << ',' << o.cargo_measurement_direct << ','
                        << o.measured_cargo_length << ',' << o.measured_cargo_width << ','
                        << o.measured_cargo_height << ',' << o.measured_cargo_points << ','
                        << o.cargo.direct << ',' << o.cargo.held << ','
                        << (o.cargo.length_locked ? o.cargo.model_length : o.cargo.length) << ','
                        << o.cargo.width << ',' << o.cargo.height << ',' << o.cargo_shrink_held << ','
                        << o.cargo_shrink_hold_elapsed_sec << ',' << o.state << '\n';
        if (audit_)
          audit_ << stamp << ',' << received_ << ',' << processed_ << ',' << ms << '\n';
      }
      catch (const std::exception&e){RCLCPP_ERROR(get_logger(), "V2 frame rejected: %s", e.what());
      }
    });
}
};
int main(int argc, char**argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CraneV2Experimental>());
  rclcpp::shutdown();
}
