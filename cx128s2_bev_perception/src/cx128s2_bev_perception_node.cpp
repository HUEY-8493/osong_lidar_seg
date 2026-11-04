#include "cx128s2_bev_perception/general_ground_noise_filter.hpp"
#include <cstddef>
#include <memory_resource>
#include <cassert>
#include <functional>
#include <algorithm>
#include <array>
#include <bitset>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <optional>
#include <omp.h>
#include <fstream>
#include <iterator>
#include <iomanip>
#include <limits>
#include <memory>
#include <numeric>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
#include <filesystem>
#include <stdexcept>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cx128s2_bev_perception/srv/commit_shape_memory.hpp>
#include <cx128s2_bev_perception/srv/control_shape_memory.hpp>
#include <cx128s2_bev_perception/srv/prepare_shape_memory_commit.hpp>

#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <yaml-cpp/yaml.h>

#include <cx128s2_bev_perception/msg/tracked_object.hpp>
#include <cx128s2_bev_perception/msg/tracked_object_array.hpp>
#include <cx128s2_bev_perception/msg/perception_recording_frame.hpp>
#include <cx128s2_bev_perception/msg/recording_object_cluster.hpp>
#include <cx128s2_bev_perception/msg/crane_beam_status.hpp>
#include <cx128s2_bev_perception/msg/crane_beam_motion_debug.hpp>
#include <cx128s2_bev_perception/linear_kalman_filter.hpp>
#include <cx128s2_bev_perception/vehicle_status.hpp>
#include <cx128s2_bev_perception/point_types.hpp>
#include <cx128s2_bev_perception/crane_beam_detector.hpp>
#include <cx128s2_bev_perception/crane_pipeline.hpp>
#include <cx128s2_bev_perception/crane_hybrid_policy.hpp>
#include <cx128s2_bev_perception/visibility_model.hpp>
#include <cx128s2_bev_perception/vehicle_surface_tracking.hpp>
#include <cx128s2_bev_perception/raw_vehicle_box_refinement.hpp>
#include <std_msgs/msg/int32.hpp>

namespace {

using cx128s2_bev_perception::PointXYZIRT;
using cx128s2_bev_perception::VisibilityInfo;
using cx128s2_bev_perception::VisibilityReason;
using cx128s2_bev_perception::VisibilityBox;
using cx128s2_bev_perception::VisibilityModel;
using cx128s2_bev_perception::VisibilityConfig;
using cx128s2_bev_perception::visibilityReasonName;

#include "general/data_types.inc.hpp"

#include "general/clustering.inc.hpp"

#include "crane_cargo_bev_scratch.inc.hpp"

#include "general/cluster_plugins.inc.hpp"

struct ScopedTimer {
  double & target_ms;
  std::chrono::steady_clock::time_point start;
  ScopedTimer(double & target) : target_ms(target), start(std::chrono::steady_clock::now()) {}
  ~ScopedTimer() {
    auto end = std::chrono::steady_clock::now();
    target_ms = std::chrono::duration<double, std::milli>(end - start).count();
  }
};

}  // namespace

struct QueuedCloud {
  sensor_msgs::msg::PointCloud2::SharedPtr msg;
  std::chrono::steady_clock::time_point enqueue_time;
  uint64_t sequence = 0;
};

class Cx128s2BevPerception : public rclcpp::Node {
  std::unique_ptr<CargoExtensionBevScratch>cargo_extension_bev_scratch_;
  LatestSuspendedPolicy crane_hybrid_policy_;
  friend struct TrackingRegressionAccess;
public:
  explicit Cx128s2BevPerception(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("cx128s2_bev_perception", options) {
    declareParams();
    loadParams();
    configureVisibility();
    if (crane_pipeline_name_=="height_surface" && crane_height_enabled_ && !visibility_enabled_)
      throw std::invalid_argument("Height surfaces require a configured VisibilityModel; check visibility driver/FOV settings");
    clusterer_ = createClusterer();

    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&Cx128s2BevPerception::onSetParameters, this, std::placeholders::_1));

    if (use_tf_transform_) {
      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    // The deployed LSLiDAR drivers and Crane bags offer reliable PointCloud2.
    // Match that contract in Crane mode; keep General's sensor QoS unchanged.
    const auto cloud_qos = perception_mode_ == "crane_beam_work" ?
        rclcpp::QoS(10) : rclcpp::QoS(rclcpp::SensorDataQoS());
    sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, cloud_qos,
        std::bind(&Cx128s2BevPerception::onCloud, this, std::placeholders::_1));

    pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("/debug/object_markers", 10);
    pub_detected_objects_ =
        create_publisher<cx128s2_bev_perception::msg::TrackedObjectArray>("/detected_objects", 10);
    pub_tracked_objects_ =
        create_publisher<cx128s2_bev_perception::msg::TrackedObjectArray>("/tracked_objects", 10);
    pub_motion_states_ =
        create_publisher<cx128s2_bev_perception::msg::TrackedObjectArray>("/object_motion_states", 10);
    pub_vehicle_tracks_ =
        create_publisher<cx128s2_bev_perception::msg::TrackedObjectArray>("/road_vehicle_tracks", 10);
    pub_vehicle_identity_memory_ =
        create_publisher<cx128s2_bev_perception::msg::TrackedObjectArray>("/vehicle_identity_memory", 10);
    // Keep the distinct intermediate cloud and reference boxes independently
    // of the optional detailed diagnostics/duplicate cloud stages.
    if (perception_mode_ != "crane_beam_work" && publish_debug_foreground_points_) {
      pub_foreground_ = create_publisher<sensor_msgs::msg::PointCloud2>("/debug/foreground_points", 10);
    }
    if (perception_mode_ != "crane_beam_work" && visibility_enabled_ && visibility_publish_markers_) {
      pub_visibility_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("/debug/visibility_markers", 10);
    }
    if (publish_debug_topics_) {
      pub_road_frame_ = createCloudPublisher(transformed_cloud_topic_);
      pub_valid_ = createCloudPublisher("/debug/valid_points");
      pub_roi_filtered_ = createCloudPublisher(roi_filtered_points_topic_);
      pub_static_background_ = createCloudPublisher("/debug/static_background_points");
      pub_clusters_ = createCloudPublisher("/debug/bev_clusters");
      pub_background_status_ = create_publisher<diagnostic_msgs::msg::KeyValue>("/background_status", 10);
      pub_object_status_ = create_publisher<diagnostic_msgs::msg::KeyValue>("/object_status", 10);
      pub_diagnostics_table_ = create_publisher<std_msgs::msg::String>(diagnostics_table_topic_, 10);
      pub_track_table_ = create_publisher<std_msgs::msg::String>("/debug/track_table", 10);
      pub_candidate_table_ = create_publisher<std_msgs::msg::String>("/debug/candidate_table", 10);
      pub_roi_marker_ = create_publisher<visualization_msgs::msg::MarkerArray>("/debug/roi_marker", 10);
      pub_visibility_ring_profile_ = create_publisher<std_msgs::msg::String>(
          "/debug/visibility_ring_profile", rclcpp::QoS(1).transient_local());
      pub_shape_graph_markers_ =
          create_publisher<visualization_msgs::msg::MarkerArray>("/debug/observed_shape_graph_markers", 10);
      pub_shape_memory_markers_ =
          create_publisher<visualization_msgs::msg::MarkerArray>("/debug/shape_memory_markers", 10);

    }
    if (publish_recording_frame_) {
      pub_recording_frame_ = create_publisher<cx128s2_bev_perception::msg::PerceptionRecordingFrame>(
          recording_frame_topic_, rclcpp::SensorDataQoS());
    }
    if (perception_mode_ == "crane_beam_work") {
      pub_crane_beam_status_ =
          create_publisher<cx128s2_bev_perception::msg::CraneBeamStatus>("/crane_beam/status", 10);
      pub_crane_beam_working_status_ =
          create_publisher<std_msgs::msg::String>("/crane_beam/working_status", 10);
      pub_crane_command_status_ = create_publisher<std_msgs::msg::String>(
          "/crane_beam/command_status", rclcpp::QoS(1).transient_local());
      // Only the opt-in Boundary boolean expires. Keep the original sensor header
      // and all Work/FSM fields: this is stale evidence, not a new measurement.
      if(crane_pipeline_ && crane_pipeline_name_=="slope" &&
          crane_pipeline_->config().work_status.boundary_temporal_enabled){
        crane_motion_expiry_timer_=create_wall_timer(std::chrono::milliseconds(100),[this](){
          std::lock_guard<std::mutex> lock(crane_motion_expiry_mutex_);
          if(!crane_motion_expiry_status_ || !crane_motion_expiry_status_->crane_moving)return;
          const double age=(get_clock()->now()-rclcpp::Time(crane_motion_expiry_status_->header.stamp,get_clock()->get_clock_type())).seconds();
          if(age>=crane_motion_expiry_timeout_ || age<0){
            crane_motion_expiry_status_->crane_moving=false;
            pub_crane_beam_status_->publish(*crane_motion_expiry_status_);
          }
        });
      }
      sub_crane_command_ = create_subscription<std_msgs::msg::String>(
          "/crane_beam/command", 10,
          std::bind(&Cx128s2BevPerception::handleCraneCommand, this, std::placeholders::_1));

      if (publish_debug_topics_ || crane_beam_config_.publish_debug) {
        pub_crane_beam_diagnostics_ =
            create_publisher<std_msgs::msg::String>("/crane_beam/diagnostics", 10);
        pub_crane_beam_motion_debug_ =
            create_publisher<cx128s2_bev_perception::msg::CraneBeamMotionDebug>("/crane_beam/debug/motion", 10);
        if (crane_pipeline_ && crane_pipeline_name_=="slope") {
          if (crane_slope_publish_debug_clouds_) {
            pub_crane_slope_ground_ = createCloudPublisher("/crane_slope/ground");
            pub_crane_slope_non_ground_ = createCloudPublisher("/crane_slope/non_ground");
            pub_crane_slope_unknown_ = createCloudPublisher("/crane_slope/unknown");
          }
          if (crane_slope_publish_colored_cloud_)
            pub_crane_slope_classified_ = createCloudPublisher("/crane_slope/classified");
          pub_crane_slope_clusters_ = createCloudPublisher("/crane_slope/clusters");
          if (crane_plane_enabled_ && crane_plane_publish_debug_clouds_) {
            pub_crane_plane_ = createCloudPublisher("/crane_plane/plane");
            pub_crane_line_only_ = createCloudPublisher("/crane_plane/line_only");
            pub_crane_non_planar_ = createCloudPublisher("/crane_plane/non_planar");
            pub_crane_plane_regions_ = create_publisher<std_msgs::msg::String>("/crane_plane/regions", 10);
          }
          if (crane_plane_enabled_ && crane_plane_publish_colored_cloud_)
            pub_crane_surface_classified_ = createCloudPublisher("/crane_plane/classified");
          if (crane_slope_publish_direction_clouds_) {
            pub_crane_slope_vertical_ = createCloudPublisher("/crane_slope/vertical");
            pub_crane_slope_horizontal_ = createCloudPublisher("/crane_slope/horizontal");
          }
          pub_crane_slope_boundary_ = createCloudPublisher("/crane_slope/boundary");
          pub_crane_slope_boundary_clusters_ = createCloudPublisher("/crane_slope/boundary_clusters");
          pub_crane_slope_z_gaussian_layers_ = createCloudPublisher("/crane_slope/z_gaussian_layers");
          pub_crane_slope_closed_loop_groups_ = createCloudPublisher("/crane_slope/closed_loop_groups");
          pub_crane_slope_boundary_diagnostics_ = create_publisher<std_msgs::msg::String>("/crane_slope/boundary_diagnostics", 10);
        }
        if (crane_pipeline_ && crane_pipeline_name_=="height_surface" && crane_height_publish_debug_) {
          pub_crane_height_candidates_=createCloudPublisher("/crane_height/candidates");
          pub_crane_height_surfaces_=createCloudPublisher("/crane_height/surfaces");
          pub_crane_height_unassigned_=createCloudPublisher("/crane_height/unassigned");
          pub_crane_height_regions_=create_publisher<std_msgs::msg::String>("/crane_height/regions",10);
        }
        pub_crane_beam_roi_points_ = createCloudPublisher("/crane_beam/debug/beam_roi_points");
        pub_crane_beam_bg_roi_points_ = createCloudPublisher("/crane_beam/debug/background_roi_points");
        pub_crane_beam_z_candidates_ = createCloudPublisher("/crane_beam/debug/z_candidate_points");
        pub_crane_beam_cluster_ = createCloudPublisher("/crane_beam/debug/beam_cluster");
        pub_crane_beam_support_points_ = createCloudPublisher("/crane_beam/debug/support_points");
        pub_crane_beam_markers_ =
            create_publisher<visualization_msgs::msg::MarkerArray>("/crane_beam/debug/markers", 10);
        pub_crane_hook_left_body_ = createCloudPublisher("/crane_hook/left_body");
        pub_crane_hook_right_body_ = createCloudPublisher("/crane_hook/right_body");
        pub_crane_hook_markers_ =
            create_publisher<visualization_msgs::msg::MarkerArray>("/crane_hook/markers", 10);
      }
    }

    if (!crane_pipeline_) {
      crane_beam_detector_ =
          std::make_unique<cx128s2_bev_perception::CraneBeamDetector>(crane_beam_config_);
      if (perception_mode_ == "crane_beam_work") {
        std::string report;
        if (crane_beam_detector_->getHookDetector().loadReference(crane_hook_reference_file_, target_frame_, report))
          RCLCPP_INFO(get_logger(), "[HOOK_REFERENCE] %s", report.c_str());
        else RCLCPP_WARN(get_logger(), "[HOOK_REFERENCE] %s", report.c_str());
      }
    }


    srv_learn_background_ = create_service<std_srvs::srv::Trigger>(
        "/learn_static_background",
        std::bind(&Cx128s2BevPerception::handleLearnBackground, this, std::placeholders::_1,
                  std::placeholders::_2));
    srv_save_background_ = create_service<std_srvs::srv::Trigger>(
        "/save_static_background",
        std::bind(&Cx128s2BevPerception::handleSaveBackground, this, std::placeholders::_1,
                  std::placeholders::_2));
    srv_load_background_ = create_service<std_srvs::srv::Trigger>(
        "/load_static_background",
        std::bind(&Cx128s2BevPerception::handleLoadBackground, this, std::placeholders::_1,
                  std::placeholders::_2));
    srv_clear_background_ = create_service<std_srvs::srv::Trigger>(
        "/clear_static_background",
        std::bind(&Cx128s2BevPerception::handleClearBackground, this, std::placeholders::_1,
                  std::placeholders::_2));
    srv_dump_background_ = create_service<std_srvs::srv::Trigger>(
        "/dump_background_table",
        std::bind(&Cx128s2BevPerception::handleDumpBackground, this, std::placeholders::_1,
                  std::placeholders::_2));
    srv_dump_track_ = create_service<std_srvs::srv::Trigger>(
        "/dump_track_table",
        std::bind(&Cx128s2BevPerception::handleDumpTrack, this, std::placeholders::_1,
                  std::placeholders::_2));
    srv_dump_candidate_ = create_service<std_srvs::srv::Trigger>(
        "/dump_candidate_table",
        std::bind(&Cx128s2BevPerception::handleDumpCandidate, this, std::placeholders::_1,
                  std::placeholders::_2));
    srv_clear_roi_ = create_service<std_srvs::srv::Trigger>(
        "/clear_roi",
        std::bind(&Cx128s2BevPerception::handleClearRoi, this, std::placeholders::_1,
                  std::placeholders::_2));
    srv_save_roi_ = create_service<std_srvs::srv::Trigger>(
        "/save_roi",
        std::bind(&Cx128s2BevPerception::handleSaveRoi, this, std::placeholders::_1,
                  std::placeholders::_2));
    srv_load_roi_ = create_service<std_srvs::srv::Trigger>(
        "/load_roi",
        std::bind(&Cx128s2BevPerception::handleLoadRoi, this, std::placeholders::_1,
                  std::placeholders::_2));

    srv_prepare_shape_memory_commit_ = create_service<cx128s2_bev_perception::srv::PrepareShapeMemoryCommit>(
        "/prepare_shape_memory_commit",
        std::bind(&Cx128s2BevPerception::handlePrepareShapeMemoryCommit, this, std::placeholders::_1,
                  std::placeholders::_2));

    srv_commit_shape_memory_ = create_service<cx128s2_bev_perception::srv::CommitShapeMemory>(
        "/commit_shape_memory",
        std::bind(&Cx128s2BevPerception::handleCommitShapeMemory, this, std::placeholders::_1,
                  std::placeholders::_2));
    srv_clear_shape_memory_ = create_service<cx128s2_bev_perception::srv::ControlShapeMemory>(
        "/clear_shape_memory",
        std::bind(&Cx128s2BevPerception::handleClearShapeMemory, this, std::placeholders::_1,
                  std::placeholders::_2));
    srv_freeze_shape_memory_ = create_service<cx128s2_bev_perception::srv::ControlShapeMemory>(
        "/freeze_shape_memory",
        std::bind(&Cx128s2BevPerception::handleFreezeShapeMemory, this, std::placeholders::_1,
                  std::placeholders::_2));
    srv_unfreeze_shape_memory_ = create_service<cx128s2_bev_perception::srv::ControlShapeMemory>(
        "/unfreeze_shape_memory",
        std::bind(&Cx128s2BevPerception::handleUnfreezeShapeMemory, this, std::placeholders::_1,
                  std::placeholders::_2));
    srv_dump_shape_memory_ = create_service<cx128s2_bev_perception::srv::ControlShapeMemory>(
        "/dump_shape_memory",
        std::bind(&Cx128s2BevPerception::handleDumpShapeMemory, this, std::placeholders::_1,
                  std::placeholders::_2));
    srv_save_object_memory_bank_ = create_service<std_srvs::srv::Trigger>(
        "/save_object_memory_bank",
        std::bind(&Cx128s2BevPerception::handleSaveObjectMemoryBank, this, std::placeholders::_1,
                  std::placeholders::_2));
    srv_load_object_memory_bank_ = create_service<std_srvs::srv::Trigger>(
        "/load_object_memory_bank",
        std::bind(&Cx128s2BevPerception::handleLoadObjectMemoryBank, this, std::placeholders::_1,
                  std::placeholders::_2));
    srv_clear_object_memory_bank_ = create_service<std_srvs::srv::Trigger>(
        "/clear_object_memory_bank",
        std::bind(&Cx128s2BevPerception::handleClearObjectMemoryBank, this, std::placeholders::_1,
                  std::placeholders::_2));
    srv_dump_object_memory_bank_ = create_service<std_srvs::srv::Trigger>(
        "/dump_object_memory_bank",
        std::bind(&Cx128s2BevPerception::handleDumpObjectMemoryBank, this, std::placeholders::_1,
                  std::placeholders::_2));
    srv_reset_tracking_ = create_service<std_srvs::srv::Trigger>(
        "/reset_tracking",
        std::bind(&Cx128s2BevPerception::handleResetTracking, this, std::placeholders::_1,
                  std::placeholders::_2));
    srv_reset_bev_components_ = create_service<std_srvs::srv::Trigger>(
        "/reset_bev_components",
        std::bind(&Cx128s2BevPerception::handleResetBevComponents, this, std::placeholders::_1,
                  std::placeholders::_2));
    srv_reset_perception_state_ = create_service<std_srvs::srv::Trigger>(
        "/reset_perception_state",
        std::bind(&Cx128s2BevPerception::handleResetPerceptionState, this, std::placeholders::_1,
                  std::placeholders::_2));

    if (usesStaticBackground() && auto_load_background_on_start_) {
      std::string err;
      if (!loadBackground(err)) {
        RCLCPP_WARN(this->get_logger(), "Background auto-load failed: %s", err.c_str());
      }
    }
    if (use_roi_policy_) {
      std::string err;
      if (!loadRoiPolicy(err)) {
        RCLCPP_WARN(this->get_logger(), "ROI policy auto-load failed: %s", err.c_str());
      }
    }
    if (object_memory_bank_enabled_ && auto_load_object_memory_bank_on_start_) {
      std::string err;
      if (!loadObjectMemoryBank(err)) {
        RCLCPP_WARN(this->get_logger(), "Object memory bank auto-load failed: %s", err.c_str());
      }
    }

    RCLCPP_INFO(this->get_logger(),
                "cx128s2_bev_perception started. input=%s target_frame=%s background_mode=%s manual_background=%s "
                "background_learning_input_mode=%s roi_policy=%s zones=%zu",
                input_topic_.c_str(), target_frame_.c_str(), background_mode_.c_str(),
                require_manual_background_learning_ ? "true" : "false",
                background_learning_input_mode_.c_str(), use_roi_policy_ ? "true" : "false",
                roi_zones_.size());

    // Start the perception worker thread
    worker_thread_ = std::thread(&Cx128s2BevPerception::perceptionWorkerLoop, this);
  }

  ~Cx128s2BevPerception() override {
    // 1. Terminate and join the worker thread first to prevent accessing destroyed member states
    worker_running_ = false;
    latest_msg_cv_.notify_all();
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }

    // 2. Perform shutdown saves
    if (save_csv_on_shutdown_) {
      writeTrackCsv(track_table_file_);
      writeCandidateCsv(candidate_table_file_);
    }
  }

private:
  void configureVisibility() {
    visibility_enabled_ = get_parameter("visibility.enabled").as_bool();
    visibility_publish_markers_ = get_parameter("visibility.publish_markers").as_bool();
    visibility_adaptive_loss_enabled_ = get_parameter("visibility.adaptive_loss_enabled").as_bool();
    if (!visibility_enabled_) return;
    VisibilityConfig cfg;
    cfg.sensor_frame = get_parameter("visibility.sensor_frame").as_string();
    cfg.elevation_min_deg = get_parameter("visibility.elevation_min_deg").as_double();
    cfg.elevation_max_deg = get_parameter("visibility.elevation_max_deg").as_double();
    cfg.elevation_bin_deg = get_parameter("visibility.elevation_bin_deg").as_double();
    cfg.max_range_m = get_parameter("visibility.physical_max_range_m").as_double();
    cfg.range_noise_m = get_parameter("visibility.range_noise_m").as_double();
    cfg.ground_z = get_parameter("visibility.ground_z").as_double();
    cfg.static_xy_resolution_m = std::max(bev_x_res_, bev_y_res_);
    cfg.sample_heights_m = {cfg.ground_z, cfg.ground_z + vehicle_min_height_,
                            cfg.ground_z + (vehicle_min_height_ + vehicle_max_height_) * 0.5,
                            cfg.ground_z + vehicle_max_height_};
    try {
      std::string path = get_parameter("visibility.driver_config_file").as_string();
      if (path.empty()) {
        path = ament_index_cpp::get_package_share_directory("lslidar_driver") +
            "/config/lslidar_ch.yaml";
      }
      std::ifstream driver_file(path);
      if (!driver_file) throw std::runtime_error("cannot open driver config: " + path);
      std::string driver_yaml((std::istreambuf_iterator<char>(driver_file)),
                              std::istreambuf_iterator<char>());
      // The shipped driver YAML contains a tab before a comment. yaml-cpp
      // rejects it; normalize only our in-memory copy.
      std::replace(driver_yaml.begin(), driver_yaml.end(), '\t', ' ');
      const auto root = YAML::Load(driver_yaml);
      const auto params = root["ch"]["lslidar_driver_node"]["ros__parameters"];
      if (!params || !params["scan_start_angle"] || !params["scan_end_angle"] ||
          !params["min_range"] || !params["max_range"] || !params["frame_id"]) {
        throw std::runtime_error("missing LiDAR sector/range/frame parameters");
      }
      if (params["frame_id"].as<std::string>() != cfg.sensor_frame ||
          (params["is_pretreatment"] && params["is_pretreatment"].as<bool>()) ||
          (params["is_MatrixTransformation"] && params["is_MatrixTransformation"].as<bool>())) {
        throw std::runtime_error("driver frame or internal coordinate transform differs from visibility model");
      }
      cfg.azimuth_min_deg = params["scan_start_angle"].as<double>() * 0.01;
      cfg.azimuth_max_deg = params["scan_end_angle"].as<double>() * 0.01;
      cfg.min_range_m = params["min_range"].as<double>();
      cfg.max_range_m = std::min(cfg.max_range_m, params["max_range"].as<double>());
      if (cfg.azimuth_min_deg >= cfg.azimuth_max_deg ||
          cfg.elevation_min_deg >= cfg.elevation_max_deg ||
          cfg.min_range_m >= cfg.max_range_m) {
        throw std::runtime_error("invalid FOV/range bounds");
      }
      // The vendor vertical envelope is only a possible FOV. Runtime DIFOP
      // prism angles are not exposed here, so exact channel coverage is unknown.
      cfg.calibration_verified = false;
      visibility_model_.configure(cfg);
      RCLCPP_INFO(get_logger(), "Visibility model: driver=%s az=[%.2f,%.2f] el=[%.2f,%.2f] range=[%.2f,%.2f] calibration=unverified",
                  path.c_str(), cfg.azimuth_min_deg, cfg.azimuth_max_deg,
                  cfg.elevation_min_deg, cfg.elevation_max_deg, cfg.min_range_m, cfg.max_range_m);
    } catch (const std::exception & e) {
      visibility_enabled_ = false;
      RCLCPP_WARN(get_logger(), "Visibility disabled: %s", e.what());
    }
  }

  void estimateVisibilityAngularSpacing(const std::vector<PointXYZIRT> & points,
                                        const tf2::Transform & world_from_sensor) {
    if (!visibility_frame_valid_ || !last_input_has_ring_ || visibility_azimuth_bin_measured_) return;
    std::vector<double> azimuths;
    azimuths.reserve(4096);
    const auto sensor_from_world = world_from_sensor.inverse();
    for (const auto & p : points) {
      if (p.ring != 0) continue;
      const auto local = sensor_from_world * tf2::Vector3(p.x, p.y, p.z);
      double az = std::atan2(local.y(), local.x()) * 180.0 / M_PI;
      if (az < 0.0) az += 360.0;
      if (az >= visibility_model_.config().azimuth_min_deg &&
          az <= visibility_model_.config().azimuth_max_deg) azimuths.push_back(az);
      if (azimuths.size() >= 4096) break;
    }
    if (azimuths.size() < 40) return;
    std::sort(azimuths.begin(), azimuths.end());
    std::vector<double> gaps;
    gaps.reserve(azimuths.size());
    for (size_t i = 1; i < azimuths.size(); ++i) {
      const double d = azimuths[i] - azimuths[i - 1];
      if (d >= 0.02 && d <= 0.5) gaps.push_back(d);
    }
    if (gaps.size() < 20) return;
    auto mid = gaps.begin() + gaps.size() / 2;
    std::nth_element(gaps.begin(), mid, gaps.end());
    const double spacing = *mid;
    if (spacing < 0.04 || spacing > 0.4) return;
    auto cfg = visibility_model_.config();
    cfg.azimuth_bin_deg = spacing;
    visibility_model_.configure(cfg);
    visibility_azimuth_bin_measured_ = true;
    RCLCPP_INFO(get_logger(), "Visibility azimuth bin measured from ring 0 cloud: %.4f deg (%zu gaps)",
                spacing, gaps.size());
  }

  void publishVisibilityRingProfile(const std::vector<PointXYZIRT> & points,
                                    const tf2::Transform & world_from_sensor) {
    if ((perception_mode_ == "crane_beam_work" && !pub_visibility_ring_profile_) ||
        !visibility_frame_valid_ || !last_input_has_ring_ ||
        visibility_ring_profile_published_) return;
    std::array<std::vector<double>, 128> elevations;
    const auto sensor_from_world = world_from_sensor.inverse();
    for (const auto & p : points) {
      if (p.ring >= elevations.size() || elevations[p.ring].size() >= 512) continue;
      const auto local = sensor_from_world * tf2::Vector3(p.x, p.y, p.z);
      elevations[p.ring].push_back(std::atan2(local.z(), std::hypot(local.x(), local.y())) * 180.0 / M_PI);
    }
    std::ostringstream out;
    out << "ring,count,median_elevation_deg,min_elevation_deg,max_elevation_deg\n";
    int valid_rings = 0;
    std::vector<double> ring_medians;
    std::vector<std::pair<double,double>> ring_bands;
    for (size_t ring = 0; ring < elevations.size(); ++ring) {
      auto & v = elevations[ring];
      if (v.empty()) continue;
      ++valid_rings;
      std::sort(v.begin(), v.end());
      ring_medians.push_back(v[v.size() / 2]);
      ring_bands.emplace_back(v.front(),v.back());
      out << ring << "," << v.size() << "," << v[v.size() / 2]
          << "," << v.front() << "," << v.back() << "\n";
    }
    if (valid_rings < 16) return;
    if (perception_mode_ != "crane_beam_work")
      visibility_model_.setMeasuredRingProfile(ring_bands);
    std::sort(ring_medians.begin(), ring_medians.end());
    std::vector<double> ring_gaps;
    for (size_t i = 1; i < ring_medians.size(); ++i) {
      const double gap = ring_medians[i] - ring_medians[i - 1];
      if (gap >= 0.02 && gap <= 2.0) ring_gaps.push_back(gap);
    }
    if (ring_gaps.size() >= 16) {
      auto mid = ring_gaps.begin() + ring_gaps.size() / 2;
      std::nth_element(ring_gaps.begin(), mid, ring_gaps.end());
      visibility_model_.setRayElevationSpacing(*mid);
    }
    std_msgs::msg::String msg;
    msg.data = out.str();
    if (pub_visibility_ring_profile_) pub_visibility_ring_profile_->publish(msg);
    visibility_ring_profile_published_ = true;
  }

  void prewarmVisibilityRoi() {
    // Cache a bounded, height-aware grid for fixed watch zones. Arbitrary box
    // samples are still computed lazily at the native BEV resolution.
    size_t sampled = 0;
    visibility_static_debug_cells_.clear();
    const auto sample_area = [&](double x_min, double x_max, double y_min, double y_max,
                                 const RoiZone * source_zone) {
      if (!(x_min < x_max && y_min < y_max)) return;
      const double area = (x_max - x_min) * (y_max - y_min);
      const double step = std::max(1.0, std::sqrt(area / 4096.0));
      for (double x = x_min + step * 0.5; x < x_max && sampled < 4096; x += step) {
        for (double y = y_min + step * 0.5; y < y_max && sampled < 4096; y += step) {
          if (source_zone && !pointInZone(x, y, *source_zone)) continue;
          if (use_roi_policy_ && pointPolicy(x, y) != RoiDecision::KEEP) continue;
          if (!use_roi_policy_ && roi_enabled_ && !insideRoi(x, y)) continue;
          ++sampled;
          for (double z : visibility_model_.config().sample_heights_m) {
            const auto info = visibility_model_.cacheStaticPoint(x, y, z);
            if (sampled % 64 == 0 && visibility_static_debug_cells_.size() < 256) {
              visibility_static_debug_cells_.emplace_back(tf2::Vector3(x, y, z), info.reason);
            }
          }
        }
      }
    };
    if (use_roi_policy_ && !roi_zones_.empty()) {
      for (const auto & zone : roi_zones_) {
        if (!zone.enabled || zone.type != "watch") continue;
        if (zone.shape == "box") {
          sample_area(zone.x_min, zone.x_max, zone.y_min, zone.y_max, &zone);
        } else if (!zone.polygon_x.empty() && !zone.polygon_y.empty()) {
          const auto [min_x, max_x] = std::minmax_element(zone.polygon_x.begin(), zone.polygon_x.end());
          const auto [min_y, max_y] = std::minmax_element(zone.polygon_y.begin(), zone.polygon_y.end());
          sample_area(*min_x, *max_x, *min_y, *max_y, &zone);
        }
        if (sampled >= 4096) break;
      }
    } else if (roi_enabled_ && roi_shape_ == "box") {
      sample_area(roi_x_min_, roi_x_max_, roi_y_min_, roi_y_max_, nullptr);
    }
  }

#include "general/configuration.inc.hpp"

#include "general/input_adapter.inc.hpp"

#include "general/frame_pipeline.inc.hpp"

#include "general/foreground.inc.hpp"

  bool isVehicleLikeMemoryLabel(const std::string & label) const {
    return label == "TRUCK" || label == "VEHICLE" || label == "TRUCK_LIKE" ||
           label == "VEHICLE_LIKE";
  }

  bool mirrorAllowedForClass(const ShapeMemory & mem) const {
    if (!shape_memory_mirror_mode_enabled_) return false;
    const std::string label = mem.class_label != "UNKNOWN" ? mem.class_label : mem.class_prior;
    return std::find(shape_memory_mirror_only_for_class_labels_.begin(),
                     shape_memory_mirror_only_for_class_labels_.end(), label) !=
           shape_memory_mirror_only_for_class_labels_.end();
  }

  std::string mirrorPlaneFromDeprecatedAxis(const std::string & axis) const {
    if (axis == "local_x") return "yz";
    if (axis == "local_y") return "xz";
    if (axis == "local_z") return "xy";
    return "xz";
  }

  bool validMirrorPlane(const std::string & plane) const {
    return plane == "xy" || plane == "xz" || plane == "yz";
  }

  double flippedAxisValue(const ShapeMemoryNode & n, const MirrorModeInfo & info) const {
    if (info.flipped_axis == "local_x") return n.local_x;
    if (info.flipped_axis == "local_y") return n.local_y;
    if (info.flipped_axis == "local_z") return n.local_z;
    return 0.0;
  }

  MirrorModeInfo estimateMirrorPlaneInfo(const ShapeMemory & mem, const std::string & plane) const {
    MirrorModeInfo info;
    info.enabled = true;
    info.plane = plane;
    info.center_x = 0.5 * (mem.min_local_x + mem.max_local_x);
    info.center_y = 0.5 * (mem.min_local_y + mem.max_local_y);
    info.center_z = 0.5 * (mem.min_local_z + mem.max_local_z);
    info.mirror_centerline_x = info.center_x;
    info.mirror_centerline_y = info.center_y;
    info.mirror_centerline_z = info.center_z;

    if (plane == "xy") {
      info.flipped_axis = "local_z";
      info.flipped_axis_center = info.center_z;
    } else if (plane == "xz") {
      info.flipped_axis = "local_y";
      info.flipped_axis_center = info.center_y;
    } else if (plane == "yz") {
      info.flipped_axis = "local_x";
      info.flipped_axis_center = info.center_x;
    }
    info.mirror_axis = info.flipped_axis;
    info.mirror_centerline = info.flipped_axis_center;
    return info;
  }

  ShapeMemoryNode mirrorNodeByPlane(
      const ShapeMemoryNode & src, const MirrorModeInfo & info) const {
    ShapeMemoryNode dst = src;
    if (info.plane == "xy") {
      dst.local_z = 2.0 * info.center_z - src.local_z;
    } else if (info.plane == "xz") {
      dst.local_y = 2.0 * info.center_y - src.local_y;
    } else if (info.plane == "yz") {
      dst.local_x = 2.0 * info.center_x - src.local_x;
    }
    dst.is_virtual = true;
    dst.is_mirrored = true;
    dst.mirrored_from_node_id = src.id;
    dst.visibility_confidence = shape_memory_mirror_virtual_node_confidence_;
    dst.confidence = std::min(src.confidence * shape_memory_mirror_confidence_scale_,
                              shape_memory_mirror_virtual_node_confidence_);
    dst.observed_count = 0;
    dst.missing_count = 0;
    dst.point_count = 0;
    dst.semantic_hint = src.semantic_hint == "unknown" ? "mirrored_soft_prior"
                                                       : src.semantic_hint + "_mirrored";
    return dst;
  }

  ShapeMemoryEdge makeMirroredEdgeFromMirroredNodes(
      const ShapeMemoryEdge & src_edge,
      const ShapeMemoryNode & mirrored_a,
      const ShapeMemoryNode & mirrored_b) const {
    ShapeMemoryEdge edge = src_edge;
    edge.node_a = mirrored_a.id;
    edge.node_b = mirrored_b.id;
    edge.local_dx = mirrored_b.local_x - mirrored_a.local_x;
    edge.local_dy = mirrored_b.local_y - mirrored_a.local_y;
    edge.local_dz = mirrored_b.local_z - mirrored_a.local_z;
    edge.length = std::sqrt(edge.local_dx * edge.local_dx +
                            edge.local_dy * edge.local_dy +
                            edge.local_dz * edge.local_dz);
    edge.direction_yaw = std::atan2(edge.local_dy, edge.local_dx);
    edge.height_delta = edge.local_dz;
    edge.confidence = std::min(src_edge.confidence * shape_memory_mirror_confidence_scale_,
                               shape_memory_mirror_virtual_edge_confidence_);
    edge.observed_count = 0;
    edge.missing_count = 0;
    edge.is_virtual = true;
    edge.is_mirrored = true;
    edge.mirrored_from_edge_id = src_edge.id;
    edge.visibility_confidence = shape_memory_mirror_virtual_edge_confidence_;
    return edge;
  }

  void validateMirroredGeometry(
      const ShapeMemory & before, const ShapeMemory & after, MirrorModeInfo & info) const {
    std::unordered_map<int, const ShapeMemoryNode *> before_nodes;
    for (const auto & n : before.nodes) before_nodes[n.id] = &n;
    int debug_count = 0;
    for (const auto & n : after.nodes) {
      if (!n.is_mirrored || n.mirrored_from_node_id < 0) continue;
      const auto it = before_nodes.find(n.mirrored_from_node_id);
      if (it == before_nodes.end()) continue;
      const auto & src = *it->second;
      const double delta = flippedAxisValue(n, info) - flippedAxisValue(src, info);
      if (shape_memory_mirror_warn_if_near_identical_ && std::abs(delta) < 1e-3) {
        info.near_identical_warning_count++;
        RCLCPP_WARN(get_logger(),
                    "Mirror plane generated near-identical node. plane=%s flipped_axis=%s "
                    "src_id=%d dst_id=%d center=%.3f",
                    info.plane.c_str(), info.flipped_axis.c_str(), src.id, n.id,
                    info.flipped_axis_center);
      } else if (debug_count < shape_memory_mirror_debug_samples_) {
        RCLCPP_DEBUG(get_logger(),
                     "[MIRROR_PLANE] plane=%s flipped_axis=%s center=(%.3f, %.3f, %.3f) "
                     "src_node=%d src=(%.3f, %.3f, %.3f) dst_node=%d dst=(%.3f, %.3f, %.3f) "
                     "delta_flipped_axis=%.3f",
                     info.plane.c_str(), info.flipped_axis.c_str(), info.center_x, info.center_y,
                     info.center_z, src.id, src.local_x, src.local_y, src.local_z,
                     n.id, n.local_x, n.local_y, n.local_z, delta);
        debug_count++;
      }
    }

    std::unordered_set<int> mirrored_ids;
    for (const auto & n : after.nodes) {
      if (n.is_mirrored) mirrored_ids.insert(n.id);
    }
    for (const auto & e : after.edges) {
      if (!e.is_mirrored) continue;
      if (mirrored_ids.count(e.node_a) == 0 || mirrored_ids.count(e.node_b) == 0) {
        info.invalid_edge_warning_count++;
        RCLCPP_WARN(get_logger(),
                    "Mirrored edge connects non-mirrored node. edge=%d nodes=(%d,%d)",
                    e.id, e.node_a, e.node_b);
      }
    }
  }

  void augmentShapeMemoryWithMirrorMode(ShapeMemory & mem, MirrorModeInfo * info = nullptr) const {
    if (mem.mirror_info.enabled) {
      if (info) *info = mem.mirror_info;
      return;
    }
    if (!mirrorAllowedForClass(mem)) return;
    const std::string plane = shape_memory_mirror_plane_.empty() ?
        mirrorPlaneFromDeprecatedAxis(shape_memory_mirror_axis_) : shape_memory_mirror_plane_;
    if (!validMirrorPlane(plane)) {
      RCLCPP_WARN(get_logger(), "Invalid shape_memory_mirror_plane='%s'. Mirror skipped.",
                  plane.c_str());
      return;
    }
    const int observed_node_count = static_cast<int>(std::count_if(
        mem.nodes.begin(), mem.nodes.end(), [](const ShapeMemoryNode & n) { return !n.is_virtual; }));
    if (observed_node_count < shape_memory_mirror_min_observed_nodes_) return;

    const double observed_width = std::max(1e-3, mem.max_local_y - mem.min_local_y);
    if (observed_width < shape_memory_mirror_min_observed_width_ratio_ * std::max(0.1, mem.width)) {
      return;
    }

    const ShapeMemory before = mem;
    MirrorModeInfo local_info = estimateMirrorPlaneInfo(mem, plane);
    local_info.symmetry_confidence = shape_memory_mirror_confidence_scale_;
    local_info.observed_node_count = observed_node_count;
    local_info.observed_edge_count = static_cast<int>(std::count_if(
        mem.edges.begin(), mem.edges.end(), [](const ShapeMemoryEdge & e) { return !e.is_virtual; }));
    int next_node_id = 0;
    int next_edge_id = 0;
    for (const auto & n : mem.nodes) next_node_id = std::max(next_node_id, n.id + 1);
    for (const auto & e : mem.edges) next_edge_id = std::max(next_edge_id, e.id + 1);
    const std::vector<ShapeMemoryNode> original_nodes = mem.nodes;
    const std::vector<ShapeMemoryEdge> original_edges = mem.edges;
    std::unordered_map<int, ShapeMemoryNode> mirrored_node_by_original_id;

    for (const auto & n : original_nodes) {
      if (n.is_virtual) continue;
      ShapeMemoryNode mirrored = mirrorNodeByPlane(n, local_info);
      mirrored.id = next_node_id++;
      mirrored_node_by_original_id[n.id] = mirrored;
      mem.nodes.push_back(mirrored);
      local_info.mirrored_node_count++;
    }

    for (const auto & e : original_edges) {
      if (e.is_virtual) continue;
      const auto na = mirrored_node_by_original_id.find(e.node_a);
      const auto nb = mirrored_node_by_original_id.find(e.node_b);
      if (na == mirrored_node_by_original_id.end() || nb == mirrored_node_by_original_id.end()) {
        local_info.invalid_edge_warning_count++;
        continue;
      }
      ShapeMemoryEdge mirrored = makeMirroredEdgeFromMirroredNodes(e, na->second, nb->second);
      mirrored.id = next_edge_id++;
      mem.edges.push_back(mirrored);
      local_info.mirrored_edge_count++;
      if (local_info.mirrored_edge_count <= shape_memory_mirror_debug_samples_) {
        RCLCPP_DEBUG(get_logger(),
                     "[MIRROR_EDGE] src_edge=%d src_nodes=(%d,%d) mirrored_edge=%d "
                     "mirrored_nodes=(%d,%d) recomputed_dx=%.3f recomputed_dy=%.3f recomputed_yaw=%.3f",
                     e.id, e.node_a, e.node_b, mirrored.id, mirrored.node_a, mirrored.node_b,
                     mirrored.local_dx, mirrored.local_dy, mirrored.direction_yaw);
      }
    }

    validateMirroredGeometry(before, mem, local_info);
    mem.mirror_info = local_info;
    if (info) *info = local_info;
  }

  HeightProfile computeHeightProfileFromIndices(
      const std::vector<PointXYZIRT> & points,
      const std::vector<int> & indices,
      double center_x,
      double min_x,
      double max_x) const {
    HeightProfile hp;
    if (indices.empty()) return hp;
    std::vector<double> zs;
    zs.reserve(indices.size());
    double z_sum = 0.0;
    double z2_sum = 0.0;
    double front_max = std::numeric_limits<double>::lowest();
    double mid_max = std::numeric_limits<double>::lowest();
    double rear_max = std::numeric_limits<double>::lowest();
    const double len = std::max(0.1, max_x - min_x);
    const double rear_cut = min_x + len / 3.0;
    const double front_cut = max_x - len / 3.0;
    for (int idx : indices) {
      if (idx < 0 || idx >= static_cast<int>(points.size())) continue;
      const auto & p = points[idx];
      const double rel_z = p.z;
      z_sum += rel_z;
      z2_sum += rel_z * rel_z;
      hp.max_height = std::max(hp.max_height, rel_z);
      if (p.x < rear_cut) rear_max = std::max(rear_max, rel_z);
      else if (p.x > front_cut) front_max = std::max(front_max, rel_z);
      else mid_max = std::max(mid_max, rel_z);
      zs.push_back(rel_z);
    }
    const double n = std::max<size_t>(1, zs.size());
    const double mean = z_sum / n;
    hp.height_variance = std::max(0.0, z2_sum / n - mean * mean);
    hp.front_height = front_max == std::numeric_limits<double>::lowest() ? hp.max_height : front_max;
    hp.middle_height = mid_max == std::numeric_limits<double>::lowest() ? hp.max_height : mid_max;
    hp.rear_height = rear_max == std::numeric_limits<double>::lowest() ? hp.max_height : rear_max;
    hp.front_rear_height_delta = hp.front_height - hp.rear_height;
    (void)center_x;
    return hp;
  }

  double computeCargoScore(const HeightProfile & memory_profile,
                           const HeightProfile & current_profile) const {
    const double middle_delta = current_profile.middle_height - memory_profile.middle_height;
    const double rear_delta = current_profile.rear_height - memory_profile.rear_height;
    const double max_delta = current_profile.max_height - memory_profile.max_height;
    const double score = 0.45 * clamp(middle_delta / std::max(0.1, cargo_height_delta_threshold_), 0.0, 1.0) +
                         0.35 * clamp(rear_delta / std::max(0.1, cargo_height_delta_threshold_), 0.0, 1.0) +
                         0.20 * clamp(max_delta / std::max(0.1, cargo_height_delta_threshold_), 0.0, 1.0);
    return clamp(score, 0.0, 1.0);
  }

  ObjectFeature makeFeatureFromClusterForMemoryMatch(
      const std::vector<PointXYZIRT> & points,
      const ClusterResult & cluster) const {
    ObjectFeature f;
    f.id = cluster.id;
    f.indices = cluster.indices;
    f.point_count = static_cast<int>(cluster.indices.size());
    f.min_x = f.min_y = f.min_z = std::numeric_limits<double>::max();
    f.max_x = f.max_y = f.max_z = std::numeric_limits<double>::lowest();
    for (int idx : cluster.indices) {
      if (idx < 0 || idx >= static_cast<int>(points.size())) continue;
      const auto & p = points[idx];
      f.centroid_x += p.x;
      f.centroid_y += p.y;
      f.centroid_z += p.z;
      f.min_x = std::min(f.min_x, static_cast<double>(p.x));
      f.max_x = std::max(f.max_x, static_cast<double>(p.x));
      f.min_y = std::min(f.min_y, static_cast<double>(p.y));
      f.max_y = std::max(f.max_y, static_cast<double>(p.y));
      f.min_z = std::min(f.min_z, static_cast<double>(p.z));
      f.max_z = std::max(f.max_z, static_cast<double>(p.z));
    }
    const double n = std::max(1, f.point_count);
    f.centroid_x /= n;
    f.centroid_y /= n;
    f.centroid_z /= n;
    f.length = std::max(0.05, f.max_x - f.min_x);
    f.width = std::max(0.05, f.max_y - f.min_y);
    if (f.width > f.length) std::swap(f.width, f.length);
    f.height = std::max(0.05, f.max_z - f.min_z);
    f.range = sensorRange(f.centroid_x, f.centroid_y);
    return f;
  }

  ObjectMemoryBankMatch matchObjectMemoryBank(
      const ObjectFeature & f,
      const std::vector<PointXYZIRT> & points) const {
    (void)points;
    ObjectMemoryBankMatch best;
    if (!object_memory_bank_enabled_ || memoryMatchingBlocked(f.range)) return best;
    for (size_t i = 0; i < object_memory_bank_.entries.size(); ++i) {
      const auto & entry = object_memory_bank_.entries[i];
      const auto & mem = entry.shape_memory;
      if (!mem.valid) continue;
      if (!isMemoryApplicableToComponent(entry, f.component_type)) {
        best.skipped_by_component_type = true;
        best.skipped_by_component_type_count++;
        best.skip_reason = "component_type_not_applicable";
        continue;
      }

      const double length_score = 1.0 - std::abs(f.length - mem.length) / std::max(0.5, mem.length);
      const double width_score = 1.0 - std::abs(f.width - mem.width) / std::max(0.3, mem.width);
      const double height_score = 1.0 - std::abs(f.height - mem.height) / std::max(0.3, mem.height);
      const double bbox_score = clamp(0.45 * length_score + 0.30 * width_score + 0.25 * height_score, 0.0, 1.0);
      const double node_score = clamp(static_cast<double>(mem.nodes.size()) / 40.0, 0.0, 1.0) *
                                clamp(mem.memory_confidence, 0.0, 1.0);
      const double edge_score = clamp(static_cast<double>(mem.edges.size()) / 80.0, 0.0, 1.0) *
                                clamp(mem.memory_confidence, 0.0, 1.0);
      const double relation_score = clamp(static_cast<double>(mem.endpoint_relations.size()) /
                                          std::max(1, shape_memory_max_match_relations_), 0.0, 1.0) *
                                    clamp(mem.memory_confidence, 0.0, 1.0);
      const double hp_score = clamp(1.0 - std::abs(f.max_z - entry.height_profile.max_height) /
                                    std::max(0.5, mem.height), 0.0, 1.0);
      double distance_score = 1.0;
      if (entry.distance_profile == "near") distance_score = f.range < near_range_max_ ? 1.0 : 0.6;
      else if (entry.distance_profile == "mid") distance_score = (f.range >= near_range_max_ && f.range < mid_range_max_) ? 1.0 : 0.7;
      else if (entry.distance_profile == "long") distance_score = f.range >= mid_range_max_ ? 1.0 : 0.7;

      const double final_score = 0.25 * bbox_score + 0.20 * node_score + 0.20 * edge_score +
                                 0.15 * relation_score + 0.10 * hp_score + 0.10 * distance_score;
      if (!best.matched || final_score > best.final_score) {
        best.matched = true;
        best.entry_index = static_cast<int>(i);
        best.memory_id = entry.memory_id;
        best.class_label = entry.class_label;
        best.class_prior = entry.class_prior;
        best.final_score = final_score;
        best.bbox_score = bbox_score;
        best.node_score = node_score;
        best.edge_score = edge_score;
        best.relation_score = relation_score;
        best.height_profile_score = hp_score;
        best.distance_profile_score = distance_score;
        best.skip_reason.clear();
      }
    }
    return best;
  }

  bool shouldTryObjectMemoryBankForTrack(const Track & tr) const {
    if (!object_memory_bank_enabled_ || object_memory_bank_.entries.empty()) return false;
    if (tr.shape_memory.valid && tr.shape_memory.committed) return false;
    if (!tr.feature.object_memory_id.empty()) return false;
    return true;
  }

  bool attachObjectMemoryBankIfMatched(
      Track & tr,
      ObjectFeature & tf,
      const std::vector<PointXYZIRT> & points,
      double stamp) {
    if (!object_memory_bank_enabled_ || object_memory_bank_.entries.empty() ||
        memoryMatchingBlockedPair(tr, tf)) return false;

    last_object_memory_match_attempt_count_++;
    const auto bank_match = matchObjectMemoryBank(tf, points);
    if (bank_match.skipped_by_component_type) {
      last_object_memory_skipped_by_component_type_count_ +=
          bank_match.skipped_by_component_type_count;
    }

    tf.object_memory_skipped_by_component_type = bank_match.skipped_by_component_type;
    tf.object_memory_skip_reason = bank_match.skip_reason;

    if (bank_match.matched) {
      last_object_memory_applicable_match_count_++;
      last_object_memory_best_match_id_ = bank_match.memory_id;
      last_object_memory_best_match_class_ =
          bank_match.class_label != "UNKNOWN" ? bank_match.class_label : bank_match.class_prior;
      last_object_memory_best_match_score_ =
          std::max(last_object_memory_best_match_score_, bank_match.final_score);
    }

    if (!bank_match.matched || bank_match.entry_index < 0 ||
        bank_match.final_score < object_memory_bank_attach_threshold_) {
      return false;
    }

    auto & entry = object_memory_bank_.entries[bank_match.entry_index];
    tr.shape_memory = entry.shape_memory;
    tr.shape_memory.committed = true;
    tr.shape_memory.frozen = true;
    tr.shape_memory.track_id = tr.id;
    tr.shape_memory.center_x = tr.x;
    tr.shape_memory.center_y = tr.y;
    tr.shape_memory.center_z = tr.z;

    tf.object_memory_id = bank_match.memory_id;
    tf.object_memory_class_label =
        bank_match.class_label != "UNKNOWN" ? bank_match.class_label : bank_match.class_prior;
    tf.object_memory_bank_match_score = bank_match.final_score;
    tf.object_memory_applicable_match = true;
    tf.object_memory_skipped_by_component_type = false;
    tf.object_memory_skip_reason.clear();
    tf.shape_memory_match_score = bank_match.final_score;
    tf.shape_memory_final_score = bank_match.final_score;
    tf.final_association_score = std::max(tf.final_association_score, bank_match.final_score);
    tf.decision_reason += "+object_memory_bank_prior";

    entry.matched_count++;
    entry.last_matched_time = stamp;
    last_object_memory_attach_count_++;
    return true;
  }

  FemOwnershipResult splitPointsByFemMemory(
      const ObjectMemoryEntry & memory,
      const ObjectFeature & candidate,
      const std::vector<PointXYZIRT> & points) const {
    FemOwnershipResult result;
    const auto & mem = memory.shape_memory;
    if (!mem.valid) {
      result.outside_indices = candidate.indices;
      return result;
    }

    const double yaw = std::isfinite(mem.yaw) ? mem.yaw : 0.0;
    const double c = std::cos(-yaw);
    const double s = std::sin(-yaw);
    const double node_support2 = fem_cluster_node_support_dist_ * fem_cluster_node_support_dist_;
    const double edge_support = fem_cluster_edge_support_dist_;
    std::unordered_set<int> supported_observed_nodes;
    std::unordered_set<int> supported_virtual_nodes;

    for (int idx : candidate.indices) {
      if (idx < 0 || idx >= static_cast<int>(points.size())) continue;
      const auto & p = points[idx];
      const double dx = p.x - candidate.centroid_x;
      const double dy = p.y - candidate.centroid_y;
      const double lx = dx * c - dy * s;
      const double ly = dx * s + dy * c;
      const double lz = p.z - candidate.centroid_z;

      const bool in_xy_body =
          lx >= mem.min_local_x - fem_cluster_body_margin_ &&
          lx <= mem.max_local_x + fem_cluster_body_margin_ &&
          ly >= mem.min_local_y - fem_cluster_body_margin_ &&
          ly <= mem.max_local_y + fem_cluster_body_margin_;
      const bool in_z_body =
          lz >= mem.min_local_z - fem_cluster_body_margin_ &&
          lz <= mem.max_local_z + fem_cluster_body_margin_;

      double best_node_d2 = std::numeric_limits<double>::max();
      const ShapeMemoryNode * best_node = nullptr;
      for (const auto & n : mem.nodes) {
        const double ndx = lx - n.local_x;
        const double ndy = ly - n.local_y;
        const double ndz = lz - n.local_z;
        const double d2 = ndx * ndx + ndy * ndy + 0.5 * ndz * ndz;
        if (d2 < best_node_d2) {
          best_node_d2 = d2;
          best_node = &n;
        }
      }
      const bool node_supported = best_node && best_node_d2 <= node_support2;
      if (node_supported) {
        if (best_node->is_virtual) supported_virtual_nodes.insert(best_node->id);
        else supported_observed_nodes.insert(best_node->id);
      }

      const bool cargo_candidate =
          cargo_detection_enabled_ && in_xy_body &&
          lz > mem.max_local_z + cargo_height_delta_threshold_;

      if (in_xy_body && (in_z_body || node_supported)) {
        if (cargo_candidate && fem_cluster_split_cargo_) {
          result.cargo_candidate_indices.push_back(idx);
        } else {
          result.owned_indices.push_back(idx);
        }
      } else if (cargo_candidate && fem_cluster_split_cargo_) {
        result.cargo_candidate_indices.push_back(idx);
      } else if (in_xy_body && fem_cluster_split_attached_unknown_) {
        result.attached_unknown_indices.push_back(idx);
      } else if (fem_cluster_absorb_orphan_points_ && in_xy_body) {
        result.owned_indices.push_back(idx);
      } else {
        result.outside_indices.push_back(idx);
      }
      (void)edge_support;
    }

    result.observed_supported_nodes = static_cast<int>(supported_observed_nodes.size());
    result.virtual_supported_nodes = static_cast<int>(supported_virtual_nodes.size());
    result.bbox_score = candidate.indices.empty() ? 0.0 :
        static_cast<double>(result.owned_indices.size() + result.cargo_candidate_indices.size()) /
        std::max<size_t>(1, candidate.indices.size());
    result.node_support_score =
        clamp((result.observed_supported_nodes + 0.45 * result.virtual_supported_nodes) /
              std::max<size_t>(1, mem.nodes.size()), 0.0, 1.0);
    const HeightProfile current_profile =
        computeHeightProfileFromIndices(points, candidate.indices, candidate.centroid_x, candidate.min_x, candidate.max_x);
    result.height_profile_score = 1.0 - computeCargoScore(memory.height_profile, current_profile);
    result.edge_support_score = result.node_support_score;
    result.ownership_score = clamp(0.45 * result.bbox_score + 0.35 * result.node_support_score +
                                   0.10 * result.edge_support_score + 0.10 * result.height_profile_score,
                                   0.0, 1.0);
    return result;
  }

  ClusterResult makeFemCluster(
      const std::vector<PointXYZIRT> & points,
      const std::vector<int> & indices,
      int id,
      const std::string & method,
      const std::string & role,
      const std::string & memory_id,
      double ownership_score) const {
    ClusterResult c;
    c.id = id;
    c.indices = indices;
    c.method = method;
    c.fem_cluster_role = role;
    c.fem_memory_id = memory_id;
    c.fem_ownership_score = ownership_score;
    std::unordered_set<int64_t> cells;
    for (int idx : indices) {
      if (idx < 0 || idx >= static_cast<int>(points.size())) continue;
      cells.insert(bevKey(points[idx]));
    }
    c.cell_keys.assign(cells.begin(), cells.end());
    fillClusterStats(points, c);
    return c;
  }

  std::vector<ClusterResult> clusterIndicesForFemRemainder(
      const std::vector<PointXYZIRT> & points,
      const std::vector<int> & indices,
      int & next_id,
      const std::string & role,
      const std::string & memory_id) const {
    std::unordered_map<int64_t, std::vector<int>> cells;
    for (int idx : indices) cells[bevKey(points[idx])].push_back(idx);
    std::unordered_set<int64_t> visited;
    std::vector<ClusterResult> out;
    for (const auto & kv : cells) {
      if (visited.count(kv.first)) continue;
      std::vector<int64_t> q{kv.first};
      visited.insert(kv.first);
      std::vector<int> ci;
      std::vector<int64_t> keys;
      for (size_t head = 0; head < q.size(); ++head) {
        const int64_t key = q[head];
        keys.push_back(key);
        const auto it = cells.find(key);
        if (it != cells.end()) ci.insert(ci.end(), it->second.begin(), it->second.end());
        const int ix = unpackX(key);
        const int iy = unpackY(key);
        for (int dx = -1; dx <= 1; ++dx) {
          for (int dy = -1; dy <= 1; ++dy) {
            if (cluster_connectivity_ == 4 && std::abs(dx) + std::abs(dy) > 1) continue;
            if (dx == 0 && dy == 0) continue;
            const int64_t nk = hash2d(ix + dx, iy + dy);
            if (cells.count(nk) && !visited.count(nk)) {
              visited.insert(nk);
              q.push_back(nk);
            }
          }
        }
      }
      if (ci.empty()) continue;
      ClusterResult c = makeFemCluster(points, ci, next_id++, "fem_assisted_recluster",
                                       role, memory_id, 0.0);
      c.cell_keys = keys;
      out.push_back(c);
    }
    return out;
  }

  std::vector<ClusterResult> refineClustersWithFemMemory(
      const std::vector<PointXYZIRT> & points,
      const std::vector<int> & foreground_indices,
      const std::vector<ClusterResult> & initial_clusters) {
    (void)foreground_indices;
    last_fem_cluster_refined_count_ = 0;
    last_object_memory_best_match_id_.clear();
    last_object_memory_best_match_class_.clear();
    last_object_memory_best_match_score_ = 0.0;
    last_fem_owned_point_count_ = 0;
    last_fem_outside_point_count_ = 0;
    last_fem_cargo_candidate_point_count_ = 0;
    last_fem_attached_unknown_point_count_ = 0;
    last_fem_split_unknown_count_ = 0;
    last_fem_split_cargo_count_ = 0;
    if (!fem_assisted_clustering_enabled_ || !object_memory_bank_enabled_ ||
        object_memory_bank_.entries.empty()) {
      return initial_clusters;
    }

    std::vector<ClusterResult> refined;
    int next_id = 0;
    for (const auto & cluster : initial_clusters) {
      ObjectFeature rough = makeFeatureFromClusterForMemoryMatch(points, cluster);
      const auto match = matchObjectMemoryBank(rough, points);
      if (!match.matched || match.entry_index < 0 ||
          match.final_score < fem_cluster_min_match_score_) {
        ClusterResult passthrough = cluster;
        passthrough.id = next_id++;
        refined.push_back(passthrough);
        continue;
      }

      last_object_memory_best_match_id_ = match.memory_id;
      last_object_memory_best_match_class_ =
          match.class_label != "UNKNOWN" ? match.class_label : match.class_prior;
      last_object_memory_best_match_score_ =
          std::max(last_object_memory_best_match_score_, match.final_score);

      const auto & entry = object_memory_bank_.entries[match.entry_index];
      const FemOwnershipResult own = splitPointsByFemMemory(entry, rough, points);
      if (own.owned_indices.empty() || own.ownership_score < fem_cluster_min_match_score_) {
        ClusterResult passthrough = cluster;
        passthrough.id = next_id++;
        refined.push_back(passthrough);
        continue;
      }

      last_fem_cluster_refined_count_++;
      last_fem_owned_point_count_ += static_cast<int>(own.owned_indices.size());
      last_fem_outside_point_count_ += static_cast<int>(own.outside_indices.size());
      last_fem_cargo_candidate_point_count_ += static_cast<int>(own.cargo_candidate_indices.size());
      last_fem_attached_unknown_point_count_ += static_cast<int>(own.attached_unknown_indices.size());

      refined.push_back(makeFemCluster(points, own.owned_indices, next_id++,
                                       "fem_assisted_memory", "TRUCK_BODY",
                                       entry.memory_id, own.ownership_score));
      if (fem_cluster_split_cargo_ && !own.cargo_candidate_indices.empty()) {
        last_fem_split_cargo_count_++;
        refined.push_back(makeFemCluster(points, own.cargo_candidate_indices, next_id++,
                                         "fem_assisted_memory", "TRUCK_CARGO_CANDIDATE",
                                         entry.memory_id, own.ownership_score));
      }
      if (fem_cluster_split_attached_unknown_ && !own.attached_unknown_indices.empty()) {
        last_fem_split_unknown_count_++;
        auto parts = clusterIndicesForFemRemainder(points, own.attached_unknown_indices, next_id,
                                                   "ATTACHED_UNKNOWN_OBJECT", entry.memory_id);
        refined.insert(refined.end(), parts.begin(), parts.end());
      }
      if (!own.outside_indices.empty()) {
        auto parts = clusterIndicesForFemRemainder(points, own.outside_indices, next_id,
                                                   "OUTSIDE_FEM_MEMORY", entry.memory_id);
        refined.insert(refined.end(), parts.begin(), parts.end());
      }
    }
    return refined;
  }

#include "general/vehicle_features.inc.hpp"

  std::string classifyComponentType(
      const ObjectFeature & f, double & confidence, std::string & reason) const {
    const double elongation = f.length / std::max(0.05, f.width);
    const bool vehicle_like =
        f.height >= vehicle_min_height_ && f.height <= vehicle_max_height_ &&
        f.width >= vehicle_min_width_ && f.width <= vehicle_max_width_ &&
        f.length <= vehicle_max_length_ && f.point_count >= min_cluster_points_long_;
    if (vehicle_like) {
      confidence = 0.82;
      reason = "vehicle_size_match";
      return "vehicle_like";
    }
    const bool wall_like =
        f.length >= wall_length_min_ && f.width <= wall_width_max_ &&
        elongation >= wall_elongation_ratio_min_;
    if (wall_like) {
      confidence = 0.78;
      reason = "wall_elongation_match";
      return "wall_like";
    }
    const bool crane_like =
        f.height >= crane_like_min_height_ &&
        (f.max_z - f.min_z) >= crane_like_min_vertical_extent_ &&
        f.speed <= crane_like_max_speed_;
    if (crane_like) {
      confidence = 0.62;
      reason = "vertical_extent_match";
      return "crane_like";
    }
    const bool human_like =
        f.height >= human_like_min_height_ && f.height <= human_like_max_height_ &&
        f.width <= human_like_max_width_ && f.length <= human_like_max_length_ &&
        elongation < 2.5;
    if (human_like) {
      confidence = 0.65;
      reason = "human_size_match";
      return "human_like";
    }
    if (isStaticStructure(f)) {
      confidence = 0.70;
      reason = "static_structure_match";
      return "static_structure_like";
    }
    confidence = 0.35;
    reason = "no_specific_component_match";
    return "unknown_like";
  }

  std::vector<std::string> defaultApplicableComponentTypesForClass(
      const std::string & class_label, const std::string & source_component_type) const {
    std::string label = class_label;
    std::transform(label.begin(), label.end(), label.begin(), ::toupper);
    if (label == "TRUCK" || label == "VEHICLE" || label == "CAR" || label == "TRUCK_LIKE" ||
        label == "VEHICLE_LIKE") {
      return {"vehicle_like"};
    }
    if (label == "HUMAN" || label == "PERSON") return {"human_like"};
    if (label == "WALL") return {"wall_like"};
    if (label == "CRANE") return {"crane_like"};
    if (label == "STATIC_STRUCTURE") return {"static_structure_like"};
    return {source_component_type.empty() ? std::string("unknown_like") : source_component_type};
  }

  bool isMemoryApplicableToComponent(
      const ObjectMemoryEntry & memory, const std::string & component_type) const {
    if (memory.applicable_component_types.empty()) return true;
    return std::find(memory.applicable_component_types.begin(),
                     memory.applicable_component_types.end(),
                     component_type) != memory.applicable_component_types.end();
  }

  ObjectFeature makeTrackFeature(const ObjectFeature & src) const {
    ObjectFeature dst = src;
    dst.indices.clear();
    return dst;
  }

  bool featureHasShapeQuality(const ObjectFeature & f) const {
    const int min_nodes = std::max(1, shape_memory_min_update_nodes_);
    return f.point_count >= shape_memory_min_update_points_ &&
           (f.shape_node_count >= min_nodes || f.cell_count >= min_nodes);
  }

  void rebuildBoxShapeMemoryGraph(ShapeMemory & mem) const {
    mem.nodes.clear();
    mem.edges.clear();
    const double xs[2] = {mem.min_local_x, mem.max_local_x};
    const double ys[2] = {mem.min_local_y, mem.max_local_y};
    const double zs[2] = {mem.min_local_z, mem.max_local_z};
    for (double x : xs) {
      for (double y : ys) {
        for (double z : zs) {
          ShapeMemoryNode node;
          node.local_x = x;
          node.local_y = y;
          node.local_z = z;
          node.confidence = mem.confidence;
          mem.nodes.push_back(node);
        }
      }
    }
    auto add_edge = [&](int a, int b) {
      ShapeMemoryEdge edge;
      edge.node_a = a;
      edge.node_b = b;
      edge.confidence = mem.confidence;
      mem.edges.push_back(edge);
    };
    add_edge(0, 1); add_edge(0, 2); add_edge(0, 4);
    add_edge(3, 1); add_edge(3, 2); add_edge(3, 7);
    add_edge(5, 1); add_edge(5, 4); add_edge(5, 7);
    add_edge(6, 2); add_edge(6, 4); add_edge(6, 7);
  }

  double getTrackYaw(const Track & tr) const {
    if (perception_mode_ != "crane_beam_work") return vehicle_fixed_yaw_rad_;
    if (persistentVehicle(tr)) return tr.observation_memory.vehicle_obb.yaw;
    if (tr.shape_memory.valid && tr.shape_memory.committed) {
      return tr.shape_memory.yaw;
    }

    if (perception_mode_ != "crane_beam_work" && kalmanEnabled()) {
      const auto & mem = tr.observation_memory;
      if (mem.reliable && mem.vehicle_obb.valid) return mem.vehicle_obb.yaw;
      if (tr.feature.obb.valid) return tr.feature.obb.yaw;
      return 0.0; // KF velocity is never converted into box yaw.
    }

    if (std::isfinite(tr.yaw)) {
      return tr.yaw;
    }

    const double speed = std::hypot(tr.vx, tr.vy);
    if (speed > motion_speed_threshold_) {
      return std::atan2(tr.vy, tr.vx);
    }

    return 0.0;
  }

  double getTrackMotionYaw(const Track & tr) const {
    const double speed = std::hypot(tr.vx, tr.vy);
    if (speed > motion_speed_threshold_) {
      return std::atan2(tr.vy, tr.vx);
    }
    if (tr.shape_memory.valid && std::isfinite(tr.shape_memory.learned_motion_yaw)) {
      return tr.shape_memory.learned_motion_yaw;
    }
    if (std::isfinite(tr.yaw)) {
      return tr.yaw;
    }
    return getTrackYaw(tr);
  }

  geometry_msgs::msg::Point transformMemoryNodeToWorld(const Track & tr, const ShapeMemoryNode & n) const {
    const double yaw = getTrackYaw(tr);
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    geometry_msgs::msg::Point p;
    p.x = tr.x + n.local_x * c - n.local_y * s;
    p.y = tr.y + n.local_x * s + n.local_y * c;
    p.z = tr.z + n.local_z;
    return p;
  }

  void selectEndpointsAndRelations(ShapeMemory & mem) const {
    mem.endpoint_relations.clear();
    if (mem.nodes.empty()) return;

    // Compute degree of each node in the graph
    std::unordered_map<int, int> degrees;
    for (const auto & n : mem.nodes) {
      degrees[n.id] = 0;
    }
    for (const auto & e : mem.edges) {
      degrees[e.node_a]++;
      degrees[e.node_b]++;
    }

    // Determine endpoints
    double min_lx = std::numeric_limits<double>::max();
    double max_lx = std::numeric_limits<double>::lowest();
    double min_ly = std::numeric_limits<double>::max();
    double max_ly = std::numeric_limits<double>::lowest();
    for (const auto & n : mem.nodes) {
      min_lx = std::min(min_lx, n.local_x);
      max_lx = std::max(max_lx, n.local_x);
      min_ly = std::min(min_ly, n.local_y);
      max_ly = std::max(max_ly, n.local_y);
    }

    std::unordered_set<int> endpoint_ids;
    for (auto & n : mem.nodes) {
      bool is_ep = false;
      // 1. degree <= 1
      if (degrees[n.id] <= 1) {
        is_ep = true;
      }
      // 2. near bbox boundary (bbox corner candidates)
      if (std::abs(n.local_x - min_lx) < 0.3 || std::abs(n.local_x - max_lx) < 0.3 ||
          std::abs(n.local_y - min_ly) < 0.3 || std::abs(n.local_y - max_ly) < 0.3) {
        if (degrees[n.id] > 0) {
          is_ep = true;
          n.is_boundary = true;
        }
      }
      n.is_endpoint = is_ep;
      if (is_ep) {
        endpoint_ids.insert(n.id);
      }
    }

    // 3. Include endpoints of long structural edges or high-score edges
    for (const auto & e : mem.edges) {
      if ((e.is_structural_edge && e.length > 2.0) || e.top_score > 0.7 || e.side_score > 0.7 || e.front_score > 0.7) {
        for (auto & n : mem.nodes) {
          if (n.id == e.node_a || n.id == e.node_b) {
            n.is_endpoint = true;
            endpoint_ids.insert(n.id);
          }
        }
      }
    }

    auto find_node = [&](int id) -> const ShapeMemoryNode * {
      for (const auto & n : mem.nodes) {
        if (n.id == id) return &n;
      }
      return nullptr;
    };

    struct CandidateRelation {
      int a;
      int b;
      double dist;
      double yaw;
      double height_delta;
      double score;
      std::string type;
      bool uses_virtual_node = false;
      bool uses_mirrored_node = false;
    };
    std::vector<CandidateRelation> relation_candidates;

    auto add_relation = [&](int a_id, int b_id, const std::string & type, double bonus) {
      const auto * na = find_node(a_id);
      const auto * nb = find_node(b_id);
      if (!na || !nb || a_id == b_id) return;
      const double dx = nb->local_x - na->local_x;
      const double dy = nb->local_y - na->local_y;
      const double dz = nb->local_z - na->local_z;
      const double d = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (d < 1e-3) return;
      CandidateRelation cand;
      cand.a = a_id;
      cand.b = b_id;
      cand.dist = d;
      cand.yaw = std::atan2(dy, dx);
      cand.height_delta = dz;
      cand.type = type;
      cand.uses_virtual_node = na->is_virtual || nb->is_virtual;
      cand.uses_mirrored_node = na->is_mirrored || nb->is_mirrored;
      const double span_score =
          clamp(d / std::max(0.1, std::max({mem.length, mem.width, mem.height})), 0.0, 1.0);
      const double soft_penalty = cand.uses_virtual_node ? 0.35 : 0.0;
      cand.score = clamp(bonus + 0.4 * span_score +
                         0.1 * (na->confidence + nb->confidence) - soft_penalty, 0.0, 1.5);
      relation_candidates.push_back(cand);
    };

    auto extremal_node = [&](char axis, bool high, bool observed_only) -> int {
      int best_id = -1;
      double best_val = high ? std::numeric_limits<double>::lowest()
                             : std::numeric_limits<double>::max();
      for (const auto & n : mem.nodes) {
        if (!endpoint_ids.count(n.id)) continue;
        if (observed_only && n.is_virtual) continue;
        const double v = axis == 'x' ? n.local_x : (axis == 'y' ? n.local_y : n.local_z);
        if ((high && v > best_val) || (!high && v < best_val)) {
          best_val = v;
          best_id = n.id;
        }
      }
      return best_id;
    };

    auto best_extremal = [&](char axis, bool high) {
      const int observed = extremal_node(axis, high, true);
      return observed >= 0 ? observed : extremal_node(axis, high, false);
    };
    const int x_min_id = best_extremal('x', false);
    const int x_max_id = best_extremal('x', true);
    const int y_min_id = best_extremal('y', false);
    const int y_max_id = best_extremal('y', true);
    const int z_min_id = best_extremal('z', false);
    const int z_max_id = best_extremal('z', true);
    add_relation(x_min_id, x_max_id, "length", 1.0);
    add_relation(y_min_id, y_max_id, "width", 0.9);
    add_relation(z_min_id, z_max_id, "height", 0.85);

    // Build endpoint relation candidates selectively, then cap globally.
    for (const auto & e : mem.edges) {
      bool ep_a = endpoint_ids.count(e.node_a) > 0;
      bool ep_b = endpoint_ids.count(e.node_b) > 0;
      if (ep_a && ep_b) {
        CandidateRelation cand;
        cand.a = e.node_a;
        cand.b = e.node_b;
        cand.dist = e.length;
        cand.yaw = e.direction_yaw;
        cand.height_delta = e.local_dz;
        cand.type = "structural";
        const auto * na = find_node(e.node_a);
        const auto * nb = find_node(e.node_b);
        cand.uses_virtual_node = (na && na->is_virtual) || (nb && nb->is_virtual) || e.is_virtual;
        cand.uses_mirrored_node = (na && na->is_mirrored) || (nb && nb->is_mirrored) || e.is_mirrored;
        cand.score = e.confidence + (e.is_structural_edge ? 0.25 : 0.0) +
                     0.1 * clamp(e.length / std::max(0.1, mem.length), 0.0, 1.0) -
                     (cand.uses_virtual_node ? 0.35 : 0.0);
        relation_candidates.push_back(cand);
      }
    }

    // Also include bbox corner relations (e.g. diagonal relations for bounding box models)
    if (relation_candidates.empty() || mem.edges.size() < 12) {
      std::vector<int> eps(endpoint_ids.begin(), endpoint_ids.end());
      for (size_t i = 0; i < eps.size(); ++i) {
        for (size_t j = i + 1; j < eps.size(); ++j) {
          const ShapeMemoryNode* na = nullptr;
          const ShapeMemoryNode* nb = nullptr;
          for (const auto & n : mem.nodes) {
            if (n.id == eps[i]) na = &n;
            if (n.id == eps[j]) nb = &n;
          }
          if (na && nb) {
            double dx = nb->local_x - na->local_x;
            double dy = nb->local_y - na->local_y;
            double dz = nb->local_z - na->local_z;
            double d = std::sqrt(dx*dx + dy*dy + dz*dz);
            if (d > 1.5 && d < 15.0) {
              double rel_score = d * 0.1;
              if (na->is_boundary && nb->is_boundary) rel_score += 0.5;
              if (na->is_corner_candidate || nb->is_corner_candidate) rel_score += 0.3;

              CandidateRelation cand;
              cand.a = eps[i];
              cand.b = eps[j];
              cand.dist = d;
              cand.yaw = std::atan2(dy, dx);
              cand.height_delta = dz;
              cand.type = "structural";
              cand.uses_virtual_node = na->is_virtual || nb->is_virtual;
              cand.uses_mirrored_node = na->is_mirrored || nb->is_mirrored;
              cand.score = rel_score - (cand.uses_virtual_node ? 0.35 : 0.0);
              relation_candidates.push_back(cand);
            }
          }
        }
      }
    }

    std::sort(relation_candidates.begin(), relation_candidates.end(),
              [](const CandidateRelation & r1, const CandidateRelation & r2) {
                if (r1.uses_virtual_node != r2.uses_virtual_node) return !r1.uses_virtual_node;
                return r1.score > r2.score;
              });

    std::unordered_set<int64_t> used_pairs;
    for (const auto & c : relation_candidates) {
      if (static_cast<int>(mem.endpoint_relations.size()) >=
          std::max(0, shape_memory_max_endpoint_relations_)) {
        break;
      }
      const int a = std::min(c.a, c.b);
      const int b = std::max(c.a, c.b);
      const int64_t pair_key = hash2d(a, b);
      if (!used_pairs.insert(pair_key).second) continue;

      EndpointRelation rel;
      rel.endpoint_a = c.a;
      rel.endpoint_b = c.b;
      rel.expected_distance = c.dist;
      rel.expected_direction_yaw = c.yaw;
      rel.expected_height_delta = c.height_delta;
      rel.confidence = clamp(c.score, 0.0, 1.0);
      rel.relation_type = c.type;
      rel.relation_score = c.score;
      rel.uses_virtual_node = c.uses_virtual_node;
      rel.uses_mirrored_node = c.uses_mirrored_node;
      rel.observed_count = 1;
      rel.missing_count = 0;
      mem.endpoint_relations.push_back(rel);
    }
  }

  std::vector<ShapeNode> buildCacheNodes(const std::vector<PointXYZIRT> & points) const {
    const double vs = std::max(0.05, shape_graph_voxel_size_);
    std::unordered_map<int64_t, ShapeNode> keyed_nodes;
    for (size_t i = 0; i < points.size(); ++i) {
      const auto & p = points[i];
      const int vx = static_cast<int>(std::floor(p.x / vs));
      const int vy = static_cast<int>(std::floor(p.y / vs));
      const int vz = static_cast<int>(std::floor(p.z / vs));
      auto & n = keyed_nodes[hash3d(vx, vy, vz)];
      if (n.point_indices.empty()) {
        n.min_x = n.max_x = p.x;
        n.min_y = n.max_y = p.y;
        n.min_z = n.max_z = p.z;
      }
      n.point_indices.push_back(static_cast<int>(i));
      n.x += p.x;
      n.y += p.y;
      n.z += p.z;
      n.min_x = std::min(n.min_x, static_cast<double>(p.x));
      n.max_x = std::max(n.max_x, static_cast<double>(p.x));
      n.min_y = std::min(n.min_y, static_cast<double>(p.y));
      n.max_y = std::max(n.max_y, static_cast<double>(p.y));
      n.min_z = std::min(n.min_z, static_cast<double>(p.z));
      n.max_z = std::max(n.max_z, static_cast<double>(p.z));
    }
    std::vector<ShapeNode> nodes;
    nodes.reserve(keyed_nodes.size());
    int id = 0;
    for (auto & kv : keyed_nodes) {
      auto n = kv.second;
      n.id = id++;
      n.point_count = static_cast<int>(n.point_indices.size());
      const double denom = std::max(1, n.point_count);
      n.x /= denom;
      n.y /= denom;
      n.z /= denom;
      nodes.push_back(n);
    }
    return nodes;
  }

  std::vector<ShapeEdge> buildCacheEdges(const std::vector<ShapeNode> & nodes) const {
    std::vector<ShapeEdge> edges;
    for (size_t i = 0; i < nodes.size(); ++i) {
      for (size_t j = i + 1; j < nodes.size(); ++j) {
        ShapeEdge e;
        if (makeCacheShapeEdgeIfValid(nodes[i], nodes[j], e)) edges.push_back(e);
      }
    }
    return edges;
  }

  bool makeCacheShapeEdgeIfValid(const ShapeNode & a, const ShapeNode & b, ShapeEdge & e) const {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    const double d3 = std::sqrt(dx * dx + dy * dy + dz * dz);
    const double range = 0.5 * (std::hypot(a.x, a.y) + std::hypot(b.x, b.y));
    const double max_d = maxCacheEdgeDistance(range);
    if (d3 > max_d) return false;

    e.a = a.id;
    e.b = b.id;
    e.top_score = shape_graph_use_top_view_ ? cacheDistanceScore(std::hypot(a.x - b.x, a.y - b.y), max_d) : 0.0;
    e.side_score = shape_graph_use_side_view_ ? cacheDistanceScore(std::hypot(a.x - b.x, a.z - b.z), max_d) : 0.0;
    e.front_score = shape_graph_use_front_view_ ? cacheDistanceScore(std::hypot(a.y - b.y, a.z - b.z), max_d) : 0.0;
    const double dist_penalty = clamp(1.0 - d3 / std::max(1e-3, max_d), 0.0, 1.0);
    const double weighted =
        shape_graph_top_weight_ * e.top_score + shape_graph_side_weight_ * e.side_score + shape_graph_front_weight_ * e.front_score;
    e.total_score = 0.75 * weighted + 0.25 * dist_penalty;
    if (e.total_score < shape_graph_min_edge_score_) return false;
    e.reason = "MULTI_VIEW_DISTANCE";
    return true;
  }

  double maxCacheEdgeDistance(double range) const {
    if (range < near_range_max_) return shape_graph_max_edge_dist_near_;
    if (range < mid_range_max_) return shape_graph_max_edge_dist_mid_;
    return shape_graph_max_edge_dist_long_;
  }

  double cacheDistanceScore(double d, double max_d) const {
    return clamp(1.0 - d / std::max(1e-3, max_d), 0.0, 1.0);
  }

  void clearObservedShapeCache(ObservedShapeGraphCache & cache) const {
    cache.valid = false;
    cache.track_id = -1;
    cache.start_frame = 0;
    cache.last_update_frame = 0;
    cache.captured_frames = 0;
    cache.target_capture_frames = 0;
    cache.accumulated_local_points.clear();
    cache.nodes.clear();
    cache.edges.clear();
    cache.confidence = 0.0;
    cache.state = "empty";
  }

  double selectShapeMemoryCompareVoxelSize(double range) const {
    if (range < near_range_max_) return std::max(0.05, shape_memory_compare_voxel_size_near_);
    if (range < mid_range_max_) return std::max(0.05, shape_memory_compare_voxel_size_mid_);
    return std::max(0.05, shape_memory_compare_voxel_size_long_);
  }

  std::vector<PointXYZIRT> downsamplePointsForShapeMemoryCompare(
      const ObjectFeature & f, const std::vector<PointXYZIRT> & points) const {
    const double vs = selectShapeMemoryCompareVoxelSize(f.range);
    std::unordered_map<int64_t, PointXYZIRT> voxels;
    std::unordered_map<int64_t, int> counts;
    for (int idx : f.indices) {
      if (idx < 0 || idx >= static_cast<int>(points.size())) continue;
      const auto & p = points[idx];
      const int vx = static_cast<int>(std::floor(p.x / vs));
      const int vy = static_cast<int>(std::floor(p.y / vs));
      const int vz = static_cast<int>(std::floor(p.z / vs));
      const int64_t key = hash3d(vx, vy, vz);
      auto & acc = voxels[key];
      acc.x += p.x;
      acc.y += p.y;
      acc.z += p.z;
      acc.intensity += p.intensity;
      acc.ring = p.ring;
      counts[key]++;
    }

    std::vector<PointXYZIRT> support;
    support.reserve(voxels.size());
    for (auto & kv : voxels) {
      const int c = std::max(1, counts[kv.first]);
      kv.second.x /= c;
      kv.second.y /= c;
      kv.second.z /= c;
      kv.second.intensity /= c;
      support.push_back(kv.second);
    }
    if (shape_memory_compare_max_points_ > 0 &&
        static_cast<int>(support.size()) > shape_memory_compare_max_points_) {
      std::partial_sort(support.begin(), support.begin() + shape_memory_compare_max_points_,
                        support.end(), [](const PointXYZIRT & a, const PointXYZIRT & b) {
                          return std::hypot(a.x, a.y) < std::hypot(b.x, b.y);
                        });
      support.resize(shape_memory_compare_max_points_);
    }
    return support;
  }

  std::vector<const ShapeMemoryNode*> selectMatchNodes(const ShapeMemory & mem) const {
    std::vector<const ShapeMemoryNode*> nodes;
    nodes.reserve(mem.nodes.size());
    for (const auto & n : mem.nodes) nodes.push_back(&n);
    std::sort(nodes.begin(), nodes.end(), [](const ShapeMemoryNode * a, const ShapeMemoryNode * b) {
      if (a->is_endpoint != b->is_endpoint) return a->is_endpoint;
      if (a->confidence != b->confidence) return a->confidence > b->confidence;
      return a->point_count > b->point_count;
    });
    if (shape_memory_max_match_nodes_ > 0 &&
        static_cast<int>(nodes.size()) > shape_memory_max_match_nodes_) {
      nodes.resize(shape_memory_max_match_nodes_);
    }
    return nodes;
  }

  std::vector<const ShapeMemoryEdge*> selectMatchEdges(const ShapeMemory & mem) const {
    std::vector<const ShapeMemoryEdge*> edges;
    edges.reserve(mem.edges.size());
    for (const auto & e : mem.edges) edges.push_back(&e);
    std::sort(edges.begin(), edges.end(), [](const ShapeMemoryEdge * a, const ShapeMemoryEdge * b) {
      if (a->is_structural_edge != b->is_structural_edge) return a->is_structural_edge;
      return a->confidence > b->confidence;
    });
    if (shape_memory_max_match_edges_ > 0 &&
        static_cast<int>(edges.size()) > shape_memory_max_match_edges_) {
      edges.resize(shape_memory_max_match_edges_);
    }
    return edges;
  }

  std::vector<const EndpointRelation*> selectMatchRelations(const ShapeMemory & mem) const {
    std::vector<const EndpointRelation*> rels;
    rels.reserve(mem.endpoint_relations.size());
    for (const auto & r : mem.endpoint_relations) rels.push_back(&r);
    std::sort(rels.begin(), rels.end(), [](const EndpointRelation * a, const EndpointRelation * b) {
      return a->confidence > b->confidence;
    });
    const int cap = std::min(shape_memory_max_match_relations_, shape_memory_max_endpoint_relations_);
    if (cap > 0 && static_cast<int>(rels.size()) > cap) rels.resize(cap);
    return rels;
  }

  const ShapeMemoryNode * findMemoryNode(const ShapeMemory & mem, int id) const {
    for (const auto & n : mem.nodes) {
      if (n.id == id) return &n;
    }
    return nullptr;
  }

  const PointXYZIRT * nearestSupportPoint(
      const std::vector<PointXYZIRT> & support,
      const geometry_msgs::msg::Point & target,
      double max_distance,
      double & best_distance) const {
    const PointXYZIRT * best = nullptr;
    best_distance = std::numeric_limits<double>::max();
    for (const auto & p : support) {
      const double dx = p.x - target.x;
      const double dy = p.y - target.y;
      const double dz = p.z - target.z;
      const double d = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (d < best_distance && d <= max_distance) {
        best_distance = d;
        best = &p;
      }
    }
    return best;
  }

  double computeShapeMemoryConfidence(
      const ShapeMemory & mem,
      const ObservedShapeGraphCache & cache,
      const ObjectFeature & f) const {
    double edge_conf_sum = 0.0;
    for (const auto & e : mem.edges) edge_conf_sum += e.confidence;
    double rel_conf_sum = 0.0;
    for (const auto & r : mem.endpoint_relations) rel_conf_sum += r.confidence;
    const double node_quality = clamp(static_cast<double>(mem.nodes.size()) /
                                      std::max(1, shape_memory_commit_min_nodes_ * 4), 0.0, 1.0);
    const double edge_quality = mem.edges.empty() ? 0.0 :
        clamp(0.5 * static_cast<double>(mem.edges.size()) / std::max(1, shape_memory_commit_min_edges_ * 8) +
              0.5 * edge_conf_sum / std::max<size_t>(1, mem.edges.size()), 0.0, 1.0);
    const double relation_quality = mem.endpoint_relations.empty() ? 0.0 :
        clamp(0.5 * static_cast<double>(mem.endpoint_relations.size()) / 2.0 +
              0.5 * rel_conf_sum / std::max<size_t>(1, mem.endpoint_relations.size()), 0.0, 1.0);
    const bool vehicle_size =
        f.width >= vehicle_min_width_ && f.width <= vehicle_max_width_ &&
        f.height >= vehicle_min_height_ && f.height <= vehicle_max_height_ &&
        f.length >= 2.0 && f.length <= vehicle_max_length_;
    const double bbox_quality = vehicle_size ? 1.0 : 0.45;
    const double observation_quality = clamp(
        0.45 * static_cast<double>(f.point_count) / std::max(1, shape_memory_min_update_points_ * 5) +
        0.35 * static_cast<double>(cache.captured_frames) / std::max(1, cache.target_capture_frames) +
        0.20 * cache.confidence, 0.0, 1.0);
    return clamp(0.25 * node_quality + 0.25 * edge_quality + 0.20 * relation_quality +
                 0.15 * bbox_quality + 0.15 * observation_quality, 0.0, 1.0);
  }

  void computeShapeMemoryClassPrior(ShapeMemory & mem, const ObjectFeature & f) const {
    const double elongation = f.length / std::max(0.05, f.width);
    double vehicle = 0.0;
    vehicle += (f.width >= vehicle_min_width_ && f.width <= vehicle_max_width_) ? 0.22 : 0.0;
    vehicle += (f.height >= vehicle_min_height_ && f.height <= vehicle_max_height_) ? 0.22 : 0.0;
    vehicle += (f.length >= 2.0 && f.length <= vehicle_max_length_) ? 0.20 : 0.0;
    vehicle += elongation >= 1.5 ? 0.18 : 0.0;
    vehicle += mem.endpoint_relations.size() >= 2 ? 0.18 : 0.0;

    double truck = 0.0;
    truck += f.length >= 5.0 ? 0.30 : 0.0;
    truck += f.height >= 1.8 ? 0.22 : 0.0;
    truck += f.width >= 1.8 ? 0.18 : 0.0;
    truck += mem.top_area >= 10.0 ? 0.18 : 0.0;
    truck += mem.endpoint_relations.size() >= 4 ? 0.12 : 0.0;

    mem.vehicle_like_score = clamp(vehicle, 0.0, 1.0);
    mem.truck_like_score = clamp(truck, 0.0, 1.0);
    if (mem.truck_like_score >= 0.65) {
      mem.class_prior = "TRUCK_LIKE";
      mem.class_confidence = mem.truck_like_score;
    } else if (mem.vehicle_like_score >= 0.55) {
      mem.class_prior = "VEHICLE_LIKE";
      mem.class_confidence = mem.vehicle_like_score;
    } else {
      mem.class_prior = "UNKNOWN";
      mem.class_confidence = std::max(mem.vehicle_like_score, mem.truck_like_score);
    }
  }

  void buildShapeMemoryFromCache(ShapeMemory & mem, const ObservedShapeGraphCache & cache) const {
    mem.nodes.clear();
    mem.edges.clear();
    mem.endpoint_relations.clear();

    for (const auto & cn : cache.nodes) {
      ShapeMemoryNode mn;
      mn.id = cn.id;
      mn.local_x = cn.x;
      mn.local_y = cn.y;
      mn.local_z = cn.z;
      mn.point_count = cn.point_count;
      mn.confidence = mem.confidence;
      mn.observed_count = 1;
      mn.missing_count = 0;
      mn.is_boundary = false;
      mn.is_corner_candidate = false;
      mn.is_endpoint = false;
      mem.nodes.push_back(mn);
    }

    for (const auto & ce : cache.edges) {
      ShapeMemoryEdge me;
      me.node_a = ce.a;
      me.node_b = ce.b;

      const ShapeNode* na = nullptr;
      const ShapeNode* nb = nullptr;
      for (const auto & n : cache.nodes) {
        if (n.id == ce.a) na = &n;
        if (n.id == ce.b) nb = &n;
      }
      if (na && nb) {
        me.local_dx = nb->x - na->x;
        me.local_dy = nb->y - na->y;
        me.local_dz = nb->z - na->z;
        me.length = std::sqrt(me.local_dx*me.local_dx + me.local_dy*me.local_dy + me.local_dz*me.local_dz);
        me.direction_yaw = std::atan2(me.local_dy, me.local_dx);
        me.height_delta = std::abs(me.local_dz);
      } else {
        me.local_dx = me.local_dy = me.local_dz = 0.0;
        me.length = 0.0;
        me.direction_yaw = 0.0;
        me.height_delta = 0.0;
      }
      me.confidence = ce.total_score;
      me.observed_count = 1;
      me.missing_count = 0;
      me.top_score = ce.top_score;
      me.side_score = ce.side_score;
      me.front_score = ce.front_score;
      me.is_structural_edge = (ce.total_score > 0.7);
      mem.edges.push_back(me);
    }

    selectEndpointsAndRelations(mem);
  }

  ShapeMemoryMatchBreakdown computeCommittedMemoryToCurrentObservationBreakdown(
      const Track & tr, const ObjectFeature & f, const std::vector<PointXYZIRT> & points,
      double mode_multiplier) const {
    ShapeMemoryMatchBreakdown out;
    const ShapeMemory & mem = tr.shape_memory;
    double l_ratio = std::min(f.length, mem.length) / std::max(0.05, std::max(f.length, mem.length));
    double w_ratio = std::min(f.width, mem.width) / std::max(0.05, std::max(f.width, mem.width));
    double h_ratio = std::min(f.height, mem.height) / std::max(0.05, std::max(f.height, mem.height));
    out.bbox_score = l_ratio * w_ratio * h_ratio;

    const auto support = downsamplePointsForShapeMemoryCompare(f, points);
    const auto match_nodes = selectMatchNodes(mem);
    const auto match_edges = selectMatchEdges(mem);
    const auto match_relations = selectMatchRelations(mem);
    const double node_gate = std::max(0.4, selectShapeMemoryCompareVoxelSize(f.range) * 1.5);

    int matched_nodes = 0;
    for (const auto * mn : match_nodes) {
      geometry_msgs::msg::Point world_p = transformMemoryNodeToWorld(tr, *mn);
      double best_dist = 0.0;
      if (nearestSupportPoint(support, world_p, node_gate, best_dist)) matched_nodes++;
    }
    out.node_score = match_nodes.empty() ? 1.0 : static_cast<double>(matched_nodes) / match_nodes.size();

    int matched_relations = 0;
    int active_relations = 0;
    for (const auto * rel : match_relations) {
      const ShapeMemoryNode* mn_a = findMemoryNode(mem, rel->endpoint_a);
      const ShapeMemoryNode* mn_b = findMemoryNode(mem, rel->endpoint_b);
      if (!mn_a || !mn_b) continue;

      active_relations++;

      geometry_msgs::msg::Point world_a = transformMemoryNodeToWorld(tr, *mn_a);
      geometry_msgs::msg::Point world_b = transformMemoryNodeToWorld(tr, *mn_b);

      double dist_a_min = std::numeric_limits<double>::max();
      double dist_b_min = std::numeric_limits<double>::max();
      const PointXYZIRT* pt_a = nearestSupportPoint(support, world_a, 1.0, dist_a_min);
      const PointXYZIRT* pt_b = nearestSupportPoint(support, world_b, 1.0, dist_b_min);

      if (pt_a && pt_b) {
        double cur_dx = pt_b->x - pt_a->x;
        double cur_dy = pt_b->y - pt_a->y;
        double cur_dz = pt_b->z - pt_a->z;
        double cur_dist = std::sqrt(cur_dx*cur_dx + cur_dy*cur_dy + cur_dz*cur_dz);
        double cur_yaw = std::atan2(cur_dy, cur_dx);
        double cur_h = std::abs(cur_dz);

        double dist_diff = std::abs(cur_dist - rel->expected_distance);
        double yaw_diff = std::abs(cur_yaw - rel->expected_direction_yaw);
        if (yaw_diff > M_PI) yaw_diff = 2.0 * M_PI - yaw_diff;
        double h_diff = std::abs(cur_h - rel->expected_height_delta);

        if (dist_diff < 0.8 && yaw_diff < 0.5 && h_diff < 0.5) {
          matched_relations++;
        }
      }
    }
    out.endpoint_relation_score = active_relations == 0 ? 1.0 : static_cast<double>(matched_relations) / active_relations;

    int matched_edges = 0;
    int active_edges = 0;
    for (const auto * edge : match_edges) {
      const ShapeMemoryNode* mn_a = findMemoryNode(mem, edge->node_a);
      const ShapeMemoryNode* mn_b = findMemoryNode(mem, edge->node_b);
      if (!mn_a || !mn_b) continue;

      active_edges++;

      geometry_msgs::msg::Point world_a = transformMemoryNodeToWorld(tr, *mn_a);
      geometry_msgs::msg::Point world_b = transformMemoryNodeToWorld(tr, *mn_b);

      double dist_a_min = std::numeric_limits<double>::max();
      double dist_b_min = std::numeric_limits<double>::max();
      const PointXYZIRT* pt_a = nearestSupportPoint(support, world_a, 1.0, dist_a_min);
      const PointXYZIRT* pt_b = nearestSupportPoint(support, world_b, 1.0, dist_b_min);

      if (pt_a && pt_b) {
        double cur_dx = pt_b->x - pt_a->x;
        double cur_dy = pt_b->y - pt_a->y;
        double cur_dz = pt_b->z - pt_a->z;
        double cur_dist = std::sqrt(cur_dx*cur_dx + cur_dy*cur_dy + cur_dz*cur_dz);
        double cur_yaw = std::atan2(cur_dy, cur_dx);
        double cur_h = std::abs(cur_dz);

        double dist_diff = std::abs(cur_dist - edge->length);
        double yaw_diff = std::abs(cur_yaw - edge->direction_yaw);
        if (yaw_diff > M_PI) yaw_diff = 2.0 * M_PI - yaw_diff;
        double h_diff = std::abs(cur_h - edge->height_delta);

        if (dist_diff < 0.8 && yaw_diff < 0.5 && h_diff < 0.5) {
          matched_edges++;
        }
      }
    }
    out.edge_score = active_edges == 0 ? 1.0 : static_cast<double>(matched_edges) / active_edges;

    double h_diff = std::abs(f.height - mem.height);
    out.height_score = clamp(1.0 - h_diff / std::max(0.5, mem.height), 0.0, 1.0);

    double raw_score = 0.20 * out.bbox_score +
                       0.20 * out.node_score +
                       0.25 * out.endpoint_relation_score +
                       0.25 * out.edge_score +
                       0.10 * out.height_score;

    out.final_score = clamp(raw_score, 0.0, 1.0) * mem.confidence * mode_multiplier;
    return out;
  }

  double computeCommittedMemoryToCurrentObservationScore(
      const Track & tr, const ObjectFeature & f, const std::vector<PointXYZIRT> & points,
      double mode_multiplier, ShapeMemoryMatchBreakdown * breakdown = nullptr) const {
    ShapeMemoryMatchBreakdown out =
        computeCommittedMemoryToCurrentObservationBreakdown(tr, f, points, mode_multiplier);
    if (breakdown) *breakdown = out;
    return out.final_score;
  }

  // Used when shape_graph_compute_policy == "always":
  // Compare committed memory nodes/edges against the current frame's observed shape graph
  // (ObjectFeature::shape_nodes / shape_edges).
  double computeShapeGraphToShapeGraphScore(
      const Track & tr, const ObjectFeature & f, double mode_multiplier) const {
    const ShapeMemory & mem = tr.shape_memory;

    double l_ratio = std::min(f.length, mem.length) / std::max(0.05, std::max(f.length, mem.length));
    double w_ratio = std::min(f.width, mem.width) / std::max(0.05, std::max(f.width, mem.width));
    double h_ratio = std::min(f.height, mem.height) / std::max(0.05, std::max(f.height, mem.height));
    double bbox_overlap_score = l_ratio * w_ratio * h_ratio;

    int matched_nodes = 0;
    for (const auto & mn : mem.nodes) {
      geometry_msgs::msg::Point world_p = transformMemoryNodeToWorld(tr, mn);
      for (const auto & on : f.shape_nodes) {
        double dx = on.x - world_p.x;
        double dy = on.y - world_p.y;
        double dz = on.z - world_p.z;
        double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (dist < std::max(0.4, shape_graph_voxel_size_ * 2.0)) {
          matched_nodes++;
          break;
        }
      }
    }
    double visible_node_score = mem.nodes.empty() ? 1.0 : static_cast<double>(matched_nodes) / mem.nodes.size();

    int matched_relations = 0;
    int active_relations = 0;
    for (const auto & rel : mem.endpoint_relations) {
      const ShapeMemoryNode* mn_a = nullptr;
      const ShapeMemoryNode* mn_b = nullptr;
      for (const auto & mn : mem.nodes) {
        if (mn.id == rel.endpoint_a) mn_a = &mn;
        if (mn.id == rel.endpoint_b) mn_b = &mn;
      }
      if (!mn_a || !mn_b) continue;
      active_relations++;

      const ShapeNode* on_a = nullptr;
      const ShapeNode* on_b = nullptr;
      double dist_a = std::numeric_limits<double>::max();
      double dist_b = std::numeric_limits<double>::max();
      geometry_msgs::msg::Point world_a = transformMemoryNodeToWorld(tr, *mn_a);
      geometry_msgs::msg::Point world_b = transformMemoryNodeToWorld(tr, *mn_b);
      for (const auto & on : f.shape_nodes) {
        double dx_a = on.x - world_a.x, dy_a = on.y - world_a.y, dz_a = on.z - world_a.z;
        double d_a = std::sqrt(dx_a*dx_a + dy_a*dy_a + dz_a*dz_a);
        if (d_a < dist_a && d_a < 1.0) { dist_a = d_a; on_a = &on; }
        double dx_b = on.x - world_b.x, dy_b = on.y - world_b.y, dz_b = on.z - world_b.z;
        double d_b = std::sqrt(dx_b*dx_b + dy_b*dy_b + dz_b*dz_b);
        if (d_b < dist_b && d_b < 1.0) { dist_b = d_b; on_b = &on; }
      }
      if (on_a && on_b) {
        double cur_dx = on_b->x - on_a->x, cur_dy = on_b->y - on_a->y, cur_dz = on_b->z - on_a->z;
        double cur_dist = std::sqrt(cur_dx*cur_dx + cur_dy*cur_dy + cur_dz*cur_dz);
        double cur_yaw = std::atan2(cur_dy, cur_dx);
        double cur_h = std::abs(cur_dz);
        double dist_diff = std::abs(cur_dist - rel.expected_distance);
        double yaw_diff = std::abs(cur_yaw - rel.expected_direction_yaw);
        if (yaw_diff > M_PI) yaw_diff = 2.0 * M_PI - yaw_diff;
        double h_diff = std::abs(cur_h - rel.expected_height_delta);
        if (dist_diff < 0.8 && yaw_diff < 0.5 && h_diff < 0.5) matched_relations++;
      }
    }
    double endpoint_relation_score = active_relations == 0 ? 1.0 : static_cast<double>(matched_relations) / active_relations;

    int matched_edges = 0;
    int active_edges = 0;
    for (const auto & edge : mem.edges) {
      const ShapeMemoryNode* mn_a = nullptr;
      const ShapeMemoryNode* mn_b = nullptr;
      for (const auto & mn : mem.nodes) {
        if (mn.id == edge.node_a) mn_a = &mn;
        if (mn.id == edge.node_b) mn_b = &mn;
      }
      if (!mn_a || !mn_b) continue;
      active_edges++;

      const ShapeNode* on_a = nullptr;
      const ShapeNode* on_b = nullptr;
      double dist_a = std::numeric_limits<double>::max();
      double dist_b = std::numeric_limits<double>::max();
      geometry_msgs::msg::Point world_a = transformMemoryNodeToWorld(tr, *mn_a);
      geometry_msgs::msg::Point world_b = transformMemoryNodeToWorld(tr, *mn_b);
      for (const auto & on : f.shape_nodes) {
        double dx_a = on.x - world_a.x, dy_a = on.y - world_a.y, dz_a = on.z - world_a.z;
        double d_a = std::sqrt(dx_a*dx_a + dy_a*dy_a + dz_a*dz_a);
        if (d_a < dist_a && d_a < 1.0) { dist_a = d_a; on_a = &on; }
        double dx_b = on.x - world_b.x, dy_b = on.y - world_b.y, dz_b = on.z - world_b.z;
        double d_b = std::sqrt(dx_b*dx_b + dy_b*dy_b + dz_b*dz_b);
        if (d_b < dist_b && d_b < 1.0) { dist_b = d_b; on_b = &on; }
      }
      if (on_a && on_b) {
        double cur_dx = on_b->x - on_a->x, cur_dy = on_b->y - on_a->y, cur_dz = on_b->z - on_a->z;
        double cur_dist = std::sqrt(cur_dx*cur_dx + cur_dy*cur_dy + cur_dz*cur_dz);
        double cur_yaw = std::atan2(cur_dy, cur_dx);
        double cur_h = std::abs(cur_dz);
        double dist_diff = std::abs(cur_dist - edge.length);
        double yaw_diff = std::abs(cur_yaw - edge.direction_yaw);
        if (yaw_diff > M_PI) yaw_diff = 2.0 * M_PI - yaw_diff;
        double h_diff = std::abs(cur_h - edge.height_delta);
        if (dist_diff < 0.8 && yaw_diff < 0.5 && h_diff < 0.5) matched_edges++;
      }
    }
    double edge_relation_score = active_edges == 0 ? 1.0 : static_cast<double>(matched_edges) / active_edges;

    double h_diff = std::abs(f.height - mem.height);
    double height_profile_score = clamp(1.0 - h_diff / std::max(0.5, mem.height), 0.0, 1.0);

    double raw_score = 0.20 * bbox_overlap_score +
                       0.20 * visible_node_score +
                       0.25 * endpoint_relation_score +
                       0.25 * edge_relation_score +
                       0.10 * height_profile_score;

    return clamp(raw_score, 0.0, 1.0) * mem.confidence * mode_multiplier;
  }

  void buildShapeMemoryFromObservedFeature(ShapeMemory & mem, const ObjectFeature & f) const {
    mem.nodes.clear();
    mem.edges.clear();
    mem.endpoint_relations.clear();

    if (f.shape_nodes.empty()) {
      rebuildBoxShapeMemoryGraph(mem);
      for (auto & n : mem.nodes) n.is_endpoint = true;
      selectEndpointsAndRelations(mem);
      return;
    }

    // Copy nodes and convert coordinates to local space
    for (const auto & n : f.shape_nodes) {
      ShapeMemoryNode smn;
      smn.id = n.id;
      smn.local_x = n.x - f.centroid_x;
      smn.local_y = n.y - f.centroid_y;
      smn.local_z = n.z - f.centroid_z;
      smn.point_count = n.point_count;
      smn.confidence = mem.confidence;
      smn.observed_count = 1;
      smn.missing_count = 0;
      smn.is_endpoint = false;
      smn.is_boundary = false;
      smn.is_corner_candidate = false;
      mem.nodes.push_back(smn);
    }

    // Copy edges and map node indices
    for (const auto & e : f.shape_edges) {
      ShapeMemoryEdge sme;
      sme.id = mem.edges.size();
      sme.node_a = e.a;
      sme.node_b = e.b;

      int idx_a = -1, idx_b = -1;
      for (size_t i = 0; i < f.shape_nodes.size(); ++i) {
        if (f.shape_nodes[i].id == e.a) idx_a = i;
        if (f.shape_nodes[i].id == e.b) idx_b = i;
      }

      if (idx_a != -1 && idx_b != -1) {
        sme.local_dx = f.shape_nodes[idx_b].x - f.shape_nodes[idx_a].x;
        sme.local_dy = f.shape_nodes[idx_b].y - f.shape_nodes[idx_a].y;
        sme.local_dz = f.shape_nodes[idx_b].z - f.shape_nodes[idx_a].z;
        sme.length = std::sqrt(sme.local_dx*sme.local_dx + sme.local_dy*sme.local_dy + sme.local_dz*sme.local_dz);
        sme.direction_yaw = std::atan2(sme.local_dy, sme.local_dx);
        sme.height_delta = std::abs(sme.local_dz);
        sme.is_structural_edge = (e.total_score > 0.7);
      } else {
        sme.local_dx = sme.local_dy = sme.local_dz = 0.0;
        sme.length = 0.0;
        sme.direction_yaw = 0.0;
        sme.height_delta = 0.0;
        sme.is_structural_edge = false;
      }
      sme.top_score = e.top_score;
      sme.side_score = e.side_score;
      sme.front_score = e.front_score;
      sme.confidence = e.total_score;
      sme.observed_count = 1;
      sme.missing_count = 0;
      mem.edges.push_back(sme);
    }

    // Select endpoints and relations
    selectEndpointsAndRelations(mem);
  }


  void initializeShapeMemory(Track & tr, const ObjectFeature & f) const {
    if (!enable_shape_memory_tracking_ || !featureHasShapeQuality(f)) return;
    // manual_prepare 모드에서는 새 track 생성 시 shape graph를 계산하지 않는다.
    // prepare_shape_memory_commit 요청 후에만 계산한다.
    if (shape_graph_compute_policy_ != "always") return;
    ShapeMemory mem;
    mem.valid = true;
    mem.committed = false;
    mem.frozen = false;
    mem.track_id = tr.id;
    mem.length = f.length;
    mem.width = f.width;
    mem.height = f.height;
    mem.yaw = 0.0;
    mem.min_local_x = f.min_x - f.centroid_x;
    mem.max_local_x = f.max_x - f.centroid_x;
    mem.min_local_y = f.min_y - f.centroid_y;
    mem.max_local_y = f.max_y - f.centroid_y;
    mem.min_local_z = f.min_z - f.centroid_z;
    mem.max_local_z = f.max_z - f.centroid_z;
    mem.confidence = clamp(0.25 + 0.05 * f.shape_node_count + 0.02 * f.shape_edge_count, 0.25, 1.0);
    mem.update_count = 1;

    buildShapeMemoryFromObservedFeature(mem, f);
    tr.shape_memory = mem;
  }

  double limitedShrink(double old_value, double observed_value) const {
    if (old_value <= 1e-3) return observed_value;
    const double min_allowed = old_value * (1.0 - clamp(shape_memory_dimension_shrink_limit_, 0.0, 0.95));
    return std::max(observed_value, min_allowed);
  }

  void updateShapeMemory(Track & tr, const ObjectFeature & f, const std::vector<PointXYZIRT> & points) const {
    if (!enable_shape_memory_tracking_) return;

    if (shape_graph_compute_policy_ == "manual_prepare") {
      if (tr.observed_shape_cache.state != "capturing") {
        // Manual prepare is the only mode that builds an observed cache. Ready/empty caches are frozen.
      } else if (tr.observed_shape_cache.captured_frames >= tr.observed_shape_cache.target_capture_frames) {
        tr.observed_shape_cache.state = "ready";
        tr.observed_shape_cache.valid = true;
        return;
      } else {
      double tx = tr.x;
      double ty = tr.y;
      double tz = tr.z;
      double tyaw = getTrackYaw(tr);
      double cos_yaw = std::cos(-tyaw);
      double sin_yaw = std::sin(-tyaw);

      for (int idx : f.indices) {
        if (idx < 0 || idx >= static_cast<int>(points.size())) continue;
        const auto & pt = points[idx];
        double dx = pt.x - tx;
        double dy = pt.y - ty;
        double dz = pt.z - tz;

        PointXYZIRT local_pt = pt;
        local_pt.x = dx * cos_yaw - dy * sin_yaw;
        local_pt.y = dx * sin_yaw + dy * cos_yaw;
        local_pt.z = dz;
        const_cast<Track&>(tr).observed_shape_cache.accumulated_local_points.push_back(local_pt);
      }

      const_cast<Track&>(tr).observed_shape_cache.captured_frames++;
      const_cast<Track&>(tr).observed_shape_cache.last_update_frame = frame_id_;

      if (tr.observed_shape_cache.captured_frames >= tr.observed_shape_cache.target_capture_frames) {
        auto nodes = buildCacheNodes(tr.observed_shape_cache.accumulated_local_points);
        auto edges = buildCacheEdges(nodes);

        const_cast<Track&>(tr).observed_shape_cache.nodes = nodes;
        const_cast<Track&>(tr).observed_shape_cache.edges = edges;
        const_cast<Track&>(tr).observed_shape_cache.confidence = clamp(0.25 + 0.05 * nodes.size() + 0.02 * edges.size(), 0.25, 1.0);
        const_cast<Track&>(tr).observed_shape_cache.valid = true;
        const_cast<Track&>(tr).observed_shape_cache.state = "ready";

        const_cast<Cx128s2BevPerception*>(this)->observed_shape_cache_valid_track_count_++;
        const_cast<Cx128s2BevPerception*>(this)->shape_graph_computed_this_frame_++;
        return;
      }
      }
    }

    if (!tr.shape_memory.valid) {
      if (shape_graph_compute_policy_ == "always") {
        initializeShapeMemory(tr, f);
      }
      return;
    }

    ShapeMemory & mem = tr.shape_memory;

    if (mem.frozen || shape_graph_compute_policy_ == "manual_prepare") {
      for (auto & mn : mem.nodes) {
        bool node_observed = false;
        double tx = tr.x;
        double ty = tr.y;
        double tz = tr.z;
        double tyaw = getTrackYaw(tr);
        double cos_yaw = std::cos(-tyaw);
        double sin_yaw = std::sin(-tyaw);

        for (int idx : f.indices) {
          if (idx < 0 || idx >= static_cast<int>(points.size())) continue;
          const auto & pt = points[idx];
          double dx = pt.x - tx;
          double dy = pt.y - ty;
          double dz = pt.z - tz;
          double lx = dx * cos_yaw - dy * sin_yaw;
          double ly = dx * sin_yaw + dy * cos_yaw;
          double lz = dz;

          double dist = std::sqrt((mn.local_x - lx)*(mn.local_x - lx) +
                                  (mn.local_y - ly)*(mn.local_y - ly) +
                                  (mn.local_z - lz)*(mn.local_z - lz));
          if (dist < std::max(0.4, shape_graph_voxel_size_ * 2.0)) {
            node_observed = true;
            break;
          }
        }

        if (node_observed) {
          mn.observed_count++;
          mn.missing_count = 0;
        } else {
          mn.missing_count++;
        }
      }

      for (auto & me : mem.edges) {
        bool na_obs = false;
        bool nb_obs = false;
        for (const auto & mn : mem.nodes) {
          if (mn.id == me.node_a && mn.missing_count == 0) na_obs = true;
          if (mn.id == me.node_b && mn.missing_count == 0) nb_obs = true;
        }
        if (na_obs && nb_obs) {
          me.observed_count++;
          me.missing_count = 0;
        } else {
          me.missing_count++;
        }
      }

      for (auto & r : mem.endpoint_relations) {
        bool ep_a_obs = false;
        bool ep_b_obs = false;
        for (const auto & mn : mem.nodes) {
          if (mn.id == r.endpoint_a && mn.missing_count == 0) ep_a_obs = true;
          if (mn.id == r.endpoint_b && mn.missing_count == 0) ep_b_obs = true;
        }
        if (ep_a_obs && ep_b_obs) {
          r.observed_count++;
          r.missing_count = 0;
        } else {
          r.missing_count++;
        }
      }

      double decay_val = shape_memory_node_missing_decay_ * 0.01;
      mem.confidence = std::max(0.2, mem.confidence - decay_val);
      return;
    }

    // 이 이하 경로는 always 정책에서만 실행된다.
    // manual_prepare 정책에서는 위의 frozen/manual_prepare 분기에서 이미 return됐으므로
    // 여기까지 도달하지 않는다. 단, 안전을 위해 policy를 한번 더 확인한다.
    if (shape_graph_compute_policy_ != "always") return;

    if (!featureHasShapeQuality(f)) return;

    const double a = clamp(shape_memory_update_alpha_, 0.0, 1.0);
    const double obs_length = shape_memory_allow_dimension_shrink_ ? f.length : limitedShrink(mem.length, f.length);
    const double obs_width = shape_memory_allow_dimension_shrink_ ? f.width : limitedShrink(mem.width, f.width);
    const double obs_height = shape_memory_allow_dimension_shrink_ ? f.height : limitedShrink(mem.height, f.height);

    mem.length = (1.0 - a) * mem.length + a * obs_length;
    mem.width = (1.0 - a) * mem.width + a * obs_width;
    mem.height = (1.0 - a) * mem.height + a * obs_height;

    mem.min_local_x = (1.0 - a) * mem.min_local_x + a * std::min(f.min_x - f.centroid_x, -0.5 * mem.length);
    mem.max_local_x = (1.0 - a) * mem.max_local_x + a * std::max(f.max_x - f.centroid_x, 0.5 * mem.length);
    mem.min_local_y = (1.0 - a) * mem.min_local_y + a * std::min(f.min_y - f.centroid_y, -0.5 * mem.width);
    mem.max_local_y = (1.0 - a) * mem.max_local_y + a * std::max(f.max_y - f.centroid_y, 0.5 * mem.width);
    mem.min_local_z = (1.0 - a) * mem.min_local_z + a * std::min(f.min_z - f.centroid_z, -0.5 * mem.height);
    mem.max_local_z = (1.0 - a) * mem.max_local_z + a * std::max(f.max_z - f.centroid_z, 0.5 * mem.height);

    const double obs_conf = clamp(0.25 + 0.05 * f.shape_node_count + 0.02 * f.shape_edge_count, 0.25, 1.0);
    mem.confidence = clamp((1.0 - a) * mem.confidence + a * obs_conf, 0.0, 1.0);
    mem.update_count++;

    buildShapeMemoryFromObservedFeature(mem, f);
  }

  double computeShapeMemoryMatchScore(
      const Track & tr, const ObjectFeature & f, const std::vector<PointXYZIRT> & points,
      ShapeMemoryMatchBreakdown * breakdown = nullptr) const {
    if (!enable_shape_memory_tracking_ || !tr.shape_memory.valid ||
        memoryMatchingBlockedPair(tr, f)) return 0.0;
    const ShapeMemory & mem = tr.shape_memory;

    if (mem.confidence < shape_memory_min_confidence_to_use_) return 0.0;

    double mode_multiplier = 1.0;
    if (shape_memory_learning_mode_ == "manual_commit") {
      if (!mem.committed) {
        return 0.0;
      }
    } else if (shape_memory_learning_mode_ == "hybrid") {
      if (!mem.committed) {
        mode_multiplier = 0.3;
      }
    }

    if (shape_graph_compute_policy_ == "manual_prepare") {
      const_cast<Cx128s2BevPerception*>(this)->memory_compare_using_cache_count_++;
      return computeCommittedMemoryToCurrentObservationScore(tr, f, points, mode_multiplier, breakdown);
    } else {
      const_cast<Cx128s2BevPerception*>(this)->memory_compare_using_current_shape_graph_count_++;
      const double score = computeShapeGraphToShapeGraphScore(tr, f, mode_multiplier);
      if (breakdown) {
        breakdown->final_score = score;
        breakdown->node_score = score;
      }
      return score;
    }
  }

#include "general/tracking.inc.hpp"

#include "general/publication.inc.hpp"

  void publishCraneBeamMarkers(
      const std_msgs::msg::Header & header,
      const cx128s2_bev_perception::CraneBeamDetectionResult & res,
      const cx128s2_bev_perception::msg::CraneBeamStatus & effective) {
    if (!pub_crane_beam_markers_ || pub_crane_beam_markers_->get_subscription_count() == 0) return;
    visualization_msgs::msg::MarkerArray arr;

    // 1. Beam ROI box
    {
      visualization_msgs::msg::Marker box;
      box.header = header;
      box.ns = "crane_beam_roi";
      box.id = 0;
      box.type = visualization_msgs::msg::Marker::CUBE;
      box.action = visualization_msgs::msg::Marker::ADD;
      box.pose.position.x = (crane_beam_config_.beam_roi_x_min + crane_beam_config_.beam_roi_x_max) * 0.5;
      box.pose.position.y = (crane_beam_config_.beam_roi_y_min + crane_beam_config_.beam_roi_y_max) * 0.5;
      box.pose.position.z = (crane_beam_config_.beam_roi_z_min + crane_beam_config_.beam_roi_z_max) * 0.5;
      box.pose.orientation.w = 1.0;
      box.scale.x = std::max(0.1, crane_beam_config_.beam_roi_x_max - crane_beam_config_.beam_roi_x_min);
      box.scale.y = std::max(0.1, crane_beam_config_.beam_roi_y_max - crane_beam_config_.beam_roi_y_min);
      box.scale.z = std::max(0.1, crane_beam_config_.beam_roi_z_max - crane_beam_config_.beam_roi_z_min);
      box.color.r = 0.0f;
      box.color.g = 1.0f;
      box.color.b = 0.3f;
      box.color.a = 0.18f;
      box.lifetime = rclcpp::Duration::from_seconds(0.5);
      arr.markers.push_back(box);
    }

    // 2. Background ROI box (hidden/deleted by default to keep view clean)
    {
      visualization_msgs::msg::Marker box;
      box.header = header;
      box.ns = "crane_background_roi";
      box.id = 1;
      if (crane_beam_config_.publish_background_roi_marker) {
        box.type = visualization_msgs::msg::Marker::CUBE;
        box.action = visualization_msgs::msg::Marker::ADD;
        box.pose.position.x = (crane_beam_config_.bg_roi_x_min + crane_beam_config_.bg_roi_x_max) * 0.5;
        box.pose.position.y = (crane_beam_config_.bg_roi_y_min + crane_beam_config_.bg_roi_y_max) * 0.5;
        box.pose.position.z = (crane_beam_config_.bg_roi_z_min + crane_beam_config_.bg_roi_z_max) * 0.5;
        box.pose.orientation.w = 1.0;
        box.scale.x = std::max(0.1, crane_beam_config_.bg_roi_x_max - crane_beam_config_.bg_roi_x_min);
        box.scale.y = std::max(0.1, crane_beam_config_.bg_roi_y_max - crane_beam_config_.bg_roi_y_min);
        box.scale.z = std::max(0.1, crane_beam_config_.bg_roi_z_max - crane_beam_config_.bg_roi_z_min);
        box.color.r = 1.0f;
        box.color.g = 0.85f;
        box.color.b = 0.0f;
        box.color.a = 0.08f;
        box.lifetime = rclcpp::Duration::from_seconds(0.5);
      } else {
        box.action = visualization_msgs::msg::Marker::DELETE;
      }
      arr.markers.push_back(box);
    }

    // 3. Beam Cluster PCA major axis line
    if (res.beam_cluster_valid && res.beam_length_raw > 0.05) {
      visualization_msgs::msg::Marker line;
      line.header = header;
      line.ns = "crane_beam_pca_axis";
      line.id = 2;
      line.type = visualization_msgs::msg::Marker::LINE_LIST;
      line.action = visualization_msgs::msg::Marker::ADD;
      line.pose.orientation.w = 1.0;
      line.scale.x = 0.12;  // line width
      line.color.r = 0.0f;
      line.color.g = 0.9f;
      line.color.b = 1.0f;
      line.color.a = 0.95f;
      line.lifetime = rclcpp::Duration::from_seconds(0.5);

      const double half_len = res.beam_length_raw * 0.5;
      const double cos_y = std::cos(res.beam_major_axis_yaw);
      const double sin_y = std::sin(res.beam_major_axis_yaw);
      geometry_msgs::msg::Point p1, p2;
      p1.x = res.cluster_centroid_x - half_len * cos_y;
      p1.y = res.cluster_centroid_y - half_len * sin_y;
      p1.z = res.z_beam_filtered;
      p2.x = res.cluster_centroid_x + half_len * cos_y;
      p2.y = res.cluster_centroid_y + half_len * sin_y;
      p2.z = res.z_beam_filtered;
      line.points.push_back(p1);
      line.points.push_back(p2);
      arr.markers.push_back(line);
    }

    // 4. Tracked Beam 3D Bounding Box (Oriented Bounding Box)
    if (res.beam_cluster_valid && res.beam_length_raw > 0.05) {
      visualization_msgs::msg::Marker obb;
      obb.header = header;
      obb.ns = "crane_beam_tracked_box";
      obb.id = 4;
      obb.type = visualization_msgs::msg::Marker::CUBE;
      obb.action = visualization_msgs::msg::Marker::ADD;
      obb.pose.position.x = res.cluster_centroid_x;
      obb.pose.position.y = res.cluster_centroid_y;
      obb.pose.position.z = res.z_beam_filtered;
      obb.pose.orientation.z = std::sin(res.beam_major_axis_yaw * 0.5);
      obb.pose.orientation.w = std::cos(res.beam_major_axis_yaw * 0.5);
      obb.scale.x = std::max(0.2, res.beam_length_raw);
      obb.scale.y = 0.50;
      obb.scale.z = std::max(0.1, crane_beam_config_.beam_default_height);
      obb.color.r = 0.0f;
      obb.color.g = 0.8f;
      obb.color.b = 1.0f;
      obb.color.a = 0.40f;
      obb.lifetime = rclcpp::Duration::from_seconds(0.5);
      arr.markers.push_back(obb);
    }

    // 5. Support Surface Plane
    if (res.support_valid) {
      visualization_msgs::msg::Marker sup;
      sup.header = header;
      sup.ns = "crane_beam_support_surface";
      sup.id = 5;
      sup.type = visualization_msgs::msg::Marker::CUBE;
      sup.action = visualization_msgs::msg::Marker::ADD;
      sup.pose.position.x = res.cluster_centroid_x;
      sup.pose.position.y = res.cluster_centroid_y;
      sup.pose.position.z = res.support_z;
      sup.pose.orientation.z = std::sin(res.beam_major_axis_yaw * 0.5);
      sup.pose.orientation.w = std::cos(res.beam_major_axis_yaw * 0.5);
      sup.scale.x = std::max(0.4, res.beam_length_raw + 2.0 * crane_beam_config_.support_search_margin_xy);
      sup.scale.y = 0.50 + 2.0 * crane_beam_config_.support_search_margin_xy;
      sup.scale.z = 0.04;
      sup.color.r = 0.2f;
      sup.color.g = 0.4f;
      sup.color.b = 1.0f;
      sup.color.a = 0.35f;
      sup.lifetime = rclcpp::Duration::from_seconds(0.5);
      arr.markers.push_back(sup);
    }

    // 5.5 Longitudinal Tilt Segments (Tip A, Center, Tip B)
    if (res.beam_cluster_valid && res.beam_length_raw > 0.5 && crane_beam_config_.tilt_segmentation_enabled) {
      const int K = 3;
      const double seg_len = res.beam_length_raw / K;
      const double cos_y = std::cos(res.beam_major_axis_yaw);
      const double sin_y = std::sin(res.beam_major_axis_yaw);
      for (int k = 0; k < K; ++k) {
        double u_k = -res.beam_length_raw * 0.5 + (k + 0.5) * seg_len;
        visualization_msgs::msg::Marker seg;
        seg.header = header;
        seg.ns = "crane_beam_tilt_segments";
        seg.id = 10 + k;
        seg.type = visualization_msgs::msg::Marker::CUBE;
        seg.action = visualization_msgs::msg::Marker::ADD;
        seg.pose.position.x = res.cluster_centroid_x + u_k * cos_y;
        seg.pose.position.y = res.cluster_centroid_y + u_k * sin_y;
        seg.pose.position.z = res.z_beam_filtered;
        seg.pose.orientation.z = std::sin(res.beam_major_axis_yaw * 0.5);
        seg.pose.orientation.w = std::cos(res.beam_major_axis_yaw * 0.5);
        seg.scale.x = seg_len * 0.95;
        seg.scale.y = 0.45;
        seg.scale.z = 0.25;
        if (k == 0 && res.contact_candidate) {
          seg.color.r = 0.2f; seg.color.g = 1.0f; seg.color.b = 0.2f; seg.color.a = 0.60f;
        } else if (res.supported_segment_ratio >= 0.60) {
          seg.color.r = 0.2f; seg.color.g = 1.0f; seg.color.b = 0.2f; seg.color.a = 0.60f;
        } else {
          seg.color.r = 1.0f; seg.color.g = 0.5f; seg.color.b = 0.1f; seg.color.a = 0.45f;
        }
        seg.lifetime = rclcpp::Duration::from_seconds(0.5);
        arr.markers.push_back(seg);
      }
    }

    // 6. State & Measurement Info Overlay Text
    {
      visualization_msgs::msg::Marker text;
      text.header = header;
      text.ns = "crane_beam_info_text";
      text.id = 3;
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;
      const bool is_truck_area = (crane_beam_config_.beam_roi_x_min < 0.0);
      text.pose.position.x = is_truck_area ? -12.0 : 1.0;
      text.pose.position.y = is_truck_area ? -3.0 : -6.0;
      text.pose.position.z = is_truck_area ? 6.0 : 7.5;
      text.pose.orientation.w = 1.0;
      text.scale.z = 1.1;
      text.color.r = 1.0f;
      text.color.g = 0.95f;
      text.color.b = 0.2f;
      text.color.a = 1.0f;
      text.lifetime = rclcpp::Duration::from_seconds(0.5);

      const std::string src_str = (res.measurement_source == cx128s2_bev_perception::BeamMeasurementSource::INITIAL_ROI)
          ? "INITIAL_ROI" : "TRACKED_CLUSTER";
      std::ostringstream ss;
      ss << "[Crane Beam Work Detection]\n"
         << "Mode: " << effective.control_mode << " | State: " << effective.state
         << " | working_status: " << effective.working_status << "\n"
         << "AUTO State: " << effective.auto_state << " | Beam detected: " << effective.beam_detected << "\n"
         << "Source: " << src_str << "\n"
         << "Beam Z: " << std::fixed << std::setprecision(2) << res.z_beam_filtered
         << " m | Support Z: " << (res.support_valid ? (std::to_string(res.support_z).substr(0, 4) + " m") : "N/A") << "\n"
         << "motion_dz: " << std::showpos << res.motion_dz << std::noshowpos
         << " m/fr (cum: " << res.motion_cumulative_dz << " m)\n"
         << "Contact: " << (res.contact_candidate ? "TRUE" : "FALSE")
         << " | Supported: " << std::fixed << std::setprecision(2) << res.supported_segment_ratio << "\n"
         << "Placed: " << (res.placed_candidate ? "TRUE" : "FALSE")
         << " | Work Complete: " << (effective.work_complete ? "TRUE" : "FALSE");
      text.text = ss.str();
      arr.markers.push_back(text);
    }

    pub_crane_beam_markers_->publish(arr);
  }

  void publishCraneHookMarkers(
      const std_msgs::msg::Header & header,
      const cx128s2_bev_perception::CraneHookResult & hook_res,
      const cx128s2_bev_perception::msg::CraneBeamStatus & effective,
      const cx128s2_bev_perception::CraneTopDownHookResult * top_down_res = nullptr,
      const cx128s2_bev_perception::CraneWorkStatusResult * work_res = nullptr,
      const cx128s2_bev_perception::CraneBeamGeometry * beam_geom = nullptr,
      const std::vector<cx128s2_bev_perception::CraneSubcluster> * subclusters = nullptr,
      double pure_cpp_ms = 0.0) {
    (void)effective;
    (void)work_res;
    (void)beam_geom;
    (void)subclusters;
    (void)pure_cpp_ms;
    if (!pub_crane_hook_markers_ || pub_crane_hook_markers_->get_subscription_count() == 0) return;

    visualization_msgs::msg::MarkerArray arr;

    // Clear all previous markers first so stale markers never accumulate across frames
    {
      visualization_msgs::msg::Marker clear_m;
      clear_m.header = header;
      clear_m.action = visualization_msgs::msg::Marker::DELETEALL;
      arr.markers.push_back(clear_m);
    }

    const cx128s2_bev_perception::CraneHookNearestCluster empty_nc{};
    const auto & left_lc = top_down_res ? top_down_res->left_largest_cluster : empty_nc;
    const auto & right_lc = top_down_res ? top_down_res->right_largest_cluster : empty_nc;

    for (int side = 0; side < 2; ++side) {
      const bool is_left = (side == 0);
      const auto & hook = is_left ? hook_res.left : hook_res.right;

      const int chain_idx = is_left ? (top_down_res ? top_down_res->left_chain_idx : -1)
                                    : (top_down_res ? top_down_res->right_chain_idx : -1);
      const cx128s2_bev_perception::CraneHookChain * chain_ptr = nullptr;
      if (top_down_res && chain_idx >= 0 && chain_idx < static_cast<int>(top_down_res->chains.size())) {
        chain_ptr = &top_down_res->chains[chain_idx];
      }

      const auto * fx_state_ptr = top_down_res ? (is_left ? &top_down_res->left_root_state : &top_down_res->right_root_state) : nullptr;
      const auto & fx_roi = (chain_ptr && chain_ptr->root_roi.sizeX() > 0.0) ? chain_ptr->root_roi :
                            (crane_pipeline_ ? (is_left ? crane_pipeline_->config().fixed_root.left_roi : crane_pipeline_->config().fixed_root.right_roi) : cx128s2_bev_perception::CraneFixedRootRoi{});

      const double root_x = chain_ptr ? chain_ptr->root_x : (fx_roi.sizeX() > 0.0 ? fx_roi.centerX() : hook.x);
      const double root_y = chain_ptr ? chain_ptr->root_y : (fx_roi.sizeY() > 0.0 ? fx_roi.centerY() : hook.y);
      const double root_z = chain_ptr ? chain_ptr->root_z : (fx_roi.sizeZ() > 0.0 ? fx_roi.centerZ() : hook.z);

      const double lowest_x = chain_ptr ? chain_ptr->lowest_x : (hook.detected ? hook.x : root_x);
      const double lowest_y = chain_ptr ? chain_ptr->lowest_y : (hook.detected ? hook.y : root_y);
      const double lowest_z = chain_ptr ? chain_ptr->lowest_z : (hook.detected ? hook.z : root_z);

      const size_t chain_nodes_count = chain_ptr ? chain_ptr->nodes.size() : (hook.detected ? 1 : 0);
      const auto & lc = is_left ? left_lc : right_lc;

      // 0. Fixed Root ROI Bounding Box Marker
      if (fx_roi.sizeX() > 0.0) {
        visualization_msgs::msg::Marker roi_m;
        roi_m.header = header;
        roi_m.ns = "fixed_root_roi";
        roi_m.id = side;
        roi_m.type = visualization_msgs::msg::Marker::CUBE;
        roi_m.action = visualization_msgs::msg::Marker::ADD;
        roi_m.pose.position.x = fx_roi.centerX();
        roi_m.pose.position.y = fx_roi.centerY();
        roi_m.pose.position.z = fx_roi.centerZ();
        roi_m.pose.orientation.w = 1.0;
        roi_m.scale.x = fx_roi.sizeX();
        roi_m.scale.y = fx_roi.sizeY();
        roi_m.scale.z = fx_roi.sizeZ();
        if (is_left) {
          roi_m.color.r = 0.00f; roi_m.color.g = 0.85f; roi_m.color.b = 1.00f; roi_m.color.a = 0.18f;
        } else {
          roi_m.color.r = 0.15f; roi_m.color.g = 0.95f; roi_m.color.b = 0.45f; roi_m.color.a = 0.18f;
        }
        roi_m.lifetime = rclcpp::Duration::from_seconds(0.3);
        arr.markers.push_back(roi_m);
      }

      // 1. Root Node Marker (Top of Crane Plumb-Line)
      {
        visualization_msgs::msg::Marker root_m;
        root_m.header = header;
        root_m.ns = "hook_root";
        root_m.id = side;
        root_m.type = visualization_msgs::msg::Marker::SPHERE;
        root_m.action = visualization_msgs::msg::Marker::ADD;
        root_m.pose.position.x = root_x;
        root_m.pose.position.y = root_y;
        root_m.pose.position.z = root_z;
        root_m.pose.orientation.w = 1.0;
        root_m.scale.x = 0.22;
        root_m.scale.y = 0.22;
        root_m.scale.z = 0.22;
        const bool is_obs = fx_state_ptr ? (fx_state_ptr->obs_status == cx128s2_bev_perception::CraneRootObservationStatus::OBSERVED) : true;
        const float alpha = is_obs ? 0.95f : 0.40f;
        if (is_left) {
          root_m.color.r = 0.00f; root_m.color.g = 0.85f; root_m.color.b = 1.00f; root_m.color.a = alpha;
        } else {
          root_m.color.r = 0.15f; root_m.color.g = 0.95f; root_m.color.b = 0.45f; root_m.color.a = alpha;
        }
        root_m.lifetime = rclcpp::Duration::from_seconds(0.3);
        arr.markers.push_back(root_m);
      }

      // If neither hook nor chain is active, continue after showing the permanent Root ROI & Sphere
      if (!hook.detected && !chain_ptr) continue;

      // 2. Top-Down Connection Lines: Direct (Solid) vs Estimated (Gap-Tolerant)
      if (chain_ptr && chain_ptr->nodes.size() > 1) {
        visualization_msgs::msg::Marker direct_line;
        direct_line.header = header;
        direct_line.ns = "chain_direct_line";
        direct_line.id = side;
        direct_line.type = visualization_msgs::msg::Marker::LINE_LIST;
        direct_line.action = visualization_msgs::msg::Marker::ADD;
        direct_line.pose.orientation.w = 1.0;
        direct_line.scale.x = 0.040;
        if (is_left) {
          direct_line.color.r = 0.00f; direct_line.color.g = 0.85f; direct_line.color.b = 1.00f; direct_line.color.a = 0.92f;
        } else {
          direct_line.color.r = 0.15f; direct_line.color.g = 0.95f; direct_line.color.b = 0.45f; direct_line.color.a = 0.92f;
        }
        direct_line.lifetime = rclcpp::Duration::from_seconds(0.3);

        visualization_msgs::msg::Marker gap_line;
        gap_line.header = header;
        gap_line.ns = "chain_gap_line";
        gap_line.id = side;
        gap_line.type = visualization_msgs::msg::Marker::LINE_LIST;
        gap_line.action = visualization_msgs::msg::Marker::ADD;
        gap_line.pose.orientation.w = 1.0;
        gap_line.scale.x = 0.032;
        gap_line.color.r = 1.00f; gap_line.color.g = 0.70f; gap_line.color.b = 0.10f; gap_line.color.a = 0.90f; // Amber
        gap_line.lifetime = rclcpp::Duration::from_seconds(0.3);

        for (size_t i = 1; i < chain_ptr->nodes.size(); ++i) {
          const auto & p_prev = chain_ptr->nodes[i - 1];
          const auto & p_curr = chain_ptr->nodes[i];
          geometry_msgs::msg::Point pt1, pt2;
          pt1.x = p_prev.x; pt1.y = p_prev.y; pt1.z = p_prev.z;
          pt2.x = p_curr.x; pt2.y = p_curr.y; pt2.z = p_curr.z;
          if (p_curr.is_connection_stable) {
            direct_line.points.push_back(pt1);
            direct_line.points.push_back(pt2);
          } else if (p_curr.is_estimated_gap || p_curr.is_held || p_prev.is_held) {
            gap_line.points.push_back(pt1);
            gap_line.points.push_back(pt2);
          } else {
            direct_line.points.push_back(pt1);
            direct_line.points.push_back(pt2);
          }
        }

        if (!direct_line.points.empty()) arr.markers.push_back(direct_line);
        if (!gap_line.points.empty()) arr.markers.push_back(gap_line);

        // 2-B. Layer-wise Dual-ROI Anchoring & Motion Monitoring Markers
        const auto & dual_cfg = crane_pipeline_->config().dual_roi_anchor;
        if (dual_cfg.enabled && dual_cfg.publish_markers) {
          for (size_t ni = 1; ni < chain_ptr->nodes.size(); ++ni) {
            const auto & cn = chain_ptr->nodes[ni];
            if (!cn.anchor_locked) continue;

            // Anchor IDs can repeat across preserved Chain nodes; Marker IDs must not.
            const int marker_id_base = static_cast<int>(ni);

            // Fixed Monitoring ROI (CUBE)
            visualization_msgs::msg::Marker froi_m;
            froi_m.header = header;
            froi_m.ns = is_left ? "left_layer_fixed_roi" : "right_layer_fixed_roi";
            froi_m.id = marker_id_base;
            froi_m.type = visualization_msgs::msg::Marker::CUBE;
            froi_m.action = visualization_msgs::msg::Marker::ADD;
            froi_m.pose.position.x = cn.anchor_x;
            froi_m.pose.position.y = cn.anchor_y;
            froi_m.pose.position.z = cn.anchor_z;
            froi_m.pose.orientation.w = 1.0;
            froi_m.scale.x = dual_cfg.fixed_roi_size_xy;
            froi_m.scale.y = dual_cfg.fixed_roi_size_xy;
            froi_m.scale.z = dual_cfg.fixed_roi_size_z;
            if (cn.is_sheave) {
              froi_m.color.r = 0.70f; froi_m.color.g = 0.20f; froi_m.color.b = 0.90f; froi_m.color.a = 0.32f;
            } else if (cn.is_beam_intruded) {
              froi_m.color.r = 1.00f; froi_m.color.g = 0.15f; froi_m.color.b = 0.15f; froi_m.color.a = 0.35f;
            } else if (cn.is_held) {
              froi_m.color.r = 1.00f; froi_m.color.g = 0.70f; froi_m.color.b = 0.10f; froi_m.color.a = 0.25f;
            } else {
              froi_m.color.r = 0.10f; froi_m.color.g = 0.35f; froi_m.color.b = 0.95f; froi_m.color.a = 0.28f;
            }
            froi_m.lifetime = rclcpp::Duration::from_seconds(0.3);
            arr.markers.push_back(froi_m);

            // Moving Monitoring ROI (CUBE)
            visualization_msgs::msg::Marker mroi_m;
            mroi_m.header = header;
            mroi_m.ns = is_left ? "left_layer_moving_roi" : "right_layer_moving_roi";
            mroi_m.id = marker_id_base;
            mroi_m.type = visualization_msgs::msg::Marker::CUBE;
            mroi_m.action = visualization_msgs::msg::Marker::ADD;
            mroi_m.pose.position.x = cn.moving_x;
            mroi_m.pose.position.y = cn.moving_y;
            mroi_m.pose.position.z = cn.moving_z;
            mroi_m.pose.orientation.w = 1.0;
            mroi_m.scale.x = dual_cfg.moving_roi_size_xy;
            mroi_m.scale.y = dual_cfg.moving_roi_size_xy;
            mroi_m.scale.z = dual_cfg.moving_roi_size_z;
            if (cn.is_sheave) {
              mroi_m.color.r = 0.85f; mroi_m.color.g = 0.30f; mroi_m.color.b = 1.00f; mroi_m.color.a = 0.22f;
            } else {
              mroi_m.color.r = 0.05f; mroi_m.color.g = 0.85f; mroi_m.color.b = 0.90f; mroi_m.color.a = 0.18f;
            }
            mroi_m.lifetime = rclcpp::Duration::from_seconds(0.3);
            arr.markers.push_back(mroi_m);

            // Fixed Anchor Center Sphere
            visualization_msgs::msg::Marker fanchor_m;
            fanchor_m.header = header;
            fanchor_m.ns = is_left ? "left_layer_fixed_anchor" : "right_layer_fixed_anchor";
            fanchor_m.id = marker_id_base;
            fanchor_m.type = visualization_msgs::msg::Marker::SPHERE;
            fanchor_m.action = visualization_msgs::msg::Marker::ADD;
            fanchor_m.pose.position.x = cn.anchor_x;
            fanchor_m.pose.position.y = cn.anchor_y;
            fanchor_m.pose.position.z = cn.anchor_z;
            fanchor_m.pose.orientation.w = 1.0;
            fanchor_m.scale.x = 0.08; fanchor_m.scale.y = 0.08; fanchor_m.scale.z = 0.08;
            if (cn.is_relocated_this_frame) {
              fanchor_m.color.r = 1.00f; fanchor_m.color.g = 0.84f; fanchor_m.color.b = 0.00f; fanchor_m.color.a = 0.95f;
            } else {
              fanchor_m.color.r = 0.05f; fanchor_m.color.g = 0.40f; fanchor_m.color.b = 1.00f; fanchor_m.color.a = 0.90f;
            }
            fanchor_m.lifetime = rclcpp::Duration::from_seconds(0.3);
            arr.markers.push_back(fanchor_m);

            // Moving Center Sphere
            visualization_msgs::msg::Marker mcent_m;
            mcent_m.header = header;
            mcent_m.ns = is_left ? "left_layer_moving_center" : "right_layer_moving_center";
            mcent_m.id = marker_id_base;
            mcent_m.type = visualization_msgs::msg::Marker::SPHERE;
            mcent_m.action = visualization_msgs::msg::Marker::ADD;
            mcent_m.pose.position.x = cn.moving_x;
            mcent_m.pose.position.y = cn.moving_y;
            mcent_m.pose.position.z = cn.moving_z;
            mcent_m.pose.orientation.w = 1.0;
            mcent_m.scale.x = 0.06; mcent_m.scale.y = 0.06; mcent_m.scale.z = 0.06;
            mcent_m.color.r = 0.00f; mcent_m.color.g = 1.00f; mcent_m.color.b = 0.85f; mcent_m.color.a = 0.90f;
            mcent_m.lifetime = rclcpp::Duration::from_seconds(0.3);
            arr.markers.push_back(mcent_m);

            // Offset Line between Anchor and Moving Center
            if (cn.disp_from_anchor_xy > 0.02 || cn.disp_from_anchor_z > 0.02) {
              visualization_msgs::msg::Marker line_m;
              line_m.header = header;
              line_m.ns = is_left ? "left_layer_offset_line" : "right_layer_offset_line";
              line_m.id = marker_id_base;
              line_m.type = visualization_msgs::msg::Marker::LINE_LIST;
              line_m.action = visualization_msgs::msg::Marker::ADD;
              line_m.pose.orientation.w = 1.0;
              line_m.scale.x = 0.015;
              geometry_msgs::msg::Point p1, p2;
              p1.x = cn.anchor_x; p1.y = cn.anchor_y; p1.z = cn.anchor_z;
              p2.x = cn.moving_x; p2.y = cn.moving_y; p2.z = cn.moving_z;
              line_m.points.push_back(p1);
              line_m.points.push_back(p2);
              line_m.color.r = 0.95f; line_m.color.g = 0.95f; line_m.color.b = 0.20f; line_m.color.a = 0.85f;
              line_m.lifetime = rclcpp::Duration::from_seconds(0.3);
              arr.markers.push_back(line_m);
            }
          }
        }

        // 2-C. Virtual Rope Centerline & Wire Bundle Markers
        const auto & vr_cfg = crane_pipeline_->config().virtual_rope;
        if (vr_cfg.enabled && vr_cfg.publish_markers && chain_ptr) {
          // Virtual Rope Centerline LINE_STRIP (Root -> Virtual Node Centers)
          visualization_msgs::msg::Marker vr_line;
          vr_line.header = header;
          vr_line.ns = is_left ? "left_virtual_rope_centerline" : "right_virtual_rope_centerline";
          vr_line.id = side;
          vr_line.type = visualization_msgs::msg::Marker::LINE_STRIP;
          vr_line.action = visualization_msgs::msg::Marker::ADD;
          vr_line.pose.orientation.w = 1.0;
          vr_line.scale.x = 0.035; // distinct visible thickness
          // Cyan / Azure color
          vr_line.color.r = 0.00f; vr_line.color.g = 0.95f; vr_line.color.b = 1.00f; vr_line.color.a = 0.95f;
          vr_line.lifetime = rclcpp::Duration::from_seconds(0.3);

          for (const auto & cn : chain_ptr->nodes) {
            geometry_msgs::msg::Point p;
            p.x = cn.x; p.y = cn.y; p.z = cn.z;
            vr_line.points.push_back(p);
          }
          if (vr_line.points.size() >= 2) {
            arr.markers.push_back(vr_line);
          }

          // Strand A & B Spheres and Cross-Tie Line
          for (size_t ni = 1; ni < chain_ptr->nodes.size(); ++ni) {
            const auto & cn = chain_ptr->nodes[ni];
            if (!cn.has_virtual_rope) continue;
            const auto & b = cn.wire_bundle;
            const int marker_id_base = side * 100 + static_cast<int>(ni);

            if (b.mode == cx128s2_bev_perception::CraneWireBundleMode::BOTH_STRANDS) {
              // Strand A sphere (gold/amber)
              visualization_msgs::msg::Marker sa_m;
              sa_m.header = header;
              sa_m.ns = is_left ? "left_strand_a" : "right_strand_a";
              sa_m.id = marker_id_base;
              sa_m.type = visualization_msgs::msg::Marker::SPHERE;
              sa_m.action = visualization_msgs::msg::Marker::ADD;
              sa_m.pose.position.x = b.strand_a_x; sa_m.pose.position.y = b.strand_a_y; sa_m.pose.position.z = b.center_z;
              sa_m.pose.orientation.w = 1.0;
              sa_m.scale.x = 0.05; sa_m.scale.y = 0.05; sa_m.scale.z = 0.05;
              sa_m.color.r = 1.00f; sa_m.color.g = 0.80f; sa_m.color.b = 0.20f; sa_m.color.a = 0.90f;
              sa_m.lifetime = rclcpp::Duration::from_seconds(0.3);
              arr.markers.push_back(sa_m);

              // Strand B sphere (gold/amber)
              visualization_msgs::msg::Marker sb_m;
              sb_m.header = header;
              sb_m.ns = is_left ? "left_strand_b" : "right_strand_b";
              sb_m.id = marker_id_base;
              sb_m.type = visualization_msgs::msg::Marker::SPHERE;
              sb_m.action = visualization_msgs::msg::Marker::ADD;
              sb_m.pose.position.x = b.strand_b_x; sb_m.pose.position.y = b.strand_b_y; sb_m.pose.position.z = b.center_z;
              sb_m.pose.orientation.w = 1.0;
              sb_m.scale.x = 0.05; sb_m.scale.y = 0.05; sb_m.scale.z = 0.05;
              sb_m.color.r = 1.00f; sb_m.color.g = 0.80f; sb_m.color.b = 0.20f; sb_m.color.a = 0.90f;
              sb_m.lifetime = rclcpp::Duration::from_seconds(0.3);
              arr.markers.push_back(sb_m);

              // Cross-tie line
              visualization_msgs::msg::Marker tie_m;
              tie_m.header = header;
              tie_m.ns = is_left ? "left_strand_tie" : "right_strand_tie";
              tie_m.id = marker_id_base;
              tie_m.type = visualization_msgs::msg::Marker::LINE_LIST;
              tie_m.action = visualization_msgs::msg::Marker::ADD;
              tie_m.pose.orientation.w = 1.0;
              tie_m.scale.x = 0.012;
              geometry_msgs::msg::Point pa, pb;
              pa.x = b.strand_a_x; pa.y = b.strand_a_y; pa.z = b.center_z;
              pb.x = b.strand_b_x; pb.y = b.strand_b_y; pb.z = b.center_z;
              tie_m.points.push_back(pa); tie_m.points.push_back(pb);
              tie_m.color.r = 1.00f; tie_m.color.g = 0.85f; tie_m.color.b = 0.30f; tie_m.color.a = 0.80f;
              tie_m.lifetime = rclcpp::Duration::from_seconds(0.3);
              arr.markers.push_back(tie_m);
            }
          }
        }

        // Housing diagnostics are independent of terminal availability.
        const auto & sheave_cfg = crane_pipeline_->config().sheave;
        if (sheave_cfg.enabled && sheave_cfg.publish_markers && top_down_res) {
          const auto & housing = is_left ? top_down_res->left_housing : top_down_res->right_housing;
          const auto & memory = is_left ? top_down_res->left_housing_memory : top_down_res->right_housing_memory;
          const auto & sh = housing.sheave;
          using Status = cx128s2_bev_perception::CraneSheaveDetectionStatus;
          if(is_left && sheave_cfg.debug_seed_world.size()==3){
            visualization_msgs::msg::Marker seed;seed.header=header;seed.ns="housing_user_positive_seed";seed.id=0;
            seed.type=visualization_msgs::msg::Marker::SPHERE;seed.action=visualization_msgs::msg::Marker::ADD;seed.pose.orientation.w=1;
            seed.pose.position.x=sheave_cfg.debug_seed_world[0];seed.pose.position.y=sheave_cfg.debug_seed_world[1];seed.pose.position.z=sheave_cfg.debug_seed_world[2];
            seed.scale.x=seed.scale.y=seed.scale.z=.07;seed.color.r=1;seed.color.g=.1;seed.color.a=.7;seed.lifetime=rclcpp::Duration::from_seconds(.3);arr.markers.push_back(seed);
            seed.type=visualization_msgs::msg::Marker::TEXT_VIEW_FACING;seed.id=1;seed.scale.z=.07;seed.pose.position.z+=.12;seed.text="User positive seed (world) / NOT axis truth";arr.markers.push_back(seed);
          }

          if (sh.status != Status::NONE) {
            auto marker = [&](const std::string & ns, int type) {
              visualization_msgs::msg::Marker m;
              m.header = header; m.ns = ns; m.id = side; m.type = type;
              m.action = visualization_msgs::msg::Marker::ADD; m.pose.orientation.w = 1.0;
              m.lifetime = rclcpp::Duration::from_seconds(0.3);
              m.color.r = 0.8f; m.color.g = 0.3f; m.color.b = 1.0f;
              m.color.a = housing.is_held ? 0.35f : 0.85f;
              return m;
            };
            auto sphere = [&](const std::string & ns, double x, double y, double z) {
              auto m = marker(ns, visualization_msgs::msg::Marker::SPHERE);
              m.pose.position.x=x; m.pose.position.y=y; m.pose.position.z=z;
              m.scale.x=m.scale.y=m.scale.z=0.045;
              return m;
            };
            auto roi_box = [&](const std::string & ns, const cx128s2_bev_perception::CraneFixedRootRoi & roi, float r, float g) {
              auto m=marker(ns, visualization_msgs::msg::Marker::CUBE);
              m.pose.position.x=roi.centerX();m.pose.position.y=roi.centerY();m.pose.position.z=roi.centerZ();
              m.scale.x=roi.sizeX();m.scale.y=roi.sizeY();m.scale.z=roi.sizeZ();
              m.color.r=r;m.color.g=g;m.color.a=0.08f;arr.markers.push_back(m);
            };
            if (!housing.is_held) {
              auto cloud=marker("housing_candidate_points", visualization_msgs::msg::Marker::POINTS);
              cloud.scale.x=cloud.scale.y=0.018;
              const auto & active=(!sh.points_from_current_frame && crane_pipeline_->hasAccumulatedPoints())?crane_pipeline_->accumulatedPoints():frame_points_;
              for(int idx:sh.point_indices)if(idx>=0 && static_cast<size_t>(idx)<active.size()) {
                geometry_msgs::msg::Point p;p.x=active[idx].x;p.y=active[idx].y;p.z=active[idx].z;cloud.points.push_back(p);
              }
              arr.markers.push_back(cloud);
              auto raw=sphere("housing_observed_center",sh.outer_center_x,sh.outer_center_y,sh.outer_center_z);
              raw.color.r=1.0f;raw.color.g=0.55f;raw.color.b=0.0f;arr.markers.push_back(raw);
              auto box=marker("housing_candidate_bbox", visualization_msgs::msg::Marker::CUBE);
              box.pose.position.x=(sh.min_x+sh.max_x)*0.5;box.pose.position.y=(sh.min_y+sh.max_y)*0.5;box.pose.position.z=(sh.min_z+sh.max_z)*0.5;
              box.scale.x=sh.max_x-sh.min_x;box.scale.y=sh.max_y-sh.min_y;box.scale.z=sh.max_z-sh.min_z;box.color.a=0.12f;arr.markers.push_back(box);
            }
            auto tracked=sphere("housing_tracked_center",housing.x,housing.y,housing.z);
            tracked.color.r=0.1f;tracked.color.g=1.0f;tracked.color.b=0.8f;arr.markers.push_back(tracked);
            if (housing.anchor_locked) roi_box("housing_fixed_roi",housing.fixed_roi,0.1f,0.3f);
            roi_box("housing_moving_roi",housing.moving_roi,0.1f,1.0f);
            auto label=marker("housing_memory_status", visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
            label.pose.position.x=housing.x;label.pose.position.y=housing.y+0.3;label.pose.position.z=housing.z;label.scale.z=0.09;
            std::ostringstream text;
            text << (is_left?"LEFT":"RIGHT") << " Housing candidate " << to_string(sh.status)
                 << "\nTrack=" << memory.track_id << " Geometry=" << (memory.confirmed?"CONFIRMED":(memory.dormant?"DORMANT":"PENDING")) << " match=" << sh.geometry_matched
                 << " hits=" << memory.confirm_count << " miss=" << memory.missed_frames
                 << " reloc=" << housing.relocate_count
                 << "\nReference=" << (sh.reference_id.empty()?"UNMATCHED":sh.reference_id)
                 << (sh.reference_matched?" MATCH":" NO_CURRENT_MATCH")
                 << " range=" << (sh.sensor_range>0?std::to_string(sh.sensor_range):"UNKNOWN")
                 << " score=" << (sh.reference_score>=0?std::to_string(sh.reference_score):"UNKNOWN")
                 << " evidence=" << sh.shape_reliability << " assisted=" << sh.reference_assisted << "\nAxis=UNKNOWN";
            label.text=text.str();arr.markers.push_back(label);
          }
          // Preserve the original rope when a separately validated output terminal exists.
          if (chain_ptr->has_sheave && chain_idx < static_cast<int>(top_down_res->original_chains.size())) {
            visualization_msgs::msg::Marker original;
            original.header=header;original.ns="original_virtual_rope";original.id=side;
            original.type=visualization_msgs::msg::Marker::LINE_STRIP;original.action=visualization_msgs::msg::Marker::ADD;
            original.pose.orientation.w=1.0;original.scale.x=0.015;original.color.r=0.7f;original.color.g=0.7f;original.color.b=0.7f;original.color.a=0.6f;
            original.lifetime=rclcpp::Duration::from_seconds(0.3);
            for(const auto & node:top_down_res->original_chains[chain_idx].nodes){geometry_msgs::msg::Point p;p.x=node.x;p.y=node.y;p.z=node.z;original.points.push_back(p);}
            arr.markers.push_back(original);
          }
        }

      }

      // 3. Lowest Terminal Node Marker
      {
        visualization_msgs::msg::Marker term_m;
        term_m.header = header;
        term_m.ns = "hook_terminal";
        term_m.id = side;
        term_m.type = visualization_msgs::msg::Marker::SPHERE;
        term_m.action = visualization_msgs::msg::Marker::ADD;
        term_m.pose.position.x = lowest_x;
        term_m.pose.position.y = lowest_y;
        term_m.pose.position.z = lowest_z;
        term_m.pose.orientation.w = 1.0;
        term_m.scale.x = 0.26;
        term_m.scale.y = 0.26;
        term_m.scale.z = 0.26;
        const float term_alpha = (chain_ptr && chain_ptr->is_held) ? 0.50f : 0.95f;
        if (is_left) {
          term_m.color.r = 0.10f; term_m.color.g = 0.80f; term_m.color.b = 1.00f; term_m.color.a = term_alpha;
        } else {
          term_m.color.r = 0.20f; term_m.color.g = 0.95f; term_m.color.b = 0.40f; term_m.color.a = term_alpha;
        }
        term_m.lifetime = rclcpp::Duration::from_seconds(0.3);
        arr.markers.push_back(term_m);
      }

      // 4. Largest Connected Cluster Bounding Box (Distinct Coral/Orange, Yaw Aligned)
      if (lc.valid) {
        const bool duplicate_shared_box =
            (!is_left && left_lc.valid && left_lc.subcluster_id == lc.subcluster_id);
        if (!duplicate_shared_box) {
          visualization_msgs::msg::Marker box;
          box.header = header;
          box.ns = "largest_connected_box";
          box.id = side;
          box.type = visualization_msgs::msg::Marker::CUBE;
          box.action = visualization_msgs::msg::Marker::ADD;
          box.pose.position.x = lc.centroid_x;
          box.pose.position.y = lc.centroid_y;
          box.pose.position.z = lc.centroid_z;
          box.pose.orientation.x = 0.0;
          box.pose.orientation.y = 0.0;
          box.pose.orientation.z = std::sin(lc.yaw * 0.5);
          box.pose.orientation.w = std::cos(lc.yaw * 0.5);
          box.scale.x = std::max(0.08, lc.length);
          box.scale.y = std::max(0.08, lc.width);
          box.scale.z = std::max(0.08, lc.height);
          if (lc.is_held) {
            box.color.r = 1.00f; box.color.g = 0.65f; box.color.b = 0.20f; box.color.a = 0.22f;
          } else {
            box.color.r = 1.00f; box.color.g = 0.45f; box.color.b = 0.12f; box.color.a = 0.45f;
          }
          box.lifetime = rclcpp::Duration::from_seconds(0.3);
          arr.markers.push_back(box);
        }
      }

      // 5. Concise Text Overlay (Clean layout with Fixed Root coordinates, OBSERVED / HELD, Crane Move)
      {
        visualization_msgs::msg::Marker text;
        text.header = header;
        text.ns = "hook_info_text";
        text.id = side;
        text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::msg::Marker::ADD;
        text.pose.position.x = lowest_x + (is_left ? -0.40 : 0.40);
        text.pose.position.y = lowest_y;
        text.pose.position.z = lowest_z + 0.45;
        text.pose.orientation.w = 1.0;
        text.scale.z = 0.20;
        if (is_left) {
          text.color.r = 0.55f; text.color.g = 0.95f; text.color.b = 1.00f; text.color.a = 1.0f;
        } else {
          text.color.r = 0.60f; text.color.g = 1.00f; text.color.b = 0.70f; text.color.a = 1.0f;
        }
        text.lifetime = rclcpp::Duration::from_seconds(0.3);

        const std::string chain_status = (chain_ptr && chain_ptr->is_held) ? "HELD" : "OBSERVED";
        const std::string largest_status = lc.is_held ? "HELD" : "OBSERVED";

        std::ostringstream ss;
        ss << (is_left ? "LEFT HOOK\n" : "RIGHT HOOK\n")
           << "Root Center: (" << std::fixed << std::setprecision(2) << root_x << ", " << root_y << ", " << root_z << ")\n";
        if (fx_state_ptr) {
          ss << "Root Obs: " << cx128s2_bev_perception::to_string(fx_state_ptr->obs_status)
             << " (" << fx_state_ptr->point_count << " pts)\n"
             << "Crane Move: " << cx128s2_bev_perception::to_string(fx_state_ptr->motion_status)
             << " (" << std::fixed << std::setprecision(2) << fx_state_ptr->smoothed_speed << " m/s)\n";
        }
        ss << "Chain: " << chain_nodes_count << " nodes\n"
           << "Status: " << chain_status << "\n";
          if (chain_ptr) {
            ss << "Max Slope: " << std::fixed << std::setprecision(1) << chain_ptr->max_slope_angle_deg << "°\n"
               << "Anchors: " << chain_ptr->locked_anchor_count << " locked"
               << (chain_ptr->relocated_anchor_count > 0 ? " (RELOC)" : "")
               << (chain_ptr->beam_intruded_count > 0 ? " (INTRUDED)" : "") << "\n";
            int vr_cnt = 0;
            double avg_spacing = 0.0;
            for (const auto & n : chain_ptr->nodes) {
              if (n.has_virtual_rope && n.wire_bundle.mode == cx128s2_bev_perception::CraneWireBundleMode::BOTH_STRANDS) {
                vr_cnt++;
                avg_spacing += n.wire_bundle.spacing_dist;
              }
            }
            if (vr_cnt > 0) {
              ss << "Virtual Rope: 2-Strand (" << std::fixed << std::setprecision(2) << (avg_spacing / vr_cnt) << "m)\n";
            }
            if (chain_ptr->has_sheave) {
              const auto & sh = chain_ptr->sheave;
              ss << "Sheave: [" << cx128s2_bev_perception::to_string(sh.status) << "] "
                 << (sh.axis_uncertain ? "[UNCERTAIN] " : "")
                 << "R=" << std::fixed << std::setprecision(2) << sh.radius << "m "
                 << "RMS=" << std::setprecision(0) << (sh.circle_fit_rms * 1000.0) << "mm "
                 << "Arc=" << std::setprecision(0) << sh.arc_angle_deg << "°\n"
                 << "  Axis Center: (" << std::setprecision(2) << sh.axis_center_x << ", "
                 << sh.axis_center_y << ", " << sh.axis_center_z << ")\n"
                 << "  Outer Center: (" << sh.outer_center_x << ", "
                 << sh.outer_center_y << ", " << sh.outer_center_z << ")\n";
            }
          }
        ss << "\nLARGEST CONNECTED\n";
        if (lc.valid) {
          ss << "Layer: " << lc.layer_id << "\n"
             << "Length: " << std::fixed << std::setprecision(2) << lc.length << "m\n"
             << "Width: " << std::fixed << std::setprecision(2) << lc.width << "m\n"
             << "Area: " << std::fixed << std::setprecision(2) << lc.area_xy << "m²\n"
             << "Status: " << largest_status;
        } else {
          ss << "NONE";
        }
        if (chain_ptr && chain_ptr->exception_gate.status != cx128s2_bev_perception::CraneExceptionGateStatus::NONE) {
          const auto & eg = chain_ptr->exception_gate;
          ss << "\n\nException Gate: " << cx128s2_bev_perception::to_string(eg.status) << "\n"
             << "Reason: " << cx128s2_bev_perception::to_string(eg.reason) << "\n"
             << "Visibility: " << cx128s2_bev_perception::to_string(eg.visibility_status);
        }
        text.text = ss.str();
        arr.markers.push_back(text);
      }
    }

    if(work_res && (work_res->fsm_enabled || work_res->boundary_motion_enabled)){
      const auto & w=*work_res;
      auto base=[&](const std::string & ns,int id,int type){visualization_msgs::msg::Marker m;
        m.header=header;m.ns=ns;m.id=id;m.type=type;m.action=visualization_msgs::msg::Marker::ADD;
        m.pose.orientation.w=1;m.color.a=.7;m.lifetime=rclcpp::Duration::from_seconds(.3);return m;};
      auto box=[&](const std::string & ns,int id,const auto & r,float red,float green,float blue){
        if(r.sizeX()<=0 || r.sizeY()<=0 || r.sizeZ()<=0)return;
        auto m=base(ns,id,visualization_msgs::msg::Marker::CUBE);
        m.pose.position.x=r.centerX();m.pose.position.y=r.centerY();m.pose.position.z=r.centerZ();
        m.scale.x=r.sizeX();m.scale.y=r.sizeY();m.scale.z=r.sizeZ();m.color.r=red;m.color.g=green;m.color.b=blue;m.color.a=.08;
        arr.markers.push_back(m);};
      if(w.boundary_motion_enabled)for(int i=0;i<8;++i){const auto & p=w.boundary_points[i];
        if(p.observation=="NONE")continue;
        auto m=base("crane_move_raw_patches",i,visualization_msgs::msg::Marker::SPHERE);
        m.pose.position.x=p.x;m.pose.position.y=p.y;m.pose.position.z=p.z;m.scale.x=m.scale.y=m.scale.z=.08;
        m.color.r=p.observation=="DIRECT"?.1:1;m.color.g=.8;m.color.b=.2;arr.markers.push_back(m);
        auto text=base("crane_move_patch_labels",i,visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
        text.pose=m.pose;text.pose.position.z+=.1;text.scale.z=.08;text.color.r=text.color.g=text.color.b=1;
        text.text=std::to_string(i)+" "+p.observation;arr.markers.push_back(text);
      }
      if(w.fsm_enabled){
        for(int side=0;side<2;++side){const auto & h=side==0?w.left_hook:w.right_hook;
          if(!h.hook_detected)continue;
          auto m=base("fsm_hook_observation",side,visualization_msgs::msg::Marker::SPHERE);
          m.pose.position.x=h.hook_x;m.pose.position.y=h.hook_y;m.pose.position.z=h.hook_z;
          m.scale.x=m.scale.y=m.scale.z=.07;m.color.g=1;m.color.b=1;arr.markers.push_back(m);
          box("cargo_detection_roi",side,w.cargo_detection_rois[side],0,1,1);
        }
        box("cargo_tracking_roi",0,w.cargo_tracking_roi,0,1,0);
        if(w.geometry_locked)box("cargo_frozen_completion_boundary",0,w.completion_boundary,1,0,1);
        if(w.cargo_track_id>=0){
          auto m=base("fsm_cargo",0,visualization_msgs::msg::Marker::CUBE);
          m.pose.position.x=w.cargo_x;m.pose.position.y=w.cargo_y;m.pose.position.z=w.cargo_z;
          m.pose.orientation.z=std::sin(w.yaw*.5);m.pose.orientation.w=std::cos(w.yaw*.5);
          m.scale.x=w.length;m.scale.y=w.width;m.scale.z=w.height;
          m.color.r=w.geometry_locked?0:1;m.color.g=1;m.color.b=.2;m.color.a=.35;arr.markers.push_back(m);
        }
      }
      if(w.extension_valid){
        auto m=base("cargo_extended_geometry",0,visualization_msgs::msg::Marker::CUBE);
        m.pose.position.x=w.extended_x;m.pose.position.y=w.extended_y;m.pose.position.z=w.extended_z;
        m.pose.orientation.z=std::sin(w.extended_yaw*.5);m.pose.orientation.w=std::cos(w.extended_yaw*.5);
        m.scale.x=w.extended_length;m.scale.y=w.extended_width;m.scale.z=std::max(.01,w.extended_height);
        m.color.r=.2;m.color.g=.8;m.color.b=1;m.color.a=.08;arr.markers.push_back(m);
        auto segments=[&](const std::string & ns,const auto & spans,bool inferred){
          auto line=base(ns,0,visualization_msgs::msg::Marker::LINE_LIST);line.scale.x=.025;
          line.color.r=inferred?1:.1;line.color.g=inferred?.5:1;line.color.b=0;
          for(const auto & span:spans){
            const double step=crane_pipeline_->config().work_status.grid_resolution_m;
            for(double lo=span[0];lo<span[1];lo+=inferred?2*step:span[1]-span[0]){
              geometry_msgs::msg::Point a,b;a.x=w.cargo_x+lo*std::cos(w.yaw);a.y=w.cargo_y+lo*std::sin(w.yaw);a.z=w.extended_z;
              const double hi=inferred?std::min(span[1],lo+step):span[1];
              b.x=w.cargo_x+hi*std::cos(w.yaw);b.y=w.cargo_y+hi*std::sin(w.yaw);b.z=w.extended_z;
              line.points.push_back(a);line.points.push_back(b);
            }
          }arr.markers.push_back(line);
        };
        segments("cargo_observed_spans",w.observed_intervals,false);segments("cargo_inferred_gaps",w.inferred_gaps,true);
        const auto & wc=crane_pipeline_->config().work_status;
        for(int side=0;side<3;++side){auto roi=base("cargo_oriented_gap_roi",side,visualization_msgs::msg::Marker::CUBE);
          const double v=side==0?w.extended_width*.5+wc.grid_resolution_m+wc.cargo_min_surface_width_m*.5:
            (side==1?-w.extended_width*.5-wc.grid_resolution_m-wc.cargo_min_surface_width_m*.5:0);
          roi.pose.position.x=w.extended_x-v*std::sin(w.extended_yaw);roi.pose.position.y=w.extended_y+v*std::cos(w.extended_yaw);
          roi.pose.position.z=(w.gap_roi_z_min+w.gap_roi_z_max)*.5;
          roi.pose.orientation.z=std::sin(w.extended_yaw*.5);roi.pose.orientation.w=std::cos(w.extended_yaw*.5);
          roi.scale.x=2*std::max(wc.cargo_min_surface_width_m,w.extended_length*.25);
          roi.scale.y=side==2?w.extended_width:wc.cargo_min_surface_width_m;roi.scale.z=w.gap_roi_z_max-w.gap_roi_z_min;
          roi.color.r=w.gap_sides[side]=="UNKNOWN"?.6:0;roi.color.g=w.gap_sides[side]=="SUPPORTED"?1:.4;roi.color.b=w.gap_sides[side]=="GAP_OBSERVED"?1:.4;roi.color.a=.07;arr.markers.push_back(roi);
        }
      }
      auto text=base("crane_work_fsm",0,visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
      text.pose.position.z=6.5;text.scale.z=.13;text.color.r=text.color.g=text.color.b=1;
      text.text=std::string("FSM ")+cx128s2_bev_perception::to_string(w.fsm_state)+" work_status="+cx128s2_bev_perception::workingStatusName(w.working_status)+
        "\nCargo #"+std::to_string(w.cargo_track_id)+" "+w.cargo_side+" "+w.cargo_observation+(w.geometry_locked?" LOCKED":"")+
        "\nSheave "+w.sheave_visibility[0]+" / "+w.sheave_visibility[1]+" (Axis UNKNOWN)"+
        "\nExtension "+w.extension_observation+" L="+std::to_string(w.extended_length)+" gap="+w.gap_diagnostic+"\nCrane XY move="+(w.boundary_moving?"true":"false")+" "+w.boundary_reason;
      arr.markers.push_back(text);
    }

    // Presentation-only view on the existing topic. Original Chain/Anchor/Housing
    // state and all debug markers above remain untouched.
    if(top_down_res){
      auto marker=[&](int id,int type){visualization_msgs::msg::Marker m;
        m.header=header;m.ns=(id==0 || id==1)?"crane_housing_view":((id==10 || id==11 || id==50)?"crane_hook_ready_view":((id==20 || id==21)?"crane_sling_view":"crane_work_view"));m.id=id;m.type=type;
        m.action=visualization_msgs::msg::Marker::ADD;m.pose.orientation.w=1;
        m.color.a=.95;m.lifetime=rclcpp::Duration::from_seconds(.3);return m;};
      const bool enabled=work_res && work_res->fsm_enabled;
      std::ostringstream summary;summary<<"HOOK / READY";
      if(work_res){summary<<"\nBeam-Ground: ";if(work_res->beam_ground_clearance_valid)summary<<work_res->beam_ground_clearance_m<<" m";else summary<<"UNKNOWN";}
      if(!enabled)summary<<"  [FSM OFF - NOT EVALUATED]";
      for(int side=0;side<2;++side){
        const auto & housing=side==0?top_down_res->left_housing:top_down_res->right_housing;
        const auto & sh=housing.sheave;
        const bool observed=sh.status==cx128s2_bev_perception::CraneSheaveDetectionStatus::DIRECT_OBSERVED;
        const bool held=sh.status==cx128s2_bev_perception::CraneSheaveDetectionStatus::HELD;
        if(observed || held){
          auto m=marker(side,visualization_msgs::msg::Marker::SPHERE);
          m.pose.position.x=housing.x;
          m.pose.position.y=housing.y;
          m.pose.position.z=housing.z;
          m.scale.x=m.scale.y=m.scale.z=.12;m.color.r=held?.6:.1;m.color.g=held?.6:1;m.color.b=held?.6:.8;
          arr.markers.push_back(m);
        }
        // Interpret only existing, thin, currently supported Chain segments below
        // a directly observed Housing. No new tracking and no old point indices.
        bool sling=false;const int ci=side==0?top_down_res->left_chain_idx:top_down_res->right_chain_idx;
        auto line=marker(20+side,visualization_msgs::msg::Marker::LINE_LIST);
        line.scale.x=.025;line.color.r=1;line.color.g=.65;line.color.b=.1;
        if(observed && ci>=0 && size_t(ci)<top_down_res->chains.size()){
          const auto & nodes=top_down_res->chains[ci].nodes;
          for(size_t ni=1;ni<nodes.size();++ni){const auto & a=nodes[ni-1];const auto & b=nodes[ni];
            if(a.is_held || b.is_held || b.is_sheave || b.z>=sh.min_z-.05 ||
                b.max_x-b.min_x>.15 || b.max_y-b.min_y>.15)continue;
            geometry_msgs::msg::Point first,last;first.x=a.x;first.y=a.y;first.z=a.z;
            last.x=b.x;last.y=b.y;last.z=b.z;
            if(first.z>sh.min_z){
              if(last.z>=first.z)continue;
              const double t=(first.z-sh.min_z)/(first.z-last.z);
              first.x+=t*(last.x-first.x);first.y+=t*(last.y-first.y);first.z=sh.min_z;
            }
            const double dx=last.x-first.x,dy=last.y-first.y,dz=last.z-first.z;
            const double norm=dx*dx+dy*dy+dz*dz;if(norm<.01)continue;
            std::array<int,3> support{};
            for(const auto & p:frame_points_){
              const double t=((p.x-first.x)*dx+(p.y-first.y)*dy+(p.z-first.z)*dz)/norm;
              if(!std::isfinite(t) || t<0 || t>=1)continue;
              const double ex=p.x-first.x-t*dx,ey=p.y-first.y-t*dy,ez=p.z-first.z-t*dz;
              if(ex*ex+ey*ey+ez*ez<=.06*.06)++support[std::min(2,int(t*3))];
            }
            if(support[0]<3 || support[1]<3 || support[2]<3)continue;
            line.points.push_back(first);line.points.push_back(last);sling=true;
          }
        }
        if(sling)arr.markers.push_back(line);
        const auto * hook=work_res?(side==0?&work_res->left_hook:&work_res->right_hook):nullptr;
        if(hook && hook->hook_detected){
          auto m=marker(10+side,visualization_msgs::msg::Marker::SPHERE);
          m.pose.position.x=hook->hook_x;m.pose.position.y=hook->hook_y;m.pose.position.z=hook->hook_z;
          m.scale.x=m.scale.y=m.scale.z=.08;m.color.r=hook->hook_observation=="VIRTUAL_HELD"?.6:0;m.color.g=m.color.b=1;arr.markers.push_back(m);
          const auto & roi=work_res->cargo_detection_rois[side];
          auto box=marker(30+side,visualization_msgs::msg::Marker::CUBE);
          box.pose.position.x=roi.centerX();box.pose.position.y=roi.centerY();box.pose.position.z=roi.centerZ();
          box.scale.x=roi.sizeX();box.scale.y=roi.sizeY();box.scale.z=roi.sizeZ();
          box.color.g=box.color.b=1;box.color.a=.06;
          if(box.scale.x>0 && box.scale.y>0 && box.scale.z>0)arr.markers.push_back(box);
        }
        summary<<"\n\n"<<(side==0?"LEFT":"RIGHT")<<"\n Hook : ";
        if(hook && hook->hook_detected)
          summary<<std::fixed<<std::setprecision(2)<<"("<<hook->hook_x<<", "<<hook->hook_y<<", "<<hook->hook_z<<") "<<hook->hook_observation;
        else summary<<(enabled?"NOT DETECTED":"UNKNOWN (FSM OFF)");
        summary<<"\n Source : VIRTUAL_SHEAVE (-"<<crane_pipeline_->config().work_status.virtual_hook_offset_z_m<<"m Z)"
          <<"\n Housing : "<<(observed?"DIRECT":(held?"HELD":"NONE"))
          <<"\n Ready : "<<(enabled && hook?(hook->work_status?"YES":"NO"):"UNKNOWN (FSM OFF)");
      }
      if(enabled && work_res->cargo_track_id>=0){
        const auto & w=*work_res;auto m=marker(40,visualization_msgs::msg::Marker::CUBE);
        m.pose.position.x=w.cargo_x;m.pose.position.y=w.cargo_y;m.pose.position.z=w.cargo_z;
        m.pose.orientation.z=std::sin(w.yaw*.5);m.pose.orientation.w=std::cos(w.yaw*.5);
        m.scale.x=w.length;m.scale.y=w.width;m.scale.z=w.height;
        m.color.r=w.cargo_observation=="HELD"?.6:1;m.color.g=1;m.color.b=.2;m.color.a=.35;arr.markers.push_back(m);
      }
      summary<<"\n\nGLOBAL work_status : "<<(enabled?cx128s2_bev_perception::workingStatusName(work_res->public_working_status==3?3:work_res->working_status):"UNKNOWN (FSM OFF)");
      auto text=marker(50,visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
      text.pose.position.x=-2.5;text.pose.position.y=1.3;text.pose.position.z=4.5;text.scale.z=.14;
      text.color.r=text.color.g=text.color.b=1;text.text=summary.str();arr.markers.push_back(text);
    }

    pub_crane_hook_markers_->publish(arr);
  }


  // Both callers hold crane_command_mutex_: command replies and sensor frames
  // use the same final projection, without writing into detector/FSM state.
  cx128s2_bev_perception::msg::CraneBeamStatus publishCraneStatus(
      const std::string & command = "", bool success = true, const std::string & error = "") {
    auto status = crane_auto_status_.value_or(cx128s2_bev_perception::msg::CraneBeamStatus{});
    if (!crane_auto_status_) { status.state = "UNKNOWN"; status.working_status = "idle"; }
    if(crane_auto_status_ && crane_pipeline_){
      const auto & work=crane_pipeline_->result().work_status;
      if(work.fsm_enabled){
        status.state=cx128s2_bev_perception::to_string(work.fsm_state);
        status.working_status=cx128s2_bev_perception::workingStatusName(work.public_working_status==3?3:work.working_status);
        status.beam_ground_clearance_valid=work.beam_ground_clearance_valid;
        status.beam_ground_clearance_m=work.beam_ground_clearance_m;
        status.beam_bottom_z=work.beam_bottom_z;status.ground_surface_z=work.ground_surface_z;
        status.initial_clearance_valid=work.initial_clearance_valid;status.initial_clearance_m=work.initial_clearance_m;
        status.clearance_delta_m=work.clearance_delta_m;
        status.beam_ground_clearance_samples_m.assign(work.beam_ground_clearance_samples_m.begin(),work.beam_ground_clearance_samples_m.end());
        status.beam_bottom_samples_z.assign(work.beam_bottom_samples_z.begin(),work.beam_bottom_samples_z.end());
        status.ground_surface_samples_z.assign(work.ground_surface_samples_z.begin(),work.ground_surface_samples_z.end());
        status.beam_ground_clearance_sample_valid.assign(work.beam_ground_clearance_sample_valid.begin(),work.beam_ground_clearance_sample_valid.end());
        status.beam_ground_clearance_reason=work.beam_ground_clearance_reason;
        status.prepare_ready=work.geometry_locked;
        status.beam_cluster_valid=work.geometry_locked && work.cargo_observation=="DIRECT";
        status.beam_z_valid=status.beam_cluster_valid;
        status.length_locked=work.geometry_locked;
        status.beam_length_locked=work.length;status.beam_length=work.geometry_locked?work.length:0;
        status.work_complete=work.fsm_state==cx128s2_bev_perception::CraneBeamState::LOWERING_CONFIRMED;
      }
      if(work.boundary_motion_enabled)status.crane_moving=crane_pipeline_->confirmedCraneMoving();
    }
    if(status.state=="LIFTING" && status.working_status=="working" &&
        (!crane_pipeline_ || !crane_pipeline_->result().work_status.fsm_enabled))status.working_status="lifting";
    // Reuse the validated C3 projection before manual overrides.
    if(crane_auto_status_ && crane_pipeline_ && crane_pipeline_->result().work_status.fsm_enabled){
      status.state=crane_hybrid_policy_.state=="WORKING"?"IDLE":crane_hybrid_policy_.state;
      status.working_status=crane_hybrid_policy_.state=="WORKING"?"idle":crane_hybrid_policy_.working;
      status.work_complete=crane_hybrid_policy_.state=="LOWERING_CONFIRMATION";
    }
    // Public contract preserves all five FSM states; internal enum names stay unchanged.
    const auto project_public_state = [](auto & output) {
      if (output.state == "COMPLETE_CONFIRM" || output.state == "LOWERING_CONFIRMED" ||
          output.state == "LOWERING_CONFIRMATION") output.state = "LOWERING_CONFIRM";
      if (output.state != "IDLE" && output.state != "LIFTING" &&
          output.state != "SUSPENDED" && output.state != "LOWERING" &&
          output.state != "LOWERING_CONFIRM") output.state = "IDLE";
      output.working_status = output.state;
      output.work_complete = output.state == "LOWERING_CONFIRM";
    };
    project_public_state(status);
    status.auto_state = crane_auto_status_ ? status.state : "UNKNOWN";
    status.auto_working_status = status.working_status;
    status.auto_beam_detected = status.beam_z_valid && status.beam_cluster_valid;
    status.auto_prepare_ready = status.prepare_ready;
    status.auto_work_complete = status.work_complete;
    status.control_mode = crane_manual_mode_ ? "MANUAL_OVERRIDE" : "AUTO";
    status.beam_detected = status.auto_beam_detected;
    status.manual_override = "beam=" + std::string(crane_beam_override_ ?
        (*crane_beam_override_ ? "true" : "false") : "AUTO") +
        ",working=" + (crane_working_override_ ? std::to_string(*crane_working_override_) : "AUTO") +
        ",fsm=" + (crane_fsm_override_ ? std::string(cx128s2_bev_perception::to_string(*crane_fsm_override_)) : "AUTO") +
        ",ready=" + (crane_ready_override_ ? (*crane_ready_override_ ? "true" : "false") : "AUTO");
    if (crane_manual_mode_) {
      if (crane_beam_override_) status.beam_detected = *crane_beam_override_;
      if (crane_fsm_override_) {
        const auto state = *crane_fsm_override_;
        status.state = cx128s2_bev_perception::to_string(state);
        status.working_status = cx128s2_bev_perception::workingStatusName(state == cx128s2_bev_perception::CraneBeamState::LIFTING ? 3 :
            ((state == cx128s2_bev_perception::CraneBeamState::SUSPENDED ||
              state == cx128s2_bev_perception::CraneBeamState::LOWERING) ? 1 : 0));
        status.work_complete = state == cx128s2_bev_perception::CraneBeamState::COMPLETE_CONFIRM;
      }
      if (crane_working_override_) status.working_status = cx128s2_bev_perception::workingStatusName(*crane_working_override_);
      if (crane_ready_override_) status.prepare_ready = *crane_ready_override_;
    }
    if (crane_manual_mode_ && crane_working_override_) {
      switch (*crane_working_override_) {
        case 0: status.state = "IDLE"; break;
        case 1:
          if (status.state != "SUSPENDED" && status.state != "LOWERING") status.state = "SUSPENDED";
          break;
        case 2: status.state = "LOWERING_CONFIRM"; break;
        case 3: status.state = "LIFTING"; break;
      }
    }
    project_public_state(status);
    status.work_state=status.working_status;
    status.work_active=status.state=="LIFTING" || status.state=="SUSPENDED" || status.state=="LOWERING";
    status.prepare_status = status.prepare_ready ? "READY" : "NOT_READY";
    // No frame means no genuine measurement/header to publish. Command feedback
    // remains available; a cached frame retains its original sensor timestamp.
    if (crane_auto_status_) {
      if(crane_pipeline_ && crane_pipeline_name_=="slope" &&
          crane_pipeline_->config().work_status.boundary_temporal_enabled){
        std::lock_guard<std::mutex> lock(crane_motion_expiry_mutex_);
        crane_motion_expiry_timeout_=crane_pipeline_->config().work_status.boundary_support_timeout_sec;
        const double age=(get_clock()->now()-rclcpp::Time(status.header.stamp,get_clock()->get_clock_type())).seconds();
        if(age>=crane_motion_expiry_timeout_ || age<0)status.crane_moving=false;
        crane_motion_expiry_status_=status;
      }
      pub_crane_beam_status_->publish(status);
      std_msgs::msg::String working;
      working.data = status.working_status;
      pub_crane_beam_working_status_->publish(working);
    }
    if (!command.empty()) {
      std::ostringstream reply;
      reply << "success=" << std::boolalpha << success << " command=" << command
            << " mode=" << status.control_mode << " reset_pending=" << crane_reset_pending_
            << " auto_available=" << crane_auto_status_.has_value()
            << "\nAUTO result: state=" << status.auto_state
            << " working=" << status.auto_working_status
            << " beam_detected=" << status.auto_beam_detected
            << " ready=" << status.auto_prepare_ready << " complete=" << status.auto_work_complete
            << "\nMANUAL override: " << status.manual_override
            << "\neffective result: state=" << status.state
            << " working=" << status.working_status
            << " beam_detected=" << status.beam_detected
            << " ready=" << status.prepare_ready << " complete=" << status.work_complete;
      if (!error.empty()) reply << (success ? "\ndetail=" : "\nerror=") << error;
      std_msgs::msg::String feedback;
      feedback.data = reply.str();
      pub_crane_command_status_->publish(feedback);
      if (success) RCLCPP_INFO(get_logger(), "[CRANE_COMMAND] %s", feedback.data.c_str());
      else RCLCPP_WARN(get_logger(), "[CRANE_COMMAND] %s", feedback.data.c_str());
    }
    return status;
  }

  void handleCraneCommand(const std_msgs::msg::String::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(crane_command_mutex_);
    std::istringstream input(msg->data);
    std::string command, value, extra;
    input >> command >> value >> extra;
    std::transform(command.begin(), command.end(), command.begin(),
        [](unsigned char c) { return std::toupper(c); });
    std::transform(value.begin(), value.end(), value.begin(),
        [](unsigned char c) { return std::toupper(c); });
    const bool reference_command = command == "HOOK_REFERENCE_BEGIN" || command == "HOOK_REFERENCE_SAVE" ||
        command == "HOOK_REFERENCE_LOAD" || command == "HOOK_REFERENCE_STATUS" || command == "HOOK_REFERENCE_CANCEL";
    const bool boolean_command = command == "SET_BEAM_DETECTED" || command == "SET_BEAM" || command == "SET_READY";
    const bool no_value_command = command == "AUTO" || command == "CLEAR_OVERRIDE" ||
        command == "CLEAR" || command == "RESET" || command == "MANUAL_OVERRIDE" ||
        command == "SET_WORKING" || command == "SET_IDLE" || command == "GET_STATUS" || reference_command;
    std::optional<cx128s2_bev_perception::CraneBeamState> state;
    if (command == "SET_FSM") {
      if (value == "LOWERING_CONFIRM") state = cx128s2_bev_perception::CraneBeamState::COMPLETE_CONFIRM;
      for (const auto candidate : {cx128s2_bev_perception::CraneBeamState::IDLE,
          cx128s2_bev_perception::CraneBeamState::LIFTING,
          cx128s2_bev_perception::CraneBeamState::SUSPENDED,
          cx128s2_bev_perception::CraneBeamState::LOWERING,
          cx128s2_bev_perception::CraneBeamState::COMPLETE_CONFIRM}) {
        if (value == cx128s2_bev_perception::to_string(candidate)) state = candidate;
      }
    }
    std::string error;
    if (perception_mode_ != "crane_beam_work") error = "Commands require Crane Mode";
    else if (msg->data.size() > 256 || !extra.empty()) error = "Invalid command syntax";
    else if (no_value_command && !value.empty()) error = "This command takes no value";
    else if (boolean_command && value != "TRUE" && value != "FALSE") error = "Expected true or false";
    else if (command == "SET_FSM" && !state) error = "Expected existing FSM state; use SET_READY true for READY";
    else if (!boolean_command && !no_value_command && command != "SET_FSM") error = "Unknown command";
    if (!error.empty()) {
      publishCraneStatus(command + " " + value, false, error);
      return;
    }
    if (reference_command && crane_pipeline_) {
      publishCraneStatus(command, false, "Hook reference commands belong to legacy pipeline");
      return;
    }
    if (reference_command) {
      auto & detector = crane_beam_detector_->getHookDetector();
      std::string report;
      bool success = true;
      if (command == "HOOK_REFERENCE_BEGIN") {
        detector.beginReferenceLearning();
        crane_beam_detector_->reset();
        crane_auto_status_.reset();
        report = "Explicit offline collection started; " + detector.referenceSummary();
      } else if (command == "HOOK_REFERENCE_CANCEL") {
        detector.cancelReferenceLearning();
        crane_beam_detector_->reset();
        crane_auto_status_.reset();
        report = "Offline collection cancelled; persisted reference unchanged; " + detector.referenceSummary();
      } else if (command == "HOOK_REFERENCE_SAVE") {
        RCLCPP_INFO(get_logger(), "[HOOK_REFERENCE] Before save: %s", detector.referenceSummary().c_str());
        success = detector.saveReference(crane_hook_reference_file_, target_frame_, report);
      } else if (command == "HOOK_REFERENCE_LOAD") {
        success = detector.loadReference(crane_hook_reference_file_, target_frame_, report);
        if (success) { crane_beam_detector_->reset(); crane_auto_status_.reset(); }
      } else report = detector.referenceSummary() + " file=" + crane_hook_reference_file_;
      publishCraneStatus(command, success, report);
      return;
    }
    const bool was_manual = crane_manual_mode_;
    if (command == "AUTO" || command == "CLEAR_OVERRIDE" || command == "CLEAR" || command == "RESET") {
      crane_manual_mode_ = false;
      crane_beam_override_.reset();
      crane_working_override_.reset();
      crane_fsm_override_.reset();
      crane_ready_override_.reset();
      if (command == "RESET") {
        crane_reset_pending_ = true;
        crane_auto_status_.reset();
      }
    } else if (command != "GET_STATUS") {
      crane_manual_mode_ = true;
      if (command == "SET_BEAM_DETECTED" || command == "SET_BEAM") crane_beam_override_ = value == "TRUE";
      else if (command == "SET_READY") crane_ready_override_ = value == "TRUE";
      else if (command == "SET_FSM") crane_fsm_override_ = state;
      else if (command == "SET_WORKING" || command == "SET_IDLE") crane_working_override_ = command == "SET_WORKING";
    }
    RCLCPP_INFO(get_logger(), "[CRANE_COMMAND] mode=%s -> %s command=%s value=%s",
        was_manual ? "MANUAL_OVERRIDE" : "AUTO", crane_manual_mode_ ? "MANUAL_OVERRIDE" : "AUTO",
        command.c_str(), value.c_str());
    publishCraneStatus(command + (value.empty() ? "" : " " + value));
  }

  // Scoped to the opt-in path; legacy/General parsing behavior is unchanged.
  static bool validateCraneSlopeCloud(const sensor_msgs::msg::PointCloud2 & msg) {
    if (msg.is_bigendian || msg.point_step == 0 ||
        static_cast<uint64_t>(msg.row_step) != static_cast<uint64_t>(msg.width)*msg.point_step ||
        msg.data.size() < static_cast<uint64_t>(msg.row_step)*msg.height) return false;
    bool x = false, y = false, z = false;
    std::set<std::string> names;
    for (const auto & f : msg.fields) {
      const bool xyz = f.name == "x" || f.name == "y" || f.name == "z";
      const bool used = xyz || f.name == "intensity" || f.name == "time" || f.name == "ring";
      if (!used) continue;
      const auto datatype = f.name == "ring" ? sensor_msgs::msg::PointField::UINT16 :
          sensor_msgs::msg::PointField::FLOAT32;
      const uint64_t bytes = f.name == "ring" ? 2 : 4;
      if (!names.insert(f.name).second || f.count != 1 || f.datatype != datatype ||
          static_cast<uint64_t>(f.offset) + bytes > msg.point_step) return false;
      x |= f.name == "x"; y |= f.name == "y"; z |= f.name == "z";
    }
    return x && y && z;
  }

  void publishCraneFoundationStatus(const std_msgs::msg::Header & header,
      cx128s2_bev_perception::CraneBeamState state) {
    cx128s2_bev_perception::msg::CraneBeamStatus status;
    status.header = header;
    status.state = cx128s2_bev_perception::to_string(state);
    status.motion_reference_mode = "absolute";
    status.beam_tracking_state = "UNIMPLEMENTED";
    status.left_hook_tracking_state = status.right_hook_tracking_state = "UNIMPLEMENTED";
    // State names/interface are retained. No fabricated detection or automatic transitions.
    crane_auto_status_ = status;
    publishCraneStatus();
  }

  template<class ColorAt>
  void publishCraneColorCloud(const std_msgs::msg::Header & header,const std::vector<PointXYZIRT> & points,
      const std::vector<int> & indices,const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher,
      ColorAt color_at,const std::vector<int> * ids=nullptr,const char * id_name="surface_id") {
    if (!publisher || !publisher->get_subscription_count() || (ids && ids->size()!=points.size())) return;
    sensor_msgs::msg::PointCloud2 cloud;cloud.header=header;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    if(ids)modifier.setPointCloud2Fields(5,"x",1,sensor_msgs::msg::PointField::FLOAT32,
        "y",1,sensor_msgs::msg::PointField::FLOAT32,"z",1,sensor_msgs::msg::PointField::FLOAT32,
        "rgb",1,sensor_msgs::msg::PointField::FLOAT32,id_name,1,sensor_msgs::msg::PointField::INT32);
    else modifier.setPointCloud2Fields(4,"x",1,sensor_msgs::msg::PointField::FLOAT32,
        "y",1,sensor_msgs::msg::PointField::FLOAT32,"z",1,sensor_msgs::msg::PointField::FLOAT32,
        "rgb",1,sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(indices.size());cloud.is_dense=true;
    sensor_msgs::PointCloud2Iterator<float> x(cloud,"x"),y(cloud,"y"),z(cloud,"z"),rgb(cloud,"rgb");
    size_t cursor=0;
    for(int idx:indices) {
      const uint32_t color=color_at(idx);float packed;std::memcpy(&packed,&color,4);
      *x=points[idx].x;*y=points[idx].y;*z=points[idx].z;*rgb=packed;
      if(ids) {const int32_t id=(*ids)[idx];std::memcpy(cloud.data.data()+cursor*cloud.point_step+16,&id,4);}
      ++cursor;++x;++y;++z;++rgb;
    }
    publisher->publish(std::move(cloud));
  }

  void processCraneHeight(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg,
      const std_msgs::msg::Header & header,const std::vector<PointXYZIRT> & points,
      const std::vector<int> & indices,const tf2::Transform & pose,bool debug) {
    std::lock_guard<std::mutex> lock(crane_command_mutex_);
    if(crane_reset_pending_) {resetPerceptionRuntimeState();crane_reset_pending_=false;}
    if(crane_height_enabled_ && !visibility_frame_valid_) {
      RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),2000,"Height surface input frame must match visibility.sensor_frame");
      return;
    }
    const auto & result=crane_pipeline_->process(points,indices,pose,last_input_has_ring_,&visibility_model_);
    const auto & h=result.height;publishCraneFoundationStatus(header,result.state);
    if(!debug)return;
    auto world_header=msg->header;world_header.frame_id="world";
    publishCraneColorCloud(world_header,points,h.candidates,pub_crane_height_candidates_,[&](int i) {
      return cx128s2_bev_perception::CranePipeline::heightColor(h.patch_ids[i]);
    },&h.patch_ids,"patch_id");
    publishCraneColorCloud(world_header,points,h.surface_points,pub_crane_height_surfaces_,[&](int i) {
      return cx128s2_bev_perception::CranePipeline::heightColor(h.surface_ids[i]);
    },&h.surface_ids);
    publishCloud(world_header,points,h.unassigned,pub_crane_height_unassigned_);
    if(pub_crane_height_regions_ && pub_crane_height_regions_->get_subscription_count()) {
      std_msgs::msg::String message;std::ostringstream json;
      json << "{\"frame_id\":\"world\",\"stamp\":{\"sec\":" << world_header.stamp.sec
           << ",\"nanosec\":" << world_header.stamp.nanosec << "},\"surfaces\":[";
      for(size_t i=0;i<h.surfaces.size();++i) {
        const auto & a=h.surfaces[i];if(i)json << ',';
        json << "{\"id\":" << a.id << ",\"points\":" << a.point_count
             << ",\"center_z\":" << a.center_z
             << ",\"min_z\":" << a.min_z << ",\"max_z\":" << a.max_z
             << ",\"z_spread\":" << a.max_z-a.min_z
             << ",\"center_x\":" << a.center_x << ",\"center_y\":" << a.center_y
             << ",\"length\":" << a.length << ",\"width\":" << a.width
             << ",\"plane_rms\":" << a.plane_rms << '}';
      }
      json << "]}";message.data=json.str();pub_crane_height_regions_->publish(message);
    }
    if(pub_crane_beam_diagnostics_) {
      std_msgs::msg::String message;std::ostringstream text;
      text << "pipeline=height_surface fsm=IDLE transitions=unimplemented processed=" << indices.size()
           << " candidates=" << h.candidates.size() << " surfaces=" << h.surfaces.size()
           << " surface_points=" << h.surface_points.size() << " unassigned=" << h.unassigned.size()
           << " excluded=" << h.excluded.size() << " patch_count=" << h.patch_count
           << " spatial_cells=" << h.spatial_cells << " neighbor_tests=" << h.neighbor_tests
           << " occlusion_flagged=" << h.occlusion_flagged_points
           << " visibility_ms=" << h.visibility_ms << " patch_ms=" << h.patch_ms
           << " fitting_ms=" << h.fitting_ms << " merging_ms=" << h.merging_ms
           << " finalization_ms=" << h.finalization_ms
           << " height_total_ms=" << h.total_ms << " workspace_bytes=" << result.workspace_bytes;
      message.data=text.str();pub_crane_beam_diagnostics_->publish(message);
    }
  }

  void processCraneSlope(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg,
      const std_msgs::msg::Header & header, const std::vector<PointXYZIRT> & points,
      const std::vector<int> & indices, const tf2::Transform & pose, bool debug) {
    std::lock_guard<std::mutex> lock(crane_command_mutex_);
    if (crane_reset_pending_) {
      resetPerceptionRuntimeState(); crane_reset_pending_ = false;
    }
    const double stamp_sec = stampSeconds(header);
    crane_pipeline_->setSheaveMotionFrame(header.frame_id);
    const auto t_pipe_start = std::chrono::steady_clock::now();
    cx128s2_bev_perception::CraneBeamDetectionResult beam_res;
    bool has_beam_res = false;
    const bool need_legacy_beam_res =
        crane_closed_loop_grouping_enabled_ ||
        (pub_crane_beam_roi_points_ && pub_crane_beam_roi_points_->get_subscription_count() > 0) ||
        (pub_crane_beam_bg_roi_points_ && pub_crane_beam_bg_roi_points_->get_subscription_count() > 0);
    if (crane_beam_detector_ && need_legacy_beam_res && crane_pipeline_->beamWorkProcessingAllowed()) {
      beam_res = crane_beam_detector_->process(points, indices, stamp_sec);
      has_beam_res = true;
    }

    const auto effective_state = crane_fsm_override_.has_value() ?
        *crane_fsm_override_ :
        cx128s2_bev_perception::CraneBeamState::IDLE;

    auto pipe_cfg = crane_pipeline_->config();
    pipe_cfg.roi_filter.enabled = crane_slope_roi_filter_enabled_;
    pipe_cfg.roi_filter.only_crane_roi = crane_slope_roi_only_crane_roi_;
    pipe_cfg.roi_filter.active_include_background = crane_slope_roi_active_include_background_;
    pipe_cfg.roi_filter.idle_exclude_beam_roi = crane_slope_roi_idle_exclude_beam_roi_;
    pipe_cfg.roi_filter.expand_beam_roi = crane_slope_roi_expand_beam_roi_;
    pipe_cfg.roi_filter.current_state = effective_state;
    pipe_cfg.roi_filter.beam_roi_x_min = crane_beam_config_.beam_roi_x_min;
    pipe_cfg.roi_filter.beam_roi_x_max = crane_beam_config_.beam_roi_x_max;
    pipe_cfg.roi_filter.beam_roi_y_min = crane_beam_config_.beam_roi_y_min;
    pipe_cfg.roi_filter.beam_roi_y_max = crane_beam_config_.beam_roi_y_max;
    pipe_cfg.roi_filter.beam_roi_z_min = crane_beam_config_.beam_roi_z_min;
    pipe_cfg.roi_filter.beam_roi_z_max = crane_beam_config_.beam_roi_z_max;
    pipe_cfg.roi_filter.bg_roi_x_min = crane_beam_config_.bg_roi_x_min;
    pipe_cfg.roi_filter.bg_roi_x_max = crane_beam_config_.bg_roi_x_max;
    pipe_cfg.roi_filter.bg_roi_y_min = crane_beam_config_.bg_roi_y_min;
    pipe_cfg.roi_filter.bg_roi_y_max = crane_beam_config_.bg_roi_y_max;
    pipe_cfg.roi_filter.bg_roi_z_min = crane_beam_config_.bg_roi_z_min;
    pipe_cfg.roi_filter.bg_roi_z_max = crane_beam_config_.bg_roi_z_max;
    pipe_cfg.top_down_hook.roi_left_x_min = crane_beam_config_.hook.roi_left_x_min;
    pipe_cfg.top_down_hook.roi_left_x_max = crane_beam_config_.hook.roi_left_x_max;
    pipe_cfg.top_down_hook.roi_right_x_min = crane_beam_config_.hook.roi_right_x_min;
    pipe_cfg.top_down_hook.roi_right_x_max = crane_beam_config_.hook.roi_right_x_max;
    pipe_cfg.top_down_hook.roi_y_min = crane_beam_config_.hook.roi_y_min;
    pipe_cfg.top_down_hook.roi_y_max = crane_beam_config_.hook.roi_y_max;
    const bool need_boundary_clustering =
        !crane_z_gaussian_clustering_enabled_ ||
        (pub_crane_slope_boundary_ && pub_crane_slope_boundary_->get_subscription_count() > 0) ||
        (pub_crane_slope_boundary_clusters_ && pub_crane_slope_boundary_clusters_->get_subscription_count() > 0) ||
        (pub_crane_slope_boundary_diagnostics_ && pub_crane_slope_boundary_diagnostics_->get_subscription_count() > 0);
    if (!need_boundary_clustering) {
      pipe_cfg.boundary_clustering.enabled = false;
    }
    crane_pipeline_->updateConfig(pipe_cfg);

    // Mathematical Pre-filtering: P_s = { p_i in P | p_i in R_s }
    // Drops points outside the active ROI before slope segmentation, boundary clustering, and grouping
    std::vector<int> active_indices;
    if (pipe_cfg.roi_filter.enabled) {
      active_indices.reserve(indices.size());
      for (int idx : indices) {
        if (idx < 0 || static_cast<size_t>(idx) >= points.size()) continue;
        const auto & p = points[idx];
        if (!pipe_cfg.roi_filter.inActiveRoi(p.x, p.y, p.z)) continue;
        active_indices.push_back(idx);
      }
    }
    const auto & input_indices = pipe_cfg.roi_filter.enabled ? active_indices : indices;
    const auto & res = crane_pipeline_->process(points, input_indices, pose, last_input_has_ring_, &visibility_model_);
    const auto & active_points = crane_pipeline_->hasAccumulatedPoints() ? crane_pipeline_->accumulatedPoints() : points;
    const auto begin = std::chrono::steady_clock::now();
    // Existing configurable clusterer; when Z-Gaussian grouping is enabled, run per Z-layer.
    std::vector<ClusterResult> clusters;
    std::vector<int> cluster_layer_ids;
    if (crane_z_gaussian_clustering_enabled_) {
      for (const auto & layer : res.z_gaussian.layers) {
        auto layer_clusters = clusterer_->cluster(
            active_points, layer.point_indices, processed_frame_count_, stampSeconds(header));
        for (auto & c : layer_clusters) {
          cluster_layer_ids.push_back(layer.id);
          clusters.push_back(std::move(c));
        }
      }
    } else {
      clusters = clusterer_->cluster(active_points, res.clustering, processed_frame_count_, stampSeconds(header));
      cluster_layer_ids.assign(clusters.size(), 0);
    }

    // Stabilize cluster IDs across frames so shaking points do not change cluster RGB colors
    {
      std::vector<double> cluster_cz(clusters.size(), 0.0);
      for (size_t ci = 0; ci < clusters.size(); ++ci) {
        double sum_z = 0.0;
        int cnt = 0;
        for (int idx : clusters[ci].indices) {
          if (idx >= 0 && static_cast<size_t>(idx) < active_points.size()) {
            sum_z += active_points[idx].z;
            ++cnt;
          }
        }
        cluster_cz[ci] = cnt > 0 ? (sum_z / cnt) : 0.0;
      }

      const size_t prev_track_count = tracked_slope_clusters_.size();
      std::vector<uint8_t> track_used(prev_track_count, 0);
      for (size_t ci = 0; ci < clusters.size(); ++ci) {
        auto & cl = clusters[ci];
        const double cz = cluster_cz[ci];
        int best_t = -1;
        double best_dist = std::numeric_limits<double>::infinity();
        for (size_t t = 0; t < prev_track_count; ++t) {
          if (track_used[t]) continue;
          const auto & tr = tracked_slope_clusters_[t];
          const double dxy = std::hypot(cl.centroid_x - tr.cx, cl.centroid_y - tr.cy);
          const double dz = std::abs(cz - tr.cz);
          if (dxy <= 0.85 && dz <= 0.45) {
            const double d3 = std::hypot(dxy, dz);
            if (d3 < best_dist) {
              best_dist = d3;
              best_t = static_cast<int>(t);
            }
          }
        }
        if (best_t >= 0) {
          track_used[best_t] = 1;
          auto & tr = tracked_slope_clusters_[best_t];
          cl.id = tr.stable_id;
          tr.cx = 0.7 * tr.cx + 0.3 * cl.centroid_x;
          tr.cy = 0.7 * tr.cy + 0.3 * cl.centroid_y;
          tr.cz = 0.7 * tr.cz + 0.3 * cz;
          tr.missed_frames = 0;
        } else {
          cl.id = next_slope_cluster_id_++;
          tracked_slope_clusters_.push_back(
              TrackedSlopeCluster{cl.id, cl.centroid_x, cl.centroid_y, cz, 0});
        }
      }
      for (size_t t = prev_track_count; t > 0; --t) {
        const size_t idx = t - 1;
        if (!track_used[idx]) {
          if (++tracked_slope_clusters_[idx].missed_frames > 25) {
            tracked_slope_clusters_.erase(tracked_slope_clusters_.begin() + idx);
          }
        }
      }
    }

    // BEV Subclusters preparation
    std::vector<cx128s2_bev_perception::CraneSubcluster> raw_subclusters;
    raw_subclusters.reserve(clusters.size());
    for (size_t ci = 0; ci < clusters.size(); ++ci) {
      cx128s2_bev_perception::CraneSubcluster sub;
      sub.subcluster_id = clusters[ci].id;
      sub.layer_id = ci < cluster_layer_ids.size() ? cluster_layer_ids[ci] : 0;
      sub.point_indices = clusters[ci].indices;
      sub.centroid_x = clusters[ci].centroid_x;
      sub.centroid_y = clusters[ci].centroid_y;
      sub.min_x = clusters[ci].min_x;
      sub.max_x = clusters[ci].max_x;
      sub.min_y = clusters[ci].min_y;
      sub.max_y = clusters[ci].max_y;
      double min_z = std::numeric_limits<double>::infinity();
      double max_z = -min_z;
      double sum_z = 0.0;
      size_t cnt_z = 0;
      for (int idx : sub.point_indices) {
        if (idx >= 0 && static_cast<size_t>(idx) < active_points.size()) {
          const double z = active_points[idx].z;
          min_z = std::min(min_z, z);
          max_z = std::max(max_z, z);
          sum_z += z;
          ++cnt_z;
        }
      }
      sub.min_z = cnt_z > 0 ? min_z : 0.0;
      sub.max_z = cnt_z > 0 ? max_z : 0.0;
      sub.centroid_z = cnt_z > 0 ? (sum_z / cnt_z) : 0.0;
      raw_subclusters.push_back(std::move(sub));
    }

    if (crane_closed_loop_grouping_enabled_) {
      double ext_bg_z = std::numeric_limits<double>::quiet_NaN();
      double ext_dz_bg = std::numeric_limits<double>::quiet_NaN();
      if (has_beam_res && beam_res.bg_z_valid) {
        ext_bg_z = beam_res.z_bg_filtered;
        ext_dz_bg = beam_res.dz_bg_filtered;
      }
      crane_pipeline_->processClosedLoopGrouping(active_points, raw_subclusters, ext_bg_z, ext_dz_bg);
    }
    const auto & cl_subs = crane_pipeline_->result().closed_loop.subclusters;
    if (visibility_frame_valid_ && visibility_model_.depthCellCount() == 0) {
      std::vector<int> dummy_owners;
      visibility_model_.buildDepth(points, dummy_owners);
    }
    crane_pipeline_->processHookAndBeamDetection(
        active_points, (crane_closed_loop_grouping_enabled_ && !cl_subs.empty()) ? cl_subs : raw_subclusters,
        stampSeconds(header), &visibility_model_, &points);

    const auto cargo_extension_begin=std::chrono::steady_clock::now();
    // Reuse the configured BEV implementation without mutating its diagnostic/history state.
    // These current-Raw layers and clusters are downstream-only; Housing/Wire stay ROI-scoped.
    // Current Raw depth for the Cargo consumer is isolated from upstream
    // Housing/Wire visibility history. Reuse the same model/config, not stale rays.
    VisibilityModel * cargo_visibility=nullptr;
    if(visibility_frame_valid_ && crane_pipeline_->result().work_status.cargo_observation=="DIRECT") {
      if(!cargo_visibility_configured_) {
        cargo_visibility_model_.configure(visibility_model_.config());cargo_visibility_configured_=true;
      }
      // Pose is needed for range-dependent tolerances even without a depth query.
      cargo_visibility_model_.setPose(pose);
      cargo_visibility=&cargo_visibility_model_;
    }
    const auto & cargo_layers=crane_pipeline_->prepareCargoExtension(points,pose,cargo_visibility);
    if(!cargo_layers.layers.empty()){
      if(const auto * bev=dynamic_cast<const BevConnectedComponentClusterer *>(clusterer_.get())){
        auto extension_clusterer=*bev;
        extension_clusterer.reserveLayerPoints(4);
        if(!cargo_extension_bev_scratch_)cargo_extension_bev_scratch_=std::make_unique<CargoExtensionBevScratch>(true,true,true);
        std::vector<cx128s2_bev_perception::CraneSubcluster> extension_clusters;
        int next_fragment=0;
        for(const auto & layer:cargo_layers.layers){
          auto fragments=cargo_extension_bev_scratch_->cluster(extension_clusterer,points,layer.point_indices,processed_frame_count_,stampSeconds(header));
          extension_clusters.reserve(extension_clusters.size()+fragments.size());
          for(auto & fragment:fragments){
            cx128s2_bev_perception::CraneSubcluster sc;sc.subcluster_id=next_fragment++;sc.layer_id=layer.id;
            sc.point_indices=std::move(fragment.indices);sc.centroid_x=fragment.centroid_x;sc.centroid_y=fragment.centroid_y;
            sc.min_x=fragment.min_x;sc.max_x=fragment.max_x;sc.min_y=fragment.min_y;sc.max_y=fragment.max_y;
            sc.min_z=std::numeric_limits<double>::infinity();sc.max_z=-sc.min_z;
            for(int id:sc.point_indices){sc.centroid_z+=points[id].z;sc.min_z=std::min(sc.min_z,double(points[id].z));sc.max_z=std::max(sc.max_z,double(points[id].z));}
            if(!sc.point_indices.empty())sc.centroid_z/=sc.point_indices.size();
            extension_clusters.push_back(std::move(sc));
          }
        }
        crane_pipeline_->recoverCargoFromRawFragments(points,extension_clusters,stampSeconds(header));
        crane_pipeline_->updateCargoExtension(points,extension_clusters,stampSeconds(header),[&](){
          if(cargo_visibility)cargo_visibility_model_.buildDepth(points,{});
        });
      }
    }

    // Ground may be visible around a broad Cargo outside the Hook-sized candidate window.
    crane_hybrid_policy_.cfg["ground_roi_width_x_m"]=pipe_cfg.roi_filter.beam_roi_x_max-pipe_cfg.roi_filter.beam_roi_x_min;
    crane_hybrid_policy_.cfg["ground_roi_width_y_m"]=pipe_cfg.roi_filter.beam_roi_y_max-pipe_cfg.roi_filter.beam_roi_y_min;
    // Current Cargo seed and extension are complete; match the C3 evaluation order.
    crane_hybrid_policy_.process(points,crane_pipeline_->result(),
        crane_pipeline_->config().work_status,stamp_sec);
    crane_pipeline_->updateBeamGroundClearance(points,stamp_sec,crane_beam_config_);
    crane_hybrid_policy_.processMotionAndCompletion(points,crane_pipeline_->result(),
      crane_pipeline_->config().work_status,crane_beam_config_,stamp_sec,crane_pipeline_->config().z_gaussian.min_layer_points,crane_pipeline_->config().dual_roi_anchor.noise_deadband_z);
    const double cargo_extension_full_ms=std::chrono::duration<double,std::milli>(
        std::chrono::steady_clock::now()-cargo_extension_begin).count();

    crane_pipeline_->observeClusters(clusters.size());
    const double cluster_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now()-begin).count();
    const double pure_cpp_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now()-t_pipe_start).count();

    // Publish CraneBeamStatus & WorkingStatus (External Topic Compatibility preserved; no attachment judgment)
    cx128s2_bev_perception::msg::CraneBeamStatus effective_status;
    if (pub_crane_beam_status_) {
      cx128s2_bev_perception::msg::CraneBeamStatus status_msg;
      status_msg.header = header;
      status_msg.state = cx128s2_bev_perception::to_string(effective_state);
      status_msg.working_status = cx128s2_bev_perception::workingStatusName((effective_state == cx128s2_bev_perception::CraneBeamState::LIFTING ||
                                   effective_state == cx128s2_bev_perception::CraneBeamState::SUSPENDED ||
                                   effective_state == cx128s2_bev_perception::CraneBeamState::LOWERING) ? 1 : 0);
      status_msg.work_complete = (effective_state == cx128s2_bev_perception::CraneBeamState::COMPLETE_CONFIRM);
      const auto & pipe_hook = crane_pipeline_->result().hook;
      status_msg.motion_reference_mode = "absolute";
      status_msg.beam_tracking_state = "NONE";
      status_msg.beam_cluster_point_count = static_cast<uint32_t>(clusters.size());
      status_msg.left_hook_detected = pipe_hook.left.detected;
      status_msg.left_hook_tracked = pipe_hook.left.initialized;
      status_msg.left_hook_tracking_state = pipe_hook.left.tracking_state;
      status_msg.left_hook_x = static_cast<float>(pipe_hook.left.x);
      status_msg.left_hook_z = static_cast<float>(pipe_hook.left.z);
      status_msg.left_hook_filtered_z = static_cast<float>(pipe_hook.left.filtered_z);
      status_msg.right_hook_detected = pipe_hook.right.detected;
      status_msg.right_hook_tracked = pipe_hook.right.initialized;
      status_msg.right_hook_tracking_state = pipe_hook.right.tracking_state;
      status_msg.right_hook_x = static_cast<float>(pipe_hook.right.x);
      status_msg.right_hook_z = static_cast<float>(pipe_hook.right.z);
      status_msg.right_hook_filtered_z = static_cast<float>(pipe_hook.right.filtered_z);
      status_msg.prepare_ready = pipe_hook.prepare_ready;
      status_msg.prepare_status = pipe_hook.prepare_ready ? "READY" : "NOT_READY";
      if (crane_pipeline_) {
        status_msg.crane_moving = crane_pipeline_->result().crane_moving;
      }
      crane_auto_status_ = std::move(status_msg);
      effective_status = publishCraneStatus();
    }
    if (debug) {
      auto world_header = msg->header; world_header.frame_id = "world";
      // Exact partition: do not independently downsample its three components.
      // publishCloud checks subscribers before allocating/serializing any cloud.
      publishCloud(world_header, points, res.ground, pub_crane_slope_ground_);
      publishCloud(world_header, points, res.non_ground, pub_crane_slope_non_ground_);
      publishCloud(world_header, points, res.unknown, pub_crane_slope_unknown_);
      const auto publish_rgb = [&](const auto & publisher, const auto & color_at) {
        publishCraneColorCloud(world_header,points,indices,publisher,color_at);
      };
      const auto publish_direction_rgb = [&](const auto & labels, const auto & publisher) {
        if (labels.size()!=points.size()) return;
        publish_rgb(publisher,[&](int idx) {
          const auto label=labels[idx];
          return label==cx128s2_bev_perception::CraneSlopeLabel::GROUND ? 0x00ff00u :
              label==cx128s2_bev_perception::CraneSlopeLabel::NON_GROUND ? 0xff0000u : 0xffff00u;
        });
      };
      publish_direction_rgb(res.labels, pub_crane_slope_classified_);
      publish_direction_rgb(res.vertical_labels, pub_crane_slope_vertical_);
      publish_direction_rgb(res.horizontal_labels, pub_crane_slope_horizontal_);
      // Boundary Clustering Debug Clouds
      const auto & bc = res.boundary_clustering;
      if (pub_crane_slope_boundary_ && pub_crane_slope_boundary_->get_subscription_count() && !bc.boundary_indices.empty()) {
        publishCraneColorCloud(world_header, points, bc.boundary_indices, pub_crane_slope_boundary_, [&](int idx) {
          const auto bt = (idx >= 0 && static_cast<size_t>(idx) < bc.boundary_types.size()) ? bc.boundary_types[idx] : cx128s2_bev_perception::CraneBoundaryType::NONE;
          const bool has_height = hasBoundaryFlag(bt, cx128s2_bev_perception::CraneBoundaryType::HEIGHT_STEP);
          const bool has_slope = hasBoundaryFlag(bt, cx128s2_bev_perception::CraneBoundaryType::SLOPE_DISCONTINUITY);
          const bool has_range = hasBoundaryFlag(bt, cx128s2_bev_perception::CraneBoundaryType::RANGE_JUMP);
          const bool has_gnd = hasBoundaryFlag(bt, cx128s2_bev_perception::CraneBoundaryType::GROUND_NEIGHBOR);

          if (has_height && has_slope) return 0xff00ffu; // Magenta: Multi-edge
          if (has_height) return 0xff0000u;              // Red: Height Step
          if (has_slope) return 0xffff00u;               // Yellow: Slope Discontinuity
          if (has_range) return 0x0080ffu;               // Blue: Range Jump
          if (has_gnd) return 0xff8000u;                 // Orange: Ground Neighbor
          return 0xffffffu;
        });
      }

      if (pub_crane_slope_boundary_clusters_ && pub_crane_slope_boundary_clusters_->get_subscription_count() && !bc.clusters.empty()) {
        std::vector<int> all_cluster_indices;
        for (const auto & cl : bc.clusters) {
          all_cluster_indices.insert(all_cluster_indices.end(), cl.point_indices.begin(), cl.point_indices.end());
        }
        publishCraneColorCloud(world_header, points, all_cluster_indices, pub_crane_slope_boundary_clusters_, [&](int idx) {
          const int cid = (idx >= 0 && static_cast<size_t>(idx) < bc.cluster_labels.size()) ? bc.cluster_labels[idx] : -1;
          if (cid <= 0) return 0x808080u;
          return cx128s2_bev_perception::CranePipeline::heightColor(cid);
        });
      }

      const auto & zg = res.z_gaussian;
      if (pub_crane_slope_z_gaussian_layers_ && pub_crane_slope_z_gaussian_layers_->get_subscription_count() && !zg.layers.empty()) {
        std::vector<int> all_layer_indices;
        all_layer_indices.reserve(zg.grouped_points);
        for (const auto & layer : zg.layers) {
          all_layer_indices.insert(all_layer_indices.end(), layer.point_indices.begin(), layer.point_indices.end());
        }
        publishCraneColorCloud(world_header, active_points, all_layer_indices, pub_crane_slope_z_gaussian_layers_, [&](int idx) {
          const int lid = (idx >= 0 && static_cast<size_t>(idx) < zg.point_layer_ids.size()) ? zg.point_layer_ids[idx] : -1;
          if (lid <= 0) return 0x808080u;
          return cx128s2_bev_perception::CranePipeline::heightColor(lid);
        });
      }

      const auto & clg = res.closed_loop;
      if (pub_crane_slope_closed_loop_groups_ && pub_crane_slope_closed_loop_groups_->get_subscription_count() && !clg.groups.empty()) {
        std::vector<int> grouped_indices;
        for (const auto & grp : clg.groups) {
          grouped_indices.insert(grouped_indices.end(),
                                 grp.merged_point_indices.begin(),
                                 grp.merged_point_indices.end());
        }
        publishCraneColorCloud(world_header, active_points, grouped_indices, pub_crane_slope_closed_loop_groups_, [&](int idx) {
          const int gid = (idx >= 0 && static_cast<size_t>(idx) < clg.point_group_ids.size()) ? clg.point_group_ids[idx] : -1;
          if (gid <= 0) return 0x808080u;
          return cx128s2_bev_perception::CranePipeline::heightColor(gid);
        }, &clg.point_group_ids, "group_id");
      }

      const auto & td_hook = crane_pipeline_->result().top_down_hook;
      const auto & lnc = td_hook.left_nearest_cluster;
      const auto & rnc = td_hook.right_nearest_cluster;
      const auto & llc = td_hook.left_largest_cluster;
      const auto & rlc = td_hook.right_largest_cluster;

      if (pub_crane_slope_boundary_diagnostics_ && pub_crane_slope_boundary_diagnostics_->get_subscription_count()) {
        std_msgs::msg::String bdiag;
        std::ostringstream btext;
        btext << std::fixed << std::setprecision(4)
              << "{\"clusters\":" << bc.clusters.size()
              << ",\"z_gaussian_layers\":" << zg.layers.size()
              << ",\"z_gaussian_clusters\":" << clusters.size()
              << ",\"z_gaussian_ms\":" << zg.grouping_ms
              << ",\"closed_loop_groups\":" << clg.groups.size()
              << ",\"closed_loop_regions\":" << clg.closed_loop_regions
              << ",\"subclusters\":" << clg.subclusters.size()
              << ",\"dz_background\":" << clg.dz_background
              << ",\"closed_loop_ms\":" << clg.grouping_ms
              << ",\"fsm_state\":\"" << cx128s2_bev_perception::to_string(effective_state) << "\""
              << ",\"active_roi\":\"CRANE_ROI\""
              << ",\"bg_roi_pts\":" << (has_beam_res ? beam_res.bg_roi_point_count : 0)
              << ",\"crane_roi_pts\":" << (has_beam_res ? beam_res.beam_roi_point_count : 0)
              << ",\"z_bg\":" << (has_beam_res ? beam_res.z_bg_filtered : 0.0)
              << ",\"dz_bg\":" << (has_beam_res ? beam_res.dz_bg_filtered : 0.0)
              << ",\"relative_dz\":" << (has_beam_res ? beam_res.relative_dz_filtered : 0.0)
              << ",\"boundaries\":" << bc.boundary_indices.size()
              << ",\"height_steps\":" << bc.height_step_indices.size()
              << ",\"slope_discontinuities\":" << bc.slope_indices.size()
              << ",\"range_jumps\":" << bc.range_jump_indices.size()
              << ",\"severed_edges\":" << bc.severed_edges
              << ",\"tested_edges\":" << bc.total_tested_edges
              << ",\"detection_ms\":" << bc.boundary_detection_ms
              << ",\"region_growing_ms\":" << bc.region_growing_ms
              << ",\"total_ms\":" << bc.total_ms
              << ",\"pure_cpp_ms\":" << pure_cpp_ms
              << ",\"work_status\":\"" << cx128s2_bev_perception::workingStatusName(res.work_status.working_status) << "\""
              << ",\"work_reason\":\"" << res.work_status.status_reason << "\""
              << ",\"cargo_track_id\":" << res.work_status.cargo_track_id
              << ",\"extension_observation\":\"" << res.work_status.extension_observation << "\""
              << ",\"extension_reason\":\"" << res.work_status.extension_reason << "\""
              << ",\"extended_length\":" << res.work_status.extended_length
              << ",\"extended_width\":" << res.work_status.extended_width
              << ",\"extended_height\":" << res.work_status.extended_height
              << ",\"observed_length_sum\":" << res.work_status.observed_length_sum
              << ",\"inferred_gap_count\":" << res.work_status.inferred_gaps.size()
              << ",\"extension_ms\":" << res.work_status.extension_ms
              << ",\"extension_full_ms\":" << cargo_extension_full_ms
              << ",\"beam_gap\":\"" << res.work_status.gap_diagnostic << "\""
              << ",\"beam_gap_reason\":\"" << res.work_status.gap_reason << "\""
              << ",\"left_nearest_id\":" << (lnc.valid ? lnc.subcluster_id : -1)
              << ",\"left_nearest_dxy\":" << (lnc.valid ? lnc.dxy : -1.0)
              << ",\"left_nearest_dz\":" << (lnc.valid ? lnc.dz : -1.0)
              << ",\"right_nearest_id\":" << (rnc.valid ? rnc.subcluster_id : -1)
              << ",\"right_nearest_dxy\":" << (rnc.valid ? rnc.dxy : -1.0)
              << ",\"right_nearest_dz\":" << (rnc.valid ? rnc.dz : -1.0)
              << ",\"left_largest_id\":" << (llc.valid ? llc.subcluster_id : -1)
              << ",\"left_largest_len\":" << (llc.valid ? llc.length : 0.0)
              << ",\"left_largest_wid\":" << (llc.valid ? llc.width : 0.0)
              << ",\"left_largest_area\":" << (llc.valid ? llc.area_xy : 0.0)
              << ",\"left_largest_yaw\":" << (llc.valid ? llc.yaw : 0.0)
              << ",\"left_largest_held\":" << (llc.is_held ? 1 : 0)
              << ",\"right_largest_id\":" << (rlc.valid ? rlc.subcluster_id : -1)
              << ",\"right_largest_len\":" << (rlc.valid ? rlc.length : 0.0)
              << ",\"right_largest_wid\":" << (rlc.valid ? rlc.width : 0.0)
              << ",\"right_largest_area\":" << (rlc.valid ? rlc.area_xy : 0.0)
              << ",\"right_largest_yaw\":" << (rlc.valid ? rlc.yaw : 0.0)
              << ",\"right_largest_held\":" << (rlc.is_held ? 1 : 0)
              << ",\"left_chain_held\":" << (td_hook.left_chain_idx >= 0 && td_hook.left_chain_idx < static_cast<int>(td_hook.chains.size()) && td_hook.chains[td_hook.left_chain_idx].is_held ? 1 : 0)
              << ",\"right_chain_held\":" << (td_hook.right_chain_idx >= 0 && td_hook.right_chain_idx < static_cast<int>(td_hook.chains.size()) && td_hook.chains[td_hook.right_chain_idx].is_held ? 1 : 0)
              << ",\"left_chain_max_slope\":" << (td_hook.left_chain_idx >= 0 && td_hook.left_chain_idx < static_cast<int>(td_hook.chains.size()) ? td_hook.chains[td_hook.left_chain_idx].max_slope_angle_deg : 0.0)
              << ",\"left_chain_max_turn\":" << (td_hook.left_chain_idx >= 0 && td_hook.left_chain_idx < static_cast<int>(td_hook.chains.size()) ? td_hook.chains[td_hook.left_chain_idx].max_turn_angle_deg : 0.0)
              << ",\"right_chain_max_slope\":" << (td_hook.right_chain_idx >= 0 && td_hook.right_chain_idx < static_cast<int>(td_hook.chains.size()) ? td_hook.chains[td_hook.right_chain_idx].max_slope_angle_deg : 0.0)
              << ",\"right_chain_max_turn\":" << (td_hook.right_chain_idx >= 0 && td_hook.right_chain_idx < static_cast<int>(td_hook.chains.size()) ? td_hook.chains[td_hook.right_chain_idx].max_turn_angle_deg : 0.0)
              << ",\"left_exception_status\":\"" << cx128s2_bev_perception::to_string(td_hook.left_exception_gate.status) << "\""
              << ",\"left_exception_reason\":\"" << cx128s2_bev_perception::to_string(td_hook.left_exception_gate.reason) << "\""
              << ",\"left_exception_vis\":\"" << cx128s2_bev_perception::to_string(td_hook.left_exception_gate.visibility_status) << "\""
              << ",\"left_exception_max_len\":" << td_hook.left_exception_gate.max_connected_length
              << ",\"left_exception_max_area\":" << td_hook.left_exception_gate.max_connected_area
              << ",\"right_exception_status\":\"" << cx128s2_bev_perception::to_string(td_hook.right_exception_gate.status) << "\""
              << ",\"right_exception_reason\":\"" << cx128s2_bev_perception::to_string(td_hook.right_exception_gate.reason) << "\""
              << ",\"right_exception_vis\":\"" << cx128s2_bev_perception::to_string(td_hook.right_exception_gate.visibility_status) << "\""
              << ",\"right_exception_max_len\":" << td_hook.right_exception_gate.max_connected_length
              << ",\"right_exception_max_area\":" << td_hook.right_exception_gate.max_connected_area
              << "}";
        bdiag.data = btext.str();
        pub_crane_slope_boundary_diagnostics_->publish(bdiag);
      }

      if (pub_crane_slope_clusters_ && pub_crane_slope_clusters_->get_subscription_count() && !clusters.empty()) {
        std::vector<int> clustered;
        std::vector<int> pt_cluster_id(active_points.size(), -1);
        for (size_t ci = 0; ci < clusters.size(); ++ci) {
          const int cid = clusters[ci].id;
          for (int idx : clusters[ci].indices) {
            clustered.push_back(idx);
            if (idx >= 0 && static_cast<size_t>(idx) < active_points.size()) {
              pt_cluster_id[idx] = cid;
            }
          }
        }
        publishCraneColorCloud(world_header, active_points, clustered, pub_crane_slope_clusters_, [&](int idx) {
          const int cid = (idx >= 0 && static_cast<size_t>(idx) < pt_cluster_id.size()) ? pt_cluster_id[idx] : -1;
          if (cid <= 0) return 0x808080u;
          return cx128s2_bev_perception::CranePipeline::heightColor(cid);
        });
      }

      if (crane_beam_detector_) {
        int dummy = 0;
        publishDebugCloud(world_header, points, crane_beam_detector_->getBeamRoiIndices(),
                          pub_crane_beam_roi_points_, true, dummy);
        publishDebugCloud(world_header, points, crane_beam_detector_->getBgRoiIndices(),
                          pub_crane_beam_bg_roi_points_, true, dummy);
      }
      const auto & p_hook = crane_pipeline_->result().hook;

      if (pub_crane_beam_markers_ && pub_crane_beam_markers_->get_subscription_count() > 0) {
        visualization_msgs::msg::MarkerArray clear_arr;
        visualization_msgs::msg::Marker clear_m;
        clear_m.header = world_header;
        clear_m.action = visualization_msgs::msg::Marker::DELETEALL;
        clear_arr.markers.push_back(clear_m);
        pub_crane_beam_markers_->publish(clear_arr);
      }
      if (crane_beam_config_.publish_debug && pub_crane_hook_markers_) {
        const auto * td_res = crane_pipeline_ ? &crane_pipeline_->result().top_down_hook : nullptr;
        const auto * ws_res = crane_pipeline_ ? &crane_pipeline_->result().work_status : nullptr;
        const auto * bm_res = crane_pipeline_ ? &crane_pipeline_->result().beam : nullptr;
        publishCraneHookMarkers(world_header, p_hook, effective_status, td_res, ws_res, bm_res, &raw_subclusters, pure_cpp_ms);
      }
      if (pub_crane_hook_left_body_) {
        int dummy = 0;
        publishDebugCloud(world_header, active_points, p_hook.left.body_indices, pub_crane_hook_left_body_, true, dummy);
      }
      if (pub_crane_hook_right_body_) {
        int dummy = 0;
        publishDebugCloud(world_header, active_points, p_hook.right.body_indices, pub_crane_hook_right_body_, true, dummy);
      }

      if (pub_crane_beam_diagnostics_) {
        std_msgs::msg::String diag;
        std::ostringstream text;
        text << "pipeline=slope fsm=" << cx128s2_bev_perception::to_string(effective_state)
             << " active_roi=CRANE_ROI"
             << " hook_L=" << (p_hook.left.detected ? "DET" : "NONE")
             << " hook_R=" << (p_hook.right.detected ? "DET" : "NONE")
             << " hook_ready=" << (p_hook.prepare_ready ? "READY" : "NOT_READY")
             << " near_L=" << (lnc.valid ? lnc.subcluster_id : -1)
             << " near_L_dxy=" << std::fixed << std::setprecision(3) << (lnc.valid ? lnc.dxy : -1.0)
             << " near_L_dz=" << (lnc.valid ? lnc.dz : -1.0)
             << " near_R=" << (rnc.valid ? rnc.subcluster_id : -1)
             << " near_R_dxy=" << (rnc.valid ? rnc.dxy : -1.0)
             << " near_R_dz=" << (rnc.valid ? rnc.dz : -1.0)
             << " processed=" << indices.size()
             << " model_mismatch=" << res.model_mismatch_points
             << " ground=" << res.ground.size()
             << " non_ground=" << res.non_ground.size() << " unknown=" << res.unknown.size()
             << " algorithm=supplied_cascaded_prefix overwritten=" << res.overwritten_points
             << " unsupported=" << res.unsupported_points << " rings=" << res.observed_rings << " ring_available=" << last_input_has_ring_
             << " pose_valid=" << res.pose_valid << " neighbor_pairs=" << res.neighbor_pairs
             << " broken_pairs=" << res.broken_pairs
             << " horizontal_pairs=" << res.horizontal_neighbor_pairs
             << " horizontal_broken=" << res.horizontal_broken_pairs
             << " horizontal_gaps=" << res.horizontal_gap_breaks
             << " horizontal_unsupported=" << res.horizontal_unsupported_pairs
             << " horizontal_runs=" << res.horizontal_runs << " clusters=" << clusters.size()
             << " z_gaussian_layers=" << zg.layers.size()
             << " z_gaussian_ms=" << zg.grouping_ms
             << " closed_loop_groups=" << clg.groups.size()
             << " closed_loop_regions=" << clg.closed_loop_regions
             << " dz_background=" << clg.dz_background
             << " closed_loop_ms=" << clg.grouping_ms
             << " boundary_clusters=" << bc.clusters.size()
             << " boundary_points=" << bc.boundary_indices.size()
             << " segmentation_ms=" << res.segmentation_ms << " clustering_ms=" << cluster_ms
             << " pure_cpp_ms=" << pure_cpp_ms
             << " boundary_clustering_ms=" << bc.total_ms
             << " workspace_bytes=" << res.workspace_bytes;
        diag.data = text.str(); pub_crane_beam_diagnostics_->publish(diag);
      }
    }
  }

  void processCraneBeamPerception(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg,
      const std_msgs::msg::Header & out_header,
      const std::vector<PointXYZIRT> & points,
      const std::vector<int> & valid_idx,
      double now_sec,
      std::chrono::steady_clock::time_point start_time,
      bool publish_debug_cloud) {
    if (!crane_beam_detector_) return;
    std::lock_guard<std::mutex> lock(crane_command_mutex_);
    if (crane_reset_pending_) {
      resetPerceptionRuntimeState();
      crane_reset_pending_ = false;
      RCLCPP_INFO(get_logger(), "[CRANE_COMMAND] RESET applied on sensor frame");
    }

    const double stamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    const auto detector_begin = std::chrono::steady_clock::now();
    const auto previous_hook_modes = std::array<std::string, 2>{
        crane_beam_detector_->getHookDetector().getResult().left.position_tracking_state,
        crane_beam_detector_->getHookDetector().getResult().right.position_tracking_state};
    const auto & res = crane_beam_detector_->process(points, valid_idx, stamp);
    const std::array<const cx128s2_bev_perception::HookDetection *, 2> position_hooks{
        &res.hook.left, &res.hook.right};
    for (size_t side = 0; side < position_hooks.size(); ++side) {
      const auto & hook = *position_hooks[side];
      if (hook.position_tracking_state != previous_hook_modes[side]) {
        RCLCPP_INFO(get_logger(),
            "[CRANE_HOOK][%s] %s -> %s body_detected=%d position_valid=%d "
            "position=(%.3f,%.3f,%.3f) fallback_valid=%d reason=%s",
            side == 0 ? "LEFT" : "RIGHT", previous_hook_modes[side].c_str(),
            hook.position_tracking_state.c_str(), hook.detected, hook.position_valid,
            hook.tracking_x, hook.tracking_y, hook.tracking_z, hook.fallback_point_valid,
            hook.position_tracking_state == "LOST" ? "NO_VALID_ROI_POINT_HOLD" : hook.selection_reason.c_str());
      }
    }
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
        "[CRANE_HOOK] LEFT=%s body=%d fallback=(%.3f,%.3f,%.3f) "
        "RIGHT=%s body=%d fallback=(%.3f,%.3f,%.3f)",
        res.hook.left.position_tracking_state.c_str(), res.hook.left.detected,
        res.hook.left.fallback_x, res.hook.left.fallback_y, res.hook.left.fallback_z,
        res.hook.right.position_tracking_state.c_str(), res.hook.right.detected,
        res.hook.right.fallback_x, res.hook.right.fallback_y, res.hook.right.fallback_z);
    const double detector_cpu_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - detector_begin).count();

    if (res.crane_move_reason.rfind("START_", 0) == 0 ||
        res.crane_move_reason == "STOP_STABLE_ANCHOR_UPDATED") {
      RCLCPP_INFO(get_logger(),
          "[CRANE_MOVE] state=%s sources=%d anchor=(%.3f,%.3f) hook=(%.3f,%.3f) "
          "net=(%.3f,%.3f) START=(%.3f,%.3f) span=(%.3f,%.3f) STOP=(%.3f,%.3f) "
          "valid_measured_frames=%d/%d reason=%s",
          res.crane_moving ? "MOVING" : "STOPPED", res.crane_move_source_mask,
          res.crane_move_anchor_x, res.crane_move_anchor_y, res.crane_move_hook_x, res.crane_move_hook_y,
          res.crane_move_net_dx, res.crane_move_net_dy,
          crane_beam_config_.crane_move_start_x, crane_beam_config_.crane_move_start_y,
          res.crane_move_x_span, res.crane_move_y_span,
          crane_beam_config_.crane_move_stop_x, crane_beam_config_.crane_move_stop_y,
          res.crane_move_measured_frames, crane_beam_config_.crane_move_stop_frames, res.crane_move_reason.c_str());
    }


    // 1. Log INFO on FSM state transitions
    if (res.state_changed) {
      RCLCPP_INFO(get_logger(), "[CraneBeam][AUTO] %s -> %s",
                  res.prev_state_str.c_str(), res.curr_state_str.c_str());
    }

    // 2. Publish structured status message (/crane_beam/status)
    cx128s2_bev_perception::msg::CraneBeamStatus effective_status;
    if (pub_crane_beam_status_) {
      cx128s2_bev_perception::msg::CraneBeamStatus status_msg;
      status_msg.header = out_header;
      status_msg.state = cx128s2_bev_perception::to_string(res.state);
      status_msg.working_status =cx128s2_bev_perception::workingStatusName(res.working_status);
      status_msg.motion_reference_mode = res.motion_reference_mode;
      status_msg.z_beam = static_cast<float>(res.z_beam_filtered);
      status_msg.z_background = static_cast<float>(res.z_bg_filtered);
      status_msg.dz_beam = static_cast<float>(res.dz_beam_filtered);
      status_msg.dz_background = static_cast<float>(res.dz_bg_filtered);
      status_msg.relative_dz = static_cast<float>(res.relative_dz_filtered);
      status_msg.motion_dz = static_cast<float>(res.motion_dz);
      status_msg.beam_mad = static_cast<float>(res.beam_mad);
      status_msg.background_mad = static_cast<float>(res.bg_mad);
      status_msg.beam_roi_point_count = static_cast<uint32_t>(res.beam_roi_point_count);
      status_msg.background_roi_point_count = static_cast<uint32_t>(res.bg_roi_point_count);
      status_msg.beam_cluster_point_count = static_cast<uint32_t>(res.beam_cluster_point_count);
      status_msg.beam_length = static_cast<float>(res.length_locked ? res.beam_length_locked : res.beam_length_raw);
      status_msg.active_left_hook = res.active_left;
      status_msg.active_right_hook = res.active_right;
      status_msg.beam_tracking_state = res.beam_tracking_state;
      status_msg.beam_lost_frames = res.lost_frames_counter;
      status_msg.left_prepare_ready = res.hook.prepare_sides[0];
      status_msg.right_prepare_ready = res.hook.prepare_sides[1];
      status_msg.beam_length_locked = static_cast<float>(res.beam_length_locked);
      status_msg.length_locked = res.length_locked;
      status_msg.beam_major_axis_yaw = static_cast<float>(res.beam_major_axis_yaw);
      status_msg.beam_z_valid = res.beam_z_valid;
      status_msg.background_z_valid = res.bg_z_valid;
      status_msg.beam_cluster_valid = res.beam_cluster_valid;
      status_msg.crane_moving = res.crane_moving;
      status_msg.placed_candidate = res.placed_candidate;
      status_msg.work_complete = res.work_complete;
      status_msg.beam_measurement_source =
          (res.measurement_source == cx128s2_bev_perception::BeamMeasurementSource::INITIAL_ROI)
              ? "INITIAL_ROI" : "TRACKED_CLUSTER";
      status_msg.support_valid = res.support_valid;
      status_msg.support_z = static_cast<float>(res.support_z);
      status_msg.support_point_count = static_cast<uint32_t>(res.support_point_count);
      status_msg.placement_gap_z = static_cast<float>(res.placement_gap_z);
      status_msg.contact_candidate = res.contact_candidate;
      status_msg.supported_segment_ratio = res.supported_segment_ratio;
      status_msg.motion_cumulative_dz = res.motion_cumulative_dz;
      status_msg.beam_z_min = static_cast<float>(res.z_beam_min);
      status_msg.beam_z_max = static_cast<float>(res.z_beam_max);
      status_msg.beam_centroid_x = static_cast<float>(res.cluster_centroid_x);
      status_msg.beam_centroid_y = static_cast<float>(res.cluster_centroid_y);
      status_msg.tracking_candidate_count = static_cast<uint32_t>(res.tracking_candidate_count);
      status_msg.selected_tracking_score = static_cast<float>(res.selected_tracking_score);

      // Hook measurements & auxiliary readiness flag; preserve the legacy string field.
      status_msg.left_hook_detected = res.hook.left.detected;
      status_msg.left_hook_x = static_cast<float>(res.hook.left.x);
      status_msg.left_hook_z = static_cast<float>(res.hook.left.z);
      status_msg.left_ground_z = static_cast<float>(res.hook.left.ground_z);
      status_msg.left_hook_ground_distance = static_cast<float>(res.hook.left.ground_distance);

      status_msg.right_hook_detected = res.hook.right.detected;
      status_msg.right_hook_x = static_cast<float>(res.hook.right.x);
      status_msg.right_hook_z = static_cast<float>(res.hook.right.z);
      status_msg.right_ground_z = static_cast<float>(res.hook.right.ground_z);
      status_msg.right_hook_ground_distance = static_cast<float>(res.hook.right.ground_distance);

      status_msg.mean_hook_ground_distance = static_cast<float>(res.hook.mean_ground_distance);
      status_msg.prepare_ready = res.hook.prepare_ready;
      status_msg.prepare_status = res.hook.prepare_ready ? "READY" : "NOT_READY";
      status_msg.left_hook_tracked = res.hook.left.initialized;
      status_msg.left_hook_position_tracking_state = res.hook.left.position_tracking_state;
      status_msg.left_hook_position_valid = res.hook.left.position_valid;
      status_msg.left_hook_position_updated = res.hook.left.position_updated;
      status_msg.left_hook_fallback_point_valid = res.hook.left.fallback_point_valid;
      status_msg.left_hook_tracking_position.x = res.hook.left.tracking_x;
      status_msg.left_hook_tracking_position.y = res.hook.left.tracking_y;
      status_msg.left_hook_tracking_position.z = res.hook.left.tracking_z;
      status_msg.left_hook_fallback_position.x = res.hook.left.fallback_x;
      status_msg.left_hook_fallback_position.y = res.hook.left.fallback_y;
      status_msg.left_hook_fallback_position.z = res.hook.left.fallback_z;
      status_msg.right_hook_position_tracking_state = res.hook.right.position_tracking_state;
      status_msg.right_hook_position_valid = res.hook.right.position_valid;
      status_msg.right_hook_position_updated = res.hook.right.position_updated;
      status_msg.right_hook_fallback_point_valid = res.hook.right.fallback_point_valid;
      status_msg.right_hook_tracking_position.x = res.hook.right.tracking_x;
      status_msg.right_hook_tracking_position.y = res.hook.right.tracking_y;
      status_msg.right_hook_tracking_position.z = res.hook.right.tracking_z;
      status_msg.right_hook_fallback_position.x = res.hook.right.fallback_x;
      status_msg.right_hook_fallback_position.y = res.hook.right.fallback_y;
      status_msg.right_hook_fallback_position.z = res.hook.right.fallback_z;
      status_msg.left_hook_tracking_state = res.hook.left.tracking_state;
      status_msg.left_hook_lost_frames = static_cast<uint32_t>(res.hook.left.lost_frames);
      status_msg.left_hook_filtered_z = static_cast<float>(res.hook.left.filtered_z);
      status_msg.left_hook_vz = static_cast<float>(res.hook.left.vz);
      status_msg.right_hook_tracked = res.hook.right.initialized;
      status_msg.right_hook_tracking_state = res.hook.right.tracking_state;
      status_msg.right_hook_lost_frames = static_cast<uint32_t>(res.hook.right.lost_frames);
      status_msg.right_hook_filtered_z = static_cast<float>(res.hook.right.filtered_z);
      status_msg.right_hook_vz = static_cast<float>(res.hook.right.vz);

      crane_auto_status_ = std::move(status_msg);
      effective_status = publishCraneStatus();
    }

    // Motion-tuning telemetry. All values are scalars already calculated by the detector.
    if (crane_beam_config_.publish_debug && pub_crane_beam_motion_debug_) {
      cx128s2_bev_perception::msg::CraneBeamMotionDebug debug_msg;
      debug_msg.header = out_header;
      debug_msg.fsm_state = static_cast<uint8_t>(res.state);
      debug_msg.fsm_state_name = cx128s2_bev_perception::to_string(res.state);
      debug_msg.motion_reference_mode = res.motion_reference_mode;
      debug_msg.dt_sec = res.dt_sec;
      debug_msg.z_background_raw = res.z_bg_raw;
      debug_msg.z_background_filtered = res.z_bg_filtered;
      debug_msg.z_beam_raw = res.z_beam_raw;
      debug_msg.z_beam_filtered = res.z_beam_filtered;
      debug_msg.height_gap_raw = res.height_gap_raw;
      debug_msg.height_gap_filtered = res.height_gap_filtered;
      debug_msg.beam_high_z_indicator = res.beam_high_z_indicator;
      debug_msg.beam_high_z_reject_threshold = crane_beam_config_.beam_high_z_reject_threshold;
      debug_msg.dz_background_raw = res.dz_bg_raw;
      debug_msg.dz_background_filtered = res.dz_bg_filtered;
      debug_msg.dz_beam_raw = res.dz_beam_raw;
      debug_msg.dz_beam_filtered = res.dz_beam_filtered;
      debug_msg.relative_dz_raw = res.relative_dz_raw;
      debug_msg.relative_dz_filtered = res.relative_dz_filtered;
      debug_msg.v_background_raw = res.v_bg_raw;
      debug_msg.v_background_filtered = res.v_bg_filtered;
      debug_msg.v_beam_raw = res.v_beam_raw;
      debug_msg.v_beam_filtered = res.v_beam_filtered;
      debug_msg.bg_mad = res.bg_mad;
      debug_msg.beam_mad = res.beam_mad;
      debug_msg.move_threshold = crane_beam_config_.crane_move_start_x; // Legacy scalar: START_X.
      debug_msg.background_valid = res.bg_z_valid;
      debug_msg.beam_valid = res.beam_z_valid;
      debug_msg.beam_cluster_valid = res.beam_cluster_valid;
      debug_msg.beam_high_z_rejected = res.beam_high_z_rejected;
      debug_msg.seed_roi_connected = res.seed_roi_connected;
      debug_msg.acquisition_reset_reason = static_cast<uint8_t>(res.acquisition_reset_reason);
      debug_msg.crane_moving = res.crane_moving;
      debug_msg.initial_cluster_length = res.initial_cluster_length;
      debug_msg.initial_cluster_elongation = res.initial_cluster_elongation;
      debug_msg.background_point_count = static_cast<uint32_t>(res.bg_roi_point_count);
      debug_msg.beam_roi_point_count = static_cast<uint32_t>(res.beam_roi_point_count);
      debug_msg.beam_cluster_point_count = static_cast<uint32_t>(res.beam_cluster_point_count);
      pub_crane_beam_motion_debug_->publish(debug_msg);
    }

    // The structured and simple working status share the final projection above.

    // 4. Publish throttled text diagnostics (/crane_beam/diagnostics)
    if (pub_crane_beam_diagnostics_ &&
        (now_sec - last_crane_diag_time_ >= crane_beam_config_.diagnostics_period_sec || last_crane_diag_time_ == 0.0)) {
      last_crane_diag_time_ = now_sec;
      const std::string src_str = (res.measurement_source == cx128s2_bev_perception::BeamMeasurementSource::INITIAL_ROI)
          ? "INITIAL_ROI" : "TRACKED_CLUSTER";
      std::ostringstream oss;
      oss << "detector_cpu_ms: " << detector_cpu_ms << "\n";

      oss << "--- Crane Beam Perception Diagnostics ---\n"
          << "control_mode: " << effective_status.control_mode << "\n"
          << "MANUAL override: " << effective_status.manual_override << "\n"
          << "effective result: state=" << effective_status.state
          << ", working=" << effective_status.working_status
          << ", beam_detected=" << effective_status.beam_detected
          << ", ready=" << effective_status.prepare_ready << ", complete=" << effective_status.work_complete << "\n"
          << "AUTO state: " << cx128s2_bev_perception::to_string(res.state) << "\n"
          << "AUTO working_status: " << res.working_status << "\n"
          << "measurement_source: " << src_str << "\n"
          << "target_frame: " << crane_beam_config_.target_frame << "\n"
          << "motion_reference_mode: " << res.motion_reference_mode << "\n"
          << "transform_frozen: " << (has_frozen_crane_transform_ ? "true" : "false") << "\n"
          << "frozen_reference_frame: " << frozen_crane_target_frame_ << "\n"
          << "frozen_input_frame: " << frozen_crane_input_frame_ << "\n"
          << "frozen_translation: [" << (has_frozen_crane_transform_ ? frozen_crane_transform_.getOrigin().x() : 0.0) << ", "
                                     << (has_frozen_crane_transform_ ? frozen_crane_transform_.getOrigin().y() : 0.0) << ", "
                                     << (has_frozen_crane_transform_ ? frozen_crane_transform_.getOrigin().z() : 0.0) << "]\n"
          << "frozen_rotation_quat: [" << (has_frozen_crane_transform_ ? frozen_crane_transform_.getRotation().x() : 0.0) << ", "
                                       << (has_frozen_crane_transform_ ? frozen_crane_transform_.getRotation().y() : 0.0) << ", "
                                       << (has_frozen_crane_transform_ ? frozen_crane_transform_.getRotation().z() : 0.0) << ", "
                                       << (has_frozen_crane_transform_ ? frozen_crane_transform_.getRotation().w() : 1.0) << "]\n"

          << "z_beam_raw: " << res.z_beam_raw << " m, filtered: " << res.z_beam_filtered << " m (valid=" << (res.beam_z_valid ? "true" : "false") << ")\n"
          << "z_beam quartiles: [min=" << res.z_beam_min << ", Q1=" << res.z_beam_q1 << ", med=" << res.z_beam_raw << ", Q3=" << res.z_beam_q3 << ", max=" << res.z_beam_max << "]\n"
          << "z_bg_raw: " << res.z_bg_raw << " m, filtered: " << res.z_bg_filtered << " m (valid=" << (res.bg_z_valid ? "true" : "false") << ")\n"
          << "z_bg quartiles: [min=" << res.z_bg_min << ", Q1=" << res.z_bg_q1 << ", med=" << res.z_bg_raw << ", Q3=" << res.z_bg_q3 << ", max=" << res.z_bg_max << "]\n"
          << "dz_beam: " << res.dz_beam_filtered << " m\n"
          << "dz_bg: " << res.dz_bg_filtered << " m\n"
          << "relative_dz: " << res.relative_dz_filtered << " m\n"
          << "motion_dz: " << res.motion_dz << " m\n"
          << "beam_mad: " << res.beam_mad << " m, bg_mad: " << res.bg_mad << " m\n"
          << "beam_roi_pts: " << res.beam_roi_point_count << ", bg_roi_pts: " << res.bg_roi_point_count << ", cluster_pts: " << res.beam_cluster_point_count << "\n"
          << "beam_length_raw: " << res.beam_length_raw << " m, locked: " << res.beam_length_locked << " m (length_locked=" << (res.length_locked ? "true" : "false") << ")\n"
          << "beam_yaw: " << (res.beam_major_axis_yaw * 180.0 / M_PI) << " deg\n"
          << "support_valid: " << (res.support_valid ? "true" : "false") << ", support_z: " << res.support_z
          << " m, support_pts: " << res.support_point_count << ", gap_z: " << res.placement_gap_z << " m\n"
          << "contact_candidate: " << (res.contact_candidate ? "true" : "false")
          << ", supported_segment_ratio: " << res.supported_segment_ratio
          << ", motion_cumulative_dz: " << res.motion_cumulative_dz << " m\n"
          << "tracking_candidates: " << res.tracking_candidate_count << ", selected_score: " << res.selected_tracking_score << "\n"
          << "beam_tracking_state: " << res.beam_tracking_state << ", active_left=" << res.active_left << ", active_right=" << res.active_right << "\n"
          << "crane_moving: " << (res.crane_moving ? "true" : "false") << "\n"
          << "[CRANE_MOVE] state=" << (res.crane_moving ? "MOVING" : "STOPPED")
          << " valid=" << res.crane_move_valid << " sources=" << res.crane_move_source_mask
          << " anchor=(" << res.crane_move_anchor_x << "," << res.crane_move_anchor_y << ")"
          << " hook=(" << res.crane_move_hook_x << "," << res.crane_move_hook_y << ")"
          << " net=(" << res.crane_move_net_dx << "," << res.crane_move_net_dy << ")"
          << " start=(" << crane_beam_config_.crane_move_start_x << "," << crane_beam_config_.crane_move_start_y << ")"
          << " span=(" << res.crane_move_x_span << "," << res.crane_move_y_span << ")"
          << " stop=(" << crane_beam_config_.crane_move_stop_x << "," << crane_beam_config_.crane_move_stop_y << ")"
          << " valid_measured_frames=" << res.crane_move_measured_frames << "/" << crane_beam_config_.crane_move_stop_frames
          << " reason=" << res.crane_move_reason << "\n"
          << "placed_candidate: " << (res.placed_candidate ? "true" : "false") << "\n"
          << "work_complete: " << (res.work_complete ? "true" : "false") << "\n"
          << "counters: lift=" << res.lift_counter << ", stop=" << res.stop_counter
          << ", lower=" << res.lower_counter << ", placed=" << res.placed_counter
          << ", complete=" << res.complete_counter << ", lock=" << res.length_lock_counter
          << ", lost=" << res.lost_frames_counter << "\n"
          << "hook: left=[det=" << (res.hook.left.detected ? "true" : "false")
          << ", z=" << res.hook.left.z << ", dist=" << res.hook.left.ground_distance
          << ", pts=" << res.hook.left.body_points << "], right=[det=" << (res.hook.right.detected ? "true" : "false")
          << ", z=" << res.hook.right.z << ", dist=" << res.hook.right.ground_distance
          << ", pts=" << res.hook.right.body_points << "], mean_dist=" << res.hook.mean_ground_distance
          << ", prepare_ready=" << (res.hook.prepare_ready ? "true" : "false") << "\n";
      for (const auto * hook : {&res.hook.left, &res.hook.right}) {
        oss << (hook == &res.hook.left ? "left" : "right")
            << "_hook_tracking: state=" << hook->tracking_state
            << " measurement_valid=" << hook->detected << " lost_frames=" << hook->lost_frames
            << " measured=(" << hook->x << "," << hook->y << "," << hook->z << ")"
            << " predicted=(" << hook->predicted_x << "," << hook->predicted_y << "," << hook->predicted_z << ")"
            << " filtered=(" << hook->filtered_x << "," << hook->filtered_y << "," << hook->filtered_z << ")"
            << " velocity=(" << hook->vx << "," << hook->vy << "," << hook->vz << ")"
            << " predicted_ground_distance_valid=" << hook->predicted_ground_distance_valid
            << " predicted_ground_distance=" << hook->predicted_ground_distance
            << " ground_valid=" << hook->ground_valid << " ground_z=" << hook->ground_z
            << " ground_points=" << hook->ground_points
            << " beam_dz_valid=" << hook->beam_distance_valid
            << " beam_reference_z=" << hook->beam_reference_z
            << " hook_beam_dz=" << hook->beam_dz
            << " candidate_count=" << hook->candidate_count
            << " selected_points=" << hook->body_points
            << " body_bottom_z=" << hook->body_min[2]
            << " body_xy=" << hook->body_width << "/" << hook->body_length
            << " saved_reference_loaded=" << crane_beam_detector_->getHookDetector().getSavedReference(
                hook == &res.hook.left ? 0 : 1).reference_valid
            << " reference_valid=" << hook->reference_valid
            << " reference_xy=" << hook->reference_width << "/" << hook->reference_length
            << " reference_z_extent=" << hook->reference_max[2] - hook->reference_min[2]
            << " body_surface_ratio=" << hook->body_surface_ratio
            << " reference_surface_ratio=" << hook->reference_surface_ratio
            << " partial_observation=" << hook->partial_observation
            << " selection_prediction_z=" << hook->selection_prediction_z
            << " previous_measurement_z=" << hook->selection_previous_z
            << " prediction_error_z=" << hook->prediction_error_z
            << " previous_error_z=" << hook->previous_error_z
            << " previous_window_overlap=" << hook->previous_window_overlap
            << " selection_gate_z=" << hook->selection_gate_z
            << " selection_reason=" << hook->selection_reason
            << " candidate_details=" << hook->candidate_details << "\n";
      }
      std_msgs::msg::String diag_msg;
      diag_msg.data = oss.str();
      pub_crane_beam_diagnostics_->publish(diag_msg);

      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "[CraneDiag] state=%s ws=%d src=%s tf_frozen=%s z_beam=%.3f z_bg=%.3f sup_z=%.2f(gap=%.2f,valid=%s) contact=%s(ratio=%.2f) motion_dz=%.3f(cum=%.3f) pts(beam=%zu,bg=%zu,clust=%zu) len=%.2f(lock=%s) complete=%s hook:L=[%s,z=%.2f,d=%.2f] R=[%s,z=%.2f,d=%.2f] prepare_ready=%s",
          cx128s2_bev_perception::to_string(res.state), res.working_status,
          src_str.c_str(), (has_frozen_crane_transform_ ? "Y" : "N"),
          res.z_beam_filtered, res.z_bg_filtered,
          res.support_z, res.placement_gap_z, (res.support_valid ? "Y" : "N"),
          (res.contact_candidate ? "Y" : "N"), res.supported_segment_ratio,
          res.motion_dz, res.motion_cumulative_dz,
          res.beam_roi_point_count, res.bg_roi_point_count, res.beam_cluster_point_count,
          res.beam_length_raw, (res.length_locked ? "Y" : "N"), (res.work_complete ? "Y" : "N"),
          (res.hook.left.detected ? "Y" : "N"), res.hook.left.z, res.hook.left.ground_distance,
          (res.hook.right.detected ? "Y" : "N"), res.hook.right.z, res.hook.right.ground_distance,
          res.hook.prepare_ready ? "true" : "false");
    }

    // CSV debug logging (/tmp/crane_beam_telemetry.csv)
    if (crane_beam_config_.publish_debug) {
      static bool header_written = false;
      std::ofstream csv_out("/tmp/crane_beam_telemetry.csv", std::ios::app);
      if (csv_out.is_open()) {
        if (!header_written) {
          csv_out << "stamp,state,working_status,measurement_source,z_beam_raw,z_beam_filtered,z_beam_min,z_beam_q1,z_beam_q3,z_beam_max,"
                  << "z_bg_raw,z_bg_filtered,z_bg_min,z_bg_q1,z_bg_q3,z_bg_max,"
                  << "dz_beam,dz_bg,relative_dz,motion_dz,beam_mad,bg_mad,"
                  << "beam_roi_pts,bg_roi_pts,cluster_pts,beam_length,length_locked,"
                  << "support_valid,support_z,support_point_count,placement_gap_z,"
                  << "tracking_candidate_count,selected_tracking_score,"
                  << "placed_cand,work_complete,lift_cnt,stop_cnt,lower_cnt,placed_cnt,complete_cnt,"
                  << "stamp_ns,beam_valid,beam_high_z_rejected,prepare_ready,"
                  << "left_hook_detected,left_hook_tracked,left_hook_tracking_state,left_hook_lost_frames,"
                  << "left_hook_z,left_ground_valid,left_hook_ground_z,left_ground_points,left_hook_ground_distance,"
                  << "left_hook_beam_dz_valid,left_hook_beam_reference_z,left_hook_beam_dz,left_hook_filtered_z,left_hook_vz,"
                  << "left_window_min,left_window_max,left_body_points,left_candidate_count,left_valid_window_count,"
                  << "left_selection_prediction_z,left_selection_previous_z,left_selection_gate_z,"
                  << "left_prediction_error_z,left_previous_error_z,left_window_overlap,left_candidate_switched,"
                  << "left_selection_reason,left_candidate_details,"
                  << "right_hook_detected,right_hook_tracked,right_hook_tracking_state,right_hook_lost_frames,"
                  << "right_hook_z,right_ground_valid,right_hook_ground_z,right_ground_points,right_hook_ground_distance,"
                  << "right_hook_beam_dz_valid,right_hook_beam_reference_z,right_hook_beam_dz,right_hook_filtered_z,right_hook_vz,"
                  << "right_window_min,right_window_max,right_body_points,right_candidate_count,right_valid_window_count,"
                  << "right_selection_prediction_z,right_selection_previous_z,right_selection_gate_z,"
                  << "right_prediction_error_z,right_previous_error_z,right_window_overlap,right_candidate_switched,"
                  << "right_selection_reason,right_candidate_details,"
                  << "beam_tracking_state,beam_locked,active_left,active_right,left_prepare_ready,right_prepare_ready\n";
          header_written = true;
        }
        const std::string src_str = (res.measurement_source == cx128s2_bev_perception::BeamMeasurementSource::INITIAL_ROI)
            ? "INITIAL_ROI" : "TRACKED_CLUSTER";
        csv_out << std::setprecision(17) << stamp << ","
                << cx128s2_bev_perception::to_string(res.state) << ","
                << res.working_status << ","
                << src_str << ","
                << res.z_beam_raw << "," << res.z_beam_filtered << ","
                << res.z_beam_min << "," << res.z_beam_q1 << "," << res.z_beam_q3 << "," << res.z_beam_max << ","
                << res.z_bg_raw << "," << res.z_bg_filtered << ","
                << res.z_bg_min << "," << res.z_bg_q1 << "," << res.z_bg_q3 << "," << res.z_bg_max << ","
                << res.dz_beam_filtered << "," << res.dz_bg_filtered << ","
                << res.relative_dz_filtered << "," << res.motion_dz << ","
                << res.beam_mad << "," << res.bg_mad << ","
                << res.beam_roi_point_count << "," << res.bg_roi_point_count << "," << res.beam_cluster_point_count << ","
                << res.beam_length_raw << "," << (res.length_locked ? 1 : 0) << ","
                << (res.support_valid ? 1 : 0) << "," << res.support_z << ","
                << res.support_point_count << "," << res.placement_gap_z << ","
                << res.tracking_candidate_count << "," << res.selected_tracking_score << ","
                << (res.placed_candidate ? 1 : 0) << "," << (res.work_complete ? 1 : 0) << ","
                << res.lift_counter << "," << res.stop_counter << "," << res.lower_counter << ","
                << res.placed_counter << "," << res.complete_counter << ","
                << static_cast<int64_t>(msg->header.stamp.sec) * 1000000000LL + msg->header.stamp.nanosec << ","
                << res.beam_z_valid << "," << res.beam_high_z_rejected << "," << res.hook.prepare_ready;
        for (const auto * hook : {&res.hook.left, &res.hook.right}) {
          csv_out << "," << hook->detected << "," << hook->initialized << "," << hook->tracking_state
                  << "," << hook->lost_frames << ",";
          if (hook->detected) csv_out << hook->z;
          csv_out << "," << hook->ground_valid << ",";
          if (hook->ground_valid) csv_out << hook->ground_z;
          csv_out << "," << hook->ground_points << ",";
          if (hook->detected && hook->ground_valid) csv_out << hook->ground_distance;
          csv_out << "," << hook->beam_distance_valid << ",";
          if (hook->beam_distance_valid) csv_out << hook->beam_reference_z;
          csv_out << ",";
          if (hook->beam_distance_valid) csv_out << hook->beam_dz;
          csv_out << ",";
          if (hook->initialized) csv_out << hook->filtered_z;
          csv_out << ",";
          if (hook->initialized) csv_out << hook->vz;
          csv_out << ",";
          if (hook->detected) csv_out << hook->window_z_min;
          csv_out << ",";
          if (hook->detected) csv_out << hook->window_z_max;
          csv_out << "," << hook->body_points << "," << hook->candidate_count
                  << "," << hook->valid_window_count << "," << hook->selection_prediction_z
                  << "," << hook->selection_previous_z << "," << hook->selection_gate_z
                  << "," << hook->prediction_error_z << "," << hook->previous_error_z
                  << "," << hook->previous_window_overlap << "," << hook->candidate_switched
                  << "," << hook->selection_reason << "," << hook->candidate_details;
        }
        csv_out << "," << res.beam_tracking_state << "," << res.length_locked
              << "," << res.active_left << "," << res.active_right
              << "," << res.hook.prepare_sides[0] << "," << res.hook.prepare_sides[1] << "\n";
      }
    }

    // 5. Debug PointClouds
    if (crane_beam_config_.publish_debug && publish_debug_cloud) {
      int dummy = 0;
      publishDebugCloud(out_header, points, crane_beam_detector_->getBeamRoiIndices(),
                        pub_crane_beam_roi_points_, true, dummy);
      publishDebugCloud(out_header, points, crane_beam_detector_->getBgRoiIndices(),
                        pub_crane_beam_bg_roi_points_, true, dummy);
      publishDebugCloud(out_header, points, crane_beam_detector_->getZCandidateIndices(),
                        pub_crane_beam_z_candidates_, true, dummy);
      publishDebugCloud(out_header, points, crane_beam_detector_->getBeamClusterIndices(),
                        pub_crane_beam_cluster_, true, dummy);
      publishDebugCloud(out_header, points, crane_beam_detector_->getSupportIndices(),
                        pub_crane_beam_support_points_, true, dummy);
      if (pub_crane_hook_left_body_) {
        publishDebugCloud(out_header, points, res.hook.left.body_indices,
                          pub_crane_hook_left_body_, true, dummy);
      }
      if (pub_crane_hook_right_body_) {
        publishDebugCloud(out_header, points, res.hook.right.body_indices,
                          pub_crane_hook_right_body_, true, dummy);
      }
    }

    // 6. Debug Markers
    if (crane_beam_config_.publish_debug && pub_crane_beam_markers_) {
      publishCraneBeamMarkers(out_header, res, effective_status);
    }
    if (crane_beam_config_.publish_debug && pub_crane_hook_markers_) {
      const auto * td_res = crane_pipeline_ ? &crane_pipeline_->result().top_down_hook : nullptr;
      const auto * ws_res = crane_pipeline_ ? &crane_pipeline_->result().work_status : nullptr;
      const auto * bm_res = crane_pipeline_ ? &crane_pipeline_->result().beam : nullptr;
      publishCraneHookMarkers(out_header, res.hook, effective_status, td_res, ws_res, bm_res, nullptr);
    }

    total_pipeline_ms_ = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start_time).count();
  }

  void updateShapeGraphDebugStats(const std::vector<ClusterResult> & clusters) {
    last_shape_graph_component_count_ = static_cast<int>(clusters.size());
    last_shape_graph_cluster_count_ = 0;
    last_shape_graph_largest_component_nodes_ = 0;
    last_shape_graph_largest_component_edges_ = 0;
    last_shape_graph_largest_component_points_ = 0;
    last_shape_graph_avg_edge_score_ = 0.0;
    int edge_count_sum = 0;
    double edge_score_sum = 0.0;
    for (const auto & c : clusters) {
      if (c.method == "shape_graph") last_shape_graph_cluster_count_++;
      if (c.shape_node_count > last_shape_graph_largest_component_nodes_) {
        last_shape_graph_largest_component_nodes_ = c.shape_node_count;
        last_shape_graph_largest_component_edges_ = c.shape_edge_count;
        last_shape_graph_largest_component_points_ = c.point_count;
      }
      for (const auto & e : c.shape_edges) {
        edge_score_sum += e.total_score;
        edge_count_sum++;
      }
    }
    last_shape_graph_avg_edge_score_ =
        edge_count_sum > 0 ? edge_score_sum / edge_count_sum : 0.0;
    if (clusterer_ && clusterer_->name() == "shape_graph") {
      last_shape_graph_node_count_ = static_cast<int>(clusterer_->debugShapeNodes().size());
      last_shape_graph_edge_count_ = static_cast<int>(clusterer_->debugShapeEdges().size());
    } else {
      last_shape_graph_node_count_ = 0;
      last_shape_graph_edge_count_ = 0;
    }
    last_shape_graph_endpoint_relation_count_ = 0;
    last_shape_graph_length_relation_found_ = false;
    last_shape_graph_width_relation_found_ = false;
    last_shape_graph_height_relation_found_ = false;
    for (const auto & tr : tracks_) {
      if (!tr.shape_memory.valid) continue;
      last_shape_graph_endpoint_relation_count_ +=
          static_cast<int>(tr.shape_memory.endpoint_relations.size());
      for (const auto & rel : tr.shape_memory.endpoint_relations) {
        if (rel.relation_type == "length") last_shape_graph_length_relation_found_ = true;
        if (rel.relation_type == "width") last_shape_graph_width_relation_found_ = true;
        if (rel.relation_type == "height") last_shape_graph_height_relation_found_ = true;
      }
    }
  }

  void publishShapeGraphMarkers(const std_msgs::msg::Header & header) {
    last_shape_graph_marker_published_ = false;
    if (!publish_shape_graph_markers_ || !pub_shape_graph_markers_ ||
        pub_shape_graph_markers_->get_subscription_count() == 0) {
      return;
    }

    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker clear;
    clear.header = header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear);

    if (shape_graph_compute_policy_ != "manual_prepare") {
      if (!clusterer_ || clusterer_->name() != "shape_graph") {
        pub_shape_graph_markers_->publish(arr);
        return;
      }

      const auto & nodes = clusterer_->debugShapeNodes();
      const auto & edges = clusterer_->debugShapeEdges();
      if (nodes.empty()) {
        pub_shape_graph_markers_->publish(arr);
        return;
      }

      std::unordered_map<int, int> degrees;
      for (const auto & n : nodes) degrees[n.id] = 0;
      for (const auto & e : edges) {
        degrees[e.a]++;
        degrees[e.b]++;
      }

      visualization_msgs::msg::Marker node_marker;
      node_marker.header = header;
      node_marker.ns = "shape_nodes";
      node_marker.id = 1;
      node_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
      node_marker.action = visualization_msgs::msg::Marker::ADD;
      node_marker.scale.x = std::max(0.05, shape_graph_voxel_size_);
      node_marker.scale.y = node_marker.scale.x;
      node_marker.scale.z = node_marker.scale.x;
      node_marker.color.r = 0.2f;
      node_marker.color.g = 0.9f;
      node_marker.color.b = 1.0f;
      node_marker.color.a = 0.75f;
      node_marker.lifetime = rclcpp::Duration::from_seconds(shape_graph_marker_lifetime_);

      visualization_msgs::msg::Marker ep_marker = node_marker;
      ep_marker.ns = "shape_endpoint_nodes";
      ep_marker.id = 3;
      ep_marker.scale.x = std::max(0.05, shape_graph_voxel_size_) * 1.5;
      ep_marker.scale.y = ep_marker.scale.x;
      ep_marker.scale.z = ep_marker.scale.x;
      ep_marker.color.r = 1.0f;
      ep_marker.color.g = 0.2f;
      ep_marker.color.b = 0.2f;
      ep_marker.color.a = 0.85f;

      for (const auto & n : nodes) {
        geometry_msgs::msg::Point p;
        p.x = n.x;
        p.y = n.y;
        p.z = n.z;
        if (degrees[n.id] <= 1) {
          ep_marker.points.push_back(p);
        } else {
          node_marker.points.push_back(p);
        }
      }
      if (!node_marker.points.empty()) arr.markers.push_back(node_marker);
      if (!ep_marker.points.empty()) arr.markers.push_back(ep_marker);

      visualization_msgs::msg::Marker edge_marker;
      edge_marker.header = header;
      edge_marker.ns = "shape_edges";
      edge_marker.id = 2;
      edge_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
      edge_marker.action = visualization_msgs::msg::Marker::ADD;
      edge_marker.scale.x = 0.03;
      edge_marker.color.r = 0.1f;
      edge_marker.color.g = 0.8f;
      edge_marker.color.b = 0.3f;
      edge_marker.color.a = 0.5f;
      edge_marker.lifetime = rclcpp::Duration::from_seconds(shape_graph_marker_lifetime_);

      for (const auto & e : edges) {
        if (e.a < 0 || e.b < 0 || e.a >= static_cast<int>(nodes.size()) ||
            e.b >= static_cast<int>(nodes.size())) {
          continue;
        }
        geometry_msgs::msg::Point pa, pb;
        pa.x = nodes[e.a].x;
        pa.y = nodes[e.a].y;
        pa.z = nodes[e.a].z;
        pb.x = nodes[e.b].x;
        pb.y = nodes[e.b].y;
        pb.z = nodes[e.b].z;
        edge_marker.points.push_back(pa);
        edge_marker.points.push_back(pb);
      }
      if (!edge_marker.points.empty()) arr.markers.push_back(edge_marker);
      last_shape_graph_marker_published_ = true;
      pub_shape_graph_markers_->publish(arr);
      return;
    }

    int id = 100;
    for (const auto & tr : tracks_) {
      if (!physicalOutputAllowed(tr) || tr.observed_shape_cache.state == "empty") continue;
      const auto & cache = tr.observed_shape_cache;

      visualization_msgs::msg::Marker node_marker;
      node_marker.header = header;
      node_marker.ns = "shape_nodes";
      node_marker.id = id++;
      node_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
      node_marker.action = visualization_msgs::msg::Marker::ADD;
      node_marker.scale.x = std::max(0.05, shape_graph_voxel_size_);
      node_marker.scale.y = node_marker.scale.x;
      node_marker.scale.z = node_marker.scale.x;
      node_marker.color.r = 1.0f;
      node_marker.color.g = 0.6f;
      node_marker.color.b = 0.0f;
      node_marker.color.a = 0.85f;
      node_marker.lifetime = rclcpp::Duration::from_seconds(shape_graph_marker_lifetime_);

      for (const auto & cn : cache.nodes) {
        ShapeMemoryNode temp_smn;
        temp_smn.local_x = cn.x;
        temp_smn.local_y = cn.y;
        temp_smn.local_z = cn.z;
        geometry_msgs::msg::Point p = transformMemoryNodeToWorld(tr, temp_smn);
        node_marker.points.push_back(p);
      }
      if (!node_marker.points.empty()) arr.markers.push_back(node_marker);

      visualization_msgs::msg::Marker edge_marker;
      edge_marker.header = header;
      edge_marker.ns = "shape_edges";
      edge_marker.id = id++;
      edge_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
      edge_marker.action = visualization_msgs::msg::Marker::ADD;
      edge_marker.scale.x = 0.03;
      edge_marker.color.r = 1.0f;
      edge_marker.color.g = 0.8f;
      edge_marker.color.b = 0.0f;
      edge_marker.color.a = 0.6f;
      edge_marker.lifetime = rclcpp::Duration::from_seconds(shape_graph_marker_lifetime_);

      for (const auto & ce : cache.edges) {
        const ShapeNode* na = nullptr;
        const ShapeNode* nb = nullptr;
        for (const auto & n : cache.nodes) {
          if (n.id == ce.a) na = &n;
          if (n.id == ce.b) nb = &n;
        }
        if (!na || !nb) continue;
        ShapeMemoryNode temp_smn_a, temp_smn_b;
        temp_smn_a.local_x = na->x;
        temp_smn_a.local_y = na->y;
        temp_smn_a.local_z = na->z;
        temp_smn_b.local_x = nb->x;
        temp_smn_b.local_y = nb->y;
        temp_smn_b.local_z = nb->z;
        geometry_msgs::msg::Point pa = transformMemoryNodeToWorld(tr, temp_smn_a);
        geometry_msgs::msg::Point pb = transformMemoryNodeToWorld(tr, temp_smn_b);
        edge_marker.points.push_back(pa);
        edge_marker.points.push_back(pb);
      }
      if (!edge_marker.points.empty()) arr.markers.push_back(edge_marker);

      visualization_msgs::msg::Marker cache_text;
      cache_text.header = header;
      cache_text.ns = "observed_shape_cache_text_track_" + std::to_string(tr.id);
      cache_text.id = id++;
      cache_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      cache_text.action = visualization_msgs::msg::Marker::ADD;
      cache_text.pose.position.x = outputX(tr);
      cache_text.pose.position.y = outputY(tr);
      cache_text.pose.position.z = tr.z + std::max(1.0, tr.feature.height + 1.2);
      cache_text.pose.orientation.w = 1.0;
      cache_text.scale.z = 0.55;
      cache_text.color.r = 1.0f;
      cache_text.color.g = 0.6f;
      cache_text.color.b = 0.0f;
      cache_text.color.a = 0.95f;
      cache_text.lifetime = rclcpp::Duration::from_seconds(shape_graph_marker_lifetime_);

      std::ostringstream ss;
      ss << "ObservedCache: " << cache.state << " (Track " << tr.id << ")\n"
         << "captured=" << cache.captured_frames << "/" << cache.target_capture_frames << "\n"
         << "nodes=" << cache.nodes.size() << " edges=" << cache.edges.size() << "\n"
         << "confidence=" << std::fixed << std::setprecision(2) << cache.confidence;
      cache_text.text = ss.str();
      arr.markers.push_back(cache_text);

      if (tr.shape_memory.valid && !tr.shape_memory.endpoint_relations.empty()) {
        auto find_mem_node = [&](int node_id) -> const ShapeMemoryNode * {
          for (const auto & n : tr.shape_memory.nodes) {
            if (n.id == node_id) return &n;
          }
          return nullptr;
        };
        auto make_relation_marker = [&](const std::string & ns, int marker_id,
                                        float r, float g, float b) {
          visualization_msgs::msg::Marker m;
          m.header = header;
          m.ns = ns;
          m.id = marker_id;
          m.type = visualization_msgs::msg::Marker::LINE_LIST;
          m.action = visualization_msgs::msg::Marker::ADD;
          m.scale.x = 0.06;
          m.color.r = r;
          m.color.g = g;
          m.color.b = b;
          m.color.a = 0.9f;
          m.lifetime = rclcpp::Duration::from_seconds(shape_graph_marker_lifetime_);
          return m;
        };
        auto length_marker = make_relation_marker("shape_length_relations", id++, 1.0f, 0.1f, 0.1f);
        auto width_marker = make_relation_marker("shape_width_relations", id++, 0.1f, 0.4f, 1.0f);
        auto height_marker = make_relation_marker("shape_height_relations", id++, 0.8f, 0.1f, 1.0f);
        auto structural_marker = make_relation_marker("shape_structural_relations", id++, 0.9f, 0.9f, 0.1f);
        for (const auto & rel : tr.shape_memory.endpoint_relations) {
          const auto * na = find_mem_node(rel.endpoint_a);
          const auto * nb = find_mem_node(rel.endpoint_b);
          if (!na || !nb) continue;
          const auto pa = transformMemoryNodeToWorld(tr, *na);
          const auto pb = transformMemoryNodeToWorld(tr, *nb);
          auto append = [&](visualization_msgs::msg::Marker & m) {
            m.points.push_back(pa);
            m.points.push_back(pb);
          };
          if (rel.relation_type == "length") append(length_marker);
          else if (rel.relation_type == "width") append(width_marker);
          else if (rel.relation_type == "height") append(height_marker);
          else append(structural_marker);
        }
        if (!length_marker.points.empty()) arr.markers.push_back(length_marker);
        if (!width_marker.points.empty()) arr.markers.push_back(width_marker);
        if (!height_marker.points.empty()) arr.markers.push_back(height_marker);
        if (!structural_marker.points.empty()) arr.markers.push_back(structural_marker);
      }

      last_shape_graph_marker_published_ = true;
    }

    pub_shape_graph_markers_->publish(arr);
  }

  void publishShapeMemoryMarkers(const std_msgs::msg::Header & header) {
    if (!publish_shape_memory_markers_ || !pub_shape_memory_markers_ ||
        pub_shape_memory_markers_->get_subscription_count() == 0) {
      return;
    }
    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker clear;
    clear.header = header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear);

    int id = 1;
    for (const auto & tr : tracks_) {
      if (!physicalOutputAllowed(tr) || !tr.shape_memory.valid) continue;
      const ShapeMemory & mem = tr.shape_memory;

      double alpha_scale = tr.lost_frames > 0 ? 0.4 : 1.0;
      double yaw = mem.yaw;

      visualization_msgs::msg::Marker box;
      box.header = header;
      box.ns = "shape_memory_box";
      box.id = id++;
      box.type = visualization_msgs::msg::Marker::CUBE;
      box.action = visualization_msgs::msg::Marker::ADD;
      box.pose.position.x = outputX(tr);
      box.pose.position.y = outputY(tr);
      box.pose.position.z = tr.z;
      box.pose.orientation.x = 0.0;
      box.pose.orientation.y = 0.0;
      box.pose.orientation.z = std::sin(yaw * 0.5);
      box.pose.orientation.w = std::cos(yaw * 0.5);
      box.scale.x = std::max(0.2, mem.length);
      box.scale.y = std::max(0.2, mem.width);
      box.scale.z = std::max(0.2, mem.height);
      box.color.r = mem.committed ? 1.0f : 0.15f;
      box.color.g = mem.frozen ? 0.2f : 0.45f;
      box.color.b = mem.committed ? 0.0f : 1.0f;
      box.color.a = 0.15f * alpha_scale;
      box.lifetime = rclcpp::Duration::from_seconds(shape_graph_marker_lifetime_);
      arr.markers.push_back(box);

      visualization_msgs::msg::Marker node_marker;
      node_marker.header = header;
      node_marker.ns = "shape_memory_nodes";
      node_marker.id = id++;
      node_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
      node_marker.action = visualization_msgs::msg::Marker::ADD;
      node_marker.scale.x = 0.15;
      node_marker.scale.y = 0.15;
      node_marker.scale.z = 0.15;
      node_marker.color.r = mem.committed ? 1.0f : 0.2f;
      node_marker.color.g = mem.committed ? 0.6f : 0.8f;
      node_marker.color.b = mem.committed ? 0.0f : 1.0f;
      node_marker.color.a = 0.7f * alpha_scale;
      node_marker.lifetime = rclcpp::Duration::from_seconds(shape_graph_marker_lifetime_);

      visualization_msgs::msg::Marker ep_marker = node_marker;
      ep_marker.ns = "shape_memory_endpoints";
      ep_marker.id = id++;
      ep_marker.scale.x = 0.25;
      ep_marker.scale.y = 0.25;
      ep_marker.scale.z = 0.25;
      ep_marker.color.r = 1.0f;
      ep_marker.color.g = 0.1f;
      ep_marker.color.b = 0.1f;
      ep_marker.color.a = 0.85f * alpha_scale;

      for (const auto & n : mem.nodes) {
        geometry_msgs::msg::Point p = transformMemoryNodeToWorld(tr, n);
        if (n.is_endpoint) {
          ep_marker.points.push_back(p);
        } else {
          node_marker.points.push_back(p);
        }
      }
      if (!node_marker.points.empty()) arr.markers.push_back(node_marker);
      if (!ep_marker.points.empty()) arr.markers.push_back(ep_marker);

      visualization_msgs::msg::Marker edge_marker;
      edge_marker.header = header;
      edge_marker.ns = "shape_memory_edges";
      edge_marker.id = id++;
      edge_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
      edge_marker.action = visualization_msgs::msg::Marker::ADD;
      edge_marker.scale.x = 0.04;
      edge_marker.color.r = 0.2f;
      edge_marker.color.g = 0.8f;
      edge_marker.color.b = 1.0f;
      edge_marker.color.a = 0.4f * alpha_scale;
      edge_marker.lifetime = rclcpp::Duration::from_seconds(shape_graph_marker_lifetime_);

      for (const auto & e : mem.edges) {
        const ShapeMemoryNode* na = nullptr;
        const ShapeMemoryNode* nb = nullptr;
        for (const auto & n : mem.nodes) {
          if (n.id == e.node_a) na = &n;
          if (n.id == e.node_b) nb = &n;
        }
        if (!na || !nb) continue;

        geometry_msgs::msg::Point pa = transformMemoryNodeToWorld(tr, *na);
        geometry_msgs::msg::Point pb = transformMemoryNodeToWorld(tr, *nb);
        edge_marker.points.push_back(pa);
        edge_marker.points.push_back(pb);
      }
      if (!edge_marker.points.empty()) arr.markers.push_back(edge_marker);

      visualization_msgs::msg::Marker rel_marker;
      rel_marker.header = header;
      rel_marker.ns = "shape_memory_endpoint_relations";
      rel_marker.id = id++;
      rel_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
      rel_marker.action = visualization_msgs::msg::Marker::ADD;
      rel_marker.scale.x = 0.08;
      rel_marker.color.r = 1.0f;
      rel_marker.color.g = 0.5f;
      rel_marker.color.b = 0.0f;
      rel_marker.color.a = 0.7f * alpha_scale;
      rel_marker.lifetime = rclcpp::Duration::from_seconds(shape_graph_marker_lifetime_);

      for (const auto & r : mem.endpoint_relations) {
        const ShapeMemoryNode* na = nullptr;
        const ShapeMemoryNode* nb = nullptr;
        for (const auto & n : mem.nodes) {
          if (n.id == r.endpoint_a) na = &n;
          if (n.id == r.endpoint_b) nb = &n;
        }
        if (!na || !nb) continue;

        geometry_msgs::msg::Point pa = transformMemoryNodeToWorld(tr, *na);
        geometry_msgs::msg::Point pb = transformMemoryNodeToWorld(tr, *nb);
        rel_marker.points.push_back(pa);
        rel_marker.points.push_back(pb);
      }
      if (!rel_marker.points.empty()) arr.markers.push_back(rel_marker);

      visualization_msgs::msg::Marker text;
      text.header = header;
      text.ns = "shape_memory_text";
      text.id = id++;
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;
      text.pose.position.x = outputX(tr);
      text.pose.position.y = outputY(tr);
      text.pose.position.z = tr.z + std::max(1.0, mem.height + 0.6);
      text.pose.orientation.w = 1.0;
      text.scale.z = 0.55;
      text.color.r = 1.0f;
      text.color.g = 1.0f;
      text.color.b = 1.0f;
      text.color.a = 0.95f * alpha_scale;
      text.lifetime = rclcpp::Duration::from_seconds(shape_graph_marker_lifetime_);

      std::string mem_state = "UNCOMMITTED";
      if (mem.committed) {
        if (mem.frozen) {
          mem_state = "FROZEN";
        } else {
          mem_state = "COMMITTED";
        }
      }

      std::ostringstream ss;
      ss << "Memory: " << mem_state << " (Track " << tr.id << ")\n"
         << "nodes=" << mem.nodes.size() << " edges=" << mem.edges.size() << "\n"
         << "endpoint_relations=" << mem.endpoint_relations.size() << "\n"
         << "match_score=" << std::fixed << std::setprecision(2) << tr.feature.shape_memory_match_score;
      text.text = ss.str();
      arr.markers.push_back(text);
    }

    pub_shape_memory_markers_->publish(arr);
  }

  sensor_msgs::msg::PointCloud2 makeCloud(const std_msgs::msg::Header & header,
                                           const std::vector<PointXYZIRT> & points,
                                           const std::vector<int> & indices) const {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = header;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(indices.size());
    sensor_msgs::PointCloud2Modifier mod(cloud);
    if (last_input_has_time_) {
      mod.setPointCloud2Fields(6, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
                               sensor_msgs::msg::PointField::FLOAT32, "z", 1,
                               sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
                               sensor_msgs::msg::PointField::FLOAT32, "ring", 1,
                               sensor_msgs::msg::PointField::UINT16, "time", 1,
                               sensor_msgs::msg::PointField::FLOAT32);
    } else {
      mod.setPointCloud2Fields(5, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
                               sensor_msgs::msg::PointField::FLOAT32, "z", 1,
                               sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
                               sensor_msgs::msg::PointField::FLOAT32, "ring", 1,
                               sensor_msgs::msg::PointField::UINT16);
    }
    mod.resize(indices.size());
    sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> it_i(cloud, "intensity");
    sensor_msgs::PointCloud2Iterator<uint16_t> it_r(cloud, "ring");
    std::unique_ptr<sensor_msgs::PointCloud2Iterator<float>> it_t;
    if (last_input_has_time_) it_t = std::make_unique<sensor_msgs::PointCloud2Iterator<float>>(cloud, "time");
    for (int idx : indices) {
      const auto & p = points[idx];
      *it_x = p.x; ++it_x;
      *it_y = p.y; ++it_y;
      *it_z = p.z; ++it_z;
      *it_i = p.intensity; ++it_i;
      *it_r = p.ring; ++it_r;
      if (it_t) {
        **it_t = p.time;
        ++(*it_t);
      }
    }
    return cloud;
  }

  void publishCloud(const std_msgs::msg::Header & header, const std::vector<PointXYZIRT> & points,
                    const std::vector<int> & indices,
                    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub) const {
    if (!pub || pub->get_subscription_count() == 0) return;
    pub->publish(makeCloud(header, points, indices));
  }

  bool loadRoiPolicy(std::string & err) {
    try {
      YAML::Node root = YAML::LoadFile(roi_policy_file_);
      YAML::Node policy = root["roi_policy"] ? root["roi_policy"] : root;
      if (!policy["zones"] || !policy["zones"].IsSequence()) {
        err = "roi policy has no zones sequence: " + roi_policy_file_;
        return false;
      }
      std::vector<RoiZone> zones;
      for (const auto & node : policy["zones"]) {
        RoiZone zone;
        zone.name = node["name"] ? node["name"].as<std::string>() : "roi";
        zone.type = node["type"] ? node["type"].as<std::string>() : "watch";
        zone.shape = node["shape"] ? node["shape"].as<std::string>() : "polygon";
        zone.enabled = node["enabled"] ? node["enabled"].as<bool>() : true;
        zone.x_min = node["x_min"] ? node["x_min"].as<double>() : 0.0;
        zone.x_max = node["x_max"] ? node["x_max"].as<double>() : 0.0;
        zone.y_min = node["y_min"] ? node["y_min"].as<double>() : 0.0;
        zone.y_max = node["y_max"] ? node["y_max"].as<double>() : 0.0;
        if (node["polygon_x"]) zone.polygon_x = node["polygon_x"].as<std::vector<double>>();
        if (node["polygon_y"]) zone.polygon_y = node["polygon_y"].as<std::vector<double>>();
        if (zone.type != "watch" && zone.type != "ignore" && zone.type != "hard_ignore") {
          RCLCPP_WARN(get_logger(), "Skipping ROI zone '%s' with invalid type '%s'.",
                      zone.name.c_str(), zone.type.c_str());
          continue;
        }
        if (zone.shape != "box" && zone.shape != "polygon") {
          RCLCPP_WARN(get_logger(), "Skipping ROI zone '%s' with invalid shape '%s'.",
                      zone.name.c_str(), zone.shape.c_str());
          continue;
        }
        zones.push_back(zone);
      }
      roi_zones_ = zones;
      visibility_model_.clearStaticCache();
      visibility_prewarm_pending_ = true;
      RCLCPP_INFO(get_logger(), "Loaded ROI policy: file=%s zones=%zu",
                  roi_policy_file_.c_str(), roi_zones_.size());
      return true;
    } catch (const std::exception & ex) {
      err = ex.what();
      return false;
    }
  }

  bool saveRoiPolicy(std::string & err) const {
    try {
      YAML::Emitter out;
      out << YAML::BeginMap;
      out << YAML::Key << "roi_policy" << YAML::Value << YAML::BeginMap;
      out << YAML::Key << "version" << YAML::Value << 1;
      out << YAML::Key << "frame_id" << YAML::Value
          << (last_target_frame_.empty() ? target_frame_ : last_target_frame_);
      out << YAML::Key << "zones" << YAML::Value << YAML::BeginSeq;
      for (const auto & zone : roi_zones_) {
        out << YAML::BeginMap;
        out << YAML::Key << "name" << YAML::Value << zone.name;
        out << YAML::Key << "type" << YAML::Value << zone.type;
        out << YAML::Key << "shape" << YAML::Value << zone.shape;
        out << YAML::Key << "enabled" << YAML::Value << zone.enabled;
        if (zone.shape == "box") {
          out << YAML::Key << "x_min" << YAML::Value << zone.x_min;
          out << YAML::Key << "x_max" << YAML::Value << zone.x_max;
          out << YAML::Key << "y_min" << YAML::Value << zone.y_min;
          out << YAML::Key << "y_max" << YAML::Value << zone.y_max;
        } else {
          out << YAML::Key << "polygon_x" << YAML::Value << YAML::Flow << zone.polygon_x;
          out << YAML::Key << "polygon_y" << YAML::Value << YAML::Flow << zone.polygon_y;
        }
        out << YAML::EndMap;
      }
      out << YAML::EndSeq;
      out << YAML::EndMap;
      out << YAML::EndMap;
      std::ofstream f(roi_policy_file_);
      if (!f.is_open()) {
        err = "cannot open " + roi_policy_file_;
        return false;
      }
      f << out.c_str() << "\n";
      return true;
    } catch (const std::exception & ex) {
      err = ex.what();
      return false;
    }
  }

  bool saveBackground(std::string & err) const {
    std::ofstream f(background_map_file_);
    if (!f.is_open()) {
      err = "cannot open " + background_map_file_;
      return false;
    }
    f << "frame_id " << background_frame_id_ << "\n";
    f << "resolution " << bg_x_res_ << " " << bg_y_res_ << "\n";
    for (const auto & kv : background_map_) {
      const auto & c = kv.second;
      f << "cell " << kv.first << " " << c.ix << " " << c.iy << " " << c.center_x << " "
        << c.center_y << " " << c.occupancy_probability << " " << c.seen_count << " "
        << c.ground_z << " " << c.mean_z << " " << c.std_z << " " << c.max_z << " "
        << c.min_z << " " << c.max_density << " " << c.static_score << " "
        << c.last_seen_time << " " << c.valid << "\n";
    }
    return true;
  }

  bool loadBackground(std::string & err) {
    std::ifstream f(background_map_file_);
    if (!f.is_open()) {
      err = "cannot open " + background_map_file_;
      return false;
    }
    std::string tag;
    background_map_.clear();
    bool loaded_background_header = false;
    while (f >> tag) {
      if (tag == "frame_id") {
        f >> background_frame_id_;
        loaded_background_header = true;
      } else if (tag == "resolution") {
        double rx = 0.0, ry = 0.0;
        f >> rx >> ry;
        if (std::abs(rx - bg_x_res_) > 1e-6 || std::abs(ry - bg_y_res_) > 1e-6) {
          RCLCPP_WARN(get_logger(), "Loaded background resolution differs from current parameters.");
        }
      } else if (tag == "cell") {
        int64_t key;
        BackgroundCell c;
        f >> key >> c.ix >> c.iy >> c.center_x >> c.center_y >> c.occupancy_probability >>
            c.seen_count >> c.ground_z >> c.mean_z >> c.std_z >> c.max_z >> c.min_z;
        if (!(f >> c.max_density >> c.static_score >> c.last_seen_time >> c.valid)) {
          f.clear();
          c.max_density = normalPdf(c.mean_z, c.mean_z, std::max(gdg_min_std_z_, c.std_z));
          c.static_score = c.occupancy_probability;
          c.last_seen_time = now().seconds();
          c.valid = true;
        }
        if (c.valid) background_map_[key] = c;
      }
    }
    background_ready_ = loaded_background_header;
    return background_ready_;
  }

  void handleLearnBackground(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    if (!usesStaticBackground()) {
      res->success = false;
      res->message = "background_mode=ground_only: static background learning is disabled";
      return;
    }
    learning_accum_.clear();
    learned_frame_count_ = 0;
    learning_start_time_ = -1.0;
    learning_active_ = true;
    background_ready_ = false;
    resetTrackingState();
    res->success = true;
    res->message = "static background learning started";
  }

  void handleSaveBackground(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    if (!usesStaticBackground()) {
      res->success = false;
      res->message = "background_mode=ground_only: static background is disabled";
      return;
    }
    std::string err;
    res->success = saveBackground(err);
    res->message = res->success ? "background saved" : err;
  }

  void handleLoadBackground(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    if (!usesStaticBackground()) {
      res->success = false;
      res->message = "background_mode=ground_only: static background is disabled";
      return;
    }
    std::string err;
    res->success = loadBackground(err);
    res->message = res->success ? "background loaded" : err;
  }

  void handleClearBackground(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    if (!usesStaticBackground()) {
      res->success = false;
      res->message = "background_mode=ground_only: static background is disabled";
      return;
    }
    background_map_.clear();
    learning_accum_.clear();
    background_ready_ = false;
    learning_active_ = false;
    resetTrackingState();
    res->success = true;
    res->message = "background cleared";
  }

  void handleDumpBackground(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    if (!usesStaticBackground()) {
      res->success = false;
      res->message = "background_mode=ground_only: static background is disabled";
      return;
    }
    res->success = writeBackgroundCsv(background_table_file_);
    res->message = res->success ? background_table_file_ : "failed to write background csv";
  }

  void handleDumpTrack(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    res->success = writeTrackCsv(track_table_file_);
    res->message = res->success ? track_table_file_ : "failed to write track csv";
  }

  void handleDumpCandidate(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    res->success = writeCandidateCsv(candidate_table_file_);
    res->message = res->success ? candidate_table_file_ : "failed to write candidate csv";
  }

  void handlePrepareShapeMemoryCommit(
      const std::shared_ptr<cx128s2_bev_perception::srv::PrepareShapeMemoryCommit::Request> req,
      std::shared_ptr<cx128s2_bev_perception::srv::PrepareShapeMemoryCommit::Response> res) {
    int target_id = req->track_id;
    int cf = req->capture_frames;
    if (cf < 1) cf = 1;

    prepare_shape_memory_requested_count_++;

    for (auto & tr : tracks_) {
      if (tr.id == target_id) {
        tr.observed_shape_cache.valid = false;
        tr.observed_shape_cache.track_id = tr.id;
        tr.observed_shape_cache.state = "capturing";
        tr.observed_shape_cache.captured_frames = 0;
        tr.observed_shape_cache.target_capture_frames = cf;
        tr.observed_shape_cache.accumulated_local_points.clear();
        tr.observed_shape_cache.nodes.clear();
        tr.observed_shape_cache.edges.clear();
        tr.observed_shape_cache.confidence = 0.0;
        tr.observed_shape_cache.start_frame = frame_id_;
        tr.observed_shape_cache.last_update_frame = frame_id_;

        prepare_shape_memory_success_count_++;

        res->success = true;
        res->message = "Preparing shape memory commit for track " + std::to_string(target_id) +
                       " over " + std::to_string(cf) + " frames.";
        return;
      }
    }

    res->success = false;
    res->message = "Track ID " + std::to_string(target_id) + " not found.";
  }

  void handleCommitShapeMemory(
      const std::shared_ptr<cx128s2_bev_perception::srv::CommitShapeMemory::Request> req,
      std::shared_ptr<cx128s2_bev_perception::srv::CommitShapeMemory::Response> res) {
    int target_id = req->track_id;

    for (auto & tr : tracks_) {
      if (tr.id == target_id) {
        if (shape_graph_compute_policy_ == "manual_prepare") {
          if (!tr.observed_shape_cache.valid || tr.observed_shape_cache.state != "ready") {
            res->success = false;
            res->message = "track " + std::to_string(target_id) + " has no prepared observed shape graph. Call /prepare_shape_memory_commit first.";
            return;
          }

          const auto & cache = tr.observed_shape_cache;
          if (static_cast<int>(cache.nodes.size()) < shape_memory_commit_min_nodes_) {
            res->success = false;
            res->message = "Prepared cache for track " + std::to_string(target_id) + " has only " +
                           std::to_string(cache.nodes.size()) + " shape nodes; min required is " +
                           std::to_string(shape_memory_commit_min_nodes_);
            return;
          }

          if (static_cast<int>(cache.edges.size()) < shape_memory_commit_min_edges_) {
            res->success = false;
            res->message = "Prepared cache for track " + std::to_string(target_id) + " has only " +
                           std::to_string(cache.edges.size()) + " shape edges; min required is " +
                           std::to_string(shape_memory_commit_min_edges_);
            return;
          }

          double confidence = cache.confidence;
          if (confidence < shape_memory_commit_min_confidence_) {
            res->success = false;
            res->message = "Prepared cache for track " + std::to_string(target_id) + " shape confidence " +
                           std::to_string(confidence) + " is below min threshold " +
                           std::to_string(shape_memory_commit_min_confidence_);
            return;
          }

          ShapeMemory mem;
          mem.valid = true;
          mem.committed = true;
          mem.frozen = req->freeze_after_commit;
          mem.track_id = tr.id;
          mem.length = tr.feature.length;
          mem.width = tr.feature.width;
          mem.height = tr.feature.height;
          mem.center_x = tr.x;
          mem.center_y = tr.y;
          mem.center_z = tr.z;
          mem.yaw = getTrackYaw(tr);
          
          double min_lx = std::numeric_limits<double>::max();
          double max_lx = std::numeric_limits<double>::lowest();
          double min_ly = std::numeric_limits<double>::max();
          double max_ly = std::numeric_limits<double>::lowest();
          double min_lz = std::numeric_limits<double>::max();
          double max_lz = std::numeric_limits<double>::lowest();
          for (const auto & n : cache.nodes) {
            min_lx = std::min(min_lx, n.x);
            max_lx = std::max(max_lx, n.x);
            min_ly = std::min(min_ly, n.y);
            max_ly = std::max(max_ly, n.y);
            min_lz = std::min(min_lz, n.z);
            max_lz = std::max(max_lz, n.z);
          }
          mem.min_local_x = min_lx;
          mem.max_local_x = max_lx;
          mem.min_local_y = min_ly;
          mem.max_local_y = max_ly;
          mem.min_local_z = min_lz;
          mem.max_local_z = max_lz;
          mem.confidence = confidence;
          mem.update_count = 1;
          mem.commit_frame = frame_id_;
          mem.class_label = req->class_label.empty() ? "UNKNOWN" : req->class_label;
          mem.source_component_type = tr.feature.component_type;
          mem.source_component_type_confidence = tr.feature.component_type_confidence;
          mem.source_component_type_reason = tr.feature.component_type_reason;
          mem.applicable_component_types = req->applicable_component_types.empty() ?
              defaultApplicableComponentTypesForClass(mem.class_label, mem.source_component_type) :
              req->applicable_component_types;
          mem.learned_avg_speed = std::hypot(tr.vx, tr.vy);
          mem.learned_motion_yaw = getTrackMotionYaw(tr);
          const double range = std::hypot(tr.x, tr.y);
          mem.learned_radial_velocity = range > 1e-3 ? (tr.x * tr.vx + tr.y * tr.vy) / range : 0.0;
          mem.learned_motion_samples = 1;

          buildShapeMemoryFromCache(mem, cache);
          mem.top_area = std::max(0.0, mem.max_local_x - mem.min_local_x) *
                         std::max(0.0, mem.max_local_y - mem.min_local_y);
          mem.memory_confidence = computeShapeMemoryConfidence(mem, cache, tr.feature);
          mem.confidence = mem.memory_confidence;
          computeShapeMemoryClassPrior(mem, tr.feature);
          augmentShapeMemoryWithMirrorMode(mem);
          selectEndpointsAndRelations(mem);
          tr.shape_memory = mem;
          addObjectMemoryEntryFromTrack(tr);
          committed_shape_memory_track_count_++;
          const size_t node_count = mem.nodes.size();
          const size_t edge_count = mem.edges.size();
          const size_t relation_count = mem.endpoint_relations.size();

          if (shape_memory_clear_observed_cache_after_commit_) {
            clearObservedShapeCache(tr.observed_shape_cache);
          }

          res->success = true;
          res->message = "Shape memory committed for track " + std::to_string(target_id) +
                         ". ClassLabel: " + mem.class_label +
                         ", ClassPrior: " + mem.class_prior +
                         ", MemoryConfidence: " + std::to_string(mem.memory_confidence) +
                         ", Nodes: " + std::to_string(node_count) +
                         ", Edges: " + std::to_string(edge_count) +
                         ", Relations: " + std::to_string(relation_count) +
                         (mem.frozen ? " (FROZEN)" : "");
          return;
        } else {
          // fallback to always mode behavior
          if (tr.feature.shape_nodes.empty()) {
            res->success = false;
            res->message = "Track " + std::to_string(target_id) + " has no observed shape graph";
            return;
          }

          if (static_cast<int>(tr.feature.shape_nodes.size()) < shape_memory_commit_min_nodes_) {
            res->success = false;
            res->message = "Track " + std::to_string(target_id) + " has only " +
                           std::to_string(tr.feature.shape_nodes.size()) + " shape nodes; min required is " +
                           std::to_string(shape_memory_commit_min_nodes_);
            return;
          }

          if (static_cast<int>(tr.feature.shape_edges.size()) < shape_memory_commit_min_edges_) {
            res->success = false;
            res->message = "Track " + std::to_string(target_id) + " has only " +
                           std::to_string(tr.feature.shape_edges.size()) + " shape edges; min required is " +
                           std::to_string(shape_memory_commit_min_edges_);
            return;
          }

          double confidence = tr.feature.shape_graph_score;
          if (confidence < shape_memory_commit_min_confidence_) {
            res->success = false;
            res->message = "Track " + std::to_string(target_id) + " shape confidence " +
                           std::to_string(confidence) + " is below min threshold " +
                           std::to_string(shape_memory_commit_min_confidence_);
            return;
          }

          ShapeMemory mem;
          mem.valid = true;
          mem.committed = true;
          mem.frozen = req->freeze_after_commit;
          mem.track_id = tr.id;
          mem.length = tr.feature.length;
          mem.width = tr.feature.width;
          mem.height = tr.feature.height;
          mem.center_x = tr.x;
          mem.center_y = tr.y;
          mem.center_z = tr.z;
          mem.yaw = getTrackYaw(tr);
          mem.min_local_x = tr.feature.min_x - tr.feature.centroid_x;
          mem.max_local_x = tr.feature.max_x - tr.feature.centroid_x;
          mem.min_local_y = tr.feature.min_y - tr.feature.centroid_y;
          mem.max_local_y = tr.feature.max_y - tr.feature.centroid_y;
          mem.min_local_z = tr.feature.min_z - tr.feature.centroid_z;
          mem.max_local_z = tr.feature.max_z - tr.feature.centroid_z;
          mem.confidence = confidence;
          mem.update_count = 1;
          mem.commit_frame = frame_id_;
          mem.class_label = req->class_label.empty() ? "UNKNOWN" : req->class_label;
          mem.source_component_type = tr.feature.component_type;
          mem.source_component_type_confidence = tr.feature.component_type_confidence;
          mem.source_component_type_reason = tr.feature.component_type_reason;
          mem.applicable_component_types = req->applicable_component_types.empty() ?
              defaultApplicableComponentTypesForClass(mem.class_label, mem.source_component_type) :
              req->applicable_component_types;
          mem.learned_avg_speed = std::hypot(tr.vx, tr.vy);
          mem.learned_motion_yaw = getTrackMotionYaw(tr);
          const double range = std::hypot(tr.x, tr.y);
          mem.learned_radial_velocity = range > 1e-3 ? (tr.x * tr.vx + tr.y * tr.vy) / range : 0.0;
          mem.learned_motion_samples = 1;

          buildShapeMemoryFromObservedFeature(mem, tr.feature);
          mem.top_area = tr.feature.shape_area_top > 0.0 ? tr.feature.shape_area_top :
              std::max(0.0, mem.max_local_x - mem.min_local_x) *
              std::max(0.0, mem.max_local_y - mem.min_local_y);
          ObservedShapeGraphCache pseudo_cache;
          pseudo_cache.valid = true;
          pseudo_cache.state = "ready";
          pseudo_cache.captured_frames = 1;
          pseudo_cache.target_capture_frames = 1;
          pseudo_cache.nodes = tr.feature.shape_nodes;
          pseudo_cache.edges = tr.feature.shape_edges;
          pseudo_cache.confidence = confidence;
          mem.memory_confidence = computeShapeMemoryConfidence(mem, pseudo_cache, tr.feature);
          mem.confidence = mem.memory_confidence;
          computeShapeMemoryClassPrior(mem, tr.feature);
          if (!shape_memory_mirror_only_after_prepare_ready_) {
            augmentShapeMemoryWithMirrorMode(mem);
            selectEndpointsAndRelations(mem);
          }
          tr.shape_memory = mem;
          addObjectMemoryEntryFromTrack(tr);
          committed_shape_memory_track_count_++;

          res->success = true;
          res->message = "Shape memory committed for track " + std::to_string(target_id) +
                         ". ClassLabel: " + mem.class_label +
                         ", ClassPrior: " + mem.class_prior +
                         ", MemoryConfidence: " + std::to_string(mem.memory_confidence) +
                         ", Nodes: " + std::to_string(mem.nodes.size()) +
                         ", Edges: " + std::to_string(mem.edges.size()) +
                         ", Relations: " + std::to_string(mem.endpoint_relations.size()) +
                         (mem.frozen ? " (FROZEN)" : "");
          return;
        }
      }
    }

    res->success = false;
    res->message = "Track ID " + std::to_string(target_id) + " not found.";
  }

  void handleClearShapeMemory(
      const std::shared_ptr<cx128s2_bev_perception::srv::ControlShapeMemory::Request> req,
      std::shared_ptr<cx128s2_bev_perception::srv::ControlShapeMemory::Response> res) {
    int target_id = req->track_id;
    int count = 0;

    for (auto & tr : tracks_) {
      if (target_id < 0 || tr.id == target_id) {
        tr.shape_memory = ShapeMemory();
        count++;
        if (target_id >= 0) break;
      }
    }

    if (count > 0) {
      res->success = true;
      res->message = "Cleared shape memory for " + std::to_string(count) + " tracks.";
    } else {
      res->success = false;
      res->message = target_id < 0 ? "No active tracks found to clear."
                                   : "Track ID " + std::to_string(target_id) + " not found.";
    }
  }

  void handleFreezeShapeMemory(
      const std::shared_ptr<cx128s2_bev_perception::srv::ControlShapeMemory::Request> req,
      std::shared_ptr<cx128s2_bev_perception::srv::ControlShapeMemory::Response> res) {
    int target_id = req->track_id;
    int count = 0;

    for (auto & tr : tracks_) {
      if (target_id < 0 || tr.id == target_id) {
        if (tr.shape_memory.valid) {
          tr.shape_memory.frozen = true;
          count++;
        }
        if (target_id >= 0) break;
      }
    }

    if (count > 0) {
      res->success = true;
      res->message = "Frozen shape memory for " + std::to_string(count) + " tracks.";
    } else {
      res->success = false;
      res->message = target_id < 0 ? "No tracks with valid shape memory found to freeze."
                                   : "Track ID " + std::to_string(target_id) + " has no valid shape memory.";
    }
  }

  void handleUnfreezeShapeMemory(
      const std::shared_ptr<cx128s2_bev_perception::srv::ControlShapeMemory::Request> req,
      std::shared_ptr<cx128s2_bev_perception::srv::ControlShapeMemory::Response> res) {
    int target_id = req->track_id;
    int count = 0;

    for (auto & tr : tracks_) {
      if (target_id < 0 || tr.id == target_id) {
        if (tr.shape_memory.valid) {
          tr.shape_memory.frozen = false;
          count++;
        }
        if (target_id >= 0) break;
      }
    }

    if (count > 0) {
      res->success = true;
      res->message = "Unfrozen shape memory for " + std::to_string(count) + " tracks.";
    } else {
      res->success = false;
      res->message = target_id < 0 ? "No tracks with valid shape memory found to unfreeze."
                                   : "Track ID " + std::to_string(target_id) + " has no valid shape memory.";
    }
  }

  bool dumpSingleShapeMemory(const Track & tr, std::string & err_msg) const {
    const auto & mem = tr.shape_memory;
    int target_id = tr.id;

    std::string yaml_path = "/tmp/cx128s2_shape_memory_track_" + std::to_string(target_id) + ".yaml";
    std::ofstream f_yaml(yaml_path);
    if (!f_yaml.is_open()) {
      err_msg = "Failed to open YAML file: " + yaml_path;
      return false;
    }

    f_yaml << "track_id: " << target_id << "\n"
           << "committed: " << (mem.committed ? "true" : "false") << "\n"
           << "frozen: " << (mem.frozen ? "true" : "false") << "\n"
           << "center: [" << mem.center_x << ", " << mem.center_y << ", " << mem.center_z << "]\n"
           << "yaw: " << mem.yaw << "\n"
           << "dimensions: [" << mem.length << ", " << mem.width << ", " << mem.height << "]\n"
           << "nodes_count: " << mem.nodes.size() << "\n"
           << "edges_count: " << mem.edges.size() << "\n"
           << "endpoint_relations_count: " << mem.endpoint_relations.size() << "\n"
           << "confidence: " << mem.confidence << "\n"
           << "class_label: " << mem.class_label << "\n"
           << "class_prior: " << mem.class_prior << "\n"
           << "class_confidence: " << mem.class_confidence << "\n"
           << "vehicle_like_score: " << mem.vehicle_like_score << "\n"
           << "truck_like_score: " << mem.truck_like_score << "\n"
           << "memory_confidence: " << mem.memory_confidence << "\n"
           << "learned_avg_speed: " << mem.learned_avg_speed << "\n"
           << "learned_motion_yaw: " << mem.learned_motion_yaw << "\n"
           << "learned_radial_velocity: " << mem.learned_radial_velocity << "\n"
           << "learned_motion_samples: " << mem.learned_motion_samples << "\n"
           << "reprepare_needed: " << (mem.reprepare_needed ? "true" : "false") << "\n"
           << "mirror_mode:\n"
           << "  enabled: " << (mem.mirror_info.enabled ? "true" : "false") << "\n"
           << "  plane: " << mem.mirror_info.plane << "\n"
           << "  flipped_axis: " << mem.mirror_info.flipped_axis << "\n"
           << "  center_x: " << mem.mirror_info.center_x << "\n"
           << "  center_y: " << mem.mirror_info.center_y << "\n"
           << "  center_z: " << mem.mirror_info.center_z << "\n"
           << "  flipped_axis_center: " << mem.mirror_info.flipped_axis_center << "\n"
           << "  mirror_axis: " << mem.mirror_info.mirror_axis << "\n"
           << "  mirror_centerline: " << mem.mirror_info.mirror_centerline << "\n"
           << "  mirror_centerline_x: " << mem.mirror_info.mirror_centerline_x << "\n"
           << "  mirror_centerline_y: " << mem.mirror_info.mirror_centerline_y << "\n"
           << "  mirror_centerline_z: " << mem.mirror_info.mirror_centerline_z << "\n"
           << "  observed_node_count: " << mem.mirror_info.observed_node_count << "\n"
           << "  mirrored_node_count: " << mem.mirror_info.mirrored_node_count << "\n"
           << "  observed_edge_count: " << mem.mirror_info.observed_edge_count << "\n"
           << "  mirrored_edge_count: " << mem.mirror_info.mirrored_edge_count << "\n"
           << "  near_identical_warning_count: " << mem.mirror_info.near_identical_warning_count << "\n"
           << "  invalid_edge_warning_count: " << mem.mirror_info.invalid_edge_warning_count << "\n"
           << "  symmetry_confidence: " << mem.mirror_info.symmetry_confidence << "\n"
           << "endpoint_relations:\n";
    for (const auto & rel : mem.endpoint_relations) {
      f_yaml << "  - relation_type: " << rel.relation_type << "\n"
             << "    endpoint_a: " << rel.endpoint_a << "\n"
             << "    endpoint_b: " << rel.endpoint_b << "\n"
             << "    expected_distance: " << rel.expected_distance << "\n"
             << "    expected_direction_yaw: " << rel.expected_direction_yaw << "\n"
             << "    expected_height_delta: " << rel.expected_height_delta << "\n"
             << "    confidence: " << rel.confidence << "\n"
             << "    relation_score: " << rel.relation_score << "\n"
             << "    uses_virtual_node: " << (rel.uses_virtual_node ? "true" : "false") << "\n"
             << "    uses_mirrored_node: " << (rel.uses_mirrored_node ? "true" : "false") << "\n";
    }
    f_yaml.close();

    std::string nodes_path = "/tmp/cx128s2_shape_memory_track_" + std::to_string(target_id) + "_nodes.csv";
    std::ofstream f_nodes(nodes_path);
    if (f_nodes.is_open()) {
      f_nodes << "node_id,local_x,local_y,local_z,point_count,confidence,is_endpoint\n";
      for (const auto & n : mem.nodes) {
        f_nodes << n.id << "," << n.local_x << "," << n.local_y << "," << n.local_z << ","
                << n.point_count << "," << n.confidence << "," << (n.is_endpoint ? 1 : 0) << "\n";
      }
      f_nodes.close();
    }

    std::string edges_path = "/tmp/cx128s2_shape_memory_track_" + std::to_string(target_id) + "_edges.csv";
    std::ofstream f_edges(edges_path);
    if (f_edges.is_open()) {
      f_edges << "edge_id,node_a,node_b,local_dx,local_dy,local_dz,length,direction_yaw,height_delta,confidence,is_structural\n";
      for (const auto & e : mem.edges) {
        f_edges << e.id << "," << e.node_a << "," << e.node_b << "," << e.local_dx << ","
                << e.local_dy << "," << e.local_dz << "," << e.length << "," << e.direction_yaw
                << "," << e.height_delta << "," << e.confidence << "," << (e.is_structural_edge ? 1 : 0) << "\n";
      }
      f_edges.close();
    }

    std::string rels_path = "/tmp/cx128s2_shape_memory_track_" + std::to_string(target_id) + "_endpoint_relations.csv";
    std::ofstream f_rels(rels_path);
    if (f_rels.is_open()) {
      f_rels << "relation_type,endpoint_a,endpoint_b,expected_distance,expected_direction_yaw,expected_height_delta,confidence,relation_score,uses_virtual_node,uses_mirrored_node\n";
      for (const auto & r : mem.endpoint_relations) {
        f_rels << r.relation_type << "," << r.endpoint_a << "," << r.endpoint_b << ","
                << r.expected_distance << "," << r.expected_direction_yaw << ","
                << r.expected_height_delta << "," << r.confidence << ","
                << r.relation_score << "," << r.uses_virtual_node << ","
                << r.uses_mirrored_node << "\n";
      }
      f_rels.close();
    }
    return true;
  }

  std::string makeObjectMemoryId(const ShapeMemory & mem) const {
    const std::string label = mem.class_label != "UNKNOWN" ? mem.class_label : mem.class_prior;
    std::ostringstream ss;
    ss << (label.empty() ? "UNKNOWN" : label) << "_" << std::setw(4) << std::setfill('0')
       << (object_memory_bank_.entries.size() + 1);
    return ss.str();
  }

  std::string distanceProfileForRange(double range) const {
    if (range < near_range_max_) return "near";
    if (range < mid_range_max_) return "mid";
    return "long";
  }

  ObjectMemoryEntry makeObjectMemoryEntryFromTrack(const Track & tr, ShapeMemory mem) const {
    ObjectMemoryEntry entry;
    entry.memory_id = makeObjectMemoryId(mem);
    entry.class_label = mem.class_label;
    entry.class_prior = mem.class_prior;
    entry.source_component_type = mem.source_component_type;
    entry.source_component_type_confidence = mem.source_component_type_confidence;
    entry.source_component_type_reason = mem.source_component_type_reason;
    entry.applicable_component_types = mem.applicable_component_types.empty() ?
        defaultApplicableComponentTypesForClass(mem.class_label, mem.source_component_type) :
        mem.applicable_component_types;
    entry.distance_profile = distanceProfileForRange(trackSensorRange(tr));
    MirrorModeInfo mirror_info;
    if (mem.mirror_info.enabled) {
      mirror_info = mem.mirror_info;
    } else if (!shape_memory_mirror_only_after_prepare_ready_) {
      augmentShapeMemoryWithMirrorMode(mem, &mirror_info);
    }
    entry.shape_memory = mem;
    entry.height_profile =
        computeHeightProfileFromIndices(std::vector<PointXYZIRT>{}, std::vector<int>{}, tr.x, tr.feature.min_x, tr.feature.max_x);
    entry.height_profile.front_height = tr.feature.max_z;
    entry.height_profile.middle_height = tr.feature.max_z;
    entry.height_profile.rear_height = tr.feature.max_z;
    entry.height_profile.max_height = tr.feature.max_z;
    entry.height_profile.height_variance = 0.0;
    entry.height_profile.front_rear_height_delta = 0.0;
    entry.mirror_info = mirror_info;
    entry.shape_memory.mirror_info = mirror_info;
    entry.shape_memory.source_component_type = entry.source_component_type;
    entry.shape_memory.source_component_type_confidence = entry.source_component_type_confidence;
    entry.shape_memory.source_component_type_reason = entry.source_component_type_reason;
    entry.shape_memory.applicable_component_types = entry.applicable_component_types;
    entry.memory_confidence = mem.memory_confidence > 0.0 ? mem.memory_confidence : mem.confidence;
    entry.class_confidence = mem.class_confidence;
    entry.observed_count = 1;
    entry.created_time = now().seconds();
    entry.last_matched_time = 0.0;
    return entry;
  }

  void addObjectMemoryEntryFromTrack(const Track & tr) {
    if (!object_memory_bank_enabled_ || !tr.shape_memory.valid) return;
    ObjectMemoryEntry entry = makeObjectMemoryEntryFromTrack(tr, tr.shape_memory);
    object_memory_bank_.entries.push_back(entry);
    class_memory_bank_[entry.class_label != "UNKNOWN" ? entry.class_label : entry.class_prior].push_back(entry.shape_memory);
    if (auto_save_object_memory_bank_after_commit_) {
      std::string err;
      if (!saveObjectMemoryBank(err)) {
        RCLCPP_WARN(get_logger(), "Object memory bank auto-save failed: %s", err.c_str());
      }
    }
  }

  bool saveObjectMemoryBank(std::string & err) const {
    try {
      YAML::Emitter out;
      out << YAML::BeginMap;
      out << YAML::Key << "version" << YAML::Value << 1;
      out << YAML::Key << "frame_id" << YAML::Value << target_frame_;
      out << YAML::Key << "memory_bank" << YAML::Value << YAML::BeginSeq;
      for (const auto & entry : object_memory_bank_.entries) {
        const auto & mem = entry.shape_memory;
        out << YAML::BeginMap;
        out << YAML::Key << "memory_id" << YAML::Value << entry.memory_id;
        out << YAML::Key << "class_label" << YAML::Value << entry.class_label;
        out << YAML::Key << "class_prior" << YAML::Value << entry.class_prior;
        out << YAML::Key << "source_component_type" << YAML::Value << entry.source_component_type;
        out << YAML::Key << "source_component_type_confidence" << YAML::Value
            << entry.source_component_type_confidence;
        out << YAML::Key << "source_component_type_reason" << YAML::Value
            << entry.source_component_type_reason;
        out << YAML::Key << "applicable_component_types" << YAML::Value
            << YAML::Flow << entry.applicable_component_types;
        out << YAML::Key << "distance_profile" << YAML::Value << entry.distance_profile;
        out << YAML::Key << "memory_confidence" << YAML::Value << entry.memory_confidence;
        out << YAML::Key << "class_confidence" << YAML::Value << entry.class_confidence;
        out << YAML::Key << "observed_count" << YAML::Value << entry.observed_count;
        out << YAML::Key << "matched_count" << YAML::Value << entry.matched_count;
        out << YAML::Key << "false_match_count" << YAML::Value << entry.false_match_count;
        out << YAML::Key << "created_time" << YAML::Value << entry.created_time;
        out << YAML::Key << "last_matched_time" << YAML::Value << entry.last_matched_time;

        out << YAML::Key << "shape_memory" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "center" << YAML::Value << YAML::Flow
            << std::vector<double>{mem.center_x, mem.center_y, mem.center_z};
        out << YAML::Key << "yaw" << YAML::Value << mem.yaw;
        out << YAML::Key << "dimensions" << YAML::Value << YAML::Flow
            << std::vector<double>{mem.length, mem.width, mem.height};
        out << YAML::Key << "local_bounds" << YAML::Value << YAML::Flow
            << std::vector<double>{mem.min_local_x, mem.max_local_x, mem.min_local_y,
                                   mem.max_local_y, mem.min_local_z, mem.max_local_z};
        out << YAML::Key << "confidence" << YAML::Value << mem.confidence;
        out << YAML::Key << "committed" << YAML::Value << mem.committed;
        out << YAML::Key << "frozen" << YAML::Value << mem.frozen;
        out << YAML::EndMap;

        out << YAML::Key << "height_profile" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "front_height" << YAML::Value << entry.height_profile.front_height;
        out << YAML::Key << "middle_height" << YAML::Value << entry.height_profile.middle_height;
        out << YAML::Key << "rear_height" << YAML::Value << entry.height_profile.rear_height;
        out << YAML::Key << "front_rear_height_delta" << YAML::Value << entry.height_profile.front_rear_height_delta;
        out << YAML::Key << "max_height" << YAML::Value << entry.height_profile.max_height;
        out << YAML::Key << "height_variance" << YAML::Value << entry.height_profile.height_variance;
        out << YAML::Key << "cargo_score" << YAML::Value << entry.height_profile.cargo_score;
        out << YAML::EndMap;

        out << YAML::Key << "mirror_mode" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "enabled" << YAML::Value << entry.mirror_info.enabled;
        out << YAML::Key << "plane" << YAML::Value << entry.mirror_info.plane;
        out << YAML::Key << "flipped_axis" << YAML::Value << entry.mirror_info.flipped_axis;
        out << YAML::Key << "center_x" << YAML::Value << entry.mirror_info.center_x;
        out << YAML::Key << "center_y" << YAML::Value << entry.mirror_info.center_y;
        out << YAML::Key << "center_z" << YAML::Value << entry.mirror_info.center_z;
        out << YAML::Key << "flipped_axis_center" << YAML::Value
            << entry.mirror_info.flipped_axis_center;
        out << YAML::Key << "mirror_axis" << YAML::Value << entry.mirror_info.mirror_axis;
        out << YAML::Key << "mirror_centerline" << YAML::Value << entry.mirror_info.mirror_centerline;
        out << YAML::Key << "mirror_centerline_x" << YAML::Value << entry.mirror_info.mirror_centerline_x;
        out << YAML::Key << "mirror_centerline_y" << YAML::Value << entry.mirror_info.mirror_centerline_y;
        out << YAML::Key << "mirror_centerline_z" << YAML::Value << entry.mirror_info.mirror_centerline_z;
        out << YAML::Key << "symmetry_confidence" << YAML::Value << entry.mirror_info.symmetry_confidence;
        out << YAML::Key << "observed_node_count" << YAML::Value << entry.mirror_info.observed_node_count;
        out << YAML::Key << "mirrored_node_count" << YAML::Value << entry.mirror_info.mirrored_node_count;
        out << YAML::Key << "observed_edge_count" << YAML::Value << entry.mirror_info.observed_edge_count;
        out << YAML::Key << "mirrored_edge_count" << YAML::Value << entry.mirror_info.mirrored_edge_count;
        out << YAML::Key << "near_identical_warning_count" << YAML::Value
            << entry.mirror_info.near_identical_warning_count;
        out << YAML::Key << "invalid_edge_warning_count" << YAML::Value
            << entry.mirror_info.invalid_edge_warning_count;
        out << YAML::EndMap;

        out << YAML::Key << "nodes" << YAML::Value << YAML::BeginSeq;
        for (const auto & n : mem.nodes) {
          out << YAML::BeginMap;
          out << YAML::Key << "id" << YAML::Value << n.id;
          out << YAML::Key << "local_x" << YAML::Value << n.local_x;
          out << YAML::Key << "local_y" << YAML::Value << n.local_y;
          out << YAML::Key << "local_z" << YAML::Value << n.local_z;
          out << YAML::Key << "point_count" << YAML::Value << n.point_count;
          out << YAML::Key << "confidence" << YAML::Value << n.confidence;
          out << YAML::Key << "is_endpoint" << YAML::Value << n.is_endpoint;
          out << YAML::Key << "is_boundary" << YAML::Value << n.is_boundary;
          out << YAML::Key << "is_virtual" << YAML::Value << n.is_virtual;
          out << YAML::Key << "is_mirrored" << YAML::Value << n.is_mirrored;
          out << YAML::Key << "mirrored_from_node_id" << YAML::Value << n.mirrored_from_node_id;
          out << YAML::Key << "visibility_confidence" << YAML::Value << n.visibility_confidence;
          out << YAML::Key << "semantic_hint" << YAML::Value << n.semantic_hint;
          out << YAML::EndMap;
        }
        out << YAML::EndSeq;

        out << YAML::Key << "edges" << YAML::Value << YAML::BeginSeq;
        for (const auto & e : mem.edges) {
          out << YAML::BeginMap;
          out << YAML::Key << "id" << YAML::Value << e.id;
          out << YAML::Key << "node_a" << YAML::Value << e.node_a;
          out << YAML::Key << "node_b" << YAML::Value << e.node_b;
          out << YAML::Key << "local_dx" << YAML::Value << e.local_dx;
          out << YAML::Key << "local_dy" << YAML::Value << e.local_dy;
          out << YAML::Key << "local_dz" << YAML::Value << e.local_dz;
          out << YAML::Key << "length" << YAML::Value << e.length;
          out << YAML::Key << "direction_yaw" << YAML::Value << e.direction_yaw;
          out << YAML::Key << "height_delta" << YAML::Value << e.height_delta;
          out << YAML::Key << "confidence" << YAML::Value << e.confidence;
          out << YAML::Key << "is_structural_edge" << YAML::Value << e.is_structural_edge;
          out << YAML::Key << "is_virtual" << YAML::Value << e.is_virtual;
          out << YAML::Key << "is_mirrored" << YAML::Value << e.is_mirrored;
          out << YAML::Key << "mirrored_from_edge_id" << YAML::Value << e.mirrored_from_edge_id;
          out << YAML::Key << "visibility_confidence" << YAML::Value << e.visibility_confidence;
          out << YAML::EndMap;
        }
        out << YAML::EndSeq;

        out << YAML::Key << "endpoint_relations" << YAML::Value << YAML::BeginSeq;
        for (const auto & r : mem.endpoint_relations) {
          out << YAML::BeginMap;
          out << YAML::Key << "relation_type" << YAML::Value << r.relation_type;
          out << YAML::Key << "endpoint_a" << YAML::Value << r.endpoint_a;
          out << YAML::Key << "endpoint_b" << YAML::Value << r.endpoint_b;
          out << YAML::Key << "expected_distance" << YAML::Value << r.expected_distance;
          out << YAML::Key << "expected_direction_yaw" << YAML::Value << r.expected_direction_yaw;
          out << YAML::Key << "expected_height_delta" << YAML::Value << r.expected_height_delta;
          out << YAML::Key << "confidence" << YAML::Value << r.confidence;
          out << YAML::Key << "relation_score" << YAML::Value << r.relation_score;
          out << YAML::Key << "uses_virtual_node" << YAML::Value << r.uses_virtual_node;
          out << YAML::Key << "uses_mirrored_node" << YAML::Value << r.uses_mirrored_node;
          out << YAML::EndMap;
        }
        out << YAML::EndSeq;
        out << YAML::EndMap;
      }
      out << YAML::EndSeq;
      out << YAML::EndMap;
      std::ofstream f(object_memory_bank_file_);
      if (!f.is_open()) {
        err = "cannot open " + object_memory_bank_file_;
        return false;
      }
      f << out.c_str() << "\n";
      return true;
    } catch (const std::exception & ex) {
      err = ex.what();
      return false;
    }
  }

  bool loadObjectMemoryBank(std::string & err) {
    try {
      if (!std::filesystem::exists(object_memory_bank_file_)) {
        const auto parent = std::filesystem::path(object_memory_bank_file_).parent_path();
        if (!parent.empty()) std::filesystem::create_directories(parent);
        std::ofstream f(object_memory_bank_file_);
        if (!f.is_open()) {
          err = "cannot create empty object memory bank: " + object_memory_bank_file_;
          return false;
        }
        f << "version: 1\nframe_id: " << target_frame_ << "\ncreated_at: 0.0\n\nmemory_bank: []\n";
        object_memory_bank_.entries.clear();
        class_memory_bank_.clear();
        return true;
      }
      YAML::Node root = YAML::LoadFile(object_memory_bank_file_);
      YAML::Node bank = root["memory_bank"] ? root["memory_bank"] : root;
      if (!bank || !bank.IsSequence()) {
        err = "object memory bank has no memory_bank sequence: " + object_memory_bank_file_;
        return false;
      }
      ObjectMemoryBank loaded;
      for (const auto & node : bank) {
        ObjectMemoryEntry entry;
        entry.memory_id = node["memory_id"] ? node["memory_id"].as<std::string>() : "MEMORY";
        entry.class_label = node["class_label"] ? node["class_label"].as<std::string>() : "UNKNOWN";
        entry.class_prior = node["class_prior"] ? node["class_prior"].as<std::string>() : "UNKNOWN";
        entry.source_component_type =
            node["source_component_type"] ? node["source_component_type"].as<std::string>() : "unknown_like";
        entry.source_component_type_confidence =
            node["source_component_type_confidence"] ? node["source_component_type_confidence"].as<double>() : 0.0;
        entry.source_component_type_reason =
            node["source_component_type_reason"] ? node["source_component_type_reason"].as<std::string>() : "";
        if (node["applicable_component_types"]) {
          entry.applicable_component_types =
              node["applicable_component_types"].as<std::vector<std::string>>();
        } else {
          entry.applicable_component_types =
              defaultApplicableComponentTypesForClass(entry.class_label, entry.source_component_type);
        }
        entry.distance_profile = node["distance_profile"] ? node["distance_profile"].as<std::string>() : "mixed";
        entry.memory_confidence = node["memory_confidence"] ? node["memory_confidence"].as<double>() : 0.0;
        entry.class_confidence = node["class_confidence"] ? node["class_confidence"].as<double>() : 0.0;
        entry.observed_count = node["observed_count"] ? node["observed_count"].as<int>() : 0;
        entry.matched_count = node["matched_count"] ? node["matched_count"].as<int>() : 0;
        entry.false_match_count = node["false_match_count"] ? node["false_match_count"].as<int>() : 0;
        entry.created_time = node["created_time"] ? node["created_time"].as<double>() : 0.0;
        entry.last_matched_time = node["last_matched_time"] ? node["last_matched_time"].as<double>() : 0.0;

        auto & mem = entry.shape_memory;
        mem.valid = true;
        mem.committed = true;
        mem.frozen = true;
        mem.class_label = entry.class_label;
        mem.class_prior = entry.class_prior;
        mem.source_component_type = entry.source_component_type;
        mem.source_component_type_confidence = entry.source_component_type_confidence;
        mem.source_component_type_reason = entry.source_component_type_reason;
        mem.applicable_component_types = entry.applicable_component_types;
        mem.class_confidence = entry.class_confidence;
        mem.memory_confidence = entry.memory_confidence;
        mem.confidence = entry.memory_confidence;
        if (node["shape_memory"]) {
          const auto sm = node["shape_memory"];
          if (sm["center"]) {
            auto v = sm["center"].as<std::vector<double>>();
            if (v.size() >= 3) { mem.center_x = v[0]; mem.center_y = v[1]; mem.center_z = v[2]; }
          }
          mem.yaw = sm["yaw"] ? sm["yaw"].as<double>() : 0.0;
          if (sm["dimensions"]) {
            auto v = sm["dimensions"].as<std::vector<double>>();
            if (v.size() >= 3) { mem.length = v[0]; mem.width = v[1]; mem.height = v[2]; }
          }
          if (sm["local_bounds"]) {
            auto v = sm["local_bounds"].as<std::vector<double>>();
            if (v.size() >= 6) {
              mem.min_local_x = v[0]; mem.max_local_x = v[1];
              mem.min_local_y = v[2]; mem.max_local_y = v[3];
              mem.min_local_z = v[4]; mem.max_local_z = v[5];
            }
          }
          mem.confidence = sm["confidence"] ? sm["confidence"].as<double>() : mem.confidence;
          mem.committed = sm["committed"] ? sm["committed"].as<bool>() : true;
          mem.frozen = sm["frozen"] ? sm["frozen"].as<bool>() : true;
        }
        if (node["height_profile"]) {
          const auto hp = node["height_profile"];
          entry.height_profile.front_height = hp["front_height"] ? hp["front_height"].as<double>() : 0.0;
          entry.height_profile.middle_height = hp["middle_height"] ? hp["middle_height"].as<double>() : 0.0;
          entry.height_profile.rear_height = hp["rear_height"] ? hp["rear_height"].as<double>() : 0.0;
          entry.height_profile.front_rear_height_delta =
              hp["front_rear_height_delta"] ? hp["front_rear_height_delta"].as<double>() : 0.0;
          entry.height_profile.max_height = hp["max_height"] ? hp["max_height"].as<double>() : 0.0;
          entry.height_profile.height_variance = hp["height_variance"] ? hp["height_variance"].as<double>() : 0.0;
          entry.height_profile.cargo_score = hp["cargo_score"] ? hp["cargo_score"].as<double>() : 0.0;
        }
        if (node["mirror_mode"]) {
          const auto mm = node["mirror_mode"];
          entry.mirror_info.enabled = mm["enabled"] ? mm["enabled"].as<bool>() : false;
          entry.mirror_info.plane = mm["plane"] ? mm["plane"].as<std::string>() :
              (mm["mirror_axis"] ? mirrorPlaneFromDeprecatedAxis(mm["mirror_axis"].as<std::string>()) : "xz");
          entry.mirror_info.flipped_axis =
              mm["flipped_axis"] ? mm["flipped_axis"].as<std::string>() :
              (entry.mirror_info.plane == "xy" ? "local_z" :
               (entry.mirror_info.plane == "yz" ? "local_x" : "local_y"));
          entry.mirror_info.center_x = mm["center_x"] ? mm["center_x"].as<double>() : 0.0;
          entry.mirror_info.center_y = mm["center_y"] ? mm["center_y"].as<double>() : 0.0;
          entry.mirror_info.center_z = mm["center_z"] ? mm["center_z"].as<double>() : 0.0;
          entry.mirror_info.flipped_axis_center =
              mm["flipped_axis_center"] ? mm["flipped_axis_center"].as<double>() : 0.0;
          entry.mirror_info.mirror_axis = mm["mirror_axis"] ? mm["mirror_axis"].as<std::string>() : "local_y";
          entry.mirror_info.mirror_centerline =
              mm["mirror_centerline"] ? mm["mirror_centerline"].as<double>() : 0.0;
          entry.mirror_info.mirror_centerline_x =
              mm["mirror_centerline_x"] ? mm["mirror_centerline_x"].as<double>() : 0.0;
          entry.mirror_info.mirror_centerline_y =
              mm["mirror_centerline_y"] ? mm["mirror_centerline_y"].as<double>() : 0.0;
          entry.mirror_info.mirror_centerline_z =
              mm["mirror_centerline_z"] ? mm["mirror_centerline_z"].as<double>() : 0.0;
          entry.mirror_info.symmetry_confidence =
              mm["symmetry_confidence"] ? mm["symmetry_confidence"].as<double>() : 0.0;
          entry.mirror_info.observed_node_count =
              mm["observed_node_count"] ? mm["observed_node_count"].as<int>() : 0;
          entry.mirror_info.mirrored_node_count =
              mm["mirrored_node_count"] ? mm["mirrored_node_count"].as<int>() : 0;
          entry.mirror_info.observed_edge_count =
              mm["observed_edge_count"] ? mm["observed_edge_count"].as<int>() : 0;
          entry.mirror_info.mirrored_edge_count =
              mm["mirrored_edge_count"] ? mm["mirrored_edge_count"].as<int>() : 0;
          entry.mirror_info.near_identical_warning_count =
              mm["near_identical_warning_count"] ? mm["near_identical_warning_count"].as<int>() : 0;
          entry.mirror_info.invalid_edge_warning_count =
              mm["invalid_edge_warning_count"] ? mm["invalid_edge_warning_count"].as<int>() : 0;
          mem.mirror_info = entry.mirror_info;
        }
        if (node["nodes"]) {
          for (const auto & nn : node["nodes"]) {
            ShapeMemoryNode n;
            n.id = nn["id"] ? nn["id"].as<int>() : 0;
            n.local_x = nn["local_x"] ? nn["local_x"].as<double>() : 0.0;
            n.local_y = nn["local_y"] ? nn["local_y"].as<double>() : 0.0;
            n.local_z = nn["local_z"] ? nn["local_z"].as<double>() : 0.0;
            n.point_count = nn["point_count"] ? nn["point_count"].as<int>() : 0;
            n.confidence = nn["confidence"] ? nn["confidence"].as<double>() : 0.0;
            n.is_endpoint = nn["is_endpoint"] ? nn["is_endpoint"].as<bool>() : false;
            n.is_boundary = nn["is_boundary"] ? nn["is_boundary"].as<bool>() : false;
            n.is_virtual = nn["is_virtual"] ? nn["is_virtual"].as<bool>() : false;
            n.is_mirrored = nn["is_mirrored"] ? nn["is_mirrored"].as<bool>() : false;
            n.mirrored_from_node_id =
                nn["mirrored_from_node_id"] ? nn["mirrored_from_node_id"].as<int>() : -1;
            n.visibility_confidence =
                nn["visibility_confidence"] ? nn["visibility_confidence"].as<double>() : 1.0;
            n.semantic_hint = nn["semantic_hint"] ? nn["semantic_hint"].as<std::string>() : "unknown";
            mem.nodes.push_back(n);
          }
        }
        if (node["edges"]) {
          for (const auto & en : node["edges"]) {
            ShapeMemoryEdge e;
            e.id = en["id"] ? en["id"].as<int>() : 0;
            e.node_a = en["node_a"] ? en["node_a"].as<int>() : -1;
            e.node_b = en["node_b"] ? en["node_b"].as<int>() : -1;
            e.local_dx = en["local_dx"] ? en["local_dx"].as<double>() : 0.0;
            e.local_dy = en["local_dy"] ? en["local_dy"].as<double>() : 0.0;
            e.local_dz = en["local_dz"] ? en["local_dz"].as<double>() : 0.0;
            e.length = en["length"] ? en["length"].as<double>() : 0.0;
            e.direction_yaw = en["direction_yaw"] ? en["direction_yaw"].as<double>() : 0.0;
            e.height_delta = en["height_delta"] ? en["height_delta"].as<double>() : 0.0;
            e.confidence = en["confidence"] ? en["confidence"].as<double>() : 0.0;
            e.is_structural_edge = en["is_structural_edge"] ? en["is_structural_edge"].as<bool>() : false;
            e.is_virtual = en["is_virtual"] ? en["is_virtual"].as<bool>() : false;
            e.is_mirrored = en["is_mirrored"] ? en["is_mirrored"].as<bool>() : false;
            e.mirrored_from_edge_id =
                en["mirrored_from_edge_id"] ? en["mirrored_from_edge_id"].as<int>() : -1;
            e.visibility_confidence =
                en["visibility_confidence"] ? en["visibility_confidence"].as<double>() : 1.0;
            mem.edges.push_back(e);
          }
        }
        if (node["endpoint_relations"]) {
          for (const auto & rn : node["endpoint_relations"]) {
            EndpointRelation r;
            r.relation_type = rn["relation_type"] ? rn["relation_type"].as<std::string>() : "unknown";
            r.endpoint_a = rn["endpoint_a"] ? rn["endpoint_a"].as<int>() : -1;
            r.endpoint_b = rn["endpoint_b"] ? rn["endpoint_b"].as<int>() : -1;
            r.expected_distance = rn["expected_distance"] ? rn["expected_distance"].as<double>() : 0.0;
            r.expected_direction_yaw =
                rn["expected_direction_yaw"] ? rn["expected_direction_yaw"].as<double>() : 0.0;
            r.expected_height_delta =
                rn["expected_height_delta"] ? rn["expected_height_delta"].as<double>() : 0.0;
            r.confidence = rn["confidence"] ? rn["confidence"].as<double>() : 0.0;
            r.relation_score = rn["relation_score"] ? rn["relation_score"].as<double>() : r.confidence;
            r.uses_virtual_node = rn["uses_virtual_node"] ? rn["uses_virtual_node"].as<bool>() : false;
            r.uses_mirrored_node = rn["uses_mirrored_node"] ? rn["uses_mirrored_node"].as<bool>() : false;
            mem.endpoint_relations.push_back(r);
          }
        }
        loaded.entries.push_back(entry);
      }
      object_memory_bank_ = loaded;
      class_memory_bank_.clear();
      for (const auto & entry : object_memory_bank_.entries) {
        class_memory_bank_[entry.class_label != "UNKNOWN" ? entry.class_label : entry.class_prior]
            .push_back(entry.shape_memory);
      }
      return true;
    } catch (const std::exception & ex) {
      err = ex.what();
      return false;
    }
  }

  bool dumpObjectMemoryBank(std::string & err) const {
    std::string save_err;
    if (!saveObjectMemoryBank(save_err)) {
      err = save_err;
      return false;
    }
    return true;
  }

  void handleSaveObjectMemoryBank(
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    std::string err;
    res->success = saveObjectMemoryBank(err);
    res->message = res->success ? object_memory_bank_file_ : err;
  }

  void handleLoadObjectMemoryBank(
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    std::string err;
    res->success = loadObjectMemoryBank(err);
    res->message = res->success ? ("loaded object memories=" + std::to_string(object_memory_bank_.entries.size()))
                                : err;
  }

  void handleClearObjectMemoryBank(
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    object_memory_bank_.entries.clear();
    class_memory_bank_.clear();
    res->success = true;
    res->message = "object memory bank cleared";
  }

  void handleDumpObjectMemoryBank(
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    std::string err;
    res->success = dumpObjectMemoryBank(err);
    res->message = res->success ? object_memory_bank_file_ : err;
  }

  void handleDumpShapeMemory(
      const std::shared_ptr<cx128s2_bev_perception::srv::ControlShapeMemory::Request> req,
      std::shared_ptr<cx128s2_bev_perception::srv::ControlShapeMemory::Response> res) {
    int target_id = req->track_id;
    int count = 0;
    std::string last_err;

    for (const auto & tr : tracks_) {
      if (target_id < 0 || tr.id == target_id) {
        if (tr.shape_memory.valid) {
          std::string err;
          if (dumpSingleShapeMemory(tr, err)) {
            count++;
          } else {
            last_err = err;
          }
        }
        if (target_id >= 0) break;
      }
    }

    if (count > 0) {
      res->success = true;
      res->message = "Dumped shape memory files for " + std::to_string(count) + " tracks.";
    } else {
      res->success = false;
      res->message = last_err.empty() ?
                     (target_id < 0 ? "No active tracks with valid shape memory found to dump."
                                    : "Track ID " + std::to_string(target_id) + " has no valid shape memory.")
                     : last_err;
    }
  }

  void handleClearRoi(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    roi_enabled_ = false;
    roi_zones_.clear();
    visibility_model_.clearStaticCache();
    visibility_prewarm_pending_ = true;
    res->success = true;
    res->message = "roi disabled and roi policy cleared";
  }

  void handleSaveRoi(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    if (use_roi_policy_) {
      std::string err;
      res->success = saveRoiPolicy(err);
      res->message = res->success ? roi_policy_file_ : err;
      return;
    }
    std::ofstream f(roi_file_);
    if (!f.is_open()) {
      res->success = false;
      res->message = "cannot open " + roi_file_;
      return;
    }
    f << "roi_enabled " << roi_enabled_ << "\n";
    f << "roi_shape " << roi_shape_ << "\n";
    f << "roi_box " << roi_x_min_ << " " << roi_x_max_ << " " << roi_y_min_ << " " << roi_y_max_ << "\n";
    f << "roi_polygon_x";
    for (double v : roi_polygon_x_) f << " " << v;
    f << "\nroi_polygon_y";
    for (double v : roi_polygon_y_) f << " " << v;
    f << "\n";
    res->success = true;
    res->message = roi_file_;
  }

  void handleLoadRoi(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    if (use_roi_policy_) {
      std::string err;
      res->success = loadRoiPolicy(err);
      res->message = res->success ? "roi policy loaded" : err;
      return;
    }
    std::ifstream f(roi_file_);
    if (!f.is_open()) {
      res->success = false;
      res->message = "cannot open " + roi_file_;
      return;
    }
    std::string tag;
    while (f >> tag) {
      if (tag == "roi_enabled") {
        f >> roi_enabled_;
      } else if (tag == "roi_shape") {
        f >> roi_shape_;
      } else if (tag == "roi_box") {
        f >> roi_x_min_ >> roi_x_max_ >> roi_y_min_ >> roi_y_max_;
      } else if (tag == "roi_polygon_x") {
        std::string line;
        std::getline(f, line);
        std::istringstream ss(line);
        roi_polygon_x_.clear();
        double v;
        while (ss >> v) roi_polygon_x_.push_back(v);
      } else if (tag == "roi_polygon_y") {
        std::string line;
        std::getline(f, line);
        std::istringstream ss(line);
        roi_polygon_y_.clear();
        double v;
        while (ss >> v) roi_polygon_y_.push_back(v);
      }
    }
    res->success = true;
    res->message = "roi loaded";
    visibility_model_.clearStaticCache();
    visibility_prewarm_pending_ = true;
  }

  bool writeBackgroundCsv(const std::string & path) const {
    std::ofstream f(path);
    if (!f.is_open()) return false;
    f << "key,cell_x,cell_y,center_x,center_y,occupancy_probability,seen_count,ground_z,mean_z,"
         "std_z,max_z,min_z,max_density,static_score,last_seen_time,valid\n";
    for (const auto & kv : background_map_) {
      const auto & c = kv.second;
      f << kv.first << "," << c.ix << "," << c.iy << "," << c.center_x << "," << c.center_y
        << "," << c.occupancy_probability << "," << c.seen_count << "," << c.ground_z
        << "," << c.mean_z << "," << c.std_z << "," << c.max_z << "," << c.min_z
        << "," << c.max_density << "," << c.static_score << "," << c.last_seen_time
        << "," << c.valid << "\n";
    }
    return true;
  }

  rcl_interfaces::msg::SetParametersResult onSetParameters(
      const std::vector<rclcpp::Parameter> & parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    // Create temporary variables to hold validation changes
    std::string new_mode = foreground_classifier_mode_;
    double new_pdf_threshold = foreground_pdf_density_threshold_;
    int new_voxel_diff = foreground_voxel_diff_threshold_;
    int new_min_persistence = foreground_min_persistence_frames_;
    bool new_ror_enabled = enable_radius_outlier_removal_;
    double new_ror_radius = ror_radius_;
    int new_ror_min_neighbors = ror_min_neighbors_;
    bool new_enable_parallel = enable_parallel_processing_;
    int new_parallel_threads = parallel_threads_;
    bool new_z_profile_enabled = cluster_z_profile_diagnostics_enabled_;
    double new_z_profile_bin = cluster_z_profile_bin_size_;
    double new_vehicle_stopped_duration_sec = vehicle_stopped_duration_sec_;
    bool new_raw_vehicle_box_refinement_enabled = raw_vehicle_box_refinement_enabled_;

    for (const auto & param : parameters) {
      const std::string & name = param.get_name();
      if (name.rfind("general_ground_noise.", 0) == 0) {
        result.successful = false; result.reason = name + " requires node restart"; return result;
      }
      if((name.rfind("crane_slope.sheave.reference_",0)==0 && name!="crane_slope.sheave.reference_matching_enabled") || name=="crane_slope.sheave.debug_seed_world" || name=="crane_slope.sheave.learned_reference_file"){
        result.successful=false;result.reason="Reviewed Housing reference data are immutable during a run; edit YAML and restart";return result;
      }

      if (name == "visibility_box_prior_weight" || name == "vehicle_fixed_yaw_rad") {
        result.successful = false;
        result.reason = name + " requires node restart";
        return result;
      }
      if (name == "crane_pipeline" || name.rfind("crane_plane.", 0) == 0 ||
          name == "crane_slope.azimuth_columns" || name == "crane_slope.lidar_channels" ||
          name == "crane_slope.lidar_model" || name == "crane_slope.publish_debug_clouds" ||
          name == "crane_slope.publish_colored_cloud" || name == "crane_slope.publish_direction_clouds" ||
          name == "height_surface.publish_debug") {
        result.successful = false;
        result.reason = name + " requires node restart";
        return result;
      }
      if (name == "publish_debug_topics" || name == "publish_recording_frame" ||
          name == "publish_debug_foreground_points" || name == "perception_mode") {
        result.successful = false;
        result.reason = name + " requires node restart to recreate topic endpoints";
        return result;
      }
      if (name == "crane_beam.crane_move_start_x" || name == "crane_beam.crane_move_start_y" ||
          name == "crane_beam.crane_move_stop_x" || name == "crane_beam.crane_move_stop_y") {
        if (!std::isfinite(param.as_double()) || param.as_double() <= 0.0) {
          result.successful = false; result.reason = "Crane movement XY thresholds must be finite and positive";
          return result;
        }
      }
      if (name == "crane_beam.crane_move_stop_frames" && param.as_int() < 1) {
        result.successful = false; result.reason = "Crane movement stop_frames must be positive";
        return result;
      }
      if (name == "crane_beam.hook.reference_file" || name == "crane_beam.hook.reference_min_frames") {
        result.successful = false;
        result.reason = "Hook reference file/min_frames require restart; use HOOK_REFERENCE commands";
        return result;
      }

      if (name == "raw_vehicle_box_refinement_enabled") {
        new_raw_vehicle_box_refinement_enabled = param.as_bool();
      } else if (name.rfind("raw_vehicle_box_", 0) == 0) {
        result.successful = false;
        result.reason = "raw_vehicle_box parameters require node restart";
        return result;
      }
      if (name == "vehicle_require_entry_direction") {
        result.successful = false;
        result.reason = "vehicle_require_entry_direction requires node restart";
        return result;
      }
      if (name.rfind("visibility.", 0) == 0) {
        result.successful = false;
        result.reason = "visibility parameters require node restart to reload driver calibration";
        return result;
      }
      if (name == "foreground_classifier_mode") {
        std::string val = param.as_string();
        if (val != "legacy_threshold" && val != "statistical_score" && val != "gdg_point_pdf") {
          result.successful = false;
          result.reason = "Invalid foreground_classifier_mode. Must be legacy_threshold, statistical_score, or gdg_point_pdf.";
          return result;
        }
        new_mode = val;
      } else if (name == "foreground_pdf_density_threshold") {
        double val = param.as_double();
        if (val < 0.0) {
          result.successful = false;
          result.reason = "foreground_pdf_density_threshold must be non-negative.";
          return result;
        }
        new_pdf_threshold = val;
      } else if (name == "foreground_voxel_diff_threshold") {
        int val = param.as_int();
        if (val < 0) {
          result.successful = false;
          result.reason = "foreground_voxel_diff_threshold must be non-negative.";
          return result;
        }
        new_voxel_diff = val;
      } else if (name == "foreground_min_persistence_frames") {
        int val = param.as_int();
        if (val < 1) {
          result.successful = false;
          result.reason = "foreground_min_persistence_frames must be at least 1.";
          return result;
        }
        new_min_persistence = val;
      } else if (name == "enable_radius_outlier_removal") {
        new_ror_enabled = param.as_bool();
      } else if (name == "ror_radius") {
        double val = param.as_double();
        if (val < 0.05) {
          result.successful = false;
          result.reason = "ror_radius must be at least 0.05.";
          return result;
        }
        new_ror_radius = val;
      } else if (name == "ror_min_neighbors") {
        int val = param.as_int();
        if (val < 0) {
          result.successful = false;
          result.reason = "ror_min_neighbors must be non-negative.";
          return result;
        }
        new_ror_min_neighbors = val;
      } else if (name == "enable_parallel_processing") {
        new_enable_parallel = param.as_bool();
      } else if (name == "parallel_threads") {
        int val = param.as_int();
        if (val < 1 || val > 64) {
          result.successful = false;
          result.reason = "parallel_threads must be between 1 and 64.";
          return result;
        }
        new_parallel_threads = val;
      } else if (name == "cluster_z_profile_diagnostics_enabled") {
        new_z_profile_enabled = param.as_bool();
      } else if (name == "cluster_z_profile_bin_size") {
        double val = param.as_double();
        if (val < 0.05) {
          result.successful = false;
          result.reason = "cluster_z_profile_bin_size must be at least 0.05.";
          return result;
        }
        new_z_profile_bin = val;
      } else if (name == "vehicle_stopped_duration_sec") {
        double val = param.as_double();
        if (val < 0.0) {
          result.successful = false;
          result.reason = "vehicle_stopped_duration_sec must be non-negative.";
          return result;
        }
        new_vehicle_stopped_duration_sec = val;
      } else if (name == "bev_x_resolution" || name == "bev_y_resolution" ||
                 name == "min_points_per_occupied_cell" || name == "cluster_connectivity" ||
                 name == "cluster_z_guard_enabled" || name == "cluster_neighbor_max_z_gap" ||
                 name == "cluster_z_layer_enabled" || name == "cluster_z_layer_resolution" ||
                 name == "cluster_gap_bridging_enabled" || name == "cluster_bridge_max_z_gap" ||
                 name == "min_cluster_points_near" || name == "min_cluster_points_mid" ||
                 name == "min_cluster_points_long" || name == "min_cluster_cells_near" ||
                 name == "min_cluster_cells_mid" || name == "min_cluster_cells_long") {
        std::lock_guard<std::mutex> crane_lock(crane_command_mutex_);
        if (name == "bev_x_resolution") bev_x_res_ = std::max(0.01, param.as_double());
        else if (name == "bev_y_resolution") bev_y_res_ = std::max(0.01, param.as_double());
        else if (name == "min_points_per_occupied_cell") min_points_per_occupied_cell_ = std::max(1, static_cast<int>(param.as_int()));
        else if (name == "cluster_connectivity") cluster_connectivity_ = static_cast<int>(param.as_int());
        else if (name == "cluster_z_guard_enabled") cluster_z_guard_enabled_ = param.as_bool();
        else if (name == "cluster_neighbor_max_z_gap") cluster_neighbor_max_z_gap_ = param.as_double();
        else if (name == "cluster_z_layer_enabled") cluster_z_layer_enabled_ = param.as_bool();
        else if (name == "cluster_z_layer_resolution") cluster_z_layer_resolution_ = std::max(0.01, param.as_double());
        else if (name == "cluster_gap_bridging_enabled") cluster_gap_bridging_enabled_ = param.as_bool();
        else if (name == "cluster_bridge_max_z_gap") cluster_bridge_max_z_gap_ = param.as_double();
        else if (name == "min_cluster_points_near") min_cluster_points_near_ = static_cast<int>(param.as_int());
        else if (name == "min_cluster_points_mid") min_cluster_points_mid_ = static_cast<int>(param.as_int());
        else if (name == "min_cluster_points_long") min_cluster_points_long_ = static_cast<int>(param.as_int());
        else if (name == "min_cluster_cells_near") min_cluster_cells_near_ = static_cast<int>(param.as_int());
        else if (name == "min_cluster_cells_mid") min_cluster_cells_mid_ = static_cast<int>(param.as_int());
        else if (name == "min_cluster_cells_long") min_cluster_cells_long_ = static_cast<int>(param.as_int());
        clusterer_ = createClusterer();
        RCLCPP_INFO(get_logger(), "[PARAM] Updated BEV clusterer parameter: %s", name.c_str());
      } else if (name.rfind("crane_slope.", 0) == 0 || name.rfind("height_surface.", 0) == 0) {
        std::lock_guard<std::mutex> crane_lock(crane_command_mutex_);
        if (crane_pipeline_) {
          auto c = crane_pipeline_->config();
          try {
            if (name == "crane_slope.max_slope_degree") c.max_slope_deg = param.as_double();
            else if (name == "crane_slope.lidar_degree_start") c.lidar_degree_start = param.as_double();
            else if (name == "crane_slope.lidar_degree_step") c.lidar_degree_step = param.as_double();
            else if (name == "crane_slope.lidar_height") c.lidar_height = param.as_double();
            else if (name == "crane_slope.cx128s2_prism_offset_deg") c.cx128s2_prism_offset_deg = param.as_double();
            else if (name == "crane_slope.cx128s2_prism_angles_deg") c.cx128s2_prism_angles_deg = param.as_double_array();
            else if (name == "crane_slope.model_angle_tolerance_deg") c.model_angle_tolerance_deg = param.as_double();
            else if (name == "crane_slope.max_vertical_height") c.max_vertical_height = param.as_double();
            else if (name == "crane_slope.ground_z") c.ground_z = param.as_double();
            else if (name == "crane_slope.force_ground_below_z") c.force_ground_below_z = param.as_double();
            else if (name == "crane_slope.include_ground_in_clustering") c.include_ground_in_clustering = param.as_bool();
            else if (name == "crane_slope.horizontal_enabled") c.horizontal_enabled = param.as_bool();
            else if (name == "crane_slope.horizontal_max_azimuth_gap_deg") c.horizontal_max_azimuth_gap_deg = param.as_double();
            else if (name == "crane_slope.boundary_clustering.enabled") {
              c.boundary_clustering.enabled = param.as_bool();
              crane_boundary_clustering_enabled_ = c.boundary_clustering.enabled;
            }
            else if (name == "crane_slope.boundary_clustering.slope_threshold_deg") c.boundary_clustering.slope_threshold_deg = param.as_double();
            else if (name == "crane_slope.boundary_clustering.height_step_threshold") c.boundary_clustering.height_step_threshold = param.as_double();
            else if (name == "crane_slope.boundary_clustering.range_jump_threshold") c.boundary_clustering.range_jump_threshold = param.as_double();
            else if (name == "crane_slope.boundary_clustering.max_neighbor_distance") c.boundary_clustering.max_neighbor_distance = param.as_double();
            else if (name == "crane_slope.boundary_clustering.min_height_above_ground") c.boundary_clustering.min_height_above_ground = param.as_double();
            else if (name == "crane_slope.boundary_clustering.min_cluster_points") c.boundary_clustering.min_cluster_points = static_cast<int>(param.as_int());
            else if (name == "crane_slope.boundary_clustering.max_cluster_points") c.boundary_clustering.max_cluster_points = static_cast<int>(param.as_int());
            else if (name == "crane_slope.boundary_clustering.include_unknown") c.boundary_clustering.include_unknown = param.as_bool();
            else if (name == "crane_slope.boundary_clustering.enable_morphological_closing") c.boundary_clustering.enable_morphological_closing = param.as_bool();
            else if (name == "crane_slope.boundary_clustering.morph_closing_radius_m") c.boundary_clustering.morph_closing_radius_m = param.as_double();
            else if (name == "crane_slope.z_gaussian_clustering.enabled") {
              c.z_gaussian.enabled = param.as_bool();
              crane_z_gaussian_clustering_enabled_ = c.z_gaussian.enabled;
            }
            else if (name == "crane_slope.z_gaussian_clustering.sigma_z") c.z_gaussian.sigma_z = param.as_double();
            else if (name == "crane_slope.z_gaussian_clustering.range_sigma_factor") c.z_gaussian.range_sigma_factor = param.as_double();
            else if (name == "crane_slope.z_gaussian_clustering.confidence_sigma_multiplier") c.z_gaussian.confidence_sigma_multiplier = param.as_double();
            else if (name == "crane_slope.z_gaussian_clustering.step_sigma_multiplier") c.z_gaussian.step_sigma_multiplier = param.as_double();
            else if (name == "crane_slope.z_gaussian_clustering.max_layer_z_spread") c.z_gaussian.max_layer_z_spread = param.as_double();
            else if (name == "crane_slope.z_gaussian_clustering.min_height_above_ground") c.z_gaussian.min_height_above_ground = param.as_double();
            else if (name == "crane_slope.z_gaussian_clustering.min_layer_points") c.z_gaussian.min_layer_points = static_cast<int>(param.as_int());
            else if (name == "crane_slope.closed_loop_grouping.enabled") {
              c.closed_loop.enabled = param.as_bool();
              crane_closed_loop_grouping_enabled_ = c.closed_loop.enabled;
            }
            else if (name == "crane_slope.closed_loop_grouping.grid_resolution_m") c.closed_loop.grid_resolution_m = param.as_double();
            else if (name == "crane_slope.closed_loop_grouping.closing_radius_m") c.closed_loop.closing_radius_m = param.as_double();
            else if (name == "crane_slope.closed_loop_grouping.min_containment_ratio") c.closed_loop.min_containment_ratio = param.as_double();
            else if (name == "crane_slope.closed_loop_grouping.min_enclosed_cells") c.closed_loop.min_enclosed_cells = static_cast<int>(param.as_int());
            else if (name == "crane_slope.closed_loop_grouping.use_boundary_points_for_loop") c.closed_loop.use_boundary_points_for_loop = param.as_bool();
            else if (name == "crane_slope.closed_loop_grouping.subcluster_match_xy_gate") c.closed_loop.subcluster_match_xy_gate = param.as_double();
            else if (name == "crane_slope.closed_loop_grouping.subcluster_match_z_gate") c.closed_loop.subcluster_match_z_gate = param.as_double();
            else if (name == "crane_slope.roi_filter.enabled") {
              c.roi_filter.enabled = param.as_bool();
              crane_slope_roi_filter_enabled_ = c.roi_filter.enabled;
            }
            else if (name == "crane_slope.roi_filter.only_crane_roi") {
              c.roi_filter.only_crane_roi = param.as_bool();
              crane_slope_roi_only_crane_roi_ = c.roi_filter.only_crane_roi;
            }
            else if (name == "crane_slope.roi_filter.active_include_background") {
              c.roi_filter.active_include_background = param.as_bool();
              crane_slope_roi_active_include_background_ = c.roi_filter.active_include_background;
            }
            else if (name == "crane_slope.roi_filter.idle_exclude_beam_roi") {
              c.roi_filter.idle_exclude_beam_roi = param.as_bool();
              crane_slope_roi_idle_exclude_beam_roi_ = c.roi_filter.idle_exclude_beam_roi;
            }
            else if (name == "crane_slope.roi_filter.expand_beam_roi") {
              c.roi_filter.expand_beam_roi = param.as_bool();
              crane_slope_roi_expand_beam_roi_ = c.roi_filter.expand_beam_roi;
            }
            else if (name == "crane_slope.work_status.fsm_enabled") c.work_status.fsm_enabled = param.as_bool();
            else if (name == "crane_slope.work_status.boundary_motion_enabled") c.work_status.boundary_motion_enabled = param.as_bool();
            else if (name == "crane_slope.work_status.boundary_temporal_enabled") c.work_status.boundary_temporal_enabled = param.as_bool();
            else if (name == "crane_slope.work_status.boundary_start_confirm_sec") c.work_status.boundary_start_confirm_sec = param.as_double();
            else if (name == "crane_slope.work_status.boundary_stop_confirm_sec") c.work_status.boundary_stop_confirm_sec = param.as_double();
            else if (name == "crane_slope.work_status.boundary_support_timeout_sec") c.work_status.boundary_support_timeout_sec = param.as_double();
            else if (name == "crane_slope.work_status.cargo_roi_width_x_m") c.work_status.cargo_roi_width_x_m=param.as_double();
            else if (name == "crane_slope.work_status.cargo_roi_width_y_m") c.work_status.cargo_roi_width_y_m=param.as_double();
            else if (name == "crane_slope.work_status.cargo_start_offset_m") c.work_status.cargo_start_offset_m = param.as_double();
            else if (name == "crane_slope.work_status.virtual_hook_offset_z_m") c.work_status.virtual_hook_offset_z_m = param.as_double();
            else if (name == "crane_slope.work_status.cargo_confirm_sec") c.work_status.cargo_confirm_sec = param.as_double();
            else if (name == "crane_slope.work_status.cargo_loss_sec") c.work_status.cargo_loss_sec = param.as_double();
            else if (name == "crane_slope.work_status.lifting_event_hold_sec") c.work_status.lifting_event_hold_sec=param.as_double();
            else if (name == "crane_slope.work_status.lifting_clearance_delta_m") c.work_status.lifting_clearance_delta_m=param.as_double();
            else if (name == "crane_slope.work_status.lifting_min_clearance_m") c.work_status.lifting_min_clearance_m=param.as_double();
            else if (name == "crane_slope.work_status.cargo_min_surface_width_m") c.work_status.cargo_min_surface_width_m = param.as_double();
            else if (name == "crane_slope.work_status.cargo_min_fill_ratio") c.work_status.cargo_min_fill_ratio = param.as_double();
            else if (name == "crane_slope.work_status.boundary_search_radius_m") c.work_status.boundary_search_radius_m = param.as_double();
            else if (name == "crane_slope.work_status.boundary_min_coherent_points") c.work_status.boundary_min_coherent_points = param.as_int();
            else if (name == "crane_slope.work_status.enabled") c.work_status.enabled = param.as_bool();
            else if (name == "crane_slope.work_status.grid_resolution_m") c.work_status.grid_resolution_m = param.as_double();
            else if (name == "crane_slope.work_status.size_ratio_k") c.work_status.size_ratio_k = param.as_double();
            else if (name == "crane_slope.work_status.connect_xy_margin") c.work_status.connect_xy_margin = param.as_double();
            else if (name == "crane_slope.work_status.connect_max_gap_z") c.work_status.connect_max_gap_z = param.as_double();
            else if (name == "crane_slope.work_status.min_candidate_points") c.work_status.min_candidate_points = static_cast<size_t>(param.as_int());
            else if (name == "crane_slope.work_status.temporal_window_n") c.work_status.temporal_window_n = static_cast<int>(param.as_int());
            else if (name == "crane_slope.work_status.confirm_m_on") c.work_status.confirm_m_on = static_cast<int>(param.as_int());
            else if (name == "crane_slope.work_status.confirm_m_off") c.work_status.confirm_m_off = static_cast<int>(param.as_int());
            else if (name == "crane_slope.temporal_stabilization.enabled") c.temporal_stabilization.enabled = param.as_bool();
            else if (name == "crane_slope.temporal_stabilization.mode") c.temporal_stabilization.mode = param.as_string();
            else if (name == "crane_slope.temporal_stabilization.window_size") c.temporal_stabilization.window_size = static_cast<int>(param.as_int());
            else if (name == "crane_slope.temporal_stabilization.voxel_size_xy") c.temporal_stabilization.voxel_size_xy = param.as_double();
            else if (name == "crane_slope.temporal_stabilization.voxel_size_z") c.temporal_stabilization.voxel_size_z = param.as_double();
            else if (name == "crane_slope.temporal_stabilization.min_hit_count") c.temporal_stabilization.min_hit_count = static_cast<int>(param.as_int());
            else if (name == "crane_slope.temporal_stabilization.temporal_stabilization_enabled") c.temporal_stabilization.temporal_stabilization_enabled = param.as_bool();
            else if (name == "crane_slope.temporal_stabilization.stabilization_alpha") c.temporal_stabilization.stabilization_alpha = param.as_double();
            else if (name == "crane_slope.temporal_stabilization.jump_tolerance_length") c.temporal_stabilization.jump_tolerance_length = param.as_double();
            else if (name == "crane_slope.temporal_stabilization.jump_tolerance_width") c.temporal_stabilization.jump_tolerance_width = param.as_double();
            else if (name == "crane_slope.temporal_stabilization.jump_confirm_frames") c.temporal_stabilization.jump_confirm_frames = static_cast<int>(param.as_int());
            else if (name == "crane_slope.temporal_stabilization.max_hold_missed_frames") c.temporal_stabilization.max_hold_missed_frames = static_cast<int>(param.as_int());
            else if (name == "crane_slope.temporal_stabilization.node_confirm_frames") c.temporal_stabilization.node_confirm_frames = static_cast<int>(param.as_int());
            else if (name == "crane_slope.temporal_stabilization.node_max_hold_frames") c.temporal_stabilization.node_max_hold_frames = static_cast<int>(param.as_int());
            else if (name == "crane_slope.temporal_stabilization.largest_swap_confirm_frames") c.temporal_stabilization.largest_swap_confirm_frames = static_cast<int>(param.as_int());
            else if (name == "crane_slope.temporal_stabilization.largest_match_dist") c.temporal_stabilization.largest_match_dist = param.as_double();
            else if (name == "crane_slope.temporal_stabilization.yaw_jump_tolerance_deg") c.temporal_stabilization.yaw_jump_tolerance_deg = param.as_double();
            else if (name == "crane_slope.temporal_stabilization.yaw_alpha") c.temporal_stabilization.yaw_alpha = param.as_double();
            else if (name == "crane_slope.top_down_hook.max_slope_angle_deg") c.top_down_hook.max_slope_angle_deg = param.as_double();
            else if (name == "crane_slope.top_down_hook.max_turn_angle_deg") c.top_down_hook.max_turn_angle_deg = param.as_double();
            else if (name == "crane_slope.exception_gate.enabled") c.exception_gate.enabled = param.as_bool();
            else if (name == "crane_slope.exception_gate.min_length_m") c.exception_gate.min_length_m = param.as_double();
            else if (name == "crane_slope.exception_gate.min_area_m2") c.exception_gate.min_area_m2 = param.as_double();
            else if (name == "crane_slope.exception_gate.use_visibility") c.exception_gate.use_visibility = param.as_bool();
            else if (name == "crane_slope.exception_gate.max_occlusion_fraction") c.exception_gate.max_occlusion_fraction = param.as_double();
            else if (name == "crane_slope.exception_gate.scan_all_chain_nodes") c.exception_gate.scan_all_chain_nodes = param.as_bool();
            else if (name == "crane_slope.connection_stabilization.enabled") c.connection_stabilization.enabled = param.as_bool();
            else if (name == "crane_slope.connection_stabilization.history_frames") c.connection_stabilization.history_frames = param.as_int();
            else if (name == "crane_slope.connection_stabilization.min_direct_frames") c.connection_stabilization.min_direct_frames = param.as_int();
            else if (name == "crane_slope.connection_stabilization.min_transition_count") c.connection_stabilization.min_transition_count = param.as_int();
            else if (name == "crane_slope.connection_stabilization.missing_hold_frames") c.connection_stabilization.missing_hold_frames = param.as_int();
            else if (name == "crane_slope.connection_stabilization.spatial_continuity_enabled") c.connection_stabilization.spatial_continuity_enabled = param.as_bool();
            else if (name == "crane_slope.connection_stabilization.spatial_min_direct_frames") c.connection_stabilization.spatial_min_direct_frames = param.as_int();
            else if (name == "crane_slope.connection_stabilization.spatial_max_gap_z") c.connection_stabilization.spatial_max_gap_z = param.as_double();
            else if (name == "crane_slope.connection_stabilization.spatial_max_dxy") c.connection_stabilization.spatial_max_dxy = param.as_double();
            else if (name == "crane_slope.dual_roi_anchor.enabled") c.dual_roi_anchor.enabled = param.as_bool();
            else if (name == "crane_slope.dual_roi_anchor.publish_markers") c.dual_roi_anchor.publish_markers = param.as_bool();
            else if (name == "crane_slope.dual_roi_anchor.fixed_roi_size_xy") c.dual_roi_anchor.fixed_roi_size_xy = param.as_double();
            else if (name == "crane_slope.dual_roi_anchor.fixed_roi_size_z") c.dual_roi_anchor.fixed_roi_size_z = param.as_double();
            else if (name == "crane_slope.dual_roi_anchor.moving_roi_size_xy") c.dual_roi_anchor.moving_roi_size_xy = param.as_double();
            else if (name == "crane_slope.dual_roi_anchor.moving_roi_size_z") c.dual_roi_anchor.moving_roi_size_z = param.as_double();
            else if (name == "crane_slope.dual_roi_anchor.confirm_frames") c.dual_roi_anchor.confirm_frames = param.as_int();
            else if (name == "crane_slope.dual_roi_anchor.min_direct_evidence_frames") c.dual_roi_anchor.min_direct_evidence_frames = param.as_int();
            else if (name == "crane_slope.dual_roi_anchor.max_lock_slope_deg") c.dual_roi_anchor.max_lock_slope_deg = param.as_double();
            else if (name == "crane_slope.dual_roi_anchor.max_lock_turn_deg") c.dual_roi_anchor.max_lock_turn_deg = param.as_double();
            else if (name == "crane_slope.dual_roi_anchor.noise_deadband_xy") c.dual_roi_anchor.noise_deadband_xy = param.as_double();
            else if (name == "crane_slope.dual_roi_anchor.noise_deadband_z") c.dual_roi_anchor.noise_deadband_z = param.as_double();
            else if (name == "crane_slope.dual_roi_anchor.relocate_thresh_xy") c.dual_roi_anchor.relocate_thresh_xy = param.as_double();
            else if (name == "crane_slope.dual_roi_anchor.relocate_thresh_z") c.dual_roi_anchor.relocate_thresh_z = param.as_double();
            else if (name == "crane_slope.dual_roi_anchor.relocate_confirm_frames") c.dual_roi_anchor.relocate_confirm_frames = param.as_int();
            else if (name == "crane_slope.dual_roi_anchor.anchor_memory_frames") c.dual_roi_anchor.anchor_memory_frames = param.as_int();
            else if (name == "crane_slope.dual_roi_anchor.beam_intrusion_guard_enabled") c.dual_roi_anchor.beam_intrusion_guard_enabled = param.as_bool();
            else if (name == "crane_slope.dual_roi_anchor.beam_min_length_m") c.dual_roi_anchor.beam_min_length_m = param.as_double();
            else if (name == "crane_slope.dual_roi_anchor.beam_min_aspect_ratio") c.dual_roi_anchor.beam_min_aspect_ratio = param.as_double();
            else if (name == "crane_slope.dual_roi_anchor.intrusion_hold_frames") c.dual_roi_anchor.intrusion_hold_frames = param.as_int();
            else if (name == "crane_slope.virtual_rope.enabled") c.virtual_rope.enabled = param.as_bool();
            else if (name == "crane_slope.virtual_rope.wire_min_spacing_m") c.virtual_rope.wire_min_spacing_m = param.as_double();
            else if (name == "crane_slope.virtual_rope.wire_max_spacing_m") c.virtual_rope.wire_max_spacing_m = param.as_double();
            else if (name == "crane_slope.virtual_rope.wire_merge_span_m") c.virtual_rope.wire_merge_span_m = param.as_double();
            else if (name == "crane_slope.virtual_rope.memory_frames") c.virtual_rope.memory_frames = param.as_int();
            else if (name == "crane_slope.virtual_rope.publish_markers") c.virtual_rope.publish_markers = param.as_bool();
            else if (name == "crane_slope.sheave.surface_gap_m") c.sheave.surface_gap_m = param.as_double();
            else if (name == "crane_slope.sheave.track_continuity_enabled") c.sheave.track_continuity_enabled=param.as_bool();
            else if (name == "crane_slope.sheave.reference_matching_enabled") c.sheave.reference_matching_enabled=param.as_bool();
            else if (name == "crane_slope.sheave.enabled") c.sheave.enabled = param.as_bool();
            else if (name == "crane_slope.sheave.min_radius_m") c.sheave.min_radius_m = param.as_double();
            else if (name == "crane_slope.sheave.max_radius_m") c.sheave.max_radius_m = param.as_double();
            else if (name == "crane_slope.sheave.default_radius_m") c.sheave.default_radius_m = param.as_double();
            else if (name == "crane_slope.sheave.min_arc_points") c.sheave.min_arc_points = param.as_int();
            else if (name == "crane_slope.sheave.max_circle_rms_m") c.sheave.max_circle_rms_m = param.as_double();
            else if (name == "crane_slope.sheave.max_sheave_width_m") c.sheave.max_sheave_width_m = param.as_double();
            else if (name == "crane_slope.sheave.confirm_frames") c.sheave.confirm_frames = param.as_int();
            else if (name == "crane_slope.sheave.terminate_chain_at_sheave") c.sheave.terminate_chain_at_sheave = param.as_bool();
            else if (name == "crane_slope.sheave.publish_markers") c.sheave.publish_markers = param.as_bool();
            else if (name == "crane_slope.z_temporal_median.enabled") c.z_temporal_median.enabled = param.as_bool();
            else if (name == "crane_slope.z_temporal_median.window_frames") c.z_temporal_median.window_frames = param.as_int();
            else if (name == "crane_beam.publish_background_roi_marker") {
              crane_beam_config_.publish_background_roi_marker = param.as_bool();
            }
            else if (name == "height_surface.enabled") {
              c.height.enabled = param.as_bool();
              crane_height_enabled_ = c.height.enabled;
            }
            else if (name == "height_surface.z_bin_size") c.height.patch_resolution = param.as_double();
            else if (name == "height_surface.max_neighbor_dz") c.height.max_neighbor_dz = param.as_double();
            else if (name == "height_surface.max_surface_z_spread") c.height.max_patch_z_spread = param.as_double();
            else if (name == "height_surface.max_xy_neighbor_distance") c.height.max_xy_gap = param.as_double();
            else if (name == "height_surface.min_surface_points") c.height.min_surface_points = static_cast<int>(param.as_int());
            else if (name == "height_surface.min_surface_length") c.height.min_surface_length = param.as_double();
            else if (name == "height_surface.min_surface_width") c.height.min_surface_width = param.as_double();
            crane_pipeline_->updateConfig(c);
            RCLCPP_INFO(get_logger(), "[PARAM] Updated crane pipeline parameter: %s", name.c_str());
          } catch (const std::exception & ex) {
            result.successful = false;
            result.reason = ex.what();
            return result;
          }
        }
      } else if (name == "perception_mode") {
        perception_mode_ = param.as_string();
      } else if (name == "crane_beam_enabled") {
        crane_beam_enabled_ = param.as_bool();
        crane_beam_config_.enabled = crane_beam_enabled_;
        if (crane_beam_detector_) crane_beam_detector_->updateConfig(crane_beam_config_);
      } else if (name.rfind("crane_beam.", 0) == 0) {
        std::lock_guard<std::mutex> crane_lock(crane_command_mutex_);
        if (name == "crane_beam.motion_reference_mode") crane_beam_config_.motion_reference_mode = param.as_string();
        else if (name == "crane_beam.beam_roi_x_min") crane_beam_config_.beam_roi_x_min = param.as_double();
        else if (name == "crane_beam.beam_roi_x_max") crane_beam_config_.beam_roi_x_max = param.as_double();
        else if (name == "crane_beam.beam_roi_y_min") crane_beam_config_.beam_roi_y_min = param.as_double();
        else if (name == "crane_beam.beam_roi_y_max") crane_beam_config_.beam_roi_y_max = param.as_double();
        else if (name == "crane_beam.beam_roi_z_min") crane_beam_config_.beam_roi_z_min = param.as_double();
        else if (name == "crane_beam.beam_roi_z_max") crane_beam_config_.beam_roi_z_max = param.as_double();
        else if (name == "crane_beam.background_roi_x_min") crane_beam_config_.bg_roi_x_min = param.as_double();
        else if (name == "crane_beam.background_roi_x_max") crane_beam_config_.bg_roi_x_max = param.as_double();
        else if (name == "crane_beam.background_roi_y_min") crane_beam_config_.bg_roi_y_min = param.as_double();
        else if (name == "crane_beam.background_roi_y_max") crane_beam_config_.bg_roi_y_max = param.as_double();
        else if (name == "crane_beam.background_roi_z_min") crane_beam_config_.bg_roi_z_min = param.as_double();
        else if (name == "crane_beam.background_roi_z_max") crane_beam_config_.bg_roi_z_max = param.as_double();
        else if (name == "crane_beam.min_beam_roi_points") crane_beam_config_.min_beam_roi_points = static_cast<size_t>(param.as_int());
        else if (name == "crane_beam.min_background_roi_points") crane_beam_config_.min_bg_roi_points = static_cast<size_t>(param.as_int());
        else if (name == "crane_beam.min_beam_cluster_points") crane_beam_config_.min_beam_cluster_points = static_cast<size_t>(param.as_int());
        else if (name == "crane_beam.beam_z_band") crane_beam_config_.beam_z_band = param.as_double();
        else if (name == "crane_beam.hook_beam_dz_match_tolerance") crane_beam_config_.hook_beam_dz_match_tolerance = param.as_double();
        else if (name == "crane_beam.lift_dz_threshold") crane_beam_config_.lift_dz_threshold = param.as_double();
        else if (name == "crane_beam.lower_dz_threshold") crane_beam_config_.lower_dz_threshold = param.as_double();
        else if (name == "crane_beam.stop_dz_threshold") crane_beam_config_.stop_dz_threshold = param.as_double();
        else if (name == "crane_beam.ground_tolerance") crane_beam_config_.ground_tolerance = param.as_double();
        else if (name == "crane_beam.crane_move_start_x") crane_beam_config_.crane_move_start_x = param.as_double();
        else if (name == "crane_beam.crane_move_start_y") crane_beam_config_.crane_move_start_y = param.as_double();
        else if (name == "crane_beam.crane_move_stop_x") crane_beam_config_.crane_move_stop_x = param.as_double();
        else if (name == "crane_beam.crane_move_stop_y") crane_beam_config_.crane_move_stop_y = param.as_double();
        else if (name == "crane_beam.crane_move_stop_frames") crane_beam_config_.crane_move_stop_frames = param.as_int();
        else if (name == "crane_beam.lift_confirm_frames") crane_beam_config_.lift_confirm_frames = param.as_int();
        else if (name == "crane_beam.suspend_confirm_frames") crane_beam_config_.suspend_confirm_frames = param.as_int();
        else if (name == "crane_beam.lower_confirm_frames") crane_beam_config_.lower_confirm_frames = param.as_int();
        else if (name == "crane_beam.placed_confirm_frames") crane_beam_config_.placed_confirm_frames = param.as_int();
        else if (name == "crane_beam.complete_confirm_frames") crane_beam_config_.complete_confirm_frames = param.as_int();
        else if (name == "crane_beam.beam_measurement_max_lost_frames") crane_beam_config_.beam_measurement_max_lost_frames = param.as_int();
        else if (name == "crane_beam.length_stable_threshold") crane_beam_config_.length_stable_threshold = param.as_double();
        else if (name == "crane_beam.length_lock_frames") crane_beam_config_.length_lock_frames = param.as_int();
        else if (name == "crane_beam.cluster_bev_res") crane_beam_config_.cluster_bev_res = param.as_double();
        else if (name == "crane_beam.continuity_gate_xy") crane_beam_config_.continuity_gate_xy = param.as_double();
        else if (name == "crane_beam.landing_height_gap_threshold") crane_beam_config_.landing_height_gap_threshold = param.as_double();
        else if (name == "crane_beam.use_z_ema") crane_beam_config_.use_z_ema = param.as_bool();
        else if (name == "crane_beam.z_ema_alpha") crane_beam_config_.z_ema_alpha = param.as_double();
        else if (name == "crane_beam.beam_high_z_gate_enabled") crane_beam_config_.beam_high_z_gate_enabled = param.as_bool();
        else if (name == "crane_beam.beam_high_z_reject_threshold") crane_beam_config_.beam_high_z_reject_threshold = param.as_double();
        else if (name == "crane_beam.beam_high_z_require_cluster_valid") crane_beam_config_.beam_high_z_require_cluster_valid = param.as_bool();
        else if (name == "crane_beam.publish_debug") crane_beam_config_.publish_debug = param.as_bool();
        else if (name == "crane_beam.diagnostics_period_sec") crane_beam_config_.diagnostics_period_sec = param.as_double();
        else if (name == "crane_beam.freeze_sensor_transform") {
          crane_beam_config_.freeze_sensor_transform = param.as_bool();
          if (!crane_beam_config_.freeze_sensor_transform) {
            has_frozen_crane_transform_ = false;
          }
        }
        else if (name == "crane_beam.min_support_points") crane_beam_config_.min_support_points = static_cast<size_t>(param.as_int());
        else if (name == "crane_beam.beam_z_histogram_bin_size") crane_beam_config_.beam_z_histogram_bin_size = param.as_double();
        else if (name == "crane_beam.beam_ground_clearance") crane_beam_config_.beam_ground_clearance = param.as_double();
        else if (name == "crane_beam.tracking_gate_xy") crane_beam_config_.tracking_gate_xy = param.as_double();
        else if (name == "crane_beam.tracking_gate_z") crane_beam_config_.tracking_gate_z = param.as_double();
        else if (name == "crane_beam.tracking_gate_yaw") crane_beam_config_.tracking_gate_yaw = param.as_double();
        else if (name == "crane_beam.tracking_gate_length") crane_beam_config_.tracking_gate_length = param.as_double();
        else if (name == "crane_beam.support_search_margin_xy") crane_beam_config_.support_search_margin_xy = param.as_double();
        else if (name == "crane_beam.placement_tolerance") crane_beam_config_.placement_tolerance = param.as_double();
        else if (name == "crane_beam.beam_default_height") crane_beam_config_.beam_default_height = param.as_double();
        else if (name == "crane_beam.cumulative_motion_enabled") crane_beam_config_.cumulative_motion_enabled = param.as_bool();
        else if (name == "crane_beam.cumulative_lift_distance") crane_beam_config_.cumulative_lift_distance = param.as_double();
        else if (name == "crane_beam.cumulative_lower_distance") crane_beam_config_.cumulative_lower_distance = param.as_double();
        else if (name == "crane_beam.cumulative_motion_min_frames") crane_beam_config_.cumulative_motion_min_frames = param.as_int();
        else if (name == "crane_beam.cumulative_direction_epsilon") crane_beam_config_.cumulative_direction_epsilon = param.as_double();
        else if (name == "crane_beam.support_min_xy_extent") crane_beam_config_.support_min_xy_extent = param.as_double();
        else if (name == "crane_beam.support_max_z_spread") crane_beam_config_.support_max_z_spread = param.as_double();
        else if (name == "crane_beam.support_temporal_continuity") crane_beam_config_.support_temporal_continuity = param.as_bool();
        else if (name == "crane_beam.tilt_segmentation_enabled") crane_beam_config_.tilt_segmentation_enabled = param.as_bool();
        else if (name == "crane_beam.tilt_num_segments") crane_beam_config_.tilt_num_segments = param.as_int();
        else if (name == "crane_beam.min_supported_segment_ratio") crane_beam_config_.min_supported_segment_ratio = param.as_double();
        else if (name == "crane_beam.min_initial_cluster_length") crane_beam_config_.min_initial_cluster_length = param.as_double();
        else if (name == "crane_beam.min_initial_cluster_elongation") crane_beam_config_.min_initial_cluster_elongation = param.as_double();
        else if (name == "crane_beam.hook.enabled") crane_beam_config_.hook.enabled = param.as_bool();
        else if (name == "crane_beam.hook.tracking_enabled") crane_beam_config_.hook.tracking_enabled = param.as_bool();
        else if (name == "crane_beam.hook.tracking_confirm_frames") crane_beam_config_.hook.tracking_confirm_frames = param.as_int();
        else if (name == "crane_beam.hook.tracking_gate_xy") crane_beam_config_.hook.tracking_gate_xy = param.as_double();
        else if (name == "crane_beam.hook.tracking_gate_z") crane_beam_config_.hook.tracking_gate_z = param.as_double();
        else if (name == "crane_beam.hook.tracking_gate_z_max") crane_beam_config_.hook.tracking_gate_z_max = param.as_double();
        else if (name == "crane_beam.hook.tracking_max_lost_frames") crane_beam_config_.hook.tracking_max_lost_frames = param.as_int();
        else if (name == "crane_beam.hook.tracking_prediction_timeout_sec") crane_beam_config_.hook.tracking_prediction_timeout_sec = param.as_double();
        else if (name == "crane_beam.hook.tracking_accel_std_xy") crane_beam_config_.hook.tracking_accel_std_xy = param.as_double();
        else if (name == "crane_beam.hook.tracking_accel_std_z") crane_beam_config_.hook.tracking_accel_std_z = param.as_double();
        else if (name == "crane_beam.hook.tracking_measurement_std") crane_beam_config_.hook.tracking_measurement_std = param.as_double();
        else if (name == "crane_beam.hook.roi_left_x_min") crane_beam_config_.hook.roi_left_x_min = param.as_double();
        else if (name == "crane_beam.hook.roi_left_x_max") crane_beam_config_.hook.roi_left_x_max = param.as_double();
        else if (name == "crane_beam.hook.roi_right_x_min") crane_beam_config_.hook.roi_right_x_min = param.as_double();
        else if (name == "crane_beam.hook.roi_right_x_max") crane_beam_config_.hook.roi_right_x_max = param.as_double();
        else if (name == "crane_beam.hook.roi_y_min") crane_beam_config_.hook.roi_y_min = param.as_double();
        else if (name == "crane_beam.hook.roi_y_max") crane_beam_config_.hook.roi_y_max = param.as_double();
        else if (name == "crane_beam.hook.roi_z_min") crane_beam_config_.hook.roi_z_min = param.as_double();
        else if (name == "crane_beam.hook.roi_z_max") crane_beam_config_.hook.roi_z_max = param.as_double();
        else if (name == "crane_beam.hook.window_size_z") crane_beam_config_.hook.window_size_z = param.as_double();
        else if (name == "crane_beam.hook.window_step_z") crane_beam_config_.hook.window_step_z = param.as_double();
        else if (name == "crane_beam.hook.min_body_points") crane_beam_config_.hook.min_body_points = static_cast<size_t>(param.as_int());
        else if (name == "crane_beam.hook.ground_search_z_min") crane_beam_config_.hook.ground_search_z_min = param.as_double();
        else if (name == "crane_beam.hook.ground_search_z_max") crane_beam_config_.hook.ground_search_z_max = param.as_double();
        else if (name == "crane_beam.hook.ground_bin_size") crane_beam_config_.hook.ground_bin_size = param.as_double();
        else if (name == "crane_beam.hook.min_ground_points") crane_beam_config_.hook.min_ground_points = static_cast<size_t>(param.as_int());
        else if (name == "crane_beam.hook.prepare_distance") crane_beam_config_.hook.prepare_distance = param.as_double();
        else if (name == "crane_beam.hook.prepare_release_margin") crane_beam_config_.hook.prepare_release_margin = param.as_double();
        else if (name == "crane_beam.hook.prepare_confirm_frames") crane_beam_config_.hook.prepare_confirm_frames = param.as_int();

        if (crane_beam_detector_) {
          crane_beam_detector_->updateConfig(crane_beam_config_);
        }
      }
    }

    // Apply values if all validation checks passed
    foreground_classifier_mode_ = new_mode;
    foreground_pdf_density_threshold_ = new_pdf_threshold;
    foreground_voxel_diff_threshold_ = new_voxel_diff;
    foreground_min_persistence_frames_ = new_min_persistence;
    enable_radius_outlier_removal_ = new_ror_enabled;
    ror_radius_ = new_ror_radius;
    ror_min_neighbors_ = new_ror_min_neighbors;
    enable_parallel_processing_ = new_enable_parallel;
    parallel_threads_ = new_parallel_threads;
    vehicle_stopped_duration_sec_ = new_vehicle_stopped_duration_sec;
    if (new_raw_vehicle_box_refinement_enabled != raw_vehicle_box_refinement_enabled_) {
      raw_vehicle_box_refinement_enabled_ = new_raw_vehicle_box_refinement_enabled;
      RCLCPP_INFO(get_logger(), "Raw vehicle box refinement %s",
          raw_vehicle_box_refinement_enabled_ ? "enabled" : "disabled");
    }

    if (new_z_profile_enabled != cluster_z_profile_diagnostics_enabled_ ||
        new_z_profile_bin != cluster_z_profile_bin_size_) {
      cluster_z_profile_diagnostics_enabled_ = new_z_profile_enabled;
      cluster_z_profile_bin_size_ = new_z_profile_bin;
      clusterer_ = createClusterer();
    }

    return result;
  }

  bool writeTrackCsv(const std::string & path) const {
    std::ofstream f(path);
    if (!f.is_open()) return false;
    f << trackTableCsvHeader() << "\n";
    for (const auto & tr : tracks_) f << trackRow(tr) << "\n";
    return true;
  }

  bool writeCandidateCsv(const std::string & path) const {
    std::ofstream f(path);
    if (!f.is_open()) return false;
    f << "candidate_id,cluster_id,class_score_vehicle,class_score_person,class_score_unknown,"
         "selected_class,rejection_reason,centroid_x,centroid_y,centroid_z,distance,length,width,"
         "height,point_count,cell_count,density,ring_coverage,mean_intensity,background_similarity,"
         "dynamic_score,foreground_ratio,is_static_background,is_dynamic_candidate,inside_roi,"
         "component_type,component_type_confidence,component_type_reason,"
         "shape_node_count,shape_edge_count,shape_graph_score,shape_memory_match_score,"
         "visibility_reason,observable_fraction,occluded_fraction,supported_fraction,"
         "sensor_distance,azimuth_deg,elevation_deg,in_horizontal_fov,in_vertical_fov,"
         "in_valid_range,near_blind_zone,expected_samples,observable_samples,occluded_samples,"
         "supported_samples,free_samples,unknown_samples,visibility_confidence\n";
    for (const auto & c : last_candidates_) {
      f << c.id << "," << c.id << ",0,0," << c.dynamic_score << ",UNKNOWN,"
        << csvSafe(c.rejection_reason) << "," << c.centroid_x << "," << c.centroid_y << ","
        << c.centroid_z << "," << c.range << "," << c.length << "," << c.width << ","
        << c.height << "," << c.point_count << "," << c.cell_count << "," << c.density << ","
        << c.ring_coverage << "," << c.mean_intensity << "," << c.background_similarity << ","
        << c.dynamic_score << "," << c.dynamic_score << ",0," << (c.dynamic_score > 0.0)
        << "," << c.inside_roi << "," << csvSafe(c.component_type) << ","
        << c.component_type_confidence << "," << csvSafe(c.component_type_reason)
        << "," << c.shape_node_count << "," << c.shape_edge_count
        << "," << c.shape_graph_score << "," << c.shape_memory_match_score << ","
        << visibilityReasonName(c.visibility.reason) << "," << c.visibility.observable_fraction
        << "," << c.visibility.occluded_fraction << "," << c.visibility.supported_fraction
        << "," << c.visibility.sensor_distance << "," << c.visibility.azimuth_deg
        << "," << c.visibility.elevation_deg << "," << c.visibility.in_horizontal_fov
        << "," << c.visibility.in_vertical_fov << "," << c.visibility.in_valid_range
        << "," << c.visibility.near_blind_zone << "," << c.visibility.expected_samples
        << "," << c.visibility.observable_samples << "," << c.visibility.occluded_samples
        << "," << c.visibility.supported_samples << "," << c.visibility.free_samples
        << "," << c.visibility.unknown_samples << "," << c.visibility.confidence << "\n";
    }
    return true;
  }

  double computeEndpointMatchScoreOnly(const Track & tr, const ObjectFeature & f) const {
    if (!tr.shape_memory.valid) return 0.0;
    const auto & mem = tr.shape_memory;
    int matched_relations = 0;
    int active_relations = 0;
    for (const auto & rel : mem.endpoint_relations) {
      const ShapeMemoryNode* mn_a = nullptr;
      const ShapeMemoryNode* mn_b = nullptr;
      for (const auto & mn : mem.nodes) {
        if (mn.id == rel.endpoint_a) mn_a = &mn;
        if (mn.id == rel.endpoint_b) mn_b = &mn;
      }
      if (!mn_a || !mn_b) continue;

      active_relations++;

      const ShapeNode* on_a = nullptr;
      const ShapeNode* on_b = nullptr;
      double dist_a = std::numeric_limits<double>::max();
      double dist_b = std::numeric_limits<double>::max();

      geometry_msgs::msg::Point world_a = transformMemoryNodeToWorld(tr, *mn_a);
      geometry_msgs::msg::Point world_b = transformMemoryNodeToWorld(tr, *mn_b);

      for (const auto & on : f.shape_nodes) {
        double dx_a = on.x - world_a.x;
        double dy_a = on.y - world_a.y;
        double dz_a = on.z - world_a.z;
        double d_a = std::sqrt(dx_a*dx_a + dy_a*dy_a + dz_a*dz_a);
        if (d_a < dist_a && d_a < 1.0) {
          dist_a = d_a;
          on_a = &on;
        }
        double dx_b = on.x - world_b.x;
        double dy_b = on.y - world_b.y;
        double dz_b = on.z - world_b.z;
        double d_b = std::sqrt(dx_b*dx_b + dy_b*dy_b + dz_b*dz_b);
        if (d_b < dist_b && d_b < 1.0) {
          dist_b = d_b;
          on_b = &on;
        }
      }

      if (on_a && on_b) {
        double cur_dx = on_b->x - on_a->x;
        double cur_dy = on_b->y - on_a->y;
        double cur_dz = on_b->z - on_a->z;
        double cur_dist = std::sqrt(cur_dx*cur_dx + cur_dy*cur_dy + cur_dz*cur_dz);
        double cur_yaw = std::atan2(cur_dy, cur_dx);
        double cur_h = std::abs(cur_dz);

        double dist_diff = std::abs(cur_dist - rel.expected_distance);
        double yaw_diff = std::abs(cur_yaw - rel.expected_direction_yaw);
        if (yaw_diff > M_PI) yaw_diff = 2.0 * M_PI - yaw_diff;
        double h_diff = std::abs(cur_h - rel.expected_height_delta);

        if (dist_diff < 0.8 && yaw_diff < 0.5 && h_diff < 0.5) {
          matched_relations++;
        }
      }
    }
    return active_relations == 0 ? 1.0 : static_cast<double>(matched_relations) / active_relations;
  }

  double computeEdgeMatchScoreOnly(const Track & tr, const ObjectFeature & f) const {
    if (!tr.shape_memory.valid) return 0.0;
    const auto & mem = tr.shape_memory;
    int matched_edges = 0;
    int active_edges = 0;
    for (const auto & edge : mem.edges) {
      const ShapeMemoryNode* mn_a = nullptr;
      const ShapeMemoryNode* mn_b = nullptr;
      for (const auto & mn : mem.nodes) {
        if (mn.id == edge.node_a) mn_a = &mn;
        if (mn.id == edge.node_b) mn_b = &mn;
      }
      if (!mn_a || !mn_b) continue;

      active_edges++;

      const ShapeNode* on_a = nullptr;
      const ShapeNode* on_b = nullptr;
      double dist_a = std::numeric_limits<double>::max();
      double dist_b = std::numeric_limits<double>::max();

      geometry_msgs::msg::Point world_a = transformMemoryNodeToWorld(tr, *mn_a);
      geometry_msgs::msg::Point world_b = transformMemoryNodeToWorld(tr, *mn_b);

      for (const auto & on : f.shape_nodes) {
        double dx_a = on.x - world_a.x;
        double dy_a = on.y - world_a.y;
        double dz_a = on.z - world_a.z;
        double d_a = std::sqrt(dx_a*dx_a + dy_a*dy_a + dz_a*dz_a);
        if (d_a < dist_a && d_a < 1.0) {
          dist_a = d_a;
          on_a = &on;
        }
        double dx_b = on.x - world_b.x;
        double dy_b = on.y - world_b.y;
        double dz_b = on.z - world_b.z;
        double d_b = std::sqrt(dx_b*dx_b + dy_b*dy_b + dz_b*dz_b);
        if (d_b < dist_b && d_b < 1.0) {
          dist_b = d_b;
          on_b = &on;
        }
      }

      if (on_a && on_b) {
        double cur_dx = on_b->x - on_a->x;
        double cur_dy = on_b->y - on_a->y;
        double cur_dz = on_b->z - on_a->z;
        double cur_dist = std::sqrt(cur_dx*cur_dx + cur_dy*cur_dy + cur_dz*cur_dz);
        double cur_yaw = std::atan2(cur_dy, cur_dx);
        double cur_h = std::abs(cur_dz);

        double dist_diff = std::abs(cur_dist - edge.length);
        double yaw_diff = std::abs(cur_yaw - edge.direction_yaw);
        if (yaw_diff > M_PI) yaw_diff = 2.0 * M_PI - yaw_diff;
        double h_diff = std::abs(cur_h - edge.height_delta);

        if (dist_diff < 0.8 && yaw_diff < 0.5 && h_diff < 0.5) {
          matched_edges++;
        }
      }
    }
    return active_edges == 0 ? 1.0 : static_cast<double>(matched_edges) / active_edges;
  }

  std::string trackTableCsvHeader() const {
    return "id,class_name,lifecycle_state,motion_state,vehicle_status,radial_motion_state,x,y,z,vx,vy,speed,"
           "distance,radial_velocity,range_slope,length,width,height,point_count,cell_count,"
           "ring_coverage,dynamic_score,background_similarity,detector_score,confidence,"
           "decision_reason,rejection_reason,shape_memory_valid,shape_memory_confidence,"
           "component_type,component_type_confidence,component_type_reason,"
           "shape_node_count,shape_edge_count,shape_graph_score,shape_memory_match_score,"
           "shape_memory_committed,shape_memory_frozen,shape_memory_node_count,shape_memory_edge_count,"
           "shape_memory_endpoint_relation_count,shape_memory_endpoint_match_score,shape_memory_edge_match_score,"
           "shape_memory_state,shape_memory_class_label,shape_memory_class_prior,shape_memory_class_confidence,"
           "shape_memory_source_component_type,shape_memory_applicable_component_types,"
           "shape_memory_mirror_plane,shape_memory_mirror_flipped_axis,"
           "shape_memory_mirror_center_x,shape_memory_mirror_center_y,shape_memory_mirror_center_z,"
           "shape_memory_mirror_flipped_axis_center,shape_memory_mirror_observed_node_count,"
           "shape_memory_mirror_virtual_node_count,shape_memory_mirror_observed_edge_count,"
           "shape_memory_mirror_virtual_edge_count,shape_memory_mirror_near_identical_warning_count,"
           "shape_memory_mirror_invalid_edge_warning_count,"
           "shape_memory_bbox_score,shape_memory_node_score,shape_memory_endpoint_relation_score,"
           "shape_memory_edge_score,shape_memory_height_score,shape_memory_final_score,"
           "final_association_score,shape_memory_reprepare_needed,observed_shape_cache_state,"
           "observed_shape_cache_captured_frames,observed_shape_cache_target_frames,"
           "observed_shape_cache_point_count,object_memory_id,object_memory_class_label,"
           "object_memory_bank_match_score,object_memory_applicable_match,"
           "object_memory_skipped_by_component_type,object_memory_skip_reason,"
           "visibility_reason,sensor_distance,azimuth_deg,elevation_deg,in_horizontal_fov,"
           "in_vertical_fov,in_valid_range,near_blind_zone,expected_samples,observable_samples,"
           "occluded_samples,supported_samples,free_samples,unknown_samples,observable_fraction,"
           "occluded_fraction,supported_fraction,visibility_confidence,lost_frames,track_hits,"
           "adaptive_expected_rays,adaptive_supported_rays,adaptive_azimuth_width_deg,"
           "adaptive_elevation_width_deg,horizontal_point_spacing_m,vertical_point_spacing_m,"
           "current_observation_weight,memory_weight,reliable_geometry,reliable_length,"
           "reliable_width,reliable_height,geometry_confidence,observation_age_sec,missed_time_sec,"
           "miss_reason,length_evidence,width_evidence,height_evidence,"
           "track_position_sigma_m,reliable_expected_rays,"
           "vehicle_quality,predicted_x,predicted_y,predicted_yaw,vehicle_length,vehicle_width,"
           "observed_length,observed_width,expected_visible_samples,visible_supported_samples,"
           "owned_component_count,sparse_support_count,measurement_std_used,geometry_update,association_source,"
           "raw_centroid_x,raw_centroid_y,measured_x,measured_y,reliable_lateral_y,"
           "lateral_innovation,measurement_std_x,measurement_std_y,"
           "lateral_measurement_accepted,lateral_update_reason,actual_y_update,"
           "measured_yaw,yaw_innovation,yaw_update_accepted,yaw_update_reason,"
           "actual_yaw_update,anchor_updated,frame_stamp,vehicle_state,lost_age_sec,"
           "last_observed_x,last_observed_y,current_predicted_x,current_predicted_y,"
           "final_predicted_x,final_predicted_y,final_predicted_pose_valid,dormant_age_sec,identity_pose_source,"
           "last_seen_stamp,last_reliable_stamp,last_known_distance,last_observed_quality,"
           "reacquire_best_score,reacquire_second_score,reacquire_position_error,"
           "reacquire_lateral_error,reacquire_yaw_error,reacquire_result,"
           "fifo_entry_sequence,fifo_entry_timestamp,fifo_phase,fifo_direction,fifo_front,"
           "confirmed_vehicle,measurement_valid,associated_cluster_id,associated_point_count,"
           "association_distance,configured_association_gate_m,reacquired,dormant,active_lost_limit,tracking_policy,physical_output,fifo_exit_observed,raw_box_used,raw_box_point_count,raw_box_foreground_count,raw_box_reason";
  }

  std::string trackRow(const Track & tr) const {
    const auto o = toMsg(tr);
    std::ostringstream ss;
    ss << std::setprecision(17); // Preserve sensor timestamps and observed/predicted coordinates.
    ss << o.id << "," << o.class_name << "," << o.lifecycle_state << "," << o.motion_state
       << "," << o.vehicle_status << "," << o.radial_motion_state << "," << o.position.x << "," << o.position.y << ","
       << o.position.z << "," << o.velocity.x << "," << o.velocity.y << "," << o.speed << ","
       << o.distance << "," << o.radial_velocity << "," << o.range_slope << ","
       << o.dimensions.x << "," << o.dimensions.y << "," << o.dimensions.z << ","
       << o.point_count << "," << o.cell_count << "," << o.ring_coverage << ","
       << tr.feature.dynamic_score << "," << tr.feature.background_similarity << ","
       << o.confidence << "," << o.confidence << "," << csvSafe(o.decision_reason) << ","
       << csvSafe(o.rejection_reason) << "," << tr.shape_memory.valid << ","
       << tr.shape_memory.confidence << ","
       << csvSafe(tr.feature.component_type) << ","
       << tr.feature.component_type_confidence << ","
       << csvSafe(tr.feature.component_type_reason) << ","
       << tr.feature.shape_node_count << ","
       << tr.feature.shape_edge_count << "," << tr.feature.shape_graph_score << ","
       << tr.feature.shape_memory_match_score << ","
       << tr.shape_memory.committed << ","
       << tr.shape_memory.frozen << ","
       << tr.shape_memory.nodes.size() << ","
       << tr.shape_memory.edges.size() << ","
       << tr.shape_memory.endpoint_relations.size() << ","
       << computeEndpointMatchScoreOnly(tr, tr.feature) << ","
       << computeEdgeMatchScoreOnly(tr, tr.feature) << ","
       << (tr.shape_memory.committed ? "committed" : (tr.shape_memory.valid ? "learned" : "empty")) << ","
       << csvSafe(tr.shape_memory.class_label) << ","
       << csvSafe(tr.shape_memory.class_prior) << ","
       << tr.shape_memory.class_confidence << ","
       << csvSafe(tr.shape_memory.source_component_type) << ","
       << csvSafe(joinStrings(tr.shape_memory.applicable_component_types)) << ","
       << csvSafe(tr.shape_memory.mirror_info.plane) << ","
       << csvSafe(tr.shape_memory.mirror_info.flipped_axis) << ","
       << tr.shape_memory.mirror_info.center_x << ","
       << tr.shape_memory.mirror_info.center_y << ","
       << tr.shape_memory.mirror_info.center_z << ","
       << tr.shape_memory.mirror_info.flipped_axis_center << ","
       << tr.shape_memory.mirror_info.observed_node_count << ","
       << tr.shape_memory.mirror_info.mirrored_node_count << ","
       << tr.shape_memory.mirror_info.observed_edge_count << ","
       << tr.shape_memory.mirror_info.mirrored_edge_count << ","
       << tr.shape_memory.mirror_info.near_identical_warning_count << ","
       << tr.shape_memory.mirror_info.invalid_edge_warning_count << ","
       << tr.feature.shape_memory_bbox_score << ","
       << tr.feature.shape_memory_node_score << ","
       << tr.feature.shape_memory_endpoint_relation_score << ","
       << tr.feature.shape_memory_edge_score << ","
       << tr.feature.shape_memory_height_score << ","
       << tr.feature.shape_memory_final_score << ","
       << tr.feature.final_association_score << ","
       << tr.shape_memory.reprepare_needed << ","
       << csvSafe(tr.observed_shape_cache.state) << ","
       << tr.observed_shape_cache.captured_frames << ","
       << tr.observed_shape_cache.target_capture_frames << ","
       << tr.observed_shape_cache.accumulated_local_points.size() << ","
       << csvSafe(tr.feature.object_memory_id) << ","
       << csvSafe(tr.feature.object_memory_class_label) << ","
       << tr.feature.object_memory_bank_match_score << ","
       << tr.feature.object_memory_applicable_match << ","
       << tr.feature.object_memory_skipped_by_component_type << ","
       << csvSafe(tr.feature.object_memory_skip_reason);
    const auto & vis = tr.visibility;
    ss << "," << visibilityReasonName(vis.reason) << "," << vis.sensor_distance
       << "," << vis.azimuth_deg << "," << vis.elevation_deg
       << "," << vis.in_horizontal_fov << "," << vis.in_vertical_fov
       << "," << vis.in_valid_range << "," << vis.near_blind_zone
       << "," << vis.expected_samples << "," << vis.observable_samples
       << "," << vis.occluded_samples << "," << vis.supported_samples
       << "," << vis.free_samples << "," << vis.unknown_samples
       << "," << vis.observable_fraction << "," << vis.occluded_fraction
       << "," << vis.supported_fraction << "," << vis.confidence
       << "," << tr.lost_frames << "," << tr.hits
       << "," << vis.adaptive_expected_rays << "," << vis.adaptive_supported_rays
       << "," << vis.adaptive_azimuth_width_deg << "," << vis.adaptive_elevation_width_deg
       << "," << vis.horizontal_point_spacing_m << "," << vis.vertical_point_spacing_m
       << "," << vis.current_observation_weight << "," << vis.memory_weight
       << "," << tr.observation_memory.reliable
       << "," << tr.observation_memory.reliable_length
       << "," << tr.observation_memory.reliable_width
       << "," << tr.observation_memory.reliable_height
       << "," << tr.observation_memory.geometry_confidence
       << "," << tr.observation_memory.observation_age_sec
       << "," << tr.observation_memory.missed_time_sec
       << "," << visibilityReasonName(tr.observation_memory.miss_reason)
       << "," << geometryEvidenceName(tr.observation_memory.length_evidence)
       << "," << geometryEvidenceName(tr.observation_memory.width_evidence)
       << "," << geometryEvidenceName(tr.observation_memory.height_evidence)
       << "," << trackPositionSigma(tr)
       << "," << tr.observation_memory.reliable_expected_rays;
    const auto & vm = tr.observation_memory;
    ss << "," << vehicleQualityName(vm.vehicle_quality)
       << "," << (persistentVehicle(tr) ? vm.predicted_x : outputX(tr))
       << "," << (persistentVehicle(tr) ? vm.predicted_y : outputY(tr))
       << "," << (persistentVehicle(tr) ? vm.predicted_yaw : vm.vehicle_obb.yaw)
       << "," << vm.vehicle_obb.length << "," << vm.vehicle_obb.width
       << "," << tr.feature.length << "," << tr.feature.width
       << "," << vm.expected_visible_samples << "," << vm.visible_supported_samples
       << "," << vm.owned_component_count << "," << vm.sparse_support_count
       << "," << vm.measurement_std_used << "," << vm.geometry_update
       << "," << vm.association_source
       << "," << vm.raw_centroid_x << "," << vm.raw_centroid_y
       << "," << vm.measured_x << "," << vm.measured_y
       << "," << vm.lateral_anchor.y << "," << vm.lateral_innovation
       << "," << vm.measurement_std_x << "," << vm.measurement_std_y
       << "," << vm.lateral_measurement_accepted << "," << vm.lateral_update_reason
       << "," << (persistentVehicle(tr) ? outputY(tr) - vm.predicted_y : 0.0)
       << "," << vm.measured_yaw << "," << vm.yaw_innovation
       << "," << vm.yaw_update_accepted << "," << vm.yaw_update_reason
       << "," << (persistentVehicle(tr) ?
                    cx128s2_bev_perception::vehicle_surface::angleDifference(
                        vm.vehicle_obb.yaw, vm.predicted_yaw) : 0.0)
       << "," << vm.anchor_updated << "," << vm.frame_stamp
       << "," << (dormantVehicle(tr) ? "DORMANT" :
           (predictiveLostVehicle(tr) ? "LOST_PREDICTIVE" : "ACTIVE"))
       << "," << (predictiveLostVehicle(tr) ? std::max(0.0, vm.frame_stamp - vm.lost_since) : 0.0)
       << "," << vm.last_observed_x << "," << vm.last_observed_y
       << "," << outputX(tr) << "," << outputY(tr)
       << "," << vm.final_predicted_x << "," << vm.final_predicted_y
       << "," << vm.final_predicted_valid
       << "," << (dormantVehicle(tr) ? std::max(0.0, vm.frame_stamp - vm.dormant_since) : 0.0)
       << "," << (dormantVehicle(tr) ? "FINAL_PREDICTED_FROZEN" :
           (predictiveLostVehicle(tr) ? "KF_PREDICTION" : "CURRENT_OBSERVATION"))
       << "," << vm.last_seen_stamp << "," << vm.last_reliable_stamp
       << "," << vm.last_known_distance << "," << vehicleQualityName(vm.last_observed_quality)
       << "," << vm.reacquire_best_score << "," << vm.reacquire_second_score
       << "," << vm.reacquire_position_error << "," << vm.reacquire_lateral_error
       << "," << vm.reacquire_yaw_error << "," << vm.reacquire_result
       << "," << vm.entry_sequence << "," << vm.entry_timestamp
       << "," << cx128s2_bev_perception::vehicle_surface::fifoPhaseName(vm.fifo_phase)
       << "," << cx128s2_bev_perception::vehicle_surface::directionName(
           vm.corridor_direction.stable)
       << "," << (inside_vehicle_queue_.front() == tr.id);
    const auto observed = recording_track_features_.find(tr.id);
    const auto * feature = tr.lost_frames == 0 && observed != recording_track_features_.end() ?
        observed->second : nullptr;
    ss << "," << persistentVehicle(tr)
       << "," << (tr.lost_frames == 0 && (feature || vm.measured_center_valid))
       << "," << (feature ? feature->id : -1)
       << "," << (feature ? feature->point_count : 0)
       << "," << (feature ? std::hypot(feature->centroid_x - vm.predicted_x,
                                      feature->centroid_y - vm.predicted_y) :
                  std::numeric_limits<double>::quiet_NaN())
       << "," << gateForRange(feature ? feature->range : trackSensorRange(tr))
       << "," << (vm.reacquire_result == "REACQUIRED")
       << "," << dormantVehicle(tr) << "," << maxLostForTrack(tr);
    ss << "," << (persistentVehicle(tr) ? "CONFIRMED_SURFACE" :
              memoryMatchingBlocked(trackSensorRange(tr)) ? "FAR_MEMORY_NO_MATCH" : (kalmanEnabled() ? "MID_NEAR_MEMORY" : "LEGACY"))
       << "," << physicalOutputAllowed(tr)
       << "," << vm.fifo_exit_observed
       << "," << (feature && feature->raw_box_used)
       << "," << (feature ? feature->raw_box_point_count : 0)
       << "," << (feature ? feature->raw_box_foreground_count : 0)
       << "," << (feature ? feature->raw_box_reason : "NO_CURRENT_OBSERVATION");
    return ss.str();
  }

  void publishTables(const std_msgs::msg::Header & header) {
    if (!publish_debug_topics_) return;
    (void)header;
    const double t = now().seconds();
    const auto profile_diag = clusterer_ ? clusterer_->diagnostics() : ClusterDiagnostics{};
    auto formatArray = [](const auto & arr) {
      std::ostringstream oss;
      oss << "[";
      for (size_t i = 0; i < arr.size(); ++i) {
        if (i > 0) oss << ",";
        oss << arr[i];
      }
      oss << "]";
      return oss.str();
    };
    if (publish_diagnostics_table_ && t - last_diag_publish_time_ >= diagnostics_table_period_sec_) {
      last_diag_publish_time_ = t;
      int observed_cache_capturing = 0;
      int observed_cache_ready = 0;
      int observed_cache_points = 0;
      int reprepare_tracks = 0;
      double mirror_centerline_y = 0.0;
      double mirror_centerline = 0.0;
      double mirror_centerline_x = 0.0;
      double mirror_centerline_z = 0.0;
      double mirror_center_x = 0.0;
      double mirror_center_y = 0.0;
      double mirror_center_z = 0.0;
      double mirror_flipped_axis_center = 0.0;
      double mirror_symmetry_confidence = 0.0;
      int mirror_observed_node_count = 0;
      int mirror_virtual_node_count = 0;
      int mirror_observed_edge_count = 0;
      int mirror_virtual_edge_count = 0;
      int mirror_near_identical_warning_count = 0;
      int mirror_invalid_edge_warning_count = 0;
      int mirror_info_count = 0;
      std::string mirror_plane = shape_memory_mirror_plane_;
      std::string mirror_flipped_axis;
      int object_memory_skipped_by_component_type = 0;
      for (const auto & tr : tracks_) {
        observed_cache_points += static_cast<int>(tr.observed_shape_cache.accumulated_local_points.size());
        if (tr.observed_shape_cache.state == "capturing") observed_cache_capturing++;
        if (tr.observed_shape_cache.state == "ready") observed_cache_ready++;
        if (tr.shape_memory.reprepare_needed) reprepare_tracks++;
        if (tr.feature.object_memory_skipped_by_component_type) {
          object_memory_skipped_by_component_type++;
        }
        if (tr.shape_memory.mirror_info.enabled) {
          mirror_plane = tr.shape_memory.mirror_info.plane;
          mirror_flipped_axis = tr.shape_memory.mirror_info.flipped_axis;
          mirror_centerline += tr.shape_memory.mirror_info.mirror_centerline;
          mirror_centerline_x += tr.shape_memory.mirror_info.mirror_centerline_x;
          mirror_centerline_y += tr.shape_memory.mirror_info.mirror_centerline_y;
          mirror_centerline_z += tr.shape_memory.mirror_info.mirror_centerline_z;
          mirror_center_x += tr.shape_memory.mirror_info.center_x;
          mirror_center_y += tr.shape_memory.mirror_info.center_y;
          mirror_center_z += tr.shape_memory.mirror_info.center_z;
          mirror_flipped_axis_center += tr.shape_memory.mirror_info.flipped_axis_center;
          mirror_symmetry_confidence += tr.shape_memory.mirror_info.symmetry_confidence;
          mirror_observed_node_count += tr.shape_memory.mirror_info.observed_node_count;
          mirror_virtual_node_count += tr.shape_memory.mirror_info.mirrored_node_count;
          mirror_observed_edge_count += tr.shape_memory.mirror_info.observed_edge_count;
          mirror_virtual_edge_count += tr.shape_memory.mirror_info.mirrored_edge_count;
          mirror_near_identical_warning_count += tr.shape_memory.mirror_info.near_identical_warning_count;
          mirror_invalid_edge_warning_count += tr.shape_memory.mirror_info.invalid_edge_warning_count;
          mirror_info_count++;
        }
      }
      if (mirror_info_count > 0) {
        mirror_centerline /= mirror_info_count;
        mirror_centerline_x /= mirror_info_count;
        mirror_centerline_y /= mirror_info_count;
        mirror_centerline_z /= mirror_info_count;
        mirror_center_x /= mirror_info_count;
        mirror_center_y /= mirror_info_count;
        mirror_center_z /= mirror_info_count;
        mirror_flipped_axis_center /= mirror_info_count;
        mirror_symmetry_confidence /= mirror_info_count;
      }
      std_msgs::msg::String s;
      std::ostringstream ss;
      ss << "timestamp: " << std::fixed << std::setprecision(3) << t << "\n"
         << "input_frame_id: " << last_input_frame_id_ << "\n"
         << "target_frame: " << last_target_frame_ << "\n"
         << "raw_vehicle_box_refinement_enabled: " << raw_vehicle_box_refinement_enabled_ << "\n"
         << "raw_vehicle_box_refine_ms: " << raw_vehicle_box_refine_ms_ << "\n"
         << "raw_vehicle_box_queries: " << raw_vehicle_box_queries_ << "\n"
         << "raw_vehicle_box_used: " << raw_vehicle_box_used_ << "\n"
         << "raw_vehicle_box_scanned_points: " << raw_vehicle_box_stats_.scanned_points << "\n"
         << "raw_vehicle_box_box_tests: " << raw_vehicle_box_stats_.box_tests << "\n"
         << "input_point_count: " << last_input_count_ << "\n"
         << "transformed_point_count: " << last_transformed_count_ << "\n"
         << "valid_point_count: " << last_valid_count_ << "\n"
         << "ring_min: " << last_ring_min_ << "\nring_max: " << last_ring_max_ << "\n"
         << "range_min: " << last_range_min_ << "\nrange_max: " << last_range_max_ << "\n"
         << "background_ready: " << background_ready_ << "\nlearning_active: " << learning_active_
         << "\nbackground_learning_input_mode: " << background_learning_input_mode_
         << "\nuse_roi_policy: " << use_roi_policy_
         << "\nroi_policy_file: " << roi_policy_file_
         << "\nroi_zone_count: " << roi_zones_.size()
         << "\nwatch_roi_count: " << countRoiZones("watch")
         << "\nignore_roi_count: " << countRoiZones("ignore")
         << "\nhard_ignore_roi_count: " << countRoiZones("hard_ignore")
         << "\nroi_filtered_point_count: " << last_roi_filtered_count_
         << "\nroi_ignored_point_count: " << last_roi_ignored_count_
         << "\nroi_hard_ignored_point_count: " << last_roi_hard_ignored_count_
         << "\ndiscard_points_outside_watch_roi: " << discard_points_outside_watch_roi_
         << "\napply_ignore_before_learning: " << apply_ignore_before_learning_
         << "\nroi_input_mode: " << roiInputModeString()
         << "\nlearned_frame_count: " << learned_frame_count_
         << "\nbackground_valid_cell_count: " << background_map_.size()
         << "\nground_point_count: " << last_ground_count_
         << "\nobstacle_point_count: " << last_obstacle_count_
         << "\nignored_point_count: " << last_ignored_count_
         << "\nforeground_point_count: " << last_foreground_count_
         << "\nstatic_background_point_count: " << last_static_count_
         << "\ndynamic_point_count: " << last_dynamic_count_
         << "\nforeground_count_diff_threshold: " << foreground_count_diff_threshold_
         << "\nbackground_height_diff_threshold: " << background_height_diff_threshold_
         << "\nforeground_min_persistence_frames: " << foreground_min_persistence_frames_
         << "\nforeground_classifier_mode: " << foreground_classifier_mode_
         << "\nforeground_score_threshold: " << foreground_score_threshold_
         << "\nforeground_pdf_density_threshold: " << foreground_pdf_density_threshold_
         << "\nforeground_voxel_diff_threshold: " << foreground_voxel_diff_threshold_
         << "\ngdg_pdf_foreground_cell_count: " << last_gdg_pdf_foreground_cell_count_
         << "\ngdg_pdf_tested_cell_count: " << last_gdg_pdf_tested_cell_count_
         << "\ngdg_pdf_tested_point_count: " << last_gdg_pdf_tested_point_count_
         << "\ngdg_pdf_foreground_point_count: " << last_gdg_pdf_foreground_point_count_
         << "\ngdg_pdf_background_point_count: " << last_gdg_pdf_background_point_count_
         << "\ngdg_pdf_unknown_cell_point_count: " << last_gdg_pdf_unknown_cell_point_count_
         << "\ngdg_pdf_count_diff_rejected_cell_count: " << last_gdg_pdf_count_diff_rejected_cell_count_
         << "\nforeground_persistence_bypassed: " << last_foreground_persistence_bypassed_
         << "\nforeground_count_weight: " << foreground_count_weight_
         << "\nforeground_mean_z_weight: " << foreground_mean_z_weight_
         << "\nforeground_max_z_weight: " << foreground_max_z_weight_
         << "\nforeground_min_z_weight: " << foreground_min_z_weight_
         << "\nforeground_unknown_cell_score: " << foreground_unknown_cell_score_
         << "\nforeground_score_max: " << last_foreground_score_max_
         << "\nforeground_positive_score_mean: " << last_foreground_positive_score_mean_
         << "\nforeground_score_positive_cell_count: " << last_foreground_score_positive_cell_count_
         << "\nstat_reason_count_change: " << last_stat_count_reason_count_
         << "\nstat_reason_mean_z_change: " << last_stat_mean_z_reason_count_
         << "\nstat_reason_max_z_rise: " << last_stat_max_z_reason_count_
         << "\nstat_reason_min_z_drop: " << last_stat_min_z_reason_count_
         << "\nstat_reason_unknown_cell: " << last_stat_unknown_cell_reason_count_
         << "\nnote: dynamic_points publishes foreground_points for compatibility"
         << "\nclustering_method: " << clustering_method_
         << "\nactive_clusterer_name: " << (clusterer_ ? clusterer_->name() : std::string("none"));
      const bool shape_graph_used = (clusterer_ && clusterer_->name() == "shape_graph") ||
          (enable_shape_memory_tracking_ && shape_graph_computed_this_frame_);
      if (shape_graph_used) {
        ss << "\nshape_graph_enabled: " << shape_graph_used
         << "\nshape_graph_node_count: " << last_shape_graph_node_count_
         << "\nshape_graph_edge_count: " << last_shape_graph_edge_count_
         << "\nshape_graph_component_count: " << last_shape_graph_component_count_
         << "\nshape_graph_cluster_count: " << last_shape_graph_cluster_count_
         << "\nshape_graph_largest_component_nodes: " << last_shape_graph_largest_component_nodes_
         << "\nshape_graph_largest_component_edges: " << last_shape_graph_largest_component_edges_
         << "\nshape_graph_largest_component_points: " << last_shape_graph_largest_component_points_
         << "\nshape_graph_avg_edge_score: " << last_shape_graph_avg_edge_score_
         << "\nshape_graph_endpoint_relation_count: " << last_shape_graph_endpoint_relation_count_
         << "\nshape_graph_length_relation_found: " << last_shape_graph_length_relation_found_
         << "\nshape_graph_width_relation_found: " << last_shape_graph_width_relation_found_
         << "\nshape_graph_height_relation_found: " << last_shape_graph_height_relation_found_
         << "\nshape_graph_marker_published: " << last_shape_graph_marker_published_;
      }
      if (enable_shape_memory_tracking_) {
        ss << "\nenable_shape_memory_tracking: " << enable_shape_memory_tracking_
         << "\nshape_memory_track_count: " << last_shape_memory_track_count_
         << "\nshape_graph_compute_policy: " << shape_graph_compute_policy_
         << "\nprepare_shape_memory_requested_count: " << prepare_shape_memory_requested_count_
         << "\nprepare_shape_memory_success_count: " << prepare_shape_memory_success_count_
         << "\nobserved_shape_cache_valid_track_count: " << observed_shape_cache_valid_track_count_
         << "\ncommitted_shape_memory_track_count: " << committed_shape_memory_track_count_
         << "\nmemory_compare_using_cache_count: " << memory_compare_using_cache_count_
         << "\nmemory_compare_using_current_shape_graph_count: " << memory_compare_using_current_shape_graph_count_
         << "\nshape_graph_computed_this_frame: " << shape_graph_computed_this_frame_
         << "\nshape_graph_skipped_this_frame: " << shape_graph_skipped_this_frame_
         << "\nshape_memory_association_used_count: " << last_shape_memory_association_used_count_
         << "\nshape_memory_match_score_max: " << last_shape_memory_match_score_max_
         << "\nshape_memory_match_score_mean: " << last_shape_memory_match_score_mean_
         << "\nshape_memory_reprepare_needed: " << (shape_memory_reprepare_needed_ || reprepare_tracks > 0)
         << "\nshape_memory_reprepare_track_count: " << reprepare_tracks
         << "\nobserved_shape_cache_capturing_track_count: " << observed_cache_capturing
         << "\nobserved_shape_cache_ready_track_count: " << observed_cache_ready
         << "\nobserved_shape_cache_point_count: " << observed_cache_points
         << "\nshape_memory_compare_voxel_size_near: " << shape_memory_compare_voxel_size_near_
         << "\nshape_memory_compare_voxel_size_mid: " << shape_memory_compare_voxel_size_mid_
         << "\nshape_memory_compare_voxel_size_long: " << shape_memory_compare_voxel_size_long_
         << "\nshape_memory_compare_max_points: " << shape_memory_compare_max_points_;
      }
      if (enable_shape_memory_tracking_ && object_memory_bank_enabled_) {
        ss << "\nobject_memory_bank_enabled: " << object_memory_bank_enabled_
         << "\nobject_memory_bank_file: " << object_memory_bank_file_
         << "\nobject_memory_bank_size: " << object_memory_bank_.entries.size()
         << "\nobject_memory_best_match_id: " << last_object_memory_best_match_id_
         << "\nobject_memory_best_match_class: " << last_object_memory_best_match_class_
         << "\nobject_memory_best_match_score: " << last_object_memory_best_match_score_
         << "\nobject_memory_match_attempt_count: " << last_object_memory_match_attempt_count_
         << "\nobject_memory_applicable_match_count: " << last_object_memory_applicable_match_count_
         << "\nobject_memory_attach_count: " << last_object_memory_attach_count_
         << "\nobject_memory_skipped_by_component_type_count: "
         << last_object_memory_skipped_by_component_type_count_
         << "\nobject_memory_skipped_by_component_type: " << object_memory_skipped_by_component_type
         << "\nobject_memory_skip_reason: component_type_not_applicable"
         << "\nlast_candidate_component_type: "
         << (last_candidates_.empty() ? std::string("none") : last_candidates_.front().component_type)
         << "\nlast_candidate_component_type_confidence: "
         << (last_candidates_.empty() ? 0.0 : last_candidates_.front().component_type_confidence)
         << "\nlast_candidate_component_type_reason: "
         << (last_candidates_.empty() ? std::string("") : last_candidates_.front().component_type_reason);
      }
      if (enable_shape_memory_tracking_ && shape_memory_mirror_mode_enabled_ && mirror_info_count > 0) {
        ss << "\nmirror_mode_enabled: " << shape_memory_mirror_mode_enabled_
         << "\nshape_memory_mirror_plane: " << mirror_plane
         << "\nshape_memory_mirror_flipped_axis: " << mirror_flipped_axis
         << "\nshape_memory_mirror_center_x: " << mirror_center_x
         << "\nshape_memory_mirror_center_y: " << mirror_center_y
         << "\nshape_memory_mirror_center_z: " << mirror_center_z
         << "\nshape_memory_mirror_flipped_axis_center: " << mirror_flipped_axis_center
         << "\nshape_memory_mirror_observed_node_count: " << mirror_observed_node_count
         << "\nshape_memory_mirror_virtual_node_count: " << mirror_virtual_node_count
         << "\nshape_memory_mirror_observed_edge_count: " << mirror_observed_edge_count
         << "\nshape_memory_mirror_virtual_edge_count: " << mirror_virtual_edge_count
         << "\nshape_memory_mirror_near_identical_warning_count: " << mirror_near_identical_warning_count
         << "\nshape_memory_mirror_invalid_edge_warning_count: " << mirror_invalid_edge_warning_count
         << "\nmirror_axis_deprecated: " << shape_memory_mirror_axis_
         << "\nmirror_centerline: " << mirror_centerline
         << "\nmirror_centerline_x: " << mirror_centerline_x
         << "\nmirror_centerline_y: " << mirror_centerline_y
         << "\nmirror_centerline_z: " << mirror_centerline_z
         << "\nmirror_observed_node_count: " << mirror_observed_node_count
         << "\nmirror_virtual_node_count: " << mirror_virtual_node_count
         << "\nmirror_symmetry_confidence: " << mirror_symmetry_confidence;
      }
      ss << "\nbev_cluster_count: " << last_cluster_count_
         << "\ncluster_connectivity: " << cluster_connectivity_
         << "\nmin_cluster_points_near: " << min_cluster_points_near_
         << "\nmin_cluster_points_mid: " << min_cluster_points_mid_
         << "\nmin_cluster_points_long: " << min_cluster_points_long_
         << "\nmin_cluster_cells_near: " << min_cluster_cells_near_
         << "\nmin_cluster_cells_mid: " << min_cluster_cells_mid_
         << "\nmin_cluster_cells_long: " << min_cluster_cells_long_
         << "\ncandidate_count: " << last_candidate_count_
         << "\nactive_track_count: " << last_active_track_count_
         << "\ntracked_count: " << last_active_track_count_
         << "\nstationary_track_count: " << last_stationary_count_
         << "\nmoving_track_count: " << last_moving_count_
         << "\nvehicle_detected_count: " << last_vehicle_count_
         << "\nlost_vehicle_count: " << last_lost_vehicle_count_
         << "\nlost_predictive_vehicle_count: " << last_lost_predictive_count_
         << "\ndormant_vehicle_count: " << last_dormant_count_
         << "\nvehicle_fifo_queue_size: " << inside_vehicle_queue_.size()
         << "\nvehicle_fifo_queue: " << fifoQueueString()
         << "\nvehicle_fifo_pending_exit_track_id: " << pending_exit_track_id_
         << "\nvehicle_fifo_entry_count: " << vehicle_fifo_entry_count_
         << "\nvehicle_fifo_exit_candidate_count: " << vehicle_fifo_exit_candidate_count_
         << "\nvehicle_fifo_reacquire_success_count: " << vehicle_fifo_reacquire_success_count_
         << "\nvehicle_fifo_validation_reject_count: " << vehicle_fifo_validation_reject_count_
         << "\nvehicle_fifo_direction_unknown_count: " << vehicle_fifo_direction_unknown_count_
         << "\nvehicle_fifo_expired_count: " << vehicle_fifo_expired_count_
         << "\nvehicle_fifo_exit_complete_count: " << vehicle_fifo_exit_complete_count_
         << "\nvehicle_fifo_anomaly_count: " << vehicle_fifo_anomaly_count_
         << "\nvehicle_lost_prediction_expired_count: " << vehicle_lost_prediction_expired_count_
         << "\nvehicle_dormant_enter_count: " << vehicle_dormant_enter_count_
         << "\nvehicle_dormant_expired_count: " << vehicle_dormant_expired_count_
         << "\nvehicle_short_reacquire_success_count: " << vehicle_reacquire_success_count_
         << "\nvehicle_dormant_reacquire_attempt_count: " << vehicle_dormant_reacquire_attempt_count_
         << "\nvehicle_dormant_reacquire_success_count: " << vehicle_dormant_reacquire_success_count_
         << "\nvehicle_dormant_reacquire_ambiguous_count: " << vehicle_dormant_reacquire_ambiguous_count_
         << "\nvehicle_lost_enter_count: " << vehicle_lost_enter_count_
         << "\nvehicle_reacquire_attempt_count: " << vehicle_reacquire_attempt_count_
         << "\nvehicle_reacquire_success_count: " << vehicle_reacquire_success_count_
         << "\nvehicle_reacquire_ambiguous_count: " << vehicle_reacquire_ambiguous_count_
         << "\nvehicle_reacquire_reject_count: " << vehicle_reacquire_reject_count_
         << "\nvehicle_lost_expired_count: " << vehicle_lost_expired_count_
         << "\nunknown_candidate_count: " << last_unknown_count_
         << "\nrejected_candidate_count: " << last_rejected_candidate_count_
         << "\ntracking_feature_input_count: " << last_tracking_feature_input_count_
         << "\ntracking_candidate_count: " << last_tracking_candidate_count_
         << "\nactive_track_count_before: " << last_active_track_count_before_
         << "\nactive_track_count_after: " << last_active_track_count_
         << "\ntracking_association_pairs_checked: " << last_tracking_association_pairs_checked_;
      const bool debug_transformed_used = publish_transformed_cloud_ && publish_debug_transformed_cloud_ &&
          (pub_road_frame_ ? pub_road_frame_->get_subscription_count() : 0u) > 0;
      const bool debug_valid_used = publish_debug_valid_points_ && (pub_valid_ ? pub_valid_->get_subscription_count() : 0u) > 0;
      const bool debug_foreground_used = publish_debug_foreground_points_ &&
          (pub_foreground_ ? pub_foreground_->get_subscription_count() : 0u) > 0;
      const bool debug_static_used = publish_debug_static_background_points_ &&
          (pub_static_background_ ? pub_static_background_->get_subscription_count() : 0u) > 0;
      const bool debug_clusters_used = publish_debug_clusters_ && (pub_clusters_ ? pub_clusters_->get_subscription_count() : 0u) > 0;
      const bool debug_roi_used = publish_debug_roi_filtered_points_ &&
          (pub_roi_filtered_ ? pub_roi_filtered_->get_subscription_count() : 0u) > 0;
      if (debug_transformed_used || debug_valid_used || debug_foreground_used || debug_static_used ||
          debug_clusters_used || debug_roi_used) {
        ss << "\ndebug_cloud_publish_period_sec: " << debug_cloud_publish_period_sec_
           << "\ndebug_cloud_max_points: " << debug_cloud_max_points_;
      }
      if (debug_transformed_used) ss << "\npublish_debug_transformed_cloud: " << publish_debug_transformed_cloud_;
      if (debug_valid_used) ss << "\npublish_debug_valid_points: " << publish_debug_valid_points_;
      if (debug_foreground_used) ss << "\npublish_debug_foreground_points: " << publish_debug_foreground_points_;
      if (debug_static_used) ss << "\npublish_debug_static_background_points: " << publish_debug_static_background_points_;
      if (debug_clusters_used) ss << "\npublish_debug_clusters: " << publish_debug_clusters_;
      if (debug_roi_used) ss << "\npublish_debug_roi_filtered_points: " << publish_debug_roi_filtered_points_;
      ss << "\nenable_radius_outlier_removal: " << enable_radius_outlier_removal_
         << "\nskip_ror_when_dense: " << skip_ror_when_dense_
         << "\nror_max_input_points: " << ror_max_input_points_
         << "\nror_input_count: " << last_ror_input_count_
         << "\nror_skipped: " << last_ror_skipped_
         << "\nread_cloud_ms: " << read_cloud_ms_
         << "\nvalid_crop_ms: " << valid_crop_ms_
         << "\nroi_policy_ms: " << roi_policy_ms_
         << "\nbackground_learning_ms: " << background_learning_ms_
         << "\nforeground_classification_ms: " << foreground_classification_ms_
         << "\nradius_outlier_removal_ms: " << radius_outlier_removal_ms_
         << "\nclustering_ms: " << clustering_ms_
         << "\nbev_build_ms: " << bev_build_ms_
         << "\nconnected_component_bfs_ms: " << connected_component_ms_
         << "\ncluster_point_gather_ms: " << cluster_point_gather_ms_
         << "\ncluster_filter_ms: " << cluster_filter_ms_
         << "\noccupied_cell_count: " << last_occupied_cell_count_
         << "\nraw_component_count: " << last_connected_component_count_
         << "\nconnected_component_count: " << last_connected_component_count_
         << "\nvalid_cluster_count: " << last_valid_cluster_count_
         << "\ntiny_component_count: " << last_tiny_component_count_
         << "\nsingle_cell_component_count: " << last_single_cell_component_count_
         << "\ntwo_cell_component_count: " << last_two_cell_component_count_
         << "\nlargest_component_cells: " << last_largest_component_cells_
         << "\nlargest_component_points: " << last_largest_component_points_
         << "\nmedian_component_cells: " << last_median_component_cells_
         << "\nmedian_component_points: " << last_median_component_points_
         << "\ncomponent_cell_histogram_1: " << last_component_cell_histogram_[0]
         << "\ncomponent_cell_histogram_2: " << last_component_cell_histogram_[1]
         << "\ncomponent_cell_histogram_3_5: " << last_component_cell_histogram_[2]
         << "\ncomponent_cell_histogram_6_10: " << last_component_cell_histogram_[3]
         << "\ncomponent_cell_histogram_11_30: " << last_component_cell_histogram_[4]
         << "\ncomponent_cell_histogram_31_plus: " << last_component_cell_histogram_[5]
         << "\ncomponent_point_histogram_1: " << last_component_point_histogram_[0]
         << "\ncomponent_point_histogram_2: " << last_component_point_histogram_[1]
         << "\ncomponent_point_histogram_3_5: " << last_component_point_histogram_[2]
         << "\ncomponent_point_histogram_6_10: " << last_component_point_histogram_[3]
         << "\ncomponent_point_histogram_11_30: " << last_component_point_histogram_[4]
         << "\ncomponent_point_histogram_31_plus: " << last_component_point_histogram_[5];
      if (cluster_edge_diagnostics_enabled_) {
        ss << "\ncluster_edge_diagnostics_enabled: " << cluster_edge_diagnostics_enabled_;
      }
      if (cluster_z_profile_diagnostics_enabled_) {
        ss << "\ncluster_z_profile_diagnostics_enabled: " << cluster_z_profile_diagnostics_enabled_
         << "\ncluster_z_profile_bin_size: " << cluster_z_profile_bin_size_
         << "\nz_profile_build_ms: " << z_profile_build_ms_
         << "\nz_profile_edge_diag_ms: " << z_profile_edge_diag_ms_
         << "\nz_profile_cardinal_edges: " << profile_diag.z_profile_cardinal_edges
         << "\nz_profile_diagonal_edges: " << profile_diag.z_profile_diagonal_edges
         << "\nz_profile_bridge_edges: " << profile_diag.z_profile_bridge_edges
         << "\nz_profile_cardinal_accepted: " << profile_diag.z_profile_cardinal_accepted
         << "\nz_profile_diagonal_accepted: " << profile_diag.z_profile_diagonal_accepted
         << "\nz_profile_bridge_accepted: " << profile_diag.z_profile_bridge_accepted
         << "\nz_profile_cardinal_rejected: " << profile_diag.z_profile_cardinal_rejected
         << "\nz_profile_diagonal_rejected: " << profile_diag.z_profile_diagonal_rejected
         << "\nz_profile_bridge_rejected: " << profile_diag.z_profile_bridge_rejected
         << "\nz_profile_cardinal_exact_r_min_mean: " << (profile_diag.z_profile_cardinal_edges ? profile_diag.z_profile_cardinal_exact_r_min_sum / profile_diag.z_profile_cardinal_edges : 0.0)
         << "\nz_profile_cardinal_tolerant_r_min_mean: " << (profile_diag.z_profile_cardinal_edges ? profile_diag.z_profile_cardinal_tolerant_r_min_sum / profile_diag.z_profile_cardinal_edges : 0.0)
         << "\nz_profile_cardinal_exact_jaccard_mean: " << (profile_diag.z_profile_cardinal_edges ? profile_diag.z_profile_cardinal_exact_jaccard_sum / profile_diag.z_profile_cardinal_edges : 0.0)
         << "\nz_profile_cardinal_tolerant_jaccard_mean: " << (profile_diag.z_profile_cardinal_edges ? profile_diag.z_profile_cardinal_tolerant_jaccard_sum / profile_diag.z_profile_cardinal_edges : 0.0)
         << "\nz_profile_diagonal_exact_r_min_mean: " << (profile_diag.z_profile_diagonal_edges ? profile_diag.z_profile_diagonal_exact_r_min_sum / profile_diag.z_profile_diagonal_edges : 0.0)
         << "\nz_profile_diagonal_tolerant_r_min_mean: " << (profile_diag.z_profile_diagonal_edges ? profile_diag.z_profile_diagonal_tolerant_r_min_sum / profile_diag.z_profile_diagonal_edges : 0.0)
         << "\nz_profile_diagonal_exact_jaccard_mean: " << (profile_diag.z_profile_diagonal_edges ? profile_diag.z_profile_diagonal_exact_jaccard_sum / profile_diag.z_profile_diagonal_edges : 0.0)
         << "\nz_profile_diagonal_tolerant_jaccard_mean: " << (profile_diag.z_profile_diagonal_edges ? profile_diag.z_profile_diagonal_tolerant_jaccard_sum / profile_diag.z_profile_diagonal_edges : 0.0)
         << "\nz_profile_bridge_exact_r_min_mean: " << (profile_diag.z_profile_bridge_edges ? profile_diag.z_profile_bridge_exact_r_min_sum / profile_diag.z_profile_bridge_edges : 0.0)
         << "\nz_profile_bridge_tolerant_r_min_mean: " << (profile_diag.z_profile_bridge_edges ? profile_diag.z_profile_bridge_tolerant_r_min_sum / profile_diag.z_profile_bridge_edges : 0.0)
         << "\nz_profile_bridge_exact_jaccard_mean: " << (profile_diag.z_profile_bridge_edges ? profile_diag.z_profile_bridge_exact_jaccard_sum / profile_diag.z_profile_bridge_edges : 0.0)
         << "\nz_profile_bridge_tolerant_jaccard_mean: " << (profile_diag.z_profile_bridge_edges ? profile_diag.z_profile_bridge_tolerant_jaccard_sum / profile_diag.z_profile_bridge_edges : 0.0)
         << "\nz_profile_cardinal_z_gap_mean: " << (profile_diag.z_profile_cardinal_edges ? profile_diag.z_profile_cardinal_z_gap_sum / profile_diag.z_profile_cardinal_edges : 0.0)
         << "\nz_profile_diagonal_z_gap_mean: " << (profile_diag.z_profile_diagonal_edges ? profile_diag.z_profile_diagonal_z_gap_sum / profile_diag.z_profile_diagonal_edges : 0.0)
         << "\nz_profile_bridge_z_gap_mean: " << (profile_diag.z_profile_bridge_edges ? profile_diag.z_profile_bridge_z_gap_sum / profile_diag.z_profile_bridge_edges : 0.0)
         << "\nz_profile_cardinal_exact_r_min_hist: " << formatArray(profile_diag.z_profile_cardinal_exact_r_min_hist)
         << "\nz_profile_cardinal_tolerant_r_min_hist: " << formatArray(profile_diag.z_profile_cardinal_tolerant_r_min_hist)
         << "\nz_profile_cardinal_exact_jaccard_hist: " << formatArray(profile_diag.z_profile_cardinal_exact_jaccard_hist)
         << "\nz_profile_cardinal_tolerant_jaccard_hist: " << formatArray(profile_diag.z_profile_cardinal_tolerant_jaccard_hist)
         << "\nz_profile_cardinal_accepted_tol_r_min_hist: " << formatArray(profile_diag.z_profile_cardinal_accepted_tol_r_min_hist)
         << "\nz_profile_cardinal_rejected_tol_r_min_hist: " << formatArray(profile_diag.z_profile_cardinal_rejected_tol_r_min_hist)
         << "\nz_profile_cardinal_z_gap_hist: " << formatArray(profile_diag.z_profile_cardinal_z_gap_hist)
         << "\nz_profile_diagonal_exact_r_min_hist: " << formatArray(profile_diag.z_profile_diagonal_exact_r_min_hist)
         << "\nz_profile_diagonal_tolerant_r_min_hist: " << formatArray(profile_diag.z_profile_diagonal_tolerant_r_min_hist)
         << "\nz_profile_diagonal_exact_jaccard_hist: " << formatArray(profile_diag.z_profile_diagonal_exact_jaccard_hist)
         << "\nz_profile_diagonal_tolerant_jaccard_hist: " << formatArray(profile_diag.z_profile_diagonal_tolerant_jaccard_hist)
         << "\nz_profile_diagonal_accepted_tol_r_min_hist: " << formatArray(profile_diag.z_profile_diagonal_accepted_tol_r_min_hist)
         << "\nz_profile_diagonal_rejected_tol_r_min_hist: " << formatArray(profile_diag.z_profile_diagonal_rejected_tol_r_min_hist)
         << "\nz_profile_diagonal_z_gap_hist: " << formatArray(profile_diag.z_profile_diagonal_z_gap_hist)
         << "\nz_profile_diagonal_support_hist: " << formatArray(profile_diag.z_profile_diagonal_support_hist)
         << "\nz_profile_bridge_exact_r_min_hist: " << formatArray(profile_diag.z_profile_bridge_exact_r_min_hist)
         << "\nz_profile_bridge_tolerant_r_min_hist: " << formatArray(profile_diag.z_profile_bridge_tolerant_r_min_hist)
         << "\nz_profile_bridge_exact_jaccard_hist: " << formatArray(profile_diag.z_profile_bridge_exact_jaccard_hist)
         << "\nz_profile_bridge_tolerant_jaccard_hist: " << formatArray(profile_diag.z_profile_bridge_tolerant_jaccard_hist)
         << "\nz_profile_bridge_accepted_tol_r_min_hist: " << formatArray(profile_diag.z_profile_bridge_accepted_tol_r_min_hist)
         << "\nz_profile_bridge_rejected_tol_r_min_hist: " << formatArray(profile_diag.z_profile_bridge_rejected_tol_r_min_hist)
         << "\nz_profile_bridge_z_gap_hist: " << formatArray(profile_diag.z_profile_bridge_z_gap_hist)
         << "\nz_profile_bridge_axis_support_hist: " << formatArray(profile_diag.z_profile_bridge_axis_support_hist);
      }
      if (cluster_z_layer_enabled_) {
        ss << "\ncluster_z_layer_enabled: " << cluster_z_layer_enabled_
         << "\ncluster_z_layer_resolution: " << cluster_z_layer_resolution_
         << "\nz_layer_build_ms: " << z_layer_build_ms_
         << "\nlayer_cell_count: " << last_layer_cell_count_
         << "\nstrong_layer_cell_count: " << last_strong_layer_cell_count_
         << "\nweak_layer_cell_count: " << last_weak_layer_cell_count_
         << "\naverage_layers_per_xy_cell: " << last_average_layers_per_xy_cell_
         << "\nmax_layers_per_xy_cell: " << last_max_layers_per_xy_cell_;
        if (cluster_weak_support_enabled_) {
          ss << "\ncluster_weak_support_enabled: " << cluster_weak_support_enabled_
         << "\ncluster_weak_support_min_density: " << cluster_weak_support_min_density_
         << "\nweak_support_candidates: " << last_weak_support_candidates_
         << "\nweak_support_accepted: " << last_weak_support_accepted_
         << "\nweak_support_density_rejected: " << last_weak_support_density_rejected_;
        }
        if (cluster_density_enabled_) {
          ss << "\ncluster_density_enabled: " << cluster_density_enabled_
         << "\ncluster_density_window_cells: " << cluster_density_window_cells_
         << "\ndensity_build_ms: " << density_build_ms_;
        }
      }
      if (cluster_edge_diagnostics_enabled_) {
        ss << "\nz_rejected_direct_edges: " << last_z_rejected_direct_edges_
         << "\nbridge_candidates: " << last_bridge_candidates_
         << "\nbridge_accepted: " << last_bridge_accepted_
         << "\nz_rejected_bridge_edges: " << last_z_rejected_bridge_edges_;
      }
      ss << "\nfeature_build_ms: " << feature_build_ms_
         << "\ntracking_ms: " << tracking_ms_
         << "\nvehicle_lost_predict_ms: " << vehicle_lost_predict_ms_
         << "\nvehicle_reacquire_ms: " << vehicle_reacquire_ms_
         << "\nvehicle_dormant_management_ms: " << vehicle_dormant_management_ms_
         << "\nvehicle_dormant_reacquire_ms: " << vehicle_dormant_reacquire_ms_
         << "\nvehicle_fifo_identity_ms: " << vehicle_fifo_identity_ms_;
      if (visibility_enabled_) {
        double expected_rays = 0.0, supported_rays = 0.0;
        double azimuth_width = 0.0, elevation_width = 0.0;
        double sensor_distance = 0.0, current_weight = 0.0, memory_weight = 0.0;
        for (const auto & tr : tracks_) {
          const auto & v = tr.visibility;
          expected_rays += v.adaptive_expected_rays;
          supported_rays += v.adaptive_supported_rays;
          azimuth_width += v.adaptive_azimuth_width_deg;
          elevation_width += v.adaptive_elevation_width_deg;
          sensor_distance += v.sensor_distance;
          current_weight += v.current_observation_weight;
          memory_weight += v.memory_weight;
        }
        const double n = std::max<size_t>(1, tracks_.size());
        ss << "\nvisibility_static_lookup_ms: " << visibility_static_lookup_ms_
         << "\nvisibility_depth_build_ms: " << visibility_depth_build_ms_
         << "\nvehicle_surface_predict_ms: " << vehicle_surface_predict_ms_
         << "\nvehicle_visibility_query_ms: " << vehicle_visibility_query_ms_
         << "\nvehicle_ownership_ms: " << vehicle_ownership_ms_
         << "\nvehicle_sparse_support_ms: " << vehicle_sparse_support_ms_
         << "\nvisibility_cluster_ms: " << visibility_cluster_ms_
         << "\nvisibility_track_ms: " << visibility_track_ms_
         << "\nadaptive_track_visibility_ms: " << adaptive_track_visibility_ms_
         << "\ntrack_memory_update_ms: " << track_memory_update_ms_
         << "\nvisibility_total_ms: " << (visibility_static_lookup_ms_ + visibility_depth_build_ms_ + visibility_cluster_ms_ + visibility_track_ms_)
         << "\nvisibility_depth_cells: " << (visibility_frame_valid_ ? visibility_model_.depthCellCount() : 0)
         << "\nvisibility_static_cells: " << visibility_model_.staticCellCount()
         << "\nvisibility_static_cache_hits: " << visibility_model_.staticCacheHits()
         << "\nvisibility_static_cache_misses: " << visibility_model_.staticCacheMisses()
         << "\ntrack_observation_memory_count: " << tracks_.size()
         << "\ntrack_observation_memory_bytes_estimate: " << tracks_.size() * sizeof(TrackObservationMemory)
         << "\ntrack_memory_created: " << track_memory_created_
         << "\ntrack_memory_removed: " << track_memory_removed_
         << "\nvisibility_adaptive_loss_enabled: " << visibility_adaptive_loss_enabled_
         << "\nadaptive_expected_rays: " << expected_rays
         << "\nadaptive_supported_rays: " << supported_rays
         << "\nadaptive_azimuth_width_deg: " << azimuth_width / n
         << "\nadaptive_elevation_width_deg: " << elevation_width / n
         << "\ndistance_to_sensor: " << sensor_distance / n
         << "\ncurrent_observation_weight: " << current_weight / n
         << "\nmemory_weight: " << memory_weight / n
         << "\nvisibility_azimuth_bin_deg: " << visibility_model_.azimuthBinDeg()
         << "\nvisibility_azimuth_bin_measured: " << visibility_azimuth_bin_measured_
         << "\nvisibility_elevation_bin_deg: " << visibility_model_.elevationBinDeg()
         << "\nvisibility_elevation_ray_spacing_deg: " << visibility_model_.rayElevationSpacingDeg()
         << "\nvisibility_frame_valid: " << visibility_frame_valid_;
      }
      if (shape_memory_match_ms_ > 0.0) {
        ss << "\nshape_memory_match_ms: " << shape_memory_match_ms_;
      }
      if (fem_assisted_clustering_enabled_) {
        ss << "\nfem_refine_ms: " << fem_refine_ms_;
      }
      if ((debug_valid_used && last_debug_valid_published_points_ > 0) ||
          (debug_roi_used && last_debug_roi_filtered_published_points_ > 0) ||
          (debug_foreground_used && last_debug_foreground_published_points_ > 0) ||
          (debug_clusters_used && last_debug_clusters_published_points_ > 0)) {
        ss << "\ndebug_cloud_build_ms: " << debug_cloud_build_ms_;
      }
      ss << "\nmarker_build_ms: " << marker_build_ms_
         << "\npublish_ms: " << publish_ms_;
      const size_t object_array_subscribers =
          pub_detected_objects_->get_subscription_count() + pub_tracked_objects_->get_subscription_count() +
          pub_motion_states_->get_subscription_count() + pub_vehicle_tracks_->get_subscription_count();
      if (object_array_subscribers > 0) {
        ss << "\nobject_arrays_build_ms: " << object_arrays_build_ms_
         << "\nobject_arrays_publish_ms: " << object_arrays_publish_ms_
         << "\nobject_arrays_bytes_estimate: " << object_arrays_bytes_
         << "\nobject_array_subscribers: " << object_array_subscribers;
      }
      const size_t track_table_subscribers = pub_track_table_->get_subscription_count();
      if (track_table_subscribers > 0) {
        ss << "\ntrack_table_build_ms: " << track_table_build_ms_
         << "\ntrack_table_publish_ms: " << track_table_publish_ms_
         << "\ntrack_table_bytes: " << track_table_bytes_
         << "\ntrack_table_subscribers: " << track_table_subscribers;
      }
      const size_t candidate_table_subscribers = pub_candidate_table_->get_subscription_count();
      if (candidate_table_subscribers > 0) {
        ss << "\ncandidate_table_build_ms: " << candidate_table_build_ms_
         << "\ncandidate_table_publish_ms: " << candidate_table_publish_ms_
         << "\ncandidate_table_bytes: " << candidate_table_bytes_
         << "\ncandidate_table_subscribers: " << candidate_table_subscribers;
      }
      ss << "\ntotal_pipeline_ms: " << total_pipeline_ms_
         << "\ninput_queue_delay_ms: " << input_queue_delay_ms_
         << "\nprocessed_frame_count: " << processed_frame_count_
         << "\nreceived_frame_count: " << received_frame_count_
         << "\ndropped_frame_count: " << dropped_frame_count_
         << "\ndrop_ratio: " << (received_frame_count_ > 0 ? static_cast<double>(dropped_frame_count_) / received_frame_count_ : 0.0)
         << "\nparallel_enabled: " << (enable_parallel_processing_ ? "true" : "false");
      if (enable_parallel_processing_) ss << "\nparallel_threads: " << parallel_threads_;
      if (debug_valid_used && last_debug_valid_published_points_ > 0)
        ss << "\ndebug_valid_published_points: " << last_debug_valid_published_points_;
      if (debug_roi_used && last_debug_roi_filtered_published_points_ > 0)
        ss << "\ndebug_roi_filtered_published_points: " << last_debug_roi_filtered_published_points_;
      if (debug_foreground_used && last_debug_foreground_published_points_ > 0)
        ss << "\ndebug_foreground_published_points: " << last_debug_foreground_published_points_;
      if (debug_clusters_used && last_debug_clusters_published_points_ > 0)
        ss << "\ndebug_clusters_published_points: " << last_debug_clusters_published_points_;
      s.data = ss.str();
      pub_diagnostics_table_->publish(s);
      if (cluster_z_profile_diagnostics_enabled_ && !profile_diag.edge_samples.empty()) {
        static bool s_edge_csv_header_written = false;
        std::ofstream ofs("/tmp/cx128s2_edge_diagnostics.csv", std::ios::app);
        if (ofs.is_open()) {
          if (!s_edge_csv_header_written) {
            ofs << "frame_index,timestamp,edge_type,ix_a,iy_a,ix_b,iy_b,points_a,points_b,min_z_a,max_z_a,min_z_b,max_z_b,interval_z_gap,popcount_a,popcount_b,exact_r_min,tolerant_r_min,exact_jaccard,tolerant_jaccard,directional_support,baseline_decision\n";
            s_edge_csv_header_written = true;
          }
          for (const auto & es : profile_diag.edge_samples) {
            ofs << es.frame_index << "," << std::fixed << std::setprecision(3) << es.timestamp << ","
                << es.edge_type << "," << es.ix_a << "," << es.iy_a << "," << es.ix_b << "," << es.iy_b << ","
                << es.points_a << "," << es.points_b << ","
                << std::setprecision(3) << es.min_z_a << "," << es.max_z_a << ","
                << es.min_z_b << "," << es.max_z_b << ","
                << es.interval_z_gap << "," << es.popcount_a << "," << es.popcount_b << ","
                << es.exact_r_min << "," << es.tolerant_r_min << ","
                << es.exact_jaccard << "," << es.tolerant_jaccard << ","
                << es.directional_support << "," << (es.baseline_decision ? 1 : 0) << "\n";
          }
        }
      }
      if (print_diagnostics_table_to_console_) RCLCPP_INFO(get_logger(), "\n%s", s.data.c_str());
    }

    const auto track_build_start = std::chrono::steady_clock::now();
    std_msgs::msg::String track_s;
    std::ostringstream ts;
    ts << trackTableCsvHeader() << "\n";
    for (const auto & tr : tracks_) ts << trackRow(tr) << "\n";
    track_s.data = ts.str();
    track_table_build_ms_ = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - track_build_start).count();
    track_table_bytes_ = track_s.data.size();
    const auto track_publish_start = std::chrono::steady_clock::now();
    pub_track_table_->publish(track_s);
    track_table_publish_ms_ = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - track_publish_start).count();

    const auto candidate_build_start = std::chrono::steady_clock::now();
    std_msgs::msg::String cand_s;
    std::ostringstream cs;
    cs << "candidate_id,cluster_id,selected_class,rejection_reason,centroid_x,centroid_y,centroid_z,"
          "distance,length,width,height,point_count,cell_count,density,ring_coverage,mean_intensity,"
          "background_similarity,dynamic_score,inside_roi,component_type,component_type_confidence,"
          "component_type_reason,shape_node_count,shape_edge_count,"
          "shape_graph_score,shape_memory_match_score,visibility_reason,observable_fraction,"
          "occluded_fraction,supported_fraction,sensor_distance,azimuth_deg,elevation_deg,"
          "in_horizontal_fov,in_vertical_fov,in_valid_range,near_blind_zone,expected_samples,"
          "observable_samples,occluded_samples,supported_samples,free_samples,unknown_samples,"
          "visibility_confidence\n";
    for (const auto & c : last_candidates_) {
      cs << c.id << "," << c.id << ",UNKNOWN," << csvSafe(c.rejection_reason) << ","
         << c.centroid_x << "," << c.centroid_y << "," << c.centroid_z << "," << c.range
         << "," << c.length << "," << c.width << "," << c.height << "," << c.point_count
         << "," << c.cell_count << "," << c.density << "," << c.ring_coverage << ","
         << c.mean_intensity << "," << c.background_similarity << "," << c.dynamic_score
         << "," << c.inside_roi << "," << csvSafe(c.component_type) << ","
         << c.component_type_confidence << "," << csvSafe(c.component_type_reason)
         << "," << c.shape_node_count << "," << c.shape_edge_count
         << "," << c.shape_graph_score << "," << c.shape_memory_match_score << ","
         << visibilityReasonName(c.visibility.reason) << "," << c.visibility.observable_fraction
         << "," << c.visibility.occluded_fraction << "," << c.visibility.supported_fraction
         << "," << c.visibility.sensor_distance << "," << c.visibility.azimuth_deg
         << "," << c.visibility.elevation_deg << "," << c.visibility.in_horizontal_fov
         << "," << c.visibility.in_vertical_fov << "," << c.visibility.in_valid_range
         << "," << c.visibility.near_blind_zone << "," << c.visibility.expected_samples
         << "," << c.visibility.observable_samples << "," << c.visibility.occluded_samples
         << "," << c.visibility.supported_samples << "," << c.visibility.free_samples
         << "," << c.visibility.unknown_samples << "," << c.visibility.confidence << "\n";
    }
    cand_s.data = cs.str();
    candidate_table_build_ms_ = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - candidate_build_start).count();
    candidate_table_bytes_ = cand_s.data.size();
    const auto candidate_publish_start = std::chrono::steady_clock::now();
    pub_candidate_table_->publish(cand_s);
    candidate_table_publish_ms_ = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - candidate_publish_start).count();
  }

  void publishStatus() {
    if (!pub_background_status_ || !pub_object_status_) return;
    diagnostic_msgs::msg::KeyValue bg;
    bg.key = "background_status";
    if (!usesStaticBackground()) bg.value = "BACKGROUND_DISABLED|mode=ground_only";
    else if (learning_active_) bg.value = "LEARNING|" + std::to_string(learned_frame_count_) + "/" +
                                    std::to_string(background_learning_frames_);
    else if (background_ready_) bg.value = "BACKGROUND_READY|cells=" + std::to_string(background_map_.size());
    else bg.value = "BACKGROUND_NOT_READY";
    pub_background_status_->publish(bg);

    diagnostic_msgs::msg::KeyValue obj;
    obj.key = "object_status";
    obj.value = "tracks=" + std::to_string(tracks_.size()) + "|vehicles=" +
                std::to_string(last_vehicle_count_) + "|unknown=" + std::to_string(last_unknown_count_);
    pub_object_status_->publish(obj);
  }

  void publishRoiMarker(const std_msgs::msg::Header & header) {
    if (!pub_roi_marker_ || pub_roi_marker_->get_subscription_count() == 0) return;
    visualization_msgs::msg::MarkerArray arr;
    int id = 0;
    auto append_zone = [&](const RoiZone & zone) {
      if (!zone.enabled) return;
      std::vector<double> xs, ys;
      if (zone.shape == "box") {
        xs = {zone.x_min, zone.x_max, zone.x_max, zone.x_min, zone.x_min};
        ys = {zone.y_min, zone.y_min, zone.y_max, zone.y_max, zone.y_min};
      } else {
        xs = zone.polygon_x;
        ys = zone.polygon_y;
        if (!xs.empty()) {
          xs.push_back(xs.front());
          ys.push_back(ys.front());
        }
      }
      if (xs.empty() || xs.size() != ys.size()) return;

      visualization_msgs::msg::Marker line;
      line.header = header;
      line.ns = "roi_policy";
      line.id = id++;
      line.type = visualization_msgs::msg::Marker::LINE_STRIP;
      line.action = visualization_msgs::msg::Marker::ADD;
      line.scale.x = 0.25;
      if (zone.type == "watch") {
        line.color.r = 0.1f; line.color.g = 0.9f; line.color.b = 0.2f;
      } else if (zone.type == "ignore") {
        line.color.r = 1.0f; line.color.g = 0.85f; line.color.b = 0.05f;
      } else {
        line.color.r = 1.0f; line.color.g = 0.1f; line.color.b = 0.1f;
      }
      line.color.a = 1.0f;
      line.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_);
      double cx = 0.0, cy = 0.0;
      for (size_t i = 0; i < xs.size(); ++i) {
        geometry_msgs::msg::Point p;
        p.x = xs[i];
        p.y = ys[i];
        p.z = 0.05;
        line.points.push_back(p);
        cx += xs[i];
        cy += ys[i];
      }
      arr.markers.push_back(line);

      visualization_msgs::msg::Marker text;
      text.header = header;
      text.ns = "roi_policy_text";
      text.id = id++;
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;
      text.pose.position.x = cx / std::max<size_t>(1, xs.size());
      text.pose.position.y = cy / std::max<size_t>(1, ys.size());
      text.pose.position.z = 1.0;
      text.pose.orientation.w = 1.0;
      text.scale.z = 0.8;
      text.color = line.color;
      text.color.a = 1.0f;
      text.text = zone.name + "\n" + zone.type;
      text.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_);
      arr.markers.push_back(text);
    };

    if (use_roi_policy_ && publish_roi_policy_markers_) {
      for (const auto & zone : roi_zones_) append_zone(zone);
    } else if (roi_enabled_) {
      RoiZone legacy;
      legacy.name = "legacy_roi";
      legacy.type = "watch";
      legacy.shape = roi_shape_;
      legacy.x_min = roi_x_min_;
      legacy.x_max = roi_x_max_;
      legacy.y_min = roi_y_min_;
      legacy.y_max = roi_y_max_;
      legacy.polygon_x = roi_polygon_x_;
      legacy.polygon_y = roi_polygon_y_;
      legacy.enabled = true;
      append_zone(legacy);
    }
    if (arr.markers.empty()) {
      visualization_msgs::msg::Marker clear;
      clear.header = header;
      clear.action = visualization_msgs::msg::Marker::DELETEALL;
      arr.markers.push_back(clear);
    }
    pub_roi_marker_->publish(arr);
  }

  std::string roiInputModeString() const {
    if (!use_roi_policy_) return "ROI_DISABLED";
    if (require_watch_roi_ && discard_points_outside_watch_roi_ && apply_ignore_before_learning_) {
      return "WATCH_ONLY";
    }
    if (countRoiZones("watch") > 0 && !require_watch_roi_ && !discard_points_outside_watch_roi_) {
      return "WATCH_DEFINED_BUT_NOT_REQUIRED";
    }
    if (countRoiZones("watch") == 0 && (countRoiZones("ignore") > 0 || countRoiZones("hard_ignore") > 0)) {
      return "IGNORE_ONLY";
    }
    return "CUSTOM_OR_MIXED";
  }

  void resetFrameStats() {
    last_range_min_ = std::numeric_limits<double>::max();
    last_range_max_ = 0.0;
    last_ring_min_ = std::numeric_limits<int>::max();
    last_ring_max_ = 0;
    last_ground_count_ = 0;
    last_obstacle_count_ = 0;
    last_ignored_count_ = 0;
    last_roi_filtered_count_ = 0;
    last_roi_ignored_count_ = 0;
    last_roi_hard_ignored_count_ = 0;
    bev_build_ms_ = 0.0;
    connected_component_ms_ = 0.0;
    cluster_point_gather_ms_ = 0.0;
    cluster_filter_ms_ = 0.0;
    z_layer_build_ms_ = 0.0;
    density_build_ms_ = 0.0;
    last_occupied_cell_count_ = 0;
    last_connected_component_count_ = 0;
    last_valid_cluster_count_ = 0;
    last_tiny_component_count_ = 0;
    last_single_cell_component_count_ = 0;
    last_two_cell_component_count_ = 0;
    last_largest_component_cells_ = 0;
    last_largest_component_points_ = 0;
    last_median_component_cells_ = 0;
    last_median_component_points_ = 0;
    last_component_cell_histogram_.fill(0);
    last_component_point_histogram_.fill(0);
    last_z_rejected_direct_edges_ = 0;
    last_bridge_candidates_ = 0;
    last_bridge_accepted_ = 0;
    last_z_rejected_bridge_edges_ = 0;
    last_layer_cell_count_ = 0;
    last_strong_layer_cell_count_ = 0;
    last_weak_layer_cell_count_ = 0;
    last_max_layers_per_xy_cell_ = 0;
    last_average_layers_per_xy_cell_ = 0.0;
    last_weak_support_candidates_ = 0;
    last_weak_support_accepted_ = 0;
    last_weak_support_density_rejected_ = 0;
  }

  void updateRangeRingStats(const PointXYZIRT & p) {
    const double r = std::hypot(p.x, p.y);
    last_range_min_ = std::min(last_range_min_, r);
    last_range_max_ = std::max(last_range_max_, r);
    last_ring_min_ = std::min(last_ring_min_, static_cast<int>(p.ring));
    last_ring_max_ = std::max(last_ring_max_, static_cast<int>(p.ring));
  }

  void clearObjectMarkers() {
    if (!pub_markers_ || pub_markers_->get_subscription_count() == 0) return;
    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = last_target_frame_.empty() ? target_frame_ : last_target_frame_;
    clear.header.stamp = now();
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear);
    pub_markers_->publish(arr);
  }

  void resetTrackingState() {
    const bool had_active_tracks = !tracks_.empty();
    track_memory_removed_ += tracks_.size();
    tracks_.clear();
    inside_vehicle_queue_.clear();
    pending_exit_track_id_ = -1;
    next_vehicle_entry_sequence_ = 0;
    if (had_active_tracks) clearObjectMarkers();
    last_candidates_.clear();
    last_tracking_stamp_ = 0.0;
    last_active_track_count_ = 0;
    last_lost_vehicle_count_ = last_lost_predictive_count_ = last_dormant_count_ = 0;
    last_vehicle_count_ = 0;
    last_unknown_count_ = 0;
    last_rejected_candidate_count_ = 0;
    last_moving_count_ = 0;
    last_stationary_count_ = 0;
    last_object_memory_best_match_id_.clear();
    last_object_memory_best_match_class_.clear();
    last_object_memory_best_match_score_ = 0.0;
    shape_memory_reprepare_needed_ = false;
  }

  void resetBevComponentState() {
    foreground_cell_persistence_.clear();
    last_point_bg_similarity_.clear();
    last_candidates_.clear();
    last_cluster_count_ = 0;
    last_candidate_count_ = 0;
    last_shape_graph_component_count_ = 0;
    last_shape_graph_cluster_count_ = 0;
    last_shape_graph_largest_component_nodes_ = 0;
    last_shape_graph_largest_component_edges_ = 0;
    last_shape_graph_largest_component_points_ = 0;
    last_shape_graph_endpoint_relation_count_ = 0;
    last_shape_graph_avg_edge_score_ = 0.0;
    last_shape_graph_length_relation_found_ = false;
    last_shape_graph_width_relation_found_ = false;
    last_shape_graph_height_relation_found_ = false;
  }

  void resetPerceptionRuntimeState() {
    resetTrackingState();
    resetBevComponentState();
    if (crane_beam_detector_) {
      crane_beam_detector_->reset();
    }
    cargo_extension_bev_scratch_.reset();
    if (crane_pipeline_) {crane_pipeline_->reset();crane_hybrid_policy_=LatestSuspendedPolicy{};}
    tracked_slope_clusters_.clear();
    next_slope_cluster_id_ = 1;
    has_frozen_crane_transform_ = false;
  }

  void handleResetTracking(
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    const size_t active_tracks = tracks_.size();
    const size_t candidates = last_candidates_.size();
    resetTrackingState();
    res->success = true;
    res->message = "Reset tracking state: active_tracks=" + std::to_string(active_tracks) +
                   " cleared, candidates=" + std::to_string(candidates) +
                   " cleared; track IDs remain monotonic";
  }

  void handleResetBevComponents(
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    resetBevComponentState();
    res->success = true;
    res->message = "Reset BEV component state: foreground persistence and cluster transient caches cleared";
  }

  void handleResetPerceptionState(
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    if (perception_mode_ == "crane_beam_work") {
      auto command = std::make_shared<std_msgs::msg::String>();
      command->data = "RESET";
      handleCraneCommand(command);
      res->success = true;
      res->message = "AUTO selected; runtime reset queued for the next sensor frame";
      return;
    }
    resetPerceptionRuntimeState();
    res->success = true;
    res->message = "Reset perception runtime state: tracking + BEV components cleared. Background/ROI/object memory preserved.";
  }

  std::string resolvePath(const std::string & path) const {
    if (path.empty()) return path;

    const std::string scheme = "package://";

    // 1. package://<package>/<relative_path>
    if (path.size() >= scheme.size() && path.compare(0, scheme.size(), scheme) == 0) {
      const std::string rest = path.substr(scheme.size());
      const auto slash = rest.find('/');
      if (slash == std::string::npos || slash == 0 || slash + 1 >= rest.size()) {
        throw std::runtime_error("Invalid package:// path: " + path);
      }

      const std::string package_name = rest.substr(0, slash);
      const std::string relative_path = rest.substr(slash + 1);
      const auto share_dir = ament_index_cpp::get_package_share_directory(package_name);
      return share_dir + "/" + relative_path;
    }

    // 2. /파일명 shortcut
    if (path.size() > 1 && path[0] == '/') {
      const std::string rest = path.substr(1);
      if (rest.find('/') == std::string::npos) {
        const auto share_dir = ament_index_cpp::get_package_share_directory("cx128s2_bev_perception");
        return share_dir + "/config/" + rest;
      }
      return path;
    }

    // 3. 상대 파일명 shortcut
    if (path.find('/') == std::string::npos) {
      const auto share_dir = ament_index_cpp::get_package_share_directory("cx128s2_bev_perception");
      return share_dir + "/config/" + path;
    }

    return path;
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_road_frame_, pub_valid_, pub_roi_filtered_, pub_foreground_, pub_static_background_, pub_clusters_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_, pub_roi_marker_, pub_visibility_markers_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_shape_graph_markers_, pub_shape_memory_markers_;
  rclcpp::Publisher<cx128s2_bev_perception::msg::TrackedObjectArray>::SharedPtr pub_detected_objects_;
  rclcpp::Publisher<cx128s2_bev_perception::msg::TrackedObjectArray>::SharedPtr pub_tracked_objects_;
  rclcpp::Publisher<cx128s2_bev_perception::msg::TrackedObjectArray>::SharedPtr pub_motion_states_;
  rclcpp::Publisher<cx128s2_bev_perception::msg::TrackedObjectArray>::SharedPtr pub_vehicle_tracks_;
  rclcpp::Publisher<cx128s2_bev_perception::msg::TrackedObjectArray>::SharedPtr pub_vehicle_identity_memory_;
  rclcpp::Publisher<cx128s2_bev_perception::msg::PerceptionRecordingFrame>::SharedPtr pub_recording_frame_;
  rclcpp::Publisher<diagnostic_msgs::msg::KeyValue>::SharedPtr pub_background_status_, pub_object_status_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_diagnostics_table_, pub_track_table_, pub_candidate_table_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_visibility_ring_profile_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_learn_background_, srv_save_background_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_load_background_, srv_clear_background_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_dump_background_, srv_dump_track_, srv_dump_candidate_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_clear_roi_, srv_save_roi_, srv_load_roi_;
  rclcpp::Service<cx128s2_bev_perception::srv::CommitShapeMemory>::SharedPtr srv_commit_shape_memory_;
  rclcpp::Service<cx128s2_bev_perception::srv::PrepareShapeMemoryCommit>::SharedPtr srv_prepare_shape_memory_commit_;
  rclcpp::Service<cx128s2_bev_perception::srv::ControlShapeMemory>::SharedPtr srv_clear_shape_memory_;
  rclcpp::Service<cx128s2_bev_perception::srv::ControlShapeMemory>::SharedPtr srv_freeze_shape_memory_;
  rclcpp::Service<cx128s2_bev_perception::srv::ControlShapeMemory>::SharedPtr srv_unfreeze_shape_memory_;
  rclcpp::Service<cx128s2_bev_perception::srv::ControlShapeMemory>::SharedPtr srv_dump_shape_memory_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_save_object_memory_bank_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_load_object_memory_bank_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_clear_object_memory_bank_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_dump_object_memory_bank_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset_tracking_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset_bev_components_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset_perception_state_;

  std::string input_topic_, target_frame_, transformed_cloud_topic_, output_frame_policy_, recording_frame_topic_;
  std::string roi_shape_, roi_apply_stage_, background_mode_, background_update_mode_, background_map_file_;
  std::string diagnostics_table_topic_, background_table_file_, track_table_file_, candidate_table_file_, roi_file_;
  std::string roi_filtered_points_topic_;
  std::string background_frame_id_, last_input_frame_id_, last_target_frame_;
  std::string background_learning_input_mode_ = "valid";
  std::string roi_policy_file_ = "/cx128s2_roi_policy.yaml";
  std::vector<RoiZone> roi_zones_;
  VisibilityModel visibility_model_;
  VisibilityModel cargo_visibility_model_;
  bool cargo_visibility_configured_=false;
  bool visibility_enabled_ = false, visibility_frame_valid_ = false;
  bool visibility_azimuth_bin_measured_ = false;
  bool visibility_ring_profile_published_ = false;
  cx128s2_bev_perception::GeneralGroundNoiseConfig general_ground_noise_config_;
  cx128s2_bev_perception::GeneralGroundNoiseFilter general_ground_noise_filter_;
  cx128s2_bev_perception::GeneralGroundNoiseStats last_general_ground_noise_stats_;
  std::vector<int> general_ground_noise_removed_;
  bool last_input_has_ring_ = false;
  bool visibility_prewarm_pending_ = true;
  bool visibility_publish_markers_ = true;
  bool visibility_adaptive_loss_enabled_ = false;
  std::vector<std::pair<tf2::Vector3, cx128s2_bev_perception::VisibilityReason>> visibility_static_debug_cells_;
  double visibility_static_lookup_ms_ = 0.0, visibility_depth_build_ms_ = 0.0;
  double visibility_cluster_ms_ = 0.0, visibility_track_ms_ = 0.0;
  double adaptive_track_visibility_ms_ = 0.0, track_memory_update_ms_ = 0.0;
  size_t track_memory_created_ = 0, track_memory_removed_ = 0;
  bool use_tf_transform_ = true, fallback_to_raw_frame_ = false, publish_transformed_cloud_ = true;
  bool publish_debug_topics_ = false;
  bool publish_recording_frame_ = false, recording_include_full_cloud_ = true,
       recording_include_foreground_cloud_ = false;
  bool roi_enabled_ = false, require_manual_background_learning_ = true, learning_active_ = false;
  bool background_ready_ = false, disable_alerts_during_background_learning_ = true;
  bool freeze_background_after_learning_ = true, auto_save_background_after_learning_ = true;
  bool auto_load_background_on_start_ = false, use_temporal_ground_map_ = true;
  bool use_roi_policy_ = true, require_watch_roi_ = false, discard_points_outside_watch_roi_ = false;
  bool apply_ignore_before_learning_ = true, publish_roi_policy_markers_ = true;
  bool enable_gdg_density_test_ = true;
  bool enable_radius_outlier_removal_ = false;
  bool skip_ror_when_dense_ = true;
  int ror_max_input_points_ = 20000;
  int last_ror_input_count_ = 0;
  bool last_ror_skipped_ = false;
  bool vehicle_detector_enabled_ = true, unknown_obstacle_detector_enabled_ = true;
  bool vehicle_require_entry_direction_ = false;
  bool raw_vehicle_box_refinement_enabled_ = false;
  int raw_vehicle_box_max_objects_ = 4;
  double raw_vehicle_box_margin_m_ = 0.10, raw_vehicle_box_min_foreground_ratio_ = 0.25;
  double raw_vehicle_box_max_centroid_shift_m_ = 0.35;
  double raw_vehicle_box_refine_ms_ = 0.0;
  int raw_vehicle_box_queries_ = 0, raw_vehicle_box_used_ = 0;
  cx128s2_bev_perception::raw_box::Workspace raw_vehicle_box_workspace_;
  cx128s2_bev_perception::raw_box::Statistics raw_vehicle_box_stats_;
  std::vector<int> frame_cluster_owner_;
  bool enable_tracking_ = true, keep_box_while_lost_ = true, enable_risk_assessment_ = false;
  bool kf_debug_ = false;
  double visibility_box_prior_weight_ = 0.75;
  double vehicle_fixed_yaw_rad_ = 0.0;
  bool enable_static_structure_filter_ = true;
  bool use_fixed_ground_plane_ = false;
  double fixed_ground_z_ = 0.0;
  bool publish_debug_transformed_cloud_ = false, publish_debug_valid_points_ = true;
  bool publish_debug_roi_filtered_points_ = true;
  bool publish_debug_foreground_points_ = true;
  bool publish_debug_static_background_points_ = false;
  bool publish_debug_clusters_ = true;
  bool publish_shape_graph_markers_ = true, publish_shape_memory_markers_ = true;
  bool shape_graph_use_top_view_ = true, shape_graph_use_side_view_ = true, shape_graph_use_front_view_ = true;
  bool enable_shape_memory_tracking_ = true, shape_memory_keep_while_lost_ = true;
  bool publish_diagnostics_table_ = true, print_diagnostics_table_to_console_ = true;
  bool save_csv_on_shutdown_ = true, warned_missing_ring_ = false, last_input_has_time_ = false;
  double tf_timeout_sec_ = 0.05, valid_x_min_ = -20.0, valid_x_max_ = 220.0;
  double valid_y_min_ = -120.0, valid_y_max_ = 120.0, valid_z_min_ = -5.0, valid_z_max_ = 8.0;
  double valid_range_min_ = 0.2, valid_range_max_ = 200.0;
  double roi_x_min_ = 0.0, roi_x_max_ = 200.0, roi_y_min_ = -8.0, roi_y_max_ = 8.0;
  std::vector<double> roi_polygon_x_, roi_polygon_y_;
  double background_learning_duration_sec_ = 20.0, bg_x_res_ = 0.5, bg_y_res_ = 0.5;
  double background_occupancy_threshold_ = 0.7, background_height_diff_threshold_ = 0.35;
  double background_std_multiplier_ = 3.0, gdg_density_threshold_ = 0.05, gdg_min_std_z_ = 0.05;
  double foreground_min_height_change_ = 0.35;
  double foreground_std_multiplier_ = 3.0, x_bin_size_ = 2.0, y_bin_size_ = 0.5;
  double ground_percentile_ = 0.15, ground_map_alpha_ = 0.05;
  double sensor_origin_x_ = 0.0, sensor_origin_y_ = 0.0;
  double near_range_max_ = 20.0, mid_range_max_ = 45.0, long_range_max_ = 100.0;
  double ground_margin_near_ = 0.20, ground_margin_mid_ = 0.30, ground_margin_long_ = 0.50;
  double obstacle_min_height_above_ground_ = 0.35, obstacle_max_height_above_ground_ = 5.0;
  double bev_x_res_ = 0.5, bev_y_res_ = 0.25, ror_radius_ = 0.5;
  double vehicle_min_height_ = 0.6, vehicle_max_height_ = 4.5;
  double vehicle_min_width_ = 1.0, vehicle_max_width_ = 4.0, vehicle_max_length_ = 15.0;
  double vehicle_min_score_ = 0.60, unknown_min_dynamic_score_ = 0.4;
  double human_like_min_height_ = 0.8, human_like_max_height_ = 2.2;
  double human_like_max_width_ = 1.2, human_like_max_length_ = 1.2;
  double crane_like_min_height_ = 3.0, crane_like_min_vertical_extent_ = 2.5;
  double crane_like_max_speed_ = 0.3;
  double gate_threshold_near_ = 9.0, gate_threshold_mid_ = 12.0, gate_threshold_long_ = 16.0;
  double motion_speed_threshold_ = 0.3, direction_dominance_ratio_ = 1.5;
  double vehicle_stopped_duration_sec_ = 10.0;
  double predicted_track_marker_alpha_ = 0.18, static_velocity_threshold_ = 0.25;
  std::string tracking_filter_mode_ = "kalman";
  double kf_process_accel_std_mps2_ = 1.5, kf_measurement_position_std_m_ = 0.10;
  double kf_initial_position_std_m_ = 0.10, kf_initial_velocity_std_mps_ = 1.0;
  double kf_min_dt_sec_ = 0.01, kf_max_dt_sec_ = 0.5;
  double wall_length_min_ = 4.0, wall_width_max_ = 0.8, wall_elongation_ratio_min_ = 4.0;
  double diagnostics_table_period_sec_ = 1.0, marker_lifetime_ = 0.5, learning_start_time_ = -1.0;
  double debug_cloud_publish_period_sec_ = 0.2, last_debug_cloud_publish_time_ = 0.0;
  double shape_graph_voxel_size_ = 0.25;
  double shape_graph_max_edge_dist_near_ = 1.5, shape_graph_max_edge_dist_mid_ = 3.0;
  double shape_graph_max_edge_dist_long_ = 5.0;
  double shape_graph_top_weight_ = 0.5, shape_graph_side_weight_ = 0.25;
  double shape_graph_front_weight_ = 0.25, shape_graph_min_edge_score_ = 0.6;
  double shape_graph_marker_lifetime_ = 0.5;
  double shape_memory_match_weight_ = 0.5, shape_memory_distance_weight_ = 0.5;
  double shape_memory_min_match_score_ = 0.35, shape_memory_update_alpha_ = 0.2;
  double shape_memory_dimension_shrink_limit_ = 0.15;
  double shape_memory_compare_voxel_size_near_ = 0.50, shape_memory_compare_voxel_size_mid_ = 0.80;
  double shape_memory_compare_voxel_size_long_ = 1.20;
  double shape_memory_reprepare_match_threshold_ = 0.25;
  double last_diag_publish_time_ = 0.0, last_range_min_ = 0.0, last_range_max_ = 0.0;
  double voxel_size_ = 0.1;  // 논문 voxel_size: GDG 생성 및 비교 시 사용
  int background_learning_frames_ = 200, min_points_per_background_cell_ = 3;
  int foreground_count_diff_threshold_ = 3, min_points_per_ground_cell_ = 3;
  int foreground_min_persistence_frames_ = 3, min_points_per_occupied_cell_ = 2;
  int ror_min_neighbors_ = 2, cluster_connectivity_ = 8;
  bool cluster_z_guard_enabled_ = false, cluster_gap_bridging_enabled_ = false;
  bool cluster_z_layer_enabled_ = false, cluster_weak_support_enabled_ = false;
  bool cluster_density_enabled_ = false;
  bool cluster_z_profile_diagnostics_enabled_ = false;
  bool cluster_edge_diagnostics_enabled_ = true;
  double cluster_neighbor_max_z_gap_ = 0.8, cluster_bridge_max_z_gap_ = 0.4;
  double cluster_z_layer_resolution_ = 0.3;
  double cluster_z_profile_bin_size_ = 0.30;
  int cluster_gap_cells_near_ = 0, cluster_gap_cells_mid_ = 1, cluster_gap_cells_long_ = 1;
  int cluster_density_window_cells_ = 1, cluster_weak_support_min_density_ = 4;
  int min_cluster_cells_near_ = 3, min_cluster_cells_mid_ = 2, min_cluster_cells_long_ = 1;
  int min_cluster_points_near_ = 8, min_cluster_points_mid_ = 4, min_cluster_points_long_ = 2;
  int shape_graph_min_component_nodes_near_ = 3, shape_graph_min_component_nodes_mid_ = 2;
  int shape_graph_min_component_nodes_long_ = 1;
  int shape_memory_min_update_points_ = 20, shape_memory_min_update_nodes_ = 3;
  int shape_memory_max_lost_frames_near_ = 20, shape_memory_max_lost_frames_mid_ = 35;
  int shape_memory_max_lost_frames_long_ = 60;
  std::string shape_memory_learning_mode_ = "manual_commit";
  bool shape_memory_freeze_after_commit_ = true;
  bool shape_memory_allow_auto_update_after_commit_ = false;
  bool shape_memory_allow_dimension_shrink_ = false;
  bool shape_memory_clear_observed_cache_after_commit_ = true;
  bool shape_memory_reprepare_enabled_ = true;
  bool object_memory_bank_enabled_ = true;
  bool auto_load_object_memory_bank_on_start_ = true;
  bool auto_save_object_memory_bank_after_commit_ = true;
  bool shape_memory_mirror_mode_enabled_ = true;
  bool shape_memory_mirror_only_after_prepare_ready_ = true;
  bool shape_memory_mirror_warn_if_near_identical_ = true;
  bool fem_assisted_clustering_enabled_ = false;
  bool fem_cluster_absorb_orphan_points_ = false;
  bool fem_cluster_split_attached_unknown_ = true;
  bool fem_cluster_split_cargo_ = true;
  bool cargo_detection_enabled_ = false;
  int shape_memory_commit_min_nodes_ = 5;
  int shape_memory_commit_min_edges_ = 4;
  double shape_memory_commit_min_confidence_ = 0.5;
  double shape_memory_edge_missing_decay_ = 0.02;
  double shape_memory_node_missing_decay_ = 0.02;
  double shape_memory_min_confidence_to_use_ = 0.25;
  int shape_memory_max_endpoint_relations_ = 12;
  int shape_memory_compare_max_points_ = 300;
  int shape_memory_max_match_nodes_ = 80;
  int shape_memory_max_match_edges_ = 120;
  int shape_memory_max_match_relations_ = 12;
  int shape_memory_reprepare_bad_frames_ = 5;
  int shape_memory_mirror_min_observed_nodes_ = 5;
  int shape_memory_mirror_debug_samples_ = 5;
  std::string object_memory_bank_file_ = "/cx128s2_object_memory_bank.yaml";
  std::string shape_memory_mirror_plane_ = "xz";
  std::string shape_memory_mirror_center_policy_ = "bbox_center";
  std::string shape_memory_mirror_axis_ = "local_y";
  std::vector<std::string> shape_memory_mirror_only_for_class_labels_ =
      {"TRUCK", "VEHICLE", "TRUCK_LIKE", "VEHICLE_LIKE"};
  double object_memory_bank_attach_threshold_ = 0.45;
  double shape_memory_mirror_min_observed_width_ratio_ = 0.35;
  double shape_memory_mirror_confidence_scale_ = 0.55;
  double shape_memory_mirror_virtual_node_confidence_ = 0.45;
  double shape_memory_mirror_virtual_edge_confidence_ = 0.40;
  double fem_cluster_min_match_score_ = 0.45;
  double fem_cluster_body_margin_ = 0.6;
  double fem_cluster_edge_support_dist_ = 0.5;
  double fem_cluster_node_support_dist_ = 0.6;
  double cargo_height_delta_threshold_ = 0.5;
  double cargo_score_threshold_ = 0.5;
  std::string shape_graph_compute_policy_ = "manual_prepare";
  int prepare_shape_memory_requested_count_ = 0;
  int prepare_shape_memory_success_count_ = 0;
  int observed_shape_cache_valid_track_count_ = 0;
  int committed_shape_memory_track_count_ = 0;
  int memory_compare_using_cache_count_ = 0;
  int memory_compare_using_current_shape_graph_count_ = 0;
  int shape_graph_computed_this_frame_ = 0;
  int shape_graph_skipped_this_frame_ = 0;
  bool shape_memory_reprepare_needed_ = false;
  std::unordered_map<std::string, std::vector<ShapeMemory>> class_memory_bank_;  // TODO: persist class memories.
  ObjectMemoryBank object_memory_bank_;
  std::string last_object_memory_best_match_id_;
  std::string last_object_memory_best_match_class_;
  double last_object_memory_best_match_score_ = 0.0;
  int last_object_memory_match_attempt_count_ = 0;
  int last_object_memory_applicable_match_count_ = 0;
  int last_object_memory_attach_count_ = 0;
  int last_object_memory_skipped_by_component_type_count_ = 0;
  int last_fem_cluster_refined_count_ = 0;
  int last_fem_owned_point_count_ = 0;
  int last_fem_outside_point_count_ = 0;
  int last_fem_cargo_candidate_point_count_ = 0;
  int last_fem_attached_unknown_point_count_ = 0;
  int last_fem_split_unknown_count_ = 0;
  int last_fem_split_cargo_count_ = 0;
  int unknown_min_persistence_frames_ = 3, tentative_confirm_frames_ = 5, tentative_confirm_hits_ = 3;
  int max_tracking_candidates_ = 80, max_active_tracks_ = 100;
  int max_lost_frames_near_ = 15, max_lost_frames_mid_ = 25, max_lost_frames_long_ = 40;
  bool vehicle_reacquire_enabled_ = true;
  double vehicle_lost_memory_sec_ = 4.0, vehicle_identity_memory_sec_ = 36000.0;
  double last_tracking_stamp_ = 0.0;
  cx128s2_bev_perception::vehicle_surface::SingleLaneVehicleQueue inside_vehicle_queue_;
  uint64_t next_vehicle_entry_sequence_ = 0;
  int pending_exit_track_id_ = -1;
  int static_persistence_frames_ = 5, learned_frame_count_ = 0, next_track_id_ = 1, frame_id_ = 0;
  int debug_cloud_max_points_ = 50000;
  int last_input_count_ = 0, last_transformed_count_ = 0, last_valid_count_ = 0, last_ground_count_ = 0;
  int last_obstacle_count_ = 0, last_ignored_count_ = 0, last_ground_cell_count_ = 0;
  int last_roi_filtered_count_ = 0, last_roi_ignored_count_ = 0, last_roi_hard_ignored_count_ = 0;
  int last_foreground_count_ = 0, last_static_count_ = 0, last_dynamic_count_ = 0, last_rejected_count_ = 0;
  int last_cluster_count_ = 0, last_candidate_count_ = 0, last_active_track_count_ = 0;
  int last_vehicle_count_ = 0, last_unknown_count_ = 0, last_rejected_candidate_count_ = 0;
  int last_moving_count_ = 0, last_stationary_count_ = 0, last_ring_min_ = 0, last_ring_max_ = 0;
  int last_tracking_feature_input_count_ = 0, last_tracking_candidate_count_ = 0;
  int last_active_track_count_before_ = 0, last_tracking_association_pairs_checked_ = 0;
  int last_shape_graph_node_count_ = 0, last_shape_graph_edge_count_ = 0;
  int last_shape_graph_component_count_ = 0, last_shape_graph_cluster_count_ = 0;
  int last_shape_graph_largest_component_nodes_ = 0;
  int last_shape_graph_largest_component_edges_ = 0;
  int last_shape_graph_largest_component_points_ = 0;
  int last_shape_graph_endpoint_relation_count_ = 0;
  int last_shape_memory_track_count_ = 0, last_shape_memory_association_used_count_ = 0;
  int last_debug_valid_published_points_ = 0, last_debug_roi_filtered_published_points_ = 0;
  int last_debug_foreground_published_points_ = 0;
  int last_debug_clusters_published_points_ = 0;
  
  std::string foreground_classifier_mode_ = "statistical_score";
  double foreground_score_threshold_ = 1.0;
  double foreground_count_weight_ = 0.45;
  double foreground_mean_z_weight_ = 0.25;
  double foreground_max_z_weight_ = 0.20;
  double foreground_min_z_weight_ = 0.10;
  double foreground_unknown_cell_score_ = 1.2;
  double foreground_min_bg_std_z_ = 0.05;
  double foreground_count_score_clip_ = 3.0;
  double foreground_z_score_clip_ = 3.0;
  double foreground_pdf_density_threshold_ = 0.3;
  int foreground_voxel_diff_threshold_ = 3;

  int last_gdg_pdf_foreground_cell_count_ = 0;
  int last_gdg_pdf_tested_cell_count_ = 0;
  int last_gdg_pdf_tested_point_count_ = 0;
  int last_gdg_pdf_foreground_point_count_ = 0;
  int last_gdg_pdf_background_point_count_ = 0;
  int last_gdg_pdf_unknown_cell_point_count_ = 0;
  int last_gdg_pdf_count_diff_rejected_cell_count_ = 0;
  bool last_foreground_persistence_bypassed_ = false;
  bool last_shape_graph_marker_published_ = false;
  bool last_shape_graph_length_relation_found_ = false;
  bool last_shape_graph_width_relation_found_ = false;
  bool last_shape_graph_height_relation_found_ = false;
  double last_shape_graph_avg_edge_score_ = 0.0;
  double last_shape_memory_match_score_max_ = 0.0;
  double last_shape_memory_match_score_mean_ = 0.0;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  int last_stat_count_reason_count_ = 0;
  int last_stat_mean_z_reason_count_ = 0;
  int last_stat_max_z_reason_count_ = 0;
  int last_stat_min_z_reason_count_ = 0;
  int last_stat_unknown_cell_reason_count_ = 0;
  double last_foreground_score_max_ = 0.0;
  double last_foreground_positive_score_mean_ = 0.0;
  int last_foreground_score_positive_cell_count_ = 0;

  std::string cluster_input_mode_ = "foreground";
  std::string clustering_method_ = "bev_connected_component";
  std::unique_ptr<IClusterer> clusterer_;
  // Pointers are valid only while processCloudFrame() owns its local feature vector.  They expose
  // the exact tracking association for the recorder publisher without retaining point indices in Track.
  std::unordered_map<int, const ObjectFeature *> recording_track_features_;
  // Stable storage through publication and track vector compaction this frame.
  std::unordered_map<int, ObjectFeature> recording_sparse_features_;
  std::unordered_map<int64_t, BackgroundCell> background_map_;
  std::unordered_map<int64_t, AccumCell> learning_accum_;
  std::unordered_map<int64_t, GroundCell> temporal_ground_map_;
  std::unordered_map<int64_t, int> foreground_cell_persistence_;
  std::vector<double> last_point_bg_similarity_;
  std::vector<Track> tracks_;
  std::vector<ObjectFeature> last_candidates_;
  std::vector<PointXYZIRT> frame_points_;

  // Phase 1 timing diagnostics variables (in milliseconds)
  double read_cloud_ms_ = 0.0;
  double valid_crop_ms_ = 0.0;
  double roi_policy_ms_ = 0.0;
  double background_learning_ms_ = 0.0;
  double foreground_classification_ms_ = 0.0;
  double radius_outlier_removal_ms_ = 0.0;
  double clustering_ms_ = 0.0;
  double bev_build_ms_ = 0.0;
  double connected_component_ms_ = 0.0;
  double cluster_point_gather_ms_ = 0.0;
  double cluster_filter_ms_ = 0.0;
  double z_layer_build_ms_ = 0.0;
  double density_build_ms_ = 0.0;
  double z_profile_build_ms_ = 0.0;
  double z_profile_edge_diag_ms_ = 0.0;
  double feature_build_ms_ = 0.0;
  double tracking_ms_ = 0.0;
  double vehicle_surface_predict_ms_ = 0.0, vehicle_visibility_query_ms_ = 0.0;
  double vehicle_ownership_ms_ = 0.0, vehicle_sparse_support_ms_ = 0.0;
  double vehicle_lost_predict_ms_ = 0.0, vehicle_reacquire_ms_ = 0.0;
  double vehicle_dormant_management_ms_ = 0.0, vehicle_dormant_reacquire_ms_ = 0.0;
  double vehicle_fifo_identity_ms_ = 0.0;
  uint64_t vehicle_fifo_entry_count_ = 0, vehicle_fifo_exit_candidate_count_ = 0;
  uint64_t vehicle_fifo_reacquire_success_count_ = 0, vehicle_fifo_validation_reject_count_ = 0;
  uint64_t vehicle_fifo_direction_unknown_count_ = 0, vehicle_fifo_expired_count_ = 0;
  uint64_t vehicle_fifo_anomaly_count_ = 0, vehicle_fifo_exit_complete_count_ = 0;
  int last_lost_vehicle_count_ = 0, last_lost_predictive_count_ = 0, last_dormant_count_ = 0;
  uint64_t vehicle_lost_enter_count_ = 0, vehicle_reacquire_attempt_count_ = 0;
  uint64_t vehicle_reacquire_success_count_ = 0, vehicle_reacquire_ambiguous_count_ = 0;
  uint64_t vehicle_reacquire_reject_count_ = 0, vehicle_lost_expired_count_ = 0;
  uint64_t vehicle_lost_prediction_expired_count_ = 0, vehicle_dormant_enter_count_ = 0;
  uint64_t vehicle_dormant_expired_count_ = 0, vehicle_dormant_reacquire_attempt_count_ = 0;
  uint64_t vehicle_dormant_reacquire_success_count_ = 0;
  uint64_t vehicle_dormant_reacquire_ambiguous_count_ = 0;
  double shape_memory_match_ms_ = 0.0;
  double fem_refine_ms_ = 0.0;
  double debug_cloud_build_ms_ = 0.0;
  double marker_build_ms_ = 0.0;
  double publish_ms_ = 0.0;
  double object_arrays_build_ms_ = 0.0, object_arrays_publish_ms_ = 0.0;
  double track_table_build_ms_ = 0.0, track_table_publish_ms_ = 0.0;
  double candidate_table_build_ms_ = 0.0, candidate_table_publish_ms_ = 0.0;
  double total_pipeline_ms_ = 0.0;
  double input_queue_delay_ms_ = 0.0;
  int last_occupied_cell_count_ = 0;
  int last_connected_component_count_ = 0;
  int last_valid_cluster_count_ = 0;
  int last_tiny_component_count_ = 0;
  int last_single_cell_component_count_ = 0;
  int last_two_cell_component_count_ = 0;
  int last_largest_component_cells_ = 0;
  int last_largest_component_points_ = 0;
  int last_median_component_cells_ = 0;
  int last_median_component_points_ = 0;
  std::array<int, 6> last_component_cell_histogram_{};
  std::array<int, 6> last_component_point_histogram_{};
  int last_z_rejected_direct_edges_ = 0, last_bridge_candidates_ = 0;
  int last_bridge_accepted_ = 0, last_z_rejected_bridge_edges_ = 0;
  int last_layer_cell_count_ = 0, last_strong_layer_cell_count_ = 0;
  int last_weak_layer_cell_count_ = 0, last_max_layers_per_xy_cell_ = 0;
  int last_weak_support_candidates_ = 0, last_weak_support_accepted_ = 0;
  int last_weak_support_density_rejected_ = 0;
  double last_average_layers_per_xy_cell_ = 0.0;
  size_t object_arrays_bytes_ = 0, track_table_bytes_ = 0, candidate_table_bytes_ = 0;

  // Frame processing counters
  uint64_t processed_frame_count_ = 0;
  uint64_t received_frame_count_ = 0;
  uint64_t dropped_frame_count_ = 0;
  uint64_t sequence_counter_ = 0;

  bool enable_parallel_processing_ = false;
  int parallel_threads_ = 2;

  std::string crane_pipeline_name_ = "legacy";
  std::unique_ptr<cx128s2_bev_perception::CranePipeline> crane_pipeline_;
  bool crane_slope_publish_debug_clouds_ = false, crane_slope_publish_colored_cloud_ = false,
      crane_slope_publish_direction_clouds_ = false;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_crane_slope_ground_,
      pub_crane_slope_non_ground_, pub_crane_slope_unknown_, pub_crane_slope_clusters_, pub_crane_slope_classified_,
      pub_crane_slope_vertical_, pub_crane_slope_horizontal_;
  bool crane_boundary_clustering_enabled_ = false;
  bool crane_z_gaussian_clustering_enabled_ = false;
  bool crane_closed_loop_grouping_enabled_ = false;
  bool crane_slope_roi_filter_enabled_ = true;
  bool crane_slope_roi_only_crane_roi_ = true;
  bool crane_slope_roi_active_include_background_ = false;
  bool crane_slope_roi_idle_exclude_beam_roi_ = false;
  bool crane_slope_roi_expand_beam_roi_ = true;
  struct TrackedSlopeCluster {
    int stable_id = 1;
    double cx = 0.0, cy = 0.0, cz = 0.0;
    int missed_frames = 0;
  };
  std::vector<TrackedSlopeCluster> tracked_slope_clusters_;
  int next_slope_cluster_id_ = 1;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_crane_slope_boundary_,
      pub_crane_slope_boundary_clusters_, pub_crane_slope_z_gaussian_layers_,
      pub_crane_slope_closed_loop_groups_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_crane_slope_boundary_diagnostics_;
  bool crane_plane_enabled_ = false, crane_plane_publish_debug_clouds_ = false,
      crane_plane_publish_colored_cloud_ = false;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_crane_plane_, pub_crane_line_only_,
      pub_crane_non_planar_, pub_crane_surface_classified_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_crane_plane_regions_;

  bool crane_height_enabled_=false,crane_height_publish_debug_=false;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_crane_height_candidates_,
      pub_crane_height_surfaces_,pub_crane_height_unassigned_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_crane_height_regions_;

  // Crane Beam Perception members
  std::string perception_mode_ = "default";
  bool crane_beam_enabled_ = true;
  cx128s2_bev_perception::CraneBeamConfig crane_beam_config_;
  std::unique_ptr<cx128s2_bev_perception::CraneBeamDetector> crane_beam_detector_;

  rclcpp::Publisher<cx128s2_bev_perception::msg::CraneBeamStatus>::SharedPtr pub_crane_beam_status_;
  rclcpp::TimerBase::SharedPtr crane_motion_expiry_timer_;
  std::mutex crane_motion_expiry_mutex_;
  std::optional<cx128s2_bev_perception::msg::CraneBeamStatus> crane_motion_expiry_status_;
  double crane_motion_expiry_timeout_=2.5;

  rclcpp::Publisher<cx128s2_bev_perception::msg::CraneBeamMotionDebug>::SharedPtr pub_crane_beam_motion_debug_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_crane_beam_working_status_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_crane_beam_diagnostics_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_crane_beam_roi_points_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_crane_beam_bg_roi_points_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_crane_beam_z_candidates_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_crane_beam_cluster_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_crane_beam_support_points_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_crane_beam_markers_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_crane_hook_left_body_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_crane_hook_right_body_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_crane_hook_markers_;
  double last_crane_diag_time_ = 0.0;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_crane_command_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_crane_command_status_;
  std::string crane_hook_reference_file_;
  std::mutex crane_command_mutex_;
  bool crane_manual_mode_ = false, crane_reset_pending_ = false;
  std::optional<bool> crane_beam_override_, crane_ready_override_;
  std::optional<int> crane_working_override_;
  std::optional<cx128s2_bev_perception::CraneBeamState> crane_fsm_override_;
  std::optional<cx128s2_bev_perception::msg::CraneBeamStatus> crane_auto_status_;

  // Crane Beam frozen reference transform state (disables dynamic Gazebo TF compensation)
  bool has_frozen_crane_transform_ = false;
  tf2::Transform frozen_crane_transform_;
  std::string frozen_crane_input_frame_;
  std::string frozen_crane_target_frame_;

  // Threading synchronizations
  std::mutex latest_msg_mutex_;
  std::optional<QueuedCloud> latest_msg_;
  std::condition_variable latest_msg_cv_;
  std::atomic<bool> worker_running_{true};
  std::thread worker_thread_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Cx128s2BevPerception>());
  rclcpp::shutdown();
  return 0;
}
