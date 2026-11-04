#pragma once
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
namespace crane_v2 {
struct Point {
  float x, y, z;
  uint32_t raw;
};
struct Vec3 {
  double x = 0, y = 0, z = 0;
};
struct Config {
  std::string crane_motion_mode = "scene_histogram";
  double crane_hist_xy_bin_m=.5;
  double crane_hist_bin_m=.10, crane_hist_start_score=.08, crane_hist_stop_score=.03;
  double crane_hist_lag_sec=.4, crane_hist_start_confirm_sec=.2, crane_hist_stop_confirm_sec=.8;
  int crane_hist_min_points=1000;
  bool crane_xy_enabled = true;
  double crane_xy_start_m = .04, crane_xy_min_speed = .03;
  double crane_xy_start_confirm_sec = .2, crane_xy_stop_span_m = .015;
  double crane_xy_stop_confirm_sec = .8;
  int crane_xy_min_points = 15;
  // Existing calibrated fixed-root ROIs: xmin,xmax,ymin,ymax,zmin,zmax.
  std::array<std::array<double,6>,2> crane_root_rois{{
    {{-1.25,-.55,-.25,.35,4.95,5.55}},
    {{.72,1.42,-.25,.35,5.00,5.60}}}};
  double cargo_length_confirm_sec = 2.;
  // Downstream display/state grace for an abruptly smaller SAME confirmed load.
  // Zero disables it. Hook occlusion and missing-Cargo timeout are unchanged.
  double cargo_shrink_hold_sec = 3., cargo_shrink_ratio = .5, cargo_shrink_recovery_ratio = .8;
  double voxel_xy = .08, voxel_z = .05;
  // Installation/watch limits in calibrated world coordinates, not cargo length.
  double x_min = -19.04839650145773, x_max = 30.43498542274051;
  double y_min = -9.9148688046647, y_max = 19.822740524781352, z_min = -.5, z_max = 6.3;
  double virtual_hook_offset = .9, connection_gap = 1.5;
  double confirm_sec = 2., loss_sec = 1.3, occlusion_sec = .5;
  double suspended_clearance = .10, contact_clearance = .38, completion_hold_sec = 3.;
  double contact_confirm_sec = .3, detach_confirm_sec = .3;
  double slow_motion_window = 1.5, slow_motion_threshold = .018;
  double motion_window = .5, motion_threshold = .025;
};
struct Hook {
  Vec3 body_min, body_max;
  std::vector<uint32_t> raw_indices; // Current selected Holder Raw, visualization only.
  bool direct = false, held = false;
  Vec3 housing, virtual_hook, delta;
  uint32_t points = 0;
  double last_direct_sec = 0, confidence = 0;
};
struct Cargo {
  Vec3 observed_start,observed_end,observed_axis;double observed_length_3d=0;
  std::string length_observation="PARTIAL_OBSERVATION",model_extent_observation="ESTIMATED_EXTENT";
  uint32_t start_raw_support=0,end_raw_support=0,extent_added_points=0,extent_removed_points=0;
  bool extent_gap_recovered=false;
  std::string bottom_observation = "UNKNOWN";
  double measured_section_depth = 0, raw_z_min = 0, raw_z_max = 0;
  uint64_t id = 0;
  int supporting_side = -1;
  bool direct = false, held = false, confirmed = false, connected = false, bottom_valid = false;
  Vec3 center;
  double length = 0, width = 0, height = 0, yaw = 0, bottom_z = 0;
  double model_length = 0, model_width = 0, last_direct_sec = 0;
  bool length_locked = false;
  std::vector<uint32_t> raw_indices;
};
struct Ground {
  // LOCAL_PLANE is estimated under cargo from CURRENT neighboring raw floor.
  // DIRECT only describes the measured floor samples, never an occluded underside.
  bool valid = false;
  std::string observation = "UNKNOWN";
  double z = 0, mad = 0;
  std::vector<uint32_t> raw_indices;
};
struct Output {
  double crane_hist_score=0, crane_hist_recent_score=0;
  std::array<double,3> crane_hist_axis_scores{};
  uint32_t crane_hist_points=0;
  struct ExtentBox { Vec3 center; double length=0,width=0,height=0,yaw=0; };
  bool retained_box_valid = false;
  ExtentBox retained_box;
  bool crane_moving = false, crane_motion_valid = false;
  std::string crane_motion_reason = "WAIT_BODY_ROOTS";
  std::array<Vec3,2> crane_roots{};
  std::array<bool,2> crane_root_observed{};
  std::array<uint32_t,2> crane_root_points{};
  // Current measured fragment is distinct from a historical HELD display box.
  bool cargo_measurement_direct = false, cargo_shrink_held = false;
  double measured_cargo_length = 0, measured_cargo_width = 0, measured_cargo_height = 0;
  Vec3 measured_cargo_center;
  uint32_t measured_cargo_points = 0;
  double cargo_shrink_hold_elapsed_sec = 0;
  // Populated only during shrink hold: actual current fragment Raw, never old Raw.
  std::vector<uint32_t> measured_cargo_raw_indices;
  int64_t stamp_ns = 0;
  std::string state = "IDLE", reason = "NO_CONFIRMED_CARGO";
  std::string connection_observation = "UNKNOWN";
  bool hook_separation_observed = false; // Geometry retention, independent of FSM armed state.
  Cargo cargo;
  Ground ground;
  std::array<Hook, 2>hooks;
  bool contact_ground_direct_valid = false;
  bool ground_supported = false, ground_support_held = false;
  bool clearance_valid = false, work_complete = false, work_active = false;
  double clearance = 0, motion_dz = 0, matched_fraction = 0;
  uint32_t input_points = 0, voxel_cells = 0, components = 0, candidates = 0;
  std::vector<uint32_t>support_path_raw;
  std::vector<std::array<double, 21> >candidate_metrics;
  std::array<double, 5>stage_ms{}; // index, hook, ground, cargo, tracking/motion/FSM
};
// No ROS, legacy perception, KF, process-global buffers, worker or clock access.
// One instance is used serially. Reset destroys all observation/event history.
class Engine {
public: explicit Engine(Config config = {});
~Engine();
Engine(Engine&&)noexcept;
Engine&operator=(Engine&&)noexcept;
Engine(const Engine&) = delete;
Engine&operator=(const Engine&) = delete;
Output process(const std::vector<Point>&world_points, int64_t sensor_stamp_ns);
void reset();
private: struct Impl;
std::unique_ptr<Impl>impl_;
};
}
