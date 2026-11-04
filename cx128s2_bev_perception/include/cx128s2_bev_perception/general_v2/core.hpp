#pragma once
#include <Eigen/Core>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>
#include <map>
#include "cx128s2_bev_perception/vehicle_status.hpp"

namespace general_v2
{
std::string core_source_sha();
struct Point
{
  double x = 0, y = 0, z = 0;
  uint32_t source = 0;
  // Optional physical beam index. Unknown is never inferred as ring zero.
  uint16_t ring = UINT16_MAX;
};
struct Zone
{
  std::string name, type = "watch";
  bool enabled = true;
  std::vector<double> x, y;
  bool contains(double px, double py) const;
};
struct Config
{
  double xmin = -100, xmax = 100, ymin = -120, ymax = 120, zmin = -2, zmax = 6, range_min = .2,
         range_max = 200;
  double ground_tile = 2, ground_seed_min_z = -1, ground_seed_max_z = 1;
  double ground_inlier = .12, ground_max_slope = .20, ground_min_span = 3;
  int ground_min_tiles = 12, ground_min_points = 6;
  // Thin, ground-connected returns can rise above the dominant plane due to noise.
  double ground_local_cell=.5,ground_local_band=.12,ground_local_max_rise=1.0;
  double ground_local_slope=.25,ground_local_residual=.06;
  int ground_local_min_points=6;
  bool candidate_ground_noise_rejection = true;
  double obstacle_min = .2, obstacle_max = 5;
  double cell = .3, connection_gap = .48, vertical_gap = .6;
  int min_points = 25, min_cells = 4, max_objects = 128, max_cells = 200000;
  double vehicle_min_length = 2.5, vehicle_min_width = 2.2, vehicle_max_width = 4;
  double vehicle_min_height = 2.3, vehicle_max_height = 4.5, vehicle_max_length = 20;
  double confirmation_sec = .5;
  int confirmation_hits = 3;
  double hold_sec = 1.3, candidate_hold_sec = .3, reid_sec = 10, association_gate = 2.5;
  double extent_confirmation_sec = 2, extent_growth_sec = .3, partial_ratio = .8;
  double motion_window = .8, moving_speed = .3, stationary_speed = .15, motion_confirm_sec = .3,
         stopped_sec = 10;
  bool require_watch = false;
  std::vector<Zone> zones;
  void validate() const;
};
struct Ground
{
  bool valid = false;
  double a = 0, b = 0, c = 0, residual = 0;
  int support = 0, seeds = 0;
  std::string reason = "UNKNOWN";
  double local_cell=0,local_band=0;
  std::map<std::pair<int,int>,Eigen::Vector3d> local_surfaces;
  bool surface_return(const Point&) const;
  double at(double x, double y) const
  {
    return a * x + b * y + c;
  }
};
struct Observation
{
  int cluster_id = 0;
  Eigen::Vector3d center = Eigen::Vector3d::Zero(), size = Eigen::Vector3d::Zero();
  double yaw = 0;
  bool vehicle = false;
  std::vector<uint32_t> indices;
  int cells = 0;
};
struct Object
{
  int id = 0, age = 0, hits = 0, lost = 0;
  bool confirmed = false, vehicle = false, extent_locked = false, partial = false;
  std::string observation = "NONE", status = "stopping", motion = "STATIONARY";
  Eigen::Vector3d center = Eigen::Vector3d::Zero(), size = Eigen::Vector3d::Zero(),
                  observed_size = Eigen::Vector3d::Zero();
  Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
  double yaw = 0, range_speed = 0, last_direct_sec = 0;
  int cells = 0;
  std::vector<uint32_t> indices;
};
struct Result
{
  int64_t stamp_ns = 0;
  size_t raw_points = 0, valid_points = 0;
  Ground ground;
  Eigen::Vector2d sensor_origin=Eigen::Vector2d::Zero();
  std::vector<uint32_t> foreground, ground_indices, candidate_excluded;
  std::vector<Observation> observations;
  std::vector<Object> objects;
  std::string quality = "OK";
  double ground_ms = 0, cluster_ms = 0, tracking_ms = 0, total_ms = 0;
};
Ground estimate_ground(const std::vector<Point>&, const std::vector<uint32_t>&, const Config&);
std::vector<Observation> detect(const std::vector<Point>&, std::vector<uint32_t>&, const Config&,
                                const Ground&, const std::vector<uint32_t>&, std::vector<uint32_t>&);
std::vector<int> assignment(const std::vector<std::vector<double> >& costs, double unmatched);
class Core
{
public:
  explicit Core(Config config = {});
  Result process(int64_t stamp_ns, const std::vector<Point>& points, bool input_valid = true, const Eigen::Vector2d &sensor_origin=Eigen::Vector2d::Zero());
  void reset();
  void set_zones(std::vector<Zone> zones);
  const Config& config() const
  {
    return cfg_;
  }
private:
  struct Track
  {
    Object out;
    Eigen::Vector4d state = Eigen::Vector4d::Zero();
    Eigen::Matrix4d covariance = Eigen::Matrix4d::Identity();
    double first = 0, last = 0, last_direct = 0, evidence_start = 0, growth_start = -1, motion_start = -1;
    bool confirmed_vehicle = false;
    int vehicle_hits = 0;
    Eigen::Vector3d last_observed_center=Eigen::Vector3d::Zero(), last_observed_size=Eigen::Vector3d::Zero();
    double last_observed_yaw=0;
    bool last_partial=false;
    double vehicle_since = -1, vehicle_last = -1;
    Eigen::Vector3d extent = Eigen::Vector3d::Zero(), growth = Eigen::Vector3d::Zero();
    std::deque<std::pair<double, Eigen::Vector3d> > extents;
    std::deque<std::pair<double, Eigen::Vector2d> > motion_samples;
    cx128s2_bev_perception::VehicleStatusState vehicle_status;
    std::string pending_motion;
  };
  Config cfg_;
  int next_id_ = 1;
  int64_t previous_stamp_ = 0;
  std::vector<Track> tracks_;
  void update(Result&, double);
};
} // namespace general_v2
