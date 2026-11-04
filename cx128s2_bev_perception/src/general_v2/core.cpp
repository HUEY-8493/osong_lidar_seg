#include "cx128s2_bev_perception/general_v2/core.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#ifndef GENERAL_V2_SOURCE_SHA
#define GENERAL_V2_SOURCE_SHA "unknown"
#endif
namespace general_v2
{
std::string core_source_sha(){return GENERAL_V2_SOURCE_SHA;}

bool Zone::contains(double px, double py) const
{
  bool inside = false;
  if (!enabled || x.size() < 3 || x.size() != y.size())
  {
    return false;
  }
  for (size_t i = 0, j = x.size() - 1; i < x.size(); j = i++)
  {
    const double cross = (px - x[j]) * (y[i] - y[j]) - (py - y[j]) * (x[i] - x[j]);
    if (std::abs(cross) < 1e-9 && px >= std::min(x[i], x[j]) && px <= std::max(x[i],
                                                                               x[j]) && py >= std::min(y[i],
                                                                                                       y
                                                                                                       [j]) &&
        py <= std::max(y[i], y[j]))
    {
      return true;
    }
    if ((y[i] > py) != (y[j] > py) && px < (x[j] - x[i]) * (py - y[i]) / (y[j] - y[i]) + x[i])
    {
      inside = !inside;
    }
  }
  return inside;
}
void Config::validate() const
{
  for (double v:{xmin, xmax, ymin, ymax, zmin, zmax, range_min, range_max, ground_tile, ground_seed_min_z,
                 ground_seed_max_z, ground_inlier, ground_max_slope, ground_min_span, ground_local_cell, ground_local_band, ground_local_max_rise, ground_local_slope, ground_local_residual, obstacle_min,
                 obstacle_max, cell, connection_gap, vertical_gap, vehicle_min_length, vehicle_min_width,
                 vehicle_max_width, vehicle_min_height, vehicle_max_height, vehicle_max_length,
                 confirmation_sec, hold_sec, candidate_hold_sec, reid_sec, association_gate,
                 extent_confirmation_sec, extent_growth_sec, partial_ratio, motion_window, moving_speed,
                 stationary_speed, motion_confirm_sec, stopped_sec})
  {
    if (!std::isfinite(v))
    {
      throw std::invalid_argument("GeneralV2 config contains a nonfinite value");
    }
  }
  if (xmin >= xmax || ymin >= ymax || zmin >= zmax || range_min < 0 || range_max <= range_min ||
      ground_tile <= 0 ||
      ground_seed_min_z >= ground_seed_max_z || ground_inlier <= 0 || ground_max_slope <= 0 ||
      ground_min_span <= 0 ||
      ground_min_tiles < 3 || ground_min_points < 1 || ground_local_cell<=0 || ground_local_band<=0 || ground_local_max_rise<0 || ground_local_slope<=0 || ground_local_residual<=0 || ground_local_min_points<3 || obstacle_min < 0 || obstacle_max <= obstacle_min ||
      cell <= 0 ||
      connection_gap < 0 || connection_gap / cell > 8 || vertical_gap < 0 || min_points < 3 ||
      min_cells < 1 || max_cells < 1 ||
      max_objects < 1 || max_objects > 512 || vehicle_min_length <= 0 || vehicle_min_width <= 0 ||
      vehicle_max_width < vehicle_min_width || vehicle_min_height <= 0 ||
      vehicle_max_height < vehicle_min_height ||
      vehicle_max_length < vehicle_min_length || confirmation_sec < 0 || confirmation_hits < 1 ||
      hold_sec <= 0 ||
      candidate_hold_sec <= 0 || reid_sec < hold_sec || association_gate <= 0 ||
      extent_confirmation_sec <= 0 ||
      extent_growth_sec <= 0 || partial_ratio <= 0 || partial_ratio >= 1 || motion_window <= 0 ||
      moving_speed <= stationary_speed || stationary_speed < 0 || motion_confirm_sec < 0 || stopped_sec < 0)
  {
    throw std::invalid_argument("GeneralV2 config out of range");
  }
  for (const auto&z:zones)
  {
    if (z.x.size() != z.y.size() || z.x.size() < 3 ||
        (z.type != "watch" && z.type != "ignore" && z.type != "hard_ignore"))
    {
      throw std::invalid_argument("GeneralV2 invalid ROI zone");
    }
  }
}
Core::Core(Config config) : cfg_(std::move(config))
{
  cfg_.validate();
}
void Core::reset()
{
  tracks_.clear();
  previous_stamp_ = 0;
}
void Core::set_zones(std::vector<Zone> zones)
{
  auto candidate = cfg_;
  candidate.zones = std::move(zones);
  candidate.validate();
  cfg_ = std::move(candidate);
  reset();
}
Result Core::process(int64_t stamp, const std::vector<Point>& points, bool input_valid, const Eigen::Vector2d &sensor_origin)
{
  using Clock = std::chrono::steady_clock;
  const auto start = Clock::now();
  auto ms = [](auto a){
              return std::chrono::duration<double, std::milli>(Clock::now() - a).count();
            };
  Result r;
  r.stamp_ns = stamp;
  r.sensor_origin=sensor_origin;
  if(!sensor_origin.allFinite())throw std::invalid_argument("Invalid sensor origin");
  r.raw_points = points.size();
  if (stamp <= 0)
  {
    r.quality = "INVALID_SENSOR_TIMESTAMP";
    return r;
  }
  if (stamp == previous_stamp_)
  {
    r.quality = "DUPLICATE_SENSOR_TIMESTAMP";
    return r;
  }
  if (previous_stamp_ && stamp < previous_stamp_)
  {
    reset();
    r.quality = "TIMESTAMP_REGRESSION_NEW_SESSION";
  }
  previous_stamp_ = stamp;
  std::vector<uint32_t> valid;
  bool has_watch = false;
  for (const auto&z:cfg_.zones)
    has_watch |= z.enabled && z.type == "watch";
  if (input_valid)
  {
    for (size_t i = 0; i < points.size(); ++i)
    {
      const auto&p = points[i];
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || p.x < cfg_.xmin ||
          p.x > cfg_.xmax ||
          p.y < cfg_.ymin || p.y > cfg_.ymax || p.z < cfg_.zmin || p.z > cfg_.zmax)
      {
        continue;
      }
      const double range = std::hypot(p.x-sensor_origin.x(), p.y-sensor_origin.y());
      if (range < cfg_.range_min || range > cfg_.range_max)
      {
        continue;
      }
      bool rejected = false, watch = false;
      for (const auto&z:cfg_.zones)
      {
        if (z.contains(p.x, p.y))
        {
          if (z.type == "watch")
          {
            watch = true;
          }
          else{
            rejected = true;
          }
        }
      }
      if (!rejected && (!cfg_.require_watch || !has_watch || watch))
      {
        valid.push_back(uint32_t(i));
      }
    }
  }
  r.valid_points = valid.size();
  auto stage = Clock::now();
  if (input_valid)
  {
    r.ground = estimate_ground(points, valid, cfg_);
  }
  else{
    r.quality = "INPUT_UNAVAILABLE";
  }
  if (r.ground.valid)
  {
    for (auto i:valid)
    {
      const auto&p = points[i];
      const double dz = p.z - r.ground.at(p.x, p.y);
      if (dz < cfg_.obstacle_min || r.ground.surface_return(p))
      {
        r.ground_indices.push_back(i);
      }
      else if (dz <= cfg_.obstacle_max)
      {
        r.foreground.push_back(i);
      }
    }
  }
  else if (input_valid && r.quality == "OK")
  {
    r.quality = "GROUND_UNKNOWN";
  }
  r.ground_ms = ms(stage);
  stage = Clock::now();
  r.observations = detect(points, r.foreground, cfg_, r.ground, valid, r.candidate_excluded);
  r.cluster_ms = ms(stage);
  stage = Clock::now();
  update(r, double(stamp) * 1e-9);
  r.tracking_ms = ms(stage);
  r.total_ms = ms(start);
  return r;
}
}
