#include "cx128s2_bev_perception/general_v2/core.hpp"
#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <limits>
namespace general_v2
{
namespace
{
double angle_distance(double a, double b)
{
  return std::abs(std::remainder(a - b, std::acos(-1.)));
}
}
void Core::update(Result&r, double now)
{
  // Keep a bounded dormant identity only for confirmed stationary vehicles.
  tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(), [&](const Track&t){
      const double ttl = t.out.confirmed && t.confirmed_vehicle &&
                         t.out.status ==
                         "stopped" ? cfg_.reid_sec :
                         (t.out.confirmed ? cfg_.hold_sec : cfg_.candidate_hold_sec);
      return now - t.last_direct > ttl;
    }), tracks_.end());
  for (auto&t:tracks_)
  {
    const double dt = std::max(0., now - t.last);
    Eigen::Matrix4d f = Eigen::Matrix4d::Identity();
    f(0, 2) = f(1, 3) = dt;
    t.state = f * t.state;
    Eigen::Matrix4d q = Eigen::Matrix4d::Zero();
    const double d = std::min(dt, 2.);
    q(0, 0) = q(1, 1) = .1 * d * d + .001;
    q(2, 2) = q(3, 3) = .5 * d + .001;
    t.covariance = f * t.covariance * f.transpose() + q;
    t.last = now;
    ++t.out.age;
  }
  std::vector<std::vector<double> > costs(tracks_.size(), std::vector<double>(r.observations.size(), 1e6));
  for (size_t i = 0; i < tracks_.size(); ++i)
  {
    for (size_t j = 0; j < r.observations.size(); ++j)
    {
      const auto&t = tracks_[i];
      const auto&o = r.observations[j];
      const Eigen::Vector2d delta = o.center.head<2>() - t.state.head<2>();
      const double d = delta.norm(), age = now - t.last_direct;
      const double gate = cfg_.association_gate + std::min(.8, t.state.tail<2>().norm() * age);
      // A smaller measured fragment may lie inside the retained body. It must still
      // overlap that body and agree in width/yaw; proximity alone does not merge IDs.
      const double co = std::cos(t.out.yaw), si = std::sin(t.out.yaw);
      const double along = std::abs(co * delta.x() + si * delta.y()),
                   across = std::abs(-si * delta.x() + co * delta.y());
      const bool partial = (o.size.x() < cfg_.partial_ratio * t.out.size.x() ||
                            o.size.z() < cfg_.partial_ratio * t.out.size.z()) &&
                           o.size.y() > .4 * t.out.size.y() &&
                           along < .5 * t.out.size.x() && across < .5 * t.out.size.y();
      if (d > gate && !partial)
      {
        continue;
      }
      if (angle_distance(o.yaw, t.out.yaw) > .65 && std::min(o.size.x() / std::max(.1, o.size.y()),
                                                             t.out.size.x() / std::max(.1,
                                                                                       t.out.size.y())) > 1.8)
      {
        continue;
      }
      if (t.out.vehicle && !o.vehicle && !partial)
      {
        continue;
      }
      const double shape = std::abs(std::log(std::max(.1, o.size.x()) / std::max(.1,
                                                                                 t.out.size.x()))) + .5 *
                           std::abs(std::log(std::max(.1, o.size.y()) / std::max(.1, t.out.size.y())));
      // Dormant tracks require a conservative actual-position match, not unlimited prediction.
      if (age > cfg_.hold_sec && d > 1.0)
      {
        continue;
      }
      costs[i][j] = (partial ? std::min(d, gate * .7) : d) / gate + .25 * shape;
    }
  }
  const auto matches = assignment(costs, 1.5);
  std::vector<bool> used(r.observations.size());
  for (size_t i = 0; i < tracks_.size(); ++i)
  {
    auto&t = tracks_[i];
    const int index = matches[i];
    if (index < 0)
    {
      ++t.out.lost;
      t.out.observation = "HELD";
      t.out.indices.clear();
      t.out.observed_size.setZero();
      t.out.velocity = t.state.tail<2>();
      t.out.center.head<2>() = t.state.head<2>();
      cx128s2_bev_perception::updateVehicleStatus(t.vehicle_status, t.out.motion, t.out.range_speed, now,
                                                  cfg_.stopped_sec, cfg_.moving_speed, false);
      continue;
    }
    used[index] = true;
    const auto&o = r.observations[index];
    const bool gap = t.out.lost > 0;
    t.out.partial = t.out.extent_locked &&
                    (o.size.x() < cfg_.partial_ratio * t.extent.x() ||
                     o.size.y() < cfg_.partial_ratio * t.extent.y() ||
                     o.size.z() < cfg_.partial_ratio * t.extent.z());
    Eigen::Vector2d measured = o.center.head<2>();
    // The transition to a short visible fragment is not body displacement.
    // Repeated same-shape fragments can, however, supply observed translation.
    bool pose_supported=!t.out.partial;
    if(t.out.partial){
      const double shape_delta=((o.size-t.last_observed_size).array().abs() /
          t.last_observed_size.array().max(.2)).maxCoeff();
      if(!gap && t.last_partial && shape_delta<.15 && angle_distance(o.yaw,t.last_observed_yaw)<.2){
        measured=t.out.center.head<2>()+(o.center-t.last_observed_center).head<2>();
        pose_supported=true;
      }else{
        measured=t.state.head<2>();t.state.tail<2>()*=.8;
      }
    }
    if(gap || t.out.partial!=t.last_partial || !pose_supported){t.motion_samples.clear();t.pending_motion.clear();t.motion_start=-1;}
    const double retained_z=t.out.center.z();
    Eigen::Matrix<double, 2, 4> h = Eigen::Matrix<double, 2, 4>::Zero();
    h(0, 0) = h(1, 1) = 1;
    Eigen::Matrix2d noise = Eigen::Matrix2d::Identity() * (t.out.partial ? (pose_supported?.1:1.0) : .04);
    const Eigen::Matrix2d s = h * t.covariance * h.transpose() + noise;
    const Eigen::Matrix<double, 4,
                        2> k = t.covariance * h.transpose() * s.ldlt().solve(Eigen::Matrix2d::Identity());
    t.state += k * (measured - h * t.state);
    const Eigen::Matrix4d correction = Eigen::Matrix4d::Identity() - k * h;
    t.covariance = correction * t.covariance * correction.transpose() + k * noise * k.transpose();
    if (gap)
    {
      t.evidence_start = now;
      t.extents.clear();
      t.growth_start = -1;
      t.motion_samples.clear();
      t.motion_start = -1;
      t.pending_motion.clear();
    }
    ++t.out.hits;
    t.out.lost = 0;
    t.last_direct = now;
    t.out.last_direct_sec = now;
    t.out.observation = "DIRECT";
    t.out.indices = o.indices;
    t.out.cells = o.cells;
    t.out.observed_size = o.size;
    t.out.center = o.center;
    if(t.out.partial)t.out.center.z()=retained_z;
    t.last_partial=t.out.partial;t.last_observed_center=o.center;t.last_observed_size=o.size;t.last_observed_yaw=o.yaw;
    t.out.center.head<2>() = t.state.head<2>();
    if (!t.out.partial)
    {
      t.out.yaw = o.yaw;
    }
    const bool was_vehicle=t.confirmed_vehicle;
    if (o.vehicle)
    {
      if(t.vehicle_last<0 || gap || now-t.vehicle_last>cfg_.candidate_hold_sec){t.vehicle_since=now;t.vehicle_hits=0;}
      ++t.vehicle_hits;t.vehicle_last=now;
      if(t.vehicle_hits>=cfg_.confirmation_hits && now-t.vehicle_since>=cfg_.confirmation_sec)t.confirmed_vehicle=true;
    }
    else if(!t.confirmed_vehicle){t.vehicle_since=-1;t.vehicle_last=-1;t.vehicle_hits=0;}
    if(!was_vehicle && t.confirmed_vehicle)
    {
      // UNKNOWN geometry has never passed the vehicle constraints. Start the
      // retained vehicle model from independently confirmed current evidence.
      t.out.extent_locked=false;
      t.out.partial=false;
      t.last_partial=false;
      t.extents.clear();
      t.extent.setZero();
      t.growth_start=-1;
      t.evidence_start=now;
      t.out.size=o.size;
      t.out.center.z()=o.center.z();
      t.out.yaw=o.yaw;
    }
    t.out.vehicle = t.confirmed_vehicle;
    if (t.out.hits >= cfg_.confirmation_hits && now - t.evidence_start >= cfg_.confirmation_sec)
    {
      t.out.confirmed = true;
    }
    if (!t.out.partial)
    {
      t.extents.emplace_back(now, o.size);
      while (!t.extents.empty() && now - t.extents.front().first > cfg_.extent_confirmation_sec + .1)
      {
        t.extents.pop_front();
      }
      if (!t.out.extent_locked)
      {
        t.out.size = o.size;
        if (now - t.evidence_start >= cfg_.extent_confirmation_sec &&
            t.extents.size() >= size_t(cfg_.confirmation_hits))
        {
          t.extent.setZero();
          for (const auto&e:t.extents)
          {
            t.extent = t.extent.cwiseMax(e.second);
          }
          t.out.extent_locked = true;
          t.out.size = t.extent;
        }
      }
      else {
        const bool bigger = (o.size.array() > t.extent.array() * 1.02).any();
        if (bigger)
        {
          if (t.growth_start < 0)
          {
            t.growth_start = now;
            t.growth = o.size;
          }
          else{
            t.growth = t.growth.cwiseMin(o.size);
          }
          if (now - t.growth_start >= cfg_.extent_growth_sec)
          {
            t.extent = t.extent.cwiseMax(t.growth);
            t.growth_start = -1;
          }
        }
        else{
          t.growth_start = -1;
        }
      }
    }
    if (t.out.extent_locked)
    {
      t.out.size = t.extent;
    }
    if (pose_supported)
    {
      t.motion_samples.emplace_back(now, t.state.head<2>());
    }
    while (!t.motion_samples.empty() && now - t.motion_samples.front().first > cfg_.motion_window)
    {
      t.motion_samples.pop_front();
    }
    Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
    bool motion_valid = false;
    const double minimum_motion_span=std::min(.4,cfg_.motion_window*.5);
    if (pose_supported && t.motion_samples.size()>=3 &&
        now-t.motion_samples.front().first>=minimum_motion_span)
    {
      double mean_t = 0, den = 0;
      Eigen::Vector2d mean = Eigen::Vector2d::Zero();
      for (auto&m:t.motion_samples)
      {
        mean_t += m.first;
        mean += m.second;
      }
      mean_t /= t.motion_samples.size();
      mean /= double(t.motion_samples.size());
      for (auto&m:t.motion_samples)
      {
        const double dt = m.first - mean_t;
        den += dt * dt;
        velocity += dt * (m.second - mean);
      }
      if (den > 1e-8)
      {
        velocity /= den;
        motion_valid = true;
      }
    }
    if (motion_valid)
    {
      const double speed = velocity.norm();
      std::string desired = t.out.motion;
      if (speed >= cfg_.moving_speed)
      {
        desired = "MOVING";
      }
      else if (speed <= cfg_.stationary_speed)
      {
        desired = "STATIONARY";
      }
      if (desired != t.out.motion)
      {
        if (t.pending_motion != desired)
        {
          t.pending_motion = desired;
          t.motion_start = now;
        }
        else if (now - t.motion_start >= cfg_.motion_confirm_sec)
        {
          t.out.motion = desired;
          t.motion_start = -1;
        }
      }
      else {
        t.pending_motion.clear();
        t.motion_start = -1;
      }
      t.out.velocity = velocity;
      const double rho = t.state.head<2>().norm();
      t.out.range_speed = rho > 1e-6 ? t.state.head<2>().dot(velocity) / rho : 0;
      cx128s2_bev_perception::updateVehicleStatus(t.vehicle_status, t.out.motion, t.out.range_speed, now,
                                                  cfg_.stopped_sec, cfg_.moving_speed, true);
    }
    else{
      cx128s2_bev_perception::updateVehicleStatus(t.vehicle_status, t.out.motion, t.out.range_speed, now,
                                                  cfg_.stopped_sec, cfg_.moving_speed, false);
    }
    t.out.status = t.vehicle_status.value;
  }
  for (size_t j = 0; j < r.observations.size(); ++j)
  {
    if (!used[j] && tracks_.size() < size_t(cfg_.max_objects) * 2)
    {
      const auto&o = r.observations[j];
      Track t;
      t.out.id = next_id_++;
      t.out.age = t.out.hits = 1;
      t.out.confirmed=cfg_.confirmation_hits<=1 && cfg_.confirmation_sec<=0;
    t.out.vehicle = t.confirmed_vehicle = o.vehicle && t.out.confirmed;
    if(o.vehicle){t.vehicle_since=t.vehicle_last=now;t.vehicle_hits=1;}
      t.out.observation = "DIRECT";
      t.out.center = o.center;
      t.out.size = t.out.observed_size = o.size;
      t.out.yaw = o.yaw;
      t.out.indices = o.indices;
      t.out.cells = o.cells;
      t.state.head<2>() = o.center.head<2>();
    t.last_observed_center=o.center;t.last_observed_size=o.size;t.last_observed_yaw=o.yaw;
      t.first = t.last = t.last_direct = t.evidence_start = now;
      t.out.last_direct_sec = now;
      t.extents.emplace_back(now, o.size);
      t.motion_samples.emplace_back(now, o.center.head<2>());
      tracks_.push_back(std::move(t));
    }
  }
  for (const auto&t:tracks_)
  {
    if (now - t.last_direct <= (t.out.confirmed ? cfg_.hold_sec : cfg_.candidate_hold_sec))
    {
      r.objects.push_back(t.out);
    }
  }
  std::sort(r.objects.begin(), r.objects.end(), [](const Object&a, const Object&b){
      return a.id < b.id;
    });
}
}
