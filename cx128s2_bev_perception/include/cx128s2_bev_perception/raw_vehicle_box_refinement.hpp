#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>
#include <cx128s2_bev_perception/point_types.hpp>

namespace cx128s2_bev_perception::raw_box {
struct Query {
  int cluster_id = -1;
  double x = 0.0, y = 0.0;
  double cos_yaw = 1.0, sin_yaw = 0.0;
  double half_length = 0.0, half_width = 0.0;
  double min_z = 0.0, max_z = 0.0;
};
struct Observation {
  int points = 0, foreground_points = 0;
  double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
  double min_x = std::numeric_limits<double>::max(), max_x = std::numeric_limits<double>::lowest();
  double min_y = std::numeric_limits<double>::max(), max_y = std::numeric_limits<double>::lowest();
  double min_z = std::numeric_limits<double>::max(), max_z = std::numeric_limits<double>::lowest();
  double min_local_x = std::numeric_limits<double>::max(), max_local_x = std::numeric_limits<double>::lowest();
  double min_local_y = std::numeric_limits<double>::max(), max_local_y = std::numeric_limits<double>::lowest();
};
struct Statistics { size_t scanned_points = 0, boxed_points = 0, box_tests = 0; };
struct Workspace {
  std::vector<uint64_t> x_masks;
  std::vector<Observation> observations;
};

// One source-cloud scan. X buckets hold box bitmasks, rather than point lists:
// there is no new point hash/grid, point-cloud copy, PCA or clustering pass.
// Original foreground ownership wins; unowned points belong to at most one box.
template<class AcceptPoint>
Statistics collect(const std::vector<PointXYZIRT> & points,
                   const std::vector<int> & foreground_owner,
                   const std::vector<Query> & queries,
                   AcceptPoint accept_point, Workspace & workspace,
                   const std::vector<int> * source_indices = nullptr) {
  Statistics stats;
  workspace.observations.assign(queries.size(), Observation{});
  if (queries.empty()) return stats;
  if (queries.size() > 64) throw std::invalid_argument("raw box query cap is 64");
  if (foreground_owner.size() != points.size())
    throw std::invalid_argument("raw box foreground ownership must match source cloud");
  double min_x = 1e30, max_x = -1e30, min_y = 1e30, max_y = -1e30, min_z = 1e30, max_z = -1e30;
  std::vector<double> x_min(queries.size()), x_max(queries.size());
  for (size_t i = 0; i < queries.size(); ++i) {
    const auto & q = queries[i];
    if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.half_length) ||
        !std::isfinite(q.half_width) || q.half_length <= 0.0 || q.half_width <= 0.0 ||
        !std::isfinite(q.cos_yaw) || !std::isfinite(q.sin_yaw) ||
        !std::isfinite(q.min_z) || !std::isfinite(q.max_z) || q.min_z > q.max_z)
      throw std::invalid_argument("invalid raw vehicle box");
    const double hx = std::abs(q.cos_yaw) * q.half_length + std::abs(q.sin_yaw) * q.half_width;
    const double hy = std::abs(q.sin_yaw) * q.half_length + std::abs(q.cos_yaw) * q.half_width;
    x_min[i] = q.x - hx; x_max[i] = q.x + hx;
    min_x = std::min(min_x, x_min[i]); max_x = std::max(max_x, x_max[i]);
    min_y = std::min(min_y, q.y - hy); max_y = std::max(max_y, q.y + hy);
    min_z = std::min(min_z, q.min_z); max_z = std::max(max_z, q.max_z);
  }
  // Bound scratch memory even when boxes are far apart in world coordinates.
  const double bin_width = std::max(2.0, (max_x - min_x) / 1023.0);
  const double inverse_bin_width = 1.0 / bin_width;
  const size_t bins = static_cast<size_t>((max_x - min_x) * inverse_bin_width) + 1;
  workspace.x_masks.assign(bins, 0);
  for (size_t i = 0; i < queries.size(); ++i) {
    const size_t first = static_cast<size_t>((x_min[i] - min_x) * inverse_bin_width);
    const size_t last = std::min(bins - 1, static_cast<size_t>((x_max[i] - min_x) * inverse_bin_width));
    for (size_t bin = first; bin <= last; ++bin) workspace.x_masks[bin] |= uint64_t{1} << i;
  }
  stats.scanned_points = source_indices ? source_indices->size() : points.size();
  for (size_t cursor = 0; cursor < stats.scanned_points; ++cursor) {
    const size_t pi = source_indices ? static_cast<size_t>((*source_indices)[cursor]) : cursor;
    if (pi >= points.size()) continue;
    const auto & p = points[pi];
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
        p.x < min_x || p.x > max_x || p.y < min_y || p.y > max_y || p.z < min_z || p.z > max_z) continue;
    const size_t bin = std::min(bins - 1, static_cast<size_t>((p.x - min_x) * inverse_bin_width));
    uint64_t mask = workspace.x_masks[bin];
    int best = -1;
    double best_score = std::numeric_limits<double>::max(), best_lx = 0.0, best_ly = 0.0;
    while (mask) {
      const int i = __builtin_ctzll(mask); mask &= mask - 1;
      const auto & q = queries[i];
      if (foreground_owner[pi] >= 0 && foreground_owner[pi] != q.cluster_id) continue;
      ++stats.box_tests;
      if (p.z < q.min_z || p.z > q.max_z) continue;
      const double dx = p.x - q.x, dy = p.y - q.y;
      const double lx = q.cos_yaw * dx + q.sin_yaw * dy;
      const double ly = -q.sin_yaw * dx + q.cos_yaw * dy;
      if (std::abs(lx) > q.half_length || std::abs(ly) > q.half_width) continue;
      const double score = (lx * lx) / (q.half_length * q.half_length) +
          (ly * ly) / (q.half_width * q.half_width);
      if (score < best_score) { best = i; best_score = score; best_lx = lx; best_ly = ly; }
    }
    if (best < 0 || !accept_point(p)) continue;
    auto & o = workspace.observations[best];
    ++o.points; ++stats.boxed_points;
    if (foreground_owner[pi] == queries[best].cluster_id) ++o.foreground_points;
    o.sum_x += p.x; o.sum_y += p.y; o.sum_z += p.z;
    o.min_x = std::min(o.min_x, double(p.x)); o.max_x = std::max(o.max_x, double(p.x));
    o.min_y = std::min(o.min_y, double(p.y)); o.max_y = std::max(o.max_y, double(p.y));
    o.min_z = std::min(o.min_z, double(p.z)); o.max_z = std::max(o.max_z, double(p.z));
    o.min_local_x = std::min(o.min_local_x, best_lx); o.max_local_x = std::max(o.max_local_x, best_lx);
    o.min_local_y = std::min(o.min_local_y, best_ly); o.max_local_y = std::max(o.max_local_y, best_ly);
  }
  return stats;
}
}  // namespace cx128s2_bev_perception::raw_box
