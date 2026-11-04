#pragma once

#include "cx128s2_bev_perception/point_types.hpp"
#include <algorithm>
#include <array>
#include <bitset>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace cx128s2_bev_perception {

// Current-frame evidence only. A candidate is excluded from vehicle linkage and
// extent queries, not relabelled as measured ground or used to fit the ground.
struct GeneralGroundNoiseConfig {
  bool enabled = true;
  double cell_m = 0.5;
  double ground_reference_z = 0.0;
  double floor_half_band_m = 0.15;
  int floor_min_points = 20, floor_min_rings = 4;
  double floor_max_spread_m = 0.12;
  double raised_min_height_m = 0.18, raised_max_height_m = 0.8;
  int raised_min_points = 3, raised_max_rings = 4;
  double raised_max_floor_ratio = 0.35, raised_top_two_fraction = 0.75;
  double body_min_height_m = 0.9, body_max_height_m = 5.0;
  int body_min_points = 6, body_min_rings = 3, body_protect_cells = 2;

  void validate() const {
    for (double value : {cell_m, ground_reference_z, floor_half_band_m,
         floor_max_spread_m, raised_min_height_m, raised_max_height_m,
         raised_max_floor_ratio, raised_top_two_fraction,
         body_min_height_m, body_max_height_m}) {
      if (!std::isfinite(value)) throw std::invalid_argument("Nonfinite general_ground_noise parameter");
    }
    if (cell_m < 0.05 || floor_half_band_m <= 0 || floor_max_spread_m <= 0 ||
        floor_min_points < 1 || floor_min_rings < 2 || floor_min_rings > 256 ||
        raised_min_height_m < 0 || raised_max_height_m <= raised_min_height_m ||
        raised_min_points < 1 || raised_max_rings < 1 || raised_max_rings > 8 ||
        raised_max_floor_ratio <= 0 || raised_max_floor_ratio > 1 ||
        raised_top_two_fraction <= 0 || raised_top_two_fraction > 1 ||
        body_min_height_m <= raised_max_height_m || body_max_height_m <= body_min_height_m ||
        body_min_points < 1 || body_min_rings < 2 || body_min_rings > 256 ||
        body_protect_cells < 0 || body_protect_cells > 4) {
      throw std::invalid_argument("general_ground_noise parameter out of range");
    }
  }
};

struct GeneralGroundNoiseStats {
  std::string reason = "DISABLED";
  size_t raw_candidates = 0, removed_foreground = 0;
  size_t supported_floor_cells = 0, protected_cells = 0;
  double elapsed_ms = 0.0;
};

class GeneralGroundNoiseFilter {
  struct Key {
    int x, y;
    bool operator==(const Key & other) const { return x == other.x && y == other.y; }
  };
  struct Hash {
    size_t operator()(const Key & key) const {
      return std::hash<uint64_t>{}((uint64_t(uint32_t(key.x)) << 32) | uint32_t(key.y));
    }
  };
  struct Cell {
    std::vector<float> floor;
    std::bitset<256> floor_rings, body_rings;
    size_t body_points = 0, raised_points = 0;
    std::array<uint16_t, 8> raised_ring{};
    std::array<size_t, 8> raised_count{};
    int distinct_raised = 0;
    double level = 0.0;
    bool floor_valid = false, protected_body = false, candidate = false;
  };
  std::unordered_map<Key, Cell, Hash> cells_;
  std::vector<Cell *> point_cells_;
  std::vector<uint8_t> candidate_;

  static double quantile(const std::vector<float> & sorted, double q) {
    const double position = q * double(sorted.size() - 1);
    const size_t low = size_t(position), high = std::min(low + 1, sorted.size() - 1);
    return double(sorted[low]) + (position - low) * (double(sorted[high]) - sorted[low]);
  }

public:
  const std::vector<uint8_t> & candidates() const { return candidate_; }

  // evidence_allowed applies spatial/range limits, but intentionally not the
  // detector's valid_z_min: discarded floor returns still corroborate the floor.
  template<class EvidenceAllowed>
  GeneralGroundNoiseStats filter(const std::vector<PointXYZIRT> & points,
      std::vector<int> & foreground, std::vector<int> & removed,
      const GeneralGroundNoiseConfig & config, bool has_ring,
      EvidenceAllowed evidence_allowed) {
    GeneralGroundNoiseStats result;
    candidate_.assign(points.size(), 0);
    removed.clear();
    if (!config.enabled) return result;
    if (!has_ring) { result.reason = "MISSING_RING_PASSTHROUGH"; return result; }
    config.validate();
    cells_.clear();
    cells_.reserve(points.size() / 16 + 1);
    point_cells_.assign(points.size(), nullptr);
    // A malformed/unsupported beam layout must never become a low-ring layer.
    for (const auto & point : points) {
      if (point.ring >= 256) { result.reason = "UNSUPPORTED_RING_PASSTHROUGH"; return result; }
    }
    for (size_t i = 0; i < points.size(); ++i) {
      const auto & p = points[i];
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
          !evidence_allowed(p)) continue;
      const double ix = std::floor(double(p.x) / config.cell_m);
      const double iy = std::floor(double(p.y) / config.cell_m);
      if (std::abs(ix) > std::numeric_limits<int>::max() / 2 ||
          std::abs(iy) > std::numeric_limits<int>::max() / 2) continue;
      auto & cell = cells_[{int(ix), int(iy)}];
      point_cells_[i] = &cell;
      const double h = double(p.z) - config.ground_reference_z;
      if (std::abs(h) < config.floor_half_band_m) {
        cell.floor.push_back(p.z); cell.floor_rings.set(p.ring);
      }
      if (h > config.body_min_height_m && h < config.body_max_height_m) {
        ++cell.body_points; cell.body_rings.set(p.ring);
      }
    }
    for (auto & entry : cells_) {
      auto & cell = entry.second;
      const auto & key = entry.first;
      for (int dx = -config.body_protect_cells; dx <= config.body_protect_cells && !cell.protected_body; ++dx) {
        for (int dy = -config.body_protect_cells; dy <= config.body_protect_cells; ++dy) {
          const auto found = cells_.find({key.x + dx, key.y + dy});
          if (found != cells_.end() && found->second.body_points >= size_t(config.body_min_points) &&
              found->second.body_rings.count() >= size_t(config.body_min_rings)) {
            cell.protected_body = true; break;
          }
        }
      }
      if (cell.protected_body) { ++result.protected_cells; continue; }
      if (cell.floor.size() < size_t(config.floor_min_points) ||
          cell.floor_rings.count() < size_t(config.floor_min_rings)) continue;
      std::sort(cell.floor.begin(), cell.floor.end());
      if (quantile(cell.floor, .9) - quantile(cell.floor, .1) > config.floor_max_spread_m) continue;
      cell.level = quantile(cell.floor, .5);
      cell.floor_valid = true; ++result.supported_floor_cells;
    }
    for (size_t i = 0; i < points.size(); ++i) {
      auto * cell = point_cells_[i];
      if (!cell || !cell->floor_valid) continue;
      const auto & p = points[i]; const double h = double(p.z) - cell->level;
      if (h < config.raised_min_height_m || h > config.raised_max_height_m) continue;
      ++cell->raised_points;
      int j = 0;
      while (j < std::min(cell->distinct_raised, 8) && cell->raised_ring[j] != p.ring) ++j;
      if (j < 8) {
        if (j == cell->distinct_raised) { cell->raised_ring[j] = p.ring; ++cell->distinct_raised; }
        ++cell->raised_count[j];
      } else cell->distinct_raised = 9;
    }
    for (auto & entry : cells_) {
      auto & cell = entry.second;
      if (!cell.floor_valid || cell.raised_points < size_t(config.raised_min_points) ||
          cell.distinct_raised > config.raised_max_rings ||
          double(cell.raised_points) / cell.floor.size() > config.raised_max_floor_ratio) continue;
      size_t first = 0, second = 0;
      for (size_t count : cell.raised_count) {
        if (count > first) { second = first; first = count; }
        else if (count > second) second = count;
      }
      cell.candidate = double(first + second) / cell.raised_points >= config.raised_top_two_fraction;
    }
    for (size_t i = 0; i < points.size(); ++i) {
      const auto * cell = point_cells_[i];
      if (cell && cell->candidate) {
        const double h = double(points[i].z) - cell->level;
        if (h >= config.raised_min_height_m && h <= config.raised_max_height_m) {
          candidate_[i] = 1; ++result.raw_candidates;
        }
      }
    }
    foreground.erase(std::remove_if(foreground.begin(), foreground.end(), [&](int i) {
      if (i >= 0 && size_t(i) < candidate_.size() && candidate_[i]) {
        removed.push_back(i); return true;
      }
      return false;
    }), foreground.end());
    result.removed_foreground = removed.size();
    result.reason = "CURRENT_FRAME_FLOOR_AND_BODY_SUPPORT";
    return result;
  }
};
}  // namespace cx128s2_bev_perception
