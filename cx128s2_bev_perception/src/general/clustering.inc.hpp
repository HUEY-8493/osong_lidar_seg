class IClusterer {
public:
  virtual ~IClusterer() = default;
  void setSensorOrigin(double x, double y) { sensor_x_ = x; sensor_y_ = y; }
  double sensorRange(double x, double y) const { return std::hypot(x - sensor_x_, y - sensor_y_); }
private:
  double sensor_x_ = 0.0, sensor_y_ = 0.0;
public:

  virtual std::vector<ClusterResult> cluster(
      const std::vector<PointXYZIRT> & points,
      const std::vector<int> & foreground_indices) = 0;

  virtual std::vector<ClusterResult> cluster(
      const std::vector<PointXYZIRT> & points,
      const std::vector<int> & foreground_indices,
      uint64_t /*frame_index*/, double /*timestamp*/) {
    return cluster(points, foreground_indices);
  }

  virtual std::string name() const = 0;
  virtual ClusterDiagnostics diagnostics() const { return {}; }

  virtual const std::vector<ShapeNode> & debugShapeNodes() const {
    static const std::vector<ShapeNode> empty;
    return empty;
  }

  virtual const std::vector<ShapeEdge> & debugShapeEdges() const {
    static const std::vector<ShapeEdge> empty;
    return empty;
  }
};

void fillClusterStats(const std::vector<PointXYZIRT> & points, ClusterResult & out) {
  out.point_count = static_cast<int>(out.indices.size());
  out.cell_count = static_cast<int>(out.cell_keys.size());
  if (out.indices.empty()) return;
  out.min_x = out.min_y = std::numeric_limits<double>::max();
  out.max_x = out.max_y = std::numeric_limits<double>::lowest();
  for (int idx : out.indices) {
    const auto & p = points[idx];
    out.centroid_x += p.x;
    out.centroid_y += p.y;
    out.min_x = std::min(out.min_x, static_cast<double>(p.x));
    out.max_x = std::max(out.max_x, static_cast<double>(p.x));
    out.min_y = std::min(out.min_y, static_cast<double>(p.y));
    out.max_y = std::max(out.max_y, static_cast<double>(p.y));
  }
  const double n = std::max(1, out.point_count);
  out.centroid_x /= n;
  out.centroid_y /= n;
  out.length = std::max(0.05, out.max_x - out.min_x);
  out.width = std::max(0.05, out.max_y - out.min_y);
  if (out.width > out.length) std::swap(out.width, out.length);
}

class BevConnectedComponentClusterer : public IClusterer {
  friend class CargoExtensionBevScratch;
public:
  BevConnectedComponentClusterer(
      double bev_x_res, double bev_y_res, int cluster_connectivity,
      int min_points_per_occupied_cell,
      int min_cluster_cells_near, int min_cluster_cells_mid, int min_cluster_cells_long,
      int min_cluster_points_near, int min_cluster_points_mid, int min_cluster_points_long,
      double near_range_max, double mid_range_max, bool z_guard, bool gap_bridge,
      double neighbor_z_gap, double bridge_z_gap, int gap_near, int gap_mid, int gap_long,
      bool z_layer_enabled, double z_layer_resolution, bool weak_support_enabled,
      bool density_enabled, int density_window_cells, int weak_support_min_density,
      bool z_profile_diagnostics_enabled, double z_profile_bin_size, double z_profile_min_z = -0.5)
  : bev_x_res_(bev_x_res),
    bev_y_res_(bev_y_res),
    cluster_connectivity_(cluster_connectivity),
    min_points_per_occupied_cell_(min_points_per_occupied_cell),
    min_cluster_cells_near_(min_cluster_cells_near),
    min_cluster_cells_mid_(min_cluster_cells_mid),
    min_cluster_cells_long_(min_cluster_cells_long),
    min_cluster_points_near_(min_cluster_points_near),
    min_cluster_points_mid_(min_cluster_points_mid),
    min_cluster_points_long_(min_cluster_points_long),
    near_range_max_(near_range_max),
    mid_range_max_(mid_range_max), z_guard_(z_guard), gap_bridge_(gap_bridge),
    neighbor_z_gap_(neighbor_z_gap), bridge_z_gap_(bridge_z_gap), gap_near_(gap_near),
    gap_mid_(gap_mid), gap_long_(gap_long), z_layer_enabled_(z_layer_enabled),
    weak_support_enabled_(weak_support_enabled), density_enabled_(density_enabled),
    z_layer_resolution_(z_layer_resolution), density_window_cells_(density_window_cells),
    weak_support_min_density_(weak_support_min_density),
    z_profile_diagnostics_enabled_(z_profile_diagnostics_enabled), z_profile_bin_size_(z_profile_bin_size),
    z_profile_min_z_(z_profile_min_z) {}

  // A bounded capacity hint for a downstream copy; upstream uses zero.
  void reserveLayerPoints(int capacity) { initial_layer_point_capacity_=std::max(0,capacity); }

  std::vector<ClusterResult> cluster(
      const std::vector<PointXYZIRT> & points,
      const std::vector<int> & foreground_indices) override {
    return cluster(points, foreground_indices, 0, 0.0);
  }

  std::vector<ClusterResult> cluster(
      const std::vector<PointXYZIRT> & points,
      const std::vector<int> & foreground_indices,
      uint64_t frame_index, double timestamp) override {
    if (z_layer_enabled_) return clusterLayered(points, foreground_indices);
    diagnostics_ = {};
    const auto bev_start = std::chrono::steady_clock::now();
    struct BevCell {
      std::vector<int> indices;
      float min_z = std::numeric_limits<float>::max();
      float max_z = std::numeric_limits<float>::lowest();
      uint64_t z_mask = 0;
    };
    std::unordered_map<int64_t, BevCell> cells;
    cells.reserve(foreground_indices.size() / 2 + 1);
    const double inv_bin_size = 1.0 / std::max(0.05, z_profile_bin_size_);
    const auto z_build_start = std::chrono::steady_clock::now();
    for (int idx : foreground_indices) {
      const auto & p = points[idx];
      auto & cell = cells[bevKey(p)];
      cell.indices.push_back(idx);
      cell.min_z = std::min(cell.min_z, p.z);
      cell.max_z = std::max(cell.max_z, p.z);
      if (z_profile_diagnostics_enabled_) {
        const int bin = static_cast<int>(std::floor((p.z - z_profile_min_z_) * inv_bin_size));
        if (bin >= 0 && bin < 64) {
          cell.z_mask |= (uint64_t{1} << bin);
        }
      }
    }
    if (z_profile_diagnostics_enabled_) {
      diagnostics_.z_profile_build_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - z_build_start).count();
    }
    diagnostics_.bev_build_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - bev_start).count();
    diagnostics_.occupied_cell_count = static_cast<int>(cells.size());

    const auto recordEdgeDiagnostic = [this, &cells, frame_index, timestamp](
        int ix_a, int iy_a, const BevCell & a,
        int ix_b, int iy_b, const BevCell & b,
        int edge_type, int dx, int dy, bool baseline_decision) {
      const auto diag_start = std::chrono::steady_clock::now();

      const int ca = __builtin_popcountll(a.z_mask);
      const int cb = __builtin_popcountll(b.z_mask);
      const int min_c = std::min(ca, cb);
      const int shared_exact = __builtin_popcountll(a.z_mask & b.z_mask);
      const int union_exact = ca + cb - shared_exact;
      const double exact_r_min = min_c > 0 ? static_cast<double>(shared_exact) / min_c : 0.0;
      const double exact_jaccard = union_exact > 0 ? static_cast<double>(shared_exact) / union_exact : 0.0;

      const uint64_t expanded_a = a.z_mask | (a.z_mask >> 1) | (a.z_mask << 1);
      const uint64_t expanded_b = b.z_mask | (b.z_mask >> 1) | (b.z_mask << 1);
      const int a_supp = __builtin_popcountll(a.z_mask & expanded_b);
      const int b_supp = __builtin_popcountll(b.z_mask & expanded_a);
      const int shared_tol = std::min(a_supp, b_supp);
      const int union_tol = ca + cb - shared_tol;
      const double tol_r_min = min_c > 0 ? static_cast<double>(shared_tol) / min_c : 0.0;
      const double tol_jaccard = union_tol > 0 ? static_cast<double>(shared_tol) / union_tol : 0.0;

      const float g_z = std::max(0.0f, std::max(a.min_z, b.min_z) - std::min(a.max_z, b.max_z));

      int directional_support = 0;
      if (edge_type == 1) {
        directional_support = (cells.count(hash2d(ix_a + dx, iy_a)) ? 1 : 0) +
                              (cells.count(hash2d(ix_a, iy_a + dy)) ? 1 : 0);
      } else if (edge_type == 2) {
        const int axis_behind = cells.count(hash2d(ix_a - dx, iy_a - dy)) ? 1 : 0;
        const int axis_ahead = cells.count(hash2d(ix_b + dx, iy_b + dy)) ? 1 : 0;
        directional_support = axis_behind + axis_ahead;
      }

      const int r_min_bin = std::clamp(static_cast<int>(exact_r_min * 10.0), 0, 9);
      const int tol_r_min_bin = std::clamp(static_cast<int>(tol_r_min * 10.0), 0, 9);
      const int jaccard_bin = std::clamp(static_cast<int>(exact_jaccard * 10.0), 0, 9);
      const int tol_jaccard_bin = std::clamp(static_cast<int>(tol_jaccard * 10.0), 0, 9);

      int z_gap_bin = 0;
      if (g_z <= 0.0f) z_gap_bin = 0;
      else if (g_z <= 0.1f) z_gap_bin = 1;
      else if (g_z <= 0.2f) z_gap_bin = 2;
      else if (g_z <= 0.4f) z_gap_bin = 3;
      else if (g_z <= 0.8f) z_gap_bin = 4;
      else if (g_z <= 1.2f) z_gap_bin = 5;
      else z_gap_bin = 6;

      if (edge_type == 0) {
        ++diagnostics_.z_profile_cardinal_edges;
        if (baseline_decision) {
          ++diagnostics_.z_profile_cardinal_accepted;
          ++diagnostics_.z_profile_cardinal_accepted_tol_r_min_hist[tol_r_min_bin];
        } else {
          ++diagnostics_.z_profile_cardinal_rejected;
          ++diagnostics_.z_profile_cardinal_rejected_tol_r_min_hist[tol_r_min_bin];
        }
        ++diagnostics_.z_profile_cardinal_exact_r_min_hist[r_min_bin];
        ++diagnostics_.z_profile_cardinal_tolerant_r_min_hist[tol_r_min_bin];
        ++diagnostics_.z_profile_cardinal_exact_jaccard_hist[jaccard_bin];
        ++diagnostics_.z_profile_cardinal_tolerant_jaccard_hist[tol_jaccard_bin];
        ++diagnostics_.z_profile_cardinal_z_gap_hist[z_gap_bin];
        diagnostics_.z_profile_cardinal_exact_r_min_sum += exact_r_min;
        diagnostics_.z_profile_cardinal_tolerant_r_min_sum += tol_r_min;
        diagnostics_.z_profile_cardinal_exact_jaccard_sum += exact_jaccard;
        diagnostics_.z_profile_cardinal_tolerant_jaccard_sum += tol_jaccard;
        diagnostics_.z_profile_cardinal_z_gap_sum += g_z;
      } else if (edge_type == 1) {
        ++diagnostics_.z_profile_diagonal_edges;
        if (baseline_decision) {
          ++diagnostics_.z_profile_diagonal_accepted;
          ++diagnostics_.z_profile_diagonal_accepted_tol_r_min_hist[tol_r_min_bin];
        } else {
          ++diagnostics_.z_profile_diagonal_rejected;
          ++diagnostics_.z_profile_diagonal_rejected_tol_r_min_hist[tol_r_min_bin];
        }
        ++diagnostics_.z_profile_diagonal_exact_r_min_hist[r_min_bin];
        ++diagnostics_.z_profile_diagonal_tolerant_r_min_hist[tol_r_min_bin];
        ++diagnostics_.z_profile_diagonal_exact_jaccard_hist[jaccard_bin];
        ++diagnostics_.z_profile_diagonal_tolerant_jaccard_hist[tol_jaccard_bin];
        ++diagnostics_.z_profile_diagonal_z_gap_hist[z_gap_bin];
        if (directional_support >= 0 && directional_support <= 2) {
          ++diagnostics_.z_profile_diagonal_support_hist[directional_support];
        }
        diagnostics_.z_profile_diagonal_exact_r_min_sum += exact_r_min;
        diagnostics_.z_profile_diagonal_tolerant_r_min_sum += tol_r_min;
        diagnostics_.z_profile_diagonal_exact_jaccard_sum += exact_jaccard;
        diagnostics_.z_profile_diagonal_tolerant_jaccard_sum += tol_jaccard;
        diagnostics_.z_profile_diagonal_z_gap_sum += g_z;
      } else {
        ++diagnostics_.z_profile_bridge_edges;
        if (baseline_decision) {
          ++diagnostics_.z_profile_bridge_accepted;
          ++diagnostics_.z_profile_bridge_accepted_tol_r_min_hist[tol_r_min_bin];
        } else {
          ++diagnostics_.z_profile_bridge_rejected;
          ++diagnostics_.z_profile_bridge_rejected_tol_r_min_hist[tol_r_min_bin];
        }
        ++diagnostics_.z_profile_bridge_exact_r_min_hist[r_min_bin];
        ++diagnostics_.z_profile_bridge_tolerant_r_min_hist[tol_r_min_bin];
        ++diagnostics_.z_profile_bridge_exact_jaccard_hist[jaccard_bin];
        ++diagnostics_.z_profile_bridge_tolerant_jaccard_hist[tol_jaccard_bin];
        ++diagnostics_.z_profile_bridge_z_gap_hist[z_gap_bin];
        if (directional_support >= 0 && directional_support <= 2) {
          ++diagnostics_.z_profile_bridge_axis_support_hist[directional_support];
        }
        diagnostics_.z_profile_bridge_exact_r_min_sum += exact_r_min;
        diagnostics_.z_profile_bridge_tolerant_r_min_sum += tol_r_min;
        diagnostics_.z_profile_bridge_exact_jaccard_sum += exact_jaccard;
        diagnostics_.z_profile_bridge_tolerant_jaccard_sum += tol_jaccard;
        diagnostics_.z_profile_bridge_z_gap_sum += g_z;
      }

      if (diagnostics_.edge_samples.size() < 100) {
        EdgeDiagnosticSample s;
        s.frame_index = frame_index;
        s.timestamp = timestamp;
        s.edge_type = edge_type;
        s.ix_a = ix_a; s.iy_a = iy_a;
        s.ix_b = ix_b; s.iy_b = iy_b;
        s.points_a = static_cast<int>(a.indices.size());
        s.points_b = static_cast<int>(b.indices.size());
        s.min_z_a = a.min_z; s.max_z_a = a.max_z;
        s.min_z_b = b.min_z; s.max_z_b = b.max_z;
        s.interval_z_gap = g_z;
        s.popcount_a = ca; s.popcount_b = cb;
        s.exact_r_min = static_cast<float>(exact_r_min);
        s.tolerant_r_min = static_cast<float>(tol_r_min);
        s.exact_jaccard = static_cast<float>(exact_jaccard);
        s.tolerant_jaccard = static_cast<float>(tol_jaccard);
        s.directional_support = directional_support;
        s.baseline_decision = baseline_decision;
        diagnostics_.edge_samples.push_back(s);
      }

      diagnostics_.z_profile_edge_diag_ms += std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - diag_start).count();
    };

    std::unordered_set<int64_t> visited;
    visited.reserve(cells.size());
    std::vector<ClusterResult> clusters;
    std::vector<int> raw_component_cells;
    std::vector<int> raw_component_points;
    raw_component_cells.reserve(cells.size());
    raw_component_points.reserve(cells.size());
    const auto addHistogram = [](int value, std::array<int, 6> & histogram) {
      const int bin = value <= 1 ? 0 : value == 2 ? 1 : value <= 5 ? 2 :
          value <= 10 ? 3 : value <= 30 ? 4 : 5;
      ++histogram[bin];
    };
    int cid = 0;
    for (const auto & kv : cells) {
      if (visited.count(kv.first)) continue;
      std::vector<int64_t> q{kv.first};
      visited.insert(kv.first);
      ClusterResult result;
      result.id = cid;
      result.method = name();
      const auto bfs_start = std::chrono::steady_clock::now();
      double component_gather_ms = 0.0;
      for (size_t head = 0; head < q.size(); ++head) {
        const int64_t key = q[head];
        const auto cit = cells.find(key);
        if (cit == cells.end()) continue;
        const auto gather_start = std::chrono::steady_clock::now();
        result.indices.insert(result.indices.end(), cit->second.indices.begin(), cit->second.indices.end());
        component_gather_ms += std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - gather_start).count();
        result.cell_keys.push_back(key);
        const int ix = unpackX(key);
        const int iy = unpackY(key);
        for (int dx = -1; dx <= 1; ++dx) {
          for (int dy = -1; dy <= 1; ++dy) {
            if (cluster_connectivity_ == 4 && std::abs(dx) + std::abs(dy) > 1) continue;
            if (dx == 0 && dy == 0) continue;
            const int64_t nk = hash2d(ix + dx, iy + dy);
            const auto nit = cells.find(nk);
            if (nit != cells.end()) {
              if (!visited.count(nk)) {
                const bool baseline_connect = (!z_guard_ || zCompatible(cit->second, nit->second, neighbor_z_gap_));
                if (z_profile_diagnostics_enabled_) {
                  const int edge_type = (std::abs(dx) + std::abs(dy) == 1) ? 0 : 1;
                  recordEdgeDiagnostic(ix, iy, cit->second, ix + dx, iy + dy, nit->second,
                                       edge_type, dx, dy, baseline_connect);
                }
                if (baseline_connect) {
                  visited.insert(nk);
                  q.push_back(nk);
                } else {
                  diagnostics_.z_rejected_direct_edges++;
                }
              }
            } else if (gap_bridge_ && std::abs(dx) + std::abs(dy) == 1 && nit == cells.end() &&
                       gapAllowed(ix, iy, dx, dy)) {
              diagnostics_.bridge_candidates++;
              const int64_t bk = hash2d(ix + 2 * dx, iy + 2 * dy);
              const auto bit = cells.find(bk);
              if (bit != cells.end()) {
                if (!visited.count(bk)) {
                  const bool baseline_connect = (!z_guard_ || zCompatible(cit->second, bit->second, bridge_z_gap_));
                  if (z_profile_diagnostics_enabled_) {
                    recordEdgeDiagnostic(ix, iy, cit->second, ix + 2 * dx, iy + 2 * dy, bit->second,
                                         2, dx, dy, baseline_connect);
                  }
                  if (baseline_connect) {
                    visited.insert(bk);
                    q.push_back(bk);
                    diagnostics_.bridge_accepted++;
                  } else {
                    diagnostics_.z_rejected_bridge_edges++;
                  }
                }
              }
            }
          }
        }
      }
      diagnostics_.cc_bfs_ms += std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - bfs_start).count() - component_gather_ms;
      diagnostics_.point_gather_ms += component_gather_ms;
      diagnostics_.connected_component_count++;
      diagnostics_.largest_component_cells = std::max(
          diagnostics_.largest_component_cells, static_cast<int>(result.cell_keys.size()));
      diagnostics_.largest_component_points = std::max(
          diagnostics_.largest_component_points, static_cast<int>(result.indices.size()));
      const int component_cells = static_cast<int>(result.cell_keys.size());
      const int component_points = static_cast<int>(result.indices.size());
      raw_component_cells.push_back(component_cells);
      raw_component_points.push_back(component_points);
      addHistogram(component_cells, diagnostics_.component_cell_histogram);
      addHistogram(component_points, diagnostics_.component_point_histogram);
      if (component_cells <= 2) {
        ++diagnostics_.tiny_component_count;
        if (component_cells == 1) ++diagnostics_.single_cell_component_count;
        if (component_cells == 2) ++diagnostics_.two_cell_component_count;
      }
      const auto filter_start = std::chrono::steady_clock::now();
      const bool reject = static_cast<int>(result.indices.size()) < minClusterPoints(points, result.indices) ||
                          static_cast<int>(result.cell_keys.size()) < minClusterCells(points, result.indices);
      diagnostics_.filter_ms += std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - filter_start).count();
      if (reject) continue;
      fillClusterStats(points, result);
      result.id = cid++;
      clusters.push_back(result);
    }
    diagnostics_.valid_cluster_count = static_cast<int>(clusters.size());
    const auto median = [](std::vector<int> & values) {
      if (values.empty()) return 0;
      const auto middle = values.begin() + values.size() / 2;
      std::nth_element(values.begin(), middle, values.end());
      return *middle;
    };
    diagnostics_.median_component_cells = median(raw_component_cells);
    diagnostics_.median_component_points = median(raw_component_points);
    return clusters;
  }

  std::string name() const override {
    return "bev_connected_component";
  }

  ClusterDiagnostics diagnostics() const override { return diagnostics_; }

  // Same measured 3-D connectivity, including weak returns for an exhaustive
  // partition only. Normal cluster support thresholds are unchanged.
  std::vector<ClusterResult> observationLayerComponents(
      const std::vector<PointXYZIRT> & points,const std::vector<int> & indices) {
    const auto saved=diagnostics_;
    auto parts=clusterLayered(points,indices,true);
    diagnostics_=saved;
    return parts;
  }

  bool connectedHeightSupported(const std::vector<PointXYZIRT> & points,
                                const std::vector<int> & indices,
                                uint64_t height_layers, double min_height) {
    const double z_res = std::max(0.05, z_layer_resolution_);
    int previous = -1;
    bool separated = false;
    for (int bin = 0; bin < 64; ++bin) {
      if (!(height_layers & (uint64_t{1} << bin))) continue;
      if (previous >= 0 && (bin - previous - 1) * z_res > neighbor_z_gap_)
        separated = true;
      previous = bin;
    }
    if (!separated) return true;
    // Reuse the existing 3-D connectivity and range support filters only for
    // ambiguous height evidence. Public BEV clusters and IDs stay unchanged.
    const auto saved = diagnostics_;
    const auto parts = clusterLayered(points, indices);
    diagnostics_ = saved;
    for (const auto & part : parts) {
      double low = std::numeric_limits<double>::max();
      double high = std::numeric_limits<double>::lowest();
      for (int idx : part.indices) {
        low = std::min(low, static_cast<double>(points[idx].z));
        high = std::max(high, static_cast<double>(points[idx].z));
      }
      if (high - low >= min_height) return true;
    }
    return false;
  }

private:
  struct LayerCell {
    std::vector<int> indices;
    int ix = 0, iy = 0, iz = 0;
    bool visited = false;
    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::lowest();
  };

  std::vector<ClusterResult> clusterLayered(
      const std::vector<PointXYZIRT> & points,
      const std::vector<int> & foreground_indices, bool include_weak = false) {
    diagnostics_ = {};
    const auto layer_build_start = std::chrono::steady_clock::now();
    std::unordered_map<int64_t, LayerCell> layers;
    std::unordered_map<int64_t, int> layers_per_xy;
    layers.reserve(foreground_indices.size() / 2 + 1);
    const double z_res = std::max(0.05, z_layer_resolution_);
    for (int idx : foreground_indices) {
      const auto & p = points[idx];
      const int ix = static_cast<int>(std::floor(p.x / std::max(0.05, bev_x_res_)));
      const int iy = static_cast<int>(std::floor(p.y / std::max(0.05, bev_y_res_)));
      const int iz = static_cast<int>(std::floor(p.z / z_res));
      const int64_t key = hash3d(ix, iy, iz);
      auto [it, inserted] = layers.try_emplace(key);
      auto & cell = it->second;
      if (inserted) {
        if(initial_layer_point_capacity_>0)cell.indices.reserve(initial_layer_point_capacity_);
        cell.ix = ix; cell.iy = iy; cell.iz = iz;
        ++layers_per_xy[hash2d(ix, iy)];
      }
      cell.indices.push_back(idx);
      cell.min_z = std::min(cell.min_z, p.z);
      cell.max_z = std::max(cell.max_z, p.z);
    }
    diagnostics_.z_layer_build_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - layer_build_start).count();
    diagnostics_.occupied_cell_count = static_cast<int>(layers_per_xy.size());
    diagnostics_.layer_cell_count = static_cast<int>(layers.size());
    for (const auto & kv : layers_per_xy) {
      diagnostics_.max_layers_per_xy_cell = std::max(diagnostics_.max_layers_per_xy_cell, kv.second);
      diagnostics_.average_layers_per_xy_cell += kv.second;
    }
    if (!layers_per_xy.empty()) diagnostics_.average_layers_per_xy_cell /= layers_per_xy.size();
    const auto strong = [this, include_weak](const LayerCell & cell) {
      return include_weak || static_cast<int>(cell.indices.size()) >= min_points_per_occupied_cell_;
    };
    for (const auto & kv : layers) {
      if (strong(kv.second)) ++diagnostics_.strong_layer_cell_count;
      else ++diagnostics_.weak_layer_cell_count;
    }

    std::unordered_map<int64_t, int> local_density;
    if (density_enabled_) {
      const auto density_start = std::chrono::steady_clock::now();
      local_density.reserve(layers.size());
      for (const auto & kv : layers) {
        const auto & cell = kv.second;
        int total = 0;
        for (int dx = -density_window_cells_; dx <= density_window_cells_; ++dx) {
          for (int dy = -density_window_cells_; dy <= density_window_cells_; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
              const auto it = layers.find(hash3d(cell.ix + dx, cell.iy + dy, cell.iz + dz));
              if (it != layers.end()) total += static_cast<int>(it->second.indices.size());
            }
          }
        }
        local_density.emplace(kv.first, total);
      }
      diagnostics_.density_build_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - density_start).count();
    }

    const auto compatible = [this](const LayerCell & a, const LayerCell & b, double max_gap) {
      return std::abs(a.iz - b.iz) <= 1 && (!z_guard_ || zCompatible(a, b, max_gap));
    };
    const auto densityEnough = [this, &local_density](int64_t key) {
      return !density_enabled_ || local_density[key] >= weak_support_min_density_;
    };
    const auto addHistogram = [](int value, std::array<int, 6> & histogram) {
      const int bin = value <= 1 ? 0 : value == 2 ? 1 : value <= 5 ? 2 :
          value <= 10 ? 3 : value <= 30 ? 4 : 5;
      ++histogram[bin];
    };

    std::vector<int> raw_component_cells, raw_component_points;
    std::vector<ClusterResult> clusters;
    int cid = 0;
    for (auto & root : layers) {
      if (!strong(root.second) || root.second.visited) continue;
      const auto bfs_start = std::chrono::steady_clock::now();
      std::vector<int64_t> queue{root.first};
      root.second.visited = true;
      std::unordered_set<int64_t> component_xy;
      ClusterResult result;
      result.method = name();
      double gather_ms = 0.0;
      for (size_t head = 0; head < queue.size(); ++head) {
        const int64_t key = queue[head];
        const auto cit = layers.find(key);
        if (cit == layers.end()) continue;
        const auto & cell = cit->second;
        const auto gather_start = std::chrono::steady_clock::now();
        result.indices.insert(result.indices.end(), cell.indices.begin(), cell.indices.end());
        gather_ms += std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - gather_start).count();
        const int64_t xy_key = hash2d(cell.ix, cell.iy);
        if (component_xy.insert(xy_key).second) result.cell_keys.push_back(xy_key);

        for (int dx = -1; dx <= 1; ++dx) {
          for (int dy = -1; dy <= 1; ++dy) {
            // Adjacent vertical layers in the same XY cell are part of the
            // same sparse BEV column. Keeping them disconnected caused a
            // single vertical surface to fragment into many layer components.
            const bool vertical_neighbor = dx == 0 && dy == 0;
            if (!vertical_neighbor && cluster_connectivity_ == 4 &&
                std::abs(dx) + std::abs(dy) > 1) continue;
            for (int dz = -1; dz <= 1; ++dz) {
              if (vertical_neighbor && dz == 0) continue;
              const int64_t nk = hash3d(cell.ix + dx, cell.iy + dy, cell.iz + dz);
              const auto nit = layers.find(nk);
              if (nit == layers.end() || !strong(nit->second) || nit->second.visited) continue;
              if (compatible(cell, nit->second, neighbor_z_gap_)) {
                nit->second.visited = true; queue.push_back(nk);
              } else if (z_guard_) {
                ++diagnostics_.z_rejected_direct_edges;
              }
            }
          }
        }

        // A weak layer cell is evidence only: it connects two strong endpoints but is
        // deliberately excluded from cluster points, density and cell statistics.
        if (weak_support_enabled_) {
          for (const auto & dir : std::array<std::pair<int, int>, 4>{{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}}) {
            for (int wdz = -1; wdz <= 1; ++wdz) {
              const int wx = cell.ix + dir.first, wy = cell.iy + dir.second, wz = cell.iz + wdz;
              const int64_t wk = hash3d(wx, wy, wz);
              const auto wit = layers.find(wk);
              if (wit == layers.end() || strong(wit->second) || !compatible(cell, wit->second, neighbor_z_gap_)) continue;
              for (int edz = -1; edz <= 1; ++edz) {
                const int64_t ek = hash3d(cell.ix + 2 * dir.first, cell.iy + 2 * dir.second, wz + edz);
                const auto eit = layers.find(ek);
                if (eit == layers.end() || !strong(eit->second) || eit->second.visited ||
                    !compatible(wit->second, eit->second, neighbor_z_gap_)) continue;
                ++diagnostics_.weak_support_candidates;
                if (!densityEnough(key) || !densityEnough(ek)) {
                  ++diagnostics_.weak_support_density_rejected;
                  continue;
                }
                eit->second.visited = true; queue.push_back(ek);
                ++diagnostics_.weak_support_accepted;
              }
            }
          }
        }

        // Preserve the existing empty-cell bridge for A/B comparison. A weak
        // cell is not empty, so it must use the evidence-based path above.
        if (gap_bridge_) {
          for (const auto & dir : std::array<std::pair<int, int>, 4>{{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}}) {
            const int mx = cell.ix + dir.first, my = cell.iy + dir.second;
            if (layers_per_xy.count(hash2d(mx, my)) || !gapAllowed(cell.ix, cell.iy, dir.first, dir.second)) continue;
            ++diagnostics_.bridge_candidates;
            for (int dz = -1; dz <= 1; ++dz) {
              const int64_t ek = hash3d(cell.ix + 2 * dir.first, cell.iy + 2 * dir.second, cell.iz + dz);
              const auto eit = layers.find(ek);
              if (eit == layers.end() || !strong(eit->second) || eit->second.visited) continue;
              if (compatible(cell, eit->second, bridge_z_gap_)) {
                eit->second.visited = true; queue.push_back(ek);
                ++diagnostics_.bridge_accepted;
              } else if (z_guard_) {
                ++diagnostics_.z_rejected_bridge_edges;
              }
            }
          }
        }
      }
      diagnostics_.cc_bfs_ms += std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - bfs_start).count() - gather_ms;
      diagnostics_.point_gather_ms += gather_ms;
      ++diagnostics_.connected_component_count;
      const int cells = static_cast<int>(result.cell_keys.size());
      const int point_count = static_cast<int>(result.indices.size());
      raw_component_cells.push_back(cells); raw_component_points.push_back(point_count);
      addHistogram(cells, diagnostics_.component_cell_histogram);
      addHistogram(point_count, diagnostics_.component_point_histogram);
      if (cells <= 2) {
        ++diagnostics_.tiny_component_count;
        if (cells == 1) ++diagnostics_.single_cell_component_count;
        if (cells == 2) ++diagnostics_.two_cell_component_count;
      }
      diagnostics_.largest_component_cells = std::max(diagnostics_.largest_component_cells, cells);
      diagnostics_.largest_component_points = std::max(diagnostics_.largest_component_points, point_count);
      const auto filter_start = std::chrono::steady_clock::now();
      const bool reject = !include_weak &&
          (point_count < minClusterPoints(points, result.indices) ||
           cells < minClusterCells(points, result.indices));
      diagnostics_.filter_ms += std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - filter_start).count();
      if (reject) continue;
      fillClusterStats(points, result);
      result.id = cid++;
      clusters.push_back(std::move(result));
    }
    diagnostics_.valid_cluster_count = static_cast<int>(clusters.size());
    const auto median = [](std::vector<int> & values) {
      if (values.empty()) return 0;
      const auto middle = values.begin() + values.size() / 2;
      std::nth_element(values.begin(), middle, values.end());
      return *middle;
    };
    diagnostics_.median_component_cells = median(raw_component_cells);
    diagnostics_.median_component_points = median(raw_component_points);
    return clusters;
  }

  template<typename CellT>
  bool zCompatible(const CellT & a, const CellT & b, double max_gap) const {
    return std::max(0.0, std::max<double>(a.min_z, b.min_z) - std::min<double>(a.max_z, b.max_z)) <= max_gap;
  }
  bool gapAllowed(int ix, int iy, int dx, int dy) const {
    const double r = sensorRange((ix + dx) * bev_x_res_, (iy + dy) * bev_y_res_);
    return r < near_range_max_ ? gap_near_ >= 1 : (r < mid_range_max_ ? gap_mid_ >= 1 : gap_long_ >= 1);
  }
  int64_t bevKey(const PointXYZIRT & p) const {
    return hash2d(static_cast<int>(std::floor(p.x / std::max(0.05, bev_x_res_))),
                  static_cast<int>(std::floor(p.y / std::max(0.05, bev_y_res_))));
  }

  int minClusterPoints(const std::vector<PointXYZIRT> & points,
                       const std::vector<int> & cluster_indices) const {
    double cx = 0.0, cy = 0.0;
    for (int idx : cluster_indices) {
      cx += points[idx].x;
      cy += points[idx].y;
    }
    const double r = cluster_indices.empty() ? 0.0 :
        sensorRange(cx / cluster_indices.size(), cy / cluster_indices.size());
    if (r < near_range_max_) return min_cluster_points_near_;
    if (r < mid_range_max_) return min_cluster_points_mid_;
    return min_cluster_points_long_;
  }

  int minClusterCells(const std::vector<PointXYZIRT> & points,
                      const std::vector<int> & cluster_indices) const {
    double cx = 0.0, cy = 0.0;
    for (int idx : cluster_indices) {
      cx += points[idx].x;
      cy += points[idx].y;
    }
    const double r = cluster_indices.empty() ? 0.0 :
        sensorRange(cx / cluster_indices.size(), cy / cluster_indices.size());
    if (r < near_range_max_) return min_cluster_cells_near_;
    if (r < mid_range_max_) return min_cluster_cells_mid_;
    return min_cluster_cells_long_;
  }

  int initial_layer_point_capacity_=0;
  double bev_x_res_;
  double bev_y_res_;
  int cluster_connectivity_;
  int min_points_per_occupied_cell_;
  int min_cluster_cells_near_;
  int min_cluster_cells_mid_;
  int min_cluster_cells_long_;
  int min_cluster_points_near_;
  int min_cluster_points_mid_;
  int min_cluster_points_long_;
  double near_range_max_;
  double mid_range_max_;
  bool z_guard_, gap_bridge_; double neighbor_z_gap_, bridge_z_gap_; int gap_near_, gap_mid_, gap_long_;
  bool z_layer_enabled_, weak_support_enabled_, density_enabled_;
  double z_layer_resolution_;
  int density_window_cells_, weak_support_min_density_;
  bool z_profile_diagnostics_enabled_;
  double z_profile_bin_size_;
  double z_profile_min_z_ = -0.5;
  ClusterDiagnostics diagnostics_;
};

