class ShapeGraphClusterer : public IClusterer {
public:
  ShapeGraphClusterer(
      double voxel_size, double bev_x_res, double bev_y_res,
      double max_edge_dist_near, double max_edge_dist_mid, double max_edge_dist_long,
      double top_weight, double side_weight, double front_weight, double min_edge_score,
      int min_component_nodes_near, int min_component_nodes_mid, int min_component_nodes_long,
      bool use_top_view, bool use_side_view, bool use_front_view,
      int min_cluster_cells_near, int min_cluster_cells_mid, int min_cluster_cells_long,
      int min_cluster_points_near, int min_cluster_points_mid, int min_cluster_points_long,
      double near_range_max, double mid_range_max)
  : voxel_size_(voxel_size),
    bev_x_res_(bev_x_res),
    bev_y_res_(bev_y_res),
    max_edge_dist_near_(max_edge_dist_near),
    max_edge_dist_mid_(max_edge_dist_mid),
    max_edge_dist_long_(max_edge_dist_long),
    top_weight_(top_weight),
    side_weight_(side_weight),
    front_weight_(front_weight),
    min_edge_score_(min_edge_score),
    min_component_nodes_near_(min_component_nodes_near),
    min_component_nodes_mid_(min_component_nodes_mid),
    min_component_nodes_long_(min_component_nodes_long),
    use_top_view_(use_top_view),
    use_side_view_(use_side_view),
    use_front_view_(use_front_view),
    min_cluster_cells_near_(min_cluster_cells_near),
    min_cluster_cells_mid_(min_cluster_cells_mid),
    min_cluster_cells_long_(min_cluster_cells_long),
    min_cluster_points_near_(min_cluster_points_near),
    min_cluster_points_mid_(min_cluster_points_mid),
    min_cluster_points_long_(min_cluster_points_long),
    near_range_max_(near_range_max),
    mid_range_max_(mid_range_max) {}

  std::vector<ClusterResult> cluster(
      const std::vector<PointXYZIRT> & points,
      const std::vector<int> & foreground_indices) override {
    last_nodes_ = buildNodes(points, foreground_indices);
    last_edges_ = buildEdges(last_nodes_);

    std::vector<std::vector<int>> adjacency(last_nodes_.size());
    for (const auto & e : last_edges_) {
      if (e.a < 0 || e.b < 0 || e.a >= static_cast<int>(last_nodes_.size()) ||
          e.b >= static_cast<int>(last_nodes_.size())) {
        continue;
      }
      adjacency[e.a].push_back(e.b);
      adjacency[e.b].push_back(e.a);
    }

    std::vector<ClusterResult> clusters;
    std::vector<bool> visited(last_nodes_.size(), false);
    int cid = 0;
    for (size_t i = 0; i < last_nodes_.size(); ++i) {
      if (visited[i]) continue;
      std::vector<int> q{static_cast<int>(i)};
      visited[i] = true;
      std::vector<int> component_nodes;
      for (size_t head = 0; head < q.size(); ++head) {
        const int ni = q[head];
        component_nodes.push_back(ni);
        for (int nb : adjacency[ni]) {
          if (visited[nb]) continue;
          visited[nb] = true;
          q.push_back(nb);
        }
      }

      ClusterResult result;
      result.id = cid;
      result.method = name();
      std::unordered_set<int64_t> cell_keys;
      double edge_score_sum = 0.0;
      int component_edge_count = 0;
      for (int ni : component_nodes) {
        const auto & n = last_nodes_[ni];
        result.indices.insert(result.indices.end(), n.point_indices.begin(), n.point_indices.end());
        for (int idx : n.point_indices) cell_keys.insert(bevKey(points[idx]));
      }
      std::unordered_set<int> node_set(component_nodes.begin(), component_nodes.end());
      for (const auto & e : last_edges_) {
        if (node_set.count(e.a) && node_set.count(e.b)) {
          edge_score_sum += e.total_score;
          component_edge_count++;
        }
      }
      result.shape_nodes.reserve(component_nodes.size());
      for (int ni : component_nodes) {
        result.shape_nodes.push_back(last_nodes_[ni]);
      }
      for (const auto & e : last_edges_) {
        if (node_set.count(e.a) && node_set.count(e.b)) {
          result.shape_edges.push_back(e);
        }
      }
      result.cell_keys.assign(cell_keys.begin(), cell_keys.end());
      fillClusterStats(points, result);
      result.shape_node_count = static_cast<int>(component_nodes.size());
      result.shape_edge_count = component_edge_count;
      result.shape_graph_score = component_edge_count > 0 ?
          edge_score_sum / std::max(1, component_edge_count) : 0.0;
      result.shape_top_linearity = result.length / std::max(0.05, result.width);
      result.shape_side_linearity = result.length;
      result.shape_front_linearity = result.width;
      result.shape_area_top = std::max(0.0, result.max_x - result.min_x) *
                              std::max(0.0, result.max_y - result.min_y);

      const double range = sensorRange(result.centroid_x, result.centroid_y);
      if (result.point_count < minClusterPoints(range)) continue;
      if (result.cell_count < minClusterCells(range)) continue;
      if (result.shape_node_count < minComponentNodes(range)) continue;
      result.id = cid++;
      clusters.push_back(result);
    }
    return clusters;
  }

  std::string name() const override {
    return "shape_graph";
  }

  const std::vector<ShapeNode> & debugShapeNodes() const override {
    return last_nodes_;
  }

  const std::vector<ShapeEdge> & debugShapeEdges() const override {
    return last_edges_;
  }

private:
  std::vector<ShapeNode> buildNodes(
      const std::vector<PointXYZIRT> & points,
      const std::vector<int> & foreground_indices) const {
    const double vs = std::max(0.05, voxel_size_);
    std::unordered_map<int64_t, ShapeNode> keyed_nodes;
    for (int idx : foreground_indices) {
      const auto & p = points[idx];
      const int vx = static_cast<int>(std::floor(p.x / vs));
      const int vy = static_cast<int>(std::floor(p.y / vs));
      const int vz = static_cast<int>(std::floor(p.z / vs));
      auto & n = keyed_nodes[hash3d(vx, vy, vz)];
      if (n.point_indices.empty()) {
        n.min_x = n.max_x = p.x;
        n.min_y = n.max_y = p.y;
        n.min_z = n.max_z = p.z;
      }
      n.point_indices.push_back(idx);
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

  std::vector<ShapeEdge> buildEdges(const std::vector<ShapeNode> & nodes) const {
    std::vector<ShapeEdge> edges;
    for (size_t i = 0; i < nodes.size(); ++i) {
      for (size_t j = i + 1; j < nodes.size(); ++j) {
        ShapeEdge e;
        if (makeShapeEdgeIfValid(nodes[i], nodes[j], e)) edges.push_back(e);
      }
    }
    return edges;
  }

  bool makeShapeEdgeIfValid(const ShapeNode & a, const ShapeNode & b, ShapeEdge & e) const {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    const double d3 = std::sqrt(dx * dx + dy * dy + dz * dz);
    const double range = 0.5 * (sensorRange(a.x, a.y) + sensorRange(b.x, b.y));
    const double max_d = maxEdgeDistance(range);
    if (d3 > max_d) return false;

    e.a = a.id;
    e.b = b.id;
    e.top_score = use_top_view_ ? computeViewConnectionScoreTop(a, b) : 0.0;
    e.side_score = use_side_view_ ? computeViewConnectionScoreSide(a, b) : 0.0;
    e.front_score = use_front_view_ ? computeViewConnectionScoreFront(a, b) : 0.0;
    const double dist_penalty = clamp(1.0 - d3 / std::max(1e-3, max_d), 0.0, 1.0);
    const double weighted =
        top_weight_ * e.top_score + side_weight_ * e.side_score + front_weight_ * e.front_score;
    e.total_score = 0.75 * weighted + 0.25 * dist_penalty;
    if (e.total_score < min_edge_score_) return false;
    e.reason = "MULTI_VIEW_DISTANCE";
    return true;
  }

  double computeViewConnectionScoreTop(const ShapeNode & a, const ShapeNode & b) const {
    return distanceScore(std::hypot(a.x - b.x, a.y - b.y), maxEdgeDistance(0.5 * (
        sensorRange(a.x, a.y) + sensorRange(b.x, b.y))));
  }

  double computeViewConnectionScoreSide(const ShapeNode & a, const ShapeNode & b) const {
    return distanceScore(std::hypot(a.x - b.x, a.z - b.z), maxEdgeDistance(0.5 * (
        sensorRange(a.x, a.y) + sensorRange(b.x, b.y))));
  }

  double computeViewConnectionScoreFront(const ShapeNode & a, const ShapeNode & b) const {
    return distanceScore(std::hypot(a.y - b.y, a.z - b.z), maxEdgeDistance(0.5 * (
        sensorRange(a.x, a.y) + sensorRange(b.x, b.y))));
  }

  double distanceScore(double d, double max_d) const {
    return clamp(1.0 - d / std::max(1e-3, max_d), 0.0, 1.0);
  }

  double maxEdgeDistance(double range) const {
    if (range < near_range_max_) return max_edge_dist_near_;
    if (range < mid_range_max_) return max_edge_dist_mid_;
    return max_edge_dist_long_;
  }

  int minComponentNodes(double range) const {
    if (range < near_range_max_) return min_component_nodes_near_;
    if (range < mid_range_max_) return min_component_nodes_mid_;
    return min_component_nodes_long_;
  }

  int minClusterPoints(double range) const {
    if (range < near_range_max_) return min_cluster_points_near_;
    if (range < mid_range_max_) return min_cluster_points_mid_;
    return min_cluster_points_long_;
  }

  int minClusterCells(double range) const {
    if (range < near_range_max_) return min_cluster_cells_near_;
    if (range < mid_range_max_) return min_cluster_cells_mid_;
    return min_cluster_cells_long_;
  }

  int64_t bevKey(const PointXYZIRT & p) const {
    return hash2d(static_cast<int>(std::floor(p.x / std::max(0.05, bev_x_res_))),
                  static_cast<int>(std::floor(p.y / std::max(0.05, bev_y_res_))));
  }

  double voxel_size_;
  double bev_x_res_;
  double bev_y_res_;
  double max_edge_dist_near_;
  double max_edge_dist_mid_;
  double max_edge_dist_long_;
  double top_weight_;
  double side_weight_;
  double front_weight_;
  double min_edge_score_;
  int min_component_nodes_near_;
  int min_component_nodes_mid_;
  int min_component_nodes_long_;
  bool use_top_view_;
  bool use_side_view_;
  bool use_front_view_;
  int min_cluster_cells_near_;
  int min_cluster_cells_mid_;
  int min_cluster_cells_long_;
  int min_cluster_points_near_;
  int min_cluster_points_mid_;
  int min_cluster_points_long_;
  double near_range_max_;
  double mid_range_max_;
  std::vector<ShapeNode> last_nodes_;
  std::vector<ShapeEdge> last_edges_;
};

class NoOpClusterer : public IClusterer {
public:
  std::vector<ClusterResult> cluster(
      const std::vector<PointXYZIRT> & points,
      const std::vector<int> & foreground_indices) override {
    if (foreground_indices.empty()) return {};
    ClusterResult result;
    result.id = 0;
    result.indices = foreground_indices;
    result.method = name();
    fillClusterStats(points, result);
    result.cell_count = 1;
    return {result};
  }

  std::string name() const override {
    return "none";
  }
};

