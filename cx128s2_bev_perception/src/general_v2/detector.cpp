#include "cx128s2_bev_perception/general_v2/core.hpp"
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <unordered_map>
#include <stdexcept>
#include <array>
#include <bitset>
namespace general_v2
{
namespace
{
using Key = std::pair<int, int>;
struct Hash
{
  size_t operator()(const Key&k) const
  {
    return std::hash<uint64_t>{}((uint64_t(uint32_t(k.first)) << 32) | uint32_t(k.second));
  }
};
struct Cell
{
  Key key;
  std::vector<uint32_t> indices;
  double low = 1e9, high = -1e9;
};
struct Dsu
{
  std::vector<int> p, rank;
  explicit Dsu(size_t n) : p(n), rank(n)
  {
    std::iota(p.begin(), p.end(), 0);
  }
  int find(int a)
  {
    while (a != p[a])
    {
      p[a] = p[p[a]];
      a = p[a];
    }
    return a;
  }
  void join(int a, int b)
  {
    a = find(a);
    b = find(b);
    if (a == b)
    {
      return;
    }
    if (rank[a] < rank[b])
    {
      std::swap(a, b);
    }
    p[b] = a;
    if (rank[a] == rank[b])
    {
      ++rank[a];
    }
  }
};
}
std::vector<Observation> detect(const std::vector<Point>& points, std::vector<uint32_t>& fg,
                                const Config& c, const Ground& ground,
                                const std::vector<uint32_t>& valid, std::vector<uint32_t>& excluded)
{
  // Admission precedes connected components, geometry and tracking. Excluded
  // returns never enter a candidate or its measured box; they are not ground.
  excluded.clear();
  if (c.candidate_ground_noise_rejection && ground.valid) {
    struct Evidence {
      std::vector<double> floor;
      std::bitset<256> floor_rings, body_rings;
      std::array<size_t, 256> low_ring_counts{};
      size_t body = 0, low = 0;
      double level = 0;
      bool unknown = false, supported = false, protected_body = false, reject = false;
    };
    constexpr double cell_m = .5;
    std::unordered_map<Key, Evidence, Hash> evidence;
    evidence.reserve(valid.size()/16+1);
    auto key_of = [](const Point&p) {return Key{int(std::floor(p.x/cell_m)),int(std::floor(p.y/cell_m))};};
    for (auto index : valid) {
      const auto&p = points[index];
      auto & e = evidence[key_of(p)];
      if (p.ring >= 256) {e.unknown = true; continue;}
      const double h = p.z-ground.at(p.x,p.y);
      if (std::abs(h) < .15) {e.floor.push_back(h);e.floor_rings.set(p.ring);}
      if (h > .9 && h < 5) {++e.body;e.body_rings.set(p.ring);}
    }
    auto q = [](const std::vector<double>&v,double fraction) {
      const double position=fraction*(v.size()-1);const size_t lo=size_t(position),hi=std::min(lo+1,v.size()-1);
      return v[lo]+(position-lo)*(v[hi]-v[lo]);
    };
    for (auto&entry:evidence) {
      auto&e=entry.second;
      for(int dx=-2;dx<=2&&!e.protected_body;++dx)for(int dy=-2;dy<=2;++dy) {
        const auto other=evidence.find({entry.first.first+dx,entry.first.second+dy});
        if(other!=evidence.end()&&other->second.body>=6&&other->second.body_rings.count()>=3) {
          e.protected_body=true;break;
        }
      }
      if(e.unknown||e.protected_body||e.floor.size()<20||e.floor_rings.count()<4)continue;
      std::sort(e.floor.begin(),e.floor.end());
      if(q(e.floor,.9)-q(e.floor,.1)>.12)continue;
      e.level=q(e.floor,.5);e.supported=true;
    }
    for(auto index:valid) {
      const auto&p=points[index];auto&e=evidence.at(key_of(p));
      if(!e.supported)continue;
      const double h=p.z-ground.at(p.x,p.y)-e.level;
      if(h>=.18&&h<=.8) {++e.low;++e.low_ring_counts[p.ring];}
    }
    for(auto&entry:evidence) {
      auto&e=entry.second;
      if(!e.supported||e.low<3||double(e.low)/e.floor.size()>.35)continue;
      size_t first=0,second=0,beams=0;
      for(auto count:e.low_ring_counts) {
        if(count)++beams;
        if(count>first){second=first;first=count;}else if(count>second)second=count;
      }
      e.reject=beams<=4&&double(first+second)/e.low>=.75;
    }
    fg.erase(std::remove_if(fg.begin(),fg.end(),[&](uint32_t index) {
      const auto&p=points[index];const auto&e=evidence.at(key_of(p));
      const double h=p.z-ground.at(p.x,p.y)-e.level;
      if(e.reject&&h>=.18&&h<=.8) {excluded.push_back(index);return true;}
      return false;
    }),fg.end());
  }
  std::unordered_map<Key, int, Hash> lookup;
  lookup.reserve(fg.size() / 4 + 1);
  std::vector<Cell> cells;
  for (auto index:fg)
  {
    const auto&p = points[index];
    Key k{int(std::floor(p.x / c.cell)), int(std::floor(p.y / c.cell))};
    auto [it, added] = lookup.emplace(k, int(cells.size()));
    if (added)
    {
      if (int(cells.size()) >= c.max_cells)
      {
        throw std::runtime_error("GeneralV2 occupied-cell budget exceeded");
      }
      cells.push_back({k, {}, 1e9, -1e9});
    }
    auto&cell = cells[it->second];
    cell.indices.push_back(index);
    cell.low = std::min(cell.low, p.z);
    cell.high = std::max(cell.high, p.z);
  }
  Dsu groups(cells.size());
  const int radius = std::max(1, int(std::ceil(c.connection_gap / c.cell)));
  for (size_t i = 0; i < cells.size(); ++i)
  {
    for (int dx = -radius; dx <= radius; ++dx)
    {
      for (int dy = -radius; dy <= radius; ++dy)
      {
        if (dx < 0 || (dx == 0 && dy <= 0))
        {
          continue;
        }
        if (std::hypot(std::max(0, std::abs(dx) - 1) * c.cell,
                       std::max(0, std::abs(dy) - 1) * c.cell) > c.connection_gap)
        {
          continue;
        }
        auto it = lookup.find({cells[i].key.first + dx, cells[i].key.second + dy});
        if (it == lookup.end())
        {
          continue;
        }
        const auto&other = cells[it->second];
        if (std::max(cells[i].low, other.low) - std::min(cells[i].high, other.high) > c.vertical_gap)
        {
          continue;
        }
        groups.join(int(i), it->second);
      }
    }
  }
  std::map<int, std::vector<int> > components;
  for (size_t i = 0; i < cells.size(); ++i)
  {
    components[groups.find(int(i))].push_back(int(i));
  }
  std::vector<Observation> out;
  for (const auto&kv:components)
  {
    if (int(kv.second.size()) < c.min_cells)
    {
      continue;
    }
    Observation o;
    o.cluster_id = kv.first;
    o.cells = int(kv.second.size());
    for (auto j:kv.second)
    {
      o.indices.insert(o.indices.end(), cells[j].indices.begin(), cells[j].indices.end());
    }
    if (int(o.indices.size()) < c.min_points)
    {
      continue;
    }
    Eigen::Vector2d mean = Eigen::Vector2d::Zero();
    for (auto j:o.indices)
    {
      mean += Eigen::Vector2d(points[j].x, points[j].y);
    }
    mean /= double(o.indices.size());
    Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
    for (auto j:o.indices)
    {
      Eigen::Vector2d d = Eigen::Vector2d(points[j].x, points[j].y) - mean;
      cov += d * d.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(cov);
    Eigen::Vector2d axis = solver.eigenvectors().col(1);
    if (axis.x() < 0)
    {
      axis = -axis;
    }
    o.yaw = std::atan2(axis.y(), axis.x());
    Eigen::Vector2d side(-axis.y(), axis.x());
    Eigen::Vector3d low = Eigen::Vector3d::Constant(1e9), high = Eigen::Vector3d::Constant(-1e9);
    for (auto j:o.indices)
    {
      const auto&p = points[j];
      Eigen::Vector2d xy(p.x, p.y);
      Eigen::Vector3d local(axis.dot(xy), side.dot(xy), p.z);
      low = low.cwiseMin(local);
      high = high.cwiseMax(local);
    }
    const Eigen::Vector3d mid = (low + high) * .5;
    o.center << axis.x() * mid.x() + side.x() * mid.y(), axis.y() * mid.x() + side.y() * mid.y(), mid.z();
    o.size = high - low;
    if (o.size.x() < o.size.y())
    {
      std::swap(o.size.x(), o.size.y());
      o.yaw += std::acos(-1.) * .5;
    }
    // Oversized components remain UNKNOWN; they are never silently chopped into vehicles.
    o.vehicle = o.size.x() >= c.vehicle_min_length && o.size.x() <= c.vehicle_max_length &&
                o.size.y() >= c.vehicle_min_width && o.size.y() <= c.vehicle_max_width &&
                o.size.z() >= c.vehicle_min_height && o.size.z() <= c.vehicle_max_height;
    if (o.size.x() < .4 || o.size.y() < .2 || o.size.z() < .2)
    {
      continue;
    }
    out.push_back(std::move(o));
  }
  std::sort(out.begin(), out.end(), [](const Observation&a, const Observation&b){
      if (a.indices.size() != b.indices.size())
      {
        return a.indices.size() > b.indices.size();
      }
      return a.cluster_id < b.cluster_id;
    });
  if (int(out.size()) > c.max_objects)
  {
    out.resize(c.max_objects);
  }
  return out;
}
// Rectangular Hungarian assignment, with one independent unmatched dummy per row.
std::vector<int> assignment(const std::vector<std::vector<double> >& cost, double unmatched)
{
  const int n = int(cost.size());
  if (!n)
  {
    return {};
  }
  const int real = int(cost.front().size()), m = real + n;
  std::vector<double> u(n + 1), v(m + 1);
  std::vector<int> p(m + 1), way(m + 1);
  for (int i = 1; i <= n; ++i)
  {
    p[0] = i;
    int j0 = 0;
    std::vector<double> minv(m + 1, 1e12);
    std::vector<bool>used(m + 1);
    do{
      used[j0] = true;
      int i0 = p[j0], j1 = 0;
      double delta = 1e12;
      for (int j = 1; j <= m; ++j)
      {
        if (!used[j])
        {
          double cur = (j <= real ? cost[i0 - 1][j - 1] : unmatched) - u[i0] - v[j];
          if (cur < minv[j])
          {
            minv[j] = cur;
            way[j] = j0;
          }
          if (minv[j] < delta)
          {
            delta = minv[j];
            j1 = j;
          }
        }
      }
      for (int j = 0; j <= m; ++j)
      {
        if (used[j])
        {
          u[p[j]] += delta;
          v[j] -= delta;
        }
        else{
          minv[j] -= delta;
        }
      }
      j0 = j1;
    }while (p[j0] != 0);
    do{
      int j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    }while (j0);
  }
  std::vector<int> result(n, -1);
  for (int j = 1; j <= real; ++j)
  {
    if (p[j] && cost[p[j] - 1][j - 1] < unmatched)
    {
      result[p[j] - 1] = j - 1;
    }
  }
  return result;
}
}
