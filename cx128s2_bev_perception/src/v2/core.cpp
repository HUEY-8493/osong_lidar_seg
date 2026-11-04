#include "cx128s2_bev_perception/v2/length_lock.hpp"
#include "crane_xy_motion.hpp"
#include "crane_hist_motion.hpp"
#include "retained_extent.hpp"
#include "cx128s2_bev_perception/v2/core.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <unordered_map>
namespace crane_v2 {
namespace {
using Clock = std::chrono::steady_clock;
double elapsed(Clock::time_point t)
{
  return std::chrono::duration<double, std::milli>(Clock::now() - t).count();
}
constexpr uint64_t absent = ~uint64_t(0);
constexpr int bias = 1 << 20;
uint64_t key(int x, int y, int z)
{
  return (uint64_t(x + bias) << 42) | (uint64_t(y + bias) << 21) | uint64_t(z + bias);
}
uint64_t hash(uint64_t v)
{
  v ^= v >> 30;
  v *= 0xbf58476d1ce4e5b9ULL;
  v ^= v >> 27;
  v *= 0x94d049bb133111ebULL;
  return v ^ (v >> 31);
}
double sq(double v)
{
  return v * v;
}
double distance(Vec3 a, Vec3 b)
{
  return std::sqrt(sq(a.x - b.x) + sq(a.y - b.y) + sq(a.z - b.z));
}
double median(std::vector<double>v)
{
  if (v.empty())
    return 0;
  auto m = v.begin() + v.size() / 2;
  std::nth_element(v.begin(), m, v.end());
  return *m;
}
struct Entry {
  uint64_t key;
  uint32_t point;
};
struct Cell {
  uint64_t key;
  uint32_t begin, end, parent;
  int x, y, z;
  Vec3 mean;
};
struct FlatIndex {
  std::vector<uint64_t>keys;
  std::vector<uint32_t>values;
  size_t mask = 0;
  void clear(size_t n)
  {
    size_t cap = 8;
    while (cap < n * 2)
      cap *= 2;
    keys.assign(cap, absent);
    values.resize(cap);
    mask = cap - 1;
  }
  void insert(uint64_t k, uint32_t v)
  {
    size_t p = hash(k) & mask;
    while (keys[p] != absent)
      p = (p + 1) & mask;
    keys[p] = k;
    values[p] = v;
  }
  int find(uint64_t k)const
  {
    if (keys.empty())
      return -1;
    size_t p = hash(k) & mask;
    while (keys[p] != absent)
    {
      if (keys[p] == k)
        return int(values[p]);
      p = (p + 1) & mask;
    }
    return -1;
  }
};
struct Surface {
  uint32_t root = 0, count = 0;
  double sx = 0, sy = 0, sz = 0, xx = 0, xy = 0, yy = 0;
  double xmin = 1e9, xmax = -1e9, ymin = 1e9, ymax = -1e9, zmin = 1e9, zmax = -1e9;
};
struct Patch {
  int x, y;
  double z;
};
struct Plane {
  double a = 0, b = 0, c = 0;
  bool valid = false;
  double at(double x, double y)const
  {
    return a * x + b * y + c;
  }
};
Plane bodyPlane(const std::vector<Point>&points, const std::vector<uint32_t>&ids)
{
  Plane best;
  double best_count = 0;
  uint64_t rng = 0x1234abcd88776655ULL;
  std::vector<std::pair<uint64_t, size_t> >xy_groups;
  xy_groups.reserve(ids.size());
  for (size_t i = 0; i < ids.size(); ++i)
  {
    auto &p = points[ids[i]];
    xy_groups.push_back({key(int(std::floor(p.x / .04)), int(std::floor(p.y / .04)), 0), i});
  }
  std::sort(xy_groups.begin(), xy_groups.end());
  std::vector<double>weight(ids.size());
  for (size_t begin = 0; begin < xy_groups.size();)
  {
    size_t end = begin + 1;
    while (end < xy_groups.size()&&xy_groups[end].first == xy_groups[begin].first)
      ++end;
    for (size_t j = begin; j < end; ++j)
      weight[xy_groups[j].second] = 1. / (end - begin);
    begin = end;
  }
  auto next = [&](){
                rng ^= rng << 13;
                rng ^= rng >> 7;
                rng ^= rng << 17;
                return size_t(rng % ids.size());
              };
  for (int trial = 0; trial < 64; ++trial)
  {
    auto &p = points[ids[next()]], &q = points[ids[next()]], &r = points[ids[next()]];
    double xx = q.x - p.x, xy = q.y - p.y, xz = q.z - p.z, yx = r.x - p.x, yy = r.y - p.y, yz = r.z - p.z;
    double nx = xy * yz - xz * yy, ny = xz * yx - xx * yz, nz = xx * yy - xy * yx;
    if (std::abs(nz) < .002)
      continue;
    Plane candidate{-nx / nz, -ny / nz, 0, true};
    if (candidate.a * candidate.a + candidate.b * candidate.b > .333)
      continue;
    candidate.c = p.z - candidate.a * p.x - candidate.b * p.y;
    double n = 0;
    for (size_t j = 0; j < ids.size(); ++j)
    {
      auto &v = points[ids[j]];
      if (std::abs(v.z - candidate.at(v.x, v.y)) < .025)
        n += weight[j];
    }
    if (n > best_count)
    {
      best_count = n;
      best = candidate;
    }
  }
  if (!best.valid||best_count < 20)
    return {}
  ;
  double n = 0, x = 0, y = 0, z = 0, xx = 0, xy = 0, yy = 0, xz = 0, yz = 0;
  for (size_t j = 0; j < ids.size(); ++j)
  {
    auto&p = points[ids[j]];
    if (std::abs(p.z - best.at(p.x, p.y)) >= .025)
      continue;
    double w = weight[j];
    n += w;
    x += w * p.x;
    y += w * p.y;
    z += w * p.z;
    xx += w * p.x * p.x;
    xy += w * p.x * p.y;
    yy += w * p.y * p.y;
    xz += w * p.x * p.z;
    yz += w * p.y * p.z;
  }
  x /= n;
  y /= n;
  z /= n;
  xx = xx / n - x * x;
  xy = xy / n - x * y;
  yy = yy / n - y * y;
  xz = xz / n - x * z;
  yz = yz / n - y * z;
  double determinant = xx * yy - xy * xy;
  if (determinant > 1e-8)
  {
    best.a = (xz * yy - yz * xy) / determinant;
    best.b = (yz * xx - xz * xy) / determinant;
    best.c = z - best.a * x - best.b * y;
  }
  return best;
}

}
struct Engine::Impl {
  LengthLock published_length;
  RetainedExtent retained_extent;
  Cargo candidate_reference;
  double last_connected_confirmation = -1;
  uint64_t length_relation_id = 0;
  double length_separation_since = -1;
  CraneXYMotion crane_xy;
  CraneHistogramMotion crane_hist;
  Config c;
  Plane selected_plane, tracked_plane;
  std::vector<Entry>entries, temp;
  std::vector<Cell>cells;
  FlatIndex index;
  std::vector<Surface>surfaces;
  std::vector<int>root_surface;
  std::vector<uint32_t>ground_points;
  std::vector<uint8_t>floor_mask;
  std::array<Hook, 2>hooks{};
  Cargo track;
  struct ShrinkHold {
    Cargo box;
    std::string state;
    bool active = false;
    double since = -1;
  } shrink_hold;
  void prepareShrinkHold(const Cargo&measured, double stamp, bool continuous)
  {
    if (!continuous||c.cargo_shrink_hold_sec <= 0||!armed||completion_consumed||
        !shrink_hold.box.id||shrink_hold.box.id != track.id)
    {
      shrink_hold = {};
      return;
    }
    const bool same = !measured.direct||sameBody(measured.center, measured.length,
                                                measured.width, measured.yaw, &selected_plane);
    if (!same)
    {
      shrink_hold = {};
      return;
    }
    if (shrink_hold.active)
    {
      if (stamp - shrink_hold.since + 1e-8 >= c.cargo_shrink_hold_sec||
          (measured.direct&&measured.length >= c.cargo_shrink_recovery_ratio * shrink_hold.box.length))
        shrink_hold = {};
      return; // A window never restarts on the same expiration/recovery frame.
    }
    if (measured.direct&&measured.length <= c.cargo_shrink_ratio * shrink_hold.box.length)
    {
      shrink_hold.active = true;
      shrink_hold.since = stamp;
    }
  }
  void publishShrinkHold(Output&o, double stamp)
  {
    if (o.work_complete||o.cargo.id != shrink_hold.box.id)
    {
      shrink_hold = {};
    }
    if (shrink_hold.active)
    {
      if (o.cargo.direct)
        o.measured_cargo_raw_indices = o.cargo.raw_indices;
      o.cargo = shrink_hold.box;
      o.cargo.direct = false;
      o.cargo.held = true;
      o.cargo.connected = false;
      o.cargo.bottom_valid = false;
      o.cargo.bottom_observation = "UNKNOWN";
      o.cargo.raw_indices.clear();
      o.cargo.length_observation = "PARTIAL_OBSERVATION";
      o.cargo_shrink_held = true;
      o.cargo_shrink_hold_elapsed_sec = stamp - shrink_hold.since;
      o.state = shrink_hold.state;
      o.reason = "CARGO_LENGTH_SHRINK_HOLD";
      o.work_active = true;
      o.work_complete = false;
      o.clearance_valid = false;
      o.clearance = 0;
      o.connection_observation = "UNKNOWN";
      o.ground_supported = o.ground_support_held = o.contact_ground_direct_valid = false;
      o.support_path_raw.clear();
      // No frozen box or rejected footprint creates motion evidence.
      o.motion_dz = o.matched_fraction = 0;
      motion.clear();
      last_patches.clear();
      return;
    }
    if (o.cargo.direct&&o.cargo.confirmed&&o.work_active&&!o.work_complete)
    {
      shrink_hold.box = o.cargo;
      shrink_hold.box.raw_indices.clear(); // Never retain old Raw ownership.
      shrink_hold.state = o.state;
    }
    else if (!o.cargo.held||!o.work_active) shrink_hold = {};
  }
  double first_seen = -1, confirmed_direct_duration = 0, last_stamp = -1, run_start = -1;
  uint64_t next_id = 1;
  bool armed = false, completion_consumed = false;
  double event_start = -1, contact_since = -1, detach_since = -1;
  std::string completion_reason;
  std::vector<Patch>last_patches;
  std::vector<uint32_t>body_refs, motion_refs;
  double last_ground_support = -1;
  std::vector<uint32_t>support_paths;
  struct MotionSample {
    double stamp, dz;
  };
  std::vector<MotionSample>motion;
  double latest_dz = 0;
  explicit Impl(Config cfg) : c(cfg)
  {
    if(c.crane_motion_mode!="scene_histogram"&&c.crane_motion_mode!="z_histogram"&&c.crane_motion_mode!="body_xy")
      throw std::invalid_argument("Crane motion mode must be scene_histogram, z_histogram or body_xy");
    if(!std::isfinite(c.crane_hist_xy_bin_m)||c.crane_hist_xy_bin_m<.05||c.crane_hist_xy_bin_m>2.||
       !std::isfinite(c.x_min)||!std::isfinite(c.x_max)||!std::isfinite(c.y_min)||!std::isfinite(c.y_max)||
       c.x_max<=c.x_min||c.y_max<=c.y_min||
       (c.x_max-c.x_min+c.y_max-c.y_min)/c.crane_hist_xy_bin_m>4096)
      throw std::invalid_argument("invalid Crane scene histogram bounds");
    if(!std::isfinite(c.crane_hist_bin_m)||c.crane_hist_bin_m<.02||c.crane_hist_bin_m>1.||
       !std::isfinite(c.crane_hist_start_score)||!std::isfinite(c.crane_hist_stop_score)||
       c.crane_hist_stop_score<=0||c.crane_hist_start_score<=c.crane_hist_stop_score||c.crane_hist_start_score>1||
       c.crane_hist_min_points<10||!std::isfinite(c.z_min)||!std::isfinite(c.z_max)||
       c.z_max<=c.z_min||(c.z_max-c.z_min)/c.crane_hist_bin_m>4096)
      throw std::invalid_argument("invalid Crane histogram configuration");
    for(double sec:{c.crane_hist_lag_sec,c.crane_hist_start_confirm_sec,c.crane_hist_stop_confirm_sec})
      if(!std::isfinite(sec)||sec<=0)throw std::invalid_argument("invalid Crane histogram timing");
    for(double value:{c.crane_xy_start_m,c.crane_xy_min_speed,c.crane_xy_start_confirm_sec,
                      c.crane_xy_stop_span_m,c.crane_xy_stop_confirm_sec})
      if(!std::isfinite(value)||value<=0)throw std::invalid_argument("invalid Crane XY configuration");
    if(c.crane_xy_min_points<3)throw std::invalid_argument("Crane XY needs at least three points per root");
    for(const auto& r:c.crane_root_rois)
      for(int i=0;i<6;i+=2)
        if(!std::isfinite(r[i])||!std::isfinite(r[i+1])||r[i]>=r[i+1])
          throw std::invalid_argument("invalid Crane body root ROI");
    if (!std::isfinite(c.cargo_length_confirm_sec)||c.cargo_length_confirm_sec <= 0)
      throw std::invalid_argument("invalid Cargo length confirmation time");
    if (!std::isfinite(c.cargo_shrink_hold_sec)||c.cargo_shrink_hold_sec < 0||
        !std::isfinite(c.cargo_shrink_ratio)||!std::isfinite(c.cargo_shrink_recovery_ratio)||
        c.cargo_shrink_ratio <= 0||c.cargo_shrink_ratio >= c.cargo_shrink_recovery_ratio||
        c.cargo_shrink_recovery_ratio > 1)
      throw std::invalid_argument("invalid Cargo shrink hold configuration");
    if (!std::isfinite(c.voxel_xy)||!std::isfinite(c.voxel_z)||c.voxel_xy < .001||c.voxel_z < .001||c.confirm_sec < 0||c.loss_sec <= 0||c.contact_confirm_sec < 0||c.detach_confirm_sec < 0)
      throw std::invalid_argument("invalid V2 configuration");
  }
  uint32_t root(uint32_t i)
  {
    while (cells[i].parent != i)
    {
      cells[i].parent = cells[cells[i].parent].parent;
      i = cells[i].parent;
    }
    return i;
  }
  void join(uint32_t a, uint32_t b)
  {
    a = root(a);
    b = root(b);
    if (a != b)
      cells[std::max(a, b)].parent = std::min(a, b);
  }
  void build(const std::vector<Point>&p)
  {
    entries.clear();
    for (uint32_t i = 0; i < p.size(); ++i)
    {
      auto&a = p[i];
      if (!std::isfinite(a.x)||!std::isfinite(a.y)||!std::isfinite(a.z)||a.x < c.x_min||a.x > c.x_max||a.y < c.y_min||a.y > c.y_max||a.z < c.z_min||a.z > c.z_max)
        continue;
      entries.push_back({key(int(std::floor(a.x / c.voxel_xy)), int(std::floor(a.y / c.voxel_xy)), int(std::floor(a.z / c.voxel_z))), i});
    }
    // Stable radix sorting: bounded 8 passes, contiguous reusable memory.
    temp.resize(entries.size());
    for (unsigned shift = 0; shift < 64; shift += 8)
    {
      std::array<size_t, 256>count{};
      for (auto&e:entries)
        ++count[(e.key >> shift) & 255];
      size_t sum = 0;
      for (auto&n:count)
      {
        size_t old = n;
        n = sum;
        sum += old;
      }
      for (auto&e:entries)
        temp[count[(e.key >> shift) & 255]++] = e;
      entries.swap(temp);
    }
    cells.clear();
    for (size_t begin = 0; begin < entries.size();)
    {
      size_t end = begin + 1;
      while (end < entries.size()&&entries[end].key == entries[begin].key)
        ++end;
      uint64_t k = entries[begin].key;
      Cell cell{k, uint32_t(begin), uint32_t(end), uint32_t(cells.size()), int(k >> 42) - bias, int((k >> 21) & ((1 << 21) - 1)) - bias, int(k & ((1 << 21) - 1)) - bias, {}};
      for (size_t i = begin; i < end; ++i)
      {
        auto&a = p[entries[i].point];
        cell.mean.x += a.x;
        cell.mean.y += a.y;
        cell.mean.z += a.z;
      }
      double n = end - begin;
      cell.mean.x /= n;
      cell.mean.y /= n;
      cell.mean.z /= n;
      cells.push_back(cell);
      begin = end;
    }
    index.clear(cells.size());
    for (uint32_t i = 0; i < cells.size(); ++i)
      index.insert(cells[i].key, i);
  }
  void observeHooks(const std::vector<Point>&p, double stamp)
  {
    std::vector<uint8_t>eligible(cells.size(), 0);
    for (size_t i = 0; i < cells.size(); ++i)
    {
      auto &v = cells[i];
      if (v.end - v.begin < 2||v.mean.z < 1.05||v.mean.z > 6.2||std::abs(v.mean.y) > .8||std::abs(v.mean.x) < .3||std::abs(v.mean.x) > 1.5)
        continue;
      int lateral = 0;
      for (int dx = -1; dx <= 1; ++dx)
        for (int dy = -1; dy <= 1; ++dy)
        {
          if (!dx&&!dy)
            continue;
          int j = index.find(key(v.x + dx, v.y + dy, v.z));
          if (j >= 0&&cells[j].end - cells[j].begin >= 2)
            ++lateral;
        }
      eligible[i] = lateral >= 2;
    }
    for (uint32_t i = 0; i < cells.size(); ++i)
      if (eligible[i])
      {
        auto &v = cells[i];
        for (int dx = -1; dx <= 1; ++dx)
          for (int dy = -1; dy <= 1; ++dy)
            for (int dz = -1; dz <= 1; ++dz)
            {
              if (dx > 0||(dx == 0&&dy > 0)||(dx == 0&&dy == 0&&dz >= 0))
                continue;
              int j = index.find(key(v.x + dx, v.y + dy, v.z + dz));
              if (j >= 0&&eligible[j]&&(cells[j].mean.x > 0) == (v.mean.x > 0))
                join(i, j);
            }
      }
    surfaces.clear();
    root_surface.assign(cells.size(), -1);
    for (uint32_t i = 0; i < cells.size(); ++i)
      if (eligible[i])
      {
        uint32_t r = root(i);
        if (root_surface[r] < 0)
        {
          root_surface[r] = surfaces.size();
          Surface s;
          s.root = r;
          surfaces.push_back(s);
        }
        auto &s = surfaces[root_surface[r]];
        for (uint32_t j = cells[i].begin; j < cells[i].end; ++j)
        {
          auto &a = p[entries[j].point];
          ++s.count;
          s.sx += a.x;
          s.sy += a.y;
          s.sz += a.z;
          s.xmin = std::min(s.xmin, double(a.x));
          s.xmax = std::max(s.xmax, double(a.x));
          s.ymin = std::min(s.ymin, double(a.y));
          s.ymax = std::max(s.ymax, double(a.y));
          s.zmin = std::min(s.zmin, double(a.z));
          s.zmax = std::max(s.zmax, double(a.z));
        }
      }
    for (int side = 0; side < 2; ++side)
    {
      int selected = -1;
      double score = -1e9;
      for (size_t i = 0; i < surfaces.size(); ++i)
      {
        auto &s = surfaces[i];
        double x = s.sx / s.count, y = s.sy / s.count, z = s.sz / s.count;
        if ((x > 0) != bool(side)||s.count < 60||s.xmax - s.xmin < .08||s.xmax - s.xmin > .55||s.ymax - s.ymin < .08||s.ymax - s.ymin > .55||s.zmax - s.zmin < .10||s.zmax - s.zmin > .75)
          continue;
        if (s.xmin < (side?.3:-1.5) + .03||s.xmax > (side?1.5:-.3) - .03||s.ymin < -.77||s.ymax > .77)
          continue;
        double d = 0;
        if (hooks[side].last_direct_sec > 0&&stamp - hooks[side].last_direct_sec < .5)
        {
          d = distance(Vec3{x, y, z}, hooks[side].housing);
          if (d > .6)
            continue;
        }
        // The hanging Housing is below upper rigging/structural hardware.
        // This is an ordering preference, not a mandatory acquisition height.
        double candidate = std::log(double(s.count)) - 1.5 * z - 3 * d;
        if (candidate > score)
        {
          score = candidate;
          selected = i;
        }
      }
      if (selected >= 0)
      {
        auto &s = surfaces[selected];
        std::vector<double>xs, ys, zs;
        std::vector<uint32_t> holder_raw;
        for (uint32_t i = 0; i < cells.size(); ++i)
          if (eligible[i]&&root(i) == s.root)
            for (uint32_t j = cells[i].begin; j < cells[i].end; ++j)
            {
              auto&a = p[entries[j].point];
              xs.push_back(a.x);
              ys.push_back(a.y);
              zs.push_back(a.z);
              holder_raw.push_back(a.raw);
            }
        Hook h;
        h.direct = true;
        h.body_min={s.xmin,s.ymin,s.zmin};
        h.body_max={s.xmax,s.ymax,s.zmax};
        h.raw_indices=std::move(holder_raw);
        h.housing = {median(xs), median(ys), median(zs)};
        h.virtual_hook = h.housing;
        h.virtual_hook.z -= c.virtual_hook_offset;
        h.points = xs.size();
        h.last_direct_sec = stamp;
        h.confidence = std::min(1., h.points / 300.);
        if (hooks[side].last_direct_sec > 0&&stamp - hooks[side].last_direct_sec < .5)
          h.delta = {h.housing.x - hooks[side].housing.x, h.housing.y - hooks[side].housing.y, h.housing.z - hooks[side].housing.z}
        ;
        hooks[side] = h;
      }
      else{
        hooks[side].direct = false;
        hooks[side].held = hooks[side].last_direct_sec > 0&&stamp - hooks[side].last_direct_sec <= c.occlusion_sec;
        hooks[side].points = 0;
        hooks[side].raw_indices.clear();
        hooks[side].delta = {};
        if (!hooks[side].held)
          hooks[side].confidence = 0;
      }
    }
  }
  Ground ground(const std::vector<Point>&p)
  {
    Ground g;
    ground_points.clear();
    floor_mask.assign(p.size(), 0);
    // Current-frame low, spatially distributed support. Calibration bounds the
    // search, not the result; no Z=0 fallback or stored floor measurement.
    std::array<uint32_t, 65>hist{};
    for (auto&e:entries)
    {
      auto&a = p[e.point];
      if (a.z >= -.5&&a.z < .8)
        ++hist[std::clamp(int((a.z + .5) / .02), 0, 64)];
    }
    int mode = -1;
    uint32_t best = 0;
    for (int b = 0; b < 65; ++b)
    {
      if (hist[b] > best)
      {
        best = hist[b];
        mode = b;
      }
    }
    if (mode < 0||best < 100)
      return g;
    // Prefer an independently substantial lower surface to a dense cargo top.
    for (int b = 0; b < mode - 4; ++b)
      if (hist[b] >= std::max(uint32_t(100), best / 5))
      {
        mode = b;
        break;
      }
    const double center = -.5 + (mode + .5) * .02;
    double sum = 0;
    std::vector<double>zs;
    double xmin = 1e9, xmax = -1e9, ymin = 1e9, ymax = -1e9;
    for (auto&e:entries)
    {
      auto&a = p[e.point];
      if (std::abs(a.z - center) < .04)
      {
        ground_points.push_back(e.point);
        g.raw_indices.push_back(a.raw);
        sum += a.z;
        zs.push_back(a.z);
        xmin = std::min(xmin, double(a.x));
        xmax = std::max(xmax, double(a.x));
        ymin = std::min(ymin, double(a.y));
        ymax = std::max(ymax, double(a.y));
      }
    }
    if (zs.size() < 100||xmax - xmin < 2||ymax - ymin < 2)
    {
      g.raw_indices.clear();
      ground_points.clear();
      return g;
    }
    g.z = sum / zs.size();
    for (auto&z:zs)
      z = std::abs(z - g.z);
    g.mad = median(zs);
    g.valid = g.mad < .04;
    if (g.valid)
      for (auto i:ground_points)
        floor_mask[i] = 1;
    g.observation = g.valid?"DIRECT":"UNKNOWN";
    return g;
  }
  std::vector<uint64_t> bodySheet(const std::vector<Point>&p, const std::vector<uint32_t>&ids, const Plane&plane)
  {
    constexpr double resolution = .04;
    std::vector<Entry>v;
    for (auto i:ids)
      if (std::abs(p[i].z - plane.at(p[i].x, p[i].y)) < .03)
        v.push_back({key(int(std::floor(p[i].x / resolution)), int(std::floor(p[i].y / resolution)), 0), i});
    std::sort(v.begin(), v.end(), [](auto&a, auto&b){
        return a.key < b.key;
      });
    struct Node {
      uint64_t k;
      uint32_t begin, end, parent;
      int x, y;
      bool core = false;
    };
    std::vector<Node>nodes;
    FlatIndex lookup;
    for (uint32_t i = 0; i < v.size();)
    {
      uint32_t end = i + 1;
      while (end < v.size()&&v[end].key == v[i].key)
        ++end;
      auto k = v[i].key;
      nodes.push_back({k, i, end, uint32_t(nodes.size()), int(k >> 42) - bias, int((k >> 21) & ((1 << 21) - 1)) - bias, false});
      i = end;
    }
    lookup.clear(nodes.size());
    for (uint32_t i = 0; i < nodes.size(); ++i)
      lookup.insert(nodes[i].k, i);
    for (auto &n:nodes)
    {
      int neighbors = 0;
      for (int dx = -2; dx <= 2; ++dx)
        for (int dy = -2; dy <= 2; ++dy)
          if (lookup.find(key(n.x + dx, n.y + dy, 0)) >= 0)
            ++neighbors;
      n.core = neighbors >= 12;
    }
    auto root = [&](uint32_t i){
                  while (nodes[i].parent != i)
                  {
                    nodes[i].parent = nodes[nodes[i].parent].parent;
                    i = nodes[i].parent;
                  }
                  return i;
                };
    for (uint32_t i = 0; i < nodes.size(); ++i)
      if (nodes[i].core)
        for (int dx = -1; dx <= 1; ++dx)
          for (int dy = -1; dy <= 1; ++dy)
          {
            int j = lookup.find(key(nodes[i].x + dx, nodes[i].y + dy, 0));
            if (j >= 0&&nodes[j].core)
            {
              auto a = root(i), b = root(j);
              nodes[std::max(a, b)].parent = std::min(a, b);
            }
          }
    std::vector<double>score(nodes.size(), 0), sx(nodes.size(), 0), sy(nodes.size(), 0);
    std::vector<int>support(nodes.size(), 0);
    for (uint32_t i = 0; i < nodes.size(); ++i)
      if (nodes[i].core)
      {
        auto r = root(i);
        for (uint32_t j = nodes[i].begin; j < nodes[i].end; ++j)
        {
          auto&a = p[v[j].point];
          ++score[r];
          sx[r] += a.x;
          sy[r] += a.y;
          for (auto&h:hooks)
            if (h.direct&&std::abs(a.x - h.virtual_hook.x) <= .15&&std::abs(a.y - h.virtual_hook.y) <= .15&&a.z < h.virtual_hook.z + .05)
              ++support[r];
        }
      }
    int selected = -1;
    double best = -1e9;
    for (uint32_t i = 0; i < nodes.size(); ++i)
      if (score[i] >= 40)
      {
        bool same = track.id&&(last_stamp - track.last_direct_sec <= c.loss_sec||last_stamp - run_start < c.loss_sec)&&std::hypot(sx[i] / score[i] - track.center.x, sy[i] / score[i] - track.center.y) < 1.;
        if (support[i] < 8&&!same)
          continue;
        double rank = std::log(score[i]) + (same?3:0) + (support[i] >= 8?5:0);
        if (rank > best)
        {
          best = rank;
          selected = i;
        }
      }
    std::vector<uint64_t>footprint;
    if (selected < 0)
      return footprint;
    for (uint32_t i = 0; i < nodes.size(); ++i)
      if (nodes[i].core&&root(i) == uint32_t(selected))
        for (int dx = -1; dx <= 1; ++dx)
          for (int dy = -1; dy <= 1; ++dy)
            footprint.push_back(key(nodes[i].x + dx, nodes[i].y + dy, 0));
    std::sort(footprint.begin(), footprint.end());
    footprint.erase(std::unique(footprint.begin(), footprint.end()), footprint.end());
    return footprint;
  }
  bool observedConnection(int side, const Plane&plane, const std::vector<uint64_t>&footprint)
  {
    const auto&h = hooks[side];
    if (!h.direct)
      return false;
    double gap = h.virtual_hook.z - plane.at(h.virtual_hook.x, h.virtual_hook.y);
    if (gap < -.05||gap > c.connection_gap)
      return false;
    if (gap <= .20)
      return true;
    double xmin = h.virtual_hook.x - .2, xmax = h.virtual_hook.x + .2, ymin = h.virtual_hook.y - .2, ymax = h.virtual_hook.y + .2;
    for (auto k:footprint)
    {
      double x = (int(k >> 42) - bias + .5) * .04, y = (int((k >> 21) & ((1 << 21) - 1)) - bias + .5) * .04;
      xmin = std::min(xmin, x - .1);
      xmax = std::max(xmax, x + .1);
      ymin = std::min(ymin, y - .1);
      ymax = std::max(ymax, y + .1);
    }
    std::vector<uint8_t>visited(cells.size(), 0);
    std::vector<uint32_t>queue;
    int ix = int(std::floor(h.virtual_hook.x / c.voxel_xy)), iy = int(std::floor(h.virtual_hook.y / c.voxel_xy)), iz = int(std::floor(h.virtual_hook.z / c.voxel_z));
    for (int dx = -3; dx <= 3; ++dx)
      for (int dy = -3; dy <= 3; ++dy)
        for (int dz = -3; dz <= 3; ++dz)
        {
          int j = index.find(key(ix + dx, iy + dy, iz + dz));
          if (j >= 0&&distance(cells[j].mean, h.virtual_hook) < .20)
          {
            visited[j] = 1;
            queue.push_back(j);
          }
        }
    for (size_t head = 0; head < queue.size(); ++head)
    {
      auto &a = cells[queue[head]];
      if (std::abs(a.mean.z - plane.at(a.mean.x, a.mean.y)) < .05&&std::binary_search(footprint.begin(), footprint.end(), key(int(std::floor(a.mean.x / .04)), int(std::floor(a.mean.y / .04)), 0)))
        return true;
      for (int dx = -1; dx <= 1; ++dx)
        for (int dy = -1; dy <= 1; ++dy)
          for (int dz = -1; dz <= 1; ++dz)
          {
            int j = index.find(key(a.x + dx, a.y + dy, a.z + dz));
            if (j < 0||visited[j])
              continue;
            auto &b = cells[j];
            if (b.mean.x < xmin||b.mean.x > xmax||b.mean.y < ymin||b.mean.y > ymax||b.mean.z < plane.at(b.mean.x, b.mean.y) - .04||b.mean.z > h.virtual_hook.z + .2||distance(a.mean, b.mean) > .14)
              continue;
            visited[j] = 1;
            queue.push_back(j);
          }
    }
    return false;
  }
  struct Prism {
    std::vector<uint32_t>ids;
    Plane roof;
    double coverage = 0;
  };
  Prism beamBody(const std::vector<Point>&p, const std::vector<uint32_t>&input)
  {
    Prism best;
    double best_score = -1;
    std::vector<uint32_t>seed;
    for (auto i:input)
      if (!floor_mask[i])
        for (auto&h:hooks)
          if (h.direct&&std::abs(p[i].x - h.virtual_hook.x) < .2&&std::abs(p[i].y - h.virtual_hook.y) < .2&&p[i].z<h.virtual_hook.z + .05&&p[i].z>h.virtual_hook.z - c.connection_gap)
          {
            seed.push_back(i);
            break;
          }
    if (seed.size() < 8)
      return best;
    struct Bin {
      int n = 0;
      std::array<int, 32>v{};
      std::array<int, 40>w{};
      std::array<uint32_t, 40>joint{};
    };
    uint64_t random = 0x881234abcdefULL;
    auto next = [&](size_t n){
                  random ^= random << 13;
                  random ^= random >> 7;
                  random ^= random << 17;
                  return size_t(random % n);
                };
    auto quantile = [](const auto&hist, double fraction){
                      int total = 0;
                      for (auto n:hist)
                        total += n;
                      int target = std::max(1, int(std::ceil(total * fraction))), sum = 0;
                      for (size_t i = 0; i < hist.size(); ++i)
                      {
                        sum += hist[i];
                        if (sum >= target)
                          return int(i);
                      }
                      return int(hist.size() - 1);
                    };
    for (int trial = 0; trial < 24; ++trial)
    {
      const auto&a = p[seed[next(seed.size())]];
      uint32_t other = 0;
      bool found = false;
      double dx = 0, dy = 0, slope = 0;
      for (int attempt = 0; attempt < 64; ++attempt)
      {
        other = input[next(input.size())];
        auto&b = p[other];
        double length = std::hypot(b.x - a.x, b.y - a.y);
        if (length < .8||std::abs(b.z - a.z) > .577 * length)
          continue;
        dx = (b.x - a.x) / length;
        dy = (b.y - a.y) / length;
        slope = (b.z - a.z) / length;
        found = true;
        break;
      }
      if (!found)
        continue;
      double umin = 1e9, umax = -1e9;
      for (auto i:input)
      {
        double u = (p[i].x - a.x) * dx + (p[i].y - a.y) * dy;
        umin = std::min(umin, u);
        umax = std::max(umax, u);
      }
      int first = int(std::floor(umin / .2)), last = int(std::floor(umax / .2));
      if (last - first > 1000)
        continue;
      std::vector<Bin>bins(last - first + 1);
      for (auto i:input)
      {
        auto&q = p[i];
        if (floor_mask[i])
          continue;
        double u = (q.x - a.x) * dx + (q.y - a.y) * dy, v = -(q.x - a.x) * dy + (q.y - a.y) * dx, w = q.z - a.z - slope * u;
        if (std::abs(v) > .30||std::abs(w) > .35)
          continue;
        auto&b = bins[int(std::floor(u / .2)) - first];
        int iy = std::clamp(int((v + .32) / .02), 0, 31), iz = std::clamp(int((w + .4) / .02), 0, 39);
        ++b.n;
        ++b.v[iy];
        ++b.w[iz];
        b.joint[iz] |= uint32_t(1) << iy;
      }
      std::vector<uint8_t>valid(bins.size(), 0);
      std::vector<double>vc(bins.size()), wc(bins.size()), vw(bins.size()), wh(bins.size());
      for (size_t j = 0; j < bins.size(); ++j)
      {
        auto&b = bins[j];
        if (b.n < 12)
          continue;
        int v0 = quantile(b.v, .01), v1 = quantile(b.v, .99), w0 = quantile(b.w, .01), w1 = quantile(b.w, .99);
        vw[j] = (v1 - v0) * .02;
        wh[j] = (w1 - w0) * .02;
        vc[j] = (v1 + v0 + 1) * .01 - .32;
        wc[j] = (w1 + w0 + 1) * .01 - .4;
        if (vw[j] < .04||wh[j] < .10||vw[j] > .5||wh[j] > .6||std::max(vw[j], wh[j]) < .15)
          continue;
        uint32_t lower = 0, upper = 0;
        int middle = 0;
        for (int z = w0; z <= w1; ++z)
        {
          if (z <= w0 + (w1 - w0) / 4)
            lower |= b.joint[z];
          if (z >= w1 - (w1 - w0) / 4)
            upper |= b.joint[z];
          if (z > w0 + (w1 - w0) / 4&&z < w1 - (w1 - w0) / 4&&b.w[z])
            ++middle;
        }
        if (middle < 2&&(__builtin_popcount(lower) < 3||__builtin_popcount(upper) < 3))
          continue;
        valid[j] = 1;
      }
      for (size_t start = 0; start < valid.size();)
      {
        while (start < valid.size()&&!valid[start])
          ++start;
        if (start == valid.size())
          break;
        size_t end = start;
        while (end + 1 < valid.size()&&(valid[end + 1]||(end + 2 < valid.size()&&valid[end + 2])))
          ++end;
        if (end - start + 1 < 4||first + int(start) > 1||first + int(end) < -1)
        {
          start = end + 1;
          continue;
        }
        std::vector<double>width, height;
        double n = 0, su = 0, suu = 0, sv = 0, suv = 0, sw = 0, suw = 0;
        for (size_t j = start; j <= end; ++j)
          if (valid[j])
          {
            double u = (first + int(j) + .5) * .2;
            ++n;
            su += u;
            suu += u * u;
            sv += vc[j];
            suv += u * vc[j];
            sw += wc[j];
            suw += u * wc[j];
            width.push_back(vw[j]);
            height.push_back(wh[j]);
          }
        double width_med = median(width), height_med = median(height), spread = 0;
        for (auto h:height)
          spread += sq(h - height_med);
        if (std::sqrt(spread / std::max(1., n)) > .06)
        {
          start = end + 1;
          continue;
        }
        double determinant = n * suu - su * su;
        if (determinant < 1e-8)
        {
          start = end + 1;
          continue;
        }
        double kv = (n * suv - su * sv) / determinant, v0 = (sv - kv * su) / n, kw = (n * suw - su * sw) / determinant, w0 = (sw - kw * su) / n;
        double half_v = width_med / 2 + .025, half_w = height_med / 2 + .03;
        std::vector<uint32_t>fit;
        std::vector<int>counts(bins.size()), spill(bins.size());
        for (auto i:input)
        {
          auto&q = p[i];
          if (floor_mask[i])
            continue;
          double u = (q.x - a.x) * dx + (q.y - a.y) * dy, v = -(q.x - a.x) * dy + (q.y - a.y) * dx - (v0 + kv * u), w = q.z - a.z - slope * u - (w0 + kw * u);
          int b = int(std::floor(u / .2)) - first;
          if (std::abs(w) > half_w)
            continue;
          if (std::abs(v) <= half_v)
          {
            fit.push_back(i);
            ++counts[b];
          }
          else if (std::abs(v) < half_v + .10&&std::abs(v) > half_v + .025)
            ++spill[b];
        }
        bool solid = true;
        for (size_t j = start; j <= end; ++j)
          if (spill[j] > 10&&spill[j] > counts[j] / 2)
            solid = false;
        if (!solid)
        {
          start = end + 1;
          continue;
        }
        size_t left = start, right = end;
        while (left > 0&&counts[left - 1] > 0&&(spill[left - 1] <= 10||spill[left - 1] <= counts[left - 1] / 2))
          --left;
        while (right + 1 < counts.size()&&counts[right + 1] > 0&&(spill[right + 1] <= 10||spill[right + 1] <= counts[right + 1] / 2))
          ++right;
        fit.erase(std::remove_if(fit.begin(), fit.end(), [&](auto i){
            double u = (p[i].x - a.x) * dx + (p[i].y - a.y) * dy;
            int b = int(std::floor(u / .2)) - first;
            return b < int(left)||b > int(right);
          }), fit.end());
        double score = (right - left + 1) * .2 * n / (end - start + 1);
        if (fit.size() >= 80&&score > best_score)
        {
          best_score = score;
          best.ids = std::move(fit);
          best.coverage = n / (end - start + 1);
          best.roof = {dx*(slope + kw), dy*(slope + kw), 0, true};
          best.roof.c = a.z + w0 + height_med / 2 - best.roof.a * a.x - best.roof.b * a.y;
        }
        start = end + 1;
      }
    }
    return best;
  }
  std::vector<uint32_t> separatedProfile(const std::vector<Point>&p, const Prism &beam)
  {
    if (beam.ids.empty())
      return {}
    ;
    double mx = 0, my = 0, xx = 0, xy = 0, yy = 0;
    for (auto i:beam.ids)
    {
      mx += p[i].x;
      my += p[i].y;
    }
    mx /= beam.ids.size();
    my /= beam.ids.size();
    for (auto i:beam.ids)
    {
      double x = p[i].x - mx, y = p[i].y - my;
      xx += x * x;
      xy += x * y;
      yy += y * y;
    }
    double yaw = .5 * std::atan2(2 * xy, xx - yy), co = std::cos(yaw), si = std::sin(yaw);
    struct Cell {
      int v = 0, w = 0, n = 0;
      double sv = 0, sw = 0, umin = 1e9, umax = -1e9;
      uint32_t parent = 0;
    };
    std::vector<std::pair<uint64_t, uint32_t> >refs;
    refs.reserve(beam.ids.size());
    for (auto i:beam.ids)
    {
      auto&a = p[i];
      double v = -(a.x - mx) * si + (a.y - my) * co, w = a.z - beam.roof.at(a.x, a.y);
      refs.push_back({key(int(std::floor(v / .02)), int(std::floor(w / .02)), 0), i});
    }
    std::sort(refs.begin(), refs.end());
    std::vector<Cell>cs;
    FlatIndex lookup;
    lookup.clear(refs.size());
    for (size_t i = 0; i < refs.size();)
    {
      size_t end = i + 1;
      while (end < refs.size()&&refs[end].first == refs[i].first)
        ++end;
      auto&a = p[refs[i].second];
      Cell c;
      c.v = int(std::floor((-(a.x - mx) * si + (a.y - my) * co) / .02));
      c.w = int(std::floor((a.z - beam.roof.at(a.x, a.y)) / .02));
      c.parent = cs.size();
      for (size_t j = i; j < end; ++j)
      {
        auto&q = p[refs[j].second];
        double u = (q.x - mx) * co + (q.y - my) * si;
        c.n++;
        c.sv += -(q.x - mx) * si + (q.y - my) * co;
        c.sw += q.z - beam.roof.at(q.x, q.y);
        c.umin = std::min(c.umin, u);
        c.umax = std::max(c.umax, u);
      }
      lookup.insert(refs[i].first, cs.size());
      cs.push_back(c);
      i = end;
    }
    auto root = [&](uint32_t i){
                  while (cs[i].parent != i)
                  {
                    cs[i].parent = cs[cs[i].parent].parent;
                    i = cs[i].parent;
                  }
                  return i;
                };
    auto strong = [&](int i){
                    return cs[i].n >= 6&&cs[i].umax - cs[i].umin >= .5;
                  };
    for (uint32_t i = 0; i < cs.size(); ++i)
      if (strong(i))
        for (int dv = -1; dv <= 1; ++dv)
          for (int dw = -1; dw <= 1; ++dw)
          {
            int j = lookup.find(key(cs[i].v + dv, cs[i].w + dw, 0));
            if (j < 0||!strong(j))
              continue;
            if (std::hypot(cs[i].sv / cs[i].n - cs[j].sv / cs[j].n, cs[i].sw / cs[i].n - cs[j].sw / cs[j].n) <= .03)
              cs[root(i)].parent = root(j);
          }
    struct Shape {
      int count = 0, cells = 0;
      double v0 = 1e9, v1 = -1e9, w0 = 1e9, w1 = -1e9;
    };
    std::vector<Shape>shapes(cs.size());
    for (uint32_t i = 0; i < cs.size(); ++i)
      if (strong(i))
      {
        auto&s = shapes[root(i)];
        s.count += cs[i].n;
        s.cells++;
        s.v0 = std::min(s.v0, cs[i].v * .02);
        s.v1 = std::max(s.v1, (cs[i].v + 1) * .02);
        s.w0 = std::min(s.w0, cs[i].w * .02);
        s.w1 = std::max(s.w1, (cs[i].w + 1) * .02);
      }
    int chosen = -1;
    double best_gap = 1e9;
    for (uint32_t i = 0; i < shapes.size(); ++i)
    {
      auto&s = shapes[i];
      if (s.count < 80||s.cells < 6||s.v1 - s.v0 < .04||s.w1 - s.w0 < .10)
        continue;
      for (auto&h:hooks)
        if (h.direct)
        {
          double v = -(h.virtual_hook.x - mx) * si + (h.virtual_hook.y - my) * co, w = h.virtual_hook.z - beam.roof.at(h.virtual_hook.x, h.virtual_hook.y);
          double dv = std::max({s.v0 - v, 0., v - s.v1}), gap = std::hypot(dv, w - s.w1);
          if (dv <= .20&&w >= s.w1 - .1&&gap <= c.connection_gap&&gap < best_gap)
          {
            best_gap = gap;
            chosen = i;
          }
        }
    }
    if (chosen < 0)
      return beam.ids;
    std::vector<uint32_t>selected;
    for (auto [k, i]:refs)
    {
      int j = lookup.find(k);
      bool keep = strong(j)&&int(root(j)) == chosen;
      if (!keep)
        for (int dv = -1; dv <= 1&&!keep; ++dv)
          for (int dw = -1; dw <= 1; ++dw)
          {
            int n = lookup.find(key(cs[j].v + dv, cs[j].w + dw, 0));
            if (n >= 0&&strong(n)&&int(root(n)) == chosen)
            {
              keep = true;
              break;
            }
          }
      if (keep)
        selected.push_back(i);
    }
    return selected;
  }
  // Infer the low edge of a repeated measured cross-section, not its world Z
  // span. A tilted top sheet alone supplies no depth and must remain UNKNOWN.
  // This estimates a prismatic body's underside; it never claims DIRECT bottom.
  void estimateBottom(Cargo &body, const Plane &roof, const std::vector<Point>&p,
                      const std::vector<uint32_t>&ids)
  {
    body.bottom_valid = false;
    body.bottom_observation = "UNKNOWN";
    if (!roof.valid||body.length < .6)
      return;
    struct Section {
      std::vector<double> depth;
      std::vector<std::pair<uint64_t, double> >profile;
    };
    const double step = .25, co = std::cos(body.yaw), si = std::sin(body.yaw);
    const int count = std::max(3, int(std::ceil(body.length / step)));
    std::vector<Section> sections(count);
    for (auto i:ids)
    {
      const auto&a = p[i];
      double u = (a.x - body.center.x) * co + (a.y - body.center.y) * si;
      int j = std::clamp(int((u + body.length * .5) / step), 0, count - 1);
      double depth = roof.at(a.x, a.y) - a.z, v = -(a.x - body.center.x) * si + (a.y - body.center.y) * co;
      sections[j].depth.push_back(depth);
      sections[j].profile.push_back({key(int(std::floor(v / .025)), int(std::floor(depth / .025)), 0), depth});
    }
    std::vector<double>depths;
    std::vector<std::pair<int, double> >observations;
    for (int j = 0; j < count; ++j)
    {
      auto &d = sections[j].depth;
      if (d.size() < 20)
        continue;
      // Equal weight per measured surface cell; a dense top sheet must not
      // hide a sparser but spatially supported vertical side in Raw quantiles.
      auto &profile = sections[j].profile;
      std::sort(profile.begin(), profile.end());
      std::vector<double>edges;
      for (size_t begin = 0; begin < profile.size();)
      {
        size_t end = begin + 1;
        double sum = profile[begin].second;
        while (end < profile.size()&&profile[end].first == profile[begin].first)
        {
          sum += profile[end].second;
          ++end;
        }
        if (end - begin >= 2)
          edges.push_back(sum / (end - begin));
        begin = end;
      }
      if (edges.size() < 6)
        continue;
      auto it = edges.begin() + edges.size() * 19 / 20;
      std::nth_element(edges.begin(), it, edges.end());
      double depth = *it;
      if (depth < .08||depth > .60)
        continue;
      int top = 0, wall = 0, low = 0;
      for (auto z:d)
      {
        if (std::abs(z) < .04)
          ++top;
        if (z > .04&&z < depth - .04)
          ++wall;
        if (std::abs(z - depth) < .03)
          ++low;
      }
      if (top < 5||wall < 5||low < 5)
        continue;
      depths.push_back(depth);
      observations.emplace_back(j, depth);
    }
    if (depths.size() < 3)
      return;
    const double depth = median(depths);
    std::vector<double>bottoms;
    int first = count, last = -1;
    for (auto [j, d]:observations)
      if (std::abs(d - depth) <= .035)
      {
        double u = -body.length * .5 + (j + .5) * step;
        bottoms.push_back(roof.at(body.center.x + u * co, body.center.y + u * si) - depth);
        first = std::min(first, j);
        last = std::max(last, j);
      }
    if (bottoms.size() < 3||bottoms.size() < .75 * observations.size()||
        (last - first) * step < std::min(.5, .5 * body.length))
      return;
    auto it = bottoms.begin() + bottoms.size() / 10;
    std::nth_element(bottoms.begin(), it, bottoms.end());
    body.bottom_z = *it;
    body.measured_section_depth = depth;
    body.bottom_valid = true;
    body.bottom_observation = "ESTIMATED";
  }
  bool sameBody(Vec3 position, double length, double width, double yaw, const Plane*plane = nullptr)const
  {
    if (!track.id||(last_stamp - track.last_direct_sec > c.loss_sec&&last_stamp - run_start >= c.loss_sec))
      return false;
    double dt = std::max(.0, last_stamp - track.last_direct_sec), limit = std::min(.8, .06 + 1.2 * dt);
    double dz = std::abs(position.z - track.center.z);
    if (plane&&plane->valid&&tracked_plane.valid)
      dz = std::abs(plane->at(track.center.x, track.center.y) - tracked_plane.at(track.center.x, track.center.y));
    if (dz > limit)
      return false;
    double x = position.x - track.center.x, y = position.y - track.center.y;
    if (track.model_length / std::max(.05, track.model_width) > 2&&length / std::max(.05, width) > 2)
    {
      double along = x * std::cos(track.yaw) + y * std::sin(track.yaw), across = -x*std::sin(track.yaw) + y * std::cos(track.yaw);
      double angle = std::abs(std::remainder(yaw - track.yaw, 3.141592653589793));
      return angle < .35&&std::abs(across) <= std::max(.15, (track.model_width + width) * .5) + .08&&std::abs(along) < (track.model_length + length) * .5 + .25;
    }
    return std::hypot(x, y) < .5 * std::max(std::hypot(track.model_length, track.model_width), std::hypot(length, width)) + .25;
  }
  bool sameUnconfirmedFragment(const Cargo& measured, double stamp)const
  {
    const auto& r=candidate_reference;
    if(track.confirmed||!track.id||r.id!=track.id||!measured.connected||
       measured.supporting_side!=r.supporting_side||stamp-track.last_direct_sec>c.occlusion_sec)
      return false;
    if(std::abs(measured.center.z-r.center.z)>.10)return false;
    const double overlap=std::min(measured.raw_z_max,r.raw_z_max)-std::max(measured.raw_z_min,r.raw_z_min);
    if(overlap<.5*std::min(measured.raw_z_max-measured.raw_z_min,r.raw_z_max-r.raw_z_min))return false;
    const double x=measured.center.x-r.center.x,y=measured.center.y-r.center.y;
    const double along=x*std::cos(r.yaw)+y*std::sin(r.yaw);
    const double across=-x*std::sin(r.yaw)+y*std::cos(r.yaw);
    if(std::abs(along)>(r.length+measured.length)*.5+.25||
       std::abs(across)>(r.width+measured.width)*.5+.08)return false;
    return measured.length<=.65*r.length||std::abs(std::remainder(measured.yaw-r.yaw,3.141592653589793))<.35;
  }

  struct ExtentSample {
    uint32_t point = 0, cell = 0;
    int profile_v = 0, profile_w = 0;
  };
  struct ExtentBin {
    std::vector<ExtentSample>samples;
    int connected = 0, distinct = 0, foreign = 0, above = 0, spill = 0, spill_left = 0, spill_right = 0;
    double vmin = 1e9, vmax = -1e9, wmin = 1e9, wmax = -1e9, top = 0;
    double above_vmin = 1e9, above_vmax = -1e9, spill_umin = 1e9, spill_umax = -1e9;
    bool weak = false, strong = false, barrier = false;
    void clear()
    {
      samples.clear();
      connected = distinct = foreign = above = spill = spill_left = spill_right = 0;
      spill_umin = 1e9;
      spill_umax = -1e9;
      vmin = wmin = above_vmin = 1e9;
      vmax = wmax = above_vmax = -1e9;
      top = 0;
      weak = strong = barrier = false;
    }
  };
  struct ExtentEvidence {
    bool valid = false;
    uint64_t id = 0;
    Vec3 start, end, hook;
    double yaw = 0, width = 0, length = 0;
  }
  extent_evidence;
  double extent_last_seen = -1;
  std::vector<ExtentBin>extent_bins;
  std::vector<uint8_t>extent_cell_present, extent_cell_seen, extent_cell_neighbors;
  std::vector<uint32_t>extent_queue, extent_points;
  FlatIndex extent_profile_lookup, extent_seed_lookup;

  void extentDiagnostics(Cargo&b, const std::vector<uint32_t>&ids, const std::vector<Point>&p, bool partial)
  {
    if (ids.empty())
      return;
    double co = std::cos(b.yaw), si = std::sin(b.yaw), lo = 1e9, hi = -1e9, su = 0, sz = 0, suu = 0, suz = 0;
    uint32_t start = ids.front(), end = start;
    for (auto i:ids)
    {
      double u = p[i].x * co + p[i].y * si;
      su += u;
      sz += p[i].z;
      suu += u * u;
      suz += u * p[i].z;
      if (u < lo)
      {
        lo = u;
        start = i;
      }
      if (u > hi)
      {
        hi = u;
        end = i;
      }
    }
    b.observed_start = {p[start].x, p[start].y, p[start].z};
    b.observed_end = {p[end].x, p[end].y, p[end].z};
    double n = ids.size(), det = n * suu - su * su, pitch = det > 1e-8?(n * suz - su * sz) / det:0, scale = 1 / std::sqrt(1 + pitch * pitch), lo3 = 1e9, hi3 = -1e9;
    b.observed_axis = {co*scale, si*scale, pitch*scale};
    b.start_raw_support = b.end_raw_support = 0;
    for (auto i:ids)
    {
      double u = p[i].x * co + p[i].y * si;
      if (u - lo <= 2 * c.voxel_xy)
        ++b.start_raw_support;
      if (hi - u <= 2 * c.voxel_xy)
        ++b.end_raw_support;
      double along3 = scale * (u + pitch * p[i].z);
      lo3 = std::min(lo3, along3);
      hi3 = std::max(hi3, along3);
    }
    b.observed_length_3d = hi3 - lo3;
    auto clipped = [&](uint32_t i){
                     const auto&a = p[i];
                     return a.x - c.x_min < c.voxel_xy||c.x_max - a.x < c.voxel_xy||a.y - c.y_min < c.voxel_xy||c.y_max - a.y < c.voxel_xy;
                   };
    b.length_observation = partial||(track.confirmed&&track.model_length > b.length + 2 * c.voxel_xy)||b.start_raw_support < 3||b.end_raw_support < 3||clipped(start)||clipped(end)?"PARTIAL_OBSERVATION":"OBSERVED_LENGTH";
    b.model_extent_observation = track.model_length > b.length + 2 * c.voxel_xy?"ESTIMATED_EXTENT":"OBSERVED_LENGTH";
  }

  // Recover only CURRENT measured material around a selected slender body's
  // repeated cross-section. Existing voxels are scanned, never another cloud
  // clustering pass. History can identify a currently seen separated fragment;
  // it cannot contribute points or extend a measured endpoint into empty space.
  bool recoverObservedBody(Cargo&b, std::vector<uint32_t>&ids, const std::vector<Point>&p)
  {
    extentDiagnostics(b, ids, p, false);
    if (extent_last_seen >= 0&&last_stamp - extent_last_seen > .5)
      extent_evidence = {}
    ;
    extent_last_seen = last_stamp;
    if (extent_evidence.id != track.id)
      extent_evidence = {}
    ;
    if (!track.confirmed||ids.size() < 80||b.width <= 0||b.length / b.width < 4||
        (track.confirmed&&track.model_width > 0&&track.model_length / track.model_width < 4))
      return false;
    constexpr double step = .16, profile_step = .04;
    const Vec3 origin = b.center;
    const double co = std::cos(b.yaw), si = std::sin(b.yaw);
    auto along = [&](const Point&a){
                   return (a.x - origin.x) * co + (a.y - origin.y) * si;
                 };
    auto across = [&](const Point&a){
                    return -(a.x - origin.x) * si + (a.y - origin.y) * co;
                  };
    double seed_lo = 1e9, seed_hi = -1e9, seed_vlo = 1e9, seed_vhi = -1e9;
    for (auto i:ids)
    {
      double u = along(p[i]), v = across(p[i]);
      seed_lo = std::min(seed_lo, u);
      seed_hi = std::max(seed_hi, u);
      seed_vlo = std::min(seed_vlo, v);
      seed_vhi = std::max(seed_vhi, v);
    }
    const int first_seed = int(std::floor(seed_lo / step)), last_seed = int(std::floor(seed_hi / step));
    std::vector<std::vector<double> >heights(last_seed - first_seed + 1);
    std::vector<double>sum_u(heights.size());
    for (auto i:ids)
    {
      double u = along(p[i]);
      int j = int(std::floor(u / step)) - first_seed;
      heights[j].push_back(p[i].z);
      sum_u[j] += u;
    }
    std::vector<std::array<double, 2> >roof;
    for (size_t j = 0; j < heights.size(); ++j)
      if (heights[j].size() >= 12)
      {
        auto&z = heights[j];
        auto q = z.begin() + z.size() * 9 / 10;
        std::nth_element(z.begin(), q, z.end());
        roof.push_back({sum_u[j] / z.size(), *q});
      }
    if (roof.size() < 4)
      return false;
    // A single robust 1D envelope fit avoids repeating plane RANSAC. Per-section
    // weight is equal, so a dense middle cannot dominate a sparse measured end.
    double slope = 0, intercept = 0;
    std::vector<double>weights(roof.size(), 1), residuals;
    for (int iteration = 0; iteration < 4; ++iteration)
    {
      double n = 0, x = 0, z = 0, xx = 0, xz = 0;
      for (size_t i = 0; i < roof.size(); ++i)
      {
        double w = weights[i];
        n += w;
        x += w * roof[i][0];
        z += w * roof[i][1];
        xx += w * sq(roof[i][0]);
        xz += w * roof[i][0] * roof[i][1];
      }
      double det = n * xx - x * x;
      if (det < 1e-8)
        return false;
      slope = (n * xz - x * z) / det;
      intercept = (z - slope * x) / n;
      residuals.clear();
      for (auto&a:roof)
        residuals.push_back(std::abs(a[1] - slope * a[0] - intercept));
      auto sorted = residuals;
      double scale = std::max(.012, 2.5 * median(sorted));
      for (size_t i = 0; i < weights.size(); ++i)
        weights[i] = std::min(1., scale / std::max(1e-6, residuals[i]));
    }
    if (std::abs(slope) > .577)
      return false;
    struct ProfilePoint {
      uint64_t key;
      int bin;
      double v, w;
    };
    std::vector<ProfilePoint>profile;
    profile.reserve(ids.size());
    for (auto i:ids)
    {
      double u = along(p[i]), v = across(p[i]), w = p[i].z - slope * u - intercept;
      profile.push_back({key(int(std::floor(v / profile_step)), int(std::floor(w / profile_step)), 0), int(std::floor(u / step)), v, w});
    }
    std::sort(profile.begin(), profile.end(), [](auto&a, auto&b){
      return a.key < b.key||(a.key == b.key&&a.bin < b.bin);
    });
    std::vector<uint64_t>material;
    std::vector<Vec3>profile_centers;
    int pvmin = 100000, pvmax = -100000, pwmin = 100000, pwmax = -100000;
    for (size_t j = 0; j < profile.size();)
    {
      size_t end = j + 1;
      while (end < profile.size()&&profile[end].key == profile[j].key)
        ++end;
      int distinct = 1;
      for (size_t k = j + 1; k < end; ++k)
        if (profile[k].bin != profile[k - 1].bin)
          ++distinct;
      if (distinct >= std::max(3, (last_seed - first_seed + 1) / 4)&&profile[end - 1].bin - profile[j].bin >= std::max(3, (last_seed - first_seed) / 4))
      {
        auto k = profile[j].key;
        double sv = 0, sw = 0;
        for (size_t n = j; n < end; ++n)
        {
          sv += profile[n].v;
          sw += profile[n].w;
        }
        profile_centers.push_back({sv / (end - j), sw / (end - j), 0});
        material.push_back(k);
        int v = int(k >> 42) - bias, w = int((k >> 21) & ((1 << 21) - 1)) - bias;
        pvmin = std::min(pvmin, v);
        pvmax = std::max(pvmax, v);
        pwmin = std::min(pwmin, w);
        pwmax = std::max(pwmax, w);
      }
      j = end;
    }
    if (material.size() < 3||(pvmax - pvmin) * profile_step < .08)
      return false;
    extent_profile_lookup.clear(material.size());
    for (uint32_t i = 0; i < material.size(); ++i)
      extent_profile_lookup.insert(material[i], i);
    extent_seed_lookup.clear(ids.size());
    for (auto i:ids)
      extent_seed_lookup.insert(i, 0);
    double scan_lo = 1e9, scan_hi = -1e9;
    for (double x:{c.x_min, c.x_max})
      for (double y:{c.y_min, c.y_max})
      {
        double u = (x - origin.x) * co + (y - origin.y) * si;
        scan_lo = std::min(scan_lo, u);
        scan_hi = std::max(scan_hi, u);
      }
    int first = int(std::floor(scan_lo / step)) - 1, last = int(std::floor(scan_hi / step)) + 1;
    extent_bins.resize(last - first + 1);
    for (auto&bin:extent_bins)
      bin.clear();
    extent_cell_present.assign(cells.size(), 0);
    extent_cell_seen.assign(cells.size(), 0);
    extent_cell_neighbors.assign(cells.size(), 255);
    extent_queue.clear();
    double vlo = (pvmin - 1) * profile_step, vhi = (pvmax + 2) * profile_step, wlo = (pwmin - 1) * profile_step, whi = (pwmax + 2) * profile_step;
    // One contiguous CELL scan, then only Raw references in the narrow material
    // corridor. Ground ownership is always disjoint. No raw cloud is recopied.
    for (uint32_t ci = 0; ci < cells.size(); ++ci)
    {
      auto&cell = cells[ci];
      double u = (cell.mean.x - origin.x) * co + (cell.mean.y - origin.y) * si, v = -(cell.mean.x - origin.x) * si + (cell.mean.y - origin.y) * co, w = cell.mean.z - slope * u - intercept;
      if (v < vlo - .12 - c.voxel_xy||v > vhi + .12 + c.voxel_xy||w < wlo - c.voxel_z||w > std::max(.75, whi) + c.voxel_z)
        continue;
      bool seed_cell = false;
      for (uint32_t k = cell.begin; k < cell.end; ++k)
      {
        uint32_t i = entries[k].point;
        const auto&a = p[i];
        if (floor_mask[i])
          continue;
        double uu = along(a), vv = across(a), ww = a.z - slope * uu - intercept;
        int j = int(std::floor(uu / step)) - first;
        if (j < 0||j >= int(extent_bins.size()))
          continue;
        auto&bin = extent_bins[j];
        if (vv >= seed_vlo - .03&&vv <= seed_vhi + .03&&ww > .15&&ww < .75)
        {
          ++bin.above;
          bin.above_vmin = std::min(bin.above_vmin, vv);
          bin.above_vmax = std::max(bin.above_vmax, vv);
        }
        if (ww < wlo||ww > whi)
          continue;
        if (vv < vlo||vv > vhi)
        {
          if (vv > vlo - .12&&vv < vhi + .12)
          {
            ++bin.spill;
            if (vv < vlo)
              ++bin.spill_left;
            else ++bin.spill_right;
            bin.spill_umin = std::min(bin.spill_umin, uu);
            bin.spill_umax = std::max(bin.spill_umax, uu);
          }
          continue;
        }
        int pv = int(std::floor(vv / profile_step)), pw = int(std::floor(ww / profile_step));
        bool match = false;
        for (int dv = -1; dv <= 1&&!match; ++dv)
          for (int dw = -1; dw <= 1; ++dw)
          {
            int t = extent_profile_lookup.find(key(pv + dv, pw + dw, 0));
            if (t >= 0&&sq(vv - profile_centers[t].x) + sq(ww - profile_centers[t].y) <= sq(profile_step))
            {
              match = true;
              break;
            }
          }
        if (!match)
        {
          ++bin.foreign;
          continue;
        }
        bin.samples.push_back({i, ci, pv, pw});
        bin.vmin = std::min(bin.vmin, vv);
        bin.vmax = std::max(bin.vmax, vv);
        bin.wmin = std::min(bin.wmin, ww);
        bin.wmax = std::max(bin.wmax, ww);
        extent_cell_present[ci] = 1;
        if (extent_seed_lookup.find(i) >= 0)
          seed_cell = true;
      }
      if (seed_cell&&!extent_cell_seen[ci])
      {
        extent_cell_seen[ci] = 1;
        extent_queue.push_back(ci);
      }
    }
    for (size_t head = 0; head < extent_queue.size(); ++head)
    {
      auto&a = cells[extent_queue[head]];
      for (int dx = -1; dx <= 1; ++dx)
        for (int dy = -1; dy <= 1; ++dy)
          for (int dz = -1; dz <= 1; ++dz)
          {
            int j = index.find(key(a.x + dx, a.y + dy, a.z + dz));
            if (j >= 0&&extent_cell_present[j]&&!extent_cell_seen[j])
            {
              extent_cell_seen[j] = 1;
              extent_queue.push_back(j);
            }
          }
    }
    std::vector<uint64_t>occupied;
    std::vector<double>z;
    for (auto&bin:extent_bins)
    {
      occupied.clear();
      z.clear();
      for (auto&s:bin.samples)
      {
        occupied.push_back(key(s.profile_v, s.profile_w, 0));
        z.push_back(p[s.point].z - slope * along(p[s.point]) - intercept);
        bin.connected += extent_cell_seen[s.cell];
      }
      std::sort(occupied.begin(), occupied.end());
      bin.distinct = std::unique(occupied.begin(), occupied.end()) - occupied.begin();
      if (!z.empty())
      {
        auto q = z.begin() + z.size() * 9 / 10;
        std::nth_element(z.begin(), q, z.end());
        bin.top = *q;
      }
      bin.weak = bin.samples.size() >= 3&&bin.distinct >= 2;
      bin.strong = bin.samples.size() >= 8&&bin.distinct >= std::min(6, std::max(3, int(material.size() / 3)))&&bin.vmax - bin.vmin >= std::min(.06, .5 * b.width)&&bin.wmax - bin.wmin >= std::min(.06, .3 * b.height)&&std::abs(bin.top) <= 1.5 * c.voxel_z;
      bin.barrier = (bin.above >= 12&&bin.above_vmax - bin.above_vmin >= .08)||(bin.spill >= 12&&(bin.spill > int(bin.samples.size()) / 2||(bin.spill_left >= 4&&bin.spill_right >= 4)));
    }
    int side = b.supporting_side;
    bool hook_valid = side >= 0&&hooks[side].direct;
    bool historic = extent_evidence.valid&&extent_evidence.id == track.id&&track.confirmed&&hook_valid&&std::abs(std::remainder(b.yaw - extent_evidence.yaw, 3.141592653589793)) < .20;
    double hist_lo = 1e9, hist_hi = -1e9;
    if (historic)
    {
      double dx = hooks[side].virtual_hook.x - extent_evidence.hook.x, dy = hooks[side].virtual_hook.y - extent_evidence.hook.y, vlo_history = 1e9, vhi_history = -1e9;
      for (auto a:{extent_evidence.start, extent_evidence.end})
      {
        double x = a.x + dx - origin.x, y = a.y + dy - origin.y;
        double u = x * co + y * si, v = -x * si + y * co;
        hist_lo = std::min(hist_lo, u);
        hist_hi = std::max(hist_hi, u);
        vlo_history = std::min(vlo_history, v);
        vhi_history = std::max(vhi_history, v);
      }
      double overlap = std::min(seed_vhi, vhi_history + .5 * extent_evidence.width) - std::max(seed_vlo, vlo_history - .5 * extent_evidence.width);
      if (overlap < std::min(.5 * c.voxel_xy, .3 * std::min(b.width, extent_evidence.width)))
        historic = false;
    }
    int accepted_left = first_seed, accepted_right = last_seed;
    bool bridged = false;
    auto walk = [&](int direction){
                  int accepted = direction > 0?last_seed:first_seed;
                  int j = accepted;
                  int weak_run = 0;
                  for (; j >= first&&j <= last; j += direction)
                  {
                    auto&bin = extent_bins[j - first];
                    bool outside = direction > 0?j > last_seed:j < first_seed;
                    if (outside&&bin.barrier)
                      break;
                    if (outside&&bin.foreign >= 20&&bin.foreign > 3 * int(bin.samples.size()))
                      break;
                    bool connected = bin.connected >= std::min(3, int(bin.samples.size()));
                    if (bin.weak&&(connected||!outside))
                    {
                      if (bin.strong)
                      {
                        accepted = j;
                        weak_run = 0;
                      }
                      else if (++weak_run <= 1&&bin.distinct >= 3&&bin.vmax - bin.vmin >= std::min(.06, .5 * b.width))
                        accepted = j;
                      continue;
                    }
                    // Cross an occlusion only inside a previously DIRECT measured footprint,
                    // with a substantial current matching far surface and no intervening
                    // incompatible object. A distance alone cannot authorize a connection.
                    if (!historic)
                      break;
                    double edge = (j + (direction > 0?1.:0.)) * step;
                    if (edge < hist_lo - 2 * c.voxel_xy||edge > hist_hi + 2 * c.voxel_xy)
                      break;
                    int q = j, first_strong = 0, strong_count = 0, end_strong = j;
                    bool blocked = false;
                    for (; q >= first&&q <= last; q += direction)
                    {
                      auto&far = extent_bins[q - first];
                      double pos = (q + .5) * step;
                      if (pos < hist_lo - 2 * c.voxel_xy||pos > hist_hi + 2 * c.voxel_xy||far.barrier||(far.foreign >= 20&&far.foreign > 3 * int(far.samples.size())))
                      {
                        if (!strong_count)
                          blocked = true;
                        break;
                      }
                      if (far.strong)
                      {
                        if (!strong_count)
                          first_strong = q;
                        ++strong_count;
                        end_strong = q;
                      }
                      else if (strong_count)
                        break;
                    }
                    double gap = std::abs(first_strong - j) * step, span = strong_count * step;
                    if (blocked||strong_count < 3||gap > .5 * std::min(b.length, span))
                      break;
                    accepted = end_strong;
                    j = end_strong;
                    bridged = true;
                    weak_run = 0;
                  }
                  return accepted;
                };
    accepted_left = walk(-1);
    accepted_right = walk(1);
    // Record measured, connected seed extent only. A recovered gap never grows
    // this evidence and no held/model-only span can become historical Raw proof.
    bool continuous = true;
    int strong_seed = 0;
    for (int j = first_seed + 1; j < last_seed; ++j)
    {
      const auto&bin = extent_bins[j - first];
      if (!bin.weak||bin.connected < 3)
        continuous = false;
      if (bin.strong)
        ++strong_seed;
    }
    auto remember = [&](){
                      if (continuous&&strong_seed >= 3&&hook_valid&&!bridged&&(!extent_evidence.valid||b.length >= extent_evidence.length - 2 * c.voxel_xy))
                      {
                        // This is the footprint of measured Raw, not a raw endpoint or a generated
                        // body point. Surface extrema lie on an edge and must not be treated as
                        // the central axis when checking another partial view of the same body.
                        double bc = std::cos(b.yaw), bs = std::sin(b.yaw), u0 = 1e9, u1 = -1e9, v0 = 1e9, v1 = -1e9;
                        for (auto i:ids)
                        {
                          double u = p[i].x * bc + p[i].y * bs, v = -p[i].x * bs + p[i].y * bc;
                          u0 = std::min(u0, u);
                          u1 = std::max(u1, u);
                          v0 = std::min(v0, v);
                          v1 = std::max(v1, v);
                        }
                        double vm = .5 * (v0 + v1);
                        Vec3 start{u0*bc - vm*bs, u0*bs + vm*bc, 0}, end{u1*bc - vm*bs, u1*bs + vm*bc, 0};
                        extent_evidence = {true, track.id, start, end, hooks[side].virtual_hook, b.yaw, b.width, b.length};
                      }
                    };
    extent_points = ids;
    for (int j = accepted_left; j <= accepted_right; ++j)
    {
      const auto&bin = extent_bins[j - first];
      // The endpoint bin may also contain a vertical structure. Preserve the
      // original seed but do not add material from that ambiguous contact bin.
      if (bin.barrier)
        continue;
      for (auto&s:bin.samples)
      {
        double u = along(p[s.point]);
        if (u >= seed_lo&&u <= seed_hi)
          continue;
        const auto&cell = cells[s.cell];
        int neighbors = extent_cell_neighbors[s.cell];
        if (neighbors == 255)
        {
          neighbors = 0;
          for (int dx = -1; dx <= 1; ++dx)
            for (int dy = -1; dy <= 1; ++dy)
              for (int dz = -1; dz <= 1; ++dz)
              {
                if (!dx&&!dy&&!dz)
                  continue;
                int k = index.find(key(cell.x + dx, cell.y + dy, cell.z + dz));
                if (k >= 0&&extent_cell_present[k])
                  ++neighbors;
              }
          extent_cell_neighbors[s.cell] = neighbors;
        }
        if (neighbors >= 2||(neighbors >= 1&&cell.end - cell.begin >= 2))
          extent_points.push_back(s.point);
      }
    }
    // An independent tall structure at a measured end is not an end cap of
    // the repeated extrusion. Exclude that ambiguous contact bin, while a
    // directly measured supporting Hook at the end is an expected attachment.
    double safe_lo = -1e9, safe_hi = 1e9;
    bool trimmed = false;
    auto supporting_hook_at = [&](int j){
                                if (!hook_valid)
                                  return false;
                                const auto &h = hooks[side];
                                double u = (h.virtual_hook.x - origin.x) * co + (h.virtual_hook.y - origin.y) * si, v = -(h.virtual_hook.x - origin.x) * si + (h.virtual_hook.y - origin.y) * co;
                                return std::abs(u - (j + .5) * step) < step&&std::abs(v) < .2&&h.virtual_hook.z >= slope * u + intercept - .05;
                              };
    if (extent_bins[first_seed - first].barrier&&!supporting_hook_at(first_seed))
    {
      safe_lo = (first_seed + 1) * step;
      trimmed = true;
    }
    if (extent_bins[last_seed - first].barrier&&!supporting_hook_at(last_seed))
    {
      safe_hi = last_seed * step;
      trimmed = true;
    }
    // A terminal widening can already have entered the seed component. Locate
    // the start of its contiguous side spill rather than trimming only its last
    // voxel. A genuine constant-section body is retained up to that boundary.
    auto widening = [&](int j){
                      const auto&bin = extent_bins[j - first];
                      return bin.spill >= 12&&(bin.spill > int(bin.samples.size()) / 2||(bin.spill_left >= 4&&bin.spill_right >= 4));
                    };
    int left_change = first_seed - 1, right_change = last_seed + 1;
    double left_boundary = -1e9, right_boundary = 1e9;
    for (int j = first_seed; j <= last_seed; ++j)
    {
      if (!widening(j)||supporting_hook_at(j))
        break;
      left_change = j;
      left_boundary = std::max(left_boundary, extent_bins[j - first].spill_umax);
    }
    for (int j = last_seed; j >= first_seed; --j)
    {
      if (!widening(j)||supporting_hook_at(j))
        break;
      right_change = j;
      right_boundary = std::min(right_boundary, extent_bins[j - first].spill_umin);
    }
    auto clear_interior = [&](int j, int direction){
                            for (int n = 0; n < 3; ++n, j += direction)
                            {
                              if (j < first_seed||j > last_seed||widening(j)||!extent_bins[j - first].strong)
                                return false;
                            }
                            return true;
                          };
    if (left_change >= first_seed&&clear_interior(left_change + 1, 1))
    {
      safe_lo = left_boundary + .5 * profile_step;
      trimmed = true;
    }
    if (right_change <= last_seed&&clear_interior(right_change - 1, -1))
    {
      safe_hi = right_boundary - .5 * profile_step;
      trimmed = true;
    }
    extent_points.erase(std::remove_if(extent_points.begin(), extent_points.end(), [&](auto i){
      double u = along(p[i]);
      return u < safe_lo||u > safe_hi;
    }), extent_points.end());
    // Fine, spatially supported endpoint bands retain sparse real faces but do
    // not let one far point or a thin attached wire define the body's length.
    if (!extent_points.empty())
    {
      constexpr double end_step = .02;
      double lo = 1e9, hi = -1e9;
      for (auto i:extent_points)
      {
        double u = along(p[i]);
        lo = std::min(lo, u);
        hi = std::max(hi, u);
      }
      int first_end = int(std::floor(lo / end_step)), last_end = int(std::floor(hi / end_step));
      struct EndBand {
        int n = 0;
        double v0 = 1e9, v1 = -1e9, w0 = 1e9, w1 = -1e9;
      };
      std::vector<EndBand>ends(last_end - first_end + 1);
      for (auto i:extent_points)
      {
        double u = along(p[i]), v = across(p[i]), w = p[i].z - slope * u - intercept;
        auto &band = ends[int(std::floor(u / end_step)) - first_end];
        ++band.n;
        band.v0 = std::min(band.v0, v);
        band.v1 = std::max(band.v1, v);
        band.w0 = std::min(band.w0, w);
        band.w1 = std::max(band.w1, w);
      }
      int left = -1, right = -1;
      for (int j = 0; j < int(ends.size()); ++j)
      {
        auto &band = ends[j];
        if (band.n >= 3&&(band.v1 - band.v0 >= std::min(.06, .5 * b.width)||band.w1 - band.w0 >= std::min(.06, .3 * b.height)))
        {
          if (left < 0)
            left = j;
          right = j;
        }
      }
      if (left >= 0&&right > left)
      {
        extent_points.erase(std::remove_if(extent_points.begin(), extent_points.end(), [&](auto i){
          int j = int(std::floor(along(p[i]) / end_step)) - first_end;
          return j < left||j > right;
        }), extent_points.end());
      }
    }
    if (extent_points.size() < 80)
    {
      b.length_observation = "PARTIAL_OBSERVATION";
      return false;
    }
    std::sort(extent_points.begin(), extent_points.end());
    extent_points.erase(std::unique(extent_points.begin(), extent_points.end()), extent_points.end());
    auto original = ids;
    std::sort(original.begin(), original.end());
    if (extent_points == original)
    {
      remember();
      return false;
    }
    uint32_t added = 0, removed = 0;
    for (auto i:extent_points)
      if (!std::binary_search(original.begin(), original.end(), i))
        ++added;
    for (auto i:original)
      if (!std::binary_search(extent_points.begin(), extent_points.end(), i))
        ++removed;
    ids = extent_points;
    double mx = 0, my = 0, mz = 0, zmin = 1e9, zmax = -1e9, xx = 0, xy = 0, yy = 0;
    for (auto i:ids)
    {
      const auto&a = p[i];
      mx += a.x;
      my += a.y;
      mz += a.z;
      zmin = std::min(zmin, double(a.z));
      zmax = std::max(zmax, double(a.z));
    }
    mx /= ids.size();
    my /= ids.size();
    mz /= ids.size();
    for (auto i:ids)
    {
      xx += sq(p[i].x - mx);
      xy += (p[i].x - mx) * (p[i].y - my);
      yy += sq(p[i].y - my);
    }
    double yaw = .5 * std::atan2(2 * xy, xx - yy), ux = std::cos(yaw), uy = std::sin(yaw), umin = 1e9, umax = -1e9, vmin = 1e9, vmax = -1e9;
    for (auto i:ids)
    {
      double u = (p[i].x - mx) * ux + (p[i].y - my) * uy, v = -(p[i].x - mx) * uy + (p[i].y - my) * ux;
      umin = std::min(umin, u);
      umax = std::max(umax, u);
      vmin = std::min(vmin, v);
      vmax = std::max(vmax, v);
    }
    b.center = {mx, my, mz};
    b.yaw = yaw;
    b.length = umax - umin;
    b.width = vmax - vmin;
    b.height = zmax - zmin;
    b.raw_z_min = zmin;
    b.raw_z_max = zmax;
    b.raw_indices.clear();
    b.raw_indices.reserve(ids.size());
    for (auto i:ids)
      b.raw_indices.push_back(p[i].raw);
    b.extent_added_points = added;
    b.extent_removed_points = removed;
    b.extent_gap_recovered = bridged;
    extentDiagnostics(b, ids, p, bridged||trimmed);
    remember();
    return true;
  }

  Cargo detect(const std::vector<Point>&p, const Ground&g, Output&o)
  {
    body_refs.clear();
    motion_refs.clear();
    for (uint32_t i = 0; i < cells.size(); ++i)
      cells[i].parent = i;
    std::vector<uint8_t>body(cells.size(), 0);
    for (size_t i = 0; i < cells.size(); ++i)
    {
      const auto &v = cells[i];
      if (v.end - v.begin < 2||(g.valid&&v.mean.z <= g.z + .07))
        continue;
      int lateral = 0;
      for (int dx = -1; dx <= 1; ++dx)
        for (int dy = -1; dy <= 1; ++dy)
        {
          if (dx == 0&&dy == 0)
            continue;
          int j = index.find(key(v.x + dx, v.y + dy, v.z));
          if (j >= 0&&cells[j].end - cells[j].begin >= 2)
            ++lateral;
        }
      body[i] = lateral >= 2;
    }
    // 26-neighbour raw-supported voxel graph, one pass over active cells.
    for (uint32_t i = 0; i < cells.size(); ++i)
      if (body[i])
      {
        auto&v = cells[i];
        for (int dx = -1; dx <= 1; ++dx)
          for (int dy = -1; dy <= 1; ++dy)
            for (int dz = -1; dz <= 1; ++dz)
            {
              if (dx > 0||(dx == 0&&dy > 0)||(dx == 0&&dy == 0&&dz >= 0))
                continue;
              int j = index.find(key(v.x + dx, v.y + dy, v.z + dz));
              if (j >= 0&&body[j])
                join(i, j);
            }
      }
    surfaces.clear();
    root_surface.assign(cells.size(), -1);
    for (uint32_t i = 0; i < cells.size(); ++i)
      if (body[i])
      {
        uint32_t r = root(i);
        if (root_surface[r] < 0)
        {
          root_surface[r] = surfaces.size();
          Surface s;
          s.root = r;
          surfaces.push_back(s);
        }
        auto&s = surfaces[root_surface[r]];
        for (uint32_t j = cells[i].begin; j < cells[i].end; ++j)
        {
          auto&a = p[entries[j].point];
          ++s.count;
          s.sx += a.x;
          s.sy += a.y;
          s.sz += a.z;
          s.xx += a.x * a.x;
          s.xy += a.x * a.y;
          s.yy += a.y * a.y;
          s.xmin = std::min(s.xmin, double(a.x));
          s.xmax = std::max(s.xmax, double(a.x));
          s.ymin = std::min(s.ymin, double(a.y));
          s.ymax = std::max(s.ymax, double(a.y));
          s.zmin = std::min(s.zmin, double(a.z));
          s.zmax = std::max(s.zmax, double(a.z));
        }
      }
    o.components = surfaces.size();
    Cargo best;
    double best_score = -1e9;
    for (auto&s:surfaces)
    {
      if (s.count < 80)
        continue;
      std::array<double, 21>debug{};
      debug[0] = s.root;
      debug[2] = s.count;
      auto record = [&](int reason){
#ifdef CRANE_V2_TRACE
                      debug[1] = reason;
                      o.candidate_metrics.push_back(debug);
#else
                      (void)reason;
#endif
                    };
      bool possible = false;
      for (auto &h:hooks)
        if (h.direct&&h.virtual_hook.x >= s.xmin - .15&&h.virtual_hook.x <= s.xmax + .15&&h.virtual_hook.y >= s.ymin - .15&&h.virtual_hook.y <= s.ymax + .15)
          possible = true;
      if (track.id&&(last_stamp - track.last_direct_sec <= c.loss_sec||last_stamp - run_start < c.loss_sec)&&track.center.x >= s.xmin - 1&&track.center.x <= s.xmax + 1&&track.center.y >= s.ymin - 1&&track.center.y <= s.ymax + 1)
        possible = true;
      if (!possible)
      {
        record(8);
        continue;
      }
      double mx = s.sx / s.count, my = s.sy / s.count;
      double xx = s.xx / s.count - mx * mx, xy = s.xy / s.count - mx * my, yy = s.yy / s.count - my * my;
      double yaw = .5 * std::atan2(2 * xy, xx - yy), ux = std::cos(yaw), uy = std::sin(yaw);
      double umin = 1e9, umax = -1e9, vmin = 1e9, vmax = -1e9;
      std::vector<uint32_t>ids;
      std::vector<double>zs;
      std::array<int, 2>connection_points{};
      std::array<double, 2>gaps{1e9, 1e9};
      for (uint32_t i = 0; i < cells.size(); ++i)
        if (body[i]&&root(i) == s.root)
          for (uint32_t j = cells[i].begin; j < cells[i].end; ++j)
          {
            uint32_t id = entries[j].point;
            auto&a = p[id];
            double u = (a.x - mx) * ux + (a.y - my) * uy, v = -(a.x - mx) * uy + (a.y - my) * ux;
            umin = std::min(umin, u);
            umax = std::max(umax, u);
            vmin = std::min(vmin, v);
            vmax = std::max(vmax, v);
            ids.push_back(id);
            zs.push_back(a.z);
            for (int side = 0; side < 2; ++side)
            {
              const auto&h = hooks[side];
              if (h.direct&&std::abs(a.x - h.virtual_hook.x) <= .15&&std::abs(a.y - h.virtual_hook.y) <= .15&&a.z < h.virtual_hook.z + .05)
              {
                ++connection_points[side];
                gaps[side] = std::min(gaps[side], h.virtual_hook.z - a.z);
              }
            }
          }
      auto component_ids = std::move(ids);
      for (int primitive = 0; primitive < 2; ++primitive)
      {
        ids = component_ids;
        debug[20] = primitive;
        double beam_coverage = 0;
        // Separate the broad horizontal body from narrow high rigging that can
        // touch it in a 3D component. The selected points remain current raw data.
        Plane plane;
        if (primitive == 0)
          plane = bodyPlane(p, ids);
        else{
          auto beam = beamBody(p, ids);
          if (beam.ids.empty())
          {
            record(9);
            continue;
          }
          ids = separatedProfile(p, beam);
          plane = beam.roof;
          beam_coverage = beam.coverage;
        }
        if (!plane.valid)
        {
          record(1);
          continue;
        }
        debug[10] = plane.a;
        debug[11] = plane.b;
        debug[12] = plane.c;
        std::vector<uint64_t>footprint;
        if (primitive == 0)
          footprint = bodySheet(p, ids, plane);
        else{
          for (auto i:ids)
            footprint.push_back(key(int(std::floor(p[i].x / .04)), int(std::floor(p[i].y / .04)), 0));
          std::sort(footprint.begin(), footprint.end());
          footprint.erase(std::unique(footprint.begin(), footprint.end()), footprint.end());
        }
        if (footprint.empty())
        {
          record(7);
          continue;
        }
        ids.erase(std::remove_if(ids.begin(), ids.end(), [&](uint32_t i){
            return !std::binary_search(footprint.begin(), footprint.end(), key(int(std::floor(p[i].x / .04)), int(std::floor(p[i].y / .04)), 0))||floor_mask[i]||p[i].z > plane.at(p[i].x, p[i].y) + .10||p[i].z < plane.at(p[i].x, p[i].y) - .60;
          }), ids.end());
        if (ids.size() < 80)
        {
          record(2);
          continue;
        }
        mx = my = 0;
        double mz = 0, zmin = 1e9, zmax = -1e9;
        for (auto i:ids)
        {
          mx += p[i].x;
          my += p[i].y;
          mz += p[i].z;
          zmin = std::min(zmin, double(p[i].z));
          zmax = std::max(zmax, double(p[i].z));
        }
        mx /= ids.size();
        my /= ids.size();
        mz /= ids.size();
        xx = xy = yy = 0;
        for (auto i:ids)
        {
          xx += sq(p[i].x - mx);
          xy += (p[i].x - mx) * (p[i].y - my);
          yy += sq(p[i].y - my);
        }
        yaw = .5 * std::atan2(2 * xy, xx - yy);
        ux = std::cos(yaw);
        uy = std::sin(yaw);
        umin = vmin = 1e9;
        umax = vmax = -1e9;
        zs.clear();
        connection_points = {};
        gaps = {1e9, 1e9};
        std::vector<uint64_t>occupied_xy;
        occupied_xy.reserve(ids.size());
        for (auto i:ids)
        {
          auto&a = p[i];
          double u = (a.x - mx) * ux + (a.y - my) * uy, v = -(a.x - mx) * uy + (a.y - my) * ux;
          umin = std::min(umin, u);
          umax = std::max(umax, u);
          vmin = std::min(vmin, v);
          vmax = std::max(vmax, v);
          zs.push_back(a.z);
          if (std::abs(a.z - plane.at(a.x, a.y)) < .03)
            occupied_xy.push_back(key(int(std::floor(a.x / .04)), int(std::floor(a.y / .04)), 0));
          for (int side = 0; side < 2; ++side)
          {
            auto&h = hooks[side];
            if (h.direct&&std::abs(a.x - h.virtual_hook.x) <= .15&&std::abs(a.y - h.virtual_hook.y) <= .15&&a.z < h.virtual_hook.z + .05)
            {
              ++connection_points[side];
              gaps[side] = std::min(gaps[side], h.virtual_hook.z - a.z);
            }
          }
        }
        std::sort(occupied_xy.begin(), occupied_xy.end());
        size_t occupied = std::unique(occupied_xy.begin(), occupied_xy.end()) - occupied_xy.begin();
        double length = umax - umin, width = vmax - vmin, height = zmax - zmin;
        debug[3] = mx;
        debug[4] = my;
        debug[5] = mz;
        debug[6] = length;
        debug[7] = width;
        debug[8] = height;
        bool existing = sameBody(Vec3{mx, my, mz}, length, width, yaw, &plane);
        double fill = occupied * .04 * .04 / std::max(.001, length * width);
        if (primitive == 1)
          fill = beam_coverage;
        debug[9] = fill;
        if (fill < ((existing&&track.confirmed)?.30:.55))
        {
          record(3);
          continue;
        }
        // Body area and width reject wire/sling; calibrated Ground is disjoint.
        bool compact_body=primitive==0&&length>=4*c.voxel_xy&&width>=.20&&length*width>=.10&&fill>=.75&&height>=.08&&(!track.id||track.model_length<.8||last_stamp-track.last_direct_sec>c.loss_sec);
        if ((length<.8&&!compact_body)||(primitive == 0?width < .15:width < .04)||width > 4||height > 2||length > 45||ids.size() * c.voxel_xy * c.voxel_xy / (length * width) < .5)
        {
          record(4);
          continue;
        }
        ++o.candidates;
        Vec3 pos{mx, my, mz};
        bool same = track.confirmed&&sameBody(pos, length, width, yaw, &plane);
        int side = connection_points[0] >= connection_points[1]?0:1;
        bool connected = connection_points[side] >= 8&&gaps[side] <= c.connection_gap&&observedConnection(side, plane, footprint);
        debug[15] = connection_points[0];
        debug[16] = connection_points[1];
        debug[17] = gaps[0];
        debug[18] = gaps[1];
        if (!connected&&!same)
        {
          record(5);
          continue;
        }
        if (armed&&track.confirmed&&!same)
        {
          record(10);
          continue;
        }
        double reference_change = track.confirmed&&tracked_plane.valid?std::abs(plane.at(track.center.x, track.center.y) - tracked_plane.at(track.center.x, track.center.y)):0;
        double score = std::log(double(ids.size())) + (connected?5:0) + (same?4:0) - reference_change / .05;
        debug[19] = score;
        if (score <= best_score)
        {
          record(6);
          break;
        }
        record(0);
        best_score = score;
        selected_plane = plane;
        Cargo b;
        b.direct = true;
        b.connected = connected;
        b.supporting_side = connected?side:track.supporting_side;
        b.center = pos;
        b.length = length;
        b.width = width;
        b.height = height;
        b.yaw = yaw;
        for (auto id:ids)
          b.raw_indices.push_back(p[id].raw);
        b.raw_z_min = zmin;
        b.raw_z_max = zmax;
        estimateBottom(b, plane, p, ids);
        best = std::move(b);
        body_refs = ids;
        motion_refs.clear();
        for (auto i:ids)
          if (std::abs(p[i].z - plane.at(p[i].x, p[i].y)) < .03)
            motion_refs.push_back(i);
        break;
      }
    }
    if(best.direct&&recoverObservedBody(best,body_refs,p)){
      estimateBottom(best,selected_plane,p,body_refs);
      motion_refs.clear();
      for(auto i:body_refs)if(std::abs(p[i].z-selected_plane.at(p[i].x,p[i].y))<.03)motion_refs.push_back(i);
    }
    return best;
  }
  bool currentGroundSupport(const std::vector<Point>&p, const Ground&g)
  {
    support_paths.clear();
    if (!track.direct||!track.bottom_valid||!g.valid||track.bottom_z - g.z < .10)
      return false;
    std::vector<uint64_t>columns;
    std::vector<uint32_t>body_sorted = body_refs;
    std::sort(body_sorted.begin(), body_sorted.end());
    for (auto i:body_refs)
    {
      auto&a = p[i];
      if (a.z <= track.bottom_z + .04)
        columns.push_back(key(int(std::floor(a.x / (2 * c.voxel_xy))), int(std::floor(a.y / (2 * c.voxel_xy))), 0));
    }
    std::sort(columns.begin(), columns.end());
    columns.erase(std::unique(columns.begin(), columns.end()), columns.end());
    std::vector<Vec3>supports;
    std::vector<uint32_t>refs;
    for (auto k:columns)
    {
      int x = int(k >> 42) - bias, y = int((k >> 21) & ((1 << 21) - 1)) - bias;
      int floors = 0;
      refs.clear();
      int z0 = int(std::floor((g.z - .025) / c.voxel_z)), z1 = int(std::floor((track.bottom_z + .04) / c.voxel_z));
      for (int dx = 0; dx < 2; ++dx)
        for (int dy = 0; dy < 2; ++dy)
          for (int z = z0; z <= z1; ++z)
          {
            int cell = index.find(key(x * 2 + dx, y * 2 + dy, z));
            if (cell < 0)
              continue;
            for (uint32_t j = cells[cell].begin; j < cells[cell].end; ++j)
            {
              uint32_t id = entries[j].point;
              auto&a = p[id];
              if (a.z < g.z - .025||a.z > track.bottom_z + .04)
                continue;
              refs.push_back(id);
              if (floor_mask[id])
                ++floors;
            }
          }
      if (floors < 3||refs.size() < 12)
        continue;
      std::sort(refs.begin(), refs.end(), [&](auto a, auto b){
          return p[a].z < p[b].z;
        });
      std::vector<int>parent(refs.size(), -1);
      int endpoint = -1;
      for (size_t i = 0; i < refs.size(); ++i)
      {
        auto&a = p[refs[i]];
        if (floor_mask[refs[i]])
        {
          parent[i] = -2;
          continue;
        }
        for (int j = int(i) - 1; j >= 0; --j)
        {
          auto &b = p[refs[j]];
          if (a.z - b.z > .04)
            break;
          if (parent[j] == -1)
            continue;
          if (sq(a.x - b.x) + sq(a.y - b.y) + sq(a.z - b.z) <= sq(.055))
          {
            parent[i] = j;
            break;
          }
        }
        if (parent[i] >= 0&&a.z >= track.bottom_z - .04&&std::binary_search(body_sorted.begin(), body_sorted.end(), refs[i]))
        {
          endpoint = i;
          break;
        }
      }
      if (endpoint < 0)
        continue;
      auto &end = p[refs[endpoint]];
      Vec3 position{end.x, end.y, 0};
      for (int i = endpoint; i >= 0; i = parent[i])
        support_paths.push_back(p[refs[i]].raw);
      for (auto&a:supports)
        if (distance(position, a) >= .25)
          return true;
      supports.push_back(position);
    }
    support_paths.clear();
    return false;
  }
  void update(Cargo measured, const std::vector<Point>&points, Output&o, double stamp, bool continuous)
  {
    if (!continuous)
    {
      motion.clear();
      last_patches.clear();
      contact_since = detach_since = -1;
      run_start = stamp;
    }
    const Plane previous_plane = tracked_plane;
    const Cargo previous_track = track;
    if (measured.direct)
    {
      bool same = sameBody(measured.center, measured.length, measured.width, measured.yaw, &selected_plane)||
                  sameUnconfirmedFragment(measured,stamp);
      if (!same)
      {
        track = {};
        track.id = next_id++;
        first_seen = stamp;
        confirmed_direct_duration = 0;
        last_connected_confirmation = -1;
        candidate_reference = {};
        last_ground_support = -1;
        last_patches.clear();
        motion.clear();
        completion_consumed = false;
      }
      measured.id = track.id;
      if (!track.confirmed&&(!same||!continuous||last_connected_confirmation<0||
                            stamp-last_connected_confirmation>c.occlusion_sec))
        confirmed_direct_duration = 0;
      if (same&&continuous&&track.direct&&track.connected&&measured.connected)
      {
        confirmed_direct_duration += stamp - track.last_direct_sec;
      }
      if(measured.connected)last_connected_confirmation=stamp;
      measured.confirmed = track.confirmed||confirmed_direct_duration + 1e-8 >= c.confirm_sec;
      measured.model_length = std::max(track.model_length, measured.length);
      measured.model_width = std::max(track.model_width, measured.width);
      measured.last_direct_sec = stamp;
      track = std::move(measured);
      if(track.connected&&(candidate_reference.id!=track.id||track.length>candidate_reference.length)) {
        candidate_reference=track;candidate_reference.raw_indices.clear();
      }
      tracked_plane = selected_plane;
    }
    else{
      track.direct = false;
      track.held = track.id&&stamp - track.last_direct_sec <= c.occlusion_sec;
      track.connected = false;
      track.raw_indices.clear();
      track.bottom_valid = false;
      track.bottom_observation = "UNKNOWN";
    }
    o.cargo = track;
    // Floor support local to the observed Cargo, independent of Cargo ownership.
    if (track.direct&&o.ground.valid)
    {
      std::vector<double>zs;
      double minx = 1e9, maxx = -1e9, miny = 1e9, maxy = -1e9;
      for (auto i:ground_points)
      {
        auto&p = points[i];
        if (std::abs(p.x - track.center.x) < std::max(1.5, track.length * .7)&&std::abs(p.y - track.center.y) < std::max(1.5, track.width))
        {
          zs.push_back(p.z);
          minx = std::min(minx, double(p.x));
          maxx = std::max(maxx, double(p.x));
          miny = std::min(miny, double(p.y));
          maxy = std::max(maxy, double(p.y));
        }
      }
      if (zs.size() >= 50&&maxx - minx > .5&&maxy - miny > .5)
      {
        o.ground.z = median(zs);
        o.ground.observation = "LOCAL_PLANE";
        o.clearance_valid = track.bottom_valid;
        o.clearance = track.bottom_z - o.ground.z;
      }
      else{
        o.ground.valid = false;
        o.ground.observation = "UNKNOWN";
      }
    }
    // Match current raw patches after compensating measured XY translation. No
    // centroid Z difference, Hook motion or held position creates Cargo motion.
    std::vector<Patch>patches;
    if (track.direct)
    {
      std::unordered_map<uint64_t, std::pair<double, int> >bins;
      // Use current Cargo's actual raw IDs; raw IDs need not be dense after filtering.
      double co = std::cos(track.yaw), si = std::sin(track.yaw);
      for (auto id:motion_refs)
      {
        const auto &p = points[id];
        int x = int(std::floor(((p.x - track.center.x) * co + (p.y - track.center.y) * si) / .20)), y = int(std::floor((-(p.x - track.center.x) * si + (p.y - track.center.y) * co) / .20));
        auto&b = bins[key(x, y, 0)];
        b.first += p.z;
        ++b.second;
      }
      for (auto&kv:bins)
        if (kv.second.second >= 5)
        {
          uint64_t k = kv.first;
          patches.push_back({int(k >> 42) - bias, int((k >> 21) & ((1 << 21) - 1)) - bias, kv.second.first / kv.second.second});
        }
      std::vector<double>deltas;
      for (auto&a:patches)
        for (auto&b:last_patches)
          if (a.x == b.x&&a.y == b.y)
          {
            deltas.push_back(a.z - b.z);
            break;
          }
      o.matched_fraction = patches.empty()?0:double(deltas.size()) / patches.size();
      if (continuous&&deltas.size() >= 4&&o.matched_fraction >= .5)
      {
        double dz = median(deltas);
        int agree = std::count_if(deltas.begin(), deltas.end(), [&](double v){
            return std::abs(v - dz) < .025;
          });
        // Object-relative patches change identity when a tilted body is
        // progressively occluded. Require the independently measured planes to
        // move in the same direction at the SAME world XY in that case.
        const bool changing_footprint = previous_track.id == track.id&&
                                        (track.length < .95 * track.model_length||previous_track.length < .95 * track.model_length||
                                         track.width<.95 * track.model_width||previous_track.width<.95 * track.model_width||
                                                                                                   std::abs(track.length - previous_track.length)>.02 * std::max(.1, previous_track.length)||
                                                     std::abs(track.width - previous_track.width)>.02 * std::max(.1, previous_track.width));
        double plane_dz = selected_plane.at(track.center.x, track.center.y) - previous_plane.at(track.center.x, track.center.y);
        const bool plane_agrees = !changing_footprint||std::abs(dz) <= .002||
                                  (previous_plane.valid&&selected_plane.valid&&dz * plane_dz > 0&&
                                   std::abs(plane_dz) >= std::max(.002, .3 * std::abs(dz)));
        if (agree >= int(deltas.size() * .7)&&plane_agrees)
          motion.push_back({stamp, dz});
        else motion.clear();
      }
      else motion.clear();
      last_patches = std::move(patches);
    }
    else{
      last_patches.clear();
      motion.clear();
    }
    motion.erase(std::remove_if(motion.begin(), motion.end(), [&](auto&s){
        return stamp - s.stamp > c.slow_motion_window;
      }), motion.end());
    double dz = 0, slow_dz = 0;
    int up_steps = 0, down_steps = 0, slow_up = 0, slow_down = 0;
    for (auto &sample:motion)
    {
      slow_dz += sample.dz;
      if (sample.dz > .0005)
        ++slow_up;
      if (sample.dz < -.0005)
        ++slow_down;
      if (stamp - sample.stamp <= c.motion_window)
      {
        dz += sample.dz;
        if (sample.dz > .002)
          ++up_steps;
        if (sample.dz < -.002)
          ++down_steps;
      }
    }
    bool long_enough = !motion.empty()&&stamp - motion.front().stamp >= 1.0;
    bool slow_rising = long_enough&&slow_up >= 8&&slow_up >= 3 * slow_down&&slow_dz > c.slow_motion_threshold;
    bool slow_falling = long_enough&&slow_down >= 8&&slow_down >= 3 * slow_up&&slow_dz < -c.slow_motion_threshold;
    bool rising = (up_steps >= 3&&up_steps >= 3 * down_steps&&dz > c.motion_threshold)||slow_rising;
    bool falling = (down_steps >= 3&&down_steps >= 3 * up_steps&&dz < -c.motion_threshold)||slow_falling;
    if (slow_rising||slow_falling)
      dz = slow_dz;
    o.motion_dz = dz;
    o.ground_supported = currentGroundSupport(points, o.ground);
    o.support_path_raw = support_paths;
    if (o.ground_supported)
      last_ground_support = stamp;
    o.ground_support_held = !o.ground_supported&&last_ground_support >= 0&&stamp - last_ground_support <= .5;
    bool eligible = track.confirmed&&track.direct&&track.connected&&o.clearance_valid&&!o.ground_supported&&!o.ground_support_held&&o.clearance + 1e-8 >= c.suspended_clearance;
    // Direct surface motion can establish work without visible Ground.
    // Ground UNKNOWN still cannot establish static SUSPENDED or contact.
    const bool direct_motion = track.confirmed&&track.direct&&track.connected&&
                               !o.ground_supported&&!o.ground_support_held&&
                               (rising||falling);
    if ((eligible||direct_motion)&&!completion_consumed)
      armed = true;
    if (event_start >= 0&&stamp - event_start < c.completion_hold_sec)
    {
      o.state = "LOWERING_CONFIRMATION";
      o.reason = completion_reason;
      o.work_complete = true;
      return;
    }
    if (event_start >= 0)
    {
      event_start = -1;
      armed = false;
    }
    auto finish = [&](const char*why){
                    o.state = "LOWERING_CONFIRMATION";
                    o.reason = why;
                    o.work_complete = true;
                    event_start = stamp;
                    completion_reason = why;
                    armed = false;
                    completion_consumed = true;
                  };
    int overlap = 0;
    std::array<bool, 3>segments{};
    if (track.direct&&o.clearance_valid)
    {
      double co = std::cos(track.yaw), si = std::sin(track.yaw);
      for (auto i:body_refs)
      {
        const auto&p = points[i];
        if (p.z > track.bottom_z + .04)
          continue;
        int ix = int(std::floor(p.x / c.voxel_xy)), iy = int(std::floor(p.y / c.voxel_xy)), iz = int(std::floor(o.ground.z / c.voxel_z));
        bool floor = false;
        for (int dz = -1; dz <= 1; ++dz)
        {
          int j = index.find(key(ix, iy, iz + dz));
          if (j < 0)
            continue;
          for (uint32_t k = cells[j].begin; k < cells[j].end; ++k)
          {
            auto &g = points[entries[k].point];
            if (floor_mask[entries[k].point]&&g.raw != p.raw)
              floor = true;
          }
        }
        if (floor)
        {
          ++overlap;
          double u = (p.x - track.center.x) * co + (p.y - track.center.y) * si;
          int seg = std::clamp(int((u / std::max(.1, track.length) + .5) * 3), 0, 2);
          segments[seg] = true;
        }
      }
    }
    o.contact_ground_direct_valid = overlap >= 12&&std::count(segments.begin(), segments.end(), true) >= 2;
    bool contact = armed&&track.direct&&o.clearance_valid&&(o.contact_ground_direct_valid||o.ground_supported)&&o.clearance >= -.05&&o.clearance <= c.contact_clearance;
    bool direct_hook = track.supporting_side >= 0&&hooks[track.supporting_side].direct;
    bool full_body = track.direct&&track.length >= .65 * track.model_length&&track.width >= .5 * track.model_width;
    double nearest_xy = 1e9, nearest_3d = 1e9;
    int anchor_points = 0;
    if (direct_hook&&track.direct)
    {
      auto &h = hooks[track.supporting_side];
      for (auto i:body_refs)
      {
        auto&p = points[i];
        nearest_xy = std::min(nearest_xy, std::hypot(p.x - h.virtual_hook.x, p.y - h.virtual_hook.y));
        nearest_3d = std::min(nearest_3d, distance(Vec3{p.x, p.y, p.z}, h.virtual_hook));
        if (std::abs(p.x - h.virtual_hook.x) <= .15&&std::abs(p.y - h.virtual_hook.y) <= .15&&p.z < h.virtual_hook.z + .05)
          ++anchor_points;
      }
    }
    // Missing rigging pixels or a partial body are UNKNOWN, not proof that a
    // previously confirmed load detached from its supporting Hook.
    const bool separation_evidence = full_body&&direct_hook&&!track.connected&&anchor_points < 8&&(nearest_xy > .30||nearest_3d > c.connection_gap + .2);
    o.hook_separation_observed = track.confirmed&&separation_evidence;
    bool detached = armed&&separation_evidence;
    o.connection_observation = track.connected?"DIRECT_CONNECTED":detached?"DIRECT_SEPARATED":"UNKNOWN";
    contact_since = contact?(contact_since < 0?stamp:contact_since):-1;
    detach_since = detached?(detach_since < 0?stamp:detach_since):-1;
    if (!shrink_hold.active&&armed&&!track.direct&&continuous&&stamp - run_start >= c.loss_sec&&stamp - track.last_direct_sec + 1e-8 >= c.loss_sec)
    {
      finish("LOSS_TIMEOUT");
      return;
    }
    if (contact_since >= 0&&stamp - contact_since + 1e-8 >= c.contact_confirm_sec)
    {
      finish("GROUND_CONTACT");
      return;
    }
    if (detach_since >= 0&&stamp - detach_since + 1e-8 >= c.detach_confirm_sec)
    {
      finish("HOOK_DETACHED");
      return;
    }
    bool hold_cycle = armed&&!detached&&(shrink_hold.active||track.direct||stamp - track.last_direct_sec < c.loss_sec||stamp - run_start < c.loss_sec);
    if (!eligible&&!hold_cycle)
    {
      o.reason = o.ground_supported?"CURRENT_GROUND_SUPPORTED_BODY":o.ground_support_held?"GROUND_SUPPORT_TEMPORARILY_OCCLUDED":track.direct?"CARGO_NOT_AIRBORNE_CONNECTED":"NO_DIRECT_CARGO";
      return;
    }
    o.work_active = true;
    o.state = "SUSPENDED";
    o.reason = eligible?"CURRENT_BODY_GROUND_HOOK":"HOLD_CONFIRMED_LOAD_OBSERVATION_UNCERTAIN";
    if (track.direct&&rising)
    {
      o.state = "LIFTING";
      o.reason = "MATCHED_DIRECT_SURFACE_RISE";
    }
    if (track.direct&&falling)
    {
      o.state = "LOWERING";
      o.reason = "MATCHED_DIRECT_SURFACE_FALL";
    }
  }
  Output process(const std::vector<Point>&points, int64_t stamp_ns)
  {
    Output o;
    o.stamp_ns = stamp_ns;
    o.input_points = points.size();
    double stamp = stamp_ns * 1e-9;
    if (stamp_ns <= 0)
    {
      o.reason = "INVALID_SENSOR_TIMESTAMP";
      return o;
    }
    if (last_stamp >= 0&&stamp <= last_stamp)
    {
      o.reason = stamp == last_stamp?"DUPLICATE_TIMESTAMP":"REVERSED_TIMESTAMP";
      return o;
    }
    bool continuous = last_stamp >= 0&&stamp - last_stamp <= .5;
    if (!continuous)
      run_start = stamp;
    last_stamp = stamp;
    auto t = Clock::now();
    build(points);
    o.stage_ms[0] = elapsed(t);
    o.voxel_cells = cells.size();
    t = Clock::now();
    observeHooks(points, stamp);
    o.hooks = hooks;
    o.stage_ms[1] = elapsed(t);
    t = Clock::now();
    o.ground = ground(points);
    o.stage_ms[2] = elapsed(t);
    t = Clock::now();
    Cargo measured = detect(points, o.ground, o);
    o.stage_ms[3] = elapsed(t);
    t = Clock::now();
    o.cargo_measurement_direct = measured.direct;
    if (measured.direct)
    {
      o.measured_cargo_center = measured.center;
      o.measured_cargo_length = measured.length;
      o.measured_cargo_width = measured.width;
      o.measured_cargo_height = measured.height;
      o.measured_cargo_points = measured.raw_indices.size();
    }
    prepareShrinkHold(measured, stamp, continuous);
    update(std::move(measured), points, o, stamp, continuous);
    publishShrinkHold(o, stamp);
    // Publication only: tracking, Raw ownership and FSM use the measured body.
    if (length_relation_id != track.id || !continuous)
      length_separation_since = -1;
    length_relation_id = track.id;
    if (o.hook_separation_observed) {
      if (length_separation_since < 0) length_separation_since = stamp;
    } else length_separation_since = -1;
    const bool length_detached = length_separation_since >= 0 &&
      stamp - length_separation_since + 1e-8 >= c.detach_confirm_sec;
    const bool length_lost = !track.direct && !shrink_hold.active && continuous &&
      stamp - run_start >= c.loss_sec && stamp - track.last_direct_sec + 1e-8 >= c.loss_sec;
    const bool ended_connection = completion_consumed &&
      (completion_reason == "HOOK_DETACHED" || completion_reason == "LOSS_TIMEOUT");
    if(o.work_active&&track.direct&&track.connected&&track.confirmed&&candidate_reference.id==track.id)
      published_length.confirmObservedPeak(track.id,candidate_reference.length,
        confirmed_direct_duration,c.cargo_length_confirm_sec);
    const double locked = published_length.update(o.cargo.id,
      track.direct && o.cargo_measurement_direct, track.confirmed,
      o.measured_cargo_length, stamp, continuous,
      ended_connection || length_detached || length_lost, c.cargo_length_confirm_sec,
      o.work_active, track.connected);
    o.cargo.length_locked = locked > 0;
    if (o.cargo.length_locked) {
      o.cargo.model_length = locked;
      if (locked > o.measured_cargo_length + 1e-8)
        o.cargo.model_extent_observation = "ESTIMATED_EXTENT";
    }
    retained_extent.update(track,o.hooks,o.work_active,locked,
      ended_connection || length_detached || length_lost,c.voxel_xy,o);
    if(c.crane_motion_mode!="body_xy")crane_hist.update(points,c,stamp,continuous,o);
    else crane_xy.update(points,c,stamp,continuous,o);
    o.stage_ms[4] = elapsed(t);
    return o;
  }
};
Engine::Engine(Config c) : impl_(std::make_unique<Impl>(c))
{
}
Engine::~Engine() = default;
Engine::Engine(Engine&&)noexcept= default;
Engine&Engine::operator=(Engine&&)noexcept= default;
Output Engine::process(const std::vector<Point>&p, int64_t stamp)
{
  return impl_->process(p, stamp);
}
void Engine::reset()
{
  auto c = impl_->c;
  impl_ = std::make_unique<Impl>(c);
}
}
