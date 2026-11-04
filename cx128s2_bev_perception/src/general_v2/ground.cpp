#include "cx128s2_bev_perception/general_v2/core.hpp"
#include <Eigen/QR>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <map>
#include <random>
#include <deque>
#include <set>
namespace general_v2
{
namespace {
// Current-frame local surfaces only. Vertical body panels cannot supply a floor
// fit, and a disconnected horizontal load surface cannot become a ground seed.
void local_surfaces(Ground &ground,const std::vector<Point>&points,
                    const std::vector<uint32_t>&valid,const Config&c)
{
  using Key=std::pair<int,int>;
  std::map<Key,std::vector<uint32_t>> cells;
  for(auto i:valid){const auto&p=points[i];const double h=p.z-ground.at(p.x,p.y);
    if(h>=-.15&&h<=c.ground_local_max_rise)
      cells[{int(std::floor(p.x/c.ground_local_cell)),int(std::floor(p.y/c.ground_local_cell))}].push_back(i);
  }
  std::map<Key,Eigen::Vector3d> candidates;
  std::deque<Key> queue;
  std::set<Key> accepted;
  for(auto &kv:cells){
    auto&ids=kv.second;if(int(ids.size())<c.ground_local_min_points)continue;
    std::vector<double> heights;heights.reserve(ids.size());
    for(auto i:ids)heights.push_back(points[i].z-ground.at(points[i].x,points[i].y));
    auto nth=heights.begin()+heights.size()/10;std::nth_element(heights.begin(),nth,heights.end());const double low=*nth;
    const double cx=(kv.first.first+.5)*c.ground_local_cell,cy=(kv.first.second+.5)*c.ground_local_cell;
    Eigen::Matrix3d normal=Eigen::Matrix3d::Zero();Eigen::Vector3d rhs=Eigen::Vector3d::Zero();
    double minx=1e9,maxx=-1e9,miny=1e9,maxy=-1e9;std::vector<uint32_t> selected;
    for(auto i:ids){const auto&p=points[i];const double h=p.z-ground.at(p.x,p.y);
      if(h>low+c.ground_local_band)continue;
      const Eigen::Vector3d row(p.x-cx,p.y-cy,1);normal+=row*row.transpose();rhs+=row*h;selected.push_back(i);
      minx=std::min(minx,p.x);maxx=std::max(maxx,p.x);miny=std::min(miny,p.y);maxy=std::max(maxy,p.y);
    }
    if(int(selected.size())<c.ground_local_min_points||maxx-minx<c.ground_local_cell*.3||maxy-miny<c.ground_local_cell*.3)continue;
    auto qr=normal.colPivHouseholderQr();if(qr.rank()<3)continue;
    const Eigen::Vector3d relative=qr.solve(rhs);
    if(!relative.allFinite()||std::hypot(ground.a+relative.x(),ground.b+relative.y())>c.ground_local_slope)continue;
    double residual=0;for(auto i:selected){const auto&p=points[i];residual+=std::abs(p.z-ground.at(p.x,p.y)-relative.dot(Eigen::Vector3d(p.x-cx,p.y-cy,1)));}
    if(residual/selected.size()>c.ground_local_residual)continue;
    const Eigen::Vector3d absolute(ground.a+relative.x(),ground.b+relative.y(),ground.c+relative.z()-relative.x()*cx-relative.y()*cy);
    candidates[kv.first]=absolute;
    if(std::abs(relative.z())<=.10){accepted.insert(kv.first);queue.push_back(kv.first);}
  }
  while(!queue.empty()){
    const auto k=queue.front();queue.pop_front();const auto plane=candidates.at(k);
    const double x=(k.first+.5)*c.ground_local_cell,y=(k.second+.5)*c.ground_local_cell;
    const double z=plane.dot(Eigen::Vector3d(x,y,1));
    for(int dx=-1;dx<=1;++dx)for(int dy=-1;dy<=1;++dy){if(!dx&&!dy)continue;Key next{k.first+dx,k.second+dy};
      if(accepted.count(next))continue;const auto it=candidates.find(next);if(it==candidates.end())continue;
      const double nx=x+dx*c.ground_local_cell,ny=y+dy*c.ground_local_cell;
      const double nz=it->second.dot(Eigen::Vector3d(nx,ny,1));
      if(std::abs(nz-z)>c.ground_local_slope*std::hypot(nx-x,ny-y)+.02)continue;
      accepted.insert(next);queue.push_back(next);
    }
  }
  ground.local_cell=c.ground_local_cell;ground.local_band=c.ground_local_band;
  for(const auto&k:accepted)ground.local_surfaces.emplace(k,candidates.at(k));
}
}
bool Ground::surface_return(const Point&p) const
{
  if(local_surfaces.empty()||local_cell<=0)return false;
  const int x=int(std::floor(p.x/local_cell)),y=int(std::floor(p.y/local_cell));
  for(int dx=-1;dx<=1;++dx)for(int dy=-1;dy<=1;++dy){const auto it=local_surfaces.find({x+dx,y+dy});
    if(it!=local_surfaces.end()&&std::abs(p.z-it->second.dot(Eigen::Vector3d(p.x,p.y,1)))<=local_band)return true;
  }
  return false;
}
Ground estimate_ground(const std::vector<Point>& points, const std::vector<uint32_t>& valid, const Config& c)
{
  Ground out;
  // Current-frame lower-envelope samples only; no learned/static map or assumed z=0 floor.
  std::map<std::pair<int, int>, std::vector<Point> > tiles;
  for (auto i:valid)
  {
    const auto&p = points[i];
    if (p.z >= c.ground_seed_min_z && p.z <= c.ground_seed_max_z)
    {
      tiles[{int(std::floor(p.x / c.ground_tile)), int(std::floor(p.y / c.ground_tile))}].push_back(p);
    }
  }
  std::vector<Eigen::Vector3d> seeds;
  for (auto &kv:tiles)
  {
    auto &v = kv.second;
    if (int(v.size()) < c.ground_min_points)
    {
      continue;
    }
    std::sort(v.begin(), v.end(), [](const Point&a, const Point&b){
        return a.z < b.z;
      });
    const auto&p = v[(v.size() - 1) / 10];
    seeds.emplace_back(p.x, p.y, p.z);
  }
  out.seeds = int(seeds.size());
  if (int(seeds.size()) < c.ground_min_tiles)
  {
    out.reason = "UNKNOWN_INSUFFICIENT_FLOOR_SAMPLES";
    return out;
  }
  std::mt19937 gen(17);
  std::uniform_int_distribution<size_t> pick(0, seeds.size() - 1);
  std::vector<size_t> best;
  for (int trial = 0; trial < 128; ++trial)
  {
    const auto &p = seeds[pick(gen)], &q = seeds[pick(gen)], &r = seeds[pick(gen)];
    Eigen::Vector3d n = (q - p).cross(r - p);
    if (std::abs(n.z()) < 1e-8)
    {
      continue;
    }
    const double a = -n.x() / n.z(), b = -n.y() / n.z(), z = p.z() - a * p.x() - b * p.y();
    if (std::hypot(a, b) > c.ground_max_slope)
    {
      continue;
    }
    std::vector<size_t> inliers;
    for (size_t i = 0; i < seeds.size(); ++i)
    {
      if (std::abs(seeds[i].z() - a * seeds[i].x() - b * seeds[i].y() - z) <= c.ground_inlier)
      {
        inliers.push_back(i);
      }
    }
    if (inliers.size() > best.size())
    {
      best = std::move(inliers);
    }
  }
  if (int(best.size()) < c.ground_min_tiles)
  {
    out.reason = "UNKNOWN_NO_SUPPORTED_PLANE";
    return out;
  }
  Eigen::MatrixXd design(best.size(), 3);
  Eigen::VectorXd values(best.size());
  double minx = 1e9, maxx = -1e9, miny = 1e9, maxy = -1e9;
  for (size_t j = 0; j < best.size(); ++j)
  {
    const auto&p = seeds[best[j]];
    design.row(j) << p.x(), p.y(), 1;
    values(j) = p.z();
    minx = std::min(minx, p.x());
    maxx = std::max(maxx, p.x());
    miny = std::min(miny, p.y());
    maxy = std::max(maxy, p.y());
  }
  if (maxx - minx < c.ground_min_span || maxy - miny < c.ground_min_span)
  {
    out.reason = "UNKNOWN_NARROW_GROUND_SUPPORT";
    return out;
  }
  auto qr = design.colPivHouseholderQr();
  if (qr.rank() < 3)
  {
    out.reason = "UNKNOWN_DEGENERATE_PLANE";
    return out;
  }
  Eigen::Vector3d fit = qr.solve(values);
  if (!fit.allFinite() || std::hypot(fit.x(), fit.y()) > c.ground_max_slope)
  {
    out.reason = "UNKNOWN_INVALID_FIT";
    return out;
  }
  out.a = fit.x();
  out.b = fit.y();
  out.c = fit.z();
  out.support = int(best.size());
  out.residual = (design * fit - values).array().abs().mean();
  out.valid = true;
  out.reason = "DIRECT_CURRENT_FRAME_PLANE";
  local_surfaces(out,points,valid,c);
  return out;
}
}
