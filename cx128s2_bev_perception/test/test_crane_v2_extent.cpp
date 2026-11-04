#include "cx128s2_bev_perception/v2/core.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <utility>
using namespace crane_v2;
#define REQUIRE(v) do {if (!(v)) throw std::runtime_error(std::string(__func__) + ":" + std::to_string(__LINE__) + ": " #v);}while (0)
std::vector<Point> environment()
{
  std::vector<Point> p;
  auto add = [&](double x, double y, double z){
               p.push_back({float(x), float(y), float(z), uint32_t(p.size())});
             };
  for (double x = -3; x < 3; x += .08)
    for (double y = -3; y < 3; y += .08)
      add(x, y, 0);
  // The unchanged R15 Hook fixture isolates tests of Body geometry.
  for (double x = -1.08; x < -.72; x += .018)
    for (double y = -.12; y < .12; y += .018)
      for (double z = 2.05; z < 2.35; z += .025)
        add(x, y, z);
  return p;
}
Output run(Engine&e, const std::vector<Point>&p, int frame)
{
  return e.process(p, 1000000000LL + frame * 100000000LL);
}
#include <unordered_set>
#include <functional>
struct Fixture {
  std::vector<Point>points;
  std::unordered_set<uint32_t>body, other;
};
Fixture beam(double length = 12, double width = .24, double angle = 0, double bottom = 1, bool gap = false, bool sparse = false, bool parallel = false, bool column = false, bool wire = false, bool outlier = false, double clip = 1e9)
{
  Fixture f;
  f.points = environment();
  double co = std::cos(angle), si = std::sin(angle), height = .24;
  auto add = [&](double u, double v, double z, bool body){
               Point p{float(-.9 + u * co - v * si), float(u * si + v * co), float(z), uint32_t(f.points.size())};
               (body?f.body:f.other).insert(p.raw);
               f.points.push_back(p);
             };
  auto surface = [&](double y, bool target){
                   int n = int(std::round(length / .03)), m = int(std::round(width / .025));
                   for (int j = 0; j <= n; ++j)
                   {
                     double u = -length / 2 + length * j / n;
                     if (target&&u > clip)
                       continue;
                     if (target&&gap&&u > 1.2&&u < 1.65)
                       continue;
                     if (target&&sparse&&u > .8&&u < 1.05)
                     {
                       add(u, y - width * .5, bottom + .09 + .03 * (j % 2), true);
                       add(u, y - width * .5 + .04, bottom + height, true);
                       continue;
                     }
                     for (int k = 0; k <= m; ++k)
                     {
                       double v = y - width / 2 + width * k / m;
                       add(u, v, bottom + height, target);
                       if (k == 0||k == m||j == 0||j == n)
                         for (double z = bottom; z < bottom + height; z += .025)
                           add(u, v, z, target);
                     }
                   }
                 };
  surface(0, true);
  if (parallel)
    surface(.6, false);
  if (column)
    for (double u = length / 2 + .02; u < length / 2 + .3; u += .025)
      for (double v = -.35; v < .35; v += .025)
        for (double z = .1; z < 2.; z += .04)
          if (u < length / 2 + .05||u > length / 2 + .26||std::abs(v) > .31||z > 1.95)
            add(u, v, z, false);
  if (wire)
    for (double u = length / 2 + .03; u < length / 2 + 3; u += .025)
      add(u, 0, bottom + height + .02, false);
  if (outlier)
    add(length / 2 + 2, 0, bottom + height, false);
  return f;
}
void ownership(const Fixture&f, const Output&o)
{
  std::unordered_set<uint32_t>current;
  for (const auto&p:f.points)
    current.insert(p.raw);
  for (auto raw:o.cargo.raw_indices)
  {
    REQUIRE(current.count(raw));
    REQUIRE(f.body.count(raw));
  }
}
Output warm(Engine&e, const Fixture&f, int frames = 30)
{
  Output o;
  for (int i = 0; i < frames; ++i)
    o = run(e, f.points, i);
  return o;
}
void longAndYaw()
{
  for (double angle:{0., .35, -.4})
  {
    auto f = beam(12, .24, angle);
    Engine e;
    auto o = warm(e, f);
    REQUIRE(o.cargo.direct);
    REQUIRE(o.cargo.length > 11.8);
    REQUIRE(o.cargo.length < 12.15);
    ownership(f, o);
    std::cout << "PASS long/Hook-ROI-ends/yaw " << angle << " L=" << o.cargo.length << '\n';
  }
}
void missingMiddle()
{
  auto full = beam();
  Engine e;
  auto o = warm(e, full);
  auto id = o.cargo.id;
  auto g = beam(12, .24, 0, 1, true);
  REQUIRE(e.process(full.points, 0).reason == "INVALID_SENSOR_TIMESTAMP");
  REQUIRE(run(e, full.points, 29).reason == "DUPLICATE_TIMESTAMP");
  REQUIRE(run(e, full.points, 10).reason == "REVERSED_TIMESTAMP");
  for (int i = 30; i < 35; ++i)
    o = run(e, g.points, i);
  REQUIRE(o.cargo.id == id);
  REQUIRE(o.cargo.direct);
  REQUIRE(o.cargo.length > 11.8);
  ownership(g, o);
  std::cout << "PASS middle occlusion current Raw L=" << o.cargo.length << '\n';
  Engine cold;
  auto partial = warm(cold, g);
  REQUIRE(partial.cargo.length < 9.);
  ownership(g, partial);
  std::cout << "PASS separate aligned pieces without prior connected evidence remain separate\n";
}
void sparseNeck()
{
  Engine e;
  auto f = beam(12, .24, 0, 1, false, true);
  auto o = warm(e, f);
  REQUIRE(o.cargo.direct);
  REQUIRE(o.cargo.length > 11.8);
  ownership(f, o);
  std::cout << "PASS sparse actual side continuity L=" << o.cargo.length << '\n';
}
void invisibleEndAndReturn()
{
  Engine e;
  auto f = beam();
  auto o = warm(e, f);
  auto id = o.cargo.id;
  auto partial = beam(12, .24, 0, 1, false, false, false, false, false, false, 2.);
  for (int i = 30; i < 35; ++i)
    o = run(e, partial.points, i);
  REQUIRE(o.cargo.id == id);
  REQUIRE(o.cargo.length < 8.15);
  REQUIRE(o.cargo.model_length > 11.8);
  ownership(partial, o);
  for (int i = 35; i < 40; ++i)
    o = run(e, f.points, i);
  REQUIRE(o.cargo.id == id);
  REQUIRE(o.cargo.length > 11.8);
  ownership(f, o);
  std::cout << "PASS invisible end not invented; actual reacquisition ID retained\n";
}
void neighbours()
{
  for (int kind = 0; kind < 4; ++kind)
  {
    Engine e;
    auto f = beam(12, .24, 0, 1, false, false, kind == 0, kind == 1, kind == 2, kind == 3);
    auto o = warm(e, f);
    REQUIRE(o.cargo.direct);
    REQUIRE(o.cargo.length < 12.16);
    int wrong = 0;
    for (auto raw:o.cargo.raw_indices)
      if (f.other.count(raw))
        ++wrong;
    std::cout << "NEIGHBOR " << kind << " L=" << o.cargo.length << " other=" << wrong << '\n';
    REQUIRE(wrong == 0);
  }
}
void nonmovingOtherFragment()
{
  Engine e;
  auto full = beam();
  warm(e, full);
  auto f = beam(12, .24, 0, 1.1, true);
  for (auto &p:f.points)
    if (f.body.count(p.raw)&&p.x > -.9 + 1.65)
    {
      p.z -= .3;
      f.body.erase(p.raw);
      f.other.insert(p.raw);
    }
  auto o = run(e, f.points, 30);
  REQUIRE(o.cargo.direct);
  REQUIRE(o.cargo.length < 9.);
  ownership(f, o);
  std::cout << "PASS prior extent does not promote a lower unrelated fragment\n";
}
void sparseEnds()
{
  auto f = beam();
  std::vector<Point>kept;
  for (auto p:f.points)
  {
    if (f.body.count(p.raw)&&p.x > -.9 + 5.5&&p.raw % 7 != 0)
      continue;
    kept.push_back(p);
  }
  f.points = std::move(kept);
  Engine e;
  auto o = warm(e, f);
  REQUIRE(o.cargo.direct);
  REQUIRE(o.cargo.length > 11.8);
  ownership(f, o);
  std::cout << "PASS unequal density and sparse observed endpoint L=" << o.cargo.length << '\n';
}
void narrowToWideEnd()
{
  auto f = beam();
  for (double x = 5.12; x < 5.7; x += .025)
    for (double y = -.6; y < .6; y += .025)
    {
      uint32_t id = f.points.size();
      f.points.push_back({float(x), float(y), 1.24f, id});
      f.other.insert(id);
    }
  Engine e;
  auto o = warm(e, f);
  REQUIRE(o.cargo.direct);
  std::cout << "WIDE END L=" << o.cargo.length << " W=" << o.cargo.width << " added=" << o.cargo.extent_added_points << " removed=" << o.cargo.extent_removed_points << " other=";
  int bad = 0;
  for (auto i:o.cargo.raw_indices)
    bad += f.other.count(i);
  std::cout << bad << std::endl;
  REQUIRE(o.cargo.length < 12.16);
  ownership(f, o);
  std::cout << "PASS nearby wide surface not used to lengthen the Beam\n";
}
void longTwenty()
{
  Engine e;
  auto f = beam(20, .24, .2);
  auto o = warm(e, f);
  REQUIRE(o.cargo.direct);
  REQUIRE(o.cargo.length > 19.8&&o.cargo.length < 20.15);
  ownership(f, o);
  std::cout << "PASS 20m body outside Hook ROI, within watch volume\n";
}
void oneEndTouchesGround()
{
  auto f = beam(6, .24, 0, .45);
  for (auto &a:f.points)
    if (f.body.count(a.raw))
      a.z += .15 * (a.x + .9);
  for (double z = .69; z < 1.31; z += .025)
  {
    auto id = uint32_t(f.points.size());
    f.points.push_back({-.9f, 0, float(z), id});
    f.other.insert(id);
  }
  Engine e;
  auto o = warm(e, f);
  REQUIRE(o.cargo.direct);
  REQUIRE(o.cargo.length > 5.8);
  for (auto id:o.cargo.raw_indices)
    REQUIRE(f.body.count(id)||f.other.count(id));
  std::cout << "PASS inclined body with one grounded end, XY L=" << o.cargo.length << " axis3D=" << o.cargo.observed_length_3d << '\n';
}
void interveningObject()
{
  Engine e;
  auto full = beam();
  warm(e, full);
  auto f = beam(12, .24, 0, 1, true);
  for (double u = 1.38; u < 1.44; u += .025)
    for (double v = -.2; v < .2; v += .025)
      for (double z = 1.; z < 1.7; z += .025)
      {
        auto id = uint32_t(f.points.size());
        f.points.push_back({float(-.9 + u), float(v), float(z), id});
        f.other.insert(id);
      }
  auto o = run(e, f.points, 30);
  REQUIRE(o.cargo.direct);
  REQUIRE(o.cargo.length < 9.);
  ownership(f, o);
  std::cout << "PASS intervening independent material blocks historical gap connection\n";
}
void shortCargo()
{
  auto f = beam(.5, .30);
  Engine e;
  auto o = warm(e, f);
  std::cout << "SMALL .5m direct=" << o.cargo.direct << " L=" << o.cargo.length << '\n';
  // Body-only requirement; SUSPENDED is not asserted because the frozen R15
  // bottom estimator separately requires >=0.6m longitudinal measurement.
  REQUIRE(o.cargo.direct);
  REQUIRE(o.cargo.length > .42&&o.cargo.length < .58);
  ownership(f, o);
}
int main()
{
  try{
    longAndYaw();
    missingMiddle();
    sparseNeck();
    invisibleEndAndReturn();
    neighbours();
    nonmovingOtherFragment();
    sparseEnds();
    narrowToWideEnd();
    longTwenty();
    oneEndTouchesGround();
    interveningObject();
    shortCargo();
    std::cout << "PASS extent deterministic scenes\n";
    return 0;
  }
  catch (const std::exception&e){
    std::cerr << e.what() << '\n';
    return 1;
  }
}
