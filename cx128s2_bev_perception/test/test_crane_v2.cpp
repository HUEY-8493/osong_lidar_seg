#include "cx128s2_bev_perception/v2/core.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace crane_v2;
#define REQUIRE(v) do {if (!(v)) throw std::runtime_error(std::string(__func__) + ":" + std::to_string(__LINE__) + ": " #v);}while (0)
std::vector<Point> scene(bool cargo = true, bool hook = true, bool floor = true, double z = 1.0, double y = 0, double angle = 0, double hook_y = 0)
{
  std::vector<Point>p;
  auto add = [&](double x, double y, double z){
               p.push_back({float(x), float(y), float(z), uint32_t(p.size())});
             };
  if (floor)
    for (double x = -3; x < 3; x += .08)
      for (double a = -3; a < 3; a += .08)
        add(x, a, 0);
  if (hook)
    for (double x = -1.08; x < -.72; x += .018)
      for (double a = hook_y - .12; a < hook_y + .12; a += .018)
        for (double h = 2.05; h < 2.35; h += .025)
          add(x, a, h);
  // Visible surface fixture, not points filling an object's interior.
  if (cargo)
    for (double u = -1.5; u < 1.5; u += .03)
      for (double v = -.17; v < .17; v += .03)
      {
        double x = -.9 + u * std::cos(angle) - v * std::sin(angle), yy = y + u * std::sin(angle) + v * std::cos(angle);
        add(x, yy, z + .28);
        if (v < -.16||v > .15||u < -1.49||u > 1.46)
          for (double h = 0; h < .28; h += .035)
            add(x, yy, z + h);
      }
  return p;
}
Output run(Engine&e, const std::vector<Point>&p, int frame)
{
  return e.process(p, 1000000000LL + frame * 100000000LL);
}
void stationary()
{
  Engine e;
  Output o;
  auto p = scene();
  for (int i = 0; i < 30; ++i)
    o = run(e, p, i);
  REQUIRE(o.cargo.direct);
  REQUIRE(o.cargo.confirmed);
  REQUIRE(o.state == "SUSPENDED");
  REQUIRE(o.cargo.length > 2.8);
  REQUIRE(o.cargo.id == 1);
  REQUIRE(o.clearance_valid);
  REQUIRE(o.clearance > .9);
  auto raw = o.cargo.raw_indices;
  for (auto id:raw)
    REQUIRE(std::find(o.ground.raw_indices.begin(), o.ground.raw_indices.end(), id) == o.ground.raw_indices.end());
  for (int i = 30; i < 38; ++i)
  {
    o = run(e, scene(true, true, true, 1 + (i - 29) * .012), i);
  }
  REQUIRE(o.state == "LIFTING");
  for (int i = 38; i < 46; ++i)
    o = run(e, scene(true, true, true, 1.096 - (i - 37) * .012), i);
  REQUIRE(o.state == "LOWERING");
  for (int i = 46; i < 60; ++i)
    o = run(e, p, i);
  REQUIRE(o.state == "SUSPENDED");
  e.reset();
  o = run(e, p, 100);
  REQUIRE(!o.cargo.confirmed);
  REQUIRE(!o.work_complete);
}
void negatives()
{
  for (auto p:{scene(false, true), scene(false, false), scene(true, false), std::vector<Point>{}, std::vector<Point>{{0, 0, 1, 0}}})
  {
    Engine e;
    for (int i = 0; i < 30; ++i)
    {
      auto o = run(e, p, i);
      REQUIRE(o.state == "IDLE");
      REQUIRE(!o.work_complete);
    }
  }
  Engine e;
  auto p = scene(false);
  for (double z = .2; z < 2.; z += .01)
    p.push_back({-.9f, 0, float(z), uint32_t(p.size())});
  for (int i = 0; i < 30; ++i)
    REQUIRE(run(e, p, i).state == "IDLE");
}
void unknownGround()
{
  Engine e;
  for (int i = 0; i < 30; ++i)
  {
    auto o = run(e, scene(true, true, false), i);
    REQUIRE(!o.clearance_valid);
    REQUIRE(!o.work_complete);
  }
}
void loss()
{
  Engine e;
  for (int i = 0; i < 30; ++i)
    run(e, scene(), i);
  auto p = scene(false);
  for (int i = 30; i < 42; ++i)
    REQUIRE(!run(e, p, i).work_complete);
  auto o = run(e, p, 42);
  REQUIRE(o.work_complete);
  REQUIRE(o.reason == "LOSS_TIMEOUT");
  for (int i = 43; i < 72; ++i)
    REQUIRE(run(e, p, i).work_complete);
  REQUIRE(!run(e, p, 73).work_complete);
  REQUIRE(!run(e, p, 74).work_complete);
  Engine g;
  for (int i = 0; i < 30; ++i)
    run(g, scene(), i);
  o = run(g, p, 100);
  REQUIRE(!o.work_complete);
  for (int i = 101; i < 113; ++i)
    REQUIRE(!run(g, p, i).work_complete);
}
void contact()
{
  Engine e;
  for (int i = 0; i < 30; ++i)
    run(e, scene(), i);
  Output o;
  for (int i = 30; i < 38; ++i)
    o = run(e, scene(true, true, true, 1. - .1 * (i - 29)), i);
  for (int i = 38; i < 44; ++i)
    o = run(e, scene(true, true, true, .2), i);
  REQUIRE(o.work_complete);
  REQUIRE(o.reason == "GROUND_CONTACT");
}
void detach()
{
  Engine e;
  for (int i = 0; i < 30; ++i)
    run(e, scene(), i);
  Output o;
  for (int i = 30; i < 35; ++i)
    o = run(e, scene(true, true, true, 1, 0, 0, .6), i);
  REQUIRE(o.work_complete);
  REQUIRE(o.reason == "HOOK_DETACHED");
  Engine h;
  for (int i = 0; i < 30; ++i)
    run(h, scene(), i);
  for (int i = 30; i < 40; ++i)
    REQUIRE(!run(h, scene(true, false), i).work_complete);
}
void timestamps()
{
  Engine e;
  auto p = scene();
  auto a = run(e, p, 10);
  auto b = run(e, p, 10);
  REQUIRE(b.reason == "DUPLICATE_TIMESTAMP");
  REQUIRE(!b.cargo.direct);
  b = run(e, p, 9);
  REQUIRE(b.reason == "REVERSED_TIMESTAMP");
  REQUIRE(!b.work_complete);
  b = e.process(p, 0);
  REQUIRE(b.reason == "INVALID_SENSOR_TIMESTAMP");
  Engine other;
  auto c = run(other, p, 11);
  REQUIRE(c.cargo.id == a.cargo.id);
}
void geometryAndOcclusion()
{
  Engine e;
  Output o;
  for (int i = 0; i < 30; ++i)
    o = run(e, scene(), i);
  const auto id = o.cargo.id;
  for (int i = 30; i < 45; ++i)
  {
    o = run(e, scene(true, true, true, 1., .003 * (i - 29), .01 * (i - 29)), i);
    REQUIRE(o.cargo.id == id);
    REQUIRE(o.state != "LIFTING"&&o.state != "LOWERING");
  }
  auto partial = scene();
  partial.erase(std::remove_if(partial.begin(), partial.end(), [](const Point&p){
    return p.z >= 1&&p.z < 1.3&&p.x < -1.4;
  }), partial.end());
  o = run(e, partial, 45);
  REQUIRE(o.cargo.direct);
  REQUIRE(o.cargo.id == id);
  REQUIRE(o.cargo.model_length > 2.8);
  auto large = scene();
  auto original = large;
  for (int copy = 0; copy < 95; ++copy)
    for (auto p:original)
    {
      p.raw = large.size();
      large.push_back(p);
    }
  REQUIRE(large.size() >= 400000);
  Engine separate;
  o = run(separate, large, 1);
  REQUIRE(o.input_points == large.size());
  REQUIRE(!o.work_complete);
  o = run(separate, {}, 2);
  REQUIRE(!o.cargo.direct);
  REQUIRE(o.cargo.raw_indices.empty());
  o = run(separate, scene(), 3);
  REQUIRE(!o.cargo.confirmed);
}
void tiltedOcclusion()
{
  Engine e;
  Output o;
  auto tilted = [](double clip){
                  auto p = scene();
                  p.erase(std::remove_if(p.begin(), p.end(), [&](const Point&a){
      return a.z >= .99&&a.z < 1.3&&a.x < clip;
    }), p.end());
                  for (auto &a:p)
                    if (a.z >= .99&&a.z < 1.3)
                      a.z += .25 * (a.x + .9);
                  return p;
                };
  for (int i = 0; i < 30; ++i)
    o = run(e, tilted(-3), i);
  REQUIRE(o.cargo.confirmed);
  auto id = o.cargo.id;
  for (int i = 30; i < 55; ++i)
  {
    o = run(e, tilted(-2.4 + .08 * (i - 29)), i);
    std::cerr << "OCC " << i << " " << o.state << " dz=" << o.motion_dz << " length=" << o.cargo.length << "\n";
    REQUIRE(o.cargo.id == id);
    REQUIRE(o.state != "LIFTING"&&o.state != "LOWERING");
  }
}
void directMotionWithoutGroundOrHook()
{
  Engine e;
  Output o;
  for (int i = 0; i < 30; ++i)
    o = run(e, scene(true, true, false), i);
  REQUIRE(o.cargo.confirmed);
  REQUIRE(!o.clearance_valid);
  for (int i = 30; i < 38; ++i)
    o = run(e, scene(true, true, false, 1 + .012 * (i - 29)), i);
  REQUIRE(o.state == "LIFTING");
  REQUIRE(!o.clearance_valid);
  REQUIRE(!o.work_complete);
  Engine h;
  for (int i = 0; i < 30; ++i)
    o = run(h, scene(), i);
  auto id = o.cargo.id;
  for (int i = 30; i < 38; ++i)
  {
    o = run(h, scene(true, false, true, 1 + .012 * (i - 29)), i);
    REQUIRE(!o.work_complete);
    REQUIRE(o.cargo.id == id);
  }
  REQUIRE(o.cargo.direct);
  REQUIRE(!o.hooks[0].direct);
  REQUIRE(o.state == "LIFTING");
  for (int i = 38; i < 48; ++i)
  {
    o = run(h, scene(true, false, true, 1.096), i);
    REQUIRE(!o.work_complete);
  }
  REQUIRE(o.state == "SUSPENDED");
  REQUIRE(o.connection_observation == "UNKNOWN");
}
void contactWithoutLoweringHistory()
{
  Engine e;
  Output o;
  for (int i = 0; i < 30; ++i)
    o = run(e, scene(), i);
  // Sensor stream gap proves no intervening motion. A fresh independently
  // observed contact must still complete the previously confirmed work.
  for (int i = 50; i < 55; ++i)
  {
    o = run(e, scene(true, true, true, .2), i);
    REQUIRE(o.state != "LOWERING");
  }
  REQUIRE(o.work_complete);
  REQUIRE(o.reason == "GROUND_CONTACT");
}
void estimatedBottomAndTiltedTop()
{
  Engine e;
  Output o;
  for (int i = 0; i < 30; ++i)
    o = run(e, scene(), i);
  REQUIRE(o.cargo.bottom_observation == "ESTIMATED");
  REQUIRE(o.cargo.bottom_valid);
  REQUIRE(std::abs(o.cargo.bottom_z - 1.) < .06);
  auto p = scene();
  p.erase(std::remove_if(p.begin(), p.end(), [](const Point&a){
    return a.z > .99&&a.z < 1.27;
  }), p.end());
  for (auto &a:p)
    if (a.z > 1.27&&a.z < 1.3)
      a.z += .2 * (a.x + .9);
  Engine top;
  for (int i = 0; i < 30; ++i)
    o = run(top, p, i);
  REQUIRE(o.cargo.direct);
  REQUIRE(o.cargo.confirmed);
  REQUIRE(!o.cargo.bottom_valid);
  REQUIRE(o.cargo.bottom_observation == "UNKNOWN");
  REQUIRE(!o.clearance_valid);
  REQUIRE(o.state == "IDLE");
  REQUIRE(!o.work_complete);
}
void slowMeasuredRise()
{
  Engine e;
  Output o;
  for (int i = 0; i < 30; ++i)
    o = run(e, scene(), i);
  for (int i = 30; i < 60; ++i)
    o = run(e, scene(true, true, true, 1 + .0015 * (i - 29)), i);
  REQUIRE(o.cargo.direct);
  REQUIRE(o.state == "LIFTING");
  for (int i = 60; i < 80; ++i)
    o = run(e, scene(true, true, true, 1.045), i);
  REQUIRE(o.state == "SUSPENDED");
  for (int i = 80; i < 110; ++i)
    o = run(e, scene(true, true, true, 1.045 - .0015 * (i - 79)), i);
  REQUIRE(o.state == "LOWERING");
}
void nearbyParallelBodyMustNotHijack()
{
  Engine e;Output o;
  for(int i=0;i<30;++i)o=run(e,scene(),i);
  auto id=o.cargo.id;
  auto partial=scene();
  partial.erase(std::remove_if(partial.begin(),partial.end(),[](const Point&a){return a.z>.99&&a.z<1.3&&a.x<-.2;}),partial.end());
  auto neighbor=scene(true,false,false,1.,.60);
  for(auto a:neighbor){a.raw=partial.size();partial.push_back(a);}
  for(int i=30;i<35;++i){o=run(e,partial,i);REQUIRE(o.cargo.id==id);REQUIRE(!o.work_complete);if(o.cargo.direct)REQUIRE(o.cargo.center.y<.25);REQUIRE(o.state!="LIFTING");}
  auto only_neighbor=scene(false);
  for(auto a:neighbor){a.raw=only_neighbor.size();only_neighbor.push_back(a);}
  o=run(e,only_neighbor,35);
  REQUIRE(o.cargo.id==id);REQUIRE(!o.cargo.direct);REQUIRE(!o.work_complete);
}
int main()
{
  try{
    nearbyParallelBodyMustNotHijack();
    slowMeasuredRise();
    estimatedBottomAndTiltedTop();
    directMotionWithoutGroundOrHook();
    contactWithoutLoweringHistory();
    tiltedOcclusion();
    stationary();
    negatives();
    unknownGround();
    loss();
    contact();
    detach();
    timestamps();
    geometryAndOcclusion();
    std::cout << "PASS independent V2: stationary/restart/motion/ownership/negatives/ground/loss/stream gap/contact/detach/time/reset/instances\n";
    return 0;
  }
  catch (const std::exception&e){
    std::cerr << e.what() << '\n';
    return 1;
  }
}
