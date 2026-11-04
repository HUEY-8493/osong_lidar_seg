#include "cx128s2_bev_perception/general_v2/core.hpp"
#include <iostream>
#include <stdexcept>
#include <limits>
#include <cmath>
using namespace general_v2;
void check(bool ok, const char*why)
{
  if (!ok)
  {
    throw std::runtime_error(why);
  }
}
std::vector<Point> floor_scene()
{
  std::vector<Point> p;
  for (int ix = -30; ix <= 90; ++ix)
  {
    for (int iy = -30; iy <= 30; ++iy)
    {
      p.push_back({ix*.4, iy * .4, 0, uint32_t(p.size())});
    }
  }
  return p;
}
void vehicle(std::vector<Point>&p, double cx = 8, double cy = 0, double length = 8, double width = 2.6,
             double height = 2.6)
{
  for (double x = -length * .5; x <= length * .5 + .001; x += .2)
  {
    for (double y = -width * .5; y <= width * .5 + .001; y += .2)
    {
      p.push_back({cx + x, cy + y, height + .25, uint32_t(p.size())});
    }
  }
  for (double z = .25; z <= height + .251; z += .2)
  {
    for (double x = -length * .5; x <= length * .5 + .001; x += .2)
    {
      for (double y:{-width * .5, width*.5})
      {
        p.push_back({cx + x, cy + y, z, uint32_t(p.size())});
      }
    }
    for (double y = -width * .5; y <= width * .5 + .001; y += .2)
    {
      for (double x:{-length * .5, length*.5})
      {
        p.push_back({cx + x, cy + y, z, uint32_t(p.size())});
      }
    }
  }
}
int64_t stamp(int frame)
{
  return 1000000000LL + frame * 100000000LL;
}
Result frame(Core&c, int f, double x = 8, double length = 8)
{
  auto p = floor_scene();
  vehicle(p, x, 0, length);
  return c.process(stamp(f), p);
}
int main()
{
  try {
    Config cfg;
    cfg.stopped_sec = 1;
    Core c(cfg);
    Result r;
    for (int f = 0; f < 35; ++f)
    {
      r = frame(c, f);
    }
    check(r.ground.valid, "Ground must be directly observed");
    check(r.objects.size() == 1, "Single raw vehicle must remain one object");
    auto id = r.objects[0].id;
    check(r.objects[0].confirmed && r.objects[0].vehicle,
          "Stationary vehicle must confirm without motion/history");
    check(r.objects[0].extent_locked, "Two-second geometry confirmation");
    check(r.objects[0].status == "stopped", "Stationary observed vehicle should stop after dwell");
    const double full = r.objects[0].size.x();
    check(full > 7.8 && full < 8.2, "Observed full vehicle length");
    for (int f = 35; f < 41; ++f)
    {
      r = frame(c, f, 6, 4);
    }
    check(r.objects.size() == 1 && r.objects[0].id == id, "Partial body must retain same ID");
    check(r.objects[0].partial, "Partial body evidence must be explicit");
    check(std::abs(r.objects[0].size.x() - full) < 1e-8 && r.objects[0].observed_size.x() < 4.1,
          "Retained extent separate from current smaller raw observation");
    check(r.objects[0].motion == "STATIONARY", "Partial endpoint shift must not create motion");
    for (int f = 41; f < 48; ++f)
    {
      r = frame(c, f, 9, 10);
    }
    check(r.objects[0].size.x() > 9.8, "Larger same body must expand after temporal support");
    r = c.process(stamp(48), {}, false);
    check(r.objects.size() == 1 && r.objects[0].observation == "HELD" && r.objects[0].indices.empty(),
          "Loss may hold but must not invent direct raw points");
    r = c.process(stamp(61), {}, false);
    check(r.objects.empty(), "Public stale tracks expire by sensor time");
    Core boot(cfg);
    auto b = frame(boot, 0);
    check(!b.objects.empty() && !b.objects[0].confirmed && b.objects[0].status == "stopping",
          "Reboot cannot manufacture history");
    auto duplicate = frame(boot, 0);
    check(duplicate.quality == "DUPLICATE_SENSOR_TIMESTAMP", "Duplicate sensor timestamps rejected");
    auto invalid = boot.process(0, {});
    check(invalid.quality == "INVALID_SENSOR_TIMESTAMP",
          "Invalid sensor clock cannot be replaced by wall clock");
    auto regressed = boot.process(500000000, floor_scene());
    check(regressed.quality == "TIMESTAMP_REGRESSION_NEW_SESSION",
          "Timestamp regression must reset temporal evidence");
    Core moving(cfg);
    for (int f = 0; f < 70; ++f)
    {
      r = frame(moving, f, 8 + .06 * f);
    }
    check(r.objects.size() == 1 && r.objects[0].status == "entering",
          "World-range increasing observed motion is entering");
    Core exiting(cfg);
    for (int f = 0; f < 70; ++f)
    {
      r = frame(exiting, f, 16 - .06 * f);
    }
    check(r.objects.size() == 1 && r.objects[0].status == "exiting",
          "World-range decreasing observed motion is exiting");
    Core partial_motion(cfg);
    for(int f=0;f<35;++f)r=frame(partial_motion,f);
    for(int f=35;f<75;++f)r=frame(partial_motion,f,6+.06*(f-35),4);
    check(r.objects.size()==1&&r.objects[0].partial&&r.objects[0].size.x()>7.8,"Moving fragment must retain full extent");
    check(r.objects[0].center.x()>9.5&&r.objects[0].status=="entering","Same-shape moving partial Raw must provide real translation evidence");
    Core multi(cfg);
    for (int f = 0; f < 15; ++f)
    {
      auto p = floor_scene();
      vehicle(p, 8, -5, 8);
      vehicle(p, 12, 5, 12);
      r = multi.process(stamp(f), p);
    }
    check(r.objects.size() == 2 && r.objects[0].id != r.objects[1].id, "Independent multiple vehicle IDs");
    Core classification(cfg);
    for(int f=0;f<20;++f){auto p=floor_scene();vehicle(p,8,0,8,1.0);r=classification.process(stamp(f),p);}
    check(!r.objects.empty()&&!r.objects[0].vehicle,"UNKNOWN geometry cannot use track age as vehicle proof");
    for(int f=20;f<27;++f){auto p=floor_scene();vehicle(p);r=classification.process(stamp(f),p);if(f==20)check(!r.objects[0].vehicle,"One vehicle-like frame cannot promote old UNKNOWN track");}
    check(r.objects[0].vehicle,"Independent repeated vehicle geometry must confirm");
    // Field regression: UNKNOWN's oversized retained box must not become a vehicle model.
    Core contaminated(cfg);
    for(int f=0;f<30;++f){auto p=floor_scene();vehicle(p,8,0,22.5,4.4,4.8);r=contaminated.process(stamp(f),p);}
    check(!r.objects.empty()&&!r.objects[0].vehicle&&r.objects[0].extent_locked,
          "Oversized UNKNOWN starts with its own non-vehicle retained extent");
    for(int f=30;f<40;++f){auto p=floor_scene();vehicle(p,8,0,18.5,3.7,4.1);r=contaminated.process(stamp(f),p);}
    check(r.objects.size()==1&&r.objects[0].vehicle,"Independent valid vehicle geometry can confirm the same identity");
    check(r.objects[0].size.x()<=cfg.vehicle_max_length&&r.objects[0].size.y()<=cfg.vehicle_max_width&&r.objects[0].size.z()<=cfg.vehicle_max_height,
          "UNKNOWN retained geometry cannot contaminate a confirmed vehicle's dimensions");
    const auto a = assignment({{.1, .2}, {.11, 10}}, 1.5);
    check(a.size() == 2 && a[0] == 1 && a[1] == 0, "Global assignment must avoid greedy ID stealing");
    Core unknown(cfg);
    auto nofloor = std::vector<Point>{{1, 1, 3, 0}, {2, 2, 3, 1}};
    r = unknown.process(stamp(0), nofloor);
    check(!r.ground.valid && r.foreground.empty() && r.objects.empty(),
          "Unobserved floor must remain unknown");
    auto p = floor_scene();
    const auto below = p.size();
    p.push_back({1.01, 1.01, .199, uint32_t(below)});
    p.push_back({1.02, 1.02, .2, uint32_t(below + 1)});
    p.push_back({1.03, 1.03, .201, uint32_t(below + 2)});
    p.push_back({NAN, 0, 1, uint32_t(below + 3)});
    Core threshold(cfg);
    r = threshold.process(stamp(0), p);
    check(std::find(r.foreground.begin(), r.foreground.end(), below) == r.foreground.end(),
          "Below 0.2m excluded");
    check(std::find(r.foreground.begin(), r.foreground.end(), below + 1) != r.foreground.end(),
          "Exactly 0.2m included");
    check(std::find(r.foreground.begin(), r.foreground.end(), below + 2) != r.foreground.end(),
          "Above 0.2m included");
    // Field-labelled elevated ground returns: a thin ramp joins the observed floor.
    auto noisy=floor_scene();
    noisy.erase(std::remove_if(noisy.begin(),noisy.end(),[](const Point&p){return (p.x>=0&&p.x<=8&&std::abs(p.y)<=2)||(p.x>=11&&p.x<=15&&std::abs(p.y)<=1.5);}),noisy.end());
    const auto ramp_start=noisy.size();
    for(int ix=0;ix<=80;++ix)for(int iy=-20;iy<=20;++iy)noisy.push_back({ix*.1,iy*.1,.8-ix*.01,uint32_t(noisy.size())});
    const auto ramp_end=noisy.size();
    // A floating flat load surface is NOT connected to ground and must stay Raw foreground.
    const auto bed_start=noisy.size();
    for(int ix=110;ix<=150;++ix)for(int iy=-15;iy<=15;++iy)noisy.push_back({ix*.1,iy*.1,.8,uint32_t(noisy.size())});
    const auto bed_end=noisy.size();
    vehicle(noisy,-4,0,8);
    Core ground_noise(cfg);r=ground_noise.process(stamp(0),noisy);
    int ramp_kept=0,bed_kept=0;
    for(auto i:r.foreground){ramp_kept+=i>=ramp_start&&i<ramp_end;bed_kept+=i>=bed_start&&i<bed_end;}
    check(r.ground.valid&&!r.ground.local_surfaces.empty(),"Current-frame ground-connected local surfaces must have real support");
    check(ramp_kept<int((ramp_end-ramp_start)*.05),"Ground-connected rising Raw noise must not stretch vehicle boxes");
    check(bed_kept>int((bed_end-bed_start)*.95),"Disconnected horizontal cargo/bed must not become ground");
    check(std::any_of(r.observations.begin(),r.observations.end(),[](const Observation&o){return o.vehicle&&o.size.x()>7.8&&o.size.x()<8.2;}),
          "A real vehicle touching raised ground noise must retain its body without the floor extension");
    Config roi = cfg;
    roi.require_watch = true;
    roi.zones.push_back({"left", "watch", true, {-12, 3, 3, -12}, {-12, -12, 12, 12}});
    Core mask(roi);
    auto masked = floor_scene();
    vehicle(masked, 12);
    r = mask.process(stamp(0), masked);
    check(r.objects.empty(), "Watch-only ROI must reject excluded vehicle");
    auto slope = floor_scene();
    for (auto&v:slope)
    {
      v.z = .01 * v.x + .02 * v.y + .1;
    }
    Core sloped(cfg);
    r = sloped.process(stamp(0), slope);
    check(r.ground.valid && std::abs(r.ground.a - .01) < .001 && std::abs(r.ground.b - .02) < .001,
          "Current raw slope fit");
    std::cout <<
      "GeneralV2 controlled tests passed: ground, 0.2m boundary, stationary/reboot, direct/held/loss, partial/stable/growing dimensions, multi-ID/global assignment, entry/exit, ROI, slope, sensor timestamps\n";
    return 0;
  }
  catch (const std::exception&e){
    std::cerr << "GeneralV2 test failure: " << e.what() << '\n';
    return 1;
  }
}
