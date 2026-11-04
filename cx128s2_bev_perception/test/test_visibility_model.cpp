#include <cx128s2_bev_perception/visibility_model.hpp>
#include <cx128s2_bev_perception/vehicle_surface_tracking.hpp>

#include <cstdlib>
#include <cstdio>
#include <vector>

using namespace cx128s2_bev_perception;

#define require(condition) do { if (!(condition)) { \
  std::fprintf(stderr, "visibility test failed at line %d: %s\n", __LINE__, #condition); \
  std::abort(); } } while (0)

int main() {
  VisibilityModel model;
  VisibilityConfig config;
  model.configure(config);
  tf2::Transform pose;
  pose.setIdentity();
  pose.setOrigin(tf2::Vector3(0.0, 0.0, 7.0));
  model.setPose(pose);

  require(model.evaluatePoint(5.0, 0.0, 7.0).reason ==
         VisibilityReason::OUT_OF_HORIZONTAL_FOV);
  require(model.evaluatePoint(0.0, 0.05, 7.0).reason == VisibilityReason::TOO_CLOSE);
  require(model.evaluatePoint(0.0, 5.0, 1.0).reason == VisibilityReason::NEAR_BLIND);
  require(model.evaluatePoint(0.0, 5.0, 6.0).reason == VisibilityReason::UNKNOWN);
  require(model.evaluatePoint(0.0, 5.0, 7.0).reason == VisibilityReason::UNKNOWN);

  PointXYZIRT farther;
  farther.y = 10.0f;
  farther.z = 7.0f;
  model.buildDepth({farther}, {2});
  require(model.evaluatePoint(0.0, 5.0, 7.0, 1).reason == VisibilityReason::VISIBLE_EMPTY);

  PointXYZIRT nearer;
  nearer.y = 3.0f;
  nearer.z = 7.0f;
  model.buildDepth({nearer}, {2});
  require(model.evaluatePoint(0.0, 5.0, 7.0, 1).reason == VisibilityReason::OCCLUDED_BY_OTHER);
  require(model.evaluatePoint(0.0, 5.0, 7.0, 2).reason == VisibilityReason::SELF_OCCLUSION_POSSIBLE);

  std::vector<PointXYZIRT> screen;
  for (int ix = -20; ix <= 20; ++ix) {
    for (int iz = -20; iz <= 20; ++iz) {
      PointXYZIRT p;
      p.x = ix * 0.01f;
      p.y = 3.0f;
      p.z = 7.0f + iz * 0.01f;
      screen.push_back(p);
    }
  }
  model.buildDepth(screen, std::vector<int>(screen.size(), 2));
  VisibilityBox predicted{-0.2, 0.2, 5.0, 5.2, 6.8, 7.2};
  const auto hidden = model.evaluateBox(predicted, -2, true);
  require(hidden.reason == VisibilityReason::FULLY_OCCLUDED_LIKELY);
  require(hidden.has_occluder);
  // The same screen with no owner contributes to aggregate occlusion, but
  // does not provide a known occluder that can relax raw support requirements.
  model.buildDepth(screen, std::vector<int>(screen.size(), -1));
  const auto unknown_hidden = model.evaluateBox(predicted, -2, true);
  require(unknown_hidden.occluded_fraction > 0.3);
  require(!unknown_hidden.has_occluder);
  double effective_expected = 0.0;
  require(!vehicle_surface::enoughSparseSupport(
      6, 2, 8400.0, unknown_hidden.occluded_fraction,
      unknown_hidden.has_occluder, &effective_expected));
  require(effective_expected == 8400.0);

  model.buildDepth({}, {});
  require(model.evaluatePoint(0.0, 5.0, 7.0).reason == VisibilityReason::UNKNOWN);

  model.clearStaticCache();
  for (double z : {0.0, 2.0, 4.0}) model.cacheStaticPoint(0.0, 20.0, z);
  const size_t fixed_cells = model.staticCellCount();
  require(fixed_cells == 3);
  for (int i = 0; i < 1000; ++i) {
    const double y = 10.0 + i * 0.025;
    (void)model.evaluateBox({-1.0, 1.0, y, y + 2.0, 6.5, 7.5}, -2);
  }
  require(model.staticCellCount() == fixed_cells);
  require(model.staticCacheMisses() > 0);

  const auto close = model.evaluateAdaptiveBox({-1.0, 1.0, 10.0, 12.0, 6.5, 7.5}, -2, false, 0.1);
  const auto distant = model.evaluateAdaptiveBox({-1.0, 1.0, 50.0, 52.0, 6.5, 7.5}, -2, false, 0.1);
  require(close.adaptive_expected_rays > distant.adaptive_expected_rays);
  require(close.horizontal_point_spacing_m < distant.horizontal_point_spacing_m);
  require(close.current_observation_weight > distant.current_observation_weight);
  const auto uncertain = model.evaluateAdaptiveBox(
      {-1.0, 1.0, 50.0, 52.0, 6.5, 7.5}, -2, false, 2.0);
  require(uncertain.adaptive_expected_rays == distant.adaptive_expected_rays);
  require(uncertain.adaptive_azimuth_width_deg > distant.adaptive_azimuth_width_deg);
  model.setRayElevationSpacing(0.25);
  const auto distant_sparse_rings = model.evaluateAdaptiveBox(
      {-1.0, 1.0, 50.0, 52.0, 6.5, 7.5}, -2, false, 0.1);
  require(distant_sparse_rings.adaptive_expected_rays < distant.adaptive_expected_rays);
  require(distant_sparse_rings.vertical_point_spacing_m > distant.vertical_point_spacing_m);
  const auto blind = model.evaluateAdaptiveBox({-1.0, 1.0, 3.0, 4.0, 0.0, 2.0}, -2, false, 0.1);
  require(blind.current_observation_weight == 0.0);
  require(blind.memory_weight == 1.0);
  // One vehicle moves far -> mid -> near -> blind -> far again. Expected
  // support follows angular size and blind geometry overrides the near rule.
  const auto mid = model.evaluateAdaptiveBox({-1.0, 1.0, 25.0, 27.0, 6.5, 7.5}, -2, false, 0.1);
  const auto near_again = model.evaluateAdaptiveBox({-1.0, 1.0, 10.0, 12.0, 6.5, 7.5}, -2, false, 0.1);
  require(distant_sparse_rings.adaptive_expected_rays < mid.adaptive_expected_rays);
  require(mid.adaptive_expected_rays < near_again.adaptive_expected_rays);
  require(near_again.adaptive_expected_rays > blind.adaptive_expected_rays);
  // A: real angular expectations from the existing visibility model at
  // sensor distances 3--5 m reject tiny support, including known occlusion.
  for (double distance : {3.0, 4.0, 5.0}) {
    const auto very_close = model.evaluateAdaptiveBox(
        {-1.0, 1.0, distance - 0.5, distance + 0.5, 6.5, 7.5}, -2, true, 0.1);
    require(std::abs(very_close.sensor_distance - distance) < 1e-9);
    require(very_close.adaptive_expected_rays > 120);
    for (int bins = 3; bins <= 6; ++bins)
      require(!vehicle_surface::enoughSparseSupport(
          bins, 2, very_close.adaptive_expected_rays, 1.0, true));
    // B: sufficient visible support can pass under partial known occlusion.
    const int visible_support = static_cast<int>(
        std::ceil(very_close.adaptive_expected_rays * 0.06));
    require(vehicle_surface::enoughSparseSupport(
        visible_support, 2, very_close.adaptive_expected_rays, 0.5, true));
  }
  const auto far_again = model.evaluateAdaptiveBox({-1.0, 1.0, 50.0, 52.0, 6.5, 7.5}, -2, false, 0.1);
  require(far_again.adaptive_expected_rays == distant_sparse_rings.adaptive_expected_rays);
  pose.setOrigin(tf2::Vector3(0.0, 0.0, 7.1));
  require(model.setPose(pose));
  require(model.staticCellCount() == 0);
  // Nonuniform measured channel coverage is opt-in for confirmed fronts.
  VisibilityModel measured;
  VisibilityConfig measured_config;
  measured_config.ray_elevation_spacing_deg=0.1;
  measured.configure(measured_config);measured.setPose(pose);
  const VisibilityBox measured_box{-.2,.2,9.9,10.1,7.5,8.0};
  const auto uniform=measured.evaluateAdaptiveBox(measured_box,-1,false,0.0);
  std::vector<std::pair<double,double>> bands;
  for(int ring=0;ring<128;++ring) {
    const double angle=-12.7+.2*ring;
    bands.push_back({angle-.005,angle+.005});
  }
  measured.setMeasuredRingProfile(bands);
  const auto profiled=measured.evaluateAdaptiveBox(measured_box,-1,false,0.0,true);
  require(profiled.adaptive_expected_rays>0);
  require(profiled.adaptive_expected_rays<uniform.adaptive_expected_rays);
  require(measured.evaluateAdaptiveBox(measured_box,-1,false,0.0).adaptive_expected_rays==
      uniform.adaptive_expected_rays); // Ordinary and Crane calls stay unchanged.
  measured.configure(measured_config);measured.setPose(pose);
  require(measured.evaluateAdaptiveBox(measured_box,-1,false,0.0,true).adaptive_expected_rays==
      profiled.adaptive_expected_rays); // Depth-cache configure preserves observed profile.
  bands.pop_back();measured.setMeasuredRingProfile(bands);
  require(measured.evaluateAdaptiveBox(measured_box,-1,false,0.0,true).adaptive_expected_rays==
      uniform.adaptive_expected_rays); // Missing channels cannot lower expectation.
  bands.assign(128,{-10,-9});measured.setMeasuredRingProfile(bands);
  require(measured.evaluateAdaptiveBox(measured_box,-1,false,0.0,true).adaptive_expected_rays==0);
  require(measured.evaluateAdaptiveBox(measured_box,-1,false,0.0).adaptive_expected_rays==
      uniform.adaptive_expected_rays);
  bands[0]={0,5};measured.setMeasuredRingProfile(bands);
  require(measured.evaluateAdaptiveBox(measured_box,-1,false,0.0,true).adaptive_expected_rays>0);
  std::puts("Measured full ring bands: opt-in footprint, conservative interval overlap, incomplete fallback and legacy API passed");
  return 0;
}
