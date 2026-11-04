#include <cassert>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <limits>
#include <string>
#include <vector>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

#undef assert
#define assert(cond) do { if (!(cond)) { std::cerr << "Assertion failed: " #cond " at line " << __LINE__ << std::endl; std::abort(); } } while (0)

#include <cx128s2_bev_perception/crane_beam_detector.hpp>

using cx128s2_bev_perception::BeamMeasurementSource;
using cx128s2_bev_perception::AcquisitionResetReason;
using cx128s2_bev_perception::CraneBeamConfig;
using cx128s2_bev_perception::CraneBeamDetector;
using cx128s2_bev_perception::CraneBeamState;
using cx128s2_bev_perception::PointXYZIRT;

// Helper to create synthetic points for beam, support surfaces, and background
static void generateSyntheticScene(
    std::vector<PointXYZIRT> & points,
    std::vector<int> & valid_indices,
    double beam_cx, double beam_cy, double beam_z_center,
    double beam_length, double beam_yaw_rad,
    double bg_z,
    bool include_beam = true,
    bool include_bg = true,
    double support_deck_z = -999.0,
    double deck_cx = 0.0, double deck_cy = 0.0,
    double deck_half_x = 3.0, double deck_half_y = 1.5,
    bool ground_dominated_beam_roi = false,
    bool include_neighbor_beam = false,
    double neighbor_cy = 1.5) {
  points.clear();
  valid_indices.clear();

  // 1. Generate background / ground points across [-6, 6] x [-6, 6]
  if (include_bg) {
    for (double x = -6.0; x <= 6.0; x += 0.25) {
      for (double y = -6.0; y <= 6.0; y += 0.25) {
        PointXYZIRT p;
        p.x = static_cast<float>(x);
        p.y = static_cast<float>(y);
        p.z = static_cast<float>(bg_z);
        valid_indices.push_back(static_cast<int>(points.size()));
        points.push_back(p);
      }
    }
  }

  // 2. Elevated support deck (e.g. truck flatbed at Z = support_deck_z)
  if (support_deck_z > -900.0) {
    for (double x = deck_cx - deck_half_x; x <= deck_cx + deck_half_x; x += 0.2) {
      for (double y = deck_cy - deck_half_y; y <= deck_cy + deck_half_y; y += 0.2) {
        PointXYZIRT p;
        p.x = static_cast<float>(x);
        p.y = static_cast<float>(y);
        p.z = static_cast<float>(support_deck_z);
        valid_indices.push_back(static_cast<int>(points.size()));
        points.push_back(p);
      }
    }
  }

  // 3. Ground points inside Beam ROI if ground_dominated_beam_roi is requested
  if (ground_dominated_beam_roi) {
    for (double x = -1.8; x <= 1.8; x += 0.15) {
      for (double y = -1.8; y <= 1.8; y += 0.15) {
        PointXYZIRT p;
        p.x = static_cast<float>(x);
        p.y = static_cast<float>(y);
        p.z = static_cast<float>(bg_z);
        valid_indices.push_back(static_cast<int>(points.size()));
        points.push_back(p);
      }
    }
  }

  // 4. Target Beam (3D box: length, width=0.3, height=0.4)
  if (include_beam) {
    const double cos_y = std::cos(beam_yaw_rad);
    const double sin_y = std::sin(beam_yaw_rad);
    const double half_len = beam_length / 2.0;

    for (double s = -half_len; s <= half_len; s += 0.10) {
      for (double w = -0.15; w <= 0.15; w += 0.10) {
        for (double dz = -0.20; dz <= 0.20; dz += 0.10) {
          PointXYZIRT p;
          p.x = static_cast<float>(beam_cx + s * cos_y - w * sin_y);
          p.y = static_cast<float>(beam_cy + s * sin_y + w * cos_y);
          p.z = static_cast<float>(beam_z_center + dz);
          valid_indices.push_back(static_cast<int>(points.size()));
          points.push_back(p);
        }
      }
    }
  }

  // 5. Neighbor Beam (parallel at neighbor_cy)
  if (include_neighbor_beam) {
    const double cos_y = std::cos(beam_yaw_rad);
    const double sin_y = std::sin(beam_yaw_rad);
    const double half_len = beam_length / 2.0;

    for (double s = -half_len; s <= half_len; s += 0.10) {
      for (double w = -0.15; w <= 0.15; w += 0.10) {
        for (double dz = -0.20; dz <= 0.20; dz += 0.10) {
          PointXYZIRT p;
          p.x = static_cast<float>(beam_cx + s * cos_y - w * sin_y);
          p.y = static_cast<float>(neighbor_cy + s * sin_y + w * cos_y);
          p.z = static_cast<float>(beam_z_center + dz);
          valid_indices.push_back(static_cast<int>(points.size()));
          points.push_back(p);
        }
      }
    }
  }
}

static void addBox(
    std::vector<PointXYZIRT> & points,
    std::vector<int> & valid_indices,
    double cx, double cy, double cz,
    double lx, double ly, double lz,
    double step = 0.10) {
  for (double x = cx - lx / 2.0; x <= cx + lx / 2.0; x += step) {
    for (double y = cy - ly / 2.0; y <= cy + ly / 2.0; y += step) {
      for (double z = cz - lz / 2.0; z <= cz + lz / 2.0; z += step) {
        PointXYZIRT p;
        p.x = static_cast<float>(x);
        p.y = static_cast<float>(y);
        p.z = static_cast<float>(z);
        valid_indices.push_back(static_cast<int>(points.size()));
        points.push_back(p);
      }
    }
  }
}

static void addCylinder(
    std::vector<PointXYZIRT> & points,
    std::vector<int> & valid_indices,
    double cx, double cy,
    double z_min, double z_max,
    double radius,
    double step = 0.05) {
  for (double z = z_min; z <= z_max; z += step) {
    for (double th = 0; th < 2 * M_PI; th += 0.3) {
      PointXYZIRT p;
      p.x = static_cast<float>(cx + radius * std::cos(th));
      p.y = static_cast<float>(cy + radius * std::sin(th));
      p.z = static_cast<float>(z);
      valid_indices.push_back(static_cast<int>(points.size()));
      points.push_back(p);
    }
  }
}

static void addTiltedBeam(
    std::vector<PointXYZIRT> & points,
    std::vector<int> & valid_indices,
    double cx, double cy, double cz,
    double length, double width, double height,
    double yaw_rad, double pitch_rad,
    double step = 0.10) {
  const double cos_y = std::cos(yaw_rad);
  const double sin_y = std::sin(yaw_rad);
  const double cos_p = std::cos(pitch_rad);
  const double sin_p = std::sin(pitch_rad);
  const double half_len = length / 2.0;
  const double half_w = width / 2.0;
  const double half_h = height / 2.0;

  for (double l = -half_len; l <= half_len; l += step) {
    for (double w = -half_w; w <= half_w; w += step) {
      for (double h = -half_h; h <= half_h; h += step) {
        double lx = l * cos_p;
        double ly = w;
        double lz = h + l * sin_p;

        PointXYZIRT p;
        p.x = static_cast<float>(cx + lx * cos_y - ly * sin_y);
        p.y = static_cast<float>(cy + lx * sin_y + ly * cos_y);
        p.z = static_cast<float>(cz + lz);
        valid_indices.push_back(static_cast<int>(points.size()));
        points.push_back(p);
      }
    }
  }
}

int main() {
  std::cout << "[Test] Running CraneBeamDetector Unit Tests..." << std::endl;

  CraneBeamConfig cfg;
  cfg.beam_roi_x_min = -2.0;
  cfg.beam_roi_x_max = 2.0;
  cfg.beam_roi_y_min = -2.0;
  cfg.beam_roi_y_max = 2.0;
  cfg.beam_roi_z_min = -0.5;
  cfg.beam_roi_z_max = 8.0;

  cfg.bg_roi_x_min = -8.0;
  cfg.bg_roi_x_max = 8.0;
  cfg.bg_roi_y_min = -8.0;
  cfg.bg_roi_y_max = 8.0;
  cfg.bg_roi_z_min = -0.5;
  cfg.bg_roi_z_max = 8.0;

  cfg.lift_dz_threshold = 0.05;
  cfg.lower_dz_threshold = 0.05;
  cfg.stop_dz_threshold = 0.02;
  cfg.ground_tolerance = 0.05;

  cfg.lift_confirm_frames = 3;
  cfg.suspend_confirm_frames = 3;
  cfg.lower_confirm_frames = 3;
  cfg.placed_confirm_frames = 3;
  cfg.complete_confirm_frames = 4;

  cfg.length_lock_frames = 3;
  cfg.length_stable_threshold = 0.15;
  cfg.use_z_ema = false;  // deterministic step tests

  cfg.beam_z_histogram_bin_size = 0.05;
  cfg.beam_ground_clearance = 0.12;
  cfg.tracking_gate_xy = 1.2;
  cfg.tracking_gate_z = 0.50;
  cfg.tracking_gate_yaw = 0.35;
  cfg.tracking_gate_length = 1.0;
  cfg.support_search_margin_xy = 0.60;
  cfg.min_support_points = 15;
  cfg.placement_tolerance = 0.10;
  cfg.beam_default_height = 0.40;

  CraneBeamDetector detector(cfg);
  std::vector<PointXYZIRT> points;
  std::vector<int> valid_indices;

  // Production height is measured body bottom minus actual local ground.
  // Acquisition needs >=3m; a confirmed Hook can descend below 3m.
  for (int side : {0, 1}) {
    cx128s2_bev_perception::CraneHookConfig hc;
    const double x = side == 0 ? -0.9 : 0.9;
    const auto scene = [&](double height) {
      points.clear(); valid_indices.clear();
      addBox(points, valid_indices, x, 0, height + .6 + .175, .2, .2, .35, .025);
      addBox(points, valid_indices, x, 0, .6, .4, .4, 0, .025);
    };
    for (double height : {2.5, 2.9, 3.0, 3.5, 5.0}) {
      cx128s2_bev_perception::CraneHookDetector hook(hc);
      for (int frame = 0; frame < 6; ++frame) {
        scene(height);
        const auto & out = hook.process(points, valid_indices, frame * .1);
        const auto & measured = side == 0 ? out.left : out.right;
        if (measured.detected != (height >= 3.0)) std::cerr << "height=" << height
            << " ground=" << measured.ground_z << " bottom=" << measured.body_min[2]
            << " reason=" << measured.selection_reason << " " << measured.candidate_details << std::endl;
        assert(measured.detected == (height >= 3.0));
        if (height >= 3.0) assert(std::abs(measured.ground_distance - height) < .03);
        if (frame >= 2 && height >= 3.0) assert(measured.initialized && measured.reference_valid);
        assert(!out.prepare_ready); // being below 4m without descent is insufficient
      }
    }
    cx128s2_bev_perception::CraneHookDetector hook(hc);
    const std::vector<double> heights{4.3, 4.3, 4.3, 4.2, 4.1, 3.9, 3.8, 3.7, 3.6,
                                    3.5, 3.4, 3.3, 3.2, 3.1, 3.0, 2.9, 2.8, 2.7};
    for (size_t frame = 0; frame < heights.size(); ++frame) {
      scene(heights[frame]);
      const auto & out = hook.process(points, valid_indices, frame * .1);
      const auto & measured = side == 0 ? out.left : out.right;
      assert(measured.detected);
      assert(out.prepare_ready == (frame >= 5));
      assert(out.prepare_sides[side] == out.prepare_ready && !out.prepare_sides[1 - side]);
      if (frame >= 2) assert(measured.initialized);
    }
    hook.reset();
    const std::vector<double> ascending{3.5, 3.5, 3.5, 3.6, 3.7, 3.8, 3.9, 4.1};
    for (size_t frame = 0; frame < ascending.size(); ++frame) {
      scene(ascending[frame]);
      assert(!hook.process(points, valid_indices, frame * .1).prepare_ready);
    }
    hook.reset();
    for (int frame = 0; frame < 3; ++frame) {
      scene(4.1);
      assert(!hook.process(points, valid_indices, frame * .1).prepare_ready);
    }
    assert(!hook.process({}, {}, .3).prepare_ready);
    scene(3.9);
    assert(!hook.process(points, valid_indices, .4).prepare_ready); // prediction/dropout is not descent
    hook.reset();
    points.clear(); valid_indices.clear();
    addBox(points, valid_indices, x, 0, 3.8, .2, .2, .35, .025);
    const auto & no_ground = hook.process(points, valid_indices, .5);
    assert(!(side == 0 ? no_ground.left.detected : no_ground.right.detected));
  }
  std::cout << "[Hook height] local ground .6m, acquisition 2.5/3.5/5.0m, descending 4.1->3.9->2.7, ascending and dropout passed" << std::endl;

  // The previous Beam-only work-cycle assertions are replaced by measured
  // Hook-coupled cycles. Geometry, support, motion reference and reset are
  // exercised below without relying on automatic completion-to-IDLE.
  cfg.beam_roi_x_min = -0.2; cfg.beam_roi_x_max = 0.2;
  cfg.beam_roi_y_min = 0.8; cfg.beam_roi_y_max = 1.2;
  cfg.beam_roi_z_min = 0.3; cfg.beam_roi_z_max = 5.0;
  cfg.min_beam_cluster_points = 5;
  cfg.hook.prepare_confirm_frames = 3;
  cfg.hook.min_ground_points = 5;
  cfg.hook.min_body_points = 100;
  cfg.hook.roi_z_min = 1.2;
  cfg.hook.prepare_distance = 1.8; // Scaled legacy fixtures still exercise a downward crossing.
  cfg.hook_beam_dz_match_tolerance = 0.04;
  cfg.lift_dz_threshold = cfg.lower_dz_threshold = 0.025;

  // LEFT, RIGHT, BOTH: also exercise EMA and slow cumulative motion.
  // Movement consumes only tracks checked against a loaded reference. Keep
  // this synthetic profile independent of the installation's real Hook file.
  const std::string movement_reference_path = "/tmp/crane-hook-reference-unit-test.yaml";
  const std::string movement_reference_yaml = R"(
version: 2
frame_id: world
body_representation: xy_surface_in_z_density_band
window_size_z: 0.30
window_step_z: 0.02
tracking_measurement_std: 0.03
roi: [-1.5, -0.3, 0.3, 1.5, -0.8, 0.8, 6.2]
left:
  frames: 40
  median: [0.20, 0.20, 0.30, -0.12, -0.12, -0.15, 0.12, 0.12, 0.15, 300, 9, 0.5]
right:
  frames: 40
  median: [0.20, 0.20, 0.30, -0.12, -0.12, -0.15, 0.12, 0.12, 0.15, 300, 9, 0.5]
)";
  { std::ofstream file(movement_reference_path); file << movement_reference_yaml; }
  for (int mask : {1, 2, 3}) for (bool ema : {false, true}) for (bool cumulative : {false, true}) {
    auto cycle_cfg = cfg;
    cycle_cfg.hook.roi_z_min = 3.0;
    cycle_cfg.hook.prepare_distance = 4.0;
    cycle_cfg.use_z_ema = ema;
    cycle_cfg.cumulative_motion_enabled = cumulative;
    cycle_cfg.lift_confirm_frames = cycle_cfg.lower_confirm_frames = 10;
    cycle_cfg.complete_confirm_frames = 5;
    cycle_cfg.hook.prepare_confirm_frames = 5;
    cycle_cfg.hook_beam_dz_match_tolerance = 0.03;
    cycle_cfg.lift_dz_threshold = cycle_cfg.lower_dz_threshold = 0.05;
    cycle_cfg.motion_reference_mode = mask == 2 ? "absolute" : "relative_background";
    CraneBeamDetector cycle(cycle_cfg);
    std::string reference_report;
    assert(cycle.getHookDetector().loadReference(movement_reference_path, "world", reference_report));
    double stamp = 0.0, locked_length = 0.0;
    bool seen_lift = false, seen_suspend = false, seen_lower = false, seen_complete = false;
    int transitions = 0;
    for (int frame = 0; frame < 170; ++frame) {
      // Initial preparation -> 0.6m lift -> stop -> lower -> stop -> complete.
      const double step = cumulative ? 0.02 : 0.06;
      const double displacement = frame < 15 ? 0.0 : frame < 45 ? (frame - 14) * step :
          frame < 75 ? 30 * step : frame < 105 ? (104 - frame) * step : 0.0;
      const double background = frame >= 160 ? (frame - 159) * 0.12 : frame == 145 ? (cumulative ? 4.0 : 0.4) : 0.0;
      const bool beam_visible = frame != 34 && frame != 35 && frame != 145 && !(frame >= 105 && frame <= 107) && frame < 150;
      const double center_x = 0.05 * std::sin(frame * 0.1);
      generateSyntheticScene(points, valid_indices, center_x, 1.0, 0.65 + displacement,
          4.0, 0.0, background, beam_visible, true, 0.0);
      for (int side = 0; side < 2; ++side) {
        const double hook_delta = (mask & (1 << side)) ? displacement : frame >= 130 ? std::min(0.3, (frame - 129) * 0.02) : 0.0;
        const bool hook_visible = (frame < 150 || frame >= 155) && frame != 145 &&
            !(side == 0 && (mask & 1) && frame >= 36 && frame <= 38) &&
            !((mask & (1 << side)) && ((frame >= 46 && frame <= 48) ||
              (frame >= 78 && frame <= 80) || (frame >= 108 && frame <= 110)));
        // Completion now needs measured Hook XY movement as well as the
        // unchanged Strict-FSM background direction/reset confirmations.
        const double crane_xy = frame >= 160 ? (frame - 159) * .04 : 0.0;
        if (hook_visible) addBox(points, valid_indices, (side == 0 ? -0.9 : 0.9) + crane_xy,
            0.10 * std::sin(frame * 0.2), (frame < 3 ? 4.3 : frame == 3 ? 4.15 : frame == 4 ? 4.0 : 3.9) + hook_delta, 0.20, 0.20, 0.35, 0.025);
      }
      stamp += 0.1;
      const auto & out = cycle.process(points, valid_indices, stamp);
      if (out.length_locked && locked_length == 0.0) locked_length = out.beam_length_locked;
      if (frame >= 8 && frame < 160) {
        assert(out.length_locked && locked_length > 3.8);
        assert(out.beam_length_locked == locked_length && out.beam_length_raw == locked_length);
        if (frame > 8) assert(out.measurement_source == BeamMeasurementSource::TRACKED_CLUSTER);
      }
      if (frame < 15) assert(out.state == CraneBeamState::IDLE);
      if (seen_lift && frame < 45) assert(out.state == CraneBeamState::LIFTING);
      // The world-Z Hook fixture finishes ten slow-motion confirmations by frame 101.
      if (frame >= 101 && frame <= 110) assert(out.state == CraneBeamState::LOWERING);
      if (out.state_changed) {
        ++transitions;
        if (out.curr_state_str == "IDLE") {
          assert(!out.length_locked && !out.hook.prepare_ready && !out.work_complete);
          assert(!out.active_left && !out.active_right);
        }
        std::cout << "transition " << frame << " " << out.prev_state_str << " -> " << out.curr_state_str << std::endl;
        assert((out.prev_state_str == "IDLE" && out.curr_state_str == "LIFTING") ||
          (out.prev_state_str == "LIFTING" && out.curr_state_str == "SUSPENDED") ||
          (out.prev_state_str == "SUSPENDED" && out.curr_state_str == "LOWERING") ||
          (out.prev_state_str == "LOWERING" && out.curr_state_str == "COMPLETE_CONFIRM") ||
          (out.prev_state_str == "COMPLETE_CONFIRM" && out.curr_state_str == "IDLE"));
      }
      seen_lift |= out.state == CraneBeamState::LIFTING;
      seen_suspend |= out.state == CraneBeamState::SUSPENDED;
      seen_lower |= out.state == CraneBeamState::LOWERING;
      seen_complete |= out.state == CraneBeamState::COMPLETE_CONFIRM;
      if (out.state != CraneBeamState::IDLE) {
        assert(out.active_left == bool(mask & 1));
        assert(out.active_right == bool(mask & 2));
      }
      if (frame >= 140 && frame < 160) {
        assert(out.state == CraneBeamState::COMPLETE_CONFIRM && out.work_complete);
        assert(out.length_locked && !out.working_status);
      }
      if (frame == 169) {
        assert(out.state == CraneBeamState::IDLE && !out.active_left && !out.active_right && !out.work_complete);
      }
    }
    assert(seen_lift && seen_suspend && seen_lower && seen_complete && transitions == 5);
    std::cout << "[FSM cycle] mask=" << mask << " EMA=" << ema << " cumulative=" << cumulative << " all five transitions passed" << std::endl;
  }

  // Prepare works with only one Hook present, not just one moving Hook.
  for (int side : {0, 1}) {
    CraneBeamDetector one(cfg);
    for (int frame = 0; frame < 40; ++frame) {
      const double dz = frame < 10 ? 0.0 : (frame - 9) * 0.02;
      generateSyntheticScene(points, valid_indices, 0.0, 1.0, 0.65 + dz, 4.0, 0.0, 0.0);
      addBox(points, valid_indices, side == 0 ? -0.9 : 0.9, 0.0, (frame < 3 ? 2.0 : frame == 3 ? 1.85 : 1.65) + dz, 0.2, 0.2, 0.3, 0.025);
      const auto & out = one.process(points, valid_indices, frame * 0.1);
      if (frame == 39) assert(out.state == CraneBeamState::LIFTING &&
          out.active_left == (side == 0) && out.active_right == (side == 1));
    }
  }
  std::cout << "[FSM] single visible side preparation and lifting passed" << std::endl;

  // The locked object travels outside its fixed acquisition ROI in XY.
  {
    CraneBeamDetector translated(cfg);
    double length = 0.0;
    for (int frame = 0; frame < 40; ++frame) {
      const double displacement = frame < 10 ? 0.0 : (frame - 9) * 0.02;
      const double x = frame < 10 ? 0.0 : (frame - 9) * 0.08;
      const double y = frame < 10 ? 1.0 : 1.0 + (frame - 9) * 0.04;
      generateSyntheticScene(points, valid_indices, x, y, 0.65 + displacement, 4, 0, 0);
      addBox(points, valid_indices, -0.9, 0, (frame < 3 ? 2.0 : frame == 3 ? 1.85 : 1.65) + displacement, 0.2, 0.2, 0.3, 0.025);
      const auto & out = translated.process(points, valid_indices, frame * 0.1);
      if (frame == 5) length = out.beam_length_locked;
      if (frame >= 10) {
        assert(out.beam_tracking_state == "MEASURED" && out.beam_length_locked == length);
        assert(std::abs(out.cluster_centroid_x - x) < 0.15 && std::abs(out.cluster_centroid_y - y) < 0.15);
        assert(out.measurement_source == BeamMeasurementSource::TRACKED_CLUSTER);
      }
    }
  }
  std::cout << "[Local tracking] XY translation outside initial ROI passed" << std::endl;

  // Wrong-direction Hook, isolated spikes and missing measurements cannot lift.
  for (int scenario = 0; scenario < 7; ++scenario) {
    CraneBeamDetector guarded(cfg);
    for (int frame = 0; frame < 45; ++frame) {
      double beam_dz = 0.0, hook_dz = 0.0;
      if (scenario == 0 && frame >= 12) { beam_dz = (frame - 11) * 0.02; hook_dz = -beam_dz; }
      if (scenario == 1 && frame == 15) hook_dz = 0.28;
      if (scenario == 2 && frame == 15) beam_dz = 0.28;
      if (scenario == 3 && frame == 15) beam_dz = hook_dz = 0.28;
      if (scenario == 4 && frame >= 15 && frame <= 17) beam_dz = (frame - 14) * 0.04;
      bool beam = !(scenario == 5 && frame >= 15 && frame <= 17);
      if (scenario == 6 && frame >= 15 && frame <= 20) beam = false;
      generateSyntheticScene(points, valid_indices, 0.0, 1.0, 0.65 + beam_dz, 4.0, 0.0, 0.0, beam);
      if (!(scenario == 4 && frame >= 15 && frame <= 17))
        addBox(points, valid_indices, -0.9, 0.0, (frame < 3 ? 2.0 : frame == 3 ? 1.85 : 1.65) + hook_dz, 0.2, 0.2, 0.3, 0.025);
      addBox(points, valid_indices, 0.9, 0.0, frame < 3 ? 2.0 : frame == 3 ? 1.85 : 1.65, 0.2, 0.2, 0.3, 0.025);
      const auto & out = guarded.process(points, valid_indices, 0.1 * (frame + 1));
      assert(out.state == CraneBeamState::IDLE);
      if (scenario == 4 && frame >= 15 && frame <= 17) assert(out.hook.left.tracking_state == "PREDICTED");
      if (scenario == 5 && frame >= 15 && frame <= 17) assert(out.beam_tracking_state == "PREDICTED" && out.length_locked);
      if (scenario == 6 && frame >= 19) assert(out.beam_tracking_state == "LOST" && out.length_locked);
    }
  }
  std::cout << "[FSM] wrong Hook, Hook/Beam spikes, prediction-only and long loss passed" << std::endl;

  // Geometry rejection retries safely; lock never switches to a nearby beam.
  for (int frame = 0; frame < 35; ++frame) {
    if (frame == 0) detector = CraneBeamDetector(cfg);
    generateSyntheticScene(points, valid_indices, 0.0, 1.0, 0.65, frame < 10 ? 0.5 : 4.0,
        0.0, 0.0, frame < 25, true, -999, 0, 0, 3, 1.5, true, frame >= 20, 2.5);
    addBox(points, valid_indices, -0.9, 0.0, frame < 3 ? 2.0 : frame == 3 ? 1.85 : 1.65, 0.2, 0.2, 0.3, 0.025);
    const auto & out = detector.process(points, valid_indices, (frame + 1) * 0.1);
    if (frame >= 5 && frame < 10) assert(!out.length_locked && !out.beam_cluster_valid && out.beam_z_valid);
    if (frame >= 10) assert(out.length_locked && out.beam_length_locked > 3.8);
    if (frame >= 29) assert(out.beam_tracking_state == "LOST" && !out.beam_cluster_valid);
    assert(out.state == CraneBeamState::IDLE);
  }
  detector.reset();
  assert(!detector.getHookDetector().getResult().prepare_ready);
  std::cout << "[Geometry] shape rejection/retry, ground exclusion, neighbor protection and reset passed" << std::endl;

  // Independent prepare sides, equality at threshold and confirmed hysteresis.
  {
    auto hc = cfg.hook; hc.tracking_confirm_frames = 1;
    cx128s2_bev_perception::CraneHookDetector ready(hc);
    for (int frame = 0; frame < 30; ++frame) {
      points.clear(); valid_indices.clear();
      const double z = frame == 0 ? 1.95 : frame < 5 ? 1.79 : frame < 20 ? (frame % 2 ? 1.79 : 1.81) : 2.05;
      addBox(points, valid_indices, -0.9, 0.0, z + .15, 0.2, 0.2, 0.3, 0.025);
      addBox(points, valid_indices, -0.9, 0.0, 0.0, 0.2, 0.2, 0.0, 0.025);
      const auto & out = ready.process(points, valid_indices, frame * 0.1);
      if (frame >= 3 && frame < 20) assert(out.prepare_ready && out.prepare_sides[0] && !out.prepare_sides[1]);
      if (frame >= 24) assert(!out.prepare_ready);
    }
    ready.reset();
    for (int frame = 0; frame < 10; ++frame) {
      points.clear(); valid_indices.clear();
      const double x = frame % 2 ? 0.9 : -0.9;
      addBox(points, valid_indices, x, 0.0, 1.65, 0.2, 0.2, 0.3, 0.025);
      addBox(points, valid_indices, x, 0.0, 0.0, 0.2, 0.2, 0.0, 0.025);
      assert(!ready.process(points, valid_indices, frame * 0.1).prepare_ready);
    }
  }
  std::cout << "[Prepare] one side, threshold jitter, release and alternating-side debounce passed" << std::endl;

  // Background motion still exists for compensation, but cannot assert crane
  // movement when fallback XY stays fixed (pure Z change).
  {
    CraneBeamDetector motion(cfg);
    for (int frame = 0; frame < 8; ++frame) {
      generateSyntheticScene(points, valid_indices, 0, 1, 1, 4, 0, frame * 0.1, false);
      const auto & out = motion.process(points, valid_indices, frame * 0.1);
      assert(!out.crane_moving);
      assert(!out.hook.left.detected && out.hook.left.position_tracking_state == "FALLBACK_TRACKING");
      if (frame > 0) assert(std::abs(out.dz_bg_filtered) > .05);
      assert(out.state == CraneBeamState::IDLE && !out.length_locked);
    }
  }

  // Preserve support geometry checks as diagnostics, independent of the new
  // Hook-ground completion condition. Tilt/worker/pole algorithms stay active.
  for (bool tilted : {false, true}) {
    CraneBeamDetector support(cfg);
    for (int frame = 0; frame < 145; ++frame) {
      const double displacement = frame < 15 ? 0.0 : frame < 45 ? (frame - 14) * 0.02 :
          frame < 75 ? 0.6 : frame < 105 ? (104 - frame) * 0.02 : 0.0;
      generateSyntheticScene(points, valid_indices, 0, 1, 0.65 + displacement, 4, 0, 0,
          !tilted, true, frame < 75 ? -999.0 : 0.45, 0, 1, 3, 0.3);
      if (tilted) addTiltedBeam(points, valid_indices, 0, 1, 0.65 + displacement,
          4, 0.3, 0.4, 0, 0.10);
      addBox(points, valid_indices, -0.9, 0, (frame < 3 ? 2.0 : frame == 3 ? 1.85 : 1.65) + displacement, 0.2, 0.2, 0.3, 0.025);
      addBox(points, valid_indices, 0.4, 1.8, 1.3, 0.35, 0.35, 1.7, 0.08);
      addCylinder(points, valid_indices, -0.4, 1.8, 0, 3, 0.08);
      const auto & out = support.process(points, valid_indices, (frame + 1) * 0.1);
      if (frame == 144) {
        std::cout << "support final tilted=" << tilted << " state=" << cx128s2_bev_perception::to_string(out.state) << " track=" << out.beam_tracking_state << " z=" << out.z_beam_filtered << " motion=" << out.motion_dz << " length=" << out.beam_length_locked << std::endl;
        assert(out.state == CraneBeamState::COMPLETE_CONFIRM);
        std::cout << "support tilted=" << tilted << " valid=" << out.support_valid << " z=" << out.support_z << " gap=" << out.placement_gap_z << " ratio=" << out.supported_segment_ratio << std::endl;
        assert(out.support_valid);
        if (!tilted) assert(std::abs(out.support_z - 0.45) < 0.02);
        else assert(out.support_z <= 0.45 && out.supported_segment_ratio < 1.0);
        assert(out.supported_segment_ratio >= 0.0 && out.supported_segment_ratio <= 1.0);
        if (!tilted) assert(out.contact_candidate && out.supported_segment_ratio >= cfg.min_supported_segment_ratio);
      }
    }
  }
  std::cout << "[Support] level/tilted support with worker/pole contamination passed" << std::endl;

  // Common sensor motion cannot create relative Hook/Beam lifting. Repeated
  // timestamps cannot advance preparation or transition confirmation either.
  {
    CraneBeamDetector common(cfg);
    for (int frame = 0; frame < 25; ++frame) {
      const double dz = frame < 10 ? 0.0 : (frame - 9) * 0.03;
      generateSyntheticScene(points, valid_indices, 0, 1, 0.65 + dz, 4, 0, dz);
      addBox(points, valid_indices, -0.9, 0, (frame < 3 ? 2.0 : frame == 3 ? 1.85 : 1.65) + dz, 0.2, 0.2, 0.3, 0.025);
      const auto & out = common.process(points, valid_indices, frame * 0.1);
      assert(out.state == CraneBeamState::IDLE);
      for (int duplicate = 0; duplicate < 5; ++duplicate) {
        const auto & repeated = common.process(points, valid_indices, frame * 0.1);
        assert(repeated.state == CraneBeamState::IDLE && !repeated.state_changed);
        assert(repeated.lift_counter == 0);
      }
    }
    const auto & rewind = common.process({}, {}, 0.0);
    assert(!rewind.length_locked && !rewind.hook.prepare_ready && !rewind.active_left);
  }
  std::cout << "[Motion reference] common motion, duplicate timestamps and rewind passed" << std::endl;

  // Net rise and signed direction consistency reject sustained vibration;
  // actual rising body/beam measurements still enter the original LIFTING state.
  {
    auto vibration_cfg = cfg;
    vibration_cfg.hook.roi_z_min = 3.0;
    vibration_cfg.hook.prepare_distance = 4.0;
    vibration_cfg.hook.prepare_confirm_frames = 1;
    vibration_cfg.lift_confirm_frames = 2;
    vibration_cfg.cumulative_motion_enabled = false;
    vibration_cfg.motion_reference_mode = "absolute";
    CraneBeamDetector vibration(vibration_cfg);
    for (int frame = 0; frame < 65; ++frame) {
      const std::array<double, 5> jitter{3.50, 3.56, 3.49, 3.55, 3.50};
      const double height = frame < 3 ? 3.60 : frame < 20 ? 3.50 : frame < 60 ?
          jitter[(frame - 20) % jitter.size()] : 3.50 + (frame - 59) * .11;
      const double beam_delta = frame < 20 ? 0.0 : height - 3.50;
      generateSyntheticScene(points, valid_indices, 0, 1, .65 + beam_delta, 4, 0, 0);
      addBox(points, valid_indices, -.9, 0, height, .2, .2, .35, .025);
      const auto & out = vibration.process(points, valid_indices, 200 + frame * .1);
      if (frame < 60) assert(out.state == CraneBeamState::IDLE);
      if (frame == 64) assert(out.state == CraneBeamState::LIFTING);
    }
    std::cout << "[Net rise] 3.50/3.56/3.49/3.55/3.50 vibration rejected; consistent actual rise accepted." << std::endl;
  }

  // 30. Startup Hook configuration must reach the embedded detector.
  {
    CraneBeamConfig hook_cfg;
    hook_cfg.hook.enabled = false;
    hook_cfg.hook.min_body_points = 20;
    CraneBeamDetector startup_detector(hook_cfg);
    assert(!startup_detector.getHookDetector().getConfig().enabled);
    assert(startup_detector.getHookDetector().getConfig().min_body_points == 20);
    std::cout << "[Test 30] Hook startup config forwarding passed." << std::endl;
  }

  // 31. Hook readiness remains an auxiliary flag, including config changes,
  // evidence loss, hysteresis, disable/reset, and overlapping ROI membership.
  {
    CraneBeamConfig hook_cfg;
    hook_cfg.hook.roi_z_min = 0.5;
    hook_cfg.hook.min_body_points = 20;
    hook_cfg.hook.min_ground_points = 5;
    hook_cfg.hook.prepare_confirm_frames = 2;
    hook_cfg.hook.tracking_confirm_frames = 1;
    hook_cfg.hook.prepare_distance = 1.3;
    CraneBeamDetector hook_detector(hook_cfg);
    std::vector<PointXYZIRT> hook_points;
    std::vector<int> hook_indices;
    for (float x : {-0.9f, 0.9f}) {
      for (int i = 0; i < 32; ++i) {
        PointXYZIRT p;
        p.x = x;
        p.z = 0.9f + 0.01f * i;
        hook_indices.push_back(static_cast<int>(hook_points.size()));
        hook_points.push_back(p);
      }
      for (int i = 0; i < 10; ++i) {
        PointXYZIRT p;
        p.x = x;
        p.z = 0.05f; p.y = .15f + (i % 6) * .1f;
        hook_indices.push_back(static_cast<int>(hook_points.size()));
        hook_points.push_back(p);
      }
    }
    for (auto & p : hook_points) if (p.z > .5f) p.z += .2f;
    assert(!hook_detector.process(hook_points, hook_indices, 0.0).hook.prepare_ready);
    for (auto & p : hook_points) if (p.z > .5f) p.z -= .2f;
    auto first = hook_detector.process(hook_points, hook_indices, 0.1);
    assert(first.hook.left.detected && first.hook.right.detected);
    assert(first.hook.left.ground_valid && first.hook.right.ground_valid);
    assert(first.hook.left.body_points >= 20 && first.hook.right.body_points >= 20);
    assert(!first.hook.prepare_ready && first.state == CraneBeamState::IDLE);
    auto ready = hook_detector.process(hook_points, hook_indices, 0.2);
    assert(ready.hook.prepare_ready && ready.state == CraneBeamState::IDLE);
    assert(std::abs(ready.hook.left.ground_distance -
        (ready.hook.left.body_min[2] - ready.hook.left.ground_z)) < 1e-9);

    // An interruption before N release frames must keep readiness latched.
    hook_detector.process({}, {}, 0.3);
    assert(hook_detector.getHookDetector().getResult().prepare_ready);
    hook_detector.process(hook_points, hook_indices, 0.4);
    assert(hook_detector.getHookDetector().getResult().prepare_ready);

    // Inside the release margin, entry may be false while readiness stays true.
    hook_cfg.hook.prepare_distance = std::max(ready.hook.left.ground_distance, ready.hook.right.ground_distance) - 0.1;
    hook_cfg.hook.prepare_release_margin = 0.2;
    hook_detector.updateConfig(hook_cfg);
    for (int i = 0; i < 3; ++i)
      assert(hook_detector.process(hook_points, hook_indices, 0.5 + i * 0.1).hook.prepare_ready);
    hook_cfg.hook.prepare_distance = 0.5;
    hook_detector.updateConfig(hook_cfg);
    assert(hook_detector.process(hook_points, hook_indices, 0.8).hook.prepare_ready);
    assert(!hook_detector.process(hook_points, hook_indices, 0.9).hook.prepare_ready);

    hook_cfg.hook.enabled = false;
    hook_detector.updateConfig(hook_cfg);
    const auto disabled = hook_detector.process(hook_points, hook_indices, 1.0);
    assert(!disabled.hook.prepare_ready && !disabled.hook.left.detected);
    assert(disabled.hook.left.body_indices.empty() && disabled.hook.left.roi_indices.empty());
    hook_cfg.hook.enabled = true;
    hook_cfg.hook.prepare_distance = 1.5;
    hook_cfg.hook.roi_left_x_max = 1.5;
    hook_cfg.hook.roi_right_x_min = -1.5;
    hook_detector.updateConfig(hook_cfg);
    for (auto & p : hook_points) if (p.z > .5f) p.z += .2f;
    assert(!hook_detector.process(hook_points, hook_indices, 1.1).hook.prepare_ready);
    for (auto & p : hook_points) if (p.z > .5f) p.z -= .2f;
    auto overlap = hook_detector.process(hook_points, hook_indices, 1.2);
    assert(overlap.hook.left.roi_indices == overlap.hook.right.roi_indices);
    assert(overlap.hook.left.body_indices == overlap.hook.right.body_indices);
    assert(!overlap.hook.prepare_ready);
    const auto overlap_ready = hook_detector.process(hook_points, hook_indices, 1.3);
    if (!overlap_ready.hook.prepare_ready) std::cerr << "overlap ready: detected=" << overlap_ready.hook.left.detected
        << " initialized=" << overlap_ready.hook.left.initialized << " z=" << overlap_ready.hook.left.z
        << " pred=" << overlap_ready.hook.left.predicted_z << " reason=" << overlap_ready.hook.left.selection_reason
        << " details=" << overlap_ready.hook.left.candidate_details << std::endl;
    assert(overlap_ready.hook.prepare_ready);
    hook_detector.reset();
    assert(!hook_detector.getHookDetector().getResult().prepare_ready);
    assert(hook_detector.getHookDetector().getResult().prepare_counter == 0);
    std::cout << "[Test 31] Hook readiness/config/reset/ROI regression passed." << std::endl;
  }

  // 32. A Beam without measured Hook preparation cannot start a work cycle.
  {
    auto disabled_cfg = cfg; disabled_cfg.hook.enabled = false;
    CraneBeamDetector disabled(disabled_cfg);
    for (int frame = 0; frame < 30; ++frame) {
      generateSyntheticScene(points, valid_indices, 0.0, 1.0, 0.65 + frame * 0.05, 4.0, 0.0, 0.0);
      const auto & out = disabled.process(points, valid_indices, (frame + 1) * 0.1);
      assert(out.state == CraneBeamState::IDLE && !out.length_locked);
    }
    CraneBeamDetector fallback_only(cfg);
    for (int frame = 0; frame < 30; ++frame) {
      generateSyntheticScene(points, valid_indices, 0.0, 1.0, 0.65 + frame * 0.05, 4.0, 0.0, 0.0);
      PointXYZIRT highest; highest.x = -.85f; highest.y = .5f; highest.z = 5.8f;
      valid_indices.push_back(static_cast<int>(points.size())); points.push_back(highest);
      const auto & out = fallback_only.process(points, valid_indices, (frame + 1) * .1);
      assert(out.hook.left.position_tracking_state == "FALLBACK_TRACKING");
      assert(!out.hook.left.detected && !out.hook.prepare_ready);
      assert(out.state == CraneBeamState::IDLE && !out.length_locked && out.working_status == 0);
    }
    std::cout << "[Test 32] Beam-only work transition blocked." << std::endl;
  }

  // 33. Independent Hook CV tracks: acquisition, motion, sway, missing,
  // wrong-body rejection, timeout, reset, and measured-only readiness.
  {
    cx128s2_bev_perception::CraneHookConfig cfg;
    cfg.roi_z_min = 0.5;
    cfg.prepare_distance = 3.0;
    cfg.prepare_confirm_frames = 2;
    cx128s2_bev_perception::CraneHookDetector detector(cfg);
    std::vector<PointXYZIRT> base;
    for (float center_x : {-0.85f, 0.85f}) {
      for (int i = 0; i < 8; ++i) for (int j = 0; j < 8; ++j) {
        PointXYZIRT p;
        p.x = center_x + (i - 3.5f) * 0.02f;
        p.y = (j - 3.5f) * 0.02f;
        for (int g = 0; g < 5; ++g) base.push_back(p);  // ground support during dropout
        for (int k = 0; k < 7; ++k) {
          p.z = 2.65f + k * 0.05f;
          base.push_back(p);
        }
      }
    }
    double stamp = 0.0;
    auto frame = [&](double dz, double sx, double sy, bool hide_left, bool hide_right) {
      std::vector<PointXYZIRT> points;
      std::vector<int> indices;
      for (auto p : base) {
        if (p.z > cfg.roi_z_min) {
          if (p.x < 0 ? hide_left : hide_right) continue;
          p.x += static_cast<float>(sx);
          p.y += static_cast<float>(sy);
          p.z += static_cast<float>(dz);
        }
        indices.push_back(static_cast<int>(points.size()));
        points.push_back(p);
      }
      stamp += 0.1;
      return detector.process(points, indices, stamp);
    };
    for (int i = 0; i < 3; ++i) {
      auto r = frame(0, 0, 0, false, false);
      assert(r.left.detected && r.right.detected);
      assert(r.left.initialized == (i == 2));
      assert(r.right.initialized == (i == 2));
    }
    for (int i = 0; i < 15; ++i) frame(0, 0, 0, false, false);
    const double initial_z = detector.getResult().left.z;
    assert(std::abs(detector.getResult().left.vz) < 1e-6);
    for (int i = 1; i <= 20; ++i) {
      auto r = frame(i * 0.05, 0.12 * std::sin(i * 0.4),
          0.12 * std::cos(i * 0.4), false, false);
      assert(r.left.detected && r.right.detected && r.left.initialized && r.right.initialized);
      if (i >= 10) {
        assert(std::abs(r.left.filtered_z - r.left.z) < 0.04);
        assert(std::abs(r.left.vz - 0.5) < 0.08);
        assert(std::abs(r.right.vz - 0.5) < 0.08);
      }
    }
    for (int i = 1; i <= 20; ++i) {
      auto r = frame(1.0 - i * 0.05, 0, 0, false, false);
      assert(r.left.detected && r.right.detected);
      if (i >= 10) {
        assert(std::abs(r.left.filtered_z - r.left.z) < 0.04);
        assert(std::abs(r.left.vz + 0.5) < 0.08);
      }
    }
    for (int i = 0; i < 10; ++i) frame(0, 0, 0, false, false);
    assert(detector.getResult().prepare_ready);
    cfg.tracking_enabled = false;
    detector.updateConfig(cfg);
    assert(detector.getResult().prepare_ready && !detector.getResult().left.initialized);
    auto measured_only = frame(0, 0, 0, false, false);
    assert(measured_only.left.detected && !measured_only.left.initialized && measured_only.prepare_ready);
    cfg.tracking_enabled = true;
    detector.updateConfig(cfg);
    assert(detector.getResult().prepare_ready);
    for (int i = 0; i < 3; ++i) frame(.1, 0, 0, false, false);
    frame(.05, 0, 0, false, false);
    frame(0, 0, 0, false, false);
    for (int i = 1; i <= 3; ++i) {
      auto r = frame(0, 0, 0, true, false);
      assert(!r.left.detected && r.left.initialized);
      assert(r.left.tracking_state == "PREDICTED" && r.left.lost_frames == i);
      assert(r.left.body_indices.empty() && r.left.body_points == 0);
      assert(r.left.predicted_ground_distance_valid && r.left.ground_distance == 0.0);
      assert(r.right.detected && r.right.lost_frames == 0);
      if (i >= 2) assert(r.prepare_ready && r.prepare_sides[1]); // measured Right preserves preparation
    }
    auto restored = frame(0, 0, 0, false, false);
    assert(restored.left.initialized && restored.left.detected && restored.left.lost_frames == 0);
    assert(std::abs(restored.left.filtered_z - initial_z) < 0.04);
    // A larger unrelated density body in the same ROI must not beat a visible
    // Hook, nor replace it when the actual Hook disappears.
    for (bool visible : {true, false}) {
      std::vector<PointXYZIRT> points;
      std::vector<int> indices;
      for (const auto & original : base) {
        if (original.z <= cfg.roi_z_min || visible) {
          indices.push_back(static_cast<int>(points.size()));
          points.push_back(original);
        }
        if (original.z > cfg.roi_z_min) for (int copy = 0; copy < 3; ++copy) {
          auto distractor = original;
          distractor.y += 0.6f;
          indices.push_back(static_cast<int>(points.size()));
          points.push_back(distractor);
        }
      }
      stamp += 0.1;
      const auto & r = detector.process(points, indices, stamp);
      assert(r.left.initialized && r.left.detected == visible);
      assert(std::abs(r.left.filtered_y) < 0.15);
      assert(std::abs(r.left.filtered_z - initial_z) < 0.05);
    }
    frame(0, 0, 0, false, false);
    // Same density, still in the fixed ROI, but 0.6m away in Y.
    auto wrong = frame(0, 0, 0.6, false, false);
    assert(!wrong.left.detected && wrong.left.initialized);
    assert(std::abs(wrong.left.filtered_y) < 0.15);
    frame(0, 0, 0, false, false);
    wrong = frame(1.5, 0, 0, false, false);
    assert(!wrong.left.detected && wrong.left.initialized);
    assert(std::abs(wrong.left.filtered_z - initial_z) < 0.05);
    frame(0, 0, 0, false, false);
    for (int i = 1; i <= 6; ++i) {
      auto r = frame(0, 0, 0, true, false);
      assert(!r.left.detected && r.left.lost_frames == i);
      assert(r.left.initialized == (i <= 5));
      assert(r.left.tracking_state == (i <= 5 ? "PREDICTED" : "LOST"));
      assert(r.right.initialized && r.right.detected);
    }
    for (int i = 1; i <= 3; ++i) {
      auto r = frame(0.9, 0, 0, false, true);  // full-ROI acquisition after LOST
      assert(r.left.detected && r.left.initialized);
    }
    stamp += 2.0;
    auto gap = frame(0.9, 0, 0, false, false);
    assert(gap.left.detected && gap.left.initialized && gap.left.tracking_state == "MEASURED");
    auto rewind = detector.process(base, {}, 0.0);
    assert(!rewind.left.initialized && rewind.left.tracking_state == "UNINITIALIZED");
    detector.reset();
    assert(!detector.getResult().right.initialized);
    std::cout << "[Test 33] Hook CV tracking/motion/sway/occlusion/gates/timeouts passed." << std::endl;
  }

  // 34. Optional recorded PCD regression + movement of the measured body points.
  // Sensor->world calibration matches lidar_bringup/config/lidar_tf_crane.yaml.
  {
    const auto directory = std::filesystem::absolute(__FILE__).parent_path().parent_path().parent_path() /
        "lidar_ground_tools/measurements/crane_2026_09_22_032118/pcd";
    if (std::filesystem::exists(directory / "frame_0001.pcd")) {
      cx128s2_bev_perception::CraneHookConfig cfg;
      cx128s2_bev_perception::CraneHookDetector detector(cfg);
      cfg.tracking_enabled = false;
      cx128s2_bev_perception::CraneHookDetector baseline(cfg);
      cfg.tracking_enabled = true;
      std::vector<PointXYZIRT> seed;
      cx128s2_bev_perception::CraneHookResult seed_detection;
      double max_static_vz = 0.0, max_static_error = 0.0;
      for (int frame = 1; frame <= 30; ++frame) {
        std::ostringstream name;
        name << "frame_" << std::setw(4) << std::setfill('0') << frame << ".pcd";
        std::ifstream input(directory / name.str(), std::ios::binary);
        assert(input.good());
        std::string line;
        size_t count = 0;
        while (std::getline(input, line)) {
          if (line.rfind("POINTS ", 0) == 0) count = std::stoul(line.substr(7));
          if (line == "DATA binary") break;
        }
        assert(count > 0 && line == "DATA binary");
        std::vector<PointXYZIRT> points(count);
        std::vector<int> indices;
        const double roll = -1.5671972765207525, pitch = -0.027489352235824036;
        const double cr = std::cos(roll), sr = std::sin(roll);
        const double cp = std::cos(pitch), sp = std::sin(pitch);
        for (size_t i = 0; i < count; ++i) {
          float xyz[3];
          input.read(reinterpret_cast<char *>(xyz), sizeof(xyz));
          assert(input.good());
          const double y = cr * xyz[1] - sr * xyz[2];
          const double z = sr * xyz[1] + cr * xyz[2];
          points[i].x = static_cast<float>(cp * xyz[0] + sp * z);
          points[i].y = static_cast<float>(y);
          points[i].z = static_cast<float>(-sp * xyz[0] + cp * z + 6.638276670931532);
          indices.push_back(static_cast<int>(i));
        }
        const auto & old = baseline.process(points, indices, frame * 0.1);
        const auto & tracked = detector.process(points, indices, frame * 0.1);
        assert(old.left.detected && old.right.detected);
        if (!tracked.left.detected || !tracked.right.detected) std::cerr << "PCD frame=" << frame
            << " left=" << tracked.left.selection_reason << " " << tracked.left.candidate_details
            << " right=" << tracked.right.selection_reason << " " << tracked.right.candidate_details << std::endl;
        assert(tracked.left.detected && tracked.right.detected);
        assert(std::abs(tracked.left.ground_z - old.left.ground_z) < cfg.ground_bin_size);
        assert(std::abs(tracked.right.ground_z - old.right.ground_z) < cfg.ground_bin_size);
        assert(tracked.prepare_ready == old.prepare_ready);
        assert(std::abs(tracked.left.ground_distance - (tracked.left.body_min[2] - tracked.left.ground_z)) < 1e-9);
        if (frame >= 10) for (const auto * hook : {&tracked.left, &tracked.right}) {
          assert(hook->initialized);
          max_static_vz = std::max(max_static_vz, std::abs(hook->vz));
          max_static_error = std::max(max_static_error, std::abs(hook->filtered_z - hook->z));
        }
        if (frame == 1) { seed = points; seed_detection = old; }
      }
      assert(max_static_vz < 0.1 && max_static_error < 0.04);
      std::cout << "[Test 34] PCD 30/30 both hooks; max static |vz|=" << max_static_vz
                << "m/s, filtered error=" << max_static_error << "m." << std::endl;
      std::cout << "  PCD static: left z=" << detector.getResult().left.filtered_z
                << " vz=" << detector.getResult().left.vz << "; right z="
                << detector.getResult().right.filtered_z << " vz=" << detector.getResult().right.vz
                << std::endl;
      // Keep real PCD body samples and the fixed ground; move only body samples.
      std::vector<PointXYZIRT> body_scene;
      for (int idx : seed_detection.left.body_indices) body_scene.push_back(seed[idx]);
      for (int idx : seed_detection.right.body_indices) body_scene.push_back(seed[idx]);
      for (const auto & p : seed) if (p.z >= cfg.ground_search_z_min &&
          p.z <= cfg.ground_search_z_max) body_scene.push_back(p);
      for (int direction : {1, -1}) {
        detector.reset();
        double stamp = 0.0;
        // 12 moving frames keep the entire real body inside fixed Z <= 6.2m.
        for (int frame = 0; frame < 18; ++frame) {
          std::vector<PointXYZIRT> points;
          std::vector<int> indices;
          const double dz = direction * std::max(0, frame - 5) * 0.05;
          for (auto p : body_scene) {
            if (p.z >= cfg.roi_z_min) {
              p.z += static_cast<float>(dz);
              p.x += static_cast<float>(0.12 * std::sin(frame * 0.35));
              p.y += static_cast<float>(0.12 * std::cos(frame * 0.35));
              if (frame >= 11 && frame <= 13) continue;
            }
            indices.push_back(static_cast<int>(points.size()));
            points.push_back(p);
          }
          stamp += 0.1;
          const auto & r = detector.process(points, indices, stamp);
          for (const auto * hook : {&r.left, &r.right}) {
            if (frame >= 11 && frame <= 13) {
              assert(!hook->detected && hook->initialized && hook->tracking_state == "PREDICTED");
            } else {
              if (!hook->detected) std::cerr << "PCD direction=" << direction << " frame="
                  << frame << " state=" << hook->tracking_state << " predicted_z="
                  << hook->predicted_z << std::endl;
              assert(hook->detected);
              if (frame >= 15) {
                assert(std::abs(hook->vz - direction * 0.5) < 0.1);
                assert(std::abs(hook->filtered_z - hook->z) < 0.05);
              }
            }
          }
          if (frame == 17) std::cout << "  PCD motion " << direction << ": left z="
              << r.left.filtered_z << " vz=" << r.left.vz << "; right z="
              << r.right.filtered_z << " vz=" << r.right.vz << std::endl;
        }
      }
    } else std::cout << "[Test 34] Recorded PCD unavailable; synthetic Hook tests still run." << std::endl;
  }

  // 35. Beam comparison must preserve tracking/readiness and reject invalid references.
  {
    CraneBeamConfig cfg;
    cfg.hook.roi_z_min = 1.2; // Scaled comparison fixture includes a Hook below its Beam.
    cfg.beam_roi_x_min = cfg.beam_roi_y_min = -0.2;
    cfg.beam_roi_x_max = cfg.beam_roi_y_max = 0.2;
    cfg.beam_roi_z_min = 0.3;
    cfg.min_beam_cluster_points = 5;
    cfg.hook.min_body_points = 300;
    cfg.hook.min_ground_points = 5;
    CraneBeamDetector detector(cfg);
    cx128s2_bev_perception::CraneHookDetector standalone(cfg.hook);
    for (int frame = 0; frame < 7; ++frame) {
      std::vector<PointXYZIRT> points;
      std::vector<int> indices;
      generateSyntheticScene(points, indices, 0.0, 0.0, 2.0, 4.0, 0.0, 0.0, frame != 4);
      for (int i = 0; i < 600; ++i) {
        PointXYZIRT p;
        for (float x : {-0.9f, 0.9f}) {
          p.x = x; p.y = .15f + (i % 6) * .1f; p.z = x < 0 ? 0.1f : 0.2f;
          indices.push_back(static_cast<int>(points.size())); points.push_back(p);
          if (frame == 5 && x < 0) continue;
          p.y = 0.0f;
          // Keep this comparison-only fixture outside the new 0.30m
          // acquisition exclusion band around the Beam at 2m.
          p.z = (x < 0 ? 3.0f + frame * 0.02f : 1.3f) + i * 0.0006f;
          indices.push_back(static_cast<int>(points.size())); points.push_back(p);
        }
      }
      const auto & r = detector.process(points, indices, 0.1 * (frame + 1));
      const auto & h = standalone.process(points, indices, 0.1 * (frame + 1));
      assert(r.hook.prepare_ready == h.prepare_ready);
      for (const auto * hook : {&r.hook.left, &r.hook.right}) {
        const auto & old = hook == &r.hook.left ? h.left : h.right;
        assert(hook->z == old.z && hook->filtered_z == old.filtered_z);
        assert(hook->vz == old.vz && hook->tracking_state == old.tracking_state);
        assert(hook->ground_z == old.ground_z && hook->ground_distance == old.ground_distance);
        assert(hook->beam_distance_valid == (r.beam_z_valid && hook->detected));
        if (hook->beam_distance_valid) {
          assert(hook->beam_reference_z == r.z_beam_filtered);
          assert(hook->beam_dz == hook->z - r.z_beam_filtered);
        } else assert(hook->beam_reference_z == 0.0 && hook->beam_dz == 0.0);
      }
      if (frame == 4) assert(!r.beam_z_valid && !r.hook.left.beam_distance_valid);
      if (frame == 5) assert(r.hook.left.tracking_state == "PREDICTED" &&
          !r.hook.left.beam_distance_valid && r.hook.right.beam_distance_valid);
      if (frame == 0) assert(r.hook.left.beam_dz > 0.0 && r.hook.right.beam_dz < 0.0);
    }
    std::cout << "[Test 35] Independent Hook/Beam comparison and invalid-reference regression passed." << std::endl;
  }
  // 36. Denser in-gate Z peak must not replace the accepted body. Identity
  // survives missing frames, and the two sides can move in opposite directions.
  {
    cx128s2_bev_perception::CraneHookConfig cfg;
    cfg.roi_z_min = 1.5; // Preserve the existing low-height identity/motion fixture.
    cx128s2_bev_perception::CraneHookDetector detector(cfg);
    std::vector<PointXYZIRT> base;
    for (float x : {-0.85f, 0.85f}) {
      for (int i = 0; i < 8; ++i) for (int j = 0; j < 8; ++j) {
        PointXYZIRT p; p.x = x + (i - 3.5f) * 0.02f; p.y = (j - 3.5f) * 0.02f;
        for (int k = 0; k < 7; ++k) { p.z = 2.65f + k * 0.05f; base.push_back(p); }
      }
    }
    for (float x : {-0.85f, 0.85f}) for (int i = 0; i < 150; ++i) {
      PointXYZIRT ground; ground.x = x; ground.y = .15f + (i % 6) * .1f; ground.z = 0.0f; base.push_back(ground);
    }
    double stamp = 0.0;
    double initial_z = 0.0;
    for (int frame = 0; frame < 45; ++frame) {
      std::vector<PointXYZIRT> points;
      std::vector<int> indices;
      const bool wrong_peak = frame >= 15 && frame < 25;
      const bool missing = frame >= 19 && frame <= 21;
      const double movement = frame >= 30 ? (frame - 29) * 0.05 : 0.0;
      for (auto p : base) {
        const bool left = p.x < 0;
        if (p.z <= cfg.ground_search_z_max || !left || !missing) {
          if (p.z > cfg.ground_search_z_max) p.z += static_cast<float>(left ? movement : -movement);
          indices.push_back(static_cast<int>(points.size())); points.push_back(p);
        }
        if (left && p.z > cfg.ground_search_z_max && wrong_peak) for (int copy = 0; copy < 3; ++copy) {
          auto wrong = p; wrong.z = 2.38f + (p.z - 2.65f) * 0.4f;
          indices.push_back(static_cast<int>(points.size())); points.push_back(wrong);
        }
      }
      if (wrong_peak) {
        // Sparse cable return provides Z search extent without body support.
        PointXYZIRT cable; cable.x = -0.85f; cable.z = 3.0f;
        indices.push_back(static_cast<int>(points.size())); points.push_back(cable);
      }
      stamp += 0.1;
      const auto & r = detector.process(points, indices, stamp);
      if (frame == 10) initial_z = r.left.z;
      if (frame >= 15 && frame < 25) {
        assert(r.left.initialized && r.right.initialized && r.right.detected);
        if (missing) {
          assert(!r.left.detected && r.left.tracking_state == "PREDICTED");
          assert(r.left.lost_frames == frame - 18);
          assert(r.left.selection_reason == "PREDICTION_OR_SHAPE_INCONSISTENT");
          assert(r.left.candidate_count > 0); // density-valid wrong peak is in the point gate
          assert(r.left.last_measurement_z == initial_z);
        } else {
          assert(r.left.detected && std::abs(r.left.z - initial_z) < 0.06);
          assert(r.left.prediction_error_z < 0.06);
          assert(!r.left.candidate_switched);
          if (frame == 15) {
            assert(r.left.candidate_count >= 2);
            size_t largest_peak = 0;
            for (size_t pos = r.left.candidate_details.find(";pts="); pos != std::string::npos;
                 pos = r.left.candidate_details.find(";pts=", pos + 1)) {
              largest_peak = std::max(largest_peak,
                  static_cast<size_t>(std::stoull(r.left.candidate_details.substr(pos + 5))));
            }
            assert(largest_peak > r.left.body_points); // continuity beats denser in-gate peak
          }
        }
      }
      if (frame >= 39) {
        assert(r.left.detected && r.right.detected);
        assert(std::abs(r.left.vz - 0.5) < 0.08);
        assert(std::abs(r.right.vz + 0.5) < 0.08);
      }
    }
    // Faster +/-0.15m per frame must track; the other side remains stationary.
    for (int direction : {1, -1}) {
      detector.reset();
      for (int frame = 0; frame < 13; ++frame) {
        std::vector<PointXYZIRT> points;
        std::vector<int> indices;
        for (auto p : base) {
          if (p.z > cfg.ground_search_z_max) p.z += 1.0f;
          if (p.x < 0 && p.z > cfg.ground_search_z_max) {
            p.z += static_cast<float>(direction * frame * 0.15);
            p.x += static_cast<float>(0.12 * std::sin(frame * 0.35));
            p.y += static_cast<float>(0.12 * std::cos(frame * 0.35));
          }
          indices.push_back(static_cast<int>(points.size())); points.push_back(p);
        }
        const auto & r = detector.process(points, indices, frame * 0.1);
        if (!r.left.detected || !r.right.detected) std::cerr << "fast direction=" << direction << " frame=" << frame
            << " L=" << r.left.selection_reason << " " << r.left.candidate_details
            << " R=" << r.right.selection_reason << std::endl;
        assert(r.left.detected && r.right.detected);
        if (frame >= 9) {
          assert(std::abs(r.left.vz - direction * 1.5) < 0.10);
          assert(std::abs(r.right.vz) < 1e-6);
        }
      }
    }
    std::cout << "[Test 36] In-gate denser wrong peak, prediction fallback, retained identity, independent motion and +/-1.5m/s with sway passed." << std::endl;
  }

  // 37. Reference geometry stays fixed while a confirmed body becomes partial;
  // nearby, dense thin ropes cannot replace it or manufacture READY.
  {
    cx128s2_bev_perception::CraneHookConfig hc;
    cx128s2_bev_perception::CraneHookDetector tracked(hc);
    double stamp = 0.0;
    const auto frame = [&](double height, bool partial, bool body, bool rope, bool ground = true) {
      std::vector<PointXYZIRT> cloud;
      std::vector<int> indices;
      if (ground) addBox(cloud, indices, -.9, 0, .2, .6, .6, 0, .04);
      if (body) addBox(cloud, indices, -.9, 0, height + .2 + (partial ? .05 : .175),
          partial ? .2 : .4, partial ? .2 : .4, partial ? .1 : .35, .04);
      if (rope) for (int i = 0; i < 4000; ++i) {
        PointXYZIRT p;
        p.x = -.9f + static_cast<float>(i % 3) * .002f;
        p.y = .30f + static_cast<float>(i % 5) * .001f;
        p.z = static_cast<float>(height + .2 - .7 + (i % 2000) * .001);
        indices.push_back(static_cast<int>(cloud.size())); cloud.push_back(p);
      }
      stamp += .1;
      return tracked.process(cloud, indices, stamp);
    };
    for (int i = 0; i < 6; ++i) frame(3.8, false, true, false);
    const auto reference = tracked.getResult().left;
    assert(reference.initialized && reference.reference_valid && reference.reference_width > .3);
    assert(!tracked.getResult().prepare_ready);
    // A closer dense rope alone: no accepted measurement, no READY, same memory.
    for (int i = 1; i <= 2; ++i) {
      auto r = frame(3.8, false, false, true);
      assert(!r.left.detected && r.left.initialized && r.left.tracking_state == "PREDICTED");
      assert(r.left.lost_frames == i && !r.prepare_ready);
      assert(r.left.reference_width == reference.reference_width);
      assert(r.left.candidate_details.find("shape_ok=0") != std::string::npos);
    }
    auto restored = frame(3.8, false, true, true);
    assert(restored.left.initialized && restored.left.detected && restored.left.lost_frames == 0);
    assert(std::abs(restored.left.y - reference.y) < 2 * hc.tracking_measurement_std);
    // Smaller actual fragments and an in-gate wire coexist during descent.
    for (int i = 0; i <= 20; ++i) {
      const double height = 3.7 - i * .05;
      auto r = frame(height, true, true, true);
      assert(r.left.initialized && r.left.detected && r.left.partial_observation);
      assert(r.left.body_points < hc.min_body_points && r.left.body_points >= hc.min_body_points / 4);
      assert(r.left.reference_width == reference.reference_width);
      assert(r.left.reference_length == reference.reference_length);
      assert(r.left.reference_min == reference.reference_min && r.left.reference_max == reference.reference_max);
      if (!(std::abs(r.left.y) < 2 * hc.tracking_measurement_std && std::abs(r.left.ground_distance - height) < .05))
        std::cerr << "partial i=" << i << " y=" << r.left.y << " ref.y=" << reference.y
                  << " height=" << height << " measured=" << r.left.ground_distance << " reason=" << r.left.selection_reason
                  << " " << r.left.candidate_details << std::endl;
      assert(std::abs(r.left.y) < 2 * hc.tracking_measurement_std && std::abs(r.left.ground_distance - height) < .05);
      assert(r.prepare_ready);
    }
    assert(tracked.getResult().left.ground_distance < 3.0);
    // Ground loss preserves identity/measurements but supplies no READY height.
    auto missing_ground = frame(2.7, true, true, false, false);
    assert(missing_ground.left.detected && missing_ground.left.initialized && !missing_ground.prepare_ready);
    assert(!missing_ground.left.ground_valid);
    // Body tracker expires, but remembered shape can recover the same low Hook.
    for (int i = 1; i <= 6; ++i) {
      auto lost = frame(2.7, false, false, true);
      assert(!lost.left.detected);
      assert(lost.left.initialized == (i <= 5));
      assert(lost.left.reference_valid == (i <= 5));
    }
    auto low_acquisition = frame(2.7, true, true, false);
    assert(low_acquisition.left.detected && low_acquisition.left.initialized && !low_acquisition.prepare_ready);
    tracked.reset();
    assert(!tracked.getResult().left.reference_valid);
    // Explicit offline learning needs many full, stable frames and persists a
    // multi-frame reference; runtime never writes current fragments back to disk.
    const std::string reference_path = "/tmp/crane-hook-reference-unit-test.yaml";
    std::string report;
    tracked.beginReferenceLearning();
    for (int i = 0; i < 5; ++i) frame(3.8, false, true, false);
    assert(!tracked.saveReference(reference_path, "world", report));
    for (int i = 0; i < 70; ++i) frame(3.8, false, true, false);
    assert(tracked.saveReference(reference_path, "world", report));
    assert(report.find("SAVED") != std::string::npos && report.find("shape_spread=") != std::string::npos);
    // Default development installs use config symlinks. Atomic SAVE must retain
    // that link and update the real file, which is preserved across rebuilds.
    const auto reference_link = reference_path + ".link";
    std::filesystem::create_symlink(reference_path, reference_link);
    tracked.beginReferenceLearning();
    for (int i = 0; i < 35; ++i) frame(3.8, false, true, false);
    assert(tracked.saveReference(reference_link, "world", report));
    assert(std::filesystem::is_symlink(reference_link));
    std::filesystem::remove(reference_link);
    std::ifstream before_input(reference_path);
    const std::string before_data((std::istreambuf_iterator<char>(before_input)), {});
    assert(!tracked.loadReference(reference_path, "map", report));
    assert(tracked.loadReference(reference_path, "world", report));
    assert(tracked.referenceSummary().find("LEFT loaded=1") != std::string::npos);
    // Saved shape is used for initial acquisition, before any per-run track exists.
    auto rope_acquisition = frame(3.8, false, false, true);
    assert(!rope_acquisition.left.detected && !rope_acquisition.left.initialized);
    for (int i = 0; i < 3; ++i) frame(3.8, false, true, false);
    assert(tracked.getResult().left.initialized);
    frame(3.7, true, true, true);
    frame(3.65, true, true, true);
    assert(tracked.getResult().left.detected && tracked.getResult().left.partial_observation);
    std::ifstream after_input(reference_path);
    const std::string after_data((std::istreambuf_iterator<char>(after_input)), {});
    assert(before_data == after_data);
    // An incomplete recollection cannot drop an already trusted side from disk.
    tracked.beginReferenceLearning();
    std::vector<PointXYZIRT> right_only;
    std::vector<int> right_indices;
    addBox(right_only, right_indices, .9, 0, .2, .6, .6, 0, .04);
    addBox(right_only, right_indices, .9, 0, 4.175, .4, .4, .35, .04);
    for (int i = 0; i < 70; ++i) tracked.process(right_only, right_indices, 50 + i * .1);
    assert(!tracked.saveReference(reference_path, "world", report));
    assert(report.find("retaining trusted LEFT") != std::string::npos);
    std::ifstream retained_input(reference_path);
    assert(std::string((std::istreambuf_iterator<char>(retained_input)), {}) == before_data);
    tracked.beginReferenceLearning();
    frame(3.8, false, true, false);
    tracked.cancelReferenceLearning();
    assert(tracked.referenceSummary().find("learning=0 LEFT loaded=1 frames=0") != std::string::npos);
    auto changed_profile = hc;
    changed_profile.roi_y_max += .1;
    tracked.updateConfig(changed_profile);
    assert(tracked.referenceSummary().find("LEFT loaded=0") != std::string::npos);
    assert(!tracked.loadReference(reference_path, "world", report));
    tracked.updateConfig(hc);
    assert(tracked.loadReference(reference_path, "world", report));
    std::filesystem::remove(reference_path);

    // Dense chain/rope below the Hook is excluded from offset ground support.
    tracked.reset();
    std::vector<PointXYZIRT> ground_scene;
    std::vector<int> ground_indices;
    addBox(ground_scene, ground_indices, -.9, 0, 4.175, .4, .4, .35, .04);
    addBox(ground_scene, ground_indices, -.9, 0, .2, .6, .6, 0, .04);
    for (int i = 0; i < 2000; ++i) {
      PointXYZIRT chain; chain.x = -.9f; chain.z = .5f;
      ground_indices.push_back(static_cast<int>(ground_scene.size())); ground_scene.push_back(chain);
    }
    for (int i = 0; i < 2000; ++i) {
      PointXYZIRT offset_chain; offset_chain.x = -.6f; offset_chain.y = .3f; offset_chain.z = -.1f;
      ground_indices.push_back(static_cast<int>(ground_scene.size())); ground_scene.push_back(offset_chain);
    }
    PointXYZIRT outlier; outlier.x = -.6f; outlier.y = .3f; outlier.z = -.4f;
    ground_indices.push_back(static_cast<int>(ground_scene.size())); ground_scene.push_back(outlier);
    for (int i = 0; i < 4; ++i) {
      const auto & r = tracked.process(ground_scene, ground_indices, 100 + i * .1);
      assert(r.left.detected && r.left.ground_valid && std::abs(r.left.ground_z - .2) < 1e-6);
    }
    std::cout << "[Test 37] Immutable geometry, partial support, dense wire rejection, 3m-below tracking, READY identity, hold/reacquisition and ground loss passed." << std::endl;
  }

  // 38. The real scanner sees a very thin XY surface. A tilted wire can have
  // the same XY/Z bounds, support and footprint, but no independent U/Z area.
  {
    cx128s2_bev_perception::CraneHookDetector detector;
    const auto scene = [](bool wire) {
      std::vector<PointXYZIRT> cloud;
      std::vector<int> indices;
      addBox(cloud, indices, -.9, 0, .2, .6, .6, 0, .04);
      for (int x = 0; x <= 40; ++x) for (int y = 0; y <= 2; ++y) for (int z = 0; z <= 30; ++z) {
        PointXYZIRT p;
        p.x = static_cast<float>(-1.1 + x * .01);
        p.y = static_cast<float>(-.01 + y * .01);
        p.z = static_cast<float>(4.0 + (wire ? x * .0075 : z * .01));
        indices.push_back(static_cast<int>(cloud.size())); cloud.push_back(p);
      }
      return std::make_pair(cloud, indices);
    };
    const auto body = scene(false), wire = scene(true);
    for (int i = 0; i < 6; ++i) detector.process(body.first, body.second, 1 + .1 * i);
    const auto reference = detector.getResult().left;
    assert(reference.initialized && reference.reference_width < .04);
    assert(reference.reference_surface_ratio > .1);
    const auto & rejected = detector.process(wire.first, wire.second, 1.6);
    assert(!rejected.left.detected && rejected.left.initialized && rejected.left.tracking_state == "PREDICTED");
    assert(rejected.left.candidate_details.find("shape_ok=0") != std::string::npos);
    assert(rejected.left.reference_surface_ratio == reference.reference_surface_ratio);
    const auto & restored = detector.process(body.first, body.second, 1.7);
    assert(restored.left.detected && restored.left.initialized && restored.left.lost_frames == 0);
    std::cout << "[Test 38] Thin real-style surface retained; same-extent dense tilted wire rejected by U/Z surface geometry." << std::endl;
    // Large U/Z variance alone must not turn thin surfaces/parallel ropes into
    // persistent body memory. Transverse size must exceed sensor uncertainty.
    detector.beginReferenceLearning();
    for (int i = 0; i < 70; ++i) detector.process(body.first, body.second, 1.8 + i * .1);
    std::string report;
    assert(!detector.saveReference("/tmp/crane-hook-unresolved-reference.yaml", "world", report));
    assert(detector.referenceSummary().find("LEFT loaded=0 frames=0") != std::string::npos);
    assert(!std::filesystem::exists("/tmp/crane-hook-unresolved-reference.yaml"));
    detector.cancelReferenceLearning();
    detector.reset();
    std::vector<PointXYZIRT> floor;
    std::vector<int> floor_indices;
    addBox(floor, floor_indices, -.9, 0, .2, .6, .6, 0, .04);
    for (int i = 0; i < 4; ++i) detector.process(floor, floor_indices, 2.0 + i * .1);
    floor.clear(); floor_indices.clear();
    addBox(floor, floor_indices, -.9, 0, 0, .6, .6, 0, .04);
    addBox(floor, floor_indices, -.9, 0, 3.325, .4, .4, .35, .04);
    const auto & unstable_ground = detector.process(floor, floor_indices, 2.4);
    assert(unstable_ground.left.ground_valid && std::abs(unstable_ground.left.ground_z - .2) < 1e-6);
    assert(!unstable_ground.left.detected && !unstable_ground.left.initialized);
    assert(unstable_ground.left.selection_reason == "ACQUISITION_BELOW_STABLE_GROUND_HEIGHT");
  }

  // 39. Beam-level rejection is per candidate and acquisition-only. A current
  // valid Beam median must not stop the search or invalidate confirmed identity.
  {
    cx128s2_bev_perception::CraneHookDetector detector;
    const auto scene = [](double low_z, bool upper, bool rope = false) {
      std::vector<PointXYZIRT> points;
      std::vector<int> indices;
      addBox(points, indices, -.9, 0, 0, .8, .8, 0, .04);
      if (low_z > 0) addBox(points, indices, -.9, 0, low_z,
          .32, rope ? 0.0 : .16, .24, .01);
      if (upper) addBox(points, indices, -.9, 0, 4.6, .32, .16, .24, .02);
      return std::make_pair(points, indices);
    };
    const auto upper_only = scene(0, true);
    detector.beginReferenceLearning();
    for (int i = 0; i < 40; ++i) detector.process(upper_only.first, upper_only.second, 1 + .1 * i);
    const std::string path = "/tmp/crane-hook-reference-unit-test.yaml";
    std::string report;
    assert(detector.saveReference(path, "world", report));
    assert(detector.loadReference(path, "world", report));
    std::filesystem::remove(path);
    const auto beam_only = scene(3.85, false);
    auto rejected = detector.process(beam_only.first, beam_only.second, 10, 3.8, .3);
    assert(!rejected.left.detected && !rejected.left.initialized);
    assert(rejected.left.selection_reason == "REJECT_BEAM_LEVEL");
    assert(rejected.left.candidate_details.find("reject_reason=REJECT_BEAM_LEVEL") != std::string::npos);
    const auto both = scene(3.85, true);
    for (int i = 0; i < 3; ++i) {
      const auto & found = detector.process(both.first, both.second, 10.1 + .1 * i, 3.8, .3);
      assert(found.left.detected && std::abs(found.left.z - 4.6) < .03);
      assert(found.left.candidate_details.find("REJECT_BEAM_LEVEL") != std::string::npos);
    }
    assert(detector.getResult().left.initialized);
    const auto memory = detector.getResult().left;
    for (int i = 0; i < 8; ++i) {
      const auto descending = scene(4.5 - .1 * i, false);
      const auto & tracked = detector.process(descending.first, descending.second, 10.4 + .1 * i, 3.8, .3);
      assert(tracked.left.detected && tracked.left.initialized);
      assert(tracked.left.reference_width == memory.reference_width);
      assert(tracked.left.reference_min == memory.reference_min);
      assert(tracked.left.candidate_details.find("REJECT_BEAM_LEVEL") == std::string::npos);
    }
    detector.reset();
    const auto rope = scene(3.85, false, true);
    assert(!detector.process(rope.first, rope.second, 20, 3.8, .3).left.detected);
    assert(detector.getResult().left.candidate_details.find("REJECT_BEAM_LEVEL") != std::string::npos);
    detector.reset();
    const auto boundary = scene(4.1, false);
    assert(!detector.process(boundary.first, boundary.second, 21, 3.8, .3).left.detected);
    // Missing Beam data must not reuse a previous Beam Z or stop acquisition.
    assert(detector.process(beam_only.first, beam_only.second, 21.1).left.detected);
    // Verify the production caller passes this frame's ROI median on the very
    // first frame, then keeps searching above it. No one-frame-old Beam gate.
    CraneBeamConfig cfg;
    cfg.beam_roi_x_min = cfg.beam_roi_y_min = -.2;
    cfg.beam_roi_x_max = cfg.beam_roi_y_max = .2;
    cfg.beam_roi_z_min = 3.5; cfg.beam_roi_z_max = 4.0;
    cfg.bg_roi_z_max = .5; // Ground reference excludes these dense elevated bodies.
    cfg.beam_z_band = .3;
    CraneBeamDetector integrated(cfg);
    for (int i = 0; i < 4; ++i) {
      auto input = scene(3.85, i > 0);
      addBox(input.first, input.second, 0, 0, 3.8, .3, .3, .02, .01);
      const auto & result = integrated.process(input.first, input.second, 30 + .1 * i);
      assert(result.beam_z_valid && std::abs(result.z_beam_raw - 3.8) < .02);
      assert(result.hook.left.detected == (i > 0));
      if (i > 0) assert(std::abs(result.hook.left.z - 4.6) < .03);
      if (i == 3) assert(result.hook.left.initialized);
      assert(result.hook.left.candidate_details.find("REJECT_BEAM_LEVEL") != std::string::npos);
    }
    std::cout << "[Test 39] Beam-level rejection, higher reference candidate, Beam-only NONE, rope, boundary, invalid Beam and confirmed descent passed." << std::endl;
  }

  // 40. Confirmed measured KF XY drives fixed-anchor START and latched STOP.
  {
    CraneBeamConfig config;
    config.beam_roi_z_max = 2.0; // The Hook at 4.6m is not a Beam-level candidate.
    config.bg_roi_z_max = .5;
    { std::ofstream file(movement_reference_path); file << movement_reference_yaml; }
    const auto input = [](double dx, double dy, int mask, bool opposite = false) {
      std::vector<PointXYZIRT> cloud;
      std::vector<int> indices;
      for (int side = 0; side < 2; ++side) {
        if (mask == 0) continue; // genuinely empty ROI: no fallback measurement
        const double x = side == 0 ? -.9 : .9;
        addBox(cloud, indices, x, 0, 0, .8, .8, 0, .04);
        if (mask & (1 << side)) addBox(cloud, indices,
            x + (opposite && side == 1 ? -dx : dx),
            opposite && side == 1 ? -dy : dy, 4.6, .32, .16, .24, .02);
      }
      return std::make_pair(cloud, indices);
    };
    for (int axis = 0; axis < 2; ++axis) {
      CraneBeamDetector move(config);
      std::string report;
      assert(move.getHookDetector().loadReference(movement_reference_path, "world", report));
      double stamp = 0;
      const auto frame = [&](double displacement, int mask = 3) {
        auto p = input(axis == 0 ? displacement : 0, axis == 1 ? displacement : 0, mask);
        return move.process(p.first, p.second, stamp += .1);
      };
      for (int i = 0; i < 20; ++i) assert(!frame(0).crane_moving);
      const auto stopped = frame(0);
      if (!stopped.crane_move_valid || stopped.crane_move_source_mask != 3)
        std::cerr << "move init left=" << stopped.hook.left.tracking_state << " " << stopped.hook.left.selection_reason
                  << " right=" << stopped.hook.right.tracking_state << " " << stopped.hook.right.selection_reason
                  << " reason=" << stopped.crane_move_reason << " sources=" << stopped.crane_move_source_mask
                  << " details=" << stopped.hook.left.candidate_details << std::endl;
      assert(stopped.crane_move_valid && stopped.crane_move_source_mask == 3);
      for (double displacement : {.02, .05, .08, .11, .14, .18}) frame(displacement);
      auto moving = frame(.18);
      assert(moving.crane_moving);
      assert(std::abs(axis == 0 ? moving.crane_move_net_dx : moving.crane_move_net_dy) >= .10);
      // Prediction/loss cannot complete STOP, even if the last KF position was stable.
      for (int i = 0; i < 2; ++i) {
        auto missing = frame(.18, 0);
        assert(missing.crane_moving && !missing.crane_move_valid && missing.crane_move_measured_frames == 0);
      }
      for (int i = 0; i < 4; ++i) assert(frame(.18).crane_moving);
      bool stopped_again = false;
      for (int i = 0; i < 20; ++i) {
        auto r = frame(.18);
        if (!r.crane_moving) {
          assert(r.crane_move_reason == "STOP_STABLE_ANCHOR_UPDATED");
          assert(r.crane_move_measured_frames == 5);
          assert(r.crane_move_x_span <= .02 && r.crane_move_y_span <= .05);
          assert(std::abs(axis == 0 ? r.crane_move_net_dx : r.crane_move_net_dy) < .02);
          assert(std::abs(axis == 0 ? r.crane_move_anchor_x - stopped.crane_move_anchor_x :
              r.crane_move_anchor_y - stopped.crane_move_anchor_y) > .10);
          stopped_again = true;
          break;
        }
      }
      assert(stopped_again);
      for (int i = 0; i < 6; ++i) assert(!frame(.18).crane_moving);
      // A second real translation uses the newly fixed anchor, not the old origin.
      for (double displacement : {.20, .23, .26, .29, .32, .35}) frame(displacement);
      assert(frame(.35).crane_moving);
      for (int i = 0; i < 10; ++i) assert(frame(.35, 0).crane_moving);
      bool recovered_stop = false;
      for (int i = 0; i < 30; ++i) {
        auto r = frame(.35);
        if (!r.crane_moving) { assert(r.crane_move_valid); recovered_stop = true; break; }
      }
      assert(recovered_stop);
    }
    CraneBeamDetector pair(config);
    std::string report;
    assert(pair.getHookDetector().loadReference(movement_reference_path, "world", report));
    double stamp = 0;
    const auto frame = [&](double swing, int mask, bool opposite) {
      auto p = input(swing, 0, mask, opposite);
      return pair.process(p.first, p.second, stamp += .1);
    };
    for (int i = 0; i < 20; ++i) assert(!frame(0, 3, false).crane_moving);
    for (int i = 0; i < 20; ++i) {
      auto r = frame(.14 * std::sin(i * .3), 3, true);
      assert(!r.crane_moving && r.crane_move_source_mask == 3);
    }
    for (int i = 0; i < 20; ++i) assert(!frame(0, 3, false).crane_moving);
    for (int mask : {1, 2, 3}) for (int i = 0; i < 10; ++i) assert(!frame(0, mask, false).crane_moving);
    pair.reset();
    for (int i = 0; i < 2; ++i) {
      auto r = frame(0, 1, false);
      assert(!r.crane_moving && !r.crane_move_valid); // Unconfirmed candidates are not sources.
    }
    CraneBeamDetector no_reference(config);
    for (int i = 0; i < 20; ++i) {
      auto p = input(i * .015, 0, 1);
      const auto & r = no_reference.process(p.first, p.second, 100 + i * .1);
      assert(!r.crane_moving && !(r.crane_move_source_mask & 1));
      // Left body still requires a saved reference; Right fallback does not.
    }
    std::filesystem::remove(movement_reference_path);
    std::cout << "[Test 40] Hook KF X/Y START, fixed/rebased anchors, five-frame STOP, prediction/long-loss latch, opposite swing and side availability passed." << std::endl;
  }

  std::cout << "\n=======================================================" << std::endl;
  // 41. Continuous highest-Z fallback, held empty ROI, recovery and independent sides.
  {
    cx128s2_bev_perception::CraneHookConfig hc;
    cx128s2_bev_perception::CraneHookDetector detector(hc);
    double stamp = 0.0;
    auto frame = [&](bool left_body, bool right_body, double highest_z,
                     bool empty_left = false, bool invalid = false) {
      std::vector<PointXYZIRT> points;
      std::vector<int> indices;
      auto add = [&](float x, float y, float z) {
        PointXYZIRT p; p.x = x; p.y = y; p.z = z;
        indices.push_back(static_cast<int>(points.size())); points.push_back(p);
      };
      for (int side = 0; side < 2; ++side) {
        if (side == 0 && empty_left) continue;
        const float cx = side == 0 ? -.85f : .85f;
        for (int i = 0; i < 12; ++i) for (int j = 0; j < 12; ++j)
          add(cx + (i - 5.5f) * .035f, (j - 5.5f) * .035f, 0.f);
        if (side == 0 ? left_body : right_body) {
          for (int i = 0; i < 8; ++i) for (int j = 0; j < 8; ++j)
            for (int k = 0; k < 7; ++k)
              add(cx + (i - 3.5f) * .02f, (j - 3.5f) * .02f, 3.65f + k * .05f);
        }
        if (side == 0 && highest_z >= 0)
          add(-1.1f + (highest_z - 4.5) * .1, .45f - (highest_z - 4.5) * .1, highest_z);
      }
      if (invalid) {
        add(-.8f, 0.f, std::numeric_limits<float>::quiet_NaN());
        add(std::numeric_limits<float>::infinity(), 0.f, 6.f);
        add(-.8f, 0.f, 6.3f); // outside the existing upper Z ROI
        indices.push_back(-1); indices.push_back(static_cast<int>(points.size()) + 10);
      }
      stamp += .1;
      return detector.process(points, indices, stamp);
    };
    for (int i = 0; i < 3; ++i) {
      auto v = frame(true, true, -1);
      assert(v.left.detected && v.right.detected);
      assert(v.left.position_tracking_state == "TRACKING");
    }
    for (int i = 0; i < 20; ++i) {
      const auto v = frame(false, true, 4.5 + i * .02, false, true);
      assert(!v.left.detected && v.left.body_indices.empty());
      assert(v.left.position_tracking_state == "FALLBACK_TRACKING");
      assert(v.left.position_updated && v.left.fallback_point_valid);
      assert(std::abs(v.left.tracking_x - (-1.1 + i * .002)) < 1e-6);
      assert(std::abs(v.left.tracking_y - (.45 - i * .002)) < 1e-6);
      assert(std::abs(v.left.tracking_z - (4.5 + i * .02)) < 1e-6);
      assert(v.right.detected && v.right.position_tracking_state == "TRACKING");
      assert(!v.prepare_ready);
    }
    const auto before = detector.getResult().left;
    for (int i = 0; i < 10; ++i) {
      const auto v = frame(false, true, -1, true, true);
      assert(v.left.position_tracking_state == "LOST");
      assert(v.left.position_valid && !v.left.position_updated && !v.left.fallback_point_valid);
      assert(v.left.tracking_z == before.tracking_z && v.left.tracking_x == before.tracking_x);
      assert(v.right.detected);
    }
    auto resumed = frame(false, true, 5.2);
    assert(resumed.left.position_tracking_state == "FALLBACK_TRACKING");
    auto recovered = frame(true, true, 5.2);
    assert(recovered.left.detected && recovered.left.initialized);
    assert(recovered.left.position_tracking_state == "TRACKING");
    assert(!recovered.left.fallback_point_valid && !recovered.prepare_ready);
    assert(std::abs(recovered.left.filtered_z - recovered.left.z) < 1e-9);
    for (int i = 0; i < 10; ++i) {
      auto v = frame(true, false, 5.2);
      assert(v.left.detected && v.left.position_tracking_state == "TRACKING");
      assert(!v.right.detected && v.right.position_tracking_state == "FALLBACK_TRACKING");
    }
    detector.reset();
    assert(!detector.getResult().left.position_valid);
    const auto cold = frame(false, false, 5.8);
    assert(cold.left.position_tracking_state == "FALLBACK_TRACKING" && !cold.left.initialized);
    assert(!cold.prepare_ready);
    hc.tracking_enabled = false; detector.updateConfig(hc);
    assert(frame(false, false, 5.9).left.position_tracking_state == "FALLBACK_TRACKING");
    std::cout << "[Test 41] Continuous highest-Z fallback / empty hold / independent recovery passed." << std::endl;
  }

  // 42. Highest-Z fallback drives crane XY movement without body/reference.
  {
    CraneBeamConfig config;
    config.crane_move_stop_frames = 5;
    CraneBeamDetector detector(config);
    double stamp = 0;
    auto frame = [&](double dx, double dy, int mask, double z = 5.0) {
      std::vector<PointXYZIRT> cloud;
      std::vector<int> indices;
      for (int side = 0; side < 2; ++side) {
        if (!(mask & (1 << side))) continue;
        PointXYZIRT p;
        p.x = (side == 0 ? -.9 : .9) + dx;
        p.y = dy; p.z = z;
        indices.push_back(static_cast<int>(cloud.size())); cloud.push_back(p);
      }
      return detector.process(cloud, indices, stamp += .1);
    };
    for (int axis = 0; axis < 2; ++axis) {
      detector.reset();
      for (int i = 0; i < 5; ++i) assert(!frame(0, 0, 3).crane_moving);
      assert(detector.getHookDetector().getResult().left.position_tracking_state == "FALLBACK_TRACKING");
      auto moved = frame(axis == 0 ? .15 : 0, axis == 1 ? .15 : 0, 3);
      assert(moved.crane_move_valid && moved.crane_move_source_mask == 3 && moved.crane_moving);
      assert(!moved.hook.left.detected && !moved.hook.prepare_ready);
      assert(moved.state == CraneBeamState::IDLE && moved.working_status == 0 && !moved.length_locked);
      for (int i = 0; i < 12; ++i) {
        auto empty = frame(0, 0, 0);
        assert(empty.crane_moving && !empty.crane_move_valid);
      }
      for (int i = 0; i < 5; ++i) {
        auto stopped = frame(axis == 0 ? .15 : 0, axis == 1 ? .15 : 0, 3);
        assert(stopped.crane_moving == (i < 4));
        if (i == 4) assert(stopped.crane_move_reason == "STOP_STABLE_ANCHOR_UPDATED");
      }
    }
    // Pure Z motion does not imply horizontal crane movement.
    detector.reset();
    for (int i = 0; i < 20; ++i) assert(!frame(0, 0, 1, 4.5 + i * .05).crane_moving);
    // A single independent fallback side can start movement.
    assert(frame(.15, 0, 1).crane_moving);
    detector.reset();
    for (int i = 0; i < 5; ++i) frame(0, 0, 2);
    assert(frame(0, .15, 2).crane_moving);
    std::cout << "[Test 42] Fallback XY START/STOP, empty hold, independent sides and measured-only work FSM passed." << std::endl;
  }

  std::cout << ">>> Hook-coupled FSM, geometry and existing Hook regressions passed! <<<" << std::endl;
  std::cout << "=======================================================" << std::endl;
  return 0;
}
