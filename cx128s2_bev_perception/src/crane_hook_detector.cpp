#include <cx128s2_bev_perception/crane_hook_detector.hpp>

#include <cmath>
#include <algorithm>
#include <vector>
#include <tuple>
#include <sstream>
#include <iomanip>
#include <map>
#include <set>
#include <fstream>
#include <filesystem>
#include <yaml-cpp/yaml.h>

namespace cx128s2_bev_perception {

CraneHookDetector::CraneHookDetector(const CraneHookConfig & config)
: config_(config) {
  reset();
}

void CraneHookDetector::updateConfig(const CraneHookConfig & config) {
  const bool profile_changed = config.roi_left_x_min != config_.roi_left_x_min ||
      config.roi_left_x_max != config_.roi_left_x_max ||
      config.roi_right_x_min != config_.roi_right_x_min ||
      config.roi_right_x_max != config_.roi_right_x_max ||
      config.roi_y_min != config_.roi_y_min || config.roi_y_max != config_.roi_y_max ||
      config.roi_z_max != config_.roi_z_max || config.window_size_z != config_.window_size_z ||
      config.window_step_z != config_.window_step_z ||
      config.tracking_measurement_std != config_.tracking_measurement_std;
  const bool reset_tracks = profile_changed || config.tracking_enabled != config_.tracking_enabled ||
      config.roi_z_min != config_.roi_z_min;
  if (profile_changed) {
    saved_reference_ = {};
    reference_samples_ = {};
    reference_rejections_ = {};
    learning_reference_ = false;
  }
  config_ = config;
  if (reset_tracks) {
    // Tracking configuration must not alter the existing readiness latch.
    const auto readiness = result_;
    const int prepare_counter = result_.prepare_counter;
    const int release_counter = result_.release_counter;
    reset();
    result_.prepare_ready = readiness.prepare_ready;
    result_.prepare_sides = readiness.prepare_sides;
    result_.prepare_side_counters = readiness.prepare_side_counters;
    result_.release_side_counters = readiness.release_side_counters;
    result_.prepare_counter = prepare_counter;
    result_.release_counter = release_counter;
  }
}

void CraneHookDetector::reset() {
  result_ = CraneHookResult{};
  xy_filters_ = {};
  z_filters_ = {};
  last_measurement_stamps_ = {};
  last_stamp_ = std::numeric_limits<double>::quiet_NaN();
  ground_history_ = {};
  confirmed_bodies_ = {};
}

void CraneHookDetector::beginReferenceLearning() {
  reference_samples_ = {};
  reference_rejections_ = {};
  learning_reference_ = true;
  reset();
}

void CraneHookDetector::cancelReferenceLearning() {
  learning_reference_ = false;
  reference_samples_ = {};
  reference_rejections_ = {};
  reset();
}

std::string CraneHookDetector::referenceSummary() const {
  std::ostringstream out;
  out << "learning=" << learning_reference_;
  const std::array<const char *, 5> reasons{"not_confirmed_or_ground", "partial_or_support",
      "rope_or_unresolved_shape", "unstable_tracking", "sample_limit"};
  for (size_t side = 0; side < 2; ++side) {
    out << " " << (side == 0 ? "LEFT" : "RIGHT") << " loaded=" << saved_reference_[side].reference_valid
        << " frames=" << reference_samples_[side].size();
    if (saved_reference_[side].reference_valid) {
      const auto & saved = saved_reference_[side];
      out << " saved_xy=" << saved.reference_width << "/" << saved.reference_length
          << " saved_height=" << saved.reference_max[2] - saved.reference_min[2]
          << " saved_surface_ratio=" << saved.reference_surface_ratio;
    }
    for (size_t feature : {size_t{0}, size_t{1}, size_t{2}, size_t{9}, size_t{10}, size_t{11}}) {
      if (reference_samples_[side].empty()) break;
      std::vector<double> values;
      for (const auto & sample : reference_samples_[side]) values.push_back(sample[feature]);
      std::sort(values.begin(), values.end());
      out << " " << (feature == 0 ? "width" : feature == 1 ? "length" : feature == 2 ? "height" :
          feature == 9 ? "points" : feature == 10 ? "xy_cells" : "surface_ratio")
          << "=" << values[values.size() / 2] << " p10/p90=" << values[(values.size() - 1) / 10]
          << "/" << values[9 * (values.size() - 1) / 10];
      if (feature < 3) out << " shape_spread=" <<
          (values[9 * (values.size() - 1) / 10] - values[(values.size() - 1) / 10]) /
          std::max(0.001, values[values.size() / 2]);
    }
    for (size_t reason = 0; reason < reasons.size(); ++reason)
      out << " rejected_" << reasons[reason] << "=" << reference_rejections_[side][reason];
  }
  return out.str();
}

bool CraneHookDetector::saveReference(
    const std::string & path, const std::string & frame, std::string & report) {
  if (!learning_reference_) { report = "Start HOOK_REFERENCE_BEGIN before saving"; return false; }
  try {
    YAML::Node root;
    root["version"] = 2;
    root["frame_id"] = frame;
    root["body_representation"] = "xy_surface_in_z_density_band";
    root["window_size_z"] = config_.window_size_z;
    root["window_step_z"] = config_.window_step_z;
    root["tracking_measurement_std"] = config_.tracking_measurement_std;
    root["roi"] = std::vector<double>{config_.roi_left_x_min, config_.roi_left_x_max,
        config_.roi_right_x_min, config_.roi_right_x_max, config_.roi_y_min, config_.roi_y_max, config_.roi_z_max};
    auto references = saved_reference_;
    std::ostringstream summary;
    bool any = false;
    for (size_t side = 0; side < 2; ++side) {
      const auto & samples = reference_samples_[side];
      const size_t minimum = static_cast<size_t>(std::max(30, config_.reference_min_frames));
      if (samples.size() < minimum) {
        summary << (side == 0 ? "LEFT" : "RIGHT") << " insufficient_frames=" << samples.size() << "; ";
        references[side] = HookDetection{};
        continue;
      }
      std::array<double, 3> center{};
      for (size_t feature = 0; feature < 3; ++feature) {
        std::vector<double> values;
        for (const auto & sample : samples) values.push_back(sample[feature]);
        std::sort(values.begin(), values.end());
        center[feature] = values[values.size() / 2];
      }
      std::vector<std::array<double, 12>> inliers;
      for (const auto & sample : samples) {
        bool stable = true;
        for (size_t feature = 0; feature < 3; ++feature) {
          const double tolerance = std::max(2.0 * config_.tracking_measurement_std,
              0.3 * center[feature]);
          if (std::abs(sample[feature] - center[feature]) > tolerance) stable = false;
        }
        if (stable) inliers.push_back(sample);
      }
      if (inliers.size() < minimum || inliers.size() * 4 < samples.size() * 3) {
        summary << (side == 0 ? "LEFT" : "RIGHT") << " rejected_consistent_inliers="
            << inliers.size() << "/" << samples.size() << " required=" << minimum << "; ";
        references[side] = HookDetection{};
        continue;
      }
      std::array<double, 12> medians{}, p10{}, p90{};
      for (size_t feature = 0; feature < medians.size(); ++feature) {
        std::vector<double> values;
        for (const auto & sample : inliers) values.push_back(sample[feature]);
        std::sort(values.begin(), values.end());
        medians[feature] = values[values.size() / 2];
        p10[feature] = values[(values.size() - 1) / 10];
        p90[feature] = values[9 * (values.size() - 1) / 10];
      }
      bool stable_distribution = true;
      for (size_t feature = 0; feature < 3; ++feature) {
        if (p90[feature] - p10[feature] > std::max(2.0 * config_.tracking_measurement_std,
            0.3 * medians[feature])) stable_distribution = false;
      }
      if (!stable_distribution) {
        summary << (side == 0 ? "LEFT" : "RIGHT") << " rejected_unstable_inlier_geometry; ";
        references[side] = HookDetection{};
        continue;
      }
      YAML::Node node;
      node["frames"] = inliers.size();
      node["excluded_geometry_outliers"] = samples.size() - inliers.size();
      node["median"] = std::vector<double>(medians.begin(), medians.end());
      node["p10"] = std::vector<double>(p10.begin(), p10.end());
      node["p90"] = std::vector<double>(p90.begin(), p90.end());
      node["feature_order"] = "width,length,height,min_x,min_y,min_z,max_x,max_y,max_z,points,xy_cells,surface_ratio";
      node["rejection_counts"] = std::vector<size_t>(reference_rejections_[side].begin(), reference_rejections_[side].end());
      root[side == 0 ? "left" : "right"] = node;
      auto & reference = references[side];
      reference.reference_valid = true;
      reference.reference_width = medians[0]; reference.reference_length = medians[1];
      reference.reference_surface_ratio = medians[11];
      for (size_t axis = 0; axis < 3; ++axis) {
        reference.reference_min[axis] = medians[3 + axis];
        reference.reference_max[axis] = medians[6 + axis];
      }
      summary << (side == 0 ? "LEFT" : "RIGHT") << " used=" << inliers.size()
          << " width=" << medians[0] << "[" << p10[0] << "," << p90[0] << "]"
          << " length=" << medians[1] << "[" << p10[1] << "," << p90[1] << "]"
          << " height=" << medians[2] << "[" << p10[2] << "," << p90[2] << "]"
          << " surface_ratio=" << medians[11] << " points=" << medians[9] << " cells=" << medians[10]
          << " shape_spread=" << (p90[0] - p10[0]) / std::max(0.001, medians[0])
          << "/" << (p90[1] - p10[1]) / std::max(0.001, medians[1])
          << "/" << (p90[2] - p10[2]) / std::max(0.001, medians[2]) << "; ";
      any = true;
    }
    if (!any) { report = "REJECT: insufficient stable frames; " + summary.str() + referenceSummary(); return false; }
    for (size_t side = 0; side < 2; ++side) {
      if (saved_reference_[side].reference_valid && !references[side].reference_valid) {
        report = "REJECT: retaining trusted " + std::string(side == 0 ? "LEFT" : "RIGHT") +
            " reference; " + summary.str() + referenceSummary();
        return false;
      }
    }
    // Symlink installs point at source config: replace the actual data file,
    // preserving the link so a later rebuild cannot discard learned geometry.
    const auto destination = std::filesystem::weakly_canonical(path);
    if (!destination.parent_path().empty()) std::filesystem::create_directories(destination.parent_path());
    const std::string temporary = destination.string() + ".tmp";
    std::ofstream output(temporary, std::ios::trunc);
    if (!output) throw std::runtime_error("cannot open " + temporary);
    output << root;
    output.flush();
    if (!output) throw std::runtime_error("failed writing " + temporary);
    output.close();
    std::filesystem::rename(temporary, destination);
    saved_reference_ = std::move(references);
    learning_reference_ = false;
    report = "SAVED " + path + "; " + summary.str() + referenceSummary();
    return true;
  } catch (const std::exception & exception) { report = exception.what(); return false; }
}

bool CraneHookDetector::loadReference(
    const std::string & path, const std::string & frame, std::string & report) {
  try {
    const auto root = YAML::LoadFile(path);
    if (root["version"].as<int>() != 2 || root["frame_id"].as<std::string>() != frame ||
        root["body_representation"].as<std::string>() != "xy_surface_in_z_density_band" ||
        std::abs(root["window_size_z"].as<double>() - config_.window_size_z) > 1e-6 ||
        std::abs(root["window_step_z"].as<double>() - config_.window_step_z) > 1e-6 ||
        std::abs(root["tracking_measurement_std"].as<double>() - config_.tracking_measurement_std) > 1e-6)
      throw std::runtime_error("reference frame/feature configuration mismatch");
    const auto roi = root["roi"].as<std::vector<double>>();
    const std::vector<double> expected_roi{config_.roi_left_x_min, config_.roi_left_x_max,
        config_.roi_right_x_min, config_.roi_right_x_max, config_.roi_y_min, config_.roi_y_max, config_.roi_z_max};
    if (roi.size() != expected_roi.size()) throw std::runtime_error("invalid reference ROI");
    for (size_t i = 0; i < roi.size(); ++i) if (!std::isfinite(roi[i]) || std::abs(roi[i] - expected_roi[i]) > 1e-6)
      throw std::runtime_error("reference ROI mismatch");
    std::array<HookDetection, 2> loaded;
    bool any = false;
    for (size_t side = 0; side < 2; ++side) {
      const auto node = root[side == 0 ? "left" : "right"];
      if (!node) continue;
      const auto values = node["median"].as<std::vector<double>>();
      if (node["frames"].as<int>() < std::max(30, config_.reference_min_frames) || values.size() != 12)
        throw std::runtime_error("insufficient reference frames or invalid feature vector");
      for (double value : values) if (!std::isfinite(value)) throw std::runtime_error("non-finite reference feature");
      if (values[0] < 2.0 * config_.tracking_measurement_std || values[1] < values[0] ||
          values[2] <= 0.0 || values[9] <= 0.0 || values[10] < 3.0 || values[11] < .05 || values[11] > 1.0)
        throw std::runtime_error("invalid body dimensions/support");
      auto & reference = loaded[side];
      reference.reference_valid = true;
      reference.reference_width = values[0]; reference.reference_length = values[1];
      reference.reference_surface_ratio = values[11];
      for (size_t axis = 0; axis < 3; ++axis) {
        reference.reference_min[axis] = values[3 + axis];
        reference.reference_max[axis] = values[6 + axis];
        if (reference.reference_min[axis] > 0.0 || reference.reference_max[axis] < 0.0 ||
            reference.reference_max[axis] < reference.reference_min[axis])
          throw std::runtime_error("invalid relative reference bounds");
      }
      any = true;
    }
    if (!any) throw std::runtime_error("no Hook body reference");
    saved_reference_ = std::move(loaded);
    learning_reference_ = false;
    reset();
    report = "LOADED " + path + "; " + referenceSummary();
    return true;
  } catch (const std::exception & exception) {
    const bool retained = std::any_of(saved_reference_.begin(), saved_reference_.end(),
        [](const HookDetection & reference) { return reference.reference_valid; });
    report = std::string("HOOK reference not loaded; ") +
        (retained ? "retaining existing reference: " : "fallback to normal acquisition: ") + exception.what();
    return false;
  }
}

HookDetection CraneHookDetector::detectSingleHook(
    const std::vector<PointXYZIRT> & points,
    const std::vector<int> & roi_indices,
    const HookDetection * prediction, double gate_z, double innovation_gate_z,
    const HookDetection * saved_reference, double beam_z, double beam_z_separation,
    bool reacquiring) {
  HookDetection hook;
  hook.roi_indices = roi_indices;
  if (prediction) {
    hook.selection_prediction_z = prediction->predicted_z;
    hook.selection_previous_z = prediction->last_measurement_z;
    hook.selection_gate_z = innovation_gate_z;
  }

  // Reuse the fixed-ROI ground samples, excluding the Hook/rope center.
  const auto estimate_ground = [&](double center_x, double center_y, double inner_radius) {
    hook.ground_valid = false; hook.ground_z = 0.0; hook.ground_points = 0;
    std::vector<int> ground_zone_idx;
    const double outer_radius = std::hypot(
        0.5 * std::max(config_.roi_left_x_max - config_.roi_left_x_min,
                       config_.roi_right_x_max - config_.roi_right_x_min),
        0.5 * (config_.roi_y_max - config_.roi_y_min));
    for (int idx : roi_indices) {
      const auto & point = points[idx];
      const double radius = std::hypot(point.x - center_x, point.y - center_y);
      if (point.z >= config_.ground_search_z_min && point.z <= config_.ground_search_z_max &&
          radius >= inner_radius && radius <= outer_radius) ground_zone_idx.push_back(idx);
    }
    if (ground_zone_idx.size() >= config_.min_ground_points) {
    const double bin_size = config_.ground_bin_size;
    const double gz_min = config_.ground_search_z_min;
    const double gz_max = config_.ground_search_z_max;
    const int num_bins = static_cast<int>(std::ceil((gz_max - gz_min) / bin_size));

    if (num_bins > 0) {
      std::vector<std::vector<float>> bins(num_bins);
      std::vector<std::set<std::pair<int, int>>> bin_cells(num_bins);
      for (int idx : ground_zone_idx) {
        const float pz = points[idx].z;
        int b = static_cast<int>(std::floor((pz - gz_min) / bin_size));
        if (b >= 0 && b < num_bins) {
          bins[b].push_back(pz);
          bin_cells[b].emplace(static_cast<int>(std::floor(points[idx].x / bin_size)),
                               static_cast<int>(std::floor(points[idx].y / bin_size)));
        }
      }

      int peak_b = -1;
      size_t peak_cnt = 0;
      for (int b = 0; b < num_bins; ++b) {
        if (bins[b].size() >= config_.min_ground_points && bin_cells[b].size() >= 3) {
          peak_cnt = bins[b].size();
          peak_b = b;
          break;
        }
      }

      if (peak_b >= 0 && peak_cnt >= config_.min_ground_points) {
        std::vector<float> ground_pts;
        for (int db = -1; db <= 1; ++db) {
          int nb = peak_b + db;
          if (nb >= 0 && nb < num_bins) {
            ground_pts.insert(ground_pts.end(), bins[nb].begin(), bins[nb].end());
          }
        }
        if (!ground_pts.empty()) {
          std::nth_element(ground_pts.begin(), ground_pts.begin() + ground_pts.size() / 2, ground_pts.end());
          hook.ground_z = ground_pts[ground_pts.size() / 2];
          hook.ground_points = ground_pts.size();
          hook.ground_valid = true;
        }
      }
    }
  }

  };
  double center_x = prediction ? prediction->predicted_x : 0.0;
  double center_y = prediction ? prediction->predicted_y : 0.0;
  if (!prediction && !roi_indices.empty()) {
    std::vector<float> xs, ys;
    for (int idx : roi_indices) { xs.push_back(points[idx].x); ys.push_back(points[idx].y); }
    std::nth_element(xs.begin(), xs.begin() + xs.size() / 2, xs.end());
    std::nth_element(ys.begin(), ys.begin() + ys.size() / 2, ys.end());
    center_x = xs[xs.size() / 2]; center_y = ys[ys.size() / 2];
  }
  const double inner_radius = std::max(config_.ground_bin_size, 0.25 *
      (prediction && prediction->reference_valid ? prediction->reference_width : config_.window_size_z));
  estimate_ground(center_x, center_y, inner_radius);

  // The existing roi_z_min is an acquisition height, never a tracked-point gate.
  // Require an actual local ground measurement for acquisition/readiness.
  if (!prediction && !hook.ground_valid) {
    hook.selection_reason = "ACQUISITION_NO_GROUND";
    return hook;
  }
  const size_t required_points = prediction && !reacquiring ?
      std::max(size_t{3}, config_.min_body_points / 4) : config_.min_body_points;
  const double shape_noise = std::max(config_.window_step_z,
      config_.tracking_measurement_std);
  std::vector<int> hook_zone_idx;
  hook_zone_idx.reserve(roi_indices.size());
  for (int idx : roi_indices) {
    const auto & point = points[idx];
    const double lower_z = hook.ground_valid ? hook.ground_z : config_.ground_search_z_min;
    if (point.z <= lower_z || point.z > config_.roi_z_max) continue;
    if (prediction && !reacquiring) {
      // Include the remembered body around the predicted center plus the old sway gate.
      const double half_x = prediction->reference_valid ?
          std::max(std::abs(prediction->reference_min[0]), std::abs(prediction->reference_max[0])) : 0.0;
      const double half_y = prediction->reference_valid ?
          std::max(std::abs(prediction->reference_min[1]), std::abs(prediction->reference_max[1])) : 0.0;
      if (std::abs(point.x - prediction->predicted_x) > config_.tracking_gate_xy + half_x ||
          std::abs(point.y - prediction->predicted_y) > config_.tracking_gate_xy + half_y ||
          std::abs(point.z - prediction->predicted_z) > gate_z) continue;
    }
    hook_zone_idx.push_back(idx);
  }

  // Reuse body samples for medians, bounds and rotation-invariant XY dimensions.
  // One local calculation serves acquisition and tracked candidate comparison.
  const auto measure_body = [&](HookDetection & body) {
    std::vector<float> xs, ys, zs;
    double mean_x = 0.0, mean_y = 0.0;
    for (int idx : body.body_indices) {
      const auto & point = points[idx];
      xs.push_back(point.x); ys.push_back(point.y); zs.push_back(point.z);
      mean_x += point.x; mean_y += point.y;
    }
    std::sort(xs.begin(), xs.end()); std::sort(ys.begin(), ys.end()); std::sort(zs.begin(), zs.end());
    body.body_points = zs.size();
    body.x = xs[xs.size() / 2]; body.y = ys[ys.size() / 2]; body.z = zs[zs.size() / 2];
    body.body_min = {xs.front(), ys.front(), zs.front()};
    body.body_max = {xs.back(), ys.back(), zs.back()};
    std::map<std::pair<int, int>, std::vector<int>> spatial_cells;
    const double spatial_step = std::max(config_.window_step_z, 2.0 * config_.tracking_measurement_std);
    for (int idx : body.body_indices) spatial_cells[{
        static_cast<int>(std::floor(points[idx].x / spatial_step)),
        static_cast<int>(std::floor(points[idx].y / spatial_step))}].push_back(idx);
    std::vector<double> cell_x, cell_y;
    for (const auto & cell : spatial_cells) {
      double x = 0.0, y = 0.0;
      for (int idx : cell.second) { x += points[idx].x; y += points[idx].y; }
      cell_x.push_back(x / cell.second.size()); cell_y.push_back(y / cell.second.size());
    }
    std::sort(cell_x.begin(), cell_x.end()); std::sort(cell_y.begin(), cell_y.end());
    body.x = 0.5 * (cell_x[cell_x.size() / 2] + cell_x[(cell_x.size() - 1) / 2]);
    body.y = 0.5 * (cell_y[cell_y.size() / 2] + cell_y[(cell_y.size() - 1) / 2]);
    mean_x /= xs.size(); mean_y /= ys.size();
    double xx = 0.0, yy = 0.0, xy = 0.0;
    for (int idx : body.body_indices) {
      const double dx = points[idx].x - mean_x, dy = points[idx].y - mean_y;
      xx += dx * dx; yy += dy * dy; xy += dx * dy;
    }
    body.occupied_cells = spatial_cells.size();
    const double angle = 0.5 * std::atan2(2.0 * xy, xx - yy);
    const double ca = std::cos(angle), sa = std::sin(angle);
    std::vector<double> us, vs;
    for (int idx : body.body_indices) {
      const double dx = points[idx].x - mean_x, dy = points[idx].y - mean_y;
      us.push_back(ca * dx + sa * dy); vs.push_back(-sa * dx + ca * dy);
    }
    std::sort(us.begin(), us.end()); std::sort(vs.begin(), vs.end());
    const size_t low = (us.size() - 1) / 20, high = 19 * (us.size() - 1) / 20;
    body.body_width = std::min(us[high] - us[low], vs[high] - vs[low]);
    body.body_length = std::max(us[high] - us[low], vs[high] - vs[low]);
    // A Hook can present a narrow XY silhouette yet still has a vertical surface.
    // A tilted wire instead couples XY-major displacement almost linearly to Z.
    double mean_z = 0.0;
    for (int idx : body.body_indices) mean_z += points[idx].z;
    mean_z /= body.body_points;
    double uu = 0.0, zz = 0.0, uz = 0.0;
    for (int idx : body.body_indices) {
      const double u = ca * (points[idx].x - mean_x) + sa * (points[idx].y - mean_y);
      const double dz = points[idx].z - mean_z;
      uu += u * u; zz += dz * dz; uz += u * dz;
    }
    const double spread = std::sqrt((uu - zz) * (uu - zz) + 4.0 * uz * uz);
    body.body_surface_ratio = std::max(0.0, uu + zz - spread) /
        std::max(1e-12, uu + zz + spread);
  };

  if (hook_zone_idx.size() >= required_points) {
    double min_z = std::numeric_limits<double>::max(), max_z = -min_z;
    for (int idx : hook_zone_idx) {
      min_z = std::min(min_z, static_cast<double>(points[idx].z));
      max_z = std::max(max_z, static_cast<double>(points[idx].z));
    }
    const double win_size = config_.window_size_z, win_step = config_.window_step_z;
    std::vector<double> starts;
    std::vector<size_t> counts;
    // A fragment thinner than the original window still produces one candidate.
    for (double zs = min_z; zs <= std::max(min_z, max_z - win_size + win_step); zs += win_step) {
      size_t count = 0;
      for (int idx : hook_zone_idx) if (points[idx].z >= zs && points[idx].z < zs + win_size) ++count;
      starts.push_back(zs); counts.push_back(count);
      if (count >= required_points) ++hook.valid_window_count;
    }
    std::vector<HookDetection> candidates;
    for (size_t i = 0; i < counts.size();) {
      size_t end = i;
      while (end + 1 < counts.size() && counts[end + 1] == counts[i]) ++end;
      const bool peak = (i == 0 || counts[i] > counts[i - 1]) &&
          (end + 1 == counts.size() || counts[end] > counts[end + 1]);
      if (peak && counts[i] >= required_points) {
        size_t pick = i;
        if (prediction) {
          const double expected_center = 0.5 *
              (prediction->last_window_z_min + prediction->last_window_z_max) +
              prediction->predicted_z - prediction->last_measurement_z;
          for (size_t j = i + 1; j <= end; ++j) {
            if (std::abs(starts[j] + win_size * 0.5 - expected_center) <
                std::abs(starts[pick] + win_size * 0.5 - expected_center)) pick = j;
          }
        }
        std::vector<std::vector<int>> components;
        std::vector<int> band;
        for (int idx : hook_zone_idx) if (points[idx].z >= starts[pick] &&
            points[idx].z < starts[pick] + win_size) band.push_back(idx);
        // Retain the original density band: sparse LiDAR surface rows of one
        // body need not be adjacent XY cells. Reference bounds validate this union.
        components.push_back(band);
        if (prediction) {
          // Split spatially separate body/rope returns within a Z peak. Cell size
          // derives from existing measurement uncertainty, not another parameter.
          const double cell_size = std::max(win_step, 2.0 * config_.tracking_measurement_std);
          std::map<std::pair<int, int>, std::vector<int>> cells;
          for (int idx : hook_zone_idx) if (points[idx].z >= starts[pick] &&
              points[idx].z < starts[pick] + win_size) {
            cells[{static_cast<int>(std::floor(points[idx].x / cell_size)),
                   static_cast<int>(std::floor(points[idx].y / cell_size))}].push_back(idx);
          }
          while (!cells.empty()) {
            components.emplace_back();
            std::vector<std::pair<int, int>> pending{cells.begin()->first};
            components.back() = std::move(cells.begin()->second);
            cells.erase(cells.begin());
            for (size_t k = 0; k < pending.size(); ++k) {
              const auto cell = pending[k];
              for (int dx = -1; dx <= 1; ++dx) for (int dy = -1; dy <= 1; ++dy) {
                auto found = cells.find({cell.first + dx, cell.second + dy});
                if (found == cells.end()) continue;
                components.back().insert(components.back().end(), found->second.begin(), found->second.end());
                pending.push_back(found->first); cells.erase(found);
              }
            }
          }
        }
        for (auto & component : components) {
          if (component.size() < required_points) continue;
          HookDetection candidate;
          candidate.window_z_min = starts[pick]; candidate.window_z_max = starts[pick] + win_size;
          candidate.body_indices = std::move(component);
          measure_body(candidate);
          if (prediction && prediction->reference_valid) {
            // Keep only returns explainable by the remembered body around this
            // occupied-cell center; dense wires outside it cannot shift the KF.
            const auto inside_body = [&](int idx) {
              const auto & point = points[idx];
              return point.x >= candidate.x + prediction->reference_min[0] - shape_noise &&
                  point.x <= candidate.x + prediction->reference_max[0] + shape_noise &&
                  point.y >= candidate.y + prediction->reference_min[1] - shape_noise &&
                  point.y <= candidate.y + prediction->reference_max[1] + shape_noise;
            };
            candidate.body_indices.erase(std::remove_if(candidate.body_indices.begin(), candidate.body_indices.end(),
                [&](int idx) { return !inside_body(idx); }), candidate.body_indices.end());
            if (candidate.body_indices.size() < required_points) continue;
            measure_body(candidate);
          }

          if (prediction) {
            candidate.prediction_error_z = std::abs(candidate.z - prediction->predicted_z);
            candidate.previous_error_z = std::abs(candidate.z - prediction->last_measurement_z);
            candidate.previous_window_overlap = std::max(0.0,
                std::min(candidate.window_z_max, prediction->last_window_z_max) -
                std::max(candidate.window_z_min, prediction->last_window_z_min));
          }
          candidates.push_back(std::move(candidate));
        }
      }
      i = end + 1;
    }
    hook.candidate_count = candidates.size();
    hook.selection_reason = candidates.empty() ? "NO_VALID_WINDOW" :
        (prediction ? "PREDICTION_OR_SHAPE_INCONSISTENT" : "ACQUISITION_HEIGHT_OR_SHAPE_INCONSISTENT");
    const double resolution = std::max(0.001, win_step);
    std::tuple<double, double, double, double, double> best_rank;
    const HookDetection * selected = nullptr;
    bool rejected_beam_level = false;
    std::ostringstream details;
    details << std::setprecision(10);
    for (const auto & candidate : candidates) {
      bool shape_ok = true;
      const double xy_error = prediction ? std::hypot(candidate.x - prediction->predicted_x,
          candidate.y - prediction->predicted_y) : 0.0;
      for (const auto * reference : {prediction, saved_reference}) {
        if (!reference || !reference->reference_valid) continue;
        // Permit partial bodies, but reject transverse collapse or a needle with
        // much greater aspect ratio than the remembered body. Noise floors also
        // preserve pre-existing degenerate references with no resolved XY width.
        shape_ok = shape_ok && candidate.body_width + 0.5 * win_step >= (prediction && !reacquiring ? 0.25 : 0.5) * reference->reference_width &&
            candidate.body_length + 0.5 * win_step >= (prediction && !reacquiring ? 0.25 : 0.5) * reference->reference_length &&
            candidate.body_width <= 1.5 * reference->reference_width + shape_noise &&
            candidate.body_length <= 1.5 * reference->reference_length + shape_noise;
        if (reference->reference_surface_ratio > .01 && candidate.body_surface_ratio <
            (prediction && !reacquiring ? .25 : .5) * reference->reference_surface_ratio) shape_ok = false;
        const double reference_extent = reference->reference_max[2] - reference->reference_min[2];
        if (reference_extent > win_step && candidate.body_max[2] - candidate.body_min[2] +
            0.5 * win_step < (prediction && !reacquiring ? 0.25 : 0.5) * reference_extent) shape_ok = false;
        const double reference_aspect = std::max(reference->reference_length, reference_extent) /
            std::max(shape_noise, reference->reference_width);
        const double candidate_aspect = std::max(candidate.body_length,
            candidate.body_max[2] - candidate.body_min[2]) / std::max(shape_noise, candidate.body_width);
        // When reference width is resolved, a near-zero-width rope must not be
        // rescued solely by the noise allowance used for ordinary partials.
        if (reference->reference_width > 2.0 * shape_noise &&
            candidate.body_width < (prediction && !reacquiring ? 0.25 : 0.5) * reference->reference_width &&
            candidate_aspect > 2.0 * reference_aspect) shape_ok = false;
        if (reference != prediction || reacquiring) continue;
        const std::array<double, 3> center{prediction->predicted_x, prediction->predicted_y,
                                         prediction->predicted_z};
        for (size_t axis = 0; axis < 3; ++axis) {
          const double margin = axis == 2 ? innovation_gate_z : config_.tracking_gate_xy;
          if (candidate.body_min[axis] < center[axis] + reference->reference_min[axis] - margin ||
              candidate.body_max[axis] > center[axis] + reference->reference_max[axis] + margin) shape_ok = false;
        }
      }
      const bool xy_partial = prediction && prediction->reference_valid &&
          (candidate.body_width + shape_noise < prediction->reference_width ||
           candidate.body_length + shape_noise < prediction->reference_length);
      const double partial_z_allowance = xy_partial ?
          0.5 * std::max(0.0, prediction->reference_max[2] - prediction->reference_min[2] -
              (candidate.body_max[2] - candidate.body_min[2])) : 0.0;
      // Reject this candidate only. Confirmed identity updates may approach
      // the Beam; cold acquisition still evaluates all remaining Z windows.
      const bool beam_level = !prediction && std::isfinite(beam_z) &&
          std::isfinite(beam_z_separation) && beam_z_separation >= 0.0 &&
          std::abs(candidate.z - beam_z) <= beam_z_separation + 1e-6;
      rejected_beam_level = rejected_beam_level || beam_level;
      // Full-ROI recovery keeps acquisition height validation. A remembered
      // low Hook may recover within the existing maximum vertical tracking gate.
      const bool recovery_height_ok = hook.ground_valid &&
          (candidate.body_min[2] - hook.ground_z + 1e-6 >= config_.roi_z_min ||
           (prediction && candidate.previous_error_z <= config_.tracking_gate_z_max));
      const bool admissible = !beam_level && (reacquiring ? shape_ok && recovery_height_ok : prediction ?
          candidate.prediction_error_z <= std::min(gate_z, innovation_gate_z + partial_z_allowance) &&
          std::abs(candidate.x - prediction->predicted_x) <= config_.tracking_gate_xy &&
          std::abs(candidate.y - prediction->predicted_y) <= config_.tracking_gate_xy && shape_ok :
          // Point coordinates are float32: tolerate only their rounding at 3m.
          candidate.body_min[2] - hook.ground_z + 1e-6 >= config_.roi_z_min && shape_ok);
      details << "z=" << candidate.z << ";pts=" << candidate.body_points
              << ";win=" << candidate.window_z_min << "/" << candidate.window_z_max
              << ";pred_err=" << candidate.prediction_error_z << ";prev_err=" << candidate.previous_error_z
              << ";overlap=" << candidate.previous_window_overlap << ";xy_err=" << xy_error
              << ";width=" << candidate.body_width << ";length=" << candidate.body_length
              << ";surface_ratio=" << candidate.body_surface_ratio << ";shape_ok=" << shape_ok
              << ";beam_z=" << beam_z << ";beam_z_separation=" << beam_z_separation
              << ";reject_reason=" << (beam_level ? "REJECT_BEAM_LEVEL" : "NONE")
              << ";eligible=" << admissible << "|";
      if (!admissible) continue;
      const auto rank = prediction && !reacquiring ? std::make_tuple(
          std::floor(candidate.prediction_error_z / resolution),
          std::floor(candidate.previous_error_z / resolution),
          -candidate.previous_window_overlap, xy_error, -static_cast<double>(candidate.body_points)) :
          std::make_tuple(0.0, 0.0, 0.0, 0.0, -static_cast<double>(candidate.body_points));
      if (!selected || rank < best_rank) { selected = &candidate; best_rank = rank; }
    }
    hook.candidate_details = details.str();
    if (!selected && rejected_beam_level) hook.selection_reason = "REJECT_BEAM_LEVEL";
    if (selected) {
      hook.detected = true;
      hook.body_indices = selected->body_indices;
      hook.body_points = selected->body_points;
      hook.x = selected->x; hook.y = selected->y; hook.z = selected->z;
      hook.body_min = selected->body_min; hook.body_max = selected->body_max;
      hook.occupied_cells = selected->occupied_cells;
      hook.body_surface_ratio = selected->body_surface_ratio;
      hook.body_width = selected->body_width; hook.body_length = selected->body_length;
      hook.window_z_min = selected->window_z_min; hook.window_z_max = selected->window_z_max;
      hook.prediction_error_z = selected->prediction_error_z;
      hook.previous_error_z = selected->previous_error_z;
      hook.previous_window_overlap = selected->previous_window_overlap;
      hook.candidate_switched = prediction && selected->previous_window_overlap <= 0.0;
      hook.partial_observation = prediction && prediction->reference_valid &&
          (hook.body_width + shape_noise < prediction->reference_width ||
           hook.body_length + shape_noise < prediction->reference_length ||
           hook.body_max[2] - hook.body_min[2] + win_step <
               prediction->reference_max[2] - prediction->reference_min[2]);
      hook.selection_reason = prediction ?
          (hook.partial_observation ? "REFERENCE_PARTIAL" : "PREDICTION_REFERENCE_CONTINUITY") : "ACQUISITION_MAX_DENSITY";
      hook.body_density = static_cast<double>(hook.body_points) / win_size;
      hook.score = static_cast<double>(hook.body_points) / hook_zone_idx.size();
    }
  }

  if (hook.detected) {
    estimate_ground(hook.x, hook.y,
        std::max(config_.ground_bin_size, 0.25 * hook.body_width));
    if (!prediction && (!hook.ground_valid ||
        hook.body_min[2] - hook.ground_z + 1e-6 < config_.roi_z_min)) {
      hook.detected = false;
      hook.selection_reason = hook.ground_valid ? "ACQUISITION_BELOW_MIN_HEIGHT" : "ACQUISITION_NO_GROUND";
      hook.body_indices.clear(); hook.body_points = 0;
      hook.x = hook.y = hook.z = 0.0;
    }
  }

  // 5. Vertical Physical Distance between Hook and Ground [m]
  if (hook.detected && hook.ground_valid) {
    hook.ground_distance = std::max(0.0, hook.body_min[2] - hook.ground_z);
  } else {
    hook.ground_distance = 0.0;
  }

  return hook;
}

const CraneHookResult & CraneHookDetector::process(
    const std::vector<PointXYZIRT> & points,
    const std::vector<int> & valid_indices, double stamp,
    double beam_z, double beam_z_separation) {
  if (!config_.enabled || !std::isfinite(stamp)) {
    reset();
    return result_;
  }
  if (std::isfinite(last_stamp_) && stamp == last_stamp_) return result_;
  // A new replay/session must acquire real measurements again.
  if (std::isfinite(last_stamp_) && stamp < last_stamp_) reset();
  const double dt = std::isfinite(last_stamp_) ? stamp - last_stamp_ : 0.0;
  last_stamp_ = stamp;

  // Partition the already-transformed valid points once for both Hook ROIs.
  std::vector<int> left_roi_indices, right_roi_indices;
  left_roi_indices.reserve(5000);
  right_roi_indices.reserve(5000);
  for (int idx : valid_indices) {
    if (idx < 0 || static_cast<size_t>(idx) >= points.size()) continue;
    const auto & p = points[idx];
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
    if (!(p.y >= config_.roi_y_min && p.y <= config_.roi_y_max)) continue;
    if (p.x >= config_.roi_left_x_min && p.x <= config_.roi_left_x_max)
      left_roi_indices.push_back(idx);
    // Independent membership preserves behavior when configured ROIs overlap.
    if (p.x >= config_.roi_right_x_min && p.x <= config_.roi_right_x_max)
      right_roi_indices.push_back(idx);
  }
  const std::array<const std::vector<int> *, 2> rois{&left_roi_indices, &right_roi_indices};
  const std::array<HookDetection *, 2> hooks{&result_.left, &result_.right};
  std::array<bool, 2> descending_measurements{};
  for (size_t side = 0; side < hooks.size(); ++side) {
    auto previous = std::move(*hooks[side]);
    auto & xy = xy_filters_[side];
    auto & vertical = z_filters_[side];
    const bool expired = previous.initialized &&
        stamp - last_measurement_stamps_[side] >
            std::max(0.0, config_.tracking_prediction_timeout_sec);
    if (previous.initialized && !expired) {
      // Reuse the existing 2D CV KF: XY directly, Z in its first axis.
      // The unused second axis in the vertical filter stays independent at zero.
      double remaining = dt;
      while (remaining > 1e-9) {
        const double step = std::min(remaining, 0.1);
        xy.predict(step, config_.tracking_accel_std_xy);
        vertical.predict(step, config_.tracking_accel_std_z);
        remaining -= step;
      }
      previous.predicted_x = xy.x();
      previous.predicted_y = xy.y();
      previous.predicted_z = vertical.x();
    }
    const double gate_z = std::min(
        std::max(config_.tracking_gate_z, config_.tracking_gate_z_max),
        config_.tracking_gate_z + std::abs(vertical.vx()) * dt);
    // Association uncertainty is separate from the unchanged point-search gate.
    // It expands with KF covariance during missing frames and with predicted
    // motion; it never hard-gates distance to a fixed previous measured Z.
    const double innovation_std = std::sqrt(std::max(0.0, vertical.positionVarianceX()) +
        config_.tracking_measurement_std * config_.tracking_measurement_std);
    const double innovation_gate_z = std::min(gate_z,
        std::max(config_.window_size_z * 0.5, 3.0 * innovation_std) +
        std::abs(vertical.vx()) * dt);
    auto hook = detectSingleHook(points, *rois[side],
        previous.initialized ? &previous : nullptr, gate_z, innovation_gate_z,
        !learning_reference_ && saved_reference_[side].reference_valid ? &saved_reference_[side] : nullptr,
        beam_z, beam_z_separation);
    // Search the same full ROI with the remembered body shape after a dropout.
    // Do not center association on a highest-Z return (which may be a rope).
    bool recovered = false;
    if ((!hook.detected || expired || !previous.initialized) && config_.tracking_enabled &&
        confirmed_bodies_[side].initialized &&
        (expired || previous.position_tracking_state != "TRACKING")) {
      auto remembered = confirmed_bodies_[side];
      remembered.predicted_x = remembered.x;
      remembered.predicted_y = remembered.y;
      remembered.predicted_z = remembered.z;
      // First reuse ordinary partial-body validation around the last real body.
      // Fallback coordinates and stale extrapolated KF velocity are not inputs.
      const double remembered_gate = std::max(config_.window_size_z * 0.5,
          3.0 * config_.tracking_measurement_std);
      auto candidate = detectSingleHook(points, *rois[side], &remembered,
          config_.tracking_gate_z, remembered_gate,
          !learning_reference_ && saved_reference_[side].reference_valid ? &saved_reference_[side] : nullptr,
          beam_z, beam_z_separation);
      if (!candidate.detected) {
        // A moved body must pass the existing stricter acquisition shape ratios.
        candidate = detectSingleHook(points, *rois[side], &remembered,
            gate_z, innovation_gate_z,
            !learning_reference_ && saved_reference_[side].reference_valid ? &saved_reference_[side] : nullptr,
            beam_z, beam_z_separation, true);
      }
      if (candidate.detected) {
        hook = std::move(candidate);
        recovered = true;
      }
    }
    if (hook.ground_valid) {
      auto & history = ground_history_[side];
      history.push_back(hook.ground_z);
      if (history.size() > 5) history.erase(history.begin());
      auto sorted = history;
      std::sort(sorted.begin(), sorted.end());
      hook.ground_z = sorted[sorted.size() / 2];
      if (hook.detected) hook.ground_distance = std::max(0.0, hook.body_min[2] - hook.ground_z);
    } else ground_history_[side].clear();
    // Acquisition must also pass the temporally stabilized ground reference,
    // rather than accepting a raw-ground outlier that moves the height above 3m.
    if (!previous.initialized && !recovered && hook.detected &&
        hook.ground_distance + 1e-6 < config_.roi_z_min) {
      hook.detected = false;
      hook.selection_reason = "ACQUISITION_BELOW_STABLE_GROUND_HEIGHT";
      hook.x = hook.y = hook.z = hook.ground_distance = 0.0;
      hook.body_points = 0;
      hook.body_indices.clear();
    }
    // Only a consecutive, actual observation of the same confirmed identity
    // supplies descent evidence. A partial/rope rejection never advances READY.
    descending_measurements[side] = previous.initialized && hook.detected && previous.detected &&
        hook.ground_valid && previous.ground_valid && !expired &&
        dt > 0.0 && dt <= config_.tracking_prediction_timeout_sec &&
        hook.z < previous.last_measurement_z - std::max(0.001, config_.tracking_measurement_std * 0.1) &&
        hook.ground_distance < previous.ground_distance - std::max(0.001, config_.tracking_measurement_std * 0.1);
    if (hook.detected && (!expired || recovered)) {
      hook.last_measurement_z = hook.z;
      hook.last_window_z_min = hook.window_z_min;
      hook.last_window_z_max = hook.window_z_max;
    } else if (previous.initialized) {
      hook.last_measurement_z = previous.last_measurement_z;
      hook.last_window_z_min = previous.last_window_z_min;
      hook.last_window_z_max = previous.last_window_z_max;
    }
    if (!config_.tracking_enabled) {
      // Body-only operation still supplies the independent continuous position.
    } else if (recovered) {
      const auto & body = confirmed_bodies_[side];
      hook.initialized = true;
      hook.reference_valid = body.reference_valid;
      hook.reference_min = body.reference_min;
      hook.reference_max = body.reference_max;
      hook.reference_width = body.reference_width;
      hook.reference_length = body.reference_length;
      hook.reference_surface_ratio = body.reference_surface_ratio;
      hook.confirm_hits = body.confirm_hits;
      const double position_std = std::max(0.001, config_.tracking_measurement_std);
      xy.initialize(hook.x, hook.y, position_std, 1.0);
      vertical.initialize(hook.z, 0.0, position_std, 1.0);
      last_measurement_stamps_[side] = stamp;
      hook.predicted_x = hook.x;
      hook.predicted_y = hook.y;
      hook.predicted_z = hook.z;
      hook.tracking_state = "MEASURED";
    } else if (previous.initialized) {
      hook.predicted_x = previous.predicted_x;
      hook.predicted_y = previous.predicted_y;
      hook.predicted_z = previous.predicted_z;
      if (expired) {
        // Report LOST in this frame; full fixed-ROI acquisition resumes next frame.
        hook.selection_reason = "PREDICTION_TIMEOUT";
        hook.candidate_switched = false;
        hook.detected = false;
        hook.x = hook.y = hook.z = hook.ground_distance = 0.0;
        hook.body_points = 0;
        hook.body_density = hook.score = 0.0;
        hook.body_indices.clear();
      }
      hook.lost_frames = hook.detected ? 0 : previous.lost_frames + 1;
      hook.initialized = !expired &&
          hook.lost_frames <= std::max(0, config_.tracking_max_lost_frames);
      if (hook.initialized) {
        hook.reference_valid = previous.reference_valid;
        hook.reference_min = previous.reference_min;
        hook.reference_max = previous.reference_max;
        hook.reference_width = previous.reference_width;
        hook.reference_length = previous.reference_length;
        hook.reference_surface_ratio = previous.reference_surface_ratio;
        hook.confirm_hits = previous.confirm_hits;
      }
      if (hook.initialized && hook.detected) {
        xy.update(hook.x, hook.y, std::max(0.001, config_.tracking_measurement_std));
        vertical.update(hook.z, 0.0, std::max(0.001, config_.tracking_measurement_std));
        last_measurement_stamps_[side] = stamp;
      }
      hook.tracking_state = hook.initialized ?
          (hook.detected ? "MEASURED" : "PREDICTED") : "LOST";
    } else if (hook.detected) {
      const bool coherent = previous.detected && dt > 0.0 &&
          dt <= config_.tracking_prediction_timeout_sec &&
          std::abs(hook.x - previous.x) <= config_.tracking_gate_xy &&
          std::abs(hook.y - previous.y) <= config_.tracking_gate_xy &&
          std::abs(hook.z - previous.z) <= config_.tracking_gate_z;
      hook.confirm_hits = coherent ? previous.confirm_hits + 1 : 1;
      if (hook.confirm_hits >= std::max(1, config_.tracking_confirm_frames)) {
        const double position_std = std::max(0.001, config_.tracking_measurement_std);
        xy.initialize(hook.x, hook.y, position_std, 1.0);
        vertical.initialize(hook.z, 0.0, position_std, 1.0);
        last_measurement_stamps_[side] = stamp;
        hook.initialized = true;
        hook.reference_valid = true;
        const std::array<double, 3> center{hook.x, hook.y, hook.z};
        for (size_t axis = 0; axis < 3; ++axis) {
          hook.reference_min[axis] = hook.body_min[axis] - center[axis];
          hook.reference_max[axis] = hook.body_max[axis] - center[axis];
        }
        hook.reference_width = hook.body_width;
        hook.reference_length = hook.body_length;
        hook.reference_surface_ratio = hook.body_surface_ratio;
        if (!learning_reference_ && saved_reference_[side].reference_valid) {
          const auto & saved = saved_reference_[side];
          hook.reference_width = std::max(hook.reference_width, saved.reference_width);
          hook.reference_length = std::max(hook.reference_length, saved.reference_length);
          hook.reference_surface_ratio = std::max(hook.reference_surface_ratio, saved.reference_surface_ratio);
          for (size_t axis = 0; axis < 3; ++axis) {
            hook.reference_min[axis] = std::min(hook.reference_min[axis], saved.reference_min[axis]);
            hook.reference_max[axis] = std::max(hook.reference_max[axis], saved.reference_max[axis]);
          }
        }
        hook.predicted_x = hook.x;
        hook.predicted_y = hook.y;
        hook.predicted_z = hook.z;
        hook.tracking_state = "MEASURED";
      } else hook.tracking_state = "ACQUIRING";
    }
    if (hook.initialized) {
      hook.filtered_x = xy.x();
      hook.filtered_y = xy.y();
      hook.filtered_z = vertical.x();
      hook.vx = xy.vx();
      hook.vy = xy.vy();
      hook.vz = vertical.vx();
      if (!hook.detected && hook.ground_valid) {
        hook.predicted_ground_distance_valid = true;
        hook.predicted_ground_distance = std::max(0.0,
            hook.filtered_z + hook.reference_min[2] - hook.ground_z);
      }
    }
    if (learning_reference_) {
      size_t rejection = 5;
      if (!hook.initialized || !hook.detected || !hook.ground_valid) rejection = 0;
      else if (hook.body_points < std::max(size_t{30}, config_.min_body_points / 2) ||
          (hook.partial_observation && (hook.body_length + config_.window_step_z < 0.75 * hook.reference_length ||
           hook.body_width + config_.window_step_z < 0.5 * hook.reference_width))) rejection = 1;
      // Multiple parallel ropes can have a large XY-major/Z covariance ratio.
      // Learn only a resolved transverse BODY surface, above existing sensor noise.
      else if (hook.body_width < 2.0 * config_.tracking_measurement_std || hook.occupied_cells < 3 || hook.body_surface_ratio < .05 ||
          hook.body_length < 0.5 * (hook.body_max[2] - hook.body_min[2])) rejection = 2;
      else if (hook.prediction_error_z > 3.0 * config_.tracking_measurement_std ||
          std::hypot(hook.x - hook.predicted_x, hook.y - hook.predicted_y) >
              3.0 * config_.tracking_measurement_std) rejection = 3;
      else if (reference_samples_[side].size() >= 10000) rejection = 4;
      if (rejection < 5) ++reference_rejections_[side][rejection];
      else reference_samples_[side].push_back({hook.body_width, hook.body_length,
          hook.body_max[2] - hook.body_min[2],
          hook.body_min[0] - hook.x, hook.body_min[1] - hook.y, hook.body_min[2] - hook.z,
          hook.body_max[0] - hook.x, hook.body_max[1] - hook.y, hook.body_max[2] - hook.z,
          static_cast<double>(hook.body_points), static_cast<double>(hook.occupied_cells), hook.body_surface_ratio});
    }
    if (hook.detected && hook.initialized) {
      confirmed_bodies_[side] = hook;
      // Avoid retaining large per-frame debug buffers in identity memory.
      confirmed_bodies_[side].body_indices.clear();
      confirmed_bodies_[side].roi_indices.clear();
      confirmed_bodies_[side].candidate_details.clear();
    }
    // The physical ROI partition is XY-only because roi_z_min is an acquisition
    // height above local ground, not a point crop. Keep the existing upper Z cap.
    hook.position_valid = previous.position_valid;
    hook.tracking_x = previous.tracking_x;
    hook.tracking_y = previous.tracking_y;
    hook.tracking_z = previous.tracking_z;
    hook.fallback_x = previous.fallback_x;
    hook.fallback_y = previous.fallback_y;
    hook.fallback_z = previous.fallback_z;
    if (hook.detected) {
      hook.position_tracking_state = "TRACKING";
      hook.position_valid = hook.position_updated = true;
      hook.tracking_x = hook.initialized ? hook.filtered_x : hook.x;
      hook.tracking_y = hook.initialized ? hook.filtered_y : hook.y;
      hook.tracking_z = hook.initialized ? hook.filtered_z : hook.z;
    } else {
      int highest = -1;
      for (int idx : *rois[side]) {
        if (points[idx].z > config_.roi_z_max) continue;
        if (highest < 0 || points[idx].z > points[highest].z) highest = idx;
      }
      if (highest >= 0) {
        const auto & point = points[highest];
        hook.position_tracking_state = "FALLBACK_TRACKING";
        hook.position_valid = hook.position_updated = hook.fallback_point_valid = true;
        hook.tracking_x = hook.fallback_x = point.x;
        hook.tracking_y = hook.fallback_y = point.y;
        hook.tracking_z = hook.fallback_z = point.z;
      } else hook.position_tracking_state = "LOST";
    }
    *hooks[side] = std::move(hook);
  }

  const bool both_valid = result_.left.detected && result_.left.ground_valid &&
                          result_.right.detected && result_.right.ground_valid;

  // 2. Calculate Distance Metrics
  if (both_valid) {
    result_.mean_ground_distance = 0.5 * (result_.left.ground_distance + result_.right.ground_distance);
    result_.max_ground_distance = std::max(result_.left.ground_distance, result_.right.ground_distance);
    result_.min_ground_distance = std::min(result_.left.ground_distance, result_.right.ground_distance);
  } else if (result_.left.detected && result_.left.ground_valid) {
    result_.mean_ground_distance = result_.left.ground_distance;
    result_.max_ground_distance = result_.left.ground_distance;
    result_.min_ground_distance = result_.left.ground_distance;
  } else if (result_.right.detected && result_.right.ground_valid) {
    result_.mean_ground_distance = result_.right.ground_distance;
    result_.max_ground_distance = result_.right.ground_distance;
    result_.min_ground_distance = result_.right.ground_distance;
  } else {
    result_.mean_ground_distance = 0.0;
    result_.max_ground_distance = 0.0;
    result_.min_ground_distance = 0.0;
  }

  // Each side owns its confirmation and release evidence. Alternating sides
  // cannot combine their measurements to manufacture a preparation latch.
  for (size_t side = 0; side < 2; ++side) {
    const auto & hook = *hooks[side];
    const bool measured = hook.detected && hook.initialized && hook.ground_valid;
    if (!result_.prepare_sides[side]) {
      result_.prepare_side_counters[side] = measured &&
          hook.ground_distance <= config_.prepare_distance &&
          (descending_measurements[side] || result_.prepare_side_counters[side] > 0) ?
          result_.prepare_side_counters[side] + 1 : 0;
      if (result_.prepare_side_counters[side] >= std::max(1, config_.prepare_confirm_frames)) {
        result_.prepare_sides[side] = true;
        result_.prepare_side_counters[side] = 0;
        result_.release_side_counters[side] = 0;
      }
    } else {
      const bool release = !measured || hook.ground_distance >
          config_.prepare_distance + config_.prepare_release_margin;
      result_.release_side_counters[side] = release ? result_.release_side_counters[side] + 1 : 0;
      if (result_.release_side_counters[side] >= std::max(1, config_.prepare_confirm_frames)) {
        result_.prepare_sides[side] = false;
        result_.release_side_counters[side] = 0;
        result_.prepare_side_counters[side] = 0;
      }
    }
  }
  result_.prepare_ready = result_.prepare_sides[0] || result_.prepare_sides[1];
  result_.prepare_counter = std::max(result_.prepare_side_counters[0], result_.prepare_side_counters[1]);
  result_.release_counter = std::max(result_.release_side_counters[0], result_.release_side_counters[1]);

  return result_;
}

}  // namespace cx128s2_bev_perception
