  std::unordered_map<int64_t, GroundCell> buildGroundMap(
      const std::vector<PointXYZIRT> & points, const std::vector<int> & indices) {
    std::unordered_map<int64_t, std::vector<float>> bins;
    std::vector<float> global;
    for (int idx : indices) {
      bins[groundKey(points[idx])].push_back(points[idx].z);
      global.push_back(points[idx].z);
    }
    double global_z = 0.0;
    if (!global.empty()) {
      std::sort(global.begin(), global.end());
      global_z = global[static_cast<size_t>(clamp(ground_percentile_, 0.0, 1.0) *
                                            static_cast<double>(global.size() - 1))];
    }
    std::unordered_map<int64_t, GroundCell> out;
    for (auto & kv : bins) {
      auto & zs = kv.second;
      std::sort(zs.begin(), zs.end());
      GroundCell c;
      if (static_cast<int>(zs.size()) >= min_points_per_ground_cell_) {
        c.z = zs[static_cast<size_t>(clamp(ground_percentile_, 0.0, 1.0) *
                                     static_cast<double>(zs.size() - 1))];
        c.valid = true;
      } else {
        c.z = global_z;
      }
      if (use_temporal_ground_map_) {
        auto & tc = temporal_ground_map_[kv.first];
        if (tc.valid) c.z = (1.0 - ground_map_alpha_) * tc.z + ground_map_alpha_ * c.z;
        tc = c;
      }
      out[kv.first] = c;
    }
    last_ground_cell_count_ = static_cast<int>(out.size());
    return out;
  }

  double groundMargin(double range) const {
    if (range < near_range_max_) return ground_margin_near_;
    if (range < mid_range_max_) return ground_margin_mid_;
    return ground_margin_long_;
  }

  void splitGroundObstacle(const std::vector<PointXYZIRT> & points, const std::vector<int> & valid_idx,
                           const std::unordered_map<int64_t, GroundCell> & ground_map,
                           std::vector<int> & ground_idx, std::vector<int> & obstacle_idx,
                           std::vector<int> & ignored_idx) {
    for (int idx : valid_idx) {
      const auto & p = points[idx];
      const auto git = ground_map.find(groundKey(p));
      const double gz = git == ground_map.end() ? p.z : git->second.z;
      const double dz = p.z - gz;
      if (dz < groundMargin(sensorRange(p.x, p.y))) {
        ground_idx.push_back(idx);
      } else if (dz >= obstacle_min_height_above_ground_ &&
                 dz <= obstacle_max_height_above_ground_) {
        obstacle_idx.push_back(idx);
      } else {
        ignored_idx.push_back(idx);
      }
    }
    last_ground_count_ = static_cast<int>(ground_idx.size());
    last_obstacle_count_ = static_cast<int>(obstacle_idx.size());
    last_ignored_count_ = static_cast<int>(ignored_idx.size());
  }

  void splitFixedGroundObstacle(const std::vector<PointXYZIRT> & points, const std::vector<int> & valid_idx,
                               std::vector<int> & ground_idx, std::vector<int> & obstacle_idx,
                               std::vector<int> & ignored_idx) {
    for (int idx : valid_idx) {
      const auto & p = points[idx];
      const double dz = p.z - fixed_ground_z_;
      if (dz < ground_margin_near_) {
        ground_idx.push_back(idx);
      } else if (dz >= obstacle_min_height_above_ground_ &&
                 dz <= obstacle_max_height_above_ground_) {
        obstacle_idx.push_back(idx);
      } else {
        ignored_idx.push_back(idx);
      }
    }
    last_ground_count_ = static_cast<int>(ground_idx.size());
    last_obstacle_count_ = static_cast<int>(obstacle_idx.size());
    last_ignored_count_ = static_cast<int>(ignored_idx.size());
  }

  std::unordered_map<int64_t, FrameCell> buildFrameCells(
      const std::vector<PointXYZIRT> & points, const std::vector<int> & indices) const {
    std::unordered_map<int64_t, FrameCell> cells;
    cells.reserve(indices.size() / 4 + 1);
    for (int idx : indices) {
      auto & c = cells[bgKey(points[idx])];
      c.indices.push_back(idx);
      c.z_sum += points[idx].z;
      c.min_z = std::min(c.min_z, static_cast<double>(points[idx].z));
      c.max_z = std::max(c.max_z, static_cast<double>(points[idx].z));
    }
    return cells;
  }

  struct ForegroundScore {
    double total = 0.0;
    double count_score = 0.0;
    double mean_z_score = 0.0;
    double max_z_score = 0.0;
    double min_z_score = 0.0;
    std::string reason;
  };

  ForegroundScore computeForegroundScore(
      const BackgroundCell & bg,
      const FrameCell & fc,
      int current_voxel_count) const {
    ForegroundScore score;
    const double bg_count = std::max(1, bg.seen_count);
    score.count_score =
        clamp(std::abs(current_voxel_count - bg.seen_count) / bg_count,
              0.0, foreground_count_score_clip_);

    const double cur_mean_z =
        fc.z_sum / std::max<size_t>(1, fc.indices.size());

    const double bg_std =
        std::max(foreground_min_bg_std_z_, bg.std_z);

    score.mean_z_score =
        clamp(std::abs(cur_mean_z - bg.mean_z) / bg_std,
              0.0, foreground_z_score_clip_);

    score.max_z_score =
        clamp(std::max(0.0, fc.max_z - bg.max_z) /
              std::max(1e-3, background_height_diff_threshold_),
              0.0, foreground_z_score_clip_);

    score.min_z_score =
        clamp(std::max(0.0, bg.min_z - fc.min_z) /
              std::max(1e-3, background_height_diff_threshold_),
              0.0, foreground_z_score_clip_);

    score.total =
        foreground_count_weight_ * score.count_score +
        foreground_mean_z_weight_ * score.mean_z_score +
        foreground_max_z_weight_ * score.max_z_score +
        foreground_min_z_weight_ * score.min_z_score;

    if (score.count_score >= score.mean_z_score &&
        score.count_score >= score.max_z_score &&
        score.count_score >= score.min_z_score) {
      score.reason = "STAT_SCORE:COUNT_CHANGE";
    } else if (score.mean_z_score >= score.max_z_score &&
               score.mean_z_score >= score.min_z_score) {
      score.reason = "STAT_SCORE:MEAN_Z_CHANGE";
    } else if (score.max_z_score >= score.min_z_score) {
      score.reason = "STAT_SCORE:MAX_Z_RISE";
    } else {
      score.reason = "STAT_SCORE:MIN_Z_DROP";
    }

    return score;
  }

  void classifyForeground(const std::vector<PointXYZIRT> & points, const std::vector<int> & valid_idx,
                          std::vector<int> & foreground_idx, std::vector<int> & background_idx,
                          bool keep_bg) {
    if (!background_ready_) {
      if (keep_bg) background_idx.insert(background_idx.end(), valid_idx.begin(), valid_idx.end());
      foreground_cell_persistence_.clear();
      for (int idx : valid_idx) last_point_bg_similarity_[idx] = 1.0;
      last_foreground_count_ = 0;
      last_static_count_ = static_cast<int>(valid_idx.size());
      return;
    }

    last_gdg_pdf_foreground_cell_count_ = 0;
    last_gdg_pdf_tested_cell_count_ = 0;
    last_gdg_pdf_tested_point_count_ = 0;
    last_gdg_pdf_foreground_point_count_ = 0;
    last_gdg_pdf_background_point_count_ = 0;
    last_gdg_pdf_unknown_cell_point_count_ = 0;
    last_gdg_pdf_count_diff_rejected_cell_count_ = 0;
    last_foreground_persistence_bypassed_ = (foreground_classifier_mode_ == "gdg_point_pdf");

    last_stat_count_reason_count_ = 0;
    last_stat_mean_z_reason_count_ = 0;
    last_stat_max_z_reason_count_ = 0;
    last_stat_min_z_reason_count_ = 0;
    last_stat_unknown_cell_reason_count_ = 0;
    last_foreground_score_max_ = 0.0;
    double foreground_positive_score_sum = 0.0;
    last_foreground_score_positive_cell_count_ = 0;

    const bool use_legacy = foreground_classifier_mode_ == "legacy_threshold";
    const bool use_statistical = foreground_classifier_mode_ == "statistical_score";
    const bool use_gdg_pdf = foreground_classifier_mode_ == "gdg_point_pdf";
    if (!use_legacy && !use_statistical && !use_gdg_pdf) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Unknown foreground_classifier_mode='%s'. Falling back to statistical_score.",
          foreground_classifier_mode_.c_str());
    }
    const bool is_legacy_mode = use_legacy;

    const auto frame_cells = buildFrameCells(points, valid_idx);
    const auto current_voxel_counts = countVoxelized(points, valid_idx);
    std::unordered_set<int64_t> raw_foreground_keys;

    if (enable_parallel_processing_) {
      std::vector<const std::pair<const int64_t, FrameCell>*> cell_ptrs;
      cell_ptrs.reserve(frame_cells.size());
      for (const auto & kv : frame_cells) {
        cell_ptrs.push_back(&kv);
      }

      struct CellDecision {
        bool cell_has_raw_foreground = false;
        double similarity = 0.0;
        std::string reason;
        double cell_score = 0.0;
        bool cell_processed_by_gdg = false;
        bool count_diff_exceeded = false;
        std::vector<double> point_sims;
        std::vector<bool> point_is_fg;
        bool persistent_foreground = false;
      };

      const size_t num_cells = cell_ptrs.size();
      std::vector<CellDecision> decisions(num_cells);

      #pragma omp parallel for
      for (size_t i = 0; i < num_cells; ++i) {
        const auto & kv = *cell_ptrs[i];
        const auto & fc = kv.second;
        auto & dec = decisions[i];
        const auto bit = background_map_.find(kv.first);

        dec.cell_processed_by_gdg = false;
        if (use_gdg_pdf) {
          if (static_cast<int>(fc.indices.size()) < min_points_per_occupied_cell_) {
            dec.cell_processed_by_gdg = true;
          } else if (bit == background_map_.end() || !bit->second.valid) {
            dec.cell_has_raw_foreground = true;
            dec.reason = "STAT_SCORE:UNKNOWN_BACKGROUND_CELL";
            dec.cell_score = foreground_unknown_cell_score_;
            dec.cell_processed_by_gdg = true;
          } else {
            const auto & bg = bit->second;
            const auto vit = current_voxel_counts.find(kv.first);
            const int cur_vox = (vit != current_voxel_counts.end()) ? vit->second : 0;
            const double bg_std = std::max(foreground_min_bg_std_z_, bg.std_z);
            const double max_density = bg.max_density > 1e-9 ? bg.max_density : normalPdf(bg.mean_z, bg.mean_z, bg_std);
            const double pdf_threshold = foreground_pdf_density_threshold_ * max_density;
            dec.count_diff_exceeded = std::abs(cur_vox - bg.seen_count) >= foreground_voxel_diff_threshold_;

            if (dec.count_diff_exceeded) {
              dec.point_sims.resize(fc.indices.size());
              dec.point_is_fg.resize(fc.indices.size());
              int cell_fg_count = 0;
              for (size_t pi = 0; pi < fc.indices.size(); ++pi) {
                int idx = fc.indices[pi];
                const auto & p = points[idx];
                const double pdf = normalPdf(p.z, bg.mean_z, bg_std);
                dec.point_sims[pi] = clamp(pdf / std::max(1e-6, max_density), 0.0, 1.0);
                if (pdf < pdf_threshold) {
                  dec.point_is_fg[pi] = true;
                  cell_fg_count++;
                } else {
                  dec.point_is_fg[pi] = false;
                }
              }
              if (cell_fg_count > 0) {
                dec.cell_has_raw_foreground = true;
                dec.reason = "GDG_PDF:HEIGHT_OUTLIER";
                dec.cell_score = 1.0;
              }
            }
            dec.cell_processed_by_gdg = true;
          }
        }

        if (!dec.cell_processed_by_gdg) {
          if (static_cast<int>(fc.indices.size()) < min_points_per_occupied_cell_) {
            dec.similarity = 1.0;
          } else if (bit != background_map_.end() && bit->second.valid) {
            const auto & bg = bit->second;
            const auto vit = current_voxel_counts.find(kv.first);
            const int cur_vox = (vit != current_voxel_counts.end()) ? vit->second : 0;
            if (is_legacy_mode) {
              const bool count_diff =
                  std::abs(cur_vox - bg.seen_count) >= foreground_count_diff_threshold_;
              const bool height_rise = fc.max_z > bg.max_z + background_height_diff_threshold_;
              const bool height_drop = fc.min_z < bg.min_z - background_height_diff_threshold_;
              if (count_diff || height_rise || height_drop) {
                dec.cell_has_raw_foreground = true;
                dec.similarity = 0.0;
              } else {
                dec.similarity = 1.0;
              }
            } else {
              const auto score = computeForegroundScore(bg, fc, cur_vox);
              dec.cell_has_raw_foreground = score.total >= foreground_score_threshold_;
              dec.similarity = 1.0 / (1.0 + score.total);
              dec.reason = score.reason;
              dec.cell_score = score.total;
            }
          } else {
            if (is_legacy_mode) {
              dec.cell_has_raw_foreground = true;
            } else {
              dec.cell_has_raw_foreground =
                  foreground_unknown_cell_score_ >= foreground_score_threshold_;
              dec.similarity = 0.0;
              dec.reason = "STAT_SCORE:UNKNOWN_BACKGROUND_CELL";
              dec.cell_score = foreground_unknown_cell_score_;
            }
          }

          const auto pit = foreground_cell_persistence_.find(kv.first);
          const int prev_persistence = (pit != foreground_cell_persistence_.end()) ? pit->second : 0;
          const int persistence = dec.cell_has_raw_foreground ? (prev_persistence + 1) : 0;
          dec.persistent_foreground =
              dec.cell_has_raw_foreground && persistence >= std::max(1, foreground_min_persistence_frames_);
        }
      }

      for (size_t i = 0; i < num_cells; ++i) {
        const auto & kv = *cell_ptrs[i];
        const auto & fc = kv.second;
        const auto & dec = decisions[i];

        if (use_gdg_pdf) {
          if (static_cast<int>(fc.indices.size()) < min_points_per_occupied_cell_) {
            for (int idx : fc.indices) {
              last_point_bg_similarity_[idx] = 1.0;
              if (keep_bg) background_idx.push_back(idx);
            }
          } else if (background_map_.find(kv.first) == background_map_.end() || !background_map_[kv.first].valid) {
            last_gdg_pdf_unknown_cell_point_count_ += static_cast<int>(fc.indices.size());
            for (int idx : fc.indices) {
              last_point_bg_similarity_[idx] = 0.0;
              foreground_idx.push_back(idx);
            }
          } else {
            if (!dec.count_diff_exceeded) {
              last_gdg_pdf_count_diff_rejected_cell_count_++;
              last_gdg_pdf_background_point_count_ += static_cast<int>(fc.indices.size());
              for (int idx : fc.indices) {
                last_point_bg_similarity_[idx] = 1.0;
                if (keep_bg) background_idx.push_back(idx);
              }
            } else {
              last_gdg_pdf_tested_cell_count_++;
              last_gdg_pdf_tested_point_count_ += static_cast<int>(fc.indices.size());
              for (size_t pi = 0; pi < fc.indices.size(); ++pi) {
                int idx = fc.indices[pi];
                last_point_bg_similarity_[idx] = dec.point_sims[pi];
                if (dec.point_is_fg[pi]) {
                  foreground_idx.push_back(idx);
                  last_gdg_pdf_foreground_point_count_++;
                } else {
                  if (keep_bg) background_idx.push_back(idx);
                  last_gdg_pdf_background_point_count_++;
                }
              }
              if (dec.cell_has_raw_foreground) {
                last_gdg_pdf_foreground_cell_count_++;
              }
            }
          }
        } else {
          if (dec.cell_has_raw_foreground && !is_legacy_mode) {
            if (dec.reason == "STAT_SCORE:COUNT_CHANGE") last_stat_count_reason_count_++;
            else if (dec.reason == "STAT_SCORE:MEAN_Z_CHANGE") last_stat_mean_z_reason_count_++;
            else if (dec.reason == "STAT_SCORE:MAX_Z_RISE") last_stat_max_z_reason_count_++;
            else if (dec.reason == "STAT_SCORE:MIN_Z_DROP") last_stat_min_z_reason_count_++;
            else if (dec.reason == "STAT_SCORE:UNKNOWN_BACKGROUND_CELL") last_stat_unknown_cell_reason_count_++;

            last_foreground_score_max_ = std::max(last_foreground_score_max_, dec.cell_score);
            foreground_positive_score_sum += dec.cell_score;
            last_foreground_score_positive_cell_count_++;
          }

          const int persistence = dec.cell_has_raw_foreground ? (foreground_cell_persistence_[kv.first] + 1) : 0;
          if (dec.cell_has_raw_foreground) foreground_cell_persistence_[kv.first] = persistence;
          else foreground_cell_persistence_.erase(kv.first);

          for (int idx : fc.indices) {
            if (dec.persistent_foreground) {
              foreground_idx.push_back(idx);
              last_point_bg_similarity_[idx] = dec.similarity;
            } else if (dec.cell_has_raw_foreground) {
              last_point_bg_similarity_[idx] = dec.similarity;
            } else {
              if (keep_bg) background_idx.push_back(idx);
              last_point_bg_similarity_[idx] = dec.similarity;
            }
          }
        }

        if (dec.cell_has_raw_foreground) {
          raw_foreground_keys.insert(kv.first);
        }
      }
    } else {
      for (const auto & kv : frame_cells) {
        const auto bit = background_map_.find(kv.first);
        const auto & fc = kv.second;
        bool cell_has_raw_foreground = false;
        double similarity = 0.0;
        std::string reason;
        double cell_score = 0.0;

        bool cell_processed_by_gdg = false;
        if (use_gdg_pdf) {
          if (static_cast<int>(fc.indices.size()) < min_points_per_occupied_cell_) {
            for (int idx : fc.indices) {
              last_point_bg_similarity_[idx] = 1.0;
              if (keep_bg) background_idx.push_back(idx);
            }
            cell_processed_by_gdg = true;
          } else if (bit == background_map_.end() || !bit->second.valid) {
            cell_has_raw_foreground = true;
            reason = "STAT_SCORE:UNKNOWN_BACKGROUND_CELL";
            cell_score = foreground_unknown_cell_score_;
            last_gdg_pdf_unknown_cell_point_count_ += static_cast<int>(fc.indices.size());
            for (int idx : fc.indices) {
              last_point_bg_similarity_[idx] = 0.0;
              foreground_idx.push_back(idx);
            }
            cell_processed_by_gdg = true;
          } else {
            const auto & bg = bit->second;
            const auto vit = current_voxel_counts.find(kv.first);
            const int cur_vox = (vit != current_voxel_counts.end()) ? vit->second : 0;
            const double bg_std = std::max(foreground_min_bg_std_z_, bg.std_z);
            
            const double max_density = bg.max_density > 1e-9 ? bg.max_density : normalPdf(bg.mean_z, bg.mean_z, bg_std);
            const double pdf_threshold = foreground_pdf_density_threshold_ * max_density;
            const bool count_diff_exceeded = std::abs(cur_vox - bg.seen_count) >= foreground_voxel_diff_threshold_;

            if (!count_diff_exceeded) {
              last_gdg_pdf_count_diff_rejected_cell_count_++;
              last_gdg_pdf_background_point_count_ += static_cast<int>(fc.indices.size());
              for (int idx : fc.indices) {
                last_point_bg_similarity_[idx] = 1.0;
                if (keep_bg) background_idx.push_back(idx);
              }
            } else {
              last_gdg_pdf_tested_cell_count_++;
              last_gdg_pdf_tested_point_count_ += static_cast<int>(fc.indices.size());
              int cell_fg_count = 0;
              for (int idx : fc.indices) {
                const auto & p = points[idx];
                const double pdf = normalPdf(p.z, bg.mean_z, bg_std);
                const double point_sim = clamp(pdf / std::max(1e-6, max_density), 0.0, 1.0);
                last_point_bg_similarity_[idx] = point_sim;

                if (pdf < pdf_threshold) {
                  foreground_idx.push_back(idx);
                  cell_fg_count++;
                  last_gdg_pdf_foreground_point_count_++;
                } else {
                  if (keep_bg) background_idx.push_back(idx);
                  last_gdg_pdf_background_point_count_++;
                }
              }
              if (cell_fg_count > 0) {
                cell_has_raw_foreground = true;
                reason = "GDG_PDF:HEIGHT_OUTLIER";
                cell_score = 1.0;
                last_gdg_pdf_foreground_cell_count_++;
              }
            }
            cell_processed_by_gdg = true;
          }
        }

        if (!cell_processed_by_gdg) {
          if (static_cast<int>(fc.indices.size()) < min_points_per_occupied_cell_) {
            similarity = 1.0;
          } else if (bit != background_map_.end() && bit->second.valid) {
            const auto & bg = bit->second;
            const auto vit = current_voxel_counts.find(kv.first);
            const int cur_vox = (vit != current_voxel_counts.end()) ? vit->second : 0;
            if (is_legacy_mode) {
              const bool count_diff =
                  std::abs(cur_vox - bg.seen_count) >= foreground_count_diff_threshold_;
              const bool height_rise = fc.max_z > bg.max_z + background_height_diff_threshold_;
              const bool height_drop = fc.min_z < bg.min_z - background_height_diff_threshold_;
              if (count_diff || height_rise || height_drop) {
                cell_has_raw_foreground = true;
                similarity = 0.0;
              } else {
                similarity = 1.0;
              }
            } else {
              const auto score = computeForegroundScore(bg, fc, cur_vox);
              cell_has_raw_foreground = score.total >= foreground_score_threshold_;
              similarity = 1.0 / (1.0 + score.total);
              reason = score.reason;
              cell_score = score.total;
            }
          } else {
            if (is_legacy_mode) {
              cell_has_raw_foreground = true;
            } else {
              cell_has_raw_foreground =
                  foreground_unknown_cell_score_ >= foreground_score_threshold_;
              similarity = 0.0;
              reason = "STAT_SCORE:UNKNOWN_BACKGROUND_CELL";
              cell_score = foreground_unknown_cell_score_;
            }
          }

          if (cell_has_raw_foreground && !is_legacy_mode) {
            if (reason == "STAT_SCORE:COUNT_CHANGE") last_stat_count_reason_count_++;
            else if (reason == "STAT_SCORE:MEAN_Z_CHANGE") last_stat_mean_z_reason_count_++;
            else if (reason == "STAT_SCORE:MAX_Z_RISE") last_stat_max_z_reason_count_++;
            else if (reason == "STAT_SCORE:MIN_Z_DROP") last_stat_min_z_reason_count_++;
            else if (reason == "STAT_SCORE:UNKNOWN_BACKGROUND_CELL") last_stat_unknown_cell_reason_count_++;

            last_foreground_score_max_ = std::max(last_foreground_score_max_, cell_score);
            foreground_positive_score_sum += cell_score;
            last_foreground_score_positive_cell_count_++;
          }
          if (cell_has_raw_foreground) raw_foreground_keys.insert(kv.first);

          const int persistence = cell_has_raw_foreground ? (foreground_cell_persistence_[kv.first] + 1) : 0;
          if (cell_has_raw_foreground) foreground_cell_persistence_[kv.first] = persistence;
          else foreground_cell_persistence_.erase(kv.first);
          const bool persistent_foreground =
              cell_has_raw_foreground && persistence >= std::max(1, foreground_min_persistence_frames_);

          for (int idx : fc.indices) {
            if (persistent_foreground) {
              foreground_idx.push_back(idx);
              last_point_bg_similarity_[idx] = similarity;
            } else if (cell_has_raw_foreground) {
              last_point_bg_similarity_[idx] = similarity;
            } else {
              if (keep_bg) background_idx.push_back(idx);
              last_point_bg_similarity_[idx] = similarity;
            }
          }
        } else {
          if (cell_has_raw_foreground) {
            if (reason == "STAT_SCORE:UNKNOWN_BACKGROUND_CELL") {
              last_stat_unknown_cell_reason_count_++;
            }
            last_foreground_score_max_ = std::max(last_foreground_score_max_, cell_score);
            foreground_positive_score_sum += cell_score;
            last_foreground_score_positive_cell_count_++;
            raw_foreground_keys.insert(kv.first);
          }

          if (cell_has_raw_foreground) {
            foreground_cell_persistence_[kv.first] = 1;
          } else {
            foreground_cell_persistence_.erase(kv.first);
          }
        }
      }
    }
    for (auto it = foreground_cell_persistence_.begin(); it != foreground_cell_persistence_.end();) {
      if (raw_foreground_keys.count(it->first) == 0) it = foreground_cell_persistence_.erase(it);
      else ++it;
    }
    last_foreground_count_ = static_cast<int>(foreground_idx.size());
    // Use true counts for diagnostics if possible, else approximate or use actual vector size.
    last_static_count_ = static_cast<int>(background_idx.size());
    last_foreground_positive_score_mean_ = last_foreground_score_positive_cell_count_ > 0 ? (foreground_positive_score_sum / last_foreground_score_positive_cell_count_) : 0.0;
  }

  void radiusOutlierRemoval(const std::vector<PointXYZIRT> & points, const std::vector<int> & input,
                            std::vector<int> & kept, std::vector<int> & rejected) const {
    if (input.empty()) return;
    const double radius = std::max(0.05, ror_radius_);
    const double radius2 = radius * radius;
    std::unordered_map<int64_t, std::vector<int>> grid;
    grid.reserve(input.size());
    for (int idx : input) {
      const int ix = static_cast<int>(std::floor(points[idx].x / radius));
      const int iy = static_cast<int>(std::floor(points[idx].y / radius));
      grid[hash2d(ix, iy)].push_back(idx);
    }

    for (int idx : input) {
      const int ix = static_cast<int>(std::floor(points[idx].x / radius));
      const int iy = static_cast<int>(std::floor(points[idx].y / radius));
      int neighbors = 0;
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          const auto it = grid.find(hash2d(ix + dx, iy + dy));
          if (it == grid.end()) continue;
          for (int nb : it->second) {
            if (nb == idx) continue;
            const double px = points[idx].x - points[nb].x;
            const double py = points[idx].y - points[nb].y;
            const double pz = points[idx].z - points[nb].z;
            if (px * px + py * py + 0.25 * pz * pz <= radius2) neighbors++;
          }
        }
      }
      if (neighbors >= ror_min_neighbors_) kept.push_back(idx);
      else rejected.push_back(idx);
    }
  }

  // 논문 Phase-1: 3D voxelization 후 bg_cell별 고유 voxel 수를 반환
  // voxelSize를 사용해 (floor(x/vs), floor(y/vs), floor(z/vs))로 voxel 구분,
  // 동일 bg_cell 내 중복 voxel을 제거한 voxel 수를 반환합니다.
  std::unordered_map<int64_t, int> countVoxelized(
      const std::vector<PointXYZIRT> & points,
      const std::vector<int> & indices) const {
    const double vs = std::max(0.02, voxel_size_);
    std::unordered_set<int64_t> seen_voxels;
    seen_voxels.reserve(indices.size());
    std::unordered_map<int64_t, int> cell_voxel_count;
    cell_voxel_count.reserve(indices.size() / 4 + 1);
    for (int idx : indices) {
      const auto & p = points[idx];
      const int vx = static_cast<int>(std::floor(p.x / vs));
      const int vy = static_cast<int>(std::floor(p.y / vs));
      const int vz = static_cast<int>(std::floor(p.z / vs));
      const int64_t vk = hash3d(vx, vy, vz);
      if (seen_voxels.insert(vk).second) {
        // 새 voxel: 해당 bg_cell의 카운트 증가
        cell_voxel_count[bgKey(p)]++;
      }
    }
    return cell_voxel_count;
  }

  void accumulateBackground(const std::vector<PointXYZIRT> & points, const std::vector<int> & indices,
                            double stamp) {
    const auto frame_cells = buildFrameCells(points, indices);
    // 논문 Phase-1: 이번 프레임의 voxelized count 계산
    const auto voxel_counts = countVoxelized(points, indices);
    for (const auto & kv : frame_cells) {
      auto & c = learning_accum_[kv.first];
      c.seen_frames++;
      c.point_count_sum += static_cast<int>(kv.second.indices.size());
      // voxelized count 누적 (논문의 num_points에 해당)
      const auto vit = voxel_counts.find(kv.first);
      c.voxel_count_sum += (vit != voxel_counts.end()) ? vit->second : 0;
      c.z_sum += kv.second.z_sum;
      for (int idx : kv.second.indices) {
        c.z2_sum += static_cast<double>(points[idx].z) * points[idx].z;
      }
      c.min_z = std::min(c.min_z, kv.second.min_z);
      c.max_z = std::max(c.max_z, kv.second.max_z);
    }
    learned_frame_count_++;
    if (learning_start_time_ <= 0.0) learning_start_time_ = stamp;
  }

  bool learningComplete(double stamp) const {
    const bool frame_done = learned_frame_count_ >= background_learning_frames_;
    const bool time_done = learning_start_time_ > 0.0 &&
                           (stamp - learning_start_time_) >= background_learning_duration_sec_;
    return frame_done || time_done;
  }

  void finalizeBackground(const std::string & frame_id) {
    background_map_.clear();
    background_frame_id_ = frame_id;
    for (const auto & kv : learning_accum_) {
      const auto & a = kv.second;
      const int total_points = std::max(1, a.point_count_sum);
      const double mean_z = a.z_sum / total_points;
      const double var = std::max(0.0, a.z2_sum / total_points - mean_z * mean_z);
      BackgroundCell bg;
      bg.ix = unpackX(kv.first);
      bg.iy = unpackY(kv.first);
      bg.center_x = (bg.ix + 0.5) * bg_x_res_;
      bg.center_y = (bg.iy + 0.5) * bg_y_res_;
      bg.occupancy_probability =
          clamp(static_cast<double>(a.seen_frames) / std::max(1, learned_frame_count_), 0.0, 1.0);
      // 논문 방식: seen_count = 프레임당 평균 voxelized count
      // voxel_count_sum이 있으면 사용, 없으면 기존 raw count 사용
      const int avg_raw = static_cast<int>(std::round(
          static_cast<double>(a.point_count_sum) / std::max(1, a.seen_frames)));
      const int avg_vox = (a.voxel_count_sum > 0)
          ? static_cast<int>(std::round(
                static_cast<double>(a.voxel_count_sum) / std::max(1, a.seen_frames)))
          : avg_raw;
      bg.seen_count = avg_vox;
      bg.ground_z = a.min_z;
      bg.mean_z = mean_z;
      bg.std_z = std::max(gdg_min_std_z_, std::sqrt(var));
      bg.min_z = a.min_z;
      bg.max_z = a.max_z;
      bg.max_density = normalPdf(bg.mean_z, bg.mean_z, bg.std_z);
      bg.static_score = bg.occupancy_probability;
      bg.last_seen_time = now().seconds();
      bg.valid = bg.occupancy_probability >= background_occupancy_threshold_ &&
                 bg.seen_count >= min_points_per_background_cell_;
      if (bg.valid) background_map_[kv.first] = bg;
    }
    learning_active_ = false;
    background_ready_ = learned_frame_count_ > 0;
    learning_accum_.clear();
    RCLCPP_INFO(get_logger(), "Static background learned: cells=%zu frame=%s",
                background_map_.size(), background_frame_id_.c_str());
    if (auto_save_background_after_learning_) {
      std::string err;
      if (!saveBackground(err)) RCLCPP_WARN(get_logger(), "Auto-save background failed: %s", err.c_str());
    }
  }

  std::vector<std::vector<int>> bevClusters(const std::vector<PointXYZIRT> & points,
                                            const std::vector<int> & indices) {
    std::unordered_map<int64_t, std::vector<int>> cells;
    for (int idx : indices) cells[bevKey(points[idx])].push_back(idx);
    std::unordered_set<int64_t> visited;
    std::vector<std::vector<int>> clusters;
    for (const auto & kv : cells) {
      if (visited.count(kv.first)) continue;
      std::vector<int64_t> q{kv.first};
      visited.insert(kv.first);
      std::vector<int> cluster_indices;
      int cell_count = 0;
      for (size_t head = 0; head < q.size(); ++head) {
        const int64_t key = q[head];
        const auto cit = cells.find(key);
        if (cit == cells.end()) continue;
        cluster_indices.insert(cluster_indices.end(), cit->second.begin(), cit->second.end());
        cell_count++;
        const int ix = unpackX(key);
        const int iy = unpackY(key);
        for (int dx = -1; dx <= 1; ++dx) {
          for (int dy = -1; dy <= 1; ++dy) {
            if (cluster_connectivity_ == 4 && std::abs(dx) + std::abs(dy) > 1) continue;
            if (dx == 0 && dy == 0) continue;
            const int64_t nk = hash2d(ix + dx, iy + dy);
            if (cells.count(nk) && !visited.count(nk)) {
              visited.insert(nk);
              q.push_back(nk);
            }
          }
        }
      }
      if (static_cast<int>(cluster_indices.size()) < minClusterPoints(points, cluster_indices)) continue;
      if (cell_count < minClusterCells(points, cluster_indices)) continue;
      clusters.push_back(cluster_indices);
    }
    last_cluster_count_ = static_cast<int>(clusters.size());
    return clusters;
  }

  int minClusterPoints(const std::vector<PointXYZIRT> & points, const std::vector<int> & c) const {
    double cx = 0.0, cy = 0.0;
    for (int idx : c) {
      cx += points[idx].x;
      cy += points[idx].y;
    }
    const double r = c.empty() ? 0.0 : sensorRange(cx / c.size(), cy / c.size());
    if (r < near_range_max_) return min_cluster_points_near_;
    if (r < mid_range_max_) return min_cluster_points_mid_;
    return min_cluster_points_long_;
  }

  int minClusterCells(const std::vector<PointXYZIRT> & points, const std::vector<int> & c) const {
    double cx = 0.0, cy = 0.0;
    for (int idx : c) {
      cx += points[idx].x;
      cy += points[idx].y;
    }
    const double r = c.empty() ? 0.0 : sensorRange(cx / c.size(), cy / c.size());
    if (r < near_range_max_) return min_cluster_cells_near_;
    if (r < mid_range_max_) return min_cluster_cells_mid_;
    return min_cluster_cells_long_;
  }

