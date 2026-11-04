#pragma once
// Private Cargo consumer. The General BEV implementation remains the oracle.
// Only storage/allocation and component membership bookkeeping differ; the
// integer hash keys, input/order, neighbours, filtering and arithmetic are kept.
// This object is owned by one Node instance and used by its existing worker.
// Output vectors own their data; no pool allocation escapes a call.
class CargoExtensionBevScratch {
  class Upstream final:public std::pmr::memory_resource{
   public:size_t current=0,peak=0,calls=0;
   private:void*do_allocate(size_t n,size_t a)override{void*p=std::pmr::new_delete_resource()->allocate(n,a);current+=n;peak=std::max(peak,current);++calls;return p;}
   void do_deallocate(void*p,size_t n,size_t a)override{assert(current>=n);current-=n;std::pmr::new_delete_resource()->deallocate(p,n,a);}
   bool do_is_equal(const std::pmr::memory_resource&o)const noexcept override{return this==&o;}
  };

  // Call-local monotonic allocation over instance-owned 64KiB blocks. Large
  // bucket arrays remain ordinary upstream allocations and are freed normally;
  // variable large requests therefore cannot accumulate obsolete large blocks.
  class FrameArena final:public std::pmr::memory_resource{
    static constexpr size_t block_size=64*1024;
    struct Block{void*data;size_t used;};
    std::pmr::memory_resource*upstream_;
    std::vector<Block>blocks_;size_t current_=0;
    void*do_allocate(size_t n,size_t alignment)override{
      if(n>block_size || alignment>alignof(std::max_align_t))return upstream_->allocate(n,alignment);
      const size_t bytes=std::max(size_t(1),n);
      while(current_<blocks_.size()){
        auto&b=blocks_[current_];uintptr_t address=reinterpret_cast<uintptr_t>(b.data)+b.used;
        const size_t padding=(alignment-(address&(alignment-1)))&(alignment-1);
        if(bytes+padding<=block_size-b.used){b.used+=padding+bytes;return reinterpret_cast<void*>(address+padding);}
        ++current_;
      }
      void*p=upstream_->allocate(block_size,alignof(std::max_align_t));
      try{blocks_.push_back({p,bytes});}catch(...){upstream_->deallocate(p,block_size,alignof(std::max_align_t));throw;}
      return p;
    }
    void do_deallocate(void*p,size_t n,size_t alignment)override{
      if(n>block_size || alignment>alignof(std::max_align_t))upstream_->deallocate(p,n,alignment);
    }
    bool do_is_equal(const std::pmr::memory_resource&o)const noexcept override{return this==&o;}
   public:explicit FrameArena(std::pmr::memory_resource*r):upstream_(r){}
    ~FrameArena(){release();}
    FrameArena(const FrameArena&)=delete;FrameArena&operator=(const FrameArena&)=delete;
    void rewind()noexcept{for(auto&b:blocks_)b.used=0;current_=0;}
    void release()noexcept{for(auto&b:blocks_)upstream_->deallocate(b.data,block_size,alignof(std::max_align_t));std::vector<Block>().swap(blocks_);current_=0;}
  };

  struct Cell{
    std::pmr::vector<int>indices;int ix=0,iy=0,iz=0;bool visited=false;
    float min_z=std::numeric_limits<float>::max(),max_z=std::numeric_limits<float>::lowest();
    explicit Cell(std::pmr::memory_resource*r):indices(r){}
  };
  struct XYInfo{int count=0;uint64_t component=0;};
  Upstream upstream_;
  std::unique_ptr<FrameArena>pool_;
  std::vector<int64_t>queue_;
  std::vector<int>component_cells_,component_points_;
  bool tag_membership_,reuse_buffers_;
  size_t max_foreground_=0,max_cells_=0;
 public:
  CargoExtensionBevScratch(bool pooled,bool tags,bool reuse):tag_membership_(tags),reuse_buffers_(reuse){
    if(pooled)pool_=std::make_unique<FrameArena>(&upstream_);
  }
  CargoExtensionBevScratch(const CargoExtensionBevScratch&)=delete;
  CargoExtensionBevScratch&operator=(const CargoExtensionBevScratch&)=delete;
  void reset(){
    if(pool_)pool_->release();
    std::vector<int64_t>().swap(queue_);std::vector<int>().swap(component_cells_);std::vector<int>().swap(component_points_);
    max_foreground_=max_cells_=0;
  }
  // retained upstream bytes, high-water upstream bytes, upstream heap calls,
  // queue capacity, component-count capacity, actual maximum foreground/cells.
  std::array<size_t,7>memoryStats()const{return {upstream_.current,upstream_.peak,upstream_.calls,queue_.capacity(),component_cells_.capacity()+component_points_.capacity(),max_foreground_,max_cells_};}
  std::vector<ClusterResult>cluster(BevConnectedComponentClusterer&cfg,
      const std::vector<PointXYZIRT>&points,const std::vector<int>&foreground_indices,
      uint64_t frame_index,double timestamp,bool include_weak=false){
    queue_.clear();component_cells_.clear();component_points_.clear();
    if(!cfg.z_layer_enabled_)return cfg.cluster(points,foreground_indices,frame_index,timestamp);
    auto*memory=pool_?static_cast<std::pmr::memory_resource*>(pool_.get()):&upstream_;
    // Declared before every arena-backed container: their destructors run first
    // on normal return and exception paths. Returned vectors use std::allocator.
    struct Epoch{FrameArena*arena;~Epoch(){if(arena)arena->rewind();}}epoch{pool_.get()};

    cfg.diagnostics_ = {};
    const auto layer_build_start = std::chrono::steady_clock::now();
    std::pmr::unordered_map<int64_t, Cell> layers(memory);
    std::pmr::unordered_map<int64_t, XYInfo> layers_per_xy(memory);
    layers.reserve(foreground_indices.size() / 2 + 1);
    max_foreground_=std::max(max_foreground_,foreground_indices.size());
    const double z_res = std::max(0.05, cfg.z_layer_resolution_);
    for (int idx : foreground_indices) {
      const auto & p = points[idx];
      const int ix = static_cast<int>(std::floor(p.x / std::max(0.05, cfg.bev_x_res_)));
      const int iy = static_cast<int>(std::floor(p.y / std::max(0.05, cfg.bev_y_res_)));
      const int iz = static_cast<int>(std::floor(p.z / z_res));
      const int64_t key = hash3d(ix, iy, iz);
      auto [it, inserted] = layers.try_emplace(key,memory);
      auto & cell = it->second;
      if (inserted) {
        if(cfg.initial_layer_point_capacity_>0)cell.indices.reserve(cfg.initial_layer_point_capacity_);
        cell.ix = ix; cell.iy = iy; cell.iz = iz;
        ++layers_per_xy[hash2d(ix, iy)].count;
      }
      cell.indices.push_back(idx);
      cell.min_z = std::min(cell.min_z, p.z);
      cell.max_z = std::max(cell.max_z, p.z);
    }
    max_cells_=std::max(max_cells_,layers.size());
    cfg.diagnostics_.z_layer_build_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - layer_build_start).count();
    cfg.diagnostics_.occupied_cell_count = static_cast<int>(layers_per_xy.size());
    cfg.diagnostics_.layer_cell_count = static_cast<int>(layers.size());
    for (const auto & kv : layers_per_xy) {
      cfg.diagnostics_.max_layers_per_xy_cell = std::max(cfg.diagnostics_.max_layers_per_xy_cell, kv.second.count);
      cfg.diagnostics_.average_layers_per_xy_cell += kv.second.count;
    }
    if (!layers_per_xy.empty()) cfg.diagnostics_.average_layers_per_xy_cell /= layers_per_xy.size();
    const auto strong = [&](const Cell & cell) {
      return include_weak || static_cast<int>(cell.indices.size()) >= cfg.min_points_per_occupied_cell_;
    };
    for (const auto & kv : layers) {
      if (strong(kv.second)) ++cfg.diagnostics_.strong_layer_cell_count;
      else ++cfg.diagnostics_.weak_layer_cell_count;
    }

    std::pmr::unordered_map<int64_t, int> local_density(memory);
    if (cfg.density_enabled_) {
      const auto density_start = std::chrono::steady_clock::now();
      local_density.reserve(layers.size());
      for (const auto & kv : layers) {
        const auto & cell = kv.second;
        int total = 0;
        for (int dx = -cfg.density_window_cells_; dx <= cfg.density_window_cells_; ++dx) {
          for (int dy = -cfg.density_window_cells_; dy <= cfg.density_window_cells_; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
              const auto it = layers.find(hash3d(cell.ix + dx, cell.iy + dy, cell.iz + dz));
              if (it != layers.end()) total += static_cast<int>(it->second.indices.size());
            }
          }
        }
        local_density.emplace(kv.first, total);
      }
      cfg.diagnostics_.density_build_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - density_start).count();
    }

    const auto compatible = [&](const Cell & a, const Cell & b, double max_gap) {
      return std::abs(a.iz - b.iz) <= 1 && (!cfg.z_guard_ || cfg.zCompatible(a, b, max_gap));
    };
    const auto densityEnough = [&](int64_t key) {
      return !cfg.density_enabled_ || local_density[key] >= cfg.weak_support_min_density_;
    };
    const auto addHistogram = [](int value, std::array<int, 6> & histogram) {
      const int bin = value <= 1 ? 0 : value == 2 ? 1 : value <= 5 ? 2 :
          value <= 10 ? 3 : value <= 30 ? 4 : 5;
      ++histogram[bin];
    };

    std::vector<int> local_component_cells,local_component_points;
    auto&raw_component_cells=reuse_buffers_?component_cells_:local_component_cells;
    auto&raw_component_points=reuse_buffers_?component_points_:local_component_points;
    raw_component_cells.clear();raw_component_points.clear();
    uint64_t component_token=0;
    std::vector<ClusterResult> clusters;
    int cid = 0;
    for (auto & root : layers) {
      if (!strong(root.second) || root.second.visited) continue;
      const auto bfs_start = std::chrono::steady_clock::now();
      ++component_token;
      std::vector<int64_t> local_queue;
      auto&queue=reuse_buffers_?queue_:local_queue;
      queue.clear();queue.push_back(root.first);
      root.second.visited = true;
      std::pmr::unordered_set<int64_t> component_xy(memory);
      ClusterResult result;
      result.method = cfg.name();
      double gather_ms = 0.0;
      for (size_t head = 0; head < queue.size(); ++head) {
        const int64_t key = queue[head];
        const auto cit = layers.find(key);
        if (cit == layers.end()) continue;
        const auto & cell = cit->second;
        const auto gather_start = std::chrono::steady_clock::now();
        result.indices.insert(result.indices.end(), cell.indices.begin(), cell.indices.end());
        gather_ms += std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - gather_start).count();
        const int64_t xy_key = hash2d(cell.ix, cell.iy);
        const bool first_xy=[&](){
          if(!tag_membership_)return component_xy.insert(xy_key).second;
          auto it=layers_per_xy.find(xy_key);
          // Every visited Cell was inserted with this exact XY key above.
          assert(it!=layers_per_xy.end());
          if(it->second.component==component_token)return false;
          it->second.component=component_token;return true;
        }();
        if(first_xy)result.cell_keys.push_back(xy_key);

        for (int dx = -1; dx <= 1; ++dx) {
          for (int dy = -1; dy <= 1; ++dy) {
            // Adjacent vertical layers in the same XY cell are part of the
            // same sparse BEV column. Keeping them disconnected caused a
            // single vertical surface to fragment into many layer components.
            const bool vertical_neighbor = dx == 0 && dy == 0;
            if (!vertical_neighbor && cfg.cluster_connectivity_ == 4 &&
                std::abs(dx) + std::abs(dy) > 1) continue;
            for (int dz = -1; dz <= 1; ++dz) {
              if (vertical_neighbor && dz == 0) continue;
              const int64_t nk = hash3d(cell.ix + dx, cell.iy + dy, cell.iz + dz);
              const auto nit = layers.find(nk);
              if (nit == layers.end() || !strong(nit->second) || nit->second.visited) continue;
              if (compatible(cell, nit->second, cfg.neighbor_z_gap_)) {
                nit->second.visited = true; queue.push_back(nk);
              } else if (cfg.z_guard_) {
                ++cfg.diagnostics_.z_rejected_direct_edges;
              }
            }
          }
        }

        // A weak layer cell is evidence only: it connects two strong endpoints but is
        // deliberately excluded from cluster points, density and cell statistics.
        if (cfg.weak_support_enabled_) {
          for (const auto & dir : std::array<std::pair<int, int>, 4>{{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}}) {
            for (int wdz = -1; wdz <= 1; ++wdz) {
              const int wx = cell.ix + dir.first, wy = cell.iy + dir.second, wz = cell.iz + wdz;
              const int64_t wk = hash3d(wx, wy, wz);
              const auto wit = layers.find(wk);
              if (wit == layers.end() || strong(wit->second) || !compatible(cell, wit->second, cfg.neighbor_z_gap_)) continue;
              for (int edz = -1; edz <= 1; ++edz) {
                const int64_t ek = hash3d(cell.ix + 2 * dir.first, cell.iy + 2 * dir.second, wz + edz);
                const auto eit = layers.find(ek);
                if (eit == layers.end() || !strong(eit->second) || eit->second.visited ||
                    !compatible(wit->second, eit->second, cfg.neighbor_z_gap_)) continue;
                ++cfg.diagnostics_.weak_support_candidates;
                if (!densityEnough(key) || !densityEnough(ek)) {
                  ++cfg.diagnostics_.weak_support_density_rejected;
                  continue;
                }
                eit->second.visited = true; queue.push_back(ek);
                ++cfg.diagnostics_.weak_support_accepted;
              }
            }
          }
        }

        // Preserve the existing empty-cell bridge for A/B comparison. A weak
        // cell is not empty, so it must use the evidence-based path above.
        if (cfg.gap_bridge_) {
          for (const auto & dir : std::array<std::pair<int, int>, 4>{{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}}) {
            const int mx = cell.ix + dir.first, my = cell.iy + dir.second;
            if (layers_per_xy.count(hash2d(mx, my)) || !cfg.gapAllowed(cell.ix, cell.iy, dir.first, dir.second)) continue;
            ++cfg.diagnostics_.bridge_candidates;
            for (int dz = -1; dz <= 1; ++dz) {
              const int64_t ek = hash3d(cell.ix + 2 * dir.first, cell.iy + 2 * dir.second, cell.iz + dz);
              const auto eit = layers.find(ek);
              if (eit == layers.end() || !strong(eit->second) || eit->second.visited) continue;
              if (compatible(cell, eit->second, cfg.bridge_z_gap_)) {
                eit->second.visited = true; queue.push_back(ek);
                ++cfg.diagnostics_.bridge_accepted;
              } else if (cfg.z_guard_) {
                ++cfg.diagnostics_.z_rejected_bridge_edges;
              }
            }
          }
        }
      }
      cfg.diagnostics_.cc_bfs_ms += std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - bfs_start).count() - gather_ms;
      cfg.diagnostics_.point_gather_ms += gather_ms;
      ++cfg.diagnostics_.connected_component_count;
      const int cells = static_cast<int>(result.cell_keys.size());
      const int point_count = static_cast<int>(result.indices.size());
      raw_component_cells.push_back(cells); raw_component_points.push_back(point_count);
      addHistogram(cells, cfg.diagnostics_.component_cell_histogram);
      addHistogram(point_count, cfg.diagnostics_.component_point_histogram);
      if (cells <= 2) {
        ++cfg.diagnostics_.tiny_component_count;
        if (cells == 1) ++cfg.diagnostics_.single_cell_component_count;
        if (cells == 2) ++cfg.diagnostics_.two_cell_component_count;
      }
      cfg.diagnostics_.largest_component_cells = std::max(cfg.diagnostics_.largest_component_cells, cells);
      cfg.diagnostics_.largest_component_points = std::max(cfg.diagnostics_.largest_component_points, point_count);
      const auto filter_start = std::chrono::steady_clock::now();
      const bool reject = !include_weak &&
          (point_count < cfg.minClusterPoints(points, result.indices) ||
           cells < cfg.minClusterCells(points, result.indices));
      cfg.diagnostics_.filter_ms += std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - filter_start).count();
      if (reject) continue;
      fillClusterStats(points, result);
      result.id = cid++;
      clusters.push_back(std::move(result));
    }
    cfg.diagnostics_.valid_cluster_count = static_cast<int>(clusters.size());
    const auto median = [](std::vector<int> & values) {
      if (values.empty()) return 0;
      const auto middle = values.begin() + values.size() / 2;
      std::nth_element(values.begin(), middle, values.end());
      return *middle;
    };
    cfg.diagnostics_.median_component_cells = median(raw_component_cells);
    cfg.diagnostics_.median_component_points = median(raw_component_points);
    queue_.clear();raw_component_cells.clear();raw_component_points.clear();
    return clusters;
  }
};
