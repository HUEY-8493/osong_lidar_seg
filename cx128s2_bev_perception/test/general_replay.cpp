// Extend the existing synchronous include-node harness; production algorithms stay in the node.
#define main perception_node_main
#include "../src/cx128s2_bev_perception_node.cpp"
#undef main
#include <rosbag2_cpp/reader.hpp>
#include <rclcpp/serialization.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <sys/resource.h>
#include <sys/stat.h>
#include "general_replay_cache.hpp"
using namespace replay_cache;
namespace replay_cache {
template<>struct Custom<tf2::Transform>{template<class A>static void visit(A&a,tf2::Transform&t){
  for(int i=0;i<3;++i)for(int j=0;j<3;++j)a(t.getBasis()[i][j]);
  for(int i=0;i<3;++i)a(t.getOrigin()[i]);
}};
template<>struct Custom<cx128s2_bev_perception::LinearKalmanFilter>{template<class A>static void visit(A&a,cx128s2_bev_perception::LinearKalmanFilter&k);};
}
struct TrackingRegressionAccess {
using Node=Cx128s2BevPerception;
using Frame=Node::OfflinePreparedFrame;
using Clock=std::chrono::steady_clock;
static double seconds(Clock::time_point t){return std::chrono::duration<double>(Clock::now()-t).count();}
template<class A>static void kf(A&a,cx128s2_bev_perception::LinearKalmanFilter&k){a(k.initialized_);a(k.x_);a(k.p_);a(k.innovation_);}
static std::string depth(Node&n){auto&m=n.visibility_model_;Writer w;
 w(m.config_);w(m.world_from_sensor_);w(m.sensor_from_world_);w(m.pose_valid_);
 w(m.az_bin_min_);w(m.el_bin_min_);w(m.az_bin_count_);w(m.el_bin_count_);w(m.occupied_depth_cells_);
 w(m.occupied_depth_indices_);for(auto ix:m.occupied_depth_indices_){w(ix);w(m.depth_.at(ix));}
 return sha(w.bytes.data(),w.bytes.size());
}
// This snapshot is strictly pre-remap. Every replay reconstructs an independent work grid.
static std::vector<char> depthSnapshot(Node&n){auto&m=n.visibility_model_;Writer w;
 w(digest(m.config_,m.world_from_sensor_,m.sensor_from_world_));
 w(m.az_bin_min_);w(m.el_bin_min_);w(m.az_bin_count_);w(m.el_bin_count_);
 w(m.occupied_depth_cells_);w(m.occupied_depth_indices_);
 for(auto ix:m.occupied_depth_indices_)w(m.depth_.at(ix));
 return w.bytes;
}
static void restoreDepth(Node&n,const std::vector<char>&b){auto&m=n.visibility_model_;Reader a{b};
 std::string signature;a(signature);if(signature!=digest(m.config_,m.world_from_sensor_,m.sensor_from_world_))throw std::runtime_error("Initial Depth pose/calibration mismatch");
 int azmin,elmin,azcount,elcount;a(azmin);a(elmin);a(azcount);a(elcount);
 if(azmin!=m.az_bin_min_||elmin!=m.el_bin_min_||azcount!=m.az_bin_count_||elcount!=m.el_bin_count_)throw std::runtime_error("Initial Depth bin mismatch");
 a(m.occupied_depth_cells_);a(m.occupied_depth_indices_);
 m.depth_.assign(m.depth_.size(),{});
 for(auto ix:m.occupied_depth_indices_){if(ix<0||size_t(ix)>=m.depth_.size())throw std::runtime_error("Invalid Depth index");a(m.depth_[ix]);}
 if(a.pos!=b.size())throw std::runtime_error("Trailing initial Depth bytes");
}
template<class T>static std::vector<uint8_t> cdr(const T&msg){rclcpp::SerializedMessage s;rclcpp::Serialization<T>().serialize_message(&msg,&s);auto&r=s.get_rcl_serialized_message();return {r.buffer,r.buffer+r.buffer_length};}
// The shared production suffix consumes World XYZIRT, never msg->data.
// Preserve every PointCloud2 metadata field; sensor-based queries use the exact saved TF
// and World points through the existing inverse transform, just as full replay does.
static std::vector<uint8_t> cloudMetadata(const sensor_msgs::msg::PointCloud2&m){
 sensor_msgs::msg::PointCloud2 v;v.header=m.header;v.height=m.height;v.width=m.width;
 v.fields=m.fields;v.is_bigendian=m.is_bigendian;v.point_step=m.point_step;
 v.row_step=m.row_step;v.is_dense=m.is_dense;return cdr(v);
}
template<class T>static T decode(const std::vector<uint8_t>&b){rclcpp::SerializedMessage s(b.size());auto&r=s.get_rcl_serialized_message();std::memcpy(r.buffer,b.data(),b.size());r.buffer_length=b.size();T v;rclcpp::Serialization<T>().deserialize_message(&s,&v);return v;}
static long rss(){std::ifstream s("/proc/self/status");std::string x;while(std::getline(s,x))if(x.rfind("VmRSS:",0)==0)return std::stol(x.substr(6));return 0;}
static Json bagIdentity(const fs::path&bag){Json files=Json::array();for(auto&e:fs::recursive_directory_iterator(bag)){if(!e.is_regular_file())continue;struct stat st{};if(stat(e.path().c_str(),&st))throw std::runtime_error("Bag stat failed");Json f={{"path",fs::canonical(e.path()).string()},{"size",st.st_size},{"mtime_sec",st.st_mtim.tv_sec},{"mtime_nsec",st.st_mtim.tv_nsec},{"inode",st.st_ino}};if(e.path().extension()!=".mcap")f["sha256"]=shaFile(e.path());files.push_back(f);}std::sort(files.begin(),files.end(),[](const Json&a,const Json&b){return a["path"]<b["path"];});return files;}
static Json contract(Node&n,const fs::path&bag,const fs::path&exe,const std::vector<uint64_t>&sequence,bool depth_cache,bool compressed,bool dataset){
 auto dependencies=readJson(exe.parent_path()/"runtime_dependencies.json");
 for(auto it=dependencies.begin();it!=dependencies.end();++it)if(shaFile(it.key())!=it.value())throw std::runtime_error("Stale executable/dependency; rebuild: "+it.key());
 for(const auto&path:std::vector<std::string>{n.background_map_file_,n.roi_policy_file_,
     n.get_parameter("visibility.driver_config_file").as_string().empty() ? ament_index_cpp::get_package_share_directory("lslidar_driver")+"/config/lslidar_ch.yaml" : n.get_parameter("visibility.driver_config_file").as_string()}) {
   if(!path.empty() && fs::is_regular_file(path)) dependencies[fs::canonical(path).string()]=shaFile(path);
 }
 Json params=Json::object();for(auto&key:n.list_parameters({},100).names){auto p=n.get_parameter(key);params[key]={{"type",int(p.get_type())},{"value",p.value_to_string()}};}
 if(dataset){
  auto prefix=readJson(exe.parent_path()/"prefix_dependencies.json");if(prefix.value("full_only",false))throw std::runtime_error("Full-only build cannot create/replay cached datasets; rebuild without --full-only");Json selected=Json::object();
  for(const auto&key:prefix.at("parameters"))if(params.contains(key.get<std::string>()))selected[key.get<std::string>()]=params.at(key.get<std::string>());
  Json runtime=Json::object();for(auto it=dependencies.begin();it!=dependencies.end();++it)
    if(it.key().rfind("/usr/",0)==0||it.key().rfind("/opt/ros/",0)==0)runtime[it.key()]=it.value();
  for(const auto&path:std::vector<std::string>{n.background_map_file_,n.roi_policy_file_, n.get_parameter("visibility.driver_config_file").as_string().empty()?ament_index_cpp::get_package_share_directory("lslidar_driver")+"/config/lslidar_ch.yaml":n.get_parameter("visibility.driver_config_file").as_string()})if(!path.empty())runtime[path]=shaFile(path);
  return {{"schema",4},{"dataset_version",1},{"compression",compressed?"lz4-lossless-default":"none"},{"depth_cache",depth_cache},{"prefix",prefix},{"parameters",selected},{"runtime",runtime},{"sequence",sequence},{"sizes",{{"double",sizeof(double)},{"float",sizeof(float)},{"size_t",sizeof(size_t)},{"feature",sizeof(ObjectFeature)}}}};
 }
 return {{"schema",4},{"compression",compressed?"lz4-lossless-default":"none"},{"depth_cache",depth_cache},{"contract","general-pre-tracking-fieldwise-v1"},{"bag",fs::canonical(bag).string()},{"bag_files",bagIdentity(bag)},{"dependencies",dependencies},{"parameters",params},{"sequence",sequence},{"mode",sequence.empty()?"all-inputs":"processed-sequence"},{"cloud_topic","/ch/lslidar_point_cloud"},{"tf_topics",Json::array({"/tf_static","/tf"})},{"float_comparison","bit-exact (zero tolerance), same binary/ABI"},{"sizes",{{"double",sizeof(double)},{"float",sizeof(float)},{"size_t",sizeof(size_t)},{"feature",sizeof(ObjectFeature)},{"track",sizeof(Track)}}}};
}
static void stop(Node&n){n.worker_running_=false;n.latest_msg_cv_.notify_all();if(n.worker_thread_.joinable())n.worker_thread_.join();}
static void supported(Node&n){if(n.perception_mode_=="crane_beam_work"||!n.usesStaticBackground()||n.learning_active_||!n.background_ready_||!n.clusterer_||n.clusterer_->name()!="bev_connected_component"||n.object_memory_bank_enabled_)throw std::runtime_error("Cache v1 requires General, frozen ready static background, BEV CC, memory bank OFF; use full otherwise");}
static void run(Node&n,const std::string&mode,const fs::path&bag,const fs::path&cache,const fs::path&prefix,const fs::path&exe,const std::vector<uint64_t>&sequence,bool depth_cache,bool fast,bool compressed,bool dataset,uint64_t stop_frame,uint64_t inspect_frame){
 stop(n);if(mode!="full")supported(n);
 const auto run_begin=Clock::now();auto expected=contract(n,bag,exe,sequence,depth_cache,compressed,dataset);
 const bool build=mode=="cache-build", cached=mode=="cache-replay";
 if(!build&&!cached&&mode!="full")throw std::runtime_error("Unsupported engine mode");
 Json manifest;std::ofstream cache_out,index_out;std::ifstream cache_in;
 if(build){if(fs::exists(cache))throw std::runtime_error("Cache destination exists; choose a new directory");fs::create_directories(cache);saveJson(cache/"incomplete.json",expected);cache_out.open(cache/"frames.bin",std::ios::binary);if(dataset)index_out.open(cache/"frames.index");}
 if(cached){manifest=readJson(cache/"manifest.json");if(manifest.at("contract")!=expected){saveJson(prefix.string()+"_expected_contract.json",expected);throw std::runtime_error("Preprocessing contract mismatch; inspect expected_contract.json versus manifest (source/parameter/schema); explicitly regenerate this dataset");}cache_in.open(cache/"frames.bin",std::ios::binary);if(!cache_in)throw std::runtime_error("Missing cache data");if(uint64_t(fs::file_size(cache/"frames.bin"))!=manifest.at("bytes").get<uint64_t>())throw std::runtime_error("Cache file size mismatch");}
 fs::create_directories(prefix.parent_path());std::ofstream trace(prefix.string()+"_trace.jsonl"),evidence(prefix.string()+"_evidence.jsonl"),times(prefix.string()+"_times.csv"),trackcsv(prefix.string()+"_tracks.csv");
 times<<"frame,input_sequence,stamp,reader_ms,deserialize_ms,cache_load_ms,pipeline_ms,result_ms,rss_kb,read_cloud_ms,crop_ms,roi_ms,fg_ms,ror_ms,cluster_ms,feature_ms,raw_box_ms,depth_ms,initial_visibility_ms,tracking_ms,memory_ms\n";
 trackcsv<<"frame,stamp,id,lost,public,vehicle,status,association,points,x,y,vx,vy\n";
 size_t count=0,input_count=0;int64_t first_stamp=0,last_stamp=0;double read_sec=0,deser_sec=0,load_sec=0,pipeline_sec=0,result_sec=0,write_sec=0;
 Json framehash;std::string initial_depth,initial_visibility,tf_digest;std::vector<std::string>tf_hashes;
 bool captured=false;Frame loaded;std::vector<char> initial_depth_bytes;
 n.offline_prefix_only_=build;
 n.offline_capture_=[&](const Frame&f){captured=true;if(!fast&&!build)framehash={{"input",digest(f.tf,f.output_frame,f.points,f.compact_to_original,f.has_ring,f.has_time,f.tf_fallback)},{"foreground",digest(f.valid,f.roi,f.ignored,f.hard_ignored,f.foreground_before_ror,f.foreground,f.similarity,f.background_ready,f.learning_active,f.enabled,f.publish_confirmed)},{"cluster",digest(f.clusters,f.owners)},{"feature_raw_box",digest(f.features)}};if(build){loaded=f;initial_depth_bytes=(depth_cache&&f.enabled&&n.visibility_frame_valid_)?depthSnapshot(n):std::vector<char>{};}};
 n.offline_restore_depth_=[&](){if(!depth_cache)return false;if(initial_depth_bytes.empty())throw std::runtime_error("Missing initial Depth snapshot");restoreDepth(n,initial_depth_bytes);return true;};
 n.offline_before_tracking_=[&](){if(!fast&&!build)initial_depth=depth(n);};
 n.offline_initial_visibility_=[&](const auto&features){if(!fast&&!build)initial_visibility=digest(features);};
 std::ofstream public_frames(prefix.string()+"_vehicles.jsonl");
 Json public_hash;std::string observed_hash;
 n.offline_after_frame_=[&](){if(fast||build)return;std::map<int,ObjectFeature> observed;for(const auto&kv:n.recording_track_features_)if(kv.second)observed.emplace(kv.first,*kv.second);observed_hash=digest(observed);};
 std::set<uint64_t> snapshot_frames;
 fs::path preview_dir;
 if(const char* value=std::getenv("GENERAL_REVIEW_PREVIEW_DIR")){preview_dir=value;fs::create_directories(preview_dir);}
 if(const char* value=std::getenv("GENERAL_REVIEW_PREVIEW_FRAMES")){
   std::string list=value;std::replace(list.begin(),list.end(),',',' ');std::istringstream input(list);uint64_t f;while(input>>f)snapshot_frames.insert(f);
 }
 n.offline_public_=[&](const auto&detected,const auto&tracked,const auto&motion,const auto&vehicles,const auto&identity){
   Json rows=Json::array();
   for(const auto&v:vehicles.objects){
     Json row={{"id",v.id},{"length",v.dimensions.x},{"width",v.dimensions.y},{"height",v.dimensions.z},
       {"x",v.position.x},{"y",v.position.y},{"z",v.position.z},{"distance",v.distance},{"status",v.vehicle_status},
       {"age",v.age},{"lost",v.lost_frames},{"points",v.point_count},{"observed_length",0},{"observed_width",0},{"observed_height",0},{"observation","HELD"}};
     const auto found=n.recording_track_features_.find(v.id);
     if(v.lost_frames==0 && found!=n.recording_track_features_.end()&&found->second){
       const auto&f=*found->second;row["observation"]="DIRECT";row["observed_length"]=f.obb.valid?f.obb.length:f.length;
       row["observed_width"]=f.obb.valid?f.obb.width:f.width;row["observed_height"]=f.height;row["observed_min_z"]=f.min_z;row["observed_max_z"]=f.max_z;
       row["observed_points"]=f.indices.size();row["raw_box_points"]=f.raw_box_point_count;
     }
     rows.push_back(row);
   }
   Json frame={{"frame",count},{"stamp_sec",vehicles.header.stamp.sec},{"stamp_nanosec",vehicles.header.stamp.nanosec},
     {"raw_points",n.frame_points_.size()},{"valid_points",n.last_valid_count_},{"ground_points",n.last_ground_count_},
     {"foreground_points",n.last_foreground_count_},{"noise_candidates",n.last_general_ground_noise_stats_.raw_candidates},{"noise_removed",n.last_general_ground_noise_stats_.removed_foreground},{"noise_ms",n.last_general_ground_noise_stats_.elapsed_ms},{"vehicles",rows}};
   public_frames<<frame.dump()<<'\n';
   if(!preview_dir.empty()&&snapshot_frames.count(count)){
     std::vector<int32_t> owner(n.frame_points_.size(),-1);
     for(size_t i=0;i<n.frame_points_.size();++i)if(n.frame_points_[i].z<n.valid_z_min_)owner[i]=-3;
     for(int i:n.general_ground_noise_removed_)if(i>=0&&size_t(i)<owner.size())owner[size_t(i)]=-4;
     for(const auto&v:vehicles.objects){const auto it=n.recording_track_features_.find(v.id);
       if(v.lost_frames==0&&it!=n.recording_track_features_.end()&&it->second)
         for(int i:it->second->indices)if(i>=0&&size_t(i)<owner.size())owner[size_t(i)]=v.id;
     }
     const std::string file="frame_"+std::to_string(count)+".bin";std::ofstream binary(preview_dir/file,std::ios::binary);
     for(size_t i=0;i<n.frame_points_.size();++i){const auto&p=n.frame_points_[i];float xyz[3]={p.x,p.y,p.z};binary.write(reinterpret_cast<const char*>(xyz),sizeof(xyz));binary.write(reinterpret_cast<const char*>(&owner[i]),sizeof(int32_t));}
     frame["raw_file"]=file;frame["valid_z_min"]=n.valid_z_min_;frame["frame_id"]=vehicles.header.frame_id;
     saveJson(preview_dir/("frame_"+std::to_string(count)+".json"),frame);
   }
   if(fast||build)return;
   public_hash={{"detected",digest(cdr(detected))},{"tracked",digest(cdr(tracked))},{"motion",digest(cdr(motion))},{"vehicles",digest(cdr(vehicles))},{"identity",digest(cdr(identity))}};
 };
 if(fast&&mode=="full")n.offline_capture_={};
 std::unique_ptr<rosbag2_cpp::Reader>reader;
 if(!cached){reader=std::make_unique<rosbag2_cpp::Reader>();reader->open(bag.string());rosbag2_storage::StorageFilter filter;filter.topics={"/tf_static","/tf","/ch/lslidar_point_cloud"};reader->set_filter(filter);}
 auto process=[&](const sensor_msgs::msg::PointCloud2::SharedPtr&m,uint64_t input_seq,double reader_ms,double deserialize_ms,double cache_load_ms){
   if(!rclcpp::ok())throw std::runtime_error("Replay interrupted; cache remains incomplete");
   ++count;int64_t stamp=int64_t(m->header.stamp.sec)*1000000000LL+m->header.stamp.nanosec;if(count==1)first_stamp=stamp;
   if(count>1&&stamp<=last_stamp)throw std::runtime_error("Nonmonotonic input timestamp; unsupported sequence");
   last_stamp=stamp;
   captured=false;initial_depth.clear();initial_visibility.clear();framehash=Json::object();public_hash=Json::object();
   if(cached)n.offline_capture_(loaded);
   auto t=Clock::now();n.processCloudFrame({m,Clock::now(),input_seq});double pipeline_ms=seconds(t)*1000;pipeline_sec+=pipeline_ms*.001;
   if(!captured&&!(fast&&mode=="full"))throw std::runtime_error("Frame did not reach cache boundary (TF/parsing failure); incomplete cache rejected at input "+std::to_string(input_seq));
   t=Clock::now();
   if(build){auto offset=cache_out.tellp();Writer w;w(input_seq);w(cloudMetadata(*m));w(loaded);w(initial_depth_bytes);writeRecord(cache_out,w.bytes,compressed);write_sec+=seconds(t);if(dataset){auto cfg=n.visibility_model_.config();index_out<<Json({{"frame",count},{"input_sequence",input_seq},{"stamp_ns",stamp},{"relative_sec",double(stamp-first_stamp)*1e-9},{"offset",uint64_t(offset)},{"record_bytes",uint64_t(cache_out.tellp())-uint64_t(offset)},{"points",loaded.points.size()},{"clusters",loaded.clusters.size()},{"enabled",loaded.enabled},{"azimuth_bin_deg",cfg.azimuth_bin_deg},{"ray_elevation_spacing_deg",cfg.ray_elevation_spacing_deg}}).dump()<<'\n';}}
   else{
     if(!fast){Json out={{"cloud_metadata",digest(cloudMetadata(*m))},{"frame",count},{"input_sequence",input_seq},{"stamp_ns",stamp},{"stages",framehash},{"initial_depth",initial_depth},{"initial_visibility",initial_visibility},{"final_depth",depth(n)},{"tracks",digest(n.tracks_)},{"fifo",digest(n.inside_vehicle_queue_.ids(),n.pending_exit_track_id_,n.next_vehicle_entry_sequence_)},{"memory_banks",digest(n.object_memory_bank_,n.class_memory_bank_)},{"runtime",digest(n.next_track_id_,n.last_tracking_stamp_,n.last_candidates_,n.recording_sparse_features_)},{"public",public_hash}};
     // Pointer maps are encoded through their actual feature values, never addresses.
     out["observed_features"]=observed_hash;
     evidence<<out.dump()<<'\n';}
     for(const auto&tr:n.tracks_){bool pub=n.physicalOutputAllowed(tr)&&tr.detection.is_valid&&tr.detection.lifecycle_state!="CANDIDATE"&&tr.detection.lifecycle_state!="REJECTED";trackcsv<<count<<','<<std::setprecision(17)<<double(stamp-first_stamp)*1e-9<<','<<tr.id<<','<<tr.lost_frames<<','<<pub<<','<<(pub&&tr.detection.class_name=="VEHICLE")<<','<<tr.vehicle_status.value<<','<<tr.observation_memory.association_source<<','<<(tr.lost_frames==0?tr.feature.point_count:0)<<','<<n.outputX(tr)<<','<<n.outputY(tr)<<','<<n.outputVx(tr)<<','<<n.outputVy(tr)<<'\n';}
   }
   if(dataset&&!build){Json event={{"frame",count},{"input_sequence",input_seq},{"stamp_ns",stamp},{"fifo",n.inside_vehicle_queue_.ids()},{"pending_exit_track_id",n.pending_exit_track_id_},{"tracks",Json::array()},{"candidates",Json::array()}};
   for(const auto&tr:n.tracks_)event["tracks"].push_back({{"id",tr.id},{"lost_frames",tr.lost_frames},{"lifecycle",tr.detection.lifecycle_state},{"rejection_reason",tr.detection.rejection_reason},{"association_source",tr.observation_memory.association_source},{"status",tr.vehicle_status.value},{"dormant",n.dormantVehicle(tr)},{"x",n.outputX(tr)},{"y",n.outputY(tr)},{"vx",n.outputVx(tr)},{"vy",n.outputVy(tr)}});
   for(const auto&f:n.last_candidates_)event["candidates"].push_back({{"id",f.id},{"rejection_reason",f.rejection_reason},{"component_type",f.component_type},{"decision_reason",f.decision_reason},{"visibility_reason",cx128s2_bev_perception::visibilityReasonName(f.visibility.reason)}});
   trace<<event.dump()<<'\n';}
   double result_ms=seconds(t)*1000;result_sec+=result_ms*.001;
   times<<count<<','<<input_seq<<','<<std::setprecision(17)<<double(stamp-first_stamp)*1e-9<<','<<reader_ms<<','<<deserialize_ms<<','<<cache_load_ms<<','<<pipeline_ms<<','<<result_ms<<','<<rss()<<','<<n.read_cloud_ms_<<','<<n.valid_crop_ms_<<','<<n.roi_policy_ms_<<','<<n.foreground_classification_ms_<<','<<n.radius_outlier_removal_ms_<<','<<n.clustering_ms_<<','<<n.feature_build_ms_<<','<<n.raw_vehicle_box_refine_ms_<<','<<n.visibility_depth_build_ms_<<','<<n.visibility_cluster_ms_<<','<<n.tracking_ms_<<','<<n.track_memory_update_ms_<<'\n';
   if(count%200==0)std::cerr<<mode<<" frame="<<count<<"\n";
 };
 if(cached){size_t first_index=0;if(inspect_frame){if(!dataset)throw std::runtime_error("inspect requires dataset");std::ifstream idx(cache/"frames.index");std::string line;bool found=false;while(std::getline(idx,line)){auto row=Json::parse(line);if(row.at("frame")==inspect_frame){cache_in.seekg(row.at("offset").get<uint64_t>());first_index=inspect_frame-1;found=true;break;}}if(!found)throw std::runtime_error("Frame not in index");}
 for(size_t i=first_index;i<manifest.at("frames").get<size_t>();++i){auto t=Clock::now();auto bytes=readRecord(cache_in,!fast);Reader a{bytes};uint64_t seq;std::vector<uint8_t>raw;a(seq);a(raw);a(loaded);a(initial_depth_bytes);if(a.pos!=bytes.size())throw std::runtime_error("Trailing record bytes");if(!sequence.empty()&&seq!=sequence.at(i))throw std::runtime_error("Input sequence mismatch");if(sequence.empty()&&seq!=i+1)throw std::runtime_error("Missing/reordered input frame");auto m=std::make_shared<sensor_msgs::msg::PointCloud2>(decode<sensor_msgs::msg::PointCloud2>(raw));if(inspect_frame){Json out={{"frame",seq},{"stamp",m->header.stamp.sec+m->header.stamp.nanosec*1e-9},{"roi_indices",loaded.roi},{"foreground_indices",loaded.foreground},{"owners",loaded.owners},{"compact_to_original",loaded.compact_to_original},{"points",Json::array()},{"clusters",Json::array()},{"features",Json::array()},{"sensor_origin",{loaded.tf.getOrigin().x(),loaded.tf.getOrigin().y(),loaded.tf.getOrigin().z()}},{"sensor_rotation",Json::array()},{"depth_snapshot_bytes",initial_depth_bytes.size()}};
 for(const auto&p:loaded.points)out["points"].push_back({p.x,p.y,p.z,p.intensity,p.ring,p.time});
 for(int r=0;r<3;++r)out["sensor_rotation"].push_back({loaded.tf.getBasis()[r][0],loaded.tf.getBasis()[r][1],loaded.tf.getBasis()[r][2]});
 for(const auto&c:loaded.clusters)out["clusters"].push_back({{"id",c.id},{"indices",c.indices}});
 for(const auto&f:loaded.features)out["features"].push_back({{"id",f.id},{"points",f.point_count},{"box",{f.min_x,f.max_x,f.min_y,f.max_y,f.min_z,f.max_z}}});
 out["roi_zones"]=Json::array();for(const auto&z:n.roi_zones_)out["roi_zones"].push_back({{"name",z.name},{"shape",z.shape},{"type",z.type},{"enabled",z.enabled},{"bounds",{z.x_min,z.x_max,z.y_min,z.y_max}},{"polygon_x",z.polygon_x},{"polygon_y",z.polygon_y}});
 if(!initial_depth_bytes.empty()){Reader d{initial_depth_bytes};std::string sig;int amin,emin,ac,ec;size_t occupied;std::vector<int> indices;d(sig);d(amin);d(emin);d(ac);d(ec);d(occupied);d(indices);out["depth_cells"]=Json::array();for(auto ix:indices){cx128s2_bev_perception::VisibilityModel::DepthCell cell;d(cell);out["depth_cells"].push_back({{"index",ix},{"nearest",cell.nearest},{"farthest",cell.farthest},{"owner",cell.nearest_owner},{"count",cell.count},{"world",{cell.nearest_x,cell.nearest_y,cell.nearest_z}}});}out["depth_grid"]={{"az_min_bin",amin},{"el_min_bin",emin},{"az_count",ac},{"el_count",ec}};}
 saveJson(prefix.string()+"_frame.json",out);break;}
 double ms=seconds(t)*1000;load_sec+=ms*.001;n.offline_loaded_=&loaded;process(m,seq,0,0,ms);if(stop_frame&&count>=stop_frame)break;}if(!inspect_frame&&!stop_frame&&cache_in.peek()!=std::char_traits<char>::eof())throw std::runtime_error("Trailing cache record");if(!inspect_frame&&!stop_frame&&(count!=manifest.at("frames").get<size_t>()||first_stamp!=manifest.at("first_stamp_ns").get<int64_t>()||last_stamp!=manifest.at("last_stamp_ns").get<int64_t>()))throw std::runtime_error("Frame range mismatch");}
 else{size_t sequence_index=0;for(;;){auto t=Clock::now();const bool next=reader->has_next();const double next_ms=seconds(t)*1000;if(!next){read_sec+=next_ms*.001;break;}t=Clock::now();auto b=reader->read_next();rclcpp::SerializedMessage s(*b->serialized_data);double r_ms=seconds(t)*1000+next_ms;read_sec+=r_ms*.001;t=Clock::now();if(b->topic_name=="/tf_static"||b->topic_name=="/tf"){tf2_msgs::msg::TFMessage tf;rclcpp::Serialization<tf2_msgs::msg::TFMessage>().deserialize_message(&s,&tf);for(auto&v:tf.transforms)n.tf_buffer_->setTransform(v,"bag",b->topic_name=="/tf_static");tf_hashes.push_back(digest(cdr(tf)));continue;}// Static transforms are valid for all cloud timestamps. A recorder can write its
// transient-local snapshot after the first cloud; recover only that initial case.
if(input_count==0&&tf_hashes.empty()){
 rosbag2_cpp::Reader statics;statics.open(bag.string());rosbag2_storage::StorageFilter sf;sf.topics={"/tf_static"};statics.set_filter(sf);
 while(statics.has_next()){auto sb=statics.read_next();rclcpp::SerializedMessage ss(*sb->serialized_data);tf2_msgs::msg::TFMessage tf;rclcpp::Serialization<tf2_msgs::msg::TFMessage>().deserialize_message(&ss,&tf);for(auto&v:tf.transforms)n.tf_buffer_->setTransform(v,"bag-static-bootstrap",true);}
}
++input_count;if(!sequence.empty()){if(sequence_index==sequence.size()||input_count!=sequence[sequence_index])continue;++sequence_index;}auto m=std::make_shared<sensor_msgs::msg::PointCloud2>();rclcpp::Serialization<sensor_msgs::msg::PointCloud2>().deserialize_message(&s,m.get());double d_ms=seconds(t)*1000;deser_sec+=d_ms*.001;process(m,input_count,r_ms,d_ms,0);}if(!sequence.empty()&&sequence_index!=sequence.size())throw std::runtime_error("Requested input sequence absent from bag");}
 if(build){cache_out.close();index_out.close();manifest={{"contract",expected},{"frames",count},{"source_cloud_frames",input_count},{"first_stamp_ns",first_stamp},{"last_stamp_ns",last_stamp},{"ordered_tf_sha256",digest(tf_hashes)},{"bytes",fs::file_size(cache/"frames.bin")},{"complete",true}};if(dataset){manifest["provenance"]={{"bag",fs::canonical(bag).string()},{"bag_files",bagIdentity(bag)},{"producer_executable",shaFile(exe)}};manifest["index_sha256"]=shaFile(cache/"frames.index");}saveJson(cache/"manifest.json.tmp",manifest);fs::rename(cache/"manifest.json.tmp",cache/"manifest.json");fs::remove(cache/"incomplete.json");}
 struct rusage u{};getrusage(RUSAGE_SELF,&u);Json stats={{"mode",mode},{"fast",fast},{"frames",count},{"wall_sec",seconds(run_begin)},{"reader_sec",read_sec},{"deserialize_sec",deser_sec},{"cache_load_sec",load_sec},{"pipeline_sec",pipeline_sec},{"result_sec",result_sec},{"cache_write_sec",write_sec},{"user_sec",u.ru_utime.tv_sec+u.ru_utime.tv_usec*1e-6},{"system_sec",u.ru_stime.tv_sec+u.ru_stime.tv_usec*1e-6},{"peak_rss_kb",u.ru_maxrss},{"end_rss_kb",rss()}};saveJson(prefix.string()+"_stats.json",stats);std::cerr<<stats.dump()<<'\n';
 n.offline_restore_depth_={};n.offline_loaded_=nullptr;n.offline_capture_={};n.offline_before_tracking_={};n.offline_initial_visibility_={};n.offline_public_={};n.offline_after_frame_={};
}
};
namespace replay_cache {template<class A>void Custom<cx128s2_bev_perception::LinearKalmanFilter>::visit(A&a,cx128s2_bev_perception::LinearKalmanFilter&k){TrackingRegressionAccess::kf(a,k);}}
int main(int argc,char**argv){try{
 if(argc<6)throw std::runtime_error("usage: general_replay {full|cache-build|cache-replay} BAG CACHE OUTPUT [--sequence FILE] [--depth-cache] --ros-args ...");
 std::string mode=argv[1];fs::path bag=argv[2],cache=argv[3],out=argv[4];std::vector<uint64_t>seq;int offset=5;bool depth_cache=false,fast=false,compressed=false,dataset=false;uint64_t stop_frame=0,inspect_frame=0;
 while(offset<argc&&std::string(argv[offset])!="--ros-args"){
  std::string option=argv[offset++];
  if(option=="--dataset")dataset=true;
  else if(option=="--stop-frame"&&offset<argc)stop_frame=std::stoull(argv[offset++]);
  else if(option=="--inspect-frame"&&offset<argc)inspect_frame=std::stoull(argv[offset++]);
  else if(option=="--depth-cache")depth_cache=true;
  else if(option=="--fast")fast=true;
  else if(option=="--compress")compressed=true;
  else if(option=="--sequence"&&offset<argc){std::ifstream f(argv[offset++]);uint64_t x;while(f>>x)seq.push_back(x);if(!f.eof()||seq.empty()||!std::is_sorted(seq.begin(),seq.end())||std::adjacent_find(seq.begin(),seq.end())!=seq.end()||seq.front()==0)throw std::runtime_error("Invalid sequence file");}
  else throw std::runtime_error("Unknown offline option: "+option);
 }
 rclcpp::init(argc-offset+1,argv+offset-1);auto n=std::make_shared<Cx128s2BevPerception>();
 TrackingRegressionAccess::run(*n,mode,bag,cache,out,fs::canonical("/proc/self/exe"),seq,depth_cache,fast,compressed,dataset,stop_frame,inspect_frame);
 n.reset();rclcpp::shutdown();return 0;
}catch(const std::exception&e){std::cerr<<"REPLAY ERROR: "<<e.what()<<'\n';if(rclcpp::ok())rclcpp::shutdown();return 2;}}
