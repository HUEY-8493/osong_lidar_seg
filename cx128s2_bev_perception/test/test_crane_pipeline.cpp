// Oracle functions below are copied from the user's supplied source, unchanged.
// Production implementation is exercised separately against this C32 baseline.
#include <cx128s2_bev_perception/crane_pipeline.hpp>
#include <cx128s2_bev_perception/crane_hybrid_policy.hpp>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <numeric>
#include <random>
#include <stdexcept>
#include <string>
#include <sstream>
using namespace cx128s2_bev_perception;
using PointXYZIR = PointXYZIRT;
constexpr float EPS = 1e-6f;
constexpr double pi = 3.14159265358979323846;
#define CHECK(x) do { if (!(x)) { std::cerr << "Failed: " #x << " line " << __LINE__ << '\n'; std::abort(); } } while (0)
struct SuppliedReference {
  int cfg_lidar_channel=32, cfg_width=666;
  float cfg_lidar_height=.75f, cfg_lidar_degree_step=1, cfg_lidar_degree_start=-50;
  float cfg_max_slope_degree=10, cfg_max_vertical_height=.08f;
  std::vector<double> rtable_prefix_;
    void initRadiusTable() {
        float alpha = cfg_lidar_degree_step * M_PI / 180.0f;
        float beta = cfg_max_slope_degree * M_PI / 180.0f;

        std::vector<double> r0(cfg_lidar_channel);
        for (int i = 0; i < cfg_lidar_channel; ++i) {
            float theta = (cfg_lidar_degree_start + i * cfg_lidar_degree_step) * M_PI / 180.0f;
            if (std::abs(theta) < 0.1f * M_PI / 180.0f) {
                r0[i] = 0.0;
                continue;
            }
            double d1 = std::tan(theta) + std::tan(beta);
            double d2 = std::tan(alpha + theta) + std::tan(beta);
            double r_diff = (std::abs(d1) < EPS || std::abs(d2) < EPS) ? 1e6 : (1.0 / d1 - 1.0 / d2);
            r0[i] = cfg_lidar_height * r_diff;
        }

        rtable_prefix_.assign(cfg_lidar_channel + 1, 0.0);
        for (int i = 0; i < cfg_lidar_channel; ++i) {
            rtable_prefix_[i + 1] = rtable_prefix_[i] + r0[i];
        }
    }

    void segmentGroundCpu(const std::vector<PointXYZIR>& pts,
                          const std::vector<int>& depth,
                          std::vector<int>& ground_idx,
                          std::vector<int>& obstacle_idx) {
        std::vector<int> indices;
        std::vector<int> rings_present;
        std::vector<int> cluster;
        indices.reserve(cfg_lidar_channel);
        rings_present.reserve(cfg_lidar_channel);
        cluster.reserve(cfg_lidar_channel);

        for (int c = 0; c < cfg_width; ++c) {
            indices.clear();
            rings_present.clear();
            for (int r = 0; r < cfg_lidar_channel; ++r) {
                const int point_idx = depth[c * cfg_lidar_channel + r];
                if (point_idx != -1) {
                    indices.push_back(point_idx);
                    rings_present.push_back(r);
                }
            }
            if (indices.size() < 2) continue;

            cluster.clear();
            cluster.push_back(indices[0]);
            float lz_min = pts[indices[0]].z;
            float lz_max = pts[indices[0]].z;

            for (size_t k = 0; k < indices.size() - 1; ++k) {
                int a = rings_present[k];
                int b = rings_present[k + 1];
                float thresh = static_cast<float>(rtable_prefix_[b] - rtable_prefix_[a]);

                float r_curr = std::hypot(pts[indices[k]].x, pts[indices[k]].y);
                float r_next = std::hypot(pts[indices[k + 1]].x, pts[indices[k + 1]].y);

                if (std::abs(r_next - r_curr) < thresh) {
                    cluster.push_back(indices[k + 1]);
                    lz_min = std::min(lz_min, pts[indices[k + 1]].z);
                    lz_max = std::max(lz_max, pts[indices[k + 1]].z);
                } else {
                    auto& target = (lz_max - lz_min > cfg_max_vertical_height) ? obstacle_idx : ground_idx;
                    target.insert(target.end(), cluster.begin(), cluster.end());
                    cluster.clear();
                    cluster.push_back(indices[k + 1]);
                    lz_min = lz_max = pts[indices[k + 1]].z;
                }
            }
            auto& target = (lz_max - lz_min > cfg_max_vertical_height) ? obstacle_idx : ground_idx;
            target.insert(target.end(), cluster.begin(), cluster.end());
        }
    }

};

static tf2::Transform installation() {
  tf2::Quaternion q;q.setRPY(-1.5671972765207525,-0.027489352235824036,0);
  return tf2::Transform(q,{0,0,6.638276670931532});
}
static std::vector<int> allIndices(size_t n) {
  std::vector<int> indices(n);std::iota(indices.begin(),indices.end(),0);return indices;
}
static void partition(const CraneSlopeResult & r,size_t n,bool all=false) {
  auto indices=r.ground;indices.insert(indices.end(),r.non_ground.begin(),r.non_ground.end());
  indices.insert(indices.end(),r.unknown.begin(),r.unknown.end());std::sort(indices.begin(),indices.end());
  CHECK(indices==allIndices(n));CHECK(r.labels.size()==n);
  CHECK(r.clustering.size()==(all?n:r.non_ground.size()+r.unknown.size()));
  CHECK(r.state==CraneBeamState::IDLE);
}
static PointXYZIRT rayPoint(const tf2::Transform & pose,double az,double el,double z,uint16_t ring) {
  az*=pi/180;el*=pi/180;
  const auto d=pose.getBasis()*tf2::Vector3(std::cos(el)*std::cos(az),std::cos(el)*std::sin(az),std::sin(el));
  CHECK(d.z()<0);
  const auto p=pose.getOrigin()+d*((z-pose.getOrigin().z())/d.z());
  PointXYZIRT out;out.x=p.x();out.y=p.y();out.z=p.z();out.ring=ring;return out;
}
// Independent fixture ray generator transcribed from the installed driver's
// init tables / convertCoordinate_cx128s2, including its .01deg lookup rounding.
static double driverElevation(int ring,double az,const std::vector<double> & prism={0,-.17,-.34,-.51},double offset=0) {
  const double base[32]={-12,-11,-10,-9,-8,-7,-6,-5,-4.125,-4,-3.125,-3,-2.125,-2,-1.125,-1,
      -.125,0,.875,1,1.875,2,3,4,5,6,7,8,9,10,11,12};
  const double a=(base[ring/4]+(ring/4%2==0?offset:0))*pi/180;
  const double b=prism[ring%4]*pi/180;
  const int angle_h=std::lround(az*100);
  const double reflection=std::cos(b)*std::cos(a)*std::cos((angle_h/2)*.01*pi/180)-std::sin(b)*std::sin(a);
  return std::asin(std::sin(a)+2*reflection*std::sin(b))*180/pi;
}
static std::vector<PointXYZIRT> cxPlane(const tf2::Transform & pose,double z,
    const std::vector<double> & prism={0,-.17,-.34,-.51},double offset=0) {
  std::vector<PointXYZIRT> points;
  for(double az:{45.05,90.05,134.95}) for(int ring=0;ring<128;++ring)
    points.push_back(rayPoint(pose,az,driverElevation(ring,az,prism,offset),z,ring));
  return points;
}

static void suppliedParity() {
  // Actual pasted initRadiusTable/segmentGroundCpu are the independent oracle.
  // Angles deliberately avoid the source's horizon/pole singularities.
  for(float beta:{0.f,5.f,10.f}) for(bool missing:{false,true}) for(bool duplicate:{false,true}) {
    SuppliedReference source;source.cfg_max_slope_degree=beta;source.initRadiusTable();
    CraneSlopeConfig config;config.lidar_model="uniform";config.lidar_degree_start=-50;
    config.lidar_channels=32;config.max_slope_deg=beta;
    CranePipeline pipeline(config);
    std::vector<PointXYZIRT> raw,world;
    for(int j=0;j<32;++j) {
      if(missing && (j==10||j==19))continue;
      const double theta=(-50+j)*pi/180;
      const float z=(j>=7&&j<=14)? .25f+.02f*(j-7):.01f;
      PointXYZIRT p;p.y=(z-.75f)/std::tan(theta);p.z=z-.75f;p.ring=31-j;
      raw.push_back(p);
    }
    if(duplicate) {auto p=raw[4];p.y*=.6f;p.z*=.6f;raw.push_back(p);}
    std::vector<int> depth(source.cfg_width*source.cfg_lidar_channel,-1);
    for(size_t i=0;i<raw.size();++i) {
      float az=std::atan2(raw[i].y,raw[i].x);if(az<0)az+=2.f*M_PI;
      const int col=source.cfg_width-static_cast<int>(source.cfg_width*(az*180.f/M_PI)/360.f)-1;
      depth[col*source.cfg_lidar_channel+source.cfg_lidar_channel-raw[i].ring-1]=i;
      auto p=raw[i];p.z+=.75f;world.push_back(p);
    }
    std::vector<int> ground,obstacle;source.segmentGroundCpu(raw,depth,ground,obstacle);
    std::vector<CraneSlopeLabel> expected(raw.size(),CraneSlopeLabel::UNKNOWN);
    for(int i:ground)expected[i]=CraneSlopeLabel::GROUND;
    for(int i:obstacle)expected[i]=CraneSlopeLabel::NON_GROUND;
    const tf2::Transform pose(tf2::Quaternion::getIdentity(),{0,0,.75});
    const auto & r=pipeline.process(world,allIndices(world.size()),pose,true);
    partition(r,world.size());
    if(r.labels!=expected)std::cerr<<"parity beta="<<beta<<" missing="<<missing<<" duplicate="<<duplicate<<'\n';
    CHECK(r.labels==expected);CHECK(!r.ground.empty());CHECK(!r.non_ground.empty());
    CHECK(r.overwritten_points==static_cast<size_t>(duplicate));
  }
  std::cout<<"supplied source parity: beta=0/5/10, missing rings, cell overwrites (12 cases) passed\n";
}

static void cxGeometry() {
  const auto pose=installation();CraneSlopeConfig cfg;cfg.azimuth_columns=3600;
  cfg.include_ground_in_clustering=true;CranePipeline pipeline(cfg);
  auto points=cxPlane(pose,0);auto indices=allIndices(points.size());
  auto r=pipeline.process(points,indices,pose,true);partition(r,points.size(),true);
  CHECK(r.pose_valid && r.observed_rings==128 && r.model_mismatch_points==0);
  CHECK(r.ground.size()==points.size());
  const auto original=r;tf2::Quaternion yaw;yaw.setRPY(0,0,1.1);const tf2::Transform change(yaw,{7,-3,0});
  for(auto & p:points) {const auto q=change*tf2::Vector3(p.x,p.y,p.z);p.x=q.x();p.y=q.y();p.z=q.z();}
  r=pipeline.process(points,indices,change*pose,true);CHECK(r.labels==original.labels);
  points=cxPlane(pose,2);r=pipeline.process(points,indices,pose,true);
  partition(r,points.size(),true);CHECK(r.ground.size()==points.size());
  // Ring identity is now physical: scrambling it must trigger disagreement.
  points=cxPlane(pose,0);for(auto & p:points)p.ring=127-p.ring;
  r=pipeline.process(points,indices,pose,true);CHECK(r.model_mismatch_points>0);
  points=cxPlane(pose,0);auto raised=cxPlane(pose,2);points.insert(points.end(),raised.begin(),raised.end());
  indices=allIndices(points.size());r=pipeline.process(points,indices,pose,true);partition(r,points.size(),true);
  CHECK(r.overwritten_points==raised.size() && r.unknown.size()==raised.size());
  r=pipeline.process(points,indices,pose,false);CHECK(r.unknown.size()==points.size());
  points=cxPlane(pose,0);points[0].ring=500;indices=allIndices(points.size());
  r=pipeline.process(points,indices,pose,true);CHECK(r.labels[0]==CraneSlopeLabel::UNKNOWN);
  auto bad=pose;bad.setOrigin({0,0,0});r=pipeline.process(points,indices,bad,true);
  CHECK(!r.pose_valid && r.unknown.size()==points.size());
  // Nondefault device calibration, matching the driver's DIFOP behavior.
  cfg.cx128s2_prism_angles_deg={.04,-.14,-.30,-.46};cfg.cx128s2_prism_offset_deg=.2;
  CranePipeline calibrated(cfg);points=cxPlane(pose,0,cfg.cx128s2_prism_angles_deg,.2);
  r=calibrated.process(points,allIndices(points.size()),pose,true);
  CHECK(r.model_mismatch_points==0 && r.ground.size()==points.size());
  // Missing rings use the original telescoping prefix-gap relation.
  points.erase(std::remove_if(points.begin(),points.end(),[](const auto & p){return p.ring%3==0;}),points.end());
  r=calibrated.process(points,allIndices(points.size()),pose,true);partition(r,points.size(),true);
  CHECK(r.model_mismatch_points==0 && r.ground.size()==points.size());
  std::cout<<"CX driver model: 128 physical rings, mounting TF, nadir, yaw/translation, elevated plane, mismatches, collisions, missing rings and custom DIFOP passed\n";
}
static void degeneracies() {
  const auto pose=installation();CraneSlopeConfig cfg;cfg.azimuth_columns=3600;
  cfg.include_ground_in_clustering=true;CranePipeline pipeline(cfg);
  auto point=rayPoint(pose,90.05,driverElevation(0,90.05),0,0);
  auto r=pipeline.process({point},{0},pose,true);CHECK(r.unknown.size()==1);
  std::vector<PointXYZIRT> wall;
  for(int j=0;j<128;++j) {
    auto p=rayPoint(pose,90.05,driverElevation(j,90.05),0,j);
    const auto d=(tf2::Vector3(p.x,p.y,p.z)-pose.getOrigin()).normalized();
    if(d.y()<=.01)continue; // Only forward intersections with the visible wall.
    const auto q=pose.getOrigin()+d*(.08/d.y());
    p.x=q.x();p.y=q.y();p.z=q.z();wall.push_back(p);
  }
  r=pipeline.process(wall,allIndices(wall.size()),pose,true);partition(r,wall.size(),true);
  CHECK(!r.non_ground.empty() && r.model_mismatch_points==0);
  // A C32 uniform model at theta=-beta is singular; no artificial 1e6 bridge.
  cfg.lidar_model="uniform";cfg.lidar_channels=2;cfg.lidar_degree_start=-11;
  CranePipeline uniform(cfg);const tf2::Transform upright(tf2::Quaternion::getIdentity(),{0,0,.75});
  auto singular=rayPoint(upright,90,-10,0,0), valid=rayPoint(upright,90,-11,0,1);
  r=uniform.process({singular,valid},{0,1},upright,true);
  CHECK(r.labels[0]==CraneSlopeLabel::UNKNOWN && r.unsupported_points>=1);
  // Explicit original lidar_height overrides automatic height, without changing TF.
  cfg.lidar_height=.75;CranePipeline explicitHeight(cfg);
  CHECK(explicitHeight.process({singular,valid},{0,1},upright,true).labels==r.labels);
  pipeline.reset();CHECK(pipeline.result().labels.empty());
  for(int which=0;which<9;++which) {
    cfg=CraneSlopeConfig{};
    if(which==0)cfg.azimuth_columns=0;
    if(which==1)cfg.lidar_channels=32;
    if(which==2)cfg.max_slope_deg=std::numeric_limits<double>::quiet_NaN();
    if(which==3)cfg.lidar_model="typo";
    if(which==4)cfg.lidar_degree_step=0;
    if(which==5)cfg.lidar_height=-1;
    if(which==6)cfg.cx128s2_prism_angles_deg={0};
    if(which==7)cfg.model_angle_tolerance_deg=0;
    if(which==8)cfg.cx128s2_prism_offset_deg=std::numeric_limits<double>::infinity();
    bool rejected=false;try{CranePipeline bad(cfg);}catch(const std::invalid_argument &){rejected=true;}CHECK(rejected);
  }
  std::cout<<"boundaries: sparse column, vertical surface, singularity, explicit height and validation passed\n";
}
static void finalGroundOverride() {
  const auto pose=installation();auto points=cxPlane(pose,0);
  // Include exact 3cm boundary and immediately above it using physical Ring rays.
  for(int ring=0;ring<4;++ring) {
    const float z=ring==0?-.02f: ring==1?.03f: ring==2?std::nextafter(.03f,1.f):.2f;
    points[ring]=rayPoint(pose,45.05,driverElevation(ring,45.05),z,ring);
    points[ring].z=z;
  }
  auto extra=points[0];extra.ring=500;points.push_back(extra);
  const auto indices=allIndices(points.size());CraneSlopeConfig cfg;cfg.azimuth_columns=3600;
  CranePipeline original(cfg);const auto before=original.process(points,indices,pose,true);
  cfg.force_ground_below_z=.03;CranePipeline enabled(cfg);
  const auto after=enabled.process(points,indices,pose,true);partition(after,points.size());
  for(size_t i=0;i<points.size();++i)
    CHECK(after.labels[i]==(points[i].z<=.03 ? CraneSlopeLabel::GROUND : before.labels[i]));
  CHECK(after.labels.back()==CraneSlopeLabel::GROUND);
  CHECK(after.neighbor_pairs==before.neighbor_pairs && after.broken_pairs==before.broken_pairs);
  CHECK(after.model_mismatch_points==before.model_mismatch_points);
  CHECK(after.unsupported_points==before.unsupported_points);
  cfg.include_ground_in_clustering=true;CranePipeline keepAll(cfg);
  partition(keepAll.process(points,indices,pose,true),points.size(),true);
  points.push_back(extra);points[0].z=std::numeric_limits<float>::quiet_NaN();
  const auto missing=keepAll.process(points,indices,pose,false);
  CHECK(missing.labels[0]==CraneSlopeLabel::UNKNOWN && missing.labels.back()==CraneSlopeLabel::UNKNOWN);
  CHECK(missing.labels[1]==CraneSlopeLabel::GROUND && missing.labels[2]==CraneSlopeLabel::UNKNOWN);
  for(double bad:{std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()}) {
    cfg.force_ground_below_z=bad;bool rejected=false;
    try{CranePipeline invalid(cfg);}catch(const std::invalid_argument &){rejected=true;}CHECK(rejected);
  }
  std::cout<<"final 3cm override: inclusive boundary, unchanged slope diagnostics, partitions, Unknown and finite inputs passed\n";
}
static void twoDirections() {
  const auto pose=installation();CraneSlopeConfig cfg;
  cfg.azimuth_columns=3600;cfg.horizontal_enabled=true;
  cfg.include_ground_in_clustering=true;cfg.max_vertical_height=.05;
  const auto makeScene = [&](bool ramp,double first_az=65.05) {
    std::vector<PointXYZIRT> points;
    for(int col=0;col<80;++col) for(int ring=0;ring<128;++ring) {
      const double az=first_az+.1*col;
      points.push_back(rayPoint(pose,az,driverElevation(ring,az),ramp?1+.004*col:1,ring));
    }
    return points;
  };
  CranePipeline pipeline(cfg);auto points=makeScene(false);auto ids=allIndices(points.size());
  auto flat=pipeline.process(points,ids,pose,true);partition(flat,points.size(),true);
  CHECK(flat.vertical_labels.size()==points.size() && flat.horizontal_labels.size()==points.size());
  CHECK(flat.horizontal_neighbor_pairs>0 && flat.horizontal_runs>0);
  CHECK(flat.ground.size()==points.size());
  for(auto l:flat.vertical_labels)CHECK(l==CraneSlopeLabel::GROUND);
  for(auto l:flat.horizontal_labels)CHECK(l==CraneSlopeLabel::GROUND);
  // Rigid yaw / XY origin changes must not change either direction.
  tf2::Quaternion yaw;yaw.setRPY(0,0,.7);const tf2::Transform change(yaw,{4,-2,0});
  for(auto & p:points){const auto q=change*tf2::Vector3(p.x,p.y,p.z);p.x=q.x();p.y=q.y();p.z=q.z();}
  auto changed=pipeline.process(points,ids,change*pose,true);
  CHECK(changed.vertical_labels==flat.vertical_labels && changed.horizontal_labels==flat.horizontal_labels);
  CHECK(changed.labels==flat.labels);
  // A surface whose height changes only across azimuth: vertical-only misses
  // that variation, horizontal run-height classification must detect it.
  points=makeScene(true);auto ramp=pipeline.process(points,ids,pose,true);partition(ramp,points.size(),true);
  CHECK(!ramp.non_ground.empty());
  for(auto l:ramp.vertical_labels)CHECK(l==CraneSlopeLabel::GROUND);
  size_t horizontal_non_ground=0;
  for(auto l:ramp.horizontal_labels)horizontal_non_ground+=l==CraneSlopeLabel::NON_GROUND;
  CHECK(horizontal_non_ground==points.size());
  for(size_t i=0;i<points.size();++i) {
    const auto v=ramp.vertical_labels[i],h=ramp.horizontal_labels[i];
    const auto expected=(v==CraneSlopeLabel::NON_GROUND||h==CraneSlopeLabel::NON_GROUND)?CraneSlopeLabel::NON_GROUND:
        (v==CraneSlopeLabel::GROUND&&h==CraneSlopeLabel::GROUND)?CraneSlopeLabel::GROUND:CraneSlopeLabel::UNKNOWN;
    CHECK(ramp.labels[i]==expected);
  }
  // Geometry remains valid across the nadir; do not use unsigned XY radii.
  points=makeScene(false,86.05);auto nadir=pipeline.process(points,ids,pose,true);
  CHECK(nadir.horizontal_neighbor_pairs>0 && nadir.model_mismatch_points==0);
  CHECK(nadir.ground.size()>points.size()*9/10);
  // Three separated columns: no horizontal evidence may bridge a 45deg gap.
  points=cxPlane(pose,1);ids=allIndices(points.size());auto sparse=pipeline.process(points,ids,pose,true);
  partition(sparse,points.size(),true);CHECK(sparse.unknown.size()==points.size());
  CHECK(sparse.horizontal_gap_breaks>0 && sparse.horizontal_neighbor_pairs==0);
  for(auto l:sparse.horizontal_labels)CHECK(l==CraneSlopeLabel::UNKNOWN);
  cfg.horizontal_enabled=false;CranePipeline verticalOnly(cfg);
  const auto original=verticalOnly.process(points,ids,pose,true);
  CHECK(original.ground.size()==points.size() && original.vertical_labels.empty() && original.horizontal_labels.empty());
  CHECK(original.horizontal_neighbor_pairs==0);
  // A missing Ring field remains unknown in both directions.
  auto missing=pipeline.process(points,ids,pose,false);CHECK(missing.unknown.size()==points.size());
  // Displaced cell representatives remain Unknown, no duplicated partition.
  points=makeScene(false);auto more=points;points.insert(points.end(),more.begin(),more.end());ids=allIndices(points.size());
  auto collision=pipeline.process(points,ids,pose,true);partition(collision,points.size(),true);
  CHECK(collision.overwritten_points==more.size() && collision.unknown.size()==more.size());
  // Final 3cm override does not change either raw directional result.
  cfg.horizontal_enabled=true;cfg.force_ground_below_z=.03;CranePipeline override(cfg);
  points=cxPlane(pose,0);ids=allIndices(points.size());auto low=override.process(points,ids,pose,true);
  CHECK(low.ground.size()==points.size());
  for(auto l:low.horizontal_labels)CHECK(l==CraneSlopeLabel::UNKNOWN);
  for(double bad:{0.,-1.,std::numeric_limits<double>::quiet_NaN(),6.}) {
    cfg.horizontal_max_azimuth_gap_deg=bad;bool rejected=false;
    try{CranePipeline invalid(cfg);}catch(const std::invalid_argument &){rejected=true;}CHECK(rejected);
  }
  pipeline.reset();CHECK(pipeline.result().vertical_labels.empty() && pipeline.result().horizontal_labels.empty());
  CHECK(pipeline.result().horizontal_neighbor_pairs==0);
  std::cout<<"two directions: flat/ramp, nadir, TF invariance, gaps, fusion, disabled parity, collisions and final override passed\n";
}

static PointXYZIRT planeRay(const tf2::Transform & pose,double az,int ring,
    const tf2::Vector3 & normal,const tf2::Vector3 & center) {
  const double el=driverElevation(ring,az)*pi/180,phi=az*pi/180;
  const auto d=pose.getBasis()*tf2::Vector3(std::cos(el)*std::cos(phi),std::cos(el)*std::sin(phi),std::sin(el));
  const double t=normal.dot(center-pose.getOrigin())/normal.dot(d);
  CHECK(t>0 && std::isfinite(t));const auto q=pose.getOrigin()+d*t;
  PointXYZIRT p;p.x=q.x();p.y=q.y();p.z=q.z();p.ring=ring;return p;
}
static std::vector<PointXYZIRT> planeScene(const tf2::Vector3 & normal,const tf2::Vector3 & center,
    int columns=80,double start=65.05,double step=.1) {
  std::vector<PointXYZIRT> points;const auto pose=installation();
  for(int col=0;col<columns;++col)for(int ring=0;ring<128;++ring)
    points.push_back(planeRay(pose,start+step*col,ring,normal,center));
  return points;
}
static void boundaryClusteringDetection() {
  const auto pose = installation();
  CraneSlopeConfig cfg;
  cfg.azimuth_columns = 3600;
  cfg.horizontal_enabled = true;
  cfg.include_ground_in_clustering = false;
  cfg.boundary_clustering.enabled = true;
  cfg.boundary_clustering.slope_threshold_deg = 20.0;
  cfg.boundary_clustering.height_step_threshold = 0.08;
  cfg.boundary_clustering.range_jump_threshold = 0.20;
  cfg.boundary_clustering.max_neighbor_distance = 0.50;
  cfg.boundary_clustering.min_cluster_points = 5;
  cfg.boundary_clustering.include_unknown = true;
  CranePipeline pipeline(cfg);

  std::vector<PointXYZIRT> scene;
  for (int col = 0; col < 60; ++col) {
    for (int ring = 0; ring < 128; ++ring) {
      const double az = 60.05 + 0.1 * col;
      double z = 0.0;
      if (col >= 10 && col < 25) z = 0.5;
      else if (col >= 35 && col < 50) z = 0.5;
      scene.push_back(rayPoint(pose, az, driverElevation(ring, az), z, ring));
    }
  }

  auto r = pipeline.process(scene, allIndices(scene.size()), pose, true);
  const auto & bc = r.boundary_clustering;

  std::cout << "boundary_clustering test: clusters=" << bc.clusters.size()
            << " boundaries=" << bc.boundary_indices.size()
            << " height_steps=" << bc.height_step_indices.size()
            << " slopes=" << bc.slope_indices.size()
            << " severed_edges=" << bc.severed_edges
            << " total_ms=" << bc.total_ms << std::endl;

  CHECK(bc.clusters.size() >= 2);
  CHECK(!bc.boundary_indices.empty());
  CHECK(bc.total_tested_edges > 0);
  CHECK(bc.severed_edges > 0);

  // Disabled mode verification
  cfg.boundary_clustering.enabled = false;
  CranePipeline disabled(cfg);
  auto r_off = disabled.process(scene, allIndices(scene.size()), pose, true);
  CHECK(r_off.boundary_clustering.clusters.empty());
  CHECK(r_off.boundary_clustering.boundary_indices.empty());
  std::cout << "boundary_clustering test passed successfully!\n";
}

static void zGaussianLayerGrouping() {
  const auto pose = installation();
  CraneSlopeConfig cfg;
  cfg.azimuth_columns = 3600;
  cfg.horizontal_enabled = true;
  cfg.include_ground_in_clustering = false;
  cfg.z_gaussian.enabled = true;
  cfg.z_gaussian.sigma_z = 0.025;
  cfg.z_gaussian.range_sigma_factor = 0.0005;
  cfg.z_gaussian.confidence_sigma_multiplier = 3.0;
  cfg.z_gaussian.step_sigma_multiplier = 2.5;
  cfg.z_gaussian.max_layer_z_spread = 0.25;
  cfg.z_gaussian.min_height_above_ground = 0.08;
  cfg.z_gaussian.min_layer_points = 10;
  CranePipeline pipeline(cfg);

  std::mt19937 rng(42);
  std::normal_distribution<double> g_noise(0.0, 0.02);

  std::vector<PointXYZIRT> scene;
  for (int col = 0; col < 60; ++col) {
    for (int ring = 0; ring < 128; ++ring) {
      const double az = 60.05 + 0.1 * col;
      double z = 0.0;
      if (col >= 10 && col < 25) {
        z = 0.50 + std::clamp(g_noise(rng), -0.06, 0.06);
      } else if (col >= 35 && col < 50) {
        z = 1.00 + std::clamp(g_noise(rng), -0.06, 0.06);
      }
      scene.push_back(rayPoint(pose, az, driverElevation(ring, az), z, ring));
    }
  }

  auto r = pipeline.process(scene, allIndices(scene.size()), pose, true);
  const auto & zg = r.z_gaussian;

  std::cout << "z_gaussian test: layers=" << zg.layers.size()
            << " candidates=" << zg.candidate_points
            << " grouped=" << zg.grouped_points
            << " grouping_ms=" << zg.grouping_ms << std::endl;

  CHECK(zg.layers.size() == 2);
  CHECK(std::abs(zg.layers[0].mean_z - 0.50) < 0.03);
  CHECK(std::abs(zg.layers[1].mean_z - 1.00) < 0.03);
  CHECK(zg.layers[0].point_indices.size() >= 1500);
  CHECK(zg.layers[1].point_indices.size() >= 1500);

  cfg.z_gaussian.enabled = false;
  CranePipeline disabled(cfg);
  auto r_off = disabled.process(scene, allIndices(scene.size()), pose, true);
  CHECK(r_off.z_gaussian.layers.empty());
  std::cout << "z_gaussian test passed successfully!\n";
}

static void closedLoopObjectGrouping() {
  const auto pose = installation();
  CraneSlopeConfig cfg;
  cfg.z_gaussian.enabled = true;
  cfg.closed_loop.enabled = true;
  cfg.closed_loop.grid_resolution_m = 0.08;
  cfg.closed_loop.closing_radius_m = 0.12;
  cfg.closed_loop.min_containment_ratio = 0.70;
  cfg.closed_loop.min_enclosed_cells = 4;

  // Build synthetic scene:
  // 1) Ground points at z = 0.0
  // 2) Outer rectangular closed loop at z = 1.0 m (x in [1.0, 2.6], y in [-0.8, 0.8], hollow interior)
  // 3) Inner floating object at z = 2.5 m inside the closed loop (x in [1.5, 2.1], y in [-0.3, 0.3])
  // 4) Separate outside object at z = 1.0 m outside the closed loop (x in [3.5, 4.0], y in [1.5, 2.0])
  std::vector<PointXYZIRT> points;
  std::vector<int> ground_indices;
  for (int ix = 0; ix < 10; ++ix) {
    for (int iy = 0; iy < 10; ++iy) {
      PointXYZIRT p{};
      p.x = 0.5f + 0.1f * ix;
      p.y = -2.0f + 0.1f * iy;
      p.z = 0.0f;
      ground_indices.push_back(static_cast<int>(points.size()));
      points.push_back(p);
    }
  }

  CraneSubcluster sub_loop{};
  sub_loop.subcluster_id = 0;
  sub_loop.layer_id = 0;
  for (int ix = 0; ix <= 40; ++ix) {
    for (int iy = 0; iy <= 40; ++iy) {
      const float x = 1.0f + 0.04f * ix;
      const float y = -0.8f + 0.04f * iy;
      const bool on_border = (x <= 1.16f || x >= 2.44f || y <= -0.64f || y >= 0.64f);
      if (!on_border) continue;
      PointXYZIRT p{};
      p.x = x;
      p.y = y;
      p.z = 1.0f;
      const int idx = static_cast<int>(points.size());
      points.push_back(p);
      sub_loop.point_indices.push_back(idx);
      sub_loop.boundary_point_indices.push_back(idx);
    }
  }
  sub_loop.min_x = 1.0; sub_loop.max_x = 2.6;
  sub_loop.min_y = -0.8; sub_loop.max_y = 0.8;
  sub_loop.min_z = 0.98; sub_loop.max_z = 1.02;
  sub_loop.centroid_x = 1.8; sub_loop.centroid_y = 0.0; sub_loop.centroid_z = 1.0;

  CraneSubcluster sub_inner{};
  sub_inner.subcluster_id = 1;
  sub_inner.layer_id = 1;
  std::vector<int> inner_point_indices;
  for (int ix = 0; ix <= 10; ++ix) {
    for (int iy = 0; iy <= 10; ++iy) {
      PointXYZIRT p{};
      p.x = 1.5f + 0.05f * ix;
      p.y = -0.25f + 0.05f * iy;
      p.z = 2.5f;
      const int idx = static_cast<int>(points.size());
      points.push_back(p);
      inner_point_indices.push_back(idx);
      sub_inner.point_indices.push_back(idx);
      sub_inner.boundary_point_indices.push_back(idx);
    }
  }
  sub_inner.min_x = 1.5; sub_inner.max_x = 2.0;
  sub_inner.min_y = -0.25; sub_inner.max_y = 0.25;
  sub_inner.min_z = 2.48; sub_inner.max_z = 2.52;
  sub_inner.centroid_x = 1.75; sub_inner.centroid_y = 0.0; sub_inner.centroid_z = 2.5;

  CraneSubcluster sub_outside{};
  sub_outside.subcluster_id = 2;
  sub_outside.layer_id = 0;
  for (int ix = 0; ix <= 8; ++ix) {
    for (int iy = 0; iy <= 8; ++iy) {
      PointXYZIRT p{};
      p.x = 3.5f + 0.05f * ix;
      p.y = 1.5f + 0.05f * iy;
      p.z = 1.0f;
      const int idx = static_cast<int>(points.size());
      points.push_back(p);
      sub_outside.point_indices.push_back(idx);
      sub_outside.boundary_point_indices.push_back(idx);
    }
  }
  sub_outside.min_x = 3.5; sub_outside.max_x = 3.9;
  sub_outside.min_y = 1.5; sub_outside.max_y = 1.9;
  sub_outside.min_z = 0.98; sub_outside.max_z = 1.02;
  sub_outside.centroid_x = 3.7; sub_outside.centroid_y = 1.7; sub_outside.centroid_z = 1.0;

  CranePipeline pipeline(cfg);
  // Prime ground indices via process on ground points
  pipeline.process(points, ground_indices, pose, false);

  std::vector<CraneSubcluster> raw_subs = {sub_loop, sub_inner, sub_outside};
  const auto & cl1 = pipeline.processClosedLoopGrouping(points, raw_subs);

  std::cout << "closed_loop frame1: detected_loops=" << cl1.detected_closed_loops
            << " subclusters=" << cl1.subclusters.size()
            << " groups=" << cl1.groups.size()
            << " grouping_ms=" << cl1.grouping_ms << std::endl;

  CHECK(cl1.detected_closed_loops >= 1);
  CHECK(cl1.subclusters.size() == 3);
  // Outer loop (z=1.0) and inner floating cluster (z=2.5) must be grouped into 1 unified ObjectGroup,
  // while outside cluster remains separate -> total 2 groups!
  CHECK(cl1.groups.size() == 2);

  const CraneObjectGroup * merged_group = nullptr;
  for (const auto & g : cl1.groups) {
    if (g.subclusters.size() == 2) {
      merged_group = &g;
    }
  }
  CHECK(merged_group != nullptr);
  CHECK(merged_group->is_closed_loop);
  CHECK(merged_group->subclusters.size() == 2);
  CHECK(merged_group->merged_point_indices.size() ==
        sub_loop.point_indices.size() + sub_inner.point_indices.size());

  // Frame 2: inner subcluster moves up by +0.20 m while background & outer loop stay stationary
  for (int idx : inner_point_indices) {
    points[static_cast<size_t>(idx)].z += 0.20f;
  }
  raw_subs[1].min_z += 0.20;
  raw_subs[1].max_z += 0.20;
  raw_subs[1].centroid_z += 0.20;

  const auto & cl2 = pipeline.processClosedLoopGrouping(points, raw_subs);
  CHECK(cl2.groups.size() == 2);

  bool found_moving_inner = false;
  bool found_static_loop = false;
  for (const auto & s : cl2.subclusters) {
    if (s.point_indices.size() == sub_inner.point_indices.size()) {
      CHECK(s.motion_valid);
      CHECK(std::abs(s.dz_relative - 0.20) < 1e-4);
      found_moving_inner = true;
    } else if (s.point_indices.size() == sub_loop.point_indices.size()) {
      CHECK(s.motion_valid);
      CHECK(std::abs(s.dz_relative) < 1e-4);
      found_static_loop = true;
    }
  }
  CHECK(found_moving_inner && found_static_loop);
  std::cout << "closed_loop test passed successfully!\n";
}

static void craneRoiFsmIntegration() {
  const auto pose = installation();
  CraneSlopeConfig cfg;
  cfg.z_gaussian.enabled = true;
  cfg.closed_loop.enabled = true;
  cfg.roi_filter.enabled = true;
  cfg.roi_filter.beam_roi_x_min = -2.0;
  cfg.roi_filter.beam_roi_x_max = 2.0;
  cfg.roi_filter.beam_roi_y_min = -2.0;
  cfg.roi_filter.beam_roi_y_max = 2.0;
  cfg.roi_filter.beam_roi_z_min = -0.5;
  cfg.roi_filter.beam_roi_z_max = 7.5;

  cfg.roi_filter.bg_roi_x_min = -8.0;
  cfg.roi_filter.bg_roi_x_max = 8.0;
  cfg.roi_filter.bg_roi_y_min = -8.0;
  cfg.roi_filter.bg_roi_y_max = 8.0;
  cfg.roi_filter.bg_roi_z_min = -0.5;
  cfg.roi_filter.bg_roi_z_max = 7.5;
  cfg.roi_filter.active_include_background = false;
  cfg.roi_filter.current_state = CraneBeamState::IDLE;

  // Scene with:
  // - 1 object inside Background ROI (at x=5.0, y=5.0, z=0.5) [outside Crane ROI]
  // - 1 object inside Crane ROI (at x=0.0, y=0.0, z=0.5)
  // - 1 object outside both ROIs (at x=12.0, y=12.0, z=0.5)
  std::vector<PointXYZIRT> points;
  std::vector<int> bg_indices;
  std::vector<int> beam_indices;
  std::vector<int> out_indices;

  for (int ix = 0; ix < 15; ++ix) {
    for (int iy = 0; iy < 15; ++iy) {
      // Background ROI object: centered at (5.0, 5.0, 0.5)
      PointXYZIRT p_bg{};
      p_bg.x = 4.8f + 0.02f * ix;
      p_bg.y = 4.8f + 0.02f * iy;
      p_bg.z = 0.5f;
      bg_indices.push_back(static_cast<int>(points.size()));
      points.push_back(p_bg);

      // Crane ROI object: centered at (0.0, 0.0, 0.5)
      PointXYZIRT p_crane{};
      p_crane.x = -0.1f + 0.02f * ix;
      p_crane.y = -0.1f + 0.02f * iy;
      p_crane.z = 0.5f;
      beam_indices.push_back(static_cast<int>(points.size()));
      points.push_back(p_crane);

      // Outside both ROIs object: centered at (12.0, 12.0, 0.5)
      PointXYZIRT p_out{};
      p_out.x = 11.8f + 0.02f * ix;
      p_out.y = 11.8f + 0.02f * iy;
      p_out.z = 0.5f;
      out_indices.push_back(static_cast<int>(points.size()));
      points.push_back(p_out);
    }
  }

  // 0) Test with only_crane_roi = true (Strict Crane ROI only mode, ignoring state)
  cfg.roi_filter.only_crane_roi = true;
  CranePipeline pipeline_only_crane(cfg);
  auto r_only_crane = pipeline_only_crane.process(points, allIndices(points.size()), pose, false);
  CHECK(r_only_crane.z_gaussian.candidate_points == beam_indices.size());
  for (const auto & layer : r_only_crane.z_gaussian.layers) {
    for (int idx : layer.point_indices) {
      CHECK(points[idx].x >= -2.0f && points[idx].x <= 2.0f);
      CHECK(points[idx].y >= -2.0f && points[idx].y <= 2.0f);
    }
  }

  // 1) Test with only_crane_roi = false in IDLE State (Background ROI encompasses entire [-8, 8] including center)
  cfg.roi_filter.only_crane_roi = false;
  CranePipeline pipeline(cfg);
  auto r_idle = pipeline.process(points, allIndices(points.size()), pose, false);

  // In IDLE State:
  // Points outside Background ROI (p_out) MUST NOT be in candidate points
  // Points inside Background ROI (both p_bg and p_crane) are clustered!
  CHECK(r_idle.z_gaussian.candidate_points == bg_indices.size() + beam_indices.size());
  for (const auto & layer : r_idle.z_gaussian.layers) {
    for (int idx : layer.point_indices) {
      CHECK(points[idx].x <= 8.0f && points[idx].x >= -8.0f);
      CHECK(points[idx].y <= 8.0f && points[idx].y >= -8.0f);
    }
  }

  // 1-b) Test with idle_exclude_beam_roi = true
  cfg.roi_filter.idle_exclude_beam_roi = true;
  pipeline.updateConfig(cfg);
  auto r_idle_exclude = pipeline.process(points, allIndices(points.size()), pose, false);
  CHECK(r_idle_exclude.z_gaussian.candidate_points == bg_indices.size());
  cfg.roi_filter.idle_exclude_beam_roi = false;
  pipeline.updateConfig(cfg);

  // 2) Test transition to LIFTING State
  cfg.roi_filter.current_state = CraneBeamState::LIFTING;
  pipeline.updateConfig(cfg);
  auto r_lifting = pipeline.process(points, allIndices(points.size()), pose, false);

  // In LIFTING State (with active_include_background = false):
  // Only Crane ROI points (p_crane) should be candidates!
  CHECK(r_lifting.z_gaussian.candidate_points == beam_indices.size());
  for (const auto & layer : r_lifting.z_gaussian.layers) {
    for (int idx : layer.point_indices) {
      CHECK(points[idx].x >= -1.0f && points[idx].x <= 1.0f);
      CHECK(points[idx].y >= -1.0f && points[idx].y <= 1.0f);
    }
  }

  // 3) Test transition to LIFTING State with active_include_background = true
  cfg.roi_filter.active_include_background = true;
  pipeline.updateConfig(cfg);
  auto r_both = pipeline.process(points, allIndices(points.size()), pose, false);
  CHECK(r_both.z_gaussian.candidate_points == bg_indices.size() + beam_indices.size());

  // 4) Test external_bg_z and external_dz_bg injection into processClosedLoopGrouping
  std::vector<CraneSubcluster> dummy_subs;
  CraneSubcluster s1;
  s1.subcluster_id = 1;
  s1.layer_id = 1;
  s1.point_indices = beam_indices;
  s1.centroid_x = 0.0; s1.centroid_y = 0.0; s1.centroid_z = 0.5;
  s1.min_x = -0.2; s1.max_x = 0.2;
  s1.min_y = -0.2; s1.max_y = 0.2;
  s1.min_z = 0.45; s1.max_z = 0.55;
  dummy_subs.push_back(s1);

  // Frame 1 with external background dz = 0.05
  const auto & cl_res1 = pipeline.processClosedLoopGrouping(points, dummy_subs, 0.10, 0.05);
  CHECK(std::abs(cl_res1.dz_background - 0.05) < 1e-6);

  // Frame 2 with external background dz = 0.08, and cluster moved by dz = 0.12
  dummy_subs[0].centroid_z += 0.12;
  dummy_subs[0].min_z += 0.12;
  dummy_subs[0].max_z += 0.12;
  for (int idx : beam_indices) {
    points[idx].z += 0.12f;
  }
  const auto & cl_res2 = pipeline.processClosedLoopGrouping(points, dummy_subs, 0.18, 0.08);
  CHECK(std::abs(cl_res2.dz_background - 0.08) < 1e-6);
  CHECK(cl_res2.subclusters.size() == 1);
  CHECK(cl_res2.subclusters[0].motion_valid);
  // dz_relative = dz_i - dz_background = 0.12 - 0.08 = 0.04
  CHECK(std::abs(cl_res2.subclusters[0].dz_relative - 0.04) < 1e-4);

  std::cout << "craneRoiFsmIntegration test passed successfully!\n";
}

static void hookAndBeamDetection() {
  CraneSlopeConfig cfg;
  cfg.work_status.fsm_enabled=false; // Observational fixture; field FSM is tested independently.
  cfg.ground_z = 0.0;
  cfg.hook.enabled = true;
  cfg.hook.roi_left_x_min = -1.5;
  cfg.hook.roi_left_x_max = -0.3;
  cfg.hook.roi_right_x_min = 0.3;
  cfg.hook.roi_right_x_max = 1.5;
  cfg.hook.roi_y_min = -0.8;
  cfg.hook.roi_y_max = 0.8;
  cfg.hook.roi_z_min = 3.0;
  cfg.hook.prepare_distance = 4.0;

  CranePipeline pipeline(cfg);

  // Synthesize points
  std::vector<PointXYZIRT> points;
  std::vector<CraneSubcluster> subclusters;

  // 1. Beam: rectangular box centered at (0.0, 0.0, 0.45)
  CraneSubcluster beam_sc;
  beam_sc.subcluster_id = 1;
  beam_sc.group_id = 1;
  beam_sc.centroid_x = 0.0; beam_sc.centroid_y = 0.0; beam_sc.centroid_z = 0.45;
  beam_sc.min_x = -1.4; beam_sc.max_x = 1.4;
  beam_sc.min_y = -0.15; beam_sc.max_y = 0.15;
  beam_sc.min_z = 0.35; beam_sc.max_z = 0.55;
  for (int i = 0; i < 28; ++i) {
    for (int j = 0; j < 4; ++j) {
      PointXYZIRT p{};
      p.x = -1.35f + 0.1f * i;
      p.y = -0.15f + 0.1f * j;
      p.z = 0.45f;
      beam_sc.point_indices.push_back(static_cast<int>(points.size()));
      points.push_back(p);
    }
  }
  subclusters.push_back(beam_sc);

  // 2. Left Root & Hook subclusters
  // Left Root (trolley/boom): (-1.0, 0.0, 5.4)
  CraneSubcluster left_root_sc;
  left_root_sc.subcluster_id = 2;
  left_root_sc.group_id = 2;
  left_root_sc.centroid_x = -1.0; left_root_sc.centroid_y = 0.0; left_root_sc.centroid_z = 5.4;
  left_root_sc.min_x = -1.15; left_root_sc.max_x = -0.85;
  left_root_sc.min_y = -0.15; left_root_sc.max_y = 0.15;
  left_root_sc.min_z = 5.3; left_root_sc.max_z = 5.5;
  for (int i = 0; i < 20; ++i) {
    PointXYZIRT p{}; p.x = -1.0f; p.y = 0.0f; p.z = 5.4f;
    left_root_sc.point_indices.push_back(static_cast<int>(points.size()));
    points.push_back(p);
  }
  subclusters.push_back(left_root_sc);

  // Left Hook (block): (-1.0, 0.0, 4.2)
  CraneSubcluster left_hook_sc;
  left_hook_sc.subcluster_id = 3;
  left_hook_sc.group_id = 2;
  left_hook_sc.centroid_x = -1.0; left_hook_sc.centroid_y = 0.0; left_hook_sc.centroid_z = 4.2;
  left_hook_sc.min_x = -1.15; left_hook_sc.max_x = -0.85;
  left_hook_sc.min_y = -0.15; left_hook_sc.max_y = 0.15;
  left_hook_sc.min_z = 4.05; left_hook_sc.max_z = 4.35;
  for (int i = 0; i < 30; ++i) {
    PointXYZIRT p{}; p.x = -1.0f; p.y = 0.0f; p.z = 4.2f;
    left_hook_sc.point_indices.push_back(static_cast<int>(points.size()));
    points.push_back(p);
  }
  subclusters.push_back(left_hook_sc);

  // 3. Right Root & Hook subclusters
  // Right Root (trolley/boom): (1.0, 0.0, 5.4)
  CraneSubcluster right_root_sc;
  right_root_sc.subcluster_id = 4;
  right_root_sc.group_id = 3;
  right_root_sc.centroid_x = 1.0; right_root_sc.centroid_y = 0.0; right_root_sc.centroid_z = 5.4;
  right_root_sc.min_x = 0.85; right_root_sc.max_x = 1.15;
  right_root_sc.min_y = -0.15; right_root_sc.max_y = 0.15;
  right_root_sc.min_z = 5.3; right_root_sc.max_z = 5.5;
  for (int i = 0; i < 20; ++i) {
    PointXYZIRT p{}; p.x = 1.0f; p.y = 0.0f; p.z = 5.4f;
    right_root_sc.point_indices.push_back(static_cast<int>(points.size()));
    points.push_back(p);
  }
  subclusters.push_back(right_root_sc);

  // Right Hook (block): (1.0, 0.0, 4.2)
  CraneSubcluster right_hook_sc;
  right_hook_sc.subcluster_id = 5;
  right_hook_sc.group_id = 3;
  right_hook_sc.centroid_x = 1.0; right_hook_sc.centroid_y = 0.0; right_hook_sc.centroid_z = 4.2;
  right_hook_sc.min_x = 0.85; right_hook_sc.max_x = 1.15;
  right_hook_sc.min_y = -0.15; right_hook_sc.max_y = 0.15;
  right_hook_sc.min_z = 4.05; right_hook_sc.max_z = 4.35;
  for (int i = 0; i < 30; ++i) {
    PointXYZIRT p{}; p.x = 1.0f; p.y = 0.0f; p.z = 4.2f;
    right_hook_sc.point_indices.push_back(static_cast<int>(points.size()));
    points.push_back(p);
  }
  subclusters.push_back(right_hook_sc);

  // Frame 1: Initial detection -> Top-down directed graph builds 2 chains & finds nearest cluster below each hook
  const auto & res1 = pipeline.processHookAndBeamDetection(points, subclusters, 1.0);
  CHECK(!res1.beam.detected);
  CHECK(!res1.work_status.work_status);
  CHECK(res1.working_status == 0);
  CHECK(res1.work_status.status_reason == "NONE");

  // Check top-down hook chains
  const auto & td1 = res1.top_down_hook;
  CHECK(td1.chains.size() == 2);
  CHECK(td1.left_detected && td1.right_detected);
  CHECK(td1.left_nearest_cluster.valid);
  CHECK(td1.right_nearest_cluster.valid);
  CHECK(td1.left_nearest_cluster.subcluster_id == 1);
  CHECK(td1.right_nearest_cluster.subcluster_id == 1);
  CHECK(td1.left_nearest_cluster.length > 2.5 && td1.left_nearest_cluster.length < 3.0);
  const auto & left_chain1 = td1.chains[td1.left_chain_idx];
  CHECK(left_chain1.side == "LEFT");
  CHECK(left_chain1.state == CraneHookConnectionState::CANDIDATE);
  CHECK(left_chain1.root_subcluster_id == 2);
  CHECK(left_chain1.lowest_subcluster_id == 3);
  CHECK(left_chain1.nodes.size() == 2);
  CHECK(left_chain1.has_gap); // Gap between 5.4m and 4.2m is 1.2m > 0.8m
  CHECK(std::abs(left_chain1.lowest_z - 4.2) < 0.1);

  const auto & right_chain1 = td1.chains[td1.right_chain_idx];
  CHECK(right_chain1.side == "RIGHT");
  CHECK(right_chain1.state == CraneHookConnectionState::CANDIDATE);
  CHECK(right_chain1.root_subcluster_id == 4);
  CHECK(right_chain1.lowest_subcluster_id == 5);

  // Downstream compatibility check
  CHECK(res1.hook.left.detected);
  CHECK(std::abs(res1.hook.left.x - (-1.0)) < 0.1);
  CHECK(std::abs(res1.hook.left.z - 4.2) < 0.1);
  CHECK(res1.hook.left.tracking_state == "CANDIDATE");
  CHECK(res1.hook.left.ground_distance > 4.0);
  CHECK(!res1.hook.prepare_ready);

  // Frames 2..5: Feed 4 more frames with same positions -> promote to PERSISTENT (M=5 of N=10)
  for (int f = 2; f <= 5; ++f) {
    pipeline.processHookAndBeamDetection(points, subclusters, 1.0 + 0.1 * f);
  }
  const auto & res5 = pipeline.result();
  CHECK(res5.top_down_hook.chains[res5.top_down_hook.left_chain_idx].state == CraneHookConnectionState::PERSISTENT);
  CHECK(res5.top_down_hook.chains[res5.top_down_hook.right_chain_idx].state == CraneHookConnectionState::PERSISTENT);
  CHECK(res5.hook.left.tracking_state == "PERSISTENT");
  CHECK(res5.hook.right.tracking_state == "PERSISTENT");

  // Frame 6: Lower hooks to 3.6m -> prepare_ready triggers!
  subclusters[2].centroid_z = 3.6; // left hook
  subclusters[2].min_z = 3.45; subclusters[2].max_z = 3.75;
  subclusters[4].centroid_z = 3.6; // right hook
  subclusters[4].min_z = 3.45; subclusters[4].max_z = 3.75;

  const auto & res6 = pipeline.processHookAndBeamDetection(points, subclusters, 1.6);
  CHECK(res6.hook.left.detected);
  CHECK(res6.hook.right.detected);
  CHECK(res6.hook.mean_ground_distance < 4.0);
  CHECK(res6.hook.prepare_ready);
  CHECK(res6.top_down_hook.prepare_ready);
  CHECK(!res6.top_down_hook.crane_moving); // stationary so far

  // Frame 7: Move the topmost part (roots) in X axis by +0.12m -> crane_moving triggers!
  subclusters[1].centroid_x += 0.12; // left root
  subclusters[3].centroid_x += 0.12; // right root
  const auto & res7 = pipeline.processHookAndBeamDetection(points, subclusters, 1.7);
  CHECK(res7.top_down_hook.crane_moving);
  CHECK(res7.crane_moving);
  CHECK(res7.top_down_hook.crane_move_reason.find("START_ROOT") != std::string::npos);

  // Frames 8..13: Roots stop moving at new position -> crane_moving resets to false
  for (int f = 8; f <= 13; ++f) {
    pipeline.processHookAndBeamDetection(points, subclusters, 1.0 + 0.1 * f);
  }
  const auto & res13 = pipeline.result();
  CHECK(!res13.top_down_hook.crane_moving);
  CHECK(!res13.crane_moving);

  std::cout << "hookAndBeamDetection (Top-Down Directed Graph + Gap Tolerance + Temporal Consistency + Root XY Motion) test passed successfully!\n";
}

static void hookNearestClusterBelowDetection() {
  CraneSlopeConfig cfg;
  cfg.work_status.fsm_enabled=false; // Observational fixture; field FSM is tested independently.
  cfg.top_down_hook.enabled = true;
  cfg.top_down_hook.roi_z_min = 2.5;
  cfg.hook.enabled = true;
  cfg.ground_z = 0.0;
  CranePipeline pipeline(cfg);

  // 1. Unit Test for computeOccupiedArea
  {
    std::vector<PointXYZIRT> test_pts;
    std::vector<int> test_idx;
    CHECK(CranePipeline::computeOccupiedArea(test_idx, test_pts, 0.05) == 0.0);

    for (int gx = 0; gx < 4; ++gx) {
      for (int gy = 0; gy < 4; ++gy) {
        PointXYZIRT p{}; p.x = 0.05f * gx + 0.01f; p.y = 0.05f * gy + 0.01f; p.z = 1.0f;
        test_idx.push_back(static_cast<int>(test_pts.size()));
        test_pts.push_back(p);
      }
    }
    double grid_area = CranePipeline::computeOccupiedArea(test_idx, test_pts, 0.05);
    CHECK(std::abs(grid_area - 0.04) < 1e-6);
  }

  // 2. Scenario A: Empty Hooks (only Root & Hook Block, plus a tiny split wire fragment above the hook)
  // Verify Hook Chain nodes and split wire fragments are excluded, and nearest_cluster.valid == false
  std::vector<PointXYZIRT> points;
  std::vector<CraneSubcluster> subclusters;

  // Left Root: (-1.0, 0.0, 5.4)
  CraneSubcluster left_root;
  left_root.subcluster_id = 10; left_root.layer_id = 10; left_root.group_id = 1;
  left_root.centroid_x = -1.0; left_root.centroid_y = 0.0; left_root.centroid_z = 5.4;
  left_root.min_x = -1.1; left_root.max_x = -0.9;
  left_root.min_y = -0.1; left_root.max_y = 0.1;
  left_root.min_z = 5.3; left_root.max_z = 5.5;
  for (int i = 0; i < 20; ++i) {
    PointXYZIRT p{}; p.x = -1.0f; p.y = 0.0f; p.z = 5.4f;
    left_root.point_indices.push_back(static_cast<int>(points.size()));
    points.push_back(p);
  }
  subclusters.push_back(left_root);

  // Left Hook Block: (-1.0, 0.0, 3.0), bottom at 2.85m
  CraneSubcluster left_hook;
  left_hook.subcluster_id = 11; left_hook.layer_id = 8; left_hook.group_id = 1;
  left_hook.centroid_x = -1.0; left_hook.centroid_y = 0.0; left_hook.centroid_z = 3.0;
  left_hook.min_x = -1.1; left_hook.max_x = -0.9;
  left_hook.min_y = -0.1; left_hook.max_y = 0.1;
  left_hook.min_z = 2.85; left_hook.max_z = 3.15;
  for (int i = 0; i < 25; ++i) {
    PointXYZIRT p{}; p.x = -1.0f; p.y = 0.0f; p.z = 3.0f;
    left_hook.point_indices.push_back(static_cast<int>(points.size()));
    points.push_back(p);
  }
  subclusters.push_back(left_hook);

  // Right Root: (1.0, 0.0, 5.4)
  CraneSubcluster right_root = left_root;
  right_root.subcluster_id = 20; right_root.group_id = 2;
  right_root.centroid_x = 1.0; right_root.min_x = 0.9; right_root.max_x = 1.1;
  right_root.point_indices.clear();
  for (int i = 0; i < 20; ++i) {
    PointXYZIRT p{}; p.x = 1.0f; p.y = 0.0f; p.z = 5.4f;
    right_root.point_indices.push_back(static_cast<int>(points.size()));
    points.push_back(p);
  }
  subclusters.push_back(right_root);

  // Right Hook Block: (1.0, 0.0, 3.0), bottom at 2.85m
  CraneSubcluster right_hook = left_hook;
  right_hook.subcluster_id = 21; right_hook.group_id = 2;
  right_hook.centroid_x = 1.0; right_hook.min_x = 0.9; right_hook.max_x = 1.1;
  right_hook.point_indices.clear();
  for (int i = 0; i < 25; ++i) {
    PointXYZIRT p{}; p.x = 1.0f; p.y = 0.0f; p.z = 3.0f;
    right_hook.point_indices.push_back(static_cast<int>(points.size()));
    points.push_back(p);
  }
  subclusters.push_back(right_hook);

  const auto & res_empty = pipeline.processHookAndBeamDetection(points, subclusters, 1.0);
  CHECK(res_empty.hook.left.detected);
  CHECK(res_empty.hook.right.detected);
  CHECK(!res_empty.top_down_hook.left_nearest_cluster.valid);
  CHECK(!res_empty.top_down_hook.right_nearest_cluster.valid);
  CHECK(res_empty.working_status == 0);
  CHECK(res_empty.work_status.status_reason == "NONE");

  // 3. Scenario B: 3D Distance Criterion (XY + Z combined)
  // Add two clusters below Left Hook (-1.0, 0.0, bottom_z = 2.85):
  // - Cluster #30 (Floor cluster): directly under hook in XY (-1.0, 0.0), but far down in Z (z = 0.30m -> dZ = 2.55m, dXY = 0.00m)
  // - Cluster #31 (Upper cluster): slightly offset in XY (-0.88, 0.0 -> dXY = 0.12m), but close in Z below hook (z = 2.35m -> dZ = 0.50m)
  // Combined 3D distance must select #31, NOT #30!
  CraneSubcluster floor_sc;
  floor_sc.subcluster_id = 30; floor_sc.layer_id = 1; floor_sc.group_id = 3;
  floor_sc.centroid_x = -1.0; floor_sc.centroid_y = 0.0; floor_sc.centroid_z = 0.30;
  floor_sc.min_x = -1.15; floor_sc.max_x = -0.85;
  floor_sc.min_y = -0.15; floor_sc.max_y = 0.15;
  floor_sc.min_z = 0.20; floor_sc.max_z = 0.40;
  for (int i = 0; i < 20; ++i) {
    PointXYZIRT p{}; p.x = -1.0f; p.y = 0.0f; p.z = 0.30f;
    floor_sc.point_indices.push_back(static_cast<int>(points.size()));
    points.push_back(p);
  }
  subclusters.push_back(floor_sc);

  CraneSubcluster upper_sc;
  upper_sc.subcluster_id = 31; upper_sc.layer_id = 5; upper_sc.group_id = 4;
  upper_sc.centroid_x = -0.88; upper_sc.centroid_y = 0.0; upper_sc.centroid_z = 2.35;
  upper_sc.min_x = -0.98; upper_sc.max_x = -0.78;
  upper_sc.min_y = -0.15; upper_sc.max_y = 0.15;
  upper_sc.min_z = 2.25; upper_sc.max_z = 2.45;
  for (int i = 0; i < 20; ++i) {
    PointXYZIRT p{}; p.x = -0.88f; p.y = 0.0f; p.z = 2.35f;
    upper_sc.point_indices.push_back(static_cast<int>(points.size()));
    points.push_back(p);
  }
  subclusters.push_back(upper_sc);

  const auto & res_3d = pipeline.processHookAndBeamDetection(points, subclusters, 1.1);
  const auto & lnc = res_3d.top_down_hook.left_nearest_cluster;
  CHECK(lnc.valid);
  CHECK(lnc.subcluster_id == 31);
  CHECK(lnc.layer_id == 5);
  CHECK(std::abs(lnc.dxy - 0.12) < 0.02);
  CHECK(std::abs(lnc.dz - (2.85 - 2.35)) < 0.02);
  // Status remains unjudged (0 / "NONE")
  CHECK(res_3d.working_status == 0);
  CHECK(res_3d.work_status.status_reason == "NONE");

  std::cout << "hookNearestClusterBelowDetection (Hook Chain Exclusion + 3D XY/Z Distance Criterion + Non-judgmental Status) passed successfully!\n";
}

static void temporalStabilizationDetection() {
  CraneSlopeConfig cfg;
  cfg.top_down_hook.enabled = true;
  cfg.roi_filter.enabled = true;
  cfg.temporal_stabilization.enabled = true;
  cfg.temporal_stabilization.mode = "sliding_window";
  cfg.temporal_stabilization.window_size = 5;
  cfg.temporal_stabilization.voxel_size_xy = 0.04;
  cfg.temporal_stabilization.voxel_size_z = 0.03;
  cfg.temporal_stabilization.min_hit_count = 2;
  cfg.temporal_stabilization.temporal_stabilization_enabled = true;
  cfg.temporal_stabilization.stabilization_alpha = 0.20;
  cfg.temporal_stabilization.jump_tolerance_length = 0.50;
  cfg.temporal_stabilization.jump_tolerance_width = 0.25;
  cfg.temporal_stabilization.jump_confirm_frames = 3;
  cfg.temporal_stabilization.max_hold_missed_frames = 3;

  CranePipeline pipeline(cfg);

  // 1. Synthetic points: persistent cluster + transient noise
  std::vector<PointXYZIRT> frame1_pts;
  for (int i = 0; i < 50; ++i) {
    PointXYZIRT p{};
    p.x = 1.0f + 0.05f * i;
    p.y = 0.0f;
    p.z = 2.0f;
    frame1_pts.push_back(p);
  }
  std::vector<int> frame1_indices = allIndices(frame1_pts.size());

  // Frame 1: crane is stationary (crane_moving = false)
  pipeline.updateAccumulatedPoints(frame1_pts, frame1_indices, 0.0);
  CHECK(pipeline.hasAccumulatedPoints());
  CHECK(pipeline.accumulatedPoints().size() == frame1_pts.size());

  // Frame 2: same points + 1 transient noise point inside Crane ROI
  std::vector<PointXYZIRT> frame2_pts = frame1_pts;
  PointXYZIRT noise_pt{};
  noise_pt.x = 0.5f; noise_pt.y = 0.5f; noise_pt.z = 2.0f;
  frame2_pts.push_back(noise_pt);
  std::vector<int> frame2_indices = allIndices(frame2_pts.size());

  pipeline.updateAccumulatedPoints(frame2_pts, frame2_indices, 0.1);
  const auto & acc_pts = pipeline.accumulatedPoints();
  bool found_noise = false;
  for (const auto & pt : acc_pts) {
    if (std::hypot(pt.x - 0.5f, pt.y - 0.5f) < 0.1f) found_noise = true;
  }
  CHECK(!found_noise);
  CHECK(acc_pts.size() == 50);

  // 2. Motion Trigger: Instant Zero-Lag Reversion
  pipeline.setExternalCraneMoving(true);
  pipeline.updateAccumulatedPoints(frame2_pts, frame2_indices, 0.2);
  CHECK(!pipeline.hasAccumulatedPoints());
  CHECK(pipeline.accumulatedPoints().empty());

  pipeline.setExternalCraneMoving(false);
  pipeline.updateAccumulatedPoints(frame1_pts, frame1_indices, 0.3);
  CHECK(pipeline.hasAccumulatedPoints());

  std::cout << "temporalStabilizationDetection (Stationary Multi-frame Accumulation + Noise Rejection + Zero-lag Motion Trigger) test passed successfully!\n";
}

static CraneSlopeResult heightCase(const std::string & name,const std::vector<PointXYZIRT> & points,
    CraneHeightConfig config={true}) {
  CraneSlopeConfig cfg;cfg.height_mode=true;cfg.height=config;
  // These archived settings are intentionally invalid/unused in height mode.
  cfg.lidar_model="not_used_by_height";cfg.max_slope_deg=std::numeric_limits<double>::quiet_NaN();
  cfg.horizontal_enabled=false;
  VisibilityModel visibility;VisibilityConfig fov;visibility.configure(fov);
  CranePipeline pipeline(cfg);const auto r=pipeline.process(points,allIndices(points.size()),installation(),false,&visibility);
  const auto & h=r.height;
  CHECK(r.labels.empty() && r.ground.empty() && r.clustering.empty());
  CHECK(r.neighbor_pairs==0 && r.horizontal_neighbor_pairs==0 && r.state==CraneBeamState::IDLE);
  CHECK(h.candidates.size()+h.excluded.size()==points.size());
  CHECK(h.surface_points.size()+h.unassigned.size()==h.candidates.size());
  auto partition=h.surface_points;partition.insert(partition.end(),h.unassigned.begin(),h.unassigned.end());
  partition.insert(partition.end(),h.excluded.begin(),h.excluded.end());std::sort(partition.begin(),partition.end());
  CHECK(partition==allIndices(points.size()));
  for(const auto & a:h.surfaces) {
    CHECK(a.id>=0 && a.point_count>=static_cast<size_t>(config.min_surface_points));
    CHECK(a.max_z-a.min_z<=config.max_patch_z_spread+1e-6);
    CHECK(a.length>=config.min_surface_length && a.width>=config.min_surface_width);
    CHECK(a.patch_count>=1 && a.plane_rms<=config.max_plane_rms);
    for(size_t k=a.index_begin;k<a.index_begin+a.point_count;++k)CHECK(h.surface_ids[h.surface_points[k]]==a.id);
  }
  for(int i:h.unassigned)CHECK(h.surface_ids[i]<0);
  std::cout<<"height "<<name<<" input="<<points.size()<<" candidates="<<h.candidates.size()
      <<" surfaces="<<h.surfaces.size()<<" accepted="<<h.surface_points.size()<<" unassigned="<<h.unassigned.size()
      <<" excluded="<<h.excluded.size()<<" visibility_ms="<<h.visibility_ms<<" bins_ms="<<h.patch_ms
      <<" growing_ms="<<h.merging_ms<<" validation_ms="<<h.finalization_ms<<" total_ms="<<h.total_ms
      <<" neighbor_tests="<<h.neighbor_tests<<std::endl;
  return r;
}
static void heightDetection() {
  const auto pose=installation();
  auto floor=planeScene({0,0,1},{0,0,0});auto r=heightCase("floor",floor);
  CHECK(r.height.surfaces.size()==1 && r.height.surface_points.size()>floor.size()*9/10);
  CHECK(std::abs(r.height.surfaces[0].center_z)<1e-5);
  auto beam=planeScene({0,0,1},{0,0,2});r=heightCase("elevated_beam",beam);
  CHECK(r.height.surfaces.size()==1 && std::abs(r.height.surfaces[0].center_z-2)<1e-5);
  auto both=floor;both.insert(both.end(),beam.begin(),beam.end());r=heightCase("overlapping_xy_two_heights",both);
  CHECK(r.height.surfaces.size()==2 && r.height.occlusion_flagged_points>0);
  CHECK(std::abs(r.height.surfaces[0].center_z-r.height.surfaces[1].center_z)>1.9);
  auto left=planeScene({0,0,1},{0,0,1},40,45.05);auto right=planeScene({0,0,1},{0,0,1},40,100.05);
  auto separated=left;separated.insert(separated.end(),right.begin(),right.end());r=heightCase("same_height_separated_xy",separated);
  CHECK(r.height.surfaces.size()==2);
  CHECK(std::abs(r.height.surfaces[0].center_z-r.height.surfaces[1].center_z)<1e-5);
  CHECK(CranePipeline::heightColor(r.height.surfaces[0].id)!=CranePipeline::heightColor(r.height.surfaces[1].id));
  std::vector<PointXYZIRT> narrow;
  for(int col=0;col<240;++col)for(int ring=0;ring<128;++ring) {
    const double az=45.05+.2*col;
    if(std::abs(driverElevation(ring,az))<=.4)narrow.push_back(planeRay(pose,az,ring,{0,0,1},{0,0,1}));
  }
  r=heightCase("narrow_long_beam",narrow);CHECK(r.height.surfaces.size()==1 && r.height.surface_points.size()>narrow.size()*9/10);
  CHECK(r.height.surfaces[0].length>10*r.height.surfaces[0].width);
  auto noisy=beam;std::mt19937 rng(72);std::normal_distribution<float> noise(0,.008);
  for(auto & p:noisy)p.z+=std::clamp(noise(rng),-.025f,.025f);
  r=heightCase("height_noise",noisy);CHECK(r.height.surface_points.size()>r.height.candidates.size()*9/10);
  auto boundary=planeScene({0,0,1},{0,0,.05});
  for(size_t i=0;i<boundary.size();++i)boundary[i].z+=(i%2?-.001f:.001f);
  r=heightCase("z_bin_boundary",boundary);CHECK(r.height.surfaces.size()==1);
  auto stairs=beam;
  for(size_t i=0;i<stairs.size();++i)stairs[i]=planeRay(pose,65.05+.1*(i/128),stairs[i].ring,{0,0,1},{0,0,.03*((i/128)/8)});
  r=heightCase("stairs_accumulation_guard",stairs);CHECK(r.height.surfaces.size()>=3);
  for(const auto & a:r.height.surfaces)CHECK(a.max_z-a.min_z<=.080001);
  std::vector<PointXYZIRT> corner;
  for(int col=0;col<160;++col)for(int ring=0;ring<128;++ring) {
    const double az=60.05+.1*col;
    auto a=planeRay(pose,az,ring,{0,0,1},{0,0,0});auto b=planeRay(pose,az,ring,{1,0,0},{2.4,0,0});
    if((tf2::Vector3(a.x,a.y,a.z)-pose.getOrigin()).length2()>(tf2::Vector3(b.x,b.y,b.z)-pose.getOrigin()).length2())a=b;
    corner.push_back(a);
  }
  r=heightCase("wall_meets_floor",corner);CHECK(!r.height.surfaces.empty());
  for(const auto & a:r.height.surfaces)CHECK(a.max_z<=.080001);
  auto wall=planeScene({1,0,0},{1.5,0,0});r=heightCase("vertical_wall",wall);CHECK(r.height.surfaces.empty());
  std::vector<PointXYZIRT> holes,uneven;
  for(size_t i=0;i<beam.size();++i) {
    const int col=i/128;
    if(!(col>=25&&col<50&&beam[i].ring>=45&&beam[i].ring<90))holes.push_back(beam[i]);
    if(col>=40 || col%3==0)uneven.push_back(beam[i]);
  }
  r=heightCase("partial_occlusion",holes);CHECK(r.height.surface_points.size()>r.height.candidates.size()*9/10);
  r=heightCase("nonuniform_density",uneven);CHECK(r.height.surfaces.size()==1 && r.height.surface_points.size()>r.height.candidates.size()*9/10);
  std::vector<PointXYZIRT> dense_point(1000,beam[40]);r=heightCase("dense_single_location",dense_point);CHECK(r.height.surfaces.empty());
  std::vector<PointXYZIRT> line;
  for(int col=0;col<120;++col)line.push_back(planeRay(pose,65.05+.1*col,68,{0,0,1},{0,0,1}));
  r=heightCase("one_dimensional_xy_line",line);CHECK(r.height.surfaces.empty());
  std::vector<PointXYZIRT> isolated{beam[0],beam[beam.size()/2],beam.back()};
  r=heightCase("isolated",isolated);CHECK(r.height.surfaces.empty());
  auto invalid=beam;invalid[0].x=std::numeric_limits<float>::quiet_NaN();r=heightCase("invalid_point",invalid);
  CHECK(std::find(r.height.excluded.begin(),r.height.excluded.end(),0)!=r.height.excluded.end());
  // Histogram origin changes must not split a connected real surface.
  CraneHeightConfig different_bins;different_bins.enabled=true;different_bins.patch_resolution=.03;
  r=heightCase("different_bin_size",boundary,different_bins);CHECK(r.height.surfaces.size()==1);
  // Historical Ground/Plane options cannot affect a height-only result.
  CraneSlopeConfig cfg;cfg.height_mode=true;cfg.height.enabled=true;CranePipeline a(cfg);
  cfg.force_ground_below_z=100;cfg.horizontal_enabled=true;CranePipeline b(cfg);
  VisibilityModel va,vb;VisibilityConfig fov;va.configure(fov);vb.configure(fov);
  const auto ra=a.process(both,allIndices(both.size()),pose,false,&va);
  const auto rb=b.process(both,allIndices(both.size()),pose,true,&vb);
  CHECK(ra.height.surface_ids==rb.height.surface_ids && ra.height.surface_points==rb.height.surface_points);
  a.reset();CHECK(a.result().height.surface_ids.empty());
  cfg.height.enabled=false;CranePipeline disabled(cfg);const auto rd=disabled.process(beam,allIndices(beam.size()),pose,false);
  CHECK(rd.height.surfaces.empty() && rd.labels.empty());
  for(int which=0;which<4;++which) {
    auto bad=cfg;bad.height.enabled=true;
    if(which==0)bad.height.patch_resolution=0;
    if(which==1)bad.height.min_surface_points=1;
    if(which==2)bad.height.max_patch_z_spread=std::numeric_limits<double>::quiet_NaN();
    if(which==3)bad.height.max_xy_gap=0;
    bool rejected=false;try{CranePipeline x(bad);}catch(const std::invalid_argument &){rejected=true;}CHECK(rejected);
  }
  // Warm-buffer stage timings on synthetic observations only; no rosbag/PCD IO.
  for(const auto & scene:std::vector<std::pair<std::string,const std::vector<PointXYZIRT> *>>{
      {"floor",&floor},{"beam",&beam},{"two_heights",&both}}) {
    CraneSlopeConfig timing;timing.height_mode=true;timing.height.enabled=true;
    CranePipeline pipeline(timing);VisibilityModel visibility;visibility.configure(fov);
    const auto indices=allIndices(scene.second->size());std::vector<double> totals;
    double visibility_ms=0,binning_ms=0,growing_ms=0,validation_ms=0;size_t workspace=0;
    for(int frame=0;frame<23;++frame) {
      const auto & frame_result=pipeline.process(*scene.second,indices,pose,false,&visibility);
      const auto & h=frame_result.height;
      if(frame<3)continue;
      visibility_ms+=h.visibility_ms;binning_ms+=h.patch_ms;growing_ms+=h.merging_ms;
      validation_ms+=h.finalization_ms;totals.push_back(h.total_ms);workspace=frame_result.workspace_bytes;
    }
    double total=0;for(double t:totals)total+=t;std::sort(totals.begin(),totals.end());
    std::cout<<"height_benchmark "<<scene.first<<" frames=20 input="<<scene.second->size()
        <<" visibility_ms="<<visibility_ms/20<<" bins_ms="<<binning_ms/20
        <<" growing_ms="<<growing_ms/20<<" validation_ms="<<validation_ms/20
        <<" mean_ms="<<total/20<<" p95_ms="<<totals[18]<<" vector_workspace_bytes="<<workspace<<std::endl;
  }
}

static void topDownHookLargestConnectedClusterTest() {
  CraneSlopeConfig cfg;
  cfg.work_status.fsm_enabled=false; // Observational fixture; field FSM is tested independently.
  cfg.top_down_hook.enabled = true;
  cfg.top_down_hook.roi_z_min = -0.5; // full downward descent
  cfg.hook.enabled = true;
  cfg.ground_z = 0.0;
  CranePipeline pipeline(cfg);

  std::vector<PointXYZIRT> points;
  std::vector<CraneSubcluster> subclusters;

  // 1. Left Root: (-1.0, 0.0, 5.4m)
  CraneSubcluster left_root;
  left_root.subcluster_id = 1; left_root.layer_id = 10; left_root.group_id = 1;
  left_root.centroid_x = -1.0; left_root.centroid_y = 0.0; left_root.centroid_z = 5.4;
  left_root.min_x = -1.1; left_root.max_x = -0.9;
  left_root.min_y = -0.1; left_root.max_y = 0.1;
  left_root.min_z = 5.3; left_root.max_z = 5.5;
  for (int i = 0; i < 20; ++i) {
    PointXYZIRT p{}; p.x = -1.0f; p.y = 0.0f; p.z = 5.4f;
    left_root.point_indices.push_back(static_cast<int>(points.size()));
    points.push_back(p);
  }
  subclusters.push_back(left_root);

  // 2. Left Cable segment: (-1.0, 0.0, 4.4m)
  CraneSubcluster left_cable;
  left_cable.subcluster_id = 2; left_cable.layer_id = 9; left_cable.group_id = 1;
  left_cable.centroid_x = -1.0; left_cable.centroid_y = 0.0; left_cable.centroid_z = 4.4;
  left_cable.min_x = -1.05; left_cable.max_x = -0.95;
  left_cable.min_y = -0.05; left_cable.max_y = 0.05;
  left_cable.min_z = 4.3; left_cable.max_z = 4.5;
  for (int i = 0; i < 20; ++i) {
    PointXYZIRT p{}; p.x = -1.0f; p.y = 0.0f; p.z = 4.4f;
    left_cable.point_indices.push_back(static_cast<int>(points.size()));
    points.push_back(p);
  }
  subclusters.push_back(left_cable);

  // 3. Left Hook Block: (-1.0, 0.0, 3.2m)
  CraneSubcluster left_hook;
  left_hook.subcluster_id = 3; left_hook.layer_id = 8; left_hook.group_id = 1;
  left_hook.centroid_x = -1.0; left_hook.centroid_y = 0.0; left_hook.centroid_z = 3.2;
  left_hook.min_x = -1.15; left_hook.max_x = -0.85;
  left_hook.min_y = -0.15; left_hook.max_y = 0.15;
  left_hook.min_z = 3.05; left_hook.max_z = 3.35;
  for (int i = 0; i < 30; ++i) {
    PointXYZIRT p{}; p.x = -1.0f; p.y = 0.0f; p.z = 3.2f;
    left_hook.point_indices.push_back(static_cast<int>(points.size()));
    points.push_back(p);
  }
  subclusters.push_back(left_hook);

  // 4. Shackle / sling node below hook: (-1.0, 0.0, 2.0m)
  CraneSubcluster left_shackle;
  left_shackle.subcluster_id = 4; left_shackle.layer_id = 7; left_shackle.group_id = 1;
  left_shackle.centroid_x = -1.0; left_shackle.centroid_y = 0.0; left_shackle.centroid_z = 2.0;
  left_shackle.min_x = -1.08; left_shackle.max_x = -0.92;
  left_shackle.min_y = -0.08; left_shackle.max_y = 0.08;
  left_shackle.min_z = 1.85; left_shackle.max_z = 2.15;
  for (int i = 0; i < 15; ++i) {
    PointXYZIRT p{}; p.x = -1.0f; p.y = 0.0f; p.z = 2.0f;
    left_shackle.point_indices.push_back(static_cast<int>(points.size()));
    points.push_back(p);
  }
  subclusters.push_back(left_shackle);

  // 5. Large Attached Beam: Centered at (0.0, 0.0, 0.50m), length 5.4m, width 0.38m
  // Spans along X from -2.7m to +2.7m; passes directly underneath Left Hook at x = -1.0m!
  CraneSubcluster beam_sc;
  beam_sc.subcluster_id = 5; beam_sc.layer_id = 4; beam_sc.group_id = 2;
  beam_sc.centroid_x = 0.0; beam_sc.centroid_y = 0.0; beam_sc.centroid_z = 0.50;
  beam_sc.min_x = -2.70; beam_sc.max_x = 2.70;
  beam_sc.min_y = -0.19; beam_sc.max_y = 0.19;
  beam_sc.min_z = 0.35; beam_sc.max_z = 0.65;
  for (int gx = 0; gx < 108; ++gx) {
    for (int gy = 0; gy < 8; ++gy) {
      PointXYZIRT p{};
      p.x = static_cast<float>(-2.675 + 0.05 * gx);
      p.y = static_cast<float>(-0.175 + 0.05 * gy);
      p.z = 0.50f;
      beam_sc.point_indices.push_back(static_cast<int>(points.size()));
      points.push_back(p);
    }
  }
  subclusters.push_back(beam_sc);

  // 6. Ground contact residue cluster below the beam: (-1.0, 0.0, 0.10m)
  CraneSubcluster ground_node;
  ground_node.subcluster_id = 6; ground_node.layer_id = 1; ground_node.group_id = 2;
  ground_node.centroid_x = -1.0; ground_node.centroid_y = 0.0; ground_node.centroid_z = 0.10;
  ground_node.min_x = -1.20; ground_node.max_x = -0.80;
  ground_node.min_y = -0.10; ground_node.max_y = 0.10;
  ground_node.min_z = 0.06; ground_node.max_z = 0.14;
  for (int i = 0; i < 15; ++i) {
    PointXYZIRT p{}; p.x = -1.0f; p.y = 0.0f; p.z = 0.10f;
    ground_node.point_indices.push_back(static_cast<int>(points.size()));
    points.push_back(p);
  }
  subclusters.push_back(ground_node);

  const auto & res = pipeline.processHookAndBeamDetection(points, subclusters, 1.0);

  // Verification 1: Chain exists and detected
  CHECK(res.top_down_hook.left_detected);
  const auto & left_chain = res.top_down_hook.chains[res.top_down_hook.left_chain_idx];
  CHECK(left_chain.side == "LEFT");
  CHECK(left_chain.root_subcluster_id == 1);

  CHECK(left_chain.nodes.size() == 6);
  CHECK(left_chain.lowest_subcluster_id == 6);
  CHECK(std::abs(left_chain.lowest_z - 0.10) < 0.05);

  // Verification 3: Gap link distinction
  CHECK(left_chain.nodes[3].is_estimated_gap); // shackle has dz = 1.2m > 0.8m
  CHECK(!left_chain.nodes[5].is_estimated_gap); // ground node has dz = 0.40m <= 0.8m -> direct link

  // Verification 4: Largest Connected Cluster is the Beam (subcluster_id == 5)
  const auto & largest = left_chain.largest_cluster;
  CHECK(largest.valid);
  CHECK(largest.subcluster_id == 5);
  CHECK(largest.layer_id == 4);
  CHECK(std::abs(largest.length - 5.40) < 0.10);
  CHECK(std::abs(largest.width - 0.38) < 0.10);
  CHECK(largest.area_xy > 1.80 && largest.area_xy < 2.20);
  CHECK(res.top_down_hook.left_largest_cluster.subcluster_id == 5);
  CHECK(std::abs(res.top_down_hook.left_largest_cluster.area_xy - largest.area_xy) < 1e-6);

  // Verification 5: Non-judgmental status
  CHECK(!res.beam.detected);
  CHECK(res.working_status == 0);
  CHECK(res.work_status.status_reason == "NONE");

  std::cout << "topDownHookLargestConnectedClusterTest (Full Descent to Ground + Gap Link Distinction + Largest Connected Cluster Area/Length/Width) passed successfully!\n";
}

static void topDownTemporalStabilizationTest() {
  CraneSlopeConfig cfg;
  cfg.top_down_hook.enabled = true;
  cfg.top_down_hook.roi_left_x_min = -1.50;
  cfg.top_down_hook.roi_left_x_max = -0.50;
  cfg.top_down_hook.roi_y_min = -0.50;
  cfg.top_down_hook.roi_y_max = 0.50;
  cfg.top_down_hook.roi_z_min = -0.50;
  cfg.top_down_hook.root_min_z = 4.80;
  cfg.top_down_hook.temporal_window_n = 5;
  cfg.top_down_hook.persistent_confirm_m = 2;
  cfg.top_down_hook.direct_link_max_gap_z = 0.80;
  cfg.top_down_hook.max_gap_z = 3.50;
  cfg.temporal_stabilization.enabled = true;
  cfg.temporal_stabilization.temporal_stabilization_enabled = true;
  cfg.temporal_stabilization.node_confirm_frames = 2;
  cfg.temporal_stabilization.node_max_hold_frames = 2;
  cfg.temporal_stabilization.largest_swap_confirm_frames = 3;
  cfg.temporal_stabilization.largest_match_dist = 0.50;
  cfg.temporal_stabilization.yaw_jump_tolerance_deg = 30.0;
  cfg.temporal_stabilization.yaw_alpha = 0.15;

  CranePipeline pipeline(cfg);

  // Helper to build standard frame
  auto make_frame = [](bool include_ground, double beam_metric_boost, double new_bot_node_z) {
    std::vector<PointXYZIRT> points;
    std::vector<CraneSubcluster> subclusters;

    // 1. Root
    CraneSubcluster root;
    root.subcluster_id = 1; root.layer_id = 10;
    root.centroid_x = -1.0; root.centroid_y = 0.0; root.centroid_z = 5.20;
    root.min_x = -1.15; root.max_x = -0.85; root.min_y = -0.15; root.max_y = 0.15;
    root.min_z = 5.05; root.max_z = 5.35;
    for (int i = 0; i < 20; ++i) { PointXYZIRT p{}; p.x = -1.0f; p.y = 0.0f; p.z = 5.20f; root.point_indices.push_back(points.size()); points.push_back(p); }
    subclusters.push_back(root);

    // 2. Chain upper
    CraneSubcluster c1;
    c1.subcluster_id = 2; c1.layer_id = 8;
    c1.centroid_x = -1.0; c1.centroid_y = 0.0; c1.centroid_z = 4.40;
    c1.min_x = -1.10; c1.max_x = -0.90; c1.min_y = -0.10; c1.max_y = 0.10;
    c1.min_z = 4.30; c1.max_z = 4.50;
    for (int i = 0; i < 15; ++i) { PointXYZIRT p{}; p.x = -1.0f; p.y = 0.0f; p.z = 4.40f; c1.point_indices.push_back(points.size()); points.push_back(p); }
    subclusters.push_back(c1);

    // 3. Chain mid
    CraneSubcluster c2;
    c2.subcluster_id = 3; c2.layer_id = 6;
    c2.centroid_x = -1.0; c2.centroid_y = 0.0; c2.centroid_z = 3.60;
    c2.min_x = -1.10; c2.max_x = -0.90; c2.min_y = -0.10; c2.max_y = 0.10;
    c2.min_z = 3.50; c2.max_z = 3.70;
    for (int i = 0; i < 15; ++i) { PointXYZIRT p{}; p.x = -1.0f; p.y = 0.0f; p.z = 3.60f; c2.point_indices.push_back(points.size()); points.push_back(p); }
    subclusters.push_back(c2);

    // 4. Beam (Largest Cluster): L=5.4m, W=0.38m, Z=0.5m
    CraneSubcluster beam;
    beam.subcluster_id = 4; beam.layer_id = 3;
    beam.centroid_x = 0.0; beam.centroid_y = 0.0; beam.centroid_z = 0.50;
    beam.min_x = -2.70; beam.max_x = 2.70; beam.min_y = -0.19; beam.max_y = 0.19;
    beam.min_z = 0.35; beam.max_z = 0.65;
    for (int gx = 0; gx < 108; ++gx) {
      for (int gy = 0; gy < 8; ++gy) {
        PointXYZIRT p{};
        p.x = static_cast<float>(-2.675 + 0.05 * gx);
        p.y = static_cast<float>(-0.175 + 0.05 * gy);
        p.z = 0.50f;
        beam.point_indices.push_back(points.size());
        points.push_back(p);
      }
    }
    subclusters.push_back(beam);

    // Optional fluctuating cluster with slightly higher metric
    if (beam_metric_boost > 0.0) {
      CraneSubcluster rival;
      rival.subcluster_id = 99; rival.layer_id = 5;
      rival.centroid_x = -1.0; rival.centroid_y = 0.0; rival.centroid_z = 2.0;
      rival.min_x = -1.8; rival.max_x = -0.2; rival.min_y = -0.7; rival.max_y = 0.7;
      rival.min_z = 1.9; rival.max_z = 2.1;
      for (int gx = 0; gx < 32; ++gx) {
        for (int gy = 0; gy < 28; ++gy) {
          PointXYZIRT p{}; p.x = -1.0f + 0.05f * gx; p.y = 0.05f * gy; p.z = 2.0f;
          rival.point_indices.push_back(points.size()); points.push_back(p);
        }
      }
      subclusters.push_back(rival);
    }

    // 5. Ground node
    if (include_ground) {
      CraneSubcluster gnd;
      gnd.subcluster_id = 5; gnd.layer_id = 1;
      gnd.centroid_x = -1.0; gnd.centroid_y = 0.0; gnd.centroid_z = 0.10;
      gnd.min_x = -1.20; gnd.max_x = -0.80; gnd.min_y = -0.10; gnd.max_y = 0.10;
      gnd.min_z = 0.06; gnd.max_z = 0.14;
      for (int i = 0; i < 15; ++i) { PointXYZIRT p{}; p.x = -1.0f; p.y = 0.0f; p.z = 0.10f; gnd.point_indices.push_back(points.size()); points.push_back(p); }
      subclusters.push_back(gnd);
    }

    // 6. Transient bottom node
    if (new_bot_node_z > -10.0) {
      CraneSubcluster bot;
      bot.subcluster_id = 88; bot.layer_id = 1;
      bot.centroid_x = -1.0; bot.centroid_y = 0.0; bot.centroid_z = new_bot_node_z;
      bot.min_x = -1.1; bot.max_x = -0.9; bot.min_y = -0.1; bot.max_y = 0.1;
      bot.min_z = new_bot_node_z - 0.05; bot.max_z = new_bot_node_z + 0.05;
      for (int i = 0; i < 10; ++i) { PointXYZIRT p{}; p.x = -1.0f; p.y = 0.0f; p.z = static_cast<float>(new_bot_node_z); bot.point_indices.push_back(points.size()); points.push_back(p); }
      subclusters.push_back(bot);
    }

    return std::make_pair(points, subclusters);
  };

  // Frame 1: Initial full observation (5 nodes: Root, C1, C2, Beam, Ground)
  auto [pts1, subs1] = make_frame(true, 0.0, -99.0);
  auto res1 = pipeline.processHookAndBeamDetection(pts1, subs1, 1.0);
  CHECK(res1.top_down_hook.left_detected);
  const auto & chain1 = res1.top_down_hook.chains[res1.top_down_hook.left_chain_idx];
  CHECK(chain1.nodes.size() == 5);
  CHECK(chain1.lowest_subcluster_id == 5);
  CHECK(chain1.largest_cluster.subcluster_id == 4);
  CHECK(!chain1.is_held);

  // Frame 2: Node 5 (ground) drops for 1 frame
  auto [pts2, subs2] = make_frame(false, 0.0, -99.0);
  auto res2 = pipeline.processHookAndBeamDetection(pts2, subs2, 1.1);
  const auto & chain2 = res2.top_down_hook.chains[res2.top_down_hook.left_chain_idx];
  // Temporal Stabilization: Terminal does NOT jump up! Node 5 is held!
  CHECK(chain2.nodes.size() == 5);
  CHECK(chain2.nodes.back().is_held);
  CHECK(chain2.nodes.back().is_estimated_gap);
  CHECK(chain2.is_held);
  CHECK(chain2.largest_cluster.subcluster_id == 4);

  // Frame 3: Fluctuating rival cluster with slightly larger area appears for 1 frame
  auto [pts3, subs3] = make_frame(true, 1.0, -99.0);
  auto res3 = pipeline.processHookAndBeamDetection(pts3, subs3, 1.2);
  const auto & chain3 = res3.top_down_hook.chains[res3.top_down_hook.left_chain_idx];
  // Temporal Stabilization: Largest Cluster maintains identity (Beam id == 4), does NOT chatter!
  CHECK(chain3.largest_cluster.subcluster_id == 4);

  // Frame 4: Transient bottom node 88 appears for only 1 frame
  auto [pts4, subs4] = make_frame(true, 0.0, 0.02);
  auto res4 = pipeline.processHookAndBeamDetection(pts4, subs4, 1.3);
  const auto & chain4 = res4.top_down_hook.chains[res4.top_down_hook.left_chain_idx];
  // Node 88 is pending (seen only 1 frame < node_confirm_frames=2), NOT confirmed yet!
  CHECK(chain4.lowest_subcluster_id == 5);

  std::cout << "topDownTemporalStabilizationTest (Node Hold on Dropout + Anti-Chattering Hysteresis + Transient Node Rejection) passed successfully!\n";
}

static void topDownSlopeAndTurnAngleLimitTest() {
  CraneSlopeConfig cfg;
  cfg.top_down_hook.enabled = true;
  cfg.top_down_hook.roi_left_x_min = -1.50;
  cfg.top_down_hook.roi_left_x_max = -0.50;
  cfg.top_down_hook.roi_y_min = -0.50;
  cfg.top_down_hook.roi_y_max = 0.50;
  cfg.top_down_hook.roi_z_min = -0.50;
  cfg.top_down_hook.root_min_z = 4.80;
  cfg.top_down_hook.direct_link_max_gap_z = 0.80;
  cfg.top_down_hook.max_gap_z = 2.50;
  cfg.top_down_hook.max_slope_angle_deg = 35.0; // 35 deg limit
  cfg.top_down_hook.max_turn_angle_deg = 40.0;  // 40 deg limit
  cfg.temporal_stabilization.enabled = true;
  cfg.temporal_stabilization.temporal_stabilization_enabled = true;

  CranePipeline pipeline(cfg);

  std::vector<PointXYZIRT> points;
  std::vector<CraneSubcluster> subclusters;

  // 1. Root: (-1.0, 0.0, 5.2m)
  CraneSubcluster root;
  root.subcluster_id = 1; root.layer_id = 10;
  root.centroid_x = -1.0; root.centroid_y = 0.0; root.centroid_z = 5.20;
  root.min_x = -1.15; root.max_x = -0.85; root.min_y = -0.15; root.max_y = 0.15;
  root.min_z = 5.05; root.max_z = 5.35;
  for (int i = 0; i < 20; ++i) { PointXYZIRT p{}; p.x = -1.0f; p.y = 0.0f; p.z = 5.20f; root.point_indices.push_back(points.size()); points.push_back(p); }
  subclusters.push_back(root);

  // 2. Cable: (-1.0, 0.0, 4.4m)
  CraneSubcluster cable;
  cable.subcluster_id = 2; cable.layer_id = 8;
  cable.centroid_x = -1.0; cable.centroid_y = 0.0; cable.centroid_z = 4.40;
  cable.min_x = -1.10; cable.max_x = -0.90; cable.min_y = -0.10; cable.max_y = 0.10;
  cable.min_z = 4.30; cable.max_z = 4.50;
  for (int i = 0; i < 15; ++i) { PointXYZIRT p{}; p.x = -1.0f; p.y = 0.0f; p.z = 4.40f; cable.point_indices.push_back(points.size()); points.push_back(p); }
  subclusters.push_back(cable);

  // 3. Hook Block: (-0.98, 0.01, 3.6m) - slight natural sway (slope ~1.0 deg)
  CraneSubcluster hook;
  hook.subcluster_id = 3; hook.layer_id = 6;
  hook.centroid_x = -0.98; hook.centroid_y = 0.01; hook.centroid_z = 3.60;
  hook.min_x = -1.10; hook.max_x = -0.86; hook.min_y = -0.10; hook.max_y = 0.12;
  hook.min_z = 3.45; hook.max_z = 3.75;
  for (int i = 0; i < 25; ++i) { PointXYZIRT p{}; p.x = -0.98f; p.y = 0.01f; p.z = 3.60f; hook.point_indices.push_back(points.size()); points.push_back(p); }
  subclusters.push_back(hook);

  // 4. Abnormal Lateral Distractor: (-0.30, 0.0, 3.3m) -> dx = 0.68m, dz = 0.3m -> slope = 66.2 deg > 35 deg, turn > 40 deg!
  CraneSubcluster distractor;
  distractor.subcluster_id = 88; distractor.layer_id = 5;
  distractor.centroid_x = -0.30; distractor.centroid_y = 0.0; distractor.centroid_z = 3.30;
  distractor.min_x = -0.35; distractor.max_x = -0.25; distractor.min_y = -0.05; distractor.max_y = 0.05;
  distractor.min_z = 3.25; distractor.max_z = 3.35;
  for (int i = 0; i < 15; ++i) { PointXYZIRT p{}; p.x = -0.30f; p.y = 0.0f; p.z = 3.30f; distractor.point_indices.push_back(points.size()); points.push_back(p); }
  subclusters.push_back(distractor);

  // 5. Normal Shackle below Hook: (-0.98, 0.01, 2.4m) -> dz = 1.2m, dx = 0.0 -> slope = 0.0 deg, turn = 0.0 deg
  CraneSubcluster shackle;
  shackle.subcluster_id = 4; shackle.layer_id = 4;
  shackle.centroid_x = -0.98; shackle.centroid_y = 0.01; shackle.centroid_z = 2.40;
  shackle.min_x = -1.05; shackle.max_x = -0.91; shackle.min_y = -0.06; shackle.max_y = 0.08;
  shackle.min_z = 2.30; shackle.max_z = 2.50;
  for (int i = 0; i < 15; ++i) { PointXYZIRT p{}; p.x = -0.98f; p.y = 0.01f; p.z = 2.40f; shackle.point_indices.push_back(points.size()); points.push_back(p); }
  subclusters.push_back(shackle);

  // 6. Large Horizontal Beam: Centered at (0, 0, 0.50m), spans [-2.7, 2.7] in X -> covers Hook at x = -0.98m
  CraneSubcluster beam;
  beam.subcluster_id = 5; beam.layer_id = 2;
  beam.centroid_x = 0.0; beam.centroid_y = 0.0; beam.centroid_z = 0.50;
  beam.min_x = -2.70; beam.max_x = 2.70; beam.min_y = -0.19; beam.max_y = 0.19;
  beam.min_z = 0.35; beam.max_z = 0.65;
  for (int gx = 0; gx < 108; ++gx) {
    for (int gy = 0; gy < 8; ++gy) {
      PointXYZIRT p{};
      p.x = static_cast<float>(-2.675 + 0.05 * gx);
      p.y = static_cast<float>(-0.175 + 0.05 * gy);
      p.z = 0.50f;
      beam.point_indices.push_back(points.size());
      points.push_back(p);
    }
  }
  subclusters.push_back(beam);

  // 7. Ground contact below Hook: (-0.98, 0.0, 0.10m)
  CraneSubcluster ground;
  ground.subcluster_id = 6; ground.layer_id = 1;
  ground.centroid_x = -0.98; ground.centroid_y = 0.0; ground.centroid_z = 0.10;
  ground.min_x = -1.15; ground.max_x = -0.81; ground.min_y = -0.10; ground.max_y = 0.10;
  ground.min_z = 0.06; ground.max_z = 0.14;
  for (int i = 0; i < 15; ++i) { PointXYZIRT p{}; p.x = -0.98f; p.y = 0.0f; p.z = 0.10f; ground.point_indices.push_back(points.size()); points.push_back(p); }
  subclusters.push_back(ground);

  const auto & res = pipeline.processHookAndBeamDetection(points, subclusters, 1.0);

  // Verification 1: Chain exists
  CHECK(res.top_down_hook.left_detected);
  const auto & chain = res.top_down_hook.chains[res.top_down_hook.left_chain_idx];
  CHECK(chain.nodes.size() == 6); // Root (1), Cable (2), Hook (3), Shackle (4), Beam (5), Ground (6)

  // Verification 2: Lateral distractor 88 was strictly REJECTED by slope/turn limit!
  for (const auto & n : chain.nodes) {
    CHECK(n.subcluster_id != 88);
  }

  // Verification 3: Max slope and turn angles along the chain are within configured limits
  CHECK(chain.max_slope_angle_deg <= cfg.top_down_hook.max_slope_angle_deg);
  CHECK(chain.max_turn_angle_deg <= cfg.top_down_hook.max_turn_angle_deg);

  // Verification 4: Large Horizontal Beam was NOT blocked and was correctly selected as Largest Connected Cluster
  CHECK(chain.largest_cluster.valid);
  CHECK(chain.largest_cluster.subcluster_id == 5);
  CHECK(chain.lowest_subcluster_id == 6); // Reached ground

  std::cout << "topDownSlopeAndTurnAngleLimitTest (Lateral Distractor Rejection + Continuous Angle Limits + Beam Acceptance) passed successfully!\n";
}

void virtualRopeWireBundleCenterlineTest() {
  CraneSlopeConfig cfg;
  cfg.virtual_rope.enabled = true;
  cfg.virtual_rope.wire_min_spacing_m = 0.10;
  cfg.virtual_rope.wire_max_spacing_m = 0.45;
  cfg.virtual_rope.wire_merge_span_m = 0.16;
  cfg.virtual_rope.memory_frames = 30;

  // Case 1: Two subclusters in same layer with severe point count asymmetry (150 pts vs 15 pts)
  // Strand A at X = -1.05, Strand B at X = -0.75 (Midpoint = -0.90)
  CraneSubcluster sc_a;
  sc_a.subcluster_id = 1; sc_a.layer_id = 10;
  sc_a.centroid_x = -1.05; sc_a.centroid_y = 0.05; sc_a.centroid_z = 4.5;
  sc_a.min_x = -1.08; sc_a.max_x = -1.02; sc_a.min_y = 0.03; sc_a.max_y = 0.07;
  sc_a.min_z = 4.45; sc_a.max_z = 4.55;
  sc_a.point_indices.resize(150, 0);

  CraneSubcluster sc_b;
  sc_b.subcluster_id = 2; sc_b.layer_id = 10;
  sc_b.centroid_x = -0.75; sc_b.centroid_y = 0.05; sc_b.centroid_z = 4.5;
  sc_b.min_x = -0.78; sc_b.max_x = -0.72; sc_b.min_y = 0.03; sc_b.max_y = 0.07;
  sc_b.min_z = 4.45; sc_b.max_z = 4.55;
  sc_b.point_indices.resize(15, 0);

  std::vector<CraneSubcluster> subclusters = {sc_a, sc_b};
  std::vector<PointXYZIRT> points;

  CraneHookChainNode rn;
  rn.layer_id = 10; rn.subcluster_id = 1;
  rn.x = -1.05; rn.y = 0.05; rn.z = 4.5;

  auto geom1 = CranePipeline::resolveWireBundleGeometry(
      rn, subclusters, points, nullptr, cfg.virtual_rope, cfg.dual_roi_anchor);

  CHECK(geom1.mode == CraneWireBundleMode::BOTH_STRANDS);
  CHECK(std::abs(geom1.center_x - (-0.90)) < 1e-4);
  CHECK(std::abs(geom1.center_y - 0.05) < 1e-4);
  CHECK(std::abs(geom1.spacing_dist - 0.30) < 1e-4);

  // Case 2: Memory retention when Strand B disappears (only Strand A visible)
  CranePipeline::TrackedChainNodeState prior_sn;
  prior_sn.wire_bundle = geom1;
  prior_sn.remembered_bundle = geom1;
  prior_sn.wire_bundle_memory_age = 0;

  std::vector<CraneSubcluster> subclusters_single_a = {sc_a};
  auto geom2 = CranePipeline::resolveWireBundleGeometry(
      rn, subclusters_single_a, points, &prior_sn, cfg.virtual_rope, cfg.dual_roi_anchor);

  CHECK(geom2.mode == CraneWireBundleMode::ESTIMATED_FROM_SINGLE_A);
  // Center should NOT snap to -1.05! It must hold at -0.90 via remembered offset (+0.15m)
  CHECK(std::abs(geom2.center_x - (-0.90)) < 1e-4);
  CHECK(std::abs(geom2.center_y - 0.05) < 1e-4);

  // Case 3: Memory retention when Strand A disappears (only Strand B visible)
  CraneHookChainNode rn_b;
  rn_b.layer_id = 10; rn_b.subcluster_id = 2;
  rn_b.x = -0.75; rn_b.y = 0.05; rn_b.z = 4.5;
  std::vector<CraneSubcluster> subclusters_single_b = {sc_b};
  auto geom3 = CranePipeline::resolveWireBundleGeometry(
      rn_b, subclusters_single_b, points, &prior_sn, cfg.virtual_rope, cfg.dual_roi_anchor);

  CHECK(geom3.mode == CraneWireBundleMode::ESTIMATED_FROM_SINGLE_B);
  // Center should NOT snap to -0.75! It must hold at -0.90 via remembered offset (-0.15m)
  CHECK(std::abs(geom3.center_x - (-0.90)) < 1e-4);
  CHECK(std::abs(geom3.center_y - 0.05) < 1e-4);

  // Case 4: Merged cluster with 2 internal modes
  CraneHookChainNode rn_merged;
  rn_merged.layer_id = 10;
  rn_merged.x = -0.92; rn_merged.y = 0.0; rn_merged.z = 4.5;
  rn_merged.min_x = -1.10; rn_merged.max_x = -0.70;
  rn_merged.min_y = -0.05; rn_merged.max_y = 0.05;
  rn_merged.min_z = 4.45; rn_merged.max_z = 4.55;

  std::vector<PointXYZIRT> merged_pts;
  for (int i = 0; i < 20; ++i) {
    PointXYZIRT p{}; p.x = -1.05f; p.y = 0.0f; p.z = 4.5f;
    rn_merged.point_indices.push_back(merged_pts.size());
    merged_pts.push_back(p);
  }
  for (int i = 0; i < 20; ++i) {
    PointXYZIRT p{}; p.x = -0.75f; p.y = 0.0f; p.z = 4.5f;
    rn_merged.point_indices.push_back(merged_pts.size());
    merged_pts.push_back(p);
  }

  std::vector<CraneSubcluster> empty_subclusters;
  auto geom4 = CranePipeline::resolveWireBundleGeometry(
      rn_merged, empty_subclusters, merged_pts, nullptr, cfg.virtual_rope, cfg.dual_roi_anchor);

  CHECK(geom4.mode == CraneWireBundleMode::BOTH_STRANDS);
  CHECK(std::abs(geom4.center_x - (-0.90)) < 1e-2);
  CHECK(std::abs(geom4.spacing_dist - 0.30) < 1e-2);

  std::cout << "virtualRopeWireBundleCenterlineTest (Midpoint Asymmetry Immunity + Memory Offset Holding + Merged 2-Means) passed successfully!\n";
}

struct HousingFixture {
  std::vector<PointXYZIRT> points;
  std::vector<CraneSubcluster> subs;
  void add(int id, const std::vector<PointXYZIRT> & pts) {
    CraneSubcluster sc;sc.subcluster_id=id;sc.layer_id=id;
    sc.min_x=sc.min_y=sc.min_z=1e9;sc.max_x=sc.max_y=sc.max_z=-1e9;
    for(const auto&p:pts){sc.point_indices.push_back(points.size());points.push_back(p);
      sc.centroid_x+=p.x;sc.centroid_y+=p.y;sc.centroid_z+=p.z;
      sc.min_x=std::min(sc.min_x,double(p.x));sc.max_x=std::max(sc.max_x,double(p.x));
      sc.min_y=std::min(sc.min_y,double(p.y));sc.max_y=std::max(sc.max_y,double(p.y));
      sc.min_z=std::min(sc.min_z,double(p.z));sc.max_z=std::max(sc.max_z,double(p.z));}
    sc.centroid_x/=pts.size();sc.centroid_y/=pts.size();sc.centroid_z/=pts.size();subs.push_back(sc);
  }
};
static HousingFixture housingFixture(double shift=0, bool body=true, bool split=false, bool affect_right=true) {
  HousingFixture f;
  for(int side=0;side<2;++side){const double cx=side?1.07:-.90;const double side_shift=(side && !affect_right)?0:shift;const bool include_body=(side && !affect_right)?true:body;std::vector<PointXYZIRT> root,wires,housing,upper,cargo;
    auto point=[](double x,double y,double z){PointXYZIRT p;p.x=x;p.y=y;p.z=z;return p;};
    for(int i=0;i<30;++i)root.push_back(point(cx+.02*(i%3-1),.05+.02*((i/3)%3-1),5.25+.01*(i%5-2)));
    for(int i=0;i<24;++i)for(int strand:{-1,1})wires.push_back(point(cx+.15*strand,.05,4.0+.6*i/23.));
    for(int i=0;i<7;++i)for(int j=0;j<5;++j)for(int k=0;k<7;++k){auto p=point(cx-.175+.35*i/6.,-.04+.18*j/4.,3.5+.25*k/6.+side_shift);housing.push_back(p);if(k<4)upper.push_back(p);}
    for(int i=0;i<20;++i)cargo.push_back(point(cx-.5+i/19.,.05,2.5));
    int id=side*10;f.add(id+1,root);f.add(id+2,wires);
    if(include_body){if(split){std::vector<PointXYZIRT> lower;for(const auto &p:housing)if(p.z>3.5+.125+side_shift)lower.push_back(p);f.add(id+3,upper);f.add(id+4,lower);}else f.add(id+3,housing);}
    f.add(id+5,cargo);
  }
  return f;
}
static CraneSlopeConfig housingConfig(bool fixed=true) {
  CraneSlopeConfig c;c.top_down_hook.enabled=true;c.fixed_root.enabled=fixed;c.fixed_root.min_points=5;
  c.fixed_root.left_roi={-1.25,-.55,-.25,.35,4.8,5.7};c.fixed_root.right_roi={.72,1.42,-.25,.35,4.8,5.7};
  c.top_down_hook.max_gap_z=2.5;c.top_down_hook.direct_link_max_gap_z=1.;
  c.top_down_hook.max_slope_angle_deg=35;c.top_down_hook.max_turn_angle_deg=40;
  c.temporal_stabilization.temporal_stabilization_enabled=true;c.temporal_stabilization.node_confirm_frames=1;
  c.temporal_stabilization.node_max_hold_frames=2;c.temporal_stabilization.stabilization_alpha=.8;
  c.virtual_rope.enabled=true;c.dual_roi_anchor.enabled=true;c.sheave.enabled=true;c.sheave.confirm_frames=2;
  return c;
}
static void sameWireOutput(const CraneTopDownHookResult & a,const CraneTopDownHookResult & b) {
  CHECK(a.left_chain_idx==b.left_chain_idx);CHECK(a.right_chain_idx==b.right_chain_idx);CHECK(a.chains.size()==b.chains.size());
  for(size_t i=0;i<a.chains.size();++i){const auto &x=a.chains[i];const auto &y=b.chains[i];
    CHECK(x.root_x==y.root_x && x.root_y==y.root_y && x.root_z==y.root_z);CHECK(x.nodes.size()==y.nodes.size());
    CHECK(x.lowest_x==y.lowest_x && x.lowest_y==y.lowest_y && x.lowest_z==y.lowest_z);
    CHECK(x.state==y.state && x.is_held==y.is_held && x.held_frames==y.held_frames);
    CHECK(x.largest_cluster.centroid_z==y.largest_cluster.centroid_z);CHECK(x.locked_anchor_count==y.locked_anchor_count);
    for(size_t j=0;j<x.nodes.size();++j){const auto&p=x.nodes[j];const auto&q=y.nodes[j];
      CHECK(p.x==q.x && p.y==q.y && p.z==q.z);CHECK(p.subcluster_id==q.subcluster_id && p.parent_subcluster_id==q.parent_subcluster_id);
      CHECK(p.point_indices==q.point_indices);CHECK(p.anchor_id==q.anchor_id && p.anchor_locked==q.anchor_locked);
      CHECK(p.anchor_x==q.anchor_x && p.anchor_y==q.anchor_y && p.anchor_z==q.anchor_z);
      CHECK(p.moving_x==q.moving_x && p.moving_y==q.moving_y && p.moving_z==q.moving_z);
      CHECK(p.is_held==q.is_held && p.held_frames==q.held_frames && p.relocate_count==q.relocate_count);
      CHECK(p.wire_bundle.mode==q.wire_bundle.mode && p.wire_bundle.center_x==q.wire_bundle.center_x && p.wire_bundle.center_y==q.wire_bundle.center_y);
      CHECK(p.is_connection_stable==q.is_connection_stable && p.is_estimated_gap==q.is_estimated_gap);
      CHECK(!q.is_sheave);
    }
  }
}
namespace cx128s2_bev_perception {
struct CranePipelineRegressionAccess {
 static void independentGroundTest(){
  for(bool independent:{false,true}){CraneSlopeConfig c;c.work_status.fsm_enabled=true;CranePipeline p(c);CraneBeamConfig bc;
   auto&w=p.work_cycle_;w.fsm_enabled=w.work_status=true;w.working_status=1;w.cargo_track_id=7;w.cargo_side="LEFT";w.fsm_state=CraneBeamState::LOCKED_TRACKING;w.length=3;w.width=.4;w.yaw=0;
   std::vector<PointXYZIRT>raw;std::vector<int>beam;
   for(int x=0;x<=60;++x)for(int y=0;y<=8;++y)for(int z=0;z<=1;++z){PointXYZIRT q;q.x=-1.5+x*.05;q.y=-.2+y*.05;q.z=1+z*.4;beam.push_back(raw.size());raw.push_back(q);}
   if(independent){for(int x=0;x<=80;++x)for(int y=0;y<=32;++y){PointXYZIRT q;q.x=-2+x*.05;q.y=-.8+y*.05;q.z=0;p.result_.ground.push_back(raw.size());raw.push_back(q);}}
   else p.result_.ground=beam;
   w.cargo_observation=w.extension_observation="DIRECT";w.extended_raw_indices=beam;w.left_hook.object_point_indices=beam;
   p.updateBeamGroundClearance(raw,1,bc);CHECK(w.beam_ground_clearance_valid==independent);
   if(independent)CHECK(std::abs(w.beam_ground_clearance_m-1)<.01);
  }
 }


 static void sparseBottomTest(){
  for(bool sparse:{false,true}){CraneSlopeConfig c;c.work_status.fsm_enabled=true;CranePipeline p(c);CraneBeamConfig bc;
   auto&w=p.work_cycle_;w.fsm_enabled=w.work_status=true;w.working_status=1;w.cargo_track_id=7;w.cargo_side="LEFT";w.fsm_state=CraneBeamState::LOCKED_TRACKING;w.length=3;w.width=.4;w.yaw=0;
   std::vector<PointXYZIRT>raw;std::vector<int>beam;
   for(int x=0;x<=60;++x)for(int y=0;y<=8;++y)for(int z=0;z<=1;++z){if(sparse && z==0 && y!=0 && y!=8)continue;PointXYZIRT q;q.x=-1.5+x*.05;q.y=-.2+y*.05;q.z=1+z*.4;beam.push_back(raw.size());raw.push_back(q);}
   for(int x=0;x<=80;++x)for(int y=0;y<=32;++y){PointXYZIRT q;q.x=-2+x*.05;q.y=-.8+y*.05;q.z=0;p.result_.ground.push_back(raw.size());raw.push_back(q);}
   w.cargo_observation=w.extension_observation="DIRECT";w.extended_raw_indices=beam;w.left_hook.object_point_indices=beam;
   p.updateBeamGroundClearance(raw,1,bc);CHECK(w.beam_ground_clearance_valid==!sparse);
  }
 }

 static void rawOwnershipTest(){
  CraneSlopeConfig c;c.work_status.fsm_enabled=true;CranePipeline p(c);auto&w=p.work_cycle_;
  w.geometry_locked=w.work_status=true;w.cargo_track_id=7;w.cargo_observation="DIRECT";w.cargo_side="LEFT";
  w.length=1;w.width=.5;w.height=.2;w.cargo_z=1.1;w.confirmed_observation_sec=3;
  p.cargo_hits_=30;p.cargo_last_seen_=1;p.cargo_xy_filter_.initialize(0,0,.03,1);p.cargo_z_filter_.initialize(1.1,0,.03,1);
  std::vector<PointXYZIRT>raw;std::vector<int>ids;
  for(int i=0;i<=20;++i)for(int j=0;j<=10;++j)for(int k=0;k<=4;++k){PointXYZIRT q;q.x=-.5+i*.05;q.y=-.25+j*.05;q.z=1+k*.05;ids.push_back(raw.size());raw.push_back(q);}
  p.rememberCargoRaw(raw,ids,1);const auto static_body=p.currentOwnedCargoRaw(raw,1.1);p.result_.top_down_hook.left_housing.sheave.outer_center_z=10;CHECK(p.currentOwnedCargoRaw(raw,1.1)==static_body);CHECK(p.currentOwnedCargoRaw(raw,1.1).size()>=c.work_status.min_candidate_points);CHECK(p.currentOwnedCargoRaw(raw,2.31).empty());
  auto foreign=raw;for(auto&q:foreign)q.x+=2;CHECK(p.currentOwnedCargoRaw(foreign,1.1).empty());
  w.cargo_observation="HELD";p.recoverCargoFromRawFragments(raw,{},1.1);CHECK(w.cargo_observation=="DIRECT" && w.cargo_track_id==7);CHECK(w.length==1 && w.width==.5);
  std::vector<PointXYZIRT> hook_only;for(const auto&q:raw)if(std::abs(q.x)<=.1 && std::abs(q.y)<=.1 && q.z>=1.1)hook_only.push_back(q);
  w.cargo_observation="HELD";p.recoverCargoFromRawFragments(hook_only,{},1.2);CHECK(w.cargo_observation=="HELD");
  p.recoverCargoFromRawFragments(raw,{},1.3);CHECK(w.cargo_observation=="DIRECT" && w.cargo_track_id==7);
  w.cargo_observation="HELD";p.recoverCargoFromRawFragments(foreign,{},1.4);CHECK(w.cargo_observation=="HELD");
  p.reset();CHECK(p.cargo_owned_points_.empty());
 }


  static void orderedWorkFsmTest(){
    CraneSlopeConfig cfg;cfg.work_status.fsm_enabled=true;cfg.work_status.virtual_hook_offset_z_m=.5;
    cfg.work_status.grid_resolution_m=.05;cfg.z_gaussian.min_layer_points=5;
    auto roi=[&](CranePipeline & p,double t,double rise,bool body=true,bool ground=true,int motion=2){
      auto & h=p.result_.top_down_hook.left_housing;auto & m=p.result_.top_down_hook.left_housing_memory;
      h.x=-.4;h.y=0;h.z=4.5+rise;h.sheave.status=CraneSheaveDetectionStatus::DIRECT_OBSERVED;
      h.sheave.points_from_current_frame=true;m.confirmed=true;m.track_id=11;
      p.sheave_motion_state_[0]=motion;p.sheave_motion_id_[0]=11;
      std::vector<PointXYZIRT> raw;p.result_.ground.clear();
      if(body)for(int x=0;x<=20;++x)for(int y=0;y<=20;++y)for(int z=0;z<=4;++z){PointXYZIRT v;
        v.x=-.6+x*.02;v.y=-.2+y*.02;v.z=3.2+rise+z*.05;raw.push_back(v);}
      if(ground)for(int x=0;x<=20;++x)for(int y=0;y<=20;++y){PointXYZIRT v;v.x=-.6+x*.02;v.y=-.2+y*.02;v.z=0;
        p.result_.ground.push_back(raw.size());raw.push_back(v);}
      p.observeOrderedHookRoi(raw,t);p.projectOrderedState();return raw;
    };
    CranePipeline empty(cfg);for(int i=0;i<12;++i)roi(empty,1+i*.1,.03*i,false);
    CHECK(empty.ordered_state_==CraneBeamState::IDLE);CHECK(!empty.beamWorkProcessingAllowed());
    CranePipeline wire(cfg);roi(wire,1,0,false,false);
    for(int i=1;i<12;++i){std::vector<PointXYZIRT> thin;for(int k=0;k<100;++k){PointXYZIRT v;v.x=-.4;v.y=0;v.z=2+k*.015+i*.03;thin.push_back(v);}
      wire.observeOrderedHookRoi(thin,1+i*.1);CHECK(wire.ordered_state_==CraneBeamState::IDLE);}
    CranePipeline hook_body(cfg);roi(hook_body,1,0,false,false);
    for(int i=1;i<12;++i){std::vector<PointXYZIRT> block;
      for(int x=0;x<=10;++x)for(int y=0;y<=10;++y)for(int z=0;z<=4;++z){PointXYZIRT v;v.x=-.5+x*.02;v.y=-.1+y*.02;v.z=3.2+z*.05+i*.03;block.push_back(v);}
      hook_body.observeOrderedHookRoi(block,1+i*.1);CHECK(hook_body.ordered_state_==CraneBeamState::IDLE);}
    // Current solid local ROI + current supporting rise, no Beam candidate/geometry.
    CranePipeline p(cfg);for(int i=0;i<=6;++i)roi(p,1+i*.1,.03*i);
    CHECK(p.ordered_state_==CraneBeamState::LIFTING);CHECK(!p.beamWorkProcessingAllowed());
    CHECK(p.work_cycle_.cargo_track_id==-1 && !p.work_cycle_.geometry_locked);
    work(p,{}, {},1.65,0);CHECK(p.work_cycle_.cargo_track_id==-1); // early consumer gate
    for(int i=0;i<3;++i)roi(p,1.7+i*.1,.18,true,true,1);
    CHECK(p.ordered_state_==CraneBeamState::SUSPENDED);CHECK(p.beamWorkProcessingAllowed());
    CHECK(p.work_cycle_.cargo_track_id==-1); // SUSPENDED admission is not a public confirmed Beam.
    CranePipeline unknown(cfg);for(int i=0;i<=6;++i)roi(unknown,1+i*.1,.03*i,true,false);
    for(int i=0;i<5;++i)roi(unknown,1.7+i*.1,.18,true,false,1);
    CHECK(unknown.ordered_state_==CraneBeamState::LIFTING);
    // Existing acquisition and 2s geometry confirmation run ONLY after SUSPENDED.
    auto scene=[](double dx=0,double dz=0,bool partial=false){
      std::vector<PointXYZIRT> raw;CraneSubcluster sc;sc.subcluster_id=70;sc.centroid_x=-.4+dx;sc.centroid_z=3.3+dz;
      sc.min_x=-1.4+dx;sc.max_x=.6+dx;sc.min_y=partial?-.1:-.2;sc.max_y=partial?.1:.2;
      sc.min_z=3.2+dz;sc.max_z=3.4+dz;
      for(int x=0;x<=100;++x)for(int y=0;y<=20;++y)for(int z=0;z<=4;++z){PointXYZIRT v;
        v.x=sc.min_x+.02*x;v.y=sc.min_y+(sc.max_y-sc.min_y)*y/20;v.z=sc.min_z+.05*z;
        sc.point_indices.push_back(raw.size());raw.push_back(v);}
      for(int i=0;i<35;++i){PointXYZIRT v;v.x=-.4;v.y=0;v.z=3.46+dz+i*.015;raw.push_back(v);}
      return std::make_pair(raw,std::vector<CraneSubcluster>{sc});
    };
    CraneBeamConfig bc;
    for(int i=0;i<28;++i){auto f=scene();auto w=work(p,f.first,f.second,2+i*.1,1);
      if(i<19)CHECK(!w.geometry_locked);p.updatePublicWorkStatus(2+i*.1,bc);}
    CHECK(p.work_cycle_.geometry_locked);const auto locked=p.work_cycle_;const int id=locked.cargo_track_id;
    auto f=scene(.04,0,true);work(p,f.first,f.second,4.8,1);p.updatePublicWorkStatus(4.8,bc);
    CHECK(p.work_cycle_.cargo_track_id==id && p.work_cycle_.geometry_locked);
    CHECK(p.work_cycle_.length==locked.length && p.work_cycle_.width==locked.width && p.work_cycle_.height==locked.height);
    CHECK(p.work_cycle_.cargo_x!=locked.cargo_x); // Size lock is not position lock.
    CranePipeline foreign=p;auto replacement=scene(2.0);replacement.second[0].subcluster_id=99;
    work(foreign,replacement.first,replacement.second,4.9,1);
    CHECK(foreign.work_cycle_.cargo_observation=="HELD" && foreign.work_cycle_.cargo_track_id==id);
    work(foreign,replacement.first,replacement.second,6.3,1);
    CHECK(foreign.ordered_state_==CraneBeamState::IDLE && !foreign.work_cycle_.geometry_locked);

    // Historical direct Beam descent + supporting direct Sheave descent.
    p.ordered_last_seed_stamp_=5;p.ordered_last_seed_z_=3.3;p.ordered_lower_accum_=0;p.ordered_lower_count_=0;
    for(int i=1;i<=12;++i){p.work_cycle_.cargo_observation="DIRECT";p.cargo_last_seen_=5+i*.1;p.cargo_last_z_=3.3-.04*i;
      p.sheave_motion_state_[0]=3;p.result_.top_down_hook.left_housing.sheave.points_from_current_frame=true;
      p.updatePublicWorkStatus(5+i*.1,bc);}
    CHECK(p.ordered_state_==CraneBeamState::LOWERING);CHECK(p.work_cycle_.length==locked.length);
    // Loss remains reset, never lowering-complete.
    CranePipeline lost=p;lost.cargo_last_seen_=1;lost.work_last_stamp_=6.2;work(lost,{}, {},6.3,0);
    CHECK(lost.ordered_state_==CraneBeamState::IDLE);CHECK(lost.work_cycle_.working_status==0);
    p.work_cycle_.left_hook.hook_observation="VIRTUAL_DIRECT";p.work_cycle_.left_hook.hook_x=p.work_cycle_.cargo_x;
    p.work_cycle_.left_hook.hook_y=p.work_cycle_.cargo_y;p.work_cycle_.left_hook.hook_z=p.cargo_last_z_+1;
    for(int i=0;i<5;++i){p.work_cycle_.cargo_observation="DIRECT";p.work_cycle_.beam_ground_clearance_valid=true;
      p.work_cycle_.beam_ground_clearance_m=.2;p.cargo_last_seen_=6.3+i*.1;p.updatePublicWorkStatus(6.3+i*.1,bc);}
    CHECK(p.ordered_state_==CraneBeamState::LOWERING_CONFIRMED && p.work_cycle_.working_status==2);
    const double completed=p.ordered_completed_;p.updatePublicWorkStatus(6.9,bc);CHECK(p.ordered_completed_==completed);
    roi(p,completed+2.99,0,false);CHECK(p.ordered_state_==CraneBeamState::LOWERING_CONFIRMED);
    roi(p,completed+3.01,0,false);CHECK(p.ordered_state_==CraneBeamState::IDLE && !p.work_cycle_.geometry_locked);
    // No fabricated history after restart, including already elevated static ROI.
    for(int oldstage=0;oldstage<3;++oldstage){CranePipeline reboot(cfg);
      for(int i=0;i<12;++i)roi(reboot,10+i*.1,.5,true,true,1);
      CHECK(reboot.ordered_state_==CraneBeamState::IDLE && reboot.work_cycle_.cargo_track_id==-1);}
    // Hook sway alone cannot cause LOWERING without current Beam descent.
    CranePipeline sway(cfg);sway.ordered_state_=CraneBeamState::SUSPENDED;sway.ordered_side_=0;
    sway.work_cycle_=locked;sway.work_cycle_.cargo_observation="DIRECT";sway.cargo_last_z_=3.3;
    sway.sheave_motion_state_[0]=3;sway.result_.top_down_hook.left_housing.sheave.points_from_current_frame=true;
    sway.result_.top_down_hook.left_housing.sheave.status=CraneSheaveDetectionStatus::DIRECT_OBSERVED;
    for(int i=0;i<15;++i){sway.cargo_last_seen_=10+i*.1;sway.updatePublicWorkStatus(10+i*.1,bc);}
    CHECK(sway.ordered_state_==CraneBeamState::SUSPENDED);
    std::cout<<"orderedWorkFsmTest passed: ROI-only prelift, no pre-SUSPENDED Cargo calls, grounded/UNKNOWN admission, measured 2s lock and partial/position, historical descent, contact completion/3s hold, loss reset, static restart and sway negatives\n";
  }
  static void fieldStatusPolicyTest(){
    CraneSlopeConfig cfg;cfg.work_status.fsm_enabled=true;
    auto seed=[&](CranePipeline & p){auto & w=p.work_cycle_;w.cargo_track_id=7;w.cargo_cluster_id=70;
      w.geometry_locked=w.work_status=w.fsm_enabled=true;w.working_status=1;w.cargo_side="LEFT";
      w.fsm_state=CraneBeamState::LOCKED_TRACKING;w.cargo_observation="DIRECT";w.length=2;w.width=.4;w.height=.3;
      w.cargo_x=0;w.cargo_y=0;w.cargo_z=1;p.cargo_last_z_=1;p.cargo_last_seen_=1;
      w.left_hook.hook_observation="VIRTUAL_DIRECT";w.left_hook.hook_z=2;
      p.cargo_xy_filter_.initialize(0,0,.03,1);p.cargo_z_filter_.initialize(1,0,.03,1);
      p.cargo_hits_=20;p.work_last_stamp_=1;};
    auto finish=[&](CranePipeline & p,double t){p.lifting_last_stamp_=t;p.updateCargoCompletion(t);};
    CraneBeamConfig bc;
    // Fresh processes must acquire static attached Cargo from measurements, no prior lift history.
    auto attached=[&](CranePipeline & p,double t){
      std::vector<PointXYZIRT> raw;CraneSubcluster sc;sc.subcluster_id=70;
      sc.min_x=-1.4;sc.max_x=.6;sc.min_y=-.2;sc.max_y=.2;sc.min_z=3.2;sc.max_z=3.4;
      sc.centroid_x=-.4;sc.centroid_y=0;sc.centroid_z=3.3;
      for(int x=0;x<=100;++x)for(int y=0;y<=20;++y)for(int z=0;z<=3;++z){PointXYZIRT v;
        v.x=-1.4+x*.02;v.y=-.2+y*.02;v.z=3.2+z*.2/3;sc.point_indices.push_back(raw.size());raw.push_back(v);}
      for(int i=0;i<35;++i){PointXYZIRT v;v.x=-.4;v.y=0;v.z=3.46+i*.015;raw.push_back(v);}
      return work(p,raw,{sc},t,1);
    };
    for(int reboot=0;reboot<2;++reboot){CranePipeline p(cfg);CraneWorkStatusResult w;
      for(int i=0;i<28;++i)w=attached(p,10+i*.1);
      CHECK(w.work_status && w.geometry_locked && w.working_status==1);CHECK(w.public_working_status!=3);
      CHECK(w.fsm_state!=CraneBeamState::COMPLETED);
    }
    CranePipeline empty(cfg);for(int i=0;i<30;++i){auto w=work(empty,{}, {},10+i*.1,0);CHECK(w.working_status==0);}
    CranePipeline lift(cfg);seed(lift);auto & h=lift.result_.top_down_hook.left_housing;
    auto & mem=lift.result_.top_down_hook.left_housing_memory;mem.confirmed=true;mem.track_id=9;mem.last_frame_stamp=1.1;
    h.sheave.status=CraneSheaveDetectionStatus::DIRECT_OBSERVED;h.sheave.points_from_current_frame=true;
    lift.sheave_motion_state_[0]=2;lift.sheave_motion_id_[0]=9;lift.sheave_motion_last_direct_[0]=1.1;lift.lifting_last_stamp_=1;
    lift.updatePublicWorkStatus(1.1,bc);CHECK(lift.work_cycle_.public_working_status==3);
    lift.sheave_motion_state_[0]=1;lift.updatePublicWorkStatus(1.2,bc);CHECK(lift.work_cycle_.public_working_status==1);
    lift.work_cycle_.work_status=false;lift.work_cycle_.working_status=0;lift.sheave_motion_state_[0]=2;
    lift.updatePublicWorkStatus(1.3,bc);CHECK(lift.work_cycle_.public_working_status==0);
    // Direct Cargo rise remains usable with no Sheave/Hook visibility.
    CranePipeline body(cfg);seed(body);body.work_cycle_.left_hook.hook_observation="INVALID";body.lifting_last_stamp_=1;
    for(int i=1;i<=8;++i){body.cargo_last_z_=1+.03*i;body.cargo_last_seen_=1+i*.1;
      body.updatePublicWorkStatus(1+i*.1,bc);}
    CHECK(body.work_cycle_.public_working_status==3);
    body.updatePublicWorkStatus(1.9,bc);CHECK(body.work_cycle_.public_working_status==1);
    body.work_cycle_.cargo_observation="HELD";body.updatePublicWorkStatus(2,bc);CHECK(body.work_cycle_.public_working_status==1);
    CranePipeline lost(cfg);seed(lost);finish(lost,2.299);CHECK(lost.work_cycle_.working_status==1);
    finish(lost,2.3);CHECK(lost.work_cycle_.working_status==2 && lost.work_cycle_.status_reason=="LOSS_TIMEOUT");
    double event=lost.drop_started_stamp_;finish(lost,2.4);CHECK(lost.drop_started_stamp_==event);
    work(lost,{}, {},5.299,0);CHECK(lost.work_cycle_.working_status==2);
    work(lost,{}, {},5.3,0);CHECK(lost.work_cycle_.working_status==0);
    work(lost,{}, {},6,0);CHECK(lost.work_cycle_.working_status==0);
    CranePipeline provisional(cfg);seed(provisional);provisional.work_cycle_.geometry_locked=false;
    finish(provisional,2.5);CHECK(provisional.work_cycle_.working_status!=2);
    for(bool valid:{false,true}){CranePipeline p(cfg);seed(p);p.work_cycle_.beam_ground_clearance_valid=valid;
      p.work_cycle_.beam_ground_clearance_m=.38;
      for(int i=0;i<5;++i){p.cargo_last_seen_=1+i*.1;finish(p,1+i*.1);}
      CHECK((p.work_cycle_.working_status==2)==valid);
      if(valid)CHECK(p.work_cycle_.status_reason=="GROUND_CONTACT");
    }
    CranePipeline detached(cfg);seed(detached);detached.work_cycle_.left_hook.hook_x=5;
    for(int i=0;i<5;++i){detached.cargo_last_seen_=1+i*.1;finish(detached,1+i*.1);}
    CHECK(detached.work_cycle_.working_status==2 && detached.work_cycle_.status_reason=="HOOK_DETACHED");
    CranePipeline unseen(cfg);seed(unseen);unseen.work_cycle_.left_hook.hook_observation="INVALID";
    for(int i=0;i<5;++i){unseen.cargo_last_seen_=1+i*.1;finish(unseen,1+i*.1);}CHECK(unseen.work_cycle_.working_status==1);
    // Branch votes never combine. Invalid/missing observations break confirmation.
    CranePipeline split(cfg);seed(split);
    for(int i=0;i<8;++i){split.cargo_last_seen_=1+i*.1;split.work_cycle_.beam_ground_clearance_valid=i%2==0;
      split.work_cycle_.beam_ground_clearance_m=.2;split.work_cycle_.left_hook.hook_x=i%2?5:0;finish(split,1+i*.1);}
    CHECK(split.work_cycle_.working_status==1);
    body.reset();CHECK(body.work_cycle_.working_status==0 && body.field_cargo_rise_.empty());
    std::cout<<"fieldStatusPolicyTest passed: stationary/reboot/idle, independent Sheave/Cargo ascent, stop/HELD, independent LOSS/CONTACT/DETACHED, unknown, 3s hold, no repeat and branch mixing\n";
  }
  static CraneWorkStatusResult work(CranePipeline & p,const std::vector<PointXYZIRT> & raw,
      const std::vector<CraneSubcluster> & subs,double stamp,unsigned housings,double right_x=.4,double housing_rise=0,unsigned held=0){
    p.result_.top_down_hook=CraneTopDownHookResult{};
    for(int side=0;side<2;++side)if(housings&(1u<<side)){
      p.result_.top_down_hook.chains.push_back(CraneHookChain{});
      auto & h=side==0?p.result_.top_down_hook.left_housing.sheave:p.result_.top_down_hook.right_housing.sheave;
      h.status=CraneSheaveDetectionStatus::DIRECT_OBSERVED;h.outer_center_x=side==0?-.4:right_x;
      h.outer_center_y=0;h.outer_center_z=4.5+housing_rise;h.min_z=4.3+housing_rise;h.max_z=4.7+housing_rise;
      auto & node=side==0?p.result_.top_down_hook.left_housing:p.result_.top_down_hook.right_housing;
      node.x=h.outer_center_x;node.y=h.outer_center_y;node.z=h.outer_center_z;
      if(held&(1u<<side)){h.status=CraneSheaveDetectionStatus::HELD;node.is_held=true;}
      auto & memory=side==0?p.result_.top_down_hook.left_housing_memory:p.result_.top_down_hook.right_housing_memory;
      memory.initialized=memory.confirmed=true;memory.last_direct_stamp=stamp-.1;

    }
    p.updateWorkStatus(raw,subs,stamp);if(p.work_cycle_.gap_diagnostic=="SUPPORTED")p.updateCargoCompletion(stamp);return p.result_.work_status;
  }

  static void dropWindowAndMotionTest(){
    CraneSlopeConfig cfg;cfg.work_status.fsm_enabled=true;cfg.work_status.boundary_motion_enabled=true;
    CranePipeline p(cfg);
    // 20 sensor intervals of 0.1s. Current endpoint alone never backfills history.
    for(int i=0;i<20;++i){CHECK(!p.updateDropCandidate(1+i*.1,i<16?1:0));}
    CHECK(p.updateDropCandidate(3,1)); // 80% TRUE, full 2s history
    for(int i=1;i<=20;++i)p.updateDropCandidate(3+i*.1,i%2);
    CHECK(p.drop_candidate_); // 50% retains positive candidate
    p.resetDropCandidate();
    for(int i=0;i<=20;++i)p.updateDropCandidate(10+i*.1,i%2);
    CHECK(!p.drop_candidate_); // 50% retains negative candidate
    p.resetDropCandidate();p.drop_candidate_=true;
    for(int i=0;i<=20;++i)p.updateDropCandidate(20+i*.1,i<4?1:0);
    CHECK(!p.drop_candidate_); // 20% TRUE + 80% observed FALSE clears
    p.resetDropCandidate();p.drop_candidate_=true;
    for(int i=0;i<=20;++i)p.updateDropCandidate(25+i*.1,i<5?1:(i<15?0:-1));
    CHECK(!p.drop_candidate_); // exact 25%, with 75% valid time: clear
    p.resetDropCandidate();p.drop_candidate_=true;
    for(int i=0;i<=20;++i)p.updateDropCandidate(28+i*.1,i<10?0:-1);
    CHECK(p.drop_candidate_); // 0% TRUE but only 50% valid: UNKNOWN cannot drive the low edge
    p.resetDropCandidate();
    for(int i=0;i<=20;++i)p.updateDropCandidate(30+i*.1,1);
    CHECK(p.drop_candidate_);
    CHECK(p.updateDropCandidate(32.1,-1));CHECK(p.drop_observations_.size()>1);
    for(int i=2;i<=30;++i)p.updateDropCandidate(32+i*.1,-1);
    CHECK(p.drop_candidate_ && p.drop_window_valid_sec_==0); // UNKNOWN cannot clear or create evidence
    const auto size=p.drop_observations_.size();p.updateDropCandidate(35,1);CHECK(p.drop_observations_.size()==size);
    p.updateDropCandidate(40,1);CHECK(p.drop_window_valid_sec_==0); // full input gap is UNKNOWN
    CHECK(!p.updateDropCandidate(2,1)); // rewind resets
    p.resetDropCandidate();
    for(int i=0;i<=30;++i)p.updateDropCandidate(50+i*.1,-1);
    CHECK(!p.drop_candidate_); // UNKNOWN cannot create positive candidate
    // Irregular sampling: frame majority is negative, elapsed-time majority positive.
    p.resetDropCandidate();p.updateDropCandidate(60,1);
    for(int i=1;i<=8;++i)p.updateDropCandidate(60+i*.2,i==8?0:1);
    for(int i=1;i<=40;++i)p.updateDropCandidate(61.6+i*.01,0);
    CHECK(p.drop_candidate_);
    auto & td=p.result_.top_down_hook;
    td.left_housing.sheave.status=td.right_housing.sheave.status=CraneSheaveDetectionStatus::DIRECT_OBSERVED;
    td.left_housing.sheave.points_from_current_frame=td.right_housing.sheave.points_from_current_frame=true;
    for(int side=0;side<2;++side){
      auto & memory=side==0?td.left_housing_memory:td.right_housing_memory;
      memory.initialized=memory.ever_confirmed=memory.confirmed=true;memory.track_id=side+1;memory.last_frame_stamp=100;
      p.sheave_motion_id_[side]=side+1;p.sheave_motion_last_direct_[side]=100;
    }
    for(int count=0;count<=2;++count)for(int sheaves=0;sheaves<=2;++sheaves){
      p.boundary_move_count_=count;
      p.sheave_motion_state_={sheaves>=1?2:1,sheaves>=2?3:1};
      CHECK(p.confirmedCraneMoving()==(count>=2));
    }
    p.boundary_move_count_=1;p.sheave_motion_state_={2,1};
    td.left_housing.is_held=true;CHECK(!p.confirmedCraneMoving());td.left_housing.is_held=false;
    td.left_housing.sheave.status=CraneSheaveDetectionStatus::HELD;CHECK(!p.confirmedCraneMoving());
    td.left_housing.sheave.status=CraneSheaveDetectionStatus::DIRECT_OBSERVED;
    td.left_housing.point_indices={7};p.boundary_moving_points_={{7,8}};
    CHECK(!p.confirmedCraneMoving()); // one patch and the same Sheave support are not independent
    p.boundary_moving_points_.clear();td.left_housing.point_indices.clear();p.sheave_motion_state_={0,0};
    cfg.work_status.boundary_motion_enabled=false;cfg.fixed_root.enabled=true;p.updateConfig(cfg);
    for(auto * r:{&p.left_fixed_root_state_,&p.right_fixed_root_state_}){
      r->obs_status=CraneRootObservationStatus::OBSERVED;r->motion_status=CraneRootMotionStatus::MOVING;r->subcluster_id=5;
    }
    CHECK(!p.confirmedCraneMoving()); // same measured component is only one target
    p.right_fixed_root_state_.subcluster_id=6;CHECK(p.confirmedCraneMoving());
    p.right_fixed_root_state_.is_held=true;CHECK(!p.confirmedCraneMoving());
    std::cout<<"dropWindowAndMotionTest passed: 80/50/20, UNKNOWN, sensor time, duplicate/rewind/gap, motion truth table and HELD/dedup\n";
  }

  static void liftingClearanceTest(){
    auto cfg=housingConfig();cfg.work_status.fsm_enabled=true;cfg.work_status.boundary_motion_enabled=true;
    CranePipeline p(cfg);CraneBeamConfig bc;auto & w=p.work_cycle_;
    w.fsm_enabled=w.work_status=true;w.working_status=1;w.cargo_track_id=9;w.cargo_side="LEFT";
    w.fsm_state=CraneBeamState::LOCKED_TRACKING;w.length=3;w.width=.4;w.yaw=0;
    auto frame=[&](double t,double bottom,double ground,bool moving=false,bool rising=false,
        bool see_ground=true,bool profile=true,bool outlier=false,bool tilted=false){
      std::vector<PointXYZIRT> raw;std::vector<int> beam;p.result_.ground.clear();
      auto add=[&](double x,double y,double z){PointXYZIRT v;v.x=x;v.y=y;v.z=z;raw.push_back(v);return int(raw.size()-1);};
      for(int x=0;x<=60;++x)for(int y=0;y<=8;++y){double px=-1.5+x*.05;
        if(profile)beam.push_back(add(px,-.2+y*.05,bottom+(tilted?.05*px:0)));
        beam.push_back(add(px,-.2+y*.05,bottom+.4+(tilted?.05*px:0)));}
      if(outlier)beam.push_back(add(0,0,bottom-3));
      if(see_ground)for(int x=0;x<=80;++x)for(int y=0;y<=32;++y)p.result_.ground.push_back(add(-2+x*.05,-.8+y*.05,ground));
      if(outlier && see_ground)p.result_.ground.push_back(add(0,0,ground+3));
      w.cargo_observation=w.extension_observation="DIRECT";w.extended_raw_indices=beam;
      w.left_hook.object_point_indices=beam;w.left_hook.hook_observation="VIRTUAL_DIRECT";w.left_hook.cargo_rise=rising?.2:0;
      p.cargo_hits_=10;p.cargo_stationary_current_=true;p.boundary_move_count_=moving?2:0;
      auto & m=p.result_.top_down_hook.left_housing_memory;auto & h=p.result_.top_down_hook.left_housing;
      m.initialized=m.ever_confirmed=m.confirmed=true;m.track_id=71;m.last_frame_stamp=t;
      h.sheave.status=CraneSheaveDetectionStatus::DIRECT_OBSERVED;h.sheave.points_from_current_frame=true;
      p.sheave_motion_id_[0]=71;p.sheave_motion_last_direct_[0]=t;p.sheave_motion_state_[0]=rising?2:1;
      p.updateBeamGroundClearance(raw,t,bc);return p.work_cycle_;
    };
    for(int i=0;i<8;++i){auto r=frame(1+i*.1,1,0);CHECK(r.public_working_status==1);}
    CHECK(w.beam_ground_clearance_valid && w.initial_clearance_valid);
    CHECK(std::abs(w.beam_ground_clearance_m-1)<.01 && std::abs(w.beam_bottom_z-w.ground_surface_z-w.beam_ground_clearance_m)<1e-9);
    CHECK(w.beam_ground_clearance_sample_valid.size()==3);
    CHECK(frame(1.8,1,0,true).public_working_status==1); // Pure XY, same clearance.
    CHECK(frame(1.9,1,-.2,true).public_working_status==1); // One new-gap observation is insufficient.
    CHECK(frame(2,1,-.2,true).public_working_status==3); // No Z rise; measured lower ground + actual XY.
    for(int i=1;i<=20;++i){auto r=frame(2+i*.1,1,-.2,true);if(i>=6)CHECK(r.public_working_status==1);}
    // A: linked, current Sheave and actual Cargo rise; XY output remains horizontal only.
    CHECK(frame(4.1,1.2,-.2,false,true).public_working_status==3);CHECK(w.work_status);
    CHECK(!p.confirmedCraneMoving());
    CHECK(frame(4.2,1.2,-.2,false,false).public_working_status==1);
    CHECK(frame(4.3,1.2,-.2,true,false,false).public_working_status==1);CHECK(!w.beam_ground_clearance_valid);
    CHECK(frame(4.4,1.2,-.2,true,false).public_working_status==1); // Ground return alone is not a fresh gap.
    frame(4.5,1.2,-.2,false,false,true,false);CHECK(!w.beam_ground_clearance_valid); // Top-only profile is UNKNOWN.
    frame(4.6,1.2,-.2,false,false,true,true,true);CHECK(w.beam_ground_clearance_valid);
    CHECK(std::abs(w.beam_ground_clearance_m-1.4)<.01); // Single low beam/high ground noise cannot move surfaces.
    frame(4.7,1.2,-.2,false,false,true,true,false,true);CHECK(w.beam_ground_clearance_valid);
    CHECK(*std::max_element(w.beam_ground_clearance_samples_m.begin(),w.beam_ground_clearance_samples_m.end())-
          *std::min_element(w.beam_ground_clearance_samples_m.begin(),w.beam_ground_clearance_samples_m.end())>.05);
    w.work_status=false;w.working_status=0;
    for(int i=0;i<8;++i)CHECK(frame(5+i*.1,1.2,-.6,true).public_working_status==0); // Beam only never forces working.
    w.work_status=true;w.working_status=2;w.fsm_state=CraneBeamState::COMPLETED;
    CHECK(frame(5.9,1.2,-.8,true,true).public_working_status==2); // DROP has priority.
    p.reset();w.fsm_enabled=w.work_status=true;w.working_status=1;w.cargo_track_id=9;w.cargo_side="LEFT";
    w.fsm_state=CraneBeamState::LOCKED_TRACKING;w.length=3;w.width=.4;w.yaw=0;
    for(int i=0;i<8;++i)frame(10+i*.1,.02,0);
    CHECK(w.initial_clearance_valid);
    frame(10.8,.02,-.1,true);CHECK(frame(10.9,.02,-.1,true).public_working_status==3); // actual contact separation
    for(int i=0;i<14;++i)frame(11+i*.1,.02,-.1,true);
    CHECK(w.public_working_status==1);
    frame(12.4,.02,-.3,true);CHECK(frame(12.5,.02,-.3,true).public_working_status==3); // new increase rearms
    CHECK(std::string(workingStatusName(0))=="idle" && std::string(workingStatusName(1))=="working" &&
      std::string(workingStatusName(2))=="drop" && std::string(workingStatusName(3))=="lifting");
    std::cout<<"liftingClearanceTest passed: measured segmented surfaces, A/B, hold/rearm, XY-only, missing/top-only/noise, no forced work, DROP priority\n";
  }

  static void sheaveZMotionTest(){
    auto cfg=housingConfig();cfg.sheave.track_continuity_enabled=true;cfg.work_status.boundary_motion_enabled=true;
    CranePipeline p(cfg);p.setSheaveMotionFrame("world");
    auto feed=[&](double t,double left_z,double right_z=4.,unsigned direct=3){
      auto & td=p.result_.top_down_hook;
      for(int side=0;side<2;++side){
        auto & h=side==0?td.left_housing:td.right_housing;
        auto & m=side==0?td.left_housing_memory:td.right_housing_memory;
        if(!m.initialized){m.initialized=m.ever_confirmed=m.confirmed=true;m.track_id=side+11;}
        m.last_frame_stamp=t;m.frame_period=.1;
        h.sheave.status=(direct&(1u<<side))?CraneSheaveDetectionStatus::DIRECT_OBSERVED:CraneSheaveDetectionStatus::HELD;
        h.sheave.points_from_current_frame=bool(direct&(1u<<side));h.is_held=!(direct&(1u<<side));
        h.sheave.outer_center_z=side==0?left_z:right_z;
      }
      p.updateSheaveMotion(td,t);
    };
    for(int i=0;i<=15;++i)feed(1+i*.1,4.);
    CHECK(p.sheave_motion_state_[0]==1 && p.sheave_motion_state_[1]==1);
    CHECK(p.sheave_stationary_anchor_z_[0]==4.);
    double moving_anchor=NAN;bool rose=false,fell=false;
    for(int i=1;i<=25;++i){feed(2.5+i*.1,4+i*.04,4-i*.04);
      rose=rose || p.sheave_motion_state_[0]==2;fell=fell || p.sheave_motion_state_[1]==3;
      if(p.sheave_motion_state_[0]==2){
        if(!std::isfinite(moving_anchor))moving_anchor=p.sheave_stationary_anchor_z_[0];
        CHECK(p.sheave_stationary_anchor_z_[0]==moving_anchor);
      }
    }
    CHECK(rose && fell && p.sheave_motion_state_[0]==2 && p.sheave_motion_state_[1]==3);
    p.boundary_move_count_=0;CHECK(!p.confirmedCraneMoving());
    p.boundary_move_count_=1;CHECK(!p.confirmedCraneMoving());
    // One missed observation preserves the anchor, but cannot supply condition B.
    feed(5.1,5,3,0);CHECK(p.sheave_stationary_anchor_z_[0]==moving_anchor);CHECK(!p.confirmedCraneMoving());
    for(int i=2;i<=22;++i)feed(5+i*.1,5,3);
    CHECK(p.sheave_motion_state_[0]==1 && p.sheave_motion_state_[1]==1);
    CHECK(p.sheave_stationary_anchor_z_[0]==5 && p.sheave_stationary_anchor_z_[1]==3);
    CHECK(!p.confirmedCraneMoving());
    for(int i=1;i<=25;++i)feed(7.2+i*.1,5-i*.04,3+i*.04);
    CHECK(p.sheave_motion_state_[0]==3 && p.sheave_motion_state_[1]==2);
    // HELD/DORMANT do not update a stationary anchor or provide moving evidence.
    const auto held_anchor=p.sheave_stationary_anchor_z_[0];
    for(int i=1;i<=12;++i)feed(9.7+i*.1,100,-100,0);
    CHECK(p.sheave_stationary_anchor_z_[0]==held_anchor && p.sheave_motion_state_[0]==0);
    CHECK(!p.confirmedCraneMoving());
    p.result_.top_down_hook.left_housing_memory.dormant=true;
    p.result_.top_down_hook.left_housing_memory.confirmed=false;
    feed(11,50);CHECK(p.sheave_stationary_anchor_z_[0]==held_anchor);
    p.result_.top_down_hook.left_housing_memory.dormant=false;p.result_.top_down_hook.left_housing_memory.confirmed=true;
    // Real identity change discards the old anchor; stable new direct position seeds it.
    p.result_.top_down_hook.left_housing_memory.track_id=77;feed(11.1,7);
    CHECK(!std::isfinite(p.sheave_stationary_anchor_z_[0]));feed(11.2,7);CHECK(p.sheave_stationary_anchor_z_[0]==7);
    const auto count=p.sheave_motion_samples_[0].size();feed(11.2,100);CHECK(p.sheave_motion_samples_[0].size()==count);
    feed(12,7);CHECK(p.sheave_motion_state_[0]==0 && p.sheave_stationary_anchor_z_[0]==7);
    feed(2,7);CHECK(!std::isfinite(p.sheave_stationary_anchor_z_[0]));
    p.setSheaveMotionFrame("other_world");CHECK(!std::isfinite(p.sheave_stationary_anchor_z_[0]));CHECK(p.sheave_motion_state_[0]==0);
    // Deterministic 0.8s time weights, including exact 75/25 and 50% hold.
    auto votes=[&](double t,int old_state,int ones,int twos,int unknown){
      p.resetSheaveMotion(0);auto & m=p.result_.top_down_hook.left_housing_memory;
      m.track_id=77;m.initialized=m.confirmed=m.ever_confirmed=true;m.dormant=false;
      p.sheave_motion_id_[0]=77;p.sheave_stationary_anchor_z_[0]=4;
      p.sheave_motion_first_stamp_[0]=t-.8;p.sheave_motion_last_stamp_[0]=t-.1;p.sheave_motion_state_[0]=old_state;
      for(int i=0;i<8;++i){int evidence=i<ones?1:(i<ones+twos?2:0);
        p.sheave_motion_samples_[0].push_back({t-.8+i*.1,4.+i*.04,double(evidence)});}
      CHECK(ones+twos+unknown==8);feed(t,4.32);
      return p.sheave_motion_state_[0];
    };
    CHECK(votes(20,0,2,6,0)==2);CHECK(votes(21,2,4,4,0)==2);
    CHECK(votes(22,1,4,4,0)==1);CHECK(votes(23,2,6,2,0)==1);
    CHECK(votes(24,2,0,2,6)==2); // UNKNOWN cannot drive a 25% clear
    // Real stationary noise never becomes motion.
    p.reset();p.setSheaveMotionFrame("world");
    for(int i=0;i<40;++i){feed(30+i*.1,4+(i%2?.01:-.01));CHECK(p.sheave_motion_state_[0]!=2 && p.sheave_motion_state_[0]!=3);}
    // Slow coherent motion must accumulate against a fixed stopped anchor,
    // even when each 0.8s span is smaller than the existing 8cm deadband.
    p.reset();p.setSheaveMotionFrame("world");
    for(int i=0;i<=15;++i)feed(40+i*.1,4);
    const auto slow_anchor=p.sheave_stationary_anchor_z_[0];bool slow_rise=false;
    for(int i=1;i<=160;++i){feed(41.5+i*.1,4+i*.005);
      CHECK(p.sheave_stationary_anchor_z_[0]==slow_anchor);slow_rise=slow_rise || p.sheave_motion_state_[0]==2;}
    CHECK(slow_rise && p.sheave_motion_state_[0]==2);
    for(int i=1;i<=25;++i)feed(57.5+i*.1,4.8+(i%2?.005:-.005));
    CHECK(p.sheave_motion_state_[0]==1 && std::abs(p.sheave_stationary_anchor_z_[0]-4.8)<.01);
    // Uncertain direct Z is not fresh moving evidence indefinitely.
    for(int i=1;i<=25;++i)feed(60+i*.1,4.8+i*.04);
    CHECK(p.sheave_motion_state_[0]==2);
    for(int i=1;i<=20;++i)feed(62.5+i*.1,5.8+(i%2?.2:-.2));
    CHECK(p.sheave_motion_state_[0]==0);
    std::cout<<"sheaveZMotionTest passed: stationary anchors, rise/fall, elevated stop, new anchor/reverse, 75/25/50, noise, UNKNOWN, held/dormant, ID/frame/time discontinuities, crane_moving\n";
  }

  static void dropCompletionWindowTest(){
    auto seed=[](CranePipeline & p){
      auto & w=p.work_cycle_;w.fsm_enabled=w.work_status=true;w.working_status=1;
      w.fsm_state=CraneBeamState::LOCKED_TRACKING;w.cargo_side="LEFT";
      w.cargo_observation=w.extension_observation="DIRECT";w.gap_diagnostic="SUPPORTED";
      w.length=2;w.width=.4;w.cargo_z=1;w.left_hook.hook_observation="VIRTUAL_DIRECT";
      w.left_hook.hook_x=5;w.left_hook.hook_z=2;
      p.cargo_xy_filter_.initialize(0,0,.03,1);p.cargo_z_filter_.initialize(1,0,.03,1);
      p.cargo_stationary_current_=p.cargo_descent_seen_=true;p.work_elapsed_current_=.1;
    };
    CraneSlopeConfig cfg;cfg.work_status.fsm_enabled=true;
    CranePipeline p(cfg);seed(p);
    for(int i=0;i<=20;++i){
      p.work_cycle_.left_hook.hook_x=(i>=16 && i<20)?0:5;
      p.updateCargoCompletion(1+i*.1);
      CHECK((p.work_cycle_.fsm_state==CraneBeamState::COMPLETED)==(i==20));
    }
    const double completed=p.drop_started_stamp_;CHECK(completed==3);
    p.work_cycle_.left_hook.hook_observation="INVALID";p.updateCargoCompletion(3.1);
    CHECK(p.work_cycle_.fsm_state==CraneBeamState::COMPLETED); // event is latched
    p.updateWorkStatus({}, {},completed+2.999);CHECK(p.work_cycle_.fsm_state==CraneBeamState::COMPLETED);
    p.updateWorkStatus({}, {},completed+3);CHECK(p.work_cycle_.working_status==0);
    for(int positives:{4,10}){
      CranePipeline q(cfg);seed(q);
      for(int i=0;i<=20;++i){q.work_cycle_.left_hook.hook_x=(i<positives || i==20)?5:0;q.updateCargoCompletion(10+i*.1);}
      CHECK(q.work_cycle_.fsm_state!=CraneBeamState::COMPLETED);
    }
    for(const std::string missing:{"HOOK","CARGO","SUPPORT","EXTENSION","DESCENT","MOTION"}){
      CranePipeline q(cfg);seed(q);
      for(int i=0;i<=20;++i)q.updateDropCandidate(20+i*.1,1); // old positive candidate
      if(missing=="HOOK")q.work_cycle_.left_hook.hook_observation="VIRTUAL_HELD";
      if(missing=="CARGO")q.work_cycle_.cargo_observation="HELD";
      if(missing=="SUPPORT")q.work_cycle_.gap_diagnostic="UNKNOWN";
      if(missing=="EXTENSION")q.work_cycle_.extension_observation="PARTIAL_HELD";
      if(missing=="DESCENT")q.cargo_descent_seen_=false;
      if(missing=="MOTION")q.cargo_stationary_current_=false;
      for(int i=1;i<=20;++i)q.updateCargoCompletion(22+i*.1);
      CHECK(q.work_cycle_.fsm_state!=CraneBeamState::COMPLETED);
      if(missing=="HOOK"){
        q.work_cycle_.left_hook.hook_observation="VIRTUAL_DIRECT";q.updateCargoCompletion(24.1);
        CHECK(q.work_cycle_.fsm_state!=CraneBeamState::COMPLETED); // latest direct alone, no valid coverage
      }
    }
    std::cout<<"dropCompletionWindowTest passed: actual 80% completion, 50/20 negatives, all current gates, UNKNOWN coverage, latched 3s hold/reset\n";
  }

  static void bothHooksMustSeparate(CranePipeline & p) {
    auto & w=p.work_cycle_;w=CraneWorkStatusResult{};
    w.work_status=w.fsm_enabled=true;w.working_status=1;w.fsm_state=CraneBeamState::LOCKED_TRACKING;
    w.cargo_side="BOTH";w.cargo_observation=w.extension_observation="DIRECT";w.gap_diagnostic="SUPPORTED";
    w.length=2;w.width=.4;w.cargo_z=1;
    w.left_hook.hook_observation=w.right_hook.hook_observation="VIRTUAL_DIRECT";
    w.left_hook.hook_x=5;w.right_hook.hook_x=0;w.left_hook.hook_z=w.right_hook.hook_z=2;
    p.cargo_xy_filter_.initialize(0,0,.03,1);p.cargo_z_filter_.initialize(1,0,.03,1);
    p.cargo_stationary_current_=p.cargo_descent_seen_=true;p.work_elapsed_current_=.1;
    for(int i=0;i<40;++i){p.updateCargoCompletion(1+i*.1);CHECK(w.working_status==1);}
    w.right_hook.hook_observation="VIRTUAL_HELD";
    for(int i=0;i<40;++i){p.updateCargoCompletion(5+i*.1);CHECK(w.working_status==1);}
    w.right_hook.hook_observation="VIRTUAL_DIRECT";w.right_hook.hook_x=-5;
    for(int i=0;i<25;++i)p.updateCargoCompletion(9+i*.1);
    CHECK(w.working_status==2);
  }
  static void visibility(CranePipeline & p,VisibilityModel * v){p.visibility_model_ptr_=v;}
  static void support(CranePipeline & p){p.work_cycle_.gap_diagnostic="SUPPORTED";p.work_cycle_.extension_observation="DIRECT";}
  static void seedExtension(CranePipeline & p,double x=0,double y=0,double yaw=0){
    p.ordered_state_=CraneBeamState::SUSPENDED; // This unit fixture supplies a confirmed suspended seed, not startup acquisition.
    p.work_cycle_=CraneWorkStatusResult{};auto & w=p.work_cycle_;
    w.fsm_enabled=w.work_status=true;w.fsm_state=CraneBeamState::READY;w.working_status=1;
    w.cargo_track_id=41;w.cargo_side="RIGHT";w.cargo_observation="DIRECT";
    w.cargo_x=x;w.cargo_y=y;w.cargo_z=1;w.yaw=yaw;w.length=1;w.width=.3;w.height=.05;
  }
  static void seedRaw(CranePipeline & p,const std::vector<int> & ids){p.work_cycle_.left_hook.object_point_indices=ids;p.work_cycle_.right_hook.object_point_indices=ids;}
  static void heldExtension(CranePipeline & p){p.work_cycle_.cargo_observation="HELD";}
  static std::string wireState(const CranePipeline & p) {
    std::ostringstream out;out.precision(17);
    for(const auto *t:{&p.left_td_track_,&p.right_td_track_}){
      out<<t->initialized<<','<<t->next_anchor_id<<','<<t->largest_valid<<','<<t->largest_confirmed_frames<<','<<t->largest_missed_frames;
      for(const auto *v:{&t->stable_nodes,&t->pending_nodes,&t->remembered_anchors}){
        out<<'|'<<v->size();for(const auto & n:*v){
          out<<','<<n.layer_id<<','<<n.subcluster_id<<','<<n.x<<','<<n.y<<','<<n.z<<','<<n.confirmed_hits<<','<<n.missed_frames<<','<<n.is_held;
          out<<','<<n.anchor_locked<<','<<n.anchor_id<<','<<n.anchor_x<<','<<n.anchor_y<<','<<n.anchor_z<<','<<n.moving_x<<','<<n.moving_y<<','<<n.moving_z;
          out<<','<<n.relocate_count<<','<<n.relocate_confirm_count<<','<<n.is_connection_stable<<','<<n.consecutive_missing<<','<<n.memory_age_frames;
          out<<','<<int(n.wire_bundle.mode)<<','<<n.wire_bundle.center_x<<','<<n.wire_bundle.center_y<<','<<n.wire_bundle.center_z<<','<<n.wire_bundle.spacing_dist;
          out<<','<<int(n.remembered_bundle.mode)<<','<<n.remembered_bundle.spacing_dist<<','<<n.wire_bundle_memory_age;
          for(double z:n.z_history)out<<','<<z;
          for(int e:n.edge_type_history)out<<','<<e;
        }
      }
    }
    return out.str();
  }
};
}
static void sheaveCandidateDetectionTest() {
  auto f=housingFixture();auto cfg=housingConfig();CraneHookChainNode wire;wire.x=-.9;wire.y=.05;wire.z=4.4;WireBundleGeometry bundle;
  auto g=CranePipeline::detectSheaveCandidate(wire,f.subs,f.points,bundle,cfg.sheave,cfg.dual_roi_anchor);
  CHECK(g.status==CraneSheaveDetectionStatus::DIRECT_OBSERVED);CHECK(g.wire_supported);CHECK(g.axis_uncertain);CHECK(g.axis_status==CraneSheaveDetectionStatus::NONE);
  CHECK(std::abs(g.outer_center_z-3.625)<.01);CHECK(g.shape_eigenvalues[1]>.08);CHECK(g.point_count==245);
  auto split=housingFixture(0,true,true);cfg.sheave.surface_gap_m=.08;
  auto merged=CranePipeline::detectSheaveCandidate(wire,split.subs,split.points,bundle,cfg.sheave,cfg.dual_roi_anchor);
  CHECK(merged.point_count==g.point_count);CHECK(std::abs(merged.housing_height-.25)<1e-5);
  // No current strand support: a broad hook/cargo/noise surface alone is unknown.
  std::vector<CraneSubcluster> only={f.subs[2]};auto body_points=f.points;for(auto &p:body_points)if(p.z>3.8)p.z=0;
  auto unsupported=CranePipeline::detectSheaveCandidate(wire,only,body_points,bundle,cfg.sheave,cfg.dual_roi_anchor);
  CHECK(unsupported.status==CraneSheaveDetectionStatus::NONE);
  // Dense straight and overlapping strands cannot pass the broad-surface/PCA gate.
  for(int type=0;type<3;++type){HousingFixture noise;std::vector<PointXYZIRT> pts;
    for(int i=0;i<60;++i){PointXYZIRT p;p.x=-.9+(type==1?(i%2)*.02:(type==2?.2*i/59.:0));p.y=.05+(type==2?.2*i/59.:0);p.z=3.5+.4*i/59.;pts.push_back(p);}noise.add(1,pts);
    auto rejected=CranePipeline::detectSheaveCandidate(wire,noise.subs,noise.points,bundle,cfg.sheave,cfg.dual_roi_anchor);CHECK(rejected.status==CraneSheaveDetectionStatus::NONE);
  }
  // A transverse hook below the same two strands is not a broad housing plate.
  auto hook=housingFixture(0,false);std::vector<PointXYZIRT> hook_points;
  for(int i=0;i<5;++i)for(int j=0;j<7;++j)for(int k=0;k<5;++k){PointXYZIRT p;p.x=-.94+.08*i/4.;p.y=-.05+.2*j/6.;p.z=3.5+.25*k/4.;hook_points.push_back(p);}
  hook.add(30,hook_points);
  auto hook_geom=CranePipeline::detectSheaveCandidate(wire,hook.subs,hook.points,bundle,cfg.sheave,cfg.dual_roi_anchor);
  CHECK(hook_geom.status==CraneSheaveDetectionStatus::NONE);
  HousingFixture rotated;
  for(const auto & sc:f.subs){std::vector<PointXYZIRT> pts;for(int i:sc.point_indices){auto p=f.points[i];double x=p.x;p.x=-p.y;p.y=x;pts.push_back(p);}rotated.add(sc.subcluster_id,pts);}
  CraneHookChainNode rotated_wire=wire;rotated_wire.x=-wire.y;rotated_wire.y=wire.x;
  auto rotated_geom=CranePipeline::detectSheaveCandidate(rotated_wire,rotated.subs,rotated.points,bundle,cfg.sheave,cfg.dual_roi_anchor);
  CHECK(rotated_geom.status==CraneSheaveDetectionStatus::DIRECT_OBSERVED);
  HousingGeometryMemory mem;mem.initialized=true;mem.length=.05;mem.width=.02;mem.height=.05;mem.housing_center_x=g.outer_center_x;mem.housing_center_y=g.outer_center_y;mem.housing_center_z=g.outer_center_z;
  auto mismatch=CranePipeline::detectSheaveCandidate(wire,f.subs,f.points,bundle,cfg.sheave,cfg.dual_roi_anchor,nullptr,&mem);CHECK(mismatch.status==CraneSheaveDetectionStatus::NONE);
  std::cout<<"sheaveCandidateDetectionTest passed (surface connectivity, PCA, strand support, unknown axis, hard mismatch gate)\n";
}
static void fixedRootSheaveTerminationTest() {
  CraneHookChain original;original.root_z=5.25;
  for(double z:{5.25,4.4,3.75,2.5}){CraneHookChainNode n;n.x=-.9;n.y=.05;n.z=z;n.subcluster_id=original.nodes.size()+1;original.nodes.push_back(n);}original.nodes.front().is_root=true;
  CraneHookChainNode terminal;terminal.sheave.status=CraneSheaveDetectionStatus::DIRECT_OBSERVED;terminal.sheave.wire_supported=true;
  terminal.sheave.axis_center_x=-.9;terminal.sheave.axis_center_y=.05;terminal.sheave.axis_center_z=3.6;
  auto output=original;CHECK(!CranePipeline::appendSheaveTerminal(output,terminal));CHECK(output.nodes.size()==original.nodes.size());
  // Explicit synthetic axis truth tests only the output gate; Raw axis remains unknown.
  terminal.sheave.axis_status=CraneSheaveDetectionStatus::DIRECT_OBSERVED;terminal.sheave.axis_uncertain=false;
  CHECK(CranePipeline::appendSheaveTerminal(output,terminal));CHECK(output.nodes.back().is_sheave);CHECK(output.nodes.back().z==3.6);CHECK(original.nodes.back().z==2.5);
  for(const auto&n:output.nodes)CHECK(n.z>=3.6);
  terminal.is_held=true;auto held=original;CHECK(!CranePipeline::appendSheaveTerminal(held,terminal));CHECK(held.nodes.size()==original.nodes.size());
  terminal.is_held=false;terminal.sheave.axis_center_x=10;CHECK(!CranePipeline::appendSheaveTerminal(held,terminal));
  std::cout<<"fixedRootSheaveTerminationTest passed (output copy only, trusted synthetic axis, held/disconnected rejection)\n";
}
static void sheaveHousingGeometryMemoryAndInvarianceTest() {
  for(bool fixed:{true,false})for(bool stabilize:{true,false}){
    auto cfg=housingConfig(fixed);cfg.temporal_stabilization.temporal_stabilization_enabled=stabilize;
    auto off=cfg;off.sheave.enabled=false;CranePipeline a(off),b(cfg);
    bool observed=false,confirmed=false,relocated=false,held=false,cleared=false;
    for(int frame=0;frame<65;++frame){
      // Direct observations, vertical lowering, two misses, reacquisition, complete loss.
      bool body=frame<18 || (frame>=20 && frame<25);
      auto f=housingFixture(frame>=6?-.025*std::min(frame-6,10):0,body);
      if(frame>=30){f.points.clear();f.subs.clear();}
      auto x=a.processHookAndBeamDetection(f.points,f.subs,1+frame*.1);
      auto y=b.processHookAndBeamDetection(f.points,f.subs,1+frame*.1);
      sameWireOutput(x.top_down_hook,y.top_down_hook);
      CHECK(CranePipelineRegressionAccess::wireState(a)==CranePipelineRegressionAccess::wireState(b));
      const auto & h=y.top_down_hook.left_housing;const auto & m=y.top_down_hook.left_housing_memory;
      for(int side=0;side<2;++side){const auto & housing=side==0?y.top_down_hook.left_housing:y.top_down_hook.right_housing;
        const auto & hook=side==0?y.work_status.left_hook:y.work_status.right_hook;
        const bool valid=housing.sheave.status==CraneSheaveDetectionStatus::DIRECT_OBSERVED || housing.sheave.status==CraneSheaveDetectionStatus::HELD;
        CHECK(hook.hook_detected==valid);
        if(valid){CHECK(hook.hook_x==housing.x && hook.hook_y==housing.y && hook.hook_z==housing.z-.5);
          CHECK(hook.hook_observation==(housing.is_held?"VIRTUAL_HELD":"VIRTUAL_DIRECT"));}
      }

      if(frame==0)CHECK(!m.confirmed);
      if(frame==1 && fixed)CHECK(m.confirmed);
      observed|=h.sheave.status==CraneSheaveDetectionStatus::DIRECT_OBSERVED;confirmed|=m.confirmed;relocated|=h.relocate_count>0;held|=h.is_held;
      if(frame==19 && m.initialized){CHECK(m.confirm_count==0);CHECK(m.last_direct_stamp<2.8);CHECK(h.sheave.point_indices.empty());}
      if(frame==29){CHECK(m.dormant && !m.confirmed && !h.anchor_locked);CHECK(h.sheave.status==CraneSheaveDetectionStatus::NONE);}
      if(frame==64){CHECK(!m.initialized && !h.anchor_locked);cleared=true;}
      for(const auto & c:y.top_down_hook.chains)CHECK(!c.has_sheave);
    }
    if(fixed){CHECK(observed);CHECK(confirmed);CHECK(relocated);CHECK(held);CHECK(cleared);}
  }
  // Left lowering/dropout cannot relocate or erase the right housing identity.
  {auto cfg=housingConfig();CranePipeline p(cfg);double right_z=0;
    for(int frame=0;frame<18;++frame){auto f=housingFixture(frame>=4?-.02*std::min(frame-4,10):0,frame<14,false,false);
      auto r=p.processHookAndBeamDetection(f.points,f.subs,1+frame*.1);
      if(frame==2)right_z=r.top_down_hook.right_housing.anchor_z;
      if(frame>2){CHECK(r.top_down_hook.right_housing_memory.confirmed);CHECK(r.top_down_hook.right_housing.anchor_z==right_z);CHECK(r.top_down_hook.right_housing.relocate_count==0);}
      if(frame==17){CHECK(r.top_down_hook.left_housing_memory.dormant);CHECK(!r.top_down_hook.left_housing.anchor_locked);}
    }
  }
  // An accumulated surface without current raw support cannot generate hits.
  {auto cfg=housingConfig();CranePipeline p(cfg);auto old=housingFixture();auto current=housingFixture(0,false);
    for(int i=0;i<2;++i)p.processHookAndBeamDetection(old.points,old.subs,1+i*.1,nullptr,&old.points);
    for(int i=0;i<3;++i){auto r=p.processHookAndBeamDetection(old.points,old.subs,1.2+i*.1,nullptr,&current.points);
      CHECK(r.top_down_hook.left_housing_memory.confirm_count==0);
      CHECK(r.top_down_hook.left_housing.sheave.point_indices.empty());
      if(i==2){CHECK(r.top_down_hook.left_housing_memory.dormant);CHECK(!r.top_down_hook.left_housing.anchor_locked);}
    }
  }
  {auto cfg=housingConfig();CranePipeline p(cfg);auto f=housingFixture();
    p.processHookAndBeamDetection(f.points,f.subs,1);auto old=p.processHookAndBeamDetection(f.points,f.subs,1.1);
    auto duplicate=p.processHookAndBeamDetection(f.points,f.subs,1.1);
    CHECK(duplicate.top_down_hook.left_housing_memory.confirm_count==0);
    CHECK(duplicate.top_down_hook.left_housing.sheave.status==CraneSheaveDetectionStatus::HELD);
    CHECK(duplicate.top_down_hook.left_housing_memory.track_id==old.top_down_hook.left_housing_memory.track_id);
  }
  // Held Wire indices belong to the old cloud and must never feed current PCA.
  {auto cfg=housingConfig();CranePipeline p(cfg);auto f=housingFixture();
    for(int i=0;i<4;++i)p.processHookAndBeamDetection(f.points,f.subs,1+i*.1);
    HousingFixture roots;for(const auto&sc:f.subs)if(sc.subcluster_id==1 || sc.subcluster_id==11){std::vector<PointXYZIRT> pts;for(int i:sc.point_indices)pts.push_back(f.points[i]);roots.add(sc.subcluster_id,pts);}
    auto held=p.processHookAndBeamDetection(roots.points,roots.subs,1.4);
    for(const auto&c:held.top_down_hook.chains){for(const auto&n:c.nodes)if(n.is_held)CHECK(n.point_indices.empty());CHECK(std::isfinite(c.largest_cluster.length));CHECK(c.largest_cluster.length<20);}
  }
  // Seeking timestamps clears or quarantines identity rather than increasing confirmation.
  auto cfg=housingConfig();CranePipeline p(cfg);auto f=housingFixture();
  p.processHookAndBeamDetection(f.points,f.subs,1);p.processHookAndBeamDetection(f.points,f.subs,1.1);
  auto gap=p.processHookAndBeamDetection(f.points,f.subs,10);CHECK(!gap.top_down_hook.left_housing_memory.confirmed);
  auto rewind=p.processHookAndBeamDetection(f.points,f.subs,1);CHECK(!rewind.top_down_hook.left_housing_memory.confirmed);
  cfg.sheave.enabled=false;p.updateConfig(cfg);auto disabled=p.processHookAndBeamDetection(f.points,f.subs,1.1);CHECK(!disabled.top_down_hook.left_housing_memory.initialized);
  std::cout<<"sheaveHousingGeometryMemoryAndInvarianceTest passed (fixed/dynamic, stabilization ON/OFF, exact wire state equality, lowering, finite hold, expiry, timestamps)\n";
}

static void housingReferenceLibraryTest() {
  auto f=housingFixture();auto c=housingConfig();CraneHookChainNode wire;wire.x=-.9;wire.y=.05;wire.z=4.4;WireBundleGeometry bundle;
  auto raw=CranePipeline::detectSheaveCandidate(wire,f.subs,f.points,bundle,c.sheave,c.dual_roi_anchor);
  CHECK(raw.status==CraneSheaveDetectionStatus::DIRECT_OBSERVED);
  HousingShapeReference r;r.id="reviewed_a";r.label="USER_SEEDED";r.manually_checked=true;r.family_id=7;r.reliability=1;
  r.dimensions={raw.housing_length,raw.housing_width,raw.housing_height};r.spectrum=raw.shape_eigenvalues;r.z_distribution=raw.z_distribution;r.horizontal_distribution=raw.horizontal_distribution;
  if(raw.wire_dir_x<0)std::reverse(r.horizontal_distribution.begin(),r.horizontal_distribution.end());
  c.sheave.reference_matching_enabled=true;c.sheave.references={r};auto g=raw;
  CHECK(CranePipeline::matchHousingReference(g,c.sheave));CHECK(g.reference_id==r.id && g.reference_trusted && g.reference_family_id==7);
  auto axis_reference=r;axis_reference.horizontal_distribution={.80,.15,.05};
  c.sheave.references={axis_reference};
  auto reversed_axis=raw;reversed_axis.wire_dir_x=1;reversed_axis.wire_dir_y=0;
  reversed_axis.horizontal_distribution={.05,.15,.80};
  CHECK(CranePipeline::matchHousingReference(reversed_axis,c.sheave));
  CHECK(reversed_axis.reference_family_id==r.family_id);
  CHECK(reversed_axis.reference_score<1e-9);
  c.sheave.references={r};
  auto local_reference=r;local_reference.has_local_shape=true;
  local_reference.local_extent=raw.local_extent;local_reference.local_occupancy=raw.local_occupancy;
  c.sheave.references={local_reference};auto local_surface=raw;
  CHECK(CranePipeline::matchHousingReference(local_surface,c.sheave));
  auto angular_reference=local_reference;angular_reference.sensor_range=3;
  angular_reference.point_count=5.*raw.point_count;c.sheave.references={angular_reference};
  auto sparse_patch=raw;sparse_patch.sensor_range=3;
  CHECK(!CranePipeline::matchHousingReference(sparse_patch,c.sheave));
  auto farther_observation=raw;farther_observation.sensor_range=6;
  CHECK(CranePipeline::matchHousingReference(farther_observation,c.sheave));
  c.sheave.references={local_reference};
  auto negative_reference=local_reference;negative_reference.id="verified_non_housing";negative_reference.label="NEGATIVE";
  c.sheave.references={local_reference,negative_reference};auto ambiguous_class=raw;
  CHECK(!CranePipeline::matchHousingReference(ambiguous_class,c.sheave));
  c.sheave.references={negative_reference};auto negative_only=raw;
  CHECK(!CranePipeline::matchHousingReference(negative_only,c.sheave));
  c.sheave.references={local_reference};
  auto same_moments_wrong_surface=raw;same_moments_wrong_surface.local_occupancy.fill(0);
  same_moments_wrong_surface.local_occupancy[13]=1;
  CHECK(!CranePipeline::matchHousingReference(same_moments_wrong_surface,c.sheave));
  auto asymmetric_reference=local_reference;asymmetric_reference.local_occupancy.fill(0);
  asymmetric_reference.local_occupancy[0]=.8;asymmetric_reference.local_occupancy[5]=.2;
  c.sheave.references={asymmetric_reference};auto sign_flipped=raw;sign_flipped.local_occupancy.fill(0);
  sign_flipped.local_occupancy[26]=.8;sign_flipped.local_occupancy[21]=.2;
  CHECK(CranePipeline::matchHousingReference(sign_flipped,c.sheave));
  CHECK(sign_flipped.reference_local_score<1e-9);
  // A reviewed current 3D surface plus one actual attached strand is distinct
  // from approving an arbitrary Chain node. No raw strand/no trusted surface
  // must still reject, even with an otherwise matching reference.
  auto one_strand=f;
  for(const auto & sc:one_strand.subs)if(sc.subcluster_id==2)
    for(int i:sc.point_indices)one_strand.points[i].z=std::numeric_limits<float>::quiet_NaN();
  for(int i=0;i<20;++i){PointXYZIRT p;p.x=-1.;p.y=.05;p.z=3.84+.18*i/19.;one_strand.points.push_back(p);}
  c.sheave.references={local_reference};c.sheave.reference_wire_mode_search=true;
  auto supported_surface=CranePipeline::detectSheaveCandidate(wire,one_strand.subs,one_strand.points,bundle,c.sheave,c.dual_roi_anchor,nullptr,nullptr,&one_strand.points);
  CHECK(supported_surface.status==CraneSheaveDetectionStatus::DIRECT_OBSERVED);
  CHECK(supported_surface.wire_observation_mode==3 && supported_surface.reference_trusted);
  c.sheave.references[0].label="PROBABLE";
  auto unreviewed_surface=CranePipeline::detectSheaveCandidate(wire,one_strand.subs,one_strand.points,bundle,c.sheave,c.dual_roi_anchor,nullptr,nullptr,&one_strand.points);
  CHECK(unreviewed_surface.status==CraneSheaveDetectionStatus::NONE);
  c.sheave.reference_wire_mode_search=false;c.sheave.references={r};
  // Even a broad current surface with two observed wires cannot initialize a
  // Housing/Virtual Hook when the configured library rejects its geometry.
  auto incompatible=r;incompatible.dimensions={3.,3.,3.};
  c.sheave.references={incompatible};
  auto unmatched=CranePipeline::detectSheaveCandidate(wire,f.subs,f.points,bundle,c.sheave,c.dual_roi_anchor,nullptr,nullptr,&f.points);
  CHECK(unmatched.status==CraneSheaveDetectionStatus::NONE);
  CHECK(unmatched.rejection_counts[7]>0);
  CranePipeline rejected_pipeline(c);
  for(int i=0;i<5;++i){
    auto rejected=rejected_pipeline.processHookAndBeamDetection(f.points,f.subs,10.+i*.1);
    CHECK(!rejected.top_down_hook.left_housing_memory.initialized);
    CHECK(!rejected.work_status.left_hook.hook_detected);
  }
  c.sheave.references={r};
  auto alternate=r;alternate.id="reviewed_b";alternate.spectrum={.42,.38,.20};alternate.horizontal_distribution={.25,.50,.25};c.sheave.references.push_back(alternate);
  auto different=raw;different.shape_eigenvalues=alternate.spectrum;different.horizontal_distribution=alternate.horizontal_distribution;
  CHECK(CranePipeline::matchHousingReference(different,c.sheave));CHECK(different.reference_id==alternate.id);
  auto incompatible_surface=raw;incompatible_surface.z_distribution={1,0,0};
  c.sheave.references[0].z_distribution={0,0,1};c.sheave.references.resize(1);
  CHECK(!CranePipeline::matchHousingReference(incompatible_surface,c.sheave));c.sheave.references={r};
  auto no_evidence=raw;no_evidence.point_count=0;CHECK(!CranePipeline::matchHousingReference(no_evidence,c.sheave));
  auto wrong_size=raw;wrong_size.housing_length*=4;CHECK(!CranePipeline::matchHousingReference(wrong_size,c.sheave));
  c.sheave.references={r};c.sheave.references[0].label="AMBIGUOUS";auto ambiguous=raw;CHECK(!CranePipeline::matchHousingReference(ambiguous,c.sheave));
  // Sparse returns between two dense current strands defeat a largest-empty-gap
  // rule. The reviewed path needs both vertically overlapping modes, not memory.
  auto bridged=f;
  for(int i=1;i<15;++i){PointXYZIRT p;p.x=-1.05+.30*i/15.;p.y=.05;p.z=4.05+.20*(i%4)/3.;bridged.points.push_back(p);}
  auto legacy=CranePipeline::detectSheaveCandidate(wire,bridged.subs,bridged.points,bundle,c.sheave,c.dual_roi_anchor,nullptr,nullptr,&bridged.points);
  CHECK(legacy.status==CraneSheaveDetectionStatus::NONE);
  c.sheave.references={r};c.sheave.reference_wire_mode_search=true;
  auto reviewed=CranePipeline::detectSheaveCandidate(wire,bridged.subs,bridged.points,bundle,c.sheave,c.dual_roi_anchor,nullptr,nullptr,&bridged.points);
  CHECK(reviewed.status==CraneSheaveDetectionStatus::DIRECT_OBSERVED);CHECK(reviewed.reference_assisted && reviewed.reference_trusted);CHECK(reviewed.axis_status==CraneSheaveDetectionStatus::NONE);
  c.sheave.references[0].label="PROBABLE";
  auto unverified=CranePipeline::detectSheaveCandidate(wire,bridged.subs,bridged.points,bundle,c.sheave,c.dual_roi_anchor,nullptr,nullptr,&bridged.points);
  CHECK(unverified.status==CraneSheaveDetectionStatus::NONE);
  // Reference availability and a debug seed cannot manufacture current evidence.
  c.sheave.reference_wire_mode_search=false;c.sheave.references={r};c.sheave.debug_seed_world={-100,200,300};
  CranePipeline p(c);auto empty=p.processHookAndBeamDetection({}, {},1);
  CHECK(empty.top_down_hook.left_housing.sheave.status==CraneSheaveDetectionStatus::NONE);
  auto first=p.processHookAndBeamDetection(f.points,f.subs,1.1);auto bank=p.config().sheave.references;
  for(int i=0;i<6;++i)p.processHookAndBeamDetection({}, {},1.2+i*.1);
  CHECK(p.config().sheave.references.size()==bank.size());CHECK(p.config().sheave.references[0].spectrum==bank[0].spectrum);
  std::cout<<"housingReferenceLibraryTest passed (bounded immutable observations, provenance, alternate features, current paired modes, no phantom evidence)\n";
}

static void craneWorkCycleTest(){
  auto cfg=housingConfig();cfg.work_status.fsm_enabled=true;cfg.work_status.confirm_m_on=3;
  cfg.roi_filter.beam_roi_x_min=-3;cfg.roi_filter.beam_roi_x_max=3;
  struct Scene {std::vector<PointXYZIRT> points;std::vector<CraneSubcluster> subs;};
  auto scene=[&](unsigned hooks,unsigned cargos,double dx=0,bool partial=false,bool shared=false,bool sling=false){
    Scene f;
    auto add=[&](int id,double x,double y,double z,double l,double w,double h){
      CraneSubcluster sc;sc.subcluster_id=id;sc.min_x=x-l*.5;sc.max_x=x+l*.5;
      sc.min_y=y-w*.5;sc.max_y=y+w*.5;sc.min_z=z-h*.5;sc.max_z=z+h*.5;
      sc.centroid_x=x;sc.centroid_y=y;sc.centroid_z=z;
      const int nx=std::max(2,int(l/.02)),ny=std::max(2,int(w/.02)),nz=h>.03?4:1;
      for(int a=0;a<=nx;++a)for(int b=0;b<=ny;++b)for(int c=0;c<nz;++c){PointXYZIRT p;
        p.x=sc.min_x+l*a/nx;p.y=sc.min_y+w*b/ny;p.z=nz==1?z:sc.min_z+h*c/(nz-1);
        sc.point_indices.push_back(f.points.size());f.points.push_back(p);}
      f.subs.push_back(sc);
    };
    for(int side=0;side<2;++side)if(hooks&(1u<<side)){
      const double x=side==0?-.4:.4;add(10+side,x,0,4.02,.16,.12,.16);
      for(int i=0;i<10;++i){PointXYZIRT p;p.x=x;p.y=0;p.z=4.11+.018*i;f.points.push_back(p);}
    }
    if(shared && cargos){add(70,dx,0,3.3,1.2,partial?.2:.4,.02);
      for(int side=0;side<2;++side)for(int i=0;i<30;++i){PointXYZIRT p;p.x=side==0?-.4:.4;p.y=0;p.z=3.38+.016*i;f.points.push_back(p);}
    }
    else for(int side=0;side<2;++side)if(cargos&(1u<<side)){
      add(70+side,(side==0?-.4:.4)+dx,0,3.3,partial?.3:.6,partial?.2:.4,.02);
      for(int i=0;i<30;++i){PointXYZIRT p;p.x=side==0?-.4:.4;p.y=0;p.z=3.38+.016*i;f.points.push_back(p);}
    }
    if(sling){CraneSubcluster sc;sc.subcluster_id=90;sc.min_x=-.7;sc.max_x=-.1;sc.min_y=-.3;sc.max_y=.3;sc.min_z=2.9;sc.max_z=3.7;sc.centroid_x=-.4;sc.centroid_z=3.3;
      for(int i=0;i<100;++i)for(int sign:{-1,1}){PointXYZIRT p;const double a=i/99.;p.x=-.4+sign*.3*a;p.y=sign*.3*a;p.z=3.7-.8*a;sc.point_indices.push_back(f.points.size());f.points.push_back(p);}f.subs.push_back(sc);}
    return f;
  };
  auto run=[&](CranePipeline & p,Scene f,double stamp,unsigned housings=3){
    return CranePipelineRegressionAccess::work(p,f.points,f.subs,stamp,housings);
  };
  CranePipeline p(cfg);double t=1;
  for(int i=0;i<30;++i){auto w=run(p,scene(1,0,0,false,false,true),t+=.1);CHECK(!w.work_status);CHECK(!w.geometry_locked);CHECK(w.left_hook.display_state=="WAITING");CHECK(!w.left_hook.work_status);}
  std::vector<std::string> transitions;CraneBeamState last=CraneBeamState::IDLE;
  for(int i=0;i<26;++i){auto w=run(p,scene(1,1,.002*i),t+=.1);
    if(w.fsm_state!=last){transitions.push_back(to_string(w.fsm_state));last=w.fsm_state;}
    if(i<2)CHECK(!w.work_status);if(i==2)CHECK(w.fsm_state==CraneBeamState::READY);
    if(i<20)CHECK(!w.geometry_locked);
  }
  CranePipeline both_separation(cfg);CranePipelineRegressionAccess::bothHooksMustSeparate(both_separation);
  auto locked=p.result().work_status;CHECK(locked.geometry_locked);CHECK(locked.cargo_side=="LEFT");
  CHECK(locked.left_hook.display_state=="PREPARING" && locked.left_hook.work_status);
  CHECK(locked.right_hook.display_state=="WAITING" && !locked.right_hook.work_status);
  // Size Lock / elapsed confirmation alone never means STARTING. Coherent
  // direct Housing + Hook + Cargo ascent with stable gaps is separate evidence.
  auto shifted=[&](double z,bool cargo_only=false){auto f=scene(1,1);
    for(auto & point:f.points)if(!cargo_only || point.z<3.9)point.z+=z;
    for(auto & sc:f.subs)if(!cargo_only || sc.subcluster_id==70){sc.min_z+=z;sc.max_z+=z;sc.centroid_z+=z;}
    return f;
  };
  CranePipeline rising(cfg),cargo_alone(cfg),stationary(cfg);bool saw_start=false;
  for(int i=0;i<25;++i){run(rising,scene(1,1),1+i*.1);run(cargo_alone,scene(1,1),1+i*.1);
    auto still=run(stationary,scene(1,1),1+i*.1);CHECK(still.left_hook.display_state!="STARTING");}
  for(int i=1;i<=10;++i){const double z=.04*i;auto f=shifted(z);
    auto moving=CranePipelineRegressionAccess::work(rising,f.points,f.subs,3.4+i*.1,1,.4,z);
    CHECK(moving.right_hook.display_state=="WAITING");
    if(moving.left_hook.display_state=="STARTING"){
      saw_start=true;CHECK(moving.work_status);CHECK(moving.left_hook.hook_rise>.24);
      CHECK(moving.left_hook.cargo_rise>.24);CHECK(moving.left_hook.start_evidence=="DIRECT_HOUSING_HOOK_CARGO_Z_RISE_WITH_STABLE_GAPS");
    }
    auto alone=run(cargo_alone,shifted(z,true),3.4+i*.1);CHECK(alone.left_hook.display_state!="STARTING");
  }
  CHECK(saw_start);
  // A single relocation followed by still frames is not repeated lift evidence.
  CranePipeline pulse(cfg);for(int i=0;i<25;++i)run(pulse,scene(1,1),1+i*.1);
  for(int i=0;i<8;++i){auto f=shifted(.30);auto v=CranePipelineRegressionAccess::work(pulse,f.points,f.subs,3.5+i*.1,1,.4,.30);
    CHECK(v.left_hook.display_state!="STARTING");}
  auto duplicate_start=CranePipelineRegressionAccess::work(rising,shifted(.4).points,shifted(.4).subs,4.4,1,.4,.4);
  CHECK(duplicate_start.left_hook.display_state!="STARTING");
  // Thin, spread strands can have a large bounding box and occupied corners.
  // They must not become a filled Cargo surface just by being connected.
  auto outline=scene(1,1);
  outline.points.erase(std::remove_if(outline.points.begin(),outline.points.end(),[](const auto & v){
    return std::abs(v.z-3.3)<.015 && std::abs(v.x+.4)<.28 && std::abs(v.y)<.18;
  }),outline.points.end());
  CranePipeline hollow(cfg);for(int i=0;i<25;++i){auto v=run(hollow,outline,1+i*.1);CHECK(!v.work_status);}
  auto missing=run(rising,scene(0,0),4.5,0);CHECK(missing.left_hook.display_state!="STARTING");

  // Drop is not Cargo/Hook co-motion or blind loss. Exercise positive descent,
  // current support input and DIRECT separated Housing, then exact sensor-time hold.
  CranePipeline placed(cfg);double place_t=10;CraneWorkStatusResult pw;
  for(int i=0;i<25;++i)pw=run(placed,scene(2,2),place_t+=.1,2);
  auto lowered=[&](double dz){auto f=scene(2,2);for(auto & v:f.points)v.z+=dz;
    for(auto & sc:f.subs){sc.min_z+=dz;sc.max_z+=dz;sc.centroid_z+=dz;}return f;};
  for(int i=1;i<=8;++i){auto f=lowered(-.04*i);
    pw=CranePipelineRegressionAccess::work(placed,f.points,f.subs,place_t+=.1,2,.4,-.04*i);CHECK(pw.working_status!=2);}
  auto down=lowered(-.32);bool saw_drop=false;double drop_stamp=0;
  for(int i=0;i<25;++i){pw=CranePipelineRegressionAccess::work(placed,down.points,down.subs,place_t+=.1,2,2.0,-.32);CHECK(pw.working_status!=2);}

  for(int i=0;i<80;++i){CranePipelineRegressionAccess::support(placed);
    pw=CranePipelineRegressionAccess::work(placed,down.points,down.subs,place_t+=.1,2,2.0,-.32);
    if(pw.working_status==2){saw_drop=true;drop_stamp=place_t;break;}}
  CHECK(saw_drop && pw.fsm_state==CraneBeamState::COMPLETED && !pw.work_status);
  pw=CranePipelineRegressionAccess::work(placed,{}, {},drop_stamp,0);CHECK(pw.working_status==2);
  pw=CranePipelineRegressionAccess::work(placed,{}, {},drop_stamp+2.999,0);CHECK(pw.working_status==2);
  pw=CranePipelineRegressionAccess::work(placed,{}, {},drop_stamp+3.0,0);CHECK(pw.working_status==0);
  pw=CranePipelineRegressionAccess::work(placed,{}, {},drop_stamp-1.0,0);CHECK(pw.working_status==0);

  const int id=locked.cargo_track_id;
  for(int i=0;i<4;++i){auto w=run(p,scene(1,1,.052+.02*i,true),t+=.1);CHECK(w.geometry_locked);CHECK(w.cargo_track_id==id);CHECK(w.length==locked.length && w.width==locked.width && w.height==locked.height && w.yaw==locked.yaw);}
  for(int i=0;i<3;++i){auto w=run(p,scene(0,0),t+=.1,0);CHECK(w.fsm_state!=CraneBeamState::COMPLETED);CHECK(w.cargo_observation=="HELD");CHECK(w.length==locked.length);}
  auto w=run(p,scene(1,1,.10),t+=.1);CHECK(w.cargo_track_id==id);CHECK(w.cargo_observation=="DIRECT");
  // Cargo leaving the old frozen ROI is NOT positive placement/separation evidence.
  for(int i=1;i<13;++i){w=run(p,scene(1,1,.10+i*.07),t+=.1);CHECK(w.fsm_state!=CraneBeamState::COMPLETED);}
  for(auto side:{1u,2u,3u}){CranePipeline q(cfg);for(int i=0;i<25;++i)w=run(q,scene(side,side,0,false,side==3),1+i*.1,side);
    if(!w.geometry_locked)std::cerr<<"SIDE "<<side<<" state="<<to_string(w.fsm_state)<<" hooks="<<w.left_hook.hook_detected<<w.right_hook.hook_detected<<" candidate="<<w.cargo_cluster_id<<" side="<<w.cargo_side<<" sec="<<w.confirmed_observation_sec<<" reason="<<w.status_reason<<"\n";
    CHECK(w.geometry_locked);CHECK(w.cargo_side==(side==1?"LEFT":side==2?"RIGHT":"BOTH"));CHECK(w.left_hook.work_status==bool(side&1));CHECK(w.right_hook.work_status==bool(side&2));}
  CranePipeline sling_and_cargo(cfg);
  for(int i=0;i<25;++i)w=run(sling_and_cargo,scene(1,1,0,false,false,true),1+i*.1);
  CHECK(w.work_status && w.left_hook.display_state=="PREPARING");
  // Two separate clusters inside two ROIs must never become BOTH.
  CranePipeline separate(cfg);for(int i=0;i<25;++i)w=run(separate,scene(3,3),1+i*.1);CHECK(w.cargo_side!="BOTH");
  // A broad background surface without current connecting returns is not READY.
  CranePipeline disconnected(cfg);auto no_link=scene(1,1);
  for(auto & point:no_link.points)if(point.z>3.36 && point.z<3.90)point.x+=2;
  for(int i=0;i<25;++i){w=run(disconnected,no_link,1+i*.1);CHECK(!w.work_status);}
  // Independent vertical column at the candidate end does not reach the Hook.
  // Existing count/radius limits stay unchanged; no fixed Bag target rule.
  auto offset_column=scene(1,1,.7);
  for(auto & point:offset_column.points)if(point.z>3.36 && point.z<3.95)point.x=0;
  CranePipeline wrong_link(cfg);for(int i=0;i<25;++i){auto v=run(wrong_link,offset_column,1+i*.1);CHECK(!v.work_status);}
  // A connected, filled small patch below Hook is not enough to start work.
  // Reuse existing object LENGTH OR occupied AREA limits; no Size Lock/full restoration prerequisite.
  auto small_patch=scene(1,1);for(auto & sc:small_patch.subs)if(sc.subcluster_id==70){
    sc.min_x=-.55;sc.max_x=-.25;sc.min_y=-.125;sc.max_y=.125;
    for(int id:sc.point_indices){small_patch.points[id].x=-.4+(small_patch.points[id].x+.4)*.5;small_patch.points[id].y*=.625;}}
  CranePipeline small_object(cfg);for(int i=0;i<25;++i){auto v=run(small_object,small_patch,1+i*.1);CHECK(!v.work_status);}
  // Physical Hook clustering is no longer required: use the virtual reference.
  CranePipeline no_hook(cfg);for(int i=0;i<25;++i){w=run(no_hook,scene(0,1),1+i*.1);}
  CHECK(w.work_status && w.left_hook.hook_detected);
  CranePipeline no_housing(cfg);for(int i=0;i<25;++i){w=run(no_housing,scene(1,1),1+i*.1,0);CHECK(!w.work_status && !w.left_hook.hook_detected);}

  // Current Cargo remains DIRECT across a hidden connector after work confirmation,
  // using the same geometry/KF gates; no Size Lock prerequisite is introduced.
  CranePipeline connector_loss(cfg);CraneWorkStatusResult cl;
  for(int i=0;i<6;++i)cl=run(connector_loss,scene(1,1),1+i*.1,1);
  const int connector_id=cl.cargo_track_id;
  for(int i=0;i<5;++i){cl=run(connector_loss,no_link,1.6+i*.1,1);
    CHECK(cl.work_status && cl.cargo_observation=="DIRECT" && cl.cargo_track_id==connector_id);}

  // Confirmed work survives a brief pre-lock Raw dropout, bounded by existing loss time.
  CranePipeline brief_loss(cfg);CraneWorkStatusResult bl;
  for(int i=0;i<6;++i)bl=run(brief_loss,scene(1,1),1+i*.1,1);
  const int brief_id=bl.cargo_track_id;
  CHECK(bl.work_status && !bl.geometry_locked);
  bl=run(brief_loss,scene(1,0),1.6,1);
  CHECK(bl.work_status && bl.cargo_observation=="HELD" && bl.cargo_track_id==brief_id);
  bl=run(brief_loss,scene(1,1),1.7,1);
  CHECK(bl.work_status && bl.cargo_track_id==brief_id);
  bl=run(brief_loss,scene(1,0),2.6,1);
  CHECK(!bl.work_status && bl.working_status==0);

  CranePipeline duplicate(cfg);for(int i=0;i<40;++i)w=run(duplicate,scene(1,1),1);CHECK(!w.geometry_locked);CHECK(w.confirmed_observation_sec==0);CHECK(w.cargo_observation!="DIRECT");
  CranePipeline lost(cfg);for(int i=0;i<25;++i)w=run(lost,scene(1,1),1+i*.1);
  for(int i=0;i<12;++i){w=run(lost,scene(0,0),3.5+i*.1,0);CHECK(w.fsm_state!=CraneBeamState::COMPLETED);}CHECK(!w.geometry_locked && !w.work_status);
  // Identity/geometry change before two seconds restarts the observation period.
  CranePipeline swap(cfg);for(int i=0;i<12;++i)run(swap,scene(1,1),1+i*.1);
  for(int i=0;i<10;++i)w=run(swap,scene(2,2),2.2+i*.1);CHECK(!w.geometry_locked);
  // Same object ID can change the supported side before Lock. Reconfirm it,
  // but keep the configured FSM active through the candidate-only reset.
  CranePipeline side_change(cfg);for(int i=0;i<10;++i)run(side_change,scene(1,1),1+i*.1,1);
  w=CranePipelineRegressionAccess::work(side_change,scene(1,1).points,scene(1,1).subs,2.0,2,-.4);
  CHECK(w.fsm_enabled && w.cargo_side=="RIGHT" && !w.geometry_locked);CHECK(w.right_hook.hook_detected && w.right_hook.hook_x==-.4 && w.right_hook.hook_z==4.0);
  auto narrow=cfg;narrow.work_status.cargo_roi_width_x_m=.2;CranePipeline narrow_x(narrow);
  for(int i=0;i<25;++i)w=run(narrow_x,scene(1,1,.2),1+i*.1);CHECK(w.work_status);
  narrow=cfg;narrow.work_status.cargo_roi_width_y_m=.2;CranePipeline narrow_y(narrow);
  auto shifted_y=scene(1,1);for(auto & sc:shifted_y.subs)if(sc.subcluster_id==70){
    sc.centroid_y+=.2;sc.min_y+=.2;sc.max_y+=.2;for(int index:sc.point_indices)shifted_y.points[index].y+=.2;}
  for(int i=0;i<25;++i)w=run(narrow_y,shifted_y,1+i*.1);CHECK(w.work_status);
  // Exact consumer relation, including FSM OFF, unsupported Chain/Cluster
  // changes, offset updates, held/missing positions and independent sides.
  CranePipeline virtual_only(cfg);auto v=run(virtual_only,scene(0,0),10);
  CHECK(v.left_hook.hook_detected && v.right_hook.hook_detected);
  CHECK(v.left_hook.hook_x==-.4 && v.left_hook.hook_y==0 && v.left_hook.hook_z==4.0);
  const auto before=CranePipelineRegressionAccess::wireState(virtual_only);
  auto noisy=scene(3,3,10,false,false,true);
  v=run(virtual_only,noisy,10.1);CHECK(v.left_hook.hook_x==-.4 && v.left_hook.hook_z==4.0);
  CHECK(CranePipelineRegressionAccess::wireState(virtual_only)==before);
  v=CranePipelineRegressionAccess::work(virtual_only,{}, {},10.2,3,.4,.2,1);
  CHECK(v.left_hook.hook_observation=="VIRTUAL_HELD" && std::abs(v.left_hook.hook_z-4.2)<1e-9);
  CHECK(v.right_hook.hook_observation=="VIRTUAL_DIRECT");
  v=run(virtual_only,scene(0,0),10.3,2);CHECK(!v.left_hook.hook_detected && v.right_hook.hook_detected);
  auto adjusted=cfg;adjusted.work_status.virtual_hook_offset_z_m=.8;virtual_only.updateConfig(adjusted);
  v=run(virtual_only,scene(0,0),10.4);CHECK(std::abs(v.left_hook.hook_z-3.7)<1e-9);
  adjusted.work_status.fsm_enabled=false;virtual_only.updateConfig(adjusted);
  v=run(virtual_only,scene(0,0),10.5);CHECK(!v.fsm_enabled && v.left_hook.hook_detected && std::abs(v.left_hook.hook_z-3.7)<1e-9);
  v=run(virtual_only,scene(0,0),9.0);CHECK(v.left_hook.hook_detected && std::abs(v.left_hook.hook_z-3.7)<1e-9 && !v.work_status);
  CranePipeline paused(cfg);for(int i=0;i<12;++i)run(paused,scene(1,1),1+i*.1);
  const double confirmed=paused.result().work_status.confirmed_observation_sec;
  for(int i=0;i<2;++i){auto f=scene(1,1);v=CranePipelineRegressionAccess::work(paused,f.points,f.subs,2.2+i*.1,1,.4,0,1);
    CHECK(v.work_status && !v.geometry_locked && v.confirmed_observation_sec==confirmed && v.cargo_observation=="HELD");}
  v=run(paused,scene(1,1),2.4);CHECK(v.work_status && !v.geometry_locked);
  CranePipeline paused_both(cfg);for(int i=0;i<12;++i)run(paused_both,scene(3,3,0,false,true),1+i*.1);
  auto shared=scene(3,3,0,false,true);
  v=CranePipelineRegressionAccess::work(paused_both,shared.points,shared.subs,2.2,3,.4,0,1);
  CHECK(v.work_status && v.cargo_side=="BOTH" && v.cargo_observation=="HELD" && !v.geometry_locked);
  auto invalid_offset=cfg;invalid_offset.work_status.virtual_hook_offset_z_m=-1;bool invalid=false;
  try{CranePipeline rejected(invalid_offset);}catch(const std::invalid_argument &){invalid=true;}CHECK(invalid);
  auto bad=cfg;bad.work_status.cargo_confirm_sec=0;bool threw=false;try{CranePipeline q(bad);}catch(const std::invalid_argument &){threw=true;}CHECK(threw);
  std::cout<<"craneWorkCycleTest passed: sling/outline/no-hook negatives, sling+cargo, coherent STARTING, pulse/duplicate/loss rejection, READY, timestamp 2s, fixed L/W/H/yaw, ID, hold/loss, LEFT RIGHT BOTH, separate objects, outside/reset; transitions:";
  for(const auto & s:transitions)std::cout<<' '<<s;std::cout<<'\n';
}

static void craneBoundaryMotionTest(){
  auto c=housingConfig();c.work_status.boundary_motion_enabled=true;c.work_status.boundary_search_radius_m=.6;c.work_status.cargo_loss_sec=.8;
  c.roi_filter.beam_roi_x_min=-2;c.roi_filter.beam_roi_x_max=2;c.roi_filter.beam_roi_y_min=-2;c.roi_filter.beam_roi_y_max=2;
  c.top_down_hook.root_move_velocity_thresh=.03;
  const std::array<std::array<double,2>,8> slots{{{{-2,-2}},{{0,-2}},{{2,-2}},{{2,0}},{{2,2}},{{0,2}},{{-2,2}},{{-2,0}}}};
  auto raw=[&](double shift,double z=4,unsigned mask=255){std::vector<PointXYZIRT> points;
    for(int i=0;i<8;++i)if(mask&(1u<<i))for(int a=-2;a<=2;++a)for(int b=-2;b<=2;++b){PointXYZIRT p;p.x=slots[i][0]+shift+a*.008;p.y=slots[i][1]+b*.008;p.z=z;points.push_back(p);}return points;};
  CranePipeline p(c);CraneWorkStatusResult w;
  for(int i=0;i<5;++i)w=CranePipelineRegressionAccess::work(p,raw(0),{},1+i*.1,0);
  CHECK(!w.boundary_moving);
  for(int i=1;i<8;++i)w=CranePipelineRegressionAccess::work(p,raw(i*.02),{},1.4+i*.1,0);
  CHECK(w.boundary_moving);
  w=CranePipelineRegressionAccess::work(p,raw(.16,4,15),{},2.2,0);CHECK(w.boundary_moving);
  // Loss exceeds bounded hold; reacquire at fixed slots, no displacement jump.
  for(int i=0;i<10;++i)w=CranePipelineRegressionAccess::work(p,{}, {},2.3+i*.1,0);
  w=CranePipelineRegressionAccess::work(p,raw(.4),{},3.4,0);CHECK(!w.boundary_moving);
  for(int i=0;i<8;++i)CHECK(w.boundary_points[i].observation=="REACQUIRED");
  for(int i=0;i<8;++i)w=CranePipelineRegressionAccess::work(p,raw(.4,4+i*.02),{},3.5+i*.1,0);
  CHECK(!w.boundary_moving);
  // Exercise the count from actual current patch measurements, not only injected bools.
  for(unsigned moving:{1u,3u}){
    CranePipeline q(c);
    for(int i=0;i<5;++i)CranePipelineRegressionAccess::work(q,raw(0),{},10+i*.1,0);
    auto measured=raw(.02,4,moving);const auto stable=raw(0,4,255u^moving);
    measured.insert(measured.end(),stable.begin(),stable.end());
    auto current=CranePipelineRegressionAccess::work(q,measured,{},10.5,0);
    CHECK(q.confirmedCraneMoving()==(moving==3u));CHECK(current.boundary_moving==q.confirmedCraneMoving());
    CranePipelineRegressionAccess::work(q,{}, {},10.6,0);CHECK(!q.confirmedCraneMoving());
  }
  auto overlap=c;overlap.roi_filter.beam_roi_x_min=overlap.roi_filter.beam_roi_y_min=-.01;
  overlap.roi_filter.beam_roi_x_max=overlap.roi_filter.beam_roi_y_max=.01;
  CranePipeline q(overlap);
  auto one_patch=[](double shift){std::vector<PointXYZIRT> points;
    for(int a=-2;a<=2;++a)for(int b=-2;b<=2;++b){PointXYZIRT p;p.x=shift+a*.008;p.y=b*.008;p.z=4;points.push_back(p);}return points;};
  for(int i=0;i<5;++i)CranePipelineRegressionAccess::work(q,one_patch(0),{},20+i*.1,0);
  CranePipelineRegressionAccess::work(q,one_patch(.02),{},20.5,0);CHECK(!q.confirmedCraneMoving());
  std::cout<<"craneBoundaryMotionTest passed: eight actual surface patches, independent 1/2 moving counts, shared-Raw dedup, bounded loss, reacquisition, Z-only no move\n";
}

static void cargoExtensionAndGapTest(){
  auto cfg=housingConfig();cfg.work_status.fsm_enabled=true;cfg.work_status.confirm_m_on=3;cfg.work_status.cargo_confirm_sec=2.0;
  cfg.z_gaussian.sigma_z=.005;cfg.z_gaussian.range_sigma_factor=0;cfg.z_gaussian.max_layer_z_spread=.02;
  cfg.dual_roi_anchor.noise_deadband_z=.01;
  struct Scene {std::vector<PointXYZIRT> raw;std::vector<CraneSubcluster> clusters;};
  auto patch=[](Scene & s,double x0,double x1,double y0,double y1,double z,int id){
    CraneSubcluster sc;sc.subcluster_id=id;sc.min_x=x0;sc.max_x=x1;sc.min_y=y0;sc.max_y=y1;sc.min_z=sc.max_z=sc.centroid_z=z;
    sc.centroid_x=(x0+x1)*.5;sc.centroid_y=(y0+y1)*.5;
    for(double x=x0;x<=x1+1e-6;x+=.025)for(double y=y0;y<=y1+1e-6;y+=.025){PointXYZIRT p;p.x=x;p.y=y;p.z=z;sc.point_indices.push_back(s.raw.size());s.raw.push_back(p);}
    s.clusters.push_back(std::move(sc));
  };
  Scene s;patch(s,-.5,.5,-.15,.15,1,1);patch(s,.8,1.8,-.15,.15,1,2);
  patch(s,.8,1.8,.40,.70,1,3); // same height, wrong lateral axis
  patch(s,3,4,-.15,.15,1,4); // same direction, unsupported long gap
  patch(s,.8,1.8,-.15,.15,1.5,5); // different layer
  CranePipeline p(cfg);CranePipelineRegressionAccess::seedExtension(p);CranePipelineRegressionAccess::seedRaw(p,s.clusters.front().point_indices);
  const auto wire=CranePipelineRegressionAccess::wireState(p);
  for(int i=0;i<4;++i){
    const auto & layers=p.prepareCargoExtension(s.raw,tf2::Transform::getIdentity());CHECK(!layers.layers.empty());
    for(const auto & layer:layers.layers)for(int id:layer.point_indices)CHECK(s.raw[id].z<1.1);
    auto subs=s.clusters;for(auto & sc:subs)sc.subcluster_id+=i*100; // no dependence on unstable per-frame cluster IDs
    p.updateCargoExtension(s.raw,subs,1+i*.1);
  }
  auto w=p.result().work_status;CHECK(w.cargo_track_id==41 && w.extension_valid);
  CHECK(w.extended_length>2.2 && w.extended_length<2.4 && w.extended_width<.35);
  CHECK(w.extension_cluster_ids.size()==2 && !w.inferred_gaps.empty());
  CHECK(w.extended_length>w.observed_length_sum+.2);
  for(int id:w.extended_raw_indices)CHECK(s.raw[id].x<2 && s.raw[id].y<.2 && s.raw[id].z<1.1);
  CHECK(w.gap_diagnostic=="UNKNOWN" && w.work_status);
  CHECK(CranePipelineRegressionAccess::wireState(p)==wire);
  const double held_length=w.extended_length;
  Scene partial;patch(partial,-.5,.5,-.15,.15,1,1);
  p.prepareCargoExtension(partial.raw,tf2::Transform::getIdentity());p.updateCargoExtension(partial.raw,partial.clusters,1.4);
  auto partial_w=p.result().work_status;CHECK(partial_w.extension_observation=="PARTIAL_HELD" && partial_w.extended_length==held_length && partial_w.cargo_track_id==41);
  CHECK(partial_w.gap_diagnostic=="UNKNOWN" && partial_w.extended_raw_indices.size()==partial.raw.size());
  for(int i=0;i<4;++i){p.prepareCargoExtension(s.raw,tf2::Transform::getIdentity());p.updateCargoExtension(s.raw,s.clusters,1.5+i*.1);}
  CHECK(p.result().work_status.extension_observation=="DIRECT" && std::abs(p.result().work_status.extended_length-held_length)<.01 && p.result().work_status.cargo_track_id==41);

  CranePipelineRegressionAccess::heldExtension(p);CHECK(p.prepareCargoExtension(s.raw,tf2::Transform::getIdentity()).layers.empty());
  w=p.result().work_status;CHECK(w.extension_observation=="HELD" && w.extended_length==held_length && w.extended_raw_indices.empty());

  // Rotation covariance: lateral association and diagnostics use PCA axes, never fixed world Y.
  const double yaw=.7,c=std::cos(yaw),sn=std::sin(yaw);Scene rotated=s;
  for(auto & point:rotated.raw){const double x=point.x,y=point.y;point.x=c*x-sn*y;point.y=sn*x+c*y;}
  CranePipeline q(cfg);CranePipelineRegressionAccess::seedExtension(q,0,0,yaw);CranePipelineRegressionAccess::seedRaw(q,rotated.clusters.front().point_indices);
  for(int i=0;i<4;++i){q.prepareCargoExtension(rotated.raw,tf2::Transform::getIdentity());q.updateCargoExtension(rotated.raw,rotated.clusters,1+i*.1);}
  auto rw=q.result().work_status;CHECK(std::abs(rw.extended_length-held_length)<.01 && std::abs(std::remainder(rw.extended_yaw-yaw,pi))<.01);

  // Gap requires positive observed lower layers AND free rays on both sides and under.
  Scene gap_scene;patch(gap_scene,-.5,.5,-.15,.15,1,1);patch(gap_scene,.8,1.8,-.15,.15,1,2);
  const auto beam_clusters=gap_scene.clusters;patch(gap_scene,-1,2.5,-.6,.6,.4,9);gap_scene.clusters=beam_clusters;
  CranePipeline gap(cfg);CranePipelineRegressionAccess::seedExtension(gap);CranePipelineRegressionAccess::seedRaw(gap,gap_scene.clusters.front().point_indices);
  for(int i=0;i<4;++i){gap.prepareCargoExtension(gap_scene.raw,tf2::Transform::getIdentity());gap.updateCargoExtension(gap_scene.raw,gap_scene.clusters,1+i*.1);}
  auto shape=gap.result().work_status;
  VisibilityConfig vc;vc.azimuth_min_deg=-180;vc.azimuth_max_deg=180;vc.elevation_min_deg=-89;vc.elevation_max_deg=89;vc.range_noise_m=.001;
  vc.azimuth_bin_deg=.5;vc.elevation_bin_deg=.5;VisibilityModel visibility;visibility.configure(vc);tf2::Transform pose=tf2::Transform::getIdentity();pose.setOrigin({0,-5,.65});visibility.setPose(pose);
  const double hu=std::max(cfg.work_status.cargo_min_surface_width_m,shape.extended_length*.25),hv=shape.extended_width*.5;
  const double ec=std::cos(shape.extended_yaw),es=std::sin(shape.extended_yaw);
  for(int side=0;side<3;++side)for(int k=-1;k<=1;++k){
    const double v=side==0?hv+cfg.work_status.grid_resolution_m+cfg.work_status.cargo_min_surface_width_m*.5:
      (side==1?-hv-cfg.work_status.grid_resolution_m-cfg.work_status.cargo_min_surface_width_m*.5:0);
    const double x=shape.extended_x+k*hu*.5*ec-v*es,y=shape.extended_y+k*hu*.5*es+v*ec;
    PointXYZIRT point;point.x=1.5*x;point.y=-5+1.5*(y+5);point.z=.725;gap_scene.raw.push_back(point);
  }
  visibility.buildDepth(gap_scene.raw,{});CranePipelineRegressionAccess::visibility(gap,&visibility);
  // Lazy depth must match eager current-Raw depth, and build at most once
  // even when fragment association and all three Gap ROIs query it.
  CranePipeline lazy_gap=gap;VisibilityModel lazy_visibility;lazy_visibility.configure(vc);lazy_visibility.setPose(pose);
  for(int i=0;i<25;++i){
    gap.prepareCargoExtension(gap_scene.raw,pose,&visibility);gap.updateCargoExtension(gap_scene.raw,gap_scene.clusters,1.4+i*.1);
    int builds=0;lazy_gap.prepareCargoExtension(gap_scene.raw,pose,&lazy_visibility);
    lazy_gap.updateCargoExtension(gap_scene.raw,gap_scene.clusters,1.4+i*.1,[&](){++builds;lazy_visibility.buildDepth(gap_scene.raw,{});});
    CHECK(builds==1);
    const auto & eager=gap.result().work_status;const auto & lazy=lazy_gap.result().work_status;
    CHECK(eager.gap_diagnostic==lazy.gap_diagnostic && eager.gap_sides==lazy.gap_sides);
    CHECK(eager.extended_raw_indices==lazy.extended_raw_indices && eager.extension_cluster_ids==lazy.extension_cluster_ids);
    CHECK(eager.observed_intervals==lazy.observed_intervals && eager.inferred_gaps==lazy.inferred_gaps);
    CHECK(eager.extended_length==lazy.extended_length && eager.gap_confirmed_sec==lazy.gap_confirmed_sec);
  }
  int duplicate_builds=0;lazy_gap.updateCargoExtension(gap_scene.raw,gap_scene.clusters,3.8,[&](){++duplicate_builds;});
  CHECK(duplicate_builds==0);
  auto gw=gap.result().work_status;
  if(gw.gap_diagnostic!="GAP_OBSERVED")std::cerr<<"gap synthetic="<<gw.gap_diagnostic<<" sides="<<gw.gap_sides[0]<<','<<gw.gap_sides[1]<<','<<gw.gap_sides[2]<<" lower="<<gw.lower_layer_z[0]<<','<<gw.lower_layer_z[1]<<','<<gw.lower_layer_z[2]<<'\n';
  CHECK(gw.gap_diagnostic=="GAP_OBSERVED" && gw.work_status && gw.working_status==1);
  CranePipelineRegressionAccess::visibility(gap,nullptr);gap.prepareCargoExtension(gap_scene.raw,pose);gap.updateCargoExtension(gap_scene.raw,gap_scene.clusters,3.9);
  CHECK(gap.result().work_status.gap_diagnostic=="UNKNOWN" && gap.result().work_status.work_status);
  // Near-contact layers within the existing noise model; not a measured CAD underside.
  Scene supported;patch(supported,-.5,.5,-.15,.15,1,1);patch(supported,.8,1.8,-.15,.15,1,2);
  const auto supported_clusters=supported.clusters;patch(supported,-1,2.5,-.6,.6,.9775,9);supported.clusters=supported_clusters;
  CranePipeline contact(cfg);CranePipelineRegressionAccess::seedExtension(contact);CranePipelineRegressionAccess::seedRaw(contact,supported.clusters.front().point_indices);
  for(int i=0;i<26;++i){contact.prepareCargoExtension(supported.raw,pose);contact.updateCargoExtension(supported.raw,supported.clusters,1+i*.1);}
  auto cw=contact.result().work_status;
  if(cw.gap_diagnostic!="SUPPORTED")std::cerr<<"support synthetic="<<cw.gap_diagnostic<<" sides="<<cw.gap_sides[0]<<','<<cw.gap_sides[1]<<','<<cw.gap_sides[2]<<'\n';
  CHECK(cw.gap_diagnostic=="SUPPORTED" && cw.work_status);
  std::cout<<"cargoExtensionAndGapTest passed: seed conditioned full Raw, temporal gap, wrong layer/axis/long gap rejection, changing cluster IDs, held, rotation, free-ray gap vs UNKNOWN, near-support\n";
}


// Final boolean confirmation is independent of patch geometry and Work FSM.
static void learnedHousingReferenceContextTest() {
  auto f=housingFixture();auto cfg=housingConfig();CraneHookChainNode wire;wire.x=-.9;wire.y=.05;wire.z=4.4;WireBundleGeometry bundle;
  const auto raw=CranePipeline::detectSheaveCandidate(wire,f.subs,f.points,bundle,cfg.sheave,cfg.dual_roi_anchor);
  CHECK(raw.status==CraneSheaveDetectionStatus::DIRECT_OBSERVED && raw.wire_observation_mode==2);
  HousingShapeReference r;r.id="learned_test";r.label="VERIFIED";r.manually_checked=true;r.family_id=7;
  r.dimensions={raw.housing_length,raw.housing_width,raw.housing_height};r.spectrum=raw.shape_eigenvalues;
  r.z_distribution=raw.z_distribution;r.horizontal_distribution=raw.horizontal_distribution;
  r.has_local_shape=true;r.local_extent=raw.local_extent;r.local_occupancy=raw.local_occupancy;
  r.sensor_range=1;r.view_direction={0,0,-1};auto sample=raw;sample.sensor_range=1;sample.view_direction={0,0,-1};
  auto model=cfg.sheave;model.reference_matching_enabled=true;model.learned_references={r};
  auto g=sample;CHECK(!CranePipeline::matchHousingReference(g,model)); // no cold seeding
  HousingGeometryMemory memory;memory.initialized=memory.ever_confirmed=true;memory.last_frame_stamp=2;memory.last_direct_stamp=1.9;memory.reference_family_id=7;
  g=sample;CHECK(CranePipeline::matchHousingReference(g,model,&memory));CHECK(g.reference_from_learned && g.reference_id==r.id);
  memory.ever_confirmed=false;g=sample;CHECK(!CranePipeline::matchHousingReference(g,model,&memory));memory.ever_confirmed=true;
  g=sample;g.wire_supported=false;CHECK(!CranePipeline::matchHousingReference(g,model,&memory));
  g=sample;g.wire_observation_mode=1;CHECK(!CranePipeline::matchHousingReference(g,model,&memory));
  g=sample;g.sensor_range=3;CHECK(!CranePipeline::matchHousingReference(g,model,&memory));
  g=sample;g.view_direction={1,0,0};CHECK(!CranePipeline::matchHousingReference(g,model,&memory));
  memory.last_frame_stamp=10;g=sample;CHECK(!CranePipeline::matchHousingReference(g,model,&memory));memory.last_frame_stamp=2;
  memory.reference_family_id=8;g=sample;CHECK(!CranePipeline::matchHousingReference(g,model,&memory));memory.reference_family_id=7;
  auto negative=r;negative.id="hard_negative";negative.label="NEGATIVE";model.references={negative};
  g=sample;CHECK(!CranePipeline::matchHousingReference(g,model,&memory));CHECK(g.negative_reference_rejected && !g.reference_from_learned);
  std::cout<<"learnedHousingReferenceContextTest passed: no cold/old/missing-wire/cross-family use, hard negatives preserved\n";
}

static void boundaryTemporalConfirmationTest() {
  CraneBoundaryMotionConfirmation f;
  auto u=[&](double t,bool support,bool raw){return f.update(t,support,raw,.5,1.5,2.5);};
  CHECK(!u(10,true,true));CHECK(!u(10.4,true,true));CHECK(!u(10.4,true,true));CHECK(u(10.5,true,true));
  CHECK(u(10.6,true,false));CHECK(u(11,true,true)); // short stop
  CHECK(u(11.1,false,false));CHECK(u(11.2,true,true)); // one missing observation
  CHECK(u(11.3,true,false));CHECK(u(12.7,true,false));CHECK(!u(12.8,true,false));
  CHECK(!u(14,true,true));CHECK(u(14.5,true,true));CHECK(!u(17.1,false,false));
  CHECK(!u(18,true,true));CHECK(u(18.5,true,true));CHECK(!u(5,true,true)); // rewind
  CHECK(!u(5,true,true));CHECK(u(5.5,true,true));CHECK(!u(20,true,true)); // input gap
  CHECK(!u(std::numeric_limits<double>::quiet_NaN(),true,true));
  std::cout<<"boundaryTemporalConfirmationTest passed\n";
}

static void acquisitionRegression(){
  auto cfg=housingConfig();cfg.work_status.fsm_enabled=true;cfg.work_status.confirm_m_on=3;
  cfg.roi_filter.beam_roi_x_min=-3;cfg.roi_filter.beam_roi_x_max=3;
  struct Scene {std::vector<PointXYZIRT> points;std::vector<CraneSubcluster> subs;};
  auto scene=[&](unsigned hooks,unsigned cargos,double dx=0,bool partial=false,bool shared=false,bool sling=false,double target_width=-1,double target_height=.02){
    Scene f;
    auto add=[&](int id,double x,double y,double z,double l,double w,double h){
      CraneSubcluster sc;sc.subcluster_id=id;sc.min_x=x-l*.5;sc.max_x=x+l*.5;
      sc.min_y=y-w*.5;sc.max_y=y+w*.5;sc.min_z=z-h*.5;sc.max_z=z+h*.5;
      sc.centroid_x=x;sc.centroid_y=y;sc.centroid_z=z;
      const int nx=std::max(2,int(l/.02)),ny=std::max(2,int(w/.02)),nz=h>.03?4:1;
      for(int a=0;a<=nx;++a)for(int b=0;b<=ny;++b)for(int c=0;c<nz;++c){PointXYZIRT p;
        p.x=sc.min_x+l*a/nx;p.y=sc.min_y+w*b/ny;p.z=nz==1?z:sc.min_z+h*c/(nz-1);
        sc.point_indices.push_back(f.points.size());f.points.push_back(p);}
      f.subs.push_back(sc);
    };
    for(int side=0;side<2;++side)if(hooks&(1u<<side)){
      const double x=side==0?-.4:.4;add(10+side,x,0,4.02,.16,.12,.16);
      for(int i=0;i<10;++i){PointXYZIRT p;p.x=x;p.y=0;p.z=4.11+.018*i;f.points.push_back(p);}
    }
    if(shared && cargos){add(70,dx,0,3.3,1.2,target_width>0?target_width:(partial?.2:.4),target_height);
      for(int side=0;side<2;++side)for(int i=0;i<30;++i){PointXYZIRT p;p.x=side==0?-.4:.4;p.y=0;p.z=3.38+.016*i;f.points.push_back(p);}
    }
    else for(int side=0;side<2;++side)if(cargos&(1u<<side)){
      add(70+side,(side==0?-.4:.4)+dx,0,3.3,partial?.3:.6,target_width>0?target_width:(partial?.2:.4),target_height);
      for(int i=0;i<30;++i){PointXYZIRT p;p.x=side==0?-.4:.4;p.y=0;p.z=3.38+.016*i;f.points.push_back(p);}
    }
    if(sling){CraneSubcluster sc;sc.subcluster_id=90;sc.min_x=-.7;sc.max_x=-.1;sc.min_y=-.3;sc.max_y=.3;sc.min_z=2.9;sc.max_z=3.7;sc.centroid_x=-.4;sc.centroid_z=3.3;
      for(int i=0;i<100;++i)for(int sign:{-1,1}){PointXYZIRT p;const double a=i/99.;p.x=-.4+sign*.3*a;p.y=sign*.3*a;p.z=3.7-.8*a;sc.point_indices.push_back(f.points.size());f.points.push_back(p);}f.subs.push_back(sc);}
    return f;
  };
  auto run=[&](CranePipeline & p,Scene f,double stamp,unsigned housings=3){
    return CranePipelineRegressionAccess::work(p,f.points,f.subs,stamp,housings);
  };
  CranePipeline p(cfg);double t=1;
  for(int i=0;i<30;++i){auto w=run(p,scene(1,0,0,false,false,true),t+=.1);CHECK(!w.work_status);CHECK(!w.geometry_locked);CHECK(w.left_hook.display_state=="WAITING");CHECK(!w.left_hook.work_status);}
  std::vector<std::string> transitions;CraneBeamState last=CraneBeamState::IDLE;
  for(int i=0;i<26;++i){auto w=run(p,scene(1,1,.002*i),t+=.1);
    if(w.fsm_state!=last){transitions.push_back(to_string(w.fsm_state));last=w.fsm_state;}
    if(i<2)CHECK(!w.work_status);if(i==2)CHECK(w.fsm_state==CraneBeamState::READY);
    if(i<20)CHECK(!w.geometry_locked);
  }

 auto w=p.result().work_status;CHECK(w.geometry_locked);CHECK(w.cargo_observation=="DIRECT");
 const auto locked=w;const double before_x=w.cargo_x;
 for(int i=0;i<4;++i){auto v=run(p,scene(1,1,.052+.02*i,true),t+=.1);CHECK(v.geometry_locked);CHECK(v.cargo_track_id==locked.cargo_track_id);CHECK(v.length==locked.length&&v.width==locked.width&&v.height==locked.height&&v.yaw==locked.yaw);}
 CHECK(p.result().work_status.cargo_x>before_x);
 CranePipeline no_hook(cfg),only_hook(cfg),sling_body(cfg);
 for(int i=0;i<25;++i){auto a=run(no_hook,scene(0,1),20+i*.1,0);CHECK(!a.work_status);CHECK(!a.geometry_locked);auto b=run(only_hook,scene(1,0),20+i*.1);CHECK(!b.work_status);CHECK(!b.geometry_locked);auto c=run(sling_body,scene(1,1,0,false,false,true),20+i*.1);if(i==24)CHECK(c.geometry_locked);}

 CranePipeline separate(cfg);for(int i=0;i<25;++i){auto v=run(separate,scene(3,3),30+i*.1);CHECK(v.cargo_side!="BOTH");}
 auto no_link=scene(1,1);for(auto&point:no_link.points)if(point.z>3.36&&point.z<3.90)point.x+=2;
 CranePipeline disconnected(cfg);for(int i=0;i<25;++i){auto v=run(disconnected,no_link,40+i*.1);CHECK(!v.work_status);CHECK(!v.geometry_locked);}
 auto outline=scene(1,1);std::vector<int> remap(outline.points.size(),-1);std::vector<PointXYZIRT> hollow_raw;
 for(size_t i=0;i<outline.points.size();++i){const auto&v=outline.points[i];if(std::abs(v.z-3.3)<.015&&std::abs(v.x+.4)<.28&&std::abs(v.y)<.18)continue;remap[i]=hollow_raw.size();hollow_raw.push_back(v);}
 for(auto&sc:outline.subs){std::vector<int>ids;for(int old:sc.point_indices)if(old>=0&&size_t(old)<remap.size()&&remap[old]>=0)ids.push_back(remap[old]);sc.point_indices=ids;}outline.points=hollow_raw;

 CranePipeline hollow(cfg);for(int i=0;i<25;++i){auto v=run(hollow,outline,50+i*.1);CHECK(!v.work_status);CHECK(!v.geometry_locked);}
 CranePipeline brief(cfg);CraneWorkStatusResult held;
 for(int i=0;i<6;++i)held=run(brief,scene(1,1),60+i*.1);int saved_id=held.cargo_track_id;
 held=run(brief,scene(1,0),60.6);CHECK(held.cargo_observation=="HELD");CHECK(held.cargo_track_id==saved_id);
 held=run(brief,scene(1,1),60.7);CHECK(held.cargo_observation=="DIRECT");CHECK(held.cargo_track_id==saved_id);
 held=run(brief,scene(1,0),60.7+cfg.work_status.cargo_loss_sec+.1);CHECK(!held.work_status);CHECK(held.cargo_observation!="DIRECT");

 // Known identity narrow surface: genuine XY patch with independent vertical columns.
 CranePipeline narrow(cfg);CraneWorkStatusResult nw;
 for(int i=0;i<26;++i)nw=run(narrow,scene(1,1,0,false,false,false,.4,.20),70+i*.1);
 CHECK(nw.geometry_locked);auto ng=nw;const int nid=nw.cargo_track_id;
 auto partial_surface=scene(1,1,0,false,false,false,.12,.20);
 nw=run(narrow,partial_surface,72.6);
 CHECK(nw.cargo_observation=="DIRECT");CHECK(nw.cargo_track_id==nid);
 CHECK(nw.geometry_locked && nw.length==ng.length && nw.width==ng.width && nw.height==ng.height && nw.yaw==ng.yaw);
 // A new ID cannot inherit the narrow fragment exception.
 auto different=partial_surface;for(auto&sc:different.subs)if(sc.subcluster_id==70)sc.subcluster_id=170;
 nw=run(narrow,different,72.7);CHECK(nw.cargo_observation!="DIRECT");
 // A narrow one-plane surface and thin line cannot satisfy vertical-column support.
 nw=run(narrow,scene(1,1,0,false,false,false,.12,.02),72.8);CHECK(nw.cargo_observation!="DIRECT");
 nw=run(narrow,scene(1,1,0,false,false,false,.02,.20),72.9);CHECK(nw.cargo_observation!="DIRECT");
 CranePipeline fresh_narrow(cfg);for(int i=0;i<26;++i){auto q=run(fresh_narrow,partial_surface,80+i*.1);CHECK(!q.geometry_locked);CHECK(q.cargo_observation!="DIRECT");}
 // Actual rotation still uses baseline association and locked geometry; never fabricate new size/yaw.
 auto rotate=scene(1,1);double angle=.10;
 for(auto&sc:rotate.subs)if(sc.subcluster_id==70){
  sc.min_x=sc.min_y=1e9;sc.max_x=sc.max_y=-1e9;
  for(int id:sc.point_indices){auto&v=rotate.points[id];double dx=v.x+.4,dy=v.y;v.x=-.4+dx*std::cos(angle)-dy*std::sin(angle);v.y=dx*std::sin(angle)+dy*std::cos(angle);sc.min_x=std::min(sc.min_x,double(v.x));sc.max_x=std::max(sc.max_x,double(v.x));sc.min_y=std::min(sc.min_y,double(v.y));sc.max_y=std::max(sc.max_y,double(v.y));}
 }
 nw=run(narrow,rotate,73.0);CHECK(nw.cargo_observation=="DIRECT");CHECK(nw.cargo_track_id==nid);CHECK(nw.yaw==ng.yaw);
 // A measured owned partial surface may contain an outlying tail return.
 auto tail=partial_surface;int tail_id=tail.points.size();PointXYZIRT tail_point;
 tail_point.x=-.765;tail_point.y=0;tail_point.z=3.3;tail.points.push_back(tail_point);
 for(auto&sc:tail.subs)if(sc.subcluster_id==70){sc.point_indices.push_back(tail_id);sc.min_x=tail_point.x;}
 nw=run(narrow,tail,73.1);CHECK(nw.cargo_observation=="DIRECT");CHECK(nw.cargo_track_id==nid);
 CHECK(nw.length==ng.length && nw.width==ng.width && nw.height==ng.height && nw.yaw==ng.yaw);
 CHECK(std::find(nw.left_hook.object_point_indices.begin(),nw.left_hook.object_point_indices.end(),tail_id)==nw.left_hook.object_point_indices.end());
 tail.points[tail_id].z=3.19;for(auto&sc:tail.subs)if(sc.subcluster_id==70)sc.min_z=3.19;
 nw=run(narrow,tail,73.2);CHECK(nw.cargo_observation!="DIRECT");
 std::cout<<"PASS: locked same-ID narrow measured surface; original acquisition strict; reject alternate ID/flat narrow patch/thin line; controlled rotation keeps original lock contract\n";
 std::cout<<"PASS: no merging independent Hook objects; disconnected surface and hollow outline rejected; brief Raw dropout HELD then same-ID DIRECT; loss expiry never creates DIRECT\n";
 std::cout<<"PASS: original wide Cargo acquisition, Hook-only and Sling-only rejected, real Cargo with sling retained; no FSM transition policy modified\n";
}

static void groundMixedCargoAcquisitionTest() {
  auto cfg=housingConfig();cfg.work_status.fsm_enabled=true;cfg.work_status.cargo_confirm_sec=2;
  cfg.work_status.virtual_hook_offset_z_m=.9;
  std::vector<PointXYZIRT> raw;CraneSubcluster mixed;mixed.subcluster_id=70;
  auto point=[&](double x,double y,double z){PointXYZIRT p;p.x=x;p.y=y;p.z=z;mixed.point_indices.push_back(raw.size());raw.push_back(p);};
  for(int i=0;i<=60;++i)for(int j=0;j<=40;++j)point(-1.3+i*.03,-.6+j*.03,0);
  for(int i=0;i<=60;++i)for(int j=0;j<=15;++j)for(int k=0;k<4;++k)point(-1+i*.02,-.15+j*.02,.12+k*(.16/3));
  mixed.min_x=-1.3;mixed.max_x=.5;mixed.min_y=-.6;mixed.max_y=.6;mixed.min_z=0;mixed.max_z=.28;
  mixed.centroid_x=-.4;mixed.centroid_y=0;mixed.centroid_z=.1;
  for(int i=0;i<60;++i){PointXYZIRT p;p.x=-.4;p.y=0;p.z=.30+i*.018;raw.push_back(p);}
  CranePipeline pipeline(cfg);
  for(int i=0;i<27;++i)CranePipelineRegressionAccess::work(pipeline,raw,{mixed},100+i*.1,1,.4,-2.2);
  const auto & w=pipeline.result().work_status;
  CHECK(w.geometry_locked && w.cargo_observation=="DIRECT");
  CHECK(w.length>1.1 && w.length<1.3 && w.width<.4);
  for(int id:w.left_hook.object_point_indices)CHECK(raw[id].z>.05);
  std::cout<<"groundMixedCargoAcquisitionTest passed: connected raised beam retains confirmation and excludes broad floor\n";
}

static void hybridReconfirmationTest() {
  // The raised object's denser surface must not replace a lower measured floor.
  {HookZDensity density;std::vector<PointXYZIRT> raw;
   for(int i=0;i<30;++i)for(int j=0;j<20;++j){PointXYZIRT p;p.x=-.5+i*.03;p.y=-.5+j*.04;p.z=(i%3-1)*.003;raw.push_back(p);}
   for(int i=0;i<60;++i)for(int j=0;j<20;++j){PointXYZIRT p;p.x=-.5+i*.015;p.y=-.5+j*.04;p.z=.28+(i%3-1)*.003;raw.push_back(p);}
   const auto observed=density.detect(raw,0,0,1.4,1.2,1.2);
   CHECK(observed["ground_valid"].get<bool>());
   CHECK(std::abs(observed["ground_z"].get<double>())<.02);
   CHECK(observed["valid"].get<bool>() && observed["selected"]["mean_z"].get<double>()>.2);
  }

  CraneWorkStatusConfig c;
  c.cargo_confirm_sec=2;c.cargo_loss_sec=1.3;
  c.cargo_roi_width_x_m=c.cargo_roi_width_y_m=1.2;c.connect_max_gap_z=2.5;
  auto fixture=[](){
    std::pair<std::vector<PointXYZIRT>,CraneSlopeResult> f;
    auto & raw=f.first;auto & r=f.second;auto & w=r.work_status;
    for(int i=0;i<200;++i){PointXYZIRT p;p.x=(i%5)*.04;p.y=(i%7)*.02;p.z=0;r.ground.push_back(raw.size());raw.push_back(p);}
    for(int i=0;i<30;++i){PointXYZIRT p;p.x=.02+.002*(i%3);p.y=.02+.002*(i%4);p.z=1;w.extended_raw_indices.push_back(raw.size());raw.push_back(p);}
    w.cargo_track_id=1;w.cargo_observation="DIRECT";w.cargo_side="RIGHT";
    w.extension_observation="DIRECT";w.work_status=true;w.confirmed_observation_sec=3;
    w.geometry_locked=true;w.length=1.8;w.width=.4;w.height=.3;w.yaw=0;
    w.right_hook.hook_detected=true;w.right_hook.hook_z=1.5;w.right_hook.hook_observation="VIRTUAL_DIRECT";
    r.top_down_hook.right_housing.sheave.status=CraneSheaveDetectionStatus::DIRECT_OBSERVED;
    r.top_down_hook.right_housing.sheave.points_from_current_frame=true;
    r.top_down_hook.right_housing_memory.confirmed=true;
    return f;
  };
  for(int kind=0;kind<10;++kind){
    auto f=fixture();auto & raw=f.first;auto & r=f.second;auto & w=r.work_status;
    LatestSuspendedPolicy p;
    p.process(raw,r,c,10);CHECK(p.state!="SUSPENDED");
    p.process(raw,r,c,10.1);CHECK(p.state!="SUSPENDED");
    p.process(raw,r,c,10.2);CHECK(p.state=="SUSPENDED");
    w.cargo_observation="HELD";p.process(raw,r,c,10.3);
    CHECK(p.state!="SUSPENDED" && p.state!="LIFTING");
    w.cargo_observation="DIRECT";double stamp=10.4;
    if(kind==1)w.cargo_track_id=2;
    if(kind==2)stamp=11.6;
    if(kind==3)w.width+=.01;
    if(kind==4)for(int id:w.extended_raw_indices)raw[id].x=.5;
    if(kind==5)for(int id:w.extended_raw_indices)raw[id].z=.09;
    if(kind==6){r.ground.clear();raw.erase(raw.begin(),raw.begin()+200);for(int & id:w.extended_raw_indices)id-=200;}
    if(kind==7)stamp=10.1;
    if(kind==8){w.cargo_side="LEFT";w.left_hook=w.right_hook;r.top_down_hook.left_housing=r.top_down_hook.right_housing;r.top_down_hook.left_housing_memory=r.top_down_hook.right_housing_memory;}
    if(kind==9)w.geometry_locked=false;
    p.process(raw,r,c,stamp);
    if(kind==0){CHECK(p.state=="SUSPENDED");CHECK(p.evidence["reconfirmation_reused"]==true);CHECK(p.lift_count==0);}
    else CHECK(p.state!="SUSPENDED");
  }
  auto f=fixture();LatestSuspendedPolicy p;
  for(int i=0;i<3;++i)p.process(f.first,f.second,c,20+i*.1);
  for(int id:f.second.work_status.extended_raw_indices)f.first[id].x=.5;
  p.process(f.first,f.second,c,20.3);CHECK(p.state!="SUSPENDED");
  for(int id:f.second.work_status.extended_raw_indices)f.first[id].x=.02;
  p.process(f.first,f.second,c,20.4);CHECK(p.state!="SUSPENDED");
  p.process(f.first,f.second,c,20.5);CHECK(p.state!="SUSPENDED");
  p.process(f.first,f.second,c,20.6);CHECK(p.state=="SUSPENDED");
  LatestSuspendedPolicy reboot;reboot.process(f.first,f.second,c,30);CHECK(reboot.state!="SUSPENDED");
  std::cout<<"hybridReconfirmationTest passed: current DIRECT gates, same locked ID/side/geometry/TTL only, no HELD rise, first/reboot confirmation retained\n";
}

namespace hybrid_lowering_regression {
struct Fixture {
 LatestSuspendedPolicy policy;CraneSlopeResult r;CraneWorkStatusConfig c;CraneBeamConfig bc;
 std::vector<PointXYZIRT>raw;double stamp=10;bool ordered_projection=false;
 Fixture(){c.cargo_loss_sec=1.3;c.cargo_roi_width_x_m=c.cargo_roi_width_y_m=1.2;c.cargo_confirm_sec=2;
  auto&w=r.work_status;w.cargo_track_id=1;w.cargo_cluster_id=70;w.cargo_observation="DIRECT";w.cargo_side="RIGHT";
  w.geometry_locked=w.work_status=true;w.confirmed_observation_sec=3;w.length=.8;w.width=.4;w.height=.2;
  w.extension_observation="DIRECT";w.right_hook.hook_detected=true;w.right_hook.hook_observation="VIRTUAL_DIRECT";
  auto&h=r.top_down_hook.right_housing;h.sheave.status=CraneSheaveDetectionStatus::DIRECT_OBSERVED;h.sheave.points_from_current_frame=true;
  r.top_down_hook.right_housing_memory.confirmed=true;r.top_down_hook.right_housing_memory.track_id=5;
  scene(1,2.1);
 }
 void scene(double bottom,double hook){raw.clear();r.ground.clear();auto&w=r.work_status;w.extended_raw_indices.clear();
  for(int i=0;i<20;++i)for(int j=0;j<20;++j){PointXYZIRT p;p.x=-.5+i*.05;p.y=-.5+j*.05;p.z=0;r.ground.push_back(raw.size());raw.push_back(p);}
  for(int i=0;i<21;++i)for(int j=0;j<11;++j)for(int k=0;k<5;++k){PointXYZIRT p;p.x=-.4+i*.04;p.y=-.2+j*.04;p.z=bottom+k*.05;w.extended_raw_indices.push_back(raw.size());raw.push_back(p);}
  w.cargo_z=bottom+.1;w.right_hook.hook_z=hook;
 }
 std::string step(double dt=.1){stamp+=dt;if(ordered_projection)r.work_status.work_status=true;policy.process(raw,r,c,stamp);if(ordered_projection)r.work_status.work_status=false;policy.processMotionAndCompletion(raw,r,c,bc,stamp);return policy.state;}
 void warm(){for(int i=0;i<5;++i)step();CHECK(policy.state=="SUSPENDED");}
};
static void run(){
 {Fixture f;f.scene(.05,1.15);for(int i=0;i<30;++i){CHECK(f.step()!="SUSPENDED");CHECK(f.policy.work_cargo<0);}
  f.r.work_status.cargo_observation="HELD";for(int i=0;i<20;++i)CHECK(f.step()!="LOWERING_CONFIRMATION");}
 for(double gap:{0.,.09,.099,.10,.101,.30,.40}){Fixture f;f.scene(gap,gap+1.1);for(int i=0;i<5;++i)f.step();CHECK((f.policy.state=="SUSPENDED")== (gap>=.10));}
 {Fixture f;f.warm();bool lower=false;for(int i=1;i<=12;++i){f.scene(1-i*.025,2.1-i*.025);lower|=f.step()=="LOWERING";}CHECK(lower);f.scene(.7,1.8);for(int i=0;i<4;++i)f.step();CHECK(f.policy.state=="SUSPENDED");CHECK(f.policy.event_started<0);}
 {Fixture f;f.warm();bool lower=false;for(int i=1;i<=12;++i){f.scene(1-i*.025,2.1);f.r.work_status.right_hook.hook_detected=false;f.r.top_down_hook.right_housing.sheave.points_from_current_frame=false;lower|=f.step()=="LOWERING";}CHECK(lower);}
 {Fixture f;f.warm();for(int i=1;i<12;++i){f.scene(1,2.1-i*.025);CHECK(f.step()!="LOWERING");}}
 {Fixture f;f.warm();for(int i=1;i<12;++i){f.scene(1-i*.02,2.1-i*.02);f.r.work_status.cargo_observation="HELD";CHECK(f.step()!="LOWERING");}}
 {Fixture f;f.warm();f.r.work_status.cargo_observation="HELD";for(int i=0;i<12;++i){f.step();CHECK(f.policy.state!="LOWERING_CONFIRMATION");}f.step();CHECK(f.policy.state=="LOWERING_CONFIRMATION");CHECK(f.policy.completion_reason=="LOSS_TIMEOUT");for(int i=0;i<29;++i)CHECK(f.step()=="LOWERING_CONFIRMATION");for(int i=0;i<4;++i)f.step();CHECK(f.policy.state=="IDLE");}
 {Fixture f;f.warm();f.r.work_status.cargo_observation="NONE";f.r.work_status.work_status=false;f.r.work_status.geometry_locked=false;f.r.work_status.cargo_track_id=-1;f.step(4);CHECK(f.policy.state!="LOWERING_CONFIRMATION");for(int i=0;i<12;++i){f.step();CHECK(f.policy.state!="LOWERING_CONFIRMATION");}f.step();CHECK(f.policy.completion_reason=="LOSS_TIMEOUT");}
 {Fixture f;f.r.work_status.cargo_observation="NONE";f.r.work_status.geometry_locked=false;for(int i=0;i<40;++i)CHECK(f.step()!="LOWERING_CONFIRMATION");}
 {Fixture f;f.warm();f.r.work_status.beam_ground_clearance_valid=true;f.r.work_status.beam_ground_clearance_m=.38;for(int i=0;i<2;++i)CHECK(f.step()!="LOWERING_CONFIRMATION");for(int i=0;i<3;++i)f.step();CHECK(f.policy.state=="LOWERING_CONFIRMATION");CHECK(f.policy.completion_reason=="GROUND_CONTACT");for(int i=0;i<40;++i)f.step();CHECK(f.policy.state=="IDLE");}
 {Fixture f;f.warm();f.r.work_status.beam_ground_clearance_valid=false;f.r.work_status.beam_ground_clearance_m=0;for(int i=0;i<10;++i)CHECK(f.step()!="LOWERING_CONFIRMATION");}
 {Fixture f;f.warm();f.r.work_status.right_hook.hook_x=3;for(int i=0;i<5;++i)f.step();CHECK(f.policy.state=="LOWERING_CONFIRMATION");CHECK(f.policy.completion_reason=="HOOK_DETACHED");}
 {Fixture f;f.warm();f.r.work_status.right_hook.hook_detected=false;for(int i=0;i<10;++i)CHECK(f.step()!="LOWERING_CONFIRMATION");}
 {Fixture f;f.warm();for(int i=1;i<10;++i){f.scene(1,2.1);f.r.work_status.cargo_track_id=i+1;CHECK(f.step()!="LOWERING");}}
 {Fixture f;f.warm();for(int i=1;i<12;++i){f.scene(1+(i%2)*.001,2.1+(i%2)*.001);CHECK(f.step()!="LOWERING");}}
 {Fixture f;f.warm();for(int i=0;i<10;++i){f.scene(1,2.1);auto&ids=f.r.work_status.extended_raw_indices;ids.erase(std::remove_if(ids.begin(),ids.end(),[&](int k){return f.raw[k].z>1.12;}),ids.end());CHECK(f.step()!="LOWERING");}}
 {Fixture f;f.warm();f.step(-5);CHECK(f.policy.state!="LOWERING"&&f.policy.state!="LOWERING_CONFIRMATION");}
 {Fixture f;f.warm();bool lower=false;for(int i=1;i<=12;++i){f.scene(1-i*.025,2.1-i*.025);lower|=f.step()=="LOWERING";}CHECK(lower);f.r.work_status.beam_ground_clearance_valid=true;f.r.work_status.beam_ground_clearance_m=.38;for(int i=0;i<5;++i)f.step();CHECK(f.policy.state=="LOWERING_CONFIRMATION" && f.policy.completion_reason=="GROUND_CONTACT");}
 {Fixture f;f.warm();f.step(-5);for(int i=0;i<5;++i)f.step();CHECK(f.policy.state=="SUSPENDED");CHECK(f.policy.completion_reason.empty());}
 {Fixture f;f.ordered_projection=true;f.warm();bool down=false;for(int i=1;i<=12;++i){f.scene(1-i*.025,2.1-i*.025);down|=f.step()=="LOWERING";}CHECK(down);f.r.work_status.cargo_observation="HELD";for(int i=0;i<14;++i)f.step();CHECK(f.policy.state=="LOWERING_CONFIRMATION");}
 {Fixture f;f.warm();bool lower=false;for(int i=1;i<=12;++i){f.scene(1-i*.02,2.1-i*.02);f.r.ground.clear();f.raw.erase(f.raw.begin(),f.raw.begin()+400);for(int&k:f.r.work_status.extended_raw_indices)k-=400;lower|=f.step()=="LOWERING";}CHECK(lower);CHECK(f.policy.completion_reason.empty());}
 {Fixture f;f.warm();bool down=false;for(int i=1;i<=12;++i){f.scene(1-i*.025,2.1);down|=f.step()=="LOWERING";}CHECK(down);}
 std::cout<<"PASS: 10cm boundaries, direct descent with/without sheave, hook-only/noise/HELD/ID/visibility rejection; stop->SUSPENDED; independent LOSS/GROUND/DETACH; input outage; 3s hold/duplicate suppression; restart/rewind\n";
}

}
static void broadGroundAndContactObservationTest() {
  // A broad raised body hides Ground within Hook ROI. Observe the surrounding
  // configured Crane ground area without changing the candidate Hook ROI.
  {HookZDensity density;std::vector<PointXYZIRT> raw;
   for(int i=0;i<=80;++i)for(int j=0;j<=80;++j){PointXYZIRT p;p.x=-2+i*.05;p.y=-2+j*.05;p.z=0;
     if(std::abs(p.x)>.8 || std::abs(p.y)>.8)raw.push_back(p);}
   for(int i=0;i<=50;++i)for(int j=0;j<=50;++j){PointXYZIRT p;p.x=-.75+i*.03;p.y=-.75+j*.03;p.z=.7;raw.push_back(p);}
   auto old=density.detect(raw,0,0,1.2,1.2,1.2);
   auto widened=density.detect(raw,0,0,1.2,1.2,1.2,{},4,4);
   CHECK(old["ground_z"].get<double>()>.6);
   CHECK(widened["ground_valid"].get<bool>() && std::abs(widened["ground_z"].get<double>())<.02);
   CHECK(widened["valid"].get<bool>() && widened["selected"]["mean_z"].get<double>()>.6);
  }
  for(int kind=0;kind<6;++kind){
    auto cfg=housingConfig();cfg.work_status.fsm_enabled=true;cfg.work_status.virtual_hook_offset_z_m=.9;
    cfg.roi_filter.beam_roi_x_min=cfg.roi_filter.beam_roi_y_min=-2;
    cfg.roi_filter.beam_roi_x_max=cfg.roi_filter.beam_roi_y_max=2;
    std::vector<PointXYZIRT> raw;CraneSubcluster sc;sc.subcluster_id=70;
    const double center_x=-.4,center_y=kind==2?.35:0;
    const double length=kind==1?.1:1.0,width=kind==1?.1:(kind==4?1.0:.4);
    const double top=kind==3?0:.7,bottom=kind==3?0:.5;
    sc.min_x=center_x-length*.5;sc.max_x=center_x+length*.5;sc.min_y=center_y-width*.5;sc.max_y=center_y+width*.5;
    sc.min_z=bottom;sc.max_z=top;sc.centroid_x=center_x;sc.centroid_y=center_y;sc.centroid_z=(bottom+top)*.5;
    for(int i=0;i<=50;++i)for(int j=0;j<=20;++j)for(int k=0;k<4;++k){PointXYZIRT p;
      p.x=sc.min_x+length*i/50;p.y=sc.min_y+width*j/20;p.z=bottom+(top-bottom)*k/3;
      sc.point_indices.push_back(raw.size());raw.push_back(p);}
    if(kind!=5)for(int i=0;i<=80;++i)for(int j=0;j<=80;++j){PointXYZIRT p;p.x=-2+i*.05;p.y=-2+j*.05;p.z=0;raw.push_back(p);}
    CranePipeline pipeline(cfg);const double hook=top+.02;
    for(int i=0;i<27;++i)CranePipelineRegressionAccess::work(pipeline,raw,{sc},200+i*.1,1,.4,hook+.9-4.5);
    if(kind==0 || kind==4){CHECK(pipeline.result().work_status.geometry_locked);CHECK(pipeline.result().work_status.cargo_observation=="DIRECT");}
    else CHECK(!pipeline.result().work_status.geometry_locked);
  }
  std::cout<<"broadGroundAndContactObservationTest passed: surrounding observed Ground, real short contact; reject Hook-only, no near-Hook surface and floor-only\n";
}


int main() { CranePipelineRegressionAccess::sparseBottomTest(); CranePipelineRegressionAccess::independentGroundTest(); CranePipelineRegressionAccess::rawOwnershipTest();broadGroundAndContactObservationTest(); hybrid_lowering_regression::run(); acquisitionRegression();groundMixedCargoAcquisitionTest();hybridReconfirmationTest(); CranePipelineRegressionAccess::orderedWorkFsmTest();CranePipelineRegressionAccess::sheaveZMotionTest();learnedHousingReferenceContextTest();boundaryTemporalConfirmationTest();suppliedParity();cxGeometry();degeneracies();finalGroundOverride();twoDirections();boundaryClusteringDetection();zGaussianLayerGrouping();closedLoopObjectGrouping();craneRoiFsmIntegration();hookAndBeamDetection();hookNearestClusterBelowDetection();temporalStabilizationDetection();topDownHookLargestConnectedClusterTest();topDownTemporalStabilizationTest();topDownSlopeAndTurnAngleLimitTest();virtualRopeWireBundleCenterlineTest();sheaveCandidateDetectionTest();fixedRootSheaveTerminationTest();sheaveHousingGeometryMemoryAndInvarianceTest();housingReferenceLibraryTest();craneBoundaryMotionTest();cargoExtensionAndGapTest();heightDetection(); }
