#define main crane_bev_node_fixture_unused_main
#include "../src/cx128s2_bev_perception_node.cpp"
#undef main
#include <memory>
#include <cassert>
#include <iostream>
#include <random>

#define CARGO_CHECK(x)do{if(!(x)){std::cerr<<"failed "<<__LINE__<<": "#x<<"\n";std::abort();}}while(0)
static BevConnectedComponentClusterer cargoFixtureConfig(int connectivity=8,bool layers=true,bool weak=true,bool density=true,bool bridge=true){
 return BevConnectedComponentClusterer(.1,.1,connectivity,2,1,1,1,1,1,1,5,15,true,bridge,.2,.3,1,1,1,layers,.1,weak,density,1,3,false,.1,-.5);
}
static void cargoFixtureEqual(const std::vector<ClusterResult>&a,const std::vector<ClusterResult>&b){
 CARGO_CHECK(a.size()==b.size());for(size_t i=0;i<a.size();++i){auto&x=a[i];auto&y=b[i];
 CARGO_CHECK(x.id==y.id);CARGO_CHECK(x.indices==y.indices);CARGO_CHECK(x.cell_keys==y.cell_keys);CARGO_CHECK(x.centroid_x==y.centroid_x);CARGO_CHECK(x.centroid_y==y.centroid_y);CARGO_CHECK(x.min_x==y.min_x);CARGO_CHECK(x.max_x==y.max_x);CARGO_CHECK(x.min_y==y.min_y);CARGO_CHECK(x.max_y==y.max_y);CARGO_CHECK(x.length==y.length);CARGO_CHECK(x.width==y.width);CARGO_CHECK(x.point_count==y.point_count);CARGO_CHECK(x.cell_count==y.cell_count);CARGO_CHECK(x.method==y.method);CARGO_CHECK(x.roi_name==y.roi_name);CARGO_CHECK(x.roi_type==y.roi_type);CARGO_CHECK(x.shape_nodes.empty()&&y.shape_nodes.empty());CARGO_CHECK(x.shape_edges.empty()&&y.shape_edges.empty());
 }}
static void cargoFixtureCompare(CargoExtensionBevScratch&scratch,BevConnectedComponentClusterer&base,const std::vector<PointXYZIRT>&p,std::vector<int>ids,uint64_t frame,double stamp){
 auto ref=base;ref.reserveLayerPoints(4);auto cfg=base;cfg.reserveLayerPoints(4);
 auto a=ref.cluster(p,ids,frame,stamp);auto b=scratch.cluster(cfg,p,ids,frame,stamp);cargoFixtureEqual(a,b);
 auto da=ref.diagnostics(),db=cfg.diagnostics();
 CARGO_CHECK(da.occupied_cell_count==db.occupied_cell_count);CARGO_CHECK(da.layer_cell_count==db.layer_cell_count);CARGO_CHECK(da.connected_component_count==db.connected_component_count);CARGO_CHECK(da.valid_cluster_count==db.valid_cluster_count);CARGO_CHECK(da.max_layers_per_xy_cell==db.max_layers_per_xy_cell);CARGO_CHECK(da.average_layers_per_xy_cell==db.average_layers_per_xy_cell);CARGO_CHECK(da.component_cell_histogram==db.component_cell_histogram);CARGO_CHECK(da.component_point_histogram==db.component_point_histogram);CARGO_CHECK(da.strong_layer_cell_count==db.strong_layer_cell_count);CARGO_CHECK(da.weak_layer_cell_count==db.weak_layer_cell_count);CARGO_CHECK(da.weak_support_candidates==db.weak_support_candidates);CARGO_CHECK(da.weak_support_accepted==db.weak_support_accepted);CARGO_CHECK(da.bridge_candidates==db.bridge_candidates);CARGO_CHECK(da.bridge_accepted==db.bridge_accepted);
 // Output is independent of the next call and of released pool memory.
 auto saved=b;scratch.cluster(cfg,{}, {},frame,stamp);cargoFixtureEqual(b,saved);
}
int main(){
 std::mt19937 rng(71031);size_t cases=0;
 for(int mode=0;mode<3;++mode){CargoExtensionBevScratch scratch(mode!=1,mode!=0,mode!=1);CargoExtensionBevScratch independent(mode!=1,mode!=0,mode!=1);
  for(int n=0;n<160;++n){auto cfg=cargoFixtureConfig(n%2?4:8,n%13!=0,n%3!=0,n%4!=0,n%5!=0);cfg.setSensorOrigin(n%2?.5:0,n%3?.2:0);
   std::vector<PointXYZIRT>p;int count=n%7?30+rng()%3000:n%3;
   for(int i=0;i<count;++i){PointXYZIRT q;q.x=(int(rng()%50)-25)*.05f;q.y=(int(rng()%50)-25)*.05f;q.z=(int(rng()%25)-5)*.05f;p.push_back(q);}
   std::vector<int>ids;for(size_t i=0;i<p.size();++i)if(rng()%7)ids.push_back(i);if(n%2)std::shuffle(ids.begin(),ids.end(),rng);
   if(n%9==0&&!ids.empty()){std::vector<int>extra(ids.begin(),ids.begin()+std::min(size_t(5),ids.size()));ids.insert(ids.end(),extra.begin(),extra.end());}
   cargoFixtureCompare(scratch,cfg,p,ids,n%11?uint64_t(n):0,n%6?100+n*.1:50-n*.1);cargoFixtureCompare(independent,cfg,p,ids,n,10+n*.1);++cases;
   if(n%17==0)scratch.reset();
  }
  for(bool one:{false,true}){auto cfg=cargoFixtureConfig(8,true,false,false,true);std::vector<PointXYZIRT>p;std::vector<int>ids;
   for(int i=0;i<200000;++i){PointXYZIRT q;q.x=one?(i%300)*.05f:(i%300)*.3f;q.y=one?(i/300)*.05f:(i/300)*.3f;q.z=(i%3)*.01f;ids.push_back(p.size());p.push_back(q);ids.push_back(p.size());p.push_back(q);}
   cargoFixtureCompare(scratch,cfg,p,ids,9,100.);cargoFixtureCompare(scratch,cfg,{PointXYZIRT{}},{0},9,100.);cargoFixtureCompare(scratch,cfg,{}, {},8,99.);++cases;
  }
  auto memory=scratch.memoryStats();std::cout<<"mode "<<mode<<" retained="<<memory[0]<<" peak="<<memory[1]<<" heap_calls="<<memory[2]<<" queue_cap="<<memory[3]<<" component_cap="<<memory[4]<<" max_input="<<memory[5]<<" max_cells="<<memory[6]<<"\n";auto cfg=cargoFixtureConfig();cfg.reserveLayerPoints(4);auto alive=scratch.cluster(cfg,{PointXYZIRT{},PointXYZIRT{}},{0,1},5,100.);auto copied=alive;scratch.reset();cargoFixtureEqual(alive,copied);CARGO_CHECK(scratch.memoryStats()[3]==0);CARGO_CHECK(scratch.memoryStats()[4]==0);
 }
 std::cout<<"PASS "<<cases<<" cases: exact ordered indices/cells/geometry/diagnostics, empty/small/large/split/single/duplicates/rewind/reset/independent instances/large->small and output lifetime\n";
}
