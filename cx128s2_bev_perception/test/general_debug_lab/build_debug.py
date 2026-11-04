#!/usr/bin/env python3
"""Generate diagnostic annotations from current source, never edit production or shared harness."""
from pathlib import Path
import argparse,os,sys,json,hashlib,fcntl
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
here=Path(__file__).resolve().parent;test=here.parent;pkg=test.parent;out=a.output.resolve();out.mkdir(parents=True,exist_ok=True)
sys.path.insert(0,str(test))
lock=(out/'debug_generator.lock').open('w');fcntl.flock(lock,fcntl.LOCK_EX)
def write(path,s):
 if not path.exists() or path.read_text()!=s:path.write_text(s)
def replace_once(s,old,new):
 if s.count(old)!=1:raise RuntimeError('Diagnostic source anchor changed; refusing stale instrumentation: '+old[:90])
 return s.replace(old,new,1)
node=(pkg/'src/cx128s2_bev_perception_node.cpp').read_text();original=node
node='#include "'+str(here/'lab_trace.hpp')+'"\n'+node
node=replace_once(node,'  friend struct TrackingRegressionAccess;','  friend struct TrackingRegressionAccess;\n  mutable std::map<int, Track> lab_detection_snapshots_;')
node=replace_once(node,'    DetectionResult best;\n    const auto & f = tr.feature;', '    if(general_lab::active) lab_detection_snapshots_[tr.id]=tr;\n    DetectionResult best;\n    const auto & f = tr.feature;')
start=node.index('    if (vehicle_detector_enabled_ &&');end=node.index('      best.class_name = "VEHICLE";',start)
condition=node[start:end]
clauses=[('enabled','vehicle_detector_enabled_','vehicle_detector_enabled_','1'),('height_support','(f.vehicle_height_supported || tr.front_class_confirmed)','(f.vehicle_height_supported || tr.front_class_confirmed)','1'),('height_min','vehicle_height >= vehicle_min_height_','vehicle_height','vehicle_min_height_'),('height_max','vehicle_height <= vehicle_max_height_','vehicle_height','vehicle_max_height_'),('width_min','vehicle_width >= vehicle_min_width_','vehicle_width','vehicle_min_width_'),('width_max','vehicle_width <= vehicle_max_width_','vehicle_width','vehicle_max_width_'),('length_min','(vehicle_length >= 2.0 || front)','vehicle_length','2.0'),('length_max','vehicle_length <= vehicle_max_length_','vehicle_length','vehicle_max_length_'),('vehicle_score','size_score >= vehicle_min_score_','size_score','vehicle_min_score_')]
catalog=[]
for name,expr,val,threshold in clauses:
 line=original[:original.index(expr,original.index('  DetectionResult evaluateDetection('))].count('\n')+1
 # Log only the actual final conjunction; preserve C++ short-circuit order.
 condition=replace_once(condition,expr,f'general_lab::gate("evaluateDetection",tr.id,f.id,"{name}",({expr}),{val},{threshold},{line})')
 catalog.append({'function':'evaluateDetection','gate':name,'line':line,'source':str(pkg/'src/cx128s2_bev_perception_node.cpp')})
node=node[:start]+condition+node[end:]
start=node.index('  const char * vehicleObservationFailure(');end=node.index('  bool kalmanEnabled()',start);body=node[start:end]
for name,expr,val,threshold in [('point_count','f.point_count < min_points','f.point_count','min_points'),('cell_count','f.cell_count < min_cells','f.cell_count','min_cells')]:
 line=original[:original.index(expr,original.index('  const char * vehicleObservationFailure('))].count('\n')+1
 body=replace_once(body,expr,f'!general_lab::gate("vehicleObservationFailure",tr.id,f.id,"{name}",!({expr}),{val},{threshold},{line})')
 catalog.append({'function':'vehicleObservationFailure','gate':name,'line':line,'source':str(pkg/'src/cx128s2_bev_perception_node.cpp')})
import re
body=re.sub(r'return "([A-Z_]+)";',lambda m:'return general_lab::reject(tr.id,f.id,"'+m[1]+'",'+str(original[:original.index('return "'+m[1]+'";',original.index('  const char * vehicleObservationFailure('))].count('\n')+1)+');',body)
node=node[:start]+body+node[end:]
expr='std::abs(dx) <= gate && std::abs(dy) <= gate && d2 < gate * gate'
line=original[:original.index(expr)].count('\n')+1
node=replace_once(node,'const bool inside_gate = '+expr+';',f'const bool inside_gate = general_lab::gate("updateTracks",tracks_[ti].id,f.id,"generic_spatial_gate",({expr}),std::sqrt(d2),gate,{line});')
catalog.append({'function':'updateTracks','gate':'generic_spatial_gate','line':line,'source':str(pkg/'src/cx128s2_bev_perception_node.cpp')})
# Record the already-computed score even if an earlier classification gate fails.
score_index=node.index('    if (',node.index('    const double size_score =',node.index('  DetectionResult evaluateDetection(')))
node=node[:score_index]+'''    if(general_lab::active) general_lab::gates.push_back({
        {"function","evaluateDetectionMetrics"},{"track",tr.id},{"cluster",f.id},
        {"vehicle_size_score",size_score},{"vehicle_min_score",vehicle_min_score_}});
'''+node[score_index:]

# Front diagnostics preserve short-circuit evaluation; no classification is re-run.
start=node.index('  bool vehicleFrontSupported(');end=node.index('  DetectionResult evaluateDetection(',start)
body=node[start:end]
front_clauses=[('visibility_valid','!visibility_frame_valid_','visibility_frame_valid_','1'),
 ('component_type','f.component_type != "vehicle_like"','0','0'),
 ('width_min','f.width < vehicle_min_width_','f.width','vehicle_min_width_'),
 ('height_min','f.height < vehicle_min_height_','f.height','vehicle_min_height_'),
 ('expected_rays','v.adaptive_expected_rays <= 0','v.adaptive_expected_rays','1')]
for name,expr,val,threshold in front_clauses:
 body=replace_once(body,expr,f'!general_lab::gate("vehicleFrontSupported",tr.id,f.id,"{name}",!({expr}),{val},{threshold},0)')
old='''    return cx128s2_bev_perception::vehicle_surface::enoughSparseSupport(
        static_cast<int>(bins.size()), 2, v.adaptive_expected_rays,
        v.occluded_fraction, v.has_occluder);'''
new='''    double effective_expected=0.0;
    const bool pass=cx128s2_bev_perception::vehicle_surface::enoughSparseSupport(
        static_cast<int>(bins.size()),2,v.adaptive_expected_rays,v.occluded_fraction,v.has_occluder,
        &effective_expected);
    if(general_lab::active) general_lab::gates.push_back({
        {"function","vehicleFrontSupported"},{"track",tr.id},{"cluster",f.id},
        {"gate","current_angular_support"},{"evaluated",true},{"pass",pass},
        {"value",bins.size()},{"threshold",std::max(3.0,std::ceil(effective_expected/10.0))},
        {"actual_bins",bins.size()},{"expected_rays",v.adaptive_expected_rays},
        {"occluded_fraction",v.occluded_fraction},{"has_occluder",v.has_occluder},
        {"point_count",f.point_count},{"width",f.width},{"height",f.height},{"length",f.length},
        {"box",{f.min_x,f.max_x,f.min_y,f.max_y,f.min_z,f.max_z}}});
    return pass;'''
body=replace_once(body,old,new);node=node[:start]+body+node[end:]
start=node.index('std::vector<ObjectFeature> front_observations;');end=node.index('// Confirmed vehicles claim',start);body=node[start:end]
body=body.replace('Track probe;','Track probe; probe.id=tr.id;')
left=body.index('        for (const auto * f : candidates) {');right=body.index('          Track probe;',left)
section=body[left:right];i=section.index('if (');j=section.rindex(') continue;');condition=section[i+4:j]
section=section[:i]+'''if ([&](){const bool rejected=('''+condition+''');
            if(general_lab::active) general_lab::gates.push_back({
              {"function","front_component_bounds"},{"track",tr.id},{"cluster",f->id},
              {"pass",!rejected},{"box",{f->min_x,f->max_x,f->min_y,f->max_y,f->min_z,f->max_z}},
              {"predicted_previous_box",{tr.feature.min_x+sx-margin,tr.feature.max_x+sx+margin,
                tr.feature.min_y+sy-margin,tr.feature.max_y+sy+margin,tr.feature.min_z,tr.feature.max_z}},
              {"points",f->point_count},{"height",f->height},{"centroid",{f->centroid_x,f->centroid_y}}});
            return rejected;}()) continue;
'''+section[j+len(') continue;'):]
body=body[:left]+section+body[right:]
anchor=re.search(r' *pieces\.push_back\(f\);\n *\}',body).group(0)
body=replace_once(body,anchor,anchor+'''
        if(general_lab::active) general_lab::gates.push_back({
          {"function","front_aggregate_eligibility"},{"track",tr.id},
          {"pieces",pieces.size()},{"ordinary_front",ordinary_front},{"contested",contested},
          {"pass",!(ordinary_front||contested||pieces.size()<2)}});''')
anchor='        auto & f = built.front();'
body=replace_once(body,anchor,anchor+'''
        if(general_lab::active) general_lab::gates.push_back({
          {"function","front_aggregate_current_geometry"},{"track",tr.id},{"cluster",f.id},
          {"points",f.point_count},{"width",f.width},{"height",f.height},{"length",f.length},
          {"component_type",f.component_type},{"component_reason",f.component_type_reason},
          {"box",{f.min_x,f.max_x,f.min_y,f.max_y,f.min_z,f.max_z}}});''')
node=node[:start]+body+node[end:]
anchor='''      if (best >= 0) {
        assigned_track[fi] = best;'''
node=replace_once(node,anchor,'''      if (best >= 0) {
        if(general_lab::active) general_lab::gates.push_back({
          {"function","generic_selection"},{"track",tracks_[best].id},{"cluster",f.id},
          {"distance",std::sqrt(best_d2)},{"squared_distance",best_d2},
          {"shape_score",best_shape_score},{"final_score",best_score}});
        assigned_track[fi] = best;''')

write(out/'instrumented_node.cpp',node)
harness=(test/'general_replay.cpp').read_text();harness=replace_once(harness,'#include "../src/cx128s2_bev_perception_node.cpp"','#include "'+str(out/'instrumented_node.cpp')+'"');harness=harness.replace('#include "general_replay_cache.hpp"','#include "'+str(test/'general_replay_cache.hpp')+'"')

# Copy current observation indices inside the production frame callback.
# Reading recording_track_features_ after processCloudFrame would dereference
# frame-local features; the copied maps below are valid for the later trace.
harness=replace_once(harness,' n.offline_after_frame_=[&](){if(fast||build)return;', ''' std::map<int,std::vector<int>> lab_current_indices;
 std::map<int,Json> lab_current_components;
 n.offline_after_frame_=[&](){
 if(general_lab::active){
   std::map<int,int> source_counts;
   for(const auto & feature:n.last_candidates_)source_counts[feature.id]=feature.point_count;
   for(const auto & kv:n.recording_track_features_)if(kv.second){
     lab_current_indices[kv.first]=kv.second->indices;
     std::map<int,int> counts;
     for(int index:kv.second->indices)if(index>=0&&size_t(index)<n.frame_cluster_owner_.size()){
       const int owner=n.frame_cluster_owner_[index];if(owner>=0)++counts[owner];
     }
     Json memberships=Json::array();
     for(const auto & pair:counts)memberships.push_back({{"cluster",pair.first},
       {"associated_points",pair.second},{"cluster_points",source_counts[pair.first]},
       {"complete",pair.second==source_counts[pair.first]&&pair.second>0}});
     lab_current_components[kv.first]=std::move(memberships);
   }
 }
 if(fast||build)return;''')
# Inspection only decodes stored records; it never runs the changed prefix/suffix.
# Keep Replay strict, and require the exact serializer, ABI sizes and format.
harness=replace_once(harness,'if(manifest.at("contract")!=expected){','''if(manifest.at("contract")!=expected && !(inspect_frame && dataset &&
 manifest.at("contract").at("schema")==expected.at("schema") &&
 manifest.at("contract").at("dataset_version")==expected.at("dataset_version") &&
 manifest.at("contract").at("compression")==expected.at("compression") &&
 manifest.at("contract").at("depth_cache")==expected.at("depth_cache") &&
 manifest.at("contract").at("sizes")==expected.at("sizes") &&
 manifest.at("contract").at("prefix").at("codec")==expected.at("prefix").at("codec"))) {''')
harness=replace_once(harness,'   auto t=Clock::now();n.processCloudFrame', '''   lab_current_indices.clear();lab_current_components.clear();
   general_lab::active = count>=size_t(general_lab::number("GENERAL_LAB_DETAIL_START",0)) && count<=size_t(general_lab::number("GENERAL_LAB_DETAIL_END",0));
   general_lab::gates=Json::array();n.lab_detection_snapshots_.clear();
   auto t=Clock::now();n.processCloudFrame''')
harness=replace_once(harness,"   trace<<event.dump()<<'\\n';}", (here/'trace_event.inc').read_text()+"\n   trace<<event.dump()<<'\\n';}")
harness=replace_once(harness,'static long rss(){', (here/'whatif.inc').read_text()+'\nstatic long rss(){')
harness=replace_once(harness,' TrackingRegressionAccess::run(*n,mode,bag,cache,out,', ' if(mode=="what-if"){TrackingRegressionAccess::whatIf(*n,bag,out);n.reset();rclcpp::shutdown();return 0;}\n TrackingRegressionAccess::run(*n,mode,bag,cache,out,')
write(out/'general_debug_replay.cpp',harness)
write(out/'gate_catalog.json',json.dumps(catalog,indent=2))
# Reuse compiler/link/dependency checks without modifying shared build_replay.py.
script=(test/'build_replay.py').read_text().replace("test/'general_replay.cpp'",repr(str(out/'general_debug_replay.cpp')))
# Original expressions use Path APIs in some contexts: use Path object literal.
script=script.replace(repr(str(out/'general_debug_replay.cpp')), 'Path('+repr(str(out/'general_debug_replay.cpp'))+')')
sys.argv=[str(test/'build_replay.py'),'--output',str(out)]
exec(compile(script,str(test/'build_replay.py'),'exec'),{'__name__':'__main__','__file__':str(test/'build_replay.py')})
