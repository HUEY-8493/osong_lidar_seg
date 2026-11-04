#!/usr/bin/env python3
"""Sequential, isolated General replay; deletes only its own temporary bag copy."""
import argparse, csv, hashlib, html, json, os, shutil, signal, subprocess, sys, tempfile, time
from pathlib import Path
import yaml

PKG = Path(__file__).resolve().parents[1]

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--engine', choices=['v2','legacy'], default='legacy', help='Original General by default; V2 is experimental')
    p.add_argument('--source', type=Path, default=Path('/mnt/z/osong/oosong_record/enterance'))
    p.add_argument('--results', type=Path, required=True)
    p.add_argument('--scratch', type=Path, default=Path.home() / '.cache/general_bag_review')
    p.add_argument('--limit', type=int, default=2, help='Maximum number of bags; default: latest two compatible bags')
    p.add_argument('--bag', action='append', help='Exact directory name; repeatable')
    p.add_argument('--list-only', action='store_true')
    p.add_argument('--params', type=Path, default=PKG/'config/cx128s2_bev_perception.yaml')
    p.add_argument('--background-file', type=Path)
    p.add_argument('--background-mode', choices=['configured', 'ground_only'], default='ground_only', help='ground_only explicitly changes offline test preprocessing only')
    p.add_argument('--build-dir', type=Path, help='Package CMake build directory containing test_tracking_policy flags')
    a = p.parse_args()
    if a.limit < 1:
        p.error('--limit must be at least 1')
    source = a.source.resolve(strict=True)
    bags = []
    inventory = []
    for metadata in sorted(source.glob('*/metadata.yaml')):
        if a.bag and metadata.parent.name not in a.bag:
            continue
        data = yaml.safe_load(metadata.read_text())['rosbag2_bagfile_information']
        topics = data.get('topics_with_message_count', [])
        compatible = any(x['topic_metadata']['name'] == '/ch/lslidar_point_cloud' for x in topics)
        inventory.append({'bag': metadata.parent.name, 'compatible': compatible, 'reason': '' if compatible else 'NO_SUPPORTED_RAW_TOPIC'})
        if not compatible:
            continue
        bags.append((metadata.parent, data))
    if a.bag and set(a.bag) - {b.name for b, _ in bags}:
        p.error('Requested bags missing or lack /ch/lslidar_point_cloud')
    if not bags:
        p.error('No compatible metadata.yaml bags found')
    bags = bags[-a.limit:]
    selected_names = {b.name for b, _ in bags}
    for item in inventory:
        item['selected'] = item['bag'] in selected_names
    if a.list_only:
        for b, d in bags:
            print(b.name, d.get('message_count'), 'messages')
        for item in inventory:
            if not item['compatible']:
                print(item['bag'], 'SKIPPED:', item['reason'])
        return 0
    scratch = a.scratch.resolve(); results = a.results.resolve()
    for x in (scratch, results):
        if x == source or source in x.parents or x in source.parents:
            p.error('Scratch/results must be separate from source')
    if scratch == results or scratch in results.parents or results in scratch.parents:
        p.error('Results and scratch must be separate')
    scratch.mkdir(parents=True, exist_ok=True)
    # Reject NAS-backed scratch even when mounted somewhere other than /mnt/z.
    probe = subprocess.check_output(['findmnt', '-n', '-o', 'FSTYPE', '-T', str(scratch)], text=True).strip()
    if probe in ('cifs', 'nfs', 'nfs4', 'smb3', 'fuse.sshfs'):
        p.error('Scratch must reside on local storage')
    results.mkdir(parents=True, exist_ok=False)
    (results/'bag_inventory.json').write_text(json.dumps(inventory, indent=2))
    params = yaml.safe_load(a.params.read_text())
    blocks = [v['ros__parameters'] for v in params.values() if isinstance(v, dict) and 'ros__parameters' in v]
    if len(blocks) != 1:
        p.error('Expected one parameter block')
    settings = blocks[0]
    if a.engine == 'v2':
        overlay = yaml.safe_load((PKG/'config/general_v2.yaml').read_text())['/**']['ros__parameters']
        settings.update(overlay)
    if a.background_mode != 'configured':
        settings['background_mode'] = a.background_mode
    if a.background_file:
        settings['background_map_file'] = str(a.background_file.resolve(strict=True))
    if settings.get('background_mode', 'static') == 'static' and not Path(settings.get('background_map_file', '')).is_file():
        p.error('Static background missing: supply --background-file MAP.yaml, or explicitly choose --background-mode ground_only. No empty-scene learning is assumed.')
    from ament_index_python.packages import get_package_share_directory
    def resolve_input(value):
        if not value:
            return value
        if value.startswith('package://'):
            package, relative = value[10:].split('/',1)
            return str(Path(get_package_share_directory(package))/relative)
        name = value[1:] if value.startswith('/') else value
        if '/' not in name:
            return str(Path(get_package_share_directory('cx128s2_bev_perception'))/'config'/name)
        return value
    for key in ('roi_file','roi_policy_file','object_memory_bank_file'):
        if settings.get(key):
            settings[key] = resolve_input(settings[key])
    env = dict(os.environ, ROS_DOMAIN_ID='173', ROS_LOCALHOST_ONLY='1', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    if a.build_dir:
        env['PERCEPTION_LAB_BUILD_DIR'] = str(a.build_dir.resolve())
    if a.engine == 'legacy':
        with (results/'build.log').open('w') as log:
            subprocess.run([sys.executable, str(PKG/'test/build_replay.py'), '--full-only', '--output', str(results/'engine')], env=env, stdout=log, stderr=subprocess.STDOUT, check=True)
        executable = results/'engine/general_replay'
    else:
        build_dir = a.build_dir.resolve() if a.build_dir else PKG.parents[1]/'build/cx128s2_bev_perception'
        executable = build_dir/'general_v2_replay'
        # An isolated replay must not load an older installed core with a different ABI.
        env['LD_LIBRARY_PATH'] = str(build_dir)+os.pathsep+env.get('LD_LIBRARY_PATH','')
        if not executable.is_file():
            p.error('General V2 replay executable missing; first colcon build with BUILD_TESTING=ON, or provide --build-dir')
        info = json.loads(subprocess.check_output([str(executable),'--build-info'], env=env, text=True))
        inputs = sorted([*PKG.glob('src/general_v2/*.cpp'), *PKG.glob('src/general_v2/*.hpp'), *PKG.glob('include/cx128s2_bev_perception/general_v2/*.hpp'), PKG/'include/cx128s2_bev_perception/vehicle_status.hpp'])
        signature = ''.join(str(f.relative_to(PKG))+':'+hashlib.sha256(f.read_bytes()).hexdigest()+'\n' for f in inputs)
        if info.get('core_source_sha256') != info.get('source_sha256'):
            p.error('General V2 replay/core library mismatch; use a matching build directory')
        if info.get('source_sha256') != hashlib.sha256(signature.encode()).hexdigest():
            p.error('General V2 executable is stale for current source; rebuild before testing')
        (results/'build_info.json').write_text(json.dumps(info,indent=2))
    (results/'source_manifest.json').write_text(json.dumps({str(f.relative_to(PKG)): hashlib.sha256(f.read_bytes()).hexdigest() for root in ('src','include','config','tools','test') for f in (PKG/root).rglob('*') if f.is_file() and f.suffix in ('.cpp','.hpp','.py','.yaml')}, indent=2))
    summaries = []
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    try:
        for bag, metadata in bags:
            out = results/bag.name; out.mkdir()
            report = {'bag':bag.name, 'source':str(bag), 'state':'ERROR', 'ground_truth':'UNKNOWN', 'background_mode':settings.get('background_mode'), 'engine':a.engine}
            report['ground_policy'] = {key: settings.get(key) for key in ('use_fixed_ground_plane','fixed_ground_z','ground_margin_near','ground_margin_mid','ground_margin_long','obstacle_min_height_above_ground')}
            started = time.monotonic()
            try:
                # TemporaryDirectory owns exactly this newly allocated directory; never rmtree a supplied path.
                with tempfile.TemporaryDirectory(prefix='owned-general-', dir=scratch) as temp:
                    local = Path(temp)/bag.name; local.mkdir()
                    members = ['metadata.yaml', *metadata['relative_file_paths']]
                    total = 0; files = []
                    for name in members:
                        rel = Path(name)
                        if rel.is_absolute() or '..' in rel.parts:
                            raise ValueError('Unsafe metadata relative file path')
                        src = bag/rel
                        if any(x.is_symlink() for x in [src, *src.parents] if x != source.parent):
                            raise ValueError('Symlink source rejected')
                        src.resolve(strict=True).relative_to(bag.resolve())
                        stat = src.stat(); total += stat.st_size; files.append((rel,src,stat))
                    if shutil.disk_usage(scratch).free < total + 2*1024**3:
                        raise RuntimeError('Insufficient local scratch space (bag + 2 GiB required)')
                    copied = []
                    for rel, src, before in files:
                        dst=local/rel; dst.parent.mkdir(parents=True,exist_ok=True); digest=hashlib.sha256()
                        with src.open('rb') as inp, dst.open('xb') as output:
                            while chunk := inp.read(8*1024**2):
                                output.write(chunk); digest.update(chunk)
                        after=src.stat()
                        if (before.st_size,before.st_mtime_ns)!=(after.st_size,after.st_mtime_ns) or dst.stat().st_size!=before.st_size:
                            raise RuntimeError('Source changed during copying')
                        copied.append({'file':str(rel),'bytes':before.st_size,'sha256':digest.hexdigest()})
                    (out/'copy_manifest.json').write_text(json.dumps(copied,indent=2))
                    runtime = json.loads(json.dumps(params)); cfg=next(v['ros__parameters'] for v in runtime.values() if isinstance(v,dict) and 'ros__parameters' in v)
                    # Prevent calibration/memory writes to production paths.
                    for key in ('auto_save_background_after_learning','auto_save_object_memory_bank_after_commit','publish_debug_topics','publish_recording_frame'):
                        cfg[key]=False
                    runtime_inputs = []
                    for key in ('background_map_file','roi_file','roi_policy_file','object_memory_bank_file'):
                        if a.engine == 'v2' and key in ('background_map_file','object_memory_bank_file'):
                            continue
                        if cfg.get(key) and Path(cfg[key]).is_file():
                            dest=Path(temp)/(key+'.yaml'); shutil.copy2(cfg[key],dest)
                            runtime_inputs.append({'parameter':key,'source':cfg[key],'sha256':hashlib.sha256(dest.read_bytes()).hexdigest()})
                            cfg[key]=str(dest)
                    for key in ('background_table_file','track_table_file','candidate_table_file'):
                        cfg[key]=str(out/(key+'.csv'))
                    cfg['perception_mode']='default'; cfg['input_topic']='/ch/lslidar_point_cloud'
                    (out/'runtime_inputs.json').write_text(json.dumps(runtime_inputs,indent=2))
                    config=Path(temp)/'runtime.yaml'; config.write_text(yaml.safe_dump(runtime))
                    (out/'runtime_parameters.yaml').write_text(yaml.safe_dump(runtime))
                    if a.engine == 'v2':
                        previews = Path(temp)/'preview'
                        cmd=[str(executable),str(local),str(out/'replay'),str(previews),'--ros-args','--params-file',str(config)]
                    else:
                        cmd=[str(executable),'full',str(local),str(Path(temp)/'unused-cache'),str(out/'replay'),'--fast','--ros-args','--params-file',str(config)]
                    (out/'command.json').write_text(json.dumps(cmd,indent=2))
                    with (out/'replay.log').open('w') as log:
                        subprocess.run(cmd, env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
                    stats=json.loads((out/'replay_stats.json').read_text()); report.update(stats)
                    expected=sum(t['message_count'] for t in metadata['topics_with_message_count'] if t['topic_metadata']['name']=='/ch/lslidar_point_cloud')
                    report['expected_cloud_frames']=expected
                    output_frames=0; vehicle_frames=0; statuses={}; ids=set()
                    with (out/'replay_vehicles.jsonl').open() as public:
                        for line in public:
                            row=json.loads(line); output_frames+=1
                            vehicle_frames+=bool(row['vehicles'])
                            for v in row['vehicles']:
                                ids.add(v['id']); statuses[v['status']]=statuses.get(v['status'],0)+1
                    report.update(output_frames=output_frames, vehicle_frames=vehicle_frames, vehicle_ids=sorted(ids), vehicle_status_observations=statuses)
                    if output_frames != expected:
                        raise RuntimeError('Some input frames did not reach public output boundary; inspect TF/background/logs')
                    if stats['frames']!=expected:
                        raise RuntimeError('Replay frame count differs from metadata')
                    rows=list(csv.DictReader((out/'replay_times.csv').open()))
                    timing=sorted(float(r['pipeline_ms']) for r in rows)
                    report['pipeline_ms']={'mean':sum(timing)/len(timing),'p99':timing[min(len(timing)-1,int(len(timing)*.99))],'max':max(timing)} if timing else {}
                    if a.engine == 'v2':
                        if stats.get('quality_counts',{}).get('INPUT_UNAVAILABLE',0):
                            raise RuntimeError('Some source frames have unavailable TF; review sensor-time transforms')
                        from general_v2_render import render
                        render(previews, out)
                    report['state']='COMPLETED_UNREVIEWED'
            except Exception as e:
                report['error']=str(e)
            finally:
                report['elapsed_sec']=time.monotonic()-started
                (out/'summary.json').write_text(json.dumps(report,indent=2)); summaries.append(report)
                (results/'summary.json').write_text(json.dumps(summaries,indent=2))
                links=''.join(f'<li>{html.escape(x["bag"])}: {html.escape(x["state"])} — <a href="{html.escape(x["bag"])}/summary.json">Summary</a> / <a href="{html.escape(x["bag"])}/replay_vehicles.jsonl">Vehicles</a> / <a href="{html.escape(x["bag"])}/replay_tracks.csv">Tracks</a> / <a href="{html.escape(x["bag"])}/replay_times.csv">Timing</a> / <a href="{html.escape(x["bag"])}/replay.log">Log</a> / <a href="{html.escape(x["bag"])}/contact_sheet.png">PointCloud</a> / <a href="{html.escape(x["bag"])}/timeline.png">Timeline</a> / <a href="{html.escape(x["bag"])}/visual_review.html">Visual review</a></li>' for x in summaries)
                (results/'review_index.html').write_text('<!doctype html><meta charset="utf-8"><title>General review</title><h1>General offline review</h1><p>COMPLETED means execution completed; detection accuracy requires human review. Original NAS bags are preserved.</p><ul>'+links+'</ul>')
                print(bag.name, report['state'], flush=True)
    except KeyboardInterrupt:
        print('Interrupted: owned temporary bag copy cleaned; partial results retained.', file=sys.stderr)
        return 130
    return 0 if all(r['state']=='COMPLETED_UNREVIEWED' for r in summaries) else 1

if __name__=='__main__':
    sys.exit(main())
