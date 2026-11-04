#!/usr/bin/env bash
# Offline synchronous replay only. Does not start/stop services or connect to Redis.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /home/sjjin/construct_ws/install/setup.bash
export ROS_DOMAIN_ID=${GENERAL_REPLAY_DOMAIN:-73}
export ROS_LOCALHOST_ONLY=1
export GENERAL_REPLAY_TEST_DIR="$(cd "$(dirname "$0")" && pwd)"
python3 - "$@" <<'PY'
import argparse,csv,json,os,subprocess,sys,time,math,hashlib
from pathlib import Path
p=argparse.ArgumentParser(description='General offline Phase A replay cache')
p.add_argument('mode',choices=['full','cache-build','cache-replay','cache-verify','benchmark'])
p.add_argument('--bag',required=True,type=Path);p.add_argument('--cache',required=True,type=Path);p.add_argument('--output',required=True,type=Path)
p.add_argument('--tool',required=True,type=Path);p.add_argument('--params',type=Path);p.add_argument('--background',type=Path);p.add_argument('--roi',type=Path)
p.add_argument('--reuse-reference',action='store_true');p.add_argument('--compress',action='store_true');p.add_argument('--fast',action='store_true');p.add_argument('--depth-cache',action='store_true');p.add_argument('--sequence',type=Path);p.add_argument('--repeat',type=int,default=10);p.add_argument('--cores',default='12-17')
a=p.parse_args();test=Path(os.environ['GENERAL_REPLAY_TEST_DIR']);pkg=test.parent
a.output.mkdir(parents=True,exist_ok=True)
subprocess.run([sys.executable,str(test/'build_replay.py'),'--output',str(a.tool)],check=True)
params=a.params or pkg/'config/cx128s2_bev_perception.yaml';bg=a.background or pkg/'config/cx128s2_static_background.yaml';roi=a.roi or pkg/'config/cx128s2_roi_policy.yaml'
def run(mode,name,fast=False):
 if fast and mode=='cache-replay':
  marker=a.cache/'verified.json'
  if not marker.exists():raise RuntimeError('Cache has not passed strict verification; run cache-verify first')
  v=json.loads(marker.read_text())
  if not v.get('equal') or any(v.get(k)!=value for k,value in reference_binding().items()):raise RuntimeError('Strict verification is stale; run cache-verify first')
 prefix=a.output/name;cmd=['taskset','-c',a.cores,str(a.tool.resolve()/'general_replay'),mode,str(a.bag.resolve()),str(a.cache.resolve()),str(prefix.resolve())]
 if a.sequence:cmd+=['--sequence',str(a.sequence.resolve())]
 if a.depth_cache:cmd+=['--depth-cache']
 if fast:cmd+=['--fast']
 if a.compress:cmd+=['--compress']
 cmd+=['--ros-args','--params-file',str(params.resolve()),'-p','background_map_file:='+str(bg.resolve()),'-p','roi_policy_file:='+str(roi.resolve())]
 t=time.perf_counter()
 with open(str(prefix)+'.log','w') as f:
  result=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT)
 elapsed=time.perf_counter()-t
 if result.returncode:raise RuntimeError(f'{mode} failed ({result.returncode}); see {prefix}.log')
 stats=json.loads(Path(str(prefix)+'_stats.json').read_text());stats['process_wall_sec']=elapsed;stats['command']=cmd
 Path(str(prefix)+'_stats.json').write_text(json.dumps(stats,indent=2));print(json.dumps({'mode':mode,'frames':stats['frames'],'process_wall_sec':elapsed}),flush=True);return prefix,stats

def reference_binding():
 return {'manifest_sha256':hashlib.sha256((a.cache/'manifest.json').read_bytes()).hexdigest(),'executable_sha256':hashlib.sha256((a.tool/'general_replay').read_bytes()).hexdigest()}
def verify(full,cached):
 n=0
 with open(str(full)+'_evidence.jsonl') as f,open(str(cached)+'_evidence.jsonl') as c:
  for n,(left,right) in enumerate(__import__('itertools').zip_longest(f,c),1):
   if left is None or right is None:raise RuntimeError(f'Frame count mismatch at {n}')
   x,y=json.loads(left),json.loads(right)
   if x!=y:
    def diff(a,b,path=''):
     if type(a)!=type(b):return path,a,b
     if isinstance(a,dict):
      for k in dict.fromkeys([*a,*b]):
       if k not in a or k not in b:return path+'.'+k,a.get(k),b.get(k)
       if a[k]!=b[k]:return diff(a[k],b[k],path+'.'+k)
     return path,a,b
    report={'equal':False,'first_frame':n,'mismatch':diff(x,y)}; (a.output/'verification.json').write_text(json.dumps(report,indent=2));raise RuntimeError(str(report))
 report={'equal':True,**reference_binding(),'frames':n,'integer_tolerance':0,'float_tolerance':0,'comparison':'fieldwise bit-exact SHA256; all Track fields, KF covariance, memory, FIFO, initial/final depth, actual public arrays'}
 (a.output/'verification.json').write_text(json.dumps(report,indent=2));(a.cache/'verified.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
if a.mode in ('full','cache-build','cache-replay'):run(a.mode,a.mode,a.fast)
elif a.mode=='cache-verify':
 f,_=run('full','verify_full');c,_=run('cache-replay','verify_cached');verify(f,c)
else:
 if a.repeat<1:raise ValueError('repeat must be positive')
 creation=None
 if not (a.cache/'manifest.json').exists():_,creation=run('cache-build','benchmark_build')
 elif (a.output/'cache-build_stats.json').exists():creation=json.loads((a.output/'cache-build_stats.json').read_text())
 if a.reuse_reference:
  v=json.loads((a.output/'verification.json').read_text())
  if not v.get('equal') or any(v.get(k)!=value for k,value in reference_binding().items()):raise RuntimeError('Reference verification is stale; run cache-verify')
 else:
  f,_=run('full','benchmark_reference_full');c,_=run('cache-replay','benchmark_reference_cached');verify(f,c)
 full=[];cached=[]
 for i in range(a.repeat):
  f,fs=run('full',f'benchmark_full_{i+1}',True);c,cs=run('cache-replay',f'benchmark_cached_{i+1}',True)
  if fs['frames']!=cs['frames'] or Path(str(f)+'_tracks.csv').read_bytes()!=Path(str(c)+'_tracks.csv').read_bytes():raise RuntimeError('Benchmark public/track CSV mismatch')
  full.append(fs);cached.append(cs)
 report={'full':full,'cached':cached,'cache_creation':creation,'cache_bytes':sum(f.stat().st_size for f in a.cache.rglob('*') if f.is_file()),'build':json.loads((a.tool/'last_build.json').read_text()),'cumulative':{}}
 for n in (1,5,10):
  if n<=a.repeat:report['cumulative'][str(n)]={'full_sec':sum(x['process_wall_sec'] for x in full[:n]),'cached_sec':sum(x['process_wall_sec'] for x in cached[:n]),'cached_plus_create_sec':None if creation is None else creation['process_wall_sec']+sum(x['process_wall_sec'] for x in cached[:n])}
 (a.output/'benchmark.json').write_text(json.dumps(report,indent=2))
PY
