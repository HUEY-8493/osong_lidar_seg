#!/usr/bin/env python3
"""Persistent General datasets. Only prepare/verify use rosbag; test/inspect do not."""
from pathlib import Path
import argparse,csv,hashlib,json,os,signal,subprocess,sys,time,fcntl
HERE=Path(__file__).resolve().parent
WS=HERE.parents[2]
ROOT=WS/'general_replay_dataset'
BAGS={'20261023':'rosbag2_2026_10_23-05_26_42','20261020':'rosbag2_2026_10_20-06_14_32','20260729':'rosbag2_2026_07_29-06_12_46','20261007':'rosbag2_2026_10_07-07_40_05'}
def read(p):return json.loads(p.read_text())
def save(p,v):p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(v,indent=2,ensure_ascii=False))
def digest(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for b in iter(lambda:f.read(1048576),b''):h.update(b)
 return h.hexdigest()
def shell_env():
 env=os.environ.copy()
 if True: # Always select this workspace, even if caller sourced another ROS workspace.
  raw=subprocess.check_output(['bash','-c','source /opt/ros/jazzy/setup.bash\nsource /home/sjjin/construct_ws/install/setup.bash\nenv -0'])
  env.update(dict(x.decode().split('=',1) for x in raw.split(b'\0') if b'=' in x))
 env.update(ROS_DOMAIN_ID=env.get('GENERAL_REPLAY_DOMAIN','73'),ROS_LOCALHOST_ONLY='1')
 return env
def build(root):
 t=time.perf_counter();subprocess.run([sys.executable,str(HERE/'build_replay.py'),'--output',str(root/'tool')],env=shell_env(),check=True)
 return time.perf_counter()-t
def execute(root,key,mode,out,*,fast=False,stop=0,inspect=0):
 out.parent.mkdir(parents=True,exist_ok=True)
 cmd=['taskset','-c',os.environ.get('GENERAL_REPLAY_CORES','12-17'),str(root/'tool/general_replay'),mode,str(Path('/home/sjjin/enterance_ros_bag')/BAGS[key]),str(root/key),str(out),'--dataset','--depth-cache','--compress']
 if fast:cmd+=['--fast']
 if stop:cmd+=['--stop-frame',str(stop)]
 if inspect:cmd+=['--inspect-frame',str(inspect)]
 cmd+=['--ros-args','--params-file',str(HERE.parent/'config/cx128s2_bev_perception.yaml'),'-p','background_map_file:='+str(HERE.parent/'config/cx128s2_static_background.yaml'),'-p','roi_policy_file:='+str(HERE.parent/'config/cx128s2_roi_policy.yaml')]
 start=time.perf_counter()
 print(f'{key}: {mode} → {out}',flush=True)
 with Path(str(out)+'.log').open('w') as log:
  child=subprocess.Popen(cmd,env=shell_env(),stdout=log,stderr=subprocess.STDOUT)
  try:code=child.wait()
  except BaseException:
   child.terminate();child.wait();raise
 if code:raise RuntimeError(f'Failed ({code}): {out}.log')
 stats=read(Path(str(out)+'_stats.json'));stats.update(process_wall_sec=time.perf_counter()-start,command=cmd)
 save(Path(str(out)+'_stats.json'),stats);return stats
def index(root,key):
 d=root/key;m=read(d/'manifest.json')
 if not m.get('complete') or digest(d/'frames.index')!=m['index_sha256']:raise RuntimeError('Incomplete/corrupt frame index: '+key)
 rows=[json.loads(x) for x in (d/'frames.index').read_text().splitlines()]
 if len(rows)!=m['frames'] or (d/'frames.bin').stat().st_size!=m['bytes']:raise RuntimeError('Size/frame mismatch: '+key)
 end=0
 for i,r in enumerate(rows,1):
  if r['frame']!=i or r['input_sequence']!=i or r['offset']!=end:raise RuntimeError('Index order/gap mismatch: '+key)
  end+=r['record_bytes']
 if end!=m['bytes']:raise RuntimeError('Trailing record/index mismatch')
 return rows
def verify(root,key):
 index(root,key);out=root/'results'/key/'verification';f=out/'full';c=out/'dataset'
 fs=execute(root,key,'full',f);cs=execute(root,key,'cache-replay',c)
 import itertools
 n=0
 with Path(str(f)+'_evidence.jsonl').open() as left,Path(str(c)+'_evidence.jsonl').open() as right:
  for n,(x,y) in enumerate(itertools.zip_longest(left,right),1):
   if x!=y:
    a=json.loads(x) if x else {};b=json.loads(y) if y else {}
    def first(a,b,path=''):
     if isinstance(a,dict) and isinstance(b,dict):
      for k in dict.fromkeys([*a,*b]):
       if a.get(k)!=b.get(k):return first(a.get(k),b.get(k),path+'.'+k)
     return {'path':path,'full':a,'dataset':b}
    result={'equal':False,'first_frame':n,'mismatch':first(a,b)};save(out/'verification.json',result);raise RuntimeError(str(result))
 result={'equal':True,'frames':n,'integer_tolerance':0,'float_tolerance':0,'manifest_sha256':digest(root/key/'manifest.json'),'candidate_sha256':digest(root/'tool/general_replay'),'full':fs,'dataset':cs}
 save(out/'verification.json',result);print(f'{key}: {n} frames bit-exact',flush=True);return result
def tracks(prefix):return list(csv.DictReader(Path(str(prefix)+'_tracks.csv').open()))
def summary(prefix):
 rows=tracks(prefix);observed={(int(x['frame']),int(x['id'])) for x in rows if x['lost']=='0' and x['vehicle']=='1'}
 public=[x for x in rows if x['vehicle']=='1'];ids=sorted({i for _,i in observed})
 result={'first_public_sec':min((float(x['stamp']) for x in public),default=None),'actual_vehicle_frame_id_pairs':len(observed),'actual_vehicle_frames':len({f for f,_ in observed}),'ids':ids,'maximum_observation_gap':{}}
 for i in ids:
  frames=sorted(f for f,j in observed if j==i);result['maximum_observation_gap'][str(i)]=max((b-a-1 for a,b in zip(frames,frames[1:])),default=0)
 return result,observed
def selection(rows,a):
 if a.frames:
  v=a.frames.split(':');lo=int(v[0]);hi=int(v[-1]);selected=[r['frame'] for r in rows if lo<=r['frame']<=hi]
 elif a.seconds:
  lo,hi=map(float,a.seconds.split(':'));selected=[r['frame'] for r in rows if lo<=r['relative_sec']<=hi]
 else:selected=[]
 if (a.frames or a.seconds) and not selected:raise RuntimeError('No frames in requested interval')
 return selected
def inspect(root,key,frame,out,track_prefix=None):
 rows=index(root,key);execute(root,key,'cache-replay',out,inspect=frame)
 import numpy as np
 import matplotlib;matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 data=read(Path(str(out)+'_frame.json'));points=np.array(data['points']);origin=np.array(data['sensor_origin']);rot=np.array(data['sensor_rotation']);sensor=(points[:,:3]-origin)@rot
 fig,axes=plt.subplots(2,2,figsize=(17,13));bev,angular,depth_ax,ring_ax=axes.flat
 bev.scatter(points[:,0],points[:,1],s=.2,c='#999999',label='actual World points')
 roi=np.array(data['roi_indices'],dtype=int)
 if len(roi):bev.scatter(points[roi,0],points[roi,1],s=.3,c='#55aacc',label='actual ROI points')
 for c in data['clusters']:
  ix=np.array(c['indices'],dtype=int)
  if len(ix):bev.scatter(points[ix,0],points[ix,1],s=3,label=f"C{c['id']} n={len(ix)}");bev.text(*points[ix,:2].mean(axis=0),f"C{c['id']}:{len(ix)}",fontsize=6)
 for z in data['roi_zones']:
  if not z['enabled']:continue
  if z['shape']=='polygon' and z['polygon_x']:
   x=z['polygon_x'];y=z['polygon_y'];bev.plot(x+[x[0]],y+[y[0]],'k--',lw=.8)
  else:
   lo,hi,bot,top=z['bounds'];bev.plot([lo,hi,hi,lo,lo],[bot,bot,top,top,bot],'k--',lw=.8)
 if track_prefix:
  shown=0
  frame_tracks=[tr for tr in tracks(track_prefix) if int(tr['frame'])==frame]
  frame_tracks.sort(key=lambda tr:(tr['vehicle']!='1',tr['lost']!='0',int(tr['id'])))
  for tr in frame_tracks:
   observed=tr['lost']=='0';vehicle=tr['vehicle']=='1'
   bev.plot(float(tr['x']),float(tr['y']),'o' if observed else 'x',color=('red' if vehicle else '#555555') if observed else 'purple',markersize=4)
   if shown<6:
    bev.annotate(f"T{tr['id']} {'VEHICLE '+tr['status'] if vehicle else 'candidate'} {'obs' if observed else 'prediction'}",(float(tr['x']),float(tr['y'])),xytext=(5,5+shown*8),textcoords='offset points',fontsize=6);shown+=1
 distance=np.linalg.norm(sensor,axis=1);az=np.degrees(np.arctan2(sensor[:,1],sensor[:,0]));el=np.degrees(np.arctan2(sensor[:,2],np.hypot(sensor[:,0],sensor[:,1])))
 sc=angular.scatter(az,el,c=distance,s=.5,cmap='viridis');fig.colorbar(sc,ax=angular,label='actual return range (m)')
 cells=data.get('depth_cells',[])
 if cells:
  q=np.array([c['world'] for c in cells]);local=(q-origin)@rot
  da=np.degrees(np.arctan2(local[:,1],local[:,0]));de=np.degrees(np.arctan2(local[:,2],np.hypot(local[:,0],local[:,1])))
  z=depth_ax.scatter(da,de,c=[c['nearest'] for c in cells],s=2,cmap='viridis');fig.colorbar(z,ax=depth_ax,label='initial depth nearest return (m)')
 depth_ax.set(title='Stored pre-remap depth (cluster ownership, no Track ID)',xlabel='azimuth (deg)',ylabel='elevation (deg)')
 rings=points[:,4].astype(int);medians=[];ring_ids=[]
 for r in sorted(set(rings)):
  e=el[rings==r][:512]
  if len(e):ring_ids.append(r);medians.append(float(np.sort(e)[len(e)//2]))
 ring_ax.plot(ring_ids,medians,'.');ring_ax.set(title='Actual ring elevation profile (first ≤512 returns/ring)',xlabel='Ring ID',ylabel='median elevation (deg)')
 row=rows[frame-1];angular.set_title(f"Actual sensor returns | az cell={row['azimuth_bin_deg']:.4f}°, ray={row['ray_elevation_spacing_deg']:.4f}°")
 angular.set(xlabel='azimuth (deg)',ylabel='elevation (deg)');bev.set(xlabel='World X (m)',ylabel='World Y (m)',title=f'{key} frame {frame} +{row["relative_sec"]:.3f}s | actual points / clusters; dashed ROI; x prediction')
 bev.set_aspect('equal',adjustable='box');bev.set_xlim(points[:,0].min()-3,points[:,0].max()+3);bev.set_ylim(points[:,1].min()-3,points[:,1].max()+3);fig.tight_layout();fig.savefig(str(out)+'.png',dpi=140);plt.close(fig)
 return str(out)+'.png'
def timeline(prefix,out):
 import matplotlib;matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 rows=tracks(prefix);fig,ax=plt.subplots(figsize=(12,4))
 for ident in sorted({r['id'] for r in rows},key=int):
  actual=[r for r in rows if r['id']==ident and r['lost']=='0'];pred=[r for r in rows if r['id']==ident and r['lost']!='0']
  ax.scatter([float(r['stamp']) for r in actual],[int(ident)]*len(actual),s=4,c='green')
  ax.scatter([float(r['stamp']) for r in pred],[int(ident)]*len(pred),s=5,c='purple',marker='x')
 ax.set(xlabel='Seconds from first cloud',ylabel='Track ID',title='Actual associated observations (green) / prediction-only (purple x), including candidates')
 fig.tight_layout();fig.savefig(out,dpi=130);plt.close(fig)
def main():
 global BAGS
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('command',choices=['prepare','test','compare','inspect','verify','list']);p.add_argument('--root',type=Path,default=ROOT);p.add_argument('--bag',default='all');p.add_argument('--bag-dir',type=Path,help='Folder containing rosbag directories, or one bag directory');p.add_argument('--label',default='current');p.add_argument('--baseline',default='baseline');p.add_argument('--frames');p.add_argument('--seconds');p.add_argument('--frame',type=int,default=1);p.add_argument('--regenerate',action='store_true');p.add_argument('--verify',action='store_true');p.add_argument('--repeat',type=int,default=1);p.add_argument('--full-reference',action='store_true');a=p.parse_args()
 if any('/' in x or x in ('.','..') for x in (a.label,a.baseline)):p.error('labels must be simple directory names')
 if a.repeat<1:p.error('--repeat must be positive')
 if a.frame<1:p.error('--frame must be at least 1')
 if a.full_reference and (a.frames or a.seconds):p.error('--full-reference requires full sequence')
 root=a.root.resolve();root.mkdir(parents=True,exist_ok=True)
 registry=root/'datasets.json'
 if a.bag_dir:
  source=a.bag_dir.resolve()
  folders=[source] if (source/'metadata.yaml').is_file() else sorted(x.parent for x in source.glob('*/metadata.yaml'))
  if not folders:p.error('No rosbag metadata.yaml found in '+str(source))
  discovered={folder.name:str(folder) for folder in folders}
  previous=read(registry) if registry.exists() else {}
  for key,path in discovered.items():
   if key in previous and previous[key]!=path:p.error('Bag key already registered at a different path; use a separate --root: '+key)
  BAGS={**previous,**discovered}
  if a.command!='list':save(registry,BAGS)
 elif registry.exists():BAGS=read(registry)
 if a.bag!='all' and a.bag not in BAGS:p.error('Unknown bag key; select from '+', '.join(BAGS))
 keys=list(BAGS) if a.bag=='all' else [a.bag]
 if a.command=='list':
  for key in keys:
   try:r=index(root,key);print(key,len(r),'frames',f'{(root/key/"frames.bin").stat().st_size/2**30:.2f} GiB','prepared (compatibility checked on test)',flush=True)
   except (FileNotFoundError,KeyError,RuntimeError) as e:print(key,'not ready:',e)
  return
 # Single writer/executable build per root; shared datasets never modified by tests.
 with (root/'.command.lock').open('w') as lock:
  fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);build_sec=build(root)
  for key in keys:
   d=root/key
   if a.command=='prepare':
    if a.regenerate and d.exists():d.rename(root/(key+'.archived.'+str(time.time_ns())))
    if d.exists():index(root,key);print(key,'already prepared; preserving dataset',flush=True)
    else:
     execute(root,key,'cache-build',root/'results'/key/'prepare'/'build')
     for file in d.iterdir():file.chmod(0o444)
     d.chmod(0o555)
    if a.verify:verify(root,key)
   elif a.command=='verify':verify(root,key)
   elif a.command=='test':
    rows=index(root,key);selected=selection(rows,a);stop=max(selected) if selected else 0;out=root/'results'/key/a.label;measured=[]
    for i in range(a.repeat):
     prefix=out/('run' if i==0 else f'run_{i+1}');s=execute(root,key,'cache-replay',prefix,fast=True,stop=stop);s['summary']=summary(prefix)[0]
     import numpy as np
     timing=list(csv.DictReader(Path(str(prefix)+'_times.csv').open()))
     for label,columns in {'frame_ms':['reader_ms','deserialize_ms','cache_load_ms','pipeline_ms','result_ms'],'pipeline_ms':['pipeline_ms'],'load_ms':['cache_load_ms']}.items():
      values=[sum(float(row[c]) for c in columns) for row in timing]
      s[label]={'mean':float(np.mean(values)),'p95':float(np.percentile(values,95)),'p99':float(np.percentile(values,99)),'max':float(np.max(values))}
     s['cpu_percent']=100*(s['user_sec']+s['system_sec'])/s['process_wall_sec']
     if a.full_reference:
      if stop:raise RuntimeError('Full reference requires full sequence')
      s['full_reference']=execute(root,key,'full',out/f'full_{i+1}',fast=True)
      if Path(str(prefix)+'_tracks.csv').read_bytes()!=Path(str(out/f'full_{i+1}')+'_tracks.csv').read_bytes():raise RuntimeError('Repeated Full/Dataset Track CSV mismatch')
     measured.append(s)
    report={'build_sec':build_sec,'runs':measured,'selection':selected,'warmup_from_frame':1,'cumulative':{}}
    for n in (1,5,10):
     if len(measured)>=n:
      v={'dataset_sec':sum(x['process_wall_sec'] for x in measured[:n])}
      if a.full_reference:v['full_sec']=sum(x['full_reference']['process_wall_sec'] for x in measured[:n]);v['saved_sec']=v['full_sec']-v['dataset_sec']
      report['cumulative'][str(n)]=v
    save(out/'results.json',report)
    timeline(out/'run',out/'timeline.png')
    # Detailed PNG only for selected endpoints; all intermediate states are in CSV.
    for f in sorted(set(selected[:1]+selected[-1:])):inspect(root,key,f,out/f'frame_{f:06}',out/'run')
   elif a.command=='inspect':
    rows=index(root,key);selected=selection(rows,a) or [a.frame]
    if max(selected)>len(rows):raise RuntimeError('Frame outside dataset')
    out=root/'results'/key/a.label/'inspect'
    # Warm up current algorithm from frame 1 through the requested last frame.
    execute(root,key,'cache-replay',out/'warm',fast=True,stop=max(selected))
    for f in selected:inspect(root,key,f,out/f'frame_{f:06}',out/'warm')
   elif a.command=='compare':
    left=root/'results'/key/a.baseline/'run';right=root/'results'/key/a.label/'run';ls,lo=summary(left);rs,ro=summary(right);out=root/'results'/key/(a.baseline+'_vs_'+a.label);out.mkdir(parents=True,exist_ok=True)
    save(out/'comparison.json',{'baseline':ls,'candidate':rs,'lost_observations':sorted(lo-ro),'gained_observations':sorted(ro-lo)})
    l={(x['frame'],x['id']):x for x in tracks(left)};r={(x['frame'],x['id']):x for x in tracks(right)}
    with (out/'comparison.csv').open('w') as f:
     w=csv.writer(f);w.writerow(['frame','id','baseline','candidate','equal'])
     for k in sorted(l.keys()|r.keys(),key=lambda x:tuple(map(int,x))):w.writerow([*k,json.dumps(l.get(k)),json.dumps(r.get(k)),l.get(k)==r.get(k)])
    frames=sorted({f for f,_ in (lo^ro)})[:4] or [1]
    for f in frames:
     inspect(root,key,f,out/f'baseline_{f:06}',left);inspect(root,key,f,out/f'candidate_{f:06}',right)
if __name__=='__main__':
 try:main()
 except KeyboardInterrupt:print('Stopped; completed datasets preserved',file=sys.stderr);sys.exit(130)
 except Exception as e:print('DATASET ERROR:',e,file=sys.stderr);sys.exit(1)
