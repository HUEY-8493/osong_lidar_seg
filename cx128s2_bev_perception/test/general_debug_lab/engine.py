#!/usr/bin/env python3
"""Local, read-only dataset viewer; all derived files live under a separate lab state."""
from pathlib import Path
import collections,csv,fcntl,hashlib,json,math,os,re,signal,subprocess,sys,threading,time,uuid
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE.parent))
import general_dataset as dataset
import yaml

def load(p):return json.loads(p.read_text())
def dump(p,value):
 p.parent.mkdir(parents=True,exist_ok=True);tmp=p.with_suffix(p.suffix+'.tmp');tmp.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False));tmp.replace(p)
def safe(v):
 if isinstance(v,float) and not math.isfinite(v):return None
 if isinstance(v,dict):return {k:safe(x) for k,x in v.items()}
 if isinstance(v,(list,tuple)):return [safe(x) for x in v]
 return v
class Lab:
 def __init__(self,state,roots):
  self.state=state;state.mkdir(parents=True,exist_ok=True);self.tool=state/'tool';self.roots=roots;self.lock=threading.RLock();self.engine_lock=threading.Lock();self.jobs={};self.children={};self.indices={};self.runs_cache={};self.frames=collections.OrderedDict();self.offset_cache={};self.environment=dataset.shell_env();self.environment['ROS_DOMAIN_ID']='74';self.environment['GENERAL_LAB_DETAIL_START']='0';self.environment['GENERAL_LAB_DETAIL_END']='0'
  # Re-sourcing the operating workspace can put its old message ABI first.
  if os.environ.get('PERCEPTION_LAB_LIBRARY_DIR'):
   self.environment['LD_LIBRARY_PATH']=os.environ['PERCEPTION_LAB_LIBRARY_DIR']+':'+self.environment.get('LD_LIBRARY_PATH','')
  self.base=yaml.safe_load((HERE.parent.parent/'config/cx128s2_bev_perception.yaml').read_text());self.params={}
  def flatten(x,prefix=''):
   for k,v in x.items():
    name=prefix+k
    if isinstance(v,dict):flatten(v,name+'.')
    else:self.params[name]=v
  flatten(next(iter(self.base.values()))['ros__parameters'])
  for p in (state/'experiments').glob('*/experiment.json'):
   try:
    m=load(p)
    if m['status'] in ('running','queued'):m.update(status='interrupted',error='Application stopped before completion');dump(p,m)
    self.jobs[m['id']]=m
   except (ValueError,KeyError):pass
 def metadata(self,rootid,key):
  if rootid not in self.roots:raise ValueError('Unknown dataset root')
  root=self.roots[rootid]
  if key not in self.keys(rootid):raise ValueError('Unknown dataset')
  path=root/key;manifest=path/'manifest.json'
  if not manifest.is_file():return {'root':rootid,'key':key,'status':'Not Prepared','path':str(path)}
  m=load(manifest)
  if not m.get('complete') or m.get('contract',{}).get('schema')!=4:raise ValueError('Not Prepared: missing completed schema 4 manifest')
  signature=(manifest.stat().st_mtime_ns,(path/'frames.index').stat().st_mtime_ns,(path/'frames.bin').stat().st_size)
  cachekey=(rootid,key)
  if cachekey not in self.indices or self.indices[cachekey][0]!=signature:
   rows=dataset.index(root,key);self.indices[cachekey]=(signature,rows,dataset.digest(manifest))
  rows=self.indices[cachekey][1];verification=None
  if not rows:raise ValueError('Dataset contains no PointCloud frames')
  vp=root/'results'/key/'verification/verification.json'
  if vp.exists():
   v=load(vp)
   if v.get('manifest_sha256')==self.indices[cachekey][2]:verification={'equal':v.get('equal'),'frames':v.get('frames'),'binary':v.get('candidate_sha256')}
  current_prefix=load(self.tool/'prefix_dependencies.json') if (self.tool/'prefix_dependencies.json').exists() else None
  prefix_compatible=current_prefix==m['contract']['prefix']
  return {'prefix_compatible':prefix_compatible,'root':rootid,'key':key,'status':'Prepared','path':str(path),'bag':m.get('provenance',{}).get('bag'),'frames':len(rows),'start_ns':rows[0]['stamp_ns'],'end_ns':rows[-1]['stamp_ns'],'duration_sec':rows[-1]['relative_sec'],'bytes':m['bytes'],'schema':4,'manifest_sha256':self.indices[cachekey][2],'verified':verification,'compatibility':'Checked by C++ on frame/replay; no automatic regeneration','jetson':'Not Available'}
 def keys(self,rootid):
  root=self.roots[rootid];keys=set(dataset.BAGS) if rootid=='local' else set()
  registry=root/'datasets.json'
  if registry.exists():
   try:keys.update(load(registry))
   except ValueError:pass
  if root.exists():keys.update(p.name for p in root.iterdir() if p.is_dir() and (p/'manifest.json').exists() and '.archived.' not in p.name)
  return sorted(k for k in keys if '/' not in k and k not in ('.','..'))
 def datasets(self):
  values=[]
  for rid in self.roots:
   for key in self.keys(rid):
    try:values.append(self.metadata(rid,key))
    except (ValueError,KeyError,OSError) as e:values.append({'root':rid,'key':key,'status':'Not Prepared','error':str(e)})
  return values
 def ready(self,rid,key):
  m=self.metadata(rid,key)
  if m['status']!='Prepared':raise ValueError('Not Prepared: generation is never started by this application')
  return m
 def runs(self,rid,key):
  self.ready(rid,key);runs=[];root=self.roots[rid]
  for stats in (root/'results'/key).rglob('*_stats.json'):
   try:
    s=load(stats);prefix=Path(str(stats)[:-len('_stats.json')]);tracks=Path(str(prefix)+'_tracks.csv')
    if s.get('mode') not in ('full','cache-replay') or not tracks.exists() or tracks.stat().st_mtime_ns>stats.stat().st_mtime_ns:continue
    identifier=hashlib.sha256(str(prefix).encode()).hexdigest()[:20]
    runs.append({'id':identifier,'label':str(prefix.relative_to(root/'results'/key)),'kind':'Full' if s['mode']=='full' else 'Dataset','status':'completed','prefix':str(prefix),'stats':s,'source_version':'verification-bound' if 'verification/' in str(prefix) else 'Not recorded'})
   except (ValueError,OSError):pass
  with self.lock:experiments=list(self.jobs.values())
  for j in experiments:
   if j['root']==rid and j['dataset']==key and j['status']=='completed' and j.get('mode')!='what-if':runs.append({'id':j['id'],'label':j['label'],'kind':j['kind'],'status':j['status'],'prefix':j['prefix'],'stats':j.get('stats'),'source_version':j.get('source_sha256'),'experiment':j})
  self.runs_cache[(rid,key)]={x['id']:x for x in runs};return [{k:v for k,v in x.items() if k!='prefix'} for x in runs]
 def run(self,rid,key,identifier):
  self.runs(rid,key)
  if identifier not in self.runs_cache[(rid,key)]:raise ValueError('Unknown completed run')
  return self.runs_cache[(rid,key)][identifier]
 def rows(self,rid,key):self.ready(rid,key);return self.indices[(rid,key)][1]
 def line(self,path,frame,trace=True):
  if not path.exists():return None
  sig=(str(path),path.stat().st_mtime_ns,path.stat().st_size,trace)
  if sig not in self.offset_cache:
   offsets={}
   with path.open('r') as f:
    if not trace:header=next(csv.reader([f.readline()]))
    while True:
     pos=f.tell();line=f.readline()
     if not line:break
     row=json.loads(line) if trace else dict(zip(header,next(csv.reader([line]))))
     offsets.setdefault(int(row['frame']),[]).append(pos)
   self.offset_cache[sig]=(offsets,None if trace else header)
   if len(self.offset_cache)>16:self.offset_cache.pop(next(iter(self.offset_cache)))
  offsets,header=self.offset_cache[sig];values=[]
  with path.open() as f:
   for pos in offsets.get(frame,[]):
    f.seek(pos);line=f.readline();values.append(json.loads(line) if trace else dict(zip(header,next(csv.reader([line])))))
  return (values[0] if values else None) if trace else values
 def state_at(self,rid,key,identifier,frame):
  run=self.run(rid,key,identifier);prefix=run['prefix'];trace=self.line(Path(prefix+'_trace.jsonl'),frame);rows=self.line(Path(prefix+'_tracks.csv'),frame,False)
  result={'run':identifier,'frame':frame,'trace':trace,'tracks':rows or [],'detail_available':bool(trace and trace.get('detail_evaluated'))}
  if trace and trace.get('detail_evaluated'):
   catalog_path=Path(run['prefix']+'_gate_catalog.json')
   catalog=load(catalog_path if catalog_path.exists() else self.tool/'gate_catalog.json');gates=trace.get('gates',[])
   result['gate_catalog']=catalog
   for tr in trace.get('tracks',[]):
    observed=[g for g in gates if g.get('track')==tr['id']];groups=[];current={}
    # First gate marks each actual invocation; no additional C++ condition is evaluated.
    tr['diagnostic_records']=[g for g in observed if 'gate' not in g or 'pass' not in g]
    for gate in observed:
     if 'gate' not in gate or 'pass' not in gate:continue
     function=gate['function'];starts=gate.get('gate') in ('enabled','point_count','generic_spatial_gate')
     if starts or function not in current:
      group={'function':function,'cluster':gate.get('cluster'),'call':len(groups)+1,'gates':[]};groups.append(group);current[function]=group
     current[function]['gates'].append(gate)
    for group in groups:
     recorded=group['gates'];group['gates']=recorded+[dict(g,evaluated=False,status='NOT_EVALUATED') for g in catalog if g['function']==group['function'] and not any(x.get('gate')==g['gate'] for x in recorded)]
    tr['gate_calls']=groups
    detection=[g for g in groups if g['function']=='evaluateDetection'];latest=detection[-1] if detection else (groups[-1] if groups else None)
    tr['gate_inspector']=latest['gates'] if latest else []
    tr['selected_call']=latest
  return result
 def events(self,rid,key,identifier):
  run=self.run(rid,key,identifier);prefix=Path(run['prefix']);trace=Path(str(prefix)+'_trace.jsonl');previous={};fifo=[];events=[];first_candidate=False;first_vehicle=False
  if not trace.exists():return {'events':[],'notice':'No per-frame trace; unavailable events are not inferred'}
  with trace.open() as f:
   for line in f:
    row=json.loads(line);frame=row['frame'];current={int(t['id']):t for t in row.get('tracks',[])}
    def add(kind,track=None,detail=None):events.append({'frame':frame,'track':track,'kind':kind,'detail':detail})
    if row.get('candidates') and not first_candidate:add('First component candidate',detail='Not a confirmed vehicle');first_candidate=True
    for ident,tr in current.items():
     prev=previous.get(ident)
     if prev is None:add('New Track',ident)
     if prev and tr['lost_frames']>0 and prev['lost_frames']==0:add('Observation lost',ident,'Cause unavailable unless detailed C++ gate recorded')
     if prev and tr['lost_frames']==0 and prev['lost_frames']>0:add('Observation resumed',ident,'Track ID preserved; not proof of physical identity')
     if tr.get('dormant') and (not prev or not prev.get('dormant')):add('DORMANT',ident)
     if tr.get('memory',{}).get('reacquire_result')=='ACCEPTED' and (not prev or prev.get('memory',{}).get('reacquire_result')!='ACCEPTED'):add('Reacquire accepted',ident)
     if tr.get('public') and tr.get('vehicle') and not first_vehicle:add('First Public VEHICLE',ident);first_vehicle=True
    for ident in set(fifo)-set(row.get('fifo',[])):add('FIFO removed',ident)
    for gate in row.get('gates',[]):
     if gate.get('final_reject'):add('Observation pair rejected',gate.get('track'),gate.get('reason'))
    previous=current;fifo=row.get('fifo',[])
  # Actual public vehicle count uses the existing published-contract CSV.
  csvpath=Path(str(prefix)+'_tracks.csv');per={}
  if csvpath.exists():
   for t in dataset.tracks(prefix):per.setdefault(int(t['frame']),[]).append(t)
   was=None
   processed=min(int((run.get('stats') or {}).get('frames',max(per,default=0))),len(self.rows(rid,key)))
   events=[e for e in events if e['kind']!='First Public VEHICLE']
   csv_first=False
   for frame in range(1,processed+1):
    vehicles=[t for t in per.get(frame,[]) if t['vehicle']=='1'];count=len(vehicles)
    if vehicles and not csv_first:
     events.append({'frame':frame,'track':int(vehicles[0]['id']),'kind':'First Public VEHICLE','detail':'Recorded public-contract CSV'})
     csv_first=True
    actual=[t for t in vehicles if t['lost']=='0']
    if actual:events.append({'frame':frame,'track':int(actual[0]['id']),'kind':'Actual VEHICLE observation','detail':'Current associated point evidence; not prediction/memory'})
    if count!=was and count==0:events.append({'frame':frame,'track':None,'kind':'No Public VEHICLE','detail':'Internal candidate/memory may still exist'})
    was=count
  return {'events':sorted(events,key=lambda x:x['frame']),'notice':'Track ID changes are not automatically treated as different physical vehicles; recorded Jetson result unavailable'}
 def parameters(self,rid,key):
  m=self.ready(rid,key);manifest=load(self.roots[rid]/key/'manifest.json');prefix=set(manifest['contract']['prefix']['parameters']);source=(HERE.parent.parent/'src/cx128s2_bev_perception_node.cpp').read_text();values=[]
  for name,value in self.params.items():
   if not isinstance(value,(int,float,bool)):continue
   if name.startswith('crane_'):continue
   if not any(t in name for t in ('kf_','gate_','vehicle_','tentative_','lost_','shape_memory_','motion_')):continue
   assignments=re.findall(r'\b(\w+_)\s*=\s*get_parameter\("'+re.escape(name)+r'"\)',source)
   if not hasattr(self,'function_bodies'):
    import dataset_dependencies as dependency_parser
    data=source.encode();self.function_bodies=[]
    for node in dependency_parser.walk(dependency_parser.PARSER.parse(data).root_node):
     if node.type!='function_definition':continue
     declarator=node.child_by_field_name('declarator')
     while declarator and declarator.type not in ('identifier','field_identifier','qualified_identifier','destructor_name','operator_name'):
      declarator=declarator.child_by_field_name('declarator')
     if declarator:self.function_bodies.append((data[declarator.start_byte:declarator.end_byte].decode().split('::')[-1],data[node.start_byte:node.end_byte].decode()))
   refs=[function for function,body in self.function_bodies if function not in ('declareParams','loadParams') and any(re.search(r'\b'+re.escape(member)+r'\b',body) for member in assignments)]
   declaration=re.search(r'declare_parameter\("'+re.escape(name)+r'",\s*([^;\n]+)\);',source)
   readonly=name in prefix or name=='vehicle_fixed_yaw_rad' or name.startswith('shape_memory_')
   unit='m/s²' if name.endswith('mps2') else 'm/s' if name.endswith('mps') else 'm' if name.endswith('_m') else 's' if name.endswith(('_sec','_seconds')) else 'deg' if name.endswith('_deg') else 'source-defined'
   values.append({'name':name,'current':value,'default_source':declaration[1] if declaration else 'See C++ declaration','unit':unit,'range':'No runtime range descriptor found; engine/type validation applies','functions':sorted(set(refs))[:12],'requires_regeneration':name in prefix,'editable':not readonly,'whatif':name=='vehicle_min_score','requires_replay':True})
  return values
 def command(self,rid,key,mode,prefix,params,stop=0,inspect=0):
  m=self.ready(rid,key);contract=load(self.roots[rid]/key/'manifest.json')['contract'];cmd=['taskset','-c',os.environ.get('GENERAL_REPLAY_CORES','12-17'),str(self.tool/'general_replay'),mode,m.get('bag') or '/unavailable/original_bag',str(self.roots[rid]/key),str(prefix),'--dataset']
  if contract['depth_cache']:cmd+=['--depth-cache']
  if contract['compression']!='none':cmd+=['--compress']
  if stop:cmd+=['--stop-frame',str(stop)]
  if inspect:cmd+=['--inspect-frame',str(inspect)]
  cmd+=['--ros-args','--params-file',str(params),'-p','background_map_file:='+str(HERE.parent.parent/'config/cx128s2_static_background.yaml'),'-p','roi_policy_file:='+str(HERE.parent.parent/'config/cx128s2_roi_policy.yaml')];return cmd
 def frame(self,rid,key,number):
  m=self.ready(rid,key);rows=self.rows(rid,key)
  if not 1<=number<=len(rows):raise ValueError('Frame outside dataset')
  cachekey=(m['manifest_sha256'],number);cached=self.state/'frames'/m['manifest_sha256']/f'{number}.json'
  with self.lock:
   if cachekey in self.frames:self.frames.move_to_end(cachekey);return self.frames[cachekey]
  if cached.exists():value=load(cached)
  else:
   if not self.engine_lock.acquire(blocking=False):raise ValueError('Replay is running; uncached frame loading is temporarily busy')
   try:
    prefix=cached.parent/(str(number)+'_raw');prefix.parent.mkdir(parents=True,exist_ok=True);cmd=self.command(rid,key,'cache-replay',prefix,HERE.parent.parent/'config/cx128s2_bev_perception.yaml',inspect=number)
    with Path(str(prefix)+'.log').open('w') as log:
     done=subprocess.run(cmd,env=self.environment,stdout=log,stderr=subprocess.STDOUT,timeout=45)
    if done.returncode:raise ValueError('Frame reader failed: '+Path(str(prefix)+'.log').read_text()[-1800:])
    raw=load(Path(str(prefix)+'_frame.json'));value=self.lod(raw,rows[number-1]);dump(cached,value)
    Path(str(prefix)+'_frame.json').unlink() # derived full JSON only, never source dataset
   finally:self.engine_lock.release()
  with self.lock:
   self.frames[cachekey]=value
   while len(self.frames)>12:self.frames.popitem(last=False)
  return value
 def lod(self,raw,index):
  import numpy as np
  p=np.array(raw['points'],dtype=float);count=len(p);foreground=set(raw['foreground_indices']);roi=set(raw['roi_indices']);owned={i for c in raw['clusters'] for i in c['indices']};step=max(1,math.ceil(count/16000));selected=sorted(set(range(0,count,step))|set(sorted(owned)[::max(1,math.ceil(len(owned)/8000))]));sensor=(p[selected,:3]-np.array(raw['sensor_origin']))@np.array(raw['sensor_rotation']);points=[]
  for j,i in enumerate(selected):
   x,y,z=sensor[j];points.append([*p[i,:3],int(p[i,4]),raw['owners'][i] if i<len(raw['owners']) else -1,2 if i in foreground else 1 if i in roi else 0,i,float(np.degrees(np.arctan2(y,x))),float(np.degrees(np.arctan2(z,np.hypot(x,y)))),float(np.linalg.norm(sensor[j]))])
  clusters=[]
  for c in raw['clusters']:
   q=p[c['indices'],:3]
   if len(q):clusters.append({'id':c['id'],'count':len(q),'center':q.mean(axis=0).tolist(),'min':q.min(axis=0).tolist(),'max':q.max(axis=0).tolist()})
  depth=raw.get('depth_cells',[]);sampled=depth[::max(1,math.ceil(len(depth)/9000))];returns=[]
  for c in sampled:
   q=(np.array(c['world'])-np.array(raw['sensor_origin']))@np.array(raw['sensor_rotation']);returns.append([float(np.degrees(np.arctan2(q[1],q[0]))),float(np.degrees(np.arctan2(q[2],np.hypot(q[0],q[1])))),c['nearest'],c['owner'],c['count']])
  return safe({'frame':raw['frame'],'stamp':raw['stamp'],'index':index,'actual_points':count,'rendered_points':len(points),'points':points,'clusters':clusters,'features':raw['features'],'roi':raw['roi_zones'],'sensor_origin':raw['sensor_origin'],'sensor_rotation':raw['sensor_rotation'],'depth':returns,'depth_count':len(depth),'notice':'LOD is display-only. Expected rays/visibility use the existing AABB angular model, not ground-truth surface visibility.'})
 def start(self,request):
  rid=request['root'];key=request['dataset'];m=self.ready(rid,key);mode=request.get('mode','dataset')
  if mode not in ('dataset','full','what-if'):raise ValueError('Unsupported experiment mode')
  allowed={x['name']:x for x in self.parameters(rid,key)};overrides=request.get('overrides',{})
  if not isinstance(overrides,dict):raise ValueError('Invalid parameter overrides')
  for name,value in overrides.items():
   if name not in allowed or not allowed[name]['editable']:raise ValueError('Parameter is unavailable or requires dataset regeneration: '+name)
   expected=self.params[name]
   if isinstance(expected,bool):valid=isinstance(value,bool)
   elif isinstance(expected,int):valid=isinstance(value,int) and not isinstance(value,bool)
   else:valid=isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value)
   if not valid:raise ValueError('Parameter type/nonfinite value: '+name)
  if mode=='what-if' and set(overrides)-{'vehicle_min_score'}:raise ValueError('Frozen what-if supports vehicle_min_score only; use Replay for other parameters')
  end=int(request.get('end',m['frames']));start=int(request.get('detail_start',request.get('frame',1)));detail_end=int(request.get('detail_end',start))
  if not 1<=start<=detail_end<=end<=m['frames']:raise ValueError('Invalid frame range; warm-up always starts at frame 1')
  if mode=='full' and end!=m['frames']:raise ValueError('Full comparison uses the complete original sequence')
  identifier=time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8];folder=self.state/'experiments'/identifier;folder.mkdir(parents=True,exist_ok=False);prefix=folder/'run';params=folder/'params.yaml';base=json.loads(json.dumps(self.base));values=next(iter(base.values()))['ros__parameters']
  for name,value in overrides.items():
   target=values;parts=name.split('.')
   for p in parts[:-1]:target=target[p]
   target[parts[-1]]=value
  params.write_text(yaml.safe_dump(base,sort_keys=False))
  meta={'id':identifier,'label':str(request.get('label') or identifier)[:100],'root':rid,'dataset':key,'manifest_sha256':m['manifest_sha256'],'source_sha256':dataset.digest(HERE.parent.parent/'src/cx128s2_bev_perception_node.cpp'),'build_fingerprint':load(self.tool/'build_fingerprint.json'),'binary_sha256':dataset.digest(self.tool/'general_replay'),'overrides':overrides,'status':'queued','mode':mode,'kind':'Full' if mode=='full' else 'Dataset','prefix':str(prefix),'params':str(params),'start_time':time.time(),'frame_range':[1,end],'detail_range':[start,detail_end],'track':request.get('track'),'baseline':request.get('baseline')}
  if mode=='what-if':
   run=self.run(rid,key,request['run']);snapshot=Path(run['prefix']+f'_frame_{start}.snapshot')
   if not snapshot.exists():raise ValueError('No frozen detection snapshot for this frame. Run Detailed Replay first.')
   meta['snapshot']=str(snapshot);meta['frame_range']=[start,start];meta['replay_history']='Frozen detection snapshot only; no sequence replay'
   if run.get('experiment'):
    # Preserve the base run parameters, changing only the supported gate.
    original=yaml.safe_load(Path(run['experiment']['params']).read_text());next(iter(original.values()))['ros__parameters']['vehicle_min_score']=overrides.get('vehicle_min_score',self.params['vehicle_min_score']);params.write_text(yaml.safe_dump(original,sort_keys=False))
  with self.lock:
   if any(x['status'] in ('queued','running') for x in self.jobs.values()):raise ValueError('Another Lab replay is running; cancel or wait')
   self.jobs[identifier]=meta;dump(folder/'experiment.json',meta)
  threading.Thread(target=self.work,args=(identifier,),daemon=True).start();return {'id':identifier}
 def work(self,identifier):
  meta=self.jobs[identifier];folder=Path(meta['prefix']).parent;proc=None;shared=None
  try:
   with self.engine_lock:
    if meta['status']=='cancelled':return
    lock=self.roots[meta['root']]/'.command.lock'
    if lock.exists():
     shared=lock.open('r')
     try:fcntl.flock(shared,fcntl.LOCK_SH|fcntl.LOCK_NB)
     except BlockingIOError:raise RuntimeError('Dataset preparation/CLI is active in this root; Lab replay was not started')
     shared.close();shared=None # Probe only; do not hold up a later dataset preparation.
    with self.lock:
     if meta['status']=='cancelled':return
     meta['status']='running';meta['pid']=None;dump(folder/'experiment.json',meta)
    build_start=time.perf_counter()
    with (folder/'build.log').open('w') as log:
     with self.lock:
      if meta['status']=='cancelled':return
      builder=subprocess.Popen([sys.executable,str(HERE/'build_debug.py'),'--output',str(self.tool)],env=self.environment,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
      self.children[identifier]=builder;meta['pid']=builder.pid
     build_code=builder.wait()
    if meta['status']=='cancelled':return
    if build_code:raise RuntimeError('Private debug build failed: '+(folder/'build.log').read_text()[-2000:])
    meta['build_sec']=time.perf_counter()-build_start;meta['build_fingerprint']=load(self.tool/'build_fingerprint.json');meta['binary_sha256']=dataset.digest(self.tool/'general_replay');meta['source_sha256']=dataset.digest(HERE.parent.parent/'src/cx128s2_bev_perception_node.cpp')
    if meta['status']=='cancelled':return
    if meta['mode']=='what-if':cmd=self.command(meta['root'],meta['dataset'],'what-if',Path(meta['prefix']),Path(meta['params']));cmd[5]=meta['snapshot']
    else:cmd=self.command(meta['root'],meta['dataset'],'full' if meta['mode']=='full' else 'cache-replay',Path(meta['prefix']),Path(meta['params']),stop=0 if meta['mode']=='full' else meta['frame_range'][1])
    env=self.environment.copy();env.update(GENERAL_LAB_DETAIL_START=str(meta['detail_range'][0]),GENERAL_LAB_DETAIL_END=str(meta['detail_range'][1]));meta['command']=cmd;start=time.perf_counter()
    with (folder/'console.log').open('w') as log:
     with self.lock:
      if meta['status']=='cancelled':return
      proc=subprocess.Popen(cmd,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
      self.children[identifier]=proc;meta['pid']=proc.pid
     dump(folder/'experiment.json',meta);code=proc.wait()
    meta['process_wall_sec']=time.perf_counter()-start
    if meta['status']=='cancelled':return
    if code:raise RuntimeError(f'C++ returned {code}: '+(folder/'console.log').read_text()[-2000:])
    if meta['mode']=='what-if':meta['whatif']=load(Path(meta['prefix']+'_whatif.json'))
    else:
     meta['stats']=load(Path(meta['prefix']+'_stats.json'));meta['stats']['process_wall_sec']=meta['process_wall_sec'];meta['summary']=dataset.summary(Path(meta['prefix']))[0]
    if meta.get('baseline') and meta['mode']!='what-if':
     try:meta['baseline_comparison']=self.baseline_summary(meta)
     except (ValueError,KeyError,OSError) as e:meta['baseline_comparison']={'status':'Not Available','reason':str(e)}
    with self.lock:
     if meta['status']!='cancelled':meta['status']='completed';meta['finished']=time.time()
  except BaseException as e:
   if meta['status']!='cancelled':meta.update(status='failed',error=str(e),finished=time.time())
  finally:
   if shared:shared.close()
   with self.lock:self.children.pop(identifier,None)
   dump(folder/'experiment.json',safe(meta))
 def baseline_summary(self,meta):
  baseline=self.run(meta['root'],meta['dataset'],meta['baseline']);a,apairs=dataset.summary(Path(baseline['prefix']));b,bpairs=dataset.summary(Path(meta['prefix']))
  # Physical identity cannot be recovered by equating numeric IDs across runs.
  end=min(meta['frame_range'][1],int((baseline.get('stats') or {}).get('frames',meta['frame_range'][1])));arows=dataset.tracks(Path(baseline['prefix']));brows=dataset.tracks(Path(meta['prefix']))
  def by_frame(rows):
   result={}
   for row in rows:
    frame=int(row['frame'])
    if frame<=end:result.setdefault(frame,[]).append(row)
   return result
  ar=by_frame(arows);br=by_frame(brows)
  def actual_frames(rows):return {f for f,values in rows.items() if any(t['vehicle']=='1' and t['lost']=='0' for t in values)}
  af=actual_frames(ar);bf=actual_frames(br)
  return {'status':'Compared','baseline':meta['baseline'],'frame_range':[1,end],'baseline_summary_full_run':a,'candidate_summary':b,'actual_vehicle_frames_lost':sorted(af-bf),'actual_vehicle_frames_gained':sorted(bf-af),'physical_identity':'Frame-level public observations only; numeric track IDs do not establish physical correspondence','baseline_stats':baseline.get('stats'),'candidate_stats':meta.get('stats')}
 def cancel(self,identifier):
  with self.lock:
   if identifier not in self.jobs:raise ValueError('Unknown job')
   j=self.jobs[identifier]
   if j['status'] not in ('running','queued'):return j
   j['status']='cancelled';p=self.children.get(identifier)
   if p and p.poll() is None:os.killpg(p.pid,signal.SIGTERM)
   dump(Path(j['prefix']).parent/'experiment.json',j);return j
 def jobs_view(self):
  result=[]
  with self.lock:jobs=list(self.jobs.values())
  for x in jobs:
   value={k:v for k,v in x.items() if k not in ('build_fingerprint','command')};trace=Path(x['prefix']+'_trace.jsonl');progress=0
   if trace.exists():
    with trace.open('rb') as f:
     f.seek(max(0,trace.stat().st_size-65536));lines=f.read().splitlines()
    for line in reversed(lines):
     try:progress=json.loads(line)['frame'];break
     except (ValueError,KeyError):pass
   value['progress_frame']=(1 if x['status']=='completed' else 0) if x['mode']=='what-if' else progress;value['progress_total']=1 if x['mode']=='what-if' else x['frame_range'][1];result.append(value)
  return result
 def compare(self,rid,key,left,right,frame):
  a=self.state_at(rid,key,left,frame);b=self.state_at(rid,key,right,frame);ar=self.run(rid,key,left);br=self.run(rid,key,right);rows=self.rows(rid,key);ev1=self.line(Path(ar['prefix']+'_evidence.jsonl'),frame);ev2=self.line(Path(br['prefix']+'_evidence.jsonl'),frame)
  same_version=ar.get('source_version')==br.get('source_version') and ar.get('source_version') not in ('Not recorded',None)
  if ar.get('experiment') and br.get('experiment'):same_version=ar['experiment']['binary_sha256']==br['experiment']['binary_sha256'] and ar['experiment']['overrides']==br['experiment']['overrides']
  # Legacy verification runs have an independently persisted same-binary proof.
  verified_pair=Path(ar['prefix']).parent==Path(br['prefix']).parent and Path(ar['prefix']).parent.name=='verification'
  proof=self.ready(rid,key).get('verified');verified_pair=bool(verified_pair and proof)
  equivalence=ar['kind']!=br['kind'] and (same_version or verified_pair)
  details={};ta={int(x['id']):x for x in a['tracks']};tb={int(x['id']):x for x in b['tracks']}
  for ident in sorted(ta.keys()|tb.keys()):
   x=ta.get(ident);y=tb.get(ident);details[str(ident)]={'before':x,'after':y,'changed':[k for k in (x or y) if not x or not y or x.get(k)!=y.get(k)]}
  alignment={'frame':frame,'timestamp_ns':rows[frame-1]['stamp_ns'],'basis':'same immutable dataset sequence; saved trace timestamp checked when present'}
  for state in (a,b):
   if state['trace'] and state['trace'].get('stamp_ns')!=alignment['timestamp_ns']:raise ValueError('Run timestamp does not match dataset index')
  stage_differences=[]
  if ev1 and ev2:
   for stage in sorted(set(ev1.get('stages',{}))|set(ev2.get('stages',{}))):
    if ev1.get('stages',{}).get(stage)!=ev2.get('stages',{}).get(stage):stage_differences.append(stage)
  return {'left':a,'right':b,'differences':details,'stage_differences':stage_differences,'base_overlay_notice':'Both views show stored dataset points/clusters; Full track geometry comes from its trace. Changed Full prefix geometry is represented by stage digest mismatch, not reconstructed here.','evidence_equal':ev1==ev2 if ev1 is not None and ev2 is not None else None,'alignment':alignment,'classification':'Replay/Cache equivalence check' if equivalence else 'A/B (version/override may differ; equivalence not asserted)','jetson':'Not Available','physical_identity':'IDs are same-run bookkeeping; numeric ID changes alone do not prove a different vehicle'}
