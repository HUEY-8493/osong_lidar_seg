#!/usr/bin/env python3
"""Private child-process JSONL transport for the existing Lab. No sockets or Qt."""
import argparse, csv, fcntl, json, os, sys, traceback, signal, subprocess, select, threading, time, collections
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'general_debug_lab'))
from engine import Lab, dataset, safe, load, dump
import view_evidence as view

class StudioLab(Lab):
 """Existing Lab LOD/cache, with its C++ inspection function kept alive."""
 def __init__(self,*args):
  super().__init__(*args);self.reader=None;self.reader_log=None;self.reader_calls=0
  self.ram=collections.OrderedDict();self.ram_sizes={};self.ram_bytes=0;self.ram_limit=512*1024**2;self.ram_hits=0;self.ram_misses=0
  self.meta_cache={};self.run_cache={};self.trace_ram=collections.OrderedDict();self.trace_bytes=0;self.trace_limit=128*1024**2
  self.io_condition=threading.Condition();self.io_busy=False;self.foreground_waiters=0
  self.prefetch_stop=threading.Event();self.prefetch_thread=None;self.prefetch_generation=0;self.auto_prefetch=True;self.lookahead=2
  self.prefetch_progress={'status':'idle','done':0,'total':0};self.last_frame_request=None
  settings=self.state/'buffer_settings.json'
  if settings.exists():
   try:self.configure_cache(load(settings))
   except (ValueError,TypeError,OSError):pass
 def metadata(self,rid,key):
  cache=getattr(self,'meta_cache',{});item=cache.get((rid,key))
  if item and time.monotonic()-item[0]<5:return item[1]
  with self.lock:
   value=super().metadata(rid,key);cache[(rid,key)]=(time.monotonic(),value)
  return value
 def runs(self,rid,key):
  item=self.run_cache.get((rid,key))
  if item and time.monotonic()-item[0]<2:return item[1]
  with self.lock:
   value=super().runs(rid,key);self.run_cache[(rid,key)]=(time.monotonic(),value)
  return value
 def line(self,path,frame,trace=True):
  # Completed run files are immutable. Refresh on reopen / explicit reload.
  key=(str(path),trace)
  with self.lock:
   cached=self.trace_ram.get(key)
   if cached and time.monotonic()>cached[4]:
    signature=(path.stat().st_mtime_ns,path.stat().st_size) if path.exists() else None
    if signature!=cached[3]:self.trace_ram.pop(key);self.trace_bytes-=cached[2];cached=None
    else:cached=(*cached[:4],time.monotonic()+5);self.trace_ram[key]=cached
   if cached:self.trace_ram.move_to_end(key)
  if not cached:
   if not path.exists():return None
   size=path.stat().st_size
   if size>self.trace_limit//2:return super().line(path,frame,trace)
   groups={};header=None
   with path.open('rb') as stream:
    if not trace:header=next(csv.reader([stream.readline().decode()]))
    for line in stream:
     row=json.loads(line) if trace else dict(zip(header,next(csv.reader([line.decode()]))))
     groups.setdefault(int(row['frame']),[]).append(line)
   weight=sum(sys.getsizeof(line) for rows in groups.values() for line in rows)+sum(sys.getsizeof(v) for v in groups.values())+sys.getsizeof(groups)
   cached=(groups,header,weight,(path.stat().st_mtime_ns,path.stat().st_size),time.monotonic()+5)
   with self.lock:
    while self.trace_ram and self.trace_bytes+weight>self.trace_limit:
     _,old=self.trace_ram.popitem(last=False);self.trace_bytes-=old[2]
    if weight<=self.trace_limit:self.trace_ram[key]=cached;self.trace_bytes+=weight
  rows=[json.loads(line) if trace else dict(zip(cached[1],next(csv.reader([line.decode()])))) for line in cached[0].get(frame,[])]
  return (rows[0] if rows else None) if trace else rows
 @staticmethod
 def weight(value):
  seen=set()
  def size(v):
   identity=id(v)
   if identity in seen:return 0
   seen.add(identity);n=sys.getsizeof(v)
   if isinstance(v,dict):return n+sum(size(k)+size(x) for k,x in v.items())
   if isinstance(v,(list,tuple)):return n+sum(size(x) for x in v)
   return n
  return size(value)
 def trim_ram(self):
  while self.ram and self.ram_bytes>self.ram_limit:
   key,_=self.ram.popitem(last=False);self.ram_bytes-=self.ram_sizes.pop(key)
 def frame(self,rid,key,number,background=False,generation=None):
  meta=self.ready(rid,key);identity=(meta['manifest_sha256'],number)
  with self.lock:
   if identity in self.ram:self.ram.move_to_end(identity);self.ram_hits+=1;return self.ram[identity]
  with self.io_condition:
   if not background:self.foreground_waiters+=1
   try:
    while self.io_busy or (background and self.foreground_waiters):
     if background and (self.prefetch_stop.is_set() or generation!=self.prefetch_generation):return None
     self.io_condition.wait(.05)
    if background and (self.prefetch_stop.is_set() or generation!=self.prefetch_generation):return None
    self.io_busy=True
   finally:
    if not background:self.foreground_waiters-=1
  try:
   with self.lock:
    if identity in self.ram:self.ram.move_to_end(identity);self.ram_hits+=1;return self.ram[identity]
   value=self._read_frame(rid,key,number);weight=self.weight(value)
   with self.lock:
    self.ram_misses+=1;self.frames.clear()
    if weight<=self.ram_limit and (not background or not self.prefetch_stop.is_set()):
     self.ram[identity]=value;self.ram_sizes[identity]=weight;self.ram_bytes+=weight;self.trim_ram()
   return value
  finally:
   with self.io_condition:self.io_busy=False;self.io_condition.notify_all()
 def cache_status(self):
  with self.lock:return dict(self.prefetch_progress,frames=len(self.ram),bytes=self.ram_bytes,limit=self.ram_limit,trace_bytes=self.trace_bytes,trace_limit=self.trace_limit,hits=self.ram_hits,misses=self.ram_misses,auto=self.auto_prefetch,lookahead=self.lookahead)
 def cancel_prefetch(self):
  self.prefetch_stop.set()
  with self.io_condition:self.io_condition.notify_all()
  with self.lock:
   if self.prefetch_progress['status']=='loading':self.prefetch_progress['status']='stopping'
  return self.cache_status()
 def preload(self,rid,key,start,end,automatic=False):
  progress=self.prefetch_progress
  if progress.get('status')=='loading' and progress.get('root')==rid and progress.get('dataset')==key and start>=progress.get('start',0) and end<=progress.get('end',0):return self.cache_status()
  if automatic and self.prefetch_progress.get('status') in ('loading','stopping') and not self.prefetch_progress.get('automatic',False):return self.cache_status()
  self.cancel_prefetch()
  if self.prefetch_thread and self.prefetch_thread.is_alive():
   # One read may already be in flight. Replace only the queued range.
   self.prefetch_pending=(rid,key,start,end,automatic);return self.cache_status()
  total=len(self.rows(rid,key));start=max(1,int(start));end=min(total,int(end))
  if end<start:raise ValueError('Preload range is outside the dataset or start exceeds end')
  self.prefetch_pending=None;self.prefetch_stop.clear();self.prefetch_generation+=1;generation=self.prefetch_generation
  with self.lock:self.prefetch_progress={'status':'loading','done':0,'total':end-start+1,'start':start,'end':end,'root':rid,'dataset':key,'automatic':automatic,'error':''}
  def run():
   try:
    for number in range(start,end+1):
     if self.prefetch_stop.is_set():break
     while self.engine_lock.locked():
      if self.prefetch_stop.wait(.1):break
     if self.prefetch_stop.is_set():break
     value=self.frame(rid,key,number,background=True,generation=generation)
     if value is None:break
     with self.lock:self.prefetch_progress['done']+=1;self.prefetch_progress['current']=number
     self.prefetch_stop.wait(.02)
    with self.lock:self.prefetch_progress['status']='cancelled' if self.prefetch_stop.is_set() else 'complete'
   except Exception as e:
    with self.lock:self.prefetch_progress.update(status='failed',error=str(e))
  self.prefetch_thread=threading.Thread(target=run,name='studio-prefetch',daemon=True);self.prefetch_thread.start();return self.cache_status()
 def service_prefetch(self):
  pending=getattr(self,'prefetch_pending',None)
  if pending and not self.prefetch_thread.is_alive():self.prefetch_pending=None;self.preload(*pending)
 def configure_cache(self,r):
  if r.get('clear'):self.prefetch_pending=None;self.cancel_prefetch()
  with self.lock:
   self.auto_prefetch=bool(r.get('auto',self.auto_prefetch));self.lookahead=max(1,min(16,int(r.get('lookahead',self.lookahead))))
   self.ram_limit=max(32,min(2048,int(r.get('limit_mb',self.ram_limit//1024**2))))*1024**2;self.trim_ram()
   if r.get('clear'):self.ram.clear();self.ram_sizes.clear();self.ram_bytes=0;self.trace_ram.clear();self.trace_bytes=0;self.frames.clear()
  if not self.auto_prefetch:self.prefetch_pending=None;self.cancel_prefetch()
  dump(self.state/'buffer_settings.json',{'auto':self.auto_prefetch,'lookahead':self.lookahead,'limit_mb':self.ram_limit//1024**2})
  return self.cache_status()
 def refresh_view(self):
  with self.lock:self.meta_cache.clear();self.run_cache.clear();self.trace_ram.clear();self.trace_bytes=0
 def stop_loading(self):
  self.prefetch_pending=None;self.cancel_prefetch()
  if self.prefetch_thread and self.prefetch_thread.is_alive():self.prefetch_thread.join(timeout=46)
  self.close_reader()
 def _read_frame(self,rid,key,number):
  m=self.ready(rid,key);rows=self.rows(rid,key)
  if not 1<=number<=len(rows):raise ValueError('Frame outside dataset')
  cached=self.state/'frames'/m['manifest_sha256']/f'{number}.json'
  if cached.exists() or (m['manifest_sha256'],number) in self.frames:return super().frame(rid,key,number)
  if not self.engine_lock.acquire(blocking=False):raise ValueError('Replay is running; uncached frame reading is temporarily busy')
  try:
   prefix=cached.parent/(str(number)+'_raw');prefix.parent.mkdir(parents=True,exist_ok=True)
   if self.reader is None or self.reader.poll() is not None:
    executable=self.state/'studio_reader/general_replay'
    if not executable.exists():raise ValueError('Persistent reader is not built; use start_general_perception_studio.sh')
    cmd=self.command(rid,key,'studio-reader',prefix,Path(__file__).resolve().parent.parent.parent/'config/cx128s2_bev_perception.yaml')
    cmd[3]=str(executable);self.reader_log=(self.state/'studio_reader_console.log').open('a')
    self.reader=subprocess.Popen(cmd,env=self.environment,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=self.reader_log,text=True,start_new_session=True)
   contract=load(self.roots[rid]/key/'manifest.json')['contract']
   request={'frame':number,'bag':m.get('bag') or '/unavailable/original_bag','cache':str(self.roots[rid]/key),'prefix':str(prefix),'depth_cache':contract['depth_cache'],'compressed':contract['compression']!='none'}
   self.reader.stdin.write(json.dumps(request)+'\n');self.reader.stdin.flush();self.reader_calls+=1
   if not select.select([self.reader.stdout],[],[],45)[0]:
    self.close_reader();raise ValueError('Stored frame reader timed out (45 s)')
   line=self.reader.stdout.readline()
   if not line:raise ValueError('Stored frame reader exited; see studio_reader_console.log')
   response=json.loads(line)
   if not response.get('ok'):raise ValueError('Stored frame reader: '+response.get('error','Unknown error'))
   raw_path=Path(str(prefix)+'_frame.json');raw=load(raw_path);value=self.lod(raw,rows[number-1]);dump(cached,value);raw_path.unlink()
  finally:self.engine_lock.release()
  return super().frame(rid,key,number)
 def close_reader(self):
  if self.reader:
   if self.reader.poll() is None:
    self.reader.stdin.close()
    try:self.reader.wait(timeout=3)
    except subprocess.TimeoutExpired:os.killpg(self.reader.pid,signal.SIGTERM);self.reader.wait(timeout=3)
   self.reader=None
  if self.reader_log:self.reader_log.close();self.reader_log=None

def main():
 p=argparse.ArgumentParser();p.add_argument('--state',type=Path,required=True);p.add_argument('--dataset-root',type=Path,action='append',default=[]);a=p.parse_args()
 a.state.mkdir(parents=True,exist_ok=True)
 lock=(a.state/'desktop.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 roots={'local':dataset.ROOT,'z':Path('/mnt/z/osong/oosong_record/enterance_dataset')}
 for i,path in enumerate(a.dataset_root):roots[f'custom{i}']=path.resolve()
 lab=StudioLab(a.state,roots)
 graph_cache={}
 def shutdown(signum=None,frame=None):
  for job in lab.jobs_view():
   if job['status'] in ('queued','running'):lab.cancel(job['id'])
  lab.stop_loading()
  if signum is not None:raise SystemExit(0)
 signal.signal(signal.SIGTERM,shutdown)
 signal.signal(signal.SIGINT,shutdown)
 def context(r):return r['root'],r['dataset']
 def catalog():
  result=[]
  for root in lab.roots:
   for key in lab.keys(root):
    try:result.append(lab.metadata(root,key))
    except Exception as e:result.append({'root':root,'key':key,'status':'Not Prepared','error':str(e)})
  return result
 def bundle(r):
  root,key=context(r);frame=int(r['frame']);data=lab.frame(root,key,frame);states=[]
  for run in r.get('runs',[]):
   if not run:states.append({});continue
   st=lab.state_at(root,key,run,frame);st['display_tracks']=view.tracks(st)
   for t in st['display_tracks']:
    t['display_flags']=view.flags(t);t['display_position'],t['position_source']=view.position(t,data,st)
    t['camera_bounds']=[{'bounds':bounds,'reason':reason} for bounds,reason in (view.selected_bounds(data,st,int(t['id']),None,target) for target in ['Observed Cluster','Current Geometry','Reliable Geometry','Track + Prediction'])]
   meta,links=view.evidence(data,st);st['cluster_tags']={str(k):sorted(v) for k,v in meta.items()};st['verified_links']={str(k):sorted(v) for k,v in links.items()};states.append(st)
  comparison={'notice':'Select two completed results'}
  if len(r.get('runs',[]))==2 and all(r['runs']):
   try:comparison=lab.compare(root,key,*r['runs'],frame)
   except Exception as e:comparison={'notice':str(e)}
  if lab.auto_prefetch and root!='local':lab.preload(root,key,frame+1,frame+lab.lookahead,automatic=True)
  return {'data':data,'states':states,'comparison':comparison,'unsupported_filters':view.UNSUPPORTED,'buffer':lab.cache_status()}
 def graph(r):
  root,key=context(r);run=lab.run(root,key,r['run']);prefix=Path(run['prefix']);tid=int(r['track']);sig=(str(prefix),tid)
  if sig in graph_cache:return graph_cache[sig]
  series=view.tracking_series(lab,root,key,r['run'],tid)
  timing=[];tp=Path(str(prefix)+'_times.csv')
  if tp.exists():
   for q in csv.DictReader(tp.open()):timing.append({'frame':int(q['frame']),'pipeline_ms':float(q['pipeline_ms'])})
  result=dict(series,timing=timing) # tracking_series already supplies the recorded lifecycle lanes/events
  graph_cache[sig]=result
  if len(graph_cache)>8:graph_cache.pop(next(iter(graph_cache)))
  return result
 def dispatch(r):
  op=r['op']
  if op=='catalog':return catalog()
  if op=='buffer-status':lab.service_prefetch();return lab.cache_status()
  if op=='buffer-config':return lab.configure_cache(r)
  if op=='buffer-start':return lab.preload(*context(r),r['start'],r['end'])
  if op=='buffer-cancel':lab.prefetch_pending=None;return lab.cancel_prefetch()
  if op=='reader-info':return {'pid':lab.reader.pid if lab.reader else None,'requests':lab.reader_calls}
  if op=='open':
   lab.prefetch_pending=None;lab.cancel_prefetch();lab.refresh_view();root,key=context(r);return {'metadata':lab.ready(root,key),'rows':lab.rows(root,key),'runs':lab.runs(root,key),'parameters':lab.parameters(root,key)}
  if op=='frame':return bundle(r)
  if op=='events':return lab.events(*context(r),r['run'])
  if op=='graph':return graph(r)
  if op=='jobs':return lab.jobs_view()
  if op=='start':lab.prefetch_pending=None;lab.cancel_prefetch();return lab.start(r['request'])
  if op=='cancel':return lab.cancel(r['job'])
  if op=='restore':
   j=lab.jobs[r['job']];return {k:v for k,v in j.items() if k!='build_fingerprint'}
  if op=='quit':
   for job in lab.jobs_view():
    if job['status'] in ('queued','running'):lab.cancel(job['id'])
   lab.stop_loading();return {'quit':True}
  raise ValueError('Unsupported operation: '+str(op))
 for line in sys.stdin:
  request={}
  try:
   request=json.loads(line);result=dispatch(request);response={'id':request['id'],'ok':True,'result':safe(result)}
  except Exception as e:
   traceback.print_exc(file=sys.stderr);response={'id':request.get('id',0),'ok':False,'error':str(e)}
  sys.stdout.write(json.dumps(response,allow_nan=False,separators=(',',':'))+'\n');sys.stdout.flush()
  if request.get('op')=='quit':break
 else:
  lab.stop_loading()
  for job in lab.jobs_view():
   if job['status'] in ('queued','running'):lab.cancel(job['id'])
if __name__=='__main__':main()
