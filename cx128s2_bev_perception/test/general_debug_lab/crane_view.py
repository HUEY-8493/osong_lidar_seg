"""Read-only Crane adapter and Qt Explore tab. No detector or Python tracking.
Raw sensor bytes + recorded TF, timestamp-joined current C++ output only.
"""
from pathlib import Path
import json, bisect, time, hashlib
from collections import OrderedDict
import numpy as np
from PyQt5 import QtCore as C, QtWidgets as W, QtGui as G
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg, NavigationToolbar2QT
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import ui_style as ui
ROOT=Path(__file__).resolve().parents[4]
DEFAULT=ROOT/'log/crane_perception_evaluation_20261025'
BOOKMARKS=[(500,'Two DIRECT bodies'),(1525,'Measurement / tracker difference'),(2500,'RIGHT HELD / rejects'),(3000,'DIRECT at another height'),(3500,'XY motion before'),(3510,'XY motion after'),(4220,'LEFT before spike'),(4225,'Unconfirmed LEFT'),(4230,'LEFT after spike'),(4398,'Working begins'),(4450,'Cargo / long chain'),(4503,'Cargo 103'),(4504,'Cargo 104 / idle')]
LAYERS=['Raw PointCloud','Z Gaussian Layer','BEV Cluster','Root / Wire / Chain','LEFT Housing','RIGHT Housing','Observation Center','Tracked Center','Virtual Hook','Cargo Seed','Cargo Extended Geometry','Observed Spans','Inferred Gaps','Cargo ROI','Gap ROI','Moving Boundary Patch','Rejected Candidates','Unknown / Unconfirmed Objects','Reference Candidates','Previous Housing Center','Wire Support / Association']
STATUS={0:'NONE / visibility unproven',1:'DIRECT',3:'HELD'}
GATE_NAMES=['Surface size / points / beam exclusion','Relative wire distance / height','Projected plane span','Housing Raw point count','Covariance spectrum','Housing dimensions','CURRENT_WIRE_SUPPORT','Reference / family / trust','MEMORY_POSITION','Memory dimensions / shape / wire offset','DUAL_ROI','No current chain']
GREEN='#31ad66';YELLOW='#e5b43e';RED='#df5c57';GRAY='#919d9a';PURPLE='#b28ce8'
class CraneData:
    def __init__(self,folder):
        self.folder=Path(folder).resolve();self.index=json.loads((self.folder/'raw_index.json').read_text());self.stamps=[r['stamp_ns'] for r in self.index];self.times=[(s-self.stamps[0])/1e9 for s in self.stamps]
        self.raw_path=ROOT/'log/crane_oct24_diagnosis_20261024/raw.bin'
        tf_path=ROOT/'log/crane_ros_rawtf_20261025/A/monitor.jsonl'
        transform=next(json.loads(l)['transforms'][0] for l in tf_path.open() if '"tf_static"' in l)
        assert transform['parent']=='world' and transform['child']=='laser_link'
        x,y,z,w=transform['rotation'];self.rotation=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
        self.translation=np.array(transform['translation']);self.runs={};self.paths={};self.offsets={};self.trace_paths={};self.trace_offsets={};self.run_config_paths={};self.cluster_offsets={};self.cache=OrderedDict();self.raw_cache=OrderedDict()
        self.add_run('A — original',ROOT/'log/crane_ros_rawtf_20261025/eval/frames.jsonl')
        candidate=ROOT/'log/crane_qt_motion_20261025/B_complete/frames.jsonl'
        if candidate.exists() and (candidate.parent/'final.jsonl').exists():self.add_run('B — temporal',candidate)
        registry=self.folder/'housing_runs.json'
        if registry.exists():
            for item in json.loads(registry.read_text()):
                self.add_run(item['name'],Path(item['path']))
                self.trace_paths[item['name']]=Path(item['trace'])
                self.run_config_paths[item['name']]=[Path(v) for v in item['configs']]
            stamps=[v/1e9 for v in self.stamps]
            for name,path in self.trace_paths.items():
                offsets={}
                with path.open('rb') as f:
                    while True:
                        offset=f.tell();line=f.readline()
                        if not line:break
                        row=json.loads(line);i=min(len(stamps)-1,bisect.bisect_left(stamps,row['stamp']))
                        if i and abs(stamps[i-1]-row['stamp'])<abs(stamps[i]-row['stamp']):i-=1
                        if abs(stamps[i]-row['stamp'])>1e-5:raise ValueError('Housing trace timestamp mismatch')
                        offsets[(i,row['side'])]=offset
                self.trace_offsets[name]=offsets
        if not self.raw_path.exists():raise ValueError('Prepared Raw byte cache missing: '+str(self.raw_path))
        self.cluster_path=ROOT/'log/crane_qt_motion_20261025/B_complete/clusters.jsonl'
        if self.cluster_path.exists():
            with self.cluster_path.open('rb') as f:
                for row in self.index:
                    offset=f.tell();x=json.loads(f.readline())
                    if x['stamp_ns']!=row['stamp_ns']:raise ValueError('Cluster timestamp mismatch')
                    self.cluster_offsets[row['stamp_ns']]=offset
        base=ROOT/'log/crane_ros_rawtf_20261025/audit_install/cx128s2_bev_perception/share/cx128s2_bev_perception/config'
        self.config_artifacts={}
        for name in self.runs:
            paths=self.run_config_paths.get(name) or [base/'cx128s2_bev_perception_crane.yaml',ROOT/'log/crane_qt_motion_20261025/motion_experimental.yaml' if name.startswith('B') else base/'crane_boundary_clustering_experimental.yaml']
            self.config_artifacts[name]=[{'path':str(path),'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'yaml':path.read_text()} for path in paths]
        self.provenance={'raw':str(self.raw_path),'tf':transform,'index':str(self.folder/'raw_index.json'),'source_bag':'/mnt/z/osong/oosong_record/crane/rosbag2_2026_10_24-06_03_54','frame_base':0,'frame_id':'world','notice':'C++ outputs; raw display LOD only. NONE is not ground-truth miss.'}
    def add_run(self,name,path):
        offsets=[];summary=[]
        with Path(path).open('rb') as f:
            for row in self.index:
                offset=f.tell();line=f.readline()
                if not line:raise ValueError('Run incomplete: '+str(path))
                x=json.loads(line)
                if x['stamp_ns']!=row['stamp_ns']:raise ValueError('Sensor timestamp alignment mismatch')
                offsets.append(offset);summary.append({'moving':x['moving'],'raw':x.get('motion_temporal',{}).get('raw'),'supported':x.get('motion_temporal',{}).get('supported'),'left':x['housing'][0]['status'],'right':x['housing'][1]['status'],'cargo':x['cargo']['id'],'work':x['work'],'gap':x['gap']['state'],'lid':x['housing'][0]['track_id'],'rid':x['housing'][1]['track_id'],'lc':x['housing'][0]['confirmed'],'rc':x['housing'][1]['confirmed']})
            if f.readline():raise ValueError('Unexpected extra result frames')
        self.offsets[name]=offsets;self.runs[name]=summary;self.paths[name]=Path(path)
    def result(self,name,i):
        key=(name,i)
        if key not in self.cache:
            with self.paths[name].open('rb') as f:f.seek(self.offsets[name][i]);self.cache[key]=json.loads(f.readline())
            if name in self.trace_offsets:
                trace=[]
                with self.trace_paths[name].open('rb') as f:
                    for side in range(2):
                        f.seek(self.trace_offsets[name][(i,side)]);trace.append(json.loads(f.readline()))
                self.cache[key]['housing_trace']=trace
            if self.cluster_offsets:
                with self.cluster_path.open('rb') as f:
                    f.seek(self.cluster_offsets[self.stamps[i]]);self.cache[key]['subclusters_detail']=json.loads(f.readline())['subclusters_detail']
                self.cache[key]['subclusters_source']='separate C++ current-node replay; timestamp aligned; geometry parity checked in ab_summary'
            while len(self.cache)>16:self.cache.popitem(last=False)
        return self.cache[key]
    def raw(self,i):
        if i not in self.raw_cache:
            row=self.index[i]
            with self.raw_path.open('rb') as f:f.seek(row['offset']);b=f.read(row['points']*32)
            if len(b)!=row['points']*32:raise ValueError('Truncated prepared Raw cache')
            xyz=np.ndarray((row['points'],3),'<f4',buffer=b,strides=(32,4));valid=np.isfinite(xyz).all(axis=1)&(np.linalg.norm(xyz,axis=1)>=1e-6);xyz=xyz[valid].astype(float)@self.rotation.T+self.translation
            self.raw_cache[i]=xyz.astype(np.float32)
            while len(self.raw_cache)>3:self.raw_cache.popitem(last=False)
        return self.raw_cache[i]
    def frame(self,i,names):return {'frame':i,'points':self.raw(i),'results':[self.result(n,i) for n in names],'names':names,'stamp_ns':self.stamps[i],'sec':self.times[i]}

def cargo_box(c):
    p=np.array(c['xyz']);l,w,h,yaw=c['lwh_yaw'];corners=np.array([[x*l/2,y*w/2,z*h/2] for x in [-1,1] for y in [-1,1] for z in [-1,1]])
    rot=np.array([[np.cos(yaw),-np.sin(yaw),0],[np.sin(yaw),np.cos(yaw),0],[0,0,1]]);return corners@rot.T+p

def box_corners(b):return np.array([[b[0+ix],b[2+iy],b[4+iz]] for ix in [0,1] for iy in [0,1] for iz in [0,1]])
EDGES=[(i,j) for i in range(8) for j in range(i+1,8) if bin(i^j).count('1')==1]
def objects(r):
    result={}
    for s in range(2):
        h=r['housing'][s];key=['LEFT Housing','RIGHT Housing'][s]
        if h['status'] in [1,3]:result[key]={'corners':box_corners(h['bbox']),'center':h['xyz'],'data':h,'color':GREEN if h['status']==1 and h['confirmed'] else YELLOW,'kind':'housing','side':s}
        if r['hooks'][s]['valid']:
            k=['LEFT Virtual Hook','RIGHT Virtual Hook'][s];v=r['hooks'][s];p=np.array(v['xyz']);result[k]={'corners':box_corners([p[0]-.1,p[0]+.1,p[1]-.1,p[1]+.1,p[2]-.1,p[2]+.1]),'center':v['xyz'],'data':v,'color':PURPLE,'kind':'hook','side':s}
        if h.get('rejected_distance',-1)>=0:
            p=np.array(h['rejected_center']);k=['LEFT Rejected Center','RIGHT Rejected Center'][s];result[k]={'center':p,'corners':np.array([p-.2,p+.2]),'data':{'reason':h['closest_rejection'],'center':h['rejected_center'],'distance':h['rejected_distance'],'notice':'Closest rejected center only. No saved candidate point membership/bounds.'},'color':RED,'kind':'reject','side':s}
    for side,h in enumerate(r['housing']):
        if h.get('temporal_pending'):
            name=['LEFT','RIGHT'][side]+' Pending Housing'
            result[name]={'corners':box_corners(h['bbox']),'center':h['observed'],'data':h,'color':YELLOW,'kind':'pending','side':side}
    for side,trace in enumerate(r.get('housing_trace',[])):
        candidates=[];seen=set();prior=trace['previous']['memory_xyz']
        for candidate in trace['records']:
            stage=candidate['stage'];key=(stage,tuple(candidate['bbox']))
            if key in seen or stage.startswith('REF_') or stage=='RANK':continue
            seen.add(key)
            if candidate['points']<3:continue
            candidates.append(candidate)
        candidates.sort(key=lambda x:float(np.linalg.norm(np.array(x['xyz'])-prior)))
        for i,candidate in enumerate(candidates[:4]):
            accepted=candidate['stage']=='ACCEPTED_PROPOSAL';name=['LEFT','RIGHT'][side]+(' Reference Candidate ' if accepted else ' Rejected Geometry ')+str(i)
            result[name]={'corners':box_corners(candidate['bbox']),'center':candidate['xyz'],'data':candidate,'color':YELLOW if accepted else RED,'kind':'proposal' if accepted else 'rejected_geometry','side':side}
    c=r['cargo']
    if c['id']>=0:result['Cargo Seed']={'corners':cargo_box(c),'center':c['xyz'],'data':c,'color':GREEN if c['locked'] and c['observation']=='DIRECT' else YELLOW,'kind':'cargo'}
    e=r['extension']
    if e['valid']:result['Cargo Extended Geometry']={'corners':cargo_box(e),'center':e['xyz'],'data':e,'color':GREEN if e['observation']=='DIRECT' else YELLOW,'kind':'extension'}
    return result

class CraneCanvas(FigureCanvasQTAgg):
    selected=C.pyqtSignal(str,int)
    def __init__(self,parent=None):
        self.figure=Figure(facecolor='#17221f');super().__init__(self.figure);self.setParent(parent);self.axes=[];self.artists={};self.focus=None;self.last=None
        self.mpl_connect('pick_event',self.pick)
    def pick(self,e):
        if e.artist in self.artists:self.selected.emit(*self.artists[e.artist])
    def render(self,value,projection,layers,selected=None,fit=False):
        old=[]
        for a in self.axes:
            old.append((a.get_xlim(),a.get_ylim(),a.get_zlim() if hasattr(a,'get_zlim') else None,getattr(a,'elev',None),getattr(a,'azim',None)))
        self.figure.clear();self.axes=[];self.artists={};self.last=value;self.focus=selected
        dims={'XY':(0,1),'XZ':(0,2),'YZ':(1,2)};dim=dims.get(projection);count=len(value['results'])
        for side,r in enumerate(value['results']):
            a=self.figure.add_subplot(1,count,side+1,projection='3d' if projection=='3D' else None);self.axes.append(a);a.set_facecolor('#17221f');a.tick_params(colors='#b5c6bc',labelsize=8)
            for spine in a.spines.values():spine.set_color('#42544a')
            def line(p,color,lw=1,ls='-',label=None,picker=False):
                p=np.asarray(p)
                if not len(p):return
                ar=a.plot(p[:,0],p[:,1],p[:,2],color=color,lw=lw,ls=ls,label=label,picker=5)[0] if dim is None else a.plot(p[:,dim[0]],p[:,dim[1]],color=color,lw=lw,ls=ls,label=label,picker=5)[0]
                return ar
            def dot(p,color,marker='o',size=24):
                p=np.asarray(p).reshape(-1,3)
                return a.scatter(p[:,0],p[:,1],p[:,2],c=color,s=size,marker=marker,picker=True) if dim is None else a.scatter(p[:,dim[0]],p[:,dim[1]],c=color,s=size,marker=marker,picker=True)
            xyz=value['points'];xyz=xyz[(abs(xyz[:,0])<9)&(abs(xyz[:,1])<5)&(xyz[:,2]>-.3)&(xyz[:,2]<7.5)]
            if layers['Raw PointCloud']:
                # Rendering LOD only. No alteration of C++ input or association.
                p=xyz
                focus=objects(r).get(selected)
                if focus:
                    bounds=np.asarray(focus['corners']);lo=bounds.min(axis=0)-.65;hi=bounds.max(axis=0)+.65
                    p=p[np.all((p>=lo)&(p<=hi),axis=1)]
                p=p[::max(1,len(p)//24000)];dot(p,'#80938a','.',.8)
            objs=objects(r)
            for name,o in objs.items():
                kind=o['kind'];enabled=(layers.get(name,True) if kind in ['housing','cargo','extension'] else layers['Virtual Hook'] if kind=='hook' else layers['Unknown / Unconfirmed Objects'] if kind=='pending' else layers['Reference Candidates'] if kind=='proposal' else layers['Rejected Candidates'])
                if kind=='housing' and (r['housing'][o['side']]['status']==3 or not r['housing'][o['side']]['confirmed']) and not layers['Unknown / Unconfirmed Objects']:enabled=False
                if not enabled:continue
                color=o['color'];p=o['corners'];highlight=name==selected
                if kind!='reject':
                    for i,j in EDGES:line([p[i],p[j]],color,2.4 if highlight else 1)
                if kind=='reject' and highlight:
                    nearby=xyz[np.linalg.norm(xyz-np.array(o['center']),axis=1)<.25]
                    if len(nearby):dot(nearby,RED,'.',3)
                ar=dot([o['center']],color,'x' if kind=='reject' else '+' if kind=='hook' else 'o',65 if highlight else 30);self.artists[ar]=(name,side)
                if dim is not None:a.annotate((name.replace(' Housing','').replace(' Virtual Hook',' Hook')+(f" #{o['data']['track_id']} {STATUS.get(o['data']['status'],'?')}" if kind=='housing' else '')),np.array(o['center'])[list(dim)],color=color,fontsize=8)
                if kind in ['housing','pending']:
                    h=o['data']
                    if layers['Observation Center'] and h['status']==1:dot([h['observed']],'#ffdf91','o',18)
                    if layers['Tracked Center']:dot([h['xyz']],'#ffffff','x',25)
            for housing_side,trace in enumerate(r.get('housing_trace',[])):
                h=r['housing'][housing_side];prev=trace['previous'];current=h['xyz']
                if layers['Previous Housing Center'] and prev['initialized']:
                    p=prev['memory_xyz'] if prev['dormant'] else prev['xyz']
                    dot([p],YELLOW,'s',30)
                    if h['status'] in [1,3] and prev['id']==h['track_id']:line([p,current],YELLOW,.9,'--')
                if layers['Wire Support / Association'] and h['status'] in [1,3] and dim is not None:
                    a.annotate(f"wire={trace['wire']} mode={trace['wire_mode']}",np.array(current)[list(dim)],xytext=(0,-14),textcoords='offset points',color=GREEN if trace['wire'] else YELLOW,fontsize=7)
            if layers['Root / Wire / Chain']:
                for c in r['chains_detail']:
                    ns=c['nodes'];dot([c['root']],YELLOW if not c['root_observed'] else GREEN,'*',35)
                    for prev,n in zip(ns,ns[1:]):line([prev['xyz'],n['xyz']],YELLOW if n['held'] or n['estimated_gap'] else '#668fd9',1.1,'--' if n['held'] else '-')
            if layers['Cargo ROI']:
                for b in r['cargo_rois']:
                    if b[1]>b[0]:
                        p=box_corners(b)
                        for i,j in EDGES:line([p[i],p[j]],PURPLE,.8,':')
            if layers['Moving Boundary Patch']:
                for p in r['boundary_patches']:
                    x,y,z=p['xyz'];sx,sy=p['search'];dot([[sx,sy,0]],YELLOW,'s',20)
                    if p['observation']!='NONE':dot([[x,y,z]],GREEN if p['points'] else YELLOW,'o',30);line([[x,y,z],[x+10*p['delta'][0],y+10*p['delta'][1],z]],RED,2)
            if layers['Z Gaussian Layer']:
                for l in r.get('z_layers',[]):line([[-2,0,l['mean']],[2,0,l['mean']]],'#55726a',.5,':')
            if layers['BEV Cluster']:
                for c in r.get('subclusters_detail',[]):
                    p=box_corners(c['bbox'])
                    for i,j in EDGES:line([p[i],p[j]],'#4b6c65',.4)
            if layers['Observed Spans'] or layers['Inferred Gaps']:
                e=r['extension'];cx,cy,cz=e['xyz'];yaw=e['lwh_yaw'][3]
                for k,label,col in [('intervals','Observed Spans',GREEN),('gaps','Inferred Gaps',YELLOW)]:
                    if layers[label]:
                        for lo,hi in e[k]:line([[cx+lo*np.cos(yaw),cy+lo*np.sin(yaw),cz],[cx+hi*np.cos(yaw),cy+hi*np.sin(yaw),cz]],col,4)
            if dim is None:a.set_xlabel('world X (m)',color='#b5c6bc');a.set_ylabel('Y (m)',color='#b5c6bc');a.set_zlabel('Z (m)',color='#b5c6bc');a.set_xlim(-3,3);a.set_ylim(-2.5,2.5);a.set_zlim(-.3,6.5);a.view_init(20,-65)
            else:a.set_xlabel('world '+['X','Y','Z'][dim[0]]+' (m)',color='#b5c6bc');a.set_ylabel('world '+['X','Y','Z'][dim[1]]+' (m)',color='#b5c6bc');a.set_xlim(-3,3);a.set_ylim((-.3,6.5) if dim[1]==2 else (-2.5,2.5));a.set_aspect('equal',adjustable='box');a.grid(alpha=.12)
            if fit and selected in objs:
                p=np.array(objs[selected]['corners']);lo=p.min(axis=0)-.3;hi=p.max(axis=0)+.3
                if dim is None:a.set_xlim(lo[0],hi[0]);a.set_ylim(lo[1],hi[1]);a.set_zlim(lo[2],hi[2])
                else:a.set_xlim(lo[dim[0]],hi[dim[0]]);a.set_ylim(lo[dim[1]],hi[dim[1]])
            elif not fit and side<len(old) and self._projection==projection:
                lim=old[side];a.set_xlim(lim[0]);a.set_ylim(lim[1])
                if dim is None and lim[2] is not None:a.set_zlim(lim[2]);a.view_init(lim[3],lim[4])
            a.set_title(f"{value['names'][side]} | f{value['frame']} / {value['sec']:.3f}s\nmove={r['moving']} work={r['work']} {r['fsm']} cargo={r['cargo']['id']}",color='#d8e4dc',fontsize=9)
            diagnostic=[]
            for hs,h in enumerate(r['housing']):
                diagnostic.append(f"{'L' if hs==0 else 'R'} {STATUS[h['status']]} id={h['track_id']} raw={h['points']}")
                diagnostic.append(f"ref={h.get('reference','?')} score={h.get('score',-1):.3f}")
                if h['status']!=1:diagnostic.append(str(h.get('closest_rejection',''))[:100])
            text='\n'.join(diagnostic)
            if dim is None:a.text2D(.01,.99,text,transform=a.transAxes,color='#e7d5b9',fontsize=6,va='top')
            else:a.text(.01,.99,text,transform=a.transAxes,color='#e7d5b9',fontsize=6,va='top',bbox=dict(facecolor='#17221f',alpha=.85,edgecolor='none'))
        self._projection=projection;self.figure.subplots_adjust(left=.06,right=.96,bottom=.09,top=.9,wspace=.26);self.draw()

class MotionTimeline(FigureCanvasQTAgg):
    seek=C.pyqtSignal(int)
    def __init__(self):
        self.figure=Figure(figsize=(10,2),facecolor='#17221f');super().__init__(self.figure);self.setMinimumHeight(225);self.setMaximumHeight(270);self.data=None;self.mpl_connect('button_press_event',self.click)
    def click(self,e):
        if self.data is not None and e.xdata is not None:self.seek.emit(min(len(self.data.times)-1,bisect.bisect_left(self.data.times,e.xdata)))
    def render(self,data,names,frame,housing_ids=False):
        self.data=data;self.figure.clear();self.cursors=[];labels=['Housing L / R','Raw / A / B','Cargo ID','Work / gap UNKNOWN'];axes=self.figure.subplots(4,1,sharex=True)
        colors=[GREEN,PURPLE,YELLOW]
        for side,name in enumerate(names):
            rows=data.runs[name];ts=data.times
            for s,k in enumerate(['left','right']):axes[0].step(ts,[{0:0,3:1,1:2}[r[k]]+s*3 for r in rows],color=colors[side],ls='-' if s==0 else ':',lw=.65,where='post')
            raw=[np.nan if r['raw'] is None else int(r['raw']) for r in rows]
            if side==len(names)-1:axes[1].step(ts,raw,color=YELLOW,lw=.6,where='post',label='raw evidence')
            axes[1].step(ts,[int(r['moving'])+side*1.2 for r in rows],color=colors[side],lw=.8,where='post',label=name)
            if housing_ids:
                axes[2].plot(ts,[r['lid'] if r['left'] in [1,3] else np.nan for r in rows],color=colors[side],lw=.7)
                axes[2].plot(ts,[r['rid']-1000000 if r['right'] in [1,3] else np.nan for r in rows],color=colors[side],lw=.7,ls='--')
            else:axes[2].plot(ts,[r['cargo'] if r['cargo']>=0 else np.nan for r in rows],color=colors[side],lw=.6)
            axes[3].step(ts,[r['work']+side*.15 for r in rows],color=colors[side],lw=.6,where='post')
        if housing_ids:labels[2]='Housing ID L / R*'
        for a,label in zip(axes,labels):
            a.set_facecolor('#17221f');a.tick_params(colors='#b5c6bc',labelsize=7);a.set_ylabel(label,color='#b5c6bc',fontsize=8,rotation=0,ha='right',va='center',labelpad=9);self.cursors.append(a.axvline(data.times[frame],color='#e7d5b9',lw=.8));a.grid(alpha=.1)
        axes[1].legend(fontsize=6,loc='upper left',ncol=3);axes[3].set_xlabel('Raw-relative seconds; click any row to seek. R* ID displayed minus 1,000,000. Gap UNKNOWN.',color='#b5c6bc',fontsize=7);self.figure.subplots_adjust(left=.13,right=.99,top=.98,bottom=.16,hspace=.25);self.draw()

    def cursor(self,frame,span=0):
        for line in self.cursors:
            line.set_xdata([self.data.times[frame]]*2)
            line.axes.set_xlim(max(0,self.data.times[frame]-span),min(self.data.times[-1],self.data.times[frame]+span)) if span else line.axes.set_xlim(0,self.data.times[-1])
        self.draw_idle()

class CranePane(W.QWidget):
    change_mode=C.pyqtSignal(str)
    def __init__(self,worker,parent=None):
        super().__init__(parent);self.Worker=worker;self.workers=set();self.data=None;self.value=None;self.busy=False;self.pending=None;self.frame_load_ms=[];self.closing=False;self.selected=None;self.selected_side=0;self._timeline_key=None
        layout=W.QVBoxLayout(self);layout.setContentsMargins(20,14,20,10);layout.setSpacing(8)
        bar=W.QHBoxLayout();bar.addWidget(ui.label('PERCEPTION LAB','eyebrow'));self.mode=ui.ComboBox();self.mode.addItems(['General','Crane']);self.mode.setCurrentText('Crane');self.mode.currentTextChanged.connect(self.change_mode);bar.addWidget(self.mode)
        self.dataset_label=ui.label('Oct24 Raw + recorded TF','subtitle');bar.addWidget(self.dataset_label,1);load=ui.button('Load Crane dataset',callback=self.choose_dataset,role='small');bar.addWidget(load);layout.addLayout(bar)
        controls=W.QHBoxLayout();self.a=ui.ComboBox();self.b=ui.ComboBox();self.compare=W.QCheckBox('A/B Compare');self.c=ui.ComboBox();self.triple=W.QCheckBox('+ E');controls.addWidget(self.a);controls.addWidget(self.compare);controls.addWidget(self.b);controls.addWidget(self.triple);controls.addWidget(self.c);self.projection=ui.ComboBox();self.projection.addItems(['XZ','XY','YZ','3D']);controls.addWidget(self.projection);self.timeline_window=ui.ComboBox();self.timeline_window.addItems(['Full timeline','±5 s','±15 s','±60 s']);self.timeline_window.currentIndexChanged.connect(lambda *_:self.redraw());controls.addWidget(self.timeline_window);self.id_metric=ui.ComboBox();self.id_metric.addItems(['Cargo ID','Housing IDs']);self.id_metric.currentIndexChanged.connect(lambda *_:self.redraw());controls.addWidget(self.id_metric)
        controls.addWidget(ui.button('Layers',callback=self.show_layers,role='small'));controls.addWidget(ui.button('Fit selected',callback=lambda:self.redraw(True),role='small'));controls.addWidget(ui.button('Fit scene',callback=self.fit_scene,role='small'));controls.addWidget(ui.button('Export',callback=self.export_menu,role='small'));layout.addLayout(controls)
        split=W.QSplitter();layout.addWidget(split,1);scene=W.QWidget();sl=W.QVBoxLayout(scene);sl.setContentsMargins(0,0,0,0);self.canvas=CraneCanvas();self.toolbar=NavigationToolbar2QT(self.canvas,self);sl.addWidget(self.toolbar);sl.addWidget(self.canvas,1);split.addWidget(scene)
        inspector=W.QWidget();il=W.QVBoxLayout(inspector);il.setContentsMargins(8,0,0,0);self.inspect_side=ui.ComboBox();self.inspect_side.addItems(['Inspect A / left pane','Inspect D / middle pane','Inspect E / right pane']);self.inspect_side.currentIndexChanged.connect(self.change_side);il.addWidget(self.inspect_side);self.object_choice=ui.ComboBox();il.addWidget(ui.label('Crane inspector','subtitle'));il.addWidget(self.object_choice);self.summary=W.QLabel();self.summary.setWordWrap(True);il.addWidget(self.summary);self.details=W.QTreeWidget();self.details.setHeaderLabels(['Recorded field','Value']);self.details.setColumnWidth(0,155);il.addWidget(self.details,1);self.reason=ui.label('Only recorded reasons. Red indicates a rejected center, not a proven physical miss.','muted',True);il.addWidget(self.reason);split.addWidget(inspector);split.setSizes([1050,390])
        self.timeline=MotionTimeline();layout.addWidget(self.timeline);self.timeline.seek.connect(self.seek)
        bookmarks=W.QHBoxLayout();bookmarks.addWidget(ui.button('Previous Δ',callback=lambda:self.go_housing_change(-1),role='small'));bookmarks.addWidget(ui.button('Next housing Δ',callback=lambda:self.go_housing_change(1),role='small'));self.bookmarks=ui.ComboBox();self.bookmarks.addItem('Problem bookmarks…',None)
        for i,label in BOOKMARKS:self.bookmarks.addItem(f'f{i} · {label}',i)
        self.bookmarks.activated.connect(self.bookmark);bookmarks.addWidget(self.bookmarks,1);self.mark=W.QLineEdit();self.mark.setPlaceholderText('New bookmark label');bookmarks.addWidget(self.mark);bookmarks.addWidget(ui.button('Bookmark frame',callback=self.add_bookmark,role='small'));layout.addLayout(bookmarks)
        incidents=W.QHBoxLayout();self.incidents=ui.ComboBox();self.incidents.addItem('Housing loss events…',None);incidents.addWidget(self.incidents,1);self.incident_phase=ui.ComboBox();self.incident_phase.addItems(['Previous DIRECT','Before loss','First missing','Middle missing','Reappeared']);incidents.addWidget(self.incident_phase);incidents.addWidget(ui.button('Go event',callback=self.go_incident,role='small'));layout.addLayout(incidents)
        transport=W.QHBoxLayout();self.prev=ui.button('‹',callback=lambda:self.seek(self.frame.value()-1),role='small');transport.addWidget(self.prev);self.play=ui.button('Play',role='small',checkable=True);transport.addWidget(self.play);transport.addWidget(ui.button('›',callback=lambda:self.seek(self.frame.value()+1),role='small'));self.frame=W.QSpinBox();self.frame.setRange(0,0);self.frame.setPrefix('f');transport.addWidget(self.frame);self.slider=W.QSlider(C.Qt.Horizontal);self.slider.setRange(0,0);transport.addWidget(self.slider,1);self.speed=ui.ComboBox();self.speed.addItems(['0.25x','0.5x','1x','2x']);self.speed.setCurrentIndex(2);transport.addWidget(self.speed);self.time_basis=ui.ComboBox();self.time_basis.addItems(['Relative seconds','Sensor seconds']);self.time_basis.currentIndexChanged.connect(self.time_basis_changed);transport.addWidget(self.time_basis);self.seconds=W.QDoubleSpinBox();self.seconds.setDecimals(3);self.seconds.setSuffix(' s');transport.addWidget(self.seconds);transport.addWidget(ui.button('Go time',callback=self.seek_seconds,role='small'));self.stamp_label=ui.label('Sensor timestamp','muted');transport.addWidget(self.stamp_label);layout.addLayout(transport)
        self.notice=ui.label('Load indexed Raw bytes + current C++ results. No replay on frame navigation.','muted',True);layout.addWidget(self.notice)
        self.layer_dialog=W.QDialog(self);self.layer_dialog.setWindowTitle('Crane layers · display only');ll=W.QVBoxLayout(self.layer_dialog);self.layers={}
        for name in LAYERS:
            cb=W.QCheckBox(name);cb.setChecked(name not in ['BEV Cluster','Z Gaussian Layer','Observed Spans','Inferred Gaps','Moving Boundary Patch']);cb.toggled.connect(lambda *_:self.redraw());ll.addWidget(cb);self.layers[name]=cb
        self.layers['Gap ROI'].setEnabled(False);self.layers['Gap ROI'].setToolTip('ROI geometry is not serialized in current diagnostics. No invented ROI.');self.layers['Gap ROI'].setChecked(False)
        self.slider.valueChanged.connect(self.frame.setValue);self.frame.valueChanged.connect(self.slider.setValue);self.frame.valueChanged.connect(self.seek);self.canvas.selected.connect(self.select_object);self.object_choice.currentTextChanged.connect(self.choose_object)
        self.projection.currentTextChanged.connect(lambda *_:self.redraw(True));self.compare.toggled.connect(self.request_current);self.triple.toggled.connect(self.request_current)
        for box in [self.a,self.b,self.c]:box.currentTextChanged.connect(self.request_current)
        self.timer=C.QTimer(self);self.timer.setInterval(100);self.timer.timeout.connect(self.tick);self.timer.start()
    def async_call(self,fn,cb):
        w=self.Worker(fn,self);self.workers.add(w)
        def done(value,error):
            try:cb(value,error)
            finally:self.workers.discard(w)
        w.done.connect(done);w.finished.connect(w.deleteLater);w.start()
    def choose_dataset(self):
        folder=W.QFileDialog.getExistingDirectory(self,'Prepared Crane evaluation directory',str(DEFAULT))
        if folder:self.load(folder)
    def load(self,folder=DEFAULT):
        if self.workers:return
        self.busy=True;self.notice.setText('Indexing timestamp-aligned C++ results…')
        def ready(data,error):
            self.busy=False
            if error:self.notice.setText('Crane dataset unavailable: '+error);return
            self.data=data
            for box in [self.a,self.b,self.c]:
                box.blockSignals(True);box.clear();box.addItems(list(data.runs));box.blockSignals(False)
            self.b.blockSignals(True);self.b.setCurrentIndex(self.b.count()-1);self.b.blockSignals(False);
            if self.a.findText('Housing A — baseline')>=0:
                self.a.blockSignals(True);self.a.setCurrentText('Housing A — baseline');self.a.blockSignals(False);self.b.blockSignals(True);self.b.setCurrentText('Housing C — learned' if self.b.findText('Housing C — learned')>=0 else 'Housing B — continuity');self.b.blockSignals(False);self.id_metric.setCurrentIndex(1)
            self.c.setCurrentIndex(self.c.count()-1)
            if self.c.findText('Housing E — selective')>=0:
                for box,name in [(self.b,'Housing D — current'),(self.c,'Housing E — selective')]:
                    box.blockSignals(True);box.setCurrentText(name);box.blockSignals(False)
                self.triple.setChecked(True)
            self.compare.setEnabled(self.b.count()>1);self.compare.setChecked(self.b.count()>1);self.inspect_side.setCurrentIndex(1 if self.b.count()>1 else 0)
            self.frame.setRange(0,len(data.index)-1);self.slider.setRange(0,len(data.index)-1);self.seconds.setRange(0,data.times[-1]);self.dataset_label.setText(f'Oct24 · {len(data.index):,} frames · world');self.seek(500)
            events=data.folder/'housing_loss_events.json'
            self.incidents.clear();self.incidents.addItem('Housing loss events…',None)
            if events.exists():
                for event in json.loads(events.read_text()):self.incidents.addItem(f"{'LEFT' if event['side']==0 else 'RIGHT'} f{event['first_missing']}–{event['last_missing']} · {event['classification']}",event)
            saved=data.folder/'qt_bookmarks.json'
            if saved.exists():
                for entry in json.loads(saved.read_text()):
                    if self.bookmarks.findData(entry['frame'])<0:self.bookmarks.addItem(entry['label'],entry['frame'])
        self.async_call(lambda:CraneData(folder),ready)
    def names(self):
        names=[self.a.currentText()]
        if self.compare.isChecked():names.append(self.b.currentText())
        if self.compare.isChecked() and self.triple.isChecked():names.append(self.c.currentText())
        return names
    def request_current(self,*_):self.seek(self.frame.value())
    def seek(self,i):
        if self.data is None or self.closing:return
        i=max(0,min(len(self.data.index)-1,int(i)));self.frame.blockSignals(True);self.frame.setValue(i);self.frame.blockSignals(False);self.slider.blockSignals(True);self.slider.setValue(i);self.slider.blockSignals(False)
        if self.busy:self.pending=i;return
        self.busy=True;self.pending=None;names=self.names();begin=time.perf_counter();self.notice.setText(f'Loading f{i}…')
        def ready(value,error):
            self.busy=False
            if error:self.play.setChecked(False);self.notice.setText('Frame read error: '+error)
            else:
                self.value=value;self.frame_load_ms.append((time.perf_counter()-begin)*1000);self.stamp_label.setText(f"{value['sec']:.3f}s · {value['stamp_ns']} ns");self.refresh_objects();self.redraw();self.notice.setText(f"f{i} / {len(self.data.index)-1} · Raw {len(value['points']):,} finite points; display LOD ≤24k · C++ output / no rerun. NONE ≠ proven miss.")
            if self.pending is not None:
                pending=self.pending;self.pending=None;self.seek(pending)
        self.async_call(lambda:self.data.frame(i,names),ready)
    def refresh_objects(self):
        old=self.selected;opts=objects(self.value['results'][min(self.selected_side,len(self.value['results'])-1)]);self.object_choice.blockSignals(True);self.object_choice.clear();self.object_choice.addItem('Frame overview');self.object_choice.addItems(list(opts));self.object_choice.setCurrentText(old if old in opts else 'Frame overview');self.object_choice.blockSignals(False);self.selected=old if old in opts else None;self.inspect()
        # Capabilities are based on serialized evidence, never generated classifications.
        for name,key in [('Z Gaussian Layer','z_layers'),('BEV Cluster','subclusters_detail')]:
            available=any(bool(r.get(key)) for r in self.value['results']);self.layers[name].setEnabled(available);self.layers[name].setToolTip('Recorded C++ geometry' if available else 'Not serialized in this saved run; choose enriched B run.')
    def change_side(self,i):
        self.selected_side=i
        if self.value:self.refresh_objects();self.redraw()
    def choose_object(self,name):self.selected=None if name=='Frame overview' else name;self.inspect();self.redraw()
    def select_object(self,name,side):self.selected_side=side;self.selected=name;self.refresh_objects();self.redraw()
    def inspect(self):
        if not self.value:return
        side=min(self.selected_side,len(self.value['results'])-1);r=self.value['results'][side];o=objects(r).get(self.selected);info=o['data'].copy() if o else {'LEFT Housing':r['housing'][0],'RIGHT Housing':r['housing'][1],'Virtual Hook':r['hooks'],'Cargo':r['cargo'],'Gap':r['gap'],'Moving':r.get('motion_temporal',{'raw':'Not recorded in A; use enriched B'}),'Boundary Patches':r['boundary_patches'],'Work':r['work'],'FSM':r['fsm']}
        if o and o['kind'] in ['housing','pending']:
            h=o['data'];info.update({'Detection':STATUS[h['status']],'measurement_tracker_difference_m':float(np.linalg.norm(np.array(h['xyz'])-h['observed'])) if h['status']==1 else 'Not a current direct measurement','last_DIRECT_age_sec':self.value['stamp_ns']/1e9-h['last_direct_stamp'] if h.get('last_direct_stamp',0)>0 else 'Not recorded'})
        if o and o['kind']=='housing':info['named_gate_counts']={name:o['data']['gates'][i] for i,name in enumerate(GATE_NAMES)}
        if 'housing_trace' in r:
            info['actual_housing_trace']=r['housing_trace']
        self.details.clear()
        def add(parent,k,v):
            item=W.QTreeWidgetItem(parent,[str(k),'' if isinstance(v,(dict,list)) else str(v)])
            if isinstance(v,dict):
                for kk,vv in v.items():add(item,kk,vv)
            elif isinstance(v,list):
                for kk,vv in enumerate(v):add(item,kk,vv)
            return item
        for k,v in info.items():add(self.details,k,v)
        self.details.expandToDepth(0);m=r.get('motion_temporal',{});self.summary.setText(f"{self.value['names'][side]} · {self.selected or 'Frame overview'}\nL: {STATUS[r['housing'][0]['status']]} / R: {STATUS[r['housing'][1]['status']]}\nMoving raw={m.get('raw','not recorded')} supported={m.get('supported','not recorded')} → output={r['moving']}\nStart={m.get('start_elapsed','N/A')} s / Stop={m.get('stop_elapsed','N/A')} s\nLast transition={m.get('transition_stamp','N/A')} sensor sec\nWork={r['work']} · {r['fsm']} · Cargo={r['cargo']['id']}")
        if o and o['kind']=='reject':self.reason.setText('RED = closest C++ rejected center. '+o['data']['reason']+'; exact Raw membership/bounds unavailable. Not a confirmed physical miss.')
        else:self.reason.setText('Green: confirmed DIRECT. Yellow: HELD/unconfirmed. Purple: derived Hook. Red: recorded rejected center; physical miss unproven. NONE does not imply a miss.')
    def redraw(self,fit=False):
        if not self.value:return
        render_begin=time.perf_counter()
        self.canvas.render(self.value,self.projection.currentText(),{k:v.isChecked() and v.isEnabled() for k,v in self.layers.items()},self.selected,fit)
        key=tuple(self.value['names'])
        # Timeline uses cached scalar results; no raw reads or C++ execution.
        if (key,self.id_metric.currentIndex())!=self._timeline_key:self.timeline.render(self.data,list(key),self.value['frame'],self.id_metric.currentIndex()==1);self._timeline_key=(key,self.id_metric.currentIndex())
        self.timeline.cursor(self.value['frame'],[0,5,15,60][self.timeline_window.currentIndex()])
        self.render_ms=(time.perf_counter()-render_begin)*1000
    def fit_scene(self):self.selected=None;self.object_choice.setCurrentIndex(0);self.redraw(True)
    def show_layers(self):self.layer_dialog.show()
    def time_basis_changed(self):
        if self.data:
            absolute=self.time_basis.currentIndex()==1
            base=self.data.stamps[0]/1e9 if absolute else 0
            self.seconds.setRange(base,base+self.data.times[-1]);self.seconds.setValue(base+(self.value['sec'] if self.value else 0))
    def seek_seconds(self):
        if self.data:
            target=self.seconds.value()-(self.data.stamps[0]/1e9 if self.time_basis.currentIndex()==1 else 0)
            i=min(len(self.data.times)-1,bisect.bisect_left(self.data.times,target))
            if i>0 and abs(self.data.times[i-1]-target)<abs(self.data.times[i]-target):i-=1
            self.seek(i)
    def go_housing_change(self,direction):
        if not self.data or not self.compare.isChecked():return
        names=self.names();rows=[self.data.runs[n] for n in names]
        changes=[]
        for i,values in enumerate(zip(*rows)):
            a=values[0]
            if any(a[k]!=b[k] for b in values[1:] for k in ['left','right','lid','rid','lc','rc']):changes.append(i)
        if not changes:self.notice.setText('No saved Housing classification / identity / confirmation changes in this pair.');return
        index=bisect.bisect_right(changes,self.frame.value()) if direction>0 else bisect.bisect_left(changes,self.frame.value())-1
        if 0<=index<len(changes):self.seek(changes[index])
    def go_incident(self):
        e=self.incidents.currentData()
        if e:
            frames=[e['before'],max(0,e['first_missing']-1),e['first_missing'],(e['first_missing']+e['last_missing'])//2,e['reappeared']]
            self.seek(frames[self.incident_phase.currentIndex()])
    def bookmark(self,index):
        i=self.bookmarks.itemData(index)
        if i is not None:
            for n in ['Rejected Candidates','Unknown / Unconfirmed Objects','Virtual Hook','Cargo ROI','Root / Wire / Chain']:self.layers[n].setChecked(True)
            self.seek(i)
    def add_bookmark(self):
        if not self.value:return
        label=self.mark.text().strip() or 'User bookmark';self.bookmarks.addItem(f"f{self.frame.value()} · {label}",self.frame.value());p=self.data.folder/'qt_bookmarks.json';rows=[{'frame':self.bookmarks.itemData(i),'label':self.bookmarks.itemText(i)} for i in range(1,self.bookmarks.count())];p.write_text(json.dumps(rows,indent=2))
    def tick(self):
        if self.play.isChecked() and self.data and not self.busy:
            if self.frame.value()>=len(self.data.index)-1:self.play.setChecked(False)
            else:self.timer.setInterval(int(100/[.25,.5,1,2][self.speed.currentIndex()]));self.seek(self.frame.value()+1)
    def export_menu(self):
        if not self.value or self.busy:return
        menu=W.QMenu(self)
        for label,kind in [('Current Qt screen PNG','screen'),('Selected object zoom PNG','selected'),('Previous / current / next PNG','sequence'),('All problem bookmarks PNG','bookmarks')]:menu.addAction(label,lambda checked=False,k=kind:self.export(k))
        menu.exec_(self.mapToGlobal(C.QPoint(200,60)))
    def save_current(self,folder,name='screen'):
        folder=Path(folder);folder.mkdir(parents=True,exist_ok=True);self.grab().save(str(folder/(name+'.png')))
        meta={**self.data.provenance,'frame':self.value['frame'],'stamp_ns':self.value['stamp_ns'],'sec':self.value['sec'],'projection':self.projection.currentText(),'selected':self.selected,'selected_side':self.selected_side,'layers':{k:v.isChecked() for k,v in self.layers.items()},'runs':{n:str(self.data.paths[n]) for n in self.value['names']},'config_artifacts':{n:self.data.config_artifacts[n] for n in self.value['names']},'cameras':[{'xlim':a.get_xlim(),'ylim':a.get_ylim(),'zlim':a.get_zlim() if hasattr(a,'get_zlim') else None,'elev':getattr(a,'elev',None),'azim':getattr(a,'azim',None)} for a in self.canvas.axes],'display_raw_lod_max':24000,'results':self.value['results']};(folder/(name+'.json')).write_text(json.dumps(meta,ensure_ascii=False,indent=2));return folder/(name+'.png')
    def export(self,kind='screen',folder=None):
        if not self.value or self.busy:return
        folder=Path(folder) if folder else self.data.folder/'qt_exports'/time.strftime('%Y%m%d_%H%M%S')
        if kind in ['screen','selected']:
            if kind=='selected' and self.selected is None:self.notice.setText('Select an actual object before exporting an object zoom.');return
            if kind=='selected':self.redraw(True)
            self.save_current(folder,kind);self.notice.setText('Saved Qt PNG + sensor timestamp + full recorded diagnostics: '+str(folder));return
        frames=[max(0,self.value['frame']-1),self.value['frame'],min(len(self.data.index)-1,self.value['frame']+1)] if kind=='sequence' else sorted({self.bookmarks.itemData(i) for i in range(1,self.bookmarks.count())})
        self.play.setChecked(False);self.export_queue=list(frames);self.export_folder=folder;self.export_kind=kind;self.export_original=self.value['frame'];self.export_paths=[];self.export_timer=C.QTimer(self);self.export_timer.timeout.connect(self.export_step);self.export_timer.start(100);self.seek(frames[0])
    def export_step(self):
        if self.busy or not self.value:return
        if self.export_queue:
            target=self.export_queue[0]
            if self.value['frame']!=target:self.seek(target);return
            if self.export_kind=='bookmarks':self.fit_scene()
            self.save_current(self.export_folder,f'f{target:04d}');self.export_paths.append(str(self.export_folder/f'f{target:04d}.png'));self.export_queue.pop(0)
            if self.export_queue:self.seek(self.export_queue[0]);return
        self.export_timer.stop()
        if self.export_kind=='sequence':
            images=[G.QImage(path) for path in self.export_paths];canvas=G.QImage(sum(im.width() for im in images),max(im.height() for im in images),G.QImage.Format_RGB32);canvas.fill(G.QColor('#17221f'));painter=G.QPainter(canvas);x=0
            for im in images:painter.drawImage(x,0,im);x+=im.width()
            painter.end();canvas.save(str(self.export_folder/'sequence_compare.png'))
        self.seek(self.export_original);self.notice.setText('Batch Qt export complete: '+str(self.export_folder))
