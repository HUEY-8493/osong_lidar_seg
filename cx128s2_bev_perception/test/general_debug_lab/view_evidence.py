"""Shared display evidence only; no Qt, no perception or association evaluation."""
import json, math

DEFAULTS={k:True for k in ('Points','Foreground','ROI non-foreground','Outside ROI','ROI inside','Clusters','Vehicle-like','Matched','Rejected components','Unknown-like','Unclassified metadata','Tracks','Actual observations','Prediction-only','DORMANT','Public VEHICLE','Non-public','Rejected tracks','Geometry','Memory Geometry','KF Prediction','FOV','ROI','Grid')}


UNSUPPORTED={'Accepted classification':'No per-initial-cluster Detection acceptance verdict is serialized. A matched cluster can still belong to a rejected non-public track.', 'Background':'The frame exporter has ROI and foreground masks, not an explicit background mask. Use ROI non-foreground; it is not a semantic background label.', 'Ignore / Excluded':'The derived frame export does not include the ignored / hard-ignored masks. Outside ROI is not equivalent to ignored.', 'Unknown points':'No per-point semantic UNKNOWN class is serialized.', 'Unassociated':'No complete per-cluster final association table is recorded. Lack of a verified match does not mean unassociated.', 'LOST lifecycle':'The saved trace has missed-frame counters, not the internal vehicle lifecycle enum. Do not equate a rejected generic track with canonical LOST.'}


COLORS={'observed':'#a9d9b3','prediction':'#c5a1dc','dormant':'#aa9890','rejected':'#b57e73','selected':'#79e2a5','unknown':'#b8bdb7','memory':'#c2a16e'}


def truth(v):return v is True or v==1 or v=='1' or v=='true'


def finite(v):
    try:return math.isfinite(float(v))
    except (ValueError,TypeError):return False


def tracks(state):
    csv={int(t['id']):t for t in state.get('tracks',[])};trace=(state.get('trace') or {}).get('tracks')
    if trace is None:return list(csv.values())
    return [dict(csv.get(int(t['id']),{}),**t) for t in trace]


def flags(t):
    lost=t.get('lost_frames',t.get('lost'));lost=int(lost) if lost is not None else None
    actual=t.get('observation',{}).get('actual')
    if actual is None and lost is not None:
        # C++ trace_event observation.actual is defined as lost_frames == 0.
        actual=lost==0
    dormant=truth(t.get('dormant',False));life=t.get('lifecycle')
    return {'actual':actual,'prediction':actual is False and not dormant,'dormant':dormant,'missed':lost is not None and lost>0,'lost':t.get('canonical_vehicle_lifecycle',life) in ('LOST','LOST_PREDICTIVE'),'rejected':life=='REJECTED','public':truth(t.get('public',False)) and truth(t.get('vehicle',False))}


def aabb(box):
    if not isinstance(box,(list,tuple)) or len(box)<4 or not all(finite(v) for v in box[:4]):return None
    x1,x2,y1,y2=map(float,box[:4])
    if x2<x1 or y2<y1 or (x2==x1 and y2==y1):return None
    return (x1,x2,y1,y2)


def cluster_box(c):return (float(c['min'][0]),float(c['max'][0]),float(c['min'][1]),float(c['max'][1]))


def union(bounds):
    bounds=[b for b in bounds if b is not None]
    return (min(b[0] for b in bounds),max(b[1] for b in bounds),min(b[2] for b in bounds),max(b[3] for b in bounds)) if bounds else None


def verified_links(data,state):
    cs={int(c['id']):c for c in data.get('clusters',[])};features={int(f['id']):f for f in data.get('features',[])};result={}
    for tr in tracks(state):
        if flags(tr)['actual'] is True and 'current_component_support' in tr:
            ids={int(m['cluster']) for m in tr['current_component_support']
                 if m.get('complete') and int(m.get('associated_points',0))==int(m.get('cluster_points',-1))
                 and int(m.get('cluster_points',0))>0 and int(m['cluster']) in cs
                 and int(cs[int(m['cluster'])]['count'])==int(m['cluster_points'])}
            if ids:result[int(tr['id'])]=ids
            continue
        cid=tr.get('component_id');c=cs.get(cid);f=features.get(cid)
        if not c or not f or flags(tr)['actual'] is not True:continue
        box=tr.get('geometry',{}).get('aabb',[]);fb=f.get('box',[]);cb=[c['min'][0],c['max'][0],c['min'][1],c['max'][1],c['min'][2],c['max'][2]]
        if len(box)!=6 or len(fb)!=6:continue
        if int(tr.get('observation',{}).get('points',-1))!=int(c['count']) or int(f.get('points',-1))!=int(c['count']):continue
        # Equality checks, not distance/ownership heuristics. Merged/refined cases
        # whose stored geometry changed are intentionally left unlinked.
        if all(abs(float(x)-float(y))<1e-6 and abs(float(x)-float(z))<1e-6 for x,y,z in zip(box,fb,cb)):result[int(tr['id'])]={int(cid)}
    return result


def evidence(data,state):
    candidates={int(c['id']):c for c in (state.get('trace') or {}).get('candidates',[])};links=verified_links(data,state);matched=set().union(*links.values()) if links else set();meta={}
    for c in data.get('clusters',[]):
        cid=int(c['id']);candidate=candidates.get(cid);tags=set()
        if candidate:
            if candidate.get('component_type')=='vehicle_like':tags.add('Vehicle-like')
            if candidate.get('component_type')=='unknown_like':tags.add('Unknown-like')
            if candidate.get('rejection_reason'):tags.add('Rejected components')
        else:tags.add('Unclassified metadata')
        if cid in matched:tags.add('Matched')
        meta[cid]=tags
    return meta,links


def track_visible(t,layers):
    f=flags(t)
    if not layers.get('Tracks',True):return False
    for key,active in [('Actual observations',f['actual'] is True),('Prediction-only',f['prediction']),('DORMANT',f['dormant']),('LOST lifecycle',f['lost']),('Public VEHICLE',f['public']),('Non-public',not f['public']),('Rejected tracks',f['rejected'])]:
        if active and not layers.get(key,True):return False
    return True


def position(tr,data,state):
    ob=tr.get('observation',{});actual=flags(tr)['actual']
    if actual and ob.get('current_measurement_valid') and len(ob.get('current_measurement',[]))==2 and all(finite(v) for v in ob['current_measurement']):return tuple(map(float,ob['current_measurement'])),'Recorded applied measurement'
    if actual and ob.get('measurement_valid') and len(ob.get('measurement',[]))==2 and all(finite(v) for v in ob['measurement']):return tuple(map(float,ob['measurement'])),'Recorded measurement'
    links=verified_links(data,state).get(int(tr['id']),set())
    if actual and links:
        c=next(c for c in data['clusters'] if int(c['id']) in links);return tuple(map(float,c['center'][:2])),'Observed cluster center'
    p=ob.get('predicted',[]) if actual is False else []
    if len(p)==2 and all(finite(v) for v in p):return tuple(map(float,p)),('Prediction-only · recorded KF prediction' if tr.get('kf',{}).get('initialized') else 'Prediction-only · recorded predicted position')
    if finite(tr.get('x')) and finite(tr.get('y')):return (float(tr['x']),float(tr['y'])),('Recorded track output' if actual else 'Prediction-only · track output')
    return None,'Position unavailable'


def selected_bounds(data,state,tid,cid,target='Observed Cluster'):
    if cid is not None:
        c=next((c for c in data.get('clusters',[]) if int(c['id'])==cid),None)
        return (cluster_box(c),'Selected initial cluster · actual point bounds') if c else (None,'Selected cluster unavailable at this frame')
    tr=next((t for t in tracks(state) if int(t['id'])==tid),None)
    if tr is None:return None,'Selected track unavailable at this frame'
    geometry=tr.get('geometry',{});ob=tr.get('observation',{});actual=flags(tr)['actual'];pos,source=position(tr,data,state)
    if target=='Reliable Geometry':
        b=aabb(geometry.get('reliable_aabb',[])) if tr.get('memory',{}).get('last_reliable_stamp',0)>0 else None
        return b,'Stored reliable memory geometry' if b else 'Reliable geometry not recorded'
    if target=='Current Geometry':
        b=aabb(geometry.get('aabb',[]));return b,('Current observed geometry' if actual else 'Stored geometry · not current observation') if b else 'Geometry not recorded'
    if target=='Observed Cluster' and actual:
        ids=verified_links(data,state).get(int(tid),set());bounds=[cluster_box(c) for c in data.get('clusters',[]) if int(c['id']) in ids]
        if bounds:return union(bounds),'Observed cluster · verified stored ownership'
        b=aabb(geometry.get('aabb',[]))
        if b:return b,'Observed feature geometry · initial cluster mapping unavailable'
    if pos:
        b=aabb(geometry.get('aabb',[]));w=(b[1]-b[0])/2 if b else 0.;h=(b[3]-b[2])/2 if b else 0.
        if target=='Track + Prediction' and actual and b:
            bounds=[b,(pos[0],pos[0],pos[1],pos[1])];prediction=ob.get('predicted',[])
            if len(prediction)==2 and all(finite(v) for v in prediction):bounds.append((float(prediction[0]),float(prediction[0]),float(prediction[1]),float(prediction[1])))
            return union(bounds),'Recorded geometry + track / measurement + KF prediction'
        return (pos[0]-w,pos[0]+w,pos[1]-h,pos[1]+h),source+' · stored size is a viewport guide only'
    return None,source


LANES=[('Observed','actual','#679c77'),('Prediction','prediction','#b09bc5'),('Missed counter','missed','#b58b6c'),('DORMANT','dormant','#9e9d8e'),('Reacquired','reacquired','#568b78'),('Removed / FIFO','removed','#ad796d')]


def lifecycle(lab,rid,key,runid,tid):
    """Recorded snapshots only. No extension beyond the last recorded frame."""
    from pathlib import Path
    run=lab.run(rid,key,runid);path=Path(run['prefix']+'_trace.jsonl');rows=lab.rows(rid,key);cachekey=(str(path),path.stat().st_mtime_ns if path.exists() else 0,int(tid))
    with lab.lock:
        cache=getattr(lab,'lifecycle_view_cache',{})
        if cachekey in cache:return cache[cachekey]
    points=[];events=[];previous=None;previous_frame=None;fifo=[];conflicts=0
    if path.exists():
        with path.open() as stream:
            for line in stream:
                frame=json.loads(line);n=int(frame['frame']);tr=next((t for t in frame.get('tracks',[]) if int(t['id'])==int(tid)),None)
                if tr:
                    f=flags(tr);raw=tr.get('association_source');conflicts+=int(f['actual'] is True and raw=='PREDICTION_ONLY');f['frame']=n;f['stamp']=rows[n-1]['relative_sec'];f['raw_association']=raw;f['lost_frames']=tr.get('lost_frames');points.append(f)
                    if tr.get('memory',{}).get('reacquire_result')=='ACCEPTED':events.append({'kind':'reacquired','frame':n,'source':'memory.reacquire_result=ACCEPTED'})
                if previous is not None and previous_frame==n-1 and tr is None:events.append({'kind':'removed','frame':n,'source':'Track removed from consecutive recorded snapshots; FIFO completion not inferred'})
                if int(tid) in fifo and int(tid) not in frame.get('fifo',[]):events.append({'kind':'removed','frame':n,'source':'Explicit FIFO membership removed; departure completion not inferred'})
                previous=tr;previous_frame=n;fifo=frame.get('fifo',[])
    intervals=[]
    for _,name,_ in LANES[:4]:
        current=None
        for p in points:
            if not p[name]:
                if current:intervals.append(current);current=None
                continue
            if current and p['frame']==current['end']+1:current['end']=p['frame'];current['end_sec']=p['stamp']
            else:
                if current:intervals.append(current)
                current={'lane':name,'start':p['frame'],'end':p['frame'],'start_sec':p['stamp'],'end_sec':p['stamp']}
        if current:intervals.append(current)
    result={'track':int(tid),'run':runid,'intervals':intervals,'events':events,'samples':points,'conflicting_association_source_frames':conflicts,'notice':'Observation follows the C++ lost_frames==0 observation flag; raw association_source can disagree. Missed is a counter, not canonical LOST. Missing snapshots are not interpolated. No removal is inferred at bag end.','canonical_lost_recorded':any(p['lost'] for p in points)}
    with lab.lock:
        if len(cache)>12:cache.clear()
        cache[cachekey]=result;lab.lifecycle_view_cache=cache
    return result




def tracking_series(lab,rid,key,runid,tid):
    """Display only: recorded C++ values; absent values stay null, lines stay broken."""
    import csv
    from pathlib import Path
    run=lab.run(rid,key,runid);prefix=Path(run['prefix']);trace=Path(str(prefix)+'_trace.jsonl');csvpath=Path(str(prefix)+'_tracks.csv')
    signature=(str(prefix),int(tid),trace.stat().st_mtime_ns if trace.exists() else 0,csvpath.stat().st_mtime_ns if csvpath.exists() else 0)
    cache=getattr(lab,'tracking_series_cache',{})
    if signature in cache:return cache[signature]
    csvrows={}
    if csvpath.exists():
        for row in csv.DictReader(csvpath.open()):
            if int(row['id'])==int(tid):csvrows[int(row['frame'])]=row
    rows=lab.rows(rid,key);samples=[];events=[];previous=None;previous_frame=None
    def pair(v):return v if isinstance(v,(list,tuple)) and len(v)==2 and all(finite(x) for x in v) else None
    if trace.exists():
        for line in trace.open():
            frame=json.loads(line);number=int(frame['frame']);tr=next((t for t in frame.get('tracks',[]) if int(t['id'])==int(tid)),None)
            q={'frame':number,'sec':rows[number-1]['relative_sec'],'track_present':tr is not None,'track_id':int(tid)}
            if tr is None:
                if previous is not None and previous_frame==number-1:events.append({'kind':'removed','frame':number})
                samples.append(q);previous=None;previous_frame=number;continue
            t=dict(csvrows.get(number,{}),**tr);f=flags(t);ob=t.get('observation',{});kf=t.get('kf',{});geo=t.get('geometry',{})
            matched=f['actual'] is True
            current_count=ob.get('points',t.get('points'))
            actual=matched and current_count is not None and int(current_count)>0
            public=f['public'];classified=truth(tr['vehicle']) if 'vehicle' in tr else True if truth(csvrows.get(number,{}).get('vehicle',False)) else None
            q.update(actual=actual,cluster_matched=matched,vehicle_classified=classified,vehicle_public=actual and public,prediction_only=f['prediction'],dormant=f['dormant'],lost=f['lost'],classification_rejected=(actual and not classified) if classified is not None else None,not_public=actual and not public,kf_available=truth(kf.get('initialized',False)),lost_frames=t.get('lost_frames',t.get('lost')))
            canonical=t.get('canonical_vehicle_lifecycle')
            if canonical is None and t.get('lifecycle') not in ('LOST','LOST_PREDICTIVE','DORMANT'):q['lost']=None
            if 'unmatched_bounded_components' in t:q['unmatched_cluster']=int(t['unmatched_bounded_components'])>0;q['unmatched_cluster_count']=int(t['unmatched_bounded_components'])
            for name in ('x','y','vx','vy','score'):
                if finite(t.get(name)):q[name]=float(t[name])
            q['point_count']=int(ob.get('points',t.get('points',0))) if actual else None
            measurement=pair(ob.get('current_measurement')) if truth(ob.get('current_measurement_valid',False)) else pair(ob.get('measurement')) if actual and truth(ob.get('measurement_valid',False)) else None
            if measurement:q['measurement_x'],q['measurement_y']=map(float,measurement)
            prediction=pair(ob.get('predicted'))
            if prediction and q['kf_available'] and ob.get('prediction_valid',True):q['prediction_x'],q['prediction_y']=map(float,prediction)
            if q['kf_available']:
                state=kf.get('state',[])
                for i,name in enumerate(('kf_x','kf_y','kf_vx','kf_vy')):
                    if len(state)>i and finite(state[i]):q[name]=float(state[i])
                covariance=kf.get('covariance',[])
                for i,row in enumerate(covariance):
                    if isinstance(row,list):
                        for j,value in enumerate(row):
                            if finite(value):q[f'cov_{i}_{j}']=float(value)
                innovation=pair(kf.get('innovation'))
                if actual and innovation and kf.get('innovation_current_valid',True):q['innovation_x'],q['innovation_y']=map(float,innovation)
            for name in ('length','width','height'):
                if actual and finite(geo.get(name)):q[name]=float(geo[name])
            if truth(geo.get('reliable',False)):
                for name,value in zip(('memory_length','memory_width','memory_height'),geo.get('reliable_size',[])):
                    if finite(value):q[name]=float(value)
            if finite(t.get('vehicle_size_score')):q['vehicle_size_score']=float(t['vehicle_size_score'])
            if finite(t.get('vehicle_score_threshold')):q['vehicle_score_threshold']=float(t['vehicle_score_threshold'])
            gates=[g for g in frame.get('gates',[]) if g.get('track')==int(tid)]
            angular=[g for g in gates if g.get('gate')=='current_angular_support']
            if angular:
                a=angular[-1];bins=a.get('actual_bins');expected=a.get('expected_rays')
                if finite(bins) and finite(expected):
                    relief=min(.5,max(0,float(a.get('occluded_fraction',0)))) if a.get('has_occluder') else 0
                    effective=float(expected)*(1-relief);q.update(angular_bins=float(bins),expected_rays=float(expected),required_bins=float(a['threshold']) if finite(a.get('threshold')) else None,support_ratio=float(bins)/effective if effective>0 else None,angular_scope='current matched feature' if actual else 'unmatched current proposal',angular_pass=a.get('pass'))
            chosen=[g for g in gates if g.get('function')=='generic_selection']
            if actual and chosen:
                g=chosen[-1]
                if finite(g.get('distance')):q['association_distance']=float(g['distance'])
                if finite(g.get('squared_distance')):q['association_cost_squared_distance']=float(g['squared_distance'])
            for g in gates:
                if g.get('function')=='evaluateDetectionMetrics':q['vehicle_size_score']=g.get('vehicle_size_score');q['vehicle_score_threshold']=g.get('vehicle_min_score')
                if g.get('gate')=='generic_spatial_gate' and g.get('evaluated') and finite(g.get('threshold')):q['association_spatial_gate']=float(g['threshold'])
            if previous is not None and previous_frame==number-1:
                if not previous.get('actual',False) and actual:events.append({'kind':'observed_again','frame':number,'source':'recorded observation resumed; not necessarily canonical Reacquire'})
            if t.get('memory',{}).get('reacquire_result') in ('ACCEPTED','REACQUIRED') and (previous is None or not previous.get('actual',False)):events.append({'kind':'reacquired','frame':number})
            samples.append(q);previous=q;previous_frame=number
    result={'track':int(tid),'run':runid,'samples':samples,'events':events,'notice':'Current measurements are recorded C++ inputs, KF is separate. Gaps/nulls are not interpolated. Unmatched lane means recorded bounded pieces unlinked to this selected track, not an asserted vehicle. Run-local IDs are not physical identity labels.'}
    if len(cache)>8:cache.clear()
    cache[signature]=result;lab.tracking_series_cache=cache
    return result
