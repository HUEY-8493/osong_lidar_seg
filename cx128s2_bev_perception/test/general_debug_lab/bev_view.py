"""Read-only render evidence, camera bounds, and native BEV/lifecycle views.

No associations are calculated here. Track-point membership requires a stored
component ID AND unchanged point count / exact stored feature bounds.
"""
import json, math
from PyQt5 import QtCore as C, QtGui as G, QtWidgets as W
from view_evidence import *

def rect(bounds,padding=True):
    x1,x2,y1,y2=bounds
    if padding:
        span=max(x2-x1,y2-y1);pad=max(.25,span*.18);cx=(x1+x2)/2;cy=(y1+y2)/2;w=max(2.,x2-x1+2*pad);h=max(2.,y2-y1+2*pad);x1=cx-w/2;x2=cx+w/2;y1=cy-h/2;y2=cy+h/2
    return C.QRectF(x1*12,-y2*12,(x2-x1)*12,(y2-y1)*12)






class HoverBox(W.QGraphicsRectItem):
    def __init__(self,rectangle,stroke):
        super().__init__(rectangle);self.base_pen=G.QPen(stroke);self.setPen(stroke);self.setAcceptHoverEvents(True)
    def hoverEnterEvent(self,event):
        p=G.QPen(self.base_pen);p.setColor(G.QColor('#b8efc6'));p.setWidthF(max(2.,p.widthF()));self.setPen(p);self.setZValue(5);super().hoverEnterEvent(event)
    def hoverLeaveEvent(self,event):self.setPen(self.base_pen);self.setZValue(0);super().hoverLeaveEvent(event)

class View(W.QGraphicsView):
    selected=C.pyqtSignal(str,int);coordinates=C.pyqtSignal(float,float);camera_changed=C.pyqtSignal();manual_pan=C.pyqtSignal()
    def __init__(self):
        super().__init__();self.setScene(W.QGraphicsScene(self));self.scene().setSceneRect(-100000,-100000,200000,200000);self.setBackgroundBrush(G.QColor('#191d1b'));self.setFrameShape(W.QFrame.NoFrame);self.setHorizontalScrollBarPolicy(C.Qt.ScrollBarAlwaysOff);self.setVerticalScrollBarPolicy(C.Qt.ScrollBarAlwaysOff);self.setDragMode(self.ScrollHandDrag);self.setRenderHint(G.QPainter.Antialiasing);self.setTransformationAnchor(self.AnchorUnderMouse);self.setMouseTracking(True);self.first=True;self.selected_id=None;self.selected_cluster=None;self.focus_only=False;self.visible_bounds=[];self.roi_bounds=[];self.press_pos=None;self.last_render={};self.hover_item=None;self.point_paths=[]
    def wheelEvent(self,e):
        factor=1.2 if e.angleDelta().y()>0 else 1/1.2
        if .02<self.transform().m11()*factor<1000:self.scale(factor,factor);self.update_point_size()
        self.camera_changed.emit()
    def mousePressEvent(self,e):self.press_pos=e.pos();super().mousePressEvent(e)
    def mouseMoveEvent(self,e):
        p=self.mapToScene(e.pos());self.coordinates.emit(p.x()/12,-p.y()/12);super().mouseMoveEvent(e)
        if e.buttons() & C.Qt.LeftButton and self.press_pos is not None and (e.pos()-self.press_pos).manhattanLength()>4:self.manual_pan.emit();self.camera_changed.emit()
    def mouseReleaseEvent(self,e):
        item=self.itemAt(e.pos())
        if item and item.data(0) and self.press_pos is not None and (e.pos()-self.press_pos).manhattanLength()<5:self.selected.emit(item.data(0),int(item.data(1)))
        super().mouseReleaseEvent(e)
    def reset_view(self):
        b=union(self.visible_bounds)
        if b:self.fitInView(rect(b),C.Qt.KeepAspectRatio);self.update_point_size()
    def update_point_size(self):
        zoom=self.transform().m11()
        for item,base in self.point_paths:
            stroke=item.pen();stroke.setWidthF(min(2.3,1.2+zoom*.035) if base>=2.5 else min(1.85,1.0+zoom*.025) if base>=1.9 else 1.05);item.setPen(stroke)
    def drawBackground(self,painter,r):
        super().drawBackground(painter,r)
        if not getattr(self,'grid',True):return
        zoom=max(.001,self.transform().m11());step=12*(1 if zoom>8 else 5 if zoom>1 else 20);pen=G.QPen(G.QColor('#2c352e'));pen.setCosmetic(True);painter.setPen(pen)
        for x in range(math.floor(r.left()/step),math.ceil(r.right()/step)+1):painter.drawLine(C.QPointF(x*step,r.top()),C.QPointF(x*step,r.bottom()))
        for y in range(math.floor(r.top()/step),math.ceil(r.bottom()/step)+1):painter.drawLine(C.QPointF(r.left(),y*step),C.QPointF(r.right(),y*step))
    def render_data(self,data,state,layers,angular=False):
        center=self.mapToScene(self.viewport().rect().center());transform=G.QTransform(self.transform());self.scene().clear();self.point_paths=[];scene=self.scene();self.visible_bounds=[];self.roi_bounds=[];self.grid=layers.get('Grid',True) and not angular;meta,links=evidence(data,state);cs={int(c['id']):c for c in data.get('clusters',[])};ts=tracks(state);focus_ids={self.selected_cluster} if self.selected_cluster is not None else links.get(self.selected_id,set());selected_ids=focus_ids
        if self.focus_only and self.selected_id is not None:
            selected_track=next((t for t in ts if int(t['id'])==self.selected_id),None)
            if selected_track is None or not track_visible(selected_track,layers):focus_ids=set();selected_ids=set()
        point_indices=[];shown_clusters=[];shown_tracks=[];paths={}
        def point(x,y):return C.QPointF(float(x)*12,-float(y)*12)
        def pen(color,width=1.,dashed=False):
            p=G.QPen(G.QColor(color));p.setCosmetic(True);p.setWidthF(width)
            if dashed:p.setStyle(C.Qt.DashLine)
            return p
        def visible_cluster(cid):return all(layers.get(tag,True) for tag in meta.get(cid,set())) and (not self.focus_only or cid in focus_ids)
        def tagged(item,kind,ident,tip):item.setData(0,kind);item.setData(1,int(ident));item.setToolTip(tip);return item
        if layers.get('Points',True):
            for p in (data.get('depth',[]) if angular else data.get('points',[])):
                if angular:x,y=p[:2];color='#b9c9b5';width=1.7
                else:
                    x,y=p[:2];owner=int(p[4]);group=int(p[5])
                    if not layers.get({0:'Outside ROI',1:'ROI non-foreground',2:'Foreground'}[group],True):continue
                    if group!=0 and not layers.get('ROI inside',True):continue
                    if owner>=0 and not visible_cluster(owner):continue
                    if self.focus_only and owner not in focus_ids:continue
                    selected=owner in selected_ids;color=COLORS['selected'] if selected else '#accdb0' if group==2 else '#747e76' if group==1 else '#a9afaa';width=2.5 if selected else 1.9 if group==2 else 1.35
                    if owner>=0 and 'Rejected components' in meta.get(owner,set()) and not selected:color=COLORS['rejected']
                    if selected_ids and (self.selected_id is not None or self.selected_cluster is not None):
                        if not selected:color='#647267' if group==2 else '#4d5850';width=1.25
                    point_indices.append(int(p[6]))
                path=paths.setdefault((color,width),G.QPainterPath());q=point(x,y);path.moveTo(q);path.lineTo(q+C.QPointF(.015,.015));self.visible_bounds.append((float(x),float(x),float(y),float(y)))
            for (color,width),path in paths.items():stroke=pen(color,width);stroke.setCapStyle(C.Qt.RoundCap);item=scene.addPath(path,stroke);item.setToolTip('Recorded current points · render LOD; no synthetic membership');self.point_paths.append((item,width))
        if not angular:
            if layers.get('ROI',True):
                for roi in data.get('roi',[]):
                    if not roi.get('enabled'):continue
                    if roi.get('polygon_x'):
                        xs,ys=roi['polygon_x'],roi['polygon_y'];self.roi_bounds.append((min(xs),max(xs),min(ys),max(ys)));scene.addPolygon(G.QPolygonF([point(x,y) for x,y in zip(xs,ys)]),pen('#59745e',1,True))
                    else:
                        b=tuple(map(float,roi['bounds']));self.roi_bounds.append(b);scene.addRect(rect(b,False),pen('#59745e' if roi.get('type')=='watch' else '#755e58',1,True))
            if layers.get('Clusters',True):
                for cid,c in cs.items():
                    if not visible_cluster(cid):continue
                    selected=cid in selected_ids;shown_clusters.append(cid);self.visible_bounds.append(cluster_box(c));color=COLORS['selected'] if selected else COLORS['rejected'] if 'Rejected components' in meta[cid] else COLORS['unknown'];bounds=cluster_box(c);outline=HoverBox(rect(bounds,False),pen(color,2.2 if selected else 1.1));scene.addItem(outline);outline=tagged(outline, 'cluster',cid,f"Cluster {cid} · {c['count']:,} actual points\n{', '.join(sorted(meta[cid])) or 'No recorded object classification'}\n{bounds}");outline.setAcceptHoverEvents(True)
                    if selected:
                        text=scene.addText(f'C{cid} · {c["count"]:,} points',G.QFont('Lato',10));text.setDefaultTextColor(G.QColor(color));text.setPos(point(bounds[0],bounds[3]));text.setFlag(W.QGraphicsItem.ItemIgnoresTransformations);tagged(text,'cluster',cid,outline.toolTip())
            for tr in ts:
                ident=int(tr['id']);f=flags(tr)
                if not track_visible(tr,layers) or (self.focus_only and ident!=self.selected_id):continue
                pos,source=position(tr,data,state)
                if not pos:continue
                selected=ident==self.selected_id;shown_tracks.append(ident);self.visible_bounds.append((pos[0],pos[0],pos[1],pos[1]));color=COLORS['dormant'] if f['dormant'] else COLORS['prediction'] if f['prediction'] else COLORS['observed'];q=point(*pos);tip=f"Track {ident} · {source}\nObservation: {f['actual']} · lifecycle: {tr.get('lifecycle','Not recorded')}\nRaw association_source: {tr.get('association_source','Not recorded')}"
                marker=scene.addEllipse(-5,-5,10,10,pen(color,2,not f['actual']));marker.setPos(q);marker.setFlag(W.QGraphicsItem.ItemIgnoresTransformations);tagged(marker,'track',ident,tip)
                if not selected:marker.setOpacity(.4)
                else:
                    text=scene.addText(f'T{ident} · {"observed" if f["actual"] else "prediction" if f["prediction"] else "dormant"}',G.QFont('Lato',10));text.setDefaultTextColor(G.QColor(color));text.setParentItem(marker);text.setPos(10,-13);text.setFlag(W.QGraphicsItem.ItemIgnoresTransformations);tagged(text,'track',ident,tip)
                    geom=tr.get('geometry',{});ob=tr.get('observation',{})
                    def box(values,col,dashed=False):
                        b=aabb(values)
                        if b:self.visible_bounds.append(b);tagged(scene.addRect(rect(b,False),pen(col,1.8,dashed)),'track',ident,tip)
                    if layers.get('Geometry',True):box(geom.get('aabb',[]),'#bbccba')
                    if layers.get('Memory Geometry',True) and tr.get('memory',{}).get('last_reliable_stamp',0)>0:box(geom.get('reliable_aabb',[]),COLORS['memory'],True)
                    if layers.get('KF Prediction',True) and tr.get('kf',{}).get('initialized') and len(ob.get('predicted',[]))==2 and all(finite(v) for v in ob['predicted']):
                        pred=point(*ob['predicted']);path=G.QPainterPath();path.moveTo(-6,-6);path.lineTo(6,6);path.moveTo(-6,6);path.lineTo(6,-6);cross=scene.addPath(path,pen(COLORS['prediction'],2));cross.setPos(pred);cross.setFlag(W.QGraphicsItem.ItemIgnoresTransformations);self.visible_bounds.append((float(ob['predicted'][0]),float(ob['predicted'][0]),float(ob['predicted'][1]),float(ob['predicted'][1])))
                    if ob.get('measurement_valid') and len(ob.get('measurement',[]))==2:
                        m=scene.addEllipse(-4,-4,8,8,pen('#f0f1e8',1.8));m.setPos(point(*ob['measurement']));m.setFlag(W.QGraphicsItem.ItemIgnoresTransformations)
            if layers.get('FOV',True) and not self.focus_only and data:
                origin=data.get('sensor_origin',[0,0]);item=scene.addEllipse(-3,-3,6,6,pen('#d9dfd2',1.5));item.setPos(point(*origin[:2]));item.setFlag(W.QGraphicsItem.ItemIgnoresTransformations);item.setToolTip('Recorded LiDAR origin; no inferred surface visibility')
        self.last_render={'point_indices':point_indices,'point_count':len(point_indices),'cluster_ids':shown_clusters,'track_ids':shown_tracks,'focus_cluster_ids':sorted(focus_ids),'verified_track_cluster_links':{str(k):sorted(v) for k,v in links.items()}}
        self.setTransform(transform);self.centerOn(center);self.update_point_size()
        if self.first and data:
            self.reset_view();self.first=False


class LifecycleTimeline(W.QWidget):
    seek=C.pyqtSignal(int);range_selected=C.pyqtSignal(int,int)
    def __init__(self):
        super().__init__();self.setMinimumHeight(158);self.setMaximumHeight(158);self.setMouseTracking(True);self.result={};self.rows=[];self.frame=1;self.drag=None;self.hit=[]
    def set_data(self,result,rows):self.result=result;self.rows=rows;self.update()
    def x(self,frame):return 125+(self.width()-143)*(frame-1)/max(1,len(self.rows)-1)
    def at(self,x):return min(max(1,round(1+(x-125)/max(1,self.width()-143)*max(1,len(self.rows)-1))),len(self.rows) or 1)
    def paintEvent(self,event):
        p=G.QPainter(self);p.setRenderHint(G.QPainter.Antialiasing);p.setFont(G.QFont('Lato',9));self.hit=[]
        for i,(label,name,color) in enumerate(LANES):
            y=5+i*22;p.setPen(G.QColor('#787e72'));p.drawText(C.QRectF(0,y,120,18),C.Qt.AlignVCenter,label);p.setPen(G.QColor('#e7e9e0'));p.drawLine(C.QPointF(125,y+9),C.QPointF(self.width()-18,y+9))
            for segment in self.result.get('intervals',[]):
                if segment['lane']!=name:continue
                r=C.QRectF(self.x(segment['start']),y+4,max(3,self.x(segment['end'])-self.x(segment['start'])+2),10);p.setPen(C.Qt.NoPen);p.setBrush(G.QColor(color));p.drawRoundedRect(r,2,2);self.hit.append((r,segment))
            for ev in self.result.get('events',[]):
                if ev['kind']==name:
                    r=C.QRectF(self.x(ev['frame'])-4,y+5,8,8);p.setBrush(G.QColor(color));p.setPen(C.Qt.NoPen);p.drawEllipse(r);self.hit.append((r,ev))
        p.setPen(G.QPen(G.QColor('#4c6c53'),1));p.drawLine(C.QPointF(self.x(self.frame),0),C.QPointF(self.x(self.frame),136))
        if self.rows:
            p.setPen(G.QColor('#82877b'));p.drawText(C.QRectF(125,138,120,18),C.Qt.AlignLeft,f"{self.rows[0]['relative_sec']:.2f}s");p.drawText(C.QRectF(self.width()-138,138,120,18),C.Qt.AlignRight,f"{self.rows[-1]['relative_sec']:.2f}s")
        if self.drag:
            a,b=self.drag;p.fillRect(C.QRectF(min(a,b),0,abs(a-b),136),G.QColor(90,131,98,32))
    def mousePressEvent(self,event):self.drag=(event.x(),event.x())
    def mouseMoveEvent(self,event):
        if self.drag:self.drag=(self.drag[0],event.x());self.update()
        else:
            for r,segment in self.hit:
                if r.adjusted(-2,-2,2,2).contains(event.pos()):
                    if 'start' in segment:tip=f"{segment['lane']} · F{segment['start']}–{segment['end']} · +{segment['start_sec']:.3f}–{segment['end_sec']:.3f}s"
                    else:tip=f"F{segment['frame']} · {segment['source']}"
                    if segment.get('lane')=='missed':tip+='\nMissed observation counter; canonical LOST not serialized.'
                    W.QToolTip.showText(event.globalPos(),tip,self);break
    def mouseReleaseEvent(self,event):
        if self.drag:
            a,b=self.drag;self.drag=None
            if abs(a-b)>5:self.range_selected.emit(*sorted((self.at(a),self.at(b))))
            else:self.seek.emit(self.at(b))
            self.update()
