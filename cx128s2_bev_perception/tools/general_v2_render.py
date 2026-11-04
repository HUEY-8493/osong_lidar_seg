"""Render current-frame Raw evidence; invoked before owned scratch is removed."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image, ImageDraw


def render(previews: Path, output: Path):
    index = json.loads((previews/'index.json').read_text())
    frames = output/'frames'
    frames.mkdir(exist_ok=True)
    boxes = [o for e in index for o in e['objects'] if o['confirmed']]
    if boxes:
        xmin = min(o['x']-max(o['length'], o['width'])/2 for o in boxes)-5
        xmax = max(o['x']+max(o['length'], o['width'])/2 for o in boxes)+5
        ymin = min(o['y']-max(o['length'], o['width'])/2 for o in boxes)-5
        ymax = max(o['y']+max(o['length'], o['width'])/2 for o in boxes)+5
    else:
        xmin, xmax, ymin, ymax = -30, 40, -20, 20
    images = []
    for event in index:
        raw = np.fromfile(previews/event['raw_file'], dtype=np.dtype([('x','=f4'),('y','=f4'),('z','=f4'),('owner','=i4')]))
        fig, axes = plt.subplots(1, 3, figsize=(18, 6))
        for ax, (h, v), bounds in zip(axes, [('x','y'),('x','z'),('y','z')], [(xmin,xmax,ymin,ymax),(xmin,xmax,-2,7),(ymin,ymax,-2,7)]):
            for mask, color in [(raw['owner']==-1, '#999999'),(raw['owner']==-2, '#a47745'),(raw['owner']==-4, '#e77f17'),(raw['owner']>=0, '#00ad39')]:
                # Every point in the temporary original frame is supplied to the renderer.
                ax.scatter(raw[h][mask], raw[v][mask], s=.4, c=color, rasterized=True)
            for obj in event['objects']:
                x,y,z = obj['x'],obj['y'],obj['z']
                length,width,height,yaw = obj['length'],obj['width'],obj['height'],obj['yaw']
                if h == 'x' and v == 'y':
                    local=np.array([[-length/2,-width/2],[length/2,-width/2],[length/2,width/2],[-length/2,width/2],[-length/2,-width/2]])
                    rotation=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
                    line=local@rotation.T+np.array([x,y])
                else:
                    span=(abs(np.cos(yaw))*length+abs(np.sin(yaw))*width) if h=='x' else (abs(np.sin(yaw))*length+abs(np.cos(yaw))*width)
                    center=x if h=='x' else y
                    line=np.array([[center-span/2,z-height/2],[center+span/2,z-height/2],[center+span/2,z+height/2],[center-span/2,z+height/2],[center-span/2,z-height/2]])
                ax.plot(line[:,0],line[:,1], '--' if obj['extent_locked'] or obj['observation']=='HELD' else '-', c='#2942ab',lw=1)
                ax.text(line[0,0],line[0,1],str(obj['id']),fontsize=8,color='#2222aa')
            ax.set(xlim=bounds[:2],ylim=bounds[2:],xlabel=h+' [m]',ylabel=v+' [m]')
            ax.grid(alpha=.2)
        detail=' | '.join(f"ID {o['id']} {o['observation']} {o['status']} L={o['length']:.2f} rawL={o['observed_length']:.2f} n={o['points']}" for o in event['objects'][:4])
        fig.suptitle(f"{output.name} / Frame {event['frame']} / +{event['relative_sec']:.2f}s / sensor_ns={event['stamp_ns']}\n{event['quality']} ground={event['ground']['reason']}\n{detail}\nGreen=current assigned Raw; gray=other Raw; brown=ground-filtered Raw; orange=excluded candidate Raw; blue dashed=retained/predicted model. Common cropped view; no point decimation.",fontsize=9)
        fig.tight_layout(rect=(0,0,1,.84))
        path=frames/f"frame_{event['frame']:06d}.png"
        fig.savefig(path,dpi=150)
        plt.close(fig)
        images.append((event['frame'],path))
    if images:
        thumb_w, thumb_h, columns = 720, 260, 2
        rows=(len(images)+columns-1)//columns
        sheet=Image.new('RGB',(columns*thumb_w,rows*thumb_h),'white')
        draw=ImageDraw.Draw(sheet)
        for i,(frame,path) in enumerate(images):
            with Image.open(path) as source:
                source.thumbnail((thumb_w,thumb_h-20))
                pos=((i%columns)*thumb_w,(i//columns)*thumb_h)
                sheet.paste(source,pos)
                draw.text((pos[0]+5,pos[1]+thumb_h-18),f'Frame {frame}',fill='black')
        sheet.save(output/'contact_sheet.png')
    observations={}
    with (output/'replay_trace.jsonl').open() as trace:
        for line in trace:
            event=json.loads(line)
            for obj in event['objects']:
                if not obj['vehicle']:
                    continue
                observations.setdefault(obj['id'],[]).append((event['relative_sec'],obj['length'],obj['observed_length'],obj['status'],obj['observation']))
    fig,axes=plt.subplots(3,1,figsize=(15,9),sharex=True)
    states={'entering':0,'stopping':1,'stopped':2,'exiting':3}
    for ident,rows in observations.items():
        t=[r[0] for r in rows]
        axes[0].plot(t,[r[1] for r in rows],label=f'ID {ident} retained')
        direct=[r for r in rows if r[4]=='DIRECT']
        axes[0].plot([r[0] for r in direct],[r[2] for r in direct],':',alpha=.6)
        axes[1].scatter(t,[states[r[3]] for r in rows],s=3,label=f'ID {ident}')
        axes[2].scatter(t,[1 if r[4]=='DIRECT' else 0 for r in rows],s=3)
    axes[0].set_ylabel('Length [m] solid=model / dotted=Raw')
    axes[1].set_yticks(list(states.values()),list(states))
    axes[2].set_yticks([0,1],['HELD','DIRECT'])
    axes[2].set_xlabel('Sensor-relative seconds')
    if observations:
        axes[0].legend(ncol=4,fontsize=7)
    for ax in axes: ax.grid(alpha=.25)
    fig.suptitle(f'{output.name} — GeneralV2; automatic evidence, not ground truth')
    fig.tight_layout()
    fig.savefig(output/'timeline.png',dpi=150)
    plt.close(fig)
    links=''.join(f'<a href="frames/{p.name}"><img loading="lazy" src="frames/{p.name}" width="900" alt="Frame {n}"></a><br>' for n,p in images)
    (output/'visual_review.html').write_text('<!doctype html><meta charset="utf-8"><title>GeneralV2 Raw review</title><h1>'+output.name+'</h1><p>Current Raw evidence and separately retained geometry. Labels are algorithm outputs, not independent ground truth.</p><img src="timeline.png" width="1000"><br>'+links)
