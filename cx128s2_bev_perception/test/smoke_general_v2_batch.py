#!/usr/bin/env python3
"""Synthetic-only end-to-end check: two temporary bags, actual replay, images, cleanup.
Never reads, copies or deletes a field/NAS rosbag.
"""
import argparse
import json
import struct
import subprocess
import sys
import tempfile
from pathlib import Path
import rosbag2_py
from rclpy.serialization import serialize_message
from sensor_msgs.msg import PointCloud2, PointField
import yaml


def points(length=8.0, partial=False):
    values=[(ix*.4,iy*.4,0.0) for ix in range(-30,61) for iy in range(-20,21)]
    extent=4.0 if partial else length
    cx=6.0 if partial else 8.0
    nx=int(round(extent/.2))
    for k in range(14):
        for i in range(nx+1):
            for j in range(14):
                if k!=13 and i not in (0,nx) and j not in (0,13):
                    continue
                values.append((cx-extent/2+i*.2,-1.3+j*.2,.25+k*.2))
    return values


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build-dir',type=Path,required=True)
    parser.add_argument('--artifacts',type=Path,required=True)
    args=parser.parse_args()
    pkg=Path(__file__).resolve().parents[1]
    args.artifacts.mkdir(parents=True,exist_ok=True)
    params=args.artifacts/'synthetic_params.yaml'
    params.write_text(yaml.safe_dump({'/**':{'ros__parameters':{'use_roi_policy':False,'target_frame':'world','input_topic':'/ch/lslidar_point_cloud','background_mode':'ground_only'}}}))
    with tempfile.TemporaryDirectory(prefix='synthetic-general-v2-',dir=args.artifacts) as temporary:
        root=Path(temporary)
        for index in (1,2):
            bag=root/f'rosbag2_2099_01_01-00_00_0{index}'
            writer=rosbag2_py.SequentialWriter()
            writer.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
            writer.create_topic(rosbag2_py.TopicMetadata(id=0,name='/ch/lslidar_point_cloud',type='sensor_msgs/msg/PointCloud2',serialization_format='cdr'))
            for frame in range(40):
                xyz=[] if index==2 and frame>=35 else points(8 if index==1 else 12,index==1 and frame>=25)
                msg=PointCloud2()
                msg.header.frame_id='world'
                msg.header.stamp.sec=1000+frame//10
                msg.header.stamp.nanosec=(frame%10)*100_000_000
                msg.height=1;msg.width=len(xyz);msg.is_bigendian=False;msg.point_step=12;msg.row_step=msg.width*12;msg.is_dense=True
                msg.fields=[PointField(name=name,offset=offset,datatype=PointField.FLOAT32,count=1) for name,offset in [('x',0),('y',4),('z',8)]]
                msg.data=b''.join(struct.pack('<fff',*p) for p in xyz)
                writer.write('/ch/lslidar_point_cloud',serialize_message(msg),msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec)
            writer.close()
        subprocess.run([sys.executable,str(pkg/'tools/general_batch_review.py'),'--engine','v2','--source',str(root),'--results',str(args.artifacts/'review'),'--scratch',str(args.artifacts/'scratch'),'--build-dir',str(args.build_dir),'--params',str(params)],check=True)
        result=json.loads((args.artifacts/'review/summary.json').read_text())
        assert len(result)==2 and all(x['engine']=='GeneralV2' and x['frames']==40 and x['output_frames']==40 for x in result)
        assert all(x['state']=='COMPLETED_UNREVIEWED' for x in result)
        for item in result:
            bag=args.artifacts/'review'/item['bag']
            assert (bag/'contact_sheet.png').is_file() and (bag/'timeline.png').is_file() and (bag/'visual_review.html').is_file()
            assert (root/item['bag']/'metadata.yaml').is_file(), 'Source fixtures must not be deleted by the batch runner'
        assert not list((args.artifacts/'scratch').glob('owned-general-*')), 'Owned temporary copies must be removed'
        # Check the actual unchanged bridge projection on real V2 replay DTOs.
        sys.path.insert(0,str(pkg.parent/'guardian_lidar_bridge/guardian_lidar_bridge'))
        from state_projection import FirstVehicleSelector, general_summary
        from types import SimpleNamespace as NS
        rows=[]
        for item in result:
            stream=args.artifacts/'review'/item['bag']/'replay_vehicles.jsonl'
            rows.append(next(json.loads(line) for line in stream.read_text().splitlines() if json.loads(line)['vehicles']))
        vehicles=[NS(id=i+1,dimensions=NS(x=row['vehicles'][0]['length']),age=row['vehicles'][0]['age'],distance=row['vehicles'][0]['distance'],vehicle_status=row['vehicles'][0]['status'],motion_state='STATIONARY') for i,row in enumerate(rows)]
        selected=FirstVehicleSelector().select(vehicles)
        assert selected.id==2, 'The longest vehicle wins regardless of distance/order'
        row=rows[1]
        payload=general_summary(vehicles,vehicles,vehicles,vehicles,row['stamp_sec']*1000+row['stamp_nanosec']//1_000_000,selected)
        assert payload['vehicle_length']==selected.dimensions.x and payload['vehicle_distance']==selected.distance
        assert payload['vehicle_status'] in ('entering','stopping','stopped','exiting')
        (args.artifacts/'bridge_projection.json').write_text(json.dumps(payload,indent=2))
    print('PASS: synthetic two-bag Raw replay, current sensor timestamps, projection, visual artifacts, source preservation and temporary-copy cleanup')

if __name__=='__main__':
    main()
