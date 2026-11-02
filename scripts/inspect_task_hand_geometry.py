"""Report collision-mesh AABBs in the local wrist frame; these are not box shapes."""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import trimesh
from task_hand_control import model_joints, POSTURES
from workbench_model import OfflineModel


def inspect(path):
    model=OfflineModel(path)
    result={'urdf':str(Path(path).resolve()),'units':'mm','bounds_frame':'local wrist_yaw_link',
            'note':'Axis-aligned envelope of actual collision geometry, NOT a configured box','hands':{}}
    cache={}
    for side in ('left','right'):
        articulated=model.robot.find(f"joint[@name='{side}_middle_1_joint']") is not None
        records={}
        for posture in (POSTURES if articulated else ('loaded_static',)):
            q=model_joints(side,POSTURES[posture]) if articulated else {}
            inv=np.linalg.inv(model.transform(side+'_wrist_yaw_link',q))
            parts={}; hand=[];assembly=[]
            for link in model.robot.findall('link'):
                name=link.get('name')
                if not name.startswith(tuple(side+'_'+part for part in ('rubber_hand','hand_','palm','index_',
                        'thumb_','middle_','ring_','little_','payload_collision'))):continue
                vertices=[]
                for collision in link.findall('collision'):
                    geometry=collision.find('geometry');mesh=geometry.find('mesh');cylinder=geometry.find('cylinder')
                    if mesh is not None:
                        filename=mesh.get('filename').removeprefix('file://')
                        if filename not in cache:cache[filename]=trimesh.load(filename,force='mesh',process=False).vertices
                        v=cache[filename]*np.array(list(map(float,mesh.get('scale','1 1 1').split())))
                    elif cylinder is not None:
                        v=trimesh.creation.cylinder(radius=float(cylinder.get('radius')),
                            height=float(cylinder.get('length')),sections=128).vertices
                    elif geometry.find('box') is not None:
                        v=trimesh.creation.box(extents=list(map(float,geometry.find('box').get('size').split()))).vertices
                    else:raise ValueError('Unsupported collision geometry for '+name)
                    transform=inv@model.transform(name,q)
                    origin=collision.find('origin')
                    if origin is not None:
                        local=np.eye(4);local[:3,:3]=Rotation.from_euler('xyz',list(map(float,origin.get('rpy','0 0 0').split()))).as_matrix()
                        local[:3,3]=list(map(float,origin.get('xyz','0 0 0').split()));transform=transform@local
                    vertices.extend(v@transform[:3,:3].T+transform[:3,3])
                if not vertices:continue
                v=np.asarray(vertices);lo,hi=v.min(axis=0),v.max(axis=0)
                parts[name]={'min_mm':(lo*1000).tolist(),'max_mm':(hi*1000).tolist(),
                             'extent_mm':((hi-lo)*1000).tolist()}
                assembly.extend(v)
                if name!=side+'_payload_collision':hand.extend(v)
            def bounds(v):
                v=np.asarray(v)
                return {'extent_mm':((v.max(axis=0)-v.min(axis=0))*1000).tolist(),
                        'min_mm':(v.min(axis=0)*1000).tolist(),'max_mm':(v.max(axis=0)*1000).tolist()}
            records[posture]={'hand':bounds(hand),'hand_and_connector':bounds(assembly),'parts':parts}
        result['hands'][side]={'articulated':articulated,'postures':records}
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('urdf',type=Path)
    parser.add_argument('--output',type=Path);args=parser.parse_args()
    report=inspect(args.urdf)
    if args.output:args.output.write_text(json.dumps(report,indent=2)+'\n')
    for side,hand in report['hands'].items():
        for posture,record in hand['postures'].items():
            print(side,posture,'hand mm:',np.round(record['hand']['extent_mm'],2),
                  'with connector mm:',np.round(record['hand_and_connector']['extent_mm'],2))
