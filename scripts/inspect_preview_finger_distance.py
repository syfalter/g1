"""Sample non-index collision-mesh vertices versus the axis-center point, not base clearance."""
import argparse
import json
from pathlib import Path
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation
from task_preview_playback import PreviewTimeline
from task_kinematics import capture_transform
from workbench_model import OfflineModel


def inspect(path,step=.1):
    report=json.loads(path.read_text());root=path.parent;side=report['arm']
    start=json.loads((root/'reconstructed_state.json').read_text())['joints']
    timeline=PreviewTimeline(report,start);model=OfflineModel(root/'robot.urdf')
    geometry_path=root/'preview_geometry.json'
    geometry=json.loads((geometry_path if geometry_path.exists() else Path(report['capture'])/'geometry.json').read_text())
    frozen=capture_transform(geometry,model,start)
    center=frozen[:3,:3]@np.asarray(geometry['task_preview']['axis_center_m'])+frozen[:3,3]
    first=next(p for p in timeline.parts if p[-1]=='Insert');posture=timeline.sample(first[0]+.01)[0]
    inverse=np.linalg.inv(model.transform(side+'_wrist_yaw_link',posture))
    vertices=[];names=[]
    for link in model.robot.findall('link'):
        name=link.get('name')
        if not name.startswith(tuple(side+'_'+part for part in ('thumb_','middle_','ring_','little_'))):continue
        for collision in link.findall('collision'):
            mesh=collision.find('geometry/mesh')
            if mesh is None:continue
            data=trimesh.load(mesh.get('filename').removeprefix('file://'),force='mesh',process=False)
            points=data.vertices*np.array(list(map(float,mesh.get('scale','1 1 1').split())))
            origin=collision.find('origin');local=np.eye(4)
            if origin is not None:
                local[:3,:3]=Rotation.from_euler('xyz',list(map(float,origin.get('rpy','0 0 0').split()))).as_matrix()
                local[:3,3]=list(map(float,origin.get('xyz','0 0 0').split()))
            transform=inverse@model.transform(name,posture)@local
            points=points@transform[:3,:3].T+transform[:3,3]
            vertices.extend(points);names.extend([name]*len(points))
    vertices=np.asarray(vertices);minimum=float('inf');worst=None;samples=0
    contact={'Insert','Left 120','Right 120','Return center','Withdraw'}
    for t in np.arange(0,timeline.duration,step):
        q,label=timeline.sample(t)
        if label not in contact:continue
        transform=model.transform(side+'_wrist_yaw_link',q)
        points=vertices@transform[:3,:3].T+transform[:3,3]
        distances=np.linalg.norm(points-center,axis=1);index=int(np.argmin(distances));samples+=1
        if distances[index]<minimum:
            minimum=float(distances[index]);worst={'link':names[index],'phase':label,'time_s':float(t),
                'point_pelvis_m':points[index].tolist(),'axis_center_pelvis_m':center.tolist()}
    return {'robot_raise_m':report.get('robot_raise_m',0.),'source':str(path.resolve()),
        'minimum_mesh_vertex_distance_to_center_m':minimum,'worst':worst,'sample_count':samples,
        'sample_interval_s':step,'collision_checked':False,
        'note':'Sampled mesh vertices to a POINT, not signed base-surface clearance; base dimensions unknown'}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('reports',type=Path,nargs='+')
    args=parser.parse_args()
    for path in args.reports:
        result=inspect(path)
        (path.parent/'finger_center_distance.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result),flush=True)
