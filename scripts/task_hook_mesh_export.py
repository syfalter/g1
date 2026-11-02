"""Export conservative closed convex collision shapes using the CAD environment."""
import json
import sys
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
from task_model_resources import resolve_mesh


def export(path):
    shapes=[]
    for link in ET.parse(path).getroot().findall('link'):
        name=link.get('name')
        body=name in ('torso_link','pelvis_contour_link','waist_collision','head_link')
        arm=name.startswith(('left_','right_')) and any(part in name for part in (
            'thumb','index','middle','ring','little','hand','connector','palm','payload_collision',
            'elbow_link','wrist_roll_link','wrist_pitch_link','shoulder_yaw_link'))
        if not (body or arm):continue
        for collision in link.findall('collision'):
            origin=collision.find('origin');mesh=collision.find('geometry/mesh')
            record={'link':name,'xyz':[0.,0.,0.],'rpy':[0.,0.,0.]}
            if origin is not None:
                for key in ('xyz','rpy'):record[key]=list(map(float,origin.get(key,'0 0 0').split()))
            if mesh is not None:
                data=trimesh.load(resolve_mesh(mesh.get('filename'),path),force='mesh')
                data.apply_scale(list(map(float,mesh.get('scale','1 1 1').split())))
                hull=data.convex_hull
                record.update(kind='convex',vertices=hull.vertices.tolist(),faces=hull.faces.tolist())
            else:
                primitive=list(collision.find('geometry'))[0]
                record.update(kind=primitive.tag,parameters=dict(primitive.attrib))
            shapes.append(record)
    if not shapes:raise ValueError('No hand collision shapes')
    return shapes


if __name__=='__main__':print(json.dumps(export(sys.argv[1])))
