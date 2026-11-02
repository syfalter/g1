"""Continuous free-space TCP route through the physical center's overhead point."""
import math
import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def approach_geometry(initial,pre,center,height,line_points,step_deg,reference_rotation=None):
    initial=np.asarray(initial);pre=np.asarray(pre);center=np.asarray(center)
    via=center+np.array([0.,0.,height])
    # Keep the overhead leg no lower than the chosen preinsert point.
    via[2]=max(via[2],pre[2,3])
    vertices=np.array([initial[:3,3],via,pre[:3,3]])
    delta=np.diff(vertices,axis=0);lengths=np.linalg.norm(delta,axis=1)
    if not np.isfinite(vertices).all() or np.min(lengths)<1e-6:
        raise ValueError('Approach needs two nonzero finite Cartesian legs')
    progress=np.r_[0.,np.cumsum(lengths)]
    rotation=Rotation.from_matrix(np.array([initial[:3,:3],pre[:3,:3]]))
    angle=math.degrees((rotation[0].inv()*rotation[1]).magnitude())
    interpolation=Slerp([0.,progress[-1]],rotation)

    def project(points):
        points=np.atleast_2d(points)
        fractions=np.clip(np.einsum('nli,li->nl',points[:,None,:]-vertices[:-1],delta)/lengths**2,0.,1.)
        nearest=vertices[:-1]+fractions[...,None]*delta
        distances=np.linalg.norm(points[:,None,:]-nearest,axis=2)
        leg=np.argmin(distances,axis=1);rows=np.arange(len(points))
        return distances[rows,leg],progress[leg]+fractions[rows,leg]*lengths[leg]

    def curve(point):return float(project(point)[0][0])
    curve.batch=lambda points:project(points)[0]
    def rotations(points):
        distances=project(points)[1]
        if reference_rotation is not None:
            return np.asarray([reference_rotation(float(s/progress[-1])) for s in distances])
        return interpolation(distances).as_quat()
    def nominal(point):return rotations(point)[0]
    nominal.batch=rotations
    nominal.fraction=lambda point:float(project(point)[1][0]/progress[-1])
    points=[]
    for i,length in enumerate(lengths):
        count=max(line_points,math.ceil(length/.004)+1,
                  math.ceil(angle*length/progress[-1]/(step_deg*.5))+1)
        points.extend(np.linspace(vertices[i],vertices[i+1],count)[1:])
    record=dict(method='cartesian_center_above_polyline',physical_center_m=center.tolist(),
                center_above_m=via.tolist(),vertices_m=vertices.tolist(),points=len(points),
                reference_frame='planner_fixed_frame',hand_posture='unchanged_until_existing_hand_stage',
                tcp='virtual_index_only_tcp_during_open_hand_approach',
                path_length_m=float(progress[-1]),orientation_transition_deg=angle)
    return points,nominal,curve,record
