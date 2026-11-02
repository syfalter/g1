"""Spatial denoising candidates with continuous joint and conservative TCP bounds."""
import copy
import math

import numpy as np
from scipy.interpolate import make_interp_spline
from scipy.ndimage import gaussian_filter1d
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint

from task_trajectory import coefficients,time_seconds,peak


def evaluate(trajectory,times,order=0):
    knots=np.array([time_seconds(p) for p in trajectory.points])
    owners=np.clip(np.searchsorted(knots,times,side='right')-1,0,len(knots)-2)
    values=np.empty((len(times),len(trajectory.joint_names)))
    for i in np.unique(owners):
        selected=owners==i;dt=knots[i+1]-knots[i]
        poly=coefficients(trajectory.points[i],trajectory.points[i+1],dt)
        values[selected]=np.polynomial.polynomial.polyval((times[selected]-knots[i])/dt,
            np.polynomial.polynomial.polyder(poly,order)).T/dt**order
    return values


def deviation_bounds(reference,candidate,model,link):
    """Same-clock curves, equal knots. Rotation uses a triangle-inequality bound."""
    if reference.joint_names!=candidate.joint_names or len(reference.points)!=len(candidate.points):
        raise ValueError('Deviation requires aligned trajectory knots')
    delta=np.zeros(len(reference.joint_names))
    for a,b,c,d in zip(reference.points,reference.points[1:],candidate.points,candidate.points[1:]):
        if time_seconds(a)!=time_seconds(c) or time_seconds(b)!=time_seconds(d):
            raise ValueError('Deviation requires equal times')
        dt=time_seconds(b)-time_seconds(a)
        difference=coefficients(a,b,dt)-coefficients(c,d,dt)
        delta=np.maximum(delta,[peak(axis) for axis in difference.T])
    return bounds_from_joint_delta(delta,reference.joint_names,model,link)


def bounds_from_joint_delta(delta,names,model,link):
    columns={name:i for i,name in enumerate(names)}
    radius=0.;position=0.;rotation=0.
    while link in model.joints:
        parent,kind,origin,_,_,name,multiplier,_=model.joints[link]
        if name in columns:
            amount=abs(multiplier)*delta[columns[name]]
            if kind in ('revolute','continuous'):position+=radius*amount;rotation+=amount
            elif kind=='prismatic':position+=amount
        radius+=float(np.linalg.norm(origin[:3,3]))
        if kind=='prismatic':
            if name not in model.limits:raise ValueError('Missing prismatic travel bound')
            radius+=max(abs(v) for v in model.limits[name])
        link=parent
    return dict(joint_rad=float(max(delta)),tcp_m=float(position),orientation_rad=float(rotation))


def smooth_candidate(source,model,side,*,joint_tolerance=.003,tcp_tolerance=.001,
                     orientation_tolerance=math.radians(.5),window=.02,check=lambda:None):
    from task_composition_preview import stamp,check_positions
    values=(joint_tolerance,tcp_tolerance,orientation_tolerance,window)
    if not all(math.isfinite(v) and v>0 for v in values):raise ValueError('Invalid smoothing bounds')
    duration=time_seconds(source.points[-1])
    count=max(32,math.ceil(duration/.01)+1)
    if count>60000:raise ValueError('Smoothing sample budget exceeded')
    times=np.linspace(0.,duration,count);q=evaluate(source,times)
    arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(q,axis=0),axis=1))]
    if arc[-1]<1e-8:
        return copy.deepcopy(source),dict(applied=False,reason='stationary',joint_rad=0.,tcp_m=0.,orientation_rad=0.)
    keep=np.r_[True,np.diff(arc)>1e-12]
    grid=np.linspace(0.,arc[-1],min(20000,max(32,math.ceil(arc[-1]/(window/6))+1)))
    rows=np.column_stack([np.interp(grid,arc[keep],axis[keep]) for axis in q.T])
    filtered=gaussian_filter1d(rows,window/(grid[1]-grid[0]),axis=0,mode='nearest')
    source_knots=np.array([time_seconds(p) for p in source.points])
    indices=np.unique(np.rint(np.linspace(0,len(source_knots)-1,
        min(len(source_knots),max(12,math.ceil(duration/.12)+1)))).astype(int))
    knots=source_knots[indices]
    progress=np.interp(knots,times,arc)
    target=np.column_stack([np.interp(progress,grid,axis) for axis in filtered.T])
    target[0]=source.points[0].positions;target[-1]=source.points[-1].positions
    spline=make_interp_spline(knots,target,k=5,bc_type=(
        [(1,np.array(source.points[0].velocities)),(2,np.array(source.points[0].accelerations))],
        [(1,np.array(source.points[-1].velocities)),(2,np.array(source.points[-1].accelerations))]))
    # Use existing breakpoints: near-coincident new knots amplify timestamp roundoff.
    if not set(spline.t)<=set(source_knots):raise ValueError('Unexpected smoothing spline knots')
    knots=source_knots
    reference=JointTrajectory(joint_names=list(source.joint_names))
    proposal=JointTrajectory(joint_names=list(source.joint_names))
    old=[evaluate(source,knots,k) for k in range(3)]
    new=[spline(knots,nu=k) for k in range(3)]
    for i,t in enumerate(knots):
        reference.points.append(stamp(JointTrajectoryPoint(positions=old[0][i].tolist(),
            velocities=old[1][i].tolist(),accelerations=old[2][i].tolist()),t))
        proposal.points.append(stamp(JointTrajectoryPoint(positions=new[0][i].tolist(),
            velocities=new[1][i].tolist(),accelerations=new[2][i].tolist()),t))
    check();bounds=deviation_bounds(reference,proposal,model,side+'_index_tcp')
    alpha=min(1.,*(limit/max(bounds[key],1e-15) for key,limit in zip(
        ('joint_rad','tcp_m','orientation_rad'),values[:3])))*.98
    # Convex blending scales the full continuous difference, not just sampled errors.
    for a,b in zip(reference.points,proposal.points):
        for key in ('positions','velocities','accelerations'):
            x=np.asarray(getattr(a,key));y=np.asarray(getattr(b,key))
            setattr(b,key,(x+alpha*(y-x)).tolist())
    for i in (0,-1):
        for key in ('positions','velocities','accelerations'):
            setattr(proposal.points[i],key,list(getattr(source.points[i],key)))
    check();bounds=deviation_bounds(reference,proposal,model,side+'_index_tcp')
    for key,limit in zip(('joint_rad','tcp_m','orientation_rad'),values[:3]):
        if bounds[key]>limit+1e-9:raise ValueError('Smoothing deviation exceeds bound: '+key)
    check_positions(proposal,model)
    return proposal,dict(applied=True,**bounds,blend=alpha,window_joint_arc_rad=window,
        joint_tolerance_rad=joint_tolerance,tcp_tolerance_m=tcp_tolerance,
        orientation_tolerance_rad=orientation_tolerance,endpoints_and_derivatives_preserved=True,
        bound_policy='continuous_joint_polynomial_and_global_chain_radius',hardware_executable=False)
