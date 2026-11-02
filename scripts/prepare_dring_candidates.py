"""Offline candidate generation from preserved demonstrations; no robot clients."""
import argparse
import copy
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.interpolate import CubicSpline
from scipy.optimize import least_squares
from scipy.signal import savgol_filter
from scipy.spatial.transform import Rotation

from task_dring_teach_data import arrays,moving
from task_screen_teach import fingerprint,write_json,now
from workbench_model import OfflineModel

ROOT=Path(__file__).resolve().parents[1]
ARCHIVE=ROOT/'data/dring_preserved/20260920_left3_right1'


def blend(x):
    x=np.clip(x,0.,1.)
    return x**3*(10.-15.*x+6.*x*x)


def circle_fit(points,times):
    low=points[:,2].min()
    mask=points[:,2]<low+.008
    xy=points[mask,:2]
    linear=np.linalg.lstsq(np.c_[2*xy,np.ones(len(xy))],np.sum(xy*xy,axis=1),rcond=None)[0]
    seed=np.r_[linear[:2],np.mean(np.linalg.norm(xy-linear[:2],axis=1))]
    fit=least_squares(lambda v:np.linalg.norm(xy-v[:2],axis=1)-v[2],seed,loss='soft_l1',f_scale=.001)
    center=fit.x[:2];radius=float(fit.x[2])
    error=np.linalg.norm(xy-center,axis=1)-radius
    angle=np.unwrap(np.arctan2(xy[:,1]-center[1],xy[:,0]-center[0]))
    sweep=float(np.degrees(angle[-1]-angle[0]))
    if not .025<radius<.15 or np.sqrt(np.mean(error**2))>.003 or abs(sweep)<25:
        raise ValueError('Recorded curve does not support a bounded local circle estimate')
    return dict(center_xy_pelvis_m=center.tolist(),radius_m=radius,
        radial_rms_mm=float(np.sqrt(np.mean(error**2))*1000),sweep_deg=sweep,
        contact_interval_s=[float(times[mask][0]),float(times[mask][-1])],
        physically_calibrated=False,source='nominal open-index FK, low-height path fit')


def candidates(trace,model,side,offset):
    names=list(moving(side));link=side+'_wrist_yaw_link'
    raw_q,raw_t=arrays(trace,model,side)
    t=np.linspace(0.,raw_t[-1],int(np.ceil(raw_t[-1]*100))+1)
    q=np.column_stack([np.interp(t,raw_t,raw_q[:,i]) for i in range(len(names))])
    def poses(values):return model.batch_transforms([link],{},names,values)[link]@offset
    original=poses(q)
    smooth=savgol_filter(q,31,3,axis=0,mode='interp')
    smooth+=(q[0]-smooth[0])[None,:]*(1-blend(t/.4))[:,None]
    smooth+=(q[-1]-smooth[-1])[None,:]*blend((t-(t[-1]-.4))/.4)[:,None]
    delta=smooth-q
    scale=min(1.,.003/max(float(np.max(np.abs(delta))),1e-12))
    smooth=q+delta*scale
    deviation=np.linalg.norm(poses(smooth)[:,:3,3]-original[:,:3,3],axis=1)
    if np.max(deviation)>.0015:smooth=q+(smooth-q)*(.0015/np.max(deviation))
    smooth[0]=q[0];smooth[-1]=q[-1]
    frames=poses(smooth);circle=circle_fit(original[:,:3,3],t)
    contact_end=circle['contact_interval_s'][1]
    peak_index=int(np.searchsorted(t,contact_end))
    clear=np.flatnonzero((t>contact_end)&(frames[:,2,3]>frames[peak_index,2,3]+.02))
    if not len(clear):raise ValueError('No measured withdrawal clearance after fitted arc')
    release=float(t[clear[0]])
    if release+1.5>=t[-1]:raise ValueError('Insufficient withdrawal tail for smooth correction release')
    weight=blend((t-(contact_end-2.))/2.)*(1-blend((t-release)/1.5))
    limits=np.array([model.limits[n] for n in names[:7]])
    source_sha=fingerprint(trace);results=[]
    for degrees in (0.,1.,2.,3.):
        angle=np.sign(circle['sweep_deg'])*np.radians(degrees)*weight
        rotate=Rotation.from_euler('z',angle).as_matrix()
        targets=frames.copy();center=np.r_[circle['center_xy_pelvis_m'],0.]
        targets[:,:3,3]=center+np.einsum('nij,nj->ni',rotate,frames[:,:3,3]-center)
        targets[:,:3,:3]=rotate@frames[:,:3,:3]
        output=smooth.copy()
        if degrees:
            indices=np.unique(np.r_[np.arange(0,len(t),8),len(t)-1])
            corrections=[]
            for idx in indices:
                if weight[idx]<1e-12:corrections.append(np.zeros(7));continue
                base=smooth[idx];target=targets[idx];state=dict(zip(names,base))
                def residual(x):
                    state.update(zip(names[:7],x));pose=model.transform(link,state)@offset
                    return np.r_[pose[:3,3]-target[:3,3],
                        .1*Rotation.from_matrix(target[:3,:3].T@pose[:3,:3]).as_rotvec(),.001*(x-base[:7])]
                solution=least_squares(residual,base[:7],bounds=(limits[:,0]+.02,limits[:,1]-.02),
                    ftol=1e-10,xtol=1e-10,gtol=1e-10,max_nfev=40)
                if not solution.success:raise ValueError('Local correction IK failed')
                corrections.append(solution.x-base[:7])
            correction=CubicSpline(t[indices],corrections,bc_type='clamped')(t)
            output[:,:7]+=correction
        output[0]=raw_q[0];output[-1]=raw_q[-1]
        measured=poses(output)
        position_error=np.linalg.norm(measured[:,:3,3]-targets[:,:3,3],axis=1)
        orientation_error=Rotation.from_matrix(np.transpose(targets[:,:3,:3],(0,2,1))@measured[:,:3,:3]).magnitude()
        displacement=np.linalg.norm(measured[:,:3,3]-original[:,:3,3],axis=1)
        if max(position_error)>.0003 or max(orientation_error)>np.radians(.15):
            raise ValueError('Interpolated IK correction exceeds Cartesian tolerance')
        if max(displacement)>.007 or np.max(np.abs(output-q))>.10:
            raise ValueError('Candidate correction exceeds small-change budget')
        label='平滑' if degrees==0 else f'平滑+{degrees:g}度'
        candidate=dict(raw=dict(time_s=t.tolist(),positions=output.tolist(),
            stamp_ns=[trace['raw']['stamp_ns'][0]+round(v*1e9) for v in t],sample_policy='offline_candidate'),
            hand=None,recorded_at=now(),optimization=dict(label=label,source_sha256=source_sha,
                extra_degrees=degrees,circle=circle,correction_peak_s=contact_end,release_start_s=release,
                smoothing_window_s=.30,smoothing_max_joint_delta_rad=float(np.max(np.abs(smooth-q))),
                max_tcp_change_mm=float(np.max(displacement)*1000),
                ik_max_position_error_mm=float(np.max(position_error)*1000),
                original_start_end_preserved=True,waist_correction=False,
                physical_tested=False,collision_checked=False))
        arrays(candidate,model,side)
        results.append(candidate)
    return results,circle


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--install',action='store_true')
    args=parser.parse_args()
    original=json.loads((ARCHIVE/'dring_demonstration.json').read_text())
    model=OfflineModel(ARCHIVE/'robot.urdf');side=original['side']
    offset=np.array(json.loads((ARCHIVE/'index_tcp.json').read_text())['sides'][side]['states']['open']
                    ['wrist_yaw_from_index_tcp']['matrix_row_major'])
    bundle=copy.deepcopy(original);bundle['traces']={};report={}
    for direction,old in (('left','left_3'),('right','right')):
        baseline=copy.deepcopy(original['traces'][old]);bundle['traces'][direction]=baseline
        variants,circle=candidates(baseline,model,side,offset);report[direction]=circle
        for index,variant in enumerate(variants,2):bundle['traces'][f'{direction}_{index}']=variant
        print(direction,json.dumps(circle),flush=True)
    bundle['saved_at']=now()
    out=ARCHIVE/'candidates';write_json(out/'demonstration.json',bundle);write_json(out/'circle_fit.json',report)
    hashes={str(p.relative_to(ARCHIVE)):hashlib.sha256(p.read_bytes()).hexdigest()
            for p in ARCHIVE.rglob('*') if p.is_file() and p.name!='manifest.json'}
    write_json(ARCHIVE/'manifest.json',dict(created_at=now(),sha256=hashes))
    if args.install:
        live=ROOT/'data/task_panel/dring_demonstration.json'
        if fingerprint(json.loads(live.read_text()))!=fingerprint(original):
            raise RuntimeError('Live recording changed after preservation; candidates not installed')
        write_json(live,bundle);write_json(live.with_name('dring_demonstration_'+side+'.json'),bundle)


if __name__=='__main__':main()
