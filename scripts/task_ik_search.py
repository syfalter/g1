"""Bounded initial-pose coverage and the original workbench IK objective."""
import math
import time
import numpy as np
from scipy.spatial.transform import Rotation
from workbench_orientation import orientation_allowed


def bounded_pose_ik(model,link,names,limits,current,target,offset,check,timeout,*,
                    continuity_reference=None,position_tolerance=.0002,orientation_tolerance=.002):
    """Deterministic single-seed SciPy IK candidate; the caller must still validate it."""
    from scipy.optimize import least_squares
    names=list(names);target=np.asarray(target,dtype=float)
    if target.shape!=(4,4) or not np.isfinite(target).all():raise ValueError('Invalid IK target')
    if not math.isfinite(timeout) or timeout<=0:raise ValueError('Invalid local IK budget')
    if (not math.isfinite(position_tolerance) or not 0<position_tolerance<=.002 or
            not math.isfinite(orientation_tolerance) or not 0<orientation_tolerance<=math.radians(2.)):
        raise ValueError('Invalid candidate pose tolerances')
    seed=np.array([current[name] for name in names],dtype=float)
    lower=np.array([limits[name][0]+.021 for name in names])
    upper=np.array([limits[name][1]-.021 for name in names])
    if not np.isfinite(np.r_[seed,lower,upper]).all() or np.any(lower>=upper):
        raise ValueError('Invalid guarded IK joint interval')
    reference=None
    if continuity_reference is not None:
        reference=np.asarray([continuity_reference[name] for name in names],dtype=float)
        if not np.isfinite(reference).all():raise ValueError('Invalid IK continuity reference')
        # Stay inside the unchanged 0.12 rad continuation gate, including alternate seeds.
        lower=np.maximum(lower,reference-.119)
        upper=np.minimum(upper,reference+.119)
        if np.any(lower>=upper):raise ValueError('Empty guarded IK continuation interval')
    began=time.monotonic();deadline=began+timeout
    experimental=reference is not None or position_tolerance!=.0002 or orientation_tolerance!=.002
    diagnostic=dict(backend='scipy_bounded_task_trf_v1' if experimental else 'scipy_bounded_trf_v1',
        code='LOCAL_IK_FAILED',rpc_seconds=0.,guarded_margin_rad=.021,max_evaluations=60,
        position_scale=10.,randomized=False,candidate_position_tolerance_m=position_tolerance,
        candidate_orientation_tolerance_rad=orientation_tolerance,
        continuity_interval_rad=.119 if reference is not None else None)
    cached=None;cached_value=None
    class LocalBudget(Exception):pass
    def evaluate(values):
        nonlocal cached,cached_value
        check()
        if time.monotonic()>=deadline:raise LocalBudget
        if cached is not None and np.array_equal(cached,values):return cached_value
        state=dict(current);state.update(zip(names,map(float,values)))
        actual,geometric=model.pose_jacobian(link,state,names,offset)
        error=np.r_[10*(actual[:3,3]-target[:3,3]),(actual[:3,:3]-target[:3,:3]).ravel()]
        rotation=np.cross(geometric[3:].T[:,None,:],actual[:3,:3].T[None,:,:])
        jacobian=np.vstack((10*geometric[:3],rotation.transpose(0,2,1).reshape(len(names),9).T))
        cached=values.copy();cached_value=(error,jacobian)
        return cached_value
    try:
        check()
        result=least_squares(lambda values:evaluate(values)[0],np.clip(seed,lower,upper),
            jac=lambda values:evaluate(values)[1],bounds=(lower,upper),method='trf',max_nfev=60,
            ftol=1e-8,xtol=1e-8,gtol=1e-8)
        check()
        if time.monotonic()>=deadline:raise LocalBudget
        answer=dict(current);answer.update(zip(names,map(float,result.x)))
        actual=model.transform(link,answer)@offset
        position=float(np.linalg.norm(actual[:3,3]-target[:3,3]))
        angle=float(Rotation.from_matrix(target[:3,:3].T@actual[:3,:3]).magnitude())
        diagnostic.update(evaluations=result.nfev,optimizer_status=int(result.status),
                          position_error_m=position,orientation_error_rad=angle,
                          candidate_joints={name:float(value) for name,value in zip(names,result.x)})
        if reference is not None:
            diagnostic['max_continuation_step_rad']=float(np.max(np.abs(result.x-reference)))
        if position<=position_tolerance and angle<=orientation_tolerance and np.isfinite(result.x).all():
            diagnostic['code']='LOCAL_IK_POSE_VALID'
            return answer,diagnostic
        return None,diagnostic
    except LocalBudget:
        diagnostic['code']='LOCAL_IK_BUDGET'
        return None,diagnostic
    finally:
        diagnostic['local_seconds']=time.monotonic()-began


def waist_biased_continuation(names, limits, current, target, tip):
    """Local kinematic preference, not a force or motor-load optimizer."""
    from scipy.optimize import least_squares
    seed=np.array([current[n] for n in names],dtype=float)
    lower=np.array([max(limits[n][0]+.021,current[n]-.10) for n in names])
    upper=np.array([min(limits[n][1]-.021,current[n]+.10) for n in names])
    if np.any(lower>=upper):return None
    weights=np.array([.5 if n=='waist_yaw_joint' else
                      3. if n.endswith('shoulder_roll_joint') else 2. for n in names])
    rotation=Rotation.from_matrix(target[:3,:3])
    def state(values):
        result=dict(current);result.update(zip(names,map(float,values)));return result
    def residual(values):
        actual=tip(state(values))
        return np.r_[500*(actual[:3,3]-target[:3,3]),
                     10*(rotation.inv()*Rotation.from_matrix(actual[:3,:3])).as_rotvec(),
                     .1*weights*(values-seed)]
    result=least_squares(residual,np.clip(seed,lower,upper),bounds=(lower,upper),max_nfev=35)
    if not result.success or not np.isfinite(result.x).all():return None
    answer=state(result.x);actual=tip(answer)
    if (np.linalg.norm(actual[:3,3]-target[:3,3])>.001 or
            (rotation.inv()*Rotation.from_matrix(actual[:3,:3])).magnitude()>math.radians(.25)):
        return None
    return answer


def solution_score(names, limits, previous, answer, previous_quat, actual_quat):
    delta=np.array([answer[n]-previous[n] for n in names],dtype=float)
    margins=np.array([min(answer[n]-limits[n][0],limits[n][1]-answer[n]) for n in names])
    if not np.isfinite(delta).all() or not np.isfinite(margins).all() or np.min(margins)<.02:
        return None
    joint=float(delta@delta)
    wrist=sum(float((answer[n]-previous[n])**2) for n in names if '_wrist_' in n)
    penalty=float(np.square(np.maximum(0.,.15-margins)).sum())
    change=float((Rotation.from_quat(previous_quat).inv()*Rotation.from_quat(actual_quat)).magnitude())
    return {'total':joint+.5*wrist+2*penalty+.2*change**2,
            'joint_change_sq':joint,'wrist_change_sq':wrist,'limit_penalty':penalty,
            'orientation_change_rad':change,'max_joint_step_rad':float(np.max(abs(delta))),
            'min_joint_margin_rad':float(np.min(margins))}


def preinsert_orientations(nominal, policy):
    reference=Rotation.from_quat(nominal)
    rotations=[reference]
    # Keep all seven previous poses, then cover diagonal swing + twist combinations.
    for axis,limit in ((0,policy['tilt_deg']),(1,policy['tilt_deg']),(2,policy['roll_deg'])):
        for sign in (-1,1):
            v=np.zeros(3);v[axis]=math.radians(sign*limit*.9)
            rotations.append(reference*Rotation.from_rotvec(v))
    swings=[Rotation.identity()]
    for fraction in (.5,1.):
        for azimuth in np.linspace(0.,2*math.pi,8,endpoint=False):
            vector=np.array([math.cos(azimuth),math.sin(azimuth),0.])
            swings.append(Rotation.from_rotvec(vector*math.radians(policy['tilt_deg']*fraction)))
    for swing in swings:
        for fraction in (0.,-.5,.5,-1.,1.):
            twist=Rotation.from_rotvec([0.,0.,math.radians(policy['roll_deg']*fraction)])
            rotations.append(reference*swing*twist)
    result=[]
    for rotation in rotations:
        quat=rotation.as_quat()
        if (orientation_allowed(nominal,quat,policy,epsilon=1e-7) and
                not any(abs(float(quat@other))>1-1e-12 for other in result)):
            result.append(quat)
    return result


def broad_preinsert_orientations(nominal):
    """Original CAD's 5 tilts x 8 azimuths x 4 twists, in the frozen frame."""
    reference=Rotation.from_quat(nominal).as_matrix()
    horizontal=reference[:,2].copy();horizontal[2]=0.
    if np.linalg.norm(horizontal)<1e-8:
        horizontal=np.cross(reference[:,0],[0.,0.,1.])
        horizontal[2]=0.
    if np.linalg.norm(horizontal)<1e-8:raise ValueError('No horizontal insertion basis')
    horizontal/=np.linalg.norm(horizontal)
    n=np.array([0.,0.,1.]);result=[]
    for tilt in (20,35,50,0,65):
        for yaw in (0,30,-30,60,-60,90,-90,180):
            h=Rotation.from_euler('z',yaw,degrees=True).apply(horizontal)
            z=-n*math.cos(math.radians(tilt))+h*math.sin(math.radians(tilt))
            x=np.cross(n,h)
            base=Rotation.from_matrix(np.column_stack((x,np.cross(z,x),z)))
            for roll in (0,90,180,270):
                result.append((base*Rotation.from_euler('z',roll,degrees=True)).as_quat())
    return result


def preinsert_seeds(side, start, limits):
    sign=1. if side=='left' else -1.
    offsets=({},
        {'shoulder_roll':.35*sign,'shoulder_yaw':.45,'elbow':.2},
        {'shoulder_roll':.35*sign,'shoulder_yaw':-.45,'elbow':.2},
        {'shoulder_pitch':-.35,'shoulder_roll':.65*sign,'shoulder_yaw':-.65*sign,
         'elbow':-.35,'wrist_roll':.35*sign},
        {'shoulder_pitch':.35,'shoulder_roll':.15*sign,'shoulder_yaw':.65*sign,
         'elbow':.45,'wrist_roll':-.35*sign})
    seeds=[]
    for offset in offsets:
        seed=dict(start)
        for suffix,delta in offset.items():
            name=side+'_'+suffix+'_joint'
            lower,upper=limits[name]
            seed[name]=float(np.clip(start[name]+delta,lower+.03,upper-.03))
        if not any(all(abs(seed[n]-old[n])<1e-10 for n in limits) for old in seeds):
            seeds.append(seed)
    if 'waist_yaw_joint' in limits:
        lower,upper=limits['waist_yaw_joint']
        for amount in (-.3,.3):
            seed=dict(start)
            seed['waist_yaw_joint']=float(np.clip(start['waist_yaw_joint']+amount,lower+.03,upper-.03))
            seeds.append(seed)
    return seeds


def preinsert_jobs(orientation_count, seed_count, *, policy='legacy', initial_trials=32):
    if policy not in ('legacy','diverse'):raise ValueError('Unknown preinsert search policy')
    if policy=='diverse':
        from itertools import islice
        if orientation_count<1 or seed_count<1 or initial_trials<1:
            raise ValueError('Invalid preinsert grid dimensions or initial batch')
        seen=set()
        # Preserve the complete first batch before interleaving the remaining grid.
        for job in islice(preinsert_jobs(orientation_count,seed_count),initial_trials):
            seen.add(job);yield job
        for height_round in range(3):
            for seed_round in range(seed_count):
                for orientation in range(orientation_count):
                    job=(orientation,(orientation+height_round+seed_round)%3,
                         (orientation+seed_round)%seed_count)
                    if job not in seen:
                        seen.add(job);yield job
        return
    seen=set()
    # Spread a limited wall-clock budget across all angles, heights and seeds.
    # Then revisit the entire old coarse grid before further refinement.
    for orientation in range(orientation_count):
        job=(orientation,orientation%3,orientation%seed_count)
        seen.add(job);yield job
    for orientation in range(min(7,orientation_count)):
        for height in range(3):
            for seed in range(min(3,seed_count)):
                job=(orientation,height,seed)
                if job not in seen:
                    seen.add(job);yield job
    for seed in range(seed_count):
        for orientation in range(orientation_count):
            for height in range(3):
                job=(orientation,height,seed)
                if job not in seen:
                    seen.add(job);yield job


def diverse_preinsert_batch(candidates, delivered, names, count):
    """Balance tested orientation families, then maximize sampled joint separation."""
    if not delivered:return candidates[:count]
    remaining=list(candidates);history=list(delivered);selected=[]
    while remaining and len(selected)<count:
        def priority(candidate):
            same=[old for old in history if old['orientation_index']==candidate['orientation_index']]
            distance=min((max(abs(candidate['goal'][name]-old['goal'][name]) for name in names)
                          for old in same),default=0.)
            return len(same),-distance,candidate['score']['total'],candidate['trial_index']
        chosen=min(range(len(remaining)),key=lambda index:priority(remaining[index]))
        candidate=remaining.pop(chosen)
        selected.append(candidate);history.append(candidate)
    return selected
