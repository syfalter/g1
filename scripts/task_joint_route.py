"""Opt-in offline return routing; all final checks stay with ScopedTaskPlanner."""
import hashlib
from pathlib import Path
import time

import numpy as np
from moveit_msgs.msg import RobotTrajectory
from task_motion_planner import PlanningFailure
from task_path_smoothing import joint_waypoint_trajectory


def backend_identity():
    import _task_ompl_route as native
    path=Path(native.__file__)
    return dict(method='ompl_rrtconnect_scoped_return_v1',ompl=native.ompl_version,
        native_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),seed=19,
        range_rad=.25,candidate_edge_l2_step_rad=.02,guarded_margin_rad=.021,
        search_budget_seconds=4.,collision_scope_unchanged=True,final_validators_unchanged=True)


def return_route(planner,side,q,goal,ticket,label,original_failure):
    import _task_ompl_route as native
    stage='return_ready';names=list(planner.names(side));limits=planner.joint_limits(side)
    start=np.array([q[n] for n in names]);end=np.array([goal[n] for n in names])
    lower=np.array([limits[n][0]+.021 for n in names]);upper=np.array([limits[n][1]-.021 for n in names])
    record=dict(backend_identity(),accepted=False,direct_failure=dict(original_failure),state_checks=0,
                shortcut_edge_tests=0)
    planner.search_diagnostics.setdefault('return_routing',[]).append(record)
    began=time.monotonic();deadline=began+4.
    class RouteBudget(Exception):pass
    def check():
        planner.check_plan(ticket)
        if time.monotonic()>=deadline:raise RouteBudget
    def valid(values):
        check();record['state_checks']+=1
        state=dict(q);state.update(zip(names,map(float,values)))
        try:planner.validate(state,ticket,side,stage)
        except PlanningFailure as error:
            if error.diagnostic['code'] not in ('COLLISION','JOINT_MARGIN'):raise
            return False
        return True
    def stop():
        planner.check_plan(ticket)
        return time.monotonic()>=deadline
    def edge(a,b):
        check();record['shortcut_edge_tests']+=1
        count=max(2,int(np.ceil(np.max(abs(b-a))/.02))+1)
        try:
            planner.validate_trajectory_samples(q,ticket,side,stage,names,np.linspace(a,b,count),None,
                                                lambda _:check())
        except PlanningFailure as error:
            if error.diagnostic['code'] not in ('COLLISION','JOINT_MARGIN'):raise
            return False
        check();return True
    try:
        if (np.any(start<lower) or np.any(start>upper) or np.any(end<lower) or np.any(end>upper)):
            raise PlanningFailure('MOTION_PLAN_FAILED',stage,'Return endpoints outside OMPL internal margin')
        try:
            raw=np.asarray(native.solve(start,end,lower,upper,valid,stop,19),dtype=float)
        except RouteBudget:
            raw=np.empty((0,len(names)))
        if raw.size==0:
            raise PlanningFailure('MOTION_PLAN_FAILED',stage,'No exact scoped OMPL return within local budget')
        if (raw.ndim!=2 or raw.shape[1]!=len(names) or len(raw)<2 or not np.isfinite(raw).all() or
                np.any(raw<lower) or np.any(raw>upper) or
                not np.allclose(raw[0],start,atol=1e-9,rtol=0) or not np.allclose(raw[-1],end,atol=1e-9,rtol=0)):
            raise RuntimeError('OMPL return violated endpoint or joint bounds contract')
        raw[0]=start;raw[-1]=end
        record['raw_waypoints']=raw.tolist()
        chosen=[raw[0]];index=0
        while index<len(raw)-1:
            next_index=index+1
            try:
                for trial in range(len(raw)-1,index+1,-1):
                    if record['shortcut_edge_tests']>=24:break
                    if edge(raw[index],raw[trial]):
                        next_index=trial;break
            except RouteBudget:
                chosen.extend(raw[index+1:]);break
            chosen.append(raw[next_index]);index=next_index
        record['waypoints']=np.asarray(chosen).tolist()
        record['search_seconds']=time.monotonic()-began
        planner.check_plan(ticket)
        trajectory=joint_waypoint_trajectory(names,chosen)
        path=RobotTrajectory();path.joint_trajectory.joint_names=names
        result=planner.retime(side,q,path,ticket,label,stage=stage,timed_trajectory=trajectory)
        record['accepted']=True
        result['joint_route']=dict(method=record['method'],waypoints=len(chosen),seed=19)
        return result
    except PlanningFailure as error:
        record['failure']=dict(error.diagnostic)
        raise
    finally:
        record['elapsed_seconds']=time.monotonic()-began
