"""One-shot, visualization-only playback of an editor trace."""
import copy
import numpy as np
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from task_composition_preview import at, stamp
from task_trajectory import time_seconds


def trajectory(trace, model, side):
    from task_dring_teach_data import moving
    names = moving(side)
    if trace.get('optimization', {}).get('kind') == 'fast_timing':
        from task_trace_timing import stored_trajectory
        return stored_trajectory(trace, model, names)[0]
    return JointTrajectory(joint_names=list(names), points=[
        stamp(JointTrajectoryPoint(positions=list(q)), t)
        for t, q in zip(trace['raw']['time_s'], trace['raw']['positions'])])


def message(source, trace, position, model, side, context, config, slot):
    from task_motion_planner import robot_state
    from task_trace_markers import hand_context
    from task_hand_control import model_joints
    context = dict(context, **trace.get('context_joints', {}))
    context = hand_context(trace, side, context, config, slot)
    points = [at(source, position)]+[copy.deepcopy(p) for p in source.points if time_seconds(p)>position+1e-9]
    names = list(source.joint_names)
    if isinstance(trace.get('hand'), dict):
        hand = trace['hand']
        # Arm raw samples and hand samples are piecewise-linear recordings.
        times = sorted(set([time_seconds(p) for p in points]+[
            float(t) for t in hand['time_s'] if position<t<time_seconds(source.points[-1])]))
        points = [at(source, t) for t in times]
        for p in points:
            values = model_joints(side, [np.interp(time_seconds(p), hand['time_s'], axis)
                                        for axis in np.asarray(hand['q']).T])
            extra = [name for name in values if name not in source.joint_names]
            p.positions.extend(values[name] for name in extra)
            p.velocities=[]; p.accelerations=[]
        names += extra
    elif config.get('hands') and slot=='pull' and 'pull_grip' in config['hands']:
        context.update(model_joints(side,config['hands']['pull_grip']['feedback_q']))
    movable = {j.get('name') for j in model.robot.findall('joint')
               if j.get('type') in ('revolute','continuous','prismatic')}
    context.update(zip(names, points[0].positions))
    context = {n:float(v) for n,v in context.items() if n in movable}
    return DisplayTrajectory(trajectory_start=robot_state(context), trajectory=[RobotTrajectory(
        joint_trajectory=JointTrajectory(joint_names=names,points=[
            stamp(p,time_seconds(p)-position) for p in points]))])
