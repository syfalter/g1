"""Exercise scoped contact against real MoveIt/FCL, on loopback domain 93 only."""
import os
import time
import uuid
from pathlib import Path
from scipy.spatial.transform import Rotation
import numpy as np
from geometry_msgs.msg import Pose, Point
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from g1_upper_body_moveit_config.srv import ValidateTaskState
from rosidl_runtime_py.convert import message_to_ordereddict
from task_motion_planner import robot_state
from task_session import ARMS


def verify_contact_contract(planner, q):
    if os.environ.get('ROS_DOMAIN_ID')!='93' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise RuntimeError('Synthetic collision tests are offline only')
    report=[]
    def acm():
        req=GetPlanningScene.Request()
        req.components.components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        return message_to_ordereddict(planner.call(planner.scene,req).scene.allowed_collision_matrix)
    before=acm()
    previous=[]
    def scene(background, target):
        nonlocal previous
        snapshot='task_measured_rgbd_snapshot_'+uuid.uuid4().hex
        ring='task_ring_contact_sweep_'+uuid.uuid4().hex
        req=ApplyPlanningScene.Request();req.scene.is_diff=True
        req.scene.world.collision_objects=[CollisionObject(id=n,operation=CollisionObject.REMOVE) for n in previous]
        for name,point in ((snapshot,background),(ring,target)):
            obj=CollisionObject(id=name,operation=CollisionObject.ADD)
            obj.header.frame_id='pelvis'
            obj.primitives=[SolidPrimitive(type=SolidPrimitive.SPHERE,dimensions=[.003])]
            p=Pose(position=Point(**dict(zip('xyz',map(float,point)))));p.orientation.w=1.
            obj.primitive_poses=[p];req.scene.world.collision_objects.append(obj)
        assert planner.call(planner.apply,req).success
        previous=[snapshot,ring]
        read=GetPlanningScene.Request()
        read.components.components=PlanningSceneComponents.WORLD_OBJECT_NAMES
        present={o.id for o in planner.call(planner.scene,read).scene.world.collision_objects}
        if present!=set(previous):
            raise RuntimeError('Test scene replacement failed in MoveIt: '+str(sorted(present)))
        return snapshot,ring
    def check(name,ids,side,stage,expected,state=None):
        req=ValidateTaskState.Request(robot_state=robot_state(q if state is None else state),arm=side,stage=stage,
            snapshot_object_id=ids[0],target_object_id=ids[1])
        deadline=time.monotonic()+5
        result=planner.call(planner.valid,req)
        while result.code in ('SCENE_NOT_READY','TARGET_NOT_SEPARATED') and time.monotonic()<deadline:
            time.sleep(.02);result=planner.call(planner.valid,req)
        record={'name':name,'stage':stage,'arm':side,'valid':result.valid,'code':result.code,
                'contacts':[(c.contact_body_1,c.contact_body_2) for c in result.contacts]}
        report.append(record)
        if result.valid!=expected or result.code not in ('VALID','COLLISION'):
            raise RuntimeError('Contact contract failed: '+str(record))
        return result
    far=np.array([3.,3.,3.])
    ids=scene(far,far+.1)
    check('baseline',ids,'left','approach',True)
    for side,other in (('left','right'),('right','left')):
        tip=planner.model.transform(side+'_index_tip',q)[:3,3]
        ids=scene(far,tip)
        for stage in ('insert','rotate','withdraw'):
            check(side+' distal target allowed',ids,side,stage,True)
        for stage in ('prepare','recover','approach','return_ready','diagnostic','hand_transition'):
            check(side+' distal target outside contact stage',ids,side,stage,False)
        check(side+' is nonselected hand',ids,other,'insert',False)
        ids=scene(tip,far)
        check(side+' index vs background remains forbidden',ids,side,'insert',False)
        palm_link=side+('_hand_base' if getattr(planner,'articulated_hands',False) else '_rubber_hand')
        collision=planner.model.robot.find(f"link[@name='{palm_link}']/collision")
        mesh=Path(collision.find('geometry/mesh').get('filename').removeprefix('file://'))
        raw=mesh.read_bytes()
        count=int.from_bytes(raw[80:84],'little')
        if len(raw)!=84+50*count:raise ValueError('Expected binary STL collision asset')
        vertices=np.frombuffer(raw,dtype='<f4',count=9,offset=96).reshape(3,3)
        local=vertices.mean(axis=0)
        origin=collision.find('origin')
        if origin is not None:
            local=Rotation.from_euler('xyz',list(map(float,origin.get('rpy','0 0 0').split()))).apply(local)
            local+=np.array(list(map(float,origin.get('xyz','0 0 0').split())))
        palm=planner.model.transform(palm_link,q)@np.r_[local,1.]
        ids=scene(far,palm[:3])
        check(side+' palm vs target remains forbidden',ids,side,'insert',False)
    ids=scene(far,far+.1)
    rng=np.random.default_rng(7)
    detected=False
    for _ in range(50):
        sample=dict(q)
        for name in ARMS['left']:
            limit=planner.model.robot.find(f"joint[@name='{name}']/limit")
            sample[name]=float(rng.uniform(float(limit.get('lower'))+.03,float(limit.get('upper'))-.03))
        req=ValidateTaskState.Request(robot_state=robot_state(sample),arm='left',stage='rotate',
            snapshot_object_id=ids[0],target_object_id=ids[1])
        result=planner.call(planner.valid,req)
        if result.code=='COLLISION' and any(not c.contact_body_1.startswith('task_') and
                not c.contact_body_2.startswith('task_') for c in result.contacts):
            report.append({'name':'self collision still rejected in contact stage','code':result.code})
            detected=True;break
    if not detected:raise RuntimeError('No self-collision fixture found in bounded test')
    if acm()!=before:raise RuntimeError('Scoped validator modified the global ACM')
    report.append({'name':'global ACM unchanged after allowed and rejected requests','passed':True})
    return report


def verify_hand_waist_contract(planner,q):
    """Run after the contact fixtures have left both test objects far away."""
    from task_session import Ticket
    from task_hand_control import model_joints, POSTURES
    from moveit_msgs.msg import RobotTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    if os.environ.get('ROS_DOMAIN_ID')!='93' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise RuntimeError('Model trajectory tests are offline only')
    read=GetPlanningScene.Request();read.components.components=PlanningSceneComponents.WORLD_OBJECT_NAMES
    ids=[obj.id for obj in planner.call(planner.scene,read).scene.world.collision_objects]
    planner.snapshot_id=next(name for name in ids if name.startswith('task_measured_rgbd_snapshot_'))
    planner.target_id=next(name for name in ids if name.startswith('task_ring_contact_sweep_'))
    results=[]
    for side in ('left','right'):
        ticket=Ticket(1,'prepare',side,.15,.15)
        if planner.articulated_hands:
            closed=planner.hand_transition(side,q,'index_only',ticket)
            opened=planner.hand_transition(side,closed['end'],'open',ticket)
            for name in q:
                if name not in model_joints(side,POSTURES['open']):
                    assert closed['end'][name]==q[name]
            results.append({'side':side,'hand_transition_samples':162,'arm_stationary':True})
        if planner.waist_assist:
            names=planner.names(side)
            path=RobotTrajectory();path.joint_trajectory.joint_names=list(names)
            for amount in (0.,.01,.02):
                target=dict(q);target['waist_yaw_joint']+=amount
                path.joint_trajectory.points.append(JointTrajectoryPoint(positions=[target[n] for n in names]))
            segment=planner.retime(side,q,path,ticket,'Waist timing fixture',stage='prepare')
            assert len(segment['trajectory'].joint_names)==8
            assert abs(segment['end']['waist_yaw_joint']-q['waist_yaw_joint']-.02)<1e-8
            results.append({'side':side,'timed_joints':list(segment['trajectory'].joint_names),
                            'waist_delta_rad':.02,'hardware_commands_sent':0})
    return results
