"""Task planning, optional hand stages and waist assistance; no hardware commands."""
import math
import time
import uuid
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Pose, Point
from moveit_msgs.msg import RobotState, Constraints, JointConstraint, RobotTrajectory, CollisionObject, PlanningSceneComponents
from moveit_msgs.srv import GetMotionPlan, GetPositionIK, ApplyPlanningScene, GetPlanningScene
from shape_msgs.msg import Mesh, MeshTriangle
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import UInt64MultiArray
from g1_upper_body_moveit_config.srv import TimeTaskPath, ValidateTaskState, ValidateTaskTrajectory
from task_hand_model import require_task_hand_model
from task_contact_surface import separate_contact_surface
from task_session import ARMS
from task_trajectory import (condition_endpoints, samples, sample_count, stretch_to_limits,
                             validate_waist_envelope, task_jerk_limits, validate_jerk_limits, task_motion_limits,
                             time_seconds)
from workbench_orientation import orientation_policy, orientation_candidates, orientation_allowed, orientation_error
from task_planning_config import PlanningParameters
from task_ik_search import solution_score, preinsert_orientations, broad_preinsert_orientations, preinsert_seeds, preinsert_jobs
from task_ik_search import diverse_preinsert_batch
from task_kinematics import arm_names, arm_group, capture_transform, WAIST
from task_hand_control import model_joints, model_feedback, POSTURES


class PlanningFailure(RuntimeError):
    def __init__(self, code, stage, message, **details):
        self.diagnostic = dict(code=code,stage=stage,message=message,**details)
        super().__init__(f'{stage}: {code}: {message}')


def robot_state(q):
    result = RobotState()
    result.joint_state.name = list(q)
    result.joint_state.position = [float(v) for v in q.values()]
    return result


def pose(matrix):
    msg = Pose()
    msg.position = Point(**dict(zip('xyz', map(float, matrix[:3,3]))))
    for key, value in zip('xyzw', Rotation.from_matrix(matrix[:3,:3]).as_quat()):
        setattr(msg.orientation, key, float(value))
    return msg


def surface_mesh(xyz, resolution=.015):
    """Occupied measured-surface voxels, never declare unobserved space measured free."""
    xyz = np.asarray(xyz)
    if xyz.ndim != 2 or xyz.shape[1] != 3 or not np.isfinite(xyz).all():
        raise ValueError('Invalid measured surface coordinates')
    cells = np.unique(np.floor(xyz/resolution).astype(np.int64), axis=0)
    if not 0 < len(cells) <= 25000:
        raise ValueError('Empty or excessive measured collision surface')
    corners = np.array([[0,0,0],[1,0,0],[1,1,0],[0,1,0],
                        [0,0,1],[1,0,1],[1,1,1],[0,1,1]])
    faces = ((0,2,1),(0,3,2),(4,5,6),(4,6,7),(0,1,5),(0,5,4),
             (1,2,6),(1,6,5),(2,3,7),(2,7,6),(3,0,4),(3,4,7))
    result = Mesh()
    for cell in cells:
        offset = len(result.vertices)
        result.vertices.extend(Point(**dict(zip('xyz', p.tolist())))
                               for p in (cell+corners)*resolution)
        result.triangles.extend(MeshTriangle(vertex_indices=[offset+i for i in face]) for face in faces)
    return result


class TaskPlanner:
    def __init__(self, node, model, call, progress=None, *, search_seconds=40., search_attempts=600,
                 waist_assist=False, articulated_hands=False, orientation_search='broad', incremental_search=True):
        if not math.isfinite(search_seconds) or not .1<=search_seconds<=90 or not 1<=search_attempts<=5000:
            raise ValueError('Invalid initial IK search budget')
        self.search_seconds,self.search_attempts=search_seconds,search_attempts
        self.incremental_search=incremental_search
        self.search_diagnostics={}
        if orientation_search not in ('local','broad'):raise ValueError('Unknown orientation search')
        self.orientation_search=orientation_search
        self.model, self.call = model, call
        self.waist_assist=waist_assist
        self.articulated_hands=articulated_hands
        self.reference_frame='pelvis'
        self._names={side:arm_names(side,waist_assist) for side in ARMS}
        self._groups={side:arm_group(side,waist_assist) for side in ARMS}
        self._joint_limits={}
        for side,names in self._names.items():
            limits={}
            for name in names:
                if name in getattr(model,'limits',{}):
                    limits[name]=model.limits[name]
                    continue
                limit=model.robot.find(f"joint[@name='{name}']/limit")
                if limit is None:raise PlanningFailure('MODEL_MISMATCH','ik','Missing joint limit: '+name)
                limits[name]=(float(limit.get('lower')),float(limit.get('upper')))
            self._joint_limits[side]=limits
        if articulated_hands:
            for side in ARMS:
                for name in model_joints(side,POSTURES['open']):
                    joint=model.robot.find(f"joint[@name='{name}']")
                    if joint is None or joint.get('type')!='revolute':
                        raise RuntimeError('MODEL_MISMATCH: articulated hand launch required: '+name)
        self.progress = progress or (lambda ticket, message: None)
        self.motion = node.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.ik = node.create_client(GetPositionIK, '/compute_ik')
        self.valid = node.create_client(ValidateTaskState, '/g1/task/validate_state')
        self.batch_valid = node.create_client(ValidateTaskTrajectory, '/g1/task/validate_trajectory')
        self.apply = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.scene = node.create_client(GetPlanningScene, '/get_planning_scene')
        self.timing = node.create_client(TimeTaskPath, '/g1/task/time_path')
        self._pending_batch_progress=None
        self._progress_subscription=node.create_subscription(
            UInt64MultiArray,'/g1/task/validation_progress',self.batch_progress,10)
        self.snapshot_id = self.target_id = ''
        self.last_ik = {}

    def names(self, side):
        cached=getattr(self,'_names',None)
        return cached[side] if cached is not None else arm_names(side,getattr(self,'waist_assist',False))

    def group(self, side):
        cached=getattr(self,'_groups',None)
        return cached[side] if cached is not None else arm_group(side,getattr(self,'waist_assist',False))

    def hand_transition(self, side, q, posture, ticket):
        if not getattr(self,'articulated_hands',False):
            raise RuntimeError('Articulated hand model required for hand stages')
        from task_hand_control import posture_targets
        target_q=posture_targets(posture,getattr(self,'dring_hand_settings',None))
        target=model_joints(side,target_q);initial=model_feedback(side,q)
        names=list(target)
        from trajectory_msgs.msg import JointTrajectory
        trajectory=JointTrajectory(joint_names=names)
        state=dict(q)
        # This models synchronized actuators; real phalanx mapping and timing
        # remain a commissioning gate, not a claim of measured hand geometry.
        for t in np.linspace(0.,1.,81):
            blend=t*t*t*(10+t*(-15+6*t))
            state.update({n:q[n]+blend*(target[n]-q[n]) for n in names})
            self.validate(state,ticket,side,'hand_transition')
            point=JointTrajectoryPoint(positions=[state[n] for n in names])
            ns=round(float(t)*1e9)
            point.time_from_start.sec=ns//1000000000;point.time_from_start.nanosec=ns%1000000000
            trajectory.points.append(point)
        return {'kind':'hand','side':side,'stage':'hand_transition','label':'Hand '+posture,
                'posture':posture,'target_q':list(target_q),'initial_q':list(initial),
                'trajectory':trajectory,'start':dict(q),'end':state,'duration':1.}

    def install_surface(self, xyz, torso_transform, ticket, *, uv=None, bbox=None, center=None):
        require_task_hand_model(self.model)
        suffix=uuid.uuid4().hex
        self.snapshot_id='task_measured_rgbd_snapshot_'+suffix
        self.target_id=''
        background=np.asarray(xyz)
        self.scene_report={'hardware_validated':False,'source':'measured_surface_only'}
        surfaces=[]
        if ticket.kind=='task':
            background,target,self.scene_report=separate_contact_surface(xyz,uv,bbox,center)
            self.target_id='task_ring_contact_sweep_'+suffix
            surfaces.append((self.target_id,target))
        surfaces.append((self.snapshot_id,background))
        read=GetPlanningScene.Request()
        read.components.components=PlanningSceneComponents.WORLD_OBJECT_NAMES
        old=self.call(self.scene,read,ticket).scene.world.collision_objects
        req = ApplyPlanningScene.Request(); req.scene.is_diff = True
        for obj in old:
            if (obj.id=='task_measured_rgbd_snapshot' or
                    obj.id.startswith(('task_measured_rgbd_snapshot_','task_ring_contact_sweep_'))):
                req.scene.world.collision_objects.append(CollisionObject(id=obj.id,operation=CollisionObject.REMOVE))
        for name,points in surfaces:
            obj=CollisionObject(id=name,operation=CollisionObject.ADD)
            obj.header.frame_id='pelvis'
            points=points @ torso_transform[:3,:3].T + torso_transform[:3,3]
            obj.meshes=[surface_mesh(points)]
            origin=Pose();origin.orientation.w=1.
            obj.mesh_poses=[origin]
            req.scene.world.collision_objects.append(obj)
        if not self.call(self.apply, req, ticket, 30).success:
            raise RuntimeError('Measured collision surface was not accepted')
        self.scene_report.update(snapshot_object_id=self.snapshot_id,target_object_id=self.target_id)

    def validate(self, q, ticket, side=None, stage=None):
        req = ValidateTaskState.Request(robot_state=robot_state(q),arm=side or ticket.side,
            stage=stage or ticket.kind,snapshot_object_id=self.snapshot_id,target_object_id=self.target_id)
        result = self.call(self.valid, req, ticket)
        deadline=time.monotonic()+3
        while result.code=='SCENE_NOT_READY' and time.monotonic()<deadline:
            time.sleep(.02)
            result=self.call(self.valid,req,ticket)
        if not result.valid:
            contacts=[{'body_1':c.contact_body_1,'body_2':c.contact_body_2,'depth_m':c.depth} for c in result.contacts]
            message=result.message
            if contacts:message+=': '+', '.join(f"{c['body_1']}/{c['body_2']} ({1000*c['depth_m']:.1f}mm)" for c in contacts[:5])
            raise PlanningFailure(result.code,req.stage,message,contacts=contacts)

    def batch_progress(self,message):
        pending=getattr(self,'_pending_batch_progress',None)
        if pending is None or len(message.layout.dim)!=3 or len(message.data)!=3:return
        identity=tuple(d.label for d in message.layout.dim)
        started,checked,total=message.data
        if (identity!=pending['identity'] or started<pending['started_ns'] or
                total!=pending['total'] or not pending['checked']<checked<=total):return
        # These nodes share the host's monotonic clock. Ignore old requests,
        # including late messages from the same snapshot after cancellation.
        pending['checked']=checked
        try:pending['callback'](checked)
        except RuntimeError:pass  # Cancelled tickets must not update the new UI.

    def validate_trajectory_batch(self,q,ticket,side,stage,trajectory_names,positions,progress=None):
        positions=np.asarray(positions,dtype=np.float64)
        if positions.ndim!=2 or positions.shape[1]!=len(trajectory_names) or not len(positions):
            raise ValueError('Invalid trajectory validation batch')
        joint_names=list(q)
        columns={name:index for index,name in enumerate(joint_names)}
        if any(name not in columns for name in trajectory_names):
            raise ValueError('Trajectory joint missing from validation state')
        base=np.asarray([q[name] for name in joint_names],dtype=np.float64)
        states=np.broadcast_to(base,(len(positions),len(base))).copy()
        for source,name in enumerate(trajectory_names):states[:,columns[name]]=positions[:,source]
        req=ValidateTaskTrajectory.Request(joint_names=joint_names,
            positions=states.ravel().tolist(),sample_count=len(states),arm=side or ticket.side,
            stage=stage or ticket.kind,snapshot_object_id=self.snapshot_id,target_object_id=self.target_id)
        self._pending_batch_progress=(dict(identity=(self.snapshot_id,req.arm,req.stage),
            started_ns=time.monotonic_ns(),total=len(states),checked=0,callback=progress) if progress else None)
        try:
            result=self.call(self.batch_valid,req,ticket)
            deadline=time.monotonic()+3
            while result.code=='SCENE_NOT_READY' and time.monotonic()<deadline:
                time.sleep(.02)
                result=self.call(self.batch_valid,req,ticket)
        finally:self._pending_batch_progress=None
        if result.valid:
            if result.checked_samples!=len(states) or result.first_invalid_sample!=len(states):
                raise RuntimeError('Batch validator returned inconsistent success accounting')
            return
        index=int(result.first_invalid_sample)
        if index>=len(states):raise RuntimeError('Batch validator returned an invalid failure index')
        contacts=[{'body_1':c.contact_body_1,'body_2':c.contact_body_2,'depth_m':c.depth}
                  for c in result.contacts]
        message=result.message
        if contacts:message+=': '+', '.join(f"{c['body_1']}/{c['body_2']} ({1000*c['depth_m']:.1f}mm)" for c in contacts[:5])
        raise PlanningFailure(result.code,req.stage,message,contacts=contacts,
                              batch_sample_index=index,checked_samples=int(result.checked_samples))

    def joint_path(self, side, q, target, ticket, label, stage=None):
        stage=stage or ticket.kind
        if any(name not in target for name in ARMS[side]):
            raise ValueError('Incomplete selected-arm target')
        goal=dict(q)
        goal.update({name:target.get(name,q[name]) for name in self.names(side)})
        self.validate(q,ticket,side,stage)
        self.validate(goal,ticket,side,stage)
        self.progress(ticket, label+': computing joint path...')
        req = GetMotionPlan.Request(); r = req.motion_plan_request
        r.group_name = self.group(side); r.start_state = robot_state(q)
        r.allowed_planning_time = 8.; r.num_planning_attempts = 2
        r.max_velocity_scaling_factor = ticket.velocity
        r.max_acceleration_scaling_factor = ticket.acceleration
        constraints = Constraints()
        for name in self.names(side):
            j = JointConstraint(joint_name=name, position=float(goal[name]),
                                tolerance_above=.002, tolerance_below=.002, weight=1.)
            constraints.joint_constraints.append(j)
        r.goal_constraints = [constraints]
        result = self.call(self.motion, req, ticket, 35).motion_plan_response
        if result.error_code.val != 1:
            raise PlanningFailure('MOTION_PLAN_FAILED',stage,label+': MoveIt planning failed',
                                  moveit_error_code=result.error_code.val)
        return self.retime(side, q, result.trajectory, ticket, label, stage=stage)

    def tip(self, side, q, wrist_from_tip):
        if getattr(self,'reference_frame','torso_link')=='pelvis':
            return self.model.transform(side+'_wrist_yaw_link',q) @ wrist_from_tip
        torso = self.model.transform('torso_link', q)
        return np.linalg.inv(torso) @ self.model.transform(side+'_wrist_yaw_link', q) @ wrist_from_tip

    def joint_limits(self, side):
        cached=getattr(self,'_joint_limits',None)
        if cached is not None:return cached[side]
        limits={}
        model_limits=getattr(self.model,'limits',{})
        for name in self.names(side):
            if isinstance(model_limits,dict) and name in model_limits:
                limits[name]=model_limits[name]
                continue
            limit=self.model.robot.find(f"joint[@name='{name}']/limit")
            if limit is None:raise PlanningFailure('MODEL_MISMATCH','ik','Missing joint limit: '+name)
            limits[name]=(float(limit.get('lower')),float(limit.get('upper')))
        return limits

    def solve(self, side, q, target, wrist_from_tip, ticket, stage='approach', timeout=.1, rpc_timeout=15.,*,
              continuity_reference=None):
        backend=getattr(self,'ik_backend','moveit')
        if backend in ('bounded','bounded-task','constrained'):
            if ticket.kind!='task' or getattr(self,'reference_frame',None)!='pelvis':
                raise ValueError('Bounded IK experiment requires a pelvis-frame scoped task')
            from task_ik_search import bounded_pose_ik
            options={}
            if backend=='constrained' and stage in ('insert','rotate','withdraw'):
                raise ValueError('Constrained contact IK requires the waypoint task context')
            if backend=='bounded-task' and stage in ('insert','rotate','withdraw'):
                if continuity_reference is None:raise ValueError('Task-tolerance IK needs the actual previous state')
                options=dict(continuity_reference=continuity_reference,position_tolerance=.002,
                             orientation_tolerance=math.radians(2.))
            answer,diagnostic=bounded_pose_ik(self.model,side+'_wrist_yaw_link',self.names(side),
                self.joint_limits(side),q,target,wrist_from_tip,lambda:self.check_plan(ticket),min(timeout,rpc_timeout),
                **options)
            self.last_ik=dict(diagnostic,stage=stage)
            if answer is None:return None
            try:self.validate(answer,ticket,side,stage)
            except PlanningFailure as error:
                if error.diagnostic['code']!='COLLISION':raise
                self.last_ik.update(error.diagnostic)
                return None
            self.last_ik['code']='VALID'
            return answer
        req = GetPositionIK.Request(); r = req.ik_request
        r.group_name = self.group(side); r.ik_link_name = side+'_wrist_yaw_link'
        # Kinematic candidates are never accepted without the scoped validator.
        r.robot_state = robot_state(q)
        # In free-space stages MoveIt can search another IK branch internally.
        # Contact stages still need our request-scoped, distal-index-only ACM.
        r.avoid_collisions = self.ik_avoid_collisions(ticket,stage)
        r.pose_stamped.header.frame_id = getattr(self,'reference_frame','torso_link')
        r.pose_stamped.pose = pose(target @ np.linalg.inv(wrist_from_tip))
        r.timeout.nanosec = int(timeout*1e9)
        began=time.monotonic()
        result = self.call(self.ik, req, ticket, rpc_timeout)
        rpc_seconds=time.monotonic()-began
        self.last_ik={'code':'IK_FAILED','moveit_error_code':result.error_code.val,'stage':stage,
                      'rpc_seconds':rpc_seconds}
        if result.error_code.val != 1: return None
        answer = dict(q)
        solved = dict(zip(result.solution.joint_state.name, result.solution.joint_state.position))
        answer.update({n:solved[n] for n in self.names(side)})
        try:self.validate(answer,ticket,side,stage)
        except PlanningFailure as error:
            if error.diagnostic['code']!='COLLISION':raise
            self.last_ik=dict(error.diagnostic,rpc_seconds=rpc_seconds)
            return None
        self.last_ik={'code':'VALID','stage':stage,'rpc_seconds':rpc_seconds,
                      'validation_seconds':time.monotonic()-began-rpc_seconds}
        return answer

    def ik_avoid_collisions(self,ticket,stage):
        return (getattr(self,'orientation_search','local')=='broad' and
                stage not in ('insert','rotate','withdraw'))

    def retime(self, side, q, path, ticket, label, geometry_check=None, stage=None, *, timed_trajectory=None):
        self.progress(ticket, label+': time parameterization...')
        if timed_trajectory is None:
            req = TimeTaskPath.Request(arm=side+('_waist' if getattr(self,'waist_assist',False) else ''), start_state=robot_state(q), trajectory=path,
                                      velocity_scaling=ticket.velocity,
                                      acceleration_scaling=ticket.acceleration)
            result = self.call(self.timing, req, ticket, 40)
            if not result.success:
                if result.message in ('MoveIt TOTG failed','MoveIt task Ruckig smoothing failed'):
                    raise PlanningFailure('MOTION_PLAN_FAILED',stage or ticket.kind,
                        label+': '+result.message,service='time_task_path',retiming_error=result.message)
                raise RuntimeError(result.message)
            trajectory = result.trajectory.joint_trajectory
        else:
            trajectory=timed_trajectory
        if set(trajectory.joint_names) != set(self.names(side)):
            raise RuntimeError('Unexpected joint ownership in timed trajectory')
        vmax,amax=task_motion_limits(trajectory.joint_names,ticket.velocity,ticket.acceleration,
            getattr(self,'planning_parameters',PlanningParameters()))
        condition_endpoints(trajectory)
        jerk_limits=task_jerk_limits(trajectory.joint_names)
        before=time_seconds(trajectory.points[-1])
        try:stretch=stretch_to_limits(trajectory,vmax,amax,jerk_limits)
        except ValueError as error:
            raise PlanningFailure('MOTION_PLAN_FAILED',stage or ticket.kind,str(error)) from error
        self.search_diagnostics.setdefault('timing',[]).append(dict(label=label,stage=stage,
            method=('joint_line_quintic' if geometry_check is None else 'quintic_fit')
                   if timed_trajectory is not None else 'totg_ruckig',
            duration_before_stretch_s=before,stretch_factor=stretch,
            duration_s=time_seconds(trajectory.points[-1]),jerk_limits_rad_s3=jerk_limits,
            command_velocity_limits_rad_s=vmax))
        validate_jerk_limits(trajectory,jerk_limits)
        if getattr(self,'waist_assist',False):
            lower,upper=self.joint_limits(side)['waist_yaw_joint']
            try:validate_waist_envelope(trajectory,lower+.02,upper-.02,
                                      vmax['waist_yaw_joint'],amax['waist_yaw_joint'])
            except ValueError as error:
                raise PlanningFailure('MOTION_PLAN_FAILED',stage or ticket.kind,str(error)) from error
        record=dict(label=label,stage=stage or ticket.kind,samples=0,status='running',
                    last_progress_sample=0,batch=False)
        self.search_diagnostics.setdefault('performance',{}).setdefault('trajectory_validation',[]).append(record)
        began=time.monotonic()
        try:
            result=self.validate_timed_path(side,q,trajectory,ticket,label,geometry_check,stage,vmax,amax,record)
            record.update(status='passed',last_progress_sample=record['samples'])
            return result
        except Exception as error:
            record.update(status='failed',error_type=type(error).__name__)
            if isinstance(error,PlanningFailure):record['failure']=dict(error.diagnostic)
            raise
        finally:
            record['seconds']=time.monotonic()-began
            if record['status']=='running':record['status']='interrupted'

    def validate_timed_path(self,side,q,trajectory,ticket,label,geometry_check,stage,vmax,amax,record):
        names=self.names(side);trajectory_names=tuple(trajectory.joint_names);limits=self.joint_limits(side)
        lower=np.asarray([limits[name][0]+.02 for name in trajectory_names])
        upper=np.asarray([limits[name][1]-.02 for name in trajectory_names])
        current = dict(q)
        previous = None
        total = sample_count(trajectory)
        record['samples']=total
        self.progress(ticket, f'{label}: validating trajectory 0/{total}')
        def progress(index):
            record['last_progress_sample']=index
            self.progress(ticket,f'{label}: validating trajectory {index}/{total} ({100*index/total:.0f}%)')
        sampled=np.asarray(list(samples(trajectory,vmax,amax)))
        if len(sampled)!=total:raise RuntimeError('Trajectory sample accounting mismatch')
        outside=np.argwhere((sampled<lower)|(sampled>upper))
        if len(outside):
            raise PlanningFailure('JOINT_MARGIN',stage or ticket.kind,
                'Timed path exceeds guarded joint margin: '+trajectory_names[int(outside[0,1])])
        small_steps=(len(sampled)<2 or np.max(np.abs(np.diff(sampled,axis=0)))<=.005+1e-12)
        validation_summary=dict(samples=total,min_guarded_joint_margin_rad=float(
            min(np.min(sampled-lower),np.min(upper-sampled))))
        scoped_batch=(ticket.kind=='task' and hasattr(self,'validate_trajectory_samples') and small_steps)
        generic_batch=(ticket.kind!='task' and geometry_check is None and
                       hasattr(self,'batch_valid') and small_steps)
        if scoped_batch:
            record.update(batch=True,validator='scoped_numpy_fcl')
            try:
                self.validate_trajectory_samples(q,ticket,side,stage,trajectory_names,sampled,geometry_check,progress)
            except PlanningFailure as error:
                index=error.diagnostic.pop('batch_sample_index',None)
                if index is not None:error.diagnostic.update(timed_sample_index=index+1,timed_sample_count=total)
                raise
            current.update(zip(trajectory_names,map(float,sampled[-1])))
            return {'side':side,'label':label,'stage':stage or ticket.kind,'trajectory':trajectory,
                    'start':dict(q),'end':current,'validation':validation_summary}
        if generic_batch:
            record.update(batch=True,validator='full_scene_cpp')
            try:self.validate_trajectory_batch(q,ticket,side,stage,trajectory_names,sampled,
                progress=progress)
            except PlanningFailure as error:
                index=error.diagnostic.pop('batch_sample_index',None)
                if index is not None:error.diagnostic.update(timed_sample_index=index+1,timed_sample_count=total)
                raise
            self.progress(ticket,f'{label}: validating trajectory {total}/{total} (100%)')
            current.update(zip(trajectory_names,map(float,sampled[-1])))
            return {'side':side,'label':label,'stage':stage or ticket.kind,'trajectory':trajectory,
                    'start':dict(q),'end':current,'validation':validation_summary}
        def validate_sample(state, index, fraction=None):
            try:
                self.validate(state, ticket, side, stage)
                if geometry_check: geometry_check(state)
            except PlanningFailure as error:
                error.diagnostic.update(timed_sample_index=index, timed_sample_count=total,
                                        intersample_fraction=fraction)
                raise
        for index,positions in enumerate(sampled,1):
            current.update(zip(trajectory_names, map(float,positions)))
            validate_sample(current,index)
            if previous:
                # Recheck intersample configurations, not just the original IK samples.
                delta = max(abs(current[n]-previous[n]) for n in names)
                for t in np.linspace(0,1,max(2, math.ceil(delta/.005)+1))[1:-1]:
                    sample = dict(previous)
                    sample.update({n:previous[n]+float(t)*(current[n]-previous[n]) for n in names})
                    validate_sample(sample,index,float(t))
            previous = dict(current)
            if index % 100 == 0 or index == total:
                progress(index)
        return {'side':side, 'label':label, 'stage':stage or ticket.kind, 'trajectory':trajectory,
                'start':dict(q), 'end':current,'validation':validation_summary}

    def cartesian(self, side, q, points, nominal, wrist_from_tip, ticket, label, curve, stage, *, preferred_state=None):
        paths=self.cartesian_paths(side,q,points,nominal,wrist_from_tip,ticket,label,curve,stage,
                                   preferred_state=preferred_state)
        try:return next(paths)
        finally:paths.close()

    def cartesian_paths(self, side, q, points, nominal, wrist_from_tip, ticket, label, curve, stage,
                        *, preferred_state=None, branching=False):
        self.progress(ticket, label+': solving continuous Cartesian path...')
        path = RobotTrajectory(); path.joint_trajectory.joint_names = list(self.names(side))
        path.joint_trajectory.points = [JointTrajectoryPoint(positions=[q[n] for n in self.names(side)])]
        params=getattr(self,'planning_parameters',PlanningParameters())
        current = dict(q); policy = params.orientation_policy();limits=self.joint_limits(side)
        constrained=(getattr(self,'ik_backend','moveit')=='constrained' and
                     stage in ('insert','rotate','withdraw'))
        if stage=='approach':policy=dict(policy,tilt_deg=params.approach_tilt_deg,roll_deg=params.approach_twist_deg)
        selection_policy=dict(policy,tilt_deg=max(0.,policy['tilt_deg']-params.planning_margin_deg),
                              roll_deg=max(0.,policy['roll_deg']-params.planning_margin_deg))
        points=list(points);selection=[];checkpoints=[];repairs=[];point_index=0
        continuity=getattr(self,'continuity_policy',None)
        initial_point=self.tip(side,q,wrist_from_tip)[:3,3].copy() if continuity else None
        depths=[0]*len(points);subdivisions=[]
        curve.target=np.asarray(points[-1])
        report={'stage':stage,'label':label,'points':selection,'local_repairs':repairs,
                'planning_angular_margin_deg':params.planning_margin_deg}
        self.search_diagnostics.setdefault('cartesian',[]).append(report)
        if continuity:
            report.update(continuity_policy=continuity.record(),subdivisions=subdivisions)
        def record_rejection():
            counts=report.setdefault('rejection_counts',{})
            code=self.last_ik.get('code','IK_FAILED');counts[code]=counts.get(code,0)+1
            report.setdefault('rejection_examples',{})[code]=dict(self.last_ik,candidate_input=dict(
                point_index=point_index,point_m=np.asarray(point).tolist(),joint_names=list(self.names(side)),
                previous_state=dict(current),seed_state=dict(seed),
                target_tcp_matrix=target.tolist(),wrist_from_tip=np.asarray(wrist_from_tip).tolist(),
                reference_quaternion_xyzw=np.asarray(reference).tolist(),
                previous_quaternion_xyzw=np.asarray(previous).tolist(),
                selection_policy=dict(selection_policy),position_tolerance_m=tolerance,
                solved_state=dict(solved) if solved is not None else None))
        def repair(failed_index,error,*,downstream=False):
            nonlocal current,point_index
            local_count=sum(not item['downstream'] for item in repairs)
            if not downstream and local_count>=params.max_repairs:return False
            lower=-1 if branching else max(-1,failed_index-params.backtrack_points-1)
            for index in range(min(failed_index-1,len(checkpoints)-1),lower,-1):
                remaining=checkpoints[index]
                if not remaining:continue
                current,score=remaining.pop(0)
                del checkpoints[index+1:];del selection[index+1:]
                selection[index]=dict(selection[index],score=score,locally_reselected=True)
                path.joint_trajectory.points=path.joint_trajectory.points[:index+1]+[
                    JointTrajectoryPoint(positions=[current[n] for n in self.names(side)])]
                point_index=index+1
                repairs.append({'failed_point':failed_index,'restart_point':point_index,
                                'reselected_point':index,'reason':error.diagnostic,'downstream':downstream})
                mode='cross-stage backtrack' if downstream else f'local repair {local_count+1}/{params.max_repairs}'
                self.progress(ticket,f'{label}: {mode} at point {index+1}/{len(points)}')
                return True
            return False
        while True:
            if branching:self.check_contact_search_budget()
            if point_index==len(points):
                geometry_report={'stage':stage,'label':label,'checked_samples':0,
                                 'max_position_error_m':0.,'max_tilt_deg':0.,'max_twist_deg':0.}
                self.search_diagnostics.setdefault('retimed_geometry',[]).append(geometry_report)
                def check(state):
                    self.check_tcp_geometry(side,state,wrist_from_tip,nominal,curve,policy,stage,label,geometry_report)
                check.wrist_from_tip=wrist_from_tip
                check.batch=lambda tips:self.check_tcp_geometry_batch(
                    tips,nominal,curve,policy,stage,label,geometry_report)
                try:
                    result=self.retime_cartesian(side,q,path,ticket,label,check,stage,geometry_report)
                    residual=float(np.linalg.norm(self.tip(side,result['end'],wrist_from_tip)[:3,3]-points[-1]))
                    if residual>params.target_tolerance_mm/1000.+1e-7:
                        raise PlanningFailure('RETIMED_TCP_BOUNDS',stage,'Segment target outside position tolerance',
                            tcp_position_m=self.tip(side,result['end'],wrist_from_tip)[:3,3].tolist())
                    result['geometry_validation']=dict(geometry_report,endpoint_position_error_m=residual)
                    downstream=yield result
                    if not branching:return
                    if not repair(len(points),downstream,downstream=True):return
                    continue
                except PlanningFailure as error:
                    if error.diagnostic['code']!='RETIMED_TCP_BOUNDS':raise
                    position=np.asarray(error.diagnostic['tcp_position_m'])
                    closest=int(np.argmin(np.linalg.norm(np.asarray(points)-position,axis=1)))
                    if not repair(closest+1,error):raise
                    continue
            point=points[point_index]
            reference = nominal(point) if callable(nominal) else nominal
            previous = Rotation.from_matrix(self.tip(side,current,wrist_from_tip)[:3,:3]).as_quat()
            # Free-space approach rotates from the measured ready orientation;
            # contact heading constraints apply after this transition, not at t=0.
            candidates = (TaskPlanner.cartesian_orientations(self,reference,previous,selection_policy)
                          if stage=='approach' else self.cartesian_orientations(reference,previous,selection_policy))
            if constrained:candidates=[reference]
            answer = None; best_score=None;evaluated=0;fast_path=False;accepted=[]
            continuity_rejected=False
            tolerance=(params.target_tolerance_mm if point_index==len(points)-1 else params.path_tolerance_mm)/1000.
            # Search nearby XYZ only when nominal continuation has no accepted solution.
            offsets=[np.zeros(3)]
            if params.position_search_fraction and not constrained:
                offsets.extend(axis*sign*tolerance*params.position_search_fraction for axis in np.eye(3) for sign in (-1,1))
            trials=((offset,quat,index) for offset in offsets for index,quat in enumerate(candidates))
            # Sparse alternate seeds keep redundant joint branches without multiplying every IK call.
            branch_point=branching and (point_index%10==0 or point_index==len(points)-1)
            seeds=self.contact_seeds(side,current) if branch_point else []
            trials=[(offset,quat,index,current) for offset,quat,index in trials]
            if candidates:
                seed_quat=min(candidates,key=lambda quat:(Rotation.from_quat(reference).inv()*Rotation.from_quat(quat)).magnitude())
                trials.extend((np.zeros(3),seed_quat,len(candidates)+i,seed) for i,seed in enumerate(seeds))
                if continuity and point_index>0 and preferred_state is None and not constrained:
                    from task_path_continuity import continuation_seed
                    previous_point=points[point_index-1]
                    before_point=points[point_index-2] if point_index>1 else initial_point
                    predicted=continuation_seed(self.names(side),limits,current,
                        [p.positions for p in path.joint_trajectory.points[-2:]],
                        float(np.linalg.norm(np.asarray(previous_point)-before_point)),
                        float(np.linalg.norm(np.asarray(point)-previous_point)),continuity)
                    if predicted is not None:trials.insert(0,(np.zeros(3),seed_quat,-1,predicted))
            for offset,quat,index,seed in trials:
                if not branching and index==0 and answer is not None and np.any(offset):break
                if branching and answer is not None and np.any(offset):continue
                if branching:self.check_contact_search_budget()
                evaluated+=1
                target = np.eye(4); target[:3,3] = point+offset
                target[:3,:3] = Rotation.from_quat(quat).as_matrix()
                if branching and not constrained and not self.contact_pose_ok(target,stage):
                    counts=report.setdefault('rejection_counts',{})
                    counts['target_contact_heading']=counts.get('target_contact_heading',0)+1
                    continue
                if preferred_state is not None:
                    solved=self.solve_approach(side,current,target,wrist_from_tip,ticket,preferred_state(point))
                elif constrained:
                    solved=self.solve_constrained_contact(side,seed,current,point,reference,wrist_from_tip,
                                                          ticket,stage,selection_policy,tolerance)
                else:
                    options=({'continuity_reference':current} if getattr(self,'ik_backend','moveit')=='bounded-task' else {})
                    solved=self.solve(side,seed,target,wrist_from_tip,ticket,stage,**options)
                if solved is None:
                    record_rejection()
                    continue
                step=max(abs(solved[n]-current[n]) for n in self.names(side))
                step_limit=continuity.max_joint_step_rad if continuity else .12
                if step>step_limit:
                    continuity_rejected=True
                    self.last_ik={'code':'JOINT_CONTINUITY','stage':stage,'max_step_rad':step,
                                  'limit_rad':step_limit}
                    record_rejection()
                    continue
                actual = self.tip(side,solved,wrist_from_tip)
                residual=float(np.linalg.norm(actual[:3,3]-point))
                if residual>tolerance:
                    self.last_ik={'code':'TCP_RESIDUAL','stage':stage,'residual_m':residual,'limit_m':tolerance}
                    record_rejection()
                    continue
                actual_rotation=Rotation.from_matrix(actual[:3,:3])
                orientation_ok=(TaskPlanner.orientation_ok(self,reference,actual_rotation.as_quat(),selection_policy)
                                if stage=='approach' else self.orientation_ok(reference,actual_rotation.as_quat(),selection_policy))
                contact_ok=self.contact_pose_ok(actual,stage)
                change_deg=math.degrees((Rotation.from_quat(previous).inv()*actual_rotation).magnitude())
                if (not orientation_ok or not contact_ok or change_deg>policy['step_deg']+math.degrees(1e-4)):
                    tilt,twist=orientation_error(reference,actual_rotation.as_quat())
                    exceeded=[]
                    if tilt>selection_policy['tilt_deg']+1e-4:exceeded.append('tilt')
                    if twist>selection_policy['roll_deg']+1e-4:exceeded.append('twist')
                    if not contact_ok:exceeded.append('contact_heading')
                    if not orientation_ok and not exceeded:exceeded.append('heading')
                    if change_deg>policy['step_deg']+math.degrees(1e-4):exceeded.append('adjacent_rotation')
                    self.last_ik={'code':'ORIENTATION_CONTINUITY','stage':stage,'exceeded':exceeded,
                        'tilt_deg':tilt,'tilt_limit_deg':selection_policy['tilt_deg'],
                        'twist_deg':twist,'twist_limit_deg':selection_policy['roll_deg'],
                        'adjacent_rotation_deg':change_deg,'adjacent_rotation_limit_deg':policy['step_deg']}
                    if not contact_ok and hasattr(self,'tangent_errors') and self.tangent_mode():
                        self.last_ik.update(tangent_error_deg=float(self.tangent_errors([actual])[0]),
                                            tangent_limit_deg=5.-params.planning_margin_deg)
                    record_rejection()
                    counts=report.setdefault('rejection_counts',{})
                    for code in exceeded:counts[code]=counts.get(code,0)+1
                    continue
                score=solution_score(self.names(side),limits,current,solved,previous,actual_rotation.as_quat())
                if score is None:
                    self.last_ik={'code':'JOINT_MARGIN','stage':stage}
                    record_rejection()
                    continue
                slope_peak=0.
                if continuity and point_index>0:
                    from task_path_continuity import slope_error
                    previous_point=points[point_index-1]
                    before_point=points[point_index-2] if point_index>1 else initial_point
                    error=slope_error([p.positions for p in path.joint_trajectory.points[-2:]],
                        [solved[n] for n in self.names(side)],
                        float(np.linalg.norm(np.asarray(previous_point)-before_point)),
                        float(np.linalg.norm(np.asarray(point)-previous_point)))
                    slope_peak=float(np.max(np.abs(error)))
                    penalty=continuity.slope_weight*float(error@error)
                    score=dict(score,total=score['total']+penalty,slope_penalty=penalty,
                               max_slope_error_rad=slope_peak)
                if branching:
                    margins=np.array([min(solved[n]-limits[n][0],limits[n][1]-solved[n]) for n in self.names(side)])
                    penalty=.01*float(np.sum(1./np.maximum(margins,.02)**2))
                    score=dict(score,total=score['total']+penalty,contact_limit_penalty=penalty)
                    if any(max(abs(solved[n]-other[n]) for n in self.names(side))<.005 for other,_ in accepted):continue
                accepted.append((solved,score))
                if best_score is None or score['total']<best_score['total']:
                    answer,best_score=solved,score
                # Original workbench fast path: a tiny continuation safely away
                # from limits does not need additional angular perturbation calls.
                if (not branch_point and params.small_step_fast_path and index in (-1,0) and
                        score['max_joint_step_rad']<.04 and score['limit_penalty']==0 and
                        (not continuity or slope_peak<continuity.fast_slope_error_rad)):
                    fast_path=True;break
            if answer is None:
                if continuity and continuity_rejected:
                    from task_path_continuity import subdivision_point
                    previous_point=points[point_index-1] if point_index else initial_point
                    midpoint=subdivision_point(previous_point,point,depths[point_index],len(subdivisions),continuity)
                    if midpoint is not None:
                        depth=depths[point_index]+1
                        points.insert(point_index,midpoint);depths[point_index:point_index+1]=[depth,depth]
                        subdivisions.append(dict(point=point_index,depth=depth,target_m=midpoint.tolist()))
                        continue
                error=PlanningFailure('CONTINUOUS_IK_FAILED',stage,
                    label+': no accepted continuous IK candidate',last_candidate=self.last_ik,point_m=np.asarray(point).tolist())
                if repair(point_index,error):continue
                raise error
            current = answer
            ordered=sorted(accepted,key=lambda item:item[1]['total'])
            if branching:
                diverse=[ordered[0]]
                for item in ordered[1:]:
                    if all(max(abs(item[0][n]-other[0][n]) for n in self.names(side))>=.02 for other in diverse):
                        diverse.append(item)
                    if len(diverse)==5:break
                selected={id(item) for item in diverse}
                diverse.extend(item for item in ordered if id(item) not in selected)
                checkpoints.append(diverse[1:5])
            else:checkpoints.append(ordered[1:])
            selection.append({'point':point_index,'evaluated':evaluated,'fast_path':fast_path,'score':best_score})
            path.joint_trajectory.points.append(JointTrajectoryPoint(positions=[current[n] for n in self.names(side)]))
            point_index+=1

    def retime_cartesian(self,side,q,path,ticket,label,check,stage,geometry_report):
        params=getattr(self,'planning_parameters',PlanningParameters())
        enabled=(params.smooth_contact_path and stage in ('rotate','withdraw') or
                 params.smooth_insert_path and stage=='insert' or
                 params.smooth_joint_path and stage=='approach')
        if not (enabled and ticket.kind=='task'):
            return self.retime(side,q,path,ticket,label,check,stage)
        from task_path_smoothing import smooth_candidates
        names=path.joint_trajectory.joint_names
        positions=np.asarray([p.positions for p in path.joint_trajectory.points])
        tips=[]
        for values in positions:
            state=dict(q);state.update(zip(names,values))
            tips.append(self.tip(side,state,check.wrist_from_tip)[:3,3])
        progress=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(tips,axis=0),axis=1))]
        record=dict(label=label,stage=stage,joint_names=list(names),raw_positions=positions.tolist(),
                    cartesian_progress_m=progress.tolist(),attempts=[],method='totg_ruckig_fallback')
        self.search_diagnostics.setdefault('path_smoothing',[]).append(record)
        original_report=dict(geometry_report)
        if progress[-1]>1e-9:
            for trajectory,fit in smooth_candidates(names,positions,progress,params.smoothing_joint_deviation_deg):
                geometry_report.clear();geometry_report.update(original_report)
                record['attempts'].append(fit)
                try:
                    result=self.retime(side,q,path,ticket,label,check,stage,timed_trajectory=trajectory)
                except PlanningFailure as error:
                    if error.diagnostic['code'] not in ('RETIMED_TCP_BOUNDS','COLLISION','JOINT_MARGIN','MOTION_PLAN_FAILED'):
                        raise
                    fit['rejected']=error.diagnostic
                    continue
                fit.update(accepted=True,duration_s=time_seconds(trajectory.points[-1]))
                record['method']='quintic_fit'
                return result
        geometry_report.clear();geometry_report.update(original_report)
        return self.retime(side,q,path,ticket,label,check,stage)

    def check_tcp_geometry(self,side,state,wrist_from_tip,nominal,curve,policy,stage,label,report):
        tip=self.tip(side,state,wrist_from_tip)
        reference=nominal(tip[:3,3]) if callable(nominal) else nominal
        actual=Rotation.from_matrix(tip[:3,:3]).as_quat()
        distance=float(curve(tip[:3,3]))
        tilt,twist=orientation_error(reference,actual)
        if not np.isfinite([distance,tilt,twist]).all():
            raise PlanningFailure('INVALID_TCP_GEOMETRY',stage,'Nonfinite TCP geometry; not retryable')
        report['checked_samples']+=1
        for key,value in (('max_position_error_m',distance),('max_tilt_deg',tilt),('max_twist_deg',twist)):
            report[key]=max(report[key],value)
        exceeded=[]
        limit=self.position_limit(tip[:3,3],curve)
        if distance>limit:exceeded.append('position')
        if tilt>policy['tilt_deg']+1e-4:exceeded.append('tilt')
        if twist>policy['roll_deg']+1e-4:exceeded.append('twist')
        if exceeded:
            error=PlanningFailure('RETIMED_TCP_BOUNDS',stage,
                f'{label}: retimed TCP bounds ({", ".join(exceeded)}): '
                f'position {distance*1000:.4f}/{limit*1000:.4f} mm, '
                f'tilt {tilt:.6f}/{policy["tilt_deg"]:.6f} deg, '
                f'twist {twist:.6f}/{policy["roll_deg"]:.6f} deg',
                exceeded=exceeded,position_error_m=distance,position_limit_m=float(limit),
                tilt_deg=tilt,tilt_limit_deg=policy['tilt_deg'],
                twist_deg=twist,twist_limit_deg=policy['roll_deg'],angular_epsilon_deg=1e-4,
                tcp_position_m=tip[:3,3].tolist(),actual_quaternion_xyzw=actual.tolist(),
                reference_quaternion_xyzw=np.asarray(reference).tolist())
            report['failure']=error.diagnostic
            raise error

    def check_tcp_geometry_batch(self,tips,nominal,curve,policy,stage,label,report):
        points=np.asarray(tips)[:,:3,3]
        if callable(nominal):
            references=(nominal.batch(points) if hasattr(nominal,'batch') else
                        np.asarray([nominal(point) for point in points]))
        else:references=np.broadcast_to(np.asarray(nominal),(len(points),4))
        actual=Rotation.from_matrix(np.asarray(tips)[:,:3,:3]).as_quat()
        distances=(curve.batch(points) if hasattr(curve,'batch') else
                   np.asarray([curve(point) for point in points]))
        relative=Rotation.from_quat(references).inv()*Rotation.from_quat(actual)
        matrices=relative.as_matrix();quaternions=relative.as_quat()
        tilts=np.degrees(np.arccos(np.clip(matrices[:,2,2],-1.,1.)))
        twists=np.degrees(np.abs((2*np.arctan2(quaternions[:,2],quaternions[:,3])+math.pi)%(2*math.pi)-math.pi))
        if not np.isfinite(distances).all() or not np.isfinite(tilts).all() or not np.isfinite(twists).all():
            raise PlanningFailure('INVALID_TCP_GEOMETRY',stage,'Nonfinite TCP geometry; not retryable')
        report['checked_samples']+=len(points)
        report['max_position_error_m']=max(report['max_position_error_m'],float(np.max(distances)))
        report['max_tilt_deg']=max(report['max_tilt_deg'],float(np.max(tilts)))
        report['max_twist_deg']=max(report['max_twist_deg'],float(np.max(twists)))
        limits=self.position_limit(points,curve)
        exceeded=(distances>limits)|(tilts>policy['tilt_deg']+1e-4)|(twists>policy['roll_deg']+1e-4)
        indices=np.flatnonzero(exceeded)
        if not len(indices):return None
        index=int(indices[0]);fields=[]
        limit=float(np.broadcast_to(limits,distances.shape)[index])
        if distances[index]>limit:fields.append('position')
        if tilts[index]>policy['tilt_deg']+1e-4:fields.append('tilt')
        if twists[index]>policy['roll_deg']+1e-4:fields.append('twist')
        error=PlanningFailure('RETIMED_TCP_BOUNDS',stage,
            f'{label}: retimed TCP bounds ({", ".join(fields)}): '
            f'position {distances[index]*1000:.4f}/{limit*1000:.4f} mm, '
            f'tilt {tilts[index]:.6f}/{policy["tilt_deg"]:.6f} deg, '
            f'twist {twists[index]:.6f}/{policy["roll_deg"]:.6f} deg',
            exceeded=fields,position_error_m=float(distances[index]),position_limit_m=limit,
            tilt_deg=float(tilts[index]),tilt_limit_deg=policy['tilt_deg'],
            twist_deg=float(twists[index]),twist_limit_deg=policy['roll_deg'],angular_epsilon_deg=1e-4,
            tcp_position_m=points[index].tolist(),actual_quaternion_xyzw=actual[index].tolist(),
            reference_quaternion_xyzw=np.asarray(references[index]).tolist(),batch_sample_index=index)
        report['failure']=error.diagnostic
        return error

    def position_limit(self,points,curve):
        params=getattr(self,'planning_parameters',PlanningParameters())
        path=params.path_tolerance_mm/1000.
        target=params.target_tolerance_mm/1000.
        if not hasattr(curve,'target'):return path
        return np.where(np.linalg.norm(np.asarray(points)-curve.target,axis=-1)<=target,target,path)

    def cartesian_orientations(self, nominal, previous, policy):
        return orientation_candidates(nominal, previous, policy)

    def orientation_ok(self,nominal,actual,policy):
        return orientation_allowed(nominal,actual,policy,epsilon=1e-4)

    def preinsert_rotations(self, nominal, policy):
        return (broad_preinsert_orientations(nominal) if getattr(self,'orientation_search','local')=='broad'
                else preinsert_orientations(nominal,policy))

    def contact_hand_setup(self, side, start, geometry):
        wrist = np.asarray(geometry['fingertip_estimates']['sides'][side]['states']['open']
                           ['wrist_yaw_from_index_tcp']['matrix_row_major'])
        return 'index_only', wrist

    def rank_preinsert(self, side, start, pre, entry, wrist_from_tip, ticket):
        return next(self.preinsert_batches(side,start,pre,entry,wrist_from_tip,ticket,incremental=False))

    def complete_candidates(self, side, start, pre, entry, wrist_from_tip, ticket):
        if not getattr(self,'incremental_search',False):
            yield from self.rank_preinsert(side,start,pre,entry,wrist_from_tip,ticket)
            return
        batches=self.preinsert_batches(side,start,pre,entry,wrist_from_tip,ticket)
        try:
            for batch in batches:yield from batch
        finally:batches.close()

    def configure_contact_geometry(self,center,entry):
        pass

    def contact_pose_ok(self,tip,stage):
        return True

    def preinsert_score(self,side,start,answer,previous,actual):
        return solution_score(self.names(side),self.joint_limits(side),start,answer,previous,actual)

    def preinsert_trial_rotation(self,quaternion,position):
        return quaternion

    def preinsert_batches(self, side, start, pre, entry, wrist_from_tip, ticket, *, incremental=True, vertical=False):
        params=getattr(self,'planning_parameters',PlanningParameters())
        search_policy=getattr(self,'preinsert_policy','legacy')
        if search_policy not in ('legacy','diverse'):raise ValueError('Unknown preinsert search policy')
        policy=params.orientation_policy()
        selection_policy=dict(policy,tilt_deg=max(0.,policy['tilt_deg']-params.planning_margin_deg),
                              roll_deg=max(0.,policy['roll_deg']-params.planning_margin_deg))
        nominal=Rotation.from_matrix(pre[:3,:3]).as_quat()
        broad=getattr(self,'orientation_search','local')=='broad'
        orientations=self.preinsert_rotations(nominal,selection_policy)
        limits=self.joint_limits(side)
        seeds=preinsert_seeds(side,start,limits)
        previous=Rotation.from_matrix(self.tip(side,start,wrist_from_tip)[:3,:3]).as_quat()
        vertical_distance=pre[2,3]-entry[2]
        if vertical_distance<=0:raise PlanningFailure('INVALID_APPROACH','approach','Preinsert must be above entry')
        heights=(.10,.08,.06)
        started=time.monotonic();paused=0.
        report={'orientation_count':len(orientations),'seed_count':len(seeds),
                'orientation_search':'broad' if broad else 'local',
                'total_combinations':len(orientations)*len(heights)*len(seeds),
                'max_attempts':self.search_attempts,'budget_seconds':self.search_seconds,
                'trials':[],'selection_scope':'sampled collision-checked IK candidates',
                'stop_reason':'grid_exhausted','selected':None,'batches':[],
                'candidate_policy':search_policy,
                'search_policy':f'incremental_{params.batch_trials}_trials' if incremental else 'exhaustive_budget'}
        self.search_diagnostics['preinsert']=report
        ranked={};delivered=set();delivered_candidates=[]
        def ordered_candidates():
            ordered=sorted((c for k,c in ranked.items() if k not in delivered),key=lambda c:c['score']['total'])
            if broad:
                diverse=[];seen=set()
                for candidate in ordered:
                    if candidate['orientation_index'] not in seen:
                        diverse.append(candidate);seen.add(candidate['orientation_index'])
                selected_ids={c['trial_index'] for c in diverse}
                ordered=diverse+[c for c in ordered if c['trial_index'] not in selected_ids]
            return ordered
        def describe(batch):
            return [{'trial_index':c['trial_index'],'score':c['score'],'goal':c['goal'],
                     'orientation_index':c['orientation_index'],
                     'preinsert_tcp_matrix':c['pre'].tolist(),
                     'orientation_reference_xyzw':np.asarray(c['nominal']).tolist()} for c in batch]
        def select_batch(count):
            ordered=ordered_candidates()
            return (diverse_preinsert_batch(ordered,delivered_candidates,self.names(side),count)
                    if search_policy=='diverse' else ordered[:count])
        try:
            for orientation,height_index,seed_index in preinsert_jobs(len(orientations),len(seeds),
                    policy=search_policy,initial_trials=params.batch_trials):
                if incremental and report['trials'] and len(report['trials'])%params.batch_trials==0:
                    batch=select_batch(params.batch_candidates)
                    if batch:
                        report['batches'].append(dict(attempted=len(report['trials']),candidates=describe(batch)))
                        delivered.update(tuple(round(c['goal'][n],4) for n in self.names(side)) for c in batch)
                        delivered_candidates.extend(batch)
                        suspended=time.monotonic()
                        try:yield batch
                        finally:paused+=time.monotonic()-suspended
                remaining=self.search_seconds-(time.monotonic()-started-paused)
                if remaining<.01:
                    report['stop_reason']='time_budget';break
                if len(report['trials'])>=self.search_attempts:
                    report['stop_reason']='attempt_budget';break
                height=heights[height_index]
                trial=pre.copy()
                trial[:3,3]=entry+(pre[:3,3]-entry)*(height/vertical_distance)
                quaternion=self.preinsert_trial_rotation(orientations[orientation],trial[:3,3])
                if quaternion is None:
                    report['trials'].append(dict(code='TARGET_TANGENT_DEGENERATE',orientation_candidate=orientation,
                        seed=seed_index,preinsert_height_m=height,tcp_position_m=trial[:3,3].tolist()))
                    continue
                trial[:3,:3]=Rotation.from_quat(quaternion).as_matrix()
                # Preserve the preview's approach ray independently of orientation.
                if len(report['trials'])%10==0:
                    self.progress(ticket,f'Approach: IK search {len(report["trials"])}/{self.search_attempts}, '
                        f'{time.monotonic()-started-paused:.1f}/{self.search_seconds:.0f}s, {len(ranked)} valid candidates')
                answer=self.solve(side,seeds[seed_index],trial,wrist_from_tip,ticket,'approach',
                                  timeout=min(.06,remaining),rpc_timeout=remaining)
                record=dict(self.last_ik,orientation_candidate=orientation,seed=seed_index,
                            preinsert_height_m=height,quaternion_xyzw=np.asarray(quaternion).tolist(),
                            tcp_position_m=trial[:3,3].tolist())
                report['trials'].append(record)
                if answer is None:continue
                actual=self.tip(side,answer,wrist_from_tip)
                actual_quat=Rotation.from_matrix(actual[:3,:3]).as_quat()
                residual=float(np.linalg.norm(actual[:3,3]-trial[:3,3]))
                reference=quaternion if broad else nominal
                if (residual>.002 or not self.orientation_ok(reference,actual_quat,selection_policy)
                        or not self.contact_pose_ok(actual,'preinsert')):
                    record.update(code='TCP_RESIDUAL_OR_ORIENTATION',residual_m=residual);continue
                score=self.preinsert_score(side,start,answer,previous,actual_quat)
                if score is None:
                    record['code']='JOINT_MARGIN';continue
                record['score']=score
                key=tuple(round(answer[n],4) for n in self.names(side))
                candidate={'goal':answer,'pre':trial,'score':score,'trial_index':len(report['trials'])-1,
                           'nominal':reference,'orientation_index':orientation}
                if key not in ranked or score['total']<ranked[key]['score']['total']:ranked[key]=candidate
        except Exception:
            report['stop_reason']='interrupted'
            raise
        finally:
            report['elapsed_seconds']=time.monotonic()-started-paused
            report['attempted']=len(report['trials'])
            report['covered_orientations']=len({t['orientation_candidate'] for t in report['trials']})
            report['covered_seeds']=len({t['seed'] for t in report['trials']})
            report['covered_heights_m']=sorted({t['preinsert_height_m'] for t in report['trials']})
            report['valid_candidates']=len(ranked)
            report['truncated']=report['stop_reason']!='grid_exhausted'
        ordered=select_batch(params.complete_candidates)
        report['ranked_candidates']=describe(ordered)
        if not ranked:
            counts={}
            for trial in report['trials']:counts[trial['code']]=counts.get(trial['code'],0)+1
            report['failure_counts']=counts
            raise PlanningFailure('PREINSERT_IK_FAILED','approach',
                f'No valid candidate in {report["attempted"]} trials ({report["stop_reason"]}); {counts}',search=report)
        if ordered:yield ordered

    def ring(self, side, start, geometry, ticket):
        self.search_diagnostics={}
        self._contact_search_deadline=None
        self._contact_candidate_deadline=None
        require_task_hand_model(self.model)
        if not geometry['geometry_valid_for_planning_seed']:
            raise RuntimeError('Vision geometry gate rejected this target')
        candidate = geometry['grasp_candidates'][side]['poses']
        contact_posture, wrist_from_tip = self.contact_hand_setup(side,start,geometry)
        pre = np.asarray(candidate['preinsert_index_tcp']['matrix_row_major'])
        entry = np.asarray(geometry['task_preview']['insertion_reference_m'])
        task = geometry['task_preview']
        center = np.asarray(task.get('arc_center_m',task['axis_center_m']))
        physical_center = np.asarray(task['axis_center_m'])
        if getattr(self,'reference_frame','torso_link')=='pelvis':
            frozen=capture_transform(geometry,self.model,start)
            pre=frozen @ pre
            entry=frozen[:3,:3] @ entry+frozen[:3,3]
            center=frozen[:3,:3] @ center+frozen[:3,3]
            physical_center=frozen[:3,:3] @ physical_center+frozen[:3,3]
            if not np.allclose(frozen[:3,2],[0,0,1],atol=1e-5):
                raise PlanningFailure('CAPTURE_TILT','capture','Horizontal-plane task requires fixed waist roll/pitch')
            self.search_diagnostics['capture_frame']={'frame':'pelvis','pelvis_from_torso_row_major':frozen.tolist()}
        self.configure_contact_geometry(physical_center,entry)
        nominal = Rotation.from_matrix(pre[:3,:3]).as_quat()
        search_start=dict(start)
        hands=getattr(self,'articulated_hands',False)
        if hands:
            from task_hand_control import posture_targets
            search_start.update(model_joints(side,posture_targets(contact_posture,getattr(self,'dring_hand_settings',None))))
        candidates=self.complete_candidates(side,search_start,pre,entry,wrist_from_tip,ticket)
        failures=[]
        params=getattr(self,'planning_parameters',PlanningParameters())
        count=params.complete_candidates if getattr(self,'orientation_search','local')=='broad' else min(3,params.complete_candidates)
        from itertools import islice
        try:
            for attempt,candidate in enumerate(islice(candidates,count),1):
                self.search_diagnostics['preinsert']['approach_failures']=failures
                self.progress(ticket,f'Checking complete candidate {attempt}/{count} maximum...')
                try:
                    goal=dict(start)
                    goal.update({n:candidate['goal'][n] for n in self.names(side)})
                    approach=(self.center_approach(side,start,candidate['pre'],physical_center,wrist_from_tip,ticket,goal)
                              if params.approach_via_center else
                              self.joint_path(side,start,goal,ticket,'Approach',stage='approach'))
                    if hands:hand_stage=self.hand_transition(side,approach['end'],contact_posture,ticket)
                    selected_nominal=candidate.get('nominal',nominal)
                    prefix=[approach]+([hand_stage] if hands else [])
                    segments=self.ring_contact_path(side,start,candidate['pre'],entry,center,
                        selected_nominal,wrist_from_tip,ticket,prefix,hands)
                except PlanningFailure as error:
                    if error.diagnostic['code'] not in ('MOTION_PLAN_FAILED','COLLISION','CONTINUOUS_IK_FAILED',
                                                       'RETIMED_TCP_BOUNDS','CONTACT_CANDIDATE_BUDGET'):raise
                    error.diagnostic.update(candidate_attempt=attempt,trial_index=candidate['trial_index'])
                    failures.append(error.diagnostic)
                    self.progress(ticket,f'Candidate {attempt} rejected: {error}. Trying remaining candidates...')
                    continue
                pre=candidate['pre']
                self.search_diagnostics['preinsert']['selected']={
                    'trial_index':candidate['trial_index'],'score':candidate['score'],
                    'preinsert_tcp_matrix':pre.tolist(),'orientation_reference_xyzw':np.asarray(selected_nominal).tolist()}
                self.search_diagnostics['preinsert']['stop_reason']='complete_candidate_accepted'
                break
            else:
                self.search_diagnostics['preinsert']['approach_failures']=failures
                raise PlanningFailure('APPROACH_PATH_FAILED','approach','Top ranked IK goals have no complete accepted path',failures=failures)
        except Exception:
            report=self.search_diagnostics.get('preinsert',{})
            if report.get('stop_reason')=='grid_exhausted':report['stop_reason']='complete_path_failed_or_cancelled'
            raise
        finally:
            candidates.close()
        self.search_diagnostics['preinsert']['approach_failures']=failures
        return segments

    def solve_approach(self,side,current,target,wrist_from_tip,ticket,preferred):
        """Bounded local least-squares continuation with a redundant posture bias."""
        from scipy.optimize import least_squares
        names=self.names(side);limits=self.joint_limits(side)
        seed=np.array([current[n] for n in names]);desired=np.array([preferred[n] for n in names])
        lower=np.array([max(limits[n][0]+.021,current[n]-.10) for n in names])
        upper=np.array([min(limits[n][1]-.021,current[n]+.10) for n in names])
        rotation=Rotation.from_matrix(target[:3,:3])
        def residual(values):
            state=dict(current);state.update(zip(names,map(float,values)))
            tip=self.tip(side,state,wrist_from_tip)
            return np.r_[500*(tip[:3,3]-target[:3,3]),
                         (rotation.inv()*Rotation.from_matrix(tip[:3,:3])).as_rotvec(),
                         3*(values-desired)]
        result=least_squares(residual,np.clip(seed,lower,upper),bounds=(lower,upper),max_nfev=35)
        answer=dict(current);answer.update(zip(names,map(float,result.x)))
        try:self.validate(answer,ticket,side,'approach')
        except PlanningFailure as error:
            if error.diagnostic['code'] not in ('COLLISION','JOINT_MARGIN'):raise
            self.last_ik=error.diagnostic;return None
        self.last_ik=dict(code='LOCAL_APPROACH_IK',stage='approach',evaluations=result.nfev)
        return answer

    def center_approach(self,side,start,pre,physical_center,wrist_from_tip,ticket,goal):
        from task_center_approach import approach_geometry
        params=getattr(self,'planning_parameters',PlanningParameters())
        initial=self.tip(side,start,wrist_from_tip)
        def preferred(fraction):
            state=dict(start)
            state.update({n:float(start[n]+fraction*(goal[n]-start[n])) for n in self.names(side)})
            return state
        def reference(fraction):
            return Rotation.from_matrix(self.tip(side,preferred(fraction),wrist_from_tip)[:3,:3]).as_quat()
        points,nominal,curve,record=approach_geometry(initial,pre,physical_center,
            params.approach_center_height_mm/1000.,params.line_points,params.step_deg,reference)
        record['orientation_policy']='continuous_joint_reference_with_local_posture_bias'
        self.search_diagnostics['approach_route']=record
        result=self.cartesian(side,start,points,nominal,wrist_from_tip,ticket,'Approach',curve,'approach',
                              preferred_state=lambda point:preferred(nominal.fraction(point)))
        # A path close to either leg must also actually visit their shared waypoint.
        positions=[]
        trajectory=result['trajectory']
        vmax,amax=task_motion_limits(trajectory.joint_names,ticket.velocity,ticket.acceleration,params)
        for values in samples(trajectory,vmax,amax):
            state=dict(start);state.update(zip(trajectory.joint_names,map(float,values)))
            positions.append(self.tip(side,state,wrist_from_tip)[:3,3])
        distance=float(np.min(np.linalg.norm(np.asarray(positions)-record['center_above_m'],axis=1)))
        record['closest_center_waypoint_m']=distance
        if distance>params.path_tolerance_mm/1000.:
            raise PlanningFailure('RETIMED_TCP_BOUNDS','approach','Approach missed center-above waypoint',
                                  tcp_position_m=record['center_above_m'])
        return result

    def ring_contact_path(self,side,start,pre,entry,center,nominal,wrist_from_tip,ticket,segments,hands):
        target=getattr(self,'ready_target',None)
        if target is None:target=start
        def finish(state):
            suffix=[]
            if hands:
                suffix.append(self.hand_transition(side,state,'open',ticket))
                state=suffix[-1]['end']
            suffix.append(self.joint_path(side,state,target,ticket,'Return ready',stage='return_ready'))
            return suffix
        # Keep the contact generators alive until their exit has a validated return.
        contact=self.contact_segments(side,segments[-1]['end'],pre,entry,center,nominal,
                                      wrist_from_tip,ticket,terminal=finish)
        return segments+contact

    def contact_seeds(self,side,current):
        limits=self.joint_limits(side)
        result=[]
        for name in ('waist_yaw_joint',side+'_shoulder_yaw_joint',side+'_elbow_joint'):
            if name not in limits:continue
            lo,hi=limits[name]
            for offset in (-.08,.08):
                seed=dict(current)
                seed[name]=float(np.clip(current[name]+offset,lo+.021,hi-.021))
                if abs(seed[name]-current[name])>1e-5:result.append(seed)
        return result

    def check_contact_search_budget(self):
        deadline=getattr(self,'_contact_search_deadline',None)
        if deadline is not None and time.monotonic()>=deadline:
            raise PlanningFailure('CONTACT_SEARCH_BUDGET','contact',
                'Cross-stage contact search time budget exhausted; constraints were not relaxed')
        candidate_deadline=getattr(self,'_contact_candidate_deadline',None)
        if candidate_deadline is not None and time.monotonic()>=candidate_deadline:
            raise PlanningFailure('CONTACT_CANDIDATE_BUDGET','contact',
                'Initial candidate contact-search budget exhausted; try another initial pose')

    def search_contact_sequence(self,side,q,specifications,wrist_from_tip,ticket,*,terminal=None):
        params=getattr(self,'planning_parameters',PlanningParameters())
        if getattr(self,'_contact_search_deadline',None) is None:
            self._contact_search_deadline=time.monotonic()+params.contact_search_seconds
        self._contact_candidate_deadline=time.monotonic()+params.contact_candidate_seconds
        report={'method':'bounded_cross_stage_backtracking','backtracks':[],
                'stage_solutions':[],'accepted':False,'branch_limit':params.contact_branch_attempts}
        self.search_diagnostics.setdefault('contact_branch_search',[]).append(report)
        began=time.monotonic()
        recoverable={'CONTINUOUS_IK_FAILED','RETIMED_TCP_BOUNDS','JOINT_MARGIN','COLLISION','MOTION_PLAN_FAILED'}
        def descend(index,state):
            self.check_contact_search_budget()
            points,reference,label,curve,stage=specifications[index](state)
            paths=self.cartesian_paths(side,state,points,reference,wrist_from_tip,ticket,label,curve,stage,
                                       branching=True)
            last_error=None
            try:
                result=next(paths)
                while True:
                    report['stage_solutions'].append(stage)
                    try:
                        if index==len(specifications)-1:
                            return [result]+(terminal(result['end']) if terminal is not None else [])
                        return [result]+descend(index+1,result['end'])
                    except PlanningFailure as error:
                        if error.diagnostic['code'] not in recoverable:raise
                        last_error=error
                        if len(report['backtracks'])>=params.contact_branch_attempts:raise
                        report['backtracks'].append({'resume_stage':stage,'failure':error.diagnostic})
                        self.progress(ticket,f'Contact search: {error.diagnostic["stage"]} failed; revisiting {stage} branch')
                        result=paths.send(error)
            except StopIteration:
                if last_error:raise last_error
                raise PlanningFailure('CONTINUOUS_IK_FAILED',stage,'No remaining contact branch')
            finally:paths.close()
        try:
            result=descend(0,q)
            self.check_contact_search_budget()
            report['accepted']=True
            return result
        except PlanningFailure as error:
            report['failure']=error.diagnostic
            raise
        finally:
            report['elapsed_seconds']=time.monotonic()-began
            self._contact_candidate_deadline=None

    def contact_segments(self,side,q,pre,entry,center,nominal,wrist_from_tip,ticket,*,arc_nominal=None,terminal=None):
        """Validated contact portion; alone it is not an executable task plan."""
        if arc_nominal is None:arc_nominal=nominal
        params=getattr(self,'planning_parameters',PlanningParameters())
        segments=[]
        def line_curve(a,b):
            d = b-a
            def result(p):return float(np.linalg.norm(p-(a+np.clip(np.dot(p-a,d)/np.dot(d,d),0,1)*d)))
            result.batch=lambda points:np.linalg.norm(points-(a+np.clip((points-a)@d/np.dot(d,d),0,1)[:,None]*d),axis=1)
            return result
        insertion = np.linspace(pre[:3,3],entry,params.line_points)[1:]
        insert_spec=(insertion,arc_nominal if callable(arc_nominal) else nominal,'Insert',
                     line_curve(pre[:3,3],entry),'insert')
        radius = float(np.linalg.norm(entry[:2]-center[:2]))
        offset = entry-center
        direction=getattr(self,'sweep_direction','left')
        if direction not in ('left','right'):raise ValueError('Unknown D-ring sweep direction')
        end=120 if direction=='left' else -120
        initial_angle=math.atan2(offset[1],offset[0]);sign=1 if end>0 else -1
        def arc_errors(points):
            points=np.asarray(points)
            angles=np.arctan2(points[:,1]-center[1],points[:,0]-center[0])-initial_angle
            angles=(angles+math.pi)%(2*math.pi)-math.pi
            angles=initial_angle+sign*np.clip(sign*angles,0,math.radians(120))
            nearest=np.column_stack((center[0]+radius*np.cos(angles),center[1]+radius*np.sin(angles),
                                     np.full(len(points),entry[2])))
            return np.linalg.norm(points-nearest,axis=1)
        def circle_error(p):return float(arc_errors([p])[0])
        circle_error.batch=arc_errors
        points = [center+Rotation.from_euler('z',a,degrees=True).apply(offset)
                  for a in np.linspace(0,end,params.arc_segments+1)[1:]]
        rotate_spec=(points,arc_nominal,direction.title()+' 120',circle_error,'rotate')
        def withdraw_spec(state):
            actual=self.tip(side,state,wrist_from_tip)
            exit_start=actual[:3,3].copy();exit_end=exit_start+np.array([0.,0.,params.withdraw_height_mm/1000.])
            exit_nominal=arc_nominal if callable(arc_nominal) else Rotation.from_matrix(actual[:3,:3]).as_quat()
            return (np.linspace(exit_start,exit_end,params.line_points)[1:],exit_nominal,'Withdraw',
                    line_curve(exit_start,exit_end),'withdraw')
        specifications=(lambda state:insert_spec,lambda state:rotate_spec,withdraw_spec)
        if params.contact_branch_search:
            segments=self.search_contact_sequence(side,q,specifications,wrist_from_tip,ticket,terminal=terminal)
        else:
            for specification in specifications:
                state=segments[-1]['end'] if segments else q
                points,reference,label,curve,stage=specification(state)
                segments.append(self.cartesian(side,state,points,reference,wrist_from_tip,ticket,label,curve,stage))
            if terminal is not None:segments.extend(terminal(segments[-1]['end']))
        from task_stage_control import slow_segment
        for segment,stage in zip(segments[1:],('rotate','withdraw')):
            slow_segment(segment,getattr(params,stage+'_velocity_factor'))
        return segments
