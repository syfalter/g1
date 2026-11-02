"""Offline task console. Publishes simulated JointState only, never motor commands."""
import copy
import json
import math
import signal
from pathlib import Path
import threading
import time
import traceback
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
from PyQt5 import QtCore, QtWidgets
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, PoseArray
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from moveit_msgs.msg import RobotState, JointConstraint, Constraints, AllowedCollisionEntry, PlanningSceneComponents
from moveit_msgs.srv import GetPositionIK, GetStateValidity, GetMotionPlan, GetPlanningScene, ApplyPlanningScene

from workbench_task_geometry import parse_target, arc_point, insertion_orientation
from workbench_model import OfflineModel
from workbench_stance import STANCE_JOINTS, STANCE_LIMIT_M
from workbench_orientation import (
    orientation_policy, orientation_candidates, orientation_allowed, orientation_error,
)

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / "assets/workbench"
INDEX_OPEN=.15/1.1169
INDEX_FOLDED=1.1


def finger_pose(q,side,angle):
    result=dict(q)
    result[side+'_index_1_joint']=float(angle)
    result[side+'_index_2_joint']=float(angle*1.1169-.15)
    return result


def startup_joints(names, profile):
    posture=profile.get('hand_startup_posture','open')
    if posture not in ('open','folded'):
        raise ValueError('Unknown startup hand posture')
    q={name:float(profile['joints'].get(name,0)) for name in names}
    for side in ('left','right'):
        q=finger_pose(q,side,INDEX_OPEN if posture=='open' else INDEX_FOLDED)
    return q


class WorkbenchApp(Node):
    def __init__(self, scene_dir=None, plan_dir=None, warm_start_dir=None):
        super().__init__("g1_workbench_offline")
        self.scene_dir=Path(scene_dir or ASSETS/'task')
        self.plan_dir=Path(plan_dir or ASSETS/'task')
        self.warm_start_dir=Path(warm_start_dir or ASSETS/'task')
        self.xml = ET.parse(self.scene_dir/"robot.urdf").getroot()
        self.model=OfflineModel(self.scene_dir/"robot.urdf")
        initial_file=self.scene_dir/'initial_pose.json'
        initial_override=self.declare_parameter('initial_pose_file','').value
        if initial_override:
            initial_file=Path(initial_override)
        elif not initial_file.exists():
            initial_file=ASSETS/'task_start_pose.json'
        initial = json.loads(initial_file.read_text())
        self.names = [j.get("name") for j in self.xml.findall("joint") if j.get("type") not in ("fixed", "floating")]
        self.home = startup_joints(self.names,initial)
        self.current = dict(self.home)
        self.target = json.loads((self.scene_dir/"target.json").read_text())
        self.status = {"mode":"OFFLINE", "phase":"IDLE", "message":"Ready", "arm":"left", "progress":0.}
        self.lock = threading.RLock()
        self.busy = False
        self.playing = False
        self.cancel_event = threading.Event()
        self.plan = []
        self.plan_target = None
        self.plan_arm = None
        self.speed = 3.
        self.play_time = 0.
        self.last_tick = time.monotonic()
        self.ik = self.create_client(GetPositionIK,"/compute_ik")
        self.valid = self.create_client(GetStateValidity,"/check_state_validity")
        self.motion = self.create_client(GetMotionPlan,"/plan_kinematic_path")
        self.scene = self.create_client(GetPlanningScene,"/get_planning_scene")
        self.apply = self.create_client(ApplyPlanningScene,"/apply_planning_scene")
        self.pub = self.create_publisher(JointState,"/joint_states",10)
        self.markers = self.create_publisher(MarkerArray,"/workbench/markers",10)
        self.status_pub = self.create_publisher(String,"/workbench/status",10)
        self.create_subscription(String,"/workbench/request",self.request,10)
        self.create_subscription(PoseArray,"/workbench/vision_target",self.vision,10)
        self.create_timer(.02,self.tick)
        self.create_timer(.3,self.publish_markers)

    def message(self, phase, text):
        with self.lock:
            self.status.update(phase=phase,message=text)
        self.get_logger().info(f"{phase}: {text}")

    def call(self, client, request, timeout=12):
        if self.cancel_event.is_set():
            raise RuntimeError("Planning cancelled")
        if not client.wait_for_service(timeout_sec=3):
            raise RuntimeError(f"Service unavailable: {client.srv_name}")
        future = client.call_async(request)
        event = threading.Event()
        future.add_done_callback(lambda _:event.set())
        if not event.wait(timeout):
            raise RuntimeError(f"Service timeout: {client.srv_name}")
        return future.result()

    def state(self,q):
        state=RobotState()
        state.joint_state.name=self.names
        state.joint_state.position=[float(q[n]) for n in self.names]
        state.joint_state.header.stamp=self.get_clock().now().to_msg()
        return state

    def pose(self,p,quat):
        pose=Pose()
        pose.position.x,pose.position.y,pose.position.z=map(float,p)
        pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w=map(float,quat)
        return pose

    def solve(self,side,p,quat,q,angle=0.,avoid=True,continuous=False):
        request=GetPositionIK.Request()
        r=request.ik_request
        r.group_name=f"{side}_arm"
        r.ik_link_name=f"{side}_rubber_hand"
        seed=dict(q);seed["task_handle_angle"]=float(angle)
        r.robot_state=self.state(seed)
        r.pose_stamped.header.frame_id="world"
        hand_to_tip=np.linalg.inv(self.model.transform(side+'_rubber_hand',seed)) @ self.model.transform(side+'_index_tcp',seed)
        desired=np.eye(4);desired[:3,3]=p;desired[:3,:3]=Rotation.from_quat(quat).as_matrix()
        hand_goal=desired @ np.linalg.inv(hand_to_tip)
        r.pose_stamped.pose=self.pose(hand_goal[:3,3],Rotation.from_matrix(hand_goal[:3,:3]).as_quat())
        r.avoid_collisions=avoid
        if continuous:
            for name in self.names:
                if self.arm_joint(side,name):
                    joint=JointConstraint();joint.joint_name=name;joint.position=float(q[name])
                    joint.tolerance_above=.20;joint.tolerance_below=.20;joint.weight=1.
                    r.constraints.joint_constraints.append(joint)
        r.timeout.sec=0;r.timeout.nanosec=80000000
        result=self.call(self.ik,request)
        if result.error_code.val!=1:
            return None
        answer=dict(seed)
        answer.update(zip(result.solution.joint_state.name,result.solution.joint_state.position))
        return answer

    def validate(self,q):
        request=GetStateValidity.Request();request.robot_state=self.state(q)
        result=self.call(self.valid,request)
        if not result.valid:
            pairs=sorted({f"{c.contact_body_1}/{c.contact_body_2}" for c in result.contacts})
            raise RuntimeError("Collision or invalid state: "+", ".join(pairs[:5]))

    def solve_relaxed(self,side,p,nominal,q,angle=0.):
        policy=orientation_policy(self.target)
        previous=Rotation.from_matrix(self.model.transform(side+'_index_tcp',q)[:3,:3])
        names=[name for name in self.names if self.arm_joint(side,name)]
        best=None;best_score=float('inf')
        for index,quat in enumerate(orientation_candidates(nominal,previous.as_quat(),policy)):
            answer=self.solve(side,p,quat,q,angle,continuous=True)
            if answer is None:continue
            delta=np.array([answer[name]-q[name] for name in names])
            if not np.isfinite(delta).all() or np.max(np.abs(delta))>.20:continue
            actual=self.model.transform(side+'_index_tcp',answer)
            actual_rotation=Rotation.from_matrix(actual[:3,:3])
            if (np.linalg.norm(actual[:3,3]-p)>.002
                    or not orientation_allowed(nominal,actual_rotation.as_quat(),policy,epsilon=1e-4)
                    or math.degrees((previous.inv()*actual_rotation).magnitude())>policy['step_deg']+1e-4):
                continue
            limit_penalty=0.
            for name in names:
                limit=self.model.robot.find(f"joint[@name='{name}']/limit")
                if limit is not None and 'lower' in limit.attrib and 'upper' in limit.attrib:
                    margin=min(answer[name]-float(limit.get('lower')),float(limit.get('upper'))-answer[name])
                    if margin<0:break
                    limit_penalty+=max(0.,.15-margin)**2
            else:
                wrist=sum((answer[name]-q[name])**2 for name in names if '_wrist_' in name)
                change=(previous.inv()*actual_rotation).magnitude()
                score=float(delta@delta)+.5*wrist+2*limit_penalty+.2*change**2
                if score<best_score:best,best_score=answer,score
                # Keep a smooth, well-inside-limits solution without extra IK calls.
                if index==0 and np.max(np.abs(delta))<.04 and limit_penalty==0:
                    return answer
        return best

    def contact_policy(self,side,enabled):
        request=GetPlanningScene.Request();request.components.components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        matrix=self.call(self.scene,request).scene.allowed_collision_matrix
        links=[side+'_'+part for part in ('index_2','index_rubber_2','index_pad','index_tip')]
        for name in links+["task_handle"]:
            if name not in matrix.entry_names:
                matrix.entry_names.append(name)
                for row in matrix.entry_values:row.enabled.append(False)
                row=AllowedCollisionEntry();row.enabled=[False]*len(matrix.entry_names);matrix.entry_values.append(row)
        b=matrix.entry_names.index("task_handle")
        for name in links:
            a=matrix.entry_names.index(name)
            matrix.entry_values[a].enabled[b]=enabled;matrix.entry_values[b].enabled[a]=enabled
        request=ApplyPlanningScene.Request();request.scene.is_diff=True;request.scene.allowed_collision_matrix=matrix
        if not self.call(self.apply,request).success:raise RuntimeError("Failed to update contact policy")

    def append(self,frames,q,phase):
        if frames:
            delta=max(abs(q[n]-frames[-1]["q"][n]) for n in self.names if n!="task_handle_angle")
            dt=max(.04,delta/.22)
            t=frames[-1]["t"]+dt
        else:t=0.
        frames.append({"t":t,"q":dict(q),"phase":phase})

    @staticmethod
    def arm_joint(side,name):
        return (name.startswith(side+'_') and name.endswith('_joint') and '_index_' not in name) or name=='waist_yaw_joint'

    def change_finger(self,side,q,angle,phase,frames):
        start=q[side+'_index_1_joint']
        for value in np.linspace(start,angle,35)[1:]:
            q=finger_pose(q,side,value);self.append(frames,q,phase)
        return q

    def joint_path(self,side,start,goal,phase,frames):
        if any(abs(start[f'{side}_index_{i}_joint']-goal[f'{side}_index_{i}_joint'])>1e-8
               for i in (1,2)):
            raise RuntimeError('Arm planning cannot change index joints; use an explicit finger phase')
        request=GetMotionPlan.Request();r=request.motion_plan_request
        r.group_name=f"{side}_arm";r.start_state=self.state(start)
        r.allowed_planning_time=5.;r.num_planning_attempts=3
        r.max_velocity_scaling_factor=.5;r.max_acceleration_scaling_factor=.5
        constraint=Constraints()
        for name in self.names:
            if self.arm_joint(side,name):
                joint=JointConstraint();joint.joint_name=name;joint.position=float(goal[name]);joint.tolerance_above=.002;joint.tolerance_below=.002;joint.weight=1.
                constraint.joint_constraints.append(joint)
        r.goal_constraints=[constraint]
        result=self.call(self.motion,request,30).motion_plan_response
        if result.error_code.val!=1:raise RuntimeError(f"{phase}: MoveIt planning failed ({result.error_code.val})")
        path=result.trajectory.joint_trajectory
        q=dict(start)
        for point in path.points:
            q.update(zip(path.joint_names,point.positions))
            self.append(frames,q,phase)
        # Connect the planner's tolerance-sized endpoint error explicitly.
        self.append(frames,goal,phase)
        q=dict(goal)
        return q

    def enter_table(self,side,start,goal,frames):
        q=dict(start)
        if abs(q[side+'_index_1_joint']-INDEX_FOLDED)>1e-8:
            q=self.change_finger(side,q,INDEX_FOLDED,'FOLD_START_INDEX',frames)
            self.validate(q)
        return self.joint_path(side,q,goal,'ENTER_TABLE',frames)

    def return_home(self,side,start,home,frames):
        goal=finger_pose(home,side,start[side+'_index_1_joint'])
        q=self.joint_path(side,start,goal,'RETURN_HOME',frames)
        if abs(q[side+'_index_1_joint']-home[side+'_index_1_joint'])>1e-8:
            q=self.change_finger(side,q,home[side+'_index_1_joint'],'RESTORE_START_INDEX',frames)
        return q

    def line(self,side,start,p0,p1,quat,phase,frames):
        q=dict(start)
        steps=max(2,int(np.linalg.norm(p1-p0)/.004)+1)
        for f in np.linspace(0,1,steps)[1:]:
            next_q=self.solve_relaxed(side,p0*(1-f)+p1*f,quat,q)
            if next_q is None:raise RuntimeError(f"{phase}: no collision-free IK at {f:.0%}")
            if max(abs(next_q[n]-q[n]) for n in self.names if n.startswith(side+"_") or n=="waist_yaw_joint")>.25:
                raise RuntimeError(f"{phase}: IK jump rejected")
            q=next_q;self.append(frames,q,phase)
        return q

    def plan_worker(self,side,scope):
        frames=[]
        self.last_error_type=None
        self.last_report=None
        self.candidate_failures=[]
        self.candidate_attempts=0
        try:
            self.status["arm"]=side
            start=dict(self.current)
            self.validate(start)
            self.contact_policy("left",False);self.contact_policy("right",False)
            c,entry,n,r=parse_target(self.target)
            pre=entry+n*float(self.target.get("preinsert_height_m",.1))
            self.message("PLANNING","Searching fingertip approach orientation")
            orientations=[(tilt,yaw,roll) for tilt in (20,35,50,0,65)
                        for yaw in (0,30,-30,60,-60,90,-90,180)
                        for roll in (0,90,180,270)]
            seed=finger_pose(start,side,INDEX_OPEN)
            candidates=[]
            previous=self.warm_start_dir/f'last_{side}_full_plan.json'
            if previous.exists():
                try:
                    saved=json.loads(previous.read_text())
                    entry_frame=[f for f in saved['frames'] if f['phase']=='INSERT'][-1]
                    warm=dict(seed)
                    warm.update({name:value for name,value in entry_frame['q'].items() if self.arm_joint(side,name)})
                    cached_quat=Rotation.from_matrix(self.model.transform(side+'_index_tcp',warm)[:3,:3]).as_quat()
                    candidates.append((cached_quat,warm,'previous insertion seed'))
                except (ValueError,KeyError,IndexError,OSError):pass
            for tilt,yaw,roll in orientations:
                quat=(Rotation.from_quat(insertion_orientation(self.target,math.radians(yaw),math.radians(tilt))) * Rotation.from_euler('z',roll,degrees=True)).as_quat()
                candidates.append((quat,seed,f'{tilt}/{yaw}/{roll}'))
            failures=[]
            for k,(quat,ik_seed,label) in enumerate(candidates):
                self.active_orientation_reference=np.asarray(quat)
                self.candidate_attempts=k+1
                if self.cancel_event.is_set():raise RuntimeError("Planning cancelled")
                approach=Rotation.from_quat(quat).apply([0,0,1])
                pre=entry-approach*float(self.target.get('preinsert_height_m',.1))/(-np.dot(approach,n))
                self.message("PLANNING",f"{side}: orientation {k+1}/{len(candidates)} ({label})")
                try:
                    trial=[];self.append(trial,start,"START")
                    if scope=="entry":
                        self.contact_policy(side,False)
                        qpre=self.solve(side,pre,quat,ik_seed)
                        if qpre is None:continue
                        qclosed=finger_pose(qpre,side,INDEX_FOLDED)
                        self.enter_table(side,start,qclosed,trial)
                        outward=list(trial)
                        for frame in reversed(outward[:-1]):self.append(trial,frame['q'],"RETURN_HOME")
                    else:
                        self.contact_policy(side,True)
                        qentry=self.solve(side,entry,quat,ik_seed)
                        if qentry is None:continue
                        # Build the constrained task first, seeded at actual insertion.
                        ascent=[];self.append(ascent,qentry,"INSERT")
                        qpre=self.line(side,qentry,entry,pre,quat,"INSERT",ascent)
                        contact=[]
                        for frame in reversed(ascent):self.append(contact,frame['q'],"INSERT")
                        q=qentry
                        angle=math.radians(float(self.target.get("angle_deg",120)))
                        for end,phase in ((angle,"ROTATE_LEFT"),(-angle,"ROTATE_RIGHT")):
                            a0=q["task_handle_angle"]
                            leg=[dict(q)]
                            for theta in np.linspace(a0,end,max(2,int(abs(end-a0)/math.radians(3))+1))[1:]:
                                next_q=self.solve_relaxed(side,arc_point(self.target,theta),quat,q,theta)
                                if next_q is None:raise RuntimeError(f"{phase}: IK/collision failure at {math.degrees(theta):.1f} deg")
                                if max(abs(next_q[name]-q[name]) for name in self.names if name.startswith(side+"_") or name=="waist_yaw_joint")>.25:
                                    raise RuntimeError(f"{phase}: joint jump at {math.degrees(theta):.1f} deg")
                                q=next_q;leg.append(dict(q));self.append(contact,q,phase)
                            # Return on the same validated IK branch, avoiding redundant drift.
                            for state in reversed(leg[:-1]):
                                q=dict(state);self.append(contact,q,"RECENTER")
                        qout=self.line(side,q,entry,pre,quat,"WITHDRAW",contact)
                        self.contact_policy(side,False)
                        self.validate(qpre);self.validate(qout)
                        qclosed=finger_pose(qpre,side,INDEX_FOLDED)
                        self.validate(qclosed)
                        self.enter_table(side,start,qclosed,trial)
                        self.change_finger(side,qclosed,INDEX_OPEN,"EXTEND_INDEX",trial)
                        for frame in contact[1:]:self.append(trial,frame['q'],frame['phase'])
                        qfold=self.change_finger(side,qout,INDEX_FOLDED,"FOLD_INDEX",trial)
                        self.return_home(side,qfold,start,trial)
                    metrics=self.validate_frames(side,trial)
                    frames=trial;break
                except RuntimeError as error:
                    failures.append(str(error));self.get_logger().warn(str(error))
                    self.candidate_failures.append(str(error))
            if not frames:raise RuntimeError("No complete path. "+("; ".join(failures[-3:]) or "No reachable, collision-free insertion pose"))
            with self.lock:
                self.plan=frames;self.plan_target=json.dumps(self.target,sort_keys=True);self.play_time=0.;self.plan_arm=side
                self.status['insertion_tilt_deg']=math.degrees(math.acos(float(np.clip(-np.dot(approach,n),-1,1))))
            report={"offline_only":True,"arm":side,"scope":scope,"target":self.target,"duration":frames[-1]["t"],"validation":metrics,
                    "orientation_tolerance":orientation_policy(self.target),
                    "orientation_reference_xyzw":self.active_orientation_reference.tolist(),
                    "insertion_tilt_deg":math.degrees(math.acos(float(np.clip(-np.dot(approach,n),-1,1)))),
                    "preinsert_point":pre.tolist(),"frames":frames}
            self.last_report=report
            (self.plan_dir/f"last_{side}_{scope}_plan.json").write_text(json.dumps(report,indent=2)+"\n")
            self.message("PLAN_READY",f"{side} {scope}: {len(frames)} samples, {frames[-1]['t']:.1f} s; no hardware execution")
        except Exception as error:
            self.last_error_type=type(error).__name__
            self.plan=[];self.message("PLAN_FAILED",str(error));traceback.print_exc()
        finally:
            self.cancel_event.clear()
            try:self.contact_policy("left",False);self.contact_policy("right",False)
            except Exception:pass
            self.busy=False

    def validate_frames(self,side,frames):
        self.message("VALIDATING",f"Checking {len(frames)} segments")
        previous_contact=None
        checked=0;max_arc_error=0.;max_line_error=0.;max_tilt=0.;max_roll=0.
        policy=orientation_policy(self.target)
        reference=getattr(self,'active_orientation_reference',None)
        for i in range(1,len(frames)):
            a,b=frames[i-1],frames[i]
            contact=b["phase"] in ("INSERT","ROTATE_LEFT","ROTATE_RIGHT","RECENTER","WITHDRAW")
            if contact!=previous_contact:self.contact_policy(side,contact)
            previous_contact=contact
            if contact:
                tip_a=self.model.transform(side+'_index_tcp',a['q'])
                tip_b=self.model.transform(side+'_index_tcp',b['q'])
                change=math.degrees((Rotation.from_matrix(tip_a[:3,:3]).inv()*Rotation.from_matrix(tip_b[:3,:3])).magnitude())
                if change>policy['step_deg']+1e-4:
                    raise RuntimeError('Contact orientation step exceeds configured bound')
            delta=max(abs(b['q'][name]-a['q'][name]) for name in self.names)
            steps=max(4,int(math.ceil(delta/.01)))
            for f in np.linspace(0,1,steps+1)[1:]:
                q={name:a["q"][name]*(1-f)+b["q"][name]*f for name in self.names}
                self.validate(q);checked+=1
                if contact and reference is not None:
                    tip=self.model.transform(side+'_index_tcp',q)
                    actual=Rotation.from_matrix(tip[:3,:3]).as_quat()
                    tilt,roll=orientation_error(reference,actual)
                    max_tilt=max(max_tilt,tilt);max_roll=max(max_roll,roll)
                    if not orientation_allowed(reference,actual,policy,epsilon=1e-4):
                        raise RuntimeError('Interpolated contact orientation exceeds tolerance')
                    if b['phase'] in ('INSERT','WITHDRAW'):
                        error=float(np.linalg.norm(tip[:3,3]-(tip_a[:3,3]*(1-f)+tip_b[:3,3]*f)))
                        max_line_error=max(max_line_error,error)
                        if error>.002:raise RuntimeError('Interpolated insertion/withdrawal line error exceeds 2 mm')
                if b['phase'] in ('ROTATE_LEFT','ROTATE_RIGHT','RECENTER'):
                    error=float(np.linalg.norm(self.model.transform(side+'_index_tcp',q)[:3,3]-arc_point(self.target,q['task_handle_angle'])))
                    max_arc_error=max(max_arc_error,error)
                    if error>.002:raise RuntimeError(f"Interpolated fingertip arc error {error*1000:.2f} mm")
        return dict(checked_states=checked,max_joint_step_rad=.01,max_arc_error_m=max_arc_error,
                    max_line_error_m=max_line_error,max_orientation_tilt_deg=max_tilt,
                    max_orientation_roll_deg=max_roll,orientation_tolerance=policy,
                    waist_range_deg=[math.degrees(min(f['q']['waist_yaw_joint'] for f in frames)),
                                     math.degrees(max(f['q']['waist_yaw_joint'] for f in frames))],
                    playback='Geometric joint interpolation only; not hardware time parameterization')

    def begin_plan(self,side,scope):
        with self.lock:
            if self.busy or self.playing:return
            if side not in ("left","right") or scope not in ("entry","full"):raise ValueError("Invalid task selection")
            if max(abs(self.current[n]-self.home[n]) for n in self.names)>.002:
                raise ValueError("Reset the offline scene before planning a new task")
            self.busy=True;self.plan=[];self.cancel_event.clear()
            self.status['progress']=0.
        threading.Thread(target=self.plan_worker,args=(side,scope),daemon=True).start()

    def command(self,data):
        with self.lock:self._command(data)

    def _command(self,data):
        action=data.get("action")
        if action=="plan":self.begin_plan(data.get("arm","left"),data.get("scope","entry"))
        elif action=="play":
            if self.busy or not self.plan:raise ValueError("No completed plan")
            if self.plan_target!=json.dumps(self.target,sort_keys=True):raise ValueError("Target changed; replan")
            if self.play_time>=self.plan[-1]["t"]:self.play_time=0.
            self.playing=True;self.message("PLAYING","Offline playback")
        elif action=="pause":self.playing=False;self.message("PAUSED","Offline state held")
        elif action=="seek":
            fraction=float(data.get('fraction',0))
            if self.busy or not self.plan:raise ValueError("No completed offline plan")
            if not math.isfinite(fraction) or not 0<=fraction<=1:raise ValueError("Invalid timeline position")
            self.playing=False;self.play_time=fraction*self.plan[-1]['t'];self.sample_playback()
        elif action=="cancel":
            self.cancel_event.set();self.playing=False;self.plan=[]
            self.message("CANCELLED","Offline task cancelled; reset before a new task")
        elif action=="select_arm":
            if self.busy or self.playing:raise ValueError("Stop before changing arm")
            if data.get("arm") not in ("left","right"):raise ValueError("Invalid arm")
            self.plan=[];self.plan_arm=None;self.status.update(arm=data['arm'],progress=0.)
            self.message("IDLE","Arm selected; plan required")
        elif action=="reset":
            if self.busy:raise ValueError("Cancel planning before reset")
            self.playing=False;self.current=dict(self.home);self.play_time=0.;self.plan=[];self.status['progress']=0.;self.message("IDLE","Offline state reset")
        elif action=="stance":
            if self.busy or self.playing:
                raise ValueError("Stop planning/playback before changing stance")
            if not all(name in self.names for name in STANCE_JOINTS):
                raise ValueError("Restart the offline application to enable XY stance")
            if max(abs(self.current[n]-self.home[n]) for n in self.names)>.002:
                raise ValueError("Reset to the initial pose before changing stance")
            values=[float(data[axis]) for axis in ('x','y')]
            if any(not math.isfinite(v) or abs(v)>STANCE_LIMIT_M for v in values):
                raise ValueError("Stance offsets must be finite and within +/-0.30 m")
            if all(abs(self.home[n]-v)<1e-9 for n,v in zip(STANCE_JOINTS,values)):
                return
            self.home.update(zip(STANCE_JOINTS,values))
            self.current=dict(self.home)
            self.plan=[];self.plan_target=None;self.plan_arm=None;self.play_time=0.
            self.status['progress']=0.
            self.message("IDLE","Offline stance updated; plan required")
        elif action=="target":
            if self.busy or self.playing:raise ValueError("Stop before replacing target")
            parse_target(data["target"])
            default=json.loads((ASSETS/"task/target.json").read_text())
            q=Rotation.from_quat(data['target']['axis_quaternion_xyzw'])
            if np.linalg.norm(np.array(data['target']['axis_center'])-default['axis_center'])>.001 or q.magnitude()>.002:
                raise ValueError("Target axis differs from fixed CAD scene; register scene before use")
            self.target=copy.deepcopy(data["target"]);self.plan=[];self.message("IDLE","Target updated")
        else:raise ValueError("Unknown request")

    def request(self,msg):
        try:self.command(json.loads(msg.data))
        except Exception as e:self.message("REJECTED",str(e))

    def vision(self,msg):
        try:
            if msg.header.frame_id!="world" or len(msg.poses)!=2:raise ValueError("v1 vision input requires world frame and [axis,entry] poses")
            age=(self.get_clock().now().nanoseconds-(msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec))*1e-9
            if not 0<=age<=2:raise ValueError("Stale vision target")
            a,p=msg.poses
            target=dict(self.target)
            target.update(axis_center=[a.position.x,a.position.y,a.position.z],entry_center=[p.position.x,p.position.y,p.position.z],axis_quaternion_xyzw=[a.orientation.x,a.orientation.y,a.orientation.z,a.orientation.w],source="vision")
            self.command({"action":"target","target":target})
        except Exception as e:self.message("TARGET_REJECTED",str(e))

    def tick(self):
        now=time.monotonic();dt=min(now-self.last_tick,.1);self.last_tick=now
        with self.lock:
            if self.playing and self.plan:
                self.play_time=min(self.play_time+dt*self.speed,self.plan[-1]["t"])
                self.sample_playback()
                if self.play_time>=self.plan[-1]['t']:self.playing=False;self.message("COMPLETE","Offline sequence complete")
            msg=JointState();msg.header.stamp=self.get_clock().now().to_msg();msg.name=self.names;msg.position=[float(self.current[n]) for n in self.names]
            self.pub.publish(msg)
            self.status_pub.publish(String(data=json.dumps(dict(self.status,busy=self.busy,plan_ready=bool(self.plan),playing=self.playing,
                stance_offset_m={axis:self.home.get(name,0.) for axis,name in zip(('x','y'),STANCE_JOINTS)}))))

    def sample_playback(self):
        times=[p['t'] for p in self.plan];i=min(int(np.searchsorted(times,self.play_time)),len(times)-1)
        a,b=self.plan[max(0,i-1)],self.plan[i]
        f=(self.play_time-a['t'])/max(1e-9,b['t']-a['t'])
        self.current={n:a['q'][n]*(1-f)+b['q'][n]*f for n in self.names}
        self.status.update(phase=b['phase'],progress=self.play_time/max(times[-1],1e-9))

    def publish_markers(self):
        array=MarkerArray();c,p,n,r=parse_target(self.target)
        for i,(pos,color) in enumerate(((c,(1.,.35,.1)),(p,(.1,.9,.4)))):
            marker=Marker();marker.header.frame_id="world";marker.ns="targets";marker.id=i;marker.type=Marker.SPHERE;marker.action=Marker.ADD
            marker.pose.position.x,marker.pose.position.y,marker.pose.position.z=map(float,pos)
            marker.pose.orientation.w=1.;marker.scale.x=marker.scale.y=marker.scale.z=.012
            marker.color.r,marker.color.g,marker.color.b=color;marker.color.a=1.;array.markers.append(marker)
        marker=Marker();marker.header.frame_id="world";marker.ns="arc";marker.id=2;marker.type=Marker.LINE_STRIP;marker.action=Marker.ADD;marker.pose.orientation.w=1.
        marker.scale.x=.003;marker.color.r=.2;marker.color.g=.65;marker.color.b=1.;marker.color.a=1.
        angle=math.radians(float(self.target.get('angle_deg',120)))
        for theta in np.linspace(-angle,angle,81):
            xyz=arc_point(self.target,theta);marker.points.append(Point(x=float(xyz[0]),y=float(xyz[1]),z=float(xyz[2])))
        array.markers.append(marker);self.markers.publish(array)


class Console(QtWidgets.QWidget):
    def __init__(self,node):
        super().__init__();self.node=node
        self.setWindowTitle("G1 Workbench | OFFLINE")
        self.resize(460,460)
        layout=QtWidgets.QVBoxLayout(self)
        title=QtWidgets.QLabel("操作台任务 · 离线预演");title.setStyleSheet("font-size:20px;font-weight:600");layout.addWidget(title)
        self.arm=QtWidgets.QComboBox();self.arm.addItem("左手 / Left","left");self.arm.addItem("右手 / Right","right")
        self.arm.currentIndexChanged.connect(lambda _:self.run("select_arm",arm=self.arm.currentData()))
        layout.addWidget(self.arm)
        self.stance_sliders={};self.stance_spins={}
        for axis,label,tooltip in (('x','初始 X 偏移','正值靠近台面，负值远离台面'),
                                   ('y','初始 Y 偏移','正值向机器人左侧，负值向右侧')):
            row=QtWidgets.QHBoxLayout()
            row.addWidget(QtWidgets.QLabel(label))
            slider=QtWidgets.QSlider(QtCore.Qt.Horizontal)
            slider.setObjectName('stance_'+axis+'_slider')
            slider.setRange(-round(STANCE_LIMIT_M*1000),round(STANCE_LIMIT_M*1000))
            slider.setSingleStep(1);slider.setPageStep(10);slider.setMinimumWidth(120)
            slider.setToolTip(tooltip)
            spin=QtWidgets.QDoubleSpinBox();spin.setObjectName('stance_'+axis+'_cm')
            spin.setDecimals(1);spin.setRange(-STANCE_LIMIT_M*100,STANCE_LIMIT_M*100)
            spin.setSingleStep(.1);spin.setSuffix(' cm');spin.setFixedWidth(105)
            spin.setKeyboardTracking(False);spin.setToolTip(tooltip)
            slider.valueChanged.connect(lambda value,s=spin:s.setValue(value/10.))
            self.stance_sliders[axis]=slider;self.stance_spins[axis]=spin
            spin.valueChanged.connect(lambda _,a=axis:self.change_stance(a))
            row.addWidget(slider,1);row.addWidget(spin);layout.addLayout(row)
        self.entry=QtWidgets.QPushButton("规划进台与收臂");self.full=QtWidgets.QPushButton("规划插环及双向 120°")
        self.entry.clicked.connect(lambda:self.run("plan",scope="entry",arm=self.arm.currentData()))
        self.full.clicked.connect(lambda:self.run("plan",scope="full",arm=self.arm.currentData()))
        layout.addWidget(self.entry);layout.addWidget(self.full)
        row=QtWidgets.QHBoxLayout();self.play=QtWidgets.QPushButton("播放");pause=QtWidgets.QPushButton("暂停");reset=QtWidgets.QPushButton("重置")
        self.play.clicked.connect(lambda:self.run("play"));pause.clicked.connect(lambda:self.run("pause"));reset.clicked.connect(lambda:self.run("reset"))
        for w in (self.play,pause,reset):row.addWidget(w)
        layout.addLayout(row)
        cancel=QtWidgets.QPushButton("取消规划");cancel.clicked.connect(lambda:self.run("cancel"));layout.addWidget(cancel)
        row=QtWidgets.QHBoxLayout();row.addWidget(QtWidgets.QLabel("预演倍速"));self.speed=QtWidgets.QDoubleSpinBox();self.speed.setRange(.2,10);self.speed.setValue(3);self.speed.valueChanged.connect(lambda v:setattr(node,"speed",v));row.addWidget(self.speed);layout.addLayout(row)
        self.progress=QtWidgets.QProgressBar();layout.addWidget(self.progress)
        self.timeline=QtWidgets.QSlider(QtCore.Qt.Horizontal);self.timeline.setRange(0,1000)
        self.timeline.setToolTip('离线时间轴')
        self.timeline.sliderReleased.connect(lambda:self.run('seek',fraction=self.timeline.value()/1000.))
        layout.addWidget(self.timeline)
        self.phase=QtWidgets.QLabel("IDLE");self.phase.setStyleSheet("font-size:17px;font-weight:600");layout.addWidget(self.phase)
        self.detail=QtWidgets.QLabel();self.detail.setWordWrap(True);layout.addWidget(self.detail)
        self.message=QtWidgets.QLabel();self.message.setWordWrap(True);self.message.setMinimumHeight(95);layout.addWidget(self.message)
        layout.addStretch()
        self.timer=QtCore.QTimer();self.timer.timeout.connect(self.refresh);self.timer.start(150)

    def run(self,action,**kw):
        try:self.node.command(dict(action=action,**kw))
        except Exception as e:self.node.message("REJECTED",str(e))

    def change_stance(self,axis):
        self.run('stance',**{a:s.value()/100. for a,s in self.stance_spins.items()})
        self.refresh()

    def refresh(self):
        n=self.node;self.phase.setText(n.status['phase']);self.message.setText(n.status['message']);self.progress.setValue(int(n.status['progress']*100))
        self.arm.blockSignals(True);self.arm.setCurrentIndex(self.arm.findData(n.status['arm']));self.arm.blockSignals(False)
        at_home=max(abs(n.current[j]-n.home[j]) for j in n.names)<.002
        stance_enabled=not n.busy and not n.playing and at_home and all(j in n.names for j in STANCE_JOINTS)
        for axis,name in zip(('x','y'),STANCE_JOINTS):
            slider=self.stance_sliders[axis];spin=self.stance_spins[axis]
            slider.setEnabled(stance_enabled);spin.setEnabled(stance_enabled)
            value=n.home.get(name,0.)
            slider.blockSignals(True);slider.setValue(round(value*1000));slider.blockSignals(False)
            spin.blockSignals(True);spin.setValue(value*100);spin.blockSignals(False)
        self.entry.setEnabled(not n.busy and not n.playing and at_home);self.full.setEnabled(not n.busy and not n.playing and at_home);self.arm.setEnabled(not n.busy and not n.playing);self.play.setEnabled(bool(n.plan) and not n.busy and not n.playing)
        self.detail.setText(f"腰部转向 {math.degrees(n.current['waist_yaw_joint']):.1f}° / ±35°\n拉环角度 {math.degrees(n.current['task_handle_angle']):.1f}°")
        self.timeline.setEnabled(bool(n.plan) and not n.busy)
        if not self.timeline.isSliderDown():self.timeline.setValue(int(n.status['progress']*1000))


def main():
    rclpy.init();node=WorkbenchApp()
    thread=threading.Thread(target=rclpy.spin,args=(node,),daemon=True);thread.start()
    app=QtWidgets.QApplication([]);window=Console(node);window.show()
    signal.signal(signal.SIGINT,lambda *_:app.quit())
    signal.signal(signal.SIGTERM,lambda *_:app.quit())
    app.exec_();node.cancel_event.set()
    if rclpy.ok():rclpy.shutdown()
    thread.join(timeout=2)


if __name__=="__main__":main()
