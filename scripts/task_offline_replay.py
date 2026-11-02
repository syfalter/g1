"""Archived RGB-D replay. No gateway, hardware bridge, or action client."""
import argparse
import fcntl
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState, PointCloud2
from visualization_msgs.msg import MarkerArray
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from rosidl_runtime_py.convert import message_to_ordereddict

from task_future import wait_for_result
from task_motion_planner import TaskPlanner, robot_state
from task_preview_markers import target_markers
from task_session import Ticket, ARM_NAMES
from workbench_model import OfflineModel
from vision_task_preview import load_cloud
from task_hand_control import model_joints, POSTURES
from task_kinematics import capture_transform
from task_runtime_provenance import runtime_hashes, runtime_matches, SCOPE as RUNTIME_SCOPE

ROOT=Path(__file__).resolve().parents[1]
SESSION=ROOT/'data/task_sessions/20260909_154321_451801'


def require_offline_environment(domain):
    profile=ROOT/'scripts/task_test_fastdds.xml'
    if (domain not in range(93,100) or os.environ.get('ROS_DOMAIN_ID')!=str(domain)
            or os.environ.get('ROS_LOCALHOST_ONLY')!='1'
            or os.environ.get('RMW_IMPLEMENTATION')!='rmw_fastrtps_cpp'
            or any(Path(os.environ.get(key,'')).resolve()!=profile for key in
                   ('FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE'))
            or any(key in os.environ for key in ('ROS_DISCOVERY_SERVER','ROS_SUPER_CLIENT',
                'FASTDDS_ENVIRONMENT_FILE','SKIP_DEFAULT_XML','ROS_STATIC_PEERS',
                'ROS_AUTOMATIC_DISCOVERY_RANGE','CYCLONEDDS_URI'))):
        raise RuntimeError('Use run_task_offline_replay.sh: isolated loopback domain 93..99 is required')
    lock=Path(f'/tmp/g1-task-offline-{domain}-{os.getuid()}.lock')
    try:
        held=os.fstat(9);expected=lock.stat()
        if (held.st_dev,held.st_ino)!=(expected.st_dev,expected.st_ino):
            raise ValueError('Wrong domain lock')
        fcntl.flock(9,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except (OSError,ValueError) as error:
        raise RuntimeError('Wrapper domain flock is required') from error


def read_archive(path):
    def unique(pairs):
        result={}
        for key,value in pairs:
            if key in result:raise ValueError('Duplicate archive field: '+key)
            result[key]=value
        return result
    return json.loads(path.read_text(),object_pairs_hook=unique)


def checked_joints(value,articulated):
    hands={n for side in ('left','right') for n in model_joints(side,POSTURES['open'])}
    legs={f'{side}_{part}_joint' for side in ('left','right') for part in
          ('hip_pitch','hip_roll','hip_yaw','knee','ankle_pitch','ankle_roll')}
    required=set(ARM_NAMES)|{'waist_yaw_joint'}
    allowed=required|hands|legs
    if articulated:required|=hands
    if not isinstance(value,dict) or not required<=value.keys() or not value.keys()<=allowed:
        raise ValueError('Invalid archived joints: missing required arm/waist/hand or unknown joint')
    if any(type(v) not in (int,float) or not np.isfinite(v) for v in value.values()):
        raise ValueError('Archived joints must be finite numeric model angles (not strings/bools)')
    return dict(value)


def load_start_state(path,geometry,articulated,waist_assist):
    saved=read_archive(path)
    fields={'joints','pelvis_from_torso_row_major','captured_at','source'}
    if not isinstance(saved,dict) or set(saved)!=fields:
        raise ValueError('Expected capture_state fields: '+', '.join(sorted(fields)))
    if not isinstance(saved['source'],str) or not saved['source'].strip():
        raise ValueError('Capture state source is required')
    if not isinstance(saved['captured_at'],str):raise ValueError('Invalid captured_at')
    stamp=datetime.fromisoformat(saved['captured_at'])
    if stamp.tzinfo is None:raise ValueError('captured_at requires a timezone')
    q=checked_joints(saved['joints'],articulated)
    if not articulated:raise ValueError('--start-state requires --articulated-hands; no assumed hand posture')
    if not waist_assist and abs(q['waist_yaw_joint'])>1e-8:
        raise ValueError('Nonzero archived waist requires --waist-assist')
    frame=geometry.get('capture_frame',{})
    if geometry.get('frame_id')!='torso_link' or frame.get('frame')!='pelvis':
        raise ValueError('Expected torso_link geometry with frozen pelvis capture frame')
    matrices=[]
    for value in (saved['pelvis_from_torso_row_major'],frame.get('pelvis_from_torso_row_major')):
        raw=np.asarray(value)
        if raw.shape!=(4,4) or raw.dtype.kind not in 'fi' or not np.isfinite(raw).all():
            raise ValueError('Capture transform must be a finite numeric 4x4 matrix')
        matrix=capture_transform({'capture_frame':{'pelvis_from_torso_row_major':value}},None,q)
        matrices.append(matrix)
    if not np.allclose(*matrices,atol=1e-8,rtol=0):
        raise ValueError('State and geometry capture transforms differ')
    yaw=q['waist_yaw_joint'];c,s=np.cos(yaw),np.sin(yaw)
    if not np.allclose(matrices[0][:3,:3],[[c,-s,0],[s,c,0],[0,0,1]],atol=1e-6,rtol=0):
        raise ValueError('Capture transform rotation disagrees with archived waist yaw')
    return q


def add_replay_source_arguments(parser):
    parser.add_argument('--ros-domain-id',type=int,choices=range(93,100),default=93)
    source=parser.add_mutually_exclusive_group()
    source.add_argument('--start-plan',type=Path,help='Reconstruct a completed plan endpoint')
    source.add_argument('--start-state',type=Path,help='Strict capture_state.json model joint archive')


def legacy_geometry_offsets(task):
    """Infer metadata only when the entire frozen arc proves the legacy convention."""
    from scipy.spatial.transform import Rotation
    vectors=[]
    for name in ('axis_center_m','insertion_reference_m','assumed_normal'):
        value=np.asarray(task.get(name))
        if value.shape!=(3,) or value.dtype.kind not in 'fi' or not np.isfinite(value).all():
            raise ValueError('Legacy geometry lacks a numeric '+name)
        vectors.append(value)
    center,entry,normal=vectors
    if not np.allclose(normal,[0.,0.,1.],atol=1e-12,rtol=0):
        raise ValueError('Legacy geometry needs an explicitly horizontal arc')
    height=float(entry[2]-center[2]);pivot=center+normal*height
    if 'arc_center_m' in task and not np.allclose(task['arc_center_m'],pivot,atol=1e-9,rtol=0):
        raise ValueError('Legacy arc center is not the physical center')
    points=np.asarray(task.get('arc_minus120_to_plus120_m'))
    if (points.ndim!=2 or points.shape[1]!=3 or len(points)<3 or points.dtype.kind not in 'fi'
            or not np.isfinite(points).all()):
        raise ValueError('Legacy geometry lacks the frozen full arc')
    expected=pivot+Rotation.from_euler('z',np.linspace(-120.,120.,len(points)),degrees=True).apply(entry-pivot)
    if not np.allclose(points,expected,atol=1e-9,rtol=0):
        raise ValueError('Frozen arc does not prove legacy geometry offsets')
    if 'height_offset_m' in task and not np.isclose(task['height_offset_m'],height,atol=1e-9,rtol=0):
        raise ValueError('Legacy height disagrees with frozen arc')
    return dict(height_offset_m=height,arc_center_toward_robot_m=0.)


def await_service(client,check,until):
    while True:
        check()
        remaining=until-time.monotonic()
        if remaining<=0:raise RuntimeError('Unavailable '+client.srv_name)
        if client.wait_for_service(timeout_sec=min(.2,remaining)):
            check();return


def archived_geometry_record(geometry,parameters,*,require_match=False):
    archived={}
    current={}
    derived={}
    task=geometry['task_preview']
    if any(name not in task for name in ('height_offset_m','arc_center_toward_robot_m')):
        proved=legacy_geometry_offsets(task)
        derived={name:value for name,value in proved.items() if name not in task}
    for name in ('radial_offset','height_offset','arc_center_toward_robot'):
        value=task.get(name+'_m',derived.get(name+'_m'))
        if type(value) not in (int,float) or not np.isfinite(value):
            raise ValueError('Missing/nonfinite archived geometry offset: '+name)
        archived[name+'_m']=value
        current[name+'_m']=getattr(parameters,name+'_mm')/1000.
    matches=all(np.isclose(archived[key],current[key],atol=1e-12,rtol=0) for key in archived)
    if require_match and not matches:
        raise ValueError('YAML geometry differs from archive; use --preserve-archive-geometry to keep archived geometry/TCP')
    def digest(value):
        return hashlib.sha256(json.dumps(value,sort_keys=True,allow_nan=False).encode()).hexdigest()
    return dict(policy='preserve_archive',archived_offsets=archived,current_yaml_offsets=current,
                yaml_offsets_match_archive=matches,geometry_sha256=digest(geometry),
                fingertip_estimates_sha256=digest(geometry['fingertip_estimates']),
                calibration=geometry.get('calibration'),current_calibration_loaded=False,
                derived_legacy_offsets=derived,
                derivation_evidence='Full archived arc matches physical-center circle to 1e-9m; input geometry and TCP unchanged' if derived else None)


def main():
    from dataclasses import asdict,replace
    from task_planning_config import DEFAULT_CONFIG,load_parameters,configure_planner
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument('--capture',type=Path,default=SESSION/'plan_0008')
    add_replay_source_arguments(parser)
    parser.add_argument('--arm',choices=('left','right'),default='left')
    parser.add_argument('--sweep-direction',choices=('left','right'),default='left')
    parser.add_argument('--velocity',type=float,default=.65)
    parser.add_argument('--acceleration',type=float,default=.65)
    parser.add_argument('--rviz',action='store_true')
    parser.add_argument('--keep-open',action='store_true')
    parser.add_argument('--contract-tests',action='store_true')
    parser.add_argument('--contracts-only',action='store_true')
    parser.add_argument('--budget',type=float,default=180.)
    parser.add_argument('--output-dir',type=Path,help='New output directory; existing paths are rejected')
    parser.add_argument('--export-archive',action='store_true',help='Freeze and reload a complete production offline task')
    parser.add_argument('--planning-config',type=Path,default=DEFAULT_CONFIG)
    parser.add_argument('--ik-search-seconds',type=float)
    parser.add_argument('--ik-search-attempts',type=int)
    parser.add_argument('--ik-backend',choices=('moveit','bounded','bounded-task','constrained'),default='moveit',
        help='Explicit offline candidate-solver ablation; final scoped validators are unchanged')
    parser.add_argument('--joint-route-backend',choices=('line','ompl'),default='line',
        help='Offline return-only OMPL fallback after scoped joint-line collision')
    parser.add_argument('--preinsert-policy',choices=('legacy','diverse'),default='legacy',
        help='Offline initial-candidate scheduling experiment; first batch and validation gates are unchanged')
    parser.add_argument('--contact-search-seconds',type=float,
        help='Override production contact-search total budget from first contact search')
    parser.add_argument('--production-planner',action='store_true',
        help='Offline adapter for production ScopedTaskPlanner; scoped collision checks only')
    parser.add_argument('--preserve-archive-geometry',action='store_true',
        help='Keep archived compensation geometry and TCP even when current YAML offsets differ')
    parser.add_argument('--articulated-hands',action='store_true')
    parser.add_argument('--waist-assist',action='store_true')
    parser.add_argument('--orientation-search',choices=('local','broad'),default='broad')
    parser.add_argument('--kinematic-preview',action='store_true',
        help='Offline only: skip ALL collision checks; never produce a hardware-validated plan')
    parser.add_argument('--hook-preview',action='store_true',help='Offline curled index with hand/base-only FCL checks')
    parser.add_argument('--straight-preview',action='store_true',help='Offline steeper straight index with scoped base/body checks')
    parser.add_argument('--center-check',action='store_true',help='Noncontact index TCP at shaft center plus configured vertical height')
    parser.add_argument('--compare-contact-timing',action='store_true',
        help='Offline only: validate old and fitted timing on each identical raw IK contact path')
    parser.add_argument('--max-index-tilt-deg',type=float)
    parser.add_argument('--robot-raise-m',type=float,default=0.,
        help='Synthetic additional robot height relative to the archive, meters; preview only')
    parser.add_argument('--contact-feasibility-only',type=Path,metavar='REPORT',
        help='Diagnose contact paths from a prior replay candidate report; never publish an executable plan')
    args=parser.parse_args()
    if args.start_plan is None and args.start_state is None:
        args.start_plan=SESSION/'plan_0004/plan.json'
    require_offline_environment(args.ros_domain_id)
    if args.kinematic_preview and args.ros_domain_id!=93:
        parser.error('Existing unchecked preview planner supports domain 93 only; use collision-checked replay on 94..99')
    if args.ik_backend!='moveit' and not args.production_planner:
        parser.error('Experimental --ik-backend requires --production-planner')
    if args.joint_route_backend!='line' and not args.production_planner:
        parser.error('Experimental --joint-route-backend requires --production-planner')
    if args.preinsert_policy!='legacy' and not args.production_planner:
        parser.error('Experimental --preinsert-policy requires --production-planner')
    if args.production_planner and (args.kinematic_preview or args.hook_preview or args.straight_preview
                                   or not args.articulated_hands):
        parser.error('--production-planner requires --articulated-hands and no preview modes')
    if args.export_archive and (not args.production_planner or args.contracts_only or
                               args.contact_feasibility_only or args.center_check):
        parser.error('--export-archive requires a complete production D-ring task')
    if args.contact_search_seconds is not None and not args.production_planner:
        parser.error('--contact-search-seconds requires --production-planner')
    if not np.isfinite(args.budget) or args.budget<=0:parser.error('--budget must be finite and positive')
    if args.center_check and (not args.straight_preview or args.compare_contact_timing):
        parser.error('Center check requires --straight-preview, without timing comparison')
    parameters,config_record=load_parameters(args.planning_config)
    if args.ik_search_seconds is None:args.ik_search_seconds=parameters.budget_seconds if (args.straight_preview or args.production_planner) else 40.
    if args.ik_search_attempts is None:args.ik_search_attempts=parameters.max_attempts if (args.straight_preview or args.production_planner) else 600
    if args.max_index_tilt_deg is None:args.max_index_tilt_deg=parameters.max_index_tilt_deg
    parameters=replace(parameters,budget_seconds=args.ik_search_seconds,max_attempts=args.ik_search_attempts,
                       max_index_tilt_deg=args.max_index_tilt_deg)
    if args.contact_search_seconds is not None:
        parameters=replace(parameters,contact_search_seconds=args.contact_search_seconds)
    from task_planning_config import parameters_from_values
    parameters=parameters_from_values(asdict(parameters))
    config_record['effective_values']=asdict(parameters)
    if (args.hook_preview or args.straight_preview) and not (args.kinematic_preview and args.articulated_hands):
        parser.error('Scoped previews require --kinematic-preview --articulated-hands')
    if args.hook_preview and args.straight_preview:parser.error('Choose one finger posture')
    if args.compare_contact_timing and not args.straight_preview:
        parser.error('Contact timing comparison requires --straight-preview')
    if not np.isfinite(args.max_index_tilt_deg) or not 10<=args.max_index_tilt_deg<=50:
        parser.error('Max index tilt must be in [10,50] degrees from downward')
    if args.robot_raise_m and not args.kinematic_preview:
        parser.error('Synthetic height experiments require --kinematic-preview')
    if not np.isfinite(args.robot_raise_m) or not 0<=args.robot_raise_m<=.30:
        parser.error('--robot-raise-m must be between 0 and 0.30')
    if args.kinematic_preview and (args.contract_tests or args.contracts_only or args.contact_feasibility_only):
        parser.error('Kinematic preview cannot be combined with collision contract/feasibility tests')
    case_started=time.monotonic()
    geometry=read_archive(args.capture/'geometry.json')
    geometry_record=None
    if args.production_planner or args.straight_preview or args.preserve_archive_geometry:
        geometry_record=archived_geometry_record(geometry,parameters,
            require_match=args.straight_preview and not args.preserve_archive_geometry)
    if args.start_state:
        q=load_start_state(args.start_state,geometry,args.articulated_hands,args.waist_assist)
    else:
        saved=read_archive(args.start_plan)
        q=dict(saved['start'])
        for segment in saved['segments']:
            trajectory=segment['trajectory']
            names=trajectory['joint_names'];positions=trajectory['points'][-1]['positions']
            if len(names)!=len(positions) or len(set(names))!=len(names):
                raise ValueError('Malformed archived plan endpoint')
            q.update(zip(names,positions))
        q=checked_joints(q,args.articulated_hands)
    output=args.output_dir or ROOT/'data/offline_task_tests'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    output.mkdir(parents=True)
    report={'offline_only':True,'hardware_commands_sent':0,'capture':str(args.capture.resolve()),
            'state_source':str((args.start_state or args.start_plan).resolve()),
            'state_note':('Archived capture-time measured state, NOT live feedback' if args.start_state else
                          'Reconstructed previous completed plan endpoint, NOT measured state at this capture'),
            'arm':args.arm,'sweep_direction':args.sweep_direction,'ros_domain_id':args.ros_domain_id,'success':False,
            'velocity_scaling':args.velocity,'acceleration_scaling':args.acceleration,
            'articulated_hands':args.articulated_hands,'waist_assist':args.waist_assist,
            'orientation_search':args.orientation_search,'ik_backend':args.ik_backend}
    report.update(collision_checked=not args.kinematic_preview,hardware_executable=False,
                  mode='KINEMATIC_PREVIEW_UNCHECKED' if args.kinematic_preview else 'COLLISION_CHECKED_OFFLINE')
    report['robot_raise_m']=args.robot_raise_m
    report['budget_seconds']=args.budget
    report['planner_class']='ScopedTaskPlanner' if args.production_planner else ('preview' if args.kinematic_preview else 'TaskPlanner')
    report['planning_config']=config_record
    report['runtime_sha256']=runtime_hashes()
    report['runtime_snapshot_scope']=RUNTIME_SCOPE
    if geometry_record is not None:report['archive_geometry']=geometry_record
    if args.hook_preview:report.update(mode='HOOK_PREVIEW_BASE_ONLY',base_collision_checked=True)
    if args.straight_preview:report.update(mode='STRAIGHT_PREVIEW_SCOPED',base_collision_checked=True,
        arm_body_collision_checked=True,max_index_tilt_deg=args.max_index_tilt_deg)
    if args.production_planner:report.update(mode='PRODUCTION_SCOPED_OFFLINE',collision_checked=False,
        base_collision_checked=True,arm_body_collision_checked=True,full_self_collision_checked=False,
        environment_collision_checked=False)
    report['height_note']='Additional height relative to archive; archived physical platform height unknown'
    print('Offline report: '+str(output),flush=True)
    if geometry_record is not None:
        report.update(geometry_record['archived_offsets'])
        (output/'preview_geometry.json').write_text(json.dumps(geometry,indent=2)+'\n')
    cloud=load_cloud(args.capture,geometry)
    # Validate saved RGB and depth as well as the cloud checksum checked above.
    manifest=json.loads((args.capture/'manifest.json').read_text())
    for name in ('rgb.png','depth_aligned.png'):
        if hashlib.sha256((args.capture/name).read_bytes()).hexdigest()!=manifest['sha256'][name]:
            raise RuntimeError('Saved RGB/depth checksum mismatch: '+name)
    if args.articulated_hands:
        report['hand_state_note']='Archived model hand angles; no missing posture inferred; not a calibration'
    if args.waist_assist and 'waist_yaw_joint' not in q:
        raise RuntimeError('Archived waist yaw required; cannot assume a capture transform')
    (output/'reconstructed_state.json').write_text(json.dumps({'joints':q,'provenance':report['state_note']},indent=2)+'\n')
    urdf=output/'robot.urdf'
    import yaml
    config=yaml.safe_load((ROOT/'assets/camera_reference/real_task.rviz').read_text())
    config['Visualization Manager']['Displays'][0]['Name']='Robot - ARCHIVED reconstructed joints (OFFLINE)'
    config['Visualization Manager']['Displays'][1]['Name']='ARCHIVED RGB-D - not live'
    trajectory_topic='/g1/offline/unchecked_path' if args.kinematic_preview else '/g1/task/validated_path'
    if args.kinematic_preview:
        display_config=config['Visualization Manager']['Displays'][3]
        display_config['Name']='OFFLINE ONLY - collision UNCHECKED'
        display_config['Trajectory Topic']=trajectory_topic
        display_config['Robot Alpha']=1.
        config['Visualization Manager']['Displays'][0]['Enabled']=False
    rviz=output/'offline.rviz';rviz.write_text(yaml.safe_dump(config,sort_keys=False))
    command=['ros2','launch','g1_upper_body_moveit_config','real_shadow_moveit.launch.py',
        'task_application_enabled:=true','active_arm:='+args.arm,'start_state_bridge:=false',
        'task_articulated_hands:='+str(args.articulated_hands).lower(),
        'task_waist_enabled:='+str(args.waist_assist).lower(),
        'start_hardware_gateway:=false','hardware_output_enabled:=false','start_shadow_executor:=false',
        'enable_rviz_interactive_execution:=false','show_planned_overlay:=false',
        'use_rviz:='+str(args.rviz).lower(),'export_robot_urdf:='+str(urdf),'rviz_config:='+str(rviz)]
    rclpy.init()
    node_name=f'g1_task_offline_replay_{os.getpid()}'
    node=rclpy.create_node(node_name)
    # Never share a preexisting planning scene, even on the isolated test domain.
    time.sleep(.5)
    if any(name!=node_name for name,_ in node.get_node_names_and_namespaces()):
        node.destroy_node();rclpy.shutdown()
        raise RuntimeError(f'Domain {args.ros_domain_id} has existing nodes; refusing to overwrite their scene')
    executor=MultiThreadedExecutor(num_threads=3);executor.add_node(node)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    state_pub=node.create_publisher(JointState,'/joint_states',10)
    cloud_pub=node.create_publisher(PointCloud2,'/g1/vision_rgbd_surface',qos)
    markers=node.create_publisher(MarkerArray,'/g1/vision_task_preview',qos)
    display=node.create_publisher(DisplayTrajectory,trajectory_topic,qos)
    def state():
        msg=JointState();msg.header.stamp=node.get_clock().now().to_msg()
        msg.name=list(q);msg.position=[float(q[n]) for n in q];state_pub.publish(msg)
    node.create_timer(.02,state)
    deadline=case_started+args.budget
    supervised=os.environ.get('G1_OFFLINE_SUPERVISED_PGID')==str(os.getpgrp())
    rpc_seconds={}
    def check():
        if time.monotonic()>deadline:raise TimeoutError('Offline planning budget exceeded')
    def call(client,request,ticket=None,timeout=15):
        check()
        started=time.monotonic()
        try:
            await_service(client,check,min(deadline,time.monotonic()+10.))
            return wait_for_result(client.call_async(request),check,timeout)
        finally:rpc_seconds.setdefault(client.srv_name,[]).append(time.monotonic()-started)
    launch=None
    try:
        with (output/'ros_launch.log').open('w') as log:
            launch=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=not supervised)
            while not urdf.is_file():
                check()
                if launch.poll() is not None:raise RuntimeError('Offline launch failed; inspect ros_launch.log')
                time.sleep(.1)
            model=OfflineModel(urdf)
            if args.kinematic_preview:
                from task_preview_height import raised_preview_geometry
                geometry=raised_preview_geometry(geometry,model,q,args.robot_raise_m)
                (output/'preview_geometry.json').write_text(json.dumps(geometry,indent=2)+'\n')
            planner_class=TaskPlanner
            preview_options={}
            if args.production_planner:
                from task_scoped_planner import ScopedTaskPlanner
                planner_class=ScopedTaskPlanner
                preview_options.update(robot_urdf=urdf,check_plan=lambda ticket:check(),
                                       max_index_tilt=args.max_index_tilt_deg)
            if args.kinematic_preview:
                from task_kinematic_preview import KinematicPreviewPlanner
                planner_class=KinematicPreviewPlanner;preview_options['check_preview']=check
                if args.hook_preview:
                    from task_hook_preview import HookPreviewPlanner
                    planner_class=HookPreviewPlanner;preview_options['preview_urdf']=urdf
                if args.straight_preview:
                    from task_straight_preview import StraightPreviewPlanner
                    planner_class=StraightPreviewPlanner;preview_options.update(preview_urdf=urdf,max_index_tilt=args.max_index_tilt_deg)
            planner=planner_class(node,model,call,lambda t,m:print(m,flush=True),
                                search_seconds=args.ik_search_seconds,search_attempts=args.ik_search_attempts,
                                waist_assist=args.waist_assist,articulated_hands=args.articulated_hands,
                                orientation_search=args.orientation_search,**preview_options)
            planner.ik_backend=args.ik_backend
            planner.joint_route_backend=args.joint_route_backend
            planner.preinsert_policy=args.preinsert_policy
            report['preinsert_policy']=args.preinsert_policy
            report['joint_route_backend']=args.joint_route_backend
            if args.joint_route_backend=='ompl':
                from task_joint_route import backend_identity
                report['joint_route_backend_version']=backend_identity()
            if args.ik_backend in ('bounded','bounded-task','constrained'):
                import scipy
                report['ik_backend_version']=dict(method=('scipy_bounded_task_trf_v1'
                    if args.ik_backend=='bounded-task' else 'scipy_bounded_trf_v1'),scipy=scipy.__version__,
                    numpy=np.__version__,randomized=False,seed_policy='existing ordered preinsert/contact seeds',
                    note='Iteration order is deterministic; wall-clock deadlines can still truncate search differently')
                if args.ik_backend=='bounded-task':
                    report['ik_backend_version'].update(contact_candidate_position_tolerance_m=.002,
                        contact_candidate_orientation_tolerance_rad=math.radians(2.),
                        contact_continuity_interval_rad=.119,preinsert_unchanged=True,
                        final_validators_unchanged=True)
                if args.ik_backend=='constrained':
                    report['ik_backend_version'].update(method='scipy_task_slsqp_v1',
                        contact_candidate_position_tolerance_m=.002,contact_continuity_interval_rad=.119,
                        tangent_reference='physical_center_fixed_branch_actual_and_nominal_tcp',
                        contact_candidates='one constrained solve per existing ordered joint seed',
                        preinsert_method='scipy_bounded_trf_v1',preinsert_unchanged=True,
                        final_validators_unchanged=True)
            if args.straight_preview or args.production_planner:
                configure_planner(planner,parameters)
                home_path=args.capture.parent/'session_home.json'
                startup=json.loads(home_path.read_text())['joints'] if home_path.is_file() else q
                ready_path=args.capture.parent/'ready_poses.json'
                ready_data=json.loads(ready_path.read_text()) if ready_path.is_file() else {}
                ready=ready_data.get(args.arm,{})
                ready_available=bool(ready and 'joints' in ready)
                if not ready_available:ready={'joints':q}
                planner.configure_task(args.arm,startup,ready['joints'],args.sweep_direction)
                report['startup_heading_source']=str(home_path) if home_path.is_file() else 'capture state fallback'
                report['ready_source']=str(ready_path) if ready_available else 'capture state fallback; no recorded '+args.arm+' ready pose'
                if args.compare_contact_timing:
                    import copy
                    from check_task_path_smoothing import metrics
                    fitted_retime=planner.retime_cartesian
                    def compare(side,q,path,ticket,label,check,stage,geometry_report):
                        if stage not in ('rotate','withdraw'):
                            return fitted_retime(side,q,path,ticket,label,check,stage,geometry_report)
                        empty=copy.deepcopy(geometry_report)
                        baseline=planner.retime(side,q,copy.deepcopy(path),ticket,label,check,stage)
                        baseline_geometry=copy.deepcopy(geometry_report)
                        geometry_report.clear();geometry_report.update(empty)
                        fitted=fitted_retime(side,q,path,ticket,label,check,stage,geometry_report)
                        for endpoint in ('start','end'):
                            np.testing.assert_allclose([fitted[endpoint][n] for n in planner.names(side)],
                                [baseline[endpoint][n] for n in planner.names(side)],atol=1e-10,rtol=0)
                        report.setdefault('same_raw_path_comparison',[]).append(dict(label=label,
                            before=metrics(baseline['trajectory']),after=metrics(fitted['trajectory']),
                            before_geometry=baseline_geometry,after_geometry=copy.deepcopy(geometry_report)))
                        return fitted
                    planner.retime_cartesian=compare
            readiness_started=time.monotonic();readiness_deadline=min(deadline,readiness_started+60.)
            for client in (planner.valid,planner.scene,planner.ik,planner.motion,planner.timing):
                await_service(client,check,readiness_deadline)
            report['service_readiness_seconds']=time.monotonic()-readiness_started
            if args.contract_tests or args.contracts_only:
                from task_offline_contract import verify_contact_contract, verify_hand_waist_contract
                report['contact_contract']=verify_contact_contract(planner,q)
                if args.articulated_hands or args.waist_assist:
                    report['hand_waist_contract']=verify_hand_waist_contract(planner,q)
            if args.contracts_only:
                report['success']=True
                report['note']='Contract fixtures only; no D-ring plan attempted'
                return 0
            torso=capture_transform(geometry,model,q)
            with np.load(args.capture/'cloud.npz',allow_pickle=False) as data:
                xyz,uv=data['xyz'].copy(),data['uv'].copy()
            packed=np.frombuffer(bytes(cloud.data),dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('rgb','<u4')]).copy()
            frozen=xyz@torso[:3,:3].T+torso[:3,3]
            for i,axis in enumerate('xyz'):packed[axis]=frozen[:,i]
            cloud.data=packed.tobytes();cloud.header.frame_id='pelvis'
            cloud.header.stamp=node.get_clock().now().to_msg();cloud_pub.publish(cloud)
            markers.publish(target_markers(geometry,torso,args.arm,'task'))
            server=json.loads((args.capture/'server_result.json').read_text())
            ticket=Ticket(1,'task',args.arm,args.velocity,args.acceleration)
            planner.install_surface(xyz,torso,ticket,uv=uv,bbox=server.get('pedal_centered'),
                                    center=geometry['task_preview']['axis_center_m'])
            report['scene']=planner.scene_report
            started=time.monotonic()
            try:
                if args.contact_feasibility_only:
                    previous=json.loads(args.contact_feasibility_only.read_text())
                    previous_state=json.loads((args.contact_feasibility_only.parent/'reconstructed_state.json').read_text())['joints']
                    if (not args.waist_assist or previous.get('orientation_search')!='broad' or
                            previous['capture']!=str(args.capture.resolve()) or previous['arm']!=args.arm or
                            previous['waist_assist']!=args.waist_assist or
                            previous['articulated_hands']!=args.articulated_hands or previous_state!=q):
                        raise RuntimeError('Contact diagnostic requires broad waist search and the same capture, state, arm and model flags')
                    presearch=previous['search_diagnostics']['preinsert']
                    wrist=np.asarray(geometry['fingertip_estimates']['sides'][args.arm]['states']['open']
                                     ['wrist_yaw_from_index_tcp']['matrix_row_major'])
                    frozen=capture_transform(geometry,model,q)
                    transform=lambda p:frozen[:3,:3]@np.asarray(p)+frozen[:3,3]
                    entry=transform(geometry['task_preview']['insertion_reference_m'])
                    task=geometry['task_preview']
                    center=torso[:3,:3]@np.asarray(task.get('arc_center_m',task['axis_center_m']))+torso[:3,3]
                    report['contact_feasibility']={'source':str(args.contact_feasibility_only),
                        'complete':False,'trials':[],
                        'note':'Diagnostic only: excludes approach, hand transitions and return-to-ready; no PLAN_READY'}
                    for candidate in presearch['ranked_candidates']:
                        trial={'trial_index':candidate['trial_index']}
                        report['contact_feasibility']['trials'].append(trial)
                        try:
                            planner.validate(candidate['goal'],ticket,args.arm,'approach')
                            segments=planner.contact_segments(args.arm,candidate['goal'],
                                np.asarray(candidate['preinsert_tcp_matrix']),entry,center,
                                candidate.get('orientation_reference_xyzw',
                                    presearch['trials'][candidate['trial_index']]['quaternion_xyzw']),wrist,ticket)
                            trial['complete']=True
                            report['contact_feasibility']['complete']=True
                            report['contact_feasibility']['segment_labels']=[s['label'] for s in segments]
                            break
                        except Exception as error:
                            trial['error']=str(error);trial['diagnostic']=getattr(error,'diagnostic',None)
                            if isinstance(error,TimeoutError):raise
                            if trial['diagnostic'] is None:raise
                    return 0 if report['contact_feasibility']['complete'] else 1
                if args.center_check:
                    from task_center_check import plan_center_check,SCHEMA,measurement
                    segments=plan_center_check(planner,args.arm,q,geometry,ticket)
                    report['task_schema']=SCHEMA
                    report['center_check']=geometry['center_check']
                    report['ideal_endpoint_measurement']=measurement(model,args.arm,segments[-1]['end'],geometry['center_check'])
                    (output/'preview_geometry.json').write_text(json.dumps(geometry,indent=2)+'\n')
                else:segments=planner.ring(args.arm,q,geometry,ticket)
                report['success']=True
                report['segments']=[{'label':s['label'],'stage':s['stage'],'kind':s.get('kind','arm'),
                    'side':s['side'],'posture':s.get('posture'),'target_q':s.get('target_q'),
                    'validation':s.get('validation'),'geometry_validation':s.get('geometry_validation'),
                    'trajectory':message_to_ordereddict(s['trajectory'])} for s in segments]
                if args.export_archive:
                    from task_offline_export import export_archive
                    export_started=time.monotonic()
                    report['planning_seconds_at_export']=export_started-started
                    report['case_seconds_at_export']=export_started-case_started
                    report['search_diagnostics']=planner.search_diagnostics
                    report['archive']=export_archive(args.capture,output/'archive',urdf,report,q,check)
                    report['export_seconds']=time.monotonic()-export_started
                message=DisplayTrajectory(trajectory_start=robot_state(q))
                message.trajectory=[RobotTrajectory(joint_trajectory=s['trajectory']) for s in segments]
                display.publish(message)
            finally:
                report['planning_seconds']=time.monotonic()-started
                report['search_diagnostics']=planner.search_diagnostics
    except Exception as error:
        report['success']=False
        report['error']=str(error);report['diagnostic']=getattr(error,'diagnostic',None)
        print(str(error),flush=True)
    finally:
        report['rpc_timing']={name:{'count':len(values),'total_seconds':sum(values),
            'median_seconds':float(np.median(values)),'p95_seconds':float(np.percentile(values,95)),
            'max_seconds':max(values)} for name,values in rpc_seconds.items()}
        report['timing_scope']='case reads through validation; includes service startup; excludes cleanup'
        if not runtime_matches(report['runtime_sha256']):
            report.update(success=False,error='Native runtime changed during offline replay')
        report['case_seconds_before_report_save']=time.monotonic()-case_started
        if report['case_seconds_before_report_save']>=args.budget:
            report.update(success=False,error='Full case deadline exceeded')
        (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        print('Saved '+str(output/'report.json'),flush=True)
        try:
            if args.keep_open and launch is not None and launch.poll() is None:
                print('OFFLINE preview only. Ctrl+C closes only this replay.',flush=True)
                while launch.poll() is None:time.sleep(.2)
        except KeyboardInterrupt:pass
        finally:
            if launch is not None and launch.poll() is None:
                if supervised:launch.send_signal(signal.SIGINT)
                else:os.killpg(launch.pid,signal.SIGINT)
                try:launch.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    if supervised:launch.kill()
                    else:os.killpg(launch.pid,signal.SIGKILL)
                    launch.wait()
            executor.shutdown();thread.join(timeout=3);node.destroy_node();rclpy.shutdown()
    return 0 if report['success'] else 1


if __name__=='__main__':raise SystemExit(main())
