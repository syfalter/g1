"""One-time legacy GUI migration. Retains the gateway and uses its guarded hold API."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from g1_unitree_control.srv import ResumeTaskHold

from task_console_handoff import process_identity, supported_gateway, validate_handoff
from task_future import wait_for_result
from task_session import Session, ARM_NAMES

ROOT=Path(__file__).resolve().parents[1]


def argv(pid):
    return [value.decode() for value in (Path('/proc')/str(pid)/'cmdline').read_bytes().split(b'\0') if value]


def descendants(pid):
    path=Path(f'/proc/{pid}/task/{pid}/children')
    try: children=[int(value) for value in path.read_text().split()]
    except FileNotFoundError: return []
    return children+[child for parent in children for child in descendants(parent)]


def replacement_command(console_args, manifest_path, saved_test_archive=None, dring_contact_test=False):
    command=list(console_args)
    if '--attach-handoff' in command:
        index=command.index('--attach-handoff')
        del command[index:index+2]
    if saved_test_archive:
        if '--saved-test-archive' in command:
            index=command.index('--saved-test-archive');del command[index:index+2]
        command+=['--saved-test-archive',str(saved_test_archive)]
    if dring_contact_test and '--dring-contact-test' not in command:command.append('--dring-contact-test')
    return command+['--attach-handoff',str(manifest_path)]


def ready_poses_for_handoff(session_dir, console_args):
    path=session_dir/'ready_poses.json'
    if not path.exists() and '--ready-poses' in console_args:
        path=Path(console_args[console_args.index('--ready-poses')+1])
    if not path.is_file():
        raise RuntimeError('Current ready poses unavailable; refusing a historical fallback')
    return json.loads(path.read_text())


def verify_console_unit(unit,pid,parent_unit):
    result=subprocess.run(['systemctl','--user','show',unit,'-p','MainPID','-p','BindsTo'],
                          check=True,capture_output=True,text=True,timeout=5)
    fields=dict(line.split('=',1) for line in result.stdout.splitlines() if '=' in line)
    if fields.get('MainPID')!=str(pid) or not parent_unit or parent_unit not in fields.get('BindsTo','').split():
        raise RuntimeError('Console unit identity/lifecycle does not match the retained supervisor')


def start_unit(name, command, environment, stop_on_exit=False, parent_unit=None):
    keys=('PATH','LD_LIBRARY_PATH','PYTHONPATH','AMENT_PREFIX_PATH','CMAKE_PREFIX_PATH',
          'RMW_IMPLEMENTATION','CYCLONEDDS_URI','ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY',
          'DISPLAY','WAYLAND_DISPLAY','XDG_RUNTIME_DIR','XAUTHORITY','DBUS_SESSION_BUS_ADDRESS',
          'G1_VISION_SSH_CONTROL_PATH','QT_QPA_PLATFORM','QT_PLUGIN_PATH')
    env_args=['--setenv='+key+'='+environment[key] for key in keys if key in environment]
    stop_args=[]
    if stop_on_exit:
        stop_args=['--property=TimeoutStopSec=5s',
                   '--property=ExecStopPost=/usr/bin/timeout 5s /usr/bin/python3 /opt/ros/humble/bin/ros2 '
                   'service call /g1/arm_gateway/stop_task std_srvs/srv/Trigger {}']
    lifecycle=(['--property=PartOf='+parent_unit,'--property=BindsTo='+parent_unit,
                '--property=After='+parent_unit] if parent_unit else [])
    subprocess.run(['systemd-run','--user','--collect','--unit='+name,*env_args,*stop_args,*lifecycle,'--',*command],check=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--console-pid',type=int,required=True)
    parser.add_argument('--gateway-pid',type=int,required=True)
    parser.add_argument('--launcher-pid',type=int,required=True)
    parser.add_argument('--rviz-pid',type=int,required=True)
    parser.add_argument('--session-dir',type=Path,required=True)
    parser.add_argument('--parent-unit',help='Existing user service that owns the retained gateway')
    parser.add_argument('--console-unit',help='Existing attached console unit; its stop hook must finish before hold resumes')
    parser.add_argument('--saved-test-archive',type=Path,help='Preselect saved test data without loading or executing')
    parser.add_argument('--dring-contact-test',action='store_true',help='Enable explicitly requested full live contact tests')
    parser.add_argument('--allow-missing-home',action='store_true',
                        help='Attach with planning disabled until operator captures a return point')
    args=parser.parse_args()
    old_identity=process_identity(args.console_pid)
    gateway_identity=process_identity(args.gateway_pid)
    console_args=argv(args.console_pid)
    if str(ROOT/'scripts/real_task_app.py') not in console_args or '--hardware' not in console_args:
        raise RuntimeError('Not the expected hardware console')
    if 'arm_sdk_hardware_gateway' not in Path(argv(args.gateway_pid)[0]).name:
        raise RuntimeError('Not the expected gateway')
    if str(ROOT/'scripts/run_real_task_application.sh') not in argv(args.launcher_pid):
        raise RuntimeError('Not the expected legacy supervisor')
    family=descendants(args.launcher_pid)
    if any(pid not in family for pid in (args.gateway_pid,args.rviz_pid)):
        raise RuntimeError('Processes do not belong to the same legacy session')
    if args.console_unit:verify_console_unit(args.console_unit,args.console_pid,args.parent_unit)
    elif args.console_pid not in family:raise RuntimeError('Console is not a child of the expected supervisor')
    rviz_args=argv(args.rviz_pid)
    if Path(rviz_args[0]).name!='rviz2':raise RuntimeError('Not the expected RViz process')
    urdf=Path(console_args[console_args.index('--robot-urdf')+1])
    if not urdf.is_file():raise RuntimeError('Existing robot model is unavailable')
    environment=dict(item.decode().split('=',1) for item in
                     (Path('/proc')/str(args.console_pid)/'environ').read_bytes().split(b'\0') if item)
    home_path=args.session_dir/'session_home.json'
    home=(json.loads(home_path.read_text()) if home_path.exists() else None)
    if home is None and not args.allow_missing_home:
        raise RuntimeError('Missing session home; explicit pending-capture attachment required')
    ready=ready_poses_for_handoff(args.session_dir,console_args)
    report={'gateway_identity':gateway_identity,'old_console_identity':old_identity,
            'source_session':str(args.session_dir),'motion_trajectories_sent':0,'arm_calls':0,
            'gateway_restarted':False,'phases':[]}
    directory=ROOT/'data/console_handoffs'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    directory.mkdir(parents=True)
    rclpy.init();node=rclpy.create_node('task_console_migration')
    lock=threading.RLock(); state=Session(); diagnostics={}; received=float('-inf'); ui={}; ui_received=float('-inf')
    def on_diagnostics(msg):
        nonlocal diagnostics,received
        with lock:
            for status in msg.status:diagnostics={value.key:value.value for value in status.values}
            received=time.monotonic()
            phase=diagnostics.get('phase')
            if not report['phases'] or report['phases'][-1]['phase']!=phase:
                report['phases'].append({'phase':phase,'at':time.time()})
    def on_joints(msg):
        with lock:
            try:state.update(dict(zip(msg.name,msg.position)))
            except ValueError:pass
    def on_ui(msg):
        nonlocal ui,ui_received
        with lock:ui=json.loads(msg.data);ui_received=time.monotonic()
    node.create_subscription(DiagnosticArray,'/g1/arm_gateway/diagnostics',on_diagnostics,10)
    node.create_subscription(JointState,'/joint_states',on_joints,10)
    node.create_subscription(String,'/g1/task/console_status',on_ui,
                             QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    resume=node.create_client(ResumeTaskHold,'/g1/arm_gateway/hold_after_stop')
    executor=SingleThreadedExecutor();executor.add_node(node)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    def fresh():
        if process_identity(args.gateway_pid)!=gateway_identity:
            raise RuntimeError('Gateway identity changed; migration aborted')
        if time.monotonic()-received>1.:
            raise RuntimeError('Gateway diagnostics are stale')
        return dict(diagnostics)
    try:
        time.sleep(1.3)
        with lock:
            before=fresh();supported_gateway(before)
            initial=state.measured(stationary=True)
            if before['phase']!='ARMED_HOLD' or before['task_stop_active']!='false':
                raise RuntimeError('Migration only starts from an existing stationary hold')
            if (time.monotonic()-ui_received>1.5 or ui.get('console_pid')!=args.console_pid or
                    ui.get('worker_busy') is not False or ui.get('phase') in ('PLANNING','EXECUTING')):
                raise RuntimeError('Expected fresh idle console status; migration aborted')
            epoch=int(before['task_stop_epoch'])+1
            baseline={key:before[key] for key in ('received_trajectories','accepted_trajectories','rejected_trajectories')}
        manifest={'created_at_epoch_s':time.time(),'gateway_identity':gateway_identity,
                  'source_session':str(args.session_dir),'session_home':home,'ready_poses':ready,'stop_epoch':epoch}
        if home is None:manifest['startup_capture_pending']=True
        validate_handoff(manifest)
        if not resume.wait_for_service(timeout_sec=2):raise RuntimeError('Guarded hold service unavailable')
        if process_identity(args.console_pid)!=old_identity:raise RuntimeError('Console PID changed')
        print('Preflight passed. Replacing only the legacy GUI; controller stays alive.',flush=True)
        managed_stop_completed=False
        if args.console_unit:
            verify_console_unit(args.console_unit,args.console_pid,args.parent_unit)
            subprocess.run(['systemctl','--user','stop',args.console_unit],check=True,timeout=12)
            managed_stop_completed=True
        else:os.kill(args.console_pid,signal.SIGTERM)
        deadline=time.monotonic()+3
        while Path(f'/proc/{args.console_pid}').exists() and time.monotonic()<deadline:time.sleep(.02)
        if Path(f'/proc/{args.console_pid}').exists():
            if process_identity(args.console_pid)!=old_identity:raise RuntimeError('Console PID changed')
            os.kill(args.console_pid,signal.SIGKILL)
        # The legacy supervisor performs exactly one stop on GUI exit. Do not
        # resume until its service client has exited, or if another stop occurred.
        saw_cleanup=managed_stop_completed;deadline=time.monotonic()+10
        while time.monotonic()<deadline:
            cleanup=[]
            for pid in descendants(args.launcher_pid):
                try:command=argv(pid)
                except (FileNotFoundError,ProcessLookupError):continue
                if '/g1/arm_gateway/stop_task' in command:cleanup.append(pid)
            saw_cleanup=saw_cleanup or bool(cleanup)
            with lock:
                current=fresh()
                if int(current['task_stop_epoch'])>epoch:
                    raise RuntimeError('Additional stop observed: left stopped, no automatic resume')
                acknowledged=current['task_stop_active']=='true' and int(current['task_stop_epoch'])==epoch
            if saw_cleanup and not cleanup and acknowledged:break
            time.sleep(.02)
        else:raise RuntimeError('Legacy stop completion not verified; no resume sent')
        with lock:
            current=fresh();supported_gateway(current)
            measured=state.measured(stationary=True)
            watched=ARM_NAMES+(('waist_yaw_joint',) if 'waist_yaw_joint' in initial else ())
            drift=max(abs(measured[n]-initial[n]) for n in watched)
            if drift>.012:raise RuntimeError('Arms moved during migration; no automatic resume')
            if any(current[key]!=value for key,value in baseline.items()):
                raise RuntimeError('Unexpected trajectory activity; no automatic resume')
        response=wait_for_result(resume.call_async(ResumeTaskHold.Request(stop_epoch=epoch)),lambda:None,3)
        if not response.success:raise RuntimeError('Guarded hold refused: '+response.message)
        deadline=time.monotonic()+5
        while time.monotonic()<deadline:
            with lock:
                current=fresh()
                if int(current['task_stop_epoch'])!=epoch:raise RuntimeError('New stop; handoff aborted')
                if current['phase']=='ARMED_HOLD' and current['task_stop_active']=='false':break
            time.sleep(.02)
        else:raise RuntimeError('Hold acknowledgement timed out')
        manifest['created_at_epoch_s']=time.time()
        manifest_path=directory/'handoff.json'
        manifest_path.write_text(json.dumps(manifest,indent=2)+'\n')
        unit='g1-task-console-'+directory.name.lower().replace('_','-')
        command=replacement_command(console_args,manifest_path,args.saved_test_archive,args.dring_contact_test)
        start_unit(unit,command,environment,stop_on_exit=True,parent_unit=args.parent_unit)
        report['console_unit']=unit
        report['console_command']=command
        deadline=time.monotonic()+15
        while time.monotonic()<deadline:
            with lock:
                if (ui.get('console_pid')!=args.console_pid and time.monotonic()-ui_received<1.5 and
                        ui.get('phase')=='HOLD_FOR_REPLAN' and ui.get('gateway')=='ARMED_HOLD'):break
            time.sleep(.1)
        else:raise RuntimeError('New console did not confirm attachment; controller was left holding')
        report['rviz_restarted']=False
        report['rviz_pid']=args.rviz_pid
        time.sleep(1)
        with lock:
            final=fresh();supported_gateway(final)
            if final['phase']!='ARMED_HOLD' or final['task_stop_active']!='false':raise RuntimeError('Hold changed after attachment')
            if any(final[key]!=value for key,value in baseline.items()):raise RuntimeError('Trajectory counters changed')
            report.update({'result':'PASS','new_console_status':dict(ui),
                           'max_joint_drift_before_hold_rad':drift,'stop_epoch':epoch,
                           'trajectory_counters':baseline,'gateway_command_count_before':before['hardware_command_count'],
                           'gateway_command_count_after':final['hardware_command_count']})
        print(json.dumps(report,indent=2),flush=True)
    except Exception as error:
        report.update({'result':'INCOMPLETE','error':str(error)})
        print(json.dumps(report,indent=2),flush=True)
        raise
    finally:
        (directory/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        executor.shutdown();thread.join(timeout=3);node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
