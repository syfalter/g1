"""Close/reopen the GUI while retaining the original hardware controller."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from migrate_legacy_task_console import argv,descendants,ready_poses_for_handoff,replacement_command,start_unit,verify_console_unit
from task_console_handoff import acquire_console_lock,process_identity,supported_gateway,validate_handoff
from task_planning_config import DEFAULT_CONFIG,load_parameters

ROOT=Path(__file__).resolve().parents[1]
CONTEXT=ROOT/'data/task_panel/retained_session.json'
ENV_KEYS=('PATH','LD_LIBRARY_PATH','PYTHONPATH','AMENT_PREFIX_PATH','CMAKE_PREFIX_PATH',
          'RMW_IMPLEMENTATION','CYCLONEDDS_URI','ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY',
          'DISPLAY','WAYLAND_DISPLAY','XDG_RUNTIME_DIR','XAUTHORITY','DBUS_SESSION_BUS_ADDRESS',
          'G1_VISION_SSH_CONTROL_PATH','QT_QPA_PLATFORM','QT_PLUGIN_PATH')


def check_identity(context):
    for key in ('gateway_identity','launcher_identity'):
        identity=context[key]
        if process_identity(identity['pid'])!=identity:raise RuntimeError('Original live session changed: '+key)
    command=context['console_command']
    if not Path(command[command.index('--robot-urdf')+1]).is_file():raise RuntimeError('Original robot model missing')


def attachment(context,gateway,*,panel_only=False,resume_session=False):
    check_identity(context)
    stopped=gateway['task_stop_active']=='true'
    if not panel_only:
        supported_gateway(gateway)
        if not stopped and gateway['phase']!='ARMED_HOLD':raise RuntimeError('Controller must be idle before closing the panel')
    manifest=dict(created_at_epoch_s=time.time(),gateway_identity=context['gateway_identity'],
                  source_session=context['source_session'],
                  session_home=context['session_home'] if not panel_only or resume_session else None,
                  ready_poses=context['ready_poses'],stop_epoch=int(gateway['task_stop_epoch']),
                  attachment_mode='panel' if panel_only else ('stopped' if stopped else 'hold'))
    if manifest['session_home'] is None:manifest['startup_capture_pending']=True
    return validate_handoff(manifest)


def refresh_session_context(context,directory=None):
    directory=Path(directory or context['source_session'])
    home=directory/'session_home.json'
    context.update(source_session=str(directory),session_home=json.loads(home.read_text()) if home.exists() else None,
                   ready_poses=ready_poses_for_handoff(directory,context['console_command']))


def save_context(context):
    CONTEXT.parent.mkdir(parents=True,exist_ok=True)
    temp=CONTEXT.with_suffix('.tmp')
    temp.write_text(json.dumps(context,indent=2)+'\n')
    temp.replace(CONTEXT)


def close_process(context):
    identity=context['console_identity'];pid=identity['pid']
    if process_identity(pid)!=identity:raise RuntimeError('Panel PID changed')
    if context.get('console_unit'):
        verify_console_unit(context['console_unit'],pid,context['parent_unit'])
        subprocess.run(['systemctl','--user','stop',context['console_unit']],check=True,timeout=12)
    else:
        os.kill(pid,signal.SIGTERM)
        deadline=time.monotonic()+5
        while Path(f'/proc/{pid}').exists() and time.monotonic()<deadline:time.sleep(.05)
        if Path(f'/proc/{pid}').exists():
            if process_identity(pid)!=identity:raise RuntimeError('Panel PID reused')
            os.kill(pid,signal.SIGKILL)
    deadline=time.monotonic()+3
    while Path(f'/proc/{pid}').exists() and time.monotonic()<deadline:time.sleep(.05)
    if Path(f'/proc/{pid}').exists():raise RuntimeError('Panel has not exited')
    check_identity(context)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=('close','start','status'))
    parser.add_argument('--console-pid',type=int)
    parser.add_argument('--gateway-pid',type=int)
    parser.add_argument('--launcher-pid',type=int)
    parser.add_argument('--parent-unit')
    parser.add_argument('--session-dir',type=Path)
    parser.add_argument('--planning-config',type=Path,default=DEFAULT_CONFIG)
    parser.add_argument('--resume-session',action='store_true',help='Explicitly reuse the last session return point; omit for a new startup confirmation')
    parser.add_argument('--disconnected-close',action='store_true',help='Close GUI only after disconnection or robot power-off; no stop acknowledgement')
    args=parser.parse_args()
    if args.console_pid is not None:
        if not all((args.gateway_pid,args.launcher_pid,args.parent_unit,args.session_dir)):
            parser.error('Initial context needs gateway/launcher/parent unit/session directory')
        command=argv(args.console_pid)
        if str(ROOT/'scripts/real_task_app.py') not in command or '--hardware' not in command:
            raise RuntimeError('Not the expected hardware panel')
        if (str(ROOT/'scripts/run_real_task_application.sh') not in argv(args.launcher_pid) or
                'arm_sdk_hardware_gateway' not in Path(argv(args.gateway_pid)[0]).name):
            raise RuntimeError('Not the expected controller supervisor')
        family=descendants(args.launcher_pid)
        if args.console_pid not in family or args.gateway_pid not in family:raise RuntimeError('Different live sessions')
        env=dict(item.decode().split('=',1) for item in Path(f'/proc/{args.console_pid}/environ').read_bytes().split(b'\0') if item)
        home=args.session_dir/'session_home.json'
        context=dict(console_identity=process_identity(args.console_pid),gateway_identity=process_identity(args.gateway_pid),
                     launcher_identity=process_identity(args.launcher_pid),parent_unit=args.parent_unit,
                     console_command=command,environment={k:env[k] for k in ENV_KEYS if k in env},
                     source_session=str(args.session_dir.resolve()),session_home=json.loads(home.read_text()) if home.exists() else None,
                     ready_poses=ready_poses_for_handoff(args.session_dir,command))
    else:context=json.loads(CONTEXT.read_text())
    if args.action=='close' and not Path(f"/proc/{context['console_identity']['pid']}").exists():
        print('Panel already closed. No process was stopped; controller and RViz unchanged.')
        return
    check_identity(context)
    if args.disconnected_close:
        if args.action!='close':raise RuntimeError('Disconnected close is only a GUI close operation')
        save_context(context);close_process(context)
        context.update(closed_at=datetime.now().isoformat(),close_stop_acknowledged=False)
        save_context(context)
        print('Panel closed. Controller process retained. Physical state and STOP acknowledgement unavailable.')
        return
    if args.action=='start':
        probe=acquire_console_lock();probe.close()
    os.environ.update(context['environment'])
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from std_msgs.msg import String
    from rclpy.qos import QoSProfile,DurabilityPolicy
    rclpy.init();node=rclpy.create_node('task_panel_control')
    received={};state={}
    def diagnostics(msg):
        for item in msg.status:state['gateway']={v.key:v.value for v in item.values}
        received['gateway']=time.monotonic()
    def ui(msg):
        state['ui']=json.loads(msg.data);received['ui']=time.monotonic()
    node.create_subscription(DiagnosticArray,'/g1/arm_gateway/diagnostics',diagnostics,10)
    node.create_subscription(String,'/g1/task/console_status',ui,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def wait_for(predicate,seconds=10):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
            check_identity(context)
            if time.monotonic()-received.get('gateway',float('-inf'))<1. and predicate():return
        raise RuntimeError('Timed out waiting for panel/controller acknowledgement')
    try:
        wait_for(lambda:'gateway' in state)
        if args.action=='status':
            deadline=time.monotonic()+.6
            while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
        elif args.action=='close':
            identity=context['console_identity'];pid=identity['pid']
            wait_for(lambda:time.monotonic()-received.get('ui',float('-inf'))<1.5 and state['ui'].get('console_pid')==pid)
            if state['ui'].get('worker_busy') or state['ui'].get('phase')=='EXECUTING':
                raise RuntimeError('Panel is busy; no process was stopped')
            attachment(context,state['gateway'],panel_only=state['gateway'].get('armed')=='false')
            if state['ui'].get('session_dir'):
                refresh_session_context(context,state['ui']['session_dir'])
            save_context(context)
            close_process(context)
            wait_for(lambda:not Path(f'/proc/{pid}').exists() and state['gateway'].get('task_stop_active')=='true')
            context.update(closed_at=datetime.now().isoformat(),close_stop_acknowledged=True);save_context(context)
            state['panel_closed']=True
        else:
            load_parameters(args.planning_config)
            refresh_session_context(context)
            manifest=attachment(context,state['gateway'],panel_only=True,resume_session=args.resume_session)
            stamp=datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            directory=ROOT/'data/console_handoffs'/stamp;directory.mkdir(parents=True)
            path=directory/'handoff.json';path.write_text(json.dumps(manifest,indent=2)+'\n')
            command=replacement_command(context['console_command'],path)
            if '--planning-config' in command:
                index=command.index('--planning-config');del command[index:index+2]
            command+=['--planning-config',str(args.planning_config.resolve())]
            unit='g1-task-console-'+stamp.replace('_','-')+'.service'
            start_unit(unit,command,context['environment'],stop_on_exit=True,parent_unit=context['parent_unit'])
            wait_for(lambda:state.get('ui',{}).get('console_pid')!=context['console_identity']['pid'] and
                     time.monotonic()-received.get('ui',float('-inf'))<1.5 and
                     state['ui'].get('worker_busy') is False and
                     state['ui'].get('attached_handoff')==str(path),seconds=20)
            context.update(console_unit=unit,console_identity=process_identity(state['ui']['console_pid']),console_command=command)
            refresh_session_context(context,state['ui']['session_dir'])
            save_context(context)
        print(json.dumps(state,indent=2),flush=True)
    finally:
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
