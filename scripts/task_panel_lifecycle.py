"""Start/stop the complete task application: console, controller, planner and RViz."""
import argparse
from datetime import datetime
import json
import math
import os
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from migrate_legacy_task_console import argv, descendants
from task_console_handoff import acquire_console_lock, process_identity
from task_panel_control import (ROOT, CONTEXT, ENV_KEYS, close_process,
                                refresh_session_context, save_context)
from task_planning_config import DEFAULT_CONFIG, load_parameters


def alive(identity):
    if not identity:
        return False
    try:
        current = process_identity(identity['pid'])
    except (FileNotFoundError, ProcessLookupError):
        return False
    return current == identity


def environment(pid):
    values = dict(item.decode().split('=', 1) for item in
                  Path(f'/proc/{pid}/environ').read_bytes().split(b'\0') if item)
    return {k: values[k] for k in (*ENV_KEYS, 'G1_DDS_INTERFACE_ADDRESS') if k in values}


def check_dds_address(env):
    """Reject stale explicit interface bindings before ROS creates its domain."""
    addresses = set()
    if env.get('G1_DDS_INTERFACE_ADDRESS'):
        addresses.add(env['G1_DDS_INTERFACE_ADDRESS'])
    uri = env.get('CYCLONEDDS_URI', '').strip()
    if uri.startswith('<'):
        root = ET.fromstring(uri)
        for element in root.iter():
            if element.tag.rsplit('}', 1)[-1] == 'NetworkInterface':
                address = element.get('address')
                if address and address != 'auto':
                    addresses.add(address)
    if not addresses:
        return
    result = subprocess.run(['ip', '-j', 'address', 'show'], check=True,
                            capture_output=True, text=True, timeout=5)
    interfaces = json.loads(result.stdout)
    assigned = {item['local'] for interface in interfaces
                for item in interface.get('addr_info', [])}
    missing = addresses - assigned
    if missing:
        current = ', '.join(interface['ifname'] + '=' + item['local']
                            for interface in interfaces for item in interface.get('addr_info', [])
                            if item.get('family') == 'inet') or 'none'
        raise RuntimeError('Saved DDS address is not assigned on this WSL host: ' + ', '.join(sorted(missing)) +
                           '. Current IPv4 addresses: ' + current +
                           '. Restore the original robot network before starting/closing a live controller. '
                           'No controller was started or stopped; no interface was changed.')


def silent(gateway):
    return gateway.get('armed') == 'false' and gateway.get('command_publisher_exists') == 'false'


def shutdown_issue(gateway, ui=None):
    if ui and (ui.get('worker_busy') or ui.get('phase') in ('EXECUTING', 'EXECUTION_PREFLIGHT')):
        return 'Task is active; STOP and return to the shutdown position before closing'
    if silent(gateway):
        return None
    if gateway.get('armed') != 'true':
        return 'Controller output state is unknown; controller was retained'
    if gateway.get('phase') not in ('ARMED_HOLD', 'TASK_ADMITTANCE', 'TASK_STOP_HOLD',
                                    'TASK_ADMITTANCE_BLEND', 'TASK_HOLD_BLEND', 'RELEASING_HOLD'):
        return 'Controller is not idle; STOP and return to the shutdown position before closing'
    velocity = float(gateway.get('max_measured_velocity_rad_s', 'nan'))
    if not math.isfinite(velocity) or velocity > .05:
        return 'Robot is still moving; finish returning to the shutdown position before closing'
    return None


def preserve_stack(context):
    if not alive(context.get('launcher_identity')):
        raise RuntimeError('Original application exited or changed; no process was stopped')
    pid = context['launcher_identity']['pid']
    command = argv(pid)
    if str(ROOT/'scripts/run_real_task_application.sh') not in command:
        raise RuntimeError('Unexpected application supervisor; no process was stopped')
    if unit_pid(context['parent_unit']) != pid:
        raise RuntimeError('Application service identity changed; no process was stopped')
    context['stack_command'] = command
    context['stack_environment'] = environment(pid)
    save_context(context)


def stack_command(context, planning_config):
    command = list(context.get('stack_command') or [
        '/bin/bash', str(ROOT/'scripts/run_real_task_application.sh'), 'eth0',
        '--hardware', '--dring-contact-test'])
    for option in ('--planning-config', '--ready-poses'):
        if option in command:
            i = command.index(option)
            del command[i:i+2]
    if context.get('ready_poses'):
        path = ROOT/'data/task_panel/ready_poses.json'
        path.parent.mkdir(parents=True, exist_ok=True)
        temp = path.with_suffix('.tmp')
        temp.write_text(json.dumps(context['ready_poses'], indent=2)+'\n')
        temp.replace(path)
        command += ['--ready-poses', str(path)]
    command += ['--planning-config', str(planning_config.resolve())]
    if '--managed-lifecycle' not in command:
        command.append('--managed-lifecycle')
    return command


def unit_pid(unit):
    result = subprocess.run(['systemctl', '--user', 'show', unit, '-p', 'MainPID', '--value'],
                            check=True, capture_output=True, text=True, timeout=5)
    pid = int(result.stdout.strip())
    if pid <= 0:
        raise RuntimeError('Service did not start: '+unit)
    return pid


class Probe:
    def __init__(self):
        self.node = rclpy.create_node('task_panel_lifecycle')
        self.gateway = {}
        self.ui = {}
        self.gateway_at = self.ui_at = float('-inf')
        self.node.create_subscription(DiagnosticArray, '/g1/arm_gateway/diagnostics', self.diagnostic, 1)
        self.node.create_subscription(String, '/g1/task/console_status', self.console,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def diagnostic(self, msg):
        for item in msg.status:
            self.gateway = {v.key: v.value for v in item.values}
        self.gateway_at = time.monotonic()

    def console(self, msg):
        self.ui = json.loads(msg.data)
        self.ui_at = time.monotonic()

    def wait(self, predicate, timeout=12):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=.05)
            ready = predicate()
            if ready and time.monotonic()-self.gateway_at < 1.:
                return
        raise RuntimeError('Controller acknowledgement timed out; no forced controller termination')

    def trigger(self, name):
        client = self.node.create_client(Trigger, '/g1/arm_gateway/'+name)
        try:
            if not client.wait_for_service(timeout_sec=3):
                raise RuntimeError('Controller service unavailable: '+name)
            future = client.call_async(Trigger.Request())
            self.wait(future.done)
            response = future.result()
            if not response.success:
                raise RuntimeError(response.message)
        finally:
            self.node.destroy_client(client)

    def release(self):
        issue = shutdown_issue(self.gateway)
        if issue:
            raise RuntimeError(issue)
        if not silent(self.gateway):
            requested_at = time.monotonic()
            self.trigger('disarm')
            self.wait(lambda: self.gateway_at > requested_at and silent(self.gateway), timeout=15)


def stop_stack(context, probe):
    checked_at = time.monotonic()
    probe.wait(lambda: probe.gateway_at > checked_at)
    if not silent(probe.gateway):
        raise RuntimeError('Controller resumed output while closing; process retained')
    preserve_stack(context)
    if not alive(context['gateway_identity']):
        raise RuntimeError('Original controller changed while closing; no unrelated service was stopped')
    identities = [context['launcher_identity'], context['gateway_identity']]
    for pid in descendants(context['launcher_identity']['pid']):
        try:
            identities.append(process_identity(pid))
        except (FileNotFoundError, ProcessLookupError):
            pass
    subprocess.run(['systemctl', '--user', 'stop', context['parent_unit']], check=True, timeout=35)
    if any(alive(identity) for identity in identities):
        raise RuntimeError('Some application processes have not exited; no replacement was started')
    context['application_closed'] = True
    context['closed_at'] = datetime.now().isoformat()
    save_context(context)


def close_session(context, probe):
    if not alive(context.get('launcher_identity')):
        if alive(context.get('console_identity')) or alive(context.get('gateway_identity')):
            raise RuntimeError('Application ownership changed; no unrelated process was stopped')
        print('Application already closed: panel, controller and RViz are stopped.')
        return
    preserve_stack(context)
    if not alive(context.get('gateway_identity')):
        raise RuntimeError('Controller status unavailable; application was not forcibly terminated')
    probe.wait(lambda: True)
    if alive(context['console_identity']):
        pid = context['console_identity']['pid']
        probe.wait(lambda: probe.ui.get('console_pid') == pid and time.monotonic()-probe.ui_at < 1.5)
        issue = shutdown_issue(probe.gateway, probe.ui)
        if issue:
            raise RuntimeError(issue)
        refresh_session_context(context, probe.ui['session_dir'])
        save_context(context)
    probe.release()
    if alive(context['console_identity']):
        close_process(context)
    # Wait beyond the GUI's stop-on-exit hook before terminating this controller.
    stop_stack(context, probe)
    print('Application closed: panel, controller, planning services and RViz stopped.')


def start_session(context, probe, planning_config):
    if any(alive(context.get(key)) for key in ('launcher_identity', 'gateway_identity', 'console_identity')):
        raise RuntimeError('Application is already running; close it with stop_task_panel.sh before a new start')
    lock = acquire_console_lock()
    lock.close()
    command = stack_command(context, planning_config)
    env = dict(context.get('stack_environment') or context.get('environment') or
               {k: os.environ[k] for k in ENV_KEYS if k in os.environ})
    stamp = datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    unit = 'g1-task-application-'+stamp+'.service'
    subprocess.run(['systemd-run', '--user', '--collect', '--unit='+unit,
                    '--property=TimeoutStopSec=20s',
                    *['--setenv='+k+'='+v for k, v in env.items()], '--', *command], check=True)
    context.update(parent_unit=unit, launcher_identity=process_identity(unit_pid(unit)),
                   gateway_identity=None, console_identity=None, console_unit=None,
                   stack_command=command, stack_environment=env, application_closed=False)
    save_context(context)

    def initialized():
        if not alive(context['launcher_identity']):
            raise RuntimeError('Application startup failed; inspect journalctl --user -u '+unit)
        before = (context['gateway_identity'], context['console_identity'])
        pending = None in before
        for pid in descendants(context['launcher_identity']['pid']) if pending else ():
            try:
                process_args = argv(pid)
                if not process_args:
                    continue
                if Path(process_args[0]).name == 'arm_sdk_hardware_gateway':
                    context['gateway_identity'] = process_identity(pid)
                elif str(ROOT/'scripts/real_task_app.py') in process_args:
                    context.update(console_identity=process_identity(pid), console_command=process_args,
                                   environment=environment(pid))
            except (FileNotFoundError, ProcessLookupError):
                continue
        if before != (context['gateway_identity'], context['console_identity']):
            save_context(context)
        return (context['gateway_identity'] is not None and context['console_identity'] is not None and
                probe.ui.get('console_pid') == context['console_identity']['pid'] and
                time.monotonic()-probe.ui_at < 1.5 and not probe.ui.get('worker_busy'))

    probe.wait(initialized, timeout=75)
    if not silent(probe.gateway):
        raise RuntimeError('New session unexpectedly armed; no automatic restart or reset attempted')
    refresh_session_context(context, probe.ui['session_dir'])
    context['session_home'] = None
    save_context(context)
    print('New panel, controller and RViz started. Confirm R2+A/startup, then ARM explicitly. No motion sent.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('start', 'close'))
    parser.add_argument('--planning-config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--result-file', type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args()
    result = dict(success=False, message='Application operation did not finish')
    try:
        if args.action == 'start':
            load_parameters(args.planning_config)
        context = json.loads(CONTEXT.read_text()) if CONTEXT.exists() else {}
        running = any(alive(context.get(key)) for key in
                      ('launcher_identity', 'gateway_identity', 'console_identity'))
        if args.action == 'close' and not running:
            close_session(context, None)
            result = dict(success=True, message='Application already closed; no ROS connection needed')
            return
        if args.action == 'start' and running:
            raise RuntimeError('Application is already running; close it with stop_task_panel.sh before a new start')
        os.environ.update(context.get('environment', {}))
        check_dds_address(os.environ)
        rclpy.init()
        probe = None
        try:
            probe = Probe()
            if args.action == 'close':
                close_session(context, probe)
            else:
                start_session(context, probe, args.planning_config)
        finally:
            if probe is not None:
                probe.node.destroy_node()
            rclpy.shutdown()
        result = dict(success=True, message='Application '+args.action+' completed')
    except Exception as error:
        result['message'] = str(error)
        raise
    finally:
        if args.result_file:
            temp = args.result_file.with_suffix('.tmp')
            temp.write_text(json.dumps(result)+'\n')
            temp.replace(args.result_file)


if __name__ == '__main__':
    main()
