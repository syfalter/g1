"""Exercise the ARM64 gateway startup with hardware output disabled and isolated DDS."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('binary', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--assets', type=Path, required=True)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '174' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Requires localhost-only isolated domain 174')
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    rclpy.init()
    node = rclpy.create_node('rail_offline_smoke')
    process = None
    log = None
    diagnostics = {}
    try:
        deadline = time.monotonic() + 1.
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        if any(name != 'rail_offline_smoke' for name in node.get_node_names()):
            raise RuntimeError('Offline domain already occupied')
        args.output.mkdir(parents=True, exist_ok=True)
        log = (args.output / 'gateway.log').open('w')
        process = subprocess.Popen([str(args.binary), '--ros-args',
            '-p', 'hardware_output_enabled:=false', '-p', 'gravity_feedforward_enabled:=true',
            '-p', 'gravity_urdf_path:=' + str(args.assets / 'g1_29dof_lock_waist_rev_1_0.urdf'),
            '-p', 'right_payload_fit_path:=' + str(args.assets / 'right_arm_camera_fit_20260824_185416.yaml'),
            '-p', 'left_payload_fit_path:=' + str(args.assets / 'left_arm_right_copy_seed_20260908.yaml'),
            '-p', 'service_check_enabled:=false', '-p', 'task_application_enabled:=true',
            '-p', 'task_waist_enabled:=true', '-p', 'state_topic:=/rail_offline/lowstate',
            '-p', 'command_topic:=/rail_offline/disabled_command'], stdout=log, stderr=subprocess.STDOUT)

        def received(message):
            for status in message.status:
                diagnostics.update({item.key: item.value for item in status.values})

        node.create_subscription(DiagnosticArray, '/g1/arm_gateway/diagnostics', received, 10)
        deadline = time.monotonic() + 15.
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if process.poll() is not None:
                raise RuntimeError('Gateway exited; inspect gateway.log')
            if any(node.count_publishers(name) for name in
                   ('/arm_sdk', '/rt/arm_sdk', '/rail_offline/disabled_command')):
                raise RuntimeError('Unexpected command publisher in dry run')
            if diagnostics.get('hardware_output_available') == 'false':
                report = dict(passed=True, hardware_output_available=False,
                              hardware_command_publishers=0, diagnostics=diagnostics,
                              scope='isolated process startup only; no arm, prepare, or motion request')
                (args.output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
                print(json.dumps({k:v for k,v in report.items() if k != 'diagnostics'}))
                return
        raise TimeoutError('No disabled-output diagnostics')
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try: process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill(); process.wait()
        if log is not None: log.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
