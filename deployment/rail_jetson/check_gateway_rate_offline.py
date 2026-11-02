"""Start an output-disabled gateway on isolated localhost DDS and inspect limits."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '174' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Offline smoke check requires localhost DDS domain 174')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', required=True)
    parser.add_argument('--config', required=True)
    args = parser.parse_args()
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    config = json.loads(Path(args.config).read_text())
    parameters = config['g1_arm_sdk_hardware_gateway']['ros__parameters']
    parameters.update(hardware_output_enabled=False, hardware_authorization='', service_check_enabled=False,
                      waist_command_rate_hz=100., task_waist_velocity_limit_rad_s=.35)
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / 'offline.json'
        path.write_text(json.dumps(config))
        rclpy.init()
        node = rclpy.create_node('waist_limit_offline_probe')
        received = {}
        def diagnostic(message):
            for status in message.status:
                received.update({v.key: v.value for v in status.values})
        node.create_subscription(DiagnosticArray, '/g1/arm_gateway/diagnostics', diagnostic, 10)
        log = (Path(directory) / 'gateway.log').open('w+')
        process = subprocess.Popen([args.binary, '--ros-args', '--params-file', str(path)],
                                   stdout=log, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 15.
            while time.monotonic() < deadline and not received:
                if process.poll() is not None:
                    raise RuntimeError('Offline gateway exited')
                rclpy.spin_once(node, timeout_sec=.1)
            assert received.get('hardware_output_available') == 'false', received
            assert received.get('armed') == 'false', received
            assert received.get('command_publisher_exists') == 'false', received
            assert float(received['waist_command_rate_hz']) == 100.
            assert float(received['command_waist_velocity_limit_rad_s']) == .35
            print(json.dumps({k: received.get(k) for k in (
                'hardware_output_available', 'armed', 'command_publisher_exists',
                'waist_command_rate_hz', 'command_waist_velocity_limit_rad_s')}))
        finally:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=10.)
            log.seek(0)
            if not received:
                print(log.read())
            log.close()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
