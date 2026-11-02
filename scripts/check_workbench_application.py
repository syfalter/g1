"""Exercise only the isolated offline application command interface."""
import argparse
import json
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--arm', choices=('left', 'right'), default='left')
    parser.add_argument('--scope', choices=('entry', 'full'), default='full')
    parser.add_argument('--play', action='store_true')
    parser.add_argument('--timeout', type=float, default=180)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '77' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('This check requires isolated offline domain 77, localhost only')
    rclpy.init()
    node = Node('workbench_application_check')
    publisher = node.create_publisher(String, '/workbench/request', 10)
    latest = {}
    node.create_subscription(String, '/workbench/status', lambda msg: latest.update(json.loads(msg.data)), 10)

    def wait(predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if predicate():
                return
        raise TimeoutError(json.dumps(latest))

    def send(action, **kwargs):
        publisher.publish(String(data=json.dumps(dict(action=action, **kwargs))))

    try:
        wait(lambda: latest.get('mode') == 'OFFLINE' and publisher.get_subscription_count() > 0, 10)
        if latest.get('busy') or latest.get('playing'):
            raise RuntimeError('Offline application busy; not interrupting user action')
        send('reset')
        wait(lambda: latest.get('phase') == 'IDLE' and not latest.get('plan_ready'), 5)
        send('plan', arm=args.arm, scope=args.scope)
        wait(lambda: latest.get('phase') in ('PLANNING', 'VALIDATING', 'PLAN_READY', 'PLAN_FAILED'), 5)
        wait(lambda: latest.get('phase') in ('PLAN_READY', 'PLAN_FAILED') and not latest.get('busy'), args.timeout)
        print(json.dumps(latest, ensure_ascii=False), flush=True)
        if latest['phase'] != 'PLAN_READY':
            raise RuntimeError(latest['message'])
        if args.play:
            send('play')
            wait(lambda: latest.get('playing') or latest.get('phase') == 'COMPLETE', 5)
            wait(lambda: latest.get('phase') == 'COMPLETE', args.timeout)
            print(json.dumps(latest, ensure_ascii=False), flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
