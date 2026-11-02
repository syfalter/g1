"""Bilateral offline integration check with a private scene and no execution."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
from unittest.mock import patch

import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
import rclpy

from workbench_app import WorkbenchApp, ROOT


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--timeout', type=float, default=120.)
    parser.add_argument('--stance-x', type=float, default=0.)
    parser.add_argument('--stance-y', type=float, default=0.)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '83' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Integration check requires isolated localhost domain 83')
    directory = args.output.resolve()
    directory.mkdir(parents=True, exist_ok=False)
    scene = directory/'scene';scene.mkdir()
    plans = directory/'plans';plans.mkdir()
    spec = importlib.util.spec_from_file_location('offline_launch', ROOT/'scripts/workbench_application.launch.py')
    application = importlib.util.module_from_spec(spec);spec.loader.exec_module(application)
    context = LaunchContext()
    for action in application.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    # Capture launch parameters without launching the GUI or overwriting shared assets.
    with patch.object(application, 'Node', side_effect=lambda **kw: kw), patch.object(Path, 'write_text', return_value=0):
        actions = application.setup(context)
    parameters = actions[1]['parameters'][0]
    assert parameters['allow_trajectory_execution'] is False
    (scene/'move_group.yaml').write_text(yaml.safe_dump({'/move_group': {'ros__parameters': parameters}}))
    (scene/'robot.urdf').write_text(parameters['robot_description'])
    (scene/'robot.srdf').write_text(parameters['robot_description_semantic'])
    for source, name in ((ROOT/'assets/workbench/task/target.json', 'target.json'),
                         (ROOT/'assets/workbench/task_start_pose.json', 'initial_pose.json')):
        (scene/name).write_bytes(source.read_bytes())
    server = None;node = None;thread = None
    reports = []
    try:
        with (directory/'move_group.log').open('w') as log:
            server = subprocess.Popen(
                ['ros2', 'launch', str(ROOT/'scripts/workbench_robustness.launch.py'),
                 f'run_directory:={directory}'], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        rclpy.init()
        node = WorkbenchApp(scene_dir=scene, plan_dir=plans)
        node.command(dict(action='stance', x=args.stance_x, y=args.stance_y))
        thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True);thread.start()
        if not node.scene.wait_for_service(timeout_sec=30):
            raise RuntimeError('Offline MoveIt did not become ready')
        for side in ('left', 'right'):
            node.command(dict(action='reset'))
            start = time.monotonic()
            node.command(dict(action='plan', arm=side, scope='full'))
            cancelled = False
            while node.busy:
                elapsed = time.monotonic()-start
                if elapsed > args.timeout and not cancelled:
                    node.cancel_event.set();cancelled = True
                if elapsed > args.timeout+30:
                    raise RuntimeError('Offline planner did not respond to cancellation')
                time.sleep(.05)
            reports.append(dict(arm=side, plan_ready=bool(node.plan), cancelled=cancelled,
                                seconds=time.monotonic()-start, status=dict(node.status),
                                validation=node.last_report['validation'] if node.last_report else None,
                                candidate_attempts=node.candidate_attempts,
                                candidate_failures=node.candidate_failures))
            (directory/'report.json').write_text(json.dumps(dict(offline_only=True,
                test_stance_offset_m=[args.stance_x,args.stance_y], arms=reports), indent=2)+'\n')
            print(json.dumps(reports[-1]), flush=True)
    finally:
        if node is not None:
            node.cancel_event.set()
            if rclpy.ok():rclpy.shutdown()
            if thread is not None:thread.join(timeout=5)
            node.destroy_node()
        if server is not None:
            os.killpg(server.pid, signal.SIGINT)
            try:server.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(server.pid, signal.SIGTERM);server.wait(timeout=10)
    if not all(report['plan_ready'] for report in reports):
        raise SystemExit(2)


if __name__ == '__main__':
    main()
