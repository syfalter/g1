"""Offline service-only IK diagnostic; never publishes robot state or commands."""
import argparse
from collections import Counter
import json
import math
import threading
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from moveit_msgs.srv import GetPositionIK, GetStateValidity, GetPlanningScene, ApplyPlanningScene
from workbench_app import WorkbenchApp, ASSETS, finger_pose, INDEX_OPEN
from workbench_model import OfflineModel
from workbench_task_geometry import insertion_orientation


class Probe(Node):
    call = WorkbenchApp.call
    state = WorkbenchApp.state
    pose = WorkbenchApp.pose
    solve = WorkbenchApp.solve
    validate = WorkbenchApp.validate
    contact_policy = WorkbenchApp.contact_policy
    arm_joint = staticmethod(WorkbenchApp.arm_joint)

    def __init__(self):
        super().__init__('workbench_ik_probe')
        self.cancel_event = threading.Event()
        robot = ET.parse(ASSETS / 'task/robot.urdf').getroot()
        self.names = [j.get('name') for j in robot.findall('joint') if j.get('type') not in ('fixed', 'floating')]
        initial = json.loads((ASSETS / 'task_start_pose.json').read_text())['joints']
        self.home = {n: float(initial.get(n, 0)) for n in self.names}
        self.model=OfflineModel(ASSETS / 'task/robot.urdf')
        for side in ('left','right'):self.home=finger_pose(self.home,side,INDEX_OPEN)
        self.ik = self.create_client(GetPositionIK, '/compute_ik')
        self.valid = self.create_client(GetStateValidity, '/check_state_validity')
        self.scene = self.create_client(GetPlanningScene, '/get_planning_scene')
        self.apply = self.create_client(ApplyPlanningScene, '/apply_planning_scene')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--arm', choices=('left', 'right'), default='left')
    args = parser.parse_args()
    rclpy.init()
    node = Probe()
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    target = json.loads((ASSETS / 'task/target.json').read_text())
    failures = Counter()
    accepted = []
    try:
        node.contact_policy(args.arm, True)
        for tilt in (0, 20, 35, 50, 65):
            for yaw in range(0, 360, 30):
                for roll in (0, 90, 180, 270):
                    quat = (Rotation.from_quat(insertion_orientation(target, math.radians(yaw), math.radians(tilt))) * Rotation.from_euler('z', roll, degrees=True)).as_quat()
                    q = node.solve(args.arm, target['entry_center'], quat, node.home, avoid=False)
                    if q is None:
                        failures['unreachable'] += 1
                        continue
                    try:
                        node.validate(q)
                        accepted.append(dict(tilt=tilt, yaw=yaw, roll=roll, quat=quat.tolist(), q=q))
                        print('VALID', tilt, yaw, roll, flush=True)
                    except RuntimeError as error:
                        failures[str(error)] += 1
        print('RESULT', len(accepted), failures, flush=True)
        (ASSETS / f'task/{args.arm}_ik_probe.json').write_text(json.dumps(dict(accepted=accepted, failures=dict(failures)), indent=2)+'\n')
    finally:
        node.contact_policy(args.arm, False)
        rclpy.shutdown()
        thread.join(timeout=2)


if __name__ == '__main__':
    main()
