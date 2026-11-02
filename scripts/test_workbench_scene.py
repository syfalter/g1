"""Construct launch actions without starting ROS, RViz, or any hardware client."""
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument

from workbench_app import startup_joints, INDEX_OPEN

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    'workbench_application_launch', ROOT / 'scripts/workbench_application.launch.py')
application = importlib.util.module_from_spec(spec)
spec.loader.exec_module(application)


class SceneTests(unittest.TestCase):
    def setUp(self):
        self.context = LaunchContext()
        for action in application.generate_launch_description().entities:
            if isinstance(action, DeclareLaunchArgument):
                action.execute(self.context)
        self.profile = json.loads((ROOT / 'assets/workbench/task_start_pose.json').read_text())

    def scene(self):
        with patch.object(Path, 'write_text', autospec=True) as write, contextlib.redirect_stdout(io.StringIO()):
            actions = application.setup(self.context)
        self.assertEqual(len(actions), 4)
        descriptions = {call.args[0].name: call.args[1] for call in write.call_args_list}
        return ET.fromstring(descriptions['robot.urdf'])

    def test_measured_profile_without_legacy_source_fields(self):
        self.assertNotIn('right_arm_source', self.profile)
        self.assertNotIn('left_arm_source', self.profile)
        robot = self.scene()
        names = [j.get('name') for j in robot.findall('joint')
                 if j.get('type') not in ('fixed', 'floating')]
        initial = startup_joints(names, self.profile)
        for name in set(names).intersection(self.profile['measured_joints']):
            self.assertEqual(initial[name], self.profile['joints'][name])
        for side in ('left', 'right'):
            self.assertEqual(initial[side + '_index_1_joint'], INDEX_OPEN)
            self.assertAlmostEqual(initial[side + '_index_2_joint'], 0.)
            for kind in ('visual', 'collision'):
                mesh = robot.find(f"link[@name='{side}_rubber_hand']/{kind}/geometry/mesh")
                expected = ROOT / f'assets/workbench/rh56dfx/task_open/{side}_body_{kind}.stl'
                self.assertEqual(mesh.get('filename'), expected.as_uri())
                self.assertTrue(expected.is_file())

    def test_legacy_folded_profile_uses_matching_meshes(self):
        self.profile['hand_startup_posture'] = 'folded'
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'initial.json'
            path.write_text(json.dumps(self.profile))
            self.context.launch_configurations['initial_pose_file'] = str(path)
            robot = self.scene()
        for side in ('left', 'right'):
            mesh = robot.find(f"link[@name='{side}_rubber_hand']/collision/geometry/mesh")
            self.assertIn(f'/rh56dfx/task/{side}_body_collision.stl', mesh.get('filename'))

    def test_invalid_posture_rejected_before_snapshot(self):
        self.profile['hand_startup_posture'] = 'unknown'
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'initial.json'
            path.write_text(json.dumps(self.profile))
            self.context.launch_configurations['initial_pose_file'] = str(path)
            with self.assertRaisesRegex(ValueError, 'Unknown startup hand posture'):
                self.scene()

    def test_platform_raises_only_robot(self):
        self.context.launch_configurations['platform_height'] = '0'
        ground = self.scene()
        self.context.launch_configurations['platform_height'] = '0.07'
        raised = self.scene()
        for name in ('preview_world_pelvis', 'preview_world_workbench', 'task_handle_angle'):
            a = list(map(float, ground.find(f"joint[@name='{name}']/origin").get('xyz').split()))
            b = list(map(float, raised.find(f"joint[@name='{name}']/origin").get('xyz').split()))
            self.assertEqual(a[:2], b[:2])
            self.assertAlmostEqual(b[2] - a[2], .07 if name == 'preview_world_pelvis' else 0.)

    def test_invalid_platform_height_rejected(self):
        for value in ('-0.01', '0.31', 'nan'):
            self.context.launch_configurations['platform_height'] = value
            with self.assertRaisesRegex(ValueError, 'Platform height'):
                self.scene()


if __name__ == '__main__':
    unittest.main()
