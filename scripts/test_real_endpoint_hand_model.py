"""Construct real launch parameters without starting nodes or motor output."""
import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

from launch import LaunchContext

ROOT = Path(__file__).resolve().parents[1]


class RealEndpointHandModelTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location('real_hand_launch', ROOT /
            'src/g1_upper_body_moveit_config/launch/real_shadow_moveit.launch.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        context = LaunchContext()
        context.launch_configurations.update(active_arm='right',
            enable_rviz_interactive_execution='false')
        with patch.object(module, 'Node', side_effect=lambda **kw: kw):
            cls.nodes = module._setup(context)
        cls.planner = next(n for n in cls.nodes if n.get('executable') == 'single_arm_planner_server')
        cls.params = cls.planner['parameters'][0]
        cls.robot = ET.fromstring(cls.params['robot_description'])

    def test_real_model_has_bilateral_rh56_and_short_cylinders(self):
        for side in ('left', 'right'):
            cylinder = self.robot.find(f"link[@name='{side}_payload_collision']/collision/geometry/cylinder")
            self.assertEqual(float(cylinder.get('length')), .04)
            self.assertEqual(float(cylinder.get('radius')), .03)
            hand = self.robot.find(f"link[@name='{side}_rubber_hand']")
            for kind in ('visual', 'collision'):
                uri = hand.find(f'{kind}/geometry/mesh').get('filename')
                self.assertTrue(uri.endswith(f'rh56dfx/{side}_{kind}.stl'))
                self.assertTrue(Path(uri.removeprefix('file://')).is_file())
            self.assertIsNone(self.robot.find(f"link[@name='{side}_hand_collision']/collision"))
            mount = self.robot.find(f"joint[@name='{side}_hand_palm_joint']/origin")
            self.assertAlmostEqual(float(mount.get('xyz').split()[0]), .0815)

    def test_display_and_planner_share_description(self):
        for executable in ('move_group', 'robot_state_publisher', 'rviz2'):
            node = next(n for n in self.nodes if n.get('executable') == executable)
            self.assertEqual(node['parameters'][0]['robot_description'], self.params['robot_description'])

    def test_only_fixed_mount_collision_exclusions_added(self):
        semantic = ET.fromstring(self.params['robot_description_semantic'])
        pairs = {frozenset((p.get('link1'), p.get('link2'))) for p in semantic.findall('disable_collisions')}
        for side in ('left', 'right'):
            self.assertIn(frozenset((f'{side}_payload_collision', f'{side}_wrist_yaw_link')), pairs)
            self.assertNotIn(frozenset((f'{side}_payload_collision', 'torso_link')), pairs)


if __name__ == '__main__':
    unittest.main()
