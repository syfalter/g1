"""Model/marker checks without ROS initialization or robot access."""
import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import numpy as np
from launch import LaunchContext
from visualization_msgs.msg import Marker
from estimate_index_tcp import generate
from task_hand_model import CONTACT_PARTS, fixed_hand_pairs, require_task_hand_model
from task_preview_markers import target_markers
from workbench_model import OfflineModel

ROOT=Path(__file__).resolve().parents[1]


class TaskModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec=importlib.util.spec_from_file_location('task_launch',ROOT/'src/g1_upper_body_moveit_config/launch/real_shadow_moveit.launch.py')
        module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
        context=LaunchContext()
        context.launch_configurations.update(active_arm='left',task_application_enabled='true',
                                              enable_rviz_interactive_execution='false',rviz_config='offline-test.rviz')
        with patch.object(module,'Node',side_effect=lambda **kw:kw):cls.nodes=module._setup(context)
        cls.params=next(n for n in cls.nodes if n.get('executable')=='move_group')['parameters'][0]
        cls.robot=ET.fromstring(cls.params['robot_description'])

    def test_split_task_geometry_and_unchanged_tcp(self):
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'robot.urdf'; path.write_text(self.params['robot_description'])
            model=OfflineModel(path); require_task_hand_model(model)
            for side in ('left','right'):
                actual=np.linalg.inv(model.transform(side+'_wrist_yaw_link',{}))@model.transform(side+'_index_tcp',{})
                expected=generate()['sides'][side]['states']['open']['wrist_yaw_from_index_tcp']['matrix_row_major']
                np.testing.assert_allclose(actual,expected,atol=1e-10)
                for part in CONTACT_PARTS:self.assertIsNotNone(self.robot.find(f"link[@name='{side}_{part}']"))
                for j in self.robot.findall('joint'):
                    if j.get('name').startswith(side+'_index_'):self.assertEqual(j.get('type'),'fixed')

    def test_exemptions_are_only_within_rigid_hand(self):
        for a,b in fixed_hand_pairs(self.robot):
            self.assertEqual(a.split('_')[0],b.split('_')[0])
            self.assertFalse(any(x in a+b for x in ('wrist','torso','payload','table')))

    def test_all_nodes_share_model(self):
        for executable in ('robot_state_publisher','rviz2','task_state_validator','single_arm_planner_server'):
            node=next(n for n in self.nodes if n.get('executable')==executable)
            self.assertEqual(node['parameters'][0]['robot_description'],self.params['robot_description'])


class TargetMarkerTests(unittest.TestCase):
    def test_no_target_hands_during_prepare_recover(self):
        for kind in ('prepare','recover'):
            result=target_markers({},np.eye(4),'both',kind)
            self.assertEqual(len(result.markers),1)
            self.assertEqual(result.markers[0].action,Marker.DELETEALL)

    def test_task_has_one_coordinate_frame_and_no_mesh(self):
        geometry={'task_preview':{'axis_center_m':[0,0,0],'insertion_reference_m':[1,0,0],
                                  'arc_minus120_to_plus120_m':[[1,0,0],[0,1,0]]},
                  'grasp_candidates':{'left':{'poses':{'index_tcp':{'matrix_row_major':np.eye(4).tolist()}}}}}
        result=target_markers(geometry,np.eye(4),'left','task')
        self.assertEqual(sum(m.type==Marker.ARROW and m.action!=Marker.DELETEALL for m in result.markers),3)
        self.assertFalse(any(m.type==Marker.MESH_RESOURCE for m in result.markers))
        with self.assertRaises(ValueError):target_markers(geometry,np.eye(4),'both','task')


if __name__=='__main__':unittest.main()
