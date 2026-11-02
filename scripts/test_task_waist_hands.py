"""Articulated model, frozen target and eight-axis ownership checks, offline."""
import importlib.util
import json
import copy
from pathlib import Path
from types import SimpleNamespace as S
import tempfile
import unittest
from unittest.mock import Mock, patch
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from launch import LaunchContext
from estimate_index_tcp import generate
from task_hand_control import model_joints, POSTURES
from task_hand_model import articulated_hand_pairs, require_task_hand_model
from task_kinematics import capture_transform, arm_names, WAIST, mirror_left_ready
from task_motion_planner import TaskPlanner, PlanningFailure
from task_session import ARM_NAMES, ARMS, Ticket
from task_ik_search import preinsert_seeds
from task_planning_config import DEFAULT_CONFIG,load_parameters
from workbench_model import OfflineModel

ROOT=Path(__file__).resolve().parents[1]


class ModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec=importlib.util.spec_from_file_location('new_task_launch',ROOT/'src/g1_upper_body_moveit_config/launch/real_shadow_moveit.launch.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        context=LaunchContext()
        context.launch_configurations.update(active_arm='left',task_application_enabled='true',
            task_articulated_hands='true',task_waist_enabled='true',
            task_planning_config=str(DEFAULT_CONFIG),
            enable_rviz_interactive_execution='false',rviz_config='offline.rviz')
        with patch.object(module,'Node',side_effect=lambda **kw:kw):cls.nodes=module._setup(context)
        cls.params=next(n for n in cls.nodes if n.get('executable')=='move_group')['parameters'][0]
        cls.directory=tempfile.TemporaryDirectory()
        path=Path(cls.directory.name)/'robot.urdf';path.write_text(cls.params['robot_description'])
        cls.model=OfflineModel(path)

    @classmethod
    def tearDownClass(cls):cls.directory.cleanup()

    def test_task_tcp_unchanged_on_both_hands(self):
        require_task_hand_model(self.model)
        for side in ARMS:
            q=model_joints(side,POSTURES['index_only'])
            actual=np.linalg.inv(self.model.transform(side+'_wrist_yaw_link',q))@self.model.transform(side+'_index_tcp',q)
            expected=generate()['sides'][side]['states']['open']['wrist_yaw_from_index_tcp']['matrix_row_major']
            np.testing.assert_allclose(actual,expected,atol=1e-10)
            two_finger=model_joints(side,POSTURES['dring_two_finger'])
            updated=np.linalg.inv(self.model.transform(side+'_wrist_yaw_link',two_finger))@self.model.transform(side+'_index_tcp',two_finger)
            self.assertFalse(np.allclose(updated,actual,atol=1e-10))
            self.assertEqual(POSTURES['dring_two_finger'][2:4],(.8,.8))
            for finger in ('thumb_1','thumb_2'):
                name=f'{side}_{finger}_joint'
                self.assertEqual(two_finger[name],model_joints(side,POSTURES['open'])[name])
            self.assertEqual(self.model.robot.find(f"joint[@name='{side}_middle_1_joint']").get('type'),'revolute')

    def test_dring_keeps_straight_calibrated_tcp_without_bend_compensation(self):
        from task_straight_policy import StraightIndexPolicy
        from task_calibration import DEFAULTS,snapshot
        p=StraightIndexPolicy();p.model=self.model;p.search_diagnostics={};p.max_index_tilt=45.
        for side in ARMS:
            _,tcp=p.contact_hand_setup(side,{},dict(calibration=snapshot(DEFAULTS)))
            straight=model_joints(side,POSTURES['index_only'])
            expected=np.linalg.inv(self.model.transform(side+'_wrist_yaw_link',straight))@self.model.transform(side+'_index_tcp',straight)
            expected[:3,3]=[DEFAULTS[f'{side}_{axis}_mm']/1000 for axis in 'xyz']
            np.testing.assert_allclose(tcp,expected,atol=1e-10)
            self.assertEqual(p.search_diagnostics['straight_index']['tcp_posture_reference'],
                             'straight_index_no_bend_compensation')

    def test_right_preparation_mirrors_saved_left_without_copying_other_joints(self):
        from real_task_app import LEFT_READY
        left=json.loads(LEFT_READY.read_text());before=copy.deepcopy(left)
        right=mirror_left_ready(self.model,left)
        self.assertEqual(left,before)
        self.assertEqual(set(right['joints']),set(ARMS['right'])|{WAIST})
        self.assertAlmostEqual(right['joints'][WAIST],-left['joints'][WAIST])
        self.assertFalse(right['hardware_execution_validated'])
        M=np.diag([1.,-1.,1.,1.])
        expected=M@self.model.transform('left_wrist_yaw_link',left['joints'])@M
        actual=self.model.transform('right_wrist_yaw_link',dict(left['joints'],**right['joints']))
        np.testing.assert_allclose(actual,expected,atol=2e-5,rtol=0)
        left['joints']['left_wrist_roll_joint']=10.
        with self.assertRaisesRegex(ValueError,'guarded joint range'):mirror_left_ready(self.model,left)

    def test_launcher_passes_validated_operator_integral_settings(self):
        gateway=next(n for n in self.nodes if n.get('executable')=='arm_sdk_hardware_gateway')
        expected=load_parameters()[0].arm_integral_parameters('left')
        for key,value in expected.items():self.assertEqual(gateway['parameters'][1][key],value)

    def test_original_arm_groups_unchanged_and_waist_separate(self):
        semantic=ET.fromstring(self.params['robot_description_semantic'])
        for side in ARMS:
            self.assertEqual(semantic.find(f"group[@name='{side}_arm']/chain").get('base_link'),'torso_link')
            self.assertEqual(semantic.find(f"group[@name='{side}_arm_with_waist']/chain").get('base_link'),'pelvis')
        limit=self.model.robot.find("joint[@name='waist_yaw_joint']/limit")
        self.assertAlmostEqual(float(limit.get('upper')),np.deg2rad(45))

    def test_finger_collision_not_blanket_disabled(self):
        pairs={frozenset(pair) for pair in articulated_hand_pairs(self.model.robot)}
        self.assertNotIn(frozenset(('left_index_2','left_middle_2')),pairs)
        self.assertNotIn(frozenset(('left_index_2','right_index_2')),pairs)

    def test_capture_transform_immutable_after_waist_changes(self):
        q={WAIST:.2};frozen=self.model.transform('torso_link',q)
        geometry={'capture_frame':{'pelvis_from_torso_row_major':frozen.tolist()}}
        np.testing.assert_allclose(capture_transform(geometry,self.model,{WAIST:-.3}),frozen)
        self.assertFalse(np.allclose(frozen,self.model.transform('torso_link',{WAIST:-.3})))

    def test_other_hand_world_pose_changes_when_waist_turns(self):
        q=dict.fromkeys(ARM_NAMES,0.);q[WAIST]=0.
        initial=self.model.transform('right_wrist_yaw_link',q)
        q[WAIST]=.3
        self.assertFalse(np.allclose(initial,self.model.transform('right_wrist_yaw_link',q)))


class PlannerTests(unittest.TestCase):
    def setUp(self):
        self.p=TaskPlanner.__new__(TaskPlanner)
        self.p.waist_assist=True;self.p.reference_frame='pelvis';self.p.articulated_hands=True
        self.q=dict.fromkeys(ARM_NAMES,.1);self.q[WAIST]=.2
        for side in ARMS:self.q.update(model_joints(side,POSTURES['open']))
        self.ticket=Ticket(1,'task','left',.15,.15)

    def test_eight_axis_ik_preserves_opposite_arm_and_fingers(self):
        names=arm_names('left',True)
        self.p.ik=object();self.p.call=Mock(return_value=S(error_code=S(val=1),
            solution=S(joint_state=S(name=names,position=[.3]*8))))
        self.p.validate=Mock()
        answer=self.p.solve('left',self.q,np.eye(4),np.eye(4),self.ticket)
        self.assertEqual(answer[WAIST],.3)
        self.assertEqual(self.p.call.call_args.args[1].ik_request.pose_stamped.header.frame_id,'pelvis')
        self.assertEqual(self.p.call.call_args.args[1].ik_request.group_name,'left_arm_with_waist')
        for name in self.q:
            if name not in names:self.assertEqual(answer[name],self.q[name])

    def test_hand_transition_collision_checked_and_arm_stationary(self):
        self.p.validate=Mock()
        result=self.p.hand_transition('left',self.q,'index_only',self.ticket)
        self.assertEqual(self.p.validate.call_count,81)
        self.assertTrue(all(c.args[3]=='hand_transition' for c in self.p.validate.call_args_list))
        for name in ARM_NAMES+(WAIST,):self.assertEqual(result['end'][name],self.q[name])
        for name in model_joints('right',POSTURES['open']):self.assertEqual(result['end'][name],self.q[name])
        self.assertEqual(result['target_q'],list(POSTURES['index_only']))

    def test_hand_transition_does_not_allow_contact(self):
        self.p.validate=Mock(side_effect=PlanningFailure('COLLISION','hand_transition','palm/ring'))
        with self.assertRaises(PlanningFailure):self.p.hand_transition('left',self.q,'index_only',self.ticket)

    def test_seed_search_includes_bounded_waist(self):
        limits={name:(-.61,.61) if name==WAIST else (-2.,2.) for name in self.p.names('left')}
        seeds=preinsert_seeds('left',self.q,limits)
        self.assertGreater(len({q[WAIST] for q in seeds}),1)
        for q in seeds:
            self.assertTrue(-.58<=q[WAIST]<=.58)
            for name in ARMS['right']:self.assertEqual(q[name],self.q[name])

    def test_arm_only_goal_does_not_teleport_other_arm_waist_or_fingers(self):
        self.p.waist_assist=False;self.p.motion=object()
        self.p.validate=Mock();self.p.progress=Mock();self.p.retime=Mock()
        self.p.call=Mock(return_value=S(motion_plan_response=S(error_code=S(val=1),trajectory=object())))
        target=dict.fromkeys(self.q,.3)
        self.p.joint_path('left',self.q,target,self.ticket,'recover')
        checked=self.p.validate.call_args_list[1].args[0]
        for name in self.q:
            self.assertEqual(checked[name],.3 if name in ARMS['left'] else self.q[name])


if __name__=='__main__':unittest.main()
