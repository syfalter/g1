import copy
import json
from pathlib import Path
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock,patch
import xml.etree.ElementTree as ET

from task_dring_sequence import SCHEMA
from task_path_smoothing import joint_line_trajectory
from task_saved_test import range_fingerprint,verify_test_plan
from task_saved_transition import plan_transition,attach_transition,start_differences
from task_session import ARMS,ARM_NAMES
from task_trajectory import time_seconds


class TransitionTests(unittest.TestCase):
    def setUp(self):
        self.start=dict.fromkeys(ARM_NAMES,0.)
        self.start['waist_yaw_joint']=0.
        self.target=dict(self.start)
        self.target['left_elbow_joint']=.4
        self.model=S(robot=ET.fromstring('<robot>'+''.join(
            f'<joint name="{name}"><limit lower="-3" upper="3" velocity="1"/></joint>'
            for name in self.start)+'</robot>'),limits={name:(-3.,3.) for name in self.start})
        self.ticket=S(velocity=.3,acceleration=.5)
        self.planner=S(waist_assist=True,joint_path=Mock(side_effect=self.path))
        self.scene=Mock();self.check=Mock()

    def path(self,side,q,target,ticket,label,stage=None):
        names=list(ARMS[side])+['waist_yaw_joint']
        end=dict(q);end.update({name:target[name] for name in names})
        trajectory=joint_line_trajectory(names,[q[n] for n in names],[end[n] for n in names])
        return dict(side=side,label=label,trajectory=trajectory,start=dict(q),end=end)

    def plan(self):
        return plan_transition(self.planner,self.model,self.start,self.target,'left',self.ticket,None,self.check)

    def test_matching_start_needs_no_scene_or_motion_plan(self):
        self.target=dict(self.start)
        self.assertEqual(self.plan(),[])
        self.scene.assert_not_called();self.planner.joint_path.assert_not_called()

    def test_single_arm_prefix_reaches_target_with_guarded_timing(self):
        from task_trajectory_speed import scale_arm_segments
        with patch('task_saved_transition.scale_arm_segments',wraps=scale_arm_segments) as scale:
            prefix=self.plan()
        self.assertEqual(len(prefix),1)
        self.assertEqual(prefix[0]['side'],'left')
        self.assertEqual(prefix[0]['start'],self.start)
        self.assertFalse(start_differences(prefix[-1]['end'],self.target))
        self.assertEqual(scale.call_args.args[1],1.)
        self.assertEqual(scale.call_args.kwargs['velocity'],.3)
        self.assertEqual(scale.call_args.kwargs['acceleration'],.5)
        self.planner.joint_path.assert_not_called()
        self.assertFalse(prefix[0]['collision_checked'])

    def test_passive_arm_and_waist_are_aligned_before_task_arm(self):
        self.target.update(right_elbow_joint=.2,waist_yaw_joint=.1)
        prefix=self.plan()
        self.assertEqual([s['side'] for s in prefix],['right','left'])
        self.assertEqual(prefix[0]['end'],prefix[1]['start'])
        self.assertFalse(start_differences(prefix[-1]['end'],self.target))

    def test_collision_planner_is_not_called_and_cancellation_still_stops(self):
        self.target['right_elbow_joint']=.2
        self.planner.joint_path.side_effect=RuntimeError('collision')
        self.assertEqual(len(self.plan()),2)
        self.planner.joint_path.assert_not_called()
        self.check.side_effect=RuntimeError('cancelled')
        with self.assertRaisesRegex(RuntimeError,'cancelled'):self.plan()
        self.planner.joint_path.assert_not_called()

    def test_joint_limits_still_reject_out_of_range_alignment(self):
        self.target['left_elbow_joint']=3.1
        with self.assertRaisesRegex(ValueError,'joint limit'):self.plan()

    def test_missing_waist_support_or_feedback_is_not_silently_ignored(self):
        self.target['waist_yaw_joint']=.1;self.planner.waist_assist=False
        with self.assertRaisesRegex(RuntimeError,'waist'):self.plan()
        del self.start['right_elbow_joint']
        with self.assertRaisesRegex(ValueError,'feedback'):self.plan()

    def test_combined_contract_preserves_archive_and_rejects_changed_prefix(self):
        prefix=self.plan()
        saved=self.path('left',self.target,self.target,self.ticket,'Approach')
        saved.update(number=1,kind='arm',duration=time_seconds(saved['trajectory'].points[-1]))
        plan=dict(kind='staged_test',hardware_executable=True,waist_assist=True,start=self.target,
                  task_schema=SCHEMA,sweep_direction='left',segments=[saved],
                  test_range=dict(first=1,last=1,scene_confirmed=True,archive='/fixture',
                      original_plan_sha256='hash',validated_fingerprint=range_fingerprint([saved])))
        original=copy.deepcopy(plan)
        combined=attach_transition(plan,prefix)
        with patch('task_saved_test.digest',return_value='hash'), \
                patch.object(Path,'read_text',return_value=json.dumps(dict(task_schema=SCHEMA,sweep_direction='left'))):
            verify_test_plan(combined)
            self.assertEqual(plan,original)
            self.assertEqual(combined['start'],self.start)
            self.assertEqual(combined['segments'][1:],plan['segments'])
            combined['segments'][0]['trajectory'].points[-1].positions[0]+=.01
            with self.assertRaisesRegex(RuntimeError,'changed'):verify_test_plan(combined)


if __name__=='__main__':unittest.main()
