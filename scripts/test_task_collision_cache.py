import copy
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock,patch

from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from task_collision_cache import certificate,matches,remember
from task_saved_test import _validate_range
from task_parallel_collision import validate
from rosidl_runtime_py.convert import message_to_ordereddict
import xml.etree.ElementTree as ET
from types import SimpleNamespace


class CacheTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.directory=Path(self.tmp.name)
        self.trajectory=JointTrajectory(joint_names=['left_wrist_yaw_joint'])
        for time in (0,2):
            p=JointTrajectoryPoint(positions=[0.],velocities=[0.],accelerations=[0.])
            p.time_from_start.sec=time;self.trajectory.points.append(p)
        self.segment=dict(number=1,side='left',label='Approach',kind='arm',posture=None,target_q=None,
                          trajectory=self.trajectory,duration=2.,start={'left_wrist_yaw_joint':0.},
                          end={'left_wrist_yaw_joint':0.})
        self.context=dict(model='model',scene='scene',checker='code')

    def test_persistent_explicit_cache_and_corruption_fails_closed(self):
        record=certificate(self.context,self.segment)
        self.assertFalse(matches(record,self.directory))
        remember([record],self.directory)
        self.assertTrue(matches(record,Path(str(self.directory))))
        (self.directory/(record['key']+'.json')).write_text('broken')
        self.assertFalse(matches(record,self.directory))

    def test_model_scene_code_range_and_timing_changes_invalidate(self):
        record=certificate(self.context,self.segment);remember([record],self.directory)
        for key in self.context:
            self.assertFalse(matches(certificate(dict(self.context,**{key:'changed'}),self.segment),self.directory))
        for key,value in (('number',2),('start',{'left_wrist_yaw_joint':.1})):
            self.assertFalse(matches(certificate(self.context,dict(self.segment,**{key:value})),self.directory))
        changed=copy.deepcopy(self.segment)
        changed['trajectory'].points[-1].time_from_start.sec=3
        self.assertFalse(matches(certificate(self.context,changed),self.directory))

    def test_hit_skips_collision_but_still_checks_limits_and_cancellation(self):
        record=dict(self.segment,original_segment=dict(trajectory=message_to_ordereddict(self.trajectory)))
        model=SimpleNamespace(robot=ET.fromstring('<robot><joint name="left_wrist_yaw_joint">'
            '<limit lower="-1" upper="1"/></joint></robot>'))
        factory=Mock();progress=Mock();check=Mock();report={}
        args=([record],[self.segment],{},1.,1.,model,None,factory,None,validate,check,progress,self.context,True,report)
        with patch('task_collision_cache.matches',return_value=True):
            _validate_range(*args)
            self.assertEqual(report['cached_stages'],[1])
            factory.assert_not_called();self.assertTrue(check.called)
            self.trajectory.points[-1].positions=[2.]
            with self.assertRaises(ValueError):_validate_range(*args)
            self.trajectory.points[-1].positions=[0.]
            check.side_effect=RuntimeError('cancelled')
            with self.assertRaisesRegex(RuntimeError,'cancelled'):_validate_range(*args)

    def test_failed_collision_never_produces_rememberable_certificate(self):
        record=dict(self.segment,original_segment=dict(trajectory=message_to_ordereddict(self.trajectory)))
        model=SimpleNamespace(robot=ET.fromstring('<robot><joint name="left_wrist_yaw_joint">'
            '<limit lower="-1" upper="1"/></joint></robot>'))
        report={}
        with self.assertRaisesRegex(ValueError,'collision'):
            _validate_range([record],[self.segment],{},1.,1.,model,None,Mock(),None,
                Mock(side_effect=ValueError('collision')),lambda:None,Mock(),self.context,False,report)
        self.assertEqual(report,{})


if __name__=='__main__':unittest.main()
