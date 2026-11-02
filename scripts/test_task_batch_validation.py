from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock,patch

import numpy as np
from scipy.spatial.transform import Rotation
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint

from task_motion_planner import PlanningFailure,TaskPlanner
from task_scoped_planner import ScopedTaskPlanner
from task_straight_policy import arc_reference


def geometry_report():
    return {'checked_samples':0,'max_position_error_m':0.,
            'max_tilt_deg':0.,'max_twist_deg':0.}


def circle_error(center,radius):
    def scalar(point):
        return float(np.hypot(np.linalg.norm(point[:2]-center[:2])-radius,point[2]-center[2]))
    scalar.batch=lambda points:np.hypot(
        np.linalg.norm(points[:,:2]-center[:2],axis=1)-radius,points[:,2]-center[2])
    return scalar


class BatchGeometryTests(unittest.TestCase):
    def setUp(self):
        self.planner=object.__new__(TaskPlanner)
        self.policy={'tilt_deg':5.,'roll_deg':10.,'step_deg':2.}

    def tips(self,points,references):
        tips=np.broadcast_to(np.eye(4),(len(points),4,4)).copy()
        tips[:,:3,3]=points
        offsets=[Rotation.from_euler('yz',(tilt,twist),degrees=True)
                 for tilt,twist in ((0.,0.),(1.5,2.),(2.5,-3.),(.5,4.))]
        tips[:,:3,:3]=np.asarray([(Rotation.from_quat(reference)*offset).as_matrix()
                                  for reference,offset in zip(references,offsets)])
        return tips

    def test_batch_success_matches_scalar_extrema(self):
        center=np.zeros(3);entry=np.array([.2,0.,0.]);radius=.2
        points=np.array([[.2,0.,0.],[.199,.02,.0004],[.195,-.04,-.0006],[.19,.06,.0002]])
        nominal=arc_reference([0.,0.,0.,1.],center,entry)
        references=nominal.batch(points)
        tips=self.tips(points,references);curve=circle_error(center,radius)
        self.planner.tip=lambda side,state,wrist_from_tip:tips[state['index']]
        scalar=geometry_report()
        for index in range(len(tips)):
            self.planner.check_tcp_geometry('left',{'index':index},np.eye(4),nominal,curve,
                                                    self.policy,'rotate','Arc',scalar)
        batch=geometry_report()
        self.assertIsNone(self.planner.check_tcp_geometry_batch(
            tips,nominal,curve,self.policy,'rotate','Arc',batch))
        self.assertEqual(batch['checked_samples'],scalar['checked_samples'])
        for key in ('max_position_error_m','max_tilt_deg','max_twist_deg'):
            self.assertAlmostEqual(batch[key],scalar[key],places=11)

    def test_batch_reports_same_first_failure_sample(self):
        points=np.array([[.001,0.,0.],[.003,0.,0.],[.004,0.,0.]])
        tips=np.broadcast_to(np.eye(4),(len(points),4,4)).copy();tips[:,:3,3]=points
        curve=lambda point:float(np.linalg.norm(point))
        curve.batch=lambda values:np.linalg.norm(values,axis=1)
        self.planner.tip=lambda side,state,wrist_from_tip:tips[state['index']]
        scalar=geometry_report()
        self.planner.check_tcp_geometry('left',{'index':0},np.eye(4),[0.,0.,0.,1.],curve,
                                        self.policy,'insert','Line',scalar)
        with self.assertRaises(PlanningFailure) as caught:
            self.planner.check_tcp_geometry('left',{'index':1},np.eye(4),[0.,0.,0.,1.],curve,
                                            self.policy,'insert','Line',scalar)
        batch=geometry_report()
        error=self.planner.check_tcp_geometry_batch(
            tips,[0.,0.,0.,1.],curve,self.policy,'insert','Line',batch)
        self.assertIsNotNone(error)
        self.assertEqual(error.diagnostic['batch_sample_index'],1)
        for key in ('code','exceeded','position_error_m','tilt_deg','twist_deg'):
            self.assertEqual(error.diagnostic[key],caught.exception.diagnostic[key])


class BatchOrderingTests(unittest.TestCase):
    def planner(self,geometry_index):
        planner=object.__new__(ScopedTaskPlanner)
        planner.check_plan=lambda ticket:None
        planner.scoped_generation=7
        wrists=np.broadcast_to(np.eye(4),(3,4,4)).copy()
        planner.base_check=S(check_batch=lambda *args,**kwargs:(
            {'sample_index':1,'kind':'hand','link':'left_hand_base'},wrists))
        geometry=S(wrist_from_tip=np.eye(4))
        geometry.batch=lambda tips:PlanningFailure(
            'RETIMED_TCP_BOUNDS','rotate','geometry',batch_sample_index=geometry_index)
        return planner,geometry

    def test_same_sample_preserves_collision_first_order(self):
        planner,geometry=self.planner(1);ticket=S(kind='task',side='left',generation=7)
        with self.assertRaises(PlanningFailure) as caught:
            planner.validate_trajectory_samples({},ticket,'left','rotate',[],np.empty((3,0)),
                                                geometry,lambda index:None)
        self.assertEqual(caught.exception.diagnostic['code'],'COLLISION')

    def test_earlier_geometry_failure_still_wins(self):
        planner,geometry=self.planner(0);ticket=S(kind='task',side='left',generation=7)
        with self.assertRaises(PlanningFailure) as caught:
            planner.validate_trajectory_samples({},ticket,'left','rotate',[],np.empty((3,0)),
                                                geometry,lambda index:None)
        self.assertEqual(caught.exception.diagnostic['code'],'RETIMED_TCP_BOUNDS')


class BatchFallbackTests(unittest.TestCase):
    def test_prepare_keeps_scalar_validation_for_small_steps(self):
        planner=object.__new__(ScopedTaskPlanner);planner.waist_assist=False
        planner.progress=lambda ticket,message:None;planner.search_diagnostics={}
        planner.timing=object();planner.names=lambda side:('joint',)
        planner.joint_limits=lambda side:{'joint':(-1.,1.)}
        trajectory=JointTrajectory(joint_names=['joint'])
        trajectory.points=[JointTrajectoryPoint(positions=[0.],velocities=[0.],accelerations=[0.]),
                           JointTrajectoryPoint(positions=[.004],velocities=[0.],accelerations=[0.])]
        trajectory.points[-1].time_from_start.sec=1
        planner.call=lambda *args:S(success=True,trajectory=S(joint_trajectory=trajectory))
        planner.validate_trajectory_samples=Mock(side_effect=AssertionError('task batch path used'))
        validated=[]
        planner.validate=lambda state,*args:validated.append(state['joint'])
        ticket=S(velocity=.15,acceleration=.15,kind='prepare')
        sampled=[np.array([0.]),np.array([.004])]
        path=RobotTrajectory(joint_trajectory=trajectory)
        with patch('task_motion_planner.stretch_to_limits',return_value=1.),\
             patch('task_motion_planner.sample_count',return_value=2),\
             patch('task_motion_planner.samples',return_value=iter(sampled)):
            result=planner.retime('left',{'joint':0.},path,ticket,'Prepare')
        planner.validate_trajectory_samples.assert_not_called()
        self.assertEqual(result['end']['joint'],.004)
        self.assertEqual(validated,[0.,.004])
        self.assertFalse(planner.search_diagnostics['performance']['trajectory_validation'][0].get('batch',False))

    def test_prepare_uses_full_scene_cpp_batch_when_available(self):
        planner=object.__new__(TaskPlanner);planner.waist_assist=False
        planner.progress=lambda ticket,message:None;planner.search_diagnostics={}
        planner.timing=object();planner.batch_valid=object();planner.names=lambda side:('joint',)
        planner.joint_limits=lambda side:{'joint':(-1.,1.)}
        planner.validate_trajectory_batch=Mock()
        planner.validate=Mock(side_effect=AssertionError('scalar validator used'))
        trajectory=JointTrajectory(joint_names=['joint'])
        trajectory.points=[JointTrajectoryPoint(positions=[0.],velocities=[0.],accelerations=[0.]),
                           JointTrajectoryPoint(positions=[.004],velocities=[0.],accelerations=[0.])]
        trajectory.points[-1].time_from_start.sec=1
        planner.call=lambda *args:S(success=True,trajectory=S(joint_trajectory=trajectory))
        ticket=S(velocity=.15,acceleration=.15,kind='prepare')
        sampled=[np.array([0.]),np.array([.004])]
        path=RobotTrajectory(joint_trajectory=trajectory)
        with patch('task_motion_planner.stretch_to_limits',return_value=1.),\
             patch('task_motion_planner.sample_count',return_value=2),\
             patch('task_motion_planner.samples',return_value=iter(sampled)):
            result=planner.retime('left',{'joint':0.},path,ticket,'Prepare')
        planner.validate_trajectory_batch.assert_called_once()
        planner.validate.assert_not_called()
        self.assertEqual(result['end']['joint'],.004)
        performance=planner.search_diagnostics['performance']['trajectory_validation'][0]
        self.assertTrue(performance['batch'])
        self.assertEqual(performance['validator'],'full_scene_cpp')

    def test_large_neighbor_step_uses_scalar_fallback(self):
        planner=object.__new__(TaskPlanner);planner.waist_assist=False
        planner.progress=lambda ticket,message:None;planner.search_diagnostics={}
        planner.timing=object();planner.names=lambda side:('joint',)
        planner.joint_limits=lambda side:{'joint':(-1.,1.)}
        trajectory=JointTrajectory(joint_names=['joint'])
        trajectory.points=[JointTrajectoryPoint(positions=[0.],velocities=[0.],accelerations=[0.]),
                           JointTrajectoryPoint(positions=[.02],velocities=[0.],accelerations=[0.])]
        trajectory.points[-1].time_from_start.sec=1
        planner.call=lambda *args:S(success=True,trajectory=S(joint_trajectory=trajectory))
        planner.validate_trajectory_samples=Mock(side_effect=AssertionError('batch path used'))
        validated=[]
        planner.validate=lambda state,*args:validated.append(state['joint'])
        ticket=S(velocity=.15,acceleration=.15,kind='task')
        sampled=[np.array([0.]),np.array([.01]),np.array([.02])]
        path=RobotTrajectory(joint_trajectory=trajectory)
        with patch('task_motion_planner.stretch_to_limits',return_value=1.),\
             patch('task_motion_planner.sample_count',return_value=3),\
             patch('task_motion_planner.samples',return_value=iter(sampled)):
            result=planner.retime('left',{'joint':0.},path,ticket,'Fallback')
        planner.validate_trajectory_samples.assert_not_called()
        self.assertEqual(result['end']['joint'],.02)
        self.assertEqual(validated,[0.,.01,.005,.02,.015])
        self.assertFalse(planner.search_diagnostics['performance']['trajectory_validation'][0].get('batch',False))


class FullSceneBatchRequestTests(unittest.TestCase):
    def test_progress_rejects_other_requests_stale_and_out_of_order_counts(self):
        planner=object.__new__(TaskPlanner);callback=Mock()
        planner._pending_batch_progress=dict(identity=('snapshot','left','prepare'),
            started_ns=100,total=1000,checked=0,callback=callback)
        def message(start,count,total=1000,snapshot='snapshot'):
            return S(layout=S(dim=[S(label=v) for v in (snapshot,'left','prepare')]),
                     data=[start,count,total])
        for msg in (message(99,200),message(101,200,snapshot='old'),message(101,200,500)):
            planner.batch_progress(msg)
        callback.assert_not_called()
        planner.batch_progress(message(101,200));callback.assert_called_once_with(200)
        planner.batch_progress(message(101,100));planner.batch_progress(message(101,1001))
        self.assertEqual(callback.call_count,1)
        planner.batch_progress(message(101,600));self.assertEqual(callback.call_count,2)
        planner._pending_batch_progress=None
        planner.batch_progress(message(101,1000));self.assertEqual(callback.call_count,2)

    def planner(self,result):
        planner=object.__new__(TaskPlanner)
        planner.batch_valid=object();planner.call=Mock(return_value=result)
        planner.snapshot_id='task_measured_rgbd_snapshot_batch'
        planner.target_id=''
        return planner

    def test_flattened_states_preserve_fixed_joints_and_sample_order(self):
        result=S(valid=True,checked_samples=2,first_invalid_sample=2,code='VALID')
        planner=self.planner(result);ticket=S(side='left',kind='prepare')
        planner.validate_trajectory_batch({'fixed':9.,'moving':1.},ticket,'left','prepare',
                                          ('moving',),np.array([[2.],[3.]]))
        request=planner.call.call_args.args[1]
        self.assertEqual(list(request.joint_names),['fixed','moving'])
        self.assertEqual(list(request.positions),[9.,2.,9.,3.])
        self.assertEqual(request.sample_count,2)

    def test_first_invalid_sample_maps_to_planning_diagnostic(self):
        result=S(valid=False,checked_samples=2,first_invalid_sample=1,code='COLLISION',
                 message='contact',contacts=[])
        planner=self.planner(result);ticket=S(side='left',kind='prepare')
        with self.assertRaises(PlanningFailure) as caught:
            planner.validate_trajectory_batch({'moving':1.},ticket,'left','prepare',
                                              ('moving',),np.array([[1.],[2.]]))
        self.assertEqual(caught.exception.diagnostic['batch_sample_index'],1)
        self.assertEqual(caught.exception.diagnostic['checked_samples'],2)


if __name__=='__main__':unittest.main()
