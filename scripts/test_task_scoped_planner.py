import json
from pathlib import Path
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock,patch
import numpy as np
from task_scoped_planner import ScopedTaskPlanner
from task_motion_planner import PlanningFailure
from task_straight_preview import StraightPreviewPlanner
from task_straight_policy import RADIAL_OFFSET_M,HEIGHT_OFFSET_M
from task_preview_markers import scoped_target_markers
from vision_task_geometry import with_radial_offset

ROOT=Path(__file__).resolve().parents[1]


class ScopedPlannerTests(unittest.TestCase):
    def planner(self):
        p=ScopedTaskPlanner.__new__(ScopedTaskPlanner)
        p.check_plan=Mock();p.scoped_generation=3;p.base_check=Mock()
        p.base_check.check_state.return_value=(None,None)
        p.scoped_limits={'left':{'left_elbow_joint':(-1,1)}}
        return p

    def test_archive_uses_online_implementation(self):
        self.assertTrue(issubclass(StraightPreviewPlanner,ScopedTaskPlanner))
        self.assertIs(StraightPreviewPlanner.validate,ScopedTaskPlanner.validate)
        self.assertIs(StraightPreviewPlanner.joint_path,ScopedTaskPlanner.joint_path)
        self.assertFalse(ScopedTaskPlanner.task_hardware_executable)

    def test_prepare_recover_keep_original_collision_and_motion(self):
        p=self.planner()
        for kind in ('prepare','recover'):
            ticket=S(kind=kind,side='left',generation=3)
            with patch('task_scoped_planner.TaskPlanner.validate') as validate:
                p.validate({},ticket);validate.assert_called_once()
            with patch('task_scoped_planner.TaskPlanner.joint_path') as path:
                p.joint_path('left',{}, {},ticket,kind);path.assert_called_once()
            with patch('task_scoped_planner.TaskPlanner.install_surface') as scene:
                p.install_surface([],np.eye(4),ticket,center=[0,0,0]);scene.assert_called_once()
            self.assertIsNone(p.base_check)

    def test_task_stale_target_cancel_limits_and_collisions(self):
        p=self.planner();ticket=S(kind='task',side='left',generation=3)
        q={'left_elbow_joint':0.}
        p.validate(q,ticket)
        p.base_check.check_state.assert_called_once_with(q,'left')
        p.base_check.check_state.return_value=('left_thumb_1',None)
        with self.assertRaisesRegex(PlanningFailure,'COLLISION'):p.validate(q,ticket)
        p.base_check.check_state.return_value=(None,('left_elbow_link','torso_link'))
        with self.assertRaisesRegex(PlanningFailure,'COLLISION'):p.validate(q,ticket)
        with self.assertRaisesRegex(PlanningFailure,'JOINT_MARGIN'):p.validate({'left_elbow_joint':.99},ticket)
        ticket.generation=4
        with self.assertRaisesRegex(PlanningFailure,'SCENE_NOT_READY'):p.validate(q,ticket)
        p.check_plan.side_effect=RuntimeError('STOP cancelled')
        with self.assertRaisesRegex(RuntimeError,'STOP cancelled'):p.validate(q,ticket)

    def test_only_task_ik_skips_external_scene(self):
        p=self.planner();p.orientation_search='broad'
        self.assertFalse(p.ik_avoid_collisions(S(kind='task'),'approach'))
        self.assertTrue(p.ik_avoid_collisions(S(kind='prepare'),'prepare'))

    def test_bounded_offline_candidate_still_requires_existing_scoped_validator(self):
        from task_motion_planner import TaskPlanner
        p=self.planner();p.ik_backend='bounded';p.reference_frame='pelvis';p.model=Mock();p.call=Mock()
        q={'left_elbow_joint':0.};answer={'left_elbow_joint':.1}
        p._names={'left':list(q)};p.joint_limits=lambda side:p.scoped_limits[side]
        ticket=S(kind='task',side='left',generation=3)
        with patch('task_ik_search.bounded_pose_ik',return_value=(answer,{'code':'LOCAL_IK_POSE_VALID'})):
            self.assertEqual(TaskPlanner.solve(p,'left',q,np.eye(4),np.eye(4),ticket),answer)
            p.base_check.check_state.assert_called_with(answer,'left')
            p.base_check.check_state.return_value=('left_thumb_1',None)
            self.assertIsNone(TaskPlanner.solve(p,'left',q,np.eye(4),np.eye(4),ticket))
            self.assertEqual(p.last_ik['code'],'COLLISION')
        p.call.assert_not_called()
        p.reference_frame='torso_link'
        with self.assertRaisesRegex(ValueError,'pelvis-frame'):
            TaskPlanner.solve(p,'left',q,np.eye(4),np.eye(4),ticket)

    def test_task_tolerance_backend_preserves_preinsert_and_collision_gate(self):
        from task_motion_planner import TaskPlanner
        p=self.planner();p.ik_backend='bounded-task';p.reference_frame='pelvis';p.model=Mock()
        q={'left_elbow_joint':.08};previous={'left_elbow_joint':0.};answer={'left_elbow_joint':.1}
        p._names={'left':list(q)};p.joint_limits=lambda side:p.scoped_limits[side]
        ticket=S(kind='task',side='left',generation=3)
        with patch('task_ik_search.bounded_pose_ik',return_value=(answer,{'code':'LOCAL_IK_POSE_VALID'})) as solve:
            TaskPlanner.solve(p,'left',q,np.eye(4),np.eye(4),ticket,'approach')
            self.assertEqual(solve.call_args.kwargs,{})
            with self.assertRaisesRegex(ValueError,'previous state'):
                TaskPlanner.solve(p,'left',q,np.eye(4),np.eye(4),ticket,'rotate')
            result=TaskPlanner.solve(p,'left',q,np.eye(4),np.eye(4),ticket,'rotate',continuity_reference=previous)
            self.assertEqual(result,answer)
            self.assertIs(solve.call_args.kwargs['continuity_reference'],previous)
            self.assertEqual(solve.call_args.kwargs['position_tolerance'],.002)
            p.base_check.check_state.return_value=('left_thumb_1',None)
            self.assertIsNone(TaskPlanner.solve(p,'left',q,np.eye(4),np.eye(4),ticket,'rotate',
                                               continuity_reference=previous))
            self.assertEqual(p.last_ik['code'],'COLLISION')

    def test_ompl_fallback_is_opt_in_return_collision_only(self):
        for backend,stage,code,expected in (
                ('line','return_ready','COLLISION',False),('ompl','approach','COLLISION',False),
                ('ompl','return_ready','JOINT_MARGIN',False),('ompl','return_ready','COLLISION',True)):
            with self.subTest(backend=backend,stage=stage,code=code):
                p=self.planner();p._names={'left':['left_elbow_joint']};p.joint_route_backend=backend
                p.scoped_joint_line=Mock(side_effect=PlanningFailure(code,stage,'rejected'))
                ticket=S(kind='task',side='left',generation=3);q={'left_elbow_joint':0.}
                with patch('task_joint_route.return_route',return_value='accepted') as route:
                    if expected:self.assertEqual(p.joint_path('left',q,q,ticket,'Return',stage),'accepted')
                    else:
                        with self.assertRaises(PlanningFailure):p.joint_path('left',q,q,ticket,'Return',stage)
                    self.assertEqual(route.call_count,int(expected))

    def test_constrained_backend_preserves_preinsert_and_requires_scoped_validation(self):
        from task_motion_planner import TaskPlanner
        from task_planning_config import PlanningParameters
        p=self.planner();p.ik_backend='constrained';p.reference_frame='pelvis';p.model=Mock()
        p.planning_parameters=PlanningParameters(hand_back_tangent_xy=True)
        p.tangent_center=np.zeros(3);p.dorsal_in_tip=np.array([0.,1.,0.]);p.tangent_branch=1.
        q={'left_elbow_joint':.08};previous={'left_elbow_joint':0.};answer={'left_elbow_joint':.1}
        p._names={'left':list(q)};p.joint_limits=lambda side:p.scoped_limits[side]
        ticket=S(kind='task',side='left',generation=3)
        with patch('task_ik_search.bounded_pose_ik',return_value=(answer,{'code':'VALID'})) as solve:
            TaskPlanner.solve(p,'left',q,np.eye(4),np.eye(4),ticket,'approach')
            self.assertEqual(solve.call_args.kwargs,{})
            with self.assertRaisesRegex(ValueError,'task context'):
                TaskPlanner.solve(p,'left',q,np.eye(4),np.eye(4),ticket,'rotate')
        with patch('task_contact_ik.constrained_contact_ik',return_value=(answer,{'code':'LOCAL_TASK_VALID'})) as solve:
            def run():return p.solve_constrained_contact('left',q,previous,[.4,0.,0.],[0.,0.,0.,1.],
                np.eye(4),ticket,'rotate',dict(tilt_deg=4.9,roll_deg=9.9,step_deg=2.),.003)
            self.assertEqual(run(),answer)
            self.assertIs(solve.call_args.args[5],previous)
            p.base_check.check_state.assert_called_with(answer,'left')
            p.base_check.check_state.return_value=('left_thumb_1',None)
            self.assertIsNone(run())
            self.assertEqual(p.last_ik['code'],'COLLISION')
            p.scoped_generation=4
            with self.assertRaisesRegex(PlanningFailure,'SCENE_NOT_READY'):run()
            p.reference_frame='torso_link'
            with self.assertRaisesRegex(ValueError,'fixed physical tangent'):run()


class RadialIntegrationTests(unittest.TestCase):
    def geometry(self):
        path=ROOT/'data/task_sessions/20260909_154321_451801/plan_0008/geometry.json'
        if not path.exists():self.skipTest('Archive unavailable')
        return json.loads(path.read_text())

    def test_inward_five_mm_is_applied_once_to_all_targets(self):
        before=with_radial_offset(self.geometry(),.015,height_offset_m=0.)
        saved=json.dumps(before,sort_keys=True)
        after=with_radial_offset(before,RADIAL_OFFSET_M,height_offset_m=HEIGHT_OFFSET_M)
        center=np.array(before['task_preview']['axis_center_m'])
        old=np.array(before['task_preview']['insertion_reference_m'])-center
        new=np.array(after['task_preview']['insertion_reference_m'])-center
        self.assertAlmostEqual(np.linalg.norm(old[:2])-np.linalg.norm(new[:2]),.005)
        np.testing.assert_allclose(new[:2]/np.linalg.norm(new[:2]),old[:2]/np.linalg.norm(old[:2]),atol=1e-12)
        self.assertAlmostEqual(new[2]-old[2],.010)
        np.testing.assert_allclose(after['task_preview']['axis_center_m'],center)
        for point in after['task_preview']['arc_minus120_to_plus120_m']:
            self.assertAlmostEqual(np.linalg.norm(np.array(point)[:2]-center[:2]),np.linalg.norm(new[:2]))
            self.assertAlmostEqual(point[2],center[2]+.010)
        for side in ('left','right'):
            np.testing.assert_allclose(after['grasp_candidates'][side]['poses']['index_tcp']['position_m'],new+center)
        self.assertEqual(with_radial_offset(after,RADIAL_OFFSET_M),after)
        self.assertEqual(json.dumps(before,sort_keys=True),saved)

    def test_scoped_marker_has_correct_base_and_no_old_axes(self):
        from visualization_msgs.msg import Marker
        scene={'base_top_pelvis_m':[.3,.1,.8],'base_height_m':.02,'base_radius_m':.025}
        geometry=with_radial_offset(self.geometry(),RADIAL_OFFSET_M,height_offset_m=HEIGHT_OFFSET_M)
        markers=scoped_target_markers(geometry,np.eye(4),'left',scene).markers
        base=next(m for m in markers if m.type==Marker.CYLINDER)
        self.assertAlmostEqual(base.pose.position.z,.79)
        self.assertAlmostEqual(base.scale.x,.05)
        arc=next(m for m in markers if m.type==Marker.LINE_STRIP)
        self.assertTrue(all(abs(p.z-geometry['task_preview']['insertion_reference_m'][2])<1e-12 for p in arc.points))
        self.assertFalse(any(m.type==Marker.ARROW and m.action!=Marker.DELETEALL for m in markers))


if __name__=='__main__':unittest.main()
