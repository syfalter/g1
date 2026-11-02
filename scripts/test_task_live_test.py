"""Full live commissioning authorization and execution, using nonactuating clients."""
import copy
import json
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock,patch

from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from rosidl_runtime_py.convert import message_to_ordereddict
from task_hand_control import POSTURES,model_joints,model_feedback
from task_plan_archive import digest
from task_saved_test import LABELS,live_test_plan,verify_test_plan
from task_dring_sequence import SCHEMA
from task_session import Session,ARMS,ARM_NAMES


class LiveTestTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.directory=Path(self.temp.name);self.urdf=self.directory/'robot.urdf'
        self.urdf.write_text('test model identity')
        q=dict.fromkeys(ARM_NAMES,.2)
        for side in ARMS:q.update(model_joints(side,[1.]*6))
        self.t=10.;self.session=Session(lambda:self.t)
        self.feedback(q);self.session.capture_home()
        self.ticket=self.session.begin('task','left',.15,.15)
        self.session.accept_snapshot(self.ticket,{'sequence':1})
        segments=[];current=dict(q)
        for label in LABELS:
            hand=label.startswith('Hand ');end=dict(current)
            target=list(POSTURES[label[5:]]) if hand else None
            names=list(model_joints('left',target)) if hand else list(ARMS['left'])
            if hand:end.update(model_joints('left',target))
            else:end[names[0]]+=.001
            trajectory=JointTrajectory(joint_names=names)
            for sec,state in ((0,current),(2,end)):
                point=JointTrajectoryPoint(positions=[state[n] for n in names],
                    velocities=[0.]*len(names),accelerations=[0.]*len(names))
                point.time_from_start.sec=sec;trajectory.points.append(point)
            segment=dict(side='left',label=label,kind='hand' if hand else 'arm',
                posture=label[5:] if hand else None,target_q=target,
                start=current,end=end,trajectory=trajectory)
            if hand:segment['duration']=2.
            segments.append(segment);current=end
        source=dict(kind='task',arm='left',hardware_executable=False,waist_assist=False,
            task_schema=SCHEMA,sweep_direction='left',
            planning_profile='steep_straight',start=q,
            contact_scene=dict(profile='steep_straight',arm_body_collision_checked=True,
                base_radius_m=.025,base_height_m=.02,collision_scope='test fixture'),
            segments=[dict(side=s['side'],label=s['label'],kind=s['kind'],posture=s['posture'],
                target_q=s['target_q'],trajectory=message_to_ordereddict(s['trajectory'])) for s in segments])
        self.source=self.directory/'plan.json';self.source.write_text(json.dumps(source))
        self.plan=dict(kind='task',hardware_executable=False,waist_assist=False,start=q,
            task_schema=SCHEMA,sweep_direction='left',
            directory=str(self.directory),segments=segments,live_review=dict(generation=self.ticket.generation,
                stop_epoch=3,source_plan_sha256=digest(self.source),robot_model_sha256=digest(self.urdf)))

    def feedback(self,q):
        for _ in range(15):self.t+=.02;self.session.update(q)
        for _ in range(10):self.t+=.02;self.session.update(q)

    def authorize(self,**kwargs):
        options=dict(scene_confirmed=True,contact_confirmed=True);options.update(kwargs)
        return live_test_plan(self.plan,self.ticket,3,self.urdf,**options)

    def test_exact_seven_stages_without_mutating_preview(self):
        result=self.authorize();verify_test_plan(result)
        self.assertEqual([s['number'] for s in result['segments']],list(range(1,8)))
        self.assertTrue(result['hardware_executable']);self.assertFalse(self.plan['hardware_executable'])
        self.assertFalse(json.loads(self.source.read_text())['hardware_executable'])

    def test_missing_confirmations_old_epoch_and_changed_model_are_rejected(self):
        for key in ('scene_confirmed','contact_confirmed'):
            with self.assertRaises(RuntimeError):self.authorize(**{key:False})
        self.plan['live_review']['stop_epoch']=2
        with self.assertRaises(RuntimeError):self.authorize()
        self.plan['live_review']['stop_epoch']=3;self.urdf.write_text('changed')
        with self.assertRaises(RuntimeError):self.authorize()

    def test_changed_source_or_runtime_curve_cannot_be_authorized(self):
        self.plan['segments'][0]['trajectory'].points[-1].positions[0]+=.01
        with self.assertRaisesRegex(RuntimeError,'differs'):self.authorize()
        self.source.write_text('{}')
        with self.assertRaisesRegex(RuntimeError,'changed'):self.authorize()

    def test_changed_authorized_curve_or_source_cannot_execute(self):
        result=self.authorize();result['segments'][3]['trajectory'].points[-1].positions[0]+=.01
        with self.assertRaisesRegex(RuntimeError,'changed'):verify_test_plan(result)

    def run_full(self,fail_at=None,*,contact_confirmed=True,contact_test_enabled=True):
        from real_task_app import TaskNode
        self.session.ready(self.ticket,self.plan);sent=[];hand_targets=[]
        def read_hands(ticket):
            return {side:dict(feedback_q=model_feedback(side,self.session.current)) for side in ARMS}
        def send(goal):
            sent.append(copy.deepcopy(goal.trajectory))
            return S(accepted=len(sent)!=fail_at,get_result_async=lambda:('result',goal.trajectory))
        def wait(future,*args):
            if not isinstance(future,tuple):return future
            trajectory=future[1];q=dict(self.session.current)
            q.update(zip(trajectory.joint_names,trajectory.points[-1].positions));self.feedback(q)
            return S(status=4,result=S(error_code=0,error_string=''))
        def transition(side,target,check,**kwargs):
            check();hand_targets.append(target);q=dict(self.session.current)
            q.update(model_joints(side,target));self.feedback(q);check();return []
        node=S(args=S(hardware=True,dring_contact_test=contact_test_enabled,robot_urdf=str(self.urdf)),model=Mock(),
            session=self.session,lock=threading.RLock(),gateway_received=time.monotonic(),
            gateway={'phase':'ARMED_HOLD','task_stop_epoch':'3'},saved_test_guard=Mock(),
            hand_control=True,read_hands=read_hands,hand=S(transition=transition,begin_motion=lambda check:check()),cache_hand_state=Mock(),
            actions={'left':S(wait_for_server=lambda **kw:True,send_goal_async=send)},
            check=self.session.check,wait=wait,launch_work=lambda function:function(),
            get_clock=lambda:S(now=lambda:S(to_msg=lambda:JointTrajectory().header.stamp)),
            stop=Mock(side_effect=self.session.stop))
        if fail_at:
            with self.assertRaisesRegex(RuntimeError,'rejected'):
                TaskNode.execute(node,scene_confirmed=True,contact_confirmed=contact_confirmed)
        else:TaskNode.execute(node,scene_confirmed=True,contact_confirmed=contact_confirmed)
        return node,sent,hand_targets

    def test_full_sequence_has_five_arm_actions_and_two_hand_transitions(self):
        node,sent,hands=self.run_full()
        self.assertEqual(len(sent),5);self.assertEqual(len(hands),2)
        self.assertEqual(self.session.phase,'ARRIVED_HOLD');node.stop.assert_not_called()
        self.assertEqual(len(json.loads((self.directory/'execution.json').read_text())),7)

    def test_full_sequence_can_execute_after_planning_feedback_gap_recovers(self):
        self.t+=5.
        self.session.ready(self.ticket,self.plan)
        self.assertIn('stale',self.session.preview_issue(self.plan))
        self.feedback(self.plan['start'])
        node,sent,hands=self.run_full()
        self.assertEqual(len(sent),5);self.assertEqual(len(hands),2)
        node.stop.assert_not_called()

    def test_stale_feedback_rejects_live_execute_before_authorization_or_worker(self):
        from real_task_app import TaskNode
        self.t+=5.;self.session.ready(self.ticket,self.plan)
        node=S(args=S(hardware=True,dring_contact_test=True),session=self.session,
               lock=threading.RLock(),gateway_received=time.monotonic(),
               gateway={'phase':'ARMED_HOLD'},launch_work=Mock(),saved_test_guard=Mock())
        with self.assertRaisesRegex(RuntimeError,'stale'):
            TaskNode.execute(node,scene_confirmed=True,contact_confirmed=True)
        node.launch_work.assert_not_called();node.saved_test_guard.assert_not_called()
        self.assertIs(self.session.plan,self.plan)
        self.assertFalse((self.directory/'live_test_authorization.json').exists())

    def test_rejected_action_stops_full_task_without_later_stages(self):
        node,sent,hands=self.run_full(fail_at=2)
        self.assertEqual(len(sent),2);self.assertEqual(len(hands),1)
        node.stop.assert_called_once();self.assertTrue(self.session.stopped)

    def center_plan(self):
        from task_dring_sequence import CENTER_SCHEMA,CENTER_LABELS
        source=json.loads(self.source.read_text());q=dict(self.plan['start']);segments=[]
        for index,label in enumerate(CENTER_LABELS):
            segment=copy.deepcopy(self.plan['segments'][1 if index==0 else 2])
            segment.update(label=label,start=dict(q));end=dict(q)
            if index==0:end.update(model_joints('left',segment['target_q']))
            else:end[ARMS['left'][0]]+=.001
            for p,state in zip(segment['trajectory'].points,(q,end)):
                p.positions=[state[n] for n in segment['trajectory'].joint_names]
            segment['end']=end;segments.append(segment);q=end
        self.plan.update(task_schema=CENTER_SCHEMA,sweep_direction=None,segments=segments,
                         geometry={'center_check':{}})
        source.update(task_schema=CENTER_SCHEMA,sweep_direction=None,segments=[
            dict(side=s['side'],label=s['label'],kind=s['kind'],posture=s['posture'],target_q=s['target_q'],
                 trajectory=message_to_ordereddict(s['trajectory'])) for s in segments])
        self.source.write_text(json.dumps(source))
        self.plan['live_review']['source_plan_sha256']=digest(self.source)

    def test_center_authorizes_three_stages_without_contact_but_requires_scene_and_identity(self):
        self.center_plan();result=self.authorize(contact_confirmed=False)
        verify_test_plan(result)
        self.assertEqual(result['test_range']['last'],3)
        self.assertFalse(result['test_range']['contact_confirmed'])
        with self.assertRaises(RuntimeError):self.authorize(scene_confirmed=False,contact_confirmed=False)
        result['test_range']['last']=2
        with self.assertRaises(RuntimeError):verify_test_plan(result)

    def test_center_executes_only_hand_approach_descent_then_holds_and_records(self):
        self.center_plan()
        with patch('task_center_check.measurement',return_value={'fk_error_mm':[1.,2.,3.]}):
            node,sent,hands=self.run_full(contact_confirmed=False,contact_test_enabled=False)
        self.assertEqual(len(sent),2);self.assertEqual(len(hands),1)
        self.assertEqual(self.session.phase,'ARRIVED_HOLD');node.stop.assert_not_called()
        self.assertTrue(json.loads((self.directory/'center_check_result.json').read_text())['settled'])
        self.assertIn('Center check complete',node.status)


if __name__=='__main__':unittest.main()
