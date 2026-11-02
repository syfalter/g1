import copy
import json
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace as S
import unittest
from unittest.mock import patch, Mock

import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from task_saved_test import select_range, speed_copy, needs_contact_confirmation, range_fingerprint, verify_test_plan
from task_trajectory import coefficients, time_seconds, samples
from task_session import Session, ARM_NAMES, ARMS
from task_hand_control import model_joints
from task_dring_sequence import SCHEMA


class SavedRangeTests(unittest.TestCase):
    def trajectory(self):
        trajectory=JointTrajectory(joint_names=['left_wrist_yaw_joint'])
        for seconds,position in ((0,0.),(10,.2)):
            point=JointTrajectoryPoint(positions=[position],velocities=[0.],accelerations=[0.])
            point.time_from_start.sec=seconds
            trajectory.points.append(point)
        return trajectory

    def test_all_28_ranges_are_exact_contiguous_selections(self):
        archive={'stages':[{'number':i} for i in range(1,8)]}
        for first in range(1,8):
            for last in range(first,8):
                result=select_range(archive,first,last)
                self.assertEqual([s['number'] for s in result],list(range(first,last+1)))
                result[0]['number']=99
        self.assertEqual(archive['stages'][0]['number'],1)
        for first,last in ((0,1),(4,3),(1,8),(True,1)):
            with self.assertRaises(ValueError):select_range(archive,first,last)

    def test_contact_ack_only_for_ranges_intersecting_insert_through_withdraw(self):
        for first,last,wanted in ((1,1,False),(1,2,False),(2,3,True),(4,7,True),
                                  (5,5,True),(6,7,False),(1,7,True)):
            self.assertEqual(needs_contact_confirmation(first,last),wanted)

    def test_speed_copy_preserves_positions_and_normalized_polynomial(self):
        original=self.trajectory();before=copy.deepcopy(original)
        faster=speed_copy(original,.15,.5,.5,False)
        self.assertEqual(original,before)
        self.assertLess(time_seconds(faster.points[-1]),10.)
        c0=coefficients(*original.points,10.)
        c1=coefficients(*faster.points,time_seconds(faster.points[-1]))
        np.testing.assert_allclose(c0,c1,atol=1e-8)
        self.assertGreater(len(list(samples(faster,.6*.5,1.5*.5))),1)

    def test_nonfinite_scaling_rejected(self):
        for scale in (float('nan'),0,1.1):
            with self.assertRaises(ValueError):speed_copy(self.trajectory(),.15,scale,.5,False)

    def test_fingerprint_detects_content_change_but_allows_header_stamp(self):
        segment={'number':1,'label':'Approach','trajectory':self.trajectory()}
        fingerprint=range_fingerprint([segment])
        segment['trajectory'].header.stamp.sec=500
        self.assertEqual(range_fingerprint([segment]),fingerprint)
        segment['trajectory'].points[-1].positions[0]+=.001
        self.assertNotEqual(range_fingerprint([segment]),fingerprint)

    def test_contract_rejects_unconfirmed_contact_and_changed_range(self):
        segment={'number':3,'label':'Insert','trajectory':self.trajectory()}
        plan={'kind':'staged_test','segments':[segment],'test_range':dict(first=3,last=3,
            scene_confirmed=True,contact_confirmed=False),'task_schema':SCHEMA,'sweep_direction':'left'}
        with self.assertRaisesRegex(RuntimeError,'confirmations'):verify_test_plan(plan)
        plan['test_range']['contact_confirmed']=True
        plan['test_range']['validated_fingerprint']=range_fingerprint([segment])
        plan['test_range'].update(archive='/unused',original_plan_sha256='test')
        with patch('task_saved_test.digest',return_value='test'),patch.object(Path,'read_text',return_value=json.dumps(plan,default=str)):
            verify_test_plan(plan)
        plan['segments'].append(segment)
        with self.assertRaisesRegex(RuntimeError,'changed'):verify_test_plan(plan)

    def test_stable_review_no_wall_clock_expiry_but_stop_or_motion_blocks(self):
        t=[10.];session=Session(lambda:t[0]);q=dict.fromkeys(ARM_NAMES,.2)
        for _ in range(7):t[0]+=.1;session.update(q)
        session.capture_home()
        ticket=session.begin('staged_test','both',.15,.15)
        plan=dict(kind='staged_test',hardware_executable=True,test_range={'first':1,'last':1},
                  start=q,segments=['test'])
        session.ready(ticket,plan)
        for _ in range(3000):t[0]+=.1;session.update(q)
        for _ in range(7):t[0]+=.1;session.update(q)
        self.assertIsNone(session.expire_plan());self.assertIsNone(session.execution_issue())
        moved=dict(q);moved[ARM_NAMES[0]]+=.04;session.update(moved)
        for _ in range(15):t[0]+=.1;session.update(q)
        self.assertIn('moved',session.execution_issue())
        with self.assertRaises(RuntimeError):session.execution()
        session.stop()
        with self.assertRaises(RuntimeError):session.begin('staged_test','both',.15,.15)


class ExecutionRangeTests(unittest.TestCase):
    def run_range(self, fail_second=False):
        from real_task_app import TaskNode
        t=[10.];session=Session(lambda:t[0]);q=dict.fromkeys(ARM_NAMES,.2)
        for side in ARMS:q.update(model_joints(side,[1.]*6))
        def feedback(state):
            for _ in range(15):t[0]+=.1;session.update(state)
        feedback(q);session.capture_home();ticket=session.begin('staged_test','both',.3,.3)
        segments=[];current=dict(q)
        for number,label in ((4,'Left 120'),(5,'Withdraw')):
            end=dict(current);end[ARM_NAMES[0]]+=.01
            trajectory=JointTrajectory(joint_names=list(ARMS['left']))
            for sec,state in ((0,current),(1,end)):
                point=JointTrajectoryPoint(positions=[state[n] for n in ARMS['left']],
                                           velocities=[0.]*7,accelerations=[0.]*7)
                point.time_from_start.sec=sec;trajectory.points.append(point)
            segments.append(dict(number=number,label=label,kind='arm',side='left',
                                 start=current,end=end,trajectory=trajectory))
            current=end
        sent=[]
        def send(goal):
            sent.append(copy.deepcopy(goal.trajectory))
            index=len(sent)-1
            handle=S(accepted=not(fail_second and index==1),get_result_async=lambda:('result',index))
            return ('goal',handle)
        def wait(future,*args):
            if future[0]=='goal':return future[1]
            feedback(segments[future[1]]['end'])
            return S(status=4,result=S(error_code=0,error_string=''))
        node=S(args=S(hardware=True),session=session,lock=threading.RLock(),
            gateway={'phase':'ARMED_HOLD'},gateway_received=time.monotonic(),
            saved_test_guard=Mock(),hand_control=True,read_hands=lambda ticket:{
                side:{'feedback_q':[1.]*6} for side in ARMS},
            actions={'left':S(wait_for_server=lambda **kw:True,send_goal_async=send)},
                check=session.check,wait=wait,launch_work=lambda function:function(),
                hand=S(begin_motion=lambda check:check()),
            get_clock=lambda:S(now=lambda:S(to_msg=lambda:JointTrajectory().header.stamp)),
            stop=Mock(side_effect=session.stop))
        with tempfile.TemporaryDirectory() as directory:
            plan=dict(kind='staged_test',hardware_executable=True,waist_assist=False,
                test_range={'first':4,'last':5},start=q,segments=segments,directory=directory)
            session.ready(ticket,plan)
            with patch('real_task_app.verify_test_plan'):
                if fail_second:
                    with self.assertRaisesRegex(RuntimeError,'rejected'):TaskNode.execute(node)
                    self.assertTrue((Path(directory)/'execution_failure.json').exists())
                else:
                    TaskNode.execute(node)
                    self.assertTrue((Path(directory)/'execution.json').exists())
        return node,sent

    def test_only_selected_range_runs_and_is_consumed(self):
        node,sent=self.run_range()
        self.assertEqual(len(sent),2)
        self.assertEqual(node.session.phase,'ARRIVED_HOLD')
        self.assertIsNone(node.session.plan)
        node.stop.assert_not_called()

    def test_rejection_stops_and_does_not_continue(self):
        node,sent=self.run_range(fail_second=True)
        self.assertEqual(len(sent),2)
        node.stop.assert_called_once()
        self.assertTrue(node.session.stopped)

    def test_later_range_can_preview_without_actual_start_match(self):
        t=[10.];session=Session(lambda:t[0]);q=dict.fromkeys(ARM_NAMES,.2)
        for _ in range(7):t[0]+=.1;session.update(q)
        session.capture_home();ticket=session.begin('staged_test','both',.3,.3)
        expected=dict(q);expected[ARM_NAMES[0]]+=.5
        session.ready(ticket,dict(kind='staged_test',hardware_executable=True,
            test_range={'first':3,'last':5},start=expected,segments=['test']))
        self.assertIsNone(session.expire_plan())
        self.assertEqual(session.phase,'PLAN_READY')
        self.assertIn('preview only',session.execution_issue())
        with self.assertRaises(RuntimeError):session.execution()


if __name__=='__main__':unittest.main()
