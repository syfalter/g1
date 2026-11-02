"""Source-clock and recovery state-machine simulations; no ROS initialization."""
import threading
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock, patch

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import JointState
from real_task_app import TaskNode
from task_session import Session, ARM_NAMES, FeedbackPending, RobotMoving


class SourceFeedbackTests(unittest.TestCase):
    def setUp(self):
        self.now=100.
        self.s=Session(lambda:self.now)
        self.q=dict.fromkeys(ARM_NAMES,.2)

    def sample(self,stamp=None,age=0.,delay=0.):
        source_now=round(self.now*1e9)
        stamp=source_now-round(age*1e9) if stamp is None else stamp
        return self.s.update(self.q,received_at=self.now-delay,
            source_stamp_ns=stamp,source_now_ns=source_now)

    def test_old_duplicate_reordered_missing_and_future_samples_do_not_refresh(self):
        self.assertTrue(self.sample())
        received=self.s.received;stamp=self.s.source_stamp_ns
        self.now+=.1
        for kwargs in (dict(stamp=stamp),dict(stamp=stamp-1),dict(stamp=0),
                       dict(age=.2),dict(age=-.1)):
            self.assertFalse(self.sample(**kwargs))
            self.assertEqual(self.s.received,received)
        self.assertEqual(self.s.rejected_feedback,5)
        self.now+=.1
        with self.assertRaises(FeedbackPending):self.s.measured()

    def test_source_age_and_lock_wait_both_count(self):
        self.assertFalse(self.sample(age=.12,delay=.04))
        self.assertTrue(self.sample(age=.10,delay=.01))
        self.assertAlmostEqual(self.now-self.s.received,.11)
        self.now+=.05
        with self.assertRaises(FeedbackPending):self.s.measured()

    def test_identical_positions_with_advancing_stamps_are_stationary(self):
        for _ in range(12):
            self.now+=.05;self.assertTrue(self.sample())
        self.assertEqual(self.s.measured(stationary=True),self.q)

    def test_gap_requires_new_continuous_window_not_old_buffer(self):
        for _ in range(12):self.now+=.05;self.sample()
        self.now+=.154;self.sample()
        with self.assertRaises(FeedbackPending):self.s.measured(stationary=True)
        for _ in range(9):self.now+=.05;self.sample()
        self.assertEqual(self.s.measured(stationary=True),self.q)

    def test_settling_uses_last_point_four_seconds_not_entire_one_second(self):
        for _ in range(12):self.now+=.05;self.sample()
        self.q[ARM_NAMES[0]]+=.1;self.now+=.01;self.sample()
        with self.assertRaises(RobotMoving):self.s.measured(stationary=True)
        for _ in range(10):self.now+=.05;self.sample()
        self.assertEqual(self.s.measured(stationary=True),self.q)


class RecoveryTests(unittest.TestCase):
    def setUp(self):
        self.now=100.;self.q=dict.fromkeys(ARM_NAMES,.2)
        self.s=Session(lambda:self.now)
        for _ in range(12):self.now+=.05;self.s.update(self.q)
        self.s.capture_home();self.s.stop()
        self.n=S(session=self.s,lock=threading.RLock(),hand_control=False,status='',
            recovery_pending=None,gateway_received=self.now,stop_expected_epoch=7,display_pub=Mock(),
            gateway=dict(phase='TASK_ADMITTANCE',task_stop_active='true',task_stop_epoch='7'),
            resume_hold=object(),launch_work=lambda f:f(),check=lambda ticket=None:None,
            get_clock=lambda:S(now=lambda:S(nanoseconds=round(self.now*1e9))))
        self.clock_patch=patch('real_task_app.time.monotonic',lambda:self.now)
        self.clock_patch.start();self.addCleanup(self.clock_patch.stop)

    def fresh_joints(self):
        m=JointState();m.name=list(self.q);m.position=list(self.q.values())
        ns=round(self.now*1e9);m.header.stamp.sec,m.header.stamp.nanosec=divmod(ns,1000000000)
        TaskNode.joints(self.n,m)

    def test_hold_ack_followed_by_763ms_gap_is_not_left_stopped(self):
        def call(*args):
            self.now+=.763
            self.n.gateway.update(phase='ARMED_HOLD',task_stop_active='false')
            self.n.gateway_received=self.now
            return S(success=True)
        self.n.call=call
        TaskNode.recovery(self.n,.15,.15)
        self.assertFalse(self.s.stopped)
        self.assertEqual(self.s.phase,'HOLD_AWAITING_FEEDBACK')
        with self.assertRaises(FeedbackPending):self.s.begin('task','left',.15,.15)
        for _ in range(12):self.now+=.05;self.fresh_joints()
        self.assertEqual(self.s.phase,'HOLD_FOR_RECOVERY')
        self.assertFalse(self.s.stopped)

    def test_new_stop_during_recovery_cannot_be_cleared(self):
        def call(*args):
            self.s.stop();self.n.gateway['task_stop_epoch']='8'
            return S(success=True)
        self.n.call=call
        with self.assertRaisesRegex(RuntimeError,'cancelled'):TaskNode.recovery(self.n,.15,.15)
        self.assertTrue(self.s.stopped)

    def test_lost_rpc_reply_can_be_reconciled_by_same_epoch_controller_ack(self):
        self.n.call=Mock(side_effect=TimeoutError('reply lost'))
        with self.assertRaises(TimeoutError):TaskNode.recovery(self.n,.15,.15)
        msg=DiagnosticArray(status=[DiagnosticStatus(values=[
            KeyValue(key=k,value=v) for k,v in dict(phase='ARMED_HOLD',task_stop_active='false',
                                                   task_stop_epoch='7').items()])])
        TaskNode.diagnostics(self.n,msg)
        self.assertFalse(self.s.stopped)
        self.assertEqual(self.s.phase,'HOLD_AWAITING_FEEDBACK')
        self.n.call.assert_called_once()

    def test_source_replay_through_real_callback_is_rejected(self):
        self.fresh_joints();received=self.s.received
        m=JointState();m.name=list(self.q);m.position=list(self.q.values());m.header.stamp.sec=1
        self.now+=.1;TaskNode.joints(self.n,m)
        self.assertEqual(self.s.received,received)
        self.assertIsNotNone(self.s.feedback_rejection)

    def diagnostic(self,epoch=7,stopped=True):
        msg=DiagnosticArray(status=[DiagnosticStatus(values=[KeyValue(key=k,value=v) for k,v in
            dict(phase='LATCHED_SILENT' if stopped else 'ARMED_HOLD',
                 task_stop_active='true' if stopped else 'false',task_stop_epoch=str(epoch)).items()])])
        TaskNode.diagnostics(self.n,msg)

    def test_repeated_stop_diagnostics_preserve_new_planning_and_result(self):
        ticket=self.s.begin('task','left',.15,.15)
        for _ in range(3):self.diagnostic()
        self.assertEqual(self.s.phase,'PLANNING')
        self.s.accept_snapshot(ticket,{'sequence':1})
        plan=dict(kind='task',hardware_executable=False,start=self.q,segments=[1])
        self.s.ready(ticket,plan)
        for _ in range(3):self.diagnostic()
        self.assertEqual(self.s.phase,'PLAN_READY');self.assertIs(self.s.plan,plan)
        self.assertTrue(self.s.stopped)
        self.n.display_pub.publish.assert_not_called()

    def test_new_stop_epoch_cancels_planning_ready_and_execution(self):
        for phase in ('PLANNING','PLAN_READY','EXECUTION_PREFLIGHT','EXECUTING'):
            for stopped in (True,False):
                with self.subTest(phase=phase,stopped=stopped):
                    self.s.phase='IDLE';self.n.stop_expected_epoch=7
                    ticket=self.s.begin('task','left',.15,.15)
                    self.s.phase=phase
                    self.diagnostic(8,stopped)
                    with self.assertRaisesRegex(RuntimeError,'cancelled'):self.s.check(ticket)
                    self.assertIsNone(self.s.plan);self.assertTrue(self.s.stopped)
                    self.assertEqual(self.n.stop_expected_epoch,8)

    def test_own_stop_ack_does_not_cancel_a_new_post_stop_preview(self):
        self.n.stop_expected_epoch=8
        ticket=self.s.begin('task','left',.15,.15)
        self.diagnostic(7);self.diagnostic(8)
        self.s.check(ticket);self.assertEqual(self.s.phase,'PLANNING')

    def test_same_epoch_stop_transition_cancels_non_stopped_work(self):
        self.s.stopped=False;ticket=self.s.begin('task','left',.15,.15)
        self.diagnostic()
        with self.assertRaisesRegex(RuntimeError,'cancelled'):self.s.check(ticket)

    def test_repeated_stopped_diagnostic_preserves_new_home_capture_phase(self):
        self.s.home=None;self.s.capture_home();self.diagnostic()
        self.assertEqual(self.s.phase,'HOME_CAPTURED')


if __name__=='__main__':unittest.main()
