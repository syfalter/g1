import unittest
from task_session import Session, ARM_NAMES


class SessionTest(unittest.TestCase):
    def setUp(self):
        self.t = 10.
        self.s = Session(lambda: self.t)
        self.q = dict.fromkeys(ARM_NAMES, .2)
        for _ in range(6):
            self.s.update(self.q)
            self.t += .1
        self.s.capture_home()

    def begin(self):
        return self.s.begin('task', 'left', .5, .5)

    def plan(self):
        ticket = self.begin()
        self.s.ready(ticket, {'start': self.q, 'segments': ['trajectory']})
        return ticket

    def test_home_immutable_and_no_fingers(self):
        home = dict(self.s.home)
        self.s.stop()
        self.s.recovered_hold()
        self.assertEqual(self.s.home, home)
        with self.assertRaises(RuntimeError): self.s.capture_home()
        self.assertEqual(set(home), set(ARM_NAMES))

    def test_stop_rejects_late_plan_and_snapshot(self):
        ticket = self.begin()
        self.s.stop()
        with self.assertRaises(RuntimeError): self.s.ready(ticket, {'segments': [1]})
        with self.assertRaises(RuntimeError): self.s.accept_snapshot(ticket, {})
        self.assertIsNone(self.s.plan)

    def test_stop_allows_new_preview_but_blocks_execution(self):
        self.plan()
        self.s.stop()
        with self.assertRaises(RuntimeError): self.s.execution()
        for kind in ('prepare','task','recover'):
            ticket=self.s.begin(kind,'left',.5,.5)
            self.s.accept_snapshot(ticket,{'sequence':kind})
            self.s.ready(ticket,dict(kind=kind,start=self.q,segments=['trajectory']))
            self.assertTrue(self.s.stopped)
            self.assertIn('stopped',self.s.execution_issue())
            with self.assertRaisesRegex(RuntimeError,'stopped'):self.s.execution()
            self.s.phase='EXECUTION_PREFLIGHT'
            with self.assertRaisesRegex(RuntimeError,'stopped'):self.s.execution()
            self.s.phase='PLAN_READY'

    def test_stop_during_stopped_preview_cancels_new_ticket(self):
        self.s.stop();ticket=self.begin();self.s.stop()
        with self.assertRaisesRegex(RuntimeError,'cancelled'):self.s.check(ticket)
        with self.assertRaises(RuntimeError):self.s.accept_snapshot(ticket,{})
        with self.assertRaises(RuntimeError):self.s.ready(ticket,{'segments':[1]})

    def test_saved_authorization_still_requires_hold(self):
        self.s.stop()
        with self.assertRaisesRegex(RuntimeError,'hold'):self.s.begin('staged_test','left',.5,.5)

    def test_state_stream_does_not_refresh_scene(self):
        ticket = self.begin()
        self.s.accept_snapshot(ticket, {'sequence': 123})
        for _ in range(20):
            self.t += .01
            self.s.update(self.q)
        self.assertEqual(self.s.snapshot, {'sequence':123})

    def test_expired_state(self):
        self.plan()
        self.t += .2
        with self.assertRaises(RuntimeError): self.s.execution()

    def test_changed_state_invalidates_execution(self):
        self.plan()
        changed = {n:v+.05 for n,v in self.q.items()}
        for _ in range(12):
            self.t += .1
            self.s.update(changed)
        with self.assertRaises(RuntimeError): self.s.execution()
        self.assertIsNone(self.s.plan)

    def test_plan_requires_click_and_is_one_use(self):
        self.plan()
        self.s.execution()
        with self.assertRaises(RuntimeError): self.s.execution()

    def test_invalid_scaling(self):
        for value in (0., 1.01, float('nan')):
            with self.assertRaises(ValueError): self.s.begin('task','left',value,.5)

    def test_old_worker_cannot_overwrite_new_state(self):
        ticket = self.begin()
        self.s.stop()
        self.s.recovered_hold()
        current = self.begin()
        self.s.fail(ticket)
        self.assertEqual(self.s.ticket, current)

    def test_ready_rejects_observation_expired_during_planning(self):
        ticket = self.begin()
        self.s.accept_snapshot(ticket, {'sequence': 1})
        self.t += 121.
        with self.assertRaisesRegex(RuntimeError, 'expired during planning'):
            self.s.ready(ticket, {'start': self.q, 'segments': [1]})
        self.s.fail(ticket)
        self.assertEqual(self.s.phase, 'REPLAN_REQUIRED')
        self.assertIsNone(self.s.plan)

    def test_ui_expiry_clears_ready_and_plan(self):
        self.plan()
        self.t += 61.
        self.assertIn('expired', self.s.expire_plan())
        self.assertEqual(self.s.phase, 'REPLAN_REQUIRED')
        self.assertIsNone(self.s.plan)
        self.assertIsNone(self.s.expire_plan())

    def test_ui_observation_expiry_before_plan_expiry(self):
        ticket = self.begin()
        self.s.accept_snapshot(ticket, {'sequence': 1})
        self.t += 100.
        self.s.ready(ticket, {'start': self.q, 'segments': [1]})
        self.t += 21.
        self.assertIn('RGB-D expired', self.s.expire_plan())
        self.assertEqual(self.s.phase, 'REPLAN_REQUIRED')

    def test_execution_issue_is_read_only(self):
        ticket = self.plan()
        self.assertIsNone(self.s.execution_issue())
        self.assertEqual(self.s.ticket, ticket)
        self.assertEqual(self.s.phase, 'PLAN_READY')
        self.t += .2
        self.assertIn('stale', self.s.execution_issue())
        self.assertEqual(self.s.phase, 'PLAN_READY')

    def test_execution_expiry_does_not_leave_false_ready(self):
        self.plan()
        self.t += 61.
        self.s.update(self.q)
        for _ in range(6):
            self.t += .1
            self.s.update(self.q)
        with self.assertRaisesRegex(RuntimeError, 'expired'):
            self.s.execution()
        self.assertEqual(self.s.phase, 'REPLAN_REQUIRED')

    def test_no_ready_while_planning(self):
        self.begin()
        self.assertIn('PLANNING', self.s.execution_issue())

    def frozen(self):
        ticket=self.begin();self.s.accept_snapshot(ticket,{'sequence':1})
        return ticket,{'kind':'task','hardware_executable':False,'start':dict(self.q),'segments':[1]}

    def advance_with_feedback(self,seconds):
        for _ in range(round(seconds/.1)):self.t+=.1;self.s.update(self.q)
        for _ in range(7):self.t+=.1;self.s.update(self.q)

    def test_frozen_preview_survives_planning_and_review_age_but_cannot_execute(self):
        ticket,plan=self.frozen();self.advance_with_feedback(180.)
        self.s.ready(ticket,plan);self.advance_with_feedback(90.)
        self.assertIsNone(self.s.expire_plan())
        self.assertIn('Preview only',self.s.execution_issue())
        with self.assertRaisesRegex(RuntimeError,'Preview only'):self.s.execution()

    def test_motion_even_if_returned_invalidates_frozen_preview(self):
        ticket,plan=self.frozen()
        moved=dict(self.q);moved[ARM_NAMES[0]]+=.05
        self.s.update(moved);self.advance_with_feedback(1.)
        with self.assertRaisesRegex(RuntimeError,'moved since capture'):self.s.ready(ticket,plan)

    def test_frozen_preview_waits_through_short_gap_without_losing_plan(self):
        ticket,plan=self.frozen();self.s.ready(ticket,plan);self.t+=.2
        self.assertIsNone(self.s.expire_plan());self.assertIs(self.s.plan,plan)
        with self.assertRaisesRegex(RuntimeError,'stale'):self.s.measured()
        self.s.update(self.q)
        with self.assertRaisesRegex(RuntimeError,'window'):self.s.measured(stationary=True)
        self.assertIsNone(self.s.expire_plan())
        self.advance_with_feedback(.5)
        self.assertIsNone(self.s.expire_plan());self.assertIsNone(self.s.preview_issue(plan))

    def test_frozen_preview_retained_during_sustained_loss_but_not_executable(self):
        ticket,plan=self.frozen();self.s.ready(ticket,plan);self.t+=30.
        self.assertIsNone(self.s.expire_plan());self.assertIs(self.s.plan,plan)
        self.assertIn('stale',self.s.preview_issue(plan))

    def test_long_gap_recovered_before_ui_refresh_does_not_invalidate(self):
        ticket,plan=self.frozen();self.s.ready(ticket,plan);self.t+=1.6
        self.s.update(self.q);self.advance_with_feedback(.5)
        self.assertIsNone(self.s.expire_plan());self.assertIs(self.s.plan,plan)
        self.assertIsNone(self.s.preview_issue(plan))

    def test_completion_during_long_gap_retains_exact_plan_until_feedback_recovers(self):
        ticket,plan=self.frozen();self.t+=5.
        self.s.ready(ticket,plan)
        self.assertEqual(self.s.phase,'PLAN_READY');self.assertIs(self.s.plan,plan)
        self.assertIn('stale',self.s.preview_issue(plan))
        self.s.update(self.q)
        self.assertIn('window',self.s.preview_issue(plan))
        self.advance_with_feedback(.5)
        self.assertIsNone(self.s.preview_issue(plan));self.assertIs(self.s.plan,plan)

    def test_saved_test_retains_result_but_requires_fresh_stationary_execution(self):
        ticket=self.s.begin('staged_test','both',.15,.15)
        plan=dict(kind='staged_test',hardware_executable=True,start=dict(self.q),
                  segments=[1],test_range=dict(first=1,last=10))
        self.t+=5.;self.s.ready(ticket,plan)
        self.assertIsNone(self.s.expire_plan());self.assertIs(self.s.plan,plan)
        with self.assertRaisesRegex(RuntimeError,'stale'):self.s.execution()
        self.s.update(self.q)
        with self.assertRaisesRegex(RuntimeError,'window'):self.s.execution()
        self.advance_with_feedback(.5)
        self.assertIsNone(self.s.execution_issue())
        self.assertEqual(self.s.execution(),(ticket,plan))

    def test_movement_after_long_gap_still_invalidates_retained_plan(self):
        ticket,plan=self.frozen();self.t+=5.;self.s.ready(ticket,plan)
        moved=dict(self.q);moved[ARM_NAMES[0]]+=.04;self.s.update(moved)
        self.assertIn('moved',self.s.expire_plan());self.assertIsNone(self.s.plan)

    def test_stop_clears_plan_retained_during_feedback_loss(self):
        ticket,plan=self.frozen();self.t+=5.;self.s.ready(ticket,plan)
        self.s.stop();self.advance_with_feedback(.5)
        self.assertIsNone(self.s.plan)
        with self.assertRaises(RuntimeError):self.s.ready(ticket,plan)

    def test_movement_after_short_gap_still_invalidates_plan(self):
        ticket,plan=self.frozen();self.s.ready(ticket,plan);self.t+=.2
        moved=dict(self.q);moved[ARM_NAMES[0]]+=.04;self.s.update(moved)
        self.assertIn('moved',self.s.expire_plan());self.assertIsNone(self.s.plan)

    def test_frozen_preview_cannot_reuse_previous_generation_observation(self):
        old,plan=self.frozen();self.s.fail(old);ticket=self.begin();self.advance_with_feedback(121.)
        with self.assertRaisesRegex(RuntimeError,'expired during planning'):self.s.ready(ticket,plan)


if __name__ == '__main__': unittest.main()
