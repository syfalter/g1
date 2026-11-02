import unittest
from types import SimpleNamespace as S
from task_rail_hand import MotionClock, replay


class DelayedHandTests(unittest.TestCase):
    def run_replay(self,*,stop_at=None,fail_at=None,end_at=3.,feedback=True,settle_error=False):
        self.now=0.;self.commands=[];self.events=[]
        motion=MotionClock(clock=lambda:self.now)
        def sleep(dt):
            self.now+=dt
            if feedback:
                moving=self.now>=.2
                motion.feedback(S(feedback=S(desired=S(velocities=[.1 if moving else 0.]),
                                             actual=S(velocities=[.1 if moving else 0.]))))
        def check():
            if stop_at is not None and self.now>=stop_at:raise RuntimeError('STOP')
        def transition(side,target,guard,**kwargs):
            self.commands.append(('start',self.now,kwargs['duration']))
            until=self.now+kwargs['duration']
            while self.now<until:
                guard();self.commands.append(('set',self.now));sleep(.02)
            if settle_error:raise TimeoutError('not settled')
            return ['settled']
        result=S(status=6 if fail_at is not None else 4,result=S(error_code=-1 if fail_at is not None else 0))
        future=S(done=lambda:self.now>=(fail_at if fail_at is not None else end_at),result=lambda:result)
        return replay(S(transition=transition),'right',dict(delay_s=.5,duration=1.,target_q=[.3]*6),
            future,motion,check,lambda state:None,lambda name,**kw:self.events.append((name,kw)),
            timeout=10.,clock=lambda:self.now,sleep=sleep)

    def test_delays_from_motion_feedback_and_overlaps_arm(self):
        _,evidence=self.run_replay()
        self.assertEqual(evidence,['settled'])
        self.assertAlmostEqual(self.commands[0][1],.7,places=2)
        self.assertEqual(self.commands[0][2],1.)
        self.assertLess(self.commands[-1][1],3.)

    def test_stop_before_delay_never_sends_hand_target(self):
        with self.assertRaisesRegex(RuntimeError,'STOP'):self.run_replay(stop_at=.4)
        self.assertEqual(self.commands,[])

    def test_stop_during_ramp_stops_targets(self):
        with self.assertRaisesRegex(RuntimeError,'STOP'):self.run_replay(stop_at=.9)
        self.assertLess(self.commands[-1][1],.9)

    def test_arm_failure_interrupts_ramp(self):
        with self.assertRaisesRegex(RuntimeError,'Arm execution failed'):self.run_replay(fail_at=.9)
        self.assertLess(self.commands[-1][1],.9)

    def test_early_completion_never_closes_at_rest(self):
        with self.assertRaisesRegex(RuntimeError,'ended before'):self.run_replay(end_at=.4)
        self.assertEqual(self.commands,[])

    def test_lost_feedback_never_sends(self):
        with self.assertRaisesRegex(RuntimeError,'feedback stopped'):self.run_replay(feedback=False)
        self.assertEqual(self.commands,[])

    def test_hand_failure_propagates(self):
        with self.assertRaisesRegex(TimeoutError,'not settled'):self.run_replay(settle_error=True)


if __name__=='__main__':unittest.main()
