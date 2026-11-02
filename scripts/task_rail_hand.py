"""Overlap a fixed hand transition with feedback-confirmed rail arm playback."""
import math
import threading
import time


class MotionClock:
    def __init__(self, *, clock=None):
        self.clock=clock or time.monotonic
        self.lock=threading.Lock()
        self.started=None
        self.updated=None

    def feedback(self, message):
        feedback=message.feedback
        with self.lock:
            self.updated=self.clock()
            desired=feedback.desired.velocities
            actual=feedback.actual.velocities
            if (self.started is None and
                    any(math.isfinite(v) and abs(v)>1e-6 for v in desired) and
                    any(math.isfinite(v) and abs(v)>.001 for v in actual)):
                self.started=self.updated

    def snapshot(self):
        with self.lock:return self.started,self.updated


def replay(hand,side,spec,future,motion,check,on_feedback,on_event,*,
           timeout,clock=None,sleep=None):
    clock=clock or time.monotonic
    sleep=sleep or time.sleep
    began=clock()
    def guard():
        check()
        if clock()-began>timeout:raise TimeoutError('Rail arm/hand playback timed out')
        if future.done():
            result=future.result()
            if result.status!=4 or result.result.error_code!=0:
                raise RuntimeError('Arm execution failed during delayed hand transition')
        else:
            _,updated=motion.snapshot()
            if clock()-(began if updated is None else updated)>1.5:
                raise RuntimeError('Rail arm motion feedback stopped; holding hand')
    while True:
        guard()
        if future.done():
            raise RuntimeError('B→A ended before delayed grip could start')
        started,_=motion.snapshot()
        if started is not None and clock()-started>=spec['delay_s']:break
        sleep(.01)
    on_event('delayed_hand_start',motion_elapsed_s=clock()-started,target_q=spec['target_q'])
    evidence=hand.transition(side,spec['target_q'],guard,duration=spec['duration'],
        on_feedback=on_feedback,on_event=on_event)
    while not future.done():
        guard();sleep(.01)
    guard()
    return future.result(),evidence
