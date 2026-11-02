"""Hardware-independent session and stale-work barriers for the real task console."""
from dataclasses import dataclass
import copy
import math
import time

ARM_SUFFIXES = ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow',
                'wrist_roll', 'wrist_pitch', 'wrist_yaw')
ARMS = {side: tuple(f'{side}_{name}_joint' for name in ARM_SUFFIXES)
        for side in ('left', 'right')}
ARM_NAMES = ARMS['left'] + ARMS['right']
STATE_MAX_AGE = .15
STATIONARY_WINDOW = .4


class FeedbackPending(RuntimeError):
    """Wait for fresh observations without treating an existing plan as invalid."""


class RobotMoving(RuntimeError):
    """Fresh observations exist, but the stationary window is not settled."""


@dataclass(frozen=True)
class Ticket:
    generation: int
    kind: str
    side: str
    velocity: float
    acceleration: float


class Session:
    """Caller serializes operations; all worker results require the same generation."""
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.generation = 0
        self.home = None
        self.current = {}
        self.received = float('-inf')
        self.history = []
        self.phase = 'WAIT_STATE'
        self.plan = None
        self.ticket = None
        self.snapshot = None
        self.snapshot_generation = None
        self.snapshot_joints = {}
        self.snapshot_motion = False
        self.stopped = False
        self.source_stamp_ns = None
        self.callback_received = float('-inf')
        self.rejected_feedback = 0
        self.feedback_rejection = None

    def update(self, joints, *, received_at=None, source_stamp_ns=None, source_now_ns=None):
        if not all(name in joints and math.isfinite(joints[name]) for name in ARM_NAMES):
            raise ValueError('Missing/nonfinite measured arm joint')
        now = self.clock()
        received_at = now if received_at is None else received_at
        observed_at = received_at
        reason = None
        if not math.isfinite(received_at) or received_at > now+.001:
            reason = 'Invalid feedback receive clock'
        if source_stamp_ns is not None or source_now_ns is not None:
            if (type(source_stamp_ns) is not int or type(source_now_ns) is not int or
                    source_stamp_ns <= 0):
                reason = 'Missing/invalid joint-state source stamp'
            elif self.source_stamp_ns is not None and source_stamp_ns <= self.source_stamp_ns:
                reason = 'Duplicate or out-of-order joint-state source stamp'
            else:
                source_age = (source_now_ns-source_stamp_ns)*1e-9
                if source_age < -.02:
                    reason = 'Joint-state source stamp is in the future; check ROS clock'
                observed_at -= max(0.,source_age)
        if now-observed_at > STATE_MAX_AGE:
            reason = 'Joint-state sample was already stale on receipt/lock acquisition'
        if reason:
            self.rejected_feedback += 1
            self.feedback_rejection = reason
            return False
        gap = now-self.received
        if gap > STATE_MAX_AGE:
            self.history = []
        self.current = dict(joints)
        if self.snapshot_generation == self.generation:
            tolerance = .01 if self.ticket and self.ticket.kind == 'screen_capture' else .025
            self.snapshot_motion = self.snapshot_motion or any(
                n not in joints or abs(joints[n]-v)>tolerance for n,v in self.snapshot_joints.items())
        self.received = observed_at
        self.callback_received = received_at
        self.source_stamp_ns = source_stamp_ns
        self.feedback_rejection = None
        self.history = [item for item in self.history if self.received-item[0] <= 1.] + [
            (self.received, dict(joints))]
        return True

    def measured(self, stationary=False):
        age = self.clock()-self.received
        if age > STATE_MAX_AGE:
            raise FeedbackPending(f'Real robot state is stale or unavailable (age {age:.3f}s; limit {STATE_MAX_AGE:.2f}s)')
        if stationary:
            if not self.history or self.received-self.history[0][0] < STATIONARY_WINDOW-1e-9:
                raise FeedbackPending('Waiting for a continuous stationary observation window')
            cutoff=self.received-STATIONARY_WINDOW
            first=max(i for i,(stamp,_) in enumerate(self.history) if stamp<=cutoff+1e-9)
            window=self.history[first:]
            watched=ARM_NAMES+(('waist_yaw_joint',) if 'waist_yaw_joint' in self.current else ())
            if any(any(n not in q for _,q in window) or
                   max(q[n] for _, q in window)-min(q[n] for _, q in window) > .012
                   for n in watched):
                raise RobotMoving('Arms are still moving')
        return dict(self.current)

    def capture_home(self):
        if self.home is not None:
            raise RuntimeError('Session startup pose is immutable; restart for a new R2+A session')
        q = self.measured(stationary=True)
        self.home = {n: q[n] for n in ARM_NAMES}
        if 'waist_yaw_joint' in q:self.home['waist_yaw_joint']=q['waist_yaw_joint']
        self.phase = 'HOME_CAPTURED'
        return dict(self.home)

    def invalidate(self):
        self.generation += 1
        self.plan = None
        self.ticket = None

    def begin(self, kind, side, velocity, acceleration):
        self.require_planning_ready()
        if self.stopped and kind == 'staged_test':
            raise RuntimeError('Saved-range authorization requires controller hold')
        if kind not in ('prepare', 'task', 'recover', 'staged_test', 'screen', 'screen_capture', 'rail', 'dring_demo') or side not in ('left','right','both'):
            raise ValueError('Invalid task selection')
        if not all(math.isfinite(v) and .05 <= v <= 1. for v in (velocity, acceleration)):
            raise ValueError('Scaling must be in [0.05,1.0]')
        self.invalidate()
        self.ticket = Ticket(self.generation, kind, side, velocity, acceleration)
        self.phase = 'PLANNING'
        self.planning_at = self.clock()
        if kind in ('screen', 'screen_capture', 'rail', 'dring_demo'):
            # Taught endpoint plans never inherit an old D-ring RGB-D expiry.
            self.snapshot = None
            self.snapshot_generation = self.generation
            self.snapshot_joints = {n:v for n,v in self.current.items() if n in ARM_NAMES or n=='waist_yaw_joint'}
            self.snapshot_motion = False
        if kind == 'staged_test':
            # Archived pixels keep their original capture date. This is a new
            # operator-confirmed test, with a measured-state movement barrier.
            self.snapshot = None
            self.snapshot_generation = self.generation
            self.snapshot_joints = {n:v for n,v in self.current.items() if n in ARM_NAMES or n=='waist_yaw_joint'}
            self.snapshot_motion = False
        # Retain old pixels visually until a new complete pair arrives, never as a new plan input.
        return self.ticket

    def require_planning_ready(self):
        if self.home is None:
            raise RuntimeError('Confirm and capture the current session startup/return point first')
        if self.phase in ('PLANNING', 'EXECUTION_PREFLIGHT', 'EXECUTING', 'RAIL_RECORDING', 'DEMO_RECORDING', 'SHUTTING_DOWN'):
            raise RuntimeError('An operation is already active: '+self.phase)
        self.measured(stationary=True)

    def planning_issue(self):
        try:
            self.require_planning_ready()
        except RuntimeError as error:
            return str(error)
        return None

    def check(self, ticket):
        # STOP cancels existing tickets by generation, not future preview work.
        if ticket is None or self.ticket != ticket or ticket.generation != self.generation:
            raise RuntimeError('Operation cancelled or superseded')

    def accept_snapshot(self, ticket, snapshot):
        self.check(ticket)
        if self.phase != 'PLANNING':
            raise RuntimeError('Snapshots only update during an explicit planning request')
        self.snapshot = copy.deepcopy(snapshot)
        self.snapshot_at = self.clock()
        self.snapshot_generation = ticket.generation
        self.snapshot_joints = {n:v for n,v in self.current.items() if n in ARM_NAMES or n=='waist_yaw_joint'}
        self.snapshot_motion = False

    def frozen_preview(self,plan):
        return (plan is not None and plan.get('kind')=='task' and plan.get('hardware_executable') is False and
                self.snapshot is not None and self.snapshot_generation==self.generation)

    def saved_test(self, plan):
        return (plan is not None and plan.get('kind') == 'staged_test' and
                plan.get('hardware_executable') is True and bool(plan.get('test_range')))

    def preview_issue(self,plan):
        try:q=self.measured(stationary=True)
        except RuntimeError as error:return str(error)
        if self.saved_test(plan):
            if self.snapshot_motion:
                return 'Robot moved since saved-range review; reload the range'
            if any(n not in q or abs(q[n]-v)>.025 for n,v in plan['start'].items()
                   if n in ARM_NAMES or n=='waist_yaw_joint'):
                return 'Selected range start differs from actual state; preview only, no automatic repositioning'
            return None
        if self.snapshot_motion or any(n not in q or abs(q[n]-v)>.025 for n,v in
                                      plan['start'].items() if n in ARM_NAMES or n=='waist_yaw_joint'):
            return 'Robot moved since capture; request a new observation and plan'
        return None

    def ready(self, ticket, plan):
        self.check(ticket)
        if not plan or not plan.get('segments'):
            raise RuntimeError('No complete trajectory')
        if self.saved_test(plan):
            if self.snapshot_motion:raise RuntimeError('Robot moved during saved-range review; reload')
        elif plan.get('kind') in ('screen', 'rail', 'dring_demo') and self.snapshot_motion:
            raise RuntimeError('Robot moved during endpoint planning; verify the start and replan')
        elif self.frozen_preview(plan):
            if self.snapshot_motion:
                raise RuntimeError('Robot moved since capture; request a new observation and plan')
            # A completed trajectory is retained while feedback recovers. This
            # does not authorize execution, which rechecks current observations.
            try:self.measured(stationary=True)
            except FeedbackPending:pass
            else:
                issue=self.preview_issue(plan)
                if issue:raise RuntimeError(issue)
        elif self.snapshot is not None and self.clock()-self.snapshot_at > 120.:
            raise RuntimeError('RGB-D expired during planning; no executable preview was published')
        self.plan = plan
        self.phase = 'PLAN_READY'
        self.ready_at = self.clock()

    def execution_issue(self):
        """Read-only eligibility check for the UI; execution still checks again."""
        if self.stopped:
            return 'Controller is stopped; planning is available, execution requires hold'
        if self.phase != 'PLAN_READY' or self.plan is None:
            return 'No complete validated plan ('+self.phase+')'
        if self.plan.get('hardware_executable') is False:
            return 'Preview only; hardware execution is not commissioned'
        try:
            self.check(self.ticket)
            q = self.measured(stationary=True)
        except RuntimeError as error:
            return str(error)
        if self.saved_test(self.plan):
            return self.preview_issue(self.plan)
        if self.plan.get('kind') in ('screen', 'rail', 'dring_demo') and self.snapshot_motion:
            return 'Robot moved since endpoint planning; replan even if it returned to the start'
        if self.snapshot is not None and self.clock()-self.snapshot_at > 120.:
            return 'RGB-D expired; request a fresh observation and plan'
        if self.clock()-self.ready_at > 60. and self.plan.get('kind')!='dring_demo':
            return 'Plan expired; replan'
        watched = list(ARM_NAMES)+(['waist_yaw_joint'] if 'waist_yaw_joint' in self.plan['start'] else [])
        if any(abs(q[n]-self.plan['start'][n]) > .025 for n in watched):
            return 'Robot moved since planning; replan from measured state'
        return None

    def expire_plan(self):
        if self.phase != 'PLAN_READY' or self.plan is None:
            return None
        reason = None
        if self.plan.get('kind') in ('screen', 'rail', 'dring_demo') and self.snapshot_motion:
            reason = 'Robot moved since endpoint planning; replan'
        if self.saved_test(self.plan) or self.frozen_preview(self.plan):
            if self.snapshot_motion:
                reason = 'Robot moved since capture/review; replan or reload the range'
            else:
                try:self.measured(stationary=True)
                except FeedbackPending:return None
                except RuntimeError as error:reason=str(error)
        if reason is None and self.saved_test(self.plan):
            try:self.measured(stationary=True)
            except RuntimeError as error:reason=str(error)
            if self.snapshot_motion:reason='Robot moved since saved-range review; reload the range'
        elif reason is None and self.frozen_preview(self.plan):
            reason=self.preview_issue(self.plan)
        elif reason is None and self.snapshot is not None and self.clock()-self.snapshot_at > 120.:
            reason = 'RGB-D expired; request a fresh observation and plan'
        elif reason is None and self.clock()-self.ready_at > 60. and self.plan.get('kind')!='dring_demo':
            reason = 'Plan expired; replan'
        if reason:
            self.invalidate()
            self.phase = 'REPLAN_REQUIRED'
        return reason

    def execution(self, candidate=None):
        if self.stopped:
            raise RuntimeError('Controller is stopped; execution requires hold')
        if self.phase not in ('PLAN_READY','EXECUTION_PREFLIGHT') or self.plan is None:
            raise RuntimeError('No confirmed complete plan')
        plan = self.plan if candidate is None else candidate
        if plan.get('hardware_executable') is False:
            raise RuntimeError('Preview only; hardware execution is not commissioned')
        self.check(self.ticket)
        q = self.measured(stationary=True)
        if plan.get('kind') in ('screen', 'rail') and self.snapshot_motion:
            raise RuntimeError('Robot moved since endpoint planning; replan')
        if self.saved_test(plan):
            issue = self.preview_issue(plan)
            if issue:
                self.invalidate()
                self.phase = 'REPLAN_REQUIRED'
                raise RuntimeError(issue)
            self.plan = plan
            self.phase = 'EXECUTING'
            return self.ticket, self.plan
        if self.snapshot is not None and self.clock()-self.snapshot_at > 120.:
            self.invalidate()
            self.phase = 'REPLAN_REQUIRED'
            raise RuntimeError('RGB-D observation expired; recapture and replan')
        if self.clock()-self.ready_at > 60.:
            self.invalidate()
            self.phase = 'REPLAN_REQUIRED'
            raise RuntimeError('Plan expired; request a new observation and plan')
        watched = list(ARM_NAMES)+(['waist_yaw_joint'] if 'waist_yaw_joint' in self.plan['start'] else [])
        if any(abs(q[n]-self.plan['start'][n]) > .025 for n in watched):
            self.invalidate()
            self.phase = 'REPLAN_REQUIRED'
            raise RuntimeError('Robot moved since planning; replan from measured state')
        self.phase = 'EXECUTING'
        return self.ticket, self.plan

    def stop(self):
        self.invalidate()
        self.stopped = True
        self.phase = 'STOP_REQUESTED'

    def recovered_hold(self, *, acknowledged=False):
        if not acknowledged:self.measured(stationary=True)
        self.invalidate()
        self.stopped = False
        self.phase = 'HOLD_AWAITING_FEEDBACK' if acknowledged else 'HOLD_FOR_RECOVERY'

    def fail(self, ticket):
        if self.ticket == ticket:
            self.invalidate()
            self.phase = 'REPLAN_REQUIRED'
