"""Opt-in command completion without claiming physical finger arrival or contact."""
import math

from task_hand_control import checked_q


def transition(hand, side, target, check, *, require_target=True, duration=2., timeout=8.,
               on_feedback=lambda state:None, on_event=lambda name, **details:None, finger_sequence=None):
    if type(require_target) is not bool:
        raise ValueError('Hand target requirement must be boolean')
    if finger_sequence not in (None, 'four_then_thumb', 'thumb_then_four'):
        raise ValueError('Unknown hand finger sequence')
    if finger_sequence and (require_target or duration != 1.):
        raise ValueError('Staged fingers require command completion and two 0.5 second stages')
    if require_target:
        return hand.transition(side, target, check, duration=duration, timeout=timeout,
                               on_feedback=on_feedback, on_event=on_event)
    target = checked_q(target)
    if not math.isfinite(duration) or not 1 <= duration <= 10:
        raise ValueError('Hand transition duration out of range')
    began = hand.clock(); evidence = []; confirmed = 0
    try:
        first = hand.read(side, check)
        check(); on_event('initial_feedback', state=first, target_required=False)
        start = checked_q(first['feedback_q'])
        ramp_started = hand.clock(); advanced_at = ramp_started
        last_stamp = first['timestamp_ms']
        while hand.clock()-began < timeout:
            t = min(1., (hand.clock()-ramp_started)/duration)
            check()
            early, late = min(1., t*2), max(0., t*2-1)
            progress = ([early]*4 + [late]*2 if finger_sequence == 'four_then_thumb' else
                        [late]*4 + [early]*2 if finger_sequence == 'thumb_then_four' else [t]*6)
            blends = [min(1., max(0., u*u*u*(10+u*(-15+6*u)))) for u in progress]
            q = list(target) if t == 1 else [a+blend*(b-a) for a,b,blend in zip(start, target, blends)]
            on_event('target_requested', target_q=q)
            hand.rpc('set', side, q, check)
            on_event('target_acknowledged', target_q=q)
            state = hand.read(side, check)
            on_feedback(state); on_event('feedback', state=state)
            evidence.append(state)
            if state['timestamp_ms'] > last_stamp:
                last_stamp = state['timestamp_ms']; advanced_at = hand.clock()
                confirmed = confirmed+1 if t == 1 else 0
            elif hand.clock()-advanced_at > .5:
                raise RuntimeError('Hand feedback is not advancing')
            if confirmed >= 3:
                # Acknowledged commands plus healthy fresh telemetry are not grasp verification.
                hand.rpc('hold', side, check=check)
                on_event('hold_acknowledged', target_required=False)
                return evidence
            hand.sleep(.05)
        raise TimeoutError('Hand command/feedback confirmation timed out; arm must not proceed')
    except Exception as error:
        error.hand_evidence = evidence
        try:
            hand.rpc('hold', side)
            error.hand_hold_confirmed = True
            on_event('hold_acknowledged_after_failure')
        except Exception as hold_error:
            error.hand_hold_confirmed = False
            error.hand_hold_error = str(hold_error)
        raise
