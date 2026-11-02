"""Recorded finger targets on the accepted arm goal's monotonic playback clock."""
import time

import numpy as np

from task_hand_control import require_state


def replay(hand,side,timeline,future,check,on_feedback,*,clock=time.monotonic,sleep=time.sleep):
    times=np.asarray(timeline['time_s']);positions=np.asarray(timeline['q']);began=clock()
    duration=float(times[-1]);last_stamp=None;advanced=began;next_read=began;evidence=[]
    def guard():
        check()
        if clock()-advanced>.75:raise RuntimeError('回放手指反馈超过 750 ms 未推进')
        if future.done():
            result=future.result()
            if result.status!=4 or result.result.error_code!=0:raise RuntimeError('手臂回放失败，停止手指回放')
    while not future.done():
        guard();t=min(duration,max(0.,clock()-began))
        target=[float(np.interp(t,times,positions[:,i])) for i in range(6)]
        hand.rpc('set',side,target,guard)
        if clock()>=next_read:
            state=require_state(hand.read(side,guard),side);on_feedback(state)
            if last_stamp is None or state['timestamp_ms']>last_stamp:
                last_stamp=state['timestamp_ms'];advanced=clock()
            evidence.append(dict(time_s=clock()-began,feedback_q=list(state['feedback_q'])))
            next_read=clock()+.1
        if clock()-began>2*duration+30:raise TimeoutError('录制回放动作超时')
        sleep(.05)
    guard()
    if clock()-began<duration-.15:raise RuntimeError('手臂过早结束，未完成手指时间线')
    deadline=clock()+3.;stable=0
    while clock()<deadline:
        guard();hand.rpc('set',side,positions[-1].tolist(),guard)
        state=require_state(hand.read(side,guard),side);on_feedback(state)
        if last_stamp is None or state['timestamp_ms']>last_stamp:
            last_stamp=state['timestamp_ms'];advanced=clock()
            stable=stable+1 if np.max(np.abs(np.asarray(state['feedback_q'])-positions[-1]))<=.03 else 0
        evidence.append(dict(time_s=clock()-began,feedback_q=list(state['feedback_q'])))
        if stable>=2:
            hand.rpc('hold',side,check=check)
            return future.result(),evidence
        sleep(.05)
    raise TimeoutError('回放结束后手指未到记录终点')
