"""Pure offline timeline sampling; never sends joint commands."""
from bisect import bisect_right
from types import SimpleNamespace as S
import numpy as np
from task_trajectory import coefficients


class PreviewTimeline:
    def __init__(self,report,start):
        if (report.get('mode') not in ('KINEMATIC_PREVIEW_UNCHECKED','HOOK_PREVIEW_BASE_ONLY','STRAIGHT_PREVIEW_SCOPED','ARCHIVE_SCOPED_OFFLINE') or not report.get('offline_only')
                or report.get('hardware_executable') is not False or not report.get('success')):
            raise ValueError('Only completed, explicitly offline preview reports can be played')
        self.parts=[];self.ends=[];state=dict(start);elapsed=0.
        for segment in report['segments']:
            trajectory=segment['trajectory'];names=trajectory['joint_names'];points=trajectory['points']
            times=np.array([p['time_from_start']['sec']+p['time_from_start']['nanosec']*1e-9 for p in points])
            if len(times)<2 or times[0]!=0 or np.any(np.diff(times)<=0):
                raise ValueError('Invalid preview timeline')
            if any(n not in state for n in names):raise ValueError('Incomplete preview start state')
            if max(abs(state[n]-points[0]['positions'][i]) for i,n in enumerate(names))>.025:
                raise ValueError('Discontinuous preview segment start')
            curves=[coefficients(S(**a),S(**b),dt) for a,b,dt in zip(points,points[1:],np.diff(times))]
            self.parts.append((elapsed,dict(state),names,times,curves,segment['label']))
            elapsed+=float(times[-1]);self.ends.append(elapsed)
            state.update(zip(names,points[-1]['positions']))
        if not self.parts:raise ValueError('Empty preview')
        self.duration=elapsed;self.end=state

    def sample(self,elapsed):
        if elapsed>=self.duration:return dict(self.end),self.parts[-1][-1]
        index=bisect_right(self.ends,max(0.,elapsed))
        begin,state,names,times,curves,label=self.parts[index]
        local=max(0.,elapsed-begin)
        point=min(len(curves)-1,max(0,int(np.searchsorted(times,local,side='right'))-1))
        fraction=(local-times[point])/(times[point+1]-times[point])
        result=dict(state)
        result.update(zip(names,map(float,np.polynomial.polynomial.polyval(fraction,curves[point]))))
        return result,label
