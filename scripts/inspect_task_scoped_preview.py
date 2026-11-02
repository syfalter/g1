"""Sample a completed offline path for displayed tilt and scoped FCL clearances."""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from task_preview_playback import PreviewTimeline
from task_hook_collision import HandBaseCollision
from workbench_model import OfflineModel


def inspect(path):
    report=json.loads(path.read_text());side=report['arm'];root=path.parent
    start=json.loads((root/'reconstructed_state.json').read_text())['joints']
    timeline=PreviewTimeline(report,start);model=OfflineModel(root/'robot.urdf')
    checker=HandBaseCollision(model,root/'robot.urdf',report['scene']['base_top_pelvis_m'])
    minimum_base=(float('inf'),);minimum_body=(float('inf'),);tilts=[];rotations=[]
    for t in np.arange(0,timeline.duration+.05,.05):
        q,label=timeline.sample(t)
        base=checker.clearance(q,side);body=checker.arm_body_clearance(q,side)
        if base[0]<minimum_base[0]:minimum_base=(*base,float(t),label)
        if body[0]<minimum_body[0]:minimum_body=(*body,float(t),label)
        if label in ('Insert','Left 120','Return center','Right 120','Withdraw'):
            tip=model.transform(side+'_index_tcp',q)
            tilts.append(float(np.degrees(np.arccos(np.clip(-tip[2,2],-1,1)))))
            rotations.append(Rotation.from_matrix(tip[:3,:3]))
    result={'source':str(path),'sample_interval_s':.05,'sampled_base_minimum_m_link_time_phase':minimum_base,
        'sampled_arm_body_minimum_m_pair_time_phase':minimum_body,
        'contact_tilt_from_down_deg':[min(tilts),max(tilts)],
        'max_orientation_change_from_contact_start_deg':float(max(np.degrees((rotations[0].inv()*r).magnitude()) for r in rotations)),
        'duration_s':timeline.duration,'hardware_executable':False,
        'note':'Conservative per-link convex hulls; sampled clearance, not continuous collision certification. D-ring/table/full self collision not checked.'}
    (root/'scoped_metrics.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('report',type=Path)
    print(json.dumps(inspect(parser.parse_args().report),indent=2))
