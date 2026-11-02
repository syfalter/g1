"""Compare a time-reversed gear lift with recorded lowering, without robot I/O."""
import argparse
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'deployment/rail_jetson'))
from automatic_recovery import reversed_path
from bundle import digest
from task_hand_control import model_joints
from task_composition_smoothing import evaluate
from task_offline_optimizer import decode, validate
from task_planning_config import parameters_from_values
from task_trajectory import time_seconds
from workbench_model import OfflineModel


def summarize(tr, model, context):
    duration=time_seconds(tr.points[-1])
    ts=np.linspace(0.,duration,int(np.ceil(duration/.005))+1)
    q=evaluate(tr,ts)
    frames=model.batch_transforms(['right_index_tcp'],context,tr.joint_names,q)['right_index_tcp']
    positions=frames[:,:3,3]
    distance=np.linalg.norm(np.diff(positions,axis=0),axis=1)
    speeds=distance/np.diff(ts)
    return dict(seconds=duration,path_mm=float(sum(distance)*1000),
        under_2_mm_s=float(sum(np.diff(ts)[speeds<.002])),
        under_5_mm_s=float(sum(np.diff(ts)[speeds<.005])),
        joint_travel_rad=dict(zip(tr.joint_names,np.sum(abs(np.diff(q,axis=0)),axis=0).tolist()))),frames,q


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--urdf',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    library=json.loads((ROOT/'data/rail_deployment/gear_protocol_20260928/gear_protocol.json').read_text())
    data=library['payload']
    release=json.loads((ROOT/'data/rail_deployment/approved_mirror_20260928/bundle.json').read_text())
    action=release['payload']['actions']['gear_roundtrip_lower_upper']
    snap=json.loads((ROOT/action['source']).read_text())['data']['variants']['flow']['payload']
    context=dict(snap['context'],**model_joints('right',action['program'][3]['target_q']))
    model=OfflineModel(args.urdf)
    params=parameters_from_values(action['plan']['planning_config']['values'])
    reverse=reversed_path(data['units']['lu']['trajectory'])
    original=data['units']['ul']['trajectory']
    before,pb,qb=summarize(decode(original),model,context)
    after,pa,qa=summarize(decode(reverse),model,context)
    distances,indices=cKDTree(pb[:,:3,3]).query(pa[:,:3,3])
    back=cKDTree(pa[:,:3,3]).query(pb[:,:3,3])[0]
    rotation=(Rotation.from_matrix(pb[indices,:3,:3]).inv()*Rotation.from_matrix(pa[:,:3,:3])).magnitude()
    result=dict(source_library_sha256=library['sha256'],reverse_sha256=digest(reverse),
        before=before,reverse=after,
        endpoint_max_joint_delta_rad=max(float(np.max(abs(np.array(reverse['points'][i]['positions'])-
            original['points'][i]['positions']))) for i in (0,-1)),
        sampled_bidirectional_distance_max_mm=float(max(max(distances),max(back))*1000),
        sampled_nearest_position_orientation_max_deg=float(np.rad2deg(max(rotation))),
        caveat='Nearest-position comparison is diagnostic, not a certified contact-path correspondence or collision proof.',
        reverse_validation=validate(decode(reverse),model,params,5.),
        physical_verified=False,hardware_commands_sent=0)
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
