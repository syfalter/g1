"""Compare archived and offline approach geometry using the same local robot model."""
import argparse
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from task_hand_control import POSTURES,model_joints
from task_trajectory import samples
from workbench_model import OfflineModel


def namespace(value):
    if isinstance(value,dict):return SimpleNamespace(**{k:namespace(v) for k,v in value.items()})
    if isinstance(value,list):return [namespace(v) for v in value]
    return value


def metrics(model,record,state,side):
    segment=next(s for s in record['segments'] if s['label']=='Approach')
    trajectory=namespace(segment['trajectory'])
    positions=np.asarray(list(samples(trajectory,10.,100.)))
    state=dict(state);state.update(model_joints(side,POSTURES['index_only']))
    frames=model.batch_transforms([side+'_index_tcp',side+'_elbow_link',side+'_shoulder_pitch_link'],
        state,trajectory.joint_names,positions)
    tips=frames[side+'_index_tcp'][:,:3,3]
    elbow=frames[side+'_elbow_link'][:,:3,3]
    shoulder=frames[side+'_shoulder_pitch_link'][:,:3,3]
    last=trajectory.points[-1].time_from_start
    return dict(duration_s=last.sec+last.nanosec/1e9,
        tip_path_length_m=float(np.linalg.norm(np.diff(tips,axis=0),axis=1).sum()),
        max_elbow_z_m=float(elbow[:,2].max()),max_elbow_above_shoulder_m=float((elbow-shoulder)[:,2].max()),
        elbow_vertical_range_m=[float(elbow[:,2].min()),float(elbow[:,2].max())],
        start_tip_m=tips[0].tolist(),end_tip_m=tips[-1].tolist())


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--replay',type=Path,required=True)
    parser.add_argument('--old-plan',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    new=json.loads(args.replay.read_text());old=json.loads(args.old_plan.read_text())
    model=OfflineModel(args.replay.parent/'robot.urdf')
    state=json.loads((args.replay.parent/'reconstructed_state.json').read_text())['joints']
    result=dict(replay=str(args.replay),old_plan=str(args.old_plan),arm=new['arm'],
        old=metrics(model,old,old['start'],new['arm']),new=metrics(model,new,state,new['arm']),
        limitation='Model FK of planned trajectories, not observed physical fingertip or contact stiffness.')
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
