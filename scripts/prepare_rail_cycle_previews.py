"""Generate pinned full rail cycles for both local preview and data export."""
import argparse
import json
from pathlib import Path

import numpy as np

from task_composition_preview import ROOT, materialize
from task_composition_panel import preview_samples
from task_composition_smoothing import evaluate
from task_optimizer_snapshot import save
from task_offline_optimizer import Policy
from task_planning_config import load_parameters
from task_rail_composition import comparison
from task_smooth_timing import smoothness
from task_trace_selection import restore
from workbench_model import OfflineModel


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf',type=Path,required=True)
    parser.add_argument('--source',type=Path,required=True)
    args=parser.parse_args()
    config=restore(json.loads(args.source.read_text()),args.source)
    ready=json.loads((ROOT/'data/task_panel/ready_poses.json').read_text())
    model=OfflineModel(args.urdf);parameters,_=load_parameters()
    for operation in ('push_cycle','pull_cycle'):
        options=dict(railCycle=operation,smoothFirst=True,continuousFlow=True,smoothExtraFraction=.15,rate=1.)
        payloads,_=comparison(config,ready,model,parameters,Policy(),options,progress=lambda text:print(text,flush=True))
        results={}
        for name,payload in payloads.items():
            tr,events=materialize(payload,payload['sequence'])
            times,tcp,joint=preview_samples(tr,payload,events,model,'right')
            matrices=[evaluate(tr,times,k) for k in (1,2,3)]
            results[name]=dict(payload=payload,sequence=payload['sequence'],trajectory=tr,events=events,
                times=times,tcp=tcp,joint=joint,joint_names=list(tr.joint_names),joint_matrix=matrices[0],
                acceleration_matrix=matrices[1],jerk_matrix=matrices[2],
                acceleration=np.max(abs(matrices[1]),axis=1),jerk=np.max(abs(matrices[2]),axis=1),smoothness=smoothness(tr))
        controls=dict(case=['rail',operation],jerk=5.,contact=1.,transit=5.,rate=1.,smooth_first=True,
            continuous_flow=True,extra=15.,count=1,start='LOWER',finish='UPPER',leave=True,
            manual=False,begin=0.,end=1.)
        path=save(results,controls,model)
        print(json.dumps(dict(operation=operation,snapshot=str(path),seconds={n:r['smoothness']['seconds'] for n,r in results.items()})),flush=True)


if __name__=='__main__':main()
