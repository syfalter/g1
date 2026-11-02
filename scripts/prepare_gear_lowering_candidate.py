"""Keep the reviewed lowering path budget; compress quiet intervals offline."""
import argparse
import json
from pathlib import Path
import sys

from rosidl_runtime_py.convert import message_to_ordereddict

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'deployment/rail_jetson'))
from bundle import digest
from task_offline_optimizer import decode, validate, Policy
from task_planning_config import parameters_from_values
from task_stop_blending import blend_internal_stops, stop_audit
from task_fast_timing import local_candidate
from task_trajectory import time_seconds
from workbench_model import OfflineModel


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--urdf',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--fit-order',type=int,choices=(5,7),default=7)
    args=p.parse_args()
    bundle=json.loads((ROOT/'data/rail_deployment/approved_mirror_20260928/bundle.json').read_text())
    action=bundle['payload']['actions']['gear_roundtrip_lower_upper']
    snapshot=json.loads((ROOT/action['source']).read_text())
    variants=snapshot['data']['variants']
    baseline=decode(variants['baseline']['payload']['units']['ul']['trajectory'])
    current=decode(variants['flow']['payload']['units']['ul']['trajectory'])
    model=OfflineModel(args.urdf)
    parameters=parameters_from_values(action['plan']['planning_config']['values'])
    policy=Policy()
    progress=lambda text:print(text,flush=True)
    trial,evidence=blend_internal_stops(baseline,model,'right',parameters,5.,
        policy.limits('operation')*.99,progress=progress,compress_idle=True,fit_order=args.fit_order)
    choices=[('current',current,{}),('bounded_quiet_compression',trial,evidence)]
    selected=min(choices,key=lambda item:time_seconds(item[1].points[-1]))
    report=dict(source_snapshot_sha256=snapshot['sha256'],
        original_seconds=time_seconds(current.points[-1]),
        selected_method=selected[0],selected_seconds=time_seconds(selected[1].points[-1]),
        trajectory=message_to_ordereddict(selected[1]),evidence=selected[2],
        stop_audit=stop_audit(selected[1]),peaks=validate(selected[1],model,parameters,5.),
        source_trajectory_sha256=digest(message_to_ordereddict(current)),
        physical_verified=False,collision_checked=False,hardware_commands_sent=0,
        candidates=[dict(method=n,seconds=time_seconds(t.points[-1])) for n,t,_ in choices])
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:report[k] for k in ('selected_method','original_seconds','selected_seconds')}),flush=True)


if __name__=='__main__':main()
