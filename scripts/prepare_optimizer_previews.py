"""Pin existing verified batch artifacts for direct task-selector playback."""
import argparse
import json
from pathlib import Path

import numpy as np

from task_composition_preview import ROOT,materialize
from task_composition_preview import fit
from task_composition_panel import preview_samples
from task_composition_smoothing import evaluate
from task_optimizer_snapshot import save
from task_smooth_timing import smoothness
from task_trace_selection import restore
from task_trace_versions import bank
from task_screen_teach import fingerprint
from workbench_model import OfflineModel


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf',type=Path,required=True)
    parser.add_argument('--directory',type=Path,default=ROOT/'data/composition_candidates')
    parser.add_argument('--revision',type=int,choices=(4,5),default=4)
    args=parser.parse_args();model=OfflineModel(args.urdf)
    files=sorted(args.directory.glob(f'*-jerk5-contact1-transit5-v{args.revision}-extra0.15-evidence.json'))
    # Default startup shows the gear combination that was discussed with the user.
    files.sort(key=lambda p:p.name.startswith('landing_gear-combination'))
    for path in files:
        evidence=json.loads(path.read_text());workflow=evidence['workflow'];slot=evidence['slot']
        source=Path(evidence['source']);config=restore(json.loads(source.read_text()),source)
        if slot and bank(config,slot)['selected']!=evidence['selected_version']:continue
        variants={}
        if slot:
            from rosidl_runtime_py.convert import message_to_ordereddict
            current_source_sha=fingerprint(message_to_ordereddict(fit(config['traces'][slot],model,config.get('side','right'),lambda:None)))
        for name in ('original','baseline','candidate','smooth')+(('flow',) if args.revision==5 else ()):
            artifact=json.loads(path.with_name(path.name.replace('-evidence.json','-'+name+'.json')).read_text())
            payload=artifact['units'];sequence=artifact['sequence']
            if slot:
                if payload['unitEvidence'][slot]['source_sha256']!=current_source_sha:
                    raise ValueError('Batch artifact does not match selected recording: '+str(path))
                payload['sources']['selectedSha256']=fingerprint(config['traces'][slot])
            payload['options'].update(rate=1.,assembly=dict(count=1,start='LOWER',finish='UPPER',leave=True))
            tr,events=materialize(payload,sequence)
            side=config.get('side','right');times,tcp,joint=preview_samples(tr,payload,events,model,side)
            matrices=[evaluate(tr,times,k) for k in (1,2,3)]
            variants[name]=dict(payload=payload,sequence=sequence,trajectory=tr,events=events,times=times,
                tcp=tcp,joint=joint,acceleration=np.max(abs(matrices[1]),axis=1),jerk=np.max(abs(matrices[2]),axis=1),
                joint_names=list(tr.joint_names),joint_matrix=matrices[0],acceleration_matrix=matrices[1],
                jerk_matrix=matrices[2],smoothness=smoothness(tr))
        chosen='flow' if args.revision==5 else 'smooth';sources=variants[chosen]['payload']['sources']
        controls=dict(case=[workflow,slot],jerk=5.,contact=1.,transit=5.,rate=1.,smooth_first=True,extra=15.,
            count=1,start='LOWER',finish='UPPER',leave=True,manual=False,begin=0.,end=1.,continuous_flow=args.revision==5)
        if workflow=='dring_demo' and slot is None:
            controls.update({direction:[sources[direction]['slot'],sources[direction]['selectedVersion']]
                             for direction in ('left','right')})
        saved=save(variants,controls,model)
        print(json.dumps(dict(task=controls['case'],seconds=variants[chosen]['smoothness']['seconds'],
                              snapshot=str(saved)),ensure_ascii=False),flush=True)


if __name__=='__main__':main()
