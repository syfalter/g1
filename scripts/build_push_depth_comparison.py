"""Freeze exit-adjustable depth previews for the standard offline editor."""
import argparse
import copy
from pathlib import Path
import shutil

from trajectory_only_candidates import (ROOT, BASE, read, load, decode, encode, checked,
    context_for, metrics, refresh, write_json, OfflineModel, parameters_from_values)
from trajectory_push_depth import compensation
from task_optimizer_snapshot import save, DIRECTORY


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'data/rail_deployment/push_depth_exit_v2_20260929')
    args = parser.parse_args()
    output=args.output.resolve()
    model=OfflineModel(ROOT/'data/trajectory_edit_audit/fast_timing_all/panel_robot.urdf')
    base=read(BASE/'bundle.json')
    action=base['payload']['actions']['push']
    saved=load(ROOT/action['source'],model)
    payload=saved['variants']['flow']['payload']
    source=decode(payload['units']['motion_0']['trajectory'])
    context=context_for(payload,'motion_0')
    parameters=parameters_from_values(action['plan']['planning_config']['values'])
    prior=read(ROOT/'data/rail_deployment/trajectory_only_20260929/push_depth_report.json')
    peak=prior['before']['peak_time_s']
    report=dict(source_release_sha256=base['sha256'],source_snapshot=action['source'],
        exit_geometry_change_user_approved=True,physical_axis_confirmed=False,
        axis_root=prior['axis_root'],deployed=False,hardware_commands_sent=0,candidates={})
    results={}
    for mm,key in ((0,'original'),(3,'baseline'),(4,'candidate')):
        trials=[]
        for tail in ((1.5,) if mm==0 else (1.5,1.75,2.,2.5,3.)):
            support=tuple(round(t*1e9)/1e9 for t in (peak-2.5,peak-.2,peak,peak+tail))
            print('DEPTH',mm,'EXIT',tail,flush=True)
            tr,evidence=compensation(source,model,context,prior['axis_root'],float(mm),support)
            try:
                validation=checked(tr,model,parameters)
            except ValueError as exc:
                trials.append(dict(exit_transition_s=tail,rejected=str(exc)))
                continue
            trials.append(dict(exit_transition_s=tail,passed=True,jerk=validation['jerk']))
            evidence.update(motion_limits_passed=True,derivative_validation=validation,
                metrics=metrics(tr,model,context)[0],trials=trials,publishable=False,
                exit_geometry_change_user_approved=True,physical_axis_confirmed=False,
                physical_verified=False,hardware_executable=False,
                pending_checks=['physical rail axis','mechanical end-stop clearance'],
                comparison_reference='installed frozen push, not historical pre-optimization path')
            path=refresh(saved,{'motion_0':(tr,evidence)},model,output/'individual'/str(mm),
                         'push-depth-exit-v2-'+str(mm)+'mm')
            result=load(path,model)['variants']['flow']
            result['payload']['comparisonLabel']=('0 mm / frozen baseline' if mm==0 else str(mm)+' mm / exit blend')
            result['payload']['optimizer']['variant']=key
            result['payload']['spatialComparison']=dict(focus='maximum_root_x',sampling_mm=.05,
                reference_frame='URDF root',amplitude_mm=mm,axis_root=prior['axis_root'],
                axis_measured=False,exit_transition_s=tail if mm else 0.)
            results[key]=result
            evidence['snapshot']=str(path.relative_to(ROOT))
            report['candidates'][str(mm)]=evidence
            break
        else:
            write_json(output/'report.json',report)
            raise ValueError('No derivative-valid depth candidate: '+str(mm))
    path=save(results,copy.deepcopy(saved['snapshot_controls']),model,directory=output/'comparison')
    load(path,model)
    target=DIRECTORY/path.name
    shutil.copyfile(path,target)
    manifest_path=DIRECTORY/'experiments.json'
    manifest=read(manifest_path) if manifest_path.exists() else {}
    entries=manifest.setdefault('rail:push_cycle',[])
    entries[:]=[v for v in entries if v['id']!='push-depth-exit-v2']
    entries.append(dict(id='push-depth-exit-v2',label='推进补深 0 / 3 / 4 mm',file=path.name,
                        sha256=read(path)['sha256']))
    write_json(manifest_path,manifest)
    write_json(output/'previous_latest.json',read(DIRECTORY/'latest.json'))
    write_json(DIRECTORY/'latest.json',dict(file=path.name))
    report['comparison_snapshot']=str(target.relative_to(ROOT))
    write_json(output/'report.json',report)
    print('COMPARISON',target,flush=True)


if __name__=='__main__':main()
