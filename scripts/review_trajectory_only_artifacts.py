"""Finalize immutable comparison catalogs and diagnostics, without robot I/O."""
import argparse
import copy
import shutil
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from trajectory_only_candidates import (ROOT, BASE, read, write_json, load, decode, checked,
    context_for, metrics, refresh, OfflineModel, parameters_from_values)
from automatic_recovery import reversed_path
from task_optimizer_snapshot import DIRECTORY, case_key


def catalog(paths, directory, latest_path):
    directory.mkdir(parents=True, exist_ok=True)
    old = read(directory/'catalog.json') if (directory/'catalog.json').exists() else {}
    updated = copy.deepcopy(old)
    for path in paths:
        saved = read(path)
        target = directory/path.name
        if target.exists() and target.read_bytes() != path.read_bytes():
            raise ValueError('Immutable preview already exists with different content')
        if target.resolve() != path.resolve():
            shutil.copyfile(path,target)
        updated[case_key(*saved['data']['controls']['case'])] = target.name
    if old != (read(directory/'catalog.json') if (directory/'catalog.json').exists() else {}):
        raise ValueError('Concurrent catalog edit detected; retry registration')
    write_json(directory/'catalog.json',updated)
    write_json(directory/'latest.json',dict(file=latest_path.name))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--staging',type=Path,required=True)
    parser.add_argument('--register-bounded',action='store_true')
    parser.add_argument('--refresh-bounded-metadata',action='store_true')
    args = parser.parse_args()
    stage = args.staging.resolve()
    model = OfflineModel(ROOT/'data/trajectory_edit_audit/fast_timing_all/panel_robot.urdf')
    base = read(BASE/'bundle.json')
    if args.refresh_bounded_metadata:
        for name in ('push','pull','gear_roundtrip_lower_upper'):
            report_path=stage/(name+'_report.json')
            evidence=read(report_path)
            old=load(ROOT/base['payload']['actions'][name]['source'],model)
            selected=load(ROOT/evidence['preview'],model)['variants']['flow']['payload']['units']
            changes={key:(decode(selected[key]['trajectory']),value) for key,value in evidence['units'].items()}
            path=refresh(old,changes,model,stage/'previews'/name,'trajectory-only-c2-ledger-v1')
            evidence['preview']=str(path.relative_to(ROOT))
            write_json(report_path,evidence)
    action = base['payload']['actions']['gear_roundtrip_lower_upper']
    saved = load(ROOT/action['source'],model)
    payload = saved['variants']['flow']['payload']
    context = context_for(payload,'ul')
    params = parameters_from_values(action['plan']['planning_config']['values'])
    source = decode(payload['units']['ul']['trajectory'])
    reverse = decode(reversed_path(payload['units']['lu']['trajectory']))
    before, a = metrics(source,model,context)
    after, b = metrics(reverse,model,context)
    distance, nearest = cKDTree(a[:,:3,3]).query(b[:,:3,3])
    back = cKDTree(b[:,:3,3]).query(a[:,:3,3])[0]
    angle = Rotation.from_matrix(a[nearest,:3,:3].transpose(0,2,1)@b[:,:3,:3]).magnitude()
    report = dict(before=before,reverse=after,derivative_validation=checked(reverse,model,params),
        sampled_bidirectional_distance_mm=float(max(max(distance),max(back))*1000),
        sampled_nearest_orientation_deg=float(np.rad2deg(max(angle))),
        endpoint_max_rad=max(float(np.max(abs(np.array(reverse.points[i].positions)-source.points[i].positions))) for i in (0,-1)),
        source_release_sha256=base['sha256'],publishable=False,physical_verified=False,collision_checked=False,
        contact_direction_constraints_known=False,hardware_commands_sent=0,
        diagnostic_only='Nearest-position correspondence is not collision, grasp or contact equivalence.',
        rejection_gates=['different contact path and wrist orientation',
                         'direction-dependent mechanism/grasp constraints unmeasured',
                         'requires explicit physical trial approval'])
    reverse_path = refresh(saved,{'ul':(reverse,report)},model,stage/'reverse_preview','reverse-lift-UNAPPROVED')
    report['preview'] = str(reverse_path.relative_to(ROOT))
    write_json(stage/'reverse_report.json',report)
    paths = [ROOT/read(stage/(name+'_report.json'))['preview'] for name in ('push','pull','gear_roundtrip_lower_upper')]
    catalog(paths,stage/'panel',paths[-1])
    catalog([*paths[:2],reverse_path],stage/'reverse_panel',reverse_path)
    depths = read(stage/'push_depth_report.json')
    for name,value in depths['candidates'].items():
        path = ROOT/value['preview']
        catalog([path,*paths[1:]],stage/'depth_panels'/name,path)
    if args.register_bounded:
        backup = stage/'panel_registration_before.json'
        if not backup.exists():
            write_json(backup,{name:read(DIRECTORY/name) if (DIRECTORY/name).exists() else None
                               for name in ('catalog.json','latest.json')})
        catalog(paths,DIRECTORY,paths[-1])
        write_json(stage/'panel_registration.json',dict(directory=str(DIRECTORY),
            registered=[p.name for p in paths],excluded=['reverse','depth'],hardware_commands_sent=0))
    print('bounded panel:',stage/'panel',flush=True)
    print('reverse diagnostic:',report['sampled_bidirectional_distance_mm'],report['sampled_nearest_orientation_deg'],flush=True)


if __name__ == '__main__':
    main()
