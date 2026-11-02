"""Export a complete production offline replay through the existing archive gate."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import shutil
import tempfile
import xml.etree.ElementTree as ET

from task_dring_sequence import SCHEMA, labels
from task_plan_archive import digest, freeze, stages, write_json
from task_model_resources import resolve_mesh
from task_runtime_provenance import runtime_matches


def snapshot_meshes(urdf, destination):
    resources=[]
    destination.mkdir()
    for reference in sorted({mesh.attrib['filename'] for mesh in ET.parse(urdf).iter('mesh')}):
        source=resolve_mesh(reference,urdf)
        checksum=digest(source)
        saved=destination/(checksum+source.suffix)
        if not saved.exists():shutil.copy2(source,saved)
        resources.append(dict(reference=reference,sha256=checksum,
                              snapshot='model_resources/'+saved.name))
    return resources


def export_archive(capture, destination, robot_urdf, report, start, check=lambda:None):
    """No state synthesis, retiming, authorization, or hardware access."""
    from task_saved_test import load_archive
    capture,destination,robot_urdf=map(Path,(capture,destination,robot_urdf))
    if (not report.get('success') or report.get('mode')!='PRODUCTION_SCOPED_OFFLINE'
            or report.get('hardware_executable') is not False
            or report.get('hardware_commands_sent')!=0):
        raise ValueError('Only complete, non-executable production offline results may be exported')
    if tuple(s['label'] for s in report['segments'])!=labels(report['sweep_direction']):
        raise ValueError('Incomplete D-ring sequence')
    for key in ('ready_source','startup_heading_source'):
        if not report.get(key) or 'fallback' in report[key]:
            raise ValueError('Historical ready/home required for reusable archive')
    runtime=report.get('runtime_sha256')
    if runtime is not None and not runtime_matches(runtime):
        raise ValueError('Native runtime changed since replay; export rejected')
    config=copy.deepcopy(report['planning_config'])
    config['values']=copy.deepcopy(config['effective_values'])
    plan=dict(kind='task',arm=report['arm'],start=start,segments=report['segments'],
              task_schema=SCHEMA,sweep_direction=report['sweep_direction'],
              velocity_scaling=report['velocity_scaling'],acceleration_scaling=report['acceleration_scaling'],
              waist_assist=report['waist_assist'],planning_profile='steep_straight',
              planning_config=config,hardware_executable=False,hardware_executed=False,
              hand_posture='archived_capture_model',hand_geometry_calibrated=False,
              contact_scene=report['scene'],search_diagnostics=report['search_diagnostics'])
    stages(plan)
    check()
    destination.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.offline-export-',dir=destination.parent) as temporary:
        root=Path(temporary)
        source=root/'plan'
        if any(p.is_symlink() for p in capture.rglob('*')):
            raise ValueError('Capture symlinks are not allowed')
        shutil.copytree(capture,source)
        for name in ('session_home.json','ready_poses.json','index_tcp.json'):
            path=capture.parent/name
            if path.is_file():shutil.copy2(path,root/name)
        resources=snapshot_meshes(robot_urdf,source/'model_resources')
        image_hashes={name:digest(capture/name) for name in ('rgb.png','depth_aligned.png')}
        write_json(source/'plan.json',plan)
        write_json(source/'offline_validation.json',report)
        write_json(source/'offline_provenance.json',dict(
            source_capture=str(capture.resolve()),source_model_sha256=digest(robot_urdf),
            pair_id=hashlib.sha256((image_hashes['rgb.png']+image_hashes['depth_aligned.png']).encode()).hexdigest(),
            source_sha256={p.name:digest(p) for p in capture.iterdir() if p.is_file()},
            mesh_resources=resources,hardware_commands_sent=0,
            runtime_sha256=runtime,
            runtime_snapshot_scope=report.get('runtime_snapshot_scope','not recorded in source replay'),
            scene_identity='historical capture; operator must confirm matching physical scene',
            code_sha256={p.name:digest(p) for p in Path(__file__).parent.iterdir()
                         if p.is_file() and p.suffix in ('.py','.sh','.cpp','.so')}))
        check()
        freeze(source,destination,robot_urdf,operator_scene_unchanged_commitment=False)
    check()
    loaded=load_archive(destination,robot_urdf)
    check()
    return dict(directory=str(destination.resolve()),reload_verified=True,
                original_plan_sha256=loaded['manifest']['original_plan_sha256'],
                stage_count=len(loaded['stages']),hardware_executable=False)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--replay',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    report=json.loads((args.replay/'report.json').read_text())
    report['export_mode']='posthoc; not eligible for original case deadline credit'
    start=json.loads((args.replay/'reconstructed_state.json').read_text())['joints']
    result=export_archive(Path(report['capture']),args.output,args.replay/'robot.urdf',report,start)
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
