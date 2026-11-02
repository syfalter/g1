"""Build a local, deduplicated RGB-D corpus; no ROS, SSH, or robot commands."""
import argparse
from collections import Counter
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import shutil

import cv2
import numpy as np

from vision_rgbd import validate_pair

ROOT=Path(__file__).resolve().parents[1]
ARTIFACTS=('capture.json','capture.log','capture_state.json','geometry.json','manifest.json',
           'server_result.json','intrinsics.json','cloud.npz','annotated.jpg','contact_scene.json',
           'plan.json','planning_failure.json','execution.json','execution_failure.json')


def read(path):
    return json.loads(path.read_text())


def digest(path):
    with path.open('rb') as stream:
        result=hashlib.sha256()
        for block in iter(lambda:stream.read(1024*1024),b''):result.update(block)
    return result.hexdigest()


def write(path,value):
    temporary=path.with_name(path.name+'.tmp')
    temporary.write_text(json.dumps(value,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
    temporary.replace(path)


def copy(source,destination):
    destination.parent.mkdir(parents=True,exist_ok=True)
    if destination.exists():
        if digest(source)!=digest(destination):raise ValueError('Archive conflict: '+str(destination))
        return
    temporary=destination.with_name(destination.name+'.tmp')
    shutil.copy2(source,temporary)
    temporary.replace(destination)


def inspect(path):
    record=read(path)
    capture=record['capture'];capture_id=capture['id']
    if not isinstance(capture_id,str) or Path(capture_id).name!=capture_id:
        raise ValueError('Invalid capture identity')
    validate_pair(record,record)
    for key,suffix in (('color_image','_color.png'),('depth_image','_depth.png')):
        if Path(capture[key]).name!=capture_id+suffix:raise ValueError('Image capture identity differs')
    canonical=path.name=='server_result.json'
    color=path.parent/('rgb.png' if canonical else Path(capture['color_image']).name)
    depth=path.parent/('depth_aligned.png' if canonical else Path(capture['depth_image']).name)
    bgr=cv2.imread(str(color),cv2.IMREAD_UNCHANGED)
    z=cv2.imread(str(depth),cv2.IMREAD_UNCHANGED)
    if bgr is None or z is None:raise ValueError('Missing or undecodable image')
    if z.dtype!=np.uint16 or z.ndim!=2 or bgr.dtype!=np.uint8 or bgr.shape!=(*z.shape,3):
        raise ValueError('Expected matching uint8 RGB and uint16 aligned depth')
    intr=record['color_intrinsics']
    if (intr['height'],intr['width'])!=z.shape:raise ValueError('Intrinsics/image dimensions differ')
    if (record['image_height'],record['image_width'])!=z.shape:raise ValueError('Record/image dimensions differ')
    values=np.asarray([intr[k] for k in ('fx','fy','cx','cy')],float)
    if not np.isfinite(values).all() or np.any(values[:2]<=0):raise ValueError('Invalid intrinsics')
    scale=float(record['depth_scale_m_per_unit'])
    if not np.isfinite(scale) or not 0<scale<.01:raise ValueError('Invalid depth scale')
    delta=abs(float(record['color_timestamp_ms'])-float(record['depth_timestamp_ms']))
    if delta>50 or abs(delta-float(record['timestamp_delta_ms']))>.01:
        raise ValueError('Invalid or inconsistent frame time difference')
    positive=z[z>0]
    if not positive.size:raise ValueError('Depth is entirely zero')
    hashes={'rgb.png':digest(color),'depth_aligned.png':digest(depth)}
    manifest=path.parent/'manifest.json'
    if canonical and manifest.exists():
        expected=read(manifest).get('sha256',{})
        for name,actual in dict(hashes,**{'server_result.json':digest(path)}).items():
            if name in expected and expected[name]!=actual:raise ValueError('Manifest checksum mismatch: '+name)
    envelope=path.parent/'capture.json'
    if canonical and envelope.exists():
        observation=read(envelope).get('observation')
        if observation is not None:validate_pair(observation,record)
    calibration={'camera_serial':record['camera_serial'],'color_intrinsics':intr,
                 'depth_scale_m_per_unit':scale,'depth_to_color_extrinsics':record['depth_to_color_extrinsics']}
    pair_id=hashlib.sha256((hashes['rgb.png']+hashes['depth_aligned.png']).encode()).hexdigest()
    stats={'depth_nonzero_fraction':float(positive.size/z.size),
           'depth_nonzero_p01_p50_p99_m':(np.percentile(positive,[1,50,99])*scale).tolist(),
           'width':z.shape[1],'height':z.shape[0],'timestamp_delta_ms':delta}
    bbox=record.get('pedal_centered')
    if isinstance(bbox,list) and len(bbox)==4:
        x0,y0,x1,y1=map(int,bbox)
        roi=z[max(0,y0):min(z.shape[0],y1),max(0,x0):min(z.shape[1],x1)]
        if roi.size:stats['target_depth_nonzero_fraction']=float(np.count_nonzero(roi)/roi.size)
    return record,color,depth,hashes,calibration,pair_id,stats


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--remote-mirror',type=Path,required=True)
    parser.add_argument('--local-root',type=Path,default=ROOT/'data')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    output=args.output.resolve();remote=args.remote_mirror.resolve();local=args.local_root.resolve()
    output.mkdir(parents=True,exist_ok=True)
    records=set(remote.rglob('*_result.json'))
    records.update(p for p in local.rglob('server_result.json') if output not in p.resolve().parents)
    records.update(p for p in local.rglob('*_result.json')
                   if output not in p.resolve().parents and p.name.startswith('head_rgbd_'))
    pairs={};rejected=[];cases=[];referenced=set()
    for index,path in enumerate(sorted(records),1):
        try:
            record,color,depth,hashes,calibration,key,stats=inspect(path)
            referenced.update((color.resolve(),depth.resolve()))
            pair=pairs.get(key)
            if pair is None:
                directory=output/'pairs'/key
                copy(color,directory/'rgb.png');copy(depth,directory/'depth_aligned.png')
                copy(path,directory/'server_result.json')
                write(directory/'intrinsics.json',calibration)
                pair={'id':key,'directory':str(directory.relative_to(output)),
                      'sha256':hashes,'calibration':calibration,'quality':stats,
                      'valid_2d':record.get('valid_2d') is True,'valid_3d':record.get('valid_3d') is True,
                      'sources':[],'capture_ids':[]}
                pairs[key]=pair
            else:
                old=pair['calibration']
                # JSON serializers differ in final floating digits, not in the calibrated camera.
                if old['camera_serial']!=calibration['camera_serial'] or not np.allclose(
                        [old['depth_scale_m_per_unit']]+[old['color_intrinsics'][k] for k in ('fx','fy','cx','cy')],
                        [calibration['depth_scale_m_per_unit']]+[calibration['color_intrinsics'][k] for k in ('fx','fy','cx','cy')],
                        atol=1e-9,rtol=0):raise ValueError('Identical images with conflicting calibration')
            capture_id=record['capture']['id']
            if capture_id not in pair['capture_ids']:pair['capture_ids'].append(capture_id)
            pair['valid_2d']|=record.get('valid_2d') is True
            pair['valid_3d']|=record.get('valid_3d') is True
            source={'record':str(path),'capture_id':capture_id,'recorded_at':record['capture'].get('recorded_at'),
                    'sequence':record['sequence'],'valid_2d':record.get('valid_2d'),'valid_3d':record.get('valid_3d')}
            pair['sources'].append(source)
            if path.name=='server_result.json' and path.is_relative_to(local):
                context=output/'contexts'/path.parent.relative_to(local)
                for name in ARTIFACTS:
                    if (path.parent/name).is_file():copy(path.parent/name,context/name)
                for name in ('rgb.png','depth_aligned.png'):
                    target=context/name
                    if not target.exists():os.link(output/pair['directory']/name,target)
                for name in ('session_home.json','ready_poses.json','index_tcp.json'):
                    if (path.parent.parent/name).is_file():copy(path.parent.parent/name,context.parent/name)
                geometry=read(context/'geometry.json') if (context/'geometry.json').exists() else {}
                integrity=[]
                if (context/'manifest.json').exists():
                    for name,expected in read(context/'manifest.json').get('sha256',{}).items():
                        artifact=context/name
                        if not artifact.resolve().is_relative_to(context.resolve()):
                            integrity.append('unsafe_manifest_path:'+name)
                        elif not artifact.is_file():integrity.append('missing:'+name)
                        elif digest(artifact)!=expected:integrity.append('checksum:'+name)
                replay=(bool(geometry.get('geometry_valid_for_planning_seed')) and
                        not integrity and
                        all((context/name).is_file() for name in
                            ('capture_state.json','cloud.npz','manifest.json','capture.json')))
                failure=read(context/'planning_failure.json') if (context/'planning_failure.json').exists() else None
                plan=read(context/'plan.json') if (context/'plan.json').exists() else {}
                direction=plan.get('sweep_direction')
                if not direction and failure:
                    labels={c.get('label') for c in failure.get('search_diagnostics',{}).get('cartesian',[])}
                    directions=[d for d in ('left','right') if d.title()+' 120' in labels]
                    if len(directions)==1:direction=directions[0]
                case={'pair_id':key,'source_directory':str(path.parent),'directory':str(context.relative_to(output)),
                      'planning_replay_candidate':replay,'planning_failed':failure is not None,
                      'planning_succeeded':(context/'plan.json').exists(),
                      'context_integrity_issues':integrity,
                      'recorded_kind':plan.get('kind',(failure or {}).get('kind')),
                      'recorded_arm':plan.get('arm',(failure or {}).get('side')),'recorded_direction':direction,
                      'failure_message':failure.get('message') if failure else None,
                      'failure_diagnostic':failure.get('diagnostic') if failure else None}
                cases.append(case);source['context']=case['directory']
        except (OSError,ValueError,KeyError,TypeError,cv2.error) as error:
            rejected.append({'record':str(path),'reason':str(error)})
        if index%100==0:print(f'Checked {index}/{len(records)} records; {len(pairs)} unique pairs; {len(rejected)} rejected',flush=True)
    for pair in pairs.values():write(output/pair['directory']/'pair.json',pair)
    def jsonl(name,rows):
        temporary=output/(name+'.tmp')
        with temporary.open('w') as stream:
            for row in rows:stream.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n')
        temporary.replace(output/name)
    jsonl('pairs.jsonl',pairs.values());jsonl('cases.jsonl',cases)
    jsonl('planning_failures.jsonl',(c for c in cases if c['planning_failed']))
    jsonl('replay_candidates.jsonl',(c for c in cases if c['planning_replay_candidate']))
    jsonl('rejected.jsonl',rejected)
    images=set(remote.glob('*_color.png'))|set(remote.glob('*_depth.png'))
    write(output/'unpaired_or_rejected_remote_images.json',sorted(str(p) for p in images-referenced))
    inventory=[{'path':str(p.relative_to(remote)),'bytes':p.stat().st_size,'sha256':digest(p)}
               for p in sorted(remote.rglob('*')) if p.is_file()]
    write(output/'remote_inventory.json',inventory)
    copy(ROOT/'config/task_planning.yaml',output/'task_planning_snapshot.yaml')
    summary={'created_at':datetime.now().astimezone().isoformat(),'remote_mirror':str(remote),
             'records_checked':len(records),'unique_valid_pairs':len(pairs),'rejected_records':len(rejected),
             'pairs_with_valid_2d_detection':sum(p['valid_2d'] for p in pairs.values()),
             'pairs_with_valid_3d_detection':sum(p['valid_3d'] for p in pairs.values()),
             'local_contexts':len(cases),'planning_replay_candidates':sum(c['planning_replay_candidate'] for c in cases),
             'unique_replay_pairs':len({c['pair_id'] for c in cases if c['planning_replay_candidate']}),
             'contexts_with_integrity_issues':sum(bool(c['context_integrity_issues']) for c in cases),
             'planning_failure_contexts':sum(c['planning_failed'] for c in cases),
             'unique_planning_failure_pairs':len({c['pair_id'] for c in cases if c['planning_failed']}),
             'remote_files':len(inventory),'remote_bytes':sum(p['bytes'] for p in inventory),
             'rejection_reasons':dict(Counter(p['reason'] for p in rejected)),
             'notes':['uint16 depth is aligned to RGB, not a visualization or raw depth-camera grid',
                      'Pair validity does not imply target detection or planning success',
                      'Planning replay candidates have required sidecars and matching manifest hashes; no planner was executed',
                      'Timestamps are retained verbatim; old camera clock dates may be incorrect',
                      'No geometry, calibration, depth pixels or robot services were modified']}
    write(output/'summary.json',summary)
    print(json.dumps(summary,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
