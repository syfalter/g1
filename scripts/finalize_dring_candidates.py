"""Audit all offline exports, preserve evidence, and install the ten-slot bundle."""
import copy
import csv
from dataclasses import replace
import hashlib
import json
import shutil

import numpy as np
from rosidl_runtime_py.set_message import set_message_fields
from trajectory_msgs.msg import JointTrajectory

from prepare_dring_candidates import ARCHIVE,ROOT
from task_dring_cache import cache_key
from task_dring_teach_data import SLOTS,arrays
from task_planning_config import load_parameters
from task_screen_teach import fingerprint,write_json,now
from task_trajectory import coefficients,time_seconds,peak,stretch_to_limits,task_jerk_limits,validate_waist_envelope
from task_trajectory_speed import motion_limits
from workbench_model import OfflineModel


def main():
    bundle=json.loads((ARCHIVE/'candidates/demonstration.json').read_text())
    original=json.loads((ARCHIVE/'dring_demonstration.json').read_text())
    assert set(bundle['traces'])==set(SLOTS) and len(SLOTS)==10
    assert bundle['traces']['left']==original['traces']['left_3']
    assert bundle['traces']['right']==original['traces']['right']
    assert bundle['ready']==original['ready']
    model=OfflineModel(ARCHIVE/'robot.urdf');current,_=load_parameters()
    params=replace(current,waist_velocity_rad_s=min(current.waist_velocity_rad_s,.285))
    baseline=copy.deepcopy(bundle);baseline['traces']={k:v for k,v in bundle['traces'].items() if k in ('left','right')}
    write_json(ARCHIVE/'baselines.json',baseline)
    audit=[]
    for direction,name in (('left','dring_demo_0093'),('right','dring_demo_0020')):
        saved=json.loads((ARCHIVE/name/'plan.json').read_text())
        execution=json.loads((ARCHIVE/name/'execution.json').read_text())
        assert all(row['error_code']==0 for row in execution) and len(execution)==len(saved['segments'])
        saved_tr=next(s['trajectory'] for s in saved['segments'] if s.get('recording_fit'))
        results=json.loads((ARCHIVE/'candidates'/f'{direction}_validation.json').read_text())
        assert len(results)==10
        for item in results:
            slot=item['slot'];multiplier=item['multiplier'];trace=bundle['traces'][slot]
            q,t=arrays(trace,model,bundle['side'])
            plan=json.loads((ARCHIVE/'candidates'/f'{slot}_{multiplier:g}x/plan.json').read_text())
            motion=next(s for s in plan['segments'] if s.get('recording_fit'))
            tr=JointTrajectory();set_message_fields(tr,copy.deepcopy(motion['trajectory']))
            np.testing.assert_allclose(tr.points[0].positions,q[0],atol=1e-12,rtol=0.)
            np.testing.assert_allclose(tr.points[-1].positions,q[-1],atol=1e-12,rtol=0.)
            vmax,amax=motion_limits(tr,model,.7,.7,params)
            assert stretch_to_limits(copy.deepcopy(tr),vmax,amax,task_jerk_limits(tr.joint_names))==1.
            lo,hi=model.limits['waist_yaw_joint']
            validate_waist_envelope(tr,lo+.02,hi-.02,vmax['waist_yaw_joint'],amax['waist_yaw_joint'])
            for a,b in zip(tr.points,tr.points[1:]):
                c=coefficients(a,b,time_seconds(b)-time_seconds(a))
                for axis,joint in enumerate(tr.joint_names):
                    low,high=model.limits[joint];mid=(low+high)/2;radius=(high-low)/2-.02
                    centered=c[:,axis].copy();centered[0]-=mid
                    assert peak(centered)<=radius+1e-9
            for p in (tr.points[0],tr.points[-1]):assert max(abs(v) for v in (*p.velocities,*p.accelerations))<1e-8
            exact=slot==direction and motion['trajectory']==saved_tr
            if slot==direction and multiplier==1.5:assert exact
            item['physical_tested']=exact;item['exact_saved_timing']=exact
            item['baseline_geometry_unchanged']=slot==direction
            item.pop('baseline_unchanged',None)
            for p in (params,current):
                key=cache_key(trace,model,bundle['side'],multiplier,.7,.7,p)
                cache=ROOT/'data/task_panel/dring_replay_cache'/(key+'.json')
                cache_record=json.loads(cache.read_text())
                assert fingerprint(cache_record['payload'])==cache_record['sha256']
                destination=ARCHIVE/'replay_cache'/cache.name;destination.parent.mkdir(exist_ok=True)
                shutil.copy2(cache,destination)
            audit.append(item)
        write_json(ARCHIVE/'candidates'/f'{direction}_validation.json',results)
        events=[json.loads(line) for line in next((ARCHIVE/name).glob('execution_events*.jsonl')).read_text().splitlines()]
        for event in events:
            if event['checkpoint']!='arm_accepted':continue
            stamp=event['source_stamp_ns']*1e-9
            matches=[]
            for path in (ROOT/'logs/endpoint_motion').glob('endpoint_motion_20260920_15*.csv'):
                with path.open() as stream:row=next(csv.DictReader(stream))
                if abs(float(row['ros_time_s'])-stamp)<.2:matches.append(path)
            if len(matches)!=1:raise ValueError(f'Motion log evidence match failed: {event}')
            for src in (matches[0],matches[0].parent/'waist_commands'/matches[0].name):
                dest=ARCHIVE/'motion_logs'/src.relative_to(ROOT/'logs/endpoint_motion')
                dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,dest)
    write_json(ARCHIVE/'candidates/audit.json',dict(completed_at=now(),exports=audit,
        checks=['raw endpoints','stationary boundaries','continuous joint limits','velocity','acceleration','jerk',
                'waist envelope','cache integrity','original successful timing preserved'],
        collision_checked=False,optimized_candidates_physical_tested=False))
    live=ROOT/'data/task_panel/dring_demonstration.json'
    if fingerprint(json.loads(live.read_text()))!=fingerprint(original):
        raise RuntimeError('Live bundle changed after preservation; refusing to overwrite')
    write_json(live,bundle);write_json(live.with_name('dring_demonstration_'+bundle['side']+'.json'),bundle)
    hashes={str(p.relative_to(ARCHIVE)):hashlib.sha256(p.read_bytes()).hexdigest()
            for p in ARCHIVE.rglob('*') if p.is_file() and p.name!='manifest.json'}
    write_json(ARCHIVE/'manifest.json',dict(created_at=now(),sha256=hashes))
    print('Verified and installed ten slots, twenty exports, preserved both successful baselines and motion logs.')


if __name__=='__main__':main()
