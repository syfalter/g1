"""Reproducible XY stance trials using the application's actual offline planner."""
import argparse
from collections import Counter
from datetime import datetime
import gzip
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import threading
import time
import traceback
import xml.etree.ElementTree as ET

import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[1]
ASSETS=ROOT/'assets/workbench'
STANCE=('test_stance_x','test_stance_y')
CONTACT={'INSERT','ROTATE_LEFT','ROTATE_RIGHT','RECENTER','WITHDRAW'}
ARC={'ROTATE_LEFT','ROTATE_RIGHT','RECENTER'}
EXPECTED=['START','ENTER_TABLE','EXTEND_INDEX','INSERT','ROTATE_LEFT','RECENTER',
          'ROTATE_RIGHT','RECENTER','WITHDRAW','FOLD_INDEX','RETURN_HOME']


class RosDumper(yaml.SafeDumper):
    def increase_indent(self,flow=False,indentless=False):
        return super().increase_indent(flow,False)


def write_json(path,data):
    temporary=path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(data,indent=2,allow_nan=False)+'\n')
    temporary.replace(path)


def prepare(args):
    if args.root.exists():
        raise ValueError('Run directory exists; use a new directory to preserve evidence')
    (args.root/'scene').mkdir(parents=True)
    (args.root/'warm').mkdir()
    (args.root/'results').mkdir()
    (args.root/'code').mkdir()
    source=yaml.safe_load(args.snapshot.read_text())['/move_group']['ros__parameters']

    def prune(data):
        if isinstance(data,dict):
            output={k:prune(v) for k,v in data.items() if v is not None}
            return {k:v for k,v in output.items() if v!={}}
        if isinstance(data,list):return [prune(v) for v in data]
        return data

    params=prune(source)
    assert params['allow_trajectory_execution'] is False
    baseline=params['robot_description']
    robot=ET.fromstring(baseline)
    root_joint=robot.find("joint[@name='preview_world_pelvis']")
    initial_base=[float(v) for v in root_joint.find('origin').get('xyz').split()]
    original_parent=root_joint.find('parent').get('link')
    root_joint.find('parent').set('link','test_stance_y_link')
    for name,parent,axis in ((STANCE[0],original_parent,'1 0 0'),(STANCE[1],'test_stance_x_link','0 1 0')):
        ET.SubElement(robot,'link',name=name+'_link')
        joint=ET.SubElement(robot,'joint',name=name,type='prismatic')
        ET.SubElement(joint,'parent',link=parent)
        ET.SubElement(joint,'child',link=name+'_link')
        ET.SubElement(joint,'axis',xyz=axis)
        ET.SubElement(joint,'limit',lower='-0.3',upper='0.3',effort='1',velocity='0.1')
    params['robot_description']=ET.tostring(robot,encoding='unicode')
    (args.root/'scene/robot.urdf').write_text(params['robot_description'])
    (args.root/'scene/baseline.urdf').write_text(baseline)
    (args.root/'scene/robot.srdf').write_text(params['robot_description_semantic'])
    (args.root/'scene/move_group.yaml').write_text(yaml.dump({'/move_group':{'ros__parameters':params}},Dumper=RosDumper,sort_keys=False))
    shutil.copy2(args.snapshot,args.root/'scene/runtime_parameters.yaml')
    shutil.copy2(ASSETS/'task/target.json',args.root/'scene/target.json')
    shutil.copy2(ASSETS/'task_start_pose.json',args.root/'scene/initial_pose.json')
    for side in ('left','right'):
        shutil.copy2(ASSETS/f'task/last_{side}_full_plan.json',args.root/f'warm/last_{side}_full_plan.json')
    hashes={}
    for name in ('workbench_app.py','workbench_model.py','workbench_task_geometry.py','workbench_orientation.py','workbench_stance.py','workbench_robustness.py'):
        shutil.copy2(ROOT/'scripts'/name,args.root/'code'/name)
        hashes[name]=hashlib.sha256((ROOT/'scripts'/name).read_bytes()).hexdigest()
    rng=np.random.default_rng(args.seed)
    cases=[]
    for count,lo,hi,band in ((60,.04,.10,'near'),(25,0,.04,'closer'),(15,-.06,0,'farther')):
        for x,y in zip(rng.uniform(lo,hi,count),rng.uniform(-.12,.12,count)):
            cases.append(dict(dx_m=float(x),dy_m=float(y),band=band))
    rng.shuffle(cases)
    for i,case in enumerate(cases):
        case.update(id=i,pelvis_x_m=initial_base[0]+case['dx_m'],pelvis_y_m=initial_base[1]+case['dy_m'],
                    nominal_pelvis_front_gap_m=.11-case['dx_m'])
    write_json(args.root/'cases.json',cases)
    write_json(args.root/'manifest.json',dict(created=datetime.now().astimezone().isoformat(),seed=args.seed,
        random_stances=100,arms_per_stance=2,initial_pelvis_xyz=initial_base,
        original_model_sha256=hashlib.sha256(baseline.encode()).hexdigest(),code_sha256=hashes,
        sampling='60 near (+4..10 cm), 25 closer (0..4 cm), 15 farther (-6..0 cm); all y +/-12 cm',
        warm_start='Frozen original per-arm insertion seeds; never updated by another test case',
        test_only_model='Two fixed-per-trial XY variables above pelvis; environment and planning groups unchanged',
        planning_timeout_s=180,ompl_randomness='Sample seed is fixed; OMPL/KDL outcomes may vary between runs'))
    print(f'Prepared 100 stances in {args.root}',flush=True)


def check_models(node,root):
    from workbench_model import OfflineModel
    reference=OfflineModel(root/'scene/baseline.urdf')
    for dx,dy in ((0,0),(.081,-.072),(-.05,.11)):
        q=dict(node.home);q.update(zip(STANCE,(dx,dy)))
        for link in ('pelvis','left_index_tcp','right_index_tcp','left_ankle_roll_link','task_handle','workbench'):
            a=reference.transform(link,q)
            expected=a.copy()
            if link not in ('task_handle','workbench'):expected[:3,3]+=[dx,dy,0]
            np.testing.assert_allclose(node.model.transform(link,q),expected,atol=1e-10)


def audit(node,side,frames,home):
    from workbench_app import INDEX_FOLDED
    from moveit_msgs.srv import GetPositionFK
    from workbench_task_geometry import arc_point
    phases=[f['phase'] for i,f in enumerate(frames) if i==0 or frames[i-1]['phase']!=f['phase']]
    expected=list(EXPECTED)
    if abs(home[side+'_index_1_joint']-INDEX_FOLDED)>1e-8:
        expected.insert(1,'FOLD_START_INDEX')
        expected.append('RESTORE_START_INDEX')
    if phases!=expected:raise AssertionError(f'Incomplete/order-invalid task: {phases}')
    names=node.names
    values=np.array([[f['q'][name] for name in names] for f in frames])
    times=np.array([f['t'] for f in frames])
    if not np.isfinite(values).all() or not np.isfinite(times).all() or not (np.diff(times)>0).all():
        raise AssertionError('Nonfinite state or nonmonotonic time')
    np.testing.assert_allclose(values[0],[home[n] for n in names],atol=1e-8,rtol=0)
    np.testing.assert_allclose(values[-1],values[0],atol=1e-8,rtol=0)
    arm_names={side+'_'+part+'_joint' for part in ('shoulder_pitch','shoulder_roll','shoulder_yaw','elbow','wrist_roll','wrist_pitch','wrist_yaw')}
    active=arm_names|{'waist_yaw_joint','task_handle_angle',side+'_index_1_joint',side+'_index_2_joint'}
    for j,name in enumerate(names):
        if name not in active:np.testing.assert_allclose(values[:,j],home[name],atol=1e-8,rtol=0)
        joint=node.xml.find(f"joint[@name='{name}']")
        limit=joint.find('limit')
        if limit is not None:
            if values[:,j].min()<float(limit.get('lower','-inf'))-1e-7 or values[:,j].max()>float(limit.get('upper','inf'))+1e-7:
                raise AssertionError('Joint limit exceeded: '+name)
    handle=values[:,names.index('task_handle_angle')]
    np.testing.assert_allclose([handle.min(),handle.max()],[-2*math.pi/3,2*math.pi/3],atol=1e-8)
    count=0;max_error=0.;fk_error=0.;policy=None
    for i in range(1,len(frames)):
        a,b=frames[i-1],frames[i]
        contact=b['phase'] in CONTACT
        if contact!=policy:node.contact_policy(side,contact);policy=contact
        # Double the planner's joint-space checking density, with a midpoint minimum.
        steps=max(2,int(math.ceil(np.max(np.abs(values[i]-values[i-1]))/.005)))
        for fraction in np.linspace(0,1,steps+1)[1:]:
            q={name:float(values[i-1,j]*(1-fraction)+values[i,j]*fraction) for j,name in enumerate(names)}
            try:node.validate(q)
            except RuntimeError as error:raise AssertionError(f'False PLAN_READY at segment {i}, {b["phase"]}: {error}') from error
            count+=1
            if b['phase'] in ARC:
                error=float(np.linalg.norm(node.model.transform(side+'_index_tcp',q)[:3,3]-arc_point(node.target,q['task_handle_angle'])))
                max_error=max(max_error,error)
                if error>.002:raise AssertionError(f'Arc tracking error: {error}')
            if b['phase'] not in CONTACT and abs(q['task_handle_angle'])>1e-8:
                raise AssertionError('Handle moved outside contact phases')
        if i%max(1,len(frames)//12)==0 or (b['phase'] in ARC and abs(abs(b['q']['task_handle_angle'])-2*math.pi/3)<1e-8):
            request=GetPositionFK.Request();request.header.frame_id='world'
            request.fk_link_names=[side+'_index_tcp'];request.robot_state=node.state(b['q'])
            result=node.call(node.fk,request)
            if result.error_code.val!=1:raise AssertionError('Independent MoveIt FK failed')
            point=result.pose_stamped[0].pose.position
            error=float(np.linalg.norm(np.array([point.x,point.y,point.z])-node.model.transform(side+'_index_tcp',b['q'])[:3,3]))
            fk_error=max(fk_error,error)
            if error>1e-6:raise AssertionError(f'Test model differs from MoveIt FK: {error}')
    return dict(extra_collision_states=count,max_arc_error_m=max_error,max_fk_crosscheck_error_m=fk_error,
                frame_count=len(frames),duration_s=float(times[-1]),phase_order=phases)


def run(args):
    import rclpy
    from moveit_msgs.msg import PlanningSceneComponents
    from moveit_msgs.srv import GetPositionFK,GetPlanningScene
    from workbench_app import WorkbenchApp
    if os.environ.get('ROS_DOMAIN_ID')!=str(78+args.shard) or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise RuntimeError('Worker domain/localhost guard failed')
    cases=json.loads((args.root/'cases.json').read_text())
    cases=[c for c in cases if c['id']%args.shards==args.shard]
    if args.limit is not None:cases=cases[:args.limit]
    scene=args.root/'scene'
    rclpy.init()
    node=WorkbenchApp(scene_dir=scene,plan_dir=args.root/'results',warm_start_dir=args.root/'warm')
    node.fk=node.create_client(GetPositionFK,'/compute_fk')
    thread=threading.Thread(target=rclpy.spin,args=(node,),daemon=True);thread.start()
    check_models(node,args.root)
    original_home=dict(node.home)
    completed=0

    def policy_clear():
        req=GetPlanningScene.Request();req.components.components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        matrix=node.call(node.scene,req).scene.allowed_collision_matrix
        index=matrix.entry_names.index('task_handle')
        return all(not matrix.entry_values[i].enabled[index]
                   for i,name in enumerate(matrix.entry_names) if '_index_' in name)

    try:
        if not node.scene.wait_for_service(timeout_sec=30):raise RuntimeError('Test MoveIt failed to start; no trials executed')
        for case in cases:
            for side in (('left','right') if case['id']%2==0 else ('right','left')):
                output=args.root/'results'/f'{case["id"]:03d}_{side}.json'
                if output.exists():continue
                directory=args.root/'results'/f'{case["id"]:03d}_{side}'
                directory.mkdir(exist_ok=True)
                with node.lock:
                    node.home=dict(original_home);node.home.update(zip(STANCE,(case['dx_m'],case['dy_m'])))
                    node.plan_dir=directory
                node.command(dict(action='reset'))
                node.contact_policy('left',False);node.contact_policy('right',False)
                start_error=None
                try:node.validate(node.home)
                except RuntimeError as error:start_error=str(error)
                started=time.monotonic()
                node.command(dict(action='plan',arm=side,scope='full'))
                timed_out=False
                while node.busy:
                    if time.monotonic()-started>180 and not timed_out:
                        node.command(dict(action='cancel'));timed_out=True
                    if time.monotonic()-started>225:raise RuntimeError('Planner did not respond to cancellation')
                    time.sleep(.03)
                result=dict(case=case,arm=side,planning_seconds=time.monotonic()-started,
                            phase=node.status['phase'],message=node.status['message'],initial_error=start_error,
                            exception_type=node.last_error_type,candidate_attempts=node.candidate_attempts,
                            candidate_failure_counts=dict(Counter(node.candidate_failures)),extra_audit=None)
                result['contact_policy_restored_by_planner']=policy_clear()
                if timed_out:result['outcome']='TIME_BUDGET_EXCEEDED'
                elif node.plan:
                    try:
                        if start_error:raise AssertionError('Invalid initial stance accepted')
                        result['extra_audit']=audit(node,side,node.plan,node.home)
                        # Exercise the exact playback interpolation at intermediate and final states.
                        for fraction in (0,.13,.39,.67,1):
                            node.command(dict(action='seek',fraction=fraction))
                            if not all(math.isfinite(v) for v in node.current.values()):raise AssertionError('Nonfinite playback')
                        np.testing.assert_allclose([node.current[n] for n in node.names],[node.home[n] for n in node.names],atol=1e-8,rtol=0)
                        result['outcome']='PASS'
                    except Exception as error:
                        result.update(outcome='BUG',audit_error=str(error),audit_traceback=traceback.format_exc())
                elif node.last_error_type!='RuntimeError':result['outcome']='BUG_EXCEPTION'
                elif start_error:result['outcome']='INITIAL_COLLISION_REJECTED'
                elif 'Service' in result['message']:result['outcome']='SERVICE_FAILURE'
                else:result['outcome']='NO_COMPLETE_PLAN'
                if not node.plan:
                    try:node.command(dict(action='play'))
                    except ValueError:result['stale_play_rejected']=True
                    else:result.update(outcome='BUG',audit_error='Failed trial permits stale playback')
                if not result['contact_policy_restored_by_planner']:result.update(outcome='BUG',audit_error='Contact policy leaked')
                node.contact_policy('left',False);node.contact_policy('right',False)
                result['contact_policy_restored_after_audit']=policy_clear()
                result['total_seconds']=time.monotonic()-started
                report=directory/f'last_{side}_full_plan.json'
                if report.exists():
                    with report.open('rb') as source,gzip.open(str(report)+'.gz','wb') as destination:shutil.copyfileobj(source,destination)
                    report.unlink()
                    result['plan_file']=str(report.relative_to(args.root))+'.gz'
                write_json(output,result)
                completed+=1
                print('TRIAL '+json.dumps(dict(id=case['id'],arm=side,outcome=result['outcome'],
                    gap_cm=round(case['nominal_pelvis_front_gap_m']*100,2),dy_cm=round(case['dy_m']*100,2),
                    seconds=round(result['total_seconds'],2),completed_worker=completed)),flush=True)
    finally:
        node.cancel_event.clear()
        try:node.contact_policy('left',False);node.contact_policy('right',False)
        except Exception:pass
        rclpy.shutdown();thread.join(timeout=2)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('action',choices=('prepare','run'))
    parser.add_argument('--root',type=Path,required=True)
    parser.add_argument('--snapshot',type=Path,default=Path('/tmp/g1-robustness-snapshot/move_group.yaml'))
    parser.add_argument('--seed',type=int,default=20260908)
    parser.add_argument('--shard',type=int,choices=range(8),default=0)
    parser.add_argument('--shards',type=int,default=8)
    parser.add_argument('--limit',type=int)
    args=parser.parse_args()
    (prepare if args.action=='prepare' else run)(args)


if __name__=='__main__':main()
