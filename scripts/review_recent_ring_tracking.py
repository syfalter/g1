"""Read-only archived command/feedback FK review; no ROS initialization or outputs."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from rosidl_runtime_py.set_message import set_message_fields
from trajectory_msgs.msg import JointTrajectory
from task_trajectory import samples
from workbench_model import OfflineModel

ROOT=Path(__file__).resolve().parents[1]
CASES=(('20260911_193221_310116','plan_0074','194249_018'),
       ('20260911_193221_310116','plan_0090','194915_028'),
       ('20260911_195117_856127','plan_0033','195436_004'),
       ('20260911_195117_856127','plan_0037','195629_009'))


def path_metrics(points,center):
    points=np.asarray(points);radius=np.linalg.norm(points[:,:2]-center[:2],axis=1)
    angle=np.unwrap(np.arctan2(points[:,1]-center[1],points[:,0]-center[0]))
    return dict(radius_range_mm=(np.array([radius.min(),radius.max()])*1000.).tolist(),
        sweep_deg=float(np.degrees(angle[-1]-angle[0])),height_span_mm=float(np.ptp(points[:,2])*1000.))


def review(model,session,plan_name,log_name):
    directory=ROOT/'data/task_sessions'/session/plan_name
    plan=json.loads((directory/'plan.json').read_text())
    center=np.asarray(plan['contact_scene']['base_top_pelvis_m'])
    wrist=np.asarray(plan['search_diagnostics']['straight_index']['wrist_from_tip'])
    q=dict(plan['start']);side=plan['arm']
    for segment in plan['segments']:
        trajectory=JointTrajectory();set_message_fields(trajectory,segment['trajectory'])
        if '120' in segment['label']:break
        q.update(zip(trajectory.joint_names,trajectory.points[-1].positions))
    def fk(state):return (model.transform(side+'_wrist_yaw_link',state)@wrist)[:3,3]
    planned=[]
    for values in samples(trajectory,dict.fromkeys(trajectory.joint_names,.6),dict.fromkeys(trajectory.joint_names,1.5)):
        state=dict(q);state.update(zip(trajectory.joint_names,values));planned.append(fk(state))
    log=ROOT/'logs/endpoint_motion'/('endpoint_motion_20260911_'+log_name+'.csv')
    with log.open() as stream:rows=[r for r in csv.DictReader(stream) if r['event']=='sample']
    initial=np.array([float(rows[0][n+'_target']) for n in trajectory.joint_names])
    np.testing.assert_allclose(initial,trajectory.points[0].positions,atol=1e-5,rtol=0)
    command=[];measured=[]
    for row in rows:
        for suffix,collection in (('_q',measured),('_target',command)):
            state=dict(q);state.update({n:float(row[n+suffix]) for n in trajectory.joint_names});collection.append(fk(state))
    command=np.asarray(command);measured=np.asarray(measured);planned=np.asarray(planned)
    errors={n:float(np.degrees(max(abs(float(r[n+'_target'])-float(r[n+'_q'])) for r in rows)))
            for n in trajectory.joint_names}
    failure_file=directory/'execution_failure.json'
    failure=json.loads(failure_file.read_text()) if failure_file.is_file() else {}
    keys=('smooth_joint_path','smooth_insert_path','smooth_contact_path','tilt_deg','twist_deg',
          'path_tolerance_mm','target_tolerance_mm','left_shoulder_roll_kp','left_elbow_kp','elbow_ki')
    report=dict(session=session,plan=plan_name,direction=plan['sweep_direction'],csv=str(log),
        planning_values={k:plan['planning_config']['values'].get(k) for k in keys},
        center_pelvis_m=center.tolist(),planned=path_metrics(planned,center),
        executed_command=path_metrics(command,center),feedback_fk=path_metrics(measured,center),
        max_fk_tracking_error_mm=float(np.linalg.norm(measured-command,axis=1).max()*1000.),
        max_joint_error_deg=dict(sorted(errors.items(),key=lambda item:-item[1])),
        min_time_scale=min(float(r['time_scale']) for r in rows),
        last_trajectory_progress_s=float(rows[-1]['trajectory_progress_s']),
        failure=failure.get('message'),failure_stage=failure.get('checkpoint',{}).get('segment'),
        physical_fingertip_observed=False)
    return report,(planned-center,command-center,measured-center)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();model=OfflineModel(args.urdf)
    results=[review(model,*case) for case in CASES]
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    figure,axes=plt.subplots(2,2,figsize=(11,10),constrained_layout=True)
    for axis,(report,paths) in zip(axes.flat,results):
        for p,label,color in zip(paths,('Full planned path','Executed command','Feedback FK (nominal TCP)'),('#222222','#168a78','#c94043')):
            axis.plot(p[:,0]*1000,p[:,1]*1000,label=label,color=color,linewidth=1.5)
        axis.scatter([0],[0],marker='+',color='black')
        axis.set(title=report['session'][9:15]+' '+report['plan']+' '+report['direction'].upper(),
                 xlabel='Pelvis X relative to detected center (mm)',ylabel='Pelvis Y relative to center (mm)')
        axis.set_aspect('equal',adjustable='datalim');axis.grid(alpha=.2);axis.legend(fontsize=8)
    args.output.mkdir(parents=True,exist_ok=True)
    figure.savefig(args.output/'recent_tip_paths.png',dpi=150);plt.close(figure)
    report=dict(offline_only=True,hardware_commands_sent=0,ros_initialized=False,
        note='Feedback FK assumes nominal robot and finger geometry; not independent optical tip measurement. ' 
             'Running samples only: final settling and real ring angle are not measured by these metrics.',
        plans=[r for r,_ in results])
    (args.output/'recent_review.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
