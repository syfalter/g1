"""Offline recorded/command/measured comparison; no robot transport."""
import csv
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
from rosidl_runtime_py.set_message import set_message_fields
from trajectory_msgs.msg import JointTrajectory

from task_trajectory import coefficients, time_seconds
from workbench_model import OfflineModel

ROOT = Path(__file__).resolve().parents[1]
CASES = (
    ('20260920_132002_118427', '0008', '132149_004'),
    ('20260920_134512_921745', '0004', '134640_004'),
    ('20260920_134512_921745', '0023', '135152_008'),
)


def stats(values):
    a = np.asarray(values)
    return dict(rms=float(np.sqrt(np.mean(a*a))), p95=float(np.percentile(np.abs(a), 95)),
                maximum=float(np.max(np.abs(a))))


def evaluate(tr, times):
    knots = np.array([time_seconds(p) for p in tr.points])
    indices = np.clip(np.searchsorted(knots, times, side='right')-1, 0, len(knots)-2)
    values = np.zeros((len(times), len(tr.joint_names)))
    for i in np.unique(indices):
        mask = indices == i
        dt = knots[i+1]-knots[i]
        u = np.clip((times[mask]-knots[i])/dt, 0., 1.)
        c = coefficients(tr.points[i], tr.points[i+1], dt)
        values[mask] = np.polynomial.polynomial.polyval(u, c).T
    return values


def review(model, session, number, tag):
    directory = ROOT/'data/task_sessions'/session/('dring_demo_'+number)
    plan = json.loads((directory/'plan.json').read_text())
    config = json.loads((directory/'config.json').read_text())
    stage = next(s for s in plan['segments'] if s.get('recording_fit'))
    raw = config['traces'][plan['demo']['operation']]['raw']
    tr = JointTrajectory()
    set_message_fields(tr, stage['trajectory'])
    names = list(tr.joint_names)
    path = ROOT/'logs/endpoint_motion'/('endpoint_motion_20260920_'+tag+'.csv')
    with path.open() as stream:
        all_rows = list(csv.DictReader(stream))
    rows = [r for r in all_rows if r['event'] == 'sample']
    progress = np.array([float(r['trajectory_progress_s']) for r in rows])
    recorded_t = progress*plan['speed_scaling']['applied_multiplier']
    target, actual = [np.array([[float(r[n+suffix]) for n in names] for r in rows])
                      for suffix in ('_target', '_q')]
    recorded = np.column_stack([np.interp(recorded_t, raw['time_s'], np.array(raw['positions'])[:, i])
                                for i in range(len(names))])
    np.testing.assert_allclose(target, evaluate(tr, progress), atol=2e-8, rtol=0.)
    raw_fit = evaluate(tr, np.array(raw['time_s'])/plan['speed_scaling']['applied_multiplier'])
    error = np.degrees(target-actual)
    offset = np.array(json.loads((directory.parent/'index_tcp.json').read_text())[
        'sides']['left']['states']['open']['wrist_yaw_from_index_tcp']['matrix_row_major'])
    link = 'left_wrist_yaw_link'
    frames = [model.batch_transforms([link], plan['start'], names, q)[link] for q in (recorded, target, actual)]
    tips = [(f@offset)[:, :3, 3] for f in frames]
    position_error = np.linalg.norm(tips[2]-tips[1], axis=1)*1000
    fit_error = np.linalg.norm(tips[1]-tips[0], axis=1)*1000
    corrected = actual.copy()
    corrected[:,names.index('waist_yaw_joint')] = target[:,names.index('waist_yaw_joint')]
    corrected_tip = (model.batch_transforms([link],plan['start'],names,corrected)[link]@offset)[:,:3,3]
    corrected_error = np.linalg.norm(corrected_tip-tips[1],axis=1)*1000
    peak_index = int(np.argmax(position_error))
    orientation_error = np.degrees(Rotation.from_matrix(
        np.transpose(frames[1][:, :3, :3], (0, 2, 1))@frames[2][:, :3, :3]).magnitude())
    joints = {}
    for i, name in enumerate(names):
        joints[name] = dict(error_deg=stats(error[:, i]),
            signed_mean_deg=float(np.mean(error[:, i])),
            peak_at_recorded_s=float(recorded_t[np.argmax(np.abs(error[:, i]))]),
            torque_nm=stats([float(r[name+'_tau_est']) for r in rows]))
    bins = []
    for start in np.arange(0., raw['time_s'][-1], 2.):
        mask = (recorded_t >= start) & (recorded_t < start+2.)
        if not mask.any():
            continue
        bins.append(dict(recorded_time_s=[float(start),float(min(start+2.,raw['time_s'][-1]))],
            tcp_tracking_mm=stats(position_error[mask]),orientation_deg=stats(orientation_error[mask]),
            signed_joint_mean_deg=dict(zip(names,np.mean(error[mask],axis=0).tolist())),
            target_tip_mean_m=np.mean(tips[1][mask],axis=0).tolist(),
            actual_minus_target_tip_mean_mm=(np.mean(tips[2][mask]-tips[1][mask],axis=0)*1000).tolist()))
    endpoint = all_rows[-1]
    result = dict(csv=str(path),plan=str(directory),raw_count=len(raw['time_s']),
        recorded_duration_s=raw['time_s'][-1], planned_duration_s=stage['duration'],
        progress_end_s=float(progress[-1]),recorded_progress_end_s=float(recorded_t[-1]),
        events=[r['event'] for r in all_rows if r['event'] not in ('sample','settling_hold')],
        time_scale_min=min(float(r['time_scale']) for r in rows),
        gains={k:rows[-1][k] for k in rows[-1] if k.startswith('task_stage_')},
        fit_report=stage['recording_fit'],
        independently_sampled_fit_max_rad=float(np.max(np.abs(raw_fit-np.array(raw['positions'])))),
        endpoint_command_minus_recorded_rad=(np.array(tr.points[-1].positions)-np.array(raw['positions'][-1])).tolist(),
        tcp_tracking_mm=stats(position_error),tcp_fit_mm=stats(fit_error),
        hypothetical_exact_waist_tcp_error_mm=stats(corrected_error),
        peak_snapshot=dict(recorded_time_s=float(recorded_t[peak_index]),
            playback_progress_s=float(progress[peak_index]),
            tcp_error_mm=float(position_error[peak_index]),
            hypothetical_exact_waist_error_mm=float(corrected_error[peak_index]),
            joint_target_minus_actual_deg=dict(zip(names,error[peak_index].tolist()))),
        tcp_actual_vs_recorded_mm=stats(np.linalg.norm(tips[2]-tips[0],axis=1)*1000),
        orientation_tracking_deg=stats(orientation_error), joints=joints,bins=bins,
        last_event_joint_error_deg={n:float(np.degrees(float(endpoint[n+'_target'])-float(endpoint[n+'_q']))) for n in names})
    with (path.parent/'waist_commands'/path.name).open() as stream:
        waist = [r for r in csv.DictReader(stream) if r['phase']=='RUNNING']
    result['waist_command'] = dict(kp=sorted(set(float(r['sent_kp']) for r in waist)),
        kd=sorted(set(float(r['sent_kd']) for r in waist)),
        reference_minus_sent_deg=stats([np.degrees(float(r['reference_q'])-float(r['sent_q'])) for r in waist]),
        sent_minus_actual_deg=stats([np.degrees(float(r['sent_q'])-float(r['actual_q'])) for r in waist]),
        integral_nm=stats([float(r['integral_nm']) for r in waist]))
    return result, (recorded_t, tips, position_error, error)


def main():
    model = OfflineModel('/tmp/g1-real-task-iYagNj/robot.urdf')
    out = ROOT/'logs/dring_tracking_review_20260920_1353'
    out.mkdir(exist_ok=True)
    results = [review(model, *case) for case in CASES]
    report = dict(note='Read-only offline analysis. Nominal open-index TCP FK, not an optical contact '
        'measurement. No finger feedback or ring angle in CSV. Statistics use running samples; '
        '2-second bins are recording time, not identified contact stages.', runs=[r for r,_ in results])
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3,3,figsize=(15,11),constrained_layout=True)
    for row, ((result,(t,tips,err,jerr)),case) in enumerate(zip(results,CASES)):
        for p,label in zip(tips,('Recorded','Command','Measured FK')):
            axes[row,0].plot(p[:,0]*1000,p[:,1]*1000,label=label)
            axes[row,1].plot(t,p[:,2]*1000,label=label)
        axes[row,0].set(xlabel='Pelvis X (mm)',ylabel='Pelvis Y (mm)',title=case[2])
        axes[row,0].axis('equal')
        axes[row,1].set(xlabel='Recording time (s)',ylabel='TCP height (mm)')
        axes[row,2].plot(t,err)
        axes[row,2].set(xlabel='Recording time (s)',ylabel='TCP tracking error (mm)')
        for ax in axes[row]:ax.grid(alpha=.3)
        axes[row,0].legend()
    fig.savefig(out/'tracking.png',dpi=160)
    for result,_ in results:
        print(Path(result['csv']).name, json.dumps({k:result[k] for k in
            ('events','tcp_tracking_mm','tcp_fit_mm','orientation_tracking_deg','time_scale_min')}))
        print('joints',json.dumps(result['joints']))
        print('bins',json.dumps(result['bins']))


if __name__ == '__main__':
    main()
