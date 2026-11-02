"""Offline archive analysis only. No ROS initialization or hardware transports."""
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.interpolate import BPoly
from workbench_model import OfflineModel

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'logs/dring_replay_analysis_20260918'
CASES = [
    ('20260918_201935_638354', '0004', '202100_004'),
    ('20260918_201935_638354', '0020', '202432_009'),
    ('20260918_201935_638354', '0037', '202730_014'),
    ('20260918_201935_638354', '0045', '203027_019'),
    ('20260918_204017_069986', '0004', '204137_004'),
    ('20260918_204017_069986', '0017', '204536_009'),
    ('20260918_205157_680487', '0013', '205327_005'),
    ('20260918_205157_680487', '0027', '205635_010'),
]
URDF = ROOT / 'data/offline_trajectory_library/live/20260918_180343_965974_plan_0017/context/robot.urdf'


def stats(values):
    v = np.asarray(values)
    return dict(rms=float(np.sqrt(np.mean(v*v))), median=float(np.median(v)),
                p95=float(np.percentile(v, 95)), max=float(np.max(v)))


def spans(mask, dt, progress):
    edges = np.diff(np.r_[False, mask, False].astype(int))
    result = [dict(seconds=float(np.sum(dt[a:b])), start_progress=float(progress[a]),
                   end_progress=float(progress[b-1]), first=int(a), last=int(b-1))
              for a, b in zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1))]
    return sorted(result, key=lambda v: -v['seconds'])[:5]


def analyze(model, session, number, logfile):
    directory = ROOT / 'data/task_sessions' / session / ('dring_demo_' + number)
    plan = json.loads((directory / 'plan.json').read_text())
    config = json.loads((directory / 'config.json').read_text())
    assert hashlib.sha256(URDF.read_bytes()).hexdigest() == config['identity']['robot_sha256']
    segment = next(s for s in plan['segments'] if s.get('recording_fit'))
    names = segment['trajectory']['joint_names']
    path = ROOT / 'logs/endpoint_motion' / ('endpoint_motion_20260918_' + logfile + '.csv')
    rows = [r for r in csv.DictReader(path.open()) if r['event'] == 'sample']
    col = lambda key: np.array([float(r[key]) for r in rows])
    target = np.column_stack([col(n+'_target') for n in names])
    actual = np.column_stack([col(n+'_q') for n in names])
    error = target - actual
    progress = col('trajectory_progress_s')
    dt = col('control_gap_ms') / 1000
    elapsed = np.cumsum(dt)
    scale = col('time_scale')
    points = segment['trajectory']['points']
    times = np.array([p['time_from_start']['sec']+p['time_from_start']['nanosec']*1e-9 for p in points])
    curve = BPoly.from_derivatives(times, [[p['positions'], p['velocities'], p['accelerations']] for p in points])
    # Check the offline evaluator against the reference actually logged by the gateway.
    interp_error = float(np.max(np.abs(curve(progress)-target)))
    assert interp_error < 1e-6, interp_error
    raw = config['traces'][plan['demo']['operation']]['raw']
    rt = np.array(raw['time_s']); rq = np.array(raw['positions'])
    factor = plan['speed_scaling']['applied_multiplier']
    taught = np.column_stack([np.interp(progress*factor, rt, rq[:,i]) for i in range(len(names))])
    tcp = json.loads((directory.parent/'index_tcp.json').read_text())
    offset = np.array(tcp['sides'][plan['arm']]['states']['open']['wrist_yaw_from_index_tcp']['matrix_row_major'])
    link = plan['arm'] + '_wrist_yaw_link'
    def poses(q):
        return model.batch_transforms([link], plan['start'], names, q)[link] @ offset
    for values in (target[0], actual[len(actual)//2], taught[-1]):
        state = dict(plan['start'], **dict(zip(names, values)))
        np.testing.assert_allclose(poses(values[None,:])[0], model.transform(link,state)@offset,
                                   atol=1e-12, rtol=0)
    fp, fa, fr = poses(target), poses(actual), poses(taught)
    raw_fk = poses(rq)
    fitted_raw_fk = poses(curve(rt/factor))
    tracking_mm = np.linalg.norm(fa[:,:3,3]-fp[:,:3,3], axis=1)*1000
    fit_mm = np.linalg.norm(fitted_raw_fk[:,:3,3]-raw_fk[:,:3,3], axis=1)*1000
    total_mm = np.linalg.norm(fa[:,:3,3]-fr[:,:3,3], axis=1)*1000
    rotation = np.einsum('nji,njk->nik', fp[:,:3,:3], fa[:,:3,:3])
    angle = np.degrees(np.arccos(np.clip((np.trace(rotation,axis1=1,axis2=2)-1)/2,-1,1)))
    # Derivative of nominal TCP translation along the logged reference clock.
    eps = .0001
    lo = np.maximum(0, progress-eps); hi = np.minimum(times[-1], progress+eps)
    speed = np.linalg.norm(poses(curve(hi))[:,:3,3]-poses(curve(lo))[:,:3,3], axis=1)/(hi-lo)*scale*1000
    joint_reports = {}
    dominant = np.argmax(np.abs(error), axis=1)
    for i,n in enumerate(names):
        isolated = target.copy(); isolated[:,i] = actual[:,i]
        torque = col(n+'_tau_est')
        joint_reports[n] = dict(error_deg=stats(np.abs(np.degrees(error[:,i]))),
            signed_error_deg_median=float(np.median(np.degrees(error[:,i]))),
            torque_abs_nm=stats(np.abs(torque)), torque_range_nm=[float(min(torque)),float(max(torque))],
            integral_abs_max_nm=float(np.max(np.abs(col(n+'_arm_integral_nm')))),
            max_single_joint_tip_error_mm=float(np.max(np.linalg.norm(poses(isolated)[:,:3,3]-fp[:,:3,3],axis=1))*1000),
            dominant_slow_seconds=float(np.sum(dt[(dominant==i)&(scale<.9)])))
    worst = int(np.argmin(scale))
    events = [json.loads(line) for line in next(directory.glob('execution_events*')).read_text().splitlines()]
    stage = [e for e in events if e.get('segment')==segment['label']]
    accepted = next(e for e in stage if e['checkpoint']=='arm_accepted')
    completed = next(e for e in stage if e['checkpoint']=='arm_completed_waiting_settle')
    actual_seconds = completed['monotonic_s']-accepted['monotonic_s']
    report = dict(session=session,plan=directory.name,csv=str(path.relative_to(ROOT)),
        recorded_seconds=float(rt[-1]),sample_count=len(rt),stored_hz=(len(rt)-1)/rt[-1],
        raw_sample_gap_ms=stats(np.diff(rt)*1000),applied_multiplier=factor,
        planned_seconds=segment['duration'],actual_seconds=actual_seconds,
        effective_recorded_speed=float(rt[-1]/actual_seconds),
        measured_control_seconds=float(sum(dt)),gateway_interpolation_max_error_rad=interp_error,
        profile=rows[-1]['task_stage_profile'],proximal_kp=col('task_stage_proximal_kp')[-1],
        time_scale_min=float(min(scale)),slow_below_half_seconds=float(sum(dt[scale<.5])),
        zero_time_scale_seconds=float(sum(dt[scale==0])),slow_spans=spans(scale<.5,dt,progress),
        reference_below_2mm_s=spans(speed<2,dt,progress),
        reference_below_5mm_s=spans(speed<5,dt,progress),
        fit_tip_mm=stats(fit_mm),tracking_tip_mm=stats(tracking_mm),
        taught_to_actual_tip_mm=stats(total_mm),tracking_orientation_deg=stats(angle),
        recording_fit=segment['recording_fit'],joints=joint_reports,
        worst_slow_sample=dict(progress_s=float(progress[worst]),elapsed_s=float(elapsed[worst]),
            joint=names[dominant[worst]],error_deg=float(np.degrees(error[worst,dominant[worst]])),
            torque_nm=float(col(names[dominant[worst]]+'_tau_est')[worst])),
        lowstate_age_ms=stats(col('lowstate_steady_age_ms')),control_gap_ms=stats(col('control_gap_ms')),
        ros_clock_backward_jumps=int(sum(np.diff(col('ros_time_s'))<0)))
    slow = scale < .5
    wrist = 'left_wrist_yaw_joint'
    if np.any(slow):
        from collections import Counter
        report['slow_wrist'] = dict(
            integral_modes=dict(Counter(r[wrist+'_arm_integral_mode'] for r,m in zip(rows,slow) if m)),
            integral_abs_max_nm=float(np.max(np.abs(col(wrist+'_arm_integral_nm')[slow]))),
            torque_range_nm=[float(np.min(col(wrist+'_tau_est')[slow])),float(np.max(col(wrist+'_tau_est')[slow]))],
            measured_dq_median=float(np.median(col(wrist+'_dq')[slow])),
            command_dq_median=float(np.median(col(wrist+'_command_dq')[slow])))
    drop = int(np.argmin(np.diff(tracking_mm))) + 1
    a=max(0,drop-25); b=min(len(rows)-1,drop+25)
    report['sharpest_error_drop'] = dict(elapsed_s=float(elapsed[drop]),
        tip_error_before_after_mm=[float(tracking_mm[a]),float(tracking_mm[b])],
        wrist_tau_before_after_nm=[float(col(wrist+'_tau_est')[a]),float(col(wrist+'_tau_est')[b])],
        note='Possible unloading/exit/contact loss, not proof of slip without video or force sensing.')
    report['raw_joint_trace_sha256'] = hashlib.sha256(json.dumps(raw,sort_keys=True).encode()).hexdigest()
    return report,dict(elapsed=elapsed,progress=progress,scale=scale,tracking_mm=tracking_mm,
                       target=fp[:,:3,3],actual=fa[:,:3,3],raw=fr[:,:3,3],speed=speed)


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    model=OfflineModel(URDF)
    results=[analyze(model,*case) for case in CASES]
    report=dict(offline_only=True,hardware_commands_sent=0,urdf=str(URDF),
                note='Nominal open-index TCP, fixed other joints/base. Not optical contact position. '
                     'Monotonic event times, not ROS wall clock, measure elapsed execution. '
                     'Tau_est is motor feedback estimate, not contact force or fully logged arm LowCmd torque.',
                runs=[r for r,_ in results])
    (OUT/'analysis.json').write_text(json.dumps(report,indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes=plt.subplots(4,2,figsize=(14,13),constrained_layout=True)
    for ax,(r,v) in zip(axes.flat,results):
        ax.plot(v['elapsed'],v['tracking_mm'],label='Command-to-feedback TCP error (mm)')
        ax.plot(v['elapsed'],v['scale']*10,label='Time scale x10')
        ax.set(title=Path(r['csv']).stem[-10:]+' Kp='+str(r['proximal_kp']),xlabel='Monotonic elapsed (s)')
        ax.grid(alpha=.2);ax.legend(fontsize=7)
    fig.savefig(OUT/'tracking.png',dpi=130);plt.close(fig)
    fig,axes=plt.subplots(2,2,figsize=(12,10),constrained_layout=True)
    for ax,(r,v) in zip(axes.flat,results[-4:]):
        for key,label in [('raw','Taught'),('target','Reference'),('actual','Feedback FK')]:
            p=v[key]*1000;ax.plot(p[:,0],p[:,1],label=label)
        ax.set(title=Path(r['csv']).stem[-10:],xlabel='Pelvis X (mm)',ylabel='Pelvis Y (mm)')
        ax.set_aspect('equal',adjustable='datalim');ax.legend();ax.grid(alpha=.2)
    fig.savefig(OUT/'paths_xy.png',dpi=130);plt.close(fig)
    for r,_ in results:
        print(Path(r['csv']).stem[-10:], 'seconds',round(r['actual_seconds'],2),
              'fit mm',round(r['fit_tip_mm']['max'],2),'tracking mm',
              {k:round(v,2) for k,v in r['tracking_tip_mm'].items()},
              'slowest',r['worst_slow_sample'])


if __name__=='__main__':
    main()
