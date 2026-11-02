"""Offline tracking evidence by trace, joint and failure snapshot; no ROS imports."""
import argparse
from collections import Counter
import csv
import json
from pathlib import Path

import numpy as np

ROOT=Path(__file__).resolve().parents[1]


def review(root=ROOT,date='20260911'):
    traces=[]
    for path in sorted((root/'logs/endpoint_motion').glob(f'endpoint_motion_{date}_*.csv')):
        with path.open() as stream:
            rows=[r for r in csv.DictReader(stream) if r['event'] in ('sample','settling_hold')]
        if not rows:continue
        names=[k[:-7] for k in rows[0] if k.endswith('_target')]
        stamps=np.array([float(r['ros_time_s']) for r in rows])
        sample=np.array([r['event']=='sample' for r in rows])
        hold=(~sample)&(stamps>=stamps[-1]-1.)
        joints={}
        for n in names:
            def a(suffix):return np.array([float(r[n+suffix]) for r in rows])
            e=a('_target')-a('_q');v=a('_dq')
            cmd=a('_command_dq') if n+'_command_dq' in rows[0] else a('_target_dq')
            moving=sample&(abs(cmd)>.01)
            steady=hold&(abs(v)<.03)
            item=dict(motion_samples=int(sample.sum()),steady_samples=int(steady.sum()),
                motion_error_p95_deg=float(np.degrees(np.quantile(abs(e[sample]),.95))) if sample.any() else None,
                motion_peak_velocity_rad_s=float(max(abs(v[sample]))) if sample.any() else None,
                command_peak_velocity_rad_s=float(max(abs(cmd[sample]))) if sample.any() else None,
                moving_stuck_fraction=float(np.mean(abs(v[moving])<.008)) if moving.any() else None,
                steady_signed_error_rad=float(np.median(e[steady])) if steady.any() else None,
                steady_error_deg=float(np.degrees(np.median(abs(e[steady])))) if steady.any() else None)
            if n+'_arm_integral_nm' in rows[0]:
                item['steady_integral_nm']=float(np.median(a('_arm_integral_nm')[steady])) if steady.any() else None
                item['peak_integral_nm']=float(np.max(abs(a('_arm_integral_nm'))))
            if n+'_tau_est' in rows[0]:item['peak_tau_est_nm']=float(np.max(abs(a('_tau_est'))))
            if n=='waist_yaw_joint' and 'waist_integral_mode' in rows[0]:
                item['motion_integral_modes']=dict(Counter(r['waist_integral_mode'] for r,s in zip(rows,sample) if s))
                item['steady_integral_nm']=float(np.median([float(r['waist_integral_torque_nm']) for r,s in zip(rows,steady) if s])) if steady.any() else None
                item['peak_integral_nm']=float(np.max(abs(np.array([float(r['waist_integral_torque_nm']) for r in rows]))))
            joints[n]=item
        traces.append(dict(path=str(path),joints=joints))
    failures=[]
    for path in sorted((root/'data/task_sessions').glob(f'{date}_*/plan_*/execution_failure.json')):
        d=json.loads(path.read_text());q=d.get('feedback_at_failure',{}).get('joints',{})
        expected=d.get('expected_endpoint') or {};gateway=d.get('gateway_at_failure',{})
        errors={n:float(np.degrees(expected[n]-q[n])) for n in expected if n in q and
                ('shoulder' in n or 'elbow' in n or 'wrist' in n or n=='waist_yaw_joint')}
        failures.append(dict(path=str(path),message=d['message'],errors_deg=errors,
            active_arm=gateway.get('active_arm'),phase=gateway.get('phase'),
            state_age_s=d.get('feedback_at_failure',{}).get('age_s'),
            config=d.get('planning_config',{}).get('values',{})))
    return dict(traces=traces,failures=failures,
        note='Historical settings differ. Compare per trace and snapshot; fixed feedback is not a closed-loop prediction.')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'logs/tracking_tuning_20260911/history.json')
    parser.add_argument('--date',default='20260911')
    args=parser.parse_args()
    if len(args.date)!=8 or not args.date.isdigit():parser.error('--date must be YYYYMMDD')
    report=review(date=args.date)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(args.output)
    for n in sorted({n for t in report['traces'] for n in t['joints']}):
        recent=[(t['path'],t['joints'][n]) for t in report['traces'] if n in t['joints']]
        steady=sorted([(r['steady_error_deg'],Path(p).name) for p,r in recent if r['steady_error_deg'] is not None],reverse=True)
        peaks=[r['motion_error_p95_deg'] for p,r in recent if r['motion_error_p95_deg'] is not None]
        print(n,'recent traces',len(recent),'worst hold',steady[:4],'motion p95 range', [round(min(peaks),3),round(max(peaks),3)] if peaks else [])
    print('failure counts',dict(Counter(n for f in report['failures'] for n,e in f['errors_deg'].items() if abs(e)>1.4324)))


if __name__=='__main__':main()
