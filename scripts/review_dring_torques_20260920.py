"""Read-only analysis of the two most recent completed archived demo cycles."""
import csv
import json
from pathlib import Path

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
JOINTS=('shoulder_pitch','shoulder_roll','shoulder_yaw','elbow','wrist_roll','wrist_pitch','wrist_yaw')
CASES=(('0016','121950_014',range(11,16)),('0026','122535_021',range(18,23)))


def metrics(rows,name,limit):
    torque=np.array([float(r[name+'_tau_est']) for r in rows])
    error=np.array([abs(float(r[name+'_target'])-float(r[name+'_q'])) for r in rows])
    dt=np.array([float(r['control_gap_ms'])*.001 for r in rows])
    return dict(peak_abs_nm=float(np.max(np.abs(torque))),
        rms_nm=float(np.sqrt(np.average(torque**2,weights=dt))),
        p95_abs_nm=float(np.percentile(np.abs(torque),95)),
        signed_range_nm=[float(np.min(torque)),float(np.max(torque))],
        over_continuous_seconds=float(np.sum(dt[np.abs(torque)>limit])) if limit else None,
        max_tracking_error_deg=float(np.degrees(np.max(error))))


def main():
    session=ROOT/'data/task_sessions/20260920_120520_235978'
    files=sorted((ROOT/'logs/endpoint_motion').glob('endpoint_motion_20260920_12*.csv'))
    results=[]
    for number,replay,indices in CASES:
        directory=session/('dring_demo_'+number)
        plan=json.loads((directory/'plan.json').read_text())
        execution=json.loads((directory/'execution.json').read_text())
        assert len(execution)==len(plan['segments']) and all(r['error_code']==0 for r in execution)
        selected=[p for p in files if int(p.stem.rsplit('_',1)[1]) in indices]
        assert len(selected)==5
        whole=[r for p in selected for r in csv.DictReader(p.open()) if r['event']=='sample']
        path=ROOT/'logs/endpoint_motion'/('endpoint_motion_20260920_'+replay+'.csv')
        rows=[r for r in csv.DictReader(path.open()) if r['event']=='sample']
        events=[json.loads(line) for line in next(directory.glob('execution_events*.jsonl')).read_text().splitlines()]
        stage=next(s for s in plan['segments'] if s.get('recording_fit'))
        start=next(e for e in events if e['checkpoint']=='arm_accepted' and e['segment']==stage['label'])
        end=next(e for e in events if e['checkpoint']=='arm_completed_waiting_settle' and e['segment']==stage['label'])
        result=dict(plan=str(directory.relative_to(ROOT)),replay_csv=str(path.relative_to(ROOT)),
            whole_cycle_csv=[str(p.relative_to(ROOT)) for p in selected],
            actual_replay_seconds=end['monotonic_s']-start['monotonic_s'],
            minimum_time_scale=min(float(r['time_scale']) for r in rows),
            replay_gains={k:rows[-1][k] for k in rows[-1] if k.startswith('task_stage_')},joints={})
        for side in ('left','right'):
            for joint in JOINTS:
                name=side+'_'+joint+'_joint';limit=4. if joint in ('wrist_pitch','wrist_yaw') else 20.
                result['joints'][name]=dict(continuous_software_limit_nm=limit,
                    peak_software_limit_nm=5. if limit==4 else 25.,
                    replay=metrics(rows,name,limit),whole_cycle=metrics(whole,name,limit))
        result['joints']['waist_yaw_joint']=dict(replay=metrics(rows,'waist_yaw_joint',None),
            whole_cycle=metrics(whole,'waist_yaw_joint',None))
        results.append(result)
    out=ROOT/'logs/dring_torque_review_20260920';out.mkdir(parents=True,exist_ok=True)
    report=dict(note='Motor tau_est, not independently measured contact torque. Replay samples only vs '
        'all five moving stages; excludes final settling holds. Thresholds are software limits, not '
        'confirmed hardware continuous ratings. RMS weighted by measured control interval.',runs=results)
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    lines=['# 2026-09-20 最近两轮扭矩分析','',
        '两轮录制回放开始时间为12:19:50和12:25:35，均为左臂，完整流程成功结束。',
        '数值为电机tau_est绝对峰值，单位N·m；整轮列包含准备、衔接、回放、返回的运动样本，不含最后静止观察。',
        '阈值为当前软件监控设置，不代表本次验证了硬件持续额定能力。RMS、P95、方向和跟踪误差见report.json。','',
        '| 关节 | 第1轮回放 | 第2轮回放 | 第1轮整轮 | 第2轮整轮 | 软件持续/峰值 |',
        '|---|---:|---:|---:|---:|---|']
    for name,a in results[0]['joints'].items():
        b=results[1]['joints'][name]
        limits=(f"{a['continuous_software_limit_nm']:g}/{a['peak_software_limit_nm']:g}"
                if 'continuous_software_limit_nm' in a else '单独腰部策略')
        values=[a['replay']['peak_abs_nm'],b['replay']['peak_abs_nm'],
                a['whole_cycle']['peak_abs_nm'],b['whole_cycle']['peak_abs_nm']]
        lines.append('| '+name+' | '+' | '.join(f'{v:.3f}' for v in values)+' | '+limits+' |')
    lines+=['','两轮手臂各轴在上述运动样本中均没有超过持续阈值。',
        '两轮回放肩肘Kp550，腕滚转110，腕俯仰/偏航220，日志确认实际生效。',
        '力矩低于阈值不能证明拨环成功，也不能直接按剩余力矩比例增加Kp。','',
        '本次分析没有连接机器人或修改任何历史日志。']
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')
    for r in results:
        print(r['replay_csv'],round(r['actual_replay_seconds'],3))
        for name,m in r['joints'].items():
            if name.startswith('right'):continue
            print(name,'replay peak/RMS',round(m['replay']['peak_abs_nm'],3),round(m['replay']['rms_nm'],3),
                  'whole peak',round(m['whole_cycle']['peak_abs_nm'],3))


if __name__=='__main__':main()
