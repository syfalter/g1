"""Summarize atomic trial records; safe to run while workers are active."""
import argparse
from collections import Counter,defaultdict
from datetime import datetime
import csv
import hashlib
import json
from pathlib import Path

import numpy as np

OUTCOMES=['PASS','INITIAL_COLLISION_REJECTED','NO_COMPLETE_PLAN','TIME_BUDGET_EXCEEDED',
          'BUG','BUG_EXCEPTION','SERVICE_FAILURE']
LABELS=['通过额外复查','初始碰撞/无效，拒绝规划','未找到完整路径','超过 180 秒预算',
        '路径/状态错误','非预期异常','服务故障']
ARMS=('left','right')


def load_results(root,cases):
    expected={case['id']:case for case in cases}
    if len(expected)!=len(cases):raise ValueError('Duplicate sample identifiers')
    results=[];seen=set()
    for path in sorted((root/'results').glob('???_*.json')):
        result=json.loads(path.read_text())
        key=(result['case']['id'],result['arm'])
        if (key in seen or key[0] not in expected or key[1] not in ARMS
                or result['case']!=expected[key[0]] or result['outcome'] not in OUTCOMES
                or path.name!=f'{key[0]:03d}_{key[1]}.json'):
            raise ValueError(f'Inconsistent trial record: {path}')
        seen.add(key);results.append(result)
    return results


def bucket_counts(cases,results,key,ranges):
    buckets=[]
    for lo,hi in ranges:
        ids={c['id'] for c in cases if lo<=c[key]*100<hi}
        bucket=dict(range_cm=[lo,hi],stances=len(ids))
        for side in ARMS:
            rows=[r for r in results if r['arm']==side and r['case']['id'] in ids]
            bucket[side]=dict(total=len(rows),counts=dict(Counter(r['outcome'] for r in rows)))
        buckets.append(bucket)
    return buckets


def provenance(root):
    manifest=json.loads((root/'manifest.json').read_text())
    scripts=Path(__file__).resolve().parent
    files={}
    for name,expected in manifest['code_sha256'].items():
        files[name]=dict(preparation_sha256=expected,
                        current_sha256=hashlib.sha256((scripts/name).read_bytes()).hexdigest(),
                        snapshot_sha256=hashlib.sha256((root/'code'/name).read_bytes()).hexdigest())
        files[name]['unchanged_since_preparation']=files[name]['current_sha256']==expected
    planner_names=('workbench_app.py','workbench_model.py','workbench_task_geometry.py')
    inputs={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest()
            for directory in ('scene','warm') for p in sorted((root/directory).iterdir()) if p.is_file()}
    inputs['cases.json']=hashlib.sha256((root/'cases.json').read_bytes()).hexdigest()
    data=dict(recorded=datetime.now().astimezone().isoformat(),source_files=files,input_sha256=inputs,
              planner_unchanged=all(files[n]['unchanged_since_preparation'] for n in planner_names),
              harness_note='Preparation snapshot used two shards. Batch changed only shard distribution to eight; original snapshot retained.',
              auxiliary_sha256={n:hashlib.sha256((scripts/n).read_bytes()).hexdigest() for n in
                                ('run_workbench_robustness.sh','workbench_robustness.launch.py','report_workbench_robustness.py')})
    (root/'provenance.json').write_text(json.dumps(data,indent=2)+'\n')
    return data


def summarize(root):
    cases=json.loads((root/'cases.json').read_text())
    results=load_results(root,cases)
    paired=defaultdict(dict)
    for result in results:paired[result['case']['id']][result['arm']]=result
    counts={side:dict(Counter(r['outcome'] for r in results if r['arm']==side)) for side in ('left','right')}
    both=[c for c in cases if len(paired[c['id']])==2 and all(r['outcome']=='PASS' for r in paired[c['id']].values())]
    buckets=bucket_counts(cases,results,'nominal_pelvis_front_gap_m',((0,3),(3,5),(5,7),(7,9),(9,11),(11,18)))
    for bucket in buckets:bucket['gap_cm']=bucket.pop('range_cm')
    lateral=bucket_counts(cases,results,'dy_m',((-12,-6),(-6,0),(0,6),(6,12.000001)))
    any_pass=[c for c in cases if any(r['outcome']=='PASS' for r in paired[c['id']].values())]
    neither=[c for c in cases if len(paired[c['id']])==2 and all(r['outcome']!='PASS' for r in paired[c['id']].values())]
    initial_collision=[c for c in cases if any(r['outcome']=='INITIAL_COLLISION_REJECTED' for r in paired[c['id']].values())]
    rejection_counts={side:Counter() for side in ARMS}
    for result in results:
        for message,count in result['candidate_failure_counts'].items():
            rejection_counts[result['arm']][message.split(':',1)[0]]+=count
    audited=[r for r in results if r['extra_audit']]
    summary=dict(updated=datetime.now().astimezone().isoformat(),expected_trials=2*len(cases),completed_trials=len(results),
        complete=len(results)==2*len(cases),any_pass=len(any_pass),neither_pass=len(neither),
        initial_collision_stances=[c['id'] for c in initial_collision],
        completed_stances=sum(len(v)==2 for v in paired.values()),arm_counts=counts,both_pass=len(both),
        closest_both_pass=min(both,key=lambda c:c['nominal_pelvis_front_gap_m']) if both else None,
        gap_buckets=buckets,lateral_buckets=lateral,bugs=[r for r in results if r['outcome'].startswith('BUG')],
        candidate_rejection_counts={side:dict(c) for side,c in rejection_counts.items()},
        max_audited_arc_error_mm=max((r['extra_audit']['max_arc_error_m']*1000 for r in audited),default=None),
        total_extra_checked_states=sum(r['extra_audit']['extra_collision_states'] for r in audited),
        max_fk_crosscheck_error_m=max((r['extra_audit']['max_fk_crosscheck_error_m'] for r in audited),default=None),
        stale_play_rejected=sum(r.get('stale_play_rejected',False) for r in results),
        contact_policy_restored_trials=sum(r['contact_policy_restored_by_planner'] for r in results),
        timing={side:{key:float(np.percentile([r['planning_seconds'] for r in results if r['arm']==side],p))
                      for key,p in [('median_s',50),('p95_s',95)]} for side in ('left','right')
                if any(r['arm']==side for r in results)})
    (root/'summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False)+'\n')
    with (root/'stances.csv').open('w',newline='') as stream:
        writer=csv.writer(stream)
        writer.writerow(['id','dx_cm','dy_cm','pelvis_front_gap_cm','left','right','left_plan_s','right_plan_s'])
        for c in cases:
            arms=paired[c['id']]
            writer.writerow([c['id'],c['dx_m']*100,c['dy_m']*100,c['nominal_pelvis_front_gap_m']*100,
                *[arms.get(side,{}).get('outcome','PENDING') for side in ('left','right')],
                *[arms.get(side,{}).get('planning_seconds','') for side in ('left','right')]])
    source=provenance(root)
    text=['# 100 组随机站位离线鲁棒性测试','',f'已完成 {len(results)}/200 次规划，覆盖 {summary["completed_stances"]}/100 个完整双手样本。',
          '状态：最终基线报告。' if summary['complete'] else '状态：中间结果，尚未完成。','',
          '## 采样与检查','',
          '- 相对原站位 x 后退 6 cm 至前移 10 cm、y 左右各 12 cm，随机种子 20260908。',
          '- 85/100 个样本更靠近台面，其中 60 个前移 4–10 cm；名义胯部前表面间隙约 1–17 cm。',
          '- 名义间隙按参考前平面计算，不是全身到柜体的最小距离；髋部等其他外凸部位也参与碰撞检测。',
          '- 每个站位左右手各做一次完整任务，固定原始关节姿态、CAD 目标和原规划参数。',
          '- 原成功路径仅提供冻结的逆解初值，不作为新站位的可执行路径。',
          '- 成功路径额外以不超过 0.005 rad 的关节步长检查碰撞，检查阶段顺序、关节限位、固定站位、回到初始状态、圆弧范围及指尖误差。',
          '- 指尖正解与 MoveIt FK 服务交叉核对；失败后检查旧计划不可播放、接触放行矩阵已恢复。',
          '- 运行于独立 localhost 域 78–85；测试通信使用 CycloneDDS 单播，原 RViz/控制配置未修改。','',
          '## 结果','', '| 结果 | 左手 | 右手 |','|---|---:|---:|']
    for outcome,label in zip(OUTCOMES,LABELS):text.append(f'| {label} | {counts["left"].get(outcome,0)} | {counts["right"].get(outcome,0)} |')
    text+=['',f'同一个站位两手均通过：{len(both)} 个；至少一手通过：{len(any_pass)} 个；已测完但两手均未通过：{len(neither)} 个。',
           f'初始碰撞/无效站位：{len(initial_collision)} 个。']
    if summary['closest_both_pass']:
        c=summary['closest_both_pass']
        text.append(f'最近的双手均通过样本为 #{c["id"]}：间隙 {c["nominal_pelvis_front_gap_m"]*100:.2f} cm，前移 {c["dx_m"]*100:.2f} cm，横移 {c["dy_m"]*100:.2f} cm。这不代表整个邻域都可用。')
    if audited:text.append(f'额外检查状态总数：{summary["total_extra_checked_states"]}；最大复查指尖圆弧误差：{summary["max_audited_arc_error_mm"]:.4f} mm。')
    text+=['','## 靠近台沿的影响','','| 间隙 cm | 样本数 | 左手通过/已测 | 右手通过/已测 |','|---|---:|---:|---:|']
    for b in buckets:
        text.append(f'| {b["gap_cm"][0]}–{b["gap_cm"][1]} | {b["stances"]} | {b["left"]["counts"].get("PASS",0)}/{b["left"]["total"]} | {b["right"]["counts"].get("PASS",0)}/{b["right"]["total"]} |')
    text+=['','## 横向偏移的影响','','| y 偏移 cm | 样本数 | 左手通过/已测 | 右手通过/已测 |','|---|---:|---:|---:|']
    for b in lateral:
        text.append(f'| {b["range_cm"][0]:g}–{b["range_cm"][1]:g} | {b["stances"]} | {b["left"]["counts"].get("PASS",0)}/{b["left"]["total"]} | {b["right"]["counts"].get("PASS",0)}/{b["right"]["total"]} |')
    text+=['','两手面对同一个固定 CAD 目标（world y=+22.5 cm），不是各自镜像的两个目标，因此成功率不要求对称。',
           '横移同时改变每只手的目标可达性；下表仅记录候选路径被拒的阶段次数，同一规划可能拒绝多个候选，不能当作失败样本数。','',
           '| 候选拒绝阶段/原因 | 左手次数 | 右手次数 |','|---|---:|---:|']
    for reason in sorted(set(rejection_counts['left'])|set(rejection_counts['right'])):
        text.append(f'| {reason} | {rejection_counts["left"][reason]} | {rejection_counts["right"][reason]} |')
    text+=['','## 状态与证据','',
           f'- 规划器恢复接触策略：{summary["contact_policy_restored_trials"]}/{len(results)} 次；失败后拒绝旧计划播放：{summary["stale_play_rejected"]} 次。',
           f'- 基线期间规划器、几何计算与模型辅助代码是否保持不变：{source["planner_unchanged"]}。',
           '- `provenance.json` 记录代码/输入哈希；准备时保留的是双进程测试脚本快照，正式批次仅把分片分配改为八进程，未改规划策略。',
           '- `_001`、`_002` 目录为通信环境搭建尝试，不属于本轮 100 个有效采样的结果。']
    text+=['','## 解释与边界','',
           '初始碰撞拒绝不是程序 Bug；未找到完整路径也不等于数学上不可达。180 秒超时单独统计，不能写成“不可达”。',
           'Bug 指错误报成功、非预期异常、状态/接触策略泄漏等。有限的随机采样不能证明不存在其他 Bug。',
           '本轮不验证真实跟踪误差、力矩、温升、接触/卡滞或动力学安全，社区手模型和装配仍需实物校准。',
           '本轮只扰动初始 x/y，不扰动朝向、高度、目标测量或初始关节姿态；检查的是完整任务，未单独统计“仅进台/收臂”模式。',
           '接触阶段继承原模型对食指远端与拉环的碰撞放行，未证明接触位置、接触力或无穿透；离散采样也不是连续碰撞安全证明。',
           '耗时是在并发离线测试环境中测得；随机样本可复现，但 OMPL/KDL 求解结果和耗时可能随运行变化。','',
           '逐站位数据：`stances.csv`；逐臂记录：`results/NNN_left.json` 与 `results/NNN_right.json`；成功轨迹在对应子目录的 `.json.gz` 文件内。']
    if summary['bugs']:
        text+=['','## 检出的错误','']
        for bug in summary['bugs']:text.append(f'- #{bug["case"]["id"]} {bug["arm"]}: {bug.get("audit_error",bug["message"])}')
    (root/'REPORT.md').write_text('\n'.join(text)+'\n')
    print(json.dumps({k:summary[k] for k in ('completed_trials','completed_stances','arm_counts','both_pass','max_audited_arc_error_mm')},ensure_ascii=False))
    return cases,paired,summary


def plot(root,cases,paired):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    styles={'PASS':('#16856a','o'),'INITIAL_COLLISION_REJECTED':('#d44f48','x'),
            'NO_COMPLETE_PLAN':('#747982','^'),'TIME_BUDGET_EXCEEDED':('#ce9716','s'),
            'BUG':('#bc38b5','*'),'BUG_EXCEPTION':('#bc38b5','*'),'SERVICE_FAILURE':('#191c23','D'),
            'PENDING':('#d7dadb','.')} 
    fig,axes=plt.subplots(1,2,figsize=(13,5.5),sharex=True,sharey=True)
    for ax,side in zip(axes,('left','right')):
        for outcome,(color,marker) in styles.items():
            points=[c for c in cases if paired[c['id']].get(side,{}).get('outcome','PENDING')==outcome]
            if not points:continue
            ax.scatter([c['nominal_pelvis_front_gap_m']*100 for c in points],[c['dy_m']*100 for c in points],
                       c=color,marker=marker,s=42,label=outcome,alpha=.9)
        ax.axvline(11,color='#333333',ls='--',lw=.8,label='Original gap')
        ax.set_title(side.title()+' hand')
        ax.set_xlabel('Pelvis-front gap (cm); smaller = closer')
        ax.set_xlim(0,18);ax.set_ylim(-13,13);ax.grid(alpha=.2)
    axes[0].set_ylabel('Lateral shift from original stance (cm)')
    handles,labels=[],[]
    for ax in axes:
        h,l=ax.get_legend_handles_labels()
        for handle,label in zip(h,l):
            if label not in labels:handles.append(handle);labels.append(label)
    fig.legend(handles,labels,loc='lower center',ncol=3,fontsize=8)
    fig.tight_layout(rect=(0,.13,1,1));fig.savefig(root/'stance_results.png',dpi=170)
    plt.close(fig)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('root',type=Path);parser.add_argument('--plot',action='store_true')
    args=parser.parse_args();cases,paired,summary=summarize(args.root)
    if args.plot:plot(args.root,cases,paired)


if __name__=='__main__':main()
