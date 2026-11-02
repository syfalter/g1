"""Pinned offline previews, independent of optimizer caches and current selections."""
import copy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from task_composition_preview import ROOT, materialize
from task_screen_teach import fingerprint, write_json, now
from task_trajectory import time_seconds

DIRECTORY=ROOT/'data/offline_optimizer_previews'
ARRAYS=('times','tcp','joint','acceleration','jerk','joint_matrix','acceleration_matrix','jerk_matrix')


def model_hash(model):return hashlib.sha256(ET.tostring(model.robot)).hexdigest()


def save(results,controls,model,directory=DIRECTORY):
    packed={}
    for name,r in results.items():
        item={k:copy.deepcopy(r[k]) for k in ('payload','sequence','smoothness','joint_names') if k in r}
        item.update({key:np.asarray(r[key]).tolist() for key in ARRAYS if key in r})
        item['seconds']=time_seconds(r['trajectory'].points[-1]);packed[name]=item
    data=dict(schema=1,created=now(),controls=controls,model_sha256=model_hash(model),variants=packed,
              hardware_executable=False)
    digest=fingerprint(data);path=Path(directory)/(digest+'.json')
    write_json(path,dict(sha256=digest,data=data))
    catalog_path=Path(directory)/'catalog.json'
    try:catalog=json.loads(catalog_path.read_text())
    except FileNotFoundError:catalog={}
    catalog[case_key(*controls['case'])]=path.name
    write_json(catalog_path,catalog)
    write_json(Path(directory)/'latest.json',dict(file=path.name))
    return path


def case_key(workflow,slot):return workflow+':'+(slot or 'combination')


def for_case(workflow,slot,directory=DIRECTORY):
    try:
        name=json.loads((Path(directory)/'catalog.json').read_text())[case_key(workflow,slot)]
        if Path(name).name!=name or not name.endswith('.json'):raise ValueError('Invalid snapshot path')
        return Path(directory)/name
    except (FileNotFoundError,KeyError):return None


def latest(directory=DIRECTORY):
    try:
        name=json.loads((Path(directory)/'latest.json').read_text())['file']
        if Path(name).name!=name or not name.endswith('.json'):raise ValueError('Invalid snapshot path')
        return Path(directory)/name
    except FileNotFoundError:return None


def load(path,model):
    saved=json.loads(Path(path).read_text());data=saved['data']
    if data['schema']!=1 or fingerprint(data)!=saved['sha256']:raise ValueError('预览快照校验失败')
    if data['model_sha256']!=model_hash(model):raise ValueError('预览快照的机器人模型不同，请重新生成')
    results={}
    for name,item in data['variants'].items():
        tr,events=materialize(item['payload'],item['sequence'])
        if abs(time_seconds(tr.points[-1])-item['seconds'])>1e-8:raise ValueError('预览快照时长不一致')
        r={k:copy.deepcopy(item[k]) for k in ('payload','sequence','smoothness','joint_names') if k in item}
        for key in ARRAYS:
            if key in item:
                r[key]=np.asarray(item[key],dtype=float)
                if not np.isfinite(r[key]).all():raise ValueError('预览快照曲线数据无效')
        r.update(trajectory=tr,events=events,hit=True,elapsed=0.,cache=str(path),
                 snapshot_id=saved['sha256'][:12],snapshot_created=data['created'])
        results[name]=r
    if not {'original','baseline','candidate'}<=results.keys():raise ValueError('预览快照缺少对照轨迹')
    return dict(variants=results,snapshot_controls=data['controls'],snapshot_path=str(path))


def describe(result):
    payload=result['payload'];options=payload.get('options',{});sources=payload.get('sources',{})
    parts=[]
    for direction,label in (('left','左'),('right','右')):
        source=sources.get(direction,{})
        if 'selectedVersion' in source:parts.append(f'{label} {source["slot"]} v{source["selectedVersion"]}')
    if 'selectedVersion' in sources:parts.append(f'{sources["slot"]} v{sources["selectedVersion"]}')
    parts.append(f'速度倍率 {options.get("rate",1.):g}')
    assembly=options.get('assembly')
    if assembly:
        parts.append(f'次数 {assembly["count"]}')
        if payload['workflow']=='landing_gear':
            parts.append(f'{assembly["start"]} → {assembly["finish"]} · '+('含退出' if assembly['leave'] else '不退出'))
    return ' · '.join(parts)
