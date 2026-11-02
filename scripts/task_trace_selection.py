"""Small atomic selection journal tied to an unchanged trajectory bundle."""
import json
from pathlib import Path

from task_screen_teach import write_json
from task_trace_versions import select_shared


def revision(path):
    stat=Path(path).stat()
    return dict(device=stat.st_dev,inode=stat.st_ino,size=stat.st_size,
                modified_ns=stat.st_mtime_ns,changed_ns=stat.st_ctime_ns)


def journal(path):return Path(str(path)+'.selection.json')


def save(config,path,expected):
    path=Path(path)
    if revision(path)!=expected:raise ValueError('轨迹文件已被外部修改，请重新载入后选择版本')
    selections={slot:entry['selected'] for slot,entry in config.get('trace_versions',{}).items()}
    write_json(journal(path),dict(schema=1,source=expected,selected=selections))


def restore(config,path):
    source=Path(path);sidecar=journal(source)
    if not sidecar.exists():return config
    if sidecar.stat().st_size>1000000:raise ValueError('版本选择记录过大')
    data=json.loads(sidecar.read_text())
    if data.get('schema')!=1:raise ValueError('版本选择记录格式无效')
    # Any full save/replacement supersedes the journal, including deletion and rerecording.
    if data.get('source')!=revision(source):return config
    selections=data.get('selected')
    if not isinstance(selections,dict):raise ValueError('版本选择记录无效')
    result=config
    for slot,index in selections.items():
        if slot not in config.get('trace_versions',{}):raise ValueError('版本选择记录对应的轨迹不存在')
        result=select_shared(result,slot,index)
    return result
