"""Relabel an offline demonstration slot, preserving original files as backups."""
import argparse
import copy
from datetime import datetime
import json
from pathlib import Path
import shutil


def relabel(paths,expected_recorded_at):
    pending=[]
    for path in map(Path,paths):
        original=path.read_bytes();config=json.loads(original)
        trace=config['traces'].get('left')
        if trace is None or trace.get('recorded_at')!=expected_recorded_at:
            raise ValueError(f'{path}: latest recording changed; no files modified')
        if 'right' in config['traces']:
            raise ValueError(f'{path}: right slot occupied; no files modified')
        changed=copy.deepcopy(config)
        changed['traces']['right']=changed['traces'].pop('left')
        pending.append((path,original,changed))
    stamp=datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    for path,original,changed in pending:
        if path.read_bytes()!=original:raise RuntimeError('Recording changed during relabel')
        backup=path.with_name(path.name+'.before_relabel_'+stamp)
        shutil.copy2(path,backup)
        temp=path.with_name(path.name+'.relabel_'+stamp)
        try:
            with temp.open('x') as stream:json.dump(changed,stream,ensure_ascii=False,indent=2)
            temp.chmod(path.stat().st_mode & 0o777)
            temp.replace(path)
        finally:
            temp.unlink(missing_ok=True)
        print(f'{path}: left -> right; backup: {backup}')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--recorded-at',required=True)
    parser.add_argument('paths',nargs='+')
    args=parser.parse_args()
    relabel(args.paths,args.recorded_at)
