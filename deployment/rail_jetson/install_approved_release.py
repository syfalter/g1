"""Install a staged release only while the real controller is detached.

No initialization, recovery, control acquisition or motion request is issued.
The prior selection and modified Python modules are saved for rollback.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time
import urllib.request


def health():
    opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
    with opener.open('http://127.0.0.1:8795/health',timeout=10) as response:
        return json.load(response)


def detached(state):
    native=state.get('native',{})
    return (state.get('activeTaskId') is None and not state.get('initialized')
            and not state.get('controllerRunning') and native.get('phase')=='DETACHED'
            and native.get('controlOwner')=='none')


def install(root, staged):
    root,staged=Path(root).resolve(),Path(staged).resolve()
    manifest=json.loads((staged/'install_manifest.json').read_text())
    for relative,expected in manifest['files'].items():
        if Path(relative).is_absolute() or '..' in Path(relative).parts:
            raise ValueError('Invalid staged path')
        if hashlib.sha256((staged/relative).read_bytes()).hexdigest()!=expected:
            raise ValueError('Staged file checksum mismatch: '+relative)
    if not detached(health()):
        raise RuntimeError('Refusing deployment: robot executor must be idle, uninitialized and DETACHED')
    replacements={
        'deployment/rail_jetson/releases.py':'runtime/releases.py',
        'deployment/rail_jetson/cloud_executor.py':'runtime/cloud_executor.py',
    }
    for relative,expected in manifest['previous_code'].items():
        if hashlib.sha256((root/relative).read_bytes()).hexdigest()!=expected:
            raise RuntimeError('Remote code changed since inspection: '+relative)
    backup=root/'data/deployment_backups'/(manifest['version']+'-'+str(time.time_ns()))
    backup.mkdir(parents=True,exist_ok=False)
    originals=list(replacements)+['config/selected_bundle.json','config/native.yaml']
    for relative in originals:
        destination=backup/relative;destination.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(root/relative,destination)
    subprocess.run(['sudo','-n','systemctl','stop','eln_executor.service'],check=True,timeout=190)
    status=subprocess.run(['systemctl','is-active','eln_executor.service'],capture_output=True,text=True)
    if status.stdout.strip() not in ('inactive','failed'):
        raise RuntimeError('Executor did not stop; no release changed')
    sys.path.insert(0,str(root/'deployment/rail_jetson'))
    try:
        for relative,source in replacements.items():
            path=root/relative;tmp=path.with_suffix('.deployment-tmp')
            shutil.copy2(staged/source,tmp);tmp.replace(path)
        from releases import load_release,ReleaseStore
        from select_production import configure
        envelope=load_release(staged/'bundle.json')
        if envelope['sha256']!=manifest['release_sha256']:
            raise ValueError('Wrong release payload')
        ReleaseStore(root/'data/action_releases').install(envelope)
        receipt=configure(root,[staged/'bundle.json'])
        subprocess.run(['sudo','-n','systemctl','start','eln_executor.service'],check=True,timeout=170)
        for _ in range(30):
            state=health()
            if state['releaseSha256']==receipt['sha256'] and detached(state):break
            time.sleep(1)
        else:raise RuntimeError('New executor did not become passive with the selected release')
        receipt.update(sourceVersion=envelope['payload']['version'],backup=str(backup),
                       service_started=True,initialized=False,hardware_commands_sent=0,
                       availableActions=state['availableActions'])
        (staged/'activation_receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
        print(json.dumps(receipt,indent=2))
    except Exception:
        # Never stop a controller that acquired physical ownership concurrently.
        try:
            state=health()
        except Exception:
            state=None
        if state is not None and not detached(state):
            raise RuntimeError('Deployment failed and executor is not detached; rollback deferred: '+str(backup))
        subprocess.run(['sudo','-n','systemctl','stop','eln_executor.service'],check=True,timeout=190)
        for relative in originals:shutil.copy2(backup/relative,root/relative)
        subprocess.run(['sudo','-n','systemctl','start','eln_executor.service'],check=True,timeout=170)
        raise


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,required=True)
    parser.add_argument('--staged',type=Path,required=True)
    args=parser.parse_args();install(args.root,args.staged)
