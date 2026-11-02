#!/usr/bin/env python3
"""Snapshot official repository metadata and pinned READMEs, without installing code."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess

REPOSITORIES=(
    ('swri-robotics/descartes_light','master'),
    ('tesseract-robotics/tesseract_planning','master'),
    ('tesseract-robotics/trajopt','master'),
    ('ompl/ompl','main'),
    ('moveit/moveit_task_constructor','humble'),
    ('HIRO-group/trac_ik','ros2'),
    ('TAMS-Group/bio_ik','master'),
    ('PickNikRobotics/pick_ik','main'),
    ('pantor/ruckig','main'),
)


def fetch(url,raw=False):
    command=['curl','-fsSL','--max-time','30','--retry','1']
    if raw:command+=['-H','Accept: application/vnd.github.raw+json']
    return subprocess.check_output([*command,url])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    results=[]
    for repository,branch in REPOSITORIES:
        print(repository,flush=True)
        base='https://api.github.com/repos/'+repository
        directory=args.output/repository.replace('/','__');directory.mkdir()
        item=dict(repository=repository,requested_branch=branch,verified=False)
        results.append(item)
        try:
            metadata=fetch(base);(directory/'repository.json').write_bytes(metadata)
            metadata=json.loads(metadata)
            commit=fetch(base+'/commits/'+branch);(directory/'commit.json').write_bytes(commit)
            sha=json.loads(commit)['sha']
            readme=fetch(base+'/readme?ref='+sha,True);(directory/'README.md').write_bytes(readme)
            item.update(verified=True,commit=sha,license=(metadata.get('license') or {}).get('spdx_id'),
                        repository_url=metadata['html_url'],readme_url=base+'/readme?ref='+sha,
                        readme_sha256=hashlib.sha256(readme).hexdigest())
        except (subprocess.CalledProcessError,ValueError,KeyError) as error:
            item['error']=str(error)
        (args.output/'sources.json').write_text(json.dumps(dict(
            retrieved_at=datetime.now(timezone.utc).isoformat(),sources=results),indent=2)+'\n')


if __name__=='__main__':main()
