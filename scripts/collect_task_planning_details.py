#!/usr/bin/env python3
"""Fetch pinned primary license/build evidence without installing solver dependencies."""
import argparse
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path

from collect_task_planning_sources import fetch


def collect(source,snapshots,output):
    repository=source['repository'];slug=repository.replace('/','__')
    directory=output/slug;directory.mkdir()
    record=dict(repository=repository,files=[],errors=[])
    try:
        sha=source.get('commit')
        if sha is None:
            metadata=json.loads((snapshots/slug/'repository.json').read_text())
            branch=metadata['default_branch']
            commit=fetch('https://api.github.com/repos/'+repository+'/commits/'+branch)
            (directory/'commit.json').write_bytes(commit);sha=json.loads(commit)['sha']
            record['branch_resolution']=dict(failed_requested_branch=source['requested_branch'],actual_branch=branch,
                                            note='Default branch evidence only; not a ROS2 compatibility claim')
            readme=fetch('https://api.github.com/repos/'+repository+'/readme?ref='+sha,True)
            (directory/'README.md').write_bytes(readme)
        record['commit']=sha
        tree=fetch('https://api.github.com/repos/'+repository+'/git/trees/'+sha+'?recursive=1')
        (directory/'tree.json').write_bytes(tree)
        paths=[r['path'] for r in json.loads(tree)['tree'] if r['type']=='blob']
        licenses=sorted((p for p in paths if Path(p).name.lower().startswith(('license','copying'))),
                        key=lambda p:(p.count('/'),p))
        packages=sorted((p for p in paths if Path(p).name=='package.xml'),key=lambda p:(p.count('/'),p))
        record['license_paths']=licenses;record['package_paths']=packages
        selected=licenses[:5]+packages[:3]+(['CMakeLists.txt'] if 'CMakeLists.txt' in paths else [])
        for name in selected:
            url=f'https://raw.githubusercontent.com/{repository}/{sha}/{name}'
            try:
                content=fetch(url);destination=directory/'files'/name
                destination.parent.mkdir(parents=True,exist_ok=True);destination.write_bytes(content)
                record['files'].append(dict(path=name,url=url,sha256=hashlib.sha256(content).hexdigest()))
            except Exception as error:record['errors'].append(dict(path=name,error=str(error)))
    except Exception as error:record['errors'].append(dict(error=str(error)))
    (directory/'evidence.json').write_text(json.dumps(record,indent=2)+'\n')
    print(repository+': '+str(len(record['files']))+' files, '+str(len(record['errors']))+' errors',flush=True)
    return record


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sources',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    sources=json.loads(args.sources.read_text())['sources']
    with ThreadPoolExecutor(max_workers=3) as pool:
        results=list(pool.map(lambda s:collect(s,args.sources.parent,args.output),sources))
    (args.output/'details.json').write_text(json.dumps(dict(
        retrieved_at=datetime.now(timezone.utc).isoformat(),source_manifest_sha256=hashlib.sha256(args.sources.read_bytes()).hexdigest(),
        sources=results,build_tested=False),indent=2)+'\n')


if __name__=='__main__':main()
