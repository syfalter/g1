#!/usr/bin/env python3
"""Publish byte-identical verified offline archives; never create execution authorization."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import math
from pathlib import Path
import re
import shutil
import tempfile

from task_plan_archive import digest,write_json
from task_saved_test import load_archive
from task_trajectory_library import DEFAULT_LIBRARY


def publish(summary_path,library,workers=4):
    summary_path=Path(summary_path).resolve();batch=summary_path.parent
    summary=json.loads(summary_path.read_text())
    if len(summary['results'])!=summary['selected_contexts']:
        raise ValueError('Only a completed benchmark batch may be published')
    selected=[row for row in summary['results'] if row.get('complete_archive_pass')]
    if not selected:raise ValueError('No validated archives to publish')
    identifiers=[row['case_id'] for row in selected]
    if len(set(identifiers))!=len(identifiers) or any(not re.fullmatch(r'[a-f0-9]{20}',i) for i in identifiers):
        raise ValueError('Invalid or duplicate archive case identifiers')
    for row in selected:
        seconds=row.get('case_wall_seconds',math.inf)
        if (row.get('status')!='archive_reload_pass' or not row.get('code_integrity_verified')
                or row.get('hardware_commands_sent')!=0 or not math.isfinite(seconds) or not 0<seconds<=180):
            raise ValueError('Archive is not an integrity-verified 180-second pass')
    library=Path(library).resolve();library.mkdir(parents=True,exist_ok=True)
    destination=library/batch.name
    if destination.exists():raise FileExistsError(destination)
    # Keep incomplete copies outside the catalog root; expose the whole batch atomically.
    with tempfile.TemporaryDirectory(prefix='.publish-archives-',dir=library.parent) as temporary:
        staging=Path(temporary)/batch.name;staging.mkdir()
        def copy_archive(row):
            source=Path(row['archive']['directory']).resolve()
            if not source.is_relative_to(batch) or any(p.is_symlink() for p in source.rglob('*')):
                raise ValueError('Unsafe benchmark archive path')
            original=load_archive(source,source/'context/robot.urdf')
            if original['manifest']['original_plan_sha256']!=row['archive']['original_plan_sha256']:
                raise ValueError('Benchmark archive identity changed')
            target=staging/row['case_id'];shutil.copytree(source,target)
            loaded=load_archive(target,target/'context/robot.urdf')
            if loaded['manifest']!=original['manifest'] or digest(source/'archive_manifest.json')!=digest(target/'archive_manifest.json'):
                raise ValueError('Published archive differs from benchmark archive')
            return dict(case_id=row['case_id'],pair_id=row['pair_id'],direction=row['direction'],
                source=str(source),directory=str(destination/row['case_id']),
                archive_manifest_sha256=digest(target/'archive_manifest.json'),
                complete_case_seconds=row['case_wall_seconds'],hardware_executable=False)
        with ThreadPoolExecutor(max_workers=workers) as pool:entries=list(pool.map(copy_archive,selected))
        publication=dict(benchmark=str(summary_path),benchmark_sha256=digest(summary_path),
            manifest_sha256=summary['manifest_sha256'],archive_count=len(entries),entries=entries,
            hardware_commands_sent=0,hardware_executable=False,
            note='Immutable copies; publication time does not change benchmark deadline credit')
        write_json(staging/'publication.json',publication)
        (staging/'publication.json').chmod(0o444)
        if destination.exists():raise FileExistsError(destination)
        staging.rename(destination)
    return publication


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--summary',type=Path,required=True)
    parser.add_argument('--library',type=Path,default=DEFAULT_LIBRARY)
    parser.add_argument('--workers',type=int,choices=range(1,9),default=4)
    args=parser.parse_args()
    result=publish(args.summary,args.library,args.workers)
    print(json.dumps({key:value for key,value in result.items() if key!='entries'},indent=2))


if __name__=='__main__':main()
