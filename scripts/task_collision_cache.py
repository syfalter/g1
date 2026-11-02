"""Opt-in reuse of exact, scoped collision checks, never execution authorization."""
import hashlib
import json
import os
from pathlib import Path
import tempfile

from task_plan_archive import digest

ROOT=Path(__file__).resolve().parents[1]
DEFAULT_CACHE=ROOT/'data/trajectory_collision_checks'


def context(archive):
    import task_hook_collision
    import numpy
    import scipy
    files=['task_collision_cache.py','task_saved_test.py','task_parallel_collision.py',
           'task_hook_collision.py','task_hook_mesh_export.py','workbench_model.py',
           'task_trajectory.py','task_trajectory_speed.py','task_model_resources.py']
    native=list(Path(task_hook_collision.fcl.__file__).parent.rglob('*.so*'))
    return dict(version=1,archive_sha256=archive['manifest']['sha256'],
                scene=archive['plan']['contact_scene'],
                implementation={name:digest(ROOT/'scripts'/name) for name in files},
                native={str(path):digest(path) for path in native if path.is_file()},
                numpy=numpy.__version__,scipy=scipy.__version__)


def certificate(context, segment):
    from task_saved_test import range_fingerprint
    evidence=dict(context=context,trajectory_sha256=range_fingerprint([segment]),
                  stage=segment['number'],scope='sampled_hand_base_and_arm_body')
    key=hashlib.sha256(json.dumps(evidence,sort_keys=True,allow_nan=False).encode()).hexdigest()
    return dict(key=key,evidence=evidence,passed=True)


def matches(record, directory=DEFAULT_CACHE):
    try:
        stored=json.loads((Path(directory)/(record['key']+'.json')).read_text())
        return stored==record
    except (OSError,ValueError,TypeError):
        return False


def remember(records, directory=DEFAULT_CACHE):
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
    for record in records:
        if record.get('passed') is not True:
            raise ValueError('Cannot remember an incomplete collision check')
        expected=hashlib.sha256(json.dumps(record['evidence'],sort_keys=True,allow_nan=False).encode()).hexdigest()
        if expected!=record['key']:
            raise ValueError('Collision certificate changed')
        temporary=None
        try:
            with tempfile.NamedTemporaryFile(mode='w',dir=directory,suffix='.tmp',delete=False) as output:
                temporary=Path(output.name)
                json.dump(record,output,sort_keys=True,allow_nan=False)
            os.replace(temporary,directory/(record['key']+'.json'))
        finally:
            if temporary is not None:temporary.unlink(missing_ok=True)
