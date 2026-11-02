"""Diagnose old ARM64 NumPy matmul warnings without any robot I/O."""
import argparse
import json
import sys
import warnings

from bundle import ROOT
from executor import LocalJoinPlanner, materialize
from releases import load_release


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--release', required=True)
    parser.add_argument('--urdf', required=True)
    args = parser.parse_args()
    sys.path.insert(0, str(ROOT / 'scripts'))
    import numpy as np
    import task_trajectory
    original = task_trajectory.bernstein_controls
    report = dict(calls=0, warnings=0, max_difference=0., finite=True)
    def checked(power):
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter('always')
            result = original(power)
        scalar = np.einsum('ij,j...->i...', task_trajectory._bernstein_matrix(len(power)-1), power, optimize=False)
        report['calls'] += 1
        report['warnings'] += len(caught)
        report['max_difference'] = max(report['max_difference'], float(np.max(np.abs(result-scalar))))
        report['finite'] = report['finite'] and bool(np.isfinite(power).all() and np.isfinite(result).all())
        return result
    task_trajectory.bernstein_controls = checked
    release = load_release(args.release)
    materialize(release, 'dring_left', release['payload']['initialization']['target'], LocalJoinPlanner(args.urdf))
    print(json.dumps(report))
    if not report['finite'] or report['max_difference'] > 1e-10:
        raise RuntimeError('Bernstein arithmetic differs or is not finite')


if __name__ == '__main__':
    main()
