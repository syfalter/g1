"""Prepare exact gateway interpolation inputs and compare architecture outputs."""
import argparse
import json
import math
from pathlib import Path
from bundle import load, seconds, write


def prepare(bundle_path, output):
    bundle = load(bundle_path)['payload']
    trajectories = []
    labels = []
    # Include dynamic joins from successful baselines as well as cached curves.
    for operation, action in bundle['actions'].items():
        for segment in action['plan']['segments']:
            if segment['kind'] != 'arm':
                continue
            tr = segment['trajectory']
            times = {0., seconds(tr['points'][-1]) - seconds(tr['points'][0])}
            for a, b in zip(tr['points'], tr['points'][1:]):
                t0, t1 = seconds(a), seconds(b)
                for i in range(max(2, math.ceil((t1-t0)/.01)) + 1):
                    n = max(2, math.ceil((t1-t0)/.01))
                    times.add(t0 + (t1-t0)*i/n - seconds(tr['points'][0]))
            queries = sorted(times)
            trajectories.append((tr, queries))
            labels.append(dict(operation=operation, label=segment['label'], rows=len(queries), axes=len(tr['joint_names'])))
    with Path(output).open('w') as stream:
        stream.write(str(len(trajectories)) + '\n')
        for tr, queries in trajectories:
            stream.write(f"{len(tr['joint_names'])} {len(tr['points'])} {len(queries)}\n")
            for p in tr['points']:
                stream.write(' '.join(format(x, '.17g') for x in
                    [seconds(p)] + p['positions'] + p['velocities'] + p['accelerations']) + '\n')
            stream.write('\n'.join(format(t, '.17g') for t in queries) + '\n')
    write(str(output) + '.json', labels)


def compare(first, second, metadata, tolerance=1e-10):
    labels = json.loads(Path(metadata).read_text())
    maxima = [0., 0., 0.]
    count = 0
    with Path(first).open() as a, Path(second).open() as b:
        for segment in labels:
            n = segment['axes']
            for _ in range(segment['rows']):
                x, y = [float(v) for v in a.readline().split()], [float(v) for v in b.readline().split()]
                if len(x) != 3*n or len(y) != len(x) or not all(math.isfinite(v) for v in x+y):
                    raise ValueError('Truncated/nonfinite output')
                for i, (v, w) in enumerate(zip(x, y)):
                    maxima[i//n] = max(maxima[i//n], abs(v-w))
                count += 1
        if a.read().strip() or b.read().strip():
            raise ValueError('Unexpected extra output')
    report = dict(samples=count, segments=len(labels), max_position_rad=maxima[0],
                  max_velocity_rad_s=maxima[1], max_acceleration_rad_s2=maxima[2],
                  tolerance=tolerance, passed=max(maxima) <= tolerance,
                  scope='offline interpolation only; no closed-loop or contact equivalence claim')
    if not report['passed']:
        raise ValueError(str(report))
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    prep = sub.add_parser('prepare')
    prep.add_argument('bundle'); prep.add_argument('output')
    comp = sub.add_parser('compare')
    comp.add_argument('first'); comp.add_argument('second'); comp.add_argument('metadata')
    comp.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.command == 'prepare':
        prepare(args.bundle, args.output)
    else:
        report = compare(args.first, args.second, args.metadata)
        if args.output: write(args.output, report)
        print(json.dumps(report, indent=2))
