"""Check all pinned/recovery waist polynomials without ROS or robot access."""
import argparse
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np

from automatic_recovery import recovery_route, endpoint
from bundle import ROOT
sys.path.insert(0, str(ROOT / 'scripts'))
from task_trajectory import coefficients, peak, validate_waist_envelope, bernstein_controls


def audit(trajectory):
    points = [SimpleNamespace(**dict(p, time_from_start=SimpleNamespace(**p['time_from_start'])))
              for p in trajectory['points']]
    value = SimpleNamespace(joint_names=trajectory['joint_names'], points=points)
    validate_waist_envelope(value, -.765398, .765398, .35, 1.5)
    axis = value.joint_names.index('waist_yaw_joint')
    maximum, enclosure = 0., 0.
    for a, b in zip(points, points[1:]):
        dt = ((b.time_from_start.sec - a.time_from_start.sec) +
              (b.time_from_start.nanosec - a.time_from_start.nanosec) * 1e-9)
        c = coefficients(a, b, dt)[:, axis]
        velocity = np.array([i * c[i] / dt for i in range(1, 6)])
        maximum = max(maximum, peak(velocity))
        enclosure = max(enclosure, float(np.max(np.abs(bernstein_controls(velocity)))))
    return dict(sample_peak=max(abs(p.velocities[axis]) for p in points),
                continuous_peak=maximum, conservative_envelope=enclosure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bundle', type=Path)
    args = parser.parse_args()
    release = json.loads(args.bundle.read_text())
    results = {}
    for name, action in release['payload']['actions'].items():
        results[name] = [audit(s['trajectory']) for s in action['program'] if s['kind'] == 'recorded']
        if name in ('pull', 'gear_roundtrip_lower_upper', 'gear_roundtrip_upper_lower'):
            trajectory = action['program'][2]['trajectory']
            route = recovery_route(release, name, dict(kind='endpoint', index=2, checkpoint=endpoint(trajectory, -1)))
            audit(route['steps'][-1]['trajectory'])
    print(json.dumps(dict(sha256=release['sha256'], waist_limit=.35, passed=True, actions=results), indent=2))


if __name__ == '__main__':
    main()
