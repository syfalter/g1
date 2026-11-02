"""CPU-only validation timing; does not create ROS nodes or submit tasks."""
import argparse
import copy
import json
from pathlib import Path
import time

from bundle import read
from releases import validate
from release_validation_cache import ReleaseValidationCache, PinnedRelease


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--release', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    selected = read(args.release)
    cache = ReleaseValidationCache()
    cache.snapshot(selected)
    pinned = PinnedRelease(selected)
    timings = {}
    for name, operation in (
            ('previous_copy_and_full_validation', lambda: validate(copy.deepcopy(selected))),
            ('cached_copy_and_integrity_check', lambda: cache.snapshot(selected)),
            ('pinned_immutable_release', lambda: cache.snapshot(pinned))):
        samples = []
        for _ in range(3):
            began = time.monotonic()
            result = operation()
            samples.append(time.monotonic()-began)
            assert result['sha256'] == selected['sha256']
        timings[name] = samples
    report = dict(release_sha256=selected['sha256'], timings_seconds=timings,
                  hardware_commands_sent=0, includes_robot_startup=False)
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
