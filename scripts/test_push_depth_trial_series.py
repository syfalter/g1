"""Offline regression checks for the incremental push-only trial releases."""
import hashlib
import unittest

from trajectory_only_candidates import ROOT, read, digest
from trajectory_push_depth import compensation
from releases import validate

DIRECTORY = ROOT / 'data/rail_deployment/push_depth_trial_series_20260929'


class DepthTrials(unittest.TestCase):
    def test_amplitude_bound(self):
        for value in (-1, 1, 2, 9.1, 10, float('nan')):
            with self.assertRaises(ValueError):
                compensation(None, None, {}, [1, 0, 0], value, (0, 1, 2, 3))

    def test_all_trials_preserve_other_actions_and_endpoints(self):
        parent = read(DIRECTORY / 'jetson_readonly/config/selected_bundle.json')
        for mm in range(4, 10):
            stage = DIRECTORY / ('trial_' + str(mm) + 'mm')
            manifest = read(stage / 'export_report.json')
            release = validate(read(stage / 'data_only/config/selected_bundle.json'))
            self.assertFalse(manifest['user_confirmed_disabled'])
            self.assertFalse(manifest['physical_verified'])
            self.assertEqual(release['sha256'], digest(release['payload']))
            for key, previous in parent['payload'].items():
                if key not in ('actions', 'version', 'parents'):
                    self.assertEqual(previous, release['payload'][key])
            for name, previous in parent['payload']['actions'].items():
                new = release['payload']['actions'][name]
                if name != 'push':
                    self.assertEqual(previous, new)
                    continue
                self.assertEqual(len(previous['program']), len(new['program']))
                for a, b in zip(previous['program'], new['program']):
                    if a['kind'] != 'recorded':
                        self.assertEqual(a, b)
                    else:
                        for index in (0, -1):
                            for key in ('positions', 'velocities', 'accelerations'):
                                self.assertEqual(a['trajectory']['points'][index][key], b['trajectory']['points'][index][key])
            for name, expected in manifest['files'].items():
                path = stage / 'data_only' / name
                self.assertEqual(path.suffix, '.json')
                self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), expected)
            for name in ('gear_protocol.json', 'dring_protocol.json'):
                self.assertEqual((stage / 'data_only/config' / name).read_bytes(),
                                 (DIRECTORY / 'jetson_readonly/config' / name).read_bytes())


if __name__ == '__main__':
    unittest.main()
