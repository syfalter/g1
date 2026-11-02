import json
import math
from pathlib import Path
import unittest

from bundle import digest
from home_pose import reference


class HomePoseTests(unittest.TestCase):
    def test_reference_matches_operator_confirmed_history(self):
        home = reference()
        path = Path(__file__).resolve().parents[2] / 'data' / home['source']
        if not path.exists():
            self.skipTest('Historical session exists in the WSL source workspace only')
        historical = json.loads(path.read_text())
        self.assertIs(historical['operator_confirmed_r2_a'], True)
        self.assertEqual(historical['capture_mode'], 'R2_A')
        self.assertEqual(home['captured_at'], historical['captured_at'])
        self.assertEqual(home['target'], historical['joints'])

    def test_only_both_arms_and_waist_are_targeted(self):
        home = reference()
        suffixes = ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow',
                    'wrist_roll', 'wrist_pitch', 'wrist_yaw')
        names = {side + '_' + suffix + '_joint' for side in ('left', 'right') for suffix in suffixes}
        self.assertEqual(set(home['target']), names | {'waist_yaw_joint'})
        self.assertTrue(all(math.isfinite(value) for value in home['target'].values()))
        self.assertEqual(home['sha256'], digest(home['target']))

    def test_caller_cannot_modify_future_reference(self):
        original = reference()
        changed = reference()
        changed['target']['waist_yaw_joint'] = 10.
        self.assertEqual(reference(), original)


if __name__ == '__main__':
    unittest.main()
