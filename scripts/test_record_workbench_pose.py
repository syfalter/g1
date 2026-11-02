import copy
import json
from pathlib import Path
import unittest

import numpy as np

from record_workbench_pose import build_record, VENDOR_MODEL

ROOT = Path(__file__).resolve().parents[1]


class RecordTests(unittest.TestCase):
    def setUp(self):
        self.profile = json.loads((ROOT / 'assets/workbench/task_start_pose.json').read_text())

    def test_bilateral_local_poses_do_not_depend_on_platform_height(self):
        low = build_record(self.profile, VENDOR_MODEL, 'table_ready', 0.)
        high = build_record(self.profile, VENDOR_MODEL, 'table_ready', .07)
        self.assertEqual(low['wrist_poses'], high['wrist_poses'])
        self.assertEqual(high['joints'], self.profile['joints'])
        self.assertFalse(high['hardware_execution_validated'])
        for frame in ('pelvis', 'torso_link'):
            for side in ('left', 'right'):
                pose = high['wrist_poses'][frame][side]
                self.assertTrue(np.isfinite(pose['position_m']).all())
                self.assertAlmostEqual(np.linalg.norm(pose['quaternion_xyzw']), 1.)

    def test_invalid_labels_and_nonfinite_joints(self):
        for label in ('../unsafe', '', 'a' * 65):
            with self.assertRaises(ValueError):build_record(self.profile, VENDOR_MODEL, label, .07)
        profile = copy.deepcopy(self.profile)
        profile['joints']['left_elbow_joint'] = float('nan')
        with self.assertRaises(ValueError):build_record(profile, VENDOR_MODEL, 'ready', .07)


if __name__ == '__main__':
    unittest.main()
