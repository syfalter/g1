import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import install_gear_update as installer


class InstallTests(unittest.TestCase):
    def test_detached_requires_no_controller_or_task(self):
        state = dict(initialized=False, controllerRunning=False, activeTaskId=None,
                     native=dict(controlOwner='none', phase='DETACHED'),
                     automaticRecovery=dict(status='idle'))
        installer.detached(state)
        for change in (dict(initialized=True), dict(controllerRunning=True),
                       dict(activeTaskId='task'), dict(native={}),
                       dict(automaticRecovery=dict(status='running'))):
            with self.assertRaises(RuntimeError):
                installer.detached(dict(state, **change))

    def test_preflight_new_pair_without_service_or_motion(self):
        source = Path(__file__).resolve().parents[2] / 'data/rail_deployment/gear_reverse_latency_20260928'
        manifest = json.loads((source / 'manifest.json').read_text())
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            for name in manifest['before']:
                target = root / name
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes((source / 'before' / target.name).read_bytes())
            selected = root / 'config/selected_bundle.json'
            selected.write_bytes((source / 'before/selected_bundle.json').read_bytes())
            state = dict(initialized=False, controllerRunning=False, activeTaskId=None,
                         native=dict(controlOwner='none', phase='DETACHED'),
                         automaticRecovery=dict(status='idle'))
            argv = ['install', '--root', str(root), '--staging', str(source), '--check-only']
            with patch('sys.argv', argv), patch.object(installer, 'health', return_value=state), \
                    patch.object(installer.subprocess, 'run') as run:
                installer.main()
                run.assert_not_called()
                selected.write_text('{}')
                with self.assertRaisesRegex(RuntimeError, 'Selected release changed'):
                    installer.main()


if __name__ == '__main__':
    unittest.main()
