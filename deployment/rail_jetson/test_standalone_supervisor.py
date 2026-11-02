from types import SimpleNamespace
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from standalone_supervisor import output_may_be_armed, publishers, runtime_parameters, wait_ready


class StartupTests(unittest.TestCase):
    def test_explicit_runtime_file_preserves_selected_defaults(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root / 'config').mkdir()
            path = root / 'config/native.yaml'
            config = {'g1_arm_sdk_hardware_gateway': {'ros__parameters': {
                'hardware_output_enabled': False, 'hardware_authorization': '', 'max_velocity_rad_s': .4}}}
            path.write_text(json.dumps(config))
            result = json.loads(runtime_parameters(root).read_text())['g1_arm_sdk_hardware_gateway']['ros__parameters']
            self.assertTrue(result['hardware_output_enabled'])
            self.assertEqual(result['hardware_authorization'], 'I_UNDERSTAND_ARM_SDK_OUTPUT')
            self.assertEqual(result['max_velocity_rad_s'], .4)
            self.assertEqual(json.loads(path.read_text()), config)

    def test_reports_diagnostic_only_owner(self):
        info = SimpleNamespace(node_name='wsl_gateway', node_namespace='/', endpoint_gid=[1, 2])
        node = Mock()
        node.get_publishers_info_by_topic.side_effect = [[], [info]]
        result = publishers(node)
        self.assertTrue(any(result.values()))
        self.assertEqual(result['/g1/arm_gateway/diagnostics'][0]['node'], 'wsl_gateway')

    def test_unknown_output_retains_support(self):
        for state in ({}, {'native': {'gateway': {}}}, {'native': {'gateway': {'armed': 'true'}}}):
            self.assertTrue(output_may_be_armed(state))
        self.assertFalse(output_may_be_armed({'native': {'gateway': {'armed': 'false'}}}))

    def test_waits_for_http_instead_of_process_spawn(self):
        reader = Mock(side_effect=[ConnectionRefusedError(), {'ready': True}])
        with patch('standalone_supervisor.time.sleep'):
            self.assertEqual(wait_ready(reader, [], 'api'), {'ready': True})
        self.assertEqual(reader.call_count, 2)

    def test_reports_failed_process(self):
        child = Mock()
        child.poll.return_value = 7
        with self.assertRaisesRegex(RuntimeError, 'worker exited.*7'):
            wait_ready(Mock(), [('worker', child)], 'api')

    def test_wrong_backend_fails_without_retry(self):
        reader = Mock(side_effect=RuntimeError('backend mismatch'))
        with self.assertRaisesRegex(RuntimeError, 'backend mismatch'):
            wait_ready(reader, [], 'api')
        reader.assert_called_once()

    def test_interrupted_startup(self):
        reader = Mock()
        with self.assertRaisesRegex(RuntimeError, 'interrupted'):
            wait_ready(reader, [], 'api', stopping=lambda: True)
        reader.assert_not_called()


if __name__ == '__main__':
    unittest.main()
