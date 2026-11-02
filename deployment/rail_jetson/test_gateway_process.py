from pathlib import Path
import signal
import unittest
from unittest.mock import Mock

from gateway_process import GatewayProcess, silent


class GatewayProcessTests(unittest.TestCase):
    def test_unknown_or_armed_is_not_silent(self):
        for state in ({}, {'armed': 'false'}, {'armed': 'true', 'command_publisher_exists': 'true'}):
            self.assertFalse(silent(state))

    def test_never_terminates_unknown_or_stale_output(self):
        manager = GatewayProcess(Path('/unused'))
        manager.process = Mock()
        manager.process.poll.return_value = None
        for state, fresh in (({}, True), ({'phase': 'LATCHED_SILENT', 'armed': 'false',
                                          'command_publisher_exists': 'false'}, False)):
            with self.assertRaises(RuntimeError):
                manager.retire(state, fresh)
        manager.process.send_signal.assert_not_called()

    def test_latched_silent_child_is_reaped(self):
        manager = GatewayProcess(Path('/unused'))
        process = manager.process = Mock()
        process.poll.return_value = None
        manager.retire({'phase': 'LATCHED_SILENT', 'armed': 'false', 'command_publisher_exists': 'false'}, True)
        process.send_signal.assert_called_once_with(signal.SIGINT)
        process.wait.assert_called_once()
        self.assertFalse(manager.running)
