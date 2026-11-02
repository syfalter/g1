"""ROS message serialization tests only; no node or hardware connection."""
import copy
from pathlib import Path
import sys
import threading
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'deployment/rail_jetson'))
from ros_transport import RosTransport, silent_reset_candidate
from builtin_interfaces.msg import Time


def done(value):
    return S(done=lambda: True, result=lambda: value)


class NativeSerializationTests(unittest.TestCase):
    def test_home_refuses_detached_or_unarmed_without_starting_gateway(self):
        for running, epoch, phase in ((False, None, 'DETACHED'), (True, None, 'UNARMED'), (True, 0, 'LATCHED_SILENT')):
            transport = RosTransport.__new__(RosTransport)
            transport.lock = threading.RLock()
            transport.managed_gateway = Mock(running=running)
            transport.epoch = epoch
            transport.gateway = dict(phase=phase)
            with self.assertRaises(RuntimeError):
                transport.return_home({}, lambda: None)
            transport.managed_gateway.start.assert_not_called()

    def test_home_plans_both_arms_and_waist_and_checks_endpoint(self):
        from home_pose import reference
        transport = RosTransport.__new__(RosTransport)
        transport.lock = threading.RLock()
        transport.managed_gateway = Mock(running=True)
        transport.epoch = 0
        transport.gateway = dict(phase='ARMED_HOLD', task_dual_prepare_supported='true')
        transport.verify_configuration = Mock()
        transport.set_stage = Mock()
        transport.stationary = Mock(return_value={'measured': 1})
        transport.planner = Mock(return_value={'trajectory': 1})
        transport.align_check = Mock()
        transport.arm = Mock(return_value=dict(status=4, error_code=0))
        transport.settle = Mock()
        check = Mock()
        result = transport.return_home({'payload': {'actions': {'test': {'plan': {'planning_config': {'values': {}}}}}}}, check)
        transport.planner.assert_called_once_with({'measured': 1}, reference()['target'], {}, .7, .7)
        transport.settle.assert_called_once_with(reference()['target'], {}, check)
        self.assertFalse(result['output_released'])
        self.assertFalse(result['safe_to_power_off'])
        transport.managed_gateway.start.assert_not_called()

    def test_silent_latch_can_use_explicit_reset_only_without_output(self):
        for phase in ('UNARMED', 'LATCHED_SILENT'):
            gateway = dict(phase=phase, armed='false', command_publisher_exists='false')
            self.assertTrue(silent_reset_candidate(gateway))
            for key in ('armed', 'command_publisher_exists'):
                self.assertFalse(silent_reset_candidate(dict(gateway, **{key:'true'})))
        self.assertFalse(silent_reset_candidate({'phase':'LATCHED_SILENT'}))

    def test_explicit_recovery_resets_silent_latch_without_arming(self):
        import time
        transport = RosTransport.__new__(RosTransport)
        transport.managed_gateway = None
        transport.lock = threading.RLock()
        transport.gateway_at = time.monotonic()
        transport.gateway = dict(phase='LATCHED_SILENT', armed='false', command_publisher_exists='false',
                                 task_stop_epoch='2', task_stop_requested='true', task_stop_active='false')
        transport.graph = Mock()
        transport.graph_ok = True
        transport.stopped = threading.Event()
        transport.stopped.set()
        transport.reset = object()
        def reset(client, request, check):
            self.assertIs(client, transport.reset)
            self.assertEqual(request.stop_epoch, 2)
            transport.gateway.update(phase='UNARMED', task_stop_requested='false')
            return S(success=True)
        transport.call = Mock(side_effect=reset)
        self.assertTrue(transport.recover()['requiresInitialization'])
        transport.call.assert_called_once()
        self.assertFalse(transport.stopped.is_set())
        self.assertIsNone(transport.epoch)

    def test_detached_stop_does_not_call_wsl_stop_or_hands(self):
        transport = RosTransport.__new__(RosTransport)
        transport.managed_gateway = S(running=False)
        transport.stopped = threading.Event()
        transport.services, transport.hand = Mock(), Mock()
        self.assertTrue(transport.stop_and_confirm())
        self.assertTrue(transport.stopped.is_set())
        self.assertEqual(transport.services.mock_calls, [])
        self.assertEqual(transport.hand.mock_calls, [])

    def test_detached_r2a_never_starts_controller(self):
        transport = RosTransport.__new__(RosTransport)
        transport.managed_gateway = Mock(running=False)
        transport.stopped = threading.Event()
        transport.stopped.set()
        self.assertFalse(transport.recover_from_remote()['controllerRunning'])
        transport.managed_gateway.start.assert_not_called()
        self.assertFalse(transport.stopped.is_set())

    def test_ros_conversion_preserves_frozen_json_and_endpoint(self):
        transport = RosTransport.__new__(RosTransport)
        transport.command_lock = threading.RLock()
        transport.hand_enabled = False
        transport.stage_revision = 3
        transport.gateway = dict(task_stage_revision='3')
        transport.node = S(get_clock=lambda: S(now=lambda: S(to_msg=lambda: Time())))
        result = S(status=4, result=S(error_code=0, error_string=''))
        goal = S(accepted=True, get_result_async=lambda: done(result))
        client = Mock()
        client.wait_for_server.return_value = True
        client.send_goal_async.return_value = done(goal)
        transport.actions = {'left': client}
        names = ['left_shoulder_pitch_joint']
        trajectory = dict(joint_names=names, points=[dict(positions=[q], velocities=[0.], accelerations=[0.],
                          time_from_start=dict(sec=i, nanosec=0)) for i, q in enumerate((0., .1))])
        original = copy.deepcopy(trajectory)
        for _ in range(2):
            outcome = transport.arm(dict(side='left'), trajectory, lambda: None)
            self.assertEqual(outcome['status'], 4)
            self.assertEqual(trajectory, original)
            self.assertEqual(trajectory['points'][-1]['positions'], [.1])
        self.assertEqual(client.send_goal_async.call_count, 2)


if __name__ == '__main__':
    unittest.main()
