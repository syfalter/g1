import copy
import unittest
from dds_authority import LOWCMD, native_endpoints, validate


def discovery():
    return dict(participants={'robot': dict(process='g1_arm_example', host='Unitree', pid='1051')},
                writers=[dict(guid='builtin', participant='robot', type=LOWCMD)],
                readers=[dict(guid='input', participant='robot', type=LOWCMD)],
                messages={}, unknown_messages=0)


class AuthorityTests(unittest.TestCase):
    def test_idle_native_service_is_not_a_competing_controller(self):
        state = discovery()
        self.assertEqual(native_endpoints(state), {'builtin'})
        validate(state, {'builtin'})

    def test_bare_dds_name_alone_is_not_trusted(self):
        state = discovery()
        state['participants']['robot']['process'] = 'some_sdk_client'
        with self.assertRaises(RuntimeError):
            validate(state, {'builtin'})

    def test_native_endpoint_must_also_receive_commands(self):
        state = discovery()
        state['readers'] = []
        with self.assertRaises(RuntimeError):
            native_endpoints(state)

    def test_native_command_activity_blocks_takeover(self):
        state = discovery()
        state['messages']['builtin'] = 1
        with self.assertRaisesRegex(RuntimeError, 'sent arm commands'):
            validate(state, {'builtin'})

    def test_unknown_sample_blocks(self):
        state = discovery()
        state['unknown_messages'] = 1
        with self.assertRaises(RuntimeError):
            validate(state, {'builtin'})

    def test_other_writer_blocked_even_without_samples(self):
        state = discovery()
        state['writers'].append(dict(guid='wsl', participant='pc', type=LOWCMD))
        with self.assertRaisesRegex(RuntimeError, 'Another arm command'):
            validate(state, {'builtin'})

    def test_own_commands_and_release_history(self):
        state = discovery()
        state['writers'].append(dict(guid='gateway', participant='jetson', type=LOWCMD))
        state['messages']['gateway'] = 100
        validate(state, {'builtin'}, {'gateway'})
        released = copy.deepcopy(state)
        released['writers'].pop()
        validate(released, {'builtin'}, past_own={'gateway'})
        with self.assertRaises(RuntimeError):
            validate(state, {'builtin'}, past_own={'gateway'})

    def test_native_restart_requires_new_admission(self):
        with self.assertRaisesRegex(RuntimeError, 'identity changed'):
            validate(discovery(), {'old-builtin'})


if __name__ == '__main__':
    unittest.main()
