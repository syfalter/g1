import unittest
from robot_request import callback_binding


class CallbackAddressTests(unittest.TestCase):
    def test_local_callback_stays_loopback(self):
        self.assertEqual(callback_binding('http://127.0.0.1:8795', None), ('127.0.0.1', '127.0.0.1'))

    def test_remote_requires_pc_address(self):
        for host in (None, '127.0.0.1', 'localhost', '0.0.0.0', 'http://192.168.0.20'):
            with self.assertRaises(ValueError):
                callback_binding('http://192.168.0.249:8795', host)

    def test_remote_binds_receiver_on_pc(self):
        self.assertEqual(callback_binding('http://192.168.0.249:8795', '192.168.0.20'), ('0.0.0.0', '192.168.0.20'))


if __name__ == '__main__':
    unittest.main()
