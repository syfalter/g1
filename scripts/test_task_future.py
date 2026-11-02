from concurrent.futures import Future
import threading
import unittest
from unittest.mock import patch

from task_future import wait_for_result


class FutureWaitTests(unittest.TestCase):
    def test_already_completed(self):
        future = Future(); future.set_result(42)
        self.assertEqual(wait_for_result(future, lambda: None), 42)

    def test_completion_wakes_wait_without_polling_sleep(self):
        future = Future()
        timer = threading.Timer(.003, lambda: future.set_result('ok'))
        timer.start()
        try:
            with patch('task_future.time.sleep', side_effect=AssertionError('Fixed sleep')):
                self.assertEqual(wait_for_result(future, lambda: None, 1), 'ok')
        finally:
            timer.join()

    def test_stop_before_wait(self):
        def cancelled(): raise RuntimeError('Stopped')
        with self.assertRaisesRegex(RuntimeError, 'Stopped'):
            wait_for_result(Future(), cancelled)

    def test_stop_wins_over_completed_result(self):
        future = Future(); future.set_result('old plan')
        def cancelled(): raise RuntimeError('Stopped')
        with self.assertRaisesRegex(RuntimeError, 'Stopped'):
            wait_for_result(future, cancelled)

    def test_stop_while_pending(self):
        calls = []
        def check():
            calls.append(True)
            if len(calls) > 1: raise RuntimeError('Stopped')
        with self.assertRaisesRegex(RuntimeError, 'Stopped'):
            wait_for_result(Future(), check, 1)

    def test_timeout(self):
        with self.assertRaises(TimeoutError):
            wait_for_result(Future(), lambda: None, .005)

    def test_cancelled_future(self):
        future = Future(); future.cancel()
        with self.assertRaisesRegex(RuntimeError, 'cancelled'):
            wait_for_result(future, lambda: None)

    def test_service_exception(self):
        future = Future(); future.set_exception(ValueError('Rejected'))
        with self.assertRaisesRegex(ValueError, 'Rejected'):
            wait_for_result(future, lambda: None)


if __name__ == '__main__': unittest.main()
