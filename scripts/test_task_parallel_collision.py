import concurrent.futures
import multiprocessing
import unittest
from unittest.mock import Mock

from task_parallel_collision import check_batch, validate


class FixtureChecker:
    def check(self, q, side):
        return 'fixture_collision' if q['joint'] > .8 else None

    def check_arm_body(self, q, side):
        return None


def initialize_fixture():
    import task_parallel_collision
    task_parallel_collision._checker = FixtureChecker()


class ParallelTests(unittest.TestCase):
    def run_path(self, pool, path, check=lambda:None):
        counts=[]
        validate(pool, FixtureChecker(), path, {'joint':0.}, 'left', ['joint'],
                 {'joint':(-1.,1.)}, .02, check, counts.append)
        return counts

    def pool(self):
        return concurrent.futures.ProcessPoolExecutor(max_workers=2,
            mp_context=multiprocessing.get_context('spawn'),initializer=initialize_fixture)

    def test_serial_and_processes_cover_identical_samples(self):
        path=[[i/2000.] for i in range(1201)]
        serial=self.run_path(None,path)
        with self.pool() as pool:
            parallel=self.run_path(pool,path)
        self.assertEqual(serial,parallel)
        self.assertEqual(parallel[-1],len(path))

    def test_worker_collision_and_limit_failures_propagate(self):
        with self.pool() as pool:
            for bad,message in ((.9,'collision'),(1.1,'joint limits'),(float('nan'),'joint limits')):
                with self.assertRaisesRegex(ValueError,message):
                    self.run_path(pool,[[.1]]*401+[[bad]])

    def test_cancel_does_not_submit_unbounded_work(self):
        check=Mock(side_effect=RuntimeError('cancelled'))
        pool=Mock()
        with self.assertRaisesRegex(RuntimeError,'cancelled'):
            self.run_path(pool,[[.1]]*10000,check)
        pool.submit.assert_not_called()


if __name__=='__main__':unittest.main()
