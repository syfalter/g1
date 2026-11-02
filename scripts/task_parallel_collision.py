"""Bounded, process-isolated collision checks; no ROS nodes or command clients."""
from collections import deque
from concurrent.futures import ProcessPoolExecutor, TimeoutError
from itertools import islice
import multiprocessing
import os


_checker = None


def initialize(urdf, base_top):
    global _checker
    from workbench_model import OfflineModel
    from task_hook_collision import HandBaseCollision
    _checker = HandBaseCollision(OfflineModel(urdf), urdf, base_top)


def check_batch(checker, start, side, names, limits, margin, positions):
    q = dict(start)
    for values in positions:
        q.update(zip(names, map(float, values)))
        for name, (lo, hi) in limits.items():
            if not lo+margin-1e-7 <= q[name] <= hi-margin+1e-7:
                raise ValueError('Saved test violates joint limits: '+name)
        link = checker.check(q, side) if checker is not None else None
        pair = checker.check_arm_body(q, side) if checker is not None else None
        if link or pair:
            raise ValueError('Saved test scoped collision: '+str(link or pair))
    return len(positions)


def worker_batch(*args):
    return check_batch(_checker, *args)


def create_pool(urdf, base_top, workers=None):
    workers = workers or min(4, max(1, (os.cpu_count() or 2)-1))
    return ProcessPoolExecutor(max_workers=workers, mp_context=multiprocessing.get_context('spawn'),
                               initializer=initialize, initargs=(str(urdf), base_top))


def validate(pool, checker, path, start, side, names, limits, margin, check, progress):
    """Keep at most eight 200-sample batches outstanding, including on cancellation."""
    path = iter(path)
    pending = deque()
    count = 0
    exhausted = False
    try:
        while not exhausted or pending:
            check()
            while not exhausted and len(pending) < (8 if pool else 1):
                check()
                batch = list(islice(path, 200))
                exhausted = not batch
                if exhausted:
                    break
                args = (start, side, names, limits, margin, batch)
                if pool:
                    pending.append(pool.submit(worker_batch, *args))
                else:
                    count += check_batch(checker, *args)
                    progress(count)
                    break
            if pending:
                future = pending[0]
                while True:
                    check()
                    try:
                        count += future.result(timeout=.1)
                        break
                    except TimeoutError:
                        if future.done():
                            future.result()
                        continue
                pending.popleft()
                progress(count)
    finally:
        for future in pending:
            future.cancel()
