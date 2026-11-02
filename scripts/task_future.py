"""Interruptible future waiting without a fixed delay for every ROS response."""
import threading
import time


def wait_for_result(future, check, timeout=15):
    wake = threading.Event()
    future.add_done_callback(lambda _: wake.set())
    deadline = time.monotonic() + timeout
    while not future.done():
        check()
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError('ROS response timed out')
        # Keep cancellation checks bounded, but wake immediately on completion.
        wake.wait(min(.02, remaining))
    check()
    if future.cancelled():
        raise RuntimeError('ROS request was cancelled')
    return future.result()
