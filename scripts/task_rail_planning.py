"""Isolated rail planning; this worker creates no ROS nodes or command clients."""
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
from dataclasses import asdict

from task_screen_teach import write_json


def build_isolated(urdf, config, ready, start, operation, mode, parameters, directory,
                   check, progress, *, cache_dir=None, require_cache=False, timeout=120.):
    from rosidl_runtime_py.set_message import set_message_fields
    from trajectory_msgs.msg import JointTrajectory
    directory = Path(directory)/'worker'
    directory.mkdir(parents=True, exist_ok=True)
    request = directory/'request.json'
    output = directory/'result.json'
    status = directory/'progress.json'
    check()
    write_json(request,dict(urdf=str(urdf),config=config,ready=ready,start=start,
        operation=operation,mode=mode,parameters=asdict(parameters),
        cache_dir=str(cache_dir) if cache_dir is not None else None,require_cache=require_cache))
    environment = dict(os.environ, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1')
    began = time.monotonic()
    last = None
    with (directory/'worker.log').open('w') as log:
        process = subprocess.Popen([sys.executable,str(Path(__file__).resolve()),str(request)],
            stdout=log,stderr=log,env=environment)
        try:
            while True:
                check()
                if status.exists():
                    record = json.loads(status.read_text())
                    if record != last:
                        progress(record['message']); last = record
                if process.poll() is not None:break
                if time.monotonic()-began > timeout:
                    raise TimeoutError('滑轨规划超过 120 秒，已取消计算；原始轨迹保留，未发送运动')
                threading.Event().wait(.05)
            check()
            if process.returncode != 0:
                raise RuntimeError('滑轨规划子进程失败：'+(directory/'worker.log').read_text()[-2000:])
            result = json.loads(output.read_text())
            if 'error' in result:raise RuntimeError(result['error'])
            plan = result['plan']
            for segment in plan['segments']:
                trajectory = JointTrajectory()
                set_message_fields(trajectory,segment['trajectory'])
                segment['trajectory'] = trajectory
            check()
            return plan
        finally:
            if process.poll() is None:
                process.terminate()
                try:process.wait(timeout=.5)
                except subprocess.TimeoutExpired:
                    process.kill(); process.wait(timeout=2.)


def main(request):
    from rosidl_runtime_py.convert import message_to_ordereddict
    from workbench_model import OfflineModel
    from task_planning_config import PlanningParameters
    from task_rail_trace import build_replay
    request = Path(request)
    args = json.loads(request.read_text())
    def progress(message):
        write_json(request.parent/'progress.json',dict(message=message))
    try:
        plan = build_replay(OfflineModel(Path(args['urdf'])),args['config'],args['ready'],args['start'],
            args['operation'],args['mode'],PlanningParameters(**args['parameters']),
            cache_dir=args['cache_dir'],require_cache=args['require_cache'],progress=progress)
        for segment in plan['segments']:
            segment['trajectory'] = message_to_ordereddict(segment['trajectory'])
        result = dict(plan=plan)
    except Exception as error:
        result = dict(error=str(error))
    write_json(request.parent/'result.json',result)


if __name__ == '__main__':main(sys.argv[1])
