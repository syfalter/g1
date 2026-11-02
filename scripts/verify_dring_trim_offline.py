"""Read-only regression on preserved recordings; no ROS node or robot services."""
import argparse
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np

from task_dring_teach_data import arrays,fit_trace
from task_dring_trim import cut


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('bundle',type=Path);parser.add_argument('robot',type=Path)
    parser.add_argument('--direction',choices=('left','right'),required=True)
    args=parser.parse_args();original=args.bundle.read_bytes();config=json.loads(original)
    robot=ET.fromstring(args.robot.read_bytes())
    limits={j.attrib['name']:(float(j.find('limit').attrib['lower']),float(j.find('limit').attrib['upper']))
            for j in robot.findall('joint') if j.find('limit') is not None and 'lower' in j.find('limit').attrib}
    model=SimpleNamespace(robot=robot,limits=limits)
    for slot,trace in config['traces'].items():
        if not slot.startswith(args.direction):continue
        duration=trace['raw']['time_s'][-1]
        edited=cut(trace,.35*duration,.45*duration,.6)
        q,t=arrays(edited,model,config['side'])
        np.testing.assert_array_equal(q[[0,-1]],np.asarray(trace['raw']['positions'])[[0,-1]])
        trajectory,report=fit_trace(edited,model,config['side'])
        print(json.dumps(dict(slot=slot,original_s=duration,edited_s=float(t[-1]),
            points=len(trajectory.points),fit_error_rad=report['fit_error_rad']),ensure_ascii=False),flush=True)
    assert hashlib.sha256(args.bundle.read_bytes()).digest()==hashlib.sha256(original).digest()
    print('PASS: source bundle unchanged',flush=True)


if __name__=='__main__':main()
