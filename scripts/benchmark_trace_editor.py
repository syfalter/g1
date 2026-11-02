"""Measure selection and cached visualization using a temporary bundle copy."""
import argparse
import shutil
import tempfile
import threading
import time
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock,patch

from rclpy.serialization import serialize_message

from task_dring_demo_control import DringDemoControl
from task_trace_display_cache import TraceDisplayCache
from task_trace_markers import trace_markers,pick_points
from task_trace_picker import TracePicker
from task_trace_selection import journal
from task_trace_versions import bank
from workbench_model import OfflineModel


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('robot',type=Path);parser.add_argument('bundle',type=Path)
    parser.add_argument('--slot',default='right_4');parser.add_argument('--points',type=int,default=300)
    args=parser.parse_args()
    def measure(label,work):
        started=time.perf_counter();result=work()
        print(f'{label}: {(time.perf_counter()-started)*1000:.3f} ms',flush=True)
        return result
    with tempfile.TemporaryDirectory(prefix='g1-editor-benchmark-') as directory:
        store=Path(directory)/'demo.json';shutil.copyfile(args.bundle,store)
        n=S(model=OfflineModel(args.robot),args=S(robot_urdf=args.robot),
            lock=threading.RLock(),session=S(phase='IDLE',generation=0,invalidate=Mock()),
            display_pub=Mock(),worker=None,shutdown_requested=False,run_dir=Path(directory)/'session')
        n.launch_work=lambda work:work()
        c=DringDemoControl(n,store);trace=c.config['traces'][args.slot]
        original_stat=store.stat()
        entry=bank(c.config,args.slot)
        alternate=next(i for i,v in enumerate([entry['baseline']]+entry['edits']) if v is not None and i!=entry['selected'])
        original=entry['selected']
        for index in (alternate,original,alternate,original):
            measure(f'select_{index}',lambda index=index:c.edit_version(args.slot,'select',index=index))
        assert store.stat()==original_stat,'Selection unexpectedly rewrote the bundle'
        print('bundle_bytes:',store.stat().st_size,'selection_bytes:',journal(store).stat().st_size)
        restored=DringDemoControl(n,store)
        assert bank(restored.config,args.slot)['selected']==original
        cache=TraceDisplayCache();side=c.editor_side();context=dict(trace.get('context_joints',{}))
        def render():
            geometry=cache.geometry(n.model,trace,side,context,c.config,args.slot)
            markers=trace_markers(n.model,trace,side,context,0.,0.,c.config,args.slot,
                spacing=.001,maximum=args.points,geometry=geometry)
            points=pick_points(n.model,trace,side,context,c.config,args.slot,.001,args.points,geometry=geometry)
            return serialize_message(markers),None,points
        frame=measure('cold_frame_FK_build_serialize',render)
        cache.remember_frame('frame',frame)
        measure('cached_frame_lookup',lambda:cache.frame('frame'))
        measure('cached_geometry_lookup',lambda:cache.geometry(n.model,trace,side,context,c.config,args.slot))
        with patch('task_trace_picker.InteractiveMarkerServer') as server:
            picker=TracePicker(Mock());owner=object()
            measure('cold_picker_mock_server',lambda:picker.show(owner,frame[2],lambda _:None))
            server.return_value.insert.reset_mock();server.return_value.clear.reset_mock()
            measure('unchanged_picker_mock_server',lambda:picker.show(owner,frame[2],lambda _:None))
            server.return_value.insert.assert_not_called();server.return_value.clear.assert_not_called()
        print('PASS: no source changes, durable selection, unchanged points not rebuilt; DDS/RViz render time excluded')


if __name__=='__main__':main()
