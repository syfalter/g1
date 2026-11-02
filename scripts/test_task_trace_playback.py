import copy
import json
import os
from pathlib import Path
import threading
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock

import numpy as np
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint

from task_composition_preview import at,stamp,ROOT
from task_offline_preview import PreviewTimeline
from task_trace_playback import trajectory,message
from task_trace_editor_control import TraceEditorControl


class PlaybackTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path=os.environ.get('G1_OFFLINE_TEST_URDF')
        if not path:raise unittest.SkipTest('Set G1_OFFLINE_TEST_URDF')
        from workbench_model import OfflineModel
        cls.model=OfflineModel(Path(path))
        cls.config=json.loads((ROOT/'data/task_panel/dring_demonstration.json').read_text())
        cls.trace=cls.config['trace_versions']['right_4']['edits'][0]

    def test_exact_timing_survives_seek_and_independent_playback(self):
        before=copy.deepcopy(self.trace)
        source=trajectory(self.trace,self.model,self.config['side'])
        preview=message(source,self.trace,2.125,self.model,self.config['side'],{},self.config,'right_4')
        timeline=PreviewTimeline(source.joint_names);timeline.trajectory(preview,100.)
        # No further GUI messages: the player still follows the exact curve.
        for elapsed in (0.,.1,1.,3.,7.):
            values=timeline.sample(100.+elapsed)
            np.testing.assert_allclose([values[n] for n in source.joint_names],
                                       at(source,2.125+elapsed).positions,atol=1e-9)
        self.assertEqual(self.trace,before)

    def test_one_shot_publish_and_explicit_cancel(self):
        control=TraceEditorControl();control.config=self.config
        control.idle=Mock()
        control.node=S(args=S(offline_editor=True,disconnected=True),lock=threading.RLock(),
            model=self.model,session=S(current=None),create_subscription=Mock(),
            display_pub=Mock(),demo_preview_pub=Mock())
        self.assertTrue(control.editor_playback(self.trace,'right_4',0.))
        control.node.display_pub.publish.assert_called_once()
        control.editor_stop_playback()
        self.assertEqual(control.node.display_pub.publish.call_count,2)
        self.assertEqual(control.node.display_pub.publish.call_args.args[0].trajectory,[])
        control.editor_stop_playback()
        self.assertEqual(control.node.display_pub.publish.call_count,2)
        control.node.args.offline_editor=False
        self.assertFalse(control.editor_playback(self.trace,'right_4',0.))

    def test_raw_recording_stays_linear(self):
        trace=copy.deepcopy(self.trace);trace.pop('optimization')
        source=trajectory(trace,self.model,self.config['side'])
        self.assertFalse(source.points[1].velocities)
        t=(trace['raw']['time_s'][10]+trace['raw']['time_s'][11])/2
        np.testing.assert_allclose(at(source,t).positions,
            (np.asarray(trace['raw']['positions'][10])+trace['raw']['positions'][11])/2,atol=1e-12)


if __name__=='__main__':unittest.main()
