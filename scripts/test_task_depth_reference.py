import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch
import numpy as np
from sensor_msgs.msg import PointCloud2
from task_depth_reference_panel import PACKED, reference_message


class ReferenceTests(unittest.TestCase):
    def test_preview_cloud_is_bounded_and_preserves_source(self):
        from task_depth_reference_panel import preview_cloud
        points=np.zeros(10000,dtype=PACKED);points['x']=np.arange(10000)*.001
        original=points.copy()
        result=preview_cloud(points,maximum=100)
        self.assertLessEqual(len(result),100)
        np.testing.assert_array_equal(points,original)
    def test_preferences_restore_before_async_scan(self):
        from PyQt5 import QtWidgets
        from builtin_interfaces.msg import Time
        from task_depth_reference_panel import DepthReferencePanel
        app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        node=SimpleNamespace(create_publisher=Mock(),get_clock=Mock(),run_dir=Path('/unused/session'))
        node.get_clock.return_value.now.return_value.to_msg.return_value=Time()
        with patch.object(DepthReferencePanel,'scan'):
            panel=DepthReferencePanel(node)
            settings=dict(visible=False,source='/saved/plan_0001',offsets=[.1,-.2,.3])
            panel.restore_settings(settings)
            self.assertEqual(panel.settings(),settings)
            app.processEvents();panel.shutdown();panel.close()

    def test_translation_does_not_change_source(self):
        packed=np.zeros(2,dtype=PACKED);packed['rgb']=[0x123456,0xffffff]
        source=PointCloud2(height=1,width=2,point_step=16,row_step=32,data=packed.tobytes())
        result=reference_message(source,(1.,2.,3.))
        actual=np.frombuffer(bytes(result.data),dtype=PACKED)
        for axis,value in zip('xyz',(1.,2.,3.)):np.testing.assert_allclose(actual[axis],value)
        np.testing.assert_array_equal(actual['rgb'],packed['rgb'])
        self.assertEqual(bytes(source.data),packed.tobytes())

    def test_hiding_clears_latched_cloud(self):
        source=PointCloud2(height=1,width=2,point_step=16,row_step=32,data=bytes(32))
        hidden=reference_message(source,visible=False)
        self.assertEqual(hidden.header.frame_id,'pelvis')
        self.assertEqual(hidden.width,0)
        self.assertEqual(len(hidden.data),0)
        self.assertEqual(reference_message(None).width,0)


if __name__=='__main__':unittest.main()
