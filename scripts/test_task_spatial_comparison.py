"""Standard offline-editor depth comparison and line-only display regression."""
import unittest
from types import SimpleNamespace as S
from unittest.mock import Mock,patch

import numpy as np
from PyQt5 import QtWidgets,QtCore

from trajectory_only_candidates import ROOT,BASE,read,load,OfflineModel,decode,checked,parameters_from_values
from task_optimizer_snapshot import DIRECTORY
from task_spatial_comparison import sample_path,path_markers


class SpatialTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        cls.model=OfflineModel(ROOT/'data/trajectory_edit_audit/fast_timing_all/panel_robot.urdf')
        cls.directory=ROOT/'data/rail_deployment/push_depth_exit_v2_20260929'
        cls.report=read(cls.directory/'report.json')
        cls.snapshot=load(ROOT/cls.report['comparison_snapshot'],cls.model)

    def test_all_depth_candidates_meet_unchanged_limits(self):
        action=read(BASE/'bundle.json')['payload']['actions']['push']
        parameters=parameters_from_values(action['plan']['planning_config']['values'])
        for key,result in self.snapshot['variants'].items():
            tr=decode(result['payload']['units']['motion_0']['trajectory'])
            peaks=checked(tr,self.model,parameters)
            self.assertLessEqual(peaks['jerk'],5.000005)
            self.assertFalse(result['payload']['trajectoryReview']['deployed'])
        self.assertEqual(self.report['candidates']['4']['trials'][-1]['exit_transition_s'],1.75)

    def test_dense_geometry_and_line_only_markers(self):
        curves={}
        for key,result in self.snapshot['variants'].items():
            curve=sample_path(result,self.model)
            self.assertLessEqual(curve['max_chord_mm'],.050001)
            self.assertLessEqual(curve['midpoint_error_mm'],.005001)
            curve['color']='#13865d';curves[key]=curve
        peaks=[float(curves[k]['xyz_mm'][:,0].max()) for k in ('original','baseline','candidate')]
        np.testing.assert_allclose(np.array(peaks)-peaks[0],[0.,2.954868,3.939825],atol=.005)
        from visualization_msgs.msg import Marker
        markers=path_markers(curves,set(curves))
        drawn=[m for m in markers.markers if m.action==Marker.ADD]
        self.assertEqual(len(drawn),3)
        self.assertTrue(all(m.type==Marker.LINE_STRIP for m in drawn))
        self.assertTrue(all(m.scale.x<=.0002 for m in drawn))
        self.assertEqual({m.header.frame_id for m in drawn},{'pelvis'})

    def test_standard_panel_restores_depth_group_and_millimeter_overlay(self):
        from task_offline_optimizer_panel import OptimizerPanel,data_index
        from task_trace_selection import restore
        rail_path=ROOT/'data/task_sessions/20260924_153036_985614/rail_autosave/config.json'
        gear_path=ROOT/'data/task_panel/landing_gear_demonstration.json'
        node=S(model=self.model,rail=S(config=restore(read(rail_path),rail_path),revision=0),
               landing_gear=S(config=restore(read(gear_path),gear_path),revision=0),
               demo=S(config={'traces':{}},revision=0),display_pub=Mock(),demo_preview_pub=Mock(),
               marker_pub=Mock(),status='',launch_work=Mock())
        panel=OptimizerPanel(node)
        scroll=QtWidgets.QScrollArea();scroll.setWidgetResizable(True);scroll.setWidget(panel)
        scroll.resize(1000,1050);scroll.show()
        try:
            with patch('task_offline_optimizer_panel.comparison',side_effect=AssertionError('Unexpected regeneration')):
                self.assertTrue(panel.restore_startup(),panel.viewer.summary.text())
                for _ in range(5):self.app.processEvents()
                self.assertEqual(panel.case.currentData(),('rail','push_cycle'))
                self.assertEqual(panel.comparison_set.currentText(),'推进补深 0 / 3 / 4 mm')
                panel.refresh(False);self.assertFalse(panel.generate.isEnabled())
                spatial=panel.viewer.path_panel
                self.assertEqual(len(spatial.curves),3)
                self.assertLessEqual(spatial.plot.span,24.001)
                self.assertGreaterEqual(3*spatial.plot.pixels_per_mm(),30.)
                self.assertIn('+2.955',spatial.readout.text());self.assertIn('+3.940',spatial.readout.text())
                self.assertLess(spatial.plot.geometry().bottom(),spatial.readout.geometry().top())
                self.assertLessEqual(spatial.readout.geometry().bottom(),spatial.height())
                self.assertLess(panel.viewer.view_tabs.geometry().bottom(),panel.viewer.slider.geometry().top())
                self.assertTrue(scroll.grab().save(str(self.directory/'standard_panel_depth.png')))
                self.assertTrue(spatial.grab().save(str(self.directory/'depth_xz.png')))
                spatial.plane.setCurrentIndex(1);self.app.processEvents()
                self.assertTrue(spatial.grab().save(str(self.directory/'depth_xy.png')))
                spatial.plot.span/=2;spatial.plot.update();self.app.processEvents()
                self.assertGreaterEqual(3*spatial.plot.pixels_per_mm(),60.)
                panel.viewer.variant.setCurrentIndex(panel.viewer.variant.findData('baseline'))
                self.assertEqual(panel.viewer.result['payload']['spatialComparison']['amplitude_mm'],3)
                panel.viewer.toggle();self.assertTrue(panel.viewer.playing);panel.viewer.toggle()
                scroll.resize(650,800)
                for _ in range(3):self.app.processEvents()
                self.assertLess(spatial.plot.geometry().bottom(),spatial.readout.geometry().top())
                self.assertGreaterEqual(spatial.readout.height(),spatial.readout.heightForWidth(spatial.readout.width()))
                self.assertLessEqual(spatial.readout.geometry().bottom(),spatial.height())
                self.assertTrue(scroll.grab().save(str(self.directory/'narrow_panel_depth.png')))
                panel.case.setCurrentIndex(data_index(panel.case,('rail','pull_cycle')));self.app.processEvents()
                self.assertIsNone(panel.comparison_set.currentData())
                node.launch_work.assert_not_called()
        finally:
            panel.halt();panel.viewer.timer.stop();scroll.close()


if __name__=='__main__':unittest.main()
