"""Qt retiming controls with fake publishers; no ROS initialization or robot I/O."""
import copy
import json
import os
from pathlib import Path
import tempfile
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock,patch

os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
from PyQt5 import QtWidgets
from task_offline_gallery import Gallery
from task_speed_control import SpeedInput


class GallerySpeedTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.capture=QtWidgets.QComboBox();self.capture.addItem('archive')
        self.speed=QtWidgets.QComboBox();self.speed.addItems(['0.5','1','2','4'])
        self.input=SpeedInput();self.label=QtWidgets.QLabel()
        self.widget=QtWidgets.QWidget()
        points=[dict(positions=[q],velocities=[0.],accelerations=[0.],effort=[],
                     time_from_start=dict(sec=t,nanosec=0)) for t,q in ((0,0.),(120,.2))]
        self.original=dict(segments=[dict(kind='arm',label='Rotate',
            trajectory=dict(joint_names=['left_wrist_yaw_joint'],points=points))],hardware_executable=False)
        self.gallery=S(capture=self.capture,trajectory_speed=self.input,
            source_reports=[(Path('/immutable/report.json'),copy.deepcopy(self.original))],
            reports=[(Path('/immutable/report.json'),copy.deepcopy(self.original))],
            output=Path(self.tmp.name),speed=self.speed,select=Mock(),play=Mock(),
            phase=self.label,style=self.widget.style)

    def test_custom_speed_is_applied_and_editing_does_not_compound(self):
        for speed in (24.,73.125,2.5):
            self.input.setValue(speed)
            Gallery.retime(self.gallery)
            record=self.gallery.reports[0][1]['speed_scaling']
            self.assertEqual(record['applied_multiplier'],speed)
            self.assertAlmostEqual(record['duration_after_s'],120./speed,places=7)
            self.assertFalse(record['hardware_executable'])
        self.assertEqual(self.gallery.source_reports[0][1],self.original)
        self.assertTrue((self.gallery.output/'retimed_preview_000.json').is_file())

    def test_invalid_text_preserves_current_trajectory(self):
        before=copy.deepcopy(self.gallery.reports)
        self.input.setText('nan')
        Gallery.retime(self.gallery)
        self.assertEqual(self.gallery.reports,before)
        self.assertIn('positive finite',self.label.text())

    def test_guarded_gallery_edits_keep_motion_limit_policy(self):
        original=self.gallery.source_reports[0][1]
        original.update(timing_policy='guarded',motion_parameters={},
                        velocity_limit_scaling=.3,acceleration_limit_scaling=.5)
        self.input.setValue(24.)
        with patch('workbench_model.OfflineModel') as model, \
                patch('task_trajectory_speed.scale_arm_segments') as scale, \
                patch('task_trajectory_speed.preview_segments') as unrestricted:
            scale.side_effect=lambda segments,*args,**kwargs:(segments,dict(applied_multiplier=2.,requested_multiplier=24.))
            Gallery.retime(self.gallery)
            unrestricted.assert_not_called()
            self.assertEqual(scale.call_args.args[1],24.)
            self.assertEqual(scale.call_args.kwargs['velocity'],.3)
            self.assertEqual(scale.call_args.kwargs['acceleration'],.5)
        self.assertEqual(self.gallery.reports[0][1]['timing_policy'],'guarded')

    def test_retime_uses_original_base_not_initial_preview_multiplier(self):
        original=self.gallery.source_reports[0][1]
        original['segments'][0]['trajectory']['points'][1]['time_from_start']['sec']=5
        original['speed_scaling']=dict(applied_multiplier=24.,hand_timing_preserved=False)
        self.input.setValue(4.)
        Gallery.retime(self.gallery)
        self.assertAlmostEqual(self.gallery.reports[0][1]['speed_scaling']['duration_after_s'],30.)


if __name__=='__main__':unittest.main()
