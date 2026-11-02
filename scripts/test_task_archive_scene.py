import json
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace

import numpy as np
from task_archive_scene import snapshot_overlay
from task_plan_archive import digest


class SceneTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name);self.capture=self.root/'capture';self.capture.mkdir()
        self.frame=np.eye(4);self.frame[0,3]=.12
        self.model=SimpleNamespace(transform=lambda link,q:self.frame)
        self.geometry=dict(capture_frame=dict(pelvis_from_torso_row_major=self.frame.tolist()),
                           source=dict(sequence=1,request_id='fixture'),
                           transforms=dict(torso_from_color_optical_row_major=np.eye(4).tolist()),
                           task_preview=dict(axis_center_m=[0.,0.,0.],insertion_reference_m=[0.,0.,.1]))
        for name in ('rgb.png','depth_aligned.png','intrinsics.json'):
            (self.capture/name).write_text('fixture')
        np.savez(self.capture/'cloud.npz',xyz=np.array([[0.,0.,1.]]),rgb=np.zeros((1,3),dtype=np.uint8))
        (self.capture/'manifest.json').write_text(json.dumps(dict(source_sequence=1,source_request_id='fixture',
            frame_id='torso_link',torso_from_color_optical_row_major=np.eye(4).tolist(),
            sha256={'cloud.npz':digest(self.capture/'cloud.npz')})))
        (self.capture/'capture_state.json').write_text(json.dumps(dict(joints={})))
        self.save()

    def save(self):
        (self.capture/'geometry.json').write_text(json.dumps(self.geometry))

    def snapshot(self):
        return snapshot_overlay(self.capture,self.root/'snapshot',dict(start={}),self.geometry,self.model)

    def test_snapshot_is_independent_and_never_execution_validation(self):
        report=self.snapshot()
        self.assertTrue(report['target_within_3mm'])
        self.assertFalse(report['current_scene_collision_validated'])
        self.assertFalse(report['hardware_executable'])
        (self.capture/'rgb.png').write_text('changed')
        self.assertEqual((self.root/'snapshot/rgb.png').read_text(),'fixture')

    def test_missing_evidence_and_transform_mismatch_rejected(self):
        self.geometry['capture_frame']['pelvis_from_torso_row_major']=np.eye(4).tolist();self.save()
        with self.assertRaises(ValueError):self.snapshot()
        (self.capture/'depth_aligned.png').unlink()
        with self.assertRaises(FileNotFoundError):self.snapshot()

    def test_calibration_mismatch_rejected(self):
        old=dict(self.geometry,calibration={'changed':True})
        with self.assertRaises(ValueError):
            snapshot_overlay(self.capture,self.root/'snapshot',dict(start={}),old,self.model)

    def test_changed_target_recorded_without_moving_path(self):
        old=dict(self.geometry,task_preview=dict(axis_center_m=[.01,0.,0.],insertion_reference_m=[0.,0.,.1]))
        report=snapshot_overlay(self.capture,self.root/'snapshot',dict(start={}),old,self.model)
        self.assertFalse(report['target_within_3mm'])
        self.assertFalse(report['path_transformed'])
        self.assertAlmostEqual(report['target_displacement_m']['axis_center_m'],.01)


if __name__=='__main__':unittest.main()
