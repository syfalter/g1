"""Center-point geometry and configuration tests, without ROS initialization."""
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
import numpy as np
from scipy.spatial.transform import Rotation

from task_center_check import target_geometry,measurement
from task_dring_sequence import CENTER_SCHEMA,CENTER_LABELS,plan_labels
from task_planning_config import load_parameters,parameters_from_values


class CenterCheckTests(unittest.TestCase):
    def geometry(self):
        frozen=np.eye(4);frozen[:3,:3]=Rotation.from_euler('z',40,degrees=True).as_matrix()
        frozen[:3,3]=[.01,.02,.8]
        return dict(task_preview=dict(axis_center_m=[.35,.1,-.1],insertion_reference_m=[.42,.1,-.075]),
            capture_frame=dict(frame='pelvis',pelvis_from_torso_row_major=frozen.tolist()),
            transforms=dict(torso_from_color_optical_row_major=np.eye(4).tolist()))

    def test_center_plus_world_z_does_not_use_ring_compensation(self):
        g=self.geometry();d=target_geometry(g,None,{},50.)
        center=np.asarray(g['capture_frame']['pelvis_from_torso_row_major'])@np.r_[g['task_preview']['axis_center_m'],1.]
        np.testing.assert_allclose(d['target_pelvis_m'],center[:3]+[0,0,.05])
        self.assertFalse(d['operation_offsets_applied'])
        g['capture_frame']['pelvis_from_torso_row_major']=None
        with self.assertRaises(ValueError):
            target_geometry(g,SimpleNamespace(transform=lambda *args:np.eye(4)),{},50.)

    def test_measurement_does_not_claim_physical_calibration(self):
        g=self.geometry();d=target_geometry(g,None,{},50.);d['wrist_from_index_tcp']=np.eye(4).tolist()
        tcp=np.eye(4);tcp[:3,3]=np.asarray(d['target_pelvis_m'])+[.01,-.02,.003]
        model=SimpleNamespace(transform=lambda *args:tcp.copy())
        result=measurement(model,'left',{},d)
        np.testing.assert_allclose(result['fk_error_mm'],[10,-20,3])
        self.assertFalse(result['physical_tip_observed']);self.assertFalse(result['calibration_identified'])

    def test_center_schema_is_not_a_seven_stage_ring(self):
        self.assertEqual(plan_labels({'task_schema':CENTER_SCHEMA}),CENTER_LABELS)
        self.assertEqual(len(CENTER_LABELS),3)
        with self.assertRaises(ValueError):plan_labels({'task_schema':'old_ten_stage'})

    def test_missing_live_switches_enable_continuous_timing_but_snapshots_do_not(self):
        with TemporaryDirectory() as temp:
            path=Path(temp)/'config.yaml';path.write_text('geometry:\n  height_offset_mm: 25\n')
            p,record=load_parameters(path)
            self.assertTrue(p.smooth_joint_path and p.smooth_insert_path and p.smooth_contact_path)
            self.assertEqual(len(record['live_defaults_applied']),3)
            self.assertEqual(p.height_offset_mm,25)
            path.write_text('smoothing:\n  smooth_insert_path: false\n')
            self.assertFalse(load_parameters(path)[0].smooth_insert_path)
        self.assertFalse(parameters_from_values({}).smooth_insert_path)

    def test_height_bounds(self):
        for value in (0,39,151,float('nan')):
            with self.assertRaises(ValueError):parameters_from_values({'center_check_height_mm':value})
