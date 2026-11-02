"""Compiled controller inspection only: all invoked modes exit before DDS init."""
import json
from pathlib import Path
import subprocess
import unittest
import xml.etree.ElementTree as ET

import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[1]
SDK=Path('/home/syf/unitree_sdk2/build/bin')
URDF=Path('/home/syf/unitree_ws/unitree_ros/robots/g1_description/g1_29dof_lock_waist_rev_1_0.urdf')
FIT={'left':ROOT/'calibration/left_arm_right_copy_seed_20260908.yaml',
     'right':ROOT/'calibration/right_arm_camera_fit_20260824_185416.yaml'}


def run(side,*args):
    return subprocess.run([str(SDK/f'g1_{side}_arm_payload_calibration'),*map(str,args)],
                          capture_output=True,text=True,timeout=20)


class LeftAdaptationTests(unittest.TestCase):
    def test_copied_gains_match_and_indices_are_disjoint(self):
        profiles={side:json.loads(run(side,'--describe').stdout) for side in ('left','right')}
        for field in ('admittance_kp','admittance_kd','max_velocity','max_acceleration',
                      'deadband','torque_limits','feedforward_limits'):
            self.assertEqual(profiles['left'][field],profiles['right'][field],field)
        self.assertEqual(profiles['left']['indices'],list(range(15,22)))
        self.assertEqual(profiles['right']['indices'],list(range(22,29)))
        parts=('shoulder_pitch','shoulder_roll','shoulder_yaw','elbow','wrist_roll','wrist_pitch','wrist_yaw')
        robot=ET.parse(URDF).getroot()
        for side,profile in profiles.items():
            self.assertFalse(profile['creates_dds'])
            self.assertEqual(profile['payload_frame'],side+'_wrist_yaw_link')
            for i,part in enumerate(parts):
                limit=robot.find(f"joint[@name='{side}_{part}_joint']/limit")
                self.assertAlmostEqual(profile['lower'][i],float(limit.get('lower')),places=7)
                self.assertAlmostEqual(profile['upper'][i],float(limit.get('upper')),places=7)

    def test_seed_is_numerical_copy_not_claimed_calibration(self):
        left=yaml.safe_load(FIT['left'].read_text());right=yaml.safe_load(FIT['right'].read_text())
        for field in ('mass_kg','com_xyz_m','first_moment_kg_m'):
            self.assertEqual(left[field],right[field])
        self.assertEqual(left['calibration_status'],'copied_right_unvalidated')
        self.assertEqual(left['pose_count'],0)
        self.assertNotIn('training_rmse_nm',left)

    def test_gravity_uses_correct_chain_and_rejects_wrong_payload_frame(self):
        for side,sign in (('left',1),('right',-1)):
            q=[.25,sign*.27,-sign*.08,.8,0.,0.,0.]
            result=run(side,'--check-pose',FIT[side],*q)
            self.assertEqual(result.returncode,0,result.stderr)
            report=json.loads(result.stdout)
            self.assertEqual(report['arm'],side)
            self.assertTrue(np.isfinite(report['zero_rpy_gravity_nm']).all())
            self.assertFalse(report['creates_dds'])
            other='right' if side=='left' else 'left'
            rejected=run(side,'--check-pose',FIT[other],*q)
            self.assertNotEqual(rejected.returncode,0)
            self.assertIn('frame does not match',rejected.stderr)

    def test_left_check_rejects_invalid_numbers_and_right_roll_limit(self):
        for roll in ('nan','inf','0.2garbage','-1.9'):
            result=run('left','--check-pose',FIT['left'],0,roll,0,.8,0,0,0)
            self.assertNotEqual(result.returncode,0)

    def test_left_blocks_scripted_motion_modes_before_dds(self):
        result=run('left','unused_interface','--execute','I_UNDERSTAND_12_POSE_CALIBRATION',
                   '/does/not/exist','/does/not/exist','/does/not/exist')
        self.assertNotEqual(result.returncode,0)
        self.assertIn('allows only --remote-admittance',result.stderr)

    def test_endpoint_gains_and_limits_are_shared(self):
        config=yaml.safe_load((ROOT/'src/g1_unitree_control/config/arm_sdk_hardware_gateway.yaml').read_text())
        p=config['g1_arm_sdk_hardware_gateway']['ros__parameters']
        self.assertEqual(p['active_arm'],'right')
        self.assertFalse(p['hardware_output_enabled'])
        self.assertEqual(p['left_wrist_roll_kp'],p['right_wrist_roll_kp'])
        self.assertEqual(p['left_wrist_roll_kd'],p['right_wrist_roll_kd'])
        self.assertEqual(p['max_velocity_rad_s'],.60)
        self.assertEqual(p['max_acceleration_rad_s2'],1.50)


if __name__=='__main__':unittest.main()
