import copy
import os
import unittest
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch

from task_console_handoff import process_identity, supported_gateway, supported_controller, validate_handoff
from task_session import ARM_NAMES


class HandoffTests(unittest.TestCase):
    def setUp(self):
        self.record={'created_at_epoch_s':100.,'gateway_identity':{'pid':123,'start_ticks':456},
                     'stop_epoch':1,'session_home':{'operator_confirmed_r2_a':True,
                     'joints':dict.fromkeys(ARM_NAMES,.2)},'ready_poses':{'left':None,'right':None}}
        self.gateway={'armed':'true','hardware_output_available':'true','task_application_enabled':'true',
                      'upper_body_state_safe':'true','service_state_ready':'true','mode_machine':'6','mode_pr':'0',
                      'max_measured_velocity_rad_s':'0.01'}
        for key in ('torque_overload_active','command_overload_active','velocity_recovery_active',
                    'temperature_alarm_active','feedforward_saturated','service_conflict'):
            self.gateway[key]='false'

    def validate(self, record=None, now=101):
        with patch('task_console_handoff.process_identity',return_value=self.record['gateway_identity']):
            return validate_handoff(self.record if record is None else record,now)

    def test_own_process_identity_is_stable(self):
        self.assertEqual(process_identity(os.getpid()),process_identity(os.getpid()))

    def test_preserves_original_home(self):
        self.assertEqual(self.validate()['session_home'],self.record['session_home'])

    def test_pending_capture_is_explicit_and_does_not_invent_home(self):
        record=copy.deepcopy(self.record);record['session_home']=None
        with self.assertRaisesRegex(RuntimeError,'confirmed'):self.validate(record)
        record['startup_capture_pending']=True
        self.assertIsNone(self.validate(record)['session_home'])

    def test_hold_reference_requires_explicit_confirmation_and_mode(self):
        record=copy.deepcopy(self.record);home=record['session_home']
        home.update(operator_confirmed_r2_a=False,operator_confirmed_hold_reference=True)
        with self.assertRaisesRegex(RuntimeError,'confirmed'):self.validate(record)
        home['capture_mode']='ARMED_HOLD'
        self.assertEqual(self.validate(record)['session_home'],home)

    def test_preserves_optional_measured_waist(self):
        record=copy.deepcopy(self.record);record['session_home']['joints']['waist_yaw_joint']=.1
        self.assertEqual(self.validate(record)['session_home']['joints']['waist_yaw_joint'],.1)
        record['session_home']['joints']['waist_yaw_joint']=float('nan')
        with self.assertRaisesRegex(RuntimeError,'joints'):self.validate(record)

    def test_unrelated_home_joint_is_rejected(self):
        record=copy.deepcopy(self.record);record['session_home']['joints']['waist_roll_joint']=0.
        with self.assertRaisesRegex(RuntimeError,'joints'):self.validate(record)

    def test_reload_preserves_hand_waist_profile_and_replaces_old_manifest(self):
        from migrate_legacy_task_console import replacement_command
        command=['/usr/bin/python3','real_task_app.py','--hardware','--network','eth0',
                 '--hand-control','--waist-assist','--dring-profile','steep_straight']
        expected=command+['--attach-handoff','new.json']
        self.assertEqual(replacement_command(command,'new.json'),expected)
        self.assertEqual(replacement_command(command+['--attach-handoff','old.json'],'new.json'),expected)

    def test_ready_handoff_preserves_configured_pose_and_prefers_session(self):
        from migrate_legacy_task_console import ready_poses_for_handoff
        with TemporaryDirectory() as directory:
            root=Path(directory);configured=root/'configured.json'
            configured.write_text(json.dumps({'left': 'configured'}))
            command=['--ready-poses',str(configured)]
            self.assertEqual(ready_poses_for_handoff(root,command),{'left':'configured'})
            (root/'ready_poses.json').write_text(json.dumps({'left':'session'}))
            self.assertEqual(ready_poses_for_handoff(root,command),{'left':'session'})

    def test_ready_handoff_does_not_invent_missing_pose(self):
        from migrate_legacy_task_console import ready_poses_for_handoff
        with TemporaryDirectory() as directory:
            with self.assertRaisesRegex(RuntimeError,'unavailable'):
                ready_poses_for_handoff(Path(directory),[])

    def test_managed_console_unit_identity_and_parent_must_match(self):
        from types import SimpleNamespace as S
        from migrate_legacy_task_console import verify_console_unit
        with patch('migrate_legacy_task_console.subprocess.run',return_value=S(stdout='MainPID=123\nBindsTo=live.service\n')):
            verify_console_unit('console.service',123,'live.service')
            with self.assertRaisesRegex(RuntimeError,'identity'):verify_console_unit('console.service',124,'live.service')
            with self.assertRaisesRegex(RuntimeError,'identity'):verify_console_unit('console.service',123,'other.service')

    def test_reload_preselects_archive_without_execute_flag(self):
        from migrate_legacy_task_console import replacement_command
        command=['python3','real_task_app.py','--hardware','--saved-test-archive','old']
        result=replacement_command(command,'new.json','frozen')
        self.assertEqual(result.count('--saved-test-archive'),1)
        self.assertEqual(result[result.index('--saved-test-archive')+1],'frozen')
        self.assertNotIn('--execute',result)

    def test_live_contact_test_mode_requires_explicit_migration_option(self):
        from migrate_legacy_task_console import replacement_command
        command=['python3','real_task_app.py','--hardware']
        self.assertNotIn('--dring-contact-test',replacement_command(command,'manifest'))
        enabled=replacement_command(command,'manifest',dring_contact_test=True)
        self.assertEqual(enabled.count('--dring-contact-test'),1)
        self.assertEqual(replacement_command(enabled,'next',dring_contact_test=True).count('--dring-contact-test'),1)

    def test_no_expired_or_future_handoff(self):
        for now in (99,220,1000):
            with self.assertRaisesRegex(RuntimeError,'expired'):self.validate(now=now)

    def test_restarted_gateway_rejected(self):
        with patch('task_console_handoff.process_identity',return_value={'pid':123,'start_ticks':457}):
            with self.assertRaisesRegex(RuntimeError,'restarted'):validate_handoff(self.record,101)

    def test_unconfirmed_home_rejected(self):
        record=copy.deepcopy(self.record);record['session_home']['operator_confirmed_r2_a']=False
        with self.assertRaisesRegex(RuntimeError,'confirmed'):self.validate(record)

    def test_missing_or_nonfinite_joint_rejected(self):
        record=copy.deepcopy(self.record);record['session_home']['joints'].pop(ARM_NAMES[0])
        with self.assertRaisesRegex(RuntimeError,'joints'):self.validate(record)
        record=copy.deepcopy(self.record);record['session_home']['joints'][ARM_NAMES[0]]=float('nan')
        with self.assertRaisesRegex(RuntimeError,'joints'):self.validate(record)

    def test_incomplete_preparation_rejected(self):
        record=copy.deepcopy(self.record);record['ready_poses']['left']={'joints':{}}
        with self.assertRaisesRegex(RuntimeError,'Incomplete'):self.validate(record)

    def test_stop_epoch_must_be_integer(self):
        for epoch in (-1,1.5,True):
            record=copy.deepcopy(self.record);record['stop_epoch']=epoch
            with self.assertRaisesRegex(RuntimeError,'epoch'):self.validate(record)

    def test_stopped_attachment_mode_is_explicit_and_validated(self):
        record=copy.deepcopy(self.record);record['attachment_mode']='stopped'
        self.assertEqual(self.validate(record)['attachment_mode'],'stopped')
        record['attachment_mode']='resume'
        with self.assertRaisesRegex(RuntimeError,'attachment mode'):self.validate(record)

    def test_supported_gateway(self):
        supported_gateway(self.gateway)

    def test_controller_health_is_distinct_from_stationary_handoff(self):
        self.gateway['max_measured_velocity_rad_s']='0.174844593'
        self.assertEqual(supported_controller(self.gateway),.174844593)
        with self.assertRaisesRegex(RuntimeError,'stationary'):supported_gateway(self.gateway)

    def test_controller_health_still_rejects_faults_and_nonfinite_velocity(self):
        for key,value in (('armed','false'),('service_state_ready','false'),('mode_machine','0'),
                          ('torque_overload_active','true'),('command_overload_active','true'),
                          ('velocity_recovery_active','true'),('temperature_alarm_active','true'),
                          ('feedforward_saturated','true'),('service_conflict','true'),
                          ('max_measured_velocity_rad_s','nan')):
            with self.subTest(key=key),self.assertRaises(RuntimeError):
                supported_controller(dict(self.gateway,**{key:value}))

    def test_unsafe_gateway_rejected(self):
        for key,value in (('armed','false'),('service_state_ready','false'),('mode_machine','0'),
                          ('torque_overload_active','true'),('command_overload_active','true'),
                          ('temperature_alarm_active','true'),('feedforward_saturated','true'),
                          ('max_measured_velocity_rad_s','0.1'),('max_measured_velocity_rad_s','nan')):
            gateway=dict(self.gateway);gateway[key]=value
            with self.assertRaises(RuntimeError):supported_gateway(gateway)


if __name__=='__main__':unittest.main()
