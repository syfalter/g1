"""Application lifecycle simulations. No ROS initialization or live control calls."""
from contextlib import ExitStack
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import time
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock, patch

import task_panel_lifecycle as lifecycle


def identity(pid):
    return dict(pid=pid, start_ticks=pid*10)


class LifecycleTests(unittest.TestCase):
    def gateway(self, phase='TASK_ADMITTANCE', armed=True):
        return dict(phase=phase, armed=str(armed).lower(), command_publisher_exists=str(armed).lower(),
                    max_measured_velocity_rad_s='.01')

    def context(self):
        return dict(launcher_identity=identity(10), gateway_identity=identity(20),
                    console_identity=identity(30), parent_unit='test-application.service',
                    console_command=[], source_session='/old', session_home={'old':True})

    def test_shutdown_accepts_stationary_admittance_without_recovery(self):
        self.assertIsNone(lifecycle.shutdown_issue(self.gateway(), {'worker_busy':False,'phase':'TASK_ADMITTANCE'}))

    def test_busy_running_moving_and_unknown_output_are_not_terminated(self):
        cases = [(self.gateway(), {'worker_busy':True}),
                 (self.gateway(), {'phase':'EXECUTION_PREFLIGHT'}),
                 (self.gateway('RUNNING'), None),
                 ({**self.gateway(), 'max_measured_velocity_rad_s':'.08'}, None),
                 ({**self.gateway(), 'max_measured_velocity_rad_s':'nan'}, None),
                 ({'armed':'false'}, None)]
        for gateway, ui in cases:
            with self.subTest(gateway=gateway,ui=ui):
                self.assertIsNotNone(lifecycle.shutdown_issue(gateway, ui))

    def test_release_waits_for_new_silent_ack_and_only_calls_disarm(self):
        probe=S(gateway=self.gateway(), gateway_at=0., trigger=Mock())
        def wait(predicate, timeout):
            self.assertFalse(predicate())
            probe.gateway=self.gateway('UNARMED',False)
            self.assertFalse(predicate(), 'Old diagnostic cannot acknowledge release')
            probe.gateway_at=time.monotonic()+.001
            self.assertTrue(predicate())
        probe.wait=wait
        lifecycle.Probe.release(probe)
        probe.trigger.assert_called_once_with('disarm')

    def test_latched_silent_needs_no_reset_or_disarm(self):
        probe=S(gateway=self.gateway('LATCHED_SILENT',False), trigger=Mock(), wait=Mock())
        lifecycle.Probe.release(probe)
        probe.trigger.assert_not_called();probe.wait.assert_not_called()

    def close_fixture(self, stack):
        context=self.context()
        probe=S(gateway=self.gateway(), ui={'console_pid':30,'session_dir':'/current','worker_busy':False},
                ui_at=time.monotonic(), wait=Mock(), release=Mock())
        mocks={name:stack.enter_context(patch.object(lifecycle,name)) for name in
               ('preserve_stack','save_context','refresh_session_context','close_process','stop_stack')}
        stack.enter_context(patch.object(lifecycle,'alive',return_value=True))
        return context,probe,mocks

    def test_complete_close_orders_release_gui_then_controller_and_rviz(self):
        with ExitStack() as stack:
            context,probe,mocks=self.close_fixture(stack)
            order=Mock();order.attach_mock(probe.release,'release')
            order.attach_mock(mocks['close_process'],'gui');order.attach_mock(mocks['stop_stack'],'stack')
            lifecycle.close_session(context,probe)
            self.assertEqual([call[0] for call in order.mock_calls],['release','gui','stack'])
            mocks['refresh_session_context'].assert_called_once_with(context,'/current')

    def test_failed_release_leaves_gui_and_stack_alive(self):
        with ExitStack() as stack:
            context,probe,mocks=self.close_fixture(stack)
            probe.release.side_effect=RuntimeError('release acknowledgement timeout')
            with self.assertRaisesRegex(RuntimeError,'timeout'):lifecycle.close_session(context,probe)
            mocks['close_process'].assert_not_called();mocks['stop_stack'].assert_not_called()

    def test_busy_close_sends_no_release(self):
        with ExitStack() as stack:
            context,probe,mocks=self.close_fixture(stack);probe.ui['worker_busy']=True
            with self.assertRaisesRegex(RuntimeError,'active'):lifecycle.close_session(context,probe)
            probe.release.assert_not_called();mocks['close_process'].assert_not_called()

    def test_already_closed_is_idempotent(self):
        with patch.object(lifecycle,'alive',return_value=False),patch.object(lifecycle,'preserve_stack') as preserve:
            probe=Mock();lifecycle.close_session({},probe)
            probe.release.assert_not_called();preserve.assert_not_called()

    def test_main_already_closed_does_not_initialize_ros_on_missing_network(self):
        with TemporaryDirectory() as directory:
            context=Path(directory)/'context.json'
            context.write_text(json.dumps({'environment':{'G1_DDS_INTERFACE_ADDRESS':'192.168.123.99'}}))
            result=Path(directory)/'result.json'
            with patch.object(lifecycle,'CONTEXT',context),patch.object(lifecycle,'alive',return_value=False),\
                    patch.object(lifecycle.rclpy,'init') as init,\
                    patch.object(lifecycle,'check_dds_address') as check,\
                    patch('sys.argv',['lifecycle','close','--result-file',str(result)]):
                lifecycle.main()
            init.assert_not_called();check.assert_not_called()
            self.assertTrue(json.loads(result.read_text())['success'])

    def test_stale_dds_address_reports_current_network_without_creating_ros(self):
        interfaces=[dict(ifname='eth1',addr_info=[dict(family='inet',local='192.168.0.205')])]
        with patch.object(lifecycle.subprocess,'run',return_value=S(stdout=json.dumps(interfaces))) as run:
            with self.assertRaisesRegex(RuntimeError,'192.168.123.99.*eth1=192.168.0.205'):
                lifecycle.check_dds_address({'G1_DDS_INTERFACE_ADDRESS':'192.168.123.99'})
        self.assertEqual(run.call_args.args[0],['ip','-j','address','show'])

    def test_inline_dds_binding_is_checked_even_without_launcher_variable(self):
        uri='<CycloneDDS><Domain><General><Interfaces><NetworkInterface address="192.168.123.99"/></Interfaces></General></Domain></CycloneDDS>'
        with patch.object(lifecycle.subprocess,'run',return_value=S(stdout='[]')):
            with self.assertRaisesRegex(RuntimeError,'192.168.123.99'):
                lifecycle.check_dds_address({'CYCLONEDDS_URI':uri})

    def test_matching_dds_address_and_automatic_selection_are_preserved(self):
        interfaces=[dict(ifname='eth0',addr_info=[dict(family='inet',local='192.168.123.99')])]
        with patch.object(lifecycle.subprocess,'run',return_value=S(stdout=json.dumps(interfaces))) as run:
            lifecycle.check_dds_address({'G1_DDS_INTERFACE_ADDRESS':'192.168.123.99'})
            self.assertEqual(run.call_count,1)
            lifecycle.check_dds_address({'CYCLONEDDS_URI':'<CycloneDDS/>'})
            self.assertEqual(run.call_count,1)

    def test_new_start_retains_ready_and_config_not_home_or_attachment(self):
        with TemporaryDirectory() as directory,patch.object(lifecycle,'ROOT',Path(directory)):
            config=Path(directory)/'planning.yaml'
            context=dict(stack_command=['bash','run_real_task_application.sh','eth0','--hardware',
                '--dring-contact-test','--ready-poses','/old/ready','--planning-config','/old/config'],
                ready_poses={'left':None,'right':None},session_home={'old':True})
            command=lifecycle.stack_command(context,config)
            self.assertNotIn('/old/config',command);self.assertNotIn('/old/ready',command)
            self.assertIn('--managed-lifecycle',command);self.assertIn('--dring-contact-test',command)
            self.assertNotIn('--attach-handoff',command);self.assertNotIn('--resume-session',command)
            self.assertTrue(Path(command[command.index('--ready-poses')+1]).exists())
            self.assertEqual(command[command.index('--planning-config')+1],str(config))

    def test_fresh_machine_uses_the_same_start_script(self):
        command=lifecycle.stack_command({},Path('/config.yaml'))
        self.assertIn(str(lifecycle.ROOT/'scripts/run_real_task_application.sh'),command)
        self.assertIn('--managed-lifecycle',command)

    def test_pid_reuse_is_not_the_old_application(self):
        with patch.object(lifecycle,'process_identity',return_value={'pid':10,'start_ticks':999}):
            self.assertFalse(lifecycle.alive(identity(10)))

    def test_live_application_cannot_be_started_twice(self):
        with patch.object(lifecycle,'alive',return_value=True),patch.object(lifecycle.subprocess,'run') as run:
            with self.assertRaisesRegex(RuntimeError,'already running'):
                lifecycle.start_session(self.context(),Mock(),Path('/config'))
            run.assert_not_called()

    def test_stopping_parent_service_includes_all_children_and_checks_exit(self):
        context=self.context();probe=S(gateway=self.gateway('UNARMED',False),wait=Mock())
        with patch.object(lifecycle,'preserve_stack'),patch.object(lifecycle,'save_context'),\
                patch.object(lifecycle,'descendants',return_value=[20,40,50]),\
                patch.object(lifecycle,'process_identity',side_effect=identity),\
                patch.object(lifecycle,'alive',side_effect=[True,False,False,False,False,False]),\
                patch.object(lifecycle.subprocess,'run') as run:
            lifecycle.stop_stack(context,probe)
        self.assertEqual(run.call_args.args[0],['systemctl','--user','stop','test-application.service'])
        self.assertTrue(context['application_closed'])

    def test_late_rearm_prevents_terminating_parent(self):
        probe=S(gateway=self.gateway('ARMED_HOLD'),wait=Mock())
        with patch.object(lifecycle.subprocess,'run') as run:
            with self.assertRaisesRegex(RuntimeError,'resumed output'):
                lifecycle.stop_stack(self.context(),probe)
            run.assert_not_called()

    def test_start_discovers_new_processes_and_does_not_reuse_old_confirmation(self):
        context=self.context();probe=S(gateway=self.gateway('UNARMED',False),
            ui={'console_pid':130,'session_dir':'/new','worker_busy':False},ui_at=time.monotonic())
        config=Path('/new-config')
        def wait(predicate,timeout):
            self.assertTrue(predicate())
        probe.wait=wait
        commands={120:['/bin/arm_sdk_hardware_gateway'],
                  130:['python3',str(lifecycle.ROOT/'scripts/real_task_app.py'),'--robot-urdf','/new/robot.urdf']}
        with patch.object(lifecycle,'alive',side_effect=lambda item:item==identity(110)),\
                patch.object(lifecycle,'stack_command',return_value=['bash','launcher','--managed-lifecycle']),\
                patch.object(lifecycle,'acquire_console_lock'),patch.object(lifecycle,'unit_pid',return_value=110),\
                patch.object(lifecycle,'process_identity',side_effect=identity),\
                patch.object(lifecycle,'descendants',return_value=[120,130]),\
                patch.object(lifecycle,'argv',side_effect=lambda pid:commands[pid]),\
                patch.object(lifecycle,'environment',return_value={}),\
                patch.object(lifecycle,'refresh_session_context') as refresh,\
                patch.object(lifecycle,'save_context'),patch.object(lifecycle.subprocess,'run') as run:
            lifecycle.start_session(context,probe,config)
        self.assertEqual(context['gateway_identity'],identity(120))
        self.assertEqual(context['console_identity'],identity(130))
        self.assertIsNone(context['session_home'])
        self.assertIsNone(context['console_unit'])
        refresh.assert_called_once_with(context,'/new')
        self.assertEqual(run.call_count,1)
        self.assertEqual(run.call_args.args[0][0],'systemd-run')


if __name__=='__main__':unittest.main()
