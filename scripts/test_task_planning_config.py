from dataclasses import FrozenInstanceError,replace
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock,patch

import numpy as np
from scipy.spatial.transform import Rotation
from task_planning_config import DEFAULT_CONFIG,PlanningParameters,configure_planner,load_parameters,parameters_from_values
from task_motion_planner import PlanningFailure,TaskPlanner
from task_straight_policy import StraightIndexPolicy


class PlanningConfigTests(unittest.TestCase):
    def test_rail_panel_speed_snapshot_preserves_config_and_controller_guards(self):
        from task_config_monitor import PlanningConfigMonitor
        from task_planning_config import controller_bounded_parameters,rail_panel_parameters
        with TemporaryDirectory() as directory:
            path=Path(directory)/'planning.yaml'
            path.write_text('rail_replay: {rail_replay_multiplier: 2.0}')
            monitor=PlanningConfigMonitor(path)
            gateway=dict(command_arm_velocity_limit_rad_s='.2')
            parameters,source=controller_bounded_parameters(monitor.parameters,monitor.record,gateway)
            effective,record=rail_panel_parameters(parameters,source,dict(rail_replay_multiplier=3.,rail_replay_fastest=True,
                rail_trim_idle_ends=True))
            self.assertEqual(effective.arm_velocity_rad_s,.2)
            self.assertTrue(effective.rail_trim_idle_ends)
            self.assertEqual(source['values']['rail_replay_multiplier'],2.)
            self.assertTrue(monitor.matches(record,gateway))
            self.assertFalse(monitor.matches(dict(record,values=dict(record['values'],arm_velocity_rad_s=.4)),gateway))
            for values in ({'arm_velocity_rad_s':.4},{'rail_replay_multiplier':11.},{'rail_replay_fastest':1},
                           {'rail_trim_idle_ends':1}):
                with self.assertRaises(ValueError):rail_panel_parameters(parameters,source,values)
            path.write_text('rail_replay: {rail_replay_multiplier: 4.0}')
            monitor.reload()
            self.assertFalse(monitor.matches(record,gateway))

    def test_current_right_controller_matches_left_including_bilateral_integral(self):
        from task_planning_config import ARM_INTEGRAL_JOINTS
        from task_stage_control import replay_arm_kp
        params,_=load_parameters()
        for joint in ARM_INTEGRAL_JOINTS:
            for gain in ('kp','kd'):
                self.assertEqual(getattr(params,'right_'+joint+'_'+gain),getattr(params,'left_'+joint+'_'+gain))
        self.assertEqual(replay_arm_kp(params,'right'),replay_arm_kp(params,'left'))
        left=params.controller_parameters('left');right=params.controller_parameters('right')
        for config in (left,right):
            self.assertTrue(config['arm_integral_enabled'])
            self.assertTrue(config['passive_arm_integral_enabled'])
            for gain in ('kp','kd'):self.assertEqual(config['left_arm_'+gain],config['right_arm_'+gain])
        for key in ('arm_integral_ki','arm_integral_limits_nm','arm_integral_hold_rate_nm_s'):
            self.assertEqual(left[key],right[key])

    def test_rail_recorded_stages_share_demo_gain_policy_not_hand_or_alignment(self):
        from task_stage_control import segment_stage,recorded_replay_segment
        from task_rail_trace import STAGES
        for stage in STAGES:
            segment=dict(kind='arm',trace_stage=stage,label=stage)
            self.assertTrue(recorded_replay_segment('rail',segment))
            self.assertEqual(segment_stage('rail',segment),'withdraw')
            self.assertFalse(recorded_replay_segment('dring_demo',segment))
            self.assertEqual(segment_stage('rail',dict(segment,kind='hand')),'normal')
        for role in ('rail_alignment','saved_start_transition'):
            self.assertEqual(segment_stage('rail',dict(kind='arm',role=role,label='align')),'normal')
        self.assertEqual(segment_stage('rail',dict(kind='hand',label='Rail hand open')),'normal')
        self.assertEqual(segment_stage('dring_demo',dict(kind='arm',recording_fit={},label='demo')),'withdraw')

    def test_config_monitor_debounces_and_preserves_last_valid_snapshot(self):
        from task_config_monitor import PlanningConfigMonitor
        with TemporaryDirectory() as directory:
            path=Path(directory)/'planning.yaml';path.write_text('geometry: {height_offset_mm: 0}')
            now=[0.];m=PlanningConfigMonitor(path,lambda:now[0]);old=m.record
            self.assertFalse(m.poll());now[0]=.5;m.poll()
            path.write_text('geometry: {height_offset_mm: 12}')
            now[0]=1.;self.assertFalse(m.poll())
            now[0]=1.5;self.assertTrue(m.poll())
            self.assertFalse(m.matches(old,{}));self.assertEqual(m.restart_fields(),[])
            self.assertEqual(old['values']['height_offset_mm'],0)
            path.write_text('geometry: [')
            now[0]=2.;m.poll();now[0]=2.5;self.assertTrue(m.poll())
            self.assertIsNotNone(m.execution_issue())
            self.assertEqual(m.record['values']['height_offset_mm'],12)
            path.write_text('geometry: {height_offset_mm: 12}')
            now[0]=3.;m.poll();now[0]=3.5;self.assertTrue(m.poll())
            self.assertIsNone(m.execution_issue())

    def test_config_monitor_distinguishes_startup_changes_and_actual_gains(self):
        from task_config_monitor import PlanningConfigMonitor
        with TemporaryDirectory() as directory:
            path=Path(directory)/'planning.yaml';path.write_text('arm_pd: {left_elbow_kp: 55}')
            m=PlanningConfigMonitor(path)
            path.write_text('arm_pd: {left_elbow_kp: 75}\nstage_control: {rotate_velocity_factor: 0.3}')
            m.reload()
            self.assertEqual(m.restart_fields(),['left_elbow_kp'])
            self.assertEqual(m.restart_fields(dict(phase='ARMED_HOLD',left_elbow_joint_kp='75')),[])
            # Reopening a panel must not turn a file value into an assumed controller value.
            fresh=PlanningConfigMonitor(path)
            self.assertEqual(fresh.restart_fields(dict(phase='ARMED_HOLD',left_elbow_joint_kp='55')),
                             ['left_elbow_kp'])

    def test_config_update_does_not_invalidate_executing_task(self):
        import threading
        from real_task_app import TaskNode
        from task_config_monitor import PlanningConfigMonitor
        with TemporaryDirectory() as directory:
            path=Path(directory)/'planning.yaml';path.write_text('geometry: {height_offset_mm: 0}')
            m=PlanningConfigMonitor(path)
            session=S(phase='EXECUTING',plan={'planning_config':m.record},invalidate=Mock())
            n=S(planning_monitor=m,planning_config_revision=m.revision,planning_config_pending=False,
                lock=threading.RLock(),worker=None,session=session,gateway={},display_pub=Mock())
            path.write_text('geometry: {height_offset_mm: 10}');m.reload()
            TaskNode.poll_planning_config(n);session.invalidate.assert_not_called()
            self.assertTrue(n.planning_config_pending)
            session.phase='PLAN_READY';TaskNode.poll_planning_config(n)
            session.invalidate.assert_called_once();self.assertEqual(session.phase,'REPLAN_REQUIRED')

    def test_stage_speed_stretches_time_without_changing_positions(self):
        from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
        from task_stage_control import slow_segment,stage_name
        p=JointTrajectoryPoint(positions=[.2],velocities=[.1],accelerations=[.04])
        p.time_from_start.sec=2
        segment=dict(trajectory=JointTrajectory(joint_names=['joint'],points=[p]),duration=2.)
        slow_segment(segment,.4)
        self.assertEqual(segment['duration'],5.)
        self.assertEqual(list(p.positions),[.2])
        np.testing.assert_allclose(p.velocities,[.04])
        np.testing.assert_allclose(p.accelerations,[.0064])
        self.assertEqual(stage_name('Left 120'),'rotate')
        self.assertEqual(stage_name('Withdraw'),'withdraw')
        self.assertEqual(stage_name('Insert'),'normal')
        for value in (0,1.1,float('nan')):
            with self.assertRaises(ValueError):slow_segment(segment,value)

    def test_stage_settings_are_frozen_and_numeric_without_tuning_caps(self):
        old=parameters_from_values({})
        self.assertFalse(old.stage_control_enabled)
        self.assertFalse(old.right_sweep_waist_bias)
        self.assertEqual((old.left_sweep_hand_back_yaw_deg,old.right_sweep_hand_back_yaw_deg),(0.,0.))
        self.assertEqual((old.sweep_hand_back_min_deg,old.sweep_hand_back_max_deg),(0.,0.))
        with self.assertRaises(ValueError):parameters_from_values({'left_sweep_hand_back_yaw_deg':-21})
        self.assertTrue(parameters_from_values({'right_sweep_waist_bias':True}).right_sweep_waist_bias)
        with self.assertRaises(ValueError):parameters_from_values({'right_sweep_waist_bias':1})
        self.assertEqual(old.rotate_velocity_factor,1.)
        self.assertEqual((old.stage_waist_kp,old.stage_waist_kd),(0,0))
        for kp,kd in ((100,4.5),(200,8),(300,6),(500,12),(1000,20)):
            p=parameters_from_values({'rotate_proximal_kp':kp,'withdraw_proximal_kp':kp,
                                      'rotate_proximal_kd':kd,'withdraw_proximal_kd':kd})
            self.assertEqual(p.rotate_proximal_kp,kp)
            self.assertEqual(p.withdraw_proximal_kp,kp)
            self.assertEqual(p.rotate_proximal_kd,kd)
            self.assertEqual(p.withdraw_proximal_kd,kd)

        for values in ({'rotate_proximal_kp':-1},{'withdraw_proximal_kp':1e39},
                       {'rotate_proximal_kp':float('nan')},{'withdraw_proximal_kd':-1},
                       {'rotate_proximal_kd':1e39},{'withdraw_proximal_kd':float('inf')},
                       {'rotate_velocity_factor':0},{'stage_control_enabled':1}):
            with self.assertRaises(ValueError):parameters_from_values(values)
        for kp,kd in ((180,5),(300,7),(1000,20)):
            p=parameters_from_values(dict(stage_waist_kp=kp,stage_waist_kd=kd))
            self.assertEqual((p.stage_waist_kp,p.stage_waist_kd),(kp,kd))
        for values in ({'stage_waist_kp':120},{'stage_waist_kd':5},
                       {'stage_waist_kp':19,'stage_waist_kd':5},
                       {'stage_waist_kp':120,'stage_waist_kd':.4},
                       {'stage_waist_kp':1e39,'stage_waist_kd':5},
                       {'stage_waist_kp':120,'stage_waist_kd':float('nan')}):
            with self.assertRaises(ValueError):parameters_from_values(values)

    def test_hard_hand_heading_configuration_rejects_wrong_sign_and_empty_range(self):
        values=dict(left_sweep_hand_back_yaw_deg=-20.,right_sweep_hand_back_yaw_deg=20.,
                    sweep_hand_back_min_deg=10.,sweep_hand_back_max_deg=30.)
        self.assertEqual(parameters_from_values(values).sweep_hand_back_min_deg,10.)
        for change in (dict(sweep_hand_back_min_deg=30.),dict(sweep_hand_back_max_deg=0.),
                       dict(left_sweep_hand_back_yaw_deg=20.),dict(right_sweep_hand_back_yaw_deg=10.),
                       dict(hand_back_yaw_deg=25.),dict(sweep_hand_back_max_deg=float('nan'))):
            with self.subTest(change=change),self.assertRaises(ValueError):parameters_from_values(dict(values,**change))

    def test_stage_gateway_limits_are_checked_without_changing_settings(self):
        from task_stage_control import validate_gateway_gains
        p=replace(PlanningParameters(),rotate_proximal_kp=200.,withdraw_proximal_kp=160.,
                  rotate_proximal_kd=6.,withdraw_proximal_kd=6.)
        gateway=dict(task_stage_supported='true',task_stage_max_kp='200',task_stage_max_kd='8')
        validate_gateway_gains(p,gateway)
        for value in ('80','120','160','nan','inf','invalid',None):
            with self.subTest(value=value),self.assertRaises(RuntimeError):
                validate_gateway_gains(p,dict(gateway,task_stage_max_kp=value))
        with self.assertRaises(RuntimeError):
            validate_gateway_gains(p,dict(gateway,task_stage_max_kd='5'))
        self.assertEqual(p.rotate_proximal_kp,200.)
        p=replace(p,stage_waist_kp=120.,stage_waist_kd=5.)
        with self.assertRaises(RuntimeError):validate_gateway_gains(p,gateway)
        gateway.update(task_stage_waist_supported='true',task_waist_trajectory_supported='true',
                       waist_control_mode='guarded',task_stage_waist_max_kp='120',task_stage_waist_max_kd='5')
        validate_gateway_gains(p,gateway)
        with self.assertRaises(RuntimeError):validate_gateway_gains(p,dict(gateway,waist_control_mode='official_pd'))
        p=replace(p,rotate_proximal_kp=300.,withdraw_proximal_kp=240.,stage_waist_kp=180.)
        with self.assertRaises(RuntimeError):validate_gateway_gains(p,gateway)
        gateway.update(task_stage_max_kp='unbounded',task_stage_max_kd='unbounded',
                       task_stage_waist_max_kp='unbounded',task_stage_waist_max_kd='unbounded')
        validate_gateway_gains(p,gateway)
        validate_gateway_gains(replace(p,rotate_proximal_kp=1000.,rotate_proximal_kd=20.,
                                      stage_waist_kp=1000.,stage_waist_kd=20.),gateway)
        for field in ('rotate_proximal_kp','withdraw_proximal_kd','stage_waist_kp','stage_waist_kd'):
            with self.assertRaises(RuntimeError):validate_gateway_gains(replace(p,**{field:1e39}),gateway)
        validate_gateway_gains(PlanningParameters(),dict(task_stage_supported='true'))

    def test_operator_file_loads_with_a_reproducible_snapshot(self):
        params,record=load_parameters()
        self.assertEqual(params,parameters_from_values(record['values']))
        self.assertEqual(record['path'],str(DEFAULT_CONFIG))
        self.assertEqual(len(record['sha256']),64)

    def test_reload_freezes_previous_plan_and_tracks_file_hash(self):
        with TemporaryDirectory() as directory:
            path=Path(directory)/'config.yaml'
            path.write_text('geometry:\n  height_offset_mm: 10\n')
            before,old_record=load_parameters(path)
            path.write_text('geometry:\n  height_offset_mm: 15\n')
            after,new_record=load_parameters(path)
            self.assertEqual(before.height_offset_mm,10)
            self.assertEqual(after.height_offset_mm,15)
            self.assertNotEqual(old_record['sha256'],new_record['sha256'])
            with self.assertRaises(FrozenInstanceError):before.height_offset_mm=20

    def test_typo_duplicate_nonfinite_and_wrong_types_are_rejected(self):
        bad=('wrong: {}','repair: {typo: 3}','repair: {max_repairs: 3, max_repairs: 4}',
             'geometry: {height_offset_mm: .nan}','geometry: {radial_offset_mm: 101}',
             'geometry: {radial_offset_mm: -101}',
             'geometry: {height_offset_mm: 101}','geometry: {height_offset_mm: -31}',
             'repair: {backtrack_points: 0}','sampling: {arc_segments: 100.5}',
             'orientation: {tilt_deg: 61}','orientation: {step_deg: 0}',
             'orientation: {hand_back_yaw_deg: 46}','geometry: {path_tolerance_mm: 21}',
             'execution: {endpoint_tolerance_deg: 0}', 'execution: {endpoint_tolerance_deg: 2.9}',
             'execution: {endpoint_tolerance_deg: .nan}', 'execution: {endpoint_wait_seconds: true}',
             'execution: {endpoint_wait_seconds: 0}', 'execution: {endpoint_wait_seconds: 31}',
             'arm_integral: {shoulder_roll_ki: 11}', 'arm_integral: {wrist_yaw_integral_limit_nm: 0.31}',
             'arm_integral: {left_arm_integral_enabled: 1}', 'arm_integral: {elbow_ki: .nan}',
             'arm_pd: {left_elbow_kp: 101}', 'arm_pd: {right_elbow_kd: 5.1}',
             'arm_pd: {right_wrist_pitch_kp: 110.1}',
             'arm_pd: {left_wrist_roll_kd: 8.1}', 'arm_pd: {left_elbow_kp: .inf}',
             'arm_pd: {left_elbow_kd: true}', 'waist_controller: {waist_ki: 31}',
             'waist_controller: {waist_integral_torque_limit_nm: 0}',
             'search: {small_step_fast_path: "false"}','search: {max_attempts: true}')
        with TemporaryDirectory() as directory:
            path=Path(directory)/'config.yaml'
            for text in bad:
                with self.subTest(text=text):
                    path.write_text(text)
                    with self.assertRaises(ValueError):load_parameters(path)

    def test_execution_config_snapshot_is_validated_and_not_changed_by_reload(self):
        with TemporaryDirectory() as directory:
            path=Path(directory)/'config.yaml'
            path.write_text('execution: {endpoint_tolerance_deg: 2.0, endpoint_wait_seconds: 8.0}')
            params,record=load_parameters(path)
            path.write_text('execution: {endpoint_tolerance_deg: 1.0}')
            latest,_=load_parameters(path)
            frozen=parameters_from_values(record['values'])
            self.assertEqual(frozen,params)
            self.assertEqual(frozen.endpoint_tolerance_deg,2.)
            self.assertEqual(frozen.endpoint_wait_seconds,8.)
            self.assertEqual(latest.endpoint_tolerance_deg,1.)
            with self.assertRaises(ValueError):parameters_from_values({'endpoint_tolerance_deg':float('inf')})

    def test_arm_integral_joint_order_and_right_arm_disabled_by_default(self):
        parameters=PlanningParameters()
        left=parameters.arm_integral_parameters('left')
        self.assertTrue(left['arm_integral_enabled'])
        self.assertEqual(left['arm_integral_ki'],[5.,5.,3.,3.,1.,1.,2.])
        self.assertEqual(left['arm_integral_limits_nm'],[.6,.8,.35,.4,.15,.2,.25])
        self.assertFalse(parameters.arm_integral_parameters('right')['arm_integral_enabled'])
        with self.assertRaises(ValueError):parameters.arm_integral_parameters('both')

    def test_custom_arc_density_and_fixed_initial_reference(self):
        class Planner(StraightIndexPolicy,TaskPlanner):pass
        planner=Planner.__new__(Planner)
        params=replace(PlanningParameters(),arc_segments=100,line_points=41,arc_yaw_gain=0.,
                       tilt_deg=7.,twist_deg=12.,step_deg=3.,max_repairs=5,backtrack_points=8)
        configure_planner(planner,params)
        pre=np.eye(4);pre[:3,3]=[.4,.1,.1]
        nominal=Rotation.from_euler('y',145,degrees=True).as_quat()
        planner.cartesian=Mock(return_value={'end':{}})
        planner.tip=Mock(return_value=pre)
        planner.contact_segments('left',{},pre,np.array([.4,.1,0.]),np.array([.3,.1,0.]),nominal,np.eye(4),None)
        calls=planner.cartesian.call_args_list
        self.assertEqual([len(c.args[2]) for c in calls],[40,100,40])
        np.testing.assert_allclose(calls[1].args[3],nominal,atol=1e-12)
        self.assertEqual(params.orientation_policy(),dict(tilt_deg=7.,roll_deg=12.,step_deg=3.))

    def test_controller_parameters_preserve_legacy_defaults_and_side_order(self):
        params=PlanningParameters()
        for side in ('left','right'):
            settings=params.controller_parameters(side)
            for arm in ('left','right'):
                self.assertEqual(settings[arm+'_arm_kp'],[55.,55.,55.,55.,40.,55.,55.])
                self.assertEqual(settings[arm+'_arm_kd'],[3.,3.,3.,3.,3.5,3.,3.])
            self.assertEqual(settings['waist_kp'],60.)
            self.assertEqual(settings['waist_integral_torque_limit_nm'],1.2)
        changed=replace(params,left_elbow_kp=65.,right_shoulder_yaw_kd=3.4)
        settings=changed.controller_parameters('right')
        self.assertEqual(settings['left_arm_kp'][3],65.)
        self.assertEqual(settings['right_arm_kd'][2],3.4)
        self.assertFalse(settings['arm_integral_enabled'])
        with self.assertRaises(ValueError):params.controller_parameters('both')

    def test_proximal_100_and_mirrored_pd_reach_controller(self):
        values={side+'_'+joint+'_kp':100. for side in ('left','right')
                for joint in ('shoulder_pitch','shoulder_roll','shoulder_yaw','elbow')}
        params=parameters_from_values(values)
        for active in ('left','right'):
            settings=params.controller_parameters(active)
            for arm in ('left','right'):
                self.assertEqual(settings[arm+'_arm_kp'][:4],[100.]*4)

    def test_wrist_110_reaches_both_arm_controllers_without_changing_kd(self):
        fields=[side+'_'+joint+'_kp' for side in ('left','right')
                for joint in ('wrist_roll','wrist_pitch','wrist_yaw')]
        params=parameters_from_values(dict.fromkeys(fields,110.))
        for active in ('left','right'):
            settings=params.controller_parameters(active)
            baseline=PlanningParameters().controller_parameters(active)
            for arm in ('left','right'):
                self.assertEqual(settings[arm+'_arm_kp'][4:],[110.]*3)
                self.assertEqual(settings[arm+'_arm_kp'][:4],baseline[arm+'_arm_kp'][:4])
                self.assertEqual(settings[arm+'_arm_kd'],baseline[arm+'_arm_kd'])
        for field in fields:
            for invalid in (-.1,110.1,float('nan'),float('inf')):
                with self.subTest(field=field,value=invalid):
                    with self.assertRaises(ValueError):parameters_from_values({field:invalid})

    def test_operator_wrist_gains_only_override_demo_playback(self):
        params,_=load_parameters()
        self.assertEqual(params.demo_wrist_kp,110.)
        self.assertEqual(params.demo_distal_wrist_kp,220.)
        for active in ('left','right'):
            settings=params.controller_parameters(active)
            for arm in ('left','right'):
                self.assertEqual(settings[arm+'_arm_kp'][4:],[40.,55.,55.])
                self.assertEqual(settings[arm+'_arm_kd'][4:],[3.5,3.,3.])

    def test_per_joint_replay_kp_reload_is_independent_and_keeps_old_snapshot(self):
        from task_stage_control import replay_arm_kp
        params=parameters_from_values(dict(zip(
            ('replay_'+j+'_kp' for j in ('shoulder_pitch','shoulder_roll','shoulder_yaw','elbow','wrist_roll','wrist_pitch','wrist_yaw')),
            (550.,550.,550.,550.,110.,220.,220.))))
        self.assertEqual(replay_arm_kp(params,'left'),[550.,550.,550.,550.,110.,220.,220.])
        self.assertEqual(replay_arm_kp(params,'right'),replay_arm_kp(params,'left'))
        with TemporaryDirectory() as directory:
            path=Path(directory)/'config.yaml'
            path.write_text('demo_replay: {replay_wrist_pitch_kp: 230, replay_wrist_yaw_kp: 240}')
            old,snapshot=load_parameters(path)
            path.write_text('demo_replay: {replay_wrist_pitch_kp: 250, replay_wrist_yaw_kp: 240}')
            new,_=load_parameters(path)
            self.assertEqual(replay_arm_kp(old,'left')[-2:],[230.,240.])
            self.assertEqual(replay_arm_kp(new,'left')[-2:],[250.,240.])
            self.assertEqual(parameters_from_values(snapshot['values']),old)
            self.assertEqual(old.controller_parameters('left'),new.controller_parameters('left'))
        self.assertIsNone(replay_arm_kp(PlanningParameters(),'left'))
        for value in (-2,-.5,float('inf'),float('nan'),1e39):
            with self.assertRaises(ValueError):parameters_from_values({'replay_elbow_kp':value})
        zero=parameters_from_values({'replay_elbow_kp':0})
        self.assertEqual(replay_arm_kp(zero,'left')[3],0.)

    def test_replay_waist_kp_has_no_tuning_ceiling_and_keeps_base_and_snapshot(self):
        from task_stage_control import replay_stage_parameters
        baseline=parameters_from_values(dict(stage_waist_kp=120.,stage_waist_kd=5.))
        self.assertIs(replay_stage_parameters(baseline),baseline)
        for value in (20.,180.,550.,1000.,1000000.):
            p=parameters_from_values(dict(stage_waist_kp=120.,stage_waist_kd=5.,replay_waist_kp=value))
            resolved=replay_stage_parameters(p)
            self.assertEqual((resolved.stage_waist_kp,resolved.stage_waist_kd),(value,5.))
            self.assertEqual(p.stage_waist_kp,120.)
            self.assertEqual(p.controller_parameters('left'),baseline.controller_parameters('left'))
        for value in (-2.,-.5,1.,float('nan'),float('inf'),1e39):
            with self.assertRaises(ValueError):parameters_from_values(dict(replay_waist_kp=value))
        zero=replay_stage_parameters(replace(baseline,replay_waist_kp=0.))
        self.assertEqual((zero.stage_waist_kp,zero.stage_waist_kd),(0.,0.))
        fallback=replay_stage_parameters(parameters_from_values(dict(replay_waist_kp=180.)))
        self.assertEqual(fallback.stage_waist_kd,fallback.waist_kd)
        with TemporaryDirectory() as directory:
            path=Path(directory)/'config.yaml'
            path.write_text('demo_replay: {replay_waist_kp: 180}')
            old,snapshot=load_parameters(path)
            path.write_text('demo_replay: {replay_waist_kp: 240}')
            new,_=load_parameters(path)
            self.assertEqual(old.replay_waist_kp,180.)
            self.assertEqual(new.replay_waist_kp,240.)
            self.assertEqual(parameters_from_values(snapshot['values']),old)

    def test_explicit_tuning_keeps_tolerance_and_unmeasured_right_arm(self):
        current=replace(PlanningParameters(),left_shoulder_pitch_kp=60.,left_shoulder_roll_kp=60.,
            left_elbow_kp=65.,left_shoulder_pitch_kd=3.2,left_shoulder_roll_kd=3.2,
            left_elbow_kd=3.5,elbow_ki=6.,elbow_integral_limit_nm=.8)
        old=PlanningParameters()
        settings=current.controller_parameters('left')
        self.assertEqual(settings['left_arm_kp'],[60.,60.,55.,65.,40.,55.,55.])
        self.assertEqual(settings['left_arm_kd'],[3.2,3.2,3.,3.5,3.5,3.,3.])
        self.assertEqual(settings['arm_integral_ki'][3],6.)
        self.assertEqual(settings['arm_integral_limits_nm'][3],.8)
        self.assertEqual(current.endpoint_tolerance_deg,old.endpoint_tolerance_deg)
        self.assertEqual(current.endpoint_wait_seconds,old.endpoint_wait_seconds)
        for gain in ('kp','kd'):
            self.assertEqual(settings['right_arm_'+gain],old.controller_parameters('left')['right_arm_'+gain])

    def test_configured_search_batch_size_is_used(self):
        from test_task_incremental_search import IncrementalSearchTests
        fixture=IncrementalSearchTests();fixture.setUp()
        configure_planner(fixture.p,replace(PlanningParameters(),batch_trials=10,batch_candidates=1))
        self.assertEqual(fixture.run_ring(),['complete'])
        self.assertEqual(fixture.calls,10)
        self.assertEqual(fixture.p.search_diagnostics['preinsert']['search_policy'],'incremental_10_trials')

    def test_zero_repairs_does_not_reselect_saved_alternative(self):
        from test_task_planner_contract import PlannerContractTests
        from task_session import ARMS
        fixture=PlannerContractTests();fixture.setUp();fixture.configure_ranking()
        p=fixture.p;configure_planner(p,replace(PlanningParameters(),max_repairs=0))
        states=[dict(fixture.q,**dict.fromkeys(ARMS['left'],value)) for value in (.16,.17)]
        p.solve=Mock(side_effect=[*states,None,None])
        with patch('task_motion_planner.orientation_candidates',return_value=[[0,0,0,1]]*2):
            with self.assertRaisesRegex(PlanningFailure,'CONTINUOUS_IK_FAILED'):
                p.cartesian('left',fixture.q,[[0,0,0]]*2,[0,0,0,1],np.eye(4),fixture.ticket,
                            'Insert',lambda point:0.,'insert')
        self.assertEqual(p.search_diagnostics['cartesian'][0]['local_repairs'],[])
        self.assertEqual(p.solve.call_count,4)

    def test_configured_tolerance_reaches_final_geometry_check(self):
        planner=TaskPlanner.__new__(TaskPlanner)
        tip=np.eye(4);tip[:3,:3]=Rotation.from_euler('x',6,degrees=True).as_matrix()
        report=dict(checked_samples=0,max_position_error_m=0.,max_tilt_deg=0.,max_twist_deg=0.)
        default=PlanningParameters().orientation_policy()
        custom=replace(PlanningParameters(),tilt_deg=7.).orientation_policy()
        args=(np.array([tip]),[0,0,0,1],lambda point:0.)
        self.assertIsNotNone(planner.check_tcp_geometry_batch(*args,default,'insert','Insert',dict(report)))
        self.assertIsNone(planner.check_tcp_geometry_batch(*args,custom,'insert','Insert',dict(report)))


if __name__=='__main__':unittest.main()
