"""Screen workflow regressions with real FK and fake I/O; never initializes ROS."""
import copy
import json
import math
import os
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock, patch

import numpy as np

import task_screen_teach as data
from task_screen_control import ScreenControl
from task_screen_motion import plan, normal_states
from task_session import Session, ARM_NAMES
from task_hand_control import POSTURES, model_joints
from task_planning_config import PlanningParameters
from task_saved_test import range_fingerprint
from task_trajectory import samples, task_motion_limits
from workbench_model import OfflineModel

ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT/'assets/workbench/task/robot.urdf'
HISTORY = ROOT/'data/task_sessions/20260913_160141_410400/plan_0150/center_check_result.json'


class ScreenTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = OfflineModel(URDF)
        cls.historical = json.loads(HISTORY.read_text())

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        self.directory = Path(self.tmp.name)
        self.q = dict(self.historical['measured_joints'])
        self.hands = {side:dict(code=0, hand=side, online=True, feedback_q=list(POSTURES['index_only']),
            target_q=list(POSTURES['index_only']), error=[0]*6, temperature=[30]*6, timestamp_ms=1000)
            for side in ('left', 'right')}
        for side in self.hands: self.q.update(model_joints(side, self.hands[side]['feedback_q']))
        self.calibration = self.historical['calibration']['values']
        self.tcp = data.effective_tcp(self.model, self.q, self.calibration)
        self.record = data.endpoint(self.model, self.q, self.hands, self.tcp, 100, 3)
        self.config = data.new_config({'test_identity':'offline'})
        self.config['points'] = {slot:copy.deepcopy(self.record) for slot in data.SLOTS}
        self.config['plane'] = dict(outward_normal=[0., 0., 1.])

    def node(self):
        t = time.monotonic()-.55
        session = Session(lambda:t)
        for i in range(12):
            session.update(self.q)
            t += .05 if i < 11 else 0.
        session.clock = time.monotonic
        session.capture_home()
        n = S(args=S(robot_urdf=str(URDF), intrinsics=str(ROOT/'../unitree_ws/head_camera_intrinsics.json'),
            hardware=True, disconnected=False), calibration=self.calibration, model=self.model,
            lock=threading.RLock(), session=session, run_dir=self.directory, worker=None, shutdown_requested=False,
            hand_control=True, gateway_received=time.monotonic(), hand_states={}, marker_pub=Mock(), display_pub=Mock(),
            check=lambda ticket=None:None,
            gateway=dict(phase='ARMED_HOLD', task_stop_active='false', task_stop_epoch='3', armed='true',
                waist_control_mode='guarded', waist_control_revision='0', waist_mode_switching='false',
                hardware_output_available='true', task_application_enabled='true', upper_body_state_safe='true',
                service_state_ready='true', mode_machine='6', mode_pr='0', max_measured_velocity_rad_s='0',
                torque_overload_active='false', command_overload_active='false', velocity_recovery_active='false',
                temperature_alarm_active='false', feedforward_saturated='false', service_conflict='false',
                waist_guard_lower_rad=str(self.model.limits['waist_yaw_joint'][0]+data.JOINT_MARGIN),
                waist_guard_upper_rad=str(self.model.limits['waist_yaw_joint'][1]-data.JOINT_MARGIN),
                task_waist_trajectory_supported='true'), status='', launch_work=lambda f:f())
        def read_hands(ticket=None):
            for state in self.hands.values():state['timestamp_ms']+=1
            for side, state in self.hands.items():n.hand_states[side]=dict(state=state, received=time.monotonic())
            t=time.monotonic()-.55
            session.clock=lambda:t
            session.history=[];session.received=float('-inf')
            for i in range(12):
                session.update(self.q)
                t += .05 if i < 11 else 0.
            session.clock=time.monotonic
            return copy.deepcopy(self.hands)
        n.read_hands = read_hands
        n.gateway['waist_hold_reference_policy']='continuous_hold_v1'
        n.gateway.update({'hold_reference_'+k:repr(self.q[k]) for k in data.MOVING})
        return n

    def plane_config(self):
        config = copy.deepcopy(self.config)
        center = np.array([.4, -.1, .2])
        for slot, offset in zip(data.SLOTS[1:], [[-.04,-.03,0],[.04,-.03,0],[.04,.03,0],[-.04,.03,0],[0,0,0]]):
            config['points'][slot]['pelvis_from_tcp'] = np.eye(4).tolist()
            config['points'][slot]['pelvis_from_tcp'][0][3] = float(center[0]+offset[0])
            config['points'][slot]['pelvis_from_tcp'][1][3] = float(center[1]+offset[1])
            config['points'][slot]['pelvis_from_tcp'][2][3] = float(center[2]+offset[2])
        config['points']['T']['pelvis_from_tcp'][2][3] = .3
        config['plane'] = data.fit_plane(config['points'])
        return config

    def test_plane_sign_degeneracy_and_bundle_checksum(self):
        config = self.plane_config()
        self.assertAlmostEqual(config['plane']['max_mm'], 0.)
        self.assertGreater(config['plane']['outward_normal'][2], .99)
        config['reference'] = dict(frame='pelvis')
        path = self.directory/'config.json'
        cloud = np.random.default_rng(3).normal(0, .1, (100, 3))
        data.save_bundle(path, config, cloud)
        loaded, actual = data.load_bundle(path)
        np.testing.assert_allclose(actual, cloud, atol=1e-7)
        with (self.directory/loaded['reference']['file']).open('ab') as stream:stream.write(b'broken')
        with self.assertRaisesRegex(ValueError, 'checksum'):data.load_bundle(path)
        with self.assertRaisesRegex(ValueError, 'collinear'):data.fit_plane(self.config['points'])
        with self.assertRaises(ValueError):data.parameters(dict(data.DEFAULTS, extra_feed_mm=3))

    def test_real_fk_dry_and_touch_keep_waist_fixed_return_T(self):
        initial = dict(self.q); initial['waist_yaw_joint'] -= .04
        self.config['points']['T']['joints'] = initial
        for dry in (True, False):
            with self.subTest(dry=dry):
                result = plan(self.model, self.config, 'P1', initial, dry, PlanningParameters())
                self.assertFalse(result['collision_checked'])
                self.assertEqual(len(result['segments']), 5)
                self.assertEqual(result['segments'][0]['trajectory'].joint_names, list(data.MOVING))
                for n in data.MOVING:self.assertAlmostEqual(result['segments'][-1]['end'][n], initial[n])
                for segment in result['segments'][1:-1]:
                    for p in segment['trajectory'].points:self.assertAlmostEqual(p.positions[-1], self.q['waist_yaw_joint'])
                end = result['segments'][1]['end']
                actual = self.model.transform('right_wrist_yaw_link', end) @ self.tcp
                delta = actual[:3, 3]-np.asarray(self.record['pelvis_from_tcp'])[:3, 3]
                self.assertAlmostEqual(delta[2], .01 if dry else 0., delta=.0003)
                if not dry:
                    for n in data.MOVING:self.assertAlmostEqual(end[n], self.q[n], places=7)

    def test_planning_cancellation_exits_ik(self):
        def cancel():raise RuntimeError('cancelled')
        with self.assertRaisesRegex(RuntimeError, 'cancelled'):
            plan(self.model, self.config, 'P1', self.q, True, PlanningParameters(), cancel)

    def test_waist_recovery_before_any_teaching_and_old_gateway_rejected(self):
        n=self.node();c=ScreenControl(n)
        c.plan('WAIST',True,return_confirmed=True,waist_target_deg=0.)
        result=n.session.plan
        self.assertEqual(c.config['points'],{})
        self.assertFalse(c.aligned)
        self.assertEqual(result['screen']['operation'],'waist_recovery')
        self.assertFalse(result['collision_checked'])
        segment=result['segments'][0]
        for name in ARM_NAMES:self.assertEqual(segment['end'][name],segment['start'][name])
        self.assertEqual(segment['end']['waist_yaw_joint'],0.)
        self.assertEqual(list(segment['trajectory'].points[0].velocities),[0.]*8)
        self.assertEqual(list(segment['trajectory'].points[-1].accelerations),[0.]*8)
        n.gateway.pop('waist_guard_upper_rad')
        with self.assertRaisesRegex(RuntimeError,'限幅不一致'):c.execution_guard(result,preflight=True)

    def test_waist_reference_start_is_monotonic_and_separate_from_measurement(self):
        for mode in ('guarded', 'official_pd'):
            for held_deg, measured_deg, target_deg in ((0., -.25, 20.), (20., 19.9, 0.), (20., 19.9, 20.)):
                with self.subTest(mode=mode, target=target_deg, held=held_deg):
                    self.q['waist_yaw_joint']=math.radians(measured_deg)
                    n=self.node();c=ScreenControl(n)
                    n.gateway['waist_control_mode']=mode
                    n.gateway['hold_reference_waist_yaw_joint']=repr(math.radians(held_deg))
                    n.gateway['hold_reference_right_shoulder_pitch_joint']=repr(self.q['right_shoulder_pitch_joint']+.002)
                    c.plan('WAIST',True,return_confirmed=True,waist_target_deg=target_deg)
                    result=n.session.plan;segment=result['segments'][0];trajectory=segment['trajectory']
                    self.assertEqual(result['start']['waist_yaw_joint'],math.radians(measured_deg))
                    self.assertEqual(trajectory.points[0].positions[-1],math.radians(held_deg))
                    self.assertAlmostEqual(trajectory.points[-1].positions[-1],math.radians(target_deg))
                    vmax,amax=task_motion_limits(data.MOVING,.15,.15)
                    values=np.asarray(list(samples(trajectory,vmax,amax)))
                    direction=np.sign(target_deg-held_deg)
                    self.assertTrue(np.all(direction*np.diff(values[:,-1])>=-1e-12))
                    if direction==0:np.testing.assert_allclose(values[:,-1],math.radians(held_deg),atol=1e-12)
                    np.testing.assert_allclose(values[:,0],self.q['right_shoulder_pitch_joint']+.002,atol=1e-12)
                    c.execution_guard(result,preflight=True)
                    n.gateway['hold_reference_waist_yaw_joint']=repr(math.radians(held_deg)+.001)
                    with self.assertRaisesRegex(RuntimeError,'参考已改变'):c.execution_guard(result,preflight=True)

    def test_waist_reference_missing_excess_error_and_tamper_rejected(self):
        n=self.node();c=ScreenControl(n)
        n.gateway.pop('waist_hold_reference_policy')
        with self.assertRaisesRegex(RuntimeError,'新版网关'):c.waist_hold_reference()
        n.gateway['waist_hold_reference_policy']='continuous_hold_v1'
        n.gateway['hold_reference_waist_yaw_joint']='nan'
        with self.assertRaisesRegex(RuntimeError,'非有限'):c.waist_hold_reference()
        n.gateway['hold_reference_waist_yaw_joint']=repr(self.q['waist_yaw_joint']+.026)
        with self.assertRaisesRegex(ValueError,'0.025'):c.plan('WAIST',True,return_confirmed=True,waist_target_deg=0.)
        n=self.node();c=ScreenControl(n)
        c.plan('WAIST',True,return_confirmed=True,waist_target_deg=0.)
        result=n.session.plan
        result['segments'][0]['trajectory'].points[0].positions[-1]+=.001
        with self.assertRaisesRegex(RuntimeError,'不得改变'):c.execution_guard(result,preflight=True)

    def test_record_uses_measured_feedback_and_stop_cancels_capture(self):
        n = self.node(); c = ScreenControl(n)
        n.gateway.update(phase='TASK_ADMITTANCE', task_stop_active='true')
        with self.assertRaisesRegex(RuntimeError, '示教前'): c.record('P1')
        c.aligned = True
        c.record('P1')
        self.assertTrue(c.aligned)
        self.assertEqual(c.config['points']['P1']['joints']['right_elbow_joint'], self.q['right_elbow_joint'])
        self.assertTrue((self.directory/'screen_autosave/config.json').exists())
        old = n.read_hands
        old()  # Refresh fake feedback after the synchronous fsync/FK work above.
        def stopped(ticket=None):
            result=old(); n.session.stop(); return result
        n.read_hands=stopped
        with self.assertRaisesRegex(RuntimeError, 'STOP'):c.record('P2')
        self.assertNotIn('P2', c.config['points'])
        n.session.current['waist_yaw_joint'] = n.model.limits['waist_yaw_joint'][1]
        for _, q in n.session.history: q['waist_yaw_joint'] = n.session.current['waist_yaw_joint']
        with self.assertRaisesRegex(ValueError, 'P2.*waist_yaw_joint.*可用范围'): c.record('P2')
        self.assertNotIn('P2', c.config['points'])

    def test_load_does_not_restore_alignment_or_execution(self):
        n = self.node(); c = ScreenControl(n)
        c.config['points']['T'] = copy.deepcopy(self.record)
        path = self.directory/'config.json'; c.save(path)
        c.aligned=True; c.dry_checks={'P1','P3'}
        c.load(path)
        self.assertFalse(c.aligned); self.assertEqual(c.dry_checks, set()); self.assertIsNone(n.session.plan)

    def test_guard_rejects_stop_overload_changed_calibration_and_tampering(self):
        n=self.node(); c=ScreenControl(n); c.aligned=True
        result=dict(screen=dict(revision=0, stop_epoch=3, config_sha256=data.fingerprint(c.config),
            waist_control_mode='guarded', waist_control_revision='0',
            trajectory_sha256=range_fingerprint([])), segments=[])
        c.execution_guard(result, preflight=True)
        n.gateway['waist_mode_switching']='true'
        with self.assertRaisesRegex(RuntimeError, '过渡'):c.execution_guard(result, preflight=True)
        n.gateway['waist_mode_switching']='false'
        n.gateway['waist_control_revision']='1'
        with self.assertRaisesRegex(RuntimeError, '模式已改变'):c.execution_guard(result, preflight=True)
        n.gateway['waist_control_revision']='0'
        n.gateway['command_overload_active']='true'
        with self.assertRaisesRegex(RuntimeError, 'command_overload'):c.execution_guard(result, running=True)
        n.gateway['command_overload_active']='false'; n.gateway['task_stop_active']='true'
        with self.assertRaisesRegex(RuntimeError, 'STOP'):c.execution_guard(result, running=True)
        n.gateway['task_stop_active']='false'; c.config['parameters']['speed_mm_s']=3
        with self.assertRaisesRegex(RuntimeError, '内容'):c.execution_guard(result, preflight=True)

    def test_switch_mode_in_hold_without_arm_or_restart(self):
        from real_task_app import TaskNode
        n=self.node(); n.screen=ScreenControl(n)
        n.screen.aligned=True; n.screen.dry_checks={'P1', 'P3'}
        n.waist_parameters=object()
        n.call=Mock(return_value=S(result=S(successful=True)))
        TaskNode.set_waist_control_mode(n, 'official_pd')
        n.call.assert_called_once()
        client, request=n.call.call_args.args
        self.assertIs(client, n.waist_parameters)
        self.assertEqual(len(request.parameters), 1)
        self.assertEqual(request.parameters[0].name, 'waist_control_mode')
        self.assertEqual(request.parameters[0].value.string_value, 'official_pd')
        self.assertTrue(n.screen.aligned)
        self.assertEqual(n.screen.dry_checks, set())
        n.gateway['waist_mode_switching']='true'
        with self.assertRaisesRegex(RuntimeError, '过渡'):TaskNode.set_waist_control_mode(n, 'guarded')
        n.call.assert_called_once()
        n.gateway.update(waist_control_mode='official_pd', waist_mode_switching='false')
        with self.assertRaisesRegex(RuntimeError, '原增益编辑'):
            TaskNode.set_waist_gains(n, 74., 5., 16., 1.8)
        n.call.assert_called_once()

    def test_single_point_production_execution_and_fault_without_retry(self):
        from builtin_interfaces.msg import Time
        from real_task_app import TaskNode
        n=self.node(); c=ScreenControl(n); n.screen=c
        # Construct a reachable virtual screen from the real model, not a hardware capture.
        for slot, axis in zip(data.SLOTS[1:], ([0,0,0],[1,0,0],[0,1,0],[-1,0,0],[0,-1,0])):
            q=dict(self.q)
            if any(axis):
                _, states=normal_states(self.model,self.record,np.asarray(axis),.025,0.,lambda:None)
                q=states[-1]
            c.config['points'][slot]=data.endpoint(self.model,q,self.hands,self.tcp,100,3)
        _, states=normal_states(self.model,self.record,np.array([0.,0.,1.]),.06,0.,lambda:None)
        physical=dict(states[-1]);physical['waist_yaw_joint']-=.03
        c.config['points']['T']=data.endpoint(self.model,physical,self.hands,self.tcp,100,3)
        c.config['plane']=data.fit_plane(c.config['points']);c.config['plane']['confirmed']=True
        c.aligned=True
        clock=[time.monotonic()]; n.session.clock=lambda:clock[0]
        n.closing=threading.Event(); n.check=lambda ticket=None:TaskNode.check(n,ticket)
        goals=[]; fail_on=[None]
        def advance(seconds):
            deadline=clock[0]+seconds
            while clock[0]<deadline:
                clock[0]=min(deadline,clock[0]+.01)
                n.gateway_received=clock[0];n.session.update(physical)
        advance(.6)
        def hands(ticket=None):
            n.check(ticket);advance(.04)
            result=copy.deepcopy(self.hands)
            for side,state in result.items():
                state['timestamp_ms']=round(clock[0]*1000)
                n.hand_states[side]=dict(state=state,received=clock[0])
            return result
        n.read_hands=hands
        n.hand=S(begin_motion=Mock())
        n.get_clock=lambda:S(now=lambda:S(to_msg=lambda:Time()))
        def send(goal):
            goals.append(copy.deepcopy(goal.trajectory))
            return S(accepted=True,get_result_async=lambda:S(trajectory=goal.trajectory))
        n.actions={'right':S(wait_for_server=lambda **kw:True,send_goal_async=send)}
        n.waist_action=S(wait_for_server=lambda **kw:True,send_goal_async=Mock(side_effect=send))
        n.gateway.update({'hold_reference_'+k:repr(physical[k]) for k in data.MOVING})
        def wait(future,ticket=None,*args):
            n.check(ticket)
            if not hasattr(future,'trajectory'):return future
            if fail_on[0]==len(goals):
                return S(status=6,result=S(error_code=-1,error_string='injected blocked motion'))
            n.gateway['phase']='RUNNING';n.check(ticket)
            physical.update(zip(future.trajectory.joint_names,future.trajectory.points[-1].positions))
            n.gateway.update({'hold_reference_'+k:repr(physical[k]) for k in data.MOVING})
            advance(.55);n.gateway['phase']='ARMED_HOLD'
            return S(status=4,result=S(error_code=0,error_string='offline'))
        n.wait=wait
        n.stop=Mock(side_effect=lambda:(c.invalidate(),n.session.stop()))
        with patch('real_task_app.time.monotonic',lambda:clock[0]),patch('real_task_app.time.sleep',advance):
            c.aligned=False
            before=dict(physical)
            with self.assertRaisesRegex(RuntimeError, '明确确认'):
                c.plan('WAIST',True,waist_target_deg=0.)
            with self.assertRaisesRegex(ValueError, '腰部恢复目标'):
                c.plan('WAIST',True,waist_target_deg=60.,return_confirmed=True)
            c.plan('WAIST',True,waist_target_deg=0.,return_confirmed=True)
            self.assertEqual(goals,[])
            waist_plan=n.session.plan
            TaskNode.execute(n,scene_confirmed=True)
            self.assertEqual(len(goals),1)
            n.waist_action.send_goal_async.assert_called_once()
            self.assertAlmostEqual(physical['waist_yaw_joint'],0.)
            for name in ARM_NAMES:self.assertAlmostEqual(physical[name],before[name])
            self.assertTrue((Path(waist_plan['directory'])/'waist_result.json').exists())
            self.assertFalse(c.aligned);self.assertIsNone(c.last_result)
            goals.clear()
            physical['right_elbow_joint']+=.08
            physical['left_shoulder_pitch_joint']+=.10
            advance(.6)
            c.aligned=False
            with self.assertRaisesRegex(RuntimeError,'明确确认'):c.plan('T',True)
            c.plan('T',True,return_confirmed=True)
            self.assertEqual(goals,[])
            self.assertEqual(n.session.plan['start']['right_elbow_joint'],physical['right_elbow_joint'])
            self.assertEqual(n.session.plan['segments'][0]['end']['left_shoulder_pitch_joint'],physical['left_shoulder_pitch_joint'])
            n.session.plan['screen']['return_confirmed']=False
            with self.assertRaisesRegex(RuntimeError,'独立确认'):TaskNode.execute(n,scene_confirmed=True)
            self.assertEqual(goals,[])
            n.session.plan['screen']['return_confirmed']=True
            TaskNode.execute(n,scene_confirmed=True)
            self.assertEqual(len(goals),1)
            self.assertFalse(c.aligned);self.assertEqual(c.dry_checks,set());self.assertIsNone(c.last_result)
            for name in data.MOVING:
                self.assertAlmostEqual(physical[name],c.config['points']['T']['joints'][name])
            c.aligned=True
            c.plan('P1',True)
            TaskNode.execute(n,scene_confirmed=True)
            self.assertEqual(len(goals),6);self.assertEqual(n.session.phase,'ARRIVED_HOLD')
            self.assertEqual(c.last_result['outcome'],'pending')
            c.confirm_result(True);self.assertEqual(c.dry_checks,{'P1'})
            with self.assertRaisesRegex(RuntimeError,'两个'):c.plan('P3',False)
            c.plan('P3',True)
            TaskNode.execute(n,scene_confirmed=True);c.confirm_result(True)
            self.assertEqual(c.dry_checks,{'P1','P3'})
            c.plan('P1',False);fail_on[0]=len(goals)+2
            with self.assertRaisesRegex(RuntimeError,'blocked'):TaskNode.execute(n,scene_confirmed=True)
            self.assertEqual(len(goals),13)
            n.stop.assert_called_once();self.assertFalse(c.aligned);self.assertIsNone(c.last_result)

    def test_reference_crop_refresh_and_stale_hand_frames(self):
        n=self.node(); c=ScreenControl(n)
        with self.assertRaisesRegex(RuntimeError, '对齐要求'):c.alignment_pose()
        n.session.current['waist_yaw_joint']=0.
        for _, q in n.session.history:q['waist_yaw_joint']=0.
        cloud=np.random.default_rng(2).normal(0,.1,(100,3)).astype(np.float32)
        c.current_capture=dict(xyz=cloud,uv=np.column_stack([np.arange(100),np.zeros(100)]),
            received=time.monotonic(),captured_at=data.now(),source=2,camera_serial='offline',waist_yaw_rad=0.)
        c.current_cloud=cloud.copy();c.set_reference([0,0,49,1])
        self.assertEqual(len(c.reference),50)
        c.confirm_alignment();self.assertTrue(c.aligned)
        self.assertEqual(c.config['reference']['alignment_waist_yaw_rad'],0.)
        c.set_parameters(dict(data.DEFAULTS, dwell_s=.2));self.assertTrue(c.aligned)
        c.config['reference'].pop('waist_yaw_rad')
        with self.assertRaisesRegex(RuntimeError, '旧参考'):c.confirm_alignment()
        c.config['reference']['waist_yaw_rad']=0.
        reference=c.reference.copy();c.current_cloud+=.1
        np.testing.assert_array_equal(c.reference,reference)
        c.current_capture['received']-=121
        with self.assertRaisesRegex(RuntimeError,'两分钟'):c.confirm_alignment()
        with patch.object(n,'read_hands',return_value=self.hands):
            for side,state in self.hands.items():n.hand_states[side]=dict(state=state,received=time.monotonic())
            with self.assertRaisesRegex(RuntimeError,'时间戳'):c.read_stable_hands()

    def test_waist_limit_upgrade_preserves_exact_fk_and_requires_confirmation(self):
        import hashlib
        import xml.etree.ElementTree as ET
        from task_kinematics import configure_waist, WAIST_LIMIT
        self.assertAlmostEqual(WAIST_LIMIT,math.pi/4)
        n=self.node(); c=ScreenControl(n)
        c.config['points']['P1']=copy.deepcopy(self.record)
        path=self.directory/'old/config.json';c.save(path)
        robot=ET.parse(URDF).getroot();configure_waist(robot)
        updated=self.directory/'robot.urdf';updated.write_text(ET.tostring(robot,encoding='unicode'))
        # Match the old generated URDF serialization, as runtime export does.
        old=copy.deepcopy(robot);limit=old.find("joint[@name='waist_yaw_joint']/limit")
        limit.set('lower',str(-math.radians(35.)));limit.set('upper',str(math.radians(35.)))
        c.config['identity']['urdf_sha256']=hashlib.sha256(ET.tostring(old,encoding='unicode').encode()).hexdigest()
        data.save_bundle(path,c.config,None)
        n.args.robot_urdf=str(updated);n.model=OfflineModel(updated)
        with self.assertRaisesRegex(RuntimeError,'不匹配'):c.load(path)
        c.load(path,allow_limit_upgrade=True)
        self.assertFalse(c.aligned);self.assertIsNone(n.session.plan)
        self.assertEqual(c.config['points']['P1'],self.record)
        data.guarded_joints(n.model,[math.radians(35.468)],('waist_yaw_joint',),'P1')
        with self.assertRaisesRegex(ValueError,'waist_yaw_joint'):
            data.guarded_joints(n.model,[math.radians(45.)],('waist_yaw_joint',),'P1')
        changed=json.loads(path.read_text());changed['identity']['calibration']['values']['right_x_mm']+=1.
        data.write_json(path,changed)
        with self.assertRaisesRegex(RuntimeError,'不匹配'):c.load(path,allow_limit_upgrade=True)

    def test_screen_session_clears_old_vision_and_rejects_move_away_and_back(self):
        n=self.node(); s=n.session; s.snapshot={'old':True};s.snapshot_at=-1000
        ticket=s.begin('screen','right',.15,.15)
        self.assertIsNone(s.snapshot)
        moved=dict(self.q);moved['right_elbow_joint']+=.1;s.update(moved);s.update(self.q)
        with self.assertRaisesRegex(RuntimeError, 'moved'):
            s.ready(ticket, dict(kind='screen', segments=[1], start=self.q))

    def test_panel_offscreen_no_ros_and_screenshot(self):
        os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
        from PyQt5 import QtWidgets
        from real_task_app import Window
        import test_task_console
        fixture=test_task_console.ConsoleTests('test_planning_is_disabled_and_explained')
        fixture.setUpClass(); fixture.setUp()
        self.addCleanup(fixture.tearDown); self.addCleanup(fixture.doCleanups)
        fixture.node.screen=ScreenControl(self.node())
        window=Window(fixture.node); window.timer.stop()
        self.addCleanup(window.deleteLater)
        window.tabs.setCurrentWidget(window.screen_panel)
        window.screen_panel.refresh(False)
        # Real feedback arrives after the first render, while config revision is unchanged.
        fixture.node.session.current['waist_yaw_joint']=.1
        fixture.node.session.received=fixture.node.session.clock()
        revision=window.screen_panel.revision
        for _ in range(3):
            window.refresh()
            self.assertEqual(window.screen_panel.revision,revision)
            self.assertIn('5.73',window.screen_panel.waist_label.text())
        window.show();QtWidgets.QApplication.processEvents()
        for width,height in ((720,860),(620,720),(1100,1000)):
            window.resize(width,height); QtWidgets.QApplication.processEvents()
            if os.environ.get('SCREEN_TEST_ARTIFACTS'):
                directory=Path(os.environ['SCREEN_TEST_ARTIFACTS']);directory.mkdir(parents=True,exist_ok=True)
                self.assertTrue(window.grab().save(str(directory/f'panel_{width}x{height}.png')))
                scroll=window.screen_panel.findChild(QtWidgets.QScrollArea)
                scroll.verticalScrollBar().setValue(scroll.verticalScrollBar().maximum())
                QtWidgets.QApplication.processEvents()
                self.assertEqual(scroll.horizontalScrollBar().maximum(),0)
                self.assertTrue(window.grab().save(str(directory/f'panel_bottom_{width}x{height}.png')))
                scroll.verticalScrollBar().setValue(0)
        self.assertEqual(window.screen_panel.table.rowCount(), 6)
        fixture.node.screen.node.display_pub.assert_not_called()


class RailTests(unittest.TestCase):
    """Fixed-rail additions share the existing real-FK/offline fixtures."""
    @classmethod
    def setUpClass(cls):
        ScreenTests.setUpClass()

    def setUp(self):
        from task_rail_control import RailControl
        self.fixture = ScreenTests()
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        self.n = self.fixture.node()
        self.q = self.fixture.q
        self.n.prepared = {'right':{'joints':{k:self.q[k] for k in data.MOVING}}}
        self.c = RailControl(self.n)
        self.c.cache_dir = self.n.run_dir/'rail_replay_cache'
        self.hand_persist = patch.object(self.c, 'persist_hand')
        self.hand_persist.start(); self.addCleanup(self.hand_persist.stop)
        self.n.rail = self.c

    def record(self, q):
        return dict(joints=dict(q), hand=copy.deepcopy(self.fixture.hands['right']),
            pelvis_from_wrist=self.n.model.transform('right_wrist_yaw_link', q).tolist(),
            captured_at=data.now(), source_stamp_ns=100, stop_epoch=3)

    def test_recorded_rail_and_dring_do_not_use_instant_handoff_velocity_gate(self):
        from task_dring_demo_control import DringDemoControl
        from task_landing_gear_control import LandingGearControl
        from task_console_handoff import supported_gateway
        demo=S(node=self.n,workflow='dring_demo')
        landing=LandingGearControl.__new__(LandingGearControl);landing.node=self.n
        guards=(self.c.gateway_guard,self.c.arm_gateway_guard,
                lambda:DringDemoControl.gateway_guard(demo),landing.gateway_guard)
        self.n.gateway['max_measured_velocity_rad_s']='0.08'
        for guard in guards:guard()
        with self.assertRaisesRegex(RuntimeError,'stationary'):supported_gateway(self.n.gateway)
        with self.assertRaisesRegex(RuntimeError,'stationary'):ScreenControl.gateway_guard(self.c)
        with self.assertRaisesRegex(RuntimeError,'stationary'):
            DringDemoControl.gateway_guard(S(node=self.n,workflow='landing_gear'))
        for field,value in (('task_stop_active','true'),('phase','RUNNING'),
                            ('torque_overload_active','true'),('velocity_recovery_active','true'),
                            ('max_measured_velocity_rad_s','nan')):
            with patch.dict(self.n.gateway,{field:value}):
                for guard in guards:
                    with self.assertRaises(RuntimeError):guard()

    def configured(self):
        # Real local IK generates a small straight translational rail fixture.
        tcp = np.eye(4)
        point = data.endpoint(self.n.model, self.q, self.fixture.hands, tcp, 100, 3)
        _, states = normal_states(self.n.model, point, np.array([1., 0., 0.]), .012, 0., lambda:None)
        self.c.config['points'] = {'A':self.record(self.q), 'B':self.record(states[-1])}
        self.c.config['routes'] = {'A':[self.record(self.q)], 'B':[self.record(self.q)]}
        self.c.config['hands'] = {name:copy.deepcopy(self.fixture.hands['right']) for name in ('open','grip','pull_grip')}
        self.c.config['hands']['open']['feedback_q'] = [1.]*6
        return self.c.config

    def trace(self, stage, start, end, count=81):
        import task_rail_trace as tr
        alpha = np.linspace(0.,1.,count)
        rows = [[float(start[n]+t*(end[n]-start[n])) for n in data.MOVING] for t in alpha]
        evidence = [copy.deepcopy(self.c.config['hands'][tr.STAGES[stage][1]]) for _ in range(2)]
        evidence[0]['timestamp_ms'] = 1000; evidence[1]['timestamp_ms'] = 1100
        record = dict(schema=tr.TRACE_SCHEMA,stage=stage,hand=tr.STAGES[stage][1],
            hand_sha256=tr.hand_identity(self.c.config['hands']),hand_evidence=evidence,captured_at=data.now(),
            raw=dict(positions=rows,time_s=[i*.02 for i in range(count)],
                     stamp_ns=[1000000000+i*20000000 for i in range(count)]))
        _,_,_,record['summary'] = tr.analyze(self.n.model,record)
        return record

    def traced(self):
        self.configured()
        a = dict(self.q); a['right_elbow_joint'] += .012
        b = dict(a); b['right_elbow_joint'] += .025
        self.c.config['traces'] = {stage:self.trace(stage,start,end) for stage,start,end in (
            ('approach_A',self.q,a),('push',a,b),('retreat_B',b,self.q),
            ('approach_B',self.q,b),('pull',b,a),('retreat_A',a,self.q))}
        self.c.config['trace_ready'] = self.c.ready()
        return self.c.config

    def test_keypoint_curve_replay_preserves_analytical_path(self):
        from task_trace_edit import curve
        import task_rail_trace as traces
        from task_planning_config import PlanningParameters
        config=self.traced();source=config['traces']['push']
        edited=curve(source,[0.,.8,1.6])
        start=dict(self.q,**dict(zip(data.MOVING,edited['raw']['positions'][0])))
        with patch('task_rail_trace.make_interp_spline',side_effect=AssertionError('do not refit')):
            trajectory,report=traces.replay(self.n.model,edited,start,PlanningParameters())
        self.assertEqual(report['interpolation'],'analytical_keypoint_curve')
        self.assertFalse(report['start_join'])
        np.testing.assert_allclose(trajectory.points[0].positions,edited['raw']['positions'][0],atol=1e-12)
        np.testing.assert_allclose(trajectory.points[-1].positions,edited['raw']['positions'][-1],atol=1e-12)

    def test_manual_local_speed_uses_timing_preserving_rail_branch(self):
        from task_trace_edit import curve,local_speed
        import task_rail_trace as traces
        source=self.traced()['traces']['push']
        edited=local_speed(curve(source,[0.,.8,1.6]),.2,1.4,.8,.2)
        start=dict(self.q,**dict(zip(traces.MOVING,edited['raw']['positions'][0])))
        trajectory,report=traces.replay(self.n.model,edited,start,PlanningParameters())
        self.assertEqual(report['interpolation'],'manual_local_timing')
        self.assertEqual(report['timing']['timing_policy'],'uniform')
        self.assertLessEqual(report['max_fit_mm'],.5)
        np.testing.assert_allclose(trajectory.points[-1].positions,edited['raw']['positions'][-1],atol=1e-12)

    def test_rail_startup_restores_latest_without_commands_or_writes(self):
        from task_rail_control import RailControl
        config = copy.deepcopy(self.traced())
        root = self.n.run_dir/'restore_sessions'
        old = root/'old/rail_autosave/config.json'
        latest = root/'latest/rail_autosave/config.json'
        data.write_json(old, self.c.empty())
        data.write_json(latest, config)
        os.utime(old, (10,10)); os.utime(latest, (20,20))
        original = latest.read_bytes()
        self.n.run_dir = root/'current'
        restored = RailControl(self.n)
        restored.restore_latest()
        self.assertEqual(restored.config, config)
        self.assertEqual(restored.path, str(latest))
        self.assertEqual(restored.state, 'unknown')
        self.assertIsNone(restored.state_epoch)
        self.assertIsNone(self.n.session.plan)
        self.n.display_pub.assert_not_called()
        self.assertFalse(self.n.run_dir.exists())
        self.assertEqual(latest.read_bytes(), original)
        # An explicit clear in the latest save must not resurrect older traces.
        data.write_json(latest, self.c.empty())
        cleared = RailControl(self.n); cleared.restore_latest()
        self.assertFalse(cleared.config['traces'])

    def test_rail_startup_reports_invalid_latest_without_fallback(self):
        from task_rail_control import RailControl
        root = self.n.run_dir/'restore_sessions'
        self.n.run_dir = root/'current'
        restored = RailControl(self.n); restored.restore_latest()
        self.assertIsNone(restored.path)
        old = root/'old/rail_autosave/config.json'
        latest = root/'latest/rail_autosave/config.json'
        data.write_json(old, self.traced())
        bad = self.c.empty(); bad['identity'] = {}
        data.write_json(latest, bad)
        os.utime(old, (10,10)); os.utime(latest, (20,20))
        restored.restore_latest()
        self.assertIsNone(restored.path)
        self.assertFalse(restored.config['traces'])
        self.assertIn('自动加载滑轨示教失败', restored.trace_status)
        self.assertIn(str(latest), restored.trace_status)

    def test_rail_version_selection_restores_without_rewriting_bundle(self):
        from task_rail_control import RailControl
        from task_trace_versions import append,bank
        from task_trace_selection import journal
        config=copy.deepcopy(self.traced());stage=next(iter(config['traces']))
        config=append(config,stage,config['traces'][stage])
        self.c.persist_versions(config)
        path=self.n.run_dir/'rail_autosave/config.json';before=path.read_bytes()
        self.n.launch_work=lambda work:work()
        self.c.edit_version(stage,'select',index=0)
        self.assertEqual(path.read_bytes(),before)
        self.assertTrue(journal(path).exists())
        restored=RailControl(self.n);restored.restore_latest()
        self.assertEqual(bank(restored.config,stage)['selected'],0)
        self.assertEqual(restored.config,self.c.config)
        self.n.display_pub.assert_not_called()

    def test_recorded_rail_uses_dring_retimer_without_five_mm_cap(self):
        import task_rail_trace as tr
        from task_trajectory_speed import scale_arm_segments
        self.configured();end=dict(self.q);end['right_elbow_joint']+=.1
        trace=self.trace('push',self.q,end)
        with patch('task_trajectory_speed.scale_arm_segments',wraps=scale_arm_segments) as retime:
            trajectory,report=tr.replay(self.n.model,trace,self.q,PlanningParameters())
        self.assertEqual(retime.call_args.kwargs['policy'],'recorded')
        self.assertEqual(retime.call_args.kwargs['velocity'],.7)
        self.assertEqual(retime.call_args.kwargs['acceleration'],.7)
        self.assertAlmostEqual(report['timing']['duration_before_s'],trace['raw']['time_s'][-1])
        self.assertAlmostEqual(report['replay_seconds'],trace['raw']['time_s'][-1],places=5)
        self.assertTrue(report['timing']['limits_enforced'])

    def test_rail_spline_deviation_falls_back_without_relaxing_fit_bounds(self):
        import task_rail_trace as tr
        config=self.traced();trace=config['traces']['push']
        start=dict(self.q,**tr.recorded_position(config,'push',0))
        spline=tr.make_interp_spline
        def overshoot(x,y,*args,**kwargs):
            y=np.array(y,copy=True)
            y[:,5]+=.04*np.sin(np.linspace(0.,np.pi,len(y)))
            return spline(x,y,*args,**kwargs)
        from task_trajectory_speed import scale_arm_segments
        timing_counts=[]
        def timing(segments,*args,**kwargs):
            timing_counts.append(len(segments[0]['trajectory'].points))
            return scale_arm_segments(segments,*args,**kwargs)
        with patch.object(tr,'make_interp_spline',side_effect=overshoot),patch('task_trajectory_speed.scale_arm_segments',side_effect=timing):
            trajectory,report=tr.replay(self.n.model,trace,start,PlanningParameters())
        self.assertEqual(timing_counts,[report['replay_count']])
        self.assertEqual(report['interpolation'],'quintic_measured_waypoint_fallback')
        self.assertLessEqual(report['max_fit_mm'],.5)
        self.assertLessEqual(report['max_fit_deg'],.5)
        self.assertTrue(np.allclose(trajectory.points[0].positions,trace['raw']['positions'][0]))
        self.assertTrue(np.allclose(trajectory.points[-1].positions,trace['raw']['positions'][-1]))
        waypoint=tr.joint_waypoint_trajectory
        def bad_fallback(names,q):
            q=np.array(q,copy=True);q[:,5]+=.04
            return waypoint(names,q)
        with patch.object(tr,'make_interp_spline',side_effect=overshoot),patch.object(tr,'joint_waypoint_trajectory',side_effect=bad_fallback):
            with self.assertRaisesRegex(ValueError,'回退仍超过'):
                tr.replay(self.n.model,trace,start,PlanningParameters())

    def test_rail_planning_worker_cancel_and_timeout_reap_process(self):
        import task_rail_planning as worker
        config=self.traced()
        for cancel in (True,False):
            processes=[];checks=[0]
            popen=worker.subprocess.Popen
            def launch(*args,**kwargs):
                process=popen(*args,**kwargs);processes.append(process);return process
            def check():
                checks[0]+=1
                if cancel and checks[0]>=2:raise RuntimeError('cancelled test')
            with patch.object(worker.subprocess,'Popen',side_effect=launch):
                with self.assertRaisesRegex(RuntimeError if cancel else TimeoutError,'cancelled test' if cancel else '已取消计算'):
                    worker.build_isolated(URDF,config,self.c.ready(),self.q,'push','cycle',
                        PlanningParameters(),self.n.run_dir/str(cancel),check,lambda message:None,timeout=0.)
            self.assertEqual(len(processes),1)
            self.assertIsNotNone(processes[0].poll())

    def test_rail_multiplier_and_fastest_preserve_endpoints_and_limits(self):
        from dataclasses import replace
        import task_rail_trace as tr
        from task_rail_cache import key
        config=self.traced();trace=config['traces']['push']
        start=dict(self.q,**tr.recorded_position(config,'push',0))
        base=PlanningParameters();durations=[];identities=[]
        for params in (base,replace(base,rail_replay_multiplier=2.),replace(base,rail_replay_fastest=True)):
            trajectory,report=tr.replay(self.n.model,trace,start,params)
            durations.append(report['replay_seconds']);identities.append(key(trace,self.n.model,params))
            self.assertTrue(report['timing']['limits_enforced'])
            self.assertTrue(np.allclose(trajectory.points[0].positions,trace['raw']['positions'][0],atol=1e-9))
            self.assertTrue(np.allclose(trajectory.points[-1].positions,trace['raw']['positions'][-1],atol=1e-9))
            vmax,amax=tr.task_motion_limits(data.MOVING,.7,.7,params)
            list(tr.samples(trajectory,vmax,amax))
            tr.validate_jerk_limits(trajectory,tr.task_jerk_limits(data.MOVING))
        self.assertLess(durations[1],durations[0])
        self.assertLessEqual(durations[2],durations[1]+1e-6)
        self.assertEqual(len(set(identities)),3)

    def test_rail_idle_trim_retains_raw_data_and_ab_endpoints(self):
        from dataclasses import replace
        import task_rail_trace as tr
        config=self.traced();trace=config['traces']['push'];raw=trace['raw']
        raw['positions']=[list(raw['positions'][0]) for _ in range(30)]+raw['positions']+[
            list(raw['positions'][-1]) for _ in range(40)]
        raw['time_s']=[i*.02 for i in range(len(raw['positions']))]
        raw['stamp_ns']=[1000000000+i*20000000 for i in range(len(raw['positions']))]
        original=copy.deepcopy(trace)
        start=dict(self.q,**tr.recorded_position(config,'push',0))
        trajectory,report=tr.replay(self.n.model,trace,start,replace(PlanningParameters(),rail_trim_idle_ends=True))
        self.assertEqual(trace,original)
        self.assertGreaterEqual(report['idle_trim']['leading_seconds'],.6)
        self.assertGreaterEqual(report['idle_trim']['trailing_seconds'],.8)
        np.testing.assert_allclose(trajectory.points[0].positions,raw['positions'][0],atol=1e-12,rtol=0.)
        np.testing.assert_allclose(trajectory.points[-1].positions,raw['positions'][-1],atol=1e-12,rtol=0.)
        self.assertLessEqual(report['max_fit_mm'],.5)

    def test_rail_continuous_smoothing_avoids_waypoint_stops_within_fit_bounds(self):
        from dataclasses import replace
        import task_rail_trace as tr
        config=self.traced();trace=config['traces']['push']
        start=dict(self.q,**tr.recorded_position(config,'push',0));spline=tr.make_interp_spline
        def overshoot(x,y,*args,**kwargs):
            y=np.array(y,copy=True);y[:,5]+=.04*np.sin(np.linspace(0.,np.pi,len(y)))
            return spline(x,y,*args,**kwargs)
        with patch.object(tr,'make_interp_spline',side_effect=overshoot),patch.object(
                tr,'joint_waypoint_trajectory',side_effect=AssertionError('Unnecessary stop-to-stop fallback')):
            trajectory,report=tr.replay(self.n.model,trace,start,
                replace(PlanningParameters(),rail_replay_smooth=True,rail_replay_multiplier=2.))
        self.assertEqual(report['interpolation'],'bounded_continuous_joint_spline')
        self.assertLessEqual(report['max_fit_mm'],.5)
        self.assertLessEqual(report['max_fit_deg'],.5)

    def test_three_stage_cycles_switch_hands_at_ready_and_pull_start(self):
        import task_rail_trace as tr
        config=self.traced()
        config['hands']['pull_grip']['feedback_q']=[.3]*6
        for stage in config['traces'].values():stage['hand_policy']='not_recorded'
        start=dict(self.q,**model_joints('right',[.6]*6))
        start['right_elbow_joint']+=.08
        for operation in ('push','pull'):
            plan=tr.build_replay(self.n.model,config,self.c.ready(),start,operation,'cycle',PlanningParameters())
            segments=plan['segments']
            self.assertEqual([s['trace_stage'] for s in segments if 'trace_stage' in s],tr.required_stages(operation,'cycle'))
            self.assertEqual(segments[0]['label'],'移动到右手 above-table')
            hands=[s for s in segments if s['kind']=='hand']
            self.assertEqual([s['posture'] for s in hands],['grip','open'] if operation=='push' else ['open','pull_grip','open'])
            self.assertTrue(all(s['duration']==1. for s in hands))
            self.assertFalse(any('delayed_hand' in s for s in segments))
            for name in data.MOVING:
                self.assertAlmostEqual(hands[0]['start'][name],self.c.ready()[name])
                if operation=='push':self.assertAlmostEqual(hands[-1]['start'][name],self.c.ready()[name])
            if operation=='pull':
                pull=next(s for s in segments if s.get('trace_stage')=='pull')
                for name,value in tr.recorded_position(config,'approach_B',-1).items():
                    self.assertAlmostEqual(hands[1]['start'][name],value)
                grasped=next(i for i,s in enumerate(segments) if s is hands[1])
                self.assertEqual(segments[grasped-1].get('trace_stage'),'approach_B')
                for name,value in model_joints('right',[.3]*6).items():
                    self.assertAlmostEqual(pull['start'][name],value)
                    self.assertAlmostEqual(pull['end'][name],value)
                for name,value in tr.recorded_position(config,'pull',-1).items():
                    self.assertAlmostEqual(hands[-1]['start'][name],value)
                opened=next(i for i,s in enumerate(segments) if s is hands[-1])
                retreat=next(i for i,s in enumerate(segments) if s.get('trace_stage')=='retreat_A')
                self.assertLess(opened,retreat)
            for a,b in zip(segments,segments[1:]):self.assertEqual(a['end'],b['start'])

    def test_rail_cache_reuses_curves_and_rejects_changes_or_corruption(self):
        import task_rail_trace as tr
        from task_rail_cache import cycle_key,record_success,require_success
        config=self.traced();params=PlanningParameters();root=self.c.cache_dir
        start=dict(self.q,**model_joints('right',config['hands']['open']['feedback_q']))
        options=dict(cache_dir=root)
        tr.build_replay(self.n.model,config,self.c.ready(),start,'push','cycle',params,**options)
        identity=cycle_key(config,self.n.model,params,self.c.ready(),'push','cycle')
        with self.assertRaisesRegex(RuntimeError,'成功规划'):require_success(root,identity)
        record_success(root,identity);require_success(root,identity)
        start['right_elbow_joint']+=.08
        with patch.object(tr,'replay',side_effect=AssertionError('Cached replay was recomputed')):
            plan=tr.build_replay(self.n.model,config,self.c.ready(),start,'push','cycle',params,require_cache=True,**options)
        recorded=[s for s in plan['segments'] if 'trace_stage' in s]
        self.assertTrue(all(s['trace_report']['replay_cache']['hit'] for s in recorded))
        untouched=copy.deepcopy(config)
        untouched['traces']['pull']['captured_at']='unrelated'
        self.assertEqual(identity,cycle_key(untouched,self.n.model,params,self.c.ready(),'push','cycle'))
        changed=copy.deepcopy(config);changed['traces']['push']['raw']['positions'][-1][0]+=.001
        self.assertNotEqual(identity,cycle_key(changed,self.n.model,params,self.c.ready(),'push','cycle'))
        from dataclasses import replace
        self.assertNotEqual(identity,cycle_key(config,self.n.model,replace(params,arm_velocity_rad_s=.2),self.c.ready(),'push','cycle'))
        data.write_json(recorded[0]['trace_report']['replay_cache']['path'],{})
        with patch.object(tr,'replay',side_effect=AssertionError('Must not silently recompute')):
            with self.assertRaisesRegex(RuntimeError,'缓存'):
                tr.build_replay(self.n.model,config,self.c.ready(),start,'push','cycle',params,require_cache=True,**options)

    def test_trace_sampling_joint_replay_and_both_mode_sequences(self):
        import task_rail_trace as tr
        config = self.traced()
        self.assertEqual(set(config['traces']),set(tr.STAGES))
        for record in config['traces'].values():
            tr.validate_trace(record,record['stage'],self.n.model,config['hands'])
            self.assertLess(record['summary']['replay_count'],record['summary']['raw_count'])
        start = dict(self.q,**model_joints('right',config['hands']['open']['feedback_q']))
        for mode in ('hold','release'):
            push = tr.build_replay(self.n.model,config,self.c.ready(),start,'push',mode,PlanningParameters())
            pull = tr.build_replay(self.n.model,config,self.c.ready(),push['segments'][-1]['end'],
                                  'pull',mode,PlanningParameters())
            self.assertEqual(push['segments'][-1].get('trace_stage'),'push' if mode == 'hold' else 'retreat_B')
            self.assertEqual(pull['rail']['final_state'],'outer')
            self.assertEqual([s['trace_stage'] for s in pull['segments'] if 'trace_stage' in s],
                             ['pull','retreat_A'] if mode == 'hold' else ['approach_B','pull','retreat_A'])
            for plan in (push,pull):
                self.assertEqual(plan['rail']['path_source'],'recorded')
                for a,b in zip(plan['segments'],plan['segments'][1:]):self.assertEqual(a['end'],b['start'])
                for segment in plan['segments']:
                    if segment['kind'] == 'hand':continue
                    self.assertTrue(np.allclose(segment['trajectory'].points[0].positions,
                                                [segment['start'][n] for n in data.MOVING]))
                    self.assertLessEqual(segment['trace_report']['max_fit_mm'],.5)
                    self.assertGreater(segment['duration'],0.)

    def test_trace_rejects_feedback_gaps_large_steps_tamper_and_hand_changes(self):
        import task_rail_trace as tr
        self.traced(); record = self.c.config['traces']['push']
        bad = copy.deepcopy(record); bad['raw']['time_s'][10] += .2
        with self.assertRaisesRegex(ValueError,'gap'):tr.analyze(self.n.model,bad)
        bad = copy.deepcopy(record); bad['raw']['stamp_ns'][10] = bad['raw']['stamp_ns'][9]
        with self.assertRaisesRegex(ValueError,'gap'):tr.analyze(self.n.model,bad)
        bad = copy.deepcopy(record); bad['raw']['positions'][10][0] = float('nan')
        with self.assertRaises(ValueError):tr.analyze(self.n.model,bad)
        far = dict(self.q); far['right_elbow_joint'] += .08
        with self.assertRaisesRegex(ValueError,'2 mm'):
            self.trace('push',self.q,far,count=7)
        self.c.config['hands']['grip']['feedback_q'][0] += .05
        with self.assertRaisesRegex(ValueError,'手型'):
            tr.validate_trace(record,'push',self.n.model,self.c.config['hands'])
        # Stale traces remain exportable so an operator never loses their hand captures.
        path = self.n.run_dir/'changed_hand_trace.json'
        self.c.save(path); self.c.load(path)
        self.assertIn('push',self.c.config['traces'])

    def test_edited_rail_samples_are_not_treated_as_feedback_jumps(self):
        import task_rail_trace as tr
        from task_trajectory import time_seconds
        from task_trace_edit import synthetic
        self.traced();source=self.c.config['traces']['push']
        rows=np.tile([self.q[n] for n in data.MOVING],(7,1))
        rows[:,list(data.MOVING).index('right_elbow_joint')]+=np.linspace(0,.08,7)
        candidate=synthetic(source,rows,np.linspace(0,.12,7),'edited test')
        _,_,_,report=tr.analyze(self.n.model,candidate)
        self.assertGreater(report['max_raw_step_mm'],2.)
        self.assertFalse(report['measured_step_guard_applied'])
        measured=copy.deepcopy(candidate);measured['raw'].pop('sample_policy')
        with self.assertRaisesRegex(ValueError,'2 mm'):tr.analyze(self.n.model,measured)
        undeclared=copy.deepcopy(candidate);undeclared.pop('optimization')
        with self.assertRaisesRegex(ValueError,'2 mm'):tr.analyze(self.n.model,undeclared)
        broken=copy.deepcopy(candidate);broken['raw']['positions'][3][0]+=1.
        with self.assertRaises(ValueError):tr.analyze(self.n.model,broken)
        curve,result=tr.replay(self.n.model,candidate,self.q,PlanningParameters())
        self.assertEqual(result['input_geometry'],'offline_edited')
        self.assertLessEqual(result['max_fit_mm'],.5)
        self.assertGreater(time_seconds(curve.points[-1]),0.)

    def test_rail_plan_uses_selected_edit_and_records_each_version(self):
        import task_rail_trace as tr
        from task_trace_versions import append
        config=self.traced();original=copy.deepcopy(config['traces']['push'])
        modified=copy.deepcopy(original)
        for row in modified['raw']['positions']:row[0]+=.0001
        config=append(config,'push',modified)
        plan=tr.build_replay(self.n.model,config,self.c.ready(),self.q,'push','cycle',PlanningParameters())
        stages=[s for s in plan['segments'] if 'trace_stage' in s]
        self.assertEqual([s['trace_stage'] for s in stages],['approach_A','push','retreat_B'])
        self.assertEqual([s['trace_version']['index'] for s in stages],[0,1,0])
        stage=stages[1]
        self.assertEqual(stage['trace_version']['sha256'],data.fingerprint(modified))
        np.testing.assert_allclose(stage['trajectory'].points[-1].positions,modified['raw']['positions'][-1],atol=1e-9)
        self.assertNotEqual(stage['trajectory'].points[-1].positions[0],original['raw']['positions'][-1][0])

    def test_trace_small_join_is_continuous_large_join_rejected(self):
        import task_rail_trace as tr
        self.traced(); record = self.c.config['traces']['approach_A']
        shifted = dict(self.q); shifted['right_elbow_joint'] += .0002
        trajectory, report = tr.replay(self.n.model,record,shifted,PlanningParameters())
        self.assertTrue(report['start_join'])
        self.assertTrue(np.allclose(trajectory.points[0].positions,[shifted[n] for n in data.MOVING]))
        shifted['right_elbow_joint'] += tr.BOUNDARY_JOINT+.001
        with self.assertRaisesRegex(ValueError,'轨迹端点偏差'):
            tr.replay(self.n.model,record,shifted,PlanningParameters())

    def test_trace_rotation_and_joint_only_motion_are_retained(self):
        import task_rail_trace as tr
        self.configured()
        end = dict(self.q); end['right_wrist_yaw_joint'] += .07
        record = self.trace('push',self.q,end)
        self.assertGreater(record['summary']['replay_count'],3)
        q,_,indices,_ = tr.analyze(self.n.model,record)
        self.assertEqual(indices[0],0); self.assertEqual(indices[-1],len(q)-1)
        trajectory,report = tr.replay(self.n.model,record,self.q,PlanningParameters())
        self.assertLessEqual(report['max_fit_mm'],.5)
        self.assertAlmostEqual(trajectory.points[-1].positions[-2],end['right_wrist_yaw_joint'])

    def test_trace_requires_only_selected_operation_and_never_needs_old_endpoints(self):
        import task_rail_trace as tr
        config = self.traced()
        config['points'] = {}; config['routes'] = {'A':[],'B':[]}
        start = dict(self.q,**model_joints('right',config['hands']['open']['feedback_q']))
        plan = tr.build_replay(self.n.model,config,self.c.ready(),start,'push','hold',PlanningParameters())
        self.assertEqual(plan['rail']['final_state'],'inner_held')
        del config['traces']['pull']
        tr.build_replay(self.n.model,config,self.c.ready(),start,'push','hold',PlanningParameters())
        with self.assertRaisesRegex(ValueError,'本次操作'):
            tr.build_replay(self.n.model,config,self.c.ready(),start,'pull','hold',PlanningParameters())

    def test_trace_callback_uses_only_accepted_measured_feedback(self):
        from real_task_app import TaskNode
        from sensor_msgs.msg import JointState
        import task_rail_trace as tr
        n,c = self.n,self.c
        c.recording = dict(phase='recording',initial=dict(self.q),buffer=tr.TraceBuffer(n.model))
        n.session.phase = 'RAIL_RECORDING'
        n.get_clock = lambda:S(now=lambda:S(nanoseconds=1000000000))
        msg = JointState(name=list(self.q),position=list(self.q.values()))
        msg.header.stamp.sec = 1
        with patch.object(n.model,'transform',side_effect=AssertionError('FK must not block feedback')):
            TaskNode.joints(n,msg)
            self.assertEqual(len(c.recording['buffer'].rows),1)
            stamp = n.session.source_stamp_ns
            fingers = model_joints('right',[0.]*6)
            hand_msg = JointState(name=list(fingers),position=list(fingers.values()))
            hand_msg.header.stamp.sec = 2
            TaskNode.joints(n,hand_msg)
            self.assertIsNone(c.recording['buffer'].error)
            self.assertEqual(len(c.recording['buffer'].rows),1)
            self.assertEqual(n.session.source_stamp_ns,stamp)
            partial = {key:value for key,value in self.q.items() if key != 'right_elbow_joint'}
            incomplete = JointState(name=list(partial),position=list(partial.values()))
            TaskNode.joints(n,incomplete)
            self.assertIn('缺少关节',c.recording['buffer'].error)
            c.recording['buffer'].error = None
            TaskNode.joints(n,msg)
        self.assertIn('拒绝',c.recording['buffer'].error)
        self.assertEqual(len(c.recording['buffer'].rows),1)
        c.recording = None

    def drive_record(self, outcome='ok', *, stage='approach_A', without_setup=False):
        import task_rail_trace as tr
        self.configured()
        n,c = self.n,self.c
        if without_setup:
            c.config['hands']={}; n.prepared={}; n.hand_control=False
        n.gateway.update(phase='TASK_ADMITTANCE',task_stop_active='true')
        n.session.stopped = True
        clock = [time.monotonic()]; start_at = [None]; injected = [False]
        physical = dict(self.q)
        n.session.clock = lambda:clock[0]
        def advance(seconds):
            deadline = clock[0]+seconds
            while clock[0] < deadline-1e-10:
                clock[0] = min(deadline,clock[0]+.01)
                rec = c.recording
                if rec and rec['phase'] == 'recording':
                    if start_at[0] is None:start_at[0] = clock[0]
                    elapsed = clock[0]-start_at[0]
                    physical['right_elbow_joint'] = self.q['right_elbow_joint']+min(1.,elapsed/.8)*.012
                    if elapsed > .3 and not injected[0]:
                        injected[0] = True
                        if outcome == 'gap':clock[0] += .2
                        elif outcome == 'cancel':c.cancel_trace()
                        elif outcome == 'stop':c.started(); n.session.stop()
                n.gateway_received = clock[0]
                stamp = round(clock[0]*1e9)
                n.session.update(physical,source_stamp_ns=stamp,source_now_ns=stamp)
                c.capture_feedback(physical,n.session.received,n.session.source_stamp_ns)
                if rec and rec.get('receiver') is not None:
                    rec['receiver'].feed(physical,clock[0],stamp)
                if rec and rec['phase'] == 'recording' and clock[0]-start_at[0] > 1.4:
                    c.finish_trace()
        n.hand = Mock(); n.hand.read.side_effect = AssertionError('Recording must never read hand feedback')
        n.cache_hand_state = lambda side,state:n.hand_states.update({side:dict(state=state,received=clock[0])})
        class FakeCapture:
            def __init__(self,*args,**kwargs):
                assert kwargs['full_rate'] is True
                self.buffer=tr.TraceBuffer(n.model);self.directory=n.run_dir/'fake_capture'
                self.directory.mkdir(exist_ok=True);self.finish_at=None
                self.latest={};self.feed(physical,clock[0],round(clock[0]*1e9))
            def feed(self,q,at,stamp):
                if self.finish_at is not None:return
                self.buffer.feed(q,at,stamp)
                self.latest=dict(ready=True,last_sample=dict(joints=dict(q)))
            def check(self):
                if self.buffer.error:raise RuntimeError(self.buffer.error)
                return self.latest
            def request_finish(self,at):self.finish_at=at
            def finish(self,check):
                check();return dict(self.buffer.data(),sample_policy='all_received')
            def close(self):pass
        with patch('task_rail_control.time.monotonic',lambda:clock[0]),patch('task_rail_control.time.sleep',advance), \
                patch('task_rail_control.DirectCapture',FakeCapture):
            advance(.6)
            c.start_trace(stage,confirmed=True)
        return n,c

    def test_trace_recorder_reads_only_and_preserves_existing_hands(self):
        n,c = self.drive_record()
        self.assertIsNone(c.recording)
        self.assertEqual(n.session.phase,'TASK_ADMITTANCE')
        self.assertIn('approach_A',c.config['traces'])
        self.assertEqual(n.hand.method_calls,[])
        self.assertEqual(c.config['hands']['open']['feedback_q'],[1.]*6)
        self.assertTrue((n.run_dir/'rail_autosave/config.json').is_file())

    def test_trace_recorder_cancel_stop_and_gap_never_save(self):
        for outcome in ('cancel','stop','gap'):
            with self.subTest(outcome=outcome):
                with self.assertRaises((RuntimeError,ValueError)):
                    self.drive_record(outcome)
                self.assertIsNone(self.c.recording)
                self.assertNotIn('approach_A',self.c.config.get('traces',{}))
                self.assertIn('grip',self.c.config['hands'])

    def test_all_stages_record_without_hands_ready_or_endpoint_alignment(self):
        import task_rail_trace as tr
        for stage in tr.STAGES:
            with self.subTest(stage=stage):
                n,c=self.drive_record(stage=stage,without_setup=True)
                trace=c.config['traces'][stage]
                self.assertEqual(trace['hand_policy'],'not_recorded')
                self.assertNotIn('hand_evidence',trace)
                self.assertEqual(n.hand.method_calls,[])
                tr.validate_trace(trace,stage,n.model,{})
                self.assertEqual(c.config['hands'],{})
                for key,(_,source) in tr.ENDPOINTS.items():
                    if source==stage:
                        self.assertEqual(tr.endpoint(c.config,key),dict(zip(data.MOVING,trace['raw']['positions'][-1])))

    def test_independent_endpoint_frames_and_displaced_replay_connections(self):
        import task_rail_trace as tr
        config=self.traced()
        axis=data.MOVING.index('right_shoulder_yaw_joint')
        for stage in ('approach_B','pull','retreat_A'):
            for row in config['traces'][stage]['raw']['positions']:row[axis]+=.25
        self.assertNotEqual(tr.endpoint(config,'push_A'),tr.endpoint(config,'pull_A'))
        self.assertNotEqual(tr.endpoint(config,'push_B'),tr.endpoint(config,'pull_B'))
        for key,(_,stage) in tr.ENDPOINTS.items():
            self.assertEqual(tr.endpoint(config,key),dict(zip(data.MOVING,config['traces'][stage]['raw']['positions'][-1])))
            self.assertEqual(self.c.movement_target(key,waist=0.),tr.endpoint(config,key))
        self.assertEqual(self.c.movement_target('connect:pull',waist=0.),tr.endpoint(config,'pull_B'))
        self.assertEqual(self.c.movement_target('start:pull',waist=0.),
                         dict(zip(data.MOVING,config['traces']['pull']['raw']['positions'][0])))
        start=dict(self.q,**model_joints('right',config['hands']['open']['feedback_q']))
        for mode in ('hold','release'):
            push=tr.build_replay(self.n.model,config,self.c.ready(),start,'push',mode,PlanningParameters())
            pull=tr.build_replay(self.n.model,config,self.c.ready(),push['segments'][-1]['end'],
                                 'pull',mode,PlanningParameters())
            self.assertTrue(any(s.get('role')=='rail_alignment' for s in pull['segments']))
            previous=pull['start']
            for segment in pull['segments']:
                self.assertEqual(segment['start'],previous)
                for name in data.ARMS['left']:self.assertEqual(segment['end'][name],start[name])
                previous=segment['end']
            for name in data.MOVING:self.assertAlmostEqual(previous[name],self.c.ready()[name])
        before=tr.endpoint(config,'push_B')
        config['traces']['push']['raw']['positions'][-1][axis]+=.001
        self.assertNotEqual(tr.endpoint(config,'push_B'),before)
        del config['traces']['push']
        with self.assertRaisesRegex(ValueError,'尚未录制'):tr.endpoint(config,'push_B')
        self.assertEqual(tr.endpoint(config,'pull_B'),self.c.movement_target('connect:pull',waist=0.))

    def test_new_trace_hand_changes_do_not_invalidate_recorded_path(self):
        import task_rail_trace as tr
        self.traced();trace=self.c.config['traces']['push']
        trace['hand_policy']='not_recorded'
        for key in ('hand','hand_evidence','hand_sha256'):trace.pop(key)
        tr.validate_trace(trace,'push',self.n.model,{})
        self.assertTrue(tr.trace_hand_matches(trace,'push',{}))
        self.c.config['hands']['grip']['feedback_q']=[.4]*6
        tr.validate_trace(trace,'push',self.n.model,self.c.config['hands'])

    def test_unbound_pull_requires_replay_target_but_not_rerecording(self):
        import task_rail_trace as tr
        config=self.traced();trace=config['traces']['pull'];trace['hand_policy']='not_recorded'
        del config['hands']['pull_grip']
        start=dict(self.q,**dict(zip(data.MOVING,trace['raw']['positions'][0])))
        with self.assertRaisesRegex(ValueError,'独立拉出手型'):
            tr.build_replay(self.n.model,config,self.c.ready(),start,'pull','hold',PlanningParameters())
        config['hands']['pull_grip']=copy.deepcopy(config['hands']['grip'])
        config['hands']['pull_grip']['feedback_q']=[.3]*6
        plan=tr.build_replay(self.n.model,config,self.c.ready(),start,'pull','hold',PlanningParameters())
        self.assertEqual(plan['segments'][0]['target_q'],[.3]*6)

    def test_move_authorization_rejects_missing_target_tamper_and_stop(self):
        self.traced();n,c=self.n,self.c
        n.session.clock=lambda:n.session.received
        n.hand_control=False
        with self.assertRaisesRegex(RuntimeError,'路径净空'):c.prepare_move('push_B')
        c.prepare_move('push_B',confirmed=True)
        plan=n.session.plan
        self.assertEqual(plan['rail']['target_joints'],c.movement_target('push_B',waist=0.))
        original=c.config['traces']['push']['raw']['positions'][-1][0]
        c.config['traces']['push']['raw']['positions'][-1][0]+=.001
        with self.assertRaisesRegex(RuntimeError,'目标或轨迹'):c.execution_guard(plan,preflight=True)
        c.config['traces']['push']['raw']['positions'][-1][0]=original
        n.gateway['task_stop_epoch']='4'
        with self.assertRaisesRegex(RuntimeError,'STOP'):c.execution_guard(plan,running=True)
        del c.config['traces']['push']
        with self.assertRaisesRegex(ValueError,'尚未录制'):c.movement_target('push_B',waist=0.)

    def test_move_panel_executes_only_matching_completed_generation(self):
        from PyQt5 import QtWidgets
        from task_rail_panel import RailPanel
        app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        self.app=app;self.traced();n=self.n;n.execute=Mock()
        panel=RailPanel(S(node=n,invoke=lambda fn:fn()));self.addCleanup(panel.deleteLater)
        n.session.phase='PLAN_READY'
        n.session.plan=dict(kind='rail',rail=dict(operation='move',target='push_B'))
        panel.pending_move=(n.session.generation,'push_B')
        panel.refresh(True);n.execute.assert_not_called()
        panel.refresh(False);n.execute.assert_called_once_with(scene_confirmed=True)
        panel.refresh(False);n.execute.assert_called_once()
        n.execute.reset_mock();panel.pending_move=(n.session.generation,'push_B')
        n.session.generation+=1
        panel.refresh(False);n.execute.assert_not_called()
        panel.pending_move=(n.session.generation,'push_A')
        panel.refresh(False);n.execute.assert_not_called()

    def test_cached_replay_button_executes_once_and_cancels_stale_request(self):
        from PyQt5 import QtWidgets
        from task_rail_panel import RailPanel
        self.app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        self.traced();n=self.n;n.execute=Mock()
        panel=RailPanel(S(node=n,invoke=lambda fn:fn()));self.addCleanup(panel.deleteLater)
        self.assertEqual(panel.mode.currentData(),'cycle')
        n.session.phase='PLAN_READY'
        n.session.plan=dict(kind='rail',rail=dict(operation='push',reuse=True))
        panel.pending_replay=(n.session.generation,'push')
        panel.refresh(True);n.execute.assert_not_called()
        panel.refresh(False);n.execute.assert_called_once_with(scene_confirmed=True)
        panel.refresh(False);n.execute.assert_called_once()
        n.execute.reset_mock();panel.pending_replay=(n.session.generation,'push')
        n.session.generation+=1
        panel.refresh(False);n.execute.assert_not_called()
        panel.pending_replay=(n.session.generation,'pull')
        panel.refresh(False);n.execute.assert_not_called()
        panel.pending_replay=(n.session.generation,'push');n.session.phase='REPLAN_REQUIRED'
        panel.refresh(False);n.execute.assert_not_called()

    def test_rail_panel_trim_switch_and_result_are_non_destructive(self):
        from PyQt5 import QtWidgets
        from task_rail_panel import RailPanel
        self.app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        self.traced();n=self.n
        original=copy.deepcopy(self.c.config)
        panel=RailPanel(S(node=n,invoke=lambda fn:fn()));self.addCleanup(panel.deleteLater)
        panel.trim_idle_ends.setChecked(True)
        n.session.phase='PLAN_READY'
        n.session.plan=dict(kind='rail',rail=dict(path_source='recorded'),segments=[dict(trace_stage='push',
            trace_report=dict(idle_trim=dict(leading_seconds=1.2,trailing_seconds=.8,removed_knots=4)))])
        panel.refresh(False)
        self.assertIn('2.00',panel.trim_result.text())
        self.assertIn('4',panel.trim_result.text())
        panel.pending_replay=(n.session.generation,'push')
        panel.trim_idle_ends.setChecked(False)
        self.assertIsNone(panel.pending_replay)
        self.assertIsNone(n.session.plan)
        self.assertEqual(n.session.phase,'REPLAN_REQUIRED')
        with patch.object(self.c,'plan') as plan:
            panel.plan('push')
            self.assertFalse(plan.call_args.kwargs['trim_idle_ends'])
        panel.trim_idle_ends.setChecked(True)
        with patch.object(panel,'ask',return_value=True),patch.object(self.c,'plan') as plan:
            panel.replay_cached('pull')
            self.assertTrue(plan.call_args.kwargs['trim_idle_ends'])
        panel.pending_replay=None
        panel.refresh(True)
        self.assertFalse(panel.trim_idle_ends.isEnabled())
        self.assertEqual(self.c.config,original)

    def test_trace_failed_rerecord_preserves_previous_segment(self):
        self.traced()
        previous = copy.deepcopy(self.c.config['traces']['approach_A'])
        with self.assertRaises(RuntimeError):self.drive_record('cancel')
        self.assertEqual(previous,self.c.config['traces']['approach_A'])

    def test_trace_buffer_bounds_and_recording_blocks_other_plans(self):
        import task_rail_trace as tr
        b = tr.TraceBuffer(self.n.model)
        b.feed(self.q,1.,1000000000)
        b.feed(self.q,1.2,1200000000)
        self.assertIn('中断',b.error)
        self.n.session.phase = 'RAIL_RECORDING'
        with self.assertRaisesRegex(RuntimeError,'active'):
            self.n.session.begin('prepare','right',.15,.15)

    def test_rail_hand_read_and_record_are_read_only(self):
        n = self.n; n.hand = Mock()
        n.gateway['phase'] = 'TASK_ADMITTANCE'
        n.session.stopped = True
        self.c.record(hand='grip')
        self.assertEqual(self.c.config['hands']['grip']['feedback_q'], self.fixture.hands['right']['feedback_q'])
        self.assertEqual(self.c.config['hands']['grip']['target_q'], self.fixture.hands['right']['target_q'])
        n.hand.assert_not_called()
        self.assertEqual(n.hand.method_calls, [])
        self.assertTrue((n.run_dir/'rail_autosave/config.json').is_file())
        self.c.record('A', waypoint=True)
        self.c.record('A')
        self.assertEqual(len(self.c.config['routes']['A']), 1)
        self.assertIn('A', self.c.config['points'])
        self.assertEqual(self.c.state, 'unknown')

    def test_pull_hand_record_is_independent_and_read_only(self):
        # This fixture checks recording, not elapsed feedback age.
        self.n.session.clock=lambda:self.n.session.received
        self.configured()
        old = copy.deepcopy(self.c.config['hands'])
        self.n.gateway['phase'] = 'TASK_ADMITTANCE'
        self.n.hand = Mock()
        self.fixture.hands['right']['feedback_q'] = [.3]*6
        self.c.record(hand='pull_grip')
        self.assertEqual(self.c.config['hands']['pull_grip']['feedback_q'], [.3]*6)
        for key in ('open','grip'):
            self.assertEqual(self.c.config['hands'][key],old[key])
        self.assertEqual(self.n.hand.method_calls,[])

    def test_trace_posture_fingerprint_and_legacy_compatibility(self):
        import task_rail_trace as tr
        config=self.traced();hands=config['hands']
        for stage,trace in config['traces'].items():
            trace['hand_identity_scope']='posture'
            trace['hand_sha256']=tr.hand_identity(hands,tr.STAGES[stage][1])
        hands['pull_grip']['feedback_q']=[.3]*6
        for stage,trace in config['traces'].items():
            self.assertEqual(tr.trace_hand_matches(trace,stage,hands),stage!='pull')
        legacy=copy.deepcopy(config['traces']['pull'])
        legacy.pop('hand_identity_scope');legacy['hand']='grip'
        legacy['hand_sha256']=tr.hand_identity(hands)
        self.assertFalse(tr.trace_hand_matches(legacy,'pull',hands))
        del hands['pull_grip']
        tr.validate_trace(legacy,'pull',self.n.model,hands)

    def test_legacy_two_hand_bundle_keeps_original_hold_sequence(self):
        import task_rail_trace as tr
        config=self.traced()
        del config['hands']['pull_grip']
        config['traces']['pull']['hand']='grip'
        start=dict(self.q,**dict(zip(data.MOVING,config['traces']['pull']['raw']['positions'][0])))
        plan=tr.build_replay(self.n.model,config,self.c.ready(),start,'pull','hold',PlanningParameters())
        self.assertEqual(plan['segments'][0]['trace_stage'],'pull')
        self.assertFalse(any(s.get('posture')=='pull_grip' for s in plan['segments']))

    def test_persist_pull_hand_keeps_other_default_slots(self):
        from task_rail_control import RailControl
        self.configured();saved=copy.deepcopy(self.c.config['hands'])
        state=copy.deepcopy(saved['grip']);state['feedback_q']=[.3]*6
        with patch.object(self.c,'empty',return_value=dict(hands=saved)), \
                patch('task_rail_control.write_json') as write:
            RailControl.persist_hand(self.c,'pull_grip',state)
        payload=write.call_args.args[1]
        self.assertEqual(payload['hands']['pull_grip'],state)
        for key in ('open','grip'):
            self.assertEqual(payload['hands'][key],self.c.config['hands'][key])

    def test_finish_direct_capture_sends_cutoff_before_saving(self):
        receiver=Mock()
        self.c.recording=dict(phase='recording',source='direct_lowstate',receiver=receiver,
                              finish=threading.Event())
        with patch('task_rail_control.time.monotonic',return_value=12.5):
            self.c.finish_trace()
        receiver.request_finish.assert_called_once_with(12.5)
        self.assertTrue(self.c.recording['finish'].is_set())
        self.assertEqual(self.c.recording['phase'],'saving')
        with self.assertRaises(RuntimeError):self.c.finish_trace()

    def test_running_rail_uses_controller_watchdog_but_keeps_stop_guards(self):
        proof=dict(revision=self.c.revision,stop_epoch=3,waist_control_mode='guarded',waist_control_revision='0')
        self.n.gateway_received=0.
        self.n.gateway['phase']='RUNNING'
        with self.assertRaisesRegex(RuntimeError,'状态过期'):
            self.c.execution_guard(dict(rail=proof))
        with patch.object(self.n.session,'measured',side_effect=AssertionError('UI watchdog')):
            self.c.execution_guard(dict(rail=proof),running=True)
            self.n.gateway['task_stop_epoch']='4'
            with self.assertRaisesRegex(RuntimeError,'STOP'):
                self.c.execution_guard(dict(rail=proof),running=True)

    def test_full_rate_arrays_accept_more_than_legacy_capacity(self):
        import task_rail_trace as tr
        count=30001
        raw=dict(positions=[[self.q[n] for n in data.MOVING]]*count,
                 time_s=[i*.001 for i in range(count)],
                 stamp_ns=[10**9+i*10**6 for i in range(count)],sample_policy='all_received')
        self.assertEqual(len(tr.arrays(raw,self.n.model)[0]),count)
        raw.pop('sample_policy')
        with self.assertRaises(ValueError):tr.arrays(raw,self.n.model)

    def test_rail_record_rejects_hold_and_stop_race(self):
        with self.assertRaisesRegex(RuntimeError, 'TASK_ADMITTANCE'):
            self.c.record(hand='open')
        self.n.gateway['phase'] = 'TASK_ADMITTANCE'
        read = self.n.read_hands
        def race(ticket=None):
            value = read(ticket)
            self.n.session.invalidate()
            return value
        self.n.read_hands = race
        with self.assertRaisesRegex(RuntimeError, 'STOP/session'):
            self.c.record('A')
        self.assertEqual(self.c.config['points'], {})

    def test_rail_bundle_round_trip_and_fk_tamper(self):
        # Serialization latency is not simulated feedback age in this fixture.
        self.n.session.clock=lambda:self.n.session.received
        self.configured()
        path = self.n.run_dir/'rail.json'
        self.c.save(path)
        self.c.confirm_state('outer')
        self.c.load(path)
        self.assertEqual(self.c.state, 'unknown')
        bad = json.loads(path.read_text())
        bad['points']['A']['pelvis_from_wrist'][0][3] += .01
        data.write_json(path, bad)
        with self.assertRaisesRegex(ValueError, 'FK'):
            self.c.load(path)

    def test_rail_sequences(self):
        from task_rail_motion import build
        from task_path_smoothing import joint_line_trajectory
        config = self.configured()
        def fake_line(model, a, b, parameters, check, **kwargs):
            return joint_line_trajectory(data.MOVING, [a[n] for n in data.MOVING], [b[n] for n in data.MOVING])
        with patch('task_rail_motion.cartesian', side_effect=fake_line):
            for mode in ('hold', 'release'):
                push = build(self.n.model, config, self.c.ready(), self.q, 'push', mode, PlanningParameters())
                labels = [s['label'] for s in push['segments']]
                self.assertEqual(labels[:3], ['Rail hand open', 'Rail A approach 1', 'Rail A grasp position'])
                self.assertEqual(labels[-1], 'Rail push A to B' if mode == 'hold' else 'Rail return ready')
                expected = 'inner_held' if mode == 'hold' else 'inner_released'
                self.assertEqual(push['rail']['final_state'], expected)
                pull = build(self.n.model, config, self.c.ready(), push['segments'][-1]['end'],
                             'pull', mode, PlanningParameters())
                self.assertEqual(pull['segments'][0]['label'], 'Rail hand pull_grip' if mode == 'hold' else 'Rail hand open')
                self.assertEqual(pull['segments'][-1]['label'], 'Rail return ready')
                self.assertEqual(pull['rail']['final_state'], 'outer')
                for plan in (push, pull):
                    for a,b in zip(plan['segments'], plan['segments'][1:]):
                        self.assertEqual(a['end'], b['start'])
                    self.assertTrue(range_fingerprint(plan['segments']))

    def test_rail_real_cartesian_line_and_bounds(self):
        from task_rail_motion import cartesian, LINK, VELOCITY_SCALE, ACCELERATION_SCALE
        config = self.configured()
        target = config['points']['B']['joints']
        trajectory = cartesian(self.n.model, self.q, target, PlanningParameters())
        vmax, amax = task_motion_limits(data.MOVING, VELOCITY_SCALE, ACCELERATION_SCALE, PlanningParameters())
        rows = np.asarray(list(samples(trajectory, vmax, amax)))
        poses = self.n.model.batch_transforms([LINK], self.q, data.MOVING, rows)[LINK]
        origin = self.n.model.transform(LINK, self.q)[:3, 3]
        delta = poses[:, :3, 3]-origin
        self.assertLess(np.max(np.linalg.norm(delta[:, 1:], axis=1)), .0005)
        self.assertLess(np.max(np.abs(rows[:, -1]-self.q['waist_yaw_joint'])), 1e-8)
        self.assertTrue(np.allclose(trajectory.points[-1].positions, [target[n] for n in data.MOVING]))
        data.guarded_joints(self.n.model, rows)
        noisy = dict(target, waist_yaw_joint=target['waist_yaw_joint']+.002)
        locked = cartesian(self.n.model, self.q, noisy, PlanningParameters(), fixed_waist=True)
        self.assertTrue(all(abs(p.positions[-1]-self.q['waist_yaw_joint']) < 1e-9 for p in locked.points))
        end = dict(self.q, **dict(zip(data.MOVING, locked.points[-1].positions)))
        self.assertLess(np.linalg.norm(self.n.model.transform(LINK,end)[:3,3]-
                                       self.n.model.transform(LINK,noisy)[:3,3]), .0003)

    def test_rail_rejects_contact_waist_change_and_wrong_hand(self):
        from task_rail_motion import build
        config = self.configured()
        config['points']['B']['joints']['waist_yaw_joint'] += .01
        with self.assertRaisesRegex(ValueError, 'waist'):
            build(self.n.model, config, self.c.ready(), self.q, 'push', 'hold', PlanningParameters())
        config['points']['B']['joints']['waist_yaw_joint'] -= .01
        config['points']['B']['hand']['feedback_q'] = [1.]*6
        with self.assertRaisesRegex(ValueError, 'grip posture'):
            build(self.n.model, config, self.c.ready(), self.q, 'push', 'hold', PlanningParameters())

    def test_rail_state_epoch_and_start_barriers(self):
        # Advance state/epochs explicitly without wall-clock feedback expiry.
        self.n.session.clock=lambda:self.n.session.received
        self.configured()
        with self.assertRaisesRegex(RuntimeError, 'actual rail'):
            self.c.start_guard('push', 'hold')
        self.c.confirm_state('outer')
        self.c.start_guard('push', 'release')
        self.n.gateway['task_stop_epoch'] = '4'
        with self.assertRaisesRegex(RuntimeError, 'actual rail'):
            self.c.start_guard('push', 'hold')
        self.c.confirm_state('inner_held')
        self.n.session.current['right_elbow_joint'] += .1
        with self.assertRaises(RuntimeError):
            self.c.start_guard('pull', 'hold')

    def test_rail_session_has_no_vision_and_rejects_move_return(self):
        s = self.n.session; s.snapshot = {'old':True}; s.snapshot_at = -1000
        ticket = s.begin('rail', 'right', .15, .15)
        self.assertIsNone(s.snapshot)
        moved = dict(self.q); moved['right_elbow_joint'] += .1
        s.update(moved); s.update(self.q)
        with self.assertRaisesRegex(RuntimeError, 'moved'):
            s.ready(ticket, dict(kind='rail', segments=[1], start=self.q))

    def test_rail_panel_offscreen(self):
        os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
        from PyQt5 import QtWidgets
        from task_rail_panel import RailPanel
        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        self.app = app
        self.configured()
        panel = RailPanel(S(node=self.n, invoke=lambda f:f()))
        self.addCleanup(panel.deleteLater)
        panel.refresh(False); panel.show(); app.processEvents()
        self.assertEqual(panel.hand_table.rowCount(), 6)
        self.assertEqual(panel.points.rowCount(), 4)
        self.assertEqual(panel.trace_table.rowCount(),6)
        self.assertFalse(panel.endpoint_section.isVisible())
        legacy_controls = (panel.state_prompt,panel.state,panel.confirm_state_button,panel.state_label,panel.confirm)
        self.assertTrue(all(widget.isHidden() for widget in legacy_controls))
        with patch.object(self.c,'plan') as plan:
            panel.plan('push')
            plan.assert_called_once_with('push','cycle',confirmed=False,path_source='recorded',
                **panel.speed_options())
        panel.replay_speed.setValue(3.)
        with patch.object(self.c,'plan') as plan:
            panel.plan('pull')
            self.assertEqual(plan.call_args.kwargs['replay_multiplier'],3.)
        self.n.session.plan=dict(kind='rail');self.n.session.phase='PLAN_READY'
        panel.pending_replay=(self.n.session.generation,'push')
        panel.replay_fastest.setChecked(True)
        self.assertIsNone(panel.pending_replay)
        self.assertEqual(self.n.session.phase,'REPLAN_REQUIRED')
        panel.refresh(False)
        self.assertFalse(panel.replay_speed.isEnabled())
        with patch.object(panel,'ask',return_value=True),patch.object(self.c,'plan') as plan:
            panel.replay_cached('push')
            self.assertTrue(plan.call_args.kwargs['replay_fastest'])
            self.assertEqual(plan.call_args.kwargs['replay_multiplier'],3.)
        panel.pending_replay=None
        panel.replay_fastest.setChecked(False)
        panel.path_source.setCurrentIndex(panel.path_source.findData('endpoints'))
        self.assertTrue(all(not widget.isHidden() for widget in legacy_controls))
        panel.path_source.setCurrentIndex(panel.path_source.findData('recorded'))
        self.assertTrue(all(widget.isHidden() for widget in legacy_controls))
        self.assertEqual(panel.mode.currentData(),'cycle')
        self.assertFalse(panel.mode.model().item(1).isEnabled())
        self.assertFalse(panel.mode.model().item(2).isEnabled())
        self.c.recording = dict(phase='recording',finish=threading.Event(),cancel=threading.Event(),
                               buffer=S(rows=[],times=[]))
        panel.refresh(True)
        self.assertTrue(panel.finish_trace_button.isEnabled())
        self.assertTrue(panel.cancel_trace_button.isEnabled())
        self.assertFalse(panel.start_trace_button.isEnabled())
        self.assertFalse(panel.path_source.isEnabled())
        from PyQt5 import QtCore, QtTest
        for button in (panel.finish_trace_button,panel.cancel_trace_button):
            clicked=QtTest.QSignalSpy(button.clicked)
            QtTest.QTest.mousePress(button,QtCore.Qt.LeftButton)
            panel.refresh(True)
            self.assertTrue(button.isDown())
            # Verify delivery without finalizing the synthetic recording fixture.
            with patch.object(panel.window,'invoke'):
                QtTest.QTest.mouseRelease(button,QtCore.Qt.LeftButton)
            self.assertEqual(len(clicked),1)
        self.c.recording = None; panel.refresh(False)
        for width,height in ((620,720), (720,860), (1100,1000)):
            panel.resize(width,height); app.processEvents()
            scroll = panel.findChild(QtWidgets.QScrollArea)
            self.assertEqual(scroll.horizontalScrollBar().maximum(), 0)
            if os.environ.get('SCREEN_TEST_ARTIFACTS'):
                directory = Path(os.environ['SCREEN_TEST_ARTIFACTS']); directory.mkdir(parents=True,exist_ok=True)
                self.assertTrue(panel.grab().save(str(directory/f'rail_{width}x{height}.png')))
                scroll.verticalScrollBar().setValue(scroll.verticalScrollBar().maximum())
                app.processEvents()
                self.assertTrue(panel.grab().save(str(directory/f'rail_bottom_{width}x{height}.png')))
                scroll.verticalScrollBar().setValue(0)
        self.n.display_pub.assert_not_called()

    def test_trace_tab_in_complete_console_offscreen(self):
        os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
        from real_task_app import Window
        from PyQt5 import QtWidgets
        import test_task_console
        fixture = test_task_console.ConsoleTests('test_planning_is_disabled_and_explained')
        fixture.setUpClass(); fixture.setUp()
        self.addCleanup(fixture.tearDown); self.addCleanup(fixture.doCleanups)
        fixture.node.rail = self.c
        fixture.node.hand_states = {}
        window = Window(fixture.node); window.timer.stop(); self.addCleanup(window.deleteLater)
        window.tabs.setCurrentWidget(window.rail_panel)
        window.show(); window.refresh(); QtWidgets.QApplication.processEvents()
        for width,height in ((620,720),(720,860),(1100,1000)):
            window.resize(width,height); QtWidgets.QApplication.processEvents()
            scroll = window.rail_panel.findChild(QtWidgets.QScrollArea)
            scroll.verticalScrollBar().setValue(scroll.verticalScrollBar().maximum())
            QtWidgets.QApplication.processEvents()
            self.assertEqual(scroll.horizontalScrollBar().maximum(),0)
            if os.environ.get('SCREEN_TEST_ARTIFACTS'):
                directory = Path(os.environ['SCREEN_TEST_ARTIFACTS']); directory.mkdir(parents=True,exist_ok=True)
                self.assertTrue(window.grab().save(str(directory/f'rail_console_{width}x{height}.png')))

    def test_rail_production_plan_execute_modes_and_failure_without_release(self):
        self.run_rail_production('endpoints')

    def test_recorded_rail_production_execution_modes_and_failure_without_release(self):
        self.run_rail_production('recorded')

    def test_rail_move_still_stops_above_configured_endpoint_tolerance(self):
        self.run_rail_production('recorded',endpoint_failure=True)

    def test_rail_cycle_production_arbitrary_hand_and_persistent_reuse(self):
        self.run_rail_production('recorded',cycle_only=True)

    def run_rail_production(self,path_source,*,endpoint_failure=False,cycle_only=False):
        from builtin_interfaces.msg import Time
        from rclpy.parameter import Parameter
        from real_task_app import TaskNode
        from task_hand_control import HandControl, model_feedback
        self.traced() if path_source == 'recorded' else self.configured()
        self.c.config['hands']['pull_grip']['feedback_q']=[.3]*6
        if path_source == 'recorded':
            trace=self.c.config['traces']['pull']
            for evidence in trace['hand_evidence']:evidence['feedback_q']=[.3]*6
        n, c = self.n, self.c
        physical = dict(self.q)
        hand_q = {side:list(model_feedback(side, physical)) for side in ('left', 'right')}
        hand_q['right'] = [1.]*6
        physical.update(model_joints('right', hand_q['right']))
        clock = [time.monotonic()]
        n.session.clock = lambda:clock[0]
        n.closing = threading.Event()
        n.check = lambda ticket=None:TaskNode.check(n, ticket)
        goals, commands, stage_calls = [], [], []
        endpoint_error = [0.]
        n.gateway.update(task_stage_supported='true',task_stage_profile='normal',task_stage_revision='0',
            task_stage_switching='false',task_stage_max_kp='unbounded',task_stage_max_kd='unbounded',
            task_stage_waist_supported='true',task_stage_waist_max_kp='unbounded',task_stage_waist_max_kd='unbounded',
            task_stage_wrist_supported='true',task_stage_distal_wrist_supported='true',task_stage_arm_kp_supported='true',
            task_stage_kp='0',task_stage_kd='0',task_stage_waist_kp='0',task_stage_waist_kd='0',
            task_stage_wrist_kp='0',task_stage_distal_wrist_kp='0',task_stage_arm_kp_count='0')
        def stage_request(request):
            self.assertEqual(n.gateway['phase'],'ARMED_HOLD')
            values={p.name:Parameter.from_parameter_msg(p).value for p in request.parameters}
            self.assertEqual(values['task_stage_arm'],'right')
            self.assertEqual(values['task_stage_stop_epoch'],3)
            stage_calls.append(values)
            n.gateway.update(task_stage_profile=values['task_stage_profile'],task_stage_arm='right',
                task_stage_revision=str(len(stage_calls)),task_stage_switching='false')
            for key in ('kp','kd','waist_kp','waist_kd','wrist_kp','distal_wrist_kp'):
                n.gateway['task_stage_'+key]=str(values.get('task_stage_'+key,0.))
            gains=values.get('task_stage_arm_kp',[])
            n.gateway['task_stage_arm_kp_count']=str(len(gains))
            n.gateway.update({f'task_stage_arm_kp_{i}':str(v) for i,v in enumerate(gains)})
            return S(result=S(successful=True))
        n.waist_parameters=S(wait_for_service=lambda **kw:True,call_async=stage_request)
        fail_on = [None]
        active_overlap=[None]
        def advance(seconds):
            deadline = clock[0]+seconds
            while clock[0] < deadline:
                clock[0] = min(deadline, clock[0]+.01)
                n.gateway_received = clock[0]
                if active_overlap[0] is not None:
                    until,trajectory,callback=active_overlap[0]
                    callback(S(feedback=S(desired=S(velocities=[.1]),actual=S(velocities=[.1]))))
                    if clock[0]>=until:
                        physical.update(zip(trajectory.joint_names,trajectory.points[-1].positions))
                        n.gateway['phase']='ARMED_HOLD'
                        active_overlap[0]=None
                n.session.update(physical)
        def transport(command):
            op, side, _, *values = command.split()
            commands.append((op, side, values))
            if op == 'set':
                hand_q[side] = list(map(float, values))
                physical.update(model_joints(side, hand_q[side]))
            advance(.02)
            if op != 'get':
                return dict(code=0)
            return dict(code=0, hand=side, online=True, feedback_q=list(hand_q[side]),
                target_q=list(hand_q[side]), error=[0]*6, temperature=[30]*6,
                timestamp_ms=round(clock[0]*1000))
        n.hand = HandControl('offline', enabled=True, transport=transport,
                             clock=lambda:clock[0], sleep=advance)
        n.get_clock = lambda:S(now=lambda:S(to_msg=lambda:Time()))
        n.hand_pub = Mock()
        n.cache_hand_state = lambda side,state:TaskNode.cache_hand_state(n,side,state)
        n.read_hands = lambda ticket=None:TaskNode.read_hands(n,ticket)
        def send(goal,feedback_callback=None):
            goals.append(copy.deepcopy(goal.trajectory))
            if feedback_callback is not None:
                n.gateway['phase']='RUNNING'
                until=clock[0]+2.
                active_overlap[0]=(until,goal.trajectory,feedback_callback)
                future=S(done=lambda:clock[0]>=until,
                    result=lambda:S(status=4,result=S(error_code=0,error_string='offline')))
                return S(accepted=True,get_result_async=lambda:future)
            return S(accepted=True,get_result_async=lambda:S(trajectory=goal.trajectory))
        n.actions = {'right':S(wait_for_server=lambda **kw:True,send_goal_async=send)}
        def wait(future, ticket=None, *args):
            n.check(ticket)
            if not hasattr(future, 'trajectory'):
                return future
            if fail_on[0] == len(goals):
                return S(status=6,result=S(error_code=-1,error_string='blocked rail'))
            n.gateway['phase'] = 'RUNNING'
            n.session.received-=.2; n.gateway_received-=2.
            n.check(ticket)
            physical.update(zip(future.trajectory.joint_names, future.trajectory.points[-1].positions))
            physical['right_wrist_yaw_joint'] += endpoint_error[0]
            advance(.55); n.gateway['phase'] = 'ARMED_HOLD'
            return S(status=4,result=S(error_code=0,error_string='offline'))
        n.wait = wait
        n.stop = Mock(side_effect=lambda:(c.started(),n.session.stop(),n.hand.request_stop()))
        with patch('real_task_app.time.monotonic',lambda:clock[0]), patch('real_task_app.time.sleep',advance):
            advance(.6)
            if cycle_only:
                from task_rail_control import RailControl
                import task_rail_trace as tr
                with self.assertRaisesRegex(RuntimeError,'成功规划'):
                    c.plan('push','cycle',path_source='recorded',reuse=True)
                self.assertFalse(goals)
                for reuse in (False,True):
                    for operation in ('push','pull'):
                        if reuse:
                            # New control instance emulates restart, retaining only disk data/cache.
                            config=copy.deepcopy(c.config);cache_dir=c.cache_dir
                            c=RailControl(n);c.config=config;c.cache_dir=cache_dir;n.rail=c
                        physical['right_elbow_joint']+=.05
                        hand_q['right']=[.6]*6
                        physical.update(model_joints('right',hand_q['right']))
                        advance(.6)
                        c.state='unknown';c.state_epoch=-1
                        with patch.object(c,'read_stable_hands',side_effect=AssertionError('Repeated hand stability check')):
                            with patch.object(n,'read_hands',wraps=n.read_hands) as read_hands:
                                if reuse:
                                    with patch.object(tr,'replay',side_effect=AssertionError('Recomputed persisted curve')):
                                        c.plan(operation,'cycle',path_source='recorded',reuse=True,replay_multiplier=1.5,replay_fastest=False,trim_idle_ends=False)
                                else:c.plan(operation,'cycle',path_source='recorded',replay_multiplier=1.5,replay_fastest=False,trim_idle_ends=False)
                                self.assertEqual(read_hands.call_count,1)
                        self.assertEqual(n.session.plan['planning_config']['values']['rail_replay_multiplier'],1.5)
                        self.assertFalse(n.session.plan['planning_config']['values']['rail_trim_idle_ends'])
                        self.assertTrue(n.session.plan['rail']['reuse']==reuse)
                        self.assertEqual(len([s for s in n.session.plan['segments'] if 'trace_stage' in s]),3)
                        TaskNode.execute(n,scene_confirmed=True)
                        self.assertEqual(c.state,'inner_released' if operation=='push' else 'outer')
                        self.assertEqual(hand_q['right'],[1.]*6)
                        self.assertEqual(n.session.phase,'ARRIVED_HOLD')
                        self.assertEqual(n.gateway['task_stage_profile'],'normal')
                        for name in data.MOVING:self.assertAlmostEqual(physical[name],c.ready()[name])
                        n.stop.assert_not_called()
                return
            for mode in ('hold', 'release'):
                c.confirm_state('outer')
                c.plan('push', mode, confirmed=True,path_source=path_source)
                self.assertEqual(n.session.plan['kind'], 'rail')
                self.assertEqual((n.session.ticket.velocity,n.session.ticket.acceleration),(.7,.7))
                before = len(commands)
                proof = n.session.plan['rail']
                proof['trajectory_sha256'] = 'tampered'
                with self.assertRaisesRegex(RuntimeError, 'trajectory changed'):
                    TaskNode.execute(n, scene_confirmed=True)
                self.assertEqual(len(commands), before)
                proof['trajectory_sha256'] = range_fingerprint(n.session.plan['segments'])
                TaskNode.execute(n, scene_confirmed=True)
                self.assertEqual(c.state, 'inner_held' if mode == 'hold' else 'inner_released')
                self.assertEqual(n.session.phase, 'ARRIVED_HOLD')
                self.assertEqual(hand_q['right'], c.config['hands']['grip' if mode == 'hold' else 'open']['feedback_q'])
                n.stop.assert_not_called()
                c.plan('pull', mode, confirmed=True,path_source=path_source)
                TaskNode.execute(n, scene_confirmed=True)
                self.assertEqual(c.state, 'outer')
                self.assertTrue(any(op=='set' and side=='right' and np.allclose(list(map(float,values)),[.3]*6,atol=1e-9,rtol=0.)
                                    for op,side,values in commands))
                self.assertEqual(hand_q['right'], [1.]*6)
                for name in data.MOVING:
                    self.assertAlmostEqual(physical[name], c.ready()[name])
                self.assertEqual(n.gateway['task_stage_profile'],'normal')
            if path_source=='recorded':
                from task_stage_control import replay_arm_kp,replay_stage_parameters
                from task_planning_config import load_parameters
                params,_=load_parameters()
                recorded=[s for s in stage_calls if s['task_stage_profile']=='withdraw']
                self.assertEqual(len(recorded),10)
                for request in recorded:
                    self.assertEqual(list(request['task_stage_arm_kp']),replay_arm_kp(params,'left'))
                    self.assertEqual(request['task_stage_kd'],params.withdraw_proximal_kd)
                    self.assertEqual(request['task_stage_waist_kp'],replay_stage_parameters(params).stage_waist_kp)
            else:self.assertEqual(stage_calls,[])
            move_keys=(['push_A','push_B','pull_A','pull_B','start:push','connect:pull']
                       if path_source=='recorded' else [])+['preparation']
            for key in move_keys:
                # Actual above-table failure: 0.644 degrees is within the shared tolerance.
                endpoint_error[0] = np.radians(.644) if key == 'preparation' else 0.
                physical['right_elbow_joint']+=.2
                advance(.6)
                target=c.movement_target(key,waist=physical['waist_yaw_joint'])
                before=len(commands)
                c.prepare_move(key,confirmed=True)
                self.assertEqual((n.session.ticket.velocity,n.session.ticket.acceleration),(.7,.7))
                self.assertTrue(n.session.plan['rail']['arm_only'])
                if endpoint_failure and key == 'preparation':
                    limit=n.session.plan['planning_config']['values']['endpoint_tolerance_deg']
                    endpoint_error[0]=np.radians(limit+.1)
                    with self.assertRaisesRegex(TimeoutError,'right_wrist_yaw_joint'):
                        TaskNode.execute(n,scene_confirmed=True)
                    n.stop.assert_called_once()
                    self.assertEqual(len(commands),before)
                    return
                with patch.object(n.hand,'begin_motion',side_effect=AssertionError('Arm-only move touched hand')):
                    TaskNode.execute(n,scene_confirmed=True)
                self.assertEqual(len(commands),before)
                self.assertEqual(c.state,'unknown')
                for name in data.MOVING:
                    self.assertAlmostEqual(physical[name],target[name]+(
                        endpoint_error[0] if name == 'right_wrist_yaw_joint' else 0.))
                n.stop.assert_not_called()
            physical['right_wrist_yaw_joint'] -= endpoint_error[0]
            endpoint_error[0] = 0.
            advance(.6)
            c.confirm_state('outer')
            c.plan('push', 'release', confirmed=True,path_source=path_source)
            fail_on[0] = len(goals)+(2 if path_source == 'recorded' else 3)
            with self.assertRaisesRegex(RuntimeError, 'blocked rail'):
                TaskNode.execute(n, scene_confirmed=True)
            n.stop.assert_called_once()
            self.assertEqual(c.state, 'unknown')
            self.assertEqual(hand_q['right'], c.config['hands']['grip']['feedback_q'])
            self.assertEqual(len(goals), fail_on[0])


if __name__ == '__main__': unittest.main()
