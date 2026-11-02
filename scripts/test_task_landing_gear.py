"""Offline-only regression coverage; no ROS nodes or hardware clients created."""
import copy
import json
import os
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock, patch
import xml.etree.ElementTree as ET

import numpy as np

import task_landing_gear_data as data
from task_landing_gear_control import LandingGearControl
from task_dring_demo_control import DringDemoControl
from task_hand_control import model_joints
from task_planning_config import PlanningParameters
from task_session import Session, ARMS
from test_task_dring_demo import fixture


def hand_state(value=1., side='right'):
    return dict(hand=side, code=0, online=True, feedback_q=[value]*6, target_q=[value]*6,
                error=[0]*6, temperature=[25]*6, timestamp_ms=1)


def bundle():
    model, q, _ = fixture()
    names = data.moving('right')
    points = {}
    for key, value in (('preparation', 0.), ('lower', .12), ('upper', .24)):
        points[key] = dict.fromkeys(names, 0.)
        points[key][names[0]] = value
    traces = {}
    t = np.linspace(0., 4., 401); u = t/4.; blend = u**3*(10-15*u+6*u*u)
    for slot, (first, last) in data.ENDPOINTS.items():
        a = np.array([points[first][n] for n in names]); b = np.array([points[last][n] for n in names])
        traces[slot] = dict(raw=dict(positions=(a+(b-a)*blend[:, None]).tolist(), time_s=t.tolist(),
            stamp_ns=[10**9+round(v*10**9) for v in t]), hand=None)
    return model, q, points, dict(schema=data.SCHEMA, side='right', ready=None,
        hands=dict(open=hand_state(1.), grip=hand_state(.3)), traces=traces)


class LandingDataTests(unittest.TestCase):
    def test_six_segments_have_required_hand_order_and_preserve_left_arm(self):
        model, q, points, config = bundle()
        original = copy.deepcopy(config)
        for slot, (first, last) in data.ENDPOINTS.items():
            start = dict(q, **points[first])
            plan = data.build(model, config, start, slot, PlanningParameters(), .7, .7, 1.,
                              preparation=points['preparation'])
            self.assertEqual(plan['arm'], 'right')
            self.assertFalse(plan['demo']['collision_checked'])
            self.assertEqual(plan['demo']['operation'], slot)
            hands = [s['posture'] for s in plan['segments'] if s['kind'] == 'hand']
            self.assertEqual(hands, list(data.HAND_POLICY[slot]))
            self.assertEqual(plan['segments'][0]['kind'], 'hand')
            for a, b in zip(plan['segments'], plan['segments'][1:]):self.assertEqual(a['end'], b['start'])
            for name in data.moving('right'):
                self.assertAlmostEqual(plan['segments'][-1]['end'][name], points[last][name])
            for segment in plan['segments']:
                self.assertIsNone(segment.get('recorded_hand'))
                if segment['kind'] == 'arm':
                    self.assertEqual(segment['trajectory'].joint_names, list(data.moving('right')))
                    self.assertEqual({n:segment['end'][n] for n in model_joints('right', [1.]*6)},
                                     model_joints('right', config['hands'][hands[0]]['feedback_q']))
                for name in ARMS['left']+tuple(model_joints('left', [1.]*6)):
                    self.assertEqual(segment['end'][name], q[name])
            self.assertTrue(data.preview(plan).points)
        self.assertEqual(config, original)

    def test_arbitrary_current_pose_joins_after_hand_to_recorded_first_frame(self):
        model, q, points, config = bundle()
        q[data.moving('right')[0]] = -.15
        config['points'] = {'lower':{'joints':{}}}  # Legacy points are not motion targets.
        for operation in data.SLOTS:
            plan = data.build(model, config, q, operation, PlanningParameters(), .7, .7, 1.)
            self.assertEqual(plan['demo']['transition_count'], 1)
            hand, join, recorded = plan['segments'][:3]
            self.assertEqual(hand['kind'], 'hand')
            self.assertEqual(join['role'], 'saved_start_transition')
            self.assertEqual(join['start'], hand['end'])
            self.assertEqual(join['end'], recorded['start'])
            self.assertEqual([join['end'][n] for n in data.moving('right')],
                             config['traces'][operation]['raw']['positions'][0])

    def test_return_requires_own_recording_and_missing_hand_is_rejected(self):
        model, q, _, config = bundle()
        del config['traces']['lower_to_above']
        with self.assertRaises(KeyError):
            data.build(model, config, q, 'lower_to_above', PlanningParameters(), .7, .7, 1.)
        del config['hands']['grip']
        with self.assertRaisesRegex(ValueError, '抓握'):
            data.build(model, config, q, 'above_to_upper', PlanningParameters(), .7, .7, 1.)
        plan = data.build(model, config, q, 'upper_to_above', PlanningParameters(), .7, .7, 1.)
        self.assertEqual([s['posture'] for s in plan['segments'] if s['kind'] == 'hand'], ['open'])

    def test_no_implicit_cycle_and_no_left_arm(self):
        model, q, points, config = bundle()
        with self.assertRaises(ValueError):
            data.build(model, config, q, 'above_to_lower', PlanningParameters(), .7, .7, 1., full_cycle=True)
        config['side'] = 'left'
        with self.assertRaises(ValueError):
            data.build(model, config, q, 'above_to_lower', PlanningParameters(), .7, .7, 1.)


class LandingControlTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        model, self.q, self.points, config = bundle()
        robot = self.root/'robot.urdf'; robot.write_text(ET.tostring(model.robot, encoding='unicode'))
        self.clock = [10.]
        session = Session(lambda:self.clock[0])
        for _ in range(8):
            self.clock[0] += .1; session.update(self.q)
        session.capture_home()
        self.node = S(args=S(robot_urdf=robot, hardware=False, disconnected=True), model=model,
            lock=threading.RLock(), session=session, display_pub=Mock(), demo_preview_pub=Mock(),
            run_dir=self.root/'run', worker=None, shutdown_requested=False, hand_control=False,
            prepared={'right':dict(joints=self.points['preparation'])}, gateway=dict(phase='TASK_ADMITTANCE',
                task_stop_epoch='1', task_stop_active='false'), gateway_received=0., status='')
        self.node.check = lambda ticket=None:session.check(ticket) if ticket else None
        self.node.launch_work = lambda work:work()
        self.control = LandingGearControl(self.node, self.root/'gear.json')
        config['identity'] = self.control.identity(); self.config = config
        self.control.persist(config)

    def test_persistence_restart_and_new_keep_hands_not_authorization(self):
        c = self.control
        c.save(self.root/'export.json')
        restored = LandingGearControl(self.node, c.store)
        self.assertEqual(restored.config, c.config)
        self.assertIsNone(restored.recording)
        self.assertIsNone(self.node.session.plan)
        c.new()
        self.assertEqual(c.config['traces'], {})
        c.load(self.root/'export.json')
        self.assertEqual(c.config, restored.config)

    def test_bad_import_keeps_previous_and_cannot_import_dring(self):
        for field, value in (('side', 'left'), ('schema', 'g1_dring_demonstration_v1'), ('hands', {'pull_grip':{}})):
            bad = copy.deepcopy(self.config); bad[field] = value
            path = self.root/'bad.json'; path.write_text(json.dumps(bad))
            with self.assertRaises(ValueError):self.control.load(path)
            self.assertEqual(self.control.config, self.config)

    def test_legacy_points_are_optional_and_never_validate_trace_endpoints(self):
        old = copy.deepcopy(self.config)
        old['points'] = dict(lower={'joints':{}}, upper={'joints':{}})
        self.control.persist(old)
        self.assertEqual(self.control.config['traces'], self.config['traces'])
        self.control.new()
        self.assertNotIn('points', self.control.config)

    def test_all_six_slots_have_no_prerecorded_start_requirement(self):
        self.node.prepared = {}
        for slot in data.SLOTS:self.assertIsNone(self.control.recording_start(slot))

    def test_capture_failure_to_launch_clears_recording_state(self):
        self.node.launch_work = Mock(side_effect=RuntimeError('worker unavailable'))
        with patch.object(self.control, 'gateway_guard'), self.assertRaisesRegex(RuntimeError, 'worker unavailable'):
            self.control.start_trace('above_to_lower')
        self.assertIsNone(self.control.recording)
        self.assertEqual(self.node.session.phase, 'TASK_ADMITTANCE')
        self.assertEqual(self.control.config, self.config)

    def test_capture_saves_only_selected_slot_and_uses_independent_namespace(self):
        c = self.control
        self.node.prepared = {}
        self.node.session.measured = Mock(return_value=dict(self.q, **{data.moving('right')[0]:.45}))
        raw = copy.deepcopy(self.config['traces']['above_to_lower']['raw'])
        class Capture:
            def __init__(inner, *args, **kwargs):
                self.assertEqual(kwargs['side'], 'right')
                self.assertEqual(kwargs['namespace'], 'landing_gear_recordings')
                inner.directory = self.root/'raw'
            def check(inner):return {'ready':True}
            def finish(inner, check):check(); return raw
            def close(inner):pass
        def sleep(_):c.recording['finish'].set()
        with patch.object(c, 'gateway_guard'), patch('task_dring_demo_control.DirectCapture', Capture), \
                patch('task_dring_demo_control.time.sleep', sleep):
            c.start_trace('above_to_lower')
        self.assertIsNone(c.recording)
        for slot in data.SLOTS:
            if slot != 'above_to_lower':self.assertEqual(c.config['traces'][slot], self.config['traces'][slot])
        self.assertEqual(c.config['traces']['above_to_lower']['raw'], raw)

    def test_cancel_during_capture_keeps_old_data(self):
        c = self.control
        class Capture:
            directory = self.root/'raw'
            def __init__(inner, *args, **kwargs):pass
            def check(inner):return {'ready':True}
            def close(inner):pass
        with patch.object(c, 'gateway_guard'), patch('task_dring_demo_control.DirectCapture', Capture), \
                patch('task_dring_demo_control.time.sleep', lambda _:c.cancel_trace()), self.assertRaises(RuntimeError):
            c.start_trace('above_to_lower')
        self.assertEqual(c.config, self.config)
        self.assertIsNone(c.recording)

    def test_prepare_routes_guards_and_completion_to_correct_business(self):
        from real_task_app import TaskNode, demonstration_control
        n = self.node; c = self.control
        n.hand_control = True
        n.read_hands = Mock(return_value={s:hand_state(1., s) for s in ('left', 'right')})
        n.landing_gear = c; n.demo = DringDemoControl(n, self.root/'dring.json')
        n.wait_stationary = lambda *args, **kwargs:TaskNode.wait_stationary(n, *args, **kwargs)
        with patch.object(c, 'gateway_guard'), patch.object(c, 'fresh_hands'), patch('task_dring_demo_control.load_parameters',
                return_value=(PlanningParameters(), dict(values={}))), patch(
                'task_dring_demo_control.controller_bounded_parameters', side_effect=lambda p,r,g:(p,r)):
            c.prepare('above_to_lower', 1., .7, .7, True)
            plan = n.session.plan
            self.assertEqual(n.session.phase, 'PLAN_READY')
            self.assertEqual(plan['demo']['workflow'], 'landing_gear')
            self.assertIsNone(plan['demo']['preparation'])
            self.assertFalse(plan['demo']['arm_only'])
            self.assertIs(demonstration_control(n, plan), c)
            c.execution_guard(plan, preflight=True)
            c.replay_preview()
            with self.assertRaisesRegex(RuntimeError, '业务不匹配'):n.demo.execution_guard(plan)
            n.prepared['right']['joints'] = dict(self.points['upper'])
            c.execution_guard(plan, preflight=True)
            bad = copy.deepcopy(plan); bad['segments'][0]['target_q'][0] = .2
            with self.assertRaisesRegex(RuntimeError, '内容改变'):c.execution_guard(bad, preflight=True)
            bad = copy.deepcopy(plan); bad['segments'][0]['hand_target_required'] = True
            with self.assertRaisesRegex(RuntimeError, '内容改变'):c.execution_guard(bad, preflight=True)
            bad = copy.deepcopy(plan); bad['demo']['arm_only'] = True
            with self.assertRaisesRegex(RuntimeError, '旧起落架杆'):c.execution_guard(bad, preflight=True)
            n.gateway['task_stop_epoch'] = '2'
            with self.assertRaises(RuntimeError):c.execution_guard(plan, running=True)

    def test_missing_hardware_cannot_capture_or_plan_or_change_hand(self):
        with self.assertRaises(RuntimeError):self.control.start_trace('above_to_lower')
        with self.assertRaises(RuntimeError):self.control.prepare('above_to_lower', 1., .7, .7, True)
        with self.assertRaises(RuntimeError):self.control.record(hand='open')
        with self.assertRaises(RuntimeError):self.control.apply_hand('grip', confirmed=True)
        self.assertIsNone(self.node.session.plan)

    def run_executor(self, operation, fail=None):
        from builtin_interfaces.msg import Time
        from real_task_app import TaskNode
        from task_saved_test import range_fingerprint
        from task_screen_teach import fingerprint
        n = self.node; c = self.control
        physical = dict(self.q); physical[data.moving('right')[0]] = -.1
        n.session = Session(lambda:self.clock[0])
        for _ in range(8):self.clock[0] += .1; n.session.update(physical)
        n.session.capture_home()
        n.session.measured = lambda **kwargs:dict(physical)
        n.session.current = dict(physical)
        ticket = n.session.begin('dring_demo', 'right', .7, .7)
        plan = data.build(n.model, c.config, physical, operation, PlanningParameters(), .7, .7, 1.)
        directory = self.root/operation; directory.mkdir(exist_ok=True)
        plan.update(directory=str(directory), planning_config=dict(values=dict(stage_control_enabled=False)))
        plan['demo'].update(workflow=c.workflow, revision=c.revision, stop_epoch='1',
            waist_control_mode=None, waist_control_revision=None, preparation=None,
            config_sha256=fingerprint(c.config), trajectory_sha256=range_fingerprint(plan['segments']),
            hand_sha256=fingerprint([s.get('recorded_hand') for s in plan['segments']]))
        n.session.ready(ticket, plan)
        n.args.hardware=True; n.hand_control=True; n.landing_gear=c
        n.gateway.update(phase='ARMED_HOLD', task_waist_trajectory_supported='true')
        n.gateway_received=time.monotonic(); n.stop=Mock(); n.closing=threading.Event()
        n.check=lambda t=None:TaskNode.check(n,t)
        n.get_clock=lambda:S(now=lambda:S(to_msg=lambda:Time()))
        n.read_hands=lambda *args:{s:hand_state(1.,s) for s in ('left','right')}
        n.cache_hand_state=Mock(); events=[]
        def transition(side, target, check, **kwargs):
            check(); events.append(('hand', list(target)))
            if fail=='hand':raise RuntimeError('hand failed')
            physical.update(model_joints(side,target)); n.session.current=dict(physical)
            return [hand_state(target[0])]
        n.hand=S(begin_motion=lambda check:check(), transition=transition)
        def send(goal):
            events.append(('arm', list(goal.trajectory.points[-1].positions)))
            physical.update(zip(goal.trajectory.joint_names,goal.trajectory.points[-1].positions))
            n.session.current=dict(physical)
            return S(accepted=True, get_result_async=lambda:S(status=6 if fail=='arm' else 4,
                result=S(error_code=-1 if fail=='arm' else 0,error_string='arm failed' if fail else '')))
        n.actions={'right':S(wait_for_server=lambda **kwargs:True,send_goal_async=send)}
        n.wait=lambda future,*args:future
        def stationary(node,ticket,*,guard=None,on_ready=None,**kwargs):
            node.check(ticket)
            if guard:guard()
            return on_ready(dict(physical)) if on_ready else dict(physical)
        def hand_completion(hand,side,target,check,*,require_target,**kwargs):
            self.assertIs(require_target,False)
            return hand.transition(side,target,check,**kwargs)
        with patch.object(c,'gateway_guard'), patch.object(TaskNode,'wait_stationary',side_effect=stationary), \
                patch('task_hand_completion.transition',side_effect=hand_completion), \
                patch('task_stage_control.replay_stage_parameters',side_effect=lambda p:p):
            if fail:
                with self.assertRaises(RuntimeError):TaskNode.execute(n,scene_confirmed=True)
                n.stop.assert_called_once()
            else:
                TaskNode.execute(n,scene_confirmed=True)
                self.assertEqual(n.session.phase,'ARRIVED_HOLD')
                n.stop.assert_not_called()
        return events, plan

    def test_real_executor_obeys_all_six_hand_join_replay_sequences(self):
        for operation in data.SLOTS:
            with self.subTest(operation=operation):
                events,plan=self.run_executor(operation)
                self.assertEqual([x[0] for x in events],[s['kind'] for s in plan['segments']])
                self.assertEqual([x[1] for x in events if x[0]=='hand'],
                    [self.config['hands'][p]['feedback_q'] for p in data.HAND_POLICY[operation]])

    def test_failed_open_never_sends_join_or_recording(self):
        events,_=self.run_executor('lower_to_above',fail='hand')
        self.assertEqual([x[0] for x in events],['hand'])

    def test_failed_arm_never_sends_final_grip(self):
        events,_=self.run_executor('above_to_lower',fail='arm')
        self.assertEqual([x[0] for x in events],['hand','arm'])

    def test_all_six_push_programs_preserve_hand_order(self):
        from task_jetson_push import serialize
        from releases import action_name, program
        for operation in data.SLOTS:
            plan=data.build(self.node.model,self.config,self.q,operation,PlanningParameters(),.7,.7,1.)
            plan['demo']['workflow']='landing_gear'
            frozen=serialize(plan)
            self.assertEqual(action_name(frozen),'gear_'+operation)
            published=program(frozen)
            self.assertEqual([s['target_q'] for s in published if s['kind']=='hand'],
                [self.config['hands'][p]['feedback_q'] for p in data.HAND_POLICY[operation]])
            self.assertTrue(all(s['hand_target_required'] is False for s in published if s['kind']=='hand'))
            self.assertEqual(sum(s['kind']=='recorded' for s in published),1)

    def test_hand_record_and_transition_reuse_right_hand_only(self):
        n = self.node; c = self.control; n.hand_control = True
        state = dict(hand='right', code=0, online=True, feedback_q=[.5]*6, target_q=[.5]*6,
                     error=[0]*6, temperature=[25]*6, timestamp_ms=1)
        with patch.object(c, 'gateway_guard'), patch.object(c, 'read_stable_hands', return_value={'right':state}), \
                patch.object(c, 'fresh_hands'):
            c.record(hand='grip')
        self.assertEqual(c.config['hands']['grip'], state)
        self.assertEqual(LandingGearControl(n, c.store).config['hands']['grip'], state)
        n.hand = Mock(); n.hand.transition.return_value = [state]; n.cache_hand_state = Mock()
        mismatch=copy.deepcopy(state);mismatch['feedback_q'][3] += .09
        with patch.object(c, 'gateway_guard'), patch('task_hand_completion.transition',return_value=[mismatch]) as transition:
            c.apply_hand('grip', confirmed=True)
        self.assertEqual(transition.call_args.args[:3], (n.hand,'right',tuple(state['feedback_q'])))
        self.assertIs(transition.call_args.kwargs['require_target'],False)
        n.hand.transition.assert_not_called()
        self.assertFalse((self.root/'rail_hand_presets.json').exists())

    def test_offscreen_panel_layout_and_busy_controls(self):
        os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
        from PyQt5 import QtWidgets
        from task_landing_gear_panel import LandingGearPanel
        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        self.node.landing_gear = self.control
        window = S(node=self.node, clear=QtWidgets.QCheckBox(), invoke=lambda f:f())
        panel = LandingGearPanel(window)
        for width, height in ((620, 720), (1100, 1000)):
            panel.resize(width, height); panel.show(); panel.refresh(False); app.processEvents()
            scroll = panel.findChild(QtWidgets.QScrollArea)
            self.assertEqual(scroll.horizontalScrollBar().maximum(), 0)
            self.assertEqual(panel.table.rowCount(), 6)
            self.assertFalse(any('记录上杆位' in b.text() or '记录下杆位' in b.text()
                                 for b in panel.findChildren(QtWidgets.QPushButton)))
            self.assertTrue(panel.table.isEnabled())
            path = Path('/tmp')/f'landing_gear_panel_{width}x{height}.png'
            self.assertTrue(panel.grab().save(str(path)))
            scroll.verticalScrollBar().setValue(scroll.verticalScrollBar().maximum()); app.processEvents()
            self.assertTrue(panel.grab().save(str(Path('/tmp')/f'landing_gear_panel_bottom_{width}x{height}.png')))
            scroll.verticalScrollBar().setValue(0)
        panel.refresh(True)
        self.assertFalse(panel.table.isEnabled())
        self.assertFalse(panel.finish.isEnabled())
        panel.close()

    def test_landing_tab_in_complete_console(self):
        from PyQt5 import QtWidgets
        from real_task_app import Window
        from test_task_console import ConsoleTests
        fixture = ConsoleTests('test_planning_is_disabled_and_explained')
        fixture.setUpClass(); fixture.setUp()
        self.addCleanup(fixture.doCleanups); self.addCleanup(fixture.tearDown)
        fixture.node.landing_gear = self.control
        with patch('task_trajectory_library.catalog', return_value=([], [])):
            window = Window(fixture.node)
            window.library_scan_thread.join(2.)
            window.poll_library()
        window.timer.stop(); self.addCleanup(window.deleteLater)
        self.assertEqual(window.tabs.tabText(window.tabs.indexOf(window.landing_gear_panel)), '起落架杆')
        window.tabs.setCurrentWidget(window.landing_gear_panel)
        window.show(); window.refresh()
        for width, height in ((620, 720), (1100, 1000)):
            window.resize(width, height); QtWidgets.QApplication.processEvents()
            scroll = window.landing_gear_panel.findChild(QtWidgets.QScrollArea)
            scroll.verticalScrollBar().setValue(scroll.verticalScrollBar().maximum())
            QtWidgets.QApplication.processEvents()
            self.assertEqual(scroll.horizontalScrollBar().maximum(), 0)
            self.assertTrue(window.grab().save(str(Path('/tmp')/f'landing_gear_console_{width}x{height}.png')))
        fixture.node.execute.assert_not_called()
        fixture.node.arm.assert_not_called()


if __name__ == '__main__':unittest.main()
