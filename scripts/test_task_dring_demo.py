"""Recorded D-ring workflows, with fake hardware and no robot command clients."""
import copy
import json
from pathlib import Path
from types import SimpleNamespace as S
import tempfile
import threading
import unittest
from unittest.mock import Mock,patch
import xml.etree.ElementTree as ET

import numpy as np

import task_dring_teach_data as data
from task_dring_demo_control import DringDemoControl
from task_hand_control import model_joints
from task_dring_demo_hand import replay
from task_planning_config import PlanningParameters
from task_session import ARM_NAMES,Session
from task_trajectory import coefficients,time_seconds,stretch_to_limits,task_motion_limits,task_jerk_limits


def fixture(hand=False):
    names=data.moving('left');all_names=ARM_NAMES+('waist_yaw_joint',)
    robot=ET.fromstring('<robot>'+''.join(f'<joint name="{n}"><limit lower="-2" upper="2" velocity=".4"/></joint>'
        for n in all_names)+'</robot>')
    model=S(robot=robot,limits=dict.fromkeys(all_names,(-2.,2.)))
    t=np.linspace(0.,4.,401);u=t/4.;blend=u**3*(10-15*u+6*u*u)
    rows=np.zeros((len(t),len(names)));rows[:,0]=.12*blend
    trace=dict(raw=dict(positions=rows.tolist(),time_s=t.tolist(),stamp_ns=[1_000_000_000+round(v*1e9) for v in t]),hand=None)
    if hand:trace['hand']=dict(time_s=[float(v) for v in np.linspace(-.1,4.1,43)],
        q=[[1.,1.,1.-.1*v,1.-.1*v,1.,1.] for v in np.linspace(0.,4.,43)])
    q=dict.fromkeys(all_names,0.)
    for side in ('left','right'):q.update(model_joints(side,[1.]*6))
    config=dict(schema=data.SCHEMA,side='left',ready=dict(joints={n:0. for n in names},hand_q=None),
                traces={'left':trace,'right':copy.deepcopy(trace)})
    return model,q,config


class DemoDataTests(unittest.TestCase):
    def test_full_rate_fit_avoids_near_coincident_timing_grids(self):
        from task_trajectory_speed import scale_arm_segments
        model,_,config=fixture();trace=config['traces']['left'];raw=trace['raw']
        t=np.linspace(0.,31.076443808,31078);u=t/t[-1]
        q=np.zeros((len(t),len(data.moving('left'))))
        q[:,0]=.03*np.sin(40*np.pi*u)*np.sin(np.pi*u)**2
        raw.update(time_s=t.tolist(),positions=q.tolist(),sample_policy='all_received',
                   stamp_ns=[1_000_000_000+round(v*1e9) for v in t])
        tr,report=data.fit_trace(trace,model,'left')
        intervals=np.diff([time_seconds(p) for p in tr.points])
        self.assertGreater(np.min(intervals),.01)
        self.assertLessEqual(np.max(intervals),.100000001)
        self.assertLessEqual(report['fit_error_rad'],.001)
        for multiplier in (1.,24.):
            scaled,_=scale_arm_segments([dict(trajectory=tr)],multiplier,model,policy='uniform')
            self.assertGreater(scaled[0]['duration'],0.)

    def test_fit_retains_recorded_duration_and_stationary_boundaries(self):
        model,q,config=fixture();trace=config['traces']['left']
        tr,report=data.fit_trace(trace,model,'left')
        self.assertAlmostEqual(time_seconds(tr.points[-1]),4.)
        self.assertLessEqual(report['fit_error_rad'],.005)
        for p in (tr.points[0],tr.points[-1]):
            self.assertTrue(all(abs(v)<1e-10 for v in (*p.velocities,*p.accelerations)))
        self.assertAlmostEqual(tr.points[-1].positions[0],.12)

    def test_both_direction_slots_replay_with_collision_free_prefix_code(self):
        model,q,config=fixture(True);q[ARM_NAMES[0]]=.3
        for slot in data.SLOTS:config['traces'].setdefault(slot,copy.deepcopy(config['traces']['left']))
        original=copy.deepcopy(config)
        with patch('task_motion_planner.TaskPlanner.validate',side_effect=AssertionError('Collision service must not run')):
            for slot in data.SLOTS:
                plan=data.build(model,config,q,slot,PlanningParameters(),.7,.7,1.)
                self.assertEqual(plan['kind'],'dring_demo')
                self.assertEqual(plan['demo']['transition_count'],1)
                prefix=plan['segments'][0]
                self.assertFalse(prefix['collision_checked'])
                self.assertEqual(prefix['start'],q)
                self.assertAlmostEqual(prefix['end'][ARM_NAMES[0]],0.)
                self.assertEqual(plan['segments'][1]['start'],prefix['end'])
                stage=plan['segments'][-1]
                self.assertAlmostEqual(stage['duration'],stage['recorded_hand']['time_s'][-1])
                self.assertAlmostEqual(stage['duration'],4.)
                preview=data.preview(plan)
                self.assertAlmostEqual(time_seconds(preview.points[-1]),sum(s['duration'] for s in plan['segments']))
                index=preview.joint_names.index('left_index_1_joint')
                self.assertAlmostEqual(preview.points[-1].positions[index],stage['end']['left_index_1_joint'])
        self.assertEqual(config,original)

    def test_arm_and_hand_share_scaling_but_prefix_does_not(self):
        model,q,config=fixture(True);q[ARM_NAMES[0]]=.3
        slow=data.build(model,config,q,'left',PlanningParameters(),.7,.7,1.)
        fast=data.build(model,config,q,'left',PlanningParameters(),.7,.7,2.)
        self.assertEqual(slow['segments'][0]['trajectory'],fast['segments'][0]['trajectory'])
        a,b=slow['segments'][-1],fast['segments'][-1]
        self.assertAlmostEqual(a['duration'],2*b['duration'])
        np.testing.assert_allclose(a['recorded_hand']['time_s'],np.array(b['recorded_hand']['time_s'])*2)
        self.assertEqual(a['recorded_hand']['q'],b['recorded_hand']['q'])
        vmax,amax=task_motion_limits(b['trajectory'].joint_names,.7,.7,PlanningParameters())
        self.assertEqual(stretch_to_limits(copy.deepcopy(b['trajectory']),vmax,amax,task_jerk_limits(b['trajectory'].joint_names)),1.)

    def test_ready_motion_is_explicit_and_has_no_collision_checker(self):
        model,q,config=fixture();q[ARM_NAMES[0]]=.2
        plan=data.build(model,config,q,'ready',PlanningParameters(),.7,.7,24.)
        self.assertEqual(len(plan['segments']),1)
        self.assertFalse(plan['segments'][0]['collision_checked'])
        self.assertAlmostEqual(plan['segments'][0]['end'][ARM_NAMES[0]],0.)

    def test_gaps_and_uncovered_finger_recordings_rejected(self):
        model,q,config=fixture(True);trace=config['traces']['left']
        bad=copy.deepcopy(trace);bad['raw']['time_s'][2]=.2
        with self.assertRaises(ValueError):data.arrays(bad,model,'left')
        bad=copy.deepcopy(trace);bad['hand']['time_s'][0]=.01
        with self.assertRaises(ValueError):data.arrays(bad,model,'left')

    def test_both_arm_cycles_include_preparation_ready_recording_and_return_without_collision(self):
        for side in ('left','right'):
            model,q,config=fixture();names=data.moving(side)
            config['side']=side;config['ready']['joints']=dict.fromkeys(names,0.)
            preparation=dict.fromkeys(names,0.);preparation[names[0]]=.3;q[names[0]]=.5
            with patch('task_motion_planner.TaskPlanner.validate',side_effect=AssertionError('no collision calls')):
                plan=data.build(model,config,q,'left',PlanningParameters(),.7,.7,1.,
                                preparation=preparation,full_cycle=True)
            self.assertTrue(plan['demo']['full_cycle'])
            self.assertFalse(plan['demo']['collision_checked'])
            self.assertEqual([s.get('role') for s in plan['segments']],
                             ['demo_preparation','demo_ready',None,'demo_return'])
            for a,b in zip(plan['segments'],plan['segments'][1:]):self.assertEqual(a['end'],b['start'])
            for name in names:self.assertAlmostEqual(plan['segments'][-1]['end'][name],preparation[name])
            passive=data.ARMS['right' if side=='left' else 'left']
            for segment in plan['segments']:
                for name in passive:self.assertEqual(segment['end'][name],q[name])
            preview=data.preview(plan)
            self.assertAlmostEqual(time_seconds(preview.points[-1]),sum(s['duration'] for s in plan['segments']))

    def test_preparation_return_works_before_any_teaching(self):
        model,q,config=fixture();config['ready']=None;config['traces']={}
        preparation=dict.fromkeys(data.moving('left'),0.);preparation['left_elbow_joint']=.2
        plan=data.build(model,config,q,'preparation',PlanningParameters(),.7,.7,1.,preparation=preparation)
        self.assertEqual(len(plan['segments']),1)
        self.assertAlmostEqual(plan['segments'][0]['end']['left_elbow_joint'],.2)


class DemoPersistenceTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name);model,q,config=fixture()
        robot=self.root/'robot.urdf';robot.write_text(ET.tostring(model.robot,encoding='unicode'))
        self.node=S(args=S(robot_urdf=robot),model=model,lock=threading.RLock(),session=S(phase='ARRIVED_HOLD',invalidate=Mock()),
                    display_pub=Mock(),demo_preview_pub=Mock(),run_dir=self.root/'session',worker=None,shutdown_requested=False)
        self.control=DringDemoControl(self.node,self.root/'demo.json')
        config['identity']=self.control.identity();self.config=config

    def test_ten_slots_round_trip_and_replace_only_selected_slot(self):
        config=copy.deepcopy(self.config)
        for slot in data.SLOTS:config['traces'][slot]=copy.deepcopy(self.config['traces']['left'])
        self.assertEqual(len(config['traces']),10)
        self.control.persist(config)
        replacement=copy.deepcopy(config);replacement['traces']['right_3']['recorded_at']='new'
        self.control.persist(replacement)
        restored=DringDemoControl(self.node,self.control.store)
        self.assertEqual(restored.config,replacement)
        for slot in data.SLOTS:
            if slot!='right_3':self.assertEqual(restored.config['traces'][slot],config['traces'][slot])

    def test_relabel_preserves_trace_and_backup(self):
        from relabel_dring_recording import relabel
        config=copy.deepcopy(self.config);config['traces'].pop('right')
        config['traces']['left']['recorded_at']='expected'
        path=self.root/'rename.json';path.write_text(json.dumps(config))
        original=path.read_bytes();relabel([path],'expected')
        migrated=json.loads(path.read_text())
        self.assertEqual(migrated['traces'],dict(right=config['traces']['left']))
        self.assertEqual(next(self.root.glob('rename.json.before_relabel_*')).read_bytes(),original)
        with self.assertRaises(ValueError):relabel([path],'expected')

    def test_import_export_and_restart_restore_data_not_authorization(self):
        self.control.persist(self.config)
        self.control.save(self.root/'export.json')
        other=DringDemoControl(self.node,self.root/'demo.json')
        self.assertEqual(other.config,self.config)
        self.assertIsNone(other.recording)
        self.assertEqual(self.node.session.phase,'ARRIVED_HOLD')
        self.control.load(self.root/'export.json')
        self.assertEqual(self.control.config,self.config)

    def test_invalid_import_keeps_last_good_recording(self):
        self.control.persist(self.config);old=copy.deepcopy(self.control.config)
        bad=copy.deepcopy(old);bad['identity']={}
        (self.root/'bad.json').write_text(json.dumps(bad))
        with self.assertRaises(ValueError):self.control.load(self.root/'bad.json')
        self.assertEqual(self.control.config,old)
        self.assertEqual(json.loads(self.control.store.read_text()),old)

    def test_side_switch_and_restart_keep_both_arm_bundles(self):
        c=self.control;c.persist(self.config);left=copy.deepcopy(c.config)
        c.select_side('right')
        self.assertEqual(c.config['side'],'right');self.assertIsNone(c.config['ready'])
        right=copy.deepcopy(c.config);right['ready']=dict(joints=dict.fromkeys(data.moving('right'),.1),hand_q=None)
        c.persist(right);c.select_side('left');self.assertEqual(c.config,left)
        restored=DringDemoControl(self.node,c.store);restored.select_side('right')
        self.assertEqual(restored.config,right)

    def test_cancel_only_sets_recording_event_and_never_overwrites_data(self):
        self.control.persist(self.config);old=copy.deepcopy(self.control.config)
        self.control.recording=dict(cancel=threading.Event(),error=None)
        self.control.cancel_trace()
        self.assertTrue(self.control.recording['cancel'].is_set())
        self.assertEqual(self.control.config,old)
        with self.assertRaises(RuntimeError):self.control.new('right')

    def test_capture_worker_saves_only_selected_slot_and_cancel_preserves_previous(self):
        self.control.persist(self.config)
        n=self.node;c=self.control
        _,q,_=fixture()
        n.gateway=dict(phase='TASK_ADMITTANCE',task_stop_epoch='1')
        n.session.generation=1;n.session.measured=Mock(return_value=q)
        n.check=Mock();n.launch_work=lambda f:f();n.hand_control=False
        raw=copy.deepcopy(self.config['traces']['left']['raw'])
        raw['received_monotonic_start']=100.
        receiver=Mock(directory=self.root/'raw')
        def ready():
            c.recording['finish'].set()
            return dict(ready=True)
        receiver.check.side_effect=ready;receiver.finish.return_value=raw
        old_right=copy.deepcopy(c.config['traces']['right'])
        with patch.object(c,'gateway_guard'),patch('task_dring_demo_control.DirectCapture',return_value=receiver):
            c.start_trace('left',False)
        self.assertIsNone(c.recording);receiver.close.assert_called_once()
        self.assertEqual(c.config['traces']['right'],old_right)
        self.assertEqual(c.config['traces']['left']['raw'],raw)
        previous=copy.deepcopy(c.config)
        def cancel():
            c.cancel_trace('cancelled')
            return dict(ready=True)
        receiver.reset_mock();receiver.check.side_effect=cancel
        receiver.finish.side_effect=lambda check:check()
        with patch.object(c,'gateway_guard'),patch('task_dring_demo_control.DirectCapture',return_value=receiver):
            with self.assertRaisesRegex(RuntimeError,'cancelled'):c.start_trace('left',False)
        self.assertEqual(c.config,previous);self.assertIsNone(c.recording)
        receiver.close.assert_called_once()

    def test_panel_offscreen_at_two_sizes(self):
        from PyQt5 import QtWidgets
        from task_dring_demo_panel import DringDemoPanel
        app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        self.app=app
        self.control.persist(self.config)
        self.node.demo=self.control;self.node.session.plan=None
        window=S(node=self.node,clear=QtWidgets.QCheckBox(),invoke=lambda fn:fn())
        panel=DringDemoPanel(window);self.addCleanup(panel.close)
        self.assertEqual(panel.table.rowCount(),10)
        for i,slot in enumerate(data.SLOTS):
            panel.table.selectRow(i);self.assertEqual(panel.slot(),slot)
            with patch.object(panel,'ask',return_value=True),patch.object(self.control,'start_trace') as start:
                panel.record();start.assert_called_once_with(slot,False)
        for width,height in ((620,720),(1100,1000)):
            panel.resize(width,height);panel.refresh(False);panel.show();app.processEvents()
            scroll=panel.findChild(QtWidgets.QScrollArea)
            self.assertEqual(scroll.horizontalScrollBar().maximum(),0)
            self.assertTrue(panel.grab().save(f'/tmp/dring_demo_panel_{width}.png'))
        self.control.recording=dict(phase='recording',finish=threading.Event(),cancel=threading.Event())
        panel.refresh(True)
        self.assertTrue(panel.finish.isEnabled());self.assertTrue(panel.cancel.isEnabled())
        self.assertFalse(panel.start.isEnabled())
        from PyQt5 import QtCore, QtTest
        for button in (panel.finish,panel.cancel):
            clicked=QtTest.QSignalSpy(button.clicked)
            QtTest.QTest.mousePress(button,QtCore.Qt.LeftButton)
            panel.refresh(True)
            self.assertTrue(button.isDown())
            with patch.object(window,'invoke'):
                QtTest.QTest.mouseRelease(button,QtCore.Qt.LeftButton)
            self.assertEqual(len(clicked),1)

    def test_confirmed_move_executes_only_matching_completed_generation(self):
        from PyQt5 import QtWidgets
        from task_dring_demo_panel import DringDemoPanel
        self.app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        n=self.node;n.demo=self.control;n.session.plan=None;n.session.generation=3;n.execute=Mock()
        window=S(node=n,clear=QtWidgets.QCheckBox(),invoke=lambda fn:fn())
        panel=DringDemoPanel(window);self.addCleanup(panel.close)
        with patch.object(panel,'ask',return_value=True),patch.object(self.control,'prepare') as prepare:
            panel.move('ready')
            prepare.assert_called_once_with('ready',1.,.7,.7,True)
        panel.refresh(True);n.execute.assert_not_called()
        n.session.plan=dict(kind='dring_demo',demo=dict(operation='ready'),segments=[])
        n.session.phase='PLAN_READY';panel.refresh(False)
        n.execute.assert_called_once_with(scene_confirmed=True)
        panel.pending_move=(3,'ready');n.session.generation=4;panel.refresh(False)
        self.assertIsNone(panel.pending_move);self.assertEqual(n.execute.call_count,1)

    def test_finish_click_notifies_receiver_before_waiting_for_worker(self):
        rec=dict(phase='recording',receiver=Mock(),finish=threading.Event())
        self.control.recording=rec
        rec['receiver'].request_finish.side_effect=lambda at:self.assertFalse(rec['finish'].is_set())
        self.control.finish_trace()
        rec['receiver'].request_finish.assert_called_once()
        self.assertTrue(rec['finish'].is_set());self.assertEqual(rec['phase'],'saving')

    def test_running_uses_gateway_watchdog_not_gui_sample_age(self):
        from task_session import FeedbackPending
        n=self.node;c=self.control
        n.gateway=dict(task_stop_epoch='3',task_stop_active='false',lowstate_timeout_ms='100',
                       service_state_fresh='true',phase='RUNNING')
        n.gateway_received=10.
        n.session.measured=Mock(side_effect=FeedbackPending('GUI sample age 0.166s'))
        plan=dict(demo=dict(revision=c.revision,stop_epoch='3',waist_control_mode=None,waist_control_revision=None))
        with patch.object(c,'gateway_guard') as guard,patch('task_dring_demo_control.time.monotonic',return_value=10.1):
            c.execution_guard(plan,running=True)
            guard.assert_not_called()
            n.session.measured.assert_not_called()
            n.gateway['task_stop_active']='true'
            with self.assertRaisesRegex(RuntimeError,'STOP'):c.execution_guard(plan,running=True)
            n.gateway['task_stop_active']='false'
            n.gateway['lowstate_timeout_ms']='500'
            c.execution_guard(plan,running=True)
            n.gateway['lowstate_timeout_ms']='100';n.gateway['service_state_fresh']='false'
            n.gateway_received=-100.
            c.execution_guard(plan,running=True)
            guard.assert_not_called()

    def test_actual_action_wait_survives_gui_gap_but_boundary_waits_for_fresh_state(self):
        from real_task_app import TaskNode
        from task_session import FeedbackPending
        n=self.node;c=self.control;clock=[10.];_,q,_=fixture()
        n.args.hardware=True;n.args.disconnected=False;n.closing=threading.Event();n.demo=c
        n.session=Session(lambda:clock[0])
        for _ in range(8):clock[0]+=.1;n.session.update(q)
        n.session.capture_home();ticket=n.session.begin('dring_demo','left',.7,.7)
        plan=dict(kind='dring_demo',start=q,segments=[{}],demo=dict(revision=c.revision,stop_epoch='3',
                  waist_control_mode=None,waist_control_revision=None))
        n.session.ready(ticket,plan);n.session.phase='EXECUTING'
        n.gateway=dict(phase='RUNNING',armed='true',hardware_output_available='true',task_application_enabled='true',
            upper_body_state_safe='true',service_state_ready='true',service_state_fresh='true',mode_machine='6',mode_pr='0',
            max_measured_velocity_rad_s='0',torque_overload_active='false',command_overload_active='false',
            velocity_recovery_active='false',temperature_alarm_active='false',feedforward_saturated='false',
            service_conflict='false',waist_mode_switching='false',task_waist_trajectory_supported='true',
            waist_guard_lower_rad='-1.98',waist_guard_upper_rad='1.98',task_stop_epoch='3',task_stop_active='false',
            lowstate_timeout_ms='100')
        n.gateway_received=clock[0];n.check=lambda t=None:TaskNode.check(n,t)
        began=clock[0];arrived=object()
        future=S(done=lambda:clock[0]>=began+.3,result=lambda:arrived,
                 add_done_callback=lambda callback:None,cancelled=lambda:False)
        restore=[False]
        def advance(dt):
            clock[0]+=dt;n.gateway_received=clock[0]
            if restore[0]:n.session.update(q)
        with patch('task_dring_demo_control.time.monotonic',lambda:clock[0]),patch('task_future.time.sleep',advance),\
                patch('task_future.threading.Event',return_value=S(wait=advance,set=lambda:None)):
            self.assertIs(TaskNode.wait(n,future,ticket),arrived)
            with self.assertRaises(FeedbackPending):n.session.measured()
            n.gateway['task_stop_active']='true'
            with self.assertRaises(RuntimeError):n.check(ticket)
            n.gateway['task_stop_active']='false';n.gateway['phase']='ARMED_HOLD'
            for age in (.49,.51,.6,1.,1.51,10.):
                n.gateway_received=clock[0]-age
                n.check(ticket)
            n.gateway_received=clock[0]-1.51
            with self.assertRaisesRegex(RuntimeError,'网关反馈已过期'):c.execution_guard(plan)
            n.gateway_received=clock[0]
            with self.assertRaisesRegex(TimeoutError,'feedback/settling'):
                TaskNode.wait_stationary(n,ticket,expected=q,timeout=.1)
            before=clock[0];restore[0]=True
            actual=TaskNode.wait_stationary(n,ticket,expected=q,timeout=2.)
            self.assertEqual(actual,q);self.assertGreater(clock[0]-before,.3)

    def test_index_middle_is_explicit_and_commands_only_selected_hand(self):
        _,q,_=fixture();n=self.node;c=self.control
        n.hand_control=True;n.hand=Mock();n.check=Mock();n.stop=Mock();n.cache_hand_state=Mock()
        n.gateway=dict(task_stop_epoch='3');n.session.generation=1
        n.session.measured=Mock(return_value=q);n.launch_work=lambda work:work()
        n.hand.transition.side_effect=lambda side,target,check,**kw:check()
        with patch.object(c,'gateway_guard'):c.index_middle()
        self.assertEqual(n.hand.transition.call_args.args[:2],('left',(0.,0.,1.,1.,0.,0.)))
        n.stop.assert_not_called();self.assertIsNone(c.config['ready'])

    def test_arm_only_import_preserves_raw_hands_but_does_not_replay_them(self):
        from real_task_app import TaskNode
        _,q,config=fixture(True);n=self.node;c=self.control
        config['identity']=c.identity();c.persist(config);original=copy.deepcopy(c.config)
        clock=[10.];n.session=Session(lambda:clock[0])
        for i in range(8):
            clock[0]+=.1;n.session.update(q)
        n.session.capture_home();n.gateway=dict(task_stop_epoch='3',task_stop_active='false')
        n.hand_control=True;n.hand=Mock();n.check=lambda ticket=None:n.session.check(ticket) if ticket else None
        n.wait_stationary=lambda *a,**kw:TaskNode.wait_stationary(n,*a,**kw)
        n.launch_work=lambda work:work()
        with patch.object(c,'gateway_guard'),patch('task_dring_demo_control.load_parameters',
                return_value=(PlanningParameters(),dict(values={}))),patch(
                'task_dring_demo_control.controller_bounded_parameters',side_effect=lambda p,r,g:(p,r)):
            c.prepare('left',1.,.7,.7,True)
        plan=n.session.plan
        self.assertTrue(plan['demo']['arm_only']);self.assertEqual(c.config,original)
        self.assertTrue(all(s['kind']=='arm' and s.get('recorded_hand') is None for s in plan['segments']))
        self.assertEqual(n.hand.method_calls,[])
        count=n.display_pub.publish.call_count;c.replay_preview()
        self.assertEqual(n.display_pub.publish.call_count,count+1)
        msg=n.display_pub.publish.call_args.args[0]
        self.assertAlmostEqual(time_seconds(msg.trajectory[0].joint_trajectory.points[-1]),
                               sum(s['duration'] for s in plan['segments']))

    def test_prepare_recovers_window_after_snapshot_without_holding_feedback_lock(self):
        from real_task_app import TaskNode
        _,q,config=fixture();n=self.node;c=self.control
        config['identity']=c.identity();c.persist(config)
        clock=[10.];n.session=Session(lambda:clock[0])
        for _ in range(8):clock[0]+=.1;n.session.update(q)
        n.session.capture_home();n.gateway=dict(task_stop_epoch='3',task_stop_active='false')
        n.hand_control=False;n.check=lambda ticket=None:n.session.check(ticket) if ticket else None
        n.wait_stationary=lambda *a,**kw:TaskNode.wait_stationary(n,*a,**kw)
        n.launch_work=lambda work:work()
        validate=c.validate
        def slow_validate(bundle):
            self.assertFalse(n.lock._is_owned())
            clock[0]+=.2;n.session.update(q)
            return validate(bundle)
        def feedback(dt):
            clock[0]+=dt;n.session.update(q)
        with patch.object(c,'gateway_guard'),patch.object(c,'validate',side_effect=slow_validate),\
                patch('task_dring_demo_control.load_parameters',return_value=(PlanningParameters(),dict(values={}))),\
                patch('task_dring_demo_control.controller_bounded_parameters',side_effect=lambda p,r,g:(p,r)),\
                patch('real_task_app.time.monotonic',lambda:clock[0]),patch('real_task_app.time.sleep',feedback):
            began=clock[0];c.prepare('left',1.,.7,.7,True)
        self.assertEqual(n.session.phase,'PLAN_READY')
        self.assertGreaterEqual(clock[0]-began,.59)

    def test_prepare_rejects_stop_during_snapshot(self):
        from real_task_app import TaskNode
        _,q,config=fixture();n=self.node;c=self.control
        config['identity']=c.identity();c.persist(config)
        clock=[10.];n.session=Session(lambda:clock[0])
        for _ in range(8):clock[0]+=.1;n.session.update(q)
        n.session.capture_home();n.gateway=dict(task_stop_epoch='3',task_stop_active='false')
        n.check=lambda ticket=None:n.session.check(ticket) if ticket else None
        n.wait_stationary=lambda *a,**kw:TaskNode.wait_stationary(n,*a,**kw)
        n.launch_work=lambda work:work()
        with patch.object(c,'gateway_guard'),patch.object(c,'validate',side_effect=lambda _:n.session.invalidate()),\
                patch('task_dring_demo_control.load_parameters',return_value=(PlanningParameters(),dict(values={}))),\
                patch('task_dring_demo_control.data.build') as build:
            with self.assertRaisesRegex(RuntimeError,'cancelled or superseded'):c.prepare('left',1.,.7,.7,True)
        build.assert_not_called();self.assertIsNone(n.session.plan)


class DemoHandTests(unittest.TestCase):
    def run_replay(self,*,failed=False,stale=False,stop=False):
        self.clock=0.;self.commands=[];target=[1.]*6
        result=S(status=6 if failed else 4,result=S(error_code=0))
        future=S(done=lambda:self.clock>=(.3 if failed else 1.),result=lambda:result)
        def check():
            if stop and self.clock>=.2:raise RuntimeError('STOP')
        def rpc(command,side,q=None,check=lambda:None):
            check();self.commands.append((self.clock,command,q))
            if q is not None:target[:]=q
        def read(side,check):
            check()
            return dict(code=0,hand=side,online=True,feedback_q=list(target),target_q=list(target),
                        error=[0]*6,temperature=[25]*6,timestamp_ms=1 if stale else 1+round(self.clock*1000))
        def sleep(seconds):self.clock+=seconds
        return replay(S(rpc=rpc,read=read),'left',dict(time_s=[0.,1.],q=[[1.]*6,[.5]*6]),
                      future,check,lambda state:None,clock=lambda:self.clock,sleep=sleep)

    def test_interpolation_and_final_hold(self):
        result,evidence=self.run_replay()
        self.assertEqual(result.status,4);self.assertTrue(evidence)
        self.assertEqual(self.commands[-1][1],'hold')
        self.assertEqual(self.commands[-2][2],[.5]*6)
        self.assertTrue(any(.6<q[0]<.9 for _,command,q in self.commands if command=='set'))

    def test_arm_failure_stops_finger_commands(self):
        with self.assertRaisesRegex(RuntimeError,'手臂回放失败'):self.run_replay(failed=True)
        self.assertLess(self.commands[-1][0],.3)

    def test_stop_propagates_without_later_commands(self):
        with self.assertRaisesRegex(RuntimeError,'STOP'):self.run_replay(stop=True)
        self.assertLess(self.commands[-1][0],.2)

    def test_stale_fingers_stop_replay(self):
        with self.assertRaisesRegex(RuntimeError,'750 ms'):self.run_replay(stale=True)


class DemoCaptureTests(unittest.TestCase):
    def test_receiver_finish_checks_stationary_window_at_click_not_later_samples(self):
        from task_rail_receiver import Recorder
        from collections import deque
        rec=Recorder.__new__(Recorder)
        rec.history=deque((10.+i*.01,{'joint':0. if i<=50 else .5}) for i in range(81))
        rec.spec=dict(motors={'joint':0});rec.last=10.8;rec.selected=10.8
        rec.latest=dict(received_monotonic=10.8);rec.done=False
        with patch('task_rail_receiver.time.monotonic',return_value=10.81):rec.finish(10.5)
        self.assertTrue(rec.done);self.assertEqual(rec.finish_at,10.5)

    def test_saved_samples_stop_at_requested_click_time(self):
        from task_rail_capture import DirectCapture
        model,q,_=fixture()
        with tempfile.TemporaryDirectory() as directory,patch('task_rail_capture.subprocess.Popen') as popen:
            popen.return_value.poll.return_value=0
            capture=DirectCapture(S(run_dir=Path(directory),model=model),'left',q,1,0,side='left')
            self.addCleanup(capture.close)
            rows=[dict(joints=q,received_monotonic=10.+i*.01,source_timestamp_ns=10**9+i*10**7) for i in range(81)]
            (capture.directory/'samples.jsonl').write_text('\n'.join(json.dumps(row) for row in rows))
            (capture.directory/'status.json').write_text(json.dumps(dict(done=True)))
            capture.request_finish(10.5);capture.request_finish(10.8)
            raw=capture.finish(lambda:None)
            self.assertEqual(len(raw['positions']),51);self.assertAlmostEqual(raw['time_s'][-1],.5)
            popen.return_value.stdin.write.assert_called_once_with('finish 10.500000000\n')

    def test_receiver_selects_active_and_passive_arm_without_commands(self):
        from task_rail_capture import DirectCapture
        model,q,_=fixture()
        with tempfile.TemporaryDirectory() as directory,patch('task_rail_capture.subprocess.Popen') as popen:
            popen.return_value.poll.return_value=0
            for side in ('left','right'):
                capture=DirectCapture(S(run_dir=Path(directory),model=model),'left',q,1,0,side=side)
                request=json.loads((capture.directory/'request.json').read_text())
                self.assertEqual(request['moving'],list(data.moving(side)))
                self.assertEqual(request['passive'],list(data.ARMS['right' if side=='left' else 'left']))
                self.assertEqual(request['source'],'/lowstate')
                capture.close()


if __name__=='__main__':unittest.main()
