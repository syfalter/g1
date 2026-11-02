"""Production execution + HandControl with timed in-memory DDS/action transports."""
import copy
import json
import threading
import time
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock, patch

from builtin_interfaces.msg import Time
from real_task_app import TaskNode
from task_hand_control import HandControl, model_feedback, model_joints
from task_session import ARMS
from task_planning_config import load_parameters
import test_task_live_test as live_fixture


class ExecutionTimingTests(unittest.TestCase):
    def test_per_joint_kp_rpc_exact_ack_and_normal_restoration(self):
        from task_planning_config import PlanningParameters
        gains=[500.,510.,520.,530.,110.,220.,230.]
        for mismatch in (False,True):
            n=S(lock=threading.RLock(),gateway=dict(task_stage_supported='true',
                task_stage_arm_kp_supported='true',task_stage_profile='normal',task_stage_revision='0',
                task_stage_switching='false',task_stop_epoch='3',task_stop_active='false',phase='ARMED_HOLD'),
                gateway_received=time.monotonic(),check=Mock(),waist_parameters=Mock())
            def wait(future,ticket,timeout):
                values={p.name:p.value for p in n.waist_parameters.call_async.call_args.args[0].parameters}
                profile=values['task_stage_profile'].string_value
                requested=list(values['task_stage_arm_kp'].double_array_value) if 'task_stage_arm_kp' in values else []
                self.assertEqual(requested,gains if profile=='withdraw' else [])
                n.gateway.update(task_stage_profile=profile,task_stage_arm='left',
                    task_stage_revision=str(int(n.gateway['task_stage_revision'])+1),
                    task_stage_kp=str(values['task_stage_kp'].double_value),
                    task_stage_kd=str(values['task_stage_kd'].double_value),task_stage_arm_kp_count=str(len(requested)))
                n.gateway.update({f'task_stage_arm_kp_{i}':str(v) for i,v in enumerate(requested)})
                if mismatch:n.gateway['task_stage_arm_kp_6']='220'
                return S(result=S(successful=True))
            n.wait=wait
            if mismatch:
                with self.assertRaisesRegex(RuntimeError,'acknowledgement mismatch: task_stage_arm_kp_6'):
                    TaskNode.apply_task_stage(n,'left','withdraw',PlanningParameters(),object(),3,arm_kp=gains)
            else:
                TaskNode.apply_task_stage(n,'left','withdraw',PlanningParameters(),object(),3,arm_kp=gains)
                TaskNode.apply_task_stage(n,'left','normal',PlanningParameters(),object(),3)
                self.assertEqual(n.gateway['task_stage_arm_kp_count'],'0')

    def test_replay_wrist_rpc_requires_exact_ack_and_restores_zero(self):
        from task_planning_config import PlanningParameters
        for mismatch in ('','task_stage_wrist_kp','task_stage_distal_wrist_kp'):
            n=S(lock=threading.RLock(),gateway=dict(task_stage_supported='true',
                task_stage_wrist_supported='true',task_stage_distal_wrist_supported='true',
                task_stage_profile='normal',task_stage_revision='0',
                task_stage_switching='false',task_stop_epoch='3',task_stop_active='false',phase='ARMED_HOLD'),
                gateway_received=time.monotonic(),check=Mock(),waist_parameters=Mock())
            def wait(future,ticket,timeout):
                request=n.waist_parameters.call_async.call_args.args[0]
                values={p.name:p.value for p in request.parameters}
                profile=values['task_stage_profile'].string_value
                wrist=values['task_stage_wrist_kp'].double_value if 'task_stage_wrist_kp' in values else 0.
                distal=values['task_stage_distal_wrist_kp'].double_value if 'task_stage_distal_wrist_kp' in values else 0.
                self.assertEqual(wrist,110. if profile=='withdraw' else 0.)
                self.assertEqual(distal,220. if profile=='withdraw' else 0.)
                n.gateway.update(task_stage_profile=profile,task_stage_arm='left',
                    task_stage_revision=str(int(n.gateway['task_stage_revision'])+1),
                    task_stage_kp=str(values['task_stage_kp'].double_value),
                    task_stage_kd=str(values['task_stage_kd'].double_value),
                    task_stage_wrist_kp=str(wrist),task_stage_distal_wrist_kp=str(distal))
                if mismatch:n.gateway[mismatch]='55'
                return S(result=S(successful=True))
            n.wait=wait
            if mismatch:
                with self.assertRaisesRegex(RuntimeError,'acknowledgement mismatch: '+mismatch):
                    TaskNode.apply_task_stage(n,'left','withdraw',PlanningParameters(),object(),3,
                                             wrist_kp=110.,distal_wrist_kp=220.)
            else:
                TaskNode.apply_task_stage(n,'left','withdraw',PlanningParameters(),object(),3,
                                         wrist_kp=110.,distal_wrist_kp=220.)
                TaskNode.apply_task_stage(n,'left','normal',PlanningParameters(),object(),3)
                self.assertEqual(n.gateway['task_stage_wrist_kp'],'0.0')
                self.assertEqual(n.gateway['task_stage_distal_wrist_kp'],'0.0')

    def test_stage_rpc_checks_epoch_and_exact_gain_acknowledgement(self):
        from dataclasses import replace
        from task_planning_config import PlanningParameters
        n=S(lock=threading.RLock(),gateway=dict(task_stage_supported='true',
            task_stage_max_kp='unbounded',task_stage_max_kd='unbounded',task_stage_waist_supported='true',
            task_waist_trajectory_supported='true',waist_control_mode='guarded',
            task_stage_waist_max_kp='unbounded',task_stage_waist_max_kd='unbounded',
            task_stage_profile='normal',task_stage_revision='0',task_stage_switching='false',
            task_stop_epoch='3',task_stop_active='false',phase='ARMED_HOLD'),
            gateway_received=time.monotonic(),check=Mock(),waist_parameters=Mock())
        params=replace(PlanningParameters(),rotate_proximal_kp=300.,rotate_proximal_kd=6.,
                       stage_waist_kp=180.,stage_waist_kd=5.)
        ticket=object()
        n.waist_parameters.wait_for_service.return_value=True
        def wait(future,ticket,timeout):
            n.gateway.update(task_stage_profile='rotate',task_stage_arm='left',
                task_stage_revision='1',task_stage_kp='300',task_stage_kd='6',
                task_stage_waist_kp='180',task_stage_waist_kd='5')
            return S(result=S(successful=True))
        n.wait=wait
        TaskNode.apply_task_stage(n,'left','rotate',params,ticket,3)
        request=n.waist_parameters.call_async.call_args.args[0]
        self.assertEqual({p.name for p in request.parameters},
            {'task_stage_profile','task_stage_arm','task_stage_kp','task_stage_kd','task_stage_stop_epoch',
             'task_stage_waist_kp','task_stage_waist_kd'})
        values={p.name:p.value for p in request.parameters}
        self.assertEqual(values['task_stage_kp'].double_value,300.)
        self.assertEqual(values['task_stage_kd'].double_value,6.)
        self.assertEqual(values['task_stage_waist_kp'].double_value,180.)
        self.assertEqual(values['task_stage_waist_kd'].double_value,5.)
        n.gateway['task_stage_profile']='rotate'
        def restore(future,ticket,timeout):
            n.gateway.update(task_stage_profile='normal',task_stage_revision='2',task_stage_kp='0',
                             task_stage_kd='0',task_stage_waist_kp='0',task_stage_waist_kd='0')
            return S(result=S(successful=True))
        n.wait=restore
        TaskNode.apply_task_stage(n,'left','normal',params,ticket,3)
        n.gateway['task_stop_epoch']='4'
        with self.assertRaisesRegex(RuntimeError,'STOP changed'):
            TaskNode.apply_task_stage(n,'left','rotate',params,ticket,3)
        n.gateway['task_stage_supported']='false'
        with self.assertRaises(RuntimeError):TaskNode.apply_task_stage(n,'left','normal',params,ticket,4)

    def test_stage_wait_retries_only_stationary_rejection_with_a_deadline(self):
        from task_planning_config import PlanningParameters
        for mode in ('settles','timeout','stop','fault','stale','missing_ack'):
            with self.subTest(mode=mode):
                now=[0.];calls=[]
                n=S(lock=threading.RLock(),gateway=dict(task_stage_supported='true',task_stage_profile='normal',
                    task_stage_revision='0',task_stage_switching='false',task_stop_epoch='3',
                    task_stop_active='false',phase='ARMED_HOLD'),gateway_received=0.,check=Mock(),
                    waist_parameters=Mock())
                def sleep(dt):
                    now[0]+=dt;n.gateway_received=now[0]
                    if mode=='stop':n.gateway['task_stop_epoch']='4'
                    if mode=='stale':n.gateway_received=now[0]-2
                def wait(future,ticket,timeout):
                    calls.append(timeout)
                    if mode=='fault':return S(result=S(successful=False,reason='Stage change rejected: torque overload'))
                    if mode in ('settles','missing_ack') and len(calls)>1:
                        n.gateway.update(task_stage_profile='rotate',task_stage_arm='left',task_stage_revision='1',
                                         task_stage_kp='80',task_stage_kd='4.5')
                        if mode=='missing_ack':n.gateway['task_stage_kp']='nan'
                        return S(result=S(successful=True))
                    return S(result=S(successful=False,reason='STAGE_WAIT_STATIONARY: measured speed=0.041140'))
                n.wait=wait
                with patch('real_task_app.time.monotonic',lambda:now[0]),patch('real_task_app.time.sleep',sleep):
                    if mode=='settles':TaskNode.apply_task_stage(n,'left','rotate',PlanningParameters(),object(),3)
                    else:
                        with self.assertRaises(RuntimeError):
                            TaskNode.apply_task_stage(n,'left','rotate',PlanningParameters(),object(),3)
                self.assertLessEqual(now[0],5.1)
                if mode in ('fault','stop','stale'):self.assertEqual(len(calls),1)
                if mode in ('settles','missing_ack'):self.assertEqual(len(calls),2)

    def setUp(self):
        self.f=live_fixture.LiveTestTests('test_exact_seven_stages_without_mutating_preview')
        self.f.setUp();self.addCleanup(self.f.doCleanups)
        self.s=self.f.session;self.physical=dict(self.f.plan['start'])
        self.hand_q={side:list(model_feedback(side,self.physical)) for side in ARMS}
        self.drop_until=0.;self.cancel_at=None;self.read_count=0;self.hold_at=None
        self.commands=[];self.arm_goals=[];self.boundary_waits=[]
        self.after_read=None;self.on_set=None;self.reject_hold=False
        gateway=dict(phase='ARMED_HOLD',task_stop_active='false',task_stop_epoch='3',armed='true',
            hardware_output_available='true',task_application_enabled='true',upper_body_state_safe='true',
            service_state_ready='true',mode_machine='6',mode_pr='0',max_measured_velocity_rad_s='0',
            torque_overload_active='false',command_overload_active='false',velocity_recovery_active='false',
            temperature_alarm_active='false',feedforward_saturated='false',service_conflict='false')
        self.n=S(session=self.s,lock=threading.RLock(),args=S(hardware=True,disconnected=False,
            dring_contact_test=True,robot_urdf=str(self.f.urdf)),gateway=gateway,gateway_received=self.f.t,
            hand_control=True,hand_states={},hand_pub=Mock(),status='',
            check=self.s.check,launch_work=lambda fn:fn())
        self.n.get_clock=lambda:S(now=lambda:S(nanoseconds=round(self.f.t*1e9),
            to_msg=lambda:Time(sec=int(self.f.t),nanosec=round(self.f.t%1*1e9))))
        self.n.saved_test_guard=lambda *a,**kw:TaskNode.saved_test_guard(self.n,*a,**kw)
        self.n.cache_hand_state=lambda *a:TaskNode.cache_hand_state(self.n,*a)
        self.n.hand=HandControl('offline-no-network',enabled=True,transport=self.transport,
                                clock=lambda:self.f.t,sleep=self.advance)
        self.n.read_hands=self.read_hands
        self.n.actions={'left':S(wait_for_server=lambda **kw:True,send_goal_async=self.send)}
        self.n.wait=self.wait
        self.n.stop=Mock(side_effect=lambda:(self.s.stop(),self.n.hand.request_stop()))
        self.s.ready(self.f.ticket,self.f.plan)

    def advance(self,seconds):
        target=self.f.t+seconds
        while self.f.t<target-1e-10:
            self.f.t=min(target,self.f.t+.01)
            self.n.gateway_received=self.f.t
            if self.hold_at is not None and self.f.t>=self.hold_at:
                self.hold_at=None;self.n.gateway['phase']='ARMED_HOLD'
                self.n.gateway['max_measured_velocity_rad_s']='0'
            if self.cancel_at is not None and self.f.t>=self.cancel_at:
                self.cancel_at=None;self.n.stop()
            if self.f.t>=self.drop_until:
                q=dict(self.physical)
                for side,record in self.n.hand_states.items():q.update(model_joints(side,record['state']['feedback_q']))
                stamp=round(self.f.t*1e9)
                self.s.update(q,source_stamp_ns=stamp,source_now_ns=stamp)

    def transport(self,command):
        op,side,_id,*values=command.split();self.commands.append((self.f.t,op,side,values))
        if op=='set':
            self.hand_q[side]=list(map(float,values))
            if self.on_set:self.on_set(self.hand_q[side])
        self.advance(.07 if op=='get' else .02)
        if op=='hold' and self.reject_hold:return dict(code=1)
        if op!='get':return dict(code=0)
        return dict(code=0,hand=side,online=True,feedback_q=list(self.hand_q[side]),
                    target_q=list(self.hand_q[side]),error=[0]*6,temperature=[30]*6,
                    timestamp_ms=round(self.f.t*1000))

    def read_hands(self,ticket):
        self.read_count+=1
        result=TaskNode.read_hands(self.n,ticket)
        if self.after_read:self.after_read(self.read_count)
        return result

    def send(self,goal):
        self.arm_goals.append((self.f.t,copy.deepcopy(goal.trajectory)))
        return S(accepted=True,get_result_async=lambda:S(trajectory=goal.trajectory))

    def wait(self,future,*args):
        if not hasattr(future,'trajectory'):return future
        tr=future.trajectory;self.physical.update(zip(tr.joint_names,tr.points[-1].positions))
        self.advance(.55)
        return S(status=4,result=S(error_code=0,error_string='offline action completed'))

    def run_task(self):
        def boundary_sleep(seconds):
            self.boundary_waits.append(seconds);self.advance(seconds)
        with patch('real_task_app.time.monotonic',lambda:self.f.t),\
                patch('real_task_app.time.sleep',boundary_sleep):
            TaskNode.execute(self.n,scene_confirmed=True,contact_confirmed=True)

    def failure(self):return json.loads((self.f.directory/'execution_failure.json').read_text())

    def events(self):
        return [json.loads(line) for p in self.f.directory.glob('execution_events_*.jsonl')
                for line in p.read_text().splitlines()]

    def test_seven_stages_with_real_hand_ramp_and_delayed_rpc(self):
        self.run_task()
        self.assertEqual(len(self.arm_goals),5)
        self.assertEqual(self.s.phase,'ARRIVED_HOLD');self.n.stop.assert_not_called()
        self.assertGreater(sum(op=='set' for _,op,_,_ in self.commands),20)
        self.assertEqual(len(json.loads((self.f.directory/'execution.json').read_text())),7)
        self.assertEqual(self.read_count,7)
        self.assertIn('D-ring left task complete (1-7)',self.n.status)
        self.assertEqual(self.boundary_waits,[])

    def test_154ms_boundary_gap_waits_then_runs_hand_without_stop(self):
        def inject(count):
            if count==2:
                self.drop_until=self.f.t+.22
                self.advance(.154)
        self.after_read=inject
        self.run_task()
        self.assertEqual(len(self.arm_goals),5);self.n.stop.assert_not_called()
        self.assertEqual(self.s.phase,'ARRIVED_HOLD')

    def test_dring_withdraw_allows_thumb_pose_deviation(self):
        original=self.n.hand.transport
        def drift(command):
            result=original(command)
            if command.startswith('get left ') and len(self.arm_goals)==3:
                result['feedback_q'][5]=.944  # 0.0448 rad, the recorded field failure.
            return result
        self.n.hand.transport=drift
        self.run_task()
        self.assertEqual(len(self.arm_goals),5)
        self.n.stop.assert_not_called()
        self.assertTrue(any(e['checkpoint']=='hand_health_checked' for e in self.events()))

    def test_dring_withdraw_still_rejects_hand_fault(self):
        original=self.n.hand.transport
        def fault(command):
            result=original(command)
            if command.startswith('get left ') and len(self.arm_goals)==3:
                result['error'][5]=1
            return result
        self.n.hand.transport=fault
        with self.assertRaises(RuntimeError):self.run_task()
        self.assertEqual(len(self.arm_goals),3)
        self.n.stop.assert_called_once()

    def test_completed_action_with_lagging_running_diagnostic_is_not_a_fault(self):
        original=self.n.wait
        def lag(future,*args):
            result=original(future,*args)
            if hasattr(future,'trajectory'):
                self.n.gateway['phase']='RUNNING';self.hold_at=self.f.t+.3
            return result
        self.n.wait=lag
        self.run_task()
        self.assertEqual(len(self.arm_goals),5);self.n.stop.assert_not_called()

    def test_action_endpoint_can_finish_settling_without_a_false_fault(self):
        original=self.n.wait
        def settle(future,*args):
            result=original(future,*args)
            if hasattr(future,'trajectory'):
                self.n.gateway['max_measured_velocity_rad_s']='0.06'
                self.hold_at=self.f.t+.2
            return result
        self.n.wait=settle
        self.run_task()
        self.assertEqual(len(self.arm_goals),5);self.n.stop.assert_not_called()

    def inject_hand_open_velocity_pulse(self, values):
        if len(self.arm_goals)==4:
            self.on_set=None
            # Withdraw hold trace on 2026-09-11: waist peak 0.174844593 rad/s,
            # while the held position stayed inside the accepted endpoint bound.
            self.n.gateway['max_measured_velocity_rad_s']='0.174844593'
            self.hold_at=self.f.t+.2

    def test_withdraw_hand_open_velocity_pulse_does_not_abort_or_pause_left_task(self):
        self.on_set=self.inject_hand_open_velocity_pulse
        self.run_task()
        self.assertEqual(len(self.arm_goals),5);self.n.stop.assert_not_called()
        self.assertEqual(self.s.phase,'ARRIVED_HOLD');self.assertEqual(self.boundary_waits,[])
        self.assertEqual(sum(op=='hold' for _,op,_,_ in self.commands),2)

    def test_withdraw_hand_open_velocity_pulse_does_not_abort_or_pause_right_task(self):
        from task_plan_archive import digest
        source=json.loads(self.f.source.read_text())
        source['sweep_direction']='right';self.f.plan['sweep_direction']='right'
        source['segments'][3]['label']='Right 120';self.f.plan['segments'][3]['label']='Right 120'
        self.f.source.write_text(json.dumps(source))
        self.f.plan['live_review']['source_plan_sha256']=digest(self.f.source)
        self.on_set=self.inject_hand_open_velocity_pulse
        self.run_task()
        self.assertEqual(len(self.arm_goals),5);self.n.stop.assert_not_called()
        self.assertIn('D-ring right task complete (1-7)',self.n.status)
        self.assertEqual(self.boundary_waits,[])

    def test_boundary_still_requires_stationary_controller(self):
        self.n.gateway['max_measured_velocity_rad_s']='0.174844593'
        with self.assertRaisesRegex(RuntimeError,'stationary'):self.run_task()
        self.assertFalse(self.arm_goals);self.assertFalse(self.commands)

    def test_hand_open_still_stops_on_controller_overload(self):
        def fault(values):
            if len(self.arm_goals)==4:
                self.on_set=None;self.n.gateway['command_overload_active']='true'
        self.on_set=fault
        with self.assertRaisesRegex(RuntimeError,'command_overload_active'):self.run_task()
        self.n.stop.assert_called_once();self.assertEqual(len(self.arm_goals),4)

    def test_hand_open_still_stops_on_measured_displacement(self):
        def move(values):
            if len(self.arm_goals)==4:
                self.on_set=None;self.physical[ARMS['left'][0]]+=.05
        self.on_set=move
        with self.assertRaisesRegex(RuntimeError,'Robot moved during hand'):self.run_task()
        self.n.stop.assert_called_once();self.assertEqual(len(self.arm_goals),4)

    def test_hand_open_still_stops_on_changed_epoch(self):
        def stop(values):
            if len(self.arm_goals)==4:
                self.on_set=None;self.n.gateway['task_stop_epoch']='4'
        self.on_set=stop
        with self.assertRaisesRegex(RuntimeError,'Stop epoch changed'):self.run_task()
        self.n.stop.assert_called_once();self.assertEqual(len(self.arm_goals),4)

    def configure_endpoint(self, tolerance=2., timeout=5.):
        path=self.f.directory/'planning.yaml'
        path.write_text(f'execution: {{endpoint_tolerance_deg: {tolerance}, endpoint_wait_seconds: {timeout}}}')
        _,self.f.plan['planning_config']=load_parameters(path)
        return path

    def test_stage_profiles_follow_segments_and_restore_after_execution(self):
        self.configure_endpoint()
        self.f.plan['planning_config']['values']['stage_control_enabled']=True
        self.f.plan['planning_config']['values'].update(rotate_proximal_kp=300.,withdraw_proximal_kp=240.,
                                                       rotate_proximal_kd=6.,withdraw_proximal_kd=6.,
                                                       stage_waist_kp=180.,stage_waist_kd=5.)
        self.n.gateway.update(task_stage_supported='true',task_stage_profile='normal',
                              task_stage_revision='0',task_stage_switching='false',
                              task_stage_max_kp='unbounded',task_stage_max_kd='unbounded',task_stage_waist_supported='true',
                              task_waist_trajectory_supported='true',waist_control_mode='guarded',
                              task_stage_waist_max_kp='unbounded',task_stage_waist_max_kd='unbounded')
        applied=[]
        def apply(node,side,profile,params,ticket,epoch):
            self.assertEqual(side,'left');self.assertEqual(epoch,3)
            applied.append(profile)
            node.gateway.update(task_stage_profile=profile,
                task_stage_revision=str(len(applied)),task_stage_switching='false')
        with patch.object(TaskNode,'apply_task_stage',apply):self.run_task()
        self.assertEqual(applied,['normal','normal','normal','rotate','withdraw','normal','normal','normal'])
        self.assertEqual(self.s.phase,'ARRIVED_HOLD')
        self.assertEqual(len(self.arm_goals),5)
        self.n.stop.assert_not_called()

    def test_stage_plan_rejects_old_gateway_before_any_commands(self):
        self.configure_endpoint()
        self.f.plan['planning_config']['values']['stage_control_enabled']=True
        with self.assertRaises(RuntimeError):self.run_task()
        self.assertFalse(self.arm_goals)
        self.assertFalse(any(op=='set' for _,op,_,_ in self.commands))

    def test_demo_reuses_withdraw_gains_only_on_recorded_segment_then_restores(self):
        self.configure_endpoint()
        plan=self.f.plan
        plan['kind']='dring_demo';plan['hardware_executable']=True
        plan['demo']=dict(arm_only=True,operation='left')
        plan['segments']=[s for s in plan['segments'] if s.get('kind')!='hand']
        plan['segments'][2]['recording_fit']={}
        plan['arm']=plan['segments'][2]['side']
        original=[copy.deepcopy(s['trajectory']) for s in plan['segments']]
        plan['planning_config']['values'].update(stage_control_enabled=True,withdraw_proximal_kp=550.,
            withdraw_proximal_kd=6.,stage_waist_kp=120.,stage_waist_kd=5.,replay_waist_kp=240.,
            demo_wrist_kp=110.,demo_distal_wrist_kp=220.)
        from task_planning_config import ARM_INTEGRAL_JOINTS
        plan['planning_config']['values'].update({f'replay_{joint}_kp':value for joint,value in
            zip(ARM_INTEGRAL_JOINTS,[550.,550.,550.,550.,110.,220.,220.])})
        self.n.hand_control=False;self.n.demo=S(execution_guard=Mock(),completed=Mock())
        self.n.gateway.update(task_stage_supported='true',task_stage_profile='normal',task_stage_revision='0',
            task_stage_switching='false',task_stage_max_kp='unbounded',task_stage_max_kd='unbounded',
            task_stage_waist_supported='true',task_waist_trajectory_supported='true',waist_control_mode='guarded',
            task_stage_waist_max_kp='unbounded',task_stage_waist_max_kd='unbounded',task_stage_wrist_supported='true',
            task_stage_distal_wrist_supported='true',task_stage_arm_kp_supported='true')
        applied=[]
        def apply(node,side,profile,params,ticket,epoch,*,wrist_kp=0.,distal_wrist_kp=0.,arm_kp=None):
            applied.append(profile)
            self.assertEqual(wrist_kp,110. if profile=='withdraw' else 0.)
            self.assertEqual(distal_wrist_kp,220. if profile=='withdraw' else 0.)
            self.assertEqual(arm_kp,[550.,550.,550.,550.,110.,220.,220.] if profile=='withdraw' else None)
            if profile=='withdraw':
                self.assertEqual((params.withdraw_proximal_kp,params.withdraw_proximal_kd,
                    params.stage_waist_kp,params.stage_waist_kd),(550.,6.,240.,5.))
            node.gateway.update(task_stage_profile=profile,task_stage_revision=str(len(applied)),task_stage_switching='false')
        with patch.object(TaskNode,'apply_task_stage',apply):self.run_task()
        self.assertEqual(applied,['normal','normal','withdraw','normal','normal','normal'])
        self.assertEqual(plan['planning_config']['values']['stage_waist_kp'],120.)
        for before,(_,after) in zip(original,self.arm_goals):self.assertEqual(before.points,after.points)
        self.assertEqual(self.s.phase,'ARRIVED_HOLD');self.n.stop.assert_not_called()

    def test_higher_stage_gains_reject_legacy_bounds_before_insert(self):
        self.configure_endpoint()
        self.f.plan['planning_config']['values'].update(stage_control_enabled=True,rotate_proximal_kp=160.,
                                                       rotate_proximal_kd=6.)
        self.n.gateway.update(task_stage_supported='true',task_stage_profile='normal')
        with self.assertRaisesRegex(RuntimeError,'running gateway limit 80'):self.run_task()
        self.assertFalse(self.arm_goals)
        self.assertFalse(any(op=='set' for _,op,_,_ in self.commands))

    def inject_endpoint_error(self, error=.032455247):
        # Measured shoulder-roll endpoint error from the 2026-09-11 Approach log.
        original=self.n.wait
        def lag(future,*args):
            result=original(future,*args)
            if hasattr(future,'trajectory') and len(self.arm_goals)==1:
                self.physical['left_shoulder_roll_joint']+=error
                self.advance(.55)
            return result
        self.n.wait=lag

    def test_measured_approach_error_with_legacy_defaults_reports_joint_and_stops(self):
        self.inject_endpoint_error()
        with self.assertRaisesRegex(TimeoutError,'left_shoulder_roll_joint.*1.860.*1.432'):
            self.run_task()
        self.assertEqual(len(self.arm_goals),1);self.n.stop.assert_called_once()
        self.assertIn('left_shoulder_roll_joint',self.failure()['message'])

    def test_configured_error_passes_endpoint_hand_transition_and_all_next_segments(self):
        path=self.configure_endpoint()
        self.inject_endpoint_error()
        # Changing the file after planning must not replace the accepted snapshot.
        path.write_text('execution: {endpoint_tolerance_deg: 0.1}')
        self.run_task()
        self.assertEqual(len(self.arm_goals),5);self.n.stop.assert_not_called()
        self.assertEqual(self.s.phase,'ARRIVED_HOLD')
        self.assertEqual(self.events()[0]['endpoint_tolerance_deg'],2.)
        self.assertEqual(self.boundary_waits,[])

    def test_configured_timeout_and_larger_error_still_stop_before_fingers(self):
        self.configure_endpoint(timeout=.4)
        self.inject_endpoint_error(.045)
        with self.assertRaisesRegex(TimeoutError,'error=2.578 deg, limit=2.000 deg'):
            self.run_task()
        complete=next(e for e in self.events() if e['checkpoint']=='arm_completed_waiting_settle')
        self.assertGreaterEqual(self.f.t-complete['monotonic_s'],.4)
        self.assertLess(self.f.t-complete['monotonic_s'],.43)
        self.assertFalse(any(op=='set' for _,op,_,_ in self.commands))
        self.assertEqual(len(self.arm_goals),1);self.n.stop.assert_called_once()

    def test_configured_tolerance_does_not_relax_first_execution_start(self):
        self.configure_endpoint()
        self.physical['left_shoulder_roll_joint']+=.032455247
        self.advance(.55)
        with self.assertRaises(RuntimeError):self.run_task()
        self.assertFalse(self.arm_goals);self.n.stop.assert_not_called()

    def test_stop_during_configured_endpoint_wait_cancels_without_next_segment(self):
        self.configure_endpoint(timeout=8.)
        self.inject_endpoint_error(.045)
        original=self.n.wait
        def cancel(future,*args):
            result=original(future,*args)
            if hasattr(future,'trajectory'):self.cancel_at=self.f.t+.15
            return result
        self.n.wait=cancel
        with self.assertRaisesRegex(RuntimeError,'cancelled'):self.run_task()
        self.assertEqual(len(self.arm_goals),1)
        self.assertFalse(any(op=='set' for _,op,_,_ in self.commands))

    def test_extended_wait_allows_late_settling_without_forced_pause(self):
        self.configure_endpoint(timeout=8.)
        self.inject_endpoint_error(.045)
        original=self.advance;started=[None]
        def settle(seconds):
            if self.boundary_waits:
                if started[0] is None:started[0]=self.f.t
                if self.f.t-started[0]>=6.:
                    self.physical['left_shoulder_roll_joint']=self.f.plan['segments'][0]['end']['left_shoulder_roll_joint']
            original(seconds)
        self.advance=settle
        self.run_task()
        self.assertEqual(len(self.arm_goals),5);self.n.stop.assert_not_called()
        self.assertGreater(sum(self.boundary_waits),6.)
        self.assertLess(sum(self.boundary_waits),8.)

    def test_failed_authorization_retains_original_plan_without_motion(self):
        with patch('real_task_app.live_test_plan',side_effect=RuntimeError('source changed')):
            with self.assertRaisesRegex(RuntimeError,'source changed'):self.run_task()
        self.assertIs(self.s.plan,self.f.plan);self.assertEqual(self.s.phase,'PLAN_READY')
        self.assertEqual(self.f.plan['kind'],'task')
        self.assertFalse(self.commands);self.assertFalse(self.arm_goals)
        self.n.stop.assert_not_called()
        self.assertFalse((self.f.directory/'live_test_authorization.json').exists())

    def inject_mid_hand_gap(self,values):
        if values[0]<.95:
            self.on_set=None
            self.drop_until=self.f.t+.25
            self.advance(.154)

    def test_mid_hand_gui_gap_does_not_pause_or_restart_the_ramp(self):
        self.on_set=self.inject_mid_hand_gap
        self.run_task();self.n.stop.assert_not_called()
        events=self.events();names=[e['checkpoint'] for e in events]
        self.assertFalse(any('paused' in name or 'resumed' in name for name in names))
        # Only each completed posture sends hold, never the GUI gap itself.
        self.assertEqual(sum(op=='hold' for _,op,_,_ in self.commands),2)
        hand_events=[e for e in events if e['segment']=='Hand dring_two_finger']
        started=next(e['monotonic_s'] for e in hand_events if e['checkpoint']=='hand_transition_start')
        ended=next(e['monotonic_s'] for e in hand_events if e['checkpoint']=='hand_hold_acknowledged')
        self.assertLess(ended-started,3.)
        self.assertEqual(len(self.arm_goals),5)
        self.assertEqual(self.boundary_waits,[])

    def test_final_hand_hold_rejection_never_sends_next_arm(self):
        self.on_set=self.inject_mid_hand_gap;self.reject_hold=True
        with self.assertRaisesRegex(RuntimeError,'rejected'):self.run_task()
        self.assertEqual(len(self.arm_goals),1);self.n.stop.assert_called_once()
        self.assertFalse(self.failure()['hand_hold_confirmed'])
        self.assertNotIn('hand_resumed_from_measured_hold',[e['checkpoint'] for e in self.events()])

    def test_stop_during_hand_gui_gap_never_replays_targets(self):
        def inject(values):
            if values[0]<.95:
                self.inject_mid_hand_gap(values)
                self.cancel_at=self.f.t+.10
        self.on_set=inject
        with self.assertRaisesRegex(RuntimeError,'cancelled'):self.run_task()
        self.assertEqual(len(self.arm_goals),1)
        self.assertNotIn('hand_resumed_from_measured_hold',[e['checkpoint'] for e in self.events()])
        self.assertTrue(self.s.stopped)
        stop_at=next(e['monotonic_s'] for e in self.events()
                     if e['checkpoint']=='hand_hold_acknowledged_after_failure')
        self.assertFalse(any(t>stop_at and op=='set' for t,op,_,_ in self.commands))

    def test_arm_displacement_during_hand_motion_stops(self):
        def move(values):
            if values[0]<.95:
                self.on_set=None
                self.physical[ARMS['left'][0]]+=.05
        self.on_set=move
        with self.assertRaisesRegex(RuntimeError,'Robot moved during hand'):self.run_task()
        self.n.stop.assert_called_once();self.assertEqual(len(self.arm_goals),1)

    def test_controller_fault_during_hand_gui_gap_stops(self):
        def fault(values):
            if values[0]<.95:
                self.inject_mid_hand_gap(values)
                self.n.gateway['upper_body_state_safe']='false'
        self.on_set=fault
        with self.assertRaisesRegex(RuntimeError,'healthy supported controller'):self.run_task()
        self.n.stop.assert_called_once();self.assertEqual(len(self.arm_goals),1)

    def test_lost_gateway_during_hand_gui_gap_stops(self):
        def lost(values):
            if values[0]<.95:
                self.on_set=None
                self.n.gateway['phase']='LATCHED_SILENT'
        self.on_set=lost
        with self.assertRaisesRegex(RuntimeError,'hold lost'):self.run_task()
        self.n.stop.assert_called_once();self.assertEqual(len(self.arm_goals),1)

    def test_persistent_boundary_loss_is_bounded_and_logs_check_site(self):
        def inject(count):
            if count==2:self.drop_until=self.f.t+100.;self.advance(.154)
        self.after_read=inject
        with self.assertRaisesRegex(TimeoutError,'after hand read'):self.run_task()
        self.assertEqual(len(self.arm_goals),1);self.n.stop.assert_called_once()
        report=self.failure()
        self.assertEqual(report['checkpoint']['segment'],'Hand dring_two_finger')
        self.assertIn('wait_stationary',report['traceback'])
        self.assertEqual(len(report['completed_evidence']),1)

    def test_actual_start_movement_still_stops_without_repositioning(self):
        def inject(count):
            if count==2:
                self.physical[ARMS['left'][0]]+=.05;self.advance(.01)
        self.after_read=inject
        with self.assertRaisesRegex(RuntimeError,'start differs'):self.run_task()
        self.assertEqual(len(self.arm_goals),1);self.n.stop.assert_called_once()


    def test_authorization_does_not_block_feedback_and_stop_cancels_commit(self):
        self.s.clock=time.monotonic
        now=time.monotonic();self.s.received=now;self.s.history=[(now-.5,dict(self.physical)),(now,dict(self.physical))]
        self.n.gateway_received=now
        entered=threading.Event();release=threading.Event();quit=threading.Event();errors=[]
        from real_task_app import live_test_plan
        def authorize(*args,**kwargs):
            entered.set()
            if not release.wait(2):raise RuntimeError('test release missing')
            return live_test_plan(*args,**kwargs)
        def launch(fn):
            def run():
                try:fn()
                except Exception as error:errors.append(error)
            self.worker=threading.Thread(target=run);self.worker.start()
        self.n.launch_work=launch
        def feedback():
            while not quit.wait(.005):
                with self.n.lock:self.s.update(self.physical)
        feeder=threading.Thread(target=feedback);feeder.start()
        try:
            with patch('real_task_app.live_test_plan',authorize):
                TaskNode.execute(self.n,scene_confirmed=True,contact_confirmed=True)
                self.assertTrue(entered.wait(1))
                self.assertEqual(self.s.phase,'EXECUTION_PREFLIGHT')
                received=self.s.received;time.sleep(.25)
                self.assertGreater(self.s.received,received+.15)
                self.assertIs(self.s.plan,self.f.plan)
                with self.n.lock:self.s.stop()
                release.set();self.worker.join(2)
                self.assertFalse(self.worker.is_alive())
            self.assertTrue(errors);self.assertIn('cancelled',str(errors[0]))
            self.assertFalse(self.arm_goals)
            self.assertFalse((self.f.directory/'live_test_authorization.json').exists())
            self.assertTrue(self.s.stopped)
        finally:
            release.set();quit.set();feeder.join(2)
            if hasattr(self,'worker'):self.worker.join(2)


if __name__=='__main__':unittest.main()
