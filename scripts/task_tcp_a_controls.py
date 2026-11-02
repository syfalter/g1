"""Explicit experiment controls executed only inside the original task owner.

This is not a controller. No ARM, trajectory, parameter or arbitrary hand target
is accepted. ROS is only the UI transport; admission and execution stay here.
"""
from collections import deque
import json
import math
import os
import time
import uuid

from task_hand_control import POSTURES
from task_session import ARM_NAMES

PROTOCOL = 'tcp_a_owner_v1'
TOPIC = '/g1/task/tcp_a_control_request'
OPERATIONS = ('index_only', 'admittance', 'hold')


class ExperimentControls:
    def __init__(self, node):
        self.node = node
        self.token = uuid.uuid4().hex
        self.seen = deque(maxlen=256)
        self.last = {}

    def status(self):
        n = self.node
        with n.lock:
            if self.last.get('state')=='accepted':
                phase=n.gateway.get('phase')
                fresh=time.monotonic()-n.gateway_received<1.5
                done=(self.last['op']=='admittance' and phase=='TASK_ADMITTANCE' and n.gateway.get('task_stop_active')=='true') or (
                    self.last['op']=='hold' and phase=='ARMED_HOLD' and n.gateway.get('task_stop_active')=='false')
                if self.last.get('include_waist') and n.gateway.get('task_waist_admittance_active')!='true':
                    done=False
                if fresh and done:
                    self.last.update(state='completed',message='网关已确认 '+phase+('，腰yaw导纳已启用' if self.last.get('include_waist') else ''))
                elif time.time()-self.last['stamp']>12.:
                    self.last.update(state='unconfirmed',message='结果未确认；检查原面板当前状态，勿盲目重试')
            return dict(protocol=PROTOCOL, owner_token=self.token, console_pid=os.getpid(),
                        stamp=time.time(), hardware=n.args.hardware and not n.args.disconnected,
                        hand_enabled=n.hand_control, generation=n.session.generation,
                        stop_epoch=int(n.gateway.get('task_stop_epoch', '-1')),
                        gateway_fresh=time.monotonic()-n.gateway_received < 1.5,
                        waist_admittance_supported=n.gateway.get('task_waist_admittance_supported')=='true',
                        waist_admittance_active=n.gateway.get('task_waist_admittance_active')=='true',
                        last=dict(self.last))

    def result(self, data, state, message):
        n = self.node
        with n.lock:
            record=dict(request_id=data.get('request_id', ''), op=data.get('op', ''),
                        include_waist=data.get('include_waist',False),
                        side=data.get('side', ''), state=state, message=message, stamp=time.time())
            # A STOP response must not be replaced by an older cancelled worker.
            if state in ('accepted','rejected') or not self.last or self.last.get('request_id')==record['request_id']:
                self.last=record
            n.status='实验A操作：'+message
            with (n.run_dir/'tcp_a_controls.jsonl').open('a') as stream:
                stream.write(json.dumps(dict(result=record, request=data), ensure_ascii=False)+'\n')

    def validate(self, data):
        n=self.node
        if not isinstance(data, dict) or data.get('protocol')!=PROTOCOL:
            raise ValueError('实验控制协议不匹配')
        request_id=data.get('request_id')
        if not isinstance(request_id,str) or not 8<=len(request_id)<=80:
            raise ValueError('缺少有效请求ID')
        if request_id in self.seen:
            raise ValueError('重复请求，不重发真机操作')
        self.seen.append(request_id)
        stamp=data.get('sent_at')
        if type(stamp) not in (int,float) or not math.isfinite(stamp) or abs(time.time()-stamp)>3.:
            raise ValueError('操作确认已过期，请重新确认')
        if data.get('owner_token')!=self.token or data.get('console_pid')!=os.getpid():
            raise ValueError('原任务控制进程已改变')
        op=data.get('op')
        if op not in OPERATIONS or data.get('side') not in ('left','right'):
            raise ValueError('只支持单手伸指、双臂导纳、当前姿态保持')
        if data.get('confirmed') is not True:
            raise ValueError('缺少本次真机操作确认')
        if not n.args.hardware or n.args.disconnected or n.shutdown_requested or n.closing.is_set():
            raise ValueError('原任务进程没有可用真机控制权')
        if time.monotonic()-n.gateway_received>1.5 or n.gateway.get('armed')!='true':
            raise ValueError('需要新鲜且已ARM的原控制器；本入口不会自动ARM')
        if data.get('generation')!=n.session.generation or data.get('stop_epoch')!=int(n.gateway.get('task_stop_epoch','-1')):
            raise ValueError('任务或STOP代次改变，请检查当前状态后重新确认')
        if op=='admittance':
            if type(data.get('include_waist',False)) is not bool:
                raise ValueError('腰导纳选择必须为布尔值')
            if data.get('include_waist'):
                if (data.get('waist_clear') is not True or
                    n.gateway.get('task_waist_admittance_supported')!='true' or
                    n.gateway.get('phase')!='ARMED_HOLD'):
                    raise ValueError('腰导纳需要明确净空确认、新版网关和静止保持')
                n.session.measured(stationary=True)
            if n.gateway.get('phase') not in ('ARMED_HOLD','RUNNING'):
                raise ValueError('控制器正在转换或已经停止；请查看原面板，不重复STOP')
            if data.get('both_arms_clear') is not True:
                raise ValueError('必须确认双臂均有支撑及活动空间')
            return
        if n.worker and n.worker.is_alive():
            raise ValueError('原任务进程仍在工作，请先等待或STOP')
        if n.session.phase in ('PLANNING','EXECUTION_PREFLIGHT','EXECUTING','SHUTTING_DOWN'):
            raise ValueError('不能与原任务并行操作')
        if op=='hold':
            if n.gateway.get('phase')!='TASK_ADMITTANCE' or n.gateway.get('task_stop_active')!='true':
                raise ValueError('先等待TASK_ADMITTANCE，再恢复当前姿态保持')
            if data.get('no_contact') is not True:
                raise ValueError('恢复保持前必须离开参考点，避免锁住接触力')
        else:
            if data.get('sample_count')!=0 or data.get('no_contact') is not True:
                raise ValueError('伸指仅限空会话且无接触，已有样本请新建会话')
            if not n.hand_control:
                raise ValueError('原任务进程未启用手部DDS')
            n.saved_test_guard()
            n.session.measured(stationary=True)

    def receive(self, msg):
        data={}
        try:
            if len(msg.data)>4096:
                raise ValueError('请求过长')
            data=json.loads(msg.data)
            with self.node.lock:
                self.validate(data)
                self.result(data,'accepted','请求已接收，等待实际执行/网关状态；尚不代表完成')
                if data['op']=='admittance':
                    if data.get('include_waist'):self.node.stop(include_waist=True)
                    else:self.node.stop()
                elif data['op']=='hold':
                    self.node.recovery(.1,.1)  # Existing method only captures current hold; no trajectory.
                else:
                    self.node.launch_work(lambda:self.index_only(data))
        except Exception as error:
            if not isinstance(data,dict):data={}
            self.result(data,'rejected',str(error))

    def index_only(self, data):
        n=self.node; side=data['side']
        try:
            with n.lock:
                # Recheck on the worker before changing anything; no queued late motion.
                if abs(time.time()-data['sent_at'])>3. or n.session.generation!=data['generation']:
                    raise RuntimeError('执行前确认或任务代次已过期')
                n.saved_test_guard()
                baseline=n.session.measured(stationary=True)
                epoch=int(n.gateway['task_stop_epoch'])
                n.session.invalidate()
                n.session.phase='HOLD_FOR_REPLAN'
                generation=n.session.generation
                n.display_pub.publish(n.empty_display())
            def guard():
                n.check()
                with n.lock:
                    if n.session.generation!=generation or int(n.gateway.get('task_stop_epoch','-1'))!=epoch:
                        raise RuntimeError('手型设置被STOP或新任务取消')
                    n.saved_test_guard(stationary=False)
                    measured=n.session.measured()
                    watched=ARM_NAMES+(('waist_yaw_joint',) if 'waist_yaw_joint' in baseline else ())
                    if any(abs(measured[j]-baseline[j])>.025 for j in watched):
                        raise RuntimeError('伸指期间腰臂移动，已中止')
            n.hand.begin_motion(guard)
            evidence=n.hand.transition(side,POSTURES['index_only'],guard,duration=1.,
                on_feedback=lambda state:n.cache_hand_state(side,state))
            self.result(data,'completed',side+'手伸指反馈到位并保持；请人工确认指尖形状，再进入导纳')
            with (n.run_dir/('tcp_a_hand_'+str(time.time_ns())+'.json')).open('w') as stream:
                json.dump(dict(side=side, feedback=evidence, phalanx_geometry_measured=False),stream)
        except Exception as error:
            self.result(data,'failed',str(error)+'；未自动重试，请检查原任务面板和手部状态')
