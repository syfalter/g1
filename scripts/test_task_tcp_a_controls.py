"""Owner-bridge tests use memory fakes only, never ROS or physical hardware."""
import json
import os
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock
import uuid

from task_tcp_a_controls import ExperimentControls, PROTOCOL
from task_session import ARM_NAMES


class OwnerTests(unittest.TestCase):
    def setUp(self):
        self.directory=tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.q={name:0. for name in ARM_NAMES+('waist_yaw_joint',)}
        self.session=SimpleNamespace(generation=4, phase='LIVE_READY', measured=Mock(side_effect=lambda **kw:dict(self.q)))
        self.session.invalidate=lambda:setattr(self.session,'generation',self.session.generation+1)
        self.n=SimpleNamespace(lock=threading.RLock(), args=SimpleNamespace(hardware=True,disconnected=False),
            hand_control=True, gateway=dict(armed='true', phase='ARMED_HOLD',task_stop_epoch='3',task_stop_active='false'),
            gateway_received=time.monotonic(),session=self.session,worker=None,closing=threading.Event(),
            shutdown_requested=False,run_dir=Path(self.directory.name),status='',
            hand=SimpleNamespace(begin_motion=Mock(side_effect=lambda guard:guard()),transition=Mock()),
            saved_test_guard=Mock(),stop=Mock(),recovery=Mock(),check=Mock(),cache_hand_state=Mock(),
            display_pub=SimpleNamespace(publish=Mock()),empty_display=Mock(return_value=None))
        self.jobs=[];self.n.launch_work=self.jobs.append
        self.bridge=ExperimentControls(self.n)
        def transition(side,target,guard,**kwargs):
            guard();kwargs['on_feedback']({'feedback_q':list(target)});return [{'feedback_q':list(target)}]
        self.n.hand.transition.side_effect=transition

    def data(self,op='index_only',**changes):
        return dict(protocol=PROTOCOL,owner_token=self.bridge.token,console_pid=os.getpid(),generation=4,
            stop_epoch=3,op=op,side='left',request_id=uuid.uuid4().hex,sent_at=time.time(),confirmed=True,
            no_contact=True,both_arms_clear=True,sample_count=0,**changes)

    def send(self,data):
        self.bridge.receive(SimpleNamespace(data=json.dumps(data)))

    def test_start_is_nonactuating(self):
        self.bridge.status();self.assertFalse(self.jobs);self.n.stop.assert_not_called()
        self.n.hand.begin_motion.assert_not_called();self.n.hand.transition.assert_not_called()

    def test_single_selected_hand_uses_existing_transition(self):
        self.send(self.data());self.assertEqual(len(self.jobs),1);self.jobs.pop()()
        self.assertEqual(self.bridge.last['state'],'completed')
        args=self.n.hand.transition.call_args.args
        self.assertEqual(args[0],'left');self.assertEqual(args[1],(0.,0.,0.,1.,0.,0.))
        self.assertEqual(self.session.generation,5);self.n.stop.assert_not_called()

    def test_right_only(self):
        data=self.data();data['side']='right';self.send(data);self.jobs.pop()()
        self.assertEqual(self.n.hand.transition.call_args.args[0],'right')

    def test_duplicate_not_replayed(self):
        data=self.data();self.send(data);self.send(data);self.assertEqual(len(self.jobs),1)

    def test_bad_confirmation_or_identity_rejected(self):
        for key,value in [('confirmed',False),('no_contact',False),('sample_count',1),('owner_token','wrong'),
                          ('generation',3),('stop_epoch',2),('sent_at',time.time()-4),('sent_at',float('nan')),
                          ('side','both'),('op','arm')]:
            with self.subTest(key=key):
                data=self.data();data[key]=value;self.send(data)
                self.assertEqual(self.bridge.last['state'],'rejected');self.assertFalse(self.jobs)

    def test_busy_hand_rejected_but_stop_can_interrupt(self):
        self.n.worker=SimpleNamespace(is_alive=lambda:True)
        self.send(self.data());self.assertFalse(self.jobs)
        self.send(self.data('admittance'));self.n.stop.assert_called_once()

    def test_no_hardware_no_arm_stale_shutdown_rejected(self):
        for attribute,value in [('args',SimpleNamespace(hardware=False,disconnected=False)),
                                ('gateway_received',time.monotonic()-3),('shutdown_requested',True)]:
            old=getattr(self.n,attribute);setattr(self.n,attribute,value);self.send(self.data())
            self.assertEqual(self.bridge.last['state'],'rejected');setattr(self.n,attribute,old)
        self.n.gateway['armed']='false';self.send(self.data());self.assertFalse(self.jobs)

    def test_stop_confirmation_and_final_gateway(self):
        data=self.data('admittance');data['both_arms_clear']=False;self.send(data);self.n.stop.assert_not_called()
        self.send(self.data('admittance'));self.n.stop.assert_called_once()
        self.assertEqual(self.bridge.last['state'],'accepted')
        self.n.gateway.update(phase='TASK_ADMITTANCE',task_stop_active='true')
        self.assertEqual(self.bridge.status()['last']['state'],'completed')

    def test_recovery_reuses_current_hold_only(self):
        self.send(self.data('hold'));self.n.recovery.assert_not_called()
        self.n.gateway.update(phase='TASK_ADMITTANCE',task_stop_active='true')
        self.send(self.data('hold'));self.n.recovery.assert_called_once_with(.1,.1)
        self.n.hand.transition.assert_not_called()

    def test_waist_requires_confirmation_support_and_hold(self):
        self.send(self.data('admittance',include_waist=True,waist_clear=True))
        self.n.stop.assert_not_called()
        self.n.gateway['task_waist_admittance_supported']='true'
        self.send(self.data('admittance',include_waist=True,waist_clear=False))
        self.n.stop.assert_not_called()
        self.send(self.data('admittance',include_waist=True,waist_clear=True))
        self.n.stop.assert_called_once_with(include_waist=True)
        self.n.gateway.update(phase='TASK_ADMITTANCE',task_stop_active='true')
        self.assertEqual(self.bridge.status()['last']['state'],'accepted')
        self.n.gateway['task_waist_admittance_active']='true'
        self.assertEqual(self.bridge.status()['last']['state'],'completed')

    def test_stop_generation_cancels_deferred_hand(self):
        self.send(self.data());self.session.generation+=1;self.jobs.pop()()
        self.n.hand.transition.assert_not_called();self.assertEqual(self.bridge.last['state'],'failed')

    def test_gateway_epoch_change_during_hand(self):
        def cancelled(guard):
            self.n.gateway['task_stop_epoch']='4';guard()
        self.n.hand.begin_motion.side_effect=cancelled
        self.send(self.data());self.jobs.pop()();self.n.hand.transition.assert_not_called()

    def test_movement_during_hand_rejected(self):
        def moved(guard):
            self.q[ARM_NAMES[0]]=.03;guard()
        self.n.hand.begin_motion.side_effect=moved
        self.send(self.data());self.jobs.pop()();self.n.hand.transition.assert_not_called()

    def test_transition_failure_no_retry(self):
        self.n.hand.transition.side_effect=RuntimeError('feedback failure')
        self.send(self.data());self.jobs.pop()();self.assertEqual(self.bridge.last['state'],'failed')
        self.n.hand.transition.assert_called_once()

    def test_request_timeout_is_not_completion(self):
        self.send(self.data('admittance'));self.bridge.last['stamp']-=15
        self.assertEqual(self.bridge.status()['last']['state'],'unconfirmed')
        self.n.stop.assert_called_once()

    def test_malformed_json_rejected(self):
        for data in ('{','null','[]'):
            self.bridge.receive(SimpleNamespace(data=data));self.assertFalse(self.jobs)


if __name__=='__main__':unittest.main()
