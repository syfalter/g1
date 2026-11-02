"""ROS types, local recorded URDF and in-memory state only; no DDS initialized."""
from collections import deque
from pathlib import Path
import threading
from types import SimpleNamespace
import unittest
import time

import numpy as np
from unitree_hg.msg import LowState
from tcp_a_node import MOTORS, TcpANode, measured_wrist, required_joints
from tcp_a_core import DEFAULT_LIMITS
from unittest.mock import patch
from workbench_model import OfflineModel


class NodeTests(unittest.TestCase):
    def state(self):
        fake=SimpleNamespace(data_lock=threading.Lock(),last_tick=None,last_record=0.,history=deque(),
            get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=1000000000)))
        msg=LowState();msg.tick=100
        for index in MOTORS.values():msg.motor_state[index].vol=48.
        return fake,msg

    def test_duplicate_tick_is_not_new_feedback(self):
        fake,msg=self.state();TcpANode.lowstate(fake,msg)
        fake.last_record=0.;TcpANode.lowstate(fake,msg);self.assertEqual(len(fake.history),1)

    def test_backward_tick_clears_stability_history(self):
        fake,msg=self.state();TcpANode.lowstate(fake,msg);msg.tick=90;fake.last_record=0.
        TcpANode.lowstate(fake,msg);self.assertEqual(len(fake.history),1);self.assertEqual(fake.history[0]['tick'],90)

    def test_invalid_measurements_clear_history(self):
        fake,msg=self.state();TcpANode.lowstate(fake,msg);msg.tick=101;fake.last_record=0.
        msg.motor_state[15].q=float('nan');TcpANode.lowstate(fake,msg);self.assertFalse(fake.history)

    def test_missing_joints_cannot_default_zero(self):
        fake=SimpleNamespace(joints={'left_wrist_yaw_link':('pelvis','revolute',None,None,None,'left_wrist_yaw_joint',None,None)})
        with self.assertRaisesRegex(ValueError,'缺少实际'):measured_wrist(fake,'left',{})

    def test_model_wrist_chain(self):
        paths=sorted((Path(__file__).resolve().parents[1]/'data/tcp_a/models').glob('robot_*.urdf'))
        if not paths:self.skipTest('No actual model snapshot on this machine')
        model=OfflineModel(paths[-1]);q={n:0. for n in MOTORS}
        for side in ('left','right'):
            required=required_joints(model,side+'_wrist_yaw_link')
            self.assertIn('waist_yaw_joint',required)
            self.assertEqual(len(required),8)
            zero=measured_wrist(model,side,q);q['waist_yaw_joint']=.2
            moved=measured_wrist(model,side,q)
            self.assertFalse(np.allclose(zero,moved));q['waist_yaw_joint']=0.

    def test_admittance_reference_is_unavailable_not_zero(self):
        n=SimpleNamespace(data_lock=threading.Lock(),commands=[{}],diag=dict(armed='true',phase='TASK_ADMITTANCE'),diag_at=time.monotonic())
        value,reason=TcpANode.reference(n,{},{});self.assertIsNone(value);self.assertIn('TASK_ADMITTANCE',reason)

    def test_stale_gateway_reference_rejected(self):
        n=SimpleNamespace(data_lock=threading.Lock(),commands=[{}],diag=dict(armed='true',phase='ARMED_HOLD'),diag_at=time.monotonic()-3)
        value,reason=TcpANode.reference(n,{},{});self.assertIsNone(value);self.assertIn('不记为零',reason)

    def test_fk_processing_delay_is_not_receiver_outage(self):
        rows=[dict(received=i*.01,snapshot_at=1.1,source='lowstate_actual',G=np.eye(4).tolist(),
                   dq=[0.]*8,q_stability=[0.]*8) for i in range(110)]
        fake=SimpleNamespace(data_lock=threading.Lock(),history=[dict(received=1.99)])
        session=dict(limits=DEFAULT_LIMITS,s_model=[.25,0,0])
        with patch('tcp_a_node.time.monotonic',return_value=2.):
            result=TcpANode.check_window(fake,rows,session)
        self.assertAlmostEqual(result['stream_age_s'],.01)
        self.assertGreater(result['processing_s'],.8)

    def test_receiver_outage_still_rejected_after_valid_snapshot(self):
        rows=[dict(received=i*.01,snapshot_at=1.1,source='lowstate_actual',G=np.eye(4).tolist(),
                   dq=[0.]*8,q_stability=[0.]*8) for i in range(110)]
        fake=SimpleNamespace(data_lock=threading.Lock(),history=[dict(received=1.09)])
        with patch('tcp_a_node.time.monotonic',return_value=2.):
            with self.assertRaisesRegex(ValueError,'接收端断流'):
                TcpANode.check_window(fake,rows,dict(limits=DEFAULT_LIMITS,s_model=[.25,0,0]))


if __name__=='__main__':unittest.main()
