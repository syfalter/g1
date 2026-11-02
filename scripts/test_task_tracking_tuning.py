"""Offline invariants for paired-arm compensation and the actual integral header."""
import math
import csv
from pathlib import Path
import subprocess
import tempfile
import unittest

from task_planning_config import PlanningParameters,parameters_from_values


class TrackingTuningTests(unittest.TestCase):
    def test_waist_peak_uses_waist_torque_not_arm_placeholder(self):
        from review_task_tracking import review
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);logs=root/'logs/endpoint_motion';logs.mkdir(parents=True)
            n='waist_yaw_joint'
            rows=[dict(ros_time_s=t,event=event,waist_integral_mode='hold_integrate',
                       waist_integral_torque_nm=torque,**{n+'_target':.01,n+'_q':0,
                       n+'_dq':0,n+'_target_dq':0,n+'_arm_integral_nm':0})
                  for t,event,torque in ((1,'sample',-1.8),(2,'settling_hold',-1.2))]
            with (logs/'endpoint_motion_20260912_test.csv').open('w') as stream:
                writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
            result=review(root,'20260912')['traces'][0]['joints'][n]
            self.assertEqual(result['peak_integral_nm'],1.8)
            self.assertEqual(result['steady_integral_nm'],-1.2)

    def test_old_snapshots_leave_passive_integral_disabled(self):
        old=parameters_from_values({})
        self.assertFalse(old.controller_parameters('left')['passive_arm_integral_enabled'])
        self.assertAlmostEqual(old.arm_integral_recovery_error_deg,math.degrees(.04))
        self.assertEqual(old.arm_integral_hold_rate_nm_s,.25)
        for side in ('left','right'):
            p=PlanningParameters(left_arm_integral_enabled=True,right_arm_integral_enabled=True,
                                 passive_arm_integral_enabled=True)
            self.assertTrue(p.controller_parameters(side)['passive_arm_integral_enabled'])
        p=PlanningParameters(left_arm_integral_enabled=True,right_arm_integral_enabled=False,
                             passive_arm_integral_enabled=True)
        self.assertFalse(p.controller_parameters('left')['passive_arm_integral_enabled'])
        self.assertFalse(p.controller_parameters('right')['arm_integral_enabled'])
        for values in ({'arm_integral_recovery_error_deg':3.6},{'passive_arm_integral_enabled':1},
                       {'arc_center_toward_robot_mm':31},{'shoulder_roll_integral_limit_nm':3.61},
                       {'shoulder_pitch_integral_limit_nm':1.81},{'elbow_integral_limit_nm':1.01},
                       {'arm_integral_hold_rate_nm_s':.51}):
            with self.assertRaises(ValueError):parameters_from_values(values)
        self.assertEqual(parameters_from_values({'shoulder_roll_integral_limit_nm':2.2}).
                         controller_parameters('right')['arm_integral_limits_nm'][1],2.2)

    def test_arm_recovery_is_bounded_slow_and_independent(self):
        source=r'''
#include "g1_unitree_control/task_waist_integral.hpp"
#include <cassert>
int main() {
  using namespace g1_task;
  WaistIntegralTuning tuning;tuning.recovery_error_rad=3.5*3.141592653589793/180.;
  assert(arm_integral_rate_scale(0,0,0,.4)==.4);
  assert(arm_integral_rate_scale(.04,.2,.079,.4)==.4);
  assert(arm_integral_rate_scale(.041,0,0,.4)==.25);
  assert(arm_integral_rate_scale(0,.201,0,.4)==.25);
  assert(arm_integral_rate_scale(0,0,.08,.4)==.25);
  assert(arm_integral_rate_scale(0,0,0,.51)==.25);
  for(double sign : {-1.,1.}) {
    WaistIntegral shoulder;
    for(int i=0;i<1800;++i) {
      const double previous=shoulder.torque();
      auto r=shoulder.step(sign*.04,0,0,0,.01,true,false,10,3.6,.4,tuning);
      assert(std::abs(r.torque)<=3.6 && std::abs(r.torque-previous)<=.004000001);
    }
    assert(std::abs(shoulder.torque())==3.6);
    shoulder.reset();assert(shoulder.torque()==0);
  }
  for (double sign : {-1.,1.}) {
    WaistIntegral shoulder;
    for(int i=0;i<1400;++i) {
      const double previous=shoulder.torque();
      const auto r=shoulder.step(sign*.04,0,0,0,.01,true,false,8,2.2,.25,tuning);
      assert(std::abs(r.torque)<=2.2 && std::abs(r.torque-previous)<=.002500001);
    }
    assert(std::abs(shoulder.torque())==2.2);
    auto r=shoulder.step(-sign*.02,0,0,0,.01,true,false,8,2.2,.25,tuning);
    assert(r.mode==WaistIntegralMode::ReversalBleed);
    shoulder.reset();assert(shoulder.torque()==0);
  }
  for (double sign : {-1.,1.}) {
    WaistIntegral active,passive,legacy;
    for (int i=0;i<600;++i) {
      double previous=passive.torque();
      auto p=passive.step(sign*.052,0,0,0,.01,true,false,8,1,.25,tuning);
      auto a=active.step(-sign*.02,.01,0,0,.01,true,true,6,1,.25,tuning);
      assert(sign*p.torque>=0 && sign*a.torque<=0);
      assert(std::abs(p.torque-previous)<=.002500001 && std::abs(p.torque)<=1.);
      assert(legacy.step(sign*.052,0,0,0,.01,true,false,8,1,.25).torque==0);
    }
    assert(std::abs(passive.torque())==1.);
    auto r=passive.step(sign*.07,0,0,0,.01,true,false,8,1,.25,tuning);
    assert(r.mode==WaistIntegralMode::ErrorOutOfRange);
    r=passive.step(sign*.052,sign*.06,0,0,.01,true,true,8,1,.25,tuning);
    assert(r.mode==WaistIntegralMode::ErrorOutOfRange);
    r=passive.step(sign*.052,0,0,sign*.1,.01,true,false,8,1,.25,tuning);
    assert(r.mode==WaistIntegralMode::ErrorOutOfRange);
    r=passive.step(-sign*.02,0,0,0,.01,true,false,8,1,.25,tuning);
    assert(r.mode==WaistIntegralMode::ReversalBleed);
    active.reset();passive.reset();assert(active.torque()==0 && passive.torque()==0);
    assert(passive.step(sign*.052,0,0,0,.01,false,false,8,1,.25,tuning).torque==0);
  }
}
'''
        root=Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as directory:
            binary=Path(directory)/'bounds'
            subprocess.run(['c++','-std=c++17','-Wall','-Wextra','-Werror','-x','c++','-',
                '-I'+str(root/'src/g1_unitree_control/include'),'-o',str(binary)],
                input=source,text=True,check=True,capture_output=True)
            subprocess.run([str(binary)],check=True,capture_output=True)
