"""Waist-only tuning, bounded recovery and regression isolation from arm policy."""
from pathlib import Path
import math
import subprocess
import tempfile
import unittest

from replay_waist_recovery import compile_probe, plant
from task_planning_config import parameters_from_values, PlanningParameters


class WaistRecoveryTests(unittest.TestCase):
    def test_core_boundaries_and_continuous_gain(self):
        source = r'''
#include "g1_unitree_control/task_waist_integral.hpp"
#include <cassert>
#include <cmath>
int main() {
  using namespace g1_task;
  WaistIntegralTuning tuning;
  tuning.recovery_error_rad=4.*3.14159265358979323846/180.;
  tuning.build_rate_nm_s=.6;tuning.curve_gain_scale=.5;tuning.smooth_stiction=true;
  for (double sign : {-1.,1.}) {
    WaistIntegral old, waist;
    WaistIntegralResult r;
    for (int i=0;i<500;++i) {
      const double before=waist.torque();
      r=waist.step(sign*.05215,0,0,0,.01,true,false,16,1.6,1.,tuning);
      assert(std::abs(r.torque-before)<=.006000001);
      assert(std::abs(r.torque)<=1.6);
      auto legacy=old.step(sign*.05215,0,0,0,.01,true,false,20,1.2);
      assert(legacy.mode==WaistIntegralMode::ErrorOutOfRange && legacy.torque==0);
    }
    assert(r.mode==WaistIntegralMode::SlowRecovery && sign*r.torque==1.6);
    // Error extension never permits unbounded output against a stuck load.
    for (int i=0;i<10000;++i) r=waist.step(sign*.05215,0,0,0,.01,true,false,16,1.6,1.,tuning);
    assert(sign*r.torque==1.6);
    auto outside=waist.step(sign*.08,0,0,0,.01,true,false,16,1.6,1.,tuning);
    assert(outside.mode==WaistIntegralMode::ErrorOutOfRange && std::abs(outside.torque)<1.6);
    auto fast=waist.step(sign*.05215,sign*.1,0,0,.01,true,true,16,1.6,1.,tuning);
    assert(fast.mode==WaistIntegralMode::ErrorOutOfRange);
    fast=waist.step(sign*.05215,0,0,sign*.12,.01,true,false,16,1.6,1.,tuning);
    assert(fast.mode==WaistIntegralMode::ErrorOutOfRange);
    auto reverse=waist.step(-sign*.02,-sign*.02,0,0,.01,true,true,16,1.6,1.,tuning);
    assert(reverse.mode==WaistIntegralMode::ReversalBleed);
    waist.reset();assert(waist.torque()==0);
    auto disabled=waist.step(sign*.05215,0,0,0,.01,false,false,16,1.6,1.,tuning);
    assert(disabled.torque==0 && disabled.mode==WaistIntegralMode::Disabled);
    disabled=waist.step(NAN,0,0,0,.01,true,false,16,1.6,1.,tuning);
    assert(disabled.torque==0);
  }
  // Gain has no step at the legacy 0.012 rad/s stiction boundary.
  double gains[2]{};
  for (int j=0;j<2;++j) {
    WaistIntegral controller;
    for (int i=0;i<50;++i)
      gains[j]=controller.step(.02,.02,0,.012+(j ? 1e-7 : -1e-7),.01,true,true,16,1.6,1.,tuning).gain_scale;
  }
  assert(std::abs(gains[1]-gains[0])<.0001);
  WaistIntegralTuning bad=tuning;bad.build_rate_nm_s=NAN;
  assert(!bad.valid());bad=tuning;bad.recovery_error_rad=.09;assert(!bad.valid());
}
'''
        root = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as temp:
            binary = Path(temp)/'bounds'
            subprocess.run(['c++', '-std=c++17', '-Wall', '-Wextra', '-Werror', '-x', 'c++', '-',
                            '-I'+str(root/'src/g1_unitree_control/include'), '-o', str(binary)],
                           input=source, text=True, check=True, capture_output=True)
            subprocess.run([str(binary)], check=True, capture_output=True)

    def test_configuration_and_old_snapshots(self):
        old = parameters_from_values({})
        self.assertAlmostEqual(old.waist_integral_recovery_error_deg, math.degrees(.04))
        self.assertFalse(old.waist_integral_smooth_stiction)
        for key, value in [('waist_integral_recovery_error_deg', 5.), ('waist_integral_build_rate_nm_s', 2.),
                           ('waist_integral_curve_scale', float('nan')), ('waist_integral_smooth_stiction', 1)]:
            with self.assertRaises(ValueError): parameters_from_values({key: value})
        new = PlanningParameters(waist_integral_smooth_stiction=True)
        self.assertIs(new.controller_parameters('left')['waist_integral_smooth_stiction'], True)
        self.assertEqual(new.arm_integral_parameters('left'), old.arm_integral_parameters('left'))

    def test_nominal_residual_recovers_in_simplified_plant(self):
        with tempfile.TemporaryDirectory() as temp:
            binary = compile_probe(temp)
            old = plant(binary, [60.,4.5,20.,1.2,math.degrees(.04),1.,.25,0])
            new = plant(binary, [70.,5.,16.,1.8,4.,.6,.5,1])
        for before, after in zip(old, new):
            self.assertTrue(math.isfinite(after['final_error_rad']))
            self.assertLessEqual(after['peak_integral_nm'], 1.8)
            self.assertLess(after['peak_velocity_rad_s'], .5)
            if after['kind'] == 0:
                self.assertGreater(before['final_error_rad'], .04)
                self.assertLess(after['final_error_rad'], .025)


if __name__ == '__main__':
    unittest.main()
