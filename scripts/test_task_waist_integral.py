"""Compile and exercise the exact bounded waist integrator used by the gateway."""
from pathlib import Path
import subprocess
import tempfile
import unittest


class WaistIntegralTests(unittest.TestCase):
    def test_curve_stiction_breakaway_and_reversal(self):
        source=r'''
#include "g1_unitree_control/task_waist_integral.hpp"
#include <cassert>
#include <cmath>
int main() {
  using g1_task::WaistIntegral;
  using g1_task::WaistIntegralMode;
  WaistIntegral controller;
  auto result=controller.step(.017,.016,0,0,.01,false,true,20,1.2);
  assert(result.mode==WaistIntegralMode::Disabled && result.torque==0);

  double previous=0;
  for (int i=0;i<200;++i) {
    result=controller.step(.017,.016,.02,0,.01,true,true,20,1.2);
    assert(std::abs(result.torque-previous)<=.0100001);
    previous=result.torque;
  }
  assert(result.mode==WaistIntegralMode::Stiction);
  assert(result.torque>.61 && result.torque<.63);

  const double before_breakaway=result.torque;
  for (int i=0;i<10;++i) {
    previous=result.torque;
    result=controller.step(.010,.040,0,.20,.01,true,true,20,1.2);
    assert(result.mode==WaistIntegralMode::BreakawayBleed);
    assert(result.torque<=previous && previous-result.torque<=.0300001);
  }
  assert(result.torque<before_breakaway-.25);

  controller.reset();
  for (int i=0;i<200;++i)
    result=controller.step(.008,.040,.030,.035,.01,true,true,20,1.2);
  assert(result.mode==WaistIntegralMode::Curve);
  assert(result.gain_scale>.20 && result.gain_scale<.23);
  assert(result.torque>.055 && result.torque<.065);

  controller.reset();
  for (int i=0;i<200;++i)
    result=controller.step(.010,0,0,0,.01,true,false,20,1.2);
  assert(result.mode==WaistIntegralMode::Hold);
  assert(result.gain_scale==1.0);
  assert(result.torque>.35 && result.torque<.37);

  // The recorded above-table waist command peaks at only 0.001733 rad/s.
  for (const double sign : {-1.0, 1.0}) {
    controller.reset();
    for (int i=0;i<200;++i)
      result=controller.step(sign*.008,sign*.001733,0,0,.01,true,true,20,1.7);
    assert(result.mode==WaistIntegralMode::Stiction);
    assert(result.gain_scale==1.0 && sign*result.torque>.27);
    const double before=result.torque;
    result=controller.step(-sign*.008,-sign*.001733,0,0,.01,true,true,20,1.7);
    assert(result.mode==WaistIntegralMode::ReversalBleed);
    assert(std::abs(result.torque)<std::abs(before));
  }

  // A stationary waist in an active arm path uses exactly the hold policy.
  controller.reset();
  WaistIntegral hold;
  for (int i=0;i<200;++i) {
    result=controller.step(.001,0,0,0,.01,true,true,20,1.7);
    auto expected=hold.step(.001,0,0,0,.01,true,false,20,1.7);
    assert(result.mode==expected.mode && result.torque==expected.torque);
  }
  result=controller.step(.0004,0,0,0,.01,true,true,20,1.7);
  auto expected=hold.step(.0004,0,0,0,.01,true,false,20,1.7);
  assert(result.torque==expected.torque);

  // Zero speed at a curve turnaround is not a stationary reference.
  controller.reset();
  for (int i=0;i<200;++i)
    result=controller.step(.008,0,.03,0,.01,true,true,20,1.7);
  assert(result.mode==WaistIntegralMode::Curve && result.gain_scale<.25);

  // Residual errors below the former 0.002 rad deadband now build compensation.
  controller.reset();
  for (int i=0;i<200;++i)
    result=controller.step(.001,0,0,0,.01,true,false,20,1.7);
  assert(result.mode==WaistIntegralMode::Hold);
  assert(result.torque>.018 && result.torque<.020);
  const double before_deadband=result.torque;
  result=controller.step(.0004,0,0,0,.01,true,false,20,1.7);
  assert(result.mode==WaistIntegralMode::Deadband);
  assert(std::abs(result.torque-(before_deadband-.0005))<1e-10);

  controller.reset();
  for (int i=0;i<200;++i)
    result=controller.step(-.001,0,0,0,.01,true,false,20,1.7);
  assert(result.mode==WaistIntegralMode::Hold);
  assert(result.torque<-.018 && result.torque>-.020);
  result=controller.step(-.0005,0,0,0,.01,true,false,20,1.7);
  assert(result.mode==WaistIntegralMode::Deadband);

  controller.reset();
  for (int i=0;i<400;++i)
    result=controller.step(.020,.020,0,0,.01,true,true,30,1.2);
  assert(result.torque>1.19 && result.torque<=1.2);
  assert(result.saturated);
  const double positive=result.torque;
  for (int i=0;i<20;++i)
    result=controller.step(-.015,-.020,0,0,.01,true,true,30,1.2);
  assert(std::abs(result.torque)<positive-.55);

  result=controller.step(.050,0,0,0,.01,true,false,20,1.2);
  assert(result.mode==WaistIntegralMode::ErrorOutOfRange);
  assert(std::abs(result.torque)<positive);
  result=controller.step(NAN,0,0,0,.01,true,false,20,1.2);
  assert(result.mode==WaistIntegralMode::Disabled && result.torque==0);
}
'''
        root=Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as directory:
            binary=str(Path(directory)/'waist_integral')
            subprocess.run(['c++','-std=c++17','-Wall','-Wextra','-Werror','-x','c++','-',
                            '-I'+str(root/'src/g1_unitree_control/include'),'-o',binary],
                           input=source,text=True,check=True,capture_output=True)
            subprocess.run([binary],check=True,capture_output=True)


if __name__=='__main__':unittest.main()
