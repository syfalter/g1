"""Offline checks of the exact integral policy used for selected-arm compensation."""
from pathlib import Path
import subprocess
import tempfile
import unittest
from task_planning_config import PlanningParameters


class ArmIntegralTests(unittest.TestCase):
    def test_tuned_elbow_static_residual_at_real_controller_update_rate(self):
        parameters=PlanningParameters(left_elbow_kp=65.,left_elbow_kd=3.5,
                                      elbow_ki=6.,elbow_integral_limit_nm=.8)
        source=r'''
#include "g1_unitree_control/task_waist_integral.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
int main(int argc, char** argv) {
  assert(argc==5);
  const double kp=std::stod(argv[1]),kd=std::stod(argv[2]);
  const double ki=std::stod(argv[3]),limit=std::stod(argv[4]);
  // Approximate load from the recorded last sample: 55*0.03958 + 0.0449 Nm.
  // No fitted robot inertia, coupling, contact or friction model is claimed.
  for (double inertia : {.03,.1,.4,1.}) for (double sign : {-1.,1.}) {
    g1_task::WaistIntegral controller;
    double q=-sign*.03958,v=0,integral=0,peak_v=0;
    for (int i=0;i<5000;++i) {
      if (i%10==0)
        integral=controller.step(-q,0,0,v,.01,true,false,ki,limit,.25).torque;
      const double a=(-kp*q-kd*v+integral-sign*2.222)/inertia;
      v+=a*.001;q+=v*.001;peak_v=std::max(peak_v,std::abs(v));
      assert(std::isfinite(q) && std::abs(integral)<=limit);
    }
    std::cout<<inertia<<" "<<sign<<" "<<std::abs(q)<<" "<<peak_v<<"\n";
  }
}
'''
        root=Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as directory:
            binary=str(Path(directory)/'elbow_settling')
            subprocess.run(['c++','-std=c++17','-Wall','-Wextra','-Werror','-x','c++','-',
                            '-I'+str(root/'src/g1_unitree_control/include'),'-o',binary],
                           input=source,text=True,check=True,capture_output=True)
            def simulate(gains):
                result=subprocess.run([binary,*map(str,gains)],capture_output=True,text=True,check=True)
                return [list(map(float,row.split())) for row in result.stdout.splitlines()]
            old=simulate([55,3,3,.4])
            new=simulate([parameters.left_elbow_kp,parameters.left_elbow_kd,
                          parameters.elbow_ki,parameters.elbow_integral_limit_nm])
            self.assertEqual(len(new),8)
            for before,after in zip(old,new):
                self.assertGreater(before[2],.025)
                self.assertLess(after[2],.025)
                self.assertLess(after[3],.15)

    def test_bounds_disturbance_settling_reversal_and_stop(self):
        source=r'''
#include "g1_unitree_control/task_waist_integral.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
int main() {
  using g1_task::WaistIntegral;
  using g1_task::WaistIntegralMode;
  WaistIntegral controller;
  g1_task::WaistIntegralResult r;
  double previous=0;
  for (int i=0;i<2000;++i) {
    r=controller.step(.032455247,0,0,0,.01,true,false,5,.8,.25);
    assert(std::abs(r.torque)<=.8 && r.torque-previous<=.002500001);
    previous=r.torque;
  }
  assert(r.saturated && r.torque==.8);
  // A pinned/contacted joint cannot accumulate an unlimited command.
  for (int i=0;i<10000;++i)
    r=controller.step(.032455247,0,0,0,.01,true,false,5,.8,.25);
  assert(r.torque==.8);
  r=controller.step(-.015,-.02,0,0,.01,true,true,5,.8,.25);
  assert(r.mode==WaistIntegralMode::ReversalBleed && r.torque<.8);
  r=controller.step(.05,0,0,0,.01,true,false,5,.8,.25);
  assert(r.mode==WaistIntegralMode::ErrorOutOfRange);
  controller.reset();assert(controller.torque()==0);
  r=controller.step(.02,0,0,0,.01,false,false,5,.8,.25);
  assert(r.torque==0);
  r=controller.step(NAN,0,0,0,.01,true,false,5,.8,.25);
  assert(r.torque==0);
  r=controller.step(.02,0,0,0,.01,true,false,5,.8,0);
  assert(r.torque==0);
  for (int i=0;i<200;++i)
    r=controller.step(.02,.02,0,0,.01,true,true,5,.8,.25);
  previous=r.torque;
  r=controller.step(.01,.02,0,.15,.01,true,true,5,.8,.25);
  assert(r.mode==WaistIntegralMode::BreakawayBleed && r.torque<previous);

  // A simple constant-disturbance plant, not a validated robot dynamics model.
  for (double inertia : {.03,.1,.4,1.0}) for (double sign : {-1.,1.}) {
    controller.reset();
    double q=-sign*.032455247,v=0,peak_v=0;
    const double disturbance=sign*55*.032455247,dt=.001;
    for (int i=0;i<5000;++i) {
      r=controller.step(-q,0,0,v,dt,true,false,5,.8,.25);
      const double a=(-55*q-3*v+r.torque-disturbance)/inertia;
      v+=a*dt;q+=v*dt;
      peak_v=std::max(peak_v,std::abs(v));
      assert(std::isfinite(q) && std::abs(r.torque)<=.8);
    }
    std::cout<<inertia<<" "<<sign<<" "<<std::abs(q)<<" "<<peak_v<<"\n";
    assert(std::abs(q)<.025 && peak_v<.03);
  }
  // New trajectory boundaries retain compensation until an explicit reset.
  previous=controller.torque();
  r=controller.step(.02,0,0,0,.01,true,true,5,.8,.25);
  assert(std::abs(r.torque-previous)<=.007500001);
  controller.reset();assert(controller.torque()==0);
}
'''
        root=Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as directory:
            binary=str(Path(directory)/'arm_integral')
            subprocess.run(['c++','-std=c++17','-Wall','-Wextra','-Werror','-x','c++','-',
                            '-I'+str(root/'src/g1_unitree_control/include'),'-o',binary],
                           input=source,text=True,check=True,capture_output=True)
            subprocess.run([binary],check=True,capture_output=True)


if __name__=='__main__':unittest.main()
