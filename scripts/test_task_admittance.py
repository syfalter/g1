import pathlib
import subprocess
import tempfile
import unittest

ROOT=pathlib.Path(__file__).resolve().parents[1]


class AdmittanceTest(unittest.TestCase):
    def test_bounded_persistent_reference(self):
        # Exercise the C++ implementation directly, without ROS or hardware.
        code=r'''
#include "g1_unitree_control/task_admittance.hpp"
#include <cassert>
#include <limits>
int main() {
  g1_task::Admittance adm;
  double q = 0, previous_step = 0;
  for (int i=0;i<200;++i) {
    double next=adm.step(0,q,q,10,-1,1,.01);
    double step=next-q;
    assert(step>=0 && step<=.001200001);
    assert(std::abs(step-previous_step)<=.000030001);
    previous_step=step; q=next;
  }
  adm.reset();
  double hold=.2;
  for (int i=0;i<100;++i) hold=adm.step(0,hold,.2,0,-1,1,.01);
  assert(hold==.2);
  for (int i=0;i<1000;++i) hold=adm.step(0,hold,.2,100,-1,1,.01);
  assert(hold<=.23500001);
  adm.reset();
  q=.999;
  for (int i=0;i<1000;++i) q=adm.step(0,q,q,100,-1,1,.01);
  assert(q<=1.0);
  adm.reset();
  q=adm.step(0,0,0,std::numeric_limits<double>::quiet_NaN(),-1,1,.01);
  assert(q==0);
  adm.reset();
  double left=0,right=0;
  for (int i=0;i<300;++i) {
    left=adm.step(0,left,left,.3,-1,1,.01);
    right=adm.step(7,right,right,.3,-1,1,.01);
  }
  assert(std::abs(left-right)<1e-12);
  adm.reset(); q=0; previous_step=0;
  for(int i=0;i<1000;++i) {
    double next=adm.step(14,q,q,10,-.58,.58,.01);
    double step=next-q;
    assert(step>=0 && step<=.000500001);
    assert(std::abs(step-previous_step)<=.000010001);
    previous_step=step; q=next;
  }
  adm.reset(); q=.1;
  for(int i=0;i<1000;++i)q=adm.step(14,q,.1,10,-.58,.58,.01);
  assert(q<=.120000001);
  adm.reset(); q=.1;
  for(int i=0;i<1000;++i)q=adm.step(14,q,.1,.7,-.58,.58,.01);
  assert(q==.1);
  assert(adm.step(14,q,q,std::numeric_limits<double>::quiet_NaN(),-.58,.58,.01)==q);
  adm.reset(); q=-.579;
  for(int i=0;i<1000;++i)q=adm.step(14,q,q,-10,-.58,.58,.01);
  assert(q>=-.58);
}
'''
        with tempfile.TemporaryDirectory() as directory:
            executable=str(pathlib.Path(directory)/'test')
            result=subprocess.run(['g++','-std=c++17','-x','c++','-',
                '-I'+str(ROOT/'src/g1_unitree_control/include'),'-o',executable],
                input=code,text=True,capture_output=True)
            self.assertEqual(result.returncode,0,result.stderr)
            subprocess.run([executable],check=True)


if __name__=='__main__': unittest.main()
