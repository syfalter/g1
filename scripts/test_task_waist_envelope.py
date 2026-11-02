"""Compile and exercise the same polynomial guard used by the C++ gateway."""
from pathlib import Path
import subprocess
import tempfile
import unittest


class WaistEnvelopeTests(unittest.TestCase):
    def test_interpolation_envelope(self):
        source=r'''
#include "g1_unitree_control/task_waist_trajectory.hpp"
#include <cassert>
#include <limits>
int main() {
  using g1_task::waist_segment_safe;
  for (int mode=0;mode<3;++mode) {
    assert(waist_segment_safe(.12,.14,0,0,0,0,3,mode>0,mode>1,-.59,.59,.4,.5));
    assert(waist_segment_safe(-.12,-.14,0,0,0,0,3,mode>0,mode>1,-.59,.59,.4,.5));
    assert(!waist_segment_safe(.12,.14,0,0,0,0,0,mode>0,mode>1,-.59,.59,.4,.5));
  }
  assert(!waist_segment_safe(.12,.12,.4,-.4,0,0,8,true,true,-.59,.59,.4,.5));
  assert(!waist_segment_safe(.12,.14,0,0,0,0,.01,true,true,-.59,.59,.4,.5));
  assert(!waist_segment_safe(.12,.7,0,0,0,0,10,true,true,-.59,.59,.4,.5));
  assert(!waist_segment_safe(.12,std::numeric_limits<double>::quiet_NaN(),0,0,0,0,3,true,true,-.59,.59,.4,.5));
}
'''
        root=Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as directory:
            binary=str(Path(directory)/'waist_envelope')
            subprocess.run(['c++','-std=c++17','-Wall','-Wextra','-Werror','-x','c++','-',
                            '-I'+str(root/'src/g1_unitree_control/include'),'-o',binary],
                           input=source,text=True,check=True,capture_output=True)
            subprocess.run([binary],check=True,capture_output=True)


if __name__=='__main__':unittest.main()
