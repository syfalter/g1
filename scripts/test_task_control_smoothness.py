"""Focused checks for continuous, conservative tracking slowdown."""
from pathlib import Path
import subprocess
import tempfile
import unittest


class TrackingScaleTests(unittest.TestCase):
    def test_threshold_crossings_and_immediate_pause(self):
        source = r'''
#include "g1_unitree_control/task_tracking_scale.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
int main() {
  using g1_task::tracking_time_scale;
  for (double pause : {.06, .10}) {
    for (double boundary : {.03, .06, pause}) {
      const auto a=tracking_time_scale(boundary-1e-7,.06,pause);
      const auto b=tracking_time_scale(boundary+1e-7,.06,pause);
      assert(std::abs(a-b)<1e-8);
    }
    double previous=1.;
    for (int i=0;i<=12000;++i) {
      const double error=i*.00001;
      const double scale=tracking_time_scale(error,.06,pause);
      const double old=error>=pause ? 0. : (error>=.06 ? .25 : 1.);
      assert(scale>=0. && scale<=old+1e-12 && scale<=previous+1e-12);
      assert(scale==tracking_time_scale(-error,.06,pause));
      previous=scale;
    }
  }
  double old_previous=.04,new_previous=.04,old_peak=0,new_peak=0;
  // 10 ms control ticks, .01 rad/s error ramp, fixed .04 rad/s reference.
  for (int i=0;i<=1100;++i) {
    const double error=i*.0001;
    const double old=.04*(error>=.1 ? 0. : (error>=.06 ? .25 : 1.));
    const double current=.04*tracking_time_scale(error,.06,.1);
    old_peak=std::max(old_peak,std::abs(old-old_previous));
    new_peak=std::max(new_peak,std::abs(current-new_previous));
    old_previous=old; new_previous=current;
  }
  assert(std::abs(old_peak-.03)<1e-12);
  assert(new_peak<.0002);
  assert(tracking_time_scale(.11,.06,.1)==0.);
  assert(tracking_time_scale(NAN,.06,.1)==0.);
  assert(tracking_time_scale(0,.1,.06)==0.);
  std::cout << "threshold ramp peak dq step: old=" << old_peak
            << " new=" << new_peak << " rad/s\n";
}
'''
        root = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as directory:
            binary = str(Path(directory) / 'tracking_scale')
            subprocess.run(['c++', '-std=c++17', '-Wall', '-Wextra', '-Werror', '-x', 'c++', '-',
                            '-I' + str(root / 'src/g1_unitree_control/include'), '-o', binary],
                           input=source, text=True, check=True, capture_output=True)
            result = subprocess.run([binary], text=True, check=True, capture_output=True)
            print(result.stdout.strip())


if __name__ == '__main__':
    unittest.main()
