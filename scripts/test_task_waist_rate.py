from pathlib import Path
import subprocess
import tempfile
import unittest


class WaistRateTests(unittest.TestCase):
    def test_effective_rate_and_step_guard(self):
        source = r'''
#include "g1_unitree_control/task_waist_rate.hpp"
#include <cassert>
#include <limits>
int main() {
  using namespace g1_task;
  assert(waist_command_rate(100,50)==50);
  assert(waist_command_rate(100,100)==100);
  assert(waist_command_rate(100,80)==50);
  assert(waist_command_rate(100,200)==100);
  assert(std::abs(waist_velocity_limit(.35,.006,50)-.285)<1e-12);
  assert(waist_velocity_limit(.35,.006,100)==.35);
  assert(.35/100 < .006);
  for (double v : {0., -1., std::numeric_limits<double>::infinity()}) {
    bool rejected=false;
    try { waist_command_rate(100,v); } catch (const std::invalid_argument&) { rejected=true; }
    assert(rejected);
  }
}
'''
        root = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as directory:
            binary = str(Path(directory) / 'waist_rate')
            subprocess.run(['c++', '-std=c++17', '-Wall', '-Wextra', '-Werror', '-x', 'c++', '-',
                            '-I' + str(root / 'src/g1_unitree_control/include'), '-o', binary],
                           input=source, text=True, check=True, capture_output=True)
            subprocess.run([binary], check=True)


if __name__ == '__main__':
    unittest.main()
