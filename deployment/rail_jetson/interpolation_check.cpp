// Offline harness: the included method is extracted verbatim from the gateway.
// No ROS, DDS, SDK, device, command publisher, or network dependency.
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <vector>

struct Point {
  double time;
  std::vector<double> positions, velocities, accelerations;
};
struct TrajectorySample {
  std::vector<double> positions, velocities, accelerations;
};
struct Harness {
  struct { std::vector<Point> points; } trajectory_;
  std::vector<std::size_t> active_arm_indices_;
  std::vector<double> trajectory_start_positions_;
  double trajectory_start_blend_s_ = 0.0;
  double trajectory_first_point_time_s_ = 0.0;
  static double point_time(const Point& p) { return p.time; }
#include "interpolation.inc"
};

int main() {
  std::cout << std::setprecision(17);
  std::size_t trajectories;
  if (!(std::cin >> trajectories)) return 2;
  for (std::size_t k = 0; k < trajectories; ++k) {
    std::size_t count, points, queries;
    if (!(std::cin >> count >> points >> queries) || count > 15 || count == 0 || points < 2) return 3;
    Harness h;
    h.active_arm_indices_.resize(count);
    for (std::size_t i = 0; i < points; ++i) {
      Point p;
      std::cin >> p.time;
      for (auto* row : {&p.positions, &p.velocities, &p.accelerations}) {
        row->resize(count);
        for (auto& v : *row) std::cin >> v;
      }
      h.trajectory_.points.push_back(p);
    }
    h.trajectory_first_point_time_s_ = h.trajectory_.points.front().time;
    h.trajectory_start_positions_ = h.trajectory_.points.front().positions;
    for (std::size_t i = 0; i < queries; ++i) {
      double t;
      std::cin >> t;
      if (!std::cin) return 4;
      auto sample = h.interpolate_locked(t);
      for (auto* row : {&sample.positions, &sample.velocities, &sample.accelerations})
        for (double v : *row) std::cout << v << ' ';
      std::cout << '\n';
    }
  }
}
