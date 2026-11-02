#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace g1_task {
inline double waist_command_rate(double control, double requested)
{
  if (!std::isfinite(control) || !std::isfinite(requested) || control <= 0 || requested <= 0)
    throw std::invalid_argument("waist/control command rates must be positive and finite");
  // Commands can only change on an arm timer tick. Use the actual held period.
  const double ticks = std::max(1.0, std::ceil(control / requested - 1e-9));
  return control / ticks;
}
inline double waist_velocity_limit(double requested, double step, double rate)
{
  return std::min(requested, step * rate * .95);
}
}
