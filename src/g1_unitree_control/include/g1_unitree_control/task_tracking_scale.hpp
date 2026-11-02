#pragma once

#include <algorithm>
#include <cmath>

namespace g1_task
{
// Slow before each existing guard boundary, never after it. A discontinuous
// feedback fault must still pause immediately rather than coast through a ramp.
inline double tracking_time_scale(double error, double warning, double pause)
{
  if (!std::isfinite(error) || !std::isfinite(warning) ||
      !std::isfinite(pause) || warning <= 0.0 || pause < warning)
    return 0.0;
  error = std::abs(error);
  if (error >= pause) return 0.0;
  const auto blend = [](double u) {
    u = std::clamp(u, 0.0, 1.0);
    return u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
  };
  const double onset = warning * 0.5;
  if (error <= onset) return 1.0;
  if (error < warning)
  {
    const double at_warning = pause > warning ? 0.25 : 0.0;
    return 1.0 - (1.0 - at_warning) * blend((error - onset) / (warning - onset));
  }
  return 0.25 * (1.0 - blend((error - warning) / (pause - warning)));
}
}  // namespace g1_task
