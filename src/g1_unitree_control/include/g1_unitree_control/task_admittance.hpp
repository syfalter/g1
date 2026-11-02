#pragma once
#include <algorithm>
#include <array>
#include <cmath>

namespace g1_task
{
// Same residual sign as the commissioned SDK: gravity - tau_est - frozen bias.
// No measured-velocity passthrough or breakaway assist in the first integration.
class Admittance
{
public:
  void reset() { velocity_.fill(0.0); filtered_.fill(0.0); }
  double step(std::size_t j, double reference, double measured, double residual,
              double lower, double upper, double dt)
  {
    constexpr std::array<double, 7> deadband{.08,.06,.08,.06,.06,.05,.04};
    constexpr std::array<double, 7> filter{.08,.08,.08,.10,.12,.08,.08};
    constexpr std::array<double, 7> gain{.60,.80,.70,.80,1.30,1.00,1.20};
    // Initial commissioning deliberately limits virtual motion to 0.12 rad/s.
    if (!std::isfinite(reference + measured + residual + lower + upper + dt) ||
        dt <= 0 || dt > .05 || lower >= upper || j >= 15)
      return reference;
    const auto axis = j % 7;
    // Slot 14 is waist yaw, deliberately slower than commissioned arm axes.
    const bool waist = j == 14;
    filtered_[j] += dt / ((waist ? .15 : filter[axis]) + dt) * (residual - filtered_[j]);
    const double force = std::copysign(
      std::max(0.0, std::abs(filtered_[j]) - (waist ? .8 : deadband[axis])), filtered_[j]);
    const double vmax = waist ? .05 : .12, amax = waist ? .10 : .30;
    const double lead = waist ? .02 : .035;
    const double target = std::clamp((waist ? .08 : gain[axis]) * force, -vmax, vmax);
    velocity_[j] += std::clamp(target - velocity_[j], -amax * dt, amax * dt);
    const double next = std::clamp(reference + velocity_[j] * dt,
                                  measured - lead, measured + lead);
    const double bounded = std::clamp(next, lower, upper);
    if (bounded != next) velocity_[j] = 0;
    return bounded;
  }
private:
  std::array<double, 15> velocity_{};
  std::array<double, 15> filtered_{};
};
}
