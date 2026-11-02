#pragma once

#include <algorithm>
#include <cmath>

namespace g1_task
{
inline double arm_integral_rate_scale(
  double desired_velocity, double desired_acceleration, double measured_velocity, double hold_rate)
{
  if (std::isfinite(hold_rate) && hold_rate >= 0.1 && hold_rate <= 0.5 &&
      std::abs(desired_velocity) <= 0.04 && std::abs(desired_acceleration) <= 0.20 &&
      std::abs(measured_velocity) < 0.08)
    return hold_rate;
  return 0.25;
}

enum class WaistIntegralMode
{
  Disabled,
  ErrorOutOfRange,
  Deadband,
  Waiting,
  Hold,
  Stiction,
  Curve,
  SlowRecovery,
  BreakawayBleed,
  ReversalBleed
};

inline const char* waist_integral_mode_name(WaistIntegralMode mode)
{
  switch (mode)
  {
    case WaistIntegralMode::Disabled: return "disabled";
    case WaistIntegralMode::ErrorOutOfRange: return "error_bleed";
    case WaistIntegralMode::Deadband: return "deadband";
    case WaistIntegralMode::Waiting: return "waiting";
    case WaistIntegralMode::Hold: return "hold_integrate";
    case WaistIntegralMode::Stiction: return "stiction_integrate";
    case WaistIntegralMode::Curve: return "curve_integrate";
    case WaistIntegralMode::SlowRecovery: return "bounded_slow_recovery";
    case WaistIntegralMode::BreakawayBleed: return "breakaway_bleed";
    case WaistIntegralMode::ReversalBleed: return "reversal_bleed";
  }
  return "unknown";
}

struct WaistIntegralResult
{
  double torque{};
  double gain_scale{};
  bool integrating{};
  bool saturated{};
  WaistIntegralMode mode{WaistIntegralMode::Disabled};
};

struct WaistIntegralTuning
{
  double recovery_error_rad{0.04};
  double build_rate_nm_s{1.0};
  double curve_gain_scale{0.25};
  bool smooth_stiction{false};

  bool valid() const
  {
    return std::isfinite(recovery_error_rad) && recovery_error_rad >= 0.04 - 1e-12 &&
      recovery_error_rad <= 4.5 * 3.14159265358979323846 / 180.0 &&
      std::isfinite(build_rate_nm_s) && build_rate_nm_s >= 0.1 && build_rate_nm_s <= 1.0 &&
      std::isfinite(curve_gain_scale) && curve_gain_scale >= 0.1 && curve_gain_scale <= 1.0;
  }
};

// Bounded integral disturbance compensation for the waist yaw drive. The
// state is stored directly in Nm, so gain updates do not step the output.
class WaistIntegral
{
public:
  static constexpr const char* kPolicyVersion = "slow_curve_v2";
  static constexpr const char* kWaistPolicyVersion = "bounded_slow_recovery_v3";
  static constexpr double kDeadbandRad = 0.0005;

  void reset()
  {
    torque_ = 0.0;
    error_dwell_s_ = 0.0;
    error_direction_ = 0;
    motion_direction_ = 0;
    reversal_bleed_s_ = 0.0;
  }

  double torque() const { return torque_; }

  WaistIntegralResult step(
    double error, double desired_velocity, double desired_acceleration,
    double measured_velocity, double dt, bool enabled, bool trajectory_active,
    double ki, double torque_limit, double rate_scale = 1.0,
    const WaistIntegralTuning& tuning = WaistIntegralTuning{})
  {
    WaistIntegralResult result;
    if (!std::isfinite(error) || !std::isfinite(desired_velocity) ||
        !std::isfinite(desired_acceleration) || !std::isfinite(measured_velocity) ||
        !std::isfinite(dt) || !std::isfinite(ki) || !std::isfinite(torque_limit) ||
        !std::isfinite(rate_scale) || rate_scale <= 0.0 || rate_scale > 1.0 ||
        dt <= 0.0 || dt > 0.05 || ki < 0.0 || torque_limit < 0.0 || !tuning.valid())
    {
      reset();
      return result;
    }
    torque_ = std::clamp(torque_, -torque_limit, torque_limit);

    const int desired_direction = direction(desired_velocity, kDirectionVelocity);
    // An eight-joint arm trajectory may contain a stationary waist. Classify
    // its reference, not just ownership of the active trajectory.
    const bool moving_reference = trajectory_active &&
      (desired_direction != 0 || std::abs(desired_acceleration) > kReferenceAccelerationRadS2);
    if (desired_direction != 0)
    {
      if (motion_direction_ != 0 && desired_direction != motion_direction_)
        reversal_bleed_s_ = kReversalBleedDurationS;
      motion_direction_ = desired_direction;
    }
    reversal_bleed_s_ = std::max(0.0, reversal_bleed_s_ - dt);

    if (!enabled || ki == 0.0 || torque_limit == 0.0)
    {
      torque_ = toward_zero(torque_, kFastBleedNmS * rate_scale * dt);
      clear_error_dwell();
      result.torque = torque_;
      result.mode = WaistIntegralMode::Disabled;
      return result;
    }

    const double magnitude = std::abs(error);
    // A small, slow residual must not permanently disable its own correction.
    // Recovery bounds are configured per drive; fast/large errors still bleed.
    const bool slow_recovery = magnitude > kMaximumErrorRad &&
      magnitude <= tuning.recovery_error_rad &&
      std::abs(desired_velocity) <= 0.04 && std::abs(desired_acceleration) <= 0.20 &&
      std::abs(measured_velocity) < kBreakawayVelocityRadS;
    if (magnitude > kMaximumErrorRad && !slow_recovery)
    {
      torque_ = toward_zero(torque_, kErrorBleedNmS * rate_scale * dt);
      clear_error_dwell();
      result.torque = torque_;
      result.mode = WaistIntegralMode::ErrorOutOfRange;
      return result;
    }

    const double effective_error = std::copysign(
      std::max(0.0, std::min(magnitude, kMaximumErrorRad) - kDeadbandRad), error);
    const int error_direction = direction(effective_error, 0.0);
    if (error_direction == 0)
    {
      error_dwell_s_ = 0.0;
      error_direction_ = 0;
      const double bleed = moving_reference ? kCurveDeadbandBleedNmS : kHoldDeadbandBleedNmS;
      torque_ = toward_zero(torque_, bleed * rate_scale * dt);
      result.torque = torque_;
      result.mode = WaistIntegralMode::Deadband;
      return result;
    }
    if (error_direction != error_direction_)
    {
      error_direction_ = error_direction;
      error_dwell_s_ = 0.0;
    }
    else
    {
      error_dwell_s_ += dt;
    }

    const double closing_speed = error_direction * measured_velocity;
    const double desired_closing_speed = std::max(0.0, error_direction * desired_velocity);
    const bool breakaway = closing_speed > std::max(
      kBreakawayVelocityRadS, desired_closing_speed + kOverspeedMarginRadS);
    const bool integral_opposes_error = torque_ * effective_error < 0.0;
    if (reversal_bleed_s_ > 0.0 || integral_opposes_error)
    {
      torque_ = toward_zero(torque_, kFastBleedNmS * rate_scale * dt);
      result.torque = torque_;
      result.mode = WaistIntegralMode::ReversalBleed;
      return result;
    }
    if (breakaway)
    {
      torque_ = toward_zero(torque_, kFastBleedNmS * rate_scale * dt);
      result.torque = torque_;
      result.mode = WaistIntegralMode::BreakawayBleed;
      return result;
    }
    if (error_dwell_s_ < kErrorDwellS)
    {
      result.torque = torque_;
      result.mode = WaistIntegralMode::Waiting;
      return result;
    }

    double scale = 1.0;
    WaistIntegralMode mode = WaistIntegralMode::Hold;
    if (moving_reference)
    {
      const double acceleration_scale = std::clamp(
        1.0 - std::abs(desired_acceleration) / kAccelerationDisableRadS2,
        kMinimumAccelerationScale, 1.0);
      const double curve_scale = tuning.curve_gain_scale * acceleration_scale;
      const bool stuck = std::abs(measured_velocity) < kStictionVelocityRadS &&
        std::abs(desired_velocity) > kDirectionVelocity &&
        error * desired_velocity > 0.0;
      if (stuck)
      {
        mode = WaistIntegralMode::Stiction;
      }
      else
      {
        mode = WaistIntegralMode::Curve;
        scale = curve_scale;
      }
      if (tuning.smooth_stiction && error * desired_velocity > 0.0)
      {
        const double u = std::clamp((std::abs(measured_velocity) - 0.008) / 0.012, 0.0, 1.0);
        const double blend = u * u * (3.0 - 2.0 * u);
        scale = 1.0 + (curve_scale - 1.0) * blend;
      }
    }
    if (slow_recovery) mode = WaistIntegralMode::SlowRecovery;

    const double requested_rate = ki * scale * effective_error;
    const double torque_rate = std::clamp(
      requested_rate, -tuning.build_rate_nm_s * rate_scale, tuning.build_rate_nm_s * rate_scale);
    const double unclamped = torque_ + torque_rate * dt;
    torque_ = std::clamp(unclamped, -torque_limit, torque_limit);
    result.torque = torque_;
    result.gain_scale = scale;
    result.integrating = true;
    result.saturated = torque_ != unclamped;
    result.mode = mode;
    return result;
  }

private:
  static constexpr double kMaximumErrorRad = 0.04;
  static constexpr double kErrorDwellS = 0.12;
  static constexpr double kDirectionVelocity = 0.0003;
  static constexpr double kReferenceAccelerationRadS2 = 0.001;
  static constexpr double kStictionVelocityRadS = 0.012;
  static constexpr double kBreakawayVelocityRadS = 0.08;
  static constexpr double kOverspeedMarginRadS = 0.03;
  static constexpr double kAccelerationDisableRadS2 = 0.20;
  static constexpr double kMinimumAccelerationScale = 0.25;
  static constexpr double kFastBleedNmS = 3.0;
  static constexpr double kErrorBleedNmS = 2.0;
  static constexpr double kCurveDeadbandBleedNmS = 0.8;
  static constexpr double kHoldDeadbandBleedNmS = 0.05;
  static constexpr double kReversalBleedDurationS = 0.20;

  static int direction(double value, double threshold)
  {
    if (value > threshold) return 1;
    if (value < -threshold) return -1;
    return 0;
  }

  static double toward_zero(double value, double amount)
  {
    if (value > 0.0) return std::max(0.0, value - amount);
    return std::min(0.0, value + amount);
  }

  void clear_error_dwell()
  {
    error_dwell_s_ = 0.0;
    error_direction_ = 0;
  }

  double torque_{};
  double error_dwell_s_{};
  double reversal_bleed_s_{};
  int error_direction_{};
  int motion_direction_{};
};
}  // namespace g1_task
