#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/idl/hg/SportModeState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/go2/robot_state/robot_state_client.hpp>

namespace
{
using Clock = std::chrono::steady_clock;
using LowCmd = unitree_hg::msg::dds_::LowCmd_;
using LowState = unitree_hg::msg::dds_::LowState_;
using SportModeState = unitree_hg::msg::dds_::SportModeState_;

constexpr char kStateTopic[] = "rt/lowstate";
constexpr char kSportStateTopic[] = "rt/sportmodestate";
constexpr char kArmSdkTopic[] = "rt/arm_sdk";
constexpr char kAuthorization[] = "I_UNDERSTAND_ARM_SDK_OUTPUT";
constexpr std::uint16_t kL2BMask = 0x0220;
constexpr std::size_t kWaistYaw = 12;
constexpr std::size_t kWaistRoll = 13;
constexpr std::size_t kWaistPitch = 14;
constexpr std::size_t kLeftElbow = 18;
constexpr std::size_t kWeightIndex = 29;
constexpr std::array<std::size_t, 15> kControlledJoints{
  kWaistYaw,
  15, 16, 17, 18, 19, 20, 21,
  22, 23, 24, 25, 26, 27, 28
};
constexpr std::array<std::size_t, 2> kLockedWaistJoints{kWaistRoll, kWaistPitch};

constexpr double kControlDt = 0.02;
constexpr double kArmKp = 60.0;
constexpr double kArmKd = 1.5;
constexpr double kWaistKp = 60.0;
constexpr double kWaistKd = 1.5;
constexpr double kTargetWeight = 1.0;
constexpr double kMoveSeconds = 4.0;
constexpr double kHoldSeconds = 3.0;
constexpr double kReturnSeconds = 4.0;
constexpr double kReleaseSeconds = 2.0;
constexpr double kElbowDeltaRad = 2.0 * 3.14159265358979323846 / 180.0;
constexpr double kStateTimeoutSeconds = 0.10;
constexpr double kSportStateTimeoutSeconds = 0.20;
constexpr int kRequiredFsmId = 801;
constexpr std::array<int, 2> kAllowedFsmModes{0, 3};
constexpr double kMaxVelocity = 2.50;
constexpr double kMaxArmTauEst = 12.0;
constexpr double kMaxWaistTauEst = 25.0;
constexpr double kMaxCasingTemperature = 85.0;
constexpr double kMaxWindingTemperature = 120.0;
constexpr double kMaxTrackingError = 3.0 * 3.14159265358979323846 / 180.0;
constexpr double kSettledTrackingError = 1.0 * 3.14159265358979323846 / 180.0;
constexpr double kTakeoverTransientSeconds = 0.8;
constexpr double kTrackingPersistenceSeconds = 0.3;
constexpr double kTrackingImprovementRad =
  0.03 * 3.14159265358979323846 / 180.0;
constexpr double kTauCalibrationSeconds = 1.5;
constexpr double kMaxTauCalibrationSpread = 0.75;
// The installed right-hand extension reaches just over 4 Nm in some screened
// static poses. Keep takeover feedforward well below the independent 12 Nm
// measured-torque trip while allowing an undisturbed takeover there.
constexpr double kMaxArmTauFeedforward = 6.0;

std::atomic_bool g_stop_requested{false};

void signal_handler(int)
{
  g_stop_requested.store(true);
}

struct SharedState
{
  std::mutex mutex;
  LowState message;
  Clock::time_point received_at{};
  std::uint64_t samples = 0;
  SportModeState sport_message;
  Clock::time_point sport_received_at{};
  std::uint64_t sport_samples = 0;
};

struct SafetyResult
{
  bool safe = false;
  bool l2_b = false;
  std::string reason;
  double max_velocity = 0.0;
  double max_arm_tau = 0.0;
  double waist_tau = 0.0;
  double max_temperature = 0.0;
  std::size_t max_temperature_joint = 0;
  double max_casing_temperature = 0.0;
  std::size_t max_casing_temperature_joint = 0;
  double max_winding_temperature = 0.0;
  std::size_t max_winding_temperature_joint = 0;
};

bool check_onboard_services(std::string& reason)
{
  unitree::robot::go2::RobotStateClient client;
  client.SetTimeout(3.0F);
  client.Init();
  std::vector<unitree::robot::go2::ServiceState> services;
  const auto result = client.ServiceList(services);
  if (result != 0)
  {
    reason = "ServiceList failed with code " + std::to_string(result);
    return false;
  }

  const auto status_of = [&services](const std::string& name, int32_t& status) {
    const auto found = std::find_if(services.begin(), services.end(),
      [&name](const auto& service) { return service.name == name; });
    if (found == services.end())
      return false;
    status = found->status;
    return true;
  };

  int32_t status = -1;
  if (!status_of("g1_arm_example", status) || status != 0)
  {
    reason = "g1_arm_example is missing or inactive";
    return false;
  }
  if (!status_of("ai_sport", status) || status != 0)
  {
    reason = "ai_sport is missing or not running";
    return false;
  }
  for (const auto* name : {"auto_test_arm", "auto_test_low"})
  {
    if (!status_of(name, status) || status != 1)
    {
      reason = std::string(name) + " is missing or not stopped";
      return false;
    }
  }
  std::cout << "ServiceList safe: g1_arm_example and ai_sport active; "
            << "auto_test_arm/auto_test_low stopped (" << services.size()
            << " services reported)."
            << std::endl;
  return true;
}

bool check_high_level_motion(SharedState& shared, std::string& reason,
                             bool print = false)
{
  std::lock_guard<std::mutex> lock(shared.mutex);
  if (shared.sport_samples == 0)
  {
    reason = "no rt/sportmodestate samples";
    return false;
  }
  const auto age = std::chrono::duration<double>(
    Clock::now() - shared.sport_received_at).count();
  if (age > kSportStateTimeoutSeconds)
  {
    reason = "SportModeState stale (age=" + std::to_string(age) + " s)";
    return false;
  }
  const int fsm_id = shared.sport_message.fsm_id();
  const int fsm_mode = shared.sport_message.fsm_mode();
  const bool allowed_mode = std::find(kAllowedFsmModes.begin(),
    kAllowedFsmModes.end(), fsm_mode) != kAllowedFsmModes.end();
  if (fsm_id != kRequiredFsmId || !allowed_mode)
  {
    reason = "unexpected high-level motion state (fsm_id=" +
      std::to_string(fsm_id) + ", fsm_mode=" + std::to_string(fsm_mode) + ")";
    return false;
  }
  if (print)
  {
    std::cout << "High-level motion safe: SportModeState samples="
              << shared.sport_samples << " fsm_id=" << fsm_id
              << " fsm_mode=" << fsm_mode << std::endl;
  }
  return true;
}

LowState snapshot(SharedState& shared, Clock::time_point& received_at,
                  std::uint64_t& samples)
{
  std::lock_guard<std::mutex> lock(shared.mutex);
  received_at = shared.received_at;
  samples = shared.samples;
  return shared.message;
}

SafetyResult check_state(const LowState& state, Clock::time_point received_at,
                         bool require_active_modes,
                         double max_velocity = kMaxVelocity)
{
  SafetyResult result;
  const auto age = std::chrono::duration<double>(Clock::now() - received_at).count();
  if (age > kStateTimeoutSeconds)
  {
    result.reason = "LowState stale (age=" + std::to_string(age) + " s)";
    return result;
  }

  const auto& remote = state.wireless_remote();
  const auto keys = static_cast<std::uint16_t>(remote[2]) |
    (static_cast<std::uint16_t>(remote[3]) << 8U);
  result.l2_b = (keys & kL2BMask) == kL2BMask;
  for (const auto index : kLockedWaistJoints)
  {
    const auto& motor = state.motor_state().at(index);
    if (!std::isfinite(motor.q()) || !std::isfinite(motor.dq()) ||
        !std::isfinite(motor.tau_est()))
    {
      result.reason = "invalid locked-waist state at index " +
        std::to_string(index);
      return result;
    }
    if (motor.mode() != 0U || motor.vol() > 1.0F)
    {
      result.reason = "locked waist joint unexpectedly active at index " +
        std::to_string(index);
      return result;
    }
  }

  for (const auto index : kControlledJoints)
  {
    const auto& motor = state.motor_state().at(index);
    const bool arm = index != kWaistYaw;
    result.max_velocity = std::max(result.max_velocity,
      std::abs(static_cast<double>(motor.dq())));
    if (arm)
      result.max_arm_tau = std::max(result.max_arm_tau,
        std::abs(static_cast<double>(motor.tau_est())));
    else
      result.waist_tau = std::max(result.waist_tau,
        std::abs(static_cast<double>(motor.tau_est())));
    const double temperature = static_cast<double>(
      std::max(motor.temperature()[0], motor.temperature()[1]));
    const double casing_temperature = static_cast<double>(motor.temperature()[0]);
    const double winding_temperature = static_cast<double>(motor.temperature()[1]);
    if (temperature > result.max_temperature)
    {
      result.max_temperature = temperature;
      result.max_temperature_joint = index;
    }
    if (casing_temperature > result.max_casing_temperature)
    {
      result.max_casing_temperature = casing_temperature;
      result.max_casing_temperature_joint = index;
    }
    if (winding_temperature > result.max_winding_temperature)
    {
      result.max_winding_temperature = winding_temperature;
      result.max_winding_temperature_joint = index;
    }

    if (!std::isfinite(motor.q()) || !std::isfinite(motor.dq()) ||
        !std::isfinite(motor.tau_est()) || motor.vol() <= 1.0F ||
        motor.motorstate() != 0U)
    {
      result.reason = "invalid/inactive motor state at index " + std::to_string(index);
      return result;
    }
    if (require_active_modes && motor.mode() != 1U)
    {
      result.reason = "motor mode is not 1 at index " + std::to_string(index) +
        " (mode=" + std::to_string(motor.mode()) + ")";
      return result;
    }
  }

  if (result.l2_b)
    result.reason = "native L2+B observed";
  else if (result.max_velocity > max_velocity)
    result.reason = "upper-body velocity limit exceeded";
  else if (result.max_arm_tau > kMaxArmTauEst)
    result.reason = "arm tau_est limit exceeded";
  else if (result.waist_tau > kMaxWaistTauEst)
    result.reason = "waist tau_est limit exceeded";
  else if (result.max_casing_temperature >= kMaxCasingTemperature)
    result.reason = "motor casing temperature reached official protection limit";
  else if (result.max_winding_temperature >= kMaxWindingTemperature)
    result.reason = "motor winding temperature reached official protection limit";
  else
    result.safe = true;
  return result;
}

bool collect_tau_feedforward(SharedState& shared,
                             std::array<float, 35>& tau_feedforward,
                             std::string& reason, bool print,
                             const std::array<double, 35>* per_joint_limits = nullptr)
{
  std::array<std::vector<double>, 35> samples_by_joint;
  const auto deadline = Clock::now() +
    std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(kTauCalibrationSeconds));
  std::uint64_t previous_samples = 0;
  while (Clock::now() < deadline)
  {
    Clock::time_point received_at;
    std::uint64_t samples = 0;
    const auto state = snapshot(shared, received_at, samples);
    const auto safety = check_state(state, received_at, true);
    if (!safety.safe || samples == 0 || samples == previous_samples)
    {
      reason = !safety.safe ? safety.reason : "LowState did not update during tau calibration";
      return false;
    }
    previous_samples = samples;
    for (const auto index : kControlledJoints)
    {
      if (index != kWaistYaw)
        samples_by_joint.at(index).push_back(state.motor_state().at(index).tau_est());
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  tau_feedforward.fill(0.0F);
  if (print)
    std::cout << "TAU_EST_CALIBRATION";
  for (const auto index : kControlledJoints)
  {
    if (index == kWaistYaw)
      continue;
    auto& values = samples_by_joint.at(index);
    if (values.size() < 20)
    {
      reason = "insufficient tau_est samples at joint " + std::to_string(index);
      return false;
    }
    std::sort(values.begin(), values.end());
    const double median = values.at(values.size() / 2);
    const double low = values.at(values.size() / 10);
    const double high = values.at(values.size() * 9 / 10);
    const double spread = high - low;
    const double limit = per_joint_limits
      ? per_joint_limits->at(index) : kMaxArmTauFeedforward;
    if (!std::isfinite(median) || spread > kMaxTauCalibrationSpread ||
        std::abs(median) > limit)
    {
      reason = "unstable/out-of-range tau_est calibration at joint " +
        std::to_string(index) + " (median=" + std::to_string(median) +
        ", spread=" + std::to_string(spread) +
        ", limit=" + std::to_string(limit) + ")";
      return false;
    }
    tau_feedforward.at(index) = static_cast<float>(median);
    if (print)
      std::cout << " j" << index << "_median=" << median << "_spread=" << spread;
  }
  if (print)
    std::cout << " waist_yaw_ff=0" << std::endl;
  return true;
}

void print_state(const LowState& state, const SafetyResult& safety,
                 std::uint64_t samples)
{
  const auto& elbow = state.motor_state().at(kLeftElbow);
  std::cout << std::fixed << std::setprecision(4)
            << "samples=" << samples
            << " mode_machine=" << static_cast<int>(state.mode_machine())
            << " left_elbow_q=" << elbow.q()
            << " dq_max=" << safety.max_velocity
            << " arm_tau_max=" << safety.max_arm_tau
            << " waist_tau=" << safety.waist_tau
            << " temp_max=" << safety.max_temperature
            << " temp_joint=" << safety.max_temperature_joint
            << " casing_max=" << safety.max_casing_temperature
            << " casing_joint=" << safety.max_casing_temperature_joint
            << " winding_max=" << safety.max_winding_temperature
            << " winding_joint=" << safety.max_winding_temperature_joint
            << " safe=" << (safety.safe ? "true" : "false");
  if (!safety.safe)
    std::cout << " reason=\"" << safety.reason << '"';
  std::cout << std::endl;
}

void fill_command(LowCmd& command, const std::array<float, 35>& desired,
                  const std::array<float, 35>& tau_feedforward, float weight)
{
  for (const auto index : kControlledJoints)
  {
    auto& motor = command.motor_cmd().at(index);
    motor.q(desired.at(index));
    motor.dq(0.0F);
    motor.tau(tau_feedforward.at(index) * weight);
    const bool waist = index == kWaistYaw;
    motor.kp(static_cast<float>(waist ? kWaistKp : kArmKp));
    motor.kd(static_cast<float>(waist ? kWaistKd : kArmKd));
  }
  command.motor_cmd().at(kWeightIndex).q(weight);
  // The official G1 arm_sdk example intentionally does not add LowCmd CRC.
}

double quintic(double x)
{
  x = std::clamp(x, 0.0, 1.0);
  return x * x * x * (10.0 + x * (-15.0 + 6.0 * x));
}

int monitor(SharedState& shared)
{
  std::cout << "READ-ONLY monitor: no rt/arm_sdk publisher will be created." << std::endl;
  const auto deadline = Clock::now() + std::chrono::seconds(5);
  auto next_print = Clock::now();
  while (Clock::now() < deadline && !g_stop_requested.load())
  {
    Clock::time_point received_at;
    std::uint64_t samples = 0;
    const auto state = snapshot(shared, received_at, samples);
    if (Clock::now() >= next_print)
    {
      if (samples == 0)
        std::cout << "Waiting for rt/lowstate ..." << std::endl;
      else
        print_state(state, check_state(state, received_at, true), samples);
      next_print += std::chrono::milliseconds(500);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  std::string service_reason;
  if (!check_onboard_services(service_reason))
  {
    std::cerr << "Service preflight unsafe: " << service_reason << std::endl;
    return 2;
  }
  std::string motion_reason;
  if (!check_high_level_motion(shared, motion_reason, true))
  {
    std::cerr << "High-level motion preflight unsafe: " << motion_reason << std::endl;
    return 2;
  }
  std::array<float, 35> tau_feedforward{};
  std::string tau_reason;
  if (!collect_tau_feedforward(shared, tau_feedforward, tau_reason, true))
  {
    std::cerr << "Read-only tau_est calibration failed: " << tau_reason << std::endl;
    return 2;
  }
  return 0;
}

int execute(SharedState& shared, bool move_enabled)
{
  std::cout << "EXECUTION requested, but publisher is not created yet.\n"
            << "Preflight requires 2 seconds of fresh, active, safe upper-body state."
            << std::endl;
  std::string service_reason;
  if (!check_onboard_services(service_reason))
  {
    std::cerr << "Preflight failed: " << service_reason << std::endl;
    return 2;
  }
  std::string motion_reason;
  if (!check_high_level_motion(shared, motion_reason, true))
  {
    std::cerr << "Preflight failed: " << motion_reason << std::endl;
    return 2;
  }
  const auto preflight_end = Clock::now() + std::chrono::seconds(2);
  std::uint64_t previous_samples = 0;
  while (Clock::now() < preflight_end)
  {
    Clock::time_point received_at;
    std::uint64_t samples = 0;
    const auto state = snapshot(shared, received_at, samples);
    if (samples == 0 || samples == previous_samples)
    {
      std::cerr << "Preflight failed: LowState is not updating" << std::endl;
      return 2;
    }
    previous_samples = samples;
    const auto safety = check_state(state, received_at, true);
    if (!safety.safe)
    {
      print_state(state, safety, samples);
      std::cerr << "Preflight failed: " << safety.reason << std::endl;
      return 2;
    }
    if (!check_high_level_motion(shared, motion_reason))
    {
      std::cerr << "Preflight failed: " << motion_reason << std::endl;
      return 2;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  Clock::time_point received_at;
  std::uint64_t samples = 0;
  auto state = snapshot(shared, received_at, samples);
  auto safety = check_state(state, received_at, true);
  print_state(state, safety, samples);
  std::array<float, 35> tau_feedforward{};
  std::string tau_reason;
  if (!collect_tau_feedforward(shared, tau_feedforward, tau_reason, true))
  {
    std::cerr << "Pre-prompt tau_est calibration failed: " << tau_reason << std::endl;
    return 2;
  }
  std::cout << "Exact sequence: first command uses weight=1 at the freshly captured pose; ";
  if (move_enabled)
    std::cout << "left elbow +2 deg in 4 s; hold " << kHoldSeconds
              << " s; return to initial pose in "
              << "4 s; weight 1->0 in 2 s.\n";
  else
    std::cout << "hold " << kHoldSeconds
              << " s without commanded joint motion; weight 1->0 in 2 s.\n";
  std::cout << "Kp/Kd: arms 60/1.5, waist yaw 60/1.5; takeover allows up to "
            << "3 deg for 0.8 s, then >1 deg without improvement for 0.3 s releases; "
            << "locked waist roll/pitch are "
            << "verified inactive and not commanded; 50 Hz; no leg commands.\n"
            << "Arm tau_ff uses the freshly sampled median tau_est, limited to +/-4 Nm; "
            << "waist yaw tau_ff remains zero.\n"
            << "L2+B, stale state, mode/fault, velocity, tau_est, temperature, or "
            << "tracking violation stops publishing immediately.\n"
            << "Type START and press ENTER to create the publisher: " << std::flush;
  std::string confirmation;
  std::getline(std::cin, confirmation);
  if (confirmation != "START")
  {
    std::cout << "Cancelled; no publisher was created." << std::endl;
    return 0;
  }

  state = snapshot(shared, received_at, samples);
  safety = check_state(state, received_at, true);
  if (!safety.safe)
  {
    std::cerr << "Final pre-publisher check failed: " << safety.reason << std::endl;
    return 2;
  }
  if (!check_onboard_services(service_reason))
  {
    std::cerr << "Final service check failed: " << service_reason << std::endl;
    return 2;
  }
  if (!check_high_level_motion(shared, motion_reason, true))
  {
    std::cerr << "Final high-level motion check failed: " << motion_reason << std::endl;
    return 2;
  }
  if (!collect_tau_feedforward(shared, tau_feedforward, tau_reason, true))
  {
    std::cerr << "Final tau_est calibration failed: " << tau_reason << std::endl;
    return 2;
  }
  state = snapshot(shared, received_at, samples);
  safety = check_state(state, received_at, true);
  if (!safety.safe)
  {
    std::cerr << "Final pose-capture check failed: " << safety.reason << std::endl;
    return 2;
  }

  std::array<float, 35> initial{};
  std::array<float, 35> desired{};
  for (const auto index : kControlledJoints)
    initial.at(index) = desired.at(index) = state.motor_state().at(index).q();
  const double elbow_target = initial.at(kLeftElbow) + kElbowDeltaRad;
  if (elbow_target > 2.0944)
  {
    std::cerr << "Target would exceed the left elbow upper limit; no publisher created."
              << std::endl;
    return 2;
  }

  auto publisher = std::make_unique<unitree::robot::ChannelPublisher<LowCmd>>(kArmSdkTopic);
  publisher->InitChannel();
  LowCmd command;
  std::uint64_t writes = 0;
  double peak_velocity = 0.0;
  double peak_arm_tau = 0.0;
  double peak_waist_tau = 0.0;
  double peak_tracking = 0.0;
  std::size_t peak_tracking_joint = kControlledJoints.front();
  std::array<double, 35> peak_tracking_by_joint{};
  std::array<double, 35> signed_peak_tracking_by_joint{};
  float last_commanded_weight = 0.0F;
  bool latched_silent = false;
  std::string stop_reason;

  const auto run_phase = [&](const std::string& name, double duration,
                             bool enforce_takeover_settling,
                             const auto& update_desired_and_weight) {
    std::cout << "PHASE " << name << std::endl;
    const auto started = Clock::now();
    auto next_tick = started;
    auto next_print = started;
    Clock::time_point soft_violation_since{};
    double best_soft_error = std::numeric_limits<double>::infinity();
    while (!g_stop_requested.load())
    {
      const auto now = Clock::now();
      const double elapsed = std::chrono::duration<double>(now - started).count();
      if (elapsed >= duration)
        return true;

      state = snapshot(shared, received_at, samples);
      safety = check_state(state, received_at, true);
      peak_velocity = std::max(peak_velocity, safety.max_velocity);
      peak_arm_tau = std::max(peak_arm_tau, safety.max_arm_tau);
      peak_waist_tau = std::max(peak_waist_tau, safety.waist_tau);
      if (!safety.safe)
      {
        latched_silent = true;
        stop_reason = safety.reason;
        return false;
      }
      if (!check_high_level_motion(shared, motion_reason))
      {
        latched_silent = true;
        stop_reason = motion_reason;
        return false;
      }

      float weight = 0.0F;
      update_desired_and_weight(std::clamp(elapsed / duration, 0.0, 1.0), weight);
      double tracking = 0.0;
      double signed_tracking = 0.0;
      std::size_t tracking_joint = kControlledJoints.front();
      for (const auto index : kControlledJoints)
      {
        const double signed_error = static_cast<double>(desired.at(index)) -
          static_cast<double>(state.motor_state().at(index).q());
        const double error = std::abs(signed_error);
        if (error > peak_tracking_by_joint.at(index))
        {
          peak_tracking_by_joint.at(index) = error;
          signed_peak_tracking_by_joint.at(index) = signed_error;
        }
        if (error > tracking)
        {
          tracking = error;
          signed_tracking = signed_error;
          tracking_joint = index;
        }
      }
      if (tracking > peak_tracking)
      {
        peak_tracking = tracking;
        peak_tracking_joint = tracking_joint;
      }
      if (tracking > kMaxTrackingError)
      {
        latched_silent = true;
        stop_reason = "tracking error exceeded hard limit";
        return false;
      }
      if (enforce_takeover_settling && elapsed >= kTakeoverTransientSeconds)
      {
        if (tracking <= kSettledTrackingError)
        {
          soft_violation_since = Clock::time_point{};
          best_soft_error = std::numeric_limits<double>::infinity();
        }
        else if (soft_violation_since == Clock::time_point{} ||
                 tracking < best_soft_error - kTrackingImprovementRad)
        {
          soft_violation_since = now;
          best_soft_error = tracking;
        }
        else
        {
          best_soft_error = std::min(best_soft_error, tracking);
          const double persistent_for = std::chrono::duration<double>(
            now - soft_violation_since).count();
          if (persistent_for >= kTrackingPersistenceSeconds)
          {
            stop_reason = "settled tracking error did not improve at joint " +
              std::to_string(tracking_joint) + " (error=" +
              std::to_string(tracking) + " rad)";
            return false;
          }
        }
      }

      fill_command(command, desired, tau_feedforward, weight);
      publisher->Write(command);
      last_commanded_weight = weight;
      ++writes;
      if (now >= next_print)
      {
        std::cout << std::fixed << std::setprecision(4)
                  << "  t=" << elapsed << " weight=" << weight
                  << " elbow_actual=" << state.motor_state().at(kLeftElbow).q()
                  << " elbow_cmd=" << desired.at(kLeftElbow)
                  << " tracking_max=" << tracking
                  << " tracking_signed=" << signed_tracking
                  << " tracking_joint=" << tracking_joint
                  << " tau_max=" << safety.max_arm_tau << std::endl;
        next_print += std::chrono::milliseconds(100);
      }
      next_tick += std::chrono::milliseconds(20);
      std::this_thread::sleep_until(next_tick);
    }
    stop_reason = "SIGINT requested";
    return false;
  };

  bool completed = run_phase("IMMEDIATE_WEIGHT_1_HOLD", kHoldSeconds,
    true,
    [&](double, float& weight) {
      desired = initial;
      weight = static_cast<float>(kTargetWeight);
    });
  if (completed && move_enabled)
    completed = run_phase("LEFT_ELBOW_PLUS_2_DEG", kMoveSeconds,
      false,
      [&](double phase, float& weight) {
        desired = initial;
        desired.at(kLeftElbow) = static_cast<float>(
          initial.at(kLeftElbow) + kElbowDeltaRad * quintic(phase));
        weight = static_cast<float>(kTargetWeight);
      });
  if (completed && move_enabled)
    completed = run_phase("TARGET_HOLD", kHoldSeconds, false,
      [&](double, float& weight) {
        desired = initial;
        desired.at(kLeftElbow) = static_cast<float>(elbow_target);
        weight = static_cast<float>(kTargetWeight);
      });
  if (completed && move_enabled)
    completed = run_phase("RETURN_TO_INITIAL", kReturnSeconds,
      false,
      [&](double phase, float& weight) {
        desired = initial;
        desired.at(kLeftElbow) = static_cast<float>(
          elbow_target + (initial.at(kLeftElbow) - elbow_target) * quintic(phase));
        weight = static_cast<float>(kTargetWeight);
      });

  if (!latched_silent)
  {
    if (!completed)
      std::cout << "Controlled release after: " << stop_reason << std::endl;
    g_stop_requested.store(false);
    const float release_start_weight = last_commanded_weight;
    const auto release_start = Clock::now();
    auto next_tick = release_start;
    while (true)
    {
      if (g_stop_requested.load())
      {
        latched_silent = true;
        stop_reason = "second signal received during controlled release";
        break;
      }
      const double elapsed = std::chrono::duration<double>(
        Clock::now() - release_start).count();
      if (elapsed >= kReleaseSeconds)
        break;
      state = snapshot(shared, received_at, samples);
      safety = check_state(state, received_at, true);
      if (!safety.safe)
      {
        latched_silent = true;
        stop_reason = safety.reason;
        break;
      }
      if (!check_high_level_motion(shared, motion_reason))
      {
        latched_silent = true;
        stop_reason = motion_reason;
        break;
      }
      const float weight = static_cast<float>(
        release_start_weight * (1.0 - quintic(elapsed / kReleaseSeconds)));
      fill_command(command, desired, tau_feedforward, weight);
      publisher->Write(command);
      last_commanded_weight = weight;
      ++writes;
      next_tick += std::chrono::milliseconds(20);
      std::this_thread::sleep_until(next_tick);
    }
    if (!latched_silent)
    {
      fill_command(command, desired, tau_feedforward, 0.0F);
      publisher->Write(command);
      ++writes;
    }
  }

  if (latched_silent)
    std::cerr << "LATCHED_SILENT: " << stop_reason
              << "; no further command samples sent." << std::endl;
  else
    std::cout << "Normal weight release complete; command publishing stopped." << std::endl;
  std::cout << "SUMMARY writes=" << writes
            << " peak_dq=" << peak_velocity
            << " peak_arm_tau=" << peak_arm_tau
            << " peak_waist_tau=" << peak_waist_tau
            << " peak_tracking=" << peak_tracking
            << " peak_tracking_joint=" << peak_tracking_joint << std::endl;
  std::cout << "PER_JOINT_PEAK_TRACKING";
  for (const auto index : kControlledJoints)
    std::cout << " j" << index << "_abs=" << peak_tracking_by_joint.at(index)
              << "_signed=" << signed_peak_tracking_by_joint.at(index);
  std::cout << std::endl;
  return completed && !latched_silent ? 0 : 3;
}
}  // namespace

int main(int argc, char** argv)
{
  if (argc != 2 && argc != 4)
  {
    std::cerr << "Usage:\n  " << argv[0] << " NETWORK_INTERFACE\n  "
              << argv[0] << " NETWORK_INTERFACE --execute-hold " << kAuthorization
              << "\n  " << argv[0] << " NETWORK_INTERFACE --execute-move "
              << kAuthorization
              << std::endl;
    return 1;
  }
  const bool hold_requested = argc == 4 && std::string(argv[2]) == "--execute-hold" &&
    std::string(argv[3]) == kAuthorization;
  const bool move_requested = argc == 4 && std::string(argv[2]) == "--execute-move" &&
    std::string(argv[3]) == kAuthorization;
  const bool execute_requested = hold_requested || move_requested;
  if (argc == 4 && !execute_requested)
  {
    std::cerr << "Invalid execution authorization." << std::endl;
    return 1;
  }

  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);
  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);

  SharedState shared;
  auto subscriber = std::make_unique<unitree::robot::ChannelSubscriber<LowState>>(kStateTopic);
  subscriber->InitChannel([&shared](const void* data) {
    const auto* incoming = static_cast<const LowState*>(data);
    std::lock_guard<std::mutex> lock(shared.mutex);
    shared.message = *incoming;
    shared.received_at = Clock::now();
    ++shared.samples;
  }, 1);
  auto sport_subscriber =
    std::make_unique<unitree::robot::ChannelSubscriber<SportModeState>>(kSportStateTopic);
  sport_subscriber->InitChannel([&shared](const void* data) {
    const auto* incoming = static_cast<const SportModeState*>(data);
    std::lock_guard<std::mutex> lock(shared.mutex);
    shared.sport_message = *incoming;
    shared.sport_received_at = Clock::now();
    ++shared.sport_samples;
  }, 1);

  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  return execute_requested ? execute(shared, move_requested) : monitor(shared);
}
