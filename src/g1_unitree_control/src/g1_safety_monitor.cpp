#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <unitree_hg/msg/low_state.hpp>

using namespace std::chrono_literals;

namespace
{
constexpr std::uint16_t kL2Mask = 0x0020;
constexpr std::uint16_t kBMask = 0x0200;
constexpr std::uint16_t kL2BMask = kL2Mask | kBMask;
constexpr std::array<std::size_t, 14> kArmMotorIndices{
  15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28
};
constexpr std::array<const char*, 14> kArmJointLabels{
  "L_sh_pitch", "L_sh_roll", "L_sh_yaw", "L_elbow", "L_wr_roll", "L_wr_pitch", "L_wr_yaw",
  "R_sh_pitch", "R_sh_roll", "R_sh_yaw", "R_elbow", "R_wr_roll", "R_wr_pitch", "R_wr_yaw"
};

std::string key_string(std::uint16_t keys)
{
  std::ostringstream stream;
  stream << "0x" << std::hex << std::setw(4) << std::setfill('0') << keys;
  return stream.str();
}
}  // namespace

class G1SafetyMonitor : public rclcpp::Node
{
public:
  G1SafetyMonitor() : Node("g1_safety_monitor")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/lowstate");
    observation_duration_s_ = std::clamp(
      declare_parameter("observation_duration_s", 12.0), 2.0, 60.0);
    observation_rate_hz_ = std::clamp(
      declare_parameter("observation_rate_hz", 10.0), 1.0, 100.0);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    state_sub_ = create_subscription<unitree_hg::msg::LowState>(
      input_topic_, qos,
      std::bind(&G1SafetyMonitor::state_callback, this, std::placeholders::_1));

    RCLCPP_WARN(
      get_logger(),
      "STRICTLY READ-ONLY: one LowState subscriber, zero application publishers or control clients");
    RCLCPP_INFO(
      get_logger(),
      "Watching %s for L2+B (mask 0x0220); support the robot before pressing the buttons",
      input_topic_.c_str());
  }

private:
  void state_callback(const unitree_hg::msg::LowState::SharedPtr message)
  {
    const auto stamp = now();
    const auto keys = static_cast<std::uint16_t>(message->wireless_remote[2]) |
      (static_cast<std::uint16_t>(message->wireless_remote[3]) << 8U);
    const bool l2 = (keys & kL2Mask) != 0;
    const bool b = (keys & kBMask) != 0;
    const bool combo = (keys & kL2BMask) == kL2BMask;

    if (!received_state_)
    {
      received_state_ = true;
      previous_keys_ = keys;
      previous_mode_machine_ = message->mode_machine;
      previous_mode_pr_ = message->mode_pr;
      for (std::size_t i = 0; i < kArmMotorIndices.size(); ++i)
        previous_motor_modes_[i] = message->motor_state[kArmMotorIndices[i]].mode;
      RCLCPP_INFO(
        get_logger(), "Initial state: keys=%s mode_machine=%u mode_pr=%u tick=%u",
        key_string(keys).c_str(), message->mode_machine, message->mode_pr, message->tick);
      log_joint_snapshot(*message, "INITIAL");
    }

    if (keys != previous_keys_)
    {
      RCLCPP_WARN(
        get_logger(), "KEY CHANGE: %s -> %s  L2=%d B=%d combo=%d tick=%u",
        key_string(previous_keys_).c_str(), key_string(keys).c_str(), l2, b, combo,
        message->tick);
      previous_keys_ = keys;
    }

    if (message->mode_machine != previous_mode_machine_ || message->mode_pr != previous_mode_pr_)
    {
      RCLCPP_WARN(
        get_logger(), "MODE CHANGE: mode_machine %u -> %u, mode_pr %u -> %u, keys=%s",
        previous_mode_machine_, message->mode_machine, previous_mode_pr_, message->mode_pr,
        key_string(keys).c_str());
      previous_mode_machine_ = message->mode_machine;
      previous_mode_pr_ = message->mode_pr;
    }

    for (std::size_t i = 0; i < kArmMotorIndices.size(); ++i)
    {
      const auto mode = message->motor_state[kArmMotorIndices[i]].mode;
      if (mode != previous_motor_modes_[i])
      {
        const double elapsed = combo_start_time_.nanoseconds() == 0 ? -1.0 :
          (stamp - combo_start_time_).seconds();
        RCLCPP_ERROR(
          get_logger(), "MOTOR MODE CHANGE: %02zu %-10s %u -> %u, t_from_combo=%.4f s",
          kArmMotorIndices[i], kArmJointLabels[i], previous_motor_modes_[i], mode, elapsed);
        previous_motor_modes_[i] = mode;
      }
    }

    if (combo && !previous_combo_)
    {
      combo_start_time_ = stamp;
      observation_end_time_ = stamp + rclcpp::Duration::from_seconds(observation_duration_s_);
      last_observation_log_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      observation_finished_ = false;
      for (std::size_t i = 0; i < kArmMotorIndices.size(); ++i)
        trigger_positions_[i] = message->motor_state[kArmMotorIndices[i]].q;
      RCLCPP_ERROR(
        get_logger(), "L2+B DETECTED: native robot response observation started for %.1f s",
        observation_duration_s_);
      log_joint_snapshot(*message, "L2+B RISING");
    }
    else if (!combo && previous_combo_)
    {
      RCLCPP_WARN(get_logger(), "L2+B RELEASED; observation remains active");
      log_joint_snapshot(*message, "L2+B FALLING");
    }
    previous_combo_ = combo;

    const bool observing = combo_start_time_.nanoseconds() != 0 && stamp <= observation_end_time_;
    const auto observation_period = rclcpp::Duration::from_seconds(1.0 / observation_rate_hz_);
    if (observing &&
        (last_observation_log_time_.nanoseconds() == 0 ||
         stamp - last_observation_log_time_ >= observation_period))
    {
      log_observation(*message, keys, combo, (stamp - combo_start_time_).seconds());
      last_observation_log_time_ = stamp;
    }
    else if (!observing && combo_start_time_.nanoseconds() != 0 && !observation_finished_)
    {
      observation_finished_ = true;
      RCLCPP_WARN(get_logger(), "OBSERVATION COMPLETE; monitor remains read-only and active");
      log_joint_snapshot(*message, "FINAL");
    }

    if (last_heartbeat_time_.nanoseconds() == 0 || stamp - last_heartbeat_time_ >= 2s)
    {
      if (!observing)
      {
        RCLCPP_INFO(
          get_logger(), "Waiting: keys=%s L2=%d B=%d mode_machine=%u mode_pr=%u tick=%u",
          key_string(keys).c_str(), l2, b, message->mode_machine, message->mode_pr,
          message->tick);
      }
      last_heartbeat_time_ = stamp;
    }
  }

  void log_observation(
    const unitree_hg::msg::LowState& state, std::uint16_t keys, bool combo, double elapsed)
  {
    float left_max_dq = 0.0F;
    float right_max_dq = 0.0F;
    float left_max_tau = 0.0F;
    float right_max_tau = 0.0F;
    float left_max_delta_q = 0.0F;
    float right_max_delta_q = 0.0F;
    for (std::size_t i = 0; i < kArmMotorIndices.size(); ++i)
    {
      const auto& motor = state.motor_state[kArmMotorIndices[i]];
      auto& max_dq = i < 7 ? left_max_dq : right_max_dq;
      auto& max_tau = i < 7 ? left_max_tau : right_max_tau;
      auto& max_delta_q = i < 7 ? left_max_delta_q : right_max_delta_q;
      max_dq = std::max(max_dq, std::fabs(motor.dq));
      max_tau = std::max(max_tau, std::fabs(motor.tau_est));
      max_delta_q = std::max(max_delta_q, std::fabs(motor.q - trigger_positions_[i]));
    }
    RCLCPP_WARN(
      get_logger(),
      "OBS t=%5.2f combo=%d keys=%s mode=%u/%u | L:dq=%.4f dQ=%.4f tau=%.3f | "
      "R:dq=%.4f dQ=%.4f tau=%.3f",
      elapsed, combo, key_string(keys).c_str(), state.mode_machine, state.mode_pr,
      left_max_dq, left_max_delta_q, left_max_tau,
      right_max_dq, right_max_delta_q, right_max_tau);
  }

  void log_joint_snapshot(const unitree_hg::msg::LowState& state, const char* reason)
  {
    RCLCPP_WARN(get_logger(), "JOINT SNAPSHOT [%s]", reason);
    for (std::size_t i = 0; i < kArmMotorIndices.size(); ++i)
    {
      const auto index = kArmMotorIndices[i];
      const auto& motor = state.motor_state[index];
      RCLCPP_WARN(
        get_logger(), "  %02zu %-10s mode=%u q=% .5f dq=% .5f tau=% .4f vol=%.2f",
        index, kArmJointLabels[i], motor.mode, motor.q, motor.dq, motor.tau_est, motor.vol);
    }
  }

  std::string input_topic_;
  double observation_duration_s_{};
  double observation_rate_hz_{};
  bool received_state_{};
  bool previous_combo_{};
  bool observation_finished_{};
  std::uint16_t previous_keys_{};
  std::uint8_t previous_mode_machine_{};
  std::uint8_t previous_mode_pr_{};
  std::array<float, 14> trigger_positions_{};
  std::array<std::uint8_t, 14> previous_motor_modes_{};
  rclcpp::Time combo_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time observation_end_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_observation_log_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_heartbeat_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr state_sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<G1SafetyMonitor>());
  rclcpp::shutdown();
  return 0;
}
