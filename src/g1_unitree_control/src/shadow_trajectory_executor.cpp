#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <unitree_hg/msg/low_state.hpp>

using namespace std::chrono_literals;

namespace
{
constexpr std::uint16_t kL2BMask = 0x0220;

struct JointSpec
{
  const char* name;
  std::size_t motor_index;
  double lower;
  double upper;
  bool left;
};

constexpr std::array<JointSpec, 14> kArmJoints{ {
  { "left_shoulder_pitch_joint", 15, -3.0892, 2.6704, true },
  { "left_shoulder_roll_joint", 16, -1.5882, 2.2515, true },
  { "left_shoulder_yaw_joint", 17, -2.6180, 2.6180, true },
  { "left_elbow_joint", 18, -1.0472, 2.0944, true },
  { "left_wrist_roll_joint", 19, -1.972222054, 1.972222054, true },
  { "left_wrist_pitch_joint", 20, -1.614429558, 1.614429558, true },
  { "left_wrist_yaw_joint", 21, -1.614429558, 1.614429558, true },
  { "right_shoulder_pitch_joint", 22, -3.0892, 2.6704, false },
  { "right_shoulder_roll_joint", 23, -2.2515, 1.5882, false },
  { "right_shoulder_yaw_joint", 24, -2.6180, 2.6180, false },
  { "right_elbow_joint", 25, -1.0472, 2.0944, false },
  { "right_wrist_roll_joint", 26, -1.972222054, 1.972222054, false },
  { "right_wrist_pitch_joint", 27, -1.614429558, 1.614429558, false },
  { "right_wrist_yaw_joint", 28, -1.614429558, 1.614429558, false },
} };

diagnostic_msgs::msg::KeyValue diagnostic_value(
  const std::string& key, const std::string& value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}

double point_time(const trajectory_msgs::msg::JointTrajectoryPoint& point)
{
  return rclcpp::Duration(point.time_from_start).seconds();
}

const JointSpec* find_joint(const std::string& name, std::size_t& arm_index)
{
  for (std::size_t i = 0; i < kArmJoints.size(); ++i)
  {
    if (name == kArmJoints[i].name)
    {
      arm_index = i;
      return &kArmJoints[i];
    }
  }
  return nullptr;
}
}  // namespace

class ShadowTrajectoryExecutor : public rclcpp::Node
{
public:
  ShadowTrajectoryExecutor() : Node("g1_shadow_trajectory_executor")
  {
    state_topic_ = declare_parameter<std::string>("state_topic", "/lowstate");
    trajectory_topic_ = declare_parameter<std::string>(
      "trajectory_topic", "/g1/shadow_executor/trajectory");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/g1/shadow_executor/commanded_joint_states");
    diagnostics_topic_ = declare_parameter<std::string>(
      "diagnostics_topic", "/g1/shadow_executor/diagnostics");
    update_rate_hz_ = std::clamp(declare_parameter("update_rate_hz", 50.0), 5.0, 250.0);
    state_timeout_s_ = std::clamp(declare_parameter("state_timeout_s", 0.10), 0.02, 2.0);
    start_tolerance_rad_ = std::clamp(
      declare_parameter("start_tolerance_rad", 0.10), 0.001, 1.0);
    tracking_error_limit_rad_ = std::clamp(
      declare_parameter("tracking_error_limit_rad", 0.15), 0.001, 2.0);
    max_velocity_rad_s_ = std::clamp(
      declare_parameter("max_velocity_rad_s", 0.50), 0.01, 2.0);
    max_acceleration_rad_s2_ = std::clamp(
      declare_parameter("max_acceleration_rad_s2", 0.75), 0.01, 5.0);
    joint_limit_margin_rad_ = std::clamp(
      declare_parameter("joint_limit_margin_rad", 0.02), 0.0, 0.20);
    enforce_tracking_error_ = declare_parameter("enforce_tracking_error", false);
    expected_mode_machine_ = declare_parameter("expected_mode_machine", 6);
    expected_arm_motor_mode_ = declare_parameter("expected_arm_motor_mode", 1);

    auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    state_sub_ = create_subscription<unitree_hg::msg::LowState>(
      state_topic_, state_qos,
      std::bind(&ShadowTrajectoryExecutor::state_callback, this, std::placeholders::_1));
    trajectory_sub_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
      trajectory_topic_, rclcpp::QoS(1).reliable().durability_volatile(),
      std::bind(&ShadowTrajectoryExecutor::trajectory_callback, this, std::placeholders::_1));

    shadow_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      output_topic_, rclcpp::QoS(1).best_effort().durability_volatile());
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_, rclcpp::QoS(1).reliable().durability_volatile());
    reset_service_ = create_service<std_srvs::srv::Trigger>(
      "/g1/shadow_executor/reset",
      std::bind(
        &ShadowTrajectoryExecutor::reset_callback, this,
        std::placeholders::_1, std::placeholders::_2));
    cancel_service_ = create_service<std_srvs::srv::Trigger>(
      "/g1/shadow_executor/cancel",
      std::bind(
        &ShadowTrajectoryExecutor::cancel_callback, this,
        std::placeholders::_1, std::placeholders::_2));

    command_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / update_rate_hz_),
      std::bind(&ShadowTrajectoryExecutor::command_timer_callback, this));
    diagnostics_timer_ = create_wall_timer(
      500ms, std::bind(&ShadowTrajectoryExecutor::diagnostics_timer_callback, this));

    RCLCPP_WARN(
      get_logger(),
      "SHADOW ONLY: publishes sensor_msgs/JointState on %s; no LowCmd, /arm_sdk, or /lowcmd path",
      output_topic_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "Waiting for one-arm trajectories on %s (v<=%.3f rad/s, a<=%.3f rad/s^2)",
      trajectory_topic_.c_str(), max_velocity_rad_s_, max_acceleration_rad_s2_);
  }

private:
  void latch_locked(const std::string& reason)
  {
    active_ = false;
    if (!safety_latched_ || latch_reason_ != reason)
      RCLCPP_ERROR(get_logger(), "SHADOW SAFETY LATCHED: %s", reason.c_str());
    safety_latched_ = true;
    latch_reason_ = reason;
    phase_ = "LATCHED";
  }

  void state_callback(const unitree_hg::msg::LowState::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto stamp = now();
    keys_ = static_cast<std::uint16_t>(message->wireless_remote[2]) |
      (static_cast<std::uint16_t>(message->wireless_remote[3]) << 8U);
    mode_machine_ = message->mode_machine;
    mode_pr_ = message->mode_pr;
    bool valid = true;
    bool normal_modes = true;
    for (std::size_t i = 0; i < kArmJoints.size(); ++i)
    {
      const auto& motor = message->motor_state[kArmJoints[i].motor_index];
      actual_position_[i] = motor.q;
      actual_velocity_[i] = motor.dq;
      motor_mode_[i] = motor.mode;
      valid = valid && std::isfinite(motor.q) && std::isfinite(motor.dq) && motor.vol > 1.0F;
      normal_modes = normal_modes && motor.mode == expected_arm_motor_mode_;
    }
    state_received_ = true;
    last_state_time_ = stamp;

    if (!valid)
      latch_locked("invalid or inactive arm motor state");
    else if ((keys_ & kL2BMask) == kL2BMask)
      latch_locked("native L2+B observed");
    else if (!normal_modes)
      latch_locked("arm motor mode is not the configured normal mode");
    else if (mode_machine_ != expected_mode_machine_)
      latch_locked("unexpected mode_machine");
  }

  bool validate_trajectory_locked(
    const trajectory_msgs::msg::JointTrajectory& trajectory,
    std::vector<std::size_t>& arm_indices, std::string& error) const
  {
    if (trajectory.joint_names.size() != 7)
    {
      error = "trajectory must contain exactly seven joints for one arm";
      return false;
    }
    if (trajectory.points.empty())
    {
      error = "trajectory has no points";
      return false;
    }

    std::set<std::string> unique_names;
    arm_indices.clear();
    arm_indices.reserve(7);
    bool side_initialized = false;
    bool left = false;
    for (const auto& name : trajectory.joint_names)
    {
      std::size_t index = 0;
      const auto* spec = find_joint(name, index);
      if (!spec)
      {
        error = "unknown or non-arm joint: " + name;
        return false;
      }
      if (!unique_names.insert(name).second)
      {
        error = "duplicate joint: " + name;
        return false;
      }
      if (!side_initialized)
      {
        left = spec->left;
        side_initialized = true;
      }
      else if (left != spec->left)
      {
        error = "trajectory mixes left and right arm joints";
        return false;
      }
      arm_indices.push_back(index);
    }

    double previous_time = -1.0;
    for (std::size_t p = 0; p < trajectory.points.size(); ++p)
    {
      const auto& point = trajectory.points[p];
      if (point.positions.size() != 7 ||
          (!point.velocities.empty() && point.velocities.size() != 7) ||
          (!point.accelerations.empty() && point.accelerations.size() != 7))
      {
        error = "point arrays do not match the seven trajectory joints";
        return false;
      }
      const double time = point_time(point);
      if (!std::isfinite(time) || time < 0.0 || (p != 0 && time <= previous_time))
      {
        error = "time_from_start must be finite, nonnegative, and strictly increasing";
        return false;
      }
      previous_time = time;

      for (std::size_t j = 0; j < 7; ++j)
      {
        const auto& spec = kArmJoints[arm_indices[j]];
        const double position = point.positions[j];
        if (!std::isfinite(position) ||
            position < spec.lower + joint_limit_margin_rad_ ||
            position > spec.upper - joint_limit_margin_rad_)
        {
          error = "position violates guarded joint limit: " + std::string(spec.name);
          return false;
        }
        if (!point.velocities.empty() &&
            (!std::isfinite(point.velocities[j]) ||
             std::abs(point.velocities[j]) > max_velocity_rad_s_ + 1e-6))
        {
          error = "declared velocity exceeds shadow limit: " + std::string(spec.name);
          return false;
        }
        if (!point.accelerations.empty() &&
            (!std::isfinite(point.accelerations[j]) ||
             std::abs(point.accelerations[j]) > max_acceleration_rad_s2_ + 1e-6))
        {
          error = "declared acceleration exceeds shadow limit: " + std::string(spec.name);
          return false;
        }
      }

      if (p > 0)
      {
        const double dt = time - point_time(trajectory.points[p - 1]);
        for (std::size_t j = 0; j < 7; ++j)
        {
          const double segment_velocity = std::abs(
            point.positions[j] - trajectory.points[p - 1].positions[j]) / dt;
          if (!std::isfinite(segment_velocity) ||
              segment_velocity > max_velocity_rad_s_ + 1e-6)
          {
            error = "segment velocity exceeds shadow limit: " +
              std::string(kArmJoints[arm_indices[j]].name);
            return false;
          }
        }
      }
    }

    if (point_time(trajectory.points.back()) <= 0.0)
    {
      error = "trajectory duration must be positive";
      return false;
    }
    for (std::size_t j = 0; j < 7; ++j)
    {
      if (std::abs(
          trajectory.points.front().positions[j] - actual_position_[arm_indices[j]]) >
          start_tolerance_rad_)
      {
        error = "trajectory start differs from current state: " +
          std::string(kArmJoints[arm_indices[j]].name);
        return false;
      }
    }
    return true;
  }

  void trajectory_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++received_trajectories_;
    if (safety_latched_)
    {
      ++rejected_trajectories_;
      RCLCPP_ERROR(get_logger(), "Rejected trajectory while latched: %s", latch_reason_.c_str());
      return;
    }
    if (!state_received_ || (now() - last_state_time_).seconds() > state_timeout_s_)
    {
      ++rejected_trajectories_;
      latch_locked("cannot accept trajectory without fresh LowState");
      return;
    }

    std::vector<std::size_t> indices;
    std::string error;
    if (!validate_trajectory_locked(*message, indices, error))
    {
      ++rejected_trajectories_;
      phase_ = "REJECTED";
      last_rejection_ = error;
      RCLCPP_ERROR(get_logger(), "Rejected shadow trajectory: %s", error.c_str());
      return;
    }

    trajectory_ = *message;
    active_arm_indices_ = std::move(indices);
    trajectory_start_time_ = now();
    for (std::size_t j = 0; j < active_arm_indices_.size(); ++j)
      acceptance_positions_[j] = actual_position_[active_arm_indices_[j]];
    active_ = true;
    phase_ = "RUNNING";
    last_rejection_.clear();
    peak_tracking_error_rad_ = 0.0;
    ++accepted_trajectories_;
    RCLCPP_WARN(
      get_logger(), "Accepted SHADOW trajectory: %s arm, %zu points, %.3f s",
      kArmJoints[active_arm_indices_.front()].left ? "left" : "right",
      trajectory_.points.size(), point_time(trajectory_.points.back()));
  }

  void command_timer_callback()
  {
    sensor_msgs::msg::JointState output;
    bool publish = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!active_)
        return;
      const auto stamp = now();
      if (!state_received_ || (stamp - last_state_time_).seconds() > state_timeout_s_)
      {
        latch_locked("LowState timeout during shadow playback");
        return;
      }

      const double elapsed = (stamp - trajectory_start_time_).seconds();
      const double final_time = point_time(trajectory_.points.back());
      std::array<double, 7> command_position{};
      std::array<double, 7> command_velocity{};
      bool finished = elapsed >= final_time;
      if (finished)
      {
        for (std::size_t j = 0; j < 7; ++j)
          command_position[j] = trajectory_.points.back().positions[j];
      }
      else
      {
        std::size_t upper = 0;
        while (upper < trajectory_.points.size() &&
               point_time(trajectory_.points[upper]) < elapsed)
          ++upper;

        const std::vector<double>* lower_positions = nullptr;
        double lower_time = 0.0;
        if (upper == 0)
        {
          lower_positions = nullptr;
        }
        else
        {
          lower_positions = &trajectory_.points[upper - 1].positions;
          lower_time = point_time(trajectory_.points[upper - 1]);
        }
        const auto& upper_point = trajectory_.points[upper];
        const double upper_time = point_time(upper_point);
        const double dt = upper_time - lower_time;
        const double ratio = dt > 1e-9 ? std::clamp((elapsed - lower_time) / dt, 0.0, 1.0) : 1.0;
        for (std::size_t j = 0; j < 7; ++j)
        {
          const double lower = lower_positions ? (*lower_positions)[j] : acceptance_positions_[j];
          command_position[j] = lower + ratio * (upper_point.positions[j] - lower);
          command_velocity[j] = dt > 1e-9 ? (upper_point.positions[j] - lower) / dt : 0.0;
        }
      }

      double tracking_error = 0.0;
      for (std::size_t j = 0; j < 7; ++j)
      {
        tracking_error = std::max(
          tracking_error,
          std::abs(command_position[j] - actual_position_[active_arm_indices_[j]]));
      }
      current_tracking_error_rad_ = tracking_error;
      peak_tracking_error_rad_ = std::max(peak_tracking_error_rad_, tracking_error);
      if (enforce_tracking_error_ && tracking_error > tracking_error_limit_rad_)
      {
        latch_locked("tracking error exceeded configured limit");
        return;
      }

      output.header.stamp = stamp;
      output.header.frame_id = "SHADOW_ONLY_NO_HARDWARE";
      output.name = trajectory_.joint_names;
      output.position.assign(command_position.begin(), command_position.end());
      output.velocity.assign(command_velocity.begin(), command_velocity.end());
      publish = true;
      if (finished)
      {
        active_ = false;
        phase_ = "COMPLETE";
        RCLCPP_WARN(
          get_logger(), "SHADOW playback complete; peak actual-vs-shadow error %.4f rad",
          peak_tracking_error_rad_);
      }
    }
    if (publish)
      shadow_pub_->publish(output);
  }

  void reset_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool fresh = state_received_ && (now() - last_state_time_).seconds() <= state_timeout_s_;
    const bool keys_released = (keys_ & kL2BMask) != kL2BMask;
    const bool modes_normal = std::all_of(
      motor_mode_.begin(), motor_mode_.end(),
      [this](std::uint8_t mode) { return mode == expected_arm_motor_mode_; });
    if (!fresh || !keys_released || !modes_normal || mode_machine_ != expected_mode_machine_)
    {
      response->success = false;
      response->message = "reset denied: require fresh state, released L2+B, and normal robot/motor modes";
      return;
    }
    safety_latched_ = false;
    latch_reason_.clear();
    last_rejection_.clear();
    active_ = false;
    phase_ = "IDLE";
    response->success = true;
    response->message = "shadow safety latch reset; no hardware command was sent";
    RCLCPP_WARN(get_logger(), "Shadow latch manually reset");
  }

  void cancel_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    active_ = false;
    phase_ = safety_latched_ ? "LATCHED" : "CANCELLED";
    response->success = true;
    response->message = "shadow playback cancelled; no hardware command was sent";
    RCLCPP_WARN(get_logger(), "Shadow playback cancelled");
  }

  void diagnostics_timer_callback()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    diagnostic_msgs::msg::DiagnosticStatus status;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto stamp = now();
      array.header.stamp = stamp;
      status.name = "G1 shadow trajectory executor";
      status.hardware_id = "NO_HARDWARE_OUTPUT";
      if (safety_latched_)
      {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        status.message = latch_reason_;
      }
      else if (!state_received_ || (stamp - last_state_time_).seconds() > state_timeout_s_)
      {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = "waiting for fresh LowState";
      }
      else
      {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        status.message = "shadow-only; no Unitree command path";
      }
      status.values.push_back(diagnostic_value("phase", phase_));
      status.values.push_back(diagnostic_value("state_topic", state_topic_));
      status.values.push_back(diagnostic_value("trajectory_topic", trajectory_topic_));
      status.values.push_back(diagnostic_value("output_topic", output_topic_));
      status.values.push_back(diagnostic_value("mode_machine", std::to_string(mode_machine_)));
      status.values.push_back(diagnostic_value("mode_pr", std::to_string(mode_pr_)));
      status.values.push_back(diagnostic_value("keys", std::to_string(keys_)));
      status.values.push_back(diagnostic_value(
        "tracking_error_enforced", enforce_tracking_error_ ? "true" : "false"));
      status.values.push_back(diagnostic_value(
        "current_tracking_error_rad", std::to_string(current_tracking_error_rad_)));
      status.values.push_back(diagnostic_value(
        "peak_tracking_error_rad", std::to_string(peak_tracking_error_rad_)));
      status.values.push_back(diagnostic_value(
        "received_trajectories", std::to_string(received_trajectories_)));
      status.values.push_back(diagnostic_value(
        "accepted_trajectories", std::to_string(accepted_trajectories_)));
      status.values.push_back(diagnostic_value(
        "rejected_trajectories", std::to_string(rejected_trajectories_)));
      status.values.push_back(diagnostic_value("last_rejection", last_rejection_));
    }
    array.status.push_back(std::move(status));
    diagnostics_pub_->publish(array);
  }

  std::mutex mutex_;
  std::string state_topic_;
  std::string trajectory_topic_;
  std::string output_topic_;
  std::string diagnostics_topic_;
  double update_rate_hz_{};
  double state_timeout_s_{};
  double start_tolerance_rad_{};
  double tracking_error_limit_rad_{};
  double max_velocity_rad_s_{};
  double max_acceleration_rad_s2_{};
  double joint_limit_margin_rad_{};
  bool enforce_tracking_error_{};
  int expected_mode_machine_{};
  int expected_arm_motor_mode_{};
  bool state_received_{};
  bool safety_latched_{};
  bool active_{};
  std::string phase_{"WAITING_FOR_STATE"};
  std::string latch_reason_;
  std::string last_rejection_;
  std::uint16_t keys_{};
  std::uint8_t mode_machine_{};
  std::uint8_t mode_pr_{};
  std::array<double, 14> actual_position_{};
  std::array<double, 14> actual_velocity_{};
  std::array<std::uint8_t, 14> motor_mode_{};
  std::array<double, 7> acceptance_positions_{};
  double current_tracking_error_rad_{};
  double peak_tracking_error_rad_{};
  std::uint64_t received_trajectories_{};
  std::uint64_t accepted_trajectories_{};
  std::uint64_t rejected_trajectories_{};
  rclcpp::Time last_state_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time trajectory_start_time_{0, 0, RCL_ROS_TIME};
  trajectory_msgs::msg::JointTrajectory trajectory_;
  std::vector<std::size_t> active_arm_indices_;
  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr state_sub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr shadow_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_service_;
  rclcpp::TimerBase::SharedPtr command_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ShadowTrajectoryExecutor>());
  rclcpp::shutdown();
  return 0;
}
