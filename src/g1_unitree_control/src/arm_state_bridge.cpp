#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <utility>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <unitree_hg/msg/low_state.hpp>

using namespace std::chrono_literals;

namespace
{
struct JointMap
{
  const char* name;
  std::size_t motor_index;
};

constexpr std::array<JointMap, 14> kArmJoints{ {
  { "left_shoulder_pitch_joint", 15 },
  { "left_shoulder_roll_joint", 16 },
  { "left_shoulder_yaw_joint", 17 },
  { "left_elbow_joint", 18 },
  { "left_wrist_roll_joint", 19 },
  { "left_wrist_pitch_joint", 20 },
  { "left_wrist_yaw_joint", 21 },
  { "right_shoulder_pitch_joint", 22 },
  { "right_shoulder_roll_joint", 23 },
  { "right_shoulder_yaw_joint", 24 },
  { "right_elbow_joint", 25 },
  { "right_wrist_roll_joint", 26 },
  { "right_wrist_pitch_joint", 27 },
  { "right_wrist_yaw_joint", 28 },
} };

constexpr std::array<JointMap, 13> kLowerBodyJoints{ {
  { "left_hip_pitch_joint", 0 },
  { "left_hip_roll_joint", 1 },
  { "left_hip_yaw_joint", 2 },
  { "left_knee_joint", 3 },
  { "left_ankle_pitch_joint", 4 },
  { "left_ankle_roll_joint", 5 },
  { "right_hip_pitch_joint", 6 },
  { "right_hip_roll_joint", 7 },
  { "right_hip_yaw_joint", 8 },
  { "right_knee_joint", 9 },
  { "right_ankle_pitch_joint", 10 },
  { "right_ankle_roll_joint", 11 },
  { "waist_yaw_joint", 12 },
} };

diagnostic_msgs::msg::KeyValue value(const std::string& key, const std::string& data)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = key;
  result.value = data;
  return result;
}
}  // namespace

class ArmStateBridge : public rclcpp::Node
{
public:
  ArmStateBridge() : Node("arm_state_bridge")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/lowstate");
    output_topic_ = declare_parameter<std::string>("output_topic", "/g1/arm_joint_states");
    diagnostics_topic_ = declare_parameter<std::string>(
        "diagnostics_topic", "/g1/arm_state_bridge/diagnostics");
    publish_rate_hz_ = std::clamp(declare_parameter("publish_rate_hz", 200.0), 1.0, 1000.0);
    state_timeout_s_ = std::clamp(declare_parameter("state_timeout_s", 0.10), 0.01, 5.0);
    expected_mode_machine_ = declare_parameter("expected_mode_machine", 6);
    include_effort_ = declare_parameter("include_effort", true);
    include_lower_body_ = declare_parameter("include_lower_body", false);
    minimum_publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_hz_);

    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
        output_topic_, rclcpp::QoS(1).reliable().durability_volatile());
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        diagnostics_topic_, rclcpp::QoS(1).reliable().durability_volatile());

    auto input_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    state_sub_ = create_subscription<unitree_hg::msg::LowState>(
        input_topic_, input_qos,
        std::bind(&ArmStateBridge::state_callback, this, std::placeholders::_1));
    diagnostics_timer_ = create_wall_timer(1s, std::bind(&ArmStateBridge::publish_diagnostics, this));

    std::ostringstream mapping;
    bool first_mapping = true;
    const auto append_mapping = [&mapping, &first_mapping](const auto& joints) {
      for (const auto& joint : joints)
      {
        if (!first_mapping)
          mapping << ", ";
        mapping << joint.name << "=" << joint.motor_index;
        first_mapping = false;
      }
    };
    if (include_lower_body_)
      append_mapping(kLowerBodyJoints);
    append_mapping(kArmJoints);
    RCLCPP_INFO(get_logger(), "READ-ONLY bridge: %s -> %s at <= %.1f Hz",
                input_topic_.c_str(), output_topic_.c_str(), publish_rate_hz_);
    RCLCPP_INFO(get_logger(), "G1 model joint mapping: %s", mapping.str().c_str());
  }

private:
  void state_callback(const unitree_hg::msg::LowState::SharedPtr message)
  {
    const auto stamp = now();
    ++received_frames_;
    last_receive_time_ = stamp;
    last_tick_ = message->tick;
    last_mode_machine_ = message->mode_machine;
    last_mode_pr_ = message->mode_pr;

    if (last_publish_time_.nanoseconds() != 0 &&
        stamp - last_publish_time_ < minimum_publish_period_)
      return;

    sensor_msgs::msg::JointState state;
    state.header.stamp = stamp;
    const std::size_t joint_count = kArmJoints.size() +
      (include_lower_body_ ? kLowerBodyJoints.size() : 0U);
    state.name.reserve(joint_count);
    state.position.reserve(joint_count);
    state.velocity.reserve(joint_count);
    if (include_effort_)
      state.effort.reserve(joint_count);

    bool valid = true;
    const auto append_joints = [&message, &state, &valid, this](const auto& joints) {
      for (const auto& joint : joints)
      {
        const auto& motor = message->motor_state[joint.motor_index];
        valid = valid && std::isfinite(motor.q) && std::isfinite(motor.dq) &&
                std::isfinite(motor.tau_est) && motor.vol > 1.0F;
        state.name.emplace_back(joint.name);
        state.position.push_back(motor.q);
        state.velocity.push_back(motor.dq);
        if (include_effort_)
          state.effort.push_back(motor.tau_est);
      }
    };
    if (include_lower_body_)
      append_joints(kLowerBodyJoints);
    append_joints(kArmJoints);

    if (!valid)
    {
      ++invalid_frames_;
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                            "Rejected LowState: non-finite arm value or inactive motor slot");
      return;
    }

    joint_state_pub_->publish(state);
    last_publish_time_ = stamp;
    ++published_frames_;
  }

  void publish_diagnostics()
  {
    const auto stamp = now();
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = stamp;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "G1 arm LowState bridge";
    status.hardware_id = "unitree_g1_29dof";

    const bool received = last_receive_time_.nanoseconds() != 0;
    const double age = received ? (stamp - last_receive_time_).seconds()
                                : std::numeric_limits<double>::infinity();
    if (!received || age > state_timeout_s_)
    {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "LowState timeout";
    }
    else if (last_mode_machine_ != expected_mode_machine_)
    {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "Unexpected mode_machine";
    }
    else if (invalid_frames_ != 0)
    {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "Invalid frames have been rejected";
    }
    else
    {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "Read-only arm state bridge healthy";
    }

    const auto received_delta = received_frames_ - previous_received_frames_;
    const auto published_delta = published_frames_ - previous_published_frames_;
    previous_received_frames_ = received_frames_;
    previous_published_frames_ = published_frames_;
    status.values.push_back(value("input_topic", input_topic_));
    status.values.push_back(value("output_topic", output_topic_));
    status.values.push_back(value("include_lower_body", include_lower_body_ ? "true" : "false"));
    status.values.push_back(value("mode_machine", std::to_string(last_mode_machine_)));
    status.values.push_back(value("mode_pr", std::to_string(last_mode_pr_)));
    status.values.push_back(value("last_tick", std::to_string(last_tick_)));
    status.values.push_back(value("state_age_s", std::to_string(age)));
    status.values.push_back(value("received_frames_last_second", std::to_string(received_delta)));
    status.values.push_back(value("published_frames_last_second", std::to_string(published_delta)));
    status.values.push_back(value("invalid_frames_total", std::to_string(invalid_frames_)));
    array.status.push_back(std::move(status));
    diagnostics_pub_->publish(array);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string diagnostics_topic_;
  double publish_rate_hz_{};
  double state_timeout_s_{};
  int expected_mode_machine_{};
  bool include_effort_{};
  bool include_lower_body_{};
  rclcpp::Duration minimum_publish_period_{ 0, 0 };
  rclcpp::Time last_receive_time_{ 0, 0, RCL_ROS_TIME };
  rclcpp::Time last_publish_time_{ 0, 0, RCL_ROS_TIME };
  std::uint64_t received_frames_{};
  std::uint64_t published_frames_{};
  std::uint64_t invalid_frames_{};
  std::uint64_t previous_received_frames_{};
  std::uint64_t previous_published_frames_{};
  std::uint32_t last_tick_{};
  int last_mode_machine_{};
  int last_mode_pr_{};
  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr state_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmStateBridge>());
  rclcpp::shutdown();
  return 0;
}
