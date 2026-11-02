#include <chrono>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <unitree_hg/msg/low_state.hpp>
#include <unitree_hg/msg/low_cmd.hpp>

// Local integration fixture only. C++ uses the same vendor DDS type support as
// the gateway; the test driver observes ordinary ROS JointState messages.
class Simulator : public rclcpp::Node
{
public:
  Simulator() : Node("task_gateway_simulator")
  {
    const auto positions = declare_parameter<std::vector<double>>("positions", std::vector<double>{});
    if (positions.size() != 14) throw std::runtime_error("14 starting arm positions required");
    const auto arm_bias = declare_parameter<std::vector<double>>("arm_tracking_bias_rad",std::vector<double>(14,0.0));
    if (arm_bias.size()!=14) throw std::runtime_error("14 arm tracking biases required");
    for (const auto value : arm_bias)
      if (!std::isfinite(value) || std::abs(value)>.06) throw std::runtime_error("Invalid test arm bias");
    state_.mode_machine = 6;
    for (std::size_t i = 15; i < 29; ++i)
    {
      state_.motor_state[i].mode = 1;
      state_.motor_state[i].vol = 52;
      state_.motor_state[i].q = positions[i-15];
    }
    state_.motor_state[12].mode = 1;
    state_.motor_state[12].vol = 52;
    state_.motor_state[12].q = declare_parameter("waist_yaw", 0.0);
    const bool include_waist = declare_parameter("include_waist", false);
    const double waist_tracking_bias_rad =
      declare_parameter("waist_tracking_bias_rad", 0.0);
    if (!std::isfinite(waist_tracking_bias_rad) ||
        std::abs(waist_tracking_bias_rad) > 0.02)
      throw std::runtime_error("waist_tracking_bias_rad must be finite and within +/-0.02 rad");
    const double moving_velocity_bias = declare_parameter("moving_velocity_bias_rad_s", 0.0);
    if (!std::isfinite(moving_velocity_bias) || moving_velocity_bias < 0.0 || moving_velocity_bias > 0.1)
      throw std::runtime_error("moving_velocity_bias_rad_s must be within [0,0.1]");
    const auto measured_velocity = [moving_velocity_bias](double commanded) {
      return std::abs(commanded) > 0.1 ? commanded + std::copysign(moving_velocity_bias, commanded) : commanded;
    };
    pub_ = create_publisher<unitree_hg::msg::LowState>("/task_test/lowstate", rclcpp::SensorDataQoS());
    observed_ = create_publisher<sensor_msgs::msg::JointState>("/task_test/command_state", 10);
    torque_observed_ = create_publisher<sensor_msgs::msg::JointState>("/task_test/command_tau", 1000);
    sub_ = create_subscription<unitree_hg::msg::LowCmd>("/task_test/command", 10,
      [this, include_waist, waist_tracking_bias_rad, measured_velocity, arm_bias](unitree_hg::msg::LowCmd::ConstSharedPtr msg) {
        sensor_msgs::msg::JointState output;
        output.header.stamp = now();
        for (std::size_t i = 15; i < 29; ++i)
        {
          state_.motor_state[i].q = msg->motor_cmd[i].q-arm_bias[i-15];
          state_.motor_state[i].dq = measured_velocity(msg->motor_cmd[i].dq);
          output.name.push_back(std::to_string(i));
          output.position.push_back(msg->motor_cmd[i].q);
          output.velocity.push_back(msg->motor_cmd[i].dq);
          output.effort.push_back(msg->motor_cmd[i].kp);
        }
        state_.motor_state[12].q = msg->motor_cmd[12].q - waist_tracking_bias_rad;
        state_.motor_state[12].dq = measured_velocity(msg->motor_cmd[12].dq);
        if (include_waist) {
          output.name.push_back("12");output.position.push_back(msg->motor_cmd[12].q);
          output.velocity.push_back(msg->motor_cmd[12].dq);
          output.effort.push_back(msg->motor_cmd[12].kp);
        }
        observed_->publish(output);
        for (std::size_t j = 0; j < output.name.size(); ++j)
          output.effort[j] = msg->motor_cmd[std::stoul(output.name[j])].tau;
        torque_observed_->publish(output);
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this] {
      state_.tick += 10;
      pub_->publish(state_);
    });
  }
private:
  unitree_hg::msg::LowState state_;
  rclcpp::Publisher<unitree_hg::msg::LowState>::SharedPtr pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr observed_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr torque_observed_;
  rclcpp::Subscription<unitree_hg::msg::LowCmd>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  const char* domain = std::getenv("ROS_DOMAIN_ID");
  const char* local = std::getenv("ROS_LOCALHOST_ONLY");
  if (!domain || !local || std::string(domain) != "87" || std::string(local) != "1") return 2;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Simulator>());
  rclcpp::shutdown();
}
