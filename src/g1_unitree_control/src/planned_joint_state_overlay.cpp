#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class PlannedJointStateOverlay : public rclcpp::Node
{
public:
  PlannedJointStateOverlay() : Node("g1_planned_joint_state_overlay")
  {
    const auto actual_topic = declare_parameter<std::string>(
      "actual_topic", "/joint_states");
    const auto reference_topic = declare_parameter<std::string>(
      "reference_topic", "/g1/arm_gateway/command_preview");
    const auto output_topic = declare_parameter<std::string>(
      "output_topic", "/g1/planned_joint_states");

    reference_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      reference_topic, rclcpp::QoS(10).reliable().durability_volatile(),
      std::bind(&PlannedJointStateOverlay::reference_callback, this,
        std::placeholders::_1));
    actual_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      actual_topic, rclcpp::SensorDataQoS(),
      std::bind(&PlannedJointStateOverlay::actual_callback, this,
        std::placeholders::_1));
    output_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      output_topic, rclcpp::QoS(10).best_effort().durability_volatile());
    RCLCPP_INFO(
      get_logger(), "Overlaying %s with %s on %s",
      actual_topic.c_str(), reference_topic.c_str(), output_topic.c_str());
  }

private:
  void reference_callback(const sensor_msgs::msg::JointState::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (std::size_t i = 0; i < message->name.size(); ++i)
    {
      if (i < message->position.size())
        reference_position_[message->name[i]] = message->position[i];
      if (i < message->velocity.size())
        reference_velocity_[message->name[i]] = message->velocity[i];
    }
    have_reference_ = !reference_position_.empty();
  }

  void actual_callback(const sensor_msgs::msg::JointState::SharedPtr message)
  {
    sensor_msgs::msg::JointState output = *message;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (have_reference_)
      {
        for (std::size_t i = 0; i < output.name.size(); ++i)
        {
          const auto position = reference_position_.find(output.name[i]);
          if (position != reference_position_.end() && i < output.position.size())
            output.position[i] = position->second;
          const auto velocity = reference_velocity_.find(output.name[i]);
          if (velocity != reference_velocity_.end() && i < output.velocity.size())
            output.velocity[i] = velocity->second;
        }
      }
    }
    output.header.stamp = now();
    output_pub_->publish(output);
  }

  std::mutex mutex_;
  bool have_reference_{};
  std::unordered_map<std::string, double> reference_position_;
  std::unordered_map<std::string, double> reference_velocity_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr actual_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr reference_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr output_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannedJointStateOverlay>());
  rclcpp::shutdown();
  return 0;
}
