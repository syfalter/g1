#include <rclcpp/rclcpp.hpp>
#include <unitree_hg/msg/low_cmd.hpp>
#include <iomanip>
#include <iostream>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("arm_command_source_probe");
  auto subscription = node->create_subscription<unitree_hg::msg::LowCmd>(
    "/arm_sdk", 10,
    [](unitree_hg::msg::LowCmd::ConstSharedPtr command, const rclcpp::MessageInfo &info) {
      std::cout << "{\"gid\":\"" << std::hex << std::setfill('0');
      for (auto value : info.get_rmw_message_info().publisher_gid.data)
        std::cout << std::setw(2) << static_cast<unsigned>(value);
      std::cout << std::dec << "\",\"q\":[" << std::setprecision(12);
      for (size_t i = 0; i < command->motor_cmd.size(); ++i)
        std::cout << (i ? "," : "") << command->motor_cmd[i].q;
      const auto &waist = command->motor_cmd[12];
      std::cout << "],\"waist_kp\":" << waist.kp << ",\"waist_kd\":" << waist.kd
        << ",\"waist_dq\":" << waist.dq << ",\"waist_tau\":" << waist.tau << "}" << std::endl;
    });
  rclcpp::spin(node);
  rclcpp::shutdown();
}
