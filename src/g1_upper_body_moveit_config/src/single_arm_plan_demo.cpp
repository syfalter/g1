#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "g1_upper_body_moveit_config/srv/plan_single_arm.hpp"

using namespace std::chrono_literals;
using PlanSingleArm = g1_upper_body_moveit_config::srv::PlanSingleArm;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("g1_single_arm_plan_demo");
  auto client = node->create_client<PlanSingleArm>("/plan_single_arm");
  while (rclcpp::ok() && !client->wait_for_service(1s))
    RCLCPP_INFO(node->get_logger(), "Waiting for /plan_single_arm...");
  if (!rclcpp::ok())
    return 1;

  auto request = std::make_shared<PlanSingleArm::Request>();
  request->arm = "LEFT";
  request->tool_profile = "tool0";
  request->mode = "AUTO";
  request->target.header.frame_id = "torso_link";
  request->target.pose.position.x = 0.30;
  request->target.pose.position.y = 0.29;
  request->target.pose.position.z = 0.06;
  request->target.pose.orientation.x = 0.145712255;
  request->target.pose.orientation.y = 0.087035953;
  request->target.pose.orientation.z = 0.118936094;
  request->target.pose.orientation.w = 0.978287732;
  request->target_is_relative = false;
  request->linear_step = 0.02;
  request->jump_threshold = 0.60;
  request->velocity_scaling = 0.35;
  request->acceleration_scaling = 0.30;

  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, future, 15s) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Planning service timed out");
    rclcpp::shutdown();
    return 2;
  }
  const auto response = future.get();
  if (!response->success)
  {
    RCLCPP_ERROR(node->get_logger(), "AUTO planning failed: %s", response->status.c_str());
    rclcpp::shutdown();
    return 3;
  }
  RCLCPP_INFO(node->get_logger(),
              "AUTO selected %s: %zu points, linear fraction %.3f, duration %.3f s, "
              "peak |dq| %.3f rad/s, peak |ddq| %.3f rad/s^2, planning %.3f s",
              response->used_mode.c_str(), response->trajectory.joint_trajectory.points.size(),
              response->linear_fraction, response->duration, response->peak_joint_velocity,
              response->peak_joint_acceleration, response->planning_time);
  rclcpp::shutdown();
  return 0;
}
