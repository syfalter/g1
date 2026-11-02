#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <g1_kdl_ik/srv/solve_dual_arm_ik.hpp>

namespace
{
constexpr char kBaseFrame[] = "torso_link";

double durationSeconds(const builtin_interfaces::msg::Duration& duration)
{
  return static_cast<double>(duration.sec) + 1e-9 * static_cast<double>(duration.nanosec);
}

double quaternionNorm(const geometry_msgs::msg::Quaternion& q)
{
  return std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
}
}  // namespace

class G1KdlIkServer : public rclcpp::Node
{
public:
  G1KdlIkServer() : Node("g1_kdl_ik_server")
  {
    default_timeout_ = declare_parameter("default_timeout", 0.05);
    default_motion_duration_ = declare_parameter("default_motion_duration", 2.0);
    max_visual_joint_speed_ = declare_parameter("max_visual_joint_speed", 0.5);
    publish_rate_ = declare_parameter("publish_rate", 50.0);
    publish_joint_states_ = declare_parameter("publish_joint_states", true);
    run_self_test_ = declare_parameter("run_self_test", false);
    if (default_motion_duration_ <= 0.0 || max_visual_joint_speed_ <= 0.0 || publish_rate_ <= 0.0)
      throw std::runtime_error("Motion duration, speed, and publish rate must be positive");
  }

  void initialize()
  {
    robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(
        shared_from_this(), "robot_description");
    robot_model_ = robot_model_loader_->getModel();
    if (!robot_model_)
      throw std::runtime_error("Failed to load robot model");

    for (const char* group_name_c : { "left_arm", "right_arm" })
    {
      const std::string group_name(group_name_c);
      const auto* group = robot_model_->getJointModelGroup(group_name);
      if (!group)
        throw std::runtime_error("Missing SRDF group: " + group_name);
      if (!group->getSolverInstance())
        throw std::runtime_error("KDL solver failed to initialize for " + group_name);
      RCLCPP_INFO(get_logger(), "%s: %u variables, tip=%s", group_name.c_str(),
                  group->getVariableCount(), group->getLinkModelNames().back().c_str());
    }

    service_ = create_service<moveit_msgs::srv::GetPositionIK>(
        "compute_ik", std::bind(&G1KdlIkServer::computeIk, this,
                                std::placeholders::_1, std::placeholders::_2));
    dual_service_ = create_service<g1_kdl_ik::srv::SolveDualArmIK>(
        "solve_dual_arm_ik", std::bind(&G1KdlIkServer::solveDualArmIk, this,
                                       std::placeholders::_1, std::placeholders::_2));

    display_state_ = std::make_unique<moveit::core::RobotState>(robot_model_);
    display_state_->setToDefaultValues();
    display_state_->update();
    trajectory_start_.assign(display_state_->getVariablePositions(),
                             display_state_->getVariablePositions() + robot_model_->getVariableCount());
    trajectory_goal_ = trajectory_start_;
    if (publish_joint_states_)
    {
      joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
      const auto timer_period = std::chrono::duration<double>(1.0 / publish_rate_);
      publish_timer_ = create_wall_timer(
          std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
          std::bind(&G1KdlIkServer::publishDisplayState, this));
    }
    RCLCPP_INFO(get_logger(), "KDL IK ready on /compute_ik (target frame: %s)", kBaseFrame);
    RCLCPP_INFO(get_logger(),
                "Dual-arm visualizer ready on /solve_dual_arm_ik (%.1f Hz, max %.2f rad/s)",
                publish_rate_, max_visual_joint_speed_);
    if (!publish_joint_states_)
      RCLCPP_INFO(get_logger(), "Joint-state publishing disabled; expecting an external GUI/source");

    if (run_self_test_)
      runSelfTest();
  }

private:
  using GetPositionIK = moveit_msgs::srv::GetPositionIK;
  using SolveDualArmIK = g1_kdl_ik::srv::SolveDualArmIK;

  void updateTrajectoryState()
  {
    if (!trajectory_active_)
      return;
    const double elapsed = (now() - trajectory_start_time_).seconds();
    const double u = std::clamp(elapsed / trajectory_duration_, 0.0, 1.0);
    const double scale = u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
    std::vector<double> positions(trajectory_start_.size());
    for (std::size_t i = 0; i < positions.size(); ++i)
      positions[i] = trajectory_start_[i] + scale * (trajectory_goal_[i] - trajectory_start_[i]);
    display_state_->setVariablePositions(positions);
    display_state_->update();
    if (u >= 1.0)
      trajectory_active_ = false;
  }

  void publishDisplayState()
  {
    updateTrajectoryState();
    sensor_msgs::msg::JointState message;
    message.header.stamp = now();
    message.name = robot_model_->getVariableNames();
    message.position.assign(display_state_->getVariablePositions(),
                            display_state_->getVariablePositions() + message.name.size());
    joint_state_publisher_->publish(message);
  }

  void solveDualArmIk(const std::shared_ptr<SolveDualArmIK::Request> request,
                      std::shared_ptr<SolveDualArmIK::Response> response)
  {
    if (!request->solve_left && !request->solve_right)
    {
      response->message = "At least one of solve_left or solve_right must be true";
      return;
    }
    if ((request->solve_left && quaternionNorm(request->left_target.orientation) < 1e-6) ||
        (request->solve_right && quaternionNorm(request->right_target.orientation) < 1e-6))
    {
      response->message = "Each requested target needs a valid quaternion (identity: w=1)";
      return;
    }

    // If a new command arrives during motion, start it from the currently displayed state.
    updateTrajectoryState();
    moveit::core::RobotState goal(*display_state_);
    sensor_msgs::msg::JointState seed;
    seed.name = robot_model_->getVariableNames();
    seed.position.assign(display_state_->getVariablePositions(),
                         display_state_->getVariablePositions() + seed.name.size());

    for (const bool left : { true, false })
    {
      if ((left && !request->solve_left) || (!left && !request->solve_right))
        continue;
      const std::string group_name = left ? "left_arm" : "right_arm";
      const std::string tip_name = left ? "left_tool0" : "right_tool0";
      const auto& target = left ? request->left_target : request->right_target;
      moveit::core::RobotState arm_solution(robot_model_);
      std::string error;
      if (!solve(group_name, tip_name, target, seed, default_timeout_, arm_solution, error))
      {
        response->message = group_name + ": " + error;
        RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
        return;
      }
      const auto* group = robot_model_->getJointModelGroup(group_name);
      std::vector<double> arm_positions;
      arm_solution.copyJointGroupPositions(group, arm_positions);
      goal.setJointGroupPositions(group, arm_positions);
    }
    goal.update();

    trajectory_start_.assign(display_state_->getVariablePositions(),
                             display_state_->getVariablePositions() + robot_model_->getVariableCount());
    trajectory_goal_.assign(goal.getVariablePositions(),
                            goal.getVariablePositions() + robot_model_->getVariableCount());
    double max_delta = 0.0;
    for (std::size_t i = 0; i < trajectory_goal_.size(); ++i)
      max_delta = std::max(max_delta, std::abs(trajectory_goal_[i] - trajectory_start_[i]));

    const double requested_duration = request->duration > 0.0 ?
        request->duration : default_motion_duration_;
    // Quintic time scaling has peak ds/dt = 1.875 / duration.
    const double speed_limited_duration = 1.875 * max_delta / max_visual_joint_speed_;
    trajectory_duration_ = std::max({ requested_duration, speed_limited_duration, 0.02 });
    trajectory_start_time_ = now();
    trajectory_active_ = true;

    response->success = true;
    response->actual_duration = trajectory_duration_;
    response->message = "IK solved; smooth visualization trajectory started";
    response->solution.header.stamp = now();
    response->solution.name = robot_model_->getVariableNames();
    response->solution.position = trajectory_goal_;
    RCLCPP_INFO(get_logger(), "Dual-arm goal accepted; duration %.3f s", trajectory_duration_);
  }

  bool solve(const std::string& group_name, const std::string& tip_name,
             const geometry_msgs::msg::Pose& target,
             const sensor_msgs::msg::JointState& seed, double timeout,
             moveit::core::RobotState& state, std::string& error)
  {
    const auto* group = robot_model_->getJointModelGroup(group_name);
    if (!group)
    {
      error = "group must be left_arm or right_arm";
      return false;
    }

    state.setToDefaultValues();
    if (seed.name.size() != seed.position.size())
    {
      error = "seed joint name and position lengths differ";
      return false;
    }
    const std::unordered_set<std::string> model_variables(
        robot_model_->getVariableNames().begin(), robot_model_->getVariableNames().end());
    for (std::size_t i = 0; i < seed.name.size(); ++i)
    {
      if (model_variables.count(seed.name[i]) != 0)
        state.setVariablePosition(seed.name[i], seed.position[i]);
    }
    state.enforceBounds(group);
    state.update();

    const std::string expected_tip = group_name == "left_arm" ? "left_tool0" : "right_tool0";
    const std::string selected_tip = tip_name.empty() ? expected_tip : tip_name;
    if (selected_tip != expected_tip)
    {
      error = "ik_link_name must be " + expected_tip;
      return false;
    }

    const bool found = state.setFromIK(group, target, selected_tip, timeout);
    if (!found)
    {
      error = "KDL found no solution within limits and timeout";
      return false;
    }
    state.enforceBounds(group);
    state.update();
    return true;
  }

  void computeIk(const std::shared_ptr<GetPositionIK::Request> request,
                 std::shared_ptr<GetPositionIK::Response> response)
  {
    const auto& ik = request->ik_request;
    if (ik.avoid_collisions)
    {
      response->error_code.val = moveit_msgs::msg::MoveItErrorCodes::COLLISION_CHECKING_UNAVAILABLE;
      RCLCPP_WARN(get_logger(), "Rejected IK request: collision checking is not part of KDL v1");
      return;
    }
    if (!ik.pose_stamped.header.frame_id.empty() &&
        ik.pose_stamped.header.frame_id != kBaseFrame)
    {
      response->error_code.val = moveit_msgs::msg::MoveItErrorCodes::FRAME_TRANSFORM_FAILURE;
      RCLCPP_WARN(get_logger(), "Target frame must be %s", kBaseFrame);
      return;
    }
    if (quaternionNorm(ik.pose_stamped.pose.orientation) < 1e-6)
    {
      response->error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
      RCLCPP_WARN(get_logger(), "Target quaternion is invalid; use w=1 for identity");
      return;
    }

    double timeout = durationSeconds(ik.timeout);
    if (timeout <= 0.0)
      timeout = default_timeout_;

    moveit::core::RobotState solution(robot_model_);
    std::string error;
    if (!solve(ik.group_name, ik.ik_link_name, ik.pose_stamped.pose,
               ik.robot_state.joint_state, timeout, solution, error))
    {
      response->error_code.val = robot_model_->getJointModelGroup(ik.group_name) ?
          moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION :
          moveit_msgs::msg::MoveItErrorCodes::INVALID_GROUP_NAME;
      RCLCPP_WARN(get_logger(), "%s", error.c_str());
      return;
    }

    const auto* group = robot_model_->getJointModelGroup(ik.group_name);
    response->solution.joint_state.header.stamp = now();
    response->solution.joint_state.name = group->getVariableNames();
    solution.copyJointGroupPositions(group, response->solution.joint_state.position);
    response->error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  }

  void runSelfTest()
  {
    bool all_ok = true;
    for (const char* group_name_c : { "left_arm", "right_arm" })
    {
      const std::string group_name(group_name_c);
      const auto* group = robot_model_->getJointModelGroup(group_name);
      const std::string tip = group_name == "left_arm" ? "left_tool0" : "right_tool0";
      moveit::core::RobotState reference(robot_model_);
      reference.setToDefaultValues();
      std::vector<double> q{ -0.35, group_name == "left_arm" ? 0.25 : -0.25,
                             0.15, 0.75, 0.10, -0.20, 0.12 };
      reference.setJointGroupPositions(group, q);
      reference.update();
      const Eigen::Isometry3d& target_transform = reference.getGlobalLinkTransform(tip);
      geometry_msgs::msg::Pose target;
      target.position.x = target_transform.translation().x();
      target.position.y = target_transform.translation().y();
      target.position.z = target_transform.translation().z();
      const Eigen::Quaterniond target_quaternion(target_transform.rotation());
      target.orientation.x = target_quaternion.x();
      target.orientation.y = target_quaternion.y();
      target.orientation.z = target_quaternion.z();
      target.orientation.w = target_quaternion.w();

      sensor_msgs::msg::JointState seed;
      seed.name = group->getVariableNames();
      seed.position = q;
      for (double& value : seed.position)
        value += 0.08;

      moveit::core::RobotState result(robot_model_);
      std::string error;
      const auto start = std::chrono::steady_clock::now();
      const bool found = solve(group_name, tip, target, seed, 0.1, result, error);
      const double elapsed_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - start).count();
      if (!found)
      {
        all_ok = false;
        RCLCPP_ERROR(get_logger(), "SELF-TEST %s FAILED: %s", group_name.c_str(), error.c_str());
        continue;
      }

      const Eigen::Isometry3d& achieved = result.getGlobalLinkTransform(tip);
      const Eigen::Isometry3d& desired = reference.getGlobalLinkTransform(tip);
      const double position_error = (achieved.translation() - desired.translation()).norm();
      const Eigen::Matrix3d rotation_delta = desired.rotation().transpose() * achieved.rotation();
      const double rotation_error = Eigen::AngleAxisd(rotation_delta).angle();
      const bool accurate = position_error < 1e-4 && rotation_error < 1e-4;
      all_ok = all_ok && accurate;
      RCLCPP_INFO(get_logger(),
                  "SELF-TEST %s: %s, %.3f ms, position error %.3g m, rotation error %.3g rad",
                  group_name.c_str(), accurate ? "PASS" : "FAIL", elapsed_ms,
                  position_error, rotation_error);
      RCLCPP_INFO(get_logger(),
                  "SELF-TEST %s target: p=[%.9f, %.9f, %.9f], q=[%.9f, %.9f, %.9f, %.9f]",
                  group_name.c_str(), target.position.x, target.position.y, target.position.z,
                  target.orientation.x, target.orientation.y, target.orientation.z,
                  target.orientation.w);
    }
    if (!all_ok)
      throw std::runtime_error("KDL self-test failed");
  }

  double default_timeout_{ 0.05 };
  double default_motion_duration_{ 2.0 };
  double max_visual_joint_speed_{ 0.5 };
  double publish_rate_{ 50.0 };
  bool run_self_test_{ false };
  bool publish_joint_states_{ true };
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  moveit::core::RobotModelPtr robot_model_;
  rclcpp::Service<GetPositionIK>::SharedPtr service_;
  rclcpp::Service<SolveDualArmIK>::SharedPtr dual_service_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  std::unique_ptr<moveit::core::RobotState> display_state_;
  std::vector<double> trajectory_start_;
  std::vector<double> trajectory_goal_;
  rclcpp::Time trajectory_start_time_;
  double trajectory_duration_{ 0.0 };
  bool trajectory_active_{ false };
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try
  {
    auto node = std::make_shared<G1KdlIkServer>();
    node->initialize();
    rclcpp::spin(node);
  }
  catch (const std::exception& exception)
  {
    RCLCPP_FATAL(rclcpp::get_logger("g1_kdl_ik"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
