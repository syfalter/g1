#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/ruckig_traj_smoothing.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <visualization_msgs/msg/interactive_marker_update.hpp>

#include "g1_upper_body_moveit_config/srv/plan_single_arm.hpp"
#include "g1_upper_body_moveit_config/srv/time_task_path.hpp"

using PlanSingleArm = g1_upper_body_moveit_config::srv::PlanSingleArm;
using TimeTaskPath = g1_upper_body_moveit_config::srv::TimeTaskPath;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

class SingleArmPlannerServer
{
public:
  explicit SingleArmPlannerServer(const rclcpp::Node::SharedPtr& node)
  : node_(node),
    left_arm_(node_, "left_arm"),
    right_arm_(node_, "right_arm"),
    tf_buffer_(std::make_unique<tf2_ros::Buffer>(node_->get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, true))
  {
    hardware_arm_ = node_->get_parameter("hardware_arm").as_string();
    if (hardware_arm_ != "left" && hardware_arm_ != "right")
      throw std::invalid_argument("hardware_arm must be left or right");
    display_pub_ = node_->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
        "/display_planned_path", rclcpp::QoS(1).transient_local());
    publish_shadow_trajectory_ = node_->get_parameter("publish_shadow_trajectory").as_bool();
    shadow_trajectory_topic_ = node_->get_parameter("shadow_trajectory_topic").as_string();
    max_relative_translation_m_ = std::clamp(
        node_->get_parameter("max_relative_translation_m").as_double(), 0.01, 0.50);
    max_relative_rotation_rad_ = std::clamp(
        node_->get_parameter("max_relative_rotation_rad").as_double(), 0.05, M_PI);
    max_linear_rotation_step_rad_ = std::clamp(
        node_->get_parameter("max_linear_rotation_step_rad").as_double(), 0.01, 0.50);
    max_execution_velocity_scaling_ = std::clamp(
        node_->get_parameter("max_execution_velocity_scaling").as_double(), 0.01, 1.0);
    max_execution_acceleration_scaling_ = std::clamp(
        node_->get_parameter("max_execution_acceleration_scaling").as_double(), 0.01, 1.0);
    execution_result_margin_s_ = std::clamp(
        node_->get_parameter("execution_result_margin_s").as_double(), 2.0, 60.0);
    rviz_auto_velocity_scaling_ = std::clamp(
        node_->get_parameter("rviz_auto_velocity_scaling").as_double(), 0.01, 1.0);
    rviz_auto_acceleration_scaling_ = std::clamp(
        node_->get_parameter("rviz_auto_acceleration_scaling").as_double(), 0.01, 1.0);
    load_tool_profiles();
    if (publish_shadow_trajectory_)
      shadow_trajectory_pub_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
          shadow_trajectory_topic_, rclcpp::QoS(1).reliable().durability_volatile());
    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    follow_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
        node_, "/" + hardware_arm_ + "_arm_controller/follow_joint_trajectory", callback_group_);
    service_ = node_->create_service<PlanSingleArm>(
        "/plan_single_arm",
        std::bind(&SingleArmPlannerServer::plan, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_);
    move_service_ = node_->create_service<PlanSingleArm>(
        "/move_single_arm",
        std::bind(&SingleArmPlannerServer::plan, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_);
    time_task_service_ = node_->create_service<TimeTaskPath>(
        "/g1/task/time_path", std::bind(&SingleArmPlannerServer::time_task_path, this,
        std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_);
    rviz_marker_sub_ = node_->create_subscription<
        visualization_msgs::msg::InteractiveMarkerUpdate>(
        "/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update",
        rclcpp::QoS(10).reliable().durability_volatile(),
        std::bind(&SingleArmPlannerServer::rviz_marker_callback, this, std::placeholders::_1));
    const auto scaling_qos = rclcpp::QoS(1).reliable().transient_local();
    rviz_auto_velocity_scaling_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
        "/g1/rviz_auto/velocity_scaling", scaling_qos,
        [this](const std_msgs::msg::Float64::SharedPtr message) {
          rviz_auto_scaling_callback(message->data, true);
        });
    rviz_auto_acceleration_scaling_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
        "/g1/rviz_auto/acceleration_scaling", scaling_qos,
        [this](const std_msgs::msg::Float64::SharedPtr message) {
          rviz_auto_scaling_callback(message->data, false);
        });
    rviz_auto_plan_service_ = node_->create_service<std_srvs::srv::Trigger>(
        "/g1/rviz_auto_plan",
        std::bind(&SingleArmPlannerServer::rviz_auto_plan, this,
          std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_);
    rviz_auto_execute_service_ = node_->create_service<std_srvs::srv::Trigger>(
        "/g1/rviz_auto_execute",
        std::bind(&SingleArmPlannerServer::rviz_auto_execute, this,
          std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_);

    add_demo_scene();
    RCLCPP_INFO(node_->get_logger(),
                "Ready on /plan_single_arm and /move_single_arm with %zu tool profiles "
                "(modes: AUTO, LINEAR, OMPL; guarded selected-arm execution available)",
                tool_profiles_.size());
    RCLCPP_INFO(
        node_->get_logger(),
        "RViz AUTO controls ready on /g1/rviz_auto_plan and /g1/rviz_auto_execute "
        "at scaling %.2f/%.2f",
        rviz_auto_velocity_scaling_, rviz_auto_acceleration_scaling_);
    if (publish_shadow_trajectory_)
      RCLCPP_WARN(node_->get_logger(), "Successful plans will also be copied to SHADOW topic %s",
                  shadow_trajectory_topic_.c_str());
  }

private:
  void time_task_path(const std::shared_ptr<TimeTaskPath::Request> request,
                      std::shared_ptr<TimeTaskPath::Response> response)
  {
    std::lock_guard<std::mutex> lock(planning_mutex_);
    try
    {
      const bool waist = request->arm == "left_waist" || request->arm == "right_waist";
      if ((request->arm != "left" && request->arm != "right" && !waist) ||
          !std::isfinite(request->velocity_scaling) ||
          !std::isfinite(request->acceleration_scaling) ||
          request->velocity_scaling < .05 || request->velocity_scaling > 1 ||
          request->acceleration_scaling < .05 || request->acceleration_scaling > 1)
        throw std::runtime_error("invalid side/scaling");
      const bool left = request->arm == "left" || request->arm == "left_waist";
      auto& arm = left ? left_arm_ : right_arm_;
      auto model = arm.getRobotModel();
      const auto group_name = arm.getName() + (waist ? "_with_waist" : "");
      const auto* group = model->getJointModelGroup(group_name);
      if (!group) throw std::runtime_error("requested task planning group is not loaded");
      auto names = group->getVariableNames();
      auto supplied = request->trajectory.joint_trajectory.joint_names;
      std::sort(names.begin(), names.end());
      std::sort(supplied.begin(), supplied.end());
      const std::size_t count = waist ? 8 : 7;
      if (names != supplied || names.size() != count ||
          request->trajectory.joint_trajectory.points.size() < 2 ||
          request->trajectory.joint_trajectory.points.size() > 20000)
        throw std::runtime_error("path must exactly match the selected arm/task-waist group; fingers forbidden");
      for (const auto& point : request->trajectory.joint_trajectory.points)
        if (point.positions.size() != count || std::any_of(point.positions.begin(),
              point.positions.end(), [](double x) { return !std::isfinite(x); }))
          throw std::runtime_error("nonfinite/invalid path point");
      moveit::core::RobotState state(model);
      state.setToDefaultValues();
      moveit::core::robotStateMsgToRobotState(request->start_state, state);
      robot_trajectory::RobotTrajectory path(model, group_name);
      path.setRobotTrajectoryMsg(state, request->trajectory);
      std::unordered_map<std::string, double> velocity, acceleration, jerk;
      for (const auto& name : names)
      {
        velocity[name] = (name == "waist_yaw_joint" ? .4 : .60) * request->velocity_scaling;
        acceleration[name] = 1.50 * request->acceleration_scaling;
        jerk[name] = 5.0;
      }
      constexpr double resample_dt = .1;
      trajectory_processing::TimeOptimalTrajectoryGeneration timing(.001, resample_dt, .00001);
      if (!timing.computeTimeStamps(path, velocity, acceleration))
        throw std::runtime_error("MoveIt TOTG failed");
      if (path.getWayPointCount() < 2)
        throw std::runtime_error("Task timing returned fewer than two waypoints");
      // Avoid a sub-cycle tail forcing the smoother to stretch the whole path.
      // Keep both endpoints; the caller still validates the final curve at 5 ms.
      const auto last = path.getWayPointCount() - 1;
      if (last > 1 && path.getWayPointDurationFromPrevious(last) < resample_dt * .5)
      {
        robot_trajectory::RobotTrajectory merged(model, group_name);
        for (std::size_t i = 0; i < last - 1; ++i)
          merged.addSuffixWayPoint(path.getWayPoint(i), path.getWayPointDurationFromPrevious(i));
        merged.addSuffixWayPoint(path.getWayPoint(last),
          path.getWayPointDurationFromPrevious(last - 1) + path.getWayPointDurationFromPrevious(last));
        path.swap(merged);
      }
      // Match the gateway's stationary hold before smoothing, not afterwards.
      for (const auto index : {std::size_t{0}, path.getWayPointCount() - 1})
        for (const auto& name : names)
        {
          path.getWayPointPtr(index)->setVariableVelocity(name, 0.0);
          path.getWayPointPtr(index)->setVariableAcceleration(name, 0.0);
        }
      if (!trajectory_processing::RuckigSmoothing::applySmoothing(path, velocity, acceleration, jerk))
        throw std::runtime_error("MoveIt task Ruckig smoothing failed");
      path.getRobotTrajectoryMsg(response->trajectory);
      response->success = true;
      response->message = "TOTG + Ruckig; caller must recheck final Hermite bounds, geometry and collisions before execution";
    }
    catch (const std::exception& e) { response->success = false; response->message = e.what(); }
  }

  struct ToolProfile
  {
    std::string name;
    std::string arm;
    bool enabled{};
    bool calibrated{};
    tf2::Transform tool0_to_tcp;
  };

  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::MoveGroupInterface left_arm_;
  moveit::planning_interface::MoveGroupInterface right_arm_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Service<PlanSingleArm>::SharedPtr service_;
  rclcpp::Service<PlanSingleArm>::SharedPtr move_service_;
  rclcpp::Service<TimeTaskPath>::SharedPtr time_task_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr rviz_auto_plan_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr rviz_auto_execute_service_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr follow_client_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr shadow_trajectory_pub_;
  rclcpp::Subscription<visualization_msgs::msg::InteractiveMarkerUpdate>::SharedPtr
    rviz_marker_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rviz_auto_velocity_scaling_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rviz_auto_acceleration_scaling_sub_;
  bool publish_shadow_trajectory_{};
  std::string shadow_trajectory_topic_;
  std::string hardware_arm_;
  double max_relative_translation_m_{};
  double max_relative_rotation_rad_{};
  double max_linear_rotation_step_rad_{};
  double max_execution_velocity_scaling_{};
  double max_execution_acceleration_scaling_{};
  double execution_result_margin_s_{};
  double rviz_auto_velocity_scaling_{};
  double rviz_auto_acceleration_scaling_{};
  std::map<std::string, ToolProfile> tool_profiles_;
  std::mutex planning_mutex_;
  std::mutex rviz_auto_mutex_;
  geometry_msgs::msg::PoseStamped rviz_marker_target_;
  moveit_msgs::msg::RobotTrajectory rviz_auto_trajectory_;
  double rviz_auto_duration_{};
  std::uint64_t rviz_marker_revision_{};
  std::uint64_t rviz_auto_marker_revision_{};
  std::uint64_t rviz_auto_scaling_revision_{};
  std::uint64_t rviz_auto_plan_scaling_revision_{};
  bool have_rviz_marker_target_{};
  bool have_rviz_auto_plan_{};

  static std::string upper(std::string value)
  {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    return value;
  }

  void add_builtin_tool0_profile()
  {
    ToolProfile profile;
    profile.name = "tool0";
    profile.arm = "BOTH";
    profile.enabled = true;
    profile.calibrated = true;
    profile.tool0_to_tcp.setIdentity();
    tool_profiles_.emplace(profile.name, profile);
  }

  void load_tool_profiles()
  {
    if (!node_->has_parameter("tool_profiles.names"))
    {
      add_builtin_tool0_profile();
      RCLCPP_WARN(node_->get_logger(),
                  "No tool profile config supplied; only the identity 'tool0' profile is available");
      return;
    }

    const auto names = node_->get_parameter("tool_profiles.names").as_string_array();
    if (names.empty())
      throw std::runtime_error("tool_profiles.names must not be empty");

    for (const auto& name : names)
    {
      if (name.empty())
        throw std::runtime_error("tool profile name must not be empty");
      if (tool_profiles_.count(name) != 0)
        throw std::runtime_error("duplicate tool profile name: " + name);

      const std::string prefix = "tool_profiles." + name + ".";
      const std::vector<std::string> required{
        prefix + "arm", prefix + "enabled", prefix + "calibrated",
        prefix + "translation", prefix + "quaternion_xyzw"};
      for (const auto& parameter : required)
        if (!node_->has_parameter(parameter))
          throw std::runtime_error("missing tool profile parameter: " + parameter);

      ToolProfile profile;
      profile.name = name;
      profile.arm = upper(node_->get_parameter(prefix + "arm").as_string());
      profile.enabled = node_->get_parameter(prefix + "enabled").as_bool();
      profile.calibrated = node_->get_parameter(prefix + "calibrated").as_bool();
      const auto translation = node_->get_parameter(prefix + "translation").as_double_array();
      const auto quaternion = node_->get_parameter(prefix + "quaternion_xyzw").as_double_array();
      if (profile.arm != "LEFT" && profile.arm != "RIGHT" && profile.arm != "BOTH")
        throw std::runtime_error("tool profile '" + name + "' arm must be LEFT, RIGHT, or BOTH");
      if (translation.size() != 3 || quaternion.size() != 4)
        throw std::runtime_error(
                "tool profile '" + name + "' requires translation[3] and quaternion_xyzw[4]");
      for (const double value : translation)
        if (!std::isfinite(value))
          throw std::runtime_error("tool profile '" + name + "' translation is non-finite");
      for (const double value : quaternion)
        if (!std::isfinite(value))
          throw std::runtime_error("tool profile '" + name + "' quaternion is non-finite");

      tf2::Quaternion rotation(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
      if (rotation.length() < 1e-9)
        throw std::runtime_error("tool profile '" + name + "' quaternion is zero");
      rotation.normalize();
      profile.tool0_to_tcp.setOrigin(
          tf2::Vector3(translation[0], translation[1], translation[2]));
      profile.tool0_to_tcp.setRotation(rotation);
      tool_profiles_.emplace(name, profile);
      RCLCPP_INFO(node_->get_logger(),
                  "Tool profile '%s': arm=%s enabled=%s calibrated=%s "
                  "tool0->TCP xyz=[%.4f %.4f %.4f]",
                  name.c_str(), profile.arm.c_str(), profile.enabled ? "true" : "false",
                  profile.calibrated ? "true" : "false", translation[0], translation[1],
                  translation[2]);
    }

    if (tool_profiles_.count("tool0") == 0)
      throw std::runtime_error("tool profile config must contain backward-compatible 'tool0'");
  }

  const ToolProfile* select_tool_profile(const std::string& requested_name,
                                         const std::string& arm, std::string& error) const
  {
    const std::string name = requested_name.empty() ? "tool0" : requested_name;
    const auto iterator = tool_profiles_.find(name);
    if (iterator == tool_profiles_.end())
    {
      error = "unknown tool_profile '" + name + "'";
      return nullptr;
    }
    const auto& profile = iterator->second;
    if (profile.arm != "BOTH" && profile.arm != arm)
    {
      error = "tool_profile '" + name + "' is for " + profile.arm + ", not " + arm;
      return nullptr;
    }
    if (!profile.enabled)
    {
      error = "tool_profile '" + name + "' is disabled; measure and enable its transform first";
      return nullptr;
    }
    if (!profile.calibrated)
    {
      error = "tool_profile '" + name + "' is not calibrated";
      return nullptr;
    }
    return &profile;
  }

  void add_box(moveit::planning_interface::PlanningSceneInterface& scene,
               const std::string& id, const std::string& frame, double x, double y, double z,
               double sx, double sy, double sz)
  {
    moveit_msgs::msg::CollisionObject object;
    object.header.frame_id = frame;
    object.id = id;
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions = { sx, sy, sz };
    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    object.primitives.push_back(primitive);
    object.primitive_poses.push_back(pose);
    object.operation = moveit_msgs::msg::CollisionObject::ADD;
    if (!scene.applyCollisionObject(object))
      RCLCPP_ERROR(node_->get_logger(), "Failed to apply demo collision object '%s'", id.c_str());
  }

  void add_demo_scene()
  {
    const bool table = node_->get_parameter("add_demo_table").as_bool();
    const bool obstacle = node_->get_parameter("add_demo_obstacle").as_bool();
    if (!table && !obstacle)
      return;
    // The interface waits for scene services; tasks without demo objects do not need it.
    moveit::planning_interface::PlanningSceneInterface scene;
    if (table)
      add_box(scene, "demo_table", "world", 0.25, 0.0, 0.52, 1.2, 1.2, 0.08);
    if (obstacle)
      add_box(scene, "linear_path_blocker", "torso_link", 0.2725, 0.221, 0.055, 0.050, 0.025, 0.120);
  }

  static void trajectory_metrics(const moveit_msgs::msg::RobotTrajectory& trajectory,
                                 PlanSingleArm::Response& response)
  {
    const auto& points = trajectory.joint_trajectory.points;
    response.duration = points.empty() ? 0.0 : rclcpp::Duration(points.back().time_from_start).seconds();
    for (const auto& point : points)
    {
      for (const double value : point.velocities)
        response.peak_joint_velocity = std::max(response.peak_joint_velocity, std::abs(value));
      for (const double value : point.accelerations)
        response.peak_joint_acceleration = std::max(response.peak_joint_acceleration, std::abs(value));
    }
  }

  static bool has_joint_jump(const moveit_msgs::msg::RobotTrajectory& trajectory, double limit)
  {
    const auto& points = trajectory.joint_trajectory.points;
    for (std::size_t i = 1; i < points.size(); ++i)
      for (std::size_t j = 0; j < points[i].positions.size(); ++j)
        if (std::abs(points[i].positions[j] - points[i - 1].positions[j]) > limit)
          return true;
    return false;
  }

  static bool finite_pose(const geometry_msgs::msg::Pose& pose)
  {
    return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
      std::isfinite(pose.position.z) && std::isfinite(pose.orientation.x) &&
      std::isfinite(pose.orientation.y) && std::isfinite(pose.orientation.z) &&
      std::isfinite(pose.orientation.w);
  }

  static bool same_pose(
      const geometry_msgs::msg::Pose& left, const geometry_msgs::msg::Pose& right)
  {
    constexpr double translation_tolerance_m = 5e-4;
    constexpr double rotation_tolerance_rad = 1e-3;
    const double dx = left.position.x - right.position.x;
    const double dy = left.position.y - right.position.y;
    const double dz = left.position.z - right.position.z;
    if (std::hypot(std::hypot(dx, dy), dz) > translation_tolerance_m)
      return false;

    const double left_norm = std::hypot(
      std::hypot(left.orientation.x, left.orientation.y),
      std::hypot(left.orientation.z, left.orientation.w));
    const double right_norm = std::hypot(
      std::hypot(right.orientation.x, right.orientation.y),
      std::hypot(right.orientation.z, right.orientation.w));
    if (left_norm < 1e-12 || right_norm < 1e-12)
      return false;
    const double dot = std::abs(
      (left.orientation.x * right.orientation.x +
       left.orientation.y * right.orientation.y +
       left.orientation.z * right.orientation.z +
       left.orientation.w * right.orientation.w) /
      (left_norm * right_norm));
    const double rotation_error = 2.0 * std::acos(std::clamp(dot, 0.0, 1.0));
    return rotation_error <= rotation_tolerance_rad;
  }

  void rviz_marker_callback(
      const visualization_msgs::msg::InteractiveMarkerUpdate::SharedPtr message)
  {
    for (const auto& marker_pose : message->poses)
    {
      if (marker_pose.name != "EE:goal_" + hardware_arm_ + "_tool0" || !finite_pose(marker_pose.pose))
        continue;
      std::lock_guard<std::mutex> lock(rviz_auto_mutex_);
      if (!have_rviz_marker_target_ ||
          rviz_marker_target_.header.frame_id != marker_pose.header.frame_id ||
          !same_pose(rviz_marker_target_.pose, marker_pose.pose))
      {
        ++rviz_marker_revision_;
      }
      rviz_marker_target_.header = marker_pose.header;
      rviz_marker_target_.pose = marker_pose.pose;
      have_rviz_marker_target_ = true;
      return;
    }
  }

  void rviz_auto_scaling_callback(double value, bool velocity)
  {
    if (!std::isfinite(value))
    {
      RCLCPP_WARN(node_->get_logger(), "Ignoring non-finite RViz AUTO scaling value");
      return;
    }
    const double clamped = std::clamp(value, 0.10, 1.00);
    std::lock_guard<std::mutex> lock(rviz_auto_mutex_);
    double& current = velocity ? rviz_auto_velocity_scaling_ : rviz_auto_acceleration_scaling_;
    if (std::abs(current - clamped) <= 1e-9)
      return;
    current = clamped;
    ++rviz_auto_scaling_revision_;
    have_rviz_auto_plan_ = false;
    RCLCPP_INFO(
        node_->get_logger(), "RViz AUTO scaling updated to velocity=%.2f acceleration=%.2f; "
        "saved AUTO plan invalidated",
        rviz_auto_velocity_scaling_, rviz_auto_acceleration_scaling_);
  }

  void rviz_auto_plan(
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> trigger_response)
  {
    geometry_msgs::msg::PoseStamped marker_target;
    std::uint64_t marker_revision = 0;
    std::uint64_t scaling_revision = 0;
    double velocity_scaling = 0.0;
    double acceleration_scaling = 0.0;
    {
      std::lock_guard<std::mutex> lock(rviz_auto_mutex_);
      if (!have_rviz_marker_target_)
      {
        trigger_response->success = false;
        trigger_response->message = hardware_arm_ + "-arm RViz goal marker is not available";
        return;
      }
      have_rviz_auto_plan_ = false;
      marker_target = rviz_marker_target_;
      marker_revision = rviz_marker_revision_;
      scaling_revision = rviz_auto_scaling_revision_;
      velocity_scaling = rviz_auto_velocity_scaling_;
      acceleration_scaling = rviz_auto_acceleration_scaling_;
    }

    auto request = std::make_shared<PlanSingleArm::Request>();
    auto response = std::make_shared<PlanSingleArm::Response>();
    request->arm = upper(hardware_arm_);
    request->tool_profile = "tool0";
    request->target = marker_target;
    request->target_is_relative = false;
    request->mode = "AUTO";
    request->linear_step = 0.02;
    request->jump_threshold = 0.60;
    request->velocity_scaling = velocity_scaling;
    request->acceleration_scaling = acceleration_scaling;
    request->execute = false;
    plan(request, response);
    if (!response->success)
    {
      trigger_response->success = false;
      trigger_response->message = "AUTO plan failed: " + response->status;
      return;
    }
    const auto point_count = response->trajectory.joint_trajectory.points.size();
    if (point_count < 2 || response->duration <= 1e-6)
    {
      trigger_response->success = false;
      trigger_response->message =
        "AUTO plan has no motion; move the selected-arm goal marker and plan again";
      return;
    }

    {
      std::lock_guard<std::mutex> lock(rviz_auto_mutex_);
      if (rviz_marker_revision_ != marker_revision)
      {
        trigger_response->success = false;
        trigger_response->message = "RViz goal moved during planning; plan again";
        return;
      }
      if (rviz_auto_scaling_revision_ != scaling_revision)
      {
        trigger_response->success = false;
        trigger_response->message = "AUTO scaling changed during planning; plan again";
        return;
      }
      rviz_auto_trajectory_ = response->trajectory;
      rviz_auto_duration_ = response->duration;
      rviz_auto_marker_revision_ = marker_revision;
      rviz_auto_plan_scaling_revision_ = scaling_revision;
      have_rviz_auto_plan_ = true;
    }
    RCLCPP_INFO(
        node_->get_logger(),
        "RViz AUTO plan saved: marker_revision=%lu scaling_revision=%lu points=%zu duration=%.3f",
        static_cast<unsigned long>(marker_revision),
        static_cast<unsigned long>(scaling_revision), point_count, response->duration);
    trigger_response->success = true;
    trigger_response->message =
      "AUTO ready: " + response->used_mode + ", " +
      std::to_string(point_count) + " points, " + std::to_string(response->duration) +
      " s, scaling " + std::to_string(velocity_scaling) + "/" +
      std::to_string(acceleration_scaling);
  }

  void rviz_auto_execute(
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> trigger_response)
  {
    RCLCPP_INFO(node_->get_logger(), "RViz AUTO Execute requested");
    moveit_msgs::msg::RobotTrajectory trajectory;
    double duration = 0.0;
    {
      std::lock_guard<std::mutex> lock(rviz_auto_mutex_);
      if (!have_rviz_auto_plan_)
      {
        RCLCPP_WARN(node_->get_logger(), "RViz AUTO Execute rejected: no saved plan");
        trigger_response->success = false;
        trigger_response->message = "no saved RViz AUTO plan; click AUTO Plan first";
        return;
      }
      if (rviz_marker_revision_ != rviz_auto_marker_revision_)
      {
        RCLCPP_WARN(
            node_->get_logger(),
            "RViz AUTO Execute rejected: marker revision changed from %lu to %lu",
            static_cast<unsigned long>(rviz_auto_marker_revision_),
            static_cast<unsigned long>(rviz_marker_revision_));
        have_rviz_auto_plan_ = false;
        trigger_response->success = false;
        trigger_response->message = "RViz goal changed after AUTO Plan; plan again";
        return;
      }
      if (rviz_auto_scaling_revision_ != rviz_auto_plan_scaling_revision_)
      {
        RCLCPP_WARN(
            node_->get_logger(),
            "RViz AUTO Execute rejected: scaling revision changed from %lu to %lu",
            static_cast<unsigned long>(rviz_auto_plan_scaling_revision_),
            static_cast<unsigned long>(rviz_auto_scaling_revision_));
        have_rviz_auto_plan_ = false;
        trigger_response->success = false;
        trigger_response->message = "AUTO scaling changed after AUTO Plan; plan again";
        return;
      }
      trajectory = rviz_auto_trajectory_;
      duration = rviz_auto_duration_;
    }

    RCLCPP_WARN(
        node_->get_logger(), "Submitting saved RViz AUTO trajectory: %zu points, %.3f s",
        trajectory.joint_trajectory.points.size(), duration);

    std::lock_guard<std::mutex> planning_lock(planning_mutex_);
    PlanSingleArm::Response execution_response;
    execute_arm_trajectory(trajectory, duration, execution_response);
    trigger_response->success = execution_response.execution_accepted &&
      execution_response.execution_success;
    trigger_response->message = execution_response.execution_status;
    if (execution_response.execution_accepted)
    {
      std::lock_guard<std::mutex> lock(rviz_auto_mutex_);
      have_rviz_auto_plan_ = false;
    }
  }

  std::vector<geometry_msgs::msg::Pose> tcp_waypoints_as_tool0(
      const tf2::Transform& tcp_start, const geometry_msgs::msg::PoseStamped& tcp_target,
      const ToolProfile& profile, double translation_step) const
  {
    tf2::Transform tcp_goal;
    tf2::fromMsg(tcp_target.pose, tcp_goal);
    const double translation_distance =
      (tcp_goal.getOrigin() - tcp_start.getOrigin()).length();
    tf2::Quaternion start_rotation = tcp_start.getRotation();
    tf2::Quaternion goal_rotation = tcp_goal.getRotation();
    start_rotation.normalize();
    goal_rotation.normalize();
    double dot = start_rotation.dot(goal_rotation);
    if (dot < 0.0)
    {
      goal_rotation = tf2::Quaternion(
        -goal_rotation.x(), -goal_rotation.y(), -goal_rotation.z(), -goal_rotation.w());
      dot = -dot;
    }
    const double rotation_distance =
      2.0 * std::acos(std::clamp(dot, 0.0, 1.0));
    const auto translation_segments = static_cast<std::size_t>(
      std::ceil(translation_distance / translation_step));
    const auto rotation_segments = static_cast<std::size_t>(
      std::ceil(rotation_distance / max_linear_rotation_step_rad_));
    const std::size_t segment_count =
      std::max<std::size_t>(1, std::max(translation_segments, rotation_segments));

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.reserve(segment_count);
    for (std::size_t index = 1; index <= segment_count; ++index)
    {
      const double ratio = static_cast<double>(index) / static_cast<double>(segment_count);
      tf2::Transform tcp_waypoint;
      tcp_waypoint.setOrigin(tcp_start.getOrigin().lerp(tcp_goal.getOrigin(), ratio));
      auto waypoint_rotation = start_rotation.slerp(goal_rotation, ratio);
      waypoint_rotation.normalize();
      tcp_waypoint.setRotation(waypoint_rotation);
      const tf2::Transform tool0_waypoint =
        tcp_waypoint * profile.tool0_to_tcp.inverse();
      geometry_msgs::msg::Pose pose;
      tf2::toMsg(tool0_waypoint, pose);
      waypoints.push_back(pose);
    }
    return waypoints;
  }

  bool resolve_target(const PlanSingleArm::Request& request, const std::string& tool0,
                      const ToolProfile& profile, const std::string& planning_frame,
                      geometry_msgs::msg::PoseStamped& tcp_target,
                      geometry_msgs::msg::PoseStamped& tool0_target,
                      tf2::Transform& planning_tcp_start, std::string& error)
  {
    auto requested_target = request.target;
    if (requested_target.header.frame_id.empty())
      requested_target.header.frame_id = "torso_link";
    if (!finite_pose(requested_target.pose))
    {
      error = "target pose contains a non-finite value";
      return false;
    }

    tf2::Quaternion requested_orientation;
    tf2::fromMsg(requested_target.pose.orientation, requested_orientation);
    const double requested_norm = requested_orientation.length();
    if (request.target_is_relative)
    {
      const double translation = std::hypot(
        std::hypot(requested_target.pose.position.x, requested_target.pose.position.y),
        requested_target.pose.position.z);
      if (translation > max_relative_translation_m_)
      {
        error = "relative translation exceeds configured limit of " +
          std::to_string(max_relative_translation_m_) + " m";
        return false;
      }
      if (requested_norm < 1e-9)
        requested_orientation.setValue(0.0, 0.0, 0.0, 1.0);
      else
        requested_orientation.normalize();
      const double rotation = 2.0 * std::acos(std::clamp(
        std::abs(static_cast<double>(requested_orientation.w())), 0.0, 1.0));
      if (rotation > max_relative_rotation_rad_)
      {
        error = "relative rotation exceeds configured limit of " +
          std::to_string(max_relative_rotation_rad_) + " rad";
        return false;
      }
    }
    else
    {
      if (requested_norm < 1e-9)
      {
        error = "absolute target orientation quaternion is zero";
        return false;
      }
      requested_orientation.normalize();
    }

    try
    {
      const auto current_tool0 = tf_buffer_->lookupTransform(
        planning_frame, tool0, tf2::TimePointZero, std::chrono::seconds(1));
      tf2::Transform planning_tool0_start;
      tf2::fromMsg(current_tool0.transform, planning_tool0_start);
      planning_tcp_start = planning_tool0_start * profile.tool0_to_tcp;
    }
    catch (const tf2::TransformException& exception)
    {
      error = "cannot resolve current " + tool0 + " pose in planning frame " +
        planning_frame + ": " + exception.what();
      return false;
    }

    tf2::Transform reference_tcp_target;
    if (request.target_is_relative)
    {
      geometry_msgs::msg::TransformStamped current_tool0;
      try
      {
        current_tool0 = tf_buffer_->lookupTransform(
          requested_target.header.frame_id, tool0, tf2::TimePointZero,
          std::chrono::seconds(1));
      }
      catch (const tf2::TransformException& exception)
      {
        error = "cannot resolve current " + tool0 + " pose in frame " +
          requested_target.header.frame_id + ": " + exception.what();
        return false;
      }

      tf2::Transform reference_tool0_current;
      tf2::fromMsg(current_tool0.transform, reference_tool0_current);
      reference_tcp_target = reference_tool0_current * profile.tool0_to_tcp;
      reference_tcp_target.setOrigin(reference_tcp_target.getOrigin() + tf2::Vector3(
        requested_target.pose.position.x, requested_target.pose.position.y,
        requested_target.pose.position.z));
      tf2::Quaternion resolved_orientation =
        requested_orientation * reference_tcp_target.getRotation();
      resolved_orientation.normalize();
      reference_tcp_target.setRotation(resolved_orientation);
    }
    else
    {
      reference_tcp_target.setOrigin(tf2::Vector3(
        requested_target.pose.position.x, requested_target.pose.position.y,
        requested_target.pose.position.z));
      reference_tcp_target.setRotation(requested_orientation);
    }

    tf2::Transform planning_reference;
    planning_reference.setIdentity();
    if (planning_frame != requested_target.header.frame_id)
    {
      try
      {
        const auto transform = tf_buffer_->lookupTransform(
          planning_frame, requested_target.header.frame_id, tf2::TimePointZero,
          std::chrono::seconds(1));
        tf2::fromMsg(transform.transform, planning_reference);
      }
      catch (const tf2::TransformException& exception)
      {
        error = "cannot transform target into planning frame " + planning_frame +
          ": " + exception.what();
        return false;
      }
    }

    const tf2::Transform planning_tcp_target = planning_reference * reference_tcp_target;
    const tf2::Transform planning_tool0_target =
      planning_tcp_target * profile.tool0_to_tcp.inverse();
    tcp_target.header.frame_id = planning_frame;
    tcp_target.header.stamp = node_->now();
    tf2::toMsg(planning_tcp_target, tcp_target.pose);
    tool0_target.header = tcp_target.header;
    tf2::toMsg(planning_tool0_target, tool0_target.pose);
    return true;
  }

  bool time_parameterize(moveit::planning_interface::MoveGroupInterface& arm,
                         const moveit::core::RobotStatePtr& start_state,
                         moveit_msgs::msg::RobotTrajectory& trajectory,
                         double velocity_scaling, double acceleration_scaling)
  {
    robot_trajectory::RobotTrajectory timed(arm.getRobotModel(), arm.getName());
    timed.setRobotTrajectoryMsg(*start_state, trajectory);
    trajectory_processing::TimeOptimalTrajectoryGeneration parameterization;
    if (!parameterization.computeTimeStamps(timed, velocity_scaling, acceleration_scaling))
      return false;
    if (!trajectory_processing::RuckigSmoothing::applySmoothing(
        timed, velocity_scaling, acceleration_scaling))
    {
      RCLCPP_ERROR(node_->get_logger(), "Ruckig jerk-limited smoothing failed");
      return false;
    }
    timed.getRobotTrajectoryMsg(trajectory);
    return true;
  }

  void publish(const moveit::core::RobotStatePtr& start_state,
               const moveit_msgs::msg::RobotTrajectory& trajectory)
  {
    moveit_msgs::msg::DisplayTrajectory display;
    display.model_id = "g1_upper_body";
    moveit::core::robotStateToRobotStateMsg(*start_state, display.trajectory_start);
    display.trajectory.push_back(trajectory);
    display_pub_->publish(display);
  }

  void execute_arm_trajectory(
      const moveit_msgs::msg::RobotTrajectory& trajectory, double duration,
      PlanSingleArm::Response& response)
  {
    response.execution_requested = true;
    if (!follow_client_->wait_for_action_server(std::chrono::seconds(2)))
    {
      response.execution_status =
        "guarded " + hardware_arm_ + "-arm action server is unavailable; start and arm the hardware gateway";
      return;
    }

    FollowJointTrajectory::Goal goal;
    goal.trajectory = trajectory.joint_trajectory;
    auto goal_future = follow_client_->async_send_goal(goal);
    if (goal_future.wait_for(std::chrono::seconds(3)) != std::future_status::ready)
    {
      response.execution_status = "timed out waiting for gateway trajectory acceptance";
      return;
    }
    const auto goal_handle = goal_future.get();
    if (!goal_handle)
    {
      response.execution_status =
        "gateway rejected trajectory (not armed/holding, stale state, service conflict, or safety latch)";
      return;
    }
    response.execution_accepted = true;

    auto result_future = follow_client_->async_get_result(goal_handle);
    const auto timeout = std::chrono::duration<double>(duration + execution_result_margin_s_);
    if (result_future.wait_for(timeout) != std::future_status::ready)
    {
      follow_client_->async_cancel_goal(goal_handle);
      response.execution_status = "execution result timed out; cancellation requested";
      return;
    }

    const auto wrapped = result_future.get();
    const bool action_succeeded = wrapped.code == rclcpp_action::ResultCode::SUCCEEDED;
    const bool controller_succeeded = wrapped.result &&
      wrapped.result->error_code == FollowJointTrajectory::Result::SUCCESSFUL;
    response.execution_success = action_succeeded && controller_succeeded;
    if (wrapped.result && !wrapped.result->error_string.empty())
      response.execution_status = wrapped.result->error_string;
    else if (response.execution_success)
      response.execution_status = "guarded trajectory completed and final pose is held";
    else
      response.execution_status = "guarded trajectory aborted or cancelled";
  }

  void plan(const std::shared_ptr<PlanSingleArm::Request> request,
            std::shared_ptr<PlanSingleArm::Response> response)
  {
    std::lock_guard<std::mutex> lock(planning_mutex_);
    const auto begin = std::chrono::steady_clock::now();
    const std::string arm_name = upper(request->arm);
    const std::string mode = upper(request->mode.empty() ? "AUTO" : request->mode);
    response->execution_requested = request->execute;
    if (arm_name != "LEFT" && arm_name != "RIGHT")
    {
      response->status = "arm must be LEFT or RIGHT";
      return;
    }
    if (mode != "AUTO" && mode != "LINEAR" && mode != "OMPL")
    {
      response->status = "mode must be AUTO, LINEAR or OMPL";
      return;
    }
    if (request->execute && arm_name != upper(hardware_arm_))
    {
      response->status = "hardware execution is configured for " + upper(hardware_arm_) + " arm only";
      response->execution_status = response->status;
      return;
    }

    auto& arm = arm_name == "LEFT" ? left_arm_ : right_arm_;
    const std::string tool0 = arm_name == "LEFT" ? "left_tool0" : "right_tool0";
    if (!arm.setEndEffectorLink(tool0))
    {
      response->status = "MoveIt group does not contain expected end link " + tool0;
      return;
    }
    const ToolProfile* tool_profile =
      select_tool_profile(request->tool_profile, arm_name, response->status);
    if (!tool_profile)
      return;
    response->selected_tool_profile = tool_profile->name;
    const double step = std::clamp(request->linear_step > 0.0 ? request->linear_step : 0.03, 0.005, 0.08);
    // MoveIt's relative jump test is disabled because it refuses to evaluate
    // deliberately sparse paths (<10 points).  We apply an absolute per-joint
    // delta limit to every adjacent sparse IK solution instead.
    const double jump = request->jump_threshold > 0.0 ? request->jump_threshold : 0.60;
    const double velocity_default = request->execute ? max_execution_velocity_scaling_ : 0.35;
    const double acceleration_default = request->execute ? max_execution_acceleration_scaling_ : 0.30;
    const double velocity = std::clamp(
        request->velocity_scaling > 0.0 ? request->velocity_scaling : velocity_default,
        0.01, 1.0);
    const double acceleration = std::clamp(
        request->acceleration_scaling > 0.0 ? request->acceleration_scaling : acceleration_default,
        0.01, 1.0);
    if (request->execute && velocity > max_execution_velocity_scaling_ + 1e-9)
    {
      response->status = "execute velocity_scaling exceeds configured guarded maximum " +
        std::to_string(max_execution_velocity_scaling_);
      response->execution_status = response->status;
      return;
    }
    if (request->execute && acceleration > max_execution_acceleration_scaling_ + 1e-9)
    {
      response->status = "execute acceleration_scaling exceeds configured guarded maximum " +
        std::to_string(max_execution_acceleration_scaling_);
      response->execution_status = response->status;
      return;
    }
    auto start_state = arm.getCurrentState(2.0);
    if (!start_state)
    {
      response->status = "no current joint state";
      return;
    }
    geometry_msgs::msg::PoseStamped tcp_target;
    geometry_msgs::msg::PoseStamped tool0_target;
    tf2::Transform planning_tcp_start;
    if (!resolve_target(*request, tool0, *tool_profile, arm.getPlanningFrame(), tcp_target,
                        tool0_target, planning_tcp_start, response->status))
      return;
    response->resolved_target = tcp_target;
    response->resolved_tcp_target = tcp_target;
    response->resolved_tool0_target = tool0_target;
    arm.setStartState(*start_state);
    arm.setPlanningTime(5.0);
    arm.setNumPlanningAttempts(5);
    arm.setMaxVelocityScalingFactor(velocity);
    arm.setMaxAccelerationScalingFactor(acceleration);

    if (mode == "AUTO" || mode == "LINEAR")
    {
      arm.setPoseReferenceFrame(tool0_target.header.frame_id);
      const auto waypoints = tcp_waypoints_as_tool0(
        planning_tcp_start, tcp_target, *tool_profile, step);
      moveit_msgs::msg::RobotTrajectory linear_trajectory;
      response->linear_fraction = arm.computeCartesianPath(
          waypoints, step, 0.0, linear_trajectory, true);
      const bool complete = response->linear_fraction >= 0.999 &&
                            !linear_trajectory.joint_trajectory.points.empty();
      const bool jump_valid = complete && !has_joint_jump(linear_trajectory, jump);
      const bool timing_valid = jump_valid &&
          time_parameterize(arm, start_state, linear_trajectory, velocity, acceleration);
      if (!complete)
        response->linear_status = "incomplete (IK, limit, or collision rejection)";
      else if (!jump_valid)
        response->linear_status = "rejected by sparse joint-jump limit";
      else if (!timing_valid)
        response->linear_status = "time parameterization failed";
      else
        response->linear_status = "valid";

      if (timing_valid)
      {
        response->success = true;
        response->used_mode = "LINEAR";
        response->trajectory = std::move(linear_trajectory);
        response->status = "collision-free approximate Cartesian path";
      }
      else if (mode == "LINEAR")
      {
        response->status = "linear path incomplete, in collision, or could not be time-parameterized";
      }
    }

    if (!response->success && mode != "LINEAR")
    {
      arm.setStartState(*start_state);
      arm.setPoseTarget(tool0_target, tool0);
      moveit::planning_interface::MoveGroupInterface::Plan ompl_plan;
      if (static_cast<bool>(arm.plan(ompl_plan)))
      {
        response->success = true;
        response->used_mode = "OMPL";
        response->trajectory = ompl_plan.trajectory_;
        response->status = mode == "AUTO" ? "linear path rejected; OMPL fallback succeeded" : "OMPL succeeded";
      }
      else
      {
        response->status = mode == "AUTO" ? "linear path rejected and OMPL fallback failed" : "OMPL failed";
      }
      arm.clearPoseTargets();
    }

    response->planning_time =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
    if (response->success)
    {
      trajectory_metrics(response->trajectory, *response);
      publish(start_state, response->trajectory);
      if (shadow_trajectory_pub_)
        shadow_trajectory_pub_->publish(response->trajectory.joint_trajectory);
      RCLCPP_INFO(node_->get_logger(),
                  "%s %s profile=%s: %zu points, fraction %.3f, duration %.3f s, "
                  "planning %.3f s",
                  arm_name.c_str(), response->used_mode.c_str(), tool_profile->name.c_str(),
                  response->trajectory.joint_trajectory.points.size(), response->linear_fraction,
                  response->duration, response->planning_time);
      if (request->execute)
      {
        execute_arm_trajectory(response->trajectory, response->duration, *response);
        RCLCPP_WARN(node_->get_logger(), "%s execution accepted=%s success=%s: %s", arm_name.c_str(),
                    response->execution_accepted ? "true" : "false",
                    response->execution_success ? "true" : "false",
                    response->execution_status.c_str());
      }
    }
    else
    {
      RCLCPP_WARN(node_->get_logger(), "%s %s failed: %s", arm_name.c_str(), mode.c_str(),
                  response->status.c_str());
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("g1_single_arm_planner", options);
  if (!node->has_parameter("hardware_arm"))
    node->declare_parameter<std::string>("hardware_arm", "right");
  if (!node->has_parameter("add_demo_table"))
    node->declare_parameter("add_demo_table", true);
  if (!node->has_parameter("add_demo_obstacle"))
    node->declare_parameter("add_demo_obstacle", false);
  if (!node->has_parameter("publish_shadow_trajectory"))
    node->declare_parameter("publish_shadow_trajectory", false);
    if (!node->has_parameter("shadow_trajectory_topic"))
    node->declare_parameter<std::string>(
        "shadow_trajectory_topic", "/g1/shadow_executor/trajectory");
  if (!node->has_parameter("max_relative_translation_m"))
    node->declare_parameter("max_relative_translation_m", 0.20);
  if (!node->has_parameter("max_relative_rotation_rad"))
    node->declare_parameter("max_relative_rotation_rad", M_PI / 3.0);
  if (!node->has_parameter("max_linear_rotation_step_rad"))
    node->declare_parameter("max_linear_rotation_step_rad", 0.15);
  if (!node->has_parameter("max_execution_velocity_scaling"))
    node->declare_parameter("max_execution_velocity_scaling", 0.60);
  if (!node->has_parameter("max_execution_acceleration_scaling"))
    node->declare_parameter("max_execution_acceleration_scaling", 1.00);
  if (!node->has_parameter("execution_result_margin_s"))
    node->declare_parameter("execution_result_margin_s", 10.0);
  if (!node->has_parameter("rviz_auto_velocity_scaling"))
    node->declare_parameter("rviz_auto_velocity_scaling", 0.50);
  if (!node->has_parameter("rviz_auto_acceleration_scaling"))
    node->declare_parameter("rviz_auto_acceleration_scaling", 0.50);
  auto server = std::make_shared<SingleArmPlannerServer>(node);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
