#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int64_multi_array.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include "g1_upper_body_moveit_config/srv/validate_task_state.hpp"
#include "g1_upper_body_moveit_config/srv/validate_task_trajectory.hpp"
#include "task_scene_revision.hpp"

class TaskStateValidator : public rclcpp::Node
{
public:
  using Service = g1_upper_body_moveit_config::srv::ValidateTaskState;
  using BatchService = g1_upper_body_moveit_config::srv::ValidateTaskTrajectory;
  TaskStateValidator() : Node("g1_task_state_validator", rclcpp::NodeOptions()
      .automatically_declare_parameters_from_overrides(true)) {}

  void initialize()
  {
    monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(shared_from_this(), "robot_description");
    if (!monitor_->getPlanningScene()) throw std::runtime_error("No task robot model");
    monitor_->stopPublishingPlanningScene();
    // Do not combine an asynchronous full snapshot with queued incremental
    // diffs: old ADD messages can resurrect obstacles removed from the snapshot.
    // Geometry notifications invalidate the cache; only a full service reply
    // replaces it. Robot joint updates do not invalidate this geometry cache.
    scene_updates_ = create_subscription<moveit_msgs::msg::PlanningScene>("/monitored_planning_scene", 10,
      [this](moveit_msgs::msg::PlanningScene::ConstSharedPtr msg) {
        if (scene_revisions_.changed(*msg)) {
          scene_dirty_ = true;
          ++geometry_revision_;
        }
      });
    scene_client_ = create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    progress_pub_ = create_publisher<std_msgs::msg::UInt64MultiArray>(
      "/g1/task/validation_progress", rclcpp::QoS(10).reliable());
    service_ = create_service<Service>("/g1/task/validate_state",
      [this](const std::shared_ptr<Service::Request> request, std::shared_ptr<Service::Response> response) {
        validate(*request, *response);
      });
    batch_service_ = create_service<BatchService>("/g1/task/validate_trajectory",
      [this](const std::shared_ptr<BatchService::Request> request,
             std::shared_ptr<BatchService::Response> response) {
        validateBatch(*request, *response);
      });
  }

private:
  void refreshScene()
  {
    const auto now = std::chrono::steady_clock::now();
    if (pending_scene_ && now - requested_at_ < std::chrono::seconds(2)) return;
    if (pending_scene_) scene_client_->remove_pending_request(*pending_scene_);
    auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    using Components = moveit_msgs::msg::PlanningSceneComponents;
    request->components.components = Components::SCENE_SETTINGS | Components::ROBOT_STATE |
        Components::ROBOT_STATE_ATTACHED_OBJECTS | Components::WORLD_OBJECT_GEOMETRY |
        Components::OCTOMAP | Components::TRANSFORMS | Components::ALLOWED_COLLISION_MATRIX |
        Components::LINK_PADDING_AND_SCALING;
    requested_at_ = now;
    const auto sequence = ++scene_sequence_;
    const auto revision = geometry_revision_;
    auto future = scene_client_->async_send_request(request,
      [this, sequence, revision](rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedFuture response) {
        if (sequence != scene_sequence_) return;
        pending_scene_.reset();
        auto snapshot = response.get()->scene;
        snapshot.is_diff = false;
        planning_scene_monitor::LockedPlanningSceneRW cache(monitor_);
        const bool loaded = cache->setPlanningSceneMsg(snapshot);
        scene_dirty_ = !loaded || revision != geometry_revision_;
      });
    pending_scene_ = future.request_id;
  }

  void validate(const Service::Request& request, Service::Response& response)
  {
    response.valid = false;
    response.code = "INVALID_REQUEST";
    const std::set<std::string> stages{"prepare", "recover", "approach", "insert", "rotate", "withdraw", "return_ready", "diagnostic", "hand_transition"};
    if ((request.arm != "left" && request.arm != "right") || !stages.count(request.stage)) {
      response.message = "Unknown arm or task stage"; return;
    }
    planning_scene_monitor::LockedPlanningSceneRO scene(monitor_);
    if (scene_dirty_ || request.snapshot_object_id.rfind("task_measured_rgbd_snapshot_", 0) != 0 ||
        !scene->getWorld()->hasObject(request.snapshot_object_id)) {
      refreshScene();
      response.code = "SCENE_NOT_READY"; response.message = "The requested observation is not in this planning scene"; return;
    }
    const bool contact = request.stage == "insert" || request.stage == "rotate" || request.stage == "withdraw";
    if ((contact || !request.target_object_id.empty()) && (request.target_object_id.rfind("task_ring_contact_sweep_", 0) != 0 ||
                    !scene->getWorld()->hasObject(request.target_object_id))) {
      refreshScene();
      response.code = "TARGET_NOT_SEPARATED"; response.message = "No separately identified ring contact surface"; return;
    }
    auto state = scene->getCurrentState();
    const auto& joint = request.robot_state.joint_state;
    if (joint.name.size() != joint.position.size()) { response.message = "Joint dimensions differ"; return; }
    std::set<std::string> seen;
    const auto& variables = state.getRobotModel()->getVariableNames();
    const std::set<std::string> known(variables.begin(), variables.end());
    for (std::size_t i = 0; i < joint.name.size(); ++i) {
      if (!known.count(joint.name[i]) || !std::isfinite(joint.position[i]) || !seen.insert(joint.name[i]).second) {
        response.message = "Unknown, nonfinite or duplicate joint"; return;
      }
    }
    for (const auto& side : {std::string("left"), std::string("right")}) {
      for (const auto& name : {"shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw"}) {
        if (!seen.count(side + "_" + name + "_joint")) { response.message = "Missing measured arm joints"; return; }
      }
      for (const auto& part : {"little_1", "ring_1", "middle_1", "index_1", "thumb_2", "thumb_1"}) {
        const auto name = side + "_" + part + "_joint";
        if (known.count(name) && !seen.count(name)) {
          response.message = "Missing articulated hand state: " + name; return;
        }
      }
    }
    if (state.getRobotModel()->hasJointModelGroup("left_arm_with_waist") && !seen.count("waist_yaw_joint")) {
      response.message = "Missing measured waist yaw"; return;
    }
    moveit::core::robotStateMsgToRobotState(request.robot_state, state);
    state.update();
    if (!state.satisfiesBounds()) {
      response.code = "JOINT_BOUNDS"; response.message = "State exceeds model joint bounds"; return;
    }
    // Per-request copy only: no persistent/global collision relaxation, even on
    // cancellation, process death, failed IK or the other arm's planning calls.
    auto matrix = scene->getAllowedCollisionMatrix();
    for (const auto& link : state.getRobotModel()->getLinkModelNames()) {
      matrix.setEntry(link, request.snapshot_object_id, false);
      if (!request.target_object_id.empty()) matrix.setEntry(link, request.target_object_id, false);
    }
    for (const auto& side : {std::string("left"), std::string("right")}) {
      for (const auto& part : {"index_2", "index_rubber_2", "index_pad", "index_tip"}) {
        const auto link = side + "_" + part;
        if (!state.getRobotModel()->hasLinkModel(link)) {
          response.code = "MODEL_MISMATCH"; response.message = "Task index-only model not loaded"; return;
        }
        matrix.setEntry(link, request.snapshot_object_id, false);
        if (!request.target_object_id.empty()) matrix.setEntry(link, request.target_object_id, contact && side == request.arm);
      }
    }
    collision_detection::CollisionRequest check;
    check.contacts = true; check.max_contacts = 100; check.max_contacts_per_pair = 1;
    collision_detection::CollisionResult result;
    scene->checkCollision(check, result, state, matrix);
    for (const auto& pair : result.contacts) {
      for (const auto& hit : pair.second) {
        moveit_msgs::msg::ContactInformation info;
        info.header.frame_id = scene->getPlanningFrame();
        info.position.x = hit.pos.x(); info.position.y = hit.pos.y(); info.position.z = hit.pos.z();
        info.normal.x = hit.normal.x(); info.normal.y = hit.normal.y(); info.normal.z = hit.normal.z();
        info.depth = hit.depth;
        info.contact_body_1 = hit.body_name_1; info.contact_body_2 = hit.body_name_2;
        response.contacts.push_back(info);
      }
    }
    response.valid = !result.collision;
    response.code = response.valid ? "VALID" : "COLLISION";
    response.message = response.valid ? "State passed scoped stage checks" : "Disallowed self/environment contact";
  }

  void validateBatch(const BatchService::Request& request, BatchService::Response& response)
  {
    response.valid = false;
    response.checked_samples = 0;
    response.first_invalid_sample = 0;
    response.code = "INVALID_REQUEST";
    const std::set<std::string> stages{
      "prepare", "recover", "approach", "insert", "rotate", "withdraw",
      "return_ready", "diagnostic", "hand_transition"};
    if ((request.arm != "left" && request.arm != "right") || !stages.count(request.stage))
    {
      response.message = "Unknown arm or task stage";
      return;
    }
    const std::size_t sample_count = request.sample_count;
    const std::size_t joint_count = request.joint_names.size();
    if (sample_count == 0 || sample_count > 100000 || joint_count == 0 || joint_count > 512 ||
        joint_count > std::numeric_limits<std::size_t>::max() / sample_count ||
        request.positions.size() != sample_count * joint_count)
    {
      response.message = "Trajectory dimensions differ or exceed the batch limit";
      return;
    }

    planning_scene_monitor::LockedPlanningSceneRO scene(monitor_);
    if (scene_dirty_ || request.snapshot_object_id.rfind("task_measured_rgbd_snapshot_", 0) != 0 ||
        !scene->getWorld()->hasObject(request.snapshot_object_id))
    {
      refreshScene();
      response.code = "SCENE_NOT_READY";
      response.message = "The requested observation is not in this planning scene";
      return;
    }
    const bool contact = request.stage == "insert" || request.stage == "rotate" ||
      request.stage == "withdraw";
    if ((contact || !request.target_object_id.empty()) &&
        (request.target_object_id.rfind("task_ring_contact_sweep_", 0) != 0 ||
         !scene->getWorld()->hasObject(request.target_object_id)))
    {
      refreshScene();
      response.code = "TARGET_NOT_SEPARATED";
      response.message = "No separately identified ring contact surface";
      return;
    }

    auto state = scene->getCurrentState();
    std::set<std::string> seen;
    const auto& variables = state.getRobotModel()->getVariableNames();
    const std::set<std::string> known(variables.begin(), variables.end());
    for (const auto& name : request.joint_names)
    {
      if (!known.count(name) || !seen.insert(name).second)
      {
        response.message = "Unknown or duplicate joint";
        return;
      }
    }
    for (const auto& side : {std::string("left"), std::string("right")})
    {
      for (const auto& name : {
          "shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
          "wrist_roll", "wrist_pitch", "wrist_yaw"})
      {
        if (!seen.count(side + "_" + name + "_joint"))
        {
          response.message = "Missing measured arm joints";
          return;
        }
      }
      for (const auto& part : {
          "little_1", "ring_1", "middle_1", "index_1", "thumb_2", "thumb_1"})
      {
        const auto name = side + "_" + part + "_joint";
        if (known.count(name) && !seen.count(name))
        {
          response.message = "Missing articulated hand state: " + name;
          return;
        }
      }
    }
    if (state.getRobotModel()->hasJointModelGroup("left_arm_with_waist") &&
        !seen.count("waist_yaw_joint"))
    {
      response.message = "Missing measured waist yaw";
      return;
    }

    auto matrix = scene->getAllowedCollisionMatrix();
    for (const auto& link : state.getRobotModel()->getLinkModelNames())
    {
      matrix.setEntry(link, request.snapshot_object_id, false);
      if (!request.target_object_id.empty())
        matrix.setEntry(link, request.target_object_id, false);
    }
    for (const auto& side : {std::string("left"), std::string("right")})
    {
      for (const auto& part : {"index_2", "index_rubber_2", "index_pad", "index_tip"})
      {
        const auto link = side + "_" + part;
        if (!state.getRobotModel()->hasLinkModel(link))
        {
          response.code = "MODEL_MISMATCH";
          response.message = "Task index-only model not loaded";
          return;
        }
        matrix.setEntry(link, request.snapshot_object_id, false);
        if (!request.target_object_id.empty())
          matrix.setEntry(link, request.target_object_id, contact && side == request.arm);
      }
    }

    collision_detection::CollisionRequest check;
    check.contacts = true;
    check.max_contacts = 100;
    check.max_contacts_per_pair = 1;
    // Progress stays inside this single scene-locked request. No smaller
    // batches or relaxed collision checks are needed for a responsive UI.
    std_msgs::msg::UInt64MultiArray progress;
    progress.layout.dim.resize(3);
    progress.layout.dim[0].label = request.snapshot_object_id;
    progress.layout.dim[1].label = request.arm;
    progress.layout.dim[2].label = request.stage;
    const auto started = std::chrono::steady_clock::now();
    progress.data = {static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      started.time_since_epoch()).count()), 0, sample_count};
    auto last_progress = started;
    progress_pub_->publish(progress);
    for (std::size_t sample = 0; sample < sample_count; ++sample)
    {
      const std::size_t offset = sample * joint_count;
      for (std::size_t joint = 0; joint < joint_count; ++joint)
      {
        const double value = request.positions[offset + joint];
        if (!std::isfinite(value))
        {
          response.checked_samples = sample + 1;
          response.first_invalid_sample = sample;
          response.message = "Unknown, nonfinite or duplicate joint";
          return;
        }
        state.setVariablePosition(request.joint_names[joint], value);
      }
      state.update();
      if (!state.satisfiesBounds())
      {
        response.checked_samples = sample + 1;
        response.first_invalid_sample = sample;
        response.code = "JOINT_BOUNDS";
        response.message = "State exceeds model joint bounds";
        return;
      }
      collision_detection::CollisionResult result;
      scene->checkCollision(check, result, state, matrix);
      response.checked_samples = sample + 1;
      if (!result.collision)
      {
        const auto now = std::chrono::steady_clock::now();
        if (now - last_progress >= std::chrono::milliseconds(100) || sample + 1 == sample_count)
        {
          progress.data[1] = sample + 1;
          progress_pub_->publish(progress);
          last_progress = now;
        }
        continue;
      }
      response.first_invalid_sample = sample;
      response.code = "COLLISION";
      response.message = "Disallowed self/environment contact";
      for (const auto& pair : result.contacts)
      {
        for (const auto& hit : pair.second)
        {
          moveit_msgs::msg::ContactInformation info;
          info.header.frame_id = scene->getPlanningFrame();
          info.position.x = hit.pos.x();
          info.position.y = hit.pos.y();
          info.position.z = hit.pos.z();
          info.normal.x = hit.normal.x();
          info.normal.y = hit.normal.y();
          info.normal.z = hit.normal.z();
          info.depth = hit.depth;
          info.contact_body_1 = hit.body_name_1;
          info.contact_body_2 = hit.body_name_2;
          response.contacts.push_back(info);
        }
      }
      return;
    }
    response.valid = true;
    response.first_invalid_sample = request.sample_count;
    response.code = "VALID";
    response.message = "Trajectory passed scoped stage checks";
  }
  planning_scene_monitor::PlanningSceneMonitorPtr monitor_;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr scene_client_;
  rclcpp::Subscription<moveit_msgs::msg::PlanningScene>::SharedPtr scene_updates_;
  bool scene_dirty_ = true;
  TaskSceneRevision scene_revisions_;
  unsigned int geometry_revision_ = 0;
  std::optional<int64_t> pending_scene_;
  std::chrono::steady_clock::time_point requested_at_;
  unsigned int scene_sequence_ = 0;
  rclcpp::Service<Service>::SharedPtr service_;
  rclcpp::Service<BatchService>::SharedPtr batch_service_;
  rclcpp::Publisher<std_msgs::msg::UInt64MultiArray>::SharedPtr progress_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TaskStateValidator>();
  node->initialize();
  rclcpp::spin(node);
  rclcpp::shutdown();
}
