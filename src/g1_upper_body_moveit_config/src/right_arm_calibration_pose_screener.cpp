#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using namespace std::chrono_literals;

namespace
{
constexpr std::array<const char*, 7> kJointNames{
  "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
  "right_shoulder_yaw_joint", "right_elbow_joint",
  "right_wrist_roll_joint", "right_wrist_pitch_joint",
  "right_wrist_yaw_joint"};

using Pose = std::array<double, 7>;

double normalized_distance(const Pose& a, const Pose& b,
  const std::array<double, 7>& scale)
{
  double sum = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i)
  {
    const double d = (a[i] - b[i]) / scale[i];
    sum += d * d;
  }
  return std::sqrt(sum);
}
}  // namespace

class CalibrationPoseScreener : public rclcpp::Node
{
public:
  CalibrationPoseScreener() : Node("right_arm_calibration_pose_screener")
  {
    output_path_ = declare_parameter<std::string>("output_path",
      "/home/syf/g1_upper_body_ws/calibration/right_arm_candidate_poses.yaml");
    candidate_count_ = static_cast<int>(std::clamp<std::int64_t>(
      declare_parameter("candidate_count", 12), 2, 30));
    random_samples_ = static_cast<int>(std::clamp<std::int64_t>(
      declare_parameter("random_samples", 12000), 500, 100000));
    interpolation_step_ = std::clamp(
      declare_parameter("interpolation_step_rad", 0.025), 0.005, 0.10);
    minimum_pose_distance_ = std::clamp(
      declare_parameter("minimum_pose_distance_rad", 0.28), 0.10, 1.0);
    seed_ = declare_parameter("random_seed", 20260824);
    // An empty vector default is untyped in some ROS 2 Humble parameter paths.
    // Use a typed non-finite sentinel, then normalize it back to "not set".
    target_q_ = declare_parameter<std::vector<double>>(
      "target_q", {std::numeric_limits<double>::quiet_NaN()});
    if (target_q_.size() == 1 && !std::isfinite(target_q_.front()))
      target_q_.clear();
    target_check_output_ = declare_parameter<std::string>("target_check_output",
      "/home/syf/g1_upper_body_ws/calibration/right_arm_resume_path.yaml");
    elbow_drag_range_ = std::clamp(
      declare_parameter("elbow_drag_range_rad", 0.0), 0.0, 0.35);

    state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_state_ = std::move(msg);
      });
  }

  bool run()
  {
    robot_model_loader::RobotModelLoader loader(shared_from_this(), "robot_description");
    const auto model = loader.getModel();
    if (!model)
    {
      RCLCPP_ERROR(get_logger(), "Could not load robot_description");
      return false;
    }
    const auto* group = model->getJointModelGroup("right_arm");
    if (!group || group->getActiveJointModelNames().size() != 7)
    {
      RCLCPP_ERROR(get_logger(), "right_arm group is missing or is not 7-DoF");
      return false;
    }

    sensor_msgs::msg::JointState::SharedPtr msg;
    const auto deadline = now() + rclcpp::Duration::from_seconds(5.0);
    while (rclcpp::ok() && now() < deadline)
    {
      rclcpp::spin_some(shared_from_this());
      {
        std::lock_guard<std::mutex> lock(mutex_);
        msg = latest_state_;
      }
      if (msg && msg->name.size() == msg->position.size())
        break;
      rclcpp::sleep_for(20ms);
    }
    if (!msg)
    {
      RCLCPP_ERROR(get_logger(), "No /joint_states received; run the read-only real state bridge");
      return false;
    }

    planning_scene::PlanningScene scene(model);
    moveit::core::RobotState base(model);
    base.setToDefaultValues();
    std::unordered_map<std::string, double> measured;
    for (std::size_t i = 0; i < msg->name.size(); ++i)
      if (std::isfinite(msg->position[i])) measured[msg->name[i]] = msg->position[i];
    for (const auto& variable : model->getVariableNames())
    {
      const auto found = measured.find(variable);
      if (found != measured.end()) base.setVariablePosition(variable, found->second);
    }
    base.update();

    Pose start{};
    for (std::size_t i = 0; i < start.size(); ++i)
    {
      const auto found = measured.find(kJointNames[i]);
      if (found == measured.end())
      {
        RCLCPP_ERROR(get_logger(), "Missing measured joint: %s", kJointNames[i]);
        return false;
      }
      start[i] = found->second;
    }
    if (!valid(scene, base, group, start))
    {
      RCLCPP_ERROR(get_logger(),
        "The measured start pose collides with the conservative payload envelope");
      return false;
    }

    if (elbow_drag_range_ > 0.0)
    {
      Pose lower = start;
      Pose upper = start;
      lower[3] -= elbow_drag_range_;
      upper[3] += elbow_drag_range_;
      if (!valid(scene, base, group, lower) || !valid(scene, base, group, upper) ||
          !segment_valid(scene, base, group, start, lower) ||
          !segment_valid(scene, base, group, start, upper))
      {
        RCLCPP_ERROR(get_logger(),
          "Requested elbow drag range is invalid or colliding with payload envelope");
        return false;
      }
      std::filesystem::create_directories(
        std::filesystem::path(target_check_output_).parent_path());
      std::ofstream output(target_check_output_, std::ios::trunc);
      output << std::setprecision(10)
             << "# MoveIt-checked elbow-only drag range; conservative payload envelope enabled.\n";
      const auto write_pose = [&output](const char* key, const Pose& pose) {
        output << key << ": [";
        for (std::size_t i = 0; i < pose.size(); ++i)
          output << (i ? ", " : "") << pose[i];
        output << "]\n";
      };
      write_pose("start_q", start);
      write_pose("lower_q", lower);
      write_pose("upper_q", upper);
      output << "elbow_range_rad: " << elbow_drag_range_ << '\n';
      RCLCPP_INFO(get_logger(), "Elbow-only +/-%.2f deg drag range is collision-free; wrote %s",
        elbow_drag_range_ * 180.0 / M_PI, target_check_output_.c_str());
      return true;
    }

    if (!target_q_.empty())
    {
      if (target_q_.size() != 7)
      {
        RCLCPP_ERROR(get_logger(), "target_q must contain exactly seven positions");
        return false;
      }
      Pose target{};
      std::copy(target_q_.begin(), target_q_.end(), target.begin());
      if (!valid(scene, base, group, target))
      {
        RCLCPP_ERROR(get_logger(), "Requested resume target is invalid or colliding");
        return false;
      }
      if (!segment_valid(scene, base, group, start, target))
      {
        RCLCPP_ERROR(get_logger(),
          "Direct measured-start -> resume-target interpolation collides; OMPL is required");
        return false;
      }
      std::filesystem::create_directories(
        std::filesystem::path(target_check_output_).parent_path());
      std::ofstream output(target_check_output_, std::ios::trunc);
      output << std::setprecision(10)
             << "# MoveIt-checked direct resume path; conservative payload envelope enabled.\n"
             << "start_q: [";
      for (std::size_t i = 0; i < 7; ++i) output << (i ? ", " : "") << start[i];
      output << "]\ntarget_q: [";
      for (std::size_t i = 0; i < 7; ++i) output << (i ? ", " : "") << target[i];
      output << "]\ninterpolation_step_rad: " << interpolation_step_ << '\n';
      RCLCPP_INFO(get_logger(), "Direct resume path is collision-free; wrote %s",
        target_check_output_.c_str());
      return true;
    }

    // Local excitation limits keep shoulder/elbow movements modest while the
    // larger wrist range improves payload mass/COM observability.
    const std::array<double, 7> range{0.42, 0.34, 0.38, 0.48, 0.70, 0.65, 0.70};
    std::mt19937 generator(static_cast<std::uint32_t>(seed_));
    std::uniform_real_distribution<double> uniform(-1.0, 1.0);
    std::vector<Pose> pool;
    pool.reserve(static_cast<std::size_t>(random_samples_));
    for (int sample = 0; sample < random_samples_; ++sample)
    {
      Pose pose = start;
      for (std::size_t j = 0; j < pose.size(); ++j)
        pose[j] += range[j] * uniform(generator);
      if (valid(scene, base, group, pose)) pool.push_back(pose);
    }
    RCLCPP_INFO(get_logger(), "Collision-free sampled states: %zu / %d",
      pool.size(), random_samples_);

    std::vector<Pose> selected{start};
    while (selected.size() < static_cast<std::size_t>(candidate_count_))
    {
      std::size_t best_index = pool.size();
      double best_score = -1.0;
      for (std::size_t i = 0; i < pool.size(); ++i)
      {
        double diversity = std::numeric_limits<double>::infinity();
        for (const auto& chosen : selected)
          diversity = std::min(diversity, normalized_distance(pool[i], chosen, range));
        if (diversity < minimum_pose_distance_) continue;
        if (!segment_valid(scene, base, group, selected.back(), pool[i])) continue;
        // Favor wrist excitation slightly because it separates payload COM
        // components without requiring large shoulder excursions.
        const double wrist = std::abs(pool[i][4] - start[4]) / range[4] +
          std::abs(pool[i][5] - start[5]) / range[5] +
          0.5 * std::abs(pool[i][6] - start[6]) / range[6];
        const double score = diversity + 0.10 * wrist;
        if (score > best_score)
        {
          best_score = score;
          best_index = i;
        }
      }
      if (best_index == pool.size()) break;
      selected.push_back(pool[best_index]);
      pool.erase(pool.begin() + static_cast<std::ptrdiff_t>(best_index));
    }
    if (selected.size() != static_cast<std::size_t>(candidate_count_))
    {
      RCLCPP_ERROR(get_logger(), "Only selected %zu of %d sequentially collision-free poses",
        selected.size(), candidate_count_);
      return false;
    }

    std::filesystem::create_directories(std::filesystem::path(output_path_).parent_path());
    std::ofstream out(output_path_);
    out << std::setprecision(10);
    out << "# READ-ONLY MoveIt screening; pose_00 is the measured real start.\n";
    out << "# Conservative payload box: 0.300 x 0.140 x 0.120 m.\n";
    out << "# Every listed edge pose_N -> pose_N+1 passed self-collision interpolation.\n";
    out << "joint_names: [";
    for (std::size_t i = 0; i < kJointNames.size(); ++i)
      out << (i ? ", " : "") << kJointNames[i];
    out << "]\nposes:\n";
    for (std::size_t i = 0; i < selected.size(); ++i)
    {
      out << "  - id: " << i << "\n    q: [";
      for (std::size_t j = 0; j < selected[i].size(); ++j)
        out << (j ? ", " : "") << selected[i][j];
      out << "]\n";
    }
    RCLCPP_INFO(get_logger(), "Wrote %zu poses to %s", selected.size(), output_path_.c_str());
    return true;
  }

private:
  bool valid(planning_scene::PlanningScene& scene, moveit::core::RobotState base,
    const moveit::core::JointModelGroup* group, const Pose& pose) const
  {
    base.setJointGroupPositions(group, std::vector<double>(pose.begin(), pose.end()));
    base.update();
    return base.satisfiesBounds(group) && !scene.isStateColliding(base, "right_arm", false);
  }

  bool segment_valid(planning_scene::PlanningScene& scene, const moveit::core::RobotState& base,
    const moveit::core::JointModelGroup* group, const Pose& from, const Pose& to) const
  {
    double largest = 0.0;
    for (std::size_t i = 0; i < from.size(); ++i)
      largest = std::max(largest, std::abs(to[i] - from[i]));
    const int steps = std::max(2, static_cast<int>(std::ceil(largest / interpolation_step_)));
    for (int step = 1; step <= steps; ++step)
    {
      const double alpha = static_cast<double>(step) / steps;
      Pose pose{};
      for (std::size_t i = 0; i < pose.size(); ++i)
        pose[i] = from[i] + alpha * (to[i] - from[i]);
      if (!valid(scene, base, group, pose)) return false;
    }
    return true;
  }

  std::string output_path_;
  int candidate_count_{12};
  int random_samples_{12000};
  int seed_{20260824};
  std::vector<double> target_q_;
  std::string target_check_output_;
  double elbow_drag_range_{0.0};
  double interpolation_step_{0.025};
  double minimum_pose_distance_{0.28};
  std::mutex mutex_;
  sensor_msgs::msg::JointState::SharedPtr latest_state_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CalibrationPoseScreener>();
  const bool success = node->run();
  rclcpp::shutdown();
  return success ? 0 : 1;
}
