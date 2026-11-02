#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>

#include <g1_kdl_ik/capability_map_v1.hpp>

namespace fs = std::filesystem;
using g1_kdl_ik::CapabilityRecordV1;

namespace
{
constexpr std::array<unsigned int, 7> kHaltonBases{ 2, 3, 5, 7, 11, 13, 17 };
constexpr std::int32_t kVoxelBias = 1 << 20;
constexpr double kPi = 3.14159265358979323846;

double halton(std::uint64_t index, unsigned int base)
{
  double value = 0.0;
  double fraction = 1.0;
  while (index > 0)
  {
    fraction /= static_cast<double>(base);
    value += fraction * static_cast<double>(index % base);
    index /= base;
  }
  return value;
}

std::uint64_t voxelKey(std::int32_t ix, std::int32_t iy, std::int32_t iz)
{
  const auto encode = [](std::int32_t value) -> std::uint64_t {
    const std::int64_t shifted = static_cast<std::int64_t>(value) + kVoxelBias;
    if (shifted < 0 || shifted >= (1 << 21))
      throw std::runtime_error("Voxel coordinate exceeds 21-bit key range");
    return static_cast<std::uint64_t>(shifted);
  };
  return (encode(ix) << 42) | (encode(iy) << 21) | encode(iz);
}

std::vector<Eigen::Vector3d> fibonacciSphere(std::size_t count)
{
  std::vector<Eigen::Vector3d> directions;
  directions.reserve(count);
  const double golden_angle = kPi * (3.0 - std::sqrt(5.0));
  for (std::size_t i = 0; i < count; ++i)
  {
    const double z = 1.0 - 2.0 * (static_cast<double>(i) + 0.5) /
                                   static_cast<double>(count);
    const double radius = std::sqrt(std::max(0.0, 1.0 - z * z));
    const double angle = golden_angle * static_cast<double>(i);
    directions.emplace_back(radius * std::cos(angle), radius * std::sin(angle), z);
  }
  return directions;
}

std::size_t orientationBin(const Eigen::Matrix3d& rotation,
                           const std::vector<Eigen::Vector3d>& direction_bins)
{
  const Eigen::Vector3d approach = rotation.col(0).normalized();
  std::size_t direction_index = 0;
  double best_dot = -std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < direction_bins.size(); ++i)
  {
    const double dot = approach.dot(direction_bins[i]);
    if (dot > best_dot)
    {
      best_dot = dot;
      direction_index = i;
    }
  }

  const Eigen::Vector3d reference =
      std::abs(approach.z()) < 0.9 ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitY();
  const Eigen::Vector3d tangent_y = reference.cross(approach).normalized();
  const Eigen::Vector3d tangent_z = approach.cross(tangent_y).normalized();
  const Eigen::Vector3d tool_y = rotation.col(1).normalized();
  double roll = std::atan2(tool_y.dot(tangent_z), tool_y.dot(tangent_y));
  if (roll < 0.0)
    roll += 2.0 * kPi;
  const auto roll_index = std::min<std::size_t>(
      g1_kdl_ik::kRollBins - 1,
      static_cast<std::size_t>(roll / (2.0 * kPi) * g1_kdl_ik::kRollBins));
  return direction_index * g1_kdl_ik::kRollBins + roll_index;
}
}  // namespace

class CapabilitySampler : public rclcpp::Node
{
public:
  CapabilitySampler() : Node("g1_capability_sampler")
  {
    output_dir_ = declare_parameter<std::string>("output_dir");
    model_path_ = declare_parameter<std::string>("model_path");
    samples_per_arm_ = declare_parameter<std::int64_t>("samples_per_arm", 5000000);
    resolution_ = declare_parameter<double>("resolution", 0.01);
    boundary_directions_ = declare_parameter<std::int64_t>("boundary_directions", 642);
    boundary_starts_ = declare_parameter<std::int64_t>("boundary_starts", 4);
    boundary_iterations_ = declare_parameter<std::int64_t>("boundary_iterations", 14);
    if (samples_per_arm_ <= 0 || resolution_ <= 0.0 || boundary_directions_ <= 0 ||
        boundary_starts_ <= 0 || boundary_iterations_ <= 0)
      throw std::runtime_error("All sampling parameters must be positive");
    orientation_directions_ = fibonacciSphere(g1_kdl_ik::kDirectionBins);
    support_directions_ = fibonacciSphere(static_cast<std::size_t>(boundary_directions_));
  }

  void run()
  {
    const auto loader = std::make_shared<robot_model_loader::RobotModelLoader>(
        shared_from_this(), "robot_description");
    const auto model = loader->getModel();
    if (!model)
      throw std::runtime_error("Failed to load G1 robot model");
    fs::create_directories(output_dir_);
    for (const char* group_name : { "left_arm", "right_arm" })
      sampleArm(model, group_name);
  }

private:
  struct Accumulator
  {
    std::int32_t ix{ 0 };
    std::int32_t iy{ 0 };
    std::int32_t iz{ 0 };
    std::uint32_t count{ 0 };
    std::uint8_t flags{ 0 };
    std::array<std::uint8_t, g1_kdl_ik::kOrientationBytes> orientation_mask{};
    std::array<float, 7> seed{};
  };

  using JointArray = std::array<double, 7>;

  void insertPose(std::unordered_map<std::uint64_t, Accumulator>& voxels,
                  const Eigen::Isometry3d& pose, const JointArray& q, bool boundary)
  {
    const Eigen::Vector3d position = pose.translation();
    const auto ix = static_cast<std::int32_t>(std::floor(position.x() / resolution_));
    const auto iy = static_cast<std::int32_t>(std::floor(position.y() / resolution_));
    const auto iz = static_cast<std::int32_t>(std::floor(position.z() / resolution_));
    auto [iterator, inserted] = voxels.try_emplace(voxelKey(ix, iy, iz));
    auto& voxel = iterator->second;
    if (inserted)
    {
      voxel.ix = ix;
      voxel.iy = iy;
      voxel.iz = iz;
      for (std::size_t j = 0; j < q.size(); ++j)
        voxel.seed[j] = static_cast<float>(q[j]);
    }
    if (voxel.count != UINT32_MAX)
      ++voxel.count;
    if (boundary)
      voxel.flags |= g1_kdl_ik::kBoundaryFlag;
    const std::size_t bin = orientationBin(pose.rotation(), orientation_directions_);
    voxel.orientation_mask[bin / 8] |= static_cast<std::uint8_t>(1u << (bin % 8));
  }

  Eigen::Isometry3d evaluate(moveit::core::RobotState& state,
                            const moveit::core::JointModelGroup* group,
                            const std::string& tip, const JointArray& q)
  {
    state.setJointGroupPositions(group, q.data());
    state.updateLinkTransforms();
    return state.getGlobalLinkTransform(tip);
  }

  JointArray maximizeSupport(moveit::core::RobotState& state,
                             const moveit::core::JointModelGroup* group,
                             const std::string& tip, const JointArray& lower,
                             const JointArray& upper, const Eigen::Vector3d& direction,
                             std::uint64_t direction_index)
  {
    JointArray global_best{};
    double global_score = -std::numeric_limits<double>::infinity();
    for (std::int64_t start = 0; start < boundary_starts_; ++start)
    {
      JointArray q{};
      JointArray step{};
      for (std::size_t j = 0; j < q.size(); ++j)
      {
        const std::uint64_t index = 1 + direction_index * boundary_starts_ + start;
        q[j] = lower[j] + halton(index, kHaltonBases[j]) * (upper[j] - lower[j]);
        step[j] = 0.25 * (upper[j] - lower[j]);
      }
      double score = direction.dot(evaluate(state, group, tip, q).translation());
      for (std::int64_t iteration = 0; iteration < boundary_iterations_; ++iteration)
      {
        for (std::size_t j = 0; j < q.size(); ++j)
        {
          JointArray candidate = q;
          double best = score;
          double best_value = q[j];
          for (double sign : { -1.0, 1.0 })
          {
            candidate[j] = std::clamp(q[j] + sign * step[j], lower[j], upper[j]);
            const double value = direction.dot(
                evaluate(state, group, tip, candidate).translation());
            if (value > best)
            {
              best = value;
              best_value = candidate[j];
            }
          }
          q[j] = best_value;
          score = best;
        }
        for (double& value : step)
          value *= 0.58;
      }
      if (score > global_score)
      {
        global_score = score;
        global_best = q;
      }
    }
    return global_best;
  }

  void sampleArm(const moveit::core::RobotModelPtr& model, const std::string& group_name)
  {
    const auto* group = model->getJointModelGroup(group_name);
    if (!group || group->getVariableCount() != 7)
      throw std::runtime_error("Expected a 7-DOF group: " + group_name);
    const std::string tip = group_name == "left_arm" ? "left_tool0" : "right_tool0";
    const auto& variable_names = group->getVariableNames();
    JointArray lower{};
    JointArray upper{};
    for (std::size_t j = 0; j < 7; ++j)
    {
      const auto& bounds = model->getVariableBounds(variable_names[j]);
      lower[j] = bounds.min_position_;
      upper[j] = bounds.max_position_;
    }

    std::unordered_map<std::uint64_t, Accumulator> voxels;
    voxels.reserve(static_cast<std::size_t>(samples_per_arm_ / 12));
    moveit::core::RobotState state(model);
    state.setToDefaultValues();
    const auto start_time = std::chrono::steady_clock::now();

    JointArray q{};
    for (std::int64_t sample = 1; sample <= samples_per_arm_; ++sample)
    {
      for (std::size_t j = 0; j < q.size(); ++j)
      {
        const double u = halton(static_cast<std::uint64_t>(sample), kHaltonBases[j]);
        q[j] = lower[j] + u * (upper[j] - lower[j]);
      }
      insertPose(voxels, evaluate(state, group, tip, q), q, false);
      if (sample % 1000000 == 0)
        RCLCPP_INFO(get_logger(), "%s: %ld/%ld FK samples, %zu voxels",
                    group_name.c_str(), sample, samples_per_arm_, voxels.size());
    }

    for (std::size_t i = 0; i < support_directions_.size(); ++i)
    {
      q = maximizeSupport(state, group, tip, lower, upper, support_directions_[i], i);
      insertPose(voxels, evaluate(state, group, tip, q), q, true);
    }

    std::vector<std::pair<std::uint64_t, Accumulator>> sorted;
    sorted.reserve(voxels.size());
    for (auto& entry : voxels)
      sorted.emplace_back(entry.first, entry.second);
    std::sort(sorted.begin(), sorted.end(), [](const auto& a, const auto& b) {
      return a.first < b.first;
    });

    const fs::path data_path = fs::path(output_dir_) / (group_name + "_capability_v1.bin");
    std::ofstream data(data_path, std::ios::binary);
    if (!data)
      throw std::runtime_error("Cannot open output: " + data_path.string());
    std::uint64_t total_orientations = 0;
    std::size_t boundary_voxels = 0;
    for (const auto& [key, voxel] : sorted)
    {
      (void)key;
      CapabilityRecordV1 record{};
      record.ix = voxel.ix;
      record.iy = voxel.iy;
      record.iz = voxel.iz;
      record.count = voxel.count;
      record.flags = voxel.flags;
      std::copy(voxel.orientation_mask.begin(), voxel.orientation_mask.end(),
                record.orientation_mask);
      for (const auto byte : voxel.orientation_mask)
        record.orientation_count += static_cast<std::uint16_t>(
            __builtin_popcount(static_cast<unsigned int>(byte)));
      std::copy(voxel.seed.begin(), voxel.seed.end(), record.seed);
      total_orientations += record.orientation_count;
      boundary_voxels += (record.flags & g1_kdl_ik::kBoundaryFlag) != 0;
      data.write(reinterpret_cast<const char*>(&record), sizeof(record));
    }
    data.close();

    const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start_time).count();
    const fs::path metadata_path = fs::path(output_dir_) /
                                  (group_name + "_capability_v1.json");
    std::ofstream metadata(metadata_path);
    metadata << std::fixed << std::setprecision(6)
             << "{\n"
             << "  \"format\": \"g1_capability_map_v1\",\n"
             << "  \"group\": \"" << group_name << "\",\n"
             << "  \"base_frame\": \"torso_link\",\n"
             << "  \"tip_frame\": \"" << tip << "\",\n"
             << "  \"model_path\": \"" << model_path_ << "\",\n"
             << "  \"resolution_m\": " << resolution_ << ",\n"
             << "  \"fk_samples\": " << samples_per_arm_ << ",\n"
             << "  \"boundary_directions\": " << boundary_directions_ << ",\n"
             << "  \"boundary_optimizer_starts\": " << boundary_starts_ << ",\n"
             << "  \"boundary_optimizer_iterations\": " << boundary_iterations_ << ",\n"
             << "  \"orientation_axis\": \"tool0 +X\",\n"
             << "  \"direction_bins\": " << g1_kdl_ik::kDirectionBins << ",\n"
             << "  \"roll_bins\": " << g1_kdl_ik::kRollBins << ",\n"
             << "  \"occupied_voxels\": " << sorted.size() << ",\n"
             << "  \"boundary_voxels\": " << boundary_voxels << ",\n"
             << "  \"total_orientation_bins_hit\": " << total_orientations << ",\n"
             << "  \"record_bytes\": " << sizeof(CapabilityRecordV1) << ",\n"
             << "  \"self_collision_filter\": false,\n"
             << "  \"elapsed_seconds\": " << elapsed << "\n"
             << "}\n";
    RCLCPP_INFO(get_logger(),
                "%s complete: %zu voxels, %zu boundary voxels, %.1f MiB, %.2f s",
                group_name.c_str(), sorted.size(), boundary_voxels,
                static_cast<double>(sorted.size() * sizeof(CapabilityRecordV1)) /
                    (1024.0 * 1024.0), elapsed);
  }

  std::string output_dir_;
  std::string model_path_;
  std::int64_t samples_per_arm_{ 5000000 };
  double resolution_{ 0.01 };
  std::int64_t boundary_directions_{ 642 };
  std::int64_t boundary_starts_{ 4 };
  std::int64_t boundary_iterations_{ 14 };
  std::vector<Eigen::Vector3d> orientation_directions_;
  std::vector<Eigen::Vector3d> support_directions_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try
  {
    auto node = std::make_shared<CapabilitySampler>();
    node->run();
  }
  catch (const std::exception& exception)
  {
    RCLCPP_FATAL(rclcpp::get_logger("g1_capability_sampler"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
