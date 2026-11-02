#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>

namespace fs = std::filesystem;

namespace
{
constexpr std::array<unsigned int, 7> kHaltonBases{ 2, 3, 5, 7, 11, 13, 17 };
constexpr std::int32_t kVoxelBias = 1 << 20;

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

#pragma pack(push, 1)
struct VoxelRecord
{
  std::int32_t ix;
  std::int32_t iy;
  std::int32_t iz;
  std::uint32_t count;
  float seed[7];
};
#pragma pack(pop)

static_assert(sizeof(VoxelRecord) == 44, "Unexpected reachability record padding");
}  // namespace

class ReachabilitySampler : public rclcpp::Node
{
public:
  ReachabilitySampler() : Node("g1_reachability_sampler")
  {
    output_dir_ = declare_parameter<std::string>("output_dir");
    model_path_ = declare_parameter<std::string>("model_path");
    samples_per_arm_ = declare_parameter<std::int64_t>("samples_per_arm", 1000000);
    resolution_ = declare_parameter<double>("resolution", 0.005);
    sampling_mode_ = declare_parameter<std::string>("sampling_mode", "global");
    roi_min_ = declare_parameter<std::vector<double>>(
        "roi_min", { -1e9, -1e9, -1e9 });
    roi_max_ = declare_parameter<std::vector<double>>(
        "roi_max", { 1e9, 1e9, 1e9 });
    local_range_fraction_ = declare_parameter<double>("local_range_fraction", 0.30);
    if (samples_per_arm_ <= 0 || resolution_ <= 0.0)
      throw std::runtime_error("samples_per_arm and resolution must be positive");
    if (roi_min_.size() != 3 || roi_max_.size() != 3 ||
        local_range_fraction_ <= 0.0 || local_range_fraction_ > 1.0)
      throw std::runtime_error("Invalid ROI or local_range_fraction");
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
    std::array<float, 7> seed{};
  };

  void sampleArm(const moveit::core::RobotModelPtr& model, const std::string& group_name)
  {
    const auto* group = model->getJointModelGroup(group_name);
    if (!group || group->getVariableCount() != 7)
      throw std::runtime_error("Expected a 7-DOF group: " + group_name);
    const std::string tip = group_name == "left_arm" ? "left_tool0" : "right_tool0";
    const auto& variable_names = group->getVariableNames();

    std::array<double, 7> lower{};
    std::array<double, 7> upper{};
    for (std::size_t j = 0; j < 7; ++j)
    {
      const auto& bounds = model->getVariableBounds(variable_names[j]);
      lower[j] = bounds.min_position_;
      upper[j] = bounds.max_position_;
    }

    std::unordered_map<std::uint64_t, Accumulator> voxels;
    voxels.reserve(static_cast<std::size_t>(samples_per_arm_ * 0.7));
    moveit::core::RobotState state(model);
    state.setToDefaultValues();
    std::vector<double> q(7);
    const std::array<double, 7> zero_center{};
    const std::array<double, 7> ready_center{
      -0.35, group_name == "left_arm" ? 0.25 : -0.25,
       0.15, 0.75, 0.10, -0.20, 0.12
    };
    const auto start = std::chrono::steady_clock::now();
    std::int64_t accepted_samples = 0;

    for (std::int64_t sample = 1; sample <= samples_per_arm_; ++sample)
    {
      for (std::size_t j = 0; j < 7; ++j)
      {
        const double u = halton(static_cast<std::uint64_t>(sample), kHaltonBases[j]);
        if (sampling_mode_ == "front" && sample % 3 != 0)
        {
          const auto& center = sample % 3 == 1 ? zero_center : ready_center;
          const double half_width = local_range_fraction_ * (upper[j] - lower[j]);
          q[j] = std::clamp(center[j] + (2.0 * u - 1.0) * half_width,
                            lower[j], upper[j]);
        }
        else
        {
          q[j] = lower[j] + u * (upper[j] - lower[j]);
        }
      }
      state.setJointGroupPositions(group, q);
      state.updateLinkTransforms();
      const Eigen::Vector3d position = state.getGlobalLinkTransform(tip).translation();
      if (position.x() < roi_min_[0] || position.x() > roi_max_[0] ||
          position.y() < roi_min_[1] || position.y() > roi_max_[1] ||
          position.z() < roi_min_[2] || position.z() > roi_max_[2])
        continue;
      ++accepted_samples;
      const std::int32_t ix = static_cast<std::int32_t>(std::floor(position.x() / resolution_));
      const std::int32_t iy = static_cast<std::int32_t>(std::floor(position.y() / resolution_));
      const std::int32_t iz = static_cast<std::int32_t>(std::floor(position.z() / resolution_));
      const std::uint64_t key = voxelKey(ix, iy, iz);
      auto [iterator, inserted] = voxels.try_emplace(key);
      auto& voxel = iterator->second;
      if (inserted)
      {
        voxel.ix = ix;
        voxel.iy = iy;
        voxel.iz = iz;
        for (std::size_t j = 0; j < 7; ++j)
          voxel.seed[j] = static_cast<float>(q[j]);
      }
      if (voxel.count != UINT32_MAX)
        ++voxel.count;

      if (sample % 200000 == 0)
        RCLCPP_INFO(get_logger(),
                    "%s: %ld/%ld attempts, %ld accepted, %zu occupied voxels",
                    group_name.c_str(), sample, samples_per_arm_,
                    accepted_samples, voxels.size());
    }

    std::vector<std::pair<std::uint64_t, Accumulator>> sorted;
    sorted.reserve(voxels.size());
    for (auto& entry : voxels)
      sorted.emplace_back(entry.first, entry.second);
    std::sort(sorted.begin(), sorted.end(), [](const auto& a, const auto& b) {
      return a.first < b.first;
    });

    const fs::path binary_path = fs::path(output_dir_) / (group_name + "_voxels.bin");
    std::ofstream binary(binary_path, std::ios::binary);
    if (!binary)
      throw std::runtime_error("Cannot open output: " + binary_path.string());
    for (const auto& [key, voxel] : sorted)
    {
      (void)key;
      VoxelRecord record{ voxel.ix, voxel.iy, voxel.iz, voxel.count, {} };
      std::copy(voxel.seed.begin(), voxel.seed.end(), record.seed);
      binary.write(reinterpret_cast<const char*>(&record), sizeof(record));
    }
    binary.close();

    const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();
    const fs::path metadata_path = fs::path(output_dir_) / (group_name + "_metadata.json");
    std::ofstream metadata(metadata_path);
    metadata << std::fixed << std::setprecision(6)
             << "{\n"
             << "  \"format\": \"g1_reachability_voxels_v0\",\n"
             << "  \"group\": \"" << group_name << "\",\n"
             << "  \"base_frame\": \"torso_link\",\n"
             << "  \"tip_frame\": \"" << tip << "\",\n"
             << "  \"model_path\": \"" << model_path_ << "\",\n"
             << "  \"resolution_m\": " << resolution_ << ",\n"
             << "  \"attempted_samples\": " << samples_per_arm_ << ",\n"
             << "  \"accepted_samples\": " << accepted_samples << ",\n"
             << "  \"occupied_voxels\": " << sorted.size() << ",\n"
             << "  \"record_bytes\": " << sizeof(VoxelRecord) << ",\n"
             << "  \"fields\": \"int32 ix,iy,iz; uint32 count; float32 seed[7]\",\n"
             << "  \"sampling\": \"halton_7d_" << sampling_mode_ << "\",\n"
             << "  \"roi_min_m\": [" << roi_min_[0] << ", " << roi_min_[1]
             << ", " << roi_min_[2] << "],\n"
             << "  \"roi_max_m\": [" << roi_max_[0] << ", " << roi_max_[1]
             << ", " << roi_max_[2] << "],\n"
             << "  \"local_range_fraction\": " << local_range_fraction_ << ",\n"
             << "  \"self_collision_filter\": false,\n"
             << "  \"orientation_filter\": false,\n"
             << "  \"seeds_per_voxel\": 1,\n"
             << "  \"elapsed_seconds\": " << elapsed << "\n"
             << "}\n";
    RCLCPP_INFO(get_logger(),
                "%s complete: %ld/%ld accepted, %zu voxels, %.1f MiB, %.2f s",
                group_name.c_str(), accepted_samples, samples_per_arm_, sorted.size(),
                static_cast<double>(sorted.size() * sizeof(VoxelRecord)) / (1024.0 * 1024.0),
                elapsed);
  }

  std::string output_dir_;
  std::string model_path_;
  std::int64_t samples_per_arm_{ 1000000 };
  double resolution_{ 0.005 };
  std::string sampling_mode_{ "global" };
  std::vector<double> roi_min_;
  std::vector<double> roi_max_;
  double local_range_fraction_{ 0.30 };
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try
  {
    auto node = std::make_shared<ReachabilitySampler>();
    node->run();
  }
  catch (const std::exception& exception)
  {
    RCLCPP_FATAL(rclcpp::get_logger("g1_reachability_sampler"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
