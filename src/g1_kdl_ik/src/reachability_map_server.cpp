#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <g1_kdl_ik/srv/query_reachability.hpp>

namespace fs = std::filesystem;

namespace
{
constexpr std::int32_t kVoxelBias = 1 << 20;

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
static_assert(sizeof(VoxelRecord) == 44);

std::uint64_t voxelKey(std::int32_t ix, std::int32_t iy, std::int32_t iz)
{
  const auto encode = [](std::int32_t value) -> std::uint64_t {
    const std::int64_t shifted = static_cast<std::int64_t>(value) + kVoxelBias;
    if (shifted < 0 || shifted >= (1 << 21))
      return std::numeric_limits<std::uint64_t>::max();
    return static_cast<std::uint64_t>(shifted);
  };
  const auto x = encode(ix);
  const auto y = encode(iy);
  const auto z = encode(iz);
  if (x == std::numeric_limits<std::uint64_t>::max() ||
      y == std::numeric_limits<std::uint64_t>::max() ||
      z == std::numeric_limits<std::uint64_t>::max())
    return std::numeric_limits<std::uint64_t>::max();
  return (x << 42) | (y << 21) | z;
}

double quaternionNorm(const geometry_msgs::msg::Quaternion& q)
{
  return std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
}
}  // namespace

class ReachabilityMapServer : public rclcpp::Node
{
public:
  ReachabilityMapServer() : Node("g1_reachability_map_server")
  {
    data_dir_ = declare_parameter<std::string>("data_dir");
    supplemental_data_dir_ = declare_parameter<std::string>("supplemental_data_dir", "");
    resolution_ = declare_parameter<double>("resolution", 0.005);
    validation_stride_ = declare_parameter<std::int64_t>("validation_stride", 101);
    visualization_stride_ = declare_parameter<std::int64_t>("visualization_stride", 8);
    if (resolution_ <= 0.0 || validation_stride_ <= 0 || visualization_stride_ <= 0)
      throw std::runtime_error("resolution and stride parameters must be positive");
  }

  void initialize()
  {
    loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(
        shared_from_this(), "robot_description");
    model_ = loader_->getModel();
    if (!model_)
      throw std::runtime_error("Failed to load robot model");

    left_ = loadMap("left_arm");
    right_ = loadMap("right_arm");
    if (!supplemental_data_dir_.empty())
    {
      mergeSupplement(left_);
      mergeSupplement(right_);
    }
    validateMap(left_);
    validateMap(right_);

    const auto qos = rclcpp::QoS(1).reliable().transient_local();
    left_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "reachability/left", qos);
    right_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "reachability/right", qos);
    left_publisher_->publish(makeCloud(left_, true));
    right_publisher_->publish(makeCloud(right_, false));

    service_ = create_service<g1_kdl_ik::srv::QueryReachability>(
        "query_reachability",
        std::bind(&ReachabilityMapServer::query, this,
                  std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(get_logger(),
                "Reachability server ready: left=%zu, right=%zu voxels; frame=torso_link",
                left_.records.size(), right_.records.size());
  }

private:
  struct ArmMap
  {
    std::string group_name;
    std::string tip_name;
    std::vector<VoxelRecord> records;
    std::unordered_map<std::uint64_t, std::uint32_t> index;
  };

  ArmMap loadMap(const std::string& group_name)
  {
    ArmMap map;
    map.group_name = group_name;
    map.tip_name = group_name == "left_arm" ? "left_tool0" : "right_tool0";
    const fs::path path = fs::path(data_dir_) / (group_name + "_voxels.bin");
    const auto bytes = fs::file_size(path);
    if (bytes == 0 || bytes % sizeof(VoxelRecord) != 0)
      throw std::runtime_error("Invalid 44-byte record file: " + path.string());
    map.records.resize(bytes / sizeof(VoxelRecord));
    std::ifstream input(path, std::ios::binary);
    input.read(reinterpret_cast<char*>(map.records.data()), static_cast<std::streamsize>(bytes));
    if (!input)
      throw std::runtime_error("Failed reading: " + path.string());

    map.index.reserve(map.records.size());
    std::uint64_t previous = 0;
    std::uint64_t hit_sum = 0;
    for (std::size_t i = 0; i < map.records.size(); ++i)
    {
      const auto& voxel = map.records[i];
      const std::uint64_t key = voxelKey(voxel.ix, voxel.iy, voxel.iz);
      if (key == std::numeric_limits<std::uint64_t>::max() || (i > 0 && key <= previous))
        throw std::runtime_error("Invalid or unsorted voxel key in " + path.string());
      previous = key;
      map.index.emplace(key, static_cast<std::uint32_t>(i));
      hit_sum += voxel.count;
    }
    RCLCPP_INFO(get_logger(), "%s loaded: %zu records, %lu total hits, %.1f MiB",
                group_name.c_str(), map.records.size(), hit_sum,
                static_cast<double>(bytes) / (1024.0 * 1024.0));
    return map;
  }

  void mergeSupplement(ArmMap& map)
  {
    const fs::path path = fs::path(supplemental_data_dir_) /
                          (map.group_name + "_voxels.bin");
    const auto bytes = fs::file_size(path);
    if (bytes == 0 || bytes % sizeof(VoxelRecord) != 0)
      throw std::runtime_error("Invalid supplemental record file: " + path.string());
    std::vector<VoxelRecord> supplement(bytes / sizeof(VoxelRecord));
    std::ifstream input(path, std::ios::binary);
    input.read(reinterpret_cast<char*>(supplement.data()), static_cast<std::streamsize>(bytes));
    if (!input)
      throw std::runtime_error("Failed reading supplement: " + path.string());

    std::size_t overlap = 0;
    std::size_t added = 0;
    for (const auto& voxel : supplement)
    {
      const auto key = voxelKey(voxel.ix, voxel.iy, voxel.iz);
      const auto existing = map.index.find(key);
      if (existing != map.index.end())
      {
        auto& count = map.records[existing->second].count;
        count = voxel.count > UINT32_MAX - count ? UINT32_MAX : count + voxel.count;
        ++overlap;
      }
      else
      {
        map.index.emplace(key, static_cast<std::uint32_t>(map.records.size()));
        map.records.push_back(voxel);
        ++added;
      }
    }
    std::sort(map.records.begin(), map.records.end(), [](const auto& a, const auto& b) {
      return voxelKey(a.ix, a.iy, a.iz) < voxelKey(b.ix, b.iy, b.iz);
    });
    map.index.clear();
    map.index.reserve(map.records.size());
    for (std::size_t i = 0; i < map.records.size(); ++i)
    {
      const auto& voxel = map.records[i];
      map.index.emplace(voxelKey(voxel.ix, voxel.iy, voxel.iz),
                        static_cast<std::uint32_t>(i));
    }
    RCLCPP_INFO(get_logger(),
                "%s supplement merged: %zu overlap, %zu new, %zu combined voxels",
                map.group_name.c_str(), overlap, added, map.records.size());
  }

  void validateMap(const ArmMap& map)
  {
    const auto* group = model_->getJointModelGroup(map.group_name);
    if (!group || group->getVariableCount() != 7 || !group->getSolverInstance())
      throw std::runtime_error("Invalid arm group or missing KDL solver: " + map.group_name);
    moveit::core::RobotState state(model_);
    state.setToDefaultValues();
    std::vector<double> q(7);
    std::size_t checked = 0;
    std::size_t failures = 0;
    std::size_t boundary_rounding = 0;
    for (std::size_t i = 0; i < map.records.size(); i += validation_stride_)
    {
      const auto& voxel = map.records[i];
      std::copy(std::begin(voxel.seed), std::end(voxel.seed), q.begin());
      state.setJointGroupPositions(group, q);
      state.updateLinkTransforms();
      const auto p = state.getGlobalLinkTransform(map.tip_name).translation();
      const auto ix = static_cast<std::int32_t>(std::floor(p.x() / resolution_));
      const auto iy = static_cast<std::int32_t>(std::floor(p.y() / resolution_));
      const auto iz = static_cast<std::int32_t>(std::floor(p.z() / resolution_));
      if (ix != voxel.ix || iy != voxel.iy || iz != voxel.iz)
      {
        const auto outsideDistance = [this](double value, std::int32_t cell) {
          const double lower = cell * resolution_;
          const double upper = (cell + 1) * resolution_;
          return value < lower ? lower - value : value > upper ? value - upper : 0.0;
        };
        const double outside = std::max({ outsideDistance(p.x(), voxel.ix),
                                          outsideDistance(p.y(), voxel.iy),
                                          outsideDistance(p.z(), voxel.iz) });
        if (outside <= 1e-5)
          ++boundary_rounding;
        else
          ++failures;
      }
      ++checked;
    }
    if (failures != 0)
      throw std::runtime_error(map.group_name + " FK validation failed for " +
                               std::to_string(failures) + "/" + std::to_string(checked));
    RCLCPP_INFO(get_logger(),
                "%s FK validation PASS: %zu/%zu checked, %zu float32 boundary roundings",
                map.group_name.c_str(), checked, map.records.size(), boundary_rounding);
  }

  sensor_msgs::msg::PointCloud2 makeCloud(const ArmMap& map, bool left)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = "torso_link";
    // This is a static map attached to torso_link. A zero timestamp asks TF/RViz
    // to use the latest available torso transform and avoids dropping the one-shot
    // transient-local cloud while robot_state_publisher is still starting.
    cloud.header.stamp.sec = 0;
    cloud.header.stamp.nanosec = 0;
    cloud.height = 1;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    // Explicit fields avoid the padding inserted by setPointCloud2FieldsByString,
    // keeping every visual point at 16 bytes instead of roughly 32 bytes.
    modifier.setPointCloud2Fields(
        4,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);
    const std::size_t point_count =
        (map.records.size() + visualization_stride_ - 1) / visualization_stride_;
    modifier.resize(point_count);
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<std::uint8_t> r(cloud, "r");
    sensor_msgs::PointCloud2Iterator<std::uint8_t> g(cloud, "g");
    sensor_msgs::PointCloud2Iterator<std::uint8_t> b(cloud, "b");
    for (std::size_t i = 0; i < map.records.size(); i += visualization_stride_)
    {
      const auto& voxel = map.records[i];
      *x = static_cast<float>((voxel.ix + 0.5) * resolution_);
      *y = static_cast<float>((voxel.iy + 0.5) * resolution_);
      *z = static_cast<float>((voxel.iz + 0.5) * resolution_);
      const auto density = static_cast<std::uint8_t>(
          std::min(255.0, 70.0 + 65.0 * std::log1p(static_cast<double>(voxel.count))));
      *r = left ? 30 : 255;
      *g = density;
      *b = left ? 255 : 30;
      ++x; ++y; ++z; ++r; ++g; ++b;
    }
    RCLCPP_INFO(get_logger(), "%s visualization cloud: %zu points, %.2f MiB (stride=%ld)",
                map.group_name.c_str(), point_count,
                static_cast<double>(cloud.data.size()) / (1024.0 * 1024.0),
                visualization_stride_);
    return cloud;
  }

  const VoxelRecord* nearestVoxel(const ArmMap& map, const geometry_msgs::msg::Point& target,
                                  double radius, double& distance) const
  {
    const auto cx = static_cast<std::int32_t>(std::floor(target.x / resolution_));
    const auto cy = static_cast<std::int32_t>(std::floor(target.y / resolution_));
    const auto cz = static_cast<std::int32_t>(std::floor(target.z / resolution_));
    const int cells = static_cast<int>(std::ceil(radius / resolution_));
    const VoxelRecord* best = nullptr;
    double best_squared = radius * radius;
    for (int dx = -cells; dx <= cells; ++dx)
      for (int dy = -cells; dy <= cells; ++dy)
        for (int dz = -cells; dz <= cells; ++dz)
        {
          const auto iterator = map.index.find(voxelKey(cx + dx, cy + dy, cz + dz));
          if (iterator == map.index.end())
            continue;
          const auto& voxel = map.records[iterator->second];
          const double vx = (voxel.ix + 0.5) * resolution_;
          const double vy = (voxel.iy + 0.5) * resolution_;
          const double vz = (voxel.iz + 0.5) * resolution_;
          const double squared = std::pow(vx - target.x, 2) +
                                 std::pow(vy - target.y, 2) +
                                 std::pow(vz - target.z, 2);
          if (squared <= best_squared)
          {
            best_squared = squared;
            best = &voxel;
          }
        }
    distance = best ? std::sqrt(best_squared) : std::numeric_limits<double>::infinity();
    return best;
  }

  void query(const std::shared_ptr<g1_kdl_ik::srv::QueryReachability::Request> request,
             std::shared_ptr<g1_kdl_ik::srv::QueryReachability::Response> response)
  {
    const bool left = request->arm == "left" || request->arm == "left_arm";
    const bool right = request->arm == "right" || request->arm == "right_arm";
    if (!left && !right)
    {
      response->message = "arm must be left/left_arm or right/right_arm";
      return;
    }
    const ArmMap& map = left ? left_ : right_;
    const double radius = request->search_radius > 0.0 ? request->search_radius : 0.02;
    const VoxelRecord* voxel = nearestVoxel(map, request->target.position, radius,
                                            response->nearest_distance);
    if (!voxel)
    {
      response->message = "No sampled position voxel within search radius";
      return;
    }

    response->voxel_found = true;
    response->nearest_voxel_center.x = (voxel->ix + 0.5) * resolution_;
    response->nearest_voxel_center.y = (voxel->iy + 0.5) * resolution_;
    response->nearest_voxel_center.z = (voxel->iz + 0.5) * resolution_;
    response->hit_count = voxel->count;
    const auto* group = model_->getJointModelGroup(map.group_name);
    response->seed.name = group->getVariableNames();
    response->seed.position.assign(std::begin(voxel->seed), std::end(voxel->seed));

    const double norm = quaternionNorm(request->target.orientation);
    if (norm < 1e-9)
    {
      response->message = "Position voxel found, but target quaternion is invalid (identity: w=1)";
      return;
    }
    geometry_msgs::msg::Pose target = request->target;
    target.orientation.x /= norm;
    target.orientation.y /= norm;
    target.orientation.z /= norm;
    target.orientation.w /= norm;
    moveit::core::RobotState state(model_);
    state.setToDefaultValues();
    std::vector<double> seed(response->seed.position.begin(), response->seed.position.end());
    state.setJointGroupPositions(group, seed);
    state.update();
    const double timeout = request->ik_timeout > 0.0 ? request->ik_timeout : 0.05;
    response->ik_success = state.setFromIK(group, target, map.tip_name, timeout);
    if (response->ik_success)
    {
      response->solution.name = group->getVariableNames();
      state.copyJointGroupPositions(group, response->solution.position);
      response->message = "Position voxel found and exact pose IK succeeded";
    }
    else
    {
      response->message = "Position voxel found, but exact pose IK failed for this orientation";
    }
  }

  std::string data_dir_;
  std::string supplemental_data_dir_;
  double resolution_{ 0.005 };
  std::int64_t validation_stride_{ 101 };
  std::int64_t visualization_stride_{ 8 };
  robot_model_loader::RobotModelLoaderPtr loader_;
  moveit::core::RobotModelPtr model_;
  ArmMap left_;
  ArmMap right_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr left_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr right_publisher_;
  rclcpp::Service<g1_kdl_ik::srv::QueryReachability>::SharedPtr service_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try
  {
    auto node = std::make_shared<ReachabilityMapServer>();
    node->initialize();
    rclcpp::spin(node);
  }
  catch (const std::exception& exception)
  {
    RCLCPP_FATAL(rclcpp::get_logger("g1_reachability_map_server"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
