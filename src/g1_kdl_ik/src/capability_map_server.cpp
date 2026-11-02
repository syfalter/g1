#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <memory>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>

#include <g1_kdl_ik/capability_map_v1.hpp>
#include <g1_kdl_ik/srv/query_capability.hpp>

namespace fs = std::filesystem;
using g1_kdl_ik::CapabilityRecordV1;

namespace
{
constexpr std::int32_t kVoxelBias = 1 << 20;
constexpr double kPi = 3.14159265358979323846;

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

Eigen::Isometry3d poseToEigen(const geometry_msgs::msg::Pose& pose)
{
  Eigen::Quaterniond quaternion(pose.orientation.w, pose.orientation.x,
                                pose.orientation.y, pose.orientation.z);
  quaternion.normalize();
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(
      pose.position.x, pose.position.y, pose.position.z);
  transform.linear() = quaternion.toRotationMatrix();
  return transform;
}

geometry_msgs::msg::Pose eigenToPose(const Eigen::Isometry3d& transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();
  const Eigen::Quaterniond quaternion(transform.rotation());
  pose.orientation.x = quaternion.x();
  pose.orientation.y = quaternion.y();
  pose.orientation.z = quaternion.z();
  pose.orientation.w = quaternion.w();
  return pose;
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

double rollAboutToolX(const Eigen::Matrix3d& rotation)
{
  const Eigen::Vector3d approach = rotation.col(0).normalized();
  const Eigen::Vector3d reference =
      std::abs(approach.z()) < 0.9 ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitY();
  const Eigen::Vector3d tangent_y = reference.cross(approach).normalized();
  const Eigen::Vector3d tangent_z = approach.cross(tangent_y).normalized();
  const Eigen::Vector3d tool_y = rotation.col(1).normalized();
  double roll = std::atan2(tool_y.dot(tangent_z), tool_y.dot(tangent_y));
  if (roll < 0.0)
    roll += 2.0 * kPi;
  return roll;
}

double circularDistance(double a, double b)
{
  double difference = std::fmod(std::abs(a - b), 2.0 * kPi);
  return std::min(difference, 2.0 * kPi - difference);
}
}  // namespace

class CapabilityMapServer : public rclcpp::Node
{
public:
  CapabilityMapServer() : Node("g1_capability_map_server")
  {
    data_dir_ = declare_parameter<std::string>("data_dir");
    resolution_ = declare_parameter<double>("resolution", 0.01);
    visualization_stride_ = declare_parameter<std::int64_t>("visualization_stride", 2);
    publish_debug_clouds_ = declare_parameter<bool>("publish_debug_clouds", false);
    default_search_radius_ = declare_parameter<double>("default_search_radius", 0.03);
    default_direction_tolerance_ = declare_parameter<double>("default_direction_tolerance", 0.45);
    default_roll_tolerance_ = declare_parameter<double>("default_roll_tolerance", 0.45);
    default_ik_timeout_ = declare_parameter<double>("default_ik_timeout", 0.05);
    default_max_seed_attempts_ = declare_parameter<std::int64_t>("default_max_seed_attempts", 8);
    publish_solution_joint_states_ =
        declare_parameter<bool>("publish_solution_joint_states", true);
    visualization_duration_ = declare_parameter<double>("visualization_duration", 2.5);
    max_visual_joint_speed_ = declare_parameter<double>("max_visual_joint_speed", 0.5);
    visualization_rate_ = declare_parameter<double>("visualization_rate", 50.0);
    left_tool_to_tcp_ = tcpTransform("left");
    right_tool_to_tcp_ = tcpTransform("right");
    direction_bins_ = fibonacciSphere(g1_kdl_ik::kDirectionBins);
    if (resolution_ <= 0.0 || visualization_stride_ <= 0 ||
        default_search_radius_ <= 0.0 || default_direction_tolerance_ <= 0.0 ||
        default_roll_tolerance_ <= 0.0 || default_ik_timeout_ <= 0.0 ||
        default_max_seed_attempts_ <= 0 || visualization_duration_ <= 0.0 ||
        max_visual_joint_speed_ <= 0.0 || visualization_rate_ <= 0.0)
      throw std::runtime_error("resolution and visualization_stride must be positive");
  }

  void initialize()
  {
    robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(
        shared_from_this(), "robot_description");
    robot_model_ = robot_model_loader_->getModel();
    if (!robot_model_)
      throw std::runtime_error("Failed to load G1 robot model");
    display_state_ = std::make_unique<moveit::core::RobotState>(robot_model_);
    display_state_->setToDefaultValues();
    display_state_->update();
    if (publish_solution_joint_states_)
    {
      joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
          "joint_states", rclcpp::QoS(10));
      visualization_timer_ = create_wall_timer(
          std::chrono::duration<double>(1.0 / visualization_rate_),
          std::bind(&CapabilityMapServer::publishVisualizationState, this));
    }
    left_ = load("left_arm");
    right_ = load("right_arm");
    const auto qos = rclcpp::QoS(1).reliable().transient_local();
    if (publish_debug_clouds_)
    {
      left_score_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
          "capability/left/score", qos);
      right_score_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
          "capability/right/score", qos);
      left_boundary_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
          "capability/left/boundary", qos);
      right_boundary_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
          "capability/right/boundary", qos);
      left_score_publisher_->publish(makeScoreCloud(left_.records, true));
      right_score_publisher_->publish(makeScoreCloud(right_.records, false));
      left_boundary_publisher_->publish(makeBoundaryCloud(left_.records, true));
      right_boundary_publisher_->publish(makeBoundaryCloud(right_.records, false));
    }
    const auto marker_qos = rclcpp::QoS(10).reliable().transient_local();
    query_axes_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "capability/query_axes", marker_qos);
    query_service_ = create_service<g1_kdl_ik::srv::QueryCapability>(
        "query_capability",
        std::bind(&CapabilityMapServer::handleQuery, this,
                  std::placeholders::_1, std::placeholders::_2));
    createInteractiveTargets();
    publishTargetPreview("left", currentTcp(true));
    publishTargetPreview("right", currentTcp(false));
    RCLCPP_INFO(get_logger(), "Capability v1 ready: left=%zu, right=%zu; frame=torso_link",
                left_.records.size(), right_.records.size());
    RCLCPP_INFO(get_logger(), "Debug capability clouds: %s",
                publish_debug_clouds_ ? "published" : "disabled");
    RCLCPP_INFO(get_logger(),
                "TCP query ready on /query_capability; left xyz=[%.3f %.3f %.3f], right xyz=[%.3f %.3f %.3f]",
                left_tool_to_tcp_.translation().x(), left_tool_to_tcp_.translation().y(),
                left_tool_to_tcp_.translation().z(), right_tool_to_tcp_.translation().x(),
                right_tool_to_tcp_.translation().y(), right_tool_to_tcp_.translation().z());
  }

private:
  struct ArmMap
  {
    std::string group_name;
    std::string tip_name;
    std::vector<CapabilityRecordV1> records;
    std::unordered_map<std::uint64_t, std::uint32_t> index;
  };

  struct OrientationStats
  {
    std::uint16_t matches{ 0 };
    double nearest_direction_error{ std::numeric_limits<double>::infinity() };
    double nearest_roll_error{ std::numeric_limits<double>::infinity() };
    double score{ std::numeric_limits<double>::infinity() };
  };

  struct Candidate
  {
    const CapabilityRecordV1* record{ nullptr };
    double distance{ 0.0 };
    OrientationStats orientation;
  };

  Eigen::Isometry3d tcpTransform(const std::string& side)
  {
    const double x = declare_parameter<double>(side + "_tcp_x", 0.0);
    const double y = declare_parameter<double>(side + "_tcp_y", 0.0);
    const double z = declare_parameter<double>(side + "_tcp_z", 0.0);
    const double roll = declare_parameter<double>(side + "_tcp_roll", 0.0);
    const double pitch = declare_parameter<double>(side + "_tcp_pitch", 0.0);
    const double yaw = declare_parameter<double>(side + "_tcp_yaw", 0.0);
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.translation() = Eigen::Vector3d(x, y, z);
    transform.linear() = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                          Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
    return transform;
  }

  ArmMap load(const std::string& group)
  {
    ArmMap map;
    map.group_name = group;
    map.tip_name = group == "left_arm" ? "left_tool0" : "right_tool0";
    const fs::path path = fs::path(data_dir_) / (group + "_capability_v1.bin");
    const auto bytes = fs::file_size(path);
    if (bytes == 0 || bytes % sizeof(CapabilityRecordV1) != 0)
      throw std::runtime_error("Invalid 90-byte capability file: " + path.string());
    map.records.resize(bytes / sizeof(CapabilityRecordV1));
    std::ifstream input(path, std::ios::binary);
    input.read(reinterpret_cast<char*>(map.records.data()), static_cast<std::streamsize>(bytes));
    if (!input)
      throw std::runtime_error("Failed reading: " + path.string());
    std::size_t boundary_count = 0;
    std::uint16_t max_orientations = 0;
    map.index.reserve(map.records.size());
    for (std::size_t i = 0; i < map.records.size(); ++i)
    {
      const auto& record = map.records[i];
      boundary_count += (record.flags & g1_kdl_ik::kBoundaryFlag) != 0;
      max_orientations = std::max(max_orientations, record.orientation_count);
      map.index.emplace(voxelKey(record.ix, record.iy, record.iz),
                        static_cast<std::uint32_t>(i));
    }
    RCLCPP_INFO(get_logger(), "%s loaded: %zu voxels, %zu boundary, max %u/%zu orientations",
                group.c_str(), map.records.size(), boundary_count, max_orientations,
                g1_kdl_ik::kOrientationBins);
    return map;
  }

  sensor_msgs::msg::PointCloud2 baseCloud(std::size_t size) const
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = "torso_link";
    cloud.header.stamp.sec = 0;
    cloud.header.stamp.nanosec = 0;
    cloud.height = 1;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(
        4,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(size);
    return cloud;
  }

  static void heatColor(double value, std::uint8_t& r, std::uint8_t& g, std::uint8_t& b)
  {
    value = std::clamp(value, 0.0, 1.0);
    if (value < 0.5)
    {
      r = 20;
      g = static_cast<std::uint8_t>(50.0 + 410.0 * value);
      b = static_cast<std::uint8_t>(255.0 - 310.0 * value);
    }
    else
    {
      r = static_cast<std::uint8_t>(20.0 + 470.0 * (value - 0.5));
      g = static_cast<std::uint8_t>(255.0 - 350.0 * (value - 0.5));
      b = 100;
    }
  }

  std::uint16_t percentileScale(const std::vector<CapabilityRecordV1>& records) const
  {
    std::vector<std::uint16_t> counts;
    counts.reserve(records.size());
    for (std::size_t i = 0; i < records.size(); i += visualization_stride_)
      counts.push_back(records[i].orientation_count);
    if (counts.empty())
      return 1;
    const auto index = static_cast<std::size_t>(0.98 * static_cast<double>(counts.size() - 1));
    std::nth_element(counts.begin(), counts.begin() + index, counts.end());
    return std::max<std::uint16_t>(1, counts[index]);
  }

  sensor_msgs::msg::PointCloud2 makeScoreCloud(
      const std::vector<CapabilityRecordV1>& records, bool left) const
  {
    const std::size_t size =
        (records.size() + visualization_stride_ - 1) / visualization_stride_;
    auto cloud = baseCloud(size);
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<std::uint8_t> r(cloud, "r");
    sensor_msgs::PointCloud2Iterator<std::uint8_t> g(cloud, "g");
    sensor_msgs::PointCloud2Iterator<std::uint8_t> b(cloud, "b");
    const std::uint16_t scale = percentileScale(records);
    for (std::size_t i = 0; i < records.size(); i += visualization_stride_)
    {
      const auto& record = records[i];
      *x = static_cast<float>((record.ix + 0.5) * resolution_);
      *y = static_cast<float>((record.iy + 0.5) * resolution_);
      *z = static_cast<float>((record.iz + 0.5) * resolution_);
      heatColor(static_cast<double>(record.orientation_count) / scale, *r, *g, *b);
      ++x; ++y; ++z; ++r; ++g; ++b;
    }
    RCLCPP_INFO(get_logger(), "%s score cloud: %zu points, color p98=%u bins",
                left ? "left" : "right", size, scale);
    return cloud;
  }

  sensor_msgs::msg::PointCloud2 makeBoundaryCloud(
      const std::vector<CapabilityRecordV1>& records, bool left) const
  {
    const std::size_t size = static_cast<std::size_t>(std::count_if(
        records.begin(), records.end(), [](const auto& record) {
          return (record.flags & g1_kdl_ik::kBoundaryFlag) != 0;
        }));
    auto cloud = baseCloud(size);
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<std::uint8_t> r(cloud, "r");
    sensor_msgs::PointCloud2Iterator<std::uint8_t> g(cloud, "g");
    sensor_msgs::PointCloud2Iterator<std::uint8_t> b(cloud, "b");
    for (const auto& record : records)
    {
      if ((record.flags & g1_kdl_ik::kBoundaryFlag) == 0)
        continue;
      *x = static_cast<float>((record.ix + 0.5) * resolution_);
      *y = static_cast<float>((record.iy + 0.5) * resolution_);
      *z = static_cast<float>((record.iz + 0.5) * resolution_);
      *r = 255;
      *g = left ? 255 : 70;
      *b = left ? 70 : 255;
      ++x; ++y; ++z; ++r; ++g; ++b;
    }
    RCLCPP_INFO(get_logger(), "%s optimized boundary cloud: %zu points",
                left ? "left" : "right", size);
    return cloud;
  }

  OrientationStats orientationStats(const CapabilityRecordV1& record,
                                    const Eigen::Matrix3d& target_rotation,
                                    double direction_tolerance,
                                    double roll_tolerance) const
  {
    OrientationStats result;
    const Eigen::Vector3d target_direction = target_rotation.col(0).normalized();
    const double target_roll = rollAboutToolX(target_rotation);
    for (std::size_t bin = 0; bin < g1_kdl_ik::kOrientationBins; ++bin)
    {
      if ((record.orientation_mask[bin / 8] & (1u << (bin % 8))) == 0)
        continue;
      const std::size_t direction_index = bin / g1_kdl_ik::kRollBins;
      const std::size_t roll_index = bin % g1_kdl_ik::kRollBins;
      const double direction_error = std::acos(std::clamp(
          target_direction.dot(direction_bins_[direction_index]), -1.0, 1.0));
      const double roll_center = (static_cast<double>(roll_index) + 0.5) *
                                 2.0 * kPi / g1_kdl_ik::kRollBins;
      const double roll_error = circularDistance(target_roll, roll_center);
      const double score = std::hypot(direction_error / direction_tolerance,
                                      roll_error / roll_tolerance);
      if (score < result.score)
      {
        result.score = score;
        result.nearest_direction_error = direction_error;
        result.nearest_roll_error = roll_error;
      }
      if (direction_error <= direction_tolerance && roll_error <= roll_tolerance)
        ++result.matches;
    }
    return result;
  }

  std::vector<Candidate> nearbyCandidates(const ArmMap& map,
                                          const Eigen::Isometry3d& target_tool,
                                          double radius, double direction_tolerance,
                                          double roll_tolerance) const
  {
    const Eigen::Vector3d target = target_tool.translation();
    const auto cx = static_cast<std::int32_t>(std::floor(target.x() / resolution_));
    const auto cy = static_cast<std::int32_t>(std::floor(target.y() / resolution_));
    const auto cz = static_cast<std::int32_t>(std::floor(target.z() / resolution_));
    const int cells = static_cast<int>(std::ceil(radius / resolution_));
    std::vector<Candidate> candidates;
    for (int dx = -cells; dx <= cells; ++dx)
      for (int dy = -cells; dy <= cells; ++dy)
        for (int dz = -cells; dz <= cells; ++dz)
        {
          const auto iterator = map.index.find(voxelKey(cx + dx, cy + dy, cz + dz));
          if (iterator == map.index.end())
            continue;
          const auto& record = map.records[iterator->second];
          const Eigen::Vector3d center((record.ix + 0.5) * resolution_,
                                       (record.iy + 0.5) * resolution_,
                                       (record.iz + 0.5) * resolution_);
          const double distance = (center - target).norm();
          if (distance > radius)
            continue;
          candidates.push_back(Candidate{
              &record, distance,
              orientationStats(record, target_tool.rotation(),
                               direction_tolerance, roll_tolerance) });
        }
    std::sort(candidates.begin(), candidates.end(), [](const auto& a, const auto& b) {
      const bool a_matches = a.orientation.matches != 0;
      const bool b_matches = b.orientation.matches != 0;
      if (a_matches != b_matches)
        return a_matches > b_matches;
      if (a_matches && std::abs(a.orientation.score - b.orientation.score) > 1e-12)
        return a.orientation.score < b.orientation.score;
      return a.distance < b.distance;
    });
    return candidates;
  }

  visualization_msgs::msg::Marker axisMarker(const std::string& name_space, int id,
                                               const Eigen::Isometry3d& pose,
                                               int axis, double alpha,
                                               bool delete_marker = false)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "torso_link";
    marker.header.stamp = now();
    marker.ns = name_space;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = delete_marker ? visualization_msgs::msg::Marker::DELETE :
                                    visualization_msgs::msg::Marker::ADD;
    Eigen::Isometry3d arrow_pose = pose;
    if (axis == 1)
      arrow_pose.linear() = pose.rotation() *
          Eigen::AngleAxisd(kPi / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    else if (axis == 2)
      arrow_pose.linear() = pose.rotation() *
          Eigen::AngleAxisd(-kPi / 2.0, Eigen::Vector3d::UnitY()).toRotationMatrix();
    marker.pose = eigenToPose(arrow_pose);
    const bool target = name_space.find("target_tcp") != std::string::npos;
    marker.scale.x = target ? 0.14 : 0.09;
    marker.scale.y = target ? 0.010 : 0.014;
    marker.scale.z = target ? 0.010 : 0.014;
    marker.color.a = alpha;
    marker.color.r = axis == 0 ? 1.0f : 0.05f;
    marker.color.g = axis == 1 ? 1.0f : 0.05f;
    marker.color.b = axis == 2 ? 1.0f : 0.05f;
    return marker;
  }

  visualization_msgs::msg::Marker originSphere(const std::string& name_space, int id,
                                                 const Eigen::Isometry3d& pose,
                                                 bool success, bool achieved,
                                                 bool delete_marker = false)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "torso_link";
    marker.header.stamp = now();
    marker.ns = name_space;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = delete_marker ? visualization_msgs::msg::Marker::DELETE :
                                    visualization_msgs::msg::Marker::ADD;
    marker.pose = eigenToPose(pose);
    marker.scale.x = marker.scale.y = marker.scale.z = achieved ? 0.045 : 0.055;
    marker.color.a = achieved ? 0.95f : 0.75f;
    if (achieved)
    {
      marker.color.r = 0.1f;
      marker.color.g = 1.0f;
      marker.color.b = 0.1f;
    }
    else
    {
      marker.color.r = success ? 1.0f : 1.0f;
      marker.color.g = success ? 0.85f : 0.05f;
      marker.color.b = 0.05f;
    }
    return marker;
  }

  void publishQueryAxes(const std::string& arm, const Eigen::Isometry3d& target_tcp,
                        const Eigen::Isometry3d* achieved_tcp)
  {
    visualization_msgs::msg::MarkerArray array;
    for (int axis = 0; axis < 3; ++axis)
      array.markers.push_back(axisMarker(arm + "/target_tcp", axis,
                                         target_tcp, axis, 0.45));
    array.markers.push_back(originSphere(arm + "/target_tcp", 6, target_tcp,
                                         achieved_tcp != nullptr, false));
    for (int axis = 0; axis < 3; ++axis)
      array.markers.push_back(axisMarker(arm + "/achieved_tcp", axis + 3,
                                         achieved_tcp ? *achieved_tcp : target_tcp,
                                         axis, 1.0, achieved_tcp == nullptr));
    array.markers.push_back(originSphere(arm + "/achieved_tcp", 7,
                                         achieved_tcp ? *achieved_tcp : target_tcp,
                                         true, true, achieved_tcp == nullptr));
    query_axes_publisher_->publish(array);
  }

  void publishTargetPreview(const std::string& arm,
                            const Eigen::Isometry3d& target_tcp)
  {
    visualization_msgs::msg::MarkerArray array;
    for (int axis = 0; axis < 3; ++axis)
      array.markers.push_back(axisMarker(arm + "/target_tcp", axis,
                                         target_tcp, axis, 0.55));
    array.markers.push_back(originSphere(arm + "/target_tcp", 6,
                                         target_tcp, true, false));
    for (int axis = 0; axis < 3; ++axis)
      array.markers.push_back(axisMarker(arm + "/achieved_tcp", axis + 3,
                                         target_tcp, axis, 1.0, true));
    array.markers.push_back(originSphere(arm + "/achieved_tcp", 7,
                                         target_tcp, true, true, true));
    query_axes_publisher_->publish(array);
  }

  visualization_msgs::msg::InteractiveMarkerControl axisControl(
      const std::string& name, std::uint8_t mode,
      double x, double y, double z) const
  {
    visualization_msgs::msg::InteractiveMarkerControl control;
    control.name = name;
    control.orientation.w = 1.0;
    control.orientation.x = x;
    control.orientation.y = y;
    control.orientation.z = z;
    control.orientation_mode =
        visualization_msgs::msg::InteractiveMarkerControl::FIXED;
    control.interaction_mode = mode;
    return control;
  }

  visualization_msgs::msg::InteractiveMarker makeInteractiveTarget(
      const std::string& side, const Eigen::Isometry3d& pose) const
  {
    visualization_msgs::msg::InteractiveMarker marker;
    marker.header.frame_id = "torso_link";
    marker.name = side + "_target_tcp";
    marker.description = side + " TCP target (right-click the yellow sphere)";
    marker.pose = eigenToPose(pose);
    marker.scale = 0.22;

    visualization_msgs::msg::InteractiveMarkerControl visual;
    visual.name = "target_handle";
    visual.always_visible = true;
    visual.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MENU;
    visualization_msgs::msg::Marker sphere;
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.070;
    sphere.color.r = 1.0f;
    sphere.color.g = 0.85f;
    sphere.color.b = 0.05f;
    sphere.color.a = 0.9f;
    visual.markers.push_back(sphere);
    marker.controls.push_back(visual);

    using Control = visualization_msgs::msg::InteractiveMarkerControl;
    marker.controls.push_back(axisControl("rotate_x", Control::ROTATE_AXIS, 1, 0, 0));
    marker.controls.push_back(axisControl("move_x", Control::MOVE_AXIS, 1, 0, 0));
    marker.controls.push_back(axisControl("rotate_z", Control::ROTATE_AXIS, 0, 1, 0));
    marker.controls.push_back(axisControl("move_z", Control::MOVE_AXIS, 0, 1, 0));
    marker.controls.push_back(axisControl("rotate_y", Control::ROTATE_AXIS, 0, 0, 1));
    marker.controls.push_back(axisControl("move_y", Control::MOVE_AXIS, 0, 0, 1));
    return marker;
  }

  void setInteractiveStatus(const std::string& marker_name, bool valid)
  {
    if (!interactive_server_)
      return;
    visualization_msgs::msg::InteractiveMarker marker;
    if (!interactive_server_->get(marker_name, marker) || marker.controls.empty() ||
        marker.controls.front().markers.empty())
      return;
    auto& color = marker.controls.front().markers.front().color;
    color.r = 1.0f;
    color.g = valid ? 0.85f : 0.05f;
    color.b = 0.05f;
    interactive_server_->insert(marker);
    interactive_menu_->reApply(*interactive_server_);
    interactive_server_->applyChanges();
  }

  Eigen::Isometry3d currentTcp(bool left)
  {
    updateVisualizationState();
    const std::string tip = left ? "left_tool0" : "right_tool0";
    const Eigen::Isometry3d& offset = left ? left_tool_to_tcp_ : right_tool_to_tcp_;
    return display_state_->getGlobalLinkTransform(tip) * offset;
  }

  void interactivePoseFeedback(
      const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
  {
    if (feedback->event_type !=
        visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE)
      return;
    interactive_server_->setPose(feedback->marker_name, feedback->pose);
    interactive_server_->applyChanges();
    const std::string arm = feedback->marker_name.rfind("left", 0) == 0 ?
        "left" : "right";
    publishTargetPreview(arm, poseToEigen(feedback->pose));
  }

  void interactiveQuery(
      const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
  {
    auto request = std::make_shared<g1_kdl_ik::srv::QueryCapability::Request>();
    auto response = std::make_shared<g1_kdl_ik::srv::QueryCapability::Response>();
    request->arm = feedback->marker_name.rfind("left", 0) == 0 ? "left" : "right";
    request->target_tcp = feedback->pose;
    handleQuery(request, response);
    RCLCPP_INFO(get_logger(), "Interactive %s: %s", request->arm.c_str(),
                response->message.c_str());
  }

  void interactiveReset(
      const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
  {
    const bool left = feedback->marker_name.rfind("left", 0) == 0;
    const Eigen::Isometry3d pose = currentTcp(left);
    interactive_server_->setPose(feedback->marker_name, eigenToPose(pose));
    interactive_server_->applyChanges();
    setInteractiveStatus(feedback->marker_name, true);
    publishTargetPreview(left ? "left" : "right", pose);
  }

  void createInteractiveTargets()
  {
    interactive_server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
        "capability_tcp_targets", shared_from_this());
    interactive_menu_ = std::make_unique<interactive_markers::MenuHandler>();
    interactive_menu_->insert(
        "Query and solve",
        std::bind(&CapabilityMapServer::interactiveQuery, this, std::placeholders::_1));
    interactive_menu_->insert(
        "Reset to current TCP",
        std::bind(&CapabilityMapServer::interactiveReset, this, std::placeholders::_1));
    for (const bool left : { true, false })
    {
      const std::string side = left ? "left" : "right";
      const auto marker = makeInteractiveTarget(side, currentTcp(left));
      interactive_server_->insert(
          marker, std::bind(&CapabilityMapServer::interactivePoseFeedback,
                            this, std::placeholders::_1));
      interactive_menu_->apply(*interactive_server_, marker.name);
    }
    interactive_server_->applyChanges();
    RCLCPP_INFO(get_logger(),
                "RViz interactive TCP targets ready; drag handles, then right-click the yellow sphere");
  }

  void setSeedResponse(const ArmMap& map, const CapabilityRecordV1& record,
                       sensor_msgs::msg::JointState& seed) const
  {
    const auto* group = robot_model_->getJointModelGroup(map.group_name);
    seed.name = group->getVariableNames();
    seed.position.assign(std::begin(record.seed), std::end(record.seed));
  }

  static double quintic(double value)
  {
    value = std::clamp(value, 0.0, 1.0);
    return value * value * value * (10.0 + value * (-15.0 + 6.0 * value));
  }

  void updateVisualizationState()
  {
    if (!trajectory_active_)
      return;
    const double elapsed = (now() - trajectory_start_time_).seconds();
    const double progress = std::clamp(elapsed / trajectory_duration_, 0.0, 1.0);
    const double blend = quintic(progress);
    for (std::size_t i = 0; i < trajectory_goal_.size(); ++i)
      display_state_->setVariablePosition(
          static_cast<int>(i), trajectory_start_[i] +
          blend * (trajectory_goal_[i] - trajectory_start_[i]));
    display_state_->update();
    if (progress >= 1.0)
      trajectory_active_ = false;
  }

  void publishVisualizationState()
  {
    updateVisualizationState();
    sensor_msgs::msg::JointState message;
    message.header.stamp = now();
    message.name = robot_model_->getVariableNames();
    message.position.assign(display_state_->getVariablePositions(),
                            display_state_->getVariablePositions() + message.name.size());
    joint_state_publisher_->publish(message);
  }

  void startVisualizationTrajectory(const moveit::core::RobotState& solution,
                                    const moveit::core::JointModelGroup* group)
  {
    if (!publish_solution_joint_states_)
      return;
    updateVisualizationState();
    moveit::core::RobotState goal(*display_state_);
    std::vector<double> arm_positions;
    solution.copyJointGroupPositions(group, arm_positions);
    goal.setJointGroupPositions(group, arm_positions);
    goal.update();
    trajectory_start_.assign(display_state_->getVariablePositions(),
                             display_state_->getVariablePositions() +
                                 robot_model_->getVariableCount());
    trajectory_goal_.assign(goal.getVariablePositions(),
                            goal.getVariablePositions() + robot_model_->getVariableCount());
    double max_delta = 0.0;
    for (std::size_t i = 0; i < trajectory_goal_.size(); ++i)
      max_delta = std::max(max_delta,
                           std::abs(trajectory_goal_[i] - trajectory_start_[i]));
    // Quintic interpolation has a peak normalized speed of 1.875/duration.
    trajectory_duration_ = std::max(
        visualization_duration_, 1.875 * max_delta / max_visual_joint_speed_);
    trajectory_start_time_ = now();
    trajectory_active_ = true;
  }

  void handleQuery(
      const std::shared_ptr<g1_kdl_ik::srv::QueryCapability::Request> request,
      std::shared_ptr<g1_kdl_ik::srv::QueryCapability::Response> response)
  {
    const bool left = request->arm == "left" || request->arm == "left_arm";
    const bool right = request->arm == "right" || request->arm == "right_arm";
    const std::string marker_name = left ? "left_target_tcp" :
                                    right ? "right_target_tcp" : "";
    if (!marker_name.empty() && quaternionNorm(request->target_tcp.orientation) >= 1e-9 &&
        interactive_server_)
    {
      interactive_server_->setPose(marker_name, request->target_tcp);
      interactive_server_->applyChanges();
      setInteractiveStatus(marker_name, true);
    }
    executeQuery(request, response);
    if (!marker_name.empty())
      setInteractiveStatus(marker_name, response->ik_success);
  }

  void executeQuery(
      const std::shared_ptr<g1_kdl_ik::srv::QueryCapability::Request> request,
      std::shared_ptr<g1_kdl_ik::srv::QueryCapability::Response> response)
  {
    const bool left = request->arm == "left" || request->arm == "left_arm";
    const bool right = request->arm == "right" || request->arm == "right_arm";
    if (!left && !right)
    {
      response->message = "arm must be left/left_arm or right/right_arm";
      return;
    }
    if (quaternionNorm(request->target_tcp.orientation) < 1e-9)
    {
      response->message = "target_tcp quaternion is invalid (identity: w=1)";
      return;
    }

    const ArmMap& map = left ? left_ : right_;
    const Eigen::Isometry3d& tool_to_tcp = left ? left_tool_to_tcp_ : right_tool_to_tcp_;
    const Eigen::Isometry3d target_tcp = poseToEigen(request->target_tcp);
    const Eigen::Isometry3d target_tool = target_tcp * tool_to_tcp.inverse();
    response->target_tool0 = eigenToPose(target_tool);
    const double radius = request->search_radius > 0.0 ?
        request->search_radius : default_search_radius_;
    const double direction_tolerance = request->direction_tolerance > 0.0 ?
        request->direction_tolerance : default_direction_tolerance_;
    const double roll_tolerance = request->roll_tolerance > 0.0 ?
        request->roll_tolerance : default_roll_tolerance_;
    const double timeout = request->ik_timeout > 0.0 ?
        request->ik_timeout : default_ik_timeout_;
    const std::size_t max_attempts = request->max_seed_attempts > 0 ?
        request->max_seed_attempts : static_cast<std::size_t>(default_max_seed_attempts_);

    auto candidates = nearbyCandidates(map, target_tool, radius,
                                       direction_tolerance, roll_tolerance);
    if (candidates.empty())
    {
      response->message = "No sampled tool0 position voxel inside search radius";
      publishQueryAxes(request->arm, target_tcp, nullptr);
      return;
    }

    response->position_found = true;
    const Candidate& selected = candidates.front();
    response->orientation_match = selected.orientation.matches != 0;
    response->voxel_orientation_count = selected.record->orientation_count;
    response->matching_orientation_count = selected.orientation.matches;
    response->nearest_voxel_center.x = (selected.record->ix + 0.5) * resolution_;
    response->nearest_voxel_center.y = (selected.record->iy + 0.5) * resolution_;
    response->nearest_voxel_center.z = (selected.record->iz + 0.5) * resolution_;
    response->nearest_distance = selected.distance;
    response->nearest_direction_bin_error = selected.orientation.nearest_direction_error;
    response->nearest_roll_bin_error = selected.orientation.nearest_roll_error;
    setSeedResponse(map, *selected.record, response->seed);

    const auto* group = robot_model_->getJointModelGroup(map.group_name);
    if (!group || !group->getSolverInstance())
    {
      response->message = "KDL solver is not available for " + map.group_name;
      publishQueryAxes(request->arm, target_tcp, nullptr);
      return;
    }

    moveit::core::RobotState solved_state(robot_model_);
    std::vector<double> used_seed;
    for (const auto& candidate : candidates)
    {
      if (response->ik_attempts >= max_attempts)
        break;
      std::vector<double> seed(std::begin(candidate.record->seed),
                               std::end(candidate.record->seed));
      solved_state.setToDefaultValues();
      solved_state.setJointGroupPositions(group, seed);
      solved_state.enforceBounds(group);
      solved_state.update();
      ++response->ik_attempts;
      if (solved_state.setFromIK(group, response->target_tool0, map.tip_name, timeout))
      {
        response->ik_success = true;
        used_seed = seed;
        break;
      }
    }

    if (!response->ik_success)
    {
      response->message = response->orientation_match ?
          "Capability bin matched, but KDL failed from all candidate seeds" :
          "Position matched but no orientation bin matched; exact KDL also failed";
      publishQueryAxes(request->arm, target_tcp, nullptr);
      return;
    }

    response->seed.name = group->getVariableNames();
    response->seed.position = used_seed;
    response->solution.name = group->getVariableNames();
    solved_state.copyJointGroupPositions(group, response->solution.position);
    solved_state.update();
    const Eigen::Isometry3d achieved_tool =
        solved_state.getGlobalLinkTransform(map.tip_name);
    const Eigen::Isometry3d achieved_tcp = achieved_tool * tool_to_tcp;
    response->achieved_tcp = eigenToPose(achieved_tcp);
    response->position_error = (achieved_tcp.translation() - target_tcp.translation()).norm();
    response->rotation_error = Eigen::AngleAxisd(
        target_tcp.rotation().transpose() * achieved_tcp.rotation()).angle();
    startVisualizationTrajectory(solved_state, group);
    response->message = response->orientation_match ?
        "Capability orientation matched and exact TCP IK succeeded; RViz trajectory started" :
        "Map had no matching orientation bin, but exact TCP IK succeeded; RViz trajectory started";
    publishQueryAxes(request->arm, target_tcp, &achieved_tcp);
    RCLCPP_INFO(get_logger(),
                "%s TCP query: map_orientation=%s, IK attempts=%u, errors=%.3g m/%.3g rad",
                map.group_name.c_str(), response->orientation_match ? "match" : "miss",
                response->ik_attempts, response->position_error, response->rotation_error);
  }

  std::string data_dir_;
  double resolution_{ 0.01 };
  std::int64_t visualization_stride_{ 2 };
  bool publish_debug_clouds_{ false };
  double default_search_radius_{ 0.03 };
  double default_direction_tolerance_{ 0.45 };
  double default_roll_tolerance_{ 0.45 };
  double default_ik_timeout_{ 0.05 };
  std::int64_t default_max_seed_attempts_{ 8 };
  bool publish_solution_joint_states_{ true };
  double visualization_duration_{ 2.5 };
  double max_visual_joint_speed_{ 0.5 };
  double visualization_rate_{ 50.0 };
  Eigen::Isometry3d left_tool_to_tcp_{ Eigen::Isometry3d::Identity() };
  Eigen::Isometry3d right_tool_to_tcp_{ Eigen::Isometry3d::Identity() };
  std::vector<Eigen::Vector3d> direction_bins_;
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  moveit::core::RobotModelPtr robot_model_;
  std::unique_ptr<moveit::core::RobotState> display_state_;
  std::vector<double> trajectory_start_;
  std::vector<double> trajectory_goal_;
  rclcpp::Time trajectory_start_time_;
  double trajectory_duration_{ 0.0 };
  bool trajectory_active_{ false };
  ArmMap left_;
  ArmMap right_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr left_score_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr right_score_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr left_boundary_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr right_boundary_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr query_axes_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::TimerBase::SharedPtr visualization_timer_;
  rclcpp::Service<g1_kdl_ik::srv::QueryCapability>::SharedPtr query_service_;
  std::unique_ptr<interactive_markers::InteractiveMarkerServer> interactive_server_;
  std::unique_ptr<interactive_markers::MenuHandler> interactive_menu_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try
  {
    auto node = std::make_shared<CapabilityMapServer>();
    node->initialize();
    rclcpp::spin(node);
  }
  catch (const std::exception& exception)
  {
    RCLCPP_FATAL(rclcpp::get_logger("g1_capability_map_server"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
