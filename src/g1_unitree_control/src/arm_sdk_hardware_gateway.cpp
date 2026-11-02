#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <map>
#include <mutex>
#include <regex>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/empty.hpp>
#include "../include/g1_unitree_control/task_admittance.hpp"
#include "../include/g1_unitree_control/task_waist_integral.hpp"
#include "../include/g1_unitree_control/task_tracking_scale.hpp"
#include "../include/g1_unitree_control/task_waist_trajectory.hpp"
#include "../include/g1_unitree_control/task_waist_rate.hpp"
#include "g1_unitree_control/srv/resume_task_hold.hpp"
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <unitree_api/msg/request.hpp>
#include <unitree_api/msg/response.hpp>
#include <unitree_hg/msg/low_cmd.hpp>
#include <unitree_hg/msg/low_state.hpp>

#include <Eigen/Core>
#include <kdl/chaindynparam.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/rigidbodyinertia.hpp>
#include <kdl/segment.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

using namespace std::chrono_literals;

namespace
{
constexpr std::uint16_t kL2BMask = 0x0220;
constexpr std::size_t kWeightIndex = 29;
constexpr char kRequiredAuthorization[] = "I_UNDERSTAND_ARM_SDK_OUTPUT";
constexpr std::int64_t kServiceListApiId = 1003;
// No stage tuning ceiling; reject gains that would overflow LowCmd float fields.
bool valid_stage_gain(double value)
{
  return std::isfinite(value) && value>=0 && value<=std::numeric_limits<float>::max();
}
// The installed Unitree URDF rates shoulder, elbow, and wrist-roll joints at
// 25 Nm, while wrist pitch/yaw are rated at 5 Nm. Keep a 20% continuous margin
// and allow the URDF effort limit only as a short peak.
constexpr std::array<double, 7> kArmContinuousTorqueLimits{20.0, 20.0, 20.0, 20.0,
                                                            20.0, 4.0, 4.0};
constexpr std::array<double, 7> kArmPeakTorqueLimits{25.0, 25.0, 25.0, 25.0,
                                                      25.0, 5.0, 5.0};

double quintic(double value)
{
  const double x = std::clamp(value, 0.0, 1.0);
  return x * x * x * (10.0 + x * (-15.0 + 6.0 * x));
}

class ArmGravityModel
{
public:
  bool initialize(const std::string& urdf_path, const std::string& fit_path,
                  const std::string& side,
                  std::string& reason)
  {
    KDL::Tree tree;
    if (!kdl_parser::treeFromFile(urdf_path, tree) ||
        !tree.getChain("torso_link", side + "_wrist_yaw_link", chain_) ||
        chain_.getNrOfJoints() != 7)
    {
      reason = "failed to load 7-DoF KDL " + side + "-arm chain from " + urdf_path;
      return false;
    }
    std::ifstream input(fit_path);
    if (!input)
    {
      reason = "cannot open fitted payload YAML: " + fit_path;
      return false;
    }
    double mass = std::numeric_limits<double>::quiet_NaN();
    Eigen::Vector3d first_moment = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
    std::string line, payload_frame, base_link;
    while (std::getline(input, line))
    {
      if (line.rfind("payload_frame:", 0) == 0)
        std::stringstream(line.substr(line.find(':') + 1)) >> payload_frame;
      else if (line.rfind("base_link:", 0) == 0)
        std::stringstream(line.substr(line.find(':') + 1)) >> base_link;
      else if (line.rfind("mass_kg:", 0) == 0)
      {
        std::stringstream stream(line.substr(line.find(':') + 1));
        stream >> mass;
      }
      else if (line.rfind("first_moment_kg_m:", 0) == 0)
      {
        const auto begin = line.find('[');
        const auto end = line.find(']', begin);
        if (begin == std::string::npos || end == std::string::npos)
          continue;
        std::stringstream stream(line.substr(begin + 1, end - begin - 1));
        for (std::size_t i = 0; i < 3; ++i)
        {
          if (!(stream >> first_moment(static_cast<Eigen::Index>(i))))
            break;
          if (i < 2)
          {
            char comma{};
            stream >> comma;
          }
        }
      }
    }
    if (base_link != "torso_link" || payload_frame != side + "_wrist_yaw_link")
    {
      reason = "payload fit frame does not match selected arm " + side;
      return false;
    }
    if (!std::isfinite(mass) || mass <= 0.01 || !first_moment.allFinite())
    {
      reason = "fitted payload YAML has invalid mass or first moment";
      return false;
    }
    payload_parameters_ << mass, first_moment.x(), first_moment.y(), first_moment.z();
    return true;
  }

  bool baseline(const std::array<double, 7>& q, const std::array<double, 3>& rpy,
                std::array<double, 7>& output) const
  {
    return gravity(chain_, q, rpy, output);
  }

  bool identified(const std::array<double, 7>& q, const std::array<double, 3>& rpy,
                  std::array<double, 7>& output) const
  {
    std::array<double, 7> base{}, origin{}, x{}, y{}, z{};
    if (!gravity(chain_, q, rpy, base) ||
        !gravity(with_payload(KDL::Vector::Zero()), q, rpy, origin) ||
        !gravity(with_payload(KDL::Vector(1.0, 0.0, 0.0)), q, rpy, x) ||
        !gravity(with_payload(KDL::Vector(0.0, 1.0, 0.0)), q, rpy, y) ||
        !gravity(with_payload(KDL::Vector(0.0, 0.0, 1.0)), q, rpy, z))
      return false;
    for (std::size_t i = 0; i < 7; ++i)
      output[i] = base[i] + (origin[i] - base[i]) * payload_parameters_[0] +
        (x[i] - origin[i]) * payload_parameters_[1] +
        (y[i] - origin[i]) * payload_parameters_[2] +
        (z[i] - origin[i]) * payload_parameters_[3];
    return true;
  }

private:
  KDL::Chain with_payload(const KDL::Vector& com) const
  {
    KDL::Chain result = chain_;
    result.addSegment(KDL::Segment(
      "identified_payload", KDL::Joint(KDL::Joint::Fixed), KDL::Frame::Identity(),
      KDL::RigidBodyInertia(1.0, com, KDL::RotationalInertia::Zero())));
    return result;
  }

  bool gravity(const KDL::Chain& chain, const std::array<double, 7>& q_values,
               const std::array<double, 3>& rpy,
               std::array<double, 7>& output) const
  {
    KDL::JntArray q(chain.getNrOfJoints()), torque(chain.getNrOfJoints());
    for (std::size_t i = 0; i < 7; ++i)
      q(i) = q_values[i];
    const auto world_from_torso = KDL::Rotation::RPY(rpy[0], rpy[1], rpy[2]);
    KDL::ChainDynParam dynamics(
      chain, world_from_torso.Inverse(KDL::Vector(0.0, 0.0, -9.81)));
    if (dynamics.JntToGravity(q, torque) != 0)
      return false;
    for (std::size_t i = 0; i < 7; ++i)
      output[i] = torque(i);
    return true;
  }

  KDL::Chain chain_;
  Eigen::Vector4d payload_parameters_{};
};

struct JointSpec
{
  const char* name;
  std::size_t motor_index;
  double lower;
  double upper;
  bool left;
};

constexpr std::array<JointSpec, 14> kArmJoints{ {
  { "left_shoulder_pitch_joint", 15, -3.0892, 2.6704, true },
  { "left_shoulder_roll_joint", 16, -1.5882, 2.2515, true },
  { "left_shoulder_yaw_joint", 17, -2.6180, 2.6180, true },
  { "left_elbow_joint", 18, -1.0472, 2.0944, true },
  { "left_wrist_roll_joint", 19, -1.972222054, 1.972222054, true },
  { "left_wrist_pitch_joint", 20, -1.614429558, 1.614429558, true },
  { "left_wrist_yaw_joint", 21, -1.614429558, 1.614429558, true },
  { "right_shoulder_pitch_joint", 22, -3.0892, 2.6704, false },
  { "right_shoulder_roll_joint", 23, -2.2515, 1.5882, false },
  { "right_shoulder_yaw_joint", 24, -2.6180, 2.6180, false },
  { "right_elbow_joint", 25, -1.0472, 2.0944, false },
  { "right_wrist_roll_joint", 26, -1.972222054, 1.972222054, false },
  { "right_wrist_pitch_joint", 27, -1.614429558, 1.614429558, false },
  { "right_wrist_yaw_joint", 28, -1.614429558, 1.614429558, false },
} };

constexpr JointSpec kTaskWaistYaw{"waist_yaw_joint", 12, -0.7853981633974483, 0.7853981633974483, false};

const JointSpec& trajectory_joint(std::size_t index)
{
  return index == kArmJoints.size() ? kTaskWaistYaw : kArmJoints.at(index);
}

diagnostic_msgs::msg::KeyValue diagnostic_value(
  const std::string& key, const std::string& value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}

double point_time(const trajectory_msgs::msg::JointTrajectoryPoint& point)
{
  return rclcpp::Duration(point.time_from_start).seconds();
}

struct TrajectorySample
{
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> accelerations;
};

const JointSpec* find_joint(const std::string& name, std::size_t& arm_index)
{
  if (name == kTaskWaistYaw.name) {
    arm_index = kArmJoints.size();
    return &kTaskWaistYaw;
  }
  for (std::size_t i = 0; i < kArmJoints.size(); ++i)
  {
    if (name == kArmJoints[i].name)
    {
      arm_index = i;
      return &kArmJoints[i];
    }
  }
  return nullptr;
}
}  // namespace

class ArmSdkHardwareGateway : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using FollowGoalHandle = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;
  using ResumeTaskHold = g1_unitree_control::srv::ResumeTaskHold;

  ArmSdkHardwareGateway() : Node("g1_arm_sdk_hardware_gateway")
  {
    active_arm_ = declare_parameter<std::string>("active_arm", "right");
    if (active_arm_ != "right" && active_arm_ != "left")
      throw std::invalid_argument("active_arm must be left or right");
    task_application_enabled_ = declare_parameter("task_application_enabled", false);
    task_waist_enabled_ = declare_parameter("task_waist_enabled", false);
    task_dual_prepare_enabled_ = declare_parameter("task_dual_prepare_enabled", false);
    task_client_watchdog_s_ = declare_parameter("task_client_watchdog_s", 0.0);
    if (!std::isfinite(task_client_watchdog_s_) || task_client_watchdog_s_ < 0.0)
      throw std::invalid_argument("task_client_watchdog_s must be finite and nonnegative");
    for (std::size_t i = 0; i < 7; ++i)
      active_motor_indices_[i] = (active_arm_ == "left" ? 15 : 22) + i;
    state_topic_ = declare_parameter<std::string>("state_topic", "/lowstate");
    trajectory_topic_ = declare_parameter<std::string>(
      "trajectory_topic", "/g1/arm_gateway/trajectory");
    command_topic_ = declare_parameter<std::string>("command_topic", "/arm_sdk");
    preview_topic_ = declare_parameter<std::string>(
      "preview_topic", "/g1/arm_gateway/command_preview");
    diagnostics_topic_ = declare_parameter<std::string>(
      "diagnostics_topic", "/g1/arm_gateway/diagnostics");
    trajectory_log_directory_ = declare_parameter<std::string>(
      "trajectory_log_directory", "/home/syf/g1_upper_body_ws/logs/endpoint_motion");

    hardware_output_requested_ = declare_parameter("hardware_output_enabled", false);
    const auto authorization = declare_parameter<std::string>("hardware_authorization", "");
    control_rate_hz_ = std::clamp(declare_parameter("control_rate_hz", 100.0), 20.0, 200.0);
    state_timeout_s_ = std::clamp(declare_parameter("state_timeout_s", 0.10), 0.02, 0.50);
    start_tolerance_rad_ = std::clamp(
      declare_parameter("start_tolerance_rad", 0.05), 0.001, 0.25);
    rcl_interfaces::msg::ParameterDescriptor motion_startup_only;
    motion_startup_only.read_only = true;
    motion_startup_only.description = "Command and measured motion limits are startup-only";
    max_velocity_rad_s_ = std::clamp(
      declare_parameter("max_velocity_rad_s", 0.60, motion_startup_only), 0.005, 1.00);
    task_waist_velocity_limit_rad_s_ = std::clamp(
      declare_parameter("task_waist_velocity_limit_rad_s", 0.40, motion_startup_only), 0.005, 0.40);
    max_acceleration_rad_s2_ = std::clamp(
      declare_parameter("max_acceleration_rad_s2", 1.50), 0.01, 3.00);
    max_command_step_rad_ = std::clamp(
      declare_parameter("max_command_step_rad", 0.006), 0.0001, 0.02);
    waist_command_rate_hz_ = g1_task::waist_command_rate(control_rate_hz_,
      declare_parameter("waist_command_rate_hz", 50.0, motion_startup_only));
    task_waist_velocity_limit_rad_s_ = g1_task::waist_velocity_limit(
      task_waist_velocity_limit_rad_s_, max_command_step_rad_, waist_command_rate_hz_);
    joint_limit_margin_rad_ = std::clamp(
      declare_parameter("joint_limit_margin_rad", 0.02), 0.0, 0.20);
    tracking_warning_rad_ = std::clamp(
      declare_parameter("tracking_warning_rad", 0.06), 0.005, 0.30);
    tracking_pause_rad_ = std::clamp(
      declare_parameter("tracking_pause_rad", 0.10), tracking_warning_rad_, 0.50);
    tracking_hard_limit_rad_ = std::clamp(
      declare_parameter("tracking_hard_limit_rad", 0.16), tracking_pause_rad_, 0.75);
    max_measured_velocity_rad_s_ = std::clamp(
      declare_parameter("max_measured_velocity_rad_s", 0.50, motion_startup_only), 0.05, 5.0);
    measured_velocity_hard_limit_rad_s_ = std::clamp(
      declare_parameter("measured_velocity_hard_limit_rad_s", 2.50),
      max_measured_velocity_rad_s_, 8.0);
    measured_velocity_persistence_s_ = std::clamp(
      declare_parameter("measured_velocity_persistence_s", 0.15), 0.02, 1.0);
    velocity_recovery_exit_rad_s_ = std::clamp(
      declare_parameter("velocity_recovery_exit_rad_s", 0.10), 0.01,
      max_measured_velocity_rad_s_);
    velocity_recovery_settle_s_ = std::clamp(
      declare_parameter("velocity_recovery_settle_s", 0.50), 0.10, 3.0);
    velocity_recovery_timeout_s_ = std::clamp(
      declare_parameter("velocity_recovery_timeout_s", 3.0), 0.5, 10.0);
    velocity_recovery_hard_limit_rad_s_ = std::clamp(
      declare_parameter("velocity_recovery_hard_limit_rad_s", 1.50),
      max_measured_velocity_rad_s_, measured_velocity_hard_limit_rad_s_);
    max_estimated_arm_torque_nm_ = std::clamp(
      declare_parameter("max_estimated_arm_torque_nm", 25.0), 5.0, 25.0);
    torque_peak_duration_s_ = std::clamp(
      declare_parameter("torque_peak_duration_s", 1.0), 0.05, 3.0);
    max_estimated_waist_torque_nm_ = std::clamp(
      declare_parameter("max_estimated_waist_torque_nm", 25.0), 0.5, 150.0);
    max_motor_temperature_c_ = std::clamp(
      declare_parameter("max_motor_temperature_c", 85.0), 30.0, 100.0);
    required_upper_body_motor_mode_ = static_cast<std::uint8_t>(std::clamp<std::int64_t>(
      declare_parameter<std::int64_t>("required_upper_body_motor_mode", 1), 0, 255));
    takeover_anchor_hold_s_ = std::clamp(
      declare_parameter("takeover_anchor_hold_s", 2.0), 0.5, 10.0);
    release_ramp_s_ = std::clamp(declare_parameter("release_ramp_s", 2.0), 0.5, 10.0);
    command_weight_ = std::clamp(declare_parameter("command_weight", 1.0), 0.05, 1.0);
    arm_kp_ = std::clamp(declare_parameter("arm_kp", 55.0), 0.0, 80.0);
    arm_kd_ = std::clamp(declare_parameter("arm_kd", 3.0), 0.0, 5.0);
    rcl_interfaces::msg::ParameterDescriptor integral_startup_only;
    integral_startup_only.read_only = true;
    integral_startup_only.description = "Startup-only bounded arm integral configuration";
    arm_integral_enabled_ = declare_parameter("arm_integral_enabled", false, integral_startup_only);
    passive_arm_integral_enabled_ = declare_parameter("passive_arm_integral_enabled", false, integral_startup_only);
    const double arm_recovery_deg = declare_parameter("arm_integral_recovery_error_deg",
      0.04 * 180.0 / 3.14159265358979323846, integral_startup_only);
    arm_integral_tuning_.recovery_error_rad = arm_recovery_deg * 3.14159265358979323846 / 180.0;
    if (!arm_integral_tuning_.valid() || arm_recovery_deg > 3.5)
      throw std::invalid_argument("Invalid bounded arm integral recovery range");
    arm_integral_hold_rate_nm_s_ = declare_parameter("arm_integral_hold_rate_nm_s", 0.25, integral_startup_only);
    if (!std::isfinite(arm_integral_hold_rate_nm_s_) || arm_integral_hold_rate_nm_s_ < 0.1 ||
        arm_integral_hold_rate_nm_s_ > 0.5)
      throw std::invalid_argument("Invalid bounded arm hold integral rate");
    arm_integral_ki_ = declare_parameter<std::vector<double>>(
      "arm_integral_ki", std::vector<double>(7, 0.0), integral_startup_only);
    arm_integral_limits_nm_ = declare_parameter<std::vector<double>>(
      "arm_integral_limits_nm", std::vector<double>(7, 0.0), integral_startup_only);
    if (arm_integral_ki_.size() != 7 || arm_integral_limits_nm_.size() != 7)
      throw std::invalid_argument("arm integral configuration requires seven joints");
    for (std::size_t i = 0; i < 7; ++i)
      if (!std::isfinite(arm_integral_ki_[i]) || arm_integral_ki_[i] < 0.0 || arm_integral_ki_[i] > 10.0 ||
          !std::isfinite(arm_integral_limits_nm_[i]) || arm_integral_limits_nm_[i] < 0.0 ||
          // Expand only shoulder compensation; total motor torque guards are unchanged.
          arm_integral_limits_nm_[i] > (i == 0 ? 1.8 : (i == 1 ? 3.6 : (i >= 4 ? 0.3 : 1.0))))
        throw std::invalid_argument("arm integral gains/limits outside allowed range");
    right_wrist_roll_kp_ = std::clamp(
      declare_parameter("right_wrist_roll_kp", 40.0), 0.0, arm_kp_);
    right_wrist_roll_kd_ = std::clamp(
      declare_parameter("right_wrist_roll_kd", 3.5), arm_kd_, 8.0);
    left_wrist_roll_kp_ = std::clamp(
      declare_parameter("left_wrist_roll_kp", right_wrist_roll_kp_), 0.0, arm_kp_);
    left_wrist_roll_kd_ = std::clamp(
      declare_parameter("left_wrist_roll_kd", right_wrist_roll_kd_), arm_kd_, 8.0);
    rcl_interfaces::msg::ParameterDescriptor pd_startup_only;
    pd_startup_only.read_only = true;
    pd_startup_only.description = "Startup-only seven-axis arm PD gains";
    for (const auto& side : {std::string("left"), std::string("right")})
    {
      std::vector<double> kp(7, arm_kp_), kd(7, arm_kd_);
      kp[4] = side == "left" ? left_wrist_roll_kp_ : right_wrist_roll_kp_;
      kd[4] = side == "left" ? left_wrist_roll_kd_ : right_wrist_roll_kd_;
      kp = declare_parameter<std::vector<double>>(side + "_arm_kp", kp, pd_startup_only);
      kd = declare_parameter<std::vector<double>>(side + "_arm_kd", kd, pd_startup_only);
      if (kp.size() != 7 || kd.size() != 7)
        throw std::invalid_argument("arm PD configuration requires seven joints per side");
      for (std::size_t i = 0; i < 7; ++i)
      {
        if (!std::isfinite(kp[i]) || kp[i] < 0.0 || kp[i] > (i < 4 ? 100.0 : 110.0) ||
            !std::isfinite(kd[i]) || kd[i] < 0.0 || kd[i] > (i == 4 ? 8.0 : 5.0))
          throw std::invalid_argument("arm PD gains outside allowed range");
        const auto index = (side == "left" ? 15 : 22) + i;
        motor_kp_[index] = kp[i];
        motor_kd_[index] = kd[i];
      }
    }
    declare_parameter("task_stage_profile", std::string("normal"));
    declare_parameter("task_stage_arm", std::string("left"));
    declare_parameter("task_stage_kp", 0.0);
    declare_parameter("task_stage_wrist_kp", 0.0);
    declare_parameter("task_stage_distal_wrist_kp", 0.0);
    declare_parameter<std::vector<double>>("task_stage_arm_kp", std::vector<double>{});
    declare_parameter("task_stage_kd", 0.0);
    declare_parameter("task_stage_waist_kp", 0.0);
    declare_parameter("task_stage_waist_kd", 0.0);
    declare_parameter("task_stage_stop_epoch", int64_t{0});
    velocity_recovery_kp_ = std::clamp(
      declare_parameter("velocity_recovery_kp", 10.0), 0.0, arm_kp_);
    velocity_recovery_kd_ = std::clamp(
      declare_parameter("velocity_recovery_kd", 3.0), arm_kd_, 10.0);
    gravity_feedforward_enabled_ = declare_parameter("gravity_feedforward_enabled", true);
    gravity_urdf_path_ = declare_parameter<std::string>(
      "gravity_urdf_path",
      "/home/syf/unitree_ws/unitree_ros/robots/g1_description/"
      "g1_29dof_lock_waist_rev_1_0.urdf");
    right_payload_fit_path_ = declare_parameter<std::string>(
      "right_payload_fit_path",
      "/home/syf/g1_upper_body_ws/calibration/right_arm_camera_fit_20260824_185416.yaml");
    left_payload_fit_path_ = declare_parameter<std::string>(
      "left_payload_fit_path",
      "/home/syf/g1_upper_body_ws/calibration/left_arm_right_copy_seed_20260908.yaml");
    active_payload_fit_path_ = active_arm_ == "left" ? left_payload_fit_path_ : right_payload_fit_path_;
    feedforward_blend_s_ = std::clamp(
      declare_parameter("feedforward_blend_s", 2.0), 0.5, 10.0);
    tau_anchor_sample_count_ = static_cast<std::size_t>(std::clamp<std::int64_t>(
      declare_parameter<std::int64_t>("tau_anchor_sample_count", 200), 20, 500));
    tau_anchor_max_spread_nm_ = std::clamp(
      declare_parameter("tau_anchor_max_spread_nm", 0.75), 0.05, 3.0);
    waist_kp_ = declare_parameter("waist_kp", 60.0);
    waist_control_mode_ = declare_parameter("waist_control_mode", std::string("guarded"));
    if (waist_control_mode_ != "guarded" && waist_control_mode_ != "official_pd")
      throw std::runtime_error("waist_control_mode must be guarded or official_pd");
    waist_profile_blend_ = waist_control_mode_ == "official_pd" ? 1.0 : 0.0;
    waist_kd_ = declare_parameter("waist_kd", 4.5);
    waist_ki_ = declare_parameter("waist_ki", 20.0);
    waist_integral_torque_limit_nm_ = declare_parameter(
      "waist_integral_torque_limit_nm", 1.2);
    rcl_interfaces::msg::ParameterDescriptor waist_policy_startup_only;
    waist_policy_startup_only.read_only = true;
    constexpr double degrees_per_radian = 180.0 / 3.14159265358979323846;
    waist_integral_tuning_.recovery_error_rad = declare_parameter(
      "waist_integral_recovery_error_deg", 0.04 * degrees_per_radian,
      waist_policy_startup_only) / degrees_per_radian;
    waist_integral_tuning_.build_rate_nm_s = declare_parameter(
      "waist_integral_build_rate_nm_s", 1.0, waist_policy_startup_only);
    waist_integral_tuning_.curve_gain_scale = declare_parameter(
      "waist_integral_curve_scale", 0.25, waist_policy_startup_only);
    waist_integral_tuning_.smooth_stiction = declare_parameter(
      "waist_integral_smooth_stiction", false, waist_policy_startup_only);
    if (!waist_integral_tuning_.valid())
      throw std::invalid_argument("Invalid bounded waist integral tuning");
    if (!std::isfinite(waist_kp_) || waist_kp_ < 20.0 || waist_kp_ > 80.0)
      throw std::invalid_argument("waist_kp must be finite and within [20, 80]");
    if (!std::isfinite(waist_kd_) || waist_kd_ < 0.5 || waist_kd_ > 5.0)
      throw std::invalid_argument("waist_kd must be finite and within [0.5, 5.0]");
    if (!std::isfinite(waist_ki_) || waist_ki_ < 0.0 || waist_ki_ > 30.0)
      throw std::invalid_argument("waist_ki must be finite and within [0, 30]");
    if (!std::isfinite(waist_integral_torque_limit_nm_) ||
        waist_integral_torque_limit_nm_ < 0.1 || waist_integral_torque_limit_nm_ > 2.0)
      throw std::invalid_argument(
        "waist_integral_torque_limit_nm must be finite and within [0.1, 2.0]");
    controlled_waist_indices_ = declare_parameter<std::vector<std::int64_t>>(
      "controlled_waist_indices", {12});
    std::sort(controlled_waist_indices_.begin(), controlled_waist_indices_.end());
    controlled_waist_indices_.erase(
      std::unique(controlled_waist_indices_.begin(), controlled_waist_indices_.end()),
      controlled_waist_indices_.end());
    if (task_waist_enabled_ && (!task_application_enabled_ ||
        std::find(controlled_waist_indices_.begin(), controlled_waist_indices_.end(), 12) == controlled_waist_indices_.end()))
      throw std::invalid_argument("Task waist trajectories require task mode and ownership of yaw motor 12");
    if (std::any_of(
        controlled_waist_indices_.begin(), controlled_waist_indices_.end(),
        [](const auto index) { return index < 12 || index > 14; }))
    {
      configuration_blocked_ = true;
      RCLCPP_ERROR(get_logger(), "controlled_waist_indices may contain only motor 12, 13, or 14");
    }
    if (gravity_feedforward_enabled_)
    {
      std::string reason;
      if (!active_gravity_model_.initialize(
          gravity_urdf_path_, active_payload_fit_path_, active_arm_, reason))
      {
        configuration_blocked_ = true;
        RCLCPP_ERROR(get_logger(), "Gravity feedforward configuration blocked: %s", reason.c_str());
      }
      else
      {
        RCLCPP_INFO(
          get_logger(), "%s-arm KDL + payload feedforward loaded from %s",
          active_arm_.c_str(), active_payload_fit_path_.c_str());
        if (active_arm_ == "left")
          RCLCPP_WARN(get_logger(), "LEFT commissioning: copied payload seed is NOT a validated left calibration");
      }
    }
    if (task_application_enabled_)
    {
      std::string reason;
      const auto passive = active_arm_ == "left" ? "right" : "left";
      const auto path = active_arm_ == "left" ? right_payload_fit_path_ : left_payload_fit_path_;
      if (!gravity_feedforward_enabled_ ||
          !passive_gravity_model_.initialize(gravity_urdf_path_, path, passive, reason))
        throw std::runtime_error("Task mode requires bilateral gravity support: " + reason);
    }
    service_check_enabled_ = declare_parameter("service_check_enabled", true);
    service_query_period_s_ = std::clamp(
      declare_parameter("service_query_period_s", 1.0), 0.5, 10.0);
    service_state_timeout_s_ = std::clamp(
      declare_parameter("service_state_timeout_s", 3.0), 1.0, 30.0);
    required_active_services_ = declare_parameter<std::vector<std::string>>(
      "required_active_services", {"g1_arm_example", "ai_sport"});
    required_inactive_services_ = declare_parameter<std::vector<std::string>>(
      "required_inactive_services", {"auto_test_arm", "auto_test_low"});

    hardware_output_available_ =
      hardware_output_requested_ && authorization == kRequiredAuthorization;
    if (hardware_output_available_)
    {
      phase_ = "UNARMED";
      RCLCPP_INFO(
        get_logger(),
        "Hardware path authorized; %s publisher is intentionally deferred until "
        "/g1/arm_gateway/arm passes every safety gate",
        command_topic_.c_str());
    }
    else
    {
      phase_ = hardware_output_requested_ ? "CONFIG_BLOCKED" : "OUTPUT_DISABLED";
      if (hardware_output_requested_)
        configuration_blocked_ = true;
      RCLCPP_WARN(
        get_logger(),
        "DRY RUN: no publisher was created for %s (hardware_output_enabled=%s)",
        command_topic_.c_str(), hardware_output_requested_ ? "true" : "false");
    }

    auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    state_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions state_options;
    state_options.callback_group = state_group_;
    state_sub_ = create_subscription<unitree_hg::msg::LowState>(
      state_topic_, state_qos,
      std::bind(&ArmSdkHardwareGateway::state_callback, this, std::placeholders::_1),
      state_options);
    trajectory_sub_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
      trajectory_topic_, rclcpp::QoS(1).reliable().durability_volatile(),
      std::bind(&ArmSdkHardwareGateway::trajectory_callback, this, std::placeholders::_1));
    preview_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      preview_topic_, rclcpp::QoS(1).reliable().durability_volatile());
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_, rclcpp::QoS(1).reliable().durability_volatile());
    if (service_check_enabled_)
    {
      service_request_pub_ = create_publisher<unitree_api::msg::Request>(
        "/api/robot_state/request", rclcpp::QoS(10).reliable().durability_volatile());
      service_response_sub_ = create_subscription<unitree_api::msg::Response>(
        "/api/robot_state/response", rclcpp::QoS(10).reliable().durability_volatile(),
        std::bind(&ArmSdkHardwareGateway::service_response_callback, this, std::placeholders::_1));
      service_query_timer_ = create_wall_timer(
        std::chrono::duration<double>(service_query_period_s_),
        std::bind(&ArmSdkHardwareGateway::service_query_timer_callback, this));
    }

    arm_service_ = create_service<std_srvs::srv::Trigger>(
      "/g1/arm_gateway/arm",
      std::bind(&ArmSdkHardwareGateway::arm_callback, this,
        std::placeholders::_1, std::placeholders::_2));
    disarm_service_ = create_service<std_srvs::srv::Trigger>(
      "/g1/arm_gateway/disarm",
      std::bind(&ArmSdkHardwareGateway::disarm_callback, this,
        std::placeholders::_1, std::placeholders::_2));
    reset_service_ = create_service<std_srvs::srv::Trigger>(
      "/g1/arm_gateway/reset",
      std::bind(&ArmSdkHardwareGateway::reset_callback, this,
        std::placeholders::_1, std::placeholders::_2));
    reset_task_service_ = create_service<ResumeTaskHold>(
      "/g1/arm_gateway/reset_task",
      [this](const std::shared_ptr<ResumeTaskHold::Request> request,
             std::shared_ptr<ResumeTaskHold::Response> response) {
        std::lock_guard<std::mutex> lock(mutex_);
        response->success = reset_silent_locked(response->message, &request->stop_epoch);
      });
    cancel_service_ = create_service<std_srvs::srv::Trigger>(
      "/g1/arm_gateway/cancel_trajectory",
      std::bind(&ArmSdkHardwareGateway::cancel_callback, this,
        std::placeholders::_1, std::placeholders::_2));
    follow_action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
      this, "/" + active_arm_ + "_arm_controller/follow_joint_trajectory",
      std::bind(
        &ArmSdkHardwareGateway::follow_goal_callback, this,
        std::placeholders::_1, std::placeholders::_2, false),
      std::bind(
        &ArmSdkHardwareGateway::follow_cancel_callback, this,
        std::placeholders::_1),
      std::bind(
        &ArmSdkHardwareGateway::follow_accepted_callback, this,
        std::placeholders::_1, false));
    if (task_application_enabled_)
    {
      // Receipt never waits behind the trajectory validator or a control mutex.
      stop_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      if (task_client_watchdog_s_ > 0.) {
        rclcpp::SubscriptionOptions options;
        options.callback_group = stop_group_;
        task_heartbeat_sub_ = create_subscription<std_msgs::msg::Empty>(
          "/g1/arm_gateway/task_heartbeat", rclcpp::QoS(1),
          [this](const std_msgs::msg::Empty::SharedPtr) {
            task_heartbeat_ns_.store(std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::steady_clock::now().time_since_epoch()).count());
          }, options);
      }
      stop_service_ = create_service<std_srvs::srv::Trigger>(
        "/g1/arm_gateway/stop_task",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          ++task_stop_epoch_;
          last_task_stop_stamp_.store(now().nanoseconds());
          task_waist_admittance_requested_.store(false);
          task_stop_requested_.store(true);
          response->success = true;
          response->message = "stop queued; inspect task_stop_active/phase for controller acknowledgement";
        }, rmw_qos_profile_services_default, stop_group_);
      waist_admittance_service_ = create_service<ResumeTaskHold>(
        "/g1/arm_gateway/stop_task_with_waist",
        [this](const std::shared_ptr<ResumeTaskHold::Request> request,
               std::shared_ptr<ResumeTaskHold::Response> response) {
          std::lock_guard<std::mutex> lock(mutex_);
          if (request->stop_epoch != task_stop_epoch_.load() || !task_waist_enabled_ ||
              !armed_ || phase_ != "ARMED_HOLD" || safety_latched_ || releasing_ || waist_mode_switching_ ||
              velocity_recovery_active_ || task_stop_requested_.load() || task_stop_active_ ||
              !state_is_fresh_locked() || !service_state_is_ready_locked() ||
              current_max_measured_velocity_rad_s_ > .03 || torque_overload_active_ ||
              command_overload_active_ || !upper_body_fault_reason_.empty() ||
              actual_motor_position_[12] < kTaskWaistYaw.lower + joint_limit_margin_rad_ ||
              actual_motor_position_[12] > kTaskWaistYaw.upper - joint_limit_margin_rad_) {
            response->success = false;
            response->message = "waist admittance requires current epoch, healthy stationary ARMED_HOLD and owned yaw within limits";
            return;
          }
          ++task_stop_epoch_;
          last_task_stop_stamp_.store(now().nanoseconds());
          task_waist_admittance_requested_.store(true);
          task_stop_requested_.store(true);
          response->success = true;
          response->message = "arms and waist yaw admittance queued; wait for diagnostics";
        }, rmw_qos_profile_services_default, stop_group_);
      resume_service_ = create_service<ResumeTaskHold>(
        "/g1/arm_gateway/hold_after_stop",
        std::bind(&ArmSdkHardwareGateway::hold_after_stop_callback, this,
                  std::placeholders::_1, std::placeholders::_2));
      passive_follow_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
        this, std::string(active_arm_ == "left" ? "/right" : "/left") +
        "_arm_controller/follow_joint_trajectory",
        std::bind(&ArmSdkHardwareGateway::follow_goal_callback, this,
                  std::placeholders::_1, std::placeholders::_2, false),
        std::bind(&ArmSdkHardwareGateway::follow_cancel_callback, this, std::placeholders::_1),
        std::bind(&ArmSdkHardwareGateway::follow_accepted_callback, this, std::placeholders::_1, false));
      if (task_waist_enabled_)
        waist_follow_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
          this, "/g1/arm_gateway/follow_waist_trajectory",
          std::bind(&ArmSdkHardwareGateway::follow_goal_callback, this,
                    std::placeholders::_1, std::placeholders::_2, true),
          std::bind(&ArmSdkHardwareGateway::follow_cancel_callback, this, std::placeholders::_1),
          std::bind(&ArmSdkHardwareGateway::follow_accepted_callback, this, std::placeholders::_1, true));
    }

    control_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    control_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / control_rate_hz_),
      std::bind(&ArmSdkHardwareGateway::control_timer_callback, this), control_group_);
    diagnostics_timer_ = create_wall_timer(
      100ms, std::bind(&ArmSdkHardwareGateway::diagnostics_timer_callback, this));
    parameter_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&ArmSdkHardwareGateway::parameters_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Gateway limits: v<=%.3f rad/s, a<=%.3f rad/s^2, step<=%.4f rad, track %.3f/%.3f/%.3f rad",
      max_velocity_rad_s_, max_acceleration_rad_s2_, max_command_step_rad_,
      tracking_warning_rad_, tracking_pause_rad_, tracking_hard_limit_rad_);
    RCLCPP_INFO(
      get_logger(),
      "Measured-state trip limits: |dq|<=%.3f rad/s for %.3f s or instant |dq|>%.3f rad/s, "
      "large-arm torque %.1f/%.1f Nm continuous/peak, wrist pitch/yaw %.1f/%.1f Nm, "
      "peak window %.2f s, waist |tau_est|<=%.1f Nm, temperature alarm %.1f C",
      max_measured_velocity_rad_s_, measured_velocity_persistence_s_,
      measured_velocity_hard_limit_rad_s_, kArmContinuousTorqueLimits[0],
      kArmPeakTorqueLimits[0], kArmContinuousTorqueLimits[5], kArmPeakTorqueLimits[5],
      torque_peak_duration_s_, max_estimated_waist_torque_nm_, max_motor_temperature_c_);
    RCLCPP_INFO(
      get_logger(),
      "Waist gains: Kp=%.1f Kd=%.1f Ki=%.1f, |tau_i|<=%.2f Nm; "
      "runtime updates require idle output and guarded bounds",
      waist_kp_, waist_kd_, waist_ki_, waist_integral_torque_limit_nm_);
  }

private:
  static constexpr double kTaskStageBlendSeconds = 0.4;

  double stage_blend_locked() const
  {
    return quintic(std::clamp(std::chrono::duration<double>(
      std::chrono::steady_clock::now()-stage_started_).count()/kTaskStageBlendSeconds, 0.0, 1.0));
  }

  bool stage_switching_locked() const { return stage_blend_locked() < 1.0; }

  double stage_gain_locked(std::size_t index, bool damping) const
  {
    const double base = damping ? motor_kd_[index] : motor_kp_[index];
    if (index < stage_first_motor_ || index >= stage_first_motor_+7) return base;
    const bool wrist = index >= stage_first_motor_+4;
    const double wrist_kp = index >= stage_first_motor_+5 && stage_distal_wrist_kp_>0 ?
      stage_distal_wrist_kp_ : stage_wrist_kp_;
    double target = stage_profile_ == "normal" ? base :
      (wrist ? (!damping && wrist_kp>0 ? wrist_kp : base) :
        (damping ? stage_kd_ : stage_kp_));
    if (stage_profile_ != "normal" && !damping && !stage_arm_kp_.empty())
      target = stage_arm_kp_[index-stage_first_motor_];
    const double from = damping ? stage_from_kd_[index-stage_first_motor_] :
      stage_from_kp_[index-stage_first_motor_];
    return from+(target-from)*stage_blend_locked();
  }

  double stage_waist_gain_locked(bool damping) const
  {
    const double base = damping ? waist_kd_ : waist_kp_;
    const double target = stage_profile_ == "normal" || stage_waist_kp_ == 0 ? base :
      (damping ? stage_waist_kd_ : stage_waist_kp_);
    const double from = damping ? stage_from_waist_kd_ : stage_from_waist_kp_;
    return from+(target-from)*stage_blend_locked();
  }

  void set_stage_locked(const std::string& profile, std::size_t first, double kp, double kd,
                        double waist_kp=0, double waist_kd=0, double wrist_kp=0, double distal_wrist_kp=0,
                        const std::vector<double>& arm_kp={})
  {
    std::array<double, 7> from_kp{}, from_kd{};
    for (std::size_t i=0; i<7; ++i)
    {
      from_kp[i]=stage_gain_locked(first+i, false);
      from_kd[i]=stage_gain_locked(first+i, true);
    }
    stage_from_waist_kp_=stage_waist_gain_locked(false);
    stage_from_waist_kd_=stage_waist_gain_locked(true);
    stage_waist_kp_=waist_kp; stage_waist_kd_=waist_kd;
    stage_wrist_kp_=wrist_kp;
    stage_distal_wrist_kp_=distal_wrist_kp;
    stage_arm_kp_=arm_kp;
    stage_from_kp_=from_kp; stage_from_kd_=from_kd;
    stage_first_motor_=first; stage_profile_=profile; stage_kp_=kp; stage_kd_=kd;
    stage_started_=std::chrono::steady_clock::now();
    stage_last_active_=stage_started_;
    ++stage_revision_;
  }

  void restore_stage_locked()
  {
    if (stage_profile_ != "normal") set_stage_locked("normal", stage_first_motor_, 0, 0);
  }

  rcl_interfaces::msg::SetParametersResult stage_parameters_callback(
    const std::vector<rclcpp::Parameter>& parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful=false;
    std::map<std::string, const rclcpp::Parameter*> values;
    for (const auto& p : parameters) values.emplace(p.get_name(), &p);
    std::map<std::string, rclcpp::ParameterType> types{
      {"task_stage_profile", rclcpp::ParameterType::PARAMETER_STRING},
      {"task_stage_arm", rclcpp::ParameterType::PARAMETER_STRING},
      {"task_stage_kp", rclcpp::ParameterType::PARAMETER_DOUBLE},
      {"task_stage_kd", rclcpp::ParameterType::PARAMETER_DOUBLE},
      {"task_stage_stop_epoch", rclcpp::ParameterType::PARAMETER_INTEGER}};
    const bool waist_request=values.count("task_stage_waist_kp") || values.count("task_stage_waist_kd");
    if (values.count("task_stage_wrist_kp"))
      types.emplace("task_stage_wrist_kp", rclcpp::ParameterType::PARAMETER_DOUBLE);
    if (values.count("task_stage_distal_wrist_kp"))
      types.emplace("task_stage_distal_wrist_kp", rclcpp::ParameterType::PARAMETER_DOUBLE);
    if (values.count("task_stage_arm_kp"))
      types.emplace("task_stage_arm_kp", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
    if (waist_request)
    {
      types.emplace("task_stage_waist_kp", rclcpp::ParameterType::PARAMETER_DOUBLE);
      types.emplace("task_stage_waist_kd", rclcpp::ParameterType::PARAMETER_DOUBLE);
    }
    result.reason="Stage update requires profile, arm, kp, kd, stop epoch and optionally both waist gains";
    if (parameters.size()!=types.size() || values.size()!=types.size()) return result;
    for (const auto& item : types)
      if (!values.count(item.first) || values.at(item.first)->get_type()!=item.second) return result;
    const auto profile=values.at("task_stage_profile")->as_string();
    const auto arm=values.at("task_stage_arm")->as_string();
    const auto epoch=values.at("task_stage_stop_epoch")->as_int();
    const auto arm_kp=values.count("task_stage_arm_kp") ?
      values.at("task_stage_arm_kp")->as_double_array() : std::vector<double>{};
    result.reason="Stage arm Kp requires seven nonnegative finite LowCmd gains; normal requires empty";
    if ((!arm_kp.empty() && (arm_kp.size()!=7 || !std::all_of(arm_kp.begin(),arm_kp.end(),valid_stage_gain))) ||
        (profile=="normal" && !arm_kp.empty())) return result;
    const double kp=values.at("task_stage_kp")->as_double(), kd=values.at("task_stage_kd")->as_double();
    const double wkp=waist_request ? values.at("task_stage_waist_kp")->as_double() : 0;
    const double wkd=waist_request ? values.at("task_stage_waist_kd")->as_double() : 0;
    const double wrist_kp=values.count("task_stage_wrist_kp") ? values.at("task_stage_wrist_kp")->as_double() : 0;
    result.reason="Replay wrist Kp must be finite in [0,110]; normal requires zero";
    if (!std::isfinite(wrist_kp) || wrist_kp<0 || wrist_kp>110 ||
        (profile=="normal" && wrist_kp!=0)) return result;
    const double distal_wrist_kp=values.count("task_stage_distal_wrist_kp") ?
      values.at("task_stage_distal_wrist_kp")->as_double() : 0;
    result.reason="Replay distal wrist Kp must be finite in [0,220]; normal requires zero";
    if (!std::isfinite(distal_wrist_kp) || distal_wrist_kp<0 || distal_wrist_kp>220 ||
        (profile=="normal" && distal_wrist_kp!=0)) return result;
    result.reason="Invalid stage, arm or gains; gains must be nonnegative finite LowCmd values; normal requires zeros";
    if ((profile!="normal" && profile!="rotate" && profile!="withdraw") ||
        (arm!="left" && arm!="right") || !valid_stage_gain(kp) || !valid_stage_gain(kd) ||
        (profile=="normal" && (kp!=0 || kd!=0))) return result;
    result.reason="Invalid stage waist gains; use finite LowCmd values, both zero or Kp>=20 and Kd>=0.5";
    if (!valid_stage_gain(wkp) || !valid_stage_gain(wkd) ||
        ((wkp!=0 || wkd!=0) && (wkp<20 || wkd<.5)) ||
        (profile=="normal" && (wkp!=0 || wkd!=0))) return result;
    std::lock_guard<std::mutex> lock(mutex_);
    std::string failures;
    auto blocked=[&](bool condition, const std::string& name) {
      if (condition) { if (!failures.empty()) failures+=", "; failures+=name; }
    };
    blocked(epoch<0 || static_cast<uint64_t>(epoch)!=task_stop_epoch_.load(), "STOP epoch mismatch");
    blocked(!task_application_enabled_, "task mode disabled");
    blocked(!armed_ || !command_pub_, "output not armed");
    blocked(phase_!="ARMED_HOLD", "phase="+phase_);
    blocked(active_trajectory_, "trajectory active");
    blocked(safety_latched_ || releasing_ || velocity_recovery_active_, "latch/release/recovery active");
    blocked(task_stop_active_ || task_stop_requested_.load() || task_resume_blend_, "STOP/resume active");
    blocked(waist_mode_switching_ || stage_switching_locked(), "gain blend active");
    blocked(!state_is_fresh_locked(), "LowState stale");
    blocked(!service_state_is_ready_locked(), "onboard service state not ready");
    blocked(!upper_body_state_safe_ || !upper_body_fault_reason_.empty(), "unsafe state: "+upper_body_fault_reason_);
    blocked(wkp>0 && (!task_waist_enabled_ || waist_control_mode_!="guarded"),
      "stage waist requires task waist enabled in guarded mode");
    blocked(torque_overload_active_ || command_overload_active_, "torque overload");
    blocked(temperature_alarm_active_, "temperature alarm");
    if (!failures.empty()) { result.reason="Stage change rejected: "+failures; return result; }
    if (current_max_measured_velocity_rad_s_>.03)
    {
      result.reason="STAGE_WAIT_STATIONARY: measured speed="+
        std::to_string(current_max_measured_velocity_rad_s_)+" rad/s > 0.03";
      return result;
    }
    const std::size_t first=arm=="left" ? 15 : 22;
    if (stage_profile_!="normal" && first!=stage_first_motor_)
    { result.reason="Stage arm differs; restore normal profile first"; return result; }
    for (std::size_t i=0; i<(wrist_kp>0 || distal_wrist_kp>0 || !arm_kp.empty() ? 7u : 4u); ++i)
      if (std::abs(hold_motor_position_[first+i]-actual_motor_position_[first+i])>.025)
      {
        result.reason="Stage hold error at motor "+std::to_string(first+i)+" = "+
          std::to_string(hold_motor_position_[first+i]-actual_motor_position_[first+i])+" rad exceeds 0.025";
        return result;
      }
    if (wkp>0 && std::abs(hold_motor_position_[12]-actual_motor_position_[12])>.025)
    { result.reason="Stage hold error at waist yaw exceeds 0.025 rad"; return result; }
    if (profile=="normal") restore_stage_locked();
    else set_stage_locked(profile, first, kp, kd, wkp, wkd, wrist_kp, distal_wrist_kp, arm_kp);
    result.successful=true; result.reason="Accepted; wait for stage revision and blend completion";
    return result;
  }

  rcl_interfaces::msg::SetParametersResult parameters_callback(
    const std::vector<rclcpp::Parameter>& parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    if (std::any_of(parameters.begin(), parameters.end(), [](const auto& p) {
        return p.get_name().rfind("task_stage_", 0) == 0; }))
      return stage_parameters_callback(parameters);
    for (const auto& parameter : parameters)
    {
      if (parameter.get_name() != "waist_control_mode") continue;
      std::lock_guard<std::mutex> lock(mutex_);
      if (parameters.size() != 1 ||
          parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
          (parameter.as_string() != "guarded" && parameter.as_string() != "official_pd"))
      {
        result.successful = false;
        result.reason = "Set only waist_control_mode: guarded or official_pd";
      }
      else if (stage_profile_ != "normal" || stage_switching_locked() ||
               releasing_ || safety_latched_ || velocity_recovery_active_ ||
               active_trajectory_ || task_stop_active_ || task_resume_blend_ ||
               task_stop_requested_.load() || waist_mode_switching_ ||
               torque_overload_active_ || command_overload_active_ || temperature_alarm_active_ ||
               !upper_body_fault_reason_.empty() ||
               (!armed_ && command_pub_) ||
               (armed_ && (phase_ != "ARMED_HOLD" || !state_is_fresh_locked() ||
                !service_state_is_ready_locked() || !upper_body_state_safe_ ||
                current_max_measured_velocity_rad_s_ > .03 ||
                std::abs(hold_motor_position_[12]-actual_motor_position_[12]) > .015)))
      {
        result.successful = false;
        result.reason = "Waist mode requires UNARMED or fresh stationary ARMED_HOLD, waist error<=0.015 rad; no stop, fault or transition";
      }
      else if (waist_control_mode_ != parameter.as_string())
      {
        waist_control_mode_ = parameter.as_string();
        ++waist_control_revision_;
        waist_switch_from_ = waist_profile_blend_;
        waist_switch_started_ = std::chrono::steady_clock::now();
        waist_mode_switching_ = armed_;
        waist_command_valid_ = false;
        if (!armed_)
        {
          waist_profile_blend_ = waist_control_mode_ == "official_pd" ? 1.0 : 0.0;
          reset_integral_locked();
        }
      }
      return result;
    }
    double requested_kp = 0.0;
    double requested_kd = 0.0;
    double requested_ki = 0.0;
    double requested_integral_limit = 0.0;
    bool has_kp = false;
    bool has_kd = false;
    bool has_ki = false;
    bool has_integral_limit = false;
    bool gain_update_requested = false;
    for (const auto& parameter : parameters)
    {
      if (parameter.get_name() != "waist_kp" && parameter.get_name() != "waist_kd" &&
          parameter.get_name() != "waist_ki" &&
          parameter.get_name() != "waist_integral_torque_limit_nm")
        continue;
      gain_update_requested = true;
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        result.successful = false;
        result.reason = parameter.get_name() + " must be a floating-point value";
        return result;
      }
      if (parameter.get_name() == "waist_kp")
      {
        requested_kp = parameter.as_double();
        has_kp = true;
      }
      else if (parameter.get_name() == "waist_kd")
      {
        requested_kd = parameter.as_double();
        has_kd = true;
      }
      else if (parameter.get_name() == "waist_ki")
      {
        requested_ki = parameter.as_double();
        has_ki = true;
      }
      else
      {
        requested_integral_limit = parameter.as_double();
        has_integral_limit = true;
      }
    }
    if (!gain_update_requested)
      return result;

    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_kp)
      requested_kp = waist_kp_;
    if (!has_kd)
      requested_kd = waist_kd_;
    if (!has_ki)
      requested_ki = waist_ki_;
    if (!has_integral_limit)
      requested_integral_limit = waist_integral_torque_limit_nm_;
    const auto reject = [this, &result](const std::string& reason) {
      result.successful = false;
      result.reason = reason;
      last_waist_gain_rejection_ = reason;
      RCLCPP_WARN(get_logger(), "Rejected waist gain update: %s", reason.c_str());
    };
    if (!std::isfinite(requested_kp) || requested_kp < 20.0 || requested_kp > 80.0)
    {
      reject("waist_kp must be finite and within [20, 80]");
      return result;
    }
    if (!std::isfinite(requested_kd) || requested_kd < 0.5 || requested_kd > 5.0)
    {
      reject("waist_kd must be finite and within [0.5, 5.0]");
      return result;
    }
    if (!std::isfinite(requested_ki) || requested_ki < 0.0 || requested_ki > 30.0)
    {
      reject("waist_ki must be finite and within [0, 30]");
      return result;
    }
    if (!std::isfinite(requested_integral_limit) ||
        requested_integral_limit < 0.1 || requested_integral_limit > 2.0)
    {
      reject("waist_integral_torque_limit_nm must be finite and within [0.1, 2.0]");
      return result;
    }
    if (std::abs(requested_kp - waist_kp_) > 20.0 + 1e-9 ||
        std::abs(requested_kd - waist_kd_) > 1.5 + 1e-9 ||
        std::abs(requested_ki - waist_ki_) > 20.0 + 1e-9 ||
        std::abs(requested_integral_limit - waist_integral_torque_limit_nm_) > 0.5 + 1e-9)
    {
      reject(
        "one update may change waist_kp by at most 20, waist_kd by at most 1.5, "
        "waist_ki by at most 20, and integral torque limit by at most 0.5 Nm");
      return result;
    }
    if (active_trajectory_ || releasing_ || velocity_recovery_active_ || task_stop_active_ || waist_mode_switching_ ||
        stage_profile_!="normal" || stage_switching_locked())
    {
      reject("waist gains cannot change during trajectory, release, recovery, or task stop");
      return result;
    }
    if (armed_)
    {
      if (phase_ != "ARMED_HOLD" || !state_is_fresh_locked())
      {
        reject("armed gain updates require ARMED_HOLD with fresh LowState");
        return result;
      }
      double max_velocity = 0.0;
      double max_hold_error = 0.0;
      for (const auto configured_index : controlled_waist_indices_)
      {
        const auto index = static_cast<std::size_t>(configured_index);
        max_velocity = std::max(max_velocity, std::abs(actual_motor_velocity_[index]));
        max_hold_error = std::max(
          max_hold_error, std::abs(hold_motor_position_[index] - actual_motor_position_[index]));
      }
      if (max_velocity > 0.03 || max_hold_error > 0.015)
      {
        reject("armed gain updates require waist |dq|<=0.03 rad/s and hold error<=0.015 rad");
        return result;
      }
      double active_integral = 0.0;
      for (const auto configured_index : controlled_waist_indices_)
        active_integral = std::max(
          active_integral,
          std::abs(waist_integral_torque_nm_[static_cast<std::size_t>(configured_index)]));
      if (requested_integral_limit + 1e-9 < active_integral)
      {
        reject("cannot lower integral torque limit below the active integral output while armed");
        return result;
      }
    }
    if (requested_kp == waist_kp_ && requested_kd == waist_kd_ &&
        requested_ki == waist_ki_ &&
        requested_integral_limit == waist_integral_torque_limit_nm_)
      return result;

    const double previous_kp = waist_kp_;
    const double previous_kd = waist_kd_;
    const double previous_ki = waist_ki_;
    const double previous_integral_limit = waist_integral_torque_limit_nm_;
    waist_kp_ = requested_kp;
    waist_kd_ = requested_kd;
    waist_ki_ = requested_ki;
    waist_integral_torque_limit_nm_ = requested_integral_limit;
    if (!armed_)
      reset_integral_locked();
    ++waist_gain_revision_;
    std::ostringstream audit;
    audit << "revision " << waist_gain_revision_ << ": Kp " << previous_kp << " -> " << waist_kp_
          << ", Kd " << previous_kd << " -> " << waist_kd_
          << ", Ki " << previous_ki << " -> " << waist_ki_
          << ", |tau_i| " << previous_integral_limit << " -> "
          << waist_integral_torque_limit_nm_ << " Nm"
          << ", armed=" << (armed_ ? "true" : "false")
          << ", time_ns=" << now().nanoseconds();
    last_waist_gain_change_ = audit.str();
    last_waist_gain_rejection_.clear();
    RCLCPP_WARN(get_logger(), "Applied guarded waist gain update: %s", last_waist_gain_change_.c_str());
    return result;
  }

  void reset_integral_locked()
  {
    waist_command_valid_ = false;
    if (!armed_)
    {
      waist_mode_switching_ = false;
      waist_profile_blend_ = waist_control_mode_ == "official_pd" ? 1.0 : 0.0;
      waist_official_blend_ = 0.0;
      waist_command_ = unitree_hg::msg::LowCmd{};
    }
    for (auto& controller : arm_integral_controllers_)
      controller.reset();
    arm_integral_torque_nm_.fill(0.0);
    arm_integral_modes_.fill(g1_task::WaistIntegralMode::Disabled);
    arm_integral_saturated_ = false;
    for (auto& controller : waist_integral_controllers_)
      controller.reset();
    waist_integral_torque_nm_.fill(0.0);
    current_waist_integral_torque_nm_ = 0.0;
    waist_integral_gain_scale_ = 0.0;
    waist_integral_active_ = false;
    waist_integral_saturated_ = false;
    waist_integral_mode_ = "disabled";
  }

  void update_integral_locked(
    const TrajectorySample& active_sample, double time_scale)
  {
    auto desired = hold_motor_position_;
    std::array<double, 35> desired_velocity{};
    std::array<double, 35> desired_acceleration{};
    std::array<bool, 35> moving_target{};
    if (active_trajectory_)
    {
      for (std::size_t j = 0; j < active_arm_indices_.size(); ++j)
      {
        const auto motor_index = trajectory_joint(active_arm_indices_[j]).motor_index;
        desired[motor_index] = active_sample.positions[j];
        desired_velocity[motor_index] =
          bounded_trajectory_velocity_locked(j, active_sample.velocities[j], time_scale);
        desired_acceleration[motor_index] =
          active_sample.accelerations[j] * time_scale * time_scale +
          active_sample.velocities[j] * trajectory_time_scale_rate_;
        moving_target[motor_index] = true;
      }
    }

    const bool takeover_complete = armed_ &&
      (now() - takeover_started_time_).seconds() >= takeover_anchor_hold_s_ + feedforward_blend_s_;
    const bool enabled = hardware_output_available_ && armed_ && takeover_complete &&
      !releasing_ && !velocity_recovery_active_ && !task_stop_active_ &&
      !task_resume_blend_ && task_compliance_blend_ <= 1e-9 &&
      command_weight_current_ >= command_weight_ - 1e-6;
    const double dt = 1.0 / control_rate_hz_;
    current_waist_integral_torque_nm_ = 0.0;
    waist_integral_gain_scale_ = 0.0;
    waist_integral_active_ = false;
    waist_integral_saturated_ = false;
    waist_integral_mode_ = "disabled";
    for (const auto configured_index : controlled_waist_indices_)
    {
      const auto index = static_cast<std::size_t>(configured_index);
      const auto result = waist_integral_controllers_[index].step(
        desired[index] - actual_motor_position_[index], desired_velocity[index],
        desired_acceleration[index], actual_motor_velocity_[index], dt,
        enabled && waist_profile_blend_ < 1.0,
        moving_target[index], waist_ki_, waist_integral_torque_limit_nm_, 1.0, waist_integral_tuning_);
      waist_integral_torque_nm_[index] = result.torque;
      waist_integral_active_ = waist_integral_active_ || result.integrating;
      waist_integral_saturated_ = waist_integral_saturated_ || result.saturated;
      if (index == 12 || std::abs(result.torque) > std::abs(current_waist_integral_torque_nm_))
      {
        current_waist_integral_torque_nm_ = result.torque;
        waist_integral_gain_scale_ = result.gain_scale;
        waist_integral_mode_ = g1_task::waist_integral_mode_name(result.mode);
      }
    }
    peak_waist_integral_torque_nm_ = std::max(
      peak_waist_integral_torque_nm_, std::abs(current_waist_integral_torque_nm_));
    arm_integral_saturated_ = false;
    // Captured support can drift too. Each arm integrates its own error, with
    // the same reset/overload gates and an explicit opt-in for the other arm.
    for (std::size_t j = 0; j < kArmJoints.size(); ++j)
    {
      const auto index = kArmJoints[j].motor_index;
      const auto i = j % 7;
      const bool selected = (j < 7) == (active_arm_ == "left");
      const bool arm_enabled = enabled && (selected ? arm_integral_enabled_ : passive_arm_integral_enabled_) &&
        !torque_overload_active_ && !command_overload_active_ && !feedforward_saturated_;
      const double rate_scale = g1_task::arm_integral_rate_scale(desired_velocity[index],
        desired_acceleration[index], actual_motor_velocity_[index], arm_integral_hold_rate_nm_s_);
      const auto result = arm_integral_controllers_[index].step(
        desired[index] - actual_motor_position_[index], desired_velocity[index],
        desired_acceleration[index], actual_motor_velocity_[index], dt, arm_enabled,
        moving_target[index], arm_integral_ki_[i], arm_integral_limits_nm_[i], rate_scale, arm_integral_tuning_);
      arm_integral_torque_nm_[index] = result.torque;
      arm_integral_modes_[index] = result.mode;
      arm_integral_saturated_ = arm_integral_saturated_ || result.saturated;
    }
  }

  void latch_locked(const std::string& reason)
  {
    restore_stage_locked();
    close_motion_log_locked(reason);
    active_trajectory_ = false;
    velocity_recovery_active_ = false;
    reset_integral_locked();
    if (active_follow_goal_ && active_follow_goal_->is_active())
    {
      auto result = std::make_shared<FollowJointTrajectory::Result>();
      result->error_code = FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED;
      result->error_string = reason;
      active_follow_goal_->abort(result);
    }
    active_follow_goal_.reset();
    service_state_validated_for_takeover_ = false;
    armed_ = false;
    releasing_ = false;
    command_pub_.reset();
    waist_mode_switching_ = false;
    if (!safety_latched_ || latch_reason_ != reason)
      RCLCPP_ERROR(get_logger(), "HARDWARE GATEWAY SAFETY LATCHED: %s", reason.c_str());
    safety_latched_ = true;
    latch_reason_ = reason;
    phase_ = "LATCHED_SILENT";
    // Deliberately publish nothing here. Native L2+B damping must retain priority.
  }

  bool state_is_fresh_locked() const
  {
    return state_received_ &&
      std::chrono::duration<double>(std::chrono::steady_clock::now() - last_state_steady_).count()
        <= state_timeout_s_;
  }

  bool service_state_is_fresh_locked() const
  {
    if (!service_check_enabled_)
      return true;
    return service_state_received_ &&
      (now() - last_service_state_time_).seconds() <= service_state_timeout_s_ &&
      !service_conflict_;
  }

  bool service_state_is_ready_locked() const
  {
    if (!service_check_enabled_)
      return true;
    if (service_conflict_)
      return false;
    // The current firmware stops replying to robot_state API 1003 after the
    // arm_sdk takeover. Require a fresh validation for ARM, then retain that
    // validated state while armed and continue to latch on explicit conflicts.
    if (armed_ && service_state_validated_for_takeover_)
      return true;
    return service_state_is_fresh_locked();
  }

  void service_query_timer_callback()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!service_request_pub_)
      return;
    unitree_api::msg::Request request;
    request.header.identity.api_id = kServiceListApiId;
    request.header.identity.id = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    request.parameter = "{}";
    pending_service_request_ids_.push_back(request.header.identity.id);
    const auto max_pending = std::max<std::size_t>(
      4, static_cast<std::size_t>(std::ceil(service_state_timeout_s_ / service_query_period_s_)) + 2);
    while (pending_service_request_ids_.size() > max_pending)
      pending_service_request_ids_.pop_front();
    service_request_pub_->publish(request);
  }

  void service_response_callback(const unitree_api::msg::Response::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (message->header.identity.api_id != kServiceListApiId)
      return;
    const auto request = std::find(
      pending_service_request_ids_.begin(), pending_service_request_ids_.end(),
      message->header.identity.id);
    if (request == pending_service_request_ids_.end())
      return;
    pending_service_request_ids_.erase(request);
    if (message->header.status.code != 0)
    {
      service_conflict_ = true;
      service_conflict_reason_ =
        "ServiceList query failed with code " + std::to_string(message->header.status.code);
      if (armed_)
        latch_locked(service_conflict_reason_);
      return;
    }

    std::vector<std::pair<std::string, int>> services;
    const std::regex object_pattern(R"(\{[^{}]*\})");
    const std::regex name_pattern(R"json("name"\s*:\s*"([^"]+)")json");
    const std::regex status_pattern(R"json("status"\s*:\s*(-?\d+))json");
    for (std::sregex_iterator it(message->data.begin(), message->data.end(), object_pattern), end;
         it != end; ++it)
    {
      std::smatch name_match;
      std::smatch status_match;
      const std::string object = it->str();
      if (std::regex_search(object, name_match, name_pattern) &&
          std::regex_search(object, status_match, status_pattern))
        services.emplace_back(name_match[1].str(), std::stoi(status_match[1].str()));
    }

    const auto find_status = [&services](const std::string& name, int& status) {
      const auto it = std::find_if(
        services.begin(), services.end(),
        [&name](const auto& entry) { return entry.first == name; });
      if (it == services.end())
        return false;
      status = it->second;
      return true;
    };

    service_conflict_ = false;
    service_conflict_reason_.clear();
    for (const auto& name : required_active_services_)
    {
      int status = -1;
      if (!find_status(name, status) || status != 0)
      {
        service_conflict_ = true;
        service_conflict_reason_ = "required arm service is not active: " + name;
        break;
      }
    }
    if (!service_conflict_)
    {
      for (const auto& name : required_inactive_services_)
      {
        int status = -1;
        if (!find_status(name, status) || status != 1)
        {
          service_conflict_ = true;
          service_conflict_reason_ = "conflicting onboard service is not inactive: " + name;
          break;
        }
      }
    }
    service_state_received_ = true;
    last_service_state_time_ = now();
    service_state_count_ = services.size();
    if (service_conflict_ && armed_)
      latch_locked(service_conflict_reason_);
  }

  static std::string csv_string(const std::string& value)
  {
    std::string escaped;
    escaped.reserve(value.size() + 2);
    escaped.push_back('"');
    for (const char character : value)
    {
      escaped.push_back(character);
      if (character == '"')
        escaped.push_back('"');
    }
    escaped.push_back('"');
    return escaped;
  }

  double bounded_trajectory_velocity_locked(
    std::size_t trajectory_index, double reference_velocity, double time_scale) const
  {
    const auto& spec = trajectory_joint(active_arm_indices_.at(trajectory_index));
    const double limit = spec.motor_index == 12 ?
      task_waist_velocity_limit_rad_s_ : max_velocity_rad_s_;
    return std::clamp(reference_velocity * time_scale, -limit, limit);
  }

  void open_motion_log_locked(const rclcpp::Time& stamp)
  {
    close_motion_log_locked("superseded by a new trajectory");
    std::error_code directory_error;
    std::filesystem::create_directories(trajectory_log_directory_, directory_error);
    if (directory_error)
    {
      last_motion_log_error_ = "cannot create trajectory log directory: " +
        directory_error.message();
      RCLCPP_WARN(get_logger(), "%s", last_motion_log_error_.c_str());
      return;
    }

    const auto system_now = std::chrono::system_clock::now();
    const auto time = std::chrono::system_clock::to_time_t(system_now);
    std::tm local_time{};
    localtime_r(&time, &local_time);
    std::ostringstream filename;
    filename << "endpoint_motion_" << std::put_time(&local_time, "%Y%m%d_%H%M%S") << "_"
             << std::setw(3) << std::setfill('0') << accepted_trajectories_ + 1 << ".csv";
    const auto path = std::filesystem::path(trajectory_log_directory_) / filename.str();
    motion_log_.open(path, std::ios::out | std::ios::trunc);
    if (!motion_log_)
    {
      last_motion_log_error_ = "cannot open trajectory log: " + path.string();
      RCLCPP_WARN(get_logger(), "%s", last_motion_log_error_.c_str());
      return;
    }

    last_motion_log_path_ = path.string();
    last_waist_command_log_path_.clear();
    if (task_waist_enabled_)
    {
      std::error_code error;
      const auto directory = path.parent_path() / "waist_commands";
      std::filesystem::create_directories(directory, error);
      if (!error)
      {
        waist_command_log_.clear();
        waist_command_log_.open(directory / path.filename());
      }
      if (!error && waist_command_log_.is_open())
      {
        last_waist_command_log_path_ = (directory / path.filename()).string();
        waist_command_log_rows_ = 0;
        waist_command_log_ << "publish_return_steady_ns,command_sequence,phase,waist_control_mode,"
          "waist_control_revision,waist_official_blend,control_gap_ms,lowstate_steady_age_ms,"
          "reference_q,reference_dq,sent_q,sent_dq,sent_tau,sent_kp,sent_kd,command_weight,"
          "actual_q,actual_dq,tau_est,integral_nm,start_blend_s\n";
      }
      else
        RCLCPP_WARN(get_logger(), "Cannot open waist command log in %s", directory.c_str());
    }
    last_motion_log_error_.clear();
    last_motion_log_end_reason_.clear();
    motion_log_rows_ = 0;
    motion_log_ << "ros_time_s,trajectory_progress_s,time_scale,phase,tracking_error_rad,event,"
                   "control_gap_ms,lowstate_steady_age_ms,"
                   "waist_integral_torque_nm,waist_integral_gain_scale,"
                   "waist_integral_mode,waist_integral_saturated,task_stage_profile,task_stage_revision,"
                   "task_stage_proximal_kp,task_stage_proximal_kd,"
                   "task_stage_wrist_roll_kp,task_stage_wrist_pitch_kp,task_stage_wrist_yaw_kp";
    for (std::size_t j = 0; j < trajectory_.joint_names.size(); ++j)
    {
      const auto& name = trajectory_.joint_names[j];
      motion_log_ << "," << name << "_target," << name << "_target_dq,"
                  << name << "_target_ddq," << name << "_command_dq,"
                  << name << "_command_ddq,"
                  << name << "_q," << name << "_dq," << name << "_tau_est,"
                  << name << "_arm_integral_nm," << name << "_arm_integral_mode";
    }
    for (std::size_t j = 0; j < kArmJoints.size(); ++j)
    {
      if (std::find(active_arm_indices_.begin(), active_arm_indices_.end(), j) != active_arm_indices_.end()) continue;
      const auto& name = kArmJoints[j].name;
      motion_log_ << ',' << name << "_target," << name << "_target_dq," << name << "_target_ddq,"
                  << name << "_command_dq," << name << "_command_ddq," << name << "_q,"
                  << name << "_dq," << name << "_tau_est," << name << "_arm_integral_nm,"
                  << name << "_arm_integral_mode";
    }
    motion_log_ << '\n';
    TrajectorySample accepted;
    accepted.positions = trajectory_start_positions_;
    accepted.velocities.assign(trajectory_start_positions_.size(), 0.0);
    accepted.accelerations.assign(trajectory_start_positions_.size(), 0.0);
    write_motion_log_locked(stamp, accepted, 0.0, "accepted");
  }

  void write_motion_log_locked(
    const rclcpp::Time& stamp, const TrajectorySample& sample,
    double time_scale, const std::string& event)
  {
    if (!motion_log_.is_open() || active_arm_indices_.size() != sample.positions.size() ||
        sample.velocities.size() != sample.positions.size() ||
        sample.accelerations.size() != sample.positions.size())
      return;
    motion_log_ << std::fixed << std::setprecision(9)
                << stamp.seconds() << ',' << trajectory_progress_s_ << ',' << time_scale << ','
                << csv_string(phase_) << ',' << current_tracking_error_rad_ << ','
                << csv_string(event) << ',' << current_control_gap_ms_ << ','
                << std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - last_state_steady_).count() << ','
                << current_waist_integral_torque_nm_ << ','
                << waist_integral_gain_scale_ << ',' << csv_string(waist_integral_mode_) << ','
                << (waist_integral_saturated_ ? "true" : "false") << ','
                << csv_string(stage_profile_) << ',' << stage_revision_ << ','
                << position_gain_for_motor(stage_first_motor_) << ','
                << damping_gain_for_motor(stage_first_motor_) << ','
                << position_gain_for_motor(stage_first_motor_+4) << ','
                << position_gain_for_motor(stage_first_motor_+5) << ','
                << position_gain_for_motor(stage_first_motor_+6);
    for (std::size_t j = 0; j < sample.positions.size(); ++j)
    {
      const auto arm_index = active_arm_indices_[j];
      const auto motor_index = trajectory_joint(arm_index).motor_index;
      motion_log_ << ',' << sample.positions[j]
                  << ',' << sample.velocities[j]
                  << ',' << sample.accelerations[j]
                  << ',' << bounded_trajectory_velocity_locked(j, sample.velocities[j], time_scale)
                  << ',' << (event != "sample" ? 0.0 :
                    sample.accelerations[j] * time_scale * time_scale +
                    sample.velocities[j] * trajectory_time_scale_rate_)
                  << ',' << actual_motor_position_[motor_index]
                  << ',' << actual_motor_velocity_[motor_index]
                  << ',' << actual_motor_tau_est_[motor_index]
                  << ',' << arm_integral_torque_nm_[motor_index]
                  << ',' << csv_string(g1_task::waist_integral_mode_name(arm_integral_modes_[motor_index]));
    }
    for (std::size_t j = 0; j < kArmJoints.size(); ++j)
    {
      if (std::find(active_arm_indices_.begin(), active_arm_indices_.end(), j) != active_arm_indices_.end()) continue;
      const auto index = kArmJoints[j].motor_index;
      motion_log_ << ',' << hold_motor_position_[index] << ",0,0,0,0,"
                  << actual_motor_position_[index] << ',' << actual_motor_velocity_[index] << ','
                  << actual_motor_tau_est_[index] << ',' << arm_integral_torque_nm_[index] << ','
                  << csv_string(g1_task::waist_integral_mode_name(arm_integral_modes_[index]));
    }
    motion_log_ << '\n';
    ++motion_log_rows_;
    if (motion_log_rows_ % 25 == 0)
      motion_log_.flush();
  }

  void close_motion_log_locked(const std::string& reason)
  {
    motion_log_hold_remaining_s_ = 0.0;
    if (waist_command_log_.is_open()) waist_command_log_.close();
    if (!motion_log_.is_open())
      return;
    TrajectorySample final_sample;
    final_sample.positions = previous_target_;
    final_sample.velocities = previous_target_velocity_;
    final_sample.accelerations = previous_target_acceleration_;
    write_motion_log_locked(now(), final_sample, 0.0, reason);
    motion_log_.flush();
    motion_log_.close();
    last_motion_log_end_reason_ = reason;
  }

  void enter_damped_recovery_locked(
    const rclcpp::Time& stamp, const std::string& reason)
  {
    if (velocity_recovery_active_ || releasing_ || safety_latched_)
      return;
    restore_stage_locked();
    task_zeroed_ = false;
    task_compliance_blend_ = 0;
    task_resume_blend_ = false;
    task_admittance_.reset();
    close_motion_log_locked(reason);
    reset_integral_locked();
    active_trajectory_ = false;
    if (active_follow_goal_ && active_follow_goal_->is_active())
    {
      auto result = std::make_shared<FollowJointTrajectory::Result>();
      result->error_code = FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED;
      result->error_string = reason + "; hardware output remains active in damped recovery";
      active_follow_goal_->abort(result);
    }
    active_follow_goal_.reset();
    hold_motor_position_ = actual_motor_position_;
    velocity_recovery_active_ = true;
    velocity_recovery_started_time_ = stamp;
    velocity_recovery_settle_started_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    velocity_recovery_timed_out_ = false;
    velocity_recovery_reason_ = reason;
    phase_ = "VELOCITY_RECOVERY";
    RCLCPP_WARN(
      get_logger(),
      "Entering damped recovery: %s; max velocity at %s (motor %zu, dq=%.6f rad/s); "
      "/arm_sdk remains active", reason.c_str(), current_max_velocity_joint_name_.c_str(),
      current_max_velocity_motor_index_, current_max_measured_velocity_rad_s_);
  }

  void update_velocity_recovery_locked(const rclcpp::Time& stamp)
  {
    if (!velocity_recovery_active_ || safety_latched_)
      return;
    hold_motor_position_ = actual_motor_position_;
    const double recovery_elapsed = (stamp - velocity_recovery_started_time_).seconds();
    if (current_max_measured_velocity_rad_s_ > velocity_recovery_hard_limit_rad_s_)
    {
      latch_locked("velocity recovery exceeded hard limit; output is silent");
      return;
    }
    if (recovery_elapsed > velocity_recovery_timeout_s_ && !velocity_recovery_timed_out_)
    {
      velocity_recovery_timed_out_ = true;
      RCLCPP_ERROR(
        get_logger(),
        "Damped recovery has not settled after %.2f s; continuing low-stiffness support",
        recovery_elapsed);
    }
    if (current_max_measured_velocity_rad_s_ <= velocity_recovery_exit_rad_s_ &&
        !torque_overload_active_ && !command_overload_active_)
    {
      if (velocity_recovery_settle_started_time_.nanoseconds() == 0)
        velocity_recovery_settle_started_time_ = stamp;
      else if ((stamp - velocity_recovery_settle_started_time_).seconds() >=
        velocity_recovery_settle_s_)
      {
        velocity_recovery_active_ = false;
        velocity_recovery_timed_out_ = false;
        hold_motor_position_ = actual_motor_position_;
        measured_velocity_exceeding_ = false;
        measured_velocity_exceed_duration_s_ = 0.0;
        phase_ = "ARMED_HOLD";
        RCLCPP_WARN(get_logger(), "Damped velocity recovery settled; returning to ARMED_HOLD");
      }
    }
    else
    {
      velocity_recovery_settle_started_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }
  }

  void state_callback(const unitree_hg::msg::LowState::SharedPtr message)
  {
    const auto entered = std::chrono::steady_clock::now();
    state_callback_entry_ns_.store(
      std::chrono::duration_cast<std::chrono::nanoseconds>(entered.time_since_epoch()).count());
    std::lock_guard<std::mutex> lock(mutex_);
    state_lock_wait_ms_ = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - entered).count();
    if (state_received_)
      state_callback_gap_ms_ = std::chrono::duration<double, std::milli>(
        entered - last_state_steady_).count();
    max_state_callback_gap_ms_ = std::max(max_state_callback_gap_ms_, state_callback_gap_ms_);
    const auto callback_time = now();
    const bool record_tau_anchor_sample =
      last_tau_history_time_.nanoseconds() == 0 ||
      (callback_time - last_tau_history_time_).seconds() >= 0.018;
    keys_ = static_cast<std::uint16_t>(message->wireless_remote[2]) |
      (static_cast<std::uint16_t>(message->wireless_remote[3]) << 8U);
    mode_machine_ = message->mode_machine;
    mode_pr_ = message->mode_pr;
    bool valid = true;
    upper_body_state_safe_ = true;
    upper_body_fault_reason_.clear();
    bool upper_body_modes_active = true;
    std::size_t first_inactive_motor_index = 0;
    std::uint8_t first_inactive_motor_mode = 0;
    current_max_measured_velocity_rad_s_ = 0.0;
    current_max_velocity_motor_index_ = 0;
    current_max_estimated_arm_torque_nm_ = 0.0;
    current_max_estimated_waist_torque_nm_ = 0.0;
    current_max_motor_temperature_c_ = 0.0;
    for (std::size_t i = 0; i < kArmJoints.size(); ++i)
    {
      const auto& motor = message->motor_state[kArmJoints[i].motor_index];
      actual_arm_position_[i] = motor.q;
      actual_arm_velocity_[i] = motor.dq;
      actual_motor_velocity_[kArmJoints[i].motor_index] = motor.dq;
      actual_motor_tau_est_[kArmJoints[i].motor_index] = motor.tau_est;
      arm_motor_mode_[i] = motor.mode;
      if (record_tau_anchor_sample)
      {
        auto& tau_samples = tau_est_history_[kArmJoints[i].motor_index];
        tau_samples.push_back(static_cast<double>(motor.tau_est));
        while (tau_samples.size() > tau_anchor_sample_count_)
          tau_samples.pop_front();
      }
      const double temperature = std::max(motor.temperature[0], motor.temperature[1]);
      const double abs_velocity = std::abs(static_cast<double>(motor.dq));
      if (abs_velocity > current_max_measured_velocity_rad_s_)
      {
        current_max_measured_velocity_rad_s_ = abs_velocity;
        current_max_velocity_motor_index_ = kArmJoints[i].motor_index;
        current_max_velocity_joint_name_ = kArmJoints[i].name;
      }
      current_max_estimated_arm_torque_nm_ = std::max(
        current_max_estimated_arm_torque_nm_, std::abs(static_cast<double>(motor.tau_est)));
      current_max_motor_temperature_c_ = std::max(current_max_motor_temperature_c_, temperature);
      valid = valid && std::isfinite(motor.q) && std::isfinite(motor.dq) &&
        std::isfinite(motor.tau_est) && motor.vol > 1.0F && motor.motorstate == 0U;
      if (upper_body_modes_active && motor.mode != required_upper_body_motor_mode_)
      {
        upper_body_modes_active = false;
        first_inactive_motor_index = kArmJoints[i].motor_index;
        first_inactive_motor_mode = motor.mode;
      }
    }
    for (const auto configured_index : controlled_waist_indices_)
    {
      const auto index = static_cast<std::size_t>(configured_index);
      const auto& motor = message->motor_state[index];
      actual_motor_position_[index] = motor.q;
      actual_motor_velocity_[index] = motor.dq;
      actual_motor_tau_est_[index] = motor.tau_est;
      const double temperature = std::max(motor.temperature[0], motor.temperature[1]);
      const double abs_velocity = std::abs(static_cast<double>(motor.dq));
      if (abs_velocity > current_max_measured_velocity_rad_s_)
      {
        current_max_measured_velocity_rad_s_ = abs_velocity;
        current_max_velocity_motor_index_ = index;
        current_max_velocity_joint_name_ = "waist_joint_motor_" + std::to_string(index);
      }
      current_max_estimated_waist_torque_nm_ = std::max(
        current_max_estimated_waist_torque_nm_, std::abs(static_cast<double>(motor.tau_est)));
      current_max_motor_temperature_c_ = std::max(current_max_motor_temperature_c_, temperature);
      valid = valid && std::isfinite(motor.q) && std::isfinite(motor.dq) &&
        std::isfinite(motor.tau_est) && motor.vol > 1.0F && motor.motorstate == 0U;
      if (upper_body_modes_active && motor.mode != required_upper_body_motor_mode_)
      {
        upper_body_modes_active = false;
        first_inactive_motor_index = index;
        first_inactive_motor_mode = motor.mode;
      }
    }
    for (std::size_t i = 0; i < kArmJoints.size(); ++i)
      actual_motor_position_[kArmJoints[i].motor_index] = actual_arm_position_[i];
    for (std::size_t axis = 0; axis < 3; ++axis)
      current_imu_rpy_[axis] = message->imu_state.rpy[axis];
    state_received_ = true;
    last_state_time_ = callback_time;
    // Lock wait must not make an old callback appear newly received.
    last_state_steady_ = entered;
    if (record_tau_anchor_sample)
      last_tau_history_time_ = callback_time;

    if (current_max_measured_velocity_rad_s_ > max_measured_velocity_rad_s_)
    {
      if (!measured_velocity_exceeding_)
      {
        measured_velocity_exceeding_ = true;
        measured_velocity_exceed_start_time_ = last_state_time_;
      }
      measured_velocity_exceed_duration_s_ =
        (last_state_time_ - measured_velocity_exceed_start_time_).seconds();
    }
    else
    {
      measured_velocity_exceeding_ = false;
      measured_velocity_exceed_duration_s_ = 0.0;
    }

    bool arm_torque_peak_exceeded = false;
    std::string arm_torque_joint = "none";
    std::size_t arm_torque_motor = 0;
    double arm_torque_value = 0.0;
    double arm_torque_continuous_limit = 0.0;
    double arm_torque_peak_limit = 0.0;
    double worst_torque_ratio = 0.0;
    bool arm_torque_continuous_exceeded = false;
    for (const auto& joint : kArmJoints)
    {
      const double magnitude = std::abs(actual_motor_tau_est_[joint.motor_index]);
      const double continuous_limit = continuous_torque_limit_for_motor(joint.motor_index);
      const double peak_limit = peak_torque_limit_for_motor(joint.motor_index);
      const double ratio = continuous_limit > 0.0 ? magnitude / continuous_limit : 0.0;
      if (ratio > worst_torque_ratio)
      {
        worst_torque_ratio = ratio;
        arm_torque_joint = joint.name;
        arm_torque_motor = joint.motor_index;
        arm_torque_value = magnitude;
        arm_torque_continuous_limit = continuous_limit;
        arm_torque_peak_limit = peak_limit;
      }
      arm_torque_continuous_exceeded =
        arm_torque_continuous_exceeded || magnitude > continuous_limit;
      arm_torque_peak_exceeded = arm_torque_peak_exceeded || magnitude > peak_limit;
    }
    if (arm_torque_continuous_exceeded)
    {
      if (!torque_overload_active_)
      {
        torque_overload_active_ = true;
        torque_overload_start_time_ = callback_time;
      }
      torque_overload_duration_s_ = (callback_time - torque_overload_start_time_).seconds();
      torque_overload_reason_ =
        "arm torque overload at " + arm_torque_joint + " (motor " +
        std::to_string(arm_torque_motor) + ", |tau_est|=" +
        std::to_string(arm_torque_value) + " Nm, continuous=" +
        std::to_string(arm_torque_continuous_limit) + " Nm, peak=" +
        std::to_string(arm_torque_peak_limit) + " Nm, duration=" +
        std::to_string(torque_overload_duration_s_) + " s)";
    }
    else
    {
      torque_overload_active_ = false;
      torque_overload_duration_s_ = 0.0;
      torque_overload_reason_.clear();
    }

    temperature_alarm_active_ = current_max_motor_temperature_c_ >= max_motor_temperature_c_;
    if (temperature_alarm_active_)
    {
      temperature_alarm_reason_ =
        "upper-body temperature alarm: " +
        std::to_string(current_max_motor_temperature_c_) + " C >= " +
        std::to_string(max_motor_temperature_c_) +
        " C; support remains active so the arm can move to a low-load pose";
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "%s", temperature_alarm_reason_.c_str());
    }
    else
    {
      temperature_alarm_reason_.clear();
    }

    if (!upper_body_modes_active)
    {
      upper_body_state_safe_ = false;
      upper_body_fault_reason_ =
        "upper-body motor is not active: index " + std::to_string(first_inactive_motor_index) +
        " mode " + std::to_string(first_inactive_motor_mode) +
        " (required " + std::to_string(required_upper_body_motor_mode_) + ")";
    }
    else if (current_max_measured_velocity_rad_s_ > measured_velocity_hard_limit_rad_s_)
    {
      upper_body_state_safe_ = false;
      upper_body_fault_reason_ =
        "measured upper-body velocity exceeded instantaneous hard limit at " +
        current_max_velocity_joint_name_ + " (motor " +
        std::to_string(current_max_velocity_motor_index_) + ", dq=" +
        std::to_string(current_max_measured_velocity_rad_s_) + " rad/s)";
    }
    else if (measured_velocity_exceed_duration_s_ >= measured_velocity_persistence_s_)
    {
      if (armed_ && !releasing_)
      {
        enter_damped_recovery_locked(
          callback_time,
          "sustained measured velocity exceeded the normal operating limit");
      }
      else if (releasing_)
      {
        hold_motor_position_ = actual_motor_position_;
      }
      else
      {
        upper_body_state_safe_ = false;
        upper_body_fault_reason_ =
          "measured upper-body velocity persistently exceeded limit at " +
          current_max_velocity_joint_name_ + " (motor " +
          std::to_string(current_max_velocity_motor_index_) + ", dq=" +
          std::to_string(current_max_measured_velocity_rad_s_) + " rad/s, duration=" +
          std::to_string(measured_velocity_exceed_duration_s_) + " s)";
      }
    }
    else if (arm_torque_peak_exceeded ||
      torque_overload_duration_s_ >= torque_peak_duration_s_)
    {
      if (armed_ && !releasing_)
        enter_damped_recovery_locked(callback_time, torque_overload_reason_);
      else if (!releasing_)
      {
        upper_body_state_safe_ = false;
        upper_body_fault_reason_ = torque_overload_reason_;
      }
    }
    else if (current_max_estimated_waist_torque_nm_ > max_estimated_waist_torque_nm_)
    {
      upper_body_state_safe_ = false;
      upper_body_fault_reason_ = "estimated waist torque exceeded hard limit";
    }
    if (current_max_measured_velocity_rad_s_ > peak_measured_velocity_rad_s_)
    {
      peak_measured_velocity_rad_s_ = current_max_measured_velocity_rad_s_;
      peak_velocity_motor_index_ = current_max_velocity_motor_index_;
      peak_velocity_joint_name_ = current_max_velocity_joint_name_;
    }
    peak_estimated_arm_torque_nm_ = std::max(
      peak_estimated_arm_torque_nm_, current_max_estimated_arm_torque_nm_);
    peak_estimated_waist_torque_nm_ = std::max(
      peak_estimated_waist_torque_nm_, current_max_estimated_waist_torque_nm_);

    if (!valid)
      latch_locked("invalid or inactive upper-body motor state");
    else if ((keys_ & kL2BMask) == kL2BMask)
      latch_locked("native L2+B observed; hardware output is silent");
    else if (armed_ && !upper_body_state_safe_)
      latch_locked(upper_body_fault_reason_);
    else if (velocity_recovery_active_)
      update_velocity_recovery_locked(callback_time);
  }

  bool validate_trajectory_locked(
    const trajectory_msgs::msg::JointTrajectory& trajectory,
    std::vector<std::size_t>& arm_indices, std::string& error) const
  {
    if (stage_switching_locked()) { error="Stage gains still blending"; return false; }
    if (task_application_enabled_ && hardware_output_available_ &&
        rclcpp::Time(trajectory.header.stamp).nanoseconds() < last_task_stop_stamp_.load())
    {
      error = "trajectory predates the latest task stop; new stamped plan required";
      return false;
    }
    const auto count = trajectory.joint_names.size();
    const bool dual = task_dual_prepare_enabled_ && task_application_enabled_ &&
      task_waist_enabled_ && count == 15 && stage_profile_ == "normal";
    if ((count != 7 && !(task_waist_enabled_ && count == 8) && !dual) || trajectory.points.empty())
    {
      error = "trajectory requires seven arm joints, plus optional enabled task waist yaw";
      return false;
    }
    std::set<std::string> unique_names;
    bool side_initialized = false;
    bool left = false;
    std::size_t arm_count = 0, waist_count = 0;
    arm_indices.clear();
    for (const auto& name : trajectory.joint_names)
    {
      std::size_t index = 0;
      const auto* spec = find_joint(name, index);
      if (!spec || !unique_names.insert(name).second)
      {
        error = "unknown, non-arm, or duplicate joint: " + name;
        return false;
      }
      if (spec->motor_index == 12) {
        if (!task_waist_enabled_) { error = "task waist trajectories are disabled"; return false; }
        ++waist_count;
        arm_indices.push_back(index);
        continue;
      }
      ++arm_count;
      if (stage_profile_!="normal" && spec->left!=(stage_first_motor_==15))
      { error="Trajectory arm differs from active stage profile"; return false; }
      if (!side_initialized)
      {
        left = spec->left;
        side_initialized = true;
      }
      else if (left != spec->left && !dual)
      {
        error = "trajectory mixes left and right arm joints";
        return false;
      }
      arm_indices.push_back(index);
    }
    if (arm_count != (dual ? 14U : 7U) || waist_count != count - arm_count) {
      error = "exactly seven same-side arm joints are required; waist cannot replace an arm joint";
      return false;
    }
    if (!task_application_enabled_ && left != (active_arm_ == "left"))
    {
      error = "trajectory does not match configured active arm " + active_arm_;
      return false;
    }

    double previous_time = -1.0;
    std::vector<double> previous_segment_velocity(count, 0.0);
    bool have_previous_segment = false;
    double previous_dt = 0.0;
    for (std::size_t p = 0; p < trajectory.points.size(); ++p)
    {
      const auto& point = trajectory.points[p];
      if (point.positions.size() != count ||
          (!point.velocities.empty() && point.velocities.size() != count) ||
          (!point.accelerations.empty() && point.accelerations.size() != count))
      {
        error = "trajectory point arrays must match joint names";
        return false;
      }
      if (hardware_output_available_ &&
          (point.velocities.size() != count || point.accelerations.size() != count))
      {
        error = "hardware trajectories require complete velocity and acceleration arrays";
        return false;
      }
      const double time = point_time(point);
      if (!std::isfinite(time) || time < 0.0 || (p > 0 && time <= previous_time))
      {
        error = "time_from_start must be finite, nonnegative, and strictly increasing";
        return false;
      }
      for (std::size_t j = 0; j < count; ++j)
      {
        const auto& spec = trajectory_joint(arm_indices[j]);
        const double vmax = spec.motor_index == 12 ? task_waist_velocity_limit_rad_s_ : max_velocity_rad_s_;
        const double amax = max_acceleration_rad_s2_;
        if (!std::isfinite(point.positions[j]) ||
            point.positions[j] < spec.lower + joint_limit_margin_rad_ ||
            point.positions[j] > spec.upper - joint_limit_margin_rad_)
        {
          error = "position violates guarded limit: " + std::string(spec.name);
          return false;
        }
        if (!point.velocities.empty() &&
            (!std::isfinite(point.velocities[j]) ||
             std::abs(point.velocities[j]) > vmax + 1e-6))
        {
          error = "declared velocity exceeds limit: " + std::string(spec.name);
          return false;
        }
        if (!point.accelerations.empty() &&
            (!std::isfinite(point.accelerations[j]) ||
             std::abs(point.accelerations[j]) > amax + 1e-6))
        {
          error = "declared acceleration exceeds limit: " + std::string(spec.name);
          return false;
        }
      }

      if (p > 0)
      {
        const double dt = time - previous_time;
        std::vector<double> segment_velocity(count, 0.0);
        for (std::size_t j = 0; j < count; ++j)
        {
          const auto& spec = trajectory_joint(arm_indices[j]);
          const double vmax = spec.motor_index == 12 ? task_waist_velocity_limit_rad_s_ : max_velocity_rad_s_;
          const double amax = max_acceleration_rad_s2_;
          if (spec.motor_index == 12) {
            const auto& first = trajectory.points[p-1];
            const bool velocities = first.velocities.size()==count && point.velocities.size()==count;
            const bool accelerations = first.accelerations.size()==count && point.accelerations.size()==count;
            if (!g1_task::waist_segment_safe(first.positions[j],point.positions[j],
                velocities ? first.velocities[j] : 0., velocities ? point.velocities[j] : 0.,
                accelerations ? first.accelerations[j] : 0., accelerations ? point.accelerations[j] : 0.,
                dt,velocities,accelerations,spec.lower+joint_limit_margin_rad_,
                spec.upper-joint_limit_margin_rad_,vmax,amax)) {
              error="waist Hermite position/velocity/acceleration envelope exceeds limits";
              return false;
            }
          }
          segment_velocity[j] =
            (point.positions[j] - trajectory.points[p - 1].positions[j]) / dt;
          if (!std::isfinite(segment_velocity[j]) ||
              std::abs(segment_velocity[j]) > vmax + 1e-6)
          {
            error = "segment velocity exceeds limit: " +
              std::string(spec.name);
            return false;
          }
          if (have_previous_segment)
          {
            const double acceleration =
              2.0 * (segment_velocity[j] - previous_segment_velocity[j]) /
              (dt + previous_dt);
            if (!std::isfinite(acceleration) ||
                std::abs(acceleration) > amax + 1e-6)
            {
              error = "derived segment acceleration exceeds limit: " +
                std::string(spec.name);
              return false;
            }
          }
        }
        previous_segment_velocity = segment_velocity;
        previous_dt = dt;
        have_previous_segment = true;
      }
      previous_time = time;
    }
    if (point_time(trajectory.points.back()) <= 0.0)
    {
      error = "trajectory duration must be positive";
      return false;
    }
    if (hardware_output_available_ && std::abs(point_time(trajectory.points.front())) > 1e-9)
    {
      error = "hardware trajectory must start at time zero";
      return false;
    }
    if (hardware_output_available_)
    {
      for (const auto* endpoint : {&trajectory.points.front(), &trajectory.points.back()})
      {
        for (std::size_t j = 0; j < count; ++j)
        {
          if (std::abs(endpoint->velocities[j]) > 1e-6 ||
              std::abs(endpoint->accelerations[j]) > 1e-6)
          {
            error = "hardware trajectory boundary velocity and acceleration must be zero";
            return false;
          }
        }
      }
    }
    for (std::size_t j = 0; j < count; ++j)
    {
      const auto& spec = trajectory_joint(arm_indices[j]);
      if (armed_)
      {
        const double held = hold_motor_position_[spec.motor_index];
        if (!std::isfinite(held) || held < spec.lower + joint_limit_margin_rad_ ||
            held > spec.upper - joint_limit_margin_rad_ ||
            std::abs(held - actual_motor_position_[spec.motor_index]) > start_tolerance_rad_ ||
            std::abs(held - trajectory.points.front().positions[j]) > start_tolerance_rad_)
        {
          error = "held reference cannot safely blend to trajectory start: " + std::string(spec.name);
          return false;
        }
      }
      if (std::abs(
          trajectory.points.front().positions[j] - actual_motor_position_[trajectory_joint(arm_indices[j]).motor_index]) >
          start_tolerance_rad_)
      {
        error = "trajectory start differs from measured state: " +
          std::string(trajectory_joint(arm_indices[j]).name);
        return false;
      }
    }
    return true;
  }

  void trajectory_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (task_stop_requested_.load() || task_stop_active_ || task_resume_blend_ || waist_mode_switching_) return;
    ++received_trajectories_;
    if (safety_latched_ || configuration_blocked_)
    {
      ++rejected_trajectories_;
      last_rejection_ = safety_latched_ ? latch_reason_ : "hardware authorization is invalid";
      return;
    }
    if (!state_is_fresh_locked())
    {
      ++rejected_trajectories_;
      latch_locked("trajectory received without fresh LowState");
      return;
    }
    if (hardware_output_available_ && !armed_)
    {
      ++rejected_trajectories_;
      last_rejection_ = "hardware gateway is not armed";
      return;
    }
    if (hardware_output_available_ && phase_ != "ARMED_HOLD")
    {
      ++rejected_trajectories_;
      last_rejection_ =
        "takeover tau-anchor/gravity-feedforward transition is not complete";
      return;
    }
    if (active_trajectory_)
    {
      ++rejected_trajectories_;
      last_rejection_ = "another trajectory is already active";
      return;
    }
    std::vector<std::size_t> indices;
    std::string error;
    if (!validate_trajectory_locked(*message, indices, error))
    {
      ++rejected_trajectories_;
      last_rejection_ = error;
      RCLCPP_ERROR(get_logger(), "Rejected arm trajectory: %s", error.c_str());
      return;
    }
    start_trajectory_locked(*message, std::move(indices));
  }

  void start_trajectory_locked(
    const trajectory_msgs::msg::JointTrajectory& trajectory,
    std::vector<std::size_t> indices)
  {
    // Ordinary segment transitions keep the support torque and held reference.
    // STOP/cancel/recovery still capture measured hold and explicitly reset I.
    close_motion_log_locked("next trajectory");
    trajectory_ = trajectory;
    active_arm_indices_ = std::move(indices);
    const auto count = active_arm_indices_.size();
    trajectory_start_positions_.resize(count);
    for (std::size_t j = 0; j < count; ++j)
      trajectory_start_positions_[j] = (armed_ ? hold_motor_position_ : actual_motor_position_)
        [trajectory_joint(active_arm_indices_[j]).motor_index];
    double max_start_delta = 0.0;
    for (std::size_t j = 0; j < count; ++j)
      max_start_delta = std::max(
        max_start_delta,
        std::abs(trajectory_.points.front().positions[j] - trajectory_start_positions_[j]));
    const double step_limited_velocity = max_command_step_rad_ * control_rate_hz_ * 0.95;
    const double blend_velocity = std::min({max_velocity_rad_s_, step_limited_velocity,
                                          (count == 8 || count == 15) ? task_waist_velocity_limit_rad_s_ : max_velocity_rad_s_});
    const double blend_acceleration = max_acceleration_rad_s2_;
    constexpr double kQuinticPeakVelocity = 1.875;
    constexpr double kQuinticPeakAcceleration = 5.773502691896258;
    trajectory_start_blend_s_ = max_start_delta > 1e-9 ? std::max(
      kQuinticPeakVelocity * max_start_delta / blend_velocity,
      std::sqrt(kQuinticPeakAcceleration * max_start_delta / blend_acceleration)) : 0.0;
    trajectory_first_point_time_s_ = point_time(trajectory_.points.front());
    trajectory_total_duration_s_ = trajectory_start_blend_s_ +
      point_time(trajectory_.points.back()) - trajectory_first_point_time_s_;
    previous_target_ = trajectory_start_positions_;
    previous_target_velocity_.assign(count, 0.0);
    previous_target_acceleration_.assign(count, 0.0);
    target_initialized_ = true;
    trajectory_progress_s_ = 0.0;
    previous_trajectory_time_scale_ = 1.0;
    trajectory_time_scale_rate_ = 0.0;
    active_trajectory_ = true;
    last_rejection_.clear();
    current_tracking_error_rad_ = 0.0;
    peak_tracking_error_rad_ = 0.0;
    phase_ = hardware_output_available_ ? "RUNNING" : "DRY_RUN_TRAJECTORY";
    open_motion_log_locked(now());
    ++accepted_trajectories_;
    RCLCPP_WARN(
      get_logger(),
      "%s accepted %s-arm trajectory: %zu points, %.3f s plus %.3f s start blend",
      hardware_output_available_ ? "ARMED gateway" : "DRY RUN",
      std::any_of(active_arm_indices_.begin(), active_arm_indices_.end(),
                  [](auto i) { return i < 7; }) ? "left" : "right",
      trajectory_.points.size(), point_time(trajectory_.points.back()),
      trajectory_start_blend_s_);
  }

  bool validate_waist_hold_start_locked(
    const trajectory_msgs::msg::JointTrajectory& trajectory, std::string& error) const
  {
    // Called after the ordinary trajectory validator, again at action acceptance.
    if (!task_waist_enabled_ || trajectory.joint_names.size() != 8 ||
        !state_is_fresh_locked() || !service_state_is_ready_locked() ||
        !upper_body_state_safe_ || current_max_measured_velocity_rad_s_ > .03 ||
        torque_overload_active_ || command_overload_active_ || temperature_alarm_active_)
    {
      error = "waist-only motion requires fresh healthy stationary hold";
      return false;
    }
    for (std::size_t j = 0; j < trajectory.joint_names.size(); ++j)
    {
      std::size_t unused{};
      const auto* spec = find_joint(trajectory.joint_names[j], unused);
      if (!spec || (spec->motor_index != 12 && spec->left))
      {
        error = "waist-only motion requires right-arm hold and waist yaw";
        return false;
      }
      const auto index = spec->motor_index;
      if (std::abs(hold_motor_position_[index] - trajectory.points.front().positions[j]) > 1e-12 ||
          std::abs(hold_motor_position_[index] - actual_motor_position_[index]) > .025)
      {
        error = "waist hold reference changed or differs from actual by more than 0.025 rad; replan";
        return false;
      }
      if (index == 12) continue;
      for (const auto& point : trajectory.points)
        if (std::abs(point.positions[j] - hold_motor_position_[index]) > 1e-12 ||
            point.velocities.size() != 8 || point.accelerations.size() != 8 ||
            std::abs(point.velocities[j]) > 1e-12 || std::abs(point.accelerations[j]) > 1e-12)
        {
          error = "waist-only motion must preserve every right-arm hold reference";
          return false;
        }
    }
    return true;
  }

  rclcpp_action::GoalResponse follow_goal_callback(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal, bool waist_only = false)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (task_stop_requested_.load() || task_stop_active_ || task_resume_blend_ ||
        !hardware_output_available_ || !armed_ || safety_latched_ ||
        phase_ != "ARMED_HOLD" || active_trajectory_ || waist_mode_switching_ ||
        !state_is_fresh_locked() || !service_state_is_ready_locked())
    {
      last_rejection_ = "gateway not ready for trajectory; phase=" + phase_;
      ++rejected_trajectories_;
      RCLCPP_WARN(get_logger(), "Trajectory rejected: %s", last_rejection_.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    std::vector<std::size_t> indices;
    std::string error;
    if (!validate_trajectory_locked(goal->trajectory, indices, error) ||
        (waist_only && !validate_waist_hold_start_locked(goal->trajectory, error)) ||
        indices.empty())
    {
      last_rejection_ = error.empty() ? "trajectory has no active arm joints" : error;
      ++rejected_trajectories_;
      RCLCPP_WARN(get_logger(), "Trajectory rejected: %s", last_rejection_.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    goal_stop_epochs_[uuid] = task_stop_epoch_.load();
    stage_goal_revisions_[uuid] = stage_revision_;
    if (waist_only) waist_goal_revisions_[uuid] = waist_control_revision_;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse follow_cancel_callback(
    const std::shared_ptr<FollowGoalHandle> goal_handle)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (task_stop_requested_.load() || task_stop_active_ || velocity_recovery_active_)
      return rclcpp_action::CancelResponse::REJECT;
    if (!active_follow_goal_ || goal_handle != active_follow_goal_)
      return rclcpp_action::CancelResponse::REJECT;
    close_motion_log_locked("cancelled by MoveIt");
    active_trajectory_ = false;
    reset_integral_locked();
    hold_motor_position_ = actual_motor_position_;
    // rclcpp_action enters CANCELING only after this callback returns ACCEPT.
    // Stop output progression now; finish the action on the next control tick.
    phase_ = "ACTION_CANCEL_HOLD";
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void follow_accepted_callback(const std::shared_ptr<FollowGoalHandle> goal_handle, bool waist_only = false)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++received_trajectories_;
    std::vector<std::size_t> indices;
    std::string error;
    const auto& trajectory = goal_handle->get_goal()->trajectory;
    const auto epoch = goal_stop_epochs_.find(goal_handle->get_goal_id());
    const auto stage_revision=stage_goal_revisions_.find(goal_handle->get_goal_id());
    const bool current_stage=stage_revision!=stage_goal_revisions_.end() && stage_revision->second==stage_revision_;
    if (stage_revision!=stage_goal_revisions_.end()) stage_goal_revisions_.erase(stage_revision);
    const bool current_epoch = epoch != goal_stop_epochs_.end() &&
      epoch->second == task_stop_epoch_.load();
    if (epoch != goal_stop_epochs_.end()) goal_stop_epochs_.erase(epoch);
    const auto revision = waist_goal_revisions_.find(goal_handle->get_goal_id());
    const bool current_revision = !waist_only ||
      (revision != waist_goal_revisions_.end() && revision->second == waist_control_revision_);
    if (revision != waist_goal_revisions_.end()) waist_goal_revisions_.erase(revision);
    if (!current_epoch || !current_stage || task_stop_requested_.load() || task_stop_active_ || task_resume_blend_ ||
        !hardware_output_available_ || !armed_ || safety_latched_ ||
        phase_ != "ARMED_HOLD" || active_trajectory_ || waist_mode_switching_ ||
        !current_revision || !validate_trajectory_locked(trajectory, indices, error) ||
        (waist_only && !validate_waist_hold_start_locked(trajectory, error)) ||
        indices.empty())
    {
      ++rejected_trajectories_;
      last_rejection_ = error.empty() ? "gateway changed state before RViz goal acceptance" : error;
      auto result = std::make_shared<FollowJointTrajectory::Result>();
      result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
      result->error_string = last_rejection_;
      goal_handle->abort(result);
      return;
    }
    active_follow_goal_ = goal_handle;
    start_trajectory_locked(trajectory, std::move(indices));
  }

  TrajectorySample interpolate_locked(double elapsed) const
  {
    const auto count = active_arm_indices_.size();
    TrajectorySample result;
    result.positions.assign(count, 0.0);
    result.velocities.assign(count, 0.0);
    result.accelerations.assign(count, 0.0);
    if (trajectory_start_blend_s_ > 0.0 && elapsed < trajectory_start_blend_s_)
    {
      const double u = std::clamp(elapsed / trajectory_start_blend_s_, 0.0, 1.0);
      const double u2 = u * u;
      const double u3 = u2 * u;
      const double u4 = u3 * u;
      const double u5 = u4 * u;
      const double blend = 10.0 * u3 - 15.0 * u4 + 6.0 * u5;
      const double blend_velocity = (30.0 * u2 - 60.0 * u3 + 30.0 * u4) /
        trajectory_start_blend_s_;
      const double blend_acceleration = (60.0 * u - 180.0 * u2 + 120.0 * u3) /
        (trajectory_start_blend_s_ * trajectory_start_blend_s_);
      for (std::size_t j = 0; j < count; ++j)
      {
        const double delta =
          trajectory_.points.front().positions[j] - trajectory_start_positions_[j];
        result.positions[j] = trajectory_start_positions_[j] + blend * delta;
        result.velocities[j] = blend_velocity * delta;
        result.accelerations[j] = blend_acceleration * delta;
      }
      return result;
    }

    const double trajectory_elapsed = trajectory_first_point_time_s_ +
      std::max(0.0, elapsed - trajectory_start_blend_s_);
    if (trajectory_elapsed >= point_time(trajectory_.points.back()))
    {
      std::copy_n(
        trajectory_.points.back().positions.begin(), count, result.positions.begin());
      return result;
    }
    std::size_t upper = 0;
    while (upper < trajectory_.points.size() &&
      point_time(trajectory_.points[upper]) < trajectory_elapsed)
      ++upper;
    const double lower_time = upper == 0 ? 0.0 : point_time(trajectory_.points[upper - 1]);
    const double upper_time = point_time(trajectory_.points[upper]);
    const double dt = upper_time - lower_time;
    const double ratio = dt > 0.0 ?
      std::clamp((trajectory_elapsed - lower_time) / dt, 0.0, 1.0) : 1.0;
    for (std::size_t j = 0; j < count; ++j)
    {
      const double lower = upper == 0 ? trajectory_start_positions_[j] :
        trajectory_.points[upper - 1].positions[j];
      const double upper_position = trajectory_.points[upper].positions[j];
      if (upper > 0 && trajectory_.points[upper - 1].velocities.size() == count &&
          trajectory_.points[upper].velocities.size() == count)
      {
        const auto& lower_point = trajectory_.points[upper - 1];
        const auto& upper_point = trajectory_.points[upper];
        const double u2 = ratio * ratio;
        const double u3 = u2 * ratio;
        if (lower_point.accelerations.size() == count &&
            upper_point.accelerations.size() == count)
        {
          const double u4 = u3 * ratio;
          const double u5 = u4 * ratio;
          const double h00 = 1.0 - 10.0 * u3 + 15.0 * u4 - 6.0 * u5;
          const double h10 = ratio - 6.0 * u3 + 8.0 * u4 - 3.0 * u5;
          const double h20 = 0.5 * u2 - 1.5 * u3 + 1.5 * u4 - 0.5 * u5;
          const double h01 = 10.0 * u3 - 15.0 * u4 + 6.0 * u5;
          const double h11 = -4.0 * u3 + 7.0 * u4 - 3.0 * u5;
          const double h21 = 0.5 * u3 - u4 + 0.5 * u5;
          const double dh00 = -30.0 * u2 + 60.0 * u3 - 30.0 * u4;
          const double dh10 = 1.0 - 18.0 * u2 + 32.0 * u3 - 15.0 * u4;
          const double dh20 = ratio - 4.5 * u2 + 6.0 * u3 - 2.5 * u4;
          const double dh01 = 30.0 * u2 - 60.0 * u3 + 30.0 * u4;
          const double dh11 = -12.0 * u2 + 28.0 * u3 - 15.0 * u4;
          const double dh21 = 1.5 * u2 - 4.0 * u3 + 2.5 * u4;
          const double ddh00 = -60.0 * ratio + 180.0 * u2 - 120.0 * u3;
          const double ddh10 = -36.0 * ratio + 96.0 * u2 - 60.0 * u3;
          const double ddh20 = 1.0 - 9.0 * ratio + 18.0 * u2 - 10.0 * u3;
          const double ddh01 = 60.0 * ratio - 180.0 * u2 + 120.0 * u3;
          const double ddh11 = -24.0 * ratio + 84.0 * u2 - 60.0 * u3;
          const double ddh21 = 3.0 * ratio - 12.0 * u2 + 10.0 * u3;
          const double lower_velocity = lower_point.velocities[j];
          const double upper_velocity = upper_point.velocities[j];
          const double lower_acceleration = lower_point.accelerations[j];
          const double upper_acceleration = upper_point.accelerations[j];
          result.positions[j] =
            h00 * lower + h10 * dt * lower_velocity +
            h20 * dt * dt * lower_acceleration + h01 * upper_position +
            h11 * dt * upper_velocity + h21 * dt * dt * upper_acceleration;
          result.velocities[j] =
            dh00 * lower / dt + dh10 * lower_velocity +
            dh20 * dt * lower_acceleration + dh01 * upper_position / dt +
            dh11 * upper_velocity + dh21 * dt * upper_acceleration;
          result.accelerations[j] =
            ddh00 * lower / (dt * dt) + ddh10 * lower_velocity / dt +
            ddh20 * lower_acceleration + ddh01 * upper_position / (dt * dt) +
            ddh11 * upper_velocity / dt + ddh21 * upper_acceleration;
        }
        else
        {
          const double h00 = 2.0 * u3 - 3.0 * u2 + 1.0;
          const double h10 = u3 - 2.0 * u2 + ratio;
          const double h01 = -2.0 * u3 + 3.0 * u2;
          const double h11 = u3 - u2;
          const double dh00 = 6.0 * u2 - 6.0 * ratio;
          const double dh10 = 3.0 * u2 - 4.0 * ratio + 1.0;
          const double dh01 = -6.0 * u2 + 6.0 * ratio;
          const double dh11 = 3.0 * u2 - 2.0 * ratio;
          const double ddh00 = 12.0 * ratio - 6.0;
          const double ddh10 = 6.0 * ratio - 4.0;
          const double ddh01 = -12.0 * ratio + 6.0;
          const double ddh11 = 6.0 * ratio - 2.0;
          const double lower_velocity = lower_point.velocities[j];
          const double upper_velocity = upper_point.velocities[j];
          result.positions[j] = h00 * lower + h10 * dt * lower_velocity +
            h01 * upper_position + h11 * dt * upper_velocity;
          result.velocities[j] = dh00 * lower / dt + dh10 * lower_velocity +
            dh01 * upper_position / dt + dh11 * upper_velocity;
          result.accelerations[j] = ddh00 * lower / (dt * dt) +
            ddh10 * lower_velocity / dt + ddh01 * upper_position / (dt * dt) +
            ddh11 * upper_velocity / dt;
        }
      }
      else
      {
        result.positions[j] = lower + ratio * (upper_position - lower);
        result.velocities[j] = dt > 0.0 ? (upper_position - lower) / dt : 0.0;
      }
    }
    return result;
  }

  void publish_preview_locked(
    const rclcpp::Time& stamp, const TrajectorySample& sample, double time_scale)
  {
    sensor_msgs::msg::JointState preview;
    preview.header.stamp = stamp;
    preview.header.frame_id = hardware_output_available_ ?
      "ARM_SDK_GUARDED_TARGET" : "DRY_RUN_NO_HARDWARE_PUBLISHER";
    preview.name = trajectory_.joint_names;
    preview.position = sample.positions;
    preview.velocity.resize(sample.velocities.size());
    for (std::size_t j = 0; j < sample.velocities.size(); ++j)
      preview.velocity[j] = bounded_trajectory_velocity_locked(j, sample.velocities[j], time_scale);
    preview_pub_->publish(preview);
  }

  std::size_t arm_joint_type_for_motor(std::size_t motor_index) const
  {
    for (std::size_t i = 0; i < kArmJoints.size(); ++i)
      if (kArmJoints[i].motor_index == motor_index)
        return i % 7;
    return 0;
  }

  double continuous_torque_limit_for_motor(std::size_t motor_index) const
  {
    return kArmContinuousTorqueLimits[arm_joint_type_for_motor(motor_index)];
  }

  double peak_torque_limit_for_motor(std::size_t motor_index) const
  {
    return std::min(
      kArmPeakTorqueLimits[arm_joint_type_for_motor(motor_index)],
      max_estimated_arm_torque_nm_);
  }

  double position_gain_for_motor(std::size_t motor_index) const
  {
    if (velocity_recovery_active_)
      return velocity_recovery_kp_;
    const double normal = stage_gain_locked(motor_index, false);
    const double compliant = arm_joint_type_for_motor(motor_index) >= 5 ? 1.5 : 2.0;
    return normal + task_compliance_blend_ * (compliant - normal);
  }

  double damping_gain_for_motor(std::size_t motor_index) const
  {
    if (velocity_recovery_active_)
      return velocity_recovery_kd_;
    const double normal = stage_gain_locked(motor_index, true);
    // Keep damping during transition; do not copy the low-speed drag kd blindly.
    return normal + task_compliance_blend_ * (std::max(normal, 1.6) - normal);
  }

  bool capture_feedforward_anchor_locked(std::string& reason)
  {
    tau_est_anchor_.fill(0.0);
    feedforward_anchor_baseline_.fill(0.0);
    if (!gravity_feedforward_enabled_)
    {
      feedforward_initialized_ = true;
      feedforward_blend_ = 0.0;
      return true;
    }
    if (current_max_measured_velocity_rad_s_ > 0.05)
    {
      reason = "upper body must be stationary before tau_est anchoring (dq=" +
        std::to_string(current_max_measured_velocity_rad_s_) + " rad/s at " +
        current_max_velocity_joint_name_ + ")";
      return false;
    }
    for (const auto& joint : kArmJoints)
    {
      auto values = std::vector<double>(
        tau_est_history_[joint.motor_index].begin(),
        tau_est_history_[joint.motor_index].end());
      if (values.size() < 20)
      {
        reason = "insufficient tau_est history at " + std::string(joint.name);
        return false;
      }
      std::sort(values.begin(), values.end());
      const double median = values[values.size() / 2];
      const double spread = values[values.size() * 9 / 10] - values[values.size() / 10];
      const double limit = continuous_torque_limit_for_motor(joint.motor_index);
      if (!std::isfinite(median) || spread > tau_anchor_max_spread_nm_ ||
          std::abs(median) > limit)
      {
        reason = "unstable/out-of-range tau_est anchor at " + std::string(joint.name) +
          " (median=" + std::to_string(median) + ", spread=" +
          std::to_string(spread) + ", limit=" + std::to_string(limit) + ")";
        return false;
      }
      tau_est_anchor_[joint.motor_index] = median;
    }
    std::array<double, 7> right_q{};
    for (std::size_t i = 0; i < 7; ++i)
      right_q[i] = actual_motor_position_[active_motor_indices_[i]];
    if (!active_gravity_model_.baseline(
        right_q, current_imu_rpy_, feedforward_anchor_baseline_))
    {
      reason = "KDL failed to calculate " + active_arm_ + "-arm takeover anchor";
      return false;
    }
    if (task_application_enabled_)
    {
      for (std::size_t i = 0; i < 7; ++i)
        right_q[i] = actual_motor_position_[(active_arm_ == "left" ? 22 : 15) + i];
      if (!passive_gravity_model_.baseline(right_q, current_imu_rpy_, passive_anchor_baseline_))
      {
        reason = "failed to capture passive arm gravity anchor";
        return false;
      }
    }
    feedforward_blend_ = 0.0;
    feedforward_initialized_ = true;
    return true;
  }

  bool make_feedforward_locked(std::array<double, 35>& output, std::string& reason)
  {
    output.fill(0.0);
    current_max_feedforward_nm_ = 0.0;
    current_max_feedforward_motor_index_ = 0;
    feedforward_saturated_ = false;
    feedforward_saturation_reason_.clear();
    if (!gravity_feedforward_enabled_)
      return true;
    if (!feedforward_initialized_)
    {
      reason = "gravity feedforward anchor is not initialized";
      return false;
    }
    const double takeover_elapsed = (now() - takeover_started_time_).seconds();
    feedforward_blend_ = quintic(
      (takeover_elapsed - takeover_anchor_hold_s_) / feedforward_blend_s_);

    // Preserve the passive arm's captured support; only the selected arm moves.
    for (const auto& joint : kArmJoints)
      output[joint.motor_index] = tau_est_anchor_[joint.motor_index];

    std::array<double, 7> right_q{}, baseline{}, identified{};
    for (std::size_t i = 0; i < 7; ++i)
      right_q[i] = actual_motor_position_[active_motor_indices_[i]];
    if (!active_gravity_model_.baseline(right_q, current_imu_rpy_, baseline) ||
        !active_gravity_model_.identified(right_q, current_imu_rpy_, identified))
    {
      reason = "KDL failed to calculate dynamic " + active_arm_ + "-arm gravity feedforward";
      return false;
    }
    for (std::size_t i = 0; i < 7; ++i)
    {
      const auto motor_index = active_motor_indices_[i];
      const double anchored = tau_est_anchor_[motor_index] +
        baseline[i] - feedforward_anchor_baseline_[i];
      const double value = anchored + feedforward_blend_ * (identified[i] - anchored);
      if (!std::isfinite(value))
      {
        reason = active_arm_ + "-arm gravity feedforward is non-finite at motor " +
          std::to_string(motor_index);
        return false;
      }
      const double limit = continuous_torque_limit_for_motor(motor_index);
      output[motor_index] = std::clamp(value, -limit, limit);
      if (std::abs(value) > limit)
      {
        feedforward_saturated_ = true;
        feedforward_saturation_reason_ =
          active_arm_ + "-arm gravity feedforward saturated at motor " +
          std::to_string(motor_index) + " (requested=" + std::to_string(value) +
          " Nm, applied=" + std::to_string(output[motor_index]) + " Nm)";
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "%s; support remains active",
          feedforward_saturation_reason_.c_str());
      }
    }
    if (task_application_enabled_)
    {
      const std::size_t first = active_arm_ == "left" ? 22 : 15;
      for (std::size_t i = 0; i < 7; ++i) right_q[i] = actual_motor_position_[first+i];
      if (!passive_gravity_model_.baseline(right_q, current_imu_rpy_, baseline) ||
          !passive_gravity_model_.identified(right_q, current_imu_rpy_, identified))
      {
        reason = "passive-arm gravity calculation failed";
        return false;
      }
      for (std::size_t i = 0; i < 7; ++i)
      {
        const double anchored = tau_est_anchor_[first+i] + baseline[i] - passive_anchor_baseline_[i];
        const double value = anchored + feedforward_blend_ * (identified[i] - anchored);
        if (!std::isfinite(value)) { reason = "nonfinite passive gravity"; return false; }
        const double limit = continuous_torque_limit_for_motor(first+i);
        output[first+i] = std::clamp(value, -limit, limit);
        if (std::abs(value) > limit)
        {
          feedforward_saturated_ = true;
          feedforward_saturation_reason_ = "passive-arm gravity feedforward saturated";
        }
      }
    }
    for (const auto& joint : kArmJoints)
    {
      const double magnitude = std::abs(output[joint.motor_index]);
      if (magnitude > current_max_feedforward_nm_)
      {
        current_max_feedforward_nm_ = magnitude;
        current_max_feedforward_motor_index_ = joint.motor_index;
      }
    }
    peak_feedforward_nm_ = std::max(peak_feedforward_nm_, current_max_feedforward_nm_);
    return true;
  }

  void publish_arm_sdk_locked(
    const TrajectorySample& active_sample, double time_scale, double weight)
  {
    if (!command_pub_ || !armed_)
      return;
    auto desired = hold_motor_position_;
    std::array<double, 35> desired_velocity{};
    if (active_trajectory_)
    {
      for (std::size_t j = 0; j < active_arm_indices_.size(); ++j)
      {
        const auto motor_index = trajectory_joint(active_arm_indices_[j]).motor_index;
        desired[motor_index] = active_sample.positions[j];
        desired_velocity[motor_index] =
          bounded_trajectory_velocity_locked(j, active_sample.velocities[j], time_scale);
      }
    }
    std::array<double, 35> feedforward{};
    std::string feedforward_reason;
    if (!make_feedforward_locked(feedforward, feedforward_reason))
    {
      latch_locked(feedforward_reason + "; output is silent");
      return;
    }
    update_waist_command_locked(desired, desired_velocity, feedforward, weight);
    current_max_total_command_nm_ = 0.0;
    current_max_total_command_motor_index_ = 0;
    bool command_continuous_exceeded = false;
    bool command_peak_exceeded = false;
    std::string command_joint = "none";
    double command_value = 0.0;
    double command_continuous_limit = 0.0;
    double command_peak_limit = 0.0;
    double worst_command_ratio = 0.0;
    for (const auto& joint : kArmJoints)
    {
      const auto index = joint.motor_index;
      const double total_command = weight * (
        feedforward[index] + arm_integral_torque_nm_[index] +
        position_gain_for_motor(index) * (desired[index] - actual_motor_position_[index]) -
        damping_gain_for_motor(index) *
          (actual_motor_velocity_[index] - desired_velocity[index]));
      if (!std::isfinite(total_command))
      {
        latch_locked(
          "estimated feedforward plus PID command is non-finite at " +
          std::string(joint.name) + "; output is silent");
        return;
      }
      const double magnitude = std::abs(total_command);
      const double continuous_limit = continuous_torque_limit_for_motor(index);
      const double peak_limit = peak_torque_limit_for_motor(index);
      const double ratio = continuous_limit > 0.0 ? magnitude / continuous_limit : 0.0;
      if (ratio > worst_command_ratio)
      {
        worst_command_ratio = ratio;
        command_joint = joint.name;
        command_value = magnitude;
        command_continuous_limit = continuous_limit;
        command_peak_limit = peak_limit;
      }
      command_continuous_exceeded = command_continuous_exceeded || magnitude > continuous_limit;
      command_peak_exceeded = command_peak_exceeded || magnitude > peak_limit;
      if (magnitude > current_max_total_command_nm_)
      {
        current_max_total_command_nm_ = magnitude;
        current_max_total_command_motor_index_ = index;
      }
    }
    for (const auto configured_index : controlled_waist_indices_)
    {
      const auto index = static_cast<std::size_t>(configured_index);
      const auto& motor = waist_command_.motor_cmd[index];
      const double total_command = weight * (
        motor.tau + motor.kp * (motor.q - actual_motor_position_[index]) +
        motor.kd * (motor.dq - actual_motor_velocity_[index]));
      if (!std::isfinite(total_command))
      {
        latch_locked("estimated waist PID command is non-finite; output is silent");
        return;
      }
      const double magnitude = std::abs(total_command);
      const double limit = max_estimated_waist_torque_nm_;
      const double ratio = limit > 0.0 ? magnitude / limit : 0.0;
      if (ratio > worst_command_ratio)
      {
        worst_command_ratio = ratio;
        command_joint = "waist_joint_motor_" + std::to_string(index);
        command_value = magnitude;
        command_continuous_limit = limit;
        command_peak_limit = limit;
      }
      command_continuous_exceeded = command_continuous_exceeded || magnitude > limit;
      command_peak_exceeded = command_peak_exceeded || magnitude > limit;
      if (magnitude > current_max_total_command_nm_)
      {
        current_max_total_command_nm_ = magnitude;
        current_max_total_command_motor_index_ = index;
      }
    }
    const auto command_stamp = now();
    if (command_continuous_exceeded)
    {
      if (!command_overload_active_)
      {
        command_overload_active_ = true;
        command_overload_start_time_ = command_stamp;
      }
      command_overload_duration_s_ =
        (command_stamp - command_overload_start_time_).seconds();
      command_overload_reason_ =
        "estimated command overload at " + command_joint + " (|command|=" +
        std::to_string(command_value) + " Nm, continuous=" +
        std::to_string(command_continuous_limit) + " Nm, peak=" +
        std::to_string(command_peak_limit) + " Nm, duration=" +
        std::to_string(command_overload_duration_s_) + " s)";
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "%s", command_overload_reason_.c_str());
      if ((command_peak_exceeded || command_overload_duration_s_ >= torque_peak_duration_s_) &&
          !velocity_recovery_active_)
      {
        enter_damped_recovery_locked(command_stamp, command_overload_reason_);
        return;
      }
    }
    else
    {
      command_overload_active_ = false;
      command_overload_duration_s_ = 0.0;
      command_overload_reason_.clear();
    }
    peak_total_command_nm_ = std::max(peak_total_command_nm_, current_max_total_command_nm_);
    unitree_hg::msg::LowCmd command;
    for (const auto configured_index : controlled_waist_indices_)
    {
      const auto index = static_cast<std::size_t>(configured_index);
      command.motor_cmd[index] = waist_command_.motor_cmd[index];
    }
    for (const auto& joint : kArmJoints)
    {
      command.motor_cmd[joint.motor_index].q = static_cast<float>(desired[joint.motor_index]);
      command.motor_cmd[joint.motor_index].dq =
        static_cast<float>(desired_velocity[joint.motor_index]);
      command.motor_cmd[joint.motor_index].tau =
        static_cast<float>((feedforward[joint.motor_index] + arm_integral_torque_nm_[joint.motor_index]) * weight);
      command.motor_cmd[joint.motor_index].kp =
        static_cast<float>(position_gain_for_motor(joint.motor_index));
      command.motor_cmd[joint.motor_index].kd =
        static_cast<float>(damping_gain_for_motor(joint.motor_index));
    }
    command.motor_cmd[kWeightIndex].q = static_cast<float>(weight);
    command_pub_->publish(command);
    ++hardware_command_count_;
    write_waist_command_log_locked(command, desired[12], desired_velocity[12]);
  }

  void write_waist_command_log_locked(
    const unitree_hg::msg::LowCmd& command, double reference_q, double reference_dq)
  {
    if (!waist_command_log_.is_open()) return;
    const auto stamp = std::chrono::steady_clock::now();
    const auto& motor = command.motor_cmd[12];
    // These are the fields handed to publish(), not an acknowledgement from a motor.
    waist_command_log_ << std::setprecision(17)
      << std::chrono::duration_cast<std::chrono::nanoseconds>(stamp.time_since_epoch()).count()
      << ',' << hardware_command_count_ << ',' << csv_string(phase_)
      << ',' << csv_string(waist_control_mode_) << ',' << waist_control_revision_
      << ',' << waist_official_blend_ << ',' << current_control_gap_ms_
      << ',' << std::chrono::duration<double, std::milli>(stamp - last_state_steady_).count()
      << ',' << reference_q << ',' << reference_dq << ',' << motor.q << ',' << motor.dq
      << ',' << motor.tau << ',' << motor.kp << ',' << motor.kd
      << ',' << command.motor_cmd[kWeightIndex].q << ',' << actual_motor_position_[12]
      << ',' << actual_motor_velocity_[12] << ',' << actual_motor_tau_est_[12]
      << ',' << waist_integral_torque_nm_[12] << ',' << trajectory_start_blend_s_ << '\n';
    if (++waist_command_log_rows_ % 25 == 0) waist_command_log_.flush();
  }

  void control_timer_callback()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto control_now = std::chrono::steady_clock::now();
    if (task_client_watchdog_s_ > 0. && armed_ && !task_stop_active_ &&
        !task_stop_requested_.load() &&
        (std::chrono::duration<double>(control_now.time_since_epoch()).count() -
         task_heartbeat_ns_.load()/1e9 > task_client_watchdog_s_)) {
      ++task_stop_epoch_;
      last_task_stop_stamp_.store(now().nanoseconds());
      task_stop_requested_.store(true);
      last_rejection_ = "Jetson execution heartbeat expired; explicit recovery required";
    }
    if (active_trajectory_) stage_last_active_=control_now;
    else if (!armed_ || releasing_ ||
             std::chrono::duration<double>(control_now-stage_last_active_).count()>10.0)
      restore_stage_locked();
    const double control_gap_ms = last_control_steady_.time_since_epoch().count() == 0 ? 0.0 :
      std::chrono::duration<double, std::milli>(control_now - last_control_steady_).count();
    last_control_steady_ = control_now;
    current_control_gap_ms_ = control_gap_ms;
    max_control_gap_ms_ = std::max(max_control_gap_ms_, control_gap_ms);
    if (!active_trajectory_ && active_follow_goal_ && active_follow_goal_->is_canceling())
    {
      auto result = std::make_shared<FollowJointTrajectory::Result>();
      result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
      result->error_string = "trajectory cancelled; holding measured pose";
      active_follow_goal_->canceled(result);
      active_follow_goal_.reset();
    }
    const auto stamp = now();
    const double dt = 1.0 / control_rate_hz_;
    if ((armed_ || active_trajectory_) && !state_is_fresh_locked())
    {
      const auto entry_ns = state_callback_entry_ns_.load();
      const double entry_age_ms = entry_ns == 0 ? -1.0 :
        std::chrono::duration<double, std::milli>(control_now.time_since_epoch()).count() -
          static_cast<double>(entry_ns) / 1e6;
      std::ostringstream reason;
      reason << "LowState timeout during guarded operation: steady_age_ms="
             << std::chrono::duration<double, std::milli>(control_now - last_state_steady_).count()
             << " limit_ms=" << state_timeout_s_ * 1000.0
             << " callback_entry_age_ms=" << entry_age_ms
             << " last_lock_wait_ms=" << state_lock_wait_ms_
             << " last_callback_gap_ms=" << state_callback_gap_ms_
             << " control_gap_ms=" << control_gap_ms;
      latch_locked(reason.str());
      return;
    }
    if (armed_ && !service_state_is_ready_locked())
    {
      latch_locked(
        service_conflict_reason_.empty() ?
        "onboard service-state check became stale; output is silent" :
        service_conflict_reason_);
      return;
    }
    if (safety_latched_ || configuration_blocked_)
      return;
    update_task_stop_locked(stamp);

    if (hardware_output_available_ && armed_ && !active_trajectory_)
    {
      double hold_error = 0.0;
      for (const auto& joint : kArmJoints)
        hold_error = std::max(
          hold_error,
          std::abs(hold_motor_position_[joint.motor_index] -
                   actual_motor_position_[joint.motor_index]));
      for (const auto configured_index : controlled_waist_indices_)
      {
        const auto index = static_cast<std::size_t>(configured_index);
        hold_error = std::max(
          hold_error, std::abs(hold_motor_position_[index] - actual_motor_position_[index]));
      }
      current_tracking_error_rad_ = hold_error;
      peak_tracking_error_rad_ = std::max(peak_tracking_error_rad_, hold_error);
      if (hold_error > tracking_hard_limit_rad_)
      {
        enter_damped_recovery_locked(
          stamp, "hold-position tracking error exceeded the normal operating limit");
      }
    }

    TrajectorySample target;
    target.positions.assign(active_arm_indices_.size(), 0.0);
    target.velocities.assign(active_arm_indices_.size(), 0.0);
    target.accelerations.assign(active_arm_indices_.size(), 0.0);
    double command_time_scale = 0.0;
    bool integral_updated = false;
    if (active_trajectory_)
    {
      target = interpolate_locked(trajectory_progress_s_);
      double tracking_error = 0.0;
      for (std::size_t j = 0; j < active_arm_indices_.size(); ++j)
        tracking_error = std::max(
          tracking_error,
          std::abs(previous_target_[j] - actual_motor_position_[trajectory_joint(active_arm_indices_[j]).motor_index]));
      current_tracking_error_rad_ = tracking_error;
      peak_tracking_error_rad_ = std::max(peak_tracking_error_rad_, tracking_error);
      if (tracking_error > tracking_hard_limit_rad_)
      {
        enter_damped_recovery_locked(
          stamp, "trajectory tracking error exceeded the normal operating limit");
        publish_arm_sdk_locked(target, 0.0, command_weight_current_);
        return;
      }
      const double time_scale = g1_task::tracking_time_scale(
        tracking_error, tracking_warning_rad_, tracking_pause_rad_);
      trajectory_time_scale_rate_ = (time_scale - previous_trajectory_time_scale_) / dt;
      previous_trajectory_time_scale_ = time_scale;
      command_time_scale = time_scale;

      if (active_follow_goal_ && active_follow_goal_->is_active())
      {
        auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
        feedback->header.stamp = stamp;
        feedback->joint_names = trajectory_.joint_names;
        feedback->desired.positions = target.positions;
        feedback->desired.velocities.resize(target.velocities.size());
        feedback->desired.accelerations.resize(target.accelerations.size());
        feedback->actual.positions.resize(target.positions.size());
        feedback->actual.velocities.resize(target.positions.size());
        feedback->error.positions.resize(target.positions.size());
        feedback->error.velocities.resize(target.positions.size());
        for (std::size_t j = 0; j < target.positions.size(); ++j)
        {
          const auto motor_index = trajectory_joint(active_arm_indices_[j]).motor_index;
          feedback->desired.velocities[j] =
            bounded_trajectory_velocity_locked(j, target.velocities[j], command_time_scale);
          feedback->desired.accelerations[j] =
            target.accelerations[j] * command_time_scale * command_time_scale +
            target.velocities[j] * trajectory_time_scale_rate_;
          feedback->actual.positions[j] = actual_motor_position_[motor_index];
          feedback->actual.velocities[j] = actual_motor_velocity_[motor_index];
          feedback->error.positions[j] =
            target.positions[j] - feedback->actual.positions[j];
          feedback->error.velocities[j] =
            feedback->desired.velocities[j] - feedback->actual.velocities[j];
        }
        active_follow_goal_->publish_feedback(feedback);
      }

      for (std::size_t j = 0; j < target.positions.size(); ++j)
      {
        if (target_initialized_ &&
            std::abs(target.positions[j] - previous_target_[j]) > max_command_step_rad_ + 1e-9)
        {
          enter_damped_recovery_locked(
            stamp, "trajectory produced a per-cycle command step above the guarded limit");
          publish_arm_sdk_locked(target, 0.0, command_weight_current_);
          return;
        }
      }
      update_integral_locked(target, time_scale);
      integral_updated = true;
      publish_preview_locked(stamp, target, time_scale);
      write_motion_log_locked(stamp, target, time_scale, "sample");
      previous_target_ = target.positions;
      previous_target_velocity_ = target.velocities;
      previous_target_acceleration_ = target.accelerations;
      target_initialized_ = true;
      trajectory_progress_s_ += dt * time_scale;
      if (trajectory_progress_s_ >= trajectory_total_duration_s_)
      {
        trajectory_progress_s_ = trajectory_total_duration_s_;
        target = interpolate_locked(trajectory_progress_s_);
        write_motion_log_locked(stamp, target, 0.0, "complete");
        previous_target_ = target.positions;
        previous_target_velocity_ = target.velocities;
        previous_target_acceleration_ = target.accelerations;
        for (std::size_t j = 0; j < target.positions.size(); ++j)
          hold_motor_position_[trajectory_joint(active_arm_indices_[j]).motor_index] =
            target.positions[j];
        // Observe convergence after action success without delaying the next stage.
        motion_log_hold_remaining_s_ = 10.0;
        previous_target_velocity_.assign(target.positions.size(), 0.0);
        previous_target_acceleration_.assign(target.positions.size(), 0.0);
        active_trajectory_ = false;
        phase_ = hardware_output_available_ ? "ARMED_HOLD" : "DRY_RUN_COMPLETE";
        if (active_follow_goal_ && active_follow_goal_->is_active())
        {
          auto result = std::make_shared<FollowJointTrajectory::Result>();
          result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
          result->error_string = "guarded trajectory completed; holding final pose";
          active_follow_goal_->succeed(result);
          active_follow_goal_.reset();
        }
      }
    }

    if (!integral_updated)
      update_integral_locked(target, command_time_scale);

    if (!active_trajectory_ && motion_log_hold_remaining_s_ > 0.0)
    {
      TrajectorySample held;
      for (const auto joint_index : active_arm_indices_)
        held.positions.push_back(hold_motor_position_[trajectory_joint(joint_index).motor_index]);
      held.velocities.assign(held.positions.size(), 0.0);
      held.accelerations.assign(held.positions.size(), 0.0);
      write_motion_log_locked(stamp, held, 0.0, "settling_hold");
      motion_log_hold_remaining_s_ -= dt;
      if (motion_log_hold_remaining_s_ <= 0.0)
        close_motion_log_locked("completed hold observation");
    }

    if (!hardware_output_available_ || !armed_)
      return;

    if (releasing_)
    {
      command_weight_current_ = std::max(
        0.0, command_weight_current_ - command_weight_ * dt / release_ramp_s_);
      publish_arm_sdk_locked(target, 0.0, command_weight_current_);
      if (command_weight_current_ <= 1e-6)
      {
        service_state_validated_for_takeover_ = false;
        armed_ = false;
        releasing_ = false;
        command_pub_.reset();
        phase_ = "UNARMED";
        RCLCPP_WARN(
          get_logger(), "Controlled weight release complete; /arm_sdk publisher destroyed");
      }
      return;
    }

    const double takeover_elapsed = (stamp - takeover_started_time_).seconds();
    if (velocity_recovery_active_)
    {
      phase_ = "VELOCITY_RECOVERY";
    }
    else if (takeover_elapsed < takeover_anchor_hold_s_)
    {
      phase_ = "TAU_ANCHORED_HOLD";
    }
    else if (takeover_elapsed < takeover_anchor_hold_s_ + feedforward_blend_s_)
    {
      phase_ = "GRAVITY_FEEDFORWARD_BLEND";
    }
    else if (!active_trajectory_ && !task_stop_active_ && !task_resume_blend_)
    {
      phase_ = "ARMED_HOLD";
    }
    publish_arm_sdk_locked(target, command_time_scale, command_weight_current_);
  }

  void update_waist_command_locked(
    const std::array<double, 35>& desired,
    const std::array<double, 35>& velocity,
    const std::array<double, 35>& feedforward, double weight)
  {
    // The unified packet stays at the arm rate. Only waist fields are held at
    // the configured rate; stop/recovery never waits for a held command to expire.
    waist_command_elapsed_s_ += 1.0 / control_rate_hz_;
    const double takeover = gravity_feedforward_enabled_ ? feedforward_blend_ :
      quintic(((now()-takeover_started_time_).seconds()-takeover_anchor_hold_s_)/feedforward_blend_s_);
    const bool urgent = releasing_ || velocity_recovery_active_ || task_stop_active_ ||
      task_resume_blend_ || task_stop_requested_.load() || stage_switching_locked() || takeover < .999;
    if (waist_command_valid_ && !urgent && waist_command_elapsed_s_ < 1.0 / waist_command_rate_hz_ - 1e-9)
      return;
    waist_command_elapsed_s_ = 0.0;
    waist_command_valid_ = true;
    if (waist_mode_switching_)
    {
      const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now()-waist_switch_started_).count();
      const double target = waist_control_mode_ == "official_pd" ? 1.0 : 0.0;
      waist_profile_blend_ = waist_switch_from_ + (target-waist_switch_from_)*quintic(elapsed);
      if (elapsed >= 1.0) waist_mode_switching_ = false;
    }
    const double official = !releasing_ && !velocity_recovery_active_ ?
      waist_profile_blend_*std::clamp(takeover, 0.0, 1.0) *
      (1.0-task_compliance_blend_) : 0.0;
    waist_official_blend_ = official;
    for (const auto configured_index : controlled_waist_indices_)
    {
      const auto index = static_cast<std::size_t>(configured_index);
      auto& motor = waist_command_.motor_cmd[index];
      motor.q = static_cast<float>(desired[index]);
      motor.dq = static_cast<float>((1.0-official)*velocity[index]);
      motor.tau = static_cast<float>((1.0-official)*
        (feedforward[index]+waist_integral_torque_nm_[index])*weight);
      motor.kp = static_cast<float>((1.0-official)*waist_position_gain(index)+official*240.0);
      motor.kd = static_cast<float>((1.0-official)*
        (index==12 ? stage_waist_gain_locked(true) : waist_kd_)+official*6.0);
    }
  }

  double waist_position_gain(std::size_t index) const
  {
    // The same gain is used by both command protection and the actual writer.
    const double gain=index==12 ? stage_waist_gain_locked(false) : waist_kp_;
    return index == 12 && task_waist_admittance_active_ ?
      gain + task_compliance_blend_ * (20.0 - gain) : gain;
  }

  void update_task_stop_locked(const rclcpp::Time& stamp)
  {
    if (task_stop_requested_.exchange(false))
    {
      restore_stage_locked();
      task_waist_admittance_active_ = task_waist_admittance_requested_.exchange(false);
      close_motion_log_locked("operator stop task");
      active_trajectory_ = false;
      if (active_follow_goal_ && active_follow_goal_->is_active())
      {
        auto result = std::make_shared<FollowJointTrajectory::Result>();
        result->error_code = FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED;
        result->error_string = "operator stop; plan invalidated; measured hold then admittance";
        active_follow_goal_->abort(result);
      }
      active_follow_goal_.reset();
      task_stop_active_ = true;
      task_resume_blend_ = false;
      task_compliance_blend_ = 0;
      task_zeroed_ = false;
      reset_integral_locked();
      task_settle_since_ = stamp;
      hold_motor_position_ = actual_motor_position_;
      task_admittance_.reset();
    }
    if (!armed_ || releasing_ || velocity_recovery_active_) return;
    if (task_resume_blend_)
    {
      task_compliance_blend_ = task_resume_from_ *
        (1.0 - quintic((stamp-task_transition_since_).seconds() / .5));
      phase_ = "TASK_HOLD_BLEND";
      if (task_compliance_blend_ <= 0) { task_resume_blend_ = false; phase_ = "ARMED_HOLD"; }
      return;
    }
    if (!task_stop_active_) return;
    phase_ = "TASK_STOP_HOLD";
    if (!task_zeroed_)
    {
      if (current_max_measured_velocity_rad_s_ > .05 || feedforward_blend_ < .999 ||
          torque_overload_active_ || command_overload_active_)
        task_settle_since_ = stamp;
      if ((stamp-task_settle_since_).seconds() < .3) return;
      std::array<double, 35> ff{};
      std::string reason;
      if (!make_feedforward_locked(ff, reason) || feedforward_saturated_) return;
      for (const auto& j : kArmJoints)
        task_bias_[j.motor_index] = ff[j.motor_index] - actual_motor_tau_est_[j.motor_index];
      if (task_waist_admittance_active_)
        task_bias_[12] = ff[12] - actual_motor_tau_est_[12];
      task_zeroed_ = true;
      hold_motor_position_ = actual_motor_position_;
      task_transition_since_ = stamp;
    }
    task_compliance_blend_ = quintic((stamp-task_transition_since_).seconds() / .5);
    phase_ = task_compliance_blend_ < 1 ? "TASK_ADMITTANCE_BLEND" : "TASK_ADMITTANCE";
    if (task_compliance_blend_ < 1) return;
    std::array<double, 35> ff{};
    std::string reason;
    if (!make_feedforward_locked(ff, reason) || feedforward_saturated_)
    {
      enter_damped_recovery_locked(stamp, "task admittance gravity unavailable/saturated");
      return;
    }
    for (std::size_t i = 0; i < kArmJoints.size(); ++i)
    {
      const auto& j = kArmJoints[i];
      const auto m = j.motor_index;
      hold_motor_position_[m] = task_admittance_.step(
        i, hold_motor_position_[m], actual_motor_position_[m],
        ff[m] - actual_motor_tau_est_[m] - task_bias_[m],
        j.lower + joint_limit_margin_rad_, j.upper - joint_limit_margin_rad_,
        1.0 / control_rate_hz_);
    }
    if (task_waist_admittance_active_)
      hold_motor_position_[12] = task_admittance_.step(
        14, hold_motor_position_[12], actual_motor_position_[12],
        ff[12] - actual_motor_tau_est_[12] - task_bias_[12],
        kTaskWaistYaw.lower + joint_limit_margin_rad_,
        kTaskWaistYaw.upper - joint_limit_margin_rad_, 1.0 / control_rate_hz_);
  }

  void hold_after_stop_callback(
    const std::shared_ptr<ResumeTaskHold::Request> request,
    std::shared_ptr<ResumeTaskHold::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (request->stop_epoch != task_stop_epoch_.load() ||
        !armed_ || safety_latched_ || releasing_ || velocity_recovery_active_ ||
        task_stop_requested_.load() || !state_is_fresh_locked() ||
        !service_state_is_ready_locked() || current_max_measured_velocity_rad_s_ > .05)
    {
      response->success = false;
      response->message = "hold rejected: require stationary, fresh supported state; no latch reset is performed";
      return;
    }
    if (task_stop_active_)
    {
      hold_motor_position_ = actual_motor_position_;
      task_resume_blend_ = task_compliance_blend_ > 0;
      task_resume_from_ = task_compliance_blend_;
      task_stop_active_ = false;
      task_transition_since_ = now();
      task_admittance_.reset();
      phase_ = task_resume_blend_ ? "TASK_HOLD_BLEND" : "ARMED_HOLD";
    }
    response->success = true;
    response->message = "holding measured pose; wait for ARMED_HOLD, then make a new plan";
  }

  void arm_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!hardware_output_available_)
    {
      response->success = false;
      response->message = "arm denied: hardware output was not authorized at startup";
      return;
    }
    if (task_stop_requested_.load() || task_stop_active_)
    {
      response->success = false;
      response->message = "arm denied: a task stop is active; no takeover after a newer stop";
      return;
    }
    if (armed_)
    {
      response->success = false;
      response->message = "arm denied: gateway is already armed";
      return;
    }
    if (task_client_watchdog_s_ > 0. &&
        std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count() -
          task_heartbeat_ns_.load()/1e9 > task_client_watchdog_s_) {
      response->success = false;
      response->message = "arm denied: Jetson execution heartbeat unavailable";
      return;
    }
    if (safety_latched_ || !state_is_fresh_locked() || !service_state_is_ready_locked() ||
        !upper_body_state_safe_ || temperature_alarm_active_ ||
        (keys_ & kL2BMask) == kL2BMask)
    {
      response->success = false;
      response->message =
        "arm denied: require reset latch, fresh safe state/service list, released L2+B, "
        "temperature below alarm threshold, and no onboard control conflict";
      return;
    }
    std::string feedforward_reason;
    if (!capture_feedforward_anchor_locked(feedforward_reason))
    {
      response->success = false;
      response->message = "arm denied: " + feedforward_reason;
      return;
    }
    hold_motor_position_ = actual_motor_position_;
    reset_integral_locked();
    try
    {
      command_pub_ = create_publisher<unitree_hg::msg::LowCmd>(
        command_topic_, rclcpp::QoS(10).reliable().durability_volatile());
    }
    catch (const std::exception& error)
    {
      command_pub_.reset();
      response->success = false;
      response->message = "arm denied: failed to create /arm_sdk publisher: " +
        std::string(error.what());
      return;
    }
    command_weight_current_ = command_weight_;
    task_stop_active_ = false;
    task_compliance_blend_ = 0;
    task_resume_blend_ = false;
    takeover_started_time_ = now();
    active_trajectory_ = false;
    service_state_validated_for_takeover_ = true;
    armed_ = true;
    releasing_ = false;
    velocity_recovery_active_ = false;
    velocity_recovery_timed_out_ = false;
    velocity_recovery_reason_.clear();
    phase_ = "TAU_ANCHORED_HOLD";
    response->success = true;
    response->message =
      "armed: weight=1 tau_est-anchored hold, then KDL/payload feedforward blend";
    RCLCPP_ERROR(
      get_logger(),
      "HARDWARE ARMED: publisher created; weight=1, tau_est anchor hold %.1f s, "
      "then gravity blend %.1f s", takeover_anchor_hold_s_, feedforward_blend_s_);
  }

  void disarm_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    close_motion_log_locked("gateway disarm requested");
    active_trajectory_ = false;
    reset_integral_locked();
    velocity_recovery_active_ = false;
    velocity_recovery_timed_out_ = false;
    velocity_recovery_reason_.clear();
    if (active_follow_goal_ && active_follow_goal_->is_active())
    {
      auto result = std::make_shared<FollowJointTrajectory::Result>();
      result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
      result->error_string = "gateway disarm requested; trajectory stopped";
      if (active_follow_goal_->is_canceling())
        active_follow_goal_->canceled(result);
      else
      {
        result->error_code = FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED;
        active_follow_goal_->abort(result);
      }
    }
    active_follow_goal_.reset();
    if (!armed_)
    {
      response->success = true;
      response->message = "already unarmed; no command is being published";
      return;
    }
    hold_motor_position_ = actual_motor_position_;
    releasing_ = true;
    phase_ = "RELEASING_HOLD";
    response->success = true;
    response->message = "controlled weight release started while holding measured pose";
  }

  void reset_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    response->success = reset_silent_locked(response->message);
  }

  bool reset_silent_locked(std::string& message, const std::uint64_t* stop_epoch = nullptr)
  {
    if (armed_ || command_pub_ || releasing_ || active_trajectory_)
    {
      message = "reset denied: controller must be unarmed with no command publisher or active motion";
      return false;
    }
    if (!state_is_fresh_locked() || !service_state_is_ready_locked() ||
        !upper_body_state_safe_ || temperature_alarm_active_ ||
        current_max_measured_velocity_rad_s_ > .05 ||
        (keys_ & kL2BMask) == kL2BMask)
    {
      message = "reset denied: require fresh safe stationary state/service list, released L2+B, and no onboard conflict";
      return false;
    }
    if (stop_epoch)
    {
      if (*stop_epoch != task_stop_epoch_.load())
      {
        message = "reset denied: a newer STOP arrived; inspect state and confirm again";
        return false;
      }
      // STOP receipt is lock-free. Clear the old request BEFORE checking its
      // epoch again; any later STOP stays pending and continues to block ARM.
      task_stop_requested_.exchange(false);
      if (*stop_epoch != task_stop_epoch_.load())
      {
        task_stop_requested_.store(true);
        message = "reset denied: STOP changed during reset; stop retained";
        return false;
      }
      task_stop_active_ = false;
      task_resume_blend_ = false;
      task_compliance_blend_ = 0.0;
      task_zeroed_ = false;
      task_waist_admittance_active_ = false;
      task_waist_admittance_requested_.store(false);
      task_admittance_.reset();
    }
    safety_latched_ = false;
    latch_reason_.clear();
    last_rejection_.clear();
    service_state_validated_for_takeover_ = false;
    armed_ = false;
    releasing_ = false;
    active_trajectory_ = false;
    reset_integral_locked();
    velocity_recovery_active_ = false;
    velocity_recovery_timed_out_ = false;
    velocity_recovery_reason_.clear();
    close_motion_log_locked("safety latch reset");
    goal_stop_epochs_.clear();
    stage_goal_revisions_.clear();
    waist_goal_revisions_.clear();
    command_pub_.reset();
    phase_ = hardware_output_available_ ? "UNARMED" : "OUTPUT_DISABLED";
    message = stop_epoch ? "latch and acknowledged task STOP reset; gateway remains unarmed" :
      "latch reset; gateway remains unarmed";
    return true;
  }

  void cancel_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (task_stop_active_ || task_stop_requested_.load() || velocity_recovery_active_)
    {
      response->success = false;
      response->message = "cancel cannot clear task stop or damped recovery";
      return;
    }
    close_motion_log_locked("trajectory cancelled by gateway service");
    active_trajectory_ = false;
    reset_integral_locked();
    velocity_recovery_active_ = false;
    velocity_recovery_timed_out_ = false;
    velocity_recovery_reason_.clear();
    if (active_follow_goal_ && active_follow_goal_->is_active())
    {
      auto result = std::make_shared<FollowJointTrajectory::Result>();
      result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
      result->error_string = "trajectory cancelled by gateway service";
      if (active_follow_goal_->is_canceling())
        active_follow_goal_->canceled(result);
      else
      {
        result->error_code = FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED;
        active_follow_goal_->abort(result);
      }
    }
    active_follow_goal_.reset();
    if (armed_)
    {
      hold_motor_position_ = actual_motor_position_;
      phase_ = "ARMED_HOLD";
    }
    else
      phase_ = hardware_output_available_ ? "UNARMED" : "OUTPUT_DISABLED";
    response->success = true;
    response->message = "trajectory cancelled; hardware state unchanged";
  }

  void diagnostics_timer_callback()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    const auto stamp = std::chrono::steady_clock::now();
    const bool switching = stage_switching_locked();
    // Confirm stage/phase transitions promptly; retain the idle publication rate.
    if (!switching && !diagnostics_last_switching_ &&
        phase_ == diagnostics_last_phase_ && stage_revision_ == diagnostics_last_stage_revision_ &&
        stamp - diagnostics_last_publish_ < 500ms) return;
    diagnostics_last_publish_ = stamp;
    diagnostics_last_phase_ = phase_;
    diagnostics_last_stage_revision_ = stage_revision_;
    diagnostics_last_switching_ = switching;
    diagnostic_msgs::msg::DiagnosticArray array;
    diagnostic_msgs::msg::DiagnosticStatus status;
    array.header.stamp = now();
    status.name = "G1 guarded arm_sdk hardware gateway";
    status.hardware_id = hardware_output_available_ ? "ARM_SDK_GUARDED" : "NO_HARDWARE_PUBLISHER";
    status.values.push_back(diagnostic_value("control_rate_hz", std::to_string(control_rate_hz_)));
    status.values.push_back(diagnostic_value("waist_control_mode", waist_control_mode_));
    status.values.push_back(diagnostic_value("task_stage_supported", "true"));
    status.values.push_back(diagnostic_value("task_stage_blend_seconds", std::to_string(kTaskStageBlendSeconds)));
    status.values.push_back(diagnostic_value("task_stage_max_kp", "unbounded"));
    status.values.push_back(diagnostic_value("task_stage_max_kd", "unbounded"));
    status.values.push_back(diagnostic_value("task_stage_waist_supported", "true"));
    status.values.push_back(diagnostic_value("task_stage_wrist_supported", "true"));
    status.values.push_back(diagnostic_value("task_stage_distal_wrist_supported", "true"));
    status.values.push_back(diagnostic_value("task_stage_arm_kp_supported", "true"));
    status.values.push_back(diagnostic_value("task_stage_arm_kp_count", std::to_string(stage_arm_kp_.size())));
    for (std::size_t i=0; i<stage_arm_kp_.size(); ++i)
      status.values.push_back(diagnostic_value("task_stage_arm_kp_"+std::to_string(i),std::to_string(stage_arm_kp_[i])));
    status.values.push_back(diagnostic_value("task_stage_distal_wrist_kp", std::to_string(stage_distal_wrist_kp_)));
    status.values.push_back(diagnostic_value("task_stage_wrist_kp", std::to_string(stage_wrist_kp_)));
    status.values.push_back(diagnostic_value("task_stage_waist_max_kp", "unbounded"));
    status.values.push_back(diagnostic_value("task_stage_waist_max_kd", "unbounded"));
    status.values.push_back(diagnostic_value("task_stage_waist_kp", std::to_string(stage_waist_kp_)));
    status.values.push_back(diagnostic_value("task_stage_waist_kd", std::to_string(stage_waist_kd_)));
    status.values.push_back(diagnostic_value("waist_guarded_effective_kp", std::to_string(stage_waist_gain_locked(false))));
    status.values.push_back(diagnostic_value("waist_guarded_effective_kd", std::to_string(stage_waist_gain_locked(true))));
    status.values.push_back(diagnostic_value("task_stage_profile", stage_profile_));
    status.values.push_back(diagnostic_value("task_stage_arm", stage_first_motor_==15 ? "left" : "right"));
    status.values.push_back(diagnostic_value("task_stage_revision", std::to_string(stage_revision_)));
    status.values.push_back(diagnostic_value("task_stage_switching", stage_switching_locked() ? "true" : "false"));
    status.values.push_back(diagnostic_value("task_stage_kp", std::to_string(stage_kp_)));
    status.values.push_back(diagnostic_value("task_stage_kd", std::to_string(stage_kd_)));
    status.values.push_back(diagnostic_value("waist_control_revision", std::to_string(waist_control_revision_)));
    status.values.push_back(diagnostic_value("waist_mode_switching", waist_mode_switching_ ? "true" : "false"));
    status.values.push_back(diagnostic_value("waist_command_rate_hz", std::to_string(waist_command_rate_hz_)));
    status.values.push_back(diagnostic_value("waist_hold_reference_policy", "continuous_hold_v1"));
    for (const auto& joint : kArmJoints)
    {
      if (joint.left) continue;
      std::ostringstream value;
      value << std::setprecision(17) << hold_motor_position_[joint.motor_index];
      status.values.push_back(diagnostic_value(std::string("hold_reference_") + joint.name, value.str()));
    }
    std::ostringstream waist_reference;
    waist_reference << std::setprecision(17) << hold_motor_position_[12];
    status.values.push_back(diagnostic_value("hold_reference_waist_yaw_joint", waist_reference.str()));
    status.values.push_back(diagnostic_value("last_waist_command_log_path", last_waist_command_log_path_));
    status.values.push_back(diagnostic_value("waist_official_blend", std::to_string(waist_official_blend_)));
    status.values.push_back(diagnostic_value("waist_command_kp", std::to_string(waist_command_.motor_cmd[12].kp)));
    status.values.push_back(diagnostic_value("waist_command_kd", std::to_string(waist_command_.motor_cmd[12].kd)));
    status.values.push_back(diagnostic_value("control_gap_ms", std::to_string(current_control_gap_ms_)));
    status.values.push_back(diagnostic_value("max_control_gap_ms", std::to_string(max_control_gap_ms_)));
    status.values.push_back(diagnostic_value("waist_guard_lower_rad", std::to_string(kTaskWaistYaw.lower + joint_limit_margin_rad_)));
    status.values.push_back(diagnostic_value("waist_guard_upper_rad", std::to_string(kTaskWaistYaw.upper - joint_limit_margin_rad_)));
    status.values.push_back(diagnostic_value("lowstate_timeout_ms", std::to_string(state_timeout_s_ * 1000.0)));
    status.values.push_back(diagnostic_value("lowstate_callback_gap_ms", std::to_string(state_callback_gap_ms_)));
    status.values.push_back(diagnostic_value("lowstate_max_callback_gap_ms", std::to_string(max_state_callback_gap_ms_)));
    status.values.push_back(diagnostic_value("lowstate_lock_wait_ms", std::to_string(state_lock_wait_ms_)));
    status.values.push_back(diagnostic_value("task_application_enabled", task_application_enabled_ ? "true" : "false"));
    status.values.push_back(diagnostic_value("task_waist_trajectory_supported", task_waist_enabled_ ? "true" : "false"));
    status.values.push_back(diagnostic_value("task_dual_prepare_supported", task_dual_prepare_enabled_ ? "true" : "false"));
    status.values.push_back(diagnostic_value("task_waist_admittance_supported", task_waist_enabled_ ? "true" : "false"));
    status.values.push_back(diagnostic_value("task_waist_admittance_active",
      task_stop_active_ && task_waist_admittance_active_ ? "true" : "false"));
    status.values.push_back(diagnostic_value("waist_effective_kp", std::to_string(waist_command_.motor_cmd[12].kp)));
    status.values.push_back(diagnostic_value("task_stop_active", task_stop_active_ ? "true" : "false"));
    status.values.push_back(diagnostic_value("task_reset_supported", "true"));
    status.values.push_back(diagnostic_value("task_stop_requested", task_stop_requested_.load() ? "true" : "false"));
    status.values.push_back(diagnostic_value("task_stop_epoch", std::to_string(task_stop_epoch_.load())));
    if (configuration_blocked_ || safety_latched_ || service_conflict_)
    {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      if (configuration_blocked_)
        status.message = "hardware output requested without exact authorization";
      else if (safety_latched_)
        status.message = latch_reason_;
      else
        status.message = service_conflict_reason_;
    }
    else if (!state_is_fresh_locked())
    {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "waiting for fresh LowState";
    }
    else if (service_check_enabled_ && !service_state_is_ready_locked())
    {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "waiting for a fresh read-only onboard ServiceList";
    }
    else if (service_check_enabled_ && armed_ && !service_state_is_fresh_locked())
    {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message =
        "armed from a validated ServiceList; firmware API 1003 is not currently replying";
    }
    else if (velocity_recovery_active_)
    {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = velocity_recovery_timed_out_ ?
        "damped recovery remains active; low-stiffness support is being maintained" :
        "trajectory stopped; damped recovery is settling";
    }
    else if (temperature_alarm_active_)
    {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = temperature_alarm_reason_;
    }
    else if (torque_overload_active_ || command_overload_active_)
    {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = command_overload_active_ ? command_overload_reason_ : torque_overload_reason_;
    }
    else if (waist_integral_saturated_)
    {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "waist integral torque is at its configured limit";
    }
    else if (feedforward_saturated_)
    {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = feedforward_saturation_reason_;
    }
    else
    {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = hardware_output_available_ ?
        (armed_ ? "hardware armed under safety gates" : "hardware path available but unarmed") :
        "dry run; /arm_sdk publisher does not exist";
    }
    status.values.push_back(diagnostic_value("phase", phase_));
    status.values.push_back(diagnostic_value("active_arm", active_arm_));
    status.values.push_back(diagnostic_value("active_payload_fit", active_payload_fit_path_));
    for (const auto& joint : kArmJoints)
    {
      const auto prefix = std::string(joint.name);
      status.values.push_back(diagnostic_value(prefix + "_kp", std::to_string(position_gain_for_motor(joint.motor_index))));
      status.values.push_back(diagnostic_value(prefix + "_kd", std::to_string(damping_gain_for_motor(joint.motor_index))));
    }
    status.values.push_back(diagnostic_value("arm_integral_enabled", arm_integral_enabled_ ? "true" : "false"));
    status.values.push_back(diagnostic_value("passive_arm_integral_enabled", passive_arm_integral_enabled_ ? "true" : "false"));
    status.values.push_back(diagnostic_value("arm_integral_recovery_error_deg",
      std::to_string(arm_integral_tuning_.recovery_error_rad * 180.0 / 3.14159265358979323846)));
    status.values.push_back(diagnostic_value("arm_integral_policy", "bounded_slow_recovery_v4_hold_rate"));
    status.values.push_back(diagnostic_value("arm_integral_hold_rate_nm_s", std::to_string(arm_integral_hold_rate_nm_s_)));
    status.values.push_back(diagnostic_value("arm_integral_saturated", arm_integral_saturated_ ? "true" : "false"));
    for (std::size_t j = 0; j < kArmJoints.size(); ++j)
    {
      const auto i = j % 7;
      const auto index = kArmJoints[j].motor_index;
      const auto prefix = std::string(kArmJoints[j].name);
      status.values.push_back(diagnostic_value(prefix + "_ki", std::to_string(arm_integral_ki_[i])));
      status.values.push_back(diagnostic_value(prefix + "_integral_limit_nm", std::to_string(arm_integral_limits_nm_[i])));
      status.values.push_back(diagnostic_value(prefix + "_integral_nm", std::to_string(arm_integral_torque_nm_[index])));
      status.values.push_back(diagnostic_value(prefix + "_integral_mode", g1_task::waist_integral_mode_name(arm_integral_modes_[index])));
    }
    status.values.push_back(diagnostic_value(
      "hardware_output_requested", hardware_output_requested_ ? "true" : "false"));
    status.values.push_back(diagnostic_value(
      "hardware_output_available", hardware_output_available_ ? "true" : "false"));
    status.values.push_back(diagnostic_value(
      "command_publisher_exists", command_pub_ ? "true" : "false"));
    status.values.push_back(diagnostic_value("armed", armed_ ? "true" : "false"));
    status.values.push_back(diagnostic_value("command_topic", command_topic_));
    status.values.push_back(diagnostic_value("mode_machine", std::to_string(mode_machine_)));
    status.values.push_back(diagnostic_value("mode_pr", std::to_string(mode_pr_)));
    status.values.push_back(diagnostic_value("keys", std::to_string(keys_)));
    status.values.push_back(diagnostic_value(
      "service_check_enabled", service_check_enabled_ ? "true" : "false"));
    status.values.push_back(diagnostic_value(
      "service_state_ready", service_state_is_ready_locked() ? "true" : "false"));
    status.values.push_back(diagnostic_value(
      "service_state_fresh", service_state_is_fresh_locked() ? "true" : "false"));
    status.values.push_back(diagnostic_value(
      "service_state_count", std::to_string(service_state_count_)));
    status.values.push_back(diagnostic_value(
      "service_conflict", service_conflict_ ? "true" : "false"));
    status.values.push_back(diagnostic_value(
      "service_conflict_reason", service_conflict_reason_));
    status.values.push_back(diagnostic_value(
      "command_weight", std::to_string(command_weight_current_)));
    status.values.push_back(diagnostic_value("waist_kp", std::to_string(waist_kp_)));
    status.values.push_back(diagnostic_value("waist_kd", std::to_string(waist_kd_)));
    status.values.push_back(diagnostic_value("waist_ki", std::to_string(waist_ki_)));
    status.values.push_back(diagnostic_value(
      "waist_integral_policy", g1_task::WaistIntegral::kWaistPolicyVersion));
    status.values.push_back(diagnostic_value("waist_integral_recovery_error_deg",
      std::to_string(waist_integral_tuning_.recovery_error_rad * 180.0 / 3.14159265358979323846)));
    status.values.push_back(diagnostic_value("waist_integral_build_rate_nm_s",
      std::to_string(waist_integral_tuning_.build_rate_nm_s)));
    status.values.push_back(diagnostic_value("waist_integral_curve_scale",
      std::to_string(waist_integral_tuning_.curve_gain_scale)));
    status.values.push_back(diagnostic_value("waist_integral_smooth_stiction",
      waist_integral_tuning_.smooth_stiction ? "true" : "false"));
    status.values.push_back(diagnostic_value(
      "waist_integral_deadband_rad", std::to_string(g1_task::WaistIntegral::kDeadbandRad)));
    status.values.push_back(diagnostic_value(
      "waist_integral_torque_limit_nm", std::to_string(waist_integral_torque_limit_nm_)));
    status.values.push_back(diagnostic_value(
      "waist_integral_torque_nm", std::to_string(current_waist_integral_torque_nm_)));
    status.values.push_back(diagnostic_value(
      "peak_waist_integral_torque_nm", std::to_string(peak_waist_integral_torque_nm_)));
    status.values.push_back(diagnostic_value(
      "waist_integral_gain_scale", std::to_string(waist_integral_gain_scale_)));
    status.values.push_back(diagnostic_value("waist_integral_mode", waist_integral_mode_));
    status.values.push_back(diagnostic_value(
      "waist_integral_active", waist_integral_active_ ? "true" : "false"));
    status.values.push_back(diagnostic_value(
      "waist_integral_saturated", waist_integral_saturated_ ? "true" : "false"));
    status.values.push_back(diagnostic_value(
      "waist_gain_revision", std::to_string(waist_gain_revision_)));
    status.values.push_back(diagnostic_value("last_waist_gain_change", last_waist_gain_change_));
    status.values.push_back(diagnostic_value("last_waist_gain_rejection", last_waist_gain_rejection_));
    status.values.push_back(diagnostic_value(
      "gravity_feedforward_enabled", gravity_feedforward_enabled_ ? "true" : "false"));
    status.values.push_back(diagnostic_value(
      "feedforward_blend", std::to_string(feedforward_blend_)));
    status.values.push_back(diagnostic_value(
      "max_feedforward_nm", std::to_string(current_max_feedforward_nm_)));
    status.values.push_back(diagnostic_value(
      "max_feedforward_motor_index", std::to_string(current_max_feedforward_motor_index_)));
    status.values.push_back(diagnostic_value(
      "peak_feedforward_nm", std::to_string(peak_feedforward_nm_)));
    status.values.push_back(diagnostic_value(
      "feedforward_saturated", feedforward_saturated_ ? "true" : "false"));
    status.values.push_back(diagnostic_value(
      "feedforward_saturation_reason", feedforward_saturation_reason_));
    status.values.push_back(diagnostic_value(
      "max_estimated_total_command_nm", std::to_string(current_max_total_command_nm_)));
    status.values.push_back(diagnostic_value(
      "max_estimated_total_command_motor_index",
      std::to_string(current_max_total_command_motor_index_)));
    status.values.push_back(diagnostic_value(
      "peak_estimated_total_command_nm", std::to_string(peak_total_command_nm_)));
    status.values.push_back(diagnostic_value(
      "tracking_error_rad", std::to_string(current_tracking_error_rad_)));
    status.values.push_back(diagnostic_value(
      "peak_tracking_error_rad", std::to_string(peak_tracking_error_rad_)));
    status.values.push_back(diagnostic_value(
      "upper_body_state_safe", upper_body_state_safe_ ? "true" : "false"));
    status.values.push_back(diagnostic_value(
      "required_upper_body_motor_mode", std::to_string(required_upper_body_motor_mode_)));
    status.values.push_back(diagnostic_value(
      "upper_body_fault_reason", upper_body_fault_reason_));
    status.values.push_back(diagnostic_value(
      "max_measured_velocity_rad_s", std::to_string(current_max_measured_velocity_rad_s_)));
    status.values.push_back(diagnostic_value(
      "command_arm_velocity_limit_rad_s", std::to_string(max_velocity_rad_s_)));
    status.values.push_back(diagnostic_value(
      "command_waist_velocity_limit_rad_s", std::to_string(task_waist_velocity_limit_rad_s_)));
    status.values.push_back(diagnostic_value(
      "normal_measured_velocity_limit_rad_s", std::to_string(max_measured_velocity_rad_s_)));
    status.values.push_back(diagnostic_value(
      "max_velocity_motor_index", std::to_string(current_max_velocity_motor_index_)));
    status.values.push_back(diagnostic_value(
      "max_velocity_joint_name", current_max_velocity_joint_name_));
    status.values.push_back(diagnostic_value(
      "velocity_exceed_duration_s", std::to_string(measured_velocity_exceed_duration_s_)));
    status.values.push_back(diagnostic_value(
      "velocity_recovery_active", velocity_recovery_active_ ? "true" : "false"));
    status.values.push_back(diagnostic_value(
      "velocity_recovery_timed_out", velocity_recovery_timed_out_ ? "true" : "false"));
    status.values.push_back(diagnostic_value(
      "velocity_recovery_reason", velocity_recovery_reason_));
    status.values.push_back(diagnostic_value(
      "peak_measured_velocity_rad_s", std::to_string(peak_measured_velocity_rad_s_)));
    status.values.push_back(diagnostic_value(
      "peak_velocity_motor_index", std::to_string(peak_velocity_motor_index_)));
    status.values.push_back(diagnostic_value(
      "peak_velocity_joint_name", peak_velocity_joint_name_));
    status.values.push_back(diagnostic_value(
      "max_estimated_arm_torque_nm", std::to_string(current_max_estimated_arm_torque_nm_)));
    status.values.push_back(diagnostic_value(
      "peak_estimated_arm_torque_nm", std::to_string(peak_estimated_arm_torque_nm_)));
    status.values.push_back(diagnostic_value(
      "torque_overload_active", torque_overload_active_ ? "true" : "false"));
    status.values.push_back(diagnostic_value(
      "torque_overload_duration_s", std::to_string(torque_overload_duration_s_)));
    status.values.push_back(diagnostic_value(
      "torque_peak_duration_limit_s", std::to_string(torque_peak_duration_s_)));
    status.values.push_back(diagnostic_value("torque_overload_reason", torque_overload_reason_));
    status.values.push_back(diagnostic_value(
      "command_overload_active", command_overload_active_ ? "true" : "false"));
    status.values.push_back(diagnostic_value(
      "command_overload_duration_s", std::to_string(command_overload_duration_s_)));
    status.values.push_back(diagnostic_value("command_overload_reason", command_overload_reason_));
    status.values.push_back(diagnostic_value(
      "max_estimated_waist_torque_nm", std::to_string(current_max_estimated_waist_torque_nm_)));
    status.values.push_back(diagnostic_value(
      "peak_estimated_waist_torque_nm", std::to_string(peak_estimated_waist_torque_nm_)));
    status.values.push_back(diagnostic_value(
      "max_motor_temperature_c", std::to_string(current_max_motor_temperature_c_)));
    status.values.push_back(diagnostic_value(
      "motor_temperature_alarm_c", std::to_string(max_motor_temperature_c_)));
    status.values.push_back(diagnostic_value(
      "temperature_alarm_active", temperature_alarm_active_ ? "true" : "false"));
    status.values.push_back(diagnostic_value("temperature_alarm_reason", temperature_alarm_reason_));
    status.values.push_back(diagnostic_value(
      "received_trajectories", std::to_string(received_trajectories_)));
    status.values.push_back(diagnostic_value(
      "accepted_trajectories", std::to_string(accepted_trajectories_)));
    status.values.push_back(diagnostic_value(
      "rejected_trajectories", std::to_string(rejected_trajectories_)));
    status.values.push_back(diagnostic_value(
      "hardware_command_count", std::to_string(hardware_command_count_)));
    status.values.push_back(diagnostic_value("last_motion_log_path", last_motion_log_path_));
    status.values.push_back(diagnostic_value(
      "last_motion_log_end_reason", last_motion_log_end_reason_));
    status.values.push_back(diagnostic_value("last_motion_log_error", last_motion_log_error_));
    status.values.push_back(diagnostic_value("last_rejection", last_rejection_));
    array.status.push_back(std::move(status));
    lock.unlock();
    diagnostics_pub_->publish(array);
  }

  std::mutex mutex_;
  std::string state_topic_;
  std::string trajectory_topic_;
  std::string command_topic_;
  std::string preview_topic_;
  std::string diagnostics_topic_;
  std::string trajectory_log_directory_;
  bool hardware_output_requested_{};
  bool hardware_output_available_{};
  bool configuration_blocked_{};
  bool state_received_{};
  bool safety_latched_{};
  bool armed_{};
  bool releasing_{};
  bool active_trajectory_{};
  bool target_initialized_{};
  bool velocity_recovery_active_{};
  bool velocity_recovery_timed_out_{};
  bool service_check_enabled_{};
  bool service_state_received_{};
  bool service_state_validated_for_takeover_{};
  bool service_conflict_{};
  bool upper_body_state_safe_{true};
  bool temperature_alarm_active_{};
  bool feedforward_saturated_{};
  bool torque_overload_active_{};
  bool command_overload_active_{};
  bool gravity_feedforward_enabled_{};
  bool feedforward_initialized_{};
  bool waist_integral_active_{};
  bool waist_integral_saturated_{};
  double control_rate_hz_{};
  double state_timeout_s_{};
  double start_tolerance_rad_{};
  double max_velocity_rad_s_{};
  double task_waist_velocity_limit_rad_s_{};
  double max_acceleration_rad_s2_{};
  double max_command_step_rad_{};
  double joint_limit_margin_rad_{};
  double tracking_warning_rad_{};
  double tracking_pause_rad_{};
  double tracking_hard_limit_rad_{};
  double max_measured_velocity_rad_s_{};
  double measured_velocity_hard_limit_rad_s_{};
  double measured_velocity_persistence_s_{};
  double velocity_recovery_exit_rad_s_{};
  double velocity_recovery_settle_s_{};
  double velocity_recovery_timeout_s_{};
  double velocity_recovery_hard_limit_rad_s_{};
  double max_estimated_arm_torque_nm_{};
  double torque_peak_duration_s_{};
  double max_estimated_waist_torque_nm_{};
  double max_motor_temperature_c_{};
  double takeover_anchor_hold_s_{};
  double release_ramp_s_{};
  double command_weight_{};
  double command_weight_current_{};
  double arm_kp_{};
  std::array<double, 35> motor_kp_{}, motor_kd_{};
  double motion_log_hold_remaining_s_{};
  bool arm_integral_enabled_{}, arm_integral_saturated_{};
  std::vector<double> arm_integral_ki_, arm_integral_limits_nm_;
  double arm_integral_hold_rate_nm_s_{0.25};
  bool passive_arm_integral_enabled_{};
  g1_task::WaistIntegralTuning arm_integral_tuning_;
  std::array<g1_task::WaistIntegral, 35> arm_integral_controllers_{};
  std::array<double, 35> arm_integral_torque_nm_{};
  std::array<g1_task::WaistIntegralMode, 35> arm_integral_modes_{};
  double arm_kd_{};
  std::string stage_profile_{"normal"};
  std::size_t stage_first_motor_{15};
  std::uint64_t stage_revision_{};
  std::chrono::steady_clock::time_point diagnostics_last_publish_{};
  std::string diagnostics_last_phase_;
  std::uint64_t diagnostics_last_stage_revision_{};
  bool diagnostics_last_switching_{};
  std::map<rclcpp_action::GoalUUID, std::uint64_t> stage_goal_revisions_;
  double stage_kp_{}, stage_kd_{};
  double stage_wrist_kp_{};
  double stage_distal_wrist_kp_{};
  std::vector<double> stage_arm_kp_;
  double stage_waist_kp_{}, stage_waist_kd_{}, stage_from_waist_kp_{}, stage_from_waist_kd_{};
  std::array<double, 7> stage_from_kp_{}, stage_from_kd_{};
  std::chrono::steady_clock::time_point stage_started_{}, stage_last_active_{};
  double right_wrist_roll_kp_{};
  double right_wrist_roll_kd_{};
  double left_wrist_roll_kp_{};
  double left_wrist_roll_kd_{};
  double velocity_recovery_kp_{};
  double velocity_recovery_kd_{};
  double feedforward_blend_s_{};
  double tau_anchor_max_spread_nm_{};
  double feedforward_blend_{};
  double current_max_feedforward_nm_{};
  double peak_feedforward_nm_{};
  double current_max_total_command_nm_{};
  double peak_total_command_nm_{};
  double waist_kp_{};
  std::string waist_control_mode_{"guarded"};
  std::size_t waist_control_revision_{};
  unitree_hg::msg::LowCmd waist_command_{};
  bool waist_command_valid_{};
  double waist_command_elapsed_s_{};
  double waist_command_rate_hz_{50.0};
  double waist_official_blend_{};
  double waist_profile_blend_{};
  double waist_switch_from_{};
  bool waist_mode_switching_{};
  std::chrono::steady_clock::time_point waist_switch_started_{};
  double waist_kd_{};
  double waist_ki_{};
  double waist_integral_torque_limit_nm_{};
  g1_task::WaistIntegralTuning waist_integral_tuning_;
  double current_waist_integral_torque_nm_{};
  double peak_waist_integral_torque_nm_{};
  double waist_integral_gain_scale_{};
  std::uint64_t waist_gain_revision_{};
  double trajectory_progress_s_{};
  double trajectory_first_point_time_s_{};
  double trajectory_start_blend_s_{};
  double trajectory_total_duration_s_{};
  double current_tracking_error_rad_{};
  double peak_tracking_error_rad_{};
  double current_max_measured_velocity_rad_s_{};
  double peak_measured_velocity_rad_s_{};
  double measured_velocity_exceed_duration_s_{};
  double current_max_estimated_arm_torque_nm_{};
  double peak_estimated_arm_torque_nm_{};
  double current_max_estimated_waist_torque_nm_{};
  double peak_estimated_waist_torque_nm_{};
  double current_max_motor_temperature_c_{};
  double torque_overload_duration_s_{};
  double command_overload_duration_s_{};
  double service_query_period_s_{};
  double service_state_timeout_s_{};
  std::deque<std::int64_t> pending_service_request_ids_;
  std::uint16_t keys_{};
  std::uint8_t required_upper_body_motor_mode_{};
  std::uint8_t mode_machine_{};
  std::uint8_t mode_pr_{};
  bool measured_velocity_exceeding_{};
  std::size_t current_max_velocity_motor_index_{};
  std::size_t peak_velocity_motor_index_{};
  std::string phase_;
  std::string latch_reason_;
  std::string last_rejection_;
  std::string service_conflict_reason_;
  std::string upper_body_fault_reason_;
  std::string temperature_alarm_reason_;
  std::string feedforward_saturation_reason_;
  std::string torque_overload_reason_;
  std::string command_overload_reason_;
  std::string last_waist_gain_change_{"startup configuration"};
  std::string last_waist_gain_rejection_;
  std::string waist_integral_mode_{"disabled"};
  std::string current_max_velocity_joint_name_{"none"};
  std::string peak_velocity_joint_name_{"none"};
  std::string velocity_recovery_reason_;
  std::string last_motion_log_path_;
  std::string last_waist_command_log_path_;
  std::string last_motion_log_end_reason_;
  std::string last_motion_log_error_;
  std::vector<std::string> required_active_services_;
  std::vector<std::string> required_inactive_services_;
  std::vector<std::int64_t> controlled_waist_indices_;
  std::size_t service_state_count_{};
  std::size_t tau_anchor_sample_count_{};
  std::size_t current_max_feedforward_motor_index_{};
  std::size_t current_max_total_command_motor_index_{};
  std::uint64_t received_trajectories_{};
  std::uint64_t accepted_trajectories_{};
  std::uint64_t rejected_trajectories_{};
  std::uint64_t hardware_command_count_{};
  std::uint64_t motion_log_rows_{};
  std::uint64_t waist_command_log_rows_{};
  rclcpp::Time last_state_time_{0, 0, RCL_ROS_TIME};
  std::chrono::steady_clock::time_point last_state_steady_{};
  std::chrono::steady_clock::time_point last_control_steady_{};
  std::atomic<std::int64_t> state_callback_entry_ns_{0};
  double state_lock_wait_ms_{};
  double state_callback_gap_ms_{};
  double max_state_callback_gap_ms_{};
  rclcpp::CallbackGroup::SharedPtr state_group_;
  rclcpp::CallbackGroup::SharedPtr control_group_;
  double current_control_gap_ms_{};
  double max_control_gap_ms_{};
  rclcpp::Time last_service_state_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time measured_velocity_exceed_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time torque_overload_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time command_overload_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time velocity_recovery_started_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time velocity_recovery_settle_started_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time takeover_started_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_tau_history_time_{0, 0, RCL_ROS_TIME};
  std::string gravity_urdf_path_;
  std::string right_payload_fit_path_;
  std::string left_payload_fit_path_;
  std::string active_payload_fit_path_;
  std::string active_arm_;
  bool task_application_enabled_{};
  bool task_waist_enabled_{};
  bool task_dual_prepare_enabled_{};
  double task_client_watchdog_s_{};
  std::atomic<std::int64_t> task_heartbeat_ns_{0};
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr task_heartbeat_sub_;
  std::atomic<bool> task_stop_requested_{false};
  std::atomic<std::uint64_t> task_stop_epoch_{0};
  std::atomic<std::int64_t> last_task_stop_stamp_{0};
  std::map<rclcpp_action::GoalUUID, std::uint64_t> goal_stop_epochs_;
  std::map<rclcpp_action::GoalUUID, std::uint64_t> waist_goal_revisions_;
  bool task_stop_active_{}, task_zeroed_{}, task_resume_blend_{};
  double task_compliance_blend_{}, task_resume_from_{};
  rclcpp::Time task_settle_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time task_transition_since_{0, 0, RCL_ROS_TIME};
  std::array<double, 35> task_bias_{};
  g1_task::Admittance task_admittance_;
  std::atomic<bool> task_waist_admittance_requested_{false};
  bool task_waist_admittance_active_{};
  rclcpp::Service<ResumeTaskHold>::SharedPtr waist_admittance_service_;
  ArmGravityModel passive_gravity_model_;
  std::array<double, 7> passive_anchor_baseline_{};
  rclcpp::CallbackGroup::SharedPtr stop_group_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::Service<ResumeTaskHold>::SharedPtr resume_service_;
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr passive_follow_server_;
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr waist_follow_server_;
  std::array<std::size_t, 7> active_motor_indices_{};
  ArmGravityModel active_gravity_model_;
  std::array<double, 14> actual_arm_position_{};
  std::array<double, 14> actual_arm_velocity_{};
  std::array<std::uint8_t, 14> arm_motor_mode_{};
  std::array<double, 35> actual_motor_position_{};
  std::array<double, 35> actual_motor_velocity_{};
  std::array<double, 35> actual_motor_tau_est_{};
  std::array<double, 35> hold_motor_position_{};
  std::array<double, 35> tau_est_anchor_{};
  std::array<double, 35> waist_integral_torque_nm_{};
  std::array<g1_task::WaistIntegral, 35> waist_integral_controllers_{};
  std::array<std::deque<double>, 35> tau_est_history_{};
  std::array<double, 7> feedforward_anchor_baseline_{};
  std::array<double, 3> current_imu_rpy_{};
  std::vector<double> trajectory_start_positions_;
  std::vector<double> previous_target_;
  std::vector<double> previous_target_velocity_;
  double previous_trajectory_time_scale_{1.0};
  double trajectory_time_scale_rate_{};
  std::vector<double> previous_target_acceleration_;
  std::ofstream motion_log_;
  std::ofstream waist_command_log_;
  trajectory_msgs::msg::JointTrajectory trajectory_;
  std::vector<std::size_t> active_arm_indices_;
  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr state_sub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_sub_;
  rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr command_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr preview_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr service_request_pub_;
  rclcpp::Subscription<unitree_api::msg::Response>::SharedPtr service_response_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr arm_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disarm_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::Service<ResumeTaskHold>::SharedPtr reset_task_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_service_;
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr follow_action_server_;
  std::shared_ptr<FollowGoalHandle> active_follow_goal_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  rclcpp::TimerBase::SharedPtr service_query_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

int main(int argc, char** argv)
{
  if ((argc == 4 || argc == 5) && std::string(argv[1]) == "--check-feedforward")
  {
    const std::string side = argc == 5 ? argv[4] : "right";
    if (side != "left" && side != "right") return 1;
    const std::size_t first_motor = side == "left" ? 15 : 22;
    ArmGravityModel model;
    std::string reason;
    if (!model.initialize(argv[2], argv[3], side, reason))
    {
      std::cerr << "FEEDFORWARD CHECK FAILED: " << reason << std::endl;
      return 1;
    }
    const std::array<std::array<double, 7>, 3> poses{{
      {{0.0, -0.20, 0.0, 0.70, 0.0, 0.0, 0.0}},
      {{0.35, -0.55, 0.20, 1.10, -0.15, 0.25, 0.10}},
      {{-0.30, 0.25, -0.25, 0.45, 0.20, -0.20, -0.10}}
    }};
    double peak = 0.0;
    std::size_t peak_joint = 0;
    for (const auto& pose : poses)
    {
      std::array<double, 7> torque{};
      if (!model.identified(pose, {0.03, -0.04, 0.10}, torque))
      {
        std::cerr << "FEEDFORWARD CHECK FAILED: KDL torque calculation failed" << std::endl;
        return 1;
      }
      for (std::size_t joint = 0; joint < 7; ++joint)
      {
        if (!std::isfinite(torque[joint]) ||
            std::abs(torque[joint]) > kArmContinuousTorqueLimits[joint])
        {
          std::cerr << "FEEDFORWARD CHECK FAILED: motor " << first_motor + joint
                    << " predicted tau=" << torque[joint]
                    << " exceeds continuous limit=" << kArmContinuousTorqueLimits[joint]
                    << std::endl;
          return 1;
        }
        if (std::abs(torque[joint]) > peak)
        {
          peak = std::abs(torque[joint]);
          peak_joint = first_motor + joint;
        }
      }
    }
    std::cout << "FEEDFORWARD CHECK OK: exact gateway KDL/payload model evaluated 3 poses; "
              << "peak=" << peak << " Nm at motor " << peak_joint
              << "; no ROS or DDS node was created." << std::endl;
    return 0;
  }
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  auto gateway = std::make_shared<ArmSdkHardwareGateway>();
  executor.add_node(gateway);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
