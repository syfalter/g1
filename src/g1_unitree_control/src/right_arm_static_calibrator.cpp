#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/SVD>
#include <kdl/chaindynparam.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/rigidbodyinertia.hpp>
#include <kdl/segment.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <unitree_hg/msg/low_state.hpp>

using namespace std::chrono_literals;

namespace
{
constexpr std::array<std::size_t, 7> kRightMotorIndices{22, 23, 24, 25, 26, 27, 28};
constexpr std::array<const char*, 7> kRightJointNames{
  "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
  "right_shoulder_yaw_joint", "right_elbow_joint",
  "right_wrist_roll_joint", "right_wrist_pitch_joint",
  "right_wrist_yaw_joint"};
constexpr double kGravity = 9.81;

struct RawSample
{
  std::int64_t stamp_ns = 0;
  std::array<double, 7> q{};
  std::array<double, 7> dq{};
  std::array<double, 7> tau{};
  std::array<double, 3> rpy{};
  std::uint8_t mode_machine = 0;
};

struct Observation
{
  std::int64_t stamp_ns = 0;
  std::array<double, 7> q{};
  std::array<double, 7> tau{};
  std::array<double, 3> rpy{};
};

double median(std::vector<double> values)
{
  if (values.empty())
    return std::numeric_limits<double>::quiet_NaN();
  const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
  std::nth_element(values.begin(), middle, values.end());
  return *middle;
}

double percentile(std::vector<double> values, double fraction)
{
  if (values.empty())
    return std::numeric_limits<double>::quiet_NaN();
  const auto index = static_cast<std::size_t>(std::clamp(fraction, 0.0, 1.0) *
    static_cast<double>(values.size() - 1));
  const auto element = values.begin() + static_cast<std::ptrdiff_t>(index);
  std::nth_element(values.begin(), element, values.end());
  return *element;
}

std::vector<std::string> split_csv(const std::string& line)
{
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ','))
    fields.push_back(field);
  return fields;
}
}  // namespace

class RightArmStaticCalibrator : public rclcpp::Node
{
public:
  RightArmStaticCalibrator() : Node("right_arm_static_calibrator")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/lowstate");
    urdf_path_ = declare_parameter<std::string>("urdf_path",
      "/home/syf/unitree_ws/unitree_ros/robots/g1_description/"
      "g1_29dof_lock_waist_rev_1_0.urdf");
    base_link_ = declare_parameter<std::string>("base_link", "torso_link");
    tip_link_ = declare_parameter<std::string>("tip_link", "right_wrist_yaw_link");
    output_csv_ = declare_parameter<std::string>("output_csv",
      "/home/syf/g1_upper_body_ws/calibration/right_arm_static_samples.csv");
    output_yaml_ = declare_parameter<std::string>("output_yaml",
      "/home/syf/g1_upper_body_ws/calibration/right_arm_payload_fit.yaml");
    sample_window_s_ = std::clamp(declare_parameter("sample_window_s", 1.5), 0.5, 5.0);
    max_abs_dq_ = std::clamp(declare_parameter("max_abs_dq", 0.03), 0.001, 0.2);
    max_q_span_ = std::clamp(declare_parameter("max_q_span", 0.008), 0.001, 0.05);
    max_tau_spread_ = std::clamp(
      declare_parameter("max_tau_spread", 0.75), 0.05, 3.0);
    minimum_pose_distance_rad_ = std::clamp(
      declare_parameter("minimum_pose_distance_rad", 0.20), 0.05, 1.0);
    minimum_fit_poses_ = static_cast<int>(std::clamp<std::int64_t>(
      declare_parameter("minimum_fit_poses", 8), 4, 30));
    expected_mode_machine_ = declare_parameter("expected_mode_machine", 6);

    if (!initialize_kdl())
      throw std::runtime_error("failed to initialize right-arm KDL model");
    load_observations();

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    state_sub_ = create_subscription<unitree_hg::msg::LowState>(
      input_topic_, qos,
      std::bind(&RightArmStaticCalibrator::state_callback, this, std::placeholders::_1));
    capture_service_ = create_service<std_srvs::srv::Trigger>(
      "/g1/right_arm_calibration/capture",
      std::bind(&RightArmStaticCalibrator::capture_callback, this,
        std::placeholders::_1, std::placeholders::_2));
    fit_service_ = create_service<std_srvs::srv::Trigger>(
      "/g1/right_arm_calibration/fit",
      std::bind(&RightArmStaticCalibrator::fit_callback, this,
        std::placeholders::_1, std::placeholders::_2));
    status_service_ = create_service<std_srvs::srv::Trigger>(
      "/g1/right_arm_calibration/status",
      std::bind(&RightArmStaticCalibrator::status_callback, this,
        std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(),
      "READ-ONLY right-arm calibrator: %s, chain %s -> %s, loaded poses=%zu",
      input_topic_.c_str(), base_link_.c_str(), tip_link_.c_str(), observations_.size());
    RCLCPP_INFO(get_logger(), "No command publisher is created by this node.");
  }

private:
  bool initialize_kdl()
  {
    KDL::Tree tree;
    if (!kdl_parser::treeFromFile(urdf_path_, tree))
    {
      RCLCPP_ERROR(get_logger(), "Failed to parse URDF: %s", urdf_path_.c_str());
      return false;
    }
    if (!tree.getChain(base_link_, tip_link_, arm_chain_))
    {
      RCLCPP_ERROR(get_logger(), "Failed to extract KDL chain %s -> %s",
        base_link_.c_str(), tip_link_.c_str());
      return false;
    }
    if (arm_chain_.getNrOfJoints() != kRightMotorIndices.size())
    {
      RCLCPP_ERROR(get_logger(), "Expected 7 right-arm joints, KDL chain has %u",
        arm_chain_.getNrOfJoints());
      return false;
    }
    return true;
  }

  void state_callback(const unitree_hg::msg::LowState::SharedPtr message)
  {
    const auto now_time = now();
    if (last_buffer_time_.nanoseconds() != 0 &&
        (now_time - last_buffer_time_).seconds() < 0.018)
      return;

    RawSample sample;
    sample.stamp_ns = now_time.nanoseconds();
    sample.mode_machine = message->mode_machine;
    for (std::size_t i = 0; i < kRightMotorIndices.size(); ++i)
    {
      const auto& motor = message->motor_state.at(kRightMotorIndices.at(i));
      sample.q.at(i) = motor.q;
      sample.dq.at(i) = motor.dq;
      sample.tau.at(i) = motor.tau_est;
    }
    for (std::size_t i = 0; i < 3; ++i)
      sample.rpy.at(i) = message->imu_state.rpy.at(i);

    std::lock_guard<std::mutex> lock(mutex_);
    buffer_.push_back(sample);
    const auto oldest = sample.stamp_ns - static_cast<std::int64_t>(5.0e9);
    while (!buffer_.empty() && buffer_.front().stamp_ns < oldest)
      buffer_.pop_front();
    last_buffer_time_ = now_time;
  }

  bool make_observation(Observation& observation, std::string& reason)
  {
    std::deque<RawSample> buffer;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      buffer = buffer_;
    }
    if (buffer.empty())
    {
      reason = "no LowState samples";
      return false;
    }
    const auto newest = buffer.back().stamp_ns;
    const auto cutoff = newest - static_cast<std::int64_t>(sample_window_s_ * 1.0e9);
    std::vector<RawSample> window;
    for (const auto& sample : buffer)
      if (sample.stamp_ns >= cutoff)
        window.push_back(sample);
    if (window.size() < static_cast<std::size_t>(sample_window_s_ * 35.0))
    {
      reason = "insufficient samples in static window";
      return false;
    }
    if ((now().nanoseconds() - newest) > static_cast<std::int64_t>(0.1e9))
    {
      reason = "LowState is stale";
      return false;
    }

    observation.stamp_ns = newest;
    for (const auto& sample : window)
    {
      if (sample.mode_machine != expected_mode_machine_)
      {
        reason = "unexpected mode_machine in capture window";
        return false;
      }
      for (const auto dq : sample.dq)
      {
        if (!std::isfinite(dq) || std::abs(dq) > max_abs_dq_)
        {
          reason = "right arm is not stationary";
          return false;
        }
      }
    }

    for (std::size_t joint = 0; joint < 7; ++joint)
    {
      std::vector<double> q_values;
      std::vector<double> tau_values;
      q_values.reserve(window.size());
      tau_values.reserve(window.size());
      for (const auto& sample : window)
      {
        q_values.push_back(sample.q.at(joint));
        tau_values.push_back(sample.tau.at(joint));
      }
      const double q_span = percentile(q_values, 0.9) - percentile(q_values, 0.1);
      const double tau_spread = percentile(tau_values, 0.9) - percentile(tau_values, 0.1);
      if (q_span > max_q_span_ || tau_spread > max_tau_spread_)
      {
        reason = "unstable joint " + std::string(kRightJointNames.at(joint)) +
          " (q_span=" + std::to_string(q_span) +
          ", tau_spread=" + std::to_string(tau_spread) + ")";
        return false;
      }
      observation.q.at(joint) = median(std::move(q_values));
      observation.tau.at(joint) = median(std::move(tau_values));
    }
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
      std::vector<double> values;
      values.reserve(window.size());
      for (const auto& sample : window)
        values.push_back(sample.rpy.at(axis));
      observation.rpy.at(axis) = median(std::move(values));
    }
    return true;
  }

  KDL::Vector gravity_in_torso(const Observation& observation) const
  {
    const auto world_from_torso = KDL::Rotation::RPY(
      observation.rpy.at(0), observation.rpy.at(1), observation.rpy.at(2));
    return world_from_torso.Inverse(KDL::Vector(0.0, 0.0, -kGravity));
  }

  bool gravity_for_chain(const KDL::Chain& chain, const Observation& observation,
                         Eigen::VectorXd& torque) const
  {
    KDL::JntArray q(chain.getNrOfJoints());
    for (std::size_t i = 0; i < observation.q.size(); ++i)
      q(static_cast<unsigned int>(i)) = observation.q.at(i);
    KDL::JntArray result(chain.getNrOfJoints());
    KDL::ChainDynParam dynamics(chain, gravity_in_torso(observation));
    if (dynamics.JntToGravity(q, result) != 0)
      return false;
    torque.resize(static_cast<Eigen::Index>(result.rows()));
    for (unsigned int i = 0; i < result.rows(); ++i)
      torque(static_cast<Eigen::Index>(i)) = result(i);
    return true;
  }

  KDL::Chain chain_with_unit_payload(const KDL::Vector& center_of_mass) const
  {
    auto chain = arm_chain_;
    chain.addSegment(KDL::Segment("identified_payload", KDL::Joint(KDL::Joint::Fixed),
      KDL::Frame::Identity(),
      KDL::RigidBodyInertia(1.0, center_of_mass, KDL::RotationalInertia::Zero())));
    return chain;
  }

  bool regression_for(const Observation& observation, Eigen::VectorXd& baseline,
                      Eigen::Matrix<double, 7, 4>& regressor) const
  {
    if (!gravity_for_chain(arm_chain_, observation, baseline))
      return false;
    Eigen::VectorXd at_origin, at_x, at_y, at_z;
    if (!gravity_for_chain(chain_with_unit_payload(KDL::Vector::Zero()), observation,
                          at_origin) ||
        !gravity_for_chain(chain_with_unit_payload(KDL::Vector(1.0, 0.0, 0.0)),
                          observation, at_x) ||
        !gravity_for_chain(chain_with_unit_payload(KDL::Vector(0.0, 1.0, 0.0)),
                          observation, at_y) ||
        !gravity_for_chain(chain_with_unit_payload(KDL::Vector(0.0, 0.0, 1.0)),
                          observation, at_z))
      return false;
    regressor.col(0) = at_origin - baseline;
    regressor.col(1) = at_x - at_origin;
    regressor.col(2) = at_y - at_origin;
    regressor.col(3) = at_z - at_origin;
    return true;
  }

  bool fit_payload(std::string& report)
  {
    if (observations_.size() < static_cast<std::size_t>(minimum_fit_poses_))
    {
      report = "saved poses=" + std::to_string(observations_.size()) +
        "; need at least " + std::to_string(minimum_fit_poses_) + " before fitting";
      return false;
    }
    const auto rows = static_cast<Eigen::Index>(observations_.size() * 7);
    Eigen::MatrixXd design(rows, 4);
    Eigen::VectorXd measured(rows);
    Eigen::VectorXd baseline_all(rows);
    for (std::size_t pose = 0; pose < observations_.size(); ++pose)
    {
      Eigen::VectorXd baseline;
      Eigen::Matrix<double, 7, 4> regressor;
      if (!regression_for(observations_.at(pose), baseline, regressor))
      {
        report = "KDL gravity calculation failed at pose " + std::to_string(pose);
        return false;
      }
      const auto row = static_cast<Eigen::Index>(pose * 7);
      design.block<7, 4>(row, 0) = regressor;
      baseline_all.segment<7>(row) = baseline;
      for (std::size_t joint = 0; joint < 7; ++joint)
        measured(row + static_cast<Eigen::Index>(joint)) =
          observations_.at(pose).tau.at(joint);
    }
    const Eigen::VectorXd residual = measured - baseline_all;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      design, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::Vector4d parameters = svd.solve(residual);
    const auto singular = svd.singularValues();
    const double condition = singular.size() == 0 || singular(singular.size() - 1) <= 1e-9
      ? std::numeric_limits<double>::infinity()
      : singular(0) / singular(singular.size() - 1);
    const Eigen::VectorXd prediction = baseline_all + design * parameters;
    const Eigen::VectorXd error = measured - prediction;
    const double rmse = std::sqrt(error.squaredNorm() / static_cast<double>(error.size()));
    const double mass = parameters(0);
    if (!std::isfinite(mass) || mass <= 0.01)
    {
      report = "unphysical fitted payload mass=" + std::to_string(mass) +
        " kg; collect more diverse poses and verify IMU gravity orientation";
      return false;
    }
    const Eigen::Vector3d center = parameters.tail<3>() / mass;
    if (!center.allFinite() || center.norm() > 0.50)
    {
      report = "unphysical fitted payload COM norm=" +
        std::to_string(center.norm()) +
        " m; collect more diverse poses and verify payload frame";
      return false;
    }

    std::filesystem::create_directories(std::filesystem::path(output_yaml_).parent_path());
    std::ofstream output(output_yaml_, std::ios::trunc);
    if (!output)
    {
      report = "failed to write fit file: " + output_yaml_;
      return false;
    }
    output << std::setprecision(12)
           << "base_link: " << base_link_ << '\n'
           << "payload_frame: " << tip_link_ << '\n'
           << "pose_count: " << observations_.size() << '\n'
           << "mass_kg: " << mass << '\n'
           << "com_xyz_m: [" << center.x() << ", " << center.y() << ", "
           << center.z() << "]\n"
           << "first_moment_kg_m: [" << parameters(1) << ", " << parameters(2)
           << ", " << parameters(3) << "]\n"
           << "training_rmse_nm: " << rmse << '\n'
           << "regressor_condition_number: " << condition << '\n';

    std::ostringstream stream;
    stream << std::fixed << std::setprecision(5)
           << "poses=" << observations_.size() << " mass=" << mass
           << " kg com_wrist=[" << center.x() << ',' << center.y() << ','
           << center.z() << "] m rmse=" << rmse << " Nm condition=" << condition
           << " output=" << output_yaml_;
    report = stream.str();
    return true;
  }

  void capture_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    Observation observation;
    std::string reason;
    if (!make_observation(observation, reason))
    {
      response->success = false;
      response->message = "capture rejected: " + reason;
      return;
    }
    for (std::size_t pose = 0; pose < observations_.size(); ++pose)
    {
      double squared_distance = 0.0;
      for (std::size_t joint = 0; joint < 7; ++joint)
      {
        const double delta = observation.q.at(joint) -
          observations_.at(pose).q.at(joint);
        squared_distance += delta * delta;
      }
      const double distance = std::sqrt(squared_distance);
      if (distance < minimum_pose_distance_rad_)
      {
        response->success = false;
        response->message = "capture rejected: pose is too similar to saved pose " +
          std::to_string(pose + 1) + " (joint-space distance=" +
          std::to_string(distance) + " rad)";
        return;
      }
    }

    Eigen::VectorXd baseline;
    Eigen::Matrix<double, 7, 4> regressor;
    if (!regression_for(observation, baseline, regressor))
    {
      response->success = false;
      response->message = "capture rejected: KDL gravity calculation failed";
      return;
    }
    std::ostringstream gravity_report;
    gravity_report << std::fixed << std::setprecision(4) << "KDL_BASE_RESIDUAL";
    for (std::size_t joint = 0; joint < 7; ++joint)
      gravity_report << " j" << kRightMotorIndices.at(joint)
                     << "_base=" << baseline(static_cast<Eigen::Index>(joint))
                     << "_measured=" << observation.tau.at(joint)
                     << "_residual=" << observation.tau.at(joint) -
                        baseline(static_cast<Eigen::Index>(joint));
    RCLCPP_INFO(get_logger(), "%s", gravity_report.str().c_str());

    observations_.push_back(observation);
    if (!append_observation(observation))
    {
      observations_.pop_back();
      response->success = false;
      response->message = "capture failed: could not append CSV";
      return;
    }
    std::string fit_report;
    const bool fit_ok = fit_payload(fit_report);
    response->success = true;
    response->message = "captured pose " + std::to_string(observations_.size()) +
      "; " + fit_report;
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
    if (!fit_ok)
      RCLCPP_INFO(get_logger(), "Fit deferred/rejected without affecting saved sample.");
  }

  void fit_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    response->success = fit_payload(response->message);
  }

  void status_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    response->success = !buffer_.empty();
    response->message = "saved_poses=" + std::to_string(observations_.size()) +
      " buffered_samples=" + std::to_string(buffer_.size()) +
      " csv=" + output_csv_;
  }

  bool append_observation(const Observation& observation)
  {
    const auto path = std::filesystem::path(output_csv_);
    std::filesystem::create_directories(path.parent_path());
    const bool new_file = !std::filesystem::exists(path) ||
      std::filesystem::file_size(path) == 0;
    std::ofstream output(path, std::ios::app);
    if (!output)
      return false;
    if (new_file)
    {
      output << "stamp_ns";
      for (const auto* name : kRightJointNames)
        output << ',' << name << "_q";
      for (const auto* name : kRightJointNames)
        output << ',' << name << "_tau_est";
      output << ",imu_roll,imu_pitch,imu_yaw\n";
    }
    output << std::setprecision(12) << observation.stamp_ns;
    for (const auto value : observation.q)
      output << ',' << value;
    for (const auto value : observation.tau)
      output << ',' << value;
    for (const auto value : observation.rpy)
      output << ',' << value;
    output << '\n';
    return true;
  }

  void load_observations()
  {
    std::ifstream input(output_csv_);
    if (!input)
      return;
    std::string line;
    std::getline(input, line);
    while (std::getline(input, line))
    {
      const auto fields = split_csv(line);
      if (fields.size() != 18)
        continue;
      try
      {
        Observation observation;
        observation.stamp_ns = std::stoll(fields.at(0));
        for (std::size_t i = 0; i < 7; ++i)
          observation.q.at(i) = std::stod(fields.at(1 + i));
        for (std::size_t i = 0; i < 7; ++i)
          observation.tau.at(i) = std::stod(fields.at(8 + i));
        for (std::size_t i = 0; i < 3; ++i)
          observation.rpy.at(i) = std::stod(fields.at(15 + i));
        observations_.push_back(observation);
      }
      catch (const std::exception&)
      {
        RCLCPP_WARN(get_logger(), "Skipped malformed calibration CSV row");
      }
    }
  }

  std::string input_topic_;
  std::string urdf_path_;
  std::string base_link_;
  std::string tip_link_;
  std::string output_csv_;
  std::string output_yaml_;
  double sample_window_s_ = 1.5;
  double max_abs_dq_ = 0.03;
  double max_q_span_ = 0.008;
  double max_tau_spread_ = 0.75;
  double minimum_pose_distance_rad_ = 0.20;
  int minimum_fit_poses_ = 8;
  int expected_mode_machine_ = 6;
  KDL::Chain arm_chain_;
  std::mutex mutex_;
  std::deque<RawSample> buffer_;
  std::vector<Observation> observations_;
  rclcpp::Time last_buffer_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr state_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr capture_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr fit_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr status_service_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try
  {
    rclcpp::spin(std::make_shared<RightArmStaticCalibrator>());
  }
  catch (const std::exception& error)
  {
    std::cerr << "right_arm_static_calibrator fatal: " << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
