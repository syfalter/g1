// Reuse the already hardware-tested preflight, state and command helpers.
#define main g1_arm_sdk_safe_probe_embedded_main
#include "g1_arm_sdk_safe_probe.cpp"
#undef main

#include <deque>
#include <filesystem>
#include <fstream>
#include <poll.h>
#include <sstream>

#include <Eigen/Core>
#include <Eigen/SVD>
#include <kdl/chaindynparam.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/rigidbodyinertia.hpp>
#include <kdl/segment.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

namespace calibration
{
using namespace std::chrono_literals;
#ifdef G1_CALIBRATION_LEFT_ARM
constexpr char kActiveArm[] = "left";
constexpr char kActiveWristLink[] = "left_wrist_yaw_link";
constexpr std::array<std::size_t, 7> kActiveIndices{15, 16, 17, 18, 19, 20, 21};
#else
constexpr char kActiveArm[] = "right";
constexpr char kActiveWristLink[] = "right_wrist_yaw_link";
constexpr std::array<std::size_t, 7> kActiveIndices{22, 23, 24, 25, 26, 27, 28};
#endif
constexpr double kTickSeconds = 0.02;
constexpr double kPeakCommandVelocity = 0.10;
constexpr double kMinimumMoveSeconds = 5.0;
constexpr double kStableWindowSeconds = 1.5;
constexpr double kStableTimeoutSeconds = 12.0;
constexpr double kStableMaxDq = 0.03;
constexpr double kStableQSpan = 0.008;
constexpr double kStableTauSpread = 0.75;
// Leave enough time for the operator to select NEXT and move back to a
// low-load pose before the official 85/120 C hard protection is reached.
constexpr double kCasingWarningTemperature = 80.0;
constexpr double kWindingWarningTemperature = 110.0;
constexpr double kAdmittanceKp = 2.0;
constexpr double kAdmittanceKd = 1.0;
constexpr double kAdmittanceTorqueDeadband = 0.15;
constexpr double kAdmittanceVelocityPerTorque = 0.50;
constexpr double kAdmittanceMaxVelocity = 0.12;
constexpr double kAdmittanceMaxAcceleration = 0.30;
constexpr double kAdmittanceFilterSeconds = 0.08;
constexpr double kAdmittanceTargetLeadSeconds = 0.12;
constexpr double kAdmittanceTargetLead = 1.0 * M_PI / 180.0;
constexpr double kAdmittanceTorqueReserve = 0.75;
// First full-arm sensorless-admittance profile.  Shoulder axes need a wider
// residual-torque deadband and slower virtual motion because their gravity and
// cross-axis coupling errors are larger.  Wrist axes use smaller gains and
// faster virtual motion so their much smaller user torques remain observable.
constexpr std::array<double, 7> kFullAdmittanceKp{2.0, 2.0, 2.0, 2.0, 2.0, 1.5, 1.5};
constexpr double kDistalHoldAttackSeconds = 0.25;
constexpr double kDistalHoldReleaseSeconds = 0.10;
constexpr double kDistalHoldIntegralGain = 2.0;
constexpr double kDistalHoldIntegralLimit = 0.18;
constexpr double kDistalHoldIntegralDeadband = 0.25 * M_PI / 180.0;
constexpr double kDistalHoldIntegralMaxVelocity = 0.15;
constexpr double kDistalHoldIntegralReleaseSeconds = 0.15;
constexpr double kDistalTrackingPersistenceSeconds = 0.50;
constexpr std::array<double, 7> kFullAdmittanceKd{0.45, 0.45, 0.45, 0.40,
                                                 0.25, 0.25, 0.20};
constexpr std::array<double, 7> kFullAdmittanceHighSpeedKd{1.60, 1.60, 1.60, 1.30,
                                                          0.80, 0.75, 0.65};
constexpr std::array<double, 7> kFullAdmittanceDeadband{0.08, 0.06, 0.08, 0.06,
                                                       0.06, 0.05, 0.04};
constexpr std::array<double, 7> kFullAdmittanceFilterSeconds{0.08, 0.08, 0.08, 0.10,
                                                            0.12, 0.08, 0.08};
constexpr double kFullAdmittanceOffThresholdRatio = 0.50;
constexpr std::array<double, 7> kFullAdmittanceVelocityPerTorque{0.60, 0.80, 0.70, 0.80,
                                                                1.30, 1.00, 1.20};
constexpr std::array<double, 7> kFullAdmittanceMaxVelocity{0.20, 0.20, 0.20, 0.25,
                                                          0.25, 0.30, 0.30};
constexpr std::array<double, 7> kFullAdmittanceMaxAcceleration{0.60, 0.60, 0.60, 0.80,
                                                              1.20, 1.00, 1.00};
constexpr std::array<double, 7> kFullAdmittanceDampingStartVelocity{
  0.30, 0.30, 0.30, 0.30, 0.35, 0.30, 0.30};
constexpr std::array<double, 7> kFullAdmittanceDampingFullVelocity{
  0.90, 0.90, 0.90, 0.90, 0.90, 0.90, 0.90};
constexpr std::array<double, 7> kFullAdmittanceBrakeStartVelocity{
  0.80, 0.80, 0.80, 0.80, 0.70, 0.80, 0.80};
constexpr std::array<double, 7> kFullAdmittanceBrakeFullVelocity{
  1.20, 1.20, 1.20, 1.20, 1.20, 1.20, 1.20};
constexpr std::array<double, 7> kFullAdmittanceVelocityFilterSeconds{
  0.10, 0.10, 0.10, 0.10, 0.12, 0.10, 0.10};
constexpr std::array<double, 7> kFullAdmittanceSpeedEnvelopeReleaseSeconds{
  0.12, 0.12, 0.12, 0.12, 0.05, 0.12, 0.12};
constexpr std::array<double, 7> kFullAdmittanceVelocityPassthrough{
  0.65, 0.65, 0.65, 0.65, 0.35, 0.65, 0.65};
constexpr double kFullAdmittancePassthroughFadeStart = 0.20;
constexpr double kFullAdmittancePassthroughFadeEnd = 0.50;
// Low gains intentionally permit a few degrees of sag while taking over and
// zeroing. Once admittance is active, retain the original 3 degree tracking
// guard, but reject only sustained excursions; a separate 6 degree bound
// remains an immediate hard fault.
constexpr double kAdmittanceSetupTrackingError = 5.0 * M_PI / 180.0;
constexpr double kAdmittanceHardTrackingError = 6.0 * M_PI / 180.0;
constexpr double kAdmittanceTrackingPersistenceSeconds = 0.20;
// J24 and J26 repeatedly show coarse, direction-dependent breakaway. The
// bounded assist is present only before motion and is removed rapidly after
// breakaway. It is deliberately much smaller than either joint's torque limit.
constexpr std::array<double, 7> kBreakawayTorque{0.0, 0.0, 0.08, 0.0,
                                                 0.08, 0.0, 0.0};
constexpr double kBreakawayAttackRate = 0.80;
constexpr double kBreakawayReleaseRate = 1.60;
constexpr double kBreakawayFadeStartVelocity = 0.025;
constexpr double kBreakawayFadeEndVelocity = 0.10;
// q_ref may move only while the operator holds the deadman and residual torque
// crosses its hysteretic gate. Measured velocity alone is never evidence of
// human intent: the 2026-08-31 j26 fall demonstrated that gravity breakaway can
// otherwise become a self-reinforcing reference follower.
constexpr double kReferenceMaxCommandVelocity = 0.35;
constexpr std::array<double, 7> kReferenceMaxError{
  1.5 * M_PI / 180.0, 1.5 * M_PI / 180.0, 1.5 * M_PI / 180.0,
  2.0 * M_PI / 180.0, 2.0 * M_PI / 180.0,
  2.0 * M_PI / 180.0, 2.0 * M_PI / 180.0};
constexpr double kReferenceSoftBrakeError = 3.0 * M_PI / 180.0;
// Calibration uses measured q, so a small, static gravity sag is valid data.
// Keep this just below the independent 3 degree hard tracking protection.
constexpr double kCaptureTrackingLimit = 2.8 * M_PI / 180.0;
constexpr double kStartPerJointTolerance = 5.0 * M_PI / 180.0;
constexpr double kMaxActiveFeedforward = 8.0;
constexpr std::array<double, 7> kActiveTauEstLimits{12.0, 12.0, 12.0, 12.0,
                                                   12.0, 4.0, 4.0};
constexpr std::array<double, 7> kActiveFeedforwardLimits{8.0, 8.0, 8.0, 8.0,
                                                        8.0, 4.0, 4.0};
const std::array<double, 35> kCalibrationTakeoverLimits = [] {
  std::array<double, 35> limits{};
  limits.fill(kMaxArmTauFeedforward);
  for (std::size_t j = 0; j < kActiveIndices.size(); ++j)
    limits[kActiveIndices[j]] = kActiveFeedforwardLimits[j];
  return limits;
}();
constexpr double kDragVelocityWarning = 2.20;
constexpr double kDragVelocityNominalLimit = 2.40;
constexpr double kDragVelocityAbsoluteLimit = 2.50;
constexpr double kDragVelocityPersistenceSeconds = 0.50;
#ifdef G1_CALIBRATION_LEFT_ARM
constexpr std::array<double, 7> kActiveJointLower{-3.0892, -1.5882, -2.618,
  -1.0472, -1.972222054, -1.614429558, -1.614429558};
constexpr std::array<double, 7> kActiveJointUpper{2.6704, 2.2515, 2.618,
  2.0944, 1.972222054, 1.614429558, 1.614429558};
#else
constexpr std::array<double, 7> kActiveJointLower{-3.0892, -2.2515, -2.618,
  -1.0472, -1.972222054, -1.614429558, -1.614429558};
constexpr std::array<double, 7> kActiveJointUpper{2.6704, 1.5882, 2.618,
  2.0944, 1.972222054, 1.614429558, 1.614429558};
#endif
constexpr double kJointLimitMargin = 8.0 * M_PI / 180.0;
constexpr double kJointLimitPredictionLatency = 0.12;
constexpr double kJointLimitBrakeAcceleration = 3.0;
constexpr char kCalibrationAuthorization[] = "I_UNDERSTAND_12_POSE_CALIBRATION";
constexpr std::uint16_t kRemoteF1Mask = 0x0040;
// The physical F3 button was measured read-only on this robot as the SDK F2 bit.
constexpr std::uint16_t kRemoteDeadmanMask = 0x0080;
constexpr std::uint16_t kRemoteAMask = 0x0100;
constexpr std::uint16_t kRemoteXMask = 0x0400;
constexpr std::uint16_t kRemoteToggleMask = kRemoteF1Mask | kRemoteXMask;
constexpr std::uint16_t kRemoteRecordMask = kRemoteF1Mask | kRemoteAMask;
constexpr double kRemoteToggleHoldSeconds = 0.80;
constexpr double kRemoteToggleReleaseSeconds = 0.25;
constexpr double kDeadmanReleaseSeconds = 0.06;
constexpr double kBrakeMinimumSeconds = 0.25;
constexpr double kBrakeMaximumSeconds = 0.75;
constexpr double kBrakeSettledVelocity = 0.08;
constexpr double kBrakeCommandAcceleration = 3.0;
constexpr double kGainKpSlewPerSecond = 80.0;
constexpr double kGainKdSlewPerSecond = 12.0;
constexpr std::array<double, 7> kDeadmanHoldKp{30.0, 30.0, 25.0, 25.0,
                                               20.0, 15.0, 15.0};
constexpr std::array<double, 7> kDeadmanHoldKd{3.0, 3.0, 2.5, 2.0,
                                               1.5, 1.2, 1.0};

using Pose = std::array<double, 7>;

enum class DragControlState
{
  HOLD,
  DRAG,
  BRAKE
};

const char* drag_state_name(DragControlState state)
{
  switch (state)
  {
    case DragControlState::HOLD: return "HOLD";
    case DragControlState::DRAG: return "DRAG";
    case DragControlState::BRAKE: return "BRAKE";
  }
  return "UNKNOWN";
}

std::uint16_t remote_keys(const LowState& state)
{
  return static_cast<std::uint16_t>(state.wireless_remote().at(2)) |
    (static_cast<std::uint16_t>(state.wireless_remote().at(3)) << 8U);
}

struct Observation
{
  std::int64_t stamp_ns{};
  Pose q{};
  Pose tau{};
  std::array<double, 3> rpy{};
  Pose casing_temperature{};
  Pose winding_temperature{};
};

double median(std::vector<double> values)
{
  const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
  std::nth_element(values.begin(), middle, values.end());
  return *middle;
}

double percentile(std::vector<double> values, double fraction)
{
  const auto index = static_cast<std::size_t>(fraction * (values.size() - 1));
  const auto element = values.begin() + static_cast<std::ptrdiff_t>(index);
  std::nth_element(values.begin(), element, values.end());
  return *element;
}

bool load_candidates(const std::string& path, std::vector<Pose>& poses,
                     std::string& reason, std::size_t expected_count = 12)
{
  std::ifstream input(path);
  if (!input)
  {
    reason = "cannot open candidate file: " + path;
    return false;
  }
  std::string line;
  while (std::getline(input, line))
  {
    const auto marker = line.find("q: [");
    if (marker == std::string::npos) continue;
    const auto begin = line.find('[', marker);
    const auto end = line.find(']', begin);
    if (begin == std::string::npos || end == std::string::npos)
    {
      reason = "malformed q row in candidate file";
      return false;
    }
    std::stringstream stream(line.substr(begin + 1, end - begin - 1));
    Pose pose{};
    for (std::size_t i = 0; i < pose.size(); ++i)
    {
      if (!(stream >> pose[i]))
      {
        reason = "candidate q row does not contain seven numbers";
        return false;
      }
      if (i + 1 < pose.size())
      {
        char comma = 0;
        if (!(stream >> comma) || comma != ',')
        {
          reason = "candidate q row has invalid separators";
          return false;
        }
      }
    }
    poses.push_back(pose);
  }
  if (poses.size() != expected_count)
  {
    reason = "expected exactly " + std::to_string(expected_count) +
      " poses, got " + std::to_string(poses.size());
    return false;
  }
  return true;
}

bool load_fit(const std::string& path, Eigen::Vector4d& parameters,
              std::string& reason)
{
  std::ifstream input(path);
  if (!input) { reason = "cannot open fitted payload YAML: " + path; return false; }
  double mass = std::numeric_limits<double>::quiet_NaN();
  Eigen::Vector3d first_moment = Eigen::Vector3d::Constant(
    std::numeric_limits<double>::quiet_NaN());
  std::string payload_frame, base_link;
  std::string line;
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
      const auto begin = line.find('['), end = line.find(']', begin);
      if (begin == std::string::npos || end == std::string::npos) continue;
      std::stringstream stream(line.substr(begin + 1, end - begin - 1));
      for (std::size_t i = 0; i < 3; ++i)
      {
        if (!(stream >> first_moment(static_cast<Eigen::Index>(i)))) break;
        if (i < 2) { char comma{}; stream >> comma; }
      }
    }
  }
  if (base_link != "torso_link" || payload_frame != kActiveWristLink)
  {
    reason = "payload fit frame does not match selected arm " + std::string(kActiveArm);
    return false;
  }
  if (!std::isfinite(mass) || mass <= 0.01 || !first_moment.allFinite())
  {
    reason = "fitted payload YAML has invalid mass or first moment";
    return false;
  }
  parameters << mass, first_moment.x(), first_moment.y(), first_moment.z();
  return true;
}

std::vector<std::string> split_csv_row(const std::string& line)
{
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) fields.push_back(field);
  return fields;
}

bool load_training_csv(const std::string& path, std::size_t expected_count,
                       std::vector<Observation>& observations, std::string& reason,
                       const std::string& accepted_phase = "train")
{
  std::ifstream input(path);
  if (!input) { reason = "cannot open resume CSV: " + path; return false; }
  std::string line;
  std::getline(input, line);
  while (std::getline(input, line))
  {
    const auto fields = split_csv_row(line);
    if (fields.size() != 20 || fields[1] != accepted_phase) continue;
    try
    {
      const auto id = static_cast<std::size_t>(std::stoul(fields[0]));
      if (id != observations.size())
      {
        reason = "resume CSV training pose IDs are not contiguous from zero";
        return false;
      }
      Observation observation;
      observation.stamp_ns = std::stoll(fields[2]);
      for (std::size_t j = 0; j < 7; ++j) observation.q[j] = std::stod(fields[3 + j]);
      for (std::size_t j = 0; j < 7; ++j) observation.tau[j] = std::stod(fields[10 + j]);
      for (std::size_t j = 0; j < 3; ++j) observation.rpy[j] = std::stod(fields[17 + j]);
      observations.push_back(observation);
    }
    catch (const std::exception&)
    {
      reason = "malformed numeric field in resume CSV";
      return false;
    }
  }
  if (expected_count > 0 && observations.size() != expected_count)
  {
    reason = "resume CSV contains " + std::to_string(observations.size()) +
      " training poses; expected " + std::to_string(expected_count);
    return false;
  }
  return true;
}

bool write_payload_fit(const std::string& path, const Eigen::Vector4d& parameters,
                       std::size_t pose_count, double rmse, double condition)
{
  std::ofstream out(path, std::ios::trunc);
  if (!out) return false;
  const double mass = parameters(0);
  const Eigen::Vector3d com = parameters.tail<3>() / mass;
  out << std::setprecision(12)
      << "base_link: torso_link\npayload_frame: " << kActiveWristLink << '\n'
      << "pose_count: " << pose_count << '\n'
      << "mass_kg: " << mass << '\n'
      << "com_xyz_m: [" << com.x() << ", " << com.y() << ", " << com.z() << "]\n"
      << "first_moment_kg_m: [" << parameters(1) << ", " << parameters(2)
      << ", " << parameters(3) << "]\n"
      << "training_rmse_nm: " << rmse << '\n'
      << "regressor_condition_number: " << condition << '\n';
  return true;
}

bool load_named_pose(const std::string& path, const std::string& key,
                     Pose& pose, std::string& reason)
{
  std::ifstream input(path);
  if (!input) { reason = "cannot open resume path: " + path; return false; }
  std::string line;
  while (std::getline(input, line))
  {
    if (line.rfind(key + ": [", 0) != 0) continue;
    const auto begin = line.find('['), end = line.find(']', begin);
    std::stringstream stream(line.substr(begin + 1, end - begin - 1));
    for (std::size_t j = 0; j < 7; ++j)
    {
      if (!(stream >> pose[j])) { reason = "malformed " + key; return false; }
      if (j < 6) { char comma{}; if (!(stream >> comma) || comma != ',') {
        reason = "malformed " + key; return false; } }
    }
    return true;
  }
  reason = "resume path does not contain " + key;
  return false;
}

class PayloadModel
{
public:
  bool initialize(const std::string& urdf_path, std::string& reason)
  {
    KDL::Tree tree;
    if (!kdl_parser::treeFromFile(urdf_path, tree) ||
        !tree.getChain("torso_link", kActiveWristLink, chain_))
    {
      reason = "failed to load KDL selected-arm chain from " + urdf_path;
      return false;
    }
    if (chain_.getNrOfJoints() != 7)
    {
      reason = "KDL selected-arm chain is not 7-DoF";
      return false;
    }
    return true;
  }

  bool regression(const Pose& q_values, const std::array<double, 3>& rpy,
                  Eigen::VectorXd& baseline,
                  Eigen::Matrix<double, 7, 4>& regressor) const
  {
    if (!gravity(chain_, q_values, rpy, baseline)) return false;
    Eigen::VectorXd at_origin, at_x, at_y, at_z;
    if (!gravity(with_payload(KDL::Vector::Zero()), q_values, rpy, at_origin) ||
        !gravity(with_payload(KDL::Vector(1, 0, 0)), q_values, rpy, at_x) ||
        !gravity(with_payload(KDL::Vector(0, 1, 0)), q_values, rpy, at_y) ||
        !gravity(with_payload(KDL::Vector(0, 0, 1)), q_values, rpy, at_z))
      return false;
    regressor.col(0) = at_origin - baseline;
    regressor.col(1) = at_x - at_origin;
    regressor.col(2) = at_y - at_origin;
    regressor.col(3) = at_z - at_origin;
    return true;
  }

  bool fit(const std::vector<Observation>& observations, Eigen::Vector4d& parameters,
           double& rmse, double& condition, std::string& reason) const
  {
    if (observations.size() < 8)
    {
      reason = "at least eight static poses are required";
      return false;
    }
    Eigen::MatrixXd design(observations.size() * 7, 4);
    Eigen::VectorXd residual(observations.size() * 7);
    for (std::size_t p = 0; p < observations.size(); ++p)
    {
      Eigen::VectorXd baseline;
      Eigen::Matrix<double, 7, 4> regressor;
      if (!regression(observations[p].q, observations[p].rpy, baseline, regressor))
      {
        reason = "KDL regression failed at pose " + std::to_string(p);
        return false;
      }
      design.block<7, 4>(static_cast<Eigen::Index>(p * 7), 0) = regressor;
      for (std::size_t j = 0; j < 7; ++j)
        residual(static_cast<Eigen::Index>(p * 7 + j)) = observations[p].tau[j] - baseline(j);
    }
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(design, Eigen::ComputeThinU | Eigen::ComputeThinV);
    parameters = svd.solve(residual);
    const auto singular = svd.singularValues();
    condition = singular(singular.size() - 1) <= 1e-9
      ? std::numeric_limits<double>::infinity()
      : singular(0) / singular(singular.size() - 1);
    const Eigen::VectorXd error = residual - design * parameters;
    rmse = std::sqrt(error.squaredNorm() / static_cast<double>(error.size()));
    const double mass = parameters(0);
    if (!std::isfinite(mass) || mass <= 0.01)
    {
      reason = "unphysical fitted payload mass=" + std::to_string(mass);
      return false;
    }
    const Eigen::Vector3d com = parameters.tail<3>() / mass;
    if (!com.allFinite() || com.norm() > 0.50)
    {
      reason = "unphysical fitted payload COM norm=" + std::to_string(com.norm());
      return false;
    }
    return true;
  }

  bool torque(const Pose& q, const std::array<double, 3>& rpy,
              const Eigen::Vector4d& parameters, Pose& result) const
  {
    Eigen::VectorXd baseline;
    Eigen::Matrix<double, 7, 4> regressor;
    if (!regression(q, rpy, baseline, regressor)) return false;
    const Eigen::VectorXd torque = baseline + regressor * parameters;
    for (std::size_t i = 0; i < 7; ++i) result[i] = torque(i);
    return true;
  }

private:
  KDL::Chain with_payload(const KDL::Vector& com) const
  {
    KDL::Chain result = chain_;
    result.addSegment(KDL::Segment("identified_payload", KDL::Joint(KDL::Joint::Fixed),
      KDL::Frame::Identity(),
      KDL::RigidBodyInertia(1.0, com, KDL::RotationalInertia::Zero())));
    return result;
  }

  bool gravity(const KDL::Chain& chain, const Pose& q_values,
               const std::array<double, 3>& rpy, Eigen::VectorXd& output) const
  {
    KDL::JntArray q(chain.getNrOfJoints()), torque(chain.getNrOfJoints());
    for (std::size_t i = 0; i < q_values.size(); ++i) q(i) = q_values[i];
    const auto world_from_torso = KDL::Rotation::RPY(rpy[0], rpy[1], rpy[2]);
    const KDL::Vector gravity = world_from_torso.Inverse(KDL::Vector(0, 0, -9.81));
    KDL::ChainDynParam dynamics(chain, gravity);
    if (dynamics.JntToGravity(q, torque) != 0) return false;
    output.resize(7);
    for (std::size_t i = 0; i < 7; ++i) output(i) = torque(i);
    return true;
  }

  KDL::Chain chain_;
};

class CalibrationRun
{
public:
  CalibrationRun(SharedState& shared, std::vector<Pose> candidates,
                 PayloadModel model, std::string csv_path, std::string fit_path,
                 std::vector<Observation> existing = {},
                 const Pose& resume_start = {}, std::size_t resume_index = 0)
  : shared_(shared), candidates_(std::move(candidates)), model_(std::move(model)),
    csv_path_(std::move(csv_path)), fit_path_(std::move(fit_path)),
    resume_start_(resume_start), resume_index_(resume_index),
    training_(std::move(existing)), resume_mode_(resume_index > 0) {}

  int execute()
  {
    if (!preflight()) return 2;
    Clock::time_point received_at;
    std::uint64_t samples = 0;
    auto state = snapshot(shared_, received_at, samples);
    for (const auto index : kControlledJoints)
      initial_desired_[index] = desired_[index] = state.motor_state().at(index).q();
    for (std::size_t j = 0; j < 7; ++j)
      initial_arm_[j] = state.motor_state().at(kActiveIndices[j]).q();
    if (resume_mode_)
    {
      double maximum = 0.0;
      for (std::size_t j = 0; j < 7; ++j)
        maximum = std::max(maximum, std::abs(initial_arm_[j] - resume_start_[j]));
      std::cout << "RESUME_START_DELTA max_joint_delta=" << maximum << " rad\n";
      if (maximum > kStartPerJointTolerance)
      {
        std::cerr << "Current pose drifted more than 5 deg/joint from the MoveIt-checked "
                  << "resume start; recheck path. No publisher created.\n";
        return 2;
      }
      on_screened_sequence_ = false;
    }
    double nearest_delta = std::numeric_limits<double>::infinity();
    std::size_t nearest_pose = candidates_.size();
    for (std::size_t pose = 0; !resume_mode_ && pose < candidates_.size(); ++pose)
    {
      double maximum = 0.0;
      for (std::size_t j = 0; j < 7; ++j)
        maximum = std::max(maximum, std::abs(initial_arm_[j] - candidates_[pose][j]));
      if (maximum < nearest_delta) { nearest_delta = maximum; nearest_pose = pose; }
    }
    if (!resume_mode_)
    {
      std::cout << "START_NEAREST_POSE id=" << nearest_pose
                << " max_joint_delta=" << nearest_delta << " rad\n";
      if (nearest_pose == candidates_.size() || nearest_delta > kStartPerJointTolerance)
      {
        std::cerr << "Current selected arm is not within 5 deg/joint of any screened pose; "
                  << "regenerate candidates. No publisher created.\n";
        return 2;
      }
      current_pose_index_ = nearest_pose;
    }

    std::string tau_reason;
    if (!collect_tau_feedforward(shared_, initial_tau_, tau_reason, true,
                                 &kCalibrationTakeoverLimits))
    {
      std::cerr << "Initial tau_est capture failed: " << tau_reason << '\n';
      return 2;
    }
    initial_rpy_ = current_rpy(state);
    Eigen::Matrix<double, 7, 4> unused;
    if (!model_.regression(initial_arm_, initial_rpy_, base_anchor_, unused))
    {
      std::cerr << "Initial KDL gravity calculation failed.\n";
      return 2;
    }

    std::cout << "\nREADY: " << (resume_mode_ ? "resume poses 8-11" : "12 training poses")
              << ", then reverse return, least-squares fit, "
              << "optional fitted-feedforward validation.\n"
              << "Each pose requires ENTER while the 50 Hz hold continues.\n"
              << "L2+B/state/FSM/motor/velocity/torque/temperature/tracking faults "
              << "stop publishing immediately.\n"
              << "Type START to create rt/arm_sdk publisher: " << std::flush;
    std::string confirmation;
    while (!g_stop_requested.load())
    {
      pollfd descriptor{STDIN_FILENO, POLLIN, 0};
      if (::poll(&descriptor, 1, 100) > 0 && (descriptor.revents & POLLIN))
      {
        std::getline(std::cin, confirmation);
        break;
      }
    }
    if (g_stop_requested.load())
    {
      std::cout << "Cancelled by signal; publisher was never created.\n";
      return 0;
    }
    if (confirmation != "START")
    {
      std::cout << "Cancelled; publisher was never created.\n";
      return 0;
    }

    std::string reason;
    if (!final_preflight(reason))
    {
      std::cerr << "Final pre-publisher check failed: " << reason << '\n';
      return 2;
    }
    publisher_ = std::make_unique<unitree::robot::ChannelPublisher<LowCmd>>(kArmSdkTopic);
    publisher_->InitChannel();
    fitted_ = false;
    if (!hold_for(2.0, true, reason)) return finish_fault(reason);
    if (resume_mode_)
    {
      if (!operator_gate("RESUME: press ENTER to follow the MoveIt-checked direct path to pose " +
          std::to_string(resume_index_), reason)) return finish_after_failure(reason);
      if (!move_to(candidates_[resume_index_], reason)) return finish_after_failure(reason);
      current_pose_index_ = resume_index_;
      on_screened_sequence_ = true;
    }
    else if (current_pose_index_ > 0)
    {
      if (!operator_gate("RECOVERY: currently nearest pose " +
          std::to_string(current_pose_index_) +
          "; press ENTER to return along screened path to pose 0", reason))
        return finish_after_failure(reason);
      if (!return_to_start(reason)) return finish_after_failure(reason);
    }

    std::filesystem::create_directories(std::filesystem::path(csv_path_).parent_path());
    std::ofstream csv(csv_path_, resume_mode_ ? std::ios::app : std::ios::trunc);
    if (!resume_mode_) write_csv_header(csv);

    for (std::size_t pose = resume_index_; pose < candidates_.size(); ++pose)
    {
      if (pose > resume_index_)
      {
        if (!operator_gate("NEXT pose " + std::to_string(pose) +
            "/11: press ENTER to move and auto-capture, or type ABORT", reason))
          return finish_after_failure(reason);
        if (!move_to(candidates_[pose], reason)) return finish_after_failure(reason);
        current_pose_index_ = pose;
      }
      Observation observation;
      if (!capture_stable(observation, reason)) return finish_after_failure(reason);
      training_.push_back(observation);
      write_observation(csv, pose, "train", observation);
      write_temperature_observation(pose, "train", observation);
      csv.flush();
      report_capture_temperature(pose, observation);
    }

    std::cout << "Returning to this run's measured start along checked paths.\n";
    if (!return_to_start(reason)) return finish_after_failure(reason);

    double training_rmse = 0.0;
    double condition = 0.0;
    if (!model_.fit(training_, parameters_, training_rmse, condition, reason))
      return finish_normal("fit rejected: " + reason);
    if (!write_fit(training_rmse, condition))
      return finish_normal("fit succeeded but YAML could not be written");
    const double mass = parameters_(0);
    const Eigen::Vector3d com = parameters_.tail<3>() / mass;
    std::cout << std::fixed << std::setprecision(5)
              << "FIT mass=" << mass << " kg com_wrist=[" << com.x() << ','
              << com.y() << ',' << com.z() << "] m training_rmse="
              << training_rmse << " Nm condition=" << condition << '\n';

    if (resume_mode_)
      return finish_normal("resume capture and 12-pose fit complete; validation deferred");

    if (!operator_gate("Type VALIDATE then ENTER to replay and validate poses 1, 6, 11; "
                       "anything else returns/releases", reason, "VALIDATE"))
      return finish_after_failure(reason);
    if (!activate_fitted_feedforward(reason)) return finish_after_failure(reason);

    const std::array<std::size_t, 3> validation_ids{1, 6, 11};
    std::vector<Observation> validation;
    for (std::size_t pose = 1; pose < candidates_.size(); ++pose)
    {
      if (!move_to(candidates_[pose], reason)) return finish_after_failure(reason);
      current_pose_index_ = pose;
      if (std::find(validation_ids.begin(), validation_ids.end(), pose) != validation_ids.end())
      {
        if (!operator_gate("VALIDATE pose " + std::to_string(pose) +
            ": press ENTER to capture", reason)) return finish_after_failure(reason);
        Observation observation;
        if (!capture_stable(observation, reason)) return finish_after_failure(reason);
        validation.push_back(observation);
        write_observation(csv, pose, "validation", observation);
        write_temperature_observation(pose, "validation", observation);
        csv.flush();
      }
    }
    for (std::size_t pose = candidates_.size() - 1; pose > 0; --pose)
    {
      if (!move_to(candidates_[pose - 1], reason)) return finish_after_failure(reason);
      current_pose_index_ = pose - 1;
    }
    report_validation(validation);
    return finish_normal("training, fit and validation complete");
  }

  int execute_validation(const Eigen::Vector4d& fitted_parameters)
  {
    if (candidates_.size() != 2)
    {
      std::cerr << "Validation requires current-start plus one held-out pose.\n";
      return 2;
    }
    if (!preflight()) return 2;
    Clock::time_point received_at;
    std::uint64_t samples = 0;
    const auto state = snapshot(shared_, received_at, samples);
    for (const auto index : kControlledJoints)
      initial_desired_[index] = desired_[index] = state.motor_state().at(index).q();
    for (std::size_t j = 0; j < 7; ++j)
      initial_arm_[j] = state.motor_state().at(kActiveIndices[j]).q();

    double start_delta = 0.0;
    for (std::size_t j = 0; j < 7; ++j)
      start_delta = std::max(start_delta,
        std::abs(initial_arm_[j] - candidates_.front()[j]));
    std::cout << "VALIDATION_START_DELTA max_joint_delta=" << start_delta << " rad\n";
    if (start_delta > kStartPerJointTolerance)
    {
      std::cerr << "Current pose drifted more than 5 deg/joint from the MoveIt-screened "
                   "validation start; regenerate validation poses. No publisher created.\n";
      return 2;
    }

    std::string reason;
    if (!collect_tau_feedforward(shared_, initial_tau_, reason, true,
                                 &kCalibrationTakeoverLimits))
    {
      std::cerr << "Initial tau_est capture failed: " << reason << '\n';
      return 2;
    }
    initial_rpy_ = current_rpy(state);
    Eigen::Matrix<double, 7, 4> unused;
    if (!model_.regression(initial_arm_, initial_rpy_, base_anchor_, unused))
    {
      std::cerr << "Initial KDL gravity calculation failed.\n";
      return 2;
    }
    parameters_ = fitted_parameters;
    current_pose_index_ = 0;
    on_screened_sequence_ = true;

    std::cout << "\nREADY: two-pose A/B payload-feedforward validation.\n"
              << "Pose 0 is the measured current start; pose 1 is held out from fitting.\n"
              << "Every feedforward enable/disable uses a 2 s quintic blend.\n"
              << "L2+B/state/FSM/motor/velocity/torque/temperature/tracking faults "
              << "stop publishing immediately.\n"
              << "Type START to create rt/arm_sdk publisher: " << std::flush;
    std::string confirmation;
    while (!g_stop_requested.load())
    {
      pollfd descriptor{STDIN_FILENO, POLLIN, 0};
      if (::poll(&descriptor, 1, 100) > 0 && (descriptor.revents & POLLIN))
      {
        std::getline(std::cin, confirmation);
        break;
      }
    }
    if (g_stop_requested.load() || confirmation != "START")
    {
      std::cout << "Cancelled; publisher was never created.\n";
      return 0;
    }
    if (!final_preflight(reason))
    {
      std::cerr << "Final pre-publisher check failed: " << reason << '\n';
      return 2;
    }
    publisher_ = std::make_unique<unitree::robot::ChannelPublisher<LowCmd>>(kArmSdkTopic);
    publisher_->InitChannel();
    fitted_ = false;
    fitted_blend_ = 0.0;
    if (!hold_for(2.0, true, reason)) return finish_fault(reason);

    std::filesystem::create_directories(std::filesystem::path(csv_path_).parent_path());
    std::ofstream csv(csv_path_, std::ios::trunc);
    if (!csv) return finish_normal("cannot create validation CSV");
    write_csv_header(csv);

    for (std::size_t pose = 0; pose < 2; ++pose)
    {
      Pose target = pose == 0 ? initial_arm_ : candidates_[pose];
      if (pose > 0)
      {
        if (!operator_gate("NEXT held-out pose 1: press ENTER to blend compensation out "
                           "and follow the MoveIt-screened path, or type ABORT", reason))
          return finish_after_failure(reason);
        if (!deactivate_fitted_feedforward(reason)) return finish_after_failure(reason);
        if (!move_to(target, reason)) return finish_after_failure(reason);
        current_pose_index_ = pose;
      }

      Observation baseline;
      if (!capture_stable(baseline, reason)) return finish_after_failure(reason);
      write_observation(csv, pose, "validation_baseline", baseline);
      write_temperature_observation(pose, "validation_baseline", baseline);
      csv.flush();
      report_validation_phase(pose, "BASELINE", target, baseline);

      if (!operator_gate("ENABLE fitted gravity compensation at validation pose " +
          std::to_string(pose) + ": press ENTER to blend in, or type ABORT", reason))
        return finish_after_failure(reason);
      if (!activate_fitted_feedforward(reason)) return finish_after_failure(reason);

      Observation compensated;
      if (!capture_stable(compensated, reason)) return finish_after_failure(reason);
      write_observation(csv, pose, "validation_fitted", compensated);
      write_temperature_observation(pose, "validation_fitted", compensated);
      csv.flush();
      report_validation_phase(pose, "FITTED", target, compensated);
      report_validation_pair(pose, target, baseline, compensated);
    }

    if (!operator_gate("RETURN: press ENTER to blend compensation out and return along "
                       "the checked path, or type ABORT", reason))
      return finish_after_failure(reason);
    if (!deactivate_fitted_feedforward(reason)) return finish_after_failure(reason);
    if (!return_to_start(reason)) return finish_after_failure(reason);
    return finish_normal("two-pose fitted-feedforward validation complete");
  }

  int execute_compliance_hold(const Eigen::Vector4d& fitted_parameters)
  {
    if (!preflight()) return 2;
    Clock::time_point received_at;
    std::uint64_t samples = 0;
    const auto state = snapshot(shared_, received_at, samples);
    for (const auto index : kControlledJoints)
      initial_desired_[index] = desired_[index] = state.motor_state().at(index).q();
    for (std::size_t j = 0; j < 7; ++j)
      initial_arm_[j] = state.motor_state().at(kActiveIndices[j]).q();

    std::string reason;
    if (!collect_tau_feedforward(shared_, initial_tau_, reason, true,
                                 &kCalibrationTakeoverLimits))
    {
      std::cerr << "Initial tau_est capture failed: " << reason << '\n';
      return 2;
    }
    initial_rpy_ = current_rpy(state);
    Eigen::Matrix<double, 7, 4> unused;
    if (!model_.regression(initial_arm_, initial_rpy_, base_anchor_, unused))
    {
      std::cerr << "Initial KDL gravity calculation failed.\n";
      return 2;
    }
    parameters_ = fitted_parameters;

    std::cout << "\nREADY: staged selected-arm compliant-hold test at the current pose.\n"
              << "Stages are fitted gravity at Kp/Kd 60/1.5, 30/2.0, then 15/2.5.\n"
              << "This is not drag mode: do not displace a joint beyond 3 degrees.\n"
              << "Each stage requires a separate operator confirmation.\n"
              << "L2+B/state/FSM/motor/velocity/torque/temperature/tracking faults "
              << "stop publishing immediately.\n"
              << "Type START to create rt/arm_sdk publisher: " << std::flush;
    std::string confirmation;
    while (!g_stop_requested.load())
    {
      pollfd descriptor{STDIN_FILENO, POLLIN, 0};
      if (::poll(&descriptor, 1, 100) > 0 && (descriptor.revents & POLLIN))
      {
        std::getline(std::cin, confirmation);
        break;
      }
    }
    if (g_stop_requested.load() || confirmation != "START")
    {
      std::cout << "Cancelled; publisher was never created.\n";
      return 0;
    }
    if (!final_preflight(reason))
    {
      std::cerr << "Final pre-publisher check failed: " << reason << '\n';
      return 2;
    }
    publisher_ = std::make_unique<unitree::robot::ChannelPublisher<LowCmd>>(kArmSdkTopic);
    publisher_->InitChannel();
    fitted_ = false;
    fitted_blend_ = 0.0;
    arm_kp_ = kArmKp;
    arm_kd_ = kArmKd;
    if (!hold_for(2.0, true, reason)) return finish_fault(reason);

    std::filesystem::create_directories(std::filesystem::path(csv_path_).parent_path());
    std::ofstream csv(csv_path_, std::ios::trunc);
    if (!csv) return finish_normal("cannot create compliant-hold CSV");
    write_csv_header(csv);

    if (!operator_gate("ENABLE fitted dynamic gravity compensation at Kp/Kd 60/1.5: "
                       "press ENTER, or type ABORT", reason))
      return finish_after_failure(reason);
    if (!activate_fitted_feedforward(reason)) return finish_after_failure(reason);
    Observation observation;
    if (!capture_stable(observation, reason)) return finish_after_failure(reason);
    write_observation(csv, 0, "compliance_kp60", observation);
    write_temperature_observation(0, "compliance_kp60", observation);
    report_validation_phase(0, "COMPLIANCE_KP60", initial_arm_, observation, arm_kp_);

    if (!operator_gate("LOWER selected-arm gains to Kp/Kd 30/2.0 over 3 s: press ENTER, "
                       "or type ABORT", reason))
      return finish_after_failure(reason);
    if (!ramp_arm_gains(30.0, 2.0, 3.0, reason)) return finish_after_failure(reason);
    if (!capture_stable(observation, reason)) return finish_after_failure(reason);
    write_observation(csv, 1, "compliance_kp30", observation);
    write_temperature_observation(1, "compliance_kp30", observation);
    report_validation_phase(1, "COMPLIANCE_KP30", initial_arm_, observation, arm_kp_);

    if (!operator_gate("LOWER selected-arm gains to Kp/Kd 15/2.5 over 3 s: press ENTER, "
                       "or type ABORT", reason))
      return finish_after_failure(reason);
    if (!ramp_arm_gains(15.0, 2.5, 3.0, reason)) return finish_after_failure(reason);
    if (!capture_stable(observation, reason)) return finish_after_failure(reason);
    write_observation(csv, 2, "compliance_kp15", observation);
    write_temperature_observation(2, "compliance_kp15", observation);
    csv.flush();
    report_validation_phase(2, "COMPLIANCE_KP15", initial_arm_, observation, arm_kp_);

    if (!operator_gate("RESTORE: press ENTER to restore Kp/Kd 60/1.5, blend fitted "
                       "compensation out, and release, or type ABORT", reason))
      return finish_after_failure(reason);
    if (!ramp_arm_gains(kArmKp, kArmKd, 3.0, reason))
      return finish_after_failure(reason);
    if (!deactivate_fitted_feedforward(reason)) return finish_after_failure(reason);
    return finish_normal("staged compliant-hold test complete");
  }

  int execute_elbow_drag(const Eigen::Vector4d& fitted_parameters)
  {
    if (candidates_.size() != 3)
    {
      std::cerr << "Elbow drag requires start/lower/upper MoveIt-checked poses.\n";
      return 2;
    }
    if (!preflight()) return 2;
    Clock::time_point received_at;
    std::uint64_t samples = 0;
    const auto state = snapshot(shared_, received_at, samples);
    for (const auto index : kControlledJoints)
      initial_desired_[index] = desired_[index] = state.motor_state().at(index).q();
    for (std::size_t j = 0; j < 7; ++j)
      initial_arm_[j] = state.motor_state().at(kActiveIndices[j]).q();
    double start_delta = 0.0;
    for (std::size_t j = 0; j < 7; ++j)
      start_delta = std::max(start_delta,
        std::abs(initial_arm_[j] - candidates_[0][j]));
    std::cout << "ELBOW_DRAG_START_DELTA max_joint_delta=" << start_delta << " rad\n";
    if (start_delta > kStartPerJointTolerance)
    {
      std::cerr << "Current pose drifted more than 5 deg/joint from the checked drag "
                   "start; regenerate the range. No publisher created.\n";
      return 2;
    }

    std::string reason;
    if (!collect_tau_feedforward(shared_, initial_tau_, reason, true,
                                 &kCalibrationTakeoverLimits))
    {
      std::cerr << "Initial tau_est capture failed: " << reason << '\n';
      return 2;
    }
    initial_rpy_ = current_rpy(state);
    Eigen::Matrix<double, 7, 4> unused;
    if (!model_.regression(initial_arm_, initial_rpy_, base_anchor_, unused))
    {
      std::cerr << "Initial KDL gravity calculation failed.\n";
      return 2;
    }
    parameters_ = fitted_parameters;

    std::cout << "\nREADY: selected-elbow-only limited drag test.\n"
              << "MoveIt-checked elbow range is +/-10 deg from the measured start.\n"
              << "Other arm joints remain at Kp/Kd 60/1.5; elbow uses 3/2.5.\n"
              << "Do not touch the arm until DRAG_ACTIVE is printed. Move slowly and "
              << "stop when boundary resistance is felt.\n"
              << "L2+B/state/FSM/motor/velocity/torque/temperature/tracking faults "
              << "stop publishing immediately.\n"
              << "Type START to create rt/arm_sdk publisher: " << std::flush;
    std::string confirmation;
    while (!g_stop_requested.load())
    {
      pollfd descriptor{STDIN_FILENO, POLLIN, 0};
      if (::poll(&descriptor, 1, 100) > 0 && (descriptor.revents & POLLIN))
      {
        std::getline(std::cin, confirmation);
        break;
      }
    }
    if (g_stop_requested.load() || confirmation != "START")
    {
      std::cout << "Cancelled; publisher was never created.\n";
      return 0;
    }
    if (!final_preflight(reason))
    {
      std::cerr << "Final pre-publisher check failed: " << reason << '\n';
      return 2;
    }
    publisher_ = std::make_unique<unitree::robot::ChannelPublisher<LowCmd>>(kArmSdkTopic);
    publisher_->InitChannel();
    fitted_ = false;
    fitted_blend_ = 0.0;
    arm_kp_ = kArmKp;
    arm_kd_ = kArmKd;
    elbow_gain_override_ = false;
    if (!hold_for(2.0, true, reason)) return finish_fault(reason);

    if (!operator_gate("ENABLE dynamic gravity compensation and prepare compliant gains: "
                       "press ENTER, or type ABORT", reason))
      return finish_after_failure(reason);
    if (!activate_fitted_feedforward(reason)) return finish_after_failure(reason);

    if (!operator_gate("ARM elbow drag: press ENTER; only touch after DRAG_ACTIVE, "
                       "or type ABORT", reason))
      return finish_after_failure(reason);
    Clock::time_point received;
    std::uint64_t count = 0;
    auto current = snapshot(shared_, received, count);
    desired_[kActiveIndices[3]] = current.motor_state().at(kActiveIndices[3]).q();
    if (!ramp_elbow_gains(3.0, 2.5, 3.0, reason)) return finish_after_failure(reason);

    const double lower = candidates_[1][3];
    const double upper = candidates_[2][3];
    std::filesystem::create_directories(std::filesystem::path(csv_path_).parent_path());
    std::ofstream csv(csv_path_, std::ios::trunc);
    if (!csv) return finish_normal("cannot create elbow-drag CSV");
    csv << "stamp_ns,elbow_q,elbow_target,elbow_dq,elbow_tau,casing,winding\n";
    std::cout << "DRAG_ACTIVE elbow_range_deg=[-10,+10]; move the right elbow slowly. "
                 "Type STOP to finish.\n" << std::flush;
    auto next = Clock::now();
    auto next_report = next;
    while (!g_stop_requested.load())
    {
      pollfd descriptor{STDIN_FILENO, POLLIN, 0};
      if (::poll(&descriptor, 1, 0) > 0 && (descriptor.revents & POLLIN))
      {
        std::string line;
        std::getline(std::cin, line);
        if (line == "STOP") break;
        if (line == "ABORT") return finish_after_failure("operator requested ABORT");
      }
      current = snapshot(shared_, received, count);
      const auto& elbow = current.motor_state().at(kActiveIndices[3]);
      const double measured = elbow.q();
      const double follow_error = measured - desired_[kActiveIndices[3]];
      const double step = std::clamp(0.30 * follow_error, -0.006, 0.006);
      desired_[kActiveIndices[3]] = static_cast<float>(std::clamp(
        static_cast<double>(desired_[kActiveIndices[3]]) + step, lower, upper));
      if (!tick(reason)) return finish_fault(reason);
      if (Clock::now() >= next_report)
      {
        const auto stamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count();
        csv << stamp << ',' << measured << ',' << desired_[kActiveIndices[3]] << ','
            << elbow.dq() << ',' << elbow.tau_est() << ',' << elbow.temperature()[0]
            << ',' << elbow.temperature()[1] << '\n';
        csv.flush();
        std::cout << std::fixed << std::setprecision(2)
                  << "DRAG_STATUS elbow_offset_deg="
                  << (measured - candidates_[0][3]) * 180.0 / M_PI
                  << " target_offset_deg="
                  << (desired_[kActiveIndices[3]] - candidates_[0][3]) * 180.0 / M_PI
                  << " dq=" << elbow.dq() << " tau=" << elbow.tau_est()
                  << " temp=" << static_cast<int>(elbow.temperature()[0]) << '/'
                  << static_cast<int>(elbow.temperature()[1]) << "C\n" << std::flush;
        next_report += 1s;
      }
      next += 20ms;
      std::this_thread::sleep_until(next);
    }
    if (g_stop_requested.load()) return finish_fault("signal requested");

    current = snapshot(shared_, received, count);
    desired_[kActiveIndices[3]] = static_cast<float>(std::clamp(
      static_cast<double>(current.motor_state().at(kActiveIndices[3]).q()), lower, upper));
    if (!ramp_elbow_gains(kArmKp, kArmKd, 3.0, reason))
      return finish_after_failure(reason);
    elbow_gain_override_ = false;
    if (!deactivate_fitted_feedforward(reason)) return finish_after_failure(reason);
    return finish_normal("selected-elbow limited drag test complete");
  }

  int execute_elbow_admittance(const Eigen::Vector4d& fitted_parameters)
  {
    if (!preflight()) return 2;
    Clock::time_point received_at;
    std::uint64_t samples = 0;
    const auto state = snapshot(shared_, received_at, samples);
    for (const auto index : kControlledJoints)
      initial_desired_[index] = desired_[index] = state.motor_state().at(index).q();
    desired_velocity_.fill(0.0F);
    for (std::size_t j = 0; j < 7; ++j)
      initial_arm_[j] = state.motor_state().at(kActiveIndices[j]).q();

    const std::size_t elbow_index = kActiveIndices[3];
    const double lower = kActiveJointLower[3] + kJointLimitMargin;
    const double upper = kActiveJointUpper[3] - kJointLimitMargin;
    if (initial_arm_[3] <= lower || initial_arm_[3] >= upper)
    {
      std::cerr << "Right elbow starts inside the 5 deg mechanical-limit margin; "
                   "no publisher created.\n";
      return 2;
    }

    std::string reason;
    if (!collect_tau_feedforward(shared_, initial_tau_, reason, true,
                                 &kCalibrationTakeoverLimits))
    {
      std::cerr << "Initial tau_est capture failed: " << reason << '\n';
      return 2;
    }
    initial_rpy_ = current_rpy(state);
    Eigen::Matrix<double, 7, 4> unused;
    if (!model_.regression(initial_arm_, initial_rpy_, base_anchor_, unused))
    {
      std::cerr << "Initial KDL gravity calculation failed.\n";
      return 2;
    }
    parameters_ = fitted_parameters;
    int ready_casing = std::numeric_limits<int>::min();
    int ready_winding = std::numeric_limits<int>::min();
    std::size_t ready_casing_joint = kActiveIndices.front();
    std::size_t ready_winding_joint = kActiveIndices.front();
    for (const auto index : kActiveIndices)
    {
      const auto& motor = state.motor_state().at(index);
      const int casing = static_cast<int>(motor.temperature()[0]);
      const int winding = static_cast<int>(motor.temperature()[1]);
      if (casing > ready_casing) { ready_casing = casing; ready_casing_joint = index; }
      if (winding > ready_winding) { ready_winding = winding; ready_winding_joint = index; }
    }

    std::cout << "\nREADY: selected-elbow sensorless admittance test.\n"
              << "The controller estimates human torque from fitted gravity minus "
                 "tau_est, then generates bounded q/dq targets.\n"
              << "Elbow Kp/Kd=" << kAdmittanceKp << '/' << kAdmittanceKd
              << ", torque deadband=" << kAdmittanceTorqueDeadband
              << " Nm, velocity/acceleration limits=" << kAdmittanceMaxVelocity
              << " rad/s / " << kAdmittanceMaxAcceleration << " rad/s^2.\n"
              << "Other upper-body joints remain at normal hold gains. Keep hands off "
                 "during zero-torque calibration.\n"
              << "Current selected-arm temperature max=" << ready_casing << "C@j"
              << ready_casing_joint << '/' << ready_winding << "C@j"
              << ready_winding_joint << ".\n"
              << "L2+B/state/FSM/motor/velocity/torque/temperature/tracking faults "
                 "stop publishing immediately.\n"
              << "Type START to create rt/arm_sdk publisher: " << std::flush;
    std::string confirmation;
    while (!g_stop_requested.load())
    {
      pollfd descriptor{STDIN_FILENO, POLLIN, 0};
      if (::poll(&descriptor, 1, 100) > 0 && (descriptor.revents & POLLIN))
      {
        std::getline(std::cin, confirmation);
        break;
      }
    }
    if (g_stop_requested.load() || confirmation != "START")
    {
      std::cout << "Cancelled; publisher was never created.\n";
      return 0;
    }
    if (!final_preflight(reason))
    {
      std::cerr << "Final pre-publisher check failed: " << reason << '\n';
      return 2;
    }

    publisher_ = std::make_unique<unitree::robot::ChannelPublisher<LowCmd>>(kArmSdkTopic);
    publisher_->InitChannel();
    fitted_ = false;
    fitted_blend_ = 0.0;
    arm_kp_ = kArmKp;
    arm_kd_ = kArmKd;
    elbow_gain_override_ = false;
    use_total_command_limit_ = true;
    if (!hold_for(2.0, true, reason)) return finish_fault(reason);

    if (!operator_gate("ENABLE fitted gravity compensation and prepare elbow "
                       "admittance: press ENTER, or type ABORT", reason))
      return finish_after_failure(reason);
    if (!activate_fitted_feedforward(reason)) return finish_after_failure(reason);

    Clock::time_point received;
    std::uint64_t count = 0;
    auto current = snapshot(shared_, received, count);
    desired_[elbow_index] = current.motor_state().at(elbow_index).q();
    if (!ramp_elbow_gains(kAdmittanceKp, kAdmittanceKd, 3.0, reason))
      return finish_after_failure(reason);

    std::cout << "ADMITTANCE_SETTLING: waiting 2 s after the low-gain transition.\n"
              << std::flush;
    if (!hold_for(2.0, false, reason)) return finish_fault(reason);

    std::cout << "ADMITTANCE_ZEROING: keep hands off for a 2 s residual-torque "
                 "calibration.\n" << std::flush;
    std::vector<double> zero_samples;
    const auto zero_deadline = Clock::now() + 12s;
    auto zero_next = Clock::now();
    double zeroing_peak_dq = 0.0;
    while (Clock::now() < zero_deadline && zero_samples.size() < 100)
    {
      if (!tick(reason)) return finish_fault(reason);
      current = snapshot(shared_, received, count);
      const auto& elbow = current.motor_state().at(elbow_index);
      zeroing_peak_dq = std::max(zeroing_peak_dq,
        std::abs(static_cast<double>(elbow.dq())));
      std::array<float, 35> ff{};
      if (!make_feedforward(current, ff, reason)) return finish_fault(reason);
      if (std::abs(static_cast<double>(elbow.dq())) <= 0.05)
        zero_samples.push_back(static_cast<double>(ff[elbow_index]) - elbow.tau_est());
      else
        zero_samples.clear();
      zero_next += 20ms;
      std::this_thread::sleep_until(zero_next);
    }
    if (zero_samples.size() < 100)
      return finish_after_failure(
        "elbow was not static during admittance zeroing (peak_dq=" +
        std::to_string(zeroing_peak_dq) + ", consecutive_samples=" +
        std::to_string(zero_samples.size()) + ")");
    const double zero_bias = median(zero_samples);
    const double zero_spread = percentile(zero_samples, 0.9) -
      percentile(zero_samples, 0.1);
    if (!std::isfinite(zero_bias) || zero_spread > kStableTauSpread)
      return finish_after_failure("admittance residual-torque zeroing was unstable");
    std::cout << std::fixed << std::setprecision(4)
              << "ADMITTANCE_ZEROED bias=" << zero_bias
              << " Nm p90_p10=" << zero_spread << " Nm\n" << std::flush;

    if (!operator_gate("ARM elbow admittance: press ENTER; apply only a gentle elbow "
                       "push after ADMITTANCE_ACTIVE, or type ABORT", reason))
      return finish_after_failure(reason);

    std::filesystem::create_directories(std::filesystem::path(csv_path_).parent_path());
    std::ofstream csv(csv_path_, std::ios::trunc);
    if (!csv) return finish_after_failure("cannot create elbow-admittance CSV");
    csv << "stamp_ns,q,target_q,dq,target_dq,tau_est,tau_ff,raw_human_tau,"
           "filtered_human_tau,active_human_tau,total_cmd,casing,winding\n";
    std::cout << "ADMITTANCE_ACTIVE: gently push the right elbow in either direction. "
                 "Type STOP to restore normal gains and exit.\n" << std::flush;

    double filtered_human_tau = 0.0;
    double target_velocity = 0.0;
    auto next = Clock::now();
    auto next_report = next;
    auto next_thermal_warning = next;
    while (!g_stop_requested.load())
    {
      pollfd descriptor{STDIN_FILENO, POLLIN, 0};
      if (::poll(&descriptor, 1, 0) > 0 && (descriptor.revents & POLLIN))
      {
        std::string line;
        std::getline(std::cin, line);
        if (line == "STOP" || line == "EXIT") break;
        if (line == "ABORT")
        {
          desired_velocity_[elbow_index] = 0.0F;
          return finish_after_failure("operator requested ABORT");
        }
      }

      current = snapshot(shared_, received, count);
      const auto& elbow = current.motor_state().at(elbow_index);
      std::array<float, 35> ff{};
      if (!make_feedforward(current, ff, reason)) return finish_fault(reason);
      const double raw_human_tau = static_cast<double>(ff[elbow_index]) -
        elbow.tau_est() - zero_bias;
      const double filter_alpha = kTickSeconds /
        (kAdmittanceFilterSeconds + kTickSeconds);
      filtered_human_tau += filter_alpha *
        (raw_human_tau - filtered_human_tau);
      double active_human_tau = 0.0;
      if (std::abs(filtered_human_tau) > kAdmittanceTorqueDeadband)
        active_human_tau = std::copysign(
          std::abs(filtered_human_tau) - kAdmittanceTorqueDeadband,
          filtered_human_tau);

      const double velocity_request = std::clamp(
        kAdmittanceVelocityPerTorque * active_human_tau,
        -kAdmittanceMaxVelocity, kAdmittanceMaxVelocity);
      target_velocity += std::clamp(velocity_request - target_velocity,
        -kAdmittanceMaxAcceleration * kTickSeconds,
         kAdmittanceMaxAcceleration * kTickSeconds);

      const double measured = elbow.q();
      double target = measured + std::clamp(
        kAdmittanceTargetLeadSeconds * target_velocity,
        -kAdmittanceTargetLead, kAdmittanceTargetLead);
      target = std::clamp(target, lower, upper);

      const double soft_torque_limit =
        kActiveTauEstLimits[3] - kAdmittanceTorqueReserve;
      const double position_and_ff = static_cast<double>(ff[elbow_index]) +
        kAdmittanceKp * (target - measured);
      if (std::abs(position_and_ff) < soft_torque_limit)
      {
        const double velocity_lower = elbow.dq() +
          (-soft_torque_limit - position_and_ff) / kAdmittanceKd;
        const double velocity_upper = elbow.dq() +
          ( soft_torque_limit - position_and_ff) / kAdmittanceKd;
        target_velocity = std::clamp(target_velocity, velocity_lower, velocity_upper);
      }
      else
      {
        target_velocity = 0.0;
        target = measured;
      }
      // Re-anchor the position target to measured q after any torque-budget
      // velocity clamp. This prevents a stale integrated target from opposing
      // a faster human motion while keeping the position assistance bounded.
      target = measured + std::clamp(
        kAdmittanceTargetLeadSeconds * target_velocity,
        -kAdmittanceTargetLead, kAdmittanceTargetLead);
      target = std::clamp(target, lower, upper);
      desired_[elbow_index] = static_cast<float>(target);
      desired_velocity_[elbow_index] = static_cast<float>(target_velocity);

      if (!tick(reason)) return finish_fault(reason);
      const auto stamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
      csv << stamp << ',' << measured << ',' << desired_[elbow_index] << ','
          << elbow.dq() << ',' << target_velocity << ',' << elbow.tau_est() << ','
          << ff[elbow_index] << ',' << raw_human_tau << ',' << filtered_human_tau
          << ',' << active_human_tau << ',' << last_total_command_[3] << ','
          << static_cast<int>(elbow.temperature()[0]) << ','
          << static_cast<int>(elbow.temperature()[1]) << '\n';

      int max_casing = std::numeric_limits<int>::min();
      int max_winding = std::numeric_limits<int>::min();
      std::size_t max_casing_joint = kActiveIndices.front();
      std::size_t max_winding_joint = kActiveIndices.front();
      for (const auto index : kActiveIndices)
      {
        const auto& motor = current.motor_state().at(index);
        const int casing = static_cast<int>(motor.temperature()[0]);
        const int winding = static_cast<int>(motor.temperature()[1]);
        if (casing > max_casing) { max_casing = casing; max_casing_joint = index; }
        if (winding > max_winding) { max_winding = winding; max_winding_joint = index; }
      }
      if ((max_casing >= kCasingWarningTemperature ||
           max_winding >= kWindingWarningTemperature) &&
          Clock::now() >= next_thermal_warning)
      {
        std::cout << "THERMAL_WARNING selected_arm=" << max_casing << "C@j"
                  << max_casing_joint << '/' << max_winding << "C@j"
                  << max_winding_joint
                  << "; stop pushing and return toward a low-load pose. "
                     "Official hard stop=85/120C.\n" << std::flush;
        next_thermal_warning = Clock::now() + 2s;
      }
      if (Clock::now() >= next_report)
      {
        csv.flush();
        std::cout << std::fixed << std::setprecision(3)
                  << "ADMITTANCE_STATUS q_deg=" << measured * 180.0 / M_PI
                  << " dq=" << elbow.dq() << " target_dq=" << target_velocity
                  << " human_tau_raw/filtered/active=" << raw_human_tau << '/'
                  << filtered_human_tau << '/' << active_human_tau
                  << " tau_est/ff/total=" << elbow.tau_est() << '/'
                  << ff[elbow_index] << '/' << last_total_command_[3]
                  << " elbow_temp=" << static_cast<int>(elbow.temperature()[0])
                  << '/' << static_cast<int>(elbow.temperature()[1])
                  << "C arm_temp_max=" << max_casing << "C@j" << max_casing_joint
                  << '/' << max_winding << "C@j" << max_winding_joint
                  << '\n' << std::flush;
        next_report += 1s;
      }
      next += 20ms;
      std::this_thread::sleep_until(next);
    }
    if (g_stop_requested.load()) return finish_fault("signal requested");

    current = snapshot(shared_, received, count);
    desired_velocity_[elbow_index] = 0.0F;
    desired_[elbow_index] = static_cast<float>(std::clamp(
      static_cast<double>(current.motor_state().at(elbow_index).q()), lower, upper));
    if (!ramp_elbow_gains(kArmKp, kArmKd, 3.0, reason))
      return finish_after_failure(reason);
    elbow_gain_override_ = false;
    use_total_command_limit_ = false;
    if (!deactivate_fitted_feedforward(reason)) return finish_after_failure(reason);
    return finish_normal("selected-elbow sensorless admittance test complete");
  }

  int execute_full_arm_admittance_session(
    const Eigen::Vector4d& fitted_parameters, bool remote_control = false)
  {
    if (!preflight()) return 2;
    Clock::time_point received;
    std::uint64_t count = 0;
    if (remote_control)
    {
      std::cout << "REMOTE_STANDBY: no rt/arm_sdk publisher exists. Release all "
                   "buttons, then hold F1+X (0x0440) for 0.8 s to start takeover. "
                   "After setup, physical F3 (DDS 0x0080) is the hold-to-drag "
                   "deadman. F1+A records only while the controller is active; L2+B remains "
                   "the native emergency chord.\n" << std::flush;
      bool released = false;
      bool release_timing = false;
      bool timing = false;
      auto pressed_at = Clock::now();
      auto released_at = Clock::now();
      std::uint16_t previous_keys = 0xffff;
      while (!g_stop_requested.load())
      {
        const auto waiting_state = snapshot(shared_, received, count);
        const auto keys = remote_keys(waiting_state);
        const auto toggle_bits = keys & kRemoteToggleMask;
        const bool combo = (keys & kRemoteToggleMask) == kRemoteToggleMask;
        if (keys != previous_keys)
        {
          std::cout << "REMOTE_KEYS value=0x" << std::hex << std::setw(4)
                    << std::setfill('0') << keys << std::dec << std::setfill(' ')
                    << " toggle=" << combo << '\n' << std::flush;
          previous_keys = keys;
        }
        if (!released)
        {
          if (toggle_bits == 0)
          {
            if (!release_timing)
            {
              release_timing = true;
              released_at = Clock::now();
            }
            if (std::chrono::duration<double>(Clock::now() - released_at).count() >=
                kRemoteToggleReleaseSeconds)
              released = true;
          }
          else
          {
            release_timing = false;
          }
        }
        if (!combo)
        {
          timing = false;
        }
        else if (released)
        {
          if (!timing) { timing = true; pressed_at = Clock::now(); }
          if (std::chrono::duration<double>(Clock::now() - pressed_at).count() >=
              kRemoteToggleHoldSeconds)
          {
            std::cout << "REMOTE_ENTER_ACCEPTED: F1+X held for 0.8 s. Keep the "
                         "selected arm untouched during takeover and zeroing.\n"
                      << std::flush;
            break;
          }
        }
        std::this_thread::sleep_for(20ms);
      }
      if (g_stop_requested.load()) return 0;
    }
    auto state = snapshot(shared_, received, count);
    desired_velocity_.fill(0.0F);
    for (const auto index : kControlledJoints)
      initial_desired_[index] = desired_[index] = state.motor_state().at(index).q();
    for (std::size_t j = 0; j < 7; ++j)
    {
      initial_arm_[j] = state.motor_state().at(kActiveIndices[j]).q();
      const double lower = kActiveJointLower[j] + kJointLimitMargin;
      const double upper = kActiveJointUpper[j] - kJointLimitMargin;
      if (initial_arm_[j] <= lower || initial_arm_[j] >= upper)
      {
        std::cerr << "Selected-arm joint " << kActiveIndices[j]
                  << " starts inside the 8 deg mechanical-limit margin; "
                     "no publisher created.\n";
        return 2;
      }
    }

    std::string reason;
    if (!collect_tau_feedforward(shared_, initial_tau_, reason, true,
                                 &kCalibrationTakeoverLimits))
    {
      std::cerr << "Initial tau_est capture failed: " << reason << '\n';
      return 2;
    }
    initial_rpy_ = current_rpy(state);
    Eigen::Matrix<double, 7, 4> unused;
    if (!model_.regression(initial_arm_, initial_rpy_, base_anchor_, unused))
    {
      std::cerr << "Initial KDL gravity calculation failed.\n";
      return 2;
    }
    parameters_ = fitted_parameters;

    int max_casing = std::numeric_limits<int>::min();
    int max_winding = std::numeric_limits<int>::min();
    std::size_t max_casing_joint = 0, max_winding_joint = 0;
    for (const auto index : kActiveIndices)
    {
      const auto& motor = state.motor_state().at(index);
      if (static_cast<int>(motor.temperature()[0]) > max_casing)
      {
        max_casing = motor.temperature()[0]; max_casing_joint = index;
      }
      if (static_cast<int>(motor.temperature()[1]) > max_winding)
      {
        max_winding = motor.temperature()[1]; max_winding_joint = index;
      }
    }
    std::cout << "\nREADY: 7-axis selected-arm sensorless admittance test.\n"
              << "Tuned profile: lower damping/deadbands and faster virtual response; "
                 "measured upper-body hard limit is 2.50 rad/s.\n"
              << "Current selected-arm temperature max=" << max_casing << "C@j"
              << max_casing_joint << '/' << max_winding << "C@j"
              << max_winding_joint << ". Keep hands off until ADMITTANCE_ACTIVE.\n";
    std::string confirmation = remote_control ? "START" : "";
    if (!remote_control)
    {
      std::cout << "Type START to create rt/arm_sdk publisher: " << std::flush;
      while (!g_stop_requested.load())
      {
        pollfd descriptor{STDIN_FILENO, POLLIN, 0};
        if (::poll(&descriptor, 1, 100) > 0 && (descriptor.revents & POLLIN))
        {
          std::getline(std::cin, confirmation);
          break;
        }
      }
    }
    if (g_stop_requested.load() || confirmation != "START")
    {
      std::cout << "Cancelled; publisher was never created.\n";
      return 0;
    }
    if (!final_preflight(reason))
    {
      std::cerr << "Final pre-publisher check failed: " << reason << '\n';
      return 2;
    }

    // The non-remote START gate may remain open for an arbitrary time.  Re-anchor
    // position hold and gravity feedforward to the latest state immediately
    // before creating this full-arm publisher.
    state = snapshot(shared_, received, count);
    desired_velocity_.fill(0.0F);
    for (const auto index : kControlledJoints)
      initial_desired_[index] = desired_[index] =
        state.motor_state().at(index).q();
    for (std::size_t j = 0; j < 7; ++j)
    {
      initial_arm_[j] = state.motor_state().at(kActiveIndices[j]).q();
      const double lower = kActiveJointLower[j] + kJointLimitMargin;
      const double upper = kActiveJointUpper[j] - kJointLimitMargin;
      if (initial_arm_[j] <= lower || initial_arm_[j] >= upper)
      {
        std::cerr << "Selected-arm joint " << kActiveIndices[j]
                  << " moved inside the 8 deg mechanical-limit margin while "
                     "waiting; no publisher created.\n";
        return 2;
      }
    }
    if (!collect_tau_feedforward(shared_, initial_tau_, reason, true,
                                 &kCalibrationTakeoverLimits))
    {
      std::cerr << "Final tau_est capture failed: " << reason << '\n';
      return 2;
    }
    initial_rpy_ = current_rpy(state);
    if (!model_.regression(initial_arm_, initial_rpy_, base_anchor_, unused))
    {
      std::cerr << "Final KDL gravity calculation failed; no publisher created.\n";
      return 2;
    }

    publisher_ = std::make_unique<unitree::robot::ChannelPublisher<LowCmd>>(kArmSdkTopic);
    publisher_->InitChannel();
    fitted_ = false;
    fitted_blend_ = 0.0;
    arm_kp_ = kArmKp;
    arm_kd_ = kArmKd;
    full_arm_gain_override_ = false;
    use_total_command_limit_ = true;
    arm_tracking_limit_ = kAdmittanceSetupTrackingError;
    admittance_tracking_active_ = false;
    tracking_error_exceeded_.fill(false);
    if (!hold_for(2.0, true, reason)) return finish_fault(reason);
    if (!remote_control &&
        !operator_gate("ENABLE fitted gravity compensation and prepare full-arm "
                       "admittance: press ENTER, or type ABORT", reason))
      return finish_after_failure(reason);
    if (!activate_fitted_feedforward(reason)) return finish_after_failure(reason);

    state = snapshot(shared_, received, count);
    for (std::size_t j = 0; j < 7; ++j)
      desired_[kActiveIndices[j]] = state.motor_state().at(kActiveIndices[j]).q();
    if (!ramp_full_arm_gains(kFullAdmittanceKp, kFullAdmittanceKd,
                             3.0, reason, true))
      return finish_after_failure(reason);
    std::cout << "ADMITTANCE_SETTLING: waiting 2 s after low-gain transition.\n"
              << std::flush;
    if (!hold_for(2.0, false, reason)) return finish_fault(reason);

    std::cout << "ADMITTANCE_ZEROING: keep the entire selected arm untouched.\n"
              << std::flush;
    std::array<std::vector<double>, 7> zero_samples;
    const auto zero_deadline = Clock::now() + 12s;
    auto zero_next = Clock::now();
    double zeroing_peak_dq = 0.0;
    while (Clock::now() < zero_deadline && zero_samples[0].size() < 100)
    {
      if (!tick(reason)) return finish_fault(reason);
      state = snapshot(shared_, received, count);
      std::array<float, 35> ff{};
      if (!make_feedforward(state, ff, reason)) return finish_fault(reason);
      bool static_sample = true;
      for (std::size_t j = 0; j < 7; ++j)
      {
        const auto& motor = state.motor_state().at(kActiveIndices[j]);
        zeroing_peak_dq = std::max(zeroing_peak_dq,
          std::abs(static_cast<double>(motor.dq())));
        if (std::abs(static_cast<double>(motor.dq())) > 0.05)
          static_sample = false;
      }
      if (!static_sample)
        for (auto& samples : zero_samples) samples.clear();
      else
        for (std::size_t j = 0; j < 7; ++j)
          zero_samples[j].push_back(static_cast<double>(ff[kActiveIndices[j]]) -
            state.motor_state().at(kActiveIndices[j]).tau_est());
      zero_next += 20ms;
      std::this_thread::sleep_until(zero_next);
    }
    if (zero_samples[0].size() < 100)
      return finish_after_failure("selected arm was not static during admittance zeroing "
        "(peak_dq=" + std::to_string(zeroing_peak_dq) + ")");
    Pose zero_bias{};
    Pose zero_spread{};
    for (std::size_t j = 0; j < 7; ++j)
    {
      zero_bias[j] = median(zero_samples[j]);
      zero_spread[j] = percentile(zero_samples[j], 0.9) -
                       percentile(zero_samples[j], 0.1);
      if (!std::isfinite(zero_bias[j]) || zero_spread[j] > kStableTauSpread)
        return finish_after_failure("full-arm residual-torque zeroing unstable at j" +
                                    std::to_string(kActiveIndices[j]));
    }
    std::cout << std::fixed << std::setprecision(3) << "ADMITTANCE_ZEROED";
    for (std::size_t j = 0; j < 7; ++j)
      std::cout << " j" << kActiveIndices[j] << '=' << zero_bias[j]
                << "+/-" << zero_spread[j];
    std::cout << " Nm\n" << std::flush;

    if (!remote_control &&
        !operator_gate("ARM all 7 selected-arm axes: press ENTER to enter admittance, "
                       "or type ABORT", reason))
      return finish_after_failure(reason);

    // The arm becomes actively held before the input loop starts. F3 can only
    // lower these gains after it has first been observed released. Once pressed,
    // it enters DRAG immediately from the current measured pose.
    if (!ramp_full_arm_gains(kDeadmanHoldKp, kDeadmanHoldKd,
                             0.75, reason, true))
      return finish_after_failure(reason);
    drag_velocity_tolerant_ = true;

    // The active loop converts a 3 degree selected-arm tracking excursion into
    // BRAKE/HOLD. Only a 6 degree instantaneous excursion remains a hard fault;
    // the untouched left arm keeps the normal setup tracking bound.
    admittance_tracking_active_ = true;
    tracking_error_exceeded_.fill(false);

    std::filesystem::create_directories(std::filesystem::path(csv_path_).parent_path());
    std::ofstream csv(csv_path_, std::ios::trunc);
    if (!csv) return finish_after_failure("cannot create full-arm admittance CSV");
    std::ofstream capture_csv;
    if (remote_control)
    {
      capture_csv.open(csv_path_ + ".captures.csv", std::ios::trunc);
      if (!capture_csv)
        return finish_after_failure("cannot create remote-teach capture CSV");
      capture_csv << "stamp_ns,capture_id,stable,max_abs_dq,image_status,"
                     "image_stamp_ns,image_path";
      for (std::size_t j = 0; j < 7; ++j)
        capture_csv << ",j" << kActiveIndices[j] << "_q,j" << kActiveIndices[j]
                    << "_dq,j" << kActiveIndices[j] << "_tau_est,j"
                    << kActiveIndices[j] << "_target_q,j" << kActiveIndices[j]
                    << "_target_dq,j" << kActiveIndices[j] << "_casing,j"
                    << kActiveIndices[j] << "_winding";
      capture_csv << '\n';
    }
    csv << "stamp_ns,drag_state,deadman,remote_keys,arm_intent";
    for (std::size_t j = 0; j < 7; ++j)
      csv << ",j" << kActiveIndices[j] << "_q,j" << kActiveIndices[j]
          << "_target_q,j" << kActiveIndices[j] << "_dq,j" << kActiveIndices[j]
          << "_target_dq,j" << kActiveIndices[j] << "_virtual_dq,j" << kActiveIndices[j]
          << "_filtered_dq,j" << kActiveIndices[j] << "_speed_envelope,j"
          << kActiveIndices[j] << "_passthrough,j" << kActiveIndices[j]
          << "_capture_dq,j" << kActiveIndices[j]
          << "_raw_motion_follow,j" << kActiveIndices[j]
          << "_reference_follow,j" << kActiveIndices[j]
          << "_hold_blend,j" << kActiveIndices[j]
          << "_hold_integral_tau,j" << kActiveIndices[j]
          << "_tau_est,j" << kActiveIndices[j]
          << "_tau_ff,j" << kActiveIndices[j] << "_breakaway_tau,j"
          << kActiveIndices[j] << "_raw,j" << kActiveIndices[j]
          << "_filtered,j" << kActiveIndices[j] << "_active,j" << kActiveIndices[j]
          << "_bias,j" << kActiveIndices[j] << "_engaged,j" << kActiveIndices[j]
          << "_total,j" << kActiveIndices[j] << "_kp,j" << kActiveIndices[j]
          << "_kd,j" << kActiveIndices[j]
          << "_casing,j" << kActiveIndices[j]
          << "_winding";
    csv << '\n';
    std::cout << "DEADMAN_HOLD_ACTIVE: selected arm is held; physical F3 maps to "
                 "DDS 0x0080. Hold F3 to drag immediately, and release "
                 "it to brake and hold. "
              << (remote_control
                ? "Hold F1+X for 0.8 s to exit; press F1+A to record.\n"
                : "Type STOP to restore normal gains and exit.\n")
              << std::flush;

    Pose filtered{};
    Pose filtered_velocity{};
    Pose speed_envelope{};
    Pose passthrough_applied{};
    Pose breakaway_assist{};
    Pose reference_position{};
    Pose capture_velocity{};
    Pose distal_hold_blend{};
    Pose distal_hold_integral_tau{};
    std::array<bool, 7> reference_follow_active{};
    Pose target_velocity{};
    Pose command_velocity{};
    Pose raw{};
    Pose active{};
    std::array<bool, 7> effort_engaged{};
    std::array<float, 35> ff{};
    auto next = Clock::now();
    auto next_flush = next + 1s;
    auto next_thermal_warning = next;
    DragControlState drag_state = DragControlState::HOLD;
    auto drag_state_since = next;
    bool operator_intent_active = false;
    bool deadman_rearm_required = true;
    bool deadman_release_timing = false;
    auto deadman_released_at = next;
    bool deadman_pressed = false;
    bool exit_requested = false;
    std::uint16_t active_previous_keys = 0xffff;
    bool remote_toggle_armed = false;
    bool remote_toggle_release_timing = false;
    bool remote_toggle_timing = false;
    bool remote_record_previous = true;
    auto remote_toggle_pressed_at = next;
    auto remote_toggle_released_at = next;
    std::size_t capture_id = 0;
    additional_tau_.fill(0.0);
    for (std::size_t j = 0; j < 7; ++j)
      reference_position[j] = state.motor_state().at(kActiveIndices[j]).q();
    const auto transition_drag_state = [&](DragControlState next_state,
                                           const char* cause) {
      if (drag_state == next_state) return;
      std::cout << "DEADMAN_STATE " << drag_state_name(drag_state) << " -> "
                << drag_state_name(next_state) << " cause=" << cause << '\n'
                << std::flush;
      drag_state = next_state;
      drag_state_since = Clock::now();
      if (next_state == DragControlState::BRAKE)
      {
        deadman_rearm_required = true;
        operator_intent_active = false;
        target_velocity.fill(0.0);
        effort_engaged.fill(false);
      }
    };
    while (!g_stop_requested.load())
    {
      if (!remote_control)
      {
        pollfd descriptor{STDIN_FILENO, POLLIN, 0};
        if (::poll(&descriptor, 1, 0) > 0 && (descriptor.revents & POLLIN))
        {
          std::string line;
          std::getline(std::cin, line);
          if (line == "STOP" || line == "EXIT") exit_requested = true;
          if (line == "ABORT") return finish_after_failure("operator requested ABORT");
        }
      }
      state = snapshot(shared_, received, count);
      const auto keys = remote_keys(state);
      deadman_pressed = (keys & kRemoteDeadmanMask) == kRemoteDeadmanMask;
      if (keys != active_previous_keys)
      {
        std::cout << "ACTIVE_REMOTE_KEYS value=0x" << std::hex << std::setw(4)
                  << std::setfill('0') << keys << std::dec << std::setfill(' ')
                  << " deadman=" << deadman_pressed << '\n' << std::flush;
        active_previous_keys = keys;
      }
      bool remote_record_event = false;
      if (remote_control)
      {
        const auto toggle_bits = keys & kRemoteToggleMask;
        const bool toggle_combo = (keys & kRemoteToggleMask) == kRemoteToggleMask;
        const bool record_combo = (keys & kRemoteRecordMask) == kRemoteRecordMask;
        if (!remote_toggle_armed)
        {
          // A single long entry press may span takeover and zeroing.  Require
          // both toggle buttons to be continuously released before the exit
          // detector is armed, so that press can never toggle twice.
          if (toggle_bits == 0)
          {
            if (!remote_toggle_release_timing)
            {
              remote_toggle_release_timing = true;
              remote_toggle_released_at = Clock::now();
            }
            if (std::chrono::duration<double>(Clock::now() -
                  remote_toggle_released_at).count() >=
                kRemoteToggleReleaseSeconds)
            {
              remote_toggle_armed = true;
              remote_toggle_timing = false;
              std::cout << "REMOTE_EXIT_ARMED: F1+X was fully released; the "
                           "next 0.8 s hold will exit admittance.\n"
                        << std::flush;
            }
          }
          else
          {
            remote_toggle_release_timing = false;
          }
        }
        else if (!toggle_combo)
        {
          remote_toggle_timing = false;
        }
        else if (remote_toggle_armed)
        {
          if (!remote_toggle_timing)
          {
            remote_toggle_timing = true;
            remote_toggle_pressed_at = Clock::now();
          }
          if (std::chrono::duration<double>(Clock::now() -
                remote_toggle_pressed_at).count() >= kRemoteToggleHoldSeconds)
          {
            std::cout << "REMOTE_EXIT_ACCEPTED: braking before normal arm control.\n"
                      << std::flush;
            exit_requested = true;
            remote_toggle_armed = false;
            remote_toggle_timing = false;
          }
        }
        remote_record_event = record_combo && !remote_record_previous;
        remote_record_previous = record_combo;
      }

      // A release must be observed before each new drag. This prevents a held
      // button from spanning startup, a brake event, or a stale-state recovery.
      if (!deadman_pressed)
      {
        if (!deadman_release_timing)
        {
          deadman_release_timing = true;
          deadman_released_at = Clock::now();
        }
        if (std::chrono::duration<double>(Clock::now() - deadman_released_at).count() >=
            kDeadmanReleaseSeconds)
          deadman_rearm_required = false;
      }
      else
      {
        deadman_release_timing = false;
      }

      if (drag_state == DragControlState::HOLD && deadman_pressed &&
          !deadman_rearm_required)
      {
        // A new press is a new drag operation. Start from the pose the user is
        // actually holding, not from an unfinished HOLD recovery, and discard
        // all dynamic state left by the previous drag/brake cycle.
        for (std::size_t j = 0; j < 7; ++j)
        {
          const auto index = kActiveIndices[j];
          const auto& motor = state.motor_state().at(index);
          reference_position[j] = motor.q();
          desired_[index] = motor.q();
          desired_velocity_[index] = 0.0F;
          filtered[j] = 0.0;
          filtered_velocity[j] = motor.dq();
          speed_envelope[j] = std::abs(static_cast<double>(motor.dq()));
          passthrough_applied[j] = 0.0;
          breakaway_assist[j] = 0.0;
          capture_velocity[j] = 0.0;
          distal_hold_blend[j] = 0.0;
          distal_hold_integral_tau[j] = 0.0;
          target_velocity[j] = 0.0;
          command_velocity[j] = 0.0;
          raw[j] = 0.0;
          active[j] = 0.0;
          effort_engaged[j] = false;
          reference_follow_active[j] = false;
          additional_tau_[j] = 0.0;
          tracking_error_exceeded_[index] = false;
        }
        operator_intent_active = true;
        transition_drag_state(DragControlState::DRAG, "F3 pressed");
      }
      else if (drag_state == DragControlState::DRAG && !deadman_pressed)
        transition_drag_state(DragControlState::BRAKE, "F3 released");

      if (exit_requested && drag_state != DragControlState::HOLD &&
          drag_state != DragControlState::BRAKE)
        transition_drag_state(DragControlState::BRAKE, "normal exit requested");

      const bool enforce_active_arm_hard_tracking =
        drag_state == DragControlState::DRAG;
      operator_intent_active = drag_state == DragControlState::DRAG &&
        deadman_pressed;
      if (drag_state == DragControlState::DRAG)
      {
        for (std::size_t j = 0; j < 7; ++j)
        {
          const auto& motor = state.motor_state().at(kActiveIndices[j]);
          const double measured = motor.q();
          const double velocity = motor.dq();
          const double lower = kActiveJointLower[j] + kJointLimitMargin;
          const double upper = kActiveJointUpper[j] - kJointLimitMargin;
          const double stopping_distance = velocity * kJointLimitPredictionLatency +
            std::copysign(velocity * velocity /
              (2.0 * kJointLimitBrakeAcceleration), velocity);
          const double predicted_stop = measured + stopping_distance;
          const double tracking = std::abs(reference_position[j] - measured);
          if (std::abs(velocity) > kDragVelocityNominalLimit)
          {
            transition_drag_state(DragControlState::BRAKE,
                                  "selected-arm soft velocity limit");
            break;
          }
          if (measured <= lower || measured >= upper ||
              predicted_stop <= lower || predicted_stop >= upper)
          {
            transition_drag_state(DragControlState::BRAKE,
                                  "joint-limit stopping-distance guard");
            break;
          }
          if (tracking > kReferenceSoftBrakeError)
          {
            if (tracking > kAdmittanceHardTrackingError)
            {
              reason = "tracking error exceeds instantaneous hard limit at joint " +
                std::to_string(kActiveIndices[j]) + " (error_deg=" +
                std::to_string(tracking * 180.0 / M_PI) + ")";
              fault_latched_ = true;
              return finish_fault(reason);
            }
            if (operator_intent_active)
            {
              reference_position[j] = std::clamp(reference_position[j],
                measured - kReferenceMaxError[j],
                measured + kReferenceMaxError[j]);
            }
            else
            {
              transition_drag_state(DragControlState::BRAKE,
                                    "reference tracking without arm intent");
              break;
            }
          }
        }
      }

      if (drag_state == DragControlState::BRAKE)
      {
        double max_speed = 0.0;
        for (const auto index : kActiveIndices)
          max_speed = std::max(max_speed, std::abs(static_cast<double>(
            state.motor_state().at(index).dq())));
        const double braking_time = std::chrono::duration<double>(
          Clock::now() - drag_state_since).count();
        if ((braking_time >= kBrakeMinimumSeconds &&
             max_speed <= kBrakeSettledVelocity) ||
            braking_time >= kBrakeMaximumSeconds)
          transition_drag_state(DragControlState::HOLD,
                                max_speed <= kBrakeSettledVelocity
                                  ? "brake settled" : "brake timeout");
      }
      if (exit_requested && drag_state == DragControlState::HOLD)
      {
        std::cout << "NORMAL_EXIT_HOLD: brake complete; restoring normal gains.\n"
                  << std::flush;
        break;
      }
      if (!make_feedforward(state, ff, reason)) return finish_fault(reason);
      for (std::size_t j = 0; j < 7; ++j)
      {
        const auto index = kActiveIndices[j];
        const auto& motor = state.motor_state().at(index);
        raw[j] = static_cast<double>(ff[index]) - motor.tau_est() - zero_bias[j];
        const double alpha = kTickSeconds /
          (kFullAdmittanceFilterSeconds[j] + kTickSeconds);
        filtered[j] += alpha * (raw[j] - filtered[j]);
        // zero_bias is intentionally frozen after ADMITTANCE_ZEROED. Updating
        // it in HOLD can learn the PD recovery force or an operator push as a
        // gravity/model offset and corrupt every later drag entry.
        const double off_threshold = kFullAdmittanceDeadband[j] *
          kFullAdmittanceOffThresholdRatio;
        if (drag_state != DragControlState::DRAG)
          effort_engaged[j] = false;
        else if (!effort_engaged[j] &&
                 std::abs(filtered[j]) > kFullAdmittanceDeadband[j])
          effort_engaged[j] = true;
        else if (effort_engaged[j] && std::abs(filtered[j]) < off_threshold)
          effort_engaged[j] = false;
        active[j] = 0.0;
        if (effort_engaged[j])
          active[j] = std::copysign(std::abs(filtered[j]) -
                                    off_threshold, filtered[j]);
        const double requested = drag_state == DragControlState::DRAG
          ? std::clamp(kFullAdmittanceVelocityPerTorque[j] * active[j],
              -kFullAdmittanceMaxVelocity[j], kFullAdmittanceMaxVelocity[j])
          : 0.0;
        const double virtual_acceleration =
          drag_state == DragControlState::BRAKE ? kBrakeCommandAcceleration :
          kFullAdmittanceMaxAcceleration[j];
        target_velocity[j] += std::clamp(requested - target_velocity[j],
          -virtual_acceleration * kTickSeconds,
           virtual_acceleration * kTickSeconds);

        const double measured = motor.q();
        // Preserve the low-speed hand feel while restoring strong damping as
        // a human-driven or coupled axis accelerates. Gain scheduling uses a
        // fast-attack/slow-release speed envelope: a real speed rise brakes in
        // one tick, while quantized j26 estimates cannot drop Kd every 20 ms.
        // The noisy wrist-roll axis also enters soft braking earlier.
        const double velocity_alpha = kTickSeconds /
          (kFullAdmittanceVelocityFilterSeconds[j] + kTickSeconds);
        filtered_velocity[j] += velocity_alpha *
          (static_cast<double>(motor.dq()) - filtered_velocity[j]);
        const double raw_speed = std::abs(static_cast<double>(motor.dq()));
        if (raw_speed >= speed_envelope[j])
          speed_envelope[j] = raw_speed;
        else
        {
          const double release_alpha = kTickSeconds /
            (kFullAdmittanceSpeedEnvelopeReleaseSeconds[j] + kTickSeconds);
          speed_envelope[j] += release_alpha *
            (raw_speed - speed_envelope[j]);
        }
        const double speed = speed_envelope[j];
        const double damping_blend = quintic(std::clamp(
          (speed - kFullAdmittanceDampingStartVelocity[j]) /
          (kFullAdmittanceDampingFullVelocity[j] -
           kFullAdmittanceDampingStartVelocity[j]), 0.0, 1.0));
        const bool drag_gains = drag_state == DragControlState::DRAG;
        const double kp_target = drag_gains ? kFullAdmittanceKp[j] :
          kDeadmanHoldKp[j];
        const double drag_kd = kFullAdmittanceKd[j] + damping_blend *
          (kFullAdmittanceHighSpeedKd[j] - kFullAdmittanceKd[j]);
        const double kd_target = drag_gains ? drag_kd : kDeadmanHoldKd[j];
        full_arm_kp_[j] += std::clamp(kp_target - full_arm_kp_[j],
          -kGainKpSlewPerSecond * kTickSeconds,
           kGainKpSlewPerSecond * kTickSeconds);
        full_arm_kd_[j] += std::clamp(kd_target - full_arm_kd_[j],
          -kGainKdSlewPerSecond * kTickSeconds,
           kGainKdSlewPerSecond * kTickSeconds);
        const double brake_blend = quintic(std::clamp(
          (speed - kFullAdmittanceBrakeStartVelocity[j]) /
          (kFullAdmittanceBrakeFullVelocity[j] -
           kFullAdmittanceBrakeStartVelocity[j]), 0.0, 1.0));
        if (drag_state == DragControlState::DRAG)
          target_velocity[j] *= 1.0 - brake_blend;
        const double low_speed_passthrough =
          drag_state == DragControlState::DRAG && effort_engaged[j]
          ? kFullAdmittanceVelocityPassthrough[j] : 0.0;
        const double passthrough_blend = low_speed_passthrough *
          (1.0 - quintic(std::clamp(
            (speed - kFullAdmittancePassthroughFadeStart) /
            (kFullAdmittancePassthroughFadeEnd -
             kFullAdmittancePassthroughFadeStart), 0.0, 1.0)));
        passthrough_applied[j] = passthrough_blend;
        capture_velocity[j] = 0.0;
        if (drag_state == DragControlState::DRAG)
        {
          command_velocity[j] = std::clamp(target_velocity[j] +
            passthrough_blend * filtered_velocity[j],
            -kReferenceMaxCommandVelocity, kReferenceMaxCommandVelocity);
        }
        else if (drag_state == DragControlState::BRAKE)
        {
          command_velocity[j] += std::clamp(-command_velocity[j],
            -kBrakeCommandAcceleration * kTickSeconds,
             kBrakeCommandAcceleration * kTickSeconds);
        }
        else
        {
          command_velocity[j] = 0.0;
        }
        reference_follow_active[j] =
          drag_state == DragControlState::DRAG && operator_intent_active;
        if (j >= 5)
        {
          const double hold_target = drag_state == DragControlState::HOLD ? 1.0 : 0.0;
          const double hold_seconds = hold_target > distal_hold_blend[j]
            ? kDistalHoldAttackSeconds : kDistalHoldReleaseSeconds;
          distal_hold_blend[j] += std::clamp(
            hold_target - distal_hold_blend[j],
            -kTickSeconds / hold_seconds, kTickSeconds / hold_seconds);

          if (drag_state != DragControlState::HOLD)
          {
            const double release_alpha = kTickSeconds /
              (kDistalHoldIntegralReleaseSeconds + kTickSeconds);
            distal_hold_integral_tau[j] *= 1.0 - release_alpha;
          }
          else if (distal_hold_blend[j] >= 0.99 &&
                   raw_speed < kDistalHoldIntegralMaxVelocity)
          {
            const double reference_error = reference_position[j] - measured;
            const double integral_error = std::copysign(std::max(
              std::abs(reference_error) - kDistalHoldIntegralDeadband, 0.0),
              reference_error);
            distal_hold_integral_tau[j] = std::clamp(
              distal_hold_integral_tau[j] + kDistalHoldIntegralGain *
                integral_error * kTickSeconds,
              -kDistalHoldIntegralLimit, kDistalHoldIntegralLimit);
          }
        }
        if (kBreakawayTorque[j] > 0.0)
        {
          const double fade = 1.0 - quintic(std::clamp(
            (speed - kBreakawayFadeStartVelocity) /
            (kBreakawayFadeEndVelocity -
             kBreakawayFadeStartVelocity), 0.0, 1.0));
          const bool assist_enabled = drag_state == DragControlState::DRAG &&
            effort_engaged[j];
          const double requested_assist = assist_enabled
            ? std::copysign(kBreakawayTorque[j] * fade, active[j]) : 0.0;
          const bool same_direction = requested_assist * breakaway_assist[j] >= 0.0;
          const bool building = same_direction &&
            std::abs(requested_assist) > std::abs(breakaway_assist[j]);
          const double rate = building ? kBreakawayAttackRate :
            kBreakawayReleaseRate;
          breakaway_assist[j] += std::clamp(
            requested_assist - breakaway_assist[j],
            -rate * kTickSeconds, rate * kTickSeconds);
        }
        additional_tau_[j] = breakaway_assist[j] +
          distal_hold_integral_tau[j];
        double target = std::clamp(reference_position[j],
          kActiveJointLower[j] + kJointLimitMargin,
          kActiveJointUpper[j] - kJointLimitMargin);
        const double soft_limit = kActiveTauEstLimits[j] - kAdmittanceTorqueReserve;
        const double position_and_ff = static_cast<double>(ff[index]) +
          additional_tau_[j] + full_arm_kp_[j] * (target - measured);
        if (std::abs(position_and_ff) < soft_limit)
        {
          const double velocity_lower = motor.dq() +
            (-soft_limit - position_and_ff) / full_arm_kd_[j];
          const double velocity_upper = motor.dq() +
            ( soft_limit - position_and_ff) / full_arm_kd_[j];
          command_velocity[j] = std::clamp(command_velocity[j],
                                            velocity_lower, velocity_upper);
        }
        else
        {
          target_velocity[j] = 0.0;
          command_velocity[j] = 0.0;
        }
        if (drag_state == DragControlState::DRAG)
          reference_position[j] += command_velocity[j] * kTickSeconds;
        if (reference_follow_active[j])
          reference_position[j] = std::clamp(reference_position[j],
            measured - kReferenceMaxError[j], measured + kReferenceMaxError[j]);
        reference_position[j] = std::clamp(reference_position[j],
          kActiveJointLower[j] + kJointLimitMargin,
          kActiveJointUpper[j] - kJointLimitMargin);
        desired_[index] = static_cast<float>(reference_position[j]);
        desired_velocity_[index] = static_cast<float>(command_velocity[j]);
      }
      if (!tick(reason, true, enforce_active_arm_hard_tracking, true))
        return finish_fault(reason);
      const auto stamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
      csv << stamp << ',' << drag_state_name(drag_state) << ','
          << (deadman_pressed ? 1 : 0) << ',' << keys << ','
          << (operator_intent_active ? 1 : 0);
      for (std::size_t j = 0; j < 7; ++j)
      {
        const auto index = kActiveIndices[j];
        const auto& motor = state.motor_state().at(index);
        csv << ',' << motor.q() << ',' << desired_[index] << ',' << motor.dq()
            << ',' << command_velocity[j] << ',' << target_velocity[j] << ','
            << filtered_velocity[j] << ',' << speed_envelope[j] << ','
            << passthrough_applied[j] << ',' << capture_velocity[j] << ",0,"
            << (reference_follow_active[j] ? 1 : 0) << ','
            << distal_hold_blend[j] << ','
            << distal_hold_integral_tau[j] << ','
            << motor.tau_est() << ',' << ff[index]
            << ',' << additional_tau_[j] << ',' << raw[j] << ','
            << filtered[j] << ',' << active[j] << ','
            << zero_bias[j] << ',' << (effort_engaged[j] ? 1 : 0) << ','
            << last_total_command_[j] << ',' << full_arm_kp_[j] << ','
            << full_arm_kd_[j] << ','
            << static_cast<int>(motor.temperature()[0])
            << ',' << static_cast<int>(motor.temperature()[1]);
      }
      csv << '\n';
      if (remote_control && remote_record_event)
      {
        double max_abs_dq = 0.0;
        for (const auto index : kActiveIndices)
          max_abs_dq = std::max(max_abs_dq, std::abs(static_cast<double>(
            state.motor_state().at(index).dq())));
        const bool stable = max_abs_dq <= 0.03;
        ++capture_id;
        // Camera integration intentionally remains non-blocking and disabled
        // in this first version.  These columns are the stable interface for
        // a future nearest-image capture sink.
        capture_csv << stamp << ',' << capture_id << ',' << (stable ? 1 : 0)
                    << ',' << max_abs_dq << ",NOT_CONFIGURED,,";
        for (std::size_t j = 0; j < 7; ++j)
        {
          const auto index = kActiveIndices[j];
          const auto& motor = state.motor_state().at(index);
          capture_csv << ',' << motor.q() << ',' << motor.dq() << ','
                      << motor.tau_est() << ',' << desired_[index] << ','
                      << desired_velocity_[index] << ','
                      << static_cast<int>(motor.temperature()[0]) << ','
                      << static_cast<int>(motor.temperature()[1]);
        }
        capture_csv << '\n';
        capture_csv.flush();
        std::cout << "REMOTE_CAPTURE id=" << capture_id
                  << " stable=" << stable << " max_dq=" << max_abs_dq
                  << " image=NOT_CONFIGURED file=" << csv_path_
                  << ".captures.csv\n" << std::flush;
      }
      if (Clock::now() >= next_flush) { csv.flush(); next_flush += 1s; }

      max_casing = std::numeric_limits<int>::min();
      max_winding = std::numeric_limits<int>::min();
      for (const auto index : kActiveIndices)
      {
        const auto& motor = state.motor_state().at(index);
        if (static_cast<int>(motor.temperature()[0]) > max_casing)
        { max_casing = motor.temperature()[0]; max_casing_joint = index; }
        if (static_cast<int>(motor.temperature()[1]) > max_winding)
        { max_winding = motor.temperature()[1]; max_winding_joint = index; }
      }
      if ((max_casing >= kCasingWarningTemperature ||
           max_winding >= kWindingWarningTemperature) &&
          Clock::now() >= next_thermal_warning)
      {
        std::cout << "THERMAL_WARNING selected_arm=" << max_casing << "C@j"
                  << max_casing_joint << '/' << max_winding << "C@j"
                  << max_winding_joint << "; return toward a low-load pose.\n"
                  << std::flush;
        next_thermal_warning = Clock::now() + 2s;
      }
      next += 20ms;
      std::this_thread::sleep_until(next);
    }
    if (g_stop_requested.load()) return finish_fault("signal requested");

    // A confirmed operator exit takes precedence over the debounced active
    // tracking guard while normal gains and feedforward are restored.
    admittance_tracking_active_ = false;
    arm_tracking_limit_ = kAdmittanceSetupTrackingError;
    tracking_error_exceeded_.fill(false);
    additional_tau_.fill(0.0);
    state = snapshot(shared_, received, count);
    for (std::size_t j = 0; j < 7; ++j)
    {
      const auto index = kActiveIndices[j];
      desired_velocity_[index] = 0.0F;
      desired_[index] = state.motor_state().at(index).q();
    }
    const Pose normal_kp{kArmKp, kArmKp, kArmKp, kArmKp,
                         kArmKp, kArmKp, kArmKp};
    const Pose normal_kd{kArmKd, kArmKd, kArmKd, kArmKd,
                         kArmKd, kArmKd, kArmKd};
    if (!ramp_full_arm_gains(normal_kp, normal_kd, 3.0, reason, true))
      return finish_after_failure(reason);
    full_arm_gain_override_ = false;
    use_total_command_limit_ = false;
    drag_velocity_tolerant_ = false;
    if (!deactivate_fitted_feedforward(reason)) return finish_after_failure(reason);
    return finish_normal("full selected-arm sensorless admittance test complete");
  }

  int execute_full_arm_admittance(const Eigen::Vector4d& fitted_parameters)
  {
    return execute_full_arm_admittance_session(fitted_parameters, false);
  }

  int execute_remote_admittance(const Eigen::Vector4d& fitted_parameters)
  {
    const std::filesystem::path base(csv_path_);
    const auto parent = base.parent_path();
    const auto stem = base.stem().string();
    const auto extension = base.extension().string().empty()
      ? std::string(".csv") : base.extension().string();
    std::size_t session = 0;
    std::cout << "REMOTE_TEACH_DAEMON: F1+X hold starts/stops the controller; "
                 "physical F3 (DDS 0x0080) must be held to drag. F1+A records "
                 "a joint snapshot. Camera sink is reserved but "
                 "not configured. Ctrl+C exits while idle.\n" << std::flush;
    while (!g_stop_requested.load())
    {
      ++session;
      std::ostringstream suffix;
      suffix << "_session" << std::setw(3) << std::setfill('0') << session;
      csv_path_ = (parent / (stem + suffix.str() + extension)).string();
      const int result = execute_full_arm_admittance_session(
        fitted_parameters, true);
      if (result != 0 || g_stop_requested.load()) return result;
      publisher_.reset();
      fitted_ = false;
      fitted_blend_ = 0.0;
      full_arm_gain_override_ = false;
      elbow_gain_override_ = false;
      use_total_command_limit_ = false;
      desired_velocity_.fill(0.0F);
      std::cout << "REMOTE_RETURNED_TO_STANDBY: session " << session
                << " complete; publisher destroyed.\n" << std::flush;
    }
    return 0;
  }

  int execute_full_arm_drag(const Eigen::Vector4d& fitted_parameters,
                            bool interactive_refit = false,
                            std::size_t minimum_new_cases = 1)
  {
    if (!preflight()) return 2;
    Clock::time_point received_at;
    std::uint64_t samples = 0;
    const auto state = snapshot(shared_, received_at, samples);
    for (const auto index : kControlledJoints)
      initial_desired_[index] = desired_[index] = state.motor_state().at(index).q();
    for (std::size_t j = 0; j < 7; ++j)
    {
      initial_arm_[j] = state.motor_state().at(kActiveIndices[j]).q();
      const double lower = kActiveJointLower[j] + kJointLimitMargin;
      const double upper = kActiveJointUpper[j] - kJointLimitMargin;
      if (initial_arm_[j] <= lower || initial_arm_[j] >= upper)
      {
        std::cerr << "Selected joint " << kActiveIndices[j]
                  << " starts inside the 5 deg mechanical-limit margin; "
                     "no publisher created.\n";
        return 2;
      }
    }

    std::string reason;
    if (!collect_tau_feedforward(shared_, initial_tau_, reason, true,
                                 &kCalibrationTakeoverLimits))
    {
      std::cerr << "Initial tau_est capture failed: " << reason << '\n';
      return 2;
    }
    initial_rpy_ = current_rpy(state);
    Eigen::Matrix<double, 7, 4> unused;
    if (!model_.regression(initial_arm_, initial_rpy_, base_anchor_, unused))
    {
      std::cerr << "Initial KDL gravity calculation failed.\n";
      return 2;
    }
    parameters_ = fitted_parameters;
    const std::size_t original_training_count = training_.size();

    std::cout << "\nREADY: full selected-arm drag test with 50 Hz joint logging.\n"
              << "All seven selected-arm joints use Kp 3; shoulder Kd is speed-adaptive "
              << "1.3..3.0, elbow/wrist Kd remains 2.5.\n"
              << "Targets directly follow measured q; official URDF ranges retain a "
              << "5 deg mechanical-stop margin.\n"
              << "Per operator request, no real-time self-collision guard is enabled; "
              << "the operator must keep the arm clear of the torso.\n"
              << "L2+B/state/FSM/motor/per-joint-torque/velocity/temperature/tracking "
              << "faults stop publishing immediately.\n"
              << "Type START to create rt/arm_sdk publisher: " << std::flush;
    std::string confirmation;
    while (!g_stop_requested.load())
    {
      pollfd descriptor{STDIN_FILENO, POLLIN, 0};
      if (::poll(&descriptor, 1, 100) > 0 && (descriptor.revents & POLLIN))
      {
        std::getline(std::cin, confirmation);
        break;
      }
    }
    if (g_stop_requested.load() || confirmation != "START")
    {
      std::cout << "Cancelled; publisher was never created.\n";
      return 0;
    }
    if (!final_preflight(reason))
    {
      std::cerr << "Final pre-publisher check failed: " << reason << '\n';
      return 2;
    }
    publisher_ = std::make_unique<unitree::robot::ChannelPublisher<LowCmd>>(kArmSdkTopic);
    publisher_->InitChannel();
    fitted_ = false;
    fitted_blend_ = 0.0;
    arm_kp_ = kArmKp;
    arm_kd_ = kArmKd;
    elbow_gain_override_ = false;
    full_arm_gain_override_ = false;
    waist_tracking_limit_ = 6.0 * M_PI / 180.0;
    use_total_command_limit_ = true;
    if (!hold_for(2.0, true, reason)) return finish_fault(reason);

    if (!operator_gate("ENABLE fitted dynamic gravity compensation: press ENTER, "
                       "or type ABORT", reason))
      return finish_after_failure(reason);
    if (!activate_fitted_feedforward(reason)) return finish_after_failure(reason);
    if (!operator_gate("ARM full selected-arm drag: press ENTER; only touch after "
                       "FULL_ARM_DRAG_ACTIVE, or type ABORT", reason))
      return finish_after_failure(reason);

    const Pose drag_kp{3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0};
    const Pose drag_kd{1.3, 1.3, 1.3, 2.5, 2.5, 2.5, 2.5};
    if (!ramp_full_arm_gains(drag_kp, drag_kd, 3.0, reason, true))
      return finish_after_failure(reason);
    drag_velocity_tolerant_ = true;
    drag_velocity_exceeded_ = false;

    std::filesystem::create_directories(std::filesystem::path(csv_path_).parent_path());
    std::ofstream csv(csv_path_, std::ios::trunc);
    if (!csv) return finish_normal("cannot create full-arm drag CSV");
    csv << "stamp_ns,imu_roll,imu_pitch,imu_yaw";
    for (const auto index : kActiveIndices)
      csv << ",q" << index << ",target" << index << ",dq" << index
          << ",tau_est" << index << ",tau_ff" << index << ",kp" << index
          << ",kd" << index << ",total_cmd" << index
          << ",casing" << index << ",winding" << index;
    csv << '\n';

    std::ofstream cases;
    if (interactive_refit)
    {
      cases.open(csv_path_ + ".cases.csv", std::ios::trunc);
      if (!cases) return finish_normal("cannot create selected-cases CSV");
      write_csv_header(cases);
    }

    std::cout << "FULL_ARM_DRAG_ACTIVE: move the selected arm slowly; avoid torso contact.\n";
    auto next = Clock::now();
    auto next_flush = next + 1s;
    bool awaiting_next = false;
    bool thermal_warning_active = false;
    bool velocity_warning_active = false;
    const auto print_selection = [&]() {
      if (interactive_refit)
        std::cout << (awaiting_next
          ? "SELECT [2=NEXT, 4=FIT_AND_EXIT]: "
          : "SELECT [1=RECORD, 4=FIT_AND_EXIT]: ") << std::flush;
      else
        std::cout << "SELECT [STOP=EXIT]: " << std::flush;
    };
    print_selection();
    while (!g_stop_requested.load())
    {
      pollfd descriptor{STDIN_FILENO, POLLIN, 0};
      if (::poll(&descriptor, 1, 0) > 0 && (descriptor.revents & POLLIN))
      {
        std::string line;
        std::getline(std::cin, line);
        if (line == "STOP" || line == "EXIT" || line == "4")
        {
          std::cout << "EXIT_REQUEST accepted; fitting if enough cases exist, then "
                       "restoring normal control.\n" << std::flush;
          break;
        }
        if (line == "ABORT") return finish_after_failure("operator requested ABORT");
        if ((line == "NEXT" || line == "2") && interactive_refit)
        {
          if (!awaiting_next)
          {
            std::cout << "DRAG_ALREADY_ACTIVE: move to the next pose, then type RECORD.\n"
                      << std::flush;
            print_selection();
            continue;
          }
          Clock::time_point resume_received;
          std::uint64_t resume_count = 0;
          const auto resume_state = snapshot(shared_, resume_received, resume_count);
          for (std::size_t j = 0; j < 7; ++j)
            desired_[kActiveIndices[j]] =
              resume_state.motor_state().at(kActiveIndices[j]).q();
          if (!ramp_full_arm_gains(drag_kp, drag_kd, 2.0, reason, true))
            return fault_latched_ ? finish_fault(reason) : finish_after_failure(reason);
          awaiting_next = false;
          std::cout << "NEXT_ACTIVE: drag resumed; move to the next pose.\n" << std::flush;
          print_selection();
          continue;
        }
        if ((line == "RECORD" || line == "1") && interactive_refit)
        {
          if (awaiting_next)
          {
            std::cout << "RECORD_NOT_STARTED: captured pose is still being held; "
                         "type NEXT first.\n" << std::flush;
            print_selection();
            continue;
          }
          Clock::time_point record_received;
          std::uint64_t record_count = 0;
          const auto record_state = snapshot(shared_, record_received, record_count);
          for (std::size_t j = 0; j < 7; ++j)
            desired_[kActiveIndices[j]] =
              record_state.motor_state().at(kActiveIndices[j]).q();
          std::cout << "RECORD_REQUEST accepted: keep hands off; raising hold Kp to 15 "
                       "and waiting for 1.5 s stable window.\n" << std::flush;
          const Pose record_kp{15.0, 15.0, 15.0, 15.0, 15.0, 15.0, 15.0};
          const Pose record_kd{2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5};
          if (!ramp_full_arm_gains(record_kp, record_kd, 2.0, reason))
            return fault_latched_ ? finish_fault(reason) : finish_after_failure(reason);
          Observation observation;
          const bool captured = capture_stable(observation, reason);
          if (fault_latched_) return finish_fault(reason);
          if (!captured)
          {
            Clock::time_point resume_received;
            std::uint64_t resume_count = 0;
            const auto resume_state = snapshot(shared_, resume_received, resume_count);
            for (std::size_t j = 0; j < 7; ++j)
              desired_[kActiveIndices[j]] =
                resume_state.motor_state().at(kActiveIndices[j]).q();
            if (!ramp_full_arm_gains(drag_kp, drag_kd, 2.0, reason, true))
              return fault_latched_ ? finish_fault(reason) : finish_after_failure(reason);
            std::cout << "RECORD_REJECTED: " << reason
                      << "; drag remains active.\n" << std::flush;
            reason.clear();
            print_selection();
            continue;
          }
          const std::size_t id = training_.size();
          training_.push_back(observation);
          write_observation(cases, id, "drag_case", observation);
          write_temperature_observation(id, "drag_case", observation);
          cases.flush();
          const auto casing = *std::max_element(observation.casing_temperature.begin(),
            observation.casing_temperature.end());
          const auto winding = *std::max_element(observation.winding_temperature.begin(),
            observation.winding_temperature.end());
          awaiting_next = true;
          std::cout << "RECORDED case=" << id
                    << " new_cases=" << training_.size() - original_training_count
                    << " casing_max=" << casing << "C winding_max=" << winding
                    << "C; CAPTURED_POSE_HOLDING. Type NEXT (or 2) to resume drag.\n"
                    << std::flush;
          if (casing >= kCasingWarningTemperature ||
              winding >= kWindingWarningTemperature)
            std::cout << "THERMAL_WARNING: select NEXT now and move to a low-load pose; "
                         "official 85/120C hard stop remains armed.\n" << std::flush;
          print_selection();
          continue;
        }
        std::cout << "UNKNOWN_SELECTION: use "
                  << (!interactive_refit ? "STOP.\n" :
                      (awaiting_next ? "2 or 4.\n" : "1 or 4.\n")) << std::flush;
        print_selection();
      }

      Clock::time_point received;
      std::uint64_t count = 0;
      const auto current = snapshot(shared_, received, count);
      double nearest_margin = std::numeric_limits<double>::infinity();
      int max_casing = std::numeric_limits<int>::min();
      int max_winding = std::numeric_limits<int>::min();
      std::size_t max_casing_joint = kActiveIndices.front();
      std::size_t max_winding_joint = kActiveIndices.front();
      for (std::size_t j = 0; j < 7; ++j)
      {
        const auto& motor = current.motor_state().at(kActiveIndices[j]);
        const double measured = motor.q();
        if (measured < kActiveJointLower[j] || measured > kActiveJointUpper[j])
          return finish_fault("measured right joint exceeded URDF mechanical range at j" +
            std::to_string(kActiveIndices[j]));
        const double lower = kActiveJointLower[j] + kJointLimitMargin;
        const double upper = kActiveJointUpper[j] - kJointLimitMargin;
        nearest_margin = std::min(nearest_margin, std::min(measured - lower, upper - measured));
        if (!awaiting_next)
        {
          desired_[kActiveIndices[j]] = static_cast<float>(
            std::clamp(measured, lower, upper));
          // A joint-limit virtual wall only; no self-collision logic is applied.
          const double wall_zone = 5.0 * M_PI / 180.0;
          const double wall = quintic(std::clamp((wall_zone - std::min(
            measured - lower, upper - measured)) / wall_zone, 0.0, 1.0));
          full_arm_kp_[j] = 3.0 + 27.0 * wall;
          double free_kd = 2.5;
          if (j < 3)
          {
            const double speed_blend = quintic(std::clamp(
              (std::abs(static_cast<double>(motor.dq())) - 0.08) /
              (0.30 - 0.08), 0.0, 1.0));
            free_kd = 1.3 + speed_blend * (3.0 - 1.3);
          }
          full_arm_kd_[j] = free_kd + wall * (3.0 - free_kd);
        }
        const int casing = static_cast<int>(motor.temperature()[0]);
        const int winding = static_cast<int>(motor.temperature()[1]);
        if (casing > max_casing) { max_casing = casing; max_casing_joint = kActiveIndices[j]; }
        if (winding > max_winding) { max_winding = winding; max_winding_joint = kActiveIndices[j]; }
      }

      const bool thermal_warning = max_casing >= kCasingWarningTemperature ||
        max_winding >= kWindingWarningTemperature;
      if (thermal_warning && !thermal_warning_active)
      {
        std::cout << '\n';
        std::cout << "THERMAL_WARNING casing=" << max_casing << "C@j"
                  << max_casing_joint << " winding=" << max_winding << "C@j"
                  << max_winding_joint
                  << "; move to a low-load pose now. Official hard stop=85/120C.\n"
                  << std::flush;
        thermal_warning_active = true;
        print_selection();
      }
      else if (max_casing < kCasingWarningTemperature - 2.0 &&
               max_winding < kWindingWarningTemperature - 5.0)
        thermal_warning_active = false;

      std::array<float, 35> ff{};
      if (!make_feedforward(current, ff, reason)) return finish_fault(reason);
      if (!tick(reason)) return finish_fault(reason);
      const auto stamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
      const auto rpy = current_rpy(current);
      csv << stamp << ',' << rpy[0] << ',' << rpy[1] << ',' << rpy[2];
      double max_dq = 0.0;
      std::size_t max_dq_joint = kActiveIndices.front();
      for (std::size_t j = 0; j < 7; ++j)
      {
        const auto& motor = current.motor_state().at(kActiveIndices[j]);
        const double dq = std::abs(static_cast<double>(motor.dq()));
        if (dq > max_dq) { max_dq = dq; max_dq_joint = kActiveIndices[j]; }
        csv << ',' << motor.q() << ',' << desired_[kActiveIndices[j]] << ','
            << motor.dq() << ',' << motor.tau_est() << ',' << ff[kActiveIndices[j]]
            << ',' << full_arm_kp_[j] << ',' << full_arm_kd_[j] << ','
            << last_total_command_[j] << ','
            << static_cast<int>(motor.temperature()[0]) << ','
            << static_cast<int>(motor.temperature()[1]);
      }
      csv << '\n';
      if (max_dq >= kDragVelocityWarning && !velocity_warning_active)
      {
        std::cout << '\n' << std::fixed << std::setprecision(3)
                  << "VELOCITY_WARNING max_dq=" << max_dq << "@j" << max_dq_joint
                  << "; approaching the 1.5 rad/s hard limit; slow down.\n"
                  << std::flush;
        velocity_warning_active = true;
        print_selection();
      }
      else if (max_dq < kDragVelocityWarning - 0.05)
        velocity_warning_active = false;
      if (Clock::now() >= next_flush)
      {
        csv.flush();
        next_flush += 1s;
      }
      next += 20ms;
      std::this_thread::sleep_until(next);
    }
    if (g_stop_requested.load()) return finish_fault("signal requested");

    if (interactive_refit)
    {
      const std::size_t added = training_.size() - original_training_count;
      if (added >= minimum_new_cases)
      {
        double rmse = 0.0;
        double condition = 0.0;
        Eigen::Vector4d refitted_parameters{};
        if (model_.fit(training_, refitted_parameters, rmse, condition, reason))
        {
          const auto active_parameters = parameters_;
          parameters_ = refitted_parameters;
          const bool written = write_fit(rmse, condition);
          parameters_ = active_parameters;
          if (written)
          {
            const double mass = refitted_parameters(0);
            const Eigen::Vector3d com = refitted_parameters.tail<3>() / mass;
            std::cout << std::fixed << std::setprecision(5)
                      << "REFIT old_cases=" << original_training_count
                      << " new_cases=" << added << " total_cases=" << training_.size()
                      << " mass=" << mass << " kg com_wrist=[" << com.x() << ','
                      << com.y() << ',' << com.z() << "] m rmse=" << rmse
                      << " Nm condition=" << condition << '\n';
          }
          else
            std::cout << "REFIT_REJECTED: fit succeeded but output YAML could not be "
                         "written; active controller still uses the previous fit.\n";
        }
        else
          std::cout << "REFIT_REJECTED: " << reason
                    << "; previous fit file remains unchanged.\n";
      }
      else
        std::cout << "REFIT_SKIPPED: recorded " << added
                  << " new stable cases, but this mode requires at least "
                  << minimum_new_cases << ". Selected-case CSV remains saved.\n";
    }

    const Pose normal_kp{kArmKp, kArmKp, kArmKp, kArmKp, kArmKp, kArmKp, kArmKp};
    const Pose normal_kd{kArmKd, kArmKd, kArmKd, kArmKd, kArmKd, kArmKd, kArmKd};
    drag_velocity_tolerant_ = false;
    drag_velocity_exceeded_ = false;
    if (!ramp_full_arm_gains(normal_kp, normal_kd, 3.0, reason, true))
      return finish_after_failure(reason);
    full_arm_gain_override_ = false;
    if (!deactivate_fitted_feedforward(reason)) return finish_after_failure(reason);
    return finish_normal("full selected-arm drag test complete");
  }

private:
  bool preflight()
  {
    std::string reason;
    if (!check_onboard_services(reason))
    {
      std::cerr << "Service preflight failed: " << reason << '\n';
      return false;
    }
    if (!check_high_level_motion(shared_, reason, true))
    {
      std::cerr << "Motion preflight failed: " << reason << '\n';
      return false;
    }
    const auto deadline = Clock::now() + std::chrono::seconds(2);
    std::uint64_t last = 0;
    while (Clock::now() < deadline)
    {
      Clock::time_point received;
      std::uint64_t count = 0;
      const auto state = snapshot(shared_, received, count);
      const auto safety = check_state(state, received, true);
      if (!safety.safe || count == 0 || count == last)
      {
        std::cerr << "State preflight failed: "
                  << (!safety.safe ? safety.reason : "LowState not updating") << '\n';
        return false;
      }
      last = count;
      std::this_thread::sleep_for(20ms);
    }
    return true;
  }

  bool final_preflight(std::string& reason)
  {
    Clock::time_point received;
    std::uint64_t count = 0;
    const auto state = snapshot(shared_, received, count);
    const auto safety = check_state(state, received, true);
    if (!safety.safe) { reason = safety.reason; return false; }
    if (!check_onboard_services(reason)) return false;
    return check_high_level_motion(shared_, reason, true);
  }

  std::array<double, 3> current_rpy(const LowState& state) const
  {
    return {state.imu_state().rpy()[0], state.imu_state().rpy()[1],
            state.imu_state().rpy()[2]};
  }

  Pose arm_desired() const
  {
    Pose pose{};
    for (std::size_t j = 0; j < 7; ++j) pose[j] = desired_[kActiveIndices[j]];
    return pose;
  }

  bool make_feedforward(const LowState& state, std::array<float, 35>& output,
                        std::string& reason)
  {
    output = initial_tau_;
    Pose modeled{};
    // Gravity is a function of the physical configuration.  Using measured q
    // also keeps the model valid for later compliant/drag operation, where the
    // arm intentionally does not sit exactly on a stiff position target.
    Pose q{};
    for (std::size_t j = 0; j < 7; ++j)
      q[j] = state.motor_state().at(kActiveIndices[j]).q();
    const auto rpy = current_rpy(state);
    Eigen::VectorXd baseline;
    Eigen::Matrix<double, 7, 4> unused;
    if (!model_.regression(q, rpy, baseline, unused))
    {
      reason = "KDL gravity calculation failed";
      return false;
    }
    Pose anchored{};
    for (std::size_t j = 0; j < 7; ++j)
      anchored[j] = initial_tau_[kActiveIndices[j]] + baseline(j) - base_anchor_(j);
    modeled = anchored;
    if (fitted_)
    {
      Pose identified{};
      if (!model_.torque(q, rpy, parameters_, identified))
      {
        reason = "fitted KDL gravity calculation failed";
        return false;
      }
      for (std::size_t j = 0; j < 7; ++j)
        modeled[j] = anchored[j] + fitted_blend_ * (identified[j] - anchored[j]);
    }
    for (std::size_t j = 0; j < 7; ++j)
    {
      const double feedforward_limit = use_total_command_limit_
        ? kActiveTauEstLimits[j] : kActiveFeedforwardLimits[j];
      if (!std::isfinite(modeled[j]) ||
          std::abs(modeled[j]) > feedforward_limit)
      {
        reason = "selected-arm feedforward exceeds per-joint limit at motor " +
          std::to_string(kActiveIndices[j]) + " (limit=" +
          std::to_string(feedforward_limit) + " Nm)";
        return false;
      }
      output[kActiveIndices[j]] = static_cast<float>(modeled[j]);
    }
    output[kWaistYaw] = 0.0F;
    return true;
  }

  bool tick(std::string& reason, bool enforce_tracking = true,
            bool enforce_active_arm_tracking = true,
            bool active_arm_tracking_hard_only = false)
  {
    Clock::time_point received;
    std::uint64_t count = 0;
    const auto state = snapshot(shared_, received, count);
    const auto safety = check_state(state, received, true,
      drag_velocity_tolerant_ ? kDragVelocityAbsoluteLimit : kMaxVelocity);
    peak_velocity_ = std::max(peak_velocity_, safety.max_velocity);
    peak_tau_ = std::max(peak_tau_, safety.max_arm_tau);
    if (!safety.safe) { reason = safety.reason; fault_latched_ = true; return false; }
    if (drag_velocity_tolerant_)
    {
      if (safety.max_velocity > kDragVelocityNominalLimit)
      {
        if (!drag_velocity_exceeded_)
        {
          drag_velocity_exceeded_ = true;
          drag_velocity_exceeded_since_ = Clock::now();
        }
        else if (std::chrono::duration<double>(
          Clock::now() - drag_velocity_exceeded_since_).count() >=
          kDragVelocityPersistenceSeconds)
        {
          reason = "upper-body velocity remained above " +
            std::to_string(kDragVelocityNominalLimit) + " rad/s for " +
            std::to_string(kDragVelocityPersistenceSeconds) + " s";
          fault_latched_ = true;
          return false;
        }
      }
      else
        drag_velocity_exceeded_ = false;
    }
    for (std::size_t j = 0; j < 7; ++j)
    {
      const double tau = std::abs(
        static_cast<double>(state.motor_state().at(kActiveIndices[j]).tau_est()));
      if (tau > kActiveTauEstLimits[j])
      {
        reason = "tau_est exceeds per-joint limit at motor " +
          std::to_string(kActiveIndices[j]) + " (tau=" + std::to_string(tau) +
          ", limit=" + std::to_string(kActiveTauEstLimits[j]) + " Nm)";
        fault_latched_ = true;
        return false;
      }
    }
    if (!check_high_level_motion(shared_, reason)) { fault_latched_ = true; return false; }
    double tracking = 0.0;
    for (const auto index : kControlledJoints)
    {
      const double measured = state.motor_state().at(index).q();
      const double error = std::abs(static_cast<double>(desired_[index]) - measured);
      const bool active_arm_index = std::find(kActiveIndices.begin(),
        kActiveIndices.end(), index) != kActiveIndices.end();
      const bool debounce = admittance_tracking_active_ && active_arm_index;
      const double tracking_persistence =
        index == kActiveIndices[5] || index == kActiveIndices[6]
        ? kDistalTrackingPersistenceSeconds
        : kAdmittanceTrackingPersistenceSeconds;
      const double allowed = index == kWaistYaw ? waist_tracking_limit_ :
        (debounce ? kMaxTrackingError : arm_tracking_limit_);
      // During selected-arm admittance a human pull can rotate the standing
      // body's waist a few degrees even though the waist controller is still
      // holding its original target.  Do not turn that compliant body motion
      // into a nuisance latch; waist command, gains and all non-tracking
      // protections remain unchanged.
      const bool enforce_joint_tracking = enforce_tracking &&
        !(admittance_tracking_active_ && index == kWaistYaw) &&
        (enforce_active_arm_tracking || !active_arm_index);
      if (!enforce_joint_tracking || error <= allowed)
        tracking_error_exceeded_[index] = false;
      if (enforce_joint_tracking && debounce &&
          error > kAdmittanceHardTrackingError)
      {
        reason = "tracking error exceeds instantaneous hard limit at joint " +
          std::to_string(index) + " (error_deg=" +
          std::to_string(error * 180.0 / M_PI) + ", hard_limit_deg=" +
          std::to_string(kAdmittanceHardTrackingError * 180.0 / M_PI) +
          ", desired=" +
          std::to_string(desired_[index]) + ", measured=" +
          std::to_string(measured) + ")";
        fault_latched_ = true;
        return false;
      }
      if (enforce_joint_tracking && !active_arm_tracking_hard_only && error > allowed)
      {
        if (debounce && !tracking_error_exceeded_[index])
        {
          tracking_error_exceeded_[index] = true;
          tracking_error_exceeded_since_[index] = Clock::now();
        }
        else if (!debounce || std::chrono::duration<double>(Clock::now() -
                   tracking_error_exceeded_since_[index]).count() >=
                   tracking_persistence)
        {
          reason = "tracking error exceeds limit at joint " +
            std::to_string(index) + " (error_deg=" +
            std::to_string(error * 180.0 / M_PI) + ", limit_deg=" +
            std::to_string(allowed * 180.0 / M_PI) +
            (debounce ? ", persistence_s=" +
              std::to_string(tracking_persistence) : "") +
            ", desired=" + std::to_string(desired_[index]) + ", measured=" +
            std::to_string(measured) + ")";
          fault_latched_ = true;
          return false;
        }
      }
      if (error > tracking)
        tracking = error;
    }
    std::array<float, 35> feedforward{};
    if (!make_feedforward(state, feedforward, reason))
    {
      fault_latched_ = true;
      return false;
    }
    for (std::size_t j = 0; j < 7; ++j)
    {
      const auto index = kActiveIndices[j];
      const double augmented = static_cast<double>(feedforward[index]) +
        additional_tau_[j];
      if (!std::isfinite(augmented) ||
          std::abs(augmented) > kActiveTauEstLimits[j])
      {
        reason = "feedforward plus breakaway assist exceeds per-joint limit at motor " +
          std::to_string(index);
        fault_latched_ = true;
        return false;
      }
      feedforward[index] = static_cast<float>(augmented);
    }
    fill_command(command_, desired_, feedforward, 1.0F);
    for (const auto index : kControlledJoints)
      command_.motor_cmd().at(index).dq(desired_velocity_[index]);
    for (const auto index : kActiveIndices)
    {
      command_.motor_cmd().at(index).kp(static_cast<float>(arm_kp_));
      command_.motor_cmd().at(index).kd(static_cast<float>(arm_kd_));
    }
    if (full_arm_gain_override_)
      for (std::size_t j = 0; j < 7; ++j)
      {
        command_.motor_cmd().at(kActiveIndices[j]).kp(
          static_cast<float>(full_arm_kp_[j]));
        command_.motor_cmd().at(kActiveIndices[j]).kd(
          static_cast<float>(full_arm_kd_[j]));
      }
    if (elbow_gain_override_)
    {
      command_.motor_cmd().at(kActiveIndices[3]).kp(static_cast<float>(elbow_kp_));
      command_.motor_cmd().at(kActiveIndices[3]).kd(static_cast<float>(elbow_kd_));
    }
    for (std::size_t j = 0; j < 7; ++j)
    {
      const auto index = kActiveIndices[j];
      const auto& motor = state.motor_state().at(index);
      const auto& motor_command = command_.motor_cmd().at(index);
      const double total_command = feedforward[index] +
        static_cast<double>(motor_command.kp()) *
          (static_cast<double>(desired_[index]) - motor.q()) +
        static_cast<double>(motor_command.kd()) *
          (static_cast<double>(motor_command.dq()) - motor.dq());
      last_total_command_[j] = total_command;
      if (use_total_command_limit_ &&
          std::abs(total_command) > kActiveTauEstLimits[j])
      {
        reason = "estimated total command exceeds per-joint limit at motor " +
          std::to_string(index) + " (total=" + std::to_string(total_command) +
          ", limit=" + std::to_string(kActiveTauEstLimits[j]) + " Nm)";
        fault_latched_ = true;
        return false;
      }
    }
    publisher_->Write(command_);
    ++writes_;
    peak_tracking_ = std::max(peak_tracking_, tracking);
    if (fitted_) validation_peak_tracking_ = std::max(validation_peak_tracking_, tracking);
    peak_velocity_ = std::max(peak_velocity_, safety.max_velocity);
    peak_tau_ = std::max(peak_tau_, safety.max_arm_tau);
    return true;
  }

  bool hold_for(double seconds, bool takeover, std::string& reason)
  {
    const auto start = Clock::now();
    auto next = start;
    while (std::chrono::duration<double>(Clock::now() - start).count() < seconds)
    {
      const double elapsed = std::chrono::duration<double>(Clock::now() - start).count();
      if (!tick(reason, !takeover || elapsed > kTakeoverTransientSeconds)) return false;
      next += 20ms;
      std::this_thread::sleep_until(next);
    }
    return true;
  }

  bool operator_gate(const std::string& prompt, std::string& reason,
                     const std::string& expected = "")
  {
    std::cout << prompt << ": " << std::flush;
    auto next = Clock::now();
    while (!g_stop_requested.load())
    {
      pollfd descriptor{STDIN_FILENO, POLLIN, 0};
      if (::poll(&descriptor, 1, 0) > 0 && (descriptor.revents & POLLIN))
      {
        std::string line;
        std::getline(std::cin, line);
        if ((expected.empty() && line.empty()) || (!expected.empty() && line == expected))
        {
          std::cout << "accepted\n";
          return true;
        }
        reason = line == "ABORT" ? "operator requested ABORT" : "operator declined";
        return false;
      }
      if (!tick(reason)) return false;
      next += 20ms;
      std::this_thread::sleep_until(next);
    }
    reason = "signal requested";
    fault_latched_ = true;
    return false;
  }

  bool move_to(const Pose& target, std::string& reason)
  {
    const Pose from = arm_desired();
    double largest = 0.0;
    for (std::size_t j = 0; j < 7; ++j)
      largest = std::max(largest, std::abs(target[j] - from[j]));
    const double duration = std::max(kMinimumMoveSeconds,
      1.875 * largest / kPeakCommandVelocity);
    std::cout << std::fixed << std::setprecision(2)
              << "MOVE duration=" << duration << " s max_delta=" << largest << " rad\n";
    const auto start = Clock::now();
    auto next = start;
    while (true)
    {
      const double elapsed = std::chrono::duration<double>(Clock::now() - start).count();
      if (elapsed >= duration) break;
      const double blend = quintic(elapsed / duration);
      for (std::size_t j = 0; j < 7; ++j)
        desired_[kActiveIndices[j]] = static_cast<float>(from[j] + blend * (target[j] - from[j]));
      if (!tick(reason)) return false;
      next += 20ms;
      std::this_thread::sleep_until(next);
    }
    for (std::size_t j = 0; j < 7; ++j)
      desired_[kActiveIndices[j]] = static_cast<float>(target[j]);
    return hold_for(1.0, false, reason);
  }

  bool activate_fitted_feedforward(std::string& reason)
  {
    std::cout << "Blending anchored KDL feedforward to fitted payload model over 2 s.\n";
    fitted_ = true;
    fitted_blend_ = 0.0;
    const auto start = Clock::now();
    auto next = start;
    while (true)
    {
      const double elapsed = std::chrono::duration<double>(Clock::now() - start).count();
      if (elapsed >= 2.0) break;
      fitted_blend_ = quintic(elapsed / 2.0);
      if (!tick(reason)) return false;
      next += 20ms;
      std::this_thread::sleep_until(next);
    }
    fitted_blend_ = 1.0;
    return hold_for(1.0, false, reason);
  }

  bool deactivate_fitted_feedforward(std::string& reason)
  {
    if (!fitted_) return true;
    std::cout << "Blending fitted payload feedforward out over 2 s.\n";
    const auto start = Clock::now();
    auto next = start;
    while (true)
    {
      const double elapsed = std::chrono::duration<double>(Clock::now() - start).count();
      if (elapsed >= 2.0) break;
      fitted_blend_ = 1.0 - quintic(elapsed / 2.0);
      if (!tick(reason)) return false;
      next += 20ms;
      std::this_thread::sleep_until(next);
    }
    fitted_blend_ = 0.0;
    fitted_ = false;
    return hold_for(1.0, false, reason);
  }

  bool ramp_arm_gains(double target_kp, double target_kd, double seconds,
                        std::string& reason)
  {
    const double start_kp = arm_kp_;
    const double start_kd = arm_kd_;
    const auto start = Clock::now();
    auto next = start;
    while (true)
    {
      const double elapsed = std::chrono::duration<double>(Clock::now() - start).count();
      if (elapsed >= seconds) break;
      const double blend = quintic(elapsed / seconds);
      arm_kp_ = start_kp + blend * (target_kp - start_kp);
      arm_kd_ = start_kd + blend * (target_kd - start_kd);
      if (!tick(reason)) return false;
      next += 20ms;
      std::this_thread::sleep_until(next);
    }
    arm_kp_ = target_kp;
    arm_kd_ = target_kd;
    return hold_for(2.0, false, reason);
  }

  bool ramp_elbow_gains(double target_kp, double target_kd, double seconds,
                        std::string& reason)
  {
    const double start_kp = elbow_gain_override_ ? elbow_kp_ : arm_kp_;
    const double start_kd = elbow_gain_override_ ? elbow_kd_ : arm_kd_;
    elbow_gain_override_ = true;
    const auto start = Clock::now();
    auto next = start;
    while (true)
    {
      const double elapsed = std::chrono::duration<double>(Clock::now() - start).count();
      if (elapsed >= seconds) break;
      const double blend = quintic(elapsed / seconds);
      elbow_kp_ = start_kp + blend * (target_kp - start_kp);
      elbow_kd_ = start_kd + blend * (target_kd - start_kd);
      if (!tick(reason)) return false;
      next += 20ms;
      std::this_thread::sleep_until(next);
    }
    elbow_kp_ = target_kp;
    elbow_kd_ = target_kd;
    return hold_for(1.0, false, reason);
  }

  bool ramp_full_arm_gains(const Pose& target_kp, const Pose& target_kd,
                           double seconds, std::string& reason,
                           bool follow_measured = false)
  {
    Pose start_kp{}, start_kd{};
    for (std::size_t j = 0; j < 7; ++j)
    {
      start_kp[j] = full_arm_gain_override_ ? full_arm_kp_[j] : arm_kp_;
      start_kd[j] = full_arm_gain_override_ ? full_arm_kd_[j] : arm_kd_;
    }
    full_arm_gain_override_ = true;
    const auto start = Clock::now();
    auto next = start;
    while (true)
    {
      const double elapsed = std::chrono::duration<double>(Clock::now() - start).count();
      if (elapsed >= seconds) break;
      const double blend = quintic(elapsed / seconds);
      for (std::size_t j = 0; j < 7; ++j)
      {
        full_arm_kp_[j] = start_kp[j] + blend * (target_kp[j] - start_kp[j]);
        full_arm_kd_[j] = start_kd[j] + blend * (target_kd[j] - start_kd[j]);
      }
      if (follow_measured)
      {
        Clock::time_point received;
        std::uint64_t count = 0;
        const auto state = snapshot(shared_, received, count);
        for (std::size_t j = 0; j < 7; ++j)
          desired_[kActiveIndices[j]] = state.motor_state().at(kActiveIndices[j]).q();
      }
      if (!tick(reason)) return false;
      next += 20ms;
      std::this_thread::sleep_until(next);
    }
    full_arm_kp_ = target_kp;
    full_arm_kd_ = target_kd;
    return follow_measured ? true : hold_for(1.0, false, reason);
  }

  void report_validation_phase(std::size_t pose, const char* phase,
                               const Pose& target, const Observation& observation,
                               double feedback_kp = kArmKp)
  {
    double tracking_squared = 0.0;
    double tracking_max = 0.0;
    for (std::size_t j = 0; j < 7; ++j)
    {
      const double error = target[j] - observation.q[j];
      tracking_squared += error * error;
      tracking_max = std::max(tracking_max, std::abs(error));
    }
    const double tracking_rms = std::sqrt(tracking_squared / 7.0);
    const auto casing = std::max_element(observation.casing_temperature.begin(),
      observation.casing_temperature.end());
    const auto winding = std::max_element(observation.winding_temperature.begin(),
      observation.winding_temperature.end());
    std::cout << std::fixed << std::setprecision(5)
              << "VALIDATION_" << phase << " pose=" << pose
              << " tracking_rms_deg=" << tracking_rms * 180.0 / M_PI
              << " tracking_max_deg=" << tracking_max * 180.0 / M_PI
              << " equivalent_pd_rms_nm=" << feedback_kp * tracking_rms
              << " casing_max=" << *casing << "C winding_max=" << *winding << "C\n";
  }

  void report_validation_pair(std::size_t pose, const Pose& target,
                              const Observation& baseline,
                              const Observation& compensated)
  {
    double baseline_squared = 0.0;
    double fitted_squared = 0.0;
    for (std::size_t j = 0; j < 7; ++j)
    {
      const double before = target[j] - baseline.q[j];
      const double after = target[j] - compensated.q[j];
      baseline_squared += before * before;
      fitted_squared += after * after;
    }
    const double baseline_rms = std::sqrt(baseline_squared / 7.0);
    const double fitted_rms = std::sqrt(fitted_squared / 7.0);

    Pose predicted{};
    double torque_squared = 0.0;
    if (model_.torque(compensated.q, compensated.rpy, parameters_, predicted))
      for (std::size_t j = 0; j < 7; ++j)
      {
        const double error = compensated.tau[j] - predicted[j];
        torque_squared += error * error;
      }
    else
      torque_squared = std::numeric_limits<double>::quiet_NaN();
    const double improvement = baseline_rms > 1e-6
      ? 100.0 * (baseline_rms - fitted_rms) / baseline_rms
      : std::numeric_limits<double>::quiet_NaN();
    std::cout << std::fixed << std::setprecision(5)
              << "VALIDATION_COMPARE pose=" << pose
              << " tracking_rms_change_percent=" << improvement
              << " fitted_model_tau_rmse_nm=" << std::sqrt(torque_squared / 7.0)
              << '\n';
  }

  bool capture_stable(Observation& observation, std::string& reason)
  {
    struct Sample {
      Pose q;
      Pose tau;
      Pose casing_temperature;
      Pose winding_temperature;
      std::array<double, 3> rpy;
    };
    std::deque<std::pair<Clock::time_point, Sample>> window;
    const auto deadline = Clock::now() + std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(kStableTimeoutSeconds));
    auto next = Clock::now();
    double observed_max_dq = 0.0;
    double observed_max_tracking = 0.0;
    while (Clock::now() < deadline)
    {
      if (!tick(reason)) return false;
      Clock::time_point received;
      std::uint64_t count = 0;
      const auto state = snapshot(shared_, received, count);
      Sample sample;
      sample.rpy = current_rpy(state);
      bool instant_ok = true;
      for (std::size_t j = 0; j < 7; ++j)
      {
        const auto& motor = state.motor_state().at(kActiveIndices[j]);
        sample.q[j] = motor.q();
        sample.tau[j] = motor.tau_est();
        sample.casing_temperature[j] = motor.temperature()[0];
        sample.winding_temperature[j] = motor.temperature()[1];
        observed_max_dq = std::max(observed_max_dq, std::abs(static_cast<double>(motor.dq())));
        observed_max_tracking = std::max(observed_max_tracking,
          std::abs(static_cast<double>(desired_[kActiveIndices[j]]) - motor.q()));
        instant_ok = instant_ok && std::abs(motor.dq()) <= kStableMaxDq &&
          std::abs(static_cast<double>(desired_[kActiveIndices[j]]) - motor.q()) <=
          kCaptureTrackingLimit;
      }
      if (!instant_ok) window.clear();
      else window.emplace_back(Clock::now(), sample);
      while (!window.empty() && std::chrono::duration<double>(
          Clock::now() - window.front().first).count() > kStableWindowSeconds)
        window.pop_front();
      if (window.size() >= 60 && std::chrono::duration<double>(
          window.back().first - window.front().first).count() >= kStableWindowSeconds - 0.08)
      {
        bool stable = true;
        for (std::size_t j = 0; j < 7; ++j)
        {
          std::vector<double> q, tau;
          std::vector<double> casing_temperature, winding_temperature;
          for (const auto& entry : window)
          {
            q.push_back(entry.second.q[j]);
            tau.push_back(entry.second.tau[j]);
            casing_temperature.push_back(entry.second.casing_temperature[j]);
            winding_temperature.push_back(entry.second.winding_temperature[j]);
          }
          if (percentile(q, 0.9) - percentile(q, 0.1) > kStableQSpan ||
              percentile(tau, 0.9) - percentile(tau, 0.1) > kStableTauSpread)
            stable = false;
          observation.q[j] = median(std::move(q));
          observation.tau[j] = median(std::move(tau));
          observation.casing_temperature[j] = median(std::move(casing_temperature));
          observation.winding_temperature[j] = median(std::move(winding_temperature));
        }
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
          std::vector<double> values;
          for (const auto& entry : window) values.push_back(entry.second.rpy[axis]);
          observation.rpy[axis] = median(std::move(values));
        }
        if (stable)
        {
          observation.stamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
          return true;
        }
        window.clear();
      }
      next += 20ms;
      std::this_thread::sleep_until(next);
    }
    reason = "pose did not satisfy the 1.5 s static window within 12 s "
      "(observed_max_dq=" + std::to_string(observed_max_dq) +
      ", observed_max_tracking=" + std::to_string(observed_max_tracking) + ")";
    return false;
  }

  void write_csv_header(std::ofstream& out)
  {
    out << "pose_id,phase,stamp_ns";
    for (std::size_t j = 0; j < 7; ++j) out << ",q" << kActiveIndices[j];
    for (std::size_t j = 0; j < 7; ++j) out << ",tau" << kActiveIndices[j];
    out << ",imu_roll,imu_pitch,imu_yaw\n";
  }

  void write_observation(std::ofstream& out, std::size_t id, const char* phase,
                         const Observation& observation)
  {
    out << std::setprecision(12) << id << ',' << phase << ',' << observation.stamp_ns;
    for (const auto value : observation.q) out << ',' << value;
    for (const auto value : observation.tau) out << ',' << value;
    for (const auto value : observation.rpy) out << ',' << value;
    out << '\n';
  }

  void write_temperature_observation(std::size_t id, const char* phase,
                                     const Observation& observation)
  {
    const std::string path = csv_path_ + ".temperature.csv";
    const bool new_file = !std::filesystem::exists(path) ||
      std::filesystem::file_size(path) == 0;
    std::ofstream out(path, std::ios::app);
    if (!out) return;
    if (new_file)
    {
      out << "pose_id,phase,stamp_ns";
      for (const auto index : kActiveIndices) out << ",casing_j" << index;
      for (const auto index : kActiveIndices) out << ",winding_j" << index;
      out << '\n';
    }
    out << std::setprecision(12) << id << ',' << phase << ',' << observation.stamp_ns;
    for (const auto value : observation.casing_temperature) out << ',' << value;
    for (const auto value : observation.winding_temperature) out << ',' << value;
    out << '\n';
  }

  void report_capture_temperature(std::size_t pose, const Observation& observation)
  {
    const auto casing = std::max_element(
      observation.casing_temperature.begin(), observation.casing_temperature.end());
    const auto winding = std::max_element(
      observation.winding_temperature.begin(), observation.winding_temperature.end());
    const auto casing_joint = static_cast<std::size_t>(
      std::distance(observation.casing_temperature.begin(), casing));
    const auto winding_joint = static_cast<std::size_t>(
      std::distance(observation.winding_temperature.begin(), winding));
    std::cout << "CAPTURED training pose " << pose << "/11"
              << " casing_max=" << *casing << "C@j" << kActiveIndices[casing_joint]
              << " winding_max=" << *winding << "C@j" << kActiveIndices[winding_joint]
              << '\n';
  }

  bool write_fit(double rmse, double condition)
  {
    return write_payload_fit(fit_path_, parameters_, training_.size(), rmse, condition);
  }

  void report_validation(const std::vector<Observation>& observations)
  {
    double squared = 0.0;
    std::size_t count = 0;
    for (const auto& observation : observations)
    {
      Pose predicted{};
      if (!model_.torque(observation.q, observation.rpy, parameters_, predicted)) continue;
      for (std::size_t j = 0; j < 7; ++j)
      {
        const double error = observation.tau[j] - predicted[j];
        squared += error * error;
        ++count;
      }
    }
    const double rmse = count ? std::sqrt(squared / count) :
      std::numeric_limits<double>::quiet_NaN();
    std::cout << "VALIDATION repeated_pose_torque_rmse=" << rmse
              << " Nm fitted_phase_peak_tracking=" << validation_peak_tracking_
              << " rad\n";
  }

  int finish_fault(const std::string& reason)
  {
    std::cerr << "LATCHED_SILENT: " << reason << "; no further commands sent.\n";
    publisher_.reset();
    summary();
    return 3;
  }

  int finish_after_failure(const std::string& reason)
  {
    if (fault_latched_) return finish_fault(reason);
    std::string return_reason;
    if (!return_to_start(return_reason))
      return fault_latched_ ? finish_fault(return_reason) : finish_normal(return_reason);
    return finish_normal(reason);
  }

  bool return_to_start(std::string& reason)
  {
    if (resume_mode_)
    {
      if (!on_screened_sequence_) return true;
      std::cout << "Returning through pose " << resume_index_
                << " and the checked direct path to the measured resume start.\n";
      while (current_pose_index_ > resume_index_)
      {
        const auto next_pose = current_pose_index_ - 1;
        if (!move_to(candidates_[next_pose], reason)) return false;
        current_pose_index_ = next_pose;
      }
      if (!move_to(resume_start_, reason)) return false;
      on_screened_sequence_ = false;
      return true;
    }
    if (current_pose_index_ == 0) return true;
    std::cout << "Returning to pose 0 before release.\n";
    while (current_pose_index_ > 0)
    {
      const auto next_pose = current_pose_index_ - 1;
      if (!move_to(candidates_[next_pose], reason)) return false;
      current_pose_index_ = next_pose;
    }
    return true;
  }

  int finish_normal(const std::string& reason)
  {
    std::cout << "Normal exit: " << reason << "; releasing weight over 2 s.\n";
    std::string fault;
    const auto start = Clock::now();
    auto next = start;
    while (std::chrono::duration<double>(Clock::now() - start).count() < kReleaseSeconds)
    {
      Clock::time_point received;
      std::uint64_t count = 0;
      const auto state = snapshot(shared_, received, count);
      const auto safety = check_state(state, received, true);
      if (!safety.safe || !check_high_level_motion(shared_, fault))
        return finish_fault(!safety.safe ? safety.reason : fault);
      std::array<float, 35> ff{};
      if (!make_feedforward(state, ff, fault)) return finish_fault(fault);
      const double elapsed = std::chrono::duration<double>(Clock::now() - start).count();
      const float weight = static_cast<float>(1.0 - quintic(elapsed / kReleaseSeconds));
      fill_command(command_, desired_, ff, weight);
      for (const auto index : kActiveIndices)
      {
        command_.motor_cmd().at(index).kp(static_cast<float>(arm_kp_));
        command_.motor_cmd().at(index).kd(static_cast<float>(arm_kd_));
      }
      if (full_arm_gain_override_)
        for (std::size_t j = 0; j < 7; ++j)
        {
          command_.motor_cmd().at(kActiveIndices[j]).kp(
            static_cast<float>(full_arm_kp_[j]));
          command_.motor_cmd().at(kActiveIndices[j]).kd(
            static_cast<float>(full_arm_kd_[j]));
        }
      if (elbow_gain_override_)
      {
        command_.motor_cmd().at(kActiveIndices[3]).kp(static_cast<float>(elbow_kp_));
        command_.motor_cmd().at(kActiveIndices[3]).kd(static_cast<float>(elbow_kd_));
      }
      publisher_->Write(command_);
      ++writes_;
      next += 20ms;
      std::this_thread::sleep_until(next);
    }
    std::array<float, 35> ff{};
    Clock::time_point received;
    std::uint64_t count = 0;
    const auto state = snapshot(shared_, received, count);
    if (make_feedforward(state, ff, fault))
    {
      fill_command(command_, desired_, ff, 0.0F);
      for (const auto index : kActiveIndices)
      {
        command_.motor_cmd().at(index).kp(static_cast<float>(arm_kp_));
        command_.motor_cmd().at(index).kd(static_cast<float>(arm_kd_));
      }
      if (full_arm_gain_override_)
        for (std::size_t j = 0; j < 7; ++j)
        {
          command_.motor_cmd().at(kActiveIndices[j]).kp(
            static_cast<float>(full_arm_kp_[j]));
          command_.motor_cmd().at(kActiveIndices[j]).kd(
            static_cast<float>(full_arm_kd_[j]));
        }
      if (elbow_gain_override_)
      {
        command_.motor_cmd().at(kActiveIndices[3]).kp(static_cast<float>(elbow_kp_));
        command_.motor_cmd().at(kActiveIndices[3]).kd(static_cast<float>(elbow_kd_));
      }
      publisher_->Write(command_);
      ++writes_;
    }
    publisher_.reset();
    summary();
    return 0;
  }

  void summary() const
  {
    std::cout << "SUMMARY writes=" << writes_ << " training_poses=" << training_.size()
              << " peak_dq=" << peak_velocity_ << " peak_tau=" << peak_tau_
              << " peak_tracking=" << peak_tracking_ << " csv=" << csv_path_
              << " fit=" << fit_path_ << '\n';
  }

  SharedState& shared_;
  std::vector<Pose> candidates_;
  PayloadModel model_;
  std::string csv_path_;
  std::string fit_path_;
  std::unique_ptr<unitree::robot::ChannelPublisher<LowCmd>> publisher_;
  LowCmd command_;
  std::array<float, 35> initial_desired_{};
  std::array<float, 35> desired_{};
  std::array<float, 35> desired_velocity_{};
  std::array<float, 35> initial_tau_{};
  Pose initial_arm_{};
  std::array<double, 3> initial_rpy_{};
  Eigen::VectorXd base_anchor_;
  Eigen::Vector4d parameters_{};
  std::vector<Observation> training_;
  bool fitted_{false};
  double fitted_blend_{0.0};
  bool fault_latched_{false};
  std::size_t current_pose_index_{0};
  Pose resume_start_{};
  std::size_t resume_index_{0};
  bool resume_mode_{false};
  bool on_screened_sequence_{true};
  std::uint64_t writes_{0};
  double peak_tracking_{0};
  double validation_peak_tracking_{0};
  double peak_velocity_{0};
  double peak_tau_{0};
  double arm_kp_{kArmKp};
  double arm_kd_{kArmKd};
  bool elbow_gain_override_{false};
  double elbow_kp_{kArmKp};
  double elbow_kd_{kArmKd};
  bool full_arm_gain_override_{false};
  Pose full_arm_kp_{kArmKp, kArmKp, kArmKp, kArmKp, kArmKp, kArmKp, kArmKp};
  Pose full_arm_kd_{kArmKd, kArmKd, kArmKd, kArmKd, kArmKd, kArmKd, kArmKd};
  double waist_tracking_limit_{kMaxTrackingError};
  double arm_tracking_limit_{kMaxTrackingError};
  bool admittance_tracking_active_{false};
  std::array<bool, 35> tracking_error_exceeded_{};
  std::array<Clock::time_point, 35> tracking_error_exceeded_since_{};
  Pose additional_tau_{};
  bool use_total_command_limit_{false};
  Pose last_total_command_{};
  bool drag_velocity_tolerant_{false};
  bool drag_velocity_exceeded_{false};
  Clock::time_point drag_velocity_exceeded_since_{};
};
}  // namespace calibration

int main(int argc, char** argv)
{
  const auto print_array = [](const auto& values) {
    std::cout << '[';
    for (std::size_t i = 0; i < values.size(); ++i)
      std::cout << (i ? "," : "") << values[i];
    std::cout << ']';
  };
  if (argc == 2 && std::string(argv[1]) == "--describe")
  {
    std::cout << std::setprecision(12) << "{\"arm\":\"" << calibration::kActiveArm
              << "\",\"payload_frame\":\"" << calibration::kActiveWristLink << "\",\"indices\":";
    print_array(calibration::kActiveIndices);
    std::cout << ",\"lower\":"; print_array(calibration::kActiveJointLower);
    std::cout << ",\"upper\":"; print_array(calibration::kActiveJointUpper);
    std::cout << ",\"admittance_kp\":"; print_array(calibration::kFullAdmittanceKp);
    std::cout << ",\"admittance_kd\":"; print_array(calibration::kFullAdmittanceKd);
    std::cout << ",\"max_velocity\":"; print_array(calibration::kFullAdmittanceMaxVelocity);
    std::cout << ",\"max_acceleration\":"; print_array(calibration::kFullAdmittanceMaxAcceleration);
    std::cout << ",\"deadband\":"; print_array(calibration::kFullAdmittanceDeadband);
    std::cout << ",\"torque_limits\":"; print_array(calibration::kActiveTauEstLimits);
    std::cout << ",\"feedforward_limits\":"; print_array(calibration::kActiveFeedforwardLimits);
    std::cout << ",\"creates_dds\":false}\n";
    return 0;
  }
  if (argc == 10 && std::string(argv[1]) == "--check-pose")
  {
    calibration::PayloadModel model;
    Eigen::Vector4d parameters{};
    calibration::Pose q{}, torque{};
    std::string reason;
    try {
      for (std::size_t i = 0; i < q.size(); ++i) {
        std::size_t parsed = 0;
        const std::string value(argv[3 + i]);
        q[i] = std::stod(value, &parsed);
        if (parsed != value.size() || !std::isfinite(q[i]) ||
            q[i] < calibration::kActiveJointLower[i] || q[i] > calibration::kActiveJointUpper[i])
          throw std::runtime_error("invalid/out-of-range joint");
      }
      if (!calibration::load_fit(argv[2], parameters, reason) ||
          !model.initialize("/home/syf/unitree_ws/unitree_ros/robots/g1_description/"
                            "g1_29dof_lock_waist_rev_1_0.urdf", reason) ||
          !model.torque(q, {0., 0., 0.}, parameters, torque))
        throw std::runtime_error(reason.empty() ? "gravity calculation failed" : reason);
      std::cout << std::setprecision(12) << "{\"arm\":\"" << calibration::kActiveArm
                << "\",\"zero_rpy_gravity_nm\":";
      print_array(torque);
      std::cout << ",\"creates_dds\":false}\n";
      return 0;
    } catch (const std::exception& error) {
      std::cerr << "OFFLINE CHECK FAILED: " << error.what() << '\n';
      return 1;
    }
  }
  if (argc == 4 && std::string(argv[1]) == "--fit-selected-cases")
  {
    std::vector<calibration::Observation> observations;
    Eigen::Vector4d parameters{};
    calibration::PayloadModel model;
    std::string reason;
    double rmse = 0.0, condition = 0.0;
    if (!calibration::load_training_csv(
          argv[2], 0, observations, reason, "drag_case") ||
        !model.initialize(
          "/home/syf/unitree_ws/unitree_ros/robots/g1_description/"
          "g1_29dof_lock_waist_rev_1_0.urdf", reason) ||
        !model.fit(observations, parameters, rmse, condition, reason) ||
        !calibration::write_payload_fit(
          argv[3], parameters, observations.size(), rmse, condition))
    {
      std::cerr << "SELECTED-CASE FIT FAILED: "
                << (reason.empty() ? "cannot write output YAML" : reason) << '\n';
      return 1;
    }
    const double mass = parameters(0);
    const Eigen::Vector3d com = parameters.tail<3>() / mass;
    std::cout << std::fixed << std::setprecision(6)
              << "SELECTED-CASE FIT OK: poses=" << observations.size()
              << " mass=" << mass << " kg com_wrist=[" << com.x() << ','
              << com.y() << ',' << com.z() << "] m rmse=" << rmse
              << " Nm condition=" << condition
              << "; no DDS participant or publisher was created.\n";
    return 0;
  }
  if (argc == 4 && std::string(argv[1]) == "--check-drag-refit")
  {
    std::vector<calibration::Observation> observations;
    Eigen::Vector4d active_parameters{}, recovered{};
    calibration::PayloadModel model;
    std::string reason;
    double rmse = 0.0, condition = 0.0;
    if (!calibration::load_training_csv(argv[2], 12, observations, reason) ||
        !calibration::load_fit(argv[3], active_parameters, reason) ||
        !model.initialize(
          "/home/syf/unitree_ws/unitree_ros/robots/g1_description/"
          "g1_29dof_lock_waist_rev_1_0.urdf", reason) ||
        !model.fit(observations, recovered, rmse, condition, reason))
    {
      std::cerr << "DRAG-REFIT CHECK FAILED: " << reason << '\n';
      return 1;
    }
    std::cout << "DRAG-REFIT CHECK OK: 12 existing cases parsed and refitted; rmse="
              << rmse << " Nm condition=" << condition
              << "; no DDS participant or publisher was created.\n";
    return 0;
  }
  if (argc == 4 && std::string(argv[1]) == "--check-validation")
  {
    std::vector<calibration::Pose> poses;
    Eigen::Vector4d parameters{};
    calibration::PayloadModel model;
    std::string reason;
    if (!calibration::load_candidates(argv[2], poses, reason, 2) ||
        !calibration::load_fit(argv[3], parameters, reason) ||
        !model.initialize(
          "/home/syf/unitree_ws/unitree_ros/robots/g1_description/"
          "g1_29dof_lock_waist_rev_1_0.urdf", reason))
    {
      std::cerr << "VALIDATION CHECK FAILED: " << reason << '\n';
      return 1;
    }
    double peak = 0.0;
    for (const auto& pose : poses)
    {
      calibration::Pose torque{};
      if (!model.torque(pose, {0.0, 0.0, 0.0}, parameters, torque))
      {
        std::cerr << "VALIDATION CHECK FAILED: KDL torque calculation failed\n";
        return 1;
      }
      for (const auto value : torque) peak = std::max(peak, std::abs(value));
    }
    if (peak > calibration::kMaxActiveFeedforward)
    {
      std::cerr << "VALIDATION CHECK FAILED: predicted feedforward peak=" << peak
                << " Nm exceeds +/-8 Nm\n";
      return 1;
    }
    std::cout << "VALIDATION CHECK OK: two poses and fit parsed; predicted zero-RPY "
              << "feedforward peak=" << peak
              << " Nm; no DDS participant or publisher was created.\n";
    return 0;
  }
  if (argc == 3 && std::string(argv[1]) == "--check")
  {
    std::vector<calibration::Pose> checked_candidates;
    std::string check_reason;
    calibration::PayloadModel checked_model;
    if (!calibration::load_candidates(argv[2], checked_candidates, check_reason) ||
        !checked_model.initialize(
          "/home/syf/unitree_ws/unitree_ros/robots/g1_description/"
          "g1_29dof_lock_waist_rev_1_0.urdf", check_reason))
    {
      std::cerr << "CHECK FAILED: " << check_reason << '\n';
      return 1;
    }
    const Eigen::Vector4d known_parameters(1.8, 0.18, -0.054, 0.072);
    std::vector<calibration::Observation> synthetic;
    for (const auto& pose : checked_candidates)
    {
      calibration::Observation observation;
      observation.q = pose;
      observation.rpy = {0.012, 0.023, -0.06};
      if (!checked_model.torque(
          observation.q, observation.rpy, known_parameters, observation.tau))
      {
        std::cerr << "CHECK FAILED: synthetic torque calculation failed\n";
        return 1;
      }
      synthetic.push_back(observation);
    }
    Eigen::Vector4d recovered;
    double synthetic_rmse = 0.0;
    double synthetic_condition = 0.0;
    if (!checked_model.fit(synthetic, recovered, synthetic_rmse,
                           synthetic_condition, check_reason) ||
        (recovered - known_parameters).norm() > 1e-8 || synthetic_rmse > 1e-8)
    {
      std::cerr << "CHECK FAILED: synthetic least-squares recovery failed: "
                << check_reason << '\n';
      return 1;
    }
    std::cout << "CHECK OK: 12 poses parsed; 7-DoF KDL chain loaded; "
              << "synthetic LS recovered mass/COM (condition="
              << synthetic_condition << "); "
              << "no DDS participant or command publisher was created.\n";
    return 0;
  }
  const bool resume_requested = argc == 9 && std::string(argv[2]) == "--resume" &&
    std::string(argv[3]) == calibration::kCalibrationAuthorization;
  const bool execute_requested = argc == 7 && std::string(argv[2]) == "--execute" &&
    std::string(argv[3]) == calibration::kCalibrationAuthorization;
  const bool validate_requested = argc == 7 && std::string(argv[2]) == "--validate" &&
    std::string(argv[3]) == calibration::kCalibrationAuthorization;
  const bool compliance_requested = argc == 6 &&
    std::string(argv[2]) == "--compliance-hold" &&
    std::string(argv[3]) == calibration::kCalibrationAuthorization;
  const bool elbow_drag_requested = argc == 7 &&
    std::string(argv[2]) == "--elbow-drag" &&
    std::string(argv[3]) == calibration::kCalibrationAuthorization;
  const bool elbow_admittance_requested = argc == 6 &&
    std::string(argv[2]) == "--elbow-admittance" &&
    std::string(argv[3]) == calibration::kCalibrationAuthorization;
  const bool full_arm_admittance_requested = argc == 6 &&
    std::string(argv[2]) == "--full-arm-admittance" &&
    std::string(argv[3]) == calibration::kCalibrationAuthorization;
  const bool remote_admittance_requested = argc == 6 &&
    std::string(argv[2]) == "--remote-admittance" &&
    std::string(argv[3]) == calibration::kCalibrationAuthorization;
  const bool full_arm_drag_requested = argc == 6 &&
    std::string(argv[2]) == "--full-arm-drag" &&
    std::string(argv[3]) == calibration::kCalibrationAuthorization;
  const bool drag_refit_requested = argc == 8 &&
    std::string(argv[2]) == "--drag-refit" &&
    std::string(argv[3]) == calibration::kCalibrationAuthorization;
  const bool drag_recalibrate_requested = argc == 7 &&
    std::string(argv[2]) == "--drag-recalibrate" &&
    std::string(argv[3]) == calibration::kCalibrationAuthorization;
#ifdef G1_CALIBRATION_LEFT_ARM
  if (!remote_admittance_requested)
  {
    std::cerr << "Left-arm commissioning allows only --remote-admittance. "
                 "Scripted calibration/drag/trajectory modes remain disabled.\n";
    return 1;
  }
#endif
  if (!execute_requested && !resume_requested && !validate_requested &&
      !compliance_requested && !elbow_drag_requested &&
      !elbow_admittance_requested && !full_arm_admittance_requested &&
      !remote_admittance_requested &&
      !full_arm_drag_requested &&
      !drag_refit_requested && !drag_recalibrate_requested)
  {
    std::cerr << "Usage:\n  " << argv[0] << " --check CANDIDATES_YAML\n  "
              << argv[0]
              << " NETWORK_INTERFACE --execute "
              << calibration::kCalibrationAuthorization
              << " CANDIDATES_YAML OUTPUT_CSV OUTPUT_FIT_YAML\n  "
              << argv[0] << " NETWORK_INTERFACE --resume "
              << calibration::kCalibrationAuthorization
              << " CANDIDATES_YAML EXISTING_CSV OUTPUT_FIT_YAML RESUME_PATH_YAML 8\n  "
              << argv[0] << " NETWORK_INTERFACE --validate "
              << calibration::kCalibrationAuthorization
              << " VALIDATION_POSES_YAML FIT_YAML OUTPUT_VALIDATION_CSV\n  "
              << argv[0] << " NETWORK_INTERFACE --compliance-hold "
              << calibration::kCalibrationAuthorization
              << " FIT_YAML OUTPUT_COMPLIANCE_CSV\n  "
              << argv[0] << " NETWORK_INTERFACE --elbow-drag "
              << calibration::kCalibrationAuthorization
              << " FIT_YAML CHECKED_RANGE_YAML OUTPUT_DRAG_CSV\n  "
              << argv[0] << " NETWORK_INTERFACE --elbow-admittance "
              << calibration::kCalibrationAuthorization
              << " FIT_YAML OUTPUT_ADMITTANCE_CSV\n  "
              << argv[0] << " NETWORK_INTERFACE --full-arm-admittance "
              << calibration::kCalibrationAuthorization
              << " FIT_YAML OUTPUT_ADMITTANCE_CSV\n  "
              << argv[0] << " NETWORK_INTERFACE --remote-admittance "
              << calibration::kCalibrationAuthorization
              << " FIT_YAML OUTPUT_SESSION_PREFIX_CSV\n  "
              << argv[0] << " NETWORK_INTERFACE --full-arm-drag "
              << calibration::kCalibrationAuthorization
              << " FIT_YAML OUTPUT_FULL_ARM_CSV\n  "
              << argv[0] << " NETWORK_INTERFACE --drag-refit "
              << calibration::kCalibrationAuthorization
              << " ACTIVE_FIT_YAML EXISTING_12_POSE_CSV OUTPUT_DRAG_CSV "
                 "OUTPUT_REFIT_YAML\n  "
              << argv[0] << " NETWORK_INTERFACE --drag-recalibrate "
              << calibration::kCalibrationAuthorization
              << " ACTIVE_FIT_YAML OUTPUT_DRAG_CSV OUTPUT_NEW_FIT_YAML\n";
    return 1;
  }
  std::vector<calibration::Pose> candidates;
  std::string reason;
  if (!compliance_requested && !elbow_drag_requested &&
      !elbow_admittance_requested && !full_arm_admittance_requested &&
      !remote_admittance_requested &&
      !full_arm_drag_requested &&
      !drag_refit_requested && !drag_recalibrate_requested &&
      !calibration::load_candidates(
      argv[4], candidates, reason, validate_requested ? 2 : 12))
  {
    std::cerr << reason << '\n';
    return 1;
  }
  calibration::PayloadModel model;
  if (!model.initialize(
      "/home/syf/unitree_ws/unitree_ros/robots/g1_description/"
      "g1_29dof_lock_waist_rev_1_0.urdf", reason))
  {
    std::cerr << reason << '\n';
    return 1;
  }

  std::vector<calibration::Observation> existing;
  calibration::Pose resume_start{};
  std::size_t resume_index = 0;
  Eigen::Vector4d validation_parameters{};
  if (resume_requested)
  {
    try { resume_index = static_cast<std::size_t>(std::stoul(argv[8])); }
    catch (const std::exception&) { std::cerr << "Invalid resume index\n"; return 1; }
    if (resume_index == 0 || resume_index >= candidates.size() ||
        !calibration::load_training_csv(argv[5], resume_index, existing, reason) ||
        !calibration::load_named_pose(argv[7], "start_q", resume_start, reason))
    {
      std::cerr << "Resume setup failed: " << reason << '\n';
      return 1;
    }
  }
  if (drag_refit_requested &&
      !calibration::load_training_csv(argv[5], 12, existing, reason))
  {
    std::cerr << "Drag-refit setup failed: " << reason << '\n';
    return 1;
  }
  if ((validate_requested || compliance_requested || elbow_drag_requested ||
       elbow_admittance_requested || full_arm_admittance_requested ||
       remote_admittance_requested ||
       full_arm_drag_requested ||
       drag_refit_requested || drag_recalibrate_requested) &&
      !calibration::load_fit(
        validate_requested ? argv[5] : argv[4], validation_parameters, reason))
  {
    std::cerr << "Validation setup failed: " << reason << '\n';
    return 1;
  }

  std::signal(SIGINT, signal_handler);
  std::cout << "SELECTED_ARM=" << calibration::kActiveArm << " wrist="
            << calibration::kActiveWristLink << "; passive arm stays at captured pose.\n"
            << std::flush;
  std::signal(SIGTERM, signal_handler);
  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
  SharedState shared;
  auto state_subscriber = std::make_unique<unitree::robot::ChannelSubscriber<LowState>>(kStateTopic);
  state_subscriber->InitChannel([&shared](const void* data) {
    std::lock_guard<std::mutex> lock(shared.mutex);
    shared.message = *static_cast<const LowState*>(data);
    shared.received_at = Clock::now();
    ++shared.samples;
  }, 1);
  auto sport_subscriber =
    std::make_unique<unitree::robot::ChannelSubscriber<SportModeState>>(kSportStateTopic);
  sport_subscriber->InitChannel([&shared](const void* data) {
    std::lock_guard<std::mutex> lock(shared.mutex);
    shared.sport_message = *static_cast<const SportModeState*>(data);
    shared.sport_received_at = Clock::now();
    ++shared.sport_samples;
  }, 1);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  if (validate_requested)
  {
    calibration::CalibrationRun run(shared, std::move(candidates), std::move(model),
      argv[6], argv[5]);
    return run.execute_validation(validation_parameters);
  }
  if (compliance_requested)
  {
    calibration::CalibrationRun run(shared, {}, std::move(model), argv[5], argv[4]);
    return run.execute_compliance_hold(validation_parameters);
  }
  if (elbow_drag_requested)
  {
    calibration::Pose start{}, lower{}, upper{};
    if (!calibration::load_named_pose(argv[5], "start_q", start, reason) ||
        !calibration::load_named_pose(argv[5], "lower_q", lower, reason) ||
        !calibration::load_named_pose(argv[5], "upper_q", upper, reason))
    {
      std::cerr << "Elbow drag setup failed: " << reason << '\n';
      return 1;
    }
    std::vector<calibration::Pose> range{start, lower, upper};
    calibration::CalibrationRun run(shared, std::move(range), std::move(model),
      argv[6], argv[4]);
    return run.execute_elbow_drag(validation_parameters);
  }
  if (elbow_admittance_requested)
  {
    calibration::CalibrationRun run(shared, {}, std::move(model), argv[5], argv[4]);
    return run.execute_elbow_admittance(validation_parameters);
  }
  if (full_arm_admittance_requested)
  {
    calibration::CalibrationRun run(shared, {}, std::move(model), argv[5], argv[4]);
    return run.execute_full_arm_admittance(validation_parameters);
  }
  if (remote_admittance_requested)
  {
    calibration::CalibrationRun run(shared, {}, std::move(model), argv[5], argv[4]);
    return run.execute_remote_admittance(validation_parameters);
  }
  if (full_arm_drag_requested)
  {
    calibration::CalibrationRun run(shared, {}, std::move(model), argv[5], argv[4]);
    return run.execute_full_arm_drag(validation_parameters);
  }
  if (drag_refit_requested)
  {
    calibration::CalibrationRun run(shared, {}, std::move(model), argv[6], argv[7],
      std::move(existing));
    return run.execute_full_arm_drag(validation_parameters, true);
  }
  if (drag_recalibrate_requested)
  {
    calibration::CalibrationRun run(shared, {}, std::move(model), argv[5], argv[6]);
    return run.execute_full_arm_drag(validation_parameters, true, 6);
  }
  calibration::CalibrationRun run(shared, std::move(candidates), std::move(model),
    argv[5], argv[6], std::move(existing), resume_start, resume_index);
  return run.execute();
}
