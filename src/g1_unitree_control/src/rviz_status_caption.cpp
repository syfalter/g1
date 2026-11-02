#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

using namespace std::chrono_literals;

namespace
{
std::string value_of(
  const diagnostic_msgs::msg::DiagnosticStatus & status, const std::string & key)
{
  const auto it = std::find_if(
    status.values.begin(), status.values.end(),
    [&key](const auto & value) {return value.key == key;});
  return it == status.values.end() ? std::string{} : it->value;
}

double number_of(
  const diagnostic_msgs::msg::DiagnosticStatus & status, const std::string & key,
  double fallback = 0.0)
{
  try {
    const auto value = value_of(status, key);
    return value.empty() ? fallback : std::stod(value);
  } catch (const std::exception &) {
    return fallback;
  }
}

std::string one_decimal(double value)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(1) << value;
  return stream.str();
}
}  // namespace

class RvizStatusCaption : public rclcpp::Node
{
public:
  RvizStatusCaption()
  : Node("g1_rviz_status_caption")
  {
    diagnostics_topic_ = declare_parameter<std::string>(
      "diagnostics_topic", "/g1/arm_gateway/diagnostics");
    display_trajectory_topic_ = declare_parameter<std::string>(
      "display_trajectory_topic", "/display_planned_path");
    marker_topic_ = declare_parameter<std::string>(
      "marker_topic", "/g1/rviz/status_caption");
    frame_id_ = declare_parameter<std::string>("frame_id", "head_link");
    height_above_head_m_ = declare_parameter<double>("height_above_head_m", 0.32);
    text_height_m_ = declare_parameter<double>("text_height_m", 0.075);
    diagnostics_stale_s_ = declare_parameter<double>("diagnostics_stale_s", 2.0);
    arrived_caption_s_ = declare_parameter<double>("arrived_caption_s", 4.0);

    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      marker_topic_, rclcpp::QoS(1).reliable().transient_local());
    diagnostics_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_, rclcpp::QoS(5).reliable().durability_volatile(),
      std::bind(&RvizStatusCaption::diagnostics_callback, this, std::placeholders::_1));
    trajectory_sub_ = create_subscription<moveit_msgs::msg::DisplayTrajectory>(
      display_trajectory_topic_, rclcpp::QoS(5).reliable().durability_volatile(),
      std::bind(&RvizStatusCaption::trajectory_callback, this, std::placeholders::_1));
    timer_ = create_wall_timer(200ms, std::bind(&RvizStatusCaption::publish_caption, this));
  }

private:
  void diagnostics_callback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr message)
  {
    if (message->status.empty()) {
      return;
    }
    const auto & status = message->status.front();
    const std::string previous_phase = phase_;
    phase_ = value_of(status, "phase");
    diagnostic_message_ = status.message;
    diagnostic_level_ = status.level;
    armed_ = value_of(status, "armed") == "true";
    temperature_c_ = number_of(status, "max_motor_temperature_c");
    tracking_error_deg_ = number_of(status, "tracking_error_rad") * 180.0 / M_PI;
    accepted_trajectories_ = static_cast<int>(
      number_of(status, "accepted_trajectories"));
    rejected_trajectories_ = static_cast<int>(
      number_of(status, "rejected_trajectories"));
    last_rejection_ = value_of(status, "last_rejection");
    last_diagnostics_time_ = now();
    have_diagnostics_ = true;

    if (phase_ == "RUNNING") {
      plan_ready_ = false;
    } else if (previous_phase == "RUNNING" && phase_ == "ARMED_HOLD") {
      plan_ready_ = false;
      arrived_until_ = now() + rclcpp::Duration::from_seconds(arrived_caption_s_);
    }
    if (phase_ == "LATCHED_SILENT" || !armed_) {
      plan_ready_ = false;
    }
  }

  void trajectory_callback(const moveit_msgs::msg::DisplayTrajectory::SharedPtr message)
  {
    std::size_t point_count = 0;
    for (const auto & trajectory : message->trajectory) {
      point_count += trajectory.joint_trajectory.points.size();
    }
    if (point_count == 0) {
      return;
    }
    planned_points_ = point_count;
    plan_ready_ = true;
    arrived_until_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  }

  void set_color(visualization_msgs::msg::Marker & marker, float r, float g, float b) const
  {
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0F;
  }

  std::string telemetry_line() const
  {
    return "Temp " + one_decimal(temperature_c_) + " C | Track " +
           one_decimal(tracking_error_deg_) + " deg";
  }

  void publish_caption()
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = frame_id_;
    marker.ns = "g1_endpoint_status";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.z = height_above_head_m_;
    marker.pose.orientation.w = 1.0;
    marker.scale.z = text_height_m_;
    marker.frame_locked = true;
    marker.lifetime = rclcpp::Duration::from_seconds(0.6);

    const bool stale = !have_diagnostics_ ||
      (now() - last_diagnostics_time_).seconds() > diagnostics_stale_s_;
    if (stale) {
      marker.text = "GATEWAY STATUS STALE\nNo execution allowed";
      set_color(marker, 1.0F, 0.25F, 0.1F);
    } else if (diagnostic_level_ == diagnostic_msgs::msg::DiagnosticStatus::ERROR ||
      phase_ == "LATCHED_SILENT")
    {
      marker.text = "SAFETY STOP\n" + diagnostic_message_ + "\n/arm_sdk output is silent";
      set_color(marker, 1.0F, 0.05F, 0.05F);
    } else if (phase_ == "RUNNING") {
      marker.text = "EXECUTING\n" + telemetry_line() + "\nL2+B: emergency damping";
      set_color(marker, 0.1F, 0.75F, 1.0F);
    } else if (phase_ == "TAU_ANCHORED_HOLD" || phase_ == "GRAVITY_FEEDFORWARD_BLEND") {
      marker.text = "TAKING CONTROL - KEEP CLEAR\n" + phase_ + "\n" + telemetry_line();
      set_color(marker, 1.0F, 0.7F, 0.05F);
    } else if (phase_ == "ARMED_HOLD") {
      if (plan_ready_) {
        marker.text = "PLAN READY - CLICK EXECUTE\n" +
          std::to_string(planned_points_) + " trajectory points | " + telemetry_line();
        set_color(marker, 0.2F, 0.8F, 1.0F);
      } else if (arrived_until_.nanoseconds() > 0 && now() < arrived_until_) {
        marker.text = "ARRIVED - HOLDING\nReady for the next Plan\n" + telemetry_line();
        set_color(marker, 0.1F, 1.0F, 0.25F);
      } else {
        marker.text = "HOLD - READY TO PLAN\n" + telemetry_line() +
          "\nAccepted " + std::to_string(accepted_trajectories_);
        set_color(marker, 0.2F, 1.0F, 0.35F);
      }
    } else if (!armed_) {
      marker.text = "UNARMED\nType ARM in the launch terminal\n" + diagnostic_message_;
      set_color(marker, 1.0F, 0.65F, 0.05F);
    } else {
      marker.text = phase_ + "\n" + diagnostic_message_ + "\n" + telemetry_line();
      set_color(marker, 1.0F, 0.75F, 0.1F);
    }

    if (rejected_trajectories_ > 0 && !last_rejection_.empty() &&
      phase_ == "ARMED_HOLD")
    {
      marker.text += "\nLast reject: " + last_rejection_;
    }
    marker_pub_->publish(marker);
  }

  std::string diagnostics_topic_;
  std::string display_trajectory_topic_;
  std::string marker_topic_;
  std::string frame_id_;
  double height_above_head_m_{};
  double text_height_m_{};
  double diagnostics_stale_s_{};
  double arrived_caption_s_{};

  bool have_diagnostics_{false};
  bool armed_{false};
  bool plan_ready_{false};
  std::string phase_{"WAITING"};
  std::string diagnostic_message_;
  std::string last_rejection_;
  uint8_t diagnostic_level_{diagnostic_msgs::msg::DiagnosticStatus::WARN};
  double temperature_c_{};
  double tracking_error_deg_{};
  int accepted_trajectories_{};
  int rejected_trajectories_{};
  std::size_t planned_points_{};
  rclcpp::Time last_diagnostics_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time arrived_until_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_sub_;
  rclcpp::Subscription<moveit_msgs::msg::DisplayTrajectory>::SharedPtr trajectory_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RvizStatusCaption>());
  rclcpp::shutdown();
  return 0;
}
