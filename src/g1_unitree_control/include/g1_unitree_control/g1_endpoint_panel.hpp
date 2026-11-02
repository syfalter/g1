#ifndef G1_UNITREE_CONTROL__G1_ENDPOINT_PANEL_HPP_
#define G1_UNITREE_CONTROL__G1_ENDPOINT_PANEL_HPP_

#include <chrono>
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <rviz_common/panel.hpp>
#include <std_srvs/srv/trigger.hpp>

class QLabel;
class QDoubleSpinBox;
class QPushButton;
class QTimer;
class QWidget;

namespace g1_unitree_control
{

class G1EndpointPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit G1EndpointPanel(QWidget * parent = nullptr);
  void onInitialize() override;

private Q_SLOTS:
  void refresh_ui();
  void request_arm();
  void request_disarm();
  void request_reset();
  void set_startup_pose_goal();
  void request_auto_plan();
  void request_auto_execute();
  void auto_scaling_changed(double value);

private:
  void diagnostics_callback(
    const diagnostic_msgs::msg::DiagnosticArray::SharedPtr message);
  void trajectory_callback(
    const moveit_msgs::msg::DisplayTrajectory::SharedPtr message);
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr message);
  void request_gateway(
    const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client,
    const std::string & pending_text);
  void request_auto(bool execute);

  std::mutex mutex_;
  bool have_diagnostics_{false};
  bool armed_{false};
  bool upper_body_state_safe_{false};
  bool service_state_ready_{false};
  bool service_conflict_{false};
  bool temperature_alarm_active_{false};
  bool plan_ready_{false};
  bool service_call_pending_{false};
  bool startup_pose_captured_{false};
  bool auto_plan_ready_{false};
  std::string phase_{"WAITING"};
  std::string diagnostic_message_;
  std::string service_feedback_;
  std::string last_rejection_;
  std::uint8_t diagnostic_level_{1};
  double temperature_c_{0.0};
  double temperature_hard_limit_c_{85.0};
  double tracking_error_deg_{0.0};
  int accepted_trajectories_{0};
  int rejected_trajectories_{0};
  std::size_t planned_points_{0};
  std::chrono::steady_clock::time_point last_diagnostics_time_{};
  std::chrono::steady_clock::time_point arrived_until_{};
  sensor_msgs::msg::JointState startup_joint_state_;

  QLabel * status_label_{nullptr};
  QLabel * detail_label_{nullptr};
  QLabel * temperature_value_{nullptr};
  QLabel * tracking_value_{nullptr};
  QLabel * trajectory_value_{nullptr};
  QLabel * feedback_label_{nullptr};
  QPushButton * arm_button_{nullptr};
  QPushButton * disarm_button_{nullptr};
  QPushButton * reset_button_{nullptr};
  QPushButton * startup_pose_button_{nullptr};
  QPushButton * auto_plan_button_{nullptr};
  QPushButton * auto_execute_button_{nullptr};
  QDoubleSpinBox * auto_velocity_scaling_{nullptr};
  QDoubleSpinBox * auto_acceleration_scaling_{nullptr};
  QTimer * refresh_timer_{nullptr};

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_sub_;
  rclcpp::Subscription<moveit_msgs::msg::DisplayTrajectory>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<moveit_msgs::msg::RobotState>::SharedPtr startup_goal_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr auto_velocity_scaling_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr auto_acceleration_scaling_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr arm_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr disarm_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr auto_plan_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr auto_execute_client_;
};

}  // namespace g1_unitree_control

#endif  // G1_UNITREE_CONTROL__G1_ENDPOINT_PANEL_HPP_
