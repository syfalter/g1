#include "g1_unitree_control/g1_endpoint_panel.hpp"

#include <algorithm>
#include <cmath>
#include <future>
#include <utility>

#include <QGridLayout>
#include <QHBoxLayout>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QStyle>
#include <QTimer>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>

namespace
{

constexpr std::array<const char *, 14> kArmJointNames{{
  "left_shoulder_pitch_joint",
  "left_shoulder_roll_joint",
  "left_shoulder_yaw_joint",
  "left_elbow_joint",
  "left_wrist_roll_joint",
  "left_wrist_pitch_joint",
  "left_wrist_yaw_joint",
  "right_shoulder_pitch_joint",
  "right_shoulder_roll_joint",
  "right_shoulder_yaw_joint",
  "right_elbow_joint",
  "right_wrist_roll_joint",
  "right_wrist_pitch_joint",
  "right_wrist_yaw_joint",
}};

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

QString one_decimal(double value)
{
  return QString::number(value, 'f', 1);
}

}  // namespace

namespace g1_unitree_control
{

G1EndpointPanel::G1EndpointPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  setMinimumWidth(285);

  auto * layout = new QVBoxLayout(this);
  layout->setContentsMargins(10, 10, 10, 10);
  layout->setSpacing(8);

  status_label_ = new QLabel("WAITING", this);
  status_label_->setAlignment(Qt::AlignCenter);
  status_label_->setMinimumHeight(42);
  status_label_->setWordWrap(true);
  layout->addWidget(status_label_);

  detail_label_ = new QLabel("Waiting for gateway diagnostics", this);
  detail_label_->setAlignment(Qt::AlignCenter);
  detail_label_->setWordWrap(true);
  detail_label_->setStyleSheet("color: #b8bcc5;");
  layout->addWidget(detail_label_);

  auto * telemetry = new QGridLayout();
  telemetry->setHorizontalSpacing(14);
  telemetry->setVerticalSpacing(5);
  telemetry->addWidget(new QLabel("Temperature", this), 0, 0);
  telemetry->addWidget(new QLabel("Tracking", this), 1, 0);
  telemetry->addWidget(new QLabel("Trajectories", this), 2, 0);

  temperature_value_ = new QLabel("--", this);
  tracking_value_ = new QLabel("--", this);
  trajectory_value_ = new QLabel("--", this);
  for (auto * value : {temperature_value_, tracking_value_, trajectory_value_}) {
    value->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    value->setStyleSheet("font-weight: 600;");
  }
  telemetry->addWidget(temperature_value_, 0, 1);
  telemetry->addWidget(tracking_value_, 1, 1);
  telemetry->addWidget(trajectory_value_, 2, 1);
  layout->addLayout(telemetry);

  startup_pose_button_ = new QPushButton("Set Goal: Startup Pose", this);
  startup_pose_button_->setIcon(style()->standardIcon(QStyle::SP_ArrowBack));
  startup_pose_button_->setMinimumHeight(34);
  startup_pose_button_->setEnabled(false);
  layout->addWidget(startup_pose_button_);

  auto * scaling_layout = new QGridLayout();
  scaling_layout->setHorizontalSpacing(8);
  scaling_layout->setVerticalSpacing(5);
  scaling_layout->addWidget(new QLabel("AUTO Velocity Scaling", this), 0, 0);
  scaling_layout->addWidget(new QLabel("AUTO Accel Scaling", this), 1, 0);
  auto_velocity_scaling_ = new QDoubleSpinBox(this);
  auto_acceleration_scaling_ = new QDoubleSpinBox(this);
  for (auto * scaling : {auto_velocity_scaling_, auto_acceleration_scaling_}) {
    scaling->setRange(0.10, 1.00);
    scaling->setSingleStep(0.10);
    scaling->setDecimals(2);
    scaling->setValue(0.50);
    scaling->setAlignment(Qt::AlignRight);
  }
  scaling_layout->addWidget(auto_velocity_scaling_, 0, 1);
  scaling_layout->addWidget(auto_acceleration_scaling_, 1, 1);
  layout->addLayout(scaling_layout);

  auto * auto_layout = new QHBoxLayout();
  auto_layout->setSpacing(6);
  auto_plan_button_ = new QPushButton("AUTO Plan", this);
  auto_plan_button_->setIcon(style()->standardIcon(QStyle::SP_FileDialogDetailedView));
  auto_plan_button_->setMinimumHeight(34);
  auto_plan_button_->setEnabled(false);
  auto_execute_button_ = new QPushButton("AUTO Execute", this);
  auto_execute_button_->setIcon(style()->standardIcon(QStyle::SP_MediaPlay));
  auto_execute_button_->setMinimumHeight(34);
  auto_execute_button_->setEnabled(false);
  auto_execute_button_->setToolTip("Executes the saved AUTO plan; ARM and HOLD are required");
  auto_layout->addWidget(auto_plan_button_);
  auto_layout->addWidget(auto_execute_button_);
  layout->addLayout(auto_layout);

  arm_button_ = new QPushButton("ARM - Take Control", this);
  arm_button_->setIcon(style()->standardIcon(QStyle::SP_MediaPlay));
  arm_button_->setMinimumHeight(36);
  arm_button_->setEnabled(false);
  layout->addWidget(arm_button_);

  disarm_button_ = new QPushButton("Disarm", this);
  disarm_button_->setIcon(style()->standardIcon(QStyle::SP_MediaStop));
  disarm_button_->setMinimumHeight(32);
  disarm_button_->setEnabled(false);
  layout->addWidget(disarm_button_);

  reset_button_ = new QPushButton("Reset Safety Stop", this);
  reset_button_->setIcon(style()->standardIcon(QStyle::SP_BrowserReload));
  reset_button_->setMinimumHeight(32);
  reset_button_->setEnabled(false);
  layout->addWidget(reset_button_);

  feedback_label_ = new QLabel("", this);
  feedback_label_->setWordWrap(true);
  feedback_label_->setStyleSheet("color: #b8bcc5;");
  layout->addWidget(feedback_label_);
  layout->addStretch(1);

  connect(arm_button_, &QPushButton::clicked, this, &G1EndpointPanel::request_arm);
  connect(disarm_button_, &QPushButton::clicked, this, &G1EndpointPanel::request_disarm);
  connect(reset_button_, &QPushButton::clicked, this, &G1EndpointPanel::request_reset);
  connect(
    startup_pose_button_, &QPushButton::clicked,
    this, &G1EndpointPanel::set_startup_pose_goal);
  connect(auto_plan_button_, &QPushButton::clicked, this, &G1EndpointPanel::request_auto_plan);
  connect(
    auto_execute_button_, &QPushButton::clicked,
    this, &G1EndpointPanel::request_auto_execute);
  connect(
    auto_velocity_scaling_, qOverload<double>(&QDoubleSpinBox::valueChanged),
    this, &G1EndpointPanel::auto_scaling_changed);
  connect(
    auto_acceleration_scaling_, qOverload<double>(&QDoubleSpinBox::valueChanged),
    this, &G1EndpointPanel::auto_scaling_changed);

  refresh_timer_ = new QTimer(this);
  refresh_timer_->setInterval(250);
  connect(refresh_timer_, &QTimer::timeout, this, &G1EndpointPanel::refresh_ui);
  refresh_timer_->start();
  refresh_ui();
}

void G1EndpointPanel::onInitialize()
{
  const auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    feedback_label_->setText("RViz ROS node is unavailable");
    return;
  }

  node_ = abstraction->get_raw_node();
  diagnostics_sub_ = node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/g1/arm_gateway/diagnostics",
    rclcpp::QoS(5).reliable().durability_volatile(),
    std::bind(&G1EndpointPanel::diagnostics_callback, this, std::placeholders::_1));
  trajectory_sub_ = node_->create_subscription<moveit_msgs::msg::DisplayTrajectory>(
    "/display_planned_path",
    rclcpp::QoS(5).reliable().durability_volatile(),
    std::bind(&G1EndpointPanel::trajectory_callback, this, std::placeholders::_1));
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::QoS(5).best_effort().durability_volatile(),
    std::bind(&G1EndpointPanel::joint_state_callback, this, std::placeholders::_1));
  startup_goal_pub_ = node_->create_publisher<moveit_msgs::msg::RobotState>(
    "/rviz/moveit/update_custom_goal_state",
    rclcpp::QoS(1).reliable().durability_volatile());
  auto_velocity_scaling_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
    "/g1/rviz_auto/velocity_scaling", rclcpp::QoS(1).reliable().transient_local());
  auto_acceleration_scaling_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
    "/g1/rviz_auto/acceleration_scaling", rclcpp::QoS(1).reliable().transient_local());
  arm_client_ = node_->create_client<std_srvs::srv::Trigger>("/g1/arm_gateway/arm");
  disarm_client_ = node_->create_client<std_srvs::srv::Trigger>("/g1/arm_gateway/disarm");
  reset_client_ = node_->create_client<std_srvs::srv::Trigger>("/g1/arm_gateway/reset");
  auto_plan_client_ = node_->create_client<std_srvs::srv::Trigger>("/g1/rviz_auto_plan");
  auto_execute_client_ = node_->create_client<std_srvs::srv::Trigger>(
    "/g1/rviz_auto_execute");
  auto_scaling_changed(0.0);
}

void G1EndpointPanel::diagnostics_callback(
  const diagnostic_msgs::msg::DiagnosticArray::SharedPtr message)
{
  if (message->status.empty()) {
    return;
  }

  const auto & status = message->status.front();
  std::lock_guard<std::mutex> lock(mutex_);
  const std::string previous_phase = phase_;
  phase_ = value_of(status, "phase");
  diagnostic_message_ = status.message;
  const auto active_arm = value_of(status, "active_arm");
  if (!active_arm.empty()) diagnostic_message_ = "[" + active_arm + "] " + diagnostic_message_;
  diagnostic_level_ = status.level;
  armed_ = value_of(status, "armed") == "true";
  upper_body_state_safe_ = value_of(status, "upper_body_state_safe") == "true";
  service_state_ready_ = value_of(status, "service_state_ready") == "true";
  service_conflict_ = value_of(status, "service_conflict") == "true";
  temperature_alarm_active_ = value_of(status, "temperature_alarm_active") == "true";
  temperature_c_ = number_of(status, "max_motor_temperature_c");
  temperature_hard_limit_c_ = number_of(
    status, "motor_temperature_alarm_c",
    number_of(status, "motor_temperature_hard_limit_c", 85.0));
  tracking_error_deg_ = number_of(status, "tracking_error_rad") * 180.0 / M_PI;
  accepted_trajectories_ = static_cast<int>(number_of(status, "accepted_trajectories"));
  rejected_trajectories_ = static_cast<int>(number_of(status, "rejected_trajectories"));
  last_rejection_ = value_of(status, "last_rejection");
  last_diagnostics_time_ = std::chrono::steady_clock::now();
  have_diagnostics_ = true;

  if (phase_ == "RUNNING") {
    plan_ready_ = false;
    auto_plan_ready_ = false;
  } else if (previous_phase == "RUNNING" && phase_ == "ARMED_HOLD") {
    plan_ready_ = false;
    arrived_until_ = std::chrono::steady_clock::now() + std::chrono::seconds(4);
  }
  if (phase_ == "LATCHED_SILENT" || phase_ == "VELOCITY_RECOVERY") {
    plan_ready_ = false;
    auto_plan_ready_ = false;
  }
}

void G1EndpointPanel::trajectory_callback(
  const moveit_msgs::msg::DisplayTrajectory::SharedPtr message)
{
  std::size_t point_count = 0;
  for (const auto & trajectory : message->trajectory) {
    point_count += trajectory.joint_trajectory.points.size();
  }
  if (point_count == 0) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  planned_points_ = point_count;
  plan_ready_ = true;
  arrived_until_ = std::chrono::steady_clock::time_point{};
}

void G1EndpointPanel::joint_state_callback(
  const sensor_msgs::msg::JointState::SharedPtr message)
{
  for (std::size_t joint_index = 0; joint_index < kArmJointNames.size(); ++joint_index) {
    const auto it = std::find(
      message->name.begin(), message->name.end(), kArmJointNames[joint_index]);
    if (it == message->name.end()) {
      return;
    }
    const auto message_index = static_cast<std::size_t>(
      std::distance(message->name.begin(), it));
    if (message_index >= message->position.size() ||
      !std::isfinite(message->position[message_index]))
    {
      return;
    }
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (startup_pose_captured_) {
    return;
  }
  startup_joint_state_.header = message->header;
  startup_joint_state_.name = message->name;
  startup_joint_state_.position = message->position;
  startup_pose_captured_ = true;
  service_feedback_ = "Startup bilateral arm pose captured";
}

void G1EndpointPanel::set_startup_pose_goal()
{
  moveit_msgs::msg::RobotState startup_goal;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!startup_pose_captured_) {
      service_feedback_ = "Startup pose is not available yet";
      return;
    }
    startup_goal.joint_state = startup_joint_state_;
  }

  if (!startup_goal_pub_ || !node_) {
    std::lock_guard<std::mutex> lock(mutex_);
    service_feedback_ = "MoveIt startup-goal publisher is unavailable";
    return;
  }
  startup_goal.joint_state.header.stamp = node_->now();
  startup_goal.is_diff = false;
  startup_goal_pub_->publish(startup_goal);

  std::lock_guard<std::mutex> lock(mutex_);
  plan_ready_ = false;
  arrived_until_ = std::chrono::steady_clock::time_point{};
  service_feedback_ = "Goal set to the pose captured at RViz startup";
}

void G1EndpointPanel::refresh_ui()
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto now = std::chrono::steady_clock::now();
  const bool stale = !have_diagnostics_ ||
    now - last_diagnostics_time_ > std::chrono::seconds(2);

  QString status_text;
  QString detail_text;
  QString background = "#6b4f00";
  QString foreground = "#fff4c2";

  if (stale) {
    status_text = "STATUS STALE";
    detail_text = "Gateway diagnostics unavailable";
    background = "#7a241c";
    foreground = "#ffe7e3";
  } else if (diagnostic_level_ == diagnostic_msgs::msg::DiagnosticStatus::ERROR ||
    phase_ == "LATCHED_SILENT")
  {
    status_text = "SAFETY STOP";
    detail_text = QString::fromStdString(diagnostic_message_);
    background = "#9d1c16";
    foreground = "#ffffff";
  } else if (temperature_alarm_active_) {
    status_text = "TEMP ALARM";
    detail_text = QString::fromStdString(diagnostic_message_);
    background = "#9a6700";
    foreground = "#fff8d8";
  } else if (phase_ == "RUNNING") {
    status_text = "EXECUTING";
    detail_text = "Trajectory active";
    background = "#075985";
    foreground = "#e0f2fe";
  } else if (phase_ == "VELOCITY_RECOVERY") {
    status_text = "DAMPING RECOVERY";
    detail_text = "Trajectory stopped; maintaining damped support";
    background = "#9a6700";
    foreground = "#fff8d8";
  } else if (phase_ == "TAU_ANCHORED_HOLD" || phase_ == "GRAVITY_FEEDFORWARD_BLEND") {
    status_text = "TAKING CONTROL";
    detail_text = QString::fromStdString(phase_);
    background = "#854d0e";
    foreground = "#fef3c7";
  } else if (phase_ == "ARMED_HOLD" && plan_ready_) {
    status_text = "PLAN READY";
    detail_text = QString::number(planned_points_) + " trajectory points";
    background = "#075985";
    foreground = "#e0f2fe";
  } else if (phase_ == "ARMED_HOLD" &&
    arrived_until_ != std::chrono::steady_clock::time_point{} && now < arrived_until_)
  {
    status_text = "ARRIVED - HOLDING";
    detail_text = "Ready";
    background = "#166534";
    foreground = "#dcfce7";
  } else if (phase_ == "ARMED_HOLD") {
    status_text = "HOLD - READY";
    detail_text = "Ready";
    background = "#166534";
    foreground = "#dcfce7";
  } else if (!armed_) {
    status_text = "UNARMED";
    detail_text = QString::fromStdString(diagnostic_message_);
  } else {
    status_text = QString::fromStdString(phase_);
    detail_text = QString::fromStdString(diagnostic_message_);
  }

  status_label_->setText(status_text);
  status_label_->setStyleSheet(
    "QLabel { background: " + background + "; color: " + foreground +
    "; border-radius: 4px; padding: 7px; font-size: 15px; font-weight: 700; }");
  detail_label_->setText(detail_text);

  if (have_diagnostics_) {
    temperature_value_->setText(one_decimal(temperature_c_) + " C");
    if (temperature_c_ >= temperature_hard_limit_c_ - 3.0) {
      temperature_value_->setStyleSheet("font-weight: 700; color: #ef4444;");
    } else if (temperature_c_ >= 75.0) {
      temperature_value_->setStyleSheet("font-weight: 700; color: #f59e0b;");
    } else {
      temperature_value_->setStyleSheet("font-weight: 600;");
    }
    tracking_value_->setText(one_decimal(tracking_error_deg_) + " deg");
    trajectory_value_->setText(
      QString::number(accepted_trajectories_) + " accepted / " +
      QString::number(rejected_trajectories_) + " rejected");
  } else {
    temperature_value_->setText("--");
    tracking_value_->setText("--");
    trajectory_value_->setText("--");
  }

  const bool arm_ready = !stale && !armed_ && upper_body_state_safe_ &&
    service_state_ready_ && !service_conflict_ && !temperature_alarm_active_ &&
    diagnostic_level_ != diagnostic_msgs::msg::DiagnosticStatus::ERROR &&
    arm_client_ && arm_client_->service_is_ready() && !service_call_pending_;
  arm_button_->setEnabled(arm_ready);
  disarm_button_->setEnabled(
    armed_ && disarm_client_ && disarm_client_->service_is_ready() &&
    !service_call_pending_);
  reset_button_->setEnabled(
    !stale && phase_ == "LATCHED_SILENT" && upper_body_state_safe_ &&
    service_state_ready_ && !service_conflict_ && reset_client_ &&
    reset_client_->service_is_ready() && !service_call_pending_);
  startup_pose_button_->setEnabled(
    startup_pose_captured_ && startup_goal_pub_ && !service_call_pending_ &&
    phase_ != "RUNNING" && phase_ != "VELOCITY_RECOVERY");
  auto_plan_button_->setEnabled(
    !stale && auto_plan_client_ && auto_plan_client_->service_is_ready() &&
    !service_call_pending_ && phase_ != "RUNNING" && phase_ != "VELOCITY_RECOVERY");
  auto_execute_button_->setEnabled(
    !stale && phase_ != "RUNNING" && phase_ != "VELOCITY_RECOVERY" &&
    auto_execute_client_ && auto_execute_client_->service_is_ready() &&
    !service_call_pending_);

  QString feedback = QString::fromStdString(service_feedback_);
  if (rejected_trajectories_ > 0 && !last_rejection_.empty()) {
    if (!feedback.isEmpty()) {
      feedback += "\n";
    }
    feedback += "Last reject: " + QString::fromStdString(last_rejection_);
  }
  feedback_label_->setText(feedback);
}

void G1EndpointPanel::request_arm()
{
  request_gateway(arm_client_, "Requesting control...");
}

void G1EndpointPanel::request_disarm()
{
  request_gateway(disarm_client_, "Releasing control...");
}

void G1EndpointPanel::request_reset()
{
  request_gateway(reset_client_, "Resetting safety latch...");
}

void G1EndpointPanel::request_auto_plan()
{
  request_auto(false);
}

void G1EndpointPanel::request_auto_execute()
{
  request_auto(true);
}

void G1EndpointPanel::request_auto(bool execute)
{
  const auto & client = execute ? auto_execute_client_ : auto_plan_client_;
  if (!client || !client->service_is_ready()) {
    std::lock_guard<std::mutex> lock(mutex_);
    service_feedback_ = "RViz AUTO service unavailable";
    return;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (service_call_pending_) {
      return;
    }
    service_call_pending_ = true;
    service_feedback_ = execute ? "Executing saved AUTO trajectory..." : "Planning AUTO...";
    if (execute) {
      auto_plan_ready_ = false;
    }
  }
  refresh_ui();

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  client->async_send_request(
    request,
    [this, execute](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      std::lock_guard<std::mutex> lock(mutex_);
      service_call_pending_ = false;
      try {
        const auto response = future.get();
        service_feedback_ = response->message;
        if (!execute) {
          auto_plan_ready_ = response->success;
          if (response->success && !armed_) {
            service_feedback_ += "; click ARM and wait for HOLD - READY before AUTO Execute";
          }
        }
      } catch (const std::exception & error) {
        service_feedback_ = std::string("RViz AUTO request failed: ") + error.what();
        if (!execute) {
          auto_plan_ready_ = false;
        }
      }
    });
}

void G1EndpointPanel::auto_scaling_changed(double)
{
  std_msgs::msg::Float64 velocity;
  std_msgs::msg::Float64 acceleration;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto_plan_ready_ = false;
    if (!auto_velocity_scaling_ || !auto_acceleration_scaling_) {
      return;
    }
    velocity.data = auto_velocity_scaling_->value();
    acceleration.data = auto_acceleration_scaling_->value();
    if (node_) {
      service_feedback_ = "AUTO scaling changed; run AUTO Plan again";
    }
  }
  if (auto_velocity_scaling_pub_) {
    auto_velocity_scaling_pub_->publish(velocity);
  }
  if (auto_acceleration_scaling_pub_) {
    auto_acceleration_scaling_pub_->publish(acceleration);
  }
}

void G1EndpointPanel::request_gateway(
  const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client,
  const std::string & pending_text)
{
  if (!client || !client->service_is_ready()) {
    std::lock_guard<std::mutex> lock(mutex_);
    service_feedback_ = "Gateway service unavailable";
    return;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (service_call_pending_) {
      return;
    }
    service_call_pending_ = true;
    service_feedback_ = pending_text;
  }
  refresh_ui();

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  client->async_send_request(
    request,
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      std::lock_guard<std::mutex> lock(mutex_);
      service_call_pending_ = false;
      try {
        const auto response = future.get();
        service_feedback_ = response->message;
      } catch (const std::exception & error) {
        service_feedback_ = std::string("Gateway request failed: ") + error.what();
      }
    });
}

}  // namespace g1_unitree_control

PLUGINLIB_EXPORT_CLASS(g1_unitree_control::G1EndpointPanel, rviz_common::Panel)
