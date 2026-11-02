cmake_minimum_required(VERSION 3.8)
project(g1_upper_body_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PlanSingleArm.srv"
  "srv/TimeTaskPath.srv"
  "srv/ValidateTaskState.srv"
  "srv/ValidateTaskTrajectory.srv"
  DEPENDENCIES geometry_msgs moveit_msgs
)

add_executable(single_arm_planner_server src/single_arm_planner_server.cpp)
ament_target_dependencies(single_arm_planner_server
  control_msgs
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  rclcpp_action
  shape_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
  trajectory_msgs
  visualization_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(single_arm_planner_server "${cpp_typesupport_target}")

add_executable(task_state_validator src/task_state_validator.cpp)
target_compile_features(task_state_validator PRIVATE cxx_std_17)
ament_target_dependencies(task_state_validator rclcpp std_msgs moveit_core moveit_msgs moveit_ros_planning)
target_link_libraries(task_state_validator "${cpp_typesupport_target}")

add_executable(single_arm_plan_demo src/single_arm_plan_demo.cpp)
ament_target_dependencies(single_arm_plan_demo
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
)
target_link_libraries(single_arm_plan_demo "${cpp_typesupport_target}")

add_executable(right_arm_calibration_pose_screener
  src/right_arm_calibration_pose_screener.cpp)
ament_target_dependencies(right_arm_calibration_pose_screener
  moveit_core
  moveit_ros_planning
  rclcpp
  sensor_msgs
)

install(TARGETS single_arm_planner_server single_arm_plan_demo
  right_arm_calibration_pose_screener task_state_validator
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  enable_testing()
  add_executable(task_scene_revision_test src/task_scene_revision_test.cpp)
  target_compile_features(task_scene_revision_test PRIVATE cxx_std_17)
  ament_target_dependencies(task_scene_revision_test moveit_msgs)
  add_test(NAME task_scene_revision_test COMMAND task_scene_revision_test)
endif()

ament_package()
