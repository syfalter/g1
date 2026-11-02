cmake_minimum_required(VERSION 3.8)
project(g1_unitree_control)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(diagnostic_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(unitree_api REQUIRED)
find_package(unitree_hg REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME} "srv/ResumeTaskHold.srv")
rosidl_get_typesupport_target(task_typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_executable(arm_state_bridge src/arm_state_bridge.cpp)
target_compile_features(arm_state_bridge PRIVATE cxx_std_17)
ament_target_dependencies(arm_state_bridge
  diagnostic_msgs
  rclcpp
  sensor_msgs
  unitree_hg
)

add_executable(g1_safety_monitor src/g1_safety_monitor.cpp)
target_compile_features(g1_safety_monitor PRIVATE cxx_std_17)
ament_target_dependencies(g1_safety_monitor
  rclcpp
  unitree_hg
)

add_executable(shadow_trajectory_executor src/shadow_trajectory_executor.cpp)
target_compile_features(shadow_trajectory_executor PRIVATE cxx_std_17)
ament_target_dependencies(shadow_trajectory_executor
  diagnostic_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  trajectory_msgs
  unitree_hg
)

add_executable(arm_sdk_hardware_gateway src/arm_sdk_hardware_gateway.cpp)
target_compile_features(arm_sdk_hardware_gateway PRIVATE cxx_std_17)
ament_target_dependencies(arm_sdk_hardware_gateway
  control_msgs
  diagnostic_msgs
  kdl_parser
  rcl_interfaces
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  std_srvs
  trajectory_msgs
  unitree_api
  unitree_hg
)
target_link_libraries(arm_sdk_hardware_gateway Eigen3::Eigen orocos-kdl)
target_link_libraries(arm_sdk_hardware_gateway "${task_typesupport}")
add_executable(task_gateway_simulator src/task_gateway_simulator.cpp)
target_compile_features(task_gateway_simulator PRIVATE cxx_std_17)
ament_target_dependencies(task_gateway_simulator rclcpp sensor_msgs unitree_hg)

add_executable(right_arm_static_calibrator src/right_arm_static_calibrator.cpp)
target_compile_features(right_arm_static_calibrator PRIVATE cxx_std_17)
ament_target_dependencies(right_arm_static_calibrator
  kdl_parser
  rclcpp
  std_srvs
  unitree_hg
)
target_link_libraries(right_arm_static_calibrator Eigen3::Eigen orocos-kdl)

add_executable(planned_joint_state_overlay src/planned_joint_state_overlay.cpp)
target_compile_features(planned_joint_state_overlay PRIVATE cxx_std_17)
ament_target_dependencies(planned_joint_state_overlay rclcpp sensor_msgs)

add_executable(rviz_status_caption src/rviz_status_caption.cpp)
target_compile_features(rviz_status_caption PRIVATE cxx_std_17)
ament_target_dependencies(rviz_status_caption
  diagnostic_msgs
  moveit_msgs
  rclcpp
  visualization_msgs
)

add_library(g1_endpoint_rviz_panel SHARED
  include/g1_unitree_control/g1_endpoint_panel.hpp
  src/g1_endpoint_panel.cpp
)
target_compile_features(g1_endpoint_rviz_panel PRIVATE cxx_std_17)
target_include_directories(g1_endpoint_rviz_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(g1_endpoint_rviz_panel
  diagnostic_msgs
  moveit_msgs
  pluginlib
  rclcpp
  rviz_common
  sensor_msgs
  std_msgs
  std_srvs
)
target_link_libraries(g1_endpoint_rviz_panel Qt5::Widgets)
pluginlib_export_plugin_description_file(rviz_common rviz_plugin_description.xml)

install(TARGETS
  arm_state_bridge
  g1_safety_monitor
  shadow_trajectory_executor
  arm_sdk_hardware_gateway
  task_gateway_simulator
  right_arm_static_calibrator
  planned_joint_state_overlay
  rviz_status_caption
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS g1_endpoint_rviz_panel
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md rviz_plugin_description.xml DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
