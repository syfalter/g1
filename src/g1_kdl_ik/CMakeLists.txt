cmake_minimum_required(VERSION 3.8)
project(g1_kdl_ik)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/QueryCapability.srv"
  "srv/QueryReachability.srv"
  "srv/SolveDualArmIK.srv"
  DEPENDENCIES geometry_msgs sensor_msgs
)

add_executable(kdl_ik_server src/kdl_ik_server.cpp)
ament_target_dependencies(kdl_ik_server
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning
  rclcpp
  sensor_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(kdl_ik_server "${cpp_typesupport_target}")

add_executable(reachability_sampler src/reachability_sampler.cpp)
ament_target_dependencies(reachability_sampler
  geometry_msgs
  moveit_core
  moveit_ros_planning
  rclcpp
)

add_executable(reachability_map_server src/reachability_map_server.cpp)
ament_target_dependencies(reachability_map_server
  geometry_msgs
  moveit_core
  moveit_ros_planning
  rclcpp
  sensor_msgs
)
target_link_libraries(reachability_map_server "${cpp_typesupport_target}")

add_executable(capability_sampler src/capability_sampler.cpp)
target_include_directories(capability_sampler PRIVATE include)
ament_target_dependencies(capability_sampler
  geometry_msgs
  moveit_core
  moveit_ros_planning
  rclcpp
)

add_executable(capability_map_server src/capability_map_server.cpp)
target_include_directories(capability_map_server PRIVATE include)
ament_target_dependencies(capability_map_server
  geometry_msgs
  interactive_markers
  moveit_core
  moveit_ros_planning
  rclcpp
  sensor_msgs
  visualization_msgs
)
target_link_libraries(capability_map_server "${cpp_typesupport_target}")

install(TARGETS kdl_ik_server reachability_sampler reachability_map_server
  capability_sampler capability_map_server
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
