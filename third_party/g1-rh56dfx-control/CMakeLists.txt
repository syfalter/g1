cmake_minimum_required(VERSION 3.10)
project(g1_rh56dfx_control LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(CTest)
find_package(unitree_sdk2 REQUIRED)
find_package(yaml-cpp REQUIRED)

add_subdirectory(src)

if(BUILD_TESTING)
  function(add_rh56_test target)
    add_executable(${target} tests/${target}.cpp)
    target_include_directories(${target} PRIVATE src)
    target_link_libraries(${target} PRIVATE ${ARGN})
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
    add_test(NAME ${target} COMMAND ${target})
  endfunction()

  add_rh56_test(rh56_controller_test)
  add_rh56_test(hand_api_test unitree_sdk2)
  add_rh56_test(persistent_store_test unitree_sdk2)
  add_rh56_test(vision_api_test unitree_sdk2)
endif()
