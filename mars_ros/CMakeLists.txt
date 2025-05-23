cmake_minimum_required(VERSION 3.8)
project(mars_ros2)

#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra -Wpedantic)
#endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mars_msgs REQUIRED)

find_package(Eigen3 REQUIRED)

set(mars_lib_DIR ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(mars_lib REQUIRED)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

if (CMAKE_BUILD_TYPE STREQUAL "Release")
  set(MARS_LIBRARY ${MARS_LIBRARY_RELEASE})
elseif (CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(MARS_LIBRARY ${MARS_LIBRARY_DEBUG})
endif ()

message("** ROS2 Build **")
message(" * Build type: ${CMAKE_BUILD_TYPE}")
message(" * Eigen: ${EIGEN3_INCLUDE_DIR}")
message(" * Mars Dir:  ${MARS_INCLUDE_DIRS}")
message(" * Mars Lib: ${MARS_LIBRARY}")

include_directories(
  include
  ${MARS_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  )  
  
add_executable(pose_node src/mars_node.cpp src/mars_wrapper_pose.cpp)
add_dependencies(pose_node mars_lib)
ament_target_dependencies(pose_node
rclcpp
std_msgs
sensor_msgs
geometry_msgs
nav_msgs
mars_msgs
)
target_link_libraries(pose_node ${MARS_LIBRARY})
set_property(TARGET pose_node PROPERTY COMPILE_DEFINITIONS POSE)

add_executable(dualpose_node src/mars_node.cpp src/mars_wrapper_dualpose.cpp)
add_dependencies(dualpose_node mars_lib)
ament_target_dependencies(dualpose_node
rclcpp
std_msgs
sensor_msgs
geometry_msgs
nav_msgs
mars_msgs
)
target_link_libraries(dualpose_node ${MARS_LIBRARY})
set_property(TARGET dualpose_node PROPERTY COMPILE_DEFINITIONS DUALPOSE)

install(TARGETS
  pose_node
  dualpose_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
