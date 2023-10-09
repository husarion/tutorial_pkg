cmake_minimum_required(VERSION 3.8)
project(tutorial_pkg)

# Set C++ standard to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(
  include
)

add_executable(my_first_node src/my_first_node.cpp)
ament_target_dependencies(my_first_node rclcpp sensor_msgs std_msgs std_srvs)

add_executable(tracker src/tracker.cpp)
ament_target_dependencies(tracker rclcpp OpenCV cv_bridge geometry_msgs sensor_msgs std_msgs)

add_executable(tf_broadcaster src/tf_broadcaster.cpp)
ament_target_dependencies(tf_broadcaster rclcpp geometry_msgs tf2 tf2_ros)

add_executable(tf_listener src/tf_listener.cpp)
ament_target_dependencies(tf_listener rclcpp geometry_msgs tf2 tf2_ros)


install(TARGETS
  my_first_node
  tracker
  tf_broadcaster
  tf_listener
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  maps
  rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()