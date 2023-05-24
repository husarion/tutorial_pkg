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
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(my_first_node src/my_first_node.cpp)
ament_target_dependencies(my_first_node rclcpp sensor_msgs std_msgs std_srvs)

install(TARGETS
  my_first_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()