cmake_minimum_required(VERSION 2.8.3)
project(tutorial_pkg)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp tf
)

find_package( OpenCV REQUIRED )

catkin_package(
  CATKIN_DEPENDS
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(my_first_node src/my_first_node.cpp)
target_link_libraries(my_first_node
  ${catkin_LIBRARIES}
)

add_executable(img_control_node src/img_control.cpp)
target_link_libraries(img_control_node
    ${catkin_LIBRARIES}
)

add_executable(follow_img src/follow_img.cpp)
target_link_libraries(follow_img
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

add_executable(tf_broadcaster src/tf_broadcaster.cpp)
target_link_libraries(tf_broadcaster
    ${catkin_LIBRARIES}
)

add_executable(tf_listener src/tf_listener.cpp)
target_link_libraries(tf_listener
    ${catkin_LIBRARIES}
)