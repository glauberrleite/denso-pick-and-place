cmake_minimum_required(VERSION 2.8.3)
project(vp6242_kinematics_publisher)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(vp6242_state_publisher src/vp6242_state_publisher.cpp)
target_link_libraries(vp6242_state_publisher ${catkin_LIBRARIES})

