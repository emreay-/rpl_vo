cmake_minimum_required(VERSION 2.8.3)
project(rplvo_core)
set(openCV_DIR /opt/ros/indigo/share/OpenCV-3.1.0-dev)
## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  vikit_ros
  cv_bridge
  sensor_msgs
)
find_package(OpenCV 3.1.0 REQUIRED COMPONENTS)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rplvo_visualizer
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
add_library(rplvo_visualizer src/rplvo_visualizer.cpp)
target_link_libraries(rplvo_visualizer ${LINK_LIBS} ${catkin_LIBRARIES})
