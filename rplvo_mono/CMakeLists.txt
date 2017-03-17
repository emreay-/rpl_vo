cmake_minimum_required(VERSION 2.8.3)
project(rplvo_mono)
set(openCV_DIR /opt/ros/indigo/share/OpenCV-3.1.0-dev)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  sensor_msgs
  image_transport
  vikit_ros
  tf
)
find_package(OpenCV 3.1.0 REQUIRED COMPONENTS)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rpl_mono_odometer
#  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs image_transport cv_bridge tf vikit_ros
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(rpl_mono_odometer src/rpl_mono_odometer.cpp)
target_link_libraries(rpl_mono_odometer ${LINK_LIBS} ${catkin_LIBRARIES})

#add_executable(feature_extract src/feature_extract.cpp)
#target_link_libraries(feature_extract ${catkin_LIBRARIES})
#add_dependencies(feature_extract rplvo_mono_generate_messages_cpp)

add_executable(mono src/mono.cpp)
target_link_libraries(mono rpl_mono_odometer ${catkin_LIBRARIES})
add_dependencies(mono rplvo_mono_generate_messages_cpp)

