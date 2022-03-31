cmake_minimum_required(VERSION 3.0.2)
project(ovis_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  OvisIKGoal.msg
  OvisJointAngles.msg
  OvisJointGoal.msg
)

# Generate services in the 'srv' folder
add_service_files(
  FILES
  HomeJoint.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime 
)
