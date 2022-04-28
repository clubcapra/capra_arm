cmake_minimum_required(VERSION 3.0.2)
project(ovis_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
)

add_message_files(
  FILES
  OvisIKGoal.msg
  OvisJointAngles.msg
  OvisArmJointVelocity.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime 
)
