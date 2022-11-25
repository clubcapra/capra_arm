cmake_minimum_required(VERSION 2.8.3)
project(ovis_jog_control)


find_package(catkin REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ovis_jog_launch
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)


install(PROGRAMS
   scripts/ovis_control
   scripts/ovis_action_server
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

