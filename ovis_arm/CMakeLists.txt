cmake_minimum_required(VERSION 2.8.3)
project(ovis_arm)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
 roscpp
 std_msgs
 geometry_msgs
 kinova_driver
 kinova_msgs
 ovis_msgs
)

link_directories(${PROJECT_SOURCE_DIR}/lib)

catkin_package( 
    INCLUDE_DIRS include
    CATKIN_DEPENDS kinova_driver kinova_msgs roscpp std_msgs geometry_msgs ovis_msgs
    LIBRARIES ovis_arm
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(arm 
  src/ovis_arm.cpp
  include/ovis_arm.hpp 
  include/kinova/Kinova.API.USBCommandLayerUbuntu.h
  include/kinova/Kinova.API.EthCommandLayerUbuntu.h 
  include/kinova/KinovaTypes.h)

add_dependencies(arm ${catkin_EXPORTED_TARGETS})

target_link_libraries(arm ${catkin_LIBRARIES})