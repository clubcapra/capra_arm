cmake_minimum_required(VERSION 2.8.3)
project(ovis_demo)

add_compile_options(-std=c++14)
find_package(catkin REQUIRED COMPONENTS
 roscpp
 std_msgs
 geometry_msgs
 kinova_driver
 kinova_msgs
)

catkin_package( CATKIN_DEPENDS kinova_driver kinova_msgs roscpp std_msgs geometry_msgs)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node src/ovis_demo.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})