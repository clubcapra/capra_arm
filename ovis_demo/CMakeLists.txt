cmake_minimum_required(VERSION 2.8.3)
project(ovis_demo)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
 roscpp
 std_msgs
 geometry_msgs
 kinova_driver
 kinova_msgs
)

link_directories(${PROJECT_SOURCE_DIR}/lib)

catkin_package( 
    INCLUDE_DIRS include
    CATKIN_DEPENDS kinova_driver kinova_msgs roscpp std_msgs geometry_msgs
    LIBRARIES ovis_demo
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/ovis_demo.cpp include/ovis_demo.hpp)

add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})