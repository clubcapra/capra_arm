cmake_minimum_required(VERSION 2.8.3)
project(ovis_driver)

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
    LIBRARIES ovis_driver
)

include_directories(
  include
  include/kinova
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/ovis_driver.cpp)

add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})