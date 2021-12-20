cmake_minimum_required(VERSION 2.8.3)
project(ovis_bringup)

find_package(catkin REQUIRED)
catkin_package(
)

add_executable(ovis_demo src/ovis_demo.cpp)