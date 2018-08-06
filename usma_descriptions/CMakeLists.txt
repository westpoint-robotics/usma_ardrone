cmake_minimum_required(VERSION 2.8.3)
project(usma_descriptions)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
)

## System dependencies are found with CMake's conventions
# Depend on system install of Gazebo
find_package(gazebo REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hast_kobuki_description
#  CATKIN_DEPENDS other_catkin_pkg
    DEPENDS
      gazebo_ros
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})

