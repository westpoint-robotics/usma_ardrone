cmake_minimum_required(VERSION 2.8.3)
project(usma_ardrone_gazebo)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
)

find_package(catkin REQUIRED)

catkin_package()


include_directories(
  ${catkin_INCLUDE_DIRS}
)

