cmake_minimum_required(VERSION 2.8.3)
project(msg_to_tf)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(msg_to_tf src/msg_to_tf.cpp)
target_link_libraries(msg_to_tf
  ${catkin_LIBRARIES}
)


