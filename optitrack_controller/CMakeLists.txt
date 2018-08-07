cmake_minimum_required(VERSION 2.8.3)
project(optitrack_controller)

find_package(catkin REQUIRED COMPONENTS
  ardrone_autonomy
  cv_bridge
  geometry_msgs

  image_transport
  nav_msgs
  pcl_conversions
  pcl_ros
  # opencv2
  roscpp
  sensor_msgs
  std_msgs
  tf
  #pointgrey_camera_driver
  usma_plugins
)


# find_package(OpenCV REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    # flag.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  # null.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    geometry_msgs
    nav_msgs
    sensor_msgs
    std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES optitrack_controller
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(         optitrackAutopilot  src/optitrackAutopilot.cpp)
  add_dependencies(     optitrackAutopilot   ${catkin_EXPORTED_TARGETS})
    target_link_libraries(optitrackAutopilot  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
