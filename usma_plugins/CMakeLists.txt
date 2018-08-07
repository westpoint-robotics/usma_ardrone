cmake_minimum_required(VERSION 2.8.3)
project(usma_plugins)
set (CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  ardrone_autonomy
  gazebo_ros
  gazebo_plugins
  geometry_msgs
  image_transport

  nav_msgs
  roscpp
  sensor_msgs
  std_msgs

  tf
  tf2_ros
)

## System dependencies are found with CMake's conventions
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(XML libxml-2.0)
  pkg_check_modules(OGRE OGRE)
  pkg_check_modules(OGRE-Terrain OGRE-Terrain)
  pkg_check_modules(OGRE-Paging OGRE-Paging)
else()
  message(FATAL_ERROR "pkg-config is required; please install it")
endif()

find_package(
  Boost 
  REQUIRED COMPONENTS 
  thread
)
find_package(gazebo REQUIRED)

execute_process(COMMAND
  pkg-config --variable=plugindir OGRE
  OUTPUT_VARIABLE OGRE_PLUGIN_PATH
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SetBias.srv
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  flag.msg
  pid_controller.msg
  pid_controller1.msg
  pid_controllers.msg
  pid3.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs 
  nav_msgs 
  sensor_msgs 
  std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${SDFormat_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  ${OGRE-Terrain_INCLUDE_DIRS}
  ${OGRE-Paging_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${catkin_LIBRARY_DIRS}
  ${OGRE_LIBRARY_DIRS}
  ${OGRE-Terrain_LIBRARY_DIRS}
  ${OGRE-Paging_LIBRARY_DIRS}
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES 
    usma_ardrone_ros_baro 
    usma_ardrone_simple_controller
    usma_ardrone_state_controller
    reset_plugin 
   CATKIN_DEPENDS 
    gazebo_plugins
    geometry_msgs
    kobuki_msgs
    nav_msgs
    roscpp
    sensor_msgs
    std_msgs
    image_transport 
    usma_ardrone_gazebo_msgs
   DEPENDS 
    system_lib
    tf
    tf2_ros
)


## Declare a cpp library
# if not avaible, uncomment the following line and the corresponding line in the LIBRARIES list under catkin_package()
# add_library(reset_plugin src/reset_plugin.cpp)

add_library(usma_ardrone_ros_baro src/usma_ardrone_ros_baro.cpp)
  add_dependencies(usma_ardrone_ros_baro usma_ardrone_gazebo_msgs_gencpp)

add_library(usma_ardrone_simple_controller src/usma_ardrone_simple_controller.cpp)
  add_dependencies(usma_ardrone_simple_controller ardrone_autonomy_gencpp)

add_library(usma_ardrone_state_controller src/usma_ardrone_state_controller.cpp)
  add_dependencies(usma_ardrone_state_controller ardrone_autonomy_gencpp)
  target_link_libraries(usma_ardrone_state_controller ${image_transport_LIBRARIES})

# add_library(gazebo_vicon_diff_drive src/gazebo_vicon_diff_drive.cpp)
#   target_link_libraries(gazebo_vicon_diff_drive gazebo_ros_utils ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# add_library(ugv2_gazebo_ros_kobuki 
#       src/gazebo_ros_kobuki_vicon.cpp 
#       src/gazebo_ros_kobuki_vicon_updates.cpp 
#       src/gazebo_ros_kobuki_vicon_loads.cpp)
#   add_dependencies(ugv2_gazebo_ros_kobuki ${catkin_EXPORTED_TARGETS})
#   target_link_libraries(ugv2_gazebo_ros_kobuki ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
