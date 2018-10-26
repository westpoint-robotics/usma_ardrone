cmake_minimum_required(VERSION 2.8.3)
project(ardrone_face_tracker)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
  sensor_msgs
)
find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui objdetect)

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# add_executable(face_tracker_controller src/face_tracker_controller.cpp)
#add_dependencies(face_tracker_controller ${${PROJECT_NAME}_EXPORTED_TARGETS})
# target_link_libraries(face_tracker_controller ${catkin_LIBRARIES})

# add_executable(face_tracker_node src/face_tracker_node.cpp)
# target_link_libraries(face_tracker_node
#    ${catkin_LIBRARIES}
#    ${OpenCV_LIBRARIES}
#  )
