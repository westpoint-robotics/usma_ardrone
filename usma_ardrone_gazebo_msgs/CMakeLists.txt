cmake_minimum_required(VERSION 2.8.3)
project(usma_ardrone_gazebo_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Altimeter.msg
  Altitude.msg
  AttitudeCommand.msg
  Compass.msg
  ControllerState.msg
  HeadingCommand.msg
  HeightCommand.msg
  MotorCommand.msg
  MotorPWM.msg
  MotorStatus.msg
  PositionXYCommand.msg
  RawImu.msg
  RawMagnetic.msg
  RawRC.msg
  RC.msg
  RuddersCommand.msg
  ServoCommand.msg
  Supply.msg
  ThrustCommand.msg
  VelocityXYCommand.msg
  VelocityZCommand.msg
  YawrateCommand.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
