cmake_minimum_required(VERSION 3.0.2)
project(project_robotics)

find_package(catkin REQUIRED COMPONENTS
  robotics_hw1
  dynamic_reconfigure
  message_filters
  nav_msgs
  roscpp 
  std_msgs 
  tf 
  tf2 
  tf2_ros 
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(FILES
  CustomOdometry.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  Reset.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  nav_msgs
  std_msgs
  geometry_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/dynrec.cfg
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS robotics_hw1 dynamic_reconfigure message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(odometry src/odometry.cpp)
add_dependencies(odometry ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(odometry ${catkin_LIBRARIES})
