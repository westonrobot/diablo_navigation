cmake_minimum_required(VERSION 3.0)
project(diablo_base)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  rospy
  std_msgs
  message_generation
  tf 
  diablo_sdk
)

################################################
## Declare ROS messages, services and actions ##
################################################

# add_message_files(
#   FILES
#   coors.msg
# )

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  #message_generation
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES
    CATKIN_DEPENDS
    message_runtime
    # DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
    ${catkin_INCLUDE_DIRS}
    include
)

add_executable(diablo_base src/diablo_base.cpp)
target_link_libraries(diablo_base PRIVATE ${catkin_LIBRARIES})

add_executable(diablo_base_node src/main.cpp src/diablo_robot.cpp)
target_link_libraries(diablo_base_node PRIVATE ${catkin_LIBRARIES})
