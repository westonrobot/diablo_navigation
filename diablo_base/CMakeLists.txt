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

add_message_files(
  FILES
  coors.msg
)

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
)

link_libraries(
    ${catkin_LIBRARIES}
)

add_executable(odom_broadcaster src/odom_broadcaster.cpp)
target_link_libraries(odom_broadcaster PRIVATE ${catkin_LIBRARIES})

add_executable(robot_navigation src/robot_navigation.cpp)
target_link_libraries(robot_navigation PRIVATE ${catkin_LIBRARIES})

# add_executable(tf_listener 
# ${CMAKE_CURRENT_SOURCE_DIR}/src/test_transform/tf_listener.cpp)
# # add_dependencies(tf_listener diablo_sdk_generate_messages_cpp)
# target_link_libraries(tf_listener PRIVATE Threads::Threads diablo_sdk)

# add_executable(tf_broadcaster 
# ${CMAKE_CURRENT_SOURCE_DIR}/src/test_transform/tf_broadcaster.cpp)
# # add_dependencies(tf_broadcaster diablo_sdk_generate_messages_cpp)
# target_link_libraries(tf_broadcaster PRIVATE Threads::Threads diablo_sdk)

# add_executable(formula_quat 
# ${CMAKE_CURRENT_SOURCE_DIR}/src/test_formula/formula_quat.cpp)
# # add_dependencies(formula_quat diablo_sdk_generate_messages_cpp)
# target_link_libraries(formula_quat PRIVATE Threads::Threads diablo_sdk)

# add_executable(formula_1 
# ${CMAKE_CURRENT_SOURCE_DIR}/src/test_formula/formula_1.cpp)
# # add_dependencies(formula_1 diablo_sdk_generate_messages_cpp)
# target_link_libraries(formula_1 PRIVATE Threads::Threads diablo_sdk)

# add_executable(formula_2 
# ${CMAKE_CURRENT_SOURCE_DIR}/src/test_formula/formula_2.cpp)
# # add_dependencies(formula_2 diablo_sdk_generate_messages_cpp)
# target_link_libraries(formula_2 PRIVATE Threads::Threads diablo_sdk)

# add_executable(test_base_custom 
# ${CMAKE_CURRENT_SOURCE_DIR}/src/read_and_control/test_base_custom.cpp)
# # add_dependencies(test_base_custom diablo_sdk_generate_messages_cpp)
# target_link_libraries(test_base_custom PRIVATE Threads::Threads diablo_sdk)

# add_executable(testing 
# ${CMAKE_CURRENT_SOURCE_DIR}/src/testing.cpp)
# # add_dependencies(testing diablo_sdk_generate_messages_cpp)
# target_link_libraries(testing PRIVATE Threads::Threads diablo_sdk)
