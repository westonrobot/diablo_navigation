cmake_minimum_required(VERSION 3.0)

project(diablo_base)

add_compile_options(
		-std=c++17
		-Wno-psabi
)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
add_definitions(-D_LOG_CONSOLE)

FILE(GLOB OSDK_LIB_SRCS
        ../../diablo-sdk-v1/api/src/*.cpp
)
add_library(
    ${PROJECT_NAME}
    STATIC
    ${OSDK_LIB_SRCS}
)

include_directories(
  ../../diablo-sdk-v1/api/inc
  #inc/diablo_base
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  rospy
  std_msgs
  message_generation
  tf 
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  angles.msg
  disps.msg
  coors.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  #message_generation
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
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

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
link_libraries(
        ${catkin_LIBRARIES}
    )

add_executable(tf_listener ${OSDK_LIB_SRCS}
${CMAKE_CURRENT_SOURCE_DIR}/src/test_transform/tf_listener.cpp)
add_dependencies(tf_listener diablo_sdk_generate_messages_cpp)
target_link_libraries(tf_listener PRIVATE Threads::Threads)

add_executable(tf_broadcaster ${OSDK_LIB_SRCS}
${CMAKE_CURRENT_SOURCE_DIR}/src/test_transform/tf_broadcaster.cpp)
add_dependencies(tf_broadcaster diablo_sdk_generate_messages_cpp)
target_link_libraries(tf_broadcaster PRIVATE Threads::Threads)

add_executable(formula_quat ${OSDK_LIB_SRCS}
${CMAKE_CURRENT_SOURCE_DIR}/src/test_formula/formula_quat.cpp)
add_dependencies(formula_quat diablo_sdk_generate_messages_cpp)
target_link_libraries(formula_quat PRIVATE Threads::Threads)

add_executable(formula_1 ${OSDK_LIB_SRCS}
${CMAKE_CURRENT_SOURCE_DIR}/src/test_formula/formula_1.cpp)
add_dependencies(formula_1 diablo_sdk_generate_messages_cpp)
target_link_libraries(formula_1 PRIVATE Threads::Threads)

add_executable(formula_2 ${OSDK_LIB_SRCS}
${CMAKE_CURRENT_SOURCE_DIR}/src/test_formula/formula_2.cpp)
add_dependencies(formula_2 diablo_sdk_generate_messages_cpp)
target_link_libraries(formula_2 PRIVATE Threads::Threads)

add_executable(formula_velocity ${OSDK_LIB_SRCS}
${CMAKE_CURRENT_SOURCE_DIR}/src/test_formula/formula_velocity.cpp)
add_dependencies(formula_velocity diablo_sdk_generate_messages_cpp)
target_link_libraries(formula_velocity PRIVATE Threads::Threads)

add_executable(test_base ${OSDK_LIB_SRCS}
${CMAKE_CURRENT_SOURCE_DIR}/src/read_and_control/test_base.cpp)
add_dependencies(test_base diablo_sdk_generate_messages_cpp)
target_link_libraries(test_base PRIVATE Threads::Threads)

add_executable(test_base_custom ${OSDK_LIB_SRCS}
${CMAKE_CURRENT_SOURCE_DIR}/src/read_and_control/test_base_custom.cpp)
add_dependencies(test_base_custom diablo_sdk_generate_messages_cpp)
target_link_libraries(test_base_custom PRIVATE Threads::Threads)

add_executable(testing ${OSDK_LIB_SRCS}
${CMAKE_CURRENT_SOURCE_DIR}/src/testing.cpp)
add_dependencies(testing diablo_sdk_generate_messages_cpp)
target_link_libraries(testing PRIVATE Threads::Threads)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/diablo_base.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/diablo_base_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_diablo_base.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
