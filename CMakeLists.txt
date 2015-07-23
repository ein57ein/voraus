cmake_minimum_required(VERSION 2.8.3)
project(voraus)

set(CMAKE_BUILD_TYPE Debug) #-g
#set(CMAKE_BUILD_TYPE RelWithDebInfo) #-O2 -g

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  nodelet
  pluginlib
  roscpp
  std_msgs
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
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
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
   heartbeat.msg
   cpu_demo_ready.msg
   data_demo_ready.msg
#   Message1.msg
#   Message2.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   robot_registration.srv
   module_registration.srv
#   Service1.srv
#   Service2.srv
 )

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
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES voraus
  CATKIN_DEPENDS message_generation nodelet pluginlib roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(voraus
#   src/${PROJECT_NAME}/voraus.cpp
# )
add_library(distributed_plug_in include/voraus/distributed_plug_in.cpp)
add_library(centralPlugInLib src/voraus/central_plug_in.cpp)
add_library(decentralPlugInLib src/voraus/decentral_plug_in.cpp)

add_library(centralModManLib src/module_management/central_module_management.cpp)
add_library(decentralModManLib src/module_management/decentral_module_management.cpp)

add_library(_voraus_demo src/demo.cpp)

## Declare a cpp executable
# add_executable(voraus_node src/voraus_node.cpp)
add_executable(centralPlugIn src/central_plug_in_controller.cpp)
add_executable(decentralPlugIn src/decentral_plug_in_controller.cpp)

add_executable(centralModMan src/central_mod_man.cpp)
add_executable(decentralModMan src/decentral_mod_man.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(voraus_node voraus_generate_messages_cpp)
add_dependencies(distributed_plug_in voraus_generate_messages_cpp)
add_dependencies(centralPlugInLib voraus_generate_messages_cpp)
add_dependencies(decentralPlugInLib voraus_generate_messages_cpp)

add_dependencies(centralModManLib voraus_generate_messages_cpp)
add_dependencies(decentralModManLib voraus_generate_messages_cpp)

add_dependencies(_voraus_demo voraus_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(voraus_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(centralPlugInLib ${catkin_LIBRARIES})
target_link_libraries(decentralPlugInLib ${catkin_LIBRARIES})
target_link_libraries(centralPlugIn ${catkin_LIBRARIES} centralPlugInLib)
target_link_libraries(decentralPlugIn ${catkin_LIBRARIES} decentralPlugInLib)

target_link_libraries(centralModManLib ${catkin_LIBRARIES})
target_link_libraries(decentralModManLib ${catkin_LIBRARIES})
target_link_libraries(centralModMan ${catkin_LIBRARIES} centralModManLib)
target_link_libraries(decentralModMan ${catkin_LIBRARIES} decentralModManLib)

target_link_libraries(_voraus_demo ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS voraus voraus_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_voraus.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
