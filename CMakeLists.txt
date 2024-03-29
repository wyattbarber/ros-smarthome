cmake_minimum_required(VERSION 2.8.3)
project(smarthome)

## Find dependent packages
find_package(catkin REQUIRED 
                COMPONENTS roscpp rospy message_generation std_msgs)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################
add_message_files(
  FILES
  Device.msg
  Devices.msg
)
add_service_files(
  FILES
  Register.srv
  Sync.srv
  Execute.srv
  Query.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/smarthome.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


## Build device_manager.cpp
add_executable(${PROJECT_NAME}_device_manager src/device_manager.cpp)
set_target_properties(${PROJECT_NAME}_device_manager PROPERTIES OUTPUT_NAME device_manager PREFIX "")
target_link_libraries(${PROJECT_NAME}_device_manager ${catkin_LIBRARIES})

## Build simple_light.cpp
add_executable(${PROJECT_NAME}_simple_light src/simple_light.cpp)
set_target_properties(${PROJECT_NAME}_simple_light PROPERTIES OUTPUT_NAME simple_light PREFIX "")
target_link_libraries(${PROJECT_NAME}_simple_light ${catkin_LIBRARIES})

#############
## Install ##
#############

## Install NodeJS scripts
install(PROGRAMS
  src/gateway.js
  ${PROJECT_NAME}_device_manager
  ${PROJECT_NAME}_simple_light
 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


