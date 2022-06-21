cmake_minimum_required(VERSION 2.8.3)
set( PRO_NAME "msfps_msgs" )
project( ${PRO_NAME} )

#set( CMAKE_BUILD_TYPE Release )
set(CMAKE_BUILD_TYPE Debug)
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

#message(STATUS "cmake workspace: ${CMAKE_SOURCE_DIR}")

#==============ros package======================================================
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  #sensor_msgs
  #cv_bridge
  #image_transport
)

add_message_files(
  FILES
  UWB_TagDistance.msg
  UWB_TagPosition.msg
  Navigation.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  #LIBRARIES  issa_msg
  CATKIN_DEPENDS roscpp message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
