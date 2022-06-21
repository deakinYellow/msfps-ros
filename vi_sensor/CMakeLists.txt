cmake_minimum_required(VERSION 2.8.3)
project(vi_sensor)

set(CMAKE_BUILD_TYPE "Release")
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

add_definitions(-DENABLE_MLOGD)

#//选择编译平台,默认x86_64
option( PLATFORM "selete form" x86_64-linux-gnu )
message( STATUS "***>>>current platform: ${PLATFORM}" )
#message( STATUS "***>>>current system: ${CMAKE_SYSTEM}")
#message( STATUS "***>>>current FRAMEWORK: $ENV{ULS_PLATFORM}" )
#if( "$ENV{ULS_PLATFORM}" MATCHES "AARCH64" )
#message( STATUS "***>>>AAAAAAAAAAAAAAAAAAAAA" )
#endif()


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  visualization_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
#  common_msgs
  msfps_msgs
# cv_bridge
# rospy
# tf
# tf_conversions
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs
)

find_package( Threads REQUIRED )
set( EIGEN3_DIR "/usr/include/eigen3" )
#message( FATAL_ERROR "***>>>stop build.")

include_directories( include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_DIR}
    "${PROJECT_SOURCE_DIR}/include"
    "${CMAKE_SOURCE_DIR}/common"
    ${OPENZEN_INCLUDE_DIRECTORIES}
    )

LIST( APPEND COMMON_LINK_LIBRARIES
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    )

#=========add subdirectories===============================
#add_subdirectory(src/lpms-ME1)
add_subdirectory(src/lpms-IG1)
#add_subdirectory(src/rion)
add_subdirectory(src/uwb)



