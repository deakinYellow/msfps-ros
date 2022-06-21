cmake_minimum_required(VERSION 3.0.2)
project(uifp)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  sensor_msgs
  msfps_msgs
  nlink_parser
)

set( Eigen3_DIR "/usr/include/eigen3" )
include_directories(
  #include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_DIR}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  )

LIST( APPEND COMMON_LINK_LIBRARIES
  ${catkin_LIBRARIES}
  )

set( UIFPS_CREATE ${PROJECT_NAME})

add_executable( ${UIFPS_CREATE}
  "src/uidata_manager.cc"
  "src/uidata_receive.cc"
  "src/anchors_filter.cc"
  "src/uifp_ekf.cc"
  "src/uifp.cc"
  "src/yawfusion_kf.cc"
  "src/yaw_tracking.cc"
  "src/params_load.cc"
  "src/uifp_node.cc"
  )
target_link_libraries( ${UIFPS_CREATE} ${COMMON_LINK_LIBRARIES} )


add_executable( "radar"
  "src/radar_node.cc"
  )
target_link_libraries( radar ${COMMON_LINK_LIBRARIES} )
