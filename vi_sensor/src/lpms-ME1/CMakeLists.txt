set( _projname "lpms_imu" )

include_directories( 
    include
    ${CMAKE_CURRENT_SOURCE_DIR}
    )

list( APPEND ${_projname}_LINKS_LIBS
    # ${YAMLCPP_LIB}
    ${COMMON_LINK_LIBRARIES}
    pthread
    dl
    rt
    LpSensor.so
    )

add_executable( ${_projname}
    "lpms_node.cc"
    )
target_link_libraries( ${_projname} ${${_projname}_LINKS_LIBS} )








