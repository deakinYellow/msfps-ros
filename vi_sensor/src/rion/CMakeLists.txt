set( _projname "tl740" )

include_directories( 
    include
    ${CMAKE_CURRENT_SOURCE_DIR}
    )

list( APPEND ${_projname}_LINKS_LIBS
    # ${YAMLCPP_LIB}
    ${COMMON_LINK_LIBRARIES}
    pthread
    )

add_executable( tl740
    "../uart/uart_frame_M1.cc"
    "tl740_node.cc"
    )
target_link_libraries( tl740 ${${_projname}_LINKS_LIBS} )






