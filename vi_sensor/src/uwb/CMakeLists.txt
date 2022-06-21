set( _projname "qyxk_uwb" )

include_directories( 
    include
    ${CMAKE_CURRENT_SOURCE_DIR}
    )

list( APPEND ${_projname}_LINKS_LIBS
    # ${YAMLCPP_LIB}
    ${COMMON_LINK_LIBRARIES}
    pthread
#    dl
#    rt
#    LpSensor.so
    )

#==============可选 测试＝＝＝＝＝＝＝＝＝＝＝＝＝＝
add_executable( ${_projname}
    #"../uart/uart_frame.cc"
    "qyxk_uwb_node.cc"
    )
target_link_libraries( ${_projname} ${${_projname}_LINKS_LIBS} )

add_executable( ${_projname}_hand
    "qyxk_uwb_hand_node.cc"
    )
target_link_libraries( ${_projname}_hand ${${_projname}_LINKS_LIBS} )
#==============可选＝＝＝＝＝＝＝＝＝＝＝＝＝＝

add_executable( uwb
#    "../uart/uart_frame.cc"
    "../uart/uart_frame_M0.cc"
    "uwb_node.cc"
    )
target_link_libraries( uwb ${${_projname}_LINKS_LIBS} )
