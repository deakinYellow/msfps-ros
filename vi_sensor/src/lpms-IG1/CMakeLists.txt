set( _projname "lpms_IG1" )

if( ${PLATFORM} MATCHES "x86_64-linux-gnu" )
    #set( OPENZEN_INCLUDE_DIRECTORIES "/opt/x86_64-linux-gnu/openzen/include" )
    #set( OPENZEN_LIB "/opt/x86_64-linux-gnu/openzen/lib/libOpenZen.so" )
    #本地编译生成
    set( OPENZEN_INCLUDE_DIRECTORIES "/opt/openzen/include" )
    set( OPENZEN_LIB "/opt/openzen/lib/libOpenZen.so" )
elseif( ${PLATFORM} MATCHES "aarch64-linux-gnu" )
#    message( STATUS "matches aarch64")
    set( OPENZEN_INCLUDE_DIRECTORIES "/opt/aarch64-linux-gnu/openzen/include" )
    set( OPENZEN_LIB "/opt/aarch64-linux-gnu/openzen/lib/libOpenZen.so" )
else()
    message( FATAL_ERROR "***>>>platfrom no found" )
endif()

link_directories(${OPENZEN_INCLUDE_DIRECTORIES})

include_directories( include
    ${OPENZEN_INCLUDE_DIRECTORIES}
    )

list( APPEND ${_projname}_LINKS_LIBS
    ${COMMON_LINK_LIBRARIES}
    ${OPENZEN_LIB}
    Threads::Threads
    dl
    rt
    )

add_executable( ${_projname}
    "lpms_IG1_node.cc"
    )

target_link_libraries( ${_projname} ${${_projname}_LINKS_LIBS} )

