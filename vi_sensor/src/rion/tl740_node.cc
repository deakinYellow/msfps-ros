/*************************************************************************
  > File Name: uwb_node.cpp
	> Author:Deakin
	> Mail: deakinhwang@163.com
  > Created Time:
 ************************************************************************/
#include <iostream>

#include "../uart/uart_frame_M1.h"
#include "tl740.hpp"

#include "tl740_node.hpp"

#include "../../common/utility-cc/mthread.hpp"


int main( int argc, char **argv ) {
    ROS_INFO("tl740 node start.");
#if 0
    //tl740_test();
    //muart::uart_frameM1H_test();
    //MThreadTest();
#else
    std::string node_name="tl740";
    //注意名称保持一致，涉及参数加载
    ros::init( argc, argv, node_name );
    ros::console::set_logger_level( ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info );
    TL740Node tl740_node( node_name );
    if( tl740_node.Start() ){
        tl740_node.MainLoop();
    }
    ROS_INFO("tl740 node terminal.");
#endif
    return 0;
}










