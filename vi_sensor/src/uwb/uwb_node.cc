/*************************************************************************
  > File Name: uwb_node.cpp
	> Author:Deakin
	> Mail: deakinhwang@163.com
  > Created Time:
 ************************************************************************/
#include <iostream>
#include "uwb_node.hpp"

int main( int argc, char **argv ) {
    //ROS_INFO("uart test.");
    //muart::uart_frameH_test();
    //qyxkuwb_test();

#if 1
    std::string node_name="uwb";
    //注意名称保持一致，涉及参数加载
    ros::init( argc, argv, node_name );
    ros::console::set_logger_level( ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info );
    UWB_Node uwb_node( node_name );
    if( uwb_node.Start() ){
        uwb_node.MainLoop();
    }
    ROS_INFO("uwb node terminal.");
#endif
    return 0;
}


