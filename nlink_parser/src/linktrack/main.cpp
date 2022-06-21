#include <ros/ros.h>

#include "init.h"
#include "init_serial.h"
#include "protocol_extracter/nprotocol_extracter.h"

int main(int argc, char **argv) {
  if( argc < 2 ){  //若传入一个参数,这里argc=4
    printf("please enter special node name!");
    return EXIT_FAILURE;
  }
  //initialize ros
  std::string node_name=argv[1];
  ros::init(argc, argv, node_name );
  ros::NodeHandle nh( node_name );
  ros::NodeHandlePtr nh_p= ros::NodeHandlePtr( &nh );
  serial::Serial serial;
  initSerial(&serial);
  NProtocolExtracter protocol_extraction;
  linktrack::Init init(&protocol_extraction, &serial, nh_p );
  ros::Rate loop_rate(1000);
  while (ros::ok()) {
    auto available_bytes = serial.available();
    std::string str_received;
    if (available_bytes) {
      serial.read(str_received, available_bytes);
      protocol_extraction.AddNewData(str_received);
    }
    ros::spinOnce();
    //ROS_INFO("nlink parser mainloop.");
    loop_rate.sleep();
  }
  return EXIT_SUCCESS;
}



