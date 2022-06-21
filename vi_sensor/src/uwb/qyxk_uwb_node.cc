/*************************************************************************
  > File Name: uwb_node.cpp
	> Author:Deakin
	> Mail: deakinhwang@163.com
  > Created Time:
 ************************************************************************/

#include <iostream>
//#include <map>

#include <ros/ros.h>
//#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point.h>
#include "std_msgs/UInt16.h"

#include "../uwb/qyxk_uwb.hpp"

#include "../../common/utility-cc/tool.hpp"
#include "../../common/utility-math/kalmanfilter.hpp"

//#include "common_msgs/UWB_FullRangeInfo.h"
#include "msfps_msgs/UWB_TagDistance.h"

typedef enum ANCHOR_LIST{
    ANCHOR_NO_01 = 12377,
    ANCHOR_NO_02 = 12401,
    ANCHOR_NO_03 = 12349,
    ANCHOR_NO_04 = 12382,
}ANCHOR_LIST;


///(x,y,z)单位m
void pub_tag_position( const ros::Publisher& pub, const double x, const double y, const double z ){
  //ROS_位m
  ROS_INFO("point publisher");
  geometry_msgs::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  pub.publish( point );
}

///距离偏差设置为100mm左右
void pub_tag_distance( const ros::Publisher& pub, uint anchorid, uint distance_mm, uint16_t distance_err ){
  /*
  //ROS_INFO("distances publisher");
  common_msgs::UWB_FullRangeInfo range;

  static uint16_t msg_id  = 0;

  range.msgId  = msg_id++;
  range.header.frame_id = "/map";
  range.header.stamp = ros::Time::now();

  range.responderId = anchorid;
  range.precisionRangeErrEst = distance_err;
  range.precisionRangeMm = distance_mm;
  pub.publish( range );
  */
}


int main( int argc, char **argv ) {
  //muart::uart_frameH_test();
  //qyxkuwb_test();

  ros::init( argc, argv,"qyxk_uwb");
  ros::NodeHandle nh;

  ros::Publisher tag_position_publisher = nh.advertise<geometry_msgs::Point>("qyxk_uwb/tag_position", 10 );
  //ros::Publisher tag_distance_publisher = nh.advertise<common_msgs::UWB_FullRangeInfo>("qyxk_uwb/tag_distance", 10 );
  ros::Publisher tag_d1_publisher = nh.advertise<std_msgs::UInt16>("qyxk_uwb/tag_d1", 10 );
  std_msgs::UInt16 d1;

  std::shared_ptr<QyxkUwb> uwb = std::shared_ptr<QyxkUwb>( new QyxkUwb("/dev/uwb", 115200 ) );
  uwb->StartProcess();

  QyxkUwb::UwbTagPositionT p;
  QyxkUwb::UwbTagPositionsBuffT* ps;
  ps = uwb->tag_positions_buff;

  QyxkUwb::UwbTagDistanceT d;
  QyxkUwb::UwbTagDistancesBuffT* ds;
  ds = uwb->tag_distances_buff;
  uint p_count = 0;
  uint d_count = 0;

  stool::FPS fps_d;
  //循环频率设置为1000HZ
  ros::Rate  loop_rate(1000);
  while ( ros::ok() ) {
    if( ps ){
      std::unique_lock<std::mutex> ps_lock( ps->mutex );   //线程数据同步，重要！
      if( !ps->data.empty() ){
        p = ps->data.front() ;
        //printf("%d position: %d %d %d \n", ++p_count, p->px, p->py, p->pz );
        //原始数据单位cm 转为 m 故/100
        //publishTagPosition( tag_position_publisher_ , p.px / 100.0 , p.py / 100.0 , p.pz / 100.0 );
        //printf("%d position: %d %d %d \n", ++p_count, p->px, p->py, p->pz );
        pub_tag_position( tag_position_publisher , p.px / 100.0 , p.py / 100.0 , p.pz / 100.0 );
        ps->data.pop();

      }
    }
    if( ds ){
      std::unique_lock<std::mutex> ds_lock( ds->mutex );   //线程数据同步，重要！
      if( !ds->data.empty() ){
        d = ds->data.front() ;
        //printf("%d: tag %d to anchor %d distance: %d cm\n", ++d_count, d->tagid, d->anchorid, d->distance );
        fps_d.sampling();

        //ROS_INFO("FPS, normal: %d  avr: %.2f  real: %.2f", fps_d.fps_normal, fps_d.fps_average, fps_d.fps_real );
        //printf("%d: timestamp %ld \n", ++d_count, d->timestamp );
        //publishTagDistance( tag_distance_publisher_ ,d.tagid, d.anchorid , double( d.distance )/ 100.0 , 0.05 );
        //printf("%d: timestamp %ld \n", ++d_count, d->timestamp );

        //pub_tag_distance( tag_distance_publisher , d.anchorid , d.distance * 10 , 50 );
        ds->data.pop();
      }
    }
    //ROS_INFO("ros loop.");
    loop_rate.sleep();
    ros::spinOnce();  //重要,回调函数必须
  }
  ROS_INFO("qyxk uwb terminal.");
  return 0;
}

