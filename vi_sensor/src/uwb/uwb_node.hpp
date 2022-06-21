#ifndef UWB_NODE_HPP
#define UWB_NODE_HPP

#include <iostream>
#include <map>
#include <ros/ros.h>

#include "../../common/utility-cc/tool.hpp"
#include "../../mros/node_handle.hpp"
#include "../../mros/parameter.hpp"

#include "msfps_msgs/UWB_TagDistance.h"
#include "msfps_msgs/UWB_TagPosition.h"

#include "qyxk_uwb.hpp"


class UWB_Node : public mros::NodeHandle {

public:
  explicit UWB_Node( std::string name_space ) : mros::NodeHandle( name_space ) {
    ;
  }

  bool Start( void ){

    std::shared_ptr<mros::Parameter> parameter
        = std::shared_ptr<mros::Parameter>( new mros::Parameter( nh ) );
    loadParams( parameter );
    uwb_ = std::shared_ptr<QyxkUwb>( new QyxkUwb( dev_id_ , uint( dev_baudrate_ ) ) );

    if( !uwb_ ){
      return false;
    }
    uwb_->StartProcess();
    tag_distance_publisher_ = nh->advertise<msfps_msgs::UWB_TagDistance>( tag_distance_topic_, 10 );
    tag_position_publisher_ = nh->advertise<msfps_msgs::UWB_TagPosition>( tag_position_topic_, 10 );
    return true;
  }

  void MainLoop( void ){

    QyxkUwb::UwbTagPositionT p;
    QyxkUwb::UwbTagPositionsBuffT* ps;
    ps = uwb_->tag_positions_buff;

    QyxkUwb::UwbTagDistanceT d;
    QyxkUwb::UwbTagDistancesBuffT* ds;
    ds = uwb_->tag_distances_buff;

    //uint p_count = 0;
    //uint d_count = 0;

    stool::FPS fps_d;

    //ros::Rate  loop_rate(1000);

    double last_distance_stamp = 0;
    double last_position_stamp = 0;
    ROS_INFO("uwb test 00!");

    while ( nh->ok() ) {
      //ROS_DEBUG("debug 1.");
      if( is_pub_position_ && ps ){
        std::unique_lock<std::mutex> ps_lock( ps->mutex );   //线程数据同步，重要！
        if( !ps->data.empty() ){
          p = ps->data.front() ;
          //ROS_DEBUG("debug 2.");
          //printf("%d position: %d %d %d \n", ++p_count, p->px, p->py, p->pz );
          //原始数据单位cm 转为 m 故/100
          publishTagPosition( tag_position_publisher_ , p.px / 100.0 , p.py / 100.0 , p.pz / 100.0 );
          //ROS_DEBUG("debug 3.");
          ps->data.pop();
          //ROS_DEBUG("debug 4.");
          ////test-------------------------
          double stamp = tool::sys_ms_timestamp();
          double dt = stamp - last_position_stamp;
          //ROS_INFO("uwb position data dt: %f ", dt );
          if( dt > 200 ){
            //ROS_WARN("uwb position data dt too large: %f ", dt );
            MLOGW("uwb position data dt too large: %f ", dt );
          }
          last_position_stamp = stamp;
        }
      }
      if( is_pub_distance_ && ds ){
        std::unique_lock<std::mutex> ds_lock( ds->mutex );   //线程数据同步，重要！
        if( !ds->data.empty() ){
          //ROS_DEBUG("debug 5.");
          d = ds->data.front() ;
          //printf("%d: tag %d to anchor %d distance: %d cm\n", ++d_count, d->tagid, d->anchorid, d->distance );
          fps_d.sampling();
          //ROS_DEBUG("debug 6.");
          //ROS_INFO("FPS, normal: %d  avr: %.2f  real: %.2f", fps_d.fps_normal, fps_d.fps_average, fps_d.fps_real );
          //printf("%d: timestamp %ld \n", ++d_count, d->timestamp );
          publishTagDistance( tag_distance_publisher_ ,d.tagid, d.anchorid , double( d.distance )/ 100.0 , 0.05 );
          //ROS_DEBUG("debug 7.");
          ds->data.pop();
          ////test-------------------------
          double stamp = tool::sys_ms_timestamp();
          double dt = stamp - last_distance_stamp;
          //ROS_INFO("uwb distance data dt: %f ", dt );
          if( dt > 200 ){
            //ROS_WARN("uwb distance data dt too large: %f ", dt );
            MLOGW("uwb distance data dt too large: %f ", dt );
          }
          last_distance_stamp = stamp;
        }
      }
      //loop_rate.sleep();
      M_USLEEP(1000);
      //ros::spinOnce();  //回调函数处理,若含有回调函数必须加
    }
  }
  ~UWB_Node(){
    ;
  }

private:

  //=================Params=========================
  std::string dev_id_;
  int dev_baudrate_;

  bool is_pub_distance_;
  bool is_pub_position_;
  int  tag_id_;

  std::string tag_distance_topic_;
  std::string tag_position_topic_;
  ros::Publisher tag_position_publisher_;
  ros::Publisher tag_distance_publisher_;

  void loadParams( const std::shared_ptr<mros::Parameter> parameter ){
    if( !parameter ){
      ROS_WARN("loadParams fail, nullptr!");
      return;
    }
    parameter->Get<std::string>("dev_id",dev_id_, "/dev/uwb" );
    parameter->Get<int>("dev_baudrate_", dev_baudrate_, 115200 );

    parameter->Get<std::string>("tag_distance_topic", tag_distance_topic_,"/qyxk_uwb/tag_distance" );
    parameter->Get<std::string>("tag_position_topic", tag_position_topic_, "/qyxk_uwb/tag_position" );

    parameter->Get<bool>("is_pub_distance", is_pub_distance_, true );
    parameter->Get<bool>("is_pub_position", is_pub_position_, true );

    parameter->Get<int>("tag_id", tag_id_, 51391  );

  }
  //==========================================
  std::shared_ptr<QyxkUwb> uwb_ = nullptr;

  ///发布位置信息(x,y,z)单位m
  void publishTagPosition( const ros::Publisher& publisher, const double x, const double y, const double z ){
    //ROS_INFO("point publisher");
    static uint msg_id = 0;
    static msfps_msgs::UWB_TagPosition msg;

    msg.header.stamp = ros::Time::now();
    msg.msg_id = msg_id++;
    msg.p.x = x;
    msg.p.y = y;
    msg.p.z = z;
    publisher.publish( msg );
  }

  ///发布距离信息(distance,distance_err)单位m
  void publishTagDistance( const ros::Publisher& publisher,
                           const uint16_t tag_id,
                           const uint16_t anchor_id,
                           const double distance,
                           const double distance_err ){
    //ROS_INFO("point publisher");
    static uint msg_id = 0;
    static msfps_msgs::UWB_TagDistance msg;

    msg.header.stamp = ros::Time::now();
    msg.msg_id = msg_id++;
    msg.tag_id = tag_id;
    msg.anchor_id = anchor_id;
    msg.distance = distance;
    msg.distance_err = distance_err;

    publisher.publish( msg );
  }

};

#endif
