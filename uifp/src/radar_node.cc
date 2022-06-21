#include <iostream>
#include <memory>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>

#define ENABLE_MLOGD
#include "../../common/utility-cc/tool.hpp"
#include "../../common/utility-cc/stool.hpp"
#include "../../common/utility-math/meigen.hpp"

#include "../../mros/mros.h"
#include "../../mros/conversion.h"

static  mros::VisualizationMarkerPublisher* g_marker_publisher;
static  mros::TF* g_tf;

void radar_pose_callback( geometry_msgs::Pose2D::ConstPtr msg ){
  if( msg == nullptr ){
    MLOGW("invalid msg.");
    return;
  }
  ///[0-2PI] convert to [-PI,PI]
  double yaw = msg->theta - M_PI;

  MLOGI("radar pose2d data: position( %f %f ) yaw( %f )",
        msg->x, msg->y,
        meigen::RadianToDegree<double>( yaw ) );

  geometry_msgs::Point point;
  point.z = 0;
  point.x = msg->x;
  point.y = msg->y;
  g_marker_publisher->AddPoint( point );

  geometry_msgs::Pose2D tf_pose = *msg;
  tf_pose.theta = yaw;
  g_tf->SendTransform("radar", tf_pose );

}



/**
 * @brief  订阅激光雷达数据，并发布到rviz
 * @param  [in]
 * @param  [out]
 * @retval
 * @note
 **/
int main(int argc, char **argv){
  if( argc < 2 ){  //若传入一个参数,这里argc=4
    //MLOGE("please enter special node name!");
    printf("please enter special node name! \n");
  }
  //initialize ros
  std::string node_name=argv[1];
  MLOGD("node name: %s ", node_name.c_str() );
  ros::init( argc, argv, node_name );
  ros::NodeHandle nh( node_name );
  ros::NodeHandlePtr nh_p = ros::NodeHandlePtr( &nh );
  ros::console::set_logger_level( ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

  //--------------------------params----------------------------------
  std::string pose_data_sub_topic;
  std::string visual_tracking_pub_topic;
  int markers_max;

  mros::Parameter parameter( nh_p );
  parameter.Get<std::string>("pose_data_sub_topic", pose_data_sub_topic );

  parameter.Get<std::string>("visual_tracking_pub_topic", visual_tracking_pub_topic );
  parameter.Get<int>("markers_max", markers_max );

  MLOGI("pose subscriber topic: %s ", pose_data_sub_topic.c_str() );
  //--------------------------------------------------------------------

  ros::Subscriber pose_subcriber = nh_p->subscribe( pose_data_sub_topic, 2, radar_pose_callback );

  g_marker_publisher = new mros::VisualizationMarkerPublisher( nh_p );
  mros::VisualizationMarkerPubParametersT marker_params =
    mros::VisualizationMarkerPubParameters( visual_tracking_pub_topic,30,"map",1,
                                            mros::VISUALIZATION_MARKER_COLOR_RED,
                                            mros::VISUALIZATION_MARKER_SCALE_02,
                                            markers_max );
  g_marker_publisher->Init( marker_params );
  g_tf = new mros::TF("map");

  while( ros::ok() ){
    //MLOGD("radar node main loop.");
    ros::spinOnce();
    M_MSLEEP(1);
  }

  delete g_marker_publisher;
  delete g_tf;
  ros::shutdown();
  return 0;
}
