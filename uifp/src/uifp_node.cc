
#include <iostream>
#include <memory>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>

#define ENABLE_MLOGD
#include "../../common/utility-cc/tool.hpp"
#include "../../common/utility-cc/stool.hpp"

#include "../../mros/parameter.hpp"
#include "nlink_parser/LinktrackNodeframe2.h"

#include "uidata_receive.h"
#include "yaw_tracking.h"

#include "params_load.h"
#include "uifp.h"

//TODO: 1.两标签时间同步问题

#define IMU_XAIS_TAG_ANGLE_DEVIATION  M_PI/2  /////由yawtracking得出的angle和IMU的X轴相差PI/2(取决于实际安装情况)
/////获取IMU X轴对应在map中的航向角
double get_imu_xais_yaw( const double tag_angle ){
  return uifp::TransformToCyclePi( tag_angle + IMU_XAIS_TAG_ANGLE_DEVIATION );
}

/////获取经过yaw修正后的IMU数据姿态
Eigen::Quaterniond get_imu_orientation_fix_yaw( const Eigen::Quaterniond& q, const double yaw ){
  Eigen::Vector3d ypr = meigen::QuaterniondToRotationYPR_Y( q );
  ypr[ 0 ] = yaw;
  Eigen::Quaterniond orientation = meigen::RotationYPRToQuaterniond_Y( ypr );
  return  orientation;
}
/////获取经过yaw修正后的IMU数据
uifp::ImuDataT get_imu_data_fix_yaw( const uifp::ImuDataT& imu_data, const double yaw ){
  uifp::ImuDataT dst;
  dst = imu_data;
  dst.quaterniond = get_imu_orientation_fix_yaw( imu_data.quaterniond, yaw );
  return dst;
}


///将点转到雷达所在坐标系:仅测试，非标准做法
Eigen::Vector3d convert_to_radar_coordinate( const Eigen::Vector3d& position ){
  ///先求雷达坐标系到UWB坐标系的旋转
  double ru_degress = -1.8;
  Eigen::AngleAxisd ru_rotation_vector( meigen::DegreeToRadian<double>( ru_degress ), Eigen::Vector3d ( 0,0,1 ) );
  Eigen::Quaterniond ru_q( ru_rotation_vector );
  Eigen::Vector3d radar_position = meigen::Vector3dTransformByQuaterniond( position, ru_q );
  Eigen::Vector3d p_shift( -0.05 , 0.25, 0 );
  radar_position += p_shift;
  return radar_position;
}


/**
 * @brief
 * @param  [in]
 * @param  [out]
 * @retval
 * @note
 **/
int main(int argc, char **argv){
  if( argc < 2 ){  //若传入一个参数,这里argc=4
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
  std::string tag00_sub_topic,imu00_sub_topic;
  std::string tag01_sub_topic,imu01_sub_topic;

  mros::Parameter parameter( nh_p );
  parameter.Get<std::string>("tag00_sub_topic", tag00_sub_topic );
  parameter.Get<std::string>("imu00_sub_topic", imu00_sub_topic );
  parameter.Get<std::string>("tag01_sub_topic", tag01_sub_topic );
  parameter.Get<std::string>("imu01_sub_topic", imu01_sub_topic );

  ROS_INFO("tag00_sub_topic: %s", tag00_sub_topic.c_str() );
  ROS_INFO("tag01_sub_topic: %s", tag01_sub_topic.c_str() );
  ROS_INFO("imu00_sub_topic: %s", imu00_sub_topic.c_str() );
  ROS_INFO("imu01_sub_topic: %s", imu01_sub_topic.c_str() );

  //--------------------------Data Receive----------------------------------
  uifp::UiDataManager uidata_manager_00;
  uifp::UiDataReceive uidata_receive_00( nh_p, tag00_sub_topic, imu00_sub_topic ,&uidata_manager_00 );

  uifp::UiDataManager uidata_manager_01;
  uifp::UiDataReceive uidata_receive_01( nh_p, tag01_sub_topic, imu01_sub_topic ,&uidata_manager_01 );


  //--------------------------UIFP INIT-------------------------------------------
  bool is_visualization_track;
  int visualization_track_marks_max;
  std::string linktrack_position_track_pub_topic_00;
  std::string linktrack_position_track_pub_topic_01;
  std::string uifp_position_track_pub_topic_00;
  std::string uifp_position_track_pub_topic_01;

  std::string uifp_position_track_pub_topic_fusion;
  std::string uifp_pose2d_pub_topic;


  parameter.Get<bool>("is_visualization_track", is_visualization_track );
  parameter.Get<int>("visualization_track_marks_max", visualization_track_marks_max );

  parameter.Get<std::string>("linktrack_position_track_pub_topic_00", linktrack_position_track_pub_topic_00 );
  parameter.Get<std::string>("linktrack_position_track_pub_topic_01", linktrack_position_track_pub_topic_01 );

  parameter.Get<std::string>("uifp_position_track_pub_topic_00", uifp_position_track_pub_topic_00 );
  parameter.Get<std::string>("uifp_position_track_pub_topic_01", uifp_position_track_pub_topic_01 );

  parameter.Get<std::string>("uifp_position_track_pub_topic_fusion", uifp_position_track_pub_topic_fusion );
  parameter.Get<std::string>("uifp_pose2d_pub_topic", uifp_pose2d_pub_topic );


  uifp::AnchorsMapT anchors_map;
  load_anchors_map( &parameter, anchors_map );

  uifp::UiFp uifp00( node_name );
  uifp::UiFp uifp01( node_name );


  std::shared_ptr<mros::VisualizationMarkerPublisher> uifp_marker_publisher;
  std::shared_ptr<mros::TF> uifp_tf;



  //----------------------VISUAL-----------------------------------
  if( is_visualization_track ){
#if 1
    mros::VisualizationMarkerPubParametersT lintrack_visual_params_00=
    mros::VisualizationMarkerPubParameters( linktrack_position_track_pub_topic_00,10,
                                            "map",1,
                                            mros::VISUALIZATION_MARKER_COLOR_BLUE, mros::VISUALIZATION_MARKER_SCALE_01,
                                            visualization_track_marks_max );
    uifp00.init_lintrack_visualization( lintrack_visual_params_00 );

    mros::VisualizationMarkerPubParametersT uifp_visual_params_00=

    mros::VisualizationMarkerPubParameters( uifp_position_track_pub_topic_00,20,
                                            "map",2,
                                            mros::VISUALIZATION_MARKER_COLOR_GREEN, mros::VISUALIZATION_MARKER_SCALE_01,
                                            visualization_track_marks_max );
    uifp00.init_uifp_visualization( uifp_visual_params_00, "tag00" );

    mros::VisualizationMarkerPubParametersT lintrack_visual_params_01=
    mros::VisualizationMarkerPubParameters( linktrack_position_track_pub_topic_01,10,
                                            "map",3,
                                            mros::VISUALIZATION_MARKER_COLOR_VIOLET, mros::VISUALIZATION_MARKER_SCALE_01,
                                            visualization_track_marks_max );
    uifp01.init_lintrack_visualization( lintrack_visual_params_01 );

    mros::VisualizationMarkerPubParametersT uifp_visual_params_01=
    mros::VisualizationMarkerPubParameters( uifp_position_track_pub_topic_01,20,
                                            "map",4,
                                            mros::VISUALIZATION_MARKER_COLOR_YELLOW, mros::VISUALIZATION_MARKER_SCALE_01,
                                            visualization_track_marks_max );

    uifp01.init_uifp_visualization( uifp_visual_params_01, "tag01" );

    //两标签融合位姿发布
    uifp_marker_publisher = std::shared_ptr<mros::VisualizationMarkerPublisher>( new mros::VisualizationMarkerPublisher( nh_p ) );
    mros::VisualizationMarkerPubParametersT uifp_visual_params_fusion=
    mros::VisualizationMarkerPubParameters( uifp_position_track_pub_topic_fusion,20,
                                            "map",5,
                                            mros::VISUALIZATION_MARKER_COLOR_WHITE, mros::VISUALIZATION_MARKER_SCALE_01,
                                            visualization_track_marks_max );
    uifp_marker_publisher->Init( uifp_visual_params_fusion );
    uifp_tf = std::shared_ptr<mros::TF>( new mros::TF("map") );


#endif

  }
  //----------------------------------------------------------------
  uifp::AnchorsFilterSimpleTypeParamsT anchors_filter_simple_type_params={0,20};
  uifp::AnchorsFilterRssiTypeParamsT   anchors_filter_rssi_type_params={0,20,1.0,8.0};
#if 0
  uifp00.init_anchors_filter( anchors_map ,
                              uifp::ANCHORS_FILTER_TYPE_SIMPLE_TURNS,
                             (void*)(&anchors_filter_simple_type_params) );
  uifp01.init_anchors_filter( anchors_map ,
                              uifp::ANCHORS_FILTER_TYPE_SIMPLE_TURNS,
                             (void*)(&anchors_filter_simple_type_params) );
#else
  uifp00.init_anchors_filter( anchors_map ,
                              uifp::ANCHORS_FILTER_TYPE_RSSI_TURNS,
                             (void*)(&anchors_filter_rssi_type_params) );
  uifp01.init_anchors_filter( anchors_map ,
                              uifp::ANCHORS_FILTER_TYPE_RSSI_TURNS,
                             (void*)(&anchors_filter_rssi_type_params) );
#endif


  //---------------------EKF------------------------------------------------
  Eigen::Vector3d initial_position(0,0,0);
  uifp::KfParametersT kf_params;
  uifp::DynamicThresholdParametersT distance_innovation_dynamic_threshold_params;
  uifp::KfStatusCheckParametersT kf_status_check_params;
  load_kf_params( &parameter,
                  initial_position,
                  kf_params,
                  distance_innovation_dynamic_threshold_params,
                  kf_status_check_params );

  uifp00.init_ekf( initial_position, kf_params,
                   distance_innovation_dynamic_threshold_params,
                   kf_status_check_params );

  uifp01.init_ekf( initial_position, kf_params,
                   distance_innovation_dynamic_threshold_params,
                   kf_status_check_params );

  //-------------------------------------------------------------------------

  //--------------------------UIFP END-------------------------------------------
  uifp::MultiAnchorsTagDataT  tag00_data;
  uifp::MultiAnchorsTagDataT  tag01_data;
  uifp::ImuDataT  imu00_data;
  uifp::ImuDataT  imu01_data;


#define TAG00  0
#define TAG01  0
#define FUSION  1

  uifp::YawTracking yaw_tracking;
  uifp::KfStatusCheckParametersT yaw_status_check_parms = { 100, 0.001 };
  yaw_tracking.set_parameters( yaw_status_check_parms );
  yaw_tracking.set_workmode( uifp::YAWTRACKING_WORKMODE_INITIALIZE );

  ros::Publisher yaw_publisher = nh.advertise<std_msgs::Float32>( "fusion_yaw" , 2 );
  std_msgs::Float32 yaw_msg;

  double current_yaw = 0;
  bool   current_yaw_initialized = false;

  ///pose2d publisher
  ros::Publisher pose2d_publisher = nh.advertise<geometry_msgs::Pose2D>( uifp_pose2d_pub_topic, 2 );

  while( ros::ok() ){
    //MLOGD("uifp main loop.");
    //---------------------uifp00--------------------------------
    //MLOGD("data_status_01: %d ", data_status_01 );
    //先获取数据
    if( MSUCCESS == uidata_manager_00.get_uidata( tag00_data, imu00_data, false )
        && MSUCCESS == uidata_manager_01.get_uidata( tag01_data, imu01_data, false ) ){
      uidata_manager_00.clear_top();
      uidata_manager_01.clear_top();

      ////tag之间可能存在延迟，导致定位信息不同步
      double dt = tag00_data.timestamp - tag01_data.timestamp;
      if( fabs( dt ) > 0.02 ){
        MLOGW("00 01 tag_data dt too large: %f s", dt );
      }
      else {
        Eigen::Vector3d tag00_p;
        Eigen::Vector3d tag01_p;
        ////先进行静态初始化,初步获取航向角,不引入IMU
        if( !current_yaw_initialized ){
          //进行静态更新
          MLOGI("----------------s tag00--------------------------\n");
          uifp00.stationary_update( tag00_data);
          MLOGI("----------------s tag01--------------------------\n");
          uifp01.stationary_update( tag01_data);
          //稳定后输出位置,并开始初始化航线角
          if( uifp00.get_status() == uifp::UIFP_STATUS_STABLE
            && uifp01.get_status() == uifp::UIFP_STATUS_STABLE ){
            tag00_p = uifp00.get_position();
            tag01_p = uifp01.get_position();
            MLOGI("----------------yaw initlization--------------------------\n");
            if( MSUCCESS == yaw_tracking.update( tag00_p, tag01_p, imu00_data.quaterniond, imu01_data.quaterniond ) ){
              current_yaw = get_imu_xais_yaw( yaw_tracking.get_yaw() );
              double yaw_degress = meigen::RadianToDegree<double>( current_yaw );
              yaw_msg.data = float_t( yaw_degress );
              yaw_publisher.publish( yaw_msg );
              if( yaw_tracking.get_status() == uifp::YAWTRACKING_STATUS_QUICK_TRACKING ){
                current_yaw_initialized = true;
              }
            }
          }
        }
        ////动态更新,引入IMU
        else {
          //获取修正航向角的IMU数据
          MLOGI("----------------m tag00--------------------------\n");
          uifp00.moving_update( tag00_data, get_imu_data_fix_yaw( imu00_data, current_yaw ) );
          MLOGI("----------------m tag01--------------------------\n");
          uifp01.moving_update( tag01_data, get_imu_data_fix_yaw( imu01_data, current_yaw ) );
          tag00_p = uifp00.get_position();
          tag01_p = uifp01.get_position();
          MLOGI("----------------yaw update--------------------------\n");
          if( MSUCCESS == yaw_tracking.update( tag00_p, tag01_p, imu00_data.quaterniond, imu01_data.quaterniond ) ){
            current_yaw = get_imu_xais_yaw( yaw_tracking.get_yaw() );
            MLOGI("current_yaw: %f ", meigen::RadianToDegree<double>(current_yaw ));

            double yaw_degress = meigen::RadianToDegree<double>( current_yaw );
            yaw_msg.data = float_t( yaw_degress );
            yaw_publisher.publish( yaw_msg );

            Eigen::Vector3d fusion_p = ( tag00_p + tag01_p ) / 2;
            Eigen::Vector3d radar_coordinate_fusion_p = convert_to_radar_coordinate( fusion_p );
            const double yaw_deviation = 0.0;
            double radar_coordinate_yaw = current_yaw +
                meigen::DegreeToRadian<double>( yaw_deviation ) + M_PI;
            geometry_msgs::Pose2D msg_pose2d =  mros::data_type_conversion::GetGeometryPose2D(
                  radar_coordinate_fusion_p.x(),
                  radar_coordinate_fusion_p.y(),
                  radar_coordinate_yaw );

            //publish for radar coordinate
            pose2d_publisher.publish( msg_pose2d );

            ///publish for rviz
            uifp_tf->SendTransform("f-tag", mros::data_type_conversion::GetGeometryPose2D(
                                     msg_pose2d.x, msg_pose2d.y, msg_pose2d.theta - M_PI ) );
            geometry_msgs::Point msg_point = mros::data_type_conversion::GetGeometryPoint( msg_pose2d.x, msg_pose2d.y, 0);
            uifp_marker_publisher->AddPoint( msg_point );

          }
        }
      }
    }
    //----------ros spin--------------------------
    ros::spinOnce();
    M_MSLEEP(1);
  }
  return 0;
}
