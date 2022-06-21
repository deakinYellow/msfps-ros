#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <iomanip>
#include <cmath>
#include <vector>
#include <thread>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#define OPENZEN_CXX14

#include "../../common/utility-cc/tool.hpp"
#include "../../common/utility-cc/stool.hpp"
#include "../../common/utility-math/meigen.hpp"
#include "../../mros/parameter.hpp"

#include "OpenZen.h"


using std::cout;
using std::endl;

static sensor_msgs::Imu g_imu_data;
static ros::Publisher g_imu_publisher;
static std_msgs::Float32 g_yaw_msg;
static ros::Publisher g_yaw_publisher;

static zen::ZenSensor *g_sensor;
static zen::ZenSensorComponent *g_imu;

#if 1
//note: LPMS-IG1 原始数据, 加速度单位为g ,陀螺仪单位为 deg / s
//ZenEventData_Imu imuData;
void publish_imu_data( const ZenEventData_Imu &data ){
  //fps_d.Sampling();
  //printf("FPS,normal: %d  avr: %.2f  real: %.2f \n", fps_d.fps_normal, fps_d.fps_average, fps_d.fps_real );

  g_imu_data.header.frame_id = "map";
  g_imu_data.header.stamp = ros::Time::now();

  //MLOGI("ros data timestamp: %f ", g_imu_data.header.stamp.toSec() );
  //MLOGI("imu data timestamp: %f ", data.timestamp );

  //直接取四元素在rviz中看发现方向相反,故用欧拉角输出;注意转化为弧度
  Eigen::Vector3d ypr( double( data.r[2] ), double( data.r[1] ) , double( data.r[0] ) );
  Eigen::Vector3d radian_ypr = meigen::DegreeToRadian<Eigen::Vector3d>( ypr  );
  Eigen::Quaterniond q = meigen::RotationYPRToQuaterniond( radian_ypr );
  g_imu_data.orientation.w = q.w();   // w for quaterniond
  g_imu_data.orientation.x = q.x();
  g_imu_data.orientation.y = q.y();
  g_imu_data.orientation.z = q.z();

  ///linear accleration单位g,不含重力加速度
  g_imu_data.linear_acceleration.x = double( data.linAcc[0] );
  g_imu_data.linear_acceleration.y = double( data.linAcc[1] );
  g_imu_data.linear_acceleration.z = double( data.linAcc[2] );
  //g_imu_data.linear_acceleration.x = data.aRaw[0];
  //g_imu_data.linear_acceleration.y = data.aRaw[1];
  //g_imu_data.linear_acceleration.z = data.aRaw[2];

  // Raw gyroscope sensor data. //单位degress
  g_imu_data.angular_velocity.x = double( data.g[ 0 ] );
  g_imu_data.angular_velocity.y = double( data.g[ 1 ] );
  g_imu_data.angular_velocity.z = double( data.g[ 2 ] );

  /*
  printf("linear_acceleration for x y z: %f %f %f \n",
         g_imu_data.linear_acceleration.x,
         g_imu_data.linear_acceleration.y,
         g_imu_data.linear_acceleration.z );
  printf("angular_velocity for x y z: %f %f %f \n",
         g_imu_data.angular_velocity.x,
         g_imu_data.angular_velocity.y,
         g_imu_data.angular_velocity.z );
  */

  //publish msg
  g_imu_publisher.publish( g_imu_data );

}
#endif

void keyboard_control(){
  //printf("keyboard control, input: %X \n", opt );
  char opt = 0x00;
  opt = stool::get_key();
  ///输入r方向复位
  if( opt == 'r' ){
    printf("keyboard control input: %X, set OrientationOffset!\n", opt );
    ///方向角清零
    int ret = g_imu->setInt32Property( ZenImuProperty_OrientationOffsetMode ,  ZenOrientationOffsetMode_Heading );
    printf("excute Orientation offset ret: %d  \n", ret );
  }

}


int main(int argc, char **argv){
  if( argc < 2 ){  //若传入一个参数,这里argc=4
    MLOGE_EXIT("please enter special node name!");
  }
  //initialize ros
  std::string node_name=argv[1];
  MLOGD("node name: %s ", node_name.c_str() );

  ros::init( argc, argv, node_name );
  ros::NodeHandle nh( node_name );

  //--------------------------params----------------------------------
  ros::NodeHandlePtr nh_p = ros::NodeHandlePtr( &nh );
  mros::Parameter parameter( nh_p );

  std::string  dev_type;
  std::string  dev_id;
  int samplingrate;
  bool gyr_auto_calibration;

  std::string  imu_data_pub_topic;
  std::string  yaw_pub_topic;

  parameter.Get<std::string>("dev_type", dev_type);
  parameter.Get<std::string>("dev_id", dev_id );
  parameter.Get<int>("samplingrate", samplingrate );
  parameter.Get<bool>("gyr_auto_calibration", gyr_auto_calibration );

  parameter.Get<std::string>("imu_data_pub_topic", imu_data_pub_topic );
  parameter.Get<std::string>("yaw_pub_topic", yaw_pub_topic );

  //--------------------------params----------------------------------
  std::cout<<"------------- [lpms] node starts. 100HZ -------------"<<std::endl;
  g_imu_publisher = nh.advertise<sensor_msgs::Imu>( imu_data_pub_topic , 1 );
  g_yaw_publisher = nh.advertise<std_msgs::Float32>( yaw_pub_topic , 1 );

  //---------------------init lpms IMU--------------------------------
  //ImuData lpms_imu_data;
  // enable resonable log output for OpenZen
  ZenSetLogLevel(ZenLogLevel_Info);

  // create OpenZen Clien
  auto clientPair = zen::make_client();
  auto& clientError = clientPair.first;
  auto& client = clientPair.second;

  if (clientError) {
    std::cout << "Cannot create OpenZen client" << std::endl;
    return clientError;
  }

  //connect to sensor on IO System by the sensor name
  //auto sensorPair = client.obtainSensorByName("LinuxDevice", "IG1232000470");
  MLOGI("dev_type:　%s , ID: %s ", dev_type.c_str(), dev_id.c_str() );
  auto sensorPair = client.obtainSensorByName( dev_type , dev_id );

  auto& obtainError = sensorPair.first;
  auto& sensor = sensorPair.second;   //用于访问传感器属性
  g_sensor = &sensor;

  if ( obtainError ) {
    std::cout << "Cannot connect to sensor" << std::endl;
    client.close();
    exit(1);
  }

  // check that the sensor has an IMU component
  auto imuPair = sensor.getAnyComponentOfType(g_zenSensorType_Imu);
  auto& hasImu = imuPair.first;
  auto& imu = imuPair.second;
  g_imu = &imu;

  if (!hasImu) {
    std::cout << "Connected sensor has no IMU" << std::endl;
    client.close();
    exit(1);
  }
  //打开连续数据流
  if ( 0  != imu.setBoolProperty( ZenImuProperty_StreamData, true ) ) {
    MLOGW("error, enable stream fail!\n.");
    client.close();
    exit(1);
  }

  ZenError ret;
  ///陀螺仪校准///执行过程很快,返回0,但实际效果未知
  //ret = imu.executeProperty(   ZenImuProperty_CalibrateGyro  );
  //printf("excute ZenImuProperty_CalibrateGyro ret: %d  \n", ret );

  //陀螺仪自动校准开关（本质为静止判断算法，设置阈值，小于一定值判断为静止,开启后转动慢会出现较大误差)
  ret = imu.setBoolProperty( ZenImuProperty_GyrUseAutoCalibration, gyr_auto_calibration );
  MLOGI("Set GyrUseAutoCalibration %d , operate ret: %d.", gyr_auto_calibration, ret );

  //滤波模式（待确认）
  //0x00: only groy
  //0x01: groy+acc(kalaman)
  //0x02: groy+acc+mag(kalaman)
  //0x03: groy+acc(DCM)
  //0x04: groy+acc+mag(DCM)
  ///设置滤波模式:EKF
  //ret = imu.setInt32Property( ZenImuProperty_FilterMode, 0x01 );
  //printf("excute ZenImuProperty_FilterMode ret: %d  \n", ret );

  //ret = imu.setInt32Property( ZenImuProperty_AccRange,  0x02 );
  //printf("excute ZenImuProperty_AccRange ret: %d  \n", ret );

  ///设置linearAcc输出
  //ret = imu.setBoolProperty( ZenImuProperty_OutputLinearAcc, true );
  //printf("excute ZenImuProperty_OutputLinearAcc ret: %d  \n", ret );

  ///设置输出帧率
  ret = imu.setInt32Property( ZenImuProperty_SamplingRate, samplingrate );
  MLOGI("Set SamplingRate as: %d , operate ret: %d.", samplingrate, ret );

  ///方向角清零
  //ret = imu.setInt32Property( ZenImuProperty_OrientationOffsetMode ,  ZenOrientationOffsetMode_Heading );
  //printf("excute Orientation offset ret: %d  \n", ret );
  //-------------------------------------------------------------------
  ros::Rate loop_rate(1000);
  while( nh_p->ok() ){
    auto event = client.waitForNextEvent();
    if( event.second.component.handle == imu.component().handle ){
      auto data = event.second.data.imuData;
      //printf("linAcc: %.4f  %.4f  %.4f \n", data.linAcc[0], data.linAcc[1], data.linAcc[2] );
      //printf("rpy: %.4f  %.4f  %.4f \n", data.r[0], data.r[1], data.r[2] );
      //发布IMU数据
      publish_imu_data( data );
      //发布航向角数据
      g_yaw_msg.data = data.r[2];
      g_yaw_publisher.publish( g_yaw_msg );

    }
    keyboard_control();
    //ros::spinOnce();
    M_MSLEEP(1);
  }
  return 0;
}



