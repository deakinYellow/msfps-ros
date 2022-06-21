#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <iomanip>
#include <cmath>
#include <thread>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include "lpsensor/LpmsSensorI.h"
#include "lpsensor/LpmsSensorManagerI.h"

#include "../../common/mros/parameter.hpp"
#include "../../common/utility-math/meigen.hpp"
#include "../../common/utility-cc/tool.hpp"


using std::cout;
//using std::endl;


static sensor_msgs::Imu imu_data_g;
static ros::Publisher imu_publisher_g;
//static double gravity_g = 9.7925;
static LpmsSensorI* lpms_g;


//求在原坐标系(世界坐标系,xoy为水平面)下的加速度(线性加速度) //单位g
Eigen::Vector3d get_linear_acc( const Eigen::Vector3d acc_raw, const Eigen::Quaterniond q_ins ){
    Eigen::Vector3d acc_linear = meigen::Vector3dTransformByQuaterniond( acc_raw, q_ins.inverse() );
    //减去重力加速度
    acc_linear = acc_linear - Eigen::Vector3d(0, 0, -1 );
    return  acc_linear;
}

//static stool::FPS fps_d;

//data callback
//note: LPMS-ME1-DK 原始数据, 加速度单位为 g , 陀螺仪单位为 deg / s
void lpms_callback( ImuData d, const char* dev_id ){

#if 0    //fps get
    fps_d.Sampling();
    printf("FPS,normal: %d  avr: %.2f  real: %.2f \n", fps_d.fps_normal, fps_d.fps_average, fps_d.fps_real );
#endif
    //printf("lpms callback, id: %s \n", dev_id );
    imu_data_g.header.frame_id = "map";
    imu_data_g.header.stamp = ros::Time::now();

    /*//直接读取四元数发现，绕Z轴旋转时方向是反的
    imu_data_g.orientation.w = double( d.q[ 0 ] );
    imu_data_g.orientation.x = double( d.q[ 1 ] );
    imu_data_g.orientation.y = double( d.q[ 2 ] );
    imu_data_g.orientation.z = double( d.q[ 3 ] );
    */
    //改用欧拉角输出;注意转化为弧度
    Eigen::Vector3d ypr( double( d.r[2] ), double( d.r[1] ) , double( d.r[0] ) );
    ypr = ypr / 180.0 * M_PI;
    //ROS_INFO("lpms ypr: %f %f %f ", ypr[2], ypr[1], ypr[ 0 ] );
    Eigen::Quaterniond q = meigen::RotationYPRToQuaterniond( ypr );

    imu_data_g.orientation.w = q.w();   // w for quaterniond
    imu_data_g.orientation.x = q.x();
    imu_data_g.orientation.y = q.y();
    imu_data_g.orientation.z = q.z();

    // Raw accelerometer sensor data.
    //imu_data_g.linear_acceleration.x = double( d.aRaw[ 0 ] );
    //imu_data_g.linear_acceleration.y = double( d.aRaw[ 1 ] );
    //imu_data_g.linear_acceleration.z = double( d.aRaw[ 2 ] );

    /*
     *
    ///求线性加速度
    Eigen::Vector3d acc_raw( double(d.aRaw[ 0 ]), double( d.aRaw[ 1 ]), double( d.aRaw[ 2 ] ) );
    Eigen::Vector3d acc_linear = get_linear_acc( acc_raw, q );
    acc_linear *= gravity_g;
    imu_data_g.linear_acceleration.x = acc_linear.x();
    imu_data_g.linear_acceleration.y = acc_linear.y();
    imu_data_g.linear_acceleration.z = acc_linear.z();
    */
    //直接取IMU线性加速度（去除重力干扰,参考坐标系为IMU自身坐标系）//直接取比自己换算的精度高
    imu_data_g.linear_acceleration.x = double( d.linAcc[ 0 ] );
    imu_data_g.linear_acceleration.y = double( d.linAcc[ 1 ] );
    imu_data_g.linear_acceleration.z = double( d.linAcc[ 2 ] );

    // Raw gyroscope sensor data. //单位转换为radian
    imu_data_g.angular_velocity.x = double( d.gRaw[ 0 ] ) / 180.0 * M_PI;
    imu_data_g.angular_velocity.y = double( d.gRaw[ 1 ] ) / 180.0 * M_PI;
    imu_data_g.angular_velocity.z = double( d.gRaw[ 2 ] ) / 180.0 * M_PI;
    //publish msg
    imu_publisher_g.publish( imu_data_g );

    /*
    printf("linear_acceleration for x y z: %f %f %f \n",
           imu_data_g.linear_acceleration.x,
           imu_data_g.linear_acceleration.y,
           imu_data_g.linear_acceleration.z );

    printf("angular_velocity for x y z: %f %f %f \n",
           imu_data_g.angular_velocity.x,
           imu_data_g.angular_velocity.y,
           imu_data_g.angular_velocity.z );
    */
}


void keyboard_control( const ros::TimerEvent&  event ){
    //printf("keyboard control, input: %X \n", opt );
    char opt = 0x00;
    opt = stool::get_key();
    ///输入r方向复位
    if( opt == 'r' ){
        printf("keyboard control input: %X, set OrientationOffset!\n", opt );
        //需要注意传参!!!
        //参数v=0代表相对坐标系,此时yaw roll pitch全部置0
        //参数v=1代表绝对坐标系,此时yaw置0,并确保水平静止状态下x,y轴加速度值接近0
        lpms_g->setOrientationOffset( 1 );
    }
}


int main(int argc, char **argv){

    //initialize ros
    ros::init(argc, argv, "imu");
    ros::NodeHandle nh("imu");

    //--------------------------params----------------------------------
    ros::NodeHandlePtr nh_p = ros::NodeHandlePtr( &nh );
    mros::Parameter parameter( nh_p );
    std::string  dev_id;
    std::string  imu_data_pub_topic;
    parameter.Get<std::string>("dev_id", dev_id, "/dev/imu" );
    parameter.Get<std::string>("imu_data_pub_topic", imu_data_pub_topic, "/lpms/imu_data_raw" );

    //parameter.Get("gravity", gravity_g  );

    //--------------------------params----------------------------------

    std::cout<<"------------- [lpms] node starts. 100HZ -------------"<<std::endl;
    //ros::Subscriber uav_state_sub = nh.subscribe("/uav_state/navigation_state", 10, navigation_state_callback );
    //ros::Subscriber imu_data_sub = nh.subscribe("/mavros/imu/ypr_acc_short", 10, imu_data_callback );
    imu_publisher_g = nh.advertise<sensor_msgs::Imu>( imu_data_pub_topic , 20 );
    //---------------------init lpms IMU--------------------------------
    //ImuData lpms_imu_data;
    // Gets a LpmsSensorManager instance
    LpmsSensorManagerI* lpms_manager = LpmsSensorManagerFactory();
    //baudrate 默认 115200 , 可配置921600
    lpms_manager->setRs232Baudrate( SELECT_LPMS_UART_BAUDRATE_921600 );
    // Connects to LPMS-RS232 sensor try virtual com port
    //lpms_manager->setRs232Baudrate( SELECT_LPMS_UART_BAUDRATE_115200 );

    lpms_g = lpms_manager->addSensor( DEVICE_LPMS_RS232, dev_id.c_str() );

    //lpms_g = lpms_manager->addSensor( DEVICE_LPMS_U, dev_id.c_str() );

    lpms_g->setFps( SELECT_STREAM_FREQ_100HZ  );

    M_MSLEEP(1000);
    //校准后发现加速度计不准,有问题,弄清楚再用

    ROS_INFO("Start Calibrate Gyro.");
    lpms_g->startCalibrateGyro( );
    ROS_INFO("Calibrate Gyro End.");

    //set callback
    lpms_g->setCallback( lpms_callback );
    //---------------------init lpms IMU--------------------------------
    M_MSLEEP(1000);
    //not real time
    ros::Timer keyboard_input_timer = nh.createTimer( ros::Duration(1.0/10), keyboard_control );
    //ros::Rate loop_rate( frame_rate );
    ros::spin();
    std::cout<<"------------- [lpms] node exits -------------"<<std::endl;
    // Removes the initialized sensor
    lpms_manager->removeSensor(lpms_g);
    // Deletes LpmsSensorManager object
    delete lpms_manager;
    return 0;
}








