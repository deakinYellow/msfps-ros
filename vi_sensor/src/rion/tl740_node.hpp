#ifndef TL740_NODE_HPP
#define TL740_NODE_HPP

#include <iostream>
#include <map>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include "../../common/utility-cc/tool.hpp"
#include "../../common/utility-cc/mthread.hpp"

#include "../../common/utility-math/mdatafilter.hpp"
#include "../../common/mros/node_handle.hpp"
#include "../../common/mros/parameter.hpp"

#include "tl740.hpp"


//求在原坐标系(世界坐标系,xoy为水平面)下的加速度(线性加速度) //单位g
static Eigen::Vector3d get_linear_acc( const Eigen::Vector3d acc_raw, const Eigen::Quaterniond q_ins ){
    Eigen::Vector3d acc_linear = meigen::Vector3dTransformByQuaterniond( acc_raw, q_ins.inverse() );
    //减去重力加速度
    acc_linear = acc_linear - Eigen::Vector3d(0, 0, -1 );
    return  acc_linear;
}

void* keyboardControl( void* tl740_node_obj );

class TL740Node : public mros::NodeHandle {
public:

    TL740* tl740_;
    MThread* keyboard_thread_ = ( new MThread);
    bool exit_mainloop_ = false;

    explicit TL740Node( std::string name_space ) : mros::NodeHandle( name_space ) {
        ;
    }

    bool Start( void ){
        //参数加载
        std::shared_ptr<mros::Parameter> parameter
          = std::shared_ptr<mros::Parameter>( new mros::Parameter( nh ) );
        loadParams( parameter );

        //----------------------初始化IMU对象--------------------------------
        tl740_ = new TL740( dev_id_, uint(dev_baudrate_) );
        if( !tl740_){
            return false;
        }
        tl740_->StartProcess();
        #if  USE_DATA_MODE_NORMAL_2
        tl740_->SetDataMode( TL740::DATA_MODE_NORMAL_2 );
        tl740_->SetFrameRate( TL740::DATA_FR_100HZ );
        ///关闭静止检测算法
        tl740_->SetStaticDetect( false );
        #elif USE_DATA_MODE_9A
        tl740_->SetDataMode(  TL740::DATA_MODE_9A );
        tl740_->SetFrameRate( TL740::DATA_FR_100HZ );
        #endif
        //--------------------------------------------------------------

        //ros publish
        imu_publisher_ = nh->advertise<sensor_msgs::Imu>( imu_data_pub_topic_ , 20 );
        //keyboard_control_timer_ = nh->createTimer( ros::Duration(1.0/10), &TL740Node::keyboardControl, this );
        //创建键盘输入进程
        ROS_INFO("create keyboard thread.");
        if( 0 != keyboard_thread_->Create( keyboardControl , (void*)this ) ){
            ROS_ERROR("create keyboard thread.fail");
        }
        return true;
    }

    void MainLoop( void ){
        #if  USE_DATA_MODE_NORMAL_2
        TL740::TL740Mode2DataT d;
        TL740::TL740Mode2DataBuffT* ds;
        #elif USE_DATA_MODE_9A
        TL740::TL740Mode9ADataT d;
        TL740::TL740Mode9ADataBuffT* ds;
        #endif
        ds = tl740_->data_buff;
        uint d_count=0;
        stool::FPS fps_d;
        //ros::Rate  loop_rate(2000);   ///少数情况出现误差,可能含usleep不用
        //long last_time_stamp = stool::sys_ms_ts();
        while ( nh->ok() && !exit_mainloop_ ) {
            //ROS_INFO("tl740 loop.");
            if( ds ){
                std::unique_lock<std::mutex> ds_lock( ds->mutex );  //!!!
                if( !ds->data.empty() ){
                    fps_d.Sampling();
                    //if( fps_d.fps_real < 40 )
                    //printf("fps: %.6f  avr: %.2f \n", fps_d.fps_real , fps_d.fps_average );
                    //long current_time_stamp = stool::sys_ms_ts();
                    //long dt = current_time_stamp - last_time_stamp;
                    //last_time_stamp = current_time_stamp;
                    //if( dt > 25 ){
                        //printf("imu frame dt: %ld ms,\n", dt );
                    //}
                    //取数据
                    d = ds->data.front() ;
#if USE_DATA_MODE_NORMAL_2
                    printf("mode2 data %d : %.2f  %.2f  %.2f \n", ++d_count, d.acc_x, d.acc_y, d.yaw );
#elif USE_DATA_MODE_9A
                    printf("mode 9A rpy degree %d : %.2f  %.2f  %.2f \n", ++d_count, d.roll, d.pitch, d.yaw );
#endif
                    //printf("mode 9A acc %d : %.6f  %.6f  %.6f \n", ++d_count, d.acc_x, d.acc_y, d.acc_z );
                    //发布数据
                    dataPublish( d );
                    //弹出数据
                    ds->data.pop();
                }
            }
            //ROS_DEBUG("debug 8.");
            //long last_time_stamp = stool::sys_ms_ts();
            //loop_rate.sleep();
            M_USLEEP(300);
            //ros::spinOnce();  //回调函数处理,若含有回调函数必须加
            //long current_time_stamp = stool::sys_ms_ts();
            //long dt = current_time_stamp - last_time_stamp;
            //if( dt > 1 )
                //printf("1. sleep and spinonce  cost: %ld ms,\n", dt );
        }

    }

    ~TL740Node(){
        nh->shutdown();
        delete tl740_;
        delete keyboard_thread_;
    }

private:
    //=================Params=========================
    std::string dev_id_;
    int dev_baudrate_;
    std::string imu_data_pub_topic_;
    //================================================

    sensor_msgs::Imu imu_data_;
    ros::Publisher imu_publisher_;

    void loadParams( const std::shared_ptr<mros::Parameter> parameter ){
        if( !parameter ){
            ROS_WARN("loadParams fail, nullptr!");
            return;
        }
        parameter->Get<std::string>("dev_id",dev_id_, "/dev/imu" );
        parameter->Get<int>("dev_baudrate_", dev_baudrate_, 115200 );
        parameter->Get<std::string>("imu_data_pub_topic", imu_data_pub_topic_,"/tl740/imu_data_raw" );
    }

#if USE_DATA_MODE_NORMAL_2
    void dataPublish( TL740::TL740Mode2DataT &d ){

        imu_data_.header.frame_id = "map";
        imu_data_.header.stamp = ros::Time::now();

        imu_data_.linear_acceleration.x =  d.acc_x;
        imu_data_.linear_acceleration.y =  d.acc_y;
        imu_data_.linear_acceleration.z =  0.0;

        //改用欧拉角输出;注意转化为弧度
        Eigen::Vector3d ypr( d.yaw ,0.0 ,0.0 );
        ypr = mdf::DegreeToRadian<Eigen::Vector3d>( ypr );
        //ROS_INFO("lpms ypr: %f %f %f ", ypr[2], ypr[1], ypr[ 0 ] );
        Eigen::Quaterniond q = meigen::RotationYPRToQuaterniond_Y( ypr );
        imu_data_.orientation.w = q.w();
        imu_data_.orientation.x = q.x();
        imu_data_.orientation.y = q.y();
        imu_data_.orientation.z = q.z();
        //发布数据
        imu_publisher_.publish( imu_data_ );
    }
#elif USE_DATA_MODE_9A
    void dataPublish( TL740::TL740Mode9ADataT &d ){

        imu_data_.header.frame_id = "map";
        imu_data_.header.stamp = ros::Time::now();

        //改用欧拉角输出;注意转化为弧度
        Eigen::Vector3d ypr( d.yaw ,d.pitch, d.roll );
        ypr = mdf::DegreeToRadian<Eigen::Vector3d>( ypr );
        //ROS_INFO("lpms ypr: %f %f %f ", ypr[2], ypr[1], ypr[ 0 ] );
        Eigen::Quaterniond q = meigen::RotationYPRToQuaterniond_Y( ypr );
        imu_data_.orientation.w = q.w();
        imu_data_.orientation.x = q.x();
        imu_data_.orientation.y = q.y();
        imu_data_.orientation.z = q.z();

        ///将原始加速度转换为线性加速度(去除重力干扰)
        Eigen::Vector3d raw_acc{ d.acc_x, d.acc_y, d.acc_z };
        Eigen::Vector3d line_acc = get_linear_acc( raw_acc, q );
        imu_data_.linear_acceleration.x =  line_acc.x();
        imu_data_.linear_acceleration.y =  line_acc.y();
        imu_data_.linear_acceleration.z =  line_acc.z();
        //发布数据
        imu_publisher_.publish( imu_data_ );
    }
#endif
    /*
    void keyboardControl( void ){
        char opt = 0x00;
        opt = stool::get_key();
        //printf("keyboard control, input: %X \n", opt );
        ///输入r方向复位
        if( opt == 'r' ){
            opt = 0x00;
            printf("keyboard control input: %X, reset orientation.\n", opt );
            tl740_->ResetYaw();
            tl740_->ResetAcc();
        }
        else if( opt == 'q' ){
            opt = 0x00;
            printf("exit mainloop.\n");
            exit_mainloop_ = true;
        }
    }
    */
};


void* keyboardControl( void* tl740_node_obj ){
    TL740Node* tl740node = ( TL740Node* )( tl740_node_obj );
    char opt = 0x00;
    while ( true ) {
        opt = stool::get_key();
        //printf("keyboard control, input: %X \n", opt );
        ///输入r方向复位
        if( opt == 'r' ){
            opt = 0x00;
            printf("keyboard control input: %X, reset orientation.\n", opt );
            tl740node->tl740_->ResetYaw();
            tl740node->tl740_->ResetAcc();
        }
        else if( opt == 'q' ){
            opt = 0x00;
            printf("exit mainloop.\n");
            tl740node->exit_mainloop_ = true;
            break;
        }
        M_USLEEP(1000);  //稍延时，降低cpu消耗
    }
    return nullptr;
}

#endif




