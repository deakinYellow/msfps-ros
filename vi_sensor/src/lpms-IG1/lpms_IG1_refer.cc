//===========================================================================//
//
// Copyright (C) 2020 LP-Research Inc.
//
// This file is part of OpenZen, under the MIT License.
// See https://bitbucket.org/lpresearch/openzen/src/master/LICENSE for details
// SPDX-License-Identifier: MIT
//
//===========================================================================//

// request always the C++14 interface of OpenZen, even if the compiler
// can support he C++17 interface
#define OPENZEN_CXX14
#include "openzen/include/OpenZen.h"

#include <iostream>
#include <math.h>
#include <vector>

#include "common/utility-cc/tool.hpp"
#include "common/utility-math/mdatafilter.hpp"


using namespace zen;

///获取本次 yaw 变化
///[input] yaw 范围[ -PI --- PI ]
///单位度
double getYawDeviation( const double &current_yaw, const double &last_yaw ){
    double  d_yaw = 0;
    //从 IMU ypr 中获取 d_yaw  (yaw范围[ -PI -- PI ])
    //有可能跨越边界
    //从小到大，原来为减,从-PI跨到PI
    if( fabs( current_yaw ) + fabs( last_yaw ) > 180  &&
      last_yaw < 0 &&
      current_yaw > 0 ) {
      d_yaw = current_yaw - last_yaw  - 2*180;
      //ROS_DEBUG("fix d yaw %f ", d_yaw );
    }
    //从大到小，原来为增，从PI跨到-PI
    else if( fabs( current_yaw ) + fabs( last_yaw ) > 180  &&
           last_yaw > 0 &&
           current_yaw < 0 ) {
           d_yaw = current_yaw - last_yaw + 2*180;
           //ROS_DEBUG("fix d yaw %f ", d_yaw );
    }
    else {
        d_yaw = current_yaw - last_yaw;
    }
    return  d_yaw;
}



int main(int argc, char* argv[])
{
    // enable resonable log output for OpenZen
    ZenSetLogLevel(ZenLogLevel_Info);

    // create OpenZen Clien
    auto clientPair = make_client();
    auto& clientError = clientPair.first;
    auto& client = clientPair.second;

    if (clientError) {
        std::cout << "Cannot create OpenZen client" << std::endl;
        return clientError;
    }
    ///----------------List Sensor------------------------------------
    // connect to sensor on IO System by the sensor name
    auto sensorPair = client.obtainSensorByName("LinuxDevice", "IG1232000470");
    auto& obtainError = sensorPair.first;
    auto& sensor = sensorPair.second;   //用于访问传感器属性

    if (obtainError) {
        std::cout << "Cannot connect to sensor" << std::endl;
        client.close();
        return obtainError;
    }

    // check that the sensor has an IMU component
    auto imuPair = sensor.getAnyComponentOfType(g_zenSensorType_Imu);
    auto& hasImu = imuPair.first;
    auto imu = imuPair.second;

    if (!hasImu)
    {
        std::cout << "Connected sensor has no IMU" << std::endl;
        client.close();
        return ZenError_WrongSensorType;
    }
    ///陀螺仪校准
    ///执行过程很快,返回0,但实际效果未知
    ZenError ret = sensor.executeProperty(   ZenImuProperty_CalibrateGyro  );
    printf("excute ZenImuProperty_CalibrateGyro ret: %d  \n", ret );
    ///linearAcc输出
    ////返回错误码850,不支持以下操作?
    ret = sensor.executeProperty(   ZenImuProperty_OutputLinearAcc  );
    printf("excute ZenImuProperty_OutputLinearAcc ret: %d  \n", ret );

    // readout up to 200 samples from the IMU
    //for (int i = 0; i < 200; i++) {

    std::vector<double> yaw_data;
    double yaw_sum100 = 0;
    uint yaw_data_max=100;
    stool::FPS  fps_d;

    double stop_threshold = 0.0001;  //degress
    double yaw = 0;
    double last_yaw = 0;

    bool car_stop = false;

    while( true ){
        auto event = client.waitForNextEvent();
        if ( event.second.component.handle == imu.component().handle ){

            std::cout << "> Acceleration: \t x = " << event.second.data.imuData.a[0]
                << "\t y = " << event.second.data.imuData.a[1]
                << "\t z = " << event.second.data.imuData.a[2] << std::endl;
            /*
            std::cout << "> Gyro: \t\t x = " << event.second.data.imuData.g[0]
                << "\t y = " << event.second.data.imuData.g[1]
                << "\t z = " << event.second.data.imuData.g[2] << std::endl;
            printf("rpy %.3f %.3f %.3f \n", double( event.second.data.imuData.r[0]),
                                            double( event.second.data.imuData.r[1]),
                                            double( event.second.data.imuData.r[2]) );

            printf("LineAcc %.6f %.6f %.6f \n",double( event.second.data.imuData.linAcc[0]),
                                            double( event.second.data.imuData.linAcc[1]),
                                            double( event.second.data.imuData.linAcc[2])  );

            fps_d.Sampling();
            printf("fps: real: %.2f \n",  fps_d.fps_real );
            printf("CalibGroy %.6f %.6f %.6f \n",double( event.second.data.imuData.g[0]),
                                            double( event.second.data.imuData.g[1] ),
                                            double( event.second.data.imuData.g[2] ) );
            */
            double current_yaw = double( event.second.data.imuData.r[2] );
            double d_yaw = getYawDeviation( current_yaw ,  last_yaw );
            //printf("dyaw: %f \n", d_yaw );
            last_yaw = current_yaw;

            yaw_data.push_back( d_yaw );
            yaw_sum100 += d_yaw;

            if( yaw_data.size() > yaw_data_max ){
                yaw_sum100 -= yaw_data[ 0 ];
                yaw_data.erase( yaw_data.begin() );
            }
            ulong current_count = yaw_data.size();
            double avr =  double( yaw_sum100 / current_count );
            printf("current_count: %ld avr dyaw: %f \n", current_count, avr );
            if( fabs( avr ) < stop_threshold ){
                printf("-stop\n" );
                car_stop = true;
            }
            else{
                ///需要做补偿
                printf("+move\n" );
                yaw += d_yaw;
            }
            printf("current_yaw: %.6f \n", yaw );
        }

    }
    client.close();
    std::cout << "Sensor connection closed" << std::endl;
    return 0;
}






