#ifndef YAW_TRACKING_H
#define YAW_TRACKING_H

#include "../../common/utility-math/meigen.hpp"
#include "uidata_type.h"
#include "yawfusion_kf.h"

namespace uifp {

typedef struct YawStatusCheckDataT{
    std::vector<double> data;
    Eigen::Vector3d sd;              //标准差
    KfStatusCheckParametersT  params;
}YawStatusCheckDataT;

typedef enum YawTrackingStatusL{
  YAWTRACKING_STATUS_UNINITIALIZED = 0x00,//未初始化的
  YAWTRACKING_STATUS_INITIALIZING,        //正在初始化的，此时基本由tag观测角度决定
  YAWTRACKING_STATUS_QUICK_TRACKING,      //此时tag观测影响较大
  YAWTRACKING_STATUS_SLOW_TRACKING,       //此时tag观测影响较小，缓慢修正由IMU累积的误差
  YAWTRACKING_STATUS_PAUSE_TRACKING       //暂停跟踪航向角
}YawTrackingStatusL;

#define  YAWTRACKING_WORKMODE_INITIALIZE_KF_R_SCALE     0.1
#define  YAWTRACKING_WORKMODE_QUICK_TRACKING_KF_R_SCALE 100.0 //100.0
#define  YAWTRACKING_WORKMODE_SLOW_TRACKING_KF_R_SCALE  500.0

typedef enum YawTrackingWorkModeL{
  YAWTRACKING_WORKMODE_STOP = 0x00,
  YAWTRACKING_WORKMODE_INITIALIZE,
  YAWTRACKING_WORKMODE_QUICK_TRACKING,
  YAWTRACKING_WORKMODE_SLOW_TRACKING,
  YAWTRACKING_WORKMODE_PAUSE_TRACKING
}YawTrackingWorkModeL;


/**
 * @brief  航向角跟踪，为系统提供准确可靠的航向角数据
 *         主要通过IMU航向数据和UWB的2个tag方向角融合得到航线角数据
 * @note
 *         1.注意这里的航向角，实际为tag0-->tag1的方向，不一定是实际运动方向
 *         2.航向角状态: 初始化中，快速跟踪，缓慢跟踪
 *         3.角度单位统一为弧度.统一区间[-PI,PI],注意处理边界跳边问题
 **/
class YawTracking
{
public:
  YawTracking(){
    ;
  }

  void set_parameters( const KfStatusCheckParametersT& yaw_status_check_params ){
    status_check_data_.params = yaw_status_check_params;
  }
  int update( const Eigen::Vector3d& p0, const Eigen::Vector3d& p1,
              const Eigen::Quaterniond& q0 , const Eigen::Quaterniond& q1 );
  void set_workmode( const YawTrackingWorkModeL mode );

  YawTrackingStatusL get_status( void ){
    return status_;
  }
  double get_yaw( void ){
    return yaw_;
  }

private:
  ///IMU yaw
  double last_yaw00_ = 0;
  double last_yaw01_ = 0;
  double tag_angle_ = 0;

  double yaw_;
  YawFusionKf kf_;

  void set_yaw( const double& yaw ){
    yaw_ = yaw;
  }

  YawTrackingStatusL status_ = YAWTRACKING_STATUS_UNINITIALIZED;
  YawStatusCheckDataT status_check_data_;
  void update_stuats_check_data( const double angle );
  void update_status( const double angle );
  void set_status( const YawTrackingStatusL status ){
    status_ = status;
  }
  //--------------工作模式-----------------------
  YawTrackingWorkModeL workmode_ = YAWTRACKING_WORKMODE_STOP;

};

}  //end of namespace uifp

#endif // YAW_TRACKING_H
