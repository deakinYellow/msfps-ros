#include <math.h>

#define ENABLE_MLOGD
#include "../../common/utility-cc/tool.hpp"
#include "../../common/utility-cc/stool.hpp"
#include "yaw_tracking.h"

///获取本次yaw变化
///[input] yaw 范围[ -PI --- PI ]单位弧度
static double get_dyaw( const double& current_yaw,  const double& last_yaw ){
  double  dyaw = 0;
  //需要对跨越边界进行判断
  //从小到大，原来为减,从-PI跨到PI
  if( fabs( current_yaw ) + fabs( last_yaw ) > M_PI  &&
      last_yaw < 0 &&
      current_yaw > 0 ) {
    dyaw = current_yaw - last_yaw  - 2*M_PI;
  }
  //从大到小，原来为增，从PI跨到-PI
  else if( fabs( current_yaw ) + fabs( last_yaw ) > M_PI  &&
           last_yaw > 0 &&
           current_yaw < 0 ) {
    dyaw = current_yaw - last_yaw + 2*M_PI;
  }
  else {
    dyaw = current_yaw - last_yaw;
  }
  return  dyaw;
}


void uifp::YawTracking::set_workmode( const YawTrackingWorkModeL mode ){
  workmode_ = mode;
  switch ( mode ) {
  case YAWTRACKING_WORKMODE_INITIALIZE :
    kf_.reset();  //重置kf
    kf_.set_R_scale( YAWTRACKING_WORKMODE_INITIALIZE_KF_R_SCALE ); //设置R参数为初始化模式
    set_status( YAWTRACKING_STATUS_INITIALIZING ); //修改状态为初始化状态
    break;
  case YAWTRACKING_WORKMODE_QUICK_TRACKING:
    kf_.set_R_scale( YAWTRACKING_WORKMODE_QUICK_TRACKING_KF_R_SCALE );
    set_status( YAWTRACKING_STATUS_QUICK_TRACKING );
    break;
  case YAWTRACKING_WORKMODE_SLOW_TRACKING:
    kf_.set_R_scale( YAWTRACKING_WORKMODE_SLOW_TRACKING_KF_R_SCALE );
    set_status( YAWTRACKING_STATUS_SLOW_TRACKING );
    break;
  case YAWTRACKING_WORKMODE_PAUSE_TRACKING:
    set_status( YAWTRACKING_STATUS_PAUSE_TRACKING );
    break;
  }
}


/**
 * @brief
 * @param  [in]
 * @param  [out]
 * @retval
 * @note
 **/
////实际观测角度为标签0->标签１方向,与 IMU X轴相差90度
int uifp::YawTracking::update( const Eigen::Vector3d& p0, const Eigen::Vector3d& p1,
                               const Eigen::Quaterniond& q0 , const Eigen::Quaterniond& q1 ){
  //先检查工作状态
  if( workmode_ == YAWTRACKING_WORKMODE_STOP ){
    MLOGI("current yawtracking workmode is stop.");
    return MFAIL;
  }else if( workmode_ == YAWTRACKING_WORKMODE_PAUSE_TRACKING ){
    MLOGI("current yawtracking workmode is pause_tracking.");
    return MFAIL;
  }

  ///获取dyaw
  double yaw00 = meigen::QuaterniondToRotationYPR_Y( q0 ).x();
  double yaw01 = meigen::QuaterniondToRotationYPR_Y( q1 ).x();
  double dyaw00 = get_dyaw( yaw00,last_yaw00_ );
  double dyaw01 = get_dyaw( yaw01,last_yaw01_ );

  double expection_value = 0.2;
  if( dyaw00 > expection_value || dyaw01 > expection_value ){
    MLOGW("dyaw too large: %f %f ", dyaw00, dyaw01 );
    last_yaw00_ = yaw00;
    last_yaw01_ = yaw01;
    return MFAIL;
  }
  last_yaw00_ = yaw00;
  last_yaw01_ = yaw01;

  //MLOGI("yaw0: %f yaw1: %f ", yaw00, yaw01 );
  MLOGD("dyaw00: %f dyaw01: %f ", dyaw00, dyaw01 );

  //获取tag00->tag01 与X轴的夹角
  Eigen::Vector2d base_v(1,0);
  Eigen::Vector2d v2d;
  v2d.x() = p1.x() - p0.x();
  v2d.y() = p1.y() - p0.y();
  tag_angle_ = meigen::GetAngleBetweenVec( v2d, base_v );
  if( v2d.y() < 0 ){
    tag_angle_ *= -1;
  }
  MLOGD("tag_angle_: %f degress", meigen::RadianToDegree<double>( tag_angle_ ) );
  double kfout_angle;
  double avr_dyaw = ( dyaw00 + dyaw01 ) / 2.0;
  if( MSUCCESS == kf_.update( avr_dyaw , tag_angle_ ) ){
    kfout_angle = kf_.get_angle();
    set_yaw( kfout_angle );
    MLOGD("kf out angle : %f degress", meigen::RadianToDegree( kfout_angle ) );
    ///更新状态
    update_status( get_yaw());
    return MSUCCESS;
  }
  return MFAIL;
}


////YawTracking状态判断更新
void uifp::YawTracking::update_status( const double angle ){
  //缓存区数据更新
  update_stuats_check_data( angle );
  switch ( status_ ) {
  case YAWTRACKING_STATUS_INITIALIZING :
    if( status_check_data_.data.size() == status_check_data_.params.size_max ){
      double sd = stool::GetVectorStandardDeviation<double>( status_check_data_.data );
      MLOGI("yaw initializing,status check threshold: %f current sd: %f.",
            status_check_data_.params.sd_threshold, sd );
      ///初始化成功，设置工作模式为快速跟踪
      if( sd < status_check_data_.params.sd_threshold ){
        set_workmode( YAWTRACKING_WORKMODE_QUICK_TRACKING );
      }
    }
    else {
      MLOGI("yaw initializing, waitting enough data, dst size: %ld current size: %ld ",
            status_check_data_.params.size_max, status_check_data_.data.size()  );
    }
    break;
  case YAWTRACKING_STATUS_QUICK_TRACKING :
    MLOGI("yaw status quick_tracking.");
    break;
  case YAWTRACKING_STATUS_SLOW_TRACKING :
    MLOGI("yaw status slow_tracking.");
    break;
  case YAWTRACKING_STATUS_PAUSE_TRACKING :
    MLOGI("yaw status pause_tracking.");
    break;
  }
}

////YawTracking状态判断数据缓存区更新
void uifp::YawTracking::update_stuats_check_data( const double angle ){
  ///考虑到[-PI和PI]之间边界存在跳变，这里对angle取绝对值
  double angle_abs = fabs( angle );
  status_check_data_.data.push_back( angle_abs );
  if( status_check_data_.data.size() > status_check_data_.params.size_max ){
    status_check_data_.data.erase( status_check_data_.data.begin() );
  }
}

