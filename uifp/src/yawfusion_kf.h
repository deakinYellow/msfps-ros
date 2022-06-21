#ifndef YAWFUSION_KF_H
#define YAWFUSION_KF_H

#include "../../common/utility-math/meigen.hpp"

namespace uifp {


#define DEFAULT_Q_SCALE  0.000001
#define DEFAULT_R_SCALE  1.0
#define DEFAULT_ANGLE    0


class YawFusionKf {

public:
  YawFusionKf(){
    ;
  }
  int update( const double d_angle , const double mes_angle );

  void set_R_scale( const double R_scale ){
    R_scale_ = R_scale;
  }
  double get_angle( void ){
    return angle_;
  }

  void reset( void ){
    angle_ = DEFAULT_ANGLE;
    state_cov_ = Eigen::MatrixXd::Identity(1,1);
    R_scale_ = DEFAULT_R_SCALE;
    Q_scale_ = DEFAULT_Q_SCALE;
  }

private:
  Eigen::MatrixXd state_cov_ = Eigen::MatrixXd::Identity(1,1); //系统协方差矩阵
  double angle_ = DEFAULT_ANGLE;                            //待估计状态变量,即角度
  double Q_scale_ = DEFAULT_Q_SCALE;        //系统噪声系数(实际为IMU dyaw 噪声，建议不用修改)
  double R_scale_ = DEFAULT_R_SCALE;        //观测噪声系数(tag观测角度噪声) //1.0快速响应 1000.0缓慢修正
};

} // endof namespace uifp

#endif // YAWFUSION_KF_H

