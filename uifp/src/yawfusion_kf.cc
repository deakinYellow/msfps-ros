#include <math.h>
#include "yawfusion_kf.h"

#define  ENABLE_MLOGD
#include "../../common/utility-cc/tool.hpp"

#include "uifp_common.h"

//-----------------KF 参考实现-----------------------------------
/**
 * @brief  Kalman融合IMU和tag得到的方向角信息
 *      已知观测角度 mesuar_y , 以及变换量dy, 求y
 *      状态转移矩阵　F = [ 1 ] , 控制矩阵 B=[1]
 * @param  [in]  d_angle     由IMU得出的yaw变换值
 * @param  [in]  mes_angle   由两标签得出的方向角信息
 * @retval
 * @note
 *      需要处理跨越边界问题 输入mes_angle范围[-Pi-Pi],会在边界产生跳边
 *      后续改进:
 *       1.进一步获取陀螺仪 yaw bias
 **/
int uifp::YawFusionKf::update( const double d_angle , const double mes_angle ){

  Eigen::VectorXd X(1);   //待估计状态变量,即角度
  Eigen::MatrixXd F = Eigen::MatrixXd::Ones(1,1);  //状态转移矩阵[1]
  Eigen::MatrixXd B = Eigen::MatrixXd::Ones(1,1);  //控制矩阵[1]
  Eigen::VectorXd u(1);

  //赋初值
  X << angle_;
  u << d_angle;

  Eigen::MatrixXd P = state_cov_;
  Eigen::MatrixXd Q(1,1);
  Q << Q_scale_;

  ///1.------------------------------预测-------------
  X = F * X + B*u;
  //设过程噪声的协方差矩阵(M = P(K) )
  Eigen::MatrixXd M = F*P*F.transpose() + Q;
  //MLOGD("M:");
  //std::cout << M.transpose() << std::endl;
  //2.-------------------------------更新-----------
  //测量模型矩阵
  Eigen::MatrixXd H(1,1);
  H << 1;
  //kalaman gain
  Eigen::VectorXd K = M*H.transpose() / ( (H*M*H.transpose())(0,0) + R_scale_ );
  MLOGD("R_scale: %f ", R_scale_ );

  //MLOGD("Kalman gain K: %f ", K(0) );
  //std::cout << K.transpose() << std::endl;

  // Update P for the a posteriori covariance matrix
  P = ( Eigen::MatrixXd::Identity(1,1) - K*H ) * M;

  //Update the state
  //需要根据历史值对err_angle,进行调整,不然会产生异常的观测数据
  double err_angle = mes_angle - X(0,0);
  //判断是够为跨区或者异常值
  if( err_angle > M_PI ){
    //MLOGW("detect exception err_angle: %f ", err_angle );
    err_angle -= 2 * M_PI;
  } else if( err_angle < -1 * M_PI ) {
    //MLOGW("detect exception err_angle: %f ", err_angle );
    err_angle += 2 * M_PI;
  }

  //X = X + K * ( mes_angle - X(0,0) );
  X = X + K * ( err_angle );

  //3.============================状态保存===============================
  state_cov_ = P;
  //MLOGD("P:");
  //std::cout << P.transpose() << std::endl;
  ///调整X(0)在范围[-Pi -- Pi ]
  X( 0 ) = TransformToCyclePi( X(0) );
  angle_ = X(0);

  return MSUCCESS;
}


