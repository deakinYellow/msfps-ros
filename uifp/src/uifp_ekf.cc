#include <iostream>
#include "../../common/utility-cc/tool.hpp"
#include "../../common/utility-cc/stool.hpp"

#include "uifp_ekf.h"

/**
 * @brief         初始化系统状态，以及滤波器参数
 * @param  [in]   initial_position      标签初始位置，可由最小二乘法算法求出，越准确，卡尔曼滤波器收敛越快
 * @param  [in]   kf_params     EKF参数
 * @param  [in]   distance_innovation_dynamic_threshold_parameters     EKF残差阈值动态更新参数
 * @param  [in]   kf_status_check_params    KF状态观测参数
 * @param  [out]
 * @retval
 * @note
 */
void uifp::UifpEkf::init( const Eigen::Vector3d& initial_position,
                          const KfParametersT& kf_params,
                          const DynamicThresholdParametersT& distance_innovation_dynamic_threshold_parameters,
                          const KfStatusCheckParametersT&  kf_status_check_params ){

  //动态阈值//需要根据速度设置
  distance_innovation_threshold_->data = distance_innovation_dynamic_threshold_parameters.initial_value;
  distance_innovation_threshold_->params = distance_innovation_dynamic_threshold_parameters;

  //KF状态判定参数设置
  data_for_check_kf_status_->params = kf_status_check_params;

  //KF参数设置
  *kf_params_ =  kf_params;
  position_ = initial_position;

  //以下参数采用默认设置
  velocity_ = Eigen::Vector3d( 0,0,0 );
  acc_bias_ = Eigen::Vector3d( 0,0,0 );
  nine_state_cov_ = Eigen::MatrixXd::Identity(9,9);
  //设置滤波器初始状态
  kf_status_ = KF_STATUS_INITIALIZING;
}

//通过判断n点标准差大小检查滤波器工作状态是否稳定,在一般初始化时候
void uifp::UifpEkf::update_status( const Eigen::Vector3d& position ){
  ///更新状态判断数据缓存区
  data_for_check_kf_status_->data.push_back( position );
  if( data_for_check_kf_status_->data.size() > data_for_check_kf_status_->params.size_max ){
    data_for_check_kf_status_->data.erase( data_for_check_kf_status_->data.begin() );
  }
  ///更新状态
  switch( kf_status_ ) {
  case KF_STATUS_INITIALIZING: //滤波器收敛判断
    if( data_for_check_kf_status_->data.size() == data_for_check_kf_status_->params.size_max ){
      Eigen::Vector3d sd = meigen::GetVetors3dStandardDeviation( data_for_check_kf_status_->data );
      //printf("status check sd: %f %f %f .\n", sd.x(), sd.y(), sd.z() );
      MLOGI("ekf initializing, waitting stable.");
      if( sd.x() < data_for_check_kf_status_->params.sd_threshold &&
          sd.y() < data_for_check_kf_status_->params.sd_threshold &&
          sd.z() < data_for_check_kf_status_->params.sd_threshold  ){
        kf_status_ = KF_STATUS_STABLE;
      }
    }
    else {
      MLOGI("ekf initializing, waitting enough positions.");
    }
    break;
  case KF_STATUS_STABLE:   //滤波器发散标准，暂未确定
    Eigen::Vector3d sd = meigen::GetVetors3dStandardDeviation( data_for_check_kf_status_->data );
    //printf("status check sd: %f %f %f .\n", sd.x(), sd.y(), sd.z() );
    if( sd.x() > 10 * data_for_check_kf_status_->params.sd_threshold &&
        sd.y() > 10 * data_for_check_kf_status_->params.sd_threshold ){
      MLOGW("warnning: kf divergence!!! \n");
      //kf_status_ = KF_DIVERGENCE;
    }
    break;
  }
}


/**
 * @brief   根据距离残差判断滤波结果，并动态调节距离残差阈值
 *          如果滤波器初始化中，阈值应该设置为固定值50左右即可
 *          如果滤波器工作中，阈值动态调节(残差小于阈值降低阈值，残差大于阈值，增大阈值, 同时进行幅度限制)
 * @param  [in]   kf_status      滤波器当前状态
 * @param  [in]   distance_innovation     本次残差
 * @param  [in][out]   threshold  距离残差阈值对象指针
 * @retval
 * @note
 */
#define DISTANCE_INNVOTATION_THRESHOLD 50
bool judge_ekf_ret_by_innovation( const uifp::KfStatusL kf_status,
                                  const double distance_innovation,
                                  uifp::DynamicThresholdT*  threshold ){
  bool ret = false;
  if( kf_status == uifp::KF_STATUS_INITIALIZING ){
    if( distance_innovation < DISTANCE_INNVOTATION_THRESHOLD ){
      ret = true;
    }
  }
  else if( kf_status == uifp::KF_STATUS_STABLE){
    if( distance_innovation < threshold->data ){
      threshold->data -= threshold->params.reduce_step;
      ret = true;
    }
    else {
      threshold->data += threshold->params.increase_step;
      ret = false;
    }
    //限制幅度!!
    stool::limit_value<double>( threshold->params.min, threshold->params.max, threshold->data );
  }
  return ret;
}


/**
 * @brief  UWB距离数据融合加速度，卡尔曼滤波器设计
 * @param  [in]   anchor_p      当前测量基站坐标
 * @param  [in]   meas_dis      当前测量距离
 * @param  [in]   acc           当前加速度
 * @param  [in]   current_t     当前时间戳,单位s,精确到小数点后三位
 * @param  [out]
 * @retval
 * @note
/////////////可以考虑加入Z轴先验信息进一步改进
/////////////不应该丢弃任何信息，即使它看起来并不是很可靠
 */
#define EXCLUDE_NLOS  1
int uifp::UifpEkf::update( const Eigen::Vector3d& anchor_p,
                           const double meas_dis,
                           const Eigen::Vector3d& acc,
                           const double current_t ){

  //0.================================set params=======================
  double Q_scale = kf_params_->Q_scale;
  double R_scale = kf_params_->R_scale;
  double z_damping_factor = kf_params_->z_damping_factor;
  double tao_acc =  std::pow( kf_params_->acc_gauss_noise_sqrt, 2 );
  double tao_bias = std::pow( kf_params_->acc_bias_noise_sqrt,  2 );
  double meas_dis_err = kf_params_->meas_dis_err;


  //1.===============================预估===============================
  Eigen::VectorXd X(9);   //待估计状态变量
  X <<  position_.x(), velocity_.x(),acc_bias_.x(),
      position_.y(), velocity_.y(),acc_bias_.y(),
      position_.z(), velocity_.z(),acc_bias_.z();

  //std::cout << X.transpose() << std::endl;
  Eigen::MatrixXd P = nine_state_cov_;
  //std::cout << P.transpose() << std::endl;

  double T = current_t - last_update_t_;
  last_update_t_ = current_t;     //先保存当前时间

  if( T > 1 || T < 0 ){   //时间相差太大，可能为初始化或者异常数据,直接返回
    std::cout << "uwb_imu_ekf Time invalid, maybe initial or data break off."<< std::endl;
    return MFAIL;
  }

  double sigma_r = meas_dis_err;

  // F is a 9x9 State Transition Matrix(状态转移矩阵)
  Eigen::MatrixXd F = Eigen::MatrixXd::Zero(9,9);
  Eigen::MatrixXd block_F(3,3);
  block_F << 1, T, -T*T/2.0,
      0, 1, -T,
      0, 0, 1;
  F.block<3,3>(0,0) = block_F;
  F.block<3,3>(3,3) = block_F;
  F.block<3,3>(6,6) = block_F;

  // Q is the acceleration model //系统噪声协方差矩阵
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(9,9);
  Eigen::MatrixXd block_Q(3,3);
  block_Q <<
             std::pow(T,3)/3.0*tao_acc+std::pow(T,5)/20.0*tao_bias,  std::pow(T,2)/2*tao_acc+std::pow(T,4)/8.0*tao_bias,  -std::pow(T,3)/6*tao_bias,
      std::pow(T,2)/2.0*tao_acc+std::pow(T,4)/8.0*tao_bias ,  T*tao_acc+std::pow(T,3)/3*tao_bias                ,  -std::pow(T,2)/2*tao_bias,
      -std::pow(T,3)/6.0*tao_bias                          ,  -std::pow(T,2)/2*tao_bias                         ,  T*tao_bias               ;
  Q.block<3,3>(0,0) = block_Q;
  Q.block<3,3>(3,3) = block_Q;
  Q.block<3,3>(6,6) = block_Q * z_damping_factor;   //小则相信acc
  Q *= Q_scale;

  Eigen::VectorXd u(9);   //控制向量,这里把加速度当做系统的控制输入
  u << acc.x(), 0, 0,
      acc.y(), 0, 0,
      acc.z(), 0, 0;

  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(9,9);  //控制矩阵
  Eigen::MatrixXd block_B(3,3);
  block_B << T*T/2.0,  0,  0,
      T      ,  0,  0,
      0      ,  0,  0;
  B.block<3,3>(0,0) = block_B;
  B.block<3,3>(3,3) = block_B;
  B.block<3,3>(6,6) = block_B;

  // X is the predicted state vector and the predicted covariance matrix
  //系统状态predict
  X = F*X + B * u;

  // M is the predicted covariance matrix
  // 加入系统过程噪声的协方差矩阵( M 即 P(K) )
  Eigen::MatrixXd M = F*P*F.transpose() + Q;

  //2.===============================更新===============================
  Eigen::Vector3d XYZ0 = anchor_p;

  // r_pred is the predicted range measure  //测量距离先验值
  double pred_d = std::sqrt( std::pow(X(0) - XYZ0.x(), 2) +
                             std::pow(X(3) - XYZ0.y(), 2) +
                             std::pow(X(6) - XYZ0.z(), 2) ) + 1e-9;  //确保不为0

  //（距离信息和坐标之间的关系为非线性对原始量测矩进行线性化(一阶泰勒展开)得到H
  //1.根据公式 Z = H * X
  //2. Z 为测距信息 d , 可由 X 和 X' 表达 (空间两点距离公式)
  //3. H = Z / X ,在分别对(x,y,z)求一阶偏导即可得到下列矩阵
  // H is the linearized measurement matrix //测量模型矩阵
  Eigen::MatrixXd H(1,9);
  // printf("H row: %ld  cols: %ld \n", H.rows(), H.cols() );
  H << (X(0) - XYZ0.x())/pred_d,
      0,
      0,
      (X(3) - XYZ0.y())/pred_d,
      0,
      0,
      (X(6) - XYZ0.z())/pred_d,
      0,
      0;
  // K is the Kalman Gain
  double R = std::pow(sigma_r,2) * R_scale;
  Eigen::VectorXd K = M*H.transpose() / ( (H*M*H.transpose())(0,0) + R );

  // Update P for the a posteriori covariance matrix
  P = ( Eigen::MatrixXd::Identity(9,9) - K*H ) * M;
  // Return the measurement innovation
  double innovation = std::fabs( meas_dis - pred_d ); //距离残差

  // Update the state
  X = X + K * ( meas_dis - pred_d );

  //K = K * ( meas_dis - pred_d );
  //printf(" K : ");
  //std::cout << K.transpose() << std::endl;

  //3.============================状态保存===============================
  update_status( Eigen::Vector3d( X(0), X(3), X(6) ) ); //更新kf状态

  // decide to take the range info or not.
  MLOGI(" current innovation: %f  - distance_innovation_threshold_: %f ",
         innovation, distance_innovation_threshold_->data );
  // printf("position: %f %f %f \n", X(0), X(3), X(6) );
  // printf("velocity: %f %f %f \n", X(1), X(4), X(7) );
  // printf("acc bias: %f %f %f \n", X(2), X(5), X(8) );

  //根据距离残差判断滤波结果是否稳定可以一定程度上抑制NLOS
  Eigen::Vector3d current_position;
  Eigen::Vector3d current_velocity;
  Eigen::Vector3d current_acc_bias;
  //读取本次滤波器状态变量
  set_eigen_vector3d( X(0), X(3), X(6), current_position );
  set_eigen_vector3d( X(1), X(4), X(7), current_velocity );
  set_eigen_vector3d( X(2), X(5), X(8), current_acc_bias );
#if EXCLUDE_NLOS  //会丢失部分点,动态稳定性不好, 不能连续去除，否则滤波器将收敛失败，也就是一直存在偏差
  bool avalid  = judge_ekf_ret_by_innovation( kf_status_, innovation, distance_innovation_threshold_ );
  if( avalid ){
    position_ =  current_position;
    velocity_ =  current_velocity;
    acc_bias_ =  current_acc_bias;
    nine_state_cov_ = P;
    return MSUCCESS;
  }else {
    MLOGW("ekf update fial, distance innovation too large: %f ", innovation );
    return MFAIL;
  }
#else //不会丢失点，但可能引入大的误差,动态稳定性好
  position_ = current_position;
  velocity_ = current_velocity;
  acc_bias_ = current_acc_bias;
  nine_state_cov_ = P;
  return MSUCCESS;
#endif

}


