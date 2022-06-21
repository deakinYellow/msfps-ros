#ifndef UIFP_EKF_H
#define UIFP_EKF_H

#include <vector>
#include "../../common/utility-math/meigen.hpp"
#include "../../common/utility-math/mdatafilter.hpp"

#include "uidata_type.h"

/*
 * brife: EKF对UWB测距信息，和IMU加速度信息进行紧耦合
 *        估算：标签当前位置，速度，和加速度计误差
 * note:
 */
namespace uifp {

typedef enum KfStatusL{
  KF_STATUS_UNINITIALIZED = 0x00,  //未初始化的
  KF_STATUS_INITIALIZING ,   //初始化中
  KF_STATUS_DIVERGENCE,      //发散
  KF_STATUS_STABLE = 0x10,   //正常工作状态
}KfStatusL;

//动态阈值
typedef struct DynamicThresholdT{
  double data;
  DynamicThresholdParametersT params;
}DynamicThresholdT;


typedef struct KfStatusCheckDataT{
    std::vector<Eigen::Vector3d> data;
    Eigen::Vector3d sd;              //标准差
    KfStatusCheckParametersT  params;
}KfStatusCheckDataT;
//------------------------------------


class UifpEkf{
public:
  UifpEkf() {
    ;
  }
  void init( const Eigen::Vector3d& initial_position,
             const KfParametersT& kf_params,
             const DynamicThresholdParametersT& distance_innovation_dynamic_threshold_parameters ,
             const KfStatusCheckParametersT&  kf_status_check_params );

  //bool Updata( void );
  int update( const Eigen::Vector3d& anchor_p,
              const double meas_d,
              const Eigen::Vector3d& acc,
              const double current_t );


  KfStatusL get_status( void ){
    return kf_status_;
  }

  Eigen::Vector3d get_position( void ){
    return position_;
  }
  Eigen::Vector3d get_velocity( void ){
    return velocity_;
  }
  Eigen::Vector3d get_accbias( void ){
    return acc_bias_;
  }

  ~UifpEkf(){
    delete kf_params_;
    delete distance_innovation_threshold_;
    delete data_for_check_kf_status_;
  }

private:

  KfStatusL kf_status_ = KF_STATUS_UNINITIALIZED;
  DynamicThresholdT* distance_innovation_threshold_ = ( new DynamicThresholdT ); //测距动态残差阈值
  KfParametersT* kf_params_=( new KfParametersT ); //ekf参数

  //待估计状态
  Eigen::Vector3d velocity_;
  Eigen::Vector3d position_;
  Eigen::Vector3d acc_bias_;

  Eigen::Vector3d last_velocity_;
  Eigen::Vector3d last_position_;
  Eigen::Vector3d last_acc_bias_;
  //协方差矩阵
  Eigen::MatrixXd nine_state_cov_;
  double last_update_t_ = 0;

  //用于判断当前定位状态
  KfStatusCheckDataT* data_for_check_kf_status_= ( new KfStatusCheckDataT );
  //KF状态更新
  void update_status( const Eigen::Vector3d& position );

  void set_eigen_vector3d( const double x, const double y, const double z, Eigen::Vector3d& v ){
    v.x() = x;
    v.y() = y;
    v.z() = z;
  }

};

} ///end of namespace uifp

#endif // UIFP_EKF
