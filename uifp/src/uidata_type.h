#ifndef UIDATA_TYPE_H
#define UIDATA_TYPE_H

#include <math.h>
#include <vector>
#include <map>
#include "../../common/utility-math/meigen.hpp"

namespace uifp {

//--------------------------IMU--------------------
//IMU数据 //四元素+线性加速度
typedef struct ImuDataT{
  double_t timestamp;
  Eigen::Quaterniond quaterniond;
  Eigen::Vector3d linear_acc;
}ImuDataT;

typedef struct ImuDataListT{
  size_t max_size = 1;
  std::vector<ImuDataT> data;
}ImuDataListT;

//--------------------------Tag--------------------
///单个基站基本数据
typedef struct AnchorDataT{
  uint8_t role;
  uint8_t id;
  float_t dis;
  float_t fp_rssi;
  float_t rx_rssi;
}AnchorDataT;
///多个基站(指标签一次性获取到的基站数据)基本数据
typedef std::vector<AnchorDataT>  AnchorsDataT;

///单个标签+多个基站基本数据
///有的标签会包含IMU数据
typedef struct MultiAnchorsTagDataT{
  uint8_t tag_id;
  double_t timestamp;
  ImuDataT imu;
  Eigen::Vector3d pos_3d;
  AnchorsDataT anchors;
}MultiAnchorsTagDataT;


typedef struct MultiAnchorsTagDataListT{
  size_t max_size = 1;
  std::vector<MultiAnchorsTagDataT> data;
}MultiAnchorsTagDataListT;

//单个标签+单个基站基本数据
typedef struct SingleAnchorTagDataT{
  uint8_t tag_id;
  double_t timestamp;
  ImuDataT imu;
  Eigen::Vector3d pos_3d;
  AnchorDataT data;
}SingleAnchorTagDataT;

typedef struct SingleAnchorTagDataListT{
  size_t max_size = 1;
  std::vector<AnchorDataT> data;
}SingleAnchorTagDataListT;

//---------------uifp------------

#define M_GRAVITY  9.7925

///基站ID
typedef int AnchorIdT;
typedef std::vector<AnchorIdT> AnchorIdListT;
//存储地图中所有基站ID及其对应的坐标
typedef struct AnchorsMapT{
  AnchorIdListT id;
  std::map<AnchorIdT,std::vector<double>> map;
}AnchorsMapT;

///Anchor Filter Rssi Type参数
typedef struct AnchorsFilterRssiTypeParamsT{
  float_t dis_min;
  float_t dis_max;
  float_t drf_rssi_min;
  float_t drf_rssi_max;
}AnchorsFilterRssiTypeParamsT;

typedef struct AnchorsFilterSimpleTypeParamsT{
  float_t dis_min;
  float_t dis_max;
}AnchorsFilterSimpleTypeParamsT;

//--------------EKF---------------------------
typedef struct KfParametersT{
  double Q_scale;
  double R_scale;
  double acc_gauss_noise_sqrt;
  double acc_bias_noise_sqrt;
  double z_damping_factor;
  double meas_dis_err;
}KfParametersT;

typedef struct DynamicThresholdParametersT{
  double initial_value;
  double min;
  double max;
  double reduce_step;         //缩小步进
  double increase_step;       //增大步进
}DynamicThresholdParametersT;

//用于判定KF状态( stable or divergence )
typedef struct KfStatusCheckParametersT{
  size_t     size_max;             ///取点数最大值
  double     sd_threshold;         ///标准差阈值
}KfStatusCheckParametersT;

//------------UIFP------------------------
typedef struct PoseT{
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
}PoseT;


}  //end of namespace uifp

#endif // UIDATA_TYPE_H
