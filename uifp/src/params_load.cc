
#include "../../common/utility-cc/tool.hpp"
#include "../../common/utility-cc/stool.hpp"
#include "params_load.h"

/**
 * @brief   加载基站地图(基站ID及其对应的坐标)
            需要配置文件格式对应，示例如下:
            anchor_list:  [ 10, 11 ]   ##基站ID列表
            anchor_00010: [0.0, 0.0, 2.631 ]   ##　ID:10　对应的坐标
            anchor_00011: [ 5.239, -2.767, 2.654]
 *
 * @param  [in]   parameter      ros参数加载对象
 * @param  [out]  anchors_map    输出 anchors_map
 * @retval
 * @note
 **/
void load_anchors_map( mros::Parameter* parameter, uifp::AnchorsMapT& anchors_map ){
  //先获取地图基站ID
  if( !parameter->Get<std::vector<uifp::AnchorIdT>>("anchor_list", anchors_map.id ) ){
    ROS_ERROR("param anchor_list, get fail!");
    exit(1);
  }
  for( int anchor_id : anchors_map.id ) {
    ROS_INFO("anchor : %d ", anchor_id );
    std::string anchor_id_str;
    stool::int2string( anchor_id,5, anchor_id_str );
    std::string anchor_position_key = "anchor_" + anchor_id_str;
    ROS_INFO("anchor_position_key: %s ", anchor_position_key.c_str() );
    std::vector<double> anchor_position;
    if( parameter->Get<std::vector<double>>( anchor_position_key, anchor_position ) ){
      ROS_INFO("insert anchor:( %f %f %f )", anchor_position[ 0 ], anchor_position[1], anchor_position[2] );
      anchors_map.map.insert(std::pair<uifp::AnchorIdT,std::vector<double>>( anchor_id, anchor_position ));
      auto anchor_p = anchors_map.map.at( anchor_id );
      ROS_INFO("test load from map: ( %f %f %f ) ", anchor_p[ 0 ], anchor_p[1], anchor_p[2] );
    }else {
      ROS_ERROR("param %s, get fail!", anchor_position_key.c_str() );
      exit(1);
    }
  }
}


void load_kf_params( mros::Parameter* parameter,
                     Eigen::Vector3d&  initial_position,
                     uifp::KfParametersT& kf_params,
                     uifp::DynamicThresholdParametersT& distance_innovation_dynamic_threshold_params,
                     uifp::KfStatusCheckParametersT& kf_status_check_params ){
#if 1
  //-----------------KF params----------------------------------------
  std::vector<double> position;
  if( parameter->Get<std::vector<double>>("initial_position", position ) ){
    initial_position.x() = position[0];
    initial_position.y() = position[1];
    initial_position.z() = position[2];
  } else {
    ROS_ERROR("initial_position , get fail!");
    exit(1);
  }

  XmlRpc::XmlRpcValue kf_parameters;  //xml数组格式
  if( parameter->Get("kf_parameters", kf_parameters ) ){
    //key 一定要写对，不然会获取到0
    ROS_INFO("kf Q: %f", double( kf_parameters[0]["Q_scale"] ) );
    ROS_INFO("kf R: %f", double( kf_parameters[0]["R_scale"] ) );
    kf_params.Q_scale =  double( kf_parameters[0]["Q_scale"] );
    kf_params.R_scale =  double( kf_parameters[0]["R_scale"] );
    kf_params.acc_gauss_noise_sqrt = double( kf_parameters[0]["acc_gauss_noise_sqrt"] );
    kf_params.acc_bias_noise_sqrt = double( kf_parameters[0]["acc_bias_noise_sqrt"] );
    kf_params.z_damping_factor = double( kf_parameters[0]["z_damping_factor"] );
    kf_params.meas_dis_err = double( kf_parameters[0]["meas_dis_err"] );

  }else {
    ROS_ERROR("kf_parameters , get fail!");
    exit(1);
  }

  XmlRpc::XmlRpcValue dis_dynamic_threshold_parameters;
  if( parameter->Get("distance_innovation_dynamic_threshold_parameters", dis_dynamic_threshold_parameters ) ){
    distance_innovation_dynamic_threshold_params.initial_value =  double( dis_dynamic_threshold_parameters[0]["initial_value"] );
    distance_innovation_dynamic_threshold_params.min =  double( dis_dynamic_threshold_parameters[0]["min"] );
    distance_innovation_dynamic_threshold_params.max =  double( dis_dynamic_threshold_parameters[0]["max"] );
    distance_innovation_dynamic_threshold_params.reduce_step =  double( dis_dynamic_threshold_parameters[0]["reduce_step"] );
    distance_innovation_dynamic_threshold_params.increase_step =  double( dis_dynamic_threshold_parameters[0]["increase_step"] );
  }else {
    ROS_ERROR("distance_innovation_dynamic_threshold_parameters, get fail!");
    exit(1);
  }
  XmlRpc::XmlRpcValue kf_status_check_parameters;
  if( parameter->Get("kf_status_check_parameters", kf_status_check_parameters ) ){
    kf_status_check_params.size_max = int(kf_status_check_parameters[0]["size_max"]);
    kf_status_check_params.sd_threshold =  double( kf_status_check_parameters[0]["sd_threshold"] );
    ROS_INFO("kf_status_check_params:( %ld %.2f)", kf_status_check_params.size_max, kf_status_check_params.sd_threshold );
  }else {
    ROS_ERROR("kf_status_check_parameters");
    exit(1);
  }
#endif
}




