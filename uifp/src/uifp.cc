
#define ENABLE_MLOGD
#include "../../common/utility-cc/tool.hpp"
#include "../../common/utility-cc/stool.hpp"

#include "uifp.h"

/**
 * @brief
 * @param  [in]
 * @param  [out]
 * @retval
 * @note
 **/
void uifp::UiFp::init_anchors_filter( const AnchorsMapT& anchors_map,
                                      const AnchorsFilterTypeL filter_type,
                                      void* filter_params  ){
  anchors_map_ = anchors_map;
  auto anchor_p = anchors_map_.map.at( 0 );
  MLOGI("test load from map: ( %f %f %f ) ", anchor_p[ 0 ], anchor_p[1], anchor_p[2] );
  anchors_filter_.init( anchors_map_.id );

  anchors_filter_type_ = filter_type;
  if( anchors_filter_type_ == ANCHORS_FILTER_TYPE_RSSI_TURNS ){
    anchors_filter_params_ = (AnchorsFilterRssiTypeParamsT*)(filter_params);
  }else if( anchors_filter_type_ == ANCHORS_FILTER_TYPE_SIMPLE_TURNS ){
    anchors_filter_params_ = (AnchorsFilterSimpleTypeParamsT*)(filter_params);
  }else {
    MLOGE_EXIT("invalid filter_type, no found.");
  }

  init_status_tab_.anchors_filter = true;
  update_status();
}

void uifp::UiFp::init_ekf( Eigen::Vector3d initial_position,
                           const KfParametersT& ekf_params,
                           const DynamicThresholdParametersT& distance_innovation_dynamic_threshold_params,
                           const KfStatusCheckParametersT& ekf_status_check_params ){
    //ekf_initial_position_ = initial_position_;
    //ekf_params_ = ekf_params;
    //ekf_innovation_dynamic_threshold_params_ = distance_innovation_dynamic_threshold_params;
    //ekf_status_check_params_ = ekf_status_check_params;
    uifp_ekf_.init( initial_position,
                    ekf_params,
                    distance_innovation_dynamic_threshold_params,
                    ekf_status_check_params );
    init_status_tab_.ekf = true;
    update_status();
}

///TODO: 更新ekf异常情况处理
void uifp::UiFp::update_ekf( const MultiAnchorsTagDataT& tag_data,
                             const Eigen::Vector3d& map_acc ){
  //------------选择最优基站数据----------
  AnchorDataT optimal_anchor_data;
  if( MSUCCESS != anchors_filter_.select_optimal( tag_data.anchors,
                                                  optimal_anchor_data,
                                                  anchors_filter_type_,
                                                  anchors_filter_params_ ) ){
    MLOGW("==anchors_filter select optimal fail, no available.");
    return;
  }
  double meas_dis = double( optimal_anchor_data.dis );
  AnchorIdT anchor_id = optimal_anchor_data.id;
  auto anchor_position = anchors_map_.map.at( anchor_id );
  Eigen::Vector3d anchor_p = mros::data_type_conversion::StdVector3dToEigenVector3d( anchor_position );
  double current_t = tag_data.timestamp;

  ///--------------更新ekf-------------------
  //优化方向:若当前基站数据更新失败，可选下一基站数据继续更新
  uifp_ekf_.update( anchor_p, meas_dis, map_acc, current_t );

  //获取ekf输出结果
  Eigen::Vector3d ekfout_p3d = uifp_ekf_.get_position();

  if( uifp_ekf_.get_status() == KF_STATUS_STABLE ){
    MLOGI("ekf out, position:( %f %f %f )",ekfout_p3d.x(), ekfout_p3d.y(), ekfout_p3d.z() );
    set_position( ekfout_p3d );
    set_status( UIFP_STATUS_STABLE );
  }
}


/**
 * @brief   更新UIFP,此时Acc一般为０
 * @param  [in]   tag_data   标签数据
 * @retval
 * @note
 **/
int uifp::UiFp::stationary_update( const MultiAnchorsTagDataT& tag_data ){
  //先更新工作状态
  update_status();
  if( get_status() == UIFP_STATUS_UNINITIALIZED ){
    MLOGW("uifp status is uninitialized.");
    return MFAIL;
  }else if( get_status() == UIFP_STATUS_PAUSE ){
    MLOGI("uifp status is pause.");
    return MFAIL;
  }
  //-------------处理IMU数据--------------
  Eigen::Vector3d map_acc = Eigen::Vector3d::Zero();
  MLOGD( "map_acc( %f %f %f )", map_acc.x(), map_acc.y(), map_acc.z() );
  //更新ekf
  update_ekf( tag_data, map_acc );
  //更新visualization
  update_lintrack_visualization( tag_data.pos_3d );
  update_visualization();
  return MSUCCESS;
}



/**
 * @brief   更新UIFP,此时Acc一般不为０
 * @param  [in]   tag_data   标签数据
 * @param  [in]   imu_data   IMU数据,注意取linear_acc,并且IMU姿态要和map坐标系对齐
 * @retval
 * @note
 **/
int uifp::UiFp::moving_update( const MultiAnchorsTagDataT& tag_data,
                               const ImuDataT& imu_data ){

  //先更新工作状态
  update_status();
  if( get_status() == UIFP_STATUS_UNINITIALIZED ){
    MLOGW("uifp status is uninitialized.");
    return MFAIL;
  }else if( get_status() == UIFP_STATUS_PAUSE ){
    MLOGI("uifp status is pause.");
    return MFAIL;
  }

  //-------------处理IMU数据--------------
  //获取map坐标系下acc
  Eigen::Vector3d map_acc;
  Eigen::Vector3d origin_acc = imu_data.linear_acc;
  //Eigen::Quaterniond pose_q = imu_data.quaterniond;
  map_acc = meigen::Vector3dTransformByQuaterniond( origin_acc , imu_data.quaterniond );
  //加速度限制幅度
  double max_acc_value = 0.5;
  stool::limit_value<double>( -max_acc_value, max_acc_value, map_acc.x() );
  stool::limit_value<double>( -max_acc_value, max_acc_value, map_acc.y() );
  stool::limit_value<double>( -max_acc_value, max_acc_value, map_acc.z() );
  map_acc *= M_GRAVITY;  //注意单位转为m/s^2
  MLOGD( "map_acc( %f %f %f )", map_acc.x(), map_acc.y(), map_acc.z() );
  //更新姿态
  set_orientation( imu_data.quaterniond );
  //更新ekf
  update_ekf( tag_data, map_acc );
  //更新visualization
  update_lintrack_visualization( tag_data.pos_3d );
  update_visualization();

  return MSUCCESS;
}


void uifp::UiFp::update_status( void ){
  switch( get_status() ) {
  case UIFP_STATUS_UNINITIALIZED:
    if( init_status_tab_.anchors_filter && init_status_tab_.ekf ){
      set_status( UIFP_STATUS_INITIALIZING );
    }
    else {
      if( !init_status_tab_.anchors_filter ){
        MLOGW("anchors_filter is uninitialized.");
      }
      if( !init_status_tab_.ekf ){
        MLOGW("ekf is uninitialized.");
      }
    }
    break;
  }
}


///更新 visualization
void  uifp::UiFp::update_visualization(){
  if( uifp_position_track_publisher_ != nullptr &&
      uifp_tf_ != nullptr &&
      get_status() == UIFP_STATUS_STABLE ){

    Eigen::Vector3d pos = get_position();
    pos.z() = 0;
    geometry_msgs::Point uifp_tag_p = mros::data_type_conversion::EigenPointToGeometryMsgPoint( pos );
    uifp_position_track_publisher_->AddPoint( uifp_tag_p );
    geometry_msgs::Pose tf_pose = mros::data_type_conversion::EigenPoseToGeometryMsgPose( pos, get_orientation()  );
    uifp_tf_->SendTransform( uifp_tf_id_ , tf_pose );
  }
}


///更新 linktrack visualization
void uifp::UiFp::update_lintrack_visualization( const Eigen::Vector3d& position  ){
  if( linktrack_position_track_publisher_ != nullptr ){
    geometry_msgs::Point linktarck_tag_p = mros::data_type_conversion::EigenPointToGeometryMsgPoint( position );
    linktarck_tag_p.z = 0.0;
    linktrack_position_track_publisher_->AddPoint( linktarck_tag_p );
  }
}

