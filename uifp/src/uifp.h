#ifndef UIFP_H
#define UIFP_H

#include "../../mros/mros.h"
#include "../../mros/conversion.h"

#include "uifp_common.h"
#include "anchors_filter.h"
#include "uidata_type.h"
#include "uifp_ekf.h"


namespace uifp {

typedef enum UifpStatusL{
  UIFP_STATUS_UNINITIALIZED = 0x00,//未初始化的
  UIFP_STATUS_INITIALIZING,        //正在初始化的
  UIFP_STATUS_STABLE,              //工作中
  UIFP_STATUS_PAUSE                //暂停
}UifpStatusL;

/*
typedef enum YawTrackingWorkModeL{
  YAWTRACKING_WORKMODE_STOP = 0x00,
  YAWTRACKING_WORKMODE_INITIALIZE,
  YAWTRACKING_WORKMODE_QUICK_TRACKING,
  YAWTRACKING_WORKMODE_SLOW_TRACKING,
  YAWTRACKING_WORKMODE_PAUSE_TRACKING
}YawTrackingWorkModeL;
*/

typedef struct InitStatusTabT{
  bool anchors_filter = false;
  bool ekf = false;
}InitStatusTabT;


/**
 * @brief  UWB+IMU fusion position
 *  1.对uwb数据预处理，选取最优基站数据
 *  3.初始化EKF
 *  4.更新EKF输出定位数据
 * @note
 **/
class UiFp : public mros::NodeHandle {
public:
  explicit UiFp( std::string name_space ) : mros::NodeHandle( name_space ) {
    pose_.position = Eigen::Vector3d::Zero();
    pose_.orientation = Eigen::Quaterniond::Base();
  }

  void init_lintrack_visualization( const mros::VisualizationMarkerPubParametersT& params ){
    linktrack_position_track_publisher_ = std::shared_ptr<mros::VisualizationMarkerPublisher>(
         new mros::VisualizationMarkerPublisher( nh ) );
    linktrack_position_track_publisher_->Init( params );
  }

  void init_uifp_visualization( const mros::VisualizationMarkerPubParametersT& visualmarker_params,
                                const std::string tf_id ){
    uifp_position_track_publisher_ = std::shared_ptr<mros::VisualizationMarkerPublisher>(
         new mros::VisualizationMarkerPublisher( nh ) );
    uifp_position_track_publisher_->Init( visualmarker_params );
    uifp_tf_ = std::shared_ptr<mros::TF>( new mros::TF( visualmarker_params.frame_id ) );
    uifp_tf_id_ = tf_id;
  }

  void init_anchors_filter( const AnchorsMapT&  anchors_map,
                            const AnchorsFilterTypeL filter_type,
                            void* filter_params  );

  void init_ekf( Eigen::Vector3d initial_position,
                 const KfParametersT& ekf_params,
                 const DynamicThresholdParametersT& ekf_innovation_dynamic_threshold_params,
                 const KfStatusCheckParametersT& ekf_status_check_params );

  int stationary_update( const MultiAnchorsTagDataT& tag_data );

  //int start( void );
  int moving_update( const MultiAnchorsTagDataT& tag_data,
                     const ImuDataT& imu_data );


  UifpStatusL get_status( void ){
    return status_;
  }

  Eigen::Vector3d get_position( void ){
    return pose_.position;
  }
  Eigen::Quaterniond get_orientation( void ){
    return pose_.orientation;
  }


private:
  AnchorsMapT   anchors_map_;
  AnchorsFilter anchors_filter_;
  AnchorsFilterTypeL  anchors_filter_type_;
  void*  anchors_filter_params_;

  UifpEkf uifp_ekf_;
  //Eigen::Vector3d ekf_initial_position_,
  //KfParametersT ekf_params_;
  //DynamicThresholdParametersT  distance_innovation_dynamic_threshold_params_;
  //KfStatusCheckParametersT ekf_status_check_params_;

  ///ros visualization
  ///bool is_visualization_track_;
  //std::vector<mros::VisualizationMarkerPublisher> position_track_publishers_;
  std::shared_ptr<mros::VisualizationMarkerPublisher> linktrack_position_track_publisher_;
  std::shared_ptr<mros::VisualizationMarkerPublisher> uifp_position_track_publisher_;
  std::shared_ptr<mros::TF> uifp_tf_;
  std::string uifp_tf_id_;

  //初始化状态表
  InitStatusTabT init_status_tab_;

  void update_ekf( const MultiAnchorsTagDataT& tag_data,
                   const Eigen::Vector3d& map_acc );

  void update_visualization( void );
  void update_lintrack_visualization( const Eigen::Vector3d& position  );

  ///当前位姿
  PoseT pose_;

  void set_orientation( const Eigen::Quaterniond& orientation ){
    pose_.orientation = orientation;
  }
  void set_position( const Eigen::Vector3d& position ){
    pose_.position = position;
  }

  ///UIFP当前工作状态
  UifpStatusL status_ = UIFP_STATUS_UNINITIALIZED;

  void set_status( UifpStatusL status ){
    status_ = status;
  }
  void update_status( void );

};

} // end of namespace uifps

#endif // UIFP_H


