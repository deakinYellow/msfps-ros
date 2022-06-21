//#define ENABLE_MLOGD
#include "../../common/utility-cc/tool.hpp"

#include "../../mros/data_type_conversion.hpp"

#include "uidata_receive.h"


void uifp::UiDataReceive::tag_callback( const nlink_parser::LinktrackNodeframe2::ConstPtr msg ){
#if 1
  tag_callback_fps_.sampling();
  MLOGD("topic: %s callback, fps: %f", tag_sub_topic_.c_str(), tag_callback_fps_.get_real_fps() );
  if( msg == nullptr ){
    return;
  }
  //------------anchors 数据------------------
  auto nodes_msg = msg->nodes;
  if( nodes_msg.empty() ){
    MLOGW("tag00: nodes_msg is empty.");
    return;
  }
  MLOGD("tag: all %ld nodes(anchors) info recived.", nodes_msg.size() );
  AnchorDataT  anchor_data;
  MultiAnchorsTagDataT multi_anchors_tag_data;
  multi_anchors_tag_data.timestamp = ros::Time::now().toSec();
  for( auto node : nodes_msg ) {
    anchor_data.role = node.role;
    anchor_data.id = node.id;
    anchor_data.dis = node.dis;
    anchor_data.fp_rssi = node.fp_rssi;
    anchor_data.rx_rssi = node.rx_rssi;
    multi_anchors_tag_data.anchors.push_back( anchor_data );
  }
  //----------------位置数据 ---------------------
  Eigen::Vector3d pos_3d;
  pos_3d.x() = double( msg->pos_3d[0] );
  pos_3d.y() = double( msg->pos_3d[1] );
  pos_3d.z() = double( msg->pos_3d[2] );
  multi_anchors_tag_data.pos_3d = pos_3d;
  ////数据缓存区更新
  data_manager_->update_tag( multi_anchors_tag_data );

#endif
}


void uifp::UiDataReceive::imu_callback( const sensor_msgs::Imu::ConstPtr msg ){
#if 1
  if( msg == nullptr ){
    return;
  }
  //MLOGI("addr 0: %X", &fps00 );
  imu_callback_fps_.sampling();
  MLOGD("topic: %s, callback fps: %f", imu_sub_topic_.c_str(), imu_callback_fps_.get_real_fps() );
  ImuDataT imu_data;
  imu_data.timestamp = msg->header.stamp.toSec();
  imu_data.linear_acc = mros::data_type_conversion::GeometryMsgVector3ToEigenVectord( msg->linear_acceleration);
  imu_data.quaterniond = mros::data_type_conversion::GeometryMsgQuaternionToEigenQuaterniond( msg->orientation );
  data_manager_->update_imu( imu_data );
#endif
}

