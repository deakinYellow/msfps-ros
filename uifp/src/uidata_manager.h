#ifndef UIDATA_MANAGER_H
#define UIDATA_MANAGER_H

#include <iostream>
#include "uidata_type.h"

namespace uifp {

typedef enum UiDataStatusL{
  DATA_STATUS_WAITTING_ALL = 0x01,
  DATA_STATUS_WAITTING_TAG,
  DATA_STATUS_WAITTING_IMU,
  DATA_STATUS_ALL_READY = 0x10
}UiDataStatusL;

/**
 * @brief  UWB 和　IMU 数据缓存管理器
 *         负责数据缓存更新,提供数据状态查询接口，提供特定数据
 * @note
 **/
class UiDataManager
{
public:
  explicit UiDataManager(){
    data_status_ = DATA_STATUS_WAITTING_ALL;
  }
  void update_tag( const MultiAnchorsTagDataT& tag_data );
  void update_imu( const ImuDataT& imu_data );
  UiDataStatusL get_status( void );

#if 0
  int get_tag( MultiAnchorsTagDataT& tag_data );
  int get_imu( ImuDataT& imu_data );
#endif
  void clear_top( void );
  int get_uidata( MultiAnchorsTagDataT& tag_data, ImuDataT& imu_data, bool clear );

private:

  MultiAnchorsTagDataListT  multi_anchors_tag_data_list_;
  ImuDataListT  imu_data_list_;
  UiDataStatusL data_status_;

  void set_status( const UiDataStatusL status );
  void updata_status( void );

};


}  //end of uifp


#endif // UIDATA_MANAGER_H

