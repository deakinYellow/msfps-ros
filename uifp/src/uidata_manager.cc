
#include "../../common/utility-cc/tool.hpp"
#include "uidata_manager.h"

template<typename T>
void vector_pop_top( T& vec ){
  if( !vec.empty() ){
    vec.erase( vec.begin() );
  }
}


uifp::UiDataStatusL uifp::UiDataManager::get_status( void ){
  return data_status_;
}

void uifp::UiDataManager::update_tag( const MultiAnchorsTagDataT& tag_data ){
  ulong tag_data_size =  multi_anchors_tag_data_list_.data.size();
  //缓存区满则删除最前面的数据
  if( tag_data_size >= multi_anchors_tag_data_list_.max_size ){
    vector_pop_top<std::vector<uifp::MultiAnchorsTagDataT>>( multi_anchors_tag_data_list_.data );
  }
  multi_anchors_tag_data_list_.data.push_back( tag_data );
  //更新状态
  updata_status();
}

void uifp::UiDataManager::update_imu( const ImuDataT& imu_data ){
  ulong imu_data_size =  imu_data_list_.data.size();
  if( imu_data_size >= imu_data_list_.max_size ){
    vector_pop_top<std::vector<uifp::ImuDataT>>( imu_data_list_.data );
  }
  imu_data_list_.data.push_back( imu_data );
  //更新状态
  updata_status();
}


#if 0
int uifp::UiDataManager::get_uidata( MultiAnchorsTagDataT& tag_data ){
  return MSUCCESS;
}
int uifp::UiDataManager::get_imu( ImuDataT& imu_data ){
  return MSUCCESS;
}
#endif


///弹出最新数据，并更新状态
void uifp::UiDataManager::clear_top( void ){
  vector_pop_top<std::vector<uifp::MultiAnchorsTagDataT>>( multi_anchors_tag_data_list_.data );
  vector_pop_top<std::vector<uifp::ImuDataT>>( imu_data_list_.data );
  //更新状态
  updata_status();
}

///获取最新TAG和IMU数据
int uifp::UiDataManager::get_uidata( MultiAnchorsTagDataT& tag_data, ImuDataT& imu_data ,bool clear = true ){
  ///先判断当前数据状态
  if( DATA_STATUS_ALL_READY == get_status() ){
    tag_data = multi_anchors_tag_data_list_.data[ 0 ];
    imu_data = imu_data_list_.data[ 0];
    //获取完从缓冲区删除
    if( clear ){
      clear_top();
    }
    return MSUCCESS;
  }else {
    return MFAIL;
  }
}



//---------------------private----------------------------
void uifp::UiDataManager::set_status( const UiDataStatusL status ){
    data_status_ = status;
}

///更新数据状态
void uifp::UiDataManager::updata_status( void ){
  ulong tag_data_size =  multi_anchors_tag_data_list_.data.size();
  ulong imu_data_size =  imu_data_list_.data.size();
  if( tag_data_size >= multi_anchors_tag_data_list_.max_size
      && imu_data_size >= imu_data_list_.max_size ){
    set_status( DATA_STATUS_ALL_READY );
  }
  else if( tag_data_size < multi_anchors_tag_data_list_.max_size
           && imu_data_size  < imu_data_list_.max_size ){
    set_status( DATA_STATUS_WAITTING_ALL );
  }
  else if( tag_data_size >= multi_anchors_tag_data_list_.max_size
           &&  imu_data_size  < imu_data_list_.max_size ){
    set_status( DATA_STATUS_WAITTING_IMU );
  }
  else if( tag_data_size < multi_anchors_tag_data_list_.max_size
           &&  imu_data_size  >= imu_data_list_.max_size ){
    set_status( DATA_STATUS_WAITTING_TAG );
  }
}
