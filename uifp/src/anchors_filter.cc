
#define ENABLE_MLOGD
#include "../../common/utility-cc/tool.hpp"
#include "../../common/utility-cc/stool.hpp"

#include "anchors_filter.h"


void uifp::AnchorsFilter::init( const AnchorIdListT& current_region_id_list ){
  current_region_id_list_.assign( current_region_id_list.begin(), current_region_id_list.end() );
  //直接将prefer_id_list初始化为当前区域ID列表
  prefer_id_list_.assign( current_region_id_list_.begin(), current_region_id_list_.end() );
}

///判断距离大小是否超出设定返回
bool simple_check(  const uifp::AnchorDataT& anchor_data, void* params ){
  MLOGD("distance checking, anchor id: %d ,dis: %.2f ", anchor_data.id, double(anchor_data.dis) );
  uifp::AnchorsFilterSimpleTypeParamsT* p = (uifp::AnchorsFilterSimpleTypeParamsT*)(params);
  float_t dis =  anchor_data.dis;
  ////判断距离是否符合条件
  if( dis > p->dis_max || dis < p->dis_min ){
    return false;
  }
  return true;
}

/**
 * @brief
 *   每个标签到基站测量的信号强度信息，以下为官方文档描述
 *   在协议中表示为:fp_rssi 与 rx_rssi:
 *       Node 可以输出所接收到第一路径信号强度指示 fp_rssi 与总接收信号强度指示 rx_rssi 构成。
 *   通常与距离一起输出(如 LP Mode 下标签的距离输出,DR Mode 下节点的距离输出)。一般情
 *   况下,当“rx_rssi - fp_rssi”小于 6dB 时,很有可能处于 LOS 状态,当大于 10dB 时,很有可能
 *   处于 NLOS 或多径状态(其中:rx_rssi为总接收强度，fp_rssi为第一路径接收强度)
 *   令 drf_rssi = rx_rssi - fp_rssi, 将此值作为一个判断依据，数据异常的做剔除,初步设置为(2.0-10.5),主要目的去除NLOS和多径
 * @param  [in]   anchor_data  当前基站信息
 * @param  [in]   params       算法参数
 * @retval
 * @note
 **/
bool  rssi_check( const uifp::AnchorDataT& anchor_data,
                  void* params  ){
  uifp::AnchorsFilterRssiTypeParamsT* p = (uifp::AnchorsFilterRssiTypeParamsT*)(params);
  float_t drf_rssi =  anchor_data.rx_rssi - anchor_data.fp_rssi;
  MLOGD("anchor id: %d ,  drf_rssi: %f ", anchor_data.id, double(drf_rssi) );
  float_t dis =  anchor_data.dis;
  ////判断距离是否符合条件
  if( dis > p->dis_max || dis < p->dis_min ){
    return false;
  }
  ////判断信号是否符合条件
  if( drf_rssi > p->drf_rssi_max || drf_rssi < p->drf_rssi_min ){
    return false;
  }
  return true;
}

int uifp::AnchorsFilter::select_optimal( const AnchorsDataT& anchors_data,
                                         AnchorDataT& optimal_anchor_data,
                                         const AnchorsFilterTypeL filter_type,
                                         void* filter_params ){
  if( filter_type == ANCHORS_FILTER_TYPE_SIMPLE_TURNS ){
    return select_optimal_turns( anchors_data, optimal_anchor_data, simple_check, filter_params );
  }
  else if( filter_type == ANCHORS_FILTER_TYPE_RSSI_TURNS ){
    return select_optimal_turns( anchors_data, optimal_anchor_data, rssi_check, filter_params );
  }
  else {
    MLOGE_EXIT("filter type invalid!");
    return MFAIL;
  }
}


/*
 * @brief
 *   目前LinkTrack LPMode模式下,每次测量会同时返回标签到多个基站的测距信息，uifp只需要其中一个作为输入，因此
 *   需要选取，这里采取简单的策略: 按优先级查找队列最前面的，本轮选中之后排到最后。
 * @param  [in]   anchors_data         标签一次返回的多个基站信息
 * @param  [out]  optimal_anchor_data  输出的最优基站信息
 * @retval
 * @note
 **/
int  uifp::AnchorsFilter::select_optimal_turns( const AnchorsDataT& anchors_data,
                                                AnchorDataT& optimal_anchor_data,
                                                anchor_data_is_valid is_valid,
                                                void* is_valid_param ){
  if( anchors_data.empty() ){
    return  MFAIL;
  }
  for( ulong i = 0; i < prefer_id_list_.size(); i++ ) {
    //寻找当前优先级最高id信息
    AnchorIdT prefer_id = prefer_id_list_[ i ];
    //MLOGD("prefer anchor id: %d ", prefer_id );
    AnchorDataT anchor_data;
    if( MSUCCESS == find_special_id( anchors_data, anchor_data, prefer_id ) ){
      float_t drf_rssi =  anchor_data.rx_rssi - anchor_data.fp_rssi;
      MLOGD("anchor id: %d ,  drf_rssi: %f ", anchor_data.id, double(drf_rssi) );
      ////判断该基站信息是否符合条件
      if( is_valid( anchor_data, is_valid_param ) ){
        MLOGD("seleted anchor id: %d, dis: %.2f, drf_rssi: %f.", anchor_data.id,
              double(anchor_data.dis),
              double(drf_rssi) );
        //输出
        optimal_anchor_data = anchor_data;
        //更新prefer_id_list
        prefer_id_list_.erase( prefer_id_list_.begin() + int(i) );
        prefer_id_list_.push_back( prefer_id );
        return MSUCCESS; //成功找到返回
      }
      //不符合则继续寻找下一个优先级ID
      else {
        MLOGW("invalid anchor_data id: %d, dis: %.2f drf_rssi: %f ", prefer_id,
              double(anchor_data.dis),
              double(drf_rssi) );
        continue;
      }
    }
  }
  return MFAIL;  //全部可选ID都没有符合条件的
}



//----------------------------------private---------------------------------------------
///寻找指定ID的数据///若找不到返回MFAIL
int uifp::AnchorsFilter::find_special_id( const AnchorsDataT& anchors_data,
                                          AnchorDataT& anchor_data, const AnchorIdT id ){
  for( auto info : anchors_data ) {
    if( info.id == id ){
      anchor_data.id = info.id;
      anchor_data.dis = info.dis;
      anchor_data.fp_rssi = info.fp_rssi;
      anchor_data.rx_rssi = info.rx_rssi;
      return  MSUCCESS;
    }
  }
  return  MFAIL;
}
