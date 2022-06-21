#ifndef ANCHORS_FILTER_H
#define ANCHORS_FILTER_H

#include <iostream>
#include <vector>
#include <math.h>

#include "uidata_type.h"

/*
 * 基本现有的UWB设备采集的距离数据都是短时间内采集标签到多个基站的信息然后统一回传.
 * 使用IMU和距离数据做紧耦合的好处在于每次只需要到一个基站的距离数据，就可以做一次滤波器的更新.
 * 因此需要一个算法对得到的多个基站数据进行筛选，选出最合适的一个基站数据进行本轮滤波器更新。
 */


namespace uifp {

class AnchorsFilter;
//typedef bool ( AnchorsFilter::*anchor_data_is_valid)( void*, void* );
typedef bool (*anchor_data_is_valid)( const AnchorDataT& , void* );


typedef enum AnchorsFilterTypeL{
  ANCHORS_FILTER_TYPE_SIMPLE_TURNS = 0x01,
  ANCHORS_FILTER_TYPE_RSSI_TURNS,
}AnchorsFilterTypeL;


class AnchorsFilter
{
public:
  AnchorsFilter(){
    ;
  }
  //初始化设置当前可选基站列表
  void init( const AnchorIdListT& current_region_id_list );
  int  select_optimal( const AnchorsDataT& anchors_data,
                       AnchorDataT& optimal_anchor_data,
                       const AnchorsFilterTypeL filter_type,
                       void* filter_params );

private:
  AnchorIdListT current_region_id_list_;   ///当前工作区域基站ID列表
  AnchorIdListT prefer_id_list_;           ///当前最优备选ID列表

  ///通过预定顺序轮流选择最优基站
  int  select_optimal_turns( const AnchorsDataT& anchors_data,
                             AnchorDataT& optimal_anchor_data,
                             anchor_data_is_valid is_valid,
                             void* is_valid_param );
  ///通过信号强度选择最优基站
  int  select_optimal_rssi( const AnchorsDataT& anchors_data,
                            AnchorDataT& optimal_anchor_data,
                            const AnchorsFilterRssiTypeParamsT& params );

  int find_special_id( const AnchorsDataT& anchors_data, AnchorDataT& anchor_data, const AnchorIdT id );


};


}  //end of namespace uifp

#endif // ANCHORS_FILTER_H

