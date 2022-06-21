#ifndef QYXK_UWB_HPP
#define QYXK_UWB_HPP

#include "../../common/utility-cc/tool.hpp"
#include "../../common/utility-cc/stool.hpp"
#include "../uart/uart_frame_M0.h"


class QyxkUwb{

public:

  typedef struct UwbTagPositionT{
    uint tagid;
    int px;
    int py;
    int pz;
    uint64_t timestamp;
  }UwbTagPositionT;

  typedef struct UwbTagDistanceT{
    uint16_t tagid;
    uint16_t anchorid;
    uint distance;
    uint64_t timestamp;
  }UwbTagDistanceT;


  typedef struct UwbTagPositionsBuffT{
    std::queue<UwbTagPositionT> data;
    uint max = 16;
    std::mutex mutex;
  }UwbTagPositionsBuffT;

  typedef struct UwbTagDistancesBuffT{
    std::queue<UwbTagDistanceT> data;
    uint max = 16;
    std::mutex mutex;
  }UwbTagDistancesBuffT;

  UwbTagPositionsBuffT* tag_positions_buff = ( new UwbTagPositionsBuffT );
  UwbTagDistancesBuffT* tag_distances_buff = ( new UwbTagDistancesBuffT );

  QyxkUwb( std::string dev_id, uint baud_rate ) : dev_id_( dev_id ), baud_rate_( baud_rate ){
    ;
  }

  void StartProcess( void ){
    pthread_create( &tid_, nullptr, _thread_t<QyxkUwb, &QyxkUwb::process_>, this );
  }

  void process_( void ){
    using std::cout;
    using std::endl;

    uart_fh_.SetUartParams( dev_id_ , baud_rate_ );

    if( MSUCCESS != uart_fh_.OpenDevice() ){
      std::cout <<"open " << dev_id_ << " fail!"<< std::endl;
      std::cout <<"please check params or hardware." << std::endl;
      return;
    }
    uart_fh_.StartProcess();
    std::array<uint8_t,2> header{ 0xA5 , 0x5A };
    std::array<uint8_t,2> end{ 0x55 , 0xAA };

    std::vector<uint>  frame_len_list;
    frame_len_list.push_back( distance_frame_lenth_ );
    frame_len_list.push_back( position_frame_lenth_ );

    uart_fh_.SetFrameStruct( header, end, frame_len_list );

    std::shared_ptr<muart::UartFrameT> frame =
        std::shared_ptr<muart::UartFrameT>( new muart::UartFrameT );

    UwbTagPositionT tag_position;
    UwbTagDistanceT tag_distan;

    uint count_frames = 0;
    while( !stop_process_ ){
      if( 0 == uart_fh_.GetOneFrame( frame ) ){
        //位置信息
        if( (*frame)[ 2 ] == 0x2A ){
          if( MSUCCESS == decodePositionFrame( frame,tag_position ) ){
            //printf("decode position success, %d \n", ++count_frames );
            pushTagPosition( tag_position );
          }
          else {
            printf("decode position fail!\n");
          }
        }
        //距离信息
        else if( (*frame)[ 2 ] == 0x2D ){
          if( MSUCCESS == decodeDistanceFrame( frame,tag_distan ) ){
            //printf("decode distance success, %d \n", ++count_frames );
            pushTagDistance( tag_distan );
          }
          else {
            printf("decode distance fail!\n");
          }
        }
      }
      else {
        //printf("2 get frame fail! \n" );
      }
      M_USLEEP(500);  //sleep, neccssary! (延时等待数据到达(1ms左右)，频繁判断会导致数据无法写入)
    }
    //关闭串口数据接收处理
    uart_fh_.StopProcess();
  }

  void StopProcess( void ){
    stop_process_ = true;
  }
  ~QyxkUwb( void ){
    delete tag_distances_buff;
    delete tag_positions_buff;
    //std::cout << "~ QyxkUwb " << std::endl;
    StopProcess();
    pthread_join( tid_, nullptr );
  }

private:

  pthread_t tid_;
  std::string dev_id_;
  uint baud_rate_;
  bool stop_process_ = false;

  uint distance_frame_lenth_ = 27;
  uint position_frame_lenth_ = 33;

  muart::UartFrameH  uart_fh_;

  void pushTagPosition( const UwbTagPositionT position ){
    //加锁
    std::unique_lock<std::mutex> lock( tag_positions_buff->mutex );
    if( tag_positions_buff->data.size() >= tag_positions_buff->max ){
      tag_positions_buff->data.pop();
    }
    tag_positions_buff->data.push( position );
  }

  void pushTagDistance( const UwbTagDistanceT distance ){
    //加锁
    std::unique_lock<std::mutex> lock( tag_distances_buff->mutex );
    if( tag_distances_buff->data.size() >= tag_distances_buff->max ){
      tag_distances_buff->data.pop();
    }
    tag_distances_buff->data.push( distance );
  }

  bool checkSumOk( const std::shared_ptr<muart::UartFrameT> frame ){
    if( !frame ){
      std::cout << "checkSumOk: nullptr" << std::endl;
      return false;
    }
    uint8_t r_checksum = (*frame)[ frame->size() - 3 ];  //倒数第3个为校验和位置
    uint8_t checksum = 0;
    //计算方法： 除去帧头帧尾以及校验和本身，其余相加
    for( auto it = frame->begin() + 2; it < frame->end() - 3; it++ ){
      checksum += *it;
    }
    //printf("r_checksum, check_sum : %.2X %.2X \n", r_checksum, checksum );
    if( r_checksum == checksum ){
      return true;
    }
    else {
      return false;
    }
  }

  int decodePositionFrame( const std::shared_ptr<muart::UartFrameT> frame,
                           UwbTagPositionT& p ){
    if( !frame ){
      std::cout << "decodePositionFrame: nullptr" << std::endl;
      return MFAIL;
    }
    //1.判断帧长度!
    if( position_frame_lenth_ != frame->size() ){
      std::cout << "position frame length incorect!" << std::endl;
      //exit(1);
      return MFAIL;
    }
    //2.校验!
    if( !checkSumOk( frame ) ){
      std::cout << "checksum incorrect!" << std::endl;
      return MFAIL;
    }
    auto it = frame->begin();
    it = frame->begin() + 4;  //标签ID起始
    std::vector<uint8_t> tagid_d(it, it+ 2);
    p.tagid = stool::vectorU82integerT<uint>( tagid_d  );

    it = frame->begin() + 6;  //px起始
    std::vector<uint8_t> px_d(it, it + 4);
    p.px = stool::vectorU82integerT<int>( px_d  );

    it = frame->begin() + 10;
    std::vector<uint8_t> py_d(it, it + 4);
    p.py = stool::vectorU82integerT<int>( py_d  );

    it = frame->begin() + 14;
    std::vector<uint8_t> pz_d(it, it + 4);
    p.pz = stool::vectorU82integerT<int>( pz_d  );

    it = frame->begin() + 18;
    std::vector<uint8_t> timestamp_d(it, it + 8 );
    p.timestamp = stool::vectorU82integerT<uint64_t>( timestamp_d  );
    return  MSUCCESS;
  }

  int decodeDistanceFrame( const std::shared_ptr<muart::UartFrameT> frame,
                           UwbTagDistanceT& d ){
    if( !frame ){
      std::cout << "decodeDistanceFrame: nullptr" << std::endl;
      return MFAIL;
    }
    //1.判断帧长度!
    if( distance_frame_lenth_ != frame->size() ){
      std::cout << "distance frame length incorect!" << std::endl;
      //exit(1);
      return MFAIL;
    }
    //2.校验!
    if( !checkSumOk( frame ) ){
      std::cout << "checksum incorrect!" << std::endl;
      return MFAIL;
    }
    auto it = frame->begin();
    it = frame->begin() + 4;  //标签ID起始
    std::vector<uint8_t> tagid_d(it, it+ 2);
    d.tagid = stool::vectorU82integerT<uint16_t>( tagid_d  );

    it = frame->begin() + 6;  //基站ID起始
    std::vector<uint8_t> anchorid_d(it, it + 2 );
    d.anchorid = stool::vectorU82integerT<uint16_t>( anchorid_d  );

    it = frame->begin() + 8;
    std::vector<uint8_t> distance_d(it, it + 2);
    d.distance = stool::vectorU82integerT<uint>( distance_d  );

    it = frame->begin() + 12;  //中间还预留2个，时间戳从12开始
    std::vector<uint8_t> timestamp_d(it, it + 8 );
    d.timestamp = stool::vectorU82integerT<uint64_t>( timestamp_d  );

    return  MSUCCESS;
  }

};


static void qyxkuwb_test( void ){

  QyxkUwb uwb("/dev/uwb", 115200 );
  uwb.StartProcess();

  QyxkUwb::UwbTagPositionT p;
  QyxkUwb::UwbTagPositionsBuffT* ps;
  ps = uwb.tag_positions_buff;

  QyxkUwb::UwbTagDistanceT d;
  QyxkUwb::UwbTagDistancesBuffT* ds;
  ds = uwb.tag_distances_buff;
  uint p_count=0;
  uint d_count=0;

  stool::FPS fps_d;
  long last_stamp = 0;

  while( true ){
    //printf("------------------------------\n");
    if( ps ){
      std::unique_lock<std::mutex> ps_lock( ps->mutex );  //!!!
      if( !ps->data.empty() ){
        p = ps->data.front() ;
        //printf("position %d : %d %d %d \n", ++p_count, p.px, p.py, p.pz );
        ps->data.pop();
#if 1
        long stamp = tool::sys_ms_timestamp();
        long dt = stamp - last_stamp;
        last_stamp = stamp;
        if( dt > 60 ){
          printf("ps dt: %ld ms\n", dt );
        }
#endif
      }
      //ps_lock.unlock();
    }

    if( ds ){
      std::unique_lock<std::mutex> ds_lock( ds->mutex );  ///!!!
      if( !ds->data.empty() ){
        d = ds->data.front();
        //printf("%d: tag %d to anchor %d distance: %d cm\n", ++d_count, d.tagid, d.anchorid, d.distance );
        fps_d.sampling();
        //printf("FPS, normal: %d  avr: %.2f  real: %.2f \n", fps_d.fps_normal, fps_d.fps_average, fps_d.fps_real );
        ds->data.pop();
#if 0
        long stamp = stool::sys_ms_ts();
        long dt = stamp - last_stamp;
        last_stamp = stamp;
        //printf("ds dt: %ld ms\n", dt );
        if( dt > 150 ){
          printf("ds dt: %ld ms\n", dt );
        }
#endif
      }
      //ds_lock.unlock();
    }
    M_USLEEP(1000);
  }
}

#endif // QYXK_UWB_HPP

