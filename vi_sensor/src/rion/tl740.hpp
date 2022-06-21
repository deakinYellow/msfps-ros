#ifndef TL740_HPP
#define TL740_HPP

#include "../../common/utility-cc/tool.hpp"
#include "../uart/uart_frame_M1.h"


template<std::size_t SIZE>
static uint8_t check_sum( std::array<uint8_t, SIZE > &arr, uint start_index , uint end_index ){
    uint8_t sum = 0;
    for ( auto it = arr.begin() + start_index; it < arr.begin() + end_index + 1; it++ ) {
        sum += *it;
    }
    return sum;
}

////以下只能设置1个为1
///采用标准输出2方式输出数据
#define USE_DATA_MODE_NORMAL_2  1
///采用9轴方式输出数据
#define USE_DATA_MODE_9A  0


class TL740{

public:

    ///数据输出帧率 可选 0x01: 5HZ 0x02: 15HZ 0x03: 25 0x04: 35 0x05: 50 0x06: 100
    typedef enum TL740FrameRateSelectL{
        DATA_FR_5HZ = 0x01,
        DATA_FR_15HZ,
        DATA_FR_25HZ,
        DATA_FR_35HZ,
        DATA_FR_50HZ,
        DATA_FR_100HZ
    }TL740FrameRateSelectL;

    ///数据输出模式，可选  0x70: 9轴输出  0x71:标准输出1 0x72:标准输出2
    typedef enum TL740DataModeSelectL{
        DATA_MODE_9A  = 0x70,
        DATA_MODE_NORMAL_1  = 0x71,
        DATA_MODE_NORMAL_2  = 0x72,
    }TL740DataModeSelectL;

    //------------------------标准输出2数据格式------------------------
    typedef struct TL740Mode2DataT{
        double  acc_x;
        double  acc_y;
        double  yaw;
        uint64_t timestamp;
    }TL740Mode2DataT;
    typedef struct TL740Mode2DataBuffT{
        std::queue<TL740Mode2DataT> data;
        uint size_max=10;
        std::mutex mutex;
    }TL740Mode2DataBuffT;
    //------------------------9轴输出数据格式------------------------
    typedef struct TL740Mode9ADataT{
        double  roll;
        double  pitch;
        double  yaw;
        double  acc_x;
        double  acc_y;
        double  acc_z;
        double  groy_x;
        double  groy_y;
        double  groy_z;
        uint64_t timestamp;
    }TL740Mode9ADataT;
    typedef struct TL740Mode9ADataBuffT{
        std::queue<TL740Mode9ADataT> data;
        uint size_max=10;
        std::mutex mutex;
    }TL740Mode9ADataBuffT;

#if  USE_DATA_MODE_NORMAL_2
    TL740Mode2DataBuffT* data_buff = ( new TL740Mode2DataBuffT );
#elif USE_DATA_MODE_9A
    TL740Mode9ADataBuffT* data_buff = ( new TL740Mode9ADataBuffT );
#endif

    TL740( std::string dev_id, uint baud_rate ) : dev_id_( dev_id ), baud_rate_( baud_rate ){
      ;
    }

    void StartProcess( void ){
        pthread_create( &tid_, nullptr, _thread_t<TL740, &TL740::process_>, this );
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

        std::array<uint8_t,2> header;
        header[ 0 ] = frame_header_;
        header[ 1 ] = frame_data_length_;
        uart_fh_.SetFrameStruct( header, frame_length_ );

        std::shared_ptr<muart::UartFrameT> frame =
            std::shared_ptr<muart::UartFrameT>( new muart::UartFrameT );
        //uint count_frames = 0;
        //double last_data_stamp = 0;

#if USE_DATA_MODE_NORMAL_2
        TL740Mode2DataT  tl740_data;
#elif USE_DATA_MODE_9A
        TL740Mode9ADataT tl740_data;
#endif

        while( !stop_process_ ){
            if( 0 == uart_fh_.GetOneFrame( frame ) ){
#if USE_DATA_MODE_NORMAL_2
                //printf("get frame ok %d .\n", count_frames++ );
                if( MSUCCESS == decodeMode2DataFrame( frame, tl740_data ) ) {
                    //printf("decode tl740 data success, %d \n", count_frames );
                    pushData( tl740_data );
                }
#elif USE_DATA_MODE_9A
                if( MSUCCESS == decodeMode9ADataFrame( frame, tl740_data ) ) {
                    //printf("decode tl740 data success, %d \n", count_frames );
                    pushData( tl740_data );
                }
#endif
                //else {
                    //printf("decode distance fail!\n");
                //}
            }
            M_USLEEP( 500 );  //sleep, neccssary! (延时等待其他线程写入数据，否则可能会导致数据无法写入)
        }
        //关闭串口数据接收处理
        uart_fh_.StopProcess();
    }

    void StopProcess( void ){
        stop_process_ = true;
    }

    ///方位角清零//不能短时间内连续发,否则传感器无法正确响应
    ///另未做设置结果响应检查,需要上层判断是否设置成功
    void ResetYaw( void ){
        M_MSLEEP(10);
        std::array<uint8_t,5> frame={0x68,0x04,0x00,0x28,0x2C};
        uart_fh_.Send( frame );
        M_MSLEEP(10);
    }
    ///加速度异常处理,辣鸡产品剧烈运动可能导致加速度已知存在一个偏差
    ///上层软件检测异常，可调用该命令
    void ResetAcc( void ){
        M_MSLEEP(10);
        std::array<uint8_t,5> frame={0x68,0x04,0x00,0x27,0x2B};
        uart_fh_.Send( frame );
        M_MSLEEP(10);
    }

    ///设置数据输出帧率，重新上电后会自动保存
    void SetFrameRate( TL740FrameRateSelectL frame_rate ){
        M_MSLEEP(10);
        std::array<uint8_t,6> frame={0x68,0x05,0x00,0x0C,0x00,0x00};
        frame[4] = frame_rate;
        frame[5] = check_sum( frame, 1, 4 );
        uart_fh_.Send( frame );
        M_MSLEEP(10);
    }
    ///设置数据输出模式
    void SetDataMode( TL740DataModeSelectL data_mode ){
        M_MSLEEP(20);
        std::array<uint8_t,6> frame={0x68,0x05,0x00,0xFD,0x00,0x00};
        frame[4] = data_mode;
        frame[5] = check_sum( frame, 1, 4 );
        uart_fh_.Send( frame );
        M_MSLEEP(20);
    }

    ///关闭静止检测算法
    void SetStaticDetect( bool sw ){
        M_MSLEEP(20);
        if( sw ){
            std::array<uint8_t,6> frame={0x68,0x05,0x00,0xFE,0x01,0x04};
            uart_fh_.Send( frame );
        }
        else {
            std::array<uint8_t,6> frame={0x68,0x05,0x00,0xFE,0x00,0x03};
            uart_fh_.Send( frame );
        }
        M_MSLEEP(20);
    }

    ~TL740( void ){
        delete data_buff;
        StopProcess();
        pthread_join( tid_, nullptr );
    }

private:

    pthread_t tid_;
    std::string dev_id_;
    uint baud_rate_;
    bool stop_process_ = false;

    muart::UartFrameM1H  uart_fh_;

    uint8_t frame_header_ = 0x68;
#if  USE_DATA_MODE_NORMAL_2
    uint8_t frame_data_length_ = 0x0D;  ///标准输出二数据长度
    uint frame_length_ = 0x0D+1;
    void pushData( const TL740Mode2DataT d ){
        //加锁
        std::unique_lock<std::mutex> lock( data_buff->mutex );
        if( data_buff->data.size() >= data_buff->size_max ){
            data_buff->data.pop();
        }
        data_buff->data.push( d );
    }
#elif USE_DATA_MODE_9A
    uint8_t frame_data_length_ = 0x1F;    ///9轴输出数据长度
    uint frame_length_ = 0x1F+1;

    void pushData( const TL740Mode9ADataT d ){
        //加锁
        std::unique_lock<std::mutex> lock( data_buff->mutex );
        if( data_buff->data.size() >= data_buff->size_max ){
            data_buff->data.pop();
        }
        data_buff->data.push( d );
    }
#endif

    bool checkSumOk( const std::shared_ptr<muart::UartFrameT> frame ){
        if( !frame ){
            std::cout << "checkSumOk: nullptr" << std::endl;
            return false;
        }
        uint8_t r_checksum = (*frame)[ frame->size() - 1 ];  //倒数第1个为校验和位置
        uint8_t checksum = 0;
        //计算方法： 除去帧头1字节以及校验和本身，其余相加
        for( auto it = frame->begin() + 1; it < frame->end() - 1; it++ ){
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
    ////解码标准格式2数据
    /// 68 0D 00 84 00 00 56 10 00 08 00 93 69 FB
    /// buff[4-6] X轴加速度  buff[7-9] Y轴加速度  buff[10-12] 航向角
    /// 00 00 50: 3 个字符表示 +0.050g(右)
    /// 10 00 50: 3 个字符表示 -0.050g(左)
    /// 方位角 3 (顺时)
    ///11 60 00: 3 个字符表示 - 160.00°(顺)
    ///01 60 00: 3 个字符表示 + 160.00°(逆)
    int decodeMode2DataFrame( const std::shared_ptr<muart::UartFrameT> frame,
                             TL740Mode2DataT& d ){
        if( !frame ){
            std::cout << "decodeFrame: nullptr" << std::endl;
            return MFAIL;
        }
        //1.判断帧长度!
        if( frame_length_ != frame->size() ){
            std::cout << "frame length incorect!" << std::endl;
            //exit(1);
            return MFAIL;
        }
        //2.校验!
        if( !checkSumOk( frame ) ){
            std::cout << "checksum incorrect!" << std::endl;
            return MFAIL;
        }

        double acc_x = 0; //计算绝对值
        acc_x = frame->at(5) / 16 +  ( frame->at(5) % 16 ) * 0.1
                + ( frame->at(6) / 16 ) * 0.01 + ( frame->at(6) % 16 ) * 0.001;
        if( frame->at( 4 ) / 16 == 1 ){   //acc符号位
            d.acc_x = -1 * acc_x;
        } else {
            d.acc_x = acc_x;
        }
        ////改为向左边为正 !!!
        d.acc_x *= -1;

        double acc_y = 0; //计算绝对值
        acc_y = ( frame->at(8) / 16 ) * 1 + ( frame->at(8) % 16 ) * 0.1 +
                ( frame->at(9) / 16 ) * 0.01 +  ( frame->at(9) % 16 ) * 0.001;

        if( frame->at( 7 ) / 16 == 1 ){   //acc符号位
            d.acc_y = -1 * acc_y;
        } else {
            d.acc_y = acc_y;
        }

        double yaw = 0; //计算绝对值
        yaw = ( frame->at( 10 ) % 16 ) * 100 +
             ( frame->at(11) / 16 ) * 10 +  ( frame->at(11) % 16 ) * 1 +
             ( frame->at(12) / 16 ) * 0.1 +  ( frame->at(12) % 16 ) * 0.01;
        if( frame->at( 10 ) / 16 == 1 ){   //符号位
            d.yaw = -1 * yaw;
        } else {
            d.yaw = yaw;
        }
        return  MSUCCESS;
    }
    ////解码9轴格式输出数据  //详见文档
    /// buff[4-6]--roll buff[7-9]--pith  buff[10-12]--yaw
    /// buff[13-15]--acc_x buff[16-18]--acc_y  buff[19-21]--acc_z
    /// buff[22-24]--groy_x buff[25-27]--groy_y  buff[28-30]--groy_z
    int decodeMode9ADataFrame( const std::shared_ptr<muart::UartFrameT> frame,
                               TL740Mode9ADataT& d ){
        if( !frame ){
            std::cout << "decodeFrame: nullptr" << std::endl;
            return MFAIL;
        }
        //1.判断帧长度!
        if( frame_length_ != frame->size() ){
            std::cout << "frame length incorect!" << std::endl;
            //exit(1);
            return MFAIL;
        }
        //2.校验!
        if( !checkSumOk( frame ) ){
            std::cout << "checksum incorrect!" << std::endl;
            return MFAIL;
        }
        //------------------------姿态角------------------------------
        double roll = 0;
        roll = ( frame->at( 4 ) % 16 ) * 100 +
             ( frame->at(5) / 16 ) * 10 +  ( frame->at(5) % 16 ) * 1 +
             ( frame->at(6) / 16 ) * 0.1 +  ( frame->at(6) % 16 ) * 0.01;
        if( frame->at( 4 ) / 16 == 1 ){   //符号位
            d.roll = -1 * roll;
        } else {
            d.roll = roll;
        }

        double pitch = 0;
        pitch = ( frame->at( 7 ) % 16 ) * 100 +
             ( frame->at(8) / 16 ) * 10 +  ( frame->at(8) % 16 ) * 1 +
             ( frame->at(9) / 16 ) * 0.1 +  ( frame->at(9) % 16 ) * 0.01;
        if( frame->at( 7 ) / 16 == 1 ){   //符号位
            d.pitch = -1 * pitch;
        } else {
            d.pitch =  pitch;
        }
        //实际测试发现俯仰角符号相反!!!
        d.pitch *= -1;

        double yaw = 0;
        yaw = ( frame->at( 10 ) % 16 ) * 100 +
             ( frame->at(11) / 16 ) * 10 +  ( frame->at(11) % 16 ) * 1 +
             ( frame->at(12) / 16 ) * 0.1 +  ( frame->at(12) % 16 ) * 0.01;
        if( frame->at( 10 ) / 16 == 1 ){   //符号位
            d.yaw = -1 * yaw;
        } else {
            d.yaw = yaw;
        }

        //------------------------Acc------------------------------
        //发现Y轴再到X轴
        double acc_y = 0; //计算绝对值
        acc_y = frame->at(14) / 16 +  ( frame->at(14) % 16 ) * 0.1
                + ( frame->at(15) / 16 ) * 0.01 + ( frame->at(15) % 16 ) * 0.001;
        if( frame->at( 13 ) / 16 == 1 ){   //acc符号位
            d.acc_y = -1 * acc_y;
        } else {
            d.acc_y = acc_y;
        }

        double acc_x = 0; //计算绝对值
        acc_x = ( frame->at(17) / 16 ) * 1 + ( frame->at(17) % 16 ) * 0.1 +
                ( frame->at(18) / 16 ) * 0.01 +  ( frame->at(18) % 16 ) * 0.001;

        if( frame->at( 16 ) / 16 == 1 ){   //acc符号位
            d.acc_x = -1 * acc_x;
        } else {
            d.acc_x = acc_x;
        }
        //实际测试发现X轴加速度符号相反!!!  ///转为方向超前
        d.acc_x *= -1;

        double acc_z = 0; //计算绝对值
        acc_z = ( frame->at(20) / 16 ) * 1 + ( frame->at(20) % 16 ) * 0.1 +
                ( frame->at(21) / 16 ) * 0.01 +  ( frame->at(21) % 16 ) * 0.001;
        if( frame->at( 19 ) / 16 == 1 ){   //acc符号位
            d.acc_z = -1 * acc_z;
        } else {
            d.acc_z = acc_z;
        }
        //实际测试发现Z轴加速度符号相反!!!  ///转为方向超上
        d.acc_z *= -1;

        //------------------------Groy------------------------------
        //暂时不取

        return  MSUCCESS;
    }




};

static void tl740_test( void ){

    TL740 tl740("/dev/ttyUSB0", 115200 );
    tl740.StartProcess();

#if  USE_DATA_MODE_NORMAL_2
    tl740.SetDataMode( TL740::DATA_MODE_NORMAL_2 );
    tl740.SetFrameRate( TL740::DATA_FR_100HZ );
    TL740::TL740Mode2DataT d;
    TL740::TL740Mode2DataBuffT* ds;
#elif USE_DATA_MODE_9A
    tl740.SetDataMode( TL740::DATA_MODE_9A );
    tl740.SetFrameRate( TL740::DATA_FR_100HZ );
    TL740::TL740Mode9ADataT d;
    TL740::TL740Mode9ADataBuffT* ds;
#endif
    ds = tl740.data_buff;
    uint d_count=0;
    //stool::FPS fps_d;
    while( true ){
        if( ds ){
            std::unique_lock<std::mutex> ds_lock( ds->mutex );  //!!!
            if( !ds->data.empty() ){
                d = ds->data.front() ;
#if  USE_DATA_MODE_NORMAL_2
                printf("mode normal2 data %d : %.6f  %.6f  %.2f \n", ++d_count, d.acc_x, d.acc_y, d.yaw );
#elif USE_DATA_MODE_9A
                printf("mode 9A data rpy %d : %.6f  %.6f  %.2f \n", ++d_count, d.roll, d.pitch, d.yaw );
#endif
                ds->data.pop();
            }
        }
        if( d_count % 600 == 0  && d_count != 0 ){
            d_count++;
            //printf("reset yaw and acc\n.");
            //tl740.ResetYaw();
            //tl740.ResetAcc();
        }
        M_USLEEP(500);
    }
}


#endif //TL740_HPP
