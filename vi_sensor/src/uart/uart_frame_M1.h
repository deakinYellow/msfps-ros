/************************************************************************
  > File Name: uart_frame.h
	> Author:Deakin
	> Mail: deakinhwang@163.com
  > Created Time:
 ************************************************************************/
#ifndef UART_FRAME_M1_H
#define UART_FRAME_M1_H

#include "buart.hpp"

namespace muart{


typedef struct FrameStructM1{
    std::array<uint8_t,2> header;      //帧头2字节
    uint frame_len;    ///完整帧长度
}FrameStructM1;

typedef std::vector<uint8_t> UartFrameT;

//数据帧提取
//数据帧格式：帧头只有1字节，第2字节为帧长度(不变)，没有帧尾，将帧头1字节和描述帧长的1字节组成2字节帧头
class UartFrameM1H : public Buart{

public:
    UartFrameM1H( void ){
      ;
    }
    void SetFrameStruct( const std::array<uint8_t,2> header,
                         const uint frame_len ){

        frame_struct_->header[ 0 ] = header[ 0 ];
        frame_struct_->header[ 1 ] = header[ 1 ];
        frame_struct_->frame_len  = frame_len;

    }

    int GetOneFrame( std::shared_ptr<UartFrameT> frame ){
        if( frame && !frame->empty() ){
          frame->clear();
        }
        return  this->getOneFrame( this->frame_struct_ , frame );
    }

    ~UartFrameM1H( void ){
        delete frame_struct_;
        ;
    }
private:

    FrameStructM1* frame_struct_ = ( new FrameStructM1 );

    //按照帧头以及已知帧长提取一帧数据,只支持帧头两字节的情形
    //1.寻找帧头
    //2.找帧头成功，根据已知长度截取该帧返回，并擦除原始缓存中的该段数据,
    //  找帧头失败,清空原始数据缓存
    int getOneFrame( const FrameStructM1* s , std::shared_ptr<UartFrameT> frame ){

        //check input
        if( !s || !frame ){
            std::cout << "getOneframe: nullptr!" << std::endl;
            return 1;
        }
        //rbuff加锁,线程同步，必须.
        std::unique_lock<std::mutex> rbuff_dlock( this->rbuff->mutex );
        ReceiveDataT* rdata =  &( this->rbuff->data );

        if( rdata->size() < s->frame_len ){
            return 1;
        }
        //开始找帧头
        for( uint i = 0; i < rdata->size() - 1; i++ ) {
            if( rdata->at( i ) == s->header[ 0 ] && rdata->at( i+1 ) == s->header[ 1 ] ) {
                //printf("get header %.2X  at %d.\n", rdata->at( i ), i );
                //根据存在的帧长寻找帧尾
                //先判断数据是否足够,防止越界
                if( rdata->size() - i < s->frame_len ){
                        return  1;
                }
                //数据帧输出
                for( uint j = 0 ; j < s->frame_len ; j++ ) {
                    frame->push_back( rdata->at( j + i ) );
                }
                ///清除开始到该帧尾
                rdata->erase( rdata->begin(), rdata->begin() + i + s->frame_len );
                return 0;  //帧提取成功返回０
            }
        }
        printf("can not find header, clear\n");
        ///找不到帧头，清空所有缓存
        rdata->clear();
        return 1;
    }

};

void uart_frameM1H_test( void );

}

#endif

