/************************************************************************
  > File Name: uart_frame_M0.h
	> Author:Deakin
	> Mail: deakinhwang@163.com
  > Created Time:
 ************************************************************************/
#ifndef UART_FRAME_M0_H
#define UART_FRAME_M0_H

#include "buart.hpp"

namespace muart{


typedef struct FrameStruct{
    std::array<uint8_t,2> header;
    std::array<uint8_t,2> end;
    std::vector<uint> frame_len_list;    ///可能存在的帧长度
    uint frame_len_min;
    uint frame_len_max;
}FrameStruct;

typedef std::vector<uint8_t> UartFrameT;

//数据帧提取
//目前只适应帧头帧尾均是2字节的情况,建议最大帧长不要超过128
class UartFrameH : public Buart{

public:
    UartFrameH( void ){
      ;
    }
    void SetFrameStruct( const std::array<uint8_t,2> header,
                         const std::array<uint8_t,2> end,
                         const std::vector<uint> &frame_len_list ){

        frame_struct_->header[ 0 ] = header[ 0 ];
        frame_struct_->header[ 1 ] = header[ 1 ];
        frame_struct_->end[ 0 ] = end[ 0 ];
        frame_struct_->end[ 1 ] = end[ 1 ];

        for( auto it = frame_len_list.begin(); it < frame_len_list.end(); it++ ) {
            frame_struct_->frame_len_list.push_back( *it );
        }
        frame_struct_->frame_len_min = *std::min_element( frame_len_list.begin(), frame_len_list.end() );
        frame_struct_->frame_len_max = *std::max_element( frame_len_list.begin(), frame_len_list.end() );

    }

    int GetOneFrame( std::shared_ptr<UartFrameT> frame ){
        if( frame && !frame->empty() ){
          frame->clear();
        }
        return this->getOneFrame( this->frame_struct_ , frame );
    }

    ~UartFrameH( void ){
        delete frame_struct_;
        ;
    }
private:
    FrameStruct* frame_struct_ = ( new FrameStruct );

    //按照帧头帧尾提取一帧数据,只支持帧头帧尾各两字节的情形
    //1.寻找帧头
    //2.根据可选长度，寻找帧尾
    //3.找帧尾成功，截取该帧返回，并擦除原始缓存到帧尾,找帧尾失败，并且缓存区长度大于最大帧长，说明数据丢失或者不是帧头，擦除开始到该帧头数据段
    int getOneFrame( const FrameStruct* s , std::shared_ptr<UartFrameT> frame ){
        //check input
        if( !s || !frame ){
            std::cout << "getOneframe: nullptr!" << std::endl;
            return 1;
        }
        //rbuff加锁,线程同步，必须.
        std::unique_lock<std::mutex> rbuff_dlock( this->rbuff->mutex );
        ReceiveDataT* rdata =  &( this->rbuff->data );

        if( rdata->size() < s->frame_len_min ){
            return 1;
        }
        //开始找帧头
        for( uint i = 0; i < rdata->size() - 1; i++ ) {
            if( rdata->at( i ) == s->header[ 0 ] && rdata->at( i+1 ) == s->header[ 1 ] ) {
                //printf("get header at %d .\n", i );
                //根据存在的帧长寻找帧尾
                for( auto len = s->frame_len_list.begin(); len < s->frame_len_list.end(); len++ ) {
                    //先判断数据是否足够,防止越界
                    if( rdata->size() - i < *len ){
                        return  1;
                    }
                    if( rdata->at( i + *len - 2 ) == s->end[ 0 ] && rdata->at( i + *len - 1 ) == s->end[ 1 ] ){
                        //printf("get end at %d .\n", i + *len - 2 );
                        //数据帧输出
                        for( uint j = 0 ; j < *len ; j++ ) {
                            frame->push_back( rdata->at( j + i ) );
                        }
                        ///清除开始到该帧尾
                        rdata->erase( rdata->begin(), rdata->begin() + i + *len );
                        return 0;  //帧提取成功返回０
                    }
                }
                ///找不到帧尾,并且缓存区长度大于最大长,清除从开始到该帧头数据段
                if( rdata->size() > s->frame_len_max ){
                    rdata->erase( rdata->begin(), rdata->begin() + i + 1 );
                }
            }
            //printf("rdata: %ld \n", rdata->size() );
        }
        return 1;
    }

};

void uart_frameH_test( void );


}

#endif





