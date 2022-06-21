/*************************************************************************
  > File Name: uart_frame.cc
	> Author:Deakin
	> Mail: deakinhwang@163.com
  > Created Time:
 ************************************************************************/
#include <iostream>
#include <cstring>

#include "../../common/utility-cc/tool.hpp"
#include "uart_frame_M1.h"

void muart::uart_frameM1H_test( void ){

    std::shared_ptr<muart::UartFrameM1H>  uart_fh = std::shared_ptr<muart::UartFrameM1H>( new muart::UartFrameM1H );

    uart_fh->SetUartParams("/dev/ttyUSB0", 115200 );
    uart_fh->OpenDevice();
    uart_fh->StartProcess();

    static const uint frame_data_length = 0x1F;   ///9轴输出
    std::array<uint8_t,2> header{ 0x68 , frame_data_length };
    uint  frame_len = frame_data_length + 0x01;
    uart_fh->SetFrameStruct( header, frame_len );

    std::shared_ptr<muart::UartFrameT> frame =
        std::shared_ptr<muart::UartFrameT>( new muart::UartFrameT );

    uint count_frames = 0;
    double last_data_stamp = 0;

    while( true ){
        if( 0 == uart_fh->GetOneFrame( frame ) ){
            for( ulong i = 0; i < frame->size(); i++ ) {
                printf("%.2X ", frame->at( i ) );
             }
            printf("\n----------------------------------------------------\n");
          //cout << "rbuff: " << uart_fh->rbuff->data.size() << endl;
          //uint8_t B = uart_fh->rbuff->data.front();
          //printf("%.2X ",B);
        }

        //std::array<uint8_t,5> frame={0x68,0x04,0x00,0x28,0x2C};
        //printf( "-----------%2X \n", frame[ 4 ] );
        //uart_fh->Send( frame );
        //M_USLEEP(1000);
        M_USLEEP(500);
    }
}


