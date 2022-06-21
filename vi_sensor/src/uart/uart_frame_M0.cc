/*************************************************************************
  > File Name: uart_frame.cc
	> Author:Deakin
	> Mail: deakinhwang@163.com
  > Created Time:
 ************************************************************************/
#include <iostream>
#include <cstring>

#include "../../common/utility-cc/tool.hpp"
#include "uart_frame_M0.h"


void muart::uart_frameH_test( void ){

    std::shared_ptr<muart::UartFrameH>  uart_fh = std::shared_ptr<muart::UartFrameH>( new muart::UartFrameH );
    uart_fh->SetUartParams("/dev/ttyUSB0", 115200 );
    uart_fh->OpenDevice();

    uart_fh->StartProcess();
    std::array<uint8_t,2> header{ 0xA5 , 0x5A };
    std::array<uint8_t,2> end{ 0x55 , 0xAA };
    std::vector<uint>  frame_len_list;

    frame_len_list.push_back( 27 );
    frame_len_list.push_back( 33 );

    uart_fh->SetFrameStruct( header, end, frame_len_list );

    std::shared_ptr<muart::UartFrameT> frame =
        std::shared_ptr<muart::UartFrameT>( new muart::UartFrameT );
    uint count_frames = 0;

    double last_distance_stamp = 0;
    //double last_position_stamp = 0;
    while( true ){
        if( 0 == uart_fh->GetOneFrame( frame ) && (*frame)[ 2 ] == 0x2D  ){
            //if( 0 == uart_fh->GetOneFrame( frame ) ){
            //printf("lenth: %ld \n", frame.size() );
            count_frames++;
            //cout << "frame get success count: " << count_frames << endl;
            UartFrameT::iterator i;
            for( i = frame->begin(); i != frame->end(); i++ ){
                //printf("%.2X ", *i );
            }
            //printf("\n");
            ////test time deviation-------------------------
            double stamp = tool::sys_ms_timestamp();
            double dt = stamp - last_distance_stamp;
            //printf("uwb distance data dt: %f \n", dt );
            if( dt > 200 ){
                cout << "uwb distance data dt too large: " << dt << endl;
            }
            last_distance_stamp = stamp;
        }

        if( !uart_fh->rbuff->data.empty() ){
          //cout << "rbuff: " << uart_fh->rbuff->size() << endl;
          //uint8_t B = uart_fh->rbuff->front();
          //printf("%.2X ",B);
          //uart_fh->rbuff->pop();
        }
        /*
        */
        //std::array<uint8_t,5> frame={0x68,0x04,0x00,0x28,0x2C};
        //frame[ 4 ]  = check_sum( frame, 1, 3 );  //frame[ 1 2 3 ]
        //printf( "-----------%2X \n", frame[ 4 ] );
        //uart_fh->Send( frame );

        M_USLEEP(1000);
    }
}

