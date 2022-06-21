#ifndef BUART_HPP
#define BUART_HPP

#include <iostream>
#include <mutex>
#include <memory>

#include <queue>

#include <exception>
#include <boost/asio.hpp>
//#include <boost/bind.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/exception/all.hpp>

#include "../../common/utility-cc/tool.hpp"
#include "../../common/utility-cc/process_thread.hpp"


/* 异常信息的类型 */
//typedef boost::error_info<struct tag_err_no, int> err_no;
//typedef boost::error_info<struct tag_err_str,std::string> err_str;


//muart
namespace muart{

    using namespace boost::asio;
    using std::cout;
    using std::endl;

typedef struct UartParamT{
    std::string dev_id = "/dev/ttyUSB0";          //设备名 默认ttyUSB0
    uint baud_rate = 115200;          //波特率 默认115200
    uint8_t  dlen = 8;         //数据长 默认8
}UartParamT;

typedef std::vector<uint8_t> ReceiveDataT;   //为方便操作不用queue用vector
typedef struct UartReceiveBuffT{
    ReceiveDataT data;
    //数据接收缓存区域//缓冲区最大1K，超过会自动弹出
    uint max = 1024;
    //数据读写互斥锁
    std::mutex mutex;
}UartReceiveBuffT;


class Buart{

public:

    Buart( void ){
      ;
    }
    //确定跨类使用，使用智能指针
    std::shared_ptr<UartReceiveBuffT> rbuff = std::shared_ptr<UartReceiveBuffT>( new UartReceiveBuffT );

    void SetUartParams( std::string dev_id, uint baud_rate ){
        uart_param_->dev_id = dev_id;
        uart_param_->baud_rate = baud_rate;
    }

    int OpenDevice( void ) {
        if( 0 == this->createSerialPort( uart_param_, sp_ ) ){
            cout << "Open Serial Port successed." << endl;
            return  0;
        }else {
            cout << "open serial_port fail!" << endl;
            return 1;
        }
    }

    //启动串口接收线程
    void StartProcess( void ){
        pthread_create( &tid_, nullptr, _thread_t<Buart, &Buart::process_>, this );
    }

    //串口数据接收
    void process_( void ){
        //串口数据接收  每次读入1个字节
        cout << "buart receiving start." << endl;
        std::array<uint8_t,1> buf;

        while( !stop_process_ ){
            //cout << "2buart receiving!" << endl;
            //可靠性未知
            try{
                //cout << "start read." << endl;
                //double st = stool::sys_ms_ts();

                boost::asio::read( *sp_, boost::asio::buffer( buf , 1 ) );   //阻塞

                //double et = stool::sys_ms_ts();
                //if( et - st > 21 )
                    //cout << "read byte dt. cost: " << et - st <<  "ms" << endl;

                //加数据同步锁
                std::unique_lock<std::mutex> lock( rbuff->mutex );
                this->rbuff->data.push_back( buf[ 0 ] );
                if( this->rbuff->data.size() > this->rbuff->max ){
                    //this->rbuff->data.pop();
                    this->rbuff->data.erase( this->rbuff->data.begin() );
                }
                //解锁
                lock.unlock();

                //printf("rbuff: %ld \n", this->rbuff->data.size() );
            } catch( boost::exception &e ) {
                cout << "cating boost error." << endl;
                cout << boost::diagnostic_information( e ) << endl;
            }
        }
    }

    //串口数据发送
    template<std::size_t SIZE>
    void Send( std::array<uint8_t,SIZE>  &arr ){
        if( !stop_process_ ){
            //cout << "buart writing!===========================" << endl;
            try{
                boost::asio::write( *sp_, boost::asio::buffer( arr, SIZE ) );
            } catch( boost::exception &e ) {
                cout << "cating boost error." << endl;
                cout << boost::diagnostic_information( e ) << endl;
            }
        }
    }

    void StopProcess( void ){
        stop_process_ = true;
        iosev_.stop();  //退出io_service
        sp_->close();   //关闭串口
        iosev_.run();
    }

    ~Buart( void ){
        StopProcess();
        pthread_join( tid_, nullptr );
        delete uart_param_;
    }

protected:
    pthread_t tid_;
    bool stop_process_ = false;

    io_service iosev_;

    std::shared_ptr<serial_port> sp_;
    UartParamT* uart_param_ = ( new UartParamT );

    int createSerialPort( const UartParamT* param, std::shared_ptr<serial_port>& out_sp ){
        if( param == nullptr ){
            return 1;
        }
        try{
            std::shared_ptr<serial_port> sp =
                std::shared_ptr<serial_port>( new serial_port( iosev_, param->dev_id ) );
            if( sp == nullptr ){
                return 1;
            }
            sp->set_option( serial_port::baud_rate( param->baud_rate ) );
            sp->set_option( serial_port::flow_control( serial_port::flow_control::none ) );
            sp->set_option( serial_port::parity( serial_port::parity::none ) );
            sp->set_option( serial_port::stop_bits( serial_port::stop_bits::one ) );
            sp->set_option( serial_port::character_size( param->dlen ) );
            out_sp = sp;
            return 0;
        }
        catch( boost::exception &e ) {
            cout << "cating boost error." << endl;
            cout << boost::diagnostic_information( e ) << endl;
            //重新打开
            return 1;
        }
    }

};  //end Buart

}

#endif // BUART_HPP


