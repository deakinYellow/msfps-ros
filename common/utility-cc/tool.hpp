/*
 *  @brief:  C/C++编程常用小工具: LOG,延时,系统时间,键盘输入等
 *  @author: Deakin
 *  @email:  deakinhwang@163.com
 *  @last modify:  2019/12/25
 */
#ifndef TOOL_H
#define TOOL_H

#include <string.h>
#include <iostream>
#include <assert.h>
#include <time.h>
#include <unistd.h>
#include <sys/time.h>

#include "mterminal/terminal.h"
//#define __need_timeval

#define MS_TIMESTAMP_STR_LENGTH 24
static char g_msts_str[MS_TIMESTAMP_STR_LENGTH]={'\0'};

namespace  tool {

//获取系统毫秒级时间戳
inline long sys_ms_timestamp( void ) {
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    return tv.tv_sec*1000 + tv.tv_usec/1000;  //获取毫秒
}

///毫秒部分字符串获取
inline void get_msstr( int ms, char* msstr ){
    int a;
    a = ms / 100;
    msstr[0] = char( a + 0x30 );
    a = ( ms % 100 ) / 10;
    msstr[1] = char( a + 0x30 );
    a =  ms % 10 ;
    msstr[2] = char( a + 0x30 );
}

///毫秒级时间戳转字符串
//采用固定长度字符串存储
//格式如下:2019/07/14 00:48:13.298长度为 MS_TIMESTAMP_STR_LENGTH
inline void  ms_ts2str_c( long stamp, char* timestamp_str ){
    time_t tt = time_t( stamp / 1000 );
    struct tm *ttime;
    ttime = localtime(&tt);
    char date[20]={'\0'};
    strftime( date,20,"%Y-%m-%d %H:%M:%S",ttime);//该格式返回的date实际有效长度为19
    ///先清零
    memset( timestamp_str,'\0', MS_TIMESTAMP_STR_LENGTH );
    memcpy( timestamp_str, date,19);
    memcpy( timestamp_str + 19, ".", 1 );
    int ms = int( stamp % 1000);
    char ms_str[4]={'\0'};
    get_msstr( ms, ms_str );
    memcpy( timestamp_str + 20, ms_str, 3 );
}

//获取系统毫秒级时间字符串,char*格式
inline void sys_ms_ts_str_c( char* timestamp_str ){
    long ms_timestamp = sys_ms_timestamp();
    ms_ts2str_c( ms_timestamp, timestamp_str ) ;
}

//获取系统毫秒级时间字符串,string格式
inline std::string sys_ms_ts_str( void ){
    long ms_timestamp = sys_ms_timestamp();
    char timestamp_str[MS_TIMESTAMP_STR_LENGTH]={'\0'};
    ms_ts2str_c( ms_timestamp, timestamp_str );
    std::string out_str( timestamp_str );
    return out_str;
}

//获取系统秒级时间字符串, string格式
inline std::string sys_s_ts_str( void ){
    std::string s = sys_ms_ts_str();
    return s.substr(0, ( s.size() - 4 )) ;
}

}

//--------------------------------LOG ABOUT---------------------------------------
//在需要开启debuglog的目标源文件开头包含tool.h前引入,如果全局开启,则在这里引入
//#define ENABLE_MLOGD
#ifdef ENABLE_MLOGD
    #define MLOGD(format, ...) \
      do { \
          tool::sys_ms_ts_str_c( g_msts_str );\
          fprintf( stdout, "[%s] [%-22s] [D] \033[0;33;32m " format "\033[0m\n",\
           g_msts_str,__func__,##__VA_ARGS__ ); \
      } while (0)
    #define FLOGD(format, ...) \
      do { \
        tool::sys_ms_ts_str_c( g_msts_str );\
        fprintf( stdout, "[%s] [%s@%s,%d] [D] \033[0;33;32m " format "\033[0m\n",\
        g_msts_str,__func__,__BASE_FILE__, __LINE__, ##__VA_ARGS__ ); \
      } while (0)
#else
    #define MLOGD(fmt,...)
    #define FLOGD(fmt,...)
#endif

//#define MLOGI(fmt,...)  fprintf(stderr, "%s [I] %22s | \033[0;0;37m " fmt "\033[0m\n",\
//                                    tool::sys_ms_ts_str().c_str(), __FUNCTION__, ##__VA_ARGS__ )
//#define MLOGW(fmt,...)  fprintf(stderr, "%s [W] %22s | \033[0;0;33m " fmt "\033[0m\n",\
//                                    tool::sys_ms_ts_str().c_str(), __FUNCTION__, ##__VA_ARGS__ )
//#define MLOGE(fmt,...)  fprintf(stderr, "%s [E] %22s | \033[0;33;31m " fmt "\033[0m\n",\
//                                    tool::sys_ms_ts_str().c_str(), __FUNCTION__, ##__VA_ARGS__ ) ; assert( false )

//---------------------simple-------------------------------------

#define MLOGE( format, ...) \
    do { \
        tool::sys_ms_ts_str_c( g_msts_str );\
        fprintf( stdout, "[%s] [%-22s] [E] \033[0;33;31m " format "\033[0m\n",\
           g_msts_str,__func__, ##__VA_ARGS__ ); \
  } while (0)

#define MLOGW( format, ...) \
    do { \
        tool::sys_ms_ts_str_c( g_msts_str );\
        fprintf( stdout, "[%s] [%-22s] [W] \033[0;0;33m " format "\033[0m\n",\
           g_msts_str,__func__, ##__VA_ARGS__ ); \
    } while (0)

#define MLOGI( format, ...) \
    do { \
        tool::sys_ms_ts_str_c( g_msts_str );\
        fprintf( stdout, "[%s] [%-22s] [I] \033[0;0;37m " format "\033[0m\n",\
           g_msts_str,__func__,##__VA_ARGS__ ); \
    } while (0)

//-----------------------full-----------------------------------
#define FLOGE( format, ...) \
    do { \
        tool::sys_ms_ts_str_c( g_msts_str );\
        fprintf( stdout, "[%s] [%s@%s,%d] [E] \033[0;33;31m " format "\033[0m\n",\
           g_msts_str,__func__, __BASE_FILE__, __LINE__, ##__VA_ARGS__ ); \
  } while (0)

#define FLOGW( format, ...) \
    do { \
        tool::sys_ms_ts_str_c( g_msts_str );\
        fprintf( stdout, "[%s] [%s@%s,%d] [W] \033[0;0;33m " format "\033[0m\n",\
           g_msts_str,__func__, __BASE_FILE__, __LINE__, ##__VA_ARGS__ ); \
    } while (0)

#define FLOGI( format, ...) \
    do { \
        tool::sys_ms_ts_str_c( g_msts_str );\
        fprintf( stdout, "[%s] [%s@%s,%d] [I] \033[0;0;37m " format "\033[0m\n",\
           g_msts_str,__func__, __BASE_FILE__, __LINE__, ##__VA_ARGS__ ); \
    } while (0)

#define MLOGE_EXIT( format, ...) \
    do { \
        tool::sys_ms_ts_str_c( g_msts_str );\
        fprintf( stdout, "[%s] [%-22s] [E] \033[0;33;31m " format "\033[0m\n",\
           g_msts_str,__func__, ##__VA_ARGS__ ); \
        exit(1);\
  } while (0)

//--------------------------------LOG ABOUT---------------------------------------



//--------------------------------TIME ABOUT-------------------------------------
///特指CPU消耗的实际时间，用来判断某段功能的CPU消耗(例如延时函数就不会消耗太多CPU实际使用时间)
#define  TIME_START clock_t startTimeT = clock()
#define  TIME_END   clock_t endTimeT = clock()
#define  TIME_COST  ( endTimeT - startTimeT ) / (CLOCKS_PER_SEC / 1000.0 )
#define  TIME_COSTLOGD MLOGD("cost time: %.3f ms.",double(TIME_COST) )

//#include <unistd.h>
#include <time.h>

////使用select封装微秒级延时替代usleep(已废弃)
inline void select_usleep( long us ) {
    struct timeval timeout;
    timeout.tv_sec = us / 1000000;
    timeout.tv_usec = us % 1000000;
    while (true) {
         select( 0, nullptr, nullptr, nullptr, &timeout );
         if ( timeout.tv_sec<=0 && timeout.tv_usec<=0) {
            break;
         }
    }
}
////使用nanosleep封装微秒级延时替代usleep(已废弃)
inline void nano_usleep( uint32_t useconds ) {
    struct timespec ts = {
        useconds / 1000000,
        (useconds % 1000000) * 1000
    };
    while( true ){
        int ret = nanosleep( &ts, &ts );
        if( ret < 0 ){
            if( EINTR == errno ){   ///收到中断信号
                printf("nano_usleep receives the interrupt signal.\n");
                continue;
            }
        }
        else if( ret == 0 ){
            break;
        }
    }
}

#define USE_SELECT_SELLP 0
#if USE_SELECT_SELLP
#define M_USLEEP(x)     select_usleep( x )
#define M_MSLEEP(x)     select_usleep( x * 1000 )
#define M_SLEEP(x)      select_usleep( x * 1000 * 1000 )
#else
#define M_USLEEP(x)     nano_usleep( x )
#define M_MSLEEP(x)     nano_usleep( x * 1000 )
#define M_SLEEP(x)      nano_usleep( x * 1000 * 1000 )
#endif


//---------------------------------TIME ABOUT-------------------------------------

//---------------------------------OPT RET------------------------------------------
#define MSUCCESS   0
#define MFAIL   1
#define MSTATUS int
//---------------------------------OPT RET END------------------------------------------

#endif



