#ifndef MTHREAD_H
#define MTHREAD_H

#include <iostream>
#include <mutex>
#include <pthread.h>

//函数模版不单可以替换类型本身，还能替换类型的成员函数。
/*
 * @brief 使用非静态成员函数,做为线程函数
 * @note：1.名称只能是 process_，不能在指定模版参数的时候修改；
 *        2. process_,不能在指定模版参数的时候修改； 只能是public的，除非把_thread_t定义到MyClass的内部。
 */
template <typename TYPE, void (TYPE::*process_)() >
void* _thread_t(void* param )
{
    TYPE* This = (TYPE*)param ;
    This->process_();
    return nullptr;
}


//可作为模板使用,替换类名即可
#define TEST_PTHREAD  0

#if TEST_PTHREAD

#include "utility/tool.h"

class MThread
{
public:
    std::string text;
    //线程函数
    void _runThread(){
        //this->DoSomeThing();
        while( true ){
            std::cout << this->text << std::endl;
            TPMSLEEP(10);
        }
    }

    MThread() {
        pthread_create(&tid, NULL, _thread_t<MThread, &MThread::_runThread>, this);
    }

    ~MThread(){
        pthread_join( tid, NULL );  //显示回收线程
    }

private:
    pthread_t tid;

};

//===================测试函数=====================================
void mthreadTest( void ) {
    MThread mthread1;
    mthread1.text = "hello, I am mthread 1.";
    MThread mthread2;
    mthread2.text = "hello, I am mthread 2.";
    MThread mthread3;
    mthread3.text = "hello, I am mthread 3.";
    while( true ){
        TPMSLEEP(10);
        std::cout << "main thread." << std::endl;
    }
}

void mthreadTest( void );

#endif

#endif // MTHREAD_H
