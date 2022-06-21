#ifndef MTHREAD_HPP
#define MTHREAD_HPP

#include <iostream>
#include <pthread.h>

/**
 * @brief 创建新线程,自动回收资源
 * @note
 */
class MThread {

public:

    MThread(){
      ;
    }

    typedef void* (*callback_t)(void*);

    int Create( callback_t cb, void* arg ){
        return pthread_create( &thread_, nullptr, cb, arg );
    }

    int Destroy(){
        return pthread_join( thread_, nullptr );
    }

    ~MThread(){
        Destroy();
    }

private:
    pthread_t thread_;

};


#endif // MTHREAD_HPP


#define TEST 0
#if  TEST
static int num_g = 0;
///=====================TPThread TEST==============================
///只要不含全局变量或局部静态变量，两线程可以共用函数,线程会独立开辟堆栈存储局部变量
void* testFun1( void* param ){
    int i = *(int *)param;   //引用参数
    //num_g = *(int *)param;   //引用参数
    while (true) {
        std::cout << "testThread: " << i << std::endl;
        //std::cout << "testThread: " << num_g << std::endl;
        long t = 1e10; while ( t-- != 0);
        //i++;
        //if( i > 1e6 ){
            //return NULL;
        //}
    }
}

void* testFun2( void* param ){
    int i = *(int *)param;   //引用参数
    while (true) {
        std::cout << "testThread: " << i << std::endl;
    }
}

int MThreadTest( void ){

    std::cout << "MThread test1." << std::endl;

    MThread  testThread;
    int a = 1;
    if( 0 != testThread.Create( testFun1, &a ) ){
        std::cout << "creat thread fial." << std::endl;
        return -1;
    }

    int b = 2;
    if( 0 != testThread.Create( testFun1, &b ) ){
        std::cout << "creat thread fial." << std::endl;
        return -1;
    }

    while( true ){
        long t = 1e10; while ( t-- != 0);
        //printf("testThread runing\n");
        a++;
    }
    return 0;
}
#endif
