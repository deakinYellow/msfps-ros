#include "mterminal.h"

#define ECHOFLAGS (ECHO | ECHOE | ECHOK | ECHONL)

namespace mterm{
//用于控制终端是否开启输入回显功能
//如果option为0，则关闭回显，为1则打开回显
int set_backshow( int fd,int option) {
    int err;
    struct termios term;
    if(tcgetattr(fd,&term)==-1){
        perror("Cannot get the attribution of the terminal");
        return 1;
    }
    if(option)
        term.c_lflag|=ECHOFLAGS;
    else
        term.c_lflag &=~ECHOFLAGS;
    err=tcsetattr(fd,TCSAFLUSH,&term);
    if(err==-1 && err==EINTR){
        perror("Cannot set the attribution of the terminal");
        return 1;
    }
   return 0;
}

//清空输入缓冲区
void stdin_clear( void ){
    int c;
    while((c = getchar()) != '\n' && c != EOF);
}

}



static int test_disable_back() {
	int passwd;
	puts("enter your passwd");
    mterm::set_backshow( STDIN_FILENO,0 ); //echo off
	scanf("%d",&passwd);
    mterm::set_backshow( STDIN_FILENO, 1 ); //echo on
	printf("your enter%d\n",passwd);
	return 0;
}
