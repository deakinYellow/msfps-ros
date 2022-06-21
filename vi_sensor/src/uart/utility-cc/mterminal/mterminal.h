#ifndef MTERMINAL_H
#define MTERMINAL_H

#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>

#include "terminal.h"

//注意处理打印阻塞问题
//可用fflush( stdout );/强制缓冲区输出


namespace mterm {
    int  set_backshow(int fd,int option);
    void stdin_clear( void );
}

#endif // MTERMINAL_H
