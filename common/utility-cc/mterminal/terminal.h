/* SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause) */
/*
 * Copyright (c) 2002-2007 Volkswagen Group Electronic Research
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of Volkswagen nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * Alternatively, provided that this notice is retained in full, this
 * software may be distributed under the terms of the GNU General
 * Public License ("GPL") version 2, in which case the provisions of the
 * GPL apply INSTEAD OF those given above.
 *
 * The provided data structures and external interfaces from this code
 * are not restricted to be used by modules with a GPL compatible license.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 * Send feedback to <linux-can@vger.kernel.org>
 *
 */

#ifndef TERMINAL_H
#define TERMINAL_H


#include <stdio.h>


/* reset to default */

#define ATTRESET "\33[0m"     //关闭所有属性

/* attributes */
#define ATTBOLD      "\33[1m"       //设置高亮
#define ATTUNDERLINE "\33[4m"       //下划线
#define ATTBLINK     "\33[5m"       //闪烁
#define ATTINVERSE   "\33[7m"       //反显
#define ATTINVISIBLE "\33[8m"       //消隐

/* foreground colors */             //前景色
#define FGBLACK   "\33[30m"
#define FGRED     "\33[31m"
#define FGGREEN   "\33[32m"
#define FGYELLOW  "\33[33m"
#define FGBLUE    "\33[34m"
#define FGMAGENTA "\33[35m"
#define FGCYAN    "\33[36m"
#define FGWHITE   "\33[37m"

/* background colors */

#define BGBLACK   "\33[40m"
#define BGRED     "\33[41m"
#define BGGREEN   "\33[42m"
#define BGYELLOW  "\33[43m"
#define BGBLUE    "\33[44m"
#define BGMAGENTA "\33[45m"     //品红
#define BGCYAN    "\33[46m"     //青色
#define BGWHITE   "\33[47m"


/* cursor */
#define CSR_HOME()  printf("\33[H")  //光标置顶
#define CSR_UP()    printf("\33[A")
#define CSR_DOWN()  printf("\33[B")
#define CSR_RIGHT() printf("\33[C") //右移1
#define CSR_LEFT()  printf("\33[D") //左移1


//#define CSR_UP_N(n)         "\033["#n"A"                    //光标上移n行
//#define CSR_DOWN_N(n)       "\033["#n"B"                    //光标下移n行
//#define CSR_RIGHT_N(n)      "\033["#n"C"                    //光标右移n列
//#define CSR_LEFT_N(n)       "\033["#n"D"                    //光标左移n列
//#define CSR_SET_N(n)        "\033["#n";H"                   //光标设置到n行

#define CLR_SCREEN() printf("\033[2J")              // 清除屏幕
#define CSR_MOVEUP_N(x) printf("\033[%dA", (x))     // 上移光标
#define CSR_MOVEDOWN_N(x) printf("\033[%dB", (x))   // 下移光标
#define CSR_MOVELEFT_N(y) printf("\033[%dD", (y))   // 左移光标
#define CSR_MOVERIGHT_N(y) printf("\033[%dC",(y))   // 右移光标

#define CSR_MOVETO_R(x) printf("\033[%d;H", (x) )   //移动到x行
#define CSR_MOVETO_L(y) printf("\033[;%dH", (y) )   //移动到y列
#define CSR_MOVETO(x,y) printf("\033[%d;%dH", (x), (y)) //定位光标

#define CSR_RESET() printf("\033[H") // 光标复位
#define CSR_HIDE()  printf("\33[?25l") // 隐藏光标
#define CSR_SHOW()  printf("\33[?25h") // 显示光标

//反显
#define HIGHT_LIGHT() printf("\033[7m")
#define UN_HIGHT_LIGHT() printf("\033[27m")


#endif /* TERMINAL_H */

