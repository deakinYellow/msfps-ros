#include <iostream>
#include <vector>
#include <string.h>


//c++格式化字符串
template <typename ...Args>
inline static std::string format_string(const char* format, Args... args) {
    constexpr size_t oldlen = BUFSIZ;
    char buffer[oldlen];  // 默认栈上的缓冲区

    size_t newlen = snprintf(&buffer[0], oldlen, format, args...);
    newlen++;  // 算上终止符'\0'

    if (newlen > oldlen) {  // 默认缓冲区不够大，从堆上分配
        std::vector<char> newbuffer(newlen);
        snprintf(newbuffer.data(), newlen, format, args...);
        return std::string(newbuffer.data());
    }
    return buffer;
}


template <typename ...Args>
void static mprintf( const char* format, Args... args) {
    auto msg = format_string(format, std::forward<Args>(args)...);
    fputs( msg.c_str(), stderr);
}


/**
 * @brief   格式化字符串写入(在原有字符串后面追加)
 * @param  [in,out]  dst 目标字符串
 * @retval [out]
 * @note     返回追加后的格式化字串
 */
template <typename ...Args>
void static format_string_append( std::string& dst ,const char* format, Args... args) {
    std::string str = format_string( format, std::forward<Args>(args)...);
    dst = dst + str;
}

#ifdef TEST
template <typename ...Args>
void errExit(const char* format, Args... args) {
    auto errmsg = format_string(format, std::forward<Args>(args)...);
    errmsg = errmsg + ": " + strerror(errno) + "\n";
    fputs(errmsg.c_str(), stderr);
    exit(1);
}
int main() {
    const char* s = "hello world!";
    int fd = -1;
    mprintf("hello,this num %d \n", 1 );
    if (write(fd, s, strlen(s)) == -1)
        errExit("write \"%s\" to file descriptor(%d) failed", s, fd);
    return 0;
}
#endif


