#ifndef _DS_H_
#define _DS_H_

#include <map>
#include <cstring>
#include <string.h> //strerror
#include <deque>
#include <mutex>
#include <condition_variable>
#include <sys/epoll.h>
#include "json.hpp"

namespace LikeVcom{

    struct stTtyData{
        std::string strTty1;
        int nArrLen;
        uint8_t u8Arr[0]; 
    };

    struct stTcpData{
        std::string strTty1;
        int nArrLen;
        uint8_t u8Arr[0]; 
    };

    extern std::deque<stTtyData*> gdeTtyData;
    extern std::deque<stTcpData*> gdeTcpData;

    //defines in file [serialPort.cpp] or [protocol.cpp]
    extern std::mutex gmtTty;
    extern std::mutex gmtTcp;
    extern std::condition_variable gcvTty;
    extern std::condition_variable gcvTcp;

    extern int nMaxPacketNum;
    const std::string gstrSerialPath{"/dev/pts/"};

    extern std::map<std::string, nlohmann::json> gmpConfig;

}   // namespace LikeVcom

#endif