#ifndef _SERIAL_PORT_H_
#define _SERIAL_PORT_H_

#include <stdio.h>      /*标准输入输出定义*/
#include <stdlib.h>     /*标准函数库定义*/
#include <unistd.h>     /*Unix 标准函数定义*/
#include <sys/types.h> 
#include <sys/stat.h>
#include <fcntl.h>      /*文件控制定义*/
#include <termios.h>    /*PPSIX 终端控制定义*/
#include <errno.h>      /*错误号定义*/
#include <iostream>
#include <string>
#include <map>
#include <vector>
#include <string.h>
#include <thread>
#include <algorithm>
#include "ds.h"

namespace LikeVcom{

class SerialPort{
public:
    SerialPort();
    ~SerialPort();

    void init();

    void produceTty();
    void customTcp();
        void start();
private:
    int openSerialPort(const std::string& strPortName);
    void setBaudrate(int nFd, const int& nBaudrate);
    int setParity(int nFd, int nDatabits, int nStopbits, char cParity);
    // int readBuffer(uint8_t* pBuffer, int nSize);
    // void start();
    // int writeBuffer(uint8_t* pBuffer, int nSize);

    void onSerialRead(int nFd);
    void closeSerialPort();
private:
    int nEpollFd_;
    // std::vector<std::pair<int, std::string>> vcFd_;
    std::map<std::string, int> mpFd_;
    //控制波特率的map
    std::map<int, int> mpBaudrate_;
    std::string strPortName_;
    bool bFlag_;
};

extern std::map<std::string, SerialPort> gmpSerialPort;

}   //namespace LikeVcom

#endif