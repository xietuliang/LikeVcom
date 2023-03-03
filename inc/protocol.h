#ifndef _PROTOCOL_H_
#define _PROTOCOL_H_

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <stdio.h>
#include <cstring>
#include <stdlib.h>
#include <sys/epoll.h>
#include <sys/socket.h>
#include <sys/sendfile.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cassert>
#include <iostream>
// #include <vector>

#include <thread>
#include <chrono>

#include "ds.h"

namespace LikeVcom{

class Protocol{
public:
    Protocol();
    ~Protocol();

    void init();
    void produceTcp();
    void customTty();

private:
    void addFdToEpoll(const std::string& strSerIp, const unsigned short& u16SerPort, const std::string& strTty);
    void start();
    void onSocketRead(int nFd);
    // void onSocketWrite(int nFd);
    
    //search ip and port from the gmpConfig's value
    std::map<std::string, nlohmann::json>::iterator searchIpPort(const std::string& strIp, const uint16_t& u16Port) const;

private:
    std::string strSerIp_;
    unsigned short u16SerPort_;
    int nEpollFd_;
    bool bFlag_;
    std::map<std::string, int> mpFd_;
};

    //defines in file[protocol.cpp]
    extern std::map<std::string, nlohmann::json> gmpConfig;

}   // namespace LikeVcom

#endif