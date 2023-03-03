#include "protocol.h"

namespace LikeVcom{

Protocol::Protocol()
    : strSerIp_("")
    , u16SerPort_(0)
    , nEpollFd_(-1){
        std::cout << "Protocol()" << std::endl;
}

Protocol::~Protocol(){
    std::cout << "~Protocol()" << std::endl;
    int nRet{0};
    for(auto& elem : mpFd_){
        nRet = epoll_ctl(nEpollFd_, EPOLL_CTL_DEL, elem.second, nullptr);
        if(-1 == nRet){
            std::cout << "tcp nEpollFd_:[" << nEpollFd_ << "] delete fd:[" << elem.second << "] failed:" << strerror(errno) << std::endl;
        }
        close(elem.second);
    }
    close(nEpollFd_);

    std::lock_guard<std::mutex> lk(gmtTcp);
    while(!gdeTcpData.empty()){
        auto p = gdeTcpData.front();
        gdeTcpData.pop_front();
        free(p);
        p = nullptr;
    }
}

//this function must be called before function addFdToEpoll()
void Protocol::init(){
    std::string strIp{};
    uint16_t u16Port{};
    nEpollFd_ = epoll_create1(0);
    assert(-1 != nEpollFd_);
    // vcFd_.clear();
    mpFd_.clear();
    for(auto& [key, value] : gmpConfig){
        strIp = value["ip"].get<std::string>();
        u16Port = value["port"];
        addFdToEpoll(strIp, u16Port, key);
    }
}

void Protocol::addFdToEpoll(const std::string& strSerIp, const unsigned short& u16SerPort, const std::string& strTty){
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    assert(-1 != sockfd);
    int nAlive{1};
    if(0 != setsockopt(sockfd, SOL_SOCKET, SO_KEEPALIVE, &nAlive, sizeof(nAlive))){
        std::cout << "setsockopt err:" << strerror(errno) << std::endl;
        close(sockfd);
        return;
    }
    
    auto status = fcntl(sockfd, F_GETFL);
    if(fcntl(sockfd, F_SETFL, status | O_NONBLOCK) < 0){
        std::cout << "fcntl set nonblock err:" << strerror(errno) << std::endl;
    }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr(strSerIp.data());
    addr.sin_port = htons(u16SerPort);
    int nRet{};
    while(1){
        nRet = connect(sockfd, (struct sockaddr*)&addr, sizeof(addr));
        //assert(-1 != nRet);
        if(-1 == nRet){
            //has setup connect
            if(EISCONN == errno){
                break;
            }
            std::cout << "connect server failed, connect ip:" << strSerIp << ", port:" << u16SerPort << ", err:" << strerror(errno) << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(10));
        }
        else{
            break;
        }
    }
    // vcFd_.push_back({sockfd, strTty});
    mpFd_[strTty] = sockfd;
    
    struct epoll_event ev;
    memset(&ev, 0, sizeof(ev));
    //EPOLLET: notice once
    ev.events = EPOLLIN | EPOLLET;
    ev.data.fd = sockfd;
    nRet = epoll_ctl(nEpollFd_, EPOLL_CTL_ADD, sockfd, &ev);
    if(-1 == nRet){
        std::cout << "epoll_ctl add failed:" << strerror(errno) << std::endl;
        return;
    }
    std::cout << "connected success, fd:" << sockfd << std::endl;
}

void Protocol::produceTcp(){
    while(1){
        std::unique_lock<std::mutex> lk(gmtTcp);
        // std::cout << "Protocol produceTcp wait lock:" << gdeTcpData.size() << std::endl;
        gcvTcp.wait(lk, [&](){
            return gdeTcpData.size() < nMaxPacketNum;
        });
        bFlag_ = false;
        start();
        // std::cout << "Protocol produceTcp bFlag_:" << bFlag_ << std::endl;
        if(bFlag_){
            gcvTcp.notify_all();
        }
    }
}

void Protocol::customTty(){
    while(1){
        struct stTtyData* pData{};
        {
            std::unique_lock<std::mutex> lk(gmtTty);
            std::cout << "Protocol customTty wait lock:" << gdeTtyData.size() << std::endl;
            gcvTty.wait(lk, [](){
                return !gdeTtyData.empty() ;
            });
            pData = gdeTtyData.front();
            gdeTtyData.pop_front();
            std::cout << "Protocol customTty" << std::endl;
            // if(gdeTtyData.empty()){
                std::cout << "Protocol customTty notify_all()" << std::endl;
                gcvTty.notify_all();
            // }
        }
        auto it = mpFd_.find(pData->strTty1);
        if(mpFd_.end() != it){
            uint8_t u8Arr[pData->nArrLen]{};
            int nRet{};
            std::copy(pData->u8Arr, pData->u8Arr + pData->nArrLen, u8Arr);
            nRet = write(it->second, u8Arr, pData->nArrLen);
            if(-1 == nRet){
                std::cout << "fd:[" << it->second << "] write tcp data:[" << u8Arr << "] error:" << strerror(errno) << std::endl;
            }
            free(pData);
            pData = nullptr;
            std::cout << "fd:[" << it->second << "] write tcp data:[" << u8Arr << "] finished" << std::endl;
        }
    }
}


void Protocol::start(){
    struct epoll_event ev[1024]{};
    int nReady{0};
    int nIndex{0};
    while(1){
        memset(ev, 0, sizeof(ev));
        nReady = epoll_wait(nEpollFd_, ev, 1024, 1000);
        if(nReady > 0){
            for(nIndex = 0; nIndex < nReady; ++nIndex){
                struct epoll_event stEv = ev[nIndex];
                uint32_t u32Event = stEv.events;
                int nFd = stEv.data.fd;
                //当前fd有数据可读
                if(u32Event & EPOLLIN){
                    std::cout << "fd:" << nFd << " has data to read" << std::endl;
                    onSocketRead(nFd);
                    return;
                }
                if(EWOULDBLOCK == errno || EAGAIN == errno){
                    continue;
                }
                // else if(u32Event & EPOLLOUT){
                //     std::cout << "fd:" << nFd << " has data to write" << std::endl;
                //     onSocketWrite(nFd);
                //     if(EWOULDBLOCK == errno || EAGAIN == errno){
                //         std::cout << "fd:" << nFd << " EAGAIN" << std::endl;
                //         continue;
                //     }
                // }
                if(u32Event & EINTR){
                    continue;
                }
                else{
                    std::cout << "fd:" << nFd << " epoll_wait unknown err:" << strerror(errno) << std::endl;
                }
            }
        }
        else if(-1 == nReady){
            std::cout << "nEpollFd_:" << nEpollFd_ << " tcp epoll_wait err:" << strerror(errno) << std::endl;
            // continue;
        }
    }
}

void Protocol::onSocketRead(int nFd){
    int nLen{0};
    int nLen1{0};
    uint8_t u8Arr[512]{};
    std::string strIp{};
    uint16_t u16Port{};
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    socklen_t len = sizeof(addr);

    if(-1 == getpeername(nFd, (struct sockaddr*)&addr, &len)){
        std::cout << "fd:" << nFd << ", getpeername err:" << strerror(errno) << std::endl;
    }
    strIp = inet_ntoa(addr.sin_addr);
    u16Port = ntohs(addr.sin_port);
    std::cout << "fd:" << nFd << ", getpeername ip:" << strIp << ", port:" << u16Port << std::endl;
    while((nLen1 = read(nFd, u8Arr + nLen, 512)) > 0){
        nLen += nLen1;
    }
    
    if(nLen > 0){
        auto it = searchIpPort(strIp, u16Port);
        if(gmpConfig.end() != it){
            stTcpData* pData;
            pData = (stTcpData*)malloc(sizeof(stTcpData) + nLen * sizeof(uint8_t));
            pData->strTty1 = it->first;
            pData->nArrLen = nLen;
            std::copy(u8Arr, u8Arr + nLen, pData->u8Arr);
            gdeTcpData.push_back(pData);
            bFlag_ = true;
        }
    }
    if(-1 == nLen1){
        if(errno == EAGAIN || errno == EWOULDBLOCK){
            return;
        }
        else{
            std::cout << "read buffer from tcp server err:" << strerror(errno) << std::endl;
        }
    }
}

// void Protocol::onSocketWrite(int nFd){
    // int nLen{0};
    // uint8_t u8Arr[512]{};
    // std::string strIp{};
    // uint16_t u16Port{};
    // struct sockaddr_in addr;
    // memset(&addr, 0, sizeof(addr));
    // socklen_t len = sizeof(addr);
    // if(-1 == getpeername(nFd, (struct sockaddr*)&addr, &len)){
    //     std::cout << "fd:" << nFd << ", getpeername err:" << strerror(errno) << std::endl;
    // }
    // strIp = inet_ntoa(addr.sin_addr);
    // u16Port = ntohs(addr.sin_port);
    // std::cout << "fd:" << nFd << ", getpeername ip:" << strIp << ", port:" << u16Port << std::endl;
    // auto it = searchIpPort(strIp, u16Port);
    // if(gmpConfig.end() != it){
    //     auto it1 = gmpSerialPort.find(it->first);
    //     if(gmpSerialPort.end() != it1){
    //         int nRead{};
    //         int nRead1{};
    //         std::thread{[&](){
    //             while(1){
    //                 while((nRead1 = it1->second.readBuffer(u8Arr + nRead, nLen)) > 0){
    //                     nRead += nRead1;
    //                 }
    //                 usleep(500);
    //             }
    //         }}.detach();
    //         std::cout << "fd:" << nFd << ", after readBuffer size:" << nRead << ", buf:" << std::endl;
    //         for(int i = 0; i < nRead; ++i){
    //             std::cout << u8Arr[i] << ' ';
    //         }
    //         std::cout << std::endl;

    //         //after reading data finished, send the data to tcp server
    //         if((nRead > 0) && (-1 == write(nFd, u8Arr, nRead))){
    //             std::cout << "fd:" << nFd << ", write buffer to tcp server err:" << strerror(errno) << std::endl;
    //         }
    //         if(-1 == nRead){
    //             std::cout << "fd:" << nFd << ", read buffer from tty err:" << strerror(errno) << std::endl;
    //         }
    //         else if(0 == nRead){
    //             std::cout << "fd:" << nFd << ", has no data to read, pt:" << strerror(errno) << std::endl;
    //         }
    //     }
    //     else{
    //         std::cout << "fd:" << nFd << ", gmpSerialPort not find the key:" << it->first << std::endl;
    //     }
    // }
    // else{
    //     std::cout << "fd:" << nFd << ", gmpConfig not find ip:" << strIp << ", port:" << u16Port << std::endl;
    // }
// }

std::map<std::string, nlohmann::json>::iterator Protocol::searchIpPort(const std::string& strIp, const uint16_t& u16Port) const{
    auto it = gmpConfig.begin();
    for(; it != gmpConfig.end(); ++it){
        auto strIp1 = (it->second)["ip"].get<std::string>();
        auto u16Port1 = (it->second)["port"];
        // std::cout << "while search, src ip and port:[" << strIp1 << ", " << u16Port1 << "] dest ip and port:[" << strIp << ", " << u16Port << "]" << std::endl;
        if((0 == strcmp(strIp.data(), strIp1.data())) && (u16Port == u16Port1)){
            return std::move(it);
        }
    }
    return gmpConfig.end();
}

std::map<std::string, nlohmann::json> gmpConfig;

}   // namespace LikeVcom