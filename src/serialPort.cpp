#include "serialPort.h"
#include "ds.h"

namespace LikeVcom{

    SerialPort::SerialPort()
        : nEpollFd_(-1)
        , mpBaudrate_{{0, B0}, {50, B50}, {75, B75}, {110, B110},
            {134, B134}, {150, B150}, {200, B200}, {300, B300},
            {600, B600}, {1200, B1200}, {1800, B1800}, {2400, B2400},
            {4800, B4800}, {9600, B9600}, {19200, B19200},
            {38400, B38400}, {57600, B57600}, {115200, B115200},
            {230400, B230400}, {460800, B460800}}{
                std::cout << "SerialPort()" << std::endl;
    }

    SerialPort::~SerialPort(){
        std::cout << "~SerialPort()" << std::endl;
        closeSerialPort();
        std::lock_guard<std::mutex> lk(gmtTty);
        while(!gdeTtyData.empty()){
            auto p = gdeTtyData.front();
            gdeTtyData.pop_front();
            free(p);
            p = nullptr;
        }
    }


    int SerialPort::openSerialPort(const std::string& strPortName){
        int nFd{};
        strPortName_ = strPortName;
        /*以读写方式打开串口*/
        // nFd_ = open( "/dev/ttyS0", O_RDWR);
        std::string strDev = gstrSerialPath + strPortName;
        std::cout << "serial open path:" << strDev << std::endl;
        nFd = open(strDev.data(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (-1 == nFd){ 
            /* 不能打开串口一*/ 
            std::cout << "serial open failed:" << errno << ", " << strerror(errno) << std::endl;;
        }
        return std::move(nFd);
    }

    void SerialPort::setBaudrate(int nFd, const int& nBaudrate){
        int nStatus; 
        struct termios stOpt;
        auto it = mpBaudrate_.find(nBaudrate);
        if(mpBaudrate_.end() != it){
            tcgetattr(nFd, &stOpt); 
            tcflush(nFd, TCIOFLUSH);     
            cfsetispeed(&stOpt, it->second);  
            cfsetospeed(&stOpt, it->second);   
            nStatus = tcsetattr(nFd, TCSANOW, &stOpt);  
            if  (0 != nStatus) {        
                std::cout << "tcsetattr fd:" << nFd << std::endl;  
                return;     
            }    
            tcflush(nFd,TCIOFLUSH);   
        }
    }

    /**
    *@brief   设置串口数据位，停止位和效验位
    *@param  nDatabits 类型  int 数据位   取值 为 7 或者8
    *@param  nStopbits 类型  int 停止位   取值为 1 或者2
    *@param  cParity  类型  int  效验类型 取值为N,E,O,,S
    */
    int SerialPort::setParity(int nFd, int nDatabits, int nStopbits, char cParity){ 
        struct termios stOptions; 
        if  (tcgetattr(nFd, &stOptions) != 0) { 
            perror("SetupSerial 1");     
            return -1;  
        }
        stOptions.c_cflag &= ~CSIZE; 
        switch (nDatabits){   
        case 7:{
            stOptions.c_cflag |= CS7; 
            break;
        }
        case 8:{     
            stOptions.c_cflag |= CS8;
            break;   
        }
        default:{
            fprintf(stderr,"Unsupported data size\n"); 
            return -1;  
        }
        }
        switch (cParity) {   
        case 'n':
        case 'N':{  
            stOptions.c_cflag &= ~PARENB;   /* Clear parity enable */
            stOptions.c_iflag &= ~INPCK;    /* Enable parity checking */ 
            break;
        }  
        case 'o':   
        case 'O':{
            stOptions.c_cflag |= (PARODD | PARENB); /* 设置为奇效验*/  
            stOptions.c_iflag |= INPCK;             /* Disnable parity checking */ 
            break;
        }  
        case 'e':  
        case 'E':{
            stOptions.c_cflag |= PARENB;     /* Enable parity */    
            stOptions.c_cflag &= ~PARODD;   /* 转换为偶效验*/     
            stOptions.c_iflag |= INPCK;       /* Disnable parity checking */
            break;
        }
        case 'S': 
        case 's':{
            stOptions.c_cflag &= ~PARENB;
            stOptions.c_cflag &= ~CSTOPB;
            break;
        }  
        default:{
            fprintf(stderr,"Unsupported parity\n");    
            return -1;  
        }
        }  
        switch (nStopbits){   
        case 1:{    
            stOptions.c_cflag &= ~CSTOPB;  
            break;  
        }
        case 2:{
            stOptions.c_cflag |= CSTOPB;  
            break;
        }
        default:{
            fprintf(stderr,"Unsupported stop bits\n");  
            return -1; 
        }
        } 
        /* Set input parity option */ 
        if (cParity != 'n'){
            stOptions.c_iflag |= INPCK; 
        }
        tcflush(nFd, TCIFLUSH);
        stOptions.c_cc[VTIME] = 10; /* 设置超时1 seconds*/   
        stOptions.c_cc[VMIN] = 0; /* Update the options and do it NOW */
        if (tcsetattr(nFd, TCSANOW, &stOptions) != 0){ 
            perror("SetupSerial 3");   
            return -1;  
        } 
        return 0;  
    }

    void SerialPort::init(){
        int nFd{};
        int nRet{};
        int nBaudrate{};
        int nDatabits{};
        int nStopbits{};
        std::string strParity{};
        char cParity{};
        mpFd_.clear();
        nEpollFd_ = epoll_create1(0);
        assert(-1 != nEpollFd_);
        for(auto& [key, value] : gmpConfig){
            nDatabits = value["databit"];
            nStopbits = value["stopbit"];
            strParity = value["parity"].get<std::string>();
            cParity = strParity[0];
            nBaudrate = value["baudrate"];

            nFd = openSerialPort(key);
            setBaudrate(nFd, nBaudrate);
            nRet = setParity(nFd, nDatabits, nStopbits, cParity);
            if(-1 == nRet){
                std::cout << key << " setParity err" << std::endl;
            }
            std::cout << key << " opened, baudrate:" << nBaudrate << ", databit:" << nDatabits << ", stopbits:" << 
                nStopbits << ", parity:" << cParity << std::endl;
            // vcFd_.push_back({nFd, key});
            mpFd_[key] = nFd;

            struct epoll_event ev;
            memset(&ev, 0, sizeof(ev));
            //EPOLLET: notice once
            ev.events = EPOLLIN | EPOLLET;
            ev.data.fd = nFd;
            nRet = epoll_ctl(nEpollFd_, EPOLL_CTL_ADD, nFd, &ev);
            if(-1 == nRet){
                std::cout << "epoll_ctl add failed:" << strerror(errno) << std::endl;
                return;
            }
        }
    }

    void SerialPort::produceTty(){
        while(1){
            std::unique_lock<std::mutex> lk(gmtTty);
            // std::cout << "serialPort produceTty wait lock:" << gdeTtyData.size() << std::endl;
            gcvTty.wait(lk, [](){
                return gdeTtyData.size() < nMaxPacketNum;
            });
            bFlag_ = false;
            std::cout << "serialPort produceTty before start" << std::endl;
            start();
            // std::cout << "serialPort finished bFlag_:" << bFlag_ << std::endl;
            if(bFlag_){
            // if(gdeTtyData.size() >= nMaxPacketNum){
                std::cout << "produceTty notify" << std::endl;
                gcvTty.notify_all();
            }
        }
    }

    void SerialPort::customTcp(){
        while(1){
            struct stTcpData* pData{};
            {
                std::unique_lock<std::mutex> lk(gmtTcp);
                std::cout << "serialPort customTcp wait lock:" << gdeTcpData.size() << std::endl;
                gcvTcp.wait(lk, [&](){
                    return !gdeTcpData.empty();
                });
                pData = gdeTcpData.front();
                gdeTcpData.pop_front();
                std::cout << "serialPort customTcp" << std::endl;
                gcvTcp.notify_all();
            }
            auto it = mpFd_.find(pData->strTty1);
            if(mpFd_.end() != it){
                uint8_t u8Arr[pData->nArrLen]{};
                int nRet{};
                std::copy(pData->u8Arr, pData->u8Arr + pData->nArrLen, u8Arr);
                nRet = write(it->second, u8Arr, pData->nArrLen);
                if(-1 == nRet){
                    std::cout << "fd:[" << it->first << "] tty write tty data:[" << u8Arr << "] error:" << strerror(errno) << std::endl;
                }
                free(pData);
                pData = nullptr;
                std::cout << "fd:[" << it->first << "] tty write tty data:[" << u8Arr << "] finished" << std::endl;
            }
        }
    }

    void SerialPort::start(){
        struct epoll_event ev[1024]{};
        int nReady{0};
        int nIndex{0};
        while(1){
            memset(ev, 0, sizeof(ev));
            //milliseconds
            nReady = epoll_wait(nEpollFd_, ev, 1024, -1);
            // std::cout << "serialPort enter epoll_wait ret:" << nReady << std::endl;
            if(nReady > 0){
                for(nIndex = 0; nIndex < nReady; ++nIndex){
                    struct epoll_event stEv = ev[nIndex];
                    uint32_t u32Event = stEv.events;
                    int nFd = stEv.data.fd;
                    //当前fd有数据可读
                    if(u32Event & EPOLLIN){
                        std::cout << "fd:" << nFd << " has data to read" << std::endl;
                        onSerialRead(nFd);
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
                std::cout << "nEpollFd_:" << nEpollFd_ << " tty epoll_wait err:" << strerror(errno) << std::endl;
                // continue;
            }
        }
    }

    void SerialPort::onSerialRead(int nFd){
        uint8_t u8Arr[1024]{};
        int nRead{0};
        int nRead1{0};
        while((nRead1 = read(nFd, u8Arr + nRead, 512)) > 0){
            nRead += nRead1;
        }
        std::cout << "onSerialRead read len:" << nRead << ", data: " << u8Arr << std::endl;
        if(nRead > 0){   //此处认为一包完整的数据读完了
            auto it = std::find_if(mpFd_.begin(), mpFd_.end(), 
                [&](const std::map<std::string, int>::value_type elem){
                    return elem.second == nFd;
                });
            if(mpFd_.end() != it){
                // stTtyData pData;
                stTtyData* pData = (stTtyData*)malloc(sizeof(stTtyData) + sizeof(uint8_t) * nRead);
                if(nullptr != pData){
                    // std::cout << "memory ok:" << it->first << ", " << nRead << std::endl;
                    pData->strTty1 = it->first;
                    pData->nArrLen = nRead;
                    std::copy(u8Arr, u8Arr + nRead, pData->u8Arr);
                    std::cout << "u8Arr:" << pData->u8Arr << std::endl;
                    gdeTtyData.push_back(pData);
                    bFlag_ = true;
                }
                else{
                    std::cout << "memory failed" << std::endl;
                }
            }
        }
    }

    void SerialPort::closeSerialPort(){
        int nRet{0};
        // for(auto& elem : vcFd_){
        for(auto& elem : mpFd_){
            nRet = epoll_ctl(nEpollFd_, EPOLL_CTL_DEL, elem.second, nullptr);
            if(-1 == nRet){
                std::cout << "tty nEpollFd_:[" << nEpollFd_ << "] delete fd:[" << elem.second << "] failed:" << strerror(errno) << std::endl;
            }
            close(elem.second);
        }
        close(nEpollFd_);
    }

    std::map<std::string, SerialPort> gmpSerialPort;
    std::deque<stTtyData*> gdeTtyData;
    std::deque<stTcpData*> gdeTcpData;

    std::mutex gmtTty;
    std::mutex gmtTcp;
    std::condition_variable gcvTty;
    std::condition_variable gcvTcp;

    int nMaxPacketNum{4};
}   //namespace LickVcom