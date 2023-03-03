#include <iostream>

#include <fstream>
#include "protocol.h"
#include "serialPort.h"

namespace LikeVcom{

void init(){
    std::fstream file("config.json");
    if(!file.is_open()){
        std::cout << "config.json open failed" << std::endl;
        return;
    }
    nlohmann::json jsConfig;
    file >> jsConfig;
    file.close();
    gmpConfig.clear();
    // gmpSerialPort.clear();
    for(auto& elem : jsConfig["machine"]){
        gmpConfig[elem["tty1"].get<std::string>()] = elem;
    }
}

}

int main(int argc, char* argv[]){
    using namespace LikeVcom;
    init();
    Protocol pro;
    SerialPort sal;
    pro.init();
    sal.init();
    std::cout << "all init is finished, gmpConfig.size():[" << gmpConfig.size() << "], now start work" << std::endl;
    
    std::thread t1{[&](){
        pro.produceTcp();
    }};
    std::thread t2{[&](){
        pro.customTty();
    }};
    std::thread t3{[&](){
        sal.produceTty();
    }};
    std::thread t4{[&](){
        sal.customTcp();
    }};
    t1.join();
    t2.join();
    t3.join();
    t4.join();
    // sal.start();
    std::cout << "main finished" << std::endl;
    return 0;
}