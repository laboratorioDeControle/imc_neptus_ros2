#ifndef UDP_LINK_H
#define UDP_LINK_H


#include <thread>
#include <iostream>
#include <string>
#include <stdlib.h> //exit(0);
#include <vector>

#include<stdio.h>	//printf
#include<string.h> //memset
#include<arpa/inet.h>
#include<sys/socket.h>
#include <functional>

#include "CppThread.h"
// IMC headers.
#include <IMC/Base/Parser.hpp>


#include "rclcpp/rclcpp.hpp"



class UDPLink : public CppThread {

public:

    UDPLink(std::function<void (IMC::Message*)> recv_handler,
              const std::string& bridge_addr,
              const int& bridge_port,
              int imc_id, 
              int imc_src);

    ~UDPLink();
    //bool threadActive = false;

    void publish(IMC::Message& msg, const std::string& address);
    void publish_multicast(IMC::Message& msg, const std::string& multicast_addr);
  
private:
    std::string bridge_addr;
    int bridge_port;
    std::function<void (IMC::Message*)> recv_handler_;
    IMC::Parser parser_;
    int imc_id;
    int imc_src;
    int imc_src_ent;
    
    //socket
    int s;
    struct sockaddr_in si_me, si_other;
    //int slen = sizeof(si_other);
    char buf[512];
    
    
    std::vector<int> announce_ports{30100, 30101, 30102, 30103, 30104};
    
    void run();
    void run2();
    void handle_receive();
    


};

#endif // UDP_LINK_H
