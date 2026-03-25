#include "udp_link.hpp"

#include <stdio.h>
#include <chrono>
#include <thread>
#include "rclcpp/rclcpp.hpp"

void die(const char *s)
{
    perror(s);
    exit(1);
}

void UDPLink::run() {
	while(threadActive)
	{
	    this->handle_receive();
	    std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
}

void UDPLink::run2() {

}

UDPLink::UDPLink(std::function<void (IMC::Message*)> recv_handler,
                    const std::string& bridge_addr,
                    const int& bridge_port,
                    int imc_id, 
                    int imc_src)
: bridge_addr(bridge_addr),
bridge_port(bridge_port),
recv_handler_(recv_handler),
imc_id(imc_id),
imc_src(imc_src)
{  
    //create a UDP socket
    if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
	    die("socket");
    }
	
    // zero out the structure
    memset((char *) &si_me, 0, sizeof(si_me));
    si_me.sin_family = AF_INET;
    si_me.sin_port = htons(bridge_port);
    si_me.sin_addr.s_addr = htonl(INADDR_ANY);//inet_addr("224.0.75.69");//htonl(INADDR_ANY); //inet_addr("224.0.75.69");
    
    // bind socket to port
    if( bind(s , (const struct sockaddr*)&si_me, sizeof(si_me) ) == -1)
        die("bind");
}

UDPLink::~UDPLink()
{
    threadActive = false;
}

void UDPLink::handle_receive()
{
    int recv_len;
    char buf[4096];
    socklen_t slen = sizeof(si_other);

    if ((recv_len = recvfrom(s, buf, 4096, 0, (struct sockaddr *)&si_other, &slen)) < 0)
    {
        threadActive = false;
        die("recvfrom()");
    }
    
    std::cout << "recv_len: " << recv_len << std::endl;
 
    for (int i = 0; i < recv_len; ++i) {
        IMC::Message* m = parser_.parse(buf[i]);
        //std::cout << "data: " << buf[i] << std::endl;
                
        if (m) {
            recv_handler_(m);
            delete m;
        }
    }
}

//announce
void UDPLink::publish_multicast(IMC::Message& msg, const std::string& multicast_addr)
{
    msg.setSource(imc_src);
    msg.setSourceEntity(imc_id);
    msg.setDestination(0);
    msg.setTimeStamp(rclcpp::Clock().now().seconds());
    
    char out_buffer_[4096];
    uint16_t rv = IMC::Packet::serialize(&msg, (uint8_t*)out_buffer_, sizeof(out_buffer_));
    
    for (int multicast_port : announce_ports)
    {
    	//std::cout << "Multicast_port: " << multicast_port << std::endl;
    	si_other.sin_family = AF_INET;
    	si_other.sin_port = htons(multicast_port);
    	si_other.sin_addr.s_addr = inet_addr(multicast_addr.c_str());
    	
    	if (sendto(s, out_buffer_, rv, 0, (struct sockaddr*)&si_other, sizeof(si_other)) == -1)
        {
            threadActive = false;
	        die("sendto()");
        }
    }
}

//heartbeat
void UDPLink::publish(IMC::Message& msg, const std::string& address)
{
   msg.setSource(imc_src);
   msg.setSourceEntity(imc_id);
   msg.setDestination(0);
   msg.setTimeStamp(rclcpp::Clock().now().seconds());
    
   char out_buffer_[4096];
   uint16_t rv = IMC::Packet::serialize(&msg, (uint8_t*)out_buffer_, sizeof(out_buffer_));

   si_other.sin_family = AF_INET;
   si_other.sin_port = htons(6001);
   si_other.sin_addr.s_addr = inet_addr(address.c_str());
   
   if (sendto(s, out_buffer_, rv, 0, (struct sockaddr*)&si_other, sizeof(si_other)) == -1)
   {
        threadActive = false;
	    die("sendto()");
   }

}



