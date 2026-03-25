/* Copyright 2019 The SMaRC project (https://smarc.se/)
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef UDP_LINK_H
#define UDP_LINK_H

#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>
#include <thread>
#include <iostream>

#include <string>

#include "rclcpp/rclcpp.hpp"

// IMC headers.
#include <IMC/Base/Parser.hpp>


//#define IPADDRESS "127.0.0.1" // "192.168.1.64"
//#define UDP_PORT 30101 //6001

void handler(const boost::system::error_code& error, size_t bytes_transferred)
{

}

using boost::asio::ip::udp;
using boost::asio::ip::address;

class UDPLink {

private:

    std::string bridge_addr;
    std::string bridge_port;

    boost::asio::io_service io_service;
    udp::socket socket{io_service};
    udp::socket multicast_socket{io_service};
    // 32KiB in bits. Max UDP packet size.
    boost::array<char, 262144> recv_buffer;
    udp::endpoint remote_endpoint;
    std::function<void (IMC::Message*)> recv_handler_;
    IMC::Parser parser_;
    boost::thread run_thread;

    std::vector<int> announce_ports{30100, 30101, 30102, 30103, 30104};

    bool should_shutdown;

    int imc_src = 4;
    int imc_src_ent = 32;
	int imc_id;

public:

    UDPLink(std::function<void (IMC::Message*)> recv_handler,
        const std::string& bridge_addr, const std::string& bridge_port, int imc_id, int imc_src)
    : recv_handler_(recv_handler), bridge_addr(bridge_addr), bridge_port(bridge_port), imc_id(imc_id), imc_src(imc_src)
{
    socket.open(udp::v4());
    socket.set_option(udp::socket::reuse_address(true));
    socket.bind(udp::endpoint(address::from_string(bridge_addr), boost::lexical_cast<int>(bridge_port)));

    should_shutdown = false;

    multicast_socket.open(boost::asio::ip::udp::v4());

    wait();

    //auto work = std::make_shared<boost::asio::io_service::work>(io_service);

    run_thread = boost::thread(boost::bind(&boost::asio::io_service::run, &io_service));

}

~UDPLink()
{
    //multicast_socket.shutdown();
    should_shutdown = true;
    multicast_socket.close();
    run_thread.join();
}




void publish(IMC::Message& msg, const std::string& addr)
{
    msg.setSource(imc_src);
    msg.setSourceEntity(imc_id);
    msg.setDestination(0);
    //msg.setTimeStamp(ros::Time::now().toSec());

    char out_buffer_[4096];
    uint16_t rv = IMC::Packet::serialize(&msg, (uint8_t*)out_buffer_, sizeof(out_buffer_));

    udp::endpoint destination(address::from_string(addr), 6001);
    socket.async_send_to(boost::asio::buffer(out_buffer_, rv), destination, handler);

    /*
    myfile.open ("/tmp/test.lsf", ios::out | ios::app | ios::binary);
    myfile.write(out_buffer_, rv);
    myfile.flush();
    myfile.close();
    */
}

void publish_multicast(IMC::Message& msg, const std::string& multicast_addr)
{
    msg.setSource(imc_src);
    msg.setSourceEntity(imc_id);
    msg.setDestination(0);
    //msg.setTimeStamp(ros::Time::now().toSec());

    char out_buffer_[1024];
    uint16_t rv = IMC::Packet::serialize(&msg, (uint8_t*)out_buffer_, sizeof(out_buffer_));

    std::string message;
    for (int multicast_port : announce_ports)
    {
		// std::cout << "Writing to port: " << multicast_port << std::endl;
        udp::endpoint destination(address::from_string(multicast_addr), multicast_port);
        multicast_socket.async_send_to(boost::asio::buffer(out_buffer_, rv), destination, handler);
    }

}

void wait()
{
    socket.async_receive_from(boost::asio::buffer(recv_buffer),
                              remote_endpoint,
                              boost::bind(&UDPLink::handle_receive,
                                          this, boost::asio::placeholders::error,
                                          boost::asio::placeholders::bytes_transferred));
}

void handle_receive(const boost::system::error_code& error, size_t bytes_transferred)
{
    if (error) {
        std::cout << "Receive failed: " << error.message() << "\n";
        return;
    }

    // std::cout << "Received by udp: '" << std::string(recv_buffer.begin(), recv_buffer.begin()+bytes_transferred) << "' (" << error.message() << ") ";
    // std::cout << bytes_transferred << " bytes transferred" << std::endl;

    // attempt to read the datagram header. bytes 18-19 have size info.
//    char buf[2];
//   buf[0] = recv_buffer[18];
//    buf[1] = recv_buffer[19];
//    unsigned int num_bytes = buf[0] | buf[1] << 8;
//    std::cout << "(" << num_bytes << ") bytes should have been received according to UDP header." << std::endl;
    // it seems like the socket does not return the whole package (as evidenced by bytes_transferred being smaller than what wireshark captures).
    // so we are only receiving the payload here.

    for (size_t i = 0; i < bytes_transferred; ++i) {
        IMC::Message* m = parser_.parse(recv_buffer[i]);
        if (m) {
            recv_handler_(m);
            delete m;
        }
    }

    if (!should_shutdown) {
        wait();
    }
}

};

#endif // UDP_LINK_H
