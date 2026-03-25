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

#include "imc_handle.hpp"
#include <IMC/Spec/Announce.hpp>
#include <IMC/Spec/Heartbeat.hpp>
#include <IMC/Spec/EntityInfo.hpp>

#include <functional>
#include "rclcpp/rclcpp.hpp"

void try_callback(const IMC::Message* imc_msg)
{
		//RCLCPP_INFO("Got callback!");
		std::cout << "Got callback with id: " << imc_msg->getId() << std::endl;
}

IMCHandle::IMCHandle(rclcpp::Node::SharedPtr ros_node,
                     const std::string& bridge_addr, 
					 const int& bridge_port,
                     const std::string& neptus_addr,
                     const std::string& sys_name,
                     const double initial_lat,
                     const double initial_long,
					 int imc_id, 
					 int imc_src)
    : node(ros_node),
      bridge_addr(bridge_addr),
      bridge_port(bridge_port),
      neptus_addr(neptus_addr),
      sys_name(sys_name), imc_id(imc_id), imc_src(imc_src),
      udp_link(std::bind(&IMCHandle::udp_callback, this, std::placeholders::_1), bridge_addr, bridge_port, imc_id, imc_src)
{
    lat = 0.0;
    udp_link.start();

    initial_latitude = initial_lat;
    initial_longitude = initial_long;

    timer_announce = node->create_wall_timer(std::chrono::seconds(10), std::bind(&IMCHandle::announce, this));
    timer_heartbeat = node->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&IMCHandle::publish_heartbeat, this));
    this->announce();
}

IMCHandle::~IMCHandle()
{
    udp_link.join();
}

void IMCHandle::udp_subscribe(uint16_t uid, std::function<void(const IMC::Message*)> callback)
{
    std::cout << "Id: " << uid << std::endl;
    callbacks[uid] = callback;
}

void IMCHandle::udp_callback(const IMC::Message* msg)
{
    uint16_t uid = msg->getId();
    if (callbacks.count(uid) > 0) {
		// 150 is a heartbeat and we dont really care about it. just debug it.
		// 556 is PlanDB, i _think_ its the planDB succss, meaning "i understood that you got my plan"
		// neptus basically spams this so im excluding it!
		if(uid == 150 || uid == 556){
			RCLCPP_INFO(node->get_logger(), "Got callback with id: %u", uid);
		}else{
			RCLCPP_INFO(node->get_logger(), "Got callback with id: %u", uid);
		}
		callbacks.at(uid)(msg);
    }
    else {
        RCLCPP_INFO(node->get_logger(), "Got udp message with no configure callback, msgid: %u!", uid);
	    // lets just print the whole message in json format if we can't parse it yet.
	    std::cout << "Message name: " << msg->getName() << std::endl << "Message JSON:" << std::endl;
	    // (ostream, indent)
	    msg->fieldsToJSON(std::cout, 4);
	    std::cout << std::endl;
        std::cout << "---------------------" << std::endl;
    }
}

void IMCHandle::announce()
{
    //std::string announce_addr = "224.0.75.69";
    //std::string announce_addr = "192.168.1.160";

    IMC::Announce msg;
    msg.sys_name = sys_name;
    // 0=CCU, 1=HUMANSENSOR, 2 = UUV, 3 = ASV, 4=UAV, 5=UGV, 6=STATICSENSOR
    msg.sys_type = 2; // UUV = Unmanned underwater veh.
    msg.owner = 0;
    // dont put location info here, this is only updated once
    // use EstimatedState for continous updates of location.
    //lat +=0.01;
    msg.lat = initial_latitude; // -0.40174237833249615; // -0.39840630835274565;
    msg.lon = initial_longitude; // -0.773666187225512; // -0.7535904104444776;
    //msg.height = -1.;
    //msg.services = "imc+info://0.0.0.0/version/5.4.11/;imc+udp://127.0.0.1:6002/;";
    msg.services = "imc+udp://" + bridge_addr + ":" + std::to_string(bridge_port) + "/;";
    udp_link.publish_multicast(msg, neptus_addr);

    //TEST Publish EntityInfo
    IMC::EntityInfo info_msg;
    info_msg.id = imc_src; //What is this used for?
    info_msg.label = sys_name;
    udp_link.publish(info_msg, neptus_addr);
}

void IMCHandle::publish_heartbeat()
{
    IMC::Heartbeat msg;
    udp_link.publish(msg, neptus_addr);
}
