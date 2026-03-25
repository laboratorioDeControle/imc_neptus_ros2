
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
#include <geometry_msgs/msg/pose.hpp>

#include <neptus_msgs/msg/plan_db.hpp>
#include <neptus_msgs/msg/plan_control.hpp>
#include <neptus_msgs/msg/remote_state.hpp>
#include <neptus_msgs/msg/estimated_state.hpp>
#include <neptus_msgs/msg/vehicle_state.hpp>
#include <neptus_msgs/msg/plan_control_state.hpp>
#include <neptus_msgs/msg/remote_actions_request.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <iostream>
//#include "udp_link.hpp"
#include "imc_handle.hpp"

//IMC_TO_ROS
#include <imc_ros_bridge/imc_to_ros/Heartbeat.h>
#include <imc_ros_bridge/imc_to_ros/Abort.h>
#include <imc_ros_bridge/imc_to_ros/Goto.h>
#include <imc_ros_bridge/imc_to_ros/PlanDB.h>
#include <imc_ros_bridge/imc_to_ros/PlanControl.h>
#include <imc_ros_bridge/imc_to_ros/RemoteActionsRequest.h>

//ROS_TO_IMC
#include <imc_ros_bridge/ros_to_imc/Heartbeat.h>
#include <imc_ros_bridge/ros_to_imc/Goto.h>
#include <imc_ros_bridge/ros_to_imc/RemoteState.h>
#include <imc_ros_bridge/ros_to_imc/GpsFix.h>
#include <imc_ros_bridge/ros_to_imc/GpsNavData.h>
#include <imc_ros_bridge/ros_to_imc/EstimatedState.h>
#include <imc_ros_bridge/ros_to_imc/VehicleState.h>
#include <imc_ros_bridge/ros_to_imc/PlanControlState.h>




int main(int argc, char * argv[])
{
  int bridge_port = 6002;
  int imc_src = 4;
  int imc_id = 30;
  
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("vehicle_node",
                                        rclcpp::NodeOptions()
                                            .allow_undeclared_parameters(true)
                                            .automatically_declare_parameters_from_overrides(true));
  
  std::string bridge_addr = node->get_parameter("bridge_address").as_string();
  std::string neptus_addr = node->get_parameter("neptus_address").as_string();
  std::string sys_name = node->get_parameter("sys_name").as_string();

  double start_lat = node->get_parameter("initial_lat").as_double();
  double start_long = node->get_parameter("initial_long").as_double();
  
  IMCHandle imc_handle(node, bridge_addr, bridge_port, neptus_addr, sys_name, start_lat, start_long, imc_id, imc_src);

  ros_to_imc::BridgeServer<std_msgs::msg::Empty, IMC::Heartbeat> heartbeat_server(node, imc_handle, "heartbeat");
  ros_to_imc::BridgeServer<geometry_msgs::msg::Pose, IMC::Goto> goto_server_dummy(node, imc_handle, "goto_input");
  ros_to_imc::BridgeServer<neptus_msgs::msg::RemoteState, IMC::RemoteState> RemoteState_server(node, imc_handle, "remote_state");
  ros_to_imc::BridgeServer<sensor_msgs::msg::NavSatFix, IMC::GpsFix> gpsfix_server(node, imc_handle, "gps_fix");
  ros_to_imc::BridgeServer<sensor_msgs::msg::NavSatFix, IMC::GpsNavData> gpsnavdata_server(node, imc_handle, "gps_nav_data");
  //EstimatedState for continous updates of location!
  ros_to_imc::BridgeServer<neptus_msgs::msg::EstimatedState, IMC::EstimatedState> estimatedstate_server(node, imc_handle, "estimated_state");
  ros_to_imc::BridgeServer<neptus_msgs::msg::PlanControlState, IMC::PlanControlState> plan_control_state_server(node, imc_handle, "plan_control_state");
  ros_to_imc::BridgeServer<neptus_msgs::msg::VehicleState, IMC::VehicleState> vehicle_supervisor(node, imc_handle, "vehicle_state");

  //150
  imc_to_ros::BridgeServer<IMC::Heartbeat, std_msgs::msg::Empty> imc_heartbeat_server(imc_handle, node, "imc_heartbeat");
  //550
  imc_to_ros::BridgeServer<IMC::Abort, std_msgs::msg::Empty> abort_server(imc_handle, node, "abort");
  //450
  imc_to_ros::BridgeServer<IMC::Goto, geometry_msgs::msg::Pose> goto_server(imc_handle, node, "goto_waypoint");
  //556
  imc_to_ros::BridgeServer<IMC::PlanDB, neptus_msgs::msg::PlanDB> plandb_server(imc_handle, node, "plan_db");
  //559
  imc_to_ros::BridgeServer<IMC::PlanControl, neptus_msgs::msg::PlanControl> plancontrol_server(imc_handle, node, "plan_control");
  //304
  imc_to_ros::BridgeServer<IMC::RemoteActionsRequest, neptus_msgs::msg::RemoteActionsRequest> remote_actions_request_server(imc_handle, node, "remote_actions_request");
        
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
