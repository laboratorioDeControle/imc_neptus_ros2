cmake_minimum_required(VERSION 3.5)
project(imc_ros2_bridge)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(neptus_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
  external/imc-cxx
  external/md5
  ${rclcpp_INCLUDE_DIRS}
)

## Declare a C++ library
add_library( md5 external/md5/md5.cpp )
add_library( imc_factory external/imc-cxx/IMC/Base/Factory.cpp )


file(GLOB SOURCE_FILES
 src/udp_link.cpp
 src/imc_handle.cpp
 src/imc_to_ros/Goto.cpp
 src/imc_to_ros/Abort.cpp
 src/imc_to_ros/Heartbeat.cpp
 src/imc_to_ros/PlanDB.cpp
 src/imc_to_ros/PlanControl.cpp
 src/imc_to_ros/RemoteActionsRequest.cpp

 src/ros_to_imc/Heartbeat.cpp
 src/ros_to_imc/Goto.cpp
 src/ros_to_imc/RemoteState.cpp
 src/ros_to_imc/GpsFix.cpp
 src/ros_to_imc/GpsNavData.cpp
 src/ros_to_imc/EstimatedState.cpp
 src/ros_to_imc/VehicleState.cpp
 src/ros_to_imc/PlanControlState.cpp
)

add_executable(vehicle_node src/vehicle_node.cpp ${SOURCE_FILES})
ament_target_dependencies(vehicle_node rclcpp
    "sensor_msgs" 
    "geometry_msgs"
    "neptus_msgs"
    "tf2"
    "tf2_geometry_msgs"
)

target_link_libraries(vehicle_node md5 imc_factory)

#Install C++ targets
install(TARGETS
  vehicle_node
  DESTINATION lib/${PROJECT_NAME}
)


#Install Launch file
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()

