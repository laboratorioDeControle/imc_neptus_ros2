# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch a talker and a listener in a component container."""

import launch
import launch.actions
import launch.substitutions
import launch_ros


def generate_launch_description():

    launch.actions.DeclareLaunchArgument('params_route', default_value='~/imc_neptus_ros2/config/params.yaml'),
    config = launch.substitutions.LaunchConfiguration('params_route')

    vehicle_node = launch_ros.actions.Node(package='imc_ros2_bridge',
                                           name='vehicle_node',
                                           executable='vehicle_node',
                                           parameters=[config],
                                           output='screen')

    vessel_node = launch_ros.actions.Node(package='imc_ros2_bridge',
                                     name='vessel_node',
                                     executable='vessel_node',
                                     parameters=[config],
                                     output='screen')

    transponder_node = launch_ros.actions.Node(package='imc_ros2_bridge',
                                           name='transponder_node',
                                           executable='transponder_node', 
                                           parameters=[config],
                                           output='screen')

    return launch.LaunchDescription([
        vehicle_node,
        vessel_node,
        transponder_node
    ])
