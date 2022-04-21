# Copyright (c) 2018 Intelligent Robotics Lab
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    simulator_dir = get_package_share_directory('pedsim_simulator')

    scene_file = LaunchConfiguration('scene_file')
    default_queue_size = LaunchConfiguration('default_queue_size')
    max_robot_speed = LaunchConfiguration('max_robot_speed')
    robot_mode = LaunchConfiguration('robot_mode')
    robot_radius = LaunchConfiguration('robot_radius')
    agent_radius = LaunchConfiguration('agent_radius')
    force_factor_social = LaunchConfiguration('force_factor_social')
    enable_groups = LaunchConfiguration('enable_groups')
    simulation_factor = LaunchConfiguration('simulation_factor')
    update_rate = LaunchConfiguration('update_rate')

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),

        DeclareLaunchArgument(
            'scene_file', 
            default_value=os.path.join(simulator_dir, 'scenarios', 'social_context.xml'),
            description=''),
        
        DeclareLaunchArgument(
            'default_queue_size', default_value='1',
            description=''),

        DeclareLaunchArgument(
            'max_robot_speed', default_value='1.5',
            description=''),

        DeclareLaunchArgument(
            'robot_mode', default_value='0',
            description=''),
        DeclareLaunchArgument(
            'robot_radius', default_value='0.35',
            description=''),
        DeclareLaunchArgument(
            'agent_radius', default_value='0.4',
            description=''),
        DeclareLaunchArgument(
            'force_factor_social', default_value='12.0',
            description=''),

        DeclareLaunchArgument(
            'enable_groups', default_value='true',
            description=''),

        DeclareLaunchArgument(
            'simulation_factor', default_value='1.0',
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'update_rate', default_value='25.0',
            description=''),

        Node(
            package='pedsim_simulator',
            executable='pedsim_simulator',
            name='pedsim_simulator',
            output='screen',
            parameters=[{'scene_file': scene_file},
                        {'default_queue_size': default_queue_size},
                        {'max_robot_speed': max_robot_speed},
                        {'robot_mode': robot_mode},
                        {'robot_radius': robot_radius},
                        {'agent_radius': agent_radius},
                        {'force_factor_social': force_factor_social},
                        {'enable_groups': enable_groups},
                        {'simulation_factor': simulation_factor},
                        {'update_rate': update_rate}]),
        Node(
            package='pedsim_tf2',
            executable='pedsim_tf2_node',
            name='pedsim_tf2_node',
            output='screen'),
        

    ])
