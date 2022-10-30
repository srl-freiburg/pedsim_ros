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
    # Get package directory
    simulator_dir = get_package_share_directory('pedsim_simulator')
    # Get config file
    config_file_path = os.path.join(simulator_dir, 'config', 'params.yaml')

    # Launch configuration
    scene_file = LaunchConfiguration('scene_file')
    config_file = LaunchConfiguration('config_file')


    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),

        DeclareLaunchArgument(
            'scene_file', 
            default_value=os.path.join(simulator_dir, 'scenarios', 'social_contexts.xml'),
            description=''),
        DeclareLaunchArgument(
            'config_file', 
            default_value=config_file_path,
            description=''),        
        Node(
            package='pedsim_simulator',
            executable='pedsim_simulator',
            name='pedsim_simulator',
            output='screen',
            parameters=[{'scene_file': scene_file}, config_file]),
        Node(
            package='pedsim_tf2',
            executable='pedsim_tf2_node',
            name='pedsim_tf2_node',
            output='screen'),
        

    ])
