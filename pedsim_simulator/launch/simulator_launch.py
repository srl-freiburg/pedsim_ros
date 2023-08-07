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
from launch.conditions import IfCondition

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    # Get package directory
    simulator_dir = get_package_share_directory('pedsim_simulator')
    # Get config file
    config_file_path = os.path.join(simulator_dir, 'config', 'params.yaml')

    # Launch configuration
    scene_file = LaunchConfiguration('scene_file')
    config_file = LaunchConfiguration('config_file')
    run_pedsim_tf2 = LaunchConfiguration('run_pedsim_tf2')


    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace'),
        DeclareLaunchArgument(
            'scene_file', 
            default_value=os.path.join(simulator_dir, 'scenarios', 'office-cubicles.xml'),
            description='Path to the cenario file'),
        DeclareLaunchArgument(
            'config_file', 
            default_value=config_file_path,
            description='Path to the simulator config file'),  
        DeclareLaunchArgument(
            'run_pedsim_tf2', 
            default_value='False',
            description='Whether to launch pedestrian tf2 node'),        
        Node(
            package='pedsim_simulator',
            executable='pedsim_simulator',
            name='pedsim_simulator',
            output='screen',
            namespace=namespace,
            parameters=[{'scene_file': scene_file}, config_file]),
        Node(
            package='pedsim_tf2',
            executable='pedsim_tf2_node',
            name='pedsim_tf2_node',
            output='screen',
            namespace=namespace,
            condition=IfCondition(run_pedsim_tf2))
    ])
