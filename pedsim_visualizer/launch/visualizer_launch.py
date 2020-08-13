
# Copyright (c) 2020 Intelligent Robotics Lab
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
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get the launch directory    
    
    # Set env var to print messages to stdout immediately
    SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')
    frame_id = LaunchConfiguration('frame_id')
    
    frame_id_cmd = DeclareLaunchArgument(
        'frame_id', default_value='odom', description='Reference frame')

    visualizer_cmd = Node(
        package='pedsim_visualizer',
        node_executable='pedsim_visualizer_node',
        node_name='pedsim_visualizer_node',
        output='screen',
        parameters=[{
            "frame_id": frame_id
        }]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(frame_id_cmd)

    # Declare the launch options
    ld.add_action(visualizer_cmd)

    return ld
