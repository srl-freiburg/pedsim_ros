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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')  
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')
    # Set env var to print messages to stdout immediately
    SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')
    frame_id = LaunchConfiguration('frame_id')
    walls_resolution = LaunchConfiguration('walls_resolution')

    frame_id_cmd = DeclareLaunchArgument(
        'frame_id', default_value='map', description='Reference frame')

    walls_resolution_cmd = DeclareLaunchArgument(
        'walls_resolution', default_value='0.2', description='Obstacles walls resolution')

    visualizer_cmd = Node(
        package='pedsim_visualizer',
        executable='pedsim_visualizer_node',
        name='pedsim_visualizer_node',
        namespace=namespace,
        output='screen',
        parameters=[{
            "frame_id": frame_id,
            "walls_resolution": walls_resolution
        }]
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(frame_id_cmd)
    ld.add_action(walls_resolution_cmd)

    # Declare the launch options
    ld.add_action(visualizer_cmd)

    return ld
