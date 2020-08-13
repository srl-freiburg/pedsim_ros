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
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get the launch directory
    simulator_dir = get_package_share_directory('pedsim_simulator')
    launch_dir = os.path.join(simulator_dir, 'launch')
    
    scene_file = LaunchConfiguration('scene_file')
    simulation_factor = LaunchConfiguration('simulation_factor')
    frame_id = LaunchConfiguration('frame_id')
    
    # Set env var to print messages to stdout immediately
    SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    declare_scene_file_cmd = DeclareLaunchArgument(
        'scene_file', 
        default_value=os.path.join(simulator_dir, 'scenarios', 'tb3_house_demo_crowd.xml'),
        description='')

    declare_simulation_factor_cmd = DeclareLaunchArgument(
        'simulation_factor', default_value='0.07',
        description='Top-level namespace')

    frame_id_cmd = DeclareLaunchArgument(
        'frame_id', default_value='map', description='Reference frame')

    pedsim_simulator_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'simulator_launch.py')),
        launch_arguments={'scene_file': scene_file,
                          'simulation_factor': simulation_factor}.items()
    )    

    pedsim_visualizer_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('pedsim_visualizer'), 'launch', 'visualizer_launch.py')),
        launch_arguments={'frame_id': frame_id}.items()
    )  
    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_scene_file_cmd)
    ld.add_action(declare_simulation_factor_cmd)
    ld.add_action(frame_id_cmd)

    # Add any conditioned actions
    ld.add_action(pedsim_visualizer_cmd)
    ld.add_action(pedsim_simulator_cmd)

    return ld
