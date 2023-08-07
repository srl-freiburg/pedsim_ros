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
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

def generate_launch_description():
    # Get directories
    simulator_dir = get_package_share_directory('pedsim_simulator')
    launch_dir = os.path.join(simulator_dir, 'launch')
    default_rviz_config_path = os.path.join(simulator_dir, 'rviz', 'pedsim.rviz')
    default_config_file_path = os.path.join(simulator_dir, 'config', 'params.yaml')
    default_scene_file_path = os.path.join(simulator_dir, 'scenarios', 'tb3_house_demo_crowd.xml')

    namespace = LaunchConfiguration('namespace')
    scene_file = LaunchConfiguration('scene_file')
    config_file = LaunchConfiguration('config_file')
    frame_id = LaunchConfiguration('frame_id')
    rviz = LaunchConfiguration('rviz')
    use_rviz = LaunchConfiguration('use_rviz')

    # Set env var to print messages to stdout immediately
    SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_rvizconfig_cmd = DeclareLaunchArgument(
        name='rviz', default_value=default_rviz_config_path,
        description='Absolute path to rviz config file')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz', default_value='False',
        description='Whether to use rviz')

    declare_scene_file_cmd = DeclareLaunchArgument(
        'scene_file', 
        default_value=default_scene_file_path,
        description='')

    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file', 
        default_value=default_config_file_path,
        description='')

    pedsim_simulator_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'simulator_launch.py')),
        launch_arguments={'scene_file': scene_file,
                          'config_file': config_file,
                          'namespace': namespace}.items())    

    frame_id_cmd = DeclareLaunchArgument(
        'frame_id', default_value='map', description='Reference frame')

    pedsim_visualizer_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('pedsim_visualizer'), 'launch', 'visualizer_launch.py')),
        launch_arguments={'frame_id': frame_id,
                          'namespace': namespace}.items())  
    
    rviz_node_cmd = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     output='screen',
                     arguments=['-d', rviz],
                     namespace=namespace,
                     condition=IfCondition(PythonExpression([use_rviz])))

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_scene_file_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(frame_id_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_rvizconfig_cmd)

    # Add any conditioned actions
    ld.add_action(pedsim_simulator_cmd)
    ld.add_action(pedsim_visualizer_cmd)
    ld.add_action(rviz_node_cmd)
    return ld
