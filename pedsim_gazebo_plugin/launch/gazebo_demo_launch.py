# Copyright (c) 2018 Intel Corporation
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
#!/usr/bin/env python3

"""Test gazebo_plugins for pedsim."""

import os
import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get the launch directory
    pedsim_dir = FindPackageShare(package='pedsim_gazebo_plugin').find('pedsim_gazebo_plugin')
    bringup_dir = FindPackageShare(package='nav2_bringup').find('nav2_bringup')
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
    urdf_model_path = os.path.join(bringup_dir, 'urdf', 'turtlebot3_waffle.urdf')
    sdf_model_path = os.path.join(bringup_dir, 'worlds', 'waffle.model')
    world_model_path = os.path.join(pedsim_dir, 'worlds', 'office-cubicles.world')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Launch configuration variables specific to simulation
    use_simulator = LaunchConfiguration('use_simulator')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    urdf_model = LaunchConfiguration('urdf_model')
    robot_sdf = LaunchConfiguration('robot_sdf')

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='Whether to start the simulator')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='False',
        description='Whether to start the robot state publisher')

    declare_simulator_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Whether to execute gzclient)')

    declare_robot_sdf_cmd = DeclareLaunchArgument(
        'robot_sdf',
        default_value=sdf_model_path,
        description='Full path to robot sdf file to spawn the robot in gazebo')

    declare_urdf_cmd = DeclareLaunchArgument(
        name='urdf_model',
        default_value=urdf_model_path,
        description='Absolute path to robot urdf file')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=world_model_path,
        description='Full path to world model file to load')

    # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
            condition=IfCondition(use_simulator),
            launch_arguments={'world': world}.items())
    # Start Gazebo client
    start_gazebo_client_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
            condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))

    robot_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', 'robot_test',
            '-file', robot_sdf,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0',
        ],
    )

    agent_spawner_cmd = Node(
        package='pedsim_gazebo_plugin',
        executable='spawn_pedsim_agents.py',
        name='spawn_pedsim_agents',
        output='screen')

    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher', 
        executable='robot_state_publisher',
        namespace=namespace, 
        parameters=[{'use_sim_time': use_sim_time,
                    'robot_description': Command(['xacro ', urdf_model])}], 
        remappings=remappings,
        arguments=[urdf_model])

   # robot join state publisher node
    start_joint_state_publisher_cmd = Node(
            condition=IfCondition(use_robot_state_pub),
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            arguments=[urdf_model],
            parameters=[{'source_list': ['joint_states']},{'use_gui': 'true'}]
           )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_robot_sdf_cmd)
    ld.add_action(declare_urdf_cmd)
    ld.add_action(declare_world_cmd)

    # Add any conditioned actions
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    # ld.add_action(robot_spawner_cmd)
    ld.add_action(agent_spawner_cmd)
    # ld.add_action(start_robot_state_publisher_cmd)
    # ld.add_action(start_joint_state_publisher_cmd)

    return ld
