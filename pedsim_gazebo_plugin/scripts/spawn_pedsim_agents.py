#!/usr/bin/env python3
"""
@author: mahmoud
@mantainer: jginesclavero

"""

import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
from pedsim_msgs.msg import AgentStates
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class AgentSpawner(Node):

    def __init__(self):
        super().__init__('agent_spawner')
        qos_profile = qos_profile_sensor_data
        self.sub = self.create_subscription(AgentStates, 
                                            'pedsim_simulator/simulated_agents',
                                            self.actor_poses_callback, qos_profile)
        self.sub  # prevent unused variable warning
        pedsim_dir = get_package_share_directory('pedsim_gazebo_plugin')
        file_xml = open(pedsim_dir + "/models/person_standing/model.sdf")
        self.xml_string = file_xml.read()

        self.get_logger().info("Waiting for gazebo services...")
        self.spawn_cli = self.create_client(SpawnEntity, 'spawn_entity')

        while not self.spawn_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('service not available, waiting again...')
        self.get_logger().info("service: spawn_sdf_model is available ....")

    def actor_poses_callback(self, actors):
        req = SpawnEntity.Request()
        for actor in actors.agent_states:
            actor_id = str(actor.id)
            actor_pose = actor.pose
            # self.get_logger().info("Spawning model: actor_id = %s", actor_id)
            model_pose = Pose()
            model_pose.position.x = actor_pose.position.x
            model_pose.position.y = actor_pose.position.y
            model_pose.position.z = actor_pose.position.z

            model_pose.orientation.x = actor_pose.orientation.x
            model_pose.orientation.y = actor_pose.orientation.y
            model_pose.orientation.z = actor_pose.orientation.z
            model_pose.orientation.w = actor_pose.orientation.w
            
            req.name = actor_id
            req.xml = self.xml_string
            req.robot_namespace = ""
            req.initial_pose = model_pose
            req.reference_frame = "world"
            
            future = self.spawn_cli.call_async(req)
        self.get_logger().info("all spawned agents have been updated !")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    agent_spawner = AgentSpawner()

    rclpy.spin(agent_spawner)
    agent_spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()