#!/usr/bin/env python3
"""
@author: mahmoud
@mantainer: jginesclavero

"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from gazebo_msgs.srv import SpawnEntity
from ament_index_python.packages import get_package_share_directory
from pedsim_msgs.msg  import AgentStates
from geometry_msgs.msg import Pose, Point, Quaternion
class AgentSpawner(Node):

    def __init__(self):
        super().__init__('agent_spawner')
        qos_profile = qos_profile_sensor_data
        
        pedsim_dir = get_package_share_directory('pedsim_gazebo_plugin')
        file_xml = open(pedsim_dir + "/models/person_standing/model.sdf")
        self.xml_string = file_xml.read()
        self.get_logger().info("Waiting for gazebo services...")
        self.spawn_cli = self.create_client(SpawnEntity, 'spawn_entity')

        while not self.spawn_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('service not available, waiting again...')
        self.get_logger().info("service: spawn_sdf_model is available ....")

    def spawn(self):
        req = SpawnEntity.Request()
        actor_id = "3"
        print("Spawning single model: actor_id = %s", actor_id)
        model_pose = Pose()
        model_pose.position.x = 2.4
        model_pose.position.y = -3.0
        model_pose.position.z = 0.0

        model_pose.orientation.x = 0.0
        model_pose.orientation.y = 0.0
        model_pose.orientation.z = -0.707
        model_pose.orientation.w = -0.707
            
        req.name = actor_id
        req.xml = self.xml_string
        req.robot_namespace = ""
        req.initial_pose = model_pose
        req.reference_frame = "world"
        
        future = self.spawn_cli.call_async(req)

        print("agent has been spawned !")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    agent_spawner = AgentSpawner()
    agent_spawner.spawn()
    # rclpy.spin(agent_spawner)
    agent_spawner.destroy_node()
    # rclpy.shutdown()

if __name__ == '__main__':
    main()