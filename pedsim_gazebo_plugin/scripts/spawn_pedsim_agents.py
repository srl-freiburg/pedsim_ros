#!/usr/bin/env python3
"""
@author: mahmoud
@mantainer: jginesclavero

"""

import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity, SetEntityState
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

        self.spawn_client = self.create_client(SpawnEntity, 'spawn_entity')
        self.set_state_client = self.create_client(SetEntityState, '/gazebo_spawner/set_entity_state')
        
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('service not available, waiting again...')
        # self.get_logger().info("service: spawn_sdf_model is available ....")

        max_ped = 200
        self.is_spawned = [False] * max_ped

    def actor_poses_callback(self, actors):
        for idx, actor in enumerate(actors.agent_states):
            actor_id = str(actor.id)
            actor_pose = actor.pose
            model_pose = Pose()
            model_pose.position.x = actor_pose.position.x
            model_pose.position.y = actor_pose.position.y
            model_pose.position.z = actor_pose.position.z
            model_pose.orientation.x = actor_pose.orientation.x
            model_pose.orientation.y = actor_pose.orientation.y
            model_pose.orientation.z = actor_pose.orientation.z
            model_pose.orientation.w = actor_pose.orientation.w
        
            if not self.is_spawned[idx]:
                self.spawn_entity(actor_id=actor_id, model_pose=model_pose, idx=idx)
                self.is_spawned[idx] = True
            else:
                self.set_entity_state(actor_id=actor_id, model_pose=model_pose)

    def set_entity_state(self, actor_id, model_pose):
        # Wait for the service to be available
        while not self.set_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /set_entity_state service...')
        req = SetEntityState.Request()
        req.state.name = actor_id
        req.state.pose = model_pose
        set_future = self.set_state_client.call_async(req)
        # self.get_logger().info("The spawned agent have been updated !")

    def spawn_entity(self, actor_id, model_pose, idx):
        # Wait for the service to be available
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn_entity service...')
        req = SpawnEntity.Request()
        req.name = actor_id
        req.xml = self.xml_string
        req.robot_namespace = ""
        req.initial_pose = model_pose
        req.reference_frame = "world"
        spawn_future = self.spawn_client.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    agent_spawner = AgentSpawner()

    try:
        rclpy.spin(agent_spawner)
    except KeyboardInterrupt:
        pass

    agent_spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()