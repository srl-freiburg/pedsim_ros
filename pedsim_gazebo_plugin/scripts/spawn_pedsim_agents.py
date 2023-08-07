#!/usr/bin/env python3
"""
@mantainer: stephenadhi

"""
import functools
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

            if not self.is_spawned[idx]:
                spawn_future = self.spawn_entity(actor_id=actor_id, model_pose=actor.pose)
                if spawn_future:
                    callback = functools.partial(self.callback, actor_id, actor.pose, idx)
                    spawn_future.add_done_callback(callback)
            else:
                self.set_entity_state(actor_id=actor_id, model_pose=actor.pose)

    def callback(self, actor_id, model_pose, idx, future):
        if future.exception() is None:
            self.is_spawned[idx] = True
            self.set_entity_state(actor_id=actor_id, model_pose=model_pose)

    def spawn_entity(self, actor_id, model_pose):
        try:
            req = SpawnEntity.Request()
            req.name = actor_id
            req.xml = self.xml_string
            req.robot_namespace = ""
            req.initial_pose = model_pose
            req.reference_frame = "world"
            self.spawn_client.call_async(req)
            return self.spawn_client.call_async(req)  # return the Future object
        except Exception as e:
            self.get_logger().warn(f"Failed to spawn entity: {e}")
            return None

    def set_entity_state(self, actor_id, model_pose):
        try:
            req = SetEntityState.Request()
            req.state.name = actor_id
            req.state.pose = model_pose
            self.set_state_client.call_async(req)
        except Exception as e:
            self.get_logger().warn(f"Failed to update agent state: {e}")

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