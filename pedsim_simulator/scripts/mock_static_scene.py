#!/usr/bin/env python
from __future__ import division

import rospy
import tf

import numpy as np

from std_msgs.msg import Header
from pedsim_msgs.msg import AgentState, AgentStates
from pedsim_msgs.msg import AgentGroup, AgentGroups


def create_header():
    header = Header()
    header.frame_id = 'odom'
    header.stamp = rospy.Time.now()
    return header


def create_mock_state(track_id, x, y, angle):
    agent = AgentState()
    agent.id = track_id

    theta = np.radians(angle) + np.pi / 2.
    quaternion = tf.transformations.quaternion_from_euler(0., 0., theta)

    agent.pose.position.x = x
    agent.pose.position.y = y
    agent.pose.orientation.x = quaternion[0]
    agent.pose.orientation.y = quaternion[1]
    agent.pose.orientation.z = quaternion[2]
    agent.pose.orientation.w = quaternion[3]
    agent.twist.linear.x = np.cos(theta)
    agent.twist.linear.y = np.sin(theta)

    return agent


# All the people in the static scene.
PEOPLE = {
    1: create_mock_state(1, 5, 4, 90),
    2: create_mock_state(2, 6, 5.45878, 90),
    3: create_mock_state(3, 7.22, 5.70, 90),
    4: create_mock_state(4, 2 + 7.22, 7.33, 90),
    5: create_mock_state(5, 2 + 8.92, 8.42, 90),
    6: create_mock_state(6, 2 + 7.92, 10.41, 90),
    7: create_mock_state(7, 2 + 7.2, 9.44, 90),
    8: create_mock_state(8, 2 + 7, 14 - 2, 90),
    9: create_mock_state(9, 2 + 6, 15.4123 - 2, 90),
    10: create_mock_state(10, 5 - 1, 18.595 - 5, 280),
    11: create_mock_state(11, 5 - 1, 20 - 5, 270),
    12: create_mock_state(12, 6 - 1, 21.5491 - 5, 240),
    13: create_mock_state(13, 7.48044 - 1, 19 - 5, 90),
    14: create_mock_state(14, 6, 24.5463, 45),
    15: create_mock_state(15, 8, 28, 90),
    16: create_mock_state(16, 10.4458, 23, 68),
    17: create_mock_state(17, 11.5004, 27, 88),
    18: create_mock_state(18, 14, 25.4389, 20),
    19: create_mock_state(19, 15, 21, 90),
    20: create_mock_state(20, 15, 22.4308, 92),
    21: create_mock_state(21, 15.4676, 24, 91),
    22: create_mock_state(22, 16.5423, 25.4178, 90),
    23: create_mock_state(23, 18, 20, 90),
    24: create_mock_state(24, 18.5532, 21.5011, 90),
    25: create_mock_state(25, 15.4739, 16.5314, 45),
    26: create_mock_state(26, 20, 25.5746, 90),
    27: create_mock_state(27, 21.5327, 24, 90),
    28: create_mock_state(28, 22, 26.4632, 90),
    29: create_mock_state(29, 21, 18, 45),
    30: create_mock_state(30, 23, 20.4335, 90),
    31: create_mock_state(31, 23.4972, 21.4055, 90),
    32: create_mock_state(32, 23.4025, 22.4749, 90),
    33: create_mock_state(33, 24.5281, 18.5868, 54),
    34: create_mock_state(34, 16.554, 3.40568 - 2, 94),
    35: create_mock_state(35, 16, 6 - 1, 94),
    36: create_mock_state(36, 20, 4, 0),
    37: create_mock_state(37, 19, 12, 25),
    38: create_mock_state(38, 23, 8, 50),
    39: create_mock_state(39, 24, 10, 90),
    40: create_mock_state(40, 25, 12, 120),
    41: create_mock_state(41, 7.51, 22.41, 80),
    42: create_mock_state(42, 8.21, 25.7, 81),
    43: create_mock_state(43, 3.31, 27.7, 81),
    44: create_mock_state(44, 11.421, 18.7, 75),
    45: create_mock_state(45, 25.21, 27.0, 85),
    46: create_mock_state(46, 18.23, 6.87, -91),
    47: create_mock_state(47, 18.6, 8.90, -90),
    48: create_mock_state(48, 20.4, 7.87, 85),
    49: create_mock_state(49, 15.684, 10.74, 75),
    50: create_mock_state(50, 15.72, 14.51, 70)
}


def mock_group(g_id, members):
    """ Create a mock group with specified members """
    if not members:
        raise ValueError("Empty groups are not allowed")

    group = AgentGroup()
    group.header = create_header()
    group.group_id = g_id
    # group.age = rospy.Duration.from_sec(10.)
    x = 0.
    y = 0.
    for member in members:
        x = x + PEOPLE[member].pose.position.x
        y = y + PEOPLE[member].pose.position.y
        group.members.append(PEOPLE[member].id)

    group.center_of_mass.position.x = x / len(group.members)
    group.center_of_mass.position.y = y / len(group.members)
    return group


def main():
    mock_agents_publisher = rospy.Publisher(
        '/pedsim_simulator/simulated_agents', AgentStates, queue_size=1)
    mock_groups_publisher = rospy.Publisher(
        '/pedsim_simulator/simulated_groups', AgentGroups, queue_size=1)
    rospy.init_node('mock_static_scene')
    rate = rospy.Rate(10)
    counter = 0

    while not rospy.is_shutdown():
        # Agents
        agent_states = AgentStates()
        agent_states.header = create_header()
        agent_states.header.seq = counter

        for person in PEOPLE:
            agent_states.agent_states.append(PEOPLE[person])

        mock_agents_publisher.publish(agent_states)
        counter += 1

        # Groups
        groups = AgentGroups()
        groups.header = create_header()
        groups.groups.append(mock_group(0, [1, 2]))
        groups.groups.append(mock_group(1, [3, 4]))
        groups.groups.append(mock_group(2, [5, 6, 7]))
        groups.groups.append(mock_group(3, [8, 9]))
        groups.groups.append(mock_group(4, [10, 11, 12, 13]))
        groups.groups.append(mock_group(5, [34, 35]))
        groups.groups.append(mock_group(6, [30, 31, 32]))
        groups.groups.append(mock_group(7, [23, 24]))
        groups.groups.append(mock_group(8, [19, 20, 21, 22]))
        groups.groups.append(mock_group(9, [41, 42]))
        groups.groups.append(mock_group(10, [46, 47, 48]))
        groups.groups.append(mock_group(11, [49, 50]))

        mock_groups_publisher.publish(groups)

        rate.sleep()


if __name__ == '__main__':
    main()
