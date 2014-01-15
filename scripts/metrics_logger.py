#!/usr/bin/env python
import roslib
roslib.load_manifest('simulator')
import rospy

import numpy as np
from pedsim_msgs.msg import AllAgentsState
from std_msgs.msg import String

import sys, os
import math


# Simple math utils for dealing with angles and such
def dotproduct(v1, v2):
    return sum((a*b) for a, b in zip(v1, v2))

def length(v):
    return math.sqrt(dotproduct(v, v))

def angle(v1, v2):
    return math.acos(dotproduct(v1, v2) / ( (length(v1) * length(v2)) + 1e-12) ) * (180 / math.pi)

def normangle(a, mina):
    if a < np.inf:
        while a >= mina + 2*np.pi:
            a = a - 2*np.pi
        while a < mina:
            a = a + 2*np.pi
        return a
    else:
        return np.inf


def diffangle(a1, a2):
    delta = 0
    if a1 < np.inf and a2 < np.inf:
        a1 = normangle(a1, 0)
        a2 = normangle(a2, 0)

        delta = a1 - a2
        if a1 > a2:
            while delta > np.pi:
                delta = delta - 2*np.pi
        elif a2 > a1:
            while delta < -np.pi:
                delta = delta + 2*np.pi
    else:
        delta = np.inf

    return delta

def agent_distance(robot, agent):
    return math.sqrt((robot[0]-agent[0])**2 + (robot[1]-agent[1])**2)

def count_agents_in_range(robot, agents, radius):
    count = 0
    rows, cols = agents.shape
    for row in range(rows):
        distance = agent_distance((robot[0, 2], robot[0, 3]), (agents[row, 2], agents[row, 3]))
        if distance <= radius:
            count = count + 1

    return count



class MetricsLogger(object):
    """ Logger for various experiment metrics which can be run only bag files """

    GOAL_REACHED = False

    def __init__(self):
        # subscribers
        rospy.Subscriber("AllAgentsStatus", AllAgentsState,
                         self.callback_agent_status)
        rospy.Subscriber("goal_status", String,
                         self.callback_goal_status)
    
    def callback_agent_status(self, data):
        step_data = np.array([0, 0, 0, 0, 0, 0], dtype=np.float64)
        for a in data.agent_states:
            v = np.array([rospy.get_rostime().to_sec(), a.id,
                          a.position.x, a.position.y,
                          a.velocity.x, a.velocity.y],
                         dtype=np.float64)

            if self.GOAL_REACHED is False:
                step_data =  np.vstack((step_data, v))
            
        robot = self._get_robot_data(step_data)
        intimate = count_agents_in_range(robot, step_data, 1.2)
        personal = count_agents_in_range(robot, step_data, 3.2)
        social = count_agents_in_range(robot, step_data, 5.2)
        public = count_agents_in_range(robot, step_data, 7.2)

        rospy.loginfo("P_i, P_p, P_s, P_k : (%d, %d, %d, %d)" % (intimate, personal, social, public))


    def callback_goal_status(self, data):
        if data.data == 'Arrived':
            self.GOAL_REACHED = True
        else:
            self.GOAL_REACHED = False


    def _get_robot_data(self, step_data):
        return step_data[step_data[:, 1] == 1]


def run(args):
    rospy.init_node('metrics_logger')
    d = MetricsLogger()

    # start up
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down metrics logging node"


if __name__ == '__main__':
    run(sys.argv)
