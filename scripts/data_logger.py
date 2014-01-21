#!/usr/bin/env python
import roslib
roslib.load_manifest('simulator')
import rospy

import numpy as np
from pedsim_msgs.msg import AllAgentsState
from std_msgs.msg import String

import sys, os


class DataLogger(object):

    """ Simple data logging for use with learning """

    GOAL_REACHED = False

    def __init__(self):
        # subscribers
        rospy.Subscriber("AllAgentsStatus", AllAgentsState,
                         self.callback_agent_status)
        rospy.Subscriber("goal_status", String,
                         self.callback_goal_status)

        # data store (time, agentid, x, y, vx, vy)
        self.agent_trace = np.array([0, 0, 0, 0, 0, 0], dtype=np.float64)

    def callback_agent_status(self, data):
        for a in data.agent_states:
            v = np.array([rospy.get_rostime().to_sec(), a.id,
                          a.position.x, a.position.y,
                          a.velocity.x, a.velocity.y],
                         dtype=np.float64)

            if self.GOAL_REACHED is False:
                self.agent_trace =  np.vstack((self.agent_trace, v))

    def callback_goal_status(self, data):
        if data.data == 'Arrived':
            self.GOAL_REACHED = True
            print self.agent_trace.shape
            cwd = os.getcwd()
            np.savetxt(cwd+'/traj.txt', self.agent_trace)
            rospy.loginfo('Saved robot trajectory to %s' % (cwd+'/traj.txt'))
        else:
            self.GOAL_REACHED = False

    def filter_out_robot(self, traces, robot_id):
        return traces[traces[:, 1] == robot_id]


def run(args):
    rospy.init_node('data_logger')
    DataLogger()

    # start up
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down trajectory data logger node"


if __name__ == '__main__':
    run(sys.argv)
