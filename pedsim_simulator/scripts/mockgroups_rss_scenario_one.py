#!/usr/bin/env python

__author__ = "Luigi Palmieri"
__copyright__ = "Social Robotics Lab, University of Freiburg"
__license__ = "BSD"
__version__ = "0.0.1"
__email__ = "palmieri@informatik.uni-freiburg.de"

import roslib
import time
import math
import numpy as np
import rospy
import tf
import sys
import itertools
import os
from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovariance
from nav_msgs.msg import GridCells
from spencer_tracking_msgs.msg import TrackedPersons
from spencer_tracking_msgs.msg import TrackedPerson
from spencer_tracking_msgs.msg import TrackedGroup
from spencer_tracking_msgs.msg import TrackedGroups


def groups_sender():
    global pub_groups
    global listener
    global group_id

    pub_groups = rospy.Publisher(
        '/spencer/perception/tracked_groups', TrackedGroups, queue_size=1)
    sub_agents_poses = rospy.Subscriber(
        '/spencer/perception/tracked_persons', TrackedPersons, ReadAgents, queue_size=1)
    listener = tf.TransformListener()
    r = rospy.Rate(10)  # 10hz

    readagents = 0
    while not rospy.is_shutdown():
        # rospy.loginfo("#Sending Groups")
        r.sleep()

# Reading the Agents, associate them to a single group, and send the groups msgs
# to use only for a toy example where only a single group exists


def ReadAgents(arg):
    global listener
    global groups
    global group_id

    alltrack_ids = [tracked_person.track_id for tracked_person in arg.tracks]
    # rospy.logwarn(str(alltrack_ids))
    # createGroup(arg.tracks,alltrack_ids)
    groups = TrackedGroups()
    groups.header.frame_id = "odom"
    groups.header.stamp = rospy.Time.now()

    group_id = 0

    createGroup(arg.tracks, [1, 2])
    createGroup(arg.tracks, [3, 4])

    createGroup(arg.tracks, [5, 6, 7])

    createGroup(arg.tracks, [8, 9])
    createGroup(arg.tracks, [10, 11, 12, 13])
    createGroup(arg.tracks, [34, 35])
    createGroup(arg.tracks, [30, 31, 32])
    createGroup(arg.tracks, [23, 24])
    createGroup(arg.tracks, [19, 20, 21, 22])
    createGroup(arg.tracks, [41, 42])
    createGroup(arg.tracks, [46, 47, 48])
    createGroup(arg.tracks, [49, 50])

    pub_groups.publish(groups)


def createGroup(allTracks, tracksInGroup):
    global pub_groups
    global group_id
    global groups

    group = TrackedGroup()
    group.group_id = group_id
    group.age = rospy.Duration.from_sec(10)
    x = 0
    y = 0
    nagents = 0
    for tracked_person in allTracks:
        if(tracked_person.track_id in tracksInGroup):
            quat = (tracked_person.pose.pose.orientation.x, tracked_person.pose.pose.orientation.y,
                    tracked_person.pose.pose.orientation.z, tracked_person.pose.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quat)
            tracked_person_theta = euler[2]
            x = x + tracked_person.pose.pose.position.x
            y = y + tracked_person.pose.pose.position.y
            nagents = nagents + 1
            group.track_ids.append(tracked_person.track_id)

    group.centerOfGravity.pose.position.x = x / nagents
    group.centerOfGravity.pose.position.y = y / nagents
    groups.groups.append(group)

    group_id += 1


if __name__ == '__main__':
    rospy.init_node('mockgroups_rss_scenario_one')
    try:
        groups_sender()
    except rospy.ROSInterruptException:
        pass
