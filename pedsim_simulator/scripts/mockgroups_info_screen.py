#!/usr/bin/env python

import rospy
import tf

from spencer_tracking_msgs.msg import TrackedPersons
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

# Reading the Agents, associate them to a single group, and send
# the groups msgs to use only for a toy example where only a
# single group exists


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

    # createGroup(arg.tracks, [1, 2, 3, 4])

    createGroup(arg.tracks, [1, 2])
    createGroup(arg.tracks, [1, 3])
    createGroup(arg.tracks, [1, 4])
    createGroup(arg.tracks, [1, 5])

    # createGroup(arg.tracks, [5, 6])
    # createGroup(arg.tracks, [7, 8])
    # createGroup(arg.tracks, [9, 10])
    # createGroup(arg.tracks, [11, 12])
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
    rospy.init_node('mockgroups_info_screen')
    try:
        groups_sender()
    except rospy.ROSInterruptException:
        pass
