#!/usr/bin/env python
# Author: Timm Linder, linder@cs.uni-freiburg.de
#
# Publishes fake tracked persons and the corresponding detections (if not occluded) at
# /spencer/perception/tracked_persons and /spencer/perception/detected_persons.

import rospy, yaml, tf
from spencer_tracking_msgs.msg import TrackedPersons, TrackedPerson
from nav_msgs.msg import GridCells
from math import cos, sin, tan, pi, radians

def createTrackedPerson(track_id, x, y, theta):
    trackedPerson = TrackedPerson()

    theta = radians(theta) + pi/2.0

    trackedPerson.track_id = track_id
    quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)

    trackedPerson.pose.pose.position.x = x
    trackedPerson.pose.pose.position.y = y

    trackedPerson.pose.pose.orientation.x = quaternion[0]
    trackedPerson.pose.pose.orientation.y = quaternion[1]
    trackedPerson.pose.pose.orientation.z = quaternion[2]
    trackedPerson.pose.pose.orientation.w = quaternion[3]

    trackedPerson.pose.covariance[0 + 0 * 6] = 0.001 # x
    trackedPerson.pose.covariance[1 + 1 * 6] = 0.001  # y
    trackedPerson.pose.covariance[2 + 2 * 6] = 999999 # z
    trackedPerson.pose.covariance[3 + 3 * 6] = 999999 # x rotation
    trackedPerson.pose.covariance[4 + 5 * 6] = 999999 # y rotation
    trackedPerson.pose.covariance[4 + 5 * 6] = 999999 # z rotation

    trackedPerson.twist.twist.linear.x = cos(theta)
    trackedPerson.twist.twist.linear.y = sin(theta)

    for i in range(0, 3):
        trackedPerson.twist.covariance[i + i * 6] = 1.0 # linear velocity
    for i in range(3, 6):
        trackedPerson.twist.covariance[i + i * 6] = float("inf") # rotational velocity

    return trackedPerson

def main():
    # Main code
    trackPublisher = rospy.Publisher('/spencer/perception/tracked_persons', TrackedPersons )
    #obstaclesPublisher = rospy.Publisher('/pedsim/static_obstacles', GridCells )

    rospy.init_node( 'mock_tracked_persons' )
    rate = rospy.Rate(10)

    #obstacles = yaml.load(OBSTACLE_YAML)
    #obstacles = [ d for d in obstacles]

    seqCounter = 0
    while not rospy.is_shutdown():

        trackedPersons = TrackedPersons()
        trackedPersons.header.seq = seqCounter
        trackedPersons.header.frame_id = "odom"
        trackedPersons.header.stamp = rospy.Time.now()

        #trackedPersons.tracks.append( createTrackedPerson( trackId, x, y, theta ) )

        trackedPersons.tracks.append( createTrackedPerson( 1,  2, 5, 270 ) )
        trackedPersons.tracks.append( createTrackedPerson( 2,  5, 3.5, 109 ) )
        trackedPersons.tracks.append( createTrackedPerson( 3,  6, 5, 90 ) )
        trackedPersons.tracks.append( createTrackedPerson( 4,  5, 7.2, 109 ) )
        trackedPersons.tracks.append( createTrackedPerson( 5,  9.2, 1.2, 71.56-90 ) )
        trackedPersons.tracks.append( createTrackedPerson( 6,  7.1, 2.5,  80.9097 -90) )
        trackedPersons.tracks.append( createTrackedPerson( 7,  8.2, 7.6,   8) )
        trackedPersons.tracks.append( createTrackedPerson( 8,  7.1, 6.5, 10) )
        trackedPersons.tracks.append( createTrackedPerson( 9,  2.2, 1.8, 85.2364 -90 ) )
        trackedPersons.tracks.append( createTrackedPerson( 10, 4.1, 1.9, 93.8141 -90 ) )
        trackedPersons.tracks.append( createTrackedPerson( 11, 1.7, 9.3, 78.6901 -90 ) )
        trackedPersons.tracks.append( createTrackedPerson( 12, 2.2, 7.5, 63.4349 -90) )

        trackPublisher.publish( trackedPersons )

        #obstacles['header'] = trackedPersons.header
        #obstaclesPublisher.publish( obstacles )

        seqCounter += 1
        rate.sleep()


if __name__ == '__main__':
    main()
