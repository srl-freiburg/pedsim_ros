#!/usr/bin/env python
import roslib
roslib.load_manifest('pedsim_simulator')
import rospy

import tf


if __name__ == '__main__':
    rospy.init_node('simulator_tf_broadcaster')

    br = tf.TransformBroadcaster()
    rate = rospy.Rate(20.0)
    while not rospy.is_shutdown():
        br.sendTransform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), "world", "pedsim_base")
        br.sendTransform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), "odom", "world")
        rate.sleep()
