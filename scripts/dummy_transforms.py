#!/usr/bin/env python  
import roslib
roslib.load_manifest('simulator')
import rospy

import tf


if __name__ == '__main__':
    rospy.init_node('simulator_tf_broadcaster')

    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((0.0, 0.0, 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "world",
                         "pedsim_base")
        rate.sleep()