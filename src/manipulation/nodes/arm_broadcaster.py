#!/usr/bin/env python
from math import pi

import roslib
roslib.load_manifest('manipulation')
import rospy

import tf
from geometry_msgs.msg import Point

def robot_to_uarm(uarm_cc):
    br = tf.TransformBroadcaster()
    br.sendTransform(
        # Y is how much the laser is translated to the right
        # X is how much the laser is translated forwards
        (-0.105,0.086,0.051),
        tf.transformations.quaternion_from_euler(0, 0, 4.5 * pi / 180.0),
        rospy.Time.now(),
        "uarm",
        "wheel_center"
    )

if __name__ == '__main__':
    rospy.init_node('arm_broadcaster')
    rospy.Subscriber(
        '/uarm_cc',
        Point,
        robot_to_uarm
    )
    rospy.spin()
