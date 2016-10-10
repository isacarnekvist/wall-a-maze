#!/usr/bin/env python
from math import pi

import roslib
roslib.load_manifest('localization')
import rospy

import tf
from sensor_msgs.msg import LaserScan

def robot_to_laser(scan):
    br = tf.TransformBroadcaster()
    br.sendTransform(
        # Y is how much the laser is translated to the right
        # X is how much the laser is translated forwards
        (0.08, -0.009, 0.0),
        tf.transformations.quaternion_from_euler(0, 0, -91.5 * pi / 180.0),
        rospy.Time.now(),
        "laser",
        "wheel_center"
    )

if __name__ == '__main__':
    rospy.init_node('laser_broadcaster')
    rospy.Subscriber(
        '/scan',
        LaserScan,
        robot_to_laser
    )
    rospy.spin()
