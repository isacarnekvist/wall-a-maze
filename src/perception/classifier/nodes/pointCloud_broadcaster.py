#!/usr/bin/env python
from math import pi

import roslib
roslib.load_manifest('classifier')
import rospy

import tf
from sensor_msgs.msg import PointCloud2

def robot_to_pointCloud(cloud):
    br = tf.TransformBroadcaster()
    br.sendTransform(
        # Y is how much the laser is translated to the right
        # X is how much the laser is translated forwards
        #(0.115, 0.0, 0.202),
        #tf.transformations.quaternion_from_euler(0.0, 30.0 * pi / 180.0, 0.0),
        (0.0, 0.0, 0.0),
        tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0),
        rospy.Time.now(),
        "camera_link",
        "wheel_center"
    )

if __name__ == '__main__':
    rospy.init_node('perception_broadcaster')
    rospy.Subscriber(
        '/camera/depth_registered/points',
        PointCloud2,
        robot_to_pointCloud
    )
    rospy.spin()
