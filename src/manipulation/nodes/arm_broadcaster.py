#!/usr/bin/env python
from math import pi

import roslib
roslib.load_manifest('manipulation')
import rospy

import tf
from geometry_msgs.msg import Point

def robot_to_uarm():
	br = tf.TransformBroadcaster()
	br.sendTransform(
        # Y is how much the arm is translated to the left when looking forward
        # X is how much the arm is translated forwards
        # Z is how much the arm is translated upwards 
        # OLD VALUES for old uarm python, ARM TILTED NOW 90 degree anti clockwise (-0.105,0.086,-0.051), #0.0), #0.051
        # NEW VALUES from guessing and assuming base fram uarm at center of ground rotation and height minus 2cm 
		(-0.01,0.01,0.11),
        # OLD tf.transformations.quaternion_from_euler(0, 0, 4.5 * pi / 180.0),
		#tf.transformations.quaternion_from_euler(0, 0, -85.5 * pi / 180.0),
		tf.transformations.quaternion_from_euler(0, 0, -96.0 * pi / 180.0),
		rospy.Time.now(),
		"uarm",
		"wheel_center"
    )

if __name__ == '__main__':
	rospy.init_node('arm_broadcaster')
	rate = rospy.Rate(2) #10hz
	
	while not rospy.is_shutdown():
		robot_to_uarm()
		rate.sleep()

