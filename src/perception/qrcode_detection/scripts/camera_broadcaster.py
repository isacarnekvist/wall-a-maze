#!/usr/bin/env python
from math import pi

import roslib
roslib.load_manifest('qrcode_detection')
import rospy

import tf
from geometry_msgs.msg import Point

def camera_to_robot():
	br = tf.TransformBroadcaster()
	br.sendTransform(
        # X is how much the arm is translated forwards
        # Y is how much the arm is translated to the left when looking forward        
        # Z is how much the arm is translated upwards 
		(0.135,0.00,0.155),
		tf.transformations.quaternion_from_euler(0.0, -40.0 * pi / 180.0, 0.0),
		rospy.Time.now(),
		"wheel_center",
		"camera"
    )

if __name__ == '__main__':
	rospy.init_node('camera_broadcaster')
	rate = rospy.Rate(10) #10hz
	
	while not rospy.is_shutdown():
		camera_to_robot()
		rate.sleep()

