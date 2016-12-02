#!/usr/bin/env python
from math import pi

import roslib
roslib.load_manifest('qrcode_detection')
import rospy

import tf
from geometry_msgs.msg import Point

def wheel_to_camera():
	br = tf.TransformBroadcaster()
	br.sendTransform(
        # X is how much the arm is translated forwards
        # Y is how much the arm is translated to the left when looking forward        
        # Z is how much the arm is translated upwards 
		(0.115,0.00,0.160),
		tf.transformations.quaternion_from_euler(0.0, 125.0 * pi / 180.0, -93.0 * pi /180.0,axes='rxyz'),
		rospy.Time.now(),
		"camera",
		"wheel_center"
    )

if __name__ == '__main__':
	rospy.init_node('camera_broadcaster')
	rate = rospy.Rate(10) #10hz
	
	while not rospy.is_shutdown():
		wheel_to_camera()
		rate.sleep()

