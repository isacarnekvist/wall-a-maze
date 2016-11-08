#!/usr/bin/env python 

import roslib
import rospy
import math 
import numpy as np
import tf

roslib.load_manifest('manipulation')

from geometry_msgs.msg import PointStamped

global objectPos_wheel

def object_callback(data):
	objectPos_wheel = data
	

if __name__ == '__main__':
	rospy.init_node('tf_wheelToArm')
	
	rospy.Subscriber('/objectPos_wheelcenter', PointStamped, object_callback)
	objectPos_arm_pub = rospy.Publisher('/objectPos_uarm', PointStamped,queue_size=10)

	listener = tf.TransformListener()


	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():

		objectPos_arm = listener.transformPoint('uarm',data)	
		objectPos_arm_pub.publish(objectPos_arm)
			
		rate.sleep()
		
	rospy.spin()
