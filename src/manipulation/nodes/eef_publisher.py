#!/usr/bin/env python  
import roslib
roslib.load_manifest('manipulation')
import rospy
import math
import tf

from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from std_msgs.msg import Header

if __name__ == '__main__':

	rospy.init_node('eef_publisher')
	
	listener = tf.TransformListener()
	
	eef_pub = rospy.Publisher('eef_pos/cartesian_wheelcenter', PointStamped ,queue_size=10)
	
	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		try:
			#(trans,rot) = listener.lookupTransform('/wheel_center', '/eef', rospy.Time(0))
			(trans,rot) = listener.lookupTransform('/eef', '/wheel_center', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
			
		print "Translation of EEF towards wheel base center is {}".format(trans)
		
		eef_point = Point(trans[0],trans[1],trans[2])
		eef_header = Header(float('NaN'),float('NaN'),'wheel_center')	# how to add time and really necessary?
		eef_pointstamped = PointStamped(eef_header, eef_point)
		
		eef_pub.publish(eef_pointstamped)
		
		rate.sleep()
			
		
