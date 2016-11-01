#!/usr/bin/env python  
import roslib
roslib.load_manifest('manipulation')
import rospy
import math
import tf

from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Header


if __name__ == '__main__':

	rospy.init_node('eef_publisher')
	
	listener = tf.TransformListener()
	
	eef_pub = rospy.Publisher('eef_pos/cartesian_wheelcenter', PointStamped ,queue_size=10)
	
	eef_stamped = PointStamped()
	
	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		try:
			(trans,rot) = listener.lookupTransform('wheel_center', 'eef', rospy.Time(0))
			#(trans,rot) = listener.lookupTransform('/eef', '/wheel_center', rospy.Time(0))
			(trans_uarm,rot_uarm) = listener.lookupTransform('uarm', 'eef', rospy.Time(0))
			
			eef_stamped.header.frame_id = 'uarm'
			eef_stamped.point = Point(trans_uarm[0],trans_uarm[1],trans_uarm[2])
			eef_wheel = listener.transformPoint('wheel_center',eef_stamped)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
			
		testPoint_stamped = PointStamped()
		testPoint_stamped.header.frame_id = 'wheel_center'
		testPoint_stamped.point = Point(trans[0],trans[1],trans[2])
		testPoint_uarm = listener.transformPoint('uarm',testPoint_stamped)
			
		#print "Translation of EEF towards wheel base center is {}".format(trans)
		#print "EEF transformed to wheel base is {}".format(eef_wheel.point)
		print " "
		print "EEF_uarm = {}".format(trans_uarm)
		print "testPoint_uarm = {}".format(testPoint_uarm.point)
		eef_point = Point(trans[0],trans[1],trans[2])
		eef_header = Header(float('NaN'),float('NaN'),'wheel_center')	# how to add time and really necessary?
		eef_pointstamped = PointStamped(eef_header, eef_point)
		
		eef_pub.publish(eef_pointstamped)
		
		rate.sleep()
			
		
