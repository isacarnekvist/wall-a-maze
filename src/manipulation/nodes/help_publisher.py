#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Header


def talker():
	pub = rospy.Publisher('/objectPos_wheelcenter', PointStamped, queue_size=10)
	rospy.init_node('talker', anonymous=True)
	
	# Publish some pointStamped
	header = Header()
	header.frame_id = 'wheel_center'
	
	point = Point(0.20,0.01,0.05) # in m
	
	pointStamped = PointStamped(header,point)
	
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		pub.publish(pointStamped)
		rate.sleep()


if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass	
