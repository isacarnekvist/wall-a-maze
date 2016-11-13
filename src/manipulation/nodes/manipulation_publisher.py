#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Header
from manipulation.msg import Manipulation

def talker():
	pub = rospy.Publisher('/mother/manipulation', Manipulation, queue_size=10)
	rospy.init_node('talker', anonymous=True)
	
	# Publish some Manipulation message
	msg = Manipulation()
	
	header = Header()
	header.frame_id = 'wheel_center'
	point = Point(0.25,0.00,0.04) # in m
	msg.pickupPos = PointStamped(header,point)
	
	msg.job = 'reposition'
	
	msg.placePos = msg.pickupPos	
	
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		pub.publish(msg)
		rate.sleep()


if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass	
