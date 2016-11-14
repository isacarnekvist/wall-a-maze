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
	point = Point(0.20,-0.05,0.03) # in m
	msg.pickupPos = PointStamped(header,point)
	
	msg.job = 'reposition'
	
	placePoint = Point(0.20,0.05,0.03)
	msg.placePos = PointStamped(header,placePoint)
	
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		pub.publish(msg)
		rate.sleep()


if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass	
