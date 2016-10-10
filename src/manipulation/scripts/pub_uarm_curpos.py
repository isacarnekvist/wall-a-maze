#!/usr/bin/env python

import sys
import rospy
from uarm.srv import *

from geometry_msgs.msg import Point

from uarm.msg import Coords


if __name__ == "__main__":
	pub = rospy.Publisher('/uarm_cc', Point, queue_size=10)
	rospy.init_node('uarm_pos_fromService')

	# Wait for get_position service
	rospy.wait_for_service('/uarm/get_positions')

	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		try:
			get_curPos = rospy.ServiceProxy('/uarm/get_positions', GetPositions)
			response = get_curPos()
			pub.publish(response.position)
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
		rate.sleep()
