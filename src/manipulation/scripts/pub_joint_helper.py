#!/usr/bin/env python

import sys
import rospy
from uarm.srv import *

from geometry_msgs.msg import Point

from uarm.msg import Coords


if __name__ == "__main__":
	# Position which shall be converted

	goalPos = Point(20.0,0.0,0.0)
	eef_orientation = 0.0
	check_limits = True

	# Wait for get_position service
	rospy.wait_for_service('/uarm/convert_to_joints')

	try:
		convert_to_joints = rospy.ServiceProxy('/uarm/convert_to_joints', ConvertToJoints)
		response = convert_to_joints(goalPos, eef_orientation, check_limits)
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e
	
	print "Configuration for {} is {}".format(goalPos, response)
