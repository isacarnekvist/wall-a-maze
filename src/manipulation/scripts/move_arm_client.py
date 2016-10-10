#!/usr/bin/env python

import sys
import rospy
from uarm.srv import *

from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import UInt8
from std_msgs.msg import Int32

from geometry_msgs.msg import Point

from uarm.msg import Angles
from uarm.msg import Coords
from uarm.msg import CoordsWithTime
from uarm.msg import CoordsWithTS4

cur_pos = Point(0,0,0)
#old_pos = Point(0,0,0)

def move_to_pos_client(position, move_mode, duration, path, ease):
	global cur_pos
	rospy.wait_for_service('/uarm/move_to')
	try:
		move_to_pos = rospy.ServiceProxy('/uarm/move_to', MoveTo)
		response = move_to_pos(position, move_mode, duration, path, ease)
		print "Current position is {}, {} and {}".format(response.position.x, response.position.y, response.position.z)
		cur_pos = response.position
		return response
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def pub_curPos():
	global cur_pos
	pub = rospy.Publisher('/uarm_cc', Point, queue_size=10)
	rospy.init_node('uarm_cc_aftermove')

	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		pub.publish(cur_pos)
		rate.sleep()
		


if __name__ == "__main__":
	
	# Start publisher
	#pub_curPos()

	# Resting position
	init_pos = Coords(8, 7, 22)
	
	# Define goal posiion
	goal_pos = Coords()
	goal_pos.x = 16
	goal_pos.y = 15
	goal_pos.z = -2

	# Position above goal
	aboveGoal_pos = Coords(goal_pos.x, goal_pos.y, 22)

	# Define move mode (0 absolute,1 realtive)
	move_mode = 0

	# Define duration (in s)
	duration = rospy.Duration(2)
	path_type = 0
	ease_type = 0

	# Move to resting pose
	initial_state = move_to_pos_client(init_pos, move_mode, duration, path_type, ease_type)

	# Move above goal
	aboveGoal_state = move_to_pos_client(aboveGoal_pos, move_mode, duration, path_type, ease_type)

	# Move down
	if aboveGoal_state.position.x > 11:
		atGoal_state = move_to_pos_client(goal_pos, move_mode, duration, path_type, ease_type)
	else:
		print "Do not move down as x smaller 11"

	# Move back to initial
	aboveGoal_state = move_to_pos_client(aboveGoal_pos, move_mode, duration, path_type, ease_type)
	initial_state = move_to_pos_client(init_pos, move_mode, duration, path_type, ease_type)
	
	


