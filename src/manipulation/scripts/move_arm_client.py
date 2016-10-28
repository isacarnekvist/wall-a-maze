#!/usr/bin/env python

import sys
import rospy

from uarm.srv import *
from geometry_msgs.msg import Point
from uarm.msg import Coords

goal_pos = None


def goal_callback(data):
	global goal_pos	
	goal_pos = Coords()
	print("Input data", data)
	scale = 100.0 # meter to cm
	goal_pos.x = data.x*scale + 5.0
	goal_pos.y = data.y*scale - 2.0
	goal_pos.z = data.z*scale - 8.0
	
	



def move_to_pos_client(position, move_mode, duration, path, ease):
	rospy.wait_for_service('/uarm/move_to')
	try:
		move_to_pos = rospy.ServiceProxy('/uarm/move_to', MoveTo)
		response = move_to_pos(position, move_mode, duration, path, ease)
		print "Current position is {}, {} and {}".format(response.position.x, response.position.y, response.position.z)
		return response
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e


if __name__ == "__main__":

	rospy.init_node('objectPos_arm_listener')
	rospy.Subscriber('/objectPos_uarm', Point, goal_callback)

	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
	
		if goal_pos is None:
			rate.sleep()
			continue
			
		# Resting position
		init_pos = Coords(8, 7, 22)
		goal_pos_cur = goal_pos

		# Position above goal
		aboveGoal_pos = Coords(goal_pos_cur.x, goal_pos_cur.y, 22)
		print("Goal position is ", aboveGoal_pos)

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
		#input("continue")
		
	
		# Move down
		atGoal_state = move_to_pos_client(goal_pos_cur, move_mode, duration, path_type, ease_type)
	
		# Move back to initial
		aboveGoal_state = move_to_pos_client(aboveGoal_pos, move_mode, duration, path_type, ease_type)

		initial_state = move_to_pos_client(init_pos, move_mode, duration, path_type, ease_type)
	
		
		rate.sleep()
		
	


