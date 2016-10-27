#!/usr/bin/env python

import sys
import rospy
import tf

from uarm.srv import *
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header

goal_pos = None


def goal_callback(data):
	global goal_pos	
	goal_pos = Point()
	
	# Transform to arm base
	#dataTrans = transform_wheelToArm(data)
	
	try:
		listener2 = tf.TransformListener()
		listener2.waitForTransform("/uarm", "/wheel_center", rospy.Time(), rospy.Duration(4.0))
		dataTrans = listener2.transformPoint('uarm',data)
	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		print "Failed to transform  point"
		return
	
	scale = 100.0 # meter to cm
	goal_pos.x = dataTrans.point.x*scale
	goal_pos.y = dataTrans.point.y*scale
	goal_pos.z = dataTrans.point.z*scale 
	

def transform_wheelToArm(data2):

	try:
		listener = tf.TransformListener()
		listener.waitForTransform("/uarm", "/wheel_center", rospy.Time(), rospy.Duration(4.0))
		data_arm = listener.transformPoint('uarm',data2)
	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		print "Failed to transform  point"
		return
		
	return data_arm

def move_to_pos_client(position, eef_orientation, move_mode, movement_duration, ignore_orientation, interpolation_type, check_limits):
	rospy.wait_for_service('/uarm/move_to')
	try:
		move_to_pos = rospy.ServiceProxy('/uarm/move_to', MoveTo)
		response = move_to_pos(position, eef_orientation, move_mode, movement_duration, ignore_orientation, interpolation_type, check_limits)
		#print "Current position is {}, {} and {}".format(response.position.x, response.position.y, response.position.z)
		return response
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e


if __name__ == "__main__":

	#rospy.init_node('objectPos_arm_listener')
	rospy.init_node('move_arm_client')
	rospy.Subscriber('/objectPos_wheelcenter', PointStamped, goal_callback)
	
	# Define move parameters 
	move_mode = 0	# (0 absolute,1 realtive)
	duration = rospy.Duration(1.0) # (in s)
	
	interpolation_type = 2 # 2 = linear interpolation
	check_limits = True
	eef_orientation = 0
	ignore_orientation = True

				
	# Resting position
	initPos_wheel = Point(20.0, 0.0,22.0)	# in wheel frame
	initPos_wheelHeader = Header()
	initPos_wheelHeader.frame_id = 'wheel_center'
	initPos_wheelStamped = PointStamped(initPos_wheelHeader, initPos_wheel)
	
	#print("Resting position", initPos_wheelStamped)
	initPos_armStamped = transform_wheelToArm(initPos_wheelStamped)
	initPos_arm = initPos_armStamped.point

	# Move to resting pose
	print("Sending it to resting position", initPos_arm)
	initial_state = move_to_pos_client(initPos_arm, eef_orientation, move_mode, duration, ignore_orientation, interpolation_type, check_limits)
	print("In resting position",initial_state)
	rospy.sleep(2.0)

	if goal_pos is None:
		#rate.sleep()
		#continue
		sys.exit()
		
	goal_pos_cur = goal_pos


	# Position above goal
	aboveGoal_pos = Point(goal_pos_cur.x, goal_pos_cur.y, initPos_arm.z)
	#print("Goal position is ", aboveGoal_pos)
			
	
	# Move above goal
	aboveGoal_state = move_to_pos_client(aboveGoal_pos, eef_orientation, move_mode, duration, ignore_orientation, interpolation_type, check_limits)
	print("Above goal", aboveGoal_state)
	rospy.sleep(2.0)
	#input("continue")
	

	# Move down
	atGoal_state = move_to_pos_client(goal_pos_cur, eef_orientation, move_mode, duration, ignore_orientation, interpolation_type, check_limits)
	print("On target", atGoalstate)
	rospy.sleep(2.0)

	# Move back to initial
	aboveGoal_state = move_to_pos_client(aboveGoal_pos, eef_orientation, move_mode, duration, ignore_orientation, interpolation_type, check_limits)
	print "Back in initial position"

	initial_state = move_to_pos_client(initPos_arm, eef_orientation, move_mode, duration, ignore_orientation, interpolation_type, check_limits)

	
	#rate.sleep()
		
	


