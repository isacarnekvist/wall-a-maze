#!/usr/bin/env python

import sys
import rospy
import tf

from uarm.srv import *
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header



class Manipulation():

	def __init__(self):
		rospy.init_node('move_arm_client')
	
		rospy.Subscriber('/objectPos_wheelcenter', PointStamped, self.goal_callback)
	
		# Wait for transform
		self.listener = tf.TransformListener()
		self.listener.waitForTransform("wheel_center", "uarm", rospy.Time(), rospy.Duration(4.0))
	
		# Wait for move service
		rospy.wait_for_service('/uarm/move_to')
		self.moveTo_service = rospy.ServiceProxy('/uarm/move_to', MoveTo)
	
		self.goalPosStamped = Point()
	
		# Define move parameters 
		self.move_mode = 0	# (0 absolute,1 realtive)
		self.movement_duration = rospy.Duration(1.0) # (in s)
	
		self.interpolation_type = 2 # 2 = linear interpolation
		self.check_limits = True
		self.eef_orientation = 0
		self.ignore_orientation = True

		self.moveSteps()

		rospy.spin()
	
	
	
	def goal_callback(self, data):
		self.goalPosStamped = self.transform_wheelToArm(data)
	
		print("Point before scaling:",self.goalPosStamed.point)
		scale = 100.0 # meter to cm
		self.goalPosStamped.point.x = self.goalPosStamped.point.x*scale	# CHECK IF THIS WORKS
		self.goalPosStamped.point.y = self.goalPosStamped.point.y*scale
		self.goalPosStamped.point.z = self.goalPosStamped.point.z*scale
		print("Point after scaling:",self.goalPosStamed.point)
		

	def transform_wheelToArm(self, data):
		try:	
			data_arm = self.listener.transformPoint('uarm',data)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			print "Failed to transform  point"
			return
		
		return data_arm



	def moveToPos_client(self, position):
		try:
			response = self.moveTo_service(position, self.eef_orientation, self.move_mode, self.movement_duration, self.ignore_orientation, self.interpolation_type, self.check_limits)
			return response
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
			
	
	def moveSteps(self):
		# Resting position
		initPos_arm = Point(2.3,11.8,12.6)	# in arm frame
		
	
		initial_state = self.moveToPos_client(initPos_arm) # currently to initPos_wheel
		print("In resting position",initial_state)
		rospy.sleep(2.0)
		

		
		
		testPos_Stamped = PointStamped()
		testPos_Stamped.header.frame_id = 'wheel_center'
		testPos_Stamped.point = Point(0.10,0.00,0.25)
		scale = 100.0 # meter to cm
		testPos_Stamped.point.x = testPos_Stamped.point.x*scale
		testPos_Stamped.point.y = testPos_Stamped.point.y*scale
		testPos_Stamped.point.z = testPos_Stamped.point.z*scale
		
		
		testPos_uarm = self.transform_wheelToArm(testPos_Stamped)
		
		print("Sending arm to position ... wrt armframe" , testPos_uarm)
		test_state = self.moveToPos_client(testPos_uarm.point)
		print("Arm is at", test_state)
		
		sys.exit()
		#if goal_pos is None:
			#rate.sleep()
			#continue
		
		goal_pos_cur = self.goalPosStamped.point

		# Position above goal
		aboveGoal_pos = Point(goal_pos_cur.x, goal_pos_cur.y, initPos_arm.z)
		#print("Goal position is ", aboveGoal_pos)
			
	
		# Move above goal
		aboveGoal_state = self.moveToPos_client(aboveGoal_pos)
		print("Above goal", aboveGoal_state)
		rospy.sleep(2.0)
		#input("continue")
	

		# Move down
		atGoal_state = self.moveToPos_client(goal_pos_cur)
		print("On target", atGoalstate)
		rospy.sleep(2.0)

		# Move back to initial
		aboveGoal_state = self.moveToPos_clientt(aboveGoal_pos)
		print "Back in initial position"

		initial_state = self.moveToPos_client(initPos_arm)

	


if __name__ == "__main__":
	Manipulation()
	
	
	

