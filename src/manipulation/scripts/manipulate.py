#!/usr/bin/env python

import sys
import rospy
import tf
import numpy as np

from uarm.srv import *
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header, String
from manipulation.msg import Manipulation



class Manipulate():

	def __init__(self):
		rospy.init_node('move_arm_client')
	
		rospy.Subscriber("/objectPos_wheelcenter", PointStamped, self.goal_callback)
		rospy.Subscriber("/mother/manipulation", Manipulation, self.mother_callback)
		rospy.Subscriber("/uarm/joint_state", JointState , self.joint_callback)
		 
		# Wait for transform
		self.listener = tf.TransformListener()
		self.listener.waitForTransform("wheel_center", "uarm", rospy.Time(), rospy.Duration(8.0))
	
		# Wait for move, move_joints, pump and inverse kinematics service
		rospy.wait_for_service('/uarm/move_to')
		self.moveTo_service = rospy.ServiceProxy('/uarm/move_to', MoveTo)
		
		rospy.wait_for_service('/uarm/move_to_joints')
		self.moveToJoints_service = rospy.ServiceProxy('/uarm/move_to_joints', MoveToJoints)
	
		rospy.wait_for_service('/uarm/pump')
		self.pump_service = rospy.ServiceProxy('/uarm/pump',Pump)
		
		rospy.wait_for_service('/uarm/convert_to_joints')
		self.joint_service = rospy.ServiceProxy('/uarm/convert_to_joints', ConvertToJoints)
		
		#-- Variables --#
		
		# Goal positions
		self.pickupPos_arm = Point()
		self.placePos_arm = Point()
		self.pickupPos_wheel = Point()
		self.placePos_wheel = Point()
		self.job = None
		self.drop = False

		# Other
		self.inOperation = False
		
		
		# EEF position
		self.eefJointPos_current = JointState()
		
		# Define move parameters 
		self.move_mode = 0	# (0 absolute,1 realtive)
		self.moveDuration_abs = rospy.Duration(2.0) # (in s)
		self.moveDuration_rel = rospy.Duration(0.2)
		
		self.interpol_none = 0
		self.interpol_cubic = 1
		self.interpol_linear = 2
		self.check_limits = True
		self.eef_orientation = 0
		self.ignore_orientation = True
		
		self.j0Tol = 0.5	# degrees
		self.j1Tol = 1.0
		self.j2Tol = 1.0
		
		# Initial EEF position
		self.initPos_arm = Point(1.0, 12.0, 14.0)	# in arm frame	
		self.carryOutPos_arm = Point(1.0, 12.0, 18.0)
		
		rate = rospy.Rate(10) #10hz
	
		while not rospy.is_shutdown():
			self.toInitPos()
			self.check_job()
			rate.sleep
			
		rospy.spin()
	
	
	def toInitPos(self):
		# Extend to only do once and move up first --> need eef pos_arm
		if self.inOperation == False:
			initial_state = self.moveToPos_client(self.initPos_arm, self.move_mode, self.moveDuration_abs, self.interpol_linear) 
			print("Moved to resting position",initial_state)
			
	
	
	def check_job(self):
		self.isPickedUp = False
		self.inOperation = True
		
		if self.job == None:
			self.inOperation = False
			print "No job received"
			return
		else:
			while self.job != 'cancel' and self.inOperation == True:
				
				if self.isPickedUp == False:
					self.pickup()	# Publish message inside that it was picked up	
				elif self.job == 'reposition':
					self.reposition()
				elif self.job =='carryout':
					self.carryout()
				else:
					print "Job is neither 'reposition' nor 'carryout' but '{}' ! ".format(self.job)
		
					
	def reposition(self):
		# Move above placing position
		abovePlace_pos = Point(self.placePos_arm.x, self.placePos_arm.y, self.placePos_arm.z + 10.0)			
	
		abovePlace_state = self.moveToPos_client(abovePlace_pos, self.move_mode, self.moveDuration_abs, self.interpol_linear)
		print("Moved above drop place", abovePlace_state)
		
		# Place object
		place_state = self.moveToPos_client(self.placePos_arm, self.move_mode, self.moveDuration_abs, self.interpol_linear)
		print("Placed object at", place_state)
		
		if place_state.error == False:
			self.pump_control(False)
			self.isPickedUp = False
			self.inOperation = False
			
	
	def carryout(self):
		# Move to carryout position
		carryOut_state = self.moveToPos_client(self.carryOutPos_arm, self.move_mode, self.moveDuration_abs, self.interpol_linear)		
		
		# Drop object when asked so by mother a few centimeters in front of the robot
		dropPos = self.carryOutPos_arm
		dropPos.y = dropPos.y + 10.0
		while self.inOperation == True:
			if self.drop == True:
				drop_state = self.moveToPos_client(dropPos, self.move_mode, self.moveDuration_abs, self.interpol_linear)
				if drop_state.error == False:
					self.inOperation = False
					print "Dropped object ! "
		
	
	def mother_callback(self, data):
		if data is not None:
			#pickupPos = PointStamped()
			#placePos = PointStamped()
		
			self.pickupPos_wheel = data.pickupPos.point
			self.placePos_wheel = data.placePos.point
		
			pickupPos = self.transform_wheelToArm(data.pickupPos)
			self.pickupPos_arm = pickupPos.point
			placePos = self.transform_wheelToArm(data.placePos)
			self.placePos_arm = placePos.point
		
			# Rescale to cm
			scale = 100.0 # meter to cm
			self.pickupPos_arm.x = self.pickupPos_arm.x*scale	
			self.pickupPos_arm.y = self.pickupPos_arm.y*scale
			self.pickupPos_arm.z = self.pickupPos_arm.z*scale
		
			self.placePos_arm.x = self.placePos_arm.x*scale	
			self.placePos_arm.y = self.placePos_arm.y*scale
			self.placePos_arm.z = self.placePos_arm.z*scale
		
			self.job = data.job
		
			self.drop = data.drop
		else:
			print "Mother topic empty"
		
	
	def goal_callback(self, data):
		self.goalPos_wheel = data
		self.goalPosStamped = self.transform_wheelToArm(data)
	
		scale = 100.0 # meter to cm
		self.goalPosStamped.point.x = self.goalPosStamped.point.x*scale	
		self.goalPosStamped.point.y = self.goalPosStamped.point.y*scale
		self.goalPosStamped.point.z = self.goalPosStamped.point.z*scale

	
	def joint_callback(self, data):
		self.eefJointPos_current = data
		

	def transform_wheelToArm(self, data):
		try:	
			data_arm = self.listener.transformPoint('uarm',data)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			print "Failed to transform  point"
			return
		
		return data_arm
		
		
	def transform_armToWheel(self, data):
		try:	
			data_wheel = self.listener.transformPoint('wheel_center',data)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			print "Failed to transform  point"
			return

		return data_wheel


	def pump_control(self, state):
		try:	
			data_arm = self.pump_service(state)
		except rospy.ServiceException, e:
			print "Pump service call failed: %s"%e
			return
			
			
	
	def convertToJoints_client(self, position, eef_orientation, check_limits):
		try:
			joint_position = self.joint_service(position, eef_orientation, check_limits)
		except rospy.ServiceException, e:
			print "ConvertToJoint service call failed: %s"%e
			return
			
		return joint_position
		


	def moveToPos_client(self, position, move_mode, move_duration, int_type):
		try:
			response = self.moveTo_service(position, self.eef_orientation, move_mode, move_duration, self.ignore_orientation, int_type, self.check_limits)
			return response
		except rospy.ServiceException, e:
			print "MoveTo service call failed: %s"%e
	
	
	def moveToJoints_client(self, position, move_mode, move_duration, int_type):
		try:
			response = self.moveToJoints_service(position[0], position[1], position[2], position[3], move_mode, move_duration, int_type, self.check_limits)
			return response
		except rospy.ServiceException, e:
			print "MoveToJoints service call failed: %s"%e
			
	'''
	def moveToPos_control_cartesian(self, position):
		old_position = Point()
		
		# Move absolute first time
		move_mode = 0
		firstMove_state = self.moveToPos_client(position, move_mode , self.moveDuration_abs, self.interpol_linear)
		
		offset_x = position.x-firstMove_state.position.x
		offset_y = position.y-firstMove_state.position.y
		offset_z = position.z-firstMove_state.position.z
		old_position = firstMove_state.position
		
		
		
		# Move relative until offset in x,y,z smaller than tolAbs
		while abs(offset_x) > self.absTol or abs(offset_y) > self.absTol or abs(offset_z) > self.absTol:
			move_mode = 1
			move_steps = 1.0 # number of movement steps to eliminate offset
			
			idx = np.argmax([abs(offset_x), abs(offset_y), abs(offset_z)])
			print "Offsets (x,y,z) are {}, {}, {}".format(offset_x,offset_y,offset_z)
			print ""	
			print "Maximum offset is {}".format(idx)	
			
			if idx == 0:
				moveX = Point(offset_x/move_steps,0.0,0.0)
				state = self.moveToPos_client(moveX, move_mode, self.moveDuration_rel, self.interpol_none)
				old_position = state.position
				offset_x = position.x - old_position.x 
				print "corrected x to {}".format(offset_x)
			elif idx == 1:
				moveY = Point(0.0,offset_y/move_steps,0.0)
				state = self.moveToPos_client(moveY, move_mode, self.moveDuration_rel, self.interpol_none)
				old_position = state.position
				offset_y = position.y - old_position.y
				print "corrected y to {}".format(offset_y)
			elif idx == 2:
				moveZ = Point(0.0,0.0,offset_z/move_steps)
				state = self.moveToPos_client(moveZ, move_mode, self.moveDuration_rel, self.interpol_none)
				old_position = state.position
				offset_z = position.z - old_position.z	 
				print "corrected z to {}".format(offset_z)
				
		print "Final control position is {}".format(old_position.position)		
	'''		
				
	def moveToJointPos_control(self, position):
		old_position = Point()
		
		# Move absolute first time but to higher z value
		state = None
		move_mode = 0
		position_high = position
		position_high.z = position_high.z + 3.0
		firstMove_state = self.moveToPos_client(position, move_mode , self.moveDuration_abs, self.interpol_linear)

		
		# Compute offset in joint space
		goalPos_joints = self.convertToJoints_client(position, self.eef_orientation, self.check_limits)		
		
		offset_j0 = goalPos_joints.j0 - self.eefJointPos_current.position[0]
		offset_j1 = goalPos_joints.j1 - self.eefJointPos_current.position[1]
		offset_j2 = goalPos_joints.j2 - self.eefJointPos_current.position[2]
		offset_j3 = goalPos_joints.j3 - self.eefJointPos_current.position[3]
		
		print "Servo angles are reported to be {}".format(self.eefJointPos_current.position)
		print "Goal joint position is {}".format(goalPos_joints)		

		
		# Reduce offset starting with base orientation, servo 1 and 2
		while abs(offset_j0) > self.j0Tol:
			move_mode = 1
			move_j0 = np.array([offset_j0, 0.0, 0.0, 0.0])
			state = self.moveToJoints_client(move_j0, move_mode, self.moveDuration_rel, self.interpol_none)
			print "Corrected j0 due to an offset of {}".format(offset_j0)
			offset_j0 = goalPos_joints.j0 - state.j0
			
		while abs(offset_j1) > self.j1Tol:
			move_mode = 1
			move_j1 = np.array([0.0, offset_j1, 0.0, 0.0])
			state = self.moveToJoints_client(move_j1, move_mode, self.moveDuration_rel, self.interpol_none)
			print "Corrected j1 due to an offset of {}".format(offset_j1)
			offset_j1 = goalPos_joints.j1 - state.j1
			
		while abs(offset_j2) > self.j2Tol:
			move_mode = 1
			move_j2 = np.array([0.0, 0.0, offset_j2, 0.0])
			state = self.moveToJoints_client(move_j2, move_mode, self.moveDuration_rel, self.interpol_none)
			print "Corrected j2 due to an offset of {}".format(offset_j2)
			offset_j2 = goalPos_joints.j2 - state.j2

				
		print "Final reported position is {}".format(self.eefJointPos_current.position)	
		print "Goal position is {}".format(goalPos_joints)
			
		return state
					
			
	def absoluteFromDifference(self, goal, current):
		offsetX = goal.x - current.x
		offsetY = goal.y - current.y
		offsetZ = goal.z - current.z
		print "Offsets in x, y, z are {}, {}, {}".format(offsetX, offsetY, offsetZ)
		
		newgoal = Point(goal.x + offsetX, goal.y+offsetY, goal.z+offsetZ)
		return newgoal
		
				
	def pickup(self):
		aboveGoal_pos = Point(self.pickupPos_arm.x, self.pickupPos_arm.y, self.pickupPos_arm.z + 4.0)			
	
		aboveGoal_state = self.moveToPos_client(aboveGoal_pos, self.move_mode, self.moveDuration_abs, self.interpol_linear)
		print("Moved above goal", aboveGoal_state)
	
		
		# Move down
		#print "Sending eef to target {}".format(pickupPos_arm)
		print "Sending eef to target {}".format(self.pickupPos_wheel)
		#atGoal_state = self.moveToPos_control(pickupPos_arm)
		
		# Correct position above robot
		pickupPos_high_new = self.absoluteFromDifference(self.pickupPos_arm, aboveGoal_state.position)
		aboveGoal_state = self.moveToPos_client(pickupPos_high_new, self.move_mode, self.moveDuration_abs, self.interpol_linear)
		
		# Modify goal pos to go to lower z
		pickupPos_arm_low = pickupPos_high_new
		pickupPos_arm_low.z = self.pickupPos_arm.z-1.0
		
		# Move down with corrected x,y
		atGoal_state = self.moveToPos_client(pickupPos_arm_low,self.move_mode, self.moveDuration_abs, self.interpol_linear)
		
		
		#-- Move absolute to reduce offset --#
		atGoal_state = self.moveToPos_client(pickupPos_arm_low,self.move_mode, self.moveDuration_abs, self.interpol_linear)
		
		offsetX = pickupPos_arm_low.x - atGoal_state.position.x
		offsetY = pickupPos_arm_low.y - atGoal_state.position.y
		offsetZ = pickupPos_arm_low.z - atGoal_state.position.z
		print "New Offsets in x, y, z are {}, {}, {}".format(offsetX, offsetY, offsetZ)
		
		#atGoal_state = self.moveToJointPos_control(pickupPos_arm_low)
		print("Moved to target", atGoal_state)
		
		
		if atGoal_state.error == False:
			print "Changed pickup status to picked up !"
			self.isPickedUp = True
		
		'''
		atGoal_stamped = PointStamped()
		atGoal_stamped.point = atGoal_state.position
		atGoal_stamped.header.frame_id = 'uarm'
		atGoal_state_wheel = self.transform_armToWheel(atGoal_stamped)
		print("Moved to target", atGoal_state_wheel)
		'''
		
		rospy.sleep(2.0)
		
		# Turn on pump
		self.pump_control(True)
	
		# Move back up
		aboveGoal_state = self.moveToPos_client(aboveGoal_pos, self.move_mode, self.moveDuration_abs, self.interpol_linear)
		
		

		
		return atGoal_state
		
		
				
	def moveSteps(self):
	
		scale = 100.0 # meter to cm
		
		# To Resting position
		
		
		#rospy.sleep(2.0)

 		if self.goalPosStamped is None:
			return
		
		# To GOAL position
		if self.goalPosStamped.header.frame_id is 'None':
			print ""
			print "No goal position provided"
			print ""
			return
			
		goalPos_current = self.goalPosStamped.point	
		atGoal_state = self.moveToGoal(goalPos_current)
		
		# Turn ON PUMP
		
		# CRITERIA??
		
		
		# Move back to initial in two steps
		aboveGoal_pos = Point(goalPos_current.x, goalPos_current.y, self.initPos_arm.z)
		aboveGoal_state = self.moveToPos_client(aboveGoal_pos, self.move_mode, self.moveDuration_abs, self.interpol_linear)
		self.pump_control(False)
		
		initial_state = self.moveToPos_client(self.initPos_arm, self.move_mode, self.moveDuration_abs, self.interpol_linear)
		print "Back in initial position"

		self.goalPosStamped = None
	


if __name__ == "__main__":
	Manipulate()
	
	
	

