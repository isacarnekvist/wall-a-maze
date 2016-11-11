#!/usr/bin/env python

import sys
import rospy
import tf
import numpy as np

from uarm.srv import *
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header



class Manipulation():

	def __init__(self):
		rospy.init_node('move_arm_client')
	
		rospy.Subscriber("/objectPos_wheelcenter", PointStamped, self.goal_callback)
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
		
		# Other
		self.goalPos_wheel = PointStamped()
		self.goalPosStamped = None
	
		self.eefPos_current = JointState()
		
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

		rate = rospy.Rate(10) #10hz
	
		while not rospy.is_shutdown():
			self.moveSteps()
			rate.sleep
			
		rospy.spin()
	
	
	
	def goal_callback(self, data):
		self.goalPos_wheel = data
		self.goalPosStamped = self.transform_wheelToArm(data)
	
		scale = 100.0 # meter to cm
		self.goalPosStamped.point.x = self.goalPosStamped.point.x*scale	
		self.goalPosStamped.point.y = self.goalPosStamped.point.y*scale
		self.goalPosStamped.point.z = self.goalPosStamped.point.z*scale

	
	def joint_callback(self, data):
		self.eefPos_current = data
		

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
		
		offset_j0 = goalPos_joints.j0 - self.eefPos_current.position[0]
		offset_j1 = goalPos_joints.j1 - self.eefPos_current.position[1]
		offset_j2 = goalPos_joints.j2 - self.eefPos_current.position[2]
		offset_j3 = goalPos_joints.j3 - self.eefPos_current.position[3]
		
		print "Servo angles are reported to be {}".format(self.eefPos_current.position)
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

				
		print "Final reported position is {}".format(self.eefPos_current.position)	
		print "Goal position is {}".format(goalPos_joints)
			
		return state
					
				
	def moveToGoal(self, goalPos):
		aboveGoal_pos = Point(goalPos.x, goalPos.y, self.initPos_arm.z)			
	
		aboveGoal_state = self.moveToPos_client(aboveGoal_pos, self.move_mode, self.moveDuration_abs, self.interpol_linear)
		print("Moved above goal", aboveGoal_state)
		#rospy.sleep(2.0)
		#input("continue")
	
		# To ABOVEGOAL CLOSER position
		'''
		aboveGoalClose_pos = Point(goalPos.x, goalPos.y, goalPos.z+2.0)			

		aboveGoalClose_state = self.moveToPos_control(aboveGoalClose_pos)
		print("Moved close above goal", aboveGoalClose_state)
		rospy.sleep(2.0)
		'''
		
		# Move down
		#print "Sending eef to target {}".format(goalPos)
		print "Sending eef to target {}".format(self.goalPos_wheel.point)
		#atGoal_state = self.moveToPos_control(goalPos)
		
		# Modify goal pos to go to lower z
		goalPos.z = goalPos.z-2.0
		#atGoal_state = self.moveToPos_client(goalPos,self.move_mode, self.moveDuration_abs, self.interpol_linear)
		atGoal_state = self.moveToJointPos_control(goalPos)
		
		print("Moved to target", atGoal_state)
		
		'''
		atGoal_stamped = PointStamped()
		atGoal_stamped.point = atGoal_state.position
		atGoal_stamped.header.frame_id = 'uarm'
		atGoal_state_wheel = self.transform_armToWheel(atGoal_stamped)
		print("Moved to target", atGoal_state_wheel)
		'''
		
		rospy.sleep(2.0)
	
		return atGoal_state
		
		
				
	def moveSteps(self):
	
		scale = 100.0 # meter to cm
		
		# To Resting position
		
		initial_state = self.moveToPos_client(self.initPos_arm, self.move_mode, self.moveDuration_abs, self.interpol_linear) 
		print("Moved to resting position",initial_state)
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
		self.pump_control(True)
		
		# Move back to initial in two steps
		aboveGoal_pos = Point(goalPos_current.x, goalPos_current.y, self.initPos_arm.z)
		aboveGoal_state = self.moveToPos_client(aboveGoal_pos, self.move_mode, self.moveDuration_abs, self.interpol_linear)
		self.pump_control(False)
		
		initial_state = self.moveToPos_client(self.initPos_arm, self.move_mode, self.moveDuration_abs, self.interpol_linear)
		print "Back in initial position"

		self.goalPosStamped = None
	


if __name__ == "__main__":
	Manipulation()
	
	
	

