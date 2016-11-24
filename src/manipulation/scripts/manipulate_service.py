#!/usr/bin/env python

import sys
import rospy
import tf
import numpy as np

from uarm.srv import *
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, PointStamped, Twist
from std_msgs.msg import Header, String
from manipulation.msg import Manipulation




class Manipulate():

	def __init__(self):
		self.wheels_pub = rospy.Publisher('motor_controller', Twist, queue_size=10)
		#self.state_pub = rospy.Publisher('manipulation_state', String, queue_size=10) 
		
		rospy.init_node('move_arm_client')
	
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
		self.inPickupPos = False
		
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
		
		self.toInitPos()

		# Services
		rospy.Service('pickup_object', PickupOjbect, self.handle_pickup)
		#rospy.Service('pickup_trap', PickupTrap, self.handle_pickupTrap)
		rospy.Service('place_object', PlaceObject, self.handle_place)
		rospy.Service('carry_object', CarryObject, self.handle_carryObject)

		
		rospy.spin()
			
	
	
	def toInitPos(self):
		# Extend to only do once and move up first --> need eef pos_arm
		if self.inOperation == False:
			initial_state = self.moveToPos_client(self.initPos_arm, self.move_mode, self.moveDuration_abs, self.interpol_linear) 
			#print("Moved to resting position",initial_state)
			
	def handle_pickup(self,request):
		self.toPickupPos(request)
		pickupResponse = self.pickup(request)
		return PickupObjectResponse(pickupResponse.error)	# Modify later
	
					
	def handle_place(self):
		# TODO: Sanity check on placing position

		# Move above placing position
		abovePlace_pos = Point(self.placePos_arm.x, self.placePos_arm.y, self.placePos_arm.z + 10.0)			
	
		abovePlace_state = self.moveToPos_client(abovePlace_pos, self.move_mode, self.moveDuration_abs, self.interpol_linear)
		print("Moved above drop place", abovePlace_state)
		
		# Place object
		place_state = self.moveToPos_client(self.placePos_arm, self.move_mode, self.moveDuration_abs, self.interpol_linear)
		print("Placed object at", place_state)
		
		if place_state.error == False:	# Check if actually done!
			self.pump_control(False)
		
		return PlaceObjectResponse(place_state.error)

	
	def handle_carryObject(self):
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
		

	def objectPos_client(self,request):
		rospy.wait_for_service('insert_name')	#Does this take long?
		deltaX = None
		deltaY = None
		deltaZ = None
		number = None
		delta_current = None
		delta = 100 # Assuming meters

		try:
			get_objectPos = rospy.ServiceProxy('insert_name', NAME)
			response = get_objectPos(request.color,request.type)

			# Check if requested object detected
			if response.x(0)==0:
				return False

			'''
			# Convert to world frame and check which object it is
			for i in response.x:
				objectX = response.x(i)+position.x 	# CHANGE
				objectY = response.y(i)+position.y
				objectZ = response.z(i)+position.z
				deltaX = objectX - request.position.x
				deltaY = objectY - request.position.y
				deltaZ = objectZ - request.position.z

				delta_current = np.square(deltaX) + np.square(deltaY) + np.square(deltaZ)
				if delta_current < delta:
					delta = delta_current
					number = i

			if delta < delta_tol:
			'''

			number = 0

			objectPos = PointStamped()
			objectPos.header.frame_id = response.frame_id
			objectPos.point = Point(response.x(number),response.y(number),response.z(number))

			return objectPos
		except rospy.ServiceException, e:
			print("Service call to object position failed")


	def toPickupPos(self,request):
		# Simple heuristic control:
		# 1. Rotate until object in line of sight and y=0
		# 2. Move in x until in pickable/preferred x range
		
		# ToDo: 
		# Depending on request, choose right object

		#print("Manipulation moves robot!")

		yTol = 0.02
		xTol = 0.10
		xAim = 0.26
		
		move_angular = Twist()
		move_linear = Twist()
	
		times_rotated = 0

		while times_rotated < 12:
			
			objectPos = self.objectPos_client(request)

			if objectPos == False:
				print("Rotate")
				move_angular.angular.z = 1.0 # m/s
				self.wheels_pub.publish(move_angular)		
				times_rotated += 1
				print("Times rotated {}".format(times_rotated))
				rospy.sleep(0.6)
				self.wheels_pub.publish(Twist())

			else
				print("Saw an object!")
				while abs(objectPos.point.y) > yTol:
					move_angular.angular.z = np.sign(objectPos.point.y)*0.8 # m/s
					self.wheels_pub.publish(move_angular)
					rospy.sleep(0.2)
					self.wheels_pub.publish(Twist())		
					objectPos = rospy.wait_for_message("/objectPos_wheelcenter", PointStamped,timeout=0.4)
					print("Y offset is {}".format(objectPos.point.y))

				self.wheels_pub.publish(Twist())

				print("Stopped rotating, object at y={}".format(self.pickupPos_wheel.y))
				
				while objectPos.point.x > xAim:
					move_linear.linear.x = 0.10 # m/s
					self.wheels_pub.publish(move_linear)
					objectPos = rospy.wait_for_message("/objectPos_wheelcenter", PointStamped,timeout=0.4)
					print("X offset is {}".format(objectPos.point.x))

				self.wheels_pub.publish(Twist())
				print("Stopped moving forward, object at x={}".format(objectPos.point.x))
				break
		

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
			if response.error == True:
				print "Unable to move to position {}. Returning to initial pos.".format(position)
				#self.inOperation = False
			return response
		except rospy.ServiceException, e:
			print "MoveTo service call failed: %s"%e
	
	
	def moveToJoints_client(self, position, move_mode, move_duration, int_type):
		try:
			response = self.moveToJoints_service(position[0], position[1], position[2], position[3], move_mode, move_duration, int_type, self.check_limits)
			if response.error == True:
				print "Unable to move to position {}. Returning to initial pos.".format(position)
				self.inOperation = False
			return response
		except rospy.ServiceException, e:
			print "MoveToJoints service call failed: %s"%e
			
					
			
	def absoluteFromDifference(self, goal, current):
		offsetX = goal.x - current.x
		offsetY = goal.y - current.y
		offsetZ = goal.z - current.z
		print "Offsets in x, y, z are {}, {}, {}".format(offsetX, offsetY, offsetZ)
		
		newgoal = Point(goal.x + offsetX, goal.y+offsetY, goal.z+offsetZ)
		return newgoal
		
				
	def pickup(self):
		aboveGoal_pos = Point(self.pickupPos_arm.x, self.pickupPos_arm.y, self.pickupPos_arm.z + 6.0)#correct 6		
	
		aboveGoal_state = self.moveToPos_client(aboveGoal_pos, self.move_mode, self.moveDuration_abs, self.interpol_linear)
		print("Moved above goal", aboveGoal_state)
	
		
		# Move down
		#print "Sending eef to target {}".format(pickupPos_arm)
		#print "Sending eef to target {}".format(self.pickupPos_wheel)
		#atGoal_state = self.moveToPos_control(pickupPos_arm)
		
		# Correct position above robot
		pickupPos_high_new = self.absoluteFromDifference(self.pickupPos_arm, aboveGoal_state.position)
		aboveGoal_state = self.moveToPos_client(pickupPos_high_new, self.move_mode, self.moveDuration_abs, self.interpol_linear)
		
		# Modify goal pos to go to lower z
		pickupPos_arm_low = pickupPos_high_new
		pickupPos_arm_low.z = self.pickupPos_arm.z	#change when perception correct
		
		# Turn on pump
		self.pump_control(True)
		
		# Move down with corrected x,y
		atGoal_state = self.moveToPos_client(pickupPos_arm_low,self.move_mode, self.moveDuration_abs, self.interpol_linear)
		
	
		# Move back up
		aboveGoal_state = self.moveToPos_client(aboveGoal_pos, self.move_mode, self.moveDuration_abs, self.interpol_linear)
		
		return atGoal_state
		
	


if __name__ == "__main__":
	Manipulate()
	
	
	

