#!/usr/bin/env python

import sys
import rospy
import tf
import numpy as np

from uarm.srv import *
from sensor_msgs.msg import JointState, PointCloud2
from geometry_msgs.msg import Point, PointStamped, Twist
from std_msgs.msg import Header, String
from manipulation.msg import Manipulation

from manipulation.srv import *
from classifier.srv import *



class Manipulate():

	def __init__(self):
		self.wheels_pub = rospy.Publisher('motor_controller', Twist, queue_size=10)
		#self.state_pub = rospy.Publisher('manipulation_state', String, queue_size=10) 
		
		rospy.init_node('manipulation_server')
		 
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
		
		
		# Initial EEF position
		self.initPos_arm = Point(1.0, 12.0, 14.0)	# in arm frame	
		self.carryOutPos_arm = Point(1.0, 14.0, 16.0)
		
		self.toInitPos()

		# Services
		rospy.Service('manipulation/pickup_object', PickupObject, self.handle_pickup)
		#rospy.Service('manipulation/pickup_trap', PickupTrap, self.handle_pickupTrap)
		rospy.Service('manipulation/place_object', PlaceObject, self.handle_place)
		rospy.Service('manipulation/carry_object', CarryObject, self.handle_carryObject)
		rospy.Service('manipulation/drop_object', DropObject, self.handle_drop)
		
		rospy.spin()
			
	
	
	def toInitPos(self):
		initial_state = self.moveToPos_client(self.initPos_arm, self.move_mode, self.moveDuration_abs, self.interpol_linear) 
			

	def handle_pickup(self,request):
		response = self.toPickupPos(request)
		if response == False:
			return PickupObjectResponse(False)
		else:
			pickupResponse = self.pickup(request)
			print("pickupResponse is {}".format(pickupResponse))
			return PickupObjectResponse(pickupResponse)	# Modify later
	

	def handle_place(self,request):
		# TODO: Sanity check on placing position
		placePos = self.transform_wheelToArm(request.position)
		placePos_arm = placePos.point

		scale = 100.0
		placePos_arm.x = placePos_arm.x*scale
		placePos_arm.y = placePos_arm.y*scale
		placePos_arm.z = placePos_arm.z*scale

		# Place object
		place_state = self.moveToPos_client(placePos_arm, self.move_mode, self.moveDuration_abs, self.interpol_linear)
		
		if place_state.error == False:	# Maybe add control to place it savely
			self.pump_control(False)
			print("Placed object at", place_state)
			self.toInitPos()
			return PlaceObjectResponse(True)
		else:
			return PlaceObjectResponse(False)


	def handle_carryObject(self,request):
		carryOut_state = self.moveToPos_client(self.carryOutPos_arm, self.move_mode, self.moveDuration_abs, self.interpol_linear)		
		if carryOut_state.error == False:
			return CarryObjectResponse(True)


	def handle_drop(self,request):
		# Drop object when asked so by mother a few centimeters in front of the robot
		dropPos = self.carryOutPos_arm
		dropPos.y = dropPos.y + 10.0

		drop_state = self.moveToPos_client(dropPos, self.move_mode, self.moveDuration_abs, self.interpol_linear)
		if drop_state.error == True:
			return DropObjectResponse(False)
		else:
			self.pump_control(False)
			print("Dropped object")
			self.toInitPos()
			return DropObjectResponse(True)
						

	def objectPos_client(self,request,select):
		rospy.wait_for_service('find_object')	#Does this take long?
		deltaX = None
		deltaY = None
		deltaZ = None
		number = None
		delta_current = None
		delta = 100 # Assuming meters

		color = [request.color]
		objectType = [request.type]
		

		try:
			get_objectPos = rospy.ServiceProxy('find_object', Find_Object)
			response = get_objectPos(PointCloud2(),color, objectType)
			
			# Check if requested object detected

			if not response.x:
				print("No object passed")
				return False	
			#if response.x[0]==0:
			#	return False

			

			'''
			# Convert to world frame and check which object it is
			if select == True:
				for i in response.x:
					objectX = response.x[i]+position.x 	# CHANGE
					objectY = response.y[i]+position.y
					objectZ = response.z[i]+position.z
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
			objectPos.header.frame_id = response.header.frame_id
			objectPos.point = Point(response.x[number],response.y[number],response.z[number])

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

		yTol = 0.01
		xTol = 0.10
		xAim = 0.26
		
		move_angular = Twist()
		move_linear = Twist()
	
		times_rotated = 0

		while times_rotated < 12:
			
			objectPos = self.objectPos_client(request, select=True)

			if objectPos == False:
				print("Rotate")
				move_angular.angular.z = 1.0 # m/s
				self.wheels_pub.publish(move_angular)		
				times_rotated += 1
				print("Times rotated {}".format(times_rotated))
				rospy.sleep(0.6)
				self.wheels_pub.publish(Twist())

			else:
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
				return True
		
		

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
			


	def moveToPos_client(self, position, move_mode, move_duration, int_type):
		try:
			response = self.moveTo_service(position, self.eef_orientation, move_mode, move_duration, self.ignore_orientation, int_type, self.check_limits)
			if response.error == True:
				print "Unable to move to position {}. Returning to initial pos.".format(self.initPos_arm)
				self.toInitPos()
			return response
		except rospy.ServiceException, e:
			print "MoveTo service call failed: %s"%e		
					
			
	def absoluteFromDifference(self, goal, current):
		offsetX = goal.x - current.x
		offsetY = goal.y - current.y
		offsetZ = goal.z - current.z
		print "Offsets in x, y, z are {}, {}, {}".format(offsetX, offsetY, offsetZ)
		
		newgoal = Point(goal.x + offsetX, goal.y+offsetY, goal.z+offsetZ)
		return newgoal
		
				
	def pickup(self,request):
		num_tries = 0
		max_tries = 5
		zTol = 0.03	# [m]

		# ToDo: Add multiple probing, but see first how behaves when multiple times going down!

		while num_tries < max_tries:

			objectPos = self.objectPos_client(request,select=True)
			print("It is the {}th pickup try".format(num_tries))

			if objectPos == False:
				print("Object no longer in sight!")
				return False
				break
			else:
				pickupPos_wheelcenter = objectPos.point

				# Transform to arm frame
				pickupPos = self.transform_wheelToArm(objectPos)
				pickupPos_arm = pickupPos.point
			
				# Rescale to cm
				scale = 100.0 # meter to cm
				pickupPos_arm.x = pickupPos_arm.x*scale	
				pickupPos_arm.y = pickupPos_arm.y*scale
				pickupPos_arm.z = pickupPos_arm.z*scale

				aboveGoal_pos = Point(pickupPos_arm.x, pickupPos_arm.y, pickupPos_arm.z + 4.0) #correct 6, include in param file	
			
				aboveGoal_state = self.moveToPos_client(aboveGoal_pos, self.move_mode, self.moveDuration_abs, self.interpol_linear)
				if aboveGoal_state.error == True:
					return False
					break

				#print("Moved above goal", aboveGoal_state)
				
				# Correct position above robot
				pickupPos_high_new = self.absoluteFromDifference(aboveGoal_pos, aboveGoal_state.position)
				aboveGoal_state = self.moveToPos_client(pickupPos_high_new, self.move_mode, self.moveDuration_abs, self.interpol_linear)

				if aboveGoal_state.error == True:
					return False					
					break
					
				
				# Modify goal pos to go to lower z
				pickupPos_arm_low = pickupPos_high_new
				pickupPos_arm_low.z = pickupPos_arm.z	#change when perception correct
				
				# Turn on pump
				self.pump_control(True)
				
				# Move down with corrected x,y
				atGoal_state = self.moveToPos_client(pickupPos_arm_low,self.move_mode, self.moveDuration_abs, self.interpol_linear)
				if atGoal_state.error == True:
					self.pump_control(False)
					return False					
					break					

				# Move back up
				aboveGoal_pos.z = aboveGoal_pos.z + 4.0
				aboveGoal_state = self.moveToPos_client(aboveGoal_pos, self.move_mode, self.moveDuration_abs, self.interpol_linear)
				
				# Check if picked up
				objectPos_new = self.objectPos_client(request,select=False)
				if objectPos_new == False:
					print("Object no longer in sight!")
					self.pump_control(False)
					self.toInitPos()
					return False
				
				print("Object is lifted by {} m".format(objectPos_new.point.z-pickupPos_wheelcenter.z))

				if (objectPos_new.point.z-pickupPos_wheelcenter.z) > zTol:
					print("pickup returns true")
					return True
					break
					
	
		
	


if __name__ == "__main__":
	Manipulate()
	
	
	

