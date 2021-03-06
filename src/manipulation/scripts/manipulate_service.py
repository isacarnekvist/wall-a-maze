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

		# Global variables
		self.booby_position = PointStamped()
		self.booby_detected = False

		self.wheels_pub = rospy.Publisher('motor_controller', Twist, queue_size=10)
		#self.state_pub = rospy.Publisher('manipulation_state', String, queue_size=10) 
		
		rospy.init_node('manipulation_server')
		 
		rospy.Subscriber("/perception/trap_position",PointStamped,self.booby_callback)
		rospy.Subscriber("/motor_controller", Twist, self.motor_callback)

		# Wait for transform
		self.listener = tf.TransformListener()
		self.listener.waitForTransform("wheel_center", "uarm", rospy.Time(), rospy.Duration(8.0))
	
		# Wait for move, move_joints, pump and inverse kinematics service
		rospy.wait_for_service('/uarm/move_to')
		self.moveTo_service = rospy.ServiceProxy('/uarm/move_to', MoveTo)
	
		rospy.wait_for_service('/uarm/pump')
		self.pump_service = rospy.ServiceProxy('/uarm/pump',Pump)
		
		#-- Variables --#
		self.robot_rotation = 1.0
		self.recent_robot_rotations = []
		self.pickup_type = 'nothing'
		self.pickup_color = 'nothing'

		# Goal positions
		self.pickupPos_arm = Point()
		self.placePos_arm = Point()
		self.pickupPos_wheel = Point()
		self.placePos_wheel = Point()
		self.job = None
		self.drop = False

				
		# Define move parameters 
		self.move_mode = 0	# (0 absolute,1 realtive)
		self.moveDuration_abs = rospy.Duration(1.0) # (in s)
		self.moveDuration_rel = rospy.Duration(0.2)
		
		self.interpol_way = 2 	# (0:none,1:cubic,2:linear)
		self.check_limits = True
		self.eef_orientation = 0
		self.ignore_orientation = True
		
		
		# Initial EEF position
		self.initPos_arm = Point(1.0, 12.0, 14.0)	# in arm frame	
		self.carryOutPos_arm = Point(1.0, 14.0, 16.0)
		self.dropPos = Point(0.0, 24.0, 0.0)

		self.toInitPos()
		self.pump_control(False)

		# Services
		rospy.Service('manipulation/pickup_object', PickupObject, self.handle_pickup)
		rospy.Service('manipulation/pickup_trap', PickupTrap, self.handle_pickupTrap)
		rospy.Service('manipulation/place_object', PlaceObject, self.handle_place)
		rospy.Service('manipulation/carry_object', CarryObject, self.handle_carryObject)
		rospy.Service('manipulation/drop_object', DropObject, self.handle_drop)

		rospy.spin()
			
	
	
	def toInitPos(self):
		initial_state = self.moveToPos_client(self.initPos_arm, self.move_mode, self.moveDuration_abs, self.interpol_way) 
			
	def handle_pickupTrap(self,request):
		response = self.rotateToBooby()
		if response == False:
			return PickupTrapResponse(response)
		else:
			rospy.sleep(0.5)
			pickupResponse = self.pickup(request,object_type='booby')
			if pickupResponse == False:
				self.toInitPos()
			return PickupTrapResponse(pickupResponse)



	def handle_pickup(self,request):
		response = self.toPickupPos(request)
		if response == False:
			return PickupObjectResponse(False)
		else:
			if request.doPickup:
				pickupResponse = self.pickup(request,object_type='other')
				print("pickupResponse is {}".format(pickupResponse))
				return PickupObjectResponse(pickupResponse)	# Modify later
        	
			return PickupObjectResponse(True)
    
	

	def handle_place(self,request):
		# TODO: Sanity check on placing position
		placePos = self.transform_wheelToArm(request.position)
		placePos_arm = placePos.point

		scale = 100.0
		placePos_arm.x = placePos_arm.x*scale
		placePos_arm.y = placePos_arm.y*scale
		placePos_arm.z = placePos_arm.z*scale

		# Place object
		place_state = self.moveToPos_client(placePos_arm, self.move_mode, self.moveDuration_abs, self.interpol_way)
		
		if place_state.error == False:	# Maybe add control to place it savely
			self.pump_control(False)
			print("Placed object at", place_state)
			self.toInitPos()
			return PlaceObjectResponse(True)
		else:
			return PlaceObjectResponse(False)


	def handle_carryObject(self,request):
		carryOut_state = self.moveToPos_client(self.carryOutPos_arm, self.move_mode, self.moveDuration_abs, self.interpol_way)		
		if carryOut_state.error == False:
			return CarryObjectResponse(True)


	def handle_drop(self,request):
		drop_state = self.moveToPos_client(self.dropPos, self.move_mode, self.moveDuration_abs, self.interpol_way)
		if drop_state.error == True:
			return DropObjectResponse(False)
		else:
			self.pump_control(False)
			print("Dropped object")
			self.toInitPos()
			return DropObjectResponse(True)
						

	def objectPos_client(self,request,select,forPickup=False):
		rospy.wait_for_service('find_object')	#Does this take long?
		deltaX = None
		deltaY = None
		deltaZ = None
		number = None
		delta_current = None
		delta = 100 # Assuming meters

		
		#if request.type == '':
		objectType = []

		if request.color == '':
			color = []
		else:
			color = [request.color]

		if forPickup:
			color = [self.pickup_color]
			objectType = [self.pickup_type]
			
		print("Looking for object of type {} and color {}".format(objectType, color))	
		try:
			get_objectPos = rospy.ServiceProxy('find_object', Find_Object)
			response = get_objectPos(PointCloud2(),color, objectType)

			if not response.x:
				print("No object passed")
				return False	
			
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
			
			self.pickup_type = response.type[number]
			self.pickup_color = response.color[number]	
				
			return objectPos
		except rospy.ServiceException, e:
			print("Service call to object position failed")


	def rotateToBooby(self):
		move_angular = Twist()
		yTol = 0.03
		times_rotated = 0
		move_angular.angular.z = self.robot_rotation * 1.0 # m/s

		while times_rotated < 10:
			objectPos = self.booby_position
			
			# Turn opposite
			if times_rotated > 3:
				move_angular.angular.z = -1.0

			if not self.booby_detected:
				print("Pickup position is {}".format(objectPos.point))
				print("Rotate")
				self.wheels_pub.publish(move_angular)		
				times_rotated += 1
				print("Times rotated {}".format(times_rotated))
				rospy.sleep(0.6)
				self.wheels_pub.publish(Twist())
			else:
				print("Saw trap!")
				while abs(objectPos.point.y) > yTol:
					move_angular.angular.z = np.sign(objectPos.point.y)*0.8 # m/s
					self.wheels_pub.publish(move_angular)
					rospy.sleep(0.2)
					self.wheels_pub.publish(Twist())
					rospy.sleep(0.2)
					objectPos = self.booby_position
					if objectPos.point.z < 0.01:
						print("Trap lost while rotating.")
						self.wheels_pub.publish(Twist())
						return False
						break		

				self.wheels_pub.publish(Twist())

				print("Stopped rotating, object at y={}".format(self.pickupPos_wheel.y))
				break
				return True


	def toPickupPos(self, request, setAim=False, xAim=None):
		# Simple heuristic control:
		# 1. Rotate until object in line of sight and y=0
		# 2. Move in x until in pickable/preferred x range
		
		# ToDo: 
		# Depending on request, choose right object

		#print("Manipulation moves robot!")

		yTol = 0.02
		xTol = 0.10
		xAim_current = 0.26

		if setAim:
			xAim_current = xAim
			print("Moving to object with xAim = {}".format(xAim))
		
		move_angular = Twist()
		move_linear = Twist()
	
		times_rotated = 0
		move_angular.angular.z = self.robot_rotation * 1.0 # m/s

		start_time_moving = rospy.Time.now()
		max_duration = rospy.Duration(30.0)

		while times_rotated < 12:
			
			if (rospy.Time.now()-start_time_moving) > max_duration:
				print("Time is over - quitting")
				return False


			objectPos = self.objectPos_client(request, select=True)
			if objectPos == False:
				print("Rotate")
				self.wheels_pub.publish(move_angular)		
				times_rotated += 1
				print("Times rotated {}".format(times_rotated))
				rospy.sleep(0.6)
				self.wheels_pub.publish(Twist())
				rospy.sleep(0.2)

			else:
				print("Saw an object!")
				while objectPos.point.x > xAim_current or abs(objectPos.point.y) > yTol:
				    if abs(objectPos.point.y) > yTol:
					    move_angular.angular.z = np.sign(objectPos.point.y)*0.8 # m/s
					    self.wheels_pub.publish(move_angular)
					    rospy.sleep(0.2)
					    self.wheels_pub.publish(Twist())
					    rospy.sleep(0.2)
					    objectPos = self.objectPos_client(request, select=True)
					    if objectPos == False:
							rospy.sleep(0.4)
							objectPos = self.objectPos_client(request, select=True)
							if objectPos == False:							
								print("Object lost while rotating.")
								self.wheels_pub.publish(Twist())
								return False
								break		

				    self.wheels_pub.publish(Twist())

				    #print("Stopped rotating, object at y={}".format(self.pickupPos_wheel.y))
				    

				    if objectPos.point.x > xAim_current:
					    move_linear.linear.x = 0.10  # m/s
					    self.wheels_pub.publish(move_linear)
					    rospy.sleep(0.4)
					    self.wheels_pub.publish(Twist())
					    rospy.sleep(0.2)
					    print("X offset is {}".format(objectPos.point.x))
					    objectPos = self.objectPos_client(request, select=True)

					
					    #if (rospy.get_time()-start_time_forward) < max_duration_forward :
					    #	print("time moving forward expired")

					    if objectPos == False:
							rospy.sleep(0.4)
							objectPos = self.objectPos_client(request, select=True)
							if objectPos == False:
								print("Object lost twice while moving forward.")
								self.wheels_pub.publish(Twist())
								return False
								break		

				self.wheels_pub.publish(Twist())
				#print("Stopped moving forward, object at x={}".format(objectPos.point.x))
				#break
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
		
				
	def pickup(self,request,object_type):
		num_tries = 0
		max_tries = 3

		if self.pickup_type == 'ball':
			max_tries = 1

		zTol = 0.02	# [m]

		# ToDo: Add multiple probing, but see first how behaves when multiple times going down!

		if self.pickup_type == 'cross':
			response = self.toPickupPos(request, setAim=True, xAim=0.24)
			print("Moving forward again for cross")
			if not response:
				print("Error correcting pickup pos for cross")
				return False

		if self.pickup_type == 'star' and self.pickup_color == 'purple':
			response = self.toPickupPos(request, setAim=True, xAim=0.24)
			print("Moving forward again for purple cross")
			if not response:
				print("Error correcting pickup pos for purple cross")
				return False

		if self.pickup_type == 'star' and self.pickup_color == 'orange':
			response = self.toPickupPos(request, setAim=True, xAim=0.25)
			print("Moving forward again for orange cross")
			if not response:
				print("Error correcting pickup pos for orange cross")
				return False
		

		while num_tries < max_tries:
			if object_type == 'booby':
				objectPos = self.booby_position
				if not self.booby_detected:
					print("Trap no longer in sight.")
					return False
			else:
				objectPos = self.objectPos_client(request, select=True, forPickup=True)
				if objectPos == False:
					objectPos = self.objectPos_client(request, select=True, forPickup=True)
					if objectPos == False:
						print("Object no longer in sight at start of pickup!")
						self.pump_control(False)
						self.toInitPos()
						return False

			print("It is the {} th pickup try for type {}".format(num_tries, self.pickup_type))
			
			pickupPos_wheelcenter = objectPos.point

			# Hacks for objects
			if self.pickup_type == 'cylinder':
				print("Hacked cylinder")
				objectPos.point.x = objectPos.point.x + 0.025 + 0.01
				objectPos.point.z = objectPos.point.z + 0.01
			elif self.pickup_type == 'cross':
				print("Hacked cross")
				objectPos.point.x = objectPos.point.x + 0.015 + 0.01
				objectPos.point.z = objectPos.point.z + 0.005
			elif self.pickup_type == 'triangle':
				print("Hacked triangle")
				objectPos.point.x = objectPos.point.x + 0.015 + 0.01
				objectPos.point.z = objectPos.point.z + 0.01
			elif self.pickup_type == 'cube' and self.pickup_color == 'green':
				print("Hacked green cube")
				objectPos.point.x = objectPos.point.x + 0.02
			elif self.pickup_type == 'cube' and self.pickup_color == 'red':
				print("Hacked red cube")
				objectPos.point.x = objectPos.point.x + 0.025
			elif self.pickup_type == 'cube' and self.pickup_color == 'blue':
				print("Hacked blue cube")
				objectPos.point.x = objectPos.point.x + 0.01
			elif self.pickup_type == 'star':
				print("Hacked star")
				objectPos.point.x = objectPos.point.x + 0.015 + 0.01
				objectPos.point.z = objectPos.point.z + 0.01
			elif self.pickup_type == 'hollow cube':
				print("Hacked hollow cube")
				objectPos.point.x = objectPos.point.x + 0.025 + 0.01
				objectPos.point.z = objectPos.point.z + 0.01
			elif self.pickup_type == 'ball':
				print("Hacked ball")
				objectPos.point.x = objectPos.point.x + 0.02
				objectPos.point.z = objectPos.point.z - 0.01
			elif self.pickup_type == 'hollow cube':
				print("Hacked hollow cube")
				objectPos.point.x = objectPos.point.x - 0.01
			
			if num_tries == 1:
				objectPos.point.x = objectPos.point.x + 0.015
				print("Second try, trying further away")
			if num_tries == 2:
				objectPos.point.x = objectPos.point.x - 0.015
				print("Second try, trying further away")
			'''
			if self.pickup_type != 'booby':
				correct_x = ['cylinder', 'star', 'cross', 'triangle', 'hollow cube']
				print("Start hacking, pickup_type is {}".format(self.pickup_type))
				if self.pickup_type in correct_x:
					print("Hacked x")
					objectPos.point.x = objectPos.point.x + 0.01
			'''

			# Transform to arm frame
			pickupPos = self.transform_wheelToArm(objectPos)
			pickupPos_arm = pickupPos.point
		
			# Rescale to cm
			scale = 100.0 # meter to cm
			pickupPos_arm.x = pickupPos_arm.x*scale	
			pickupPos_arm.y = pickupPos_arm.y*scale
			pickupPos_arm.z = pickupPos_arm.z*scale

			aboveGoal_pos = Point(pickupPos_arm.x, pickupPos_arm.y, pickupPos_arm.z + 4.0) #correct 6, include in param file	
			
			
			aboveGoal_state = self.moveToPos_client(aboveGoal_pos, self.move_mode, self.moveDuration_abs, self.interpol_way)
			#print("Moved above goal")
			if aboveGoal_state.error == True:
				self.toInitPos()
				return False

			#print("Moved above goal", aboveGoal_state)
			
			# Correct position above object
			pickupPos_high_new = self.absoluteFromDifference(aboveGoal_pos, aboveGoal_state.position)

			'''
			aboveGoal_state = self.moveToPos_client(pickupPos_high_new, self.move_mode, self.moveDuration_abs, self.interpol_way)

			if aboveGoal_state.error == True:
				return False					
				break
			'''	
			
			# Modify goal pos to go to lower z
			pickupPos_arm_low = pickupPos_high_new
			pickupPos_arm_low.z = pickupPos_arm.z	#change when perception correct

			# Turn on pump
			self.pump_control(True)
			

			# Move down with corrected x,y
			atGoal_state = self.moveToPos_client(pickupPos_arm_low,self.move_mode, self.moveDuration_abs, self.interpol_way)
			if atGoal_state.error == True:
				self.pump_control(False)
				self.toInitPos()
				return False										

			# Move back up
			aboveGoal_pos.z = self.initPos_arm.z #aboveGoal_pos.z + 3.0
			aboveGoal_state = self.moveToPos_client(aboveGoal_pos, self.move_mode, self.moveDuration_abs, self.interpol_way)
			
			# Quit booby without check
			
			if object_type == 'booby':
				objectPos_new = self.booby_position
				print("ObjectPos_new is {} and pickupPos_wheelcenter is {}".format(objectPos_new, pickupPos_wheelcenter))
				if (objectPos_new.point.z-pickupPos_wheelcenter.z) > zTol:
					print("booby pickup returns true")
					return True
				else:
					self.pump_control(False)
					self.toInitPos()
					return False
			

			# Check if picked up	
			if object_type !='booby':
				objectPos_new = self.objectPos_client(request,select=False)
				if objectPos_new == False:
					objectPos_new = self.objectPos_client(request,select=False)
					if objectPos_new == False:
						print("No object seen twice after pickup")
						return True

				
				#print("Object is lifted by {} m".format(objectPos_new.point.z-pickupPos_wheelcenter.z))
				#print("PickupPos wheelcenter is".format(pickupPos_wheelcenter.z))
				if (objectPos_new.point.z-pickupPos_wheelcenter.z) > zTol:
					print("pickup returns true")
					return True
				
				
			num_tries += 1

			if num_tries == max_tries:
				self.pump_control(False)
				self.toInitPos()
				return False
			
	def booby_callback(self,data):
		if data.point.z < 0.01 or data == None:
			print("Data is none or z smaller than 0.01")
			self.booby_detected = False
		else:
			self.booby_position = data	
			self.booby_detected = True
		

	def motor_callback(self, data):
		
		self.recent_robot_rotations.append(data.angular.z)
		self.recent_robot_rotations = self.recent_robot_rotations[-300:]
 
		if sum(np.sign(self.recent_robot_rotations)) > 0.0:
			self.robot_rotation = -1.0
		else:
			self.robot_rotation = 1.0

if __name__ == "__main__":
	Manipulate()
	
	
	

