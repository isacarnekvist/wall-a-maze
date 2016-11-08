#! /usr/bin/env python

import roslib; roslib.load_manifest('manipulation')
import rospy
import actionlib

import manipulation.msg
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header


class pickup_client():

	def __init__(self):
		
		self.client = actionlib.SimpleActionClient('pickup', manipulation.msg.PickUpObjectAction)
		self.client.wait_for_server()
	
		rospy.Subscriber('/objectPos_wheelcenter', PointStamped, self.object_callback)
		
		# PickUpObject Goal message
		self.goal = manipulation.msg.PickUpObjectGoal()
		
		result = self.pickup()
		print "Pickup result is {}".format(result)
		
		
	def pickup(self):
		self.client.send_goal(self.goal)
		self.client.wait_for_result()
	
		return self.client.get_result()
	
	
	def object_callback(self, data):
		# Check if position wrt wheel_center
		# Something wrong DO LATER
		'''
		print "Getting goalpos {}".format(data.header.frame_id)
		if data.header.frame_id is not 'wheel_center':
			print "Object position not in wheel center"
			return
		'''
		
		self.goal = data
	
	
		
if __name__ == '__main__':
	rospy.init_node('pickup_client')
	
	pickup_client()
	
	rospy.spin()
		
