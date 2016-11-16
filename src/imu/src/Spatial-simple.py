#!/usr/bin/env python

#Basic imports
#from ctypes import *
import sys
import math

#Phidget specific imports
#from Phidgets.Phidget import Phidget
#from Phidgets.PhidgetException import PhidgetErrorCodes, PhidgetException
#from Phidgets.Events.Events import SpatialDataEventArgs, AttachEventArgs, DetachEventArgs, ErrorEventArgs
#from Phidgets.Devices.Spatial import Spatial, SpatialEventData, TimeSpan
#from Phidgets.Phidget import PhidgetLogLevel

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import String

linearx = None
lineary = None


def callback(data):
	global linearx, lineary	
	#goal_pos = Coords()
	#print("Input data", data.linear_acceleration)
	linearx = data.linear_acceleration.x
	lineary = data.linear_acceleration.y

if __name__ == '__main__':
	
	#linearx_prev = None
	#lineary_prev = None
	
	rospy.init_node('Spatial_listener')
	rospy.Subscriber('/imu/data', Imu, callback)
	
	#rospy.init_node('Spatial_talker')
	pub = rospy.Publisher('/imu/collision', String, queue_size=10)

	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		
		#print("the accs. are:", linearx, lineary)
	
		if linearx is None: #or lineary_prev is None:
			rate.sleep()
			#linearx_prev=linearx
			#lineary_prev=lineary
			continue
		if abs(linearx) > 2 or abs(lineary) > 2:		
			print ("We have a collision",linearx, lineary)
			pub.publish("Stop")
			#deg = math.degrees(math.tan(lineary/linearx))
			#print("Angle is ",deg)
		#linearx_prev=linearx
		#lineary_prev=lineary
		rate.sleep()
		
