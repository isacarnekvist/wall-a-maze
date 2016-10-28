#!/usr/bin/env python 

import roslib
import rospy
import math 
import numpy as np
import tf

roslib.load_manifest('manipulation')

from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from math import pi

trans = None
rot = None

objectPos_arm_pub = None

def object_callback(data):
	global trans, rot, objectPos_arm_pub
	print 'Wooop'
	
	if trans is None or rot is None:
		print 'Nope'
		return
		
	rot_euler = tf.transformations.euler_from_quaternion(rot)
	
	# output np array?

	# Maybe .transformPoint easier but dont know how to define from where to which frame

	r_posObject = np.array([(data.x),(data.y),(data.z)])
	print 'Object in robot coordinates {}' .format(r_posObject)
	
	# Do transform, Replace angle with rot_euler
	angle = 4.5*pi / 180.0
	Rotation_z = np.array([(math.cos(angle), math.sin(angle), 0),(-math.sin(angle),math.cos(angle),0),(0, 0, 1)])
	translation = np.transpose(trans)
	
	a_posObject = np.add(Rotation_z.dot(r_posObject), translation)
	
	# Do dirty corrections
	a_posObject[0] = a_posObject[0]+0.05
	a_posObject[1] = a_posObject[1]-0.02
	
	print 'Object in arm coordinates {}' .format(a_posObject)
	#print 'Translation is {}' .format(translation)

	
	
	objectPos_arm = Point(a_posObject[0],a_posObject[1],a_posObject[2])
	objectPos_arm_pub.publish(objectPos_arm)

if __name__ == '__main__':
	rospy.init_node('tf_object')
	
	rospy.Subscriber('/objectPos_wheelcenter', Point, object_callback)
	
	objectPos_arm_pub = rospy.Publisher('/objectPos_uarm', Point,queue_size=10)

	listener = tf.TransformListener()

	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		try:
			(trans,rot) = listener.lookupTransform('/wheel_center', '/uarm', rospy.Time(0))
			print 'Oh yeah!'
			print 'Translation is {}' .format(trans)
			print 'Rotation is {} ' .format(rot)
			break	# It will always be the same
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			print 'Darn it!'
			continue
			
		rate.sleep()
		
	rospy.spin()
