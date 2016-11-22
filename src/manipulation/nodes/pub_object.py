#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped, Point

from std_msgs.msg import Header
from manipulation.msg import Manipulation

objectPos  = PointStamped()
pub = rospy.Publisher('/mother/manipulation', Manipulation, queue_size=10)


def callback(data):

	# Publish some Manipulation message
	msg = Manipulation()

	header = Header()
	header.frame_id = 'wheel_center'
	#point = Point(0.20,-0.05,0.03) # in m

	msg.pickupPos = data

	msg.job = 'reposition'

	placePoint = Point(0.20,0.05,0.03)
	msg.placePos = PointStamped(header,placePoint)
	
	pub.publish(msg)
	#print("Point published {}".format(msg))


if __name__ == '__main__':

	rospy.init_node('talker', anonymous=True)
	
	rospy.Subscriber("/objectPos_wheelcenter", PointStamped, callback)

	
	#rate = rospy.Rate(1)
	i=0
	notInSight = False
	if notInSight:
		#while not rospy.is_shutdown():
		while i<50:
			newObject = PointStamped()
			newObject.header.frame_id = 'wheel_center'
			newObject.point = Point(0.20,0.05,0.03)
			callback(newObject)
			i+=1
		#rate.sleep()
	

	rospy.spin()
