#!/usr/bin/env python

# All libraries needed to import 
# Import system library
import sys
import time
import rospy

# Import uarm for python library
#from UArmForPython.uarm_python import Uarm
import pyuarm

# Import messages type
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import UInt8
from std_msgs.msg import Int32
from uarm.msg import Angles
from uarm.msg import Coords
from uarm.msg import CoordsWithTime
from uarm.msg import CoordsWithTS4



	
#def get_current_pos():
	
	

# Maybe not even necessary for start
#def change_status(status):
#	pub = rospy.Publisher('/uarm_status', String, queue_size=10)
	
 #   rate = rospy.Rate(10) # 10hz
#  while not rospy.is_shutdown():
#        pub.publish(status) # need to look up what values for attach/detach -- 1 attach, 0 detach
#        rate.sleep()	
	
#def calc_next_pos(currentPos, desiredPos, nextPos, waypoint):
	# first move to end effector position above object
#	if waypoint == 1
#		nextPos.x = desiredPos.x
#		nextPos.y = desiredPos.y
#		nextPos.z = currentPos.z
	# then move down on object
#	else if waypoint == 2
#		nextPos.x = desiredPos.x
#		nextPos.y = desiredPos.y
#		nextPos.z = desiredPos.z
	

def move_to(position):
    	pub = rospy.Publisher('/move_to', Coords, queue_size=10)
        
    	rate = rospy.Rate(10) # 10hz
    	while not rospy.is_shutdown():
        	pub.publish(position)
        	rate.sleep()

if __name__ == '__main__':

	rospy.init_node('moveToGoal', anonymous=True)
	
	# Define position
	testCoord = Coords()
	testCoord.x = 20
	testCoord.y = -7
	testCoord.z = 15
	
    	try:
        	move_to(testCoord)
    	except rospy.ROSInterruptException:
        	pass
