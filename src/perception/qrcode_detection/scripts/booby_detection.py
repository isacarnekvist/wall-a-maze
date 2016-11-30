#!/usr/bin/env python

import sys
import rospy
import tf
import numpy as np

from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import Header, String


class BoobyDetection():

    def __init__(self):
        self.booby_publish = rospy.Publisher('perception/trap_position', PointStamped, queue_size=10)

        rospy.init_node('boobytrap_detection')

        rospy.Subscriber("visp_auto_tracker/object_position", PoseStamped, self.visp_callback)

        self.listener = tf.TransformListener()
        self.listener.waitForTransform("wheel_center", "camera", rospy.Time(), rospy.Duration(4.0))

        rospy.spin()


    def visp_callback(self,data):
    	
    	qr_pose_camera = data
    	qr_pose_camera.header.frame_id = 'wheel_center'
    	qr_pose_wheel = self.transform_cameraToWheel(qr_pose_camera)


    	euler = tf.transformations.euler_from_quaternion(qr_pose_wheel.pose.quaternion)

    	# Caculate pickup location given QR code center
    	qr_pickup = PointStamped()
    	qr_pickup.header.frame_id = 'wheel_center'

    	dist_qr_to_top = 0.065
    	dist_qr_to_center = 0.050
    	qr_pickup.point.z = qr_pose_wheel.pose.point.z + dist_qr_to_top

    	print("The euler angles in wheel_center are {}".format(euler))
    	self.booby_publish(qr_pickup)



	def transform_cameraToWheel(self, data):
		try:	
			data_wheel = self.listener.transformPose('wheel_center',data)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			print "Failed to transform QR pose to wheel_center"
			return

		return data_wheel

if __name__ == "__main__":
    BoobyDetection()
