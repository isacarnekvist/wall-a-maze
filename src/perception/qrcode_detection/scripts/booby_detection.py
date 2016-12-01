#!/usr/bin/env python

import sys
import rospy
import tf
import numpy as np

from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import Header, String


class BoobyDetection():

    def __init__(self):
        self.booby_publish = rospy.Publisher("perception/trap_position", PointStamped, queue_size=10)

        rospy.init_node('boobytrap_detection')

        rospy.Subscriber("visp_auto_tracker/object_position", PoseStamped, self.visp_callback)

        self.listener = tf.TransformListener()
        self.listener.waitForTransform("wheel_center", "camera", rospy.Time(), rospy.Duration(4.0))

        rospy.spin()


    def visp_callback(self,data):
    	
        if data.pose.position.z < 0.01:
            return

    	qr_pose_camera = data
    	qr_pose_camera.header.frame_id = 'camera'
    	qr_pose_wheel = self.transform_cameraToWheel(qr_pose_camera)

        # Should not happen?
        if qr_pose_wheel == None:
            return

        euler = tf.transformations.euler_from_quaternion([qr_pose_wheel.pose.orientation.x,qr_pose_wheel.pose.orientation.y,qr_pose_wheel.pose.orientation.z,qr_pose_wheel.pose.orientation.w])

    	# Caculate pickup location given QR code center
    	qr_pickup = PointStamped()
    	qr_pickup.header.frame_id = 'wheel_center'

    	dist_qr_to_top = 0.050
    	dist_qr_to_center = 0.050
    	qr_pickup.point.z = qr_pose_wheel.pose.position.z + dist_qr_to_top
        qr_pickup.point.x = qr_pose_wheel.pose.position.x + dist_qr_to_center*np.sin(euler[0])
        qr_pickup.point.y = qr_pose_wheel.pose.position.y + dist_qr_to_center*np.cos(euler[0])
        

    	print("The euler angles in wheel_center are {}".format(euler))
        print("The qr code position is at {}".format(qr_pose_wheel.pose.position))
        print("The pickup location is at {}".format(qr_pickup.point))
        
        self.booby_publish.publish(qr_pickup)


    def transform_cameraToWheel(self, data):
        try:	
            data_wheel = self.listener.transformPose('wheel_center',data)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "Failed to transform QR pose to wheel_center"
            return

        return data_wheel

if __name__ == "__main__":
    BoobyDetection()
