#!/usr/bin/env python

import sys
import rospy
import tf
import numpy as np

from math import pi
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import Header, String
from sensor_msgs.msg import LaserScan


class BoobyDetection():

    def __init__(self):
        self.booby_publish = rospy.Publisher("perception/trap_position", PointStamped, queue_size=10)

        rospy.init_node('boobytrap_detection')

        rospy.Subscriber("visp_auto_tracker/object_position", PoseStamped, self.visp_callback)
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        self.listener = tf.TransformListener()
        self.listener.waitForTransform("wheel_center", "camera", rospy.Time(), rospy.Duration(4.0))

        self.qr_position = PointStamped()

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

        # Hack first euler angle
        euler_corrected = euler[0]# + 40.0 * pi/180.0
    	qr_pickup.point.z = qr_pose_wheel.pose.position.z + dist_qr_to_top
        qr_pickup.point.x = qr_pose_wheel.pose.position.x - dist_qr_to_center*np.cos(euler_corrected)
        qr_pickup.point.y = qr_pose_wheel.pose.position.y + dist_qr_to_center*np.sin(euler_corrected)
        
        euler_degree = list(euler)

        for i,x in enumerate(euler):
            euler_degree[i] = x * 180.0 / pi

    	print("The first euler angle in wheel_center is {}".format(euler_corrected * 180.0 /pi))
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

    def laser_callback(self, scans):
        new_scans = []
        for angle in range(180, 360):
            alpha = np.pi * (angle + 88.5) / 180.0
            dist = scans.ranges[angle]
            if dist == np.inf:
                continue
            x = np.cos(alpha) * dist + 0.08
            y = np.sin(alpha) * dist + 0.009
            new_scans.append((x, y))
        self.scans = new_scans

if __name__ == "__main__":
    BoobyDetection()
