#!/usr/bin/env python

import sys
import rospy
import tf
import numpy as np

from math import pi
from geometry_msgs.msg import PoseStamped, PointStamped, Point
from std_msgs.msg import Header, String
from sensor_msgs.msg import LaserScan


class BoobyDetection():

    def __init__(self):
        self.booby_publish = rospy.Publisher("perception/trap_position", PointStamped, queue_size=10)

        rospy.init_node('boobytrap_detection')

        rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        self.listener = tf.TransformListener()
        self.listener.waitForTransform("wheel_center", "camera", rospy.Time(), rospy.Duration(4.0))

        self.booby_z = 0.12

        rospy.spin()


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

        yTol = 0.08
        x_min = 0.20
        x_max = 0.35
        x_sum = 0
        y_sum = 0
        num_scans = 0.0
        # Add filtering on x
        for x,y in new_scans:
            if np.abs(y) < yTol and x > x_min and x < x_max:
                print("Y is", y)
                print("X values are",x)
                x_sum += x
                y_sum += y
                num_scans += 1.0

        print("Number of scans is", num_scans)

        if num_scans > 0:
            booby_x = x_sum/num_scans
            booby_y = y_sum/num_scans   # Note: Might be causing errors due to zero crossing

            print("Pickup x position is", booby_x)
            print("Pickup y position is", booby_y)
            booby_pickup = PointStamped()
            booby_pickup.header.frame_id = 'wheel_center'
            booby_pickup.point = Point(booby_x+0.01,-booby_y,self.booby_z)
            self.booby_publish.publish(booby_pickup)



if __name__ == "__main__":
    BoobyDetection()
