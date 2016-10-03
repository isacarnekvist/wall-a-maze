#!/usr/bin/env python

from math import cos, sin
from datetime import datetime

import rospy
from phidgets.msg import motor_encoder
from geometry_msgs.msg import PoseStamped, Twist

class Localization():

    def __init__(self):
        rospy.init_node('localization')

        self.pose = PoseStamped()
        self.pose.header.frame_id = 'map'
        self.theta = 0.0 # Implicit in PoseStamped but easier to handle explicitly

        self.position_pub = rospy.Publisher('/position', PoseStamped, queue_size=10)

        self.odometry_timestamp = None
        rospy.Subscriber('/odometry', Twist, self.odometry_callback)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.publish_latest()
            rate.sleep()

    def publish_latest(self):
        self.position_pub.publish(self.pose)

    def odometry_callback(self, data):
        if self.odometry_timestamp is None:
            self.odometry_timestamp = datetime.now()
            return

        timedelta = datetime.now() - self.odometry_timestamp
        delta_seconds = timedelta.seconds + timedelta.microseconds / 1e6

        # Update position
        self.theta += delta_seconds * data.angular.z
        self.pose.pose.position.x += delta_seconds * data.linear.x * cos(self.theta)
        self.pose.pose.position.y += delta_seconds * data.linear.x * sin(self.theta)
        self.pose.pose.orientation.x = cos(self.theta / 2)
        self.pose.pose.orientation.y = sin(self.theta / 2)
        self.odometry_timestamp = datetime.now()


if __name__ == '__main__':
    Localization()
