#!/usr/bin/env python

from math import cos, sin
from datetime import datetime

import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped, Twist

from map import Map


class Localization():

    def __init__(self):
        rospy.init_node('localization')

        self.pose = PoseStamped()
        self.pose.header.frame_id = 'map'

        self.position_pub = rospy.Publisher('/position', PoseStamped, queue_size=10)

        self.odometry_timestamp = None
        self.kalman_timestamp = None
        rospy.Subscriber('/odometry', Twist, self.odometry_callback)
        rospy.Subscriber('/motor_controller', Twist, self.control_callback)
        # add subscriber to laser scans in robot frame:
        # TODO

        # Start of changing to kalman filtering
        # Store control signals to estimate mu 
        self.linear_control = 0.0
        self.angular_control = 0.0

        # State:
        self.last_state_update = datetime.now()
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.Gamma = np.eye(3) # Have velocities in state?

        rate = rospy.Rate(125)
        while not rospy.is_shutdown():
            self.kalman_update()
            self.publish_latest()
            rate.sleep()

    def kalman_update(self):
        if self.kalman_timestamp is None:
            self.kalman_timestamp = datetime.now()
            return
        # Calculate next state given controls and previous (current) state
        timedelta = datetime.now() - self.kalman_timestamp
        delta = timedelta.seconds + timedelta.microseconds / 1e6

        theta_bar = self.theta + delta * self.angular_control
        mu_bar = np.array([
            [self.x + delta * cos((self.theta + theta_bar) / 2) * self.linear_control],
            [self.y + delta * sin((self.theta + theta_bar) / 2) * self.linear_control],
            [theta_bar]
        ])

        # Update covariance matrix with noise
        self.Gamma += np.diag([0.01, 0.01, 0.01]) # TODO update and tweak with how much we actually trust this update, maybe as function of theta?

        # Calculate kalman gain
        # 1: Get estimated laser readings given state
        distances, dx, dy, dtheta = map.scan(*mu_bar.flatten())
        # 2: Get laser readings rotated in the same way
        # 3: Compared signals and pick suitable subset
        # 4: Build the jacobian and
        # 5: Calculate kalman gain

        # Correction step
        # state = corrected state
        # Sigma = corrected Sigma

        self.x, self.y, self.theta = mu_bar.flatten()
        rospy.loginfo('x: {} y: {} theta: {}'.format(self.x, self.y, self.theta))

        self.kalman_timestamp = datetime.now()

    def publish_latest(self):
        self.pose.pose.position.x = self.x
        self.pose.pose.position.y = self.y
        self.pose.pose.orientation.x = cos(self.theta / 2)
        self.pose.pose.orientation.y = sin(self.theta / 2)
        self.position_pub.publish(self.pose)

    def control_callback(self, data):
        self.linear_control = data.linear.x
        self.angular_control = data.angular.z

    def odometry_callback(self, data):
        if self.odometry_timestamp is None:
            self.odometry_timestamp = datetime.now()
            return

        timedelta = datetime.now() - self.odometry_timestamp
        delta_seconds = timedelta.seconds + timedelta.microseconds / 1e6

        # Implement?

        self.odometry_timestamp = datetime.now()

if __name__ == '__main__':
    Localization()
