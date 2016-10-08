#!/usr/bin/env python

from math import cos, sin
from datetime import datetime

import rospy
from geometry_msgs.msg import PoseStamped, Twist

class Localization():

    def __init__(self):
        rospy.init_node('localization')

        self.pose = PoseStamped()
        self.pose.header.frame_id = 'map'

        self.position_pub = rospy.Publisher('/position', PoseStamped, queue_size=10)

        self.odometry_timestamp = None
        rospy.Subscriber('/odometry', Twist, self.odometry_callback)


        # Start of changing to kalman filtering
        # State estimation as measurements:
        self.x_measured = 0.0
        self.y_measured = 0.0
        self.theta_measured = 0.0

        # State:
        self.last_state_update = datetime.now()
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.Gamma = np.eye(666) # TBD, this is the covariance matrix of the state

        rate = rospy.Rate(125)
        while not rospy.is_shutdown():
            self.kalman_steps()
            self.publish_latest()
            rate.sleep()

    def kalman_steps(self):
        # Calculate next state given controls and previous (current) state
        # state += timedelta * control_velocities
        # Update covariance matrix with noise
        # Calculate kalman gain
        # Correction step
        # state = corrected state
        # Sigma = corrected Sigma
        pass

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
