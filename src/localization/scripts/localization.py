#!/usr/bin/env python

import time
from math import cos, sin
from datetime import datetime

import numpy as np
import pandas as pd

import tf
import rospy
from laser_geometry import LaserProjection
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan, PointCloud

from map import Map

N_LIDAR_MEASUREMENTS = 20


def laserscan_to_coords(scan):
    """Now the laser is transformed here due to lacking functionality in rospy"""
    angles = np.arange(0, 360)
    coords = np.array(map(
        lambda x: [x[1] * np.cos(np.pi * x[0] / 180), x[1] * np.sin(np.pi * x[0] / 180)], zip(angles, scan.ranges)
    ))

    theta = 181.5 * np.pi / 180
    rotation_matrix = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta),  np.cos(theta)]
    ])
    return coords.dot(rotation_matrix)
    

class Localization():

    def __init__(self):
        rospy.init_node('localization')

        self.pose = PoseStamped()
        self.pose.header.frame_id = 'map'
        self.map = Map(map_path='/home/ras26/catkin_ws/src/ras_maze/ras_maze_map/maps/lab_maze_2016.txt')

        self.position_pub = rospy.Publisher('/position', PoseStamped, queue_size=10)

        self.odometry_timestamp = None
        self.kalman_timestamp = None

        self.lidar_coords = np.zeros((360, 2))
        self.lidar_distances = np.zeros(360)
        self.listener = tf.TransformListener()
        self.laser_projector = LaserProjection()

        rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        rospy.Subscriber('/odometry', Twist, self.odometry_callback)
        rospy.Subscriber('/motor_controller', Twist, self.control_callback)

        # Start of changing to kalman filtering
        # Store control signals to estimate mu 
        self.linear_control = 0.0
        self.angular_control = 0.0

        # keep track of change of state by odometry
        self.x_delta = 0.0
        self.y_delta = 0.0
        self.theta_delta = 0.0

        # State:
        self.last_state_update = datetime.now()
        self.x = 0.2
        self.y = 0.2
        self.theta = 0.0
        self.Sigma = 0.01 * np.eye(3)

        self.jacobian = np.zeros((N_LIDAR_MEASUREMENTS, 3))
        self.kalman_gain = np.zeros((3, 3))

        rate = rospy.Rate(125)
        while not rospy.is_shutdown():
            self.kalman_update()
            self.publish_latest()
            rate.sleep()

    def kalman_update(self):
        return
        if self.kalman_timestamp is None:
            self.kalman_timestamp = datetime.now()
            return
        # Calculate next state given controls and previous (current) state
        timedelta = datetime.now() - self.kalman_timestamp
        delta = timedelta.seconds + timedelta.microseconds / 1e6

        mu_bar = np.array([
            [self.x + self.x_delta],
            [self.y + self.y_delta],
            [self.theta + self.theta_delta]
        ])
        self.x_delta = 0.0
        self.y_delta = 0.0
        self.theta_delta = 0.0

        # Update covariance matrix with noise
        # TODO update and tweak with how much we actually trust this update, maybe as function of theta?
        self.Sigma = (
            0.01 * np.eye(3) +
            500 * np.eye(3) * delta * (np.abs(self.linear_control) + np.abs(self.angular_control))
        )

        # Calculate kalman gain
        # 1: Get estimated laser readings given state
        distances, dx, dy, dtheta = self.map.scan(*mu_bar.flatten())
        distances[distances == np.inf] = np.nan
        # 2: Get laser readings rotated in the same way
        # done
        # 3: Compared signals and pick suitable subset
        diff = pd.Series(np.abs(self.lidar_distances - distances)).dropna()
        diff.sort()
        lidar_inds = diff.index[:N_LIDAR_MEASUREMENTS]
        diff = diff[:N_LIDAR_MEASUREMENTS]

        # 4: Build the jacobian
        self.jacobian[:, 0] = dx[lidar_inds]
        self.jacobian[:, 1] = dy[lidar_inds]
        self.jacobian[:, 2] = dtheta[lidar_inds]

        # 5: Calculate kalman gain
        self.kalman_gain = self.Sigma.dot(self.jacobian.T).dot(
            np.linalg.inv(
                self.jacobian.dot(self.Sigma).dot(self.jacobian.T) +
                0.1 * np.eye(N_LIDAR_MEASUREMENTS)
            ) # TODO tweak noise
        )

        # Correction step
        mu = mu_bar + self.kalman_gain.dot(diff.reshape((N_LIDAR_MEASUREMENTS, 1)))
        self.Sigma = (np.eye(3) + self.kalman_gain.dot(self.jacobian)).dot(self.Sigma)
        # state = corrected state
        # Sigma = corrected Sigma

        self.x, self.y, self.theta = mu.flatten()
        if self.theta > 2 * np.pi:
            self.theta -= 2 * np.pi
        if self.theta < 0:
            self.theta += 2 * np.pi
        rospy.loginfo('Sigma total: {}'.format(np.abs(self.Sigma).sum()))
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

    def laser_callback(self, data):
        self.lidar_distances[:] = data.ranges
        self.lidar_distances[self.lidar_distances == np.inf] = np.nan
        self.lidar_coords = laserscan_to_coords(data)
        print(self.lidar_coords[90, :])

    def odometry_callback(self, data):
        if self.odometry_timestamp is None:
            self.odometry_timestamp = datetime.now()
            return

        timedelta = datetime.now() - self.odometry_timestamp
        self.odometry_timestamp = datetime.now()
        delta_seconds = timedelta.seconds + timedelta.microseconds / 1e6
        self.x_delta += delta_seconds * cos(self.theta) * data.linear.x
        self.y_delta += delta_seconds * sin(self.theta) * data.linear.x
        self.theta_delta += delta_seconds * data.angular.z

        # Implement?


if __name__ == '__main__':
    Localization()
