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
from particle import Particle

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
    rotated = coords.dot(rotation_matrix)
    # Translation
    rotated[:, 0] += (-0.009)
    rotated[:, 1] += 0.08
    return rotated
    

class Localization():

    def __init__(self, n_particles=512, xlim=(0, 5), ylim=(0, 5), thetalim=(0, 2 * np.pi)):
        rospy.init_node('localization')

        self.pose = PoseStamped()
        self.pose.header.frame_id = 'map'
        self.map = Map(map_path='/home/ras26/catkin_ws/src/ras_maze/ras_maze_map/maps/lab_maze_2016.txt')

        self.particles = {
            Particle(
                np.random.rand() * (xlim[1] - xlim[0]) + xlim[0],
                np.random.rand() * (ylim[1] - ylim[0]) + ylim[0],
                np.random.rand() * (thetalim[1]  - thetalim[0]) + thetalim[0]
            )
            for _ in range(n_particles)
        }

        self.position_pub = rospy.Publisher('/position', PoseStamped, queue_size=10)

        self.odometry_timestamp = None
        self.particle_update_timestamp = None

        self.lidar_coords = np.zeros((360, 2))
        self.lidar_distances = np.zeros(360)
        self.listener = tf.TransformListener()
        self.laser_projector = LaserProjection()

        rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        rospy.Subscriber('/odometry', Twist, self.odometry_callback)

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

        rate = rospy.Rate(125)
        while not rospy.is_shutdown():
            self.particles_update()
            self.publish_latest()
            rate.sleep()

    def particles_update(self):
        if self.particle_update_timestamp is None:
            self.particle_update_timestamp = datetime.now()
            return
        # Calculate next state given controls and previous (current) state
        timedelta = datetime.now() - self.particle_update_timestamp
        delta = timedelta.seconds + timedelta.microseconds / 1e6

        # TODO Update particles
        for p in self.particles:
            p.update(self.x_delta, self.y_delta, self.theta_delta)
        # print('(mean, std) x: {}, {} \t y: {}, {} \t theta: {}, {}'.format(
        #     np.mean([p.x for p in self.particles]),
        #     np.std([p.x for p in self.particles]),
        #     np.mean([p.y for p in self.particles]),
        #     np.std([p.y for p in self.particles]),
        #     np.mean([p.theta for p in self.particles]),
        #     np.std([p.theta for p in self.particles])
        # ))
        self.x_delta = 0.0
        self.y_delta = 0.0
        self.theta_delta = 0.0

        self.particle_update_timestamp = datetime.now()

    def publish_latest(self):
        self.pose.pose.position.x = self.x
        self.pose.pose.position.y = self.y
        self.pose.pose.orientation.x = cos(self.theta / 2)
        self.pose.pose.orientation.y = sin(self.theta / 2)
        self.position_pub.publish(self.pose)

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
