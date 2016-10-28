#!/usr/bin/env python

import time
from math import cos, sin, atan2
from datetime import datetime

<<<<<<< HEAD
import numpy as np
import pandas as pd

=======
>>>>>>> master
import tf
import rospy
from laser_geometry import LaserProjection
from geometry_msgs.msg import PoseStamped, Twist, Point32
from sensor_msgs.msg import LaserScan, PointCloud

from map import Map
from particle import Particle, ParticleFilter

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

    def __init__(self, n_particles=8, xlim=(0, 3), ylim=(0, 3), thetalim=(0, 2 * np.pi)):
        rospy.init_node('localization')

        self.n_particles = n_particles
        self.pose = PoseStamped()
        self.pose.header.frame_id = 'map'
        self.map = Map(map_path='/home/ras26/catkin_ws/src/ras_maze/ras_maze_map/maps/lab_maze_2016.txt')

        self.particle_filter = ParticleFilter(
            n_particles=n_particles,
            xlim=xlim,
            ylim=ylim
        )

        self.position_pub = rospy.Publisher('/position', PoseStamped, queue_size=10)
        self.particles_pub = rospy.Publisher('/particles', PointCloud, queue_size=10)

        self.odometry_timestamp = None
        self.particle_update_timestamp = None

        self.lidar_coords = np.zeros((360, 2))
        self.lidar_distances = np.zeros(360)
        self.listener = tf.TransformListener()
        self.laser_projector = LaserProjection()

        rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        rospy.Subscriber('/odometry', Twist, self.odometry_callback)

        # keep track of change of state by odometry
        self.forward_delta = 0.0
        self.theta_delta = 0.0

        # State:
        self.last_state_update = datetime.now()
        self.x = 1
        self.y = 1
        self.theta = 0.0

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
        # TODO have this somewhere

        pc = PointCloud()
        pc.header.stamp = rospy.Time.now()
        pc.header.frame_id = 'map'
        for i, p in enumerate(self.particle_filter.particles):
            pc.points.append(Point32(p.x, p.y, 0))
        self.particles_pub.publish(pc)

        timedelta = datetime.now() - self.particle_update_timestamp
        delta = timedelta.seconds + timedelta.microseconds / 1e6

        # TODO Update particles
        self.particle_filter.update(self.forward_delta, self.theta_delta)
        self.particle_filter.resample(self.lidar_coords, self.map, self.n_particles)
        self.forward_delta = 0.0
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

    def odometry_callback(self, data):
        if self.odometry_timestamp is None:
            self.odometry_timestamp = datetime.now()
            return

        timedelta = datetime.now() - self.odometry_timestamp
<<<<<<< HEAD
=======
        delta_seconds = timedelta.seconds + timedelta.microseconds / 1e6

        # Update position
        self.theta += delta_seconds * data.angular.z
        self.pose.pose.position.x += delta_seconds * data.linear.x * cos(self.theta)
        self.pose.pose.position.y += delta_seconds * data.linear.x * sin(self.theta)
	orientation = tf.transformations.quaternion_from_euler(0, 0, self.theta)
	self.pose.pose.orientation.x = orientation[0]
	self.pose.pose.orientation.y = orientation[1]
	self.pose.pose.orientation.z = orientation[2]
	self.pose.pose.orientation.w = orientation[3]
>>>>>>> master
        self.odometry_timestamp = datetime.now()
        delta_seconds = timedelta.seconds + timedelta.microseconds / 1e6
        self.forward_delta += delta_seconds * data.linear.x
        self.theta_delta += delta_seconds * data.angular.z


if __name__ == '__main__':
    Localization()
