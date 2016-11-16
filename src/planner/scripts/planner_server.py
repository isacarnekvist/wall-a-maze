#! /usr/bin/env python
from __future__ import print_function

import sys
from copy import deepcopy

import tf
import roslib
roslib.load_manifest('planner')
import rospy
import numpy as np
from time import sleep
from math import atan2
from planner.srv import PlannerStatus
from planner.msg import PlannerTarget
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Polygon, PoseStamped, Twist

from grid import euler_path_plan, lines_to_grid


def closest_theta_adjustment(current, target):
    if abs(current - target) < 2 * np.pi - abs(current - target):
        return target - current
    else:
        if current < target:
            return -(current + 2 * np.pi - target)
        else:
            return 2 * np.pi - current + target


class Planner:

    def __init__(self):
        self.grid = None
        self.graph = None
        self.goal = None
        self.plan = None
        self.scans = []
        self.x = 0.0
        self.y = 0.0
        self.wheels = rospy.Publisher('motor_controller', Twist, queue_size=1)
        self.grid_publisher = rospy.Publisher('occupancy_grid', OccupancyGrid, queue_size=1)
        self.path_publisher = rospy.Publisher('path', Path, queue_size=1)
        print('Started planner server')

    def target_callback(self, goal):
        self.abort()
        self.goal = goal
        if self.goal.is_abort_action:
            print('target was canceled')
            self.abort()
            return

        rate = rospy.Rate(5)

        # Convenience adjustments
        while not 0 <= self.goal.theta <= 2 * np.pi:
            self.goal.theta += -np.sign(self.goal.theta) * 2 * np.pi

        if self.grid is None or self.graph is None:
            self.abort()
            raise ValueError('Have not recieved occupancy grid!, ignoring request')
            return

        print('requesting euler plan...')
        try:
            self.plan = euler_path_plan(self.x, self.y, self.goal.x, self.goal.y, self.grid, self.graph)
        except IndexError:
            print('Current position or goal outside occupancy grid?', file=sys.stderr)
            self.abort()
            return

        print('executing euler plan')
        steps = len(self.plan)
        for step, (x, y) in enumerate(self.plan):
            if self.goal is None: # while we have a goal / we were not aborted
                break
            print(x, y)
            for _ in self.line_iterator(x, y, step == steps - 1):
                rate.sleep()
            self.plan = self.plan[1:]
        self.abort()

    def path_clear(self, a, b, c, x, y, z):
        return True                    

    def line_iterator(self, x, y, final_rotation=None):

        # TODO rotate to face x, y

        # Travel along a line to the partial target (x, y)
        original_position = np.array([self.x, self.y])
        target = np.array([x, y])
        distance = np.linalg.norm(target - original_position)
        #                    v distance traveled + 5 cm
        print('original distance and current distance travelled:')
        print(distance, np.linalg.norm(np.array([self.x, self.y]) - original_position) + 0.05)
        while distance > np.linalg.norm(np.array([self.x, self.y]) - original_position) + 0.05:
            self.correct_position(x, y)
            if not self.path_clear(1, 2, 3, 4, 5, 6):
                # Actually it should go into some kind of mapping mode, wait a
                # while and then send obstacle coords on some topic
                break
            yield
        self.stop()

        # TODO rotate to face x, y (if final_rotation)

    def rotation_iterator(self, theta):
        msg = Twist()
        theta_correction = closest_theta_adjustment(self.theta, theta)
        print('Correcting theta {} radians'.format(theta_correction))
        msg.angular.z = 0.80 * np.sign(theta_correction)
        time_needed = abs(theta_correction / msg.angular.z)
        secs_needed = int(time_needed)
        nsecs_needed = int((time_needed - secs_needed) * 1e9)
        stop_time = rospy.Duration(secs_needed, nsecs_needed)
        self.wheels.publish(msg)
        start_time = rospy.Time.now()
        while rospy.Time.now() < start_time + stop_time:
            yield

    """For aborting a current goal and stopping the robot"""
    def abort(self):
        self.stop()
        self.goal = None

    """For just stopping the robot, not aborting a current goal"""
    def stop(self):
        self.wheels.publish(Twist())

    def correct_position(self, x, y):
        target_theta = atan2(y - self.y, x - self.x)
        theta_error = closest_theta_adjustment(self.theta, target_theta)
        msg = Twist()
        msg.linear.x = 0.18
        msg.angular.z = theta_error * 0.5 # will be rate dependent
        self.wheels.publish(msg)

    def obstacles_callback(self, data):
        print('Collecting obstacles')
        if self.goal:
            print('copying previous goal and recreating grid/map')
            previous_goal = deepcopy(self.goal)
        else:
            previous_goal = None
        self.abort()
        lines = []
        for i in range(len(data.points) / 2):
            p1 = data.points[2 * i]
            p2 = data.points[2 * i + 1]
            lines.append((p1.x, p1.y, p2.x, p2.y))
        self.grid = lines_to_grid(lines)
        self.grid.expand_obstacles(0.19)
        self.graph = self.grid.to_graph()
        if previous_goal is not None:
            print('re-sending previous target')
            self.target_callback(previous_goal)

    def position_callback(self, data):
        self.x = data.pose.position.x
        self.y = data.pose.position.y
        q = [
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w
        ]
        self.theta = tf.transformations.euler_from_quaternion(q)[-1] # roll
        # for convenience:
        while not 0 <= self.theta <= 2 * np.pi:
            self.theta += -np.sign(self.theta) * 2 * np.pi

    def status_callback(self, args):
        return True

    def publish_visuals(self):
        if self.grid is None:
            return
        og = OccupancyGrid()
        og.header.frame_id = '/map'
        og.header.stamp = rospy.Time.now()
        og.info.resolution = self.grid.cell_width
        og.info.width = self.grid.n_width
        og.info.height = self.grid.n_height
        og.info.origin.position.x = -self.grid.padding
        og.info.origin.position.y = -self.grid.padding
        og.data = 100 * self.grid._grid.flatten()
        self.grid_publisher.publish(og)

        path = Path()
        path.header.frame_id = '/map'
        if self.plan:
            for x, y in [(self.x, self.y)] + self.plan:
                pose = PoseStamped()
                pose.pose.position.x = x
                pose.pose.position.y = y
                path.poses.append(pose)
        self.path_publisher.publish(path)

    def laser_callback(self, scans):
        new_scans = []
        for angle in range(180, 360):
            alpha = np.pi * (angle + 88.5) / 180.0
            dist = scans.ranges[angle]
            if dist == np.inf:
                continue
            x = self.x + np.cos(alpha) * dist + 0.08
            y = self.y + np.sin(alpha) * dist + 0.009
            new_scans.append((x, y))
            if angle == 270:
                print(alpha, x, y)
        self.scans = new_scans

if __name__ == '__main__':
    rospy.init_node('planner')
    planner = Planner()
    rospy.Subscriber('obstacles', Polygon, planner.obstacles_callback)
    rospy.Subscriber('position', PoseStamped, planner.position_callback)
    rospy.Subscriber('planner', PlannerTarget, planner.target_callback)
    rospy.Subscriber('scan', LaserScan, planner.laser_callback)
    status_service = rospy.Service('planner_ready', PlannerStatus, planner.status_callback)
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        planner.publish_visuals()
        rate.sleep()
