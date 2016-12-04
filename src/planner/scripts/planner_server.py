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
from planner.srv import PlannerStatus, PathPlan, BlocksPositions
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Polygon, PoseStamped, Twist, Point32

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
        self.updated_obstacles = True
        self.scans = []
        self.lines = []
        self.x = 0.0
        self.y = 0.0
        self.wheels = rospy.Publisher('motor_controller', Twist, queue_size=10)
        self.grid_publisher = rospy.Publisher('occupancy_grid', OccupancyGrid, queue_size=10)
        self.new_obstacle_publisher = rospy.Publisher('seen_obstacles', Polygon, queue_size=10)
        self.map_updated_publisher = rospy.Publisher('map_updated', Bool, queue_size=10)
        print('Started planner server')


    def publish_seen_obstacles(self):
        # TODO This needs to be implemented as a service so that the
        # C++ planner can update this and then replan when the new obstacles
        # incorporated
        msg = Polygon()
        for x, y in self.scans:
            if -0.3 < y < 0.3 and 0 < x < 1.2:
                msg.points.append(
                    Point32(
                        x=self.x + x * np.cos(self.theta) - y * np.sin(self.theta),
                        y=self.y + x * np.sin(self.theta) + y * np.cos(self.theta),
                    )
                )
        self.new_obstacle_publisher.publish(msg)

    def obstacles_callback(self, data):
        print('Collecting obstacles')
        self.lines = []
        for i in range(len(data.points) / 2):
            p1 = data.points[2 * i]
            p2 = data.points[2 * i + 1]
            self.lines.append((p1.x, p1.y, p2.x, p2.y))
        self.grid = lines_to_grid(self.lines)
        self.grid.expand_obstacles(0.20)
        self.graph = self.grid.to_graph()
        self.updated_obstacles = True
        self.map_updated_publisher.publish(True)

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

    def path_plan_callback(self, args):
        plan = euler_path_plan(self.x, self.y, args.x, args.y, self.grid, self.graph)
        res = Polygon()
        for x, y in plan:
            res.points.append(Point32(x, y, 0.0))
        return res

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
        try:
            self.grid_publisher.publish(og)
        except rospy.exceptions.ROSSerializationException:
            # Don't know why this happens, just return and be happy if it works later
            return

    def blocks_positions_callback(self, args):
        res = Polygon()
        grid = lines_to_grid(self.lines + [(args.x - 0.03, args.y - 0.03, args.x + 0.03, args.y + 0.03)])
        grid.expand_obstacles(0.20)
        graph = grid.to_graph()
        blocked_with = set()
        blocked_without = set()
        for angle in np.linspace(0, 2 * np.pi, 4):
            check_x = args.x + np.cos(angle) * 0.25
            check_y = args.y + np.sin(angle) * 0.25
            plan_without = euler_path_plan(check_x, check_y, 0.25, 0.25, self.grid, self.graph)
            plan_with = euler_path_plan(check_x, check_y, 0.25, 0.25, grid, graph)
            if len(plan_with) == 0:
                blocked_with.add((check_x, check_y))
            if len(plan_without) == 0:
                blocked_without.add((check_x, check_y))
        for x, y in blocked_with - blocked_without:
            res.points.append(Point32(x, y, 0.0))
        return res

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

if __name__ == '__main__':
    rospy.init_node('planner')
    planner = Planner()
    rospy.Subscriber('obstacles', Polygon, planner.obstacles_callback)
    rospy.Subscriber('position', PoseStamped, planner.position_callback)
    status_service = rospy.Service('planner_ready', PlannerStatus, planner.status_callback)
    path_plan_service = None
    blocks_positions_service = rospy.Service('/planner/blocks_positions', BlocksPositions, planner.blocks_positions_callback)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if planner.graph is not None and path_plan_service is None:
            path_plan_service = rospy.Service('path_plan', PathPlan, planner.path_plan_callback)
        planner.publish_visuals()
        rate.sleep()
