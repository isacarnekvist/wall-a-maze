#! /usr/bin/env python
from __future__ import print_function

import sys

import tf
import roslib
roslib.load_manifest('planner')
import rospy
import numpy as np
from time import sleep
from math import atan2
from planner.msg import PlannerTarget
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
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.has_target = False
        self.wheels = rospy.Publisher('motor_controller', Twist, queue_size=1)
        print('Started planner server')

    def target_callback(self, goal):
        if goal.is_abort_action:
            print('target was canceled')
            self.has_target = False
            self.stop()
            return

        self.has_target = True
        rate = rospy.Rate(100)

        initial_rotation_done = False

        # Convenience adjustments
        while not 0 <= self.theta <= 2 * np.pi:
            self.theta += -np.sign(self.theta) * 2 * np.pi
        while not 0 <= goal.theta <= 2 * np.pi:
            goal.theta += -np.sign(goal.theta) * 2 * np.pi

        if self.grid is None or self.graph is None:
            self.has_target = False
            raise ValueError('Have not recieved occupancy grid!, ignoring request')
            return
        plan = euler_path_plan(self.x, self.y, goal.x, goal.y, self.grid, self.graph)
        if len(plan) == 1:
            current_target = self.line_iterator(*plan[0], final_rotation=goal.theta)
        plan = plan[1:]

        while self.has_target:
            try:
                next(current_target)
            except StopIteration:
                if plan:
                    if len(plan) == 1:
                        print('plan length 1, final rot:', goal.theta)
                        current_target = self.line_iterator(*plan[0], final_rotation=goal.theta)
                    else:
                        current_target = self.line_iterator(*plan[0])
                    plan = plan[1:]
                else:
                    self.has_target = False
                    break
            rate.sleep()
        self.stop()
        self.has_target = False

    def line_iterator(self, x, y, final_rotation=None):
        # Initial rotation
        start_theta = atan2(y - self.y, x - self.x)
        if start_theta < 0.0:
            start_theta += 2 * np.pi

        for _ in self.rotation_iterator(start_theta):
            yield
        self.stop()

        sleep(0.5)

        # Move along line
        while np.sqrt((x - self.x) ** 2 + (y - self.y) **2) > 0.075:
            self.correct_position(x, y)
            yield
        self.stop()

        sleep(0.5)

        if final_rotation is not None:
            for _ in self.rotation_iterator(final_rotation):
                yield
            self.stop()
            sleep(0.5)

    def rotation_iterator(self, theta):
        msg = Twist()
        theta_correction = closest_theta_adjustment(self.theta, theta)
        print('Correcting theta {} radians'.format(theta_correction))
        msg.angular.z = 1.00 * np.sign(theta_correction)
        time_needed = abs(theta_correction / msg.angular.z)
        secs_needed = int(time_needed)
        nsecs_needed = int((time_needed - secs_needed) * 1e9)
        stop_time = rospy.Duration(secs_needed, nsecs_needed)
        self.wheels.publish(msg)
        start_time = rospy.Time.now()
        while rospy.Time.now() < start_time + stop_time:
            yield

    def stop(self):
        self.wheels.publish(Twist())

    def correct_position(self, x, y):
        target_theta = atan2(y - self.y, x - self.x)
        theta_error = closest_theta_adjustment(self.theta, target_theta)
        msg = Twist()
        msg.linear.x = 0.15
        msg.angular.z = theta_error * 0.5 # will be rate dependent
        self.wheels.publish(msg)

    def obstacles_callback(self, data):
        print('Collecting obstacles')
        lines = []
        for i in range(len(data.points) / 2):
            p1 = data.points[2 * i]
            p2 = data.points[2 * i + 1]
            lines.append((p1.x, p1.y, p2.x, p2.y))
        self.grid = lines_to_grid(lines)
        self.grid.expand_obstacles(0.18)
        self.graph = self.grid.to_graph()

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


if __name__ == '__main__':
    rospy.init_node('planner')
    planner = Planner()
    rospy.Subscriber('obstacles', Polygon, planner.obstacles_callback)
    rospy.Subscriber('position', PoseStamped, planner.position_callback)
    rospy.Subscriber('planner', PlannerTarget, planner.target_callback)
    rospy.spin()
