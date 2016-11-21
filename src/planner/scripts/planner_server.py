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
from geometry_msgs.msg import Polygon, PoseStamped, Twist, Point32

from grid import euler_path_plan, lines_to_grid


IDLE = 0
EXECUTING = 1
REPLANNING = 2
STATE_STR = {
    IDLE: 'idle',
    EXECUTING: 'executing',
    REPLANNING: 'replanning'
}

class StateMachine:
    
    def __init__(self):
        self._state = IDLE
        self._allowed_transitions = {
            IDLE: {EXECUTING, IDLE},
            EXECUTING: {EXECUTING, IDLE, REPLANNING},
            REPLANNING: {IDLE, EXECUTING}
        }

    @property
    def state(self):
        return self._state

    def transition_to(self, state):
        if state in self._allowed_transitions[self._state]:
            print('request for state transition from {} to {}'.format(STATE_STR[self._state], STATE_STR[state]))
            self._state = state
        else:
            planner.abort()
            raise ValueError('Illegal transition in planner!')


class PathBlockedException(Exception):
    pass


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
        self.state_machine = StateMachine()
        self.updated_obstacles = True
        self.scans = []
        self.x = 0.0
        self.y = 0.0
        self.wheels = rospy.Publisher('motor_controller', Twist, queue_size=10)
        self.grid_publisher = rospy.Publisher('occupancy_grid', OccupancyGrid, queue_size=10)
        self.path_publisher = rospy.Publisher('path', Path, queue_size=10)
        self.new_obstacle_publisher = rospy.Publisher('seen_obstacles', Polygon, queue_size=10)
        print('Started planner server')

    def target_callback(self, goal):
        self.abort()
        self.goal = goal
        if self.goal is None:
            # This is not supposed to happen
            self.state_machine.transition_to(IDLE)
            self.abort()
            raise ValueError('goal was None')
        if self.goal.is_abort_action:
            self.state_machine.transition_to(IDLE)
            print('target was canceled')
            self.abort()
            return

        print('self.goal: {}'.format(self.goal))

        self.state_machine.transition_to(EXECUTING)

        rate = rospy.Rate(100)

        # Convenience adjustments
        while not 0 <= self.goal.theta <= 2 * np.pi:
            self.goal.theta += -np.sign(self.goal.theta) * 2 * np.pi

        if self.grid is None or self.graph is None:
            self.abort()
            raise ValueError('Have not recieved occupancy grid!, ignoring request')
            self.state_machine.transition_to(IDLE)
            return

        print('requesting euler plan...')
        try:
            self.plan = euler_path_plan(self.x, self.y, self.goal.x, self.goal.y, self.grid, self.graph)
        except IndexError:
            print('Current position or goal outside occupancy grid?', file=sys.stderr)
            self.abort()
            self.state_machine.transition_to(IDLE)
            return

        print('executing euler plan')
        steps = len(self.plan)
        for step, (x, y) in enumerate(self.plan):
            if self.state_machine.state in {IDLE, REPLANNING}:
                break
            if step == steps - 1:
                final_rotation = self.goal.theta
            else:
                final_rotation = None
            try:
                for _ in self.line_iterator(x, y, final_rotation):
                    if self.state_machine.state != EXECUTING:
                        return
                    rate.sleep()
            except PathBlockedException as e:
                if self.state_machine.state == EXECUTING:
                    self.state_machine.transition_to(REPLANNING)
                else:
                    self.abort()
                    return
                aborted_goal = deepcopy(self.goal)
                self.abort()
                sleep(3.0)
                self.updated_obstacles = False
                self.publish_seen_obstacles()
                while not self.updated_obstacles:
                    sleep(0.1)
                if self.state_machine.state == REPLANNING:
                    self.target_callback(aborted_goal)
                    return
                else:
                    return
            self.plan = self.plan[1:]
        self.state_machine.transition_to(IDLE)
        self.abort()

    def path_clear(self, x, y):
        # Temporary!
        return True
        target_distance = np.linalg.norm(np.array([x - self.x, y - self.y]))
        w = 0.15
        for x, y in self.scans:
            if x < 0:
                continue

            # change to some inverse cone (pointy further away)
            if np.sqrt(min((1.0/2.0) * x, target_distance + w) ** 2 + y ** 2) < w:
                print('aborting due to laser measurement (robot frame):', x, y)
                print('spherical distance:', np.sqrt((x - 0.03) ** 2 + y ** 2))
                return False
        return True                    

    def publish_seen_obstacles(self):
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

    def line_iterator(self, x, y, final_rotation=None):

        # TODO rotate to face x, y
        target_theta = atan2(y - self.y, x - self.x)
        for _ in self.rotation_iterator(target_theta):
            yield
        self.stop()

        sleep(2.0)

        # Travel along a line to the partial target (x, y)
        original_position = np.array([self.x, self.y])
        target = np.array([x, y])
        distance = np.linalg.norm(target - original_position)
        #                    v distance traveled + 5 cm
        while distance > np.linalg.norm(np.array([self.x, self.y]) - original_position) + 0.06:
            self.correct_position(x, y)
            if not self.path_clear(x, y):
                raise PathBlockedException()
            yield
        self.stop()

        sleep(2.0)

        # TODO rotate to face x, y (if final_rotation)
        if final_rotation is not None:
            for _ in self.rotation_iterator(final_rotation):
                yield
            self.stop()
            sleep(2.0)

    def rotation_iterator(self, theta):
        theta_correction = closest_theta_adjustment(self.theta, theta)
        msg = Twist()
        msg.angular.z = 0.8 * np.sign(theta_correction)
        self.wheels.publish(msg)
        time_needed = theta_correction / abs(msg.angular.z)
        time_needed = abs(theta_correction / msg.angular.z)
        secs_needed = int(time_needed)
        err_correction = 1.20
        nsecs_needed = int((time_needed - secs_needed) * err_correction * 1e9)
        stop_time = rospy.Duration(secs_needed, nsecs_needed)
        start_time = rospy.Time.now()
        while rospy.Time.now() < start_time + stop_time:
            yield

    """For aborting a current goal and stopping the robot"""
    def abort(self):
        self.stop()
        self.plan = []
        self.goal = None

    """For just stopping the robot, not aborting a current goal"""
    def stop(self):
	msg = Twist()
        self.wheels.publish(msg)

    def correct_position(self, x, y):
        target_theta = atan2(y - self.y, x - self.x)
        theta_error = closest_theta_adjustment(self.theta, target_theta)
        msg = Twist()
        msg.linear.x = 0.18
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
        self.grid.expand_obstacles(0.21)
        self.graph = self.grid.to_graph()
        self.updated_obstacles = True

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
        try:
            self.grid_publisher.publish(og)
        except rospy.exceptions.ROSSerializationException:
            # Don't know why this happens, just return and be happy if it works later
            return

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
            x = np.cos(alpha) * dist + 0.08
            y = np.sin(alpha) * dist + 0.009
            new_scans.append((x, y))
        self.scans = new_scans

if __name__ == '__main__':
    rospy.init_node('planner')
    planner = Planner()
    rospy.Subscriber('obstacles', Polygon, planner.obstacles_callback)
    rospy.Subscriber('position', PoseStamped, planner.position_callback)
    rospy.Subscriber('planner', PlannerTarget, planner.target_callback)
    rospy.Subscriber('scan', LaserScan, planner.laser_callback)
    status_service = rospy.Service('planner_ready', PlannerStatus, planner.status_callback)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        planner.publish_visuals()
        rate.sleep()
