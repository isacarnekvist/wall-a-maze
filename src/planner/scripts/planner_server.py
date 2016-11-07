#! /usr/bin/env python
from __future__ import print_function

import sys

import roslib
roslib.load_manifest('planner')
import rospy
import actionlib
import numpy as np
from geometry_msgs.msg import Polygon, PoseStamped

from grid import euler_path_plan, lines_to_grid
from planner.msg import TargetAction, TargetGoal, LineTargetAction, LineTargetGoal

class Planner:

    def __init__(self):
        self.server = actionlib.SimpleActionServer('planner', TargetAction, self.execute, False)
        self.server.start()
        self.line_client = actionlib.SimpleActionClient('line_target', LineTargetAction)
        self.line_client.wait_for_server()
        self.grid = None
        self.graph = None
        self.x = 0.0
        self.y = 0.0
        print('Started planner server')

    def execute(self, goal):
        if self.grid is None or self.graph is None:
            raise ValueError('Have not recieved occupancy grid!')
        print('Executing planner goal:')
        for x, y in euler_path_plan(self.x, self.y, goal.x, goal.y, self.grid, self.graph):
            print(x, y)
            partial_goal = LineTargetGoal()
            partial_goal.x = x
            partial_goal.y = y
            partial_goal.theta = 3.14
            partial_goal.ignore_end_rotation = True
            self.line_client.send_goal(partial_goal)
            self.line_client.wait_for_result()
        self.server.set_succeeded()

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


if __name__ == '__main__':
    rospy.init_node('planner')
    planner = Planner()
    rospy.Subscriber('obstacles', Polygon, planner.obstacles_callback)
    rospy.Subscriber('position', PoseStamped, planner.position_callback)
    rospy.spin()
