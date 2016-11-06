#! /usr/bin/env python
from __future__ import print_function

import sys

import roslib
roslib.load_manifest('planner')
import rospy
import actionlib
import numpy as np

from planner.msg import TargetAction, TargetGoal

if __name__ == '__main__':
    rospy.init_node('planner_test_node')
    client = actionlib.SimpleActionClient('planner', TargetAction)
    client.wait_for_server()

    if len(sys.argv) == 4:
        goal = TargetGoal(x=float(sys.argv[1]), y=float(sys.argv[2]), theta=float(sys.argv[3]))
    else:
        goal = TargetGoal(x=0.75, y=0.5, theta=np.pi / 2)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(1.0))
