#! /usr/bin/env python
from __future__ import print_function

import sys

import roslib
roslib.load_manifest('planner')
import rospy
import actionlib
import numpy as np

from planner.msg import LineTargetAction, LineTargetGoal

if __name__ == '__main__':
    rospy.init_node('planner_test_client')
    client = actionlib.SimpleActionClient('line_target', LineTargetAction)
    print('starting server...')
    client.wait_for_server()
    print('started')

    if len(sys.argv) == 4:
        goal = LineTargetGoal(x=float(sys.argv[1]), y=float(sys.argv[2]), theta=float(sys.argv[3]))
    else:
        goal = LineTargetGoal(x=2.25, y=2.0, theta=np.pi / 2)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(1.0))
