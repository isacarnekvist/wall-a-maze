# /usr/bin/env python
from __future__ import print_function

from time import sleep
from math import atan2

import tf
import roslib
roslib.load_manifest('planner')
import rospy
import actionlib
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist

from planner.msg import LineTargetAction


def closest_theta_adjustment(current, target):
    if abs(current - target) < 2 * np.pi - abs(current - target):
        return target - current
    else:
        if current < target:
            return -(current + 2 * np.pi - target)
        else:
            return 2 * np.pi - current + target


class LineTargetServer:

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.server = actionlib.SimpleActionServer('line_target', LineTargetAction, self.execute, False)
        self.server.start()
        self.wheels = rospy.Publisher('motor_controller', Twist, queue_size=1)

    def execute(self, goal):

        start_theta = atan2(goal.y - self.y, goal.x - self.x)

        # Convenience adjustments
        while not 0 <= self.theta <= 2 * np.pi:
            self.theta += -np.sign(self.theta) * 2 * np.pi
        while not 0 <= goal.theta <= 2 * np.pi:
            goal.theta += -np.sign(goal.theta) * 2 * np.pi
        if start_theta < 0.0:
            start_theta += 2 * np.pi

        # Face the goal position
        self.correct_rotation(start_theta)
        sleep(0.5) # let pf converge

        # Move to the correct spot
        distance = np.sqrt((self.x - goal.x) ** 2 + (self.y - goal.y) ** 2)
        while distance > 0.1:
            print('distance to target:', distance)
            self.correct_position(goal.x, goal.y)
            distance = np.sqrt((self.x - goal.x) ** 2 + (self.y - goal.y) ** 2)
        self.wheels.publish(Twist()) # Stop
        sleep(0.5) # let pf converge

        # Rotate to final wanted rotation
        if not goal.ignore_end_rotation:
            self.correct_rotation(goal.theta)

        self.server.set_succeeded()

    def correct_position(self, x, y):
        target_theta = atan2(y - self.y, x - self.x)
        theta_error = closest_theta_adjustment(self.theta, target_theta)
        msg = Twist()
        msg.linear.x = 0.15
        msg.angular.z = theta_error * 1.5
        print('publishing:', msg.linear.x, msg.angular.z)
        self.wheels.publish(msg)
        sleep(0.5)

    def correct_rotation(self, theta):
        msg = Twist()
        theta_correction = closest_theta_adjustment(self.theta, theta)
        msg.angular.z = 1.00 * np.sign(theta_correction)
        time_needed = abs(theta_correction / msg.angular.z)
        self.wheels.publish(msg)
        sleep(time_needed * 0.9)
        self.wheels.publish(Twist()) # Stop
        sleep(0.5)

        # Converge and polish once
        msg = Twist()
        theta_correction = closest_theta_adjustment(self.theta, theta)
        msg.angular.z = 1.0 * np.sign(theta_correction)
        time_needed = abs(theta_correction / msg.angular.z)
        self.wheels.publish(msg)
        sleep(time_needed)
        self.wheels.publish(Twist()) # Stop


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
    rospy.init_node('line_target_server')
    server = LineTargetServer()
    rospy.Subscriber('/position', PoseStamped, server.position_callback)
    rospy.spin()
