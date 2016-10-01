#!/usr/bin/env python

from math import pi

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from phidgets.msg import motor_encoder

LEFT_KEY = 'left'
RIGHT_KEY = 'right'
RIGHT_WHEEL_BIAS = 1.0
WHEEL_RADIUS = 0.036
ROBOT_RADIUS = 0.215 / 2
LINEAR_ALPHA = 32.0
TICKS_PER_REVOLUTION = 858.0
CURRENT_SPEED_KEY = 'current_speed_key'
CURRENT_PERCENTAGE_KEY = 'current_percentage_key'
LAST_TIMESTAMP_KEY = 'last_timestamp_key'


class Controller():

    def __init__(self):

        rospy.init_node('wheels_controller')

        self.left_wheel_pub = rospy.Publisher('/left_motor/cmd_vel', Float32, queue_size=10)
        self.right_wheel_pub = rospy.Publisher('/right_motor/cmd_vel', Float32, queue_size=10)

        inner_state_dict = {
            LAST_TIMESTAMP_KEY: None,
            CURRENT_SPEED_KEY: None,
            CURRENT_PERCENTAGE_KEY: 0.0,
        }
        self.state_dict = {
            LEFT_KEY: inner_state_dict.copy(),
            RIGHT_KEY: inner_state_dict.copy(),
        }

        # This listens to wanted directions like from keyop, some navigation program etc...
        rospy.Subscriber('motor_controller', Twist, self.controller_callback)

        # These listens to encoder feedback
        rospy.Subscriber('/left_motor/encoder', motor_encoder, self.encoder_callback_left_motor)
        rospy.Subscriber('/right_motor/encoder', motor_encoder, self.encoder_callback_right_motor)
        
        # Done!
        rospy.spin()

    def controller_callback(self, data):
        if (self.state_dict[LEFT_KEY][CURRENT_SPEED_KEY] is None or 
            self.state_dict[RIGHT_KEY][CURRENT_SPEED_KEY] is None):
            return
        target_linear_velocity = data.linear.x
        target_angular_velocity = data.angular.z
        current_speed_left = self.state_dict[LEFT_KEY][CURRENT_SPEED_KEY]
        current_speed_right = self.state_dict[RIGHT_KEY][CURRENT_SPEED_KEY]
        angle_speed_add = self.angular_to_speed(target_angular_velocity)
        rospy.loginfo(angle_speed_add)
        target_left = target_linear_velocity - angle_speed_add
        target_right = target_linear_velocity + angle_speed_add
        self.state_dict[LEFT_KEY][CURRENT_PERCENTAGE_KEY] += LINEAR_ALPHA * (
            target_left - current_speed_left
        )
        self.state_dict[RIGHT_KEY][CURRENT_PERCENTAGE_KEY] += LINEAR_ALPHA * (
            (-target_right - current_speed_right) * RIGHT_WHEEL_BIAS
        )
        rospy.loginfo('Wheel target speeds: left: {}, right: {}'.format(
            target_left, target_right
        ))
        if (target_linear_velocity ** 2 + target_angular_velocity ** 2 < 0.001):
            self.left_wheel_pub.publish(0.0)
            self.right_wheel_pub.publish(0.0)
        else:
			rospy.loginfo('Left motor: {}, Right motor: {}'.format(self.state_dict[LEFT_KEY][CURRENT_PERCENTAGE_KEY], self.state_dict[RIGHT_KEY][CURRENT_PERCENTAGE_KEY]))
			self.left_wheel_pub.publish(self.state_dict[LEFT_KEY][CURRENT_PERCENTAGE_KEY])
			self.right_wheel_pub.publish(self.state_dict[RIGHT_KEY][CURRENT_PERCENTAGE_KEY])

    def encoder_callback_left_motor(self, data):
        self.update_state(data, LEFT_KEY)

    def encoder_callback_right_motor(self, data):
        self.update_state(data, RIGHT_KEY)

    def update_state(self, data, which_wheel):
        if which_wheel not in {LEFT_KEY, RIGHT_KEY}:
            raise ValueError('argument error in update_state(...)')

        if self.state_dict[which_wheel][LAST_TIMESTAMP_KEY]:
            prev_time = self.state_dict[which_wheel][LAST_TIMESTAMP_KEY]
            time_delta = data.header.stamp - self.state_dict[which_wheel][LAST_TIMESTAMP_KEY]
            ticks = data.count_change
            speed = self.calculate_speed(time_delta, ticks)
            self.state_dict[which_wheel][CURRENT_SPEED_KEY] = speed

        self.state_dict[which_wheel][LAST_TIMESTAMP_KEY] = data.header.stamp

    def calculate_speed(self, time_delta, ticks):
        angle_since_last_time = 2.0 * pi * ticks / TICKS_PER_REVOLUTION
        seconds_delta = time_delta.secs + time_delta.nsecs / 1e9
        speed = WHEEL_RADIUS * angle_since_last_time / (time_delta.secs + time_delta.nsecs / 1e9)
        return speed

    def angular_to_speed(self, angular_speed):
        return angular_speed * ROBOT_RADIUS


if __name__ == '__main__':
    Controller()
