#! /usr/bin/env python
import tf
import rospy
import numpy as np

from geometry_msgs.msg import Twist, PoseStamped

class Robot:

    def __init__(self):
        self.x = 0.25
        self.y = 0.25
        self.theta = 3.14
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.update_pose_time = None
        self.pose_publisher = rospy.Publisher('position', PoseStamped, queue_size=10)

    def run(self):
        rate = rospy.Rate(32)
        while not rospy.is_shutdown():
            self.update_pose()
            self.publish_pose()
            rate.sleep()

    def update_pose(self):
        if self.update_pose_time is None:
            self.update_pose_time = rospy.Time.now()
        delta_ns = (rospy.Time.now() - self.update_pose_time)
        delta_sec = delta_ns.to_nsec() / 1e9
        self.x += self.linear_velocity * np.cos(self.theta) * delta_sec
        self.y += self.linear_velocity * np.sin(self.theta) * delta_sec
        self.theta += (self.angular_velocity + 0.03 + np.random.randn() * 0.07) * delta_sec
        while self.theta < 0.0:
            self.theta += 2 * np.pi
        while self.theta > 2 * np.pi:
            self.theta -= 2 * np.pi
        self.update_pose_time = rospy.Time.now()
        
    def publish_pose(self):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        quat = tf.transformations.quaternion_from_euler(0, 0, self.theta)
        (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ) = quat
        self.pose_publisher.publish(pose)
    
    def motor_control_callback(self, data):
        self.linear_velocity = data.linear.x
        self.angular_velocity = data.angular.z

if __name__ == '__main__':
    robot = Robot()
    rospy.init_node('fake_planner')
    rospy.Subscriber('motor_controller', Twist, robot.motor_control_callback)
    robot.run()
