#!/usr/bin/env python
import roslib
roslib.load_manifest('localization')
import rospy

import tf
from geometry_msgs.msg import PoseStamped

def robot_to_map(pose):
    br = tf.TransformBroadcaster()
    br.sendTransform(
       (pose.pose.position.x, pose.pose.position.y, 0),
       (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w),
       rospy.Time.now(),
       "wheel_center",
       "map"
    )
    rospy.loginfo(pose)

if __name__ == '__main__':
    rospy.init_node('robot_broadcaster')
    rospy.Subscriber(
        '/position',
        PoseStamped,
        robot_to_map
    )
    rospy.spin()
