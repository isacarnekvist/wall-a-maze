from time import sleep

import tf
import rospy
import numpy as np
from std_msgs.msg import String
from planner.msg import PlannerTarget
from geometry_msgs.msg import PoseStamped, PointStamped
from classifier.msg import Object as classifierObject


class DetectedObject:
    
    def __init__(self, x, y, z, type_str, color_str):
        self.x = x
        self.y = y
        self.z = z
        self.type_str = type_str
        self.color_str = color_str

    def __eq__(self, other):
        if self.type_str != other.type_str:
            return False
        if abs(self.x - other.x) > 0.1:
            return False
        if abs(self.y - other.y) > 0.1:
            return False
        if abs(self.z - other.z) > 0.1:
            return False
        return True

class Mother:

    def __init__(self):
        self.planner = rospy.Publisher('planner', PlannerTarget, queue_size=1)
        self.arm = rospy.Publisher('objectPos_wheelcenter', PointStamped, queue_size=1)
        self.speaker = rospy.Publisher('espeak/string', String, queue_size=1)
        self.x = None
        self.y = 0.0
        self.theta = 0.0
        self.possible_objects = []

    def run(self):
        object_target = None # the actual 
        detected_object = None
        pickup_goal = None
        rate = rospy.Rate(2)
        last_target = PlannerTarget(x=2.2, y=0.2, theta=3.14 / 2, is_abort_action=False)
        cancel = PlannerTarget(is_abort_action=True)
        self.planner.publish(last_target)
        while not rospy.is_shutdown():
            rate.sleep()

    def perception_callback(self, data):
        if self.x is None: # fix this properly!
            return
        do = DetectedObject(
            self.x + data.x,
            self.y + data.y,
            data.z,
            data.type,
            data.color
        )
        for o in self.possible_objects:
            if o == do:
                o.x = do.x
                o.y = do.y
                o.z = do.z
                return
        # speak!
        self.speaker.publish('I see a {} {}'.format(data.color, data.type))
        pickup = PointStamped()
        pickup.point.x = data.x
        pickup.point.y = data.y
        pickup.point.z = data.z
        pickup.header.frame_id = 'wheel_center'
        #self.arm.publish(pickup)
        self.possible_objects.append(do)
        print('length of po:', len(self.possible_objects))

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
    mother = Mother()
    rospy.Subscriber('objectPos_wheelcenter2', classifierObject, mother.perception_callback)
    rospy.Subscriber('position', PoseStamped, mother.position_callback)
    rospy.init_node('motherbrain')
    mother.run()
    rospy.spin()
