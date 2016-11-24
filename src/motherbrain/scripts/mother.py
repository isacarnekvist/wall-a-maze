#!/usr/bin/env python 
# -*- coding: utf-8 -*-
from __future__ import print_function

import sys
import argparse
from time import sleep

import tf
import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from ras_msgs.msg import RAS_Evidence
from planner.msg import PlannerTarget
from geometry_msgs.msg import PoseStamped, PointStamped, TransformStamped
from classifier.msg import Object as classifierObject
from manipulation.msg import Manipulation

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
        self.planner = rospy.Publisher('planner', PlannerTarget, queue_size=10)
        self.arm = rospy.Publisher('mother/manipulation', Manipulation, queue_size=10)
        self.speaker = rospy.Publisher('espeak/string', String, queue_size=10)
        self.evidence = rospy.Publisher('/evidence', RAS_Evidence, queue_size=10)
        self.x = None
        self.y = 0.0
        self.theta = 0.0
        self.possible_objects = []
        # Give ROS a little time to acknowledge publisher, especially for planner
        sleep(0.5)

    def run(self, x=None, y=None, theta=None):
        self.stop()
        object_target = None
        detected_object = None
        pickup_goal = None
        rate = rospy.Rate(10)
        if x is not None:
            last_target = PlannerTarget(x=x, y=y, theta=theta, is_abort_action=False)
            self.planner.publish(last_target)
        while not rospy.is_shutdown():
            rate.sleep()

    def stop(self):
        self.planner.publish(PlannerTarget(is_abort_action=True))

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
        self.stop()
        
        #RAS Evidence
        image=rospy.client.wait_for_message('/camera/rgb/image_raw',Image)	
        
        evidence = RAS_Evidence()
        evidence.group_number = 6
        evidence. image_evidence = image
        evidence.object_id = data.color + '_' + data.type 
        
        #Camera frame!!!!!!!
        evidence.object_location.transform.translation.x = data.x
        evidence.object_location.transform.translation.y = data.y
        evidence.object_location.transform.translation.z = data.z
        
        self.evidence.publish(evidence)	

        manipObject = Manipulation()
        manipObject.pickupPos.point.x = data.x
        manipObject.pickupPos.point.y = data.y
        manipObject.pickupPos.point.z = data.z
        manipObject.pickupPos.header.frame_id = 'wheel_center'
        
        manipObject.job = 'carryout'

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

#	def collision(data):
#		if(data=="Stop"):
#			mother.stop()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='MotherBrain™ (Run at your own risk)')
    subparsers = parser.add_subparsers()

    position_parser = subparsers.add_parser('goto', help='goto a position in the map')
    position_parser.add_argument('x', type=float, help='target x coordinate, in meters')
    position_parser.add_argument('y', type=float, help='target y coordinate, in meters')
    position_parser.add_argument('theta', type=float, help='target theta coordinate, in radians')
    position_parser.set_defaults(which='goto')

    explore_parser = subparsers.add_parser('explore', help='first part of competion, exploring and mapping')
    explore_parser.set_defaults(which='explore')

    score_parser = subparsers.add_parser('score', help='second part of competion, score points!')
    score_parser.set_defaults(which='score')

    stop_parser = subparsers.add_parser('stop', help='send stop request to planner')
    stop_parser.set_defaults(which='stop')

    args = parser.parse_args()

    mother = Mother()
    rospy.Subscriber('objectPos_wheelcenter2', classifierObject, mother.perception_callback)
    rospy.Subscriber('position', PoseStamped, mother.position_callback)
    #rospy.Subscriber('/imu/collision', String, mother.collision)
    rospy.init_node('motherbrain')

    x, y, theta = None, None, None
    if args.which == 'goto':
        x = args.x
        y = args.y
        theta = args.theta
        mother.run(x=x, y=y, theta=theta)
    elif args.which == 'stop':
        mother.stop()
    else:
        raise NotImplementedError('Only goto implemented atm')

    rospy.spin()
