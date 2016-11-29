#!/usr/bin/env python 
# -*- coding: utf-8 -*-
from __future__ import print_function

import sys
import argparse
from time import sleep

import tf
import rospy
import numpy as np
import actionlib
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import String
from sensor_msgs.msg import Image
from ras_msgs.msg import RAS_Evidence
#from planner.msg import PlannerTarget
from geometry_msgs.msg import PoseStamped, PointStamped, TransformStamped

from planner.msg import PlannerTargetGoal, PlannerTargetAction
from classifier.msg import Object as classifierObject

from manipulation.srv import *

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
        self.speaker = rospy.Publisher('espeak/string', String, queue_size=10)
        self.evidence = rospy.Publisher('/evidence', RAS_Evidence, queue_size=10)
        self.planner_client = actionlib.SimpleActionClient(
            'path_executor',
            PlannerTargetAction
        )
    #self.pickup_object = rospy.ServiceProxy('
        self.planner_client.wait_for_server()
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
            goal = PlannerTargetGoal()
            goal.x = x
            goal.y = y
            goal.theta = theta
            self.planner_client.send_goal(goal)
        while not rospy.is_shutdown():
            if(self.planner_client.wait_for_result()):
                return
            rate.sleep()        
        return

    def stop(self):
        self.planner_client.cancel_goal()

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
        
        #Robot frame!!!!!!!
        evidence.object_location.transform.translation.x = self.x + data.x
        evidence.object_location.transform.translation.y = self.y + data.y
        evidence.object_location.transform.translation.z = data.z
        
        self.evidence.publish(evidence)	

        pickupObject = PointStamped()
        pickupObject.point.x = data.x
        pickupObject.point.y = data.y
        pickupObject.point.z = data.z
        pickupObject.header.frame_id = 'wheel_center'

        color ='red'
        object_type = 'cube'

        self.pickup_client(pickupObject,color,object_type)
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


    def pickup_client(self,position, color, object_type):
        rospy.wait_for_service('manipulation/pickup_object',PickupObject)
        try:
            pickup_object = rospy.ServiceProxy('manipulation/pickup_object',PickupObject)
            response = pickup_object(position, color, object_type)
            return response
        except rospy.ServiceException, e:
            print("Pickup service could not be called")

    def carry_client(self):
        rospy.wait_for_service('manipulation/carry_object',CarryObject)
        try:
            carry_object = rospy.ServiceProxy('manipulation/carry_object',CarryObject)
            response = carry_object()
            return response
        except rospy.ServiceException, e:
            print("Carry service could not be called")

    
          
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

    rospy.init_node('motherbrain')
    mother = Mother()
    rospy.Subscriber('objectPos_wheelcenter2', classifierObject, mother.perception_callback)
    rospy.Subscriber('position', PoseStamped, mother.position_callback)
    #rospy.Subscriber('/imu/collision', String, mother.collision)

    x, y, theta = None, None, None
    if args.which == 'goto':
        x = args.x
        y = args.y
        theta = args.theta
        mother.run(x=x, y=y, theta=theta)
    elif args.which == 'explore':
        x = 2.1
        y = 0.2
        theta = 0
        mother.run(x=x, y=y, theta=theta)    
        x = 2.1
        y = 2.1
        theta = 1.57
        mother.run(x=x, y=y, theta=theta)
        x = 0.2
        y = 2.1
        theta = 1.57
        mother.run(x=x, y=y, theta=theta)
        x = 0.2
        y = 0.2
        theta = 3.14
        mother.run(x=x, y=y, theta=theta)
    elif args.which == 'stop':
        mother.stop()
    else:
        raise NotImplementedError('Only goto implemented atm')

    rospy.spin()
