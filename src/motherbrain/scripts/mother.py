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
from geometry_msgs.msg import PoseStamped, PointStamped, TransformStamped, Point
from localization.srv import AddPickable, RemovePickable

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
        rospy.wait_for_service('/map/add_pickable')
        rospy.wait_for_service('/map/remove_pickable')        

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

    X=0
    Y=0
    T=0

    def __init__(self):
        self.speaker = rospy.Publisher('espeak/string', String, queue_size=10)
        self.evidence = rospy.Publisher('/evidence', RAS_Evidence, queue_size=10)
        self.planner_client = actionlib.SimpleActionClient(
            'path_executor',
            PlannerTargetAction
        )
        self.planner_client.wait_for_server()     
        self.x = None
        self.y = 0.0
        self.theta = 0.0
        self.possible_objects = []
        # Give ROS a little time to acknowledge publisher, especially for planner        
        sleep(0.5)

    def run(self, x=None, y=None, theta=None):
        self.stop()
        global X,Y,T        
        object_target = None
        detected_object = None
        pickup_goal = None
        rate = rospy.Rate(10)
        if x is not None:
            goal = PlannerTargetGoal()
            goal.x = x
            X=x
            goal.y = y
            Y=y            
            goal.theta = theta
            T=theta            
            self.planner_client.send_goal(goal)
        while not rospy.is_shutdown():
            if(self.planner_client.wait_for_result()):
                print(self.planner_client.get_result())
                return
            rate.sleep()        
        return

    def stop(self):
        self.planner_client.cancel_goal()
        return

    def approach(self,x=None, y=None, theta=None):
        global X,Y,Z        
        old_x=X
        old_y=Y
        old_theta=T
        print("going to objectwith x,y,theta:",x,y,theta)
        mother.run(x=x,y=y,theta=theta)
        print("going to previous target")
        sleep(1)
        mother.run(x=old_x,y=old_y,theta=old_theta)
        return

    def perception_callback(self, data):
        if self.x is None: # fix this properly!
            return
        do = DetectedObject(
            self.x + data.x * np.cos(self.theta) - data.y * np.sin(self.theta),
            self.y + data.x * np.sin(self.theta) + data.y * np.cos(self.theta),
            data.z,
            data.type,
            data.color
        )

        #Add pickable in the map
        try:
            add_pickable = rospy.ServiceProxy('/map/add_pickable',AddPickable)
            response = add_pickable(do.x,do.y)
            print("response:",response)
        except rospy.ServiceException, e:
            print("Add Pickable service could not be called")        
        for o in self.possible_objects:
            if o == do:
                o.x = do.x
                o.y = do.y
                o.z = do.z
                return

        # speak!
        self.speaker.publish('I see a {} {}'.format(data.color, data.type))

        #RAS Evidence
        image=rospy.client.wait_for_message('/camera/rgb/image_raw',Image)         
        evidence = RAS_Evidence()
        evidence.group_number = 6
        evidence. image_evidence = image
        evidence.object_id = data.color + '_' + data.type 
        
        #Robot frame??
        evidence.object_location.transform.translation.x = do.x
        evidence.object_location.transform.translation.y = do.y
        evidence.object_location.transform.translation.z = data.z        
        self.evidence.publish(evidence) 
        print ("I saw",data.color,data.type)

        theta=np.arctan((do.y-self.y)/(do.x-self.x))
        #Approach
        sleep(1)
        mother.approach(x=do.x, y=do.y, theta=theta)
        print("I came back here")

        # Call manipulation service
        pickup_location = PointStamped()
        pickup_location.point = Point(data.x,data.y,data.z)
        pickup_location.header.frame_id = 'wheel_center'

        object_type = data.type
        object_color = data.color

        isPickedUp = self.pickupObject_client(pickup_location,object_type,object_color)

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


    def pickupObject_client(self,position,object_type,object_color):
        rospy.wait_for_service('/manipulation/pickup_object')
        try:
            pickup_object = rospy.ServiceProxy('/manipulation/pickup_object',PickupObject)
            response = pickup_object(position,object_type,object_color)
            return response
        except rospy.ServiceException, e:
            print("PickupObject service could not be called")

    def placeObject_client(self,position):
        rospy.wait_for_service('/manipulation/place_object')
        try:
            place_object = rospy.ServiceProxy('/manipulation/place_object',PlaceObject)
            response = place_object(position)
            return response
        except rospy.ServiceException, e:
            print("PlaceObject service could not be called")

    def dropObject_client(self):
        rospy.wait_for_service('/manipulation/drop_object')
        try:
            drop_object = rospy.ServiceProxy('/manipulation/drop_object',DropObject)
            response = drop_object()
            return response
        except rospy.ServiceException, e:
            print("DropObject service could not be called")

    def carryObject_client(self):
        rospy.wait_for_service('/manipulation/carry_object')
        try:
            carry_object = rospy.ServiceProxy('/manipulation/carry_object',CarryObject)
            response = carry_object()
            return response
        except rospy.ServiceException, e:
            print("CarryObject service could not be called")
        
#   def collision(data):
#       if(data=="Stop"):
#           mother.stop()


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


#map_updated,Bool
#import os
#fn=os.environ.get('MAP_PATH')

