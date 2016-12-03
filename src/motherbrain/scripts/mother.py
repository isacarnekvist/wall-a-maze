#!/usr/bin/env python 
# -*- coding: utf-8 -*-
from __future__ import print_function

import os
import sys
import argparse
from time import sleep

import tf
import rospy
import numpy as np
import actionlib
from nav_msgs.msg import OccupancyGrid
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from ras_msgs.msg import RAS_Evidence
#from planner.msg import PlannerTarget
from geometry_msgs.msg import PoseStamped, PointStamped, TransformStamped, Point, PolygonStamped
from localization.srv import AddPickable, RemovePickable

from planner.srv import PathPlan
from classifier.msg import Object as classifierObject
from planner.msg import PlannerTargetGoal, PlannerTargetAction

from manipulation.srv import *

NON_VISITED = 0
VISITED = 1
BLOCKED = 2

REACHED_TARGET = 0
CANNOT_REACH = 1
PREEMPTED_OTHER = 2

PLANNER_DICT = {
    0: 'REACHED_TARGET',
    1: 'CANNOT_REACH',
    2: 'PREEMPTED_OTHER'
}


def euclidean(p1, p2):
    return np.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)


def path_length(path, x, y):
    plan = path.plan.points
    if len(plan) == 0:
        return np.inf
    dist = (x - plan[0].x) ** 2 + (y - plan[0].y) ** 2
    for i in range(1, len(plan)):
        dist += euclidean(plan[i-1], plan[i])
    return dist


NORMAL = 0
ROTATING = 1
CLASSIFYING = 2
CS_STATE_DICT = {
    NORMAL: 'NORMAL',
    ROTATING: 'ROTATING',
    CLASSIFYING: 'CLASSIFYING'
}

class ClassifyStateMachine:
    
    def __init__(self):
        self._state = NORMAL
        self.transitions = {
            NORMAL: [ROTATING],
            ROTATING: [CLASSIFYING],
            CLASSIFYING: [NORMAL]
        }
        self.deadline_to_state = (None, None) # a deadline to change state to 2nd element

    def transition_to(self, state):
        print('[mother] Transition from state {} to {}'.format(CS_STATE_DICT[self.state], CS_STATE_DICT[state]))
        if state in self.transitions[self.state]:
            self.state = state
        else:
            raise ValueError('Transition from {} to {} not allowed'.format(self.state, state))

    @property
    def state(self):
        return self._state


class Map:

    def __init__(self):
        map_path = os.environ.get('MAP_PATH')
        self.min_x = np.inf
        self.max_x = -np.inf
        self.min_y = np.inf
        self.max_y = -np.inf
        self.cell_width = 0.4
        self.starting_position_set_occupied = False;
        with open(map_path, 'r') as f:
            for line in f:
                if line[0] == '#':
                    continue
                x1, y1, x2, y2 = map(float, line.split())
                self.min_x = min(self.min_x, x1, x2)
                self.min_y = min(self.min_y, y1, y2)
                self.max_x = max(self.max_x, x1, x2)
                self.max_y = max(self.max_y, y1, y2)
        self.grid = np.zeros(
            ((self.max_x - self.min_x) / self.cell_width + 1,
             (self.max_y - self.min_y) / self.cell_width + 1),
            dtype=int
        )

    def get_state(self, x, y):
        x_ind, y_ind = self.coords_to_inds(x, y)
        return self.grid[x_ind, y_ind]

    def visit(self, x, y):
        x_ind, y_ind = self.coords_to_inds(x, y)
        starting_position_set_occupied = True
        if x_ind < self.grid.shape[0] and y_ind < self.grid.shape[1]:
            self.grid[x_ind, y_ind] = VISITED

    def block(self, x, y):
        x_ind, y_ind = self.coords_to_inds(x, y)
        if x_ind < self.grid.shape[0] and y_ind < self.grid.shape[1]:
            self.grid[x_ind, y_ind] = BLOCKED

    def furthest_free_euclidean(self, x, y):
        x_ind, y_ind = self.coords_to_inds(x, y)
        dists = []
        for i in range(self.grid.shape[0]):
            for j in range(self.grid.shape[1]):
                if self.grid[i, j] == NON_VISITED:
                    dists.append(((i - x_ind) ** 2 + (j - y_ind) ** 2, i, j))
        if len(dists):
            dist, i, j = sorted(dists)[-1]
            return (i + 0.5) * self.cell_width, (j + 0.5) * self.cell_width
        else:
            return None, None

    def closest_free_path(self, current_x, current_y):
        #           distance   x      y
        min_path = (np.inf, np.inf, np.inf)
        x_ind, y_ind = self.coords_to_inds(current_x, current_y)
        inds = [((x_ind - i) ** 2 + (y_ind - j) ** 2, i, j) for i in range(self.grid.shape[0]) for j in range(self.grid.shape[1])]
        for _, i, j in sorted(inds):
            if self.grid[i, j] != NON_VISITED:
                continue
            x = (i + 0.5) * self.cell_width
            y = (j + 0.5) * self.cell_width
            if np.sqrt((current_x - i * self.cell_width) ** 2 + (current_y - j * self.cell_width) ** 2) > min_path[0]:
                continue
            path_plan_srv = rospy.ServiceProxy('/path_plan', PathPlan)
            plan = path_plan_srv(x, y)
            min_path = min(min_path, (path_length(plan, current_x, current_y), x, y))
        
        if min_path[0] == np.inf:
            return None, None
        else:
            return min_path[1:]

    def coords_to_inds(self, x, y):
        return int(x / self.cell_width), int(y / self.cell_width)


class DetectedObject:

    def __init__(self, x, y, z, type_str=None, color_str=None):
        self.x = x
        self.y = y
        self.z = z
        self.position_confirmed = False
        self.type_str = type_str
        self.color_str = color_str
        self.map_id = None
        rospy.wait_for_service('/map/add_pickable')
        rospy.wait_for_service('/map/remove_pickable')        

    def position_equal(self, other):
        if abs(self.x - other.x) > 0.2:
            return False
        if abs(self.y - other.y) > 0.2:
            return False
        if abs(self.z - other.z) > 0.2:
            return False
        return True

    def __eq__(self, other):
        if other.type_str != self.type_str or other.color_str != self.color_str:
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
        self.start_time = rospy.Time.now()
        self.grid_publisher = rospy.Publisher('visited_grid', OccupancyGrid, queue_size=10)
        self.planner_client = actionlib.SimpleActionClient(
            'path_executor',
            PlannerTargetAction
        )
        self.map = Map()
        self.planner_client.wait_for_server()     
        self.time_back_needed = 0.0
        self.recieved_position = False
        self.x = None
        self.y = 0.0
        self.theta = 0.0
        self.possible_objects = []
        self.classify_state_machine = ClassifyStateMachine()
        # Give ROS a little time to acknowledge publisher, especially for planner        
        sleep(0.5)

    def goto(self, x=None, y=None, theta=None):
        rate = rospy.Rate(10)
        if x is not None:
            goal = PlannerTargetGoal()
            goal.cancel_action = False
            goal.x = x
            goal.y = y
            if theta is None:
                goal.theta = 0.0
                goal.ignore_target_theta = True
            else:
                goal.theta = theta
                goal.ignore_target_theta = False
            self.planner_client.send_goal(goal)
        while not rospy.is_shutdown():
            if self.planner_client.wait_for_result():
                break
            rate.sleep()
        # TODO how to differentiate between preempted and blocked?
        return self.planner_client.get_result()

    def explore(self):

        x_hell, y_hell = (
            self.map.max_x - 0.20,
            self.map.max_y - 0.20
        )

        print(self.map.get_state(0.25, 0.25))

        first_target = True
        rate = rospy.Rate(10)
        while not self.recieved_position and not rospy.is_shutdown():
            rate.sleep()
        
        while not rospy.is_shutdown():
            self.publish_visuals()            
            self.time_back_needed = self.calculate_time_back()
            #print("time needed to home:", self.time_back_needed)
            if self.map.get_state(x_hell, y_hell) == NON_VISITED:
                x, y = x_hell, y_hell
            else:
                x, y = self.map.closest_free_path(self.x, self.y)
            if x is None or y is None:
                print('No unexplored places!')
                break
            while (self.classify_state_machine.state in [ROTATING, CLASSIFYING]
                   and not rospy.is_shutdown()):
                rate.sleep()
            planner_res = self.goto(x, y).reached_target_state
            if planner_res == CANNOT_REACH:
                self.map.block(x, y)
            print('[mother] goto({}, {}) returned: {}'.format(x, y, PLANNER_DICT[planner_res]))

    def calculate_time_back(self):
        path_plan_srv = rospy.ServiceProxy('/path_plan', PathPlan)
        plan = path_plan_srv(0.25, 0.25)
        return path_length(plan, self.x, self.y) / 0.14 + len(plan.plan.points) * 4

    def publish_visuals(self):
        og = OccupancyGrid()
        og.header.frame_id = '/map'
        og.header.stamp = rospy.Time.now()
        og.info.resolution = self.map.cell_width
        og.info.width = self.map.grid.shape[0]
        og.info.height = self.map.grid.shape[1]
        og.info.origin.position.x = 0.0
        og.info.origin.position.y = 0.0
        og.data = 100 * self.map.grid.T.flatten() / 2
        self.grid_publisher.publish(og)

    def stop(self):
        goal = PlannerTargetGoal()
        goal.cancel_action = True
        self.planner_client.send_goal(goal)

    def perception_callback(self, data):

        if self.x is None: # we don't have a position of robot yet
            return

        do = DetectedObject(
            self.x + data.x * np.cos(self.theta) - data.y * np.sin(self.theta),
            self.y + data.x * np.sin(self.theta) + data.y * np.cos(self.theta),
            data.z,
            data.type,
            data.color
        )

        if self.classify_state_machine.state == ROTATING:
            return

        elif self.classify_state_machine.state == NORMAL:
            if not self.ignore_seen_pickable(do):
                self.stop()
                self.classify_state_machine.transition_to(ROTATING)
                self.pickup(do, False) # Change this bool depending on contest stage

                self.classify_state_machine.deadline_to_state = (rospy.Time.now() + rospy.Duration(1), NORMAL)
                self.classify_state_machine.transition_to(CLASSIFYING)
            else:
                return

        elif self.classify_state_machine.state == CLASSIFYING:
            if self.classify_state_machine.deadline_to_state[0] < rospy.Time.now():
                self.classify_state_machine.transition_to(self.classify_state_machine.deadline_to_state[1])
            elif self.is_new_object(do):
                self.acknowledge_new_pickable(do)
            return

        else:
            raise ValueError('[mother] unknown state in perception callback')
            return

    def battery_callback(self, data):

        if self.x is None: # we don't have a position of robot yet
            return

        do = DetectedObject(
            self.x + data.polygon.points[1].x + data.polygon.points[2].x,
            self.y + data.polygon.points[1].y + data.polygon.points[2].y,
            0
        )

        #if self.classify_state_machine.state == ROTATING:
        #    return

        #Add pickable in the map
        sleep(1) # let pf converge
        add_pickable = rospy.ServiceProxy('/map/add_pickable', AddPickable)
        do.map_id = add_pickable(do.x, do.y)
        sleep(1) # wait for map to be updated


    def acknowledge_new_pickable(self, do):
        print('[mother] adding', do.color_str, do.type_str)
        #Add pickable in the map
        sleep(1) # let pf converge
        add_pickable = rospy.ServiceProxy('/map/add_pickable', AddPickable)
        do.map_id = add_pickable(do.x, do.y)
        sleep(1) # wait for map to be updated

        # speak!
        self.speaker.publish('I see a {} {}'.format(do.color_str, do.type_str))

        #RAS Evidence
        image=rospy.client.wait_for_message('/camera/rgb/image_raw',Image)         
        evidence = RAS_Evidence()
        evidence.group_number = 6
        evidence. image_evidence = image
        evidence.object_id = do.color_str + '_' + do.type_str
        
        #Robot frame??
        evidence.object_location.transform.translation.x = do.x
        evidence.object_location.transform.translation.y = do.y
        evidence.object_location.transform.translation.z = do.z        
        self.evidence.publish(evidence) 
        print ("I saw", do.color_str, do.type_str, 'at', do.x, do.y, do.z)

        self.possible_objects.append(do)

    def pickup(self, do, do_pickup): # do means detected object
        # Call manipulation service
        pickup_location = PointStamped()
        pickup_location.point = Point(
            do.x, do.y, do.z
        )
        pickup_location.header.frame_id = 'wheel_center'

        isPickedUp = self.pickupObject_client(pickup_location, do.color_str, do.type_str, do_pickup)

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
        if not self.map.starting_position_set_occupied:
            self.map.visit(self.x, self.y)
        self.map.visit(self.x + np.cos(self.theta) * 0.15, self.y + np.sin(self.theta) * 0.15)
        self.recieved_position = True


    def pickupObject_client(self, position, object_type, object_color, do_pickup):
        rospy.wait_for_service('/manipulation/pickup_object')
        try:
            pickup_object = rospy.ServiceProxy('/manipulation/pickup_object', PickupObject)
            response = pickup_object(position, object_type, object_color, do_pickup)
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

    def is_new_object(self, do):
        # Object is in starting box
        if 0.0 < do.x < 0.4 and 0.0 < do.y < 0.4:
            return False

        # Check if already seen
        for o in self.possible_objects:
            if o == do: # it has the same position and identity as some other _pickable_ object
                return False
        return True

    def ignore_seen_pickable(self, do):
        # Object is in starting box
        if 0.0 < do.x < 0.4 and 0.0 < do.y < 0.4:
            return 

        # Check if already seen
        for o in self.possible_objects:
            if o.position_equal(do): # it has the same position as some other _pickable_ object
                return True

        return False

    def pickup_client(self, position, color, object_type):
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
    rospy.Subscriber('obstaclePos_wheelcenter', PolygonStamped, mother.battery_callback)
    rospy.Subscriber('position', PoseStamped, mother.position_callback)

    x, y, theta = None, None, None
    if args.which == 'goto':
        x = args.x
        y = args.y
        theta = args.theta
        mother.goto(x=x, y=y, theta=theta)
    elif args.which == 'explore':
        mother.explore()
    elif args.which == 'stop':
        mother.object_target = 'dummy ugly hack'
        mother.stop()
    else:
        raise NotImplementedError('Only goto implemented atm')

    rospy.spin()

#/obstaclePos_wheelcenter, geometry_msgs/polygonStamped
