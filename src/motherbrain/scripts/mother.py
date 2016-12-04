#!/usr/bin/env python 
# -*- coding: utf-8 -*-
from __future__ import print_function

import os
import sys
import pickle
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
from geometry_msgs.msg import PoseStamped, PointStamped, TransformStamped, Point, PolygonStamped, Twist
from localization.srv import AddPickable, RemovePickable, ResetMap

from planner.srv import PathPlan
from classifier.msg import Object as classifierObject
from planner.msg import PlannerTargetGoal, PlannerTargetAction

from manipulation.srv import *

EXPLORED_OBJECTS_FN = '/home/ras36/catkin_ws/explored_objects.pkl'

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

SCORE_DICT = {
    'ball': 10000,
    'cross': 1000,
    'star': 1000,
    'hollow cube': 5000,
    'cube': 100,
    'triangle': 100,
    'cylinder': 100,
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
BLOCKING = 3
BATTERY_CONVERGING = 4
BATTERY_CLASSIFYING = 5
CS_STATE_DICT = {
    NORMAL: 'NORMAL',
    ROTATING: 'ROTATING',
    CLASSIFYING: 'CLASSIFYING',
    BLOCKING: 'BLOCKING',
    BATTERY_CONVERGING: 'BATTERY_CONVERGING',
    BATTERY_CLASSIFYING: 'BATTERY_CLASSIFYING',
}

class ClassifyStateMachine:
    
    def __init__(self):
        self._state = NORMAL
        self.transitions = {
            NORMAL: [ROTATING, BLOCKING],
            ROTATING: [CLASSIFYING, BLOCKING],
            CLASSIFYING: [NORMAL, BLOCKING]
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
        self.robot_x = None
        self.robot_y = None
        self.priority_score = 0.0
        self.position_confirmed = False
        self.type_str = type_str
        self.color_str = color_str
        self.expected_pickup_time = None
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
    
    def pickup_pose(self):
        theta = np.atan2(self.x - self.robot_x, self.y - self.robot_y)
        while theta < 0.0:
            theta += 2 * np.pi
        return self.robot_x, self.robot_y, theta

class Battery:

    def __init__(self, x1, y1, x2, y2):
        self.min_x = min(x1, x2)
        self.max_x = max(x1, x2)
        self.min_y = min(y1, y2)
        self.max_y = max(y1, y2)

    def __eq__(self, other):
        if other.min_x < self.min_x - 0.03:
            return False
        if other.max_x > self.max_x + 0.03:
            return False
        if other.min_y < self.min_y - 0.03:
            return False
        if other.max_y > self.max_y + 0.03:
            return False
        return True
        


class Mother:

    def __init__(self):
        self.speaker = rospy.Publisher('espeak/string', String, queue_size=10)
        self.evidence = rospy.Publisher('/evidence', RAS_Evidence, queue_size=10)
        self.start_time = rospy.Time.now()
        self.grid_publisher = rospy.Publisher('visited_grid', OccupancyGrid, queue_size=10)
        self.motor_publisher = rospy.Publisher('motor_controller', Twist, queue_size=10)
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
        self.pickables = []
        self.batteries = []
        self.classify_state_machine = ClassifyStateMachine()
        # Give ROS a little time to acknowledge publisher, especially for planner        
        sleep(0.5)

    def goto(self, x=None, y=None, theta=None):
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
            yield
            if self.planner_client.wait_for_result(rospy.Duration(5.0)):
                break
        # TODO how to differentiate between preempted and blocked?
        yield self.planner_client.get_result()

    def score(self):

        # ignore seen object at this stage
        self.classify_state_machine.transition_to(BLOCKING)

        with open(EXPLORED_OBJECTS_FN, 'rb') as f:
            self.pickables, self.batteries = pickle.load(f)
        for do in self.pickables:
            do.expected_pickup_time = 2 * self.calculate_travel_time(do.robot_x, do.robot_y)
            do.priority_score = SCORE_DICT[do.type_str] / do.expected_pickup_time

        deadline = rospy.Time.now() + rospy.Duration(3 * 60)

        ordering = sorted(self.pickables, key=lambda x: -x.priority_score)
        while rospy.Time.now() < deadline and ordering:
            current_object = ordering[0]
            ordering = ordering[1:]
            for planner_res in self.goto(*current_object.pickup_pose()):
                if planner_res is not None:
                    break
                if rospy.Time.now() < deadline:
                    self.time_out(contest_stage=1)
                    exit(0)
            if planner_res.reached_target_state != REACHED_TARGET:
                continue
            # pickup
            for i in range(2):
                pickup_success = self.pickup(do, True) # Change this bool depending on contest stage
                if pickup_success:
                    break
            if not pickup_success:
                continue
            self.carryObject_client(do, True) # Change this bool depending on contest stage
            # go home
            for planner_res in self.goto(0.25, 0.25):
                if planner_res is not None:
                    break
                if rospy.Time.now() < deadline:
                    self.time_out(contest_stage=1)
                    exit(0)
            # drop
            self.dropObject_client()
            
        self.time_out(contest_stage=1)

    def explore(self):

        print('[mother] explore time is set to 3 minutes, should be 5!')
        deadline = rospy.Time.now() + rospy.Duration(3 * 60) # change to 5 minutes!

        x_hell, y_hell = (
            self.map.max_x - 0.20,
            self.map.max_y - 0.20
        )

        rate = rospy.Rate(10)
        while not self.recieved_position and not rospy.is_shutdown():
            rate.sleep()
        
        while not rospy.is_shutdown():
            if self.map.get_state(x_hell, y_hell) == NON_VISITED:
                x, y = x_hell, y_hell
            else:
                x, y = self.map.closest_free_path(self.x, self.y)
            if x is None or y is None:
                print('No unexplored places!')
                # pick up something?
                # return to base!
                break
            while (self.classify_state_machine.state not in [NORMAL, BLOCKING]
                   and not rospy.is_shutdown()):
                rate.sleep()
            for planner_res in self.goto(x, y):
                self.publish_visuals()            
                self.time_back_needed = self.calculate_time_back()
                if self.time_back_needed == np.inf:
                    self.stop()
                    print('[mother] infinity time to return (no way back), reset map as a desperate measure')
                    rospy.ServiceProxy('/map/reset_map', ResetMap)()
                    for do in self.pickables:
                        add_pickable = rospy.ServiceProxy('/map/add_pickable', AddPickable)
                        do.map_id = add_pickable(do.x, do.y)
                    sleep(1)
                    break
                print("turn back in:", deadline - (rospy.Time.now() + rospy.Duration(self.time_back_needed)))
                if deadline - rospy.Time.now() < rospy.Duration(0):
                    self.time_out(contest_stage=0)
                    exit(0)
                if planner_res is not None:
                    break
            if planner_res.reached_target_state == CANNOT_REACH:
                self.map.block(x, y)
            print('[mother] goto({}, {}) returned: {}'.format(x, y, PLANNER_DICT[planner_res.reached_target_state]))

    def time_out(self, contest_stage):
        if contest_stage == 0:
            with open(EXPLORED_OBJECTS_FN, 'wb') as f:
                pickle.dump((self.pickables, self.batteries))
            self.stop()
        if contest_stage == 1:
            self.stop()

    def calculate_time_back(self):
        return self.calculate_travel_time(0.25, 0.25)

    def calculate_travel_time(self, to_x, to_y):
        path_plan_srv = rospy.ServiceProxy('/path_plan', PathPlan)
        plan = path_plan_srv(to_x, to_y)
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

        if self.classify_state_machine.state not in [NORMAL, ROTATING, CLASSIFYING]:
            return

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
                #self.backup()
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

    def backup(self):
        # the idea is that once seeing an object, it might be a good
        # idea to back up a little bit since we cannot stop immediately
        # when spotting an object
        print('trying to back up')
        back = Twist()
        back.linear.x = -0.1
        back.angular.z = 0.0
        rate = rospy.Rate(32)
        deadline = rospy.Time.now() + rospy.Duration(1)
        while rospy.Time.now() < deadline:
            self.motor_publisher.publish(back)
            rate.sleep()
        self.motor_publisher.publish(Twist())

    def dummy(self):
        class MockMotorPub:
            def publish(self, x):
                pass
        self.backup = lambda: None
        self.pickup = lambda fuck, you: None
        self.motor_publisher = MockMotorPub()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()

    def battery_callback(self, data):
        if self.x is None: # we don't have a position of robot yet
            return
        if self.classify_state_machine.state not in [NORMAL, BATTERY_CONVERGING, BATTERY_CLASSIFYING]:
            return
        state = self.classify_state_machine.state
        p1, p2 = data.polygon.points
        battery = Battery(p1.x, p1.y, p2.x, p2.y)
        do1 = DetectedObject(
            self.x + p1.x * np.cos(self.theta) - p1.y * np.sin(self.theta),
            self.y + p1.x * np.sin(self.theta) + p1.y * np.cos(self.theta),
            p1.z
        )
        do2 = DetectedObject(
            self.x + p2.x * np.cos(self.theta) - p2.y * np.sin(self.theta),
            self.y + p2.x * np.sin(self.theta) + p2.y * np.cos(self.theta),
            p2.z
        )
        if state == NORMAL:
            self.stop()
            for o in self.batteries:
                if o == do1 or o == do2: # it has the same position as some other battery
                return
            self.classify_state_machine.deadline_to_state = (rospy.Time.now() + rospy.Duration(1), NORMAL)
            self.classify_state_machine.transition_to(BATTERY_CONVERGING)
            pass
        elif state == BATTERY_CONVERGING:
            sleep(1)   	
            pass
        elif state == BATTERY_CLASSIFYING:
            self.acknowledge_new_pickable(do1)
            self.acknowledge_new_pickable(do2)            
            return
        else:
            raise ValueError('[mother] unknown state in perception callback')
            return

    def acknowledge_new_pickable(self, do):
        if do.type_str == 'object':
            print('[mother] ignoring', do.color_str, do.type_str)
            return
        if do.type_str == None:
            add_pickable = rospy.ServiceProxy('/map/add_pickable', AddPickable)
            do.map_id = add_pickable(do.x, do.y)
            sleep(1) # wait for map to be updated        
            self.batteries.append(do)
            return

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
        if do.color_str == 'orange' and do.type_str == 'star':
            evidence.object_id = RAS_EVIDENCE.patric
        elif do.color_str == 'red' and do.type_str == 'cube':
            evidence.object_id = RAS_EVIDENCE.red_cube
        elif do.color_str == 'red' and do.type_str == 'hollow cube':
            evidence.object_id = RAS_EVIDENCE.red_hollow_cube
        elif do.color_str == 'red' and do.type_str == 'ball':
            evidence.object_id = RAS_EVIDENCE.red_ball
        elif do.color_str == 'green' and do.type_str == 'cube':
            evidence.object_id = RAS_EVIDENCE.green_cube
        elif do.color_str == 'green' and do.type_str == 'cylinder':
            evidence.object_id = RAS_EVIDENCE.green_cylinder
        elif do.color_str == 'blue' and do.type_str == 'cube':
            evidence.object_id = RAS_EVIDENCE.blue_cube
        elif do.color_str == 'blue' and do.type_str == 'triangle':
            evidence.object_id = RAS_EVIDENCE.blue_triangle
        elif do.color_str == 'yellow' and do.type_str == 'cube':
            evidence.object_id = RAS_EVIDENCE.yellow_cube
        elif do.color_str == 'yellow' and do.type_str == 'ball':
            evidence.object_id = RAS_EVIDENCE.yellow_ball
        elif do.color_str == 'purple' and do.type_str == 'cross':
            evidence.object_id = RAS_EVIDENCE.purple_cross
        elif do.color_str == 'purple' and do.type_str == 'star':
            evidence.object_id = RAS_EVIDENCE.purple_star

        #Robot frame??
        evidence.object_location.transform.translation.x = do.x
        evidence.object_location.transform.translation.y = do.y
        evidence.object_location.transform.translation.z = do.z        
        self.evidence.publish(evidence) 
        print ("I saw", do.color_str, do.type_str, 'at', do.x, do.y, do.z)

        do.robot_x = self.x
        do.robot_y = self.y
        self.pickables.append(do)

    def pickup(self, do, do_pickup, ignore_type=False): # do means detected object
        # Call manipulation service
        pickup_location = PointStamped()
        pickup_location.point = Point(
            do.x, do.y, do.z
        )
        pickup_location.header.frame_id = 'wheel_center'

        if ignore_type:
            isPickedUp = self.pickupObject_client(pickup_location, do.color_str, '', do_pickup)
        else:
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
        except rospy.ServiceException as e:
            print("PickupObject service could not be called")

    def placeObject_client(self,position):
        rospy.wait_for_service('/manipulation/place_object')
        try:
            place_object = rospy.ServiceProxy('/manipulation/place_object',PlaceObject)
            response = place_object(position)
            return response
        except rospy.ServiceException as e:
            print("PlaceObject service could not be called")

    def dropObject_client(self):
        rospy.wait_for_service('/manipulation/drop_object')
        try:
            drop_object = rospy.ServiceProxy('/manipulation/drop_object',DropObject)
            response = drop_object()
            return response
        except rospy.ServiceException as e:
            print("DropObject service could not be called")

    def carryObject_client(self):
        rospy.wait_for_service('/manipulation/carry_object')
        try:
            carry_object = rospy.ServiceProxy('/manipulation/carry_object',CarryObject)
            response = carry_object()
            return response
        except rospy.ServiceException as e:
            print("CarryObject service could not be called")

    def is_new_object(self, do):
        # Object is in starting box
        if 0.0 < do.x < 0.4 and 0.0 < do.y < 0.4:
            return False

        # Check if already seen
        for o in self.pickables:
            if o == do: # it has the same position and identity as some other _pickable_ object
                return False
        return True

    def ignore_seen_pickable(self, do):
        # Object is in starting box
        if 0.0 < do.x < 0.4 and 0.0 < do.y < 0.4:
            return 

        # Check if already seen
        for o in self.pickables:
            if o.position_equal(do): # it has the same position as some other _pickable_ object
                return True

        return False

    def pickup_client(self, position, color, object_type):
        rospy.wait_for_service('manipulation/pickup_object',PickupObject)
        try:
            pickup_object = rospy.ServiceProxy('manipulation/pickup_object',PickupObject)
            response = pickup_object(position, color, object_type)
            return response
        except rospy.ServiceException as e:
            print("Pickup service could not be called")

    def carry_client(self):
        rospy.wait_for_service('manipulation/carry_object',CarryObject)
        try:
            carry_object = rospy.ServiceProxy('manipulation/carry_object',CarryObject)
            response = carry_object()
            return response
        except rospy.ServiceException as e:
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

    dummy_parser = subparsers.add_parser('dummy', help='do nothing except listen to callbacks')
    dummy_parser.set_defaults(which='dummy')

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
        rate = rospy.Rate(10)
        for _ in mother.goto(x=x, y=y, theta=theta):
            rate.sleep()
    elif args.which == 'explore':
        mother.explore()
    elif args.which == 'stop':
        mother.object_target = 'dummy ugly hack'
        mother.stop()
    elif args.which == 'dummy':
        mother.object_target = 'dummy ugly hack'
        mother.dummy()
    elif args.which == 'score':
        mother.score()
    else:
        raise NotImplementedError('Only goto implemented atm')

    rospy.spin()

#/obstaclePos_wheelcenter, geometry_msgs/polygonStamped
