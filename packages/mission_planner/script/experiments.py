#!/usr/bin/env python
import sys
import rospy
import rospkg
import time
import math
import numpy
import os
import yaml
import threading
import numpy as np
from std_srvs.srv import SetBool
from std_srvs.srv import SetBoolRequest
from std_srvs.srv import Empty
from std_msgs.msg import Bool
from uav_abstraction_layer.srv import TakeOff
from uav_abstraction_layer.srv import TakeOffRequest
from uav_abstraction_layer.srv import Land
from uav_abstraction_layer.srv import LandRequest
from uav_abstraction_layer.srv import GoToWaypoint
from uav_abstraction_layer.srv import GoToWaypointRequest
from uav_abstraction_layer.msg import State
from mission_planner.srv import PointToInspectSrvRequest
from mission_planner.srv import PointToInspectSrv
from mission_planner.srv import DistanceSrvRequest
from mission_planner.srv import DistanceSrv
from mission_planner.srv import AngleSrvRequest
from mission_planner.srv import AngleSrv
from mission_planner.srv import WaypointSrvRequest
from mission_planner.srv import WaypointSrv
import signal
import sys
from collections import namedtuple

class Drone:
    state = 0
    def __init__(self, drone_ns):
        print("I'm a python constructor")

        # subscribe topics
        rospy.Subscriber(drone_ns+"/ual/state", State, self.callbackState)
        
        # Publishers (only one drone topic needed for each one)
        self.distance_inspection_pub = rospy.Publisher('/drone_1/mission_planner_ros/distance_to_inspection_point', Bool, queue_size = 1)
        self.relative_angle_pub      = rospy.Publisher('/drone_1/mission_planner_ros/relative_angle', Bool, queue_size = 1)

        # wait for services
        activate_planner_url = drone_ns + "/mission_planner_ros/activate_planner"
        add_waypoint_url     = drone_ns + "/mission_planner_ros/add_waypoint"
        point_to_inspect_url = drone_ns + "/mission_planner_ros/point_to_inspect"
        distance_url         = drone_ns + "/mission_planner_ros/distance_to_inspect"
        relative_angle_url   = drone_ns + "/mission_planner_ros/change_relative_angle"
        take_off_url         = drone_ns + "/ual/take_off"
        land_url             = drone_ns + "/ual/land"
        go_to_waypoint_url   = drone_ns + "/ual/go_to_waypoint"

        # Activate planner service
        rospy.wait_for_service(activate_planner_url)
        self.activate_planner_service = rospy.ServiceProxy(activate_planner_url, SetBool)

        # Add waypoint service
        rospy.wait_for_service(add_waypoint_url)
        self.add_waypoint_service     = rospy.ServiceProxy(add_waypoint_url, WaypointSrv)
   
        # Change point to inspect service
        rospy.wait_for_service(point_to_inspect_url)
        self.point_to_inspect_service = rospy.ServiceProxy(point_to_inspect_url, PointToInspectSrv)

        # Change distance to inspect service
        rospy.wait_for_service(distance_url)
        self.distance_service = rospy.ServiceProxy(distance_url, DistanceSrv)

        # Change relative angle service
        rospy.wait_for_service(relative_angle_url)
        self.relative_angle_service = rospy.ServiceProxy(relative_angle_url, AngleSrv)

        # TakeOff service
        rospy.wait_for_service(take_off_url)
        self.take_off_service = rospy.ServiceProxy(take_off_url, TakeOff)

        # Land service
        rospy.wait_for_service(land_url)
        self.land_service = rospy.ServiceProxy(land_url, Land)

        # GoToWaypoint service
        rospy.wait_for_service(go_to_waypoint_url)
        self.go_to_waypoint_service  = rospy.ServiceProxy(go_to_waypoint_url, GoToWaypoint)

    # Take off drones method
    def take_off(self, height):
        try:
            take_off            = TakeOffRequest()
            take_off.height     = height
            take_off.blocking   = True
            
            self.take_off_service(take_off)
            print "Taking off the drone"
        except rospy.ServiceException, e:
            print "Service call failed: %s" %e
    
    # Land drones method
    def land(self):
        try:
            land            = LandRequest()
            land.blocking   = False
            
            # Stop the mission before calling the service of landing
            self.stop_mission()
            
            self.land_service(land)
            print "Landing the drone"
        except rospy.ServiceException, e:
            print "Service call failed: %s" %e

    # Go to waypoint method
    def go_to_waypoint(self, waypoint):
        try:
            waypoint              = GoToWaypointRequest()
            waypoint.blocking     = False
            waypoint.waypoint.pose.position.x     = waypoint[0]
            waypoint.waypoint.pose.position.y     = waypoint[1]
            waypoint.waypoint.pose.position.z     = waypoint[2]
            
            self.go_to_waypoint_service(waypoint)
            print "LEADER: Going to initial waypoint"
        
        except rospy.ServiceException, e:
            print "Service call failed: %s" %e

    # Start mission method
    def start_mission(self):
        if self.state == State.FLYING_AUTO:
            try:
                req = SetBoolRequest()
                req.data = True
                self.activate_planner_service(req)
                print "Mission has started!"

            except rospy.ServiceException, e:
                print("Failed calling start_mission service")
        else:
            print("Start mission aborted, drone is not flying auto")

    # Add waypoint method
    def add_one_waypoint(self, waypoint):
        
        add_waypoint_req = WaypointSrvRequest()
        add_waypoint_req.waypoint.pose.pose.position.x      = waypoint[0]
        add_waypoint_req.waypoint.pose.pose.position.y      = waypoint[1]
        add_waypoint_req.waypoint.pose.pose.position.z      = waypoint[2]
        print "Waypoint sent"
        print(waypoint)
        try:
            self.add_waypoint_service(add_waypoint_req)
            print "Waypoint added!"
            
        except:
            print("Failed calling add_waypoint service")
    
    # Send home method
    def send_home(self, point):
        home = GoToWaypointRequest()
        # TODO
        home.waypoint.pose.position.x = point[0]
        home.waypoint.pose.position.y = point[1]
        home.waypoint.pose.position.z = point[2]
        home.blocking                 = True # Send one by one
        #print point
        try:
            self.go_to_waypoint_service(home)
            print "Sending drone to home position"
            
        except:
            print "Failed calling go_to_waypoint service"

    # Stop mission method
    def stop_mission(self):
        try:
            req = SetBoolRequest()
            req.data = False
            self.activate_planner_service(req)
            print "Mission has been stopped!"
        
        except rospy.ServiceException, e:
            print("Failed calling stop_mission service")
    
    # Distance to inspect method
    def set_distance_inspection(self, distance):
        r_inspect = DistanceSrvRequest()
        r_inspect.distance = distance
        
        try:
            self.distance_service(r_inspect)
            print "Distance to the inspection point changed successfully!"
        
        except:
            print("Failed calling distance_to_inspect service")

    # Change relative angle method
    def change_relative_angle(self, angle):
        relative_angle = AngleSrvRequest()
        relative_angle.angle = angle
        try:
            self.relative_angle_service(relative_angle)
            print "Relative angle changed successfully!"

        except:
            print("Failed calling change_relative_angle service")

    # Change inspection point method
    def change_inspection_point(self, point):
        inspection_point = PointToInspectSrvRequest()
    
        inspection_point.point.x = point[0]
        inspection_point.point.y = point[1]
        inspection_point.point.z = point[2]

        try:
            self.point_to_inspect_service(inspection_point)
            print "Inspection point changed successfully!"

        except:
            print("Failed calling point_to_inspect service")
    
    # Callback state
    def callbackState(self, data):
        self.state = data.state
        

   
# Menu function
def show_experiments_menu():

    # Menu
    print "\n\nWelcome to the main menu. Put the number of the desired experiment:\n"
    print "  Experiment 0: simple path going twice around the inspection point (2 drones)"
    print "  Experiment 1: simple path changing the direction once (2 drones)"
    print "  Experiment 2: simple path changing the direction repeatedly (2 drones)"
    print "  Experiment 3: simple path, but changing the height between waypoints (2 drones)"
    print "  Experiment 4: complex path changing height and direction (2 drones)"
    print "  Experiment 5: simple path going twice around the inspection point (3 drones)"
    print "  Experiment 6: simple path changing the direction once (3 drones)"
    print "  Experiment 7: simple path changing the direction repeatedly (3 drones)"
    print "  Experiment 8: simple path, but changing the height between waypoints (3 drones)"
    print "  Experiment 9: complex path changing height and direction (3 drones)"
    
    option = ord(raw_input (">> "))
    while (option < (48+0) or option > (48+9)): # ASCII for make sure there is no error of inputs. Zero --> 48
        option = ord(raw_input("Please, choose a valid option: "))

    # Return the desired experiment
    return (option - 48)    

# Finish the execution directly when Ctrl+C is pressed (signal.SIGINT received), without escalating to SIGTERM.
def signal_handler(sig, frame):
    print('Ctrl+C pressed, signal.SIGINT received.')
    sys.exit(0)    
        
# Auto function: take off the drones, set the parameters up and start the mission of each drone
def auto_function(params,drones):

    # Go through the different steps of the experiment by pressing keys
    raw_input("Press ENTER to take off the drones\n")
    # Taking of drones
    for drone in drones:
        drone.take_off(params.height)
    
    time.sleep(1)
    raw_input("Press ENTER to set up the parameters of each drone and add the waypoints\n")
    print "Adding configuration and waypoints\n"
    # Set up the parameters of each drone
    for drone in drones:
        drone.set_distance_inspection(params.inspect_point[3])
        drone.change_relative_angle(params.relative_angle)
        drone.change_inspection_point(params.inspect_point)
        for wp in params.waypoints:
            drone.add_one_waypoint(wp)
    
    time.sleep(1)
    raw_input("Press ENTER to START the mission\n")
    print "Starting the mission\n"
    
    for drone in drones:
        drone.start_mission()
    
    time.sleep(1)
    raw_input("Press ENTER to STOP the mission\n")
    print "Stopping the mission\n"
    
    for drone in drones:
        drone.stop_mission()
    
    time.sleep(3)
    raw_input("Press ENTER to send each drone to their home position\n")
    print "Sending each drone to their home position\n"
    aux = 0
    for drone in drones:
        drone.send_home(params.drone_home_position[aux])
        aux = aux + 1
    
    time.sleep(1)
    raw_input("Press ENTER to land the drones and finish the experiment")
    print "Landing the drones and finishing the experiment"

    for drone in drones:
        drone.land()


# Main function
if __name__ == "__main__":

    signal.signal(signal.SIGINT, signal_handler)    # Associate signal SIGINT (Ctrl+C pressed) to handler (function "signal_handler")

    # Create the node
    rospy.init_node("operator", anonymous=True)

    # Choose the experiment and read yml config file
    rospack = rospkg.RosPack()
    experiment = show_experiments_menu()
    f_route = rospack.get_path('mission_planner') + '/config/experiments/exp' + str(experiment) + '.yml'
    yml_file    = open(f_route, 'r')
    yml_content = yaml.load(yml_file)

    drone_ids = yml_content.get('drone_ids')
    drones = []
    for id in drone_ids:
        drones.append(Drone("/drone_"+str(id)))

    params = namedtuple('params', 'auto height inspect_point waypoints relative_angle drone_home_position')
    params.auto                      = yml_content.get('auto')
    params.height                    = yml_content.get('take_off_height')
    params.waypoints                 = yml_content.get('waypoints')
    params.inspect_point             = yml_content.get('inspect')
    params.relative_angle            = yml_content.get('relative_angle')

    # CHECK IF THIS IS DONE CORRECTLY
    params.drone_home_position = []
    for id in drone_ids:
        params.drone_home_position.append(yml_content.get('drone' + str(id) + '_home'))

    # check all drones are landed armed
    cont = 0
    while cont != len(drones):
        cont = 0
        for drone in drones:
            if drone.state == State.LANDED_ARMED:
                cont +=1
        time.sleep(1)

    # auto mode
    if params.auto:
        print "Using the automatic interface"
        auto_function(params, drones)