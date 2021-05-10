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
from uav_abstraction_layer.srv import TakeOff
from uav_abstraction_layer.srv import TakeOffRequest
from uav_abstraction_layer.srv import GoToWaypoint
from uav_abstraction_layer.srv import GoToWaypointRequest
from uav_abstraction_layer.msg import State
from mission_planner.srv import WaypointSrvRequest
from mission_planner.srv import WaypointSrv
from mission_planner.srv import PointToInspectSrvRequest
from mission_planner.srv import PointToInspectSrv
from mission_planner.srv import DistanceSrvRequest
from mission_planner.srv import DistanceSrv
from mission_planner.srv import AngleSrvRequest
from mission_planner.srv import AngleSrv
import signal
import sys
from collections import namedtuple

class Drone:
    state = 0
    def __init__(self, drone_ns):
        print("I'm a python constructor")
        #subscrbie topics
        rospy.Subscriber(drone_ns+"/ual/state", State, self.callbackState)

        #wait for services
        activate_planner_url = drone_ns + "/mission_planner_ros/activate_planner"
        add_waypoint_url     = drone_ns + "/mission_planner_ros/add_waypoint"
        clear_waypoints_url  = drone_ns + "/mission_planner_ros/clear_waypoints"
        point_to_inspect_url = drone_ns + "/mission_planner_ros/point_to_inspect"
        distance_url         = drone_ns + "/mission_planner_ros/distance_to_inspect"
        relative_angle_url   = drone_ns + "/mission_planner_ros/change_relative_angle"
        take_off_url   = drone_ns + "/ual/take_off"

        rospy.wait_for_service(activate_planner_url)
        self.activate_planner_service = rospy.ServiceProxy(activate_planner_url, SetBool)

        rospy.wait_for_service(add_waypoint_url)
        self.add_waypoint_service     = rospy.ServiceProxy(add_waypoint_url, WaypointSrv)

        rospy.wait_for_service(clear_waypoints_url)
        self.clear_waypoints_service  = rospy.ServiceProxy(clear_waypoints_url, Empty)

        rospy.wait_for_service(point_to_inspect_url)
        self.point_to_inspect_service = rospy.ServiceProxy(point_to_inspect_url, PointToInspectSrv)

        rospy.wait_for_service(distance_url)
        self.distance_service = rospy.ServiceProxy(distance_url, DistanceSrv)

        rospy.wait_for_service(relative_angle_url)
        self.relative_angle_service = rospy.ServiceProxy(relative_angle_url, AngleSrv)

        # TakeOff service --> make a for loop when necessary
        rospy.wait_for_service(take_off_url)
        self.take_off_service = rospy.ServiceProxy(take_off_url, TakeOff)

        # # GoToWaypoint service
        go_to_waypoint_url      = drone_ns + "/ual/go_to_waypoint"
        rospy.wait_for_service(go_to_waypoint_url)
        self.go_to_waypoint_service  = rospy.ServiceProxy(go_to_waypoint_url, GoToWaypoint)

    
    def set_distance_inspection(self, distance):
        r_inspect = DistanceSrvRequest()
        r_inspect.distance = distance
        
        try:
            self.distance_service(r_inspect)
            print "Distance to the inspection point changed successfully!"
        
        except:
            print("Failed calling distance_to_inspect service")

    def change_relative_angle(self, angle):
        relative_angle = AngleSrvRequest()
        relative_angle.angle = angle
        try:
            self.relative_angle_service(relative_angle)
            print "Relative angle changed successfully!"

        except:
            print("Failed calling change_relative_angle service")

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
    def take_off(self, height):
        try:
            take_off            = TakeOffRequest()
            take_off.height     = height
            take_off.blocking   = True
            
            # Leader drone service
            self.take_off_service(take_off)
            print "Taking off the drone"
        except rospy.ServiceException, e:
            print "Service call failed: %s" %e

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

    def start_mission(self):
        try:
            req = SetBoolRequest()
            req.data = True
            self.activate_planner_service(req)
            print "Mission has started!"

        except rospy.ServiceException, e:
            print("Failed calling start_mission service")

    def add_one_waypoint(self, waypoint):
        
        add_waypoint_req = WaypointSrvRequest()
        add_waypoint_req.waypoint.pose.pose.position.x      = waypoint[0]
        add_waypoint_req.waypoint.pose.pose.position.y      = waypoint[1]
        add_waypoint_req.waypoint.pose.pose.position.z      = waypoint[2]
        print "waypoint sent:"
        print(waypoint)
        try:
            self.add_waypoint_service(add_waypoint_req)
            print "Waypoint added"
            
        except:
            print("Failed calling add_waypoint service")

    def callbackState(self, data):
        self.state = data.state
    def stop_mission(self):
        try:
            req = SetBoolRequest()
            req.data = False
            activate_planner_service(req)
            activate_planner_service_follower(req)
            print "Mission has been stopped!"
        
        except rospy.ServiceException, e:
            print("Failed calling stop_mission service")
    def clear_all_waypoints():
        try:
            self.clear_waypoints_service()
            print "Waypoints cleared!"
        except:
            print("Failed calling clear_waypoints service")

   
# Menu function
def show_menu(params,drones):

    # Menu
    print "\n\nWelcome to the main menu. Put the number of the desired option:\n"
    print "\t1. Take off and send the drones to their initial points"
    print "\t2. Start the mission"
    print "\t3. Stop the mission"
    print "\t4. Add waypoint"
    print "\t5. Clear all the waypoints"
    print "\t6. Change relative angles between followers and leader"
    print "\t7. Change distance to inspection point"
    print "\t8. Change inspection point\n"
    
    option = int(raw_input (">> "))
    while (option < 1 or option > 8):
        option = raw_input("Please, choose a valid option: ")

    if option == 1: #prepare drones
        height = raw_input("Please, introduce a height to take off")
        #taking of drones
        for drone in drones:
            drone.take_off(params.height)
    elif option == 2:
        for drone in drones:
            drone.start_mission()
    elif option == 3:
        for drone in drones:
            drone.stop_mission()
    elif option == 4:
        px = float(raw_input("X pose (meters): "))
        py = float(raw_input("Y pose (meters): "))
        pz = float(raw_input("Z pose (meters): "))
        wp = [px, py, pz]
        for wp in params.waypoints:
            drone.add_one_waypoint(wp)
    elif option == 5:
        for drone in drones:
            drone.clear_all_waypoints()
    elif option == 6:
        print "Please, enter the desired relative angle between follower drones and the leader drone (Manual mode)\n"
        angle = (3.1415/180)*float(raw_input("Angle (degrees): "))
        for drone in drones:
            drone.change_relative_angle(angle)
    elif option == 7:
        print "Please, enter the desired relative angle between follower drones and the leader drone (Manual mode)\n"
        distance = float(raw_input("Distance (meters): "))
        for drone in drones:
            drone.set_distance_inspection(distance)
    elif option == 8:
        print "Please, enter the desired inspection point (Manual mode):\n"
        waypoint.append(float(raw_input("X (meters): ")))
        waypoint.append(float(raw_input("Y (meters): ")))
        waypoint.append(float(raw_input("Z (meters): ")))
        change_inspection_point(waypoint)
    else:
        print ("Option n " + str(option) + " does not exist!")

# Finish the execution directly when Ctrl+C is pressed (signal.SIGINT received), without escalating to SIGTERM.
def signal_handler(sig, frame):
    print('Ctrl+C pressed, signal.SIGINT received.')
    sys.exit(0)    
        

def auto_function(params,drones):

    #taking of drones
    for drone in drones:
        drone.take_off(params.height)

    for drone in drones:
        drone.set_distance_inspection(params.inspect_point[3])
        drone.change_relative_angle(params.relative_angle)
        drone.change_inspection_point(params.inspect_point)
        for wp in params.waypoints:
            drone.add_one_waypoint(wp)
    #starting mission
    for drone in drones:
        drone.start_mission()


# Main function
if __name__ == "__main__":

    signal.signal(signal.SIGINT, signal_handler)    # Associate signal SIGINT (Ctrl+C pressed) to handler (function "signal_handler")
    
    
    # Create the node
    rospy.init_node("operator", anonymous=True)

    # read yml config file
    rospack = rospkg.RosPack()
    f_route = rospack.get_path('mission_planner')+'/config/param.yml'
    yml_file    = open(f_route, 'r')
    yml_content = yaml.load(yml_file)

    drone_ids = yml_content.get('drone_ids')
    drones = []
    for id in drone_ids:
        drones.append(Drone("/drone_"+str(id)))

    params = namedtuple('params', 'auto height inspect_point waypoints relative_angle')
    params.auto                      = yml_content.get('auto')
    params.height                    = yml_content.get('take_off_height')
    params.waypoints                 = yml_content.get('waypoints')
    params.inspect_point             = yml_content.get('inspect')
    params.relative_angle            = yml_content.get('relative_angle')
    
    # check all drones are landed armed
    cont = 0
    while cont != len(drones):
        cont = 0
        for drone in drones:
            if drone.state == State.LANDED_ARMED:
                cont +=1
        time.sleep(1)

    #auto mode
    if params.auto:
        print "Using the automatic interface"
        auto_function(params,drones)

    #menu mode
    while (not rospy.is_shutdown()):
        show_menu(params,drones)            
        time.sleep(1)