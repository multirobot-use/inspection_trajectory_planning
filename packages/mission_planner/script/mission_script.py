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
import signal
import sys

# Global variables
global take_off_service
global go_to_waypoint_service

global leader_ready
global leader_landed

global follower_ready
global follower_landed

# Params read from .yml file
global auto
global leader_start_point
global follower_start_point
global take_off_height
global take_off_blocking
global height_to_inspect
global n_waypoints
global waypoint

# Menu function
def show_menu():
    global leader_start_point
    global follower_start_point
    global take_off_height
    global take_off_blocking
    global height_to_inspect
    
    # Menu
    print "\n\nWelcome to the main menu. Put the number of the desired option:\n"
    print "\t1. Take off and send the drones to their initial points (WORKING)"
    print "\t2. Start the mission"
    print "\t3. Stop the mission"
    print "\t4. Add waypoint (WORKING)"
    print "\t5. Clear all the waypoints (WORKING)"
    print "\t6. Change relative angles between followers and leader"
    print "\t7. Change distance to inspection point"
    print "\t8. Change height to inspection point\n"
    
    option = int(raw_input (">> "))
    while (option < 1 or option > 8):
        option = raw_input("Please, choose a valid option: ")

    if option == 1:
        preparing_drones(leader_start_point, follower_start_point, take_off_height, take_off_blocking)
    elif option == 2:
        start_mission()
    elif option == 3:
        stop_mission()
    elif option == 4:
        add_one_waypoint(False)
    elif option == 5:
        clear_all_waypoints()
    elif option == 6:
        relative_angle()
    elif option == 7:
        distance_inspection()
    elif option == 8:
        height_inspection(height_to_inspect)
    else:
        print ("Option n " + str(option) + " does not exist!")

# Finish the execution directly when Ctrl+C is pressed (signal.SIGINT received), without escalating to SIGTERM.
def signal_handler(sig, frame):
    print('Ctrl+C pressed, signal.SIGINT received.')
    sys.exit(0)    

# 1.        preparing_drones function
# Brief:    Takes off all the drones and send them to the initial point
# NOTE: Not finished
def preparing_drones(leader_start_point, follower_start_point, height, blocking):
    global take_off_service
    global go_to_waypoint_service
    
    global leader_ready
    global leader_landed
    
    global follower_ready
    global follower_landed
    print "Preparing drones function called"
    
    # DRONE 1 - LEADER
    # Only the leader drone guidable
    
    # Temporary fix: look for a better solution
    if (leader_ready and follower_ready):
        print "All drones are already taken off!"
        resp = raw_input("Do you want to send them to their respective initial point? (y/n): ")

    else:
        if not (leader_landed and follower_landed):
            print "Waiting for drones..."
        while not (leader_landed and follower_landed):
            time.sleep(1)
        
        # Taking off
        if leader_landed:
            try:
                take_off            = TakeOffRequest()
                take_off.height     = height
                take_off.blocking   = blocking
                
                # Leader drone service
                take_off_service[0](take_off)
                
                print "LEADER: Taking off the drone"
            except rospy.ServiceException, e:
                print "Service call failed: %s" %e
        
        if follower_landed:
            try:
                take_off            = TakeOffRequest()
                take_off.height     = height
                take_off.blocking   = blocking
                
                # Leader drone service
                take_off_service[1](take_off)
                
                print "FOLLOWER: Taking off the drone"
            except rospy.ServiceException, e:
                print "Service call failed: %s" %e
        
        # Wait until all of the drones are ready
        while (not (leader_ready and follower_ready)):
            time.sleep(0.2)
        
        resp = 'y'
    
    # LEADER
    # Send to the initial point
    if (leader_ready and follower_ready) and resp == 'y':
        try:
            waypoint              = GoToWaypointRequest()
            waypoint.blocking     = False
            waypoint.waypoint.pose.position.x     = leader_start_point[0]
            waypoint.waypoint.pose.position.y     = leader_start_point[1]
            waypoint.waypoint.pose.position.z     = leader_start_point[2]
            
            go_to_waypoint_service[0](waypoint)
            print "LEADER: Going to initial waypoint"
        
        except rospy.ServiceException, e:
            print "Service call failed: %s" %e
    
    # FOLLOWER
        try:
            waypoint              = GoToWaypointRequest()
            waypoint.blocking     = False
            waypoint.waypoint.pose.position.x     = follower_start_point[0]
            waypoint.waypoint.pose.position.y     = follower_start_point[1]
            waypoint.waypoint.pose.position.z     = follower_start_point[2]
            
            go_to_waypoint_service[1](waypoint)
            print "FOLLOWER: Going to initial waypoint"
        
        except rospy.ServiceException, e:
            print "Service call failed: %s" %e


# 2.        start_mission function
# Brief:    This function allows the drones to start the mission
# TODO
def start_mission():
    try:
        req = SetBoolRequest()
        req.data = True
        activate_planner_service(req)
    except rospy.ServiceException, e:
        print("Failed calling add_waypoint service")

# 3.        stop_mission function
# Brief:    This function stops mission
# TODO
def stop_mission():
    pass


# 4.        add_one_waypoint function
# Brief:    This function adds one waypoint to the queue of waypoints that the drone has to reach
def add_one_waypoint(data):
    
    global add_waypoint_service
    global n_waypoints
    # Not auto mode
    if type(data) == bool and data == False:
        px = float(raw_input("X pose: "))
        py = float(raw_input("Y pose: "))
        pz = float(raw_input("Z pose: "))
        
        tx = float(raw_input("X linear velocity: "))
        ty = float(raw_input("Y linear velocity: "))
        tz = float(raw_input("Z linear velocity: "))
        
        add_waypoint_req = WaypointSrvRequest()
        
        add_waypoint_req.waypoint.pose.pose.position.x      = px
        add_waypoint_req.waypoint.pose.pose.position.y      = py
        add_waypoint_req.waypoint.pose.pose.position.z      = pz
        
        add_waypoint_req.waypoint.twist.twist.linear.x      = tx
        add_waypoint_req.waypoint.twist.twist.linear.y      = ty
        add_waypoint_req.waypoint.twist.twist.linear.z      = tz
            
        try:
            add_waypoint_service(add_waypoint_req)
            print "Waypoint added"
        except:
            print("Failed calling add_waypoint service")
    
    # Auto mode
    if type(data) == list:
        # data are the waypoints
        for index in range(n_waypoints):
            add_waypoint_req = WaypointSrvRequest()
            add_waypoint_req.waypoint.pose.pose.position.x      = data[index, 0]
            add_waypoint_req.waypoint.pose.pose.position.y      = data[index, 1]
            add_waypoint_req.waypoint.pose.pose.position.z      = data[index, 2]
            
            try:
                add_waypoint_service(add_waypoint_req)
                print "Waypoint " + str(index) + " added"
            except:
                print("Failed calling add_waypoint service")


# 5.        clear_all_waypoints function
# Brief:    This function clears the queue of desired waypoints
def clear_all_waypoints():
    try:
        clear_waypoints_service()
    except:
        print("Failed calling clear_waypoints service")


# 6.        relative_angle function
# Brief:    This function allows to change the relative angle between the leader and the followers
# TODO
def relative_angle():
    pass


# 7.        relative_angle function
# Brief:    This function changes the distance between the inspection point and the drones
# TODO
def distance_inspection():
    pass


# 8.        height_inspection function
# Brief:    This function changes the desired height to inspect
# TODO
def height_inspection(height_to_inspect):
    pass

def automatic_function():    
    global leader_start_point
    global follower_start_point
    global take_off_height
    global take_off_blocking
    global height_to_inspect
    global waypoint
    
    print "-------- TAKE OFF AND INITIAL POINT --------\n"
    preparing_drones(leader_start_point, follower_start_point, take_off_height, take_off_blocking)
    
    print "\n-------- ADDING WAYPOINTS --------\n"
    add_one_waypoint(waypoint)
    

def callbackStateLeader(data):
    global leader_ready
    global leader_landed
    data_splitted = str(data).split(": ")
    state = int(data_splitted[1])

    # state     = 2     --> landed
    #           = 3     --> taking off
    #           = 4     --> ready for moving
    if (state == 2):
        leader_landed   = True
    else:
        leader_landed   = False
    
    if (state == 4):
        leader_ready    = True
    else:
        leader_ready    = False

def callbackStateFollower(data):
    global follower_ready
    global follower_landed
    data_splitted = str(data).split(": ")
    state = int(data_splitted[1])

    # state     = 2     --> landed
    #           = 3     --> taking off
    #           = 4     --> ready for moving
    if (state == 2):
        follower_landed = True
    else:
        follower_landed = False
        
    if (state == 4):
        follower_ready = True
    else:
        follower_ready = False

#           read_params function
# Brief:    This function reads the parameters of a .yml file
def read_params(file_route):
    global auto
    global leader_start_point
    global follower_start_point
    global take_off_height
    global take_off_blocking
    global height_to_inspect
    global n_waypoints
    global waypoint

    # Read parameters in local function:
    yml_file = open(file_route, 'r')
    yml_content = yaml.load(yml_file)
    
    auto                    = yml_content.get('auto')
    leader_start_point      = yml_content.get('leader_start_point')
    follower_start_point    = yml_content.get('follower_start_point')
    take_off_height         = yml_content.get('take_off_height')
    take_off_blocking       = yml_content.get('take_off_blocking')
    height_to_inspect       = yml_content.get('height_to_inspect')
    
    if auto:
        n_waypoints         = yml_content.get('n_waypoints')
        
        # Initialize
        waypoint            = np.zeros((n_waypoints, 3))
        
        print "Waypoints for auto mode: "
        
        for index in range(n_waypoints):
            waypoint[index] = yml_content.get('waypoint' + str(index + 1))
            print waypoint[index]


# Main function
if __name__ == "__main__":
    global leader_ready
    global leader_landed
    
    global follower_ready
    global follower_landed
    
    global take_off_service
    global add_waypoint_service
    global go_to_waypoint_service
    
    # Initialize
    leader_ready        = False
    leader_landed       = False
    follower_ready      = False
    leader_landed       = False
    
    take_off_service        = [0, 0]
    go_to_waypoint_url      = [0, 0]
    go_to_waypoint_service  = [0, 0]
    
    uav_id  = ["drone_1", "drone_2"]
    ns      = ["/drone_1", "/drone_2"]
    
    # Create the node
    rospy.init_node("operator", anonymous=True)
    
    signal.signal(signal.SIGINT, signal_handler)    # Associate signal SIGINT (Ctrl+C pressed) to handler (function "signal_handler")
    
    # Subscribers
    rospy.Subscriber("/drone_1/ual/state", State, callbackStateLeader)
    rospy.Subscriber("/drone_2/ual/state", State, callbackStateFollower)
    
    # Mission planner services
    activate_planner_url = ns[0] + "/mission_planner_ros/activate_planner" # Missing its functionality atm
    add_waypoint_url     = ns[0] + "/mission_planner_ros/add_waypoint"
    clear_waypoints_url  = ns[0] + "/mission_planner_ros/clear_waypoints"
    
    activate_planner_service = rospy.ServiceProxy(activate_planner_url, SetBool)
    add_waypoint_service     = rospy.ServiceProxy(add_waypoint_url, WaypointSrv)
    clear_waypoints_service  = rospy.ServiceProxy(clear_waypoints_url, Empty)
    
    # TakeOff service
    take_off_service[0] = rospy.ServiceProxy(ns[0]+"/ual/take_off", TakeOff)
    take_off_service[1] = rospy.ServiceProxy(ns[1]+"/ual/take_off", TakeOff)
    
    # GoToWaypoint service
    go_to_waypoint_url[0]      = ns[0] + "/ual/go_to_waypoint"
    rospy.wait_for_service(go_to_waypoint_url[0])
    go_to_waypoint_service[0]  = rospy.ServiceProxy(go_to_waypoint_url[0], GoToWaypoint)

    go_to_waypoint_url[1]      = ns[1] + "/ual/go_to_waypoint"
    rospy.wait_for_service(go_to_waypoint_url[1])
    go_to_waypoint_service[1]  = rospy.ServiceProxy(go_to_waypoint_url[1], GoToWaypoint)
    
    # Read the parameters
    rospack = rospkg.RosPack()
    f_route = rospack.get_path('mission_planner')+'/config/param.yml'
    read_params(f_route)
    
    while (not rospy.is_shutdown()):
        if not auto:
            show_menu()
        else:
            print "Using the automatic interface"
            automatic_function()
            exit()
            
        time.sleep(1)