#!/usr/bin/env python
import sys
import rospy
import time
import math
import numpy
import os
import threading
from std_srvs.srv import SetBool
from std_srvs.srv import SetBoolRequest
from std_srvs.srv import Empty
from uav_abstraction_layer.srv import TakeOff
from uav_abstraction_layer.srv import TakeOffRequest
from uav_abstraction_layer.srv import GoToWaypoint
from uav_abstraction_layer.srv import GoToWaypointRequest
from mission_planner.srv import WaypointSrvRequest
from mission_planner.srv import WaypointSrv
import signal
import sys


# Menu function
def show_menu():
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
        preparing_drones()
    elif option == 2:
        start_mission()
    elif option == 3:
        stop_mission()
    elif option == 4:
        add_one_waypoint()
    elif option == 5:
        clear_all_waypoints()
    elif option == 6:
        relative_angle()
    elif option == 7:
        distance_inspection()
    elif option == 8:
        height_inspection()
    else:
        print ("Option n" + str(option) + "does not exist!")

# Finish the execution directly when Ctrl+C is pressed (signal.SIGINT received), without escalating to SIGTERM.
def signal_handler(sig, frame):
    print('Ctrl+C pressed, signal.SIGINT received.')
    sys.exit(0)    

# 1.        preparing_drones function
# Brief:    Takes off all the drones and send them to the initial point
# NOTE: Not finished
def preparing_drones():
    print "Preparing drones called"
    
    # DRONE 1 - LEADER
    # Only the leader drone guidable
    uav_id = "drone_1"
    ns     = "/drone_1"
    
    # TakeOff service
    take_off_service = rospy.ServiceProxy(ns+"/ual/take_off", TakeOff)
    
    # GoToWaypoint service
    go_to_waypoint_url      = "/drone_1/ual/go_to_waypoint"
    rospy.wait_for_service(go_to_waypoint_url)
    go_to_waypoint_service  = rospy.ServiceProxy(go_to_waypoint_url, GoToWaypoint)
    
    # DRONE n - FOLLOWERS
    # TODO
    
    
    # Taking off
    try:
        take_off            = TakeOffRequest()
        take_off.height     = 1
        take_off.blocking   = False
        
        # Leader drone service
        take_off_service(take_off)
        
        # Followers service
        # TODO
        
        print "LEADER: Taking off the drone"
    except rospy.ServiceException, e:
        print "Service call failed: %s" %e
    
    # Send to the initial point
    # TODO: NEED TO GET THE INITIAL POINT FROM SOMEWHERE
    initial_point = [3, 4, 5]
    
    # Temporary fix: need to know if the drone has already reached the desired height of the take off
    time.sleep(7)
    
    # Going to initial point
    try:
        waypoint              = GoToWaypointRequest()
        waypoint.blocking     = False
        waypoint.waypoint.pose.position.x     = initial_point[0]
        waypoint.waypoint.pose.position.y     = initial_point[1]
        waypoint.waypoint.pose.position.z     = initial_point[2]
        
        go_to_waypoint_service(waypoint)
        print "LEADER: Going to initial waypoint"
    
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
def add_one_waypoint():
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
def height_inspection():
    pass








    
# Main function
if __name__ == "__main__":
    
    # Create the node
    rospy.init_node("operator", anonymous=True)
    
    signal.signal(signal.SIGINT, signal_handler)    # Associate signal SIGINT (Ctrl+C pressed) to handler (function "signal_handler")
    # Mission planner services
    activate_planner_url = "/drone_1/mission_planner_ros/activate_planner" # Missing its functionality atm
    add_waypoint_url     = "/drone_1/mission_planner_ros/add_waypoint"
    clear_waypoints_url  = "/drone_1/mission_planner_ros/clear_waypoints"
    
    activate_planner_service = rospy.ServiceProxy(activate_planner_url, SetBool)
    add_waypoint_service     = rospy.ServiceProxy(add_waypoint_url, WaypointSrv)
    clear_waypoints_service  = rospy.ServiceProxy(clear_waypoints_url, Empty)
    
    while (not rospy.is_shutdown()):
        show_menu()
        time.sleep(1)