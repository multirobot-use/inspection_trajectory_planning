#!/usr/bin/env python
import sys
import rospy
import time
import math
import numpy
import os
import threading
from std_srvs.srv import SetBool
from std_srvs.srv import Empty
from uav_abstraction_layer.srv import TakeOff
from uav_abstraction_layer.srv import TakeOffRequest
from uav_abstraction_layer.srv import GoToWaypoint
from mission_planner.srv import WaypointSrvRequest
from mission_planner.srv import WaypointSrv


# Main function
if __name__ == "__main__":
    
    # Create the node
    rospy.init_node("operator", anonymous=True)
    
    # Only the leader drone guidable
    uav_id = "drone_1"
    ns     = "/drone_1"
    
    # TakeOff service
    take_off_service = rospy.ServiceProxy(ns+"/ual/take_off", TakeOff)
    #
    
    # GoToWaypoint service
    go_to_waypoint_url = "/drone_1/ual/go_to_waypoint"
    rospy.wait_for_service(go_to_waypoint_url)
    go_to_waypoint = rospy.ServiceProxy(go_to_waypoint_url, GoToWaypoint)
    
    # Mission planner services
    activate_planner_url = "/drone_1/mission_planner_ros/activate_planner" # Missing its functionality atm
    add_waypoint_url     = "/drone_1/mission_planner_ros/add_waypoint"
    clear_waypoints_url  = "/drone_1/mission_planner_ros/clear_waypoints"
    
    activate_planner_service = rospy.ServiceProxy(activate_planner_url, SetBool)
    add_waypoint_service     = rospy.ServiceProxy(add_waypoint_url, WaypointSrv)
    clear_waypoints_service  = rospy.ServiceProxy(clear_waypoints_url, Empty)    
    
    # Taking off the drone
    try:
        take_off            = TakeOffRequest()
        take_off.height     = 1
        take_off.blocking   = False
        take_off_service(take_off)
    except rospy.ServiceException, e:
        print "Service call failed: %s" %e
    
    raw_input("Put ENTER to order the first command: add one waypoint")
    add_waypoint_req = WaypointSrvRequest()
    
    add_waypoint_req.waypoint.pose.pose.position.x   = 15
    add_waypoint_req.waypoint.pose.pose.position.y   = 35
    add_waypoint_req.waypoint.pose.pose.position.z   = 10
    
    add_waypoint_req.waypoint.twist.twist.linear.x  = 1.5
    add_waypoint_req.waypoint.twist.twist.linear.y  = 2.5
    add_waypoint_req.waypoint.twist.twist.linear.z  = 0.5
        
    try:
        add_waypoint_service(add_waypoint_req)
    except:
        print("Failed calling add_waypoint service")
    
    raw_input("Put ENTER to order the second command: add two more waypoints")
    
    # First waypoint
    add_waypoint_req.waypoint.pose.pose.position.x   = 3
    add_waypoint_req.waypoint.pose.pose.position.y   = 42
    add_waypoint_req.waypoint.pose.pose.position.z   = 6
    
    add_waypoint_req.waypoint.twist.twist.linear.x  = 1.0
    add_waypoint_req.waypoint.twist.twist.linear.y  = 3.5
    add_waypoint_req.waypoint.twist.twist.linear.z  = 0.2
        
    try:
        add_waypoint_service(add_waypoint_req)
    except:
        print("Failed calling add_waypoint service")
    
    time.sleep(1)
    # Second waypoint
    add_waypoint_req.waypoint.pose.pose.position.x   = 7
    add_waypoint_req.waypoint.pose.pose.position.y   = 12
    add_waypoint_req.waypoint.pose.pose.position.z   = 14
    
    add_waypoint_req.waypoint.twist.twist.linear.x  = 0.2
    add_waypoint_req.waypoint.twist.twist.linear.y  = -2.5
    add_waypoint_req.waypoint.twist.twist.linear.z  = 0.2
        
    try:
        add_waypoint_service(add_waypoint_req)
    except:
        print("Failed calling add_waypoint service")
    
    raw_input("Put ENTER to order the third command: clear all the waypoints called")
    
    try:
        clear_waypoints_service()
    except:
        print("Failed calling clear_waypoints service")