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
from std_msgs.msg import UInt8
from mission_planner.msg import Float32withHeader
from uav_abstraction_layer.srv import TakeOff
from uav_abstraction_layer.srv import TakeOffRequest
from uav_abstraction_layer.srv import Land
from uav_abstraction_layer.srv import LandRequest
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
from mission_planner.srv import FlightModeSrvRequest
from mission_planner.srv import FlightModeSrv
import signal
import sys
from collections import namedtuple

# FOR KEYBOARD
from pynput import keyboard
global pressed_key

def on_press(key):
    global pressed_key
    try:
        pressed_key = key.char
    except AttributeError:
        pass

def on_release(key):
    global pressed_key
    pressed_key = 'p' # Put a value that does not interfeer with the functionality

listener = keyboard.Listener(
    on_press=on_press,
    on_release=on_release)

listener.start()
# END KEYBOARD


class Drone:
    state = 0
    n_drones = 0

    inspection_distance = 3
    ref_distance = 3

    formation_angle = 1
    ref_angle = 1

    flight_mode = 1

    def __init__(self, drone_ns):
        print("I'm a python constructor")
                
        # Subscribe topics
        for id in drone_ids:
            rospy.Subscriber("/drone_" + str(id) + "/ual/state", State, self.callbackState)
            rospy.Subscriber("/drone_" + str(id) + "/inspection_distance", Float32withHeader, self.callbackInspectionDistance)
            if (drone_ns != "/drone_1"):
                rospy.Subscriber("/drone_" + str(id) + "/formation_angle", Float32withHeader, self.callbackFormationAngle)
        
        rospy.Subscriber("/drone_1/absolute_distance_to_inspect", Float32withHeader, self.callbackRefDistance)
        rospy.Subscriber("/drone_1/absolute_relative_angle", Float32withHeader, self.callbackRefAngle)
        rospy.Subscriber("/drone_1/flight_mode", UInt8, self.callbackFlightMode)
        
        # Publishers (only one drone topic needed for each one)
        self.distance_inspection_pub = rospy.Publisher('/drone_1/mission_planner_ros/distance_to_inspection_point', Bool, queue_size = 1)
        self.relative_angle_pub      = rospy.Publisher('/drone_1/mission_planner_ros/relative_angle', Bool, queue_size = 1)

        # Wait for services
        activate_planner_url = drone_ns + "/mission_planner_ros/activate_planner"
        add_waypoint_url     = drone_ns + "/mission_planner_ros/add_waypoint"
        clear_1_waypoint_url = drone_ns + "/mission_planner_ros/clear_first_waypoint"
        clear_waypoints_url  = drone_ns + "/mission_planner_ros/clear_waypoints"
        change_flight_url    = drone_ns + "/mission_planner_ros/change_flight_mode"
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

        # Clear waypoints service
        rospy.wait_for_service(clear_waypoints_url)
        self.clear_waypoints_service  = rospy.ServiceProxy(clear_waypoints_url, Empty)

        # Clear first waypoint service
        rospy.wait_for_service(clear_1_waypoint_url)
        self.clear_1_waypoint_service = rospy.ServiceProxy(clear_1_waypoint_url, Empty)

        # Change flight mode service
        rospy.wait_for_service(change_flight_url)
        self.change_flight_mode       = rospy.ServiceProxy(change_flight_url, FlightModeSrv)

        # Change point to inspect service
        rospy.wait_for_service(point_to_inspect_url)
        self.point_to_inspect_service = rospy.ServiceProxy(point_to_inspect_url, PointToInspectSrv)

        # Change distance to inspect service
        rospy.wait_for_service(distance_url)
        self.distance_service         = rospy.ServiceProxy(distance_url, DistanceSrv)

        # Change relative angle service
        rospy.wait_for_service(relative_angle_url)
        self.relative_angle_service   = rospy.ServiceProxy(relative_angle_url, AngleSrv)

        # TakeOff service
        rospy.wait_for_service(take_off_url)
        self.take_off_service         = rospy.ServiceProxy(take_off_url, TakeOff)
        
        # Land service
        rospy.wait_for_service(land_url)
        self.land_service             = rospy.ServiceProxy(land_url, Land)

        # GoToWaypoint service
        rospy.wait_for_service(go_to_waypoint_url)
        self.go_to_waypoint_service   = rospy.ServiceProxy(go_to_waypoint_url, GoToWaypoint)

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

    # Take off drones method
    def take_off(self, height, block_take_off):
        try:
            take_off            = TakeOffRequest()
            take_off.height     = height
            take_off.blocking   = block_take_off
            
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

    # Change Flight Mode method
    def change_flight_mode(self, mode):
        try:
            mode_req        = FlightModeSrvRequest()
            mode_req.mode   = mode
            self.change_flight_mode(mode_req)
            print "Flight mode changed!"
            
        except:
            print("Failed calling change_flight_mode service")

    # Go to waypoint method
    def go_to_waypoint(self, waypoint):
        try:
            point              = GoToWaypointRequest()
            point.blocking     = True
            point.waypoint.pose.position.x     = waypoint[0]
            point.waypoint.pose.position.y     = waypoint[1]
            point.waypoint.pose.position.z     = waypoint[2]
            
            self.go_to_waypoint_service(point)
            print "Going to waypoint"
        
        except rospy.ServiceException, e:
            print "Service call failed: %s" %e

    # Start mission method
    def start_mission(self):
        if self.state == State.FLYING_AUTO:
            try:
                req      = SetBoolRequest()
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

    # Stop mission method
    def stop_mission(self):
        try:
            req      = SetBoolRequest()
            req.data = False
            self.activate_planner_service(req)
            print "Mission has been stopped!"
        
        except rospy.ServiceException, e:
            print("Failed calling stop_mission service")
    
    # Clear the first waypoint method (FOR INSPECTION)
    def clear_first_waypoint(self):

        if (self.flight_mode == 4):
            try:
                self.clear_1_waypoint_service()
                print "First waypoint cleared!"
                
            except:
                print("Failed calling clear_first_waypoint service")

    # Clear all waypoints method
    def clear_all_waypoints(self):
        try:
            self.clear_waypoints_service()
            print "Waypoints cleared!"
            
        except:
            print("Failed calling clear_waypoints service")
    
    # Joystick simulator method
    def joystick_simulator(self):
        global pressed_key
        
        # Initialize
        inc_distance = True
        inc_angle    = True
        aux_key      = ' '
        
        while (not (pressed_key == 'q' or pressed_key == 'Q')):
            # Screen refresh
            os.system('cls' if os.name == 'nt' else 'clear')

            print "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
            print " REFERENCE OF INSPECTION DISTANCE (meters): %.2f" %(drones[0].ref_distance)
            for id in drone_ids:
                print " Inspection distance (meters) for Drone %d:  %.3f" %(id, drones[id - 1].inspection_distance)
            print "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
            print " REFERENCE OF FORMATION ANGLE (rad): %.2f" %(drones[0].ref_angle)
            for id in drone_ids:
                if (id != 1):
                    print " Formation angle (rad) for Drone %d:  %.3f" %(id, drones[id - 1].formation_angle)
            print "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
            print "\n"
            print "\t\t ~~~~~ JOYSTICK SIMULATOR ~~~~~\n\n"
            print "\t\tAFTER PRESSING A KEY, THEN RELEASE\n\n"
            print "\tTo increase distance to inspection point, press W\n"
            print "\tTo decrease distance to inspection point, press S\n"
            print "\tTo increase the relative angle, press D\n"
            print "\tTo decrease the relative angle, press A\n"
            print "\tTo quit, press Q\n\n"

            if (self.flight_mode == 4):
                print "\tFORMATION IN INSPECTING MODE!"
                print "\tPress N to go to the next waypoint"

            if (aux_key != pressed_key):
                if (pressed_key == 'w' or pressed_key == 'W'):
                    inc_distance = True
                    self.distance_inspection_pub.publish(inc_distance)
                    
                elif (pressed_key == 's' or pressed_key == 'S'):
                    inc_distance = False
                    self.distance_inspection_pub.publish(inc_distance)
                    
                elif (pressed_key == 'a' or pressed_key == 'A'):
                    inc_angle = False
                    self.relative_angle_pub.publish(inc_angle)
                    
                elif (pressed_key == 'd' or pressed_key == 'D'):
                    inc_angle = True
                    self.relative_angle_pub.publish(inc_angle)
                
                elif ((pressed_key == 'n' or pressed_key == 'N') and (self.flight_mode == 4)):
                    self.clear_1_waypoint_service()
                    print "POINT SKIPPED!"
                    time.sleep(1)
                
                aux_key = pressed_key

            time.sleep(0.08)

        time.sleep(0.2)

    # Tracking screen
    def tracking_screen(self):
        global pressed_key
        angle = 0
        diff_angle = 0
        diff_dist  = 0
        while(not(pressed_key == 'q' or pressed_key == 'Q')):
        # Screen refresh
            os.system('cls' if os.name == 'nt' else 'clear')

            print "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
            print "REFERENCE OF INSPECTION DISTANCE (meters): %.2f" %(drones[0].ref_distance)
            for id in drone_ids:
                diff_dist = abs(drones[id - 1].inspection_distance - drones[0].ref_distance)
                print "         Inspection distance (meters) for Drone %d:  %.3f" %(id, drones[id - 1].inspection_distance)
                print "Error of inspection distance (meters) for Drone %d:  %.3f" %(id, diff_dist)
                print "\n"
            print "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"

            angle = drones[0].ref_angle*(180/3.1415)
            print "REFERENCE OF FORMATION ANGLE (rad): %.2f" %(angle)
            for id in drone_ids:
                if (id != 1):
                    angle = drones[id - 1].formation_angle*(180/3.1415)
                    diff_angle = abs(drones[id - 1].formation_angle - drones[0].ref_angle)*(180/3.1415)
                    print "         Formation angle (degrees) for Drone %d:  %.3f" %(id, angle)
                    print "Error of formation angle (degrees) for Drone %d:  %.3f" %(id, diff_angle)
                    print "\n"
            print "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
            print "\n"
            print "Flight mode: "
            if (self.flight_mode == 1):
                print "\tNon-stopping"
            elif (self.flight_mode == 2):
                print "\tSmooth mode"
            elif (self.flight_mode == 3):
                print "\tStopping mode"
            elif (self.flight_mode == 4):
                print "\tInspection mode"
            
            print "Exit the tracking screen when 'q' or 'Q' key is pressed"

            time.sleep(0.1)
        time.sleep(0.2)

    # Callback state
    def callbackState(self, data):
        self.state = data.state
    
    # Callback for inspection distance
    def callbackInspectionDistance(self, data):
        self.inspection_distance = data.data
    
    # Callback for formation angle
    def callbackFormationAngle(self, data):
        self.formation_angle = data.data

    # Callback for the reference of the inspection distance
    def callbackRefDistance(self, data):
        self.ref_distance = data.data
    
    # Callback for the reference of the formation angle
    def callbackRefAngle(self, data):
        self.ref_angle = data.data
    
    # Callback for the Flight Mode
    def callbackFlightMode(self, data):
        self.flight_mode = data.data 
        

# Menu function
def show_menu(params,drones):

    # Clear the screen
    os.system('cls' if os.name == 'nt' else 'clear')

    # Menu
    print "\n################################################################################"
    print "\n Welcome to the main menu. Put the number of the desired option:\n"
    print "\ta. Take off the drones"
    print "\tb. Start the mission"
    print "\tc. Stop the mission"
    print "\td. Add waypoint"
    print "\te. Clear all the waypoints"
    print "\tf. Change relative angles between followers and leader"
    print "\tg. Change distance to inspection point"
    print "\th. Change inspection point"
    print "\ti. Joystick simulator"
    print "\tj. Send the drones to HOME position"
    print "\tk. Land the drones"
    print "\tl. Send to next waypoint (ONLY IN INSPECTION MODE)"
    print "\tm. Watch the evolution of the inspection distance & formation angle"
    print "\tn. Change flight mode"
    print "\tz. Exit the console\n"
    print "################################################################################"
    
    option = 0

    while (option < (ord('a')) or option > (ord('n'))): # ASCII for make sure there is no error of inputs
        try: 
            option = ord(raw_input (" >> "))
        except:
            pass
        if (option == ord('z')):
            sys.exit(0)

    option = option - ord('a')

    # Take off
    if option == 0: 
        for drone in drones:
            if (drone != n_drones):
                drone.take_off(params.height, False)
            else:
                drone.take_off(params.height, True)

    # Start mission
    elif option == 1:
        for drone in drones:
            drone.start_mission()
    
    # Stop mission
    elif option == 2:
        for drone in drones:
            drone.stop_mission()
    
    # Add waypoint
    elif option == 3:
        px = float(raw_input("X pose (meters): "))
        py = float(raw_input("Y pose (meters): "))
        pz = float(raw_input("Z pose (meters): "))
        wp = [px, py, pz]
        for drone in drones:
            drone.add_one_waypoint(wp)
    
    # Clear all the waypoints
    elif option == 4:
        for drone in drones:
            drone.clear_all_waypoints()
    
    # Change relative angle
    elif option == 5:
        print "Please, put the desired relative angle between follower drones and the leader drone (Manual mode)"
        angle = drones[0].ref_angle*(180/3.1415)
        print "Current formation angle (degrees): %.2f" %(angle)
        angle = (3.1415/180)*float(raw_input("Angle (degrees): "))
        for drone in drones:
            drone.change_relative_angle(angle)
    
    # Change distance to inspection point
    elif option == 6:
        print "Please, put the desired distance to the inspection point (Manual mode)"
        dist = drones[0].ref_distance
        print "Current distance to the inspection point (meters): %.2f" %(dist)
        distance = float(raw_input("Distance (meters): "))
        for drone in drones:
            drone.set_distance_inspection(distance)
    
    # Change inspection point
    elif option == 7:
        print "Please, put the desired inspection point (Manual mode):\n"
        point = [0, 0, 0]
        point[0] = (float(raw_input("X (meters): ")))
        point[1] = (float(raw_input("Y (meters): ")))
        point[2] = (float(raw_input("Z (meters): ")))
        for drone in drones:
            drone.change_inspection_point(point)
    
    # Launch joystick simulator
    elif option == 8:
        drones[0].joystick_simulator()
    
    # Send drones to HOME position
    elif option == 9:
        # It is needed to stop the mission, get closer to the HOME position and then land each one
        # Stop the mission
        print("Stopping the mission...")
        for drone in drones:
            drone.stop_mission()
        time.sleep(2)

        # Send each UAV to their HOME position
        print("Sending each UAV to their HOME position...")
        cont = 0
        for drone in drones:
            drone.go_to_waypoint(params.drone_home[cont])
            cont = cont + 1
        time.sleep(2)

        print("Landing the UAVs...")
        # Land the drones
        for drone in drones:
            drone.land()
    
    # Land the UAVs
    elif option == 10:
        print("Landing the UAVs...")
        # Land the drones
        for drone in drones:
            drone.land()
    
    # Send UAVs to the next waypoint
    elif option == 11:
        print("Sending UAV to the next waypoint...")
        # Land the drones
        for drone in drones:
            drone.clear_first_waypoint()
    
    # Monitor the tracking problem of the inspection distance and the formation angle
    elif option == 12:
        drones[0].tracking_screen()
    
    # Change flight mode
    elif option == 13:
        print("New flight mode\nChoose 1 (NON-STOPPING), 2 (SMOOTH), 3 (STOPPING), 4 (INSPECTING)")
        mode = (int(raw_input(" >> ")))
        for drone in drones:
            drone.change_flight_mode(mode)
            time.sleep(0.5)
        
    else:
        print ("Option '" + chr(option) + "' does not exist!")


# Finish the execution directly when Ctrl+C is pressed (signal.SIGINT received), without escalating to SIGTERM.
def signal_handler(sig, frame):
    print('Ctrl+C pressed, signal.SIGINT received.')
    sys.exit(0)
        

# Auto function: take off the drones, set the parameters up and start the mission of each drone
def auto_function(params,drones):

    # Taking of drones
    cont = 1
    for drone in drones:
        if (cont != n_drones):
            drone.take_off(params.height, False)
        else:
            drone.take_off(params.height, True)
        cont = cont + 1

    time.sleep(0.5)

    # Set up the parameters of each drone
    for drone in drones:
        drone.set_distance_inspection(params.inspect_point[3])
        drone.change_relative_angle(params.relative_angle)
        drone.change_inspection_point(params.inspect_point)
        for wp in params.waypoints:
            drone.add_one_waypoint(wp)
    
    time.sleep(0.5)

    # Starting mission
    for drone in drones:
        drone.start_mission()


# Main function
if __name__ == "__main__":

    signal.signal(signal.SIGINT, signal_handler)    # Associate signal SIGINT (Ctrl+C pressed) to handler (function "signal_handler")

    # Create the node
    rospy.init_node("operator", anonymous=True)

    # Read yml config file
    rospack = rospkg.RosPack()
    f_route = rospack.get_path('mission_planner') + '/config/exp_2drones.yml'
    yml_file    = open(f_route, 'r')
    yml_content = yaml.load(yml_file)

    drone_ids = yml_content.get('drone_ids')
    
    # Create a vector of objects of class Drone
    drones = []
    for id in drone_ids:
        drones.append(Drone("/drone_" + str(id)))
    
    n_drones = len(drones)
    
    # Parameters setup
    params = namedtuple('params', 'auto height inspect_point waypoints relative_angle drone_home')
    params.auto                      = yml_content.get('auto')
    params.height                    = yml_content.get('take_off_height')
    params.waypoints                 = yml_content.get('waypoints')
    params.inspect_point             = yml_content.get('inspect')
    params.relative_angle            = yml_content.get('relative_angle')

    params.drone_home = []
    for id in drone_ids:
        params.drone_home.append(yml_content.get('drone' + str(id) + '_home'))

        # Give some additional height to the home positions
        params.drone_home[id - 1][2] = params.drone_home[id - 1][2] + 0.5 

    # Check all drones are landed armed. Know if we are reconnecting by studying if the drone is already flying
    cont = 0
    reconnect = 0
    while cont != n_drones:
        cont = 0
        for drone in drones:
            # Reconnect case
            if drone.state >= 3:
                reconnect = 1
                cont += 1 
            # Initial case
            elif drone.state == State.LANDED_ARMED:
                cont += 1
        time.sleep(0.5)

    if (reconnect == 1):
        print "Reconnecting..."
        time.sleep(1)
    
    # Auto mode
    if (params.auto and (reconnect == 0)):
        # raw_input(">> Press any key to start the AUTO interface ")
        print "Using the automatic interface"
        auto_function(params, drones)

    # Menu mode
    while (not rospy.is_shutdown()):
        show_menu(params, drones)
        print("PRESS ENTER")
        raw_input(" >>")
        time.sleep(1)