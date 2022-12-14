Here we explain in deep what each parameter of mission_planner.yml does:

horizon_length      --> indicate an integer that depicts the number of steps (points) that the reference trajectory will generate.
                        This number is true in the case of the leader UAV, but not in the follower as this has to discard obsolete
                        points.

n_drones            --> indicate an integer that depicts the number of drones that the mission_planner will work with.
                        It is very important that this number concordates with the number of drones that Gazebo will launch.

leader_id           --> indicate an integer that depicts the leader_id.
                        Not advisable to change this value of "1".

step_size           --> indicate a float that depicts the time (seconds) that each step will have.

operation_mode      --> indicate an integer from 1 to 4 that depicts the Operation Mode.
                        1.  Non-stopping Operation Mode: This mode forces the formation to not stop in any case as long as they have
                        waypoints to follow.
                        2.  Smooth Operation Mode: This mode make the formation to stop only when there is a change of direction along
                        the Operation. This means that if, when the formation reaches a waypoint and the shortest path to the next
                        waypoint is in the opposite direction, it will first stop and, then, go to the waypoint.
                        3.  Stopping Operation Mode: This mode make the formation stop in every case when it reaches a waypoint.
                        4.  Inspection Operation Mode: This mode make the formation stop in every case when it reaches a waypoint. However,
                        as a difference with Stopping Operation Mode, when the formation reaches the waypoint, you can change the
                        references of inspection distance and formation angle.

planning_rate       --> indicate a float that depicts the planning rate (seconds).
                        This parameter can not have any random value. It will depend mainly on your computer hardware and on what you
                        are launching. In any case, it is advisable to have this value to "1" as minimum.

visualization_rate  --> indicate a float that depicts the visualization rate (seconds).
                        This rate is used to refresh some of the content of the RViz environment.
                        It is advisable to have this value to "2" as minimum.

clock_rate          --> indicate a float that depicts the rate (seconds) of the current_time_ refresh.
                        It is very important that this value is, at least, 2 times minor to the step_size value.

topics_rate         --> indicate a float that depicts the rate (seconds) of the publication of the inspection_distance and formation_angle
                        topics.
                        It is advisable to have this value to step_size as minimum in order to get enough data to plot this information
                        with accuracy.

vel_max             --> indicate a float that depicts the maximum cruising speed (m/s) of the formation.
                        This is the value of the desired maximum cruising speed.

vel_min             --> indicate a float that depicts the minimum cruising speed (m/s) of the formation.
                        This is the value of the desired minimum cruising speed.

orbit_time          --> indicate a float that depicts the time (s) that the formation has to last to describe a full circle around the structure.
                        This time can be chosen independent of the inspection distance and this is used to adjust the cruising speed regarding
                        the inspection distance, the cruising speed has a minimum value (vel_min) and a maximum value (vel_max).

vel_inspect         --> indicate a float that depicts the cruising speed (m/s) of the formation when inspecting.

acc_max             --> indicate a float that depicts the maximum acceleration (m/s??) of the formation.

inspection_dist     --> indicate a float that depicts the desired inspection distance (m).
                        This will be the reference of the inspection distance that the formation has to follow.

inc_distance        --> indicate a float that depicts the value of increment/decrement of the desired inspection distance (m).
                        This is a incremental value that increments or decrements the value of the inspection distance each time you
                        press a key in the joystick simulator (operator_interface).
                        Press "W" to increment inc_distance meters, press "S" to decrement inc_distance meters.

inc_angle           --> indicate a float that depicts the value of increment/decrement of the desired formation angle (rad).
                        This is a incremental value that increments or decrements the value of the formation angle each time you
                        press a key in the joystick simulator (operator_interface).
                        Press "D" to increment inc_angle meters, press "A" to decrement inc_angle meters.

frame               --> indicate a string that depicts the frame in which we are working.
                        Not advisable to change this value. Default: "map"

static_map          --> indicate a bool that depicts if you want to use a static map for obstacle avoidance or not.
                        If you want to use this map, it is very important to specify the route of the map in mission_planner launch files.
                        Default: "true"
