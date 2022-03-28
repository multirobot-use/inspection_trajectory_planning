1. Execute install_packages.sh


2. Change the following details to make everything work in ROS Noetic/Ubuntu 20:
    - Go to your_ws/src/inspection_trajectory_planning/packages/Firmware/Tools/sitl_gazebo/include/gazebo_opticalflow_plugin.h and change:
            #define HAS_GYRO TRUE           --> #define HAS_GYRO true

    - Change in your_ws/src/inspection_trajectory_planning/packages/grvc-utils/gazebo_animator/CMakeLists.txt the c++ compiler version:
            add_definitions(-std=c++11)     --> add_definitions(-std=c++17)

    - Create CATKIN_IGNORE in the following backends:
            - your_ws/src/inspection_trajectory_planning/packages/grvc-ual/ual_backend_crazyflie
            - your_ws/src/inspection_trajectory_planning/packages/grvc-ual/ual_backend_dji_ros
            - your_ws/src/inspection_trajectory_planning/packages/grvc-ual/ual_backend_mavlink
            - your_ws/src/inspection_trajectory_planning/packages/grvc-ual/ual_backend_ue

3. Execute install_packages.sh again


4. Do catkin build (maybe twice)


5. Put in terminal echo "source ~/your_ws/devel/setup.bash" >> ~/.bashrc    (or .zshrc)


6. Change #!/usr/bin/env python to #!/usr/bin/env python3 in the following files:
    - ~/your_ws/src/inspection_trajectory_planning/packages/grvc-ual/px4_bringup/scripts/*      (all the scripts in this folder)
    - ~/your_ws/src/inspection_trajectory_planning/packages/grvc-ual/ual_teleop/scripts/*       (all the scripts in this folder)


7. Change the prints on the python files and add the parenthesis.
    - ~/your_ws/src/inspection_trajectory_planning/packages/grvc-ual/px4_bringup/scripts/utils.py
    - ~/your_ws/src/inspection_trajectory_planning/packages/grvc-ual/ual_teleop/scripts/joy_handle.py


8. Make the following changes:
    - In ~/your_ws/src/inspection_trajectory_planning/packages/grvc-ual/px4_bringup/scripts/launch_gzworld.py
            if args.description_package is not "robots_description":    --> if args.description_package != "robots_description":

    - In ~/your_ws/src/inspection_trajectory_planning/packages/grvc-ual/ual_teleop/scripts/joy_handle.py
            joy_msg_map = yaml.load(joy_config)['joy_layout']           --> joy_msg_map = yaml.safe_load(joy_config)['joy_layout']
    
    - In ~/your_ws/src/inspection_trajectory_planning/packages/grvc-ual/ual_teleop/scripts/simulate_safety_pilot.py
            action_map = yaml.load(action_config)['joy_actions']        --> action_map = yaml.safe_load(action_config)['joy_actions']