# ROS-RBX1-software
Repository for the integration of the rbx1 robot arm with the Robot Operation System.

ROS Controlled RBX1
===================================

Requirements
---------------------
- Raspberry Pi with ROS distribution installed. Link of ubiquity distro at bottom of readme.
- Machine running Ubuntu 16 with ROS Kinetic installed. Tutorial in link at bottom.
- Python 2.7 and 3.5
    - PyZMQ
    - Slush
    - PyQt 5
- Some other dependencies I probably forgot.

Structure
----------------------
```bash
├── demo                    # Some example files for demonstration.
├── docs                    # Documentation files (mostly example of ROS packets)
├── moveo_ws                # ROS workspace location.
│   ├── moveo_moveit        # Some scripts and message definitions.
│   ├── moveo_moveit_config # Configuration and most of the *.launch files
│   ├── moveo_urdf          # Contains models and description of the robot in real life sizes
├── rbx1                    # Source for Python application controlling the RBX1
│   ├── controller          # Code for the controllers (motor, gripper)
│   ├── ros                 # Code for the UI that allow remote commands to be sent to RViz
│   ├── threads             # Mostly the code for the bridge that allow communications between Python 2.7 and 3.5
│   ├── ui                  # *.ui and compiled python for PyQT UI files.
├── LICENSE
└── README.md
```

How to start
----------------------
1. Configure the main computer for ROS Kinetic. Use ROS tutorials.

2. Connect machine and Pi on same network so that they can 'talk' to each other through SSH at the minimum.

3. Compile moveo_ws using the ROS instructions. (catkin build)

4. Edit launch file src/moveo_moveit_config/launch/robot.launch and modify those lines to reflect your information.
-  < arg name="remote_address" default="piarm" />
-  < arg name="username" default="ubuntu" />
-  < arg name="password" default="ubuntu" />
-  < arg name="env-loader" default="/home/ubuntu/RBX1/setup_source.sh" />
-  < machine name="piarm" user="$(arg username)" address="$(arg remote_address)" default="false" password="$(arg password)" env-loader="$(arg env-loader)"/>

5. On computer that runs ROS, use either of the command to run a simulation or
to execute code on the arm.

- roslaunch moveo_moveit_config demo.launch; or
- roslaunch moveo_moveit_config robot.launch

6. On the Raspberry Pi, run ./main.py to launch the control center. 
You can control the motors directly from here and a few other functions.
At this point, if you run 'rostopic list' on the Rpi, you should see multiple topics that weren't there initially.

Useful commands
----------------------

See list of commands that are useful in debugging the code
or provide good information:

- rostopic list
- rostopic echo [topic]

Use ROS with Python 3.5
-------------------------
Link: https://medium.com/@beta_b0t/how-to-setup-ros-with-python-3-44a69ca36674

Commands:
- catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.5m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.5m.so
- catkin config --install
- catkin build

Motor data
--------------------------
2x Nema 23

Connection:
    Black(A+), Green(A-), Red(B+), Blue(B-)
    
Useful links
----------------
- ROS Kinetic install guide - http://wiki.ros.org/kinetic/Installation/Ubuntu
- ROS Moveit tutorial - http://docs.ros.org/kinetic/api/moveit_tutorials/html/index.html
- Moveo ROS integration. (credits) - https://github.com/jesseweisberg/moveo_ros
- RPi image for ROS distro - https://downloads.ubiquityrobotics.com/pi.html
- 

