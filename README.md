# ROS-RBX1-software
Repository for the integration of the rbx1 robot arm with the Robot Operation System.

ROS Controlled RBX1
===================================

Requirements
---------------------
- Raspberry Pi with ROS distribution installed. I recommend https://downloads.ubiquityrobotics.com/pi.html to ease the Pi configuration.
- Machine running Ubuntu 16 with ROS Kinetic installed. Tutorial: http://www.ros.org/install/

Structure
----------------------
```bash
├── demo               # Some example files for demonstration.
├── docs               # Documentation files (mostly example of ROS packets)
├── moveo_ws           # ROS workspace location.
├── rbx1               # Source for Python application controlling the RBX1
├── LICENSE
└── README.md
```

How to start
----------------------
1. Configure the main machine for ROS Kinetic.

2. Connect machine and Pi on same network so that they can connect to each other through SSH at the minimum.

3. Compile moveo_ws using the ROS instructions.

4. Edit launch file src/moveo_moveit_config/launch/robot.launch and modify those lines to reflect your informations.
-  < arg name="remote_address" default="piarm" />
-  < arg name="username" default="ubuntu" />
-  < arg name="password" default="ubuntu" />
-  < arg name="env-loader" default="/home/ubuntu/RBX1/setup_source.sh" />
-  < machine name="piarm" user="$(arg username)" address="$(arg remote_address)" default="false" password="$(arg password)" env-loader="$(arg env-loader)"/>

5. On computer that runs ROS,
use either of the command to run a simulation or
to execute code on the arm.

- roslaunch moveo_moveit_config demo.launch
- roslaunch moveo_moveit_config robot.launch

Useful commands
----------------------

See list of commands that are useful in debugging the code
or provide good information:

- rostopic list
- rostopic echo [topic]


Motor data
--------------------------
2x Nema 23

Connection:
    Black(A+), Green(A-), Red(B+), Blue(B-)

