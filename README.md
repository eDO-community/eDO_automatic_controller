# eDO_automatic_controller

This package is an upper layer allowing to easily control the robot using Moveit.

## Installing using docker (simulation only)

The catkin repository will be created in the container, so do not clone this folder inside your catkin repo

Before doing anything, you must add execution permission to the xserver to run GUI application in docker
In your terminal run <br />
`xhost +`

In the .docker folder, execute the command line <br />
`docker-compose build`

Then execute <br />
`docker-compose up`

The docker instance is up and a name <the_container_name> has been given to it. You should see <br />
`Attaching to <the_container_name> in your terminal.` <br />
For instance  <br />
`Attaching to docker_edo_gazebo_1 in your terminal.`

now you can access open a new terminal inside the container using <br />
`docker exec -it <the_container_name> /bin/bash` <br />
Let's suppose <the_container_name> is docker_edo_gazebo_1. Then the command to enter is <br />
`docker exec -it docker_edo_gazebo_1 /bin/bash` 

Open 2 terminals using <br />
`docker exec -it <the_container_name> /bin/bash`

In the first terminal run the make_workspace.sh script. This file will just build the catkin repository using catkin build and add to your .bashrc what you need
If this does not work, make sure you set the permissions to run the script <br />
`./edo_automatic_simulator/make_workspace.sh`

run the command <br />
`roslaunch edo_automatic_simulation init_simulation.launch`
This will launch the gazebo simulation and moveit server made by Stefan Profanter

In the second, you can launch your own script to control the robot. For instance, enter the command <br />
`rosrun edo_automatic_simulation move_down.py`

Summary :
Terminal 1 <br />
`xhost +` <br />
`docker-compose build` <br />
`docker-compose up` <br />
Terminal 2 <br />
`docker exec -it <the_container_name> /bin/bash` <br />
`./edo_automatic_simulator/make_workspace.sh` <br />
`roslaunch edo_automatic_simulation init_simulation.launch` <br />
Terminal 3 <br /> 
`rosrun edo_automatic_simulation move_down.py`<br />

## Installing on your workstation

### ROS dependencies

This package does not need more dependencies than the packages [edo_control](https://github.com/ymollard/eDO_control) or [edo_gazebo](https://github.com/Pro/edo_gazebo)

For the previous packages, at least Moveit is necessary.

### Python dependencies

This package uses the python packages :

- numpy
- numpy-quaternion

### Simulation

Clone the directory into your catkin directory.
Your workspace should contain the following packages
| repository | repository url |
| - | - |
| edo_gripper | https://github.com/Pro/edo_gripper |
| edo_gripper_moveit | https://github.com/Pro/edo_gripper_moveit |
| edo_gazebo | https://github.com/Pro/edo_gazebo |
| edo_description | https://github.com/Pro/eDO_description |

### Real robot

Clone the directory into your catkin directory.
Your workspace should contain the following packages
| repository | repository url |
| - | - |
| edo_control/edo_control_v3 | https://github.com/ymollard/eDO_control or https://github.com/Bracewind/eDO_control_v3 |
| edo_moveit | https://github.com/Pro/eDO_moveit |
| edo_core_msgs | https://github.com/Comau/eDO_core_msgs |

To connect to the real robot, use the description given in the edo_control repository

## How to use it ?

### Simulation

- launch the file init_simulation.launch
- wait that everything has started
- launch your code

### Real robot

- Calibrate your eDO using the application
- Configure your terminal (see [this file](bash_configuration.sh) for more details)
- launch the file init_connexion.launch
- wait that everything has started
- launch your code

## How to write your own code ?

The code written should only use the functions given by the [EdoAbstractClass](https://github.com/Bracewind/eDO_automatic_controller/blob/b3949376dedb331be9029d6bc70fc3a187d9a434/src/edocontroller/edo_abstract_class.py#L7).

- The class EdoDummy is a stub used to test programs using the EdoAbstractClass
- The class MoveitSimulator is used for controlling the robot with the simulator
- The class MoveitController is used for controlling the real robot

Code examples on how to write a simple program using the controllers are given in the [example](src/example) folder
