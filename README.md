# eDO_automatic_controller

This package is an upper layer allowing to easily control the robot using Moveit.

## How to install it ?

### Python dependencies

This package uses the python packages :
- numpy
- numpy-quaternion

### Simulation

Clone the directory into your catkin directory.
Your workspace should contain the following packages if you use the simulation
| repository | repository url |
| - | - |
| edo_gripper | https://github.com/Pro/edo_gripper |
| edo_gripper_moveit | https://github.com/Pro/edo_gripper_moveit |
| edo_gazebo | https://github.com/Pro/edo_gazebo |
| edo_description | https://github.com/Pro/eDO_description |
| geometry_representation | https://github.com/Bracewind/geometry_representation 

### Real robot

Clone the directory into your catkin directory.
Your workspace should contain the following packages if you use the simulation
| repository | repository url |
| - | - |
| edo_control/edo_control_v3 | https://github.com/ymollard/eDO_control or https://github.com/Bracewind/eDO_control_v3 |
| edo_moveit | https://github.com/Pro/eDO_moveit |
| edo_core_msgs | https://github.com/Comau/eDO_core_msgs |
| geometry_representation | https://github.com/Bracewind/geometry_representation  |

To connect to the real robot, use the description given in the edo_control repository

## How to use it ?

### Simulation

- launch the file init_simulation.launch
- wait that everything has started
- launch your code

### Real robot

- Calibrate your eDO using the application
- Configure your terminal (see the file init_connexion.bash for more details)
- launch the file init_connexion.launch
- wait that everything has started
- launch your code

## How to write your own code ?

The code written should only use the function given by the EdoAbstractClass

The class EdoDummy is a stub used to test programs using the EdoAbstractClass
The class MoveitSimulator is used for controlling the robot with the simulator
The class MoveitController is used for controlling the real robot

Code examples on how to write a simple program using the controllers are given in the folder examples


