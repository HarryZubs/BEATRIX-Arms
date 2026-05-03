This Repo contains all of the required software and hardware to create an Open-Source BEATRIX Robot Arm.

# Hardware 

printFiles contains all of the STL files that are required to be 3D printed for the full robot arm. All prints are required to be 30% infill PLA.
BOM contains all of the parts required to be purchased to complete the robot arm.
buildGuide contains a guide to the assembly of the arm.
schem contains an Altium Designer Schematic file defining the electrical connections for the full system.

# Software

Contains all of the software required to control the arm.

## WSL contains all of the ROS software

This was all ran inside WSL2 in an Ubuntu 20.04 envionment using ros-noetic.
Any ros-noetic build should be functional however Ubuntu 20.04 is reccommended. 
All essnetial packages are required: python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
Additionall ros-pinocchio is required: https://github.com/stack-of-tasks/pinocchio.git

To run the robot arm all of the ros nodes must be running simultaneously in a enviornment which is communcaiting over TCP with the GUI.

## WindowsGUI contains the GUI and communcation software

This is not unique to windows but was developed for a windows envionrment.
The required python packages are: pyserial and coppeliasim_zmqremoteapi_client.
Inside the code the COM port for the ardunio and the address for the communcation with the ROS software must be configured.

## Arduino contains the motor control software

This code must be flashed onto an ardunio. Any ardunio nano should function however it has only been tested with an ardunio nano rev2.
This requires the following package: Adafruit_PWMServoDriver

## Simulator contains the CoppeliaSim model

The file can be opened inside v4.10.0 and can then communciate and be controlled by the rest of the software.

# Testing contains all of the scripts used during development and testing of the arm

IMU Res contains the ardunio, python code and results of measuring the repeatability with the IMU.
Dynamic test evalutes the workspace of the arm under dyanmic conditions
Kinematic eval evlautes the kinematic workspace of the arm when compared to other humanoid kinematic chains
Motion eval evalutes the similarity betwwen motion of the robot arm and a human arm
Payload eval evaluates the reduction in workspace in loaded conditions
Pose gen generates random reachable poses
Rand pose is a ROS node that will communicate with the ROS software to instruct a move to a randomly reachable pose
Rand traj s a ROS node that will communicate with the ROS software to instruct randomly generated trajectory
Traj eval graphs the trajectory for a random pose
