
## Overview
A Model Predictive Controller for Unitree A1 based on the framework from [unitree_guide](https://github.com/unitreerobotics/unitree_guide/tree/main/unitree_guide) using Quadprog++ as the solver. 
 I have spent weeks writing the C++ codes in an easy-to-read manner, and hopefully I could help people learn something via this project. The controller locates at [State_MPC.cpp](https://github.com/PMY9527/MPC-Controller-for-Unitree-A1/blob/main/src/FSM/State_MPC.cpp) and [State_MPC.h](https://github.com/PMY9527/MPC-Controller-for-Unitree-A1/blob/main/include/FSM/State_MPC.h).


## Environment
My environment is ROS Noetic and Ubuntu 20.04 running in WSL2 with [GUI enabled](https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps), but it should run just fine in ROS Melodic and Ubuntu 18.04.

## Dependencies
Just like unitree_guide, You will need the following:
1.[unitree_guide](https://github.com/unitreerobotics/unitree_guide/tree/main/unitree_guide).
2.[unitree_ros](https://github.com/unitreerobotics/unitree_ros).
3.[unitree_legged_msgs](https://github.com/unitreerobotics/unitree_ros_to_real)(Note that: unitree_legged_real package should not be a part of dependencies)
Put these three packages in the src folder of a ROS workspace. Paste my projects at '~\NAME_OF_YOUR_PROJECT\src\unitree_guide\unitree_guide' and replace whats replicated.

## To Run
1. Head to the project's folder:
```cd NAME_OF_YOUR_PROJECT```
2. build the workspace:
```catkin build```
3. source the workspace:
```source devel\setup.bash```
4. open Gazebo:
```roslaunch unitree_guide gazeboSim.launch```
5. load the controller:
```rosrun unitree_guide junior_ctrl```
   
After starting the controller, the robot will lie on the ground of the simulator, then press the '2' key on the keyboard to switch the robot's finite state machine (FSM) from Passive(initial state) to FixedStand, then press the '6' key to switch the FSM from FixedStand to MPC, now you can press the 'w' 'a' 's' 'd' key to control the translation of the robot, and press the 'j' 'l' key to control the rotation of the robot. Press the Spacebar, the robot will stop and stand on the ground . (If there is no response, you need to click on the terminal opened for starting the controller and then repeat the previous operation)
<p align="center">
  <img src="https://github.com/PMY9527/mpc-project/blob/main/illustration.gif" alt="Illustration">
</p>
VIDEO DEMO: https://www.youtube.com/watch?v=jxqS9TUR0ow 
