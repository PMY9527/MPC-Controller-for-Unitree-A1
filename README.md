
# Overview & Usage
A Model Predictive Controller for Unitree A1.

This MPC controller is based on the following open-source projects:
1. MIT Cheetah's single rigid body dynamics equations
2. Quadprog++ solver
3. Unitree's open-source project [Unitree_Guide](https://github.com/unitreerobotics/unitree_guide/tree/main/unitree_guide) for model and data interface.
 
The controller locates at [State_MPC.cpp](https://github.com/PMY9527/MPC-Controller-for-Unitree-A1/blob/main/src/FSM/State_MPC.cpp) and [State_MPC.h](https://github.com/PMY9527/MPC-Controller-for-Unitree-A1/blob/main/include/FSM/State_MPC.h).

## Note
This is a basic MPC controller implementation for quadrupeds. Also works for other models, of which I've tested with go1.

## Environment
My environment is ROS Noetic and Ubuntu 20.04, but it should run just fine in ROS Melodic and Ubuntu 18.04.

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
```sudo ./devel/lib/unitree_guide/junior_ctrl```
   
After starting the controller, the robot will lie on the ground of the simulator, then press the '2' key on the keyboard to switch the robot's finite state machine (FSM) from Passive(initial state) to FixedStand, then press the '6' key to switch the FSM from FixedStand to MPC, now you can press the 'w' 'a' 's' 'd' key to control the translation of the robot, and press the 'j' 'l' key to control the rotation of the robot. Press the Spacebar, the robot will stop and stand on the ground . (If there is no response, you need to click on the terminal opened for starting the controller and then repeat the previous operation)


## Tested Benchmarks
1. ~ 300 HZ at 10 Prediction Horizon 
2. Climbing stairs at 5 cm height and 20 cm length with a high chance of success.
3. Top speed ~ 1.7 m/s
<div align="center" style="display: flex; justify-content: center; gap: 200px;">
  <img src="https://github.com/PMY9527/mpc-project/blob/main/StairsDEMO.gif" alt="Illustration" style="width: 500px;">
  <img src="https://github.com/PMY9527/mpc-project/blob/main/PushDEMO.gif" alt="Illustration1" style="width: 500px;">
</div>

## Potential Improvements
Add warmstart for the solver + slope estimation.


