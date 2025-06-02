
# Overview & Usage
A Model Predictive Controller for Unitree A1.

This MPC controller is based on the following open-source projects:
1. MIT Cheetah's single rigid body dynamics equations
2. Quadprog++ solver
3. Unitree's open-source project [Unitree_Guide](https://github.com/unitreerobotics/unitree_guide/tree/main/unitree_guide) for model and data interface.
 
The controller locates at [State_MPC.cpp](https://github.com/PMY9527/MPC-Controller-for-Unitree-A1/blob/main/src/FSM/State_MPC.cpp) and [State_MPC.h](https://github.com/PMY9527/MPC-Controller-for-Unitree-A1/blob/main/include/FSM/State_MPC.h).


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
<p align="center">
  <img src="https://github.com/PMY9527/mpc-project/blob/main/StairsDEMO.gif" alt="Illustration" style="width: 80%; display: inline-block;">
  <img src="https://github.com/PMY9527/mpc-project/blob/main/PushDEMO.gif" alt="Illustration1" style="width: 80%; display: inline-block;">
</p>



## Tested Benchmarks
1. ~ 300 HZ at 10 Prediction Horizon 
2. Climbing stairs at 5 cm height and 20 cm length with a high chance of success.
3. Top speed ~ 1.7 m/s

# Dynamics & Controller design

### Translational Motion

For the translational motion of the quadruped robot, according to Newton's second law:

$$m\ddot{p} = f_1 + f_2 + f_3 + f_4 - mg$$

where $\ddot{p}$ represents the center of mass acceleration. This can be rearranged as:

$$p'' = \frac{\sum_{i=1}^{n} f_i}{m} - g \quad \text{(Equation 1)}$$

### Rotational Motion

For the rotational motion of the quadruped robot:

$$\frac{d}{dt}(I_s \omega) = I_s \omega' + \omega \times I_s \omega = \sum_{i=1}^{n} r_i \times f_i \quad \text{(Equation 2)}$$

Simplifying by ignoring $\omega \times I_s \omega$ (gyroscopic torque):

$$\frac{d}{dt}(I_s \omega) \approx I_s \omega' \quad \text{(Equation 3)}$$

Combining equations (2) and (3):

$$\frac{d}{dt}(\omega) = \sum_{i=1}^{n} I_s^{-1} r_i \times f_i \quad \text{(Equation 4)}$$

where $r$ and $f$ are the vectors from foot to center and foot force vectors, respectively.

### Coordinate Transformation

Motion in the body coordinate system can be transformed to the world coordinate system by left-multiplying with a transformation matrix $\mathbf{R}$:

$$\mathbf{R} = \mathbf{R}_z(\psi) \mathbf{R}_y(\theta) \mathbf{R}_x(\phi) \quad \text{(Equation 5)}$$

The angular velocity in the world coordinate system can be expressed as:

$$\omega = \begin{bmatrix}
\cos(\theta)\cos(\psi) & -\sin(\psi) & 0 \\
\cos(\theta)\sin(\psi) & \cos(\psi) & 0 \\
-\sin(\theta) & 0 & 1
\end{bmatrix} \begin{bmatrix}
\dot{\phi} \\
\dot{\theta} \\
\dot{\psi}
\end{bmatrix} \quad \text{(Equation 6)}$$

When $\mathbf{R}$ is full rank, the inverse relationship can be obtained:

$$\begin{bmatrix}
\dot{\phi} \\
\dot{\theta} \\
\dot{\psi}
\end{bmatrix} = \begin{bmatrix}
\cos(\psi)/\cos(\theta) & \sin(\psi)/\cos(\theta) & 0 \\
-\sin(\psi) & \cos(\psi) & 0 \\
\cos(\psi)\tan(\theta) & \sin(\psi)\tan(\theta) & 1
\end{bmatrix} \omega \quad \text{(Equation 7)}$$

### Simplified Model

Based on the condition that pitch and roll are close to 0, we can simplify:

$$\frac{d}{dt}(\Theta) = \begin{bmatrix}
\dot{\phi} \\
\dot{\theta} \\
\dot{\psi}
\end{bmatrix} \approx \begin{bmatrix}
\cos(\psi) & \sin(\psi) & 0 \\
-\sin(\psi) & \cos(\psi) & 0 \\
0 & 0 & 1
\end{bmatrix} \omega = \mathbf{R}_z^T(\psi) \omega \quad \text{(Equation 8)}$$

### Complete Dynamics Model

From equations (1), (2), and (8), we can derive the following fifth-order dynamics model:

$$\left\{
\begin{array}{l}
\frac{d}{dt}\boldsymbol{\Theta} = \mathbf{R}_z^T(\psi) \boldsymbol{\omega} \\
\frac{d}{dt}\mathbf{p} = \dot{\mathbf{p}} \\
\frac{d}{dt}\boldsymbol{\omega} = \sum_{i=1}^{4} \mathbf{I_s}^{-1}[\mathbf{r}_i]_{\times} \mathbf{f_i} \\
\frac{d}{dt}\dot{\mathbf{p}} = \sum_{i=1}^{4} \mathbf{f_i}/m - \mathbf{g} \\
\frac{d}{dt}(-\mathbf{g}) = 0
\end{array}
\right.$$

This can be organized into the final simplified single rigid body dynamics equation:

$$\frac{d}{dt} \begin{bmatrix}
\boldsymbol{\Theta} \\
\mathbf{p} \\
\boldsymbol{\omega} \\
\dot{\mathbf{p}} \\
-\mathbf{g}
\end{bmatrix} = \mathbf{A_c} \begin{bmatrix}
\boldsymbol{\Theta} \\
\mathbf{p} \\
\boldsymbol{\omega} \\
\dot{\mathbf{p}} \\
-\mathbf{g}
\end{bmatrix} + \mathbf{B_c} \begin{bmatrix}
\mathbf{f}_1 \\
\mathbf{f}_2 \\
\mathbf{f}_3 \\
\mathbf{f}_4
\end{bmatrix}$$

This can be written as $\dot{x} = A_c x + B_c u$.

### Discretization

The system can be discretized using the forward Euler method:

$$\dot{x} = \frac{x_{k+1} - x_k}{dt}$$

Therefore, the discretized matrices $A_D$ and $B_D$ can be written as:
- $A_D = I + dt \cdot A_c$
- $B_D = dt \cdot B_c$


# Controller Design

The MPC objective is to optimize the control input $u$ to minimize the cost function $J$:

$$\min_{\mathbf{u}} \boldsymbol{J} = \sum_{k=1}^{N} \left( \mathbf{X}^T Q \mathbf{X} + \mathbf{u}^T R \mathbf{u} \right) \quad \text{(Equation 9)}$$

### Quadratic Programming Formulation

This needs to be converted to the quadratic programming form suitable for QP solvers. This project uses the **Quadprog++** solver, which has the form:

$$\min_{\mathbf{x}} J = \frac{1}{2} \mathbf{x}^T G \mathbf{x} + g_0^T \mathbf{x}$$

Subject to:
- $\mathbf{CE}^T \mathbf{x} + \mathbf{ce}_0 = \mathbf{0}$ (equality constraints)
- $\mathbf{CI}^T \mathbf{x} + \mathbf{ci}_0 \geq \mathbf{0}$ (inequality constraints)

### Prediction Model

The system equation can obtain future system states through the following method. Substituting $x_{k+1} = A_D x_k + B_D u_k$ into time $k+2$:

$$x_{k+2} = A_D^2 x_k + A_D B_D u_k + B_D u_{k+1}$$

For prediction horizon $n$:

$$\begin{bmatrix}
x_{k+1|k} \\
x_{k+2|k} \\
\vdots \\
x_{k+n|k}
\end{bmatrix} = \begin{bmatrix}
A_D \\
A_D^2 \\
\vdots \\
A_D^n
\end{bmatrix} x_k + \begin{bmatrix}
B_D & 0 & \cdots & 0 \\
A_D B_D & B_D & \cdots & 0 \\
\vdots & \vdots & \ddots & \vdots \\
A_D^{n-1} B_D & A_D^{n-2} B_D & \cdots & B_D
\end{bmatrix} \begin{bmatrix}
u_{k|k} \\
u_{k+1|k} \\
\vdots \\
u_{k+n-1|k}
\end{bmatrix}$$

This can be simplified as $x = A_{qp} x_0 + B_{qp} u$.

### Cost Function Reformulation

Let $\mathbf{X} = x - x_{ref} = A_{qp} x_0 + B_{qp} u - x_{ref}$ in cost function (9):

$$\min_u \boldsymbol{J}(u) = \mathbf{X}^T Q \mathbf{X} + \mathbf{u}^T R \mathbf{u}$$

After simplification and removing constant terms:

$$\min_u \boldsymbol{J}(u) = \frac{1}{2} u^T \underbrace{(2B_{qp}^T \mathbf{Q} B_{qp} + \mathbf{R})}_{\text{Hessian, } G} u + \underbrace{[2(A_{qp} x_0 - x_{ref})^T \mathbf{Q} B_{qp}]^T}_{\text{Gradient}^T, g_0^T} u$$

## MPC Constraint Design

Constraints are set separately for stance legs and swing legs.

### Swing Leg Constraints

For swing legs, their foot forces are set to zero:

$$\begin{bmatrix}
1 & 0 & 0 \\
0 & 1 & 0 \\
0 & 0 & 1
\end{bmatrix} \begin{bmatrix}
f_x \\
f_y \\
f_z
\end{bmatrix} = \begin{bmatrix}
0 \\
0 \\
0
\end{bmatrix}$$

### Stance Leg Constraints

For stance legs, constraints ensure no slipping and $f_z < 180N$ (for stability). For each stance leg:

$$\begin{bmatrix}
1 & 0 & \mu \\
-1 & 0 & \mu \\
0 & 1 & \mu \\
0 & -1 & \mu \\
0 & 0 & -1
\end{bmatrix} \begin{bmatrix}
f_x \\
f_y \\
f_z
\end{bmatrix} \geq \begin{bmatrix}
0 \\
0 \\
0 \\
0 \\
-180
\end{bmatrix}$$

where $\mu$ is the friction coefficient.
