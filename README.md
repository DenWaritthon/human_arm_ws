# 7 DOF MANIPULATOR SIMULATES THE JOINT PATTERN OF A HUMAN ARM KINEMATICS SIMULATION

This project simulate the human arm movement from the shoulder to wrist  that was 3D visualized by RVIZ in ROS2.


# Table of Contents
- [User installation guide](#user-installation-guide)
  - [Prerequisites](#prerequisites)
  - [Install project workspace](#install-project-workspace)
- [Methodology](#methodology)
  - [MDH - Parameter](#mdh---parameter)
  - [Psudoinverse jacobian](#psudoinverse-jacobian)
  - [Joint effort](#joint-effort)
- [System architecture](#system-architecture)
- [User guide](#user-guide)
  - [How to use simulation](#how-to-use-simulation)
  - [How to use input UI](#how-to-use-input-ui)
  - [How to call simulation by service](#how-to-call-simulation-by-service)
- [Demos and Result](#demos-and-result)
- [Conclusion](#conclusion)
- [Future plan](#future-plan)
- [Developer Member](#developer-member)
  
# User installation guide 

## Prerequisites

Ensure you have the following dependencies installed

**Middle ware**
- `ROS2 Humble`

**Python Library**
- `roboticstoolbox`
- `spatialmath`
- `numpy`
- `scipy`
- `pygame`


## Install project workspace
Clone this workspace

```bash
git clone https://github.com/PoFamily/human_arm_ws.git
```

# Methodology

## MDH - Parameter

It was a paratemter the describe positioning of Joint and the table below is our MDH-Parameter of the human arm model.

| Joint | $\alpha_{i-1}$ |  $a_{i-1}$  |  $\theta_i$  | $d_i$ |
| ----- | - | - | - | - |
|   1   | 0.0 | $\pi/2$  | 0.0 | $q_1$         |
|   2   | 0.0 | $\pi/2$  | 0.0 | $q_2 + \pi/2$ |
|   3   | 0.0 | $-\pi/2$ | 0.0 | $q_3 - \pi/2$ |
|   4   | 0.4 | $-\pi/2$ | 0.0 | $q_4 - \pi/2$ |
|   5   | 0.0 | $-\pi/2$ | 0.4 | $q_5$         |
|   6   | 0.0 | $\pi/2$  | 0.0 | $q_6 + \pi/2$ |
|   7   | 0.0 | $\pi/2$  | 0.0 | $q_7$         |

## Psudoinverse jacobian

Use to find movement velocity of each joint by using Pseudoinverse method of Jacobian matrix by this equation

```math
\dot{q} = J^{\dagger}(q)\dot{x}
```
"Define"
- $\dot{q}$ represent velocity in each joint

- $\dot{x}$ represent velocity of end-effector 
- $J^{\dagger}(q)$ represent Pseudoinverse method of Jacobian matrix of each joint that was at angle q

By $J^{\dagger}(q)$  or Pesudoinverse method of jacobian matrix from

```math
J^{\dagger} = J^T(JJ^T)^{-1}
```

Define
- $J$ represent Jacobian matrix at each joint at any angle of q

## Joint effort

Find Joint Effort from equation of Static Force by this equation

```math
\tau = J^T(q)w
```

Define
- $\tau$ represent Joint Efort Value
- $w$ represent Wrench value that effect to the end-effector

# System architecture 

![System architecture](<picture/System architecture .png>)

System seperate the work into sub-node that work differently 4 node considt of

- Input_node is a node that display UI for input value to control movement of the model and display joint effort value from the calculation

- Controller_node is a node that command the control to working in various by recieve input from Input_node and then send the command to Jointstate_node 

- Jointstate_node is a node that command movement of each joint from the Controller_node

- Joint_state_publisher is a node for control movement of the model that display on the RVIZ

# User guide

After Clone workspace then Build and source

```
cd human_arm_ws/
colcon build
source install/setup.bash
```

## How to use simulation
Run the simulation in ROS2

**Run controller system**
```
ros2 launch human_arm_simulation human_arm_controller.launch.py
```

**Run Input UI**
```
ros2 run human_arm_simulation input_node.py
```

## How to use input UI

**MoveJ**
![InputUI-1](picture/InputUI-1.jpg)

Able to input data 2 ways 
1. Input configulation space of each joint by using slide bar or Input the value int the box (As shown in the picture 1)  
2. Input task space value of the target by input the value in the box (As shown in the picture 2)

**MoveL**
![InputUI-2](picture/InputUI-2.jpg)

Able to input data 2 ways 
1. Input task space value of the target by input the value in the box (As shown in the picture 3)
2. Input wrench value that effect to end-effector in each axis by input value in each box and the result of the joint effort calculation will be on the side (As shown in the picture 4)

**Usage Step**
1. Select Move Mode on the top which Mode you want to use MoveJ or MoveL
2. Specify the value to move
3. Click Move or Calculate

## How to call simulation by service

Able to use call service in various format of ROS2 to call by the structure of each service as follows
![custom service](<picture/custorm service.png>)

# Demos and Result

https://github.com/user-attachments/assets/915b964d-cd14-45ad-b883-2db0a28bd911

# Conclusion
Simulation the movement of the human arm using ROS2 Humble and use various python libraries for instance roboticstoolbox, spatialmath, numpy, scipy and pygame for the user interface. This simulation simulate the human arm from the shoulder to the wrist. And 3D visualized by RVIZ in ROS2 that can control manually by input or using the user interface for the easier use.

But the thing that we have not been done is Test and Validate compare to the real human arm so it will be in our future plans.  

# Future plan
- Control joint limit to move to the target
- Test and Validate compare to real human arm
- Add the hand control
   
# Developer Member

- Waritthon Kongnoo
- Chayanat Lertwittayanuruk
- Chawaphon Wachiraniramit
- Kraiwich Vichakhot
