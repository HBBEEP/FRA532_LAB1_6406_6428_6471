# FRA532 Mobile Robot: LAB1 Local Planner

This LAB is part of the FRA532 Mobile Robot course for third-year students at the Institute of Field Robotics (FIBO), King Mongkut’s University of Technology Thonburi (KMUTT).

## Table of Contents

- [Diagram](#Diagram)
- [Forward Kinematics](#Forward_Kinematics)
- [Inverse Kinematics](#Inverse_Kinematics)
- [Wheel Odometry](#Wheel_Odometry)
- [Controller](#Controller)
- [Installation](#Installation)
- [Usage](#Usage)
- [Experiment](#Experiment)
- [Our Team](#Our_Team)

## Diagram

![Mobile Lab1 drawio](https://github.com/HBBEEP/FRA532_LAB1_6406_6428_6471/assets/122891621/8dd7d7ec-869e-4868-9dc4-0bd8894c86ce)

## Forward_Kinematics

for convert wheel velocity to robot twist (linear velocity, angular velocity)

```
        [ v_robot ]  = r * [ 1/2  1/2 ][ w_r_wheel ]
        [ w_robot ]        [ 1/B -1/B ][ w_l_wheel ]
```


## Inverse_Kinematics

for convert robot twist (linear velocity, angular velocity) to wheel velocity

```
        [ w_r_wheel ]  = [ 1/r  B/2r ][ v_robot ]
        [ w_l_wheel ]    [ 1/r -B/2r ][ w_robot ]
```

where

r is wheel radius

B is distance between wheels

v_robot is linear velocity of robot

w_robot is angular velocity of robot

w_r_wheel is angular velocity of robot's right wheel

w_l_wheel is angular velocity of robot's left wheel


## Wheel_Odometry

to get the robot's position

```
dx = linear_velocity * cos(theta) * dt
dy = linear_velocity * sin(theta) * dt
dthata = angular_velocity * dt

x = x + dx
y = y + dx
theta = y + dtheta
```

## Controller

### Pure Pursuit

```


```
### Virtual Force Field (VFF)

```


```

## Installation

Step 1: Clone the repository to the src directory of your workspace. You must unzip and put each folder in the src directory.

```
cd ~/[your_workspace]/src
git clone [https://github.com/kkwxnn/self_balancing.git](https://github.com/HBBEEP/FRA532_LAB1_6406_6428_6471.git)
```
Step 2:  Build "self_balancing" in your workspace.
```
cd ~/[your_workspace]
colcon build 
source install/setup.bash
```
## Usage

Run Simulation

Terminal 1: Launch Gazebo
```
ros2 launch ur5_ros2_gazebo ur5_simulation.launch.py
```
Terminal 2: Launch RVIZ
```
ros2 launch carver_navigation vmegarover_navigation.launch.py
```
Terminal 3: Run velocity_controller
```
ros2 run robot_control velocity_controller.py
```

## Experiment

### Launch Environment in Map

```
ros2 launch multi_turtlebot_sim spawn_environment.launch.py
```
### Launch Test Case (Launch before run velocity_controller)

Test Case 1: No Obstacle

```
ros2 launch multi_turtlebot_sim spawn_testcase_1.launch.py
```

Test Case 2: 1 Static Obstacle

```
ros2 launch multi_turtlebot_sim spawn_testcase_2.launch.py
```

Test Case 3: 2 Static Obstacles

```
ros2 launch multi_turtlebot_sim spawn_testcase_3.launch.py
```

Test Case 4: 1 Dynamics Obstacle

```
ros2 launch multi_turtlebot_sim spawn_testcase_4.launch.py
```

Test Case 5: 1 Dynamics Obstacle with increase speed

```
ros2 launch multi_turtlebot_sim spawn_testcase_5.launch.py
```

Test Case 6: 2 Dynamics Obstacles 

```
ros2 launch multi_turtlebot_sim spawn_testcase_6.launch.py
```

Test Case 7: 2 Dynamics Obstacles with increase speed

```
ros2 launch multi_turtlebot_sim spawn_testcase_7.launch.py
```

### Result


## Our Team

1. Kullakant Keawkallaya 64340500006
2. Thamakorn Thongyod 64340500028
3. Monsicha Sopitlaptana 64340500071
