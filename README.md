# FRA532 Mobile Robot: LAB1 Local Planner

This LAB is part of the FRA532 Mobile Robot course for third-year students at the Institute of Field Robotics (FIBO), King Mongkut’s University of Technology Thonburi (KMUTT).

## Table of Contents

- [Diagram]()
- [Forward Kinematics]()
- [Inverse Kinematics]()
- [Wheel Odometry]()
- [Installation]()
- [Usage]()
- [Experiment]()
- [Our Team]()

## Diagram

## Forward Kinematics


```

        [ v_robot ]  = r * [ 1/2  1/2 ][ w_r_wheel ]
        [ w_robot ]        [ 1/B -1/B ][ w_l_wheel ]


```


## Inverse Kinematics


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


## Wheel Odometry

```

        [ w_r_wheel ]  = [ 1/r  B/2r ][ v_robot ]
        [ w_l_wheel ]    [ 1/r -B/2r ][ w_robot ]


```
## Installation

Step 1: Clone the repository to the src directory of your workspace. You must unzip and put each folder in the src directory.

```

```
## Usage

```

```

## Experiment

### Launch Environment in Map

```
ros2 launch multi_turtlebot_sim spawn_environment.launch.py
```
### Launch Test Case

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
