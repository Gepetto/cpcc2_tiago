# Chained controller for tiago, based on Crocoddyl


This repo contain the code of a new ros2 controller : cpcc2 (Crocoddyl PVEG(Position Velocity Effort Gains) Chained Controller), added to it, its launch file and config for different actuators characteristics.

## Installation

The controllers are based on ros2_control and make use of launch-pal for the launch files:

```bash
    cd your_ws
    mkdir src
    cd src
    git clone repo/url
    cd ..
    rosdep install --from-paths src
```

## Package configuration

The configuration of this package is **mandatory** for its well functionning, the parameters are easily tunable in [cpcc2_tiago_parameters.yaml](config/cpcc2_tiago_parameters.yaml)

First of all the path should be changed to match your directories, you can also enable live and file logging.

By default the torso is disabled for performance evaluation, to enable it, add "torso_lift_joint" in front of "arm_1_joint" and "position" at the beginning of "pveg_joints_command_type"

## Building 

After cloning the repo in the src folder of the workspace and cd to the root of the workspace

```bash
colcon build --cmake-args ' -DCMAKE_BUILD_TYPE=RELEASE'
```

This should start building the cpcc2_tiago package, it take a few minutes.
After the build is done you can

```bash
source path workspace_ws/install/setup.bash
```

## Overview


The controller is comprised of 2 smaller ones: one for the high level path planning : Crocoddyl, and the other for the torque to apply calculation : Pveg Chained, those two can be loaded from a single file : [cpcc2_tiago.launch.py](launch/cpcc2_tiago.launch.py)


<img src="doc/media/cpcc2_tiago_full_chain.png" width=50% height=50%>


## Controllers

#### Pveg Chained Controller

The PvegChainedController is at the lowest level in the command chain : it is connected directly to the robot
hardware through an hardware interface and sends effort command to the motors. For communicating it might be
tempting to send it command through a topic or a service but that would be an unreliable solution for real time and
fast control of Tiago, we rather turn this controller into a chained one giving it new and fast communication tools :
reference interfaces

#### Crocoddyl Controller

The CrocoddylController is a used to retrieve the solver results, it computes the command value and send them to the PvegChainedController through the virtual hardware interface. From its side those interfaces have nothing special but their prefix name.


## How to launch the controllers ?

The controllers are launched with [cpcc2_tiago.launch.py](launch/cpcc2_tiago.launch.py), after sourcing the setup.bash 

```bash
ros2 launch cpcc2_tiago cpcc2_tiago.launch.py 

```


This scripts **loads** the controllers, hence the controller manager should have been started earlier. Furthermore the used hardware interfaces (effort for the arm, position for the torso) should not have been claimed. Finally the controllers should have their update frequency above 500Hz, most of the development was done at 1 KHz 


## Evaluate performance

Python scripts are given to evaluate the performance of Tiago, [performance_evaluator.py](scripts/performance_evaluator.py), it sends Tiago 8 targets being the 8 vertices of a cube and logs many datas (see docstring).

The performance plotter and full plotter nicely show the results of those study, the full plotter is used to compare performance between the 3 interpolation types.

All these scripts can be launched with ros2 run cpcc2_tiago scripts.py args, see the scripts' docstring for the required arguments
