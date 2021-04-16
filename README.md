# flocking ROS package

* This package achieves flocking behaviors of multi robots with differential drives both in Gazebo and Stage. There is a subsidiary package to load a custom robot description.
* Two launch files under each environment (Gazebo or Stage) will be initiated.
    * First terminal: loading the simulation environment
    * Second terminal: executing flocking behavior control node
* The program is robust to achieve the flocking behavior, i. e., boids consisting of of three parts: alignment, cohesion, and separation.
* The package has a main running node in 'nodes' folder and essential code implementations (__FlockingRobot.py__, __aux_function.py__) such as functions and class in 'src/flocking' folder to make it re-usable. 

<div style="text-align:center">
<img src="./images/code_design.png" width="400" height="250"> 
</div>

## Requirements
* Tested on ROS kinetic (not sure about Melodic but it should work except for the case where topic name is different on Stage in case.)
* Gazebo 7
* Stage

## Build and setup
* Clone the repositories (__flocking__ and __custom_turtlebot3_description__) into workspace ,e.g.,`ros_workspace/src/`
* run the command `catkin_make`
* source ros_workspace/devel/setup.sh

## Run
### Gazebo
* 1st terminal
    ```
    roslaunch flocking flocking_gazebo.launch
    ```
    * This command will open the gazebo environment with three robots.

* 2nd terminal
    ```
    roslaunch flocking_control_gazebo.launch
    ```
    * This command will run a node for each robot and commence the flocking behavior.

### Stage
* 1st terminal
    ```
    roslaunch flocking flocking_stage.launch
    ```
    * This command will open the stage environment with ten robots.

* 2nd terminal
    ```
    roslaunch flocking_control_stage.launch
    ```
    * This command will run a node for each robot and commence the flocking behavior.


## Attribution & Licensing

Materials substantially authored by Mingi Jeong. Copyright 2020 by Amazon.com, Inc. or its affiliates. Licensed MIT-0 - See LICENSE for further information
