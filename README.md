# Robot Teresa 
* [Description](#description)
* [Introduction](#introduction)
* [System Requirements](#system-requirements)
* [Installation](#installation)
* [Execute Simulation](#execute-simulation)
* [Execute with the real drone](#execute-with-the-real-drone)

# Description

Implementation of a two neural networks on the Teresa robot with Gazebo simulator:
- a Dense Neural Network
- Dueling Deep Q learning algorithm with 

The aim is that the robot identify and try to center a the detected person.

# Introduction

The simulation was used to train and test the neural network. The architecture of the simulation is described below:

![software architecture](./img/project_architecture.png)

The same commands are used to train the robot on the simulator and to control the robot in real life.

# System Requirements

**Operational system**:

    Ubuntu 18.04
 
**Python Version**:

    Python 3.6.9

**Jupyter Notebook**:

    latest version
  

**create an environment of work**:

# Installation
You have to install all the dependencies manually, this includes:

1. Install [ROS melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) (Install the Desktop-Full version)

2. Install [ROS bridge package](http://wiki.ros.org/rosbridge_suite)

    `$ sudo apt-get install ros-melodic-rosbridge-server`
4. Gazebo 9

3. Install Gazebo-ROS package

    `$ sudo apt-get install ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control`


# Execute the simulation

To run the simulation it will be necessary to use 3 terminals.

## 1. First Terminal

This terminal is responsable to launch roscore

`roscore`

## 2. Second Terminal
Launch simulation

`rosrun gazebo_ros gazebo ./teresa-simulation/gazebo_envs/Teresa_Lightweight.world`

## 3. Third Terminal

Launch ROS bridge

`roslaunch rosbridge_server rosbridge_websocket.launch`

(if you encounter an error about not installed rospk, just install it via : 

`pip3 install rospkg`


## 6. Sixth Terminal

Launch the jupyter notebook

`jupyter notebook`

or 

`anaconda-navigator`

chose the environment venv that you've created and launch the jupyternotebook from here


# Execute with the real robot

To run with the real drone it is almost the same thing.It will be necessary to use ******** terminals.

## 1. First Terminal

On your PC : This terminal is responsable to launch roscore

`roscore`


## 2. Third Terminal

On the robot PC : Launch ROS bridge

`roslaunch rosbridge_server rosbridge_websocket.launch`

## 2. add indications to follow and observation to command manually
