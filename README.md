# Template ROS Package

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)


This repository is a template ROS Package which can be quickly cloned and used for **ROS Melodic** on **Ubuntu 18.04**. 

## Install via Command Line
```
cd <path to catkin workspace>
git clone https://github.com/PrannoyNamala/beginner_tutorials
catkin_make
roscore
Run publisher in a new terminal window: rosrun beginner_tutorials talker
Run subscriber in a new terminal window: rosrun beginner_tutorials listener
```
Tip - Run ```source devel/setup.bash``` when trying to run ROS commands in a new window

Please keep in mind that this code has been created for **ROS Melodic** on **Ubuntu 18.04**.

## Week 10 Updates

For this week rosservice  and a launch file has been added. 

## Using the launch file
```
cd <path to catkin workspace>
catkin_make
roscore
roslaunch beginner_tutorials pubsub.launch
# Use this command to change the rate
roslaunch beginner_tutorials pubsub.launch rate:=4
# Use this command to use the add ints service
roslaunch beginner_tutorials pubsub.launch a:=4 b:=5
```

## Using the rosservice
This repository has two services. The first one is add_two_ints server and client which adds two integers. The other one is a service in talker which adds concatenates two strings.
```
# Assuming you have sourced the setup file and have roscore running in another terminal
Add two ints service : rosrun beginner_tutorials add_two_ints_server
Add two ints client : rosrun beginner_tutorials add_two_ints_client 5 9

Concatenate stings : rosservice call add_two_strings "just " "beat it"
```

## Week 11 Uppdates

This week using tf broadcaster,unit testing and rosbag features hve been added. The tf broadcaster is integrated into the ```pubsub.launch```  file. 

## Unit Testing
```
# For running unit tests , you should be in your catkin_ws
catkin_make run_tests
```

## Recording using rosbags
For recording a bag file, in the ```pubsub.launch``` launch
```
roslaunch beginner_tutorials pubsub.launch record:=1
```
### Playing the saved rosbag with chater topic
The saved file is in the results directory. Need to be in that directory to be able to run the command.  
```
rosbag play record_beginner_tutorials.bag --topics /chatter
```
