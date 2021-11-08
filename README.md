# Template ROS Package

This repository is a template ROS Package which can be quickly cloned and used for **ROS Melodic** on **Ubuntu 10.04**. 

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

Please keep in mind that this code has been created for **ROS Melodic** on **Ubuntu 10.04**.

##Week 10 Updates

For this week rosservice  and a launch file has been added. 

## Using the launch file
```
cd <path to catkin workspace>
catkin_make
roscore
roslaunch beginner_tutorials pubsub.launch
```

## Using the rosservice
This repository has two services. The first one is add_two_ints server and client which adds two integers. The other one is a service in talker which adds concatenates two strings.
```
# Assuming you have sourced the setup file and have roscore running in another terminal
Add two ints service : rosrun beginner_tutorials add_two_ints_server
Add two ints client : rosrun beginner_tutorials add_two_ints_client 5 9

Concatenate stings : rosservice call add_two_strings "just " "beat it"
