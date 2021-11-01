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
