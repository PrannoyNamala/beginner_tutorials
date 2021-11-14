/**
 * @file add_two_ints_server.cpp
 * @author Prannoy Namala (pnamala@umd.edu)
 * @brief A ROS Server Client based on Wiki Tutorials
 * @version 0.1
 * @date 11-07-2021
 *
 * Copyright (c) 2021
 *
 * Licensed under the MIT License (the "License")
 *
 */

// cppcheck-suppress missingInclude
#include "ros/ros.h"
// cppcheck-suppress missingInclude
#include "beginner_tutorials/AddTwoInts.h"

/**
 * @brief A ROS Service to add two integers
 */
bool add(beginner_tutorials::AddTwoInts::Request  &req,
  beginner_tutorials::AddTwoInts::Response &res) {
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (int)req.a, (int)req.b);
  ROS_INFO("sending back response: [%ld]", (int)res.sum);
  return true;
}

int main(int argc, char **argv) {
  /**
   * @detail The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("add_two_ints", add);
  ROS_INFO_STREAM("Ready to add two ints.");
  ros::spin();

  return 0;
}
