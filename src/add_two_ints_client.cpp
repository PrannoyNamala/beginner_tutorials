/**
 * @file add_two_ints_server.cpp
 * @author Prannoy Namala (pnamala@umd.edu)
 * @brief A ROS Server based on Wiki Tutorials
 * @version 0.1
 * @date 11-07-2021
 *
 * Copyright (c) 2021
 *
 * Licensed under the MIT License (the "License")
 *
 */

// cppcheck-suppress missingInclude
#include <cstdlib>
// cppcheck-suppress missingInclude
#include "ros/ros.h"
// cppcheck-suppress missingInclude
#include "beginner_tutorials/AddTwoInts.h"

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
  ros::init(argc, argv, "add_two_ints_client");
  if (argc != 3) {
    ROS_WARN_STREAM("usage: add_two_ints_client X Y");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<beginner_tutorials::AddTwoInts>(
    "add_two_ints");
  beginner_tutorials::AddTwoInts srv;
  srv.request.a = atoll(argv[1]);
  srv.request.b = atoll(argv[2]);
  if (client.call(srv)) {
    ROS_INFO_STREAM("Sum: %ld", (int)srv.response.sum);
  } else {
    ROS_FATAL_STREAM("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}
