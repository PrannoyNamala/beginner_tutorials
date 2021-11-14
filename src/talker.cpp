/**
 * @file talker.cpp
 * @author Prannoy Namala (pnamala@umd.edu)
 * @brief A ROS Publisher based on Wiki Tutorials
 * @version 0.1
 * @date 10-31-2021
 *
 * Copyright (c) 2021
 *
 * Licensed under the MIT License (the "License")
 *
 */

// cppcheck-suppress missingInclude
#include <sstream>
// cppcheck-suppress missingInclude
#include "ros/ros.h"
// cppcheck-suppress missingInclude
#include "std_msgs/String.h"
// cppcheck-suppress missingInclude
#include "beginner_tutorials/ConcatStrings.h"
// cppcheck-suppress missingInclude
#include <tf/transform_broadcaster.h>


/**
 * 
 * @brief A rosservice that takes two strings and contatenates them
 */
bool concatString(beginner_tutorials::ConcatStrings::Request  &req,
               beginner_tutorials::ConcatStrings::Response &res) {
  if (req.aStr.empty() || req.bStr.empty()) {
    ROS_ERROR_STREAM("Missing input strings");
    return false;
  }
  ROS_WARN_STREAM("Now running string add service");
  res.opStr = req.aStr + req.bStr;
  ROS_INFO_STREAM("Input Strings are " + req.aStr + " and " + req.bStr);
  ROS_INFO_STREAM("Stream Output will be "+res.opStr);
  return true;
}

/**
 * @brief This tutorial demonstrates simple sending of messages over the ROS system.
 */
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
  ros::init(argc, argv, "talker");

  /**
   * @detail NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  tf::TransformBroadcaster br;  // frame transform broadcaster
  tf::Transform transform;  // frame transform handler

  /**
   * @detail Inititializing the ROS service creted earlier
   */

  ros::ServiceServer service = n.advertiseService("add_two_strings",
    concatString);

  /**
   * @detail The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  int rate = std::atoi(argv[1]);;
  
  if (rate <= 0) {
    ROS_FATAL_STREAM("No publisher rate given. Setting default");
    rate = 10;
  } else {
    ROS_DEBUG_STREAM("Rate set to given value");
  }

  ros::Rate loop_rate(rate);

  /**
   * @detail A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok()) {
    /**
     * @detail This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << "Go Terps!! PN" << count;
    msg.data = ss.str();

    ROS_INFO_STREAM(msg.data.c_str());

    /**
     * @detail The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    transform.setOrigin(tf::Vector3(1.0, 5.0, 7.0));
    transform.setRotation(tf::Quaternion(0.3, 0.9, 0.11, 1));
    br.sendTransform(tf::StampedTransform(transform,
    ros::Time::now(), "world", "talk"));

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
