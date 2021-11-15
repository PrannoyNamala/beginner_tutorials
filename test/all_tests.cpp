/**
 * @file all_tests.cpp
 * @author Prannoy Namala (pnamala@umd.edu)
 * @brief Sample testing file for ROS
 * @version 0.1
 * @date 11-07-2021
 *
 * Copyright (c) 2021
 *
 * Licensed under the MIT License (the "License")
 *
 */

// cppcheck-suppress missingInclude
#include <ros/ros.h>
// cppcheck-suppress missingInclude
#include <ros/service_client.h>
// cppcheck-suppress missingInclude
#include <gtest/gtest.h>
// cppcheck-suppress missingInclude
#include <beginner_tutorials/AddTwoInts.h>
// cppcheck-suppress missingInclude
#include "std_msgs/String.h"

void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("I heard from test: [%s]", msg->data.c_str());
}

/**
 * @brief This test checks the service of adding two ints.
 */
TEST(TESTSuite, addTwoInts)
{
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<beginner_tutorials::AddTwoInts>(
      "add_two_ints");
  bool exists(client.waitForExistence(ros::Duration(1)));
  EXPECT_TRUE(exists);

  beginner_tutorials::AddTwoInts srv;
  srv.request.a = 2;
  srv.request.b = 3;
  client.call(srv);

  EXPECT_EQ(srv.response.sum, srv.request.a + srv.request.b);
}

/**
 * @brief This test checks the existance of nodes talke and listener.
 */
TEST(TestSuite, talkerTest) {
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  EXPECT_EQ(pub.getNumSubscribers(), 1U);
  EXPECT_EQ(sub.getNumPublishers(), 1U);
}

/**
 * @brief This runs all the tests.
 */
int main(int argc,
         char **argv)
{
  ros::init(argc, argv, "all_tests");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

