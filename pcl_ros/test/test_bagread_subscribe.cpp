/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: test_bagread_subscribe.cpp 33326 2010-10-15 06:10:24Z rusu $
 *
 */

/**
\author Radu Bogdan Rusu

**/
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>

TEST (PCL_ROS, BAGReader)
{
  pcl::PointCloud<pcl::PointXYZRGB> points;

  int i = 0;
  while (i < 4)
  {
    // Get the message
    sensor_msgs::PointCloud2ConstPtr cloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2> ("pcl_ros/test_bagreader", ros::Duration (5.0));
    ASSERT_TRUE (cloud);

    // Test
    EXPECT_EQ ((int)cloud->width, 640);
    EXPECT_EQ ((int)cloud->height, 480);
    EXPECT_EQ ((bool)cloud->is_dense, true);
    EXPECT_EQ ((size_t)cloud->data.size () * 2, cloud->width * cloud->height * sizeof (pcl::PointXYZRGB));

    // Convert to a PointCloud<T> type
    pcl::fromROSMsg (*cloud, points);
    ++i;
  }

  EXPECT_NEAR ((float)points.points[12345].x, -0.157809, 1e-5);
  EXPECT_NEAR ((float)points.points[12345].y, -0.239234, 1e-5);
  EXPECT_NEAR ((float)points.points[12345].z, 1.1289,   1e-5);
}

int
  main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  ros::init (argc, argv, "test_bagread_subscribe");

  return (RUN_ALL_TESTS ());
}
