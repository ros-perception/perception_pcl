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
 * $Id: test_io.cpp 28959 2010-04-22 00:17:47Z rusu $
 *
 */
/** \author Radu Bogdan Rusu */

#include <gtest/gtest.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/io/bag_io.h>
#include <pcl/ros/conversions.h>

using namespace pcl;
using namespace pcl_ros;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL_ROS, BAGReader)
{
  sensor_msgs::PointCloud2ConstPtr cloud_blob, cloud_blob_prev;
  PointCloud<PointXYZRGB> cloud;

  BAGReader reader;
  bool res = reader.open ("test_io_bag.bag", "/narrow_stereo_textured/points2");
  EXPECT_EQ ((bool)res, true);
  int cnt = 0;
  do
  {
    cloud_blob_prev = cloud_blob;
    cloud_blob = reader.getNextCloud ();
    if (cloud_blob_prev != cloud_blob)
    {
      EXPECT_EQ ((int)cloud_blob->width, 640);
      EXPECT_EQ ((int)cloud_blob->height, 480);
      EXPECT_EQ ((bool)cloud_blob->is_dense, true);
      EXPECT_EQ ((size_t)cloud_blob->data.size () * 2,    // PointXYZRGB is 16*2 (XYZ+1, RGB+3)
                 cloud_blob->width * cloud_blob->height * sizeof (PointXYZRGB));
      cnt++;
    }
  }
  while (cloud_blob != cloud_blob_prev);

  EXPECT_EQ (cnt, 4);

  // Convert from blob to data type
  pcl::fromROSMsg (*cloud_blob, cloud);

  EXPECT_NEAR ((float)cloud.points[12345].x, -0.157809, 1e-5);
  EXPECT_NEAR ((float)cloud.points[12345].y, -0.239234, 1e-5);
  EXPECT_NEAR ((float)cloud.points[12345].z, 1.1289,   1e-5);
}

/* ---[ */
int
  main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
