/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 * $Id: test_bagread_publish.cpp 33326 2010-10-15 06:10:24Z rusu $
 *
 */

/**
\author Radu Bogdan Rusu
**/

#include <ros/ros.h>
#include "pcl_ros/io/bag_io.h"

int
  main (int argc, char** argv)
{
  ros::init (argc, argv, "test_bagread_publish");
  ros::NodeHandle n;

  if (argc != 3)
  {
    ROS_ERROR ("USAGE: %s <file_name> <frequency in seconds>", argv[0]);
    return (-1);
  }

  std::string file_name = argv[1];
  double frequency = atof (argv[2]);
  ROS_INFO ("Publishing messages with a frequency of %f Hertz.", frequency);
  ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("pcl_ros/test_bagreader", 1);

  pcl_ros::BAGReader reader;
  if (!reader.open (file_name, "/narrow_stereo_textured/points2"))
  {
    ROS_ERROR ("Couldn't open %s for reading!", file_name.c_str ());
    return (-1);
  }

  // Wait until a client subscribes
  while (pub.getNumSubscribers () < 1) { ros::Duration (0.1).sleep (); }
  
  ros::Duration d (frequency);

  // Send 4 packages
  int i = 0;
  while (i < 4)
  {
    sensor_msgs::PointCloud2ConstPtr cloud = reader.getNextCloud ();
    ROS_INFO ("Publishing %zu data...", cloud->data.size ());
    pub.publish (cloud);
    ++i;
    d.sleep ();
  }

  return (0);
}
