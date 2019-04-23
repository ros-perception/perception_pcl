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
 * $Id: surface_convex_hull.cpp 34612 2010-12-08 01:06:27Z rusu $
 *
 */

/**
 \author Ethan Rublee

 @b convert a pcd to an image file
 run with:
   rosrun pcl convert_pcd_image cloud_00042.pcd
 It will publish a ros image message on /pcd/image
 View the image with:
    rosrun image_view image_view image:=/pcd/image
 **/

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

/* ---[ */
int
main (int argc, char **argv)
{
  rclcpp::init (argc, argv);
  auto node = std::make_shared<rclcpp::Node>("image_publisher");
  auto image_pub = node->create_publisher <sensor_msgs::msg::Image> ("output", 1);
  
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  if (argc != 2)
  {
    std::cout << "usage:\n" << argv[0] << " cloud.pcd" << std::endl;
    return 1;
  }
  
  sensor_msgs::msg::Image image;
  sensor_msgs::msg::PointCloud2 cloud;
  pcl::io::loadPCDFile (std::string (argv[1]), cloud);

  try
  {
    pcl::toROSMsg (cloud, image); //convert the cloud
  }
  catch (std::runtime_error &e)
  {
    RCLCPP_ERROR(node->get_logger(), "Error in converting cloud to image message: %s" ,e.what());
    return 1; //fail!
  }
  rclcpp::Rate loop_rate (5);
  while (rclcpp::ok ())
  {
    image_pub->publish (image);
    exec.spin_once();
    loop_rate.sleep ();
  }
  
  rclcpp::shutdown();
  return (0);
}

/* ]--- */
