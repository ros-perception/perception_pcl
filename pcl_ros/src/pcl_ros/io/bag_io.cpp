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
 * $Id: bag_io.cpp 34896 2010-12-19 06:21:42Z rusu $
 *
 */

#include "class_loader/register_macro.hpp"
#include "pcl_ros/io/bag_io.h"

//////////////////////////////////////////////////////////////////////////////////////////////
/*bool
pcl_ros::BAGReader::open (const std::string &file_name, const std::string &topic_name)
{
  try
  {
    bag_.open (file_name, rosbag2::bagmode::Read);
    view_.addQuery (bag_, rosbag2::TopicQuery (topic_name));

    if (view_.size () == 0)
      return (false);

    it_ = view_.begin ();
  }
  catch (rosbag2::BagException &e)
  {
    return (false);
  }
  return (true);
}
*/
//////////////////////////////////////////////////////////////////////////////////////////////
/*pcl_ros::BAGReader ("bagreader_node") : publish_rate_ (0), output_ ()
{
  // ---[ Mandatory parameters
  if (!this->get_parameter ("file_name", file_name_))
  {
    RCLCPP_ERROR (this->get_logger(), "[onConstructor] Need a 'file_name' parameter to be set before continuing!");
    return;
  }
   if (!this->get_parameter ("topic_name", topic_name_))
  {
    RCLCPP_ERROR ("[onConstructor] Need a 'topic_name' parameter to be set before continuing!");
    return;
  }
  // ---[ Optional parameters
  int max_queue_size = 1;
  this->get_parameter ("publish_rate", publish_rate_);
  this->get_parameter ("max_queue_size", max_queue_size);

  auto pub_output = this->create_publisher<sensor_msgs::msg::PointCloud2> ("output", max_queue_size);

  RCLCPP_DEBUG (this->get_logger(), "[%s::onConstructor] Node successfully created with the following parameters:\n"
                 " - file_name    : %s\n"
                 " - topic_name   : %s",
                this->get_name(),
                file_name_.c_str (),
                topic_name_.c_str ());

  if (!open (file_name_, topic_name_))
    return;
  PointCloud output;
  output_ = std::make_shared<PointCloud> (output);
  output_->header.stamp    = this->now ();

  // Continous publishing enabled?
  while (this->ok ())
  {
    PointCloudConstPtr cloud = getNextCloud ();
    RCLCPP_DEBUG (this->get_logger(), "Publishing data (%d points) on topic %s in frame %s.", output_->width * output_->height, "output", output_->header.frame_id.c_str ());
    output_->header.stamp = this->now ();

    pub_output->publish (output_);

    rclcpp::Duration (publish_rate_).sleep ();
    rclcpp::spinOnce ();
  }
}
*/
//typedef pcl_ros::BAGReader BAGReader;
//CLASS_LOADER_REGISTER_CLASS(BAGReader, rclcpp::Node)


