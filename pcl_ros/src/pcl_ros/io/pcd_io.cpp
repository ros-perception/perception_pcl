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
 * $Id: pcd_io.cpp 35812 2011-02-08 00:05:03Z rusu $
 *
 */

#include <pcl_ros/io/pcd_io.h>

#include <rclcpp/create_publisher.hpp>
#include <rclcpp/create_timer.hpp>

using sensor_msgs::msg::PointCloud2;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl_ros::PCDReader::PCDReader (const rclcpp::NodeOptions& options) : PCLNode("PCDReaderNode", options), publish_rate_ (0), tf_frame_ ("/base_link")
{
  // Provide a latched topic
  // TODO(sloretz) what is the right QoS?
  auto qos = rclcpp::QoS(max_queue_size_);
  publisher_ = rclcpp::create_publisher<PointCloud2>(*this, "output", qos);

  // TODO(sloretz) use publish_rate_
  publish_timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration(std::chrono::milliseconds(10)),
    std::bind(&PCDReader::on_publish_timer, this));

  get_node_parameters_interface()->set_on_parameters_set_callback(
    std::bind(&PCDReader::on_parameters_set, this, std::placeholders::_1));

  // TODO(sloretz) parameter descriptors
  // publish_rate_, tf_frame_, make those read only
  // file_name_ is changeable
  publish_rate_ = declare_parameter("publish_rate", rclcpp::ParameterValue(publish_rate_)).get<double>();
  tf_frame_ = declare_parameter("tf_frame", rclcpp::ParameterValue(tf_frame_)).get<std::string>();
  file_name_ = declare_parameter("fileframe", rclcpp::ParameterValue(file_name_)).get<std::string>();

  RCLCPP_DEBUG(this->get_logger(), "Node successfully created"
                 " - publish_rate : %f\n"
                 " - tf_frame     : %s",
                 publish_rate_, tf_frame_.c_str ());
}

rcl_interfaces::msg::SetParametersResult
pcl_ros::PCDReader::on_parameters_set(const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const rclcpp::Parameter &param : params)
  {
    if (param.get_name() == "filename")
    {
      auto new_file = param.as_string();
      if (new_file != file_name_) {
        // cloud to publish has changed
        file_name_ = new_file;
        output_.reset();

        RCLCPP_INFO (this->get_logger(), "New file given: %s", file_name_.c_str ());
        pcl::PCLPointCloud2 cloud;
        if (impl_.read (file_name_, cloud) < 0)
        {
          RCLCPP_ERROR (this->get_logger(), "Error reading %s !", file_name_.c_str ());
          result.successful = false;
          result.reason = "failed to read file";
          break;
        }
        pcl_conversions::moveFromPCL(cloud, *(output_));
      }
    }
  }
  return result;
}

void
pcl_ros::PCDReader::on_publish_timer()
{
  // If there are no subscribers, skip
  if (0u == count_subscribers(publisher_->get_topic_name())) {
    return;
  }

  // If there's no data, skip
  if (!output_ || output_->data.empty()) {
    return;
  }

  // Publish the data
  RCLCPP_DEBUG (get_logger(), "Publishing data (%d points) on topic %s in frame %s.", output_->width * output_->height, "output", output_->header.frame_id.c_str ());
  output_->header.stamp = now();
  output_->header.frame_id = tf_frame_;
  publisher_->publish(*output_);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl_ros::PCDWriter::PCDWriter(const rclcpp::NodeOptions& options) : PCLNode("PCDWriterNode", options), file_name_ (""), binary_mode_ (true)
{
  // Workaround ros2/rclcpp#766
  std::function<void(PointCloud2::ConstSharedPtr)> callback = std::bind(&PCDWriter::input_callback, this, std::placeholders::_1);
  // TODO(sloretz) what QoS settings are appropriate?
  auto qos = rclcpp::QoS(1);
  sub_input_ = this->create_subscription<PointCloud2>("input", qos, callback);
  // ---[ Optional parameters
  this->get_parameter ("filename", file_name_);
  this->get_parameter ("binary_mode", binary_mode_);

  RCLCPP_DEBUG (this->get_logger(), "Node successfully created with the following parameters:\n"
                 " - filename     : %s\n"
                 " - binary_mode  : %s",
                 file_name_.c_str (), (binary_mode_) ? "true" : "false");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::PCDWriter::input_callback (PointCloud2::ConstSharedPtr cloud)
{
  if (!isValid (cloud))
    return;
  
  this->get_parameter ("filename", file_name_);

  RCLCPP_DEBUG (this->get_logger(), "PointCloud with %d data points and frame %s on topic %s received.", cloud->width * cloud->height, cloud->header.frame_id.c_str (), "input");
 
  std::string fname;
  if (file_name_.empty ())
    fname = std::to_string (cloud->header.stamp.sec) + ".pcd";
  else
    fname = file_name_;
  pcl::PCLPointCloud2 pcl_cloud;
  // It is safe to remove the const here because we are the only subscriber callback.
  pcl_conversions::moveToPCL(*(const_cast<PointCloud2*>(cloud.get())), pcl_cloud);
  impl_.write (fname, pcl_cloud, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), binary_mode_);

  RCLCPP_DEBUG (this->get_logger(), "Data saved to %s", fname.c_str ());
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_ros::PCDWriter)
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_ros::PCDReader)

