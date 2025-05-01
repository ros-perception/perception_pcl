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
 * $Id: pointcloud_to_pcd.cpp 33238 2010-03-11 00:46:58Z rusu $
 *
 */

/**
\author Radu Bogdan Rusu

@b pointcloud_to_pcd is a simple node that retrieves a ROS point cloud message and saves it to disk into a PCD (Point
Cloud Data) file format.

**/

#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace pcl_ros
{

class PointCloudToPCD : public rclcpp::Node
{
private:
  std::string prefix_;
  bool binary_;
  bool compressed_;
  std::string fixed_frame_;
  bool use_transform_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

public:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;

  void cloud_cb(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
  {
    if (cloud_msg->data.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Received empty point cloud message!");
      return;
    }

    sensor_msgs::msg::PointCloud2 transformed_cloud;
    if (!fixed_frame_.empty()) {
      use_transform_ = pcl_ros::transformPointCloud(
        fixed_frame_, *cloud_msg, transformed_cloud,
        tf_buffer_);
    } else {
      use_transform_ = false;
    }

    std::stringstream ss;
    ss << prefix_ << cloud_msg->header.stamp.sec << "."
       << std::setw(9) << std::setfill('0') << cloud_msg->header.stamp.nanosec
       << ".pcd";
    RCLCPP_INFO(this->get_logger(), "Writing to %s", ss.str().c_str());

    pcl::PCLPointCloud2 pcl_pc2;
    if (use_transform_) {
      pcl_conversions::toPCL(transformed_cloud, pcl_pc2);
    } else {
      pcl_conversions::toPCL(*cloud_msg, pcl_pc2);
    }

    writePCDFile(ss.str(), pcl_pc2);
  }

  void writePCDFile(const std::string & filename, const pcl::PCLPointCloud2 & cloud)
  {
    pcl::PCDWriter writer;
    if (binary_) {
      if (compressed_) {
        writer.writeBinaryCompressed(filename, cloud);
      } else {
        writer.writeBinary(filename, cloud);
      }
    } else {
      // Default precision is 8
      writer.writeASCII(filename, cloud);
    }
  }

  ////////////////////////////////////////////////////////////////////////////////
  explicit PointCloudToPCD(const rclcpp::NodeOptions & options)
  : rclcpp::Node("pointcloud_to_pcd", options),
    binary_(false), compressed_(false),
    tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    this->declare_parameter("prefix", prefix_);
    this->declare_parameter("fixed_frame", fixed_frame_);
    this->declare_parameter("binary", binary_);
    this->declare_parameter("compressed", compressed_);

    this->get_parameter("prefix", prefix_);
    this->get_parameter("fixed_frame", fixed_frame_);
    this->get_parameter("binary", binary_);
    this->get_parameter("compressed", compressed_);

    auto sensor_qos = rclcpp::SensorDataQoS();
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "input", sensor_qos,
      std::bind(&PointCloudToPCD::cloud_cb, this, std::placeholders::_1));
  }
};
}  // namespace pcl_ros

RCLCPP_COMPONENTS_REGISTER_NODE(pcl_ros::PointCloudToPCD)
