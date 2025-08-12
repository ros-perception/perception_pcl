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
 * $Id: voxel_grid.cpp 35876 2011-02-09 01:04:36Z rusu $
 *
 */

#include <rclcpp_components/register_node_macro.hpp>
#include "pcl_ros/filters/voxel_grid.hpp"

//////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl_ros::VoxelGrid::child_init(bool & has_service)
{
  has_service = false;

  // Declare parameters with default values
  leaf_size_ = this->declare_parameter("leaf_size", 0.01);
  filter_limit_min_ = this->declare_parameter("filter_limit_min", -std::numeric_limits<double>::max());
  filter_limit_max_ = this->declare_parameter("filter_limit_max", std::numeric_limits<double>::max());
  filter_limit_negative_ = this->declare_parameter("filter_limit_negative", false);
  filter_field_name_ = this->declare_parameter("filter_field_name", std::string(""));

  // Set the initial values
  impl_.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
  impl_.setFilterLimits(filter_limit_min_, filter_limit_max_);
  impl_.setFilterLimitsNegative(filter_limit_negative_);
  if (!filter_field_name_.empty()) {
    impl_.setFilterFieldName(filter_field_name_);
  }

  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::VoxelGrid::filter(
  const PointCloud2ConstPtr & input,
  const IndicesPtr & indices,
  PointCloud2 & output)
{
  std::lock_guard<std::mutex> lock(mutex_);
  
  pcl::PCLPointCloud2::Ptr pcl_input(new pcl::PCLPointCloud2);
  pcl_conversions::toPCL(*input, *pcl_input);
  
  impl_.setInputCloud(pcl_input);
  if (indices) {
    impl_.setIndices(indices);
  }
  
  pcl::PCLPointCloud2 pcl_output;
  impl_.filter(pcl_output);
  
  pcl_conversions::moveFromPCL(pcl_output, output);
}

//////////////////////////////////////////////////////////////////////////////////////////////
rcl_interfaces::msg::SetParametersResult
pcl_ros::VoxelGrid::config_callback(const std::vector<rclcpp::Parameter> & parameters)
{
  // Call parent callback first
  auto result = Filter::config_callback(parameters);
  if (!result.successful) {
    return result;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  for (const auto & parameter : parameters) {
    if (parameter.get_name() == "leaf_size") {
      leaf_size_ = parameter.as_double();
      impl_.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
      RCLCPP_DEBUG(
        this->get_logger(),
        "[%s::config_callback] Setting the downsampling leaf size to: %f.",
        this->get_name(), leaf_size_);
    } else if (parameter.get_name() == "filter_limit_min") {
      filter_limit_min_ = parameter.as_double();
      impl_.setFilterLimits(filter_limit_min_, filter_limit_max_);
      RCLCPP_DEBUG(
        this->get_logger(),
        "[%s::config_callback] Setting the minimum filtering value a point will be considered from to: %f.",
        this->get_name(), filter_limit_min_);
    } else if (parameter.get_name() == "filter_limit_max") {
      filter_limit_max_ = parameter.as_double();
      impl_.setFilterLimits(filter_limit_min_, filter_limit_max_);
      RCLCPP_DEBUG(
        this->get_logger(),
        "[%s::config_callback] Setting the maximum filtering value a point will be considered from to: %f.",
        this->get_name(), filter_limit_max_);
    } else if (parameter.get_name() == "filter_limit_negative") {
      filter_limit_negative_ = parameter.as_bool();
      impl_.setFilterLimitsNegative(filter_limit_negative_);
      RCLCPP_DEBUG(
        this->get_logger(),
        "[%s::config_callback] Setting the filter negative flag to: %s.",
        this->get_name(), filter_limit_negative_ ? "true" : "false");
    } else if (parameter.get_name() == "filter_field_name") {
      filter_field_name_ = parameter.as_string();
      if (!filter_field_name_.empty()) {
        impl_.setFilterFieldName(filter_field_name_);
      }
      RCLCPP_DEBUG(
        this->get_logger(),
        "[%s::config_callback] Setting the filter field name to: %s.",
        this->get_name(), filter_field_name_.c_str());
    }
  }

  return result;
}

RCLCPP_COMPONENTS_REGISTER_NODE(pcl_ros::VoxelGrid)