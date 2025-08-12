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
 * $Id: extract_indices.cpp 35876 2011-02-09 01:04:36Z rusu $
 *
 */

#include <rclcpp_components/register_node_macro.hpp>
#include "pcl_ros/filters/extract_indices.hpp"

//////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl_ros::ExtractIndices::child_init(bool & has_service)
{
  has_service = false;

  // Declare parameter
  negative_ = this->declare_parameter("negative", false);
  
  // Set the initial value
  impl_.setNegative(negative_);
  
  use_indices_ = true;
  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::ExtractIndices::filter(
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
pcl_ros::ExtractIndices::config_callback(const std::vector<rclcpp::Parameter> & parameters)
{
  // Call parent callback first
  auto result = Filter::config_callback(parameters);
  if (!result.successful) {
    return result;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  for (const auto & parameter : parameters) {
    if (parameter.get_name() == "negative") {
      negative_ = parameter.as_bool();
      impl_.setNegative(negative_);
      RCLCPP_DEBUG(
        this->get_logger(),
        "[%s::config_callback] Setting the extraction to: %s.", 
        this->get_name(),
        (negative_ ? "everything but the indices" : "indices"));
    }
  }

  return result;
}

RCLCPP_COMPONENTS_REGISTER_NODE(pcl_ros::ExtractIndices)