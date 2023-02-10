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
 * $Id: radius_outlier_removal.cpp 33319 2010-10-15 04:49:28Z rusu $
 *
 */

#include "pcl_ros/filters/radius_outlier_removal.hpp"

pcl_ros::RadiusOutlierRemoval::RadiusOutlierRemoval(const rclcpp::NodeOptions & options)
: Filter("RadiusOutlierRemovalNode", options)
{
  use_frame_params();
  std::vector<std::string> param_names = add_common_params();

  callback_handle_ =
    add_on_set_parameters_callback(
    std::bind(
      &RadiusOutlierRemoval::config_callback, this,
      std::placeholders::_1));

  config_callback(get_parameters(param_names));
  // TODO(daisukes): lazy subscription after rclcpp#2060
  subscribe();
}

//////////////////////////////////////////////////////////////////////////////////////////////
rcl_interfaces::msg::SetParametersResult
pcl_ros::RadiusOutlierRemoval::config_callback(const std::vector<rclcpp::Parameter> & params)
{
  std::lock_guard<std::mutex> lock(mutex_);

  for (const rclcpp::Parameter & param : params) {
    if (param.get_name() == "min_neighbors") {
      if (impl_.getMinNeighborsInRadius() != param.as_int()) {
        RCLCPP_DEBUG(
          get_logger(), "Setting the number of neighbors in radius: %ld.",
          param.as_int());
        impl_.setMinNeighborsInRadius(param.as_int());
      }
    }
    if (param.get_name() == "radius_search") {
      if (impl_.getRadiusSearch() != param.as_double()) {
        RCLCPP_DEBUG(
          get_logger(), "Setting the radius to search neighbors: %f.",
          param.as_double());
        impl_.setRadiusSearch(param.as_double());
      }
    }
  }

  // TODO(sloretz) constraint validation
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_ros::RadiusOutlierRemoval)
