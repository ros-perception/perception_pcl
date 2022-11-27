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
 * $Id: passthrough.cpp 36194 2011-02-23 07:49:21Z rusu $
 *
 */

#include "pcl_ros/filters/passthrough.hpp"

pcl_ros::PassThrough::PassThrough(const rclcpp::NodeOptions & options)
: Filter("PassThroughNode", options)
{
  std::vector<std::string> param_names = add_common_parameters();

  callback_handle_ =
    add_on_set_parameters_callback(
    std::bind(
      &PassThrough::config_callback, this,
      std::placeholders::_1));

  auto result = config_callback(get_parameters(param_names));
  if (!result.successful) {
    throw std::runtime_error(result.reason);
  }
  constructor_post_process();
}

//////////////////////////////////////////////////////////////////////////////////////////////
rcl_interfaces::msg::SetParametersResult
pcl_ros::PassThrough::config_callback(const std::vector<rclcpp::Parameter> & params)
{
  std::lock_guard<std::mutex> lock(mutex_);

  double filter_min, filter_max;
  impl_.getFilterLimits(filter_min, filter_max);

  for (const rclcpp::Parameter & param : params) {
    if (param.get_name() == "filter_field_name") {
      // Check the current value for the filter field
      if (impl_.getFilterFieldName() != param.as_string()) {
        // Set the filter field if different
        impl_.setFilterFieldName(param.as_string());
        RCLCPP_DEBUG(
          get_logger(), "Setting the filter field name to: %s.",
          param.as_string().c_str());
      }
    }
    if (param.get_name() == "filter_limit_min") {
      // Check the current values for filter min-max
      if (filter_min != param.as_double()) {
        filter_min = param.as_double();
        RCLCPP_DEBUG(
          get_logger(),
          "Setting the minimum filtering value a point will be considered from to: %f.",
          filter_min);
        // Set the filter min-max if different
        impl_.setFilterLimits(filter_min, filter_max);
      }
    }
    if (param.get_name() == "filter_limit_max") {
      // Check the current values for filter min-max
      if (filter_max != param.as_double()) {
        filter_max = param.as_double();
        RCLCPP_DEBUG(
          get_logger(),
          "Setting the maximum filtering value a point will be considered from to: %f.",
          filter_max);
        // Set the filter min-max if different
        impl_.setFilterLimits(filter_min, filter_max);
      }
    }
    if (param.get_name() == "filter_limit_negative") {
      // Check the current value for the negative flag
      if (impl_.getNegative() != param.as_bool()) {
        RCLCPP_DEBUG(
          get_logger(), "Setting the filter negative flag to: %s.",
          param.as_bool() ? "true" : "false");
        // Call the virtual method in the child
        impl_.setNegative(param.as_bool());
      }
    }
    if (param.get_name() == "keep_organized") {
      // Check the current value for keep_organized
      if (impl_.getKeepOrganized() != param.as_bool()) {
        RCLCPP_DEBUG(
          get_logger(), "Setting the filter keep_organized value to: %s.",
          param.as_bool() ? "true" : "false");
        // Call the virtual method in the child
        impl_.setKeepOrganized(param.as_bool());
      }
    }

    // The following parameters are updated automatically for all PCL_ROS Nodelet Filters
    // as they are inexistent in PCL
    if (param.get_name() == "input_frame") {
      if (tf_input_frame_ != param.as_string()) {
        tf_input_frame_ = param.as_string();
        RCLCPP_DEBUG(get_logger(), "Setting the input TF frame to: %s.", tf_input_frame_.c_str());
      }
    }
    if (param.get_name() == "output_frame") {
      if (tf_output_frame_ != param.as_string()) {
        tf_output_frame_ = param.as_string();
        RCLCPP_DEBUG(get_logger(), "Setting the output TF frame to: %s.", tf_output_frame_.c_str());
      }
    }
  }
  // TODO(sloretz) constraint validation
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_ros::PassThrough)
