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

#include "pcl_ros/filters/passthrough.h"

//////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl_ros::PassThrough::child_init (rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_param, bool &has_service)
{
  // TODO: remove?
  // Enable the dynamic reconfigure service
  has_service = true;

  rcl_interfaces::msg::ParameterDescriptor ffn_desc;
  ffn_desc.name = "filter_field_name";
  ffn_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  ffn_desc.description = "The field name used for filtering";
  node_param->declare_parameter (ffn_desc.name, rclcpp::ParameterValue("z"), ffn_desc);

  rcl_interfaces::msg::ParameterDescriptor flmin_desc;
  flmin_desc.name = "filter_limit_min";
  flmin_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  flmin_desc.description = "The minimum allowed field value a point will be considered from";
  rcl_interfaces::msg::FloatingPointRange flmin_range;
  flmin_range.from_value = -100000.0;
  flmin_range.to_value = 100000.0;
  flmin_desc.floating_point_range.push_back (flmin_range);
  node_param->declare_parameter (flmin_desc.name, rclcpp::ParameterValue(0.0), flmin_desc);

  rcl_interfaces::msg::ParameterDescriptor flmax_desc;
  flmax_desc.name = "filter_limit_max";
  flmax_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  flmax_desc.description = "The maximum allowed field value a point will be considered from";
  rcl_interfaces::msg::FloatingPointRange flmax_range;
  flmax_range.from_value = -100000.0;
  flmax_range.to_value = 100000.0;
  flmax_desc.floating_point_range.push_back (flmax_range);
  node_param->declare_parameter (flmax_desc.name, rclcpp::ParameterValue(1.0), flmax_desc);

  rcl_interfaces::msg::ParameterDescriptor flneg_desc;
  flneg_desc.name = "filter_limit_negative";
  flneg_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  flneg_desc.description = "Set to true if we want to return the data outside [filter_limit_min; filter_limit_max].";
  node_param->declare_parameter (flneg_desc.name, rclcpp::ParameterValue(false), flneg_desc);

  rcl_interfaces::msg::ParameterDescriptor keep_organized_desc;
  keep_organized_desc.name = "keep_organized";
  keep_organized_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  keep_organized_desc.description = "Set whether the filtered points should be kept and set to NaN, or removed from the PointCloud, thus potentially breaking its organized structure.";
  node_param->declare_parameter (keep_organized_desc.name, rclcpp::ParameterValue(false), keep_organized_desc);

  rcl_interfaces::msg::ParameterDescriptor input_frame_desc;
  input_frame_desc.name = "input_frame";
  input_frame_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  input_frame_desc.description = "The input TF frame the data should be transformed into before processing, if input.header.frame_id is different.";
  node_param->declare_parameter (input_frame_desc.name, rclcpp::ParameterValue(""), input_frame_desc);

  rcl_interfaces::msg::ParameterDescriptor output_frame_desc;
  output_frame_desc.name = "output_frame";
  output_frame_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  output_frame_desc.description = "The output TF frame the data should be transformed into after processing, if input.header.frame_id is different.";
  node_param->declare_parameter (output_frame_desc.name, rclcpp::ParameterValue(""), output_frame_desc);

  // TODO
  node_param->set_on_parameters_set_callback (boost::bind (&PassThrough::config_callback, this, _1));

  //srv_ = boost::make_shared <dynamic_reconfigure::Server<pcl_ros::FilterConfig> > (nh);
  //dynamic_reconfigure::Server<pcl_ros::FilterConfig>::CallbackType f = boost::bind (&PassThrough::config_callback, this, _1, _2);
  //srv_->setCallback (f);

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
rcl_interfaces::msg::SetParametersResult
pcl_ros::PassThrough::config_callback (const std::vector<rclcpp::Parameter> & params)
{
  std::lock_guard<std::mutex> lock (mutex_);

  double filter_min, filter_max;
  impl_.getFilterLimits (filter_min, filter_max);

  for (const rclcpp::Parameter &param : params)
  {
    if (param.get_name () == "filter_field_name")
    {
      // Check the current value for the filter field
      //std::string filter_field = impl_.getFilterFieldName ();
      if (impl_.getFilterFieldName () != param.as_string ())
      {
        // Set the filter field if different
        impl_.setFilterFieldName (param.as_string ());
        // TODO(sloretz) add "config_callback" specific logger
        RCLCPP_DEBUG (get_logger(), "Setting the filter field name to: %s.", param.as_string ().c_str ());
      }
    }
    if (param.get_name () == "filter_limit_min")
    {
      // Check the current values for filter min-max
      if (filter_min != param.as_double ())
      {
        filter_min = param.as_double ();
        RCLCPP_DEBUG (get_logger(), "Setting the minimum filtering value a point will be considered from to: %f.", filter_min);
        // Set the filter min-max if different
        impl_.setFilterLimits (filter_min, filter_max);
      }
    }
    if (param.get_name () == "filter_limit_max")
    {
      // Check the current values for filter min-max
      if (filter_max != param.as_double ())
      {
        filter_max = param.as_double ();
        RCLCPP_DEBUG (get_logger(), "Setting the maximum filtering value a point will be considered from to: %f.", filter_max);
        // Set the filter min-max if different
        impl_.setFilterLimits (filter_min, filter_max);
      }
    }
    if (param.get_name () == "filter_limit_negative")
    {
      // Check the current value for the negative flag
      if (impl_.getFilterLimitsNegative () != param.as_bool ())
      {
        RCLCPP_DEBUG (get_logger(), "Setting the filter negative flag to: %s.", param.as_bool () ? "true" : "false");
        // Call the virtual method in the child
        impl_.setFilterLimitsNegative (param.as_bool ());
      }
    }
    if (param.get_name () == "keep_organized")
    {
      // Check the current value for keep_organized
      if (impl_.getKeepOrganized () != param.as_bool ())
      {
        RCLCPP_DEBUG (get_logger(), "Setting the filter keep_organized value to: %s.", param.as_bool () ? "true" : "false");
        // Call the virtual method in the child
        impl_.setKeepOrganized (param.as_bool ());
      }
    }

    // The following parameters are updated automatically for all PCL_ROS Nodelet Filters as they are inexistent in PCL
    if (param.get_name () == "input_frame")
    {
      if (tf_input_frame_ != param.as_string ())
      {
        tf_input_frame_ = param.as_string ();
        RCLCPP_DEBUG (get_logger(), "Setting the input TF frame to: %s.", tf_input_frame_.c_str ());
      }
    }
    if (param.get_name () == "output_frame")
    {
      if (tf_output_frame_ != param.as_string ())
      {
        tf_output_frame_ = param.as_string ();
        RCLCPP_DEBUG (get_logger(), "Setting the output TF frame to: %s.", tf_output_frame_.c_str ());
      }
    }
  }
  // TODO(sloretz) parameter validation
  return rcl_interfaces::msg::SetParametersResult();
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_ros::PassThrough)
