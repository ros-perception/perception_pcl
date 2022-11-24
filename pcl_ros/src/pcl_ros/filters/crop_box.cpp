/*
 *
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
 * $Id: cropbox.cpp
 *
 */

#include "pcl_ros/filters/crop_box.hpp"

pcl_ros::CropBox::CropBox(const rclcpp::NodeOptions & options)
: Filter("CropBoxNode", options)
{
  rcl_interfaces::msg::ParameterDescriptor min_x_desc;
  min_x_desc.name = "min_x";
  min_x_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  min_x_desc.description = "X coordinate of the minimum point of the box.";
  rcl_interfaces::msg::FloatingPointRange min_x_range;
  min_x_range.from_value = -1000;
  min_x_range.to_value = 1000;
  min_x_desc.floating_point_range.push_back(min_x_range);
  declare_parameter(min_x_desc.name, rclcpp::ParameterValue(-1.0), min_x_desc);

  rcl_interfaces::msg::ParameterDescriptor max_x_desc;
  max_x_desc.name = "max_x";
  max_x_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  max_x_desc.description = "X coordinate of the maximum point of the box.";
  rcl_interfaces::msg::FloatingPointRange max_x_range;
  max_x_range.from_value = -1000;
  max_x_range.to_value = 1000;
  max_x_desc.floating_point_range.push_back(max_x_range);
  declare_parameter(max_x_desc.name, rclcpp::ParameterValue(1.0), max_x_desc);

  rcl_interfaces::msg::ParameterDescriptor min_y_desc;
  min_y_desc.name = "min_y";
  min_y_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  min_y_desc.description = "Y coordinate of the minimum point of the box.";
  rcl_interfaces::msg::FloatingPointRange min_y_range;
  min_y_range.from_value = -1000;
  min_y_range.to_value = 1000;
  min_y_desc.floating_point_range.push_back(min_y_range);
  declare_parameter(min_y_desc.name, rclcpp::ParameterValue(-1.0), min_y_desc);

  rcl_interfaces::msg::ParameterDescriptor max_y_desc;
  max_y_desc.name = "max_y";
  max_y_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  max_y_desc.description = "Y coordinate of the maximum point of the box.";
  rcl_interfaces::msg::FloatingPointRange max_y_range;
  max_y_range.from_value = -1000;
  max_y_range.to_value = 1000;
  max_y_desc.floating_point_range.push_back(max_y_range);
  declare_parameter(max_y_desc.name, rclcpp::ParameterValue(1.0), max_y_desc);

  rcl_interfaces::msg::ParameterDescriptor min_z_desc;
  min_z_desc.name = "min_z";
  min_z_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  min_z_desc.description = "Z coordinate of the minimum point of the box.";
  rcl_interfaces::msg::FloatingPointRange min_z_range;
  min_z_range.from_value = -1000;
  min_z_range.to_value = 1000;
  min_z_desc.floating_point_range.push_back(min_z_range);
  declare_parameter(min_z_desc.name, rclcpp::ParameterValue(-1.0), min_z_desc);

  rcl_interfaces::msg::ParameterDescriptor max_z_desc;
  max_z_desc.name = "max_z";
  max_z_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  max_z_desc.description = "Z coordinate of the maximum point of the box.";
  rcl_interfaces::msg::FloatingPointRange max_z_range;
  max_z_range.from_value = -1000;
  max_z_range.to_value = 1000;
  max_z_desc.floating_point_range.push_back(max_z_range);
  declare_parameter(max_z_desc.name, rclcpp::ParameterValue(1.0), max_z_desc);

  rcl_interfaces::msg::ParameterDescriptor keep_organized_desc;
  keep_organized_desc.name = "keep_organized";
  keep_organized_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  keep_organized_desc.description =
    "Set whether the filtered points should be kept and set to NaN, "
    "or removed from the PointCloud, thus potentially breaking its organized structure.";
  declare_parameter(keep_organized_desc.name, rclcpp::ParameterValue(false), keep_organized_desc);

  rcl_interfaces::msg::ParameterDescriptor neg_desc;
  neg_desc.name = "negative";
  neg_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  neg_desc.description =
    "If True the box will be empty Else the remaining points will be the ones in the box";
  declare_parameter(neg_desc.name, rclcpp::ParameterValue(false), neg_desc);

  rcl_interfaces::msg::ParameterDescriptor input_frame_desc;
  input_frame_desc.name = "input_frame";
  input_frame_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  input_frame_desc.description =
    "The input TF frame the data should be transformed into before processing, "
    "if input.header.frame_id is different.";
  declare_parameter(input_frame_desc.name, rclcpp::ParameterValue(""), input_frame_desc);

  rcl_interfaces::msg::ParameterDescriptor output_frame_desc;
  output_frame_desc.name = "output_frame";
  output_frame_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  output_frame_desc.description =
    "The output TF frame the data should be transformed into after processing, "
    "if input.header.frame_id is different.";
  declare_parameter(output_frame_desc.name, rclcpp::ParameterValue(""), output_frame_desc);

  // Validate initial values using same callback
  callback_handle_ =
    add_on_set_parameters_callback(
      std::bind(
        &CropBox::config_callback, this,
        std::placeholders::_1));
  std::vector<std::string> param_names{
    min_x_desc.name,
    min_y_desc.name,
    min_z_desc.name,
    max_x_desc.name,
    max_y_desc.name,
    max_z_desc.name,
    keep_organized_desc.name,
    neg_desc.name,
    input_frame_desc.name,
    output_frame_desc.name
  };
  auto result = config_callback(get_parameters(param_names));
  if (!result.successful) {
    throw std::runtime_error(result.reason);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
rcl_interfaces::msg::SetParametersResult
pcl_ros::CropBox::config_callback(const std::vector<rclcpp::Parameter> & params)
{
  std::lock_guard<std::mutex> lock(mutex_);

  Eigen::Vector4f min_point, max_point;
  min_point = impl_.getMin();
  max_point = impl_.getMax();

  Eigen::Vector4f new_min_point, new_max_point;
  new_min_point = min_point;
  new_max_point = max_point;

  for (const rclcpp::Parameter & param : params) {
    if (param.get_name() == "min_x") {
      new_min_point[0] = param.as_double();
    }
    if (param.get_name() == "max_x") {
      new_max_point[0] = param.as_double();
    }
    if (param.get_name() == "min_y") {
      new_min_point[1] = param.as_double();
    }
    if (param.get_name() == "max_y") {
      new_max_point[1] = param.as_double();
    }
    if (param.get_name() == "min_z") {
      new_min_point[2] = param.as_double();
    }
    if (param.get_name() == "max_z") {
      new_max_point[2] = param.as_double();
    }
  }

  // Check the current values for minimum point
  if (min_point != new_min_point) {
    RCLCPP_DEBUG(
      get_logger(), "Setting the minimum point to: %f %f %f.",
      new_min_point(0), new_min_point(1), new_min_point(2));
    // Set the filter min point if different
    impl_.setMin(new_min_point);
  }
  // Check the current values for the maximum point
  if (max_point != new_max_point) {
    RCLCPP_DEBUG(
      get_logger(), "Setting the maximum point to: %f %f %f.",
      new_max_point(0), new_max_point(1), new_max_point(2));
    // Set the filter max point if different
    impl_.setMax(new_max_point);
  }

  for (const rclcpp::Parameter & param : params) {
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
    if (param.get_name() == "negative") {
      // Check the current value for the negative flag
      if (impl_.getNegative() != param.as_bool()) {
        RCLCPP_DEBUG(
          get_logger(), "Setting the filter negative flag to: %s.",
          param.as_bool() ? "true" : "false");
        // Call the virtual method in the child
        impl_.setNegative(param.as_bool());
      }
    }
    if (param.get_name() == "input_frame") {
      // The following parameters are updated automatically for all PCL_ROS Nodelet Filters
      // as they are inexistent in PCL
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
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_ros::CropBox)
