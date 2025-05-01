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
  // This both declares and initializes the input and output frames
  use_frame_params();

  rcl_interfaces::msg::ParameterDescriptor min_x_desc;
  min_x_desc.name = "min_x";
  min_x_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  min_x_desc.description =
    "Minimum x value below which points will be removed";
  {
    rcl_interfaces::msg::FloatingPointRange float_range;
    float_range.from_value = -1000.0;
    float_range.to_value = 1000.0;
    min_x_desc.floating_point_range.push_back(float_range);
  }
  declare_parameter(min_x_desc.name, rclcpp::ParameterValue(-1.0), min_x_desc);

  rcl_interfaces::msg::ParameterDescriptor max_x_desc;
  max_x_desc.name = "max_x";
  max_x_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  max_x_desc.description =
    "Maximum x value above which points will be removed";
  {
    rcl_interfaces::msg::FloatingPointRange float_range;
    float_range.from_value = -1000.0;
    float_range.to_value = 1000.0;
    max_x_desc.floating_point_range.push_back(float_range);
  }
  declare_parameter(max_x_desc.name, rclcpp::ParameterValue(1.0), max_x_desc);

  rcl_interfaces::msg::ParameterDescriptor min_y_desc;
  min_y_desc.name = "min_y";
  min_y_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  min_y_desc.description =
    "Minimum y value below which points will be removed";
  {
    rcl_interfaces::msg::FloatingPointRange float_range;
    float_range.from_value = -1000.0;
    float_range.to_value = 1000.0;
    min_y_desc.floating_point_range.push_back(float_range);
  }
  declare_parameter(min_y_desc.name, rclcpp::ParameterValue(-1.0), min_y_desc);

  rcl_interfaces::msg::ParameterDescriptor max_y_desc;
  max_y_desc.name = "max_y";
  max_y_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  max_y_desc.description =
    "Maximum y value above which points will be removed";
  {
    rcl_interfaces::msg::FloatingPointRange float_range;
    float_range.from_value = -1000.0;
    float_range.to_value = 1000.0;
    max_y_desc.floating_point_range.push_back(float_range);
  }
  declare_parameter(max_y_desc.name, rclcpp::ParameterValue(1.0), max_y_desc);

  rcl_interfaces::msg::ParameterDescriptor min_z_desc;
  min_z_desc.name = "min_z";
  min_z_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  min_z_desc.description =
    "Minimum z value below which points will be removed";
  {
    rcl_interfaces::msg::FloatingPointRange float_range;
    float_range.from_value = -1000.0;
    float_range.to_value = 1000.0;
    min_z_desc.floating_point_range.push_back(float_range);
  }
  declare_parameter(min_z_desc.name, rclcpp::ParameterValue(-1.0), min_z_desc);

  rcl_interfaces::msg::ParameterDescriptor max_z_desc;
  max_z_desc.name = "max_z";
  max_z_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  max_z_desc.description =
    "Maximum z value above which points will be removed";
  {
    rcl_interfaces::msg::FloatingPointRange float_range;
    float_range.from_value = -1000.0;
    float_range.to_value = 1000.0;
    max_z_desc.floating_point_range.push_back(float_range);
  }
  declare_parameter(max_z_desc.name, rclcpp::ParameterValue(1.0), max_z_desc);

  rcl_interfaces::msg::ParameterDescriptor keep_organized_desc;
  keep_organized_desc.name = "keep_organized";
  keep_organized_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  keep_organized_desc.description =
    "Set whether the filtered points should be kept and set to NaN, "
    "or removed from the PointCloud, thus potentially breaking its organized structure.";
  declare_parameter(keep_organized_desc.name, rclcpp::ParameterValue(false), keep_organized_desc);

  rcl_interfaces::msg::ParameterDescriptor negative_desc;
  negative_desc.name = "negative";
  negative_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  negative_desc.description =
    "Set whether the inliers should be returned (true) or the outliers (false).";
  declare_parameter(negative_desc.name, rclcpp::ParameterValue(false), negative_desc);

  const std::vector<std::string> param_names {
    min_x_desc.name,
    max_x_desc.name,
    min_y_desc.name,
    max_y_desc.name,
    min_z_desc.name,
    max_z_desc.name,
    keep_organized_desc.name,
    negative_desc.name,
  };

  callback_handle_ =
    add_on_set_parameters_callback(
    std::bind(
      &CropBox::config_callback, this,
      std::placeholders::_1));

  config_callback(get_parameters(param_names));

  createPublishers();
}

void
pcl_ros::CropBox::filter(
  const PointCloud2::ConstSharedPtr & input, const IndicesPtr & indices,
  PointCloud2 & output)
{
  std::lock_guard<std::mutex> lock(mutex_);
  pcl::PCLPointCloud2::Ptr pcl_input(new pcl::PCLPointCloud2);
  pcl_conversions::toPCL(*(input), *(pcl_input));
  impl_.setInputCloud(pcl_input);
  impl_.setIndices(indices);
  pcl::PCLPointCloud2 pcl_output;
  impl_.filter(pcl_output);
  pcl_conversions::moveFromPCL(pcl_output, output);
  // Publish the crop box as a cube marker for visualization purposes
  if (crop_box_marker_publisher_->get_subscription_count() > 0) {
    crop_box_marker_msg_.header.frame_id =
      tf_input_frame_.empty() ? input->header.frame_id : tf_input_frame_;
    crop_box_marker_msg_.header.stamp = input->header.stamp;
    crop_box_marker_publisher_->publish(crop_box_marker_msg_);
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

  for (const rclcpp::Parameter & param : params) {
    if (param.get_name() == "min_x") {
      min_point(0) = param.as_double();
    }
    if (param.get_name() == "max_x") {
      max_point(0) = param.as_double();
    }
    if (param.get_name() == "min_y") {
      min_point(1) = param.as_double();
    }
    if (param.get_name() == "max_y") {
      max_point(1) = param.as_double();
    }
    if (param.get_name() == "min_z") {
      min_point(2) = param.as_double();
    }
    if (param.get_name() == "max_z") {
      max_point(2) = param.as_double();
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
  }

  auto crop_box_marker_updated = false;
  // Check the current values for minimum point
  if (min_point != impl_.getMin()) {
    RCLCPP_DEBUG(
      get_logger(), "Setting the minimum point to: %f %f %f.",
      min_point(0), min_point(1), min_point(2));
    impl_.setMin(min_point);
    crop_box_marker_updated = true;
  }

  // Check the current values for the maximum point
  if (max_point != impl_.getMax()) {
    RCLCPP_DEBUG(
      get_logger(), "Setting the maximum point to: %f %f %f.",
      max_point(0), max_point(1), max_point(2));
    impl_.setMax(max_point);
    crop_box_marker_updated = true;
  }
  if (crop_box_marker_updated) {
    update_marker_msg();
  }

  // Range constraints are enforced by rclcpp::Parameter.
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

void pcl_ros::CropBox::createPublishers()
{
  pcl_ros::Filter::createPublishers();
  crop_box_marker_publisher_ = create_publisher<visualization_msgs::msg::Marker>(
    "~/crop_box_marker", max_queue_size_);
}

void pcl_ros::CropBox::update_marker_msg()
{
  auto min_point = impl_.getMin();
  auto max_point = impl_.getMax();
  crop_box_marker_msg_.ns = get_name() + std::string("/crop_box_marker");
  crop_box_marker_msg_.type = visualization_msgs::msg::Marker::CUBE;
  crop_box_marker_msg_.action = visualization_msgs::msg::Marker::ADD;
  crop_box_marker_msg_.color.g = 1.0;
  crop_box_marker_msg_.color.a = 0.5;
  crop_box_marker_msg_.frame_locked = true;
  Eigen::Vector4f center = (max_point + min_point) / 2.0;
  Eigen::Vector4f size = max_point - min_point;
  crop_box_marker_msg_.pose.position.x = center.x();
  crop_box_marker_msg_.pose.position.y = center.y();
  crop_box_marker_msg_.pose.position.z = center.z();
  crop_box_marker_msg_.pose.orientation.w = 1.0;
  crop_box_marker_msg_.scale.x = size.x();
  crop_box_marker_msg_.scale.y = size.y();
  crop_box_marker_msg_.scale.z = size.z();
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_ros::CropBox)
