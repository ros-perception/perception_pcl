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
 * $Id: project_inliers.cpp 35876 2011-02-09 01:04:36Z rusu $
 *
 */

#include "pcl_ros/filters/project_inliers.hpp"

//////////////////////////////////////////////////////////////////////////////////////////////
namespace pcl_ros
{
ProjectInliers::ProjectInliers(const rclcpp::NodeOptions & options)
: PCLNode("ProjectInliersNode", options, std::vector<std::string>{"input", "indices", "model"},
    std::vector<std::string>{"output"})
{
  // ---[ Mandatory parameters
  // The type of model to use (user given parameter).
  declare_parameter("model_type", rclcpp::ParameterType::PARAMETER_INTEGER);
  int model_type;
  if (!get_parameter("model_type", model_type)) {
    RCLCPP_ERROR(
      get_logger(),
      "[onConstruct] Need a 'model_type' parameter to be set before continuing!");
    return;
  }
  // ---[ Optional parameters
  // True if all data will be returned, false if only the projected inliers. Default: false.
  rcl_interfaces::msg::ParameterDescriptor copy_all_data_desc;
  copy_all_data_desc.name = "copy_all_data";
  copy_all_data_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  copy_all_data_desc.description =
    "Whether all data will be returned, or only the projected inliers."
    "true if all data should be returned, false if only the projected inliers";
  // Optional Parameter - Default Value: false
  declare_parameter(
    copy_all_data_desc.name, rclcpp::ParameterValue(false), copy_all_data_desc);

  // True if all fields will be returned, false if only XYZ. Default: true.
  rcl_interfaces::msg::ParameterDescriptor copy_all_fields_desc;
  copy_all_fields_desc.name = "copy_all_fields";
  copy_all_fields_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  copy_all_fields_desc.description =
    "Whether all fields should be copied, or only the XYZ."
    "true if all fields will be returned, false if only XYZ";
  // Optional Parameter - Default Value: true
  declare_parameter(
    copy_all_fields_desc.name, rclcpp::ParameterValue(true), copy_all_fields_desc);
}

void ProjectInliers::compute(
  const PointCloud2 & input, const PointIndices & indices,
  const ModelCoefficients & model, PointCloud2 & output)
{
  if(input.data.empty()) {
    output = input;
    return;
  }

  pcl::PCLPointCloud2::Ptr pcl_input(new pcl::PCLPointCloud2);
  pcl_conversions::toPCL(input, *(pcl_input));
  impl_.setInputCloud(pcl_input);

  IndicesPtr pcl_indices(new pcl::PointIndices);
  pcl_indices->indices = indices.indices;
  impl_.setIndices(pcl_indices);
  pcl::ModelCoefficients::Ptr pcl_model(new pcl::ModelCoefficients);
  pcl_conversions::toPCL(model, *(pcl_model));
  impl_.setModelCoefficients(pcl_model);
  pcl::PCLPointCloud2 pcl_output;
  impl_.filter(pcl_output);
  pcl_conversions::moveFromPCL(pcl_output, output);
}

rcl_interfaces::msg::SetParametersResult ProjectInliers::onParamsChanged(
  const std::vector<rclcpp::Parameter> & params)
{
  for (const rclcpp::Parameter & param : params) {
    if (param.get_name() == "model_type") {
      if (impl_.getModelType() != param.as_int()) {
        RCLCPP_DEBUG(
          get_logger(),
          "Setting the model type to: %ld.",
          param.as_int());
        impl_.setModelType(param.as_int());
      }
    }
    if (param.get_name() == "copy_all_data") {
      if (impl_.getCopyAllData() != param.as_bool()) {
        RCLCPP_DEBUG(
          get_logger(),
          "Setting copy all data to: %s.",
          (param.as_bool() ? "true" : "false"));
        impl_.setCopyAllData(param.as_bool());
      }
    }
    if (param.get_name() == "copy_all_fields") {
      if (impl_.getCopyAllFields() != param.as_bool()) {
        RCLCPP_DEBUG(
          get_logger(),
          "Setting copy all fields to: %s.",
          (param.as_bool() ? "true" : "false"));
        impl_.setCopyAllFields(param.as_bool());
      }
    }
  }

  // Range constraints are enforced by rclcpp::Parameter.
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}
}  // namespace pcl_ros

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_ros::ProjectInliers)
