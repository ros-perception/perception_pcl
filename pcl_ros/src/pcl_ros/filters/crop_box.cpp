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

#include "class_loader/register_macro.hpp"
#include "pcl_ros/filters/crop_box.h"

//////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl_ros::CropBox::child_init (rclcpp::node_interfaces::NodeParametersInterfaces::SharedPtr node_param, bool &has_service)
{
  // TODO: Remove?
  // Enable the dynamic reconfigure service
  has_service = true;

  rcl_interfaces::msg::ParameterDescriptor min_x_desc;
  min_x_desc.name = "min_x";
  min_x_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  min_x_desc.description = "X coordinate of the minimum point of the box.";
  rcl_interfaces::msg::FloatingPointRange min_x_range;
  min_x_range.from_value = -1000;
  min_x_range.to_value = 1000;
  min_x_desc.floating_point_range.push_back (min_x_range);
  //double min_x = 
  node_param->declare_parameter (min_x_desc.name, -1, min_x_desc);
  //node_param->get_parameter (min_x_desc.name, min_x);

  rcl_interfaces::msg::ParameterDescriptor max_x_desc;
  max_x_desc.name = "max_x";
  max_x_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  max_x_desc.description = "X coordinate of the maximum point of the box.";
  rcl_interfaces::msg::FloatingPointRange max_x_range;
  max_x_range.from_value = -1000;
  max_x_range.to_value = 1000;
  max_x_desc.floating_point_range.push_back (max_x_range);
  //double max_x = 
  node_param->declare_parameter (max_x_desc.name, 1, max_x_desc);
  //node_param->get_parameter (max_x_desc.name, max_x);

  rcl_interfaces::msg::ParameterDescriptor min_y_desc;
  min_y_desc.name = "min_y";
  min_y_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  min_y_desc.description = "Y coordinate of the minimum point of the box.";
  rcl_interfaces::msg::FloatingPointRange min_y_range;
  min_y_range.from_value = -1000;
  min_y_range.to_value = 1000;
  min_y_desc.floating_point_range.push_back (min_y_range);
  //double min_y = 
  node_param->declare_parameter (min_y_desc.name, -1, min_y_desc);
  //node_param->get_parameter (min_y_desc.name, min_y);

  rcl_interfaces::msg::ParameterDescriptor max_y_desc;
  max_y_desc.name = "max_y";
  max_y_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  max_y_desc.description = "Y coordinate of the maximum point of the box.";
  rcl_interfaces::msg::FloatingPointRange max_y_range;
  max_y_range.from_value = -1000;
  max_y_range.to_value = 1000;
  max_y_desc.floating_point_range.push_back (max_y_range);
  //double max_y = 
  node_param->declare_parameter (max_y_desc.name, 1, max_y_desc);
  //node_param->get_parameter (max_y_desc.name, max_y);

  rcl_interfaces::msg::ParameterDescriptor min_z_desc;
  min_z_desc.name = "min_z";
  min_z_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  min_z_desc.description = "Z coordinate of the minimum point of the box.";
  rcl_interfaces::msg::FloatingPointRange min_z_range;
  min_z_range.from_value = -1000;
  min_z_range.to_value = 1000;
  min_z_desc.floating_point_range.push_back (min_z_range);
  //double min_z = 
  node_param->declare_parameter (min_z_desc.name, -1, min_z_desc);
  //node_param->get_parameter (min_z_desc.name, min_z);

  rcl_interfaces::msg::ParameterDescriptor max_z_desc;
  max_z_desc.name = "max_z";
  max_z_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  max_z_desc.description = "Z coordinate of the maximum point of the box.";
  rcl_interfaces::msg::FloatingPointRange max_z_range;
  max_z_range.from_value = -1000;
  max_z_range.to_value = 1000;
  max_z_desc.floating_point_range.push_back (max_z_range);
  //double max_z = 
  node_param->declare_parameter (max_z_desc.name, 1, max_z_desc);
  //node_param->get_parameter (max_z_desc.name, max_z);

  rcl_interfaces::msg::ParameterDescriptor keep_organized_desc;
  keep_organized_desc.name = "keep_organized";
  keep_organized_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  keep_organized_desc.description = "Set whether the filtered points should be kept and set to NaN, or removed from the PointCloud, thus potentially breaking its organized structure.";
  //bool keep_organized =
  node_param->declare_parameter (keep_organized_desc.name, false, keep_organized_desc);
  //node_param->get_parameter (keep_organized_desc.name, keep_organized);

  rcl_interfaces::msg::ParameterDescriptor neg_desc;
  neg_desc.name = "negative";
  neg_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  neg_desc.description = "If True the box will be empty Else the remaining points will be the ones in the box";
  //double neg = 
  node_param->declare_parameter (neg_desc.name, false, neg_desc);
  //node_param->get_parameter (neg_desc.name, neg);

  rcl_interfaces::msg::ParameterDescriptor input_frame_desc;
  input_frame_desc.name = "input_frame";
  input_frame_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  input_frame_desc.description = "The input TF frame the data should be transformed into before processing, if input.header.frame_id is different.";
  //std::string input_frame = 
  node_param->declare_parameter (input_frame_desc.name, "", input_frame_desc);
  //node_param->get_parameter (input_frame_desc.name, input_frame);

  rcl_interfaces::msg::ParameterDescriptor output_frame_desc;
  output_frame_desc.name = "output_frame";
  output_frame_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  output_frame_desc.description = "The output TF frame the data should be transformed into after processing, if input.header.frame_id is different.";
  //std::string output_frame = 
  node_param->declare_parameter (output_frame_desc.name, "", output_frame_desc);
  //node_param->get_parameter (output_frame_desc.name, output_frame);

  // TODO
  node_param->set_on_parameters_set_callback (boost::bind (&CropBox::config_callback, this, _1));

  //srv_ = boost::make_shared <dynamic_reconfigure::Server<pcl_ros::CropBoxConfig> > (nh);
  //dynamic_reconfigure::Server<pcl_ros::CropBoxConfig>::CallbackType f = boost::bind (&CropBox::config_callback, this, _1, _2);
  //srv_->setCallback (f);

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::CropBox::config_callback (const std::vector<rclcpp::Parameter> & params)
{
  boost::mutex::scoped_lock lock (mutex_);

  Eigen::Vector4f min_point,max_point; 
  min_point = impl_.getMin();
  max_point = impl_.getMax();

  double min_x = min_point(0);
  double min_y = min_point(1);
  double min_z = min_point(2);
  double max_x = max_point(0);
  double max_y = max_point(1);
  double max_z = max_point(2);

  for (const rclcpp::Parameter &param : params)
  {
    if (param.get_name () = "min_x")
    {
      min_x = param.as_double ();
    }
    if (param.get_name () = "max_x")
    {
      max_x = param.as_double ();
    }
    if (param.get_name () = "min_y")
    {
      min_y = param.as_double ();
    }
    if (param.get_name () = "max_y")
    {
      max_y = param.as_double ();
    }
    if (param.get_name () = "min_z")
    {
      min_z = param.as_double ();
    }
    if (param.get_name () = "max_z")
    {
      max_z = param.as_double ();
    }
    if (param.get_name () = "keep_organized")
    {
      // Check the current value for keep_organized
      if (impl_.getKeepOrganized () != param.as_bool ())
      {
        NODELET_DEBUG ("[%s::config_callback] Setting the filter keep_organized value to: %s.", getName ().c_str (), param.as_bool () ? "true" : "false");
        // Call the virtual method in the child
        impl_.setKeepOrganized (param.as_bool ());
      }
    }
    if (param.get_name () = "negative")
    {
      // Check the current value for the negative flag
      if (impl_.getNegative() != param.as_bool ())
      {
        NODELET_DEBUG ("[%s::config_callback] Setting the filter negative flag to: %s.", getName ().c_str (), param.as_bool () ? "true" : "false");
        // Call the virtual method in the child
        impl_.setNegative(param.as_bool ());
      }
    }
    if (param.get_name () = "input_frame")
    {
      // The following parameters are updated automatically for all PCL_ROS Nodelet Filters as they are inexistent in PCL
      if (tf_input_frame_ != param.as_string ())
      {
        tf_input_frame_ = param.as_string ();
        NODELET_DEBUG ("[%s::config_callback] Setting the input TF frame to: %s.", getName ().c_str (), tf_input_frame_.c_str ());
      }
    }
    if (param.get_name () = "output_frame")
    {
      if (tf_output_frame_ != param.as_string ())
      {
        tf_output_frame_ = param.as_string ();
        NODELET_DEBUG ("[%s::config_callback] Setting the output TF frame to: %s.", getName ().c_str (), tf_output_frame_.c_str ());
      }
    }
  }

  Eigen::Vector4f new_min_point, new_max_point;
  new_min_point << min_x, min_y, min_z, 0.0;
  new_max_point << max_x, max_y, max_z, 0.0;

  // Check the current values for minimum point
  if (min_point != new_min_point)
  {
    // TODO: Replace with RCLCPP_DEBUG, store rclcpp::Node logging_interface
    // TODO: Replace getName() with some wayto get rclcpp::Node name
    NODELET_DEBUG ("[%s::config_callback] Setting the minimum point to: %f %f %f.", getName ().c_str (), new_min_point(0),new_min_point(1),new_min_point(2));
    // Set the filter min point if different
    impl_.setMin(new_min_point);
  }
 // Check the current values for the maximum point
 if (max_point != new_max_point)
  {
    NODELET_DEBUG ("[%s::config_callback] Setting the maximum point to: %f %f %f.", getName ().c_str (), new_max_point(0),new_max_point(1),new_max_point(2));
    // Set the filter max point if different
    impl_.setMax(new_max_point);
  }
}

typedef pcl_ros::CropBox CropBox;
CLASS_LOADER_REGISTER_CLASS(CropBox, rclcpp::Node)
