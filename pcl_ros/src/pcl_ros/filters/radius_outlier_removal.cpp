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

#include <pluginlib/class_list_macros.hpp>
#include "pcl_ros/filters/radius_outlier_removal.h"

//////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl_ros::RadiusOutlierRemoval::child_init (rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_param, bool &has_service)
{
  // TODO: remove?
  // Enable the dynamic reconfigure service
  has_service = true;

  rcl_interfaces::msg::ParameterDescriptor radius_search_desc;
  radius_search_desc.name = "radius_search";
  radius_search_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  radius_search_desc.description = "Radius of the sphere that will determine which points are neighbors.";
  rcl_interfaces::msg::FloatingPointRange radius_search_range;
  radius_search_range.from_value = 0.0;
  radius_search_range.to_value = 10.0;
  radius_search_desc.floating_point_range.push_back (radius_search_range);
  double radius_search = node_param->declare_parameter (radius_search_desc.name, rclcpp::ParameterValue(0.1), radius_search_desc).get<double>();
  impl_.setRadiusSearch (radius_search);

  rcl_interfaces::msg::ParameterDescriptor min_neighbors_desc;
  min_neighbors_desc.name = "min_neighbors";
  min_neighbors_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  min_neighbors_desc.description = "The number of neighbors that need to be present in order to be classified as an inlier.";
  rcl_interfaces::msg::IntegerRange min_neighbors_range;
  min_neighbors_range.from_value = 0;
  min_neighbors_range.to_value = 1000;
  min_neighbors_desc.integer_range.push_back (min_neighbors_range);
  int min_neighbors = node_param->declare_parameter (min_neighbors_desc.name, rclcpp::ParameterValue(5), min_neighbors_desc).get<int>();
  impl_.setMinNeighborsInRadius (min_neighbors);

  // TODO
  node_param->set_on_parameters_set_callback (boost::bind (&RadiusOutlierRemoval::config_callback, this, _1));

  //srv_ = boost::make_shared <dynamic_reconfigure::Server<pcl_ros::RadiusOutlierRemovalConfig> > (nh);
  //dynamic_reconfigure::Server<pcl_ros::RadiusOutlierRemovalConfig>::CallbackType f = boost::bind (&RadiusOutlierRemoval::config_callback, this, _1, _2);
  //srv_->setCallback (f);

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
rcl_interfaces::msg::SetParametersResult
pcl_ros::RadiusOutlierRemoval::config_callback (const std::vector<rclcpp::Parameter> & params)
{
  std::lock_guard<std::mutex> lock(mutex_);

  for (const rclcpp::Parameter &param : params)
  {
    if (param.get_name () == "min_neighbors")
    {
      if (impl_.getMinNeighborsInRadius () != param.as_int ())
      {
        impl_.setMinNeighborsInRadius (param.as_int ());
        RCLCPP_DEBUG(get_logger(), "Setting the number of neighbors in radius: %d.", param.as_int ());
      }
    } 
    else if (param.get_name () == "radius_search")
    {
      if (impl_.getRadiusSearch () != param.as_double ())
      {
        impl_.setRadiusSearch (param.as_double ());
        RCLCPP_DEBUG(get_logger(), "Setting the radius to search neighbors: %f.", param.as_double ());
      }
    }
  }
  // TODO(sloretz) constraint validation
  return rcl_interfaces::msg::SetParametersResult();
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_ros::RadiusOutlierRemoval)
