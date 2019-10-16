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

#include "pcl_ros/filters/voxel_grid.h"
#include "pcl_ros/ptr_helper.h"

//////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl_ros::VoxelGrid::child_init (ros::NodeHandle &nh, bool &has_service)
{
  // TODO: Remove?
  // Enable the dynamic reconfigure service
  has_service = true;

  rcl_interfaces::msg::ParameterDescriptor leaf_size_desc;
  leaf_size_desc.name = "leaf_size";
  leaf_size_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  leaf_size_desc.description = "The size of a leaf (on x,y,z) used for downsampling.";
  rcl_interfaces::msg::FloatingPointRange leaf_size_range;
  leaf_size_range.from_value = 0;
  leaf_size_range.to_value = 1.0;
  leaf_size_desc.floating_point_range.push_back (leaf_size_range);
  node_param->declare_parameter (leaf_size_desc.name, 0.01, leaf_size_desc);

  //srv_ = boost::make_shared <dynamic_reconfigure::Server<pcl_ros::VoxelGridConfig> > (nh);
  //dynamic_reconfigure::Server<pcl_ros::VoxelGridConfig>::CallbackType f = boost::bind (&VoxelGrid::config_callback, this, _1, _2);
  //srv_->setCallback (f);

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::VoxelGrid::filter (const PointCloud2::ConstSharedPtr &input,
                            const IndicesPtr &indices, 
                            PointCloud2 &output)
{
  //std::mutex::scoped_lock lock (mutex_);
  pcl::PCLPointCloud2::Ptr pcl_input(new pcl::PCLPointCloud2);
  pcl_conversions::toPCL (*(input), *(pcl_input));
  impl_.setInputCloud (pcl_input);
  impl_.setIndices (to_boost_ptr(indices));
  pcl::PCLPointCloud2 pcl_output;
  impl_.filter (pcl_output);
  pcl_conversions::moveFromPCL(pcl_output, output);
}


typedef pcl_ros::VoxelGrid VoxelGrid;
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(VoxelGrid)
