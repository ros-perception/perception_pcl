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
 * $Id: moving_least_squares.cpp 36097 2011-02-20 14:18:58Z marton $
 *
 */

#include "pcl_ros/surface/moving_least_squares.h"
#include <pcl/io/io.h>
//////////////////////////////////////////////////////////////////////////////////////////////
pcl_ros::MovingLeastSquares::MovingLeastSquares (const rclcpp::NodeOptions& options) : PCLNode("MovingLeastSquaresNode", options)
{
  //ros::NodeHandle private_nh = getMTPrivateNodeHandle ();
  pub_output_ = this->create_publisher<PointCloudIn> ("output");
  pub_normals_ = this->create_publisher<NormalCloudOut> ("normals");
  
  //if (!this->getParam ("k_search", k_) && !this->getParam ("search_radius", search_radius_))  
  if (!this->get_parameter ("search_radius", search_radius_))
  {
    RCLCPP_ERROR (this->get_logger(), "[%s::onConstructor] Need a 'search_radius' parameter to be set before continuing!", this->get_name ());
    return;
  }
  if (!this->get_parameter ("spatial_locator", spatial_locator_type_))
  { 
    RCLCPP_ERROR (this->get_logger(), "[%s::onConstructor] Need a 'spatial_locator' parameter to be set before continuing!", this->get_name ());
    return;
  }

  // ---[ Optional parameters
  this->get_parameter ("use_indices", use_indices_);

  RCLCPP_DEBUG (this->get_logger(), "[%s::onConstructor] Node successfully created with the following parameters:\n"
                 " - use_indices    : %s",
                 getName ().c_str (), 
                 (use_indices_) ? "true" : "false");

}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::MovingLeastSquares::subscribe ()
{
  // If we're supposed to look for PointIndices (indices)
  if (use_indices_)
  {
    // Subscribe to the input using a filter
    sub_input_filter_.subscribe (this->shared_from_this (), "input");
    // If indices are enabled, subscribe to the indices
    sub_indices_filter_.subscribe (this->shared_from_this (), "indices");

    if (approximate_sync_)
    {
      sync_input_indices_a_ = std::make_shared <message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<PointCloudIn, PointIndices> > >(max_queue_size_);
      // surface not enabled, connect the input-indices duo and register
      sync_input_indices_a_->connectInput (sub_input_filter_, sub_indices_filter_);
      sync_input_indices_a_->registerCallback (std::bind (&MovingLeastSquares::input_indices_callback, this, std::placeholders::_1, std::placeholders::_2));
    }
    else
    {
      sync_input_indices_e_ = std::make_shared <message_filters::Synchronizer<message_filters::sync_policies::ExactTime<PointCloudIn, PointIndices> > >(max_queue_size_);
      // surface not enabled, connect the input-indices duo and register
      sync_input_indices_e_->connectInput (sub_input_filter_, sub_indices_filter_);
      sync_input_indices_e_->registerCallback (std::bind (&MovingLeastSquares::input_indices_callback, this, std::placeholders::_1, std::placeholders::_2));
    }
  }
  else
    // Subscribe in an old fashion to input only (no filters)
    // Type masquerading not yet supported
    //sub_input_ = this->create_subscription<PointCloudIn> ("input", std::bind (&MovingLeastSquares::input_indices_callback, this, std::placeholders::_1, PointIndicesConstPtr ()));
    std::cout << "Type masquerading not yet supported" << std::endl;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::MovingLeastSquares::unsubscribe ()
{
  if (use_indices_)
  {
    sub_input_filter_.unsubscribe ();
    sub_indices_filter_.unsubscribe ();
  }
  else
    sub_input_.shutdown ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::MovingLeastSquares::input_indices_callback (const PointCloudInConstPtr &cloud, 
                                                     const PointIndicesConstPtr &indices)
{
  // No subscribers, no work
  /*
  count_subscribers not implemented yet ROS2
  if (pub_output_.count_subscribers () <= 0 && pub_normals_.count_subscribers () <= 0)
    return;
  */

  // Output points have the same type as the input, they are only smoothed
  PointCloudIn output;
  
  // Normals are also estimated and published on a separate topic
  NormalCloudOut::Ptr normals (new NormalCloudOut ());

  // If cloud is given, check if it's valid
  if (!isValid (cloud)) 
  {
    RCLCPP_ERROR (this->get_logger(), "[%s::input_indices_callback] Invalid input!", this->get_name ());
    output.header = cloud->header;
    pub_output_->publish (output.makeShared ());
    return;
  }
  // If indices are given, check if they are valid
  if (indices && !isValid (indices, "indices"))
  {
    RCLCPP_ERROR (this->get_logger(), "[%s::input_indices_callback] Invalid indices!", this->get_name ());
    output.header = cloud->header;
    pub_output_->publish (output.makeShared ());
    return;
  }

  /// DEBUG
  if (indices)
    RCLCPP_DEBUG (this->get_logger(), "[%s::input_indices_model_callback]\n"
                   "                                 - PointCloud with %d data points (%s), stamp %f, and frame %s on topic %s received.\n"
                   "                                 - PointIndices with %zu values, stamp %f, and frame %s on topic %s received.",
                   this->get_name (),
                   cloud->width * cloud->height, pcl::getFieldsList (*cloud).c_str (), fromPCL(cloud->header).stamp.sec, cloud->header.frame_id.c_str (), "input",
                   indices->indices.size (), indices->header.stamp.sec, indices->header.frame_id.c_str (), "indices");
  else
    RCLCPP_DEBUG (this->get_logger(), "[%s::input_callback] PointCloud with %d data points, stamp %f, and frame %s on topic %s received.", this->get_name (), cloud->width * cloud->height, fromPCL(cloud->header).stamp.sec, cloud->header.frame_id.c_str (), "input");
  ///

  // Reset the indices and surface pointers
  impl_.setInputCloud (cloud);

  IndicesPtr indices_ptr;
  if (indices)
    indices_ptr.reset (new std::vector<int> (indices->indices));

  impl_.setIndices (indices_ptr.get());

  // Initialize the spatial locator
  
  // Do the reconstructon
  //impl_.process (output);

  // Publish a shared ptr const data
  // Enforce that the TF frame and the timestamp are copied
  output.header = cloud->header;
  pub_output_->publish (output);
  normals->header = cloud->header;
  pub_normals_->publish (normals);
}

typedef pcl_ros::MovingLeastSquares MovingLeastSquares;

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(MovingLeastSquares)
