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
 * $Id: convex_hull.hpp 32993 2010-09-30 23:08:57Z rusu $
 *
 */

#include <pcl/common/io.h>
#include "pcl_ros/surface/convex_hull.h"
#include "pcl_ros/ptr_helper.h"

//////////////////////////////////////////////////////////////////////////////////////////////
pcl_ros::ConvexHull2D::ConvexHull2D (const rclcpp::NodeOptions& options) : PCLNode("ConvexHull2DNode", options)
{
  pub_output_ = this->create_publisher<PointCloud> ("output", max_queue_size_);
  pub_plane_  = this->create_publisher<geometry_msgs::msg::PolygonStamped> ("output_polygon", max_queue_size_);

  // ---[ Optional parameters
  this->get_parameter ("use_indices", use_indices_);

  RCLCPP_DEBUG (this->get_logger(), "[%s::onConstructor] Node successfully created with the following parameters:\n"
                 " - use_indices    : %s",
                 this->get_name (),
                 (use_indices_) ? "true" : "false");

}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::ConvexHull2D::subscribe()
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
      sync_input_indices_a_ = std::make_shared <message_filters::Synchronizer<sync_policies::ApproximateTime<PointCloud, PointIndices> > >(max_queue_size_);
      // surface not enabled, connect the input-indices duo and register
      sync_input_indices_a_->connectInput (sub_input_filter_, sub_indices_filter_);
      sync_input_indices_a_->registerCallback (std::bind (&ConvexHull2D::input_indices_callback, this, std::placeholders::_1, std::placeholders::_2));
    }
    else
    {
      sync_input_indices_e_ = std::make_shared <message_filters::Synchronizer<sync_policies::ExactTime<PointCloud, PointIndices> > >(max_queue_size_);
      // surface not enabled, connect the input-indices duo and register
      sync_input_indices_e_->connectInput (sub_input_filter_, sub_indices_filter_);
      sync_input_indices_e_->registerCallback (std::bind (&ConvexHull2D::input_indices_callback, this, std::placeholders::_1, std::placeholders::_2));
    }
  }
  else
    // Subscribe in an old fashion to input only (no filters)
    // Type masquerading not yet supported
    // sub_input_ = this->create_subscription<pcl::PointCloud<pcl::PointXYZ>> ("input",  1, std::bind (&ConvexHull2D::input_indices_callback, this, std::placeholders::_1, PointIndicesConstPtr ()));
    std::cout << "Type masquerading not supported" << std::endl;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::ConvexHull2D::unsubscribe()
{
  if (use_indices_)
  {
    sub_input_filter_.unsubscribe();
    sub_indices_filter_.unsubscribe();
  }
  else
    // FIXME
    std::cout << "shutdown" << std::endl;
    //sub_input_.shutdown();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
  pcl_ros::ConvexHull2D::input_indices_callback (const PointCloudPtr &cloud,
                                                 const PointIndicesConstPtr &indices)
{
  // No subscribers, no work
  /*
  count_subscribers not implemented yet ROS2
  if (pub_output_->count_subscribers () <= 0 && pub_plane_->count_subscribers () <= 0)
    return;
  */

  PointCloud output;

  // If cloud is given, check if it's valid
  if (!isValid (cloud))
  {
    RCLCPP_ERROR (this->get_logger(), "[%s::input_indices_callback] Invalid input!", this->get_name ());
    // Publish an empty message
    output.header = cloud->header;
    pub_output_->publish (output);
    return;
  }
  // If indices are given, check if they are valid
  if (indices && !isValid (indices, "indices"))
  {
    RCLCPP_ERROR (this->get_logger(), "[%s::input_indices_callback] Invalid indices!", this->get_name ());
    // Publish an empty message
    output.header = cloud->header;
    pub_output_->publish (output);
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
    RCLCPP_DEBUG (this->get_logger(), "[%s::input_indices_callback] PointCloud with %d data points, stamp %f, and frame %s on topic %s received.", this->get_name (), cloud->width * cloud->height, fromPCL(cloud->header).stamp.sec, cloud->header.frame_id.c_str (), "input");

  // Reset the indices and surface pointers
  IndicesPtr indices_ptr;
  if (indices)
    indices_ptr.reset (new std::vector<int> (indices->indices));

  impl_.setInputCloud (cloud);
  impl_.setIndices (to_boost_ptr (indices_ptr));

  // Estimate the feature
  impl_.reconstruct (output);

  // If more than 3 points are present, send a PolygonStamped hull too
  if (output.points.size () >= 3)
  {
    geometry_msgs::msg::PolygonStamped poly;
    poly.header = fromPCL(output.header);
    poly.polygon.points.resize (output.points.size ());
    // Get three consecutive points (without copying)
    pcl::Vector4fMap O = output.points[1].getVector4fMap ();
    pcl::Vector4fMap B = output.points[0].getVector4fMap ();
    pcl::Vector4fMap A = output.points[2].getVector4fMap ();
    // Check the direction of points -- polygon must have CCW direction
    Eigen::Vector4f OA = A - O;
    Eigen::Vector4f OB = B - O;
    Eigen::Vector4f N = OA.cross3 (OB);
    double theta = N.dot (O);
    bool reversed = false;
    if (theta < (M_PI / 2.0))
      reversed = true;
    for (size_t i = 0; i < output.points.size (); ++i)
    {
      if (reversed)
      {
        size_t j = output.points.size () - i - 1;
        poly.polygon.points[i].x = output.points[j].x;
        poly.polygon.points[i].y = output.points[j].y;
        poly.polygon.points[i].z = output.points[j].z;
      }
      else
      {
        poly.polygon.points[i].x = output.points[i].x;
        poly.polygon.points[i].y = output.points[i].y;
        poly.polygon.points[i].z = output.points[i].z;
      }
    }
    pub_plane_->publish (std::make_shared<const geometry_msgs::msg::PolygonStamped> (poly));
  }
  // Publish a shared ptr const data
  output.header = cloud->header;
  pub_output_->publish (output);
}

typedef pcl_ros::ConvexHull2D ConvexHull2D;

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ConvexHull2D)
