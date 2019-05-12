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
 * $Id: extract_clusters.hpp 32052 2010-08-27 02:19:30Z rusu $
 *
 */

#include "class_loader/register_macro.hpp"
#include <pcl/io/io.h>
#include <pcl/PointIndices.h>
#include "pcl_ros/segmentation/extract_clusters.h"

#include <pcl_conversions/pcl_conversions.h>

using pcl_conversions::fromPCL;
using pcl_conversions::moveFromPCL;
using pcl_conversions::toPCL;

//////////////////////////////////////////////////////////////////////////////////////////////
pcl_ros::EuclideanClusterExtraction::EuclideanClusterExtraction (const rclcpp::NodeOptions& options) : PCLNode("EuclideanClusterExtraction", options), publish_indices_ (false), max_clusters_ (std::numeric_limits<int>::max ())
{
  // ---[ Mandatory parameters
  double cluster_tolerance;
  if (!this->get_parameter ("cluster_tolerance", cluster_tolerance))
  {
    RCLCPP_ERROR (this->get_logger(), "[%s::onConstructor] Need a 'cluster_tolerance' parameter to be set before continuing!", this->get_name ());
    return;
  }
  int spatial_locator;
  if (!this->get_parameter ("spatial_locator", spatial_locator))
  {
    RCLCPP_ERROR (this->get_logger(), "[%s::onConstructor] Need a 'spatial_locator' parameter to be set before continuing!", this->get_name ());
    return;
  }

  //private_nh.getParam ("use_indices", use_indices_);
  this->get_parameter ("publish_indices", publish_indices_);

  if (publish_indices_)
    pub_output_ = advertise<PointIndices> ("output", max_queue_size_);
  else
    pub_output_ = advertise<PointCloud> ("output", max_queue_size_);

  RCLCPP_DEBUG (this->get_logger(), "[%s::onConstructor] Node successfully created with the following parameters:\n"
                 " - max_queue_size    : %d\n"
                 " - use_indices       : %s\n"
                 " - cluster_tolerance : %f\n",
                 this->get_name (),
                 max_queue_size_,
                 (use_indices_) ? "true" : "false", cluster_tolerance);

  // Set given parameters here
  impl_.setClusterTolerance (cluster_tolerance);

}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::EuclideanClusterExtraction::subscribe ()
{
  // If we're supposed to look for PointIndices (indices)
  if (use_indices_)
  {
    // Subscribe to the input using a filter
    sub_input_filter_.subscribe (this->shared_from_this (), "input");
    sub_indices_filter_.subscribe (this->shared_from_this (), "indices");

    if (approximate_sync_)
    {
      sync_input_indices_a_ = std::make_shared <message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<PointCloud, PointIndices> > > (max_queue_size_);
      sync_input_indices_a_->connectInput (sub_input_filter_, sub_indices_filter_);
      sync_input_indices_a_->registerCallback (std::bind (&EuclideanClusterExtraction::input_indices_callback, this, std::placeholders::_1, std::placeholders::_2));
    }
    else
    {
      sync_input_indices_e_ = std::make_shared <message_filters::Synchronizer<message_filters::sync_policies::ExactTime<PointCloud, PointIndices> > > (max_queue_size_);
      sync_input_indices_e_->connectInput (sub_input_filter_, sub_indices_filter_);
      sync_input_indices_e_->registerCallback (std::bind (&EuclideanClusterExtraction::input_indices_callback, this, std::placeholders::_1, std::placeholders::_2));
    }
  }
  else
    // Subscribe in an old fashion to input only (no filters)
    // Type masquerading not yet supported
    // sub_input_ = this->create_subscription<PointCloud> ("input", std::bind (&EuclideanClusterExtraction::input_indices_callback, this, std::placeholders::_1, PointIndicesConstPtr ()));
    std::cout << "Type masquerading not supported" << std::endl;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::EuclideanClusterExtraction::unsubscribe ()
{
  if (use_indices_)
  {
    sub_input_filter_.unsubscribe ();
    sub_indices_filter_.unsubscribe ();
  }
  else
    std::cout << "shutdown" << std::endl;
    //sub_input_.shutdown ();
}


//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::EuclideanClusterExtraction::input_indices_callback (
      const PointCloudConstPtr &cloud, const PointIndicesConstPtr &indices)
{
  // No subscribers, no work
  /*
   count_subscribers not yet implemented ROS2
  if (pub_output_.count_subscribers () <= 0)
    return;
   */

  // If cloud is given, check if it's valid
  if (!isValid (cloud))
  {
    RCLCPP_ERROR (this->get_logger(), "[%s::input_indices_callback] Invalid input!", this->get_name ());
    return;
  }
  // If indices are given, check if they are valid
  if (indices && !isValid (indices))
  {
    RCLCPP_ERROR (this->get_logger(), "[%s::input_indices_callback] Invalid indices!", this->get_name ());
    return;
  }

  /// DEBUG
  if (indices) {
    std_msgs::msg::Header cloud_header = fromPCL(cloud->header);
    std_msgs::msg::Header indices_header = indices->header;
    RCLCPP_DEBUG (this->get_logger(), "[%s::input_indices_callback]\n"
                   "                                 - PointCloud with %d data points (%s), stamp %f, and frame %s on topic %s received.\n"
                   "                                 - PointIndices with %zu values, stamp %f, and frame %s on topic %s received.",
                   this->get_name (),
                   cloud->width * cloud->height, pcl::getFieldsList (*cloud).c_str (), cloud_header.stamp.sec, cloud_header.frame_id.c_str (), "input",
                   indices->indices.size (), indices_header.stamp.sec, indices_header.frame_id.c_str (), "indices");
  } else {
    RCLCPP_DEBUG (this->get_logger(), "[%s::input_callback] PointCloud with %d data points, stamp %f, and frame %s on topic %s received.", this->get_name (), cloud->width * cloud->height, fromPCL(cloud->header).stamp.sec, cloud->header.frame_id.c_str (), "input");
  }
  ///

  IndicesPtr indices_ptr;
  if (indices)
    indices_ptr.reset (new std::vector<int> (indices->indices));

  impl_.setInputCloud (cloud);
  impl_.setIndices (indices_ptr.get());

  std::vector<pcl::PointIndices> clusters;
  impl_.extract (clusters);

  if (publish_indices_)
  {
    for (size_t i = 0; i < clusters.size (); ++i)
    {
      if ((int)i >= max_clusters_)
        break;
      // TODO: HACK!!! We need to change the PointCloud2 message to add for an incremental sequence ID number.
      pcl_msgs::msg::PointIndices ros_pi;
      moveFromPCL(clusters[i], ros_pi);
      ros_pi.header.stamp += rclcpp::Duration (i * 0.001);
      pub_output_->publish (ros_pi);
    }

    RCLCPP_DEBUG (this->get_logger(), "[segmentAndPublish] Published %zu clusters (PointIndices) on topic %s", clusters.size (), "output");
  }
  else
  {
    for (size_t i = 0; i < clusters.size (); ++i)
    {
      if ((int)i >= max_clusters_)
        break;
      PointCloud output;
      copyPointCloud (*cloud, clusters[i].indices, output);

      //PointCloud output_blob;     // Convert from the templated output to the PointCloud blob
      //pcl::toROSMsg (output, output_blob);
      // TODO: HACK!!! We need to change the PointCloud2 message to add for an incremental sequence ID number.
      std_msgs::msg::Header header = fromPCL(output.header);
      header.stamp += rclcpp::Duration (i * 0.001);
      toPCL(header, output.header);
      // Publish a shared ptr const data
      pub_output_->publish (output.makeShared ());
      RCLCPP_DEBUG (this->get_logger(), "[segmentAndPublish] Published cluster %zu (with %zu values and stamp %f) on topic %s",
                     i, clusters[i].indices.size (), header.stamp.sec, "output");
    }
  }
}

typedef pcl_ros::EuclideanClusterExtraction EuclideanClusterExtraction;
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(EuclideanClusterExtraction)
