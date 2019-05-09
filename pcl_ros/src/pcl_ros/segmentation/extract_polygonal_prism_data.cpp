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
 * $Id: extract_polygonal_prism_data.hpp 32996 2010-09-30 23:42:11Z rusu $
 *
 */

#include "pcl_ros/transforms.h"
#include "pcl_ros/segmentation/extract_polygonal_prism_data.h"
#include <pcl/io/io.h>

#include <pcl_conversions/pcl_conversions.h>

using pcl_conversions::moveFromPCL;
using pcl_conversions::moveToPCL;

//////////////////////////////////////////////////////////////////////////////////////////////
pcl_ros::ExtractPolygonalPrismData::ExtractPolygonalPrismData (std::string node_name, const rclcpp::NodeOptions& options) : PCLNode(node_name, options)
{
  // Advertise the output topics
  pub_output_ = create_publisher<PointIndices> ("output", max_queue_size_);

}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::ExtractPolygonalPrismData::subscribe ()
{
  sub_hull_filter_.subscribe (this->shared_from_this (), "planar_hull");
  sub_input_filter_.subscribe (this->shared_from_this (), "input");

  // Create the objects here
  if (approximate_sync_)
    sync_input_hull_indices_a_ = std::make_shared <message_filters::Synchronizer<sync_policies::ApproximateTime<PointCloud, PointCloud, PointIndices> > > (max_queue_size_);
  else
    sync_input_hull_indices_e_ = std::make_shared <message_filters::Synchronizer<sync_policies::ExactTime<PointCloud, PointCloud, PointIndices> > > (max_queue_size_);

  if (use_indices_)
  {
    sub_indices_filter_.subscribe (this->shared_from_this (), "indices");
    if (approximate_sync_)
      sync_input_hull_indices_a_->connectInput (sub_input_filter_, sub_hull_filter_, sub_indices_filter_);
    else
      sync_input_hull_indices_e_->connectInput (sub_input_filter_, sub_hull_filter_, sub_indices_filter_);
  }
  else
  {
    sub_input_filter_.registerCallback (std::bind (&ExtractPolygonalPrismData::input_callback, this, std::placeholders::_1));

    if (approximate_sync_)
      sync_input_hull_indices_a_->connectInput (sub_input_filter_, sub_hull_filter_, nf_);
    else
      sync_input_hull_indices_e_->connectInput (sub_input_filter_, sub_hull_filter_, nf_);
  }
  // Register callbacks
  if (approximate_sync_)
    sync_input_hull_indices_a_->registerCallback (std::bind (&ExtractPolygonalPrismData::input_hull_indices_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  else
    sync_input_hull_indices_e_->registerCallback (std::bind (&ExtractPolygonalPrismData::input_hull_indices_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::ExtractPolygonalPrismData::unsubscribe ()
{
  sub_hull_filter_.unsubscribe ();
  sub_input_filter_.unsubscribe ();

  if (use_indices_)
    sub_indices_filter_.unsubscribe ();
}


//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::ExtractPolygonalPrismData::input_hull_indices_callback (
    const PointCloudConstPtr &cloud, 
    const PointCloudConstPtr &hull, 
    const PointIndicesConstPtr &indices)
{
  // No subscribers, no work
  /*
   count_subscribers not yet implemented in ROS2
   if (pub_output_.count_subscribers () <= 0)
    return;
  */

  // Copy the header (stamp + frame_id)
  pcl_msgs::PointIndices inliers;
  inliers.header = fromPCL(cloud->header);

  // If cloud is given, check if it's valid
  if (!isValid (cloud) || !isValid (hull, "planar_hull"))
  {
    RCLCPP_ERROR (this->get_logger(), "[%s::input_hull_indices_callback] Invalid input!", this->get_name ());
    pub_output_->publish (inliers);
    return;
  }
  // If indices are given, check if they are valid
  if (indices && !isValid (indices))
  {
    RCLCPP_ERROR (this->get_logger(), "[%s::input_hull_indices_callback] Invalid indices!", this->get_name ());
    pub_output_->publish (inliers);
    return;
  }

  /// DEBUG
  if (indices)
    RCLCPP_DEBUG (this->get_logger(), "[%s::input_indices_hull_callback]\n"
                   "                                 - PointCloud with %d data points (%s), stamp %f, and frame %s on topic %s received.\n"
                   "                                 - PointCloud with %d data points (%s), stamp %f, and frame %s on topic %s received.\n"
                   "                                 - PointIndices with %zu values, stamp %f, and frame %s on topic %s received.",
                   this->get_name (),
                   cloud->width * cloud->height, pcl::getFieldsList (*cloud).c_str (), fromPCL(cloud->header).stamp.sec, cloud->header.frame_id.c_str (), "input",
                   hull->width * hull->height, pcl::getFieldsList (*hull).c_str (), fromPCL(hull->header).stamp.sec, hull->header.frame_id.c_str (), "planar_hull",
                   indices->indices.size (), indices->header.stamp.sec, indices->header.frame_id.c_str (), "indices");
  else
    RCLCPP_DEBUG (this->get_logger(), "[%s::input_indices_hull_callback]\n"
                   "                                 - PointCloud with %d data points (%s), stamp %f, and frame %s on topic %s received.\n"
                   "                                 - PointCloud with %d data points (%s), stamp %f, and frame %s on topic %s received.",
                   this->get_name (),
                   cloud->width * cloud->height, pcl::getFieldsList (*cloud).c_str (), fromPCL(cloud->header).stamp.sec, cloud->header.frame_id.c_str (), "input",
                   hull->width * hull->height, pcl::getFieldsList (*hull).c_str (), fromPCL(hull->header).stamp.sec, hull->header.frame_id.c_str (), "planar_hull");
  ///

  if (cloud->header.frame_id != hull->header.frame_id)
  {
    RCLCPP_DEBUG (this->get_logger(), "[%s::input_hull_callback] Planar hull has a different TF frame (%s) than the input point cloud (%s)! Using TF to transform.", this->get_name (), hull->header.frame_id.c_str (), cloud->header.frame_id.c_str ());
    PointCloud planar_hull;
    if (!pcl_ros::transformPointCloud (cloud->header.frame_id, *hull, planar_hull, tf_listener_))
    {
      // Publish empty before return
      pub_output_->publish (inliers);
      return;
    }
    impl_.setInputPlanarHull (planar_hull.makeShared ());
  }
  else
    impl_.setInputPlanarHull (hull);

  IndicesPtr indices_ptr;
  if (indices && !indices->header.frame_id.empty ())
    indices_ptr.reset (new std::vector<int> (indices->indices));

  impl_.setInputCloud (cloud);
  impl_.setIndices (indices_ptr.get());

  // Final check if the data is empty (remember that indices are set to the size of the data -- if indices* = NULL)
  if (!cloud->points.empty ()) {
    pcl::PointIndices pcl_inliers;
    moveToPCL(inliers, pcl_inliers);
    impl_.segment (pcl_inliers);
    moveFromPCL(pcl_inliers, inliers);
  }
  // Enforce that the TF frame and the timestamp are copied
  inliers.header = fromPCL(cloud->header);
  pub_output_->publish (inliers);
  RCLCPP_DEBUG (this->get_logger(), "[%s::input_hull_callback] Publishing %zu indices.", this->get_name (), inliers.indices.size ());
}

typedef pcl_ros::ExtractPolygonalPrismData ExtractPolygonalPrismData;
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ExtractPolygonalPrismData)
