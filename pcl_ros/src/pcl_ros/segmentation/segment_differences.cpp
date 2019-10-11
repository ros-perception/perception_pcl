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
 * $Id: segment_differences.cpp 35361 2011-01-20 04:34:49Z rusu $
 *
 */

#include "pcl_ros/segmentation/segment_differences.h"
#include <pcl/io/io.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl_ros::SegmentDifferences::SegmentDifferences (const rclcpp::NodeOptions& options) : PCLNode("SegmentDifferencesNode", options)
{
  pub_output_ = this->create_publisher<PointCloud> ("output", max_queue_size_);

  RCLCPP_DEBUG (this->get_logger(), "[%s::onConstructor] Node successfully created with the following parameters:\n"
                 " - max_queue_size    : %d",
                 this->get_name (),
                 max_queue_size_);

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::SegmentDifferences::subscribe ()
{
  // Subscribe to the input using a filter
  sub_input_filter_.subscribe (this->shared_from_this (), "input");
  sub_target_filter_.subscribe (this->shared_from_this (), "target");

  if (approximate_sync_)
  {
    sync_input_target_a_ = std::make_shared <message_filters::Synchronizer<sync_policies::ApproximateTime<PointCloud, PointCloud> > > (max_queue_size_);
    sync_input_target_a_->connectInput (sub_input_filter_, sub_target_filter_);
    sync_input_target_a_->registerCallback (std::bind (&SegmentDifferences::input_target_callback, this, std::placeholders::_1, std::placeholders::_2));
  }
  else
  {
    sync_input_target_e_ = std::make_shared <message_filters::Synchronizer<sync_policies::ExactTime<PointCloud, PointCloud> > > (max_queue_size_);
    sync_input_target_e_->connectInput (sub_input_filter_, sub_target_filter_);
    sync_input_target_e_->registerCallback (std::bind (&SegmentDifferences::input_target_callback, this, std::placeholders::_1, std::placeholders::_2));
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::SegmentDifferences::unsubscribe ()
{
  sub_input_filter_.unsubscribe ();
  sub_target_filter_.unsubscribe ();
}


//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::SegmentDifferences::input_target_callback (const PointCloudConstPtr &cloud, 
                                                    const PointCloudConstPtr &cloud_target)
{
  /*
   count_subscribers not yet implemented ROS2
  if (pub_output_.count_subscribers () <= 0)
    return;
   */

  if (!isValid (cloud) || !isValid (cloud_target, "target")) 
  {
    RCLCPP_ERROR (this->get_logger(), "[%s::input_indices_callback] Invalid input!", this->get_name ());
    PointCloud output;
    output.header = cloud->header;
    pub_output_->publish (output);
    return;
  }

  RCLCPP_DEBUG (this->get_logger (), "[%s::input_indices_callback]\n"
                 "                                 - PointCloud with %d data points (%s), stamp %f, and frame %s on topic %s received.\n"
                 "                                 - PointCloud with %d data points (%s), stamp %f, and frame %s on topic %s received.",
                 this->get_name (),
                 cloud->width * cloud->height, pcl::getFieldsList (*cloud).c_str (), fromPCL(cloud->header).stamp.sec, cloud->header.frame_id.c_str (), "input",
                 cloud_target->width * cloud_target->height, pcl::getFieldsList (*cloud_target).c_str (), fromPCL(cloud_target->header).stamp.sec, cloud_target->header.frame_id.c_str (), "target");

  impl_.setInputCloud (cloud);
  impl_.setTargetCloud (cloud_target);

  PointCloud output;
  impl_.segment (output);

  pub_output_->publish (output);
  RCLCPP_DEBUG (this->get_logger(), "[%s::segmentAndPublish] Published PointCloud2 with %zu points and stamp %f on topic %s", this->get_name (),
                     output.points.size (), fromPCL(output.header).stamp.sec, "output");
}

typedef pcl_ros::SegmentDifferences SegmentDifferences;
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(SegmentDifferences)
