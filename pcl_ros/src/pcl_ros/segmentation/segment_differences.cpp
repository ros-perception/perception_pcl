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

#include <rclcpp_components/register_node_macro.hpp>
#include <pcl/common/io.h>
#include "pcl_ros/segmentation/segment_differences.hpp"

using pcl_conversions::fromPCL;

///////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::SegmentDifferences::onInit()
{
  // Call the super onInit ()
  PCLNodelet::onInit();

  pub_output_ = advertise<PointCloud>(*pnh_, "output", max_queue_size_);

  // Declare and get parameters
  distance_threshold_ = pnh_->declare_parameter("distance_threshold", 0.0);
  
  // Set up parameter callback
  param_callback_handle_ = pnh_->add_on_set_parameters_callback(
    std::bind(&SegmentDifferences::parametersCallback, this, std::placeholders::_1));

  RCLCPP_DEBUG(pnh_->get_logger(),
    "[%s::onInit] Nodelet successfully created with the following parameters:\n"
    " - max_queue_size    : %d\n"
    " - distance_threshold: %f",
    getName().c_str(),
    max_queue_size_,
    distance_threshold_);

  onInitPostProcess();
}

///////////////////////////////////////////////////////////////////////////////////////////////////
rcl_interfaces::msg::SetParametersResult
pcl_ros::SegmentDifferences::parametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & parameter : parameters) {
    if (parameter.get_name() == "distance_threshold") {
      distance_threshold_ = parameter.as_double();
      impl_.setDistanceThreshold(distance_threshold_);
      RCLCPP_DEBUG(pnh_->get_logger(),
        "[%s::parametersCallback] Setting new distance threshold to: %f.",
        getName().c_str(), distance_threshold_);
    }
  }

  return result;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::SegmentDifferences::subscribe()
{
  // Subscribe to the input using a filter
  sub_input_filter_.subscribe(*pnh_, "input", rmw_qos_profile_sensor_data);
  sub_target_filter_.subscribe(*pnh_, "target", rmw_qos_profile_sensor_data);

  if (approximate_sync_) {
    sync_input_target_a_ =
      std::make_shared<message_filters::Synchronizer<
          sync_policies::ApproximateTime<PointCloud, PointCloud>>>(max_queue_size_);
    sync_input_target_a_->connectInput(sub_input_filter_, sub_target_filter_);
    sync_input_target_a_->registerCallback(
      std::bind(&SegmentDifferences::input_target_callback, this,
        std::placeholders::_1, std::placeholders::_2));
  } else {
    sync_input_target_e_ =
      std::make_shared<message_filters::Synchronizer<
          sync_policies::ExactTime<PointCloud, PointCloud>>>(max_queue_size_);
    sync_input_target_e_->connectInput(sub_input_filter_, sub_target_filter_);
    sync_input_target_e_->registerCallback(
      std::bind(&SegmentDifferences::input_target_callback, this,
        std::placeholders::_1, std::placeholders::_2));
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::SegmentDifferences::unsubscribe()
{
  sub_input_filter_.unsubscribe();
  sub_target_filter_.unsubscribe();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::SegmentDifferences::input_target_callback(
  const PointCloudConstPtr & cloud,
  const PointCloudConstPtr & cloud_target)
{
  if (pub_output_.getNumSubscribers() <= 0) {
    return;
  }

  if (!isValid(cloud) || !isValid(cloud_target, "target")) {
    RCLCPP_ERROR(pnh_->get_logger(), "[%s::input_target_callback] Invalid input!", getName().c_str());
    PointCloud output;
    output.header = cloud->header;
    pub_output_.publish(output.makeShared());
    return;
  }

  RCLCPP_DEBUG(pnh_->get_logger(),
    "[%s::input_target_callback]\n"
    "                                 - PointCloud with %d data points (%s), stamp %f, and "
    "frame %s on topic %s received.\n"
    "                                 - PointCloud with %d data points (%s), stamp %f, and "
    "frame %s on topic %s received.",
    getName().c_str(),
    cloud->width * cloud->height, pcl::getFieldsList(*cloud).c_str(), 
    rclcpp::Time(cloud->header.stamp).seconds(), 
    cloud->header.frame_id.c_str(), 
    pnh_->resolve_topic_name("input").c_str(),
    cloud_target->width * cloud_target->height, pcl::getFieldsList(*cloud_target).c_str(),
    rclcpp::Time(cloud_target->header.stamp).seconds(),
    cloud_target->header.frame_id.c_str(), 
    pnh_->resolve_topic_name("target").c_str());

  impl_.setInputCloud(cloud);
  impl_.setTargetCloud(cloud_target);

  PointCloud output;
  impl_.segment(output);

  pub_output_.publish(output.makeShared());
  RCLCPP_DEBUG(pnh_->get_logger(),
    "[%s::input_target_callback] Published PointCloud2 with %zu points and stamp %f on topic %s",
    getName().c_str(),
    output.points.size(), 
    rclcpp::Time(output.header.stamp).seconds(),
    pnh_->resolve_topic_name("output").c_str());
}

RCLCPP_COMPONENTS_REGISTER_NODE(pcl_ros::SegmentDifferences)