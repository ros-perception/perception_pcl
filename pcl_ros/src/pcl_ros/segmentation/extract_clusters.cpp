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

#include <rclcpp_components/register_node_macro.hpp>
#include <pcl/io/io.h>
#include <pcl/PointIndices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>
#include "pcl_ros/segmentation/extract_clusters.hpp"

using pcl_conversions::fromPCL;
using pcl_conversions::moveFromPCL;
using pcl_conversions::toPCL;

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::EuclideanClusterExtraction::onInit()
{
  // Call the super onInit ()
  PCLNodelet::onInit();

  // ---[ Mandatory parameters
  this->declare_parameter("cluster_tolerance", 0.02);
  this->declare_parameter("spatial_locator", 0);
  this->declare_parameter("publish_indices", false);
  this->declare_parameter("cluster_min_size", 1);
  this->declare_parameter("cluster_max_size", std::numeric_limits<int>::max());
  this->declare_parameter("max_clusters", std::numeric_limits<int>::max());

  double cluster_tolerance = this->get_parameter("cluster_tolerance").as_double();
  int spatial_locator = this->get_parameter("spatial_locator").as_int();
  publish_indices_ = this->get_parameter("publish_indices").as_bool();
  max_clusters_ = this->get_parameter("max_clusters").as_int();

  if (publish_indices_) {
    pub_output_ = advertise<PointIndices>(*this, "output", max_queue_size_);
  } else {
    pub_output_ = advertise<PointCloud>(*this, "output", max_queue_size_);
  }

  // Setup parameter callback
  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&EuclideanClusterExtraction::config_callback, this, std::placeholders::_1));

  RCLCPP_DEBUG(
    this->get_logger(),
    "[%s::onInit] Nodelet successfully created with the following parameters:\n"
    " - max_queue_size    : %d\n"
    " - use_indices       : %s\n"
    " - cluster_tolerance : %f\n",
    this->get_name(),
    max_queue_size_,
    (use_indices_) ? "true" : "false", cluster_tolerance);

  // Set given parameters here
  impl_.setClusterTolerance(cluster_tolerance);
  impl_.setMinClusterSize(this->get_parameter("cluster_min_size").as_int());
  impl_.setMaxClusterSize(this->get_parameter("cluster_max_size").as_int());

  onInitPostProcess();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::EuclideanClusterExtraction::subscribe()
{
  // If we're supposed to look for PointIndices (indices)
  if (use_indices_) {
    // Subscribe to the input using a filter
    sub_input_filter_.subscribe(*this, "input", rmw_qos_profile_sensor_data);
    sub_indices_filter_.subscribe(*this, "indices", rmw_qos_profile_default);

    if (approximate_sync_) {
      sync_input_indices_a_ =
        std::make_shared<message_filters::Synchronizer<
            message_filters::sync_policies::ApproximateTime<
              PointCloud, PointIndices>>>(max_queue_size_);
      sync_input_indices_a_->connectInput(sub_input_filter_, sub_indices_filter_);
      sync_input_indices_a_->registerCallback(
        std::bind(
          &EuclideanClusterExtraction::
          input_indices_callback, this, std::placeholders::_1, std::placeholders::_2));
    } else {
      sync_input_indices_e_ =
        std::make_shared<message_filters::Synchronizer<
            message_filters::sync_policies::ExactTime<PointCloud, PointIndices>>>(max_queue_size_);
      sync_input_indices_e_->connectInput(sub_input_filter_, sub_indices_filter_);
      sync_input_indices_e_->registerCallback(
        std::bind(
          &EuclideanClusterExtraction::
          input_indices_callback, this, std::placeholders::_1, std::placeholders::_2));
    }
  } else {
    // Subscribe in an old fashion to input only (no filters)
    sub_input_ = this->create_subscription<PointCloud>(
      "input", rclcpp::SensorDataQoS(),
      std::bind(&EuclideanClusterExtraction::input_indices_callback, this, 
                std::placeholders::_1, PointIndicesConstPtr()));
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::EuclideanClusterExtraction::unsubscribe()
{
  if (use_indices_) {
    sub_input_filter_.unsubscribe();
    sub_indices_filter_.unsubscribe();
  } else {
    sub_input_.reset();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
rcl_interfaces::msg::SetParametersResult
pcl_ros::EuclideanClusterExtraction::config_callback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & param : parameters) {
    if (param.get_name() == "cluster_tolerance") {
      double cluster_tolerance = param.as_double();
      if (impl_.getClusterTolerance() != cluster_tolerance) {
        impl_.setClusterTolerance(cluster_tolerance);
        RCLCPP_DEBUG(
          this->get_logger(),
          "[%s::config_callback] Setting new clustering tolerance to: %f.",
          this->get_name(), cluster_tolerance);
      }
    } else if (param.get_name() == "cluster_min_size") {
      int cluster_min_size = param.as_int();
      if (impl_.getMinClusterSize() != cluster_min_size) {
        impl_.setMinClusterSize(cluster_min_size);
        RCLCPP_DEBUG(
          this->get_logger(),
          "[%s::config_callback] Setting the minimum cluster size to: %d.",
          this->get_name(), cluster_min_size);
      }
    } else if (param.get_name() == "cluster_max_size") {
      int cluster_max_size = param.as_int();
      if (impl_.getMaxClusterSize() != cluster_max_size) {
        impl_.setMaxClusterSize(cluster_max_size);
        RCLCPP_DEBUG(
          this->get_logger(),
          "[%s::config_callback] Setting the maximum cluster size to: %d.",
          this->get_name(), cluster_max_size);
      }
    } else if (param.get_name() == "max_clusters") {
      int max_clusters = param.as_int();
      if (max_clusters_ != max_clusters) {
        max_clusters_ = max_clusters;
        RCLCPP_DEBUG(
          this->get_logger(),
          "[%s::config_callback] Setting the maximum number of clusters to extract to: %d.",
          this->get_name(), max_clusters);
      }
    }
  }

  return result;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::EuclideanClusterExtraction::input_indices_callback(
  const PointCloudConstPtr & cloud, const PointIndicesConstPtr & indices)
{
  // No subscribers, no work
  if (pub_output_->get_subscription_count() <= 0) {
    return;
  }

  // If cloud is given, check if it's valid
  if (!isValid(cloud)) {
    RCLCPP_ERROR(this->get_logger(), "[%s::input_indices_callback] Invalid input!", this->get_name());
    return;
  }
  // If indices are given, check if they are valid
  if (indices && !isValid(indices)) {
    RCLCPP_ERROR(this->get_logger(), "[%s::input_indices_callback] Invalid indices!", this->get_name());
    return;
  }

  /// DEBUG
  if (indices) {
    std_msgs::msg::Header cloud_header = fromPCL(cloud->header);
    std_msgs::msg::Header indices_header = indices->header;
    RCLCPP_DEBUG(
      this->get_logger(),
      "[%s::input_indices_callback]\n"
      "                                 - PointCloud with %d data points (%s), stamp %f, and "
      "frame %s on topic %s received.\n"
      "                                 - PointIndices with %zu values, stamp %f, and "
      "frame %s on topic %s received.",
      this->get_name(),
      cloud->width * cloud->height, pcl::getFieldsList(*cloud).c_str(),
      rclcpp::Time(cloud_header.stamp).seconds(), cloud_header.frame_id.c_str(), "input",
      indices->indices.size(), rclcpp::Time(indices_header.stamp).seconds(),
      indices_header.frame_id.c_str(), "indices");
  } else {
    RCLCPP_DEBUG(
      this->get_logger(),
      "[%s::input_callback] PointCloud with %d data points, stamp %f, and frame %s on "
      "topic %s received.",
      this->get_name(), cloud->width * cloud->height, 
      rclcpp::Time(fromPCL(cloud->header).stamp).seconds(), 
      cloud->header.frame_id.c_str(), "input");
  }
  ///

  IndicesPtr indices_ptr;
  if (indices) {
    indices_ptr.reset(new std::vector<int>(indices->indices));
  }

  impl_.setInputCloud(pcl_ptr(cloud));
  impl_.setIndices(indices_ptr);

  std::vector<pcl::PointIndices> clusters;
  impl_.extract(clusters);

  if (publish_indices_) {
    for (size_t i = 0; i < clusters.size(); ++i) {
      if (static_cast<int>(i) >= max_clusters_) {
        break;
      }
      // TODO(xxx): HACK!!! We need to change the PointCloud2 message to add for an incremental
      // sequence ID number.
      pcl_msgs::msg::PointIndices ros_pi;
      moveFromPCL(clusters[i], ros_pi);
      auto stamp = rclcpp::Time(ros_pi.header.stamp) + rclcpp::Duration::from_nanoseconds(i * 1000000);
      ros_pi.header.stamp = stamp;
      pub_output_->publish(ros_pi);
    }

    RCLCPP_DEBUG(
      this->get_logger(),
      "[segmentAndPublish] Published %zu clusters (PointIndices) on topic %s",
      clusters.size(), "output");
  } else {
    for (size_t i = 0; i < clusters.size(); ++i) {
      if (static_cast<int>(i) >= max_clusters_) {
        break;
      }
      PointCloud output;
      copyPointCloud(*cloud, clusters[i].indices, output);

      // TODO(xxx): HACK!!! We need to change the PointCloud2 message to add for an incremental
      // sequence ID number.
      std_msgs::msg::Header header = fromPCL(output.header);
      auto stamp = rclcpp::Time(header.stamp) + rclcpp::Duration::from_nanoseconds(i * 1000000);
      header.stamp = stamp;
      toPCL(header, output.header);
      // Publish a shared ptr const data
      pub_output_->publish(output);
      RCLCPP_DEBUG(
        this->get_logger(),
        "[segmentAndPublish] Published cluster %zu (with %zu values and stamp %f) on topic %s",
        i, clusters[i].indices.size(), rclcpp::Time(header.stamp).seconds(), "output");
    }
  }
}

typedef pcl_ros::EuclideanClusterExtraction EuclideanClusterExtraction;
RCLCPP_COMPONENTS_REGISTER_NODE(EuclideanClusterExtraction)