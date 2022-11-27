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
 * $Id: filter.cpp 35876 2011-02-09 01:04:36Z rusu $
 *
 */

#include <pcl/common/io.h>
#include "pcl_ros/transforms.hpp"
#include "pcl_ros/filters/filter.hpp"
/*//#include <pcl/filters/pixel_grid.h>
//#include <pcl/filters/filter_dimension.h>
*/

/*//typedef pcl::PixelGrid PixelGrid;
//typedef pcl::FilterDimension FilterDimension;
*/

// Include the implementations instead of compiling them separately to speed up compile time
// #include "extract_indices.cpp"
// #include "passthrough.cpp"
// #include "project_inliers.cpp"
// #include "radius_outlier_removal.cpp"
// #include "statistical_outlier_removal.cpp"
// #include "voxel_grid.cpp"

/*//PLUGINLIB_EXPORT_CLASS(PixelGrid,nodelet::Nodelet);
//PLUGINLIB_EXPORT_CLASS(FilterDimension,nodelet::Nodelet);
*/

///////////////////////////////////////////////////////////////////////////////////////////////////
pcl_ros::Filter::Filter(std::string node_name, const rclcpp::NodeOptions & options)
: PCLNode(node_name, options)
{
  pub_output_ = advertise<PointCloud2>("output", cloudQoS());

  RCLCPP_DEBUG(this->get_logger(), "Node successfully created.");
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::Filter::computePublish(
  const PointCloud2::ConstSharedPtr & input,
  const IndicesPtr & indices)
{
  PointCloud2 output;
  // Call the virtual method in the child
  filter(input, indices, output);

  PointCloud2::SharedPtr cloud_tf(new PointCloud2(output));     // set the output by default
  // Check whether the user has given a different output TF frame
  if (!tf_output_frame_.empty() && output.header.frame_id != tf_output_frame_) {
    RCLCPP_DEBUG(
      this->get_logger(), "Transforming output dataset from %s to %s.",
      output.header.frame_id.c_str(), tf_output_frame_.c_str());
    // Convert the cloud into the different frame
    PointCloud2 cloud_transformed;
    if (!pcl_ros::transformPointCloud(tf_output_frame_, output, cloud_transformed, tf_buffer_)) {
      RCLCPP_ERROR(
        this->get_logger(), "Error converting output dataset from %s to %s.",
        output.header.frame_id.c_str(), tf_output_frame_.c_str());
      return;
    }
    cloud_tf.reset(new PointCloud2(cloud_transformed));
  }
  if (tf_output_frame_.empty() && output.header.frame_id != tf_input_orig_frame_) {
    // no tf_output_frame given, transform the dataset to its original frame
    RCLCPP_DEBUG(
      this->get_logger(), "Transforming output dataset from %s back to %s.",
      output.header.frame_id.c_str(), tf_input_orig_frame_.c_str());
    // Convert the cloud into the different frame
    PointCloud2 cloud_transformed;
    if (!pcl_ros::transformPointCloud(
        tf_input_orig_frame_, output, cloud_transformed,
        tf_buffer_))
    {
      RCLCPP_ERROR(
        this->get_logger(), "Error converting output dataset from %s back to %s.",
        output.header.frame_id.c_str(), tf_input_orig_frame_.c_str());
      return;
    }
    cloud_tf.reset(new PointCloud2(cloud_transformed));
  }

  // Copy timestamp to keep it
  cloud_tf->header.stamp = input->header.stamp;

  // Publish a shared ptr
  pub_output_->publish(*cloud_tf);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::Filter::subscribe()
{
  RCLCPP_DEBUG(get_logger(), "Filter::subscribe");
  // If we're supposed to look for PointIndices (indices)
  if (use_indices_) {
    // Subscribe to the input using a filter
    sub_input_filter_.subscribe(
      this->shared_from_this(), "input",
      cloudQoS().get_rmw_qos_profile());
    sub_indices_filter_.subscribe(
      this->shared_from_this(), "indices",
      indicesQoS().get_rmw_qos_profile());

    if (approximate_sync_) {
      sync_input_indices_a_ = std::make_shared<
        message_filters::Synchronizer<
          sync_policies::ApproximateTime<
            PointCloud2, pcl_msgs::msg::PointIndices>>>(max_queue_size_);
      sync_input_indices_a_->connectInput(sub_input_filter_, sub_indices_filter_);
      auto callback = std::bind(
        &Filter::input_indices_callback, this, std::placeholders::_1,
        std::placeholders::_2);
      sync_input_indices_a_->registerCallback(callback);
    } else {
      sync_input_indices_e_ = std::make_shared<
        message_filters::Synchronizer<
          sync_policies::ExactTime<
            PointCloud2, pcl_msgs::msg::PointIndices>>>(max_queue_size_);
      sync_input_indices_e_->connectInput(sub_input_filter_, sub_indices_filter_);
      auto callback = std::bind(
        &Filter::input_indices_callback, this, std::placeholders::_1,
        std::placeholders::_2);
      sync_input_indices_e_->registerCallback(callback);
    }
  } else {
    // Workaround ros2/rclcpp#766
    std::function<void(PointCloud2::ConstSharedPtr)> callback =
      std::bind(&Filter::input_indices_callback, this, std::placeholders::_1, nullptr);

    // Subscribe in an old fashion to input only (no filters)
    sub_input_ = this->create_subscription<PointCloud2>("input", cloudQoS(), callback);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::Filter::unsubscribe()
{
  if (use_indices_) {
    sub_input_filter_.unsubscribe();
    sub_indices_filter_.unsubscribe();
  } else {
    pub_output_.reset();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::Filter::input_indices_callback(
  const PointCloud2::ConstSharedPtr cloud,
  const pcl_msgs::msg::PointIndices::ConstSharedPtr indices)
{
  // If cloud is given, check if it's valid
  if (!isValid(cloud)) {
    RCLCPP_ERROR(this->get_logger(), "Invalid input!");
    return;
  }
  // If indices are given, check if they are valid
  if (indices && !isValid(indices)) {
    RCLCPP_ERROR(this->get_logger(), "Invalid indices!");
    return;
  }

  /// DEBUG
  if (indices) {
    RCLCPP_DEBUG(
      this->get_logger(), "[input_indices_callback]\n"
      "  - PointCloud with %d data points (%s), stamp %d.%09d, and frame %s on topic %s received.\n"
      "  - PointIndices with %zu values, stamp %d.%09d, and frame %s on topic %s received.",
      cloud->width * cloud->height, pcl::getFieldsList(*cloud).c_str(),
      cloud->header.stamp.sec, cloud->header.stamp.nanosec, cloud->header.frame_id.c_str(), "input",
      indices->indices.size(), indices->header.stamp.sec, indices->header.stamp.nanosec,
      indices->header.frame_id.c_str(), "indices");
  } else {
    RCLCPP_DEBUG(
      this->get_logger(), "PointCloud with %d data points and frame %s on topic %s received.",
      cloud->width * cloud->height, cloud->header.frame_id.c_str(), "input");
  }
  ///

  // Check whether the user has given a different input TF frame
  tf_input_orig_frame_ = cloud->header.frame_id;
  PointCloud2::ConstSharedPtr cloud_tf;
  if (!tf_input_frame_.empty() && cloud->header.frame_id != tf_input_frame_) {
    RCLCPP_DEBUG(
      this->get_logger(), "Transforming input dataset from %s to %s.",
      cloud->header.frame_id.c_str(), tf_input_frame_.c_str());
    // Save the original frame ID
    // Convert the cloud into the different frame
    PointCloud2 cloud_transformed;
    if (!pcl_ros::transformPointCloud(tf_input_frame_, *cloud, cloud_transformed, tf_buffer_)) {
      RCLCPP_ERROR(
        this->get_logger(), "Error converting input dataset from %s to %s.",
        cloud->header.frame_id.c_str(), tf_input_frame_.c_str());
      return;
    }
    cloud_tf = std::make_shared<PointCloud2>(cloud_transformed);
  } else {
    cloud_tf = cloud;
  }

  // Need setInputCloud () here because we have to extract x/y/z
  IndicesPtr vindices;
  if (indices) {
    vindices.reset(new std::vector<int>(indices->indices));
  }

  computePublish(cloud_tf, vindices);
}

//////////////////////////////////////////////////////////////////////////////////////////////
std::vector<std::string>
pcl_ros::Filter::add_common_parameters()
{
  rcl_interfaces::msg::ParameterDescriptor ffn_desc;
  ffn_desc.name = "filter_field_name";
  ffn_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  ffn_desc.description = "The field name used for filtering";
  declare_parameter(ffn_desc.name, rclcpp::ParameterValue("z"), ffn_desc);

  rcl_interfaces::msg::ParameterDescriptor flmin_desc;
  flmin_desc.name = "filter_limit_min";
  flmin_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  flmin_desc.description = "The minimum allowed field value a point will be considered from";
  rcl_interfaces::msg::FloatingPointRange flmin_range;
  flmin_range.from_value = -100000.0;
  flmin_range.to_value = 100000.0;
  flmin_desc.floating_point_range.push_back(flmin_range);
  declare_parameter(flmin_desc.name, rclcpp::ParameterValue(0.0), flmin_desc);

  rcl_interfaces::msg::ParameterDescriptor flmax_desc;
  flmax_desc.name = "filter_limit_max";
  flmax_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  flmax_desc.description = "The maximum allowed field value a point will be considered from";
  rcl_interfaces::msg::FloatingPointRange flmax_range;
  flmax_range.from_value = -100000.0;
  flmax_range.to_value = 100000.0;
  flmax_desc.floating_point_range.push_back(flmax_range);
  declare_parameter(flmax_desc.name, rclcpp::ParameterValue(1.0), flmax_desc);

  rcl_interfaces::msg::ParameterDescriptor flneg_desc;
  flneg_desc.name = "filter_limit_negative";
  flneg_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  flneg_desc.description =
    "Set to true if we want to return the data outside [filter_limit_min; filter_limit_max].";
  declare_parameter(flneg_desc.name, rclcpp::ParameterValue(false), flneg_desc);

  rcl_interfaces::msg::ParameterDescriptor keep_organized_desc;
  keep_organized_desc.name = "keep_organized";
  keep_organized_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  keep_organized_desc.description =
    "Set whether the filtered points should be kept and set to NaN, "
    "or removed from the PointCloud, thus potentially breaking its organized structure.";
  declare_parameter(keep_organized_desc.name, rclcpp::ParameterValue(false), keep_organized_desc);

  rcl_interfaces::msg::ParameterDescriptor input_frame_desc;
  input_frame_desc.name = "input_frame";
  input_frame_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  input_frame_desc.description =
    "The input TF frame the data should be transformed into before processing, "
    "if input.header.frame_id is different.";
  declare_parameter(input_frame_desc.name, rclcpp::ParameterValue(""), input_frame_desc);

  rcl_interfaces::msg::ParameterDescriptor output_frame_desc;
  output_frame_desc.name = "output_frame";
  output_frame_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  output_frame_desc.description =
    "The output TF frame the data should be transformed into after processing, "
    "if input.header.frame_id is different.";
  declare_parameter(output_frame_desc.name, rclcpp::ParameterValue(""), output_frame_desc);
}
