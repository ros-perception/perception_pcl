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
 * $Id: project_inliers.cpp 35876 2011-02-09 01:04:36Z rusu $
 *
 */

//#include <pluginlib/class_list_macros.h>
#include "pcl_ros/filters/project_inliers.h"
#include <pcl/io/io.h>

//////////////////////////////////////////////////////////////////////////////////////////////
pcl_ros::ProjectInliers::ProjectInliers () : pcl_ros::Filter()
{
  // ---[ Mandatory parameters
  // The type of model to use (user given parameter).
  int model_type;
  if (!this->get_parameter ("model_type", model_type))
  {
    RCLCPP_ERROR (this->get_logger(), "[%s::onConstruct] Need a 'model_type' parameter to be set before continuing!", this->get_name ());
    return;
  }
  // ---[ Optional parameters
  // True if all data will be returned, false if only the projected inliers. Default: false.
  bool copy_all_data = false;

  // True if all fields will be returned, false if only XYZ. Default: true. 
  bool copy_all_fields = true;

  this->get_parameter ("copy_all_data", copy_all_data);
  this->get_parameter ("copy_all_fields", copy_all_fields);

  pub_output_ = this->create_publisher<PointCloud2> ("output", max_queue_size_);

  // Subscribe to the input using a filter
  sub_input_filter_->subscribe ("input", max_queue_size_);

  RCLCPP_DEBUG (this->get_logger(), "[%s::onConstruct] Nodelet successfully created with the following parameters:\n"
                 " - model_type      : %d\n"
                 " - copy_all_data   : %s\n"
                 " - copy_all_fields : %s",
                 this->get_name (),
                 model_type, (copy_all_data) ? "true" : "false", (copy_all_fields) ? "true" : "false");

  // Set given parameters here
  impl_.setModelType (model_type);
  impl_.setCopyAllFields (copy_all_fields);
  impl_.setCopyAllData (copy_all_data);

}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::ProjectInliers::subscribe ()
{
/*
  TODO : implement use_indices_
  if (use_indices_)
  {*/

  sub_indices_filter_->subscribe ("indices", max_queue_size_);

  sub_model_->subscribe ("model", max_queue_size_);

  if (approximate_sync_)
  {
    sync_input_indices_model_a_ = std::make_shared <message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<PointCloud2, PointIndices, ModelCoefficients> > > (max_queue_size_);
    sync_input_indices_model_a_->connectInput (sub_input_filter_, sub_indices_filter_, sub_model_);
    sync_input_indices_model_a_->registerCallback (bind (&ProjectInliers::input_indices_model_callback, this, _1, _2, _3));
  }
  else
  {
    sync_input_indices_model_e_ = std::make_shared <message_filters::Synchronizer<message_filters::sync_policies::ExactTime<PointCloud2, PointIndices, ModelCoefficients> > > (max_queue_size_);
    sync_input_indices_model_e_->connectInput (sub_input_filter_, sub_indices_filter_, sub_model_);
    sync_input_indices_model_e_->registerCallback (bind (&ProjectInliers::input_indices_model_callback, this, _1, _2, _3));
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::ProjectInliers::unsubscribe ()
{
/*
  TODO : implement use_indices_
  if (use_indices_)
  {*/

  sub_input_filter_.unsubscribe ();
  sub_model_.unsubscribe ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::ProjectInliers::input_indices_model_callback (const PointCloud2::ConstPtr &cloud, 
                                                       const PointIndicesConstPtr &indices,
                                                       const ModelCoefficientsConstPtr &model)
{
  if (pub_output_.count_subscribers () <= 0)
    return;

  if (!isValid (model) || !isValid (indices) || !isValid (cloud))
  {
    RCLCPP_ERROR (this->get_logger(), "[%s::input_indices_model_callback] Invalid input!", this->get_name ());
    return;
  }

  RCLCPP_DEBUG (this->get_logger(), "[%s::input_indices_model_callback]\n"
                 "                                 - PointCloud with %d data points (%s), stamp %f, and frame %s on topic %s received.\n"
                 "                                 - PointIndices with %zu values, stamp %f, and frame %s on topic %s received.\n"
                 "                                 - ModelCoefficients with %zu values, stamp %f, and frame %s on topic %s received.",
                 this->get_name (),
                 cloud->width * cloud->height, pcl::getFieldsList (*cloud).c_str (), cloud->header.stamp.sec, cloud->header.frame_id.c_str (), "input",
                 indices->indices.size (), indices->header.stamp.sec, indices->header.frame_id.c_str (), "inliers",
                 model->values.size (), model->header.stamp.sec, model->header.frame_id.c_str (), "model");

  tf_input_orig_frame_ = cloud->header.frame_id;

  IndicesPtr vindices;
  if (indices)
    vindices.reset (new std::vector<int> (indices->indices));

  model_   = model;
  computePublish (cloud, vindices);
}

typedef pcl_ros::ProjectInliers ProjectInliers;
//PLUGINLIB_EXPORT_CLASS(ProjectInliers,nodelet::Nodelet);

