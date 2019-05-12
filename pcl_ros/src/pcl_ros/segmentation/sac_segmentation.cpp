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
 * $Id: sac_segmentation.hpp 33195 2010-10-10 14:12:19Z marton $
 *
 */

#include "pcl_ros/segmentation/sac_segmentation.h"
#include <pcl/io/io.h>

#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/ptr_helper.h"

using pcl_conversions::fromPCL;

//////////////////////////////////////////////////////////////////////////////////////////////
pcl_ros::SACSegmentation::SACSegmentation (const rclcpp::NodeOptions& options) : PCLNode("SACSegmentationNode", options), min_inliers_(0)
{
  // Create publishers for the output topics
  auto custom_qos = rmw_qos_profile_system_default;
  custom_qos.depth = max_queue_size_;
    
  pub_indices_ = this->create_publisher<PointIndices> ("inliers");
  pub_model_   = this->create_publisher<ModelCoefficients> ("model");

  // ---[ Mandatory parameters
  int model_type;
  if (!this->get_parameter ("model_type", model_type))
  {
    RCLCPP_ERROR (this->get_logger(), "[onConstructor] Need a 'model_type' parameter to be set before continuing!");
    return;
  }
  double threshold; // unused - set via dynamic reconfigure in the callback
  if (!this->get_parameter ("distance_threshold", threshold))
  {
    RCLCPP_ERROR (this->get_logger(), "[onConstructor] Need a 'distance_threshold' parameter to be set before continuing!");
    return;
  }

  // ---[ Optional parameters
  int method_type = 0;
  this->get_parameter ("method_type", method_type);

  std::vector<double> axis_param;
  // XmlRpcValue datatype not supported as parameter in ROS2 https://design.ros2.org/articles/ros_parameters.html
  this->get_parameter ("axis", axis_param);
  Eigen::Vector3f axis = Eigen::Vector3f::Zero ();

  if (axis_param.size () != 3)
  {
    RCLCPP_ERROR (this->get_logger(), "[%s::onConstructor] Parameter 'axis' given but with a different number of values (%d) than required (3)!", this->get_name (), sizeof (axis_param));
    return;
  }

  for (int i = 0; i < 3; ++i)
  {
    if (typeid (axis_param [i]) != typeid (double))
    {
      RCLCPP_ERROR (this->get_logger(), "[%s::onConstructor] Need floating point values for 'axis' parameter.", this->get_name ());
      return;
    }
    double value = axis_param[i]; axis[i] = value;
  }



  // Initialize the random number generator
  srand (time (0));

  RCLCPP_DEBUG (this->get_logger(), "[%s::onConstructor] Node successfully created with the following parameters:\n"
                 " - model_type               : %d\n"
                 " - method_type              : %d\n"
                 " - model_threshold          : %f\n"
                 " - axis                     : [%f, %f, %f]\n",
                 this->get_name (), model_type, method_type, threshold,
                 axis[0], axis[1], axis[2]);

  // Set given parameters here
  impl_.setModelType (model_type);
  impl_.setMethodType (method_type);
  impl_.setAxis (axis);

}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::SACSegmentation::subscribe ()
{
  // If we're supposed to look for PointIndices (indices)
  if (use_indices_)
  {
    // Subscribe to the input using a filter
    sub_input_filter_.subscribe (this->shared_from_this (), "input");
    sub_indices_filter_.subscribe (this->shared_from_this (), "indices");

    // when "use_indices" is set to true, and "latched_indices" is set to true,
    // we'll subscribe and get a separate callback for PointIndices that will 
    // save the indices internally, and a PointCloud + PointIndices callback 
    // will take care of meshing the new PointClouds with the old saved indices. 
    if (latched_indices_)
    {
      // Subscribe to a callback that saves the indices
      sub_indices_filter_.registerCallback (std::bind (&SACSegmentation::indices_callback, this, std::placeholders::_1));
      // Subscribe to a callback that sets the header of the saved indices to the cloud header
      sub_input_filter_.registerCallback (std::bind (&SACSegmentation::input_callback, this, std::placeholders::_1));

      // Synchronize the two topics. No need for an approximate synchronizer here, as we'll
      // match the timestamps exactly
      sync_input_indices_e_ = std::make_shared <message_filters::Synchronizer<sync_policies::ExactTime<PointCloud, PointIndices> > > (max_queue_size_);
      sync_input_indices_e_->connectInput (sub_input_filter_, nf_pi_);
      sync_input_indices_e_->registerCallback (std::bind (&SACSegmentation::input_indices_callback, this, std::placeholders::_1, std::placeholders::_2));
    }
    // "latched_indices" not set, proceed with regular <input,indices> pairs
    else
    {
      if (approximate_sync_)
      {
        sync_input_indices_a_ = std::make_shared <message_filters::Synchronizer<sync_policies::ApproximateTime<PointCloud, PointIndices> > > (max_queue_size_);
        sync_input_indices_a_->connectInput (sub_input_filter_, sub_indices_filter_);
        sync_input_indices_a_->registerCallback (std::bind (&SACSegmentation::input_indices_callback, this, std::placeholders::_1, std::placeholders::_2));
      }
      else
      {
        sync_input_indices_e_ = std::make_shared <message_filters::Synchronizer<sync_policies::ExactTime<PointCloud, PointIndices> > > (max_queue_size_);
        sync_input_indices_e_->connectInput (sub_input_filter_, sub_indices_filter_);
        sync_input_indices_e_->registerCallback (std::bind (&SACSegmentation::input_indices_callback, this, std::placeholders::_1, std::placeholders::_2));
      }
    }
  }
  else
    // Subscribe in an old fashion to input only (no filters)
    // Type masquerading not yet supported
    // sub_input_ = this->create_subscription<PointCloud> ("input",  std::bind (&SACSegmentation::input_indices_callback, this, std::placeholders::_1, PointIndicesPtr ()));
    std::cout << "Type masquerading not supported" << std::endl;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::SACSegmentation::unsubscribe ()
{
  if (use_indices_)
  {
    sub_input_filter_.unsubscribe ();
    sub_indices_filter_.unsubscribe ();
  }
  else
    std::cout << "shutdown" << std::endl;
    //this->shutdown ();
}


//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::SACSegmentation::input_indices_callback (const PointCloudPtr cloud,
                                                  const PointIndicesPtr indices)
{
  std::lock_guard<std::mutex> lock (mutex_);

  pcl_msgs::msg::PointIndices inliers;
  pcl_msgs::msg::ModelCoefficients model;
  // Enforce that the TF frame and the timestamp are copied
  inliers.header = model.header = fromPCL(cloud->header);

  // If cloud is given, check if it's valid
  if (!isValid (cloud)) 
  {
    RCLCPP_ERROR (this->get_logger(), "[%s::input_indices_callback] Invalid input!", this->get_name ());
    pub_indices_->publish (inliers);
    pub_model_->publish (model);
    return;
  }
  // If indices are given, check if they are valid
  if (indices && !isValid (indices))
  {
    RCLCPP_ERROR (this->get_logger(), "[%s::input_indices_callback] Invalid indices!", this->get_name ());
    pub_indices_->publish (inliers);
    pub_model_->publish (model);
    return;
  }

  /// DEBUG
  if (indices && !indices->header.frame_id.empty ())
    RCLCPP_DEBUG (this->get_logger(), "[%s::input_indices_callback]\n"
                   "                                 - PointCloud with %d data points (%s), stamp %f, and frame %s on topic %s received.\n"
                   "                                 - PointIndices with %zu values, stamp %f, and frame %s on topic %s received.",
                   this->get_name (),
                   cloud->width * cloud->height, pcl::getFieldsList (*cloud).c_str (), fromPCL(cloud->header).stamp.sec, cloud->header.frame_id.c_str (), "input",
                   indices->indices.size (), indices->header.stamp.sec, indices->header.frame_id.c_str (), "indices");
  else
    RCLCPP_DEBUG (this->get_logger(), "[%s::input_indices_callback] PointCloud with %d data points, stamp %f, and frame %s on topic %s received.",
                   this->get_name (), cloud->width * cloud->height, fromPCL(cloud->header).stamp.sec, cloud->header.frame_id.c_str (), "input");
  ///

  // Check whether the user has given a different input TF frame
  tf_input_orig_frame_ = cloud->header.frame_id;
  PointCloudConstPtr cloud_tf;
/*  if (!tf_input_frame_.empty () && cloud->header.frame_id != tf_input_frame_)
  {
    RCLCPP_DEBUG ("[input_callback] Transforming input dataset from %s to %s.", cloud->header.frame_id.c_str (), tf_input_frame_.c_str ());
    // Save the original frame ID
    // Convert the cloud into the different frame
    PointCloud cloud_transformed;
    if (!pcl::transformPointCloud (tf_input_frame_, cloud->header.stamp, *cloud, cloud_transformed, tf_listener_))
      return;
    cloud_tf.reset (new PointCloud (cloud_transformed));
  }
  else*/
    cloud_tf = cloud;

  IndicesPtr indices_ptr;
  if (indices && !indices->header.frame_id.empty ())
    indices_ptr.reset (new std::vector<int> (indices->indices));

  impl_.setInputCloud (cloud_tf);
  impl_.setIndices (to_boost_ptr(indices_ptr));

  // Final check if the data is empty (remember that indices are set to the size of the data -- if indices* = NULL)
  if (!cloud->points.empty ()) {
    pcl::PointIndices pcl_inliers;
    pcl::ModelCoefficients pcl_model;
    pcl_conversions::moveToPCL(inliers, pcl_inliers);
    pcl_conversions::moveToPCL(model, pcl_model);
    impl_.segment (pcl_inliers, pcl_model);
    pcl_conversions::moveFromPCL(pcl_inliers, inliers);
    pcl_conversions::moveFromPCL(pcl_model, model);
  }

  // Probably need to transform the model of the plane here

  // Check if we have enough inliers, clear inliers + model if not
  if ((int)inliers.indices.size () <= min_inliers_)
  {
    inliers.indices.clear (); 
    model.values.clear ();
  }

  // Publish
  pub_indices_->publish (*std::make_shared<const PointIndices> (inliers));
  pub_model_->publish (*std::make_shared<const ModelCoefficients> (model));
  RCLCPP_DEBUG (this->get_logger (), "[%s::input_indices_callback] Published PointIndices with %zu values on topic %s, and ModelCoefficients with %zu values on topic %s",
                 this->get_name (), inliers.indices.size (), "inliers",
                 model.values.size (), "model");

  if (inliers.indices.empty ())
    RCLCPP_WARN (this->get_logger(), "[%s::input_indices_callback] No inliers found!", this->get_name ());
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl_ros::SACSegmentationFromNormals::SACSegmentationFromNormals (const rclcpp::NodeOptions& options) : SACSegmentation(options)
{
  // Create publishers for the output topics
  pub_indices_ = this->create_publisher<PointIndices> ("inliers", max_queue_size_);
  pub_model_   = this->create_publisher<ModelCoefficients> ("model", max_queue_size_);
  
  // ---[ Mandatory parameters
  int model_type;
  if (!this->get_parameter ("model_type", model_type))
  {
    RCLCPP_ERROR (this->get_logger (), "[%s::onConstructor] Need a 'model_type' parameter to be set before continuing!", this->get_name ());
    return;
  }
  double threshold; // unused - set via dynamic reconfigure in the callback
  if (!this->get_parameter ("distance_threshold", threshold))
  {
    RCLCPP_ERROR (this->get_logger (), "[%s::onConstructor] Need a 'distance_threshold' parameter to be set before continuing!", this->get_name ());
    return;
  }

  // ---[ Optional parameters
  int method_type = 0;
  this->get_parameter ("method_type", method_type);

  std::vector<double> axis_param;
  this->get_parameter ("axis", axis_param);
  Eigen::Vector3f axis = Eigen::Vector3f::Zero ();
  
  if (axis_param.size () != 3)
  {
    RCLCPP_ERROR (this->get_logger (), "[%s::onConstruct] Parameter 'axis' given but with a different number of values (%d) than required (3)!", this->get_name (), sizeof (axis_param));
    return;
  }
  for (int i = 0; i < 3; ++i)
  {
    if (typeid (axis_param[i]) != typeid (double))
    {
      RCLCPP_ERROR (this->get_logger (), "[%s::onConstructor] Need floating point values for 'axis' parameter.", this->get_name ());
      return;
    }
    double value = axis_param[i]; axis[i] = value;
  }

  // Initialize the random number generator
  srand (time (0));

  RCLCPP_DEBUG (this->get_logger (), "[%s::onConstructor] Node successfully created with the following parameters:\n"
                 " - model_type               : %d\n"
                 " - method_type              : %d\n"
                 " - model_threshold          : %f\n"
                 " - axis                     : [%f, %f, %f]\n",
                 this->get_name (), model_type, method_type, threshold,
                 axis[0], axis[1], axis[2]);

  // Set given parameters here
  impl_.setModelType (model_type);
  impl_.setMethodType (method_type);
  impl_.setAxis (axis);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::SACSegmentationFromNormals::subscribe ()
{
  // Subscribe to the input and normals using filters
  sub_input_filter_.subscribe (this->shared_from_this (), "input");
  sub_normals_filter_.subscribe (this->shared_from_this (), "normals");

  // Subscribe to an axis direction along which the model search is to be constrained (the first 3 model coefficients will be checked)
  // Type masquerading not yet supported
  sub_axis_ = this->create_subscription<pcl_msgs::msg::ModelCoefficients> ("axis", std::bind(&SACSegmentationFromNormals::axis_callback, this, std::placeholders::_1));

  if (approximate_sync_)
    sync_input_normals_indices_a_ = std::make_shared <message_filters::Synchronizer<sync_policies::ApproximateTime<PointCloud, PointCloudN, PointIndices> > > (max_queue_size_);
  else
    sync_input_normals_indices_e_ = std::make_shared <message_filters::Synchronizer<sync_policies::ExactTime<PointCloud, PointCloudN, PointIndices> > > (max_queue_size_);

  // If we're supposed to look for PointIndices (indices)
  if (use_indices_)
  {
    // Subscribe to the input using a filter
    sub_indices_filter_.subscribe (this->shared_from_this(), "indices");

    if (approximate_sync_)
      sync_input_normals_indices_a_->connectInput (sub_input_filter_, sub_normals_filter_, sub_indices_filter_);
    else
      sync_input_normals_indices_e_->connectInput (sub_input_filter_, sub_normals_filter_, sub_indices_filter_);
  }
  else
  {
    // Create a different callback for copying over the timestamp to fake indices
    sub_input_filter_.registerCallback (std::bind (&SACSegmentationFromNormals::input_callback, this, std::placeholders::_1));

    if (approximate_sync_)
      sync_input_normals_indices_a_->connectInput (sub_input_filter_, sub_normals_filter_, nf_);
    else
      sync_input_normals_indices_e_->connectInput (sub_input_filter_, sub_normals_filter_, nf_);
  }

  if (approximate_sync_)
    sync_input_normals_indices_a_->registerCallback (std::bind (&SACSegmentationFromNormals::input_normals_indices_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  else
    sync_input_normals_indices_e_->registerCallback (std::bind (&SACSegmentationFromNormals::input_normals_indices_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::SACSegmentationFromNormals::unsubscribe ()
{
  sub_input_filter_.unsubscribe ();
  sub_normals_filter_.unsubscribe ();
  
  std::cout << "shutdown" << std::endl;
  //sub_axis_.shutdown ();

  if (use_indices_)
    sub_indices_filter_.unsubscribe ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::SACSegmentationFromNormals::axis_callback (const pcl_msgs::msg::ModelCoefficients::SharedPtr model)
{
  std::lock_guard<std::mutex> lock (mutex_);

  if (model->values.size () < 3)
  {
    RCLCPP_ERROR (this->get_logger (), "[%s::axis_callback] Invalid axis direction / model coefficients with %zu values sent on %s!", this->get_name (), model->values.size (), "axis");
    return;
  }
  RCLCPP_DEBUG (this->get_logger (), "[%s::axis_callback] Received axis direction: %f %f %f", this->get_name (), model->values[0], model->values[1], model->values[2]);

  Eigen::Vector3f axis (model->values[0], model->values[1], model->values[2]);
  impl_.setAxis (axis);
}


//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::SACSegmentationFromNormals::input_normals_indices_callback (
      const PointCloudConstPtr &cloud, 
      const PointCloudNConstPtr &cloud_normals,
      const PointIndicesConstPtr &indices
      )
{
  std::lock_guard<std::mutex> lock (mutex_);

  PointIndices inliers;
  ModelCoefficients model;
  // Enforce that the TF frame and the timestamp are copied
  inliers.header = model.header = fromPCL(cloud->header);

  if (impl_.getModelType () < 0)
  {
    RCLCPP_ERROR (this->get_logger (), "[%s::input_normals_indices_callback] Model type not set!", this->get_name ());
    pub_indices_->publish (*std::make_shared<const PointIndices> (inliers));
    pub_model_->publish (*std::make_shared<const ModelCoefficients> (model));
    return;
  }

  if (!isValid (cloud))// || !isValid (cloud_normals, "normals")) 
  {
    RCLCPP_ERROR (this->get_logger (), "[%s::input_normals_indices_callback] Invalid input!", this->get_name ());
    pub_indices_->publish (*std::make_shared<const PointIndices> (inliers));
    pub_model_->publish (*std::make_shared<const ModelCoefficients> (model));
    return;
  }
  // If indices are given, check if they are valid
  if (indices && !isValid (indices))
  {
    RCLCPP_ERROR (this->get_logger (), "[%s::input_normals_indices_callback] Invalid indices!", this->get_name ());
    pub_indices_->publish (*std::make_shared<const PointIndices> (inliers));
    pub_model_->publish (*std::make_shared<const ModelCoefficients> (model));
    return;
  }

  /// DEBUG
  if (indices && !indices->header.frame_id.empty ())
    RCLCPP_DEBUG (this->get_logger (), "[%s::input_normals_indices_callback]\n"
                   "                                 - PointCloud with %d data points (%s), stamp %f, and frame %s on topic %s received.\n"
                   "                                 - PointCloud with %d data points (%s), stamp %f, and frame %s on topic %s received.\n"
                   "                                 - PointIndices with %zu values, stamp %f, and frame %s on topic %s received.",
                   this->get_name (),
                   cloud->width * cloud->height, pcl::getFieldsList (*cloud).c_str (), fromPCL(cloud->header).stamp.sec, cloud->header.frame_id.c_str (), "input",
                   cloud_normals->width * cloud_normals->height, pcl::getFieldsList (*cloud_normals).c_str (), fromPCL(cloud_normals->header).stamp.sec, cloud_normals->header.frame_id.c_str (), "normals",
                   indices->indices.size (), indices->header.stamp.sec, indices->header.frame_id.c_str (), "indices");
  else
    RCLCPP_DEBUG (this->get_logger (), "[%s::input_normals_indices_callback]\n"
                   "                                 - PointCloud with %d data points (%s), stamp %f, and frame %s on topic %s received.\n"
                   "                                 - PointCloud with %d data points (%s), stamp %f, and frame %s on topic %s received.",
                   this->get_name (),
                   cloud->width * cloud->height, pcl::getFieldsList (*cloud).c_str (), fromPCL(cloud->header).stamp.sec, cloud->header.frame_id.c_str (), "input",
                   cloud_normals->width * cloud_normals->height, pcl::getFieldsList (*cloud_normals).c_str (), fromPCL(cloud_normals->header).stamp.sec, cloud_normals->header.frame_id.c_str (), "normals");
  ///


  // Extra checks for safety
  int cloud_nr_points         = cloud->width * cloud->height;
  int cloud_normals_nr_points = cloud_normals->width * cloud_normals->height;
  if (cloud_nr_points != cloud_normals_nr_points)
  {
    RCLCPP_ERROR (this->get_logger (), "[%s::input_normals_indices_callback] Number of points in the input dataset (%d) differs from the number of points in the normals (%d)!", this->get_name (), cloud_nr_points, cloud_normals_nr_points);
    pub_indices_->publish (*std::make_shared<const PointIndices> (inliers));
    pub_model_->publish (*std::make_shared<const ModelCoefficients> (model));
    return;
  }

  impl_.setInputCloud (cloud);
  impl_.setInputNormals (cloud_normals);

  IndicesPtr indices_ptr;
  if (indices && !indices->header.frame_id.empty ())
    indices_ptr.reset (new std::vector<int> (indices->indices));

  impl_.setIndices (to_boost_ptr (indices_ptr));

  // Final check if the data is empty (remember that indices are set to the size of the data -- if indices* = NULL)
  if (!cloud->points.empty ()) {
    pcl::PointIndices pcl_inliers;
    pcl::ModelCoefficients pcl_model;
    pcl_conversions::moveToPCL(inliers, pcl_inliers);
    pcl_conversions::moveToPCL(model, pcl_model);
    impl_.segment (pcl_inliers, pcl_model);
    pcl_conversions::moveFromPCL(pcl_inliers, inliers);
    pcl_conversions::moveFromPCL(pcl_model, model);
  }

  // Check if we have enough inliers, clear inliers + model if not
  if ((int)inliers.indices.size () <= min_inliers_)
  {
    inliers.indices.clear (); 
    model.values.clear ();
  }

  // Publish
  pub_indices_->publish (*std::make_shared<const PointIndices> (inliers));
  pub_model_->publish (*std::make_shared<const ModelCoefficients> (model));
  RCLCPP_DEBUG (this->get_logger (), "[%s::input_normals_callback] Published PointIndices with %zu values on topic %s, and ModelCoefficients with %zu values on topic %s",
      this->get_name (), inliers.indices.size (), "inliers",
      model.values.size (), "model");
  if (inliers.indices.empty ())
    RCLCPP_WARN (this->get_logger (), "[%s::input_indices_callback] No inliers found!", this->get_name ());
}

//typedef pcl_ros::SACSegmentation SACSegmentation;
typedef pcl_ros::SACSegmentationFromNormals SACSegmentationFromNormals;
//CLASS_LOADER_REGISTER_CLASS(SACSegmentation, rclcpp::Node)
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(SACSegmentationFromNormals)
