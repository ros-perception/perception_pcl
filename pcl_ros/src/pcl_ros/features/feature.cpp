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
 *  COPYRIGHT OWNERff OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: feature.cpp 35422 2011-01-24 20:04:44Z rusu $
 *
 */

// Include the implementations here instead of compiling them separately to speed up compile time
//#include "normal_3d.cpp"
//#include "boundary.cpp"
//#include "fpfh.cpp"
//#include "fpfh_omp.cpp"
//#include "moment_invariants.cpp"
//#include "normal_3d_omp.cpp"
//#include "normal_3d_tbb.cpp"
//#include "pfh.cpp"
//#include "principal_curvatures.cpp"
//#include "vfh.cpp"
#include <pcl/io/io.h>
#include "pcl_ros/features/feature.h"
#include <message_filters/null_types.h>

////////////////////////////////////////////////////////////////////////////////////////////
pcl_ros::Feature::Feature (std::string node_name, const rclcpp::NodeOptions& options) : pcl_ros::PCLNode(node_name, options)
{
  // Allow each individual class that inherits from us to declare their own Publisher
  // This is useful for Publisher<pcl::PointCloud<T> >, as NormalEstimation can publish PointCloud<Normal>, etc
  //pub_output_ = this->template advertise<PointCloud2> ("output", max_queue_size_);

  // ---[ Mandatory parameters
  if (!this->get_parameter ("k_search", k_) && !this->get_parameter ("radius_search", search_radius_))
  {
    RCLCPP_ERROR (this->get_logger(), "[%s::onConstructor] Neither 'k_search' nor 'radius_search' set! Need to set at least one of these parameters before continuing.", this->get_name());
    return;
  }
  if (!this->get_parameter ("spatial_locator", spatial_locator_type_))
  {
    RCLCPP_ERROR (this->get_logger(), "[%s::onConstructor] Need a 'spatial_locator' parameter to be set before continuing!", this->get_name ());
    return;
  }

  // ---[ Optional parameters
  this->get_parameter ("use_surface", use_surface_);

  RCLCPP_DEBUG (this->get_logger(), "[%s::onConstructor] Node successfully created with the following parameters:\n"
                 " - use_surface    : %s\n"
                 " - k_search       : %d\n"
                 " - radius_search  : %f\n"
                 " - spatial_locator: %d",
                this->get_name (),
                (use_surface_) ? "true" : "false", k_, search_radius_, spatial_locator_type_);
  
  pub_output_ = this->create_publisher<PointCloud2> ("output", max_queue_size_);

}

////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::Feature::subscribe ()
{
  // If we're supposed to look for PointIndices (indices) or PointCloud (surface) messages
  if (use_indices_ || use_surface_)
  {
    // Create the objects here
    if (approximate_sync_)
      sync_input_surface_indices_a_ = std::make_shared<message_filters::Synchronizer<sync_policies::ApproximateTime<PointCloudIn, PointCloudIn, PointIndices> > >(max_queue_size_);
    else
      sync_input_surface_indices_e_ = std::make_shared<message_filters::Synchronizer<sync_policies::ExactTime<PointCloudIn, PointCloudIn, PointIndices> > >(max_queue_size_);

    // Subscribe to the input using a filter
    sub_input_filter_.subscribe (this->shared_from_this (), "input");
    if (use_indices_)
    {
      // If indices are enabled, subscribe to the indices
      sub_indices_filter_.subscribe (this->shared_from_this (), "indices");
      if (use_surface_)     // Use both indices and surface
      {
        // If surface is enabled, subscribe to the surface, connect the input-indices-surface trio and register
        sub_surface_filter_.subscribe (this->shared_from_this (), "surface");
        if (approximate_sync_)
          sync_input_surface_indices_a_->connectInput (sub_input_filter_, sub_surface_filter_, sub_indices_filter_);
        else
          sync_input_surface_indices_e_->connectInput (sub_input_filter_, sub_surface_filter_, sub_indices_filter_);
      }
      else                  // Use only indices
      {
        sub_input_filter_.registerCallback (std::bind (&Feature::input_callback, this, std::placeholders::_1));
        // surface not enabled, connect the input-indices duo and register
        if (approximate_sync_)
          sync_input_surface_indices_a_->connectInput (sub_input_filter_, nf_pc_, sub_indices_filter_);
        else
          sync_input_surface_indices_e_->connectInput (sub_input_filter_, nf_pc_, sub_indices_filter_);
      }
    }
    else                    // Use only surface
    {
      sub_input_filter_.registerCallback (std::bind (&Feature::input_callback, this, std::placeholders::_1));
      // indices not enabled, connect the input-surface duo and register
      sub_surface_filter_.subscribe (this->shared_from_this (), "surface");
      if (approximate_sync_)
        sync_input_surface_indices_a_->connectInput (sub_input_filter_, sub_surface_filter_, nf_pi_);
      else
        sync_input_surface_indices_e_->connectInput (sub_input_filter_, sub_surface_filter_, nf_pi_);
    }
    // Register callbacks
    if (approximate_sync_)
      sync_input_surface_indices_a_->registerCallback (std::bind (&Feature::input_surface_indices_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    else
      sync_input_surface_indices_e_->registerCallback (std::bind (&Feature::input_surface_indices_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  }
  else
    // Subscribe in an old fashion to input only (no filters)
    // Type masquerading not yet supported
    // sub_input_ = this->create_subscription<PointCloudIn> ("input", std::bind (&Feature::input_surface_indices_callback, this, std::placeholders::_1, PointCloudInConstPtr (), PointIndicesConstPtr ()));
    std::cout << "Type masquerading not yet supported" << std::endl;

}

////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::Feature::unsubscribe ()
{
  if (use_indices_ || use_surface_)
  {
    sub_input_filter_.unsubscribe ();
    if (use_indices_)
    {
      sub_indices_filter_.unsubscribe ();
      if (use_surface_)
        sub_surface_filter_.unsubscribe ();
    }
    else
      sub_surface_filter_.unsubscribe ();
  }
  else
    // FIXME
    std::cout << "shutdown" << std::endl;
    //sub_input_.shutdown ();
}


////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::Feature::input_surface_indices_callback (const PointCloudInConstPtr &cloud, 
    const PointCloudInConstPtr &cloud_surface, const PointIndicesConstPtr &indices)
{
  // No subscribers, no work
  /*
   count_subscribers not implemented ROS2
   if (pub_output_->count_subscribers () <= 0)
    return;
 */
  // If cloud is given, check if it's valid
  if (!isValid (cloud))
  {
    RCLCPP_ERROR (this->get_logger(), "[%s::input_surface_indices_callback] Invalid input!", this->get_name ());
    emptyPublish (cloud);
    return;
  }

  // If surface is given, check if it's valid
  if (cloud_surface && !isValid (cloud_surface, "surface"))
  {
    RCLCPP_ERROR (this->get_logger(), "[%s::input_surface_indices_callback] Invalid input surface!", this->get_name ());
    emptyPublish (cloud);
    return;
  }
    
  // If indices are given, check if they are valid
  if (indices && !isValid (indices))
  {
    RCLCPP_ERROR (this->get_logger(), "[%s::input_surface_indices_callback] Invalid input indices!", this->get_name ());
    emptyPublish (cloud);
    return;
  }

  /// DEBUG
  if (cloud_surface)
    if (indices)
      RCLCPP_DEBUG (this->get_logger(), "[input_surface_indices_callback]\n"
                     "                                         - PointCloud with %d data points (%s), stamp %f, and frame %s on topic %s received.\n"
                     "                                         - PointCloud with %d data points (%s), stamp %f, and frame %s on topic %s received.\n"
                     "                                         - PointIndices with %zu values, stamp %f, and frame %s on topic %s received.",
                 cloud->width * cloud->height, pcl::getFieldsList (*cloud).c_str (), fromPCL(cloud->header).stamp.sec, cloud->header.frame_id.c_str (), "input",
                 cloud_surface->width * cloud_surface->height, pcl::getFieldsList (*cloud_surface).c_str (), fromPCL(cloud_surface->header).stamp.sec, cloud_surface->header.frame_id.c_str (), "surface",
                 indices->indices.size (), indices->header.stamp.sec, indices->header.frame_id.c_str (), "indices");
    else
      RCLCPP_DEBUG (this->get_logger(), "[input_surface_indices_callback]\n"
                     "                                 - PointCloud with %d data points (%s), stamp %f, and frame %s on topic %s received.\n"
                     "                                 - PointCloud with %d data points (%s), stamp %f, and frame %s on topic %s received.",
                     cloud->width * cloud->height, pcl::getFieldsList (*cloud).c_str (), fromPCL(cloud->header).stamp.sec, cloud->header.frame_id.c_str (), "input",
                     cloud_surface->width * cloud_surface->height, pcl::getFieldsList (*cloud_surface).c_str (), fromPCL(cloud_surface->header).stamp.sec, cloud_surface->header.frame_id.c_str (), "surface");

  else
    if (indices)
      RCLCPP_DEBUG (this->get_logger(), "[input_surface_indices_callback]\n"
                     "                                 - PointCloud with %d data points (%s), stamp %f, and frame %s on topic %s received.\n"
                     "                                 - PointIndices with %zu values, stamp %f, and frame %s on topic %s received.",
                     cloud->width * cloud->height, pcl::getFieldsList (*cloud).c_str (), fromPCL(cloud->header).stamp.sec, cloud->header.frame_id.c_str (), "input",
                     indices->indices.size (), indices->header.stamp.sec, indices->header.frame_id.c_str (), "indices");
    else
      RCLCPP_DEBUG (this->get_logger(), "[input_surface_indices_callback] PointCloud with %d data points, stamp %f, and frame %s on topic %s received.", cloud->width * cloud->height, fromPCL(cloud->header).stamp.sec, cloud->header.frame_id.c_str (), "input");
  ///


  if ((int)(cloud->width * cloud->height) < k_)
  {
    RCLCPP_ERROR (this->get_logger(), "[input_surface_indices_callback] Requested number of k-nearest neighbors (%d) is larger than the PointCloud size (%d)!", k_, (int)(cloud->width * cloud->height));
    emptyPublish (cloud);
    return;
  }

  // If indices given...
  IndicesPtr vindices;
  if (indices && !indices->header.frame_id.empty ())
    vindices.reset (new std::vector<int> (indices->indices));

  computePublish (cloud, cloud_surface, vindices);
}

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////
pcl_ros::FeatureFromNormals::FeatureFromNormals (std::string node_name, const rclcpp::NodeOptions& options) : Feature(node_name, options), normals_()
{
  // Allow each individual class that inherits from us to declare their own Publisher
  // This is useful for Publisher<pcl::PointCloud<T> >, as NormalEstimation can publish PointCloud<Normal>, etc
  //pub_output_ = this->template advertise<PointCloud2> ("output", max_queue_size_);

  // ---[ Mandatory parameters
  if (!this->get_parameter ("k_search", k_) && !this->get_parameter ("radius_search", search_radius_))
  {
    RCLCPP_ERROR (this->get_logger(), "[%s::onConstructor] Neither 'k_search' nor 'radius_search' set! Need to set at least one of these parameters before continuing.", this->get_name ());
    return;
  }
  if (!this->get_parameter ("spatial_locator", spatial_locator_type_))
  {
    RCLCPP_ERROR (this->get_logger(), "[%s::onConstructor] Need a 'spatial_locator' parameter to be set before continuing!", this->get_name ());
    return;
  }
  // ---[ Optional parameters
  this->get_parameter ("use_surface", use_surface_);

  RCLCPP_DEBUG (this->get_logger(), "[%s::onConstructor] Node successfully created with the following parameters:\n"
                 " - use_surface    : %s\n"
                 " - k_search       : %d\n"
                 " - radius_search  : %f\n"
                 " - spatial_locator: %d",
                this->get_name (),
                 (use_surface_) ? "true" : "false", k_, search_radius_, spatial_locator_type_);

}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::FeatureFromNormals::subscribe ()
{
  sub_input_filter_.subscribe (this->shared_from_this (), "input");
  sub_normals_filter_.subscribe (this->shared_from_this (), "normals");

  // Create the objects here
  if (approximate_sync_)
    sync_input_normals_surface_indices_a_ = std::make_shared <message_filters::Synchronizer<sync_policies::ApproximateTime<PointCloudIn, PointCloudN, PointCloudIn, PointIndices> > > (max_queue_size_);
  else
    sync_input_normals_surface_indices_e_ = std::make_shared <message_filters::Synchronizer<sync_policies::ExactTime<PointCloudIn, PointCloudN, PointCloudIn, PointIndices> > > (max_queue_size_);

  // If we're supposed to look for PointIndices (indices) or PointCloud (surface) messages
  if (use_indices_ || use_surface_)
  {
    if (use_indices_)
    {
      // If indices are enabled, subscribe to the indices
      sub_indices_filter_.subscribe (this->shared_from_this(), "indices");
      if (use_surface_)     // Use both indices and surface
      {
        // If surface is enabled, subscribe to the surface, connect the input-indices-surface trio and register
        sub_surface_filter_.subscribe (this->shared_from_this (), "surface");
        if (approximate_sync_)
          sync_input_normals_surface_indices_a_->connectInput (sub_input_filter_, sub_normals_filter_, sub_surface_filter_, sub_indices_filter_);
        else
          sync_input_normals_surface_indices_e_->connectInput (sub_input_filter_, sub_normals_filter_, sub_surface_filter_, sub_indices_filter_);
      }
      else                  // Use only indices
      {
        sub_input_filter_.registerCallback (std::bind (&FeatureFromNormals::input_callback, this, std::placeholders::_1));
        if (approximate_sync_)
          // surface not enabled, connect the input-indices duo and register
          sync_input_normals_surface_indices_a_->connectInput (sub_input_filter_, sub_normals_filter_, nf_pc_, sub_indices_filter_);
        else
          // surface not enabled, connect the input-indices duo and register
          sync_input_normals_surface_indices_e_->connectInput (sub_input_filter_, sub_normals_filter_, nf_pc_, sub_indices_filter_);
      }
    }
    else                    // Use only surface
    {
      // indices not enabled, connect the input-surface duo and register
      sub_surface_filter_.subscribe (this->shared_from_this (), "surface");

      sub_input_filter_.registerCallback (std::bind (&FeatureFromNormals::input_callback, this, std::placeholders::_1));
      if (approximate_sync_)
        sync_input_normals_surface_indices_a_->connectInput (sub_input_filter_, sub_normals_filter_, sub_surface_filter_, nf_pi_);
      else
        sync_input_normals_surface_indices_e_->connectInput (sub_input_filter_, sub_normals_filter_, sub_surface_filter_, nf_pi_);
    }
  }
  else
  {
    sub_input_filter_.registerCallback (std::bind (&FeatureFromNormals::input_callback, this, std::placeholders::_1));

    if (approximate_sync_)
      sync_input_normals_surface_indices_a_->connectInput (sub_input_filter_, sub_normals_filter_, nf_pc_, nf_pi_);
    else
      sync_input_normals_surface_indices_e_->connectInput (sub_input_filter_, sub_normals_filter_, nf_pc_, nf_pi_);
  }

  // Register callbacks
  if (approximate_sync_)
    sync_input_normals_surface_indices_a_->registerCallback (std::bind (&FeatureFromNormals::input_normals_surface_indices_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
  else
    sync_input_normals_surface_indices_e_->registerCallback (std::bind (&FeatureFromNormals::input_normals_surface_indices_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::FeatureFromNormals::unsubscribe ()
{
  sub_input_filter_.unsubscribe ();
  sub_normals_filter_.unsubscribe ();
  if (use_indices_ || use_surface_)
  {
    if (use_indices_)
    {
      sub_indices_filter_.unsubscribe ();
      if (use_surface_)
        sub_surface_filter_.unsubscribe ();
    }
    else
      sub_surface_filter_.unsubscribe ();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::FeatureFromNormals::input_normals_surface_indices_callback (
   const PointCloudInConstPtr &cloud, const PointCloudNConstPtr &cloud_normals,
   const PointCloudInConstPtr &cloud_surface, const PointIndicesConstPtr &indices)
{
  // No subscribers, no work
  /*
   count_subscribers not implemented ROS2
  if (pub_output_.count_subscribers () <= 0)
    return;
  */
  
  // If cloud+normals is given, check if it's valid
  if (!isValid (cloud))// || !isValid (cloud_normals, "normals"))
  {
    RCLCPP_ERROR (this->get_logger(), "[%s::input_normals_surface_indices_callback] Invalid input!", this->get_name ());
    emptyPublish (cloud);
    return;
  }

  // If surface is given, check if it's valid
  if (cloud_surface && !isValid (cloud_surface, "surface"))
  {
    RCLCPP_ERROR (this->get_logger(), "[%s::input_normals_surface_indices_callback] Invalid input surface!", this->get_name ());
    emptyPublish (cloud);
    return;
  }
    
  // If indices are given, check if they are valid
  if (indices && !isValid (indices))
  {
    RCLCPP_ERROR (this->get_logger(), "[%s::input_normals_surface_indices_callback] Invalid input indices!", this->get_name ());
    emptyPublish (cloud);
    return;
  }

  /// DEBUG
  if (cloud_surface)
    if (indices)
      RCLCPP_DEBUG (this->get_logger(), "[%s::input_normals_surface_indices_callback]\n"
                     "                                                 - PointCloud with %d data points (%s), stamp %f, and frame %s on topic %s received.\n"
                     "                                                 - PointCloud with %d data points (%s), stamp %f, and frame %s on topic %s received.\n"
                     "                                                 - PointCloud with %d data points (%s), stamp %f, and frame %s on topic %s received.\n"
                     "                                                 - PointIndices with %zu values, stamp %f, and frame %s on topic %s received.",
                     this->get_name (),
                     cloud->width * cloud->height, pcl::getFieldsList (*cloud).c_str (), fromPCL(cloud->header).stamp.sec, cloud->header.frame_id.c_str (), "input",
                     cloud_surface->width * cloud_surface->height, pcl::getFieldsList (*cloud_surface).c_str (), fromPCL(cloud_surface->header).stamp.sec, cloud_surface->header.frame_id.c_str (), "surface",
                     cloud_normals->width * cloud_normals->height, pcl::getFieldsList (*cloud_normals).c_str (), fromPCL(cloud_normals->header).stamp.sec, cloud_normals->header.frame_id.c_str (), "normals",
                     indices->indices.size (), indices->header.stamp.sec, indices->header.frame_id.c_str (), "indices");
    else
      RCLCPP_DEBUG (this->get_logger(), "[%s::input_normals_surface_indices_callback]\n"
                     "                                         - PointCloud with %d data points (%s), stamp %f, and frame %s on topic %s received.\n"
                     "                                         - PointCloud with %d data points (%s), stamp %f, and frame %s on topic %s received.\n"
                     "                                         - PointCloud with %d data points (%s), stamp %f, and frame %s on topic %s received.",
                     this->get_name (),
                     cloud->width * cloud->height, pcl::getFieldsList (*cloud).c_str (), fromPCL(cloud->header).stamp.sec, cloud->header.frame_id.c_str (), "input",
                     cloud_surface->width * cloud_surface->height, pcl::getFieldsList (*cloud_surface).c_str (), fromPCL(cloud_surface->header).stamp.sec, cloud_surface->header.frame_id.c_str (), "surface",
                     cloud_normals->width * cloud_normals->height, pcl::getFieldsList (*cloud_normals).c_str (), fromPCL(cloud_normals->header).stamp.sec, cloud_normals->header.frame_id.c_str (), "normals");
  else
    if (indices)
      RCLCPP_DEBUG (this->get_logger(), "[%s::input_normals_surface_indices_callback]\n"
                     "                                         - PointCloud with %d data points (%s), stamp %f, and frame %s on topic %s received.\n"
                     "                                         - PointCloud with %d data points (%s), stamp %f, and frame %s on topic %s received.\n"
                     "                                         - PointIndices with %zu values, stamp %f, and frame %s on topic %s received.",
                     this->get_name (),
                     cloud->width * cloud->height, pcl::getFieldsList (*cloud).c_str (), fromPCL(cloud->header).stamp.sec, cloud->header.frame_id.c_str (), "input",
                     cloud_normals->width * cloud_normals->height, pcl::getFieldsList (*cloud_normals).c_str (), fromPCL(cloud_normals->header).stamp.sec, cloud_normals->header.frame_id.c_str (), "normals",
                     indices->indices.size (), indices->header.stamp.sec, indices->header.frame_id.c_str (), "indices");
    else
      RCLCPP_DEBUG (this->get_logger(), "[%s::input_normals_surface_indices_callback]\n"
                     "                                 - PointCloud with %d data points (%s), stamp %f, and frame %s on topic %s received.\n"
                     "                                 - PointCloud with %d data points (%s), stamp %f, and frame %s on topic %s received.",
                     this->get_name (),
                     cloud->width * cloud->height, pcl::getFieldsList (*cloud).c_str (), fromPCL(cloud->header).stamp.sec, cloud->header.frame_id.c_str (), "input",
                     cloud_normals->width * cloud_normals->height, pcl::getFieldsList (*cloud_normals).c_str (), fromPCL(cloud_normals->header).stamp.sec, cloud_normals->header.frame_id.c_str (), "normals");
  ///

  if ((int)(cloud->width * cloud->height) < k_)
  {
    RCLCPP_ERROR (this->get_logger(), "[%s::input_normals_surface_indices_callback] Requested number of k-nearest neighbors (%d) is larger than the PointCloud size (%d)!", this->get_name (), k_, (int)(cloud->width * cloud->height));
    emptyPublish (cloud);
    return;
  }

  // If indices given...
  IndicesPtr vindices;
  if (indices && !indices->header.frame_id.empty ())
    vindices.reset (new std::vector<int> (indices->indices));

  computePublish (cloud, cloud_normals, cloud_surface, vindices);
}

