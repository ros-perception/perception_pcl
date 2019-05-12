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
 * $Id: sac_segmentation.h 35564 2011-01-27 07:32:12Z rusu $
 *
 */

#ifndef PCL_ROS_SAC_SEGMENTATION_H_
#define PCL_ROS_SAC_SEGMENTATION_H_

#include "pcl_ros/pcl_node.h"
#include <message_filters/pass_through.h>

// PCL includes
#include <pcl/segmentation/sac_segmentation.h>

namespace pcl_ros
{
  namespace sync_policies = message_filters::sync_policies;

  ////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b SACSegmentation represents the Node segmentation class for Sample Consensus methods and models, in
    * the sense that it just creates a Node wrapper for generic-purpose SAC-based segmentation.
    * \author Radu Bogdan Rusu
    */
  class SACSegmentation : public PCLNode
  {
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    typedef PointCloud::Ptr PointCloudPtr;
    typedef PointCloud::ConstPtr PointCloudConstPtr;

    public:
      /** \brief Constructor. */
      SACSegmentation (const rclcpp::NodeOptions& options);

      /** \brief Set the input TF frame the data should be transformed into before processing, if input.header.frame_id is different.
        * \param tf_frame the TF frame the input PointCloud should be transformed into before processing
        */
      inline void setInputTFframe (std::string tf_frame) { tf_input_frame_ = tf_frame; }
      
      /** \brief Get the TF frame the input PointCloud should be transformed into before processing. */
      inline std::string getInputTFframe () { return (tf_input_frame_); }

      /** \brief Set the output TF frame the data should be transformed into after processing.
        * \param tf_frame the TF frame the PointCloud should be transformed into after processing
        */
      inline void setOutputTFframe (std::string tf_frame) { tf_output_frame_ = tf_frame; }
      
      /** \brief Get the TF frame the PointCloud should be transformed into after processing. */
      inline std::string getOutputTFframe () { return (tf_output_frame_); }
    
      void subscribe ();
      void unsubscribe ();
    protected:
      // The minimum number of inliers a model must have in order to be considered valid.
      int min_inliers_;

      // ROS node attributes
      /** \brief The output PointIndices publisher. */
      rclcpp::Publisher<pcl_msgs::msg::PointIndices>::SharedPtr pub_indices_;

      /** \brief The output ModelCoefficients publisher. */
      rclcpp::Publisher<pcl_msgs::msg::ModelCoefficients>::SharedPtr pub_model_;

      /** \brief The input PointCloud subscriber. */
      rclcpp::Subscription<PointCloud>::SharedPtr sub_input_;

      /** \brief The input TF frame the data should be transformed into, if input.header.frame_id is different. */
      std::string tf_input_frame_;
      
      /** \brief The original data input TF frame. */
      std::string tf_input_orig_frame_;
      
      /** \brief The output TF frame the data should be transformed into, if input.header.frame_id is different. */
      std::string tf_output_frame_;

      /** \brief Null passthrough filter, used for pushing empty elements in the
        * synchronizer */
      message_filters::PassThrough<pcl_msgs::msg::PointIndices> nf_pi_;

      /** \brief Input point cloud callback. Used when \a use_indices is set.
        * \param cloud the pointer to the input point cloud
        * \param indices the pointer to the input point cloud indices
        */
      void input_indices_callback (const PointCloudPtr cloud, const PointIndicesPtr indices);

      /** \brief Pointer to a set of indices stored internally.
       * (used when \a latched_indices_ is set). 
       */
      PointIndices indices_;

      /** \brief Indices callback. Used when \a latched_indices_ is set.
        * \param indices the pointer to the input point cloud indices
        */
      inline void
      indices_callback (const PointIndicesConstPtr &indices)
      {
        indices_ = *indices.get();
      }

      /** \brief Input callback. Used when \a latched_indices_ is set.
        * \param input the pointer to the input point cloud
        */
      inline void
      input_callback (const PointCloudConstPtr &input)
      {
        indices_.header = fromPCL(input->header);
        PointIndicesConstPtr indices;
        indices.reset (new PointIndices (indices_));
        nf_pi_.add (indices);
      }

    private:
      /** \brief Internal mutex. */
      std::mutex mutex_;

      /** \brief The PCL implementation used. */
      pcl::SACSegmentation<pcl::PointXYZ> impl_;

      /** \brief Synchronized input, and indices.*/
      std::shared_ptr<message_filters::Synchronizer<sync_policies::ExactTime<PointCloud, PointIndices> > >       sync_input_indices_e_;
      std::shared_ptr<message_filters::Synchronizer<sync_policies::ApproximateTime<PointCloud, PointIndices> > > sync_input_indices_a_;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  ////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b SACSegmentationFromNormals represents the PCL node segmentation class for Sample Consensus methods and
    * models that require the use of surface normals for estimation.
    */
  class SACSegmentationFromNormals: public SACSegmentation
  {
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    typedef PointCloud::Ptr PointCloudPtr;
    typedef PointCloud::ConstPtr PointCloudConstPtr;

    typedef pcl::PointCloud<pcl::Normal> PointCloudN;
    typedef PointCloudN::Ptr PointCloudNPtr;
    typedef PointCloudN::ConstPtr PointCloudNConstPtr;

    public:
    
      SACSegmentationFromNormals(const rclcpp::NodeOptions& options);
      /** \brief Set the input TF frame the data should be transformed into before processing, if input.header.frame_id is different.
        * \param tf_frame the TF frame the input PointCloud should be transformed into before processing
        */
      inline void setInputTFframe (std::string tf_frame) { tf_input_frame_ = tf_frame; }
      
      /** \brief Get the TF frame the input PointCloud should be transformed into before processing. */
      inline std::string getInputTFframe () { return (tf_input_frame_); }

      /** \brief Set the output TF frame the data should be transformed into after processing.
        * \param tf_frame the TF frame the PointCloud should be transformed into after processing
        */
      inline void setOutputTFframe (std::string tf_frame) { tf_output_frame_ = tf_frame; }
      
      /** \brief Get the TF frame the PointCloud should be transformed into after processing. */
      inline std::string getOutputTFframe () { return (tf_output_frame_); }

    protected:
      // ROS node attributes
      /** \brief The normals PointCloud subscriber filter. */
      message_filters::Subscriber<PointCloudN> sub_normals_filter_;

      /** \brief The input PointCloud subscriber. */
      rclcpp::Subscription<pcl_msgs::msg::ModelCoefficients>::SharedPtr sub_axis_;
    
      /** \brief Input point cloud callback.
        * Because we want to use the same synchronizer object, we push back
        * empty elements with the same timestamp.
        */
      inline void
      input_callback (const PointCloudConstPtr &cloud)
      {
        PointIndices indices;
        indices.header.stamp = fromPCL(cloud->header).stamp;
        nf_.add (std::make_shared<PointIndices> (indices));
      }

      /** \brief Null passthrough filter, used for pushing empty elements in the 
        * synchronizer */
      message_filters::PassThrough<PointIndices> nf_;

      /** \brief The input TF frame the data should be transformed into, if input.header.frame_id is different. */
      std::string tf_input_frame_;
      /** \brief The original data input TF frame. */
      std::string tf_input_orig_frame_;
      /** \brief The output TF frame the data should be transformed into, if input.header.frame_id is different. */
      std::string tf_output_frame_;

      /** \brief Model callback
        * \param model the sample consensus model found
        */
      void axis_callback (const pcl_msgs::msg::ModelCoefficients::SharedPtr model);

      /** \brief Input point cloud callback.
        * \param cloud the pointer to the input point cloud
        * \param cloud_normals the pointer to the input point cloud normals
        * \param indices the pointer to the input point cloud indices
        */
      void input_normals_indices_callback (const PointCloudConstPtr &cloud, 
                                           const PointCloudNConstPtr &cloud_normals, 
                                           const PointIndicesConstPtr &indices);
   
    private:
      /** \brief Internal mutex. */
      std::mutex mutex_;

      /** \brief The PCL implementation used. */
      pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> impl_;

      /** \brief Synchronized input, normals, and indices.*/
      std::shared_ptr<message_filters::Synchronizer<sync_policies::ApproximateTime<PointCloud, PointCloudN, PointIndices> > > sync_input_normals_indices_a_;
      std::shared_ptr<message_filters::Synchronizer<sync_policies::ExactTime<PointCloud, PointCloudN, PointIndices> > > sync_input_normals_indices_e_;
    
      void subscribe();
      void unsubscribe();
    
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#endif  //#ifndef PCL_ROS_SAC_SEGMENTATION_H_
