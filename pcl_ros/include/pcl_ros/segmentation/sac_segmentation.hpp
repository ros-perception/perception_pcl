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

#ifndef PCL_ROS__SEGMENTATION__SAC_SEGMENTATION_HPP_
#define PCL_ROS__SEGMENTATION__SAC_SEGMENTATION_HPP_

#include <pcl/segmentation/sac_segmentation.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/pass_through.h>
#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include "pcl_ros/pcl_nodelet.hpp"

namespace pcl_ros
{
  namespace sync_policies = message_filters::sync_policies;

  /** \brief @b SACSegmentation represents the PCL nodelet segmentation class for Sample Consensus methods and models
    * \author Radu Bogdan Rusu
    */
  class SACSegmentation : public PCLNodelet
  {
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    typedef std::shared_ptr<PointCloud> PointCloudPtr;
    typedef std::shared_ptr<const PointCloud> PointCloudConstPtr;

    typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudRGBA;

    public:
      /** \brief Constructor. */
      SACSegmentation() : PCLNodelet("sac_segmentation"), min_inliers_(0) {}

    protected:
      /** \brief The PCL implementation used. */
      pcl::SACSegmentation<pcl::PointXYZ> impl_;

      /** \brief The output PointIndices publisher. */
      rclcpp::Publisher<PointIndices>::SharedPtr pub_indices_;

      /** \brief The output ModelCoefficients publisher. */
      rclcpp::Publisher<ModelCoefficients>::SharedPtr pub_model_;

      /** \brief The input PointCloud subscriber. */
      rclcpp::Subscription<PointCloud>::SharedPtr sub_input_;

      /** \brief Synchronized input, and indices.*/
      std::shared_ptr<message_filters::Synchronizer<sync_policies::ExactTime<PointCloud, PointIndices>>> sync_input_indices_e_;
      std::shared_ptr<message_filters::Synchronizer<sync_policies::ApproximateTime<PointCloud, PointIndices>>> sync_input_indices_a_;

      /** \brief Parameter callback handle. */
      rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

      /** \brief Minimum number of inliers required. */
      int min_inliers_;

      /** \brief Input TF frame the data should be transformed into, if input.header.frame_id is different. */
      std::string tf_input_frame_;

      /** \brief The original data input TF frame. */
      std::string tf_input_orig_frame_;

      /** \brief Output TF frame the data should be transformed into, if input.header.frame_id is different. */
      std::string tf_output_frame_;

      /** \brief Mutex. */
      std::mutex mutex_;

      /** \brief Nodelet initialization routine. */
      void onInit();

      /** \brief LazyNodelet connection routine. */
      void subscribe();
      void unsubscribe();

      /** \brief Parameter callback
        * \param parameters the changed parameters
        */
      rcl_interfaces::msg::SetParametersResult config_callback(
        const std::vector<rclcpp::Parameter> & parameters);

      /** \brief PointCloud + PointIndices callback.
        * \param cloud the pointer to the input point cloud
        * \param indices the pointer to the input point cloud indices
        */
      void input_indices_callback(
        const PointCloudConstPtr & cloud,
        const PointIndicesConstPtr & indices);

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  /** \brief @b SACSegmentationFromNormals represents the PCL nodelet segmentation class for Sample Consensus methods and models that require the use of surface normals for estimation.
    * \author Radu Bogdan Rusu
    */
  class SACSegmentationFromNormals : public PCLNodelet
  {
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    typedef std::shared_ptr<PointCloud> PointCloudPtr;
    typedef std::shared_ptr<const PointCloud> PointCloudConstPtr;

    typedef pcl::PointCloud<pcl::Normal> PointCloudN;
    typedef std::shared_ptr<PointCloudN> PointCloudNPtr;
    typedef std::shared_ptr<const PointCloudN> PointCloudNConstPtr;

    public:
      /** \brief Constructor. */
      SACSegmentationFromNormals() : PCLNodelet("sac_segmentation_from_normals"), min_inliers_(0) {}

    protected:
      /** \brief The PCL implementation used. */
      pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> impl_;

      /** \brief The output PointIndices publisher. */
      rclcpp::Publisher<PointIndices>::SharedPtr pub_indices_;

      /** \brief The output ModelCoefficients publisher. */
      rclcpp::Publisher<ModelCoefficients>::SharedPtr pub_model_;

      /** \brief The normals PointCloud subscriber filter. */
      message_filters::Subscriber<PointCloudN> sub_normals_filter_;

      /** \brief The input PointCloud subscriber filter. */
      message_filters::Subscriber<PointCloud> sub_input_filter_;

      /** \brief The axis subscriber. */
      rclcpp::Subscription<ModelCoefficients>::SharedPtr sub_axis_;

      /** \brief Synchronized input, normals, and indices.*/
      std::shared_ptr<message_filters::Synchronizer<sync_policies::ExactTime<PointCloud, PointCloudN, PointIndices>>> sync_input_normals_indices_e_;
      std::shared_ptr<message_filters::Synchronizer<sync_policies::ApproximateTime<PointCloud, PointCloudN, PointIndices>>> sync_input_normals_indices_a_;

      /** \brief Parameter callback handle. */
      rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

      /** \brief Minimum number of inliers required. */
      int min_inliers_;

      /** \brief Mutex. */
      std::mutex mutex_;

      /** \brief Nodelet initialization routine. */
      void onInit();

      /** \brief LazyNodelet connection routine. */
      void subscribe();
      void unsubscribe();

      /** \brief Parameter callback
        * \param parameters the changed parameters
        */
      rcl_interfaces::msg::SetParametersResult config_callback(
        const std::vector<rclcpp::Parameter> & parameters);

      /** \brief ModelCoefficients callback (used for setting an axis).
        * \param model a pointer to the model coefficients
        */
      void axis_callback(const ModelCoefficientsConstPtr & model);

      /** \brief PointCloud + Normals + PointIndices callback.
        * \param cloud the pointer to the input point cloud
        * \param cloud_normals the pointer to the input normals
        * \param indices the pointer to the input point cloud indices
        */
      void input_normals_indices_callback(
        const PointCloudConstPtr & cloud,
        const PointCloudNConstPtr & cloud_normals,
        const PointIndicesConstPtr & indices);

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#endif // PCL_ROS__SEGMENTATION__SAC_SEGMENTATION_HPP_