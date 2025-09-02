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

#include <string>
#include <vector>

#include "pcl_ros/pcl_node.hpp"
#include <pcl_msgs/msg/point_indices.hpp>

// PCL includes
#include <pcl/segmentation/sac_segmentation.h>

namespace pcl_ros
{
namespace sync_policies = message_filters::sync_policies;

////////////////////////////////////////////////////////////////////////////////////////////
/** \brief @b SACSegmentation represents a segmentation class for Sample Consensus
  * methods and models, in the sense that it just creates a wrapper for generic-purpose
  * SAC-based segmentation.
  * \author Radu Bogdan Rusu
  */
class SACSegmentation : public PCLNode<pcl_msgs::msg::PointIndices>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit SACSegmentation(const rclcpp::NodeOptions & options);

protected:
  /** \brief declare and subscribe to param callback for input_frame and output_frame params */
  void
  use_frame_params();

  /** \brief The input PointCloud subscriber. */
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_input_;

  message_filters::Subscriber<PointCloud2> sub_input_filter_;

  /** \brief The input TF frame the data should be transformed into,
    * if input.header.frame_id is different.
    */
  std::string tf_input_frame_;

  /** \brief The original data input TF frame. */
  std::string tf_input_orig_frame_;

  /** \brief The output TF frame the data should be transformed into,
    * if input.header.frame_id is different.
    */
  std::string tf_output_frame_;

  /** \brief Internal mutex. */
  std::mutex mutex_;

  /** \brief Call the actual filter.
    * \param input the input point cloud dataset
    * \param indices the input set of indices to use from \a input
    * \param output the resultant filtered dataset
    * \param model the coefficients of the segmented model
    */
  inline void
  segment(
    const PointCloud2::ConstSharedPtr & input, const IndicesPtr & indices,
    PointIndices & output, ModelCoefficients & model);

  /** \brief Lazy transport subscribe routine. */
  virtual void
  subscribe();

  /** \brief Lazy transport unsubscribe routine. */
  virtual void
  unsubscribe();

  /** \brief Create publishers for output PointCloud2 data. */
  virtual void
  createPublishers();

  /** \brief Call the child filter () method, optionally transform the result, and publish it.
    * \param input the input point cloud dataset.
    * \param indices a pointer to the vector of point indices to use.
    */
  void
  computePublish(const PointCloud2::ConstSharedPtr & input, const IndicesPtr & indices);

private:
  /** \brief Pointer to parameters callback handle. */
  OnSetParametersCallbackHandle::SharedPtr callback_handle_;

  /** \brief Synchronized input, and indices.*/
  std::shared_ptr<message_filters::Synchronizer<sync_policies::ExactTime<PointCloud2,
    PointIndices>>> sync_input_indices_e_;
  std::shared_ptr<message_filters::Synchronizer<sync_policies::ApproximateTime<PointCloud2,
    PointIndices>>> sync_input_indices_a_;

  /** \brief The output ModelCoefficients publisher. */
  rclcpp::Publisher<ModelCoefficients>::SharedPtr pub_model_;

  /** \brief Parameter callback
    * \param params parameter values to set
    */
  rcl_interfaces::msg::SetParametersResult
  config_callback(const std::vector<rclcpp::Parameter> & params);

  /** \brief PointCloud2 + Indices data callback. */
  void
  input_indices_callback(
    const PointCloud2::ConstSharedPtr & cloud,
    const PointIndices::ConstSharedPtr & indices);

  /** \brief The PCL implementation used. */
  pcl::SACSegmentation<pcl::PointXYZ> impl_;
};

////////////////////////////////////////////////////////////////////////////////////////////
/** \brief @b SACSegmentationFromNormals represents the PCL nodelet segmentation class for
  * Sample Consensus methods and models that require the use of surface normals for estimation.
  */
// class SACSegmentationFromNormals : public SACSegmentation
// {
//   typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
//   typedef boost::shared_ptr<PointCloud> PointCloudPtr;
//   typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;

//   typedef pcl::PointCloud<pcl::Normal> PointCloudN;
//   typedef boost::shared_ptr<PointCloudN> PointCloudNPtr;
//   typedef boost::shared_ptr<const PointCloudN> PointCloudNConstPtr;

// public:
//   /** \brief Set the input TF frame the data should be transformed into before processing,
//     * if input.header.frame_id is different.
//     * \param tf_frame the TF frame the input PointCloud should be transformed into before processing
//     */
//   inline void setInputTFframe(std::string tf_frame) {tf_input_frame_ = tf_frame;}

//   /** \brief Get the TF frame the input PointCloud should be transformed into before processing. */
//   inline std::string getInputTFframe() {return tf_input_frame_;}

//   /** \brief Set the output TF frame the data should be transformed into after processing.
//     * \param tf_frame the TF frame the PointCloud should be transformed into after processing
//     */
//   inline void setOutputTFframe(std::string tf_frame) {tf_output_frame_ = tf_frame;}

//   /** \brief Get the TF frame the PointCloud should be transformed into after processing. */
//   inline std::string getOutputTFframe() {return tf_output_frame_;}

// protected:
//   // ROS nodelet attributes
//   /** \brief The normals PointCloud subscriber filter. */
//   message_filters::Subscriber<PointCloudN> sub_normals_filter_;

//   /** \brief The input PointCloud subscriber. */
//   ros::Subscriber sub_axis_;

//   /** \brief Pointer to a dynamic reconfigure service. */
//   boost::shared_ptr<dynamic_reconfigure::Server<SACSegmentationFromNormalsConfig>> srv_;

//   /** \brief Input point cloud callback.
//     * Because we want to use the same synchronizer object, we push back
//     * empty elements with the same timestamp.
//     */
//   inline void
//   input_callback(const PointCloudConstPtr & cloud)
//   {
//     PointIndices indices;
//     indices.header.stamp = fromPCL(cloud->header).stamp;
//     nf_.add(boost::make_shared<PointIndices>(indices));
//   }

//   /** \brief Null passthrough filter, used for pushing empty elements in the
//     * synchronizer */
//   message_filters::PassThrough<PointIndices> nf_;

//   /** \brief The input TF frame the data should be transformed into,
//    * if input.header.frame_id is different.
//    */
//   std::string tf_input_frame_;
//   /** \brief The original data input TF frame. */
//   std::string tf_input_orig_frame_;
//   /** \brief The output TF frame the data should be transformed into,
//     * if input.header.frame_id is different.
//     */
//   std::string tf_output_frame_;

//   /** \brief Nodelet initialization routine. */
//   virtual void onInit();

//   /** \brief LazyNodelet connection routine. */
//   virtual void subscribe();
//   virtual void unsubscribe();

//   /** \brief Model callback
//     * \param model the sample consensus model found
//     */
//   void axis_callback(const pcl_msgs::ModelCoefficientsConstPtr & model);

//   /** \brief Dynamic reconfigure callback
//     * \param config the config object
//     * \param level the dynamic reconfigure level
//     */
//   void config_callback(SACSegmentationFromNormalsConfig & config, uint32_t level);

//   /** \brief Input point cloud callback.
//     * \param cloud the pointer to the input point cloud
//     * \param cloud_normals the pointer to the input point cloud normals
//     * \param indices the pointer to the input point cloud indices
//     */
//   void input_normals_indices_callback(
//     const PointCloudConstPtr & cloud,
//     const PointCloudNConstPtr & cloud_normals,
//     const PointIndicesConstPtr & indices);

// private:
//   /** \brief Internal mutex. */
//   boost::mutex mutex_;

//   /** \brief The PCL implementation used. */
//   pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> impl_;

//   /** \brief Synchronized input, normals, and indices.*/
//   boost::shared_ptr<message_filters::Synchronizer<sync_policies::ApproximateTime<PointCloud,
//     PointCloudN, PointIndices>>> sync_input_normals_indices_a_;
//   boost::shared_ptr<message_filters::Synchronizer<sync_policies::ExactTime<PointCloud, PointCloudN,
//     PointIndices>>> sync_input_normals_indices_e_;

// public:
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// };
}  // namespace pcl_ros

#endif  // PCL_ROS__SEGMENTATION__SAC_SEGMENTATION_HPP_
