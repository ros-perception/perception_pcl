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
 * $Id: filter.h 35876 2011-02-09 01:04:36Z rusu $
 *
 */

#ifndef PCL_ROS__FILTERS__FILTER_HPP_
#define PCL_ROS__FILTERS__FILTER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_msgs/msg/point_indices.hpp>

// TF
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// Message filters
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <memory>
#include <string>
#include <vector>

#include "pcl_ros/pcl_nodelet.hpp"

namespace pcl_ros
{
namespace sync_policies = message_filters::sync_policies;

////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
/** \brief @b Filter represents the base filter class. All filters must inherit from this class.
  * \author Radu Bogdan Rusu
  */
class Filter : public PCLNodelet
{
public:
  typedef sensor_msgs::msg::PointCloud2 PointCloud2;
  typedef PointCloud2::SharedPtr PointCloud2Ptr;
  typedef PointCloud2::ConstSharedPtr PointCloud2ConstPtr;

  typedef pcl_msgs::msg::PointIndices PointIndices;
  typedef PointIndices::SharedPtr PointIndicesPtr;
  typedef PointIndices::ConstSharedPtr PointIndicesConstPtr;

  /** \brief Empty constructor. */
  Filter() : PCLNodelet("filter_node"), tf_input_frame_(""), tf_output_frame_("") {}

  /** \brief Compute the actual filtering and publish the result.
    * \param input the input point cloud dataset.
    * \param indices the input set of indices to use from \a input
    */
  void computePublish(const PointCloud2ConstPtr & input, const IndicesPtr & indices);

protected:
  /** \brief The input PointCloud2 subscriber. */
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_input_;

  /** \brief The desired user filter field name. */
  std::string filter_field_name_;

  /** \brief The minimum allowed filter value a point will be filtered with. */
  double filter_limit_min_;

  /** \brief The maximum allowed filter value a point will be filtered with. */
  double filter_limit_max_;

  /** \brief Set to true if point filtering should be limited to a certain values range. */
  bool filter_limit_negative_;

  /** \brief The input TF frame the data should be transformed into, if input.header.frame_id is different. */
  std::string tf_input_frame_;

  /** \brief The original data input TF frame. */
  std::string tf_input_orig_frame_;

  /** \brief The output TF frame the data should be transformed into, if input.header.frame_id is different. */
  std::string tf_output_frame_;

  /** \brief TF2 buffer and listener for transforms. */
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  /** \brief The message filter subscriber for PointCloud2. */
  message_filters::Subscriber<PointCloud2> sub_input_filter_;

  /** \brief The message filter subscriber for PointIndices. */
  message_filters::Subscriber<PointIndices> sub_indices_filter_;

  /** \brief Synchronized input, and indices.*/
  std::shared_ptr<message_filters::Synchronizer<sync_policies::ExactTime<PointCloud2, PointIndices>>> sync_input_indices_e_;
  std::shared_ptr<message_filters::Synchronizer<sync_policies::ApproximateTime<PointCloud2, PointIndices>>> sync_input_indices_a_;

  /** \brief Parameter callback handle. */
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  /** \brief Child initialization routine. Internal method. */
  virtual bool child_init(bool has_service = false) = 0;

  /** \brief Filter a Point Cloud.
    * \param input the input point cloud dataset
    * \param indices the input set of indices to use from \a input
    * \param output the resultant filtered dataset
    */
  virtual void filter(const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output) = 0;

  /** \brief Parameter callback  
    * \param parameters the changed parameters
    */
  virtual rcl_interfaces::msg::SetParametersResult config_callback(const std::vector<rclcpp::Parameter> & parameters);

  /** \brief PointCloud2 + Indices data callback. */
  void input_indices_callback(const PointCloud2ConstPtr & cloud, const PointIndicesConstPtr & indices);

private:
  /** \brief Nodelet initialization routine. */
  void onInit();

  /** \brief LazyNodelet connection routine. */
  void subscribe();
  void unsubscribe();

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace pcl_ros

#endif  // PCL_ROS__FILTERS__FILTER_HPP_