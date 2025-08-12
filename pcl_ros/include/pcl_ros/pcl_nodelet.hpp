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
 */

#ifndef PCL_ROS__PCL_NODELET_HPP_
#define PCL_ROS__PCL_NODELET_HPP_

// ROS 2 includes
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/type_adapter.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_msgs/msg/point_indices.hpp>
#include <pcl_msgs/msg/model_coefficients.hpp>

// Message filters
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

// TF2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/time.h>

// PCL
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// PCL ROS 
#include <pcl_ros/point_cloud.hpp>

// STL
#include <string>
#include <memory>
#include <mutex>

namespace pcl_ros
{

/** \brief @b PCLNodelet represents the base PCL Node class for ROS 2.
  * All PCL nodes should inherit from this class.
  */
class PCLNodelet : public rclcpp::Node
{
public:
  // ROS 2 message types
  typedef sensor_msgs::msg::PointCloud2 PointCloud2;
  typedef PointCloud2::SharedPtr PointCloud2Ptr;
  typedef PointCloud2::ConstSharedPtr PointCloud2ConstPtr;

  typedef pcl_msgs::msg::PointIndices PointIndices;
  typedef PointIndices::SharedPtr PointIndicesPtr;
  typedef PointIndices::ConstSharedPtr PointIndicesConstPtr;

  typedef pcl_msgs::msg::ModelCoefficients ModelCoefficients;
  typedef ModelCoefficients::SharedPtr ModelCoefficientsPtr;
  typedef ModelCoefficients::ConstSharedPtr ModelCoefficientsConstPtr;

  // PCL types with TypeAdapter support
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
  typedef std::shared_ptr<PointCloud> PointCloudPtr;
  typedef std::shared_ptr<const PointCloud> PointCloudConstPtr;

  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;
  typedef std::shared_ptr<PointCloudRGB> PointCloudRGBPtr;
  typedef std::shared_ptr<const PointCloudRGB> PointCloudRGBConstPtr;

  typedef pcl::PointCloud<pcl::Normal> PointCloudNormal;
  typedef std::shared_ptr<PointCloudNormal> PointCloudNormalPtr;
  typedef std::shared_ptr<const PointCloudNormal> PointCloudNormalConstPtr;

  typedef pcl::IndicesPtr IndicesPtr;
  typedef pcl::IndicesConstPtr IndicesConstPtr;

  /** \brief Constructor. */
  explicit PCLNodelet(const std::string & node_name, 
                      const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node(node_name, options),
    use_indices_(false), 
    latched_indices_(false),
    max_queue_size_(3), 
    approximate_sync_(false),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // Initialize the node but don't call onInit here - let derived classes do it
  }

  virtual ~PCLNodelet() = default;

protected:
  /** \brief Set to true if point indices are used. */
  bool use_indices_;
  
  /** \brief Set to true if the indices topic is latched. */
  bool latched_indices_;

  /** \brief The message filter subscriber for PointCloud2. */
  message_filters::Subscriber<PointCloud2> sub_input_filter_;

  /** \brief The message filter subscriber for PointIndices. */
  message_filters::Subscriber<PointIndices> sub_indices_filter_;

  /** \brief The output PointCloud publisher (can use PCL types directly). */
  rclcpp::Publisher<PointCloud>::SharedPtr pub_output_;

  /** \brief The maximum queue size (default: 3). */
  int max_queue_size_;

  /** \brief True if we use an approximate time synchronizer. */
  bool approximate_sync_;

  /** \brief TF2 buffer. */
  tf2_ros::Buffer tf_buffer_;

  /** \brief TF2 listener object. */
  tf2_ros::TransformListener tf_listener_;

  /** \brief Mutex for thread safety. */
  std::mutex mutex_;

  /** \brief Test whether a PointCloud2 message is valid. */
  inline bool
  isValid(const PointCloud2ConstPtr & cloud, const std::string & topic_name = "input")
  {
    if (!cloud) {
      RCLCPP_WARN(this->get_logger(), "[%s] Null PointCloud2 on topic %s received!",
                  this->get_name(), topic_name.c_str());
      return false;
    }

    if (cloud->width == 0 || cloud->height == 0) {
      RCLCPP_WARN(this->get_logger(), 
                  "[%s] Empty PointCloud2 (width = %d, height = %d) "
                  "with frame %s on topic %s received!",
                  this->get_name(), cloud->width, cloud->height,
                  cloud->header.frame_id.c_str(), topic_name.c_str());
      return false;
    }

    if (cloud->data.empty()) {
      RCLCPP_WARN(this->get_logger(),
                  "[%s] PointCloud2 with no data (width = %d, height = %d) "
                  "with frame %s on topic %s received!",
                  this->get_name(), cloud->width, cloud->height,
                  cloud->header.frame_id.c_str(), topic_name.c_str());
      return false;
    }

    return true;
  }

  /** \brief Test whether a PCL PointCloud is valid. */
  inline bool
  isValid(const PointCloudConstPtr & cloud, const std::string & topic_name = "input")
  {
    if (!cloud) {
      RCLCPP_WARN(this->get_logger(), "[%s] Null PCL PointCloud on topic %s received!",
                  this->get_name(), topic_name.c_str());
      return false;
    }

    if (cloud->points.empty()) {
      RCLCPP_WARN(this->get_logger(),
                  "[%s] Empty PCL PointCloud (points = %zu, width = %d, height = %d) "
                  "with frame %s on topic %s received!",
                  this->get_name(), cloud->points.size(), cloud->width, cloud->height,
                  cloud->header.frame_id.c_str(), topic_name.c_str());
      return false;
    }

    if (cloud->width * cloud->height != cloud->points.size()) {
      RCLCPP_WARN(this->get_logger(),
                  "[%s] Invalid PCL PointCloud (points = %zu, width = %d, height = %d) "
                  "with frame %s on topic %s received!",
                  this->get_name(), cloud->points.size(), cloud->width, cloud->height,
                  cloud->header.frame_id.c_str(), topic_name.c_str());
      return false;
    }

    return true;
  }

  /** \brief Test whether a PointIndices message is valid. */
  inline bool
  isValid(const PointIndicesConstPtr & indices, const std::string & topic_name = "indices")
  {
    if (!indices) {
      RCLCPP_WARN(this->get_logger(), "[%s] Null PointIndices on topic %s received!",
                  this->get_name(), topic_name.c_str());
      return false;
    }

    if (indices->indices.empty()) {
      RCLCPP_DEBUG(this->get_logger(),
                   "[%s] Empty PointIndices (size = %zu) "
                   "with frame %s on topic %s received!",
                   this->get_name(), indices->indices.size(),
                   indices->header.frame_id.c_str(), topic_name.c_str());
      return true;
    }

    return true;
  }

  /** \brief Test whether a ModelCoefficients message is valid. */
  inline bool
  isValid(const ModelCoefficientsConstPtr & model, const std::string & topic_name = "model")
  {
    if (!model) {
      RCLCPP_WARN(this->get_logger(), "[%s] Null ModelCoefficients on topic %s received!",
                  this->get_name(), topic_name.c_str());
      return false;
    }

    if (model->values.empty()) {
      RCLCPP_WARN(this->get_logger(),
                  "[%s] Empty ModelCoefficients (size = %zu) "
                  "with frame %s on topic %s received!",
                  this->get_name(), model->values.size(),
                  model->header.frame_id.c_str(), topic_name.c_str());
      return false;
    }

    return true;
  }

  /** \brief Subscribe/unsubscribe routines. Override in derived classes. */
  virtual void subscribe() {}
  virtual void unsubscribe() {}

  /** \brief Node initialization routine. */
  virtual void onInit()
  {
    // Declare parameters with default values
    this->declare_parameter("max_queue_size", 3);
    this->declare_parameter("use_indices", false);
    this->declare_parameter("latched_indices", false);
    this->declare_parameter("approximate_sync", false);

    // Get parameter values
    max_queue_size_ = this->get_parameter("max_queue_size").as_int();
    use_indices_ = this->get_parameter("use_indices").as_bool();
    latched_indices_ = this->get_parameter("latched_indices").as_bool();
    approximate_sync_ = this->get_parameter("approximate_sync").as_bool();

    RCLCPP_DEBUG(this->get_logger(),
                 "[%s::onInit] PCL Node successfully created with parameters:\n"
                 " - approximate_sync : %s\n"
                 " - use_indices      : %s\n"
                 " - latched_indices  : %s\n"
                 " - max_queue_size   : %d",
                 this->get_name(),
                 approximate_sync_ ? "true" : "false",
                 use_indices_ ? "true" : "false",
                 latched_indices_ ? "true" : "false",
                 max_queue_size_);

    onInitPostProcess();
  }

  /** \brief Post-initialization routine. */
  virtual void onInitPostProcess()
  {
    subscribe();
  }

  /** \brief Helper function to create a PCL publisher. */
  template<typename PointT>
  typename rclcpp::Publisher<pcl::PointCloud<PointT>>::SharedPtr
  advertise(const std::string & topic, const rclcpp::QoS & qos)
  {
    return this->create_publisher<pcl::PointCloud<PointT>>(topic, qos);
  }

  /** \brief Helper function to create a publisher with queue size. */
  template<typename MessageT>
  typename rclcpp::Publisher<MessageT>::SharedPtr
  advertise(const std::string & topic, size_t queue_size)
  {
    return this->create_publisher<MessageT>(topic, rclcpp::QoS(queue_size));
  }

  /** \brief Helper to publish a PCL point cloud directly. */
  void publish(const PointCloud & pcl_cloud)
  {
    if (pub_output_) {
      pub_output_->publish(pcl_cloud);
    }
  }

  /** \brief Helper to publish with frame ID and timestamp. */
  void publish(const PointCloud & pcl_cloud, const std::string & frame_id)
  {
    if (pub_output_) {
      PointCloud cloud_copy = pcl_cloud;
      cloud_copy.header.frame_id = frame_id;
      rclcpp::Time now = this->get_clock()->now();
      cloud_copy.header.stamp = static_cast<std::uint64_t>(now.nanoseconds()) / 1000ull;
      pub_output_->publish(cloud_copy);
    }
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace pcl_ros

#endif  // PCL_ROS__PCL_NODELET_HPP_