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
 * $Id: pcl_node.h 33238 2010-03-11 00:46:58Z rusu $
 *
 */

/**

\author Radu Bogdan Rusu

**/

#ifndef PCL_NODE_H_
#define PCL_NODE_H_

#include <rclcpp/rclcpp.hpp>
#include <rcutils/error_handling.h>

#include <sensor_msgs/msg/point_cloud2.hpp>

// PCL includes
#include <pcl_msgs/msg/point_indices.hpp>
#include <pcl_msgs/msg/model_coefficients.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/point_cloud.h"
// ROS Node includes
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

// Include TF
#include <tf2_ros/transform_listener.h>

using pcl_conversions::fromPCL;

namespace pcl_ros
{
  ////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b PCLNode represents the base PCL Node class. All PCL node should inherit from this class. */
  class PCLNode : public rclcpp::Node
  {
    public:
      typedef sensor_msgs::msg::PointCloud2 PointCloud2;

      typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
      typedef PointCloud::Ptr PointCloudPtr;
      typedef PointCloud::ConstPtr PointCloudConstPtr;

      typedef pcl_msgs::msg::PointIndices PointIndices;
      typedef PointIndices::SharedPtr PointIndicesPtr;
      typedef PointIndices::ConstSharedPtr PointIndicesConstPtr;

      typedef pcl_msgs::msg::ModelCoefficients ModelCoefficients;
      typedef ModelCoefficients::SharedPtr ModelCoefficientsPtr;
      typedef ModelCoefficients::ConstSharedPtr ModelCoefficientsConstPtr;

      typedef std::shared_ptr <std::vector<int> > IndicesPtr;
      typedef std::shared_ptr <const std::vector<int> > IndicesConstPtr;

      /** \brief Empty constructor. */
      PCLNode (std::string node_name, const rclcpp::NodeOptions& options) : rclcpp::Node(node_name, options), tf_buffer_(this->get_clock()), use_indices_ (false), latched_indices_ (false),
                    max_queue_size_ (3), approximate_sync_ (false), tf_listener_(tf_buffer_) {
                      // Parameters that we care about only at startup
                      this->get_parameter ("max_queue_size", max_queue_size_);
                      
                      // ---[ Optional parameters
                      this->get_parameter ("use_indices", use_indices_);
                      this->get_parameter ("latched_indices", latched_indices_);
                      this->get_parameter ("approximate_sync", approximate_sync_);
                      
                      RCLCPP_DEBUG (this->get_logger(), "[PCLNode::constructor] PCL Node successfully created with the following parameters:\n"
                                    " - approximate_sync : %s\n"
                                    " - use_indices      : %s\n"
                                    " - latched_indices  : %s\n"
                                    " - max_queue_size   : %d",
                                    (approximate_sync_) ? "true" : "false",
                                    (use_indices_) ? "true" : "false",
                                    (latched_indices_) ? "true" : "false",
                                    max_queue_size_);
                      };

    protected:
      /** \brief Set to true if point indices are used.
       *
       * When receiving a point cloud, if use_indices_ is false, the entire
       * point cloud is processed for the given operation. If use_indices_ is
       * true, then the ~indices topic is read to get the vector of point
       * indices specifying the subset of the point cloud that will be used for
       * the operation. In the case where use_indices_ is true, the ~input and
       * ~indices topics must be synchronised in time, either exact or within a
       * specified jitter. See also @ref latched_indices_ and approximate_sync.
       **/
      bool use_indices_;
      /** \brief Set to true if the indices topic is latched.
       *
       * If use_indices_ is true, the ~input and ~indices topics generally must
       * be synchronised in time. By setting this flag to true, the most recent
       * value from ~indices can be used instead of requiring a synchronised
       * message.
       **/
      bool latched_indices_;

      /** \brief The message filter subscriber for PointCloud2. */
      message_filters::Subscriber<PointCloud> sub_input_filter_;

      /** \brief The message filter subscriber for PointIndices. */
      message_filters::Subscriber<PointIndices> sub_indices_filter_;

      /** \brief The output PointCloud publisher. */
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_output_;


      /** \brief The maximum queue size (default: 3). */
      int max_queue_size_;

      /** \brief True if we use an approximate time synchronizer versus an exact one (false by default). */
      bool approximate_sync_;

      /** \brief TF listener object. */
      tf2_ros::Buffer tf_buffer_;
      tf2_ros::TransformListener tf_listener_;

      /** \brief Test whether a given PointCloud message is "valid" (i.e., has points, and width and height are non-zero).
        * \param cloud the point cloud to test
        * \param topic_name an optional topic name (only used for printing, defaults to "input")
        */
      inline bool
      isValid (const PointCloud2::ConstSharedPtr &cloud, const std::string &topic_name = "input")
      {
        if (cloud->width * cloud->height * cloud->point_step != cloud->data.size ())
        {
          RCLCPP_WARN(this->get_logger(), "[%s] Invalid PointCloud (data = %zu, width = %d, height = %d, step = %d) with stamp %f, and frame %s on topic %s received!", this->get_name (), cloud->data.size (), cloud->width, cloud->height, cloud->point_step, cloud->header.stamp.sec , cloud->header.frame_id.c_str (), topic_name.c_str ());

          return (false);
        }
        return (true);
      }

      /** \brief Test whether a given PointCloud message is "valid" (i.e., has points, and width and height are non-zero).
        * \param cloud the point cloud to test
        * \param topic_name an optional topic name (only used for printing, defaults to "input")
        */
      inline bool
      isValid (const PointCloudConstPtr &cloud, const std::string &topic_name = "input")
      {
        if (cloud->width * cloud->height != cloud->points.size ())
        {
          RCLCPP_WARN (this->get_logger(), "[%s] Invalid PointCloud (points = %zu, width = %d, height = %d) with stamp %f, and frame %s on topic %s received!", this->get_name (), cloud->points.size (), cloud->width, cloud->height, fromPCL(cloud->header).stamp.sec, cloud->header.frame_id.c_str (), topic_name.c_str ());

          return (false);
        }
        return (true);
      }

      /** \brief Test whether a given PointIndices message is "valid" (i.e., has values).
        * \param indices the point indices message to test
        * \param topic_name an optional topic name (only used for printing, defaults to "indices")
        */
      inline bool
      isValid (const PointIndicesConstPtr &indices, const std::string &topic_name = "indices")
      {
        /*if (indices->indices.empty ())
        {
          RCLCPP_WARN ("[%s] Empty indices (values = %zu) with stamp %f, and frame %s on topic %s received!", this->get_name (), indices->indices.size (), indices->header.stamp.sec, indices->header.frame_id.c_str (), topic_name.c_str ());
          return (true);
        }*/
        return (true);
      }

      /** \brief Test whether a given ModelCoefficients message is "valid" (i.e., has values).
        * \param model the model coefficients to test
        * \param topic_name an optional topic name (only used for printing, defaults to "model")
        */
      inline bool
      isValid (const ModelCoefficientsConstPtr &model, const std::string &topic_name = "model")
      {
        /*if (model->values.empty ())
        {
          RCLCPP_WARN ("[%s] Empty model (values = %zu) with stamp %f, and frame %s on topic %s received!", this->get_name (), model->values.size (), model->header.stamp.sec, model->header.frame_id.c_str (), topic_name.c_str ());
          return (false);
        }*/
        return (true);
      }

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#endif  //#ifndef PCL_NODE_H_
