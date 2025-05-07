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
 */

/**
 * \author Radu Bogdan Rusu
 * \author Valerio Passamano
 *
 * @b combined_pointcloud_to_pcd is a node that accumulates multiple incoming
 * point cloud messages into a single PCD file, optionally transforming them
 * into a fixed frame.
 */

#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace pcl_ros
{

class CombinedPointCloudToPCD : public rclcpp::Node
{
public:
  explicit CombinedPointCloudToPCD(const rclcpp::NodeOptions & options)
  : rclcpp::Node("combined_pointcloud_to_pcd", options),
    binary_(false),
    compressed_(false),
    rgb_(false),
    use_transform_(false),
    save_on_shutdown_(true),
    fixed_frame_(""),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    save_triggered_(false)
  {
    // Declare parameters
    this->declare_parameter<std::string>("prefix", "combined_");
    this->declare_parameter<std::string>("fixed_frame", "");
    this->declare_parameter<bool>("binary", false);
    this->declare_parameter<bool>("compressed", false);
    this->declare_parameter<bool>("rgb", false);
    this->declare_parameter<bool>("save_on_shutdown", true);
    this->declare_parameter<double>("save_timer_sec", 0.0);  // 0.0 = disabled

    // Retrieve parameter values
    prefix_ = this->get_parameter("prefix").as_string();
    fixed_frame_ = this->get_parameter("fixed_frame").as_string();
    binary_ = this->get_parameter("binary").as_bool();
    compressed_ = this->get_parameter("compressed").as_bool();
    rgb_ = this->get_parameter("rgb").as_bool();
    save_on_shutdown_ = this->get_parameter("save_on_shutdown").as_bool();
    double save_timer_sec = this->get_parameter("save_timer_sec").as_double();

    RCLCPP_INFO(this->get_logger(), "prefix: %s", prefix_.c_str());
    RCLCPP_INFO(this->get_logger(), "fixed_frame: %s", fixed_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "binary: %s", binary_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "compressed: %s", compressed_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "rgb: %s", rgb_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "save_on_shutdown: %s", save_on_shutdown_ ? "true" : "false");
    if (save_timer_sec > 0.0) {
      RCLCPP_INFO(
        this->get_logger(),
        "PCD file will be automatically saved after %.2f seconds (save_timer_sec).",
        save_timer_sec);
    } else {
      RCLCPP_INFO(
        this->get_logger(),
        "PCD file will not be automatically saved. Will be saved on the shutdown of the node.");
    }

    // Create a subscription with SensorDataQoS
    auto sensor_qos = rclcpp::SensorDataQoS();
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "input", sensor_qos,
      std::bind(&CombinedPointCloudToPCD::cloudCb, this, std::placeholders::_1));

    // Create a timer if the user wants a periodic check for saving
    if (save_timer_sec > 0.0) {
      save_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(save_timer_sec),
        std::bind(&CombinedPointCloudToPCD::checkAndSave, this));
    }

    RCLCPP_INFO(this->get_logger(), "Initialized CombinedPointCloudToPCD node");
  }

  ~CombinedPointCloudToPCD() override
  {
    // Optionally save on node shutdown
    if (!save_triggered_ && save_on_shutdown_) {
      RCLCPP_INFO(this->get_logger(), "Node is shutting down; saving accumulated cloud.");
      saveAccumulatedCloud();
    }
  }

private:
  // Parameters
  std::string prefix_;
  bool binary_;
  bool compressed_;
  bool rgb_;
  bool use_transform_;
  bool save_on_shutdown_;
  std::string fixed_frame_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr save_timer_;

  // Internal state
  pcl::PointCloud<pcl::PointXYZ> accumulated_cloud_xyz_;
  pcl::PointCloud<pcl::PointXYZRGB> accumulated_cloud_xyzrgb_;
  bool save_triggered_;

  /**
   * @brief Point cloud callback. Accumulates all incoming messages in an internal cloud.
   */
  void cloudCb(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
  {
    if (cloud_msg->data.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty point cloud message. Skipping.");
      return;
    }

    // Attempt transform if fixed_frame_ is set
    Eigen::Vector4f translation = Eigen::Vector4f::Zero();
    Eigen::Quaternionf rotation = Eigen::Quaternionf::Identity();
    use_transform_ = false;

    if (!fixed_frame_.empty()) {
      try {
        geometry_msgs::msg::TransformStamped transform =
          tf_buffer_.lookupTransform(
          fixed_frame_, cloud_msg->header.frame_id, cloud_msg->header.stamp);

        Eigen::Affine3d transform_eigen = tf2::transformToEigen(transform);
        translation.head<3>() = transform_eigen.translation().cast<float>();
        rotation = transform_eigen.rotation().cast<float>();
        use_transform_ = true;
      } catch (tf2::TransformException & ex) {
        RCLCPP_WARN(
          this->get_logger(), "Transform to frame '%s' failed: %s. Using original frame.",
          fixed_frame_.c_str(), ex.what());
        use_transform_ = false;
      }
    }

    // Convert ROS msg to PCL
    if (rgb_) {
      pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
      pcl::fromROSMsg(*cloud_msg, pcl_cloud);

      if (use_transform_) {
        transformPointCloud(pcl_cloud, translation, rotation);
      }
      // Accumulate
      accumulated_cloud_xyzrgb_ += pcl_cloud;
      RCLCPP_INFO(
        this->get_logger(), "Accumulated (XYZRGB) cloud size: %zu",
        accumulated_cloud_xyzrgb_.size());
    } else {
      pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
      pcl::fromROSMsg(*cloud_msg, pcl_cloud);

      if (use_transform_) {
        transformPointCloud(pcl_cloud, translation, rotation);
      }
      // Accumulate
      accumulated_cloud_xyz_ += pcl_cloud;
      RCLCPP_INFO(
        this->get_logger(), "Accumulated (XYZ) cloud size: %zu",
        accumulated_cloud_xyz_.size());
    }
  }

  /**
   * @brief Apply a rigid transform (translation + rotation) to an XYZ cloud in-place.
   */
  void transformPointCloud(
    pcl::PointCloud<pcl::PointXYZ> & cloud_inout,
    const Eigen::Vector4f & translation,
    const Eigen::Quaternionf & rotation)
  {
    for (auto & pt : cloud_inout.points) {
      Eigen::Vector3f v = rotation * pt.getVector3fMap() + translation.head<3>();
      pt.x = v.x();
      pt.y = v.y();
      pt.z = v.z();
    }
  }

  /**
   * @brief Apply a rigid transform (translation + rotation) to an XYZRGB cloud in-place.
   */
  void transformPointCloud(
    pcl::PointCloud<pcl::PointXYZRGB> & cloud_inout,
    const Eigen::Vector4f & translation,
    const Eigen::Quaternionf & rotation)
  {
    for (auto & pt : cloud_inout.points) {
      Eigen::Vector3f v = rotation * pt.getVector3fMap() + translation.head<3>();
      pt.x = v.x();
      pt.y = v.y();
      pt.z = v.z();
    }
  }

  /**
   * @brief Timer-based or event-based function to check if we need to save the cloud.
   */
  void checkAndSave()
  {
    if (!save_triggered_) {
      saveAccumulatedCloud();
    }
  }

  /**
   * @brief Writes the accumulated point cloud to disk in a single PCD file.
   */
  void saveAccumulatedCloud()
  {
    if (save_triggered_) {
      return;
    }
    save_triggered_ = true;  // Prevent multiple saves

    // Create a filename
    std::stringstream ss;
    ss << prefix_ << "combined_" << this->now().seconds() << ".pcd";
    std::string filename = ss.str();

    RCLCPP_INFO(this->get_logger(), "Saving accumulated point cloud to: %s", filename.c_str());

    pcl::PCDWriter writer;

    if (rgb_) {
      if (accumulated_cloud_xyzrgb_.empty()) {
        RCLCPP_WARN(this->get_logger(), "No points in accumulated XYZRGB cloud to save.");
      } else {
        if (binary_) {
          if (compressed_) {
            writer.writeBinaryCompressed(filename, accumulated_cloud_xyzrgb_);
          } else {
            writer.writeBinary(filename, accumulated_cloud_xyzrgb_);
          }
        } else {
          writer.writeASCII(filename, accumulated_cloud_xyzrgb_, 8);
        }
        RCLCPP_INFO(
          this->get_logger(),
          "Saved %zu XYZRGB points to %s",
          accumulated_cloud_xyzrgb_.size(),
          filename.c_str()
        );
      }
    } else {
      if (accumulated_cloud_xyz_.empty()) {
        RCLCPP_WARN(this->get_logger(), "No points in accumulated XYZ cloud to save.");
      } else {
        if (binary_) {
          if (compressed_) {
            writer.writeBinaryCompressed(filename, accumulated_cloud_xyz_);
          } else {
            writer.writeBinary(filename, accumulated_cloud_xyz_);
          }
        } else {
          writer.writeASCII(filename, accumulated_cloud_xyz_, 8);
        }
        RCLCPP_INFO(
          this->get_logger(),
          "Saved %zu XYZ points to %s",
          accumulated_cloud_xyz_.size(),
          filename.c_str()
        );
      }
    }
    // Shut down the node after saving the cloud.
    RCLCPP_INFO(this->get_logger(), "Shutting down the node.");
    rclcpp::shutdown();
  }
};

}  // namespace pcl_ros

// Register as a component
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_ros::CombinedPointCloudToPCD)
