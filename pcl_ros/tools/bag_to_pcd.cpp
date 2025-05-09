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
 * $Id: bag_to_pcd.cpp 35812 2011-02-08 00:05:03Z rusu $
 *
 */

/**

\author Radu Bogdan Rusu

@b bag_to_pcd is a simple node that reads in a BAG file and saves all the PointCloud messages to disk in PCD (Point
Cloud Data) format.

 **/

#include <pcl/common/io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>

#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rosbag2_transport/reader_writer_factory.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>


using namespace std::chrono_literals;

namespace pcl_ros
{
class BagToPCD : public rclcpp::Node
{
public:
  explicit BagToPCD(const rclcpp::NodeOptions & options)
  : rclcpp::Node("bag_to_pcd", options)
  {
    bag_path_ = this->declare_parameter<std::string>("bag_path", "");
    topic_name_ = this->declare_parameter<std::string>("topic_name", "");
    output_directory_ = this->declare_parameter<std::string>("output_directory", "");

    if (bag_path_.empty() || topic_name_.empty() || output_directory_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Required parameter not set.");
      RCLCPP_ERROR(this->get_logger(),
                   "Example: ros2 run pcl_ros bag_to_pcd --ros-args "
                   "-p bag_path:=rosbag2_2025_01_01/ "
                   "-p topic_name:=/pointcloud "
                   "-p output_directory:=pcds");
      throw std::runtime_error{"Required parameter not set."};
    }

    timer_ = this->create_wall_timer(100ms, [this]() {return this->timer_callback();});

    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_path_;
    reader_ = rosbag2_transport::ReaderWriterFactory::make_reader(storage_options);
    reader_->open(storage_options);
  }

private:
  void timer_callback()
  {
    while (reader_->has_next()) {
      rosbag2_storage::SerializedBagMessageSharedPtr msg = reader_->read_next();
      if (msg->topic_name != topic_name_) {
        continue;
      }

      rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
      sensor_msgs::msg::PointCloud2 pointcloud_msg;
      serialization_.deserialize_message(&serialized_msg, &pointcloud_msg);

      pcl::PCLPointCloud2 cloud;
      pcl_conversions::moveToPCL(pointcloud_msg, cloud);

      std::stringstream ss;
      ss << output_directory_ << "/" << msg->recv_timestamp << ".pcd";
      RCLCPP_INFO(this->get_logger(), "Writing to: %s", ss.str().c_str());
      pcl::io::savePCDFile(ss.str(), cloud);
      break;
    }
  }
  std::string bag_path_;
  std::string topic_name_;
  std::string output_directory_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<rosbag2_cpp::Reader> reader_;
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization_;
};
}  // namespace pcl_ros


RCLCPP_COMPONENTS_REGISTER_NODE(pcl_ros::BagToPCD)
