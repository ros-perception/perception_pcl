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
 * $Id: pointcloud_to_pcd.cpp 33238 2010-03-11 00:46:58Z rusu $
 *
 */

// ROS core
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

// PCL includes
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Geometry>

#include <pcl_ros/tf_helper.hpp>

/**
\author Radu Bogdan Rusu

@b pointcloud_to_pcd is a simple node that retrieves a ROS point cloud message and saves it to disk into a PCD (Point
Cloud Data) file format.

**/
class PointCloudToPCD : public rclcpp::Node
{
  private:
    std::string prefix_;
    bool binary_;
    bool compressed_;
    std::string fixed_frame_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

  public:
    std::string cloud_topic_;
  
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;

    ////////////////////////////////////////////////////////////////////////////////
    // Callback
    void
      cloud_cb (sensor_msgs::msg::PointCloud2::ConstSharedPtr ros_cloud)
    {
      auto cloud = std::make_shared<pcl::PCLPointCloud2>();
      pcl_conversions::toPCL(*ros_cloud, *cloud);

      if ((cloud->width * cloud->height) == 0)
        return;

      RCLCPP_INFO (this->get_logger(), "Received %d data points in frame %s with the following fields: %s",
                (int)cloud->width * cloud->height,
                cloud->header.frame_id.c_str (),
                pcl::getFieldsList (*cloud).c_str ());

      Eigen::Vector4f v = Eigen::Vector4f::Zero ();
      Eigen::Quaternionf q = Eigen::Quaternionf::Identity ();
      if (!fixed_frame_.empty ()) {
        if (!tf_buffer_.canTransform(fixed_frame_, cloud->header.frame_id,
                                     pcl_ros::tfFromRclcpp(pcl_conversions::fromPCL(cloud->header.stamp)),
                                     pcl_ros::tfFromRclcpp(rclcpp::Duration(3))))
        {
          RCLCPP_WARN(this->get_logger(), "Could not get transform!");
          return;
        }

        Eigen::Affine3d transform;
        transform = tf2::transformToEigen(
            tf_buffer_.lookupTransform(fixed_frame_, cloud->header.frame_id,
                                       pcl_ros::tfFromRclcpp(pcl_conversions::fromPCL(cloud->header.stamp))));
        v = Eigen::Vector4f::Zero ();
        v.head<3> () = transform.translation ().cast<float> ();
        q = transform.rotation ().cast<float> ();
      }

      std::stringstream ss;
      ss << prefix_ << cloud->header.stamp << ".pcd";
      RCLCPP_INFO (this->get_logger(), "Data saved to %s", ss.str ().c_str ());

      pcl::PCDWriter writer;
      if(binary_)
	{
	  if(compressed_)
	    {
	      writer.writeBinaryCompressed (ss.str (), *cloud, v, q);
	    }
	  else
	    {
	      writer.writeBinary (ss.str (), *cloud, v, q);
	    }
	}
      else
	{
	  writer.writeASCII (ss.str (), *cloud, v, q, 8);
	}

    }

  ////////////////////////////////////////////////////////////////////////////////
  PointCloudToPCD (const rclcpp::NodeOptions& options)
    : rclcpp::Node("pointcloud_to_pcd", options),
      binary_(false),
      compressed_(false),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
  {
    // Check if a prefix parameter is defined for output file names.
    if (this->get_parameter ("prefix", prefix_))
    {
      RCLCPP_INFO (this->get_logger(), "PCD file prefix is: %s", prefix_.c_str());
    }
    else if (this->get_parameter ("prefix", prefix_))
    {
      RCLCPP_WARN (this->get_logger(), "Non-private PCD prefix parameter is DEPRECATED: %s"
                       , prefix_.c_str());
    }
    
    this->get_parameter ("fixed_frame", fixed_frame_);
    this->get_parameter ("binary", binary_);
    this->get_parameter ("compressed", compressed_);
    if(binary_)
    {
      if(compressed_)
      {
        RCLCPP_INFO (this->get_logger(), "Saving as binary compressed PCD");
      }
      else
      {
        RCLCPP_INFO (this->get_logger(), "Saving as binary PCD");
      }
    }
    else
    {
      RCLCPP_INFO (this->get_logger(), "Saving as binary PCD");
    }
    
    cloud_topic_ = "input";
    sub_ = rclcpp::create_subscription<sensor_msgs::msg::PointCloud2>(
      this, cloud_topic_,  rclcpp::QoS(rclcpp::KeepLast(1)),
      std::bind(&PointCloudToPCD::cloud_cb, this, std::placeholders::_1));
    RCLCPP_INFO (this->get_logger(), "Listening for incoming data on topic %s",
                 cloud_topic_.c_str ());
  }
};

/* ---[ */
int
  main (int argc, char** argv)
{
  rclcpp::init (argc, argv);

  rclcpp::NodeOptions options;
  auto b = std::make_shared<PointCloudToPCD> (options);
  rclcpp::spin (b);
  rclcpp::shutdown();
  return (0);
}
/* ]--- */
