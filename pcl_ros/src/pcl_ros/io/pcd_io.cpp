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
 * $Id: pcd_io.cpp 35812 2011-02-08 00:05:03Z rusu $
 *
 */

//#include <pluginlib/class_list_macros.h>
#include <pcl_ros/io/pcd_io.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl_ros::PCDReader::PCDReader (std::string node_name, const rclcpp::NodeOptions& node_options) : PCLNode(node_name, options)
{
  // Provide a latched topic
  auto pub_output = this->create_publisher<PointCloud2> ("output", max_queue_size_, true);

  this->get_parameter ("publish_rate", publish_rate_);
  this->get_parameter ("tf_frame", tf_frame_);

  RCLCPP_DEBUG (this->get_logger(), "[%s::onConstructor] Node successfully created with the following parameters:\n"
                 " - publish_rate : %f\n"
                 " - tf_frame     : %s",
                 this->get_name (),
                 publish_rate_, tf_frame_.c_str ());

  PointCloud2Ptr output_new;
  output_ = std::make_shared<PointCloud2> ();
  output_new = std::make_shared<PointCloud2> ();

  // Wait in a loop until someone connects
  do
  {
    RCLCPP_DEBUG (this->get_logger(), "[%s::onConstructor] Waiting for a client to connect...", this->get_name ());
    rclcpp::spinOnce ();
    rclcpp::Duration (0.01).sleep ();
  }
  while (this->ok () && pub_output.count_subscribers () == 0);

  std::string file_name;

  while (this->ok ())
  {
    // Get the current filename parameter. If no filename set, loop
    if (!this->get_parameter ("filename", file_name_) && file_name_.empty ())
    {
      RCLCPP_ERROR (this->get_logger(), "[%s::onConstructor] Need a 'filename' parameter to be set before continuing!", this->get_name ());
      rclcpp::Duration (0.01).sleep ();
      rclcpp::spinOnce ();
      continue;
    }

    // If the filename parameter holds a different value than the last one we read
    if (file_name_.compare (file_name) != 0 && !file_name_.empty ())
    {
      RCLCPP_INFO (this->get_logger(), "[%s::onConstructor] New file given: %s", this->get_name (), file_name_.c_str ());
      file_name = file_name_;
      pcl::PCLPointCloud2 cloud;
      if (impl_.read (file_name_, cloud) < 0)
      {
        RCLCPP_ERROR (this->get_logger(), "[%s::onConstructor] Error reading %s !", this->get_name (), file_name_.c_str ());
        return;
      }
      pcl_conversions::moveFromPCL(cloud, *(output_));
      output_->header.stamp    = this->now ();
      output_->header.frame_id = tf_frame_;
    }

    // We do not publish empty data
    if (output_->data.size () == 0)
      continue;

    if (publish_rate_ == 0)
    {
      if (output_ != output_new)
      {
        RCLCPP_DEBUG (this->get_logger(), "Publishing data once (%d points) on topic %s in frame %s.", output_->width * output_->height, "output", output_->header.frame_id.c_str ());
        pub_output.publish (output_);
        output_new = output_;
      }
      ros::Duration (0.01).sleep ();
    }
    else
    {
      RCLCPP_DEBUG (this->get_logger(), "Publishing data (%d points) on topic %s in frame %s.", output_->width * output_->height, "output", output_->header.frame_id.c_str ());
      output_->header.stamp = this->now ();
      pub_output->publish (output_);

      rclcpp::Duration (publish_rate_).sleep ();
    }

    rclcpp::spinOnce ();
    // Update parameters from server
    this->get_parameter ("publish_rate", publish_rate_);
    this->get_parameter ("tf_frame", tf_frame_);
  }

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl_ros::PCDWriter() : PCLNode()
{

  sub_input_ = this->create_subscription("input", std::bind(&PCDWriter::input_callback, this),  1);
  // ---[ Optional parameters
  this->get_parameter ("filename", file_name_);
  this->get_parameter ("binary_mode", binary_mode_);

  RCLCPP_DEBUG (this->get_logger(), "[%s::onConstructor] Node successfully created with the following parameters:\n"
                 " - filename     : %s\n"
                 " - binary_mode  : %s", 
                 this->get_name (),
                 file_name_.c_str (), (binary_mode_) ? "true" : "false");

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::PCDWriter::input_callback (const PointCloud2ConstPtr &cloud)
{
  if (!isValid (cloud))
    return;
  
  this->get_parameter ("filename", file_name_);

  RCLCPP_DEBUG (this->get_logger(), "[%s::input_callback] PointCloud with %d data points and frame %s on topic %s received.", this->get_name (), cloud->width * cloud->height, cloud->header.frame_id.c_str (), "input");
 
  std::string fname;
  if (file_name_.empty ())
    fname = std::lexical_cast<std::string> (cloud->header.stamp.sec) + ".pcd";
  else
    fname = file_name_;
  pcl::PCLPointCloud2 pcl_cloud;
  // It is safe to remove the const here because we are the only subscriber callback.
  pcl_conversions::moveToPCL(*(const_cast<PointCloud2*>(cloud.get())), pcl_cloud);
  impl_.write (fname, pcl_cloud, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), binary_mode_);

  RCLCPP_DEBUG (this->get_logger(), "[%s::input_callback] Data saved to %s", this->get_name (), fname.c_str ());
}

typedef pcl_ros::PCDReader PCDReader;
typedef pcl_ros::PCDWriter PCDWriter;
//(PCDReader,nodelet::Nodelet);
//PLUGINLIB_EXPORT_CLASS(PCDWriter,nodelet::Nodelet);

