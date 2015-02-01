/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 *
 */

/*
 * Author: Paul Bovbel
 */

#include <pointcloud_to_laserscan/pointcloud_to_laserscan_nodelet.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl_ros/transforms.h>
#include <math.h>
#include <pluginlib/class_list_macros.h>

namespace pointcloud_to_laserscan
{

  void PointCloudToLaserScanNodelet::onInit()
  {
      nh_ = getMTNodeHandle();
      private_nh_ = getMTPrivateNodeHandle();

      private_nh_.param<std::string>("target_frame", target_frame_, "");
      private_nh_.param<double>("min_height", min_height_, 0.0);
      private_nh_.param<double>("max_height", max_height_, 1.0);

      private_nh_.param<double>("angle_min", angle_min_, -M_PI/2.0);
      private_nh_.param<double>("angle_max", angle_max_, M_PI/2.0);
      private_nh_.param<double>("angle_increment", angle_increment_, M_PI/360.0);
      private_nh_.param<double>("scan_time", scan_time_, 1.0/30.0);
      private_nh_.param<double>("range_min", range_min_, 0.45);
      private_nh_.param<double>("range_max", range_max_, 4.0);

      int concurrency_level;
      private_nh_.param<int>("concurrency_level", concurrency_level, 1);
      private_nh_.param<bool>("use_inf", use_inf_, true);

      boost::mutex::scoped_lock lock(connect_mutex_);

      // Only queue one pointcloud per running thread
      if(concurrency_level > 0)
      {
          input_queue_size_ = concurrency_level;
      }else{
          input_queue_size_ = boost::thread::hardware_concurrency();
      }

      pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 10,
          boost::bind(&PointCloudToLaserScanNodelet::connectCb, this),
          boost::bind(&PointCloudToLaserScanNodelet::disconnectCb, this));
  }

  void PointCloudToLaserScanNodelet::connectCb()
  {
      boost::mutex::scoped_lock lock(connect_mutex_);
      if (!sub_ && pub_.getNumSubscribers() > 0) {
          NODELET_DEBUG("Connecting to depth topic.");
          sub_ = nh_.subscribe("cloud_in", input_queue_size_, &PointCloudToLaserScanNodelet::cloudCb, this);
      }
  }

  void PointCloudToLaserScanNodelet::disconnectCb()
  {
      boost::mutex::scoped_lock lock(connect_mutex_);
      if (pub_.getNumSubscribers() == 0) {
          NODELET_DEBUG("Unsubscribing from depth topic.");
          sub_.shutdown();
      }
  }

  void PointCloudToLaserScanNodelet::cloudCb(const PointCloud::ConstPtr &cloud_msg)
  {
      //pointer to pointcloud data to transform to laserscan
      PointCloud::ConstPtr cloud_scan;

      std_msgs::Header cloud_header = pcl_conversions::fromPCL(cloud_msg->header);

      //build laserscan output
      sensor_msgs::LaserScan output;
      output.header = cloud_header;
      output.angle_min = angle_min_;
      output.angle_max = angle_max_;
      output.angle_increment = angle_increment_;
      output.time_increment = 0.0;
      output.scan_time = scan_time_;
      output.range_min = range_min_;
      output.range_max = range_max_;

      //decide if pointcloud needs to be transformed to a target frame
      if(!target_frame_.empty() && cloud_header.frame_id != target_frame_){
          output.header.frame_id = target_frame_;

          if(tf_.waitForTransform(cloud_header.frame_id, target_frame_, cloud_header.stamp, ros::Duration(10.0))){
              PointCloud::Ptr cloud_tf(new PointCloud);
              pcl_ros::transformPointCloud(target_frame_, *cloud_msg, *cloud_tf, tf_);
              cloud_scan = cloud_tf;
          }else{
              NODELET_WARN_STREAM_THROTTLE(1.0, "Can't transform cloud with frame " << cloud_header.frame_id << " into "
                  "lasercan with frame " << target_frame_);
              return;
          }
      }else{
          cloud_scan = cloud_msg;
      }

      //determine amount of rays to create
      uint32_t ranges_size = std::ceil((output.angle_max - output.angle_min) / output.angle_increment);

      //determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
      if(use_inf_){
          output.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
      }else{
          output.ranges.assign(ranges_size, output.range_max + 1.0);
      }

      for (PointCloud::const_iterator it = cloud_scan->begin(); it != cloud_scan->end(); ++it){
          const float &x = it->x;
          const float &y = it->y;
          const float &z = it->z;

          if ( std::isnan(x) || std::isnan(y) || std::isnan(z) ){
              NODELET_DEBUG("rejected for nan in point(%f, %f, %f)\n", x, y, z);
              continue;
          }

          if (z > max_height_ || z < min_height_){
              NODELET_DEBUG("rejected for height %f not in range (%f, %f)\n", z, min_height_, max_height_);
              continue;
          }

          double range = hypot(x,y);
          if (range < range_min_){
              NODELET_DEBUG("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range, range_min_, x, y, z);
              continue;
          }

          double angle = atan2(y, x);
          if (angle < output.angle_min || angle > output.angle_max){
              NODELET_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, output.angle_min, output.angle_max);
              continue;
          }

          //overwrite range at laserscan ray if new range is smaller
          int index = (angle - output.angle_min) / output.angle_increment;
          if (range < output.ranges[index]){
              output.ranges[index] = range;
          }
      }
      pub_.publish(output);
  }

}

PLUGINLIB_DECLARE_CLASS(pointcloud_to_laserscan, PointCloudToLaserScanNodelet, pointcloud_to_laserscan::PointCloudToLaserScanNodelet, nodelet::Nodelet);

