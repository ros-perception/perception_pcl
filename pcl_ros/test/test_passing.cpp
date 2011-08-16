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
 * $Id: test_passing.cpp 33304 2010-10-15 00:09:25Z rusu $
 *
 */

#include <boost/thread.hpp>
#include <pluginlib/class_list_macros.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/pcl_nodelet.h>

namespace pcl_ros
{
  class TestTalker: public PCLNodelet
  {
    public:
      TestTalker ()
      {
        // Generate the data
        pcl::PointCloud<pcl::PointXYZ> cloud;

        cloud.width  = 640;
        cloud.height = 480;
        cloud.points.resize (cloud.width * cloud.height);
        cloud.is_dense = true;

        srand (time (NULL));
        size_t nr_p = cloud.points.size ();
        // Randomly create a new point cloud
        for (size_t i = 0; i < nr_p; ++i)
        {
          cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0);
          cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0);
          cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0);
        }

        // Convert from data type to blob
        pcl::toROSMsg (cloud, cloud_blob_);
        cloud_blob_ptr_ = boost::make_shared<sensor_msgs::PointCloud2> (cloud_blob_);
      }

    protected:
      sensor_msgs::PointCloud2 cloud_blob_;
      sensor_msgs::PointCloud2::Ptr cloud_blob_ptr_;
      ros::Publisher pub_output_;

      virtual void
        onInit ()
      {
        ros::NodeHandle private_nh = this->getMTPrivateNodeHandle ();
        pub_output_ = private_nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

        NODELET_INFO ("[onInit] Data initialized. Starting to publish...");
        ros::Duration d (2), d2 (0.001);
        d.sleep ();
        for (size_t i = 0; i < 1000; ++i)
        {
          NODELET_INFO ("[onInit] Publishing... %d", (int)i);
          pub_output_.publish (cloud_blob_ptr_);
          d2.sleep ();
        }
      }
  };

  class TestListener: public PCLNodelet
  {
    public:
      TestListener () {};

    protected:
      ros::Subscriber sub_input_;

      virtual void
        onInit ()
      {
        ros::NodeHandle private_nh = this->getMTPrivateNodeHandle ();
        sub_input_ = private_nh.subscribe ("input", 1, &TestListener::input_callback, this);
        NODELET_INFO ("[onInit] Waiting for data..."); 
      }

      void
        input_callback (const sensor_msgs::PointCloud2ConstPtr &cloud)
      {
        NODELET_INFO ("[input_callback] PointCloud with %d data points and frame %s on topic %s received.", cloud->width * cloud->height, cloud->header.frame_id.c_str (), this->getMTPrivateNodeHandle ().resolveName ("input").c_str ());
      }
  };

  class TestPingPong: public PCLNodelet
  {
    public:
      TestPingPong () : msg_count_ (0), byte_count_ (0)
      {
        // Generate the data
        pcl::PointCloud<pcl::PointXYZ> cloud;

        cloud.width  = 640;
        cloud.height = 480;
        cloud.points.resize (cloud.width * cloud.height);
        cloud.is_dense = true;

        srand (time (NULL));
        size_t nr_p = cloud.points.size ();
        // Randomly create a new point cloud
        for (size_t i = 0; i < nr_p; ++i)
        {
          cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0);
          cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0);
          cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0);
        }

        // Convert from data type to blob
        pcl::toROSMsg (cloud, cloud_blob_);
        cloud_blob_ptr_ = boost::make_shared<sensor_msgs::PointCloud2> (cloud_blob_);
      }
    
      virtual ~TestPingPong ()
      {
        t_end_ = ros::WallTime::now ();
        ROS_INFO ("Sent %d messages (%ld bytes) in %f seconds (%d msg/s).", msg_count_, byte_count_, (t_end_ - t_start_).toSec (), (int)(msg_count_ / (t_end_ - t_start_).toSec ()));
      }

    protected:
      sensor_msgs::PointCloud2 cloud_blob_;
      sensor_msgs::PointCloud2::Ptr cloud_blob_ptr_;
      ros::Subscriber sub_input_;
      ros::Publisher pub_output_;
      int msg_count_, total_msgs_;
      long int byte_count_;
      ros::WallTime t_start_, t_end_;

      virtual void
        onInit ()
      {
        ros::NodeHandle private_nh = this->getMTPrivateNodeHandle ();

        private_nh.getParam ("total_msgs", total_msgs_);

        sub_input_ = private_nh.subscribe ("input", 1, &TestPingPong::input_callback, this);
        pub_output_ = private_nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
        NODELET_INFO ("[onInit] %d threads should be available.", boost::thread::hardware_concurrency ());
        ros::Duration (1).sleep ();
        pub_output_.publish (cloud_blob_ptr_);
        t_start_ = ros::WallTime::now ();
      }

      void
        input_callback (const sensor_msgs::PointCloud2ConstPtr &cloud)
      {
        if (msg_count_ >= total_msgs_)
          ros::shutdown ();

        pub_output_.publish (cloud);
        msg_count_++;
        byte_count_ += cloud->data.size ();
      }
  };
}

typedef pcl_ros::TestTalker TestTalker;
typedef pcl_ros::TestListener TestListener;
typedef pcl_ros::TestPingPong TestPingPong;
PLUGINLIB_DECLARE_CLASS (pcl, TestTalker, TestTalker, nodelet::Nodelet);
PLUGINLIB_DECLARE_CLASS (pcl, TestListener, TestListener, nodelet::Nodelet);
PLUGINLIB_DECLARE_CLASS (pcl, TestPingPong, TestPingPong, nodelet::Nodelet);

