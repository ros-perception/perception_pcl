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
 * $Id: pcd_to_pointcloud.cpp 33238 2010-03-11 00:46:58Z rusu $
 *
 */

/**

\author Radu Bogdan Rusu

@b pcd_to_pointcloud is a simple node that loads PCD (Point Cloud Data) files from disk and publishes them as ROS messages on the network.

 **/

#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <string>
#include <sstream>

// load pointcloud from file
bool init_pointcloud(std::string const& fname,
                     sensor_msgs::PointCloud2& cloud,
                     std::string const& frame_id)
{
    if (pcl::io::loadPCDFile(fname, cloud) == -1) {
        // failed to load
        return false;
    }
    cloud.header.frame_id = frame_id;
    return true;
}

std::string cloud_topic = "cloud_pcd";

// pointcloud message and publisher
sensor_msgs::PointCloud2 cloud;
ros::Publisher pub;

void publish() {
    ROS_DEBUG_STREAM_ONCE("Publishing pointcloud");
    ROS_DEBUG_STREAM_ONCE(" * number of points: " << cloud.width * cloud.height);
    ROS_DEBUG_STREAM_ONCE(" * frame_id: " << cloud.header.frame_id);
    ROS_DEBUG_STREAM_ONCE(" * topic_name: " << cloud_topic);
    int num_subscribers = pub.getNumSubscribers();
    if (num_subscribers > 0) {
        ROS_DEBUG("Publishing data to %d subscribers.", num_subscribers);
    }
    // update timestamp and publish
    cloud.header.stamp = ros::Time::now();
    pub.publish(cloud);
}

void timer_callback(ros::TimerEvent const&) {
    // just re-publish
    publish();
}

int main (int argc, char** argv) {
    // init ROS
    ros::init(argc, argv, "pcd_to_pointcloud");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    pub = nh.advertise<sensor_msgs::PointCloud2>(cloud_topic, 1, true);
    // update potentially remapped topic name for logging
    cloud_topic = nh.resolveName(cloud_topic);
    // filename
    std::string file_name;
    // republish interval in seconds
    double interval;
    // tf2 frame_id
    std::string frame_id;
    // try to parse config from parameters
    private_nh.param<double>("interval", interval, 0);
    private_nh.param<std::string>("file_name", file_name, "");
    private_nh.param<std::string>("frame_id", frame_id, "base_link");
    // try to parse config from command line
    // command line args take precedence
    if (argc > 1) {
        file_name = argv[1];
    }
    else if (file_name.empty()) {
        ROS_ERROR_STREAM("Usage: " << argv[0] << " <file.pcd> [publishing_interval (in seconds)]");
        return -1;
    }
    if (argc > 2) {
        std::stringstream str(argv[2]);
        double x;
        if (str >> x)
            interval = x;
    }
    // dump parameters fyi
    ROS_INFO_STREAM("Recognized the following parameters");
    ROS_INFO_STREAM(" * file_name: " << file_name);
    ROS_INFO_STREAM(" * interval: " << interval);
    ROS_INFO_STREAM(" * frame_id: " << frame_id);
    ROS_INFO_STREAM(" * topic_name: " << cloud_topic);
    // try to load pointcloud from file
    if (!init_pointcloud(file_name, cloud, frame_id)) {
        ROS_ERROR_STREAM("Failed to parse pointcloud from file ('" << file_name << "')");
        return -1;
    }
    ROS_INFO_STREAM("Loaded pointcloud with the following stats");
    ROS_INFO_STREAM(" * number of points: " << cloud.width * cloud.height);
    ROS_INFO_STREAM(" * total size [bytes]: " << cloud.data.size());
    ROS_INFO_STREAM(" * channel names: " << pcl::getFieldsList(cloud));
    // publish
    publish();
    // re-publish as configured
    ros::Timer timer;
    if (interval > 0) {
        timer = nh.createTimer(ros::Duration(interval), timer_callback);
    }
    ros::spin();
}