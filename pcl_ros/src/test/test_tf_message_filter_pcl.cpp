/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Brice Rebsamen
 * Copied and adapted from geometry/test_message_filter.cpp
 */

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

using namespace tf2_ros;

// using a random point type, as we want to make sure that it does work with
// other points than just XYZ
typedef pcl::PointCloud<pcl::PointXYZRGBNormal> PCDType;


/// Sets pcl_stamp from stamp, BUT alters stamp
/// a little to round it to millisecond. This is because converting back
/// and forth from pcd to ros time induces some rounding errors.
void setStamp(rclcpp::Time &stamp, pcl::uint64_t &pcl_stamp)
{
  // round to millisecond
  static const uint32_t mult = 1e6;
  stamp = rclcpp::Time(stamp.nanoseconds() / mult);
  stamp = rclcpp::Time(stamp.nanoseconds() * mult);

  pcl_conversions::toPCL(stamp, pcl_stamp);

  // verify
  {
    rclcpp::Time t;
    pcl_conversions::fromPCL(pcl_stamp, t);
    //ROS_ASSERT_MSG(t==stamp, "%d/%d vs %d/%d", t.seconds(), t.nanoseconds(), stamp.sec, stamp.nanosec);
  }
}

class Notification
{
public:
  Notification(int expected_count)
  : count_(0)
  , expected_count_(expected_count)
  , failure_count_(0)
  {
  }

  void notify(const PCDType::ConstPtr& message)
  {
    ++count_;
  }

  void failure(const PCDType::ConstPtr& message, FilterFailureReason reason)
  {
    ++failure_count_;
  }

  int count_;
  int expected_count_;
  int failure_count_;
};

/*
TEST(MessageFilter, noTransforms)
{
  std::shared_ptr<tf2_ros::TransformListener> tf_client;
  Notification n(1);
  MessageFilter<PCDType> filter(tf_client, "frame1", 1);
  filter.registerCallback(std::bind(&Notification::notify, &n, _1));

  PCDType::Ptr msg(new PCDType);
  auto clock_ = std::make_shared<rclcpp::Clock>();
  rclcpp::Time stamp = clock_->now();
  setStamp(stamp, msg->header.stamp);
  msg->header.frame_id = "frame2";
  filter.add(msg);

  EXPECT_EQ(0, n.count_);
}
*/

/*
TEST(MessageFilter, noTransformsSameFrame)
{
  std::shared_ptr<tf2_ros::TransformListener> tf_client;
  Notification n(1);
  MessageFilter<PCDType> filter(tf_client, "frame1", 1);
  filter.registerCallback(std::bind(&Notification::notify, &n, _1));

  PCDType::Ptr msg(new PCDType);
  auto clock_ = std::make_shared<rclcpp::Clock>();
  rclcpp::Time stamp = clock_->now();
  setStamp(stamp, msg->header.stamp);
  msg->header.frame_id = "frame1";
  filter.add(msg);

  EXPECT_EQ(1, n.count_);
}
*/
/*
TEST(MessageFilter, preexistingTransforms)
{
  std::shared_ptr<tf2_ros::TransformListener> tf_client;
  Notification n(1);
  MessageFilter<PCDType> filter(tf_client, "frame1", 1);
  filter.registerCallback(std::bind(&Notification::notify, &n, _1));

  PCDType::Ptr msg(new PCDType);
  
  auto clock_ = std::make_shared<rclcpp::Clock>();
  rclcpp::Time stamp = clock_->now();
  setStamp(stamp, msg->header.stamp);

  geometry_msgs::msg::TransformStamped transform(tf2::Transform(tf2::Quaternion(0,0,0,1), tf2::Vector3(1,2,3)), stamp, "frame1", "frame2");
  tf_client.setTransform(transform);

  msg->header.frame_id = "frame2";
  filter.add(msg);

  EXPECT_EQ(1, n.count_);
}
*/
/*
TEST(MessageFilter, postTransforms)
{
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_client(tf_buffer_);
  Notification n(1);
  MessageFilter<PCDType> filter(tf_client, "frame1", 1);
  filter.registerCallback(std::bind(&Notification::notify, &n, _1));
  
  auto clock_ = std::make_shared<rclcpp::Clock>();
  rclcpp::Time stamp = clock_->now();
  PCDType::Ptr msg(new PCDType);
  setStamp(stamp, msg->header.stamp);
  msg->header.frame_id = "frame2";

  filter.add(msg);

  EXPECT_EQ(0, n.count_);

  geometry_msgs::msg::TransformStamped transform(tf2::Transform(tf2::Quaternion(0,0,0,1), tf2::Vector3(1,2,3)), stamp, "frame1", "frame2");
  tf_client.setTransform(transform);

  rclcpp::WallDuration(0.1).sleep();
  rclcpp::spinOnce();

  EXPECT_EQ(1, n.count_);
}
*/
/*
TEST(MessageFilter, queueSize)
{
  std::shared_ptr<tf2_ros::TransformListener> tf_client;
  Notification n(10);
  MessageFilter<PCDType> filter(tf_client, "frame1", 10);
 filter.registerCallback(std::bind(&Notification::notify, &n, std::placeholders::_1));
  filter.registerFailureCallback(std::bind(&Notification::failure, &n, std::placeholders::_1, std::placeholders::_2));
  
  auto clock_ = std::make_shared<rclcpp::Clock>();
  rclcpp::Time stamp = clock_->now();
  pcl::uint64_t pcl_stamp;
  setStamp(stamp, pcl_stamp);

  for (int i = 0; i < 20; ++i)
  {
    PCDType::Ptr msg(new PCDType);
    msg->header.stamp = pcl_stamp;
    msg->header.frame_id = "frame2";

    filter.add(msg);
  }

  EXPECT_EQ(0, n.count_);
  EXPECT_EQ(10, n.failure_count_);

  geometry_msgs::msg::TransformStamped transform(tf2::Transform(tf2::Quaternion(0,0,0,1), tf2::Vector3(1,2,3)), stamp, "frame1", "frame2");
  tf_client.setTransform(transform);

  rclcpp::WallDuration(0.1).sleep();
  rclcpp::spinOnce();

  EXPECT_EQ(10, n.count_);
}
*/
/*
TEST(MessageFilter, setTargetFrame)
{
  std::shared_ptr<tf2_ros::TransformListener> tf_client;
  Notification n(1);
  MessageFilter<PCDType> filter(tf_client, "frame1", 1);
  filter.registerCallback(std::bind(&Notification::notify, &n, _1));
  filter.setTargetFrame("frame1000");
  
  auto clock_ = std::make_shared<rclcpp::Clock>();
  rclcpp::Time stamp = clock_->now();
  PCDType::Ptr msg(new PCDType);
  setStamp(stamp, msg->header.stamp);
  msg->header.frame_id = "frame2";

  geometry_msgs::msg::TransformStamped transform(tf2::Transform(tf2::Quaternion(0,0,0,1), tf2::Vector3(1,2,3)), stamp, "frame1000", "frame2");
  tf_client.setTransform(transform);

  filter.add(msg);


  EXPECT_EQ(1, n.count_);
}
*/
/*
TEST(MessageFilter, multipleTargetFrames)
{
  tf::TransformListener tf_client;
  Notification n(1);
  MessageFilter<PCDType> filter(tf_client, "", 1);
  filter.registerCallback(std::bind(&Notification::notify, &n, _1));

  std::vector<std::string> target_frames;
  target_frames.push_back("frame1");
  target_frames.push_back("frame2");
  filter.setTargetFrames(target_frames);
  
  auto clock_ = std::make_shared<rclcpp::Clock>();
  rclcpp::Time stamp = clock_->now();
  PCDType::Ptr msg(new PCDType);
  setStamp(stamp, msg->header.stamp);

  geometry_msgs::msg::TransformStamped transform(tf2_ros::Transform(tf2::Quaternion(0,0,0,1), tf2::Vector3(1,2,3)), stamp, "frame1", "frame3");
  tf_client.setTransform(transform);

  msg->header.frame_id = "frame3";
  filter.add(msg);

  rclcpp::WallDuration(0.1).sleep();
  rclcpp::spinOnce();

  EXPECT_EQ(0, n.count_); // frame1->frame3 exists, frame2->frame3 does not (yet)

  //rclcpp::Time::setNow(rclcpp::Time::now() + rclcpp::Duration(1.0));

  transform.child_frame_id_ = "frame2";
  tf_client.setTransform(transform);

  rclcpp::WallTimer(0.1).sleep();
  rclcpp::spinOnce();

  EXPECT_EQ(1, n.count_);  // frame2->frame3 now exists
}
*/
/*
TEST(MessageFilter, tolerance)
{
  rclcpp::Duration offset(0.2);
  tf2_ros::TransformListener tf_client;
  Notification n(1);
  MessageFilter<PCDType> filter(tf_client, "frame1", 1);
  filter.registerCallback(std::bind(&Notification::notify, &n, _1));
  filter.setTolerance(offset);
  
  auto clock_ = std::make_shared<rclcpp::Clock>();
  rclcpp::Time stamp = clock_->now();
  pcl::uint64_t pcl_stamp;
  setStamp(stamp, pcl_stamp);
  geometry_msgs::msg::TransformStamped transform(tf2::Transform(tf2::Quaternion(0,0,0,1), tf2::Vector3(1,2,3)), stamp, "frame1", "frame2");
  tf_client.setTransform(transform);

  PCDType::Ptr msg(new PCDType);
  msg->header.frame_id = "frame2";
  msg->header.stamp = pcl_stamp;
  filter.add(msg);

  EXPECT_EQ(0, n.count_); //No return due to lack of space for offset

  //rclcpp::Time::setNow(rclcpp::Time::now() + rclcpp::Duration(0.1));

  transform.stamp_ += offset*1.1;
  tf_client.setTransform(transform);

  rclcpp::WallTimer(0.1).sleep();
  rclcpp::spinOnce();

  EXPECT_EQ(1, n.count_); // Now have data for the message published earlier

  stamp += offset;
  setStamp(stamp, pcl_stamp);
  msg->header.stamp = pcl_stamp;

  filter.add(msg);

  EXPECT_EQ(1, n.count_); // Latest message is off the end of the offset
}
*/
/*
TEST(MessageFilter, outTheBackFailure)
{
  tf2_ros::TransformListener tf_client;
  Notification n(1);
  MessageFilter<PCDType> filter(tf_client, "frame1", 1);
  filter.registerFailureCallback(std::bind(&Notification::failure, &n, std::placeholders::_1, std::placeholders::_2));
  
  auto clock_ = std::make_shared<rclcpp::Clock>();
  rclcpp::Time stamp = clock_->now();
  PCDType::Ptr msg(new PCDType);
  setStamp(stamp, msg->header.stamp);

  geometry_msgs::msg::TransformStamped transform(tf2::Transform(tf2::Quaternion(0,0,0,1), tf2::Vector3(1,2,3)), stamp, "frame1", "frame2");
  tf_client.setTransform(transform);

  transform.stamp_ = stamp + ros::Duration(10000);
  tf_client.setTransform(transform);

  msg->header.frame_id = "frame2";
  filter.add(msg);

  EXPECT_EQ(1, n.failure_count_);
}
 */
/*

TEST(MessageFilter, emptyFrameIDFailure)
{
  std::shared_ptr<tf2_ros::TransformListener> tf_client;
  Notification n(1);
  MessageFilter<PCDType> filter(tf_client, "frame1", 1);
  filter.registerFailureCallback(std::bind(&Notification::failure, &n, std::placeholders::_1, std::placeholders::_2));

  PCDType::Ptr msg(new PCDType);
  msg->header.frame_id = "";
  filter.add(msg);

  EXPECT_EQ(1, n.failure_count_);
}
*/
/*
TEST(MessageFilter, removeCallback)
{
  // Callback queue in separate thread
  rclcpp::CallbackQueue cbqueue;
  rclcpp::AsyncSpinner spinner(1, &cbqueue);
  auto threaded_nh = std::make_shared<rclcpp::Node>("threaded_nh");
  threaded_nh.setCallbackQueue(&cbqueue);

  // TF filters; no data needs to be published
  std::unique_ptr<tf2_ros::TransformListener> tf_listener;
  std::unique_ptr<tf2::MessageFilter<PCDType> > tf_filter;

  spinner.start();
  for (int i = 0; i < 3; ++i) {
    tf_listener.reset(new tf2_ros::TransformListener());
    // Have callback fire at high rate to increase chances of race condition
    tf_filter.reset(
      new tf2::MessageFilter<PCDType>(*tf_listener,
                                     "map", 5, threaded_nh,
                                     rclcpp::Duration(0.000001)));

    // Sleep and reset; sleeping increases chances of race condition
    rclcpp::Duration(0.001).sleep();
    tf_filter.reset();
    tf_listener.reset();
  }
  spinner.stop();
}
*/
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<rclcpp::Node>("test_message_filter");

  int ret = RUN_ALL_TESTS();

  return ret;
}
