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


#ifndef PCL_ROS__POINT_CLOUD_HPP__
#define PCL_ROS__POINT_CLOUD_HPP__

#include <pcl/point_cloud.h>
#include <pcl/pcl_config.h>
#include <pcl/conversions.h>
#include <pcl/for_each_type.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/type_adapter.hpp>
#include <string>
#include <memory>
#include <type_traits>

namespace pcl
{
namespace detail
{

#if PCL_VERSION_COMPARE(>=, 1, 10, 0)
template<class T>
constexpr static bool pcl_uses_boost = std::is_same<boost::shared_ptr<T>,
    pcl::shared_ptr<T>>::value;
#else
template<class T>
constexpr static bool pcl_uses_boost = true;
#endif

template<class SharedPointer>
struct Holder
{
  SharedPointer p;

  explicit Holder(const SharedPointer & p) : p(p) {}
  Holder(const Holder & other) : p(other.p) {}
  Holder(Holder && other) : p(std::move(other.p)) {}

  void operator()(...) { p.reset(); }
};

template<class T>
inline std::shared_ptr<T> to_std_ptr(const boost::shared_ptr<T> & p)
{
  typedef Holder<std::shared_ptr<T>> H;
  if (H * h = boost::get_deleter<H>(p)) {
    return h->p;
  } else {
    return std::shared_ptr<T>(p.get(), Holder<boost::shared_ptr<T>>(p));
  }
}

template<class T>
inline boost::shared_ptr<T> to_boost_ptr(const std::shared_ptr<T> & p)
{
  typedef Holder<boost::shared_ptr<T>> H;
  if (H * h = std::get_deleter<H>(p)) {
    return h->p;
  } else {
    return boost::shared_ptr<T>(p.get(), Holder<std::shared_ptr<T>>(p));
  }
}

}  // namespace detail

// Pointer compatibility helpers

template<class T>
inline boost::shared_ptr<T> ros_ptr(const boost::shared_ptr<T> & p)
{
  return p;
}

template<class T>
inline boost::shared_ptr<T> ros_ptr(const std::shared_ptr<T> & p)
{
  return detail::to_boost_ptr(p);
}

template<class T, class = typename std::enable_if<!detail::pcl_uses_boost<T>>::type>
inline std::shared_ptr<T> pcl_ptr(const std::shared_ptr<T> & p)
{
  return p;
}

template<class T, class = typename std::enable_if<!detail::pcl_uses_boost<T>>::type>
inline std::shared_ptr<T> pcl_ptr(const boost::shared_ptr<T> & p)
{
  return detail::to_std_ptr(p);
}

template<class T, class = typename std::enable_if<detail::pcl_uses_boost<T>>::type>
inline boost::shared_ptr<T> pcl_ptr(const std::shared_ptr<T> & p)
{
  return detail::to_boost_ptr(p);
}

template<class T, class = typename std::enable_if<detail::pcl_uses_boost<T>>::type>
inline boost::shared_ptr<T> pcl_ptr(const boost::shared_ptr<T> & p)
{
  return p;
}

}  // namespace pcl

// ROS 2 TypeAdapter between pcl::PointCloud<T> and sensor_msgs::msg::PointCloud2

template<typename T>
struct rclcpp::TypeAdapter<pcl::PointCloud<T>, sensor_msgs::msg::PointCloud2>
{
  using is_specialized = std::true_type;
  using custom_type = pcl::PointCloud<T>;
  using ros_message_type = sensor_msgs::msg::PointCloud2;

  static void convert_to_ros_message(
    const custom_type & source,
    ros_message_type & destination)
  {
    pcl::toROSMsg(source, destination);
  }

  static void convert_to_custom(
    const ros_message_type & source,
    custom_type & destination)
  {
    pcl::fromROSMsg(source, destination);
  }
};

// Register TypeAdapter specializations
RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  pcl::PointCloud<pcl::PointXYZ>,
  sensor_msgs::msg::PointCloud2);

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  pcl::PointCloud<pcl::PointXYZRGB>,
  sensor_msgs::msg::PointCloud2);

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  pcl::PointCloud<pcl::Normal>,
  sensor_msgs::msg::PointCloud2);

#endif  // PCL_ROS__POINT_CLOUD_HPP__