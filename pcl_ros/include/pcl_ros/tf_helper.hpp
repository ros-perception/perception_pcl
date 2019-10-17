/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Open Source Robotics Foundation, Inc.
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
#ifndef PCL_ROS__TF_HELPER_HPP_
#define PCL_ROS__TF_HELPER_HPP_

// TODO(sloretz) remove when ros2/geometry2#180 merged

namespace pcl_ros
{
  inline tf2::TimePoint tfFromRclcpp(const rclcpp::Time & time)
  {
    // tf2::TimePoint is a typedef to a system time point, but rclcpp::Time may be ROS time.
    // Ignore that, and assume the clock used from rclcpp time points is consistent.
    return tf2::TimePoint(std::chrono::nanoseconds(time.nanoseconds()));
  }

  inline rclcpp::Time tfToRclcpp(const tf2::TimePoint & time)
  {
    // tf2::TimePoint is a typedef to a system time point, but rclcpp::Time may be ROS time.
    // Use whatever the default clock is.
    return rclcpp::Time(std::chrono::nanoseconds(time.time_since_epoch()).count());
  }

  inline tf2::Duration tfFromRclcpp(const rclcpp::Duration & duration)
  {
    return tf2::Duration(std::chrono::nanoseconds(duration.nanoseconds()));
  }

  inline rclcpp::Duration tfToRclcpp(const tf2::Duration & duration)
  {
    return rclcpp::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(duration));
  }
}  // namespace pcl_ros

#endif  // PCL_ROS__TF_HELPER_HPP_
