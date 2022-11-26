/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014-2016, JSK Lab, University of Tokyo.
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
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
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
 *********************************************************************/

#ifndef PCL_ROS__NODE_LAZY_HPP_
#define PCL_ROS__NODE_LAZY_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>

namespace pcl_ros
{

/** @brief
  * Enum to represent connection status.
  */
enum ConnectionStatus
{
  NOT_INITIALIZED,
  NOT_SUBSCRIBED,
  SUBSCRIBED
};

class NodeLazy : public rclcpp::Node
{
public:
  NodeLazy(std::string node_name, const rclcpp::NodeOptions & options)
  : rclcpp::Node(node_name, options)
  {
    connection_status_ = NOT_SUBSCRIBED;

    // option to use lazy transport
    declare_parameter("lazy", rclcpp::ParameterValue(true));
    lazy_ = get_parameter("lazy").as_bool();

    // option to use lazy transport
    // TODO(daisukes): work around until connection callback is implemented
    // https://github.com/ros2/rmw/issues/330
    declare_parameter("lazy_check_interval", rclcpp::ParameterValue(1.0));
    lazy_check_interval_ = get_parameter("lazy_check_interval").as_double();
    if (lazy_) {
      timer_lazy_check_ = create_wall_timer(
        std::chrono::duration<double>(lazy_check_interval_),
        std::bind(&NodeLazy::lazy_check_callback, this)
      );
    }

    // option for logging about being subscribed
    declare_parameter("verbose_connection", rclcpp::ParameterValue(false));
    verbose_connection_ = get_parameter("verbose_connection").as_bool();

    // timer to warn when no connection in the specified seconds
    ever_subscribed_ = false;
    declare_parameter("duration_to_warn_no_connection", rclcpp::ParameterValue(5.0));
    auto duration_to_warn_no_connection =
      get_parameter("duration_to_warn_no_connection").as_double();
    if (duration_to_warn_no_connection > 0) {
      timer_ever_subscribed_ = create_wall_timer(
        std::chrono::duration<double>(duration_to_warn_no_connection),
        std::bind(&NodeLazy::warn_never_subscribed_callback, this)
      );
    }

    RCLCPP_DEBUG(
      get_logger(),
      "NodeLazy:\n"
      "  - lazy: %s\n"
      "  - lazy_check_interval: %.2f\n"
      "  - verbose_connection: %s\n"
      "  - duration_to_warn_no_connection: %s",
      lazy_ ? "true" : "false", lazy_check_interval_, verbose_connection_ ? "true" : "false",
      duration_to_warn_no_connection ? "true" : "false"
    );
  }

protected:
  /** @brief
    * Post processing of construction of node.
    * You need to call this method in order to use always_subscribe
    * feature.
    */
  virtual void constructor_post_process()
  {
    RCLCPP_DEBUG(get_logger(), "post_construction");
    if (!lazy_) {
      std::lock_guard<std::mutex> lock(connection_mutex_);
      subscribe();
      ever_subscribed_ = true;
    }
  }

  /** @brief
    * callback function which is called when new subscriber come
    */
  virtual void lazy_check_callback()
  {
    if (verbose_connection_) {
      RCLCPP_INFO(get_logger(), "Checking connection status");
    }
    if (lazy_) {
      std::lock_guard<std::mutex> lock(connection_mutex_);
      for (size_t i = 0; i < publishers_.size(); i++) {
        rclcpp::PublisherBase::SharedPtr pub = publishers_[i];
        RCLCPP_DEBUG(
          get_logger(), "publisher[%ld] subscription count = %ld",
          i, pub->get_subscription_count());
        if (pub->get_subscription_count() > 0) {
          if (connection_status_ != SUBSCRIBED) {
            if (verbose_connection_) {
              RCLCPP_INFO(get_logger(), "Subscribe input topics");
            }
            subscribe();
            connection_status_ = SUBSCRIBED;
          }
          if (!ever_subscribed_) {
            ever_subscribed_ = true;
          }
          return;
        }
      }
      if (connection_status_ == SUBSCRIBED) {
        if (verbose_connection_) {
          RCLCPP_INFO(get_logger(), "Unsubscribe input topics");
        }
        unsubscribe();
        connection_status_ = NOT_SUBSCRIBED;
      }
    }
  }

  /** @brief
    * callback function which is called when walltimer
    * duration run out.
    */
  virtual void warn_never_subscribed_callback()
  {
    if (!ever_subscribed_) {
      RCLCPP_WARN(get_logger(), "This node/nodelet subscribes topics only when subscribed.");
    }
    timer_ever_subscribed_->cancel();
  }


  /** @brief
    * This method is called when publisher is subscribed by other
    * nodes.
    * Set up subscribers in this method.
    */
  virtual void subscribe() = 0;

  /** @brief
    * This method is called when publisher is unsubscribed by other
    * nodes.
    * Shut down subscribers in this method.
    */
  virtual void unsubscribe() = 0;

  /** @brief
    * Update the list of Publishers created by this method.
    * It automatically reads latch boolean parameter from nh and
    * publish topic with appropriate latch parameter.
    *
    * @param topic topic name to advertise.
    * @param qos QoS for publisher.
    * @return Publisher for the advertised topic.
    */
  template<typename MessageT,
    typename PublisherT = rclcpp::Publisher<MessageT>>
  std::shared_ptr<PublisherT>
  advertise(std::string topic, rclcpp::QoS qos)
  {
    std::lock_guard<std::mutex> lock(connection_mutex_);
    std::shared_ptr<PublisherT> pub = create_publisher<MessageT>(topic, qos);
    publishers_.push_back(pub);
    return pub;
  }

  /** @brief
    * mutex to call subscribe() and unsubscribe() in
    * critical section.
    */
  std::mutex connection_mutex_;

  /** @brief
    * List of watching publishers
    */
  std::vector<rclcpp::PublisherBase::SharedPtr> publishers_;

  /** @brief
    * WallTimer instance for lazy check
    */
  rclcpp::TimerBase::SharedPtr timer_lazy_check_;

  /** @brief
    * WallTimer instance for warning about no connection.
    */
  rclcpp::TimerBase::SharedPtr timer_ever_subscribed_;

  /** @brief
    * A flag to check if the node has been ever subscribed
    * or not.
    */
  bool ever_subscribed_;

  /** @brief
    * A flag to disable watching mechanism and always subscribe input
    * topics. It can be specified via `~lazy` parameter.
    */
  bool lazy_;

  /** @brief
    * An interval to check publish/subscribe status (workaround)
    */
  double lazy_check_interval_;

  /** @brief
    * Status of connection
    */
  ConnectionStatus connection_status_;

  /** @brief
    * true if `~verbose_connection` or `verbose_connection` parameter is true.
    */
  bool verbose_connection_;

private:
};

}  // namespace pcl_ros


#endif  // PCL_ROS__NODE_LAZY_HPP_
