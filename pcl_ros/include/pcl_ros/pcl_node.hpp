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
 * $Id: pcl_node.h 33238 2010-03-11 00:46:58Z rusu $
 *
 */

/**

\author Radu Bogdan Rusu
\author Antonio Brandi

**/

#ifndef PCL_ROS__PCL_NODE_HPP_
#define PCL_ROS__PCL_NODE_HPP_

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <memory>
#include <string>
#include <vector>
#include <tuple>
#include <utility>
#include <algorithm>

#include <message_filters/subscriber.hpp>
#include <message_filters/synchronizer.hpp>
#include <message_filters/sync_policies/exact_time.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_msgs/msg/point_indices.hpp>
#include <pcl_msgs/msg/model_coefficients.hpp>

#include "pcl_ros/transforms.hpp"

// #include "pcl_ros/point_cloud.hpp"

using pcl_conversions::fromPCL;

namespace pcl_ros
{
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudPtr = PointCloud::Ptr;
using PointCloudConstPtr = PointCloud::ConstPtr;

using Indices = pcl::PointIndices;
using IndicesPtr = Indices::Ptr;
using IndicesConstPtr = Indices::ConstPtr;

using Coefficients = pcl::ModelCoefficients;
using CoefficientsPtr = Coefficients::Ptr;
using CoefficientsConstPtr = Coefficients::ConstPtr;

using PointCloud2 = sensor_msgs::msg::PointCloud2;

using PointIndices = pcl_msgs::msg::PointIndices;
using PointIndicesPtr = PointIndices::SharedPtr;
using PointIndicesConstPtr = PointIndices::ConstSharedPtr;

using ModelCoefficients = pcl_msgs::msg::ModelCoefficients;
using ModelCoefficientsPtr = ModelCoefficients::SharedPtr;
using ModelCoefficientsConstPtr = ModelCoefficients::ConstSharedPtr;

/**
 * @brief Check whether a given PointCloud message is "valid" (i.e., has points, and width and height are non-zero).
 * @param cloud the point cloud to test.
 */
template<typename T>
inline bool isValid(const std::shared_ptr<const T> & cloud)
{
  return cloud->width * cloud->height * cloud->point_step == cloud->data.size();
}

/**
 * @brief Template specialization for PointIndices messages.
 */
template<>
inline bool isValid(const PointIndices::ConstSharedPtr &)
{
  return true;
}

/**
 * @brief Template specialization for ModelCoefficients messages.
 */
template<>
inline bool isValid(const ModelCoefficients::ConstSharedPtr &)
{
  return true;
}

template<typename ... Ts>
struct Input {};

template<typename ... Ts>
struct Output {};

template<typename InList, typename OutList>
class PCLNode;

/** \brief @b PCLNode represents the base PCL Node class. All PCL node should inherit from
 *  this class. */
template<typename ... In, typename ... Out>
class PCLNode<Input<In...>, Output<Out...>>: public rclcpp::Node
{
  static constexpr std::size_t NInputs = sizeof...(In);
  static constexpr std::size_t NOutputs = sizeof...(Out);

  using InputsTuple = std::tuple<In...>;
  using OutputsTuple = std::tuple<Out...>;

public:
  /** \brief Empty constructor. */
  PCLNode(
    std::string node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
    std::vector<std::string> input_topics = {},
    std::vector<std::string> output_topics = {})
  : rclcpp::Node(node_name, options),
    approximate_sync_(false),
    max_queue_size_(3),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_, this)
  {
    input_topics_ = make_topics_array<NInputs>(input_topics, "input");
    output_topics_ = make_topics_array<NOutputs>(output_topics, "output");

    // Common parameters to all PCL Nodes
    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.name = "max_queue_size";
      desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
      desc.description = "QoS History depth";
      desc.read_only = true;
      max_queue_size_ = declare_parameter(desc.name, max_queue_size_, desc);
    }

    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.name = "approximate_sync";
      desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
      desc.description =
        "Match indices and point cloud messages if time stamps are approximately the same.";
      desc.read_only = true;
      approximate_sync_ = declare_parameter(desc.name, approximate_sync_, desc);
    }

    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.name = "input_frame";
      desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
      desc.description =
        "The input TF frame the data should be transformed into before processing, "
        "if input.header.frame_id is different.";
      tf_input_frame_ = declare_parameter(
        desc.name, rclcpp::ParameterValue(""),
        desc).get<std::string>();
    }

    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.name = "output_frame";
      desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
      desc.description =
        "The output TF frame the data should be transformed into before processing, "
        "if output.header.frame_id is different.";
      tf_output_frame_ = declare_parameter(
        desc.name, rclcpp::ParameterValue(""),
        desc).get<std::string>();
    }

    // Register param callback handler
    callback_handle_ =
      add_on_set_parameters_callback(
      std::bind(
        &PCLNode::paramsCallback, this,
        std::placeholders::_1));

    initSubscribers();
    initPublishers(std::make_index_sequence<NOutputs>{});

    RCLCPP_DEBUG(
      get_logger(), "PCL Node successfully created with the following parameters:\n"
      " - approximate_sync          : %s\n"
      " - max_queue_size            : %d\n"
      " - input_frame               : %s\n"
      " - output_frame              : %s\n",
      (approximate_sync_) ? "true" : "false",
      max_queue_size_,
      tf_input_frame_.c_str(),
      tf_output_frame_.c_str());
  }

  /**
   * @brief Virtual destructor
   */
  virtual ~PCLNode() = default;

protected:
  /**
   * @brief Callback for parameter changes.
   * Implement this function in your PCL Node.
   * @param params the vector of parameters that have changed.
   */
  virtual rcl_interfaces::msg::SetParametersResult onParamsChanged(
    const std::vector<rclcpp::Parameter> & params) = 0;

  /**
   * @brief Virtual abstract compute method. To be implemented by every child.
   * @param inputs the requested inputs dataset.
   * @param output the the resultant filtered outputs.
   */
  virtual void compute(const In &... inputs, Out &... output) = 0;

private:
  /**
   * @brief Internal mutex.
   */
  std::mutex mutex_;

  /**
   * @brief Pointer to the callback handle for the parameters event.
   */
  OnSetParametersCallbackHandle::SharedPtr callback_handle_;

  /**
   * @brief True if we use an approximate time synchronizer
   * versus an exact one (false by default).
   */
  bool approximate_sync_;

  /**
   * @brief The maximum queue size (default: 3).
   */
  int max_queue_size_;

  /**
   * @brief The input TF frame the data should be transformed into,
   * if input.header.frame_id is different.
   */
  std::string tf_input_frame_;

  /**
   * @brief The original data input TF frame.
   */
  std::string tf_input_orig_frame_;

  /**
   * @brief The output TF frame the data should be transformed into,
   * if input.header.frame_id is different.
   */
  std::string tf_output_frame_;

  /**
   * @brief TF listener and Buffer objects.
   */
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::array<std::string, NInputs> input_topics_{};
  std::array<std::string, NOutputs> output_topics_{};

  typename rclcpp::Subscription<typename std::tuple_element<0,
    InputsTuple>::type>::SharedPtr sub_single_;

  std::tuple<message_filters::Subscriber<In>...> mf_subs_;
  std::shared_ptr<void> sync_handle_;

  std::tuple<typename rclcpp::Publisher<Out>::SharedPtr...> publishers_;

  /**
   * @brief Lazy transport subscribe/unsubscribe routine.
   * It is optional for backward compatibility.
   */
  void initSubscribers()
  {
    if constexpr (NInputs == 1) {
      // Subscribe in an old fashion to input only (no filters)
      using First = typename std::tuple_element<0, InputsTuple>::type;
      auto qos = rclcpp::SensorDataQoS().keep_last(max_queue_size_);

      sub_single_ = create_subscription<First>(
        input_topics_[0], qos,
        [this](const typename First::ConstSharedPtr msg) {
          // Forward to unified callback
          inputCallback(msg);
        });
    } else {
      // If multiple inputs are given, we synchronize them with message filters
      auto sensor_qos = rclcpp::SensorDataQoS().keep_last(max_queue_size_);

      // Initialize MF subscribers
      initMFSubscribers(sensor_qos, std::make_index_sequence<NInputs>{});

      // Build synchronizer over all MF subscribers
      if (approximate_sync_) {
        std::apply(
          [this](auto &... subs) {
            using Policy = message_filters::sync_policies::ApproximateTime<In...>;
            using Sync = message_filters::Synchronizer<Policy>;
            auto sync = std::make_shared<Sync>(Policy(max_queue_size_), subs ...);
            sync->registerCallback(&PCLNode::inputCallback, this);
            sync_handle_ = std::move(sync);
          }, mf_subs_);
      } else {
        std::apply(
          [this](auto &... subs) {
            using Policy = message_filters::sync_policies::ExactTime<In...>;
            using Sync = message_filters::Synchronizer<Policy>;
            auto sync = std::make_shared<Sync>(Policy(max_queue_size_), subs ...);
            sync->registerCallback(&PCLNode::inputCallback, this);
            sync_handle_ = std::move(sync);
          }, mf_subs_);
      }
    }
  }

  /**
   * @brief Unsubscribe from the input and indices topics.
   */
  void unsubscribe()
  {
    if constexpr (NInputs == 1) {
      sub_single_.reset();
    } else {
      // Disconnect synchronizers
      sync_handle_.reset();
      // Unsubscribe each message_filters subscriber
      unsubscribeMF(std::make_index_sequence<NInputs>{});
    }
  }

  /**
   * @brief The callback for the input data. This is where the action happens.
   * @param inputs the input messages.
   */
  void inputCallback(
    const typename In::ConstSharedPtr &... inputs)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!(isValid(inputs) && ...)) {
      RCLCPP_ERROR(get_logger(), "One or more invalid inputs received, skipping this callback...");
      return;
    }

    // Check whether the user has given a different input TF frame
    auto in_ptr_tuple = std::forward_as_tuple(inputs ...);

    InputsTuple transformed_inputs{};
    if (!transformMessages(
        std::make_index_sequence<NInputs>{}, transformed_inputs, in_ptr_tuple, tf_input_frame_))
    {
      // transformMessages already logged the reason
      return;
    }

    // Prepare output messages
    OutputsTuple outputs{};

    // Call derived compute with const refs to messages and refs to outputs
    std::apply(
      [&](const In &... t_in) {
        std::apply(
          [&](Out &... outs) {
            compute(t_in ..., outs ...);
          },
          outputs);
      },
      transformed_inputs);

    // Check whether the user has given a different input TF frame
    OutputsTuple transformed_outputs{};
    if (!transformMessages(std::make_index_sequence<NOutputs>{}, transformed_outputs, outputs,
        tf_output_frame_))
    {
      // transformMessages already logged the reason
      return;
    }

    // Publish all outputs
    publishOutputs(transformed_outputs, std::make_index_sequence<NOutputs>{});
  }

  /**
   * @brief Callback function for parameter updates.
   * @param params parameter values to set.
   */
  rcl_interfaces::msg::SetParametersResult paramsCallback(
    const std::vector<rclcpp::Parameter> & params)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const rclcpp::Parameter & param : params) {
      if (param.get_name() == "input_frame") {
        if (tf_input_frame_ != param.as_string()) {
          tf_input_frame_ = param.as_string();
          RCLCPP_DEBUG(get_logger(), "Setting the input frame to: %s.", tf_input_frame_.c_str());
        }
      }
      if (param.get_name() == "output_frame") {
        if (tf_output_frame_ != param.as_string()) {
          tf_output_frame_ = param.as_string();
          RCLCPP_DEBUG(get_logger(), "Setting the output frame to: %s.", tf_output_frame_.c_str());
        }
      }
    }
    return onParamsChanged(params);
  }

  /**
   * @brief Create an array of topic names, given a vector of names and a prefix.
   * If the vector is empty, the names will be auto-generated using the prefix.
   * If the vector has less names than required, the remaining names will be
   * auto-generated using the prefix, and a warning will be printed.
   * If the vector has more names than required, the extra names will be ignored.
   * @param v the vector of topic names.
   * @param prefix the prefix to use for auto-generated names.
   * @return an array of topic names of size N.
   */
  template<std::size_t N>
  static std::array<std::string, N> make_topics_array(
    const std::vector<std::string> & v, const std::string & prefix)
  {
    std::array<std::string, N> arr{};
    if (v.empty()) {
      for (std::size_t i = 0; i < N; ++i) {arr[i] = prefix + std::to_string(i);}
    } else {
      if (v.size() != N) {
        // best effort: fill provided names, auto-fill remaining, warn
        RCLCPP_WARN(
          rclcpp::get_logger("PCLNode"),
          "Expected %zu %s topics, got %zu. Filling defaults for the rest.",
          N, prefix.c_str(), v.size());
      }
      std::size_t i = 0;
      for (; i < std::min<std::size_t>(N, v.size()); ++i) {arr[i] = v[i];}
      for (; i < N; ++i) {arr[i] = prefix + std::to_string(i);}
    }
    return arr;
  }

  /**
   * @brief Initialize message_filters subscribers for each input type In[I].
   * @param qos the QoS profile to use for subscribing.
   * @param index_sequence a compile-time index sequence for the input types.
   */
  template<std::size_t... I>
  void initMFSubscribers(const rclcpp::QoS & qos, std::index_sequence<I...>)
  {
    // For each input type In[I], subscribe with message_filters
    ( ( std::get<I>(mf_subs_).subscribe(this, input_topics_[I], qos) ), ... );
  }

  /**
   * @brief Initialize publishers for each output type Out[J].
   * @param index_sequence a compile-time index sequence for the output types.
   */
  template<std::size_t... J>
  void initPublishers(std::index_sequence<J...>)
  {
    // For each output type Out[J], create a publisher
    ( ( std::get<J>(publishers_) =
    create_publisher<typename std::tuple_element<J, OutputsTuple>::type>(
      output_topics_[J], rclcpp::QoS(max_queue_size_)) ), ... );
  }

  /**
   * @brief Publish each output message to its corresponding publisher.
   * @param outputs the tuple of output messages to publish.
   * @param index_sequence a compile-time index sequence for the output types.
   */
  template<std::size_t... J>
  void publishOutputs(OutputsTuple & outputs, std::index_sequence<J...>)
  {
    // Publish each output message to its corresponding publisher
    ( ( std::get<J>(publishers_)->publish(std::get<J>(outputs)) ), ... );
  }

  /**
   * @brief Unsubscribe each message_filters subscriber.
   * @param index_sequence a compile-time index sequence for the input types.
   */
  template<std::size_t... I>
  void unsubscribeMF(std::index_sequence<I...>)
  {
    ( ( std::get<I>(mf_subs_).unsubscribe() ), ... );
  }

  template<typename T>
  struct is_const_shared_ptr : std::false_type {};

  template<typename T>
  struct is_const_shared_ptr<std::shared_ptr<const T>>: std::true_type {};

  template<typename T>
  static constexpr bool is_const_shared_ptr_v = is_const_shared_ptr<T>::value;

  template<typename DestTuple, typename SourceTuple, std::size_t... Is>
  bool transformMessages(
    std::index_sequence<Is...>,
    DestTuple & dest_tuple,
    const SourceTuple & src_tuple,
    const std::string & target_frame)
  {
    bool ok = true;

    // No target frame specified
    if (target_frame.empty()) {
      (void)std::initializer_list<int>{
        ( [&] {
          // Get the source element (either a smart ptr or a value)
          const auto & src_element = std::get<Is>(src_tuple);
          using SrcElementT = std::remove_cv_t<std::remove_reference_t<decltype(src_element)>>;

          if constexpr (is_const_shared_ptr_v<SrcElementT>) {
            if (!src_element) {
              RCLCPP_ERROR(
                get_logger(), "Null at index %zu, skipping copy.",
                static_cast<size_t>(Is));
              ok = false;
              return;
            }
            std::get<Is>(dest_tuple) = *src_element;
          } else {
            std::get<Is>(dest_tuple) = src_element;
          }
        }(),
        0 )...
      };
      return ok;
    }

    // Transform to target frame
    (void)std::initializer_list<int>{
      ( [&] {
        // Get the *destination* message type (e.g., PointCloud2)
        using DestMsgT = std::tuple_element_t<Is, DestTuple>;
        // Get the *source* element (smart ptr or value)
        const auto & src_element = std::get<Is>(src_tuple);
        using SrcElementT = std::remove_cv_t<std::remove_reference_t<decltype(src_element)>>;

        auto get_msg_ptr = [&]() -> const DestMsgT * {
          if constexpr (is_const_shared_ptr_v<SrcElementT>) {
            if (!src_element) {
              RCLCPP_ERROR(
                get_logger(), "Null at index %zu.",
                static_cast<size_t>(Is));
              ok = false;
              return nullptr;
            }
            return &(*src_element);
          } else {
            return &(static_cast<const DestMsgT &>(src_element));
          }
        };

        const DestMsgT * msg_in_ptr = get_msg_ptr();
        if (!msg_in_ptr) {
          return;
        }

        const DestMsgT & msg_in = *msg_in_ptr;
        auto & msg_out = std::get<Is>(dest_tuple);

        if constexpr (std::is_same_v<DestMsgT, PointCloud2>) {
          // This is a PointCloud2, transform it.
          if (msg_in.header.frame_id != target_frame) {
            if (!pcl_ros::transformPointCloud(target_frame, msg_in, msg_out, tf_buffer_)) {
              RCLCPP_ERROR(
                get_logger(),
                "TF transform failed for %zu: '%s' -> '%s'.",
                static_cast<size_t>(Is),
                msg_in.header.frame_id.c_str(), target_frame.c_str());
              ok = false;
              return;
            }
            // Preserve stamp, set new frame
            msg_out.header.stamp = msg_in.header.stamp;
            msg_out.header.frame_id = target_frame;
          } else {
            // Already in target frame, just copy.
            msg_out = msg_in;
          }
        } else {
          // Not a PointCloud2, just copy it through.
          msg_out = msg_in;
        }
      }(),
      0 )...
    };

    return ok;
  }
};
}  // namespace pcl_ros

#endif  // PCL_ROS__PCL_NODE_HPP_
