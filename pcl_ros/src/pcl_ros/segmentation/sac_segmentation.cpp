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
 * $Id: sac_segmentation.hpp 33195 2010-10-10 14:12:19Z marton $
 *
 */

#include <limits>

#include "pcl_ros/segmentation/sac_segmentation.hpp"
#include <pcl/common/io.h>
#include "pcl_ros/transforms.hpp"


//////////////////////////////////////////////////////////////////////////////////////////////
pcl_ros::SACSegmentation::SACSegmentation(const rclcpp::NodeOptions & options)
: PCLNode("SACSegmentationNode", options)
{
  rcl_interfaces::msg::ParameterDescriptor model_type_desc;
  model_type_desc.name = "model_type";
  model_type_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  model_type_desc.description =
    "The type of model to use for segmentation.";
  {
    rcl_interfaces::msg::IntegerRange int_range;
    int_range.from_value = 0;
    int_range.to_value = 17;
    model_type_desc.integer_range.push_back(int_range);
  }
  declare_parameter(
    model_type_desc.name, rclcpp::ParameterValue(
      pcl::SACMODEL_PLANE), model_type_desc);

  rcl_interfaces::msg::ParameterDescriptor distance_threshold_desc;
  distance_threshold_desc.name = "distance_threshold";
  distance_threshold_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  distance_threshold_desc.description =
    "The minimum distance (in meters) for a point to be considered an inlier.";
  {
    rcl_interfaces::msg::FloatingPointRange float_range;
    float_range.from_value = 0.0;
    float_range.to_value = 100000.0;
    distance_threshold_desc.floating_point_range.push_back(float_range);
  }
  declare_parameter(
    distance_threshold_desc.name, rclcpp::ParameterValue(0.01), distance_threshold_desc);

  rcl_interfaces::msg::ParameterDescriptor eps_angle_desc;
  eps_angle_desc.name = "eps_angle";
  eps_angle_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  eps_angle_desc.description =
    "The maximum allowed difference (radians) between the model normal and the given axis.";
  {
    rcl_interfaces::msg::FloatingPointRange float_range;
    float_range.from_value = 0.0;
    float_range.to_value = M_PI_2;
    eps_angle_desc.floating_point_range.push_back(float_range);
  }
  declare_parameter(
    eps_angle_desc.name, rclcpp::ParameterValue(0.0), eps_angle_desc);

  rcl_interfaces::msg::ParameterDescriptor method_type_desc;
  method_type_desc.name = "method_type";
  method_type_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  method_type_desc.description =
    "The type of method to use for segmentation.";
  {
    rcl_interfaces::msg::IntegerRange int_range;
    int_range.from_value = 0;
    int_range.to_value = 6;
    method_type_desc.integer_range.push_back(int_range);
  }
  declare_parameter(
    method_type_desc.name, rclcpp::ParameterValue(pcl::SAC_RANSAC),
    method_type_desc);

  rcl_interfaces::msg::ParameterDescriptor axis_desc;
  axis_desc.name = "axis";
  axis_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY;
  axis_desc.description =
    "The axis along which the method need to search for a model perpendicular to.";
  declare_parameter(
    axis_desc.name, rclcpp::ParameterValue(std::vector<double>({0.0, 0.0, 0.0})),
    axis_desc);

  rcl_interfaces::msg::ParameterDescriptor max_iterations_desc;
  max_iterations_desc.name = "max_iterations";
  max_iterations_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  max_iterations_desc.description =
    "The maximum number of iterations the sample consensus method will run.";
  declare_parameter(
    max_iterations_desc.name, rclcpp::ParameterValue(pcl::SAC_RANSAC),
    max_iterations_desc);

  rcl_interfaces::msg::ParameterDescriptor probability_desc;
  probability_desc.name = "probability";
  probability_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  probability_desc.description =
    "The desired probability of choosing at least one sample free from outliers.";
  {
    rcl_interfaces::msg::FloatingPointRange float_range;
    float_range.from_value = 0.0;
    float_range.to_value = 1.0;
    probability_desc.floating_point_range.push_back(float_range);
  }
  declare_parameter(
    probability_desc.name, rclcpp::ParameterValue(0.99), probability_desc);

  rcl_interfaces::msg::ParameterDescriptor optimize_coefficients_desc;
  optimize_coefficients_desc.name = "optimize_coefficients";
  optimize_coefficients_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  optimize_coefficients_desc.description =
    "Model coefficient refinement. true for enabling model coefficient refinement, false otherwise.";
  declare_parameter(
    optimize_coefficients_desc.name, rclcpp::ParameterValue(true), optimize_coefficients_desc);

  rcl_interfaces::msg::ParameterDescriptor radius_min_desc;
  radius_min_desc.name = "radius_min";
  radius_min_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  radius_min_desc.description =
    "The minimum allowable radius for the model (applicable to models that estimate a radius)";
  declare_parameter(
    radius_min_desc.name, rclcpp::ParameterValue(
      -std::numeric_limits<double>::max()), radius_min_desc);

  rcl_interfaces::msg::ParameterDescriptor radius_max_desc;
  radius_max_desc.name = "radius_max";
  radius_max_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  radius_max_desc.description =
    "The maximum allowable radius for the model (applicable to models that estimate a radius)";
  declare_parameter(
    radius_max_desc.name, rclcpp::ParameterValue(
      std::numeric_limits<double>::max()), radius_max_desc);

  std::vector<std::string> param_names {
    model_type_desc.name,
    distance_threshold_desc.name,
    eps_angle_desc.name,
    method_type_desc.name,
    axis_desc.name,
    max_iterations_desc.name,
    probability_desc.name,
    optimize_coefficients_desc.name,
    radius_min_desc.name,
    radius_max_desc.name
  };

  callback_handle_ =
    add_on_set_parameters_callback(
    std::bind(
      &SACSegmentation::config_callback, this,
      std::placeholders::_1));

  config_callback(get_parameters(param_names));

  createPublishers();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::SACSegmentation::computePublish(
  const PointCloud2::ConstSharedPtr & input,
  const IndicesPtr & indices)
{
  PointCloud2::UniquePtr cloud_tf(new PointCloud2(*input));     // set the output by default
  // Check whether the user has given a different output TF frame
  if (!tf_output_frame_.empty() && input->header.frame_id != tf_output_frame_) {
    RCLCPP_DEBUG(
      this->get_logger(), "Transforming input dataset from %s to %s.",
      input->header.frame_id.c_str(), tf_output_frame_.c_str());
    // Convert the cloud into the different frame
    PointCloud2 cloud_transformed;
    if (!pcl_ros::transformPointCloud(tf_output_frame_, *input, cloud_transformed, tf_buffer_)) {
      RCLCPP_ERROR(
        this->get_logger(), "Error converting input dataset from %s to %s.",
        input->header.frame_id.c_str(), tf_output_frame_.c_str());
      return;
    }
    cloud_tf.reset(new PointCloud2(cloud_transformed));
  }
  if (tf_output_frame_.empty() && input->header.frame_id != tf_input_orig_frame_) {
    // no tf_output_frame given, transform the dataset to its original frame
    RCLCPP_DEBUG(
      this->get_logger(), "Transforming input dataset from %s back to %s.",
      input->header.frame_id.c_str(), tf_input_orig_frame_.c_str());
    // Convert the cloud into the different frame
    PointCloud2 cloud_transformed;
    if (!pcl_ros::transformPointCloud(
        tf_input_orig_frame_, *input, cloud_transformed,
        tf_buffer_))
    {
      RCLCPP_ERROR(
        this->get_logger(), "Error converting input dataset from %s back to %s.",
        input->header.frame_id.c_str(), tf_input_orig_frame_.c_str());
      return;
    }
    cloud_tf.reset(new PointCloud2(cloud_transformed));
  }

  PointIndices output;
  ModelCoefficients model;
  output.header = model.header = input->header;
  // Call the virtual method in the child
  if (!input->data.empty()) {
    segment(input, indices, output, model);
  } else {
    RCLCPP_DEBUG(this->get_logger(), "Received empty input point cloud");
  }

  // Publish the unique ptr
  pub_output_->publish(move(output));
  pub_model_->publish(move(model));
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::SACSegmentation::subscribe()
{
  // If we're supposed to look for PointIndices (indices)
  if (use_indices_) {
    // Subscribe to the input using a filter
    auto sensor_qos_profile = rclcpp::SensorDataQoS().keep_last(max_queue_size_);
    sub_input_filter_.subscribe(this, "input", sensor_qos_profile);
    sub_indices_filter_.subscribe(this, "indices", sensor_qos_profile);

    if (approximate_sync_) {
      sync_input_indices_a_ =
        std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<PointCloud2,
          pcl_msgs::msg::PointIndices>>>(max_queue_size_);
      sync_input_indices_a_->connectInput(sub_input_filter_, sub_indices_filter_);
      sync_input_indices_a_->registerCallback(
        std::bind(
          &SACSegmentation::input_indices_callback, this,
          std::placeholders::_1, std::placeholders::_2));
    } else {
      sync_input_indices_e_ =
        std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ExactTime<PointCloud2,
          pcl_msgs::msg::PointIndices>>>(max_queue_size_);
      sync_input_indices_e_->connectInput(sub_input_filter_, sub_indices_filter_);
      sync_input_indices_e_->registerCallback(
        std::bind(
          &SACSegmentation::input_indices_callback, this,
          std::placeholders::_1, std::placeholders::_2));
    }
  } else {
    // Workaround for a callback with custom arguments ros2/rclcpp#766
    std::function<void(PointCloud2::ConstSharedPtr)> callback =
      std::bind(&SACSegmentation::input_indices_callback, this, std::placeholders::_1, nullptr);

    // Subscribe in an old fashion to input only (no filters)
    sub_input_ =
      this->create_subscription<PointCloud2>(
      "input", rclcpp::SensorDataQoS().keep_last(max_queue_size_),
      callback);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::SACSegmentation::unsubscribe()
{
  if (use_indices_) {
    sub_input_filter_.unsubscribe();
    sub_indices_filter_.unsubscribe();
  } else {
    sub_input_.reset();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::SACSegmentation::createPublishers()
{
  if (use_indices_) {
    if (!sub_input_filter_.getSubscriber() || !sub_indices_filter_.getSubscriber()) {
      subscribe();
    }
  } else {
    if (!sub_input_) {
      subscribe();
    }
  }
  pub_output_ = create_publisher<PointIndices>("output", max_queue_size_);
  pub_model_ = create_publisher<ModelCoefficients>("model", max_queue_size_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::SACSegmentation::use_frame_params()
{
  rcl_interfaces::msg::ParameterDescriptor input_frame_desc;
  input_frame_desc.name = "input_frame";
  input_frame_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  input_frame_desc.description =
    "The input TF frame the data should be transformed into before processing, "
    "if input.header.frame_id is different.";
  declare_parameter(input_frame_desc.name, rclcpp::ParameterValue(""), input_frame_desc);

  rcl_interfaces::msg::ParameterDescriptor output_frame_desc;
  output_frame_desc.name = "output_frame";
  output_frame_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  output_frame_desc.description =
    "The output TF frame the data should be transformed into after processing, "
    "if input.header.frame_id is different.";
  declare_parameter(output_frame_desc.name, rclcpp::ParameterValue(""), output_frame_desc);

  // Validate initial values using same callback
  callback_handle_ =
    add_on_set_parameters_callback(
    std::bind(
      &SACSegmentation::config_callback, this,
      std::placeholders::_1));

  std::vector<std::string> param_names{input_frame_desc.name, output_frame_desc.name};
  auto result = config_callback(get_parameters(param_names));
  if (!result.successful) {
    throw std::runtime_error(result.reason);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::SACSegmentation::segment(
  const PointCloud2::ConstSharedPtr & input, const IndicesPtr & indices,
  PointIndices & output, ModelCoefficients & model)
{
  std::lock_guard<std::mutex> lock(mutex_);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_input(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input, *pcl_input);
  impl_.setInputCloud(pcl_input);
  impl_.setIndices(indices);
  pcl::PointIndices::Ptr pcl_inliers(new pcl::PointIndices());
  pcl::ModelCoefficients::Ptr pcl_model(new pcl::ModelCoefficients);
  impl_.segment(*pcl_inliers, *pcl_model);
  pcl_conversions::moveFromPCL(*pcl_inliers, output);
  pcl_conversions::moveFromPCL(*pcl_model, model);
}

//////////////////////////////////////////////////////////////////////////////////////////////
rcl_interfaces::msg::SetParametersResult
pcl_ros::SACSegmentation::config_callback(const std::vector<rclcpp::Parameter> & params)
{
  std::lock_guard<std::mutex> lock(mutex_);

  for (const rclcpp::Parameter & param : params) {
    if (param.get_name() == "input_frame") {
      if (tf_input_frame_ != param.as_string()) {
        tf_input_frame_ = param.as_string();
        RCLCPP_DEBUG(
          get_logger(),
          "Setting the input frame to: %s.",
          tf_input_frame_.c_str());
      }
    }
    if (param.get_name() == "output_frame") {
      if (tf_output_frame_ != param.as_string()) {
        tf_output_frame_ = param.as_string();
        RCLCPP_DEBUG(
          get_logger(),
          "Setting the output frame to: %s.",
          tf_output_frame_.c_str());
      }
    }
    if (param.get_name() == "model_type") {
      int model_type = impl_.getModelType();
      if (model_type != param.as_int()) {
        RCLCPP_DEBUG(
          get_logger(),
          "Setting the model type to: %u.",
          model_type);
        impl_.setModelType(param.as_int());
      }
    }
    if (param.get_name() == "distance_threshold") {
      double distance_threshold = impl_.getDistanceThreshold();
      if (distance_threshold != param.as_double()) {
        distance_threshold = param.as_double();
        RCLCPP_DEBUG(
          get_logger(),
          "Setting the distance threshold to: %f.",
          distance_threshold);
        impl_.setDistanceThreshold(distance_threshold);
      }
    }
    if (param.get_name() == "eps_angle") {
      double eps_angle = impl_.getEpsAngle();
      if (eps_angle != param.as_double()) {
        eps_angle = param.as_double();
        RCLCPP_DEBUG(
          get_logger(),
          "Setting the eps angle to: %f.",
          eps_angle);
        impl_.setEpsAngle(eps_angle);
      }
    }
    if (param.get_name() == "method_type") {
      int method_type = impl_.getMethodType();
      if (method_type != param.as_int()) {
        RCLCPP_DEBUG(
          get_logger(),
          "Setting the method type to: %u.",
          method_type);
        impl_.setMethodType(param.as_int());
      }
    }
    if (param.get_name() == "axis") {
      std::vector<double> axis_param = param.as_double_array();
      Eigen::Vector3f axis(axis_param[0], axis_param[1], axis_param[2]);
      if (impl_.getAxis() != axis) {
        RCLCPP_DEBUG(
          get_logger(), "Setting the axis to: %f %f %f.",
          axis[0], axis[1], axis[2]);
        impl_.setAxis(axis);
      }
    }
    if (param.get_name() == "max_iterations") {
      int max_iterations = impl_.getMaxIterations();
      if (max_iterations != param.as_int()) {
        RCLCPP_DEBUG(
          get_logger(),
          "Setting the max iterations to: %u.",
          max_iterations);
        impl_.setMaxIterations(param.as_int());
      }
    }
    if (param.get_name() == "probability") {
      double probability = impl_.getProbability();
      if (probability != param.as_double()) {
        probability = param.as_double();
        RCLCPP_DEBUG(
          get_logger(),
          "Setting the probability to: %f.",
          probability);
        impl_.setProbability(probability);
      }
    }
    if (param.get_name() == "optimize_coefficients") {
      bool optimize_coefficients = param.as_bool();
      if (impl_.getOptimizeCoefficients() != optimize_coefficients) {
        RCLCPP_DEBUG(
          get_logger(),
          "Setting optimize coefficients to: %s.",
          (optimize_coefficients ? "true" : "false"));
        impl_.setOptimizeCoefficients(optimize_coefficients);
      }
    }
    if (param.get_name() == "radius_min") {
      double radius_min, radius_max;
      impl_.getRadiusLimits(radius_min, radius_max);
      if (radius_min != param.as_double()) {
        radius_min = param.as_double();
        RCLCPP_DEBUG(
          get_logger(),
          "Setting the minimum radius to: %f.",
          radius_min);
        impl_.setRadiusLimits(radius_min, radius_max);
      }
    }
    if (param.get_name() == "radius_max") {
      double radius_min, radius_max;
      impl_.getRadiusLimits(radius_min, radius_max);
      if (radius_max != param.as_double()) {
        radius_max = param.as_double();
        RCLCPP_DEBUG(
          get_logger(),
          "Setting the maximum radius to: %f.",
          radius_max);
        impl_.setRadiusLimits(radius_min, radius_max);
      }
    }
  }

  // Range constraints are enforced by rclcpp::Parameter.
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::SACSegmentation::input_indices_callback(
  const PointCloud2::ConstSharedPtr & cloud,
  const PointIndices::ConstSharedPtr & indices)
{
  // If cloud is given, check if it's valid
  if (!isValid(cloud)) {
    RCLCPP_ERROR(this->get_logger(), "Invalid input!");
    return;
  }
  // If indices are given, check if they are valid
  if (indices && !isValid(indices)) {
    RCLCPP_ERROR(this->get_logger(), "Invalid indices!");
    return;
  }

  /// DEBUG
  if (indices) {
    RCLCPP_DEBUG(
      this->get_logger(), "[input_indices_callback]\n"
      "  - PointCloud with %d data points (%s), stamp %d.%09d, and frame %s on topic %s received.\n"
      "  - PointIndices with %zu values, stamp %d.%09d, and frame %s on topic %s received.",
      cloud->width * cloud->height, pcl::getFieldsList(*cloud).c_str(),
      cloud->header.stamp.sec, cloud->header.stamp.nanosec, cloud->header.frame_id.c_str(), "input",
      indices->indices.size(), indices->header.stamp.sec, indices->header.stamp.nanosec,
      indices->header.frame_id.c_str(), "indices");
  } else {
    RCLCPP_DEBUG(
      this->get_logger(), "PointCloud with %d data points and frame %s on topic %s received.",
      cloud->width * cloud->height, cloud->header.frame_id.c_str(), "input");
  }
  ///

  // Check whether the user has given a different input TF frame
  tf_input_orig_frame_ = cloud->header.frame_id;
  PointCloud2::ConstSharedPtr cloud_tf;
  if (!tf_input_frame_.empty() && cloud->header.frame_id != tf_input_frame_) {
    RCLCPP_DEBUG(
      this->get_logger(), "Transforming input dataset from %s to %s.",
      cloud->header.frame_id.c_str(), tf_input_frame_.c_str());
    // Save the original frame ID
    // Convert the cloud into the different frame
    PointCloud2 cloud_transformed;
    if (!pcl_ros::transformPointCloud(tf_input_frame_, *cloud, cloud_transformed, tf_buffer_)) {
      RCLCPP_ERROR(
        this->get_logger(), "Error converting input dataset from %s to %s.",
        cloud->header.frame_id.c_str(), tf_input_frame_.c_str());
      return;
    }
    cloud_tf = std::make_shared<PointCloud2>(cloud_transformed);
  } else {
    cloud_tf = cloud;
  }

  // Need setInputCloud () here because we have to extract x/y/z
  IndicesPtr vindices;
  if (indices) {
    vindices.reset(new std::vector<int>(indices->indices));
  }

  computePublish(cloud_tf, vindices);
}

//////////////////////////////////////////////////////////////////////////////////////////////
// void
// pcl_ros::SACSegmentationFromNormals::onInit()
// {
//   // Call the super onInit ()
//   PCLNodelet::onInit();

//   // Enable the dynamic reconfigure service
//   srv_ = boost::make_shared<dynamic_reconfigure::Server<SACSegmentationFromNormalsConfig>>(*pnh_);
//   dynamic_reconfigure::Server<SACSegmentationFromNormalsConfig>::CallbackType f = boost::bind(
//     &SACSegmentationFromNormals::config_callback, this, _1, _2);
//   srv_->setCallback(f);

//   // Advertise the output topics
//   pub_indices_ = advertise<PointIndices>(*pnh_, "inliers", max_queue_size_);
//   pub_model_ = advertise<ModelCoefficients>(*pnh_, "model", max_queue_size_);

//   // ---[ Mandatory parameters
//   int model_type;
//   if (!pnh_->getParam("model_type", model_type)) {
//     NODELET_ERROR(
//       "[%s::onInit] Need a 'model_type' parameter to be set before continuing!",
//       getName().c_str());
//     return;
//   }
//   double threshold;  // unused - set via dynamic reconfigure in the callback
//   if (!pnh_->getParam("distance_threshold", threshold)) {
//     NODELET_ERROR(
//       "[%s::onInit] Need a 'distance_threshold' parameter to be set before continuing!",
//       getName().c_str());
//     return;
//   }

//   // ---[ Optional parameters
//   int method_type = 0;
//   pnh_->getParam("method_type", method_type);

//   XmlRpc::XmlRpcValue axis_param;
//   pnh_->getParam("axis", axis_param);
//   Eigen::Vector3f axis = Eigen::Vector3f::Zero();

//   switch (axis_param.getType()) {
//     case XmlRpc::XmlRpcValue::TypeArray:
//       {
//         if (axis_param.size() != 3) {
//           NODELET_ERROR(
//             "[%s::onInit] Parameter 'axis' given but with a different number of values (%d) than "
//             "required (3)!",
//             getName().c_str(), axis_param.size());
//           return;
//         }
//         for (int i = 0; i < 3; ++i) {
//           if (axis_param[i].getType() != XmlRpc::XmlRpcValue::TypeDouble) {
//             NODELET_ERROR(
//               "[%s::onInit] Need floating point values for 'axis' parameter.",
//               getName().c_str());
//             return;
//           }
//           double value = axis_param[i]; axis[i] = value;
//         }
//         break;
//       }
//     default:
//       {
//         break;
//       }
//   }

//   // Initialize the random number generator
//   srand(time(0));

//   NODELET_DEBUG(
//     "[%s::onInit] Nodelet successfully created with the following parameters:\n"
//     " - model_type               : %d\n"
//     " - method_type              : %d\n"
//     " - model_threshold          : %f\n"
//     " - axis                     : [%f, %f, %f]\n",
//     getName().c_str(), model_type, method_type, threshold,
//     axis[0], axis[1], axis[2]);

//   // Set given parameters here
//   impl_.setModelType(model_type);
//   impl_.setMethodType(method_type);
//   impl_.setAxis(axis);

//   onInitPostProcess();
// }

// //////////////////////////////////////////////////////////////////////////////////////////////
// void
// pcl_ros::SACSegmentationFromNormals::subscribe()
// {
//   // Subscribe to the input and normals using filters
//   sub_input_filter_.subscribe(*pnh_, "input", max_queue_size_);
//   sub_normals_filter_.subscribe(*pnh_, "normals", max_queue_size_);

//   // Subscribe to an axis direction along which the model search is to be constrained (the first
//   // 3 model coefficients will be checked)
//   sub_axis_ = pnh_->subscribe("axis", 1, &SACSegmentationFromNormals::axis_callback, this);

//   if (approximate_sync_) {
//     sync_input_normals_indices_a_ =
//       boost::make_shared<message_filters::Synchronizer<
//           sync_policies::ApproximateTime<PointCloud, PointCloudN, PointIndices>>>(max_queue_size_);
//   } else {
//     sync_input_normals_indices_e_ =
//       boost::make_shared<message_filters::Synchronizer<
//           sync_policies::ExactTime<PointCloud, PointCloudN, PointIndices>>>(max_queue_size_);
//   }

//   // If we're supposed to look for PointIndices (indices)
//   if (use_indices_) {
//     // Subscribe to the input using a filter
//     sub_indices_filter_.subscribe(*pnh_, "indices", max_queue_size_);

//     if (approximate_sync_) {
//       sync_input_normals_indices_a_->connectInput(
//         sub_input_filter_, sub_normals_filter_,
//         sub_indices_filter_);
//     } else {
//       sync_input_normals_indices_e_->connectInput(
//         sub_input_filter_, sub_normals_filter_,
//         sub_indices_filter_);
//     }
//   } else {
//     // Create a different callback for copying over the timestamp to fake indices
//     sub_input_filter_.registerCallback(bind(&SACSegmentationFromNormals::input_callback, this, _1));

//     if (approximate_sync_) {
//       sync_input_normals_indices_a_->connectInput(sub_input_filter_, sub_normals_filter_, nf_);
//     } else {
//       sync_input_normals_indices_e_->connectInput(sub_input_filter_, sub_normals_filter_, nf_);
//     }
//   }

//   if (approximate_sync_) {
//     sync_input_normals_indices_a_->registerCallback(
//       bind(
//         &SACSegmentationFromNormals::
//         input_normals_indices_callback, this, _1, _2, _3));
//   } else {
//     sync_input_normals_indices_e_->registerCallback(
//       bind(
//         &SACSegmentationFromNormals::
//         input_normals_indices_callback, this, _1, _2, _3));
//   }
// }

// //////////////////////////////////////////////////////////////////////////////////////////////
// void
// pcl_ros::SACSegmentationFromNormals::unsubscribe()
// {
//   sub_input_filter_.unsubscribe();
//   sub_normals_filter_.unsubscribe();

//   sub_axis_.shutdown();

//   if (use_indices_) {
//     sub_indices_filter_.unsubscribe();
//   }
// }

// //////////////////////////////////////////////////////////////////////////////////////////////
// void
// pcl_ros::SACSegmentationFromNormals::axis_callback(
//   const pcl_msgs::ModelCoefficientsConstPtr & model)
// {
//   boost::mutex::scoped_lock lock(mutex_);

//   if (model->values.size() < 3) {
//     NODELET_ERROR(
//       "[%s::axis_callback] Invalid axis direction / model coefficients with %zu values sent on %s!",
//       getName().c_str(), model->values.size(), pnh_->resolveName("axis").c_str());
//     return;
//   }
//   NODELET_DEBUG(
//     "[%s::axis_callback] Received axis direction: %f %f %f",
//     getName().c_str(), model->values[0], model->values[1], model->values[2]);

//   Eigen::Vector3f axis(model->values[0], model->values[1], model->values[2]);
//   impl_.setAxis(axis);
// }

// //////////////////////////////////////////////////////////////////////////////////////////////
// void
// pcl_ros::SACSegmentationFromNormals::config_callback(
//   SACSegmentationFromNormalsConfig & config,
//   uint32_t level)
// {
//   boost::mutex::scoped_lock lock(mutex_);

//   if (impl_.getDistanceThreshold() != config.distance_threshold) {
//     impl_.setDistanceThreshold(config.distance_threshold);
//     NODELET_DEBUG(
//       "[%s::config_callback] Setting distance to model threshold to: %f.",
//       getName().c_str(), config.distance_threshold);
//   }
//   // The maximum allowed difference between the model normal and the given axis _in radians_
//   if (impl_.getEpsAngle() != config.eps_angle) {
//     impl_.setEpsAngle(config.eps_angle);
//     NODELET_DEBUG(
//       "[%s::config_callback] Setting new epsilon angle to model threshold to: %f (%f degrees).",
//       getName().c_str(), config.eps_angle, config.eps_angle * 180.0 / M_PI);
//   }

//   if (impl_.getMaxIterations() != config.max_iterations) {
//     impl_.setMaxIterations(config.max_iterations);
//     NODELET_DEBUG(
//       "[%s::config_callback] Setting new maximum number of iterations to: %d.",
//       getName().c_str(), config.max_iterations);
//   }

//   // Number of inliers
//   if (min_inliers_ != config.min_inliers) {
//     min_inliers_ = config.min_inliers;
//     NODELET_DEBUG(
//       "[%s::config_callback] Setting new minimum number of inliers to: %d.",
//       getName().c_str(), min_inliers_);
//   }


//   if (impl_.getProbability() != config.probability) {
//     impl_.setProbability(config.probability);
//     NODELET_DEBUG(
//       "[%s::config_callback] Setting new probability to: %f.",
//       getName().c_str(), config.probability);
//   }

//   if (impl_.getOptimizeCoefficients() != config.optimize_coefficients) {
//     impl_.setOptimizeCoefficients(config.optimize_coefficients);
//     NODELET_DEBUG(
//       "[%s::config_callback] Setting coefficient optimization to: %s.",
//       getName().c_str(), (config.optimize_coefficients) ? "true" : "false");
//   }

//   if (impl_.getNormalDistanceWeight() != config.normal_distance_weight) {
//     impl_.setNormalDistanceWeight(config.normal_distance_weight);
//     NODELET_DEBUG(
//       "[%s::config_callback] Setting new distance weight to: %f.",
//       getName().c_str(), config.normal_distance_weight);
//   }

//   double radius_min, radius_max;
//   impl_.getRadiusLimits(radius_min, radius_max);
//   if (radius_min != config.radius_min) {
//     radius_min = config.radius_min;
//     NODELET_DEBUG(
//       "[%s::config_callback] Setting minimum allowable model radius to: %f.",
//       getName().c_str(), radius_min);
//     impl_.setRadiusLimits(radius_min, radius_max);
//   }
//   if (radius_max != config.radius_max) {
//     radius_max = config.radius_max;
//     NODELET_DEBUG(
//       "[%s::config_callback] Setting maximum allowable model radius to: %f.",
//       getName().c_str(), radius_max);
//     impl_.setRadiusLimits(radius_min, radius_max);
//   }
// }

// //////////////////////////////////////////////////////////////////////////////////////////////
// void
// pcl_ros::SACSegmentationFromNormals::input_normals_indices_callback(
//   const PointCloudConstPtr & cloud,
//   const PointCloudNConstPtr & cloud_normals,
//   const PointIndicesConstPtr & indices
// )
// {
//   boost::mutex::scoped_lock lock(mutex_);

//   PointIndices inliers;
//   ModelCoefficients model;
//   // Enforce that the TF frame and the timestamp are copied
//   inliers.header = model.header = fromPCL(cloud->header);

//   if (impl_.getModelType() < 0) {
//     NODELET_ERROR("[%s::input_normals_indices_callback] Model type not set!", getName().c_str());
//     pub_indices_.publish(boost::make_shared<const PointIndices>(inliers));
//     pub_model_.publish(boost::make_shared<const ModelCoefficients>(model));
//     return;
//   }

//   if (!isValid(cloud)) {  // || !isValid (cloud_normals, "normals"))
//     NODELET_ERROR("[%s::input_normals_indices_callback] Invalid input!", getName().c_str());
//     pub_indices_.publish(boost::make_shared<const PointIndices>(inliers));
//     pub_model_.publish(boost::make_shared<const ModelCoefficients>(model));
//     return;
//   }
//   // If indices are given, check if they are valid
//   if (indices && !isValid(indices)) {
//     NODELET_ERROR("[%s::input_normals_indices_callback] Invalid indices!", getName().c_str());
//     pub_indices_.publish(boost::make_shared<const PointIndices>(inliers));
//     pub_model_.publish(boost::make_shared<const ModelCoefficients>(model));
//     return;
//   }

//   /// DEBUG
//   if (indices && !indices->header.frame_id.empty()) {
//     NODELET_DEBUG(
//       "[%s::input_normals_indices_callback]\n"
//       "                                 - PointCloud with %d data points (%s), stamp %f, and "
//       "frame %s on topic %s received.\n"
//       "                                 - PointCloud with %d data points (%s), stamp %f, and "
//       "frame %s on topic %s received.\n"
//       "                                 - PointIndices with %zu values, stamp %f, and "
//       "frame %s on topic %s received.",
//       getName().c_str(),
//       cloud->width * cloud->height, pcl::getFieldsList(*cloud).c_str(), fromPCL(
//         cloud->header).stamp.toSec(), cloud->header.frame_id.c_str(), pnh_->resolveName(
//         "input").c_str(),
//       cloud_normals->width * cloud_normals->height, pcl::getFieldsList(
//         *cloud_normals).c_str(), fromPCL(
//         cloud_normals->header).stamp.toSec(),
//       cloud_normals->header.frame_id.c_str(), pnh_->resolveName("normals").c_str(),
//       indices->indices.size(), indices->header.stamp.toSec(),
//       indices->header.frame_id.c_str(), pnh_->resolveName("indices").c_str());
//   } else {
//     NODELET_DEBUG(
//       "[%s::input_normals_indices_callback]\n"
//       "                                 - PointCloud with %d data points (%s), stamp %f, and "
//       "frame %s on topic %s received.\n"
//       "                                 - PointCloud with %d data points (%s), stamp %f, and "
//       "frame %s on topic %s received.",
//       getName().c_str(),
//       cloud->width * cloud->height, pcl::getFieldsList(*cloud).c_str(), fromPCL(
//         cloud->header).stamp.toSec(), cloud->header.frame_id.c_str(), pnh_->resolveName(
//         "input").c_str(),
//       cloud_normals->width * cloud_normals->height, pcl::getFieldsList(
//         *cloud_normals).c_str(), fromPCL(
//         cloud_normals->header).stamp.toSec(),
//       cloud_normals->header.frame_id.c_str(), pnh_->resolveName("normals").c_str());
//   }
//   ///


//   // Extra checks for safety
//   int cloud_nr_points = cloud->width * cloud->height;
//   int cloud_normals_nr_points = cloud_normals->width * cloud_normals->height;
//   if (cloud_nr_points != cloud_normals_nr_points) {
//     NODELET_ERROR(
//       "[%s::input_normals_indices_callback] Number of points in the input dataset (%d) differs "
//       "from the number of points in the normals (%d)!",
//       getName().c_str(), cloud_nr_points, cloud_normals_nr_points);
//     pub_indices_.publish(boost::make_shared<const PointIndices>(inliers));
//     pub_model_.publish(boost::make_shared<const ModelCoefficients>(model));
//     return;
//   }

//   impl_.setInputCloud(pcl_ptr(cloud));
//   impl_.setInputNormals(pcl_ptr(cloud_normals));

//   IndicesPtr indices_ptr;
//   if (indices && !indices->header.frame_id.empty()) {
//     indices_ptr.reset(new std::vector<int>(indices->indices));
//   }

//   impl_.setIndices(indices_ptr);

//   // Final check if the data is empty
//   // (remember that indices are set to the size of the data -- if indices* = NULL)
//   if (!cloud->points.empty()) {
//     pcl::PointIndices pcl_inliers;
//     pcl::ModelCoefficients pcl_model;
//     pcl_conversions::moveToPCL(inliers, pcl_inliers);
//     pcl_conversions::moveToPCL(model, pcl_model);
//     impl_.segment(pcl_inliers, pcl_model);
//     pcl_conversions::moveFromPCL(pcl_inliers, inliers);
//     pcl_conversions::moveFromPCL(pcl_model, model);
//   }

//   // Check if we have enough inliers, clear inliers + model if not
//   if (static_cast<int>(inliers.indices.size()) <= min_inliers_) {
//     inliers.indices.clear();
//     model.values.clear();
//   }

//   // Publish
//   pub_indices_.publish(boost::make_shared<const PointIndices>(inliers));
//   pub_model_.publish(boost::make_shared<const ModelCoefficients>(model));
//   NODELET_DEBUG(
//     "[%s::input_normals_callback] Published PointIndices with %zu values on topic %s, and "
//     "ModelCoefficients with %zu values on topic %s",
//     getName().c_str(), inliers.indices.size(), pnh_->resolveName("inliers").c_str(),
//     model.values.size(), pnh_->resolveName("model").c_str());
//   if (inliers.indices.empty()) {
//     NODELET_WARN("[%s::input_indices_callback] No inliers found!", getName().c_str());
//   }
// }

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_ros::SACSegmentation)
