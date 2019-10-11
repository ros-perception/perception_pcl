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
 * $Id: statistical_outlier_removal.h 35876 2011-02-09 01:04:36Z rusu $
 *
 */

#ifndef PCL_ROS_FILTERS_STATISTICALOUTLIERREMOVAL_H_
#define PCL_ROS_FILTERS_STATISTICALOUTLIERREMOVAL_H_

// PCL includes
#include <pcl/filters/statistical_outlier_removal.h>
#include "pcl_ros/filters/filter.h"
#include "pcl_ros/ptr_helper.h"
#include <mutex>

namespace pcl_ros
{
  /** \brief @b StatisticalOutlierRemoval uses point neighborhood statistics to filter outlier data. For more
    * information check:
    * <ul>
    * <li> R. B. Rusu, Z. C. Marton, N. Blodow, M. Dolha, and M. Beetz.
    *      Towards 3D Point Cloud Based Object Maps for Household Environments
    *      Robotics and Autonomous Systems Journal (Special Issue on Semantic Knowledge), 2008.
    * </ul>
    *
    * \note setFilterFieldName (), setFilterLimits (), and setFilterLimitNegative () are ignored.
    * \author Radu Bogdan Rusu
    */
  class StatisticalOutlierRemoval : public Filter
  {
    public:
      StatisticalOutlierRemoval(const rclcpp::NodeOptions& options) : Filter("StatisticalOutlierRemoval", options) {};
    
    protected:
      std::mutex mutex_;

      /** \brief Call the actual filter. 
        * \param input the input point cloud dataset
        * \param indices the input set of indices to use from \a input
        * \param output the resultant filtered dataset
        */
    
      inline void
      filter (const PointCloud2::ConstSharedPtr &input, const IndicesPtr &indices, 
              PointCloud2 &output)
      {
        std::lock_guard<std::mutex> lock (mutex_);
        pcl::PCLPointCloud2::Ptr pcl_input(new pcl::PCLPointCloud2);
        pcl_conversions::toPCL(*(input), *(pcl_input));
        impl_.setInputCloud (pcl_input);
        impl_.setIndices (to_boost_ptr(indices));
        pcl::PCLPointCloud2 pcl_output;
        impl_.filter (pcl_output);
        pcl_conversions::moveFromPCL(pcl_output, output);
      }


    private:
      /** \brief The PCL filter implementation used. */
      pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> impl_;
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#endif  //#ifndef PCL_FILTERS_STATISTICALOUTLIERREMOVAL_H_
