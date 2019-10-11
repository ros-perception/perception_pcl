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
 * $Id: convex_hull.h 36116 2011-02-22 00:05:23Z rusu $
 *
 */

#ifndef PCL_ROS_CONVEX_HULL_2D_H_
#define PCL_ROS_CONVEX_HULL_2D_H_

#include "pcl_ros/pcl_node.h"
#include <geometry_msgs/msg/polygon_stamped.hpp>

// PCL includes
#include <pcl/surface/convex_hull.h>

namespace pcl_ros
{
  namespace sync_policies = message_filters::sync_policies;

  /** \brief @b ConvexHull2D represents a 2D ConvexHull implementation.
    * \author Radu Bogdan Rusu
    */
  class ConvexHull2D : public PCLNode
  {
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    typedef PointCloud::Ptr PointCloudPtr;
    typedef PointCloud::ConstPtr PointCloudConstPtr;
    
    public:
    ConvexHull2D (const rclcpp::NodeOptions& options);
    
    private:
      /** \brief Input point cloud callback.
        * \param cloud the pointer to the input point cloud
        * \param indices the pointer to the input point cloud indices
        */
      void input_indices_callback (const PointCloudPtr &cloud,
                                   const PointIndicesConstPtr &indices);
    
      // TODO: Fix
      void subscribe();
      void unsubscribe();
    
      /** \brief The PCL implementation used. */
      pcl::ConvexHull<pcl::PointXYZ> impl_;

      /** \brief The input PointCloud subscriber. */
      rclcpp::Subscription<PointCloud>::SharedPtr sub_input_;
    
      /** \brief Publisher for PolygonStamped. */
      rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub_plane_;
      /** \brief Publisher for PointCloud. */
      rclcpp::Publisher<PointCloud>::SharedPtr pub_output_;
    
      /** \brief Synchronized input, and indices.*/
      std::shared_ptr<message_filters::Synchronizer<sync_policies::ExactTime<PointCloud, PointIndices> > >       sync_input_indices_e_;
      std::shared_ptr<message_filters::Synchronizer<sync_policies::ApproximateTime<PointCloud, PointIndices> > > sync_input_indices_a_;

  public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#endif  //#ifndef PCL_ROS_CONVEX_HULL_2D_H_
