/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 *
 */

/*
 * Author: Paul Bovbel
 */

#include <pointcloud_to_laserscan/PointCloudToLaserScanBase.h>
#include <nodelet/nodelet.h>


namespace pointcloud_to_laserscan
{

    class PointCloudToLaserScanNodelet : public nodelet::Nodelet
    {
    public:
        PointCloudToLaserScanNodelet()  {};

        ~PointCloudToLaserScanNodelet() {}

    private:
        virtual void onInit()
        {
            bool use_concurrency;
            getPrivateNodeHandle().param<bool>("use_concurrency", use_concurrency, false);

            if(use_concurrency){
                cloud_to_scan.reset(new PointCloudToLaserScanBase(getMTNodeHandle(), getPrivateNodeHandle()));
            }else{
                cloud_to_scan.reset(new PointCloudToLaserScanBase(getNodeHandle(), getPrivateNodeHandle()));
            }

        };

        boost::shared_ptr<PointCloudToLaserScanBase> cloud_to_scan;
    };

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(pointcloud_to_laserscan, PointCloudToLaserScanNodelet, pointcloud_to_laserscan::PointCloudToLaserScanNodelet, nodelet::Nodelet);

