#include <pointcloud_to_laserscan/PointCloudToLaserScanBase.h>
#include <sensor_msgs/LaserScan.h>


namespace pointcloud_to_laserscan
{ 

PointCloudToLaserScanBase::PointCloudToLaserScanBase(ros::NodeHandle& nh, ros::NodeHandle& private_nh, const unsigned int concurrency) :
    nh_(nh), private_nh_(private_nh), concurrency_(concurrency)
{
    boost::mutex::scoped_lock lock(connect_mutex_);
    pub_ = nh.advertise<sensor_msgs::LaserScan>("scan", 10, boost::bind(&PointCloudToLaserScanBase::connectCb, this), boost::bind(&PointCloudToLaserScanBase::disconnectCb, this));

    //todo params
    target_frame_ = "base_link";

}

PointCloudToLaserScanBase::~PointCloudToLaserScanBase(){
    sub_.shutdown();
}

void PointCloudToLaserScanBase::cloudCb(const PointCloud::ConstPtr& cloud){

}


void PointCloudToLaserScanBase::connectCb() {
    boost::mutex::scoped_lock lock(connect_mutex_);
    if (!sub_ && pub_.getNumSubscribers() > 0) {
        ROS_DEBUG("Connecting to depth topic.");
        sub_ = nh_.subscribe("cloud_in", concurrency_, &PointCloudToLaserScanBase::cloudCb, this);
    }
}

void PointCloudToLaserScanBase::disconnectCb() {
    boost::mutex::scoped_lock lock(connect_mutex_);
    if (pub_.getNumSubscribers() == 0) {
        ROS_DEBUG("Unsubscribing from depth topic.");
        sub_.shutdown();
    }
}

} // pointcloud_to_laserscan
