#include <ros/ros.h>
#include <pointcloud_to_laserscan/PointCloudToLaserScanBase.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "pointcloud_to_laserscan");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    unsigned int concurrency = 4;

    pointcloud_to_laserscan::PointCloudToLaserScanBase node(nh, nh_private, concurrency);

    ros::spin();

    return 0;
}
