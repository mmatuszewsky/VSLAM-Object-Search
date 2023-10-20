#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>


void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    for (const auto& point : cloud->points) {
        std::cout << "x: " << point.x << ", y: " << point.y << ", z: " << point.z << std::endl;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_subscriber");

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/cloud_in", 1, cloudCallback);

    ros::spin();

    return 0;
}
