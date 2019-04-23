#ifndef LASERSCAN_TO_POINTCLOUD_H
#define LASERSCAN_TO_POINTCLOUD_H

#include <cmath>
#include <sensor_msgs/LaserScan.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <advanced_motion_planner/parameters.h>

class LaserScanToPointCloud {
private:
    Parameters parameters;

public:
    LaserScanToPointCloud() {}
    pcl::PointCloud<pcl::PointXYZ> scanToCloud(const sensor_msgs::LaserScan scan);
};

#endif //LASERSCAN_TO_POINTCLOUD_H
