#ifndef LASERSCAN_TO_POINTCLOUD_H
#define LASERSCAN_TO_POINTCLOUD_H

#include <cmath>
#include <sensor_msgs/LaserScan.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class LaserScanToPointCloud {
private:

public:
    LaserScanToPointCloud() {}
    pcl::PointCloud<pcl::PointXYZ> scanToCloud(
        const sensor_msgs::LaserScan& scan,
        const uint16_t& min_scan,
        const uint16_t& max_scan,
        const float& min_range,
        const float& max_range,
        const float& lidar_offset,
        const bool& is_map);
};

#endif //LASERSCAN_TO_POINTCLOUD_H
