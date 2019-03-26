#ifndef LASERSCAN_TO_POINTCLOUD_H
#define LASERSCAN_TO_POINTCLOUD_H

#include <cmath>
#include <sensor_msgs/LaserScan.h>

// PCL specific includes:
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class LaserScanToPointCloud {
private:
<<<<<<< Updated upstream
    bool filter(float range, float angle);
    
=======

>>>>>>> Stashed changes
public:
    LaserScanToPointCloud() {
    }

<<<<<<< Updated upstream
    pcl::PointCloud<pcl::PointXYZ> scanToCloud(const sensor_msgs::LaserScan &scan, bool insideFilter);
=======
    pcl::PointCloud<pcl::PointXYZ> scanToCloud(const sensor_msgs::LaserScan &scan);
>>>>>>> Stashed changes
};

#endif //LASERSCAN_TO_POINTCLOUD_H
