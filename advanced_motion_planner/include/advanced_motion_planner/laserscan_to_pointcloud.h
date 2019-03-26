#ifndef LASERSCAN_TO_POINTCLOUD_H
#define LASERSCAN_TO_POINTCLOUD_H

#include <cmath>
#include <sensor_msgs/LaserScan.h>

// PCL specific includes:
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class LaserScanToPointCloud {

public:
    LaserScanToPointCloud() {}
    pcl::PointCloud<pcl::PointXYZ> scanToCloud(const sensor_msgs::LaserScan &scan, bool insideFilter);

    inline const float GetMaxDistance() const;
    inline const float GetMinDistance() const;
    inline const float GetAngle() const;

private:
    bool filter(float range, float angle);

private:

  pcl::PointXY mDistancerange;
  pcl::PointXY mAngleDistance;
};

#endif //LASERSCAN_TO_POINTCLOUD_H
