#ifndef LASERSCAN_TO_POINTCLOUD_H
#define LASERSCAN_TO_POINTCLOUD_H

#include <cmath>
#include <sensor_msgs/LaserScan.h>
// PCL specific includes:
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class LaserScanToPointCloud {
private:
    bool filter(float range, float angle);
    void resetStat();
    void updateStat(const float r, const float a, const pcl::PointXYZ &point);
    bool isStatInitialized();

public:
    struct {
      // .x is min and .y is max
      pcl::PointXY Rminmax;
      pcl::PointXY Aminmax;
      pcl::PointXY Xminmax;
      pcl::PointXY Yminmax;
    } stat;

    LaserScanToPointCloud() {}
    pcl::PointCloud<pcl::PointXYZ> scanToCloud(const sensor_msgs::LaserScan &scan, bool insideFilter);
};

#endif //LASERSCAN_TO_POINTCLOUD_H
