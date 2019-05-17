#ifndef LASERSCAN_TO_POINTCLOUD_H
#define LASERSCAN_TO_POINTCLOUD_H
/** Advanced Motion Planner (AMP) source file
  * Originally created from Basic Motion Planner (BMP)

  * History:
  * 2019-03-20  Changed from BMP by Alexander Konovalenko
  * 2019-04-23  Successfully tested on the car. Lightning in the room
  *             can negatively affect the LIDAR!!!
  *
  **/

#include <cmath>
#include <advanced_motion_planner/amp_common.h>
#include <sensor_msgs/LaserScan.h>


class LaserScanToPointCloud {
private:
    bool filterGenerel(const float range, const float angle, const float rmin, const float rmax, const float amin, const float amax);
    bool filterVisible(const float range, const float angle);
    bool filterBackOff(const float range, const float angle);

public:
    AMP_stat statVis, statInvis, statBackOff;

    LaserScanToPointCloud() {}
    void scanToCloud(pcl::PointCloud<pcl::PointXYZ> &viscld, pcl::PointCloud<pcl::PointXYZ> &inviscld, const sensor_msgs::LaserScan &scan);
};

#endif //LASERSCAN_TO_POINTCLOUD_H
