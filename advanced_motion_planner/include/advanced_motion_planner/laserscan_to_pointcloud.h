#ifndef LASERSCAN_TO_POINTCLOUD_H
#define LASERSCAN_TO_POINTCLOUD_H
/** Advanced Motion Planner (AMP) source file
  * Originally created from Basic Motion Planner (BMP)

  * History:
  * 2019-03-20  Changed from BMP by Alexander Konovalenko
  * 2019-04-23  Successfully tested on the car. Lightning in the room
  *             can negatively affect the LIDAR!!!
  * 2019-06-19  Demo has been performed on 190618. This is final commit to AMP.
  *            Things left for further improvements:
  *         1) extend decision making in motion_computer.cpp lines# 128-132,
  *          as Chrais pointed out, ratio r_best/w_best can be used with proper
  *          range (e.g. 0.8...1.2) check as additional condition on selection
  *          of the best path in lines# 148-170 of motion_computer.cpp
  *         2) Alan's implementation of reading parameters from a file and
  *          their real-time update shall be used instead of many macro defines
  *          in amp_common.h
  *         3) Implemented backward motion shall be further debugged and improved
  *          (it is disabled right now in amp_common.h).
  *         4) Positive noise (appearing ghost points) filtering is disabled
  *          (macro MAX_ALLOWED_POINTS) because of bigger problem with missing
  *          points (negative noise).
  *         5) BETTER LIDAR IS WANTED!!! 
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
