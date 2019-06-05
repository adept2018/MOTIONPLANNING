/** Advanced Motion Planner (AMP) source file
  * Originally created from Basic Motion Planner (BMP)

  * History:
  * 2019-03-20  Ported from BMP by Alexander Konovalenko
  * 2019-04-23  Successfully tested on the car. Lightning in the room
  *             can negatively affect the LIDAR!!!
  *
  **/

#include <advanced_motion_planner/laserscan_to_pointcloud.h>

inline bool LaserScanToPointCloud::filterGenerel(const float range, const float angle,
  const float rmin, const float rmax, const float amin, const float amax) {
  bool res;

  res = (range > rmax || range < rmin) ? false : ((angle < amin || angle > amax) ? false : true);

  return res;
}


bool LaserScanToPointCloud::filterVisible(const float range, const float angle) {
  return filterGenerel(range, angle, MIN_FRONT_Range, MAX_FRONT_Range, -angle_range, angle_range);
}

bool LaserScanToPointCloud::filterBackOff(const float range, const float angle) {
  return filterGenerel(range, angle, MIN_BACK_Range, MAX_BACK_Range, -angle_range_Back, angle_range_Back);
}


void LaserScanToPointCloud::scanToCloud(pcl::PointCloud<pcl::PointXYZ> &viscld, pcl::PointCloud<pcl::PointXYZ> &inviscld, const sensor_msgs::LaserScan &scan) {
  pcl::PointXYZ point;

  statVis.resetStat();
  statBackOff.resetStat();
  // we save CPU cycles here:
  statInvis.resetStat();

  for (int i = 0; i < scan.ranges.size(); ++i) {
    // this is visible cloud
    float r = scan.ranges[i];
    float theta = scan.angle_min + float(i) * scan.angle_increment + LIDAR_ANG_OFFSET;

    /*#ifdef DEBUG2
      std::cout << "scan.angle_min (deg)= " << RAD2DEG(scan.angle_min) << \
        "\t scan.angle_increment (deg) = " << RAD2DEG(scan.angle_increment) << \
        "\t scan.ranges.size() = " << scan.ranges.size() << std::endl;
    #endif */

    AMP_utils::polar2PointXYZ(point, r, theta);
    point.z = 0.0f;

    if (filterVisible(r, theta)) {
      // visible cloud

      // collect some statistics about the LIDAR points
      statVis.updateStat(r, theta, point);
      viscld.push_back(point);
    } else {
      // invisible cloud is here
      bool bckOff = filterBackOff(r, theta);
      if (bckOff) {
        // behind the car obstacle is found, only update statistics, no cloud needed
        statBackOff.updateStat(r, theta, point);
      }
      // we save CPU cycles here:
      //statInvis.updateStat(r, theta, point);
      inviscld.push_back(point);
    }
  }
}
