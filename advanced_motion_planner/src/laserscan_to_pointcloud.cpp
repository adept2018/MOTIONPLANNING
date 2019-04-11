#include <advanced_motion_planner/laserscan_to_pointcloud.h>
#include <advanced_motion_planner/amp_common.h>

bool LaserScanToPointCloud::filter(float range, float angle) {
  bool res;

  res = (range > max_range || range < min_range) ? false : ((angle < -angle_range || angle > angle_range) ? false : true);

  return res;
}

pcl::PointCloud<pcl::PointXYZ> LaserScanToPointCloud::scanToCloud(const sensor_msgs::LaserScan &scan, bool insideFilter) {
    pcl::PointXYZ point;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    resetStat();

    for (int i = 0; i < scan.ranges.size(); ++i) {

        float r = scan.ranges[i];
        float theta = scan.angle_min + float(i) * scan.angle_increment + LIDAR_ANG_OFFSET;

        if (filter(r, theta) == insideFilter) {
          // visible cloud
          AMP_utils::polar2PointXYZ(point, r, theta);
          point.z = 0.0f;

          // collect some statistics about the LIDAR points
          updateStat(r, theta, point);

          cloud.push_back(point);
        }
    }

    return cloud;
}

inline void LaserScanToPointCloud::updateStat(const float r, const float a, const pcl::PointXYZ &point) {
  if (stat.Rminmax.x >= r) stat.Rminmax.x = r; //min r
  if (stat.Rminmax.y < r) stat.Rminmax.y = r; // max r
  if (stat.Aminmax.x >= a) stat.Aminmax.x = a; //min angle
  if (stat.Aminmax.y < a) stat.Aminmax.y = a; // max angle
  if (stat.Xminmax.x >= point.x) stat.Xminmax.x = point.x; //min x
  if (stat.Xminmax.y < point.x) stat.Xminmax.y = point.x; // max x
  if (stat.Yminmax.x >= point.y) stat.Yminmax.x = point.y; //min y
  if (stat.Yminmax.y < point.y) stat.Yminmax.y = point.y; // max y
}

inline void LaserScanToPointCloud::resetStat() {
  /*
  // default values
  stat.Rminmax.x = 100000.0f;       // impossible value for min
  stat.Rminmax.y = -1.0f;           // impossible value for max
  stat.Aminmax.x = 2.0*PI;          // impossible value for min
  stat.Aminmax.y = -stat.Aminmax.x; // impossible value for max
  stat.Xminmax.x = 100000.0f;       // impossible value for min
  stat.Xminmax.y = -stat.Xminmax.x; // impossible value for max
  stat.Yminmax.x = stat.Xminmax.x;  // impossible value for min
  stat.Yminmax.y = stat.Xminmax.y;  // impossible value for max */

  // the following default values make more sense, based on filtering of visibleCloud
  stat.Rminmax.x = max_range;       // impossible value for min
  stat.Rminmax.y = min_range;           // impossible value for max
  stat.Aminmax.x = angle_range;          // impossible value for min
  stat.Aminmax.y = -stat.Aminmax.x; // impossible value for max
  stat.Xminmax.x = max_range;       // impossible value for min
  stat.Xminmax.y = -stat.Xminmax.x; // impossible value for max
  stat.Yminmax.x = stat.Xminmax.x;  // impossible value for min
  stat.Yminmax.y = -stat.Yminmax.x;  // impossible value for max
}

// check if all ranges are simultaneously processed (i.e. min <= mmax)
inline bool LaserScanToPointCloud::isStatInitialized() {
  bool res = false;
  if(stat.Rminmax.x <= stat.Rminmax.y) {
    if(stat.Aminmax.x <= stat.Aminmax.y) {
      if(stat.Xminmax.x <= stat.Xminmax.y) {
        if(stat.Yminmax.x <= stat.Yminmax.y) {
          res = true;
        }
      }
    }
  }
  return res;
}
