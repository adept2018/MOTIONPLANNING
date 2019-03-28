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

    for (int i = 0; i < scan.ranges.size(); ++i) {

        float r = scan.ranges[i];
        float theta = scan.angle_min + float(i) * scan.angle_increment + LIDAR_ANG_OFFSET;

        if (filter(r, theta) == insideFilter) {
          point.x = r * cos(theta);
          point.y = r * sin(theta);
          point.z = 0;

          cloud.push_back(point);
        }
    }

    return cloud;
}
