#include <advanced_motion_planner/laserscan_to_pointcloud.h>

LaserScanToPointCloud::LaserScanToPointCloud(): mDistanceRange(0.3f, 2.0f), mAngleRange(-0.5f, 0.5f){

}


bool LaserScanToPointCloud::isInRange(float range, float angle) {

  if (range > GetMaxDistance() ||
      range < GetMinDistance()) {
      return false;
  }

  if (angle < GetMinAngle() ||
      angle > GetMaxAngle()) {
      return false;
  }

  return true;
}

pcl::PointCloud<pcl::PointXYZ> LaserScanToPointCloud::scanToCloud(const sensor_msgs::LaserScan &scan) {

    constexpr float pi = atan(1.0f) * 4.0f;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.reserve(1081); //check this thing out

    for (uint32_t i = 0; i < scan.ranges.size(); ++i) {

        const float r = scan.ranges[i];

        // Offset of the lidar is 90 degrees
        constexpr float offset = 0.5f * pi;
        const float theta = scan.angle_min + static_cast<float>(i) * scan.angle_increment + offset;

        if (isInRange(r, theta)) {
          // point.SetX(r * cos(theta));
          // point.SetY(r * sin(theta));
          pcl::PointXYZ point;
          point.x = r * cosf(theta);
          point.y = r * sinf(theta);
          point.z = theta;

          cloud.push_back(point);
        }
    }

    return cloud;
}
