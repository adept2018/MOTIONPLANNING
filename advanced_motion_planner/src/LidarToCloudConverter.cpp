#include "advanced_motion_planner/LidarToCloudConverter.h"

LidarToCloudConverter::LidarToCloudConverter():
    mDistanceRange(0.1f, 0.5f),
    mAngleRange(-0.350f, 0.350f){}

bool LidarToCloudConverter::isInRange(float range, float angle) {

    if (angle < mAngleRange.x ||
      angle > mAngleRange.y) {
      return false;
    }

    // if (range > mDistanceRange.y ||
    //     range < mDistanceRange.x) {
    //     return false;
    // }

    return true;
}

pcl::PointCloud<pcl::PointXYZ> LidarToCloudConverter::scanToCloud(const sensor_msgs::LaserScan& scan) {

    const unsigned int RangesSize = scan.ranges.size();

    // for debugging
    // std::cout << RangesSize << std::endl; // 360

    constexpr float PI = 3.141592;

    // Offset of the lidar is 90 degrees
    constexpr float offset = 0.5f * PI;

    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.reserve(RangesSize); //check this thing out

    pcl::PointXYZ point(0.0f, 0.0f, 0.0f);

    //TODO: start/end looping directly from min/max angle
    for (uint32_t i = 0; i < RangesSize; ++i) {

        // distance
        point.x = scan.ranges[i];

        // theta
        point.y = scan.angle_min + static_cast<float>(i) * scan.angle_increment + offset;

        // doesn't represent anything
        point.z = 0.0f;

        if (!isInRange(point.x, point.y)) {
          // point.x = std::numeric_limits<float>::infinity();
          cloud.push_back(point);
        }

    }

    return cloud;
}
