#include <advanced_motion_planner/laserscan_to_pointcloud.h>

pcl::PointCloud<pcl::PointXYZ> LaserScanToPointCloud::scanToCloud(
    const sensor_msgs::LaserScan& scan,
    const uint16_t& min_scan,
    const uint16_t& max_scan,
    const float& min_range,
    const float& max_range,
    const float& lidar_offset,
    const bool& is_map) {

    pcl::PointXYZ point;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    for (uint16_t i = min_scan; i < max_scan; ++i) {

        float r = scan.ranges[i];
        if (!is_map) {
            if (scan.ranges[i] < min_range) {
                r = min_range;
            }
            else if (scan.ranges[i] > max_range) {
                r = max_range;
            }
        }

        float theta = scan.angle_min + i * scan.angle_increment + lidar_offset;

        point.x = r * cosf(theta);
        point.y = r * sinf(theta);
        point.z = 0.0f;

        cloud.push_back(point);
    }

    return cloud;
}
