#include <advanced_motion_planner/laserscan_to_pointcloud.h>

pcl::PointCloud<pcl::PointXYZ> LaserScanToPointCloud::scanToCloud(const sensor_msgs::LaserScan scan) {

    pcl::PointXYZ point;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    for (uint16_t i = 0; i < scan.ranges.size(); ++i) {

        float r = scan.ranges[i];
        float theta = scan.angle_min + i * scan.angle_increment + parameters.lidar_offset;

        point.x = r * cosf(theta);
        point.y = r * sinf(theta);
        point.z = 0.0f;

        cloud.push_back(point);
    }

    return cloud;
}
