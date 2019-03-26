#include <advanced_motion_planner/laserscan_to_pointcloud.h>

pcl::PointCloud<pcl::PointXYZ> LaserScanToPointCloud::scanToCloud(const sensor_msgs::LaserScan &scan) {
    pcl::PointXYZ point;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    for (int i = 0; i < scan.ranges.size(); ++i) {

        float r = scan.ranges[i];

        // Offset of the lidar is 90 degrees
        float pi = atan(1)*4;
        float offset = pi/2;
        float theta = scan.angle_min + i * scan.angle_increment + offset;

        point.x = r * cos(theta);
        point.y = r * sin(theta);
        point.z = 0;

        cloud.push_back(point);
        }
    }

    return cloud;
}
