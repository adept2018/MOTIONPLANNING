#ifndef MOTION_COMPUTER_H
#define MOTION_COMPUTER_H

#include <ros/ros.h>

#include <cmath>
#include <queue>
#include <sensor_msgs/LaserScan.h>
#include <advanced_motion_planner/laserscan_to_pointcloud.h>

// Offset to turn away from obstacle
const float turn_offset = 0.52;
// safe agnle where car can go through within 2m range
const float safe_angle = 0.24;

class MotionComputer {
private:
    LaserScanToPointCloud laserScanToPointCloud;

    sensor_msgs::LaserScan laser_scan;
    bool acquired_scan;

    std::queue<sensor_msgs::LaserScan> scan_queue;

    // Subscriber:
    ros::Subscriber scan_sub;

    void scanCallBack(const sensor_msgs::LaserScan::ConstPtr &scan);

public:
    MotionComputer(ros::NodeHandle &nh);
    bool computeMotion();
    std::vector<float> direction;
    pcl::PointCloud<pcl::PointXYZ> visibleCloud;
    pcl::PointCloud<pcl::PointXYZ> invisibleCloud;
};

#endif //MOTION_COMPUTER_H
