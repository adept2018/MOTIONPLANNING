#ifndef MOTION_COMPUTER_H
#define MOTION_COMPUTER_H

#include <ros/ros.h>

#include <cmath>
#include <queue>
#include <vector>
#include <sensor_msgs/LaserScan.h>
#include <advanced_motion_planner/laserscan_to_pointcloud.h>
#include <sensor_msgs/Image.h>

class MotionComputer {
private:
    LaserScanToPointCloud laserScanToPointCloud;

    sensor_msgs::LaserScan laser_scan;
    bool acquired_scan;

    std::queue<sensor_msgs::LaserScan> scan_queue;
    std::queue<std::vector<float>> depth_queue;

    // Subscriber:
    ros::Subscriber scan_sub;
    void imageDepthCallback(const sensor_msgs::Image::ConstPtr& msg);
    void scanCallBack(const sensor_msgs::LaserScan::ConstPtr &scan);

public:
    MotionComputer(ros::NodeHandle &nh);
    bool computeMotion();
    std::vector<float> direction;
    pcl::PointCloud<pcl::PointXYZ> visibleCloud;
    pcl::PointCloud<pcl::PointXYZ> invisibleCloud;
};

#endif //MOTION_COMPUTER_H
