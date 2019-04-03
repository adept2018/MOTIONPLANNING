#ifndef MOTION_COMPUTER_H
#define MOTION_COMPUTER_H

#include <ros/ros.h>

#include <cmath>
#include <queue>
#include <sensor_msgs/LaserScan.h>
#include <advanced_motion_planner/laserscan_to_pointcloud.h>

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
    std::vector<float> direction;
    pcl::PointCloud<pcl::PointXYZ> visibleCloud;
    pcl::PointCloud<pcl::PointXYZ> invisibleCloud;

    MotionComputer(ros::NodeHandle &nh);
    bool computeMotion();
    // implemented methods for determination of motion direction:
    float getWeightedAverageDirection(const int NoOfPoints);
    float getLargestRectangularDirection(const int NoOfPoints);
    void buildRectangle(pcl::PointXY &A, pcl::PointXY &B, pcl::PointXY &C, pcl::PointXY &D,
      const float a, const float r);
    bool areAnyPointsInsideRectangle(const int n, const float r_i, const float a_i);
};

#endif //MOTION_COMPUTER_H
