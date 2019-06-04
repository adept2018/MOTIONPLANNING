#ifndef MOTION_COMPUTER_H
#define MOTION_COMPUTER_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <cmath>
#include <queue>

#include <advanced_motion_planner/laserscan_to_pointcloud.h>
#include <advanced_motion_planner/package_structs.h>
#include <advanced_motion_planner/wall_follower.h>

class MotionComputer {
private:
    void scanCallBack(const sensor_msgs::LaserScan::ConstPtr &scan);

    LaserScanToPointCloud laserScanToPointCloud;
    WallFollower wallFollower;

    bool m_acquired_scan;
    sensor_msgs::LaserScan m_laser_scan;
    std::queue<sensor_msgs::LaserScan> m_scan_queue;

    // Subscriber:
    ros::Subscriber m_subscriber;

protected:
    Parameters parameters;

public:
    MotionComputer(ros::NodeHandle &nh);
    bool computeMotion();

    AckermannMsg ackMsg;
    Direction direction;
    pcl::PointCloud<pcl::PointXYZ> map;
    pcl::PointCloud<pcl::PointXYZ> observed_points;
    pcl::PointCloud<pcl::PointXYZ> wall_outline;
};

#endif //MOTION_COMPUTER_H
