#ifndef MOTION_COMPUTER_H
#define MOTION_COMPUTER_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <cmath>
#include <queue>

#include <pcl/point_cloud.h>

#include <advanced_motion_planner/laserscan_to_pointcloud.h>
#include <advanced_motion_planner/package_structs.h>
#include <advanced_motion_planner/parameters.h>
#include <advanced_motion_planner/wall_follower.h>

class MotionComputer {
private:
    void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan);

    LaserScanToPointCloud laserScanToPointCloud;
    Parameters parameters;
    WallFollower wallFollower;

    bool m_acquired_scan;
    sensor_msgs::LaserScan m_laser_scan;
    std::queue<sensor_msgs::LaserScan> m_scan_queue;

    float m_dt;

    // Subscriber:
    ros::Subscriber m_subscriber;

public:
    MotionComputer(ros::NodeHandle& nh);
    bool computeMotion();

    AckermannMessage ackMsg;
    Direction direction;
    pcl::PointCloud<pcl::PointXYZ> map;
    pcl::PointCloud<pcl::PointXYZ> observed_points;
    pcl::PointCloud<pcl::PointXYZ> carrot;
    pcl::PointCloud<pcl::PointXYZ> wall_outline;
};

#endif //MOTION_COMPUTER_H
