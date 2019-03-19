#ifndef MOTION_COMPUTER_H
#define MOTION_COMPUTER_H

#include <ros/ros.h>

#include <cmath>
#include <queue>
#include <sensor_msgs/LaserScan.h>
#include <advanced_motion_planner/laserscan_to_pointcloud.h>

struct Direction{
  float X;
  float Y;
  float Omega;
  void SetDirection(float x, float y, float omega){
    X = x;
    Y = y;
    Omega = omega;
  }
}
class MotionComputer {
private:
    void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan);

public:
    MotionComputer(ros::NodeHandle& nh);
    void computeMotion();
    Direction getDirection();
    void setDirection(Direction& dir);


private:
    std::queue<sensor_msgs::LaserScan> mScanQueue; //
    ros::Subscriber mScanSub;                      // Subscriber:
    bool mAcquiredScan;                            // maybe not a member
    LaserScanToPointCloud mLaserScanToPointCloud;  //
public:
    Direction mDirection;                          // OBS: race condition; kinda thread-safe; not simultaneous write on the members
    pcl::PointCloud<pcl::PointXYZ> mCloud;         // allocate a buffer for that
};

#endif //MOTION_COMPUTER_H
