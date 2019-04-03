#ifndef MOTION_COMPUTER_H
#define MOTION_COMPUTER_H

#include <ros/ros.h>

#include <cmath>
#include <queue>
#include <sensor_msgs/LaserScan.h>
#include <advanced_motion_planner/laserscan_to_pointcloud.h>

class MotionComputer {

public:
    MotionComputer(ros::NodeHandle& nh);
    bool computeMotion();
    inline math::vec2& getDirection() {return mDirection;}
    inline float getTurnAngle() {return mTurnAngle;}
    inline float getVelocityAmplitude() {return mVelocityAmplitude;}
    // inline void setDirection(math::vec2& dir);

private:
    void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan);

public:
    pcl::PointCloud<pcl::PointXYZ> mCloud;         // allocate a buffer for that

private:
    std::queue<sensor_msgs::LaserScan> mScanQueue; //
    LaserScanToPointCloud mLaserScanToPointCloud;  //
    ros::Subscriber mScanSub;                      // Subscriber:
//TODO: thread safety for every member
    math::vec2 mDirection;                         // OBS: race condition; kinda thread-safe; not simultaneous write on the members
    float mTurnAngle;
    float mVelocityAmplitude;
    bool mAcquiredScan;                            // maybe not a member

};

#endif //MOTION_COMPUTER_H
