#pragma once

#include <ros/ros.h>

#include <cmath>
#include <queue>
#include <sensor_msgs/LaserScan.h>
#include <advanced_motion_planner/LidarToCloudConverter.h>

struct Range {
    float AngleA = 0.0f;
    float DistanceA = 0.0f;
    float AngleB = 0.0f;
    float DistanceB = 0.0f;

    constexpr Range(float angleA, float distanceA, float angleB, float distanceB):
        AngleA(angleA), DistanceA(distanceA), AngleB(angleB), DistanceB(distanceB){}
}

class MotionComputer {

public:
    MotionComputer() = delete;
    MotionComputer(MotionComputer&) = delete;
    MotionComputer(MotionComputer&&) = delete;
    explicit MotionComputer(ros::NodeHandle& nh);

    bool computeMotion();
    inline math::vec2 getDirection() {return math::vec2(cosf(mTurnAngle), sinf(mTurnAngle));}
    inline float getTurnAngle() const {return mTurnAngle;}
    inline float getVelocityAmplitude() const {return mVelocityAmplitude;}
    inline const pcl::PointCloud<pcl::PointXYZ>& getCloud() const {return mCloud;}
private:
    void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan);

private:
    pcl::PointCloud<pcl::PointXYZ> mCloud;         // allocate a buffer for that?
    std::vector<Range> mRanges;
    pcl::PointCloud<pcl::PointXYZ> mCloud;         // allocate a buffer for that?
    std::queue<sensor_msgs::LaserScan> mScanQueue; //
    LidarToCloudConverter mLidarToCloudConverter;  //
    ros::Subscriber mScanSub;                      // Subscriber:

    //TODO: thread safety for every member
    float mTurnAngle;
    float mVelocityAmplitude;

};
