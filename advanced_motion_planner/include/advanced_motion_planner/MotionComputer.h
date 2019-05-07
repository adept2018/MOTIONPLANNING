#pragma once

#include <ros/ros.h>
#include <cmath>
#include <queue>
#include <limits>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <advanced_motion_planner/LidarToCloudConverter.h>

struct Zone {

    Zone() = delete;
    Zone(const Zone&) = default;
    Zone(Zone&&) = default;

    constexpr explicit Zone(math::vec2 point):
        PointA(point), PointB(point){}
    constexpr explicit Zone(math::vec2 pointA, math::vec2 pointB):
        PointA(pointA), PointB(pointB){}

    math::vec2 PointA;
    math::vec2 PointB;
};

enum EnvironmentState {
    Free          = 0, //all space is free to drive
    PartiallyFree = 1, //some free zones
    Blocked       = 2, // surrounded by walls
    DeadEnd       = 3, //stop moving
};

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
    inline const pcl::PointCloud<pcl::PointXYZ>& getCloud() const {
        return mCloud;
    }
    inline bool isCloudEmpty() {return mCloud.empty();}
private:
    void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan);

private:
    // TODO: use unique ptrs
    pcl::PointCloud<pcl::PointXYZ> mCloud;         // allocate a buffer for that?
    std::vector<Zone> mZones;
    std::queue<sensor_msgs::LaserScan> mScanQueue; //
    LidarToCloudConverter mLidarToCloudConverter;  //
    ros::Subscriber mScanSub;                      // Subscriber:

    //TODO: thread safety for every member
    float mSafeRadius; // {m}
    float mTurnAngle;
    float mVelocityAmplitude; // {m/s}

};
