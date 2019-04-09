#pragma once

#include <cmath>
#include <sensor_msgs/LaserScan.h>

// PCL specific includes:
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "/home/adept/catkin_ws/src/MOTIONPLANNING/advanced_motion_planner/include/advanced_motion_planner/util.h"

class LidarToCloudConverter {

public:
    LidarToCloudConverter();
    LidarToCloudConverter(LidarToCloudConverter&) = delete;
    LidarToCloudConverter(LidarToCloudConverter&&) = delete;

    // this is the real constructor!
    pcl::PointCloud<pcl::PointXYZ> scanToCloud(const sensor_msgs::LaserScan &scan);

    inline const float GetMinDistance() const {return mDistanceRange.x;}
    inline const float GetMaxDistance() const {return mDistanceRange.y;}
    inline const float GetMinAngle() const {return mAngleRange.x;}
    inline const float GetMaxAngle() const {return mAngleRange.y;}

private:
    bool isInRange(float range, float angle); // this should depend on the velocity of the car
    inline void SetMinDistance(const float min) { mDistanceRange.x = min; }
    inline void SetMaxDistance(const float max) { mDistanceRange.y = max; }
    inline void SetMinAngle(const float min) { mAngleRange.x = min; }
    inline void SetMaxAngle(const float max) { mAngleRange.y = max; }

private:
    math::vec2 mDistanceRange;
    math::vec2 mAngleRange;
};
