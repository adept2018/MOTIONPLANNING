#ifndef LASERSCAN_TO_POINTCLOUD_H
#define LASERSCAN_TO_POINTCLOUD_H

#include <cmath>
#include <sensor_msgs/LaserScan.h>

// PCL specific includes:
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "/home/adept/catkin_ws/src/MOTIONPLANNING/advanced_motion_planner/include/advanced_motion_planner/util.h"

class LaserScanToPointCloud {

public:
    LaserScanToPointCloud();

    pcl::PointCloud<pcl::PointXYZ> scanToCloud(const sensor_msgs::LaserScan &scan);

    inline const float GetMinDistance() const {return mDistanceRange.GetX();}
    inline const float GetMaxDistance() const {return mDistanceRange.GetY();}
    inline const float GetMinAngle() const {return mAngleRange.GetX();}
    inline const float GetMaxAngle() const {return mAngleRange.GetX();}

private:
    bool isInRange(float range, float angle); // this should depend on the velocity of the car
    inline void SetMinDistance(const float min) { mDistanceRange.SetX(min); }
    inline void SetMaxDistance(const float max) { mDistanceRange.SetY(max); }
    inline void SetMinAngle(const float min) { mAngleRange.SetX(min); }
    inline void SetMaxAngle(const float max) { mAngleRange.SetY(max); }

private:

  math::vec2 mDistanceRange;
  math::vec2 mAngleRange;
};

#endif //LASERSCAN_TO_POINTCLOUD_H
