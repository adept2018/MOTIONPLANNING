#ifndef WALL_FOLLOWER_H
#define WALL_FOLLOWER_H

#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <advanced_motion_planner/package_structs.h>
#include <advanced_motion_planner/parameters.h>

class WallFollower {
private:
    Parameters parameters;
    void linearLeastSquares(const uint16_t& min, const uint16_t& max, const pcl::PointCloud<pcl::PointXYZ>& cloud);

public:
    WallFollower() {}
    bool followTheWall(pcl::PointCloud<pcl::PointXYZ> cloud);

    AckermannMessage ackMsg;
    Direction direction;

    float m_a;
    float m_b;
};

#endif //WALL_FOLLOWER_H
