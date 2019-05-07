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

public:
    WallFollower() {}
    bool followTheWall(pcl::PointCloud<pcl::PointXYZ> cloud);

    AckermannMessage ackMsg;
    Direction direction;
};

#endif //WALL_FOLLOWER_H
