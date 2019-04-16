#ifndef WALL_FOLLOWER_H
#define WALL_FOLLOWER_H

#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <advanced_motion_planner/package_structs.h>

class WallFollower {
private:

public:
    bool followTheWall(pcl::PointCloud<pcl::PointXYZ> cloud);

};

#endif //WALL_FOLLOWER_H
