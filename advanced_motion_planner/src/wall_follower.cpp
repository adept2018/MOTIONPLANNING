#include <advanced_motion_planner/wall_follower.h>

// Wall follower
/*
          *
        *
            *
              *
              *
              *
              *
            *
             *
            *
              *
           *
             *
            *
             *
             *
           *

           quarter of  circle
*/

bool WallFollower::followTheWall(pcl::PointCloud<pcl::PointXYZ> cloud) {

    if (cloud.size() == 0) {
        return false;
    }

    for (uint16_t i = 0; i < cloud.size(); ++i) {

    }



    return true;
}
