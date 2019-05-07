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
        std::cerr << "WallFollower: size of input cloud is equal to 0" << std::endl;
        return false;
    }

    float min_observation point = cloud.size() * parameters.min_observation_percentage;
    float max_observation_point = cloud.size() * parameters.max_observation_percentage;

    for (uint16_t i = min_observation_point; i < max_observation_point; ++i) {




    }



    return true;
}
