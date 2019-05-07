#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <ros/ros.h>
#include <cmath>

class Parameters {
private:
    ros::NodeHandle *nodeHandle;
    bool isNodeHandleSet;
    float pi = atanf(1.0f) * 4.0f;

public:
    float lidar_offset;
    float max_scan_range;
    float min_speed;
    float max_speed;
    float speed_incr;
    uint8_t follow_wall_x;   // left = 0x0, right = 0x1
    float min_wall_distance;
    float min_observation_percentage;
    float max_observation_percentage;

    Parameters() {
        isNodeHandleSet = false;
    }

    void setNodeHandle(ros::NodeHandle *nh) {
        nodeHandle = nh;
        isNodeHandleSet = true;
    }

    void update() {
        if (!isNodeHandleSet) {
            return;
        }

        nodeHandle->param<float>("/advanced_motion_planner/lidar_offset", lidar_offset, 0.0f);
        nodeHandle->param<float>("/advanced_motion_planner/max_scan_range", max_scan_range, 2.50f);
        nodeHandle->param<float>("/advanced_motion_planner/min_speed", min_speed, 0.30f);
        nodeHandle->param<float>("/advanced_motion_planner/max_speed", max_speed, 0.50f);
        nodeHandle->param<float>("/advanced_motion_planner/speed_incr", speed_incr, 0.05f);
        nodeHandle->param<uint8_t>("/advanced_motion_planner/follow_wall_x", follow_wall_x, 1);   // right wall by default
        nodeHandle->param<float>("/advanced_motion_planner/min_wall_distance", min_wall_distance, 0.10f);
        nodeHandle->param<float>("/advanced_motion_planner/min_observation_angle", min_observation_percentage, 0.25f);
        nodeHandle->param<float>("/advanced_motion_planner/man_observation_angle", max_observation_percentage, 0.75f);

        min_observation_percentage = min_observation_percentage < 0.0f : 0.0f ? min_observation_percentage;
        min_observation_percentage =  > 1.0f : 1.0f ? min_observation_percentage;
        max_observation_percentage = max_observation_percentage < 0.0f : 0.0f ? max_observation_percentage;
        max_observation_percentage = max_observation_percentage > 1.0f : 1.0f ? max_observation_percentage;

        if (min_observation_percentage >= max_observation_percentage) {
            std::cerr << "WARNING: min_observation_percentage (" << min_observation_percentage << ") is equal to or greater than max_observation_percentage (" << max_observation_percentage << ")" << std::endl;
            std::cerr << "Resetting min_observation_percentage and max_observation_percentage to standard values..." << std::endl;
            min_observation_percentage = 0.25f;
            max_observation_percentage = 0.75f;
        }
    }
};

#endif //PARAMETERS_H
