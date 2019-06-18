#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <ros/ros.h>
#include <cmath>

class Parameters {
private:
    ros::NodeHandle* nodeHandle;
    bool isNodeHandleSet;
    // float pi = atanf(1.0f) * 4.0f;

public:
    float lidar_offset;
    float min_scan_range;
    float max_scan_range;
    float min_speed;
    float max_speed;
    float speed_incr;
    uint8_t wall_direction;   // left = 0x0, right = 0x1
    float min_wall_distance;
    float min_observation;
    float max_observation;
    float wall_distance;
    float carrot_distance;
    float carrot_radius;
    int16_t min_wall_line;
    int16_t max_wall_line;
    float K_p;
    float K_i;
    float K_d;
    float min_direction;
    float max_direction;

    Parameters() {
        isNodeHandleSet = false;
    }

    void setNodeHandle(ros::NodeHandle* nh) {
        nodeHandle = nh;
        isNodeHandleSet = true;
    }

    void update() {
        if (!isNodeHandleSet) {
            return;
        }

        float pi = atanf(1.0f)*4.0f;

        int wall_direction_int;
        int min_wall_line_int;
        int max_wall_line_int;

        nodeHandle->param<float>("/advanced_motion_planner/lidar_offset", lidar_offset, 0.00f);
        nodeHandle->param<float>("/advanced_motion_planner/min_scan_range", min_scan_range, 0.05f);
        nodeHandle->param<float>("/advanced_motion_planner/max_scan_range", max_scan_range, 2.50f);
        nodeHandle->param<float>("/advanced_motion_planner/min_speed", min_speed, 0.30f);
        nodeHandle->param<float>("/advanced_motion_planner/max_speed", max_speed, 0.50f);
        nodeHandle->param<float>("/advanced_motion_planner/speed_incr", speed_incr, 0.05f);
        nodeHandle->param<int>("/advanced_motion_planner/wall_direction", wall_direction_int, 1);  // right wall by default
        nodeHandle->param<float>("/advanced_motion_planner/min_observation", min_observation, 0.25f);
        nodeHandle->param<float>("/advanced_motion_planner/max_observation", max_observation, 0.75f);
        nodeHandle->param<float>("/advanced_motion_planner/wall_distance", wall_distance, 0.10f);
        nodeHandle->param<float>("/advanced_motion_planner/carrot_distance", carrot_distance, 1.00f);
        nodeHandle->param<float>("/advanced_motion_planner/carrot_radius", carrot_radius, 0.10f);
        nodeHandle->param<int>("/advanced_motion_planner/min_wall_line", min_wall_line_int, -10);
        nodeHandle->param<int>("/advanced_motion_planner/max_wall_line", max_wall_line_int, 10);
        nodeHandle->param<float>("/advanced_motion_planner/K_p", K_p, 1.00f);
        nodeHandle->param<float>("/advanced_motion_planner/K_i", K_i, 0.00f);
        nodeHandle->param<float>("/advanced_motion_planner/K_d", K_d, 0.00f);
        nodeHandle->param<float>("/advanced_motion_planner/min_direction", min_direction, -pi/2.0f);
        nodeHandle->param<float>("/advanced_motion_planner/max_direction", max_direction, pi/2.0f);

        wall_direction = (uint8_t) wall_direction_int;
        min_wall_line = (int16_t) min_wall_line_int;
        max_wall_line = (int16_t) max_wall_line_int;

        min_observation = min_observation < 0.0f ? 0.0f : min_observation;
        min_observation = min_observation > 1.0f ? 1.0f : min_observation;
        max_observation = max_observation < 0.0f ? 0.0f : max_observation;
        max_observation = max_observation > 1.0f ? 1.0f : max_observation;

        if (min_observation >= max_observation) {
            std::cerr << "WARNING: min_observation (" << min_observation << ") is equal to or greater than max_observation (" << max_observation << ")" << std::endl;
            std::cerr << "Resetting min_observation and max_observation to standard values..." << std::endl;
            min_observation = 0.25f;
            max_observation = 0.75f;
        }

        if (wall_direction == 0) {   // follow left wall
            float min_observation_temp = min_observation;
            min_observation = 1 - max_observation;
            max_observation = 1 - min_observation_temp;
        }

        std::cout << "lidar_offset: " << lidar_offset << std::endl;
        std::cout << "min_scan_range: " << min_scan_range << std::endl;
        std::cout << "max_scan_range: " << max_scan_range << std::endl;
        std::cout << "min_speed: " << min_speed << std::endl;
        std::cout << "max_speed: " << max_speed << std::endl;
        std::cout << "speed_incr: " << speed_incr << std::endl;
        std::cout << "wall_direction: " << (int) wall_direction << std::endl;
        std::cout << "wall_distance: " << wall_distance << std::endl;
        std::cout << "min_observation: " << min_observation << std::endl;
        std::cout << "max_observation: " << max_observation << std::endl;
        std::cout << "carrot_distance: " << carrot_distance << std::endl;
        std::cout << "carrot_radius: " << carrot_radius << std::endl;
        std::cout << "min_wall_line: " << (int) min_wall_line << std::endl;
        std::cout << "max_wall_line: " << (int) max_wall_line << std::endl;
        std::cout << "K_p: " << K_p << std::endl;
        std::cout << "K_i: " << K_i << std::endl;
        std::cout << "K_d: " << K_d << std::endl;
        std::cout << "min_direction: " << min_direction << std::endl;
        std::cout << "max_direction: " << max_direction << std::endl;
    }
};

#endif //PARAMETERS_H
