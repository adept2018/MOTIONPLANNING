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

    /*
     * Calculates m_a and m_b for the equation
     * y = m_a * x + m_b
     */
    bool linearLeastSquares(const pcl::PointCloud<pcl::PointXYZ>& cloud);

    void calculatePID(const float& setpoint, const float& direction);

    // Linear least squares parameters
    float m_a;
    float m_b;

    // PID specific parameters
    float m_Kp;
    float m_Ki;
    float m_Kd;
    float m_dt;
    float m_min;
    float m_max;
    float m_previous_error;
    float m_integral;

    //
    uint16_t m_min_wall_line;
    uint16_t m_max_wall_line;

public:
    WallFollower(const float& K_p, const float& K_i, const float& K_d, const float& dt, const float& min, const float& max, const uint16_t& min_wall_line, const uint16_t& max_wall_line) {
        m_a{0.0f};
        m_b{0.0f};
        m_Kp{K_p};
        m_Ki{K_i};
        m_Kd{K_d};
        m_dt{dt};
        m_min{min};
        m_max{max};
        m_previous_error{0.0f};
        m_integral{0.0f};
        m_min_wall_line{min_wall_line};
        m_max_wall_line{max_wall_line};
    }

    bool followTheWall(const pcl::PointCloud<pcl::PointXYZ>& cloud);

    AckermannMessage ackMsg;
    Direction direction;

    pcl::PointCloud<pcl::PointXYZ> estimated_line;
};

#endif //WALL_FOLLOWER_H
