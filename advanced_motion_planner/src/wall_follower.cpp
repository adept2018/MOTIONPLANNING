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
      x      *
           *
             *
            *
             *
             *
           *

           quarter of  circle
*/

bool WallFollower::followTheWall(
    const pcl::PointCloud<pcl::PointXYZ>& cloud,
    const float& K_p,
    const float& K_i,
    const float& K_d) {

    if (cloud.size() == 0) {
        std::cerr << "WallFollower: size of input cloud is equal to 0" << std::endl;
        return false;
    }

    if (!linearLeastSquares(cloud)) {
        return false;
    }

    pcl::PointXYZ point;
    for (uint16_t i = min_wall_line; i < max_wall_line; ++i) {
        point.x = i;
        point.y = m_a * i + m_b;
        point.z = 0;
        estimated_line.push_back(point);
    }

    calculatePID(K_p, K_i, K_d);

    for (uint16_t i = min_observation_point; i < max_observation_point; ++i) {
        // least squares + PID
        // when turning, slow down
        // if cant turn, stop, wiggle backwards a bit, then continue algorithm
    }

    return true;
}

bool WallFollower::linearLeastSquares(const pcl::PointCloud<pcl::PointXYZ>& cloud) {

    float xbar{0.0f};
    float ybar{0.0f};

    size = cloud.size()
    if (size == 0) {
        std::cerr << "linearLeastSquares: Input cloud size is 0" << std::endl;
        return false;
    }
    else if (size == 1) {
        m_a = 0.0f;
        m_b = cloud[0].points.y;
        return true;
    }

    for (uint16_t i = 0; i < size; ++i) {
        xbar += cloud[i].points.x;
        ybar += cloud[i].points.y;
    }

    xbar = xbar / static_cast<float>(size);
    ybar = ybar / static_cast<float>(size);

    float numerator{0.0f};
    float denominator{0.0f};

    for (uint16_t i = 0; i < size; ++i) {
        numerator += (cloud[i].points.x - xbar) * (cloud[i].points.y - ybar);
        denominator += pow((cloud[i].points.x - xbar), 2);
    }

    m_a = numerator/denominator;
    m_b = ybar - m_a * xbar;

    return true;
}

void WallFollower::calculatePID(const float& setpoint, const float& direction) {

    float error = setpoint - direction;

    float P = m_Kp * error;

    m_integral += error * m_dt;
    float I = m_Ki * m_integral;

    float D = m_Kd * (error - m_previous_error)/m_dt;

    m_previous_error = error;
}
