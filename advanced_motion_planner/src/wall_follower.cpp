#include <advanced_motion_planner/wall_follower.h>

bool WallFollower::followTheWall(const pcl::PointCloud<pcl::PointXYZ>& cloud) {

    if (cloud.size() == 0) {
        std::cerr << "WallFollower: size of input cloud is equal to 0" << std::endl;
        return false;
    }

    if (!linearLeastSquares(cloud)) {
        return false;
    }

    pcl::PointXYZ point;
    for (uint16_t i = min_wall_line; i < max_wall_line; ++i) {
        // point.x = i;
        // point.y = m_a * i + m_b;

        // Rotate points 90 degrees clockwise [(x,y) -> (y,-x)] since m_a and m_b calculated for points rotated 90 degrees counter-clockwise:
        point.x = m_a * i + m_b;
        point.y = -i;
        estimated_line.push_back(point);
    }

    // float x = (m_carrot_distance - m_b)/m_a - m_wall_distance;

    // Rotate points 90 degrees clockwise [(x,y) -> (y,-x)] since m_a and m_b calculated for points rotated 90 degrees counter-clockwise:
    float x = (-m_wall_distance - m_b)/m_a - m_carrot_distance;
    float y = m_a * x + m_b;

    float error_angle = atanf(y/(-m_carrot_distance));
    //float error_angle = 0.0f - atanf(m_a);
    direction = calculatePID(error_angle);

    return true;
}

bool WallFollower::linearLeastSquares(const pcl::PointCloud<pcl::PointXYZ>& cloud) {

    float xbar{0.0f};
    float ybar{0.0f};

    size = cloud.size()
    if (size == 0 || size == 1) {
        std::cerr << "linearLeastSquares: Input cloud size is 0 or 1" << std::endl;
        return false;
    }

    for (uint16_t i = 0; i < size; ++i) {
        // xbar += cloud[i].points.x;
        // ybar += cloud[i].points.y;

        // Rotate points 90 degrees counter-clockwise [(x,y) -> (-y, x)] since y = a*x + b cannot be used for vertical lines:
        xbar += -cloud[i].points.y;
        ybar += cloud[i].points.x;
    }

    xbar = xbar / static_cast<float>(size);
    ybar = ybar / static_cast<float>(size);

    float numerator{0.0f};
    float denominator{0.0f};

    for (uint16_t i = 0; i < size; ++i) {
        // numerator += (cloud[i].points.x - xbar) * (cloud[i].points.y - ybar);
        // denominator += pow((cloud[i].points.x - xbar), 2);

        // Rotate points 90 degrees counter-clockwise [(x,y) -> (-y, x)] since y = a*x + b cannot be used for vertical lines:
        numerator += (-cloud[i].points.y - xbar) * (cloud[i].points.x - ybar);
        denominator += pow((-cloud[i].points.y - xbar), 2);
    }

    m_a = numerator/denominator;
    m_b = ybar - m_a * xbar;

    return true;
}

float WallFollower::calculatePID(const float& error) {

    m_previous_error = error;

    float P = m_Kp * error;

    m_integral += error * m_dt;
    float I = m_Ki * m_integral;

    float D = m_Kd * (error - m_previous_error)/m_dt;

    float output = P + I + D;

    std::cout << "calculatePID output";
    if (output < m_min_direction) {
        output = m_min_direction;
        std::cout << " [m_min_direction]";
    }
    else if (output > m_max_direction) {
        output = m_max_direction;
        std::cout << " [m_max_direction]";
    }
    std::cout << ": " << output << std::endl;

    return output;
}
