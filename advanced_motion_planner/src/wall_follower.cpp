#include <advanced_motion_planner/wall_follower.h>

bool WallFollower::followTheWall(const pcl::PointCloud<pcl::PointXYZ>& cloud, const float& dt) {

    if (cloud.size() == 0) {
        std::cerr << "WallFollower: size of input cloud is equal to 0" << std::endl;
        return false;
    }

    if (!linearLeastSquares(cloud)) {
        return false;
    }

    // float x = (m_carrot_distance - m_b)/m_a - m_wall_distance;

    // Rotate points 90 degrees clockwise [(x,y) -> (y,-x)] since m_a and m_b calculated for points rotated 90 degrees counter-clockwise:
    float x = (-m_wall_distance - m_b)/m_a - m_carrot_distance;
    float y = m_a * x + m_b;

    // float error_angle = atanf(y/(-m_carrot_distance));
    float error_angle = atan2f(y, -m_carrot_distance);
    //float error_angle = 0.0f - atanf(m_a);
    direction = calculatePID(error_angle, dt);


    calculated_carrot.clear();
    estimated_line.clear();

    pcl::PointXYZ point;
    for (float i = m_min_wall_line; i < m_max_wall_line; i += 0.1) {
        // point.x = i;
        // point.y = m_a * i + m_b;
        // point.z = 0.0f;

        // Rotate points 90 degrees clockwise [(x,y) -> (y,-x)] since m_a and m_b calculated for points rotated 90 degrees counter-clockwise:
        point.x = m_a * i + m_b;
        point.y = -i;
        point.z = 0.0f;
        estimated_line.push_back(point);
    }

    point.x = x;
    point.y = y;
    point.z = 0.0f;
    calculated_carrot.push_back(point);
    float pi = atanf(1.0f) * 4.0f;
    for (float i = -2*pi; i < 2*pi; i += 0.1) {
        point.x = x + m_carrot_radius * cosf(i);
        point.y = y + m_carrot_radius * sinf(i);
        point.z = 0.0f;
        calculated_carrot.push_back(point);
    }

    return true;
}

bool WallFollower::linearLeastSquares(const pcl::PointCloud<pcl::PointXYZ>& cloud) {

    float xbar{0.0f};
    float ybar{0.0f};

    uint16_t size = cloud.size();
    if (size == 0 || size == 1) {
        std::cerr << "linearLeastSquares: Input cloud size is 0 or 1" << std::endl;
        return false;
    }

    for (uint16_t i = 0; i < size; ++i) {
        // xbar += cloud[i].points.x;
        // ybar += cloud[i].points.y;

        // Rotate points 90 degrees counter-clockwise [(x,y) -> (-y, x)] since y = a*x + b cannot be used for vertical lines:
        xbar += -cloud.points[i].y;
        ybar += cloud.points[i].x;
    }

    xbar = xbar / static_cast<float>(size);
    ybar = ybar / static_cast<float>(size);

    float numerator{0.0f};
    float denominator{0.0f};

    for (uint16_t i = 0; i < size; ++i) {
        // numerator += (cloud[i].points.x - xbar) * (cloud[i].points.y - ybar);
        // denominator += pow((cloud[i].points.x - xbar), 2);

        // Rotate points 90 degrees counter-clockwise [(x,y) -> (-y, x)] since y = a*x + b cannot be used for vertical lines:
        numerator += (-cloud.points[i].y - xbar) * (cloud.points[i].x - ybar);
        denominator += pow((-cloud.points[i].y - xbar), 2);
    }

    m_a = numerator/denominator;
    m_b = ybar - m_a * xbar;

    return true;
}

float WallFollower::calculatePID(const float& error, const float& dt) {

    m_previous_error = error;

    float P = m_Kp * error;

    m_integral += error * dt;
    float I = m_Ki * m_integral;

    float D = m_Kd * (error - m_previous_error)/dt;

    float output = P + I + D;

    // std::cout << "calculatePID output";
    if (output < m_min_direction) {
        output = m_min_direction;
        // std::cout << " [m_min_direction]";
    }
    else if (output > m_max_direction) {
        output = m_max_direction;
        // std::cout << " [m_max_direction]";
    }
    // std::cout << ": " << output << std::endl;

    return output;
}
