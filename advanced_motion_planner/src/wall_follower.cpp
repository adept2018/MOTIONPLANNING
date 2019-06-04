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

/*
 *
 */
bool WallFollower::followTheWall(const pcl::PointCloud<pcl::PointXYZ>& cloud, const uint8_t& wall_direction) {

    if (cloud.size() == 0) {
        std::cerr << "WallFollower: size of input cloud is equal to 0" << std::endl;
        return false;
    }

    linearLeastSquares(min_observation_point, max_observation_point, cloud);

    for (uint16_t i = min_observation_point; i < max_observation_point; ++i) {
        // least squares + PID
        // when turning, slow down
        // if cant turn, stop, wiggle backwards a bit, then continue algorithm
    }

    float theta = pi/2.0f;
    if (wall_direction == 0) {
        theta = -pi/2.0f;
    }



    return true;
}

/*
 * y = m_a * x + m_b
 */
void WallFollower::linearLeastSquares(const uint16_t& min, const uint16_t& max, const pcl::PointCloud<pcl::PointXYZ>& cloud) {

    float xbar{0.0f};
    float ybar{0.0f};

    if ((max - min) == 1) {
        m_a = 0.0f;
        m_b = cloud[max].points.y;
        return;
    }

    for (uint16_t i = min; i < max; ++i) {
        xbar += cloud[i].points.x;
        ybar += cloud[i].points.y;
    }

    xbar = xbar / static_cast<float>(max - min);
    ybar = ybar / static_cast<float>(max - min);

    float numerator{0.0f};
    float denominator{0.0f};

    for (uint16_t i = min; i < max; ++i) {
        numerator += (cloud[i].points.x - xbar) * (cloud[i].points.y - ybar);
        denominator += pow((cloud[i].points.x - xbar), 2);
    }

    m_a = numerator/denominator;
    m_b = ybar - m_a * xbar;

    // have to move this line to be parallel and in the same spot as the line x = position.min_wall_distance.
    // angle between two lines y = k1*x + m1 and y = k2*x + m2 given by:
    // tan(phi) = (k2 - k1)/(1 + k1*k2);
}
